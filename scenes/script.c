// SPDX-License-Identifier: MIT
// Replay scripts (see script.h): commands read from text, written back, and submitted as their ticks come

#include "script.h"

#include "scenes.h"

#include <stdlib.h>
#include <string.h>

static const char* s_tools[lp_toolCount] = { "rifle", "grenade", "cannon", "hammer", "ball", "flask" };

static bool SameWord( const char* a, const char* b )
{
	for ( ; *a != 0 && *b != 0; ++a, ++b )
	{
		char x = *a >= 'A' && *a <= 'Z' ? (char)( *a - 'A' + 'a' ) : *a;
		char y = *b >= 'A' && *b <= 'Z' ? (char)( *b - 'A' + 'a' ) : *b;
		if ( x != y )
		{
			return false;
		}
	}
	return *a == *b;
}

// The sandbox's normalisation, kept as it was so recorded sessions replay exactly (lpNormalize differs for tiny vectors)
static lpVec3 Normalize( lpVec3 a )
{
	float l = sqrtf( a.x * a.x + a.y * a.y + a.z * a.z );
	if ( l > 0.0f )
	{
		float s = 1.0f / l;
		return ( lpVec3 ){ s * a.x, s * a.y, s * a.z };
	}
	return a;
}

lpCommand lpScriptToolCommand( int tool, lpVec3 origin, lpVec3 dir )
{
	lpCommand c = { 0 };
	lpVec3 d = Normalize( dir );
	if ( tool == lp_toolFlask || tool == lp_toolBall )
	{
		// Thrown from just ahead of the aim: the flask lobbed and spinning, the ball fired at 45 m/s
		bool flask = tool == lp_toolFlask;
		c.kind = lp_commandSpawn;
		c.spawn.templateIndex = flask ? lp_templateFlask : lp_templateBall;
		c.spawn.transform = lpTransform_identity;
		c.spawn.transform.p = lpMulAdd( origin, flask ? 0.8f : 1.0f, d );
		c.spawn.linearVelocity = flask ? lpAdd( lpMulSV( 16.0f, d ), ( lpVec3 ){ 0.0f, 2.5f, 0.0f } ) : lpMulSV( 45.0f, d );
		c.spawn.angularVelocity = flask ? ( lpVec3 ){ 4.0f, 1.0f, 7.0f } : lpVec3_zero;
		return c;
	}
	c.kind = lp_commandImpact;
	c.impact.origin = origin;
	c.impact.range = tool == lp_toolHammer ? 4.0f : 250.0f;
	c.impact.def.direction = d;
	switch ( tool )
	{
		case lp_toolRifle:
			c.impact.def.radius = 0.35f;
			c.impact.def.energy = 4000.0f;
			c.impact.def.impulse = 20.0f;
			break;
		case lp_toolGrenade:
			c.impact.def.radius = 1.4f;
			c.impact.def.energy = 80000.0f;
			c.impact.def.impulse = 12.0f;
			c.impact.def.explosion = true;
			break;
		case lp_toolCannon:
			c.impact.def.radius = 2.3f;
			c.impact.def.energy = 350000.0f;
			c.impact.def.impulse = 18.0f;
			c.impact.def.explosion = true;
			break;
		default: // the hammer
			c.impact.def.radius = 0.6f;
			c.impact.def.energy = 14000.0f;
			c.impact.def.impulse = 60.0f;
			break;
	}
	return c;
}

static void Push( lpScript* script, lpCommand* c, long long tick, int peer )
{
	if ( script->count == script->capacity )
	{
		int capacity = script->capacity > 0 ? 2 * script->capacity : 64;
		lpCommand* commands = (lpCommand*)realloc( script->commands, (size_t)capacity * sizeof( lpCommand ) );
		if ( commands == NULL )
		{
			return;
		}
		script->commands = commands;
		script->capacity = capacity;
	}
	c->tick = tick;
	c->peer = (uint8_t)peer;
	c->seq = script->seq[peer]++;
	script->commands[script->count++] = *c;
}

bool lpScriptParseLine( lpScript* script, const char* line )
{
	// The stamp: a tick, or tick:peer
	char* end;
	long long t = strtoll( line, &end, 10 );
	if ( end == line )
	{
		return true; // a comment, a blank line
	}
	int peer = 0;
	if ( *end == ':' )
	{
		char* after;
		long p = strtol( end + 1, &after, 10 );
		if ( after == end + 1 || p < 0 || p >= LP_PEER_SCENE )
		{
			fprintf( stderr, "script: bad peer in '%s'\n", line );
			return false;
		}
		peer = (int)p;
		end = after;
	}
	const char* rest = end;
	char name[32] = { 0 };
	if ( sscanf( rest, "%31s", name ) != 1 )
	{
		return true;
	}

	lpCommand c = { 0 };
	int a = 0, b = 0, m = 0;
	lpVec3 o = { 0 }, d = { 0 };
	for ( int tool = 0; tool < lp_toolCount; ++tool )
	{
		if ( SameWord( name, s_tools[tool] ) )
		{
			if ( sscanf( rest, "%31s %f %f %f %f %f %f", name, &o.x, &o.y, &o.z, &d.x, &d.y, &d.z ) == 7 )
			{
				c = lpScriptToolCommand( tool, o, d );
				Push( script, &c, t, peer );
			}
			return true;
		}
	}
	if ( SameWord( name, "impact" ) )
	{
		c.kind = lp_commandImpact;
		c.impact.range = 250.0f;
		lpImpactDef* def = &c.impact.def;
		if ( sscanf( rest, "%31s %f %f %f %f %f %f %f %f %f", name, &o.x, &o.y, &o.z, &d.x, &d.y, &d.z, &def->radius,
					 &def->energy, &def->impulse ) >= 9 )
		{
			c.impact.origin = o;
			def->direction = Normalize( d );
			Push( script, &c, t, peer );
		}
		return true;
	}
	if ( SameWord( name, "ray" ) || SameWord( name, "point" ) )
	{
		bool ray = SameWord( name, "ray" );
		c.kind = lp_commandImpact;
		lpImpactDef* def = &c.impact.def;
		int fields = sscanf( rest, "%31s %f %f %f %f %f %f %f %f %f %d %f", name, &o.x, &o.y, &o.z, &def->direction.x,
							 &def->direction.y, &def->direction.z, &def->radius, &def->energy, &def->impulse, &a, &c.impact.range );
		if ( fields == ( ray ? 12 : 11 ) )
		{
			def->explosion = a != 0;
			c.impact.origin = ray ? o : lpVec3_zero;
			def->point = ray ? lpVec3_zero : o;
			c.impact.range = ray ? c.impact.range : 0.0f;
			Push( script, &c, t, peer );
		}
		return true;
	}
	if ( SameWord( name, "pull" ) )
	{
		c.kind = lp_commandPull;
		c.pull.generation = LP_ANY_GENERATION;
		c.pull.maxAccel = 40.0f;
		c.pull.maxMass = 400.0f;
		lpCommandPull* pull = &c.pull;
		if ( sscanf( rest, "%31s %f %f %f %f %f %f %d %u %f %f", name, &pull->target.x, &pull->target.y, &pull->target.z,
					 &pull->localPoint.x, &pull->localPoint.y, &pull->localPoint.z, &pull->piece, &pull->generation,
					 &pull->maxAccel, &pull->maxMass ) >= 8 )
		{
			Push( script, &c, t, peer );
		}
		return true;
	}
	if ( SameWord( name, "spawn" ) )
	{
		c.kind = lp_commandSpawn;
		lpCommandSpawn* s = &c.spawn;
		if ( sscanf( rest, "%31s %d %f %f %f %f %f %f %f %f %f %f %f %f %f", name, &s->templateIndex, &s->transform.p.x,
					 &s->transform.p.y, &s->transform.p.z, &s->transform.q.v.x, &s->transform.q.v.y, &s->transform.q.v.z,
					 &s->transform.q.s, &s->linearVelocity.x, &s->linearVelocity.y, &s->linearVelocity.z, &s->angularVelocity.x,
					 &s->angularVelocity.y, &s->angularVelocity.z ) == 15 )
		{
			Push( script, &c, t, peer );
		}
		return true;
	}
	if ( SameWord( name, "drive" ) )
	{
		c.kind = lp_commandVehicleControl;
		lpVehicleControl* v = &c.vehicleControl.control;
		if ( sscanf( rest, "%31s %d %f %f %f %d", name, &c.vehicleControl.vehicle, &v->throttle, &v->brake, &v->steer, &a ) == 6 )
		{
			v->handbrake = a != 0;
			Push( script, &c, t, peer );
		}
		return true;
	}
	if ( SameWord( name, "walk" ) )
	{
		c.kind = lp_commandRigControl;
		lpRigControl* r = &c.rigControl.control;
		if ( sscanf( rest, "%31s %d %f %f %f %f", name, &c.rigControl.rig, &r->forward, &r->strafe, &r->turn, &r->crouch ) == 6 )
		{
			Push( script, &c, t, peer );
		}
		return true;
	}
	if ( SameWord( name, "reach" ) )
	{
		c.kind = lp_commandLimbTarget;
		lpCommandLimbTarget* r = &c.limbTarget;
		if ( sscanf( rest, "%31s %d %d %d %f %f %f", name, &r->rig, &r->limb, &a, &r->point.x, &r->point.y, &r->point.z ) == 7 )
		{
			r->active = a != 0;
			Push( script, &c, t, peer );
		}
		return true;
	}
	if ( SameWord( name, "grab" ) || SameWord( name, "claw" ) )
	{
		c.kind = lp_commandClaw;
		lpCommandClaw* k = &c.claw;
		int fields = sscanf( rest, "%31s %d %d %d %f %f %f", name, &k->rig, &k->limb, &m, &k->maxForce, &k->maxTorque, &k->strength );
		if ( SameWord( name, "grab" ) && fields >= 3 )
		{
			// The hexapod's claw
			k->mode = lp_clawToggle;
			k->maxForce = 40000.0f;
			k->maxTorque = 15000.0f;
			k->strength = 5000.0f;
			Push( script, &c, t, peer );
		}
		else if ( fields == 7 )
		{
			k->mode = (uint8_t)m;
			Push( script, &c, t, peer );
		}
		return true;
	}
	if ( SameWord( name, "release" ) )
	{
		char which[16] = { 0 };
		c.kind = lp_commandRelease;
		if ( sscanf( rest, "%31s %15s %d", name, which, &b ) == 3 )
		{
			c.release.vehicle = SameWord( which, "vehicle" ) ? b : -1;
			c.release.rig = SameWord( which, "rig" ) ? b : -1;
			Push( script, &c, t, peer );
		}
		return true;
	}
	fprintf( stderr, "script: unknown command '%s' (expected rifle, grenade, cannon, hammer, ball, flask, impact, ray, point, "
					 "pull, spawn, drive, walk, reach, grab, claw, release)\n",
			 name );
	return false;
}

bool lpScriptLoad( lpScript* script, const char* path )
{
	FILE* f = fopen( path, "r" );
	if ( f == NULL )
	{
		fprintf( stderr, "cannot open script %s\n", path );
		return false;
	}
	char line[512];
	while ( fgets( line, sizeof( line ), f ) )
	{
		lpScriptParseLine( script, line );
	}
	fclose( f );
	// By tick, each tick in file order (scripts come nearly sorted: an insertion sort)
	for ( int i = 1; i < script->count; ++i )
	{
		lpCommand c = script->commands[i];
		int j = i;
		while ( j > 0 && script->commands[j - 1].tick > c.tick )
		{
			script->commands[j] = script->commands[j - 1];
			j -= 1;
		}
		script->commands[j] = c;
	}
	return true;
}

void lpScriptFree( lpScript* script )
{
	free( script->commands );
	memset( script, 0, sizeof( *script ) );
}

bool lpScriptWrite( FILE* file, const lpCommand* c )
{
	bool written = c->peer != LP_PEER_SCENE &&
				   ( c->kind == lp_commandImpact || c->kind == lp_commandPull || c->kind == lp_commandSpawn ||
					 c->kind == lp_commandVehicleControl || c->kind == lp_commandRigControl || c->kind == lp_commandLimbTarget ||
					 c->kind == lp_commandClaw || c->kind == lp_commandRelease );
	if ( written == false )
	{
		return false;
	}
	fprintf( file, "%lld", (long long)c->tick );
	if ( c->peer != 0 )
	{
		fprintf( file, ":%d", c->peer );
	}
	switch ( c->kind )
	{
		case lp_commandImpact:
		{
			const lpImpactDef* def = &c->impact.def;
			lpVec3 at = c->impact.range > 0.0f ? c->impact.origin : def->point;
			fprintf( file, " %s %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %d", c->impact.range > 0.0f ? "ray" : "point",
					 (double)at.x, (double)at.y, (double)at.z, (double)def->direction.x, (double)def->direction.y,
					 (double)def->direction.z, (double)def->radius, (double)def->energy, (double)def->impulse, def->explosion ? 1 : 0 );
			if ( c->impact.range > 0.0f )
			{
				fprintf( file, " %.9g", (double)c->impact.range );
			}
			break;
		}
		case lp_commandPull:
		{
			const lpCommandPull* p = &c->pull;
			fprintf( file, " pull %.9g %.9g %.9g %.9g %.9g %.9g %d %u %.9g %.9g", (double)p->target.x, (double)p->target.y,
					 (double)p->target.z, (double)p->localPoint.x, (double)p->localPoint.y, (double)p->localPoint.z, p->piece,
					 p->generation, (double)p->maxAccel, (double)p->maxMass );
			break;
		}
		case lp_commandSpawn:
		{
			const lpCommandSpawn* s = &c->spawn;
			fprintf( file, " spawn %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g", s->templateIndex,
					 (double)s->transform.p.x, (double)s->transform.p.y, (double)s->transform.p.z, (double)s->transform.q.v.x,
					 (double)s->transform.q.v.y, (double)s->transform.q.v.z, (double)s->transform.q.s, (double)s->linearVelocity.x,
					 (double)s->linearVelocity.y, (double)s->linearVelocity.z, (double)s->angularVelocity.x,
					 (double)s->angularVelocity.y, (double)s->angularVelocity.z );
			break;
		}
		case lp_commandVehicleControl:
		{
			const lpVehicleControl* v = &c->vehicleControl.control;
			fprintf( file, " drive %d %.9g %.9g %.9g %d", c->vehicleControl.vehicle, (double)v->throttle, (double)v->brake,
					 (double)v->steer, v->handbrake ? 1 : 0 );
			break;
		}
		case lp_commandRigControl:
		{
			const lpRigControl* r = &c->rigControl.control;
			fprintf( file, " walk %d %.9g %.9g %.9g %.9g", c->rigControl.rig, (double)r->forward, (double)r->strafe,
					 (double)r->turn, (double)r->crouch );
			break;
		}
		case lp_commandLimbTarget:
		{
			const lpCommandLimbTarget* r = &c->limbTarget;
			fprintf( file, " reach %d %d %d %.9g %.9g %.9g", r->rig, r->limb, r->active ? 1 : 0, (double)r->point.x,
					 (double)r->point.y, (double)r->point.z );
			break;
		}
		case lp_commandClaw:
		{
			const lpCommandClaw* k = &c->claw;
			fprintf( file, " claw %d %d %d %.9g %.9g %.9g", k->rig, k->limb, k->mode, (double)k->maxForce, (double)k->maxTorque,
					 (double)k->strength );
			break;
		}
		default: // a release
			fprintf( file, " release %s %d", c->release.vehicle >= 0 ? "vehicle" : "rig",
					 c->release.vehicle >= 0 ? c->release.vehicle : c->release.rig );
			break;
	}
	fprintf( file, "\n" );
	fflush( file );
	return true;
}

int lpScriptPlay( lpWorld* world, const lpScript* script, int next )
{
	int64_t tick = (int64_t)lpWorld_GetTick( world );
	while ( next < script->count && script->commands[next].tick <= tick )
	{
		lpWorld_Submit( world, script->commands + next ); // one whose tick has passed is refused
		next += 1;
	}
	return next;
}
