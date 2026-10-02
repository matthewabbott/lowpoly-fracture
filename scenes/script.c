// SPDX-License-Identifier: MIT
// Replay scripts (see script.h): parsing, recording and applying tick-stamped inputs

#include "script.h"

#include "scenes.h"

#include <stdlib.h>
#include <string.h>

static const char* s_tokens[lp_scriptToolCount] = { "rifle", "grenade", "cannon", "hammer", "ball", "flask", "pull", "blow" };

lpScriptState lpDefaultScriptState( void )
{
	lpScriptState s = { 0 };
	s.playerVehicle = -1;
	s.playerRig = -1;
	s.grip = -1;
	s.gripLimb = -1;
	return s;
}

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

static void Push( lpScript* script, const lpScriptEvent* e )
{
	if ( script->count == script->capacity )
	{
		int capacity = script->capacity > 0 ? 2 * script->capacity : 64;
		lpScriptEvent* events = (lpScriptEvent*)realloc( script->events, (size_t)capacity * sizeof( lpScriptEvent ) );
		if ( events == NULL )
		{
			return;
		}
		script->events = events;
		script->capacity = capacity;
	}
	script->events[script->count++] = *e;
}

bool lpScriptParseLine( lpScript* script, const char* line )
{
	if ( line[0] == '#' || line[0] == '\n' || line[0] == '\r' || line[0] == 0 )
	{
		return true;
	}
	lpScriptEvent e = { 0 };
	long long t = 0;
	char name[32] = { 0 };
	e.index = -1;
	int handbrake = 0;
	int active = 0;
	bool named = sscanf( line, "%lld %31s", &t, name ) == 2;
	if ( named && SameWord( name, "walk" ) )
	{
		if ( sscanf( line, "%lld %31s %d %f %f %f %f", &t, name, &e.index, &e.walk.forward, &e.walk.strafe, &e.walk.turn,
					 &e.walk.crouch ) == 7 )
		{
			e.tick = t;
			e.kind = lp_scriptWalk;
			Push( script, &e );
		}
		return true;
	}
	if ( named && SameWord( name, "reach" ) )
	{
		if ( sscanf( line, "%lld %31s %d %d %d %f %f %f", &t, name, &e.index, &e.limb, &active, &e.origin.x, &e.origin.y,
					 &e.origin.z ) == 8 )
		{
			e.tick = t;
			e.kind = lp_scriptReach;
			e.active = active != 0;
			Push( script, &e );
		}
		return true;
	}
	if ( named && SameWord( name, "grab" ) )
	{
		if ( sscanf( line, "%lld %31s %d %d", &t, name, &e.index, &e.limb ) == 4 )
		{
			e.tick = t;
			e.kind = lp_scriptGrab;
			Push( script, &e );
		}
		return true;
	}
	if ( named && SameWord( name, "impact" ) )
	{
		if ( sscanf( line, "%lld %31s %f %f %f %f %f %f %f %f %f", &t, name, &e.origin.x, &e.origin.y, &e.origin.z, &e.dir.x,
					 &e.dir.y, &e.dir.z, &e.radius, &e.energy, &e.impulse ) >= 10 )
		{
			e.tick = t;
			e.kind = lp_scriptImpact;
			Push( script, &e );
		}
		return true;
	}
	if ( named && SameWord( name, "drive" ) )
	{
		if ( sscanf( line, "%lld %31s %d %f %f %f %d", &t, name, &e.index, &e.control.throttle, &e.control.brake,
					 &e.control.steer, &handbrake ) == 7 )
		{
			e.tick = t;
			e.kind = lp_scriptDrive;
			e.control.handbrake = handbrake != 0;
			Push( script, &e );
		}
		return true;
	}
	if ( sscanf( line, "%lld %31s %f %f %f %f %f %f %d", &t, name, &e.origin.x, &e.origin.y, &e.origin.z, &e.dir.x, &e.dir.y,
				 &e.dir.z, &e.index ) < 8 )
	{
		return true;
	}
	e.tick = t;
	e.kind = -1;
	for ( int k = 0; k < lp_scriptToolCount; ++k )
	{
		if ( SameWord( name, s_tokens[k] ) )
		{
			e.kind = k;
		}
	}
	if ( e.kind < 0 )
	{
		fprintf( stderr, "script: unknown tool '%s' (expected rifle, grenade, cannon, hammer, ball, flask, pull, blow, drive, walk, reach, grab, impact)\n",
				 name );
		return false;
	}
	// A held blower: the last field is how many ticks it stays on
	int repeat = e.kind == lp_scriptBlow && e.index > 1 ? e.index : 1;
	for ( int k = 0; k < repeat; ++k )
	{
		lpScriptEvent copy = e;
		copy.tick = e.tick + k;
		copy.index = e.kind == lp_scriptPull ? e.index : -1;
		Push( script, &copy );
	}
	return true;
}

bool lpScriptLoad( lpScript* script, const char* path )
{
	FILE* f = fopen( path, "r" );
	if ( f == NULL )
	{
		fprintf( stderr, "cannot open script %s\n", path );
		return false;
	}
	char line[256];
	while ( fgets( line, sizeof( line ), f ) )
	{
		lpScriptParseLine( script, line );
	}
	fclose( f );
	return true;
}

void lpScriptFree( lpScript* script )
{
	free( script->events );
	script->events = NULL;
	script->count = 0;
	script->capacity = 0;
}

void lpScriptWrite( FILE* file, const lpScriptEvent* e )
{
	switch ( e->kind )
	{
		case lp_scriptWalk:
			fprintf( file, "%lld walk %d %.9g %.9g %.9g %.9g\n", (long long)e->tick, e->index, (double)e->walk.forward,
					 (double)e->walk.strafe, (double)e->walk.turn, (double)e->walk.crouch );
			break;
		case lp_scriptReach:
			fprintf( file, "%lld reach %d %d %d %.9g %.9g %.9g\n", (long long)e->tick, e->index, e->limb, e->active ? 1 : 0,
					 (double)e->origin.x, (double)e->origin.y, (double)e->origin.z );
			break;
		case lp_scriptGrab:
			fprintf( file, "%lld grab %d %d\n", (long long)e->tick, e->index, e->limb );
			break;
		case lp_scriptImpact:
			fprintf( file, "%lld impact %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n", (long long)e->tick, (double)e->origin.x,
					 (double)e->origin.y, (double)e->origin.z, (double)e->dir.x, (double)e->dir.y, (double)e->dir.z, (double)e->radius,
					 (double)e->energy, (double)e->impulse );
			break;
		case lp_scriptDrive:
			fprintf( file, "%lld drive %d %.9g %.9g %.9g %d\n", (long long)e->tick, e->index, (double)e->control.throttle,
					 (double)e->control.brake, (double)e->control.steer, e->control.handbrake ? 1 : 0 );
			break;
		default:
			fprintf( file, "%lld %s %.9g %.9g %.9g %.9g %.9g %.9g %d\n", (long long)e->tick, s_tokens[e->kind], (double)e->origin.x,
					 (double)e->origin.y, (double)e->origin.z, (double)e->dir.x, (double)e->dir.y, (double)e->dir.z, e->index );
			break;
	}
	fflush( file );
}

// A chunky hexagonal bottle with a neck, thrown: it goes off when it lands hard
static void ThrowFlask( lpWorld* world, lpVec3 origin, lpVec3 dir )
{
	lpVec3 points[15];
	for ( int i = 0; i < 6; ++i )
	{
		lpCosSin cs = lpComputeCosSin( 1.0471976f * (float)i );
		points[i] = ( lpVec3 ){ 0.09f * cs.cosine, -0.12f, 0.09f * cs.sine };
		points[6 + i] = ( lpVec3 ){ 0.09f * cs.cosine, 0.06f, 0.09f * cs.sine };
	}
	points[12] = ( lpVec3 ){ 0.04f, 0.2f, 0.0f };
	points[13] = ( lpVec3 ){ -0.03f, 0.2f, 0.035f };
	points[14] = ( lpVec3 ){ -0.03f, 0.2f, -0.035f };
	lpPartDef part = lpDefaultPartDef();
	part.points = points;
	part.pointCount = 15;
	part.material = lp_glass;
	part.color = 0x6FD68Au;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = lpMulAdd( origin, 0.8f, dir );
	def.parts = &part;
	def.partCount = 1;
	def.linearVelocity = lpAdd( lpMulSV( 16.0f, dir ), ( lpVec3 ){ 0.0f, 2.5f, 0.0f } );
	def.angularVelocity = ( lpVec3 ){ 4.0f, 1.0f, 7.0f };
	def.detonator.triggerSpeed = 4.5f;
	def.detonator.radius = 1.8f;
	def.detonator.energy = 120000.0f;
	def.detonator.speed = 12.0f;
	lpCreateObject( world, &def );
}

// A metal cannonball, a chunky low-poly sphere of golden-spiral points, fired at 45 m/s
static void FireBall( lpWorld* world, lpVec3 origin, lpVec3 dir )
{
	lpVec3 points[20];
	for ( int i = 0; i < 20; ++i )
	{
		float y = 1.0f - 2.0f * ( (float)i + 0.5f ) / 20.0f;
		float r = sqrtf( 1.0f - y * y );
		float a = 2.39996323f * (float)i;
		lpCosSin cs = lpComputeCosSin( a );
		points[i] = ( lpVec3 ){ 0.3f * r * cs.cosine, 0.3f * y, 0.3f * r * cs.sine };
	}
	lpPartDef part = lpDefaultPartDef();
	part.points = points;
	part.pointCount = 20;
	part.material = lp_metal;
	part.color = 0x3A3D42u;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = lpMulAdd( origin, 1.0f, dir );
	def.parts = &part;
	def.partCount = 1;
	def.linearVelocity = lpMulSV( 45.0f, dir );
	lpCreateObject( world, &def );
}

void lpScriptApply( lpWorld* world, const lpScriptEvent* e, lpScriptState* state )
{
	// A tool's aim is normalised here, not on load, so a recorded event replays exactly as it applied live
	lpVec3 dir = e->kind == lp_scriptPull ? e->dir : Normalize( e->dir );
	switch ( e->kind )
	{
		case lp_scriptDrive:
			lpWorld_SetVehicleControl( world, e->index, &e->control );
			state->playerVehicle = e->index; // from now on the scene's drivers leave it to the events
			return;
		case lp_scriptWalk:
			lpWorld_SetRigControl( world, e->index, &e->walk );
			state->playerRig = e->index;
			return;
		case lp_scriptReach:
			lpWorld_SetLimbTarget( world, e->index, e->limb, e->active, e->origin );
			state->playerRig = e->index;
			return;
		case lp_scriptGrab:
		{
			// Lets go of the grip it holds (if it has not broken), else grabs what the claw touches
			lpLinkState held = { 0 };
			if ( state->grip >= 0 )
			{
				held = lpWorld_GetLinkState( world, state->grip );
			}
			if ( state->grip >= 0 && held.alive && held.generation == state->gripGeneration )
			{
				lpDestroyLink( world, state->grip );
				state->grip = -1;
			}
			else
			{
				state->grip = lpRigGrab( world, e->index, e->limb );
				state->gripGeneration = state->grip >= 0 ? lpWorld_GetLinkState( world, state->grip ).generation : 0;
				state->gripLimb = e->limb;
			}
			state->playerRig = e->index;
			return;
		}
		case lp_scriptPull:
			// origin = target, dir = grabbed point in the body frame
			lpWorld_Pull( world, e->index, e->dir, e->origin, 40.0f, 400.0f );
			return;
		case lp_scriptBlow:
			// 8 m cone of air: wakes and pushes rubble, scrap and ghosts, so a road can be cleared. Gentle enough that
			// blown rubble does not smash into what it lands against (damage starts at 4 m/s).
			lpWorld_Blow( world, e->origin, dir, 8.0f, 0.35f, 4.5f );
			return;
		case lp_scriptFlask:
			ThrowFlask( world, e->origin, dir );
			return;
		case lp_scriptBall:
			FireBall( world, e->origin, dir );
			return;
		default:
			break;
	}

	float range = e->kind == lp_scriptHammer ? 4.0f : 250.0f;
	lpRayHit hit = lpWorld_CastRay( world, e->origin, lpMulSV( range, dir ) );
	if ( hit.hit == false )
	{
		return;
	}

	lpImpactDef im = { 0 };
	im.point = hit.point;
	im.direction = dir;
	switch ( e->kind )
	{
		case lp_scriptRifle:
			im.radius = 0.35f;
			im.energy = 4000.0f;
			im.impulse = 20.0f;
			break;
		case lp_scriptGrenade:
			im.radius = 1.4f;
			im.energy = 80000.0f;
			im.impulse = 12.0f;
			im.explosion = true;
			break;
		case lp_scriptCannon:
			im.radius = 2.3f;
			im.energy = 350000.0f;
			im.impulse = 18.0f;
			im.explosion = true;
			break;
		case lp_scriptHammer:
			im.radius = 0.6f;
			im.energy = 14000.0f;
			im.impulse = 60.0f;
			break;
		case lp_scriptImpact:
			im.radius = e->radius;
			im.energy = e->energy;
			im.impulse = e->impulse;
			break;
		default:
			return;
	}
	lpWorld_AddImpact( world, &im ); // dust comes from the core, in the colour of what broke
}

void lpScriptPlay( lpWorld* world, const lpScript* script, int64_t tick, lpScriptState* state )
{
	while ( state->next < script->count && script->events[state->next].tick <= tick )
	{
		lpScriptApply( world, &script->events[state->next], state );
		state->next += 1;
	}
}
