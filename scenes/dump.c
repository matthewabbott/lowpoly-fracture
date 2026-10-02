// SPDX-License-Identifier: MIT
// The state of a world as JSON (see dump.h)

#include "dump.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static const char* s_kinds[] = { "structure", "debris", "rubble", "ghost", "scrap" };
static const char* s_tiers[] = { "full", "light" };
static const char* s_links[] = { "weld", "hinge", "ball", "rope", "wheel" };

static void Num( FILE* f, float x )
{
	if ( isfinite( x ) )
	{
		fprintf( f, "%.9g", (double)x );
	}
	else
	{
		fprintf( f, "null" );
	}
}

static void Vec( FILE* f, const char* name, lpVec3 v )
{
	fprintf( f, ", \"%s\": [", name );
	Num( f, v.x );
	fprintf( f, ", " );
	Num( f, v.y );
	fprintf( f, ", " );
	Num( f, v.z );
	fprintf( f, "]" );
}

static void Field( FILE* f, const char* name, float x )
{
	fprintf( f, ", \"%s\": ", name );
	Num( f, x );
}

static const char* Name( const char** names, int count, int i )
{
	return i >= 0 && i < count ? names[i] : "?";
}

static void Stats( FILE* f, const lpStats* s )
{
	// Counters only: the timings depend on the machine
	fprintf( f, "  \"stats\": {\"pieceCount\": %d, \"bondCount\": %d, \"linkCount\": %d, \"linkBreaks\": %d, \"linkRebuilds\": %d,\n",
			 s->pieceCount, s->bondCount, s->linkCount, s->linkBreaks, s->linkRebuilds );
	fprintf( f, "    \"structureBodies\": %d, \"debrisBodies\": %d, \"awakeDebris\": %d, \"rubbleBodies\": %d, \"fullDebris\": %d, "
				"\"lightDebris\": %d, \"ghostBodies\": %d, \"scrapBodies\": %d,\n",
			 s->structureBodies, s->debrisBodies, s->awakeDebris, s->rubbleBodies, s->fullDebris, s->lightDebris, s->ghostBodies,
			 s->scrapBodies );
	fprintf( f, "    \"demotionsThisStep\": %d, \"deferredJobs\": %d, \"ghostCasts\": %d, \"impactsThisStep\": %d, "
				"\"fracturesThisStep\": %d, \"cellsThisStep\": %d, \"splitsThisStep\": %d, \"clipFailures\": %d,\n",
			 s->demotionsThisStep, s->deferredJobs, s->ghostCasts, s->impactsThisStep, s->fracturesThisStep, s->cellsThisStep,
			 s->splitsThisStep, s->clipFailures );
	fprintf( f, "    \"stressIterations\": %d, \"stressBreaks\": %d, \"stressSolves\": %d, \"stressJudged\": %d, \"stressReduced\": %d, "
				"\"stressDissolved\": %d, \"clusteredPieces\": %d, \"stressAudits\": %d,\n",
			 s->stressIterations, s->stressBreaks, s->stressSolves, s->stressJudged, s->stressReduced, s->stressDissolved,
			 s->clusteredPieces, s->stressAudits );
	fprintf( f, "    \"provisionalStructures\": %d, \"auditBacklog\": %d, \"stressWaiting\": %d, \"unsettledStructures\": %d, "
				"\"settleIterations\": %d,\n",
			 s->provisionalStructures, s->auditBacklog, s->stressWaiting, s->unsettledStructures, s->settleIterations );
	fprintf( f, "    \"wheelCasts\": %d, \"supplyUpdates\": %d, \"motorSets\": %d, \"leakingPools\": %d, \"footCasts\": %d, "
				"\"fpRepairs\": %d, \"shapes\": %d, \"contacts\": %d, \"awakeContacts\": %d},\n",
			 s->wheelCasts, s->supplyUpdates, s->motorSets, s->leakingPools, s->footCasts, s->fpRepairs, s->shapes, s->contacts,
			 s->awakeContacts );
}

static void Bodies( FILE* f, const lpWorld* world )
{
	fprintf( f, "  \"bodies\": [" );
	bool first = true;
	for ( int i = 0; i < lpWorld_GetBodyCapacity( world ); ++i )
	{
		lpBodyInfo b = lpWorld_GetBodyInfo( world, i );
		if ( b.alive == false )
		{
			continue;
		}
		fprintf( f, "%s\n    {\"index\": %d, \"generation\": %u, \"kind\": \"%s\", \"tier\": \"%s\", \"awake\": %s, \"unsettled\": %s, "
					"\"pieces\": %d",
				 first ? "" : ",", i, b.generation, Name( s_kinds, 5, b.kind ), Name( s_tiers, 2, b.tier ),
				 b.awake ? "true" : "false", b.unsettled ? "true" : "false", b.pieceCount );
		Field( f, "volume", b.volume );
		Vec( f, "position", b.transform.p );
		fprintf( f, ", \"rotation\": [" );
		Num( f, b.transform.q.v.x );
		fprintf( f, ", " );
		Num( f, b.transform.q.v.y );
		fprintf( f, ", " );
		Num( f, b.transform.q.v.z );
		fprintf( f, ", " );
		Num( f, b.transform.q.s );
		fprintf( f, "]" );
		Vec( f, "velocity", b.linearVelocity );
		Vec( f, "angularVelocity", b.angularVelocity );
		fprintf( f, "}" );
		first = false;
	}
	fprintf( f, "\n  ],\n" );
}

static void Pieces( FILE* f, const lpWorld* world )
{
	fprintf( f, "  \"pieces\": [" );
	bool first = true;
	for ( int i = 0; i < lpWorld_GetPieceCapacity( world ); ++i )
	{
		lpPieceInfo p = lpWorld_GetPieceInfo( world, i );
		if ( p.body < 0 )
		{
			continue;
		}
		lpWorldTransform xf;
		lpWorld_GetBodyTransform( world, p.body, &xf );
		fprintf( f, "%s\n    {\"index\": %d, \"body\": %d, \"generation\": %u, \"userId\": %u, \"part\": %d, \"tag\": %u, "
					"\"material\": \"%s\", \"joint\": \"%s\", \"supplied\": %u",
				 first ? "" : ",", i, p.body, p.generation, p.userId, p.part, (unsigned)p.tag, lpGetMaterial( p.material )->name,
				 lpGetJoint( p.joint )->name, (unsigned)p.supplied );
		Field( f, "volume", p.volume );
		Vec( f, "centroid", lpTransformPoint( xf, p.centroid ) );
		fprintf( f, "}" );
		first = false;
	}
	fprintf( f, "\n  ],\n" );
}

static void Bonds( FILE* f, const lpWorld* world )
{
	fprintf( f, "  \"bonds\": [" );
	bool first = true;
	for ( int i = 0; i < lpWorld_GetBondCapacity( world ); ++i )
	{
		lpBondInfo d = lpWorld_GetBondInfo( world, i );
		if ( d.alive == false )
		{
			continue;
		}
		fprintf( f, "%s\n    {\"index\": %d, \"a\": %d, \"b\": %d, \"joint\": \"%s\"", first ? "" : ",", i, d.pieceA, d.pieceB,
				 lpGetJoint( d.joint )->name );
		Field( f, "area", d.area );
		Field( f, "health", d.health );
		Field( f, "strength", d.strength );
		Field( f, "utilization", d.utilization );
		Field( f, "strain", d.strain );
		Vec( f, "centroid", d.centroid );
		Vec( f, "normal", d.normal );
		Vec( f, "force", d.force );
		Vec( f, "moment", d.moment );
		fprintf( f, "}" );
		first = false;
	}
	fprintf( f, "\n  ],\n" );
}

static void Contacts( FILE* f, const lpWorld* world )
{
	fprintf( f, "  \"contacts\": [" );
	bool first = true;
	int capacity = 0;
	lpContactInfo* contacts = NULL;
	for ( int i = 0; i < lpWorld_GetBodyCapacity( world ); ++i )
	{
		int count = lpWorld_GetBodyContacts( world, i, contacts, capacity );
		if ( count > capacity )
		{
			free( contacts );
			capacity = count;
			contacts = (lpContactInfo*)malloc( (size_t)capacity * sizeof( lpContactInfo ) );
			count = lpWorld_GetBodyContacts( world, i, contacts, capacity );
		}
		for ( int k = 0; k < count; ++k )
		{
			const lpContactInfo* c = contacts + k;
			fprintf( f, "%s\n    {\"body\": %d, \"piece\": %d, \"other\": %d", first ? "" : ",", i, c->piece, c->other );
			Vec( f, "point", c->point );
			Vec( f, "normal", c->normal );
			Field( f, "separation", c->separation );
			Field( f, "impulse", c->impulse );
			fprintf( f, "}" );
			first = false;
		}
	}
	free( contacts );
	fprintf( f, "\n  ],\n" );
}

static void Links( FILE* f, const lpWorld* world )
{
	fprintf( f, "  \"links\": [" );
	bool first = true;
	for ( int i = 0; i < lpWorld_GetLinkCapacity( world ); ++i )
	{
		lpLinkState s = lpWorld_GetLinkState( world, i );
		if ( s.alive == false )
		{
			continue;
		}
		fprintf( f, "%s\n    {\"index\": %d, \"generation\": %u, \"type\": \"%s\", \"userId\": %u, \"bodyA\": %d, \"bodyB\": %d, "
					"\"slack\": %s, \"supplied\": %u",
				 first ? "" : ",", i, s.generation, Name( s_links, 5, s.type ), s.userId, s.bodyA, s.bodyB, s.slack ? "true" : "false",
				 (unsigned)s.supplied );
		Vec( f, "pointA", s.pointA );
		Vec( f, "pointB", s.pointB );
		Vec( f, "force", s.force );
		Vec( f, "torque", s.torque );
		Field( f, "utilization", s.utilization );
		Field( f, "strain", s.strain );
		Field( f, "health", s.health );
		Field( f, "length", s.length );
		Field( f, "angle", s.angle );
		Field( f, "motorTorque", s.motorTorque );
		Field( f, "motorCap", s.motorCap );
		fprintf( f, "}" );
		first = false;
	}
	fprintf( f, "\n  ],\n" );
}

static void Vehicles( FILE* f, const lpWorld* world )
{
	fprintf( f, "  \"vehicles\": [" );
	bool first = true;
	for ( int i = 0; i < lpWorld_GetVehicleCapacity( world ); ++i )
	{
		lpVehicleState v = lpWorld_GetVehicleState( world, i );
		if ( v.alive == false )
		{
			continue;
		}
		fprintf( f, "%s\n    {\"index\": %d, \"body\": %d, \"attached\": %d, \"grounded\": %d, \"driven\": %d, \"steerable\": %d",
				 first ? "" : ",", i, v.body, v.attached, v.grounded, v.driven, v.steerable );
		Field( f, "power", v.power );
		Field( f, "speed", v.speed );
		Vec( f, "position", v.position );
		Vec( f, "forward", v.forward );
		Vec( f, "up", v.up );
		fprintf( f, ", \"wheels\": [" );
		for ( int k = 0; k < v.wheelCount; ++k )
		{
			int link = lpWorld_GetVehicleWheel( world, i, k );
			lpWheelState w = link >= 0 ? lpWorld_GetWheelState( world, link ) : ( lpWheelState ){ 0 };
			fprintf( f, "%s{\"link\": %d, \"alive\": %s, \"grounded\": %s, \"body\": %d, \"groundPiece\": %d", k > 0 ? ", " : "", link,
					 w.alive ? "true" : "false", w.grounded ? "true" : "false", w.body, w.groundPiece );
			Field( f, "load", w.load );
			Field( f, "slip", w.slip );
			Field( f, "length", w.length );
			Vec( f, "hub", w.hub.p );
			fprintf( f, "}" );
		}
		fprintf( f, "]}" );
		first = false;
	}
	fprintf( f, "\n  ],\n" );
}

static void Rigs( FILE* f, const lpWorld* world )
{
	fprintf( f, "  \"rigs\": [" );
	bool first = true;
	for ( int i = 0; i < lpWorld_GetRigCapacity( world ); ++i )
	{
		lpRigState r = lpWorld_GetRigState( world, i );
		if ( r.alive == false )
		{
			continue;
		}
		fprintf( f, "%s\n    {\"index\": %d, \"body\": %d, \"attached\": %d, \"able\": %d, \"planted\": %d, \"idle\": %s, "
					"\"crawling\": %s",
				 first ? "" : ",", i, r.body, r.attached, r.able, r.planted, r.idle ? "true" : "false", r.crawling ? "true" : "false" );
		Field( f, "height", r.height );
		Field( f, "speed", r.speed );
		Vec( f, "position", r.position );
		Vec( f, "forward", r.forward );
		Vec( f, "up", r.up );
		fprintf( f, ", \"limbs\": [" );
		for ( int k = 0; k < r.limbCount; ++k )
		{
			lpLimbState l = lpWorld_GetLimbState( world, i, k );
			fprintf( f, "%s\n      {\"attached\": %s, \"able\": %s, \"planted\": %s, \"swinging\": %s, \"reaching\": %s, \"joints\": %d, "
						"\"footBody\": %d, \"touching\": %d",
					 k > 0 ? "," : "", l.attached ? "true" : "false", l.able ? "true" : "false", l.planted ? "true" : "false",
					 l.swinging ? "true" : "false", l.reaching ? "true" : "false", l.joints, l.footBody, l.touching );
			Field( f, "strength", l.strength );
			Field( f, "reach", l.reach );
			Field( f, "depth", l.depth );
			Vec( f, "foot", l.foot );
			fprintf( f, "}" );
		}
		fprintf( f, "]}" );
		first = false;
	}
	fprintf( f, "\n  ]\n" );
}

void lpDumpWorld( FILE* f, const lpWorld* world )
{
	lpStats stats = lpWorld_GetStats( world );
	fprintf( f, "{\n  \"tick\": %llu, \"hash\": \"%016llx\", \"solverHash\": \"%016llx\",\n",
			 (unsigned long long)lpWorld_GetTick( world ), (unsigned long long)lpWorld_Hash( world ),
			 (unsigned long long)lpWorld_HashStress( world ) );
	Stats( f, &stats );
	Bodies( f, world );
	Pieces( f, world );
	Bonds( f, world );
	Contacts( f, world );
	Links( f, world );
	Vehicles( f, world );
	Rigs( f, world );
	fprintf( f, "}\n" );
}

bool lpParseDumpArg( const char* arg, int64_t* tick, const char** path )
{
	const char* colon = strchr( arg, ':' );
	// A drive letter (C:\...) is not the separator: the tick comes first and is all digits
	if ( colon == NULL || colon == arg )
	{
		return false;
	}
	for ( const char* c = arg; c < colon; ++c )
	{
		if ( *c < '0' || *c > '9' )
		{
			return false;
		}
	}
	*tick = strtoll( arg, NULL, 10 );
	*path = colon + 1;
	return **path != 0;
}
