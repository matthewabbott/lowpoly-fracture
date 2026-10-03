// SPDX-License-Identifier: MIT
// lpf_bench --twin: two worlds of one scene stepped side by side on the same inputs, a desync injected into the
// second at a tick, and what follows measured: the tick the state hash sees it (and the tick it shows outside the
// physics engine's hidden state), what the hash names when followed down, and then either how the difference spreads
// (--cone: per tick, the elements, bodies and units that differ and the farthest differing body) or whether repairing
// the units it reached brings the second world back (--repair motion,warm,sleep[@delay]: from that many ticks after
// detection, as a host would learn of it late; the units are copied as they are then).
//
//   lpf_bench --scene town --period 3 --ticks 600 --twin warm:200 --cone build/cone.txt
//   lpf_bench --scene town --period 3 --ticks 600 --twin velocity:200 --repair motion,warm,sleep@4
//
// Each world runs its own scene logic (the bombardment aims from its own state), as every machine of a session does.

#include "twin.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The body an element is about (-1: none, or not a body's)
static int ElementBody( const lpWorld* world, int category, int slot )
{
	if ( category == lp_hashBodies || category == lp_hashStress || category == lp_hashBackend )
	{
		return slot < lpWorld_GetBodyCapacity( world ) ? slot : -1; // the slot may be the other world's alone
	}
	bool piece = category == lp_hashPieces && slot < lpWorld_GetPieceCapacity( world );
	return piece ? lpWorld_GetPieceInfo( world, slot ).body : -1;
}

// The first awake debris body the injection takes: for a velocity, a moving one if there is one (an ulp of a body at
// rest is gone within the step); for a warm start, one with a touching contact (one at rest keeps it). -1 if none.
static int Inject( lpWorld* world, bool warm, int ulps )
{
	for ( int pass = warm ? 1 : 0; pass < 2; ++pass )
	{
		for ( int i = 0; i < lpWorld_GetBodyCapacity( world ); ++i )
		{
			lpBodyInfo info = lpWorld_GetBodyInfo( world, i );
			lpVec3 v = info.linearVelocity;
			if ( info.alive == false || info.kind != lp_kindDebris || info.awake == false || ( pass == 0 && v.x * v.x + v.y * v.y + v.z * v.z < 0.01f ) )
			{
				continue;
			}
			if ( warm == false )
			{
				lpLab_NudgeVelocity( world, i, ulps );
				return i;
			}
			if ( lpLab_NudgeWarmStart( world, i, ulps ) )
			{
				return i;
			}
		}
	}
	return -1;
}

static int ParseRepair( const char* text )
{
	int parts = 0;
	parts |= strstr( text, "motion" ) != NULL ? lp_labMotion : 0;
	parts |= strstr( text, "warm" ) != NULL ? lp_labWarmStarts : 0;
	parts |= strstr( text, "sleep" ) != NULL ? lp_labSleep : 0;
	return parts;
}

static lpWorld* Build( int scene, int workers )
{
	lpWorldDef def = lpDefaultWorldDef();
	def.workerCount = workers;
	lpWorld* world = lpCreateWorld( &def );
	lpBuildScene( world, scene ); // the scene builder is global: worlds are built one after another
	return world;
}

int lpBenchTwin( int scene, int period, int ticks, int workers, const lpScript* script, const char* inject,
				 const char* repair, const char* conePath )
{
	bool warm = strncmp( inject, "warm", 4 ) == 0;
	const char* colon = strchr( inject, ':' );
	int injectTick = colon != NULL ? atoi( colon + 1 ) : 200;
	const char* second = colon != NULL ? strchr( colon + 1, ':' ) : NULL;
	int ulps = second != NULL ? atoi( second + 1 ) : 1;
	int parts = repair != NULL ? ParseRepair( repair ) : 0;
	const char* at = repair != NULL ? strchr( repair, '@' ) : NULL;
	int repairDelay = at != NULL ? atoi( at + 1 ) : 0; // ticks from detection to the first repair (a host's latency)

	lpWorld* a = Build( scene, workers );
	lpWorld* b = Build( scene, workers );
	FILE* cone = conePath != NULL ? fopen( conePath, "w" ) : NULL;
	if ( cone != NULL )
	{
		fprintf( cone, "tick elements bodies units farthestM\n" );
	}
	printf( "twin: %s, a %s nudge of %d ulps at tick %d, %s\n", lpSceneName( scene ), warm ? "warm-start" : "velocity", ulps,
			injectTick, parts != 0 ? "repairing" : "watching" );

	int capacity = 4096;
	lpLabDiff* diffs = malloc( sizeof( lpLabDiff ) * (size_t)capacity );
	int* units = NULL;
	uint8_t* repairUnits = NULL;
	int unitCapacity = 0;
	int injected = -1;
	lpPos origin = { 0 };
	int detected = -1, visible = -1, repairs = 0, repairBytes = 0, lastDiffer = -1, maxElements = 0;
	int nextA = 0, nextB = 0;
	for ( int tick = 0; tick < ticks; ++tick )
	{
		nextA = lpScriptPlay( a, script, nextA );
		nextB = lpScriptPlay( b, script, nextB );
		lpSceneBombard( a, scene, tick, period );
		lpSceneBombard( b, scene, tick, period );
		lpSceneDrive( a, scene, tick );
		lpSceneDrive( b, scene, tick );
		if ( tick == injectTick )
		{
			injected = Inject( b, warm, ulps );
			lpWorldTransform xf = { 0 };
			lpWorld_GetBodyTransform( b, injected, &xf );
			origin = xf.p;
			printf( "  tick %d: nudged body %d at (%.1f, %.1f, %.1f)\n", tick, injected, (double)origin.x, (double)origin.y,
					(double)origin.z );
		}
		lpWorld_Step( a, 1.0f / 60.0f, 4 );
		lpWorld_Step( b, 1.0f / 60.0f, 4 );
		if ( tick < injectTick || injected < 0 )
		{
			continue;
		}

		uint64_t sumsA[lp_hashCategoryCount], sumsB[lp_hashCategoryCount];
		lpWorld_HashCategories( a, sumsA );
		lpWorld_HashCategories( b, sumsB );
		bool differ = false, differVisible = false;
		for ( int c = 0; c < lp_hashCategoryCount; ++c )
		{
			differ = differ || sumsA[c] != sumsB[c];
			differVisible = differVisible || ( c != lp_hashBackend && sumsA[c] != sumsB[c] );
		}
		if ( differ == false )
		{
			continue;
		}
		lastDiffer = tick;
		int count = lpLab_Diff( a, b, diffs, capacity );
		maxElements = count > maxElements ? count : maxElements;
		int shown = count < capacity ? count : capacity;
		int bodyCapacity = lpWorld_GetBodyCapacity( a );
		if ( bodyCapacity > unitCapacity )
		{
			unitCapacity = bodyCapacity;
			units = realloc( units, sizeof( int ) * (size_t)unitCapacity );
			repairUnits = realloc( repairUnits, (size_t)unitCapacity );
		}
		int unitCount = lpWorld_GetUnits( a, units, unitCapacity );
		memset( repairUnits, 0, (size_t)unitCapacity );
		int touched = 0, bodies = 0;
		float farthest = 0.0f;
		for ( int k = 0; k < shown; ++k )
		{
			int unit = lpWorld_GetElementUnit( a, units, diffs[k].category, diffs[k].slot );
			if ( unit >= 0 && unit < unitCapacity && repairUnits[unit] == 0 )
			{
				repairUnits[unit] = 1;
				touched += 1;
			}
			int body = ElementBody( a, diffs[k].category, diffs[k].slot );
			lpWorldTransform xf;
			if ( body >= 0 && lpWorld_GetBodyTransform( a, body, &xf ) )
			{
				bodies += diffs[k].category == lp_hashBodies ? 1 : 0;
				lpVec3 d = lpSubPos( xf.p, origin );
				float distance = sqrtf( d.x * d.x + d.y * d.y + d.z * d.z );
				farthest = distance > farthest ? distance : farthest;
			}
		}
		if ( detected < 0 )
		{
			detected = tick;
			const lpLabDiff* first = diffs;
			printf( "  tick %d: detected (%d ticks after), %d elements differ; first %s %d (generation %u), in unit %d of %d\n",
					tick, tick - injectTick, count, lpHashCategoryName( first->category ), first->slot, first->generation,
					lpWorld_GetElementUnit( a, units, first->category, first->slot ), unitCount );
		}
		if ( differVisible && visible < 0 )
		{
			visible = tick;
			printf( "  tick %d: visible outside the engine's hidden state (%d ticks after)\n", tick, tick - injectTick );
		}
		if ( cone != NULL )
		{
			fprintf( cone, "%d %d %d %d %.2f\n", tick, count, bodies, touched, (double)farthest );
		}
		if ( parts != 0 && tick >= detected + repairDelay )
		{
			for ( int u = 0; u < unitCount; ++u )
			{
				repairBytes += repairUnits[u] ? lpLab_RepairUnit( b, a, units, u, parts ) : 0;
			}
			repairs += 1;
			if ( repairs <= 8 || repairs % 50 == 0 )
			{
				printf( "  tick %d: repaired %d units (%d elements differed); %d bytes so far\n", tick, touched, count, repairBytes );
			}
		}
	}

	bool same = lpWorld_Hash( a ) == lpWorld_Hash( b );
	printf( "twin result: detected %d, visible %d, last differing tick %d, at most %d elements, %d repairs, %d bytes, "
			"%s at the end\n",
			detected >= 0 ? detected - injectTick : -1, visible >= 0 ? visible - injectTick : -1, lastDiffer, maxElements, repairs,
			repairBytes, same ? "in sync" : "DIFFERENT" );
	if ( cone != NULL )
	{
		fclose( cone );
	}
	free( diffs );
	free( units );
	free( repairUnits );
	lpDestroyWorld( a );
	lpDestroyWorld( b );
	return injected < 0 || detected < 0 ? 1 : 0;
}
