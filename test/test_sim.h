// SPDX-License-Identifier: MIT
// An lpf world with a scene, shared by the tests.
#pragma once

#include "scenes.h"
#include "script.h"
#include "world.h"

// Drift tests: every world clusters structures past this many pieces (0: the default), with each reduced solve checked
// against an exact one; the worst joint utilization error over all of them is kept
extern int lp_testLargeNodes;
extern float lp_testOracleWorst;
extern int lp_testOracleSolves;
extern int lp_testOracleFlips;
extern int lp_testOracleJoints;

typedef struct Sim
{
	lpWorld* world;
} Sim;

// scene < 0 builds only the ground
static inline Sim CreateSimDef( lpWorldDef ld, int scene )
{
	Sim s;
	ld.stressLargeNodes = lp_testLargeNodes > 0 ? lp_testLargeNodes : ld.stressLargeNodes;
	s.world = lpCreateWorld( &ld );
	s.world->stressOracle = lp_testLargeNodes > 0;
	if ( scene >= 0 )
	{
		lpBuildScene( s.world, scene );
	}
	else
	{
		lpAddGround( s.world, 40.0f );
	}
	return s;
}

static inline Sim CreateSim( int scene )
{
	return CreateSimDef( lpDefaultWorldDef(), scene );
}

static inline Sim CreateSimWorkers( int scene, int workers )
{
	lpWorldDef ld = lpDefaultWorldDef();
	ld.workerCount = workers;
	return CreateSimDef( ld, scene );
}

static inline void DestroySim( Sim* s )
{
	lp_testOracleWorst = s->world->oracleWorst > lp_testOracleWorst ? s->world->oracleWorst : lp_testOracleWorst;
	lp_testOracleSolves += s->world->oracleSolves;
	lp_testOracleFlips += s->world->oracleFlips;
	lp_testOracleJoints += s->world->oracleJoints;
	lpDestroyWorld( s->world );
}

// A body with a piece whose detonator is still armed; its trigger speed then goes to *triggerSpeed (if not NULL)
static inline bool BodyArmed( const lpWorld* w, int body, float* triggerSpeed )
{
	const lpBody* b = w->bodies.data + body;
	for ( int k = 0; k < b->pieces.count && b->alive; ++k )
	{
		int d = w->pieces.data[b->pieces.data[k]].detonator;
		if ( d != 0 && w->detonators.data[d - 1].armed )
		{
			if ( triggerSpeed != NULL )
			{
				*triggerSpeed = w->detonators.data[d - 1].def.triggerSpeed;
			}
			return true;
		}
	}
	return false;
}

// ---- measures shared by the tests ----

// Volume of everything still standing as structure, not counting the ground
static inline float StructureVolume( const lpWorld* w )
{
	float v = 0.0f;
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		if ( p->body >= 0 && p->material != lp_ground && w->bodies.data[p->body].kind == lp_kindStructure )
		{
			v += p->shape->volume;
		}
	}
	return v;
}

// Volume of everything loose: debris, rubble, ghosts and scrap
static inline float LooseVolume( const lpWorld* world )
{
	float v = 0.0f;
	for ( int i = 0; i < world->bodies.count; ++i )
	{
		const lpBody* b = world->bodies.data + i;
		if ( b->alive && b->kind != lp_kindStructure )
		{
			v += b->volume;
		}
	}
	return v;
}

// Volume-weighted world centroid of every piece of `material`
static inline lpVec3 MaterialCentroid( const lpWorld* w, int material )
{
	lpVec3 sum = lpVec3_zero;
	float total = 0.0f;
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		lpWorldTransform xf;
		if ( p->body < 0 || p->material != material || lpWorld_GetBodyTransform( w, p->body, &xf ) == false )
		{
			continue;
		}
		lpVec3 c = lpToVec3( lpTransformWorldPoint( xf, p->shape->centroid ) );
		sum = lpMulAdd( sum, p->shape->volume, c );
		total += p->shape->volume;
	}
	return total > 0.0f ? lpMulSV( 1.0f / total, sum ) : lpVec3_zero;
}

static inline float BodyY( const Sim* s, int body )
{
	lpWorldTransform xf;
	lpWorld_GetBodyTransform( s->world, body, &xf );
	return (float)xf.p.y;
}

// The fastest speed of any body's centre of mass (m/s): ghosts in flight count, static bodies and scrap do not. Under a
// few cm/s, everything is at rest.
static inline float MaxBodySpeed( const lpWorld* world )
{
	float fastest = 0.0f;
	for ( int i = 0; i < lpWorld_GetBodyCapacity( world ); ++i )
	{
		lpBodyInfo b = lpWorld_GetBodyInfo( world, i );
		float speed = b.alive ? lpLength( b.linearVelocity ) : 0.0f;
		fastest = speed > fastest ? speed : fastest;
	}
	return fastest;
}

// A replay script from scripts/ (LPF_ROOT is the repository, set by the build)
static inline bool LoadRepoScript( lpScript* script, const char* name )
{
	char path[512];
	snprintf( path, sizeof( path ), "%s/scripts/%s", LPF_ROOT, name );
	return lpScriptLoad( script, path );
}

static inline void Run( Sim* s, int ticks )
{
	for ( int i = 0; i < ticks; ++i )
	{
		lpWorld_Step( s->world, 1.0f / 60.0f, 4 );
	}
}
