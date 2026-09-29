// SPDX-License-Identifier: MIT
// A Box3D world plus an lpf world, shared by the world and debris tests.
#pragma once

#include "scenes.h"
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
	b3WorldId physics;
	lpWorld* world;
} Sim;

// scene < 0 builds only the ground. Box3D gets the same worker count as the lpf world.
static inline Sim CreateSimDef( lpWorldDef ld, int scene )
{
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = (b3Vec3){ 0.0f, -10.0f, 0.0f };
	wd.workerCount = (uint32_t)( ld.workerCount > 1 ? ld.workerCount : 1 );
	Sim s;
	s.physics = b3CreateWorld( &wd );
	ld.physics = s.physics;
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
	b3DestroyWorld( s->physics );
}

static inline void Run( Sim* s, int ticks )
{
	for ( int i = 0; i < ticks; ++i )
	{
		lpWorld_Step( s->world, 1.0f / 60.0f, 4 );
	}
}
