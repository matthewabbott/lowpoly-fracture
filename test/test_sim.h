// SPDX-License-Identifier: MIT
// An lpf world with a scene, shared by the tests.
#pragma once

#include "scenes.h"
#include "world.h"

// Drift tests: every world clusters structures past this many pieces (0: the default), with each reduced solve checked
// against an exact one; the worst joint utilization error over all of them is kept
extern int lp_testLargeNodes;
extern int lp_testPhysicsLag;
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
	ld.physicsLag = lp_testPhysicsLag;
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

static inline void Run( Sim* s, int ticks )
{
	for ( int i = 0; i < ticks; ++i )
	{
		lpWorld_Step( s->world, 1.0f / 60.0f, 4 );
	}
}
