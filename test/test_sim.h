// SPDX-License-Identifier: MIT
// A Box3D world plus an lpf world, shared by the world and debris tests.
#pragma once

#include "scenes.h"
#include "world.h"

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
	s.world = lpCreateWorld( &ld );
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
