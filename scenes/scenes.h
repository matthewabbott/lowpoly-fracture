// SPDX-License-Identifier: MIT
// Procedural low-poly scenes built from convex parts. Shared by the tests, the benchmark and the sandbox.

#pragma once

#include "lpf/lpf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum lpSceneId
{
	lp_sceneWall,  // brick and stone walls, a wooden fence, a glass pane
	lp_sceneHouse, // one house: brick walls, windows, wooden roof and posts
	lp_sceneTown,  // a street of houses, trees, fences and a tower
	lp_sceneTower, // a tall stone tower on a plaza
	lp_scenePile,  // loose crates and rocks dropped in a heap (physics stress)
	lp_sceneLumber, // logs, a log bridge, a woodpile, trees and a timber shed
	lp_sceneCount
} lpSceneId;

const char* lpSceneName( int scene );

// Adds everything for the scene to the world, including the ground.
void lpBuildScene( lpWorld* world, int scene );

// Scripted bombardment for benchmarks and demos: at some ticks, casts a ray from a moving attacker into the scene
// and queues a grenade (or every fourth time a cannon blast) where it hits. Deterministic for a deterministic world.
// Returns true if an impact was queued this tick.
bool lpSceneBombard( lpWorld* world, int scene, int tick, int period );

// Pieces of a scene: helpers the scenes are made of, exposed for tests
void lpAddGround( lpWorld* world, float halfSize );
void lpAddWall( lpWorld* world, b3Vec3 base, float yaw, float length, float height, float thickness, int material,
				uint32_t color, float panelWidth );
void lpAddHouse( lpWorld* world, b3Vec3 base, float yaw, uint64_t seed );
void lpAddTree( lpWorld* world, b3Vec3 base, float height, uint64_t seed );

// A round log lying along its local x axis (dynamic unless isStatic)
int lpAddLog( lpWorld* world, b3Vec3 center, float yaw, float length, float radius, bool isStatic );

#ifdef __cplusplus
}
#endif
