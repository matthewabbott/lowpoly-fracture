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
	lp_sceneRuins,	// a dry-stone arch, a colonnade with lintels, balconies: structures that know where they are weak
	lp_sceneYard,	// things joined by links: a cart of volatile crates on a ramp, a hanging sign, a door, a drawbridge
	lp_sceneKeep,	// a mortared stone keep of about 2000 pieces with wooden floors: the stress solve at scale
	lp_sceneCount
} lpSceneId;

const char* lpSceneName( int scene );

// Scene id from its name ("town") or number ("2")
int lpSceneFromName( const char* name );

// Adds everything for the scene to the world, including the ground, then settles its structures
// (lpWorld_SettleStructures).
void lpBuildScene( lpWorld* world, int scene );

// A square stone keep, 15 m across, with `floors` wooden floors (1 to 6; four make about 2000 pieces), its front door
// facing +z. Returns its body.
int lpAddKeep( lpWorld* world, b3Vec3 base, int floors );

// Scripted bombardment for benchmarks and demos: at some ticks, casts a ray from a moving attacker into the scene
// and queues a grenade (or every fourth time a cannon blast) where it hits. Deterministic for a deterministic world.
// Returns true if an impact was queued this tick.
bool lpSceneBombard( lpWorld* world, int scene, int tick, int period );

// The ground plane, for tests that build their own scene
void lpAddGround( lpWorld* world, float halfSize );

#ifdef __cplusplus
}
#endif
