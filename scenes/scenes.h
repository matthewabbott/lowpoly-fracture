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
	lp_sceneTrack,	// a ring road with kerbs, a hump, a plank bridge and a brick wall, three cars that drive laps, a crane
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

// Scripted drivers for scenes with vehicles (the track's cars drive laps), for benchmarks and demos: call every tick
// before the step. They read only simulation state, so they are deterministic. skipVehicle (-1: none) is left alone,
// for the player. Does nothing in scenes without vehicles.
void lpSceneDrive( lpWorld* world, int scene, int tick, int skipVehicle );

// The scenes' supply channels and part tags (the core never reads tags; these are the game's names)
enum
{
	lp_channelFuel = 0,
	lp_channelPower = 1, // fed by the engine, which needs fuel
	lp_channelSteer = 2, // fed by the steering box, which needs power
};

enum
{
	lp_tagFrame = 1,
	lp_tagEngine,
	lp_tagFuelTank,
	lp_tagSteering,
	lp_tagPanel,
	lp_tagGlass,
	lp_tagBumper,
};

// A car facing local +z at base (on the ground), turned by yaw: a sheet-metal floor pan carrying fuel, power and
// steering, an engine (feeds power, needs fuel), a fuel tank (feeds fuel, and goes off), a steering box (feeds
// steering, needs power), hood, boot, doors, pillars and roof bolted on, glass, rubber bumpers, and four wheels (rear
// drive, front steering, the handbrake on the rear). About 1.7 t. style picks the paint. Returns the vehicle.
int lpAddCar( lpWorld* world, b3Vec3 base, float yaw, int style );

// The crane's links, by lpLinkDef.userId
enum
{
	lp_linkSlew = 0xC4A1, // the deck on the mast: a motorised vertical hinge
	lp_linkLuff = 0xC4A2, // the jib on the deck: a motorised horizontal hinge
	lp_linkWinch = 0xC4A3, // the rope from the jib's tip to the load
};

// A tower crane at base: a steel base plate and a timber mast set in it (a structure, which carries the crane through
// its links), a slewing deck, a 10 m jib along +x and a winch rope to a steel load of loadMass kg. Returns the
// structure's body.
int lpAddCrane( lpWorld* world, b3Vec3 base, float loadMass );

// The ground plane, for tests that build their own scene
void lpAddGround( lpWorld* world, float halfSize );

#ifdef __cplusplus
}
#endif
