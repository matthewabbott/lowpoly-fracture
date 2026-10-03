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
	lp_sceneMech,	// a hexapod mech on patrol round a yard: a step, rubble, a hump, crates, a brick wall, a parked car
	lp_sceneContraption, // a domino run that tips a stone onto a volatile vial by a brick wall, about 20 s after it starts
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
int lpAddKeep( lpWorld* world, lpVec3 base, int floors );

// Scripted bombardment for benchmarks and demos: at some ticks, a grenade (or every fourth time a cannon blast) where
// a ray from a moving attacker first hits a piece, submitted as the scene's command for this tick. Deterministic for a
// deterministic world. Returns true if it fired this tick.
bool lpSceneBombard( lpWorld* world, int scene, int tick, int period );

// Scripted drivers for scenes with vehicles and rigs (the track's cars drive laps, the mech patrols its yard, the
// crane swings), for benchmarks and demos: call every tick before the step. They read only simulation state and submit
// the scene's commands (LP_PEER_SCENE), so they are deterministic, and they leave alone what a player drives. Does
// nothing in scenes without either.
void lpSceneDrive( lpWorld* world, int scene, int tick );

// The objects the sandbox's tools throw, registered by lpBuildScene first, in this order (lp_commandSpawn)
enum
{
	lp_templateFlask = 0, // a chunky glass bottle that goes off when it lands hard
	lp_templateBall = 1,  // a metal cannonball
};

// The scenes' supply channels and part tags (the core never reads tags; these are the game's names)
enum
{
	lp_channelFuel = 0,
	lp_channelPower = 1, // fed by the engine, which needs fuel
	lp_channelSteer = 2, // fed by the steering box, which needs power
	lp_channelHydraulics = 3, // the mech's: fed by its reservoir, which needs power (from its reactor)
	lp_channelControl = 4,	  // the mech's: fed by its computer, which needs power
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
	lp_tagReactor,
	lp_tagReservoir,
	lp_tagComputer,
	lp_tagLeg,
};

// A car facing local +z at base (on the ground), turned by yaw: a sheet-metal floor pan carrying fuel, power and
// steering, an engine (feeds power, needs fuel), a fuel tank (feeds fuel, and goes off), a steering box (feeds
// steering, needs power), hood, boot, doors, pillars and roof bolted on, glass, rubber bumpers, and four wheels (rear
// drive, front steering, the handbrake on the rear). About 1.7 t. style picks the paint. Returns the vehicle.
int lpAddCar( lpWorld* world, lpVec3 base, float yaw, int style );

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
int lpAddCrane( lpWorld* world, lpVec3 base, float loadMass );

// The hexapod's hinges, by lpLinkDef.userId: lp_linkHexapod + 16 * leg + joint (joint 0 the hip's yaw, 1 the femur,
// 2 the knee; legs 0 to 2 the right front, middle and rear, 3 to 5 the left rear, middle and front)
enum
{
	lp_linkHexapod = 0x4E00,
};

// A car-sized hexapod mech standing at base (its feet on the ground), facing local +z turned by yaw: a sheet-metal
// torso (frame, deck, reactor, hydraulic reservoir, computer) of about 1.8 t and six legs of three segments (hip
// block, femur, tibia with a rubber foot) on motorised hinges, about 2.7 t in all, standing on them as a rig (its limbs
// in the order of the legs above). Its systems: the reactor feeds power, the reservoir hydraulics and the computer
// control (both need power); the frame, the legs and their hinges carry all three, and every servo needs hydraulics
// and control (unfed, it goes limp). The reservoir holds 100 of fluid: a cut line leaks until its valves close, 3 s
// later. The femur and knee jam as they are damaged. style picks the paint. Returns the rig.
int lpAddHexapod( lpWorld* world, lpVec3 base, float yaw, int style );

// The contraption's vial, by lpObjectDef.userId (what the run sets off at its end)
enum
{
	lp_userContraptionVial = 0xF1A5,
};

// The ground plane, for tests that build their own scene
void lpAddGround( lpWorld* world, float halfSize );

#ifdef __cplusplus
}
#endif
