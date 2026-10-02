// SPDX-License-Identifier: MIT
// Replay scripts: tick-stamped inputs (the sandbox's tools, a vehicle's and a rig's controls) read from text and
// applied to a world, so the sandbox, the benchmark and the tests replay the same session headless or not.
//
// One event per line, `#` starts a comment:
//   tick tool ox oy oz dx dy dz [n]           tool: rifle grenade cannon hammer ball flask pull blow (dir normalised)
//   tick drive vehicle throttle brake steer handbrake
//   tick walk rig forward strafe turn crouch
//   tick reach rig limb active x y z          a limb strikes at the point (active 1) or steps back into the gait (0)
//   tick grab rig limb                        the claw grabs what it touches, or lets go of what it holds
//   tick impact ox oy oz dx dy dz radius energy [impulse]   an impact of your own where the ray hits (dir normalised)
// pull: origin is the target and dir the grabbed point in the body frame, n the piece; blow: n ticks held (default 1).
// Events apply at the start of their tick, before the step, in file order.

#pragma once

#include "lpf/lpf.h"

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum lpScriptKind
{
	lp_scriptRifle,
	lp_scriptGrenade,
	lp_scriptCannon,
	lp_scriptHammer,
	lp_scriptBall,
	lp_scriptFlask,
	lp_scriptPull,
	lp_scriptBlow,
	lp_scriptToolCount,
	lp_scriptDrive = lp_scriptToolCount,
	lp_scriptWalk,
	lp_scriptReach,
	lp_scriptGrab,
	lp_scriptImpact,
} lpScriptKind;

typedef struct lpScriptEvent
{
	int64_t tick;
	int kind;		 // lpScriptKind
	lpVec3 origin;	 // tools: where it is fired from; pull: the target; reach: the point
	lpVec3 dir;		 // tools: the aim; pull: the grabbed point in the body frame
	int index;		 // pull: the piece; drive: the vehicle; walk, reach, grab: the rig
	int limb;		 // reach, grab
	bool active;	 // reach: strike, or step back
	lpVehicleControl control; // drive
	lpRigControl walk;		  // walk
	float radius;			  // impact: m
	float energy;			  // impact: J
	float impulse;			  // impact: N*s given to loose pieces
} lpScriptEvent;

typedef struct lpScript
{
	lpScriptEvent* events;
	int count;
	int capacity;
} lpScript;

// What replaying changes besides the world: the vehicle and the rig the events steer (the scene's drivers leave them
// alone), the claw's grip (a weld made by a grab event), and the next event to apply
typedef struct lpScriptState
{
	int playerVehicle;
	int playerRig;
	int grip;
	uint32_t gripGeneration;
	int gripLimb;
	int next;
} lpScriptState;

lpScriptState lpDefaultScriptState( void );

// Appends the events of one line (none for a comment or a blank line; a held blower gives one per tick). Returns false
// if the line names an unknown tool.
bool lpScriptParseLine( lpScript* script, const char* line );

// Appends a file's events. Returns false if it cannot be read.
bool lpScriptLoad( lpScript* script, const char* path );

void lpScriptFree( lpScript* script );

// One event as a script line (exact floats, %.9g), so a recording replays the session it came from
void lpScriptWrite( FILE* file, const lpScriptEvent* event );

// Applies one event now. Deterministic: depends only on the event, the state and the world.
void lpScriptApply( lpWorld* world, const lpScriptEvent* event, lpScriptState* state );

// Applies the script's events up to and including `tick` not yet applied
void lpScriptPlay( lpWorld* world, const lpScript* script, int64_t tick, lpScriptState* state );

#ifdef __cplusplus
}
#endif
