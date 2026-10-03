// SPDX-License-Identifier: MIT
// Replay scripts: tick-stamped commands (lpf.h, "commands") read from text and submitted to a world, so the sandbox,
// the benchmark and the tests replay the same session headless or not. A recording is the commands a world applied,
// written back as lines.
//
// One command per line, `#` starts a comment. A line starts with its tick, or tick:peer (peer 0 otherwise); each peer's
// commands are numbered in file order, and a script is sorted by tick as it loads.
//   tick tool ox oy oz dx dy dz          tool: rifle grenade cannon hammer (an impact where the aim first hits), ball
//                                        flask (thrown from there): the sandbox's tools, the aim normalised
//   tick impact ox oy oz dx dy dz radius energy [impulse]   an impact of your own where the aim first hits (normalised)
//   tick ray ox oy oz dx dy dz radius energy impulse explosion range [piecesOnly]   the same, exact: as recordings
//                                        write it (piecesOnly 1: a ray that first hits a rope or a wheel does nothing)
//   tick point px py pz dx dy dz radius energy impulse explosion       an impact at a point
//   tick pull tx ty tz lx ly lz piece [generation [maxAccel maxMass]]  toward the target, the point lx..lz in the piece's
//                                        body frame, this tick (a held grab is a pull per tick)
//   tick spawn template px py pz qx qy qz qw vx vy vz wx wy wz
//   tick drive vehicle throttle brake steer handbrake
//   tick walk rig forward strafe turn crouch
//   tick reach rig limb active x y z     a limb strikes at the point (active 1) or steps back into the gait (0)
//   tick grab rig limb                   the claw toggles: grabs what it touches, or lets go (the hexapod's grip)
//   tick claw rig limb mode maxForce maxTorque strength   mode: 0 grab, 1 release, 2 toggle
//   tick release vehicle|rig index       the peer lets go: the scene's drivers take it back
// The tools are written back as the ray and spawn lines they stand for.

#pragma once

#include "lpf/lpf.h"

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum lpScriptTool
{
	lp_toolRifle,
	lp_toolGrenade,
	lp_toolCannon,
	lp_toolHammer,
	lp_toolBall,
	lp_toolFlask,
	lp_toolCount
} lpScriptTool;

typedef struct lpScript
{
	lpCommand* commands;
	int count;
	int capacity;
	uint32_t seq[LP_PEER_SCENE]; // the next number of each peer's commands
} lpScript;

// A tool fired from origin along dir (normalised here): an impact where the aim first hits, or the scene's flask or
// ball thrown (lp_templateFlask, lp_templateBall). Tick, peer and seq are the caller's.
lpCommand lpScriptToolCommand( int tool, lpVec3 origin, lpVec3 dir );

// Appends the command of one line (none for a comment or a blank line). Returns false if the line names an unknown
// command.
bool lpScriptParseLine( lpScript* script, const char* line );

// Appends a file's commands, then sorts the script by tick (stable: each tick keeps file order). Returns false if it
// cannot be read.
bool lpScriptLoad( lpScript* script, const char* path );

void lpScriptFree( lpScript* script );

// One command as a line (exact floats, %.9g), so a recording replays the session it came from. Writes nothing and
// returns false for the scene's commands (its drivers make them again) and for kinds with no line.
bool lpScriptWrite( FILE* file, const lpCommand* command );

// The same line into text (at most size bytes, NUL-terminated, no newline); returns its length, or 0 for a command
// with no line. Commands travel between machines this way (app/net).
int lpScriptFormat( char* text, int size, const lpCommand* command );

// The command of one line (its seq is 0: the caller's to set); false for a comment, a blank or an unknown line
bool lpScriptParseCommand( const char* line, lpCommand* command );

// Submits the script's commands from `next` on whose tick has come (lpWorld_GetTick); returns the next to submit
int lpScriptPlay( lpWorld* world, const lpScript* script, int next );

#ifdef __cplusplus
}
#endif
