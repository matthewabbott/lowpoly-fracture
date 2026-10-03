// SPDX-License-Identifier: MIT
// Lockstep co-op (milestone 10): every machine steps the same world on the same commands, and their state hashes agree.
//
// The host keeps the clock. It closes tick T once every machine (itself included) has stepped tick T - delay, and
// sends T's commands to everyone as one packet; every machine, the host too, steps only on packets, never on its own
// commands, and peers report their state hash after each step. A machine's commands for its player are stamped delay
// ticks ahead (lpLockstep_Command) and go to the host, which puts them in their tick's packet: by the time a machine
// has stepped T - delay, it has sent everything it will for T. The scene's own commands (its drivers, bombardment) are
// made on every machine from its own world, as LP_PEER_SCENE. A peer is welcome only if its session's description
// equals the host's (lpSessionCompare names the key that does not). A hash that differs stops the clock: once every
// machine has stepped all that was sent, the host follows the hash down over the wire (categories, buckets, elements),
// names the element and the first tick it differed, and stops everyone.
//
// Lines, one per line, in the C locale:
//   peer -> host   session <line> (each of its description's lines), then end
//   host -> peer   welcome <peer> <delay> | refuse <key>
//   peer -> host   cmd <seq> <script line> (scenes/script.h, tick:peer inside) | hash <tick> <root>
//   host -> all    tick <tick> <count>, then <count> lines: cmd <seq> <script line>
//   host -> peer   sums | buckets <category> | elements <category> <bucket>   (following a mismatch down)
//   peer -> host   sums <c0> .. <c10> | buckets <category> <count> <b0> .. | elements <category> <bucket> <e0> ..
//   host -> all    stop <reason>
#pragma once

#include "lpf/lpf.h"
#include "net.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LP_LOCKSTEP_MAX_PEERS 16

typedef enum lpLockstepState
{
	lp_lockstepJoining, // a peer: waiting for the host's welcome; the host: waiting for its peers
	lp_lockstepRunning,
	lp_lockstepDesync,	// stopped: a hash differed (the report names the element)
	lp_lockstepRefused, // a peer the host refused (the report names the key)
	lp_lockstepStopped, // the host stopped the session, or a connection closed
} lpLockstepState;

typedef struct lpLockstepDef
{
	lpWorld* world;
	const char* session; // this machine's description (lpSceneDescribeSession)
	int delay;			 // ticks between a player's command and the tick it applies at (at least 1)
	int peers;			 // the host: how many peers to wait for before the first tick
	float timeStep;
	int subSteps;
} lpLockstepDef;

typedef struct lpLockstep lpLockstep;

// Called before each step, with the tick about to be stepped (lpWorld_GetTick): the app submits the scene's own
// commands, and gives its player's to lpLockstep_Command
typedef void lpLockstepStepFcn( void* context, lpLockstep* lockstep, int64_t tick );

lpLockstep* lpLockstep_CreateHost( const lpLockstepDef* def );
lpLockstep* lpLockstep_CreatePeer( const lpLockstepDef* def, lpTransport host );
// The host: a connection accepted (it becomes a peer once its session is welcome)
void lpLockstep_AddPeer( lpLockstep* host, lpTransport peer );
void lpLockstep_Destroy( lpLockstep* lockstep );

// Reads what arrived, answers it, closes the ticks it can (the host, at most maxClose of them: pacing), and steps
// every tick it has a packet for (at most maxSteps); then flushes what it sent. Returns the steps taken.
int lpLockstep_Pump( lpLockstep* lockstep, lpLockstepStepFcn* before, void* context, int maxClose, int maxSteps );

// This machine's player's command, for the step being prepared (inside `before`): stamped tick + delay, with this
// machine's peer and its next seq, and sent to the host (a host keeps it for that tick's packet)
void lpLockstep_Command( lpLockstep* lockstep, const lpCommand* command );

lpLockstepState lpLockstep_GetState( const lpLockstep* lockstep );
int lpLockstep_GetPeer( const lpLockstep* lockstep );			 // 0 for the host, then 1, 2, ...
int64_t lpLockstep_GetClosed( const lpLockstep* lockstep );		 // the host: ticks sent; a peer: ticks received
int lpLockstep_GetPeerCount( const lpLockstep* lockstep );		 // the host: welcome peers
const char* lpLockstep_GetReport( const lpLockstep* lockstep ); // what stopped it ("" while running)
// The host: the first tick a peer's hash differed from its own (-1: none)
int64_t lpLockstep_GetDesyncTick( const lpLockstep* lockstep );
// The host: the last tick every peer has reported its hash for (each compared with its own)
int64_t lpLockstep_GetConfirmed( const lpLockstep* lockstep );
// The host: ends the session for everyone ("stop <reason>"); peers stop with that report
void lpLockstep_Stop( lpLockstep* host, const char* reason );

#ifdef __cplusplus
}
#endif
