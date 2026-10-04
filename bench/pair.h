// SPDX-License-Identifier: MIT
// lpf_bench --host / --join (pair.c): a lockstep session headless
#pragma once

#include "scenes.h"
#include "script.h"

typedef struct lpPairDef
{
	int scene;
	int period;
	int ticks;
	int workers;
	int delay;
	int hostPort;		  // >= 0: host on this port (0: any free one)
	bool loopback;		  // the host listens on 127.0.0.1 only
	int peers;			  // the host: peers to wait for
	const char* joinHost; // otherwise join this host
	int joinPort;
	const lpScript* script; // this machine's player (NULL: none)
	long long injectTick;	// nudge a body before this tick (-1: never)
	int netDelay, netJitter; // a joiner: its link held this long each way (ms; net.h's faults)
	long long stallTick;	 // a joiner: its link stalls for stallMs at this tick (-1: never)
	int stallMs;
	long long leaveTick; // this machine leaves the session at this tick (-1: never)
} lpPairDef;

// Runs the session; returns the exit code (0 in sync, 4 desync, 5 refused, 6 a machine left, 1 otherwise)
int lpBenchLockstep( const lpPairDef* def );
