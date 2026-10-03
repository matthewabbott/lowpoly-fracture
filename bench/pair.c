// SPDX-License-Identifier: MIT
// lpf_bench --host / --join: a lockstep session headless (app/net). Each process builds the scene and runs its own
// bombardment and drivers; the host keeps the clock, and a script, if given, is this machine's player (its commands
// stamped --input-delay ticks ahead and sent through the host). The host ends the session once every peer has
// confirmed the last tick.
//
//   lpf_bench --scene town --period 12 --ticks 300 --host 7777 --peers 1
//   lpf_bench --scene town --period 12 --ticks 300 --join 127.0.0.1:7777 [--inject-desync 150]
//
// Exit codes: 0 in sync to the end, 4 a desync (the host names the element and the first tick), 5 refused (the
// session's descriptions differ: the key is named), 1 anything else (a connection lost, a timeout).

#include "pair.h"

#include "lockstep.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct lpPairMachine
{
	lpWorld* world;
	int scene;
	int period;
	const lpScript* script; // this machine's player
	int next;				// its next command
	int64_t injectTick;		// nudge a body at this tick (-1: never)
} lpPairMachine;

static void lpPairStep( void* context, lpLockstep* lockstep, int64_t tick )
{
	lpPairMachine* m = context;
	lpSceneBombard( m->world, m->scene, (int)tick, m->period );
	lpSceneDrive( m->world, m->scene, (int)tick );
	while ( m->script != NULL && m->next < m->script->count && m->script->commands[m->next].tick <= tick )
	{
		lpLockstep_Command( lockstep, m->script->commands + m->next ); // restamped tick + delay, this peer, its seq
		m->next += 1;
	}
	if ( tick == m->injectTick )
	{
		for ( int i = 0; i < lpWorld_GetBodyCapacity( m->world ); ++i )
		{
			lpBodyInfo info = lpWorld_GetBodyInfo( m->world, i );
			lpVec3 v = info.linearVelocity;
			if ( info.alive && info.kind == lp_kindDebris && info.awake && v.x * v.x + v.y * v.y + v.z * v.z > 0.01f )
			{
				lpLab_NudgeVelocity( m->world, i, 1 );
				printf( "injected: body %d's velocity nudged by one ulp before tick %lld\n", i, (long long)tick );
				break;
			}
		}
	}
}

int lpBenchLockstep( const lpPairDef* def )
{
	lpWorldDef ld = lpDefaultWorldDef();
	ld.workerCount = def->workers;
	lpWorld* world = lpCreateWorld( &ld );
	lpBuildScene( world, def->scene );
	char session[8192];
	lpSceneDescribeSession( world, def->scene, def->period, 1.0f / 60.0f, 4, session, (int)sizeof( session ) );
	lpLockstepDef ls = { world, session, def->delay, def->peers, 1.0f / 60.0f, 4 };
	lpPairMachine machine = { world, def->scene, def->period, def->script, 0, def->injectTick };

	lpTcp* listener = NULL;
	lpTcp* connections[LP_LOCKSTEP_MAX_PEERS] = { 0 };
	int connected = 0;
	lpLockstep* lockstep = NULL;
	if ( def->hostPort > 0 )
	{
		listener = lpTcp_Listen( def->hostPort );
		if ( listener == NULL )
		{
			lpDestroyWorld( world );
			return 1;
		}
		lockstep = lpLockstep_CreateHost( &ls );
		printf( "hosting %s on port %d for %d peer(s), input delay %d\n", lpSceneName( def->scene ), def->hostPort, def->peers, def->delay );
	}
	else
	{
		lpTcp* host = lpTcp_Connect( def->joinHost, def->joinPort, 10000 );
		if ( host == NULL )
		{
			lpDestroyWorld( world );
			return 1;
		}
		connections[connected++] = host;
		lockstep = lpLockstep_CreatePeer( &ls, lpTcp_Transport( host ) );
		printf( "joined %s:%d\n", def->joinHost, def->joinPort );
	}
	fflush( stdout );

	int code = 1;
	int idle = 0;
	for ( ;; )
	{
		if ( listener != NULL && connected < LP_LOCKSTEP_MAX_PEERS )
		{
			lpTcp* c = lpTcp_Accept( listener );
			if ( c != NULL )
			{
				connections[connected++] = c;
				lpLockstep_AddPeer( lockstep, lpTcp_Transport( c ) );
			}
		}
		bool sending = def->hostPort > 0 && lpLockstep_GetClosed( lockstep ) < def->ticks;
		int steps = lpLockstep_Pump( lockstep, lpPairStep, &machine, sending ? 64 : 0, 64 );
		lpLockstepState state = lpLockstep_GetState( lockstep );
		if ( state == lp_lockstepDesync )
		{
			code = 4;
			break;
		}
		if ( state == lp_lockstepRefused )
		{
			code = 5;
			break;
		}
		if ( state == lp_lockstepStopped )
		{
			code = strcmp( lpLockstep_GetReport( lockstep ), "stop done" ) == 0 ? 0 : 1;
			break;
		}
		if ( def->hostPort > 0 && sending == false && (int64_t)lpWorld_GetTick( world ) == def->ticks &&
			 lpLockstep_GetConfirmed( lockstep ) == def->ticks - 1 && lpLockstep_GetDesyncTick( lockstep ) < 0 )
		{
			lpLockstep_Stop( lockstep, "done" );
			code = 0;
			break;
		}
		idle = steps > 0 ? 0 : idle + 1;
		if ( idle > 0 )
		{
			lpNet_Sleep( 1 );
		}
		if ( idle > 30000 )
		{
			fprintf( stderr, "lockstep: nothing for 30 s; giving up\n" );
			break;
		}
	}
	printf( "%s at tick %llu, hash %016llx%s%s\n", code == 0 ? "in sync" : "stopped", (unsigned long long)lpWorld_GetTick( world ),
			(unsigned long long)lpWorld_Hash( world ), lpLockstep_GetReport( lockstep )[0] != 0 ? ": " : "",
			lpLockstep_GetReport( lockstep ) );
	lpLockstep_Destroy( lockstep );
	for ( int i = 0; i < connected; ++i )
	{
		lpTcp_Close( connections[i] );
	}
	lpTcp_Close( listener );
	lpDestroyWorld( world );
	return code;
}
