// SPDX-License-Identifier: MIT
// Transports for lockstep sessions (lockstep.h): text lines, one at a time, over TCP or in memory (for tests). Outside
// the core: the engine never sees a socket.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// A connection as lines: send queues one (its newline added), flush puts what is queued on the wire, receive returns
// the next whole line that arrived (without its newline; valid until the next receive) or NULL, and closed is true
// once the other end is gone.
typedef struct lpTransport
{
	void* context;
	void ( *send )( void* context, const char* line );
	void ( *flush )( void* context );
	const char* ( *receive )( void* context );
	bool ( *closed )( void* context );
} lpTransport;

// ---- TCP (Winsock on Windows, BSD sockets elsewhere): non-blocking, no delay ----

typedef struct lpTcp lpTcp;

// A listening socket on the port (0: any free one, which lpTcp_Port names), on every interface or on loopback only,
// or NULL (the reason on stderr). No other socket may share the port while it listens.
lpTcp* lpTcp_Listen( int port, bool loopback );
int lpTcp_Port( const lpTcp* listener );
// A connection waiting on a listener, or NULL if none is
lpTcp* lpTcp_Accept( lpTcp* listener );
// A connection to host:port, waiting up to timeoutMs for it, or NULL
lpTcp* lpTcp_Connect( const char* host, int port, int timeoutMs );
lpTransport lpTcp_Transport( lpTcp* connection );
void lpTcp_Close( lpTcp* socket );

// ---- in memory: two ends of one connection, for tests ----

typedef struct lpMemoryLink lpMemoryLink;

lpMemoryLink* lpMemoryLink_Create( void );
lpTransport lpMemoryLink_End( lpMemoryLink* link, int end ); // end 0 or 1
// That end hangs up: the other end's closed() turns true (what was flushed before still arrives)
void lpMemoryLink_Close( lpMemoryLink* link, int end );
void lpMemoryLink_Destroy( lpMemoryLink* link );

// ---- faults: a transport whose lines are held back, both ways, for tests of a slow or stalled network ----
//
// Each line is held for delay plus up to jitter milliseconds (from a fixed seed), and never overtakes the line before
// it: TCP keeps order, and the lockstep protocol counts on it. A stall holds everything, both ways, for a while. Lines
// are never lost: the protocol assumes a connection that delivers or closes. The wrapper must be pumped (flush,
// receive) for held lines to move; lockstep's pump does both every call.

typedef struct lpFault lpFault;

lpFault* lpFault_Create( lpTransport inner );
lpTransport lpFault_Transport( lpFault* fault );
void lpFault_SetDelay( lpFault* fault, int delayMs, int jitterMs ); // each way
void lpFault_Stall( lpFault* fault, int milliseconds );			  // from now
// Sends what is still held (a last flush), then frees it; the inner transport is the caller's
void lpFault_Destroy( lpFault* fault );

// Sleeps the thread (a pump loop's pause)
void lpNet_Sleep( int milliseconds );
// A monotonic clock, for pacing and faults (never the simulation's)
uint64_t lpNet_Milliseconds( void );

#ifdef __cplusplus
}
#endif
