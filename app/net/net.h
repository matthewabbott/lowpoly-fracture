// SPDX-License-Identifier: MIT
// Transports for lockstep sessions (lockstep.h): text lines, one at a time, over TCP or in memory (for tests). Outside
// the core: the engine never sees a socket.
#pragma once

#include <stdbool.h>

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

// A listening socket on the port, or NULL (the reason on stderr)
lpTcp* lpTcp_Listen( int port );
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
void lpMemoryLink_Destroy( lpMemoryLink* link );

// Sleeps the thread (a pump loop's pause)
void lpNet_Sleep( int milliseconds );

#ifdef __cplusplus
}
#endif
