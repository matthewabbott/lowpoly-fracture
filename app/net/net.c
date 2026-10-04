// SPDX-License-Identifier: MIT
// Transports (net.h): lines over non-blocking TCP sockets, and over an in-memory link for tests.

#if defined( _WIN32 )
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET lpSocket;
#define LP_BAD_SOCKET INVALID_SOCKET
#define lpCloseSocket closesocket
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
typedef int lpSocket;
#define LP_BAD_SOCKET ( -1 )
#define lpCloseSocket close
#endif

#include "net.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// A growable byte buffer
typedef struct lpBytes
{
	char* data;
	int count;
	int capacity;
} lpBytes;

static void lpBytes_Append( lpBytes* b, const char* data, int count )
{
	if ( b->count + count + 1 > b->capacity )
	{
		int capacity = b->capacity < 256 ? 256 : b->capacity;
		while ( capacity < b->count + count + 1 )
		{
			capacity *= 2;
		}
		b->data = realloc( b->data, (size_t)capacity );
		b->capacity = capacity;
	}
	memcpy( b->data + b->count, data, (size_t)count );
	b->count += count;
	b->data[b->count] = 0;
}

// Takes the first line out of the buffer into line (without its newline); false if no whole line is there
static bool lpBytes_TakeLine( lpBytes* b, lpBytes* line )
{
	char* end = b->count > 0 ? memchr( b->data, '\n', (size_t)b->count ) : NULL;
	if ( end == NULL )
	{
		return false;
	}
	int length = (int)( end - b->data );
	line->count = 0;
	lpBytes_Append( line, b->data, length > 0 && b->data[length - 1] == '\r' ? length - 1 : length );
	memmove( b->data, end + 1, (size_t)( b->count - length - 1 ) );
	b->count -= length + 1;
	return true;
}

void lpNet_Sleep( int milliseconds )
{
#if defined( _WIN32 )
	Sleep( (DWORD)milliseconds );
#else
	struct timespec t = { milliseconds / 1000, ( milliseconds % 1000 ) * 1000000L };
	nanosleep( &t, NULL );
#endif
}

uint64_t lpNet_Milliseconds( void )
{
#if defined( _WIN32 )
	LARGE_INTEGER count, frequency;
	QueryPerformanceCounter( &count );
	QueryPerformanceFrequency( &frequency );
	return (uint64_t)( count.QuadPart / ( frequency.QuadPart / 1000 ) );
#else
	struct timespec t;
	clock_gettime( CLOCK_MONOTONIC, &t );
	return (uint64_t)t.tv_sec * 1000u + (uint64_t)t.tv_nsec / 1000000u;
#endif
}

// ---- TCP ----

struct lpTcp
{
	lpSocket socket;
	bool listening;
	bool closed;
	lpBytes in, out, line;
};

static bool lpNetStarted;

static bool lpNetStart( void )
{
#if defined( _WIN32 )
	if ( lpNetStarted == false )
	{
		WSADATA data;
		if ( WSAStartup( MAKEWORD( 2, 2 ), &data ) != 0 )
		{
			fprintf( stderr, "net: WSAStartup failed\n" );
			return false;
		}
	}
#endif
	lpNetStarted = true;
	return true;
}

static void lpSetNonBlocking( lpSocket s )
{
#if defined( _WIN32 )
	u_long on = 1;
	ioctlsocket( s, FIONBIO, &on );
#else
	fcntl( s, F_SETFL, fcntl( s, F_GETFL, 0 ) | O_NONBLOCK );
#endif
}

static bool lpWouldBlock( void )
{
#if defined( _WIN32 )
	int e = WSAGetLastError();
	return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS;
#else
	return errno == EWOULDBLOCK || errno == EAGAIN || errno == EINPROGRESS;
#endif
}

static lpTcp* lpTcpWrap( lpSocket s, bool listening )
{
	lpTcp* t = calloc( 1, sizeof( lpTcp ) );
	t->socket = s;
	t->listening = listening;
	lpSetNonBlocking( s );
	if ( listening == false )
	{
		int on = 1;
		setsockopt( s, IPPROTO_TCP, TCP_NODELAY, (const char*)&on, sizeof( on ) );
#if defined( SO_NOSIGPIPE )
		setsockopt( s, SOL_SOCKET, SO_NOSIGPIPE, (const char*)&on, sizeof( on ) );
#endif
	}
	return t;
}

lpTcp* lpTcp_Listen( int port, bool loopback )
{
	if ( lpNetStart() == false )
	{
		return NULL;
	}
	lpSocket s = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );
	if ( s == LP_BAD_SOCKET )
	{
		fprintf( stderr, "net: no socket\n" );
		return NULL;
	}
	int on = 1;
#if defined( _WIN32 )
	// Winsock's SO_REUSEADDR would let a second listener share a busy port (a joiner could reach a stale process)
	setsockopt( s, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, (const char*)&on, sizeof( on ) );
#else
	setsockopt( s, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof( on ) );
#endif
	struct sockaddr_in address;
	memset( &address, 0, sizeof( address ) );
	address.sin_family = AF_INET;
	address.sin_addr.s_addr = htonl( loopback ? INADDR_LOOPBACK : INADDR_ANY );
	address.sin_port = htons( (unsigned short)port );
	if ( bind( s, (struct sockaddr*)&address, sizeof( address ) ) != 0 || listen( s, 8 ) != 0 )
	{
		fprintf( stderr, "net: cannot listen on port %d\n", port );
		lpCloseSocket( s );
		return NULL;
	}
	return lpTcpWrap( s, true );
}

int lpTcp_Port( const lpTcp* listener )
{
	struct sockaddr_in address;
	socklen_t length = sizeof( address );
	memset( &address, 0, sizeof( address ) );
	if ( getsockname( listener->socket, (struct sockaddr*)&address, &length ) != 0 )
	{
		return 0;
	}
	return (int)ntohs( address.sin_port );
}

lpTcp* lpTcp_Accept( lpTcp* listener )
{
	lpSocket s = accept( listener->socket, NULL, NULL );
	return s == LP_BAD_SOCKET ? NULL : lpTcpWrap( s, false );
}

lpTcp* lpTcp_Connect( const char* host, int port, int timeoutMs )
{
	if ( lpNetStart() == false )
	{
		return NULL;
	}
	struct addrinfo hints, *found = NULL;
	memset( &hints, 0, sizeof( hints ) );
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	char service[16];
	snprintf( service, sizeof( service ), "%d", port );
	if ( getaddrinfo( host, service, &hints, &found ) != 0 || found == NULL )
	{
		fprintf( stderr, "net: cannot resolve %s\n", host );
		return NULL;
	}
	// Retried until the host listens or the time is up (a pair started together)
	for ( int waited = 0; waited <= timeoutMs; waited += 100 )
	{
		lpSocket s = socket( AF_INET, SOCK_STREAM, IPPROTO_TCP );
		if ( s != LP_BAD_SOCKET && connect( s, found->ai_addr, (int)found->ai_addrlen ) == 0 )
		{
			freeaddrinfo( found );
			return lpTcpWrap( s, false );
		}
		if ( s != LP_BAD_SOCKET )
		{
			lpCloseSocket( s );
		}
		lpNet_Sleep( 100 );
	}
	freeaddrinfo( found );
	fprintf( stderr, "net: cannot connect to %s:%d\n", host, port );
	return NULL;
}

// A closed peer must not kill the process (SIGPIPE on POSIX)
#if defined( MSG_NOSIGNAL )
#define LP_SEND_FLAGS MSG_NOSIGNAL
#else
#define LP_SEND_FLAGS 0
#endif

static void lpTcpSend( void* context, const char* line )
{
	lpTcp* t = context;
	lpBytes_Append( &t->out, line, (int)strlen( line ) );
	lpBytes_Append( &t->out, "\n", 1 );
}

static void lpTcpFlush( void* context )
{
	lpTcp* t = context;
	int sent = 0;
	while ( sent < t->out.count && t->closed == false )
	{
		int n = (int)send( t->socket, t->out.data + sent, t->out.count - sent, LP_SEND_FLAGS );
		if ( n > 0 )
		{
			sent += n;
		}
		else if ( n < 0 && lpWouldBlock() )
		{
			lpNet_Sleep( 1 ); // the other end is slow to read: wait for room
		}
		else
		{
			t->closed = true;
		}
	}
	t->out.count = 0;
}

static const char* lpTcpReceive( void* context )
{
	lpTcp* t = context;
	if ( lpBytes_TakeLine( &t->in, &t->line ) )
	{
		return t->line.data;
	}
	char buffer[4096];
	for ( ;; )
	{
		int n = (int)recv( t->socket, buffer, sizeof( buffer ), 0 );
		if ( n > 0 )
		{
			lpBytes_Append( &t->in, buffer, n );
			continue;
		}
		if ( n == 0 || lpWouldBlock() == false )
		{
			t->closed = true; // the other end closed, or the connection broke
		}
		break;
	}
	return lpBytes_TakeLine( &t->in, &t->line ) ? t->line.data : NULL;
}

static bool lpTcpClosed( void* context )
{
	const lpTcp* t = context;
	return t->closed; // what is left is at most half a line: nothing more will come
}

lpTransport lpTcp_Transport( lpTcp* connection )
{
	lpTransport transport = { connection, lpTcpSend, lpTcpFlush, lpTcpReceive, lpTcpClosed };
	return transport;
}

void lpTcp_Close( lpTcp* t )
{
	if ( t == NULL )
	{
		return;
	}
	if ( t->listening == false )
	{
		lpTcpFlush( t );
	}
	lpCloseSocket( t->socket );
	free( t->in.data );
	free( t->out.data );
	free( t->line.data );
	free( t );
}

// ---- in memory ----

typedef struct lpMemoryEnd
{
	struct lpMemoryLink* link;
	int end;
} lpMemoryEnd;

struct lpMemoryLink
{
	lpBytes queued[2]; // bytes sent by each end, not yet flushed
	lpBytes wire[2];   // bytes on their way to each end
	lpBytes line[2];
	lpMemoryEnd ends[2];
	bool hungUp[2]; // that end closed
};

static void lpMemorySend( void* context, const char* line )
{
	lpMemoryEnd* e = context;
	lpBytes_Append( &e->link->queued[e->end], line, (int)strlen( line ) );
	lpBytes_Append( &e->link->queued[e->end], "\n", 1 );
}

static void lpMemoryFlush( void* context )
{
	lpMemoryEnd* e = context;
	lpBytes* from = &e->link->queued[e->end];
	if ( e->link->hungUp[0] == false && e->link->hungUp[1] == false )
	{
		lpBytes_Append( &e->link->wire[1 - e->end], from->data != NULL ? from->data : "", from->count );
	}
	from->count = 0;
}

static const char* lpMemoryReceive( void* context )
{
	lpMemoryEnd* e = context;
	return lpBytes_TakeLine( &e->link->wire[e->end], &e->link->line[e->end] ) ? e->link->line[e->end].data : NULL;
}

static bool lpMemoryClosed( void* context )
{
	const lpMemoryEnd* e = context;
	return e->link->hungUp[1 - e->end];
}

lpMemoryLink* lpMemoryLink_Create( void )
{
	lpMemoryLink* link = calloc( 1, sizeof( lpMemoryLink ) );
	for ( int k = 0; k < 2; ++k )
	{
		link->ends[k].link = link;
		link->ends[k].end = k;
	}
	return link;
}

lpTransport lpMemoryLink_End( lpMemoryLink* link, int end )
{
	lpTransport transport = { link->ends + end, lpMemorySend, lpMemoryFlush, lpMemoryReceive, lpMemoryClosed };
	return transport;
}

void lpMemoryLink_Close( lpMemoryLink* link, int end )
{
	link->hungUp[end] = true;
}

void lpMemoryLink_Destroy( lpMemoryLink* link )
{
	for ( int k = 0; k < 2; ++k )
	{
		free( link->queued[k].data );
		free( link->wire[k].data );
		free( link->line[k].data );
	}
	free( link );
}

// ---- faults ----

typedef struct lpHeldLine
{
	char* text;
	uint64_t due; // when it may pass
} lpHeldLine;

// Lines in order, each held until its time
typedef struct lpHeld
{
	lpHeldLine* data;
	int head, count, capacity;
	uint64_t last; // the latest due time given: no line passes the one before it
} lpHeld;

struct lpFault
{
	lpTransport inner;
	int delay, jitter;
	uint64_t stallUntil;
	uint32_t random;
	lpHeld out, in;
	char* line; // the line receive returned last
};

static void lpHeld_Push( lpHeld* h, const char* text, uint64_t due )
{
	if ( h->head > 0 && h->head >= h->count / 2 )
	{
		memmove( h->data, h->data + h->head, sizeof( lpHeldLine ) * (size_t)( h->count - h->head ) );
		h->count -= h->head;
		h->head = 0;
	}
	if ( h->count == h->capacity )
	{
		h->capacity = h->capacity < 16 ? 16 : 2 * h->capacity;
		h->data = realloc( h->data, sizeof( lpHeldLine ) * (size_t)h->capacity );
	}
	size_t n = strlen( text ) + 1;
	lpHeldLine* l = h->data + h->count++;
	l->text = malloc( n );
	memcpy( l->text, text, n );
	l->due = due > h->last ? due : h->last;
	h->last = l->due;
}

// The first line if its time has come (the caller frees it), else NULL
static char* lpHeld_Pop( lpHeld* h, uint64_t now )
{
	if ( h->head == h->count || h->data[h->head].due > now )
	{
		return NULL;
	}
	return h->data[h->head++].text;
}

static uint64_t lpFaultDue( lpFault* f, uint64_t now )
{
	f->random ^= f->random << 13;
	f->random ^= f->random >> 17;
	f->random ^= f->random << 5;
	uint64_t jitter = f->jitter > 0 ? f->random % (uint32_t)f->jitter : 0;
	return now + (uint64_t)f->delay + jitter;
}

static void lpFaultSend( void* context, const char* line )
{
	lpFault* f = context;
	lpHeld_Push( &f->out, line, lpFaultDue( f, lpNet_Milliseconds() ) );
}

static void lpFaultFlush( void* context )
{
	lpFault* f = context;
	uint64_t now = lpNet_Milliseconds();
	char* text;
	while ( now >= f->stallUntil && ( text = lpHeld_Pop( &f->out, now ) ) != NULL )
	{
		f->inner.send( f->inner.context, text );
		free( text );
	}
	f->inner.flush( f->inner.context );
}

static const char* lpFaultReceive( void* context )
{
	lpFault* f = context;
	uint64_t now = lpNet_Milliseconds();
	const char* arrived;
	while ( ( arrived = f->inner.receive( f->inner.context ) ) != NULL )
	{
		lpHeld_Push( &f->in, arrived, lpFaultDue( f, now ) );
	}
	free( f->line );
	f->line = now >= f->stallUntil ? lpHeld_Pop( &f->in, now ) : NULL;
	return f->line;
}

static bool lpFaultClosed( void* context )
{
	lpFault* f = context;
	return f->inner.closed( f->inner.context ) && f->in.head == f->in.count; // closed once what arrived has passed
}

lpFault* lpFault_Create( lpTransport inner )
{
	lpFault* f = calloc( 1, sizeof( lpFault ) );
	f->inner = inner;
	f->random = 0x9e3779b9u;
	return f;
}

lpTransport lpFault_Transport( lpFault* f )
{
	lpTransport transport = { f, lpFaultSend, lpFaultFlush, lpFaultReceive, lpFaultClosed };
	return transport;
}

void lpFault_SetDelay( lpFault* f, int delayMs, int jitterMs )
{
	f->delay = delayMs > 0 ? delayMs : 0;
	f->jitter = jitterMs > 0 ? jitterMs : 0;
}

void lpFault_Stall( lpFault* f, int milliseconds )
{
	f->stallUntil = lpNet_Milliseconds() + (uint64_t)( milliseconds > 0 ? milliseconds : 0 );
}

void lpFault_Destroy( lpFault* f )
{
	if ( f == NULL )
	{
		return;
	}
	char* text;
	while ( ( text = lpHeld_Pop( &f->out, UINT64_MAX ) ) != NULL )
	{
		f->inner.send( f->inner.context, text );
		free( text );
	}
	f->inner.flush( f->inner.context );
	while ( ( text = lpHeld_Pop( &f->in, UINT64_MAX ) ) != NULL )
	{
		free( text );
	}
	free( f->out.data );
	free( f->in.data );
	free( f->line );
	free( f );
}
