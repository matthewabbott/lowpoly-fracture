// SPDX-License-Identifier: MIT
// Lockstep co-op (lockstep.h): the host's clock, the packets, the hash reports, and following a mismatch down.

#include "lockstep.h"

#include "script.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LP_HISTORY 4096 // ticks of hashes kept, by tick modulo this

typedef struct lpLockPeer
{
	lpTransport transport;
	int peer; // its peer id once welcome (0: not yet)
	bool welcome;
	int64_t stepped; // the last tick it reported a hash for (-1: none)
	uint64_t hashes[LP_HISTORY];
	char* session; // its description, as it arrives
	int sessionLength;
} lpLockPeer;

typedef struct lpCommands
{
	lpCommand* data;
	int count;
	int capacity;
} lpCommands;

struct lpLockstep
{
	bool host;
	lpWorld* world;
	char* session;
	int delay;
	int expected;
	float timeStep;
	int subSteps;
	lpLockstepState state;
	char report[512];
	int peer;
	uint32_t seq;
	lpCommands inbox; // commands of the packets received, for the ticks not stepped yet

	// The host
	lpLockPeer* peers[LP_LOCKSTEP_MAX_PEERS];
	int peerCount;
	int64_t closed; // ticks below this were sent
	lpCommands pending;
	uint64_t own[LP_HISTORY];
	int64_t desyncTick;
	int desyncPeer;
	int descent; // 0: none; 1: waiting for every machine to step what was sent; 2: sums, 3: buckets, 4: elements asked
	int descentCategory;

	// A peer
	lpTransport link;
	int64_t received; // packets received for the ticks below this
	int64_t packetTick;
	int awaiting; // commands still to read of the packet being read
};

static void lpPush( lpCommands* list, const lpCommand* c )
{
	if ( list->count == list->capacity )
	{
		list->capacity = list->capacity < 16 ? 16 : 2 * list->capacity;
		list->data = realloc( list->data, sizeof( lpCommand ) * (size_t)list->capacity );
	}
	list->data[list->count++] = *c;
}

// A growable line being written
typedef struct lpText
{
	char* data;
	int count;
	int capacity;
} lpText;

static void lpTextAdd( lpText* t, const char* s )
{
	int n = (int)strlen( s );
	if ( t->count + n + 1 > t->capacity )
	{
		t->capacity = 2 * ( t->count + n + 1 ) + 64;
		t->data = realloc( t->data, (size_t)t->capacity );
	}
	memcpy( t->data + t->count, s, (size_t)n + 1 );
	t->count += n;
}

static void lpTextHex( lpText* t, uint64_t value )
{
	char word[24];
	snprintf( word, sizeof( word ), " %016llx", (unsigned long long)value );
	lpTextAdd( t, word );
}

static void lpStop( lpLockstep* ls, lpLockstepState state, const char* report )
{
	if ( ls->state == lp_lockstepDesync || ls->state == lp_lockstepRefused || ls->state == lp_lockstepStopped )
	{
		return; // the first reason stands
	}
	ls->state = state;
	snprintf( ls->report, sizeof( ls->report ), "%s", report );
}

static void lpSendCommand( lpTransport* t, const lpCommand* c )
{
	char line[512];
	int length = lpScriptFormat( line, (int)sizeof( line ), c );
	if ( length > 0 && length < (int)sizeof( line ) )
	{
		char wire[600];
		snprintf( wire, sizeof( wire ), "cmd %u %s", c->seq, line );
		t->send( t->context, wire );
	}
}

// "cmd <seq> <script line>" into a command; false if it is not one
static bool lpReadCommand( const char* line, lpCommand* c )
{
	if ( strncmp( line, "cmd ", 4 ) != 0 )
	{
		return false;
	}
	char* rest = NULL;
	unsigned long seq = strtoul( line + 4, &rest, 10 );
	if ( rest == NULL || *rest != ' ' || lpScriptParseCommand( rest + 1, c ) == false )
	{
		return false;
	}
	c->seq = (uint32_t)seq;
	return true;
}

static lpLockstep* lpCreate( const lpLockstepDef* def, bool host )
{
	lpLockstep* ls = calloc( 1, sizeof( lpLockstep ) );
	ls->host = host;
	ls->world = def->world;
	ls->session = malloc( strlen( def->session ) + 1 );
	memcpy( ls->session, def->session, strlen( def->session ) + 1 );
	ls->delay = def->delay > 1 ? def->delay : 1;
	ls->expected = def->peers;
	ls->timeStep = def->timeStep;
	ls->subSteps = def->subSteps;
	ls->state = lp_lockstepJoining;
	ls->desyncTick = -1;
	ls->closed = (int64_t)lpWorld_GetTick( def->world );
	ls->received = ls->closed;
	return ls;
}

lpLockstep* lpLockstep_CreateHost( const lpLockstepDef* def )
{
	lpLockstep* ls = lpCreate( def, true );
	if ( ls->expected <= 0 )
	{
		ls->state = lp_lockstepRunning;
	}
	return ls;
}

lpLockstep* lpLockstep_CreatePeer( const lpLockstepDef* def, lpTransport host )
{
	lpLockstep* ls = lpCreate( def, false );
	ls->link = host;
	// Its description, line by line, for the host to compare
	const char* line = ls->session;
	while ( *line != 0 )
	{
		const char* end = strchr( line, '\n' );
		int length = end != NULL ? (int)( end - line ) : (int)strlen( line );
		char wire[600];
		snprintf( wire, sizeof( wire ), "session %.*s", length, line );
		host.send( host.context, wire );
		line += length + ( end != NULL ? 1 : 0 );
	}
	host.send( host.context, "end" );
	host.flush( host.context );
	return ls;
}

void lpLockstep_AddPeer( lpLockstep* ls, lpTransport peer )
{
	if ( ls->peerCount == LP_LOCKSTEP_MAX_PEERS )
	{
		peer.send( peer.context, "refuse full" );
		peer.flush( peer.context );
		return;
	}
	lpLockPeer* p = calloc( 1, sizeof( lpLockPeer ) );
	p->transport = peer;
	p->stepped = (int64_t)lpWorld_GetTick( ls->world ) - 1;
	ls->peers[ls->peerCount++] = p;
}

void lpLockstep_Destroy( lpLockstep* ls )
{
	if ( ls == NULL )
	{
		return;
	}
	for ( int i = 0; i < ls->peerCount; ++i )
	{
		free( ls->peers[i]->session );
		free( ls->peers[i] );
	}
	free( ls->pending.data );
	free( ls->inbox.data );
	free( ls->session );
	free( ls );
}

void lpLockstep_Command( lpLockstep* ls, const lpCommand* command )
{
	lpCommand c = *command;
	c.tick = (int64_t)lpWorld_GetTick( ls->world ) + ls->delay;
	c.peer = (uint8_t)ls->peer;
	c.seq = ls->seq++;
	if ( ls->host )
	{
		lpPush( &ls->pending, &c );
	}
	else
	{
		lpSendCommand( &ls->link, &c );
	}
}

// ---- the host ----

static void lpBroadcast( lpLockstep* ls, const char* line )
{
	for ( int i = 0; i < ls->peerCount; ++i )
	{
		if ( ls->peers[i]->welcome )
		{
			ls->peers[i]->transport.send( ls->peers[i]->transport.context, line );
		}
	}
}

// A peer's hash for tick t against the host's own, once both are known
static void lpCheckHash( lpLockstep* ls, int index, int64_t t )
{
	const lpLockPeer* p = ls->peers[index];
	if ( p->stepped < t || (int64_t)lpWorld_GetTick( ls->world ) <= t || p->stepped - t >= LP_HISTORY )
	{
		return;
	}
	if ( p->hashes[t % LP_HISTORY] != ls->own[t % LP_HISTORY] && ls->desyncTick < 0 )
	{
		ls->desyncTick = t;
		ls->desyncPeer = index;
		ls->descent = 1; // the clock stops; once everyone has stepped what was sent, follow it down
	}
}

static void lpNameElement( lpLockstep* ls, int category, int slot )
{
	char what[160] = "";
	if ( category == lp_hashBodies || category == lp_hashStress || category == lp_hashBackend )
	{
		if ( slot < lpWorld_GetBodyCapacity( ls->world ) )
		{
			lpBodyInfo info = lpWorld_GetBodyInfo( ls->world, slot );
			snprintf( what, sizeof( what ), " (body %d, generation %u, %d pieces)", slot, info.generation, info.pieceCount );
		}
	}
	else if ( category == lp_hashPieces && slot < lpWorld_GetPieceCapacity( ls->world ) )
	{
		lpPieceInfo info = lpWorld_GetPieceInfo( ls->world, slot );
		snprintf( what, sizeof( what ), " (piece %d, generation %u, of body %d)", slot, info.generation, info.body );
	}
	char report[512];
	snprintf( report, sizeof( report ), "desync: first at tick %lld (peer %d); at tick %llu the %s element %d differs%s",
			  (long long)ls->desyncTick, ls->peers[ls->desyncPeer]->peer, (unsigned long long)lpWorld_GetTick( ls->world ),
			  lpHashCategoryName( category ), slot, what );
	lpStop( ls, lp_lockstepDesync, report );
	lpBroadcast( ls, "stop desync" );
}

// The descent's replies: categories, then the buckets of the first that differs, then that bucket's elements
static void lpDescend( lpLockstep* ls, const char* line )
{
	lpTransport* t = &ls->peers[ls->desyncPeer]->transport;
	char* cursor = NULL;
	if ( ls->descent == 2 && strncmp( line, "sums ", 5 ) == 0 )
	{
		uint64_t mine[lp_hashCategoryCount];
		lpWorld_HashCategories( ls->world, mine );
		cursor = (char*)line + 4;
		for ( int c = 0; c < lp_hashCategoryCount; ++c )
		{
			uint64_t theirs = strtoull( cursor, &cursor, 16 );
			if ( theirs != mine[c] )
			{
				char ask[64];
				snprintf( ask, sizeof( ask ), "buckets %d", c );
				t->send( t->context, ask );
				ls->descent = 3;
				ls->descentCategory = c;
				return;
			}
		}
		lpStop( ls, lp_lockstepDesync, "desync: the hashes differ, but no category does (a hash past the categories?)" );
		lpBroadcast( ls, "stop desync" );
	}
	else if ( ls->descent == 3 && strncmp( line, "buckets ", 8 ) == 0 )
	{
		int c = (int)strtol( line + 8, &cursor, 10 );
		int count = (int)strtol( cursor, &cursor, 10 );
		int mine = ( lpWorld_HashSlotCount( ls->world, c ) + 63 ) / 64;
		for ( int b = 0; b < ( count > mine ? count : mine ); ++b )
		{
			uint64_t theirs = b < count ? strtoull( cursor, &cursor, 16 ) : 0;
			if ( theirs != lpWorld_HashBucket( ls->world, c, b ) )
			{
				char ask[64];
				snprintf( ask, sizeof( ask ), "elements %d %d", c, b );
				t->send( t->context, ask );
				ls->descent = 4;
				return;
			}
		}
		lpStop( ls, lp_lockstepDesync, "desync: a category differs, but none of its buckets does" );
		lpBroadcast( ls, "stop desync" );
	}
	else if ( ls->descent == 4 && strncmp( line, "elements ", 9 ) == 0 )
	{
		int c = (int)strtol( line + 9, &cursor, 10 );
		int b = (int)strtol( cursor, &cursor, 10 );
		for ( int k = 0; k < 64; ++k )
		{
			uint64_t theirs = strtoull( cursor, &cursor, 16 );
			if ( theirs != lpWorld_HashElement( ls->world, c, 64 * b + k ) )
			{
				lpNameElement( ls, c, 64 * b + k );
				return;
			}
		}
		lpStop( ls, lp_lockstepDesync, "desync: a bucket differs, but none of its elements does" );
		lpBroadcast( ls, "stop desync" );
	}
}

static void lpHostRead( lpLockstep* ls, int index )
{
	lpLockPeer* p = ls->peers[index];
	lpTransport* t = &p->transport;
	const char* line;
	while ( ( line = t->receive( t->context ) ) != NULL )
	{
		lpCommand c;
		if ( strncmp( line, "session ", 8 ) == 0 )
		{
			int n = (int)strlen( line + 8 );
			p->session = realloc( p->session, (size_t)( p->sessionLength + n + 2 ) );
			memcpy( p->session + p->sessionLength, line + 8, (size_t)n );
			p->sessionLength += n;
			p->session[p->sessionLength++] = '\n';
			p->session[p->sessionLength] = 0;
		}
		else if ( strcmp( line, "end" ) == 0 && p->welcome == false )
		{
			char key[128];
			if ( lpSessionCompare( ls->session, p->session != NULL ? p->session : "", key, (int)sizeof( key ) ) == false )
			{
				char refuse[160];
				snprintf( refuse, sizeof( refuse ), "refuse %s", key );
				t->send( t->context, refuse );
				fprintf( stderr, "lockstep: a peer refused: its '%s' differs\n", key );
				continue;
			}
			int welcomed = 0;
			for ( int i = 0; i < ls->peerCount; ++i )
			{
				welcomed += ls->peers[i]->welcome ? 1 : 0;
			}
			p->welcome = true;
			p->peer = welcomed + 1;
			char welcome[64];
			snprintf( welcome, sizeof( welcome ), "welcome %d %d", p->peer, ls->delay );
			t->send( t->context, welcome );
		}
		else if ( p->welcome && lpReadCommand( line, &c ) )
		{
			c.peer = (uint8_t)p->peer; // a peer speaks for itself only
			if ( c.tick >= ls->closed )
			{
				lpPush( &ls->pending, &c );
			}
		}
		else if ( p->welcome && strncmp( line, "hash ", 5 ) == 0 )
		{
			char* cursor = NULL;
			int64_t tick = strtoll( line + 5, &cursor, 10 );
			uint64_t root = strtoull( cursor, NULL, 16 );
			p->stepped = tick;
			p->hashes[tick % LP_HISTORY] = root;
			lpCheckHash( ls, index, tick );
		}
		else if ( index == ls->desyncPeer && ls->descent >= 2 )
		{
			lpDescend( ls, line );
		}
	}
	if ( t->closed( t->context ) && ls->state == lp_lockstepRunning )
	{
		char report[64];
		snprintf( report, sizeof( report ), "peer %d left", p->peer );
		lpStop( ls, lp_lockstepStopped, report );
	}
}

// Tick X's packet: its commands, to every peer, and to the host's own inbox
static void lpClose( lpLockstep* ls, int64_t x )
{
	int count = 0;
	for ( int k = 0; k < ls->pending.count; ++k )
	{
		count += ls->pending.data[k].tick == x ? 1 : 0;
	}
	char head[64];
	snprintf( head, sizeof( head ), "tick %lld %d", (long long)x, count );
	lpBroadcast( ls, head );
	int kept = 0;
	for ( int k = 0; k < ls->pending.count; ++k )
	{
		lpCommand c = ls->pending.data[k];
		if ( c.tick != x )
		{
			ls->pending.data[kept++] = c;
			continue;
		}
		for ( int i = 0; i < ls->peerCount; ++i )
		{
			if ( ls->peers[i]->welcome )
			{
				lpSendCommand( &ls->peers[i]->transport, &c );
			}
		}
		lpPush( &ls->inbox, &c );
	}
	ls->pending.count = kept;
	ls->closed = x + 1;
}

// ---- a peer ----

static void lpAnswer( lpLockstep* ls, const char* line )
{
	lpText reply = { 0 };
	char* cursor = NULL;
	if ( strcmp( line, "sums" ) == 0 )
	{
		uint64_t sums[lp_hashCategoryCount];
		lpWorld_HashCategories( ls->world, sums );
		lpTextAdd( &reply, "sums" );
		for ( int c = 0; c < lp_hashCategoryCount; ++c )
		{
			lpTextHex( &reply, sums[c] );
		}
	}
	else if ( strncmp( line, "buckets ", 8 ) == 0 )
	{
		int c = (int)strtol( line + 8, NULL, 10 );
		int count = ( lpWorld_HashSlotCount( ls->world, c ) + 63 ) / 64;
		char head[64];
		snprintf( head, sizeof( head ), "buckets %d %d", c, count );
		lpTextAdd( &reply, head );
		for ( int b = 0; b < count; ++b )
		{
			lpTextHex( &reply, lpWorld_HashBucket( ls->world, c, b ) );
		}
	}
	else if ( strncmp( line, "elements ", 9 ) == 0 )
	{
		int c = (int)strtol( line + 9, &cursor, 10 );
		int b = (int)strtol( cursor, NULL, 10 );
		char head[64];
		snprintf( head, sizeof( head ), "elements %d %d", c, b );
		lpTextAdd( &reply, head );
		for ( int k = 0; k < 64; ++k )
		{
			lpTextHex( &reply, lpWorld_HashElement( ls->world, c, 64 * b + k ) );
		}
	}
	if ( reply.data != NULL )
	{
		ls->link.send( ls->link.context, reply.data );
		free( reply.data );
	}
}

static void lpPeerRead( lpLockstep* ls )
{
	lpTransport* t = &ls->link;
	const char* line;
	while ( ( line = t->receive( t->context ) ) != NULL )
	{
		lpCommand c;
		if ( strncmp( line, "welcome ", 8 ) == 0 )
		{
			char* cursor = NULL;
			ls->peer = (int)strtol( line + 8, &cursor, 10 );
			ls->delay = (int)strtol( cursor, NULL, 10 );
			ls->state = lp_lockstepRunning;
		}
		else if ( strncmp( line, "refuse ", 7 ) == 0 )
		{
			char report[256];
			snprintf( report, sizeof( report ), "refused: its '%s' differs from the host's", line + 7 );
			lpStop( ls, lp_lockstepRefused, report );
		}
		else if ( strncmp( line, "tick ", 5 ) == 0 )
		{
			char* cursor = NULL;
			ls->packetTick = strtoll( line + 5, &cursor, 10 );
			ls->awaiting = (int)strtol( cursor, NULL, 10 );
			ls->received = ls->awaiting == 0 ? ls->packetTick + 1 : ls->received;
		}
		else if ( ls->awaiting > 0 && lpReadCommand( line, &c ) )
		{
			lpPush( &ls->inbox, &c );
			ls->awaiting -= 1;
			ls->received = ls->awaiting == 0 ? ls->packetTick + 1 : ls->received;
		}
		else if ( strncmp( line, "stop ", 5 ) == 0 )
		{
			bool desync = strncmp( line + 5, "desync", 6 ) == 0;
			lpStop( ls, desync ? lp_lockstepDesync : lp_lockstepStopped, desync ? "desync: the host stopped the session" : line );
		}
		else
		{
			lpAnswer( ls, line );
		}
	}
	if ( t->closed( t->context ) )
	{
		lpStop( ls, lp_lockstepStopped, "the host left" );
	}
}

// ---- both ----

// The step for this tick: the app's commands, the packet's, the step, the hash
static void lpStepTick( lpLockstep* ls, lpLockstepStepFcn* before, void* context )
{
	int64_t tick = (int64_t)lpWorld_GetTick( ls->world );
	if ( before != NULL )
	{
		before( context, ls, tick );
	}
	int kept = 0;
	for ( int k = 0; k < ls->inbox.count; ++k )
	{
		const lpCommand* c = ls->inbox.data + k;
		if ( c->tick == tick )
		{
			lpWorld_Submit( ls->world, c );
		}
		else
		{
			ls->inbox.data[kept++] = *c;
		}
	}
	ls->inbox.count = kept;
	lpWorld_Step( ls->world, ls->timeStep, ls->subSteps );
	uint64_t root = lpWorld_Hash( ls->world );
	if ( ls->host )
	{
		ls->own[tick % LP_HISTORY] = root;
		for ( int i = 0; i < ls->peerCount; ++i )
		{
			if ( ls->peers[i]->welcome )
			{
				lpCheckHash( ls, i, tick );
			}
		}
	}
	else
	{
		char line[64];
		snprintf( line, sizeof( line ), "hash %lld %016llx", (long long)tick, (unsigned long long)root );
		ls->link.send( ls->link.context, line );
	}
}

int lpLockstep_Pump( lpLockstep* ls, lpLockstepStepFcn* before, void* context, int maxClose, int maxSteps )
{
	int steps = 0;
	if ( ls->host == false )
	{
		lpPeerRead( ls );
		while ( ls->state == lp_lockstepRunning && (int64_t)lpWorld_GetTick( ls->world ) < ls->received && steps < maxSteps )
		{
			lpStepTick( ls, before, context );
			steps += 1;
		}
		ls->link.flush( ls->link.context );
		return steps;
	}

	for ( int i = 0; i < ls->peerCount; ++i )
	{
		lpHostRead( ls, i );
	}
	int welcome = 0;
	for ( int i = 0; i < ls->peerCount; ++i )
	{
		welcome += ls->peers[i]->welcome ? 1 : 0;
	}
	if ( ls->state == lp_lockstepJoining && welcome >= ls->expected )
	{
		ls->state = lp_lockstepRunning;
	}

	// Close what every machine has stepped far enough for: its commands for the tick are all here
	int64_t now = (int64_t)lpWorld_GetTick( ls->world );
	for ( int closing = 0; ls->state == lp_lockstepRunning && ls->descent == 0 && closing < maxClose; ++closing )
	{
		int64_t frontier = now - 1;
		for ( int i = 0; i < ls->peerCount; ++i )
		{
			if ( ls->peers[i]->welcome && ls->peers[i]->stepped < frontier )
			{
				frontier = ls->peers[i]->stepped;
			}
		}
		if ( ls->closed > frontier + ls->delay )
		{
			break;
		}
		lpClose( ls, ls->closed );
	}
	while ( ls->state == lp_lockstepRunning && (int64_t)lpWorld_GetTick( ls->world ) < ls->closed && steps < maxSteps )
	{
		lpStepTick( ls, before, context );
		steps += 1;
	}

	// A mismatch: once every machine has stepped all that was sent, ask the peer for its categories
	if ( ls->descent == 1 && (int64_t)lpWorld_GetTick( ls->world ) == ls->closed )
	{
		bool caught = true;
		for ( int i = 0; i < ls->peerCount; ++i )
		{
			caught = caught && ( ls->peers[i]->welcome == false || ls->peers[i]->stepped == ls->closed - 1 );
		}
		if ( caught )
		{
			lpTransport* t = &ls->peers[ls->desyncPeer]->transport;
			t->send( t->context, "sums" );
			ls->descent = 2;
		}
	}
	for ( int i = 0; i < ls->peerCount; ++i )
	{
		ls->peers[i]->transport.flush( ls->peers[i]->transport.context );
	}
	return steps;
}

lpLockstepState lpLockstep_GetState( const lpLockstep* ls )
{
	return ls->state;
}

int lpLockstep_GetPeer( const lpLockstep* ls )
{
	return ls->peer;
}

int64_t lpLockstep_GetClosed( const lpLockstep* ls )
{
	return ls->host ? ls->closed : ls->received;
}

int lpLockstep_GetPeerCount( const lpLockstep* ls )
{
	int welcome = 0;
	for ( int i = 0; i < ls->peerCount; ++i )
	{
		welcome += ls->peers[i]->welcome ? 1 : 0;
	}
	return welcome;
}

const char* lpLockstep_GetReport( const lpLockstep* ls )
{
	return ls->report;
}

int64_t lpLockstep_GetDesyncTick( const lpLockstep* ls )
{
	return ls->desyncTick;
}

int64_t lpLockstep_GetConfirmed( const lpLockstep* ls )
{
	int64_t confirmed = (int64_t)lpWorld_GetTick( ls->world ) - 1;
	for ( int i = 0; i < ls->peerCount; ++i )
	{
		if ( ls->peers[i]->welcome && ls->peers[i]->stepped < confirmed )
		{
			confirmed = ls->peers[i]->stepped;
		}
	}
	return confirmed;
}

void lpLockstep_Stop( lpLockstep* ls, const char* reason )
{
	char line[256];
	snprintf( line, sizeof( line ), "stop %s", reason );
	lpBroadcast( ls, line );
	for ( int i = 0; i < ls->peerCount; ++i )
	{
		ls->peers[i]->transport.flush( ls->peers[i]->transport.context );
	}
	lpStop( ls, lp_lockstepStopped, reason );
}
