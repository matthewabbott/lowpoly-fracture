// SPDX-License-Identifier: MIT
// Supply: which pieces the sources of each channel reach.
//
// A part carries channels, feeds some of them (a source) and may need others fed first (lpPartSystem). A bond between
// two pieces that both carry a channel carries it, and so does a link that carries it between two pieces that do. The
// pieces a channel's carriers connect form a group; the group's supply is the sum of its sources' shares (each a
// share of its object's sources of that channel, by volume), capped at 1. A source feeds in proportion to how well
// what it needs is fed: needs are on lower channels than sources, so the channels are settled in order, 0 to 7.
//
// It is recomputed only when a carrier's connections changed (a bond or link between carriers made or broken, a
// carrier freed or made), at most once per step, in piece index order. Integer groups and a fixed summation order: it
// does not depend on anything but the simulation state.
//
// Pools: a source part may hold one (lpPartSystem.pool). Each update measures the carrier volume its lowest channel's
// group reaches; when that drops, a leak opens in proportion to the share lost (a severed leg, chips shot out of a
// line; a fracture that keeps every cell loses nothing), draining that share of the pool per second and closing over
// the part's seal time. A source feeds fully while its pool holds 30% or more, then less. Draining happens before the
// update each step; a pool that crosses a sixteenth of its capacity asks for one.

#include "world.h"

#include <string.h>


static uint8_t lpQuantize( float strength )
{
	float s = lpClampFloat( strength, 0.0f, 1.0f );
	return (uint8_t)( 255.0f * s + 0.5f );
}

// How hard a pool still pushes, 0 to 1
static float lpPoolPressure( const lpPool* pool )
{
	return lpClampFloat( pool->level / ( pool->pressure * pool->capacity ), 0.0f, 1.0f );
}

// A source's feed: its share, times the worst supply among what it needs, times its pool's pressure
static float lpSourceFeed( const lpWorld* w, const lpPiece* p )
{
	float feed = p->sourceShare;
	if ( p->pool != 0 )
	{
		feed *= lpPoolPressure( w->pools.data + p->pool - 1 );
	}
	for ( int n = 0; n < LP_CHANNELS; ++n )
	{
		if ( p->needs & ( 1u << n ) )
		{
			feed = lpMinFloat( feed, p->sourceShare * (float)p->supply[n] / 255.0f );
		}
	}
	return feed;
}

// The k-th neighbour of a carrier over what carries a channel both share: its bonds, then its links (-1: none there)
static int lpCarrierNeighbour( const lpWorld* w, int pi, int k )
{
	const lpPiece* p = w->pieces.data + pi;
	int other = -1;
	uint8_t carried = p->carries;
	if ( k < p->bonds.count )
	{
		const lpBond* bond = w->bonds.data + p->bonds.data[k];
		other = bond->a == pi ? bond->b : bond->a;
	}
	else
	{
		const lpLink* l = w->links.data + p->links.data[k - p->bonds.count];
		other = l->ends[0].piece == pi ? l->ends[1].piece : l->ends[0].piece;
		carried &= l->def.carries;
	}
	bool joined = other >= 0 && w->pieces.data[other].body >= 0 && ( w->pieces.data[other].carries & carried ) != 0;
	return joined ? other : -1;
}

static bool lpLiveSite( const lpWorld* w, int site )
{
	return site >= 0 && site < w->pieces.count && w->pieces.data[site].body >= 0 && w->pieces.data[site].carries != 0;
}

// Breadth first from the queue's first `tail` pieces (their dist set), over carriers: dist for the rest it reaches
// (those still at -1); comp, when given, takes the first's set
static void lpCarrierSearch( const lpWorld* w, int* queue, int tail, int* dist, int* comp )
{
	for ( int head = 0; head < tail; ++head )
	{
		int pi = queue[head];
		const lpPiece* p = w->pieces.data + pi;
		for ( int k = 0; k < p->bonds.count + p->links.count; ++k )
		{
			int other = lpCarrierNeighbour( w, pi, k );
			if ( other >= 0 && dist[other] < 0 )
			{
				dist[other] = dist[pi] + 1;
				if ( comp != NULL )
				{
					comp[other] = comp[pi];
				}
				queue[tail++] = other;
			}
		}
	}
}

// The supply wave (supplyHopsPerTick): how many steps after this one a change reaches each carrier, from the sites
// where carriers' connections changed. A carrier d hops from the nearest site, in a connected set of carriers whose
// sites lie within r hops of its first, gets (d + 2 r) / H: no site is further than d + 2 r from it, so nothing arrives
// before its cause could have. Into offset (by piece); -1 where no site reaches.
static void lpSupplyArrivals( lpWorld* w, int* offset )
{
	int n = w->pieces.count;
	int* comp = lpAlloc( sizeof( int ) * (size_t)( n > 0 ? n : 1 ) );
	int* queue = lpAlloc( sizeof( int ) * (size_t)( n > 0 ? n : 1 ) );
	LP_ARRAY( int ) spread = { 0 };
	for ( int i = 0; i < n; ++i )
	{
		offset[i] = -1; // distances first
		comp[i] = -1;
	}

	// Each connected set holding a site: the distances from its first site, and how far its other sites lie
	for ( int s = 0; s < w->supplySites.count; ++s )
	{
		int site = w->supplySites.data[s];
		if ( lpLiveSite( w, site ) == false || comp[site] >= 0 )
		{
			continue;
		}
		comp[site] = spread.count;
		offset[site] = 0;
		queue[0] = site;
		lpCarrierSearch( w, queue, 1, offset, comp );
		int r = 0;
		for ( int t = 0; t < w->supplySites.count; ++t )
		{
			int other = w->supplySites.data[t];
			r = lpLiveSite( w, other ) && comp[other] == comp[site] ? lpMaxInt( r, offset[other] ) : r;
		}
		lpArray_Push( spread, r );
	}

	// From every site at once: the nearest
	for ( int i = 0; i < n; ++i )
	{
		offset[i] = -1;
	}
	int tail = 0;
	for ( int s = 0; s < w->supplySites.count; ++s )
	{
		int site = w->supplySites.data[s];
		if ( lpLiveSite( w, site ) && offset[site] < 0 )
		{
			offset[site] = 0;
			queue[tail++] = site;
		}
	}
	lpCarrierSearch( w, queue, tail, offset, NULL );

	int hops = w->def.supplyHopsPerTick;
	for ( int i = 0; i < n; ++i )
	{
		offset[i] = comp[i] >= 0 && offset[i] >= 0 ? ( offset[i] + 2 * spread.data[comp[i]] ) / hops : -1;
	}
	lpArray_Free( spread );
	lpFree( comp );
	lpFree( queue );
}

static int lpCompareWave( const void* x, const void* y )
{
	const lpSupplyWave* a = x;
	const lpSupplyWave* b = y;
	if ( a->tick != b->tick )
	{
		return a->tick < b->tick ? -1 : 1;
	}
	if ( a->channel != b->channel )
	{
		return a->channel < b->channel ? -1 : 1;
	}
	if ( a->piece != b->piece )
	{
		return a->piece < b->piece ? -1 : 1;
	}
	return ( a->pool > b->pool ) - ( a->pool < b->pool );
}

void lpApplySupplyWaves( lpWorld* w )
{
	int due = 0;
	while ( due < w->supplyWaves.count && w->supplyWaves.data[due].tick <= w->tick )
	{
		const lpSupplyWave* wave = w->supplyWaves.data + due;
		if ( wave->piece >= 0 )
		{
			lpPiece* p = w->pieces.data + wave->piece;
			if ( p->body >= 0 && p->generation == wave->generation && p->supply[wave->channel] != wave->value )
			{
				p->supply[wave->channel] = wave->value;
				lpHashMarkPiece( w, wave->piece );
			}
		}
		else if ( wave->pool >= 0 && wave->pool < w->pools.count )
		{
			w->pools.data[wave->pool].leak += wave->leak;
		}
		due += 1;
	}
	if ( due > 0 )
	{
		memmove( w->supplyWaves.data, w->supplyWaves.data + due, sizeof( lpSupplyWave ) * (size_t)( w->supplyWaves.count - due ) );
		w->supplyWaves.count -= due;
	}
}

// With waves: a carrier keeps what it had where what the update found has not reached it yet, and the change goes on
// its way. One already on its way with the same value keeps its arrival; one the update undoes is dropped.
static void lpSendSupply( lpWorld* w, const uint8_t* before, const int* offset )
{
	uint8_t* sent = lpAlloc( (size_t)( w->pieces.count > 0 ? w->pieces.count : 1 ) );
	memset( sent, 0, (size_t)w->pieces.count );
	int kept = 0;
	for ( int k = 0; k < w->supplyWaves.count; ++k )
	{
		lpSupplyWave wave = w->supplyWaves.data[k];
		if ( wave.piece >= 0 )
		{
			lpPiece* p = w->pieces.data + wave.piece;
			bool same = p->body >= 0 && p->generation == wave.generation && p->carries != 0;
			if ( same == false || p->supply[wave.channel] != wave.value )
			{
				continue; // the piece is gone, or the update found something else (sent below)
			}
			p->supply[wave.channel] = before[LP_CHANNELS * wave.piece + wave.channel];
			sent[wave.piece] |= (uint8_t)( 1u << wave.channel );
		}
		w->supplyWaves.data[kept++] = wave;
	}
	w->supplyWaves.count = kept;
	for ( int i = 0; i < w->scratchCarriers.count; ++i )
	{
		int pi = w->scratchCarriers.data[i];
		lpPiece* p = w->pieces.data + pi;
		for ( int c = 0; c < LP_CHANNELS && offset[pi] > 0; ++c )
		{
			uint8_t now = before[LP_CHANNELS * pi + c];
			if ( ( sent[pi] & ( 1u << c ) ) != 0 || p->supply[c] == now )
			{
				continue;
			}
			lpSupplyWave wave = { w->tick + (uint64_t)offset[pi], pi, p->generation, -1, 0.0f, (uint8_t)c, p->supply[c] };
			lpArray_Push( w->supplyWaves, wave );
			p->supply[c] = now; // here when it arrives (at once where it is due now, or where no change site reaches)
		}
	}
	lpFree( sent );
}

void lpUpdateSupply( lpWorld* w )
{
	if ( w->supplyDirty == false )
	{
		return;
	}
	w->supplyDirty = false;
	w->stats.supplyUpdates += 1;

	// Only carriers are ever fed (a piece's channels never change, and a fresh slot starts unfed)
	w->scratchCarriers.count = 0;
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		if ( w->pieces.data[i].body >= 0 && w->pieces.data[i].carries != 0 )
		{
			lpArray_Push( w->scratchCarriers, i );
		}
	}
	int count = w->scratchCarriers.count;
	lpArray_Reserve( w->scratchQueue, count );
	for ( int i = 0; i < w->pools.count; ++i )
	{
		w->pools.data[i].found = 0.0f;
		w->pools.data[i].source = -1;
	}
	// What each carrier has before the update finds anew (with waves, the change reaches it later)
	bool waves = w->def.supplyHopsPerTick > 0;
	uint8_t* before = waves ? lpAlloc( (size_t)LP_CHANNELS * (size_t)( w->pieces.count > 0 ? w->pieces.count : 1 ) ) : NULL;
	for ( int i = 0; waves && i < count; ++i )
	{
		int pi = w->scratchCarriers.data[i];
		memcpy( before + LP_CHANNELS * pi, w->pieces.data[pi].supply, LP_CHANNELS );
	}
	for ( int c = 0; c < LP_CHANNELS; ++c )
	{
		uint8_t bit = (uint8_t)( 1u << c );
		int stamp = ++w->stamp; // marks the pieces found this channel
		for ( int i = 0; i < count; ++i )
		{
			int carrier = w->scratchCarriers.data[i];
			if ( w->pieces.data[carrier].supply[c] != 0 )
			{
				lpHashMarkPiece( w, carrier ); // it may not be reached again
			}
			w->pieces.data[carrier].supply[c] = 0;
		}
		for ( int n = 0; n < count; ++n )
		{
			int seed = w->scratchCarriers.data[n];
			lpPiece* root = w->pieces.data + seed;
			if ( root->body < 0 || ( root->carries & bit ) == 0 || root->mark == stamp )
			{
				continue;
			}
			// Flood the group over carrier bonds and links, summing its sources' feeds in the order they are found
			int* queue = w->scratchQueue.data;
			int head = 0, tail = 0;
			queue[tail++] = seed;
			root->mark = stamp;
			float strength = 0.0f;
			float volume = 0.0f;
			bool pooled = false;
			while ( head < tail )
			{
				int pi = queue[head++];
				const lpPiece* p = w->pieces.data + pi;
				volume += p->shape->volume;
				if ( p->sources & bit )
				{
					strength += lpSourceFeed( w, p );
					pooled = pooled || p->pool != 0;
				}
				for ( int k = 0; k < p->bonds.count; ++k )
				{
					const lpBond* bond = w->bonds.data + p->bonds.data[k];
					int other = bond->a == pi ? bond->b : bond->a;
					lpPiece* q = w->pieces.data + other;
					if ( ( q->carries & bit ) && q->mark != stamp )
					{
						q->mark = stamp;
						queue[tail++] = other;
					}
				}
				for ( int k = 0; k < p->links.count; ++k )
				{
					const lpLink* l = w->links.data + p->links.data[k];
					int other = l->ends[0].piece == pi ? l->ends[1].piece : l->ends[0].piece;
					if ( ( l->def.carries & bit ) == 0 || other < 0 )
					{
						continue;
					}
					lpPiece* q = w->pieces.data + other;
					if ( ( q->carries & bit ) && q->mark != stamp )
					{
						q->mark = stamp;
						queue[tail++] = other;
					}
				}
			}
			uint8_t supply = lpQuantize( strength );
			for ( int k = 0; k < tail; ++k )
			{
				lpPiece* p = w->pieces.data + queue[k];
				if ( p->supply[c] != supply )
				{
					lpHashMarkPiece( w, queue[k] );
				}
				p->supply[c] = supply;
				// A pool's reach is its lowest channel's group (the most a severed line can take from it)
				uint8_t lowest = (uint8_t)( p->sources & ( ~p->sources + 1u ) );
				if ( pooled && p->pool != 0 && lowest == bit )
				{
					lpPool* pool = w->pools.data + p->pool - 1;
					pool->found = lpMaxFloat( pool->found, volume );
					pool->source = pool->source < 0 ? queue[k] : pool->source;
				}
			}
		}
	}

	int* offset = NULL;
	if ( waves )
	{
		offset = lpAlloc( sizeof( int ) * (size_t)( w->pieces.count > 0 ? w->pieces.count : 1 ) );
		lpSupplyArrivals( w, offset );
		lpSendSupply( w, before, offset );
	}

	// Lines that lost volume leak (with waves, once the change reaches the pool)
	for ( int i = 0; i < w->pools.count; ++i )
	{
		lpPool* pool = w->pools.data + i;
		if ( pool->reach > 0.0f && pool->found < pool->reach )
		{
			float leak = pool->leakRate * pool->capacity * ( pool->reach - pool->found ) / pool->reach;
			int arrives = waves && pool->source >= 0 ? offset[pool->source] : 0;
			if ( arrives > 0 )
			{
				lpSupplyWave wave = { w->tick + (uint64_t)arrives, -1, 0, i, leak, 0, 0 };
				lpArray_Push( w->supplyWaves, wave );
			}
			else
			{
				pool->leak += leak;
			}
		}
		pool->reach = pool->found;
	}
	if ( waves && w->supplyWaves.count > 1 )
	{
		qsort( w->supplyWaves.data, (size_t)w->supplyWaves.count, sizeof( lpSupplyWave ), lpCompareWave );
	}
	w->supplySites.count = 0;
	lpFree( before );
	lpFree( offset );
}

void lpDrainPools( lpWorld* w, float timeStep )
{
	for ( int i = 0; i < w->pools.count; ++i )
	{
		lpPool* pool = w->pools.data + i;
		if ( pool->leak <= 0.0f )
		{
			continue;
		}
		w->stats.leakingPools += 1;
		pool->level = lpMaxFloat( pool->level - pool->leak * timeStep, 0.0f );
		if ( pool->seal > 0.0f )
		{
			pool->leak -= pool->leak * lpMinFloat( timeStep / pool->seal, 1.0f );
			pool->leak = pool->leak < 1e-6f * pool->capacity ? 0.0f : pool->leak;
		}
		if ( pool->level <= 0.0f )
		{
			pool->leak = 0.0f; // empty: nothing left to push
			lpCarriersChanged( w, 0xFF, pool->source, -1 );
		}
		int step = (int)( 16.0f * pool->level / pool->capacity );
		if ( step != pool->step )
		{
			pool->step = step;
			lpCarriersChanged( w, 0xFF, pool->source, -1 ); // what its source feeds changes there
		}
	}
}

float lpWorld_GetPiecePool( const lpWorld* w, int piece, float* leak )
{
	if ( leak != NULL )
	{
		*leak = 0.0f;
	}
	if ( piece < 0 || piece >= w->pieces.count || w->pieces.data[piece].body < 0 || w->pieces.data[piece].pool == 0 )
	{
		return -1.0f;
	}
	const lpPool* pool = w->pools.data + w->pieces.data[piece].pool - 1;
	if ( leak != NULL )
	{
		*leak = pool->leak / pool->capacity;
	}
	return pool->level / pool->capacity;
}

float lpWorld_GetPieceSupply( const lpWorld* w, int piece, int channel )
{
	if ( piece < 0 || piece >= w->pieces.count || channel < 0 || channel >= LP_CHANNELS || w->pieces.data[piece].body < 0 )
	{
		return 0.0f;
	}
	return (float)w->pieces.data[piece].supply[channel] / 255.0f;
}

uint8_t lpSuppliedMask( const lpPiece* p )
{
	uint8_t mask = 0;
	for ( int c = 0; c < LP_CHANNELS; ++c )
	{
		mask |= p->supply[c] > 0 ? (uint8_t)( 1u << c ) : 0u;
	}
	return mask;
}

float lpSupplyOf( const lpPiece* p, uint8_t channels )
{
	float strength = 1.0f;
	for ( int c = 0; c < LP_CHANNELS; ++c )
	{
		if ( channels & ( 1u << c ) )
		{
			strength = lpMinFloat( strength, (float)p->supply[c] / 255.0f );
		}
	}
	return strength;
}
