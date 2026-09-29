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

#include "world.h"

#include <string.h>

static uint8_t lpQuantize( float strength )
{
	float s = b3ClampFloat( strength, 0.0f, 1.0f );
	return (uint8_t)( 255.0f * s + 0.5f );
}

// A source's feed: its share, times the worst supply among what it needs
static float lpSourceFeed( const lpPiece* p )
{
	float feed = p->sourceShare;
	for ( int n = 0; n < LP_CHANNELS; ++n )
	{
		if ( p->needs & ( 1u << n ) )
		{
			feed = b3MinFloat( feed, p->sourceShare * (float)p->supply[n] / 255.0f );
		}
	}
	return feed;
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
	for ( int c = 0; c < LP_CHANNELS; ++c )
	{
		uint8_t bit = (uint8_t)( 1u << c );
		int stamp = ++w->stamp; // marks the pieces found this channel
		for ( int i = 0; i < count; ++i )
		{
			w->pieces.data[w->scratchCarriers.data[i]].supply[c] = 0;
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
			while ( head < tail )
			{
				int pi = queue[head++];
				const lpPiece* p = w->pieces.data + pi;
				if ( p->sources & bit )
				{
					strength += lpSourceFeed( p );
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
				w->pieces.data[queue[k]].supply[c] = supply;
			}
		}
	}
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
			strength = b3MinFloat( strength, (float)p->supply[c] / 255.0f );
		}
	}
	return strength;
}
