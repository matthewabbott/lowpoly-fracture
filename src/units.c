// SPDX-License-Identifier: MIT
// Causal units (lpf.h): the groups of bodies whose members can affect each other within a tick. Union-find over body
// slots, joined through:
// - touching contacts, except at an anchored piece (it takes no load: its load goes to the ground) or frozen rubble
//   (static, and no stress solve reads it): they never move within a tick, so the ground, foundations and rubble are
//   a fixed boundary, not a join that would make the whole world one unit;
// - links (a grip is one), and every wheel of a vehicle and every limb of a rig with its torso (they share controls);
// - the ground under a wheel or a planted foot (they load it), with the same exception;
// - pieces that share a detonator (the first to go off disarms the rest) or a pool (they drain it together).
// A body holds its pieces, bonds and stress state, so a structure is in one unit, and a supply network crosses bodies
// only by links.
//
// What does not join, and so couples units unseen within a tick: casts and rays that only read what they hit (a
// ghost's landing, a tool's ray), and the world-wide budgets (fracture jobs, the stress share, debris ranks, freezes,
// free lists). Impacts and hits act at the next tick. Computed on demand, for the two-world lab; never on a step's path.

#include "world.h"

static int lpFind( int* parent, int i )
{
	while ( parent[i] != i )
	{
		parent[i] = parent[parent[i]];
		i = parent[i];
	}
	return i;
}

static void lpJoin( int* parent, int a, int b )
{
	if ( a < 0 || b < 0 )
	{
		return;
	}
	a = lpFind( parent, a );
	b = lpFind( parent, b );
	if ( a != b )
	{
		parent[a > b ? a : b] = a < b ? a : b; // the lower slot is the root: the numbering does not depend on the order
	}
}

static int lpPieceBody( const lpWorld* w, int piece )
{
	return piece >= 0 && piece < w->pieces.count ? w->pieces.data[piece].body : -1;
}

// The body a contact on this piece acts on within the tick, or -1 at the fixed boundary: an anchored piece, and frozen
// rubble (static, no stress solve: a hit wakes it, at the next tick)
static int lpLoadedBody( const lpWorld* w, int piece )
{
	int body = lpPieceBody( w, piece );
	bool fixed = body < 0 || w->pieces.data[piece].anchored || w->bodies.data[body].kind == lp_kindRubble;
	return fixed ? -1 : body;
}

static int lpLinkBody( const lpWorld* w, int link )
{
	if ( link < 0 || link >= w->links.count || w->links.data[link].alive == false )
	{
		return -1;
	}
	const lpLink* l = w->links.data + link;
	int body = lpPieceBody( w, l->ends[0].piece );
	return body >= 0 ? body : lpPieceBody( w, l->ends[1].piece );
}

static int lpVehicleBody( const lpWorld* w, int vehicle )
{
	const lpVehicle* v = w->vehicles.data + vehicle;
	for ( int k = 0; v->alive && k < v->wheelCount; ++k )
	{
		int body = lpLinkBody( w, v->links[k] );
		if ( body >= 0 )
		{
			return body;
		}
	}
	return -1;
}

// The first body with a piece of this detonator (shared: 1) or pool (2), index + 1 as the pieces store it
static int lpSharedBody( const lpWorld* w, int which, int index )
{
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		if ( p->body >= 0 && ( which == 1 ? p->detonator : p->pool ) == index + 1 )
		{
			return p->body;
		}
	}
	return -1;
}

int lpWorld_GetUnits( lpWorld* w, int* units, int capacity )
{
	int n = w->bodies.count;
	if ( capacity < n )
	{
		return -1;
	}
	int* parent = units;
	for ( int i = 0; i < n; ++i )
	{
		parent[i] = i;
	}

	for ( int i = 0; i < n; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		if ( b->alive == false || b->kind != lp_kindDebris || LP_PHYS_NULL( b->id ) )
		{
			continue; // every contact has a moving body; static ones are reached from it
		}
		const lpPhysContact* contacts;
		int count = lpPhys_GetBodyContacts( w->phys, b->id, &contacts );
		for ( int k = 0; k < count; ++k )
		{
			lpJoin( parent, i, lpLoadedBody( w, contacts[k].other ) ); // a moving body's own pieces are never anchored
		}
	}
	for ( int i = 0; i < w->links.count; ++i )
	{
		const lpLink* l = w->links.data + i;
		if ( l->alive )
		{
			lpJoin( parent, lpPieceBody( w, l->ends[0].piece ), lpPieceBody( w, l->ends[1].piece ) );
		}
	}
	for ( int i = 0; i < w->wheels.count; ++i )
	{
		const lpWheel* wh = w->wheels.data + i;
		if ( wh->link >= 0 && wh->grounded )
		{
			lpJoin( parent, lpLinkBody( w, wh->link ), lpLoadedBody( w, wh->groundPiece ) );
		}
	}
	for ( int i = 0; i < w->vehicles.count; ++i )
	{
		const lpVehicle* v = w->vehicles.data + i;
		for ( int k = 0; v->alive && k < v->wheelCount; ++k )
		{
			lpJoin( parent, lpVehicleBody( w, i ), lpLinkBody( w, v->links[k] ) );
		}
	}
	for ( int i = 0; i < w->rigs.count; ++i )
	{
		const lpRig* r = w->rigs.data + i;
		for ( int k = 0; r->alive && r->body >= 0 && k < r->limbCount; ++k )
		{
			const lpLimb* limb = r->limbs + k;
			int tip = limb->tipBody >= 0 && w->bodies.data[limb->tipBody].alive ? limb->tipBody : -1;
			lpJoin( parent, r->body, tip );
			lpJoin( parent, tip, lpLoadedBody( w, limb->groundPiece ) );
		}
	}
	// Pieces sharing a detonator or a pool: each joins the first body found with one
	int shared = w->detonators.count + w->pools.count;
	int* first = lpAlloc( sizeof( int ) * (size_t)( shared > 0 ? shared : 1 ) );
	for ( int k = 0; k < shared; ++k )
	{
		first[k] = -1;
	}
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		int keys[2] = { p->detonator > 0 ? p->detonator - 1 : -1, p->pool > 0 ? w->detonators.count + p->pool - 1 : -1 };
		for ( int j = 0; j < 2 && p->body >= 0; ++j )
		{
			if ( keys[j] >= 0 )
			{
				first[keys[j]] = first[keys[j]] < 0 ? p->body : first[keys[j]];
				lpJoin( parent, first[keys[j]], p->body );
			}
		}
	}
	lpFree( first );

	// Number the units in order of their lowest slot, which is each one's root: a root is numbered before its members
	for ( int i = 0; i < n; ++i )
	{
		parent[i] = lpFind( parent, i );
	}
	int count = 0;
	for ( int i = 0; i < n; ++i )
	{
		int root = parent[i];
		if ( w->bodies.data[i].alive == false )
		{
			units[i] = -1;
		}
		else if ( root == i )
		{
			units[i] = count;
			count += 1;
		}
		else
		{
			units[i] = units[root]; // already its number
		}
	}
	return count;
}

int lpWorld_GetElementUnit( const lpWorld* w, const int* units, int category, int slot )
{
	int body = -1;
	switch ( category )
	{
		case lp_hashBodies:
		case lp_hashStress:
		case lp_hashBackend:
			body = slot;
			break;
		case lp_hashPieces:
			body = lpPieceBody( w, slot );
			break;
		case lp_hashLinks:
			body = lpLinkBody( w, slot );
			break;
		case lp_hashVehicles:
			body = slot >= 0 && slot < w->vehicles.count ? lpVehicleBody( w, slot ) : -1;
			break;
		case lp_hashWheels:
			body = slot >= 0 && slot < w->wheels.count ? lpLinkBody( w, w->wheels.data[slot].link ) : -1;
			break;
		case lp_hashRigs:
			body = slot >= 0 && slot < w->rigs.count && w->rigs.data[slot].alive ? w->rigs.data[slot].body : -1;
			break;
		case lp_hashPools:
			body = slot >= 0 && slot < w->pools.count ? lpSharedBody( w, 2, slot ) : -1;
			break;
		case lp_hashDetonators:
			body = slot >= 0 && slot < w->detonators.count ? lpSharedBody( w, 1, slot ) : -1;
			break;
		default:
			break; // the world's own counters and queues are in no unit
	}
	return body >= 0 && body < w->bodies.count ? units[body] : -1;
}

void lpWorld_HashUnits( const lpWorld* w, const int* units, int count, uint64_t* sums )
{
	for ( int u = 0; u < count; ++u )
	{
		sums[u] = 0;
	}
	for ( int c = 0; c < lp_hashCategoryCount; ++c )
	{
		int slots = lpWorld_HashSlotCount( w, c );
		for ( int i = 0; i < slots; ++i )
		{
			int unit = lpWorld_GetElementUnit( w, units, c, i );
			if ( unit >= 0 && unit < count )
			{
				sums[unit] += lpWorld_HashElement( w, c, i );
			}
		}
	}
}
