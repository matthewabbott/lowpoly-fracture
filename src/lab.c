// SPDX-License-Identifier: MIT
// The two-world lab (lpf.h): two worlds stepped on the same commands, a desync injected on purpose, found by following
// the state hash down, and repaired by copying one world's state into the other, unit by unit (units.c). For
// experiments (lpf_bench --twin, the tests): a game never calls it. A repair is state only: it moves bodies and sets
// their hidden state, so both worlds must hold the same bodies with the same pieces in the unit it repairs.

#include "world.h"

#include "lpf/lplab.h"

#include <math.h>
#include <string.h>

void lpLab_NudgeVelocity( lpWorld* w, int body, int ulps )
{
	if ( body < 0 || body >= w->bodies.count || w->bodies.data[body].alive == false )
	{
		return;
	}
	lpBody* b = w->bodies.data + body;
	lpVec3 v = LP_PHYS_NULL( b->id ) ? b->v : lpPhys_GetLinearVelocity( w->phys, b->id );
	float* c = fabsf( v.y ) > fabsf( v.x ) ? &v.y : &v.x; // the largest component: an ulp of a zero would be nothing
	c = fabsf( v.z ) > fabsf( *c ) ? &v.z : c;
	uint32_t bits;
	memcpy( &bits, c, 4 );
	bits += (uint32_t)ulps;
	memcpy( c, &bits, 4 );
	if ( LP_PHYS_NULL( b->id ) )
	{
		b->v = v;
	}
	else
	{
		lpPhys_SetLinearVelocity( w->phys, b->id, v );
	}
	lpHashMark( w, body );
}

bool lpLab_NudgeWarmStart( lpWorld* w, int body, int ulps )
{
	int other = -1;
	if ( lpPhys_NudgeContact( w->phys, body, ulps, &other ) == false )
	{
		return false;
	}
	lpHashMark( w, body );
	lpHashMark( w, other ); // the contact is in both moving bodies' hashes
	return true;
}

static uint32_t lpElementGeneration( const lpWorld* w, int category, int slot )
{
	if ( ( category == lp_hashBodies || category == lp_hashStress || category == lp_hashBackend ) && slot < w->bodies.count )
	{
		return w->bodies.data[slot].generation;
	}
	if ( category == lp_hashPieces && slot < w->pieces.count )
	{
		return w->pieces.data[slot].generation;
	}
	if ( category == lp_hashLinks && slot < w->links.count )
	{
		return w->links.data[slot].generation;
	}
	return 0;
}

int lpLab_Diff( const lpWorld* a, const lpWorld* b, lpLabDiff* out, int capacity )
{
	uint64_t sumsA[lp_hashCategoryCount], sumsB[lp_hashCategoryCount];
	lpWorld_HashCategories( a, sumsA );
	lpWorld_HashCategories( b, sumsB );
	int count = 0;
	for ( int c = 0; c < lp_hashCategoryCount; ++c )
	{
		if ( sumsA[c] == sumsB[c] )
		{
			continue;
		}
		int slots = lpMaxInt( lpWorld_HashSlotCount( a, c ), lpWorld_HashSlotCount( b, c ) );
		for ( int bucket = 0; 64 * bucket < slots; ++bucket )
		{
			if ( lpWorld_HashBucket( a, c, bucket ) == lpWorld_HashBucket( b, c, bucket ) )
			{
				continue; // what a host and a peer would compare next, over the wire
			}
			int end = lpMinInt( 64 * bucket + 64, slots );
			for ( int i = 64 * bucket; i < end; ++i )
			{
				if ( lpWorld_HashElement( a, c, i ) == lpWorld_HashElement( b, c, i ) )
				{
					continue;
				}
				if ( count < capacity )
				{
					out[count] = (lpLabDiff){ c, i, lpElementGeneration( a, c, i ) };
				}
				count += 1;
			}
		}
	}
	return count;
}

int lpLab_RepairUnit( lpWorld* dst, const lpWorld* src, const int* units, int unit, int parts )
{
	int n = lpMinInt( src->bodies.count, dst->bodies.count );
	uint8_t* members = lpAlloc( (size_t)( n > 0 ? n : 1 ) );
	int bytes = 0;
	for ( int i = 0; i < n; ++i )
	{
		const lpBody* from = src->bodies.data + i;
		lpBody* to = dst->bodies.data + i;
		members[i] = units[i] == unit && from->alive && to->alive && from->kind == to->kind ? 1 : 0;
		if ( members[i] == 0 )
		{
			continue;
		}
		if ( LP_PHYS_NULL( from->id ) != LP_PHYS_NULL( to->id ) )
		{
			members[i] = 0;
			continue;
		}
		if ( ( parts & lp_labMotion ) && LP_PHYS_NULL( from->id ) )
		{
			// A ghost or scrap: the core's own motion and landing plan
			to->com = from->com;
			to->q = from->q;
			to->v = from->v;
			to->omega = from->omega;
			to->planTicks = from->planTicks;
			to->landIn = from->landIn;
			to->sinkTicks = from->sinkTicks;
			to->landPoint = from->landPoint;
			to->landNormal = from->landNormal;
			bytes += (int)( sizeof( float ) * 19 + sizeof( int ) * 3 );
		}
		else if ( parts & lp_labMotion )
		{
			// Written only where it differs: moving a body to where it is still touches the engine's state (a teleport
			// refreshes its proxies, a velocity wakes it). Then awake or asleep as in src.
			lpWorldTransform xf, xfTo;
			lpVec3 v, omega, vTo, omegaTo;
			lpPhys_GetMotion( src->phys, from->id, &xf, &v, &omega );
			lpPhys_GetMotion( dst->phys, to->id, &xfTo, &vTo, &omegaTo );
			if ( memcmp( &xf, &xfTo, sizeof( xf ) ) != 0 )
			{
				lpPhys_SetTransform( dst->phys, to->id, xf );
			}
			if ( memcmp( &v, &vTo, sizeof( v ) ) != 0 )
			{
				lpPhys_SetLinearVelocity( dst->phys, to->id, v );
			}
			if ( memcmp( &omega, &omegaTo, sizeof( omega ) ) != 0 )
			{
				lpPhys_SetAngularVelocity( dst->phys, to->id, omega );
			}
			bool awake = lpPhys_IsAwake( src->phys, from->id );
			if ( lpPhys_IsAwake( dst->phys, to->id ) != awake )
			{
				lpPhys_SetAwake( dst->phys, to->id, awake );
			}
			bytes += (int)( sizeof( float ) * 13 + 1 );
		}
		if ( ( parts & lp_labSleep ) && LP_PHYS_NULL( from->id ) == false )
		{
			float sleep = lpPhys_GetSleepTime( src->phys, from->id );
			float sleepTo = lpPhys_GetSleepTime( dst->phys, to->id );
			if ( memcmp( &sleep, &sleepTo, sizeof( float ) ) != 0 )
			{
				lpPhys_SetSleepTime( dst->phys, to->id, sleep );
			}
			bytes += (int)sizeof( float );
		}
		lpHashMark( dst, i );
	}
	if ( parts & lp_labWarmStarts )
	{
		bytes += lpPhys_CopyContacts( dst->phys, src->phys, members, n );
	}
	lpFree( members );
	return bytes;
}
