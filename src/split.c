// SPDX-License-Identifier: MIT
// Connectivity: splitting a body into its connected components, each of which becomes structure, rubble or debris of
// the tier its volume calls for; then the stress solve for what stays a structure (stress.c).

#include "world.h"

#include <math.h>
#include <stdio.h>

static int lpSplitBody( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	body->dirty = false;
	if ( body->alive == false )
	{
		return 0;
	}
	if ( body->pieces.count == 0 )
	{
		lpDestroyBody( w, bodyIndex, false );
		return 0;
	}
	if ( body->splitChecked && body->splitTopology == body->topology )
	{
		return 0; // its pieces and bonds are as they were when there was nothing to split off
	}

	// Flood fill over live bonds. Components are listed in the order of their first piece in the body list.
	w->stamp += 1;
	int stamp = w->stamp;
	w->scratchQueue.count = 0;
	lpArray_Reserve( w->scratchQueue, body->pieces.count );

	w->scratchComponents.count = 0;

	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int seed = body->pieces.data[i];
		if ( w->pieces.data[seed].mark == stamp )
		{
			continue;
		}
		lpComponent fresh = { w->scratchQueue.count, 0, 0.0f, lpVec3_zero, false, 0, 0.0f };
		lpArray_Push( w->scratchComponents, fresh );
		lpComponent* c = w->scratchComponents.data + w->scratchComponents.count - 1;

		w->pieces.data[seed].mark = stamp;
		w->scratchQueue.data[w->scratchQueue.count++] = seed;
		for ( int head = c->first; head < w->scratchQueue.count; ++head )
		{
			int pi = w->scratchQueue.data[head];
			lpPiece* p = w->pieces.data + pi;
			c->count += 1;
			c->volume += p->shape->volume;
			if ( p->shape->volume > c->largest )
			{
				c->largest = p->shape->volume;
				c->material = p->material;
			}
			c->centroid = lpMulAdd( c->centroid, p->shape->volume, p->shape->centroid );
			c->anchored = c->anchored || p->anchored;
			for ( int k = 0; k < p->bonds.count; ++k )
			{
				lpBond* bond = w->bonds.data + p->bonds.data[k];
				int other = bond->a == pi ? bond->b : bond->a;
				if ( w->pieces.data[other].mark != stamp )
				{
					w->pieces.data[other].mark = stamp;
					w->scratchQueue.data[w->scratchQueue.count++] = other;
				}
			}
		}
		c->centroid = lpMulSV( 1.0f / c->volume, c->centroid );
	}

	lpComponent* components = w->scratchComponents.data;
	int componentCount = w->scratchComponents.count;

	if ( componentCount <= 1 && ( body->kind != lp_kindStructure || componentCount == 0 || components[0].anchored ) )
	{
		body->splitChecked = true;
		body->splitTopology = body->topology;
		return 0;
	}

	// Which components stay on this body
	bool structure = body->kind == lp_kindStructure;
	int keep = -1;
	if ( structure == false )
	{
		keep = 0;
		for ( int c = 1; c < componentCount; ++c )
		{
			if ( components[c].volume > components[keep].volume )
			{
				keep = c;
			}
		}
	}

	lpWorldTransform xf = lpPhys_GetTransform( w->phys, body->id );
	lpVec3 v = lpPhys_GetLinearVelocity( w->phys, body->id );
	lpVec3 omega = lpPhys_GetAngularVelocity( w->phys, body->id );
	lpVec3 localCenter = lpPhys_GetLocalCenter( w->phys, body->id );
	bool isDynamic = lpPhys_IsDynamic( w->phys, body->id );
	// A component's new body keeps this body's frame, so it starts with the velocity of the frame's origin: setting its
	// mass moves its centre and adds the spin's share there (the component's own velocity, counted once)
	lpVec3 originV = isDynamic ? lpAdd( v, lpCross( omega, lpRotateVector( xf.q, lpNeg( localCenter ) ) ) ) : lpVec3_zero;

	int movedAny = 0;
	for ( int c = 0; c < componentCount; ++c )
	{
		lpComponent* comp = components + c;
		bool stays = structure ? comp->anchored : ( c == keep );
		if ( stays )
		{
			continue;
		}

		// Frozen rubble resting on what just left must be able to fall
		lpAABB wakeBox = { comp->centroid, comp->centroid };
		for ( int k = 0; k < comp->count; ++k )
		{
			lpPiece* p = w->pieces.data + w->scratchQueue.data[comp->first + k];
			wakeBox.lowerBound = lpMin( wakeBox.lowerBound, p->shape->bounds.lowerBound );
			wakeBox.upperBound = lpMax( wakeBox.upperBound, p->shape->bounds.upperBound );
		}
		lpVec3 wakeCenter = lpToVec3( lpTransformWorldPoint( xf, lpAABB_Center( wakeBox ) ) );
		float wakeRadius = lpLength( lpAABB_Extents( wakeBox ) ) + 0.3f;
		lpArray_Push( w->pendingWakes, ( (lpWake){ wakeCenter, wakeRadius } ) );

		lpVec3 compV = v;
		if ( isDynamic )
		{
			lpVec3 r = lpRotateVector( xf.q, lpSub( comp->centroid, localCenter ) );
			compV = lpAdd( v, lpCross( omega, r ) );
		}
		lpVec3 compOmega = isDynamic ? omega : lpVec3_zero;

		// The piece's tier follows its size
		if ( comp->volume < lpParticleVolume( w, comp->material ) )
		{
			for ( int k = 0; k < comp->count; ++k )
			{
				int pi = w->scratchQueue.data[comp->first + k];
				lpPiece* p = w->pieces.data + pi;
				while ( p->bonds.count > 0 )
				{
					lpBreakBond( w, p->bonds.data[p->bonds.count - 1] );
				}
				lpDetachPieceShape( w, pi );
				lpEmitParticle( w, xf, p->shape->centroid, compV, lpCbrt( p->shape->volume ), p->material );
				lpFreePieceSlot( w, pi );
			}
		}
		else if ( comp->volume < lpGhostVolume( w, comp->material ) )
		{
			int ghost = lpBeginGhost( w, xf, compV, compOmega, w->bodies.data[bodyIndex].gravityScale );
			body = w->bodies.data + bodyIndex;
			for ( int k = 0; k < comp->count; ++k )
			{
				int pi = w->scratchQueue.data[comp->first + k];
				lpDetachPieceShape( w, pi );
				lpAddLoosePiece( w, ghost, pi );
			}
			lpFinishLoose( w, ghost, xf );
		}
		else
		{
			uint8_t tier = comp->volume < lpLightVolume( w, comp->material ) ? lp_tierLight : lp_tierFull;
			int newIndex = lpCreateBodyInternal( w, xf, true, lp_kindDebris, tier, originV, compOmega,
												 w->bodies.data[bodyIndex].gravityScale );
			body = w->bodies.data + bodyIndex; // array may have moved
			lpBody* nb = w->bodies.data + newIndex;
			nb->solveStress = body->solveStress;
			nb->inertiaRadius = body->inertiaRadius;
			for ( int k = 0; k < comp->count; ++k )
			{
				int pi = w->scratchQueue.data[comp->first + k];
				lpPiece* p = w->pieces.data + pi;
				lpDetachPieceShape( w, pi );
				p->anchored = false;
				p->body = -1;
				lpAttachPiece( w, pi, newIndex );
			}
			lpApplyMass( w, nb );
		}
		body = w->bodies.data + bodyIndex;
		body->volume -= comp->volume;
		movedAny += 1;
		w->stats.splitsThisStep += 1;
	}

	if ( movedAny == 0 )
	{
		body->splitChecked = true; // a structure whose anchored components all stay
		body->splitTopology = body->topology;
		return 0;
	}

	// Rebuild this body's piece list from the pieces that stayed
	body->topology += 1;
	int kept = 0;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		if ( w->pieces.data[pi].body == bodyIndex )
		{
			body->pieces.data[kept++] = pi;
		}
	}
	body->pieces.count = kept;

	if ( kept == 0 )
	{
		lpDestroyBody( w, bodyIndex, false );
	}
	else if ( isDynamic )
	{
		lpApplyMass( w, body );
		int material = w->pieces.data[body->pieces.data[0]].material;
		if ( body->volume < lpGhostVolume( w, material ) )
		{
			lpConvertToGhost( w, bodyIndex );
		}
		else if ( body->tier == lp_tierFull && body->volume < lpLightVolume( w, material ) )
		{
			lpConvertToLight( w, bodyIndex );
		}
	}
	return movedAny;
}

void lpUpdateBody( lpWorld* w, int bodyIndex )
{
	int moved = lpSplitBody( w, bodyIndex );

	// What is still a structure is anchored: queue it for the stress check, which runs once every body is split
	// (lpCheckStructures). What just came off may still rest on it; its weight shows up in the contacts after the next
	// physics step, so check again then.
	lpBody* body = w->bodies.data + bodyIndex;
	if ( body->alive && body->kind == lp_kindStructure )
	{
		lpArray_Push( w->stressQueue, bodyIndex );
		if ( moved > 0 )
		{
			lpArray_Push( w->stressAgain, bodyIndex );
		}
	}
	else if ( body->alive && body->solveStress && body->kind == lp_kindDebris && ( body->reloadLoads || body->solving ) )
	{
		lpArray_Push( w->stressQueue, bodyIndex ); // a moving body is checked only when asked (a hard hit, a landing)
	}
}

void lpUpdateDirtyBodies( lpWorld* w )
{
	// lpUpdateBody may create bodies but never marks new ones dirty, so the list does not grow under the loop
	for ( int i = 0; i < w->dirtyBodies.count; ++i )
	{
		lpUpdateBody( w, w->dirtyBodies.data[i] );
	}
	w->dirtyBodies.count = 0;
}
