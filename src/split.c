// SPDX-License-Identifier: MIT
// Connectivity: the weight check for static structures, and splitting a body into its connected components, each of
// which becomes structure, rubble or debris of the tier its volume calls for.

#include "world.h"

#include <math.h>
#include <stdio.h>

// Weight check for a static structure. A BFS from the anchored pieces gives every piece a bond distance to the
// ground. Loads then flow from the top down: each piece splits its own weight plus everything resting on it across
// all its bonds to pieces one step closer to the ground, in proportion to bond area, so parallel supports share the
// load like a real wall. A bond whose share exceeds area * health * stressScale breaks and its share moves to the
// piece's other supports. One pass per step; the caller re-checks the body next step if anything broke, so an
// undermined structure comes down progressively. Returns the number of broken bonds.
static int lpStressPass( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	int n = body->pieces.count;
	if ( w->def.stressScale <= 0.0f || n < 2 )
	{
		return 0;
	}

	// BFS from all anchored pieces (in body order)
	w->stamp += 1;
	int stamp = w->stamp;
	w->scratchQueue.count = 0;
	lpArray_Reserve( w->scratchQueue, n );
	lpArray_Reserve( w->scratchLoad, n );
	for ( int i = 0; i < n; ++i )
	{
		int pi = body->pieces.data[i];
		lpPiece* p = w->pieces.data + pi;
		p->groundDepth = -1; // reused here as the BFS depth
		if ( p->anchored )
		{
			p->mark = stamp;
			p->groundDepth = 0;
			w->scratchQueue.data[w->scratchQueue.count++] = pi;
		}
	}
	for ( int head = 0; head < w->scratchQueue.count; ++head )
	{
		int pi = w->scratchQueue.data[head];
		lpPiece* p = w->pieces.data + pi;
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			lpBond* bond = w->bonds.data + p->bonds.data[k];
			int other = bond->a == pi ? bond->b : bond->a;
			lpPiece* q = w->pieces.data + other;
			if ( q->mark != stamp )
			{
				q->mark = stamp;
				q->groundDepth = p->groundDepth + 1;
				w->scratchQueue.data[w->scratchQueue.count++] = other;
			}
		}
	}

	// Loads in newtons, flowing from the deepest pieces toward the ground (reverse BFS order)
	int reached = w->scratchQueue.count;
	float g = b3Length( b3World_GetGravity( w->def.physics ) );
	for ( int i = 0; i < reached; ++i )
	{
		lpPiece* p = w->pieces.data + w->scratchQueue.data[i];
		w->scratchLoad.data[i] = p->shape->volume * lpGetMaterial( p->material )->density * g;
		p->loadSlot = i;
	}

	int broken = 0;
	float maxUtilization = 0.0f;
	for ( int i = reached - 1; i >= 0; --i )
	{
		int pi = w->scratchQueue.data[i];
		lpPiece* p = w->pieces.data + pi;
		int depth = p->groundDepth;
		if ( depth == 0 )
		{
			continue; // anchored: the ground takes it
		}
		float load = w->scratchLoad.data[i];

		// Repeatedly split over the surviving downward bonds until none is overloaded (or none is left)
		for ( int attempt = 0; attempt < 8; ++attempt )
		{
			float totalArea = 0.0f;
			for ( int k = 0; k < p->bonds.count; ++k )
			{
				lpBond* bond = w->bonds.data + p->bonds.data[k];
				int other = bond->a == pi ? bond->b : bond->a;
				if ( w->pieces.data[other].groundDepth == depth - 1 )
				{
					totalArea += bond->area;
				}
			}
			if ( totalArea <= 0.0f )
			{
				break;
			}

			bool failed = false;
			for ( int k = 0; k < p->bonds.count; )
			{
				int bi = p->bonds.data[k];
				lpBond* bond = w->bonds.data + bi;
				int other = bond->a == pi ? bond->b : bond->a;
				if ( w->pieces.data[other].groundDepth != depth - 1 )
				{
					k += 1;
					continue;
				}
				float share = load * bond->area / totalArea;
				float capacity = bond->area * bond->loadStrength * b3MaxFloat( bond->health, 0.0f ) / bond->strength * w->def.stressScale;
				float utilization = capacity > 0.0f ? share / capacity : 1e9f;
				maxUtilization = utilization > maxUtilization ? utilization : maxUtilization;
				if ( share > capacity )
				{
					lpBreakBond( w, bi ); // removes it from p->bonds
					broken += 1;
					failed = true;
				}
				else
				{
					k += 1;
				}
			}
			if ( failed )
			{
				continue; // redistribute over the survivors
			}

			for ( int k = 0; k < p->bonds.count; ++k )
			{
				lpBond* bond = w->bonds.data + p->bonds.data[k];
				int other = bond->a == pi ? bond->b : bond->a;
				lpPiece* q = w->pieces.data + other;
				if ( q->groundDepth == depth - 1 )
				{
					w->scratchLoad.data[q->loadSlot] += load * bond->area / totalArea;
				}
			}
			break;
		}
	}
	if ( w->def.debugLog )
	{
		printf( "[lpf] tick %llu stress: body %d, %d pieces, %d reach the ground, %d bonds broke\n", (unsigned long long)w->tick,
				bodyIndex, n, reached, broken );
		printf( "[lpf]   peak bond utilization %.2f\n", (double)maxUtilization );
	}
	return broken;
}

void lpUpdateBody( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	body->dirty = false;
	if ( body->alive == false )
	{
		return;
	}
	if ( body->pieces.count == 0 )
	{
		lpDestroyBody( w, bodyIndex, false );
		return;
	}

	if ( body->kind == lp_kindStructure && lpStressPass( w, bodyIndex ) > 0 )
	{
		lpArray_Push( w->stressAgain, bodyIndex );
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
		lpComponent fresh = { w->scratchQueue.count, 0, 0.0f, b3Vec3_zero, false, 0, 0.0f };
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
			c->centroid = b3MulAdd( c->centroid, p->shape->volume, p->shape->centroid );
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
		c->centroid = b3MulSV( 1.0f / c->volume, c->centroid );
	}

	lpComponent* components = w->scratchComponents.data;
	int componentCount = w->scratchComponents.count;

	if ( componentCount <= 1 && ( body->kind != lp_kindStructure || componentCount == 0 || components[0].anchored ) )
	{
		return;
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

	b3WorldTransform xf = b3Body_GetTransform( body->id );
	b3Vec3 v = b3Body_GetLinearVelocity( body->id );
	b3Vec3 omega = b3Body_GetAngularVelocity( body->id );
	b3Vec3 localCenter = b3Body_GetLocalCenter( body->id );
	bool isDynamic = b3Body_GetType( body->id ) == b3_dynamicBody;

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
		b3AABB wakeBox = { comp->centroid, comp->centroid };
		for ( int k = 0; k < comp->count; ++k )
		{
			lpPiece* p = w->pieces.data + w->scratchQueue.data[comp->first + k];
			wakeBox.lowerBound = b3Min( wakeBox.lowerBound, p->shape->bounds.lowerBound );
			wakeBox.upperBound = b3Max( wakeBox.upperBound, p->shape->bounds.upperBound );
		}
		b3Vec3 wakeCenter = b3ToVec3( b3TransformWorldPoint( xf, b3AABB_Center( wakeBox ) ) );
		float wakeRadius = b3Length( b3AABB_Extents( wakeBox ) ) + 0.3f;
		lpArray_Push( w->pendingWakes, ( (lpWake){ wakeCenter, wakeRadius } ) );

		b3Vec3 compV = v;
		if ( isDynamic )
		{
			b3Vec3 r = b3RotateVector( xf.q, b3Sub( comp->centroid, localCenter ) );
			compV = b3Add( v, b3Cross( omega, r ) );
		}
		b3Vec3 compOmega = isDynamic ? omega : b3Vec3_zero;

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
				lpEmitParticle( w, xf, p->shape->centroid, compV, cbrtf( p->shape->volume ), p->material );
				lpFreePieceSlot( w, pi );
			}
		}
		else if ( comp->volume < lpGhostVolume( w, comp->material ) )
		{
			int ghost = lpBeginGhost( w, xf, compV, compOmega );
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
			int newIndex = lpCreateBodyInternal( w, xf, b3_dynamicBody, lp_kindDebris, tier, compV, compOmega );
			body = w->bodies.data + bodyIndex; // array may have moved
			lpBody* nb = w->bodies.data + newIndex;
			for ( int k = 0; k < comp->count; ++k )
			{
				int pi = w->scratchQueue.data[comp->first + k];
				lpPiece* p = w->pieces.data + pi;
				lpDetachPieceShape( w, pi );
				p->anchored = false;
				p->body = -1;
				lpAttachPiece( w, pi, newIndex );
			}
			b3Body_ApplyMassFromShapes( nb->id );
		}
		body = w->bodies.data + bodyIndex;
		body->volume -= comp->volume;
		movedAny += 1;
		w->stats.splitsThisStep += 1;
	}

	if ( movedAny == 0 )
	{
		return;
	}

	// Rebuild this body's piece list from the pieces that stayed
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
		b3Body_ApplyMassFromShapes( body->id );
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
}
