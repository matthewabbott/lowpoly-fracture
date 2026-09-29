// SPDX-License-Identifier: MIT
// The step: pulls, wakes, freezing settled debris into rubble, and the fixed order of everything in lpWorld_Step.

#include "world.h"

#include <math.h>
#include <stdio.h>

static void lpApplyWakes( lpWorld* w )
{
	for ( int i = 0; i < w->pendingWakes.count; ++i )
	{
		lpWake wake = w->pendingWakes.data[i];
		b3Vec3 r = { wake.radius, wake.radius, wake.radius };
		lpQueryPieces( w, (b3AABB){ b3Sub( wake.center, r ), b3Add( wake.center, r ) } );
		for ( int k = 0; k < w->scratchPieces.count; ++k )
		{
			lpPiece* p = w->pieces.data + w->scratchPieces.data[k];
			if ( p->body >= 0 )
			{
				lpWakeRubble( w, p->body );
			}
		}
	}
	w->pendingWakes.count = 0;
}

static void lpFreezeOrKill( lpWorld* w )
{
	b3BodyEvents events = b3World_GetBodyEvents( w->def.physics );
	w->scratchBodies.count = 0;
	for ( int i = 0; i < events.moveCount; ++i )
	{
		const b3BodyMoveEvent* e = events.moveEvents + i;
		intptr_t data = (intptr_t)e->userData;
		if ( data <= 0 )
		{
			continue;
		}
		int bodyIndex = (int)( data - 1 );
		lpBody* b = w->bodies.data + bodyIndex;
		if ( b->alive == false || b->kind != lp_kindDebris )
		{
			continue;
		}
		if ( (float)e->transform.p.y < w->def.killDepth )
		{
			lpArray_Push( w->scratchBodies, bodyIndex );
		}
		else if ( e->fellAsleep && w->def.freezeRubble && b->freezePending == false )
		{
			b->freezePending = true;
			lpArray_Push( w->freezeCandidates, bodyIndex );
		}
	}

	// Freeze sleepers that are old enough. Fresh debris wedged in its hole can fall asleep before anything pushed
	// it out; freezing it at once would glue it back into the wall.
	int kept = 0;
	for ( int i = 0; i < w->freezeCandidates.count; ++i )
	{
		int bodyIndex = w->freezeCandidates.data[i];
		lpBody* b = w->bodies.data + bodyIndex;
		if ( b->alive == false || b->kind != lp_kindDebris || b->freezePending == false )
		{
			continue;
		}
		if ( b3Body_IsAwake( b->id ) || lpBodyLinked( w, b ) || lpTouchesLinked( w, b ) )
		{
			b->freezePending = false; // linked bodies and what rests on them sleep instead: frozen, they would jam
			continue;
		}
		uint64_t minAge = b->tier == lp_tierLight ? 6u : 30u;
		if ( w->tick - b->createdTick >= minAge && w->freezesThisStep < w->def.maxFreezesPerStep )
		{
			b->freezePending = false;
			b->kind = lp_kindRubble;
			b3Body_SetType( b->id, b3_staticBody );
			w->freezesThisStep += 1;
			continue;
		}
		w->freezeCandidates.data[kept++] = bodyIndex;
	}
	w->freezeCandidates.count = kept;

	// Kill list in index order
	if ( w->scratchBodies.count > 1 )
	{
		qsort( w->scratchBodies.data, (size_t)w->scratchBodies.count, sizeof( int ), lpCompareInt );
	}
	for ( int i = 0; i < w->scratchBodies.count; ++i )
	{
		int bodyIndex = w->scratchBodies.data[i];
		if ( w->bodies.data[bodyIndex].alive )
		{
			lpDestroyBody( w, bodyIndex, false );
		}
	}
}

void lpWorld_Pull( lpWorld* w, int piece, b3Vec3 localPoint, b3Pos target, float maxAccel, float maxMass )
{
	lpPull pull = { piece, localPoint, target, maxAccel, maxMass };
	lpArray_Push( w->pulls, pull );
}

b3Vec3 lpWorld_ToBodyFrame( const lpWorld* w, int piece, b3Pos worldPoint )
{
	const lpPiece* p = w->pieces.data + piece;
	if ( p->body < 0 )
	{
		return b3Vec3_zero;
	}
	return b3InvTransformWorldPoint( lpGetTransform( w->bodies.data + p->body ), worldPoint );
}

b3Pos lpWorld_ToWorldFrame( const lpWorld* w, int piece, b3Vec3 localPoint )
{
	const lpPiece* p = w->pieces.data + piece;
	if ( p->body < 0 )
	{
		return b3ToPos( localPoint );
	}
	return b3TransformWorldPoint( lpGetTransform( w->bodies.data + p->body ), localPoint );
}

// Spring-damper toward the target, mass-normalized and clamped, with gravity compensation up to maxMass
static void lpApplyPulls( lpWorld* w )
{
	for ( int i = 0; i < w->pulls.count; ++i )
	{
		lpPull pull = w->pulls.data[i];
		if ( pull.piece < 0 || pull.piece >= w->pieces.count || w->pieces.data[pull.piece].body < 0 )
		{
			continue;
		}
		int bodyIndex = w->pieces.data[pull.piece].body;
		lpBody* b = w->bodies.data + bodyIndex;
		if ( b->kind == lp_kindStructure )
		{
			continue;
		}
		// Whatever is grabbed gets full physics back
		if ( b->kind == lp_kindGhost || b->kind == lp_kindScrap || b->tier == lp_tierLight )
		{
			lpConvertToFull( w, bodyIndex );
			b = w->bodies.data + bodyIndex;
		}
		lpWakeRubble( w, bodyIndex );

		b3Pos point = b3TransformWorldPoint( b3Body_GetTransform( b->id ), pull.localPoint );
		b3Vec3 v = b3Body_GetWorldPointVelocity( b->id, point );
		b3Vec3 error = b3SubPos( pull.target, point );
		b3Vec3 accel = b3Sub( b3MulSV( 60.0f, error ), b3MulSV( 14.0f, v ) );
		float a = b3Length( accel );
		if ( a > pull.maxAccel )
		{
			accel = b3MulSV( pull.maxAccel / a, accel );
		}
		float mass = b3Body_GetMass( b->id );
		float m = b3MinFloat( mass, pull.maxMass );
		b3Vec3 g = b3MulSV( b->gravityScale, b3World_GetGravity( w->def.physics ) );
		b3Vec3 force = b3Sub( b3MulSV( m, accel ), b3MulSV( m, g ) );
		b3Body_ApplyForce( b->id, force, point, true );
		// A little angular damping so held things do not spin forever
		b3Vec3 omega = b3Body_GetAngularVelocity( b->id );
		b3Body_SetAngularVelocity( b->id, b3MulSV( 0.97f, omega ) );
	}
	w->pulls.count = 0;
}

void lpWorld_Step( lpWorld* w, float timeStep, int subStepCount )
{
	w->stats.impactsThisStep = 0;
	w->stats.fracturesThisStep = 0;
	w->stats.cellsThisStep = 0;
	w->stats.splitsThisStep = 0;
	w->stats.cellMs = 0.0f;
	w->stats.hullMs = 0.0f;
	w->stats.shapeMs = 0.0f;
	w->stats.bondMs = 0.0f;
	w->stats.splitMs = 0.0f;
	w->stats.voronoiCpuMs = 0.0f;
	w->stats.mergeCpuMs = 0.0f;
	w->stats.hullCpuMs = 0.0f;
	w->stats.stressMs = 0.0f;
	w->stats.stressIterations = 0;
	w->stats.stressBreaks = 0;
	w->stats.stressSolves = 0;
	w->stats.stressWaiting = 0;
	w->stats.linkBreaks = 0;
	w->stats.linkRebuilds = 0;
	w->stressWork = 0;
	w->stats.demotionsThisStep = 0;
	w->stats.ghostCasts = 0;
	w->particles.count = 0;
	w->jobsThisStep = 0;
	w->freezesThisStep = 0;

	uint64_t ticks = b3GetTicks();

	// Bodies that detonated last step are consumed by their own blast. One that is already gone may have had its slot
	// reused since, so the generation must match too.
	if ( w->pendingDestroy.count > 1 )
	{
		qsort( w->pendingDestroy.data, (size_t)w->pendingDestroy.count, sizeof( lpBodyRef ), lpCompareBodyRef );
	}
	for ( int i = 0; i < w->pendingDestroy.count; ++i )
	{
		lpBodyRef ref = w->pendingDestroy.data[i];
		const lpBody* b = w->bodies.data + ref.body;
		if ( b->alive && b->generation == ref.generation )
		{
			lpDestroyBody( w, ref.body, true );
		}
	}
	w->pendingDestroy.count = 0;

	// Fracture work deferred by earlier steps' budgets
	lpProcessDeferred( w );

	// Impacts: tool impacts in call order, then last step's collision impacts
	for ( int i = 0; i < w->nextImpacts.count; ++i )
	{
		lpArray_Push( w->impacts, w->nextImpacts.data[i] );
	}
	w->nextImpacts.count = 0;

	for ( int i = 0; i < w->impacts.count; ++i )
	{
		lpImpactDef impact = w->impacts.data[i];
		lpProcessImpact( w, &impact );
		if ( impact.impulse > 0.0f )
		{
			lpForce force = { impact.point, impact.direction, impact.explosion ? 1.5f * impact.radius : impact.radius,
							  impact.impulse, impact.explosion };
			lpArray_Push( w->forces, force );
		}
	}
	w->impacts.count = 0;

	// Split what came apart, then the stress check of the structures updated, solved in parallel
	uint64_t splitTicks = b3GetTicks();
	lpUpdateDirtyBodies( w );
	lpCheckStructures( w, false );
	w->stats.splitMs = b3GetMilliseconds( splitTicks );

	lpApplyWakes( w );
	lpApplyForces( w );
	lpApplyBlows( w );
	lpApplyPulls( w );
	lpSyncLinks( w ); // every body of this step exists now
	w->stats.fractureMs = b3GetMillisecondsAndReset( &ticks );

	b3World_Step( w->def.physics, timeStep, subStepCount );
	w->lastTimeStep = timeStep;
	w->stats.physicsMs = b3GetMillisecondsAndReset( &ticks );
	lpPollLinks( w, timeStep ); // before anything below can destroy a body under a joint

	lpStepGhosts( w, timeStep );
	lpShove( w, timeStep );
	lpCollectHits( w );
	lpFreezeOrKill( w );
	lpEnforceBudgets( w );

	// Counters
	w->stats.structureBodies = 0;
	w->stats.unsettledStructures = 0;
	w->stats.debrisBodies = 0;
	w->stats.awakeDebris = 0;
	w->stats.rubbleBodies = 0;
	w->stats.fullDebris = 0;
	w->stats.lightDebris = 0;
	w->stats.ghostBodies = 0;
	w->stats.scrapBodies = 0;
	int pieceCount = 0;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		if ( b->alive == false )
		{
			continue;
		}
		pieceCount += b->pieces.count;
		if ( b->kind == lp_kindStructure )
		{
			w->stats.structureBodies += 1;
			w->stats.unsettledStructures += b->unsettled ? 1 : 0;
		}
		else if ( b->kind == lp_kindRubble )
		{
			w->stats.rubbleBodies += 1;
		}
		else if ( b->kind == lp_kindGhost )
		{
			w->stats.ghostBodies += 1;
		}
		else if ( b->kind == lp_kindScrap )
		{
			w->stats.scrapBodies += 1;
		}
		else
		{
			w->stats.debrisBodies += 1;
			w->stats.fullDebris += b->tier == lp_tierFull ? 1 : 0;
			w->stats.lightDebris += b->tier == lp_tierLight ? 1 : 0;
			w->stats.awakeDebris += b3Body_IsAwake( b->id ) ? 1 : 0;
		}
	}
	w->stats.pieceCount = pieceCount;
	w->stats.bondCount = w->bondCount;
	w->stats.linkCount = w->linkCount;
	w->stats.deferredJobs = w->deferred.count;
	w->stats.updateMs = b3GetMillisecondsAndReset( &ticks );

	w->tick += 1;
}
