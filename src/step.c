// SPDX-License-Identifier: MIT
// The step: pulls, wakes, freezing settled debris into rubble, and the fixed order of everything in lpWorld_Step.

#include "tasks.h"
#include "world.h"

#include <math.h>
#include <stdio.h>

static void lpApplyWakes( lpWorld* w )
{
	for ( int i = 0; i < w->pendingWakes.count; ++i )
	{
		lpWake wake = w->pendingWakes.data[i];
		lpVec3 r = { wake.radius, wake.radius, wake.radius };
		lpQueryPieces( w, (lpAABB){ lpSub( wake.center, r ), lpAdd( wake.center, r ) } );
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
	const lpPhysMove* moves;
	int moveCount = lpPhys_GetMoves( w->phys, &moves );
	w->scratchBodies.count = 0;
	for ( int i = 0; i < moveCount; ++i ) // in body order, so this step's new candidates and the kill list are too
	{
		const lpPhysMove* e = moves + i;
		int bodyIndex = e->body;
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
		if ( lpPhys_IsAwake( w->phys, b->id ) || lpBodyLinked( w, b ) || lpTouchesLinked( w, b ) )
		{
			b->freezePending = false; // linked bodies and what rests on them sleep instead: frozen, they would jam
			continue;
		}
		uint64_t minAge = b->tier == lp_tierLight ? 6u : 30u;
		if ( w->tick - b->createdTick >= minAge && w->freezesThisStep < w->def.maxFreezesPerStep )
		{
			b->freezePending = false;
			b->kind = lp_kindRubble;
			lpPhys_SetDynamic( w->phys, b->id, false );
			w->freezesThisStep += 1;
			continue;
		}
		w->freezeCandidates.data[kept++] = bodyIndex;
	}
	w->freezeCandidates.count = kept;

	for ( int i = 0; i < w->scratchBodies.count; ++i )
	{
		int bodyIndex = w->scratchBodies.data[i];
		if ( w->bodies.data[bodyIndex].alive )
		{
			lpDestroyBody( w, bodyIndex, false );
		}
	}
}

static int lpComparePendingBlast( const void* a, const void* b )
{
	const lpPendingBlast* x = a;
	const lpPendingBlast* y = b;
	if ( x->body != y->body )
	{
		return ( x->body > y->body ) - ( x->body < y->body );
	}
	if ( x->generation != y->generation )
	{
		return ( x->generation > y->generation ) - ( x->generation < y->generation );
	}
	return ( x->detonator > y->detonator ) - ( x->detonator < y->detonator );
}

// The pieces of a body that share a detonator go up in its blast: the whole body when they are all of it (or it is a
// ghost or scrap), else only they, and what is left is split next
static void lpDestroyDetonated( lpWorld* w, int bodyIndex, int detonator )
{
	lpBody* b = w->bodies.data + bodyIndex;
	bool whole = true;
	for ( int k = 0; k < b->pieces.count && b->kind != lp_kindGhost && b->kind != lp_kindScrap; ++k )
	{
		whole = whole && w->pieces.data[b->pieces.data[k]].detonator == detonator;
	}
	if ( whole )
	{
		lpDestroyBody( w, bodyIndex, true );
		return;
	}
	lpWorldTransform xf = lpGetTransform( w, b );
	int kept = 0;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		int pieceIndex = b->pieces.data[k];
		lpPiece* p = w->pieces.data + pieceIndex;
		if ( p->detonator != detonator )
		{
			b->pieces.data[kept++] = pieceIndex;
			continue;
		}
		while ( p->bonds.count > 0 )
		{
			lpBreakBond( w, p->bonds.data[p->bonds.count - 1] );
		}
		lpEmitParticle( w, xf, p->shape->centroid, lpVec3_zero, lpMinFloat( lpCbrt( p->shape->volume ), 0.3f ), p->material );
		b->volume -= p->shape->volume;
		lpDetachPieceShape( w, pieceIndex );
		lpFreePieceSlot( w, pieceIndex );
	}
	b->pieces.count = kept;
	b->topology += 1;
	if ( lpPhys_IsDynamic( w->phys, b->id ) )
	{
		lpApplyMass( w, b );
	}
	lpMarkDirty( w, bodyIndex );
}

void lpWorld_Pull( lpWorld* w, int piece, lpVec3 localPoint, lpPos target, float maxAccel, float maxMass )
{
	lpPull pull = { piece, localPoint, target, maxAccel, maxMass };
	lpArray_Push( w->pulls, pull );
}

lpVec3 lpWorld_ToBodyFrame( const lpWorld* w, int piece, lpPos worldPoint )
{
	const lpPiece* p = w->pieces.data + piece;
	if ( p->body < 0 )
	{
		return lpVec3_zero;
	}
	return lpInvTransformWorldPoint( lpGetTransform( w, w->bodies.data + p->body ), worldPoint );
}

lpPos lpWorld_ToWorldFrame( const lpWorld* w, int piece, lpVec3 localPoint )
{
	const lpPiece* p = w->pieces.data + piece;
	if ( p->body < 0 )
	{
		return lpToPos( localPoint );
	}
	return lpTransformWorldPoint( lpGetTransform( w, w->bodies.data + p->body ), localPoint );
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

		lpPos point = lpTransformWorldPoint( lpPhys_GetTransform( w->phys, b->id ), pull.localPoint );
		lpVec3 v = lpPhys_GetPointVelocity( w->phys, b->id, point );
		lpVec3 error = lpSubPos( pull.target, point );
		lpVec3 accel = lpSub( lpMulSV( 60.0f, error ), lpMulSV( 14.0f, v ) );
		float a = lpLength( accel );
		if ( a > pull.maxAccel )
		{
			accel = lpMulSV( pull.maxAccel / a, accel );
		}
		float mass = lpPhys_GetMass( w->phys, b->id );
		float m = lpMinFloat( mass, pull.maxMass );
		lpVec3 g = lpMulSV( b->gravityScale, w->def.gravity );
		lpVec3 force = lpSub( lpMulSV( m, accel ), lpMulSV( m, g ) );
		lpPhys_ApplyForce( w->phys, b->id, force, point, true );
		// A little angular damping so held things do not spin forever
		lpVec3 omega = lpPhys_GetAngularVelocity( w->phys, b->id );
		lpPhys_SetAngularVelocity( w->phys, b->id, lpMulSV( 0.97f, omega ) );
	}
	w->pulls.count = 0;
}

void lpWorld_Step( lpWorld* w, float timeStep, int subStepCount )
{
	lpGuardFp( w ); // a library or driver may have changed this thread's control word since the last step
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
	w->stats.stressJudged = 0;
	w->stats.stressReduced = 0;
	w->stats.stressDissolved = 0;
	w->stats.stressAudits = 0;
	w->stats.stressWaiting = 0;
	w->stats.linkBreaks = 0;
	w->stats.linkRebuilds = 0;
	w->stats.wheelCasts = 0;
	w->stats.supplyUpdates = 0;
	w->stats.motorSets = 0;
	w->stats.footCasts = 0;
	w->stats.leakingPools = 0;
	w->stressWork = 0;
	w->stats.demotionsThisStep = 0;
	w->stats.ghostCasts = 0;
	w->particles.count = 0;
	w->jobsThisStep = 0;
	w->freezesThisStep = 0;

	uint64_t ticks = lpGetTicks();

	// Pieces that detonated last step are consumed by their own blast: the whole body when they were all of it, else
	// only they. A body that is already gone may have had its slot reused since, so the generation must match too.
	if ( w->pendingDestroy.count > 1 )
	{
		qsort( w->pendingDestroy.data, (size_t)w->pendingDestroy.count, sizeof( lpPendingBlast ), lpComparePendingBlast );
	}
	for ( int i = 0; i < w->pendingDestroy.count; ++i )
	{
		lpPendingBlast pending = w->pendingDestroy.data[i];
		const lpBody* b = w->bodies.data + pending.body;
		if ( b->alive && b->generation == pending.generation )
		{
			lpDestroyDetonated( w, pending.body, pending.detonator );
		}
	}
	w->pendingDestroy.count = 0;
	lpSpawnLostWheels( w ); // wheels that came off last step

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
	uint64_t splitTicks = lpGetTicks();
	lpUpdateDirtyBodies( w );
	lpCheckStructures( w, false );
	w->stats.splitMs = lpGetMilliseconds( splitTicks );

	lpApplyWakes( w );
	lpApplyForces( w );
	lpApplyBlows( w );
	lpApplyPulls( w );
	lpSyncLinks( w ); // every body of this step exists now
	lpDrainPools( w, timeStep );
	lpUpdateSupply( w );
	w->stats.fractureMs = lpGetMillisecondsAndReset( &ticks );
	lpStepRigs( w, timeStep );
	w->stats.rigMs = lpGetMillisecondsAndReset( &ticks );
	lpDriveMotors( w );
	w->stats.fractureMs += lpGetMillisecondsAndReset( &ticks );
	lpStepVehicles( w, timeStep );
	w->stats.vehicleMs = lpGetMillisecondsAndReset( &ticks );

	lpPhys_Step( w->phys, timeStep, subStepCount );
	w->lastTimeStep = timeStep;
	w->stats.physicsMs = lpGetMillisecondsAndReset( &ticks );
	lpPollLinks( w, timeStep ); // before anything below can destroy a body under a joint
	lpTrackMovingBodies( w );

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
	w->stats.clusteredPieces = 0;
	w->stats.provisionalStructures = 0;
	w->stats.auditBacklog = w->audits.count;
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
			w->stats.provisionalStructures += b->provisional ? 1 : 0;
			for ( int k = 0; k < b->pieces.count && b->clusters > 0; ++k )
			{
				w->stats.clusteredPieces += w->pieces.data[b->pieces.data[k]].cluster != 0 ? 1 : 0;
			}
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
			w->stats.awakeDebris += lpPhys_IsAwake( w->phys, b->id ) ? 1 : 0;
		}
	}
	w->stats.pieceCount = pieceCount;
	w->stats.bondCount = w->bondCount;
	w->stats.linkCount = w->linkCount;
	w->stats.deferredJobs = w->deferred.count;
	w->stats.fpRepairs = w->fpRepairs + lpTaskPool_FpRepairs( w->tasks );
	lpPhysCounters counters = lpPhys_GetCounters( w->phys );
	w->stats.shapes = counters.shapes;
	w->stats.contacts = counters.contacts;
	w->stats.awakeContacts = counters.awakeContacts;
	w->stats.updateMs = lpGetMillisecondsAndReset( &ticks );

	w->tick += 1;
}
