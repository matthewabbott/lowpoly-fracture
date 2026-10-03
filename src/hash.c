// SPDX-License-Identifier: MIT
// The state hash (lpWorld_Hash). Every element of the world is hashed on its own, seeded with its category, slot and
// generation:
// - a body with its pieces and bonds;
// - a structure's (or a moving body's) stress state;
// - the physics engine's state of a body (sleep, contact warm starts);
// - a link; a vehicle; a wheel; a rig with its limbs; a pool; a detonator;
// - and the world's own counters and the queues it carries from one step to the next.
// A category is the wrapping sum of its elements, so it does not depend on the order they are visited in, and a
// changed element is one subtraction and one addition. The root mixes the categories. Two machines that disagree
// compare categories, then the elements under them, to name the object. Fields go in one by one: never a struct with
// padding (determinism rule 11).

#include "world.h"

#define LP_FIELD( h, x ) ( h ) = lpHashWords( ( h ), &( x ), sizeof( x ) )

static uint64_t lpElementSeed( int category, int index, uint32_t generation )
{
	return lpMix64( lpMix64( ( (uint64_t)(uint32_t)category << 32 ) | (uint32_t)index ) ^ generation );
}

static uint64_t lpHashBool( uint64_t h, bool b )
{
	uint8_t v = b ? 1 : 0;
	return lpHashWords( h, &v, 1 );
}

static uint64_t lpHashImpact( uint64_t h, const lpImpactDef* d )
{
	float f[9] = { d->point.x, d->point.y, d->point.z, d->direction.x, d->direction.y, d->direction.z, d->radius, d->energy, d->impulse };
	h = lpHashWords( h, f, sizeof( f ) );
	return lpHashBool( h, d->explosion );
}

// ---- the world's own counters and queues ----

static uint64_t lpHashWorldElement( const lpWorld* w )
{
	uint64_t h = lpElementSeed( lp_hashWorld, 0, 0 );
	uint64_t counters[4] = { w->tick, w->pieceSerial, w->impactSerial, w->changeSerial };
	int32_t slots[12] = { w->pieces.count, w->bonds.count, w->bodies.count, w->links.count, w->wheels.count, w->freePiece,
						  w->freeBond,	   w->freeBody,	   w->freeLink,	   w->freeWheel,   w->bondCount,	w->linkCount };
	h = lpHashWords( h, counters, sizeof( counters ) );
	h = lpHashWords( h, slots, sizeof( slots ) );
	LP_FIELD( h, w->lastTimeStep );
	LP_FIELD( h, w->calmSteps );
	h = lpHashBool( h, w->supplyDirty );
	int32_t audit[2] = { w->audit.body, (int32_t)w->audit.generation };
	h = lpHashWords( h, audit, sizeof( audit ) );
	for ( int i = 0; i < w->audits.count; ++i )
	{
		int32_t ref[2] = { w->audits.data[i].body, (int32_t)w->audits.data[i].generation };
		h = lpHashWords( h, ref, sizeof( ref ) );
	}
	// What one step leaves the next: impacts queued and collision impacts found, blasts to remove, fractures deferred,
	// wheels to spawn, bodies to split, debris to freeze, pulls queued (all in order)
	for ( int i = 0; i < w->impacts.count; ++i )
	{
		h = lpHashImpact( h, w->impacts.data + i );
	}
	for ( int i = 0; i < w->nextImpacts.count; ++i )
	{
		h = lpHashImpact( lpMix64( h ), w->nextImpacts.data + i );
	}
	for ( int i = 0; i < w->pendingDestroy.count; ++i )
	{
		const lpPendingBlast* p = w->pendingDestroy.data + i;
		int32_t v[3] = { p->body, (int32_t)p->generation, p->detonator };
		h = lpHashWords( h, v, sizeof( v ) );
	}
	for ( int i = 0; i < w->deferred.count; ++i )
	{
		const lpDeferredJob* d = w->deferred.data + i;
		int32_t v[3] = { d->piece, (int32_t)d->generation, (int32_t)d->impactSerial };
		h = lpHashWords( h, v, sizeof( v ) );
		h = lpHashImpact( h, &d->impact );
		h = lpHashBool( h, d->snap );
	}
	for ( int i = 0; i < w->lostWheels.count; ++i )
	{
		const lpLostWheel* l = w->lostWheels.data + i;
		LP_FIELD( h, l->hub );
		LP_FIELD( h, l->velocity );
		LP_FIELD( h, l->omega );
		float size[2] = { l->radius, l->width };
		uint32_t look[2] = { l->material, l->color };
		h = lpHashWords( h, size, sizeof( size ) );
		h = lpHashWords( h, look, sizeof( look ) );
	}
	h = lpHashWords( lpMix64( h ), w->dirtyBodies.data, sizeof( int ) * (size_t)w->dirtyBodies.count );
	h = lpHashWords( lpMix64( h ), w->freezeCandidates.data, sizeof( int ) * (size_t)w->freezeCandidates.count );
	for ( int i = 0; i < w->pulls.count; ++i )
	{
		const lpPull* p = w->pulls.data + i;
		int32_t ref[2] = { p->piece, (int32_t)p->generation };
		float v[8] = { p->localPoint.x, p->localPoint.y, p->localPoint.z, p->target.x, p->target.y, p->target.z, p->maxAccel, p->maxMass };
		h = lpHashWords( h, ref, sizeof( ref ) );
		h = lpHashWords( h, v, sizeof( v ) );
	}
	return lpMix64( h );
}

// ---- a body with its pieces and bonds ----

static uint64_t lpHashBodyElement( const lpWorld* w, int bi )
{
	const lpBody* b = w->bodies.data + bi;
	uint64_t h = lpElementSeed( lp_hashBodies, bi, b->generation );
	int32_t head[7] = { b->kind, b->tier, b->pieces.count, (int32_t)b->topology, (int32_t)b->splitTopology, b->gridSlot, 0 };
	uint8_t flags[4] = { b->dirty, b->freezePending, b->splitChecked, b->solveStress };
	float scalars[3] = { b->volume, b->gravityScale, b->inertiaRadius };
	uint64_t ticks[2] = { b->createdTick, b->linkStamp };
	h = lpHashWords( h, head, sizeof( head ) );
	h = lpHashWords( h, flags, sizeof( flags ) );
	h = lpHashWords( h, scalars, sizeof( scalars ) );
	h = lpHashWords( h, ticks, sizeof( ticks ) );
	if ( b->kind == lp_kindGhost || b->kind == lp_kindScrap )
	{
		LP_FIELD( h, b->com );
		LP_FIELD( h, b->q );
		LP_FIELD( h, b->v );
		LP_FIELD( h, b->omega );
		LP_FIELD( h, b->localCenter );
		int32_t plan[3] = { b->planTicks, b->landIn, b->sinkTicks };
		h = lpHashWords( h, plan, sizeof( plan ) );
		LP_FIELD( h, b->landPoint );
		LP_FIELD( h, b->landNormal );
	}
	else
	{
		lpWorldTransform xf = lpPhys_GetTransform( w->phys, b->id );
		lpVec3 v = lpPhys_GetLinearVelocity( w->phys, b->id );
		lpVec3 omega = lpPhys_GetAngularVelocity( w->phys, b->id );
		LP_ASSERT( lpIsValidVec3( xf.p ) && lpIsValidVec3( v ) && lpIsValidVec3( omega ) ); // NaN in state is a bug (rule 17)
		LP_FIELD( h, xf );
		LP_FIELD( h, v );
		LP_FIELD( h, omega );
	}
	for ( int i = 0; i < b->pieces.count; ++i )
	{
		int pi = b->pieces.data[i];
		const lpPiece* p = w->pieces.data + pi;
		uint64_t digest = p->shape->digest;
		int32_t ids[12] = { pi,			(int32_t)p->generation, (int32_t)p->userId, p->part,	  p->tag,	   p->detonator,
							p->pool,	p->material,			p->joint,			p->depth,	  p->anchored, p->bonds.count };
		uint8_t channels[3 + LP_CHANNELS] = { p->carries, p->sources, p->needs };
		memcpy( channels + 3, p->supply, LP_CHANNELS );
		float geometry[8] = { p->axis.x,			   p->axis.y,				p->axis.z,				 p->anchorPlane.normal.x,
							  p->anchorPlane.normal.y, p->anchorPlane.normal.z, p->anchorPlane.offset, p->sourceShare };
		LP_FIELD( h, digest );
		h = lpHashWords( h, ids, sizeof( ids ) );
		h = lpHashWords( h, channels, sizeof( channels ) );
		h = lpHashWords( h, geometry, sizeof( geometry ) );
		h = lpHashWords( h, p->links.data, sizeof( int ) * (size_t)p->links.count );
		h = lpHashWords( h, p->bonds.data, sizeof( int ) * (size_t)p->bonds.count );
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			const lpBond* bond = w->bonds.data + p->bonds.data[k];
			if ( bond->a != pi )
			{
				continue; // each bond once, with its lower piece
			}
			int32_t ref[5] = { p->bonds.data[k], (int32_t)bond->generation, bond->b, bond->joint, (int32_t)bond->lastImpact };
			float shape[11] = { bond->area,		  bond->health,		bond->strength, bond->centroid.x, bond->centroid.y, bond->centroid.z,
								bond->normal.x, bond->normal.y, bond->normal.z, bond->h1,		  bond->h2 };
			h = lpHashWords( h, ref, sizeof( ref ) );
			h = lpHashWords( h, shape, sizeof( shape ) );
		}
	}
	return lpMix64( h );
}

// ---- a structure's (or a moving body's) stress state ----

static bool lpHasStressState( const lpBody* b )
{
	return b->kind == lp_kindStructure || b->solveStress;
}

static uint64_t lpHashSystem( uint64_t h, const lpStressSystem* s, int vectors )
{
	int32_t head[5] = { s->built, s->factored, (int32_t)s->topology, s->nodes.count, s->edges.count };
	h = lpHashWords( h, head, sizeof( head ) );
	LP_FIELD( h, s->forceScale );
	LP_FIELD( h, s->loadNorm2 );
	// x, f, r and p (the solver's z and q are made again from them)
	int n = s->nodes.count;
	if ( vectors > 0 && s->vectors.count >= 6 * n )
	{
		h = lpHashWords( h, s->vectors.data, sizeof( lpVec6 ) * (size_t)( 3 * n ) );
		h = lpHashWords( h, s->vectors.data + 4 * n, sizeof( lpVec6 ) * (size_t)n );
	}
	return h;
}

static uint64_t lpHashStressElement( const lpWorld* w, int bi )
{
	const lpBody* b = w->bodies.data + bi;
	uint64_t h = lpElementSeed( lp_hashStress, bi, b->generation );
	uint8_t flags[11] = { b->unsettled,	   b->solving,	  b->creaking,	  b->strainedLastCheck, b->reloadLoads, b->rejudge,
						  b->solveClustered, b->provisional, b->auditing, b->stepPair,		   b->hitMaterial };
	int32_t ints[8] = { b->stressSteps, (int32_t)b->solveTopology, b->solveNodes, b->solveEdges, b->clusters, b->meterRounds,
						(int32_t)b->clusterStamp, b->stressPin };
	uint64_t ticks[6] = { b->provisionalTick, b->hitCheckTick, b->loadCheckTick, b->joltTick, b->stepTick, b->hitTick };
	h = lpHashWords( h, flags, sizeof( flags ) );
	h = lpHashWords( h, ints, sizeof( ints ) );
	h = lpHashWords( h, ticks, sizeof( ticks ) );
	LP_FIELD( h, b->solveRz );
	LP_FIELD( h, b->reliefAccel );
	LP_FIELD( h, b->reliefAlpha );
	LP_FIELD( h, b->reliefOmega );
	LP_FIELD( h, b->reliefCenter );
	LP_FIELD( h, b->stepV );
	LP_FIELD( h, b->stepOmega );
	LP_FIELD( h, b->hitPoint );
	if ( b->system != NULL )
	{
		h = lpHashSystem( lpMix64( h ), b->system, b->solving ? 1 : 0 );
	}
	if ( b->reduced != NULL )
	{
		const lpStressReduced* red = b->reduced;
		int32_t head[4] = { red->built, (int32_t)red->topology, (int32_t)red->clusterStamp, red->partition.groupCount };
		h = lpHashWords( lpMix64( h ), head, sizeof( head ) );
		h = lpHashSystem( h, &red->system, b->solving ? 1 : 0 );
	}
	for ( int i = 0; i < b->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + b->pieces.data[i];
		LP_FIELD( h, p->stressX );
		LP_FIELD( h, p->stressLoad );
		LP_FIELD( h, p->stressResidual );
		float slender[4] = { p->strain, p->slenderRho, p->slenderAt, p->slenderDepth };
		int32_t solve[3] = { p->cluster, p->solveSlot, p->changed > p->accepted };
		h = lpHashWords( h, slender, sizeof( slender ) );
		h = lpHashWords( h, solve, sizeof( solve ) );
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			const lpBond* bond = w->bonds.data + p->bonds.data[k];
			if ( bond->a == b->pieces.data[i] )
			{
				float judged[8] = { bond->rho, bond->strain, bond->force.x, bond->force.y, bond->force.z, bond->moment.x, bond->moment.y, bond->moment.z };
				h = lpHashWords( h, judged, sizeof( judged ) );
			}
		}
	}
	return lpMix64( h );
}

// ---- the physics engine's state of a body: what its next step starts from besides the transform and velocities ----

static uint64_t lpHashBackendElement( const lpWorld* w, int bi, uint64_t contacts )
{
	const lpBody* b = w->bodies.data + bi;
	uint64_t h = lpElementSeed( lp_hashBackend, bi, b->generation );
	float sleep = lpPhys_GetSleepTime( w->phys, b->id );
	h = lpHashBool( h, lpPhys_IsAwake( w->phys, b->id ) );
	LP_FIELD( h, sleep );
	LP_FIELD( h, contacts );
	return lpMix64( h );
}

// ---- links, vehicles and wheels, rigs, pools, detonators ----

static uint64_t lpHashLinkElement( const lpWorld* w, int li )
{
	const lpLink* l = w->links.data + li;
	uint64_t h = lpElementSeed( lp_hashLinks, li, l->generation );
	const lpLinkDef* d = &l->def;
	int32_t ints[6] = { d->type, d->collideConnected, d->carries, (int32_t)d->userId, l->settle, l->wheel };
	float def[18] = { d->length,	   d->lowerAngle,	   d->upperAngle,		 d->coneAngle,	  d->hertz,		  d->dampingRatio,
					  d->maxForce,	   d->maxTorque,	   d->strength,		 d->tearRatio,	  d->motor.maxTorque, d->motor.maxSpeed,
					  d->motor.gain, d->motor.holdTorque, d->motor.jam,	 d->motor.jamKnock, (float)d->motor.needs, 0.0f };
	h = lpHashWords( h, ints, sizeof( ints ) );
	h = lpHashWords( h, def, sizeof( def ) );
	for ( int k = 0; k < 2; ++k )
	{
		int32_t end[2] = { l->ends[k].piece, (int32_t)l->ends[k].generation };
		h = lpHashWords( h, end, sizeof( end ) );
		LP_FIELD( h, l->ends[k].frame );
		LP_FIELD( h, l->points[k] );
	}
	float state[9] = { l->utilization, l->strain, l->health, l->target, l->appliedSpeed, l->appliedCap, l->motorCap, l->motorTorque, l->feed };
	uint8_t flags[3] = { l->targetChanged, l->motorApplied, 0 };
	h = lpHashWords( h, state, sizeof( state ) );
	h = lpHashWords( h, flags, sizeof( flags ) );
	LP_FIELD( h, l->angle );
	LP_FIELD( h, l->targetRotation );
	LP_FIELD( h, l->appliedVelocity );
	LP_FIELD( h, l->force );
	LP_FIELD( h, l->torque );
	LP_FIELD( h, l->stressForce );
	LP_FIELD( h, l->stressTorque );
	LP_FIELD( h, l->recheckTick );
	LP_FIELD( h, l->lastImpact );
	return lpMix64( h );
}

static uint64_t lpHashVehicleElement( const lpWorld* w, int vi )
{
	const lpVehicle* v = w->vehicles.data + vi;
	uint64_t h = lpElementSeed( lp_hashVehicles, vi, 0 );
	float control[3] = { v->control.throttle, v->control.brake, v->control.steer };
	uint8_t flags[4] = { v->control.handbrake, v->controlChanged, v->alive, v->controller };
	h = lpHashWords( h, control, sizeof( control ) );
	h = lpHashWords( h, flags, sizeof( flags ) );
	LP_FIELD( h, v->forward );
	LP_FIELD( h, v->up );
	LP_FIELD( h, v->wheelCount );
	h = lpHashWords( h, v->links, sizeof( int ) * (size_t)v->wheelCount );
	return lpMix64( h );
}

static uint64_t lpHashWheelElement( const lpWorld* w, int wi )
{
	const lpWheel* wh = w->wheels.data + wi;
	uint64_t h = lpElementSeed( lp_hashWheels, wi, wh->generation );
	int32_t ints[6] = { wh->link, wh->vehicle, wh->slot, wh->groundPiece, (int32_t)wh->groundGeneration, wh->stressBody };
	float state[9] = { wh->sprungMass, wh->steer, wh->spin, wh->spinSpeed, wh->length, wh->load, wh->slip, wh->friction, 0.0f };
	uint8_t flags[4] = { wh->grounded, wh->atStop, wh->sliding, 0 };
	h = lpHashWords( h, ints, sizeof( ints ) );
	h = lpHashWords( h, state, sizeof( state ) );
	h = lpHashWords( h, flags, sizeof( flags ) );
	LP_FIELD( h, wh->contactPoint );
	LP_FIELD( h, wh->contactNormal );
	LP_FIELD( h, wh->hub );
	LP_FIELD( h, wh->hubVelocity );
	LP_FIELD( h, wh->bodyOmega );
	LP_FIELD( h, wh->stressForce );
	LP_FIELD( h, wh->recheckTick );
	return lpMix64( h );
}

static uint64_t lpHashRigElement( const lpWorld* w, int ri )
{
	const lpRig* r = w->rigs.data + ri;
	uint64_t h = lpElementSeed( lp_hashRigs, ri, 0 );
	float control[4] = { r->control.forward, r->control.strafe, r->control.turn, r->control.crouch };
	uint8_t flags[7] = { r->alive, r->idle, r->crawling, r->stuck, r->controlChanged, r->posed, r->controller };
	int32_t ints[5] = { r->body, r->limbCount, r->calm, r->stall, r->ableSeen };
	h = lpHashWords( h, control, sizeof( control ) );
	h = lpHashWords( h, flags, sizeof( flags ) );
	h = lpHashWords( h, ints, sizeof( ints ) );
	LP_FIELD( h, r->desired );
	LP_FIELD( h, r->height );
	LP_FIELD( h, r->forward );
	LP_FIELD( h, r->up );
	LP_FIELD( h, r->poseLinear );
	LP_FIELD( h, r->poseAngular );
	for ( int i = 0; i < r->limbCount; ++i )
	{
		const lpLimb* l = r->limbs + i;
		int32_t li[10] = { l->joints,	l->rootBody, l->tipBody,	 (int32_t)l->tipGeneration, (int32_t)l->tipTopology,
						   l->tipJoints, l->grip,		 (int32_t)l->gripGeneration, l->touching, l->groundPiece };
		uint8_t lf[12] = { l->attached, l->able,		  l->planted,  l->swinging,	   l->castLate, l->grounded,
						   l->arrived,	l->reachWanted, l->reaching, l->target.active, 0,			0 };
		float scalars[5] = { l->strength, l->reach, l->depth, l->swingClock, l->holdClock };
		h = lpHashWords( h, li, sizeof( li ) );
		h = lpHashWords( h, lf, sizeof( lf ) );
		h = lpHashWords( h, scalars, sizeof( scalars ) );
		LP_FIELD( h, l->groundGeneration );
		LP_FIELD( h, l->recheckTick );
		LP_FIELD( h, l->foot );
		LP_FIELD( h, l->neutral );
		LP_FIELD( h, l->q );
		LP_FIELD( h, l->liftoff );
		LP_FIELD( h, l->landing );
		LP_FIELD( h, l->hold );
		LP_FIELD( h, l->reachPoint );
		LP_FIELD( h, l->target.point );
		LP_FIELD( h, l->target.velocity );
	}
	return lpMix64( h );
}

static uint64_t lpHashPoolElement( const lpWorld* w, int i )
{
	const lpPool* p = w->pools.data + i;
	uint64_t h = lpElementSeed( lp_hashPools, i, 0 );
	float state[8] = { p->capacity, p->level, p->leak, p->seal, p->reach, p->found, p->leakRate, p->pressure };
	h = lpHashWords( h, state, sizeof( state ) );
	LP_FIELD( h, p->step );
	return lpMix64( h );
}

static uint64_t lpHashDetonatorElement( const lpWorld* w, int i )
{
	const lpDetonator* d = w->detonators.data + i;
	uint64_t h = lpElementSeed( lp_hashDetonators, i, 0 );
	float state[6] = { d->def.triggerSpeed, d->def.radius, d->def.energy, d->def.speed, d->def.delay, d->fuse };
	uint8_t flags[2] = { d->armed, d->lit };
	h = lpHashWords( h, state, sizeof( state ) );
	h = lpHashWords( h, flags, sizeof( flags ) );
	return lpMix64( h );
}

// ---- the whole ----

void lpHashCategories( const lpWorld* w, uint64_t sums[lp_hashCategoryCount] )
{
	memset( sums, 0, sizeof( uint64_t ) * lp_hashCategoryCount );
	int bodies = w->bodies.count;
	uint64_t* contacts = lpAlloc( sizeof( uint64_t ) * (size_t)( bodies > 0 ? bodies : 1 ) );
	memset( contacts, 0, sizeof( uint64_t ) * (size_t)( bodies > 0 ? bodies : 1 ) );
	lpPhys_HashContacts( w->phys, NULL, contacts, bodies );

	sums[lp_hashWorld] = lpHashWorldElement( w );
	for ( int i = 0; i < bodies; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		if ( b->alive == false )
		{
			continue;
		}
		sums[lp_hashBodies] += lpHashBodyElement( w, i );
		sums[lp_hashStress] += lpHasStressState( b ) ? lpHashStressElement( w, i ) : 0;
		sums[lp_hashBackend] += LP_PHYS_NULL( b->id ) ? 0 : lpHashBackendElement( w, i, contacts[i] );
	}
	lpFree( contacts );
	for ( int i = 0; i < w->links.count; ++i )
	{
		sums[lp_hashLinks] += w->links.data[i].alive ? lpHashLinkElement( w, i ) : 0;
	}
	for ( int i = 0; i < w->vehicles.count; ++i )
	{
		sums[lp_hashVehicles] += lpHashVehicleElement( w, i );
	}
	for ( int i = 0; i < w->wheels.count; ++i )
	{
		sums[lp_hashWheels] += w->wheels.data[i].link >= 0 ? lpHashWheelElement( w, i ) : 0;
	}
	for ( int i = 0; i < w->rigs.count; ++i )
	{
		sums[lp_hashRigs] += lpHashRigElement( w, i );
	}
	for ( int i = 0; i < w->pools.count; ++i )
	{
		sums[lp_hashPools] += lpHashPoolElement( w, i );
	}
	for ( int i = 0; i < w->detonators.count; ++i )
	{
		sums[lp_hashDetonators] += lpHashDetonatorElement( w, i );
	}
}

uint64_t lpWorld_Hash( const lpWorld* w )
{
	uint64_t sums[lp_hashCategoryCount];
	lpHashCategories( w, sums );
	uint64_t h = LP_HASH_INIT;
	for ( int c = 0; c < lp_hashCategoryCount; ++c )
	{
		h = lpMix64( h ^ sums[c] );
	}
	return h;
}

uint64_t lpWorld_HashStress( const lpWorld* w )
{
	uint64_t sums[lp_hashCategoryCount];
	lpHashCategories( w, sums );
	return lpMix64( sums[lp_hashStress] );
}
