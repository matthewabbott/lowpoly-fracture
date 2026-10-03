// SPDX-License-Identifier: MIT
// The state hash (lpWorld_Hash). Every element of the world is hashed on its own, seeded with its category, slot and
// generation:
// - a body: its kind, motion and which pieces;
// - a piece, with the bonds it holds;
// - a structure's (or a moving body's) stress state;
// - the physics engine's state of a body (sleep, contact warm starts);
// - a link; a vehicle; a wheel; a rig with its limbs; a pool; a detonator;
// - and the world's own counters and the queues it carries from one step to the next.
// A category is the wrapping sum of its elements, so it does not depend on the order they are visited in, and a
// changed element is one subtraction and one addition. The root mixes the categories. Two machines that disagree
// compare categories, then the elements under them, to name the object. Fields go in one by one: never a struct with
// padding (determinism rule 11).
//
// Kept, not recomputed: the body and piece categories (the big ones) cache each slot's element hashes, and only the
// slots marked since the last hash are hashed again (in parallel, then added up in slot order). Whatever writes hashed
// state marks it (lpHashMark, lpHashMarkStress, lpHashMarkPiece; lpTouchPiece does both): the physics engine's moves and
// the setters it saw are marked at step end. The other categories are small and hashed whole each time.
// lpWorld_CheckHash recomputes everything and names an element that changed unmarked (lpf_bench --check-hash).

#include "tasks.h"
#include "world.h"

#include <stdio.h>

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
	// Supply changes on their way, and where the next ones start (supplyHopsPerTick)
	h = lpHashWords( h, w->supplySites.data, sizeof( int ) * (size_t)w->supplySites.count );
	for ( int i = 0; i < w->supplyWaves.count; ++i )
	{
		const lpSupplyWave* s = w->supplyWaves.data + i;
		uint32_t wave[7] = { (uint32_t)s->tick, (uint32_t)( s->tick >> 32 ), (uint32_t)s->piece, s->generation, (uint32_t)s->pool,
							 0, (uint32_t)s->channel | ( (uint32_t)s->value << 8 ) };
		memcpy( wave + 5, &s->leak, 4 );
		h = lpHashWords( h, wave, sizeof( wave ) );
	}
	return lpMix64( h );
}

// ---- a body: its kind, motion and which pieces ----

static uint64_t lpHashBodyElement( const lpWorld* w, int bi )
{
	const lpBody* b = w->bodies.data + bi;
	uint64_t h = lpElementSeed( lp_hashBodies, bi, b->generation );
	int32_t head[7] = { b->kind, b->tier, b->pieces.count, (int32_t)b->topology, (int32_t)b->splitTopology, b->gridSlot, 0 };
	uint8_t flags[4] = { b->freezePending, b->splitChecked, b->solveStress, 0 }; // dirty: the world's list
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
		lpWorldTransform xf;
		lpVec3 v, omega;
		lpPhys_GetMotion( w->phys, b->id, &xf, &v, &omega );
		LP_ASSERT( lpIsValidVec3( xf.p ) && lpIsValidVec3( v ) && lpIsValidVec3( omega ) ); // NaN in state is a bug (rule 17)
		LP_FIELD( h, xf );
		LP_FIELD( h, v );
		LP_FIELD( h, omega );
	}
	h = lpHashWords( h, b->pieces.data, sizeof( int ) * (size_t)b->pieces.count ); // which, in order
	return lpMix64( h );
}

// ---- a piece, with the bonds it holds (each bond with its lower piece) ----

static uint64_t lpHashPieceElement( const lpWorld* w, int pi )
{
	const lpPiece* p = w->pieces.data + pi;
	uint64_t h = lpElementSeed( lp_hashPieces, pi, p->generation );
	// One call for the fixed fields: the body, the shape's digest, ids, channels, axis and anchor plane
	uint32_t fixed[26];
	memcpy( fixed, &p->shape->digest, 8 );
	uint32_t ids[12] = { (uint32_t)p->body, (uint32_t)p->userId, p->part,	 p->tag,	 (uint32_t)p->detonator, (uint32_t)p->pool,
						 p->material,		 p->joint,			 p->depth, p->anchored, (uint32_t)p->bonds.count, (uint32_t)p->links.count };
	memcpy( fixed + 2, ids, sizeof( ids ) );
	uint8_t channels[12] = { p->carries, p->sources, p->needs, 0 };
	memcpy( channels + 4, p->supply, LP_CHANNELS );
	memcpy( fixed + 14, channels, sizeof( channels ) );
	float geometry[8] = { p->axis.x,			   p->axis.y,				p->axis.z,				 p->anchorPlane.normal.x,
						  p->anchorPlane.normal.y, p->anchorPlane.normal.z, p->anchorPlane.offset, p->sourceShare };
	memcpy( fixed + 17, geometry, sizeof( geometry ) );
	fixed[25] = 0;
	h = lpHashWords( h, fixed, sizeof( fixed ) );
	h = lpHashWords( h, p->links.data, sizeof( int ) * (size_t)p->links.count );
	h = lpHashWords( h, p->bonds.data, sizeof( int ) * (size_t)p->bonds.count );
	for ( int k = 0; k < p->bonds.count; ++k )
	{
		const lpBond* bond = w->bonds.data + p->bonds.data[k];
		if ( bond->a != pi )
		{
			continue;
		}
		uint32_t packed[16] = { (uint32_t)p->bonds.data[k], bond->generation, (uint32_t)bond->b, bond->joint, bond->lastImpact };
		float shape[11] = { bond->area,		  bond->health,		bond->strength, bond->centroid.x, bond->centroid.y, bond->centroid.z,
							bond->normal.x, bond->normal.y, bond->normal.z, bond->h1,		  bond->h2 };
		memcpy( packed + 5, shape, sizeof( shape ) );
		h = lpHashWords( h, packed, sizeof( packed ) );
	}
	return lpMix64( h );
}

static uint64_t lpHashPieceSlot( const lpWorld* w, int pi )
{
	return w->pieces.data[pi].body >= 0 ? lpHashPieceElement( w, pi ) : 0;
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
	// A solve in progress continues from r and p (z and q are made again from them; f from the pieces' loads). Its x is
	// the pieces' stressX, but for a correction through clusters (vectors 2), which keeps the x it corrects until judged.
	int n = s->nodes.count;
	if ( vectors > 0 && s->vectors.count >= 6 * n )
	{
		if ( vectors > 1 )
		{
			h = lpHashWords( h, s->vectors.data, sizeof( lpVec6 ) * (size_t)n );
		}
		h = lpHashWords( h, s->vectors.data + 2 * n, sizeof( lpVec6 ) * (size_t)n );
		h = lpHashWords( h, s->vectors.data + 4 * n, sizeof( lpVec6 ) * (size_t)n );
	}
	return h;
}

#define LP_STRESS_CHUNK 256 // pieces per part of a stress element
#define LP_STRESS_BIG 4		// parts from which a stress element is hashed in parallel

static int lpStressChunks( const lpBody* b )
{
	return ( b->pieces.count + LP_STRESS_CHUNK - 1 ) / LP_STRESS_CHUNK;
}

// The body's own stress state: its flags, counters, relief, and the systems of a solve in progress
static uint64_t lpHashStressHead( const lpWorld* w, int bi )
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
	h = lpHashBool( h, b->frontGrow ); // a region solve that grows next step
	if ( b->system != NULL )
	{
		h = lpHashSystem( lpMix64( h ), b->system, b->solving ? ( b->solveClustered ? 2 : 1 ) : 0 );
	}
	if ( b->reduced != NULL )
	{
		const lpStressReduced* red = b->reduced;
		int32_t head[4] = { red->built, (int32_t)red->topology, (int32_t)red->clusterStamp, red->partition.groupCount };
		h = lpHashWords( lpMix64( h ), head, sizeof( head ) );
		h = lpHashSystem( h, &red->system, b->solving && b->solveClustered ? 2 : 0 );
	}
	return h;
}

// Its pieces' stress state, one part of them
static uint64_t lpHashStressChunk( const lpWorld* w, int bi, int chunk )
{
	const lpBody* b = w->bodies.data + bi;
	uint64_t h = lpMix64( (uint64_t)chunk + 1 );
	int end = lpMinInt( ( chunk + 1 ) * LP_STRESS_CHUNK, b->pieces.count );
	for ( int i = chunk * LP_STRESS_CHUNK; i < end; ++i )
	{
		const lpPiece* p = w->pieces.data + b->pieces.data[i];
		float state[26];
		memcpy( state, &p->stressX, sizeof( lpVec6 ) );
		memcpy( state + 6, &p->stressLoad, sizeof( lpVec6 ) );
		memcpy( state + 12, &p->stressResidual, sizeof( lpVec6 ) );
		float slender[4] = { p->strain, p->slenderRho, p->slenderAt, p->slenderDepth };
		int32_t solve[4] = { p->cluster, p->solveSlot, p->changed > p->accepted, ( p->inFront ? 1 : 0 ) | ( p->unaudited ? 2 : 0 ) };
		memcpy( state + 18, slender, sizeof( slender ) );
		memcpy( state + 22, solve, sizeof( solve ) );
		h = lpHashWords( h, state, sizeof( state ) );
		h = lpHashWords( h, &p->acceptedLoad, sizeof( lpVec6 ) ); // the drift guard's (region solves)
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
	return h;
}

static uint64_t lpHashStressCombine( uint64_t head, const uint64_t* chunks, int count )
{
	uint64_t h = head;
	for ( int c = 0; c < count; ++c )
	{
		h = lpMix64( h ^ chunks[c] );
	}
	return lpMix64( h );
}

static uint64_t lpHashStressElement( const lpWorld* w, int bi )
{
	uint64_t h = lpHashStressHead( w, bi );
	int count = lpStressChunks( w->bodies.data + bi );
	for ( int c = 0; c < count; ++c )
	{
		h = lpMix64( h ^ lpHashStressChunk( w, bi, c ) );
	}
	return lpMix64( h );
}

static bool lpBigStress( const lpBody* b )
{
	return lpStressChunks( b ) >= LP_STRESS_BIG;
}

typedef struct lpStressChunkJob
{
	const lpWorld* world;
	int body;
	uint64_t* chunks;
} lpStressChunkJob;

static void lpRunStressChunk( int chunk, void* context )
{
	const lpStressChunkJob* job = context;
	job->chunks[chunk] = lpHashStressChunk( job->world, job->body, chunk );
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
	if ( LP_PHYS_NULL( l->joint ) == false )
	{
		// The physics engine's warm start for the joint: what its next solve starts from
		float impulses[LP_JOINT_IMPULSES];
		int count = lpPhys_GetJointImpulses( w->phys, l->joint, impulses, LP_JOINT_IMPULSES );
		h = lpHashWords( h, impulses, sizeof( float ) * (size_t)lpMinInt( count, LP_JOINT_IMPULSES ) );
	}
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
	LP_FIELD( h, p->source ); // where its leak's wave arrives
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

// ---- the whole, kept current ----

// A body slot's three element hashes (bodies, stress, backend; 0 where it has none)
static void lpHashBodySlot( const lpWorld* w, int i, uint64_t contacts, uint64_t out[3] )
{
	const lpBody* b = w->bodies.data + i;
	out[0] = 0;
	out[1] = 0;
	out[2] = 0;
	if ( b->alive )
	{
		out[0] = lpHashBodyElement( w, i );
		out[1] = lpHasStressState( b ) ? lpHashStressElement( w, i ) : 0;
		out[2] = LP_PHYS_NULL( b->id ) ? 0 : lpHashBackendElement( w, i, contacts );
	}
}

static const int lp_bodyCategories[3] = { lp_hashBodies, lp_hashStress, lp_hashBackend };

static int lpBodyCategory( int category )
{
	return category == lp_hashBodies ? 0 : ( category == lp_hashStress ? 1 : ( category == lp_hashBackend ? 2 : -1 ) );
}

// An element of the categories that are hashed whole each time (0: nothing in that slot)
static uint64_t lpHashSmallElement( const lpWorld* w, int category, int i )
{
	switch ( category )
	{
		case lp_hashWorld:
			return i == 0 ? lpHashWorldElement( w ) : 0;
		case lp_hashLinks:
			return i < w->links.count && w->links.data[i].alive ? lpHashLinkElement( w, i ) : 0;
		case lp_hashVehicles:
			return i < w->vehicles.count ? lpHashVehicleElement( w, i ) : 0;
		case lp_hashWheels:
			return i < w->wheels.count && w->wheels.data[i].link >= 0 ? lpHashWheelElement( w, i ) : 0;
		case lp_hashRigs:
			return i < w->rigs.count ? lpHashRigElement( w, i ) : 0;
		case lp_hashPools:
			return i < w->pools.count ? lpHashPoolElement( w, i ) : 0;
		case lp_hashDetonators:
			return i < w->detonators.count ? lpHashDetonatorElement( w, i ) : 0;
		default:
			return 0;
	}
}

int lpWorld_HashSlotCount( const lpWorld* w, int category )
{
	switch ( category )
	{
		case lp_hashWorld:
			return 1;
		case lp_hashBodies:
		case lp_hashStress:
		case lp_hashBackend:
			return w->bodies.count;
		case lp_hashPieces:
			return w->pieces.count;
		case lp_hashLinks:
			return w->links.count;
		case lp_hashVehicles:
			return w->vehicles.count;
		case lp_hashWheels:
			return w->wheels.count;
		case lp_hashRigs:
			return w->rigs.count;
		case lp_hashPools:
			return w->pools.count;
		case lp_hashDetonators:
			return w->detonators.count;
		default:
			return 0;
	}
}

void lpHashCategories( const lpWorld* w, uint64_t sums[lp_hashCategoryCount] )
{
	memset( sums, 0, sizeof( uint64_t ) * lp_hashCategoryCount );
	int bodies = w->bodies.count;
	uint64_t* contacts = lpAlloc( sizeof( uint64_t ) * (size_t)( bodies > 0 ? bodies : 1 ) );
	memset( contacts, 0, sizeof( uint64_t ) * (size_t)( bodies > 0 ? bodies : 1 ) );
	lpPhys_HashContacts( w->phys, NULL, 0, contacts, bodies );
	for ( int i = 0; i < bodies; ++i )
	{
		uint64_t slot[3];
		lpHashBodySlot( w, i, contacts[i], slot );
		for ( int k = 0; k < 3; ++k )
		{
			sums[lp_bodyCategories[k]] += slot[k];
		}
	}
	lpFree( contacts );
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		sums[lp_hashPieces] += lpHashPieceSlot( w, i );
	}
	for ( int c = 0; c < lp_hashCategoryCount; ++c )
	{
		for ( int i = 0; lpBodyCategory( c ) < 0 && c != lp_hashPieces && i < lpWorld_HashSlotCount( w, c ); ++i )
		{
			sums[c] += lpHashSmallElement( w, c, i );
		}
	}
}

#define LP_CONTACT_CHUNK 2048 // contact slots per job of the contact walk

typedef struct lpContactJob
{
	const lpWorld* world;
	int slots;
} lpContactJob;

static void lpRunContactChunk( int chunk, void* context )
{
	const lpContactJob* job = context;
	const lpHashCache* c = job->world->hash;
	int begin = chunk * LP_CONTACT_CHUNK;
	int end = lpMinInt( begin + LP_CONTACT_CHUNK, job->slots );
	lpPhys_HashContactRange( job->world->phys, begin, end, c->marked.data, LP_HASH_BODY, job->world->bodies.count, c->records.data + begin );
}

// The changed bodies' new element hashes, a chunk of them per job (pure: each job writes only its slots)
typedef struct lpRehash
{
	const lpWorld* world;
	const int* changed;
	int count;
	uint64_t* fresh; // three per changed body
} lpRehash;

#define LP_REHASH_CHUNK 64

static void lpRehashChunk( int chunk, void* context )
{
	const lpRehash* job = context;
	const lpWorld* w = job->world;
	const lpHashCache* c = w->hash;
	int end = lpMinInt( ( chunk + 1 ) * LP_REHASH_CHUNK, job->count );
	for ( int k = chunk * LP_REHASH_CHUNK; k < end; ++k )
	{
		int i = job->changed[k];
		const lpBody* b = w->bodies.data + i;
		uint8_t parts = c->marked.data[i];
		uint64_t* out = job->fresh + 3 * k;
		const uint64_t* old = c->slots.data + 3 * i;
		out[0] = old[0];
		out[1] = old[1];
		out[2] = old[2];
		if ( parts & LP_HASH_BODY )
		{
			out[0] = b->alive ? lpHashBodyElement( w, i ) : 0;
			out[2] = b->alive && LP_PHYS_NULL( b->id ) == false ? lpHashBackendElement( w, i, c->contacts.data[i] ) : 0;
		}
		if ( parts & LP_HASH_STRESS )
		{
			bool big = b->alive && lpHasStressState( b ) && lpBigStress( b ); // hashed after, its parts in parallel
			out[1] = b->alive && lpHasStressState( b ) ? ( big ? old[1] : lpHashStressElement( w, i ) ) : 0;
		}
	}
}

// The changed pieces' new element hashes, a chunk of them per job
typedef struct lpPieceRehash
{
	const lpWorld* world;
	uint64_t* fresh;
} lpPieceRehash;

static void lpRehashPieceChunk( int chunk, void* context )
{
	const lpPieceRehash* job = context;
	const lpHashCache* c = job->world->hash;
	int end = lpMinInt( ( chunk + 1 ) * LP_REHASH_CHUNK, c->pieceChanged.count );
	for ( int k = chunk * LP_REHASH_CHUNK; k < end; ++k )
	{
		job->fresh[k] = lpHashPieceSlot( job->world, c->pieceChanged.data[k] );
	}
}

static void lpHashFlushPieces( const lpWorld* w )
{
	lpHashCache* c = w->hash;
	int n = w->pieces.count;
	if ( c->pieceSlots.count < n )
	{
		int old = c->pieceSlots.count;
		lpArray_Reserve( c->pieceSlots, n );
		memset( c->pieceSlots.data + old, 0, sizeof( uint64_t ) * (size_t)( n - old ) );
		c->pieceSlots.count = n;
	}
	if ( c->pieceMarked.count < n )
	{
		int old = c->pieceMarked.count;
		lpArray_Reserve( c->pieceMarked, n );
		memset( c->pieceMarked.data + old, 0, (size_t)( n - old ) );
		c->pieceMarked.count = n;
	}
	int changes = c->pieceChanged.count;
	lpArray_Reserve( c->fresh, changes > 0 ? changes : 1 );
	lpPieceRehash job = { w, c->fresh.data };
	int chunks = ( changes + LP_REHASH_CHUNK - 1 ) / LP_REHASH_CHUNK;
	if ( chunks > 1 )
	{
		lpTaskPool_ParallelFor( w->tasks, chunks, lpRehashPieceChunk, &job );
	}
	else if ( chunks == 1 )
	{
		lpRehashPieceChunk( 0, &job );
	}
	for ( int k = 0; k < changes; ++k )
	{
		int i = c->pieceChanged.data[k];
		c->pieceSum += c->fresh.data[k] - c->pieceSlots.data[i];
		c->pieceSlots.data[i] = c->fresh.data[k];
		c->pieceMarked.data[i] = 0;
	}
	c->pieceChanged.count = 0;
}

// Brings the cached body slots up to date: every one the first time, then only the bodies marked since. A const world
// may: the cache is the world's, behind a pointer, and keeping it is not simulation state.
static void lpHashFlush( const lpWorld* w )
{
	lpHashCache* c = w->hash;
	int n = w->bodies.count;
	if ( c->slots.count < 3 * n )
	{
		int old = c->slots.count;
		lpArray_Reserve( c->slots, 3 * n );
		memset( c->slots.data + old, 0, sizeof( uint64_t ) * (size_t)( 3 * n - old ) );
		c->slots.count = 3 * n;
	}
	if ( c->marked.count < n )
	{
		int old = c->marked.count;
		lpArray_Reserve( c->marked, n );
		memset( c->marked.data + old, 0, (size_t)( n - old ) );
		c->marked.count = n;
	}
	lpArray_Reserve( c->contacts, n > 0 ? n : 1 );
	c->contacts.count = n;
	if ( c->valid == false )
	{
		memset( c->contacts.data, 0, sizeof( uint64_t ) * (size_t)n );
		lpPhys_HashContacts( w->phys, NULL, 0, c->contacts.data, n );
		memset( c->sums, 0, sizeof( c->sums ) );
		for ( int i = 0; i < n; ++i )
		{
			uint64_t* slot = c->slots.data + 3 * i;
			lpHashBodySlot( w, i, c->contacts.data[i], slot );
			c->sums[0] += slot[0];
			c->sums[1] += slot[1];
			c->sums[2] += slot[2];
		}
		memset( c->marked.data, 0, (size_t)n );
		c->changed.count = 0;
		lpPhys_ClearTouched( w->phys );
		int pieces = w->pieces.count;
		lpArray_Reserve( c->pieceSlots, pieces > 0 ? pieces : 1 );
		lpArray_Reserve( c->pieceMarked, pieces > 0 ? pieces : 1 );
		c->pieceSlots.count = pieces;
		c->pieceMarked.count = pieces;
		memset( c->pieceMarked.data, 0, (size_t)pieces );
		c->pieceSum = 0;
		for ( int i = 0; i < pieces; ++i )
		{
			c->pieceSlots.data[i] = lpHashPieceSlot( w, i );
			c->pieceSum += c->pieceSlots.data[i];
		}
		c->pieceChanged.count = 0;
		c->valid = true;
		return;
	}
	lpHashFlushPieces( w );
	// What calls changed in the engine since the last step (or flush)
	const int* touched;
	int count = lpPhys_GetTouched( w->phys, &touched );
	for ( int k = 0; k < count; ++k )
	{
		lpHashMark( (lpWorld*)w, touched[k] );
	}
	lpPhys_ClearTouched( w->phys );
	if ( c->changed.count == 0 )
	{
		return;
	}
	for ( int k = 0; k < c->changed.count; ++k )
	{
		c->contacts.data[c->changed.data[k]] = 0;
	}
	// Only the parts marked: a structure solving its stress rehashes its stress state, not its geometry
	int slots = lpPhys_GetContactSlotCount( w->phys );
	lpArray_Reserve( c->records, slots > 0 ? slots : 1 );
	lpContactJob contactJob = { w, slots };
	int contactChunks = ( slots + LP_CONTACT_CHUNK - 1 ) / LP_CONTACT_CHUNK;
	if ( contactChunks > 1 )
	{
		lpTaskPool_ParallelFor( w->tasks, contactChunks, lpRunContactChunk, &contactJob );
	}
	else if ( contactChunks == 1 )
	{
		lpRunContactChunk( 0, &contactJob );
	}
	for ( int k = 0; k < slots; ++k ) // added up after, so the sums are the same whatever the split
	{
		const lpPhysContactHash* r = c->records.data + k;
		if ( r->a >= 0 )
		{
			c->contacts.data[r->a] += r->hash;
		}
		if ( r->b >= 0 )
		{
			c->contacts.data[r->b] += r->hash;
		}
	}
	int changes = c->changed.count;
	lpArray_Reserve( c->fresh, 3 * changes );
	lpRehash job = { w, c->changed.data, changes, c->fresh.data };
	int chunks = ( changes + LP_REHASH_CHUNK - 1 ) / LP_REHASH_CHUNK;
	if ( chunks > 1 )
	{
		lpTaskPool_ParallelFor( w->tasks, chunks, lpRehashChunk, &job );
	}
	else
	{
		lpRehashChunk( 0, &job );
	}
	// A big structure's stress element, its parts in parallel
	for ( int k = 0; k < changes; ++k )
	{
		int i = c->changed.data[k];
		const lpBody* b = w->bodies.data + i;
		if ( ( c->marked.data[i] & LP_HASH_STRESS ) && b->alive && lpHasStressState( b ) && lpBigStress( b ) )
		{
			int parts = lpStressChunks( b );
			lpArray_Reserve( c->chunks, parts );
			lpStressChunkJob stressJob = { w, i, c->chunks.data };
			lpTaskPool_ParallelFor( w->tasks, parts, lpRunStressChunk, &stressJob );
			c->fresh.data[3 * k + 1] = lpHashStressCombine( lpHashStressHead( w, i ), c->chunks.data, parts );
		}
	}
	for ( int k = 0; k < changes; ++k )
	{
		int i = c->changed.data[k];
		uint64_t* slot = c->slots.data + 3 * i;
		for ( int j = 0; j < 3; ++j )
		{
			c->sums[j] += c->fresh.data[3 * k + j] - slot[j];
			slot[j] = c->fresh.data[3 * k + j];
		}
		c->marked.data[i] = 0;
	}
	c->changed.count = 0;
}

void lpHashTakeChanges( lpWorld* w )
{
	if ( w->hash->valid == false )
	{
		lpPhys_ClearTouched( w->phys );
		return;
	}
	// Box3D reports a move for every body it moved or put to sleep
	const lpPhysMove* moves;
	int count = lpPhys_GetMoves( w->phys, &moves );
	for ( int k = 0; k < count; ++k )
	{
		lpHashMark( w, moves[k].body );
	}
	const int* touched;
	count = lpPhys_GetTouched( w->phys, &touched );
	for ( int k = 0; k < count; ++k )
	{
		lpHashMark( w, touched[k] );
	}
	lpPhys_ClearTouched( w->phys );
}

void lpHashFree( lpWorld* w )
{
	lpArray_Free( w->hash->slots );
	lpArray_Free( w->hash->marked );
	lpArray_Free( w->hash->changed );
	lpArray_Free( w->hash->contacts );
	lpArray_Free( w->hash->fresh );
	lpArray_Free( w->hash->records );
	lpArray_Free( w->hash->chunks );
	lpArray_Free( w->hash->pieceSlots );
	lpArray_Free( w->hash->pieceMarked );
	lpArray_Free( w->hash->pieceChanged );
	lpFree( w->hash );
	w->hash = NULL;
}

void lpWorld_HashCategories( const lpWorld* w, uint64_t sums[lp_hashCategoryCount] )
{
	lpHashFlush( w );
	for ( int c = 0; c < lp_hashCategoryCount; ++c )
	{
		int k = lpBodyCategory( c );
		sums[c] = k >= 0 ? w->hash->sums[k] : ( c == lp_hashPieces ? w->hash->pieceSum : 0 );
		for ( int i = 0; k < 0 && c != lp_hashPieces && i < lpWorld_HashSlotCount( w, c ); ++i )
		{
			sums[c] += lpHashSmallElement( w, c, i );
		}
	}
}

uint64_t lpWorld_HashElement( const lpWorld* w, int category, int slot )
{
	int k = lpBodyCategory( category );
	if ( category == lp_hashPieces )
	{
		lpHashFlush( w );
		return slot >= 0 && slot < w->pieces.count ? w->hash->pieceSlots.data[slot] : 0;
	}
	if ( k < 0 )
	{
		return slot >= 0 ? lpHashSmallElement( w, category, slot ) : 0;
	}
	lpHashFlush( w );
	return slot >= 0 && slot < w->bodies.count ? w->hash->slots.data[3 * slot + k] : 0;
}

uint64_t lpWorld_HashBucket( const lpWorld* w, int category, int bucket )
{
	uint64_t sum = 0;
	int end = lpMinInt( 64 * bucket + 64, lpWorld_HashSlotCount( w, category ) );
	for ( int i = 64 * bucket; i < end; ++i )
	{
		sum += lpWorld_HashElement( w, category, i );
	}
	return sum;
}

static uint64_t lpHashRoot( const uint64_t sums[lp_hashCategoryCount] )
{
	uint64_t h = LP_HASH_INIT;
	for ( int c = 0; c < lp_hashCategoryCount; ++c )
	{
		h = lpMix64( h ^ sums[c] );
	}
	return h;
}

uint64_t lpWorld_Hash( const lpWorld* w )
{
	uint64_t sums[lp_hashCategoryCount];
	lpWorld_HashCategories( w, sums );
	return lpHashRoot( sums );
}

uint64_t lpWorld_HashStress( const lpWorld* w )
{
	lpHashFlush( w );
	return lpMix64( w->hash->sums[1] );
}

const char* lpHashCategoryName( int category )
{
	static const char* names[lp_hashCategoryCount] = { "world", "bodies",	  "pieces", "stress", "backend",   "links",
													   "vehicles", "wheels", "rigs",	  "pools",  "detonators" };
	return category >= 0 && category < lp_hashCategoryCount ? names[category] : "?";
}

bool lpWorld_CheckHash( const lpWorld* w, char* message, int size )
{
	uint64_t kept[lp_hashCategoryCount], full[lp_hashCategoryCount];
	lpWorld_HashCategories( w, kept );
	lpHashCategories( w, full );
	if ( memcmp( kept, full, sizeof( kept ) ) == 0 )
	{
		return true;
	}
	// Name the first piece or body slot whose kept hash is not what it hashes to now (the small categories are hashed
	// whole each time, so they cannot go stale)
	uint64_t* contacts = lpAlloc( sizeof( uint64_t ) * (size_t)( w->bodies.count > 0 ? w->bodies.count : 1 ) );
	memset( contacts, 0, sizeof( uint64_t ) * (size_t)( w->bodies.count > 0 ? w->bodies.count : 1 ) );
	lpPhys_HashContacts( w->phys, NULL, 0, contacts, w->bodies.count );
	snprintf( message, (size_t)size, "a category differs, no body or piece slot does" );
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		if ( lpHashPieceSlot( w, i ) != w->hash->pieceSlots.data[i] )
		{
			snprintf( message, (size_t)size, "tick %llu: pieces element %d (generation %u, body %d) changed unmarked",
					  (unsigned long long)w->tick, i, w->pieces.data[i].generation, w->pieces.data[i].body );
			lpFree( contacts );
			return false;
		}
	}
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		uint64_t now[3];
		lpHashBodySlot( w, i, contacts[i], now );
		for ( int k = 0; k < 3; ++k )
		{
			if ( now[k] != w->hash->slots.data[3 * i + k] )
			{
				const lpBody* b = w->bodies.data + i;
				snprintf( message, (size_t)size, "tick %llu: %s element %d (generation %u, kind %d, tier %d, alive %d) changed unmarked",
						  (unsigned long long)w->tick, lpHashCategoryName( lp_bodyCategories[k] ), i, b->generation, b->kind, b->tier, b->alive );
				lpFree( contacts );
				return false;
			}
		}
	}
	lpFree( contacts );
	return false;
}
