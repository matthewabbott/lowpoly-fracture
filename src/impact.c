// SPDX-License-Identifier: MIT
// Impacts: which pieces a blast, shot or hard collision reaches, fracturing them in three phases (see "fracture
// jobs"), damaging bonds, detonators, blast forces on debris, and turning Box3D hit events into impacts.

#include "tasks.h"
#include "world.h"

#include <math.h>
#include <stdio.h>

static b3AABB lpInflatedBox( b3Pos center, float r )
{
	b3Vec3 c = b3ToVec3( center );
	return (b3AABB){ b3Sub( c, (b3Vec3){ r, r, r } ), b3Add( c, (b3Vec3){ r, r, r } ) };
}

// Energy density (J/m^2) delivered at distance d from an impact
static float lpImpactDensity( const lpImpactDef* impact, float d )
{
	if ( d >= impact->radius )
	{
		return 0.0f;
	}
	float x = d < 0.0f ? 0.0f : d / impact->radius;
	float f = ( 1.0f - x ) * ( 1.0f - x );
	return impact->energy * f / ( B3_PI * impact->radius * impact->radius );
}

// ---- fracture jobs ----
//
// An impact fractures its pieces in three phases, which keeps the result independent of the thread count:
// 1. choose the pieces and snapshot their inputs (sequential, in piece order)
// 2. compute cells, Box3D hulls and sibling bonds for every piece (parallel; each job is a pure function of its input)
// 3. swap parents for their cells (sequential, in job order)

static void lpPrepareFractureJob( lpWorld* w, lpFractureJob* job, int pieceIndex, b3Vec3 localImpact, const lpImpactDef* impact )
{
	lpPiece* piece = w->pieces.data + pieceIndex;
	const lpMaterialDef* m = lpGetMaterial( piece->material );
	float fragment = m->fragmentSize * w->def.fragmentScale;

	// Radius inside which bonds will break: (1 - x)^2 >= strength * pi R^2 / E
	float ratio = m->bondStrength * B3_PI * impact->radius * impact->radius / impact->energy;
	float xb = ratio < 1.0f ? 1.0f - sqrtf( ratio ) : 0.0f;
	float breakRadius = b3MaxFloat( impact->radius * xb, 1.5f * fragment );

	job->piece = pieceIndex;
	job->localImpact = localImpact;
	job->center = piece->shape->centroid;
	lpShape_ToPoly( piece->shape, &job->poly );
	for ( int f = 0; f < job->poly.faceCount; ++f )
	{
		if ( job->poly.faces[f].tag >= 0 )
		{
			job->poly.faces[f].tag = LP_TAG_CUT; // cut faces of an earlier fracture
		}
	}
	lpPoly_Translate( &job->poly, b3Neg( job->center ) );
	job->particleVolume = lpParticleVolume( w, piece->material );
	job->ghostVolume = lpGhostVolume( w, piece->material );
	job->lightVolume = lpLightVolume( w, piece->material );
	job->cellCount = 0;
	job->bondCount = 0;
	memset( &job->stats, 0, sizeof( job->stats ) );

	lpFractureInput* input = &job->input;
	memset( input, 0, sizeof( *input ) );
	input->parent = &job->poly;
	input->impact = b3Sub( localImpact, job->center );
	input->radius = breakRadius;
	input->fragmentSize = fragment;
	input->maxCells = m->maxCells;
	input->plateSize = m->plateSize * w->def.fragmentScale;
	input->absorbVolume = b3MaxFloat( job->particleVolume, 0.1f * job->ghostVolume ); // slivers merge into neighbours
	input->pattern = (lpPatternId)m->pattern;
	input->axis = piece->axis;
	input->stretch = m->grainStretch;
	input->interiorMaterial = piece->material;
	input->seed = lpMix64( w->def.seed ^ ( w->tick << 24 ) ^ ( (uint64_t)pieceIndex << 1 ) ^ piece->generation );
	input->tolerance = 2e-5f;
}

// Phase 2. Must not touch the world. Cells inside the break radius are ejecta: their bonds would break anyway, so
// they skip bonding and connectivity and go straight to their tier. Puffs and ghosts need no Box3D hull at all.
static void lpRunFractureJob( int index, void* context )
{
	lpFractureJob* job = (lpFractureJob*)context + index;
	job->input.parent = &job->poly; // the job array may have moved since the job was prepared
	uint64_t ticks = b3GetTicks();
	job->cellCount = lpFracture( &job->input, job->cells, job->cellSites, LP_MAX_SITES, &job->stats );
	job->stats.voronoiMs = b3GetMillisecondsAndReset( &ticks );
	job->bondCount = 0;
	for ( int i = 0; i < job->cellCount; ++i )
	{
		job->hulls[i] = NULL; // the job slot is reused: never leave a stale hull for lpFreeJobOutput
	}
	if ( job->cellCount < 2 )
	{
		return;
	}
	float r2 = job->input.radius * job->input.radius;
	for ( int i = 0; i < job->cellCount; ++i )
	{
		lpShape* cell = job->cells[i];
		lpShape_Translate( cell, job->center );
		float volume = cell->volume;
		bool ejecta = b3DistanceSquared( cell->centroid, job->localImpact ) < r2;
		uint8_t cls;
		// Flying ejecta are real geometry down to the tiny particle volume; a sliver left on the piece turns to dust
		float dustBelow = ejecta ? job->particleVolume : job->input.absorbVolume;
		if ( volume < dustBelow )
		{
			cls = lp_cellPuff;
		}
		else if ( ejecta == false )
		{
			cls = lp_cellKeep;
		}
		else if ( volume < job->ghostVolume )
		{
			cls = lp_cellGhost;
		}
		else if ( volume < job->lightVolume )
		{
			cls = lp_cellLight;
		}
		else
		{
			cls = lp_cellFull;
		}
		job->cellClass[i] = cls;
	}

	// Cells that stay on the piece merge where their union is nearly convex: a log end becomes one piece
	float slack = lpGetMaterial( job->input.interiorMaterial )->mergeSlack;
	job->cellCount = lpMergeCells( job->cells, job->cellSites, job->cellClass, job->cellCount, lp_cellKeep, slack,
								   job->input.interiorMaterial );
	job->stats.mergeMs = b3GetMillisecondsAndReset( &ticks );
	for ( int i = 0; i < job->cellCount; ++i )
	{
		uint8_t cls = job->cellClass[i];
		bool needsHull = cls == lp_cellKeep || cls == lp_cellLight || cls == lp_cellFull;
		job->hulls[i] = needsHull ? lpShape_CreateHull( job->cells[i] ) : NULL;
	}
	job->stats.hullMs = b3GetMillisecondsAndReset( &ticks );
	job->bondCount = lpFindCellBonds( job->cells, job->cellSites, job->cellCount, job->bonds, LP_MAX_CELL_BONDS );

	// Ghost ejecta break into a few real chips: a dirtier spray for a few plane clips. After the bonds, which only
	// keepers use, so the chips need none. Wood splits along the grain, glass across the pane.
	const lpMaterialDef* m = lpGetMaterial( job->input.interiorMaterial );
	int original = job->cellCount;
	for ( int i = 0; i < original && m->chipSplits > 0; ++i )
	{
		if ( job->cellClass[i] != lp_cellGhost )
		{
			continue;
		}
		bool oriented = job->input.pattern == lp_breakGrain || job->input.pattern == lp_breakRadial;
		lpRandom rng;
		lpRandom_Seed( &rng, job->input.seed, 0xC41Full + (uint64_t)i );
		lpShape* chips[4];
		int room = LP_MAX_SITES - job->cellCount + 1;
		int count = lpChipCell( job->cells[i], m->chipSplits, oriented ? job->input.axis : b3Vec3_zero,
								job->input.interiorMaterial, job->particleVolume, &rng, chips, room < 4 ? room : 4 );
		if ( count == 0 )
		{
			continue;
		}
		lpShape_Destroy( job->cells[i] );
		job->cells[i] = chips[0];
		for ( int k = 1; k < count; ++k )
		{
			int c = job->cellCount++;
			job->cells[c] = chips[k];
			job->cellSites[c] = -1;
			job->cellClass[c] = lp_cellGhost;
			job->hulls[c] = NULL;
		}
	}
}

static void lpFreeJobOutput( lpFractureJob* job )
{
	for ( int i = 0; i < job->cellCount; ++i )
	{
		if ( job->cells[i] != NULL )
		{
			lpShape_Destroy( job->cells[i] );
		}
		if ( job->hulls[i] != NULL )
		{
			b3DestroyHull( job->hulls[i] );
		}
	}
	job->cellCount = 0;
}

// Phase 3: replace the parent piece by its cells. Ownership of cells and hulls moves to the new pieces.
static void lpIntegrateFractureJob( lpWorld* w, lpFractureJob* job )
{
	w->stats.clipFailures += job->stats.failureCount;
	w->stats.voronoiCpuMs += job->stats.voronoiMs;
	w->stats.mergeCpuMs += job->stats.mergeMs;
	w->stats.hullCpuMs += job->stats.hullMs;
	int pieceIndex = job->piece;
	lpPiece* piece = w->pieces.data + pieceIndex;
	if ( job->cellCount < 2 || piece->body < 0 )
	{
		lpFreeJobOutput( job );
		return;
	}

	int bodyIndex = piece->body;
	lpBody* body = w->bodies.data + bodyIndex;
	b3WorldTransform xf = b3Body_GetTransform( body->id );
	bool isDynamic = body->kind == lp_kindDebris;
	b3Vec3 v = isDynamic ? b3Body_GetLinearVelocity( body->id ) : b3Vec3_zero;
	b3Vec3 omega = isDynamic ? b3Body_GetAngularVelocity( body->id ) : b3Vec3_zero;
	b3Vec3 localCenter = isDynamic ? b3Body_GetLocalCenter( body->id ) : b3Vec3_zero;

	// Former neighbors, then retire the parent
	int neighbors[256];
	int neighborCount = 0;
	while ( piece->bonds.count > 0 )
	{
		int bondIndex = piece->bonds.data[piece->bonds.count - 1];
		lpBond* bond = w->bonds.data + bondIndex;
		int other = bond->a == pieceIndex ? bond->b : bond->a;
		if ( neighborCount < 256 )
		{
			neighbors[neighborCount++] = other;
		}
		lpBreakBond( w, bondIndex );
	}

	uint8_t material = piece->material;
	uint8_t joint = piece->joint;
	uint32_t color = piece->color;
	b3Vec3 axis = piece->axis;
	int depth = piece->depth + 1;
	bool anchored = piece->anchored;
	b3Plane anchorPlane = piece->anchorPlane;

	uint64_t shapeTicks = b3GetTicks();
	lpDetachPieceShape( w, pieceIndex );
	w->stats.shapeMs += b3GetMilliseconds( shapeTicks );
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		if ( body->pieces.data[i] == pieceIndex )
		{
			memmove( body->pieces.data + i, body->pieces.data + i + 1, sizeof( int ) * (size_t)( body->pieces.count - i - 1 ) );
			body->pieces.count -= 1;
			break;
		}
	}
	body->volume -= piece->shape->volume;
	lpFreePieceSlot( w, pieceIndex );
	piece = NULL;

	int cellToPiece[LP_MAX_SITES];
	int children[LP_MAX_SITES];
	int childCount = 0;
	int ejected = 0;
	float ejectedVolume = 0.0f;
	for ( int i = 0; i < job->cellCount; ++i )
	{
		lpShape* cell = job->cells[i];
		b3HullData* hull = job->hulls[i];
		uint8_t cls = job->cellClass[i];
		job->cells[i] = NULL;
		job->hulls[i] = NULL;
		cellToPiece[i] = -1;
		if ( ( cls == lp_cellKeep || cls == lp_cellLight || cls == lp_cellFull ) && hull == NULL )
		{
			cls = lp_cellPuff; // no valid hull (a sliver)
		}

		// Velocity of the parent at the cell
		b3Vec3 cellV = v;
		if ( isDynamic )
		{
			cellV = b3Add( v, b3Cross( omega, b3RotateVector( xf.q, b3Sub( cell->centroid, localCenter ) ) ) );
		}

		if ( cls != lp_cellKeep )
		{
			ejectedVolume += cell->volume;
		}
		if ( cls == lp_cellPuff )
		{
			b3Vec3 away = b3Normalize( b3Sub( cell->centroid, job->localImpact ) );
			b3Vec3 pv = b3Add( cellV, b3RotateVector( xf.q, b3MulSV( 2.0f, away ) ) );
			lpEmitParticle( w, xf, cell->centroid, pv, cbrtf( cell->volume ), material );
			lpShape_Destroy( cell );
			if ( hull != NULL )
			{
				b3DestroyHull( hull );
			}
			continue;
		}

		int childIndex = lpAllocPiece( w );
		lpPiece* child = w->pieces.data + childIndex;
		child->shape = cell;
		child->hull = hull;
		child->material = material;
		child->color = color;
		child->axis = axis;
		child->joint = joint;
		child->depth = (uint8_t)( depth > 255 ? 255 : depth );
		child->seed = (uint32_t)lpMix64( w->def.seed ^ w->pieceSerial++ );
		child->anchorPlane = anchorPlane;

		if ( cls == lp_cellKeep )
		{
			child->anchored = anchored && lpShape_HasFaceOnPlane( cell, anchorPlane, 1e-3f );
			if ( lpAttachPiece( w, childIndex, bodyIndex ) == false )
			{
				lpFreePieceSlot( w, childIndex );
				continue;
			}
			cellToPiece[i] = childIndex;
			children[childCount++] = childIndex;
			continue;
		}

		ejected += 1;
		if ( cls == lp_cellGhost )
		{
			// A little tumble, seeded from the piece so it is deterministic
			lpRandom rng;
			lpRandom_Seed( &rng, child->seed, 17 );
			b3Vec3 spin = { lpRandom_Range( &rng, -6.0f, 6.0f ), lpRandom_Range( &rng, -6.0f, 6.0f ), lpRandom_Range( &rng, -6.0f, 6.0f ) };
			// and a small kick away from the impact, so chips of one cell spread instead of flying as a clump
			b3Vec3 away = b3Normalize( b3Sub( cell->centroid, job->localImpact ) );
			b3Vec3 kick = b3RotateVector( xf.q, b3MulSV( lpRandom_Range( &rng, 0.5f, 2.0f ), away ) );
			int ghost = lpBeginGhost( w, xf, b3Add( cellV, kick ), b3Add( omega, spin ) );
			lpAddLoosePiece( w, ghost, childIndex );
			lpFinishLoose( w, ghost, xf );
		}
		else
		{
			uint8_t tier = cls == lp_cellLight ? lp_tierLight : lp_tierFull;
			int debris = lpCreateBodyInternal( w, xf, b3_dynamicBody, lp_kindDebris, tier, cellV, omega );
			if ( lpAttachPiece( w, childIndex, debris ) )
			{
				b3Body_ApplyMassFromShapes( w->bodies.data[debris].id );
			}
			else
			{
				lpFreePieceSlot( w, childIndex );
				lpDestroyBody( w, debris, false );
			}
		}
		body = w->bodies.data + bodyIndex; // the body array may have moved
	}
	job->cellCount = 0;

	// Impact dust in the colour of what broke. Cosmetic: it hashes its own randomness from the tick and the piece and
	// never touches simulation state.
	if ( ejectedVolume > 0.0f )
	{
		int motes = 3 + (int)( 9.0f * b3MinFloat( 1.0f, ejectedVolume / 0.02f ) );
		uint64_t h = lpMix64( ( w->tick << 20 ) ^ (uint64_t)pieceIndex );
		for ( int k = 0; k < motes; ++k )
		{
			h = lpMix64( h + (uint64_t)k );
			float rx = (float)( h & 0xFFFF ) / 65535.0f - 0.5f;
			float ry = (float)( ( h >> 16 ) & 0xFFFF ) / 65535.0f;
			float rz = (float)( ( h >> 32 ) & 0xFFFF ) / 65535.0f - 0.5f;
			float rs = (float)( ( h >> 48 ) & 0xFFFF ) / 65535.0f;
			b3Vec3 dustV = b3Add( v, (b3Vec3){ 4.0f * rx, 0.5f + 2.5f * ry, 4.0f * rz } );
			lpEmitParticle( w, xf, job->localImpact, dustV, 0.02f + 0.03f * rs, material );
		}
	}

	uint64_t bondTicks = b3GetTicks();
	for ( int i = 0; i < job->bondCount; ++i )
	{
		lpCellBond cb = job->bonds[i];
		int a = cellToPiece[cb.a];
		int b = cellToPiece[cb.b];
		if ( a >= 0 && b >= 0 )
		{
			lpAddBond( w, a, b, &cb.contact, lp_jointSolid ); // cells of one piece: its own material holds them
		}
	}
	for ( int i = 0; i < childCount; ++i )
	{
		for ( int n = 0; n < neighborCount; ++n )
		{
			lpTryBond( w, children[i], neighbors[n] );
		}
	}
	w->stats.bondMs += b3GetMilliseconds( bondTicks );

	if ( body->kind != lp_kindStructure && body->pieces.count > 0 )
	{
		b3Body_ApplyMassFromShapes( body->id );
	}

	w->stats.fracturesThisStep += 1;
	w->stats.cellsThisStep += childCount + ejected;
	lpMarkDirty( w, bodyIndex );
}

// Queue the blast of an armed body for the next step and remove the body then.
static void lpDetonate( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->alive == false || b->armed == false )
	{
		return;
	}
	b->armed = false;
	lpImpactDef blast = { 0 };
	blast.point = B3_IS_NON_NULL( b->id ) ? b3Body_GetWorldCenter( b->id ) : b->com;
	blast.radius = b->detonator.radius;
	blast.energy = b->detonator.energy;
	blast.impulse = b->detonator.speed;
	blast.explosion = true;
	lpArray_Push( w->nextImpacts, blast );
	lpArray_Push( w->pendingDestroy, bodyIndex );
}

typedef struct lpFractureCandidate
{
	uint32_t distanceBits; // non-negative floats order like their bits
	int piece;
	b3Vec3 local;
} lpFractureCandidate;

static int lpCompareFractureCandidates( const void* a, const void* b )
{
	const lpFractureCandidate* x = a;
	const lpFractureCandidate* y = b;
	if ( x->distanceBits != y->distanceBits )
	{
		return x->distanceBits < y->distanceBits ? -1 : 1;
	}
	return ( x->piece > y->piece ) - ( x->piece < y->piece );
}

static lpFractureJob* lpNextJob( lpWorld* w )
{
	if ( w->jobCount == w->jobCapacity )
	{
		int capacity = w->jobCapacity < 8 ? 8 : 2 * w->jobCapacity;
		w->jobs = lpRealloc( w->jobs, sizeof( lpFractureJob ) * (size_t)capacity );
		for ( int k = w->jobCapacity; k < capacity; ++k )
		{
			w->jobs[k].bonds = lpAlloc( sizeof( lpCellBond ) * LP_MAX_CELL_BONDS );
		}
		w->jobCapacity = capacity;
	}
	return w->jobs + w->jobCount++;
}

// Refracture the qualifying candidates of an impact, nearest first, within the step's job budget. The rest wait
// for the next step (the hole appears now, its outer refractures one step later). Deterministic: the budget is a
// count and every order is a total order.
static void lpFractureCandidates( lpWorld* w, const lpImpactDef* impact, uint32_t serial, const int* candidates, int count )
{
	lpFractureCandidate* list = lpAlloc( sizeof( lpFractureCandidate ) * (size_t)( count > 0 ? count : 1 ) );
	int n = 0;
	for ( int i = 0; i < count; ++i )
	{
		int pieceIndex = candidates[i];
		lpPiece* p = w->pieces.data + pieceIndex;
		if ( p->body < 0 || p->shape == NULL || B3_IS_NULL( p->shapeId ) )
		{
			continue;
		}
		const lpMaterialDef* m = lpGetMaterial( p->material );
		if ( m->breakable == false || p->depth >= w->def.maxDepth )
		{
			continue;
		}
		float fragment = m->fragmentSize * w->def.fragmentScale;
		if ( p->shape->radius < 1.5f * fragment )
		{
			continue;
		}
		b3WorldTransform xf = b3Body_GetTransform( w->bodies.data[p->body].id );
		b3Vec3 local = b3InvTransformWorldPoint( xf, impact->point );
		float d = lpShape_SignedDistance( p->shape, local );
		if ( lpImpactDensity( impact, d ) >= m->fractureEnergy )
		{
			float key = d > 0.0f ? d : 0.0f;
			uint32_t bits;
			memcpy( &bits, &key, sizeof( bits ) );
			list[n++] = (lpFractureCandidate){ bits, pieceIndex, local };
		}
	}
	if ( n > 1 )
	{
		qsort( list, (size_t)n, sizeof( lpFractureCandidate ), lpCompareFractureCandidates );
	}

	int budget = w->def.maxFractureJobsPerStep - w->jobsThisStep;
	budget = budget < 0 ? 0 : budget;
	w->jobCount = 0;
	for ( int i = 0; i < n; ++i )
	{
		if ( i < budget )
		{
			lpPrepareFractureJob( w, lpNextJob( w ), list[i].piece, list[i].local, impact );
		}
		else
		{
			lpDeferredJob deferred = { list[i].piece, w->pieces.data[list[i].piece].generation, serial, *impact };
			lpArray_Push( w->deferred, deferred );
		}
	}
	lpFree( list );
	w->jobsThisStep += w->jobCount;

	uint64_t cellTicks = b3GetTicks();
	lpTaskPool_ParallelFor( w->tasks, w->jobCount, lpRunFractureJob, w->jobs );
	w->stats.cellMs += b3GetMilliseconds( cellTicks );

	for ( int i = 0; i < w->jobCount; ++i )
	{
		lpIntegrateFractureJob( w, w->jobs + i );
	}
	w->jobCount = 0;
}

// Damage bonds near an impact (including fresh ones). Each bond takes each impact at most once, even when part of
// the impact's fracture work was deferred to a later step.
static void lpDamageBonds( lpWorld* w, const lpImpactDef* impact, uint32_t serial )
{
	lpQueryPieces( w, lpInflatedBox( impact->point, impact->radius ) );
	for ( int i = 0; i < w->scratchPieces.count; ++i )
	{
		lpPiece* p = w->pieces.data + w->scratchPieces.data[i];
		if ( p->body < 0 )
		{
			continue;
		}
		b3WorldTransform xf = b3Body_GetTransform( w->bodies.data[p->body].id );
		b3Vec3 local = b3InvTransformWorldPoint( xf, impact->point );

		for ( int k = 0; k < p->bonds.count; )
		{
			int bondIndex = p->bonds.data[k];
			lpBond* bond = w->bonds.data + bondIndex;
			if ( bond->lastImpact == serial )
			{
				k += 1;
				continue;
			}
			bond->lastImpact = serial;
			float density = lpImpactDensity( impact, b3Distance( bond->centroid, local ) );
			bond->health -= density;
			if ( bond->health <= 0.0f )
			{
				lpBreakBond( w, bondIndex ); // removes it from p->bonds, so k stays
				lpMarkDirty( w, p->body );
			}
			else
			{
				k += 1;
			}
		}
	}
}

void lpProcessImpact( lpWorld* w, const lpImpactDef* impact )
{
	if ( impact->radius <= 0.0f || impact->energy <= 0.0f )
	{
		return;
	}
	w->impactSerial += 1;
	uint32_t serial = w->impactSerial;

	lpQueryPieces( w, lpInflatedBox( impact->point, impact->radius ) );

	// Copy: fracturing reuses the scratch query array
	int candidateCount = w->scratchPieces.count;
	int* candidates = lpAlloc( sizeof( int ) * (size_t)( candidateCount > 0 ? candidateCount : 1 ) );
	memcpy( candidates, w->scratchPieces.data, sizeof( int ) * (size_t)candidateCount );

	// Anything frozen nearby comes back to life; volatile things caught in the blast go off (next step)
	for ( int i = 0; i < candidateCount; ++i )
	{
		lpPiece* p = w->pieces.data + candidates[i];
		if ( p->body < 0 )
		{
			continue;
		}
		lpWakeRubble( w, p->body );
		lpBody* b = w->bodies.data + p->body;
		if ( b->armed )
		{
			b3WorldTransform xf = b3Body_GetTransform( b->id );
			float d = lpShape_SignedDistance( p->shape, b3InvTransformWorldPoint( xf, impact->point ) );
			if ( lpImpactDensity( impact, d ) > 150.0f )
			{
				lpDetonate( w, p->body );
			}
		}
	}

	lpFractureCandidates( w, impact, serial, candidates, candidateCount );
	lpFree( candidates );
	lpDamageBonds( w, impact, serial );

	w->stats.impactsThisStep += 1;
	if ( w->def.debugLog )
	{
		printf( "[lpf] tick %llu impact at (%.2f %.2f %.2f) r %.2f E %.0f%s: %d candidates, fractures so far %d\n",
				(unsigned long long)w->tick, (double)impact->point.x, (double)impact->point.y, (double)impact->point.z,
				(double)impact->radius, (double)impact->energy, impact->explosion ? " (blast)" : "", candidateCount,
				w->stats.fracturesThisStep );
	}
}

// Fracture work left over from earlier steps goes first, grouped by the impact it belongs to
void lpProcessDeferred( lpWorld* w )
{
	if ( w->deferred.count == 0 )
	{
		return;
	}
	int count = w->deferred.count;
	lpDeferredJob* pending = lpAlloc( sizeof( lpDeferredJob ) * (size_t)count );
	memcpy( pending, w->deferred.data, sizeof( lpDeferredJob ) * (size_t)count );
	w->deferred.count = 0;

	int* pieces = lpAlloc( sizeof( int ) * (size_t)count );
	for ( int first = 0; first < count; )
	{
		int last = first;
		while ( last + 1 < count && pending[last + 1].impactSerial == pending[first].impactSerial )
		{
			last += 1;
		}
		int n = 0;
		for ( int i = first; i <= last; ++i )
		{
			lpPiece* p = w->pieces.data + pending[i].piece;
			if ( p->body >= 0 && p->generation == pending[i].generation )
			{
				pieces[n++] = pending[i].piece;
			}
		}
		if ( n > 1 )
		{
			qsort( pieces, (size_t)n, sizeof( int ), lpCompareInt );
		}
		lpImpactDef impact = pending[first].impact;
		lpFractureCandidates( w, &impact, pending[first].impactSerial, pieces, n );
		lpDamageBonds( w, &impact, pending[first].impactSerial );
		first = last + 1;
	}
	lpFree( pieces );
	lpFree( pending );
}

void lpApplyForces( lpWorld* w )
{
	for ( int i = 0; i < w->forces.count; ++i )
	{
		lpForce force = w->forces.data[i];
		if ( force.impulse <= 0.0f )
		{
			continue;
		}

		// One impulse per loose body near the impact. Explosions push radially and set a speed (m/s, falling off with
		// distance) so light and heavy chunks fly alike; directional hits give an impulse (N*s) capped at 12 m/s.
		// The radial push is applied slightly off center toward the blast, which gives flying chunks some spin.
		lpQueryPieces( w, lpInflatedBox( force.point, force.radius ) );
		w->stamp += 1;
		for ( int k = 0; k < w->scratchPieces.count; ++k )
		{
			lpPiece* p = w->pieces.data + w->scratchPieces.data[k];
			if ( p->body < 0 )
			{
				continue;
			}
			lpBody* b = w->bodies.data + p->body;
			if ( b->kind != lp_kindDebris || b->stamp == w->stamp )
			{
				continue;
			}
			b->stamp = w->stamp;
			b3Pos center = b3Body_GetWorldCenter( b->id );
			float d = b3Length( b3SubPos( center, force.point ) );
			if ( d > force.radius )
			{
				continue;
			}
			float f = 1.0f - d / force.radius;
			float mass = b3Body_GetMass( b->id );
			if ( force.explosion )
			{
				b3Vec3 away = d > 1e-4f ? b3MulSV( 1.0f / d, b3SubPos( center, force.point ) ) : (b3Vec3){ 0.0f, 1.0f, 0.0f };
				// A blast on a surface breaches it: bias the push along the incoming direction, plus a little lift
				away = b3Add( away, force.direction );
				away = b3Normalize( b3Add( away, (b3Vec3){ 0.0f, 0.35f, 0.0f } ) );
				float speed = force.impulse * f;
				b3Vec3 extent = b3Body_GetMaxExtent( b->id );
				b3Pos at = b3OffsetPos( center, b3MulSV( -0.3f * b3Length( extent ), away ) );
				b3Body_ApplyLinearImpulse( b->id, b3MulSV( mass * speed, away ), at, true );
			}
			else
			{
				float impulse = b3MinFloat( force.impulse * f, 12.0f * mass );
				b3Body_ApplyLinearImpulse( b->id, b3MulSV( impulse, force.direction ), center, true );
			}
		}
		lpApplyLooseForce( w, &force );
	}
	w->forces.count = 0;
}

static int lpCompareHits( const void* a, const void* b )
{
	const lpHitCandidate* x = a;
	const lpHitCandidate* y = b;
	if ( x->energy != y->energy )
	{
		return x->energy > y->energy ? -1 : 1;
	}
	return ( x->key > y->key ) - ( x->key < y->key );
}

static float lpShapeMass( b3ShapeId shapeId )
{
	b3BodyId body = b3Shape_GetBody( shapeId );
	return b3Body_GetType( body ) == b3_dynamicBody ? b3Body_GetMass( body ) : 0.0f;
}

// Collisions hard enough to hurt become impacts for the next step
void lpCollectHits( lpWorld* w )
{
	b3ContactEvents events = b3World_GetContactEvents( w->def.physics );
	w->scratchHits.count = 0;
	for ( int i = 0; i < events.hitCount; ++i )
	{
		const b3ContactHitEvent* e = events.hitEvents + i;
		if ( e->approachSpeed < w->def.wakeSpeed )
		{
			continue;
		}
		intptr_t da = (intptr_t)b3Shape_GetUserData( e->shapeIdA );
		intptr_t db = (intptr_t)b3Shape_GetUserData( e->shapeIdB );
		if ( da <= 0 && db <= 0 )
		{
			continue;
		}

		// Fragile rubble: a moving body bumping into it knocks it loose (strong static friction, not cement)
		if ( da > 0 && db > 0 )
		{
			int bodyA = w->pieces.data[da - 1].body;
			int bodyB = w->pieces.data[db - 1].body;
			if ( w->bodies.data[bodyA].kind == lp_kindRubble && w->bodies.data[bodyB].kind == lp_kindDebris )
			{
				lpWakeRubble( w, bodyA );
			}
			else if ( w->bodies.data[bodyB].kind == lp_kindRubble && w->bodies.data[bodyA].kind == lp_kindDebris )
			{
				lpWakeRubble( w, bodyB );
			}
		}
		if ( e->approachSpeed < w->def.hitSpeed )
		{
			continue;
		}

		// Volatile objects go off on a hard enough knock
		intptr_t pieceData[2] = { da, db };
		for ( int k = 0; k < 2; ++k )
		{
			if ( pieceData[k] > 0 )
			{
				int bodyIndex = w->pieces.data[pieceData[k] - 1].body;
				lpBody* b = w->bodies.data + bodyIndex;
				if ( b->armed && e->approachSpeed >= b->detonator.triggerSpeed )
				{
					lpDetonate( w, bodyIndex );
				}
			}
		}

		float ma = lpShapeMass( e->shapeIdA );
		float mb = lpShapeMass( e->shapeIdB );
		float mass = ( ma > 0.0f && mb > 0.0f ) ? ma * mb / ( ma + mb ) : b3MaxFloat( ma, mb );
		float energy = 0.5f * mass * e->approachSpeed * e->approachSpeed;
		if ( energy < 100.0f )
		{
			continue;
		}

		lpHitCandidate hit = { energy, e->point, ( (uint64_t)( da > 0 ? da : 0 ) << 32 ) | (uint64_t)( db > 0 ? db : 0 ) };
		lpArray_Push( w->scratchHits, hit );
	}

	if ( w->scratchHits.count > 1 )
	{
		qsort( w->scratchHits.data, (size_t)w->scratchHits.count, sizeof( lpHitCandidate ), lpCompareHits );
	}

	int count = w->scratchHits.count < w->def.maxHitImpacts ? w->scratchHits.count : w->def.maxHitImpacts;
	for ( int i = 0; i < count; ++i )
	{
		lpHitCandidate hit = w->scratchHits.data[i];
		lpImpactDef impact = { 0 };
		impact.point = hit.point;
		impact.radius = b3ClampFloat( 0.06f * cbrtf( hit.energy ), 0.15f, 1.2f );
		impact.energy = hit.energy;
		lpArray_Push( w->nextImpacts, impact );
	}
}
