// SPDX-License-Identifier: MIT
// Stress: a quasi-static solve on each structure's bond graph, so buildings fall where they are weak.
//
// Pieces are rigid nodes with a small translation and rotation; anchored pieces are fixed to the world. A bond is a
// short beam through its contact patch with axial, shear, bending and twisting stiffness from the patch's area and
// extents (the parallel-bond model of rock and masonry simulation). Solving K x = gravity gives every bond's force
// and moment, hence the tension, compression and shear at its most loaded fibre. Mortar and dry joints carry almost
// no tension, so an overhang's moment opens the tension side first: the part above hinges off, loses its anchor and
// the physics topples it.
//
// Stiffness is normalized (only ratios decide how load is shared), so forces come out in newtons and displacements
// are unitless. The solve is block-Jacobi-preconditioned conjugate gradient, warm-started from each piece's last
// solution and cut off by a per-step work budget: a structure that needs longer keeps "creaking" for a few steps. A
// bond over its limit accumulates strain on each converged check and breaks when strain reaches 1, the worst few per
// check, so failure cascades as the structure re-solves.
//
// Every structure updated in a step is checked together, in three phases (like fracture jobs, impact.c):
// 1. in queue order: the shortcuts that need no solve, and each structure's share of the step's budget
// 2. in parallel: build (unless continuing on the body's system) and solve each structure, and from a converged
//    solution every joint's utilization and the slender pieces' worst sections (a pure function of its own pieces and
//    bonds; solve.c has the math)
// 3. in queue order: judge each solution, strain and break joints
// Results do not depend on the worker count. Settling (lpWorld_SettleStructures, at load) runs the same check with no
// budget, so new structures start converged.
//
// Big structures (past stressLargeNodes) solve changes on their load-bearing skeleton: after an exact solve their
// lightly loaded parts form rigid clusters, and a change is solved on the reduced system for the correction to the
// last solution, with the pieces near the change resolved finely and a residual meter dissolving clusters that would
// bias it (architecture.md, "Rigid clusters and the delta form").

#include "tasks.h"
#include "world.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

static const lpVec6 lp_vec6Zero = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };

// Fixed in its body's solve: anchored on a structure, the pin on a moving body
static inline bool lpFixed( const lpBody* body, int pieceIndex, const lpPiece* p )
{
	return body->solveStress ? pieceIndex == body->stressPin : p->anchored;
}

// A moving body's pin: the piece nearest where it was struck in the last step (the crash's force enters there), else
// the piece nearest its centre of mass (ties by index); a solve in progress keeps its pin. And that centre.
static void lpChoosePin( lpWorld* w, lpBody* body, int bodyIndex )
{
	float mass = 0.0f;
	lpVec3 center = lpVec3_zero;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + body->pieces.data[i];
		float m = p->shape->volume * lpGetMaterial( p->material )->density;
		center = lpMulAdd( center, m, p->shape->centroid );
		mass += m;
	}
	center = mass > 0.0f ? lpMulSV( 1.0f / mass, center ) : center;
	body->reliefCenter = center;
	if ( body->solving && body->stressPin >= 0 && w->pieces.data[body->stressPin].body == bodyIndex )
	{
		return;
	}
	lpVec3 at = body->hitTick == w->tick ? body->hitPoint : center; // struck in the last step: where it was struck
	float nearest = FLT_MAX;
	body->stressPin = -1;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		float d = lpDistanceSquared( w->pieces.data[pi].shape->centroid, at );
		if ( d < nearest || ( d == nearest && pi < body->stressPin ) )
		{
			nearest = d;
			body->stressPin = pi;
		}
	}
}

void lpTrackMovingBodies( lpWorld* w )
{
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		if ( b->alive == false || b->solveStress == false || b->kind != lp_kindDebris || LP_PHYS_NULL( b->id ) )
		{
			continue;
		}
		b->stepPair = b->stepTick != 0 && b->stepTick == w->tick; // the record being kept is from the step before
		b->stepV[0] = b->stepV[1];
		b->stepOmega[0] = b->stepOmega[1];
		b->stepV[1] = lpPhys_GetLinearVelocity( w->phys, b->id );
		b->stepOmega[1] = lpPhys_GetAngularVelocity( w->phys, b->id );
		b->stepTick = w->tick + 1;
	}
}

// Inertia relief: the acceleration (a, alpha) that balances a moving body's weight and sampled loads exactly, with each
// piece a point mass at its centroid plus a small inertia of its own (so no line of pieces is singular). Its pieces then
// carry their loads less m (a + alpha x r + omega x (omega x r)), and less their own inertia times alpha: the pin
// carries nothing.
static void lpComputeRelief( lpWorld* w, lpBody* body, lpVec3 gravity )
{
	// Measured when it can be: the velocity change over the last step is the acceleration everything gave it, sampled
	// or not (whatever was not sampled then enters at the pin)
	lpQuat q = lpPhys_GetTransform( w->phys, body->id ).q;
	if ( body->stepPair && body->stepTick == w->tick && w->lastTimeStep > 0.0f )
	{
		float inv = 1.0f / w->lastTimeStep;
		lpVec3 accel = lpInvRotateVector( q, lpMulSV( inv, lpSub( body->stepV[1], body->stepV[0] ) ) );
		lpVec3 alpha = lpInvRotateVector( q, lpMulSV( inv, lpSub( body->stepOmega[1], body->stepOmega[0] ) ) );
		// A rigid body stops in a step; a crumple zone takes several: the harder part of the stop is spread by the struck
		// material's crush (what gravity did stays)
		float spread = body->hitTick == w->tick ? 1.0f - lpGetMaterial( body->hitMaterial )->crush : 1.0f;
		body->reliefAccel = lpMulAdd( gravity, spread, lpSub( accel, gravity ) );
		body->reliefAlpha = lpMulSV( spread, alpha );
		body->reliefOmega = lpInvRotateVector( q, body->stepOmega[1] );
		return;
	}
	lpVec3 c = body->reliefCenter;
	float mass = 0.0f;
	lpVec3 force = lpVec3_zero;
	lpVec3 torque = lpVec3_zero;
	lpMatrix3 inertia = { lpVec3_zero, lpVec3_zero, lpVec3_zero };
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + body->pieces.data[i];
		float m = p->shape->volume * lpGetMaterial( p->material )->density;
		lpVec3 r = lpSub( p->shape->centroid, c );
		lpVec3 load = lpMulAdd( p->stressLoad.f, m, gravity );
		mass += m;
		force = lpAdd( force, load );
		torque = lpAdd( torque, lpAdd( lpCross( r, load ), p->stressLoad.t ) );
		float own = m * lpCbrt( p->shape->volume ) * lpCbrt( p->shape->volume ) / 6.0f;
		float rr = lpDot( r, r );
		inertia.cx = lpAdd( inertia.cx, (lpVec3){ m * ( rr - r.x * r.x ) + own, -m * r.y * r.x, -m * r.z * r.x } );
		inertia.cy = lpAdd( inertia.cy, (lpVec3){ -m * r.x * r.y, m * ( rr - r.y * r.y ) + own, -m * r.z * r.y } );
		inertia.cz = lpAdd( inertia.cz, (lpVec3){ -m * r.x * r.z, -m * r.y * r.z, m * ( rr - r.z * r.z ) + own } );
	}
	lpQuat rotation = lpPhys_GetTransform( w->phys, body->id ).q;
	lpVec3 omega = lpInvRotateVector( rotation, lpPhys_GetAngularVelocity( w->phys, body->id ) );
	body->reliefOmega = omega;
	body->reliefAccel = mass > 0.0f ? lpMulSV( 1.0f / mass, force ) : lpVec3_zero;
	body->reliefAlpha = lpMulMV( lpInvertMatrix( inertia ), lpSub( torque, lpCross( omega, lpMulMV( inertia, omega ) ) ) );
}

// Rigid clusters (lpFormClusters, lpStressMeter)
#define LP_CLUSTER_MEMBERS 128	   // pieces in a cluster at most
#define LP_CLUSTER_RADIUS 12.0f	   // mean piece sizes from the middle of a cluster's bounds to its corners, at most
#define LP_METER_ROUNDS 2		   // times a solve may dissolve clusters and run again before it is judged
#define LP_SEED_HOPS 3			   // bonds from a changed piece within which pieces leave their clusters
#define LP_METER_DISSOLVE 0.5f	   // a cluster whose joints could be carried to this utilization dissolves (above the glue)
#define LP_METER_CHANGE 0.25f	   // or whose load changed by this share of what its most loaded member carries

// Audits (lpStressAudits): a provisional structure is solved exactly once the stress budget has been at most half used,
// with nothing waiting, for LP_CALM_STEPS steps, or once it has waited LP_AUDIT_AGE steps whatever the load. An audit
// takes at most LP_AUDIT_SHARE of the step's budget, and gives way if the structure changes while it runs (it stays
// provisional, first in line).
#define LP_CALM_STEPS 30
#define LP_AUDIT_AGE 300
#define LP_AUDIT_SHARE 0.5f

// What rests on a structure: dynamic bodies pressing on its pieces, from the last physics step's contact impulses
// (rubble on a floor, a stone on a plank, a cart on a bridge), and what hangs on it by links, into each piece's
// stressLoad. Sampled when a solve starts and kept, so the solve can continue across steps while the contacts jitter.
// Returns how much the loads changed (forces, and torques over each piece's size), relative to the structure's own
// weight; a piece whose load changed by more than 2% of its own weight is stamped.
static float lpSampleLoads( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	lpWorldTransform xf = lpPhys_GetTransform( w->phys, body->id );
	int n = body->pieces.count;
	lpArray_Reserve( w->scratchLoads, n );
	float weight = 0.0f;
	float g = body->gravityScale * lpLength( lpPhys_GetGravity( w->phys ) );
	for ( int i = 0; i < n; ++i )
	{
		lpPiece* p = w->pieces.data + body->pieces.data[i];
		w->scratchLoads.data[i] = p->stressLoad;
		p->stressLoad = lp_vec6Zero;
		weight += p->shape->volume * lpGetMaterial( p->material )->density * g;
	}

	if ( w->lastTimeStep > 0.0f )
	{
		// Summed per piece in a total order (piece, then what it touches), not the physics engine's report order
		const lpPhysContact* contacts;
		int count = lpPhys_GetBodyContacts( w->phys, body->id, &contacts );
		for ( int k = 0; k < count; ++k )
		{
			const lpPhysContact* c = contacts + k;
			lpPiece* piece = w->pieces.data + c->piece;
			if ( piece->anchored || c->impulse <= 0.0f )
			{
				continue; // an anchored piece's load goes to the ground
			}
			// The normal points from A to B: the impulse pushes B along it and A against it
			float sign = c->pieceIsA ? -1.0f : 1.0f;
			lpVec3 force = lpMulSV( sign * c->impulse / w->lastTimeStep, c->normal );
			lpVec3 local = lpInvRotateVector( xf.q, force );
			lpVec3 point = lpInvTransformWorldPoint( xf, c->point );
			piece->stressLoad.f = lpAdd( piece->stressLoad.f, local );
			piece->stressLoad.t = lpAdd( piece->stressLoad.t, lpCross( lpSub( point, piece->shape->centroid ), local ) );
		}
	}

	// Links pull on it too (a sign on a beam, a drawbridge on its ropes), with the force they held at the last step
	for ( int i = 0; i < n; ++i )
	{
		int pi = body->pieces.data[i];
		lpPiece* piece = w->pieces.data + pi;
		for ( int k = 0; k < piece->links.count && piece->anchored == false; ++k )
		{
			const lpLink* l = w->links.data + piece->links.data[k];
			int end = l->ends[1].piece == pi ? 1 : 0;
			float sign = end == 1 ? 1.0f : -1.0f; // the joint's force and torque are those on end B
			lpVec3 force = lpInvRotateVector( xf.q, lpMulSV( sign, l->force ) );
			lpVec3 torque = lpInvRotateVector( xf.q, lpMulSV( sign, l->torque ) );
			lpVec3 arm = lpSub( l->ends[end].frame.p, piece->shape->centroid );
			piece->stressLoad.f = lpAdd( piece->stressLoad.f, force );
			piece->stressLoad.t = lpAdd( piece->stressLoad.t, lpAdd( lpCross( arm, force ), torque ) );
		}
	}
	if ( w->vehicles.count > 0 )
	{
		lpAddWheelLoads( w, bodyIndex, xf ); // and wheels standing on it (a cart on a bridge)
	}

	float change = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		int pi = body->pieces.data[i];
		const lpPiece* p = w->pieces.data + pi;
		float size = lpCbrt( p->shape->volume );
		float moved = lpLength( lpSub( p->stressLoad.f, w->scratchLoads.data[i].f ) ) +
					  lpLength( lpSub( p->stressLoad.t, w->scratchLoads.data[i].t ) ) / size;
		change += moved;
		if ( moved > 0.02f * p->shape->volume * lpGetMaterial( p->material )->density * g )
		{
			lpTouchPiece( w, pi );
		}
	}
	return weight > 0.0f ? change / weight : 0.0f;
}

// Nodes and edges of a structure's system, counted the way lpStressBuild lays them out, so its budget is known before
// anything is built; and the groups and edges of its reduced system (fewer groups than nodes: parts of it move as
// rigid clusters)
static void lpStressCount( lpWorld* w, const lpBody* body, int* nodes, int* edges, int* groups, int* reducedEdges )
{
	*nodes = 0;
	*edges = 0;
	*groups = 0;
	*reducedEdges = 0;
	int clusters = 0;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int c = w->pieces.data[body->pieces.data[i]].cluster;
		clusters = c > clusters ? c : clusters;
	}
	lpArray_Reserve( w->scratchClusters, clusters + 1 );
	memset( w->scratchClusters.data, 0, sizeof( int ) * (size_t)( clusters + 1 ) );
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		const lpPiece* p = w->pieces.data + pi;
		if ( lpFixed( body, pi, p ) == false )
		{
			*nodes += 1;
			*groups += p->cluster == 0 || w->scratchClusters.data[p->cluster] == 0 ? 1 : 0;
			w->scratchClusters.data[p->cluster] = 1;
		}
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			const lpBond* bond = w->bonds.data + p->bonds.data[k];
			const lpPiece* other = w->pieces.data + bond->b;
			if ( bond->a == pi && ( lpFixed( body, pi, p ) == false || lpFixed( body, bond->b, other ) == false ) )
			{
				*edges += 1;
				*reducedEdges += p->cluster != 0 && p->cluster == other->cluster ? 0 : 1;
			}
		}
	}
}

// The reduced system of a structure moving partly as rigid clusters, and its right-hand side P^T (f - K xOld) from one
// pass over the fine edges: what the last solution leaves unbalanced under the new loads and bonds. The groups are a
// node of its own for each unclustered piece and one per cluster, in node order, its reference point at the
// cluster's centre of mass. Runs inside the structure's parallel job.
static void lpStressBuildReduced( lpWorld* w, lpStressJob* job )
{
	lpBody* body = w->bodies.data + job->body;
	lpStressReduced* red = body->reduced;
	lpStressSystem* s = job->system;
	lpPartition* part = &red->partition;
	int n = s->nodes.count;

	int clusters = 0;
	for ( int i = 0; i < n; ++i )
	{
		int c = w->pieces.data[s->nodes.data[i]].cluster;
		clusters = c > clusters ? c : clusters;
	}
	lpArray_Reserve( job->clusterGroup, clusters + 1 );
	for ( int c = 0; c <= clusters; ++c )
	{
		job->clusterGroup.data[c] = -1;
	}
	lpArray_Reserve( part->group, n );
	part->group.count = n;
	lpArray_Reserve( red->nodeRef, n );
	red->nodeRef.count = n;
	part->groupCount = 0;
	part->ref.count = 0;
	part->members.count = 0;
	job->groupMass.count = 0;
	for ( int i = 0; i < n; ++i )
	{
		const lpPiece* p = w->pieces.data + s->nodes.data[i];
		red->nodeRef.data[i] = p->shape->centroid;
		int g = p->cluster > 0 ? job->clusterGroup.data[p->cluster] : -1;
		if ( g < 0 )
		{
			g = part->groupCount++;
			lpArray_Push( part->ref, lpVec3_zero );
			lpArray_Push( part->members, 0 );
			lpArray_Push( job->groupMass, 0.0f );
			if ( p->cluster > 0 )
			{
				job->clusterGroup.data[p->cluster] = g;
			}
		}
		part->group.data[i] = g;
		float mass = p->shape->volume * lpGetMaterial( p->material )->density;
		part->members.data[g] += 1;
		job->groupMass.data[g] += mass;
		part->ref.data[g] = lpMulAdd( part->ref.data[g], mass, p->shape->centroid );
	}
	for ( int g = 0; g < part->groupCount; ++g )
	{
		if ( part->members.data[g] > 1 )
		{
			part->ref.data[g] = lpMulSV( 1.0f / job->groupMass.data[g], part->ref.data[g] );
		}
	}
	for ( int i = 0; i < n; ++i )
	{
		int g = part->group.data[i];
		if ( part->members.data[g] == 1 )
		{
			part->ref.data[g] = red->nodeRef.data[i]; // a node of its own keeps its reference exactly
		}
	}

	// What changed since the last judged solve: its residual under the new loads and bonds, less the residual that solve
	// was accepted with. Where no load or bond changed this is exactly zero, so the correction stays local.
	lpVec6* x = s->vectors.data;
	lpVec6* f = x + n;
	lpVec6* r = x + 2 * n;
	lpVec6* q = x + 5 * n;
	lpSystemApply( s, x, q );
	float inverse = 1.0f / s->forceScale;
	double load2 = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		const lpVec6* accepted = &w->pieces.data[s->nodes.data[i]].stressResidual;
		r[i].f = lpMulSub( lpSub( f[i].f, q[i].f ), inverse, accepted->f );
		r[i].t = lpMulSub( lpSub( f[i].t, q[i].t ), inverse, accepted->t );
		load2 += (double)lpDot( f[i].f, f[i].f ) + (double)lpDot( f[i].t, f[i].t );
	}

	lpStressSystem* rs = &red->system;
	lpSystemReduce( s, red->nodeRef.data, part, rs );
	int m = rs->nodes.count;
	lpVec6* y = rs->vectors.data;
	for ( int g = 0; g < m; ++g )
	{
		y[g] = lp_vec6Zero;
	}
	lpPartitionRestrict( part, red->nodeRef.data, r, n, y + m );
	rs->loadNorm2 = load2; // tolerances are relative to the whole load, not to the correction

	// Each group balances to the tolerance of its most loaded member: the sum of its members' scales would count the
	// forces inside it, which cancel, and let a big cluster sit out of balance by more than any one member carries
	lpArray_Reserve( rs->nodeScale, m );
	rs->nodeScale.count = m;
	lpArray_Reserve( rs->nodeArm, m );
	rs->nodeArm.count = m;
	for ( int g = 0; g < m; ++g )
	{
		rs->nodeScale.data[g] = 0.0f;
		rs->nodeArm.data[g] = 0.0f;
	}
	for ( int i = 0; i < n; ++i )
	{
		int g = part->group.data[i];
		float arm = lpDistance( red->nodeRef.data[i], part->ref.data[g] ) + s->nodeArm.data[i];
		rs->nodeScale.data[g] = lpMaxFloat( rs->nodeScale.data[g], s->nodeScale.data[i] );
		rs->nodeArm.data[g] = lpMaxFloat( rs->nodeArm.data[g], arm );
	}
	red->topology = body->topology;
	red->clusterStamp = body->clusterStamp;
	red->built = true;
}

// Compact system for one structure: non-anchored pieces become nodes, bonds become edges; loads, preconditioner and
// warm start. A solve continuing on the body's system from the last step only reloads the warm start. Runs inside a
// parallel job: it writes only the job, the body's system and its own pieces' solve slots.
static void lpStressBuild( lpWorld* w, lpStressJob* job )
{
	const lpBody* body = w->bodies.data + job->body;
	lpStressSystem* s = job->system;
	lpVec3 g = job->gravity;
	int n = job->nodeCount;

	if ( job->cached == false )
	{
		s->nodes.count = 0;
		float heaviest = 0.0f;
		int slot = 0;
		for ( int i = 0; i < body->pieces.count; ++i )
		{
			int pi = body->pieces.data[i];
			lpPiece* p = w->pieces.data + pi;
			p->solveSlot = -1;
			if ( lpFixed( body, pi, p ) == false )
			{
				p->solveSlot = slot++;
				lpArray_Push( s->nodes, pi );
				float weight = p->shape->volume * lpGetMaterial( p->material )->density;
				heaviest = weight > heaviest ? weight : heaviest;
			}
		}
		LP_ASSERT( slot == n );
		float scale = heaviest * lpLength( g );
		s->forceScale = scale > 0.0f ? scale : 1.0f;

		s->edges.count = 0;
		for ( int i = 0; i < body->pieces.count; ++i )
		{
			int pi = body->pieces.data[i];
			const lpPiece* p = w->pieces.data + pi;
			for ( int k = 0; k < p->bonds.count; ++k )
			{
				int bi = p->bonds.data[k];
				const lpBond* bond = w->bonds.data + bi;
				if ( bond->a != pi )
				{
					continue; // each bond once, from its lower piece
				}
				const lpPiece* pa = p;
				const lpPiece* pb = w->pieces.data + bond->b;
				if ( pa->solveSlot < 0 && pb->solveSlot < 0 )
				{
					continue; // between two anchored pieces: carries nothing we solve for
				}
				lpStressEdge e;
				e.a = pa->solveSlot;
				e.b = pb->solveSlot;
				e.bond = bi;
				e.ra = lpSub( bond->centroid, pa->shape->centroid );
				e.rb = lpSub( bond->centroid, pb->shape->centroid );
				e.n = bond->normal;
				lpContactBasis( e.n, &e.t1, &e.t2 );
				float len = lpMaxFloat( lpDistance( pa->shape->centroid, pb->shape->centroid ), 0.05f );
				float area = bond->area;
				e.kn = area / len;
				e.ks = 0.4f * e.kn;
				e.kb1 = area * bond->h2 * bond->h2 / ( 3.0f * len );
				e.kb2 = area * bond->h1 * bond->h1 / ( 3.0f * len );
				e.kt = 0.4f * ( e.kb1 + e.kb2 );
				lpArray_Push( s->edges, e );
			}
		}
		LP_ASSERT( s->edges.count == job->edgeCount );
		lpSystemResize( s );
		if ( job->clustered == false )
		{
			lpSystemFactor( s ); // a correction is solved on the reduced system's blocks
		}
		lpSystemIncidence( s );
		s->topology = body->topology;
		s->built = true;
	}

	// Vectors: x (solution) and f (load); r, z, p and q are the solver's. A correction keeps its x, the solution it
	// corrects, until it is judged.
	float scale = s->forceScale;
	lpVec6* x = s->vectors.data;
	lpVec6* f = x + n;
	if ( job->clustered == false || job->cached == false )
	{
		for ( int i = 0; i < n; ++i )
		{
			const lpPiece* p = w->pieces.data + s->nodes.data[i];
			x[i].f = lpMulSV( 1.0f / scale, p->stressX.f );
			x[i].t = lpMulSV( 1.0f / scale, p->stressX.t );
		}
	}
	if ( job->cached )
	{
		return; // loads, r and p are in the system (or the reduced one) as the last step left them
	}
	for ( int i = 0; i < n; ++i )
	{
		const lpPiece* p = w->pieces.data + s->nodes.data[i];
		float mass = p->shape->volume * lpGetMaterial( p->material )->density;
		f[i].f = lpMulSV( mass / scale, g );
		f[i].t = lpVec3_zero;
		if ( body->solveStress )
		{
			// Less what it takes to accelerate it with its body (inertia relief)
			lpVec3 r = lpSub( p->shape->centroid, body->reliefCenter );
			lpVec3 w0 = body->reliefOmega;
			lpVec3 accel = lpAdd( lpAdd( body->reliefAccel, lpCross( body->reliefAlpha, r ) ), lpCross( w0, lpCross( w0, r ) ) );
			float own = mass * lpCbrt( p->shape->volume ) * lpCbrt( p->shape->volume ) / 6.0f;
			f[i].f = lpMulSub( f[i].f, mass / scale, accel );
			f[i].t = lpMulSV( -own / scale, body->reliefAlpha );
		}
	}
	for ( int i = 0; i < n; ++i )
	{
		const lpPiece* p = w->pieces.data + s->nodes.data[i];
		f[i].f = lpMulAdd( f[i].f, 1.0f / scale, p->stressLoad.f );
		f[i].t = lpMulAdd( f[i].t, 1.0f / scale, p->stressLoad.t );
	}

	// For the per-node equilibrium test: each node's size, and the forces through it at the warm start (a crumb must
	// balance to a fraction of a crumb, a stone carrying the wall above it to a fraction of that wall)
	lpArray_Reserve( s->nodeArm, n );
	s->nodeArm.count = n;
	for ( int i = 0; i < n; ++i )
	{
		s->nodeArm.data[i] = lpCbrt( w->pieces.data[s->nodes.data[i]].shape->volume );
	}
	lpSystemNodeScales( s, 1e-3f );
	if ( job->clustered )
	{
		lpStressBuildReduced( w, job );
		return;
	}

	// A solve in progress continues from where the last step left it
	if ( job->continuing )
	{
		lpVec6* r = x + 2 * n;
		lpVec6* p = x + 4 * n;
		for ( int i = 0; i < n; ++i )
		{
			const lpPiece* piece = w->pieces.data + s->nodes.data[i];
			r[i] = piece->stressR;
			p[i] = piece->stressP;
		}
	}
}

// Limits of a bond: its joint's, or the weaker material's for a solid joint
static void lpBondLimits( const lpWorld* w, const lpBond* bond, float* tension, float* compression, float* shear, float* mu )
{
	if ( bond->joint != lp_jointSolid )
	{
		const lpJointDef* j = lpGetJoint( bond->joint );
		*tension = j->tensileStrength;
		*compression = j->compressiveStrength;
		*shear = j->shearStrength;
		*mu = j->friction;
		return;
	}
	const lpMaterialDef* ma = lpGetMaterial( w->pieces.data[bond->a].material );
	const lpMaterialDef* mb = lpGetMaterial( w->pieces.data[bond->b].material );
	*tension = lpMinFloat( ma->tensileStrength, mb->tensileStrength );
	*compression = lpMinFloat( ma->compressiveStrength, mb->compressiveStrength );
	*shear = lpMinFloat( ma->shearStrength, mb->shearStrength );
	*mu = lpGetJoint( lp_jointSolid )->friction;
}

static int lpCompareOverload( const void* a, const void* b )
{
	const lpOverload* x = a;
	const lpOverload* y = b;
	if ( x->rho != y->rho )
	{
		return x->rho > y->rho ? -1 : 1;
	}
	return ( x->bond > y->bond ) - ( x->bond < y->bond );
}

// Creaking you can see: dust trickles from a joint under strain, and puffs when one lets go. Cosmetic only: the
// randomness is hashed from the tick and the bond, never taken from simulation state.
static void lpStressDust( lpWorld* w, lpWorldTransform xf, const lpBond* bond, int bondIndex, int motes )
{
	uint8_t material = w->pieces.data[bond->a].material;
	uint64_t h = lpMix64( ( w->tick << 24 ) ^ (uint64_t)bondIndex );
	for ( int k = 0; k < motes; ++k )
	{
		h = lpMix64( h + (uint64_t)k );
		float rx = (float)( h & 0xFFFF ) / 65535.0f - 0.5f;
		float rz = (float)( ( h >> 16 ) & 0xFFFF ) / 65535.0f - 0.5f;
		float rs = (float)( ( h >> 32 ) & 0xFFFF ) / 65535.0f;
		lpVec3 v = { 0.6f * rx, -0.3f - 0.4f * rs, 0.6f * rz };
		lpEmitParticle( w, xf, bond->centroid, v, 0.02f + 0.02f * rs, material );
	}
}

// A joint over its limit strains a little more every check and creaks; at strain 1 it is queued to break
static void lpApplyStrain( lpWorld* w, lpWorldTransform xf, int bondIndex, float rho, int* strained )
{
	lpBond* bond = w->bonds.data + bondIndex;
	bond->rho = rho;
	if ( rho >= w->def.stressGlue )
	{
		// Loaded near its limit: both its pieces are resolved finely from now on
		lpPiece* a = w->pieces.data + bond->a;
		lpPiece* b = w->pieces.data + bond->b;
		if ( a->cluster != 0 || b->cluster != 0 )
		{
			a->cluster = 0;
			b->cluster = 0;
			w->bodies.data[a->body].clusterStamp += 1;
		}
	}
	if ( rho <= 1.0f )
	{
		return;
	}
	*strained += 1;
	if ( ( w->tick + (uint64_t)bondIndex ) % 4 == 0 )
	{
		lpStressDust( w, xf, bond, bondIndex, 1 );
	}
	bond->strain += ( rho - 1.0f ) * w->def.strainRate;
	if ( bond->strain >= 1.0f )
	{
		lpOverload o = { rho, bondIndex };
		lpArray_Push( w->scratchOverloads, o );
	}
}

// The queued joints break, worst first: everything at twice its limit, otherwise a few per check, so failure cascades
static int lpBreakOverloads( lpWorld* w, lpWorldTransform xf )
{
	int count = w->scratchOverloads.count;
	if ( count > 1 )
	{
		qsort( w->scratchOverloads.data, (size_t)count, sizeof( lpOverload ), lpCompareOverload );
	}
	int broken = 0;
	for ( int i = 0; i < count; ++i )
	{
		lpOverload o = w->scratchOverloads.data[i];
		if ( o.rho < 2.0f && broken >= w->def.maxStressBreaks )
		{
			break; // the rest wait for the re-solve
		}
		lpStressDust( w, xf, w->bonds.data + o.bond, o.bond, 4 );
		lpBreakBond( w, o.bond );
		broken += 1;
	}
	w->scratchOverloads.count = 0;
	return broken;
}

// A joint's utilization (1 = at its limit) under a force and moment (on piece b, body frame), from its patch, its
// limits and its health; t1, t2 are lpContactBasis( bond->normal )
static float lpBondUtilization( const lpWorld* w, const lpBond* bond, lpVec3 t1, lpVec3 t2, lpVec3 force, lpVec3 moment )
{
	lpVec3 n = bond->normal;
	float area = bond->area;
	float axialForce = lpDot( force, n ); // tension positive
	float compression = lpMaxFloat( -axialForce, 0.0f );
	float m1 = lpAbsFloat( lpDot( moment, t1 ) ); // bending about t1: the fibres at +-h2 carry it
	float m2 = lpAbsFloat( lpDot( moment, t2 ) );
	lpVec3 shearForce = lpMulSub( force, axialForce, n );
	float hMax = lpMaxFloat( bond->h1, bond->h2 );
	float shear = lpLength( shearForce ) / area +
				  3.0f * lpAbsFloat( lpDot( moment, n ) ) * hMax / ( area * ( bond->h1 * bond->h1 + bond->h2 * bond->h2 ) );

	float tensionLimit, compressionLimit, shearLimit, mu;
	lpBondLimits( w, bond, &tensionLimit, &compressionLimit, &shearLimit, &mu );
	float str = w->def.stressScale * lpMaxFloat( bond->health, 0.0f ) / bond->strength;
	float tensionCap = lpMaxFloat( str * tensionLimit, 1e4f );

	// Pulling apart and bending: the section's elastic tension capacity, plus rocking. A compressed joint holds a moment
	// until its resultant reaches the edge of the patch, so masonry tips over an edge instead of cracking as soon as the
	// load leaves the middle third.
	float cap1 = tensionCap * area * bond->h2 / 3.0f + compression * bond->h2;
	float cap2 = tensionCap * area * bond->h1 / 3.0f + compression * bond->h1;
	float rho = lpMaxFloat( axialForce, 0.0f ) / ( tensionCap * area ) + m1 / cap1 + m2 / cap2;

	// Crushing at the compressed edge, and Coulomb shear
	float edge = compression / area + 3.0f * m1 / ( area * bond->h2 ) + 3.0f * m2 / ( area * bond->h1 );
	rho = lpMaxFloat( rho, edge / lpMaxFloat( str * compressionLimit, 1e4f ) );
	return lpMaxFloat( rho, shear / ( str * ( shearLimit + mu * compression / area ) + 1e3f ) );
}

// Phase 2, converged: every bond's utilization from the solution (into the system's rho) and the force and moment it
// carries (onto the bond), and the peak
static void lpStressUtilizations( lpWorld* w, lpStressJob* job, const lpVec6* x )
{
	lpStressSystem* s = job->system;
	lpArray_Reserve( s->rho, s->edges.count );
	s->rho.count = s->edges.count;
	float peak = 0.0f;
	for ( int k = 0; k < s->edges.count; ++k )
	{
		const lpStressEdge* e = s->edges.data + k;
		lpBond* bond = w->bonds.data + e->bond;
		lpVec3 force, moment;
		lpEdgeForce( e, x, &force, &moment );
		bond->force = lpMulSV( s->forceScale, force );
		bond->moment = lpMulSV( s->forceScale, moment );
		float rho = lpBondUtilization( w, bond, e->t1, e->t2, bond->force, bond->moment );
		s->rho.data[k] = rho;
		peak = rho > peak ? rho : peak;
	}
	job->peak = peak;
}

static int lpStrainSlender( lpWorld* w, lpWorldTransform xf, int pi, int* strained, int* slender );

// A converged structure that has not changed since, but creaks (joints or slender pieces over their limit, none broken
// yet) or had joints weakened by a blast: it is judged again from its last solve, with no solve. Creaking only adds
// strain from the stored utilizations; after a blast (recompute) the joints' are computed again from the stored forces
// and their new health.
static int lpStressRejudge( lpWorld* w, int bodyIndex, bool recompute, int* strained, int* snapped )
{
	lpBody* body = w->bodies.data + bodyIndex;
	lpWorldTransform xf = lpPhys_GetTransform( w->phys, body->id );
	w->scratchOverloads.count = 0;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		const lpPiece* p = w->pieces.data + pi;
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			int bi = p->bonds.data[k];
			const lpBond* bond = w->bonds.data + bi;
			if ( bond->a != pi )
			{
				continue;
			}
			float rho = bond->rho;
			if ( recompute && ( lpFixed( body, pi, p ) == false || lpFixed( body, bond->b, w->pieces.data + bond->b ) == false ) )
			{
				lpVec3 t1, t2;
				lpContactBasis( bond->normal, &t1, &t2 );
				rho = lpBondUtilization( w, bond, t1, t2, bond->force, bond->moment );
			}
			if ( rho > 1.0f || rho != bond->rho )
			{
				lpApplyStrain( w, xf, bi, rho, strained );
			}
		}
	}
	int broken = lpBreakOverloads( w, xf );
	for ( int i = 0; i < body->pieces.count && broken == 0; ++i )
	{
		int slender = 0;
		*snapped += lpStrainSlender( w, xf, body->pieces.data[i], strained, &slender );
	}
	return broken;
}

// A long piece (beam, plank, column, lintel) of a breakable material: rigid in the solve, so its own check bends it from
// its bonds' forces. Its extents along its axis (from its centroid) and across it, if it is.
static bool lpSlenderExtents( const lpWorld* w, const lpPiece* p, float* lo, float* hi, float* w1, float* w2 )
{
	const lpMaterialDef* m = lpGetMaterial( p->material );
	if ( m->breakable == false || m->pattern == lp_breakRadial )
	{
		return false;
	}
	lpVec3 a = p->axis, t1, t2;
	lpContactBasis( a, &t1, &t2 );
	lpVec3 c = p->shape->centroid;
	*lo = FLT_MAX;
	*hi = -FLT_MAX;
	*w1 = 0.0f;
	*w2 = 0.0f;
	for ( int k = 0; k < p->shape->vertexCount; ++k )
	{
		lpVec3 d = lpSub( p->shape->vertices[k], c );
		float along = lpDot( d, a );
		*lo = along < *lo ? along : *lo;
		*hi = along > *hi ? along : *hi;
		*w1 = lpMaxFloat( *w1, lpAbsFloat( lpDot( d, t1 ) ) );
		*w2 = lpMaxFloat( *w2, lpAbsFloat( lpDot( d, t2 ) ) );
	}
	float length = *hi - *lo;
	float fragment = m->fragmentSize * w->def.fragmentScale;
	return length >= 2.5f * 2.0f * lpMaxFloat( *w1, *w2 ) && length >= 4.0f * fragment;
}

// Long pieces (beams, planks, columns, lintels) are rigid nodes, so the solve cannot bend them. Phase 2, converged: from
// the solved bond forces and the piece's own weight, find the bending moment along its axis at a few cuts between its
// supports, and keep the worst (into the job). Phase 3 (lpStressSnap) strains the overloaded ones.
static void lpStressSlender( lpWorld* w, lpStressJob* job, const lpVec6* x )
{
	const lpStressSystem* s = job->system;
	lpVec3 g = job->gravity;
	float forceScale = s->forceScale;
	for ( int i = 0; i < s->nodes.count; ++i )
	{
		int pi = s->nodes.data[i];
		const lpPiece* p = w->pieces.data + pi;
		const lpMaterialDef* m = lpGetMaterial( p->material );
		float lo, hi, w1, w2;
		if ( p->depth >= w->def.maxDepth || p->bonds.count < 2 || lpSlenderExtents( w, p, &lo, &hi, &w1, &w2 ) == false )
		{
			continue; // not slender: the joints decide
		}
		lpVec3 a = p->axis, t1, t2;
		lpContactBasis( a, &t1, &t2 );
		lpVec3 c = p->shape->centroid;
		float length = hi - lo;

		// Loads on the piece: each bond's force at its contact, and the moment it carries
		float bondLo = FLT_MAX, bondHi = -FLT_MAX;
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			float along = lpDot( lpSub( w->bonds.data[p->bonds.data[k]].centroid, c ), a );
			bondLo = along < bondLo ? along : bondLo;
			bondHi = along > bondHi ? along : bondHi;
		}
		if ( bondHi - bondLo < 0.25f * length )
		{
			continue; // held at one place only: a cantilever off one joint, which the joint check covers
		}

		float area = p->shape->volume / length;
		float limit = w->def.stressScale * m->tensileStrength;
		float mass = p->shape->volume * m->density;
		float worst = 0.0f, worstAt = 0.0f;
		const int* incident = s->incident.data;
		int first = s->incidentStart.data[i], last = s->incidentStart.data[i + 1];
		for ( int sample = 1; sample <= 9; ++sample )
		{
			float cut = bondLo + ( bondHi - bondLo ) * (float)sample / 10.0f;
			lpVec3 q = lpMulAdd( c, cut, a );
			lpVec3 forceSum = lpVec3_zero, momentSum = lpVec3_zero;
			for ( int j = first; j < last; ++j )
			{
				const lpStressEdge* e = s->edges.data + incident[j];
				const lpBond* bond = w->bonds.data + e->bond;
				if ( lpDot( lpSub( bond->centroid, c ), a ) <= cut )
				{
					continue;
				}
				lpVec3 f, mo;
				lpEdgeForce( e, x, &f, &mo );
				float sign = e->a == i ? forceScale : -forceScale; // what the bond does to this piece
				f = lpMulSV( sign, f );
				mo = lpMulSV( sign, mo );
				forceSum = lpAdd( forceSum, f );
				momentSum = lpAdd( momentSum, lpAdd( lpCross( lpSub( bond->centroid, q ), f ), mo ) );
			}
			// The piece's own weight beyond the cut, spread evenly along its length
			float share = ( hi - cut ) / length;
			lpVec3 weight = lpMulSV( share * mass, g );
			lpVec3 at = lpMulAdd( c, 0.5f * ( hi + cut ), a );
			forceSum = lpAdd( forceSum, weight );
			momentSum = lpAdd( momentSum, lpCross( lpSub( at, q ), weight ) );

			float sigma = lpAbsFloat( lpDot( forceSum, a ) ) / area + 3.0f * lpAbsFloat( lpDot( momentSum, t1 ) ) / ( area * w2 ) +
						  3.0f * lpAbsFloat( lpDot( momentSum, t2 ) ) / ( area * w1 );
			float rho = sigma / limit;
			if ( rho > worst )
			{
				worst = rho;
				worstAt = cut;
			}
		}
		lpSlenderCut slenderCut = { pi, length, worst, worstAt, 2.0f * lpMaxFloat( w1, w2 ) };
		lpArray_Push( job->slender, slenderCut );
	}
}

// An overloaded slender piece strains a little more every check (from its worst section at the last judged solve),
// and at 1 a small synthetic impact there snaps it through the normal fracture pipeline next step. Returns 1 if it was
// queued to break.
static int lpStrainSlender( lpWorld* w, lpWorldTransform xf, int pi, int* strained, int* slender )
{
	lpPiece* p = w->pieces.data + pi;
	if ( p->slenderRho <= 1.0f )
	{
		return 0;
	}
	*strained += 1;
	*slender += 1;
	p->strain += ( p->slenderRho - 1.0f ) * w->def.strainRate;
	if ( p->strain < 1.0f )
	{
		return 0;
	}

	// Snap it where it is weakest: a blow sized to the section, through the normal fracture pipeline
	const lpMaterialDef* m = lpGetMaterial( p->material );
	p->strain = 0.0f;
	lpImpactDef impact = { 0 };
	impact.point = lpTransformWorldPoint( xf, lpMulAdd( p->shape->centroid, p->slenderAt, p->axis ) );
	impact.direction = lpRotateVector( xf.q, p->axis );
	impact.radius = 1.5f * p->slenderDepth;
	impact.energy = 4.0f * lpMaxFloat( m->bondStrength, m->fractureEnergy ) * LP_PI * impact.radius * impact.radius;
	w->impactSerial += 1;
	lpDeferredJob snap = { pi, p->generation, w->impactSerial, impact, true };
	lpArray_Push( w->deferred, snap );
	lpStressDust( w, xf, w->bonds.data + p->bonds.data[0], p->bonds.data[0], 6 );
	return 1;
}

// Phase 3, a judged solve: the slender pieces' worst sections are kept on the pieces (so creaking can strain them
// without a solve), and once the joints hold, the overloaded ones strain. Returns the number of pieces queued to break.
static int lpStressSnap( lpWorld* w, const lpStressJob* job, bool jointsHold, int* strained, int* slender )
{
	const lpStressSystem* s = job->system;
	for ( int i = 0; i < s->nodes.count; ++i )
	{
		w->pieces.data[s->nodes.data[i]].slenderRho = 0.0f;
	}
	int queued = 0;
	for ( int k = 0; k < job->slender.count; ++k )
	{
		const lpSlenderCut* cut = job->slender.data + k;
		lpPiece* p = w->pieces.data + cut->piece;
		p->slenderRho = cut->worst;
		p->slenderAt = cut->worstAt;
		p->slenderDepth = cut->depth;
		if ( jointsHold == false )
		{
			continue;
		}
		if ( w->def.debugLog )
		{
			printf( "[lpf]   slender piece %d: length %.2f, worst utilization %.2f at %.2f, strain %.2f\n", cut->piece,
					(double)cut->length, (double)cut->worst, (double)cut->worstAt, (double)p->strain );
		}
		queued += lpStrainSlender( w, job->xf, cut->piece, strained, slender );
	}
	return queued;
}

// The residual meter (phase 2, a converged correction). Each cluster member's residual changes by the load the
// correction brings it through its bonds, K (x - xOld): nodes of their own are solved for theirs, and over a cluster it
// balances out, but a rigid cluster cannot share it among its members the way their bonds would. The joints inside a
// cluster were all under stressGlue at its exact solve; if a member's load changed enough to carry one of its joints
// past LP_METER_DISSOLVE (its utilization grown with the change, relative to the forces through the member), the
// cluster dissolves and the solve runs again with its pieces resolved finely. Checked against an exact solve (lpStressOracle),
// the rigid clusters' error lies near the change, where the seeds keep pieces fine, and is small elsewhere. (The
// residual the last solution was accepted with is not the clusters' doing.) Returns the number of clusters dissolved.
static int lpStressMeter( lpWorld* w, lpStressJob* job, const lpVec6* x )
{
	lpBody* body = w->bodies.data + job->body;
	const lpStressReduced* red = body->reduced;
	const lpPartition* part = &red->partition;
	lpStressSystem* s = job->system;
	int n = s->nodes.count;
	const lpVec6* xOld = s->vectors.data;
	lpVec6* change = s->vectors.data + 4 * n; // p and q: free in the delta form once solved
	lpVec6* kx = s->vectors.data + 5 * n;
	for ( int i = 0; i < n; ++i )
	{
		change[i].f = lpSub( x[i].f, xOld[i].f );
		change[i].t = lpSub( x[i].t, xOld[i].t );
	}
	lpSystemApply( s, change, kx );

	// Per cluster: the joints' growth (worst member), the largest load change, and its most loaded member's throughput
	lpArray_Reserve( job->groupMass, 3 * part->groupCount );
	job->groupMass.count = 3 * part->groupCount;
	float* worst = job->groupMass.data;
	float* leak = worst + part->groupCount;
	float* carried = leak + part->groupCount;
	for ( int g = 0; g < part->groupCount; ++g )
	{
		worst[g] = 0.0f;
		leak[g] = 0.0f;
		carried[g] = 0.0f;
	}
	for ( int i = 0; i < n; ++i )
	{
		int g = part->group.data[i];
		if ( part->members.data[g] < 2 )
		{
			continue;
		}
		const lpPiece* p = w->pieces.data + s->nodes.data[i];
		float rho = 0.0f;
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			rho = lpMaxFloat( rho, w->bonds.data[p->bonds.data[k]].rho );
		}
		float moved = lpLength( kx[i].f ) + lpLength( kx[i].t ) / s->nodeArm.data[i];
		worst[g] = lpMaxFloat( worst[g], rho * ( 1.0f + moved / s->nodeScale.data[i] ) / LP_METER_DISSOLVE );
		leak[g] = lpMaxFloat( leak[g], moved );
		carried[g] = lpMaxFloat( carried[g], s->nodeScale.data[i] );
	}
	// Past 1: a joint could be carried past LP_METER_DISSOLVE, or the cluster's load changed by more than LP_METER_CHANGE
	// of what its most loaded member carries: it is carrying a redistribution stiffly, biasing the fine joints around it
	for ( int g = 0; g < part->groupCount; ++g )
	{
		worst[g] = carried[g] > 0.0f ? lpMaxFloat( worst[g], leak[g] / carried[g] / LP_METER_CHANGE ) : worst[g];
	}
	int dissolved = 0, clusters = 0;
	for ( int g = 0; g < part->groupCount; ++g )
	{
		clusters += part->members.data[g] > 1 ? 1 : 0;
		dissolved += worst[g] > 1.0f ? 1 : 0;
		job->meterWorst = lpMaxFloat( job->meterWorst, worst[g] );
	}
	if ( 2 * dissolved > clusters || ( dissolved > 0 && body->meterRounds + 1 >= LP_METER_ROUNDS ) )
	{
		// Most of it is carrying the change (it is not local), or this was the last round: the clusters all go, and it is
		// solved exactly
		for ( int g = 0; g < part->groupCount; ++g )
		{
			worst[g] = part->members.data[g] > 1 ? 2.0f : worst[g];
		}
		dissolved = clusters;
	}
	if ( dissolved > 0 )
	{
		for ( int i = 0; i < n; ++i )
		{
			if ( worst[part->group.data[i]] > 1.0f )
			{
				w->pieces.data[s->nodes.data[i]].cluster = 0;
			}
		}
		body->clusterStamp += 1;
	}
	return dissolved;
}

// Tests (lpWorld.stressOracle): the same change solved exactly on the fine system, from the same last solution and with
// the same accepted residual, and the worst difference in any joint's utilization from the reduced solve's x
static void lpStressOracle( lpWorld* w, lpStressJob* job, const lpVec6* x )
{
	const lpStressSystem* s = job->system;
	int n = s->nodes.count;
	lpStressSystem exact = { 0 };
	for ( int i = 0; i < n; ++i )
	{
		lpArray_Push( exact.nodes, s->nodes.data[i] );
	}
	for ( int k = 0; k < s->edges.count; ++k )
	{
		lpArray_Push( exact.edges, s->edges.data[k] );
	}
	exact.forceScale = s->forceScale;
	lpSystemResize( &exact );
	lpSystemFactor( &exact );
	float inverse = 1.0f / s->forceScale;
	for ( int i = 0; i < n; ++i )
	{
		const lpVec6* accepted = &w->pieces.data[s->nodes.data[i]].stressResidual;
		exact.vectors.data[i] = s->vectors.data[i];
		exact.vectors.data[n + i].f = lpMulSub( s->vectors.data[n + i].f, inverse, accepted->f );
		exact.vectors.data[n + i].t = lpMulSub( s->vectors.data[n + i].t, inverse, accepted->t );
	}
	lpSolveState state = { 0 };
	lpSystemSolve( &exact, 20000, 1e-6, 0.001f, false, &state );
	// Where a decision is made: the error on joints both solves read within twice their limit (past that they break
	// at once either way; a dry joint in the slightest tension reads in the tens), and the joints one would strain and
	// the other not
	float worst = 0.0f, worstExact = 0.0f, worstReduced = 0.0f;
	int worstEdge = -1, flips = 0;
	for ( int k = 0; k < s->edges.count; ++k )
	{
		const lpStressEdge* e = s->edges.data + k;
		const lpBond* bond = w->bonds.data + e->bond;
		lpVec3 f1, m1, f2, m2;
		lpEdgeForce( e, exact.vectors.data, &f1, &m1 );
		lpEdgeForce( e, x, &f2, &m2 );
		float rho1 = lpBondUtilization( w, bond, e->t1, e->t2, lpMulSV( s->forceScale, f1 ), lpMulSV( s->forceScale, m1 ) );
		float rho2 = lpBondUtilization( w, bond, e->t1, e->t2, lpMulSV( s->forceScale, f2 ), lpMulSV( s->forceScale, m2 ) );
		float off = lpAbsFloat( rho1 - rho2 );
		flips += ( rho1 > 1.0f ) != ( rho2 > 1.0f ) ? 1 : 0;
		if ( lpMaxFloat( rho1, rho2 ) <= 2.0f && off > worst )
		{
			worst = off;
			worstExact = rho1;
			worstReduced = rho2;
			worstEdge = k;
		}
	}
	job->oracleWorst = worst;
	job->oracleFlips = flips;
	if ( w->def.debugLog && worstEdge >= 0 )
	{
		const lpStressEdge* e = s->edges.data + worstEdge;
		const lpBond* bond = w->bonds.data + e->bond;
		printf( "[lpf]   oracle: %d iterations; worst joint %d at %.1f %.1f %.1f: exact %.3f, reduced %.3f (clusters %d and %d)\n",
				state.iterations, e->bond, (double)bond->centroid.x, (double)bond->centroid.y, (double)bond->centroid.z,
				(double)worstExact, (double)worstReduced, w->pieces.data[bond->a].cluster, w->pieces.data[bond->b].cluster );
	}
	lpSystemFree( &exact );
}

// Phase 2, one structure: build, solve, and keep the solution (in newtons of load) and the solve's state on the
// pieces; converged, also the utilizations and the slender pieces' worst sections. A correction solved on the
// reduced system moves each member with its group, on top of the solution it corrects.
static void lpRunStressJob( int index, void* context )
{
	lpWorld* w = context;
	lpStressJob* job = w->stressJobs + index;
	lpStressBuild( w, job );
	lpStressSystem* s = job->system;
	lpStressReduced* red = job->clustered ? w->bodies.data[job->body].reduced : NULL;
	lpSystemSolve( red != NULL ? &red->system : s, job->budget, job->tolerance, job->nodeTolerance, job->continuing, &job->solve );
	int n = s->nodes.count;
	lpVec6* x = s->vectors.data;
	if ( red != NULL )
	{
		lpVec6* moved = x + 3 * n; // z: free in the delta form
		memcpy( moved, x, sizeof( lpVec6 ) * (size_t)n );
		lpPartitionProlong( &red->partition, red->nodeRef.data, red->system.vectors.data, n, moved );
		x = moved;
	}
	// Kept on the pieces: a solution continues from here after a restart. A correction is kept only once it has
	// converged: a partial one moves clusters rigidly out of balance, and a restart would chase that everywhere.
	const lpVec6* r = s->vectors.data + 2 * n;
	const lpVec6* p = s->vectors.data + 4 * n;
	for ( int i = 0; i < n && ( red == NULL || job->solve.converged ); ++i )
	{
		lpPiece* piece = w->pieces.data + s->nodes.data[i];
		piece->stressX.f = lpMulSV( s->forceScale, x[i].f );
		piece->stressX.t = lpMulSV( s->forceScale, x[i].t );
		if ( red == NULL )
		{
			piece->stressR = r[i];
			piece->stressP = p[i];
		}
	}
	job->peak = 0.0f;
	job->slender.count = 0;
	job->dissolved = 0;
	job->oracleWorst = -1.0f;
	job->meterWorst = 0.0f;
	if ( red != NULL && job->solve.converged && w->stressOracle )
	{
		lpStressOracle( w, job, x );
	}
	if ( red != NULL && job->solve.converged )
	{
		job->dissolved = lpStressMeter( w, job, x );
		job->solve.converged = job->dissolved == 0; // otherwise solved again, from here, with them resolved finely
		job->oracleWorst = job->dissolved == 0 ? job->oracleWorst : -1.0f; // only what is judged counts
	}
	if ( job->solve.converged )
	{
		lpStressUtilizations( w, job, x );
		lpStressSlender( w, job, x );

		// The residual it is accepted with, for the corrections after it
		lpVec6* kx = s->vectors.data + 5 * n; // q: free once solved
		const lpVec6* f = s->vectors.data + n;
		lpSystemApply( s, x, kx );
		for ( int i = 0; i < n; ++i )
		{
			lpVec6* residual = &w->pieces.data[s->nodes.data[i]].stressResidual;
			residual->f = lpMulSV( s->forceScale, lpSub( f[i].f, kx[i].f ) );
			residual->t = lpMulSV( s->forceScale, lpSub( f[i].t, kx[i].t ) );
		}
	}
}

static lpStressJob* lpAddStressJob( lpWorld* w )
{
	if ( w->stressJobCount == w->stressJobCapacity )
	{
		int capacity = w->stressJobCapacity < 8 ? 8 : 2 * w->stressJobCapacity;
		w->stressJobs = lpRealloc( w->stressJobs, sizeof( lpStressJob ) * (size_t)capacity );
		memset( w->stressJobs + w->stressJobCapacity, 0, sizeof( lpStressJob ) * (size_t)( capacity - w->stressJobCapacity ) );
		w->stressJobCapacity = capacity;
	}
	return w->stressJobs + w->stressJobCount++;
}

static int lpFindSet( lpClusterSet* sets, int i )
{
	while ( sets[i].parent != i )
	{
		sets[i].parent = sets[sets[i].parent].parent;
		i = sets[i].parent;
	}
	return i;
}

// Rigid clusters from an exact solve of a big structure (phase 3). Its nodes are grouped along their bonds, in edge
// order, except those a correction must resolve finely: slender pieces and both pieces of every joint loaded past
// stressGlue (anchored pieces are not nodes). A cluster holds at most LP_CLUSTER_MEMBERS pieces within
// LP_CLUSTER_RADIUS mean piece sizes of the middle of its bounds.
static void lpFormClusters( lpWorld* w, const lpStressJob* job )
{
	lpBody* body = w->bodies.data + job->body;
	const lpStressSystem* s = job->system;
	int n = s->nodes.count;
	lpArray_Reserve( w->scratchSets, n );
	lpClusterSet* sets = w->scratchSets.data;
	float meanSize = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		lpPiece* p = w->pieces.data + s->nodes.data[i];
		sets[i] = (lpClusterSet){ i, 1, p->shape->bounds, p->slenderRho < w->def.stressGlue, 0 };
		meanSize += lpCbrt( p->shape->volume );
		p->cluster = 0;
	}
	float radius = LP_CLUSTER_RADIUS * meanSize / (float)n;
	for ( int k = 0; k < s->edges.count; ++k )
	{
		const lpStressEdge* e = s->edges.data + k;
		if ( s->rho.data[k] >= w->def.stressGlue )
		{
			if ( e->a >= 0 )
			{
				sets[e->a].eligible = false;
			}
			if ( e->b >= 0 )
			{
				sets[e->b].eligible = false;
			}
		}
	}
	for ( int k = 0; k < s->edges.count; ++k )
	{
		const lpStressEdge* e = s->edges.data + k;
		if ( e->a < 0 || e->b < 0 || sets[e->a].eligible == false || sets[e->b].eligible == false ||
			 w->bonds.data[e->bond].alive == false )
		{
			continue;
		}
		int ra = lpFindSet( sets, e->a );
		int rb = lpFindSet( sets, e->b );
		if ( ra == rb || sets[ra].size + sets[rb].size > LP_CLUSTER_MEMBERS )
		{
			continue;
		}
		lpAABB box = { lpMin( sets[ra].box.lowerBound, sets[rb].box.lowerBound ), lpMax( sets[ra].box.upperBound, sets[rb].box.upperBound ) };
		if ( lpLength( lpAABB_Extents( box ) ) > radius )
		{
			continue;
		}
		int root = ra < rb ? ra : rb;
		int child = ra < rb ? rb : ra;
		sets[child].parent = root;
		sets[root].size += sets[child].size;
		sets[root].box = box;
	}

	// Cluster ids in node order; a set of one is a node of its own
	int clusters = 0;
	for ( int i = 0; i < n; ++i )
	{
		int r = lpFindSet( sets, i );
		if ( sets[r].size < 2 )
		{
			continue;
		}
		if ( sets[r].id == 0 )
		{
			sets[r].id = ++clusters;
		}
		w->pieces.data[s->nodes.data[i]].cluster = sets[r].id;
	}
	body->clusters = clusters;
	body->clusterStamp += 1;
	if ( w->def.debugLog )
	{
		int eligible = 0, clustered = 0, largest = 0;
		for ( int i = 0; i < n; ++i )
		{
			eligible += sets[i].eligible ? 1 : 0;
			clustered += w->pieces.data[s->nodes.data[i]].cluster != 0 ? 1 : 0;
			largest = sets[i].parent == i && sets[i].size > largest ? sets[i].size : largest;
		}
		printf( "[lpf] tick %llu stress: body %d, %d of %d nodes may cluster, %d in %d clusters (the largest %d), radius %.2f\n",
				(unsigned long long)w->tick, job->body, eligible, n, clustered, clusters, largest, (double)radius );
	}
}

// Phase 3, one structure: judge its solution. Converged: strain at every overloaded joint and the worst break, then
// the slender pieces. Not converged: it keeps solving next step, and nothing is judged on an unconverged solution.
static void lpStressJudge( lpWorld* w, const lpStressJob* job )
{
	lpBody* body = w->bodies.data + job->body;
	const lpStressSystem* s = job->system;
	int n = s->nodes.count;
	int edges = s->edges.count;
	int solved = job->clustered ? body->reduced->system.edges.count : edges;
	w->stressWork += job->solve.iterations * solved + ( job->cached ? 0 : 2 * edges ); // a build and a first residual
	w->stats.stressIterations += job->solve.iterations;
	w->stats.stressSolves += 1;
	body->solving = job->solve.converged == false;
	body->solveRz = job->solve.rz;
	body->solveTopology = body->topology;
	body->solveNodes = n;
	body->solveEdges = edges;
	body->solveClustered = job->clustered;

	if ( job->oracleWorst >= 0.0f )
	{
		w->oracleSolves += 1;
		w->oracleFlips += job->oracleFlips;
		w->oracleJoints += job->system->edges.count;
		if ( job->oracleWorst > w->oracleWorst )
		{
			w->oracleWorst = job->oracleWorst;
			w->oracleMeter = job->meterWorst;
		}
		if ( w->def.debugLog )
		{
			printf( "[lpf] tick %llu stress: body %d, %s solve against the exact one: worst joint utilization off by %.4f, "
					"%d flipped, the meter read %.4f\n",
					(unsigned long long)w->tick, job->body, "reduced", (double)job->oracleWorst, job->oracleFlips,
					(double)job->meterWorst );
		}
	}
	if ( job->dissolved > 0 )
	{
		body->meterRounds += 1;
		w->stats.stressDissolved += job->dissolved;
		if ( w->def.debugLog )
		{
			printf( "[lpf] tick %llu stress: body %d, the residual meter dissolved %d clusters (round %d)\n",
					(unsigned long long)w->tick, job->body, job->dissolved, body->meterRounds );
		}
	}
	if ( job->solve.converged == false )
	{
		body->stressSteps += 1;
		body->unsettled = true;
		for ( int k = 0; k < edges; ++k )
		{
			int bi = s->edges.data[k].bond;
			const lpBond* bond = w->bonds.data + bi;
			if ( bond->strain > 0.0f && ( w->tick + (uint64_t)bi ) % 4 == 0 )
			{
				lpStressDust( w, job->xf, bond, bi, 1 ); // joints that were straining keep creaking while it solves
			}
		}
		lpArray_Push( w->stressAgain, job->body );
		return;
	}

	w->stats.stressJudged += 1;
	w->stats.stressReduced += job->clustered ? 1 : 0;
	body->meterRounds = 0;

	// Judged on a reduced system: provisional until an exact solve (an audit) confirms it
	if ( job->clustered )
	{
		if ( body->provisional == false )
		{
			body->provisional = true;
			body->provisionalTick = w->tick + 1;
			lpBodyRef ref = { job->body, body->generation };
			lpArray_Push( w->audits, ref );
		}
	}
	else
	{
		w->stats.stressAudits += body->auditing ? 1 : 0;
		body->provisional = false; // a queued entry is skipped when it comes up
		body->auditing = false;
	}

	// Accepted: the pieces changed since the last judgement were its seeds
	int seeds = 0;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		lpPiece* p = w->pieces.data + body->pieces.data[i];
		seeds += p->changed > p->accepted ? 1 : 0;
		p->accepted = w->changeSerial;
	}
	body->rejudge = false;

	int strained = 0;
	w->scratchOverloads.count = 0;
	for ( int k = 0; k < edges; ++k )
	{
		lpApplyStrain( w, job->xf, s->edges.data[k].bond, s->rho.data[k], &strained );
	}
	int broken = lpBreakOverloads( w, job->xf );
	int slender = 0;
	int snapped = lpStressSnap( w, job, broken == 0, &strained, &slender ); // slender pieces strain once the joints hold
	if ( job->clustered == false && n > w->def.stressLargeNodes && body->solveStress == false )
	{
		lpFormClusters( w, job ); // from an exact solve only: its next corrections move the unloaded parts rigidly
	}
	body->unsettled = broken > 0 || snapped > 0 || strained > 0;
	body->creaking = broken == 0 && snapped == 0 && strained > 0; // next checks only add strain, joints and slender pieces
	if ( body->solveStress )
	{
		// A moving body's loads are a moment's (a crash, a landing): judged once, no creaking on; what broke splits
		body->creaking = false;
		body->unsettled = broken > 0 || snapped > 0;
	}
	body->strainedLastCheck = strained > 0;
	w->stats.stressBreaks += broken;
	if ( w->def.debugLog )
	{
		printf( "[lpf] tick %llu stress: body %d, %d nodes (%d changed), %d bonds, %d iterations over %d steps, peak "
				"utilization %.2f, %d strained, %d broke; solved on %d groups and %d bonds, %d clusters now\n",
				(unsigned long long)w->tick, job->body, n, seeds, edges, job->solve.iterations, body->stressSteps + 1,
				(double)job->peak, strained, broken, job->clustered ? body->reduced->system.nodes.count : n,
				job->clustered ? body->reduced->system.edges.count : edges, body->clusters );
	}
	body->stressSteps = 0;
	if ( body->unsettled )
	{
		lpArray_Push( w->stressAgain, job->body ); // broken joints split it next step; strained ones creak on
	}
}

// Seeds (phase 1): the pieces changed since the last judged solve (bonds, their health, loads), and everything within
// LP_SEED_HOPS bonds of them, leave their clusters, so a correction resolves the change and the load finding its way
// around it finely: a rigid cluster at the rim of a hole would carry the arching load stiffly and wrongly
static void lpStressSeed( lpWorld* w, lpBody* body )
{
	w->stamp += 1;
	int stamp = w->stamp;
	w->scratchQueue.count = 0;
	lpArray_Reserve( w->scratchQueue, body->pieces.count );
	lpArray_Reserve( w->scratchClusters, body->pieces.count );
	int* depth = w->scratchClusters.data; // hops from a seed, by queue position
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		lpPiece* p = w->pieces.data + pi;
		if ( p->changed > p->accepted )
		{
			p->mark = stamp;
			depth[w->scratchQueue.count] = 0;
			w->scratchQueue.data[w->scratchQueue.count++] = pi;
		}
	}
	bool changed = false;
	for ( int head = 0; head < w->scratchQueue.count; ++head )
	{
		int pi = w->scratchQueue.data[head];
		lpPiece* p = w->pieces.data + pi;
		changed = changed || p->cluster != 0;
		p->cluster = 0;
		if ( depth[head] == LP_SEED_HOPS )
		{
			continue;
		}
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			const lpBond* bond = w->bonds.data + p->bonds.data[k];
			int other = bond->a == pi ? bond->b : bond->a;
			if ( w->pieces.data[other].mark != stamp )
			{
				w->pieces.data[other].mark = stamp;
				depth[w->scratchQueue.count] = depth[head] + 1;
				w->scratchQueue.data[w->scratchQueue.count++] = other;
			}
		}
	}
	if ( changed )
	{
		body->clusterStamp += 1;
	}
}

// The bookkeeping of a check with no solve (lpStressRejudge)
static void lpStressRejudgeBody( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	int creaks = 0, snapped = 0;
	int broke = lpStressRejudge( w, bodyIndex, body->rejudge, &creaks, &snapped );
	body->rejudge = false;
	body->creaking = broke == 0 && snapped == 0 && creaks > 0;
	body->unsettled = broke > 0 || snapped > 0 || creaks > 0;
	body->strainedLastCheck = creaks > 0;
	w->stats.stressBreaks += broke;
	if ( body->unsettled )
	{
		lpArray_Push( w->stressAgain, bodyIndex ); // broken joints split it next step; strained ones creak on
	}
}

static void lpRunStressChecks( lpWorld* w, bool settle )
{
	uint64_t ticks = lpGetTicks();
	lpVec3 gravity = lpPhys_GetGravity( w->phys );
	int reserved = 0;
	w->stressJobCount = 0;

	// The budget is shared by the structures that want to solve: one alone may use all of it (the per-structure cap
	// bounds a step's time when many solve in parallel, and is moot then)
	int wanting = 0;
	for ( int qi = 0; qi < w->stressQueue.count; ++qi )
	{
		const lpBody* body = w->bodies.data + w->stressQueue.data[qi];
		wanting += body->solving || body->solveTopology != body->topology || body->reloadLoads || body->auditing ? 1 : 0;
	}
	int share = lpMaxInt( w->def.maxStressStructureWork, w->def.maxStressWork / lpMaxInt( wanting, 1 ) );

	// Phase 1, in queue order: what needs no solve, and what the solves may spend
	for ( int qi = 0; qi < w->stressQueue.count; ++qi )
	{
		int bodyIndex = w->stressQueue.data[qi];
		lpBody* body = w->bodies.data + bodyIndex;
		body->unsettled = false;
		if ( w->def.stressScale <= 0.0f || body->pieces.count < 2 )
		{
			body->solving = false;
			body->reloadLoads = false;
			body->rejudge = false;
			continue;
		}

		// Converged, unchanged, and nothing new resting on it: creaking joints add strain, and joints a blast weakened
		// are judged again from the last solve's forces, with no solve
		bool unchanged = body->solving == false && body->solveTopology == body->topology;
		if ( unchanged && body->reloadLoads == false && ( body->creaking || body->rejudge ) )
		{
			lpStressRejudgeBody( w, bodyIndex );
			continue;
		}

		// An audit gives way to a change: the change is solved at the usual fidelity, and the audit waits, first in line
		if ( body->auditing && body->solving && ( body->reloadLoads || body->solveTopology != body->topology ) )
		{
			body->auditing = false;
			w->audit.body = -1;
			lpBodyRef ref = { bodyIndex, body->generation };
			lpArray_Push( w->audits, ref );
			memmove( w->audits.data + 1, w->audits.data, sizeof( lpBodyRef ) * (size_t)( w->audits.count - 1 ) );
			w->audits.data[0] = ref;
		}

		// A clustered structure's changes are seeded, unless a solve is still running on them
		if ( body->clusters > 0 && ( body->solving == false || body->reloadLoads || body->solveTopology != body->topology ) )
		{
			lpStressSeed( w, body );
		}
		if ( body->solveStress )
		{
			lpChoosePin( w, body, bodyIndex ); // kept while a solve continues
		}
		int nodes, edges, groups, reducedEdges;
		lpStressCount( w, body, &nodes, &edges, &groups, &reducedEdges );
		if ( nodes == 0 || edges == 0 )
		{
			body->solving = false;
			body->creaking = false;
			body->reloadLoads = false;
			body->rejudge = false;
			continue;
		}

		// A solve in progress continues if the structure and its loads have not changed since the last step. It still has
		// its system: no build, and it is charged only its iterations (a piece that left without a bond to break
		// changes the counts, not the topology).
		// The fidelity it is solved at: exact (settling, auditing, a small structure, no clusters left), or on its reduced
		// system for a correction to its last solution (parts of it moving as rigid clusters), provisional until audited
		bool exact = settle || body->auditing || nodes <= w->def.stressLargeNodes || body->solveStress;
		bool clustered = exact == false && groups < nodes && reducedEdges > 0;
		const lpStressSystem* system = body->system;
		const lpStressReduced* red = body->reduced;
		bool continuing = body->solving && body->reloadLoads == false && body->solveTopology == body->topology &&
						  body->solveNodes == nodes && body->solveEdges == edges;
		bool cached = continuing && system != NULL && system->built && system->topology == body->topology &&
					  system->nodes.count == nodes && system->edges.count == edges &&
					  ( clustered ? red != NULL && red->built && red->topology == body->topology &&
									   red->clusterStamp == body->clusterStamp
								  : system->factored );
		// A correction continues only on its reduced system, and a solve only in the mode it started in
		continuing = continuing && body->solveClustered == clustered && ( cached || clustered == false );
		int solvedEdges = clustered ? reducedEdges : edges;
		int overhead = cached ? 0 : 2 * edges; // a build and the first residual

		// Its share of the budget, reserved whole before anything is built. The first structure of a step always gets an
		// iteration, so even one bigger than the budget makes progress. One that does not fit waits, having cost
		// nothing, and goes first next step. Settling has no budget.
		int mine = body->auditing ? lpMinInt( share, (int)( LP_AUDIT_SHARE * (float)w->def.maxStressWork ) ) : share;
		int room = ( lpMinInt( mine, w->def.maxStressWork - reserved ) - overhead ) / solvedEdges;
		int budget = lpMinInt( room, w->def.maxStressIterations );
		budget = budget < 1 && reserved == 0 ? 1 : budget;
		budget = settle ? w->def.maxSettleIterations : budget;
		if ( budget < 1 )
		{
			body->unsettled = true;
			w->stats.stressWaiting += 1;
			lpArray_Push( w->stressAgain, bodyIndex );
			continue;
		}

		if ( continuing == false )
		{
			// Asked again with the structure unchanged since it settled: only something landing on it or leaving it can
			// matter. If what rests on it barely changed, it stays settled without a solve (joints a blast weakened are
			// judged again from the last forces).
			bool loadOnly = unchanged && body->strainedLastCheck == false && body->auditing == false;
			float change = lpSampleLoads( w, bodyIndex );
			body->reloadLoads = false;
			if ( body->solveStress )
			{
				lpWorldTransform bodyXf = lpPhys_GetTransform( w->phys, body->id );
				lpComputeRelief( w, body, lpMulSV( body->gravityScale, lpInvRotateVector( bodyXf.q, gravity ) ) );
			}
			if ( loadOnly && change < 0.02f )
			{
				if ( body->rejudge )
				{
					lpStressRejudgeBody( w, bodyIndex );
				}
				continue;
			}
		}
		body->creaking = false;

		reserved += budget * solvedEdges + overhead;
		if ( body->system == NULL )
		{
			body->system = lpAlloc( sizeof( lpStressSystem ) );
			memset( body->system, 0, sizeof( lpStressSystem ) );
		}
		if ( clustered && body->reduced == NULL )
		{
			body->reduced = lpAlloc( sizeof( lpStressReduced ) );
			memset( body->reduced, 0, sizeof( lpStressReduced ) );
		}
		lpStressJob* job = lpAddStressJob( w );
		job->body = bodyIndex;
		job->system = body->system;
		job->xf = lpPhys_GetTransform( w->phys, body->id );
		job->gravity = lpMulSV( body->gravityScale, lpInvRotateVector( job->xf.q, gravity ) );
		job->nodeCount = nodes;
		job->edgeCount = edges;
		job->budget = budget;
		job->continuing = continuing;
		job->cached = cached;
		job->clustered = clustered;
		// Patience: after many steps without a judgement (restarts count too), the tolerances relax so it is judged
		bool patient = body->stressSteps >= w->def.stressPatience;
		job->tolerance = patient ? 1e-2 : 1e-3;
		job->nodeTolerance = patient ? 0.25f : 0.05f;
		job->solve.rz = body->solveRz;
	}
	w->stressQueue.count = 0;

	// Phase 2: every structure builds and solves at once
	lpTaskPool_ParallelFor( w->tasks, w->stressJobCount, lpRunStressJob, w );

	// Phase 3, in queue order: judge the solutions
	for ( int i = 0; i < w->stressJobCount; ++i )
	{
		lpStressJudge( w, w->stressJobs + i );
	}
	w->stats.stressMs += lpGetMilliseconds( ticks );
}

// Once the step's stress budget has been at most half used, with nothing waiting, for LP_CALM_STEPS steps, or once the
// oldest provisional structure has waited LP_AUDIT_AGE steps, the oldest is solved exactly (an audit, one at a time),
// continuing across steps on at most LP_AUDIT_SHARE of the budget. It is judged as usual, so a collapse a provisional
// judgement missed comes a beat late; its clusters are formed again from it.
static void lpStressAudits( lpWorld* w )
{
	bool calm = w->stats.stressWaiting == 0 && w->stressWork <= w->def.maxStressWork / 2;
	w->calmSteps = calm ? w->calmSteps + 1 : 0;
	if ( w->audit.body >= 0 )
	{
		const lpBody* b = w->bodies.data + w->audit.body;
		if ( b->alive && b->generation == w->audit.generation && b->auditing )
		{
			return;
		}
		w->audit.body = -1;
	}
	while ( w->audits.count > 0 )
	{
		lpBodyRef ref = w->audits.data[0];
		lpBody* b = w->bodies.data + ref.body;
		if ( b->alive == false || b->generation != ref.generation || b->provisional == false )
		{
			memmove( w->audits.data, w->audits.data + 1, sizeof( lpBodyRef ) * (size_t)( w->audits.count - 1 ) );
			w->audits.count -= 1;
			continue; // judged exactly since, or gone
		}
		if ( w->calmSteps < LP_CALM_STEPS && w->tick + 1 < b->provisionalTick + LP_AUDIT_AGE )
		{
			return;
		}
		memmove( w->audits.data, w->audits.data + 1, sizeof( lpBodyRef ) * (size_t)( w->audits.count - 1 ) );
		w->audits.count -= 1;
		b->auditing = true;
		w->audit = ref;
		lpMarkDirty( w, ref.body );
		if ( w->def.debugLog )
		{
			printf( "[lpf] tick %llu stress: body %d audited (provisional since tick %llu, %d calm steps)\n", (unsigned long long)w->tick,
					ref.body, (unsigned long long)( b->provisionalTick - 1 ), w->calmSteps );
		}
		return;
	}
}

int lpCheckStructures( lpWorld* w, bool settle )
{
	int iterations = 0;
	if ( w->stressQueue.count > 0 )
	{
		lpRunStressChecks( w, settle );
		for ( int i = 0; i < w->stressJobCount; ++i )
		{
			iterations += w->stressJobs[i].solve.iterations;
		}
	}
	for ( int i = 0; i < w->stressAgain.count; ++i )
	{
		lpMarkDirty( w, w->stressAgain.data[i] ); // still solving or straining: check again next step
	}
	w->stressAgain.count = 0;
	if ( settle == false )
	{
		lpStressAudits( w );
	}
	return iterations;
}

int lpWorld_SettleStructures( lpWorld* w )
{
	// A structure that loses joints to its own weight splits and is solved again, a few rounds, so what a scene does at
	// load is over before its first step. One that only strains creaks on in the steps.
	uint64_t ticks = lpGetTicks();
	int iterations = 0;
	for ( int round = 0; round < 8; ++round )
	{
		int breaks = w->stats.stressBreaks;
		lpUpdateDirtyBodies( w );
		iterations += lpCheckStructures( w, true );
		if ( w->stats.stressBreaks == breaks )
		{
			break;
		}
	}
	w->stats.settleMs = lpGetMilliseconds( ticks );
	w->stats.settleIterations = iterations;
	w->stats.bondCount = w->bondCount;
	return iterations;
}

void lpRequestStressCheck( lpWorld* w, int bodyIndex, bool duringSplits )
{
	lpBody* b = w->bodies.data + bodyIndex;
	b->reloadLoads = true; // a solve in progress restarts with them; one that had settled solves again if they changed
	if ( b->provisional && b->auditing == false )
	{
		// Something is happening to it: its audit comes first
		for ( int i = 1; i < w->audits.count; ++i )
		{
			if ( w->audits.data[i].body == bodyIndex && w->audits.data[i].generation == b->generation )
			{
				lpBodyRef ref = w->audits.data[i];
				memmove( w->audits.data + 1, w->audits.data, sizeof( lpBodyRef ) * (size_t)i );
				w->audits.data[0] = ref;
				break;
			}
		}
	}
	if ( duringSplits )
	{
		lpArray_Push( w->stressAgain, bodyIndex );
	}
	else
	{
		lpMarkDirty( w, bodyIndex );
	}
}

void lpFreeStressSystem( lpBody* b )
{
	if ( b->system != NULL )
	{
		lpSystemFree( b->system );
		lpFree( b->system );
		b->system = NULL;
	}
	if ( b->reduced != NULL )
	{
		lpPartitionFree( &b->reduced->partition );
		lpArray_Free( b->reduced->nodeRef );
		lpSystemFree( &b->reduced->system );
		lpFree( b->reduced );
		b->reduced = NULL;
	}
}
