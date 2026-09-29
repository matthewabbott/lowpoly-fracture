// SPDX-License-Identifier: MIT
// Stress: a quasi-static solve on each structure's bond graph, so buildings fall where they are weak.
//
// Pieces are rigid nodes with a small translation and rotation; anchored pieces are fixed to the world. A bond is a
// short beam through its contact patch with axial, shear, bending and twisting stiffness from the patch's area and
// extents (the parallel-bond model of rock and masonry simulation). Solving K x = gravity gives every bond's force
// and moment, hence the tension, compression and shear at its most loaded fibre. Mortar and dry joints carry almost
// no tension, so an overhang's moment opens the tension side first: the part above hinges off, loses its anchor and
// Box3D topples it.
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

#include "tasks.h"
#include "world.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

static const lpVec6 lp_vec6Zero = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };

// What rests on a structure: dynamic bodies pressing on its pieces, from the last physics step's contact impulses
// (rubble on a floor, a stone on a plank, a cart on a bridge), and what hangs on it by links, into each piece's
// stressLoad. Sampled when a solve starts and kept, so the solve can continue across steps while the contacts jitter.
// Returns how much the loads changed (forces, and torques over each piece's size), relative to the structure's own
// weight; a piece whose load changed by more than 2% of its own weight is stamped.
static float lpSampleLoads( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	b3WorldTransform xf = b3Body_GetTransform( body->id );
	int n = body->pieces.count;
	lpArray_Reserve( w->scratchLoads, n );
	float weight = 0.0f;
	float g = body->gravityScale * b3Length( b3World_GetGravity( w->def.physics ) );
	for ( int i = 0; i < n; ++i )
	{
		lpPiece* p = w->pieces.data + body->pieces.data[i];
		w->scratchLoads.data[i] = p->stressLoad;
		p->stressLoad = lp_vec6Zero;
		weight += p->shape->volume * lpGetMaterial( p->material )->density * g;
	}

	int capacity = b3Body_GetContactCapacity( body->id );
	if ( capacity > 0 && w->lastTimeStep > 0.0f )
	{
		lpArray_Reserve( w->scratchContacts, capacity );
		int count = b3Body_GetContactData( body->id, w->scratchContacts.data, capacity );
		for ( int k = 0; k < count; ++k )
		{
			const b3ContactData* contact = w->scratchContacts.data + k;
			intptr_t da = (intptr_t)b3Shape_GetUserData( contact->shapeIdA );
			intptr_t db = (intptr_t)b3Shape_GetUserData( contact->shapeIdB );
			bool mineA = da > 0 && w->pieces.data[da - 1].body == bodyIndex;
			bool mineB = db > 0 && w->pieces.data[db - 1].body == bodyIndex;
			if ( mineA == mineB )
			{
				continue;
			}
			lpPiece* piece = w->pieces.data + ( mineA ? da : db ) - 1;
			if ( piece->anchored )
			{
				continue; // the ground takes it
			}
			b3Pos centerA = b3Body_GetWorldCenter( b3Shape_GetBody( contact->shapeIdA ) );
			for ( int mi = 0; mi < contact->manifoldCount; ++mi )
			{
				const b3Manifold* manifold = contact->manifolds + mi;
				for ( int pi = 0; pi < manifold->pointCount; ++pi )
				{
					const b3ManifoldPoint* mp = manifold->points + pi;
					if ( mp->totalNormalImpulse <= 0.0f )
					{
						continue;
					}
					// The normal points from A to B: the impulse pushes B along it and A against it
					float sign = mineA ? -1.0f : 1.0f;
					b3Vec3 force = b3MulSV( sign * mp->totalNormalImpulse / w->lastTimeStep, manifold->normal );
					b3Vec3 local = b3InvRotateVector( xf.q, force );
					b3Vec3 point = b3InvTransformWorldPoint( xf, b3OffsetPos( centerA, mp->anchorA ) );
					piece->stressLoad.f = b3Add( piece->stressLoad.f, local );
					piece->stressLoad.t = b3Add( piece->stressLoad.t, b3Cross( b3Sub( point, piece->shape->centroid ), local ) );
				}
			}
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
			b3Vec3 force = b3InvRotateVector( xf.q, b3MulSV( sign, l->force ) );
			b3Vec3 torque = b3InvRotateVector( xf.q, b3MulSV( sign, l->torque ) );
			b3Vec3 arm = b3Sub( l->ends[end].frame.p, piece->shape->centroid );
			piece->stressLoad.f = b3Add( piece->stressLoad.f, force );
			piece->stressLoad.t = b3Add( piece->stressLoad.t, b3Add( b3Cross( arm, force ), torque ) );
		}
	}

	float change = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		int pi = body->pieces.data[i];
		const lpPiece* p = w->pieces.data + pi;
		float size = cbrtf( p->shape->volume );
		float moved = b3Length( b3Sub( p->stressLoad.f, w->scratchLoads.data[i].f ) ) +
					  b3Length( b3Sub( p->stressLoad.t, w->scratchLoads.data[i].t ) ) / size;
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
		if ( p->anchored == false )
		{
			*nodes += 1;
			*groups += p->cluster == 0 || w->scratchClusters.data[p->cluster] == 0 ? 1 : 0;
			w->scratchClusters.data[p->cluster] = 1;
		}
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			const lpBond* bond = w->bonds.data + p->bonds.data[k];
			const lpPiece* other = w->pieces.data + bond->b;
			if ( bond->a == pi && ( p->anchored == false || other->anchored == false ) )
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
			lpArray_Push( part->ref, b3Vec3_zero );
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
		part->ref.data[g] = b3MulAdd( part->ref.data[g], mass, p->shape->centroid );
	}
	for ( int g = 0; g < part->groupCount; ++g )
	{
		if ( part->members.data[g] > 1 )
		{
			part->ref.data[g] = b3MulSV( 1.0f / job->groupMass.data[g], part->ref.data[g] );
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

	// The last solution's residual under the new loads and bonds (x holds it; z is free in this form)
	lpVec6* x = s->vectors.data;
	lpVec6* f = x + n;
	lpVec6* r = x + 2 * n;
	lpVec6* q = x + 5 * n;
	lpSystemApply( s, x, q );
	double load2 = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		r[i].f = b3Sub( f[i].f, q[i].f );
		r[i].t = b3Sub( f[i].t, q[i].t );
		load2 += (double)b3Dot( f[i].f, f[i].f ) + (double)b3Dot( f[i].t, f[i].t );
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

	// Each group balances to the tolerance of its members together
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
		float arm = b3Distance( red->nodeRef.data[i], part->ref.data[g] ) + s->nodeArm.data[i];
		rs->nodeScale.data[g] += s->nodeScale.data[i];
		rs->nodeArm.data[g] = b3MaxFloat( rs->nodeArm.data[g], arm );
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
	b3Vec3 g = job->gravity;
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
			if ( p->anchored == false )
			{
				p->solveSlot = slot++;
				lpArray_Push( s->nodes, pi );
				float weight = p->shape->volume * lpGetMaterial( p->material )->density;
				heaviest = weight > heaviest ? weight : heaviest;
			}
		}
		LP_ASSERT( slot == n );
		float scale = heaviest * b3Length( g );
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
				e.ra = b3Sub( bond->centroid, pa->shape->centroid );
				e.rb = b3Sub( bond->centroid, pb->shape->centroid );
				e.n = bond->normal;
				lpContactBasis( e.n, &e.t1, &e.t2 );
				float len = b3MaxFloat( b3Distance( pa->shape->centroid, pb->shape->centroid ), 0.05f );
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
			x[i].f = b3MulSV( 1.0f / scale, p->stressX.f );
			x[i].t = b3MulSV( 1.0f / scale, p->stressX.t );
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
		f[i].f = b3MulSV( mass / scale, g );
		f[i].t = b3Vec3_zero;
	}
	for ( int i = 0; i < n; ++i )
	{
		const lpPiece* p = w->pieces.data + s->nodes.data[i];
		f[i].f = b3MulAdd( f[i].f, 1.0f / scale, p->stressLoad.f );
		f[i].t = b3MulAdd( f[i].t, 1.0f / scale, p->stressLoad.t );
	}

	// For the per-node equilibrium test: each node's size, and the forces through it at the warm start (a crumb must
	// balance to a fraction of a crumb, a stone carrying the wall above it to a fraction of that wall)
	lpArray_Reserve( s->nodeArm, n );
	s->nodeArm.count = n;
	for ( int i = 0; i < n; ++i )
	{
		s->nodeArm.data[i] = cbrtf( w->pieces.data[s->nodes.data[i]].shape->volume );
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
	*tension = b3MinFloat( ma->tensileStrength, mb->tensileStrength );
	*compression = b3MinFloat( ma->compressiveStrength, mb->compressiveStrength );
	*shear = b3MinFloat( ma->shearStrength, mb->shearStrength );
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
static void lpStressDust( lpWorld* w, b3WorldTransform xf, const lpBond* bond, int bondIndex, int motes )
{
	uint8_t material = w->pieces.data[bond->a].material;
	uint64_t h = lpMix64( ( w->tick << 24 ) ^ (uint64_t)bondIndex );
	for ( int k = 0; k < motes; ++k )
	{
		h = lpMix64( h + (uint64_t)k );
		float rx = (float)( h & 0xFFFF ) / 65535.0f - 0.5f;
		float rz = (float)( ( h >> 16 ) & 0xFFFF ) / 65535.0f - 0.5f;
		float rs = (float)( ( h >> 32 ) & 0xFFFF ) / 65535.0f;
		b3Vec3 v = { 0.6f * rx, -0.3f - 0.4f * rs, 0.6f * rz };
		lpEmitParticle( w, xf, bond->centroid, v, 0.02f + 0.02f * rs, material );
	}
}

// A joint over its limit strains a little more every check and creaks; at strain 1 it is queued to break
static void lpApplyStrain( lpWorld* w, b3WorldTransform xf, int bondIndex, float rho, int* strained )
{
	lpBond* bond = w->bonds.data + bondIndex;
	bond->rho = rho;
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
static int lpBreakOverloads( lpWorld* w, b3WorldTransform xf )
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
static float lpBondUtilization( const lpWorld* w, const lpBond* bond, b3Vec3 t1, b3Vec3 t2, b3Vec3 force, b3Vec3 moment )
{
	b3Vec3 n = bond->normal;
	float area = bond->area;
	float axialForce = b3Dot( force, n ); // tension positive
	float compression = b3MaxFloat( -axialForce, 0.0f );
	float m1 = b3AbsFloat( b3Dot( moment, t1 ) ); // bending about t1: the fibres at +-h2 carry it
	float m2 = b3AbsFloat( b3Dot( moment, t2 ) );
	b3Vec3 shearForce = b3MulSub( force, axialForce, n );
	float hMax = b3MaxFloat( bond->h1, bond->h2 );
	float shear = b3Length( shearForce ) / area +
				  3.0f * b3AbsFloat( b3Dot( moment, n ) ) * hMax / ( area * ( bond->h1 * bond->h1 + bond->h2 * bond->h2 ) );

	float tensionLimit, compressionLimit, shearLimit, mu;
	lpBondLimits( w, bond, &tensionLimit, &compressionLimit, &shearLimit, &mu );
	float str = w->def.stressScale * b3MaxFloat( bond->health, 0.0f ) / bond->strength;
	float tensionCap = b3MaxFloat( str * tensionLimit, 1e4f );

	// Pulling apart and bending: the section's elastic tension capacity, plus rocking. A compressed joint holds a moment
	// until its resultant reaches the edge of the patch, so masonry tips over an edge instead of cracking as soon as the
	// load leaves the middle third.
	float cap1 = tensionCap * area * bond->h2 / 3.0f + compression * bond->h2;
	float cap2 = tensionCap * area * bond->h1 / 3.0f + compression * bond->h1;
	float rho = b3MaxFloat( axialForce, 0.0f ) / ( tensionCap * area ) + m1 / cap1 + m2 / cap2;

	// Crushing at the compressed edge, and Coulomb shear
	float edge = compression / area + 3.0f * m1 / ( area * bond->h2 ) + 3.0f * m2 / ( area * bond->h1 );
	rho = b3MaxFloat( rho, edge / b3MaxFloat( str * compressionLimit, 1e4f ) );
	return b3MaxFloat( rho, shear / ( str * ( shearLimit + mu * compression / area ) + 1e3f ) );
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
		b3Vec3 force, moment;
		lpEdgeForce( e, x, &force, &moment );
		bond->force = b3MulSV( s->forceScale, force );
		bond->moment = b3MulSV( s->forceScale, moment );
		float rho = lpBondUtilization( w, bond, e->t1, e->t2, bond->force, bond->moment );
		s->rho.data[k] = rho;
		peak = rho > peak ? rho : peak;
	}
	job->peak = peak;
}

// A converged structure that has not changed since, but creaks (joints over their limit, none broken yet) or had
// joints weakened by a blast: its joints are judged again from the forces of its last solve, with no solve. Creaking
// only adds strain from the stored utilizations; after a blast (recompute) they are computed again from the stored
// forces and the joints' new health.
static int lpStressRejudge( lpWorld* w, int bodyIndex, bool recompute, int* strained )
{
	lpBody* body = w->bodies.data + bodyIndex;
	b3WorldTransform xf = b3Body_GetTransform( body->id );
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
			if ( recompute && ( p->anchored == false || w->pieces.data[bond->b].anchored == false ) )
			{
				b3Vec3 t1, t2;
				lpContactBasis( bond->normal, &t1, &t2 );
				rho = lpBondUtilization( w, bond, t1, t2, bond->force, bond->moment );
			}
			if ( rho > 1.0f || rho != bond->rho )
			{
				lpApplyStrain( w, xf, bi, rho, strained );
			}
		}
	}
	return lpBreakOverloads( w, xf );
}

// Long pieces (beams, planks, columns, lintels) are rigid nodes, so the solve cannot bend them. Phase 2, converged: from
// the solved bond forces and the piece's own weight, find the bending moment along its axis at a few cuts between its
// supports, and keep the worst (into the job). Phase 3 (lpStressSnap) strains the overloaded ones.
static void lpStressSlender( lpWorld* w, lpStressJob* job, const lpVec6* x )
{
	const lpStressSystem* s = job->system;
	b3Vec3 g = job->gravity;
	float forceScale = s->forceScale;
	for ( int i = 0; i < s->nodes.count; ++i )
	{
		int pi = s->nodes.data[i];
		const lpPiece* p = w->pieces.data + pi;
		const lpMaterialDef* m = lpGetMaterial( p->material );
		float fragment = m->fragmentSize * w->def.fragmentScale;
		if ( m->breakable == false || m->pattern == lp_breakRadial || p->depth >= w->def.maxDepth || p->bonds.count < 2 )
		{
			continue;
		}

		// Extents along the axis and across it
		b3Vec3 a = p->axis, t1, t2;
		lpContactBasis( a, &t1, &t2 );
		b3Vec3 c = p->shape->centroid;
		float lo = FLT_MAX, hi = -FLT_MAX, w1 = 0.0f, w2 = 0.0f;
		for ( int k = 0; k < p->shape->vertexCount; ++k )
		{
			b3Vec3 d = b3Sub( p->shape->vertices[k], c );
			float along = b3Dot( d, a );
			lo = along < lo ? along : lo;
			hi = along > hi ? along : hi;
			w1 = b3MaxFloat( w1, b3AbsFloat( b3Dot( d, t1 ) ) );
			w2 = b3MaxFloat( w2, b3AbsFloat( b3Dot( d, t2 ) ) );
		}
		float length = hi - lo;
		if ( length < 2.5f * 2.0f * b3MaxFloat( w1, w2 ) || length < 4.0f * fragment )
		{
			continue; // not slender: the joints decide
		}

		// Loads on the piece: each bond's force at its contact, and the moment it carries
		float bondLo = FLT_MAX, bondHi = -FLT_MAX;
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			float along = b3Dot( b3Sub( w->bonds.data[p->bonds.data[k]].centroid, c ), a );
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
			b3Vec3 q = b3MulAdd( c, cut, a );
			b3Vec3 forceSum = b3Vec3_zero, momentSum = b3Vec3_zero;
			for ( int j = first; j < last; ++j )
			{
				const lpStressEdge* e = s->edges.data + incident[j];
				const lpBond* bond = w->bonds.data + e->bond;
				if ( b3Dot( b3Sub( bond->centroid, c ), a ) <= cut )
				{
					continue;
				}
				b3Vec3 f, mo;
				lpEdgeForce( e, x, &f, &mo );
				float sign = e->a == i ? forceScale : -forceScale; // what the bond does to this piece
				f = b3MulSV( sign, f );
				mo = b3MulSV( sign, mo );
				forceSum = b3Add( forceSum, f );
				momentSum = b3Add( momentSum, b3Add( b3Cross( b3Sub( bond->centroid, q ), f ), mo ) );
			}
			// The piece's own weight beyond the cut, spread evenly along its length
			float share = ( hi - cut ) / length;
			b3Vec3 weight = b3MulSV( share * mass, g );
			b3Vec3 at = b3MulAdd( c, 0.5f * ( hi + cut ), a );
			forceSum = b3Add( forceSum, weight );
			momentSum = b3Add( momentSum, b3Cross( b3Sub( at, q ), weight ) );

			float sigma = b3AbsFloat( b3Dot( forceSum, a ) ) / area + 3.0f * b3AbsFloat( b3Dot( momentSum, t1 ) ) / ( area * w2 ) +
						  3.0f * b3AbsFloat( b3Dot( momentSum, t2 ) ) / ( area * w1 );
			float rho = sigma / limit;
			if ( rho > worst )
			{
				worst = rho;
				worstAt = cut;
			}
		}
		lpSlenderCut slenderCut = { pi, length, worst, worstAt, 2.0f * b3MaxFloat( w1, w2 ) };
		lpArray_Push( job->slender, slenderCut );
	}
}

// Phase 3, once the joints hold: an overloaded slender piece strains, and at 1 a small synthetic impact at its worst
// section snaps it through the normal fracture pipeline next step. Returns the number of pieces queued to break.
static int lpStressSnap( lpWorld* w, const lpStressJob* job, int* strained, int* slender )
{
	b3WorldTransform xf = job->xf;
	int queued = 0;
	for ( int k = 0; k < job->slender.count; ++k )
	{
		const lpSlenderCut* cut = job->slender.data + k;
		int pi = cut->piece;
		lpPiece* p = w->pieces.data + pi;
		const lpMaterialDef* m = lpGetMaterial( p->material );
		if ( w->def.debugLog )
		{
			printf( "[lpf]   slender piece %d: length %.2f, worst utilization %.2f at %.2f, strain %.2f\n", pi, (double)cut->length,
					(double)cut->worst, (double)cut->worstAt, (double)p->strain );
		}
		if ( cut->worst <= 1.0f )
		{
			continue;
		}
		*strained += 1;
		*slender += 1;
		p->strain += ( cut->worst - 1.0f ) * w->def.strainRate;
		if ( p->strain < 1.0f )
		{
			continue;
		}

		// Snap it where it is weakest: a blow sized to the section, through the normal fracture pipeline
		p->strain = 0.0f;
		lpImpactDef impact = { 0 };
		impact.point = b3TransformWorldPoint( xf, b3MulAdd( p->shape->centroid, cut->worstAt, p->axis ) );
		impact.direction = b3RotateVector( xf.q, p->axis );
		impact.radius = 1.5f * cut->depth;
		impact.energy = 4.0f * b3MaxFloat( m->bondStrength, m->fractureEnergy ) * B3_PI * impact.radius * impact.radius;
		w->impactSerial += 1;
		lpDeferredJob snap = { pi, p->generation, w->impactSerial, impact, true };
		lpArray_Push( w->deferred, snap );
		lpStressDust( w, xf, w->bonds.data + p->bonds.data[0], p->bonds.data[0], 6 );
		queued += 1;
	}
	return queued;
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
	const lpVec6* r = s->vectors.data + 2 * n;
	const lpVec6* p = s->vectors.data + 4 * n;
	for ( int i = 0; i < n; ++i )
	{
		lpPiece* piece = w->pieces.data + s->nodes.data[i];
		piece->stressX.f = b3MulSV( s->forceScale, x[i].f );
		piece->stressX.t = b3MulSV( s->forceScale, x[i].t );
		if ( red == NULL )
		{
			piece->stressR = r[i];
			piece->stressP = p[i];
		}
	}
	job->peak = 0.0f;
	job->slender.count = 0;
	if ( job->solve.converged )
	{
		lpStressUtilizations( w, job, x );
		lpStressSlender( w, job, x );
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

	// Accepted: the pieces changed since the last judgement were its seeds
	int seeds = 0;
	for ( int i = 0; i < n; ++i )
	{
		lpPiece* p = w->pieces.data + s->nodes.data[i];
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
	int snapped = broken == 0 ? lpStressSnap( w, job, &strained, &slender ) : 0; // once the joints hold
	body->unsettled = broken > 0 || snapped > 0 || strained > 0;
	body->creaking = broken == 0 && snapped == 0 && slender == 0 && strained > 0; // next checks only add strain
	body->strainedLastCheck = strained > 0;
	w->stats.stressBreaks += broken;
	if ( w->def.debugLog )
	{
		printf( "[lpf] tick %llu stress: body %d, %d nodes (%d changed), %d bonds, %d iterations over %d steps, peak "
				"utilization %.2f, %d strained, %d broke\n",
				(unsigned long long)w->tick, job->body, n, seeds, edges, job->solve.iterations, body->stressSteps + 1,
				(double)job->peak, strained, broken );
	}
	body->stressSteps = 0;
	if ( body->unsettled )
	{
		lpArray_Push( w->stressAgain, job->body ); // broken joints split it next step; strained ones creak on
	}
}

// The bookkeeping of a check with no solve (lpStressRejudge)
static void lpStressRejudgeBody( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	int creaks = 0;
	int broke = lpStressRejudge( w, bodyIndex, body->rejudge, &creaks );
	body->rejudge = false;
	body->creaking = broke == 0 && creaks > 0;
	body->unsettled = broke > 0 || creaks > 0;
	body->strainedLastCheck = creaks > 0;
	w->stats.stressBreaks += broke;
	if ( body->unsettled )
	{
		lpArray_Push( w->stressAgain, bodyIndex ); // broken joints split it next step; strained ones creak on
	}
}

static void lpRunStressChecks( lpWorld* w, bool settle )
{
	uint64_t ticks = b3GetTicks();
	b3Vec3 gravity = b3World_GetGravity( w->def.physics );
	int reserved = 0;
	w->stressJobCount = 0;

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
		// Parts moving as rigid clusters: it is solved on its reduced system, for a correction to its last solution
		bool clustered = groups < nodes && reducedEdges > 0;
		const lpStressSystem* system = body->system;
		const lpStressReduced* red = body->reduced;
		bool continuing = body->solving && body->reloadLoads == false && body->solveTopology == body->topology &&
						  body->solveNodes == nodes && body->solveEdges == edges;
		bool cached = continuing && system != NULL && system->built && system->topology == body->topology &&
					  system->nodes.count == nodes && system->edges.count == edges &&
					  ( clustered ? red != NULL && red->built && red->topology == body->topology &&
									   red->clusterStamp == body->clusterStamp
								  : system->factored );
		continuing = continuing && ( cached || clustered == false ); // a correction continues only on its reduced system
		int solvedEdges = clustered ? reducedEdges : edges;
		int overhead = cached ? 0 : 2 * edges; // a build and the first residual

		// Its share of the budget, reserved whole before anything is built. The first structure of a step always gets an
		// iteration, so even one bigger than the budget makes progress. One that does not fit waits, having cost
		// nothing, and goes first next step. Settling has no budget.
		int room = ( b3MinInt( w->def.maxStressStructureWork, w->def.maxStressWork - reserved ) - overhead ) / solvedEdges;
		int budget = b3MinInt( room, w->def.maxStressIterations );
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
			bool loadOnly = unchanged && body->strainedLastCheck == false;
			float change = lpSampleLoads( w, bodyIndex );
			body->reloadLoads = false;
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
		job->xf = b3Body_GetTransform( body->id );
		job->gravity = b3MulSV( body->gravityScale, b3InvRotateVector( job->xf.q, gravity ) );
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
	w->stats.stressMs += b3GetMilliseconds( ticks );
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
	return iterations;
}

int lpWorld_SettleStructures( lpWorld* w )
{
	// A structure that loses joints to its own weight splits and is solved again, a few rounds, so what a scene does at
	// load is over before its first step. One that only strains creaks on in the steps.
	uint64_t ticks = b3GetTicks();
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
	w->stats.settleMs = b3GetMilliseconds( ticks );
	w->stats.settleIterations = iterations;
	w->stats.bondCount = w->bondCount;
	return iterations;
}

void lpRequestStressCheck( lpWorld* w, int bodyIndex, bool duringSplits )
{
	lpBody* b = w->bodies.data + bodyIndex;
	b->reloadLoads = true; // a solve in progress restarts with them; one that had settled solves again if they changed
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
