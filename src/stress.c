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
// 2. in parallel: build and solve each structure (a pure function of its own pieces and bonds)
// 3. in queue order: judge each solution, strain and break joints
// Results do not depend on the worker count.

#include "tasks.h"
#include "world.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

static const lpVec6 lp_vec6Zero = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };

// Relative motion of the two sides at the contact, and the elastic force and moment it produces
static void lpEdgeForce( const lpStressEdge* e, const lpVec6* x, b3Vec3* force, b3Vec3* moment )
{
	lpVec6 va = e->a >= 0 ? x[e->a] : lp_vec6Zero;
	lpVec6 vb = e->b >= 0 ? x[e->b] : lp_vec6Zero;
	b3Vec3 delta = b3Sub( b3Add( vb.f, b3Cross( vb.t, e->rb ) ), b3Add( va.f, b3Cross( va.t, e->ra ) ) );
	b3Vec3 phi = b3Sub( vb.t, va.t );
	float dn = b3Dot( delta, e->n );
	*force = b3Add( b3MulSV( e->kn * dn, e->n ), b3MulSV( e->ks, b3MulSub( delta, dn, e->n ) ) );
	b3Vec3 m = b3MulSV( e->kt * b3Dot( phi, e->n ), e->n );
	m = b3MulAdd( m, e->kb1 * b3Dot( phi, e->t1 ), e->t1 );
	*moment = b3MulAdd( m, e->kb2 * b3Dot( phi, e->t2 ), e->t2 );
}

// y = K x, matrix free, in edge order
static void lpStressApply( const lpStressEdge* edges, int edgeCount, const lpVec6* x, lpVec6* y, int nodeCount )
{
	for ( int i = 0; i < nodeCount; ++i )
	{
		y[i] = lp_vec6Zero;
	}
	for ( int k = 0; k < edgeCount; ++k )
	{
		const lpStressEdge* e = edges + k;
		b3Vec3 force, moment;
		lpEdgeForce( e, x, &force, &moment );
		if ( e->a >= 0 )
		{
			y[e->a].f = b3Sub( y[e->a].f, force );
			y[e->a].t = b3Sub( y[e->a].t, b3Add( b3Cross( e->ra, force ), moment ) );
		}
		if ( e->b >= 0 )
		{
			y[e->b].f = b3Add( y[e->b].f, force );
			y[e->b].t = b3Add( y[e->b].t, b3Add( b3Cross( e->rb, force ), moment ) );
		}
	}
}

static double lpDot6( const lpVec6* a, const lpVec6* b, int n )
{
	double sum = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		sum += (double)b3Dot( a[i].f, b[i].f ) + (double)b3Dot( a[i].t, b[i].t );
	}
	return sum;
}

// Block-Jacobi preconditioner: each node's own 6x6 block of K, Cholesky-factored. The coupling between a piece's
// translation and rotation (bonds far from its centroid) is exactly what a plain diagonal misses.
static void lpAddBlock( lpBlock6* block, const lpStressEdge* e, b3Vec3 r )
{
	float n[3] = { e->n.x, e->n.y, e->n.z };
	float t1[3] = { e->t1.x, e->t1.y, e->t1.z };
	float t2[3] = { e->t2.x, e->t2.y, e->t2.z };
	float kc[3][3], kr[3][3], rx[3][3];
	for ( int a = 0; a < 3; ++a )
	{
		for ( int b = 0; b < 3; ++b )
		{
			kc[a][b] = ( a == b ? e->ks : 0.0f ) + ( e->kn - e->ks ) * n[a] * n[b];
			kr[a][b] = e->kt * n[a] * n[b] + e->kb1 * t1[a] * t1[b] + e->kb2 * t2[a] * t2[b];
		}
	}
	// rx v = r x v
	rx[0][0] = 0.0f, rx[0][1] = -r.z, rx[0][2] = r.y;
	rx[1][0] = r.z, rx[1][1] = 0.0f, rx[1][2] = -r.x;
	rx[2][0] = -r.y, rx[2][1] = r.x, rx[2][2] = 0.0f;

	// [ Kc, -Kc Rx ; Rx Kc, -Rx Kc Rx + Kr ]
	float kcr[3][3], rkc[3][3];
	for ( int a = 0; a < 3; ++a )
	{
		for ( int b = 0; b < 3; ++b )
		{
			kcr[a][b] = kc[a][0] * rx[0][b] + kc[a][1] * rx[1][b] + kc[a][2] * rx[2][b];
			rkc[a][b] = rx[a][0] * kc[0][b] + rx[a][1] * kc[1][b] + rx[a][2] * kc[2][b];
		}
	}
	for ( int a = 0; a < 3; ++a )
	{
		for ( int b = 0; b < 3; ++b )
		{
			float rkcr = rkc[a][0] * rx[0][b] + rkc[a][1] * rx[1][b] + rkc[a][2] * rx[2][b];
			block->m[a][b] += kc[a][b];
			block->m[a][3 + b] -= kcr[a][b];
			block->m[3 + a][b] += rkc[a][b];
			block->m[3 + a][3 + b] += kr[a][b] - rkcr;
		}
	}
}

// In-place Cholesky (lower triangle) of a node block, in double. A pivot that is not positive is replaced by the
// diagonal, which keeps the preconditioner symmetric positive definite.
static void lpFactorBlock( lpBlock6* block )
{
	double l[6][6] = { { 0.0 } };
	for ( int j = 0; j < 6; ++j )
	{
		double sum = block->m[j][j];
		for ( int k = 0; k < j; ++k )
		{
			sum -= l[j][k] * l[j][k];
		}
		double pivot = sum > 1e-12 * (double)block->m[j][j] && sum > 0.0 ? sum : ( block->m[j][j] > 0.0f ? block->m[j][j] : 1.0 );
		l[j][j] = sqrt( pivot );
		for ( int i = j + 1; i < 6; ++i )
		{
			double s = block->m[i][j];
			for ( int k = 0; k < j; ++k )
			{
				s -= l[i][k] * l[j][k];
			}
			l[i][j] = s / l[j][j];
		}
	}
	for ( int i = 0; i < 6; ++i )
	{
		for ( int j = 0; j < 6; ++j )
		{
			block->m[i][j] = j <= i ? (float)l[i][j] : 0.0f;
		}
	}
}

// z = (L L^T)^-1 r
static lpVec6 lpPrecondition( lpVec6 r, const lpBlock6* block )
{
	float v[6] = { r.f.x, r.f.y, r.f.z, r.t.x, r.t.y, r.t.z };
	for ( int i = 0; i < 6; ++i )
	{
		float s = v[i];
		for ( int k = 0; k < i; ++k )
		{
			s -= block->m[i][k] * v[k];
		}
		v[i] = s / block->m[i][i];
	}
	for ( int i = 5; i >= 0; --i )
	{
		float s = v[i];
		for ( int k = i + 1; k < 6; ++k )
		{
			s -= block->m[k][i] * v[k];
		}
		v[i] = s / block->m[i][i];
	}
	lpVec6 z = { { v[0], v[1], v[2] }, { v[3], v[4], v[5] } };
	return z;
}

// What rests on a structure: dynamic bodies pressing on its pieces, from the last physics step's contact impulses
// (rubble on a floor, a stone on a plank, a cart on a bridge), into each piece's stressLoad. Sampled when a solve
// starts and kept, so the solve can continue across steps while the contacts jitter. Returns how much the loads
// changed, relative to the structure's own weight.
static float lpSampleLoads( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	b3WorldTransform xf = b3Body_GetTransform( body->id );
	int n = body->pieces.count;
	lpArray_Reserve( w->scratchLoads, n );
	float weight = 0.0f;
	float g = b3Length( b3World_GetGravity( w->def.physics ) );
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

	float change = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		const lpPiece* p = w->pieces.data + body->pieces.data[i];
		change += b3Length( b3Sub( p->stressLoad.f, w->scratchLoads.data[i].f ) );
	}
	return weight > 0.0f ? change / weight : 0.0f;
}

// Nodes and edges of a structure's system, counted the way lpStressBuild lays them out, so its budget is known before
// anything is built
static void lpStressCount( const lpWorld* w, const lpBody* body, int* nodes, int* edges )
{
	*nodes = 0;
	*edges = 0;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		const lpPiece* p = w->pieces.data + pi;
		*nodes += p->anchored ? 0 : 1;
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			const lpBond* bond = w->bonds.data + p->bonds.data[k];
			*edges += bond->a == pi && ( p->anchored == false || w->pieces.data[bond->b].anchored == false ) ? 1 : 0;
		}
	}
}

// Compact system for one structure: non-anchored pieces become nodes, bonds become edges; loads, preconditioner and
// warm start. Runs inside a parallel job: it writes only the job and its own pieces' solve slots.
static void lpStressBuild( lpWorld* w, lpStressJob* job )
{
	const lpBody* body = w->bodies.data + job->body;
	b3Vec3 g = job->gravity;

	int n = 0;
	job->nodes.count = 0;
	float heaviest = 0.0f;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		lpPiece* p = w->pieces.data + pi;
		p->solveSlot = -1;
		if ( p->anchored == false )
		{
			p->solveSlot = n++;
			lpArray_Push( job->nodes, pi );
			float weight = p->shape->volume * lpGetMaterial( p->material )->density;
			heaviest = weight > heaviest ? weight : heaviest;
		}
	}
	LP_ASSERT( n == job->nodeCount );
	float scale = heaviest * b3Length( g );
	scale = scale > 0.0f ? scale : 1.0f;
	job->forceScale = scale;

	job->edges.count = 0;
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
			lpArray_Push( job->edges, e );
		}
	}
	LP_ASSERT( job->edges.count == job->edgeCount );

	// Vectors: x (solution), f (load), r, z, p, q; one factored block per node
	lpArray_Reserve( job->vectors, 6 * n );
	job->vectors.count = 6 * n;
	lpArray_Reserve( job->blocks, n );
	job->blocks.count = n;
	lpVec6* x = job->vectors.data;
	lpVec6* f = x + n;
	lpBlock6* blocks = job->blocks.data;
	memset( blocks, 0, sizeof( lpBlock6 ) * (size_t)n );
	for ( int i = 0; i < n; ++i )
	{
		const lpPiece* p = w->pieces.data + job->nodes.data[i];
		float mass = p->shape->volume * lpGetMaterial( p->material )->density;
		x[i].f = b3MulSV( 1.0f / scale, p->stressX.f );
		x[i].t = b3MulSV( 1.0f / scale, p->stressX.t );
		f[i].f = b3MulSV( mass / scale, g );
		f[i].t = b3Vec3_zero;
	}

	for ( int i = 0; i < n; ++i )
	{
		const lpPiece* p = w->pieces.data + job->nodes.data[i];
		f[i].f = b3MulAdd( f[i].f, 1.0f / scale, p->stressLoad.f );
		f[i].t = b3MulAdd( f[i].t, 1.0f / scale, p->stressLoad.t );
	}
	for ( int k = 0; k < job->edges.count; ++k )
	{
		const lpStressEdge* e = job->edges.data + k;
		if ( e->a >= 0 )
		{
			lpAddBlock( blocks + e->a, e, e->ra );
		}
		if ( e->b >= 0 )
		{
			lpAddBlock( blocks + e->b, e, e->rb );
		}
	}
	for ( int i = 0; i < n; ++i )
	{
		lpFactorBlock( blocks + i );
	}

	// A solve in progress continues from where the last step left it
	if ( job->continuing )
	{
		lpVec6* r = x + 2 * n;
		lpVec6* p = x + 4 * n;
		for ( int i = 0; i < n; ++i )
		{
			const lpPiece* piece = w->pieces.data + job->nodes.data[i];
			r[i] = piece->stressR;
			p[i] = piece->stressP;
		}
	}
}

// Preconditioned conjugate gradient, at most job->budget iterations. A fresh solve starts from the warm start x; a
// continued one picks up r, p and rz where the last step left them, so a big structure's iterations add up to one
// solve spread over several steps instead of restarting every step.
static void lpStressSolve( lpStressJob* job )
{
	int n = job->nodes.count;
	const lpStressEdge* edges = job->edges.data;
	int edgeCount = job->edges.count;
	lpVec6* x = job->vectors.data;
	lpVec6* f = x + n;
	lpVec6* r = x + 2 * n;
	lpVec6* z = x + 3 * n;
	lpVec6* p = x + 4 * n;
	lpVec6* q = x + 5 * n;
	const lpBlock6* d = job->blocks.data;

	double rz = job->rz;
	if ( job->continuing == false )
	{
		lpStressApply( edges, edgeCount, x, q, n );
		for ( int i = 0; i < n; ++i )
		{
			r[i].f = b3Sub( f[i].f, q[i].f );
			r[i].t = b3Sub( f[i].t, q[i].t );
			z[i] = lpPrecondition( r[i], d + i );
			p[i] = z[i];
		}
		rz = lpDot6( r, z, n );
	}
	double limit = job->tolerance * job->tolerance * lpDot6( f, f, n );
	bool converged = false;
	int it = 0;
	for ( ; it < job->budget; ++it )
	{
		if ( lpDot6( r, r, n ) <= limit )
		{
			converged = true;
			break;
		}
		lpStressApply( edges, edgeCount, p, q, n );
		double pq = lpDot6( p, q, n );
		if ( ( pq > 0.0 ) == false )
		{
			break;
		}
		float alpha = (float)( rz / pq );
		for ( int i = 0; i < n; ++i )
		{
			x[i].f = b3MulAdd( x[i].f, alpha, p[i].f );
			x[i].t = b3MulAdd( x[i].t, alpha, p[i].t );
			r[i].f = b3MulSub( r[i].f, alpha, q[i].f );
			r[i].t = b3MulSub( r[i].t, alpha, q[i].t );
			z[i] = lpPrecondition( r[i], d + i );
		}
		double rz2 = lpDot6( r, z, n );
		float beta = (float)( rz2 / rz );
		rz = rz2;
		for ( int i = 0; i < n; ++i )
		{
			p[i].f = b3MulAdd( z[i].f, beta, p[i].f );
			p[i].t = b3MulAdd( z[i].t, beta, p[i].t );
		}
	}
	if ( converged == false && lpDot6( r, r, n ) <= limit )
	{
		converged = true;
	}
	job->rz = rz;
	job->iterations = it;
	job->converged = converged;
}

// Phase 2, one structure: build, solve, and keep the solution (in newtons of load) and the solve's state on the pieces
static void lpRunStressJob( int index, void* context )
{
	lpWorld* w = context;
	lpStressJob* job = w->stressJobs + index;
	lpStressBuild( w, job );
	lpStressSolve( job );
	int n = job->nodes.count;
	const lpVec6* x = job->vectors.data;
	const lpVec6* r = x + 2 * n;
	const lpVec6* p = x + 4 * n;
	for ( int i = 0; i < n; ++i )
	{
		lpPiece* piece = w->pieces.data + job->nodes.data[i];
		piece->stressX.f = b3MulSV( job->forceScale, x[i].f );
		piece->stressX.t = b3MulSV( job->forceScale, x[i].t );
		piece->stressR = r[i];
		piece->stressP = p[i];
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

// Stresses at every bond from the solution; strain for the overloaded ones, and the worst of those break.
// Writes the peak utilization. Returns the number of broken bonds.
static int lpStressEvaluate( lpWorld* w, const lpStressJob* job, float* peak, int* strained )
{
	const lpVec6* x = job->vectors.data;
	w->scratchOverloads.count = 0;
	*peak = 0.0f;
	*strained = 0;
	for ( int k = 0; k < job->edges.count; ++k )
	{
		const lpStressEdge* e = job->edges.data + k;
		lpBond* bond = w->bonds.data + e->bond;
		b3Vec3 force, moment;
		lpEdgeForce( e, x, &force, &moment );
		force = b3MulSV( job->forceScale, force );
		moment = b3MulSV( job->forceScale, moment );

		float area = bond->area;
		float axialForce = b3Dot( force, e->n ); // tension positive
		float compression = b3MaxFloat( -axialForce, 0.0f );
		float m1 = b3AbsFloat( b3Dot( moment, e->t1 ) ); // bending about t1: the fibres at +-h2 carry it
		float m2 = b3AbsFloat( b3Dot( moment, e->t2 ) );
		b3Vec3 shearForce = b3MulSub( force, axialForce, e->n );
		float hMax = b3MaxFloat( bond->h1, bond->h2 );
		float shear = b3Length( shearForce ) / area +
					  3.0f * b3AbsFloat( b3Dot( moment, e->n ) ) * hMax / ( area * ( bond->h1 * bond->h1 + bond->h2 * bond->h2 ) );

		float tensionLimit, compressionLimit, shearLimit, mu;
		lpBondLimits( w, bond, &tensionLimit, &compressionLimit, &shearLimit, &mu );
		float s = w->def.stressScale * b3MaxFloat( bond->health, 0.0f ) / bond->strength;
		float tensionCap = b3MaxFloat( s * tensionLimit, 1e4f );

		// Pulling apart and bending: the section's elastic tension capacity, plus rocking. A compressed joint holds a
		// moment until its resultant reaches the edge of the patch, so masonry tips over an edge instead of cracking
		// as soon as the load leaves the middle third.
		float cap1 = tensionCap * area * bond->h2 / 3.0f + compression * bond->h2;
		float cap2 = tensionCap * area * bond->h1 / 3.0f + compression * bond->h1;
		float rho = b3MaxFloat( axialForce, 0.0f ) / ( tensionCap * area ) + m1 / cap1 + m2 / cap2;

		// Crushing at the compressed edge, and Coulomb shear
		float edge = compression / area + 3.0f * m1 / ( area * bond->h2 ) + 3.0f * m2 / ( area * bond->h1 );
		rho = b3MaxFloat( rho, edge / b3MaxFloat( s * compressionLimit, 1e4f ) );
		rho = b3MaxFloat( rho, shear / ( s * ( shearLimit + mu * compression / area ) + 1e3f ) );

		*peak = rho > *peak ? rho : *peak;
		lpApplyStrain( w, job->xf, e->bond, rho, strained );
	}
	return lpBreakOverloads( w, job->xf );
}

// A structure that converged and is only creaking (joints over their limit, nothing broken yet) does not need its
// system rebuilt or solved again: the utilizations are unchanged, so a check just adds strain from the stored ones.
static int lpStressCreak( lpWorld* w, int bodyIndex, int* strained )
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
			if ( bond->a == pi && bond->rho > 1.0f )
			{
				lpApplyStrain( w, xf, bi, bond->rho, strained );
			}
		}
	}
	return lpBreakOverloads( w, xf );
}

// Long pieces (beams, planks, columns, lintels) are rigid nodes, so the solve cannot bend them. From the solved bond
// forces and the piece's own weight, find the bending moment along its axis at a few cuts between its supports; where
// the section is overloaded, strain builds, and at 1 a small synthetic impact there snaps it through the normal
// fracture pipeline next step. Returns the number of pieces queued to break.
static int lpStressPieces( lpWorld* w, const lpStressJob* job, int* strained, int* slender )
{
	const lpVec6* x = job->vectors.data;
	b3WorldTransform xf = job->xf;
	b3Vec3 g = job->gravity;
	float forceScale = job->forceScale;
	int queued = 0;
	for ( int i = 0; i < job->nodes.count; ++i )
	{
		int pi = job->nodes.data[i];
		lpPiece* p = w->pieces.data + pi;
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
			float s = b3Dot( d, a );
			lo = s < lo ? s : lo;
			hi = s > hi ? s : hi;
			w1 = b3MaxFloat( w1, b3AbsFloat( b3Dot( d, t1 ) ) );
			w2 = b3MaxFloat( w2, b3AbsFloat( b3Dot( d, t2 ) ) );
		}
		float length = hi - lo;
		if ( length < 2.5f * 2.0f * b3MaxFloat( w1, w2 ) || length < 4.0f * fragment )
		{
			continue; // not slender: the joints decide
		}

		// Loads on the piece: each bond's force at its contact, and the moment it carries
		int slot = p->solveSlot;
		float bondLo = FLT_MAX, bondHi = -FLT_MAX;
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			float s = b3Dot( b3Sub( w->bonds.data[p->bonds.data[k]].centroid, c ), a );
			bondLo = s < bondLo ? s : bondLo;
			bondHi = s > bondHi ? s : bondHi;
		}
		if ( bondHi - bondLo < 0.25f * length )
		{
			continue; // held at one place only: a cantilever off one joint, which the joint check covers
		}

		float area = p->shape->volume / length;
		float limit = w->def.stressScale * m->tensileStrength;
		float mass = p->shape->volume * m->density;
		float worst = 0.0f, worstAt = 0.0f;
		for ( int sample = 1; sample <= 9; ++sample )
		{
			float cut = bondLo + ( bondHi - bondLo ) * (float)sample / 10.0f;
			b3Vec3 q = b3MulAdd( c, cut, a );
			b3Vec3 forceSum = b3Vec3_zero, momentSum = b3Vec3_zero;
			for ( int k = 0; k < job->edges.count; ++k )
			{
				const lpStressEdge* e = job->edges.data + k;
				if ( e->a != slot && e->b != slot )
				{
					continue;
				}
				const lpBond* bond = w->bonds.data + e->bond;
				if ( b3Dot( b3Sub( bond->centroid, c ), a ) <= cut )
				{
					continue;
				}
				b3Vec3 f, mo;
				lpEdgeForce( e, x, &f, &mo );
				float sign = e->a == slot ? forceScale : -forceScale; // what the bond does to this piece
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
		if ( w->def.debugLog )
		{
			printf( "[lpf]   slender piece %d: length %.2f, worst utilization %.2f at %.2f, strain %.2f\n", pi, (double)length,
					(double)worst, (double)worstAt, (double)p->strain );
		}
		if ( worst <= 1.0f )
		{
			continue;
		}
		*strained += 1;
		*slender += 1;
		p->strain += ( worst - 1.0f ) * w->def.strainRate;
		if ( p->strain < 1.0f )
		{
			continue;
		}

		// Snap it where it is weakest: a blow sized to the section, through the normal fracture pipeline
		p->strain = 0.0f;
		float depth = 2.0f * b3MaxFloat( w1, w2 );
		lpImpactDef impact = { 0 };
		impact.point = b3TransformWorldPoint( xf, b3MulAdd( c, worstAt, a ) );
		impact.direction = b3RotateVector( xf.q, a );
		impact.radius = 1.5f * depth;
		impact.energy = 4.0f * b3MaxFloat( m->bondStrength, m->fractureEnergy ) * B3_PI * impact.radius * impact.radius;
		w->impactSerial += 1;
		lpDeferredJob snap = { pi, p->generation, w->impactSerial, impact, true };
		lpArray_Push( w->deferred, snap );
		lpStressDust( w, xf, w->bonds.data + p->bonds.data[0], p->bonds.data[0], 6 );
		queued += 1;
	}
	return queued;
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

// Phase 3, one structure: judge its solution. Converged: stresses at every joint, strain and breaks, then the slender
// pieces. Not converged: it keeps solving next step, and nothing is judged on an unconverged solution.
static void lpStressJudge( lpWorld* w, const lpStressJob* job )
{
	lpBody* body = w->bodies.data + job->body;
	int n = job->nodes.count;
	int edges = job->edges.count;
	w->stressWork += ( job->iterations + 2 ) * edges;
	w->stats.stressIterations += job->iterations;
	w->stats.stressSolves += 1;
	body->solving = job->converged == false;
	body->solveRz = job->rz;
	body->solveTopology = body->topology;
	body->solveNodes = n;
	body->solveEdges = edges;

	if ( job->converged == false )
	{
		body->stressSteps += 1;
		body->unsettled = true;
		for ( int k = 0; k < edges; ++k )
		{
			int bi = job->edges.data[k].bond;
			const lpBond* bond = w->bonds.data + bi;
			if ( bond->strain > 0.0f && ( w->tick + (uint64_t)bi ) % 4 == 0 )
			{
				lpStressDust( w, job->xf, bond, bi, 1 ); // joints that were straining keep creaking while it solves
			}
		}
		lpArray_Push( w->stressAgain, job->body );
		return;
	}

	float peak = 0.0f;
	int strained = 0;
	int broken = lpStressEvaluate( w, job, &peak, &strained );
	int slender = 0;
	int snapped = broken == 0 ? lpStressPieces( w, job, &strained, &slender ) : 0; // once the joints hold
	body->unsettled = broken > 0 || snapped > 0 || strained > 0;
	body->creaking = broken == 0 && snapped == 0 && slender == 0 && strained > 0; // next checks only add strain
	body->strainedLastCheck = strained > 0;
	w->stats.stressBreaks += broken;
	if ( w->def.debugLog )
	{
		printf( "[lpf] tick %llu stress: body %d, %d nodes, %d bonds, %d iterations over %d steps, peak utilization %.2f, "
				"%d strained, %d broke\n",
				(unsigned long long)w->tick, job->body, n, edges, job->iterations, body->stressSteps + 1, (double)peak, strained,
				broken );
	}
	body->stressSteps = 0;
	if ( body->unsettled )
	{
		lpArray_Push( w->stressAgain, job->body ); // broken joints split it next step; strained ones creak on
	}
}

void lpCheckStructures( lpWorld* w )
{
	if ( w->stressQueue.count == 0 )
	{
		return;
	}
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
			continue;
		}

		// Converged and only creaking: the stored utilizations add strain, with no solve
		if ( body->creaking && body->solving == false && body->solveTopology == body->topology )
		{
			int creaks = 0;
			int broke = lpStressCreak( w, bodyIndex, &creaks );
			body->creaking = broke == 0 && creaks > 0;
			body->unsettled = broke > 0 || creaks > 0;
			w->stats.stressBreaks += broke;
			if ( body->unsettled )
			{
				lpArray_Push( w->stressAgain, bodyIndex );
			}
			continue;
		}
		body->creaking = false;

		int nodes, edges;
		lpStressCount( w, body, &nodes, &edges );
		if ( nodes == 0 || edges == 0 )
		{
			body->solving = false;
			continue;
		}

		// Its share of the budget, reserved whole (the build, the first residual and the iterations), before anything
		// is built. The first structure of a step always gets an iteration, so even one bigger than the budget makes
		// progress. One that does not fit waits, having cost nothing, and goes first next step.
		int room = b3MinInt( w->def.maxStressStructureWork, w->def.maxStressWork - reserved ) / edges - 2;
		int budget = b3MinInt( room, w->def.maxStressIterations );
		budget = budget < 1 && reserved == 0 ? 1 : budget;
		if ( budget < 1 )
		{
			body->unsettled = true;
			w->stats.stressWaiting += 1;
			lpArray_Push( w->stressAgain, bodyIndex );
			continue;
		}

		bool fresh = body->solving == false || body->solveTopology != body->topology;
		if ( fresh )
		{
			// Asked again with the structure unchanged since it settled: only something landing on it or leaving it can
			// matter. If what rests on it barely changed, it stays settled without a solve.
			bool loadOnly = body->solving == false && body->solveTopology == body->topology && body->strainedLastCheck == false;
			float change = lpSampleLoads( w, bodyIndex );
			if ( loadOnly && change < 0.02f )
			{
				continue;
			}
		}

		// Continue the solve in progress if the structure has not changed since the last step
		bool continuing = fresh == false && body->solveNodes == nodes && body->solveEdges == edges;
		if ( continuing == false )
		{
			body->stressSteps = 0;
		}
		reserved += ( budget + 2 ) * edges;
		lpStressJob* job = lpAddStressJob( w );
		job->body = bodyIndex;
		job->xf = b3Body_GetTransform( body->id );
		job->gravity = b3InvRotateVector( job->xf.q, gravity );
		job->nodeCount = nodes;
		job->edgeCount = edges;
		job->budget = budget;
		job->continuing = continuing;
		job->tolerance = body->stressSteps < w->def.stressPatience ? 1e-3 : 1e-2;
		job->rz = body->solveRz;
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
