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
// are unitless. The solve is Jacobi-preconditioned conjugate gradient, warm-started from each piece's last solution
// and cut off by a per-step work budget: a structure that needs longer keeps "creaking" for a few steps. A bond over
// its limit accumulates strain on each converged check and breaks when strain reaches 1, the worst few per check, so
// failure cascades as the structure re-solves. Deterministic: everything is sequential in piece and bond order.

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

// Compact system for one structure: non-anchored pieces become nodes, bonds become edges. Returns the node count.
static int lpStressBuild( lpWorld* w, int bodyIndex, float* forceScale )
{
	lpBody* body = w->bodies.data + bodyIndex;
	b3WorldTransform xf = b3Body_GetTransform( body->id );
	b3Vec3 g = b3InvRotateVector( xf.q, b3World_GetGravity( w->def.physics ) );

	int n = 0;
	w->stressNodes.count = 0;
	float heaviest = 0.0f;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		lpPiece* p = w->pieces.data + pi;
		p->solveSlot = -1;
		if ( p->anchored == false )
		{
			p->solveSlot = n++;
			lpArray_Push( w->stressNodes, pi );
			float weight = p->shape->volume * lpGetMaterial( p->material )->density;
			heaviest = weight > heaviest ? weight : heaviest;
		}
	}
	if ( n == 0 )
	{
		return 0;
	}
	float scale = heaviest * b3Length( g );
	scale = scale > 0.0f ? scale : 1.0f;
	*forceScale = scale;

	w->stressEdges.count = 0;
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
			lpArray_Push( w->stressEdges, e );
		}
	}

	// Vectors: x (solution), f (load), r, z, p, q; one factored block per node
	lpArray_Reserve( w->stressVectors, 6 * n );
	w->stressVectors.count = 6 * n;
	lpArray_Reserve( w->stressBlocks, n );
	w->stressBlocks.count = n;
	lpVec6* x = w->stressVectors.data;
	lpVec6* f = x + n;
	lpBlock6* blocks = w->stressBlocks.data;
	memset( blocks, 0, sizeof( lpBlock6 ) * (size_t)n );
	for ( int i = 0; i < n; ++i )
	{
		const lpPiece* p = w->pieces.data + w->stressNodes.data[i];
		float mass = p->shape->volume * lpGetMaterial( p->material )->density;
		x[i].f = b3MulSV( 1.0f / scale, p->stressX.f );
		x[i].t = b3MulSV( 1.0f / scale, p->stressX.t );
		f[i].f = b3MulSV( mass / scale, g );
		f[i].t = b3Vec3_zero;
	}
	for ( int k = 0; k < w->stressEdges.count; ++k )
	{
		const lpStressEdge* e = w->stressEdges.data + k;
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
	return n;
}

// Preconditioned conjugate gradient, at most `budget` iterations. A fresh solve starts from the warm start x; a
// continued one picks up r, p and rz where the last step left them, so a big structure's iterations add up to one
// solve spread over several steps instead of restarting every step. Returns the iterations used.
static int lpStressSolve( lpWorld* w, int n, int budget, bool continuing, double* rzState, double tolerance, bool* converged )
{
	const lpStressEdge* edges = w->stressEdges.data;
	int edgeCount = w->stressEdges.count;
	lpVec6* x = w->stressVectors.data;
	lpVec6* f = x + n;
	lpVec6* r = x + 2 * n;
	lpVec6* z = x + 3 * n;
	lpVec6* p = x + 4 * n;
	lpVec6* q = x + 5 * n;
	const lpBlock6* d = w->stressBlocks.data;

	double rz = *rzState;
	if ( continuing == false )
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
	double limit = tolerance * tolerance * lpDot6( f, f, n );
	*converged = false;
	int it = 0;
	for ( ; it < budget; ++it )
	{
		if ( lpDot6( r, r, n ) <= limit )
		{
			*converged = true;
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
	if ( *converged == false && lpDot6( r, r, n ) <= limit )
	{
		*converged = true;
	}
	*rzState = rz;
	return it;
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

// Stresses at every bond from the solution; strain for the overloaded ones, and the worst of those break.
// Writes the peak utilization. Returns the number of broken bonds.
static int lpStressEvaluate( lpWorld* w, b3WorldTransform xf, float forceScale, float* peak, int* strained )
{
	const lpVec6* x = w->stressVectors.data;
	w->scratchOverloads.count = 0;
	*peak = 0.0f;
	*strained = 0;
	for ( int k = 0; k < w->stressEdges.count; ++k )
	{
		const lpStressEdge* e = w->stressEdges.data + k;
		lpBond* bond = w->bonds.data + e->bond;
		b3Vec3 force, moment;
		lpEdgeForce( e, x, &force, &moment );
		force = b3MulSV( forceScale, force );
		moment = b3MulSV( forceScale, moment );

		float area = bond->area;
		float axial = b3Dot( force, e->n ) / area; // tension positive
		float bending = 3.0f * b3AbsFloat( b3Dot( moment, e->t1 ) ) / ( area * bond->h2 ) +
						3.0f * b3AbsFloat( b3Dot( moment, e->t2 ) ) / ( area * bond->h1 );
		b3Vec3 shearForce = b3MulSub( force, b3Dot( force, e->n ), e->n );
		float hMax = b3MaxFloat( bond->h1, bond->h2 );
		float shear = b3Length( shearForce ) / area +
					  3.0f * b3AbsFloat( b3Dot( moment, e->n ) ) * hMax / ( area * ( bond->h1 * bond->h1 + bond->h2 * bond->h2 ) );

		float tensionLimit, compressionLimit, shearLimit, mu;
		lpBondLimits( w, bond, &tensionLimit, &compressionLimit, &shearLimit, &mu );
		float s = w->def.stressScale * b3MaxFloat( bond->health, 0.0f ) / bond->strength;
		float rho = 0.0f;
		float tensionSide = axial + bending;
		float compressionSide = axial - bending;
		if ( tensionSide > 0.0f )
		{
			rho = b3MaxFloat( rho, tensionSide / b3MaxFloat( s * tensionLimit, 1e4f ) );
		}
		if ( compressionSide < 0.0f )
		{
			rho = b3MaxFloat( rho, -compressionSide / b3MaxFloat( s * compressionLimit, 1e4f ) );
		}
		rho = b3MaxFloat( rho, shear / ( s * ( shearLimit + mu * b3MaxFloat( -axial, 0.0f ) ) + 1e3f ) );

		*peak = rho > *peak ? rho : *peak;
		if ( rho > 1.0f )
		{
			*strained += 1;
			if ( ( w->tick + (uint64_t)e->bond ) % 4 == 0 )
			{
				lpStressDust( w, xf, bond, e->bond, 1 );
			}
			bond->strain += ( rho - 1.0f ) * w->def.strainRate;
			if ( bond->strain >= 1.0f )
			{
				lpOverload o = { rho, e->bond };
				lpArray_Push( w->scratchOverloads, o );
			}
		}
	}

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
			break; // the rest wait for the re-solve: failure cascades a few joints at a time
		}
		lpStressDust( w, xf, w->bonds.data + o.bond, o.bond, 4 );
		lpBreakBond( w, o.bond );
		broken += 1;
	}
	return broken;
}

int lpStressStep( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	body->unsettled = false;
	if ( w->def.stressScale <= 0.0f || body->pieces.count < 2 )
	{
		body->solving = false;
		return 0;
	}

	uint64_t ticks = b3GetTicks();
	float forceScale = 1.0f;
	int n = lpStressBuild( w, bodyIndex, &forceScale );
	int edges = w->stressEdges.count;
	if ( n == 0 || edges == 0 )
	{
		body->solving = false;
		w->stats.stressMs += b3GetMilliseconds( ticks );
		return 0;
	}

	int room = ( w->def.maxStressWork - w->stressWork ) / edges;
	int budget = room < w->def.maxStressIterations ? room : w->def.maxStressIterations;
	if ( budget <= 0 )
	{
		body->unsettled = true; // another structure used this step's work; try again next step
		w->stats.stressMs += b3GetMilliseconds( ticks );
		return 0;
	}

	// Continue the solve in progress if the structure has not changed since the last step
	bool continuing = body->solving && body->solveTopology == body->topology && body->solveNodes == n && body->solveEdges == edges;
	lpVec6* x = w->stressVectors.data;
	lpVec6* r = x + 2 * n;
	lpVec6* p = x + 4 * n;
	if ( continuing )
	{
		for ( int i = 0; i < n; ++i )
		{
			const lpPiece* piece = w->pieces.data + w->stressNodes.data[i];
			r[i] = piece->stressR;
			p[i] = piece->stressP;
		}
	}
	else
	{
		body->stressSteps = 0;
	}
	double tolerance = body->stressSteps < w->def.stressPatience ? 1e-3 : 1e-2;

	bool converged = false;
	double rz = body->solveRz;
	int iterations = lpStressSolve( w, n, budget, continuing, &rz, tolerance, &converged );
	w->stressWork += ( iterations + 1 ) * edges;
	w->stats.stressIterations += iterations;

	for ( int i = 0; i < n; ++i )
	{
		lpPiece* piece = w->pieces.data + w->stressNodes.data[i];
		piece->stressX.f = b3MulSV( forceScale, x[i].f );
		piece->stressX.t = b3MulSV( forceScale, x[i].t );
		piece->stressR = r[i];
		piece->stressP = p[i];
	}
	body->solving = converged == false;
	body->solveRz = rz;
	body->solveTopology = body->topology;
	body->solveNodes = n;
	body->solveEdges = edges;

	if ( converged == false )
	{
		body->stressSteps += 1;
		body->unsettled = true; // still solving: nothing is judged on an unconverged solution
		b3WorldTransform xf = b3Body_GetTransform( body->id );
		for ( int k = 0; k < w->stressEdges.count; ++k )
		{
			int bi = w->stressEdges.data[k].bond;
			const lpBond* bond = w->bonds.data + bi;
			if ( bond->strain > 0.0f && ( w->tick + (uint64_t)bi ) % 4 == 0 )
			{
				lpStressDust( w, xf, bond, bi, 1 ); // joints that were straining keep creaking while it solves
			}
		}
		w->stats.stressMs += b3GetMilliseconds( ticks );
		return 0;
	}

	float peak = 0.0f;
	int strained = 0;
	int broken = lpStressEvaluate( w, b3Body_GetTransform( body->id ), forceScale, &peak, &strained );
	body->unsettled = broken > 0 || strained > 0;
	w->stats.stressBreaks += broken;
	if ( w->def.debugLog )
	{
		printf( "[lpf] tick %llu stress: body %d, %d nodes, %d bonds, %d iterations over %d steps, peak utilization %.2f, "
				"%d strained, %d broke\n",
				(unsigned long long)w->tick, bodyIndex, n, edges, iterations, body->stressSteps + 1, (double)peak, strained,
				broken );
	}
	body->stressSteps = 0;
	w->stats.stressMs += b3GetMilliseconds( ticks );
	return broken;
}
