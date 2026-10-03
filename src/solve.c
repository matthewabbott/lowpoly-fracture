// SPDX-License-Identifier: MIT
// The stress system's math: the beam kernel, K x, the block-Jacobi preconditioner and conjugate gradient (solve.h).

#include "solve.h"

#include <math.h>

static const lpVec6 lp_vec6Zero = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };

void lpEdgeForce( const lpStressEdge* e, const lpVec6* x, lpVec3* force, lpVec3* moment )
{
	lpVec6 va = e->a >= 0 ? x[e->a] : lp_vec6Zero;
	lpVec6 vb = e->b >= 0 ? x[e->b] : lp_vec6Zero;
	lpVec3 delta = lpSub( lpAdd( vb.f, lpCross( vb.t, e->rb ) ), lpAdd( va.f, lpCross( va.t, e->ra ) ) );
	lpVec3 phi = lpSub( vb.t, va.t );
	float dn = lpDot( delta, e->n );
	*force = lpAdd( lpMulSV( e->kn * dn, e->n ), lpMulSV( e->ks, lpMulSub( delta, dn, e->n ) ) );
	lpVec3 m = lpMulSV( e->kt * lpDot( phi, e->n ), e->n );
	m = lpMulAdd( m, e->kb1 * lpDot( phi, e->t1 ), e->t1 );
	*moment = lpMulAdd( m, e->kb2 * lpDot( phi, e->t2 ), e->t2 );
}

static void lpApply( const lpStressEdge* edges, int edgeCount, const lpVec6* x, lpVec6* y, int nodeCount )
{
	for ( int i = 0; i < nodeCount; ++i )
	{
		y[i] = lp_vec6Zero;
	}
	for ( int k = 0; k < edgeCount; ++k )
	{
		const lpStressEdge* e = edges + k;
		lpVec3 force, moment;
		lpEdgeForce( e, x, &force, &moment );
		if ( e->a >= 0 )
		{
			y[e->a].f = lpSub( y[e->a].f, force );
			y[e->a].t = lpSub( y[e->a].t, lpAdd( lpCross( e->ra, force ), moment ) );
		}
		if ( e->b >= 0 )
		{
			y[e->b].f = lpAdd( y[e->b].f, force );
			y[e->b].t = lpAdd( y[e->b].t, lpAdd( lpCross( e->rb, force ), moment ) );
		}
	}
}

void lpSystemApply( const lpStressSystem* s, const lpVec6* x, lpVec6* y )
{
	lpApply( s->edges.data, s->edges.count, x, y, s->nodes.count );
}

static double lpDot6( const lpVec6* a, const lpVec6* b, int n )
{
	double sum = 0.0;
	for ( int i = 0; i < n; ++i )
	{
		sum += (double)lpDot( a[i].f, b[i].f ) + (double)lpDot( a[i].t, b[i].t );
	}
	return sum;
}

// An edge's contribution to the block of the node it meets at arm r. The coupling between a node's translation and
// rotation (bonds far from its reference point) is exactly what a plain diagonal preconditioner misses.
static void lpAddBlock( lpBlock6* block, const lpStressEdge* e, lpVec3 r )
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

void lpSystemResize( lpStressSystem* s )
{
	int n = s->nodes.count;
	lpArray_Reserve( s->vectors, 6 * n );
	s->vectors.count = 6 * n;
	// Cleared: a region solve never writes the solver's vectors past its region, and they are state (hashed while it
	// solves), so what was in the memory before must not show
	memset( s->vectors.data, 0, sizeof( lpVec6 ) * (size_t)( 6 * n ) );
	s->factored = false;
}

void lpSystemFactor( lpStressSystem* s )
{
	int n = s->nodes.count;
	lpArray_Reserve( s->blocks, n ); // only a factored system has blocks (a fine one solved through a reduced one has none)
	s->blocks.count = n;
	lpBlock6* blocks = s->blocks.data;
	memset( blocks, 0, sizeof( lpBlock6 ) * (size_t)n );
	for ( int k = 0; k < s->edges.count; ++k )
	{
		const lpStressEdge* e = s->edges.data + k;
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
	s->factored = true;
}

void lpSystemIncidence( lpStressSystem* s )
{
	// By counting sort, so each node's list is in edge order
	int n = s->nodes.count;
	lpArray_Reserve( s->incidentStart, n + 1 );
	s->incidentStart.count = n + 1;
	int* start = s->incidentStart.data;
	memset( start, 0, sizeof( int ) * (size_t)( n + 1 ) );
	for ( int k = 0; k < s->edges.count; ++k )
	{
		const lpStressEdge* e = s->edges.data + k;
		start[e->a + 1] += e->a >= 0 ? 1 : 0;
		start[e->b + 1] += e->b >= 0 ? 1 : 0;
	}
	for ( int i = 0; i < n; ++i )
	{
		start[i + 1] += start[i];
	}
	lpArray_Reserve( s->incident, start[n] );
	s->incident.count = start[n];
	for ( int k = 0; k < s->edges.count; ++k )
	{
		const lpStressEdge* e = s->edges.data + k;
		if ( e->a >= 0 )
		{
			s->incident.data[start[e->a]++] = k;
		}
		if ( e->b >= 0 )
		{
			s->incident.data[start[e->b]++] = k;
		}
	}
	for ( int i = n; i > 0; --i )
	{
		start[i] = start[i - 1];
	}
	start[0] = 0;
}

void lpSystemNodeScales( lpStressSystem* s, float floor )
{
	int n = s->nodes.count;
	const lpVec6* x = s->vectors.data;
	const lpVec6* f = x + n;
	lpArray_Reserve( s->nodeScale, n );
	s->nodeScale.count = n;
	float* scale = s->nodeScale.data;
	for ( int i = 0; i < n; ++i )
	{
		scale[i] = floor + lpLength( f[i].f );
	}
	for ( int k = 0; k < s->edges.count; ++k )
	{
		const lpStressEdge* e = s->edges.data + k;
		lpVec3 force, moment;
		lpEdgeForce( e, x, &force, &moment );
		float magnitude = lpLength( force );
		if ( e->a >= 0 )
		{
			scale[e->a] += magnitude;
		}
		if ( e->b >= 0 )
		{
			scale[e->b] += magnitude;
		}
	}
}

// Every node's residual force and torque within nodeTolerance of its scale
static bool lpNodesBalanced( const lpStressSystem* s, const lpVec6* r, float nodeTolerance )
{
	int n = s->nodes.count;
	if ( s->nodeScale.count != n || s->nodeArm.count != n )
	{
		return true;
	}
	const float* scale = s->nodeScale.data;
	const float* arm = s->nodeArm.data;
	for ( int i = 0; i < n; ++i )
	{
		float limit = nodeTolerance * scale[i];
		float torque = limit * arm[i];
		if ( lpLengthSquared( r[i].f ) > limit * limit || lpLengthSquared( r[i].t ) > torque * torque )
		{
			return false;
		}
	}
	return true;
}

void lpSystemSolve( lpStressSystem* s, int budget, double tolerance, float nodeTolerance, bool continuing, lpSolveState* state )
{
	int n = s->nodes.count;
	const lpStressEdge* edges = s->edges.data;
	int edgeCount = s->edges.count;
	lpVec6* x = s->vectors.data;
	lpVec6* f = x + n;
	lpVec6* r = x + 2 * n;
	lpVec6* z = x + 3 * n;
	lpVec6* p = x + 4 * n;
	lpVec6* q = x + 5 * n;
	const lpBlock6* d = s->blocks.data;

	double rz = state->rz;
	if ( continuing == false )
	{
		lpApply( edges, edgeCount, x, q, n );
		for ( int i = 0; i < n; ++i )
		{
			r[i].f = lpSub( f[i].f, q[i].f );
			r[i].t = lpSub( f[i].t, q[i].t );
			z[i] = lpPrecondition( r[i], d + i );
			p[i] = z[i];
		}
		rz = lpDot6( r, z, n );
	}
	double limit = tolerance * tolerance * ( s->loadNorm2 > 0.0 ? s->loadNorm2 : lpDot6( f, f, n ) );
	bool converged = false;
	int it = 0;
	for ( ; it < budget; ++it )
	{
		if ( lpDot6( r, r, n ) <= limit && lpNodesBalanced( s, r, nodeTolerance ) )
		{
			converged = true;
			break;
		}
		lpApply( edges, edgeCount, p, q, n );
		double pq = lpDot6( p, q, n );
		if ( ( pq > 0.0 ) == false )
		{
			break;
		}
		float alpha = (float)( rz / pq );
		for ( int i = 0; i < n; ++i )
		{
			x[i].f = lpMulAdd( x[i].f, alpha, p[i].f );
			x[i].t = lpMulAdd( x[i].t, alpha, p[i].t );
			r[i].f = lpMulSub( r[i].f, alpha, q[i].f );
			r[i].t = lpMulSub( r[i].t, alpha, q[i].t );
			z[i] = lpPrecondition( r[i], d + i );
		}
		double rz2 = lpDot6( r, z, n );
		float beta = (float)( rz2 / rz );
		rz = rz2;
		for ( int i = 0; i < n; ++i )
		{
			p[i].f = lpMulAdd( z[i].f, beta, p[i].f );
			p[i].t = lpMulAdd( z[i].t, beta, p[i].t );
		}
	}
	if ( converged == false && lpDot6( r, r, n ) <= limit && lpNodesBalanced( s, r, nodeTolerance ) )
	{
		converged = true;
	}
	state->rz = rz;
	state->iterations = it;
	state->converged = converged;
}

// K x over a region's edges (those with an end in it), into its nodes only: what lies past it stays where it was (its x
// is held; its p, r, z and q are zero)
static void lpApplyActive( const lpStressSystem* s, const lpVec6* x, lpVec6* y )
{
	const int* depth = s->depth.data;
	int limit = s->depthLimit;
	const int* nodes = s->activeNodes.data;
	for ( int i = 0; i < s->activeNodes.count; ++i )
	{
		y[nodes[i]] = lp_vec6Zero;
	}
	for ( int j = 0; j < s->activeEdges.count; ++j )
	{
		const lpStressEdge* e = s->edges.data + s->activeEdges.data[j];
		lpVec3 force, moment;
		lpEdgeForce( e, x, &force, &moment );
		if ( e->a >= 0 && depth[e->a] <= limit )
		{
			y[e->a].f = lpSub( y[e->a].f, force );
			y[e->a].t = lpSub( y[e->a].t, lpAdd( lpCross( e->ra, force ), moment ) );
		}
		if ( e->b >= 0 && depth[e->b] <= limit )
		{
			y[e->b].f = lpAdd( y[e->b].f, force );
			y[e->b].t = lpAdd( y[e->b].t, lpAdd( lpCross( e->rb, force ), moment ) );
		}
	}
}

static double lpDotActive( const lpStressSystem* s, const lpVec6* a, const lpVec6* b )
{
	double sum = 0.0;
	for ( int j = 0; j < s->activeNodes.count; ++j )
	{
		int i = s->activeNodes.data[j];
		sum += (double)lpDot( a[i].f, b[i].f ) + (double)lpDot( a[i].t, b[i].t );
	}
	return sum;
}

static bool lpActiveBalanced( const lpStressSystem* s, const lpVec6* r, float nodeTolerance )
{
	if ( s->nodeScale.count != s->nodes.count || s->nodeArm.count != s->nodes.count )
	{
		return true;
	}
	for ( int j = 0; j < s->activeNodes.count; ++j )
	{
		int i = s->activeNodes.data[j];
		float limit = nodeTolerance * s->nodeScale.data[i];
		float torque = limit * s->nodeArm.data[i];
		if ( lpLengthSquared( r[i].f ) > limit * limit || lpLengthSquared( r[i].t ) > torque * torque )
		{
			return false;
		}
	}
	return true;
}

void lpSystemSolveFront( lpStressSystem* s, int limit, int budget, double tolerance, float nodeTolerance, bool continuing,
						 lpSolveState* state )
{
	int n = s->nodes.count;
	const int* depth = s->depth.data;
	lpVec6* x = s->vectors.data;
	lpVec6* f = x + n;
	lpVec6* r = x + 2 * n;
	lpVec6* z = x + 3 * n;
	lpVec6* p = x + 4 * n;
	lpVec6* q = x + 5 * n;
	const lpBlock6* d = s->blocks.data;

	// The region, the edges with an end in it, and the nodes just past it (held): found when a solve starts, kept while
	// it continues (a region that changes restarts it)
	if ( continuing == false )
	{
		s->depthLimit = limit;
		s->activeNodes.count = 0;
		lpArray_Reserve( s->activeNodes, n );
		for ( int i = 0; i < n; ++i )
		{
			if ( depth[i] <= limit )
			{
				s->activeNodes.data[s->activeNodes.count++] = i;
			}
		}
		s->activeEdges.count = 0;
		s->boundary.count = 0;
		lpArray_Reserve( s->activeEdges, s->edges.count );
		lpArray_Reserve( s->boundary, 2 * s->edges.count );
		for ( int k = 0; k < s->edges.count; ++k )
		{
			const lpStressEdge* e = s->edges.data + k;
			bool inA = e->a >= 0 && depth[e->a] <= limit;
			bool inB = e->b >= 0 && depth[e->b] <= limit;
			if ( inA || inB )
			{
				s->activeEdges.data[s->activeEdges.count++] = k;
			}
			if ( inA && e->b >= 0 && inB == false )
			{
				s->boundary.data[s->boundary.count++] = e->b;
			}
			if ( inB && e->a >= 0 && inA == false )
			{
				s->boundary.data[s->boundary.count++] = e->a;
			}
		}
		for ( int j = 0; j < s->boundary.count; ++j )
		{
			int i = s->boundary.data[j];
			r[i] = lp_vec6Zero;
			z[i] = lp_vec6Zero;
			p[i] = lp_vec6Zero;
			q[i] = lp_vec6Zero;
		}
	}
	const int* active = s->activeNodes.data;
	int count = s->activeNodes.count;
#if !defined( NDEBUG ) || defined( LP_FORCE_ASSERT )
	int inRegion = 0; // the lists kept while it continues are a cache of the depths (the hashed region): they agree
	for ( int i = 0; i < n; ++i )
	{
		inRegion += depth[i] <= limit ? 1 : 0;
	}
	LP_ASSERT( inRegion == count && s->depthLimit == limit );
#endif
	if ( count == n )
	{
		// The whole system: the same steps in the same order as a plain solve, without the lists
		lpSystemSolve( s, budget, tolerance, nodeTolerance, continuing, state );
		return;
	}

	double rz = state->rz;
	if ( continuing == false )
	{
		lpApplyActive( s, x, q );
		for ( int j = 0; j < count; ++j )
		{
			int i = active[j];
			r[i].f = lpSub( f[i].f, q[i].f );
			r[i].t = lpSub( f[i].t, q[i].t );
			z[i] = lpPrecondition( r[i], d + i );
			p[i] = z[i];
		}
		rz = lpDotActive( s, r, z );
	}
	double bound = tolerance * tolerance * ( s->loadNorm2 > 0.0 ? s->loadNorm2 : lpDotActive( s, f, f ) );
	bool converged = false;
	int it = 0;
	for ( ; it < budget; ++it )
	{
		if ( lpDotActive( s, r, r ) <= bound && lpActiveBalanced( s, r, nodeTolerance ) )
		{
			converged = true;
			break;
		}
		lpApplyActive( s, p, q );
		double pq = lpDotActive( s, p, q );
		if ( ( pq > 0.0 ) == false )
		{
			break;
		}
		float alpha = (float)( rz / pq );
		for ( int j = 0; j < count; ++j )
		{
			int i = active[j];
			x[i].f = lpMulAdd( x[i].f, alpha, p[i].f );
			x[i].t = lpMulAdd( x[i].t, alpha, p[i].t );
			r[i].f = lpMulSub( r[i].f, alpha, q[i].f );
			r[i].t = lpMulSub( r[i].t, alpha, q[i].t );
			z[i] = lpPrecondition( r[i], d + i );
		}
		double rz2 = lpDotActive( s, r, z );
		float beta = (float)( rz2 / rz );
		rz = rz2;
		for ( int j = 0; j < count; ++j )
		{
			int i = active[j];
			p[i].f = lpMulAdd( z[i].f, beta, p[i].f );
			p[i].t = lpMulAdd( z[i].t, beta, p[i].t );
		}
	}
	if ( converged == false && lpDotActive( s, r, r ) <= bound && lpActiveBalanced( s, r, nodeTolerance ) )
	{
		converged = true;
	}
	state->rz = rz;
	state->iterations = it;
	state->converged = converged;
}

void lpSystemReduce( const lpStressSystem* fine, const lpVec3* nodeRef, const lpPartition* part, lpStressSystem* reduced )
{
	int groups = part->groupCount;
	const int* group = part->group.data;
	const lpVec3* ref = part->ref.data;
	lpArray_Reserve( reduced->nodes, groups );
	reduced->nodes.count = groups;
	for ( int g = 0; g < groups; ++g )
	{
		reduced->nodes.data[g] = -1;
	}
	for ( int i = 0; i < fine->nodes.count; ++i )
	{
		int g = group[i];
		reduced->nodes.data[g] = reduced->nodes.data[g] < 0 ? fine->nodes.data[i] : reduced->nodes.data[g];
	}

	reduced->edges.count = 0;
	lpArray_Reserve( reduced->edges, fine->edges.count );
	for ( int k = 0; k < fine->edges.count; ++k )
	{
		lpStressEdge e = fine->edges.data[k];
		int ga = e.a >= 0 ? group[e.a] : -1;
		int gb = e.b >= 0 ? group[e.b] : -1;
		if ( ga == gb )
		{
			continue; // inside one rigid group
		}
		if ( e.a >= 0 && part->members.data[ga] > 1 )
		{
			e.ra = lpAdd( e.ra, lpSub( nodeRef[e.a], ref[ga] ) );
		}
		if ( e.b >= 0 && part->members.data[gb] > 1 )
		{
			e.rb = lpAdd( e.rb, lpSub( nodeRef[e.b], ref[gb] ) );
		}
		e.a = ga;
		e.b = gb;
		reduced->edges.data[reduced->edges.count++] = e;
	}
	reduced->forceScale = fine->forceScale;
	lpSystemResize( reduced ); // no incidence: only the fine system's is read (the slender-piece check)
	lpSystemFactor( reduced );
}

void lpPartitionRestrict( const lpPartition* part, const lpVec3* nodeRef, const lpVec6* fine, int nodeCount, lpVec6* reduced )
{
	for ( int g = 0; g < part->groupCount; ++g )
	{
		reduced[g] = lp_vec6Zero;
	}
	for ( int i = 0; i < nodeCount; ++i )
	{
		int g = part->group.data[i];
		lpVec3 d = lpSub( nodeRef[i], part->ref.data[g] );
		reduced[g].f = lpAdd( reduced[g].f, fine[i].f );
		reduced[g].t = lpAdd( reduced[g].t, lpAdd( fine[i].t, lpCross( d, fine[i].f ) ) );
	}
}

void lpPartitionProlong( const lpPartition* part, const lpVec3* nodeRef, const lpVec6* y, int nodeCount, lpVec6* fine )
{
	for ( int i = 0; i < nodeCount; ++i )
	{
		int g = part->group.data[i];
		lpVec3 d = lpSub( nodeRef[i], part->ref.data[g] );
		fine[i].f = lpAdd( fine[i].f, lpAdd( y[g].f, lpCross( y[g].t, d ) ) );
		fine[i].t = lpAdd( fine[i].t, y[g].t );
	}
}

void lpPartitionFree( lpPartition* part )
{
	lpArray_Free( part->group );
	lpArray_Free( part->ref );
	lpArray_Free( part->members );
	part->groupCount = 0;
}

void lpSystemFree( lpStressSystem* s )
{
	lpArray_Free( s->nodes );
	lpArray_Free( s->edges );
	lpArray_Free( s->vectors );
	lpArray_Free( s->blocks );
	lpArray_Free( s->incidentStart );
	lpArray_Free( s->incident );
	lpArray_Free( s->rho );
	lpArray_Free( s->nodeScale );
	lpArray_Free( s->nodeArm );
	lpArray_Free( s->depth );
	lpArray_Free( s->activeNodes );
	lpArray_Free( s->activeEdges );
	lpArray_Free( s->boundary );
	s->built = false;
}
