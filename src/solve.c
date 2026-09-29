// SPDX-License-Identifier: MIT
// The stress system's math: the beam kernel, K x, the block-Jacobi preconditioner and conjugate gradient (solve.h).

#include "solve.h"

#include <math.h>

static const lpVec6 lp_vec6Zero = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };

void lpEdgeForce( const lpStressEdge* e, const lpVec6* x, b3Vec3* force, b3Vec3* moment )
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

static void lpApply( const lpStressEdge* edges, int edgeCount, const lpVec6* x, lpVec6* y, int nodeCount )
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

void lpSystemApply( const lpStressSystem* s, const lpVec6* x, lpVec6* y )
{
	lpApply( s->edges.data, s->edges.count, x, y, s->nodes.count );
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

// An edge's contribution to the block of the node it meets at arm r. The coupling between a node's translation and
// rotation (bonds far from its reference point) is exactly what a plain diagonal preconditioner misses.
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

void lpSystemResize( lpStressSystem* s )
{
	int n = s->nodes.count;
	lpArray_Reserve( s->vectors, 6 * n );
	s->vectors.count = 6 * n;
	lpArray_Reserve( s->blocks, n );
	s->blocks.count = n;
}

void lpSystemFactor( lpStressSystem* s )
{
	int n = s->nodes.count;
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

	// Incident edges by counting sort, so each node's list is in edge order
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

void lpSystemSolve( lpStressSystem* s, int budget, double tolerance, bool continuing, lpSolveState* state )
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
			r[i].f = b3Sub( f[i].f, q[i].f );
			r[i].t = b3Sub( f[i].t, q[i].t );
			z[i] = lpPrecondition( r[i], d + i );
			p[i] = z[i];
		}
		rz = lpDot6( r, z, n );
	}
	double limit = tolerance * tolerance * lpDot6( f, f, n );
	bool converged = false;
	int it = 0;
	for ( ; it < budget; ++it )
	{
		if ( lpDot6( r, r, n ) <= limit )
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
	state->rz = rz;
	state->iterations = it;
	state->converged = converged;
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
	s->built = false;
}
