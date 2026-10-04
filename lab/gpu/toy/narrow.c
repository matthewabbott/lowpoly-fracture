// narrow.c: the narrowphase's corpus (gate 3, NOTES.md step 3). About 10,000 hull pairs in poses that matter, built
// with the scene generator's hull code in plain double, every random draw in its own statement: box-box, box-chunk and
// chunk-chunk; faces resting (aligned like a stack, or twisted), slightly tilted, nearly parallel; edges crossing; a
// vertex on a face; generic poses; deep and shallow overlaps, separations inside, at and just outside the speculative
// distance. Each pair is quantised into the dialect and run through narrowSat and narrowClip on the twin twice: fresh
// (pass 1, no cache), then with B moved slightly and pass 1's manifolds as last tick's (pass 2: the SAT cache and the
// warm starts; the impulses are markers that name the point they came from). D writes its results (--out); F and V4
// compare theirs with D's (--ref): the axis kind and indices and the feature ids outside ties, the errors, the ties
// counted apart; and run both passes on every Vulkan GPU against their twin, word by word.
//
//   narrow_D --out FILE [--n N] [--seed S]
//   narrow_F --ref FILE [--n N] [--seed S] [--tie METRES] [--gpus MASK] [--list K] [--log PATH]   (run from lab/gpu)
//
// A tie: some decision on the pair's path was closer than the tolerance in D (NarrowDiag, Params.narrowDiag): lengths
// within --tie metres, areas within --tie x 1 m, cosines within --tie / 1 m. An axis tie (axisLen) may change the axis;
// a clip tie (clipLen, clipArea, clipCos) only the points and their ids.
#include "fpflags.h"
#include "layout.h"
#include "quant.h"
#include "rng.h"
#include "scene.h"
#include "vk_util.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( DIALECT_F )
#define DIALECT_NAME "F"
#define HAS_GPU 1
#elif defined( DIALECT_V4 )
#define DIALECT_NAME "V4"
#define HAS_GPU 1
#else
#define DIALECT_NAME "D"
#define HAS_GPU 0
#endif

#define TOY_BUFFERS 22 // kernels.slang's bindings 0..21 (the constraints and joints, 18 to 21, unused here)
void toy_bind( void* const* bufs, const size_t* counts );
const char* toy_twin_info( void );
size_t toy_sizeof( int which );
uint32_t toy_saturations( void );
void toy_reset_saturations( void );
void toy_run( int entry, uint32_t start, uint32_t count, uint32_t aux, uint32_t g0, uint32_t g1, int thread );
#define E_NSAT 6
#define E_NCLIP 7

static FILE* g_log;

static void say( const char* fmt, ... )
{
	va_list a;
	va_start( a, fmt );
	vprintf( fmt, a );
	va_end( a );
	if ( g_log )
	{
		va_start( a, fmt );
		vfprintf( g_log, fmt, a );
		va_end( a );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Double maths for the poses (quaternions x, y, z, s)
// ---------------------------------------------------------------------------------------------------------------------

static double dot3d( const double a[3], const double b[3] )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross3d( const double a[3], const double b[3], double out[3] )
{
	double x = a[1] * b[2] - a[2] * b[1];
	double y = a[2] * b[0] - a[0] * b[2];
	double z = a[0] * b[1] - a[1] * b[0];
	out[0] = x;
	out[1] = y;
	out[2] = z;
}

static void norm3d( double a[3] )
{
	double l = sqrt( dot3d( a, a ) );
	a[0] /= l;
	a[1] /= l;
	a[2] /= l;
}

static void qnorm( double q[4] )
{
	double l = sqrt( q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3] );
	for ( int i = 0; i < 4; ++i )
	{
		q[i] /= l;
	}
}

static void qmul( const double a[4], const double b[4], double out[4] )
{
	double x = a[3] * b[0] + b[3] * a[0] + a[1] * b[2] - a[2] * b[1];
	double y = a[3] * b[1] + b[3] * a[1] + a[2] * b[0] - a[0] * b[2];
	double z = a[3] * b[2] + b[3] * a[2] + a[0] * b[1] - a[1] * b[0];
	double s = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
	out[0] = x;
	out[1] = y;
	out[2] = z;
	out[3] = s;
}

// v + 2 q.v x (q.v x v + s v)
static void qrot( const double q[4], const double v[3], double out[3] )
{
	double t[3], u[3];
	cross3d( q, v, t );
	t[0] += q[3] * v[0];
	t[1] += q[3] * v[1];
	t[2] += q[3] * v[2];
	cross3d( q, t, u );
	out[0] = v[0] + 2.0 * u[0];
	out[1] = v[1] + 2.0 * u[1];
	out[2] = v[2] + 2.0 * u[2];
}

// A rotation by about 2 atan(h) about the unit axis a (no trigonometry)
static void qaxis( const double a[3], double h, double q[4] )
{
	q[0] = a[0] * h;
	q[1] = a[1] * h;
	q[2] = a[2] * h;
	q[3] = 1.0;
	qnorm( q );
}

// A unit vector perpendicular to the unit n, by rejection
static void perp( Pcg* r, const double n[3], double out[3] )
{
	for ( ;; )
	{
		double v[3];
		pcg_unit_vector( r, v );
		double d = dot3d( v, n );
		v[0] -= d * n[0];
		v[1] -= d * n[1];
		v[2] -= d * n[2];
		if ( dot3d( v, v ) > 0.01 )
		{
			norm3d( v );
			out[0] = v[0];
			out[1] = v[1];
			out[2] = v[2];
			return;
		}
	}
}

// The rotation taking the unit u to the unit v
static void qalign( Pcg* r, const double u[3], const double v[3], double q[4] )
{
	double c = dot3d( u, v );
	if ( c < -0.999999 )
	{
		double a[3];
		perp( r, u, a );
		q[0] = a[0];
		q[1] = a[1];
		q[2] = a[2];
		q[3] = 0.0;
		return;
	}
	double x[3];
	cross3d( u, v, x );
	q[0] = x[0];
	q[1] = x[1];
	q[2] = x[2];
	q[3] = 1.0 + c;
	qnorm( q );
}

static void hull_face_centroid( const SceneHull* h, int f, double c[3] )
{
	c[0] = c[1] = c[2] = 0.0;
	int e0 = h->faceEdge[f], e = e0, n = 0;
	do
	{
		const double* v = h->v[h->edge[e][2]];
		c[0] += v[0];
		c[1] += v[1];
		c[2] += v[2];
		n += 1;
		e = h->edge[e][0];
	} while ( e != e0 && n < SCENE_MAX_EDGES );
	c[0] /= n;
	c[1] /= n;
	c[2] /= n;
}

static double hull_face_radius( const SceneHull* h, int f, const double c[3] )
{
	double r = 0.0;
	int e0 = h->faceEdge[f], e = e0, n = 0;
	do
	{
		const double* v = h->v[h->edge[e][2]];
		double d[3] = { v[0] - c[0], v[1] - c[1], v[2] - c[2] };
		double l = sqrt( dot3d( d, d ) );
		r = l > r ? l : r;
		n += 1;
		e = h->edge[e][0];
	} while ( e != e0 && n < SCENE_MAX_EDGES );
	return r;
}

// ---------------------------------------------------------------------------------------------------------------------
// The corpus
// ---------------------------------------------------------------------------------------------------------------------

enum
{
	CAT_ALIGNED,
	CAT_FACE,
	CAT_TILT,
	CAT_PARALLEL,
	CAT_EDGE,
	CAT_VERTEX,
	CAT_GENERIC,
	CAT_COUNT
};
static const char* g_catNames[CAT_COUNT] = { "faces aligned (stack)", "faces twisted", "faces tilted 1e-3..0.1", "faces within 1e-4 rad",
											  "edges crossing",		   "vertex on face", "generic" };
static const int g_catWeights[CAT_COUNT] = { 10, 20, 15, 10, 20, 15, 10 };

enum
{
	BAND_DEEP,
	BAND_SHALLOW,
	BAND_INSIDE,
	BAND_SPEC,
	BAND_OUTSIDE,
	BAND_FAR,
	BAND_COUNT
};
static const char* g_bandNames[BAND_COUNT] = { "deep -0.15..-0.04", "shallow -0.02..0", "inside 0..0.02", "at 0.02 +-2e-5", "just outside 0.02..0.03",
											   "far 0.03..0.08" };
static const int g_bandWeights[BAND_COUNT] = { 15, 30, 25, 5, 15, 10 };

static const char* g_comboNames[3] = { "box-box", "box-chunk", "chunk-chunk" };

#define POOL_BIG 4
#define POOL_BOX 60
#define POOL_CHUNK 60

typedef struct Corpus
{
	int n;
	int hullCount;
	SceneHull* hulls;		  // the pool: big boxes, boxes, chunks
	SceneBody* bodies[2];	  // per pass: 2k is pair k's A, 2k + 1 its B (pass 2 moves B)
	int *category, *band, *combo;
	int chunkRedraws;
} Corpus;

static int pick( Pcg* r, const int* weights, int count )
{
	int total = 0;
	for ( int i = 0; i < count; ++i )
	{
		total += weights[i];
	}
	int x = pcg_int( r, 0, total - 1 );
	for ( int i = 0; i < count; ++i )
	{
		if ( x < weights[i] )
		{
			return i;
		}
		x -= weights[i];
	}
	return count - 1;
}

static double band_separation( Pcg* r, int band )
{
	switch ( band )
	{
		case BAND_DEEP:
			return pcg_range( r, -0.15, -0.04 );
		case BAND_SHALLOW:
			return pcg_range( r, -0.02, 0.0 );
		case BAND_INSIDE:
			return pcg_range( r, 0.0, 0.02 );
		case BAND_SPEC:
			return 0.02 + pcg_range( r, -2e-5, 2e-5 );
		case BAND_OUTSIDE:
			return pcg_range( r, 0.02, 0.03 );
		default:
			return pcg_range( r, 0.03, 0.08 );
	}
}

static void set_pose( SceneBody* b, int hull, const double p[3], const double q[4] )
{
	memset( b, 0, sizeof( *b ) );
	b->hull = hull;
	b->isStatic = 1; // no mass needed: the narrowphase reads poses and hulls only
	b->p[0] = p[0];
	b->p[1] = p[1];
	b->p[2] = p[2];
	b->q[0] = q[0];
	b->q[1] = q[1];
	b->q[2] = q[2];
	b->q[3] = q[3];
}

// B's world position so that its local point lb lands on the world point w
static void place( const double qB[4], const double lb[3], const double w[3], double pB[3] )
{
	double t[3];
	qrot( qB, lb, t );
	pB[0] = w[0] - t[0];
	pB[1] = w[1] - t[1];
	pB[2] = w[2] - t[2];
}

// An offset in the plane of the world normal n, of length up to `reach`
static void plane_offset( Pcg* r, const double n[3], double reach, double o[3] )
{
	double a[3];
	perp( r, n, a );
	double l = pcg_range( r, 0.0, reach );
	o[0] = a[0] * l;
	o[1] = a[1] * l;
	o[2] = a[2] * l;
}

static void make_pair( Corpus* c, Pcg* r, int k )
{
	int combo = pcg_int( r, 0, 2 );
	int cat = pick( r, g_catWeights, CAT_COUNT );
	int band = pick( r, g_bandWeights, BAND_COUNT );
	if ( cat == CAT_ALIGNED )
	{
		combo = 0; // boxes only
	}
	if ( cat == CAT_GENERIC )
	{
		band = BAND_COUNT; // the pose decides
	}
	// hulls: A big (a floor or a wall) for a fifth of box-box pairs; box-chunk in either order
	int ha, hb;
	int big = pcg_int( r, 0, 4 ) == 0;
	int boxA = pcg_int( r, 0, POOL_BOX - 1 );
	int boxB = pcg_int( r, 0, POOL_BOX - 1 );
	int chunkA = pcg_int( r, 0, POOL_CHUNK - 1 );
	int chunkB = pcg_int( r, 0, POOL_CHUNK - 1 );
	int bigA = pcg_int( r, 0, POOL_BIG - 1 );
	int order = pcg_int( r, 0, 1 );
	if ( combo == 0 )
	{
		ha = big ? bigA : POOL_BIG + boxA;
		hb = POOL_BIG + boxB;
	}
	else if ( combo == 1 )
	{
		ha = order ? POOL_BIG + boxA : POOL_BIG + POOL_BOX + chunkA;
		hb = order ? POOL_BIG + POOL_BOX + chunkB : POOL_BIG + boxB;
	}
	else
	{
		ha = POOL_BIG + POOL_BOX + chunkA;
		hb = POOL_BIG + POOL_BOX + chunkB;
	}
	const SceneHull* HA = c->hulls + ha;
	const SceneHull* HB = c->hulls + hb;
	double sep = band < BAND_COUNT ? band_separation( r, band ) : 0.0;

	double pA[3], qA[4], pB[3], qB[4];
	pA[0] = pcg_range( r, -2.0, 2.0 );
	pA[1] = pcg_range( r, -2.0, 2.0 );
	pA[2] = pcg_range( r, -2.0, 2.0 );
	pcg_unit_quat( r, qA );

	if ( cat == CAT_ALIGNED )
	{
		// B's axes parallel to A's, across one of A's faces: the same box straight above (a stack: every vertex on a
		// side plane), or another box offset in the plane
		int axis = pcg_int( r, 0, 2 );
		int sign = pcg_int( r, 0, 1 );
		int same = pcg_int( r, 0, 1 );
		double u = pcg_range( r, -1.0, 1.0 );
		double v = pcg_range( r, -1.0, 1.0 );
		if ( same && !big )
		{
			hb = ha;
			HB = HA;
		}
		double ln[3] = { 0.0, 0.0, 0.0 };
		ln[axis] = sign ? 1.0 : -1.0;
		double lo[3] = { 0.0, 0.0, 0.0 };
		double d = HA->boundsHalf[axis] + HB->boundsHalf[axis] + sep;
		lo[axis] = ln[axis] * d;
		if ( hb != ha )
		{
			int a1 = ( axis + 1 ) % 3, a2 = ( axis + 2 ) % 3;
			lo[a1] = u * ( HA->boundsHalf[a1] + 0.5 * HB->boundsHalf[a1] );
			lo[a2] = v * ( HA->boundsHalf[a2] + 0.5 * HB->boundsHalf[a2] );
		}
		double wo[3];
		qrot( qA, lo, wo );
		qB[0] = qA[0];
		qB[1] = qA[1];
		qB[2] = qA[2];
		qB[3] = qA[3];
		pB[0] = pA[0] + wo[0];
		pB[1] = pA[1] + wo[1];
		pB[2] = pA[2] + wo[2];
	}
	else if ( cat == CAT_FACE || cat == CAT_TILT || cat == CAT_PARALLEL || cat == CAT_VERTEX )
	{
		// a face of A (a big box: its top) and its world normal, centroid and size
		int fa = big && combo == 0 ? 2 : pcg_int( r, 0, HA->faceCount - 1 );
		double lc[3], nA[3], cA[3];
		hull_face_centroid( HA, fa, lc );
		double reach = 0.8 * hull_face_radius( HA, fa, lc );
		qrot( qA, HA->normal[fa], nA );
		qrot( qA, lc, cA );
		cA[0] += pA[0];
		cA[1] += pA[1];
		cA[2] += pA[2];
		double twist = pcg_range( r, -2.0, 2.0 );
		double o[3];
		plane_offset( r, nA, reach, o );
		double target[3] = { cA[0] + o[0] + sep * nA[0], cA[1] + o[1] + sep * nA[1], cA[2] + o[2] + sep * nA[2] };
		double minusN[3] = { -nA[0], -nA[1], -nA[2] };
		double q1[4], q2[4], lb[3];
		if ( cat == CAT_VERTEX )
		{
			// B's vertex whose normal cone holds -nA: the mean of its faces' normals turned onto -nA
			// half of them exactly the mean (a box's vertex then ties its three faces), half tipped off it
			int vb = pcg_int( r, 0, HB->vertexCount - 1 );
			int tipped = pcg_int( r, 0, 1 );
			double tip[3];
			pcg_unit_vector( r, tip );
			double cone[3] = { 0.0, 0.0, 0.0 };
			for ( int e = 0; e < HB->edgeCount; ++e )
			{
				if ( HB->edge[e][2] == vb )
				{
					const double* fn = HB->normal[HB->edge[e][3]];
					cone[0] += fn[0];
					cone[1] += fn[1];
					cone[2] += fn[2];
				}
			}
			norm3d( cone );
			if ( tipped )
			{
				cone[0] += 0.3 * tip[0];
				cone[1] += 0.3 * tip[1];
				cone[2] += 0.3 * tip[2];
				norm3d( cone );
			}
			qalign( r, cone, minusN, q1 );
			lb[0] = HB->v[vb][0];
			lb[1] = HB->v[vb][1];
			lb[2] = HB->v[vb][2];
		}
		else
		{
			int fb = pcg_int( r, 0, HB->faceCount - 1 );
			qalign( r, HB->normal[fb], minusN, q1 );
			hull_face_centroid( HB, fb, lb );
		}
		qaxis( nA, twist, q2 );
		qmul( q2, q1, qB );
		if ( cat == CAT_TILT || cat == CAT_PARALLEL )
		{
			double a[3], q3[4], t[4];
			perp( r, nA, a );
			double h = cat == CAT_TILT ? pcg_range( r, 5e-4, 0.05 ) : ldexp( 1.0, pcg_int( r, -24, -14 ) ); // 1e-7 to 1e-4 rad
			qaxis( a, h, q3 );
			qmul( q3, qB, t );
			memcpy( qB, t, sizeof( t ) );
		}
		qnorm( qB );
		place( qB, lb, target, pB );
	}
	else if ( cat == CAT_EDGE )
	{
		// an edge of A, a direction inside its arc (between its faces' normals); an edge of B turned so that a direction
		// inside its arc faces the other way, twisted about the axis, its middle across A's edge at sep along the axis
		int ea = 2 * pcg_int( r, 0, HA->edgeCount / 2 - 1 );
		int eb = 2 * pcg_int( r, 0, HB->edgeCount / 2 - 1 );
		double ta = pcg_range( r, 0.15, 0.85 );
		double tb = pcg_range( r, 0.15, 0.85 );
		double alpha = pcg_range( r, -0.3, 0.3 );
		double twist = pcg_range( r, -2.0, 2.0 );
		const double* ua = HA->normal[HA->edge[ea][3]];
		const double* va = HA->normal[HA->edge[ea + 1][3]];
		double ln[3] = { ( 1.0 - ta ) * ua[0] + ta * va[0], ( 1.0 - ta ) * ua[1] + ta * va[1], ( 1.0 - ta ) * ua[2] + ta * va[2] };
		norm3d( ln );
		double nA[3];
		qrot( qA, ln, nA );
		const double* a0 = HA->v[HA->edge[ea][2]];
		const double* a1 = HA->v[HA->edge[ea + 1][2]];
		double lm[3] = { 0.5 * ( a0[0] + a1[0] ) + alpha * ( a1[0] - a0[0] ), 0.5 * ( a0[1] + a1[1] ) + alpha * ( a1[1] - a0[1] ),
						 0.5 * ( a0[2] + a1[2] ) + alpha * ( a1[2] - a0[2] ) };
		double wm[3];
		qrot( qA, lm, wm );
		double target[3] = { wm[0] + pA[0] + sep * nA[0], wm[1] + pA[1] + sep * nA[1], wm[2] + pA[2] + sep * nA[2] };
		const double* ub = HB->normal[HB->edge[eb][3]];
		const double* vb = HB->normal[HB->edge[eb + 1][3]];
		double lmb[3] = { ( 1.0 - tb ) * ub[0] + tb * vb[0], ( 1.0 - tb ) * ub[1] + tb * vb[1], ( 1.0 - tb ) * ub[2] + tb * vb[2] };
		norm3d( lmb );
		double minusN[3] = { -nA[0], -nA[1], -nA[2] };
		double q1[4], q2[4];
		qalign( r, lmb, minusN, q1 );
		qaxis( nA, twist, q2 );
		qmul( q2, q1, qB );
		qnorm( qB );
		const double* b0 = HB->v[HB->edge[eb][2]];
		const double* b1 = HB->v[HB->edge[eb + 1][2]];
		double lb[3] = { 0.5 * ( b0[0] + b1[0] ), 0.5 * ( b0[1] + b1[1] ), 0.5 * ( b0[2] + b1[2] ) };
		place( qB, lb, target, pB );
	}
	else
	{
		// generic: any orientation, the centres apart by 0.5 to 1.05 times the sum of the extents
		double dir[3];
		pcg_unit_vector( r, dir );
		pcg_unit_quat( r, qB );
		double f = pcg_range( r, 0.5, 1.05 );
		double reachA = big ? HA->boundsHalf[1] : HA->maxExtent;
		double d = f * ( reachA + HB->maxExtent );
		if ( big )
		{
			dir[1] = dir[1] < 0.0 ? -dir[1] : dir[1];
			dir[0] *= 0.1;
			dir[2] *= 0.1;
			norm3d( dir );
		}
		pB[0] = pA[0] + d * dir[0];
		pB[1] = pA[1] + d * dir[1];
		pB[2] = pA[2] + d * dir[2];
	}
	set_pose( c->bodies[0] + 2 * k, ha, pA, qA );
	set_pose( c->bodies[0] + 2 * k + 1, hb, pB, qB );

	// pass 2: B moved by up to 0.5 mm and about 1 mrad
	double dp[3], dq[4], q2[4];
	dp[0] = pcg_range( r, -5e-4, 5e-4 );
	dp[1] = pcg_range( r, -5e-4, 5e-4 );
	dp[2] = pcg_range( r, -5e-4, 5e-4 );
	dq[0] = pcg_range( r, -5e-4, 5e-4 );
	dq[1] = pcg_range( r, -5e-4, 5e-4 );
	dq[2] = pcg_range( r, -5e-4, 5e-4 );
	dq[3] = 1.0;
	qnorm( dq );
	qmul( dq, qB, q2 );
	qnorm( q2 );
	double p2[3] = { pB[0] + dp[0], pB[1] + dp[1], pB[2] + dp[2] };
	set_pose( c->bodies[1] + 2 * k, ha, pA, qA );
	set_pose( c->bodies[1] + 2 * k + 1, hb, p2, q2 );
	c->category[k] = cat;
	c->band[k] = band;
	c->combo[k] = combo;
}

static void make_corpus( Corpus* c, int n, uint64_t seed )
{
	memset( c, 0, sizeof( *c ) );
	c->n = n;
	c->hullCount = POOL_BIG + POOL_BOX + POOL_CHUNK;
	c->hulls = (SceneHull*)calloc( (size_t)c->hullCount, sizeof( SceneHull ) );
	Pcg r = pcg_seed( seed, 31 );
	scene_box_hull( c->hulls + 0, 10.0, 0.5, 10.0 );
	scene_box_hull( c->hulls + 1, 6.0, 0.5, 6.0 );
	scene_box_hull( c->hulls + 2, 0.25, 1.25, 2.5 );
	scene_box_hull( c->hulls + 3, 2.5, 1.25, 0.25 );
	for ( int i = 0; i < POOL_BOX; ++i )
	{
		double hx = pcg_range( &r, 0.08, 0.5 );
		double hy = pcg_range( &r, 0.08, 0.5 );
		double hz = pcg_range( &r, 0.08, 0.5 );
		if ( i < 4 )
		{
			hx = hy = hz = 0.25; // stack10's cube
		}
		scene_box_hull( c->hulls + POOL_BIG + i, hx, hy, hz );
	}
	for ( int i = 0; i < POOL_CHUNK; ++i )
	{
		double hx = pcg_range( &r, 0.1, 0.3 );
		double hy = pcg_range( &r, 0.1, 0.3 );
		double hz = pcg_range( &r, 0.1, 0.3 );
		c->chunkRedraws += scene_chunk_hull( c->hulls + POOL_BIG + POOL_BOX + i, &r, hx, hy, hz );
	}
	for ( int p = 0; p < 2; ++p )
	{
		c->bodies[p] = (SceneBody*)calloc( (size_t)( 2 * n ), sizeof( SceneBody ) );
	}
	c->category = (int*)calloc( (size_t)n, sizeof( int ) );
	c->band = (int*)calloc( (size_t)n, sizeof( int ) );
	c->combo = (int*)calloc( (size_t)n, sizeof( int ) );
	for ( int k = 0; k < n; ++k )
	{
		make_pair( c, &r, k );
	}
}

static void free_corpus( Corpus* c )
{
	free( c->hulls );
	free( c->bodies[0] );
	free( c->bodies[1] );
	free( c->category );
	free( c->band );
	free( c->combo );
}

// ---------------------------------------------------------------------------------------------------------------------
// One pass on the twin
// ---------------------------------------------------------------------------------------------------------------------

typedef struct PassIO
{
	Pair* pairs;
	Manifold* prev; // last tick's (pass 2: pass 1's with markers)
	Manifold* out;
	SatAxis* sat;
	NarrowDiag* diag;
	uint32_t* lists;
} PassIO;

static void pass_alloc( PassIO* io, int n )
{
	io->pairs = (Pair*)calloc( (size_t)n, sizeof( Pair ) );
	io->prev = (Manifold*)calloc( (size_t)n, sizeof( Manifold ) );
	io->out = (Manifold*)calloc( (size_t)n, sizeof( Manifold ) );
	io->sat = (SatAxis*)calloc( (size_t)n, sizeof( SatAxis ) );
	io->diag = (NarrowDiag*)calloc( (size_t)n, sizeof( NarrowDiag ) );
	io->lists = (uint32_t*)calloc( (size_t)n, sizeof( uint32_t ) );
}

static void pass_free( PassIO* io )
{
	free( io->pairs );
	free( io->prev );
	free( io->out );
	free( io->sat );
	free( io->diag );
	free( io->lists );
}

// The marker of last tick's point i of pair k (an integer every dialect holds exactly)
static T marker( int k, int i )
{
	return (T)( 4 * k + i + 1 );
}

// Pass 2's input: pass 1's manifolds, the impulses replaced by markers
static void make_prev( const Manifold* pass1, Manifold* prev, int n )
{
	for ( int k = 0; k < n; ++k )
	{
		prev[k] = pass1[k];
		for ( int i = 0; i < MANIFOLD_POINTS; ++i )
		{
			prev[k].points[i].normalImpulse = marker( k, i );
		}
		prev[k].frictionImpulse.x = (T)( k + 1 );
	}
}

static void pass_setup( PassIO* io, int n, int pass )
{
	for ( int k = 0; k < n; ++k )
	{
		io->pairs[k].bodyA = 2 * k;
		io->pairs[k].bodyB = 2 * k + 1;
		io->pairs[k].prevIndex = pass == 0 ? -1 : k;
		io->pairs[k].colour = 0;
		io->lists[k] = (uint32_t)k;
	}
}

static void bind_all( ToyData* d, Params* P, PassIO* io, int n, BodyState* state, Aabb* aabbs, Hash2* hashes )
{
	void* bufs[TOY_BUFFERS] = { d->hulls, d->points, d->faces, d->edges, state,	 d->pose,  d->mass, aabbs,	io->lists,
								P,		  hashes,	 NULL,	   io->pairs, io->out, io->prev, io->sat, io->diag, hashes, NULL, NULL, NULL, NULL };
	size_t counts[TOY_BUFFERS] = { (size_t)d->hullCount, (size_t)d->pointCount, (size_t)d->faceCount, (size_t)d->edgeCount,
								   (size_t)d->bodyCount,
								   (size_t)d->bodyCount,
								   (size_t)d->bodyCount,
								   (size_t)d->bodyCount,
								   (size_t)n,
								   1,
								   (size_t)n,
								   0,
								   (size_t)n,
								   (size_t)n,
								   (size_t)n,
								   (size_t)n,
								   (size_t)n,
								   (size_t)n,
								   0,
								   0,
								   0,
								   0 };
	toy_bind( bufs, counts );
}

static int run_twin( ToyData* d, Params* P, PassIO* io, int n, BodyState* state, Aabb* aabbs, Hash2* hashes )
{
	bind_all( d, P, io, n, state, aabbs, hashes );
	uint32_t groups = ( (uint32_t)n + 63 ) / 64;
	int flags = 0;
	fp_clear();
	toy_run( E_NSAT, 0, (uint32_t)n, 0, 0, groups, 0 );
	flags |= fp_flags();
	fp_clear();
	toy_run( E_NCLIP, 0, (uint32_t)n, 0, 0, groups, 0 );
	flags |= fp_flags();
	return flags;
}

// ---------------------------------------------------------------------------------------------------------------------
// Results, in double, per pair and pass (D's are the reference file)
// ---------------------------------------------------------------------------------------------------------------------

typedef struct PairOut
{
	int32_t kind, type, sepA, sepB, cacheType, cacheA, cacheB, faceA, vertexB, faceB, vertexA, edgeA, edgeB;
	int32_t axisType, axisA, axisB, pointCount;
	uint32_t ids[MANIFOLD_POINTS];
	int32_t marker[MANIFOLD_POINTS]; // pass 2: the marker each point carried (0: none)
	int32_t frictionMarker;
	int32_t pad;
	double normal[3];
	double anchorA[MANIFOLD_POINTS][3];
	double anchorB[MANIFOLD_POINTS][3];
	double sep[MANIFOLD_POINTS];
	double faceASep, faceBSep, edgeSep, axisSep;
	double axisLen, clipLen, clipArea, clipCos, touchLen, supportA, supportB, supportGap;
	double faceAGap, faceBGap, edgeGap, edgeTest;
} PairOut;

#if defined( DIALECT_V4 )
#define BIGV 2147483647.0
#else
#define BIGV 1.0e6
#endif

static double margin( T x, int s )
{
	double raw = (double)x;
	return raw >= BIGV ? 1e30 : toy_val( x, s );
}

static void collect( const PassIO* io, int n, PairOut* o )
{
	for ( int k = 0; k < n; ++k )
	{
		const SatAxis* s = io->sat + k;
		const Manifold* m = io->out + k;
		const NarrowDiag* g = io->diag + k;
		PairOut* p = o + k;
		memset( p, 0, sizeof( *p ) );
		p->kind = s->kind;
		p->type = s->type;
		p->sepA = s->sepA;
		p->sepB = s->sepB;
		p->cacheType = s->cacheType;
		p->cacheA = s->cacheA;
		p->cacheB = s->cacheB;
		p->faceA = s->faceA;
		p->vertexB = s->vertexB;
		p->faceB = s->faceB;
		p->vertexA = s->vertexA;
		p->edgeA = s->edgeA;
		p->edgeB = s->edgeB;
		p->axisType = m->axisType;
		p->axisA = m->axisA;
		p->axisB = m->axisB;
		p->pointCount = m->pointCount;
		p->normal[0] = toy_val( m->normal.x, S_Q );
		p->normal[1] = toy_val( m->normal.y, S_Q );
		p->normal[2] = toy_val( m->normal.z, S_Q );
		for ( int i = 0; i < MANIFOLD_POINTS; ++i )
		{
			const ManifoldPoint* mp = m->points + i;
			p->ids[i] = mp->featureId;
			p->marker[i] = (int32_t)toy_val( mp->normalImpulse, 0 );
			p->anchorA[i][0] = toy_val( mp->anchorA.x, S_R );
			p->anchorA[i][1] = toy_val( mp->anchorA.y, S_R );
			p->anchorA[i][2] = toy_val( mp->anchorA.z, S_R );
			p->anchorB[i][0] = toy_val( mp->anchorB.x, S_R );
			p->anchorB[i][1] = toy_val( mp->anchorB.y, S_R );
			p->anchorB[i][2] = toy_val( mp->anchorB.z, S_R );
			p->sep[i] = toy_val( mp->separation, S_S );
		}
		p->frictionMarker = (int32_t)toy_val( m->frictionImpulse.x, 0 );
		p->faceASep = toy_val( s->faceASep, S_R );
		p->faceBSep = toy_val( s->faceBSep, S_R );
		p->edgeSep = toy_val( s->edgeSep, S_R );
		p->axisSep = toy_val( m->axisSeparation, S_S );
		p->axisLen = margin( g->axisLen, S_R );
		p->clipLen = margin( g->clipLen, S_R );
		p->clipArea = margin( g->clipArea, S_D2 );
		p->clipCos = margin( g->clipCos, S_Q );
		p->touchLen = margin( g->touchLen, S_R );
		p->supportA = margin( g->supportA, S_R );
		p->supportB = margin( g->supportB, S_R );
		p->supportGap = margin( g->supportGap, S_R );
		p->faceAGap = margin( g->faceAGap, S_R );
		p->faceBGap = margin( g->faceBGap, S_R );
		p->edgeGap = margin( g->edgeGap, S_R );
		p->edgeTest = margin( g->edgeTest, S_R );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// The comparison with D
// ---------------------------------------------------------------------------------------------------------------------

typedef struct Stat
{
	double sum, max;
	long long n;
	int at; // the pair with the largest
} Stat;

static void stat_add( Stat* s, double e, int k )
{
	s->sum += e * e;
	s->at = e > s->max ? k : s->at;
	s->max = e > s->max ? e : s->max;
	s->n += 1;
}

static double stat_rms( const Stat* s )
{
	return s->n ? sqrt( s->sum / (double)s->n ) : 0.0;
}

// The axis, as the gate compares it: the kind of the cache and its feature (a face, or the edge pair); the support
// vertex that goes with a face is compared apart (parallel faces tie their vertices by construction)
static int axis_equal( const PairOut* a, const PairOut* b )
{
	if ( a->axisType != b->axisType )
	{
		return 0;
	}
	switch ( a->axisType )
	{
		case AXIS_FACE_A:
			return a->axisA == b->axisA;
		case AXIS_FACE_B:
			return a->axisB == b->axisB;
		case AXIS_EDGE:
			return a->axisA == b->axisA && a->axisB == b->axisB;
		default:
			return 1;
	}
}

static int vertex_equal( const PairOut* a, const PairOut* b )
{
	if ( a->axisType == AXIS_FACE_A )
	{
		return a->axisB == b->axisB;
	}
	if ( a->axisType == AXIS_FACE_B )
	{
		return a->axisA == b->axisA;
	}
	return 1;
}

static int ids_equal( const PairOut* a, const PairOut* b )
{
	if ( a->pointCount != b->pointCount )
	{
		return 0;
	}
	for ( int i = 0; i < a->pointCount; ++i )
	{
		if ( a->ids[i] != b->ids[i] )
		{
			return 0;
		}
	}
	return 1;
}

static int markers_equal( const PairOut* a, const PairOut* b )
{
	for ( int i = 0; i < a->pointCount; ++i )
	{
		if ( a->marker[i] != b->marker[i] )
		{
			return 0;
		}
	}
	return a->frictionMarker == b->frictionMarker;
}

static const char* axis_name( int t )
{
	switch ( t )
	{
		case AXIS_FACE_A:
			return "faceA";
		case AXIS_FACE_B:
			return "faceB";
		case AXIS_EDGE:
			return "edge";
		default:
			return "none";
	}
}

static void describe( const char* what, const Corpus* c, int k, const PairOut* d, const PairOut* x )
{
	say( "    %s pair %d (%s, %s, %s): D %s %d/%d %d pts ids", what, k, g_catNames[c->category[k]],
		 c->band[k] < BAND_COUNT ? g_bandNames[c->band[k]] : "pose", g_comboNames[c->combo[k]], axis_name( d->axisType ), d->axisA, d->axisB,
		 d->pointCount );
	for ( int i = 0; i < d->pointCount; ++i )
	{
		say( " %08x", d->ids[i] );
	}
	say( " | " DIALECT_NAME " %s %d/%d %d pts ids", axis_name( x->axisType ), x->axisA, x->axisB, x->pointCount );
	for ( int i = 0; i < x->pointCount; ++i )
	{
		say( " %08x", x->ids[i] );
	}
	say( "\n      markers D" );
	for ( int i = 0; i < d->pointCount; ++i )
	{
		say( " %d", d->marker[i] );
	}
	say( " friction %d, " DIALECT_NAME, d->frictionMarker );
	for ( int i = 0; i < x->pointCount; ++i )
	{
		say( " %d", x->marker[i] );
	}
	say( " friction %d", x->frictionMarker );
	say( "\n      D: sat kind %d type %d cache %s, faceA %d (%.9g) faceB %d (%.9g) edge %d/%d (%.9g); margins axis %.3g touch %.3g clip %.3g "
		 "area %.3g cos %.3g; gaps faceA %.3g faceB %.3g edge %.3g test %.3g\n",
		 d->kind, d->type, axis_name( d->cacheType ), d->faceA, d->faceASep, d->faceB, d->faceBSep, d->edgeA, d->edgeB, d->edgeSep, d->axisLen,
		 d->touchLen, d->clipLen, d->clipArea, d->clipCos, d->faceAGap, d->faceBGap, d->edgeGap, d->edgeTest );
	say( "      " DIALECT_NAME ": sat kind %d type %d cache %s, faceA %d (%.9g) faceB %d (%.9g) edge %d/%d (%.9g); margins axis %.3g clip %.3g "
		 "area %.3g cos %.3g\n",
		 x->kind, x->type, axis_name( x->cacheType ), x->faceA, x->faceASep, x->faceB, x->faceBSep, x->edgeA, x->edgeB, x->edgeSep, x->axisLen,
		 x->clipLen, x->clipArea, x->clipCos );
}

typedef struct Verdict
{
	int compared;									   // pairs compared (pass 2: those whose pass 1 agreed)
	int axisTies, clipTies, clean;					   // pairs by D's margins
	int axisDiff, idsDiff, vertexDiff, markerDiff;	   // outside ties
	int axisDiffInTies, idsDiffInAxisTies, idsDiffInClipTies; // inside ties (reported)
	int vertexTies, faceAxes;						   // the support vertex beside a face axis: its own ties
	int touching, separated, cacheTried, cacheSeparated; // D's paths
	int cacheKept;									   // D kept the cached feature's contact
	Stat normal, anchor, sep, faceSep;
} Verdict;

// A clip tie: a clipping, reduction or incident-face decision closer than the tolerance (D's margins)
static int clip_tie( const PairOut* d, double tie )
{
	return d->clipLen < tie || d->clipArea < tie * 1.0 || d->clipCos < tie / 1.0;
}

// An axis tie: an axis decision closer than the tolerance; or, when the axis hung on a contact's outcome (a face
// contact that touches or not, its clipped separation against the edge's, a cached feature kept or not), whether the
// contact touches at all or which incident face it clips
static int axis_tie( const PairOut* d, double tie )
{
	int fromContact = d->kind == SAT_OVERLAP || ( d->kind != SAT_CACHE_SEPARATED && d->cacheType != AXIS_NONE );
	return d->axisLen < tie || ( fromContact && ( d->touchLen < tie || d->clipCos < tie / 1.0 ) );
}

// Pass 1 agreed (the axis and the ids): pass 2 starts from the same manifold (the support vertex stored beside a face is
// never read again: the cached path finds its own)
static int agreed( const PairOut* d, const PairOut* x )
{
	return axis_equal( d, x ) && ids_equal( d, x );
}

static Verdict compare( const Corpus* c, const PairOut* D, const PairOut* X, const uint8_t* mask, int n, int pass, double tie, int listMax )
{
	Verdict v;
	memset( &v, 0, sizeof( v ) );
	int listed = 0;
	int byCat[CAT_COUNT][4];
	memset( byCat, 0, sizeof( byCat ) );
	for ( int k = 0; k < n; ++k )
	{
		if ( mask && !mask[k] )
		{
			continue;
		}
		const PairOut* d = D + k;
		const PairOut* x = X + k;
		v.compared += 1;
		v.touching += d->pointCount > 0;
		v.separated += d->kind == SAT_SEPARATED || d->kind == SAT_CACHE_SEPARATED;
		v.cacheTried += d->cacheType != AXIS_NONE;
		v.cacheSeparated += d->kind == SAT_CACHE_SEPARATED;
		v.cacheKept += d->cacheType != AXIS_NONE && d->kind != SAT_CACHE_SEPARATED && d->axisType == d->cacheType;
		int ax = axis_equal( d, x );
		int ids = ids_equal( d, x );
		byCat[c->category[k]][0] += 1;
		if ( axis_tie( d, tie ) )
		{
			v.axisTies += 1;
			v.axisDiffInTies += !ax;
			v.idsDiffInAxisTies += !ids;
			byCat[c->category[k]][1] += 1;
			byCat[c->category[k]][3] += !ax || !ids;
			continue;
		}
		if ( !ax )
		{
			v.axisDiff += 1;
			if ( listed < listMax )
			{
				describe( "AXIS", c, k, d, x );
				listed += 1;
			}
			continue;
		}
		// the support vertex beside a fresh face axis (a cache carried in pass 2 holds pass 1's, compared there)
		int carried = d->kind == SAT_CACHE_SEPARATED || ( d->cacheType != AXIS_NONE && d->axisType == d->cacheType );
		if ( ( d->axisType == AXIS_FACE_A || d->axisType == AXIS_FACE_B ) && !carried )
		{
			double gap = d->axisType == AXIS_FACE_A ? d->supportA : d->supportB;
			v.faceAxes += 1;
			if ( gap < tie )
			{
				v.vertexTies += 1;
			}
			else if ( !vertex_equal( d, x ) )
			{
				v.vertexDiff += 1;
				if ( listed < listMax )
				{
					describe( "VERTEX", c, k, d, x );
					listed += 1;
				}
			}
		}
		if ( clip_tie( d, tie ) )
		{
			v.clipTies += 1;
			v.idsDiffInClipTies += !ids;
			byCat[c->category[k]][2] += 1;
			byCat[c->category[k]][3] += !ids;
			continue;
		}
		v.clean += 1;
		if ( !ids )
		{
			v.idsDiff += 1;
			if ( listed < listMax )
			{
				describe( "IDS", c, k, d, x );
				listed += 1;
			}
			continue;
		}
		if ( pass == 1 && !markers_equal( d, x ) )
		{
			v.markerDiff += 1;
			if ( listed < listMax )
			{
				describe( "WARM START", c, k, d, x );
				listed += 1;
			}
		}
		// errors on the agreeing points
		if ( d->pointCount > 0 )
		{
			double dn[3] = { x->normal[0] - d->normal[0], x->normal[1] - d->normal[1], x->normal[2] - d->normal[2] };
			stat_add( &v.normal, sqrt( dot3d( dn, dn ) ), k );
		}
		for ( int i = 0; i < d->pointCount; ++i )
		{
			double da[3] = { x->anchorA[i][0] - d->anchorA[i][0], x->anchorA[i][1] - d->anchorA[i][1], x->anchorA[i][2] - d->anchorA[i][2] };
			stat_add( &v.anchor, sqrt( dot3d( da, da ) ), k );
			stat_add( &v.sep, fabs( x->sep[i] - d->sep[i] ), k );
		}
		if ( d->faceA == x->faceA && d->faceA >= 0 )
		{
			stat_add( &v.faceSep, fabs( x->faceASep - d->faceASep ), k );
		}
		if ( d->faceB == x->faceB && d->faceB >= 0 )
		{
			stat_add( &v.faceSep, fabs( x->faceBSep - d->faceBSep ), k );
		}
	}
	say( "  pass %d: %d pairs compared", pass + 1, v.compared );
	if ( mask )
	{
		say( " (of %d: pass 1 agreed on the rest)", n );
	}
	say( "; D: %d touching, %d separated (%d by the cached axis), %d tried a cached feature and %d kept its contact\n", v.touching, v.separated,
		 v.cacheSeparated, v.cacheTried, v.cacheKept );
	say( "    ties (D's margins, tolerance %.3g m, %.3g m^2, %.3g): %d axis ties (%d with another axis, %d other points), %d clip ties (%d "
		 "with other points)\n",
		 tie, tie, tie, v.axisTies, v.axisDiffInTies, v.idsDiffInAxisTies, v.clipTies, v.idsDiffInClipTies );
	say( "    GATE outside ties: axis kind and index differ in %d of %d; point count or ids differ in %d of %d (no tie at all)", v.axisDiff,
		 v.compared - v.axisTies, v.idsDiff, v.clean );
	if ( pass == 1 )
	{
		say( "; warm-start impulses (markers) differ in %d", v.markerDiff );
	}
	say( "; the support vertex beside a face axis differs in %d of %d outside its own ties (%d ties)\n", v.vertexDiff, v.faceAxes - v.vertexTies,
		 v.vertexTies );
	say( "    errors against D (agreeing pairs): normal rms %.3g max %.3g; anchor rms %.3g max %.3g m (%lld points); separation rms %.3g "
		 "max %.3g m; SAT face separations rms %.3g max %.3g m\n",
		 stat_rms( &v.normal ), v.normal.max, stat_rms( &v.anchor ), v.anchor.max, v.anchor.n, stat_rms( &v.sep ), v.sep.max, stat_rms( &v.faceSep ),
		 v.faceSep.max );
	if ( v.anchor.n > 0 )
	{
		int k = v.anchor.at;
		say( "    largest anchor error: pair %d (%s, %s, %s); its closest clip distance in D %.3g m\n", k, g_catNames[c->category[k]],
			 c->band[k] < BAND_COUNT ? g_bandNames[c->band[k]] : "pose", g_comboNames[c->combo[k]], D[k].clipLen );
	}
	say( "    by pose (pairs, axis ties, clip ties, ties that differ):" );
	for ( int i = 0; i < CAT_COUNT; ++i )
	{
		say( " %s %d/%d/%d/%d;", g_catNames[i], byCat[i][0], byCat[i][1], byCat[i][2], byCat[i][3] );
	}
	say( "\n" );
	return v;
}

// ---------------------------------------------------------------------------------------------------------------------
// GPU: both passes through the kernels' SPIR-V, against the twin word by word
// ---------------------------------------------------------------------------------------------------------------------

#if HAS_GPU
typedef struct GpuIn
{
	const void* data[TOY_BUFFERS];
	size_t bytes[TOY_BUFFERS];
} GpuIn;

static void upload( VkGpu* g, VkBuf* dst, VkBuf* staging, const void* src, size_t bytes )
{
	memcpy( staging->mapped, src, bytes );
	VkCommandBuffer cb = vku_begin( g );
	VkBufferCopy c = { 0, 0, bytes };
	vkCmdCopyBuffer( cb, staging->buffer, dst->buffer, 1, &c );
	vku_submit_wait( g, cb );
}

static void download( VkGpu* g, VkBuf* src, VkBuf* staging, void* dst, size_t bytes )
{
	VkCommandBuffer cb = vku_begin( g );
	vku_barrier( cb );
	VkBufferCopy c = { 0, 0, bytes };
	vkCmdCopyBuffer( cb, src->buffer, staging->buffer, 1, &c );
	vku_submit_wait( g, cb );
	memcpy( dst, staging->mapped, bytes );
}

// Runs narrowSat then narrowClip over n pairs; outputs: satAxes, manifolds, diagnostics, the saturation count
static int gpu_pass( VkGpu* g, const GpuIn* in, int n, SatAxis* sat, Manifold* out, NarrowDiag* diag, uint32_t* saturations )
{
	VkDescriptorSetLayoutBinding bind[TOY_BUFFERS];
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		bind[i] = ( VkDescriptorSetLayoutBinding ){ (uint32_t)i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL };
	}
	VkDescriptorSetLayout dsl;
	VkDescriptorSetLayoutCreateInfo dci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
	dci.bindingCount = TOY_BUFFERS;
	dci.pBindings = bind;
	VK_CHECK( vkCreateDescriptorSetLayout( g->device, &dci, NULL, &dsl ) );
	VkPushConstantRange pcr = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( Push ) };
	VkPipelineLayout layout;
	VkPipelineLayoutCreateInfo lci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	lci.setLayoutCount = 1;
	lci.pSetLayouts = &dsl;
	lci.pushConstantRangeCount = 1;
	lci.pPushConstantRanges = &pcr;
	VK_CHECK( vkCreatePipelineLayout( g->device, &lci, NULL, &layout ) );
	VkPipeline pSat = vku_pipeline( g, layout, "gen/toy/" DIALECT_NAME "/narrowSat.spv", "main" );
	VkPipeline pClip = vku_pipeline( g, layout, "gen/toy/" DIALECT_NAME "/narrowClip.spv", "main" );
	if ( pSat == VK_NULL_HANDLE || pClip == VK_NULL_HANDLE )
	{
		return 0;
	}
	VkBuf bufs[TOY_BUFFERS];
	size_t maxBytes = 16;
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		size_t b = in->bytes[i] > 16 ? in->bytes[i] : 16;
		bufs[i] = vku_buffer( g, b, false );
		maxBytes = b > maxBytes ? b : maxBytes;
	}
	VkBuf staging = vku_buffer( g, maxBytes, true );
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		if ( in->data[i] )
		{
			upload( g, bufs + i, &staging, in->data[i], in->bytes[i] );
		}
		else
		{
			VkCommandBuffer cb = vku_begin( g );
			vkCmdFillBuffer( cb, bufs[i].buffer, 0, VK_WHOLE_SIZE, 0 );
			vku_submit_wait( g, cb );
		}
	}
	VkDescriptorPool pool;
	VkDescriptorPoolSize ps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, TOY_BUFFERS };
	VkDescriptorPoolCreateInfo pci = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
	pci.maxSets = 1;
	pci.poolSizeCount = 1;
	pci.pPoolSizes = &ps;
	VK_CHECK( vkCreateDescriptorPool( g->device, &pci, NULL, &pool ) );
	VkDescriptorSet set;
	VkDescriptorSetAllocateInfo ai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	ai.descriptorPool = pool;
	ai.descriptorSetCount = 1;
	ai.pSetLayouts = &dsl;
	VK_CHECK( vkAllocateDescriptorSets( g->device, &ai, &set ) );
	VkDescriptorBufferInfo bi[TOY_BUFFERS];
	VkWriteDescriptorSet w[TOY_BUFFERS];
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		bi[i] = ( VkDescriptorBufferInfo ){ bufs[i].buffer, 0, VK_WHOLE_SIZE };
		w[i] = ( VkWriteDescriptorSet ){ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
		w[i].dstSet = set;
		w[i].dstBinding = (uint32_t)i;
		w[i].descriptorCount = 1;
		w[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		w[i].pBufferInfo = &bi[i];
	}
	vkUpdateDescriptorSets( g->device, TOY_BUFFERS, w, 0, NULL );

	VkCommandBuffer cb = vku_begin( g );
	vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_COMPUTE, layout, 0, 1, &set, 0, NULL );
	Push push = { 0, (uint32_t)n, 0, 0 };
	vkCmdPushConstants( cb, layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( push ), &push );
	vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE, pSat );
	vkCmdDispatch( cb, ( (uint32_t)n + 63 ) / 64, 1, 1 );
	vku_barrier( cb );
	vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE, pClip );
	vkCmdDispatch( cb, ( (uint32_t)n + 63 ) / 64, 1, 1 );
	vku_submit_wait( g, cb );

	download( g, bufs + 15, &staging, sat, (size_t)n * sizeof( SatAxis ) );
	download( g, bufs + 13, &staging, out, (size_t)n * sizeof( Manifold ) );
	download( g, bufs + 16, &staging, diag, (size_t)n * sizeof( NarrowDiag ) );
	uint32_t counters[4];
	download( g, bufs + 11, &staging, counters, sizeof( counters ) );
	*saturations = counters[0];

	vkDestroyPipeline( g->device, pSat, NULL );
	vkDestroyPipeline( g->device, pClip, NULL );
	vkDestroyDescriptorPool( g->device, pool, NULL );
	vkDestroyPipelineLayout( g->device, layout, NULL );
	vkDestroyDescriptorSetLayout( g->device, dsl, NULL );
	for ( int i = 0; i < TOY_BUFFERS; ++i )
	{
		vku_free( g, bufs + i );
	}
	vku_free( g, &staging );
	return 1;
}

// Words that differ between the twin's and a GPU's records (count of records with any difference)
static long long diff_records( const void* a, const void* b, int n, size_t size, int* firstBad )
{
	long long bad = 0;
	*firstBad = -1;
	for ( int k = 0; k < n; ++k )
	{
		if ( memcmp( (const char*)a + (size_t)k * size, (const char*)b + (size_t)k * size, size ) != 0 )
		{
			if ( *firstBad < 0 )
			{
				*firstBad = k;
			}
			bad += 1;
		}
	}
	return bad;
}
#endif

// ---------------------------------------------------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------------------------------------------------

#define REF_MAGIC 0x4e415252u // "NARR"

int main( int argc, char** argv )
{
	int n = 10000;
	uint64_t seed = 20261004;
	const char* outPath = NULL;
	const char* refPath = NULL;
	const char* logPath = NULL;
	double tie = 1e-5;
	int gpuMask = 0xff;
	int listMax = 12;
	for ( int i = 1; i < argc; ++i )
	{
		const char* a = argv[i];
		const char* v = i + 1 < argc ? argv[i + 1] : "";
		if ( strcmp( a, "--n" ) == 0 )
			n = atoi( v ), ++i;
		else if ( strcmp( a, "--seed" ) == 0 )
			seed = (uint64_t)strtoull( v, NULL, 10 ), ++i;
		else if ( strcmp( a, "--out" ) == 0 )
			outPath = v, ++i;
		else if ( strcmp( a, "--ref" ) == 0 )
			refPath = v, ++i;
		else if ( strcmp( a, "--tie" ) == 0 )
			tie = atof( v ), ++i;
		else if ( strcmp( a, "--gpus" ) == 0 )
			gpuMask = atoi( v ), ++i;
		else if ( strcmp( a, "--list" ) == 0 )
			listMax = atoi( v ), ++i;
		else if ( strcmp( a, "--log" ) == 0 )
			logPath = v, ++i;
		else
		{
			fprintf( stderr, "unknown option %s\n", a );
			return 1;
		}
	}
	if ( logPath )
	{
		g_log = fopen( logPath, "w" );
	}
	say( "narrowphase corpus, dialect %s, %d pairs, seed %llu\n", DIALECT_NAME, n, (unsigned long long)seed );
	say( "twin: %s\n", toy_twin_info() );
	size_t cs[4] = { sizeof( Pair ), sizeof( Manifold ), sizeof( SatAxis ), sizeof( NarrowDiag ) };
	int layoutBad = 0;
	for ( int i = 0; i < 4; ++i )
	{
		layoutBad |= cs[i] != toy_sizeof( 9 + i );
	}
	say( "layout sizes C/twin: Pair %zu/%zu Manifold %zu/%zu SatAxis %zu/%zu NarrowDiag %zu/%zu\n", cs[0], toy_sizeof( 9 ), cs[1], toy_sizeof( 10 ),
		 cs[2], toy_sizeof( 11 ), cs[3], toy_sizeof( 12 ) );
	if ( layoutBad )
	{
		say( "FAIL: layout sizes differ\n" );
		return 2;
	}

	Corpus c;
	make_corpus( &c, n, seed );
	int catCount[CAT_COUNT] = { 0 }, bandCount[BAND_COUNT + 1] = { 0 }, comboCount[3] = { 0 };
	for ( int k = 0; k < n; ++k )
	{
		catCount[c.category[k]] += 1;
		bandCount[c.band[k]] += 1;
		comboCount[c.combo[k]] += 1;
	}
	say( "corpus: %d hulls (%d big boxes, %d boxes, %d chunks, %d chunks redrawn);", c.hullCount, POOL_BIG, POOL_BOX, POOL_CHUNK, c.chunkRedraws );
	for ( int i = 0; i < 3; ++i )
	{
		say( " %s %d", g_comboNames[i], comboCount[i] );
	}
	say( "\n  poses:" );
	for ( int i = 0; i < CAT_COUNT; ++i )
	{
		say( " %s %d;", g_catNames[i], catCount[i] );
	}
	say( "\n  separations:" );
	for ( int i = 0; i < BAND_COUNT; ++i )
	{
		say( " %s %d;", g_bandNames[i], bandCount[i] );
	}
	say( " generic poses %d\n", bandCount[BAND_COUNT] );

	// quantise both passes (the scene is all bodies, two per pair)
	ToySettings settings;
	toy_default_settings( &settings );
	ToyData data[2];
	for ( int p = 0; p < 2; ++p )
	{
		Scene sc;
		memset( &sc, 0, sizeof( sc ) );
		snprintf( sc.name, sizeof( sc.name ), "corpus" );
		sc.seed = seed;
		sc.bodyCount = 2 * n;
		sc.hullCount = c.hullCount;
		sc.bodies = c.bodies[p];
		sc.hulls = c.hulls;
		toy_quantize( &sc, &settings, data + p );
	}
	say( "quantisation range errors %d, %d\n", data[0].rangeErrors, data[1].rangeErrors );

	// the twin, both passes
	BodyState* state = (BodyState*)calloc( (size_t)( 2 * n ), sizeof( BodyState ) );
	Aabb* aabbs = (Aabb*)calloc( (size_t)( 2 * n ), sizeof( Aabb ) );
	Hash2* hashes = (Hash2*)calloc( (size_t)( 2 * n ), sizeof( Hash2 ) );
	PassIO io[2];
	PairOut* res[2];
	Params params[2];
	int fpFlags[2];
	uint32_t twinSat[2];
	toy_reset_saturations();
	for ( int p = 0; p < 2; ++p )
	{
		pass_alloc( io + p, n );
		pass_setup( io + p, n, p );
		if ( p == 1 )
		{
			make_prev( io[0].out, io[1].prev, n );
		}
		params[p] = data[p].params;
		params[p].narrowDiag = 1;
		params[p].recycleDistance = 0; // the full narrowphase on every pair (no contact recycling)
		params[p].recycleNonTouching = 0;
		uint32_t before = toy_saturations();
		fpFlags[p] = run_twin( data + p, params + p, io + p, n, state, aabbs, hashes );
		twinSat[p] = toy_saturations() - before;
		res[p] = (PairOut*)calloc( (size_t)n, sizeof( PairOut ) );
		collect( io + p, n, res[p] );
		char names[96];
		say( "twin pass %d: saturations %u, floating-point sentinel %s\n", p + 1, twinSat[p], fpFlags[p] ? fp_names( fpFlags[p], names, sizeof( names ) ) : "clean" );
	}
	int failures = ( fpFlags[0] | fpFlags[1] ) != 0;
	failures += ( twinSat[0] | twinSat[1] ) != 0;
	uint64_t h = 1469598103934665603ULL;
	for ( int p = 0; p < 2; ++p )
	{
		const uint8_t* b = (const uint8_t*)io[p].out;
		for ( size_t i = 0; i < (size_t)n * sizeof( Manifold ); ++i )
		{
			h ^= b[i];
			h *= 1099511628211ULL;
		}
		b = (const uint8_t*)io[p].sat;
		for ( size_t i = 0; i < (size_t)n * sizeof( SatAxis ); ++i )
		{
			h ^= b[i];
			h *= 1099511628211ULL;
		}
	}
	say( "narrow %s corpus hash %016llx (manifolds and SAT records, both passes; twin: %s)\n", DIALECT_NAME, (unsigned long long)h, toy_twin_info() );

	if ( outPath )
	{
		FILE* f = fopen( outPath, "wb" );
		if ( f == NULL )
		{
			say( "FAIL: cannot write %s\n", outPath );
			return 1;
		}
		uint32_t hdr[4] = { REF_MAGIC, (uint32_t)n, (uint32_t)sizeof( PairOut ), (uint32_t)seed };
		fwrite( hdr, sizeof( hdr ), 1, f );
		fwrite( res[0], sizeof( PairOut ), (size_t)n, f );
		fwrite( res[1], sizeof( PairOut ), (size_t)n, f );
		fclose( f );
		say( "wrote %s\n", outPath );
	}

	// against D
	if ( refPath )
	{
		FILE* f = fopen( refPath, "rb" );
		uint32_t hdr[4] = { 0, 0, 0, 0 };
		if ( f == NULL || fread( hdr, sizeof( hdr ), 1, f ) != 1 || hdr[0] != REF_MAGIC || hdr[1] != (uint32_t)n || hdr[2] != sizeof( PairOut ) ||
			 hdr[3] != (uint32_t)seed )
		{
			say( "FAIL: reference %s missing or another corpus\n", refPath );
			failures += 1;
		}
		else
		{
			PairOut* ref[2];
			for ( int p = 0; p < 2; ++p )
			{
				ref[p] = (PairOut*)calloc( (size_t)n, sizeof( PairOut ) );
				size_t got = fread( ref[p], sizeof( PairOut ), (size_t)n, f );
				(void)got;
			}
			say( "against D (%s):\n", refPath );
			uint8_t* mask = (uint8_t*)calloc( (size_t)n, 1 );
			for ( int k = 0; k < n; ++k )
			{
				mask[k] = (uint8_t)agreed( ref[0] + k, res[0] + k );
			}
			for ( int p = 0; p < 2; ++p )
			{
				Verdict v = compare( &c, ref[p], res[p], p == 0 ? NULL : mask, n, p, tie, listMax );
				failures += v.axisDiff > 0 || v.idsDiff > 0 || v.markerDiff > 0 || v.vertexDiff > 0;
			}
			free( mask );
			free( ref[0] );
			free( ref[1] );
		}
		if ( f )
		{
			fclose( f );
		}
	}

	// GPUs: both passes, against this twin word by word
#if HAS_GPU
	VkInstance inst = vku_create_instance();
	VkPhysicalDevice phys[8];
	int found = vku_list_gpus( inst, phys, 8 );
	SatAxis* gsat = (SatAxis*)calloc( (size_t)n, sizeof( SatAxis ) );
	Manifold* gout = (Manifold*)calloc( (size_t)n, sizeof( Manifold ) );
	NarrowDiag* gdiag = (NarrowDiag*)calloc( (size_t)n, sizeof( NarrowDiag ) );
	for ( int gi = 0; gi < found; ++gi )
	{
		if ( !( gpuMask & ( 1 << gi ) ) )
		{
			continue;
		}
		VkGpu g;
		vku_open_gpu( inst, phys[gi], &g );
		say( "gpu %d: %s (%s), driver 0x%x (F-plain: no float-control modes declared)\n", gi, g.props.deviceName, g.vendor, g.props.driverVersion );
		for ( int p = 0; p < 2; ++p )
		{
			GpuIn in;
			memset( &in, 0, sizeof( in ) );
			ToyData* d = data + p;
			in.data[0] = d->hulls;
			in.bytes[0] = (size_t)d->hullCount * sizeof( Hull );
			in.data[1] = d->points;
			in.bytes[1] = (size_t)d->pointCount * sizeof( V3 );
			in.data[2] = d->faces;
			in.bytes[2] = (size_t)d->faceCount * sizeof( HullFace );
			in.data[3] = d->edges;
			in.bytes[3] = (size_t)d->edgeCount * sizeof( uint32_t );
			in.bytes[4] = (size_t)d->bodyCount * sizeof( BodyState );
			in.data[5] = d->pose;
			in.bytes[5] = (size_t)d->bodyCount * sizeof( BodyPose );
			in.data[6] = d->mass;
			in.bytes[6] = (size_t)d->bodyCount * sizeof( BodyMass );
			in.bytes[7] = (size_t)d->bodyCount * sizeof( Aabb );
			in.data[8] = io[p].lists;
			in.bytes[8] = (size_t)n * sizeof( uint32_t );
			in.data[9] = params + p;
			in.bytes[9] = sizeof( Params );
			in.bytes[10] = 16;
			in.bytes[11] = 16;
			in.data[12] = io[p].pairs;
			in.bytes[12] = (size_t)n * sizeof( Pair );
			in.bytes[13] = (size_t)n * sizeof( Manifold );
			in.data[14] = io[p].prev;
			in.bytes[14] = (size_t)n * sizeof( Manifold );
			in.bytes[15] = (size_t)n * sizeof( SatAxis );
			in.bytes[16] = (size_t)n * sizeof( NarrowDiag );
			in.bytes[17] = 16;
			uint32_t sat = 0;
			if ( !gpu_pass( &g, &in, n, gsat, gout, gdiag, &sat ) )
			{
				say( "FAIL: no pipeline on %s\n", g.vendor );
				return 2;
			}
			int f1, f2, f3;
			long long bs = diff_records( io[p].sat, gsat, n, sizeof( SatAxis ), &f1 );
			long long bm = diff_records( io[p].out, gout, n, sizeof( Manifold ), &f2 );
			long long bd = diff_records( io[p].diag, gdiag, n, sizeof( NarrowDiag ), &f3 );
			say( "  %s pass %d: SAT records %lld of %d differ, manifolds %lld differ, diagnostics %lld differ, saturations %u (twin %u)%s\n", g.vendor,
				 p + 1, bs, n, bm, bd, sat, twinSat[p], bs || bm || bd || sat != twinSat[p] ? "" : ": every word identical" );
			if ( bs || bm )
			{
				int k = f1 >= 0 ? f1 : f2;
				say( "    first: pair %d (%s)", k, g_catNames[c.category[k]] );
				if ( f1 < 0 && f2 >= 0 )
				{
					// the manifold's differing words: offset in 32-bit words, the twin's and the GPU's
					const uint32_t* tw = (const uint32_t*)( io[p].out + f2 );
					const uint32_t* gw = (const uint32_t*)( gout + f2 );
					int shown = 0;
					for ( int w = 0; w < (int)( sizeof( Manifold ) / 4 ) && shown < 8; ++w )
					{
						if ( tw[w] != gw[w] )
						{
							say( "%s word %d: %08x %08x", shown ? "," : ";", w, tw[w], gw[w] );
							++shown;
						}
					}
				}
				say( "\n" );
			}
		}
		vkDeviceWaitIdle( g.device );
		vku_close_gpu( &g );
	}
	free( gsat );
	free( gout );
	free( gdiag );
	vkDestroyInstance( inst, NULL );
#endif

	say( "%s\n", failures ? "FAIL" : "PASS" );
	for ( int p = 0; p < 2; ++p )
	{
		pass_free( io + p );
		free( res[p] );
		toy_free_data( data + p );
	}
	free( state );
	free( aabbs );
	free( hashes );
	free_corpus( &c );
	if ( g_log )
	{
		fclose( g_log );
	}
	return failures ? 1 : 0;
}
