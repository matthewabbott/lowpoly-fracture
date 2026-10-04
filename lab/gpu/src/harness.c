// harness.c: E11. Runs the Box3D-style soft-step contact solve (solver.slang) on every Vulkan GPU
// and on the CPU twin (the same Slang source compiled to C++), then bit-compares the final state.
// Built once per number format: -DVAR_F, -DVAR_I64 or -DVAR_I32 (see layout.h).
//
//   harness_F --n 1000,10000,100000 --steps 60 --threads 1,8 --reps 3 --log logs/F.txt
//
// The scenario is generated in double with + - * / sqrt only (no libm), so it is the same bits on
// any IEEE machine; quantisation to each format is exact-rounded from those doubles.
#include "layout.h"
#include "vk_util.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( _WIN32 )
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#endif

#if defined( VAR_F )
#define VARIANT_NAME "F"
#elif defined( VAR_I64 )
#define VARIANT_NAME "I64"
#elif defined( VAR_V4 )
#define VARIANT_NAME "V4"
#else
#define VARIANT_NAME "I32"
#endif

// ------------------------------------------------------------------------------------------------
// CPU twin (twin_glue.cpp)
// ------------------------------------------------------------------------------------------------

void twin_bind( void* bodies, size_t nb, void* poses, size_t np, void* cons, size_t nc, void* imp, size_t ni, void* params,
				void* counters );
void twin_run( int entry, uint32_t start, uint32_t count, uint32_t g0, uint32_t g1 );
size_t twin_sizeof( int which );
const char* twin_info( void );

enum
{
	E_INTVEL = 0,
	E_WARM = 1,
	E_PUSH = 2,
	E_INTPOS = 3,
	E_RELAX = 4,
	E_FINAL = 5,
	E_COUNT = 6
};
static const char* g_entryNames[E_COUNT] = { "integrateVelocities", "warmStart", "push", "integratePositions", "relax", "finalizeBodies" };

// ------------------------------------------------------------------------------------------------
// PCG32 and portable double helpers
// ------------------------------------------------------------------------------------------------

typedef struct Pcg
{
	uint64_t state, inc;
} Pcg;

static uint32_t pcg_next( Pcg* r )
{
	uint64_t old = r->state;
	r->state = old * 6364136223846793005ULL + r->inc;
	uint32_t xs = (uint32_t)( ( ( old >> 18u ) ^ old ) >> 27u );
	uint32_t rot = (uint32_t)( old >> 59u );
	return ( xs >> rot ) | ( xs << ( ( 0u - rot ) & 31 ) );
}

static Pcg pcg_seed( uint64_t seed, uint64_t seq )
{
	Pcg r = { 0, ( seq << 1u ) | 1u };
	pcg_next( &r );
	r.state += seed;
	pcg_next( &r );
	return r;
}

static double urand( Pcg* r ) // [0, 1), 32 random bits
{
	return (double)pcg_next( r ) * ( 1.0 / 4294967296.0 );
}

static double urange( Pcg* r, double lo, double hi )
{
	return lo + ( hi - lo ) * urand( r );
}

// log-uniform without libm: a uniform decade (table) and a uniform mantissa within it
static double logrand( Pcg* r, double lo, double hi )
{
	for ( ;; )
	{
		static const double p10[] = { 1e-3, 1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4, 1e5 };
		int k = (int)( pcg_next( r ) % 9u );
		double x = p10[k] * ( 1.0 + 9.0 * urand( r ) );
		if ( x >= lo && x <= hi )
		{
			return x;
		}
	}
}

static double cbrt_newton( double v ) // v > 0; Newton from a coarse seed, + - * / only
{
	double x = v > 1.0 ? v / 3.0 : 1.0;
	for ( int i = 0; i < 100; ++i )
	{
		x = x - ( x * x * x - v ) / ( 3.0 * x * x );
	}
	return x;
}

static void unit_vector( Pcg* r, double out[3] )
{
	for ( ;; )
	{
		double x = urange( r, -1, 1 ), y = urange( r, -1, 1 ), z = urange( r, -1, 1 );
		double l2 = x * x + y * y + z * z;
		if ( l2 > 1e-4 && l2 <= 1.0 )
		{
			double s = 1.0 / sqrt( l2 );
			out[0] = x * s;
			out[1] = y * s;
			out[2] = z * s;
			return;
		}
	}
}

static void unit_quat( Pcg* r, double q[4] )
{
	for ( ;; )
	{
		double a = urange( r, -1, 1 ), b = urange( r, -1, 1 ), c = urange( r, -1, 1 ), d = urange( r, -1, 1 );
		double l2 = a * a + b * b + c * c + d * d;
		if ( l2 > 1e-4 && l2 <= 1.0 )
		{
			double s = 1.0 / sqrt( l2 );
			q[0] = a * s;
			q[1] = b * s;
			q[2] = c * s;
			q[3] = d * s;
			return;
		}
	}
}

static void quat_to_matrix( const double q[4], double m[3][3] ) // q = (x, y, z, s)
{
	double x = q[0], y = q[1], z = q[2], s = q[3];
	m[0][0] = 1 - 2 * ( y * y + z * z );
	m[0][1] = 2 * ( x * y - s * z );
	m[0][2] = 2 * ( x * z + s * y );
	m[1][0] = 2 * ( x * y + s * z );
	m[1][1] = 1 - 2 * ( x * x + z * z );
	m[1][2] = 2 * ( y * z - s * x );
	m[2][0] = 2 * ( x * z - s * y );
	m[2][1] = 2 * ( y * z + s * x );
	m[2][2] = 1 - 2 * ( x * x + y * y );
}

static void mat_vec( double m[3][3], const double v[3], double out[3] )
{
	for ( int i = 0; i < 3; ++i )
	{
		out[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
	}
}

static void cross( const double a[3], const double b[3], double out[3] )
{
	double x = a[1] * b[2] - a[2] * b[1];
	double y = a[2] * b[0] - a[0] * b[2];
	double z = a[0] * b[1] - a[1] * b[0];
	out[0] = x;
	out[1] = y;
	out[2] = z;
}

static double dot( const double a[3], const double b[3] )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

// symmetric 3x3 (xx, xy, xz, yy, yz, zz) times vector
static void sym_vec( const double s[6], const double v[3], double out[3] )
{
	out[0] = s[0] * v[0] + s[1] * v[1] + s[2] * v[2];
	out[1] = s[1] * v[0] + s[3] * v[1] + s[4] * v[2];
	out[2] = s[2] * v[0] + s[4] * v[1] + s[5] * v[2];
}

// ------------------------------------------------------------------------------------------------
// Scenario (double) and prepare (double): like b3PrepareContacts_Convex, once, on the CPU
// ------------------------------------------------------------------------------------------------

typedef struct DBody
{
	double mass, invMass;
	double half[3];
	double invI[6]; // world inverse inertia, symmetric
	double invLocalMax; // largest principal inverse inertia
	double v[3], w[3], p[3], q[4];
} DBody;

typedef struct DContact
{
	int a, b, pointCount;
	double n[3], t1[3], t2[3];
	double rA[2][3], rB[2][3], base[2], normalMass[2];
	double cA[3], cB[3], tm[3]; // friction centres, 2x2 tangent mass (xx, xy, yy)
	double friction;
} DContact;

typedef struct Scenario
{
	int bodyCount; // including the static body 0
	int contactCount;
	DBody* bodies;
	DContact* contacts;
} Scenario;

static void make_scenario( Scenario* s, int n, uint64_t seed )
{
	Pcg r = pcg_seed( seed, 11 );
	int dyn = n / 2 < 8 ? 8 : n / 2;
	s->bodyCount = dyn + 1;
	s->contactCount = n;
	s->bodies = (DBody*)calloc( (size_t)s->bodyCount, sizeof( DBody ) );
	s->contacts = (DContact*)calloc( (size_t)n, sizeof( DContact ) );

	s->bodies[0].q[3] = 1.0; // static body: zero inverse mass and inertia
	for ( int i = 1; i < s->bodyCount; ++i )
	{
		DBody* b = s->bodies + i;
		b->mass = logrand( &r, 0.05, 3000.0 );
		double density = logrand( &r, 150.0, 7800.0 );
		double edge = cbrt_newton( b->mass / density );
		for ( int k = 0; k < 3; ++k )
		{
			double e = edge * urange( &r, 0.6, 1.6 );
			e = e < 0.05 ? 0.05 : ( e > 3.0 ? 3.0 : e );
			b->half[k] = 0.5 * e;
		}
		b->invMass = 1.0 / b->mass;
		double hx2 = b->half[0] * b->half[0], hy2 = b->half[1] * b->half[1], hz2 = b->half[2] * b->half[2];
		double invLocal[3] = { 3.0 / ( b->mass * ( hy2 + hz2 ) ), 3.0 / ( b->mass * ( hx2 + hz2 ) ), 3.0 / ( b->mass * ( hx2 + hy2 ) ) };
		b->invLocalMax = invLocal[0] > invLocal[1] ? invLocal[0] : invLocal[1];
		b->invLocalMax = invLocal[2] > b->invLocalMax ? invLocal[2] : b->invLocalMax;
		unit_quat( &r, b->q );
		double R[3][3];
		quat_to_matrix( b->q, R );
		// R diag(invLocal) R^T
		double m[3][3];
		for ( int i2 = 0; i2 < 3; ++i2 )
		{
			for ( int j = 0; j < 3; ++j )
			{
				m[i2][j] = R[i2][0] * invLocal[0] * R[j][0] + R[i2][1] * invLocal[1] * R[j][1] + R[i2][2] * invLocal[2] * R[j][2];
			}
		}
		b->invI[0] = m[0][0];
		b->invI[1] = m[0][1];
		b->invI[2] = m[0][2];
		b->invI[3] = m[1][1];
		b->invI[4] = m[1][2];
		b->invI[5] = m[2][2];
		double dir[3];
		unit_vector( &r, dir );
		double speed = urange( &r, 0.0, 20.0 );
		for ( int k = 0; k < 3; ++k )
		{
			b->v[k] = speed * dir[k];
		}
		unit_vector( &r, dir );
		double spin = urange( &r, 0.0, 10.0 );
		for ( int k = 0; k < 3; ++k )
		{
			b->w[k] = spin * dir[k];
			b->p[k] = urange( &r, -100.0, 100.0 );
		}
	}

	for ( int k = 0; k < n; ++k )
	{
		DContact* c = s->contacts + k;
		c->a = 1 + (int)( pcg_next( &r ) % (uint32_t)dyn );
		if ( pcg_next( &r ) % 10u == 0 )
		{
			c->b = 0;
		}
		else
		{
			for ( ;; )
			{
				int off = 1 + (int)( pcg_next( &r ) % 64u );
				c->b = 1 + ( c->a - 1 + off ) % dyn;
				if ( c->b != c->a )
				{
					break;
				}
			}
		}
		unit_vector( &r, c->n );
		// b3PerpW then tangent2 = cross( tangent1, normal ), in double
		double p[3];
		if ( c->n[0] < -0.5 || c->n[0] > 0.5 )
		{
			p[0] = c->n[1];
			p[1] = -c->n[0];
			p[2] = 0.0;
		}
		else
		{
			p[0] = 0.0;
			p[1] = c->n[2];
			p[2] = -c->n[1];
		}
		double pl = 1.0 / sqrt( dot( p, p ) );
		for ( int i = 0; i < 3; ++i )
		{
			c->t1[i] = p[i] * pl;
		}
		cross( c->t1, c->n, c->t2 );
		c->pointCount = 1 + (int)( pcg_next( &r ) & 1u );
		c->friction = 0.6;

		DBody* A = s->bodies + c->a;
		DBody* B = s->bodies + c->b;
		double RA[3][3], RB[3][3];
		quat_to_matrix( A->q, RA );
		quat_to_matrix( B->q, RB );
		double wsum = 0.0;
		for ( int i = 0; i < 3; ++i )
		{
			c->cA[i] = 0.0;
			c->cB[i] = 0.0;
		}
		for ( int j = 0; j < c->pointCount; ++j )
		{
			double la[3], lb[3];
			for ( int i = 0; i < 3; ++i )
			{
				la[i] = urange( &r, -0.9, 0.9 ) * A->half[i];
				lb[i] = c->b == 0 ? urange( &r, -1.0, 1.0 ) : urange( &r, -0.9, 0.9 ) * B->half[i];
			}
			mat_vec( RA, la, c->rA[j] );
			if ( c->b == 0 )
			{
				memcpy( c->rB[j], lb, sizeof( lb ) );
			}
			else
			{
				mat_vec( RB, lb, c->rB[j] );
			}
			double sep = urange( &r, -0.02, 0.01 );
			double d[3] = { c->rB[j][0] - c->rA[j][0], c->rB[j][1] - c->rA[j][1], c->rB[j][2] - c->rA[j][2] };
			c->base[j] = sep - dot( d, c->n );

			double rnA[3], rnB[3], t[3];
			cross( c->rA[j], c->n, rnA );
			cross( c->rB[j], c->n, rnB );
			double k = A->invMass + B->invMass;
			sym_vec( A->invI, rnA, t );
			k += dot( rnA, t );
			sym_vec( B->invI, rnB, t );
			k += dot( rnB, t );
			c->normalMass[j] = 1.0 / k;
			// friction weight: 1 for every separation we generate (Box3D clamps 2 - s/tau to [.., 1])
			for ( int i = 0; i < 3; ++i )
			{
				c->cA[i] += c->rA[j][i];
				c->cB[i] += c->rB[j][i];
			}
			wsum += 1.0;
		}
		for ( int i = 0; i < 3; ++i )
		{
			c->cA[i] /= wsum;
			c->cB[i] /= wsum;
		}
		double rtA1[3], rtA2[3], rtB1[3], rtB2[3], iA1[3], iA2[3], iB1[3], iB2[3];
		cross( c->cA, c->t1, rtA1 );
		cross( c->cA, c->t2, rtA2 );
		cross( c->cB, c->t1, rtB1 );
		cross( c->cB, c->t2, rtB2 );
		sym_vec( A->invI, rtA1, iA1 );
		sym_vec( A->invI, rtA2, iA2 );
		sym_vec( B->invI, rtB1, iB1 );
		sym_vec( B->invI, rtB2, iB2 );
		double mAB = A->invMass + B->invMass;
		double kxx = mAB + dot( rtA1, iA1 ) + dot( rtB1, iB1 );
		double kyy = mAB + dot( rtA2, iA2 ) + dot( rtB2, iB2 );
		double kxy = dot( rtA1, iA2 ) + dot( rtB1, iB2 );
		double det = kxx * kyy - kxy * kxy;
		double invDet = 1.0 / det;
		c->tm[0] = invDet * kyy;
		c->tm[1] = -invDet * kxy;
		c->tm[2] = invDet * kxx;
	}
}

static void free_scenario( Scenario* s )
{
	free( s->bodies );
	free( s->contacts );
}

// ------------------------------------------------------------------------------------------------
// Colouring: greedy in contact order, no two contacts of a colour share a dynamic body
// ------------------------------------------------------------------------------------------------

#define MAX_COLORS 64

typedef struct Coloring
{
	int colorCount;
	int start[MAX_COLORS + 1];
	int* order; // sorted position -> scenario contact index
} Coloring;

static void make_coloring( const Scenario* s, Coloring* col )
{
	uint64_t* used = (uint64_t*)calloc( (size_t)s->bodyCount, sizeof( uint64_t ) );
	int* colorOf = (int*)malloc( (size_t)s->contactCount * sizeof( int ) );
	int counts[MAX_COLORS] = { 0 };
	col->colorCount = 0;
	for ( int k = 0; k < s->contactCount; ++k )
	{
		int a = s->contacts[k].a, b = s->contacts[k].b;
		uint64_t mask = used[a] | ( b != 0 ? used[b] : 0 );
		int c = 0;
		while ( c < MAX_COLORS && ( mask & ( 1ULL << c ) ) )
		{
			++c;
		}
		if ( c == MAX_COLORS )
		{
			fprintf( stderr, "out of colours\n" );
			exit( 1 );
		}
		used[a] |= 1ULL << c;
		if ( b != 0 )
		{
			used[b] |= 1ULL << c;
		}
		colorOf[k] = c;
		counts[c] += 1;
		if ( c + 1 > col->colorCount )
		{
			col->colorCount = c + 1;
		}
	}
	col->start[0] = 0;
	for ( int c = 0; c < col->colorCount; ++c )
	{
		col->start[c + 1] = col->start[c] + counts[c];
	}
	int fill[MAX_COLORS];
	memcpy( fill, col->start, sizeof( fill ) );
	col->order = (int*)malloc( (size_t)s->contactCount * sizeof( int ) );
	for ( int k = 0; k < s->contactCount; ++k )
	{
		col->order[fill[colorOf[k]]++] = k;
	}
	free( used );
	free( colorOf );
}

// ------------------------------------------------------------------------------------------------
// Quantisation to the variant's format
// ------------------------------------------------------------------------------------------------

typedef struct Data
{
	int bodyCount, contactCount;
	Body* bodies;
	Pose* poses;
	Constraint* cons;
	Impulse* imps;
	Params params;
	int rangeErrors; // quantisation values that did not fit (I32)
	int shiftErrors; // I32 shifts < 1
} Data;

static int g_rangeErrors;

static int64_t round_scaled( double x, int s )
{
	return (int64_t)floor( ldexp( x, s ) + 0.5 );
}

#if defined( VAR_F )
static T QT( double x, int s )
{
	(void)s;
	return (T)x;
}
static TP QP( double x )
{
	return (TP)x;
}
#elif defined( VAR_I64 )
static T QT( double x, int s )
{
	(void)s;
	if ( fabs( x ) >= 2147483648.0 )
	{
		g_rangeErrors += 1;
	}
	return round_scaled( x, 32 );
}
static TP QP( double x )
{
	return round_scaled( x, 32 );
}
#else // VAR_I32, VAR_V4
static T QT( double x, int s )
{
	int64_t v = round_scaled( x, s );
	if ( v > 2147483647LL || v < -2147483647LL )
	{
		g_rangeErrors += 1;
		v = v > 0 ? 2147483647LL : -2147483647LL;
	}
	return (T)v;
}
static TP QP( double x )
{
	return round_scaled( x, 32 );
}
#endif

// exponent e so that maxAbs * 2^e < 2^bits (I32 mantissas)
static int exp_for( double maxAbs, int bits )
{
	if ( maxAbs <= 0.0 )
	{
		return 40;
	}
	int k;
	frexp( maxAbs, &k ); // maxAbs = m 2^k, m in [0.5, 1)
	return bits - k;
}

static V3 qv3( const double v[3], int s )
{
	V3 r = { QT( v[0], s ), QT( v[1], s ), QT( v[2], s ) };
	return r;
}

static Sym3 qsym( const double m[6], int s )
{
	Sym3 r = { QT( m[0], s ), QT( m[1], s ), QT( m[2], s ), QT( m[3], s ), QT( m[4], s ), QT( m[5], s ) };
	return r;
}

struct Data;
static int clamp_shift( int sh, struct Data* d );

typedef struct Softness
{
	double biasRate, massScale, impulseScale;
} Softness;

static Softness make_soft_d( double hertz, double zeta, double h )
{
	double omega = 2.0 * 3.14159265358979323846 * hertz;
	double a1 = 2.0 * zeta + h * omega;
	double a2 = h * omega * a1;
	double a3 = 1.0 / ( 1.0 + a2 );
	Softness s = { omega / a1, a2 * a3, a3 };
	return s;
}

static int clamp_shift( int sh, Data* d )
{
	if ( sh < 1 || sh > 63 )
	{
		d->shiftErrors += 1;
		return sh < 1 ? 1 : 63;
	}
	return sh;
}

static void quantize( const Scenario* s, const Coloring* col, Data* d )
{
	g_rangeErrors = 0;
	d->shiftErrors = 0;
	d->bodyCount = s->bodyCount;
	d->contactCount = s->contactCount;
	d->bodies = (Body*)calloc( (size_t)s->bodyCount, sizeof( Body ) );
	d->poses = (Pose*)calloc( (size_t)s->bodyCount, sizeof( Pose ) );
	d->cons = (Constraint*)calloc( (size_t)s->contactCount, sizeof( Constraint ) );
	d->imps = (Impulse*)calloc( (size_t)s->contactCount, sizeof( Impulse ) );

	int* eM = (int*)malloc( (size_t)s->bodyCount * sizeof( int ) );
	int* eI = (int*)malloc( (size_t)s->bodyCount * sizeof( int ) );
	for ( int i = 0; i < s->bodyCount; ++i )
	{
		const DBody* b = s->bodies + i;
		double mi = 0.0;
		for ( int k = 0; k < 6; ++k )
		{
			mi = fabs( b->invI[k] ) > mi ? fabs( b->invI[k] ) : mi;
		}
#if defined( VAR_V4 )
		eM[i] = exp_for( b->invMass, 31 );	  // mantissa in [2^30, 2^31)
		eI[i] = exp_for( b->invLocalMax, 29 ); // largest local entry in [2^28, 2^29); world entries fit
#else
		eM[i] = exp_for( b->invMass, 30 );
		eI[i] = exp_for( mi, 30 );
#endif
		Body* B = d->bodies + i;
		B->v = qv3( b->v, S_V );
		B->w = qv3( b->w, S_W );
		double one = 1.0;
		B->dq.s = QT( one, S_Q );
		Pose* P = d->poses + i;
		P->px = QP( b->p[0] );
		P->py = QP( b->p[1] );
		P->pz = QP( b->p[2] );
		P->q.x = QT( b->q[0], S_Q );
		P->q.y = QT( b->q[1], S_Q );
		P->q.z = QT( b->q[2], S_Q );
		P->q.s = QT( b->q[3], S_Q );
	}

	for ( int k = 0; k < s->contactCount; ++k )
	{
		const DContact* c = s->contacts + col->order[k];
		const DBody* A = s->bodies + c->a;
		const DBody* B = s->bodies + c->b;
		Constraint* C = d->cons + k;
		C->indexA = c->a;
		C->indexB = c->b;
		C->pointCount = c->pointCount;
		C->soft = ( c->a == 0 || c->b == 0 ) ? 1 : 0;

		double mmax = 0.0, nmax = 0.0, tmax = 0.0;
		for ( int j = 0; j < c->pointCount; ++j )
		{
			nmax = c->normalMass[j] > nmax ? c->normalMass[j] : nmax;
		}
		for ( int j = 0; j < 3; ++j )
		{
			tmax = fabs( c->tm[j] ) > tmax ? fabs( c->tm[j] ) : tmax;
		}
		mmax = nmax > tmax ? nmax : tmax;
#if defined( VAR_V4 )
		// e_P = e_n + 10 with the manifold's largest normal mass (T10 section 1.1)
		int eNp[2] = { 0, 0 };
		for ( int j = 0; j < c->pointCount; ++j )
		{
			eNp[j] = exp_for( c->normalMass[j], 31 );
		}
		int eC = exp_for( nmax, 31 ) - 10;
		C->eN = exp_for( nmax, 31 );
		C->eT = exp_for( tmax, 30 );
#else
		// impulse scale: an impulse of (largest effective mass) x 100 m/s uses 28 bits
		int eC = exp_for( mmax * 100.0, 28 );
		C->eN = exp_for( nmax, 30 );
		C->eT = exp_for( tmax, 30 );
#endif
		C->eMA = eM[c->a];
		C->eMB = eM[c->b];
		C->eIA = eI[c->a];
		C->eIB = eI[c->b];
		C->eC = eC;
#if defined( VAR_I32 )
		int shifts[6] = { C->eIA + eC - S_W, C->eMA + eC - S_V, C->eIB + eC - S_W, C->eMB + eC - S_V, C->eN + S_V - eC, C->eT + S_V - eC };
		for ( int j = 0; j < 6; ++j )
		{
			if ( shifts[j] < 1 || shifts[j] > 62 )
			{
				d->shiftErrors += 1;
			}
		}
#endif
		C->normal = qv3( c->n, S_Q );
		C->tangent1 = qv3( c->t1, S_Q );
		C->tangent2 = qv3( c->t2, S_Q );
		C->friction = QT( c->friction, S_MS );
		C->invMassA = QT( A->invMass, C->eMA );
		C->invMassB = QT( B->invMass, C->eMB );
		C->invIA = qsym( A->invI, C->eIA );
		C->invIB = qsym( B->invI, C->eIB );
		C->tangentMass.xx = QT( c->tm[0], C->eT );
		C->tangentMass.xy = QT( c->tm[1], C->eT );
		C->tangentMass.yy = QT( c->tm[2], C->eT );
		C->centerA = qv3( c->cA, S_R );
		C->centerB = qv3( c->cB, S_R );
		for ( int j = 0; j < c->pointCount; ++j )
		{
			C->points[j].anchorA = qv3( c->rA[j], S_R );
			C->points[j].anchorB = qv3( c->rB[j], S_R );
			C->points[j].baseSeparation = QT( c->base[j], S_S );
#if defined( VAR_V4 )
			C->points[j].normalMass = QT( c->normalMass[j], eNp[j] );
			C->points[j].shN = clamp_shift( eNp[j] + S_V - eC, d );
#else
			C->points[j].normalMass = QT( c->normalMass[j], C->eN );
#endif
		}
#if defined( VAR_V4 )
		// precomputed shifts replace the exponents (the kernel reads them directly)
		C->eMA = clamp_shift( C->eMA + eC - S_V, d );
		C->eMB = clamp_shift( C->eMB + eC - S_V, d );
		C->eIA = clamp_shift( C->eIA + eC - S_W, d );
		C->eIB = clamp_shift( C->eIB + eC - S_W, d );
		C->eT = clamp_shift( C->eT + S_V - eC, d );
#endif
	}
	free( eM );
	free( eI );

	// Params. F follows Box3D in float (b3MakeSoft at h = 1/240); the integer formats round the
	// double values.
	Params* P = &d->params;
	memset( P, 0, sizeof( *P ) );
#if defined( VAR_F )
	{
		float h = ( 1.0f / 60.0f ) / 4.0f;
		float inv_h = 240.0f;
		float pi = 3.14159265359f;
		float hz[2] = { 30.0f, 60.0f }, zeta[2] = { 10.0f, 5.0f };
		float br[2], ms[2], is[2];
		for ( int i = 0; i < 2; ++i )
		{
			float omega = 2.0f * pi * hz[i];
			float a1 = 2.0f * zeta[i] + h * omega;
			float a2 = h * omega * a1;
			float a3 = 1.0f / ( 1.0f + a2 );
			br[i] = omega / a1;
			ms[i] = a2 * a3;
			is[i] = a3;
		}
		P->h = h;
		P->inv_h = inv_h;
		P->gravityDelta.x = h * 0.0f;
		P->gravityDelta.y = h * -10.0f;
		P->gravityDelta.z = h * 0.0f;
		P->dynBiasRate = ms[0] * br[0];
		P->dynMassScale = ms[0];
		P->dynImpulseScale = is[0];
		P->staBiasRate = ms[1] * br[1];
		P->staMassScale = ms[1];
		P->staImpulseScale = is[1];
		P->negContactSpeed = -3.0f;
		P->one = 1.0f;
		P->half = 0.5f;
		P->oneHalf = 1.5f;
		P->invMaxAngularSpeed = (float)( 1.0 / ( 0.25 * 3.14159265358979323846 * 60.0 ) );
		P->oneU = 1.0f;
	}
#else
	{
		double h = 1.0 / 240.0;
		Softness dy = make_soft_d( 30.0, 10.0, h ), st = make_soft_d( 60.0, 5.0, h );
		P->h = QT( h, S_H );
		P->inv_h = QT( 240.0, S_IH );
		P->gravityDelta.x = 0;
		P->gravityDelta.y = QT( -10.0 * h, S_V );
		P->gravityDelta.z = 0;
		P->dynBiasRate = QT( dy.massScale * dy.biasRate, S_BR );
		P->dynMassScale = QT( dy.massScale, S_MS );
		P->dynImpulseScale = QT( dy.impulseScale, S_MS );
		P->staBiasRate = QT( st.massScale * st.biasRate, S_BR );
		P->staMassScale = QT( st.massScale, S_MS );
		P->staImpulseScale = QT( st.impulseScale, S_MS );
		P->negContactSpeed = QT( -3.0, S_V );
		P->one = QT( 1.0, S_MS );
		P->half = QT( 0.5, S_MS );
		P->oneHalf = QT( 1.5, S_MS );
		P->invMaxAngularSpeed = QT( 1.0 / ( 0.25 * 3.14159265358979323846 * 60.0 ), S_MS );
		P->oneU = QT( 1.0, S_U );
	}
#endif
	d->rangeErrors = g_rangeErrors;
}

static void free_data( Data* d )
{
	free( d->bodies );
	free( d->poses );
	free( d->cons );
	free( d->imps );
}

// ------------------------------------------------------------------------------------------------
// The dispatch list for one step: 4 substeps of (integrate velocities, warm start, solve,
// integrate positions, relax), then finalize. Colours are separate dispatches.
// ------------------------------------------------------------------------------------------------

typedef struct Dispatch
{
	int entry;
	uint32_t start, count;
} Dispatch;

static int make_dispatches( const Coloring* col, int bodyCount, Dispatch* out )
{
	int n = 0;
	uint32_t dyn = (uint32_t)( bodyCount - 1 );
	for ( int sub = 0; sub < 4; ++sub )
	{
		out[n++] = ( Dispatch ){ E_INTVEL, 1, dyn };
		for ( int c = 0; c < col->colorCount; ++c )
		{
			out[n++] = ( Dispatch ){ E_WARM, (uint32_t)col->start[c], (uint32_t)( col->start[c + 1] - col->start[c] ) };
		}
		for ( int c = 0; c < col->colorCount; ++c )
		{
			out[n++] = ( Dispatch ){ E_PUSH, (uint32_t)col->start[c], (uint32_t)( col->start[c + 1] - col->start[c] ) };
		}
		out[n++] = ( Dispatch ){ E_INTPOS, 1, dyn };
		for ( int c = 0; c < col->colorCount; ++c )
		{
			out[n++] = ( Dispatch ){ E_RELAX, (uint32_t)col->start[c], (uint32_t)( col->start[c + 1] - col->start[c] ) };
		}
	}
	out[n++] = ( Dispatch ){ E_FINAL, 1, dyn };
	return n;
}

// ------------------------------------------------------------------------------------------------
// Hashing and comparison
// ------------------------------------------------------------------------------------------------

static uint64_t fnv( uint64_t h, const void* p, size_t n )
{
	const uint8_t* b = (const uint8_t*)p;
	for ( size_t i = 0; i < n; ++i )
	{
		h ^= b[i];
		h *= 1099511628211ULL;
	}
	return h;
}

static uint64_t hash_state( const Body* b, const Pose* p, int bodyCount )
{
	uint64_t h = 1469598103934665603ULL;
	h = fnv( h, b, (size_t)bodyCount * sizeof( Body ) );
	h = fnv( h, p, (size_t)bodyCount * sizeof( Pose ) );
	return h;
}

typedef struct Snapshot
{
	Body* bodies;
	Pose* poses;
	Impulse* imps;
} Snapshot;

static void snap_alloc( Snapshot* s, const Data* d )
{
	s->bodies = (Body*)malloc( (size_t)d->bodyCount * sizeof( Body ) );
	s->poses = (Pose*)malloc( (size_t)d->bodyCount * sizeof( Pose ) );
	s->imps = (Impulse*)malloc( (size_t)d->contactCount * sizeof( Impulse ) );
}

static void snap_free( Snapshot* s )
{
	free( s->bodies );
	free( s->poses );
	free( s->imps );
}

enum
{
	K_F32 = 0,
	K_I32 = 1,
	K_I64 = 2
};

typedef struct Compare
{
	long long words, mismatches;
	long long signedZero, subnormal, nan, ulp1, ulpSmall, larger; // F32; ints use ulp1 (1 lsb) and larger
	double maxUlp;
	int dumps;
	char dump[8][200];
} Compare;

static int is_subnormal( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	return ( u & 0x7f800000u ) == 0 && ( u & 0x007fffffu ) != 0;
}

static int64_t ordered( float f )
{
	int32_t i;
	memcpy( &i, &f, 4 );
	return i < 0 ? (int64_t)( (int32_t)0x80000000 ) - (int64_t)i : (int64_t)i;
}

static void compare_word( Compare* c, int kind, const void* a, const void* b, const char* buf, int elem, int field )
{
	c->words += 1;
	int size = kind == K_I64 ? 8 : 4;
	if ( memcmp( a, b, (size_t)size ) == 0 )
	{
		return;
	}
	c->mismatches += 1;
	char val[120];
	double diff = 0.0;
	if ( kind == K_F32 )
	{
		float x, y;
		memcpy( &x, a, 4 );
		memcpy( &y, b, 4 );
		diff = (double)llabs( ordered( x ) - ordered( y ) );
		if ( x != x || y != y )
		{
			c->nan += 1;
		}
		else if ( x == y )
		{
			c->signedZero += 1;
		}
		else if ( is_subnormal( x ) || is_subnormal( y ) )
		{
			c->subnormal += 1;
		}
		else if ( diff <= 1.0 )
		{
			c->ulp1 += 1;
		}
		else if ( diff <= 16.0 )
		{
			c->ulpSmall += 1;
		}
		else
		{
			c->larger += 1;
		}
		snprintf( val, sizeof( val ), "%.9g (0x%08x) vs %.9g (0x%08x), %.0f ulp", (double)x, *(const uint32_t*)a, (double)y,
				  *(const uint32_t*)b, diff );
	}
	else
	{
		int64_t x, y;
		if ( kind == K_I32 )
		{
			x = *(const int32_t*)a;
			y = *(const int32_t*)b;
		}
		else
		{
			x = *(const int64_t*)a;
			y = *(const int64_t*)b;
		}
		diff = (double)llabs( x - y );
		if ( diff <= 1.0 )
		{
			c->ulp1 += 1;
		}
		else
		{
			c->larger += 1;
		}
		snprintf( val, sizeof( val ), "%lld vs %lld, %.0f lsb", (long long)x, (long long)y, diff );
	}
	c->maxUlp = diff > c->maxUlp ? diff : c->maxUlp;
	if ( c->dumps < 8 )
	{
		snprintf( c->dump[c->dumps++], 200, "%s[%d].word%d: %s", buf, elem, field, val );
	}
}

static const int KT = sizeof( T ) == 8 ? K_I64 : ( sizeof( T ) == 4 && ( (T)0.5 != 0 ) ? K_F32 : K_I32 );
static const int KP = sizeof( TP ) == 8 ? K_I64 : ( ( (TP)0.5 != 0 ) ? K_F32 : K_I32 );

static Compare compare_snapshots( const Snapshot* x, const Snapshot* y, const Data* d )
{
	Compare c;
	memset( &c, 0, sizeof( c ) );
	int nb = (int)( sizeof( Body ) / sizeof( T ) );
	for ( int i = 0; i < d->bodyCount; ++i )
	{
		const T* a = (const T*)( x->bodies + i );
		const T* b = (const T*)( y->bodies + i );
		for ( int f = 0; f < nb; ++f )
		{
			compare_word( &c, KT, a + f, b + f, "body", i, f );
		}
		compare_word( &c, KP, &x->poses[i].px, &y->poses[i].px, "pose.p", i, 0 );
		compare_word( &c, KP, &x->poses[i].py, &y->poses[i].py, "pose.p", i, 1 );
		compare_word( &c, KP, &x->poses[i].pz, &y->poses[i].pz, "pose.p", i, 2 );
		const T* qa = &x->poses[i].q.x;
		const T* qb = &y->poses[i].q.x;
		for ( int f = 0; f < 4; ++f )
		{
			compare_word( &c, KT, qa + f, qb + f, "pose.q", i, f );
		}
	}
	int ni = (int)( sizeof( Impulse ) / sizeof( T ) );
	for ( int i = 0; i < d->contactCount; ++i )
	{
		const T* a = (const T*)( x->imps + i );
		const T* b = (const T*)( y->imps + i );
		for ( int f = 0; f < ni; ++f )
		{
			compare_word( &c, KT, a + f, b + f, "impulse", i, f );
		}
	}
	return c;
}

static void print_compare( FILE* log, const char* label, const Compare* c )
{
	fprintf( log, "  compare %-26s words %lld mismatches %lld", label, c->words, c->mismatches );
	if ( c->mismatches > 0 )
	{
		if ( KT == K_F32 )
		{
			fprintf( log, " (signed-zero %lld, subnormal %lld, nan %lld, 1ulp %lld, 2-16ulp %lld, larger %lld, max %.0f ulp)", c->signedZero,
					 c->subnormal, c->nan, c->ulp1, c->ulpSmall, c->larger, c->maxUlp );
		}
		else
		{
			fprintf( log, " (1lsb %lld, larger %lld, max %.0f lsb)", c->ulp1, c->larger, c->maxUlp );
		}
	}
	fprintf( log, "\n" );
	for ( int i = 0; i < c->dumps; ++i )
	{
		fprintf( log, "    first mismatches: %s\n", c->dump[i] );
	}
}

// Subnormal census of the final float state (F only).
static void subnormal_census( FILE* log, const Snapshot* s, const Data* d )
{
	if ( KT != K_F32 )
	{
		return;
	}
	long long n = 0, zeros = 0, total = 0;
	const float* f = (const float*)s->bodies;
	size_t cnt = (size_t)d->bodyCount * sizeof( Body ) / 4;
	for ( size_t i = 0; i < cnt; ++i, ++total )
	{
		n += is_subnormal( f[i] );
		zeros += f[i] == 0.0f;
	}
	f = (const float*)s->imps;
	cnt = (size_t)d->contactCount * sizeof( Impulse ) / 4;
	for ( size_t i = 0; i < cnt; ++i, ++total )
	{
		n += is_subnormal( f[i] );
		zeros += f[i] == 0.0f;
	}
	fprintf( log, "  census: %lld subnormal and %lld zero words of %lld in the final CPU state\n", n, zeros, total );
}

// ------------------------------------------------------------------------------------------------
// CPU twin runner: 1 thread, or a spin pool with one barrier per dispatch
// ------------------------------------------------------------------------------------------------

// The pool's shared words: a generation the workers wait on, a count of finished workers, quit.
// Loads acquire and increments release, so a worker that sees a new generation sees its job, and the
// caller that sees every worker done sees their writes (x86 ordered these for free; ARM does not).
#if defined( _WIN32 )
typedef volatile LONG PoolWord; // MSVC's volatile reads acquire and writes release on x64 (/volatile:ms)
#define POOL_LOAD( w ) ( *( w ) )
#define POOL_STORE( w, v ) InterlockedExchange( ( w ), ( v ) )
#define POOL_INC( w ) InterlockedIncrement( w )
#define POOL_PAUSE() YieldProcessor()
typedef HANDLE PoolThread;
#else
typedef atomic_long PoolWord;
#define POOL_LOAD( w ) atomic_load_explicit( ( w ), memory_order_acquire )
#define POOL_STORE( w, v ) atomic_store_explicit( ( w ), ( v ), memory_order_release )
#define POOL_INC( w ) atomic_fetch_add_explicit( ( w ), 1, memory_order_acq_rel )
#define POOL_PAUSE() sched_yield()
typedef pthread_t PoolThread;
#endif

typedef struct Pool
{
	int threads;
	PoolThread handles[64];
	PoolWord generation;
	PoolWord done;
	PoolWord quit;
	Dispatch job;
} Pool;

static Pool g_pool;

static void run_chunk( const Dispatch* d, int t, int threads )
{
	uint32_t groups = ( d->count + 63 ) / 64;
	uint32_t g0 = (uint32_t)( ( (uint64_t)groups * (uint64_t)t ) / (uint64_t)threads );
	uint32_t g1 = (uint32_t)( ( (uint64_t)groups * (uint64_t)( t + 1 ) ) / (uint64_t)threads );
	if ( g1 > g0 )
	{
		twin_run( d->entry, d->start, d->count, g0, g1 );
	}
}

static void pool_work( int t )
{
	long seen = 0;
	for ( ;; )
	{
		long g;
		while ( ( g = POOL_LOAD( &g_pool.generation ) ) == seen )
		{
			if ( POOL_LOAD( &g_pool.quit ) )
			{
				return;
			}
			POOL_PAUSE();
		}
		seen = g;
		Dispatch d = g_pool.job;
		run_chunk( &d, t, g_pool.threads );
		POOL_INC( &g_pool.done );
	}
}

#if defined( _WIN32 )
static DWORD WINAPI pool_worker( LPVOID arg )
{
	pool_work( (int)(intptr_t)arg );
	return 0;
}
#else
static void* pool_worker( void* arg )
{
	pool_work( (int)(intptr_t)arg );
	return NULL;
}
#endif

static void pool_start( int threads )
{
	memset( &g_pool, 0, sizeof( g_pool ) );
	g_pool.threads = threads;
	for ( int t = 1; t < threads; ++t )
	{
#if defined( _WIN32 )
		g_pool.handles[t] = CreateThread( NULL, 0, pool_worker, (LPVOID)(intptr_t)t, 0, NULL );
#else
		pthread_create( &g_pool.handles[t], NULL, pool_worker, (void*)(intptr_t)t );
#endif
	}
}

static void pool_stop( void )
{
	POOL_STORE( &g_pool.quit, 1 );
	for ( int t = 1; t < g_pool.threads; ++t )
	{
#if defined( _WIN32 )
		WaitForSingleObject( g_pool.handles[t], INFINITE );
		CloseHandle( g_pool.handles[t] );
#else
		pthread_join( g_pool.handles[t], NULL );
#endif
	}
}

static void pool_dispatch( const Dispatch* d )
{
	g_pool.job = *d;
	POOL_STORE( &g_pool.done, 0 );
	POOL_INC( &g_pool.generation ); // releases the job and the reset count
	run_chunk( d, 0, g_pool.threads );
	while ( POOL_LOAD( &g_pool.done ) < g_pool.threads - 1 )
	{
		POOL_PAUSE();
	}
}

typedef struct CpuResult
{
	double msPerStep;
	uint64_t* stepHash;
	Snapshot final;
	uint32_t counters[4];
} CpuResult;

static CpuResult run_cpu( const Data* d, const Dispatch* list, int dispatchCount, int steps, int threads )
{
	CpuResult r;
	memset( &r, 0, sizeof( r ) );
	snap_alloc( &r.final, d );
	memcpy( r.final.bodies, d->bodies, (size_t)d->bodyCount * sizeof( Body ) );
	memcpy( r.final.poses, d->poses, (size_t)d->bodyCount * sizeof( Pose ) );
	memcpy( r.final.imps, d->imps, (size_t)d->contactCount * sizeof( Impulse ) );
	Params params = d->params;
	twin_bind( r.final.bodies, (size_t)d->bodyCount, r.final.poses, (size_t)d->bodyCount, d->cons, (size_t)d->contactCount, r.final.imps,
			   (size_t)d->contactCount, &params, r.counters );
	r.stepHash = (uint64_t*)malloc( (size_t)steps * sizeof( uint64_t ) );
	if ( threads > 1 )
	{
		pool_start( threads );
	}
	double total = 0.0;
	for ( int s = 0; s < steps; ++s )
	{
		double t0 = vku_now_ms();
		for ( int i = 0; i < dispatchCount; ++i )
		{
			if ( threads > 1 )
			{
				pool_dispatch( list + i );
			}
			else
			{
				run_chunk( list + i, 0, 1 );
			}
		}
		total += vku_now_ms() - t0;
		r.stepHash[s] = hash_state( r.final.bodies, r.final.poses, d->bodyCount );
	}
	if ( threads > 1 )
	{
		pool_stop();
	}
	r.msPerStep = total / steps;
	return r;
}

// ------------------------------------------------------------------------------------------------
// GPU runner
// ------------------------------------------------------------------------------------------------

typedef struct GpuPipes
{
	VkDescriptorSetLayout dsl;
	VkPipelineLayout layout;
	VkPipeline pipes[E_COUNT];
	char spvTag[32];
} GpuPipes;

static void make_pipes( VkGpu* g, GpuPipes* p, const char* spvDir, const char* tag, FILE* log )
{
	VkDescriptorSetLayoutBinding b[6];
	for ( int i = 0; i < 6; ++i )
	{
		b[i] = ( VkDescriptorSetLayoutBinding ){ (uint32_t)i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL };
	}
	VkDescriptorSetLayoutCreateInfo dci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
	dci.bindingCount = 6;
	dci.pBindings = b;
	VK_CHECK( vkCreateDescriptorSetLayout( g->device, &dci, NULL, &p->dsl ) );
	VkPushConstantRange pcr = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( Push ) };
	VkPipelineLayoutCreateInfo lci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	lci.setLayoutCount = 1;
	lci.pSetLayouts = &p->dsl;
	lci.pushConstantRangeCount = 1;
	lci.pPushConstantRanges = &pcr;
	VK_CHECK( vkCreatePipelineLayout( g->device, &lci, NULL, &p->layout ) );
	snprintf( p->spvTag, sizeof( p->spvTag ), "%s", tag );
	for ( int e = 0; e < E_COUNT; ++e )
	{
		char path[512];
		snprintf( path, sizeof( path ), "%s/%s%s.spv", spvDir, g_entryNames[e], tag );
		p->pipes[e] = vku_pipeline( g, p->layout, path, "main" ); // slangc names the SPIR-V entry point main
		if ( p->pipes[e] == VK_NULL_HANDLE )
		{
			exit( 1 );
		}
	}
	vku_print_pipeline_stats( g, p->pipes[E_PUSH], "push", log );
	vku_print_pipeline_stats( g, p->pipes[E_RELAX], "relax", log );
	vku_print_pipeline_stats( g, p->pipes[E_WARM], "warmStart", log );
}

static void free_pipes( VkGpu* g, GpuPipes* p )
{
	for ( int e = 0; e < E_COUNT; ++e )
	{
		vkDestroyPipeline( g->device, p->pipes[e], NULL );
	}
	vkDestroyPipelineLayout( g->device, p->layout, NULL );
	vkDestroyDescriptorSetLayout( g->device, p->dsl, NULL );
}

typedef struct GpuRun
{
	VkBuf buf[6]; // bodies, poses, cons, imps, params, counters
	VkBuf staging;
	VkDescriptorPool pool;
	VkDescriptorSet set;
	VkQueryPool query;
	size_t sizes[6];
} GpuRun;

static void gpu_setup( VkGpu* g, GpuPipes* p, GpuRun* r, const Data* d, int steps )
{
	memset( r, 0, sizeof( *r ) );
	r->sizes[0] = (size_t)d->bodyCount * sizeof( Body );
	r->sizes[1] = (size_t)d->bodyCount * sizeof( Pose );
	r->sizes[2] = (size_t)d->contactCount * sizeof( Constraint );
	r->sizes[3] = (size_t)d->contactCount * sizeof( Impulse );
	r->sizes[4] = sizeof( Params );
	r->sizes[5] = 16;
	size_t total = 0;
	for ( int i = 0; i < 6; ++i )
	{
		r->buf[i] = vku_buffer( g, r->sizes[i], false );
		total += r->sizes[i];
	}
	r->staging = vku_buffer( g, total, true );

	VkDescriptorPoolSize ps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 6 };
	VkDescriptorPoolCreateInfo pci = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
	pci.maxSets = 1;
	pci.poolSizeCount = 1;
	pci.pPoolSizes = &ps;
	VK_CHECK( vkCreateDescriptorPool( g->device, &pci, NULL, &r->pool ) );
	VkDescriptorSetAllocateInfo ai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	ai.descriptorPool = r->pool;
	ai.descriptorSetCount = 1;
	ai.pSetLayouts = &p->dsl;
	VK_CHECK( vkAllocateDescriptorSets( g->device, &ai, &r->set ) );
	VkDescriptorBufferInfo bi[6];
	VkWriteDescriptorSet w[6];
	for ( int i = 0; i < 6; ++i )
	{
		bi[i] = ( VkDescriptorBufferInfo ){ r->buf[i].buffer, 0, VK_WHOLE_SIZE };
		w[i] = ( VkWriteDescriptorSet ){ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
		w[i].dstSet = r->set;
		w[i].dstBinding = (uint32_t)i;
		w[i].descriptorCount = 1;
		w[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		w[i].pBufferInfo = &bi[i];
	}
	vkUpdateDescriptorSets( g->device, 6, w, 0, NULL );

	VkQueryPoolCreateInfo qci = { VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
	qci.queryType = VK_QUERY_TYPE_TIMESTAMP;
	qci.queryCount = (uint32_t)( steps + 1 );
	VK_CHECK( vkCreateQueryPool( g->device, &qci, NULL, &r->query ) );
}

static void gpu_teardown( VkGpu* g, GpuRun* r )
{
	vkDestroyQueryPool( g->device, r->query, NULL );
	vkDestroyDescriptorPool( g->device, r->pool, NULL );
	for ( int i = 0; i < 6; ++i )
	{
		vku_free( g, &r->buf[i] );
	}
	vku_free( g, &r->staging );
}

static void gpu_upload( VkGpu* g, GpuRun* r, const Data* d )
{
	const void* src[6] = { d->bodies, d->poses, d->cons, d->imps, &d->params, NULL };
	uint8_t* st = (uint8_t*)r->staging.mapped;
	size_t off = 0;
	VkCommandBuffer cb = vku_begin( g );
	for ( int i = 0; i < 6; ++i )
	{
		if ( src[i] )
		{
			memcpy( st + off, src[i], r->sizes[i] );
		}
		else
		{
			memset( st + off, 0, r->sizes[i] );
		}
		VkBufferCopy c = { off, 0, r->sizes[i] };
		vkCmdCopyBuffer( cb, r->staging.buffer, r->buf[i].buffer, 1, &c );
		off += r->sizes[i];
	}
	vku_submit_wait( g, cb );
}

// Copies buffers [first, first+count) to staging (at their packed offsets).
static void record_download( GpuRun* r, VkCommandBuffer cb, int first, int count )
{
	size_t off = 0;
	for ( int i = 0; i < 6; ++i )
	{
		if ( i >= first && i < first + count )
		{
			VkBufferCopy c = { 0, off, r->sizes[i] };
			vkCmdCopyBuffer( cb, r->buf[i].buffer, r->staging.buffer, 1, &c );
		}
		off += r->sizes[i];
	}
}

static void gpu_download( VkGpu* g, GpuRun* r, Snapshot* s, uint32_t counters[4] )
{
	VkCommandBuffer cb = vku_begin( g );
	vku_barrier( cb );
	record_download( r, cb, 0, 6 );
	vku_submit_wait( g, cb );
	uint8_t* st = (uint8_t*)r->staging.mapped;
	memcpy( s->bodies, st, r->sizes[0] );
	memcpy( s->poses, st + r->sizes[0], r->sizes[1] );
	size_t off = r->sizes[0] + r->sizes[1] + r->sizes[2];
	memcpy( s->imps, st + off, r->sizes[3] );
	off += r->sizes[3] + r->sizes[4];
	memcpy( counters, st + off, 16 );
}

static void record_step( GpuPipes* p, GpuRun* r, VkCommandBuffer cb, const Dispatch* list, int dispatchCount )
{
	vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_COMPUTE, p->layout, 0, 1, &r->set, 0, NULL );
	int bound = -1;
	for ( int i = 0; i < dispatchCount; ++i )
	{
		const Dispatch* d = list + i;
		if ( d->entry != bound )
		{
			vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE, p->pipes[d->entry] );
			bound = d->entry;
		}
		Push push = { d->start, d->count, 0, 0 };
		vkCmdPushConstants( cb, p->layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( push ), &push );
		vkCmdDispatch( cb, ( d->count + 63 ) / 64, 1, 1 );
		vku_barrier( cb );
	}
}

typedef struct GpuResult
{
	double msPerStep;	  // mean of timestamp deltas, median over reps
	double msPerStepMin;  // fastest step seen
	double msReadback;	  // wall time per step with a per-step submit, wait and readback of bodies+poses
	int firstDivergence;  // first step whose hash differs from the CPU twin (readback mode), -1 if none
	Snapshot final;
	uint32_t counters[4];
} GpuResult;

static int cmp_double( const void* a, const void* b )
{
	double x = *(const double*)a, y = *(const double*)b;
	return x < y ? -1 : x > y ? 1 : 0;
}

static GpuResult run_gpu( VkGpu* g, GpuPipes* p, const Data* d, const Dispatch* list, int dispatchCount, int steps, int reps,
						  const uint64_t* cpuHash )
{
	GpuResult res;
	memset( &res, 0, sizeof( res ) );
	snap_alloc( &res.final, d );
	GpuRun r;
	gpu_setup( g, p, &r, d, steps );

	// Batched: all steps in one command buffer, timestamps between steps. Rep 0 is a warm-up.
	double* repMeans = (double*)malloc( (size_t)( reps + 1 ) * sizeof( double ) );
	double best = 1e30;
	uint64_t* ts = (uint64_t*)malloc( (size_t)( steps + 1 ) * sizeof( uint64_t ) );
	for ( int rep = 0; rep <= reps; ++rep )
	{
		gpu_upload( g, &r, d );
		VkCommandBuffer cb = vku_begin( g );
		vkCmdResetQueryPool( cb, r.query, 0, (uint32_t)( steps + 1 ) );
		vku_barrier( cb );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, r.query, 0 );
		for ( int s = 0; s < steps; ++s )
		{
			record_step( p, &r, cb, list, dispatchCount );
			vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, r.query, (uint32_t)( s + 1 ) );
		}
		vku_submit_wait( g, cb );
		VK_CHECK( vkGetQueryPoolResults( g->device, r.query, 0, (uint32_t)( steps + 1 ), (size_t)( steps + 1 ) * 8, ts, 8,
										 VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
		uint64_t mask = g->timestampValidBits >= 64 ? ~0ULL : ( ( 1ULL << g->timestampValidBits ) - 1 );
		double sum = 0.0;
		for ( int s = 0; s < steps; ++s )
		{
			double ms = (double)( ( ts[s + 1] - ts[s] ) & mask ) * (double)g->timestampPeriod * 1e-6;
			sum += ms;
			if ( rep > 0 && ms < best )
			{
				best = ms;
			}
		}
		repMeans[rep] = sum / steps;
	}
	qsort( repMeans + 1, (size_t)reps, sizeof( double ), cmp_double );
	res.msPerStep = repMeans[1 + reps / 2];
	res.msPerStepMin = best;
	gpu_download( g, &r, &res.final, res.counters );

	// Per-step round trip: one submit per step, wait, read back bodies and poses, hash.
	gpu_upload( g, &r, d );
	VkCommandBufferAllocateInfo ai = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
	ai.commandPool = g->cmdPool;
	ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	ai.commandBufferCount = 1;
	VkCommandBuffer cb;
	VK_CHECK( vkAllocateCommandBuffers( g->device, &ai, &cb ) );
	VkCommandBufferBeginInfo bi = { VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
	VK_CHECK( vkBeginCommandBuffer( cb, &bi ) );
	record_step( p, &r, cb, list, dispatchCount );
	record_download( &r, cb, 0, 2 );
	VK_CHECK( vkEndCommandBuffer( cb ) );
	VkFenceCreateInfo fci = { VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
	VkFence fence;
	VK_CHECK( vkCreateFence( g->device, &fci, NULL, &fence ) );
	Body* hb = (Body*)malloc( r.sizes[0] );
	Pose* hp = (Pose*)malloc( r.sizes[1] );
	res.firstDivergence = -1;
	double wall = 0.0;
	for ( int s = 0; s < steps; ++s )
	{
		double t0 = vku_now_ms();
		VkSubmitInfo si = { VK_STRUCTURE_TYPE_SUBMIT_INFO };
		si.commandBufferCount = 1;
		si.pCommandBuffers = &cb;
		VK_CHECK( vkQueueSubmit( g->queue, 1, &si, fence ) );
		VK_CHECK( vkWaitForFences( g->device, 1, &fence, VK_TRUE, UINT64_MAX ) );
		VK_CHECK( vkResetFences( g->device, 1, &fence ) );
		memcpy( hb, r.staging.mapped, r.sizes[0] );
		memcpy( hp, (uint8_t*)r.staging.mapped + r.sizes[0], r.sizes[1] );
		wall += vku_now_ms() - t0;
		uint64_t h = hash_state( hb, hp, d->bodyCount );
		if ( res.firstDivergence < 0 && cpuHash != NULL && h != cpuHash[s] )
		{
			res.firstDivergence = s;
		}
	}
	res.msReadback = wall / steps;
	free( hb );
	free( hp );
	vkDestroyFence( g->device, fence, NULL );
	vkFreeCommandBuffers( g->device, g->cmdPool, 1, &cb );
	free( ts );
	free( repMeans );
	gpu_teardown( g, &r );
	return res;
}

// ------------------------------------------------------------------------------------------------
// Trace: run CPU twin and one GPU dispatch by dispatch, stop at the first difference
// ------------------------------------------------------------------------------------------------

static void dump_words( FILE* log, const char* what, const void* p, size_t bytes )
{
	fprintf( log, "    %s:", what );
	const uint32_t* w = (const uint32_t*)p;
	for ( size_t i = 0; i < bytes / 4; ++i )
	{
		fprintf( log, " %08x", w[i] );
	}
	fprintf( log, "\n" );
}

static void trace_divergence( VkGpu* g, GpuPipes* p, const Data* d, const Dispatch* list, int dispatchCount, int steps, FILE* log )
{
	Snapshot cpu, gpu, prev;
	snap_alloc( &cpu, d );
	snap_alloc( &gpu, d );
	snap_alloc( &prev, d );
	memcpy( cpu.bodies, d->bodies, (size_t)d->bodyCount * sizeof( Body ) );
	memcpy( cpu.poses, d->poses, (size_t)d->bodyCount * sizeof( Pose ) );
	memcpy( cpu.imps, d->imps, (size_t)d->contactCount * sizeof( Impulse ) );
	Params params = d->params;
	uint32_t counters[4] = { 0 };
	twin_bind( cpu.bodies, (size_t)d->bodyCount, cpu.poses, (size_t)d->bodyCount, d->cons, (size_t)d->contactCount, cpu.imps,
			   (size_t)d->contactCount, &params, counters );
	GpuRun r;
	gpu_setup( g, p, &r, d, 1 );
	gpu_upload( g, &r, d );
	uint32_t gc[4];
	for ( int s = 0; s < steps; ++s )
	{
		for ( int i = 0; i < dispatchCount; ++i )
		{
			memcpy( prev.bodies, cpu.bodies, (size_t)d->bodyCount * sizeof( Body ) );
			memcpy( prev.imps, cpu.imps, (size_t)d->contactCount * sizeof( Impulse ) );
			run_chunk( list + i, 0, 1 );
			VkCommandBuffer cb = vku_begin( g );
			record_step( p, &r, cb, list + i, 1 );
			vku_submit_wait( g, cb );
			gpu_download( g, &r, &gpu, gc );
			Compare c = compare_snapshots( &cpu, &gpu, d );
			if ( c.mismatches > 0 )
			{
				fprintf( log, "  trace %s: first difference at step %d dispatch %d (%s start %u count %u)\n", g->vendor, s, i,
						 g_entryNames[list[i].entry], list[i].start, list[i].count );
				print_compare( log, "cpu vs gpu (trace)", &c );
				// the contacts of this dispatch that touch the first differing body
				int body = -1;
				for ( int k = 0; k < d->bodyCount && body < 0; ++k )
				{
					if ( memcmp( cpu.bodies + k, gpu.bodies + k, sizeof( Body ) ) != 0 )
					{
						body = k;
					}
				}
				fprintf( log, "    first differing body %d\n", body );
				for ( int k = 0; k < d->contactCount && body < 0; ++k )
				{
					if ( memcmp( cpu.imps + k, gpu.imps + k, sizeof( Impulse ) ) != 0 )
					{
						const Constraint* C = d->cons + k;
						fprintf( log, "    first differing impulse: contact %d (A %d, B %d)\n", k, C->indexA, C->indexB );
						dump_words( log, "constraint", C, sizeof( Constraint ) );
						dump_words( log, "impulse before", prev.imps + k, sizeof( Impulse ) );
						dump_words( log, "impulse cpu   ", cpu.imps + k, sizeof( Impulse ) );
						dump_words( log, "impulse gpu   ", gpu.imps + k, sizeof( Impulse ) );
						dump_words( log, "bodyA before", prev.bodies + C->indexA, sizeof( Body ) );
						dump_words( log, "bodyB before", prev.bodies + C->indexB, sizeof( Body ) );
						break;
					}
				}
				if ( body >= 0 )
				{
					dump_words( log, "body before", prev.bodies + body, sizeof( Body ) );
					dump_words( log, "body cpu   ", cpu.bodies + body, sizeof( Body ) );
					dump_words( log, "body gpu   ", gpu.bodies + body, sizeof( Body ) );
				}
				if ( list[i].entry == E_WARM || list[i].entry == E_PUSH || list[i].entry == E_RELAX )
				{
					for ( uint32_t k = list[i].start; k < list[i].start + list[i].count; ++k )
					{
						const Constraint* C = d->cons + k;
						if ( C->indexA == body || C->indexB == body )
						{
							fprintf( log, "    contact %u (A %d, B %d)\n", k, C->indexA, C->indexB );
							dump_words( log, "constraint", C, sizeof( Constraint ) );
							dump_words( log, "impulse before", prev.imps + k, sizeof( Impulse ) );
							dump_words( log, "impulse cpu   ", cpu.imps + k, sizeof( Impulse ) );
							dump_words( log, "impulse gpu   ", gpu.imps + k, sizeof( Impulse ) );
							dump_words( log, "bodyA before", prev.bodies + C->indexA, sizeof( Body ) );
							dump_words( log, "bodyB before", prev.bodies + C->indexB, sizeof( Body ) );
						}
					}
				}
				dump_words( log, "params", &d->params, sizeof( Params ) );
				gpu_teardown( g, &r );
				snap_free( &cpu );
				snap_free( &gpu );
				snap_free( &prev );
				return;
			}
		}
	}
	fprintf( log, "  trace %s: no difference in %d steps\n", g->vendor, steps );
	gpu_teardown( g, &r );
	snap_free( &cpu );
	snap_free( &gpu );
	snap_free( &prev );
}

// ------------------------------------------------------------------------------------------------
// main
// ------------------------------------------------------------------------------------------------

// SPIR-V file tags (float-control execution modes) per GPU vendor: --tag VENDOR=SUFFIX, VENDOR "*" for
// any vendor not named. Intel gets DenormPreserve by default (E11), every other vendor the plain build.
typedef struct VendorTag
{
	char vendor[16];
	char tag[16];
} VendorTag;

static VendorTag g_tags[16] = { { "intel", ".preserve" }, { "*", "" } };
static int g_tagCount = 2;

static void set_tag( const char* vendor, size_t vendorLen, const char* tag )
{
	int i = 0;
	while ( i < g_tagCount && !( strlen( g_tags[i].vendor ) == vendorLen && strncmp( g_tags[i].vendor, vendor, vendorLen ) == 0 ) )
	{
		++i;
	}
	if ( i == g_tagCount && g_tagCount < 16 )
	{
		++g_tagCount;
	}
	if ( i < 16 )
	{
		snprintf( g_tags[i].vendor, sizeof( g_tags[i].vendor ), "%.*s", (int)vendorLen, vendor );
		snprintf( g_tags[i].tag, sizeof( g_tags[i].tag ), "%s", tag );
	}
}

static const char* tag_for( const char* vendor )
{
	const char* any = "";
	for ( int i = 0; i < g_tagCount; ++i )
	{
		if ( strcmp( g_tags[i].vendor, vendor ) == 0 )
		{
			return g_tags[i].tag;
		}
		if ( strcmp( g_tags[i].vendor, "*" ) == 0 )
		{
			any = g_tags[i].tag;
		}
	}
	return any;
}

static int parse_list( const char* s, int* out, int max )
{
	int n = 0;
	while ( *s && n < max )
	{
		out[n++] = atoi( s );
		while ( *s && *s != ',' )
		{
			++s;
		}
		if ( *s == ',' )
		{
			++s;
		}
	}
	return n;
}

int main( int argc, char** argv )
{
	int ns[8] = { 1000, 10000, 100000 };
	int nCount = 3;
	int threads[4] = { 1, 8 };
	int tCount = 2;
	int steps = 60;
	int reps = 3;
	uint64_t seed = 20260930;
	const char* logPath = "logs/" VARIANT_NAME ".txt";
	const char* csvPath = "logs/summary.csv";
	const char* spvDir = "gen/" VARIANT_NAME;
	const char* label = VARIANT_NAME;
	int gpuMask = 0xff;
	int traceSteps = 0;
	for ( int i = 1; i < argc; ++i )
	{
		const char* a = argv[i];
		const char* v = i + 1 < argc ? argv[i + 1] : "";
		if ( strcmp( a, "--n" ) == 0 )
			nCount = parse_list( v, ns, 8 ), ++i;
		else if ( strcmp( a, "--threads" ) == 0 )
			tCount = parse_list( v, threads, 4 ), ++i;
		else if ( strcmp( a, "--steps" ) == 0 )
			steps = atoi( v ), ++i;
		else if ( strcmp( a, "--reps" ) == 0 )
			reps = atoi( v ), ++i;
		else if ( strcmp( a, "--seed" ) == 0 )
			seed = (uint64_t)strtoll( v, NULL, 10 ), ++i;
		else if ( strcmp( a, "--log" ) == 0 )
			logPath = v, ++i;
		else if ( strcmp( a, "--csv" ) == 0 )
			csvPath = v, ++i;
		else if ( strcmp( a, "--spv" ) == 0 )
			spvDir = v, ++i;
		else if ( strcmp( a, "--label" ) == 0 )
			label = v, ++i;
		else if ( strcmp( a, "--tag" ) == 0 && strchr( v, '=' ) != NULL )
			set_tag( v, (size_t)( strchr( v, '=' ) - v ), strchr( v, '=' ) + 1 ), ++i;
		else if ( strcmp( a, "--intel-tag" ) == 0 )
			set_tag( "intel", 5, v ), ++i;
		else if ( strcmp( a, "--nvidia-tag" ) == 0 )
			set_tag( "nvidia", 6, v ), ++i;
		else if ( strcmp( a, "--trace" ) == 0 )
			traceSteps = atoi( v ), ++i;
		else if ( strcmp( a, "--gpus" ) == 0 )
			gpuMask = atoi( v ), ++i;
		else
		{
			fprintf( stderr, "unknown option %s\n", a );
			return 1;
		}
	}

	FILE* log = fopen( logPath, "w" );
	FILE* csv = fopen( csvPath, "a" );
	if ( log == NULL || csv == NULL )
	{
		fprintf( stderr, "cannot open %s or %s\n", logPath, csvPath );
		return 1;
	}
	setvbuf( log, NULL, _IONBF, 0 );
	fprintf( log, "E11 harness, variant %s (label %s), T %zu bytes, TP %zu bytes; steps %d, reps %d, seed %llu\n", VARIANT_NAME, label,
			 sizeof( T ), sizeof( TP ), steps, reps, (unsigned long long)seed );
	fprintf( log, "layout sizes C/twin: Body %zu/%zu Pose %zu/%zu Constraint %zu/%zu Impulse %zu/%zu Params %zu/%zu\n", sizeof( Body ),
			 twin_sizeof( 0 ), sizeof( Pose ), twin_sizeof( 1 ), sizeof( Constraint ), twin_sizeof( 2 ), sizeof( Impulse ), twin_sizeof( 3 ),
			 sizeof( Params ), twin_sizeof( 4 ) );
	fprintf( log, "twin: %s\n", twin_info() );

	VkInstance inst = vku_create_instance();
	VkPhysicalDevice phys[8];
	int gpuCount = vku_list_gpus( inst, phys, 8 );
	VkGpu gpus[8];
	GpuPipes pipes[8];
	int open = 0;
	int gpuIndex[8];
	for ( int i = 0; i < gpuCount; ++i )
	{
		if ( !( gpuMask & ( 1 << i ) ) )
		{
			continue;
		}
		vku_open_gpu( inst, phys[i], &gpus[open] );
		VkGpu* g = &gpus[open];
		fprintf( log, "gpu %d: %s (%s), driver 0x%x, api %u.%u.%u; denorm preserve/ftz fp32 %u/%u, RTE fp32 %u, timestampPeriod %.3f ns\n", i,
				 g->props.deviceName, g->vendor, g->props.driverVersion, VK_API_VERSION_MAJOR( g->props.apiVersion ),
				 VK_API_VERSION_MINOR( g->props.apiVersion ), VK_API_VERSION_PATCH( g->props.apiVersion ),
				 g->floatControls.shaderDenormPreserveFloat32, g->floatControls.shaderDenormFlushToZeroFloat32,
				 g->floatControls.shaderRoundingModeRTEFloat32, (double)g->timestampPeriod );
		const char* tag = tag_for( g->vendor );
		if ( KT != K_F32 )
		{
			tag = ""; // integer kernels have no float modes
		}
		char missing[96];
		if ( !vku_tag_supported( g, tag, missing, sizeof( missing ) ) )
		{
			fprintf( log, "  skipped: SPIR-V *%s.spv asks for modes the device does not advertise:%s\n", tag, missing );
			vku_close_gpu( g );
			continue;
		}
		make_pipes( g, &pipes[open], spvDir, tag, log );
		fprintf( log, "  spirv %s/*%s.spv\n", spvDir, tag );
		gpuIndex[open] = i;
		++open;
	}

	for ( int ni = 0; ni < nCount; ++ni )
	{
		int n = ns[ni];
		Scenario sc;
		make_scenario( &sc, n, seed );
		Coloring col;
		make_coloring( &sc, &col );
		Data d;
		quantize( &sc, &col, &d );
		Dispatch* list = (Dispatch*)malloc( sizeof( Dispatch ) * (size_t)( 4 * ( 2 + 3 * col.colorCount ) + 1 ) );
		int dispatchCount = make_dispatches( &col, d.bodyCount, list );
		int points = 0, statics = 0;
		for ( int k = 0; k < n; ++k )
		{
			points += sc.contacts[k].pointCount;
			statics += sc.contacts[k].b == 0;
		}
		fprintf( log, "\n=== N %d contacts (%d points, %d on the static body), %d bodies, %d colours, %d dispatches per step; quantisation range errors %d, shift errors %d\n",
				 n, points, statics, d.bodyCount - 1, col.colorCount, dispatchCount, d.rangeErrors, d.shiftErrors );

		if ( traceSteps > 0 )
		{
			for ( int gi = 0; gi < open; ++gi )
			{
				trace_divergence( &gpus[gi], &pipes[gi], &d, list, dispatchCount, traceSteps, log );
			}
			free( list );
			free( col.order );
			free_data( &d );
			free_scenario( &sc );
			continue;
		}

		CpuResult ref;
		memset( &ref, 0, sizeof( ref ) );
		for ( int ti = 0; ti < tCount; ++ti )
		{
			CpuResult cr = run_cpu( &d, list, dispatchCount, steps, threads[ti] );
			fprintf( log, "  cpu twin %d thread(s): %.4f ms/step, final hash %016llx, range counter %u\n", threads[ti], cr.msPerStep,
					 (unsigned long long)cr.stepHash[steps - 1], cr.counters[0] );
			fprintf( csv, "%s,%d,%d,%d,cpu%d,batched,%.5f,,,%s\n", label, n, d.bodyCount - 1, col.colorCount, threads[ti], cr.msPerStep, "" );
			if ( ti == 0 )
			{
				ref = cr;
				subnormal_census( log, &ref.final, &d );
			}
			else
			{
				Compare c = compare_snapshots( &ref.final, &cr.final, &d );
				char lab[64];
				snprintf( lab, sizeof( lab ), "cpu1 vs cpu%d", threads[ti] );
				print_compare( log, lab, &c );
				snap_free( &cr.final );
				free( cr.stepHash );
			}
		}

		GpuResult gres[8];
		for ( int gi = 0; gi < open; ++gi )
		{
			VkGpu* g = &gpus[gi];
			gres[gi] = run_gpu( g, &pipes[gi], &d, list, dispatchCount, steps, reps, ref.stepHash );
			GpuResult* gr = &gres[gi];
			Compare c = compare_snapshots( &ref.final, &gr->final, &d );
			fprintf( log, "  gpu %s: %.4f ms/step (timestamps, median of %d reps; fastest step %.4f), %.4f ms/step with per-step submit+readback; first divergent step %d; range counter %u\n",
					 g->vendor, gr->msPerStep, reps, gr->msPerStepMin, gr->msReadback, gr->firstDivergence, gr->counters[0] );
			char lab[64];
			snprintf( lab, sizeof( lab ), "cpu vs %s", g->vendor );
			print_compare( log, lab, &c );
			fprintf( csv, "%s,%d,%d,%d,%s,batched,%.5f,%lld,%d,%.0f\n", label, n, d.bodyCount - 1, col.colorCount, g->vendor, gr->msPerStep,
					 c.mismatches, gr->firstDivergence, c.maxUlp );
			fprintf( csv, "%s,%d,%d,%d,%s,readback,%.5f,,,\n", label, n, d.bodyCount - 1, col.colorCount, g->vendor, gr->msReadback );
		}
		for ( int a = 0; a < open; ++a )
		{
			for ( int b = a + 1; b < open; ++b )
			{
				Compare c = compare_snapshots( &gres[a].final, &gres[b].final, &d );
				char lab[64];
				snprintf( lab, sizeof( lab ), "%s vs %s", gpus[a].vendor, gpus[b].vendor );
				print_compare( log, lab, &c );
			}
		}
		for ( int gi = 0; gi < open; ++gi )
		{
			snap_free( &gres[gi].final );
		}
		fflush( csv );
		snap_free( &ref.final );
		free( ref.stepHash );
		free( list );
		free( col.order );
		free_data( &d );
		free_scenario( &sc );
	}

	for ( int gi = 0; gi < open; ++gi )
	{
		vkDeviceWaitIdle( gpus[gi].device );
		free_pipes( &gpus[gi], &pipes[gi] );
		vku_close_gpu( &gpus[gi] );
	}
	(void)gpuIndex;
	vkDestroyInstance( inst, NULL );
	fclose( csv );
	fclose( log );
	printf( "done: %s\n", logPath );
	return 0;
}
