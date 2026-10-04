// rows.c: T10's E11 row benchmark (docs/research/m7-gpu-integer.md section 10). Built per format
// (-DVAR_F, -DVAR_I64, -DVAR_V3, -DVAR_V4, -DVAR_V4B). Generates the corpus in double (portable:
// + - * / sqrt only), computes a float64 reference of each row, quantises, runs the Slang kernel on
// every GPU and on the C++ twin, and reports hashes, ns/row, error against float64, clamp flips and
// counters.
//
//   rows_V4 --n 1048576 --reps 5 --log logs/rows_V4.txt [--spv gen/rows/V4.spv]
#include "rowlayout.h"
#include "vk_util.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( VAR_F )
#define VNAME "V1"
#elif defined( VAR_I64 )
#define VNAME "V2"
#elif defined( VAR_V3 )
#define VNAME "V3"
#elif defined( VAR_V4 )
#define VNAME "V4"
#else
#define VNAME "V4b"
#endif

void rtwin_bind( void* rows, size_t n, void* outs, void* counters );
void rtwin_run( uint32_t count, uint32_t g0, uint32_t g1 );
const char* rtwin_info( void );

// ------------------------------------------------------------------------------------------------
// PCG32 and helpers (same as harness.c)
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

static double urand( Pcg* r )
{
	return (double)pcg_next( r ) * ( 1.0 / 4294967296.0 );
}

static double urange( Pcg* r, double lo, double hi )
{
	return lo + ( hi - lo ) * urand( r );
}

static double logrand( Pcg* r, double lo, double hi )
{
	for ( ;; )
	{
		static const double p10[] = { 1e-3, 1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4, 1e5 };
		uint32_t k = pcg_next( r ) % 9u; // drawn first, as MSVC did for E11's corpus
		double x = p10[k] * ( 1.0 + 9.0 * urand( r ) );
		if ( x >= lo && x <= hi )
		{
			return x;
		}
	}
}

static double cbrt_newton( double v )
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

// small rotation: normalise (axis * angle / 2, 1)
static void small_quat( Pcg* r, double maxAngle, double q[4] )
{
	double ax[3];
	unit_vector( r, ax );
	double h = 0.5 * urange( r, 0.0, maxAngle );
	double x = ax[0] * h, y = ax[1] * h, z = ax[2] * h;
	double s = 1.0 / sqrt( 1.0 + x * x + y * y + z * z );
	q[0] = x * s;
	q[1] = y * s;
	q[2] = z * s;
	q[3] = s;
}

static void quat_to_matrix( const double q[4], double m[3][3] )
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

// ------------------------------------------------------------------------------------------------
// Rows in double, the reference kernel in double
// ------------------------------------------------------------------------------------------------

typedef struct DRBody
{
	double v[3], w[3], dq[4], dp[3], invMass, invI[9];
	int dynamic;
} DRBody;

typedef struct DRRow
{
	DRBody A, B;
	double rA[3], rB[3], n[3];
	double baseSep, normalMass, impulse, massScale, impulseScale, biasRate, contactSpeed, invH;
	int quarter; // 0 town, 1 chip vs mech, 2 car crash, 3 adversarial, 4 hand-made
} DRRow;

typedef struct DROut
{
	double vA[3], wA[3], vB[3], wB[3], pushImpulse, impulse;
} DROut;

static const char* g_quarterNames[5] = { "town", "chip vs mech", "car crash", "adversarial", "hand-made" };

static void d_cross( const double a[3], const double b[3], double o[3] )
{
	double x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2], z = a[0] * b[1] - a[1] * b[0];
	o[0] = x;
	o[1] = y;
	o[2] = z;
}

static double d_dot( const double a[3], const double b[3] )
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void d_mv( const double m[9], const double a[3], double o[3] )
{
	double x = m[0] * a[0] + m[1] * a[1] + m[2] * a[2];
	double y = m[3] * a[0] + m[4] * a[1] + m[5] * a[2];
	double z = m[6] * a[0] + m[7] * a[1] + m[8] * a[2];
	o[0] = x;
	o[1] = y;
	o[2] = z;
}

static void d_rotate( const double q[4], const double a[3], double o[3] )
{
	double qv[3] = { q[0], q[1], q[2] }, t[3], u[3];
	d_cross( qv, a, t );
	for ( int i = 0; i < 3; ++i )
		t[i] += t[i];
	d_cross( qv, t, u );
	for ( int i = 0; i < 3; ++i )
		o[i] = a[i] + q[3] * t[i] + u[i];
}

static void d_point( const DRRow* r, double vA[3], double wA[3], double vB[3], double wB[3], double* lambda, int relaxRow )
{
	double qa[3], qb[3], ds[3];
	d_rotate( r->A.dq, r->rA, qa );
	d_rotate( r->B.dq, r->rB, qb );
	for ( int i = 0; i < 3; ++i )
		ds[i] = ( r->B.dp[i] - r->A.dp[i] ) + ( qb[i] - qa[i] );
	double s = d_dot( ds, r->n ) + r->baseSep;
	double bias, ms, is;
	if ( relaxRow )
	{
		bias = s > 0.0 ? s * r->invH : 0.0;
		ms = 1.0;
		is = 0.0;
	}
	else if ( s > 0.0 )
	{
		bias = s * r->invH;
		ms = 1.0;
		is = 0.0;
	}
	else
	{
		double b = r->massScale * r->biasRate * s;
		bias = b > -r->contactSpeed ? b : -r->contactSpeed;
		ms = r->massScale;
		is = r->impulseScale;
	}
	double ca[3], cb[3], dv[3];
	d_cross( wA, r->rA, ca );
	d_cross( wB, r->rB, cb );
	for ( int i = 0; i < 3; ++i )
		dv[i] = ( vB[i] + cb[i] ) - ( vA[i] + ca[i] );
	double vn = d_dot( dv, r->n );
	double delta = -r->normalMass * ( ms * vn + bias ) - is * *lambda;
	double nw = *lambda + delta;
	nw = nw > 0.0 ? nw : 0.0;
	delta = nw - *lambda;
	*lambda = nw;
	double P[3] = { delta * r->n[0], delta * r->n[1], delta * r->n[2] }, L[3], d[3];
	for ( int i = 0; i < 3; ++i )
	{
		vA[i] -= r->A.invMass * P[i];
		vB[i] += r->B.invMass * P[i];
	}
	d_cross( r->rA, P, L );
	d_mv( r->A.invI, L, d );
	for ( int i = 0; i < 3; ++i )
		wA[i] -= d[i];
	d_cross( r->rB, P, L );
	d_mv( r->B.invI, L, d );
	for ( int i = 0; i < 3; ++i )
		wB[i] += d[i];
}

static void d_row( const DRRow* r, DROut* o )
{
	memcpy( o->vA, r->A.v, 24 );
	memcpy( o->wA, r->A.w, 24 );
	memcpy( o->vB, r->B.v, 24 );
	memcpy( o->wB, r->B.w, 24 );
	double lambda = r->impulse;
	d_point( r, o->vA, o->wA, o->vB, o->wB, &lambda, 0 );
	o->pushImpulse = lambda;
	d_point( r, o->vA, o->wA, o->vB, o->wB, &lambda, 1 );
	o->impulse = lambda;
}

typedef struct Softness
{
	double biasRate, massScale, impulseScale;
} Softness;

static Softness make_soft( double hertz, double zeta, double h )
{
	double omega = 2.0 * 3.14159265358979323846 * hertz;
	double a1 = 2.0 * zeta + h * omega;
	double a2 = h * omega * a1;
	double a3 = 1.0 / ( 1.0 + a2 );
	Softness s = { omega / a1, a2 * a3, a3 };
	return s;
}

static void box_body( Pcg* r, DRBody* b, double mass, double hx, double hy, double hz, double maxV, double maxW )
{
	double inv[3] = { 3.0 / ( mass * ( hy * hy + hz * hz ) ), 3.0 / ( mass * ( hx * hx + hz * hz ) ), 3.0 / ( mass * ( hx * hx + hy * hy ) ) };
	double q[4], R[3][3];
	unit_quat( r, q );
	quat_to_matrix( q, R );
	for ( int i = 0; i < 3; ++i )
		for ( int j = 0; j < 3; ++j )
			b->invI[3 * i + j] = R[i][0] * inv[0] * R[j][0] + R[i][1] * inv[1] * R[j][1] + R[i][2] * inv[2] * R[j][2];
	b->invMass = 1.0 / mass;
	b->dynamic = 1;
	double d[3];
	unit_vector( r, d );
	double sp = urange( r, 0, maxV );
	for ( int i = 0; i < 3; ++i )
		b->v[i] = sp * d[i];
	unit_vector( r, d );
	sp = urange( r, 0, maxW );
	for ( int i = 0; i < 3; ++i )
		b->w[i] = sp * d[i];
	small_quat( r, 0.05, b->dq );
	unit_vector( r, d );
	sp = urange( r, 0, 0.3 );
	for ( int i = 0; i < 3; ++i )
		b->dp[i] = sp * d[i];
}

static void iso_body( Pcg* r, DRBody* b, double mass, double inertia, double maxV, double maxW )
{
	memset( b->invI, 0, sizeof( b->invI ) );
	b->invI[0] = b->invI[4] = b->invI[8] = 1.0 / inertia;
	b->invMass = 1.0 / mass;
	b->dynamic = 1;
	double d[3];
	unit_vector( r, d );
	double sp = urange( r, 0, maxV );
	for ( int i = 0; i < 3; ++i )
		b->v[i] = sp * d[i];
	unit_vector( r, d );
	sp = urange( r, 0, maxW );
	for ( int i = 0; i < 3; ++i )
		b->w[i] = sp * d[i];
	small_quat( r, 0.05, b->dq );
	unit_vector( r, d );
	sp = urange( r, 0, 0.3 );
	for ( int i = 0; i < 3; ++i )
		b->dp[i] = sp * d[i];
}

static void static_body( DRBody* b )
{
	memset( b, 0, sizeof( *b ) );
	b->dq[3] = 1.0;
}

static double effective_mass( const DRRow* r )
{
	double rnA[3], rnB[3], t[3];
	d_cross( r->rA, r->n, rnA );
	d_cross( r->rB, r->n, rnB );
	double k = r->A.invMass + r->B.invMass;
	d_mv( r->A.invI, rnA, t );
	k += d_dot( rnA, t );
	d_mv( r->B.invI, rnB, t );
	k += d_dot( rnB, t );
	return 1.0 / k;
}

static void set_soft( DRRow* w, int isStatic )
{
	Softness s = isStatic ? make_soft( 60.0, 5.0, 1.0 / 240.0 ) : make_soft( 30.0, 10.0, 1.0 / 240.0 );
	w->massScale = s.massScale;
	w->impulseScale = s.impulseScale;
	w->biasRate = s.biasRate;
	w->contactSpeed = 3.0;
	w->invH = 240.0;
}

static void random_anchor( Pcg* r, double lim, double out[3] )
{
	for ( int i = 0; i < 3; ++i )
		out[i] = urange( r, -lim, lim );
}

static void make_corpus( DRRow* rows, int n, int hand )
{
	Pcg r = { 0x9e3779b97f4a7c15ULL, 0xda3e39cb94b95bdbULL };
	for ( int i = 0; i < n; ++i )
	{
		DRRow* w = rows + i;
		memset( w, 0, sizeof( *w ) );
		int q = (int)( ( (int64_t)i * 4 ) / n );
		w->quarter = q;
		unit_vector( &r, w->n );
		if ( q == 0 )
		{
			// town: 1-100 kg, |r| <= 0.5 m, speeds under 20 m/s
			for ( int b = 0; b < 2; ++b )
			{
				DRBody* body = b == 0 ? &w->A : &w->B;
				double m = logrand( &r, 1.0, 100.0 ), rho = urange( &r, 500.0, 2500.0 );
				double e = cbrt_newton( m / rho );
				// x, z, y: the order in which MSVC happened to evaluate these three arguments when it made E11's
				// corpus (found by trying all six); drawn explicitly, every compiler makes the same corpus
				double hx = 0.5 * e * urange( &r, 0.6, 1.4 ), hz = 0.5 * e * urange( &r, 0.6, 1.4 ), hy = 0.5 * e * urange( &r, 0.6, 1.4 );
				box_body( &r, body, m, hx, hy, hz, 20.0, 10.0 );
			}
			int isStatic = pcg_next( &r ) % 10u == 0;
			if ( isStatic )
				static_body( &w->B );
			random_anchor( &r, 0.29, w->rA );
			random_anchor( &r, 0.29, w->rB );
			w->baseSep = urange( &r, -0.02, 0.01 );
			set_soft( w, isStatic );
			w->normalMass = effective_mass( w );
			w->impulse = urange( &r, 0.0, w->normalMass * 1.0 );
		}
		else if ( q == 1 )
		{
			// chip (0.05 kg, inertia 6.3e-6) against mech (3000 kg, inertia 7000), both orders
			int swap = pcg_next( &r ) & 1;
			DRBody* chip = swap ? &w->B : &w->A;
			DRBody* mech = swap ? &w->A : &w->B;
			iso_body( &r, chip, 0.05, 6.3e-6, 20.0, 47.0 );
			iso_body( &r, mech, 3000.0, 7000.0, 20.0, 2.0 );
			random_anchor( &r, 0.014, swap ? w->rB : w->rA );
			random_anchor( &r, 1.5, swap ? w->rA : w->rB );
			w->baseSep = urange( &r, -0.02, 0.01 );
			set_soft( w, 0 );
			w->normalMass = effective_mass( w );
			w->impulse = urange( &r, 0.0, w->normalMass * 2.0 );
		}
		else if ( q == 2 )
		{
			// car crash: 1700 kg at 20 m/s into a static, large warm-start impulses
			box_body( &r, &w->A, 1700.0, 2.25, 0.9, 0.7, 0.0, 1.0 );
			double sp = urange( &r, 15.0, 20.0 );
			for ( int k = 0; k < 3; ++k )
				w->A.v[k] = sp * w->n[k] + urange( &r, -1.0, 1.0 ); // closing speed ~20 m/s along the normal
			static_body( &w->B );
			double lim[3] = { 2.25, 0.9, 0.7 };
			for ( int k = 0; k < 3; ++k )
				w->rA[k] = urange( &r, -lim[k], lim[k] );
			random_anchor( &r, 1.0, w->rB );
			w->baseSep = urange( &r, -0.05, 0.0 );
			set_soft( w, 1 );
			w->normalMass = effective_mass( w );
			w->impulse = urange( &r, 0.0, 34000.0 );
		}
		else
		{
			// adversarial: every input at a V4 format limit, expressed physically
			for ( int b = 0; b < 2; ++b )
			{
				DRBody* body = b == 0 ? &w->A : &w->B;
				body->dynamic = 1;
				double lim = ( 2147483647.0 / 4194304.0 ); // 2^31 - 1 units of Q9.22
				for ( int k = 0; k < 3; ++k )
				{
					body->v[k] = ( pcg_next( &r ) & 1 ) ? lim : -lim;
					body->w[k] = ( pcg_next( &r ) & 1 ) ? 255.99999988 : -255.99999988;
					body->dp[k] = urange( &r, -31.9, 31.9 );
				}
				double qn[4];
				unit_quat( &r, qn );
				double sc = urange( &r, 0.87, 1.13 ); // unnormalised by up to 13 %
				for ( int k = 0; k < 4; ++k )
					body->dq[k] = qn[k] * sc;
				body->invMass = ( pcg_next( &r ) & 1 ) ? 1e3 : 1e-9;
				double ii = ( pcg_next( &r ) & 1 ) ? 1e7 : 1e-9;
				memset( body->invI, 0, sizeof( body->invI ) );
				body->invI[0] = body->invI[4] = body->invI[8] = ii;
			}
			for ( int k = 0; k < 3; ++k )
			{
				w->rA[k] = ( pcg_next( &r ) & 1 ) ? 32.0 - 1.0 / 16777216.0 : -32.0;
				w->rB[k] = ( pcg_next( &r ) & 1 ) ? 32.0 - 1.0 / 16777216.0 : -32.0;
			}
			w->baseSep = ( pcg_next( &r ) & 1 ) ? 511.99999976 : -512.0;
			set_soft( w, 0 );
			w->normalMass = ( pcg_next( &r ) & 1 ) ? 1e5 : 1e-7;
			w->impulse = ( ( pcg_next( &r ) & 1 ) ? 1.0 : -1.0 ) * w->normalMass * 1023.99;
		}
	}
	// hand-made rows, exactly representable in every format
	for ( int i = n; i < n + hand; ++i )
	{
		DRRow* w = rows + i;
		memset( w, 0, sizeof( *w ) );
		w->quarter = 4;
		int k = ( i - n ) / 128;
		w->n[0] = 1.0;
		static_body( &w->A );
		static_body( &w->B );
		w->A.dynamic = 1;
		w->A.invMass = 1.0;
		w->A.invI[0] = w->A.invI[4] = w->A.invI[8] = 1.0;
		if ( k != 1 )
		{
			w->B.dynamic = 1;
			w->B.invMass = 1.0;
			w->B.invI[0] = w->B.invI[4] = w->B.invI[8] = 1.0;
		}
		set_soft( w, k == 1 );
		w->normalMass = k == 1 ? 1.0 : 0.5;
		int j = ( i - n ) % 128;
		switch ( k )
		{
			case 0: // all zero
			case 1: // static B, at rest, s = 0
				break;
			case 2: // at rest, s = 0, warm impulse > 0
				w->impulse = 0.25 * ( 1 + j % 8 );
				break;
			case 3: // exact clamp tie in the push: separated, bias 0.234375, vn 0.765625 -> delta = -lambda
				w->normalMass = 1.0;
				w->B.invMass = 0.0;
				w->B.dynamic = 0;
				memset( w->B.invI, 0, sizeof( w->B.invI ) );
				w->baseSep = 1.0 / 1024.0;
				w->A.v[0] = -0.765625;
				w->impulse = 1.0;
				break;
			case 4: // delta exactly zero: at rest, no impulse, touching
				w->baseSep = 0.0;
				break;
			case 5: // speculative, far apart: no impulse
				w->baseSep = 1.0;
				w->A.v[0] = 0.5;
				break;
			default: // equal velocities, anchors at the centre: vn exactly 0
				w->A.v[0] = w->B.v[0] = 0.125 * ( j % 16 );
				w->A.v[1] = w->B.v[1] = -0.0625 * ( j % 7 );
				w->impulse = 0.5 * ( j % 4 );
				break;
		}
	}
}

// ------------------------------------------------------------------------------------------------
// Quantisation
// ------------------------------------------------------------------------------------------------

static long long g_rangeErrorsQ[5], g_shiftClampsQ[5];
static int g_q;
#define g_rangeErrors g_rangeErrorsQ[g_q]
#define g_shiftClamps g_shiftClampsQ[g_q]

static int64_t round_scaled( double x, int s )
{
	return (int64_t)floor( ldexp( x, s ) + 0.5 );
}

static int exp_for( double maxAbs, int bits )
{
	if ( maxAbs <= 0.0 )
		return 40;
	int k;
	frexp( maxAbs, &k );
	return bits - k;
}

#if defined( VAR_F )
static T Q( double x, int s )
{
	(void)s;
	return (T)x;
}
#elif defined( VAR_I64 )
static T Q( double x, int s )
{
	(void)s;
	if ( fabs( x ) >= 2147483648.0 )
		g_rangeErrors += 1;
	return round_scaled( x, 32 );
}
#else
static T Q( double x, int s )
{
	int64_t v = round_scaled( x, s );
	if ( v > 2147483647LL || v < -2147483647LL )
	{
		g_rangeErrors += 1;
		v = v > 0 ? 2147483647LL : -2147483647LL;
	}
	return (T)v;
}
#endif

static int clamp_sh( int sh )
{
	if ( sh < 1 || sh > 63 )
	{
		g_shiftClamps += 1;
		return sh < 1 ? 1 : 63;
	}
	return sh;
}

static void q_body( const DRBody* d, RBody* b )
{
	for ( int k = 0; k < 3; ++k )
	{
		b->v[k] = Q( d->v[k], S_V );
		b->w[k] = Q( d->w[k], S_W );
		b->dp[k] = Q( d->dp[k], S_DP );
	}
	for ( int k = 0; k < 4; ++k )
		b->dq[k] = Q( d->dq[k], S_Q );
	b->flags = d->dynamic;
	double mi = 0.0;
	for ( int k = 0; k < 9; ++k )
		mi = fabs( d->invI[k] ) > mi ? fabs( d->invI[k] ) : mi;
#if defined( VAR_V3 )
	b->eM = S_IM;
	b->eI = S_II;
#else
	b->eM = exp_for( d->invMass, 31 );
	b->eI = exp_for( mi, 29 );
#endif
	b->invMass = Q( d->invMass, b->eM );
	for ( int k = 0; k < 9; ++k )
		b->invI[k] = Q( d->invI[k], b->eI );
}

static void q_row( const DRRow* d, RRow* r )
{
	memset( r, 0, sizeof( *r ) );
	q_body( &d->A, &r->A );
	q_body( &d->B, &r->B );
	for ( int k = 0; k < 3; ++k )
	{
		r->rA[k] = Q( d->rA[k], S_R );
		r->rB[k] = Q( d->rB[k], S_R );
		r->n[k] = Q( d->n[k], S_Q );
	}
	r->baseSep = Q( d->baseSep, S_S );
#if defined( VAR_V3 )
	r->eN = S_NM;
	r->eP = S_P;
#else
	r->eN = exp_for( d->normalMass, 31 );
	r->eP = r->eN - 10; // e_P = e_n + 10 in T10's sign convention
#endif
	r->normalMass = Q( d->normalMass, r->eN );
	r->impulse = Q( d->impulse, r->eP );
	r->massScale = Q( d->massScale, S_MS );
	r->impulseScale = Q( d->impulseScale, S_MS );
	r->biasRate = Q( d->biasRate, S_BR );
	r->contactSpeed = Q( d->contactSpeed, S_V );
	r->invH = Q( d->invH, S_IH );
#if defined( INT32_FORMAT )
	r->shMA = clamp_sh( r->A.eM + r->eP - S_V );
	r->shMB = clamp_sh( r->B.eM + r->eP - S_V );
	r->shIA = clamp_sh( r->A.eI + r->eP - S_W );
	r->shIB = clamp_sh( r->B.eI + r->eP - S_W );
	r->shN = clamp_sh( r->eN + S_V - r->eP );
#endif
}

static double dq_( T x, int s )
{
#if defined( VAR_F )
	(void)s;
	return (double)x;
#elif defined( VAR_I64 )
	(void)s;
	return ldexp( (double)x, -32 );
#else
	return ldexp( (double)x, -s );
#endif
}

// ------------------------------------------------------------------------------------------------
// Accuracy
// ------------------------------------------------------------------------------------------------

typedef struct Acc
{
	double maxRel[5], sumSq[5], maxAbs[5];
	long long count[5], flipsPush[5], flipsRelax[5], rows[5];
} Acc;

static double norm3( const double a[3] )
{
	return sqrt( a[0] * a[0] + a[1] * a[1] + a[2] * a[2] );
}

static void accuracy( Acc* acc, const DRRow* d, const RRow* q, const ROut* o, const DROut* ref )
{
	int k = d->quarter;
	acc->rows[k] += 1;
	// velocity deltas of the format (its own dequantised input) against the float64 deltas
	const T* inV[4] = { q->A.v, q->A.w, q->B.v, q->B.w };
	const T* outV[4] = { o->vA, o->wA, o->vB, o->wB };
	const double* refIn[4] = { d->A.v, d->A.w, d->B.v, d->B.w };
	const double* refOut[4] = { ref->vA, ref->wA, ref->vB, ref->wB };
	int dyn[4] = { d->A.dynamic, d->A.dynamic, d->B.dynamic, d->B.dynamic };
	int sc[4] = { S_V, S_W, S_V, S_W };
	for ( int v = 0; v < 4; ++v )
	{
		if ( !dyn[v] )
			continue;
		double dv[3], dr[3], e[3];
		for ( int i = 0; i < 3; ++i )
		{
			dv[i] = dq_( outV[v][i], sc[v] ) - dq_( inV[v][i], sc[v] );
			dr[i] = refOut[v][i] - refIn[v][i];
			e[i] = dv[i] - dr[i];
		}
		// relative error, with deltas under 1 mm/s (1 mrad/s) measured against 1 mm/s
		double nr = norm3( dr );
		double ne = norm3( e );
		acc->maxAbs[k] = ne > acc->maxAbs[k] ? ne : acc->maxAbs[k];
		double rel = ne / ( nr > 1e-3 ? nr : 1e-3 );
		acc->maxRel[k] = rel > acc->maxRel[k] ? rel : acc->maxRel[k];
		acc->sumSq[k] += rel * rel;
		acc->count[k] += 1;
	}
	int zp = o->pushImpulse == 0, zr = o->impulse == 0;
	acc->flipsPush[k] += zp != ( ref->pushImpulse == 0.0 );
	acc->flipsRelax[k] += zr != ( ref->impulse == 0.0 );
}

// ------------------------------------------------------------------------------------------------
// GPU
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

static int cmp_d( const void* a, const void* b )
{
	double x = *(const double*)a, y = *(const double*)b;
	return x < y ? -1 : x > y ? 1 : 0;
}

typedef struct GpuRows
{
	double nsPerRow, nsMin;
	uint64_t hash;
	uint32_t counters[4];
	long long mism;
} GpuRows;

static GpuRows run_gpu( VkGpu* g, const char* spv, const RRow* rows, int n, int reps, ROut* outs, FILE* log )
{
	GpuRows res;
	memset( &res, 0, sizeof( res ) );
	VkDescriptorSetLayoutBinding b[3];
	for ( int i = 0; i < 3; ++i )
		b[i] = ( VkDescriptorSetLayoutBinding ){ (uint32_t)i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL };
	VkDescriptorSetLayoutCreateInfo dci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
	dci.bindingCount = 3;
	dci.pBindings = b;
	VkDescriptorSetLayout dsl;
	VK_CHECK( vkCreateDescriptorSetLayout( g->device, &dci, NULL, &dsl ) );
	VkPushConstantRange pcr = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( RPush ) };
	VkPipelineLayoutCreateInfo lci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	lci.setLayoutCount = 1;
	lci.pSetLayouts = &dsl;
	lci.pushConstantRangeCount = 1;
	lci.pPushConstantRanges = &pcr;
	VkPipelineLayout layout;
	VK_CHECK( vkCreatePipelineLayout( g->device, &lci, NULL, &layout ) );
	VkPipeline pipe = vku_pipeline( g, layout, spv, "main" );
	if ( pipe == VK_NULL_HANDLE )
		exit( 1 );
	vku_print_pipeline_stats( g, pipe, "rows", log );

	size_t inBytes = (size_t)n * sizeof( RRow ), outBytes = (size_t)n * sizeof( ROut );
	VkBuf bin = vku_buffer( g, inBytes, false ), bout = vku_buffer( g, outBytes, false ), bcnt = vku_buffer( g, 16, false );
	VkBuf st = vku_buffer( g, inBytes > outBytes + 16 ? inBytes : outBytes + 16, true );
	VkDescriptorPoolSize ps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3 };
	VkDescriptorPoolCreateInfo pci = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
	pci.maxSets = 1;
	pci.poolSizeCount = 1;
	pci.pPoolSizes = &ps;
	VkDescriptorPool pool;
	VK_CHECK( vkCreateDescriptorPool( g->device, &pci, NULL, &pool ) );
	VkDescriptorSetAllocateInfo ai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	ai.descriptorPool = pool;
	ai.descriptorSetCount = 1;
	ai.pSetLayouts = &dsl;
	VkDescriptorSet set;
	VK_CHECK( vkAllocateDescriptorSets( g->device, &ai, &set ) );
	VkDescriptorBufferInfo bi[3] = { { bin.buffer, 0, VK_WHOLE_SIZE }, { bout.buffer, 0, VK_WHOLE_SIZE }, { bcnt.buffer, 0, VK_WHOLE_SIZE } };
	VkWriteDescriptorSet w[3];
	for ( int i = 0; i < 3; ++i )
	{
		w[i] = ( VkWriteDescriptorSet ){ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
		w[i].dstSet = set;
		w[i].dstBinding = (uint32_t)i;
		w[i].descriptorCount = 1;
		w[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		w[i].pBufferInfo = &bi[i];
	}
	vkUpdateDescriptorSets( g->device, 3, w, 0, NULL );
	VkQueryPoolCreateInfo qci = { VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO };
	qci.queryType = VK_QUERY_TYPE_TIMESTAMP;
	qci.queryCount = 2;
	VkQueryPool qp;
	VK_CHECK( vkCreateQueryPool( g->device, &qci, NULL, &qp ) );

	memcpy( st.mapped, rows, inBytes );
	VkCommandBuffer cb = vku_begin( g );
	VkBufferCopy c = { 0, 0, inBytes };
	vkCmdCopyBuffer( cb, st.buffer, bin.buffer, 1, &c );
	vku_submit_wait( g, cb );

	double* ns = (double*)malloc( sizeof( double ) * (size_t)( reps + 1 ) );
	double best = 1e30;
	for ( int rep = 0; rep <= reps; ++rep )
	{
		cb = vku_begin( g );
		vkCmdFillBuffer( cb, bcnt.buffer, 0, 16, 0 );
		vkCmdResetQueryPool( cb, qp, 0, 2 );
		vku_barrier( cb );
		vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE, pipe );
		vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_COMPUTE, layout, 0, 1, &set, 0, NULL );
		RPush pp = { (uint32_t)n, 0, 0, 0 };
		vkCmdPushConstants( cb, layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( pp ), &pp );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, qp, 0 );
		vkCmdDispatch( cb, ( (uint32_t)n + 63 ) / 64, 1, 1 );
		vkCmdWriteTimestamp( cb, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, qp, 1 );
		vku_barrier( cb );
		VkBufferCopy c1 = { 0, 0, outBytes };
		vkCmdCopyBuffer( cb, bout.buffer, st.buffer, 1, &c1 );
		VkBufferCopy c2 = { 0, outBytes, 16 };
		vkCmdCopyBuffer( cb, bcnt.buffer, st.buffer, 1, &c2 );
		vku_submit_wait( g, cb );
		uint64_t ts[2];
		VK_CHECK( vkGetQueryPoolResults( g->device, qp, 0, 2, sizeof( ts ), ts, 8, VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WAIT_BIT ) );
		ns[rep] = (double)( ts[1] - ts[0] ) * (double)g->timestampPeriod / (double)n;
		if ( rep > 0 && ns[rep] < best )
			best = ns[rep];
	}
	qsort( ns + 1, (size_t)reps, sizeof( double ), cmp_d );
	res.nsPerRow = ns[1 + reps / 2];
	res.nsMin = best;
	memcpy( outs, st.mapped, outBytes );
	memcpy( res.counters, (uint8_t*)st.mapped + outBytes, 16 );
	res.hash = fnv( fnv( 1469598103934665603ULL, outs, outBytes ), res.counters, 8 );
	free( ns );

	vkDestroyQueryPool( g->device, qp, NULL );
	vkDestroyDescriptorPool( g->device, pool, NULL );
	vku_free( g, &bin );
	vku_free( g, &bout );
	vku_free( g, &bcnt );
	vku_free( g, &st );
	vkDestroyPipeline( g->device, pipe, NULL );
	vkDestroyPipelineLayout( g->device, layout, NULL );
	vkDestroyDescriptorSetLayout( g->device, dsl, NULL );
	return res;
}

int main( int argc, char** argv )
{
	int n = 1 << 20, hand = 1024, reps = 5, gpuMask = 0xff;
	const char* logPath = "logs/rows_" VNAME ".txt";
	const char* csvPath = "logs/rows_summary.csv";
	const char* spv = NULL;
	const char* tag = ""; // the float-control modes the SPIR-V file declares (lab.py's file tags)
	const char* label = VNAME;
	for ( int i = 1; i < argc; ++i )
	{
		const char* v = i + 1 < argc ? argv[i + 1] : "";
		if ( strcmp( argv[i], "--n" ) == 0 )
			n = atoi( v ), ++i;
		else if ( strcmp( argv[i], "--reps" ) == 0 )
			reps = atoi( v ), ++i;
		else if ( strcmp( argv[i], "--log" ) == 0 )
			logPath = v, ++i;
		else if ( strcmp( argv[i], "--csv" ) == 0 )
			csvPath = v, ++i;
		else if ( strcmp( argv[i], "--spv" ) == 0 )
			spv = v, ++i;
		else if ( strcmp( argv[i], "--label" ) == 0 )
			label = v, ++i;
		else if ( strcmp( argv[i], "--gpus" ) == 0 )
			gpuMask = atoi( v ), ++i;
		else if ( strcmp( argv[i], "--tag" ) == 0 )
			tag = v, ++i;
	}
	char spvDefault[256];
	snprintf( spvDefault, sizeof( spvDefault ), "gen/rows/%s.spv", VNAME );
	if ( spv == NULL )
		spv = spvDefault;
	FILE* log = fopen( logPath, "w" );
	FILE* csv = fopen( csvPath, "a" );
	if ( !log || !csv )
		return 1;
	setvbuf( log, NULL, _IONBF, 0 );
	int total = n + hand;
	DRRow* drows = (DRRow*)malloc( sizeof( DRRow ) * (size_t)total );
	DROut* ref = (DROut*)malloc( sizeof( DROut ) * (size_t)total );
	RRow* rows = (RRow*)malloc( sizeof( RRow ) * (size_t)total );
	ROut* twin = (ROut*)calloc( (size_t)total, sizeof( ROut ) );
	ROut* gout = (ROut*)calloc( (size_t)total, sizeof( ROut ) );
	make_corpus( drows, n, hand );
	for ( int i = 0; i < total; ++i )
	{
		d_row( drows + i, ref + i );
		g_q = drows[i].quarter;
		q_row( drows + i, rows + i );
	}
	fprintf( log, "E11 rows %s (label %s): %d rows (%d per quarter + %d hand-made), RRow %zu bytes, ROut %zu bytes, spv %s\n", VNAME, label, total,
			 n / 4, hand, sizeof( RRow ), sizeof( ROut ), spv );
	fprintf( log, "prepare per quarter (town, chip vs mech, car crash, adversarial, hand-made): quantisation range errors %lld %lld %lld %lld %lld; shift clamps %lld %lld %lld %lld %lld\n",
			 g_rangeErrorsQ[0], g_rangeErrorsQ[1], g_rangeErrorsQ[2], g_rangeErrorsQ[3], g_rangeErrorsQ[4], g_shiftClampsQ[0], g_shiftClampsQ[1],
			 g_shiftClampsQ[2], g_shiftClampsQ[3], g_shiftClampsQ[4] );
	fprintf( log, "corpus hashes: rows %016llx (double) %016llx (quantised)\n",
			 (unsigned long long)fnv( 1469598103934665603ULL, drows, sizeof( DRRow ) * (size_t)total ),
			 (unsigned long long)fnv( 1469598103934665603ULL, rows, sizeof( RRow ) * (size_t)total ) );

	// C++ twin, one thread
	uint32_t tc[4] = { 0 };
	rtwin_bind( rows, (size_t)total, twin, tc );
	double t0 = vku_now_ms();
	rtwin_run( (uint32_t)total, 0, ( (uint32_t)total + 63 ) / 64 );
	double twinNs = ( vku_now_ms() - t0 ) * 1e6 / total;
	// saturations per quarter: rerun the twin over each quarter's groups (quarters are 64-aligned)
	uint32_t satQ[5] = { 0 };
	{
		ROut* scratch = (ROut*)calloc( (size_t)total, sizeof( ROut ) );
		uint32_t qc[4];
		for ( int k = 0; k < 5; ++k )
		{
			uint32_t first = k < 4 ? (uint32_t)( ( (int64_t)n * k ) / 4 ) : (uint32_t)n;
			uint32_t last = k < 4 ? (uint32_t)( ( (int64_t)n * ( k + 1 ) ) / 4 ) : (uint32_t)total;
			memset( qc, 0, sizeof( qc ) );
			rtwin_bind( rows, (size_t)total, scratch, qc );
			rtwin_run( last, first / 64, ( last + 63 ) / 64 );
			satQ[k] = qc[0];
		}
		free( scratch );
	}
	uint64_t twinHash = fnv( fnv( 1469598103934665603ULL, twin, sizeof( ROut ) * (size_t)total ), tc, 8 );
	fprintf( log, "twin (%s; 1 thread): %.2f ns/row, hash %016llx, counters: saturations %u, kernel shift clamps %u\n", rtwin_info(), twinNs,
			 (unsigned long long)twinHash, tc[0], tc[1] );
	fprintf( csv, "%s,twin,%.3f,,%016llx,%u,%u\n", label, twinNs, (unsigned long long)twinHash, tc[0], tc[1] );

	Acc acc;
	memset( &acc, 0, sizeof( acc ) );
	for ( int i = 0; i < total; ++i )
		accuracy( &acc, drows + i, rows + i, twin + i, ref + i );
	fprintf( log, "accuracy against float64 (relative error of the velocity deltas; clamp decisions newImpulse == 0):\n" );
	for ( int k = 0; k < 5; ++k )
	{
		fprintf( log, "  %-13s rows %7lld  max rel %.3e  rms rel %.3e  max abs %.3e  (%lld vectors)  clamp flips push %lld relax %lld  saturations %u\n",
				 g_quarterNames[k], acc.rows[k], acc.maxRel[k], acc.count[k] ? sqrt( acc.sumSq[k] / (double)acc.count[k] ) : 0.0, acc.maxAbs[k],
				 acc.count[k], acc.flipsPush[k], acc.flipsRelax[k], satQ[k] );
	}

	VkInstance inst = vku_create_instance();
	VkPhysicalDevice phys[8];
	int gpuCount = vku_list_gpus( inst, phys, 8 );
	for ( int gi = 0; gi < gpuCount; ++gi )
	{
		if ( !( gpuMask & ( 1 << gi ) ) )
			continue;
		VkGpu g;
		vku_open_gpu( inst, phys[gi], &g );
		char missing[96];
		if ( !vku_tag_supported( &g, tag, missing, sizeof( missing ) ) )
		{
			fprintf( log, "gpu %s skipped: the SPIR-V asks for modes it does not advertise:%s\n", g.vendor, missing );
			vku_close_gpu( &g );
			continue;
		}
		GpuRows gr = run_gpu( &g, spv, rows, total, reps, gout, log );
		long long mism = 0;
		int shown = 0;
		for ( int i = 0; i < total; ++i )
		{
			if ( memcmp( gout + i, twin + i, sizeof( ROut ) ) != 0 )
			{
				if ( shown < 3 )
				{
					fprintf( log, "    first mismatch row %d (%s)\n", i, g_quarterNames[drows[i].quarter] );
					if ( shown == 0 )
					{
						char dumpPath[600];
						snprintf( dumpPath, sizeof( dumpPath ), "%s.mismatch.txt", logPath );
						FILE* f = fopen( dumpPath, "w" );
						if ( f )
						{
							const int32_t* w = (const int32_t*)( rows + i );
							fprintf( f, "row %d words:", i );
							for ( size_t k = 0; k < sizeof( RRow ) / 4; ++k )
								fprintf( f, " %d", w[k] );
							fprintf( f, "\ntwin:" );
							w = (const int32_t*)( twin + i );
							for ( size_t k = 0; k < sizeof( ROut ) / 4; ++k )
								fprintf( f, " %d", w[k] );
							fprintf( f, "\ngpu:" );
							w = (const int32_t*)( gout + i );
							for ( size_t k = 0; k < sizeof( ROut ) / 4; ++k )
								fprintf( f, " %d", w[k] );
							fprintf( f, "\n" );
							fclose( f );
						}
					}
					++shown;
				}
				++mism;
			}
		}
		fprintf( log, "gpu %s: %.3f ns/row (median of %d; fastest %.3f), hash %016llx %s twin, rows differing %lld, counters: saturations %u, kernel shift clamps %u\n",
				 g.vendor, gr.nsPerRow, reps, gr.nsMin, (unsigned long long)gr.hash, gr.hash == twinHash ? "==" : "!=", mism, gr.counters[0],
				 gr.counters[1] );
		fprintf( csv, "%s,%s,%.4f,%.4f,%016llx,%u,%u\n", label, g.vendor, gr.nsPerRow, gr.nsMin, (unsigned long long)gr.hash, gr.counters[0],
				 gr.counters[1] );
		vku_close_gpu( &g );
	}
	vkDestroyInstance( inst, NULL );
	fclose( csv );
	fclose( log );
	printf( "done: %s\n", logPath );
	return 0;
}
