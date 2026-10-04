// num.slang: the toy's arithmetic, one source for three dialects (DESIGN.md "One source, three dialects").
//
//   DIALECT_F : fp32 with NoContraction (-fp-mode precise); on the GPU only + - * (no sqrt, inversesqrt, division,
//               min, max, abs, clamp, sign or fma): compares and selects are ternaries, reciprocals and square
//               roots are software (an exponent-bit seed and three Newton steps). snap( x ) sends |x| < 2^-30 to +0
//               at every select and store, so a driver that flushes subnormals and one that keeps them agree.
//   DIALECT_V4: int32 in static formats (layout.h). Products go through int64 in one helper only (mas: a sum of up
//               to three products, one rounding shift, a saturating narrow that counts); positions are two words.
//   DIALECT_D : double, twin only: the same algorithm in near-exact arithmetic (real division and sqrt).
//
// Shifts are V4's format changes and never change a value: F and D ignore them. The including file defines
// TOY_COUNTERS_BINDING before including this one.
#include "layout.h"

[[vk::binding( TOY_COUNTERS_BINDING, 0 )]] RWStructuredBuffer<uint> counters;

// The saturation counter (V4's narrows). Not hashed; atomics are the kernels' one exception (DESIGN.md).
void countSaturation()
{
#if defined( TWIN_CPP )
	counters[0] = counters[0] + 1u; // the twin gives each thread its own counters (the glue)
#else
	InterlockedAdd( counters[0], 1u );
#endif
}

// A constant in each dialect: the float literal, the double literal, the V4 integer (in the format of its use).
#if defined( DIALECT_F )
#define KT( f, d, q ) f
#elif defined( DIALECT_D )
#define KT( f, d, q ) d
#else
#define KT( f, d, q ) q
#endif

T zero() { return KT( 0.0f, 0.0l, 0 ); }

// Leading zeros of a 32-bit word (32 for zero); integer, every dialect.
int clz( uint x )
{
	int msb = int( firstbithigh( x ) );
	return x == 0u ? 32 : 31 - msb;
}

// =====================================================================================================================
// F: fp32
// =====================================================================================================================
#if defined( DIALECT_F )

static const float SNAP_LIMIT = 9.31322574615478515625e-10f; // 2^-30

T snap( T x ) { return ( x < SNAP_LIMIT && x > -SNAP_LIMIT ) ? 0.0f : x; }
T M( T a, T b, int sh ) { return a * b; }
T sum2( T a, T b, T c, T d, int sh ) { return a * b + c * d; }
T dif2( T a, T b, T c, T d, int sh ) { return a * b - c * d; }
T sum3( T a, T b, T c, T d, T e, T f, int sh ) { return a * b + c * d + e * f; }
T rescale( T x, int sh ) { return x; }

// 1/d for a normal d > 0: d = m 2^(E - 126) with m in [0.5, 1) from the exponent bits, the seed 48/17 - 32/17 m
// (relative error below 1/17), three Newton steps, then the exact scale 2^(126 - E). 0 for d >= 2^126. The steps are
// written y + y (1 - m y), a small correction to y, not y (2 - m y): within 1.5 ulp (2.5 the other way), and exact
// at powers of two.
float recipPos( float d )
{
	uint bits = asuint( d );
	uint E = ( bits >> 23 ) & 0xffu;
	float m = asfloat( ( bits & 0x007fffffu ) | 0x3f000000u );
	float y = 2.82352941176470588235f - 1.88235294117647058824f * m;
	[ForceUnroll]
	for ( int i = 0; i < 3; ++i )
	{
		float e = 1.0f - m * y;
		y = y + y * e;
	}
	float scale = asfloat( ( 253u - E ) << 23 );
	return E > 252u ? 0.0f : y * scale;
}

// 1/sqrt(x) for a normal x > 0: the exponent-bit seed 0x5f3759df - bits/2 (relative error below 3.5%), three Newton
// steps written y + y (1/2 - (x/2) y y): within 1.7 ulp, and exact at powers of four (so a unit quaternion stays one;
// y (3/2 - (x/2) y y) gives 1 - 2^-24 for x = 1).
float rsqrtPos( float x )
{
	float y = asfloat( 0x5f3759dfu - ( asuint( x ) >> 1 ) );
	float hx = 0.5f * x;
	[ForceUnroll]
	for ( int i = 0; i < 3; ++i )
	{
		float r = 0.5f - hx * y * y;
		y = y + y * r;
	}
	return y;
}

// Every reciprocal and square root below takes an input made safe before the test that discards it (d == 0 ? 1 : d),
// so a compiler that evaluates it ahead of the select (clang 18 on aarch64 does, for D's division) raises no flag.
T rsqrtT( T x, int sx, int sy )
{
	T xs = x > 0.0f ? x : 1.0f;
	T r = rsqrtPos( xs );
	return snap( x > 0.0f ? r : 0.0f );
}

// 1/d (0 for d == 0); formats ignored
T recipT( T d, int sd, int sr )
{
	T ad = d < 0.0f ? -d : d;
	T as = ad > 0.0f ? ad : 1.0f;
	T r = recipPos( as );
	r = d < 0.0f ? -r : r;
	return snap( ad > 0.0f ? r : 0.0f );
}

// n / d clamped to [-lim, lim]: n times the software reciprocal; |n| >= lim |d| (d == 0 among them) gives +-lim, n == 0
// gives 0. lim |d| must be finite.
T divClamp( T n, T d, int k, T lim )
{
	T an = n < 0.0f ? -n : n;
	T ad = d < 0.0f ? -d : d;
	T as = ad > 0.0f ? ad : 1.0f;
	bool neg = ( n < 0.0f ) != ( d < 0.0f );
	T q = 0.0f;
	if ( an == 0.0f )
	{
		q = 0.0f;
	}
	else if ( an >= lim * ad )
	{
		q = lim;
	}
	else
	{
		q = an * recipPos( as );
	}
	q = neg ? -q : q;
	return snap( q );
}

// mn / mx for 0 <= mn <= mx, mx > 0 (atan2's ratio)
T ratio01( T mn, T mx ) { return snap( mn * recipPos( mx > 0.0f ? mx : 1.0f ) ); }

// V4's exponent bookkeeping (the solver's): F tracks none, so exponents and shifts are 0
int msbT( T x ) { return 0; }
int fmtV4( int e ) { return 0; }
int shiftOf( int sh ) { return 0; }

// 1/k for k > 0, else 0 (Box3D's effective masses); er is V4's exponent
T recipE( T k, int ek, out int er )
{
	er = 0;
	T ks = k > 0.0f ? k : 1.0f;
	T r = recipPos( ks );
	return snap( k > 0.0f ? r : 0.0f );
}

// sqrt(x) for x >= 0 (0 below): x times the software rsqrt
T sqrtT( T x, int sx )
{
	T xs = x > 0.0f ? x : 1.0f;
	T r = xs * rsqrtPos( xs );
	return snap( x > 0.0f ? r : 0.0f );
}

// The friction disc's scale (Box3D's b3SolveContacts): 1 when |(x, y)| <= m, else m / |(x, y)| through the software
// rsqrt (Box3D: m / (sqrt + FLT_EPSILON)); m >= 0
T discScale( T x, T y, T m, T one )
{
	x = snap( x );
	y = snap( y );
	m = snap( m );
	T lsq = x * x + y * y; // not snapped: squares of snapped values are at least 2^-60, and small ones still clamp
	T ls = lsq > 0.0f ? lsq : 1.0f;
	T r = one;
	if ( lsq > m * m )
	{
		r = snap( m * rsqrtPos( ls ) );
	}
	return r;
}

// =====================================================================================================================
// D: double (twin only)
// =====================================================================================================================
#elif defined( DIALECT_D )

// D keeps tiny values but sends -0 to +0, so a C++ compiler that folds x < 0 ? -x : x into fabs (MSVC does, in C)
// and one that does not still agree
T snap( T x ) { return x == 0.0l ? 0.0l : x; }
T M( T a, T b, int sh ) { return a * b; }
T sum2( T a, T b, T c, T d, int sh ) { return a * b + c * d; }
T dif2( T a, T b, T c, T d, int sh ) { return a * b - c * d; }
T sum3( T a, T b, T c, T d, T e, T f, int sh ) { return a * b + c * d + e * f; }
T rescale( T x, int sh ) { return x; }

// Every division and square root divides by (or roots) an input made safe before the test that discards it: clang 18
// on aarch64 evaluated 1 / d ahead of the select, and the sentinel saw divide-by-zero. The safe input is the value
// itself when it is at least the smallest normal double, else that (safePos), a different test from the select's
// (x > 0): clang-cl 23 folded x > 0 ? 1 / sqrt( x > 0 ? x : 1 ) : 0 into a speculated 1 / sqrt( x ). Results for
// normal inputs are unchanged.
static const double D_MIN = 2.2250738585072014e-308l; // the l suffix: an unsuffixed Slang literal is a float (0 here)
T safePos( T x ) { return x > D_MIN ? x : D_MIN; }

T rsqrtT( T x, int sx, int sy )
{
	T r = 1.0l / sqrt( safePos( x ) );
	return x > 0.0l ? r : 0.0l;
}

T recipT( T d, int sd, int sr )
{
	T ad = d < 0.0l ? -d : d;
	T ds = ad > D_MIN ? d : D_MIN;
	T r = 1.0l / ds;
	return d != 0.0l ? r : 0.0l;
}

T divClamp( T n, T d, int k, T lim )
{
	T an = n < 0.0l ? -n : n;
	T ad = d < 0.0l ? -d : d;
	T as = safePos( ad );
	bool neg = ( n < 0.0l ) != ( d < 0.0l );
	T q = 0.0l;
	if ( an == 0.0l )
	{
		q = 0.0l;
	}
	else if ( an >= lim * ad )
	{
		q = lim;
	}
	else
	{
		q = an / as;
	}
	return snap( neg ? -q : q );
}

T ratio01( T mn, T mx ) { return mn / safePos( mx ); }

int msbT( T x ) { return 0; }
int fmtV4( int e ) { return 0; }
int shiftOf( int sh ) { return 0; }

T recipE( T k, int ek, out int er )
{
	er = 0;
	T r = 1.0l / safePos( k );
	return k > 0.0l ? r : 0.0l;
}

T sqrtT( T x, int sx )
{
	T r = sqrt( safePos( x ) );
	return x > 0.0l ? r : 0.0l;
}

T discScale( T x, T y, T m, T one )
{
	T lsq = x * x + y * y;
	T r = one;
	if ( lsq > m * m )
	{
		r = m / sqrt( safePos( lsq ) );
	}
	return snap( r );
}

// =====================================================================================================================
// V4: block-scaled int32
// =====================================================================================================================
#elif defined( DIALECT_V4 )

T snap( T x ) { return x; }

// The one helper with int64 inside: a*b + c*d + e*f, shifted by sh in [0, 62] and rounded to nearest with halves away
// from zero, then narrowed to int32 with saturation at +-(2^31 - 1), counted. The exact sum stays below 2^62 in
// magnitude (the callers' formats). Symmetric: -x rounds to the negation of x's rounding (half up sent exact negative
// halves toward +inf, and an aligned stack's anchors at +-0.25 m make many: a slow creep, NOTES.md step 4b). The
// offset is 2^(sh-1) for x >= 0 and 2^(sh-1) - 1 for x < 0 (0 for sh = 0): (2^sh + (x >> 63)) >> 1, no select.
int mas( int a, int b, int c, int d, int e, int f, int sh )
{
	int64_t x = int64_t( a ) * int64_t( b ) + int64_t( c ) * int64_t( d ) + int64_t( e ) * int64_t( f );
	int64_t half = ( ( int64_t( 1 ) << sh ) + ( x >> 63 ) ) >> 1;
	int64_t y = ( x + half ) >> sh;
	if ( y > 2147483647LL || y < -2147483647LL )
	{
		countSaturation();
		return y > 0LL ? 2147483647 : -2147483647;
	}
	return int( y );
}

T M( T a, T b, int sh ) { return mas( a, b, 0, 0, 0, 0, sh ); }
T sum2( T a, T b, T c, T d, int sh ) { return mas( a, b, c, d, 0, 0, sh ); }
T dif2( T a, T b, T c, T d, int sh ) { return mas( a, b, -c, d, 0, 0, sh ); }
T sum3( T a, T b, T c, T d, T e, T f, int sh ) { return mas( a, b, c, d, e, f, sh ); }

// A format change by sh bits (the value is kept): sh > 0 rounds to nearest, halves away from zero (mas), sh < 0 (to
// -30) shifts left with saturation.
T rescale( T x, int sh )
{
	int s = sh < 0 ? 0 : sh;
	int m = sh < 0 ? ( 1 << ( -sh ) ) : 1;
	return mas( x, m, 0, 0, 0, 0, s );
}

// ---- 64-bit unsigned values as two words, for the exact division (no int64 outside mas) ----

struct W64
{
	uint lo;
	uint hi;
};

// x << k for k in [0, 63] (bits beyond 64 dropped)
W64 shl64( uint x, int k )
{
	W64 r;
	if ( k == 0 )
	{
		r.lo = x;
		r.hi = 0u;
	}
	else if ( k < 32 )
	{
		r.lo = x << uint( k );
		r.hi = x >> uint( 32 - k );
	}
	else
	{
		r.lo = 0u;
		r.hi = x << uint( k - 32 );
	}
	return r;
}

// The exact product of a, b in [0, 2^31): the low word wraps, the high word comes from mas. The product x = H 2^32 + L
// (0 <= L < 2^32) is never negative, so mas's rounding (halves away from zero) is floor( (x + 2^31) / 2^32 ) = H + (L >>
// 31): the high word is mas less the low word's top bit. (lpDivQ31's remainder N - q d multiplies q >= 0 by |d|, and its
// estimate's mas, which may be negative, is clamped at 0, where a negative half's rounding cannot show: lpDivQ31 gives
// the same words under either rounding.)
W64 mul64( int a, int b )
{
	W64 r;
	r.lo = uint( a ) * uint( b );
	r.hi = uint( mas( a, b, 0, 0, 0, 0, 32 ) ) - ( r.lo >> 31 );
	return r;
}

bool geq64( W64 a, W64 b ) { return a.hi > b.hi || ( a.hi == b.hi && a.lo >= b.lo ); }

W64 sub64( W64 a, W64 b )
{
	W64 r;
	r.lo = a.lo - b.lo;
	r.hi = a.hi - b.hi - ( a.lo < b.lo ? 1u : 0u );
	return r;
}

// lpDivQ31( n, d, k ): the exact quotient n 2^k / d rounded toward zero (C's integer division), for n, d in
// [-(2^31 - 1), 2^31 - 1] and k in [0, 32]. A quotient beyond +-(2^31 - 1), or d == 0 with n != 0, saturates and
// counts. The divisor is clz-normalised to dn in [2^30, 2^31); y ~ 2^30 / x for x = (dn/2 + 1) / 2^30 in (0.5, 1]
// (so y < 2 fits Q1.30) from the seed 48/17 - 32/17 x and three Newton steps; the estimate N' y / 2^61 - 8 (from
// below, N' = N 2^(s-1)) is within 31 of the quotient; the exact remainder N - q d (two words) then fixes it in five
// binary steps.
int lpDivQ31( int n, int d, int k )
{
	uint un = uint( n < 0 ? -n : n );
	uint ud = uint( d < 0 ? -d : d );
	bool neg = ( n < 0 ) != ( d < 0 );
	int q = 0;
	if ( un != 0u )
	{
		W64 N = shl64( un, k );
		if ( ud == 0u || geq64( N, shl64( ud, 31 ) ) ) // the quotient is 2^31 or more
		{
			countSaturation();
			q = 2147483647;
		}
		else
		{
			int s = clz( ud ); // 1..31
			uint dn = ud << uint( s - 1 );
			int X = int( ( dn >> 1 ) + 1u );
			int Y = int( 3031741621u - uint( M( 2021161080, X, 30 ) ) ); // 48/17 - 32/17 x, Q1.30
			[ForceUnroll]
			for ( int i = 0; i < 3; ++i )
			{
				int p = M( X, Y, 30 );
				int e = int( 0x80000000u - uint( p ) ); // 2 - x y
				Y = M( Y, e, 30 );
			}
			W64 Np = shl64( un, k + s - 1 ); // N' < dn 2^31 < 2^62
			int Nh = int( ( Np.hi << 1u ) | ( Np.lo >> 31 ) );
			int Nl = int( Np.lo & 0x7fffffffu );
			int t = M( Nl, Y, 31 );
			q = mas( Nh, Y, t, 1, 1 << 30, -8, 30 );
			q = q < 0 ? 0 : q;
			W64 R = sub64( N, mul64( q, int( ud ) ) );
			[ForceUnroll]
			for ( int b = 4; b >= 0; --b )
			{
				W64 B = shl64( ud, b );
				if ( geq64( R, B ) )
				{
					R = sub64( R, B );
					q = q + ( 1 << b );
				}
			}
		}
	}
	return neg ? -q : q;
}

// 1/d with d in format sd, the result in format sr (sr + sd - 30 in [0, 32]); saturates and counts when 1/d does
// not fit, or d == 0.
T recipT( T d, int sd, int sr ) { return lpDivQ31( 1 << 30, d, sr + sd - 30 ); }

// n 2^k / d clamped to [-lim, lim] (lim > 0): |n| 2^k >= lim |d| (d == 0 among them) gives +-lim without counting,
// n == 0 gives 0; otherwise lpDivQ31, which then cannot saturate.
T divClamp( T n, T d, int k, T lim )
{
	uint un = uint( n < 0 ? -n : n );
	uint ud = uint( d < 0 ? -d : d );
	bool neg = ( n < 0 ) != ( d < 0 );
	T q = 0;
	if ( un == 0u )
	{
		q = 0;
	}
	else if ( geq64( shl64( un, k ), mul64( lim, int( ud ) ) ) )
	{
		q = neg ? -lim : lim;
	}
	else
	{
		q = lpDivQ31( n, d, k );
	}
	return q;
}

// mn / mx in Q1.30 for 0 <= mn <= mx, mx > 0
T ratio01( T mn, T mx ) { return lpDivQ31( mn, mx, 30 ); }

// 1/sqrt(u) for u > 0 in format su (even), the result in format sy (0 for u <= 0). u is clz-normalised by an even
// shift t to x = u 2^t / 2^30 in [0.5, 2); z ~ 1/sqrt(x) in Q1.30 from the line 1.5604 - 0.4714 x (relative error
// below 13%) and four Newton steps z (3/2 - x z^2 / 2); then 1/sqrt(u 2^-su) = z 2^((t + su - 30) / 2).
T rsqrtQ( T u, int su, int sy )
{
	T r = 0;
	if ( u > 0 )
	{
		int p = 31 - clz( uint( u ) );
		int t = ( ( 29 - p ) & 1 ) == 0 ? 29 - p : 30 - p;
		int x = int( uint( u ) << uint( t ) );
		int z = 1675443764 - M( 506166750, x, 30 );
		[ForceUnroll]
		for ( int i = 0; i < 4; ++i )
		{
			int xz2 = M( M( x, z, 30 ), z, 31 ); // x z^2 / 2
			z = M( z, 1610612736 - xz2, 30 );	  // 3/2 - x z^2 / 2
		}
		int c = ( ( t + su - 30 ) >> 1 ) + sy - 30; // the result is z 2^c
		if ( c > 30 )
		{
			countSaturation();
			r = 2147483647;
		}
		else if ( c >= 0 )
		{
			r = mas( z, 1 << c, 0, 0, 0, 0, 0 );
		}
		else
		{
			r = c < -62 ? 0 : mas( z, 1, 0, 0, 0, 0, -c );
		}
	}
	return r;
}

T rsqrtT( T x, int sx, int sy ) { return rsqrtQ( x, sx, sy ); }

// ---- the solver's exponent bookkeeping (V4 only; F and D return 0) ----

// The position of |x|'s highest set bit (-1 for 0)
int msbT( T x ) { return 31 - clz( uint( x < 0 ? -x : x ) ); }

// An exponent kept for V4 (F and D keep none)
int fmtV4( int e ) { return e; }

// A shift for mas, computed in prepare: clamped to [1, 62], counted as a saturation when it is not
int shiftOf( int sh )
{
	int r = sh;
	if ( sh < 1 || sh > 62 )
	{
		countSaturation();
		r = sh < 1 ? 1 : 62;
	}
	return r;
}

// 1/k for k > 0 in format ek, as a mantissa in (2^29, 2^30] and its exponent er (the value is r 2^-er); 0 for k <= 0.
// With p = msb(k): 2^(30 + p) / k, exact to the truncation.
T recipE( T k, int ek, out int er )
{
	T r = 0;
	er = 0;
	if ( k > 0 )
	{
		int p = msbT( k );
		r = lpDivQ31( 1 << 30, k, p );
		er = 30 + p - ek;
	}
	return r;
}

// sqrt(x) for x >= 0 in format sx (even), the result in format sx: x times its rsqrt, which goes to a format sy where
// it has about 29 bits (sy = 29 - (sx + 1 - p) / 2, p = msb(x)); 0 for x <= 0
T sqrtT( T x, int sx )
{
	T r = 0;
	if ( x > 0 )
	{
		int p = msbT( x );
		int sy = 29 - ( ( sx + 1 - p ) >> 1 );
		T y = rsqrtQ( x, sx, sy );
		r = M( x, y, sy );
	}
	return r;
}

// The friction disc's scale in Q1.30: 1 when |(x, y)| <= m, else m / |(x, y)| (m >= 0; x, y, m in one format). The
// three are shifted together so the largest has its top bit at 27 (values below 1 in Q4.28), so the squares cannot
// overflow at any magnitude; |(x, y)|^2 is then in [1/4, 2) whenever it exceeds m^2, its rsqrt in Q2.29.
T discScale( T x, T y, T m, T one )
{
	int ax = x < 0 ? -x : x;
	int ay = y < 0 ? -y : y;
	int mx = ax > ay ? ax : ay;
	mx = m > mx ? m : mx;
	T r = one;
	if ( mx > 0 )
	{
		int s = clz( uint( mx ) ) - 4;
		T a = rescale( x, -s );
		T b = rescale( y, -s );
		T c = rescale( m, -s );
		T lsq = sum2( a, a, b, b, 28 );
		T c2 = M( c, c, 28 );
		if ( lsq > c2 )
		{
			T z = rsqrtQ( lsq, 28, 29 );
			r = M( c, z, 27 );
		}
	}
	return r;
}

#endif

// =====================================================================================================================
// Compares and selects (ternaries only; F snaps what it selects, so signed zeros and subnormals never survive)
// =====================================================================================================================

T selectT( bool c, T a, T b ) { return snap( c ? a : b ); }
T minT( T a, T b ) { return selectT( a < b, a, b ); }
T maxT( T a, T b ) { return selectT( a > b, a, b ); }
T absT( T a ) { return selectT( a < zero(), -a, a ); }
T clampT( T a, T lo, T hi ) { return selectT( a < lo, lo, selectT( a > hi, hi, a ) ); }

// =====================================================================================================================
// Vectors, matrices and quaternions in Box3D's operation order
// =====================================================================================================================

V3 v3( T x, T y, T z )
{
	V3 r;
	r.x = x;
	r.y = y;
	r.z = z;
	return r;
}
V3 zero3() { return v3( zero(), zero(), zero() ); }
V3 snap3( V3 a ) { return v3( snap( a.x ), snap( a.y ), snap( a.z ) ); }
V3 add3( V3 a, V3 b ) { return v3( a.x + b.x, a.y + b.y, a.z + b.z ); }
V3 sub3( V3 a, V3 b ) { return v3( a.x - b.x, a.y - b.y, a.z - b.z ); }
V3 mulSV( T s, V3 a, int sh ) { return v3( M( s, a.x, sh ), M( s, a.y, sh ), M( s, a.z, sh ) ); }
V3 mulAddSV( V3 a, T s, V3 b, int sh ) { return add3( a, mulSV( s, b, sh ) ); } // a + s * b
V3 mulSubSV( V3 a, T s, V3 b, int sh ) { return sub3( a, mulSV( s, b, sh ) ); } // a - s * b
T dot3( V3 a, V3 b, int sh ) { return sum3( a.x, b.x, a.y, b.y, a.z, b.z, sh ); }
V3 cross3( V3 a, V3 b, int sh )
{
	return v3( dif2( a.y, b.z, a.z, b.y, sh ), dif2( a.z, b.x, a.x, b.z, sh ), dif2( a.x, b.y, a.y, b.x, sh ) );
}
V3 mulMV( Sym3 m, V3 a, int sh )
{
	return v3( sum3( m.xx, a.x, m.xy, a.y, m.xz, a.z, sh ), sum3( m.xy, a.x, m.yy, a.y, m.yz, a.z, sh ),
			   sum3( m.xz, a.x, m.yz, a.y, m.zz, a.z, sh ) );
}

// A rotation matrix by rows, Q1.30 (not stored)
struct M3
{
	V3 r0;
	V3 r1;
	V3 r2;
};

// b3MakeMatrixFromQuat (rows of its column-major matrix). Doubling is an addition and 1 - 2 s is (1 - s) - s, so the
// V4 intermediates stay within Q1.30.
M3 matrixFromQuat( Q4 q, T one )
{
	T xy = dif2( q.x, q.y, q.z, q.s, S_Q ); // xy - zw
	T xyp = sum2( q.x, q.y, q.z, q.s, S_Q ); // xy + zw
	T xz = sum2( q.x, q.z, q.y, q.s, S_Q );	 // xz + yw
	T xzm = dif2( q.x, q.z, q.y, q.s, S_Q ); // xz - yw
	T yz = dif2( q.y, q.z, q.x, q.s, S_Q );	 // yz - xw
	T yzp = sum2( q.y, q.z, q.x, q.s, S_Q ); // yz + xw
	T syz = sum2( q.y, q.y, q.z, q.z, S_Q );
	T sxz = sum2( q.x, q.x, q.z, q.z, S_Q );
	T sxy = sum2( q.x, q.x, q.y, q.y, S_Q );
	M3 m;
	m.r0 = v3( ( one - syz ) - syz, xy + xy, xz + xz );
	m.r1 = v3( xyp + xyp, ( one - sxz ) - sxz, yz + yz );
	m.r2 = v3( xzm + xzm, yzp + yzp, ( one - sxy ) - sxy );
	return m;
}

V3 mulM3V( M3 m, V3 a ) { return v3( dot3( m.r0, a, S_Q ), dot3( m.r1, a, S_Q ), dot3( m.r2, a, S_Q ) ); }

// |m| a, row by row (the half extents of a rotated box)
V3 absM3V( M3 m, V3 a )
{
	return v3( sum3( absT( m.r0.x ), a.x, absT( m.r0.y ), a.y, absT( m.r0.z ), a.z, S_Q ),
			   sum3( absT( m.r1.x ), a.x, absT( m.r1.y ), a.y, absT( m.r1.z ), a.z, S_Q ),
			   sum3( absT( m.r2.x ), a.x, absT( m.r2.y ), a.y, absT( m.r2.z ), a.z, S_Q ) );
}

// R S R^T for a symmetric S (Box3D's world inertia); the format of S is kept
Sym3 rotateSym( M3 m, Sym3 s )
{
	// A = R S, by rows
	V3 a0 = v3( sum3( m.r0.x, s.xx, m.r0.y, s.xy, m.r0.z, s.xz, S_Q ), sum3( m.r0.x, s.xy, m.r0.y, s.yy, m.r0.z, s.yz, S_Q ),
				sum3( m.r0.x, s.xz, m.r0.y, s.yz, m.r0.z, s.zz, S_Q ) );
	V3 a1 = v3( sum3( m.r1.x, s.xx, m.r1.y, s.xy, m.r1.z, s.xz, S_Q ), sum3( m.r1.x, s.xy, m.r1.y, s.yy, m.r1.z, s.yz, S_Q ),
				sum3( m.r1.x, s.xz, m.r1.y, s.yz, m.r1.z, s.zz, S_Q ) );
	V3 a2 = v3( sum3( m.r2.x, s.xx, m.r2.y, s.xy, m.r2.z, s.xz, S_Q ), sum3( m.r2.x, s.xy, m.r2.y, s.yy, m.r2.z, s.yz, S_Q ),
				sum3( m.r2.x, s.xz, m.r2.y, s.yz, m.r2.z, s.zz, S_Q ) );
	a0 = snap3( a0 ); // F: a cancelled entry times another would be subnormal
	a1 = snap3( a1 );
	a2 = snap3( a2 );
	Sym3 r;
	r.xx = snap( dot3( a0, m.r0, S_Q ) );
	r.xy = snap( dot3( a0, m.r1, S_Q ) );
	r.xz = snap( dot3( a0, m.r2, S_Q ) );
	r.yy = snap( dot3( a1, m.r1, S_Q ) );
	r.yz = snap( dot3( a1, m.r2, S_Q ) );
	r.zz = snap( dot3( a2, m.r2, S_Q ) );
	return r;
}

Q4 q4( T x, T y, T z, T s )
{
	Q4 r;
	r.x = x;
	r.y = y;
	r.z = z;
	r.s = s;
	return r;
}

Q4 quatIdentity( T one ) { return q4( zero(), zero(), zero(), one ); }

// b3MulQuat
Q4 quatMul( Q4 q1, Q4 q2 )
{
	V3 v1 = v3( q1.x, q1.y, q1.z );
	V3 v2 = v3( q2.x, q2.y, q2.z );
	V3 t1 = cross3( v1, v2, S_Q );
	V3 t2 = mulAddSV( t1, q1.s, v2, S_Q );
	V3 t3 = mulAddSV( t2, q2.s, v1, S_Q );
	return q4( t3.x, t3.y, t3.z, M( q1.s, q2.s, S_Q ) - dot3( v1, v2, S_Q ) );
}

// b3RotateVector: v + 2 cross(q.v, cross(q.v, v) + q.s v); v keeps its format
V3 rotate( Q4 q, V3 v )
{
	V3 qv = v3( q.x, q.y, q.z );
	V3 t1 = cross3( qv, v, S_Q );
	V3 t2 = mulAddSV( t1, q.s, v, S_Q );
	V3 t3 = cross3( qv, t2, S_Q );
	return add3( v, add3( t3, t3 ) );
}

// b3InvRotateVector: v + 2 cross(q.v, cross(q.v, v) - q.s v)
V3 invRotate( Q4 q, V3 v )
{
	V3 qv = v3( q.x, q.y, q.z );
	V3 t1 = cross3( qv, v, S_Q );
	V3 t2 = mulSubSV( t1, q.s, v, S_Q );
	V3 t3 = cross3( qv, t2, S_Q );
	return add3( v, add3( t3, t3 ) );
}

// b3NormalizeQuat with the software rsqrt (|q|^2 below 2 in V4); a zero quaternion gives the identity. F snaps the
// components first: a sum of products (dq + h w dq / 2) can leave one near 1e-21, whose square is subnormal.
Q4 quatNormalize( Q4 q, T one )
{
	q = q4( snap( q.x ), snap( q.y ), snap( q.z ), snap( q.s ) );
	T x = sum3( q.x, q.x, q.y, q.y, q.z, q.z, S_Q ) + M( q.s, q.s, S_Q );
	Q4 r = quatIdentity( one );
	if ( x > zero() )
	{
		T y = rsqrtT( x, S_Q, S_Q );
		r = q4( snap( M( q.x, y, S_Q ) ), snap( M( q.y, y, S_Q ) ), snap( M( q.z, y, S_Q ) ), snap( M( q.s, y, S_Q ) ) );
	}
	return r;
}

// atan2( y, x ) as lpAtan2 (Box3D's b3Atan2: a minimax polynomial on [0, 1], then the octant), with snapped inputs;
// V4 takes y and x in one format and returns Q2.29. atan2( 0, 0 ) = 0.
T atan2T( T y, T x )
{
	y = snap( y );
	x = snap( x );
	T r = zero();
	if ( !( x == zero() && y == zero() ) )
	{
		T ax = absT( x );
		T ay = absT( y );
		T mx = maxT( ay, ax );
		T mn = minT( ay, ax );
		T a = ratio01( mn, mx );
		T s = M( a, a, S_Q );
		T c = M( s, a, S_Q );
		T q = M( s, s, S_Q );
		T rr = M( KT( 0.024840285f, 0.024840285l, 26672053 ), q, S_Q ) + KT( 0.18681418f, 0.18681418l, 200590198 );
		T t = M( KT( -0.094097948f, -0.094097948l, -101036902 ), q, S_Q ) - KT( 0.33213072f, 0.33213072l, 356622645 );
		rr = M( rr, s, S_Q ) + t;
		rr = M( rr, c, S_Q ) + a;
		rr = rescale( rr, S_Q - S_A );
		// the octant: pi / 2 and pi as F's lpAtan2 has them, D's to double precision, V4's in Q2.29
		rr = ay > ax ? KT( 1.57079637f, 1.5707963267948966l, 843314857 ) - rr : rr;
		rr = x < zero() ? KT( 3.14159274f, 3.1415926535897932l, 1686629713 ) - rr : rr;
		rr = y < zero() ? -rr : rr;
		r = snap( rr );
	}
	return r;
}

// =====================================================================================================================
// The joint's helpers (step 6)
// =====================================================================================================================

// The twist angle about z of a relative rotation q (Box3D's b3GetTwistAngle): 2 atan2( qz, qs ), qs's sign taken out
// first, so the angle is in [-pi, pi]. Q3.28 (V4: atan2T's Q2.29 doubled is the same mantissa in Q3.28, exactly); the
// inputs are snapped (atan2T).
T twistAngle( Q4 q )
{
	bool neg = q.s < zero();
	T y = selectT( neg, -q.z, q.z );
	T x = selectT( neg, -q.s, q.s );
	T a = atan2T( y, x );
	return rescale( a + a, S_A - S_JA );
}

// The inverse of a symmetric positive definite 3x3 matrix by Gaussian elimination with reciprocal pivots (A = L D L^T:
// each pivot's reciprocal through divClamp, the multipliers as products, then A^-1 = L^-T D^-1 L^-1). V4: a in Q1.30
// with its largest diagonal entry in [0.5, 1) (the caller normalises), the reciprocal pivots, the multipliers and the
// inverse in Q9.22 (S_J): a pivot under 2^-9 of the largest diagonal (a condition near 512) saturates, and a pivot that
// is not positive or whose reciprocal reaches the clamp is counted as a saturation. F and D: plain arithmetic.
Sym3 invertSym( Sym3 a, T one )
{
	T lim = KT( 1.0e20f, 1.0e20l, 2147483647 );
	T rp1 = divClamp( one, a.xx, S_J, lim );
	T l21 = snap( M( a.xy, rp1, S_Q ) );
	T l31 = snap( M( a.xz, rp1, S_Q ) );
	T d2 = snap( a.yy - M( a.xy, l21, S_J ) );
	T e2 = snap( a.yz - M( a.xz, l21, S_J ) );
	T f2 = snap( a.zz - M( a.xz, l31, S_J ) );
	T rp2 = divClamp( one, d2, S_J, lim );
	T l32 = snap( M( e2, rp2, S_Q ) );
	T d3 = snap( f2 - M( e2, l32, S_J ) );
	T rp3 = divClamp( one, d3, S_J, lim );
	bool bad = !( a.xx > zero() ) || !( d2 > zero() ) || !( d3 > zero() );
	bad = bad || !( rp1 < lim ) || !( rp2 < lim ) || !( rp3 < lim );
	if ( bad )
	{
		countSaturation();
	}
	// L^-1 = [1 0 0; m21 1 0; m31 m32 1]
	T m21 = -l21;
	T m32 = -l32;
	T m31 = snap( M( l21, l32, S_J ) - l31 );
	Sym3 r;
	r.zz = rp3;
	r.yz = snap( M( m32, rp3, S_J ) );
	r.xz = snap( M( m31, rp3, S_J ) );
	r.yy = snap( rp2 + M( m32, r.yz, S_J ) );
	r.xy = snap( sum2( m21, rp2, m31, r.yz, S_J ) );
	r.xx = snap( rp1 + sum2( m21, snap( M( m21, rp2, S_J ) ), m31, r.xz, S_J ) );
	return r;
}

// =====================================================================================================================
// Positions and the broadphase grid
// =====================================================================================================================

#if defined( DIALECT_V4 )

// p + d for d in Q5.26: d's two words are d << 6 (S_P - S_DP), with a carry between the words
Pos3 posAdd( Pos3 p, V3 d )
{
	Pos3 r;
	uint lo = uint( d.x ) << 6u;
	r.xlo = p.xlo + lo;
	r.xhi = p.xhi + ( d.x >> 26 ) + ( r.xlo < lo ? 1 : 0 );
	lo = uint( d.y ) << 6u;
	r.ylo = p.ylo + lo;
	r.yhi = p.yhi + ( d.y >> 26 ) + ( r.ylo < lo ? 1 : 0 );
	lo = uint( d.z ) << 6u;
	r.zlo = p.zlo + lo;
	r.zhi = p.zhi + ( d.z >> 26 ) + ( r.zlo < lo ? 1 : 0 );
	return r;
}

// floor( p 2^10 ) from the two words
int gridOf( uint lo, int hi ) { return int( uint( hi ) << uint( S_G ) ) | int( lo >> uint( S_P - S_G ) ); }

// b - a for one coordinate, in Q8.24: the two words' difference rounded half up (+128 at bit 7, with the carry);
// beyond +-127 m it saturates and counts
int posDeltaWord( uint alo, int ahi, uint blo, int bhi )
{
	uint lo = blo - alo;
	int hi = bhi - ahi - ( blo < alo ? 1 : 0 );
	uint lr = lo + 128u;
	hi = hi + ( lr < lo ? 1 : 0 );
	int r = 0;
	if ( hi > 127 || hi < -127 )
	{
		countSaturation();
		r = hi > 0 ? 2147483647 : -2147483647;
	}
	else
	{
		r = int( ( uint( hi ) << 24u ) | ( lr >> 8u ) );
	}
	return r;
}

// b - a in Q8.24 (the narrowphase's relative position)
V3 posDelta( Pos3 a, Pos3 b )
{
	return v3( posDeltaWord( a.xlo, a.xhi, b.xlo, b.xhi ), posDeltaWord( a.ylo, a.yhi, b.ylo, b.yhi ),
			   posDeltaWord( a.zlo, a.zhi, b.zlo, b.zhi ) );
}

// The cells [floor(p + lo), floor(p + hi)] of p + [lo, hi] (offsets Q8.24), conservatively: floor(p) + floor(lo) and
// floor(p) + floor(hi) + 1
int2 gridSpan( uint plo, int phi, T lo, T hi )
{
	int g = gridOf( plo, phi );
	return int2( g + ( lo >> ( S_R - S_G ) ), g + ( hi >> ( S_R - S_G ) ) + 1 );
}

int2 gridSpanX( Pos3 p, T lo, T hi ) { return gridSpan( p.xlo, p.xhi, lo, hi ); }
int2 gridSpanY( Pos3 p, T lo, T hi ) { return gridSpan( p.ylo, p.yhi, lo, hi ); }
int2 gridSpanZ( Pos3 p, T lo, T hi ) { return gridSpan( p.zlo, p.zhi, lo, hi ); }

#else

Pos3 posAdd( Pos3 p, V3 d )
{
	Pos3 r;
	r.x = snap( p.x + d.x );
	r.y = snap( p.y + d.y );
	r.z = snap( p.z + d.z );
	return r;
}

// floor( x ) for |x| < 2^31: truncation, one less when that rounded up
int floorI( T x )
{
	int t = int( x );
	return T( t ) > x ? t - 1 : t;
}

int2 gridSpan( T p, T lo, T hi )
{
	T g = KT( 1024.0f, 1024.0l, 1024 );
	int c = floorI( p * g );
	return int2( c + floorI( lo * g ), c + floorI( hi * g ) + 1 );
}

int2 gridSpanX( Pos3 p, T lo, T hi ) { return gridSpan( p.x, lo, hi ); }
int2 gridSpanY( Pos3 p, T lo, T hi ) { return gridSpan( p.y, lo, hi ); }
int2 gridSpanZ( Pos3 p, T lo, T hi ) { return gridSpan( p.z, lo, hi ); }

// b - a (Box3D's (float)( B.p.x - A.p.x ) in F)
V3 posDelta( Pos3 a, Pos3 b ) { return snap3( v3( b.x - a.x, b.y - a.y, b.z - a.z ) ); }

#endif

// =====================================================================================================================
// The narrowphase's frames and unit vectors
// =====================================================================================================================

Q4 quatConj( Q4 q ) { return q4( -q.x, -q.y, -q.z, q.s ); }

M3 snapM3( M3 m )
{
	M3 r;
	r.r0 = snap3( m.r0 );
	r.r1 = snap3( m.r1 );
	r.r2 = snap3( m.r2 );
	return r;
}

// R^T (exact: the entries move)
M3 transposeM3( M3 m )
{
	M3 r;
	r.r0 = v3( m.r0.x, m.r1.x, m.r2.x );
	r.r1 = v3( m.r0.y, m.r1.y, m.r2.y );
	r.r2 = v3( m.r0.z, m.r1.z, m.r2.z );
	return r;
}

// R^T a; a keeps its format
V3 mulM3TV( M3 m, V3 a )
{
	return v3( sum3( m.r0.x, a.x, m.r1.x, a.y, m.r2.x, a.z, S_Q ), sum3( m.r0.y, a.x, m.r1.y, a.y, m.r2.y, a.z, S_Q ),
			   sum3( m.r0.z, a.x, m.r1.z, a.y, m.r2.z, a.z, S_Q ) );
}

#if defined( DIALECT_V4 )
// a / |a| in Q1.30 for an integer vector in any format (the format cancels); zero for a zero vector. The largest
// component is shifted into [2^28, 2^29) (exactly; rounded when it has 30 or 31 bits), so |a'|^2 / 2^30 lies in
// [1/16, 3/4] and its rsqrt in [1.15, 4] (Q3.28): 30 bits at any length.
V3 unit3( V3 a )
{
	int ax = a.x < 0 ? -a.x : a.x;
	int ay = a.y < 0 ? -a.y : a.y;
	int az = a.z < 0 ? -a.z : a.z;
	int m = ax > ay ? ax : ay;
	m = az > m ? az : m;
	V3 r = zero3();
	if ( m > 0 )
	{
		int s = clz( uint( m ) ) - 3; // m << s has its top bit at 28
		V3 b = a;
		if ( s > 0 )
		{
			b = v3( int( uint( a.x ) << uint( s ) ), int( uint( a.y ) << uint( s ) ), int( uint( a.z ) << uint( s ) ) );
		}
		else if ( s < 0 )
		{
			b = v3( rescale( a.x, -s ), rescale( a.y, -s ), rescale( a.z, -s ) );
		}
		T x = dot3( b, b, 30 );
		T y = rsqrtQ( x, 30, 28 );
		r = v3( M( b.x, y, 28 ), M( b.y, y, 28 ), M( b.z, y, 28 ) );
	}
	return r;
}
#else
// a / |a| through the software rsqrt; zero for a zero vector (Box3D's b3Normalize gives zero below FLT_EPSILON)
V3 unit3( V3 a )
{
	T x = snap( dot3( a, a, 0 ) );
	T y = rsqrtT( x, 0, 0 );
	return snap3( mulSV( y, a, 0 ) );
}
#endif

// |a| in a's format
T len3( V3 a ) { return snap( dot3( a, unit3( a ), S_Q ) ); }

// A change of frame x -> R x + p: B's local frame into A's (frameBtoA), or back (invFrame)
struct Frame
{
	M3 R; // Q1.30
	V3 p; // Q8.24
};

V3 toFrame( Frame f, V3 x ) { return snap3( add3( mulM3V( f.R, x ), f.p ) ); }
V3 rotFrame( Frame f, V3 x ) { return snap3( mulM3V( f.R, x ) ); }
V3 unrotFrame( Frame f, V3 x ) { return snap3( mulM3TV( f.R, x ) ); }

// b3InvMulWorldTransforms: B in A's frame, q = qA* qB and p = qA* (pB - pA), as a matrix
Frame frameBtoA( BodyPose a, BodyPose b, T one )
{
	Q4 q = quatMul( quatConj( a.q ), b.q );
	Frame f;
	f.R = snapM3( matrixFromQuat( q4( snap( q.x ), snap( q.y ), snap( q.z ), snap( q.s ) ), one ) );
	f.p = snap3( invRotate( a.q, posDelta( a.p, b.p ) ) );
	return f;
}

// The inverse frame: R^T and -R^T p
Frame invFrame( Frame f )
{
	Frame g;
	g.R = transposeM3( f.R );
	V3 t = unrotFrame( f, f.p );
	g.p = snap3( v3( -t.x, -t.y, -t.z ) );
	return g;
}

// =====================================================================================================================
// Hashing: two 32-bit murmur3 lanes make one 64-bit element hash (no 64-bit integer in the kernels)
// =====================================================================================================================

struct H2
{
	uint a;
	uint b;
};

uint rotl32( uint x, uint r ) { return ( x << r ) | ( x >> ( 32u - r ) ); }

uint murmurStep( uint h, uint w )
{
	uint k = w * 0xcc9e2d51u;
	k = rotl32( k, 15u );
	k = k * 0x1b873593u;
	h = h ^ k;
	h = rotl32( h, 13u );
	return h * 5u + 0xe6546b64u;
}

uint murmurFinal( uint h )
{
	h = h ^ ( h >> 16 );
	h = h * 0x85ebca6bu;
	h = h ^ ( h >> 13 );
	h = h * 0xc2b2ae35u;
	return h ^ ( h >> 16 );
}

// The element's index and category seed both lanes, so an order-free sum still tells elements apart.
H2 hashStart( uint category, uint index )
{
	H2 h;
	h.a = murmurStep( 0x243f6a88u ^ category, index );
	h.b = murmurStep( 0x85a308d3u ^ ( category * 0x9e3779b9u ), index ^ 0x5bd1e995u );
	return h;
}

H2 hashWord( H2 h, uint w )
{
	H2 r;
	r.a = murmurStep( h.a, w );
	r.b = murmurStep( h.b, w ^ 0x9e3779b9u );
	return r;
}

H2 hashT( H2 h, T x )
{
#if defined( DIALECT_F )
	return hashWord( h, asuint( x ) );
#elif defined( DIALECT_V4 )
	return hashWord( h, uint( x ) );
#else
	uint64_t bits = bit_cast<uint64_t>( x );
	return hashWord( hashWord( h, uint( bits ) ), uint( bits >> 32 ) );
#endif
}

H2 hashV3( H2 h, V3 v ) { return hashT( hashT( hashT( h, v.x ), v.y ), v.z ); }
H2 hashQ4( H2 h, Q4 q ) { return hashT( hashT( hashT( hashT( h, q.x ), q.y ), q.z ), q.s ); }

H2 hashPos( H2 h, Pos3 p )
{
#if defined( DIALECT_V4 )
	h = hashWord( hashWord( h, p.xlo ), uint( p.xhi ) );
	h = hashWord( hashWord( h, p.ylo ), uint( p.yhi ) );
	return hashWord( hashWord( h, p.zlo ), uint( p.zhi ) );
#else
	return hashT( hashT( hashT( h, p.x ), p.y ), p.z );
#endif
}

Hash2 hashEnd( H2 h )
{
	Hash2 r;
	r.lo = murmurFinal( h.a );
	r.hi = murmurFinal( h.b );
	return r;
}
