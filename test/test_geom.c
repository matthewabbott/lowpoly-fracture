// SPDX-License-Identifier: MIT
// Exact geometry (src/geom.h, src/int128.h): 128-bit integers on every path against the portable reference, canonical
// planes, the bit budget at its bounds against a 256-bit reference, the constructors and their ranges.

#include "geom.h"
#include "test_macros.h"

#include <stdlib.h>

#define A_MAX LP_PLANE_NORMAL_MAX
#define P_MAX LP_GRID_RANGE
#define D_MAX ( 3ll * A_MAX * P_MAX )

// ---- a 256-bit reference: two's complement on eight 32-bit limbs, wide enough for every product here ----

typedef struct Wide
{
	uint32_t l[8];
} Wide;

static Wide WideFrom128( lpI128 a )
{
	Wide w;
	uint32_t fill = ( a.hi >> 63 ) ? 0xFFFFFFFFu : 0u;
	w.l[0] = (uint32_t)a.lo;
	w.l[1] = (uint32_t)( a.lo >> 32 );
	w.l[2] = (uint32_t)a.hi;
	w.l[3] = (uint32_t)( a.hi >> 32 );
	for ( int i = 4; i < 8; ++i )
	{
		w.l[i] = fill;
	}
	return w;
}

static Wide WideFrom64( int64_t a )
{
	lpI128 x = { (uint64_t)a, a < 0 ? ~0ull : 0ull };
	return WideFrom128( x );
}

static Wide WideAdd( Wide a, Wide b )
{
	Wide r;
	uint64_t carry = 0;
	for ( int i = 0; i < 8; ++i )
	{
		uint64_t t = (uint64_t)a.l[i] + b.l[i] + carry;
		r.l[i] = (uint32_t)t;
		carry = t >> 32;
	}
	return r;
}

static Wide WideNeg( Wide a )
{
	Wide r;
	uint64_t carry = 1;
	for ( int i = 0; i < 8; ++i )
	{
		uint64_t t = (uint64_t)(uint32_t)~a.l[i] + carry;
		r.l[i] = (uint32_t)t;
		carry = t >> 32;
	}
	return r;
}

static Wide WideSub( Wide a, Wide b )
{
	return WideAdd( a, WideNeg( b ) );
}

static Wide WideMul( Wide a, Wide b )
{
	Wide r = { { 0 } };
	for ( int i = 0; i < 8; ++i )
	{
		uint64_t carry = 0;
		for ( int j = 0; i + j < 8; ++j )
		{
			uint64_t t = (uint64_t)a.l[i] * b.l[j] + r.l[i + j] + carry;
			r.l[i + j] = (uint32_t)t;
			carry = t >> 32;
		}
	}
	return r;
}

static int WideSign( Wide a )
{
	if ( a.l[7] >> 31 )
	{
		return -1;
	}
	uint32_t any = 0;
	for ( int i = 0; i < 8; ++i )
	{
		any |= a.l[i];
	}
	return any != 0 ? 1 : 0;
}

static bool WideEqual( Wide a, Wide b )
{
	return memcmp( a.l, b.l, sizeof( a.l ) ) == 0;
}

// a 2^k modulo 2^256 (0 <= k < 256)
static Wide WideShl( Wide a, int k )
{
	Wide p = { { 0 } };
	p.l[k / 32] = 1u << ( k % 32 );
	return WideMul( a, p );
}

static int WideCompare( Wide a, Wide b )
{
	return WideSign( WideSub( a, b ) );
}

// A finite double that is an integer or a dyadic rational, exactly, as K 2^t with K < 2^53 (K = 0 for zero)
static void DoubleParts( double d, uint64_t* k, int* t )
{
	uint64_t bits;
	memcpy( &bits, &d, sizeof( bits ) );
	int exponent = (int)( ( bits >> 52 ) & 0x7FFu );
	*k = bits & 0xFFFFFFFFFFFFFull;
	*t = exponent == 0 ? -1074 : exponent - 1075;
	*k |= exponent == 0 ? 0u : 1ull << 52;
}

// The bit length of |a|
static int WideBits( Wide a )
{
	if ( WideSign( a ) < 0 )
	{
		a = WideNeg( a );
	}
	for ( int i = 7; i >= 0; --i )
	{
		for ( int b = 31; b >= 0; --b )
		{
			if ( ( a.l[i] >> b ) & 1u )
			{
				return 32 * i + b + 1;
			}
		}
	}
	return 0;
}

// The determinant of a 3x3 matrix by expansion along its first row: a formula of its own, not geom.c's cross products
static Wide WideDet3( Wide m[3][3] )
{
	Wide a = WideSub( WideMul( m[1][1], m[2][2] ), WideMul( m[1][2], m[2][1] ) );
	Wide b = WideSub( WideMul( m[1][0], m[2][2] ), WideMul( m[1][2], m[2][0] ) );
	Wide c = WideSub( WideMul( m[1][0], m[2][1] ), WideMul( m[1][1], m[2][0] ) );
	return WideAdd( WideSub( WideMul( m[0][0], a ), WideMul( m[0][1], b ) ), WideMul( m[0][2], c ) );
}

// The reference vertex of three planes and its classification against a fourth, from determinants: W = det[n],
// X = det[d|ny|nz], Y = det[nx|d|nz], Z = det[nx|ny|d], S = n4.X - d4 W, signs normalised to W > 0
typedef struct WideVertex
{
	Wide x[3], w;
} WideVertex;

static bool WideVertexOf( const lpIPlane* p[3], WideVertex* v )
{
	Wide m[3][3];
	for ( int i = 0; i < 3; ++i )
	{
		for ( int j = 0; j < 3; ++j )
		{
			m[i][j] = WideFrom64( p[i]->n[j] );
		}
	}
	v->w = WideDet3( m );
	for ( int c = 0; c < 3; ++c )
	{
		Wide r[3][3];
		memcpy( r, m, sizeof( r ) );
		for ( int i = 0; i < 3; ++i )
		{
			r[i][c] = WideFrom64( p[i]->d );
		}
		v->x[c] = WideDet3( r );
	}
	int sign = WideSign( v->w );
	if ( sign < 0 )
	{
		v->w = WideNeg( v->w );
		for ( int c = 0; c < 3; ++c )
		{
			v->x[c] = WideNeg( v->x[c] );
		}
	}
	return sign != 0;
}

static Wide WideEvaluate( const WideVertex* v, const lpIPlane* p )
{
	Wide s = WideMul( v->x[0], WideFrom64( p->n[0] ) );
	s = WideAdd( s, WideMul( v->x[1], WideFrom64( p->n[1] ) ) );
	s = WideAdd( s, WideMul( v->x[2], WideFrom64( p->n[2] ) ) );
	return WideSub( s, WideMul( v->w, WideFrom64( p->d ) ) );
}

static Wide WideAbs( Wide a )
{
	return WideSign( a ) < 0 ? WideNeg( a ) : a;
}

// geom.c's vertex and classification equal the reference exactly (no overflow anywhere). The magnitudes go to mag:
// W, the largest numerator and the classification (zero when the planes do not meet).
static int CheckAgainstWide( const lpIPlane* p1, const lpIPlane* p2, const lpIPlane* p3, const lpIPlane* p4, Wide mag[3] )
{
	const lpIPlane* triple[3] = { p1, p2, p3 };
	WideVertex ref;
	bool refMet = WideVertexOf( triple, &ref );
	lpIVertex v;
	bool met = lpIVertex_FromPlanes( p1, p2, p3, &v );
	ENSURE( met == refMet );
	for ( int k = 0; k < 3; ++k )
	{
		mag[k] = WideFrom64( 0 );
	}
	if ( met == false )
	{
		return 0;
	}
	ENSURE( WideEqual( WideFrom128( v.w ), ref.w ) );
	ENSURE( WideEqual( WideFrom128( v.x ), ref.x[0] ) );
	ENSURE( WideEqual( WideFrom128( v.y ), ref.x[1] ) );
	ENSURE( WideEqual( WideFrom128( v.z ), ref.x[2] ) );
	Wide s = WideEvaluate( &ref, p4 );
	ENSURE( WideEqual( WideFrom128( lpIVertex_Evaluate( &v, p4 ) ), s ) );
	ENSURE( lpIVertex_Classify( &v, p4 ) == WideSign( s ) );
	mag[0] = ref.w;
	for ( int c = 0; c < 3; ++c )
	{
		Wide x = WideAbs( ref.x[c] );
		mag[1] = WideSign( WideSub( x, mag[1] ) ) > 0 ? x : mag[1];
	}
	mag[2] = WideAbs( s );
	return 0;
}

// The largest of each magnitude so far
static void KeepLargest( Wide largest[3], const Wide mag[3] )
{
	for ( int k = 0; k < 3; ++k )
	{
		largest[k] = WideSign( WideSub( mag[k], largest[k] ) ) > 0 ? mag[k] : largest[k];
	}
}

// ---- random inputs ----

static uint64_t Next64( lpRandom* rng )
{
	uint64_t hi = lpRandom_Next( rng );
	uint64_t lo = lpRandom_Next( rng );
	return hi << 32 | lo;
}

// Random int64 of a random bit length, half of them negative: edges and carries come up often
static int64_t RandomI64( lpRandom* rng )
{
	uint64_t x = Next64( rng );
	int bits = (int)( lpRandom_Next( rng ) % 65u );
	x = bits >= 64 ? x : x & ( ( 1ull << bits ) - 1u );
	return ( lpRandom_Next( rng ) & 1u ) ? (int64_t)( 0u - x ) : (int64_t)x;
}

static lpI128 RandomI128( lpRandom* rng )
{
	lpI128 r;
	r.lo = Next64( rng );
	r.hi = Next64( rng );
	int bits = (int)( lpRandom_Next( rng ) % 129u ); // keep the top `bits` bits' worth of magnitude
	if ( bits < 64 )
	{
		r.lo &= bits == 0 ? 0u : ~0ull >> ( 64 - bits );
		r.hi = 0;
	}
	else if ( bits < 128 )
	{
		r.hi &= bits == 64 ? 0u : ~0ull >> ( 128 - bits );
	}
	return ( lpRandom_Next( rng ) & 1u ) ? lpI128_NegPortable( r ) : r;
}

// An integer in [-m, m], near its ends a third of the time
static int64_t RandomIn( lpRandom* rng, int64_t m )
{
	uint64_t span = (uint64_t)m * 2u + 1u;
	uint32_t kind = lpRandom_Next( rng ) % 3u;
	int64_t x;
	if ( kind == 0 )
	{
		x = m - (int64_t)( lpRandom_Next( rng ) % 4u );
		x = ( lpRandom_Next( rng ) & 1u ) ? -x : x;
	}
	else
	{
		x = (int64_t)( Next64( rng ) % span ) - m;
	}
	return x;
}

// A plane in range (not canonical: the predicates need only the ranges), its offset near the edge of the range often
// A vector in [-1, 1]^3, its draws as statements (rule 13: an initializer's order is unspecified)
static lpVec3 RandomBox( lpRandom* rng )
{
	lpVec3 v;
	v.x = lpRandom_Range( rng, -1.0f, 1.0f );
	v.y = lpRandom_Range( rng, -1.0f, 1.0f );
	v.z = lpRandom_Range( rng, -1.0f, 1.0f );
	return v;
}

static lpIPlane RandomPlane( lpRandom* rng, int32_t maxNormal )
{
	lpIPlane p;
	do
	{
		for ( int i = 0; i < 3; ++i )
		{
			p.n[i] = (int32_t)RandomIn( rng, maxNormal );
		}
	}
	while ( p.n[0] == 0 && p.n[1] == 0 && p.n[2] == 0 );
	int64_t l1 = llabs( p.n[0] ) + llabs( p.n[1] ) + llabs( p.n[2] );
	p.d = RandomIn( rng, l1 * P_MAX );
	return p;
}

// ---- tests ----

static int CheckOps( lpI128 x, lpI128 y, int64_t a, int64_t b )
{
	ENSURE( lpI128_Equal( lpI128_Add( x, y ), lpI128_AddPortable( x, y ) ) );
	ENSURE( lpI128_Equal( lpI128_Sub( x, y ), lpI128_SubPortable( x, y ) ) );
	ENSURE( lpI128_Equal( lpI128_Neg( x ), lpI128_NegPortable( x ) ) );
	ENSURE( lpI128_Compare( x, y ) == lpI128_ComparePortable( x, y ) );
	ENSURE( lpI128_Compare( y, x ) == -lpI128_Compare( x, y ) );
	ENSURE( lpI128_Sign( x ) == lpI128_SignPortable( x ) );
	ENSURE( lpI128_Equal( lpI128_Mul( x, a ), lpI128_MulPortable( x, a ) ) );
	ENSURE( lpI128_Equal( lpI128_Mul64( a, b ), lpI128_Mul64Portable( a, b ) ) );

	// The reference against the 256-bit one: products of int64s exactly, int128 x int64 modulo 2^128 (the low limbs)
	ENSURE( WideEqual( WideFrom128( lpI128_Mul64( a, b ) ), WideMul( WideFrom64( a ), WideFrom64( b ) ) ) );
	Wide product = WideMul( WideFrom128( x ), WideFrom64( a ) );
	lpI128 low = lpI128_Mul( x, a );
	ENSURE( memcmp( WideFrom128( low ).l, product.l, 4 * sizeof( uint32_t ) ) == 0 );
	Wide sum = WideAdd( WideFrom128( x ), WideFrom128( y ) );
	Wide difference = WideSub( WideFrom128( x ), WideFrom128( y ) );
	ENSURE( memcmp( WideFrom128( lpI128_Add( x, y ) ).l, sum.l, 4 * sizeof( uint32_t ) ) == 0 );
	ENSURE( memcmp( WideFrom128( lpI128_Sub( x, y ) ).l, difference.l, 4 * sizeof( uint32_t ) ) == 0 );
	ENSURE( lpI128_Compare( x, y ) == WideSign( difference ) );
	ENSURE( lpI128_Sign( x ) == WideSign( WideFrom128( x ) ) );

	// C2's: the left shift (modulo 2^128, every k), and the conversion to double within 2^-51 of the value, exactly
	int k = (int)( (uint64_t)a % 128u );
	ENSURE( lpI128_Equal( lpI128_Shl( x, k ), lpI128_ShlPortable( x, k ) ) );
	ENSURE( memcmp( WideFrom128( lpI128_Shl( x, k ) ).l, WideShl( WideFrom128( x ), k ).l, 4 * sizeof( uint32_t ) ) == 0 );
	double d = lpI128_ToDouble( x );
	Wide dw = WideFrom64( 0 );
	if ( d != 0.0 )
	{
		uint64_t significand;
		int exponent;
		DoubleParts( d < 0.0 ? -d : d, &significand, &exponent );
		ENSURE( exponent >= -52 );
		ENSURE( exponent >= 0 || ( significand & ( ( 1ull << -exponent ) - 1u ) ) == 0 ); // an integer, as x is
		dw = exponent >= 0 ? WideShl( WideFrom64( (int64_t)significand ), exponent )
						   : WideFrom64( (int64_t)( significand >> -exponent ) );
		dw = d < 0.0 ? WideNeg( dw ) : dw;
	}
	Wide error = WideAbs( WideSub( dw, WideFrom128( x ) ) );
	ENSURE( WideCompare( WideShl( error, 51 ), WideAbs( WideFrom128( x ) ) ) <= 0 );
	ENSURE( ( d < 0.0 ) == ( lpI128_Sign( x ) < 0 ) && ( d == 0.0 ) == lpI128_IsZero( x ) );
	return 0;
}

// Every operation of this build's path against the portable reference, and both against the 256-bit one: edges
// (0, +-1, int64's extremes, carries across 2^32 and 2^64, int128's extremes) and seeded random inputs
static int TestInt128( void )
{
	printf( "  lpI128 path: %s\n", lpI128_Path() );
	const int64_t edges64[] = { 0,
								1,
								-1,
								2,
								-2,
								INT64_MAX,
								INT64_MIN,
								INT64_MIN + 1,
								INT64_MAX - 1,
								4294967295ll,
								4294967296ll,
								-4294967296ll,
								3037000499ll,
								-3037000500ll,
								1ll << 62,
								-( 1ll << 62 ),
								A_MAX,
								-A_MAX,
								D_MAX,
								-D_MAX };
	const uint64_t words[] = { 0, 1, 0xFFFFFFFFull, 0x100000000ull, 0x7FFFFFFFFFFFFFFFull, 0x8000000000000000ull,
							   0xFFFFFFFFFFFFFFFFull, 0xFFFFFFFF00000000ull };
	enum
	{
		lp_edges64 = sizeof( edges64 ) / sizeof( edges64[0] ),
		lp_words = sizeof( words ) / sizeof( words[0] )
	};
	lpI128 edges[lp_words * lp_words];
	int edgeCount = 0;
	for ( int i = 0; i < lp_words; ++i )
	{
		for ( int j = 0; j < lp_words; ++j )
		{
			edges[edgeCount].lo = words[i];
			edges[edgeCount++].hi = words[j];
		}
	}
	for ( int i = 0; i < edgeCount; ++i )
	{
		for ( int j = 0; j < edgeCount; ++j )
		{
			ENSURE( CheckOps( edges[i], edges[j], edges64[( i + j ) % lp_edges64], edges64[( i * 7 + j ) % lp_edges64] ) ==
					0 );
		}
	}
	for ( int i = 0; i < lp_edges64; ++i )
	{
		for ( int j = 0; j < lp_edges64; ++j )
		{
			ENSURE( CheckOps( lpI128_FromI64( edges64[i] ), edges[( i * 3 + j ) % edgeCount], edges64[i], edges64[j] ) ==
					0 );
		}
	}

	// Known values: carries, int64's extremes squared, the top of int128
	lpI128 allOnes = { ~0ull, 0 };
	ENSURE( lpI128_Equal( lpI128_Add( allOnes, lpI128_FromI64( 1 ) ), ( lpI128 ){ 0, 1 } ) );
	ENSURE( lpI128_Equal( lpI128_Sub( ( lpI128 ){ 0, 1 }, lpI128_FromI64( 1 ) ), allOnes ) );
	ENSURE( lpI128_Equal( lpI128_Mul64( INT64_MIN, INT64_MIN ), ( lpI128 ){ 0, 1ull << 62 } ) );
	ENSURE( lpI128_Equal( lpI128_Mul64( INT64_MAX, INT64_MAX ), ( lpI128 ){ 1, 0x3FFFFFFFFFFFFFFFull } ) );
	ENSURE( lpI128_Equal( lpI128_Mul64( -1, INT64_MIN ), ( lpI128 ){ 1ull << 63, 0 } ) );
	lpI128 top = { ~0ull, 0x7FFFFFFFFFFFFFFFull };
	lpI128 bottom = { 0, 0x8000000000000000ull };
	ENSURE( lpI128_Compare( bottom, top ) < 0 && lpI128_Sign( bottom ) < 0 && lpI128_Sign( top ) > 0 );
	ENSURE( lpI128_Equal( lpI128_Add( top, lpI128_FromI64( 1 ) ), bottom ) ); // wraps, as documented
	ENSURE( lpI128_Equal( lpI128_Mul( lpI128_Mul64( INT64_MIN, 4 ), INT64_MIN ), ( lpI128 ){ 0, 0 } ) ); // 2^128 wraps

	// The shift at every k, on every edge word pair
	for ( int i = 0; i < edgeCount; ++i )
	{
		for ( int k = 0; k < 128; ++k )
		{
			ENSURE( lpI128_Equal( lpI128_Shl( edges[i], k ), lpI128_ShlPortable( edges[i], k ) ) );
			ENSURE( memcmp( WideFrom128( lpI128_Shl( edges[i], k ) ).l, WideShl( WideFrom128( edges[i] ), k ).l,
							4 * sizeof( uint32_t ) ) == 0 );
		}
	}
	ENSURE( lpI128_Equal( lpI128_Shl( lpI128_FromI64( 1 ), 127 ), bottom ) );
	ENSURE( lpI128_ToDouble( bottom ) == -1.7014118346046923e38 && lpI128_ToDouble( top ) == 1.7014118346046923e38 );
	ENSURE( lpI128_ToDouble( lpI128_FromI64( -1 ) ) == -1.0 && lpI128_ToDouble( allOnes ) == 18446744073709551616.0 );

	// Seeded random inputs of every magnitude
	lpRandom rng;
	lpRandom_Seed( &rng, 0x128128ull, 3 );
	for ( int i = 0; i < 300000; ++i )
	{
		lpI128 x = RandomI128( &rng );
		lpI128 y = RandomI128( &rng );
		int64_t a = RandomI64( &rng );
		int64_t b = RandomI64( &rng );
		ENSURE( CheckOps( x, y, a, b ) == 0 );
	}
	return 0;
}

static int CompareForSort( const void* a, const void* b )
{
	return lpIPlane_Compare( (const lpIPlane*)a, (const lpIPlane*)b );
}

// Reduction, orientation, exact negation, equality and the total order
static int TestPlaneCanonical( void )
{
	lpIPlane p, q;
	lpGeomReject why;

	// Reduction: n = 2 (b - a), d = |b|^2 - |a|^2 divided by their gcd, doubled back only where d would be a half
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ 0, 0, 0 }, (int32_t[3]){ 4, 0, 0 }, &p, &why ) && why == lp_geomOk );
	ENSURE( p.n[0] == 1 && p.n[1] == 0 && p.n[2] == 0 && p.d == 2 );
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ 0, 0, 0 }, (int32_t[3]){ 1, 0, 0 }, &p, NULL ) );
	ENSURE( p.n[0] == 2 && p.n[1] == 0 && p.n[2] == 0 && p.d == 1 );
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ 6, -3, 9 }, (int32_t[3]){ 12, 3, 9 }, &p, NULL ) );
	ENSURE( p.n[0] == 1 && p.n[1] == 1 && p.n[2] == 0 && p.d == 9 ); // midpoint (9, 0, 9)
	ENSURE( lpIPlane_IsValid( &p ) );

	// Equal half-spaces from different constructions are equal field by field
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ 0, 5, 1 }, (int32_t[3]){ 2, 5, 1 }, &p, NULL ) );
	ENSURE( lpIPlane_MakeSnapped( (lpVec3){ 0.25f, 0.0f, 0.0f }, 9, (int32_t[3]){ 1, -70, 3 }, &q, NULL ) );
	ENSURE( lpIPlane_Equal( &p, &q ) && lpIPlane_Compare( &p, &q ) == 0 );
	ENSURE( lpIPlane_MakeSnapped( (lpVec3){ 3.0f, 0.0f, 0.0f }, 1, (int32_t[3]){ 1, 8, 8 }, &q, NULL ) );
	ENSURE( lpIPlane_Equal( &p, &q ) );

	lpRandom rng;
	lpRandom_Seed( &rng, 77, 5 );
	enum
	{
		lp_count = 600
	};
	lpIPlane* planes = malloc( 3 * lp_count * sizeof( lpIPlane ) );
	int count = 0;
	for ( int i = 0; i < lp_count; ++i )
	{
		int32_t a[3], b[3];
		int32_t range = ( i % 3 == 0 ) ? P_MAX : 64; // small ranges repeat planes, for the order's ties
		for ( int k = 0; k < 3; ++k )
		{
			a[k] = (int32_t)RandomIn( &rng, range );
			b[k] = (int32_t)RandomIn( &rng, range );
		}
		if ( lpIPlane_MakeBisector( a, b, &p, NULL ) == false )
		{
			continue;
		}
		// Orientation and exact negation: a inside, b outside; the bisector from b's side is the exact negation, and
		// canonical too
		ENSURE( lpIPlane_ClassifyPoint( &p, a ) < 0 && lpIPlane_ClassifyPoint( &p, b ) > 0 );
		ENSURE( lpIPlane_MakeBisector( b, a, &q, NULL ) );
		lpIPlane negated = lpIPlane_Negate( p );
		ENSURE( lpIPlane_Equal( &q, &negated ) );
		ENSURE( lpIPlane_IsValid( &p ) && lpIPlane_IsValid( &q ) );
		planes[count++] = p;
		if ( i % 5 == 0 )
		{
			planes[count++] = lpIPlane_Negate( q ); // made again from the other side: a tie for the order
		}

		// A snapped normal and its negation give exactly negated planes (rounding half away from zero is symmetric)
		lpVec3 n = RandomBox( &rng );
		int k = 1 + (int)( lpRandom_Next( &rng ) % 23u );
		if ( lpIPlane_MakeSnapped( n, k, a, &p, NULL ) )
		{
			ENSURE( lpIPlane_MakeSnapped( lpNeg( n ), k, a, &q, NULL ) );
			negated = lpIPlane_Negate( p );
			ENSURE( lpIPlane_Equal( &q, &negated ) && lpIPlane_IsValid( &p ) );
			planes[count++] = p;
		}
	}

	// The total order: antisymmetric, zero exactly on equality, and sorted runs are non-decreasing, so de-duplication
	// by neighbours finds every repeat
	for ( int i = 0; i < count; ++i )
	{
		for ( int j = 0; j < count; j += 7 )
		{
			int c = lpIPlane_Compare( planes + i, planes + j );
			ENSURE( c == -lpIPlane_Compare( planes + j, planes + i ) );
			ENSURE( ( c == 0 ) == lpIPlane_Equal( planes + i, planes + j ) );
		}
	}
	qsort( planes, (size_t)count, sizeof( lpIPlane ), CompareForSort );
	int unique = 1;
	for ( int i = 1; i < count; ++i )
	{
		ENSURE( lpIPlane_Compare( planes + i - 1, planes + i ) <= 0 );
		unique += lpIPlane_Equal( planes + i - 1, planes + i ) ? 0 : 1;
	}
	int brute = 0;
	for ( int i = 0; i < count; ++i )
	{
		bool seen = false;
		for ( int j = 0; j < i && seen == false; ++j )
		{
			seen = lpIPlane_Equal( planes + i, planes + j );
		}
		brute += seen ? 0 : 1;
	}
	ENSURE( unique == brute && unique < count );
	free( planes );

	// What is not canonical or not in range is not valid
	ENSURE( lpIPlane_IsValid( &(lpIPlane){ { 2, 0, 0 }, 2 } ) == false );
	ENSURE( lpIPlane_IsValid( &(lpIPlane){ { 0, 0, 0 }, 1 } ) == false );
	ENSURE( lpIPlane_IsValid( &(lpIPlane){ { A_MAX, 1, 0 }, 0 } ) );
	ENSURE( lpIPlane_IsValid( &(lpIPlane){ { A_MAX + 1, 1, 0 }, 0 } ) == false );
	ENSURE( lpIPlane_IsValid( &(lpIPlane){ { 1, 2, 0 }, 3ll * P_MAX } ) );
	ENSURE( lpIPlane_IsValid( &(lpIPlane){ { 1, 2, 0 }, 3ll * P_MAX + 1 } ) == false );
	return 0;
}

// The bit budget (geom.h) at its bounds: every int128 vertex and classification equals a 256-bit reference, the
// bounds are reached exactly and never passed; and each constructor rejects one step beyond each of its ranges
static int TestPredicateBudget( void )
{
	// Every corner of the box: rows (n_i, d_i) = (+-A, +-A, +-A, +-D), all 2^16 sign patterns. The maxima of the
	// determinants live at corners, so this sees W = 4 A^3, |X| = 12 A^3 P and |S| = 48 A^4 P exactly.
	Wide largest[3] = { WideFrom64( 0 ), WideFrom64( 0 ), WideFrom64( 0 ) }, mag[3];
	for ( int pattern = 0; pattern < ( 1 << 16 ); ++pattern )
	{
		lpIPlane planes[4];
		for ( int i = 0; i < 4; ++i )
		{
			for ( int j = 0; j < 3; ++j )
			{
				planes[i].n[j] = ( pattern >> ( 4 * i + j ) & 1 ) ? -A_MAX : A_MAX;
			}
			planes[i].d = ( pattern >> ( 4 * i + 3 ) & 1 ) ? -D_MAX : D_MAX;
		}
		ENSURE( CheckAgainstWide( planes, planes + 1, planes + 2, planes + 3, mag ) == 0 );
		KeepLargest( largest, mag );
	}
	Wide a = WideFrom64( A_MAX ), p = WideFrom64( P_MAX );
	Wide a3 = WideMul( WideMul( a, a ), a );
	ENSURE( WideEqual( largest[0], WideMul( WideFrom64( 4 ), a3 ) ) );
	ENSURE( WideEqual( largest[1], WideMul( WideFrom64( 12 ), WideMul( a3, p ) ) ) );
	ENSURE( WideEqual( largest[2], WideMul( WideFrom64( 48 ), WideMul( WideMul( a3, a ), p ) ) ) );
	// W < 2^74, numerators < 2^99, the classification < 2^125: the stated bounds, each reached to its top bit
	ENSURE( WideBits( largest[0] ) == 74 && WideBits( largest[1] ) == 99 && WideBits( largest[2] ) == 125 );

	// The Hadamard pattern reaches every step's bound at once: n4.X = -36 A^4 P, d4 W = 12 A^4 P, S = -48 A^4 P
	{
		int32_t m = A_MAX;
		int64_t d = D_MAX;
		lpIPlane h[4] = { { { m, m, m }, d }, { { m, -m, m }, -d }, { { m, m, -m }, -d }, { { m, -m, -m }, d } };
		lpIVertex v;
		ENSURE( lpIVertex_FromPlanes( h, h + 1, h + 2, &v ) );
		lpI128 partial = lpI128_Add( lpI128_Add( lpI128_Mul( v.x, m ), lpI128_Mul( v.y, -m ) ), lpI128_Mul( v.z, -m ) );
		Wide a4p = WideMul( WideMul( a3, a ), p );
		ENSURE( WideEqual( WideFrom128( partial ), WideNeg( WideMul( WideFrom64( 36 ), a4p ) ) ) );
		ENSURE( WideEqual( WideFrom128( lpI128_Mul( v.w, d ) ), WideMul( WideFrom64( 12 ), a4p ) ) );
		ENSURE( WideEqual( WideFrom128( lpIVertex_Evaluate( &v, h + 3 ) ), WideNeg( WideMul( WideFrom64( 48 ), a4p ) ) ) );
	}

	// A grid point against a plane at the edge: n.p - d = 6 A P, in int64
	{
		lpIPlane plane = { { A_MAX, A_MAX, A_MAX }, -D_MAX };
		ENSURE( lpIPlane_ClassifyPoint( &plane, (int32_t[3]){ P_MAX, P_MAX, P_MAX } ) == 1 );
		lpIPlane flipped = lpIPlane_Negate( plane );
		ENSURE( lpIPlane_ClassifyPoint( &flipped, (int32_t[3]){ P_MAX, P_MAX, P_MAX } ) == -1 );
		ENSURE( lpIPlane_ClassifyPoint( &flipped, (int32_t[3]){ -P_MAX, -P_MAX, -P_MAX } ) == 0 );
	}

	// Nearly parallel triples at the largest offsets (vertices far outside the grid, numerators near their bound), and
	// random planes with components and offsets near their ends
	lpRandom rng;
	lpRandom_Seed( &rng, 2026, 11 );
	Wide random[3] = { WideFrom64( 0 ), WideFrom64( 0 ), WideFrom64( 0 ) };
	for ( int i = 0; i < 20000; ++i )
	{
		lpIPlane planes[4];
		if ( i % 4 == 0 )
		{
			planes[0] = RandomPlane( &rng, A_MAX );
			for ( int k = 1; k < 3; ++k )
			{
				planes[k] = planes[0];
				int axis = (int)( lpRandom_Next( &rng ) % 3u );
				planes[k].n[axis] += planes[k].n[axis] > 0 ? -( 1 + k ) : 1 + k;
				int64_t l1 = llabs( planes[k].n[0] ) + llabs( planes[k].n[1] ) + llabs( planes[k].n[2] );
				planes[k].d = ( lpRandom_Next( &rng ) & 1u ) ? l1 * P_MAX : -l1 * P_MAX;
			}
			planes[3] = RandomPlane( &rng, A_MAX );
		}
		else
		{
			for ( int k = 0; k < 4; ++k )
			{
				planes[k] = RandomPlane( &rng, i % 4 == 1 ? 1000 : A_MAX );
			}
		}
		ENSURE( CheckAgainstWide( planes, planes + 1, planes + 2, planes + 3, mag ) == 0 );
		KeepLargest( random, mag );
	}
	printf( "  random planes, largest seen: W %d bits, numerators %d bits, classification %d bits (of 127)\n",
			WideBits( random[0] ), WideBits( random[1] ), WideBits( random[2] ) );

	// Each constructor rejects one step beyond each of its ranges, and takes the step before
	lpIPlane plane;
	lpGeomReject why = lp_geomOk;
	// a. sites in range; the reduced normal within A (even and odd offsets); equal sites
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ P_MAX, 0, 0 }, (int32_t[3]){ 0, 0, 0 }, &plane, &why ) );
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ P_MAX + 1, 0, 0 }, (int32_t[3]){ 0, 0, 0 }, &plane, &why ) == false &&
			why == lp_geomOffGrid );
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ -P_MAX, 0, 0 }, (int32_t[3]){ P_MAX - 1, 1, 0 }, &plane, &why ) &&
			plane.n[0] == A_MAX );
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ -P_MAX, 0, 0 }, (int32_t[3]){ P_MAX, 1, 0 }, &plane, &why ) == false &&
			why == lp_geomNormalRange );
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ 0, 0, 0 }, (int32_t[3]){ P_MAX - 1, 2, 0 }, &plane, &why ) &&
			plane.n[0] == A_MAX - 1 );
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ 0, 0, 0 }, (int32_t[3]){ P_MAX, 1, 0 }, &plane, &why ) == false &&
			why == lp_geomNormalRange );
	ENSURE( lpIPlane_MakeBisector( (int32_t[3]){ 5, 5, 5 }, (int32_t[3]){ 5, 5, 5 }, &plane, &why ) == false &&
			why == lp_geomDegenerate );

	// b. the metric's stretch (up to 22.6), extent and axis; sites on the lattice; sites a step past the span
	lpIMetric metric;
	ENSURE( lpIMetric_Make( (lpVec3){ 1.0f, 1.0f, 1.0f }, 22.62f, 1000, &metric, &why ) &&
			(int64_t)metric.t * 3 * LP_GRAIN_Q == 255ll * metric.s ); // c = 255/256, the largest
	ENSURE( lpIMetric_Make( (lpVec3){ 1.0f, 1.0f, 1.0f }, 22.63f, 1000, &metric, &why ) == false &&
			why == lp_geomParamRange );
	ENSURE( lpIMetric_Make( (lpVec3){ 1.0f, 1.0f, 1.0f }, 0.99f, 1000, &metric, &why ) == false &&
			why == lp_geomParamRange );
	ENSURE( lpIMetric_Make( (lpVec3){ 1.0f, 0.0f, 0.0f }, 3.5f, 2 * P_MAX, &metric, &why ) );
	ENSURE( lpIMetric_Make( (lpVec3){ 1.0f, 0.0f, 0.0f }, 3.5f, 2 * P_MAX + 1, &metric, &why ) == false &&
			why == lp_geomParamRange );
	ENSURE( lpIMetric_Make( (lpVec3){ 0.0f, 0.0f, 0.0f }, 3.5f, 1000, &metric, &why ) == false &&
			why == lp_geomDegenerate );
	ENSURE( lpIMetric_Make( (lpVec3){ 1.0f, 1.0f, 1.0f }, 3.5f, 8363, &metric, &why ) );
	ENSURE( metric.s == 768 && metric.t == 235 && metric.span == 8363 && metric.lattice == 1 );
	{
		// At the span: the row with the largest sum, 2 |v_0| = 2 (768 R + 235 (R - 1)) <= A; a step past it, beyond
		ENSURE( lpIPlane_MakeMetricBisector( &metric, (int32_t[3]){ -4000000, 0, 0 }, (int32_t[3]){ -3991637, -8363, -8362 },
											 &plane, &why ) );
		ENSURE( lpIPlane_MakeMetricBisector( &metric, (int32_t[3]){ -4000000, 0, 0 }, (int32_t[3]){ -3991636, -8364, -8363 },
											 &plane, &why ) == false &&
				why == lp_geomNormalRange );
		ENSURE( lpIPlane_MakeMetricBisector( &metric, (int32_t[3]){ P_MAX + 1, 0, 0 }, (int32_t[3]){ 0, 0, 0 }, &plane,
											 &why ) == false &&
				why == lp_geomOffGrid );
		lpIMetric coarse;
		ENSURE( lpIMetric_Make( (lpVec3){ 1.0f, 1.0f, 1.0f }, 3.5f, 8364, &coarse, &why ) && coarse.lattice == 2 );
		ENSURE( lpIPlane_MakeMetricBisector( &coarse, (int32_t[3]){ 0, 0, 0 }, (int32_t[3]){ 2, 4, -6 }, &plane, &why ) );
		ENSURE( lpIPlane_MakeMetricBisector( &coarse, (int32_t[3]){ 0, 0, 0 }, (int32_t[3]){ 2, 4, -5 }, &plane, &why ) ==
					false &&
				why == lp_geomOffLattice );
		// Every pair within the span gets a plane: the sign pattern of the largest row, and random steps
		lpRandom_Seed( &rng, 99, 2 );
		for ( int i = 0; i < 20000; ++i )
		{
			int32_t s0[3], s1[3];
			for ( int k = 0; k < 3; ++k )
			{
				s0[k] = (int32_t)RandomIn( &rng, P_MAX - metric.span );
				s1[k] = s0[k] + (int32_t)RandomIn( &rng, metric.span );
			}
			ENSURE( lpIPlane_MakeMetricBisector( &metric, s0, s1, &plane, &why ) || why == lp_geomDegenerate );
		}
	}

	// c. k up to 23 (a component of 2^23 is within A); p in range; a direction
	ENSURE( lpIPlane_MakeSnapped( (lpVec3){ 1.0f, 0.3f, 0.1f }, 23, (int32_t[3]){ 0, 0, 0 }, &plane, &why ) &&
			plane.n[0] == ( 1 << 23 ) );
	ENSURE( lpIPlane_MakeSnapped( (lpVec3){ 1.0f, 0.3f, 0.1f }, 24, (int32_t[3]){ 0, 0, 0 }, &plane, &why ) == false &&
			why == lp_geomParamRange );
	ENSURE( lpIPlane_MakeSnapped( (lpVec3){ 1.0f, 0.3f, 0.1f }, 5, (int32_t[3]){ 0, -P_MAX, 0 }, &plane, &why ) );
	ENSURE( lpIPlane_MakeSnapped( (lpVec3){ 1.0f, 0.3f, 0.1f }, 5, (int32_t[3]){ 0, -P_MAX - 1, 0 }, &plane, &why ) ==
				false &&
			why == lp_geomOffGrid );
	ENSURE( lpIPlane_MakeSnapped( (lpVec3){ 0.0f, 0.0f, 0.0f }, 5, (int32_t[3]){ 0, 0, 0 }, &plane, &why ) == false &&
			why == lp_geomDegenerate );

	// d. |n_i| = 2 2^23 - 1 = A taken, 2^24 refused; |g_i| up to A; s along g
	const float e = 1.0f / 8388608.0f; // 2^-23: snaps to 1 against 2^23
	ENSURE( lpIPlane_MakeAxial( (int32_t[3]){ 2, 1, 0 }, (int32_t[3]){ 0, 0, 0 }, (lpVec3){ e, 1.0f, e }, 23, &plane,
								&why ) &&
			plane.n[0] == 1 && plane.n[1] == -2 && plane.n[2] == A_MAX );
	ENSURE( lpIPlane_MakeAxial( (int32_t[3]){ 2, 1, 0 }, (int32_t[3]){ 0, 0, 0 }, (lpVec3){ 0.0f, 1.0f, e }, 23, &plane,
								&why ) == false &&
			why == lp_geomNormalRange );
	ENSURE( lpIPlane_MakeAxial( (int32_t[3]){ A_MAX, 0, 0 }, (int32_t[3]){ 0, 0, 0 }, (lpVec3){ 0.0f, 1.0f, 0.0f }, 3,
								&plane, &why ) );
	ENSURE( lpIPlane_MakeAxial( (int32_t[3]){ A_MAX + 1, 0, 0 }, (int32_t[3]){ 0, 0, 0 }, (lpVec3){ 0.0f, 1.0f, 0.0f },
								3, &plane, &why ) == false &&
			why == lp_geomParamRange );
	ENSURE( lpIPlane_MakeAxial( (int32_t[3]){ 1, 0, 0 }, (int32_t[3]){ 0, 0, 0 }, (lpVec3){ 2.0f, 0.0f, 0.0f }, 3,
								&plane, &why ) == false &&
			why == lp_geomDegenerate );
	ENSURE( lpIPlane_MakeAxial( (int32_t[3]){ 1, 0, 0 }, (int32_t[3]){ P_MAX + 1, 0, 0 }, (lpVec3){ 0.0f, 1.0f, 0.0f },
								3, &plane, &why ) == false &&
			why == lp_geomOffGrid );
	return 0;
}

static double AngleBetween( const double u[3], const double v[3] )
{
	double cross[3] = { u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0] };
	double dot = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
	return atan2( sqrt( cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2] ), dot );
}

// Each constructor's plane is the one it promises: sites on their sides, midpoints on it, the metric bisector against
// a brute-force rational check, axial planes through their whole axis, snapped planes through their point
static int TestConstructors( void )
{
	lpRandom rng;
	lpRandom_Seed( &rng, 31337, 7 );
	lpIPlane plane;

	// a. Bisectors: both sites strictly on their sides, the midpoint (when on the grid) on the plane
	for ( int i = 0; i < 20000; ++i )
	{
		int32_t a[3], b[3];
		int32_t range = i % 2 == 0 ? P_MAX : 1000;
		for ( int k = 0; k < 3; ++k )
		{
			a[k] = (int32_t)RandomIn( &rng, range );
			b[k] = ( i % 4 == 1 ) ? a[k] + 2 * (int32_t)RandomIn( &rng, 100 ) : (int32_t)RandomIn( &rng, range );
			b[k] = b[k] > P_MAX ? P_MAX : ( b[k] < -P_MAX ? -P_MAX : b[k] );
		}
		if ( lpIPlane_MakeBisector( a, b, &plane, NULL ) == false )
		{
			continue;
		}
		ENSURE( lpIPlane_IsValid( &plane ) );
		ENSURE( lpIPlane_ClassifyPoint( &plane, a ) == -1 && lpIPlane_ClassifyPoint( &plane, b ) == 1 );
		if ( ( a[0] + b[0] ) % 2 == 0 && ( a[1] + b[1] ) % 2 == 0 && ( a[2] + b[2] ) % 2 == 0 )
		{
			int32_t mid[3] = { ( a[0] + b[0] ) / 2, ( a[1] + b[1] ) / 2, ( a[2] + b[2] ) / 2 };
			ENSURE( lpIPlane_ClassifyPoint( &plane, mid ) == 0 );
		}
	}

	// b. Metric bisectors against brute force: f(x) = (x - a)^T K (x - a) - (x - b)^T K (x - b), with K = s I - t g g^T,
	// is negative exactly where x is nearer a under the metric; the plane's side must be the sign of f, everywhere
	int checked = 0;
	for ( int i = 0; i < 300; ++i )
	{
		lpVec3 axis = RandomBox( &rng );
		if ( i % 3 == 0 )
		{
			axis = (lpVec3){ 0.0f, 0.0f, 1.0f }; // the common case: grain along a box axis
		}
		float stretch = lpRandom_Range( &rng, 1.0f, 8.0f );
		int32_t extent = 1 << ( 8 + (int)( lpRandom_Next( &rng ) % 16u ) );
		lpIMetric metric;
		ENSURE( lpIMetric_Make( axis, stretch, extent, &metric, NULL ) );
		ENSURE( (int64_t)metric.span * metric.lattice >= extent );
		const int32_t* g = metric.g;
		int32_t norm2 = g[0] * g[0] + g[1] * g[1] + g[2] * g[2];
		ENSURE( metric.s > metric.t * norm2 );
		if ( lpLengthSquared( axis ) > 0.0f )
		{
			// The axis is within 5.02 degrees of the float one
			double u[3] = { axis.x, axis.y, axis.z }, v[3] = { g[0], g[1], g[2] };
			double angle = AngleBetween( u, v );
			angle = angle > 1.5707963 ? 3.14159265358979 - angle : angle;
			ENSURE( angle < 5.02 * 3.14159265358979 / 180.0 );
		}
		int32_t origin[3]; // the sites stay within origin + [0, extent], inside the grid
		for ( int k = 0; k < 3; ++k )
		{
			origin[k] = (int32_t)RandomIn( &rng, ( P_MAX - extent ) / 2 );
		}
		for ( int j = 0; j < 8; ++j )
		{
			// Two sites of the job: on its lattice, within its extent
			int32_t a[3], b[3];
			int32_t steps = extent / metric.lattice;
			for ( int k = 0; k < 3; ++k )
			{
				a[k] = origin[k] + metric.lattice * (int32_t)( lpRandom_Next( &rng ) % (uint32_t)( steps + 1 ) );
				b[k] = origin[k] + metric.lattice * (int32_t)( lpRandom_Next( &rng ) % (uint32_t)( steps + 1 ) );
			}
			lpIPlane back;
			if ( lpIPlane_MakeMetricBisector( &metric, a, b, &plane, NULL ) == false )
			{
				ENSURE( a[0] == b[0] && a[1] == b[1] && a[2] == b[2] ); // the only refusal within the extent
				continue;
			}
			ENSURE( lpIPlane_IsValid( &plane ) );
			ENSURE( lpIPlane_MakeMetricBisector( &metric, b, a, &back, NULL ) );
			lpIPlane negated = lpIPlane_Negate( plane );
			ENSURE( lpIPlane_Equal( &back, &negated ) ); // neighbours share the plane exactly
			for ( int q = 0; q < 16; ++q )
			{
				int32_t x[3];
				for ( int k = 0; k < 3; ++k )
				{
					// Near the sites, and exactly at them and their midpoint
					int64_t mid = ( (int64_t)a[k] + b[k] ) / 2;
					int64_t spread = llabs( (int64_t)b[k] - a[k] ) + 4;
					x[k] = q == 0 ? a[k] : ( q == 1 ? b[k] : (int32_t)( mid + RandomIn( &rng, spread ) ) );
				}
				Wide f = WideFrom64( 0 );
				Wide ga = WideFrom64( 0 ), gb = WideFrom64( 0 );
				Wide xa[3], xb[3];
				for ( int k = 0; k < 3; ++k )
				{
					xa[k] = WideFrom64( (int64_t)x[k] - a[k] );
					xb[k] = WideFrom64( (int64_t)x[k] - b[k] );
					ga = WideAdd( ga, WideMul( WideFrom64( g[k] ), xa[k] ) );
					gb = WideAdd( gb, WideMul( WideFrom64( g[k] ), xb[k] ) );
					f = WideAdd( f, WideMul( WideFrom64( metric.s ),
											 WideSub( WideMul( xa[k], xa[k] ), WideMul( xb[k], xb[k] ) ) ) );
				}
				f = WideSub( f, WideMul( WideFrom64( metric.t ), WideSub( WideMul( ga, ga ), WideMul( gb, gb ) ) ) );
				ENSURE( lpIPlane_ClassifyPoint( &plane, x ) == WideSign( f ) );
				checked += 1;
			}
		}
	}
	printf( "  metric bisectors: %d points against brute force\n", checked );

	// c. Snapped planes contain their point; the normal is within asin(2^-k / sqrt 2) of the float one
	for ( int i = 0; i < 20000; ++i )
	{
		lpVec3 n = RandomBox( &rng );
		int k = (int)( lpRandom_Next( &rng ) % 24u );
		int32_t p[3];
		for ( int j = 0; j < 3; ++j )
		{
			p[j] = (int32_t)RandomIn( &rng, P_MAX );
		}
		if ( lpIPlane_MakeSnapped( n, k, p, &plane, NULL ) == false )
		{
			continue;
		}
		ENSURE( lpIPlane_IsValid( &plane ) && lpIPlane_ClassifyPoint( &plane, p ) == 0 );
		int32_t s[3];
		ENSURE( lpGeom_SnapDirection( n, k, s ) );
		ENSURE( abs( s[0] ) == ( 1 << k ) || abs( s[1] ) == ( 1 << k ) || abs( s[2] ) == ( 1 << k ) );
		double u[3] = { n.x, n.y, n.z }, v[3] = { plane.n[0], plane.n[1], plane.n[2] };
		ENSURE( AngleBetween( u, v ) <= asin( 0.70710678 / (double)( 1 << k ) ) * 1.000001 + 1e-12 );
	}

	// d. Axial planes: n.g = 0, and every grid point on the axis through p lies on every plane about it, exactly
	for ( int i = 0; i < 4000; ++i )
	{
		int32_t g[3];
		int32_t gRange = i % 2 == 0 ? 8 : 4096;
		do
		{
			for ( int k = 0; k < 3; ++k )
			{
				g[k] = (int32_t)RandomIn( &rng, gRange );
			}
		}
		while ( g[0] == 0 && g[1] == 0 && g[2] == 0 );
		int32_t p[3];
		for ( int k = 0; k < 3; ++k )
		{
			p[k] = (int32_t)RandomIn( &rng, P_MAX / 2 );
		}
		lpIPlane wedges[6];
		int wedgeCount = 0;
		for ( int w = 0; w < 6; ++w )
		{
			lpVec3 direction = RandomBox( &rng );
			int k = gRange == 8 ? 16 : 8;
			if ( lpIPlane_MakeAxial( g, p, direction, k, wedges + wedgeCount, NULL ) == false )
			{
				continue;
			}
			const lpIPlane* w0 = wedges + wedgeCount;
			ENSURE( lpIPlane_IsValid( w0 ) );
			ENSURE( (int64_t)w0->n[0] * g[0] + (int64_t)w0->n[1] * g[1] + (int64_t)w0->n[2] * g[2] == 0 );
			// The snapped direction lies in the plane too
			int32_t s[3];
			ENSURE( lpGeom_SnapDirection( direction, k, s ) );
			int32_t ps[3] = { p[0] + s[0], p[1] + s[1], p[2] + s[2] };
			if ( lpGrid_Contains( ps ) )
			{
				ENSURE( lpIPlane_ClassifyPoint( w0, ps ) == 0 );
			}
			wedgeCount += 1;
		}
		for ( int t = -40; t <= 40; ++t )
		{
			int64_t step = (int64_t)t * ( t % 2 == 0 ? 1 : 997 );
			int32_t on[3];
			bool inRange = true;
			for ( int k = 0; k < 3; ++k )
			{
				int64_t c = p[k] + step * g[k];
				inRange = inRange && c >= -P_MAX && c <= P_MAX;
				on[k] = (int32_t)( inRange ? c : 0 );
			}
			for ( int w = 0; w < wedgeCount && inRange; ++w )
			{
				ENSURE( lpIPlane_ClassifyPoint( wedges + w, on ) == 0 );
			}
		}
		// The planes meet the axis as vertices too: any two wedges and a plane across the axis meet on it
		if ( wedgeCount >= 2 )
		{
			lpIPlane across;
			lpVec3 gf = { (float)g[0], (float)g[1], (float)g[2] };
			if ( lpIPlane_MakeSnapped( gf, 12, p, &across, NULL ) )
			{
				lpIVertex v;
				if ( lpIVertex_FromPlanes( wedges, wedges + 1, &across, &v ) )
				{
					for ( int w = 0; w < wedgeCount; ++w )
					{
						ENSURE( lpIVertex_Classify( &v, wedges + w ) == 0 );
					}
				}
			}
		}
	}

	// The axis snap: aligned axes snap to themselves, signs to a positive first component
	int32_t g[3];
	ENSURE( lpGeom_SnapAxis( (lpVec3){ 0.0f, 0.0f, -5.0f }, g ) && g[0] == 0 && g[1] == 0 && g[2] == 1 );
	ENSURE( lpGeom_SnapAxis( (lpVec3){ -2.0f, 2.0f, 0.0f }, g ) && g[0] == 1 && g[1] == -1 && g[2] == 0 );
	ENSURE( lpGeom_SnapAxis( (lpVec3){ 3.0f, 1.0f, -2.0f }, g ) && g[0] == 3 && g[1] == 1 && g[2] == -2 );
	ENSURE( lpGeom_SnapAxis( (lpVec3){ 0.0f, 0.0f, 0.0f }, g ) == false );

	// geom.h's numbers for wood (3.5): along a box axis a span of 32767 steps, so a 4 m piece gets a lattice of 16 u;
	// along the worst axis, (8, 8, 7), 139 steps: 512 u for a 1 m piece, 2048 u for a 4 m one
	lpIMetric metric;
	ENSURE( lpIMetric_Make( (lpVec3){ 0.0f, 0.0f, 1.0f }, 3.5f, 4 << 16, &metric, NULL ) );
	ENSURE( metric.s == 256 && metric.t == 235 && metric.span == 32767 && metric.lattice == 16 );
	ENSURE( lpIMetric_Make( (lpVec3){ 8.0f, 8.0f, 7.0f }, 3.5f, 1 << 16, &metric, NULL ) );
	ENSURE( metric.g[0] == 8 && metric.g[1] == 8 && metric.g[2] == 7 && metric.span == 139 && metric.lattice == 512 );
	ENSURE( lpIMetric_Make( (lpVec3){ 8.0f, 8.0f, 7.0f }, 3.5f, 4 << 16, &metric, NULL ) && metric.lattice == 2048 );
	return 0;
}

// The classification's cost on this build's path and on the portable reference (for milestone 11a's step C2), with the
// vertex construction and the int64 grid-point case beside it. Printed only.
static int ClassifyPortable( const lpIVertex* v, const lpIPlane* p )
{
	lpI128 s = lpI128_AddPortable( lpI128_MulPortable( v->x, p->n[0] ), lpI128_MulPortable( v->y, p->n[1] ) );
	s = lpI128_AddPortable( s, lpI128_MulPortable( v->z, p->n[2] ) );
	return lpI128_SignPortable( lpI128_SubPortable( s, lpI128_MulPortable( v->w, p->d ) ) );
}

static int TestPredicateCost( void )
{
	enum
	{
		lp_vertices = 256,
		lp_planes = 256,
		lp_rounds = 8
	};
	lpRandom rng;
	lpRandom_Seed( &rng, 4242, 1 );
	// Voronoi-like planes: bisectors of sites within 16 m
	lpIPlane* planes = malloc( lp_planes * sizeof( lpIPlane ) );
	for ( int i = 0; i < lp_planes; )
	{
		int32_t a[3], b[3];
		for ( int k = 0; k < 3; ++k )
		{
			a[k] = (int32_t)RandomIn( &rng, 1 << 20 );
			b[k] = (int32_t)RandomIn( &rng, 1 << 20 );
		}
		i += lpIPlane_MakeBisector( a, b, planes + i, NULL ) ? 1 : 0;
	}
	lpIVertex* vertices = malloc( lp_vertices * sizeof( lpIVertex ) );
	int32_t( *points )[3] = malloc( lp_vertices * sizeof( *points ) );
	uint64_t ticks = lpGetTicks();
	int made = 0;
	for ( int round = 0; round < 64; ++round )
	{
		for ( int i = 0; i < lp_vertices; ++i )
		{
			made += lpIVertex_FromPlanes( planes + i, planes + ( i * 7 + round + 1 ) % lp_planes,
										  planes + ( i * 13 + 2 * round + 5 ) % lp_planes, vertices + i )
						? 1
						: 0;
		}
	}
	float vertexMs = lpGetMilliseconds( ticks );
	for ( int i = 0; i < lp_vertices; ++i )
	{
		int offset = 1;
		while ( lpIVertex_FromPlanes( planes + i, planes + ( i + offset ) % lp_planes, planes + ( i + 2 * offset ) % lp_planes,
									  vertices + i ) == false )
		{
			offset += 1;
		}
		for ( int k = 0; k < 3; ++k )
		{
			points[i][k] = (int32_t)RandomIn( &rng, 1 << 20 );
		}
	}

	int sumNative = 0, sumPortable = 0, sumPoints = 0;
	ticks = lpGetTicks();
	for ( int round = 0; round < lp_rounds; ++round )
	{
		for ( int i = 0; i < lp_vertices; ++i )
		{
			for ( int j = 0; j < lp_planes; ++j )
			{
				sumNative += lpIVertex_Classify( vertices + i, planes + j );
			}
		}
	}
	float nativeMs = lpGetMilliseconds( ticks );
	ticks = lpGetTicks();
	for ( int round = 0; round < lp_rounds; ++round )
	{
		for ( int i = 0; i < lp_vertices; ++i )
		{
			for ( int j = 0; j < lp_planes; ++j )
			{
				sumPortable += ClassifyPortable( vertices + i, planes + j );
			}
		}
	}
	float portableMs = lpGetMilliseconds( ticks );
	ticks = lpGetTicks();
	for ( int round = 0; round < lp_rounds; ++round )
	{
		for ( int i = 0; i < lp_vertices; ++i )
		{
			for ( int j = 0; j < lp_planes; ++j )
			{
				sumPoints += lpIPlane_ClassifyPoint( planes + j, points[i] );
			}
		}
	}
	float pointMs = lpGetMilliseconds( ticks );
	float n = (float)lp_rounds * lp_vertices * lp_planes;
	printf( "  classification: %.2f ns (%s), %.2f ns (portable); grid point (int64) %.2f ns; vertex from planes %.1f ns"
			" (%d sums %d %d %d)\n",
			1e6f * nativeMs / n, lpI128_Path(), 1e6f * portableMs / n, 1e6f * pointMs / n,
			1e6f * vertexMs / ( 64.0f * lp_vertices ), made, sumNative, sumPortable, sumPoints );
	free( points );
	free( vertices );
	free( planes );
	ENSURE( sumNative == sumPortable );
	return 0;
}

// ---- canonical rounding (C2) ----

// The reference's sign of |x| / (w 2^16) - m for a positive double m, from the 256-bit reference alone
static int RefCompareMetres( Wide ax, Wide w, double m )
{
	uint64_t k;
	int t;
	DoubleParts( m, &k, &t );
	int s = t + 16;
	Wide wk = WideMul( w, WideFrom64( (int64_t)k ) );
	return s >= 0 ? WideCompare( ax, WideShl( wk, s ) ) : WideCompare( WideShl( ax, -s ), wk );
}

static float FloatOfBits( uint32_t bits )
{
	float f;
	memcpy( &f, &bits, sizeof( f ) );
	return f;
}

// lpGeom_RoundToFloat against the reference: x / (w 2^16) lies between the result's midpoints with its neighbours, a
// midpoint only with an even result; the sign is x's; the negation rounds to the negation. Returns the result's bits.
static int CheckFloatRounding( lpI128 x, lpI128 w, uint32_t* out )
{
	float f = lpGeom_RoundToFloat( x, w );
	uint32_t bits;
	memcpy( &bits, &f, sizeof( bits ) );
	*out = bits;
	ENSURE( lpGeom_RoundToFloat( lpI128_Neg( x ), w ) == -f );
	if ( lpI128_IsZero( x ) )
	{
		ENSURE( bits == 0 );
		return 0;
	}
	ENSURE( ( f < 0.0f ) == ( lpI128_Sign( x ) < 0 ) );
	float af = f < 0.0f ? -f : f;
	uint32_t ab = bits & 0x7FFFFFFFu;
	ENSURE( ab >= 0x00800000u && ab < 0x7F800000u ); // normal, finite
	double lower = 0.5 * ( (double)FloatOfBits( ab - 1u ) + (double)af );
	double upper = 0.5 * ( (double)af + (double)FloatOfBits( ab + 1u ) );
	Wide ax = WideAbs( WideFrom128( x ) ), ww = WideFrom128( w );
	int below = RefCompareMetres( ax, ww, lower );
	int above = RefCompareMetres( ax, ww, upper );
	ENSURE( below >= 0 && above <= 0 );
	if ( below == 0 || above == 0 )
	{
		ENSURE( ( ab & 1u ) == 0 ); // a tie: the even significand
	}
	return 0;
}

// lpGeom_RoundToGrid against the reference: |2x - 2qw| <= w, and at a half |q| beyond |x / w| (away from zero)
static int CheckGridRounding( lpI128 x, lpI128 w, int64_t* out )
{
	int64_t q = lpGeom_RoundToGrid( x, w );
	*out = q;
	ENSURE( lpGeom_RoundToGrid( lpI128_Neg( x ), w ) == -q );
	Wide ww = WideFrom128( w );
	Wide twoX = WideAdd( WideFrom128( x ), WideFrom128( x ) );
	Wide twoQW = WideMul( WideFrom64( 2 * q ), ww );
	int c = WideCompare( WideAbs( WideSub( twoX, twoQW ) ), ww );
	ENSURE( c <= 0 );
	if ( c == 0 )
	{
		ENSURE( WideCompare( WideAbs( twoQW ), WideAbs( twoX ) ) > 0 );
	}
	return 0;
}

// True when |x / w| < 2^50 (lpGeom_RoundToGrid's domain)
static bool InGridDomain( lpI128 x, lpI128 w )
{
	return WideCompare( WideAbs( WideFrom128( x ) ), WideShl( WideFrom128( w ), 50 ) ) < 0;
}

static lpI128 RandomPositive128( lpRandom* rng, int maxBits )
{
	int bits = 1 + (int)( lpRandom_Next( rng ) % (uint32_t)maxBits );
	lpI128 r;
	r.lo = Next64( rng );
	r.hi = Next64( rng );
	if ( bits <= 64 )
	{
		r.hi = 0;
		r.lo &= bits == 64 ? ~0ull : ( 1ull << bits ) - 1u;
		r.lo |= 1ull << ( bits - 1 );
	}
	else
	{
		r.hi &= ( 1ull << ( bits - 64 ) ) - 1u;
		r.hi |= 1ull << ( bits - 65 );
	}
	return r;
}

// Rounding to the grid (half away from zero) and to floats (to nearest, ties to even) against an exact reference: random
// rationals of every size the budget allows, vertices of random planes, constructed ties for both, both signs, and
// the budget's largest and smallest values
static int TestCanonicalRounding( void )
{
	lpRandom rng;
	lpRandom_Seed( &rng, 0xC2C2ull, 9 );
	uint32_t bits;
	int64_t q;
	int gridTies = 0, floatTies = 0, checked = 0;

	// Random rationals: |x| up to 2^99, w up to 2^74
	for ( int i = 0; i < 200000; ++i )
	{
		lpI128 w = RandomPositive128( &rng, 74 );
		lpI128 x = RandomPositive128( &rng, 99 );
		x = ( lpRandom_Next( &rng ) & 1u ) ? lpI128_Neg( x ) : x;
		ENSURE( CheckFloatRounding( x, w, &bits ) == 0 );
		if ( InGridDomain( x, w ) )
		{
			ENSURE( CheckGridRounding( x, w, &q ) == 0 );
		}
		checked += 1;
	}

	// Vertices of random planes: the points the clip makes
	for ( int i = 0; i < 40000; ++i )
	{
		lpIPlane planes[3];
		for ( int k = 0; k < 3; ++k )
		{
			planes[k] = RandomPlane( &rng, i % 2 == 0 ? A_MAX : 64 );
		}
		lpIVertex v;
		if ( lpIVertex_FromPlanes( planes, planes + 1, planes + 2, &v ) == false )
		{
			continue;
		}
		const lpI128* c[3] = { &v.x, &v.y, &v.z };
		for ( int k = 0; k < 3; ++k )
		{
			ENSURE( CheckFloatRounding( *c[k], v.w, &bits ) == 0 );
			if ( InGridDomain( *c[k], v.w ) )
			{
				ENSURE( CheckGridRounding( *c[k], v.w, &q ) == 0 );
			}
		}
		lpVec3 f = lpIVertex_RoundToFloat( &v );
		ENSURE( f.x == lpGeom_RoundToFloat( v.x, v.w ) && f.z == lpGeom_RoundToFloat( v.z, v.w ) );
	}

	// Grid ties: x / w = q + 1/2 exactly (x = (2q + 1) m, w = 2m), and one off either side
	for ( int i = 0; i < 20000; ++i )
	{
		int64_t m = (int64_t)( Next64( &rng ) >> 1 ); // one draw per statement
		m = ( m >> ( lpRandom_Next( &rng ) % 63u ) ) | 1;
		int64_t half = (int64_t)( Next64( &rng ) >> 15 );
		half >>= lpRandom_Next( &rng ) % 49u;
		half = ( lpRandom_Next( &rng ) & 1u ) ? -half : half;
		lpI128 w = lpI128_Mul64( m, 2 );
		lpI128 x = lpI128_Mul64( 2 * half + 1, m );
		ENSURE( CheckGridRounding( x, w, &q ) == 0 );
		ENSURE( q == ( half >= 0 ? half + 1 : half ) ); // away from zero
		gridTies += 1;
		ENSURE( CheckGridRounding( lpI128_Add( x, lpI128_FromI64( 1 ) ), w, &q ) == 0 );
		ENSURE( CheckGridRounding( lpI128_Sub( x, lpI128_FromI64( 1 ) ), w, &q ) == 0 );
		ENSURE( CheckFloatRounding( x, w, &bits ) == 0 );
	}

	// Float ties: x / (w 2^16) exactly the midpoint of a float and the next, as K 2^t (K odd), with w = c or c 2^-s
	for ( int i = 0; i < 20000; ++i )
	{
		int exponent = 127 - 90 + (int)( lpRandom_Next( &rng ) % 172u ); // floats from 2^-90 to 2^82 m
		uint32_t fb = (uint32_t)exponent << 23 | ( lpRandom_Next( &rng ) & 0x7FFFFFu );
		float f = FloatOfBits( fb );
		double mid = 0.5 * ( (double)f + (double)FloatOfBits( fb + 1u ) );
		uint64_t k;
		int t;
		DoubleParts( mid, &k, &t );
		while ( ( k & 1u ) == 0 )
		{
			k >>= 1;
			t += 1;
		}
		int s = t + 16; // x / w = K 2^s
		int64_t c = (int64_t)lpRandom_Next( &rng );
		c = ( c >> ( lpRandom_Next( &rng ) % 32u ) ) | 1;
		lpI128 x, w;
		if ( s >= 0 )
		{
			if ( s > 40 ) // |x| < 2^99, the budget's
			{
				continue;
			}
			w = lpI128_FromI64( c );
			x = lpI128_Shl( lpI128_Mul64( (int64_t)k, c ), s );
		}
		else
		{
			if ( -s > 40 )
			{
				continue;
			}
			w = lpI128_Shl( lpI128_FromI64( c ), -s );
			x = lpI128_Mul64( (int64_t)k, c );
		}
		x = ( i & 1 ) ? lpI128_Neg( x ) : x;
		ENSURE( CheckFloatRounding( x, w, &bits ) == 0 );
		uint32_t even = ( fb & 1u ) == 0 ? fb : fb + 1u;
		ENSURE( ( bits & 0x7FFFFFFFu ) == even );
		floatTies += 1;
		ENSURE( CheckFloatRounding( lpI128_Add( x, lpI128_FromI64( 1 ) ), w, &bits ) == 0 );
		ENSURE( CheckFloatRounding( lpI128_Sub( x, lpI128_FromI64( 1 ) ), w, &bits ) == 0 );
		if ( InGridDomain( x, w ) )
		{
			ENSURE( CheckGridRounding( x, w, &q ) == 0 );
		}
	}

	// The budget's ends: numerators of 12 A^3 P, w from 1 to 4 A^3, both signs; the grid's range and its domain's edge
	{
		Wide a3 = WideMul( WideMul( WideFrom64( A_MAX ), WideFrom64( A_MAX ) ), WideFrom64( A_MAX ) );
		lpI128 numerator = lpI128_Mul( lpI128_Mul64( 12ll * A_MAX, (int64_t)A_MAX * A_MAX ), P_MAX );
		lpI128 wMax = lpI128_Mul64( 4ll * A_MAX, (int64_t)A_MAX * A_MAX );
		ENSURE( WideEqual( WideFrom128( wMax ), WideMul( WideFrom64( 4 ), a3 ) ) );
		lpI128 ws[3] = { lpI128_FromI64( 1 ), lpI128_FromI64( 3 ), wMax };
		lpI128 xs[6] = { numerator, lpI128_FromI64( 1 ), lpI128_FromI64( 2 ), wMax, lpI128_Sub( numerator, lpI128_FromI64( 1 ) ),
						 lpI128_Mul( wMax, P_MAX ) };
		for ( int a = 0; a < 3; ++a )
		{
			for ( int b = 0; b < 6; ++b )
			{
				for ( int sign = 0; sign < 2; ++sign )
				{
					lpI128 x = sign ? lpI128_Neg( xs[b] ) : xs[b];
					ENSURE( CheckFloatRounding( x, ws[a], &bits ) == 0 );
					if ( InGridDomain( x, ws[a] ) )
					{
						ENSURE( CheckGridRounding( x, ws[a], &q ) == 0 );
					}
				}
			}
		}
		// Known answers: 1 / (4 A^3) u is the smallest |x| (2^-90 m and a little), 12 A^3 P u the largest (2^82.6 m)
		ENSURE( CheckFloatRounding( lpI128_FromI64( 1 ), wMax, &bits ) == 0 && ( bits >> 23 ) == 127 - 90 );
		ENSURE( CheckFloatRounding( numerator, lpI128_FromI64( 1 ), &bits ) == 0 && ( bits >> 23 ) == 127 + 82 );
		ENSURE( lpGeom_RoundToFloat( wMax, wMax ) == 1.0f / 65536.0f ); // one grid unit
		ENSURE( lpGeom_RoundToFloat( lpI128_Mul( wMax, -P_MAX ), wMax ) == -128.0f );
		ENSURE( CheckGridRounding( lpI128_Mul( wMax, P_MAX ), wMax, &q ) == 0 && q == P_MAX );
		// The domain's edge: |x / w| just under 2^50, at w = 4 A^3 (products near 2^126)
		lpI128 edge = lpI128_Sub( lpI128_Shl( wMax, 50 ), lpI128_FromI64( 1 ) );
		ENSURE( CheckGridRounding( edge, wMax, &q ) == 0 && q == ( 1ll << 50 ) );
		ENSURE( CheckGridRounding( lpI128_Neg( edge ), wMax, &q ) == 0 && q == -( 1ll << 50 ) );
		lpI128 below = lpI128_Sub( lpI128_Shl( wMax, 50 ), wMax ); // 2^50 - 1, and a little more
		ENSURE( CheckGridRounding( lpI128_Add( below, lpI128_FromI64( 2 * A_MAX ) ), wMax, &q ) == 0 && q == ( 1ll << 50 ) - 1 );

		// The budget's Hadamard vertex (geom.h): W = 4 A^3, numerators 12 A^3 P
		int32_t m = A_MAX;
		int64_t d = D_MAX;
		lpIPlane h[3] = { { { m, m, m }, d }, { { m, -m, m }, -d }, { { m, m, -m }, -d } };
		lpIVertex v;
		ENSURE( lpIVertex_FromPlanes( h, h + 1, h + 2, &v ) );
		const lpI128* c[3] = { &v.x, &v.y, &v.z };
		for ( int k = 0; k < 3; ++k )
		{
			ENSURE( CheckFloatRounding( *c[k], v.w, &bits ) == 0 && CheckGridRounding( *c[k], v.w, &q ) == 0 );
		}
	}

	// Grid points come back as themselves, and as the floats they are (exact below 2^24 u)
	for ( int i = 0; i < 20000; ++i )
	{
		int64_t p = RandomIn( &rng, P_MAX );
		lpI128 w = RandomPositive128( &rng, 74 );
		lpI128 x = lpI128_Mul( w, p );
		ENSURE( CheckGridRounding( x, w, &q ) == 0 && q == p );
		ENSURE( lpGeom_RoundToFloat( x, w ) == (float)p / 65536.0f );
	}
	printf( "  %d random rationals, %d grid ties, %d float ties against the 256-bit reference\n", checked, gridTies,
			floatTies );
	return 0;
}

// The rounding's cost per coordinate (fast path and exact path), printed only
static int TestRoundingCost( void )
{
	enum
	{
		lp_count = 4096
	};
	lpRandom rng;
	lpRandom_Seed( &rng, 5150, 3 );
	lpIVertex* vertices = malloc( lp_count * sizeof( lpIVertex ) );
	for ( int i = 0; i < lp_count; )
	{
		lpIPlane planes[3];
		for ( int k = 0; k < 3; ++k )
		{
			int32_t a[3], b[3];
			for ( int j = 0; j < 3; ++j )
			{
				a[j] = (int32_t)RandomIn( &rng, 1 << 20 );
				b[j] = (int32_t)RandomIn( &rng, 1 << 20 );
			}
			if ( lpIPlane_MakeBisector( a, b, planes + k, NULL ) == false )
			{
				planes[k] = planes[0];
			}
		}
		i += lpIVertex_FromPlanes( planes, planes + 1, planes + 2, vertices + i ) ? 1 : 0;
	}
	bool* inDomain = malloc( lp_count * sizeof( bool ) );
	for ( int i = 0; i < lp_count; ++i )
	{
		inDomain[i] = InGridDomain( vertices[i].x, vertices[i].w ) && InGridDomain( vertices[i].y, vertices[i].w ) &&
					  InGridDomain( vertices[i].z, vertices[i].w );
	}
	float sum = 0.0f;
	int64_t gridSum = 0;
	uint64_t ticks = lpGetTicks();
	for ( int round = 0; round < 16; ++round )
	{
		for ( int i = 0; i < lp_count; ++i )
		{
			lpVec3 f = lpIVertex_RoundToFloat( vertices + i );
			sum += f.x + f.y + f.z;
		}
	}
	float floatMs = lpGetMilliseconds( ticks );
	ticks = lpGetTicks();
	for ( int round = 0; round < 16; ++round )
	{
		for ( int i = 0; i < lp_count; ++i )
		{
			int64_t g[3];
			if ( inDomain[i] )
			{
				lpIVertex_RoundToGrid( vertices + i, g );
				gridSum += g[0] + g[1] + g[2];
			}
		}
	}
	float gridMs = lpGetMilliseconds( ticks );
	free( inDomain );
	free( vertices );
	float n = 16.0f * 3.0f * lp_count;
	printf( "  rounding a coordinate to float: %.1f ns; to the grid: %.1f ns (%s; sums %g %lld)\n",
			1e6f * floatMs / n, 1e6f * gridMs / n, lpI128_Path(), (double)sum, (long long)gridSum );
	return 0;
}

int XPolyTest( void );

int GeomTest( void )
{
	RUN_TEST( TestInt128, MECHANISM );
	RUN_TEST( TestPlaneCanonical, MECHANISM );
	RUN_TEST( TestPredicateBudget, MECHANISM );
	RUN_TEST( TestConstructors, MECHANISM );
	RUN_TEST( TestPredicateCost, TIMING );
	RUN_TEST( TestCanonicalRounding, MECHANISM );
	RUN_TEST( TestRoundingCost, TIMING );
	return XPolyTest();
}
