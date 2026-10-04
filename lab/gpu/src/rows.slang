// rows.slang: T10's row kernel (see rowlayout.h). One thread per row.
#include "rowlayout.h"

[[vk::binding( 0, 0 )]] StructuredBuffer<RRow> rows;
[[vk::binding( 1, 0 )]] RWStructuredBuffer<ROut> outs;
[[vk::binding( 2, 0 )]] RWStructuredBuffer<uint> counters; // [0] saturations, [1] shift clamps
[[vk::push_constant]] ConstantBuffer<RPush> rp;

void countAt( int k )
{
#if defined( TWIN_CPP )
	counters[k] = counters[k] + 1u;
#else
	InterlockedAdd( counters[k], 1u );
#endif
}

// ------------------------------------------------------------------------------------------------
// Arithmetic per format
// ------------------------------------------------------------------------------------------------

#if defined( VAR_F )

T M( T a, T b, int sh ) { return a * b; }
T sum3( T a, T b, T c, T d, T e, T f, int sh ) { return a * b + c * d + e * f; }
T dif2( T a, T b, T c, T d, int sh ) { return a * b - c * d; }
T zero() { return 0.0f; }
T alignDP( T dp ) { return dp; }
T adds( T a, T b ) { return a + b; }
T subs( T a, T b ) { return a - b; }

#elif defined( VAR_I64 )

static const uint64_t LOW32 = uint64_t( 0xFFFFFFFFu );
T qmul( T a, T b )
{
	uint64_t ua = uint64_t( a );
	uint64_t ub = uint64_t( b );
	uint al = uint( ua );
	uint ah = uint( ua >> 32 );
	uint bl = uint( ub );
	uint bh = uint( ub >> 32 );
	uint64_t ll = uint64_t( al ) * uint64_t( bl );
	uint64_t lh = uint64_t( al ) * uint64_t( bh );
	uint64_t hl = uint64_t( ah ) * uint64_t( bl );
	uint hh = ah * bh;
	uint64_t mid = ( ll >> 32 ) + ( lh & LOW32 ) + ( hl & LOW32 ) + ( ( ll >> 31 ) & uint64_t( 1 ) );
	uint hi = uint( lh >> 32 ) + uint( hl >> 32 ) + hh + uint( mid >> 32 );
	hi = a < 0 ? hi - bl : hi;
	hi = b < 0 ? hi - al : hi;
	return int64_t( ( uint64_t( hi ) << 32 ) | ( mid & LOW32 ) );
}
T M( T a, T b, int sh ) { return qmul( a, b ); }
T sum3( T a, T b, T c, T d, T e, T f, int sh ) { return qmul( a, b ) + qmul( c, d ) + qmul( e, f ); }
T dif2( T a, T b, T c, T d, int sh ) { return qmul( a, b ) - qmul( c, d ); }
T zero() { return 0; }
T alignDP( T dp ) { return dp; }
T adds( T a, T b ) { return a + b; }
T subs( T a, T b ) { return a - b; }

#else // int32 formats

T narrow( int64_t x )
{
	if ( x > 2147483647LL || x < -2147483647LL )
	{
		countAt( 0 );
		return x > 0 ? 2147483647 : -2147483647;
	}
	return int( x );
}
// round half up for products that feed state (T10 section 1.3)
int64_t rsh( int64_t x, int sh ) { return ( x + ( int64_t( 1 ) << ( sh - 1 ) ) ) >> sh; }
T M( T a, T b, int sh ) { return narrow( rsh( int64_t( a ) * int64_t( b ), sh ) ); }
T sum3( T a, T b, T c, T d, T e, T f, int sh )
{
	return narrow( rsh( int64_t( a ) * int64_t( b ) + int64_t( c ) * int64_t( d ) + int64_t( e ) * int64_t( f ), sh ) );
}
T dif2( T a, T b, T c, T d, int sh ) { return narrow( rsh( int64_t( a ) * int64_t( b ) - int64_t( c ) * int64_t( d ), sh ) ); }
T zero() { return 0; }
// the alignment of dp (Q5.26) to the anchor scale (Q8.24) is a floor shift
T alignDP( T dp ) { return dp >> ( S_DP - S_R ); }
// sums saturate and count, like the products (T10: velocities saturate inside the sweep)
T adds( T a, T b ) { return narrow( int64_t( a ) + int64_t( b ) ); }
T subs( T a, T b ) { return narrow( int64_t( a ) - int64_t( b ) ); }

// bit length of |x| (0 for 0), for the clz normalisation of V4b
int bitlen64( int64_t x )
{
	uint64_t u = uint64_t( x < 0 ? -x : x );
	uint hi = uint( u >> 32 );
	uint lo = uint( u );
	return hi != 0u ? 33 + int( firstbithigh( hi ) ) : ( lo != 0u ? 1 + int( firstbithigh( lo ) ) : 0 );
}

#endif

T maxT( T a, T b ) { return a > b ? a : b; }

// ------------------------------------------------------------------------------------------------
// Vectors
// ------------------------------------------------------------------------------------------------

struct V3
{
	T x, y, z;
};
V3 v3( T x, T y, T z ) { V3 r; r.x = x; r.y = y; r.z = z; return r; }
V3 ld3( T a[3] ) { return v3( a[0], a[1], a[2] ); }
V3 add3( V3 a, V3 b ) { return v3( adds( a.x, b.x ), adds( a.y, b.y ), adds( a.z, b.z ) ); }
V3 sub3( V3 a, V3 b ) { return v3( subs( a.x, b.x ), subs( a.y, b.y ), subs( a.z, b.z ) ); }
V3 mulSV( T s, V3 a, int sh ) { return v3( M( s, a.x, sh ), M( s, a.y, sh ), M( s, a.z, sh ) ); }
T dot3( V3 a, V3 b, int sh ) { return sum3( a.x, b.x, a.y, b.y, a.z, b.z, sh ); }
V3 cross3( V3 a, V3 b, int sh )
{
	return v3( dif2( a.y, b.z, a.z, b.y, sh ), dif2( a.z, b.x, a.x, b.z, sh ), dif2( a.x, b.y, a.y, b.x, sh ) );
}
V3 mulMV( T m[9], V3 a, int sh )
{
	return v3( sum3( m[0], a.x, m[1], a.y, m[2], a.z, sh ), sum3( m[3], a.x, m[4], a.y, m[5], a.z, sh ),
			   sum3( m[6], a.x, m[7], a.y, m[8], a.z, sh ) );
}

// b3RotateVector: v + 2 s (q x v) + 2 q x (q x v)
V3 rotate( T q[4], V3 a )
{
	V3 qv = v3( q[0], q[1], q[2] );
	V3 t = cross3( qv, a, S_Q );
	t = add3( t, t );
	V3 u = cross3( qv, t, S_Q );
	return add3( add3( a, mulSV( q[3], t, S_Q ) ), u );
}

// ------------------------------------------------------------------------------------------------
// Impulse application
// ------------------------------------------------------------------------------------------------

// w += sign * invI (r x P)
V3 applyAngular( V3 w, T invI[9], V3 r, V3 P, int shI, bool subtract )
{
#if defined( VAR_V4B )
	int64_t lx = int64_t( r.y ) * int64_t( P.z ) - int64_t( r.z ) * int64_t( P.y );
	int64_t ly = int64_t( r.z ) * int64_t( P.x ) - int64_t( r.x ) * int64_t( P.z );
	int64_t lz = int64_t( r.x ) * int64_t( P.y ) - int64_t( r.y ) * int64_t( P.x );
	int bits = max( bitlen64( lx ), max( bitlen64( ly ), bitlen64( lz ) ) );
	int k = max( 0, bits - 31 ); // keep 31 bits
	V3 L = v3( int( lx >> k ), int( ly >> k ), int( lz >> k ) );
	int sh = shI + S_R - k;
	if ( sh < 1 || sh > 63 ) // a shift by >= 64 is undefined: NVIDIA sign-fills, x86 and Intel mask the count
	{
		countAt( 1 );
		sh = sh < 1 ? 1 : 63;
	}
	V3 d = mulMV( invI, L, sh );
#else
	V3 d = mulMV( invI, cross3( r, P, S_R ), shI );
#endif
	return subtract ? sub3( w, d ) : add3( w, d );
}

// -normalMass * (massScale * vn + bias) - impulseScale * lambda
T normalDelta( RRow r, T ms, T vn64hiPart, T vn, T bias, T is, T lambda )
{
#if defined( VAR_V4B )
	return zero(); // unused: V4b computes this inline with 64-bit normalisation
#else
	return subs( -M( r.normalMass, adds( M( ms, vn, S_MS ), bias ), r.shN ), M( is, lambda, S_MS ) );
#endif
}

#if defined( VAR_V4B )
// V4b: vn kept in 64 bits and normalised to 31 bits before the mass products.
T normalDeltaB( RRow r, T ms, int64_t vn64, T bias, T is, T lambda )
{
	// vn64 at scale S_V + S_Q; bias at S_V
	int kv = max( 0, bitlen64( vn64 ) - 31 );
	T vn31 = int( vn64 >> kv );													 // scale S_V + S_Q - kv
	int64_t term = rsh( int64_t( ms ) * int64_t( vn31 ), S_MS );				 // scale S_V + S_Q - kv
	int64_t biasAligned = kv <= S_Q ? ( int64_t( bias ) << ( S_Q - kv ) ) : ( int64_t( bias ) >> ( kv - S_Q ) );
	int64_t sum = term + biasAligned; // |bias| < 2^31, shift <= 30
	int ks = max( 0, bitlen64( sum ) - 31 );
	T sum31 = int( sum >> ks );													 // scale S_V + S_Q - kv - ks
	int sh = r.shN + S_Q - kv - ks;
	if ( sh < 1 || sh > 63 )
	{
		countAt( 1 );
		sh = sh < 1 ? 1 : 63;
	}
	return subs( -narrow( rsh( int64_t( r.normalMass ) * int64_t( sum31 ), sh ) ), M( is, lambda, S_MS ) );
}
#endif

void solvePoint( RRow r, inout V3 vA, inout V3 wA, inout V3 vB, inout V3 wB, inout T lambda, bool relaxRow )
{
	V3 rA = ld3( r.rA );
	V3 rB = ld3( r.rB );
	V3 n = ld3( r.n );
	V3 dp = v3( subs( alignDP( r.B.dp[0] ), alignDP( r.A.dp[0] ) ), subs( alignDP( r.B.dp[1] ), alignDP( r.A.dp[1] ) ),
				subs( alignDP( r.B.dp[2] ), alignDP( r.A.dp[2] ) ) );
	V3 ds = add3( dp, sub3( rotate( r.B.dq, rB ), rotate( r.A.dq, rA ) ) );
	T s = adds( dot3( ds, n, S_R + S_Q - S_S ), r.baseSep );

	T bias, ms, is;
	if ( relaxRow )
	{
		bias = s > zero() ? M( s, r.invH, S_S + S_IH - S_V ) : zero();
		ms = r.massScale; // unused in the relax row below
		is = zero();
	}
	else if ( s > zero() )
	{
		bias = M( s, r.invH, S_S + S_IH - S_V );
		ms = r.massScale;
		is = zero();
#if defined( VAR_F )
		ms = 1.0f;
#elif defined( VAR_I64 )
		ms = int64_t( 1 ) << 32;
#else
		ms = 1 << S_MS;
#endif
	}
	else
	{
		T msbr = M( r.massScale, r.biasRate, S_MS );
		bias = maxT( M( msbr, s, S_BR + S_S - S_V ), -r.contactSpeed );
		ms = r.massScale;
		is = r.impulseScale;
	}

	V3 vrA = add3( vA, cross3( wA, rA, S_W + S_R - S_V ) );
	V3 vrB = add3( vB, cross3( wB, rB, S_W + S_R - S_V ) );
	V3 dv = sub3( vrB, vrA );
	T delta;
#if defined( VAR_V4B )
	int64_t vn64 = int64_t( dv.x ) * int64_t( n.x ) + int64_t( dv.y ) * int64_t( n.y ) + int64_t( dv.z ) * int64_t( n.z );
	if ( relaxRow )
	{
		// -normalMass * (vn + bias): massScale 1 exactly, impulseScale 0
		delta = normalDeltaB( r, 1 << S_MS, vn64, bias, zero(), lambda );
	}
	else
	{
		delta = normalDeltaB( r, ms, vn64, bias, is, lambda );
	}
#else
	T vn = dot3( dv, n, S_Q );
	if ( relaxRow )
	{
		delta = -M( r.normalMass, adds( vn, bias ), r.shN );
	}
	else
	{
		delta = normalDelta( r, ms, zero(), vn, bias, is, lambda );
	}
#endif
	T newImpulse = maxT( adds( lambda, delta ), zero() );
	delta = subs( newImpulse, lambda );
	lambda = newImpulse;

	V3 P = mulSV( delta, n, S_Q );
	vA = sub3( vA, mulSV( r.A.invMass, P, r.shMA ) );
	wA = applyAngular( wA, r.A.invI, rA, P, r.shIA, true );
	vB = add3( vB, mulSV( r.B.invMass, P, r.shMB ) );
	wB = applyAngular( wB, r.B.invI, rB, P, r.shIB, false );
}

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void rowMain( uint3 tid : SV_DispatchThreadID )
{
	uint i = tid.x;
	if ( i >= rp.count )
		return;
	RRow r = rows[i];
	V3 vA = ld3( r.A.v ), wA = ld3( r.A.w ), vB = ld3( r.B.v ), wB = ld3( r.B.w );
	T lambda = r.impulse;
	solvePoint( r, vA, wA, vB, wB, lambda, false );
	ROut o;
	o.pushImpulse = lambda;
	solvePoint( r, vA, wA, vB, wB, lambda, true );
	o.vA[0] = vA.x; o.vA[1] = vA.y; o.vA[2] = vA.z;
	o.wA[0] = wA.x; o.wA[1] = wA.y; o.wA[2] = wA.z;
	o.vB[0] = vB.x; o.vB[1] = vB.y; o.vB[2] = vB.z;
	o.wB[0] = wB.x; o.wB[1] = wB.y; o.wB[2] = wB.z;
	o.impulse = lambda;
	outs[i] = o;
}
