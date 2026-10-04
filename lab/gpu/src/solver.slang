// solver.slang: a Box3D-style soft-step contact solve (b3WarmStartContacts / b3PushContacts /
// b3SolveContacts of the convex path, extern/box3d/src/contact_solver.c:1727-2066, plus the
// integrate stages of solver.c:65-240), one source for three number formats (see layout.h).
// Compiled to SPIR-V for the GPUs and to C++ for the CPU twin.
//
// Every product carries the shift that its I32 form needs; F and I64 ignore it.
// Clamps and selects are explicit ternaries (never min/max intrinsics: their +-0/NaN rules vary).

#include "layout.h"

[[vk::binding( 0, 0 )]] RWStructuredBuffer<Body> bodies;
[[vk::binding( 1, 0 )]] RWStructuredBuffer<Pose> poses;
[[vk::binding( 2, 0 )]] StructuredBuffer<Constraint> constraints;
[[vk::binding( 3, 0 )]] RWStructuredBuffer<Impulse> impulses;
[[vk::binding( 4, 0 )]] StructuredBuffer<Params> params;
[[vk::binding( 5, 0 )]] RWStructuredBuffer<uint> counters;
[[vk::push_constant]] ConstantBuffer<Push> pc;

// ------------------------------------------------------------------------------------------------
// Arithmetic per format
// ------------------------------------------------------------------------------------------------

#if defined( VAR_F )

T M( T a, T b, int sh ) { return a * b; }
T sum2( T a, T b, T c, T d, int sh ) { return a * b + c * d; }	 // (a*b) + (c*d)
T dif2( T a, T b, T c, T d, int sh ) { return a * b - c * d; }	 // (a*b) - (c*d)
T sum3( T a, T b, T c, T d, T e, T f, int sh ) { return a * b + c * d + e * f; }
T zero() { return 0.0f; }
TP addPos( TP p, T d ) { return p + d; }
// 2^-ceil(e/2) for u = m 2^e, u >= 1, from the exponent bits (integer ops only)
T rsqrtSeed( T u )
{
	int e = int( ( asuint( u ) >> 23 ) & 0xffu ) - 127;
	int k = ( e + 1 ) >> 1;
	return asfloat( uint( 127 - k ) << 23 );
}

#elif defined( VAR_I64 )

static const uint64_t LOW32 = uint64_t( 0xFFFFFFFFu );

// Q32.32 product: the signed 128-bit product from four 32x32->64 partial products (the top one
// only needs its low 32 bits), then bits 32..95 rounded half up (add 2^31, drop the low 32 bits).
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
	// signed correction of bits 64..95: subtract b * 2^64 if a < 0 and a * 2^64 if b < 0
	hi = a < 0 ? hi - bl : hi;
	hi = b < 0 ? hi - al : hi;
	return int64_t( ( uint64_t( hi ) << 32 ) | ( mid & LOW32 ) );
}

T M( T a, T b, int sh ) { return qmul( a, b ); }
T sum2( T a, T b, T c, T d, int sh ) { return qmul( a, b ) + qmul( c, d ); }
T dif2( T a, T b, T c, T d, int sh ) { return qmul( a, b ) - qmul( c, d ); }
T sum3( T a, T b, T c, T d, T e, T f, int sh ) { return qmul( a, b ) + qmul( c, d ) + qmul( e, f ); }
T zero() { return 0; }
TP addPos( TP p, T d ) { return p + d; }
T rsqrtSeed( T u )
{
	int e = int( firstbithigh( uint( uint64_t( u ) >> 32 ) ) );
	int k = ( e + 1 ) >> 1;
	return int64_t( 1 ) << ( 32 - k );
}

#elif defined( VAR_I32 ) || defined( VAR_V4 )

#if defined( VAR_V4 )
// V4: round half up, then saturate to 32 bits and count the saturation (T10's counters).
T narrow( int64_t x )
{
	if ( x > 2147483647LL || x < -2147483647LL )
	{
#if defined( TWIN_CPP )
		counters[0] = counters[0] + 1u;
#else
		InterlockedAdd( counters[0], 1u );
#endif
		return x > 0 ? 2147483647 : -2147483647;
	}
	return int( x );
}
#else
// Round half up and narrow to 32 bits (wraps; CHECK_RANGE counts values that do not fit).
T narrow( int64_t x )
{
#if defined( CHECK_RANGE )
	if ( x > 2147483647LL || x < -2147483648LL )
	{
#if defined( TWIN_CPP )
		counters[0] = counters[0] + 1u; // the twin's range check runs single-threaded
#else
		InterlockedAdd( counters[0], 1u );
#endif
	}
#endif
	return int( x );
}
#endif
int64_t rsh( int64_t x, int sh ) { return ( x + ( int64_t( 1 ) << ( sh - 1 ) ) ) >> sh; }

// Sums of products accumulate exactly in 64 bits and round once.
T M( T a, T b, int sh ) { return narrow( rsh( int64_t( a ) * int64_t( b ), sh ) ); }
T sum2( T a, T b, T c, T d, int sh ) { return narrow( rsh( int64_t( a ) * int64_t( b ) + int64_t( c ) * int64_t( d ), sh ) ); }
T dif2( T a, T b, T c, T d, int sh ) { return narrow( rsh( int64_t( a ) * int64_t( b ) - int64_t( c ) * int64_t( d ), sh ) ); }
T sum3( T a, T b, T c, T d, T e, T f, int sh )
{
	return narrow( rsh( int64_t( a ) * int64_t( b ) + int64_t( c ) * int64_t( d ) + int64_t( e ) * int64_t( f ), sh ) );
}
T zero() { return 0; }
TP addPos( TP p, T d ) { return p + ( int64_t( d ) << ( 32 - S_DP ) ); }
T rsqrtSeed( T u )
{
	int e = int( firstbithigh( uint( u ) ) ) - S_U;
	int k = ( e + 1 ) >> 1;
	return 1 << ( 30 - k );
}

#endif

// Per-(constraint, body) shifts: V4 precomputes them in prepare; I32 adds the exponents here.
#if defined( VAR_V4 )
int shMA( Constraint c ) { return c.eMA; }
int shMB( Constraint c ) { return c.eMB; }
int shIA( Constraint c ) { return c.eIA; }
int shIB( Constraint c ) { return c.eIB; }
int shN( Constraint c, int j ) { return c.points[j].shN; }
int shT( Constraint c ) { return c.eT; }
#else
int shMA( Constraint c ) { return c.eMA + c.eC - S_V; }
int shMB( Constraint c ) { return c.eMB + c.eC - S_V; }
int shIA( Constraint c ) { return c.eIA + c.eC - S_W; }
int shIB( Constraint c ) { return c.eIB + c.eC - S_W; }
int shN( Constraint c, int j ) { return c.eN + S_V - c.eC; }
int shT( Constraint c ) { return c.eT + S_V - c.eC; }
#endif

T selectT( bool c, T a, T b ) { return c ? a : b; }
T maxT( T a, T b ) { return a > b ? a : b; } // b3MaxFloat
T clampT( T a, T lo, T hi ) { return a < lo ? lo : ( a > hi ? hi : a ); }

// ------------------------------------------------------------------------------------------------
// Vector helpers in Box3D's operation order
// ------------------------------------------------------------------------------------------------

V3 v3( T x, T y, T z ) { V3 r; r.x = x; r.y = y; r.z = z; return r; }
V3 add3( V3 a, V3 b ) { return v3( a.x + b.x, a.y + b.y, a.z + b.z ); }
V3 sub3( V3 a, V3 b ) { return v3( a.x - b.x, a.y - b.y, a.z - b.z ); }
V3 zero3() { return v3( zero(), zero(), zero() ); }
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
// a * u + b * w per component
V3 blend2( T a, V3 u, T b, V3 w, int sh )
{
	return v3( sum2( a, u.x, b, w.x, sh ), sum2( a, u.y, b, w.y, sh ), sum2( a, u.z, b, w.z, sh ) );
}

// b3InvRotateVectorW (contact_solver.c:1117)
V3 invRotate( Q4 q, V3 a )
{
	V3 qv = v3( q.x, q.y, q.z );
	V3 t = cross3( qv, a, S_Q );
	t = add3( t, t );
	V3 u = cross3( qv, t, S_Q );
	return add3( sub3( a, mulSV( q.s, t, S_Q ) ), u );
}

// b3MulQuat (math_functions.h:479)
Q4 quatMul( Q4 q1, Q4 q2 )
{
	V3 v1 = v3( q1.x, q1.y, q1.z );
	V3 v2 = v3( q2.x, q2.y, q2.z );
	V3 t1 = cross3( v1, v2, S_Q );
	V3 t2 = mulAddSV( t1, q1.s, v2, S_Q );
	V3 t3 = mulAddSV( t2, q2.s, v1, S_Q );
	Q4 r;
	r.x = t3.x;
	r.y = t3.y;
	r.z = t3.z;
	r.s = M( q1.s, q2.s, S_Q ) - dot3( v1, v2, S_Q );
	return r;
}

// Normalise with three Newton steps for 1/sqrt from y = 1 (|q|^2 is 1 + O((h|w|)^2) here), so no
// sqrt or division: Box3D's b3NormalizeQuat uses 1/sqrtf.
Q4 quatNormalize( Q4 q, Params P )
{
	T x = M( q.x, q.x, S_Q ) + M( q.y, q.y, S_Q ) + M( q.z, q.z, S_Q ) + M( q.s, q.s, S_Q );
	T hx = M( P.half, x, S_Q );
	T y = P.one;
	for ( int i = 0; i < 3; ++i )
	{
		y = M( y, P.oneHalf - M( hx, M( y, y, S_Q ), S_Q ), S_Q );
	}
	Q4 r;
	r.x = M( q.x, y, S_Q );
	r.y = M( q.y, y, S_Q );
	r.z = M( q.z, y, S_Q );
	r.s = M( q.s, y, S_Q );
	return r;
}

Q4 quatIdentity( Params P )
{
	Q4 r;
	r.x = zero();
	r.y = zero();
	r.z = zero();
	r.s = P.one;
	return r;
}

// ------------------------------------------------------------------------------------------------
// Body stages (index 0 is the static body and is skipped)
// ------------------------------------------------------------------------------------------------

// b3IntegrateVelocitiesTask: gravity only (no damping, forces, torques or gyroscopic term).
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void integrateVelocities( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = pc.start + tid.x;
	Params P = params[0];
	Body b = bodies[i];
	b.v = add3( P.gravityDelta, b.v ); // b3MulAdd( delta, damping = 1, v )
	bodies[i] = b;
}

// b3IntegratePositionsTask. The angular speed cap (0.25 pi per step) without sqrt or division:
// u = |w / maxSpeed|^2 and, above 1, w *= 1/sqrt(u) by six Newton steps from an exponent seed.
// No linear speed cap (Box3D's 400 m/s is never reached here).
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void integratePositions( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = pc.start + tid.x;
	Params P = params[0];
	Body b = bodies[i];
	V3 wn = mulSV( P.invMaxAngularSpeed, b.w, S_MS );
	T u = dot3( wn, wn, 2 * S_W - S_U );
#if defined( NO_SPEED_CAP )
	if ( false )
#else
	if ( u > P.oneU )
#endif
	{
		T hu = M( P.half, u, S_MS );
		T y = rsqrtSeed( u );
		for ( int it = 0; it < 6; ++it )
		{
			y = M( y, P.oneHalf - M( hu, M( y, y, S_Q ), S_U ), S_Q );
		}
		b.w = mulSV( y, b.w, S_Q );
	}
	b.dp = mulAddSV( b.dp, P.h, b.v, S_H + S_V - S_DP );
	// b3IntegrateRotation( dq, h * w )
	V3 hw = mulSV( P.h, b.w, S_H + S_W - S_Q );
	Q4 qd;
	V3 hh = mulSV( P.half, hw, S_MS );
	qd.x = hh.x;
	qd.y = hh.y;
	qd.z = hh.z;
	qd.s = zero();
	qd = quatMul( qd, b.dq );
	Q4 q2;
	q2.x = b.dq.x + qd.x;
	q2.y = b.dq.y + qd.y;
	q2.z = b.dq.z + qd.z;
	q2.s = qd.s + b.dq.s;
	b.dq = quatNormalize( q2, P );
	bodies[i] = b;
}

// End of step: fold the deltas into the pose and reset them.
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void finalizeBodies( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = pc.start + tid.x;
	Params P = params[0];
	Body b = bodies[i];
	Pose p = poses[i];
	p.px = addPos( p.px, b.dp.x );
	p.py = addPos( p.py, b.dp.y );
	p.pz = addPos( p.pz, b.dp.z );
	p.q = quatNormalize( quatMul( b.dq, p.q ), P );
	b.dp = zero3();
	b.dq = quatIdentity( P );
	poses[i] = p;
	bodies[i] = b;
}

// ------------------------------------------------------------------------------------------------
// Contact stages, one dispatch per colour: no two contacts of a colour share a dynamic body
// ------------------------------------------------------------------------------------------------

void storeBody( int index, Body b )
{
	if ( index != 0 )
	{
		bodies[index] = b;
	}
}

// b3WarmStartContacts_Convex without twist and rolling.
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void warmStart( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint ci = pc.start + tid.x;
	Constraint c = constraints[ci];
	Impulse im = impulses[ci];
	Body bA = bodies[c.indexA];
	Body bB = bodies[c.indexB];

	T totalNormalImpulse = zero();
	V3 momentA = zero3();
	V3 momentB = zero3();
	[ForceUnroll]
	for ( int j = 0; j < 2; ++j )
	{
		if ( j < c.pointCount )
		{
			T normalImpulse = im.normal[j];
			totalNormalImpulse = totalNormalImpulse + normalImpulse;
			momentA = mulAddSV( momentA, normalImpulse, c.points[j].anchorA, S_R );
			momentB = mulAddSV( momentB, normalImpulse, c.points[j].anchorB, S_R );
		}
	}

	V3 frictionImpulse = mulSV( im.friction[0], c.tangent1, S_Q );
	frictionImpulse = mulAddSV( frictionImpulse, im.friction[1], c.tangent2, S_Q );
	V3 linearImpulse = mulAddSV( frictionImpulse, totalNormalImpulse, c.normal, S_Q );
	V3 angularImpulseA = add3( cross3( momentA, c.normal, S_Q ), cross3( c.centerA, frictionImpulse, S_R ) );
	V3 angularImpulseB = add3( cross3( momentB, c.normal, S_Q ), cross3( c.centerB, frictionImpulse, S_R ) );

	bA.w = sub3( bA.w, mulMV( c.invIA, angularImpulseA, shIA( c ) ) );
	bA.v = mulSubSV( bA.v, c.invMassA, linearImpulse, shMA( c ) );
	bB.w = add3( bB.w, mulMV( c.invIB, angularImpulseB, shIB( c ) ) );
	bB.v = mulAddSV( bB.v, c.invMassB, linearImpulse, shMB( c ) );

	storeBody( c.indexA, bA );
	storeBody( c.indexB, bB );
}

// Apply a normal impulse increment P = delta * n at anchors rA, rB.
void applyImpulse( inout Body bA, inout Body bB, Constraint c, V3 rA, V3 rB, V3 P )
{
	bA.w = sub3( bA.w, mulMV( c.invIA, cross3( rA, P, S_R ), shIA( c ) ) );
	bA.v = mulSubSV( bA.v, c.invMassA, P, shMA( c ) );
	bB.w = add3( bB.w, mulMV( c.invIB, cross3( rB, P, S_R ), shIB( c ) ) );
	bB.v = mulAddSV( bB.v, c.invMassB, P, shMB( c ) );
}

T separationAt( Constraint c, int j, T normalSeparation, V3 normalA, V3 normalB )
{
	return ( normalSeparation + ( dot3( normalB, c.points[j].anchorB, S_Q + S_R - S_S ) - dot3( normalA, c.points[j].anchorA, S_Q + S_R - S_S ) ) ) +
		   c.points[j].baseSeparation;
}

T normalVelocity( Body bA, Body bB, V3 rA, V3 rB, V3 n )
{
	V3 vrA = add3( bA.v, cross3( bA.w, rA, S_W + S_R - S_V ) );
	V3 vrB = add3( bB.v, cross3( bB.w, rB, S_W + S_R - S_V ) );
	return dot3( sub3( vrB, vrA ), n, S_Q );
}

// b3PushContacts_Convex: normal rows with the soft bias.
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void push( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint ci = pc.start + tid.x;
	Params P = params[0];
	Constraint c = constraints[ci];
	Impulse im = impulses[ci];
	Body bA = bodies[c.indexA];
	Body bB = bodies[c.indexB];

	bool soft = c.soft != 0;
	T biasRate = selectT( soft, P.staBiasRate, P.dynBiasRate );
	T massScale = selectT( soft, P.staMassScale, P.dynMassScale );
	T impulseScale = selectT( soft, P.staImpulseScale, P.dynImpulseScale );

	V3 dp = sub3( bB.dp, bA.dp );
	T normalSeparation = dot3( c.normal, dp, S_Q + S_DP - S_S );
	V3 normalA = invRotate( bA.dq, c.normal );
	V3 normalB = invRotate( bB.dq, c.normal );

	[ForceUnroll]
	for ( int j = 0; j < 2; ++j )
	{
		if ( j < c.pointCount )
		{
			V3 rA = c.points[j].anchorA;
			V3 rB = c.points[j].anchorB;
			T s = separationAt( c, j, normalSeparation, normalA, normalB );
			bool separated = s > zero();
			T specBias = M( s, P.inv_h, S_S + S_IH - S_V );
			T overlapBias = maxT( M( biasRate, s, S_BR + S_S - S_V ), P.negContactSpeed );
			T velocityBias = selectT( separated, specBias, overlapBias );
			T pointMassScale = selectT( separated, P.one, massScale );
			T pointImpulseScale = selectT( separated, zero(), impulseScale );

			T vn = normalVelocity( bA, bB, rA, rB, c.normal );
			T negImpulse = M( c.points[j].normalMass, M( pointMassScale, vn, S_MS ) + velocityBias, shN( c, j ) ) +
						   M( pointImpulseScale, im.normal[j], S_MS );
			T newImpulse = maxT( im.normal[j] - negImpulse, zero() );
			T deltaImpulse = newImpulse - im.normal[j];
			im.normal[j] = newImpulse;

			applyImpulse( bA, bB, c, rA, rB, mulSV( deltaImpulse, c.normal, S_Q ) );
		}
	}

	impulses[ci] = im;
	storeBody( c.indexA, bA );
	storeBody( c.indexB, bB );
}

// b3SolveContacts_Convex (the relax pass): rigid normal rows, then central friction.
[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void relax( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint ci = pc.start + tid.x;
	Params P = params[0];
	Constraint c = constraints[ci];
	Impulse im = impulses[ci];
	Body bA = bodies[c.indexA];
	Body bB = bodies[c.indexB];

	V3 dp = sub3( bB.dp, bA.dp );
	T normalSeparation = dot3( c.normal, dp, S_Q + S_DP - S_S );
	V3 normalA = invRotate( bA.dq, c.normal );
	V3 normalB = invRotate( bB.dq, c.normal );

	T totalNormalImpulse = zero();
#if defined( INTEL_I64_WORKAROUND )
	[loop]
	for ( int j = 0; j < c.pointCount; ++j )
#else
	[ForceUnroll]
	for ( int j = 0; j < 2; ++j )
#endif
	{
		if ( j < c.pointCount )
		{
			V3 rA = c.points[j].anchorA;
			V3 rB = c.points[j].anchorB;
			T s = separationAt( c, j, normalSeparation, normalA, normalB );
			T velocityBias = maxT( zero(), M( s, P.inv_h, S_S + S_IH - S_V ) );
			T vn = normalVelocity( bA, bB, rA, rB, c.normal );
			T negImpulse = M( c.points[j].normalMass, vn + velocityBias, shN( c, j ) );
			T newImpulse = maxT( im.normal[j] - negImpulse, zero() );
			T deltaImpulse = newImpulse - im.normal[j];
			im.normal[j] = newImpulse;
#if !defined( INTEL_I64_WORKAROUND )
			totalNormalImpulse = totalNormalImpulse + newImpulse;
#endif
			applyImpulse( bA, bB, c, rA, rB, mulSV( deltaImpulse, c.normal, S_Q ) );
		}
	}
#if defined( INTEL_I64_WORKAROUND )
	// Same value (integer addition is exact). The Intel Gen9.5 driver (101.2137) gives the int64 sum
	// the register of im.normal[0] while that value is still live, and the final store then writes
	// the total. Store the normal impulses first; the end of the kernel stores only friction.
	impulses[ci].normal[0] = im.normal[0];
	impulses[ci].normal[1] = im.normal[1];
	totalNormalImpulse = im.normal[0] + ( c.pointCount > 1 ? im.normal[1] : zero() );
#endif

	// Central friction at the friction centre, two rows with the 2x2 tangent mass.
	{
		V3 rA = c.centerA;
		V3 rB = c.centerB;
		V3 vrA = add3( bA.v, cross3( bA.w, rA, S_W + S_R - S_V ) );
		V3 vrB = add3( bB.v, cross3( bB.w, rB, S_W + S_R - S_V ) );
		V3 vr = sub3( vrB, vrA );
		T vtx = dot3( vr, c.tangent1, S_Q );
		T vty = dot3( vr, c.tangent2, S_Q );
		int shTan = shT( c );
		T dx = sum2( c.tangentMass.xx, vtx, c.tangentMass.xy, vty, shTan );
		T dy = sum2( c.tangentMass.xy, vtx, c.tangentMass.yy, vty, shTan );
		T nx = im.friction[0] + ( -dx );
		T ny = im.friction[1] + ( -dy );
#if defined( DIAG_NO_FRICTION_MUL )
		T maxImpulse = totalNormalImpulse;
#else
		T maxImpulse = M( c.friction, totalNormalImpulse, S_MS );
#endif
#if defined( FRICTION_DISC )
		// Box3D's disc clamp (contact_solver.c:2033-2045): a sqrt and a division per contact per relax.
		T lengthSquared = nx * nx + ny * ny;
		bool over = lengthSquared > maxImpulse * maxImpulse;
		T normalize = maxImpulse / ( sqrt( lengthSquared ) + 1.1920928955078125e-7f );
		T scale = selectT( over, normalize, 1.0f );
		nx = scale * nx;
		ny = scale * ny;
#else
		// Box clamp per row (no sqrt, no division).
		T negMax = -maxImpulse;
		nx = clampT( nx, negMax, maxImpulse );
		ny = clampT( ny, negMax, maxImpulse );
#endif
		T ddx = nx - im.friction[0];
		T ddy = ny - im.friction[1];
		im.friction[0] = nx;
		im.friction[1] = ny;
		applyImpulse( bA, bB, c, rA, rB, blend2( ddx, c.tangent1, ddy, c.tangent2, S_Q ) );
	}

#if defined( INTEL_I64_WORKAROUND )
	impulses[ci].friction[0] = im.friction[0];
	impulses[ci].friction[1] = im.friction[1];
#else
	impulses[ci] = im;
#endif
	storeBody( c.indexA, bA );
	storeBody( c.indexB, bB );
}
