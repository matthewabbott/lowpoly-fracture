// SPDX-License-Identifier: MIT
// Vector maths for lpf: vectors, quaternions, transforms, boxes. Taken from Box3D's math_functions.h (MIT, Erin
// Catto), keeping each operation and its order so results stay bit-identical; only what lpf uses is here.
// Determinism: plain IEEE operations and sqrtf only (correctly rounded everywhere); the trig is our own
// (lpComputeCosSin, lpAtan2), never the C library's (docs/determinism-rules.md).

#pragma once

#include <float.h>
#include <math.h>
#include <stdbool.h>

// Until every Box3D call goes through the physics interface (milestone 8, step 3), the types are Box3D's own, so
// values pass to Box3D unconverted.
#include "box3d/math_functions.h"

#ifdef __cplusplus
extern "C" {
#define LP_LITERAL( T ) T
#else
#define LP_LITERAL( T ) ( T )
#endif

#define LP_PI 3.14159265359f

#if defined( _MSC_VER )
#define LP_FORCE_INLINE static __forceinline
#elif defined( __GNUC__ ) || defined( __clang__ )
#define LP_FORCE_INLINE static inline __attribute__( ( always_inline ) )
#else
#define LP_FORCE_INLINE static inline
#endif

typedef b3Vec3 lpVec3;
typedef b3CosSin lpCosSin; // cosine, sine
typedef b3Quat lpQuat;	   // v, s
typedef b3Transform lpTransform;
typedef b3Matrix3 lpMatrix3; // columns cx, cy, cz
typedef b3AABB lpAABB;		 // lowerBound, upperBound
typedef b3Plane lpPlane;	 // separation = dot(normal, point) - offset

// A world position, and a transform with one. The same as lpVec3 and lpTransform today; kept apart so world
// positions can widen (the integer core's 64-bit positions) without touching every local vector.
typedef lpVec3 lpPos;
typedef lpTransform lpWorldTransform;

static const lpVec3 lpVec3_zero = { 0.0f, 0.0f, 0.0f };
static const lpQuat lpQuat_identity = { { 0.0f, 0.0f, 0.0f }, 1.0f };
static const lpTransform lpTransform_identity = { { 0.0f, 0.0f, 0.0f }, { { 0.0f, 0.0f, 0.0f }, 1.0f } };

// Arctangent in [-pi, pi], hand coded for cross-platform determinism; accurate to about 0.0023 degrees.
float lpAtan2( float y, float x );

// Cosine and sine of an angle in radians, normalised; hand coded for cross-platform determinism.
lpCosSin lpComputeCosSin( float radians );

// The quaternion of a rotation matrix
lpQuat lpMakeQuatFromMatrix( const lpMatrix3* m );

// The shortest rotation taking unit vector v1 to unit vector v2
lpQuat lpComputeQuatBetweenUnitVectors( lpVec3 v1, lpVec3 v2 );

// Not NaN or infinity
bool lpIsValidVec3( lpVec3 a );

static inline int lpMinInt( int a, int b )
{
	return a < b ? a : b;
}

static inline int lpMaxInt( int a, int b )
{
	return a > b ? a : b;
}

static inline float lpAbsFloat( float a )
{
	return a < 0 ? -a : a;
}

static inline float lpMinFloat( float a, float b )
{
	return a < b ? a : b;
}

static inline float lpMaxFloat( float a, float b )
{
	return a > b ? a : b;
}

static inline float lpClampFloat( float a, float lower, float upper )
{
	return a < lower ? lower : ( upper < a ? upper : a );
}

static inline lpVec3 lpAdd( lpVec3 a, lpVec3 b )
{
	return LP_LITERAL( lpVec3 ){ a.x + b.x, a.y + b.y, a.z + b.z };
}

static inline lpVec3 lpSub( lpVec3 a, lpVec3 b )
{
	return LP_LITERAL( lpVec3 ){ a.x - b.x, a.y - b.y, a.z - b.z };
}

static inline lpVec3 lpNeg( lpVec3 a )
{
	return LP_LITERAL( lpVec3 ){ -a.x, -a.y, -a.z };
}

static inline float lpDot( lpVec3 a, lpVec3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline float lpLength( lpVec3 v )
{
	return sqrtf( lpDot( v, v ) );
}

static inline float lpLengthSquared( lpVec3 a )
{
	return a.x * a.x + a.y * a.y + a.z * a.z;
}

static inline float lpDistance( lpVec3 a, lpVec3 b )
{
	lpVec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return lpLength( dv );
}

static inline float lpDistanceSquared( lpVec3 a, lpVec3 b )
{
	lpVec3 dv = { b.x - a.x, b.y - a.y, b.z - a.z };
	return dv.x * dv.x + dv.y * dv.y + dv.z * dv.z;
}

// The unit vector along a, or zero if a is very small
LP_FORCE_INLINE lpVec3 lpNormalize( lpVec3 a )
{
	float lengthSquared = a.x * a.x + a.y * a.y + a.z * a.z;
	if ( lengthSquared > 1000.0f * FLT_MIN )
	{
		float s = 1.0f / sqrtf( lengthSquared );
		lpVec3 u = { s * a.x, s * a.y, s * a.z };
		return u;
	}
	return LP_LITERAL( lpVec3 ){ 0.0f, 0.0f, 0.0f };
}

// a + s * b
static inline lpVec3 lpMulAdd( lpVec3 a, float s, lpVec3 b )
{
	return LP_LITERAL( lpVec3 ){ a.x + s * b.x, a.y + s * b.y, a.z + s * b.z };
}

// a - s * b
static inline lpVec3 lpMulSub( lpVec3 a, float s, lpVec3 b )
{
	return LP_LITERAL( lpVec3 ){ a.x - s * b.x, a.y - s * b.y, a.z - s * b.z };
}

// s * a
static inline lpVec3 lpMulSV( float s, lpVec3 a )
{
	return LP_LITERAL( lpVec3 ){ s * a.x, s * a.y, s * a.z };
}

static inline lpVec3 lpCross( lpVec3 a, lpVec3 b )
{
	lpVec3 c;
	c.x = a.y * b.z - a.z * b.y;
	c.y = a.z * b.x - a.x * b.z;
	c.z = a.x * b.y - a.y * b.x;
	return c;
}

// (1 - alpha) * a + alpha * b, alpha in [0, 1]
static inline lpVec3 lpLerp( lpVec3 a, lpVec3 b, float alpha )
{
	lpVec3 c = {
		( 1.0f - alpha ) * a.x + alpha * b.x,
		( 1.0f - alpha ) * a.y + alpha * b.y,
		( 1.0f - alpha ) * a.z + alpha * b.z,
	};
	return c;
}

static inline lpVec3 lpAbs( lpVec3 a )
{
	return LP_LITERAL( lpVec3 ){ lpAbsFloat( a.x ), lpAbsFloat( a.y ), lpAbsFloat( a.z ) };
}

static inline lpVec3 lpMin( lpVec3 a, lpVec3 b )
{
	return LP_LITERAL( lpVec3 ){ lpMinFloat( a.x, b.x ), lpMinFloat( a.y, b.y ), lpMinFloat( a.z, b.z ) };
}

static inline lpVec3 lpMax( lpVec3 a, lpVec3 b )
{
	return LP_LITERAL( lpVec3 ){ lpMaxFloat( a.x, b.x ), lpMaxFloat( a.y, b.y ), lpMaxFloat( a.z, b.z ) };
}

static inline lpVec3 lpClamp( lpVec3 a, lpVec3 lower, lpVec3 upper )
{
	lpVec3 b;
	b.x = lpClampFloat( a.x, lower.x, upper.x );
	b.y = lpClampFloat( a.y, lower.y, upper.y );
	b.z = lpClampFloat( a.z, lower.z, upper.z );
	return b;
}

static inline lpVec3 lpRotateVector( lpQuat q, lpVec3 v )
{
	// v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
	lpVec3 t1 = lpCross( q.v, v );
	lpVec3 t2 = lpMulAdd( t1, q.s, v );
	lpVec3 t3 = lpCross( q.v, t2 );
	return lpMulAdd( v, 2.0f, t3 );
}

static inline lpVec3 lpInvRotateVector( lpQuat q, lpVec3 v )
{
	// v + 2 * cross(q.v, cross(q.v, v) - q.s * v)
	lpVec3 t1 = lpCross( q.v, v );
	lpVec3 t2 = lpMulSub( t1, q.s, v );
	lpVec3 t3 = lpCross( q.v, t2 );
	return lpMulAdd( v, 2.0f, t3 );
}

static inline float lpDotQuat( lpQuat a, lpQuat b )
{
	return a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z + a.s * b.s;
}

static inline lpQuat lpMulQuat( lpQuat q1, lpQuat q2 )
{
	lpVec3 t1 = lpCross( q1.v, q2.v );
	lpVec3 t2 = lpMulAdd( t1, q1.s, q2.v );
	lpVec3 t3 = lpMulAdd( t2, q2.s, q1.v );
	lpQuat q = { t3, q1.s * q2.s - lpDot( q1.v, q2.v ) };
	return q;
}

// inv(q1) * q2
static inline lpQuat lpInvMulQuat( lpQuat q1, lpQuat q2 )
{
	lpVec3 t1 = lpCross( q2.v, q1.v );
	lpVec3 t2 = lpMulAdd( t1, q1.s, q2.v );
	lpVec3 t3 = lpMulSub( t2, q2.s, q1.v );
	lpQuat q = { t3, q1.s * q2.s + lpDot( q1.v, q2.v ) };
	return q;
}

static inline lpQuat lpConjugate( lpQuat q )
{
	return LP_LITERAL( lpQuat ){ { -q.v.x, -q.v.y, -q.v.z }, q.s };
}

// The unit quaternion along q, or the identity if q is very small
static inline lpQuat lpNormalizeQuat( lpQuat q )
{
	float lengthSq = lpDotQuat( q, q );
	if ( lengthSq > 1000.0f * FLT_MIN )
	{
		float s = 1.0f / sqrtf( lengthSq );
		lpQuat qn = { { s * q.v.x, s * q.v.y, s * q.v.z }, s * q.s };
		return qn;
	}

	return lpQuat_identity;
}

// The rotation by an angle about a unit axis
static inline lpQuat lpMakeQuatFromAxisAngle( lpVec3 axis, float radians )
{
	lpCosSin cs = lpComputeCosSin( 0.5f * radians );
	lpQuat q = { { cs.sine * axis.x, cs.sine * axis.y, cs.sine * axis.z }, cs.cosine };
	return q;
}

// a * b: a point local to frame b, to frame a's parent
static inline lpTransform lpMulTransforms( lpTransform a, lpTransform b )
{
	lpTransform out;
	out.p = lpAdd( lpRotateVector( a.q, b.p ), a.p );
	out.q = lpMulQuat( a.q, b.q );
	return out;
}

LP_FORCE_INLINE lpVec3 lpTransformPoint( lpTransform t, lpVec3 v )
{
	lpVec3 rv = lpRotateVector( t.q, v );
	return lpAdd( rv, t.p );
}

static inline lpPos lpToPos( lpVec3 v )
{
	return LP_LITERAL( lpPos ){ v.x, v.y, v.z };
}

static inline lpVec3 lpToVec3( lpPos p )
{
	return LP_LITERAL( lpVec3 ){ (float)p.x, (float)p.y, (float)p.z };
}

// a - b, as a vector
static inline lpVec3 lpSubPos( lpPos a, lpPos b )
{
	return LP_LITERAL( lpVec3 ){ (float)( a.x - b.x ), (float)( a.y - b.y ), (float)( a.z - b.z ) };
}

// p + d
static inline lpPos lpOffsetPos( lpPos p, lpVec3 d )
{
	return LP_LITERAL( lpPos ){ p.x + d.x, p.y + d.y, p.z + d.z };
}

// A local point to a world position
static inline lpPos lpTransformWorldPoint( lpWorldTransform t, lpVec3 p )
{
	lpVec3 r = lpRotateVector( t.q, p );
	return LP_LITERAL( lpPos ){ t.p.x + r.x, t.p.y + r.y, t.p.z + r.z };
}

// A world position to a local point
static inline lpVec3 lpInvTransformWorldPoint( lpWorldTransform t, lpPos p )
{
	lpVec3 d = { (float)( p.x - t.p.x ), (float)( p.y - t.p.y ), (float)( p.z - t.p.z ) };
	return lpInvRotateVector( t.q, d );
}

static inline float lpDet( lpMatrix3 m )
{
	return lpDot( m.cx, lpCross( m.cy, m.cz ) );
}

// m * a
static inline lpVec3 lpMulMV( lpMatrix3 m, lpVec3 a )
{
	lpVec3 b = {
		m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z,
		m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z,
		m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z,
	};
	return b;
}

static inline lpMatrix3 lpTranspose( lpMatrix3 m )
{
	lpMatrix3 out;
	out.cx = LP_LITERAL( lpVec3 ){ m.cx.x, m.cy.x, m.cz.x };
	out.cy = LP_LITERAL( lpVec3 ){ m.cx.y, m.cy.y, m.cz.y };
	out.cz = LP_LITERAL( lpVec3 ){ m.cx.z, m.cy.z, m.cz.z };
	return out;
}

// The inverse of m, or zero if m is singular
static inline lpMatrix3 lpInvertMatrix( lpMatrix3 m )
{
	float det = lpDet( m );
	if ( lpAbsFloat( det ) > 1000.0f * FLT_MIN )
	{
		float invDet = 1.0f / det;
		lpMatrix3 out;
		out.cx = lpMulSV( invDet, lpCross( m.cy, m.cz ) );
		out.cy = lpMulSV( invDet, lpCross( m.cz, m.cx ) );
		out.cz = lpMulSV( invDet, lpCross( m.cx, m.cy ) );

		return lpTranspose( out );
	}

	return LP_LITERAL( lpMatrix3 ){ { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };
}

// inv(m) * a, or zero if m is singular
LP_FORCE_INLINE lpVec3 lpSolve3( lpMatrix3 m, lpVec3 a )
{
	float det = lpDet( m );
	if ( lpAbsFloat( det ) > 1000.0f * FLT_MIN )
	{
		float invDet = 1.0f / det;
		lpMatrix3 s;
		s.cx = lpCross( m.cy, m.cz );
		s.cy = lpCross( m.cz, m.cx );
		s.cz = lpCross( m.cx, m.cy );

		lpVec3 b = {
			invDet * lpDot( s.cx, a ),
			invDet * lpDot( s.cy, a ),
			invDet * lpDot( s.cz, a ),
		};

		return b;
	}

	return lpVec3_zero;
}

static inline lpVec3 lpAABB_Center( lpAABB a )
{
	return lpMulSV( 0.5f, lpAdd( a.upperBound, a.lowerBound ) );
}

// Half-widths
static inline lpVec3 lpAABB_Extents( lpAABB a )
{
	return lpMulSV( 0.5f, lpSub( a.upperBound, a.lowerBound ) );
}

#ifdef __cplusplus
}
#endif
