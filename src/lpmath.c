// SPDX-License-Identifier: MIT
// The out-of-line part of lpmath.h, taken from Box3D's math_functions.c (MIT, Erin Catto) with each operation kept.

#include "core.h"

#include <math.h>

bool lpIsValidVec3( lpVec3 a )
{
	if ( isnan( a.x ) || isnan( a.y ) || isnan( a.z ) )
	{
		return false;
	}

	if ( isinf( a.x ) || isinf( a.y ) || isinf( a.z ) )
	{
		return false;
	}

	return true;
}

// https://stackoverflow.com/questions/46210708/atan2-approximation-with-11bits-in-mantissa-on-x86with-sse2-and-armwith-vfpv4
float lpAtan2( float y, float x )
{
	// (0, 0) gives 0, as atan2f does, instead of NaN
	if ( x == 0.0f && y == 0.0f )
	{
		return 0.0f;
	}

	float ax = lpAbsFloat( x );
	float ay = lpAbsFloat( y );
	float mx = lpMaxFloat( ay, ax );
	float mn = lpMinFloat( ay, ax );
	float a = mn / mx;

	// Minimax polynomial approximation to atan(a) on [0,1]
	float s = a * a;
	float c = s * a;
	float q = s * s;
	float r = 0.024840285f * q + 0.18681418f;
	float t = -0.094097948f * q - 0.33213072f;
	r = r * s + t;
	r = r * c + a;

	// Map to full circle
	if ( ay > ax )
	{
		r = 1.57079637f - r;
	}

	if ( x < 0 )
	{
		r = 3.14159274f - r;
	}

	if ( y < 0 )
	{
		r = -r;
	}

	return r;
}

// Bhaskara I's sine approximation, normalised so cosine^2 + sine^2 = 1:
// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
lpCosSin lpComputeCosSin( float radians )
{
	// remainderf is exact in IEEE arithmetic, so it is the same everywhere (unlike sinf and cosf)
	float x = remainderf( radians, 2.0f * LP_PI );
	float pi2 = LP_PI * LP_PI;

	// cosine needs angle in [-pi/2, pi/2]
	float c;
	if ( x < -0.5f * LP_PI )
	{
		float y = x + LP_PI;
		float y2 = y * y;
		c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else if ( x > 0.5f * LP_PI )
	{
		float y = x - LP_PI;
		float y2 = y * y;
		c = -( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}
	else
	{
		float y2 = x * x;
		c = ( pi2 - 4.0f * y2 ) / ( pi2 + y2 );
	}

	// sine needs angle in [0, pi]
	float s;
	if ( x < 0.0f )
	{
		float y = x + LP_PI;
		s = -16.0f * y * ( LP_PI - y ) / ( 5.0f * pi2 - 4.0f * y * ( LP_PI - y ) );
	}
	else
	{
		s = 16.0f * x * ( LP_PI - x ) / ( 5.0f * pi2 - 4.0f * x * ( LP_PI - x ) );
	}

	float mag = sqrtf( s * s + c * c );
	float invMag = mag > 0.0f ? 1.0f / mag : 0.0f;
	lpCosSin cs = { c * invMag, s * invMag };
	return cs;
}

lpQuat lpMakeQuatFromMatrix( const lpMatrix3* m )
{
	lpVec3 c1 = m->cx;
	lpVec3 c2 = m->cy;
	lpVec3 c3 = m->cz;

	lpQuat q;

	float trace = m->cx.x + m->cy.y + m->cz.z;
	if ( trace >= 0.0f )
	{
		q.v.x = c2.z - c3.y;
		q.v.y = c3.x - c1.z;
		q.v.z = c1.y - c2.x;
		q.s = trace + 1.0f;
	}
	else
	{
		if ( c1.x > c2.y && c1.x > c3.z )
		{
			q.v.x = c1.x - c2.y - c3.z + 1.0f;
			q.v.y = c2.x + c1.y;
			q.v.z = c3.x + c1.z;
			q.s = c2.z - c3.y;
		}
		else if ( c2.y > c3.z )
		{
			q.v.x = c1.y + c2.x;
			q.v.y = c2.y - c3.z - c1.x + 1.0f;
			q.v.z = c3.y + c2.z;
			q.s = c3.x - c1.z;
		}
		else
		{
			q.v.x = c1.z + c3.x;
			q.v.y = c2.z + c3.y;
			q.v.z = c3.z - c1.x - c2.y + 1.0f;
			q.s = c1.y - c2.x;
		}
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return lpNormalizeQuat( q );
}

lpQuat lpComputeQuatBetweenUnitVectors( lpVec3 v1, lpVec3 v2 )
{
	lpQuat out;

	lpVec3 m = lpLerp( v1, v2, 0.5f );
	float tolerance = 100.0f * FLT_EPSILON;
	if ( lpLengthSquared( m ) > tolerance * tolerance )
	{
		out.v = lpCross( v1, m );
		out.s = lpDot( v1, m );
	}
	else
	{
		// Anti-parallel: use a perpendicular vector
		if ( lpAbsFloat( v1.x ) > 0.5f )
		{
			out.v.x = v1.y;
			out.v.y = -v1.x;
			out.v.z = 0.0f;
		}
		else
		{
			out.v.x = 0.0f;
			out.v.y = v1.z;
			out.v.z = -v1.y;
		}

		out.s = 0.0f;
	}

	// The algorithm is simplified and made more accurate by normalizing at the end
	return lpNormalizeQuat( out );
}
