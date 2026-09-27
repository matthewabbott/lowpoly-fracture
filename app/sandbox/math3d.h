// SPDX-License-Identifier: MIT
// Minimal matrix helpers for the sandbox. Column-major (m[col * 4 + row]), v' = M v, D3D clip space (z in [0, 1]).
#pragma once

#include <math.h>

struct lpMat4
{
	float m[16];
};

struct V3
{
	float x, y, z;
};

inline V3 operator+( V3 a, V3 b )
{
	return { a.x + b.x, a.y + b.y, a.z + b.z };
}
inline V3 operator-( V3 a, V3 b )
{
	return { a.x - b.x, a.y - b.y, a.z - b.z };
}
inline V3 operator*( float s, V3 a )
{
	return { s * a.x, s * a.y, s * a.z };
}
inline float Dot( V3 a, V3 b )
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline V3 Cross( V3 a, V3 b )
{
	return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}
inline V3 Normalize( V3 a )
{
	float l = sqrtf( Dot( a, a ) );
	return l > 0.0f ? ( 1.0f / l ) * a : a;
}

inline lpMat4 Mul( const lpMat4& a, const lpMat4& b )
{
	lpMat4 r;
	for ( int c = 0; c < 4; ++c )
	{
		for ( int row = 0; row < 4; ++row )
		{
			float s = 0.0f;
			for ( int k = 0; k < 4; ++k )
			{
				s += a.m[k * 4 + row] * b.m[c * 4 + k];
			}
			r.m[c * 4 + row] = s;
		}
	}
	return r;
}

inline lpMat4 LookAt( V3 eye, V3 target, V3 up )
{
	V3 f = Normalize( target - eye );
	V3 s = Normalize( Cross( f, up ) );
	V3 u = Cross( s, f );
	lpMat4 r = {};
	r.m[0] = s.x;
	r.m[4] = s.y;
	r.m[8] = s.z;
	r.m[1] = u.x;
	r.m[5] = u.y;
	r.m[9] = u.z;
	r.m[2] = -f.x;
	r.m[6] = -f.y;
	r.m[10] = -f.z;
	r.m[12] = -Dot( s, eye );
	r.m[13] = -Dot( u, eye );
	r.m[14] = Dot( f, eye );
	r.m[15] = 1.0f;
	return r;
}

// Right-handed, depth 0..1
inline lpMat4 Perspective( float fovY, float aspect, float zNear, float zFar )
{
	float f = 1.0f / tanf( 0.5f * fovY );
	lpMat4 r = {};
	r.m[0] = f / aspect;
	r.m[5] = f;
	r.m[10] = zFar / ( zNear - zFar );
	r.m[11] = -1.0f;
	r.m[14] = zNear * zFar / ( zNear - zFar );
	return r;
}

inline lpMat4 Ortho( float l, float r_, float b, float t, float n, float f )
{
	lpMat4 r = {};
	r.m[0] = 2.0f / ( r_ - l );
	r.m[5] = 2.0f / ( t - b );
	r.m[10] = 1.0f / ( n - f );
	r.m[12] = -( r_ + l ) / ( r_ - l );
	r.m[13] = -( t + b ) / ( t - b );
	r.m[14] = n / ( n - f );
	r.m[15] = 1.0f;
	return r;
}
