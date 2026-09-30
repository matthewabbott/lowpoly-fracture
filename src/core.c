// SPDX-License-Identifier: MIT

#include "core.h"

#include <stdio.h>

void lpAssertFailed( const char* condition, const char* file, int line )
{
	fprintf( stderr, "LP_ASSERT(%s) failed at %s:%d\n", condition, file, line );
	fflush( stderr );
	abort();
}

void* lpAlloc( size_t size )
{
	void* p = malloc( size );
	LP_ASSERT( p != NULL );
	return p;
}

void* lpRealloc( void* p, size_t size )
{
	void* q = realloc( p, size );
	LP_ASSERT( q != NULL );
	return q;
}

void lpFree( void* p )
{
	free( p );
}

#if defined( LPF_PORTABLE_MATH )
float lpCbrt( float x )
{
	if ( !( x == x ) || x == 0.0f || x - x != 0.0f )
	{
		return x; // NaN, zero, infinity
	}
	float ax = x < 0.0f ? -x : x;
	double scale = 1.0;
	if ( ax < 1.0e-30f )
	{
		ax *= 281474976710656.0f; // 2^48, exact; its root is 2^16
		scale = 1.0 / 65536.0;
	}
	// Seed: a third of the exponent by the bit trick (within a few percent), then Newton in double
	uint32_t u;
	memcpy( &u, &ax, sizeof( u ) );
	u = u / 3u + 709921077u;
	float seed;
	memcpy( &seed, &u, sizeof( seed ) );
	double a = (double)ax;
	double t = (double)seed;
	for ( int i = 0; i < 5; ++i )
	{
		t -= ( t * t * t - a ) / ( 3.0 * t * t );
	}
	float r = (float)( t * scale );
	return x < 0.0f ? -r : r;
}
#endif

void lpRandom_Seed( lpRandom* rng, uint64_t seed, uint64_t stream )
{
	rng->state = 0u;
	rng->inc = ( stream << 1u ) | 1u;
	lpRandom_Next( rng );
	rng->state += seed;
	lpRandom_Next( rng );
}

uint32_t lpRandom_Next( lpRandom* rng )
{
	uint64_t old = rng->state;
	rng->state = old * 6364136223846793005ull + rng->inc;
	uint32_t xorshifted = (uint32_t)( ( ( old >> 18u ) ^ old ) >> 27u );
	uint32_t rot = (uint32_t)( old >> 59u );
	return ( xorshifted >> rot ) | ( xorshifted << ( ( 32u - rot ) & 31u ) );
}
