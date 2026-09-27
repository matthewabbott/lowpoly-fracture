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
