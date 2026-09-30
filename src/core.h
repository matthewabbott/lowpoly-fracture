// SPDX-License-Identifier: MIT
// Internal helpers shared by the lpf core: asserts, growable arrays, deterministic random numbers.

#pragma once

#include "lpf/lpf.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined( NDEBUG ) && !defined( LP_FORCE_ASSERT )
#define LP_ASSERT( c ) ( (void)0 )
#else
#define LP_ASSERT( c ) ( ( c ) ? (void)0 : lpAssertFailed( #c, __FILE__, __LINE__ ) )
#endif

void lpAssertFailed( const char* condition, const char* file, int line );

// Cube root. The C library's cbrtf is not correctly rounded, so libms can differ in its last bit. LPF_PORTABLE_MATH
// (a research option for cross-platform checks, off by default because it changes every hash) swaps in lpCbrt,
// which uses only +, -, * and / (correctly rounded on every IEEE target).
#if defined( LPF_PORTABLE_MATH )
float lpCbrt( float x );
#define lpCbrtf( x ) lpCbrt( x )
#else
#define lpCbrtf( x ) cbrtf( x )
#endif

void* lpAlloc( size_t size );
void* lpRealloc( void* p, size_t size );
void lpFree( void* p );

// Growable array of T: `T* data; int count; int capacity;` embedded in a struct via LP_ARRAY(T) name.
#define LP_ARRAY( T )                                                                                                  \
	struct                                                                                                             \
	{                                                                                                                  \
		T* data;                                                                                                       \
		int count;                                                                                                     \
		int capacity;                                                                                                  \
	}

#define lpArray_Reserve( a, n )                                                                                        \
	do                                                                                                                 \
	{                                                                                                                  \
		if ( ( n ) > ( a ).capacity )                                                                                  \
		{                                                                                                              \
			int lpNewCap_ = ( a ).capacity < 16 ? 16 : ( a ).capacity;                                                 \
			while ( lpNewCap_ < ( n ) )                                                                                \
				lpNewCap_ *= 2;                                                                                        \
			( a ).data = lpRealloc( ( a ).data, (size_t)lpNewCap_ * sizeof( *( a ).data ) );                           \
			( a ).capacity = lpNewCap_;                                                                                \
		}                                                                                                              \
	}                                                                                                                  \
	while ( 0 )

#define lpArray_Push( a, v )                                                                                           \
	do                                                                                                                 \
	{                                                                                                                  \
		lpArray_Reserve( a, ( a ).count + 1 );                                                                         \
		( a ).data[( a ).count++] = ( v );                                                                             \
	}                                                                                                                  \
	while ( 0 )

#define lpArray_Free( a )                                                                                              \
	do                                                                                                                 \
	{                                                                                                                  \
		lpFree( ( a ).data );                                                                                          \
		( a ).data = NULL;                                                                                             \
		( a ).count = 0;                                                                                               \
		( a ).capacity = 0;                                                                                            \
	}                                                                                                                  \
	while ( 0 )

// PCG32 (O'Neill). Integer only, so bit-identical everywhere.
typedef struct lpRandom
{
	uint64_t state;
	uint64_t inc;
} lpRandom;

void lpRandom_Seed( lpRandom* rng, uint64_t seed, uint64_t stream );
uint32_t lpRandom_Next( lpRandom* rng );

// Uniform in [0, 1)
static inline float lpRandom_Float( lpRandom* rng )
{
	return (float)( lpRandom_Next( rng ) >> 8 ) * ( 1.0f / 16777216.0f );
}

// Uniform in [lo, hi)
static inline float lpRandom_Range( lpRandom* rng, float lo, float hi )
{
	return lo + ( hi - lo ) * lpRandom_Float( rng );
}

// Mixes values into a 64-bit hash (splitmix64 finalizer), for seeds.
static inline uint64_t lpMix64( uint64_t x )
{
	x += 0x9E3779B97F4A7C15ull;
	x = ( x ^ ( x >> 30 ) ) * 0xBF58476D1CE4E5B9ull;
	x = ( x ^ ( x >> 27 ) ) * 0x94D049BB133111EBull;
	return x ^ ( x >> 31 );
}

// FNV-1a over raw bytes, for state hashes.
static inline uint64_t lpHashBytes( uint64_t h, const void* data, size_t size )
{
	const uint8_t* p = (const uint8_t*)data;
	for ( size_t i = 0; i < size; ++i )
	{
		h ^= p[i];
		h *= 0x100000001B3ull;
	}
	return h;
}

#define LP_HASH_INIT 0xCBF29CE484222325ull
