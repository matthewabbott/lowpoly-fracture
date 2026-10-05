// SPDX-License-Identifier: MIT
// Internal helpers shared by the lpf core: asserts, growable arrays, deterministic random numbers.

#pragma once

#include "lpf/lpf.h"

#include <limits.h>
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

// Cube root from +, -, * and / only (correctly rounded on every IEEE target). The C library's cbrtf is not correctly
// rounded and differs between libms, so simulation and scene code never call it (determinism rule 12).
float lpCbrt( float x );

// The floating-point control word simulation threads run with: round to nearest, no flush-to-zero, no
// denormals-are-zero (determinism rule 15). A library or driver that changed it would change results; lpFpGuard puts
// it back and returns true when it had to. About 2 ns: called at step entry, per task in our workers and Box3D's
// (a patch), and at the API calls that compute in float between steps.
bool lpFpGuard( void );

// Test hook: turn flush-to-zero and denormals-are-zero on for the calling thread, as a misbehaving library would
void lpFpBreakForTest( void );

// Float to int with the value clamped into int's range first: an out-of-range conversion is undefined in C and gives
// different answers on x86 (INT_MIN) and ARM (saturation), and traps in WebAssembly (determinism rule 16). NaN gives 0.
static inline int lpFloatToInt( float x )
{
	if ( x != x )
	{
		return 0;
	}
	if ( x >= 2147483520.0f ) // the largest float below 2^31
	{
		return 2147483520;
	}
	if ( x <= -2147483648.0f )
	{
		return INT_MIN;
	}
	return (int)x;
}

// Sorts 64-bit keys in place by their bits from fromBit up (scratch: as many again), keeping the input order of keys
// equal there: least-significant-digit radix, a byte a pass, passes where every key has the same byte skipped. Linear,
// for the per-step report sorts that qsort made costly.
void lpRadixSort64( uint64_t* keys, uint64_t* scratch, int count, int fromBit );

// The next float above x (x >= 0). Cast callbacks clip the cast here rather than at their best fraction, so a hit
// at exactly the same fraction is still reported and the tie is broken by piece index, not by the physics engine's
// traversal order.
static inline float lpNextUp( float x )
{
	uint32_t u;
	memcpy( &u, &x, sizeof( u ) );
	u += 1u;
	memcpy( &x, &u, sizeof( x ) );
	return x;
}

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

// Mixes raw bytes into a 64-bit hash eight at a time: much cheaper than lpHashBytes, for state hashes whose result is
// mixed again (lpMix64) before it is combined. Feed it fields or packed arrays, never structs with padding (rule 11).
static inline uint64_t lpHashWords( uint64_t h, const void* data, size_t size )
{
	const uint8_t* p = (const uint8_t*)data;
	size_t i = 0;
	for ( ; i + 8 <= size; i += 8 )
	{
		uint64_t word;
		memcpy( &word, p + i, 8 );
		h = ( h ^ word ) * 0x100000001B3ull;
		h ^= h >> 29;
	}
	if ( i < size )
	{
		uint64_t word = 0;
		memcpy( &word, p + i, size - i );
		h = ( h ^ word ) * 0x100000001B3ull;
		h ^= h >> 29;
	}
	return h;
}

// A float's bits for a state hash, by value: -0 as +0. Box3D's SIMD contact solver left zeros of either sign
// depending on the CPU (SSE's min and max return their second operand when both are zero, NEON's ordered -0 below +0)
// until simd.h took SSE's rule on every path; there a zero's sign only reaches the signs of other zeros, never the
// motion, so a difference in it is not a desync (determinism rule 11).
static inline uint32_t lpHashFloatBits( float x )
{
	uint32_t u;
	memcpy( &u, &x, 4 );
	return u == 0x80000000u ? 0u : u;
}

// lpHashWords over a float-only field or array (a vector, a transform; at most 16 floats), by value (lpHashFloatBits)
static inline uint64_t lpHashFloats( uint64_t h, const void* data, size_t size )
{
	uint32_t bits[16];
	LP_ASSERT( size <= sizeof( bits ) && size % 4 == 0 );
	memcpy( bits, data, size );
	for ( size_t i = 0; i < size / 4; ++i )
	{
		bits[i] = bits[i] == 0x80000000u ? 0u : bits[i];
	}
	return lpHashWords( h, bits, size );
}
