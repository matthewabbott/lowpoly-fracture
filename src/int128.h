// SPDX-License-Identifier: MIT
// Signed 128-bit integers for exact geometry (geom.h): two's complement in two 64-bit words, the same bits on every
// platform, so values hash and serialise alike. Add, sub, neg and the left shift wrap modulo 2^128; the products give
// the exact result whenever it fits, which the caller guarantees (geom.h's bit budget), and wrap otherwise, never
// trapping. No signed overflow can happen in any path: the arithmetic is on unsigned words.
//
// One path is compiled as lpI128_*: __int128 on gcc and clang; _umul128 and _mul128 on MSVC x64 (clang-cl too: its
// __int128 products could need compiler-rt helpers, which MSVC's link has not); __umulh and __mulh on MSVC ARM64; and
// otherwise, or for the whole build when LP_INT128_PORTABLE is defined (CMake: -DLPF_INT128_PORTABLE=ON), the portable
// reference on 32-bit limbs. The reference (lpI128_*Portable) is always compiled and callable on its own, so tests can
// check the native path against it and time both.

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct lpI128
{
	uint64_t lo;
	uint64_t hi; // the top word, two's complement: the sign is its top bit
} lpI128;

static inline lpI128 lpI128_FromI64( int64_t a )
{
	lpI128 r = { (uint64_t)a, a < 0 ? ~0ull : 0ull };
	return r;
}

static inline bool lpI128_IsZero( lpI128 a )
{
	return ( a.lo | a.hi ) == 0;
}

static inline bool lpI128_Equal( lpI128 a, lpI128 b )
{
	return a.lo == b.lo && a.hi == b.hi;
}

// ---- the portable reference: 32-bit limbs, least significant first ----

static inline void lpI128_ToLimbs( lpI128 a, uint32_t limbs[4] )
{
	limbs[0] = (uint32_t)a.lo;
	limbs[1] = (uint32_t)( a.lo >> 32 );
	limbs[2] = (uint32_t)a.hi;
	limbs[3] = (uint32_t)( a.hi >> 32 );
}

static inline lpI128 lpI128_FromLimbs( const uint32_t limbs[4] )
{
	lpI128 r = { (uint64_t)limbs[0] | (uint64_t)limbs[1] << 32, (uint64_t)limbs[2] | (uint64_t)limbs[3] << 32 };
	return r;
}

static inline lpI128 lpI128_AddPortable( lpI128 a, lpI128 b )
{
	uint32_t x[4], y[4], r[4];
	lpI128_ToLimbs( a, x );
	lpI128_ToLimbs( b, y );
	uint64_t carry = 0;
	for ( int i = 0; i < 4; ++i )
	{
		uint64_t t = (uint64_t)x[i] + y[i] + carry;
		r[i] = (uint32_t)t;
		carry = t >> 32;
	}
	return lpI128_FromLimbs( r );
}

static inline lpI128 lpI128_SubPortable( lpI128 a, lpI128 b )
{
	uint32_t x[4], y[4], r[4];
	lpI128_ToLimbs( a, x );
	lpI128_ToLimbs( b, y );
	uint64_t carry = 1; // a + ~b + 1
	for ( int i = 0; i < 4; ++i )
	{
		uint64_t t = (uint64_t)x[i] + (uint32_t)~y[i] + carry;
		r[i] = (uint32_t)t;
		carry = t >> 32;
	}
	return lpI128_FromLimbs( r );
}

static inline lpI128 lpI128_NegPortable( lpI128 a )
{
	return lpI128_SubPortable( lpI128_FromI64( 0 ), a );
}

// a * b modulo 2^128: schoolbook on limbs, each step (2^32 - 1)^2 + 2 (2^32 - 1) = 2^64 - 1 at most
static inline lpI128 lpI128_MulWidePortable( lpI128 a, lpI128 b )
{
	uint32_t x[4], y[4], r[4] = { 0, 0, 0, 0 };
	lpI128_ToLimbs( a, x );
	lpI128_ToLimbs( b, y );
	for ( int i = 0; i < 4; ++i )
	{
		uint64_t carry = 0;
		for ( int j = 0; i + j < 4; ++j )
		{
			uint64_t t = (uint64_t)x[i] * y[j] + r[i + j] + carry;
			r[i + j] = (uint32_t)t;
			carry = t >> 32;
		}
	}
	return lpI128_FromLimbs( r );
}

// int64 x int64, exact (|a b| <= 2^126)
static inline lpI128 lpI128_Mul64Portable( int64_t a, int64_t b )
{
	return lpI128_MulWidePortable( lpI128_FromI64( a ), lpI128_FromI64( b ) );
}

// int128 x int64, exact when the product fits
static inline lpI128 lpI128_MulPortable( lpI128 a, int64_t b )
{
	return lpI128_MulWidePortable( a, lpI128_FromI64( b ) );
}

// -1, 0 or 1 as a < b, a == b, a > b (signed)
static inline int lpI128_ComparePortable( lpI128 a, lpI128 b )
{
	uint32_t x[4], y[4];
	lpI128_ToLimbs( a, x );
	lpI128_ToLimbs( b, y );
	x[3] ^= 0x80000000u; // the top limb compares signed: offset it
	y[3] ^= 0x80000000u;
	for ( int i = 3; i >= 0; --i )
	{
		if ( x[i] != y[i] )
		{
			return x[i] < y[i] ? -1 : 1;
		}
	}
	return 0;
}

static inline int lpI128_SignPortable( lpI128 a )
{
	uint32_t x[4];
	lpI128_ToLimbs( a, x );
	if ( x[3] >> 31 )
	{
		return -1;
	}
	return ( x[0] | x[1] | x[2] | x[3] ) != 0 ? 1 : 0;
}

// a * 2^k modulo 2^128, 0 <= k < 128: limbs moved up by k / 32, bits by k % 32
static inline lpI128 lpI128_ShlPortable( lpI128 a, int k )
{
	uint32_t x[4], r[4] = { 0, 0, 0, 0 };
	lpI128_ToLimbs( a, x );
	int words = k / 32;
	int bits = k % 32;
	for ( int i = words; i < 4; ++i )
	{
		uint32_t low = x[i - words];
		uint32_t carried = ( bits > 0 && i - words - 1 >= 0 ) ? x[i - words - 1] >> ( 32 - bits ) : 0u;
		r[i] = ( bits > 0 ? low << bits : low ) | carried;
	}
	return lpI128_FromLimbs( r );
}

// a as a double, the same bits on every path (one formula on the two words, nothing path-specific): |a| as
// hi 2^64 + lo with at most three roundings (hi's past 2^53, lo's, the sum's), each within 2^-53 of what it rounds, so
// the relative error is below 2^-52 + 2^-106 < 2^-51 for every a. For estimates only: exact geometry never decides on
// it (geom.h's rounding corrects it exactly).
static inline double lpI128_ToDouble( lpI128 a )
{
	bool negative = ( a.hi >> 63 ) != 0;
	uint64_t lo = negative ? 0u - a.lo : a.lo;
	uint64_t hi = negative ? 0u - a.hi - ( a.lo != 0u ? 1u : 0u ) : a.hi;
	double r = (double)hi * 18446744073709551616.0 + (double)lo;
	return negative ? -r : r;
}

// ---- the native path ----

#if defined( LP_INT128_PORTABLE )
#define LP_INT128_PATH "portable"
#elif defined( _MSC_VER ) && ( defined( _M_X64 ) || defined( _M_AMD64 ) ) && !defined( _M_ARM64EC )
#define LP_INT128_PATH "msvc-x64"
#define LP_INT128_MSVC_X64 1
#elif defined( _MSC_VER ) && ( defined( _M_ARM64 ) || defined( _M_ARM64EC ) )
#define LP_INT128_PATH "msvc-arm64"
#define LP_INT128_MSVC_ARM64 1
#elif !defined( _MSC_VER ) && defined( __SIZEOF_INT128__ )
#define LP_INT128_PATH "int128"
#define LP_INT128_NATIVE 1
#else
#define LP_INT128_PATH "portable"
#endif

// The path lpI128_* takes in this build: "int128", "msvc-x64", "msvc-arm64" or "portable"
static inline const char* lpI128_Path( void )
{
	return LP_INT128_PATH;
}

#if defined( LP_INT128_NATIVE )

__extension__ typedef unsigned __int128 lpU128Native;
__extension__ typedef __int128 lpI128Native;

static inline lpU128Native lpI128_ToNative( lpI128 a )
{
	return (lpU128Native)a.hi << 64 | a.lo;
}

static inline lpI128 lpI128_FromNative( lpU128Native x )
{
	lpI128 r = { (uint64_t)x, (uint64_t)( x >> 64 ) };
	return r;
}

static inline lpI128 lpI128_Add( lpI128 a, lpI128 b )
{
	return lpI128_FromNative( lpI128_ToNative( a ) + lpI128_ToNative( b ) );
}

static inline lpI128 lpI128_Sub( lpI128 a, lpI128 b )
{
	return lpI128_FromNative( lpI128_ToNative( a ) - lpI128_ToNative( b ) );
}

static inline lpI128 lpI128_Neg( lpI128 a )
{
	return lpI128_FromNative( (lpU128Native)0 - lpI128_ToNative( a ) );
}

static inline lpI128 lpI128_Mul64( int64_t a, int64_t b )
{
	return lpI128_FromNative( (lpU128Native)( (lpI128Native)a * (lpI128Native)b ) );
}

static inline lpI128 lpI128_Mul( lpI128 a, int64_t b )
{
	return lpI128_FromNative( lpI128_ToNative( a ) * (lpU128Native)(lpI128Native)b );
}

static inline int lpI128_Compare( lpI128 a, lpI128 b )
{
	lpI128Native x = (lpI128Native)lpI128_ToNative( a );
	lpI128Native y = (lpI128Native)lpI128_ToNative( b );
	return x < y ? -1 : ( x > y ? 1 : 0 );
}

static inline int lpI128_Sign( lpI128 a )
{
	lpI128Native x = (lpI128Native)lpI128_ToNative( a );
	return x < 0 ? -1 : ( x > 0 ? 1 : 0 );
}

static inline lpI128 lpI128_Shl( lpI128 a, int k )
{
	return lpI128_FromNative( lpI128_ToNative( a ) << k );
}

#elif defined( LP_INT128_MSVC_X64 ) || defined( LP_INT128_MSVC_ARM64 )

#include <intrin.h>

static inline lpI128 lpI128_Add( lpI128 a, lpI128 b )
{
	lpI128 r;
	r.lo = a.lo + b.lo;
	r.hi = a.hi + b.hi + ( r.lo < a.lo ? 1u : 0u );
	return r;
}

static inline lpI128 lpI128_Sub( lpI128 a, lpI128 b )
{
	lpI128 r;
	r.lo = a.lo - b.lo;
	r.hi = a.hi - b.hi - ( a.lo < b.lo ? 1u : 0u );
	return r;
}

static inline lpI128 lpI128_Neg( lpI128 a )
{
	lpI128 r;
	r.lo = 0u - a.lo;
	r.hi = 0u - a.hi - ( a.lo != 0u ? 1u : 0u );
	return r;
}

static inline lpI128 lpI128_Mul64( int64_t a, int64_t b )
{
	lpI128 r;
#if defined( LP_INT128_MSVC_X64 )
	__int64 high;
	r.lo = (uint64_t)_mul128( a, b, &high );
	r.hi = (uint64_t)high;
#else
	r.lo = (uint64_t)a * (uint64_t)b;
	r.hi = (uint64_t)__mulh( a, b );
#endif
	return r;
}

// a = hi 2^64 + lo (both as unsigned words), b sign-extended: modulo 2^128, a b = lo b + (hi b) 2^64 - [b < 0] lo 2^64
static inline lpI128 lpI128_Mul( lpI128 a, int64_t b )
{
	uint64_t ub = (uint64_t)b;
	lpI128 r;
#if defined( LP_INT128_MSVC_X64 )
	unsigned __int64 high;
	r.lo = _umul128( a.lo, ub, &high );
#else
	uint64_t high = __umulh( a.lo, ub );
	r.lo = a.lo * ub;
#endif
	r.hi = (uint64_t)high + a.hi * ub - ( b < 0 ? a.lo : 0u );
	return r;
}

static inline int lpI128_Compare( lpI128 a, lpI128 b )
{
	if ( a.hi != b.hi )
	{
		return ( a.hi ^ 0x8000000000000000ull ) < ( b.hi ^ 0x8000000000000000ull ) ? -1 : 1;
	}
	return a.lo < b.lo ? -1 : ( a.lo > b.lo ? 1 : 0 );
}

static inline int lpI128_Sign( lpI128 a )
{
	if ( a.hi >> 63 )
	{
		return -1;
	}
	return ( a.lo | a.hi ) != 0 ? 1 : 0;
}

// 0 <= k < 128; the word shifts stay below 64 (a shift by 64 is undefined in C)
static inline lpI128 lpI128_Shl( lpI128 a, int k )
{
	lpI128 r;
	if ( k >= 64 )
	{
		r.hi = a.lo << ( k - 64 );
		r.lo = 0;
	}
	else if ( k > 0 )
	{
		r.hi = a.hi << k | a.lo >> ( 64 - k );
		r.lo = a.lo << k;
	}
	else
	{
		r = a;
	}
	return r;
}

#else // the portable reference for the whole build

static inline lpI128 lpI128_Add( lpI128 a, lpI128 b )
{
	return lpI128_AddPortable( a, b );
}

static inline lpI128 lpI128_Sub( lpI128 a, lpI128 b )
{
	return lpI128_SubPortable( a, b );
}

static inline lpI128 lpI128_Neg( lpI128 a )
{
	return lpI128_NegPortable( a );
}

static inline lpI128 lpI128_Mul64( int64_t a, int64_t b )
{
	return lpI128_Mul64Portable( a, b );
}

static inline lpI128 lpI128_Mul( lpI128 a, int64_t b )
{
	return lpI128_MulPortable( a, b );
}

static inline int lpI128_Compare( lpI128 a, lpI128 b )
{
	return lpI128_ComparePortable( a, b );
}

static inline int lpI128_Sign( lpI128 a )
{
	return lpI128_SignPortable( a );
}

static inline lpI128 lpI128_Shl( lpI128 a, int k )
{
	return lpI128_ShlPortable( a, k );
}

#endif
