// SPDX-License-Identifier: MIT

#include "core.h"

#include <math.h>
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

#if defined( _M_X64 ) || defined( __x86_64__ ) || defined( _M_IX86 ) || defined( __i386__ )
#include <xmmintrin.h>

// MXCSR bits that change results: flush-to-zero (15), rounding control (13-14), denormals-are-zero (6)
#define LP_MXCSR_BAD 0xE040u

bool lpFpGuard( void )
{
	unsigned int csr = _mm_getcsr();
	if ( ( csr & LP_MXCSR_BAD ) == 0 )
	{
		return false;
	}
	_mm_setcsr( csr & ~LP_MXCSR_BAD );
	return true;
}

void lpFpBreakForTest( void )
{
	_mm_setcsr( _mm_getcsr() | 0x8040u );
}
#elif defined( _M_ARM64 ) || defined( __aarch64__ )
// FPCR bits that change results: FZ (24), rounding mode (22-23), FZ16 (19), AH (1), FIZ (0)
#define LP_FPCR_BAD 0x01D80003ull
#if defined( _MSC_VER ) && !defined( __clang__ )
#include <intrin.h>
#ifndef ARM64_FPCR
#define ARM64_FPCR ARM64_SYSREG( 3, 3, 4, 4, 0 )
#endif
static uint64_t lpGetFpcr( void )
{
	return (uint64_t)_ReadStatusReg( ARM64_FPCR );
}
static void lpSetFpcr( uint64_t value )
{
	_WriteStatusReg( ARM64_FPCR, (__int64)value );
}
#else
static uint64_t lpGetFpcr( void )
{
	uint64_t value;
	__asm__ volatile( "mrs %0, fpcr" : "=r"( value ) );
	return value;
}
static void lpSetFpcr( uint64_t value )
{
	__asm__ volatile( "msr fpcr, %0" : : "r"( value ) );
}
#endif

bool lpFpGuard( void )
{
	uint64_t fpcr = lpGetFpcr();
	if ( ( fpcr & LP_FPCR_BAD ) == 0 )
	{
		return false;
	}
	lpSetFpcr( fpcr & ~LP_FPCR_BAD );
	return true;
}

void lpFpBreakForTest( void )
{
	lpSetFpcr( lpGetFpcr() | ( 1ull << 24 ) );
}
#else
// Other targets (WebAssembly) have no control word to tamper with
bool lpFpGuard( void )
{
	return false;
}

void lpFpBreakForTest( void )
{
}
#endif

// ---- determinism self-test ----

// Inputs read through volatile so the compiler cannot fold the operations at compile time
static volatile float lp_selfIn[] = { 1.000244140625f,		// 1 + 2^-12
									  -1.00048828125f,		// -(1 + 2^-11)
									  5.9604644775390625e-08f, // 2^-24
									  1.1754943508222875e-38f, // 2^-126, the smallest normal
									  2.0f, 3.0f, -0.5f, 0.0f, 0.1f };

static uint32_t lpBits( float x )
{
	uint32_t u;
	memcpy( &u, &x, sizeof( u ) );
	return u;
}

uint64_t lpDeterminismSelfTest( int* failures )
{
	lpFpGuard();
	float a = lp_selfIn[0], c = lp_selfIn[1], e = lp_selfIn[2], tiny = lp_selfIn[3];
	float two = lp_selfIn[4], three = lp_selfIn[5], half = -lp_selfIn[6], zero = lp_selfIn[7], tenth = lp_selfIn[8];
	float one = two - 1.0f;

	// Known answers: each value with the bits it must have on every IEEE target with our build flags
	struct
	{
		float value;
		uint32_t bits;
	} known[] = {
		{ a * a + c, 0x00000000u },						  // contraction: fused it would be 2^-24
		{ one + e, 0x3F800000u },						  // a tie rounds to even (down)
		{ one + 3.0f * e, 0x3F800002u },				  // a tie rounds to even (up)
		{ tiny * half, 0x00400000u },					  // a subnormal result is kept (no flush-to-zero)
		{ ( tiny * half ) * two, 0x00800000u },			  // a subnormal input is kept (no denormals-are-zero)
		{ sqrtf( two ), 0x3FB504F3u },					  // sqrt is correctly rounded
		{ one / three, 0x3EAAAAABu },					  // so is division
		{ tenth, 0x3DCCCCCDu },							  // literals
		{ floorf( -half ), 0xBF800000u },				  // floor
		{ lpMinFloat( zero, -zero ), 0x80000000u },		  // min keeps the second operand on a tie
		{ lpMaxFloat( -zero, zero ), 0x00000000u },		  // and max
		{ (float)(int)( two * three + half ), 0x40C00000u } // float to int truncates
	};

	int failed = 0;
	uint64_t h = 0x5E1F7E57ull;
	for ( size_t i = 0; i < sizeof( known ) / sizeof( known[0] ); ++i )
	{
		uint32_t bits = lpBits( known[i].value );
		failed += bits != known[i].bits ? 1 : 0;
		h = lpMix64( h ^ bits );
	}

	// No known answers, but the same bits everywhere: our trig, arctangent and cube root
	for ( int i = 0; i < 16; ++i )
	{
		float angle = ( (float)i - 7.5f ) * 0.41f * two;
		lpCosSin cs = lpComputeCosSin( angle );
		h = lpMix64( h ^ lpBits( cs.cosine ) );
		h = lpMix64( h ^ lpBits( cs.sine ) );
		h = lpMix64( h ^ lpBits( lpAtan2( cs.sine * three, cs.cosine - half ) ) );
		h = lpMix64( h ^ lpBits( lpCbrt( angle * angle * angle + tenth ) ) );
	}

	if ( failures != NULL )
	{
		*failures = failed;
	}
	return h;
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
