// SPDX-License-Identifier: MIT
// Cross-platform probe of the C-library functions the engine calls: writes each function's results over a fixed
// sweep of inputs (binary, 4 bytes each) so the summary job can count the inputs where two platforms disagree, and
// counts where each result differs from the double-precision function rounded to float (a near-correctly-rounded
// reference).
//   libm_probe <outdir>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static float F( uint32_t u )
{
	float f;
	memcpy( &f, &u, 4 );
	return f;
}

static uint32_t U( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	return u;
}

static uint64_t Fnv( uint64_t h, uint32_t v )
{
	for ( int i = 0; i < 4; ++i )
	{
		h ^= ( v >> ( 8 * i ) ) & 0xFFu;
		h *= 0x100000001B3ull;
	}
	return h;
}

static FILE* Open( const char* dir, const char* name )
{
	char path[1024];
	snprintf( path, sizeof( path ), "%s/%s.bin", dir, name );
	return fopen( path, "wb" );
}

typedef float ( *Fn1 )( float );
typedef double ( *Ref1 )( double );

static float Cbrt( float x ) { return cbrtf( x ); }
static float Sqrt( float x ) { return sqrtf( x ); }
static float Sin( float x ) { return sinf( x ); }
static float Cos( float x ) { return cosf( x ); }

// Every `step`-th float bit pattern in [lo, hi), positive (and negative when `both`)
static void Sweep1( const char* dir, const char* name, Fn1 fn, Ref1 ref, uint32_t lo, uint32_t hi, uint32_t step, int both )
{
	FILE* f = Open( dir, name );
	uint64_t h = 0xCBF29CE484222325ull;
	long count = 0, notRounded = 0;
	for ( uint32_t u = lo; u < hi; u += step )
	{
		for ( int s = 0; s <= both; ++s )
		{
			float x = F( u | ( s ? 0x80000000u : 0u ) );
			uint32_t y = U( fn( x ) );
			uint32_t r = U( (float)ref( (double)x ) );
			notRounded += y != r;
			h = Fnv( h, y );
			if ( f )
			{
				fwrite( &y, 4, 1, f );
			}
			++count;
		}
	}
	if ( f )
	{
		fclose( f );
	}
	printf( "%-7s %8ld values, hash %016llx, %6ld differ from the double function rounded to float\n", name, count,
			(unsigned long long)h, notRounded );
}

int main( int argc, char** argv )
{
	const char* dir = argc > 1 ? argv[1] : ".";
	// cbrtf and sqrtf over 2^-30 .. 2^30 (volumes, energies, counts); sinf and cosf over 2^-12 .. 64
	Sweep1( dir, "cbrtf", Cbrt, cbrt, 0x30800000u, 0x4E800000u, 1024u, 0 );
	Sweep1( dir, "sqrtf", Sqrt, sqrt, 0x30800000u, 0x4E800000u, 1024u, 0 );
	Sweep1( dir, "sinf", Sin, sin, 0x39800000u, 0x42800000u, 512u, 1 );
	Sweep1( dir, "cosf", Cos, cos, 0x39800000u, 0x42800000u, 512u, 1 );

	// atan2f over pseudo-random pairs in [-60, 60]^2 (PCG32, fixed seed)
	FILE* f = Open( dir, "atan2f" );
	uint64_t state = 0x853C49E6748FEA9Bull, h = 0xCBF29CE484222325ull;
	long notRounded = 0, count = 1 << 18;
	for ( long i = 0; i < count; ++i )
	{
		float v[2];
		for ( int k = 0; k < 2; ++k )
		{
			uint64_t old = state;
			state = old * 6364136223846793005ull + 1442695040888963407ull;
			uint32_t xs = (uint32_t)( ( ( old >> 18u ) ^ old ) >> 27u );
			uint32_t rot = (uint32_t)( old >> 59u );
			uint32_t r = ( xs >> rot ) | ( xs << ( ( 0u - rot ) & 31u ) );
			v[k] = ( (float)( r >> 8 ) * ( 1.0f / 16777216.0f ) - 0.5f ) * 120.0f;
		}
		uint32_t y = U( atan2f( v[0], v[1] ) );
		notRounded += y != U( (float)atan2( (double)v[0], (double)v[1] ) );
		h = Fnv( h, y );
		if ( f )
		{
			fwrite( &y, 4, 1, f );
		}
	}
	if ( f )
	{
		fclose( f );
	}
	printf( "%-7s %8ld values, hash %016llx, %6ld differ from the double function rounded to float\n", "atan2f", count,
			(unsigned long long)h, notRounded );
	return 0;
}
