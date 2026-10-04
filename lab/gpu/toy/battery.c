// battery.c: the toy's helper battery (DESIGN.md "Comparing bits", step 1's gate). Every num.slang helper runs on a
// fixed-seed set of input vectors (edge cases first: zeros of both signs, negative values through rsh, saturation,
// values around 2^-30, large quotients, divisors near 1 and near the format limits) on the CPU twin and, for F and V4,
// on every Vulkan GPU; the outputs are compared word by word and hashed per helper. V4's lpDivQ31 and divClamp are
// also checked against C's integer division on every input. Built once per dialect (DIALECT_F, _V4, _D; D has no GPU
// kernels). F's SPIR-V is F-plain: no float-control execution modes.
//
//   battery_F [--gpus MASK] [--n N] [--seed S] [--log PATH]      (run from lab/gpu: kernels in gen/toy/<dialect>)
#include "fpflags.h"
#include "layout.h"
#include "rng.h"
#include "vk_util.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined( DIALECT_F )
#define DIALECT_NAME "F"
#define HAS_GPU 1
#elif defined( DIALECT_V4 )
#define DIALECT_NAME "V4"
#define HAS_GPU 1
#else
#define DIALECT_NAME "D"
#define HAS_GPU 0
#endif

void bat_bind( void* in, void* out, size_t n, uint32_t* counters );
void bat_run( uint32_t start, uint32_t count, uint32_t helper, uint32_t g0, uint32_t g1 );
size_t bat_sizeof( int which );
const char* bat_info( void );

enum
{
	H_MUL,
	H_SUM2,
	H_SUM3,
	H_RESCALE,
	H_SELECT,
	H_SNAP,
	H_RECIP,
	H_RSQRT,
	H_DIVCLAMP,
	H_DIVQ31,
	H_CLZ,
	H_ATAN2,
	H_QNORM,
	H_QMUL,
	H_ROTATE,
	H_POS,
	H_UNIT,
	H_DELTA,
	H_FRAME,
	H_COUNT
};
static const char* g_names[H_COUNT] = { "mul",	  "sum2/dif2", "sum3",	   "rescale",	   "select",  "snap",	"recip",
										"rsqrt",  "divClamp",  "lpDivQ31", "clz",		   "atan2",	  "quatNormalize",
										"quatMul", "rotate",   "pos/grid", "unit3/len3", "posDelta", "frame" };

static FILE* g_log;

static void say( const char* fmt, ... )
{
	va_list a;
	va_start( a, fmt );
	vprintf( fmt, a );
	va_end( a );
	if ( g_log )
	{
		va_start( a, fmt );
		vfprintf( g_log, fmt, a );
		va_end( a );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------------------------------------------------

// F and D: ordinary values (snapped domain plus values just under 2^-30, both zero signs)
static double fd_value( Pcg* r )
{
	uint32_t c = pcg_next( r ) % 10u;
	switch ( c )
	{
		case 0:
			return 0.0;
		case 1:
			return -0.0;
		case 2:
		{
			double m = pcg_range( r, 0.5, 2.0 );
			uint32_t s = pcg_next( r ) & 1u;
			return ldexp( s ? -m : m, -30 );
		}
		case 3:
		{
			int k = pcg_int( r, -32, 32 );
			return 0.5 * k;
		}
		case 7:
			return pcg_log( r, -30, 0 );
		case 8:
			return pcg_log( r, 10, 40 );
		case 9:
			return pcg_log( r, -8, 8 );
		default:
			return pcg_log( r, -20, 20 );
	}
}

// F and D: around the snap limit, subnormal floats, the smallest normal
static double snap_value( Pcg* r )
{
	uint32_t c = pcg_next( r ) % 12u;
	uint32_t s = pcg_next( r ) & 1u;
	double v;
	switch ( c )
	{
		case 0:
			return 0.0;
		case 1:
			return -0.0;
		case 2:
			v = ldexp( 1.0, -30 );
			break;
		case 3:
			v = (double)nextafterf( (float)ldexp( 1.0, -30 ), 0.0f );
			break;
		case 4:
			v = (double)nextafterf( (float)ldexp( 1.0, -30 ), 1.0f );
			break;
		case 5:
		{
			int k = pcg_int( r, 1, 8388607 );
			v = ldexp( (double)k, -149 ); // a subnormal float
			break;
		}
		case 6:
			v = ldexp( 1.0, -126 );
			break;
		case 7:
		{
			double m = pcg_range( r, 1.0, 2.0 );
			int e = pcg_int( r, -40, -20 );
			v = ldexp( m, e );
			break;
		}
		case 8:
		{
			double m = pcg_range( r, 1.0, 2.0 );
			int e = pcg_int( r, -32, -28 );
			v = ldexp( m, e );
			break;
		}
		default:
			return fd_value( r );
	}
	return s ? -v : v;
}

// V4: zero, +-1, +-(2^31 - 1), powers of two and their neighbours, small, medium and full-range values
static int32_t q_value( Pcg* r )
{
	uint32_t c = pcg_next( r ) % 10u;
	uint32_t s = pcg_next( r ) & 1u;
	int32_t v;
	switch ( c )
	{
		case 0:
			return 0;
		case 1:
			v = 1;
			break;
		case 2:
			v = 2147483647;
			break;
		case 3:
		{
			int k = pcg_int( r, 0, 30 );
			v = (int32_t)( 1u << k );
			break;
		}
		case 4:
		{
			int k = pcg_int( r, 1, 30 );
			uint32_t t = pcg_next( r ) & 1u;
			v = (int32_t)( 1u << k ) + ( t ? 1 : -1 );
			break;
		}
		case 5:
			v = (int32_t)( pcg_next( r ) >> 16 );
			break;
		case 6:
			v = (int32_t)( pcg_next( r ) >> 8 );
			break;
		default:
			v = (int32_t)( pcg_next( r ) >> 1 );
			break;
	}
	return s ? -v : v;
}

static int32_t q_bounded( Pcg* r, int32_t maxMag )
{
	int32_t v = q_value( r );
	return ( v > maxMag || v < -maxMag ) ? v % maxMag : v;
}

// a double in Q-format s, rounded half up, saturated to +-(2^31 - 1)
static int32_t q_of( double x, int s )
{
	double y = floor( ldexp( x, s ) + 0.5 );
	return y > 2147483647.0 ? 2147483647 : y < -2147483647.0 ? -2147483647 : (int32_t)y;
}

static int32_t bits_value( Pcg* r )
{
	uint32_t c = pcg_next( r ) % 6u;
	switch ( c )
	{
		case 0:
			return 0;
		case 1:
			return 1;
		case 2:
			return -1;
		case 3:
		{
			int k = pcg_int( r, 0, 31 );
			return (int32_t)( 1u << k );
		}
		case 4:
		{
			int k = pcg_int( r, 0, 31 );
			uint32_t u = pcg_next( r );
			return (int32_t)( u >> k );
		}
		default:
			return (int32_t)pcg_next( r );
	}
}

#if defined( DIALECT_V4 )
#define TV( x, s ) q_of( ( x ), ( s ) )
#else
#define TV( x, s ) ( (T)( x ) )
#endif

static void quat_input( Pcg* r, double q[4] )
{
	pcg_unit_quat( r, q );
	uint32_t c = pcg_next( r ) % 4u;
	double len = 1.0;
	if ( c == 1 )
	{
		len = 1.0 + pcg_range( r, -1e-3, 1e-3 );
	}
	else if ( c == 2 )
	{
		len = pcg_range( r, 0.6, 1.4 ); // V4: 1/|q| < 2 fits Q1.30
	}
	else if ( c == 3 )
	{
		len = 0.0;
	}
	for ( int i = 0; i < 4; ++i )
	{
		q[i] *= len;
	}
}

static void gen_vec( int h, BatIn* v, Pcg* r )
{
	memset( v, 0, sizeof( *v ) );
	double q[4];
	switch ( h )
	{
#if defined( DIALECT_V4 )
		case H_MUL:
			v->a = q_value( r );
			v->b = q_value( r );
			v->sh = pcg_int( r, 0, 62 );
			break;
		case H_SUM2: // each product below 2^61, the sum below 2^62
			v->a = q_bounded( r, 1518500249 );
			v->b = q_bounded( r, 1518500249 );
			v->c = q_bounded( r, 1518500249 );
			v->d = q_bounded( r, 1518500249 );
			v->sh = pcg_int( r, 0, 62 );
			break;
		case H_SUM3:
			v->a = q_bounded( r, 1073741824 );
			v->b = q_bounded( r, 1073741824 );
			v->c = q_bounded( r, 1073741824 );
			v->d = q_bounded( r, 1073741824 );
			v->e = q_bounded( r, 1073741824 );
			v->f = q_bounded( r, 1073741824 );
			v->sh = pcg_int( r, 0, 62 );
			break;
		case H_RESCALE:
			v->a = q_value( r );
			v->b = q_value( r );
			v->sh = pcg_int( r, -8, 40 );
			v->k = pcg_int( r, -30, 62 );
			break;
		case H_SELECT:
		case H_SNAP:
		case H_ATAN2:
		{
			v->a = q_value( r );
			v->b = q_value( r );
			v->c = q_value( r );
			v->d = q_value( r );
			uint32_t c = pcg_next( r ) % 5u;
			v->b = c == 1 ? v->a : c == 2 ? -v->a : v->b;
			v->d = c == 3 ? v->c : c == 4 ? -v->c : v->d;
			break;
		}
		case H_RECIP:
		{
			static const int fmt[4][2] = { { 24, 24 }, { 30, 30 }, { 22, 22 }, { 20, 30 } };
			int f = pcg_int( r, 0, 3 );
			v->a = q_value( r );
			v->sh = fmt[f][0];
			v->k = fmt[f][1];
			break;
		}
		case H_RSQRT:
		{
			static const int su[4] = { 20, 22, 24, 30 };
			static const int sy[3] = { 20, 24, 30 };
			int i = pcg_int( r, 0, 3 );
			int j = pcg_int( r, 0, 2 );
			v->a = q_value( r );
			v->sh = su[i];
			v->k = sy[j];
			break;
		}
		case H_DIVCLAMP:
		{
			v->a = q_value( r );
			v->b = q_value( r );
			v->k = pcg_int( r, 0, 32 );
			int32_t lim = q_value( r );
			v->c = lim < 0 ? -lim : lim == 0 ? 1 : lim;
			break;
		}
		case H_DIVQ31:
		{
			v->a = q_value( r );
			v->b = q_value( r );
			v->k = pcg_int( r, 0, 32 );
			uint32_t near = pcg_next( r ) % 4u;
			if ( near == 0 && v->b != 0 ) // a quotient just under 2^31 - 1, for every divisor size: the estimate's worst case
			{
				uint32_t below = pcg_next( r ) & 0xfffffu;
				int64_t q = 2147483647LL - (int64_t)below;
				int64_t ad = v->b < 0 ? -(int64_t)v->b : (int64_t)v->b;
				int64_t n = (int64_t)( ( (uint64_t)q * (uint64_t)ad ) >> v->k );
				v->a = n > 2147483647LL ? 2147483647 : (int32_t)n;
				v->a = ( v->b & 1 ) ? -v->a : v->a;
			}
			break;
		}
		case H_QNORM:
			quat_input( r, q );
			v->a = q_of( q[0], S_Q );
			v->b = q_of( q[1], S_Q );
			v->c = q_of( q[2], S_Q );
			v->d = q_of( q[3], S_Q );
			break;
		case H_QMUL:
		case H_ROTATE:
		{
			pcg_unit_quat( r, q );
			double e = h == H_QMUL ? pcg_range( r, -0.7, 0.7 ) : pcg_range( r, -30.0, 30.0 );
			double f = h == H_QMUL ? pcg_range( r, -0.7, 0.7 ) : pcg_range( r, -30.0, 30.0 );
			int s = h == H_QMUL ? S_Q : S_R;
			v->a = q_of( q[0], S_Q );
			v->b = q_of( q[1], S_Q );
			v->c = q_of( q[2], S_Q );
			v->d = q_of( q[3], S_Q );
			v->e = q_of( e, s );
			v->f = q_of( f, s );
			break;
		}
		case H_POS:
			v->a = (int32_t)pcg_next( r );
			v->b = pcg_int( r, -( 1 << 20 ), 1 << 20 );
			v->c = q_value( r );
			v->d = q_value( r );
			v->e = q_value( r );
			v->f = q_bounded( r, 1 << 30 );
			break;
		case H_UNIT:
		{
			// any integer vector: zero ones, one component, full range, tiny ones
			uint32_t c = pcg_next( r ) % 6u;
			v->a = q_value( r );
			v->b = q_value( r );
			v->c = q_value( r );
			v->d = q_value( r );
			v->e = q_value( r );
			v->f = q_value( r );
			if ( c == 1 )
			{
				v->b = 0;
				v->c = 0;
			}
			else if ( c == 2 )
			{
				v->a = 0;
				v->b = 0;
				v->c = 0;
			}
			else if ( c == 3 )
			{
				v->a = v->a >> 24;
				v->b = v->b >> 24;
				v->c = v->c >> 24;
			}
			break;
		}
		case H_DELTA:
			v->a = (int32_t)pcg_next( r );
			v->b = (int32_t)pcg_next( r );
			v->c = (int32_t)pcg_next( r );
			v->d = (int32_t)pcg_next( r );
			v->e = (int32_t)pcg_next( r );
			v->f = (int32_t)pcg_next( r );
			break;
		case H_FRAME:
		{
			pcg_unit_quat( r, q );
			double e = pcg_range( r, -20.0, 20.0 );
			double f = pcg_range( r, -20.0, 20.0 );
			v->a = q_of( q[0], S_Q );
			v->b = q_of( q[1], S_Q );
			v->c = q_of( q[2], S_Q );
			v->d = q_of( q[3], S_Q );
			v->e = q_of( e, S_R );
			v->f = q_of( f, S_R );
			break;
		}
#else
		case H_MUL:
		case H_SUM2:
		case H_SUM3:
		case H_RESCALE:
			v->a = (T)fd_value( r );
			v->b = (T)fd_value( r );
			v->c = (T)fd_value( r );
			v->d = (T)fd_value( r );
			v->e = (T)fd_value( r );
			v->f = (T)fd_value( r );
			v->sh = pcg_int( r, 0, 62 );
			v->k = pcg_int( r, -30, 62 );
			break;
		case H_SELECT:
		case H_ATAN2:
		{
			v->a = (T)fd_value( r );
			v->b = (T)fd_value( r );
			v->c = (T)fd_value( r );
			v->d = (T)fd_value( r );
			uint32_t c = pcg_next( r ) % 5u;
			v->b = c == 1 ? v->a : c == 2 ? -v->a : v->b;
			v->d = c == 3 ? v->c : c == 4 ? -v->c : v->d;
			break;
		}
		case H_SNAP:
			v->a = (T)snap_value( r );
			v->b = (T)snap_value( r );
			v->c = (T)snap_value( r );
			v->d = (T)snap_value( r );
			break;
		case H_RECIP:
			v->a = (T)fd_value( r );
			break;
		case H_RSQRT:
		{
			double x = fd_value( r );
			v->a = (T)fabs( x ); // not x < 0 ? -x : x: MSVC folds that into fabs (+0 for -0), clang keeps -0
			break;
		}
		case H_DIVCLAMP:
		{
			v->a = (T)fd_value( r );
			v->b = (T)fd_value( r );
			double lim = pcg_log( r, 0, 20 );
			v->c = (T)fabs( lim );
			break;
		}
		case H_DIVQ31:
			break;
		case H_QNORM:
			quat_input( r, q );
			v->a = (T)q[0];
			v->b = (T)q[1];
			v->c = (T)q[2];
			v->d = (T)q[3];
			break;
		case H_QMUL:
		case H_ROTATE:
		{
			pcg_unit_quat( r, q );
			double e = h == H_QMUL ? pcg_range( r, -0.7, 0.7 ) : pcg_range( r, -30.0, 30.0 );
			double f = h == H_QMUL ? pcg_range( r, -0.7, 0.7 ) : pcg_range( r, -30.0, 30.0 );
			v->a = (T)q[0];
			v->b = (T)q[1];
			v->c = (T)q[2];
			v->d = (T)q[3];
			v->e = (T)e;
			v->f = (T)f;
			break;
		}
		case H_UNIT:
		{
			uint32_t c = pcg_next( r ) % 4u;
			v->a = (T)fd_value( r );
			v->b = (T)fd_value( r );
			v->c = (T)fd_value( r );
			v->d = (T)fd_value( r );
			v->e = (T)fd_value( r );
			v->f = (T)fd_value( r );
			if ( c == 1 )
			{
				v->b = 0;
				v->c = 0;
			}
			else if ( c == 2 )
			{
				v->a = 0;
				v->b = 0;
				v->c = 0;
			}
			break;
		}
		case H_DELTA:
		{
			double a = pcg_range( r, -1e4, 1e4 );
			double b = pcg_range( r, -1e4, 1e4 );
			double c = pcg_range( r, -1e4, 1e4 );
			double d = pcg_range( r, -64.0, 64.0 );
			double e = pcg_range( r, -64.0, 64.0 );
			double f = pcg_range( r, -64.0, 64.0 );
			v->a = (T)a;
			v->b = (T)b;
			v->c = (T)c;
			v->d = (T)d;
			v->e = (T)e;
			v->f = (T)f;
			break;
		}
		case H_FRAME:
		{
			pcg_unit_quat( r, q );
			double e = pcg_range( r, -20.0, 20.0 );
			double f = pcg_range( r, -20.0, 20.0 );
			v->a = (T)q[0];
			v->b = (T)q[1];
			v->c = (T)q[2];
			v->d = (T)q[3];
			v->e = (T)e;
			v->f = (T)f;
			break;
		}
		case H_POS:
		{
			double a = pcg_range( r, -1e4, 1e4 );
			double b = pcg_range( r, -1e4, 1e4 );
			double c = pcg_range( r, -32.0, 32.0 );
			double d = pcg_range( r, -32.0, 32.0 );
			double e = pcg_range( r, -32.0, 32.0 );
			double f = pcg_range( r, 0.0, 64.0 );
			v->a = (T)a;
			v->b = (T)b;
			v->c = (T)c;
			v->d = (T)d;
			v->e = (T)e;
			v->f = (T)f;
			break;
		}
#endif
		case H_CLZ:
			v->k = bits_value( r );
			v->sh = bits_value( r );
			break;
	}
}

typedef struct Battery
{
	int n[H_COUNT];
	int start[H_COUNT + 1];
	int total;
	BatIn* in;
} Battery;

// V4's division sweep: divisors near 1 and near the format limits, against the same numerators, every shift class
static const int32_t g_special[] = { 0, 1, 2, 3, 5, 7, 255, 256, 257, 65535, 65536, 65537, 1000000007, 12345678, 1073741823,
									 1073741824, 1073741825, 2147483646, 2147483647 };
static const int g_specialShifts[] = { 0, 1, 2, 15, 16, 17, 29, 30, 31, 32 };
#define SPECIALS ( (int)( sizeof( g_special ) / sizeof( g_special[0] ) ) )
#define SPECIAL_SHIFTS ( (int)( sizeof( g_specialShifts ) / sizeof( g_specialShifts[0] ) ) )

static void make_battery( Battery* b, int n, uint64_t seed )
{
	for ( int h = 0; h < H_COUNT; ++h )
	{
		b->n[h] = n;
	}
#if defined( DIALECT_V4 )
	int sweep = ( 2 * SPECIALS ) * ( 2 * SPECIALS ) * SPECIAL_SHIFTS;
	b->n[H_DIVQ31] = 16 * n + sweep;
	b->n[H_DIVCLAMP] = 4 * n;
#else
	b->n[H_DIVQ31] = 0;
#endif
	b->total = 0;
	for ( int h = 0; h < H_COUNT; ++h )
	{
		b->start[h] = b->total;
		b->total += b->n[h];
	}
	b->start[H_COUNT] = b->total;
	b->in = (BatIn*)calloc( (size_t)b->total, sizeof( BatIn ) );
	for ( int h = 0; h < H_COUNT; ++h )
	{
		Pcg r = pcg_seed( seed, 100 + (uint64_t)h );
		int i = 0;
#if defined( DIALECT_V4 )
		if ( h == H_DIVQ31 )
		{
			for ( int x = 0; x < 2 * SPECIALS; ++x )
			{
				for ( int y = 0; y < 2 * SPECIALS; ++y )
				{
					for ( int s = 0; s < SPECIAL_SHIFTS; ++s )
					{
						BatIn* v = b->in + b->start[h] + i++;
						memset( v, 0, sizeof( *v ) );
						v->a = x < SPECIALS ? g_special[x] : -g_special[x - SPECIALS];
						v->b = y < SPECIALS ? g_special[y] : -g_special[y - SPECIALS];
						v->k = g_specialShifts[s];
					}
				}
			}
		}
#endif
		for ( ; i < b->n[h]; ++i )
		{
			gen_vec( h, b->in + b->start[h] + i, &r );
		}
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Hashing, comparison, references
// ---------------------------------------------------------------------------------------------------------------------

static uint64_t fnv( uint64_t h, const void* p, size_t n )
{
	const uint8_t* b = (const uint8_t*)p;
	for ( size_t i = 0; i < n; ++i )
	{
		h ^= b[i];
		h *= 1099511628211ULL;
	}
	return h;
}

#define FNV0 1469598103934665603ULL
#define OUT_WORDS ( (int)( sizeof( BatOut ) / 4 ) )

static void dump_vec( const char* what, const BatIn* v, const BatOut* a, const BatOut* b )
{
	const uint32_t* wi = (const uint32_t*)v;
	const uint32_t* wa = (const uint32_t*)a;
	const uint32_t* wb = (const uint32_t*)b;
	say( "      %s in:", what );
	for ( int i = 0; i < (int)( sizeof( BatIn ) / 4 ); ++i )
	{
		say( " %08x", wi[i] );
	}
	say( "\n      twin:" );
	for ( int i = 0; i < OUT_WORDS; ++i )
	{
		say( " %08x", wa[i] );
	}
	say( "\n      gpu: " );
	for ( int i = 0; i < OUT_WORDS; ++i )
	{
		say( " %08x", wb[i] );
	}
	say( "\n" );
}

static long long compare_range( const Battery* b, int h, const BatOut* twin, const BatOut* gpu, int* dumps )
{
	long long bad = 0;
	for ( int i = b->start[h]; i < b->start[h + 1]; ++i )
	{
		const uint32_t* x = (const uint32_t*)( twin + i );
		const uint32_t* y = (const uint32_t*)( gpu + i );
		int diff = 0;
		for ( int w = 0; w < OUT_WORDS; ++w )
		{
			diff |= x[w] != y[w];
		}
		if ( diff )
		{
			bad += 1;
			if ( *dumps < 6 )
			{
				*dumps += 1;
				char what[64];
				snprintf( what, sizeof( what ), "%s #%d", g_names[h], i - b->start[h] );
				dump_vec( what, b->in + i, twin + i, gpu + i );
			}
		}
	}
	return bad;
}

#if defined( DIALECT_V4 )
// C's integer division: n 2^k / d (int64, truncating toward zero), saturated to +-(2^31 - 1); d == 0 saturates by
// n's sign. |n| < 2^31 and k <= 32 keep n 2^k inside int64.
static int32_t ref_div( int32_t n, int32_t d, int k, int* sat )
{
	if ( n == 0 )
	{
		return 0;
	}
	if ( d == 0 )
	{
		*sat += 1;
		return n < 0 ? -2147483647 : 2147483647;
	}
	int64_t N = (int64_t)n * ( (int64_t)1 << k );
	int64_t q = N / (int64_t)d;
	if ( q > 2147483647LL || q < -2147483647LL )
	{
		*sat += 1;
		return q > 0 ? 2147483647 : -2147483647;
	}
	return (int32_t)q;
}

static int32_t ref_div_clamp( int32_t n, int32_t d, int k, int32_t lim )
{
	if ( n == 0 )
	{
		return 0;
	}
	uint64_t un = (uint64_t)( n < 0 ? -(int64_t)n : (int64_t)n );
	uint64_t ud = (uint64_t)( d < 0 ? -(int64_t)d : (int64_t)d );
	int neg = ( n < 0 ) != ( d < 0 );
	if ( ( un << k ) >= (uint64_t)lim * ud )
	{
		return neg ? -lim : lim;
	}
	int sat = 0;
	return ref_div( n, d, k, &sat );
}
#endif

// Accuracy of the software functions against C's double arithmetic (a report, not a gate; C's atan2 is used here only
// as a reference, never in a simulation). Saturated V4 results and F results snapped to zero are left out.
static double val( T x, int s )
{
#if defined( DIALECT_V4 )
	return ldexp( (double)x, -s );
#else
	(void)s;
	return (double)x;
#endif
}

static double snapd( double x )
{
#if defined( DIALECT_F )
	return x < ldexp( 1.0, -30 ) && x > -ldexp( 1.0, -30 ) ? 0.0 : x;
#else
	return x;
#endif
}

static void accuracy( const Battery* b, const BatOut* o )
{
	double recip = 0.0, rsq = 0.0, at = 0.0, qn = 0.0, dc = 0.0, un = 0.0;
	int nr = 0, ns = 0, na = 0, nq = 0, nd = 0, nu = 0;
	for ( int i = b->start[H_UNIT]; i < b->start[H_UNIT + 1]; ++i )
	{
		double x = val( o[i].r0, S_Q ), y = val( o[i].r1, S_Q ), z = val( o[i].r2, S_Q );
		double l = sqrt( x * x + y * y + z * z );
		if ( l == 0.0 )
		{
			continue;
		}
		double e = fabs( l - 1.0 );
		un = e > un ? e : un;
		++nu;
	}
	for ( int i = b->start[H_RECIP]; i < b->start[H_RECIP + 1]; ++i )
	{
		double d = val( b->in[i].a, b->in[i].sh ), got = val( o[i].r0, b->in[i].k );
#if defined( DIALECT_V4 )
		if ( d == 0.0 || fabs( 1.0 / d ) >= ldexp( 1.0, 31 - b->in[i].k ) )
		{
			continue;
		}
		double e = fabs( ldexp( got - 1.0 / d, b->in[i].k ) ); // in the result's lsb (truncated: below 1)
#else
		if ( d == 0.0 || ( got == 0.0 && fabs( 1.0 / d ) < ldexp( 1.0, -29 ) ) )
		{
			continue;
		}
		double e = fabs( got * d - 1.0 );
#endif
		recip = e > recip ? e : recip;
		++nr;
	}
	for ( int i = b->start[H_RSQRT]; i < b->start[H_RSQRT + 1]; ++i )
	{
		double x = val( b->in[i].a, b->in[i].sh ), got = val( o[i].r0, b->in[i].k );
		if ( !( x > 0.0 ) )
		{
			continue;
		}
		double ref = 1.0 / sqrt( x );
#if defined( DIALECT_V4 )
		if ( ref >= ldexp( 1.0, 31 - b->in[i].k ) || ref < ldexp( 1.0, 24 - b->in[i].k ) )
		{
			continue; // saturated, or too few bits in V4's format to say
		}
#endif
		double e = fabs( got / ref - 1.0 );
		rsq = e > rsq ? e : rsq;
		++ns;
	}
	for ( int i = b->start[H_ATAN2]; i < b->start[H_ATAN2 + 1]; ++i )
	{
		double y = snapd( val( b->in[i].a, 0 ) ) + 0.0, x = snapd( val( b->in[i].b, 0 ) ) + 0.0; // -0 is +0, as lpAtan2's
		double got = val( o[i].r0, S_A );
		double ref = ( x == 0.0 && y == 0.0 ) ? 0.0 : atan2( y, x );
		double e = fabs( got - ref );
		at = e > at ? e : at;
		++na;
	}
	for ( int i = b->start[H_QNORM]; i < b->start[H_QNORM + 1]; ++i )
	{
		if ( b->in[i].a == 0 && b->in[i].b == 0 && b->in[i].c == 0 && b->in[i].d == 0 )
		{
			continue;
		}
		double x = val( o[i].r0, S_Q ), y = val( o[i].r1, S_Q ), z = val( o[i].r2, S_Q ), s = val( o[i].r3, S_Q );
		double e = fabs( sqrt( x * x + y * y + z * z + s * s ) - 1.0 );
		qn = e > qn ? e : qn;
		++nq;
	}
	for ( int i = b->start[H_DIVCLAMP]; i < b->start[H_DIVCLAMP + 1]; ++i )
	{
		int k = b->in[i].k;
		double n = val( b->in[i].a, 0 ), d = val( b->in[i].b, k ), lim = val( b->in[i].c, 0 ), got = val( o[i].r0, 0 );
#if !defined( DIALECT_V4 )
		d = val( b->in[i].b, 0 );
#endif
		if ( n == 0.0 || d == 0.0 || fabs( n / d ) >= lim || fabs( n / d ) < ldexp( 1.0, -29 ) )
		{
			continue;
		}
		double ref = n / d;
#if defined( DIALECT_V4 )
		double e = fabs( got - ref ); // exact up to the truncation: below 1
#else
		double e = fabs( got / ref - 1.0 );
#endif
		dc = e > dc ? e : dc;
		++nd;
	}
	say( "accuracy (twin): recip max %s %.3g (%d), rsqrt max rel %.3g (%d), atan2 max abs %.3g rad (%d), |quatNormalize| - 1 max "
		 "%.3g (%d), divClamp max %s %.3g (%d)\n",
#if defined( DIALECT_V4 )
		 "abs (lsb)",
#else
		 "rel",
#endif
		 recip, nr, rsq, ns, at, na, qn, nq,
#if defined( DIALECT_V4 )
		 "abs (lsb)",
#else
		 "rel",
#endif
		 dc, nd );
	say( "accuracy (twin): |unit3| - 1 max %.3g (%d)\n", un, nu );
}

// ---------------------------------------------------------------------------------------------------------------------
// GPU
// ---------------------------------------------------------------------------------------------------------------------

typedef struct GpuBat
{
	VkDescriptorSetLayout dsl;
	VkPipelineLayout layout;
	VkPipeline pipe;
	VkDescriptorPool pool;
	VkDescriptorSet set;
	VkBuf in, out, counters, staging;
} GpuBat;

static int gpu_battery( VkGpu* g, const Battery* b, BatOut* out, uint32_t* sat )
{
	GpuBat G;
	memset( &G, 0, sizeof( G ) );
	VkDescriptorSetLayoutBinding bind[3];
	for ( int i = 0; i < 3; ++i )
	{
		bind[i] = ( VkDescriptorSetLayoutBinding ){ (uint32_t)i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL };
	}
	VkDescriptorSetLayoutCreateInfo dci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
	dci.bindingCount = 3;
	dci.pBindings = bind;
	VK_CHECK( vkCreateDescriptorSetLayout( g->device, &dci, NULL, &G.dsl ) );
	VkPushConstantRange pcr = { VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( Push ) };
	VkPipelineLayoutCreateInfo lci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	lci.setLayoutCount = 1;
	lci.pSetLayouts = &G.dsl;
	lci.pushConstantRangeCount = 1;
	lci.pPushConstantRanges = &pcr;
	VK_CHECK( vkCreatePipelineLayout( g->device, &lci, NULL, &G.layout ) );
	G.pipe = vku_pipeline( g, G.layout, "gen/toy/" DIALECT_NAME "/battery.spv", "main" );
	if ( G.pipe == VK_NULL_HANDLE )
	{
		return 0;
	}
	size_t inBytes = (size_t)b->total * sizeof( BatIn );
	size_t outBytes = (size_t)b->total * sizeof( BatOut );
	G.in = vku_buffer( g, inBytes, false );
	G.out = vku_buffer( g, outBytes, false );
	G.counters = vku_buffer( g, 16, false );
	G.staging = vku_buffer( g, inBytes > outBytes ? inBytes : outBytes, true );

	VkDescriptorPoolSize ps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3 };
	VkDescriptorPoolCreateInfo pci = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
	pci.maxSets = 1;
	pci.poolSizeCount = 1;
	pci.pPoolSizes = &ps;
	VK_CHECK( vkCreateDescriptorPool( g->device, &pci, NULL, &G.pool ) );
	VkDescriptorSetAllocateInfo ai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	ai.descriptorPool = G.pool;
	ai.descriptorSetCount = 1;
	ai.pSetLayouts = &G.dsl;
	VK_CHECK( vkAllocateDescriptorSets( g->device, &ai, &G.set ) );
	VkBuf* bufs[3] = { &G.in, &G.out, &G.counters };
	VkDescriptorBufferInfo bi[3];
	VkWriteDescriptorSet w[3];
	for ( int i = 0; i < 3; ++i )
	{
		bi[i] = ( VkDescriptorBufferInfo ){ bufs[i]->buffer, 0, VK_WHOLE_SIZE };
		w[i] = ( VkWriteDescriptorSet ){ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
		w[i].dstSet = G.set;
		w[i].dstBinding = (uint32_t)i;
		w[i].descriptorCount = 1;
		w[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		w[i].pBufferInfo = &bi[i];
	}
	vkUpdateDescriptorSets( g->device, 3, w, 0, NULL );

	memcpy( G.staging.mapped, b->in, inBytes );
	VkCommandBuffer cb = vku_begin( g );
	VkBufferCopy c0 = { 0, 0, inBytes };
	vkCmdCopyBuffer( cb, G.staging.buffer, G.in.buffer, 1, &c0 );
	vkCmdFillBuffer( cb, G.out.buffer, 0, VK_WHOLE_SIZE, 0 );
	vku_submit_wait( g, cb );

	for ( int h = 0; h < H_COUNT; ++h )
	{
		sat[h] = 0;
		if ( b->n[h] == 0 )
		{
			continue;
		}
		cb = vku_begin( g );
		vkCmdFillBuffer( cb, G.counters.buffer, 0, 16, 0 );
		vku_barrier( cb );
		vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipe );
		vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.layout, 0, 1, &G.set, 0, NULL );
		Push push = { (uint32_t)b->start[h], (uint32_t)b->n[h], (uint32_t)h, 0 };
		vkCmdPushConstants( cb, G.layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof( push ), &push );
		vkCmdDispatch( cb, ( (uint32_t)b->n[h] + 63 ) / 64, 1, 1 );
		vku_barrier( cb );
		VkBufferCopy cc = { 0, 0, 16 };
		vkCmdCopyBuffer( cb, G.counters.buffer, G.staging.buffer, 1, &cc );
		vku_submit_wait( g, cb );
		sat[h] = ( (uint32_t*)G.staging.mapped )[0];
	}
	cb = vku_begin( g );
	vku_barrier( cb );
	VkBufferCopy c1 = { 0, 0, outBytes };
	vkCmdCopyBuffer( cb, G.out.buffer, G.staging.buffer, 1, &c1 );
	vku_submit_wait( g, cb );
	memcpy( out, G.staging.mapped, outBytes );

	vkDestroyPipeline( g->device, G.pipe, NULL );
	vkDestroyDescriptorPool( g->device, G.pool, NULL );
	vkDestroyPipelineLayout( g->device, G.layout, NULL );
	vkDestroyDescriptorSetLayout( g->device, G.dsl, NULL );
	vku_free( g, &G.in );
	vku_free( g, &G.out );
	vku_free( g, &G.counters );
	vku_free( g, &G.staging );
	return 1;
}

// ---------------------------------------------------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------------------------------------------------

int main( int argc, char** argv )
{
	int n = 4096;
	uint64_t seed = 20261004;
	int gpuMask = 0xff;
	const char* logPath = NULL;
	for ( int i = 1; i < argc; ++i )
	{
		const char* a = argv[i];
		const char* v = i + 1 < argc ? argv[i + 1] : "";
		if ( strcmp( a, "--n" ) == 0 )
			n = atoi( v ), ++i;
		else if ( strcmp( a, "--seed" ) == 0 )
			seed = (uint64_t)strtoull( v, NULL, 10 ), ++i;
		else if ( strcmp( a, "--gpus" ) == 0 )
			gpuMask = atoi( v ), ++i;
		else if ( strcmp( a, "--log" ) == 0 )
			logPath = v, ++i;
		else
		{
			fprintf( stderr, "unknown option %s\n", a );
			return 1;
		}
	}
	if ( logPath )
	{
		g_log = fopen( logPath, "w" );
	}
	say( "toy battery, dialect %s, %d vectors per helper, seed %llu\n", DIALECT_NAME, n, (unsigned long long)seed );
	say( "twin: %s\n", bat_info() );
	say( "layout sizes C/twin: BatIn %zu/%zu BatOut %zu/%zu\n", sizeof( BatIn ), bat_sizeof( 0 ), sizeof( BatOut ), bat_sizeof( 1 ) );
	if ( sizeof( BatIn ) != bat_sizeof( 0 ) || sizeof( BatOut ) != bat_sizeof( 1 ) )
	{
		say( "FAIL: layout sizes differ\n" );
		return 2;
	}

	Battery b;
	make_battery( &b, n, seed );
	say( "inputs: %d vectors, hash %016llx\n", b.total, (unsigned long long)fnv( FNV0, b.in, (size_t)b.total * sizeof( BatIn ) ) );
	if ( getenv( "TOY_DUMP_INPUTS" ) != NULL )
	{
		FILE* f = fopen( getenv( "TOY_DUMP_INPUTS" ), "wb" );
		fwrite( b.in, sizeof( BatIn ), (size_t)b.total, f );
		fclose( f );
	}

	// The twin, one thread, the floating-point status per helper
	BatOut* twin = (BatOut*)calloc( (size_t)b.total, sizeof( BatOut ) );
	uint32_t counters[4];
	uint32_t twinSat[H_COUNT];
	int twinFlags[H_COUNT];
	uint64_t helperHash[H_COUNT];
	bat_bind( b.in, twin, (size_t)b.total, counters );
	int failures = 0;
	for ( int h = 0; h < H_COUNT; ++h )
	{
		memset( counters, 0, sizeof( counters ) );
		uint32_t groups = ( (uint32_t)b.n[h] + 63 ) / 64;
		fp_clear();
		if ( groups > 0 )
		{
			bat_run( (uint32_t)b.start[h], (uint32_t)b.n[h], (uint32_t)h, 0, groups );
		}
		twinFlags[h] = fp_flags();
		twinSat[h] = counters[0];
		helperHash[h] = fnv( FNV0, twin + b.start[h], (size_t)b.n[h] * sizeof( BatOut ) );
	}

	accuracy( &b, twin );

	// V4: lpDivQ31 and divClamp against C's integer division on every input
#if defined( DIALECT_V4 )
	{
		long long bad = 0, badClamp = 0;
		int sat = 0;
		for ( int i = b.start[H_DIVQ31]; i < b.start[H_DIVQ31 + 1]; ++i )
		{
			int32_t want = ref_div( b.in[i].a, b.in[i].b, b.in[i].k, &sat );
			if ( want != twin[i].r0 )
			{
				if ( bad < 5 )
				{
					say( "  lpDivQ31 MISMATCH: %d * 2^%d / %d: C %d, lpDivQ31 %d\n", b.in[i].a, b.in[i].k, b.in[i].b, want, twin[i].r0 );
				}
				bad += 1;
			}
		}
		for ( int i = b.start[H_DIVCLAMP]; i < b.start[H_DIVCLAMP + 1]; ++i )
		{
			int32_t want = ref_div_clamp( b.in[i].a, b.in[i].b, b.in[i].k, b.in[i].c );
			if ( want != twin[i].r0 )
			{
				if ( badClamp < 5 )
				{
					say( "  divClamp MISMATCH: %d * 2^%d / %d clamped to %d: C %d, divClamp %d\n", b.in[i].a, b.in[i].k, b.in[i].b, b.in[i].c,
						 want, twin[i].r0 );
				}
				badClamp += 1;
			}
		}
		say( "lpDivQ31 vs C's '/': %d inputs, %lld differ; saturations C %d, lpDivQ31 %u\n", b.n[H_DIVQ31], bad, sat, twinSat[H_DIVQ31] );
		say( "divClamp vs C: %d inputs, %lld differ (saturations %u)\n", b.n[H_DIVCLAMP], badClamp, twinSat[H_DIVCLAMP] );
		failures += bad > 0 || badClamp > 0 || (uint32_t)sat != twinSat[H_DIVQ31] || twinSat[H_DIVCLAMP] != 0;
	}
#endif

	// GPUs
	int gpuCount = 0;
	VkGpu gpus[8];
	BatOut* gout[8];
	uint32_t gsat[8][H_COUNT];
#if HAS_GPU
	VkInstance inst = vku_create_instance();
	VkPhysicalDevice phys[8];
	int found = vku_list_gpus( inst, phys, 8 );
	for ( int i = 0; i < found; ++i )
	{
		if ( !( gpuMask & ( 1 << i ) ) )
		{
			continue;
		}
		VkGpu* g = &gpus[gpuCount];
		vku_open_gpu( inst, phys[i], g );
		say( "gpu %d: %s (%s), driver 0x%x, api %u.%u.%u; float controls: denorm preserve/ftz %u/%u, RTE %u, SZINP %u (F-plain: none declared)\n", i,
			 g->props.deviceName, g->vendor, g->props.driverVersion, VK_API_VERSION_MAJOR( g->props.apiVersion ),
			 VK_API_VERSION_MINOR( g->props.apiVersion ), VK_API_VERSION_PATCH( g->props.apiVersion ), g->floatControls.shaderDenormPreserveFloat32,
			 g->floatControls.shaderDenormFlushToZeroFloat32, g->floatControls.shaderRoundingModeRTEFloat32,
			 g->floatControls.shaderSignedZeroInfNanPreserveFloat32 );
		gout[gpuCount] = (BatOut*)calloc( (size_t)b.total, sizeof( BatOut ) );
		if ( !gpu_battery( g, &b, gout[gpuCount], gsat[gpuCount] ) )
		{
			say( "FAIL: no pipeline on %s\n", g->vendor );
			return 2;
		}
		++gpuCount;
	}
#endif

	// Per helper: the twin's hash, saturations and floating-point status; each GPU's mismatching vectors
	uint64_t all = FNV0;
	for ( int h = 0; h < H_COUNT; ++h )
	{
		char flags[96];
		say( "helper %-14s n %6d  hash %016llx  sat %6u  fp %-24s", g_names[h], b.n[h], (unsigned long long)helperHash[h], twinSat[h],
			 fp_names( twinFlags[h], flags, sizeof( flags ) ) );
		all = fnv( all, &helperHash[h], 8 );
		all = fnv( all, &twinSat[h], 4 );
		int dumps = 0;
		for ( int gi = 0; gi < gpuCount; ++gi )
		{
			long long bad = compare_range( &b, h, twin, gout[gi], &dumps );
			say( " | %s %lld differ, sat %u", gpus[gi].vendor, bad, gsat[gi][h] );
			failures += bad > 0 || gsat[gi][h] != twinSat[h];
		}
		say( "\n" );
		// A NaN, infinity or subnormal fails, except snap's subnormal inputs (by design: it must turn them into +0)
		int allowed = h == H_SNAP ? ( FPF_DENORMAL | FPF_UNDERFLOW ) : 0;
		if ( twinFlags[h] & ~allowed )
		{
			say( "  FAIL: the twin's floating-point status after %s\n", g_names[h] );
			failures += 1;
		}
	}
	say( "battery %s hash %016llx (twin: %s)\n", DIALECT_NAME, (unsigned long long)all, bat_info() );
	say( "%s: %d GPU(s) against the twin, %s\n", failures ? "FAIL" : "PASS", gpuCount, failures ? "differences above" : "every word identical" );

#if HAS_GPU
	for ( int gi = 0; gi < gpuCount; ++gi )
	{
		vkDeviceWaitIdle( gpus[gi].device );
		vku_close_gpu( &gpus[gi] );
		free( gout[gi] );
	}
	vkDestroyInstance( inst, NULL );
#endif
	free( twin );
	free( b.in );
	if ( g_log )
	{
		fclose( g_log );
	}
	return failures ? 1 : 0;
}
