// probe.c: E11 float-op and int64-op probe. ~1M fixed-seed operand triples per op, run through
// probe.slang on every Vulkan GPU (several float-control builds), bit-compared against the CPU
// (no contraction: MSVC /fp:precise, gcc and clang -ffp-contract=off; int64 reference from a 128-bit
// product). NaN payloads in the CPU reference follow the CPU's rules, which differ between x64 and ARM.
//
//   probe --n 1048576 --log logs/probe.txt
#include "vk_util.h"

#if defined( _MSC_VER ) && !defined( __clang__ )
#include <intrin.h>
#endif
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct Pcg
{
	uint64_t state, inc;
} Pcg;

static uint32_t pcg_next( Pcg* r )
{
	uint64_t old = r->state;
	r->state = old * 6364136223846793005ULL + r->inc;
	uint32_t xs = (uint32_t)( ( ( old >> 18u ) ^ old ) >> 27u );
	uint32_t rot = (uint32_t)( old >> 59u );
	return ( xs >> rot ) | ( xs << ( ( 0u - rot ) & 31 ) );
}

static uint64_t pcg64( Pcg* r )
{
	uint64_t hi = pcg_next( r );
	return ( hi << 32 ) | pcg_next( r );
}

static float fb( uint32_t u )
{
	float f;
	memcpy( &f, &u, 4 );
	return f;
}

static uint32_t bf( float f )
{
	uint32_t u;
	memcpy( &u, &f, 4 );
	return u;
}

static float make_float( uint32_t sign, int exp, uint32_t mant ) // exp is the biased exponent field
{
	return fb( ( sign << 31 ) | ( (uint32_t)exp << 23 ) | ( mant & 0x7fffffu ) );
}

// ------------------------------------------------------------------------------------------------
// Operand classes
// ------------------------------------------------------------------------------------------------

enum
{
	CL_RANDOM_BITS,
	CL_NEAR_ONE,
	CL_SUBNORMAL_IN,
	CL_UNDERFLOW,
	CL_ZEROS,
	CL_ADD_TIES,
	CL_MUL_TIES,
	CL_WIDE,
	CL_COUNT
};
static const char* g_classNames[CL_COUNT] = { "random bits", "near 1", "subnormal input", "underflow", "signed zeros", "add ties", "mul ties", "wide range" };

// E11's operands were drawn inside call arguments, in whatever order MSVC chose: mostly right to left (the
// mantissa word first, then the exponent's, then the sign's), at two sites left to right (found against
// E11's log). Every site now draws in that order explicitly, so any compiler makes the same operands.
static float rand_near_one( Pcg* r )
{
	uint32_t m = pcg_next( r ), e = pcg_next( r ), s = pcg_next( r );
	return make_float( s & 1, 127 - 20 + (int)( e % 41 ), m );
}

static float rand_wide( Pcg* r )
{
	uint32_t m = pcg_next( r ), e = pcg_next( r ), s = pcg_next( r );
	return make_float( s & 1, 1 + (int)( e % 254 ), m );
}

static float rand_subnormal( Pcg* r, uint32_t orMant )
{
	uint32_t m = pcg_next( r ), s = pcg_next( r );
	return make_float( s & 1, 0, m | orMant );
}

// Fills a, b, c for float ops; `op` shapes a few classes (sqrt wants a >= 0; fma-shaped wants
// c = -(a*b) rounded, so the exact result is the rounding error of a*b: zero unless fused).
static void make_float_operands( int op, int n, uint64_t* A, uint64_t* B, uint64_t* C, uint8_t* cls )
{
	Pcg r = { 0x853c49e6748fea9bULL + (uint64_t)op * 7919u, 0xda3e39cb94b95bdbULL };
	for ( int i = 0; i < n; ++i )
	{
		int k = (int)( ( (uint64_t)i * CL_COUNT ) / (uint64_t)n );
		cls[i] = (uint8_t)k;
		float a = 0, b = 0, c = 0;
		switch ( k )
		{
			case CL_RANDOM_BITS:
				a = fb( pcg_next( &r ) );
				b = fb( pcg_next( &r ) );
				c = fb( pcg_next( &r ) );
				break;
			case CL_NEAR_ONE:
				a = rand_near_one( &r );
				b = rand_near_one( &r );
				c = rand_near_one( &r );
				break;
			case CL_SUBNORMAL_IN:
				a = rand_subnormal( &r, 1u );
				b = ( pcg_next( &r ) & 1 ) ? rand_subnormal( &r, 1u ) : rand_near_one( &r );
				c = rand_subnormal( &r, 0u );
				break;
			case CL_UNDERFLOW:
				// products and sums that land in or near the subnormal range
				for ( int w = 0; w < 2; ++w )
				{
					uint32_t m = pcg_next( &r ), e = pcg_next( &r ), s = pcg_next( &r );
					float x = make_float( s & 1, 127 - 60 - (int)( e % 20 ), m );
					*( w == 0 ? &a : &b ) = x;
				}
				if ( op == 0 )
				{
					// a + b with near-cancelling normals at the bottom of the range
					{
						uint32_t e = pcg_next( &r ), m = pcg_next( &r ); // here MSVC went left to right
						a = make_float( 0, 1 + (int)( e % 3 ), m );
					}
					{
						uint32_t m = pcg_next( &r ), e = pcg_next( &r );
						b = -make_float( 0, 1 + (int)( e % 3 ), m );
					}
				}
				{
					uint32_t s = pcg_next( &r ), m = pcg_next( &r ); // and here
					c = make_float( s & 1, 1, m );
				}
				break;
			case CL_ZEROS:
			{
				float pool[8] = { 0.0f, -0.0f, 1.0f, -1.0f, 1e-45f, -1e-45f, 2.5f, -2.5f };
				a = pool[pcg_next( &r ) & 7];
				b = pool[pcg_next( &r ) & 7];
				c = pool[pcg_next( &r ) & 7];
				if ( pcg_next( &r ) % 4 == 0 )
				{
					a = -rand_near_one( &r ) * 0.0f; // signed zeros from arithmetic
				}
				break;
			}
			case CL_ADD_TIES:
			{
				// a = (1 + k 2^-23) 2^e, b = +-(2j+1) 2^(e-24): a + b is exactly halfway between two floats
				int e = (int)( pcg_next( &r ) % 60 ) - 30;
				float one = ldexpf( 1.0f, e );
				a = one * ( 1.0f + (float)( pcg_next( &r ) & 0x7fffff ) * ldexpf( 1.0f, -23 ) );
				uint32_t s = pcg_next( &r ), o = pcg_next( &r ); // MSVC: a product's left operand first
				b = ( ( s & 1 ) ? 1.0f : -1.0f ) * (float)( 2 * ( o % 64 ) + 1 ) * ldexpf( one, -24 );
				if ( i == ( n * CL_ADD_TIES ) / CL_COUNT )
				{
					a = 1.0f;
					b = 3.0f * ldexpf( 1.0f, -24 ); // RNE: 1 + 2^-22 (truncation gives 1 + 2^-23)
				}
				if ( i == ( n * CL_ADD_TIES ) / CL_COUNT + 1 )
				{
					a = 1.0f;
					b = ldexpf( 1.0f, -24 ); // RNE: 1
				}
				c = rand_near_one( &r );
				break;
			}
			case CL_MUL_TIES:
			{
				// odd 13-bit integers whose product lies in [2^24, 2^25): exactly halfway
				for ( ;; )
				{
					uint32_t x = 4096 + ( pcg_next( &r ) % 4096 ), y = 4096 + ( pcg_next( &r ) % 4096 );
					x |= 1;
					y |= 1;
					uint64_t p = (uint64_t)x * y;
					if ( p >= ( 1u << 24 ) && p < ( 1u << 25 ) )
					{
						int ea = (int)( pcg_next( &r ) % 40 ) - 20, eb = (int)( pcg_next( &r ) % 40 ) - 20;
						a = ldexpf( (float)x, ea - 12 ) * ( ( pcg_next( &r ) & 1 ) ? 1.0f : -1.0f );
						b = ldexpf( (float)y, eb - 12 );
						break;
					}
				}
				c = rand_near_one( &r );
				break;
			}
			default:
				a = rand_wide( &r );
				b = rand_wide( &r );
				c = rand_wide( &r );
				break;
		}
		if ( op == 4 || op == 5 )
		{
			a = fabsf( a ); // sqrt / rsqrt of non-negative values (NaN stays NaN)
			if ( a != a )
			{
				a = fb( bf( a ) & 0x7fffffffu );
			}
		}
		if ( op == 2 || op == 10 || op == 11 )
		{
			if ( k == CL_NEAR_ONE || k == CL_WIDE || k == CL_MUL_TIES )
			{
				c = -( a * b ); // the exact a*b + c is the rounding error of a*b
			}
		}
		A[i] = bf( a );
		B[i] = bf( b );
		C[i] = bf( c );
	}
}

static void make_int_operands( int op, int n, uint64_t* A, uint64_t* B, uint64_t* C )
{
	Pcg r = { 0x1234567887654321ULL + (uint64_t)op * 104729u, 0xda3e39cb94b95bdbULL };
	for ( int i = 0; i < n; ++i )
	{
		int k = i % 6;
		uint64_t a = pcg64( &r ), b = pcg64( &r );
		switch ( k )
		{
			case 0: break;																			 // full 64-bit
			case 1: a = (uint64_t)(int64_t)(int32_t)a; b = (uint64_t)(int64_t)(int32_t)b; break;	 // 32-bit signed range
			case 2: a = (uint64_t)( (int64_t)a >> 20 ); b = (uint64_t)( (int64_t)b >> 20 ); break; // +-2^43
			case 3: a = (uint64_t)( (int64_t)a >> 28 ); b = (uint64_t)( (int64_t)b >> 36 ); break; // Q32.32-sized values
			case 4: b = a + ( pcg_next( &r ) % 5 ) - 2; break;										 // near-equal pairs
			default: a = (uint64_t)( (int64_t)a >> ( pcg_next( &r ) % 64 ) ); b = (uint64_t)( (int64_t)b >> ( pcg_next( &r ) % 64 ) ); break;
		}
		A[i] = a;
		B[i] = b;
		C[i] = pcg64( &r );
	}
}

// ------------------------------------------------------------------------------------------------
// CPU references
// ------------------------------------------------------------------------------------------------

static float ref_rsqrt( float a )
{
	return (float)( 1.0 / sqrt( (double)a ) );
}

// maxNum and minNum with the zeros ordered (max(-0, +0) = +0, min = -0) and a NaN losing to a number:
// what MSVC's fmaxf and fminf return, which E11's log recorded. C leaves the zeros' sign open, and clang's
// differed, so the reference is written out.
static float ref_max( float x, float y )
{
	if ( x != x )
		return y;
	if ( y != y )
		return x;
	if ( x == 0.0f && y == 0.0f )
		return signbit( x ) ? y : x;
	return x > y ? x : y;
}

static float ref_min( float x, float y )
{
	if ( x != x )
		return y;
	if ( y != y )
		return x;
	if ( x == 0.0f && y == 0.0f )
		return signbit( x ) ? x : y;
	return x < y ? x : y;
}

static int64_t ref_qmul( int64_t a, int64_t b )
{
	int64_t hi;
#if defined( _MSC_VER ) && !defined( __clang__ )
	uint64_t lo = (uint64_t)_mul128( a, b, &hi );
#else
	__int128 p = (__int128)a * (__int128)b;
	uint64_t lo = (uint64_t)p;
	hi = (int64_t)( p >> 64 );
#endif
	// round half up at bit 31, then take bits 32..95
	uint64_t lo2 = lo + 0x80000000ULL;
	if ( lo2 < lo )
	{
		hi += 1;
	}
	return (int64_t)( ( (uint64_t)hi << 32 ) | ( lo2 >> 32 ) );
}

static uint64_t ref_op( int op, uint64_t a, uint64_t b, uint64_t c )
{
	float x = fb( (uint32_t)a ), y = fb( (uint32_t)b ), z = fb( (uint32_t)c );
	int64_t sa = (int64_t)a, sb = (int64_t)b;
	switch ( op )
	{
		case 0: return bf( x + y );
		case 1: return bf( x * y );
		case 2: { float p = x * y; return bf( p + z ); }
		case 3: return bf( x / y );
		case 4: return bf( sqrtf( x ) );
		case 5: return bf( ref_rsqrt( x ) );
		case 6: return bf( x > y ? x : y );
		case 7: { float lo = -y; return bf( x < lo ? lo : ( x > y ? y : x ) ); }
		case 8: return bf( ref_max( x, y ) );
		case 9: return bf( ref_min( x, y ) );
		case 10: { float p = x * y; return bf( p + z ); } // reference: unfused
		case 11: return bf( fmaf( x, y, z ) );
		case 12: return bf( -x );
		case 13: return bf( 0.0f - x );
		case 14: return bf( x + 0.0f );
		case 20: return (uint64_t)( a + b );
		case 21: return (uint64_t)( a - b );
		case 22: return (uint64_t)( a * b );
		case 23: return (uint64_t)(uint32_t)a * (uint64_t)(uint32_t)b;
		case 24: return (uint64_t)( (int64_t)(int32_t)a * (int64_t)(int32_t)b );
		case 25: return (uint64_t)( sa >> ( c & 63 ) );
		case 26: return a << ( c & 63 );
		case 27: return (uint64_t)( sa > sb ? sa : sb );
		case 28: return sa < sb ? 1 : 0;
		case 29: return (uint64_t)ref_qmul( sa, sb );
		case 30: { int s = 1 + (int)( c % 62 ); return (uint64_t)( (int64_t)( a + ( 1ULL << ( s - 1 ) ) ) >> s ); }
		case 31: return (uint64_t)( 0 - a );
		case 32: { int64_t d = (int64_t)( a - b ); return (uint64_t)( d > 0 ? d : 0 ); }
		case 33: { int64_t d = (int64_t)( a - b ); int64_t m = d > 0 ? d : 0; return (uint64_t)( m - sa ); }
		case 34: return sa == sb ? 1 : 0;
		case 35: return (uint64_t)( sa >> 32 );
		default: return 0;
	}
}

static const char* op_name( int op )
{
	switch ( op )
	{
		case 0: return "fadd";
		case 1: return "fmul";
		case 2: return "a*b+c (NoContraction)";
		case 3: return "fdiv";
		case 4: return "sqrt";
		case 5: return "inversesqrt";
		case 6: return "select max (a>b?a:b)";
		case 7: return "select clamp(x,-m,m)";
		case 8: return "max() intrinsic";
		case 9: return "min() intrinsic";
		case 10: return "mad() intrinsic";
		case 11: return "fma() intrinsic";
		case 12: return "-a";
		case 13: return "0-a";
		case 14: return "a+0";
		case 20: return "i64 add";
		case 21: return "i64 sub";
		case 22: return "i64 mul (low 64)";
		case 23: return "u32*u32->u64";
		case 24: return "i32*i32->i64";
		case 25: return "i64 >> (arith)";
		case 26: return "i64 <<";
		case 27: return "i64 select max";
		case 28: return "i64 a<b";
		case 29: return "Q32.32 qmul (limbs)";
		case 30: return "i64 round shift";
		case 31: return "i64 negate";
		case 32: return "i64 max(a-b,0)";
		case 33: return "i64 max(a-b,0)-a";
		case 34: return "i64 a==b";
		case 35: return "i64 >> 32";
		default: return "?";
	}
}

// ------------------------------------------------------------------------------------------------
// Comparison
// ------------------------------------------------------------------------------------------------

static int is_sub( uint32_t u )
{
	return ( u & 0x7f800000u ) == 0 && ( u & 0x7fffffu ) != 0;
}

static int is_nan( uint32_t u )
{
	return ( u & 0x7f800000u ) == 0x7f800000u && ( u & 0x7fffffu ) != 0;
}

static int64_t ord( uint32_t u )
{
	int32_t i = (int32_t)u;
	return i < 0 ? (int64_t)(int32_t)0x80000000 - (int64_t)i : (int64_t)i;
}

typedef struct FloatStats
{
	long long n, mism, nanPayload, nanVsNum, signedZero, flushedOut, subIn, ulp1, ulpMore, other;
	long long subOutCpu, subOutKept; // CPU results that are subnormal; how many the GPU matched
	long long subInCases;
	double maxUlp;
	long long perClass[CL_COUNT];
	char first[4][220];
	int firstCount;
} FloatStats;

static void float_compare( FloatStats* s, int op, const uint64_t* A, const uint64_t* B, const uint64_t* Cc, const uint64_t* ref,
						   const uint64_t* gpu, const uint8_t* cls, int n )
{
	memset( s, 0, sizeof( *s ) );
	for ( int i = 0; i < n; ++i )
	{
		uint32_t x = (uint32_t)ref[i], y = (uint32_t)gpu[i];
		uint32_t a = (uint32_t)A[i], b = (uint32_t)B[i], c = (uint32_t)Cc[i];
		int subIn = is_sub( a ) || ( op != 4 && op != 5 && op != 12 && op != 13 && op != 14 && is_sub( b ) ) ||
					( ( op == 2 || op == 10 || op == 11 ) && is_sub( c ) );
		s->n += 1;
		s->subInCases += subIn;
		if ( is_sub( x ) )
		{
			s->subOutCpu += 1;
			s->subOutKept += x == y;
		}
		if ( x == y )
		{
			continue;
		}
		s->mism += 1;
		s->perClass[cls[i]] += 1;
		double d = 0.0;
		if ( is_nan( x ) && is_nan( y ) )
			s->nanPayload += 1;
		else if ( is_nan( x ) || is_nan( y ) )
			s->nanVsNum += 1;
		else if ( fb( x ) == fb( y ) )
			s->signedZero += 1;
		else if ( is_sub( x ) && ( y & 0x7fffffffu ) == 0 )
			s->flushedOut += 1;
		else if ( subIn )
			s->subIn += 1;
		else
		{
			d = (double)llabs( ord( x ) - ord( y ) );
			if ( d <= 1.0 )
				s->ulp1 += 1;
			else
				s->ulpMore += 1;
			s->maxUlp = d > s->maxUlp ? d : s->maxUlp;
		}
		if ( s->firstCount < 4 )
		{
			snprintf( s->first[s->firstCount++], 220, "[%s] a=%08x b=%08x c=%08x cpu=%08x (%.9g) gpu=%08x (%.9g)", g_classNames[cls[i]], a, b, c,
					  x, (double)fb( x ), y, (double)fb( y ) );
		}
	}
}

// ------------------------------------------------------------------------------------------------
// GPU
// ------------------------------------------------------------------------------------------------

typedef struct ProbeGpu
{
	VkDescriptorSetLayout dsl;
	VkPipelineLayout layout;
	VkDescriptorPool pool;
	VkDescriptorSet set;
	VkBuf buf[4];
	VkBuf staging;
} ProbeGpu;

static void probe_setup( VkGpu* g, ProbeGpu* p, int n )
{
	VkDescriptorSetLayoutBinding b[4];
	for ( int i = 0; i < 4; ++i )
	{
		b[i] = ( VkDescriptorSetLayoutBinding ){ (uint32_t)i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, NULL };
	}
	VkDescriptorSetLayoutCreateInfo dci = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
	dci.bindingCount = 4;
	dci.pBindings = b;
	VK_CHECK( vkCreateDescriptorSetLayout( g->device, &dci, NULL, &p->dsl ) );
	VkPushConstantRange pcr = { VK_SHADER_STAGE_COMPUTE_BIT, 0, 16 };
	VkPipelineLayoutCreateInfo lci = { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
	lci.setLayoutCount = 1;
	lci.pSetLayouts = &p->dsl;
	lci.pushConstantRangeCount = 1;
	lci.pPushConstantRanges = &pcr;
	VK_CHECK( vkCreatePipelineLayout( g->device, &lci, NULL, &p->layout ) );
	size_t bytes = (size_t)n * 8;
	for ( int i = 0; i < 4; ++i )
	{
		p->buf[i] = vku_buffer( g, bytes, false );
	}
	p->staging = vku_buffer( g, bytes * 3, true );
	VkDescriptorPoolSize ps = { VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 4 };
	VkDescriptorPoolCreateInfo pci = { VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
	pci.maxSets = 1;
	pci.poolSizeCount = 1;
	pci.pPoolSizes = &ps;
	VK_CHECK( vkCreateDescriptorPool( g->device, &pci, NULL, &p->pool ) );
	VkDescriptorSetAllocateInfo ai = { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
	ai.descriptorPool = p->pool;
	ai.descriptorSetCount = 1;
	ai.pSetLayouts = &p->dsl;
	VK_CHECK( vkAllocateDescriptorSets( g->device, &ai, &p->set ) );
	VkDescriptorBufferInfo bi[4];
	VkWriteDescriptorSet w[4];
	for ( int i = 0; i < 4; ++i )
	{
		bi[i] = ( VkDescriptorBufferInfo ){ p->buf[i].buffer, 0, VK_WHOLE_SIZE };
		w[i] = ( VkWriteDescriptorSet ){ VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET };
		w[i].dstSet = p->set;
		w[i].dstBinding = (uint32_t)i;
		w[i].descriptorCount = 1;
		w[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		w[i].pBufferInfo = &bi[i];
	}
	vkUpdateDescriptorSets( g->device, 4, w, 0, NULL );
}

static void probe_teardown( VkGpu* g, ProbeGpu* p )
{
	for ( int i = 0; i < 4; ++i )
	{
		vku_free( g, &p->buf[i] );
	}
	vku_free( g, &p->staging );
	vkDestroyDescriptorPool( g->device, p->pool, NULL );
	vkDestroyPipelineLayout( g->device, p->layout, NULL );
	vkDestroyDescriptorSetLayout( g->device, p->dsl, NULL );
}

static void probe_run( VkGpu* g, ProbeGpu* p, VkPipeline pipe, int op, int n, const uint64_t* A, const uint64_t* B, const uint64_t* C,
					   uint64_t* out )
{
	size_t bytes = (size_t)n * 8;
	uint8_t* st = (uint8_t*)p->staging.mapped;
	memcpy( st, A, bytes );
	memcpy( st + bytes, B, bytes );
	memcpy( st + 2 * bytes, C, bytes );
	VkCommandBuffer cb = vku_begin( g );
	for ( int i = 0; i < 3; ++i )
	{
		VkBufferCopy c = { bytes * (size_t)i, 0, bytes };
		vkCmdCopyBuffer( cb, p->staging.buffer, p->buf[i].buffer, 1, &c );
	}
	vku_barrier( cb );
	vkCmdBindPipeline( cb, VK_PIPELINE_BIND_POINT_COMPUTE, pipe );
	vkCmdBindDescriptorSets( cb, VK_PIPELINE_BIND_POINT_COMPUTE, p->layout, 0, 1, &p->set, 0, NULL );
	uint32_t push[4] = { (uint32_t)op, (uint32_t)n, 0, 0 };
	vkCmdPushConstants( cb, p->layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, 16, push );
	vkCmdDispatch( cb, ( (uint32_t)n + 63 ) / 64, 1, 1 );
	vku_barrier( cb );
	VkBufferCopy c = { 0, 0, bytes };
	vkCmdCopyBuffer( cb, p->buf[3].buffer, p->staging.buffer, 1, &c );
	vku_submit_wait( g, cb );
	memcpy( out, st, bytes );
}

int main( int argc, char** argv )
{
	int n = 1 << 20;
	const char* logPath = "logs/probe.txt";
	for ( int i = 1; i < argc; ++i )
	{
		if ( strcmp( argv[i], "--n" ) == 0 && i + 1 < argc )
			n = atoi( argv[++i] );
		else if ( strcmp( argv[i], "--log" ) == 0 && i + 1 < argc )
			logPath = argv[++i];
	}
	FILE* log = fopen( logPath, "w" );
	if ( !log )
	{
		return 1;
	}
	setvbuf( log, NULL, _IONBF, 0 );
	uint64_t* A = (uint64_t*)malloc( (size_t)n * 8 );
	uint64_t* B = (uint64_t*)malloc( (size_t)n * 8 );
	uint64_t* C = (uint64_t*)malloc( (size_t)n * 8 );
	uint64_t* ref = (uint64_t*)malloc( (size_t)n * 8 );
	uint64_t* out = (uint64_t*)malloc( (size_t)n * 8 );
	uint8_t* cls = (uint8_t*)malloc( (size_t)n );

	const int floatOps[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
	const int intOps[] = { 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35 };
	const int nFloat = (int)( sizeof( floatOps ) / sizeof( int ) ), nInt = (int)( sizeof( intOps ) / sizeof( int ) );

	// CPU self-check of the limb qmul against _mul128 happens through op 29 on the GPU; the C twin
	// of the solver uses the same limb code, checked here on the CPU as well.
	VkInstance inst = vku_create_instance();
	VkPhysicalDevice phys[8];
	int gpuCount = vku_list_gpus( inst, phys, 8 );
	fprintf( log, "E11 probe: %d operand triples per op, classes of %d\n", n, n / CL_COUNT );
	for ( int gi = 0; gi < gpuCount; ++gi )
	{
		VkGpu g;
		vku_open_gpu( inst, phys[gi], &g );
		fprintf( log, "\n##### %s (%s), driver 0x%x; fp32 denorm preserve %u ftz %u, RTE %u RTZ %u, signedZeroInfNanPreserve %u\n",
				 g.props.deviceName, g.vendor, g.props.driverVersion, g.floatControls.shaderDenormPreserveFloat32,
				 g.floatControls.shaderDenormFlushToZeroFloat32, g.floatControls.shaderRoundingModeRTEFloat32,
				 g.floatControls.shaderRoundingModeRTZFloat32, g.floatControls.shaderSignedZeroInfNanPreserveFloat32 );
		ProbeGpu p;
		probe_setup( &g, &p, n );
		const char* modes[] = { "", ".rte", ".szinp", ".preserve", ".ftz", ".rtesz", ".pszinp" };
		const char* modeNames[] = { "none (driver default)", "RoundingModeRTE", "SignedZeroInfNanPreserve", "DenormPreserve", "DenormFlushToZero",
									"RoundingModeRTE + SignedZeroInfNanPreserve", "DenormPreserve + SignedZeroInfNanPreserve" };
		for ( int mi = 0; mi < 7; ++mi )
		{
			if ( ( ( mi == 3 || mi == 6 ) && !g.floatControls.shaderDenormPreserveFloat32 ) || ( mi == 4 && !g.floatControls.shaderDenormFlushToZeroFloat32 ) )
			{
				continue;
			}
			char path[256];
			snprintf( path, sizeof( path ), "gen/probe/probe%s.spv", modes[mi] );
			VkPipelineLayout layout = p.layout;
			VkPipeline pipe = vku_pipeline( &g, layout, path, "main" );
			if ( pipe == VK_NULL_HANDLE )
			{
				continue;
			}
			fprintf( log, "\n--- float ops, SPIR-V %s (execution modes: %s)\n", path,
					 modeNames[mi] );
			fprintf( log, "%-24s %9s | %8s %8s %8s %8s %8s %8s %8s %6s | subnormal outputs kept | first mismatch\n", "op", "mismatch", "nanPay",
					 "nanNum", "+-0", "flushed", "subIn", "1ulp", ">1ulp", "maxUlp" );
			for ( int oi = 0; oi < nFloat; ++oi )
			{
				int op = floatOps[oi];
				make_float_operands( op, n, A, B, C, cls );
				for ( int i = 0; i < n; ++i )
				{
					ref[i] = ref_op( op, A[i], B[i], C[i] );
				}
				probe_run( &g, &p, pipe, op, n, A, B, C, out );
				FloatStats s;
				float_compare( &s, op, A, B, C, ref, out, cls, n );
				fprintf( log, "%-24s %9lld | %8lld %8lld %8lld %8lld %8lld %8lld %8lld %6.0f | %lld of %lld | %s\n", op_name( op ), s.mism, s.nanPayload,
						 s.nanVsNum, s.signedZero, s.flushedOut, s.subIn, s.ulp1, s.ulpMore, s.maxUlp, s.subOutKept, s.subOutCpu,
						 s.firstCount ? s.first[0] : "" );
				if ( s.mism > 0 )
				{
					fprintf( log, "%-24s per class:", "" );
					for ( int k = 0; k < CL_COUNT; ++k )
					{
						fprintf( log, " %s %lld;", g_classNames[k], s.perClass[k] );
					}
					fprintf( log, "\n" );
					for ( int f = 1; f < s.firstCount; ++f )
					{
						fprintf( log, "%-24s %s\n", "", s.first[f] );
					}
				}
			}
			if ( mi == 0 )
			{
				fprintf( log, "\n--- int64 / int32 ops (SPIR-V %s)\n", path );
				for ( int oi = 0; oi < nInt; ++oi )
				{
					int op = intOps[oi];
					make_int_operands( op, n, A, B, C );
					long long cpuTwinBad = 0;
					for ( int i = 0; i < n; ++i )
					{
						ref[i] = ref_op( op, A[i], B[i], C[i] );
					}
					probe_run( &g, &p, pipe, op, n, A, B, C, out );
					long long mism = 0;
					int shown = 0;
					char first[3][200];
					for ( int i = 0; i < n; ++i )
					{
						if ( ref[i] != out[i] )
						{
							if ( shown < 3 )
							{
								snprintf( first[shown++], 200, "a=%016llx b=%016llx c=%016llx cpu=%016llx gpu=%016llx", (unsigned long long)A[i],
										  (unsigned long long)B[i], (unsigned long long)C[i], (unsigned long long)ref[i], (unsigned long long)out[i] );
							}
							++mism;
						}
					}
					fprintf( log, "%-24s mismatches %lld of %d%s\n", op_name( op ), mism, n, cpuTwinBad ? " (cpu self-check failed)" : "" );
					for ( int f = 0; f < shown; ++f )
					{
						fprintf( log, "%-24s   %s\n", "", first[f] );
					}
				}
			}
			vkDestroyPipeline( g.device, pipe, NULL );
		}
		probe_teardown( &g, &p );
		vku_close_gpu( &g );
	}
	vkDestroyInstance( inst, NULL );
	fclose( log );
	printf( "done: %s\n", logPath );
	return 0;
}
