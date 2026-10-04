// battery.slang: every num.slang helper on a fixed set of input vectors (battery.c makes them), per dialect, on the
// GPUs and on the twin; the outputs are compared word by word. One dispatch per helper: pc.aux names it, the input
// and output vectors are [pc.start, pc.start + pc.count).
#define TOY_COUNTERS_BINDING 2
#include "num.slang"

[[vk::binding( 0, 0 )]] StructuredBuffer<BatIn> batIn;
[[vk::binding( 1, 0 )]] RWStructuredBuffer<BatOut> batOut;
[[vk::push_constant]] ConstantBuffer<Push> pc;

// helper ids (battery.c's names)
#define H_MUL 0
#define H_SUM2 1
#define H_SUM3 2
#define H_RESCALE 3
#define H_SELECT 4
#define H_SNAP 5
#define H_RECIP 6
#define H_RSQRT 7
#define H_DIVCLAMP 8
#define H_DIVQ31 9
#define H_CLZ 10
#define H_ATAN2 11
#define H_QNORM 12
#define H_QMUL 13
#define H_ROTATE 14
#define H_POS 15

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void battery( uint3 tid : SV_DispatchThreadID )
{
	if ( tid.x >= pc.count )
		return;
	uint i = pc.start + tid.x;
	BatIn v = batIn[i];
	BatOut o;
	o.r0 = zero();
	o.r1 = zero();
	o.r2 = zero();
	o.r3 = zero();
	o.u0 = 0u;
	o.u1 = 0u;
	T one = KT( 1.0f, 1.0l, 1073741824 );
	uint h = pc.aux;
	if ( h == H_MUL )
	{
		o.r0 = M( v.a, v.b, v.sh );
	}
	else if ( h == H_SUM2 )
	{
		o.r0 = sum2( v.a, v.b, v.c, v.d, v.sh );
		o.r1 = dif2( v.a, v.b, v.c, v.d, v.sh );
	}
	else if ( h == H_SUM3 )
	{
		o.r0 = sum3( v.a, v.b, v.c, v.d, v.e, v.f, v.sh );
	}
	else if ( h == H_RESCALE )
	{
		o.r0 = rescale( v.a, v.sh );
		o.r1 = rescale( v.b, v.k );
	}
	else if ( h == H_SELECT )
	{
		o.r0 = minT( v.a, v.b );
		o.r1 = maxT( v.a, v.b );
		o.r2 = absT( v.a );
		o.r3 = clampT( v.a, minT( v.b, v.c ), maxT( v.b, v.c ) );
		o.u0 = ( v.a < v.b ? 1u : 0u ) | ( v.a > v.b ? 2u : 0u ) | ( v.a == v.b ? 4u : 0u ) | ( v.a <= v.c ? 8u : 0u );
	}
	else if ( h == H_SNAP )
	{
		o.r0 = snap( v.a );
		o.r1 = snap( v.b );
		o.r2 = snap( v.c );
		o.r3 = snap( v.d );
	}
	else if ( h == H_RECIP )
	{
		o.r0 = recipT( v.a, v.sh, v.k );
	}
	else if ( h == H_RSQRT )
	{
		o.r0 = rsqrtT( v.a, v.sh, v.k );
	}
	else if ( h == H_DIVCLAMP )
	{
		o.r0 = divClamp( v.a, v.b, v.k, v.c );
	}
	else if ( h == H_DIVQ31 )
	{
#if defined( DIALECT_V4 )
		o.r0 = lpDivQ31( v.a, v.b, v.k );
#endif
	}
	else if ( h == H_CLZ )
	{
		o.u0 = uint( clz( uint( v.k ) ) );
		o.u1 = uint( clz( uint( v.sh ) ) );
	}
	else if ( h == H_ATAN2 )
	{
		o.r0 = atan2T( v.a, v.b );
		o.r1 = atan2T( v.c, v.d );
	}
	else if ( h == H_QNORM )
	{
		Q4 q = quatNormalize( q4( v.a, v.b, v.c, v.d ), one );
		o.r0 = q.x;
		o.r1 = q.y;
		o.r2 = q.z;
		o.r3 = q.s;
	}
	else if ( h == H_QMUL )
	{
		Q4 q = quatMul( q4( v.a, v.b, v.c, v.d ), q4( v.c, v.d, v.e, v.f ) );
		o.r0 = q.x;
		o.r1 = q.y;
		o.r2 = q.z;
		o.r3 = q.s;
	}
	else if ( h == H_ROTATE )
	{
		Q4 q = q4( v.a, v.b, v.c, v.d );
		V3 r = rotate( q, v3( v.e, v.f, v.e ) );
		V3 s = invRotate( q, v3( v.f, v.e, v.f ) );
		o.r0 = r.x;
		o.r1 = r.y;
		o.r2 = r.z;
		o.r3 = s.x + s.y + s.z;
	}
	else if ( h == H_POS )
	{
		// a position (V4: a = low word, b = high word), a delta in c, d, e (Q5.26), an offset f (Q8.24)
		Pos3 p;
#if defined( DIALECT_V4 )
		p.xlo = uint( v.a );
		p.xhi = v.b;
		p.ylo = uint( v.b );
		p.yhi = v.a >> 12;
		p.zlo = uint( v.c );
		p.zhi = -( v.b >> 3 );
#else
		p.x = v.a;
		p.y = v.b;
		p.z = v.a - v.b;
#endif
		Pos3 r = posAdd( p, v3( v.c, v.d, v.e ) );
		int2 gx = gridSpanX( r, -v.f, v.f );
		int2 gy = gridSpanY( r, v.f, v.f );
		int2 gz = gridSpanZ( r, zero(), v.f );
#if defined( DIALECT_V4 )
		o.r0 = int( r.xlo );
		o.r1 = r.xhi ^ r.yhi ^ int( r.ylo );
		o.r2 = int( r.zlo ) ^ r.zhi;
#else
		o.r0 = r.x;
		o.r1 = r.y;
		o.r2 = r.z;
#endif
		o.u0 = uint( gx.x ) ^ ( uint( gy.x ) << 7 ) ^ ( uint( gz.x ) << 14 );
		o.u1 = uint( gx.y ) ^ ( uint( gy.y ) << 7 ) ^ ( uint( gz.y ) << 14 );
	}
	batOut[i] = o;
}
