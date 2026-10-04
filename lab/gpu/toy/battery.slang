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
#define H_UNIT 16
#define H_DELTA 17
#define H_FRAME 18

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
	else if ( h == H_UNIT )
	{
		// the narrowphase's normalisation (any format in V4) and length
		V3 u = unit3( v3( v.a, v.b, v.c ) );
		o.r0 = u.x;
		o.r1 = u.y;
		o.r2 = u.z;
		o.r3 = len3( v3( v.d, v.e, v.f ) );
	}
	else if ( h == H_DELTA )
	{
		// b - a for two positions (V4: words from the inputs, some beyond the +-127 m range)
		Pos3 pa;
		Pos3 pb;
#if defined( DIALECT_V4 )
		pa.xlo = uint( v.a );
		pa.xhi = v.b >> 24;
		pa.ylo = uint( v.c );
		pa.yhi = v.b >> 20;
		pa.zlo = uint( v.e );
		pa.zhi = -( v.b >> 22 );
		pb.xlo = uint( v.b );
		pb.xhi = pa.xhi + ( v.d >> 25 );
		pb.ylo = uint( v.d );
		pb.yhi = pa.yhi + ( v.f >> 24 );
		pb.zlo = uint( v.f );
		pb.zhi = pa.zhi + ( v.a >> 26 );
#else
		pa.x = v.a;
		pa.y = v.b;
		pa.z = v.c;
		pb.x = v.a + v.d;
		pb.y = v.b + v.e;
		pb.z = v.c + v.f;
#endif
		V3 d = posDelta( pa, pb );
		o.r0 = d.x;
		o.r1 = d.y;
		o.r2 = d.z;
	}
	else if ( h == H_FRAME )
	{
		// B in A's frame and back: qA = (a, b, c, d) unit, qB = (b, -a, d, -c); positions from e and f (Q8.24)
		BodyPose pa;
		BodyPose pb;
		pa.q = q4( v.a, v.b, v.c, v.d );
		pb.q = q4( v.b, -v.a, v.d, -v.c );
#if defined( DIALECT_V4 )
		pa.p.xlo = uint( v.e ) << 8u;
		pa.p.xhi = v.e >> 24;
		pa.p.ylo = uint( v.f ) << 8u;
		pa.p.yhi = v.f >> 24;
		pa.p.zlo = uint( v.e ) << 8u;
		pa.p.zhi = v.e >> 24;
		pb.p.xlo = uint( v.f ) << 8u;
		pb.p.xhi = v.f >> 24;
		pb.p.ylo = uint( v.e ) << 8u;
		pb.p.yhi = v.e >> 24;
		pb.p.zlo = uint( -v.f ) << 8u;
		pb.p.zhi = ( -v.f ) >> 24;
#else
		pa.p.x = v.e;
		pa.p.y = v.f;
		pa.p.z = v.e;
		pb.p.x = v.f;
		pb.p.y = v.e;
		pb.p.z = -v.f;
#endif
		Frame f = frameBtoA( pa, pb, one );
		Frame g = invFrame( f );
		V3 x = toFrame( f, v3( v.e, v.f, -v.e ) );
		V3 y = toFrame( g, x );
		o.r0 = y.x;
		o.r1 = y.y;
		o.r2 = y.z;
		o.r3 = x.x;
		H2 hh = hashT( hashT( hashT( hashT( hashStart( 0u, 0u ), x.y ), x.z ), f.R.r1.z ), g.p.x );
		o.u0 = hh.a;
		o.u1 = hh.b;
	}
	batOut[i] = o;
}
