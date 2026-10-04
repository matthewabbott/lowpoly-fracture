// probe.slang: one operation per dispatch over a buffer of operand triples; results are raw bits.
// Float ops use fp32 with NoContraction (-fp-mode precise); integer ops are the building blocks of
// the Q32.32 and 32-bit fixed-point solvers.

struct ProbePush
{
	uint op;
	uint count;
	uint pad0;
	uint pad1;
};

[[vk::binding( 0, 0 )]] StructuredBuffer<uint64_t> A;
[[vk::binding( 1, 0 )]] StructuredBuffer<uint64_t> B;
[[vk::binding( 2, 0 )]] StructuredBuffer<uint64_t> C;
[[vk::binding( 3, 0 )]] RWStructuredBuffer<uint64_t> R;
[[vk::push_constant]] ConstantBuffer<ProbePush> pp;

float f( uint64_t x ) { return asfloat( uint( x ) ); }
uint64_t u( float x ) { return uint64_t( asuint( x ) ); }

static const uint64_t LOW32 = uint64_t( 0xFFFFFFFFu );

int64_t qmul( int64_t a, int64_t b )
{
	uint64_t ua = uint64_t( a );
	uint64_t ub = uint64_t( b );
	uint al = uint( ua );
	uint ah = uint( ua >> 32 );
	uint bl = uint( ub );
	uint bh = uint( ub >> 32 );
	uint64_t ll = uint64_t( al ) * uint64_t( bl );
	uint64_t lh = uint64_t( al ) * uint64_t( bh );
	uint64_t hl = uint64_t( ah ) * uint64_t( bl );
	uint hh = ah * bh;
	uint64_t mid = ( ll >> 32 ) + ( lh & LOW32 ) + ( hl & LOW32 ) + ( ( ll >> 31 ) & uint64_t( 1 ) );
	uint hi = uint( lh >> 32 ) + uint( hl >> 32 ) + hh + uint( mid >> 32 );
	hi = a < 0 ? hi - bl : hi;
	hi = b < 0 ? hi - al : hi;
	return int64_t( ( uint64_t( hi ) << 32 ) | ( mid & LOW32 ) );
}

[shader( "compute" )]
[numthreads( 64, 1, 1 )]
void main( uint3 tid : SV_DispatchThreadID )
{
	uint i = tid.x;
	if ( i >= pp.count )
		return;
	uint64_t a = A[i], b = B[i], c = C[i];
	uint64_t r = 0;
	switch ( pp.op )
	{
		// fp32
		case 0: r = u( f( a ) + f( b ) ); break;
		case 1: r = u( f( a ) * f( b ) ); break;
		case 2: r = u( f( a ) * f( b ) + f( c ) ); break; // NoContraction: two roundings
		case 3: r = u( f( a ) / f( b ) ); break;
		case 4: r = u( sqrt( f( a ) ) ); break;
		case 5: r = u( rsqrt( f( a ) ) ); break;
		case 6: { float x = f( a ), y = f( b ); r = u( x > y ? x : y ); break; }		  // the solver's maxT
		case 7: { float x = f( a ), m = f( b ), lo = -m; r = u( x < lo ? lo : ( x > m ? m : x ) ); break; } // clampT( x, -m, m )
		case 8: r = u( max( f( a ), f( b ) ) ); break;
		case 9: r = u( min( f( a ), f( b ) ) ); break;
		case 10: r = u( mad( f( a ), f( b ), f( c ) ) ); break; // may fuse
		case 11: r = u( fma( f( a ), f( b ), f( c ) ) ); break;	// fused
		case 12: r = u( -f( a ) ); break;
		case 13: r = u( 0.0f - f( a ) ); break;
		case 14: r = u( f( a ) + 0.0f ); break;
		// int64 and int32 building blocks
		case 20: r = uint64_t( int64_t( a ) + int64_t( b ) ); break;
		case 21: r = uint64_t( int64_t( a ) - int64_t( b ) ); break;
		case 22: r = uint64_t( int64_t( a ) * int64_t( b ) ); break;
		case 23: r = uint64_t( uint( a ) ) * uint64_t( uint( b ) ); break;
		case 24: r = uint64_t( int64_t( int( a ) ) * int64_t( int( b ) ) ); break;
		case 25: r = uint64_t( int64_t( a ) >> int( c & 63 ) ); break;
		case 26: r = uint64_t( int64_t( a ) << int( c & 63 ) ); break;
		case 27: { int64_t x = int64_t( a ), y = int64_t( b ); r = uint64_t( x > y ? x : y ); break; }
		case 28: r = int64_t( a ) < int64_t( b ) ? 1 : 0; break;
		case 29: r = uint64_t( qmul( int64_t( a ), int64_t( b ) ) ); break;
		case 30: { int s = 1 + int( c % 62 ); r = uint64_t( ( int64_t( a ) + ( int64_t( 1 ) << ( s - 1 ) ) ) >> s ); break; }
		case 31: r = uint64_t( -int64_t( a ) ); break;
		case 32: { int64_t x = int64_t( a ) - int64_t( b ); r = uint64_t( x > 0 ? x : int64_t( 0 ) ); break; } // max( a - b, 0 )
		case 33: { int64_t x = int64_t( a ), y = int64_t( b ); int64_t n = x - y; int64_t m = n > 0 ? n : int64_t( 0 ); r = uint64_t( m - x ); break; } // clamp then delta
		case 34: r = int64_t( a ) == int64_t( b ) ? 1 : 0; break;
		case 35: r = uint64_t( int64_t( a ) >> 32 ); break;
		default: r = 0; break;
	}
	R[i] = r;
}
