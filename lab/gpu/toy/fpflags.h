// fpflags.h: the CPU's floating-point status, for the twins' sentinels (DESIGN.md "The subnormal sentinel"): a NaN
// or infinity (invalid, divide by zero, overflow) or a subnormal (underflow; on x64 also MXCSR's denormal-operand
// flag) during a dispatch fails the run. Each thread has its own flags: a pool thread reads its own.
#ifndef TOY_FPFLAGS_H
#define TOY_FPFLAGS_H

#include <fenv.h>

#if defined( _M_X64 ) || defined( __x86_64__ )
#include <xmmintrin.h>
#define TOY_FP_X64 1
#endif

enum
{
	FPF_INVALID = 1,
	FPF_DIVZERO = 2,
	FPF_OVERFLOW = 4,
	FPF_UNDERFLOW = 8,
	FPF_DENORMAL = 16 // x64 only: a subnormal operand
};

static inline void fp_clear( void )
{
	feclearexcept( FE_ALL_EXCEPT );
#if defined( TOY_FP_X64 )
	_mm_setcsr( _mm_getcsr() & ~0x3fu );
#endif
}

static inline int fp_flags( void )
{
	int m = 0;
	m |= fetestexcept( FE_INVALID ) ? FPF_INVALID : 0;
	m |= fetestexcept( FE_DIVBYZERO ) ? FPF_DIVZERO : 0;
	m |= fetestexcept( FE_OVERFLOW ) ? FPF_OVERFLOW : 0;
	m |= fetestexcept( FE_UNDERFLOW ) ? FPF_UNDERFLOW : 0;
#if defined( TOY_FP_X64 )
	unsigned csr = _mm_getcsr();
	m |= ( csr & 0x01u ) ? FPF_INVALID : 0;
	m |= ( csr & 0x02u ) ? FPF_DENORMAL : 0;
	m |= ( csr & 0x04u ) ? FPF_DIVZERO : 0;
	m |= ( csr & 0x08u ) ? FPF_OVERFLOW : 0;
	m |= ( csr & 0x10u ) ? FPF_UNDERFLOW : 0;
#endif
	return m;
}

static inline const char* fp_names( int m, char* buf, int size )
{
	int n = 0;
	buf[0] = 0;
	static const char* names[5] = { "invalid", "divzero", "overflow", "underflow", "denormal" };
	for ( int i = 0; i < 5 && n < size; ++i )
	{
		if ( m & ( 1 << i ) )
		{
			int w = 0;
			const char* s = names[i];
			if ( n > 0 && n + 1 < size )
			{
				buf[n++] = ' ';
			}
			while ( s[w] && n + 1 < size )
			{
				buf[n++] = s[w++];
			}
			buf[n] = 0;
		}
	}
	return m ? buf : "none";
}

#endif
