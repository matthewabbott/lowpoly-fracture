// fpflags.h: the CPU's floating-point status, for the twins' sentinels (DESIGN.md "The subnormal sentinel"): a NaN
// or infinity (invalid, divide by zero, overflow) or a subnormal (underflow; on x64 also MXCSR's denormal-operand
// flag; on aarch64 FPSR's input-denormal flag) during a dispatch fails the run, and so does a flushing mode (x64 MXCSR's
// FTZ, bit 15, or DAZ, bit 6; aarch64 FPCR's FZ, bit 24, or FIZ, bit 0): a CPU that flushes subnormals to zero computes
// other values than one that keeps them and hides them from the flags. Each thread has its own flags and modes: a pool
// thread reads its own (fp_flags reports FPF_FLUSH too, so every dispatch checks the mode of every thread that ran it).
//
// What each machine can see. x64: invalid, divide by zero, overflow, underflow (MXCSR UE: a result that is tiny and
// inexact) and a subnormal input (DE). aarch64 with FZ off: invalid, divide by zero, overflow and underflow (UFC: tiny
// and inexact) only; FPSR's IDC (input denormal) is set only when an input is flushed, which FZ off never does, so a
// subnormal input, or a subnormal result that is exact, raises nothing there. The twins compute the same bits on
// every machine, so the x64 sentinel sees what an aarch64 run would have hidden.
#ifndef TOY_FPFLAGS_H
#define TOY_FPFLAGS_H

#include <fenv.h>
#include <stdint.h>

#if defined( _M_X64 ) || defined( __x86_64__ )
#include <xmmintrin.h>
#define TOY_FP_X64 1
#elif defined( __aarch64__ ) && ( defined( __GNUC__ ) || defined( __clang__ ) )
#define TOY_FP_A64 1
#endif

enum
{
	FPF_INVALID = 1,
	FPF_DIVZERO = 2,
	FPF_OVERFLOW = 4,
	FPF_UNDERFLOW = 8,
	FPF_DENORMAL = 16, // x64: a subnormal operand (MXCSR DE); aarch64: an input flushed (FPSR IDC)
	FPF_FLUSH = 32	   // a flushing mode is set: x64 MXCSR FTZ or DAZ, aarch64 FPCR FZ or FIZ
};

#if defined( TOY_FP_A64 )
static inline uint64_t fp_a64_fpcr( void )
{
	uint64_t x;
	__asm__ __volatile__( "mrs %0, fpcr" : "=r"( x ) );
	return x;
}
static inline uint64_t fp_a64_fpsr( void )
{
	uint64_t x;
	__asm__ __volatile__( "mrs %0, fpsr" : "=r"( x ) );
	return x;
}
static inline void fp_a64_set_fpsr( uint64_t x ) { __asm__ __volatile__( "msr fpsr, %0" : : "r"( x ) ); }
#endif

// Nonzero when this thread flushes subnormals (the sentinel cannot see them then): which mode, as text
static inline const char* fp_flush_mode( void )
{
#if defined( TOY_FP_X64 )
	unsigned csr = _mm_getcsr();
	if ( csr & 0x8000u )
	{
		return ( csr & 0x40u ) ? "MXCSR FTZ and DAZ" : "MXCSR FTZ";
	}
	if ( csr & 0x40u )
	{
		return "MXCSR DAZ";
	}
#elif defined( TOY_FP_A64 )
	uint64_t fpcr = fp_a64_fpcr();
	if ( fpcr & ( 1ull << 24 ) )
	{
		return "FPCR FZ";
	}
	if ( fpcr & 1ull ) // FEAT_AFP's FIZ (RES0 without it)
	{
		return "FPCR FIZ";
	}
#endif
	return 0;
}

static inline void fp_clear( void )
{
	feclearexcept( FE_ALL_EXCEPT );
#if defined( TOY_FP_X64 )
	_mm_setcsr( _mm_getcsr() & ~0x3fu );
#elif defined( TOY_FP_A64 )
	fp_a64_set_fpsr( fp_a64_fpsr() & ~0x9full ); // IOC, DZC, OFC, UFC, IXC and IDC (not in FE_ALL_EXCEPT)
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
	m |= ( csr & 0x8040u ) ? FPF_FLUSH : 0;
#elif defined( TOY_FP_A64 )
	uint64_t fpsr = fp_a64_fpsr();
	m |= ( fpsr & 0x01u ) ? FPF_INVALID : 0;
	m |= ( fpsr & 0x02u ) ? FPF_DIVZERO : 0;
	m |= ( fpsr & 0x04u ) ? FPF_OVERFLOW : 0;
	m |= ( fpsr & 0x08u ) ? FPF_UNDERFLOW : 0;
	m |= ( fpsr & 0x80u ) ? FPF_DENORMAL : 0;
	m |= fp_flush_mode() ? FPF_FLUSH : 0;
#endif
	return m;
}

static inline const char* fp_names( int m, char* buf, int size )
{
	int n = 0;
	buf[0] = 0;
	static const char* names[6] = { "invalid", "divzero", "overflow", "underflow", "denormal", "flush-mode" };
	for ( int i = 0; i < 6 && n < size; ++i )
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
