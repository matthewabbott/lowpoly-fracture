// twin_info.h: names the compiler, ISA and contraction setting a CPU twin was built with, for the logs.
// LAB_FMA marks the positive-control builds, whose compiler may fuse a*b+c (CMake's *-fma targets).
#ifndef TWIN_INFO_H
#define TWIN_INFO_H

#define TWIN_STR2( x ) #x
#define TWIN_STR( x ) TWIN_STR2( x )

#if defined( __clang__ )
#define TWIN_COMPILER "clang " __clang_version__
#elif defined( __GNUC__ )
#define TWIN_COMPILER "gcc " __VERSION__
#elif defined( _MSC_VER )
#define TWIN_COMPILER "MSVC " TWIN_STR( _MSC_FULL_VER )
#else
#define TWIN_COMPILER "unknown compiler"
#endif

#if defined( __aarch64__ ) || defined( _M_ARM64 )
#define TWIN_ISA "aarch64"
#elif defined( __x86_64__ ) || defined( _M_X64 )
#define TWIN_ISA "x64"
#else
#define TWIN_ISA "other ISA"
#endif

#if defined( LAB_FMA )
#define TWIN_CONTRACT "contraction ON (positive control)"
#else
#define TWIN_CONTRACT "contraction off"
#endif

#define TWIN_INFO TWIN_COMPILER ", " TWIN_ISA ", " TWIN_CONTRACT

#endif
