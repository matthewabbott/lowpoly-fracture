// SPDX-License-Identifier: MIT
#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// lpf_test <suite> <test> runs only the test of that name
extern const char* lp_testFilter;

// What a test pins (docs/catalogue.md). The contract, which a redesign or a new physics core must still meet:
// - OUTCOME: a behaviour the games rely on, with a tolerance a different but plausible simulation would also meet;
// - DETERMINISM: the same state, bit for bit, across runs and worker counts.
// The rest pins today's implementation, and may change with it:
// - MECHANISM: unit geometry, a solver against its oracle, tooling, the physics engine's own numerics;
// - TIMING: costs, printed only.
enum
{
	OUTCOME,
	DETERMINISM,
	MECHANISM,
	TIMING,
	LP_TEST_KINDS
};
extern const char* lp_testKindNames[LP_TEST_KINDS];
extern int lp_testKinds;	// bit per kind to run (lpf_test --kind, --contract)
extern bool lp_testListing; // lpf_test --list, --check-catalogue: tests are recorded, not run
void lpRecordTest( const char* name, int kind );

#define ENSURE( c )                                                                                                    \
	do                                                                                                                 \
	{                                                                                                                  \
		if ( !( c ) )                                                                                                  \
		{                                                                                                              \
			printf( "  FAILED %s:%d: %s\n", __FILE__, __LINE__, #c );                                                  \
			return 1;                                                                                                  \
		}                                                                                                              \
	}                                                                                                                  \
	while ( 0 )

// a within a fraction rel of b
#define ENSURE_REL( a, b, rel ) ENSURE_NEAR( a, b, (double)( rel ) * fabs( (double)( b ) ) )

#define ENSURE_NEAR( a, b, tol )                                                                                       \
	do                                                                                                                 \
	{                                                                                                                  \
		double lpA_ = (double)( a ), lpB_ = (double)( b );                                                             \
		if ( fabs( lpA_ - lpB_ ) > (double)( tol ) )                                                                   \
		{                                                                                                              \
			printf( "  FAILED %s:%d: %s = %.9g, %s = %.9g\n", __FILE__, __LINE__, #a, lpA_, #b, lpB_ );                \
			return 1;                                                                                                  \
		}                                                                                                              \
	}                                                                                                                  \
	while ( 0 )

#define RUN_TEST( t, kind )                                                                                            \
	do                                                                                                                 \
	{                                                                                                                  \
		if ( lp_testListing )                                                                                          \
		{                                                                                                              \
			lpRecordTest( #t, kind );                                                                                  \
		}                                                                                                              \
		else if ( ( lp_testFilter == NULL || strcmp( lp_testFilter, #t ) == 0 ) && ( lp_testKinds >> ( kind ) & 1 ) )  \
		{                                                                                                              \
			RUN_SUITE( t );                                                                                            \
		}                                                                                                              \
	}                                                                                                                  \
	while ( 0 )

// A whole suite, whatever the filter
#define RUN_SUITE( t )                                                                                                 \
	do                                                                                                                 \
	{                                                                                                                  \
		int lpR_ = t();                                                                                                \
		if ( lpR_ != 0 )                                                                                               \
		{                                                                                                              \
			printf( "test FAILED: %s\n", #t );                                                                         \
			return 1;                                                                                                  \
		}                                                                                                              \
		printf( "test passed: %s\n", #t );                                                                             \
	}                                                                                                                  \
	while ( 0 )
