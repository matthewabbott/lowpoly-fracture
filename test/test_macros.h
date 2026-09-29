// SPDX-License-Identifier: MIT
#pragma once

#include <math.h>
#include <stdio.h>
#include <string.h>

// lpf_test <suite> <test> runs only the test of that name
extern const char* lp_testFilter;

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

#define RUN_TEST( t )                                                                                                  \
	do                                                                                                                 \
	{                                                                                                                  \
		if ( lp_testFilter == NULL || strcmp( lp_testFilter, #t ) == 0 )                                               \
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
