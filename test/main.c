// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include <stdlib.h>
#include <string.h>

int PolyTest( void );
int FractureTest( void );
int WorldTest( void );
int DebrisTest( void );
int StressTest( void );
int LinkTest( void );

const char* lp_testFilter = NULL;

int main( int argc, char** argv )
{
	// Optional filters: lpf_test [poly|fracture|world|debris|stress|links [TestName]]
	const char* only = argc > 1 ? argv[1] : NULL;
	lp_testFilter = argc > 2 ? argv[2] : NULL;
	printf( "lowpoly-fracture tests\n" );
	if ( only == NULL || strcmp( only, "poly" ) == 0 )
	{
		RUN_SUITE( PolyTest );
	}
	if ( only == NULL || strcmp( only, "fracture" ) == 0 )
	{
		RUN_SUITE( FractureTest );
	}
	if ( only == NULL || strcmp( only, "world" ) == 0 )
	{
		RUN_SUITE( WorldTest );
	}
	if ( only == NULL || strcmp( only, "debris" ) == 0 )
	{
		RUN_SUITE( DebrisTest );
	}
	if ( only == NULL || strcmp( only, "stress" ) == 0 )
	{
		RUN_SUITE( StressTest );
	}
	if ( only == NULL || strcmp( only, "links" ) == 0 )
	{
		RUN_SUITE( LinkTest );
	}
	printf( "All tests passed.\n" );
	return 0;
}
