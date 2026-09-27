// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include <stdlib.h>
#include <string.h>

int PolyTest( void );
int FractureTest( void );
int WorldTest( void );

int main( int argc, char** argv )
{
	// Optional filter: lpf_test poly|fracture|world
	const char* only = argc > 1 ? argv[1] : NULL;
	printf( "lowpoly-fracture tests\n" );
	if ( only == NULL || strcmp( only, "poly" ) == 0 )
	{
		RUN_TEST( PolyTest );
	}
	if ( only == NULL || strcmp( only, "fracture" ) == 0 )
	{
		RUN_TEST( FractureTest );
	}
	if ( only == NULL || strcmp( only, "world" ) == 0 )
	{
		RUN_TEST( WorldTest );
	}
	printf( "All tests passed.\n" );
	return 0;
}
