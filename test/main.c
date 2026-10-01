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
int VehicleTest( void );
int SystemsTest( void );
int RigTest( void );

const char* lp_testFilter = NULL;
int lp_testLargeNodes = 0;
int lp_testPhysicsLag = 0;
float lp_testOracleWorst = 0.0f;
int lp_testOracleSolves = 0;
int lp_testOracleFlips = 0;
int lp_testOracleJoints = 0;

int main( int argc, char** argv )
{
	// Optional filters: lpf_test [--lag] [poly|fracture|world|debris|stress|links|vehicles|systems|rigs [TestName]]
	// (--lag: every world reads physics one step late, milestone 8's experiment)
	if ( argc > 1 && strcmp( argv[1], "--lag" ) == 0 )
	{
		lp_testPhysicsLag = 1;
		argc -= 1;
		argv += 1;
	}
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
	if ( only == NULL || strcmp( only, "vehicles" ) == 0 )
	{
		RUN_SUITE( VehicleTest );
	}
	if ( only == NULL || strcmp( only, "systems" ) == 0 )
	{
		RUN_SUITE( SystemsTest );
	}
	if ( only == NULL || strcmp( only, "rigs" ) == 0 )
	{
		RUN_SUITE( RigTest );
	}
	printf( "All tests passed.\n" );
	return 0;
}
