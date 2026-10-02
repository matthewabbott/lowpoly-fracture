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
float lp_testOracleWorst = 0.0f;
int lp_testOracleSolves = 0;
int lp_testOracleFlips = 0;
int lp_testOracleJoints = 0;

const char* lp_testKindNames[LP_TEST_KINDS] = { "outcome", "determinism", "mechanism", "timing" };
int lp_testKinds = ( 1 << LP_TEST_KINDS ) - 1;
bool lp_testListing = false;

#define MAX_TESTS 512
static const char* s_names[MAX_TESTS];
static int s_kinds[MAX_TESTS];
static const char* s_suites[MAX_TESTS];
static int s_count;
static const char* s_suite;

void lpRecordTest( const char* name, int kind )
{
	if ( s_count < MAX_TESTS )
	{
		s_names[s_count] = name;
		s_kinds[s_count] = kind;
		s_suites[s_count] = s_suite;
		s_count += 1;
	}
}

static bool Contract( int kind )
{
	return kind == OUTCOME || kind == DETERMINISM;
}

// The catalogue names each contract test in backticks; every name must be a contract test, and every contract test
// must be named
static int CheckCatalogue( const char* path )
{
	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		printf( "cannot read %s\n", path );
		return 1;
	}
	fseek( f, 0, SEEK_END );
	long size = ftell( f );
	fseek( f, 0, SEEK_SET );
	char* text = (char*)malloc( (size_t)size + 1 );
	size_t read = fread( text, 1, (size_t)size, f );
	text[read] = 0;
	fclose( f );

	int errors = 0;
	bool named[MAX_TESTS] = { false };
	for ( const char* c = strstr( text, "`Test" ); c != NULL; c = strstr( c + 1, "`Test" ) )
	{
		const char* end = strchr( c + 1, '`' );
		if ( end == NULL || end - c - 1 > 63 )
		{
			continue;
		}
		char name[64];
		memcpy( name, c + 1, (size_t)( end - c - 1 ) );
		name[end - c - 1] = 0;
		int found = -1;
		for ( int i = 0; i < s_count; ++i )
		{
			found = strcmp( s_names[i], name ) == 0 ? i : found;
		}
		if ( found < 0 )
		{
			printf( "catalogue: %s names %s, which is no test\n", path, name );
			errors += 1;
		}
		else if ( Contract( s_kinds[found] ) == false )
		{
			printf( "catalogue: %s names %s, a %s test, not part of the contract\n", path, name, lp_testKindNames[s_kinds[found]] );
			errors += 1;
		}
		else
		{
			named[found] = true;
		}
	}
	for ( int i = 0; i < s_count; ++i )
	{
		if ( Contract( s_kinds[i] ) && named[i] == false )
		{
			printf( "catalogue: %s (%s, %s) is a contract test the catalogue does not name\n", s_names[i], s_suites[i],
					lp_testKindNames[s_kinds[i]] );
			errors += 1;
		}
	}
	free( text );
	int contract = 0;
	for ( int i = 0; i < s_count; ++i )
	{
		contract += Contract( s_kinds[i] ) ? 1 : 0;
	}
	printf( "catalogue: %d tests, %d in the contract; %d problems\n", s_count, contract, errors );
	return errors > 0 ? 1 : 0;
}

static int Usage( void )
{
	printf( "usage: lpf_test [--kind outcome|determinism|mechanism|timing] [--contract] [suite [TestName]]\n"
			"       lpf_test --list | --check-catalogue docs/catalogue.md\n"
			"suites: poly fracture world debris stress links vehicles systems rigs\n" );
	return 2;
}

int main( int argc, char** argv )
{
	const char* only = NULL;
	const char* catalogue = NULL;
	bool list = false;
	for ( int i = 1; i < argc; ++i )
	{
		const char* a = argv[i];
		if ( strcmp( a, "--list" ) == 0 )
		{
			list = true;
		}
		else if ( strcmp( a, "--check-catalogue" ) == 0 && i + 1 < argc )
		{
			catalogue = argv[++i];
		}
		else if ( strcmp( a, "--contract" ) == 0 )
		{
			lp_testKinds = ( 1 << OUTCOME ) | ( 1 << DETERMINISM );
		}
		else if ( strcmp( a, "--kind" ) == 0 && i + 1 < argc )
		{
			const char* kind = argv[++i];
			lp_testKinds = 0;
			for ( int k = 0; k < LP_TEST_KINDS; ++k )
			{
				lp_testKinds |= strcmp( kind, lp_testKindNames[k] ) == 0 ? 1 << k : 0;
			}
			if ( lp_testKinds == 0 )
			{
				return Usage();
			}
		}
		else if ( a[0] == '-' )
		{
			return Usage();
		}
		else if ( only == NULL )
		{
			only = a;
		}
		else
		{
			lp_testFilter = a;
		}
	}
	lp_testListing = list || catalogue != NULL;
	if ( lp_testListing == false )
	{
		printf( "lowpoly-fracture tests\n" );
	}

	struct
	{
		const char* name;
		int ( *run )( void );
	} suites[] = {
		{ "poly", PolyTest },		{ "fracture", FractureTest }, { "world", WorldTest },	  { "debris", DebrisTest },
		{ "stress", StressTest },	{ "links", LinkTest },		  { "vehicles", VehicleTest }, { "systems", SystemsTest },
		{ "rigs", RigTest },
	};
	for ( int i = 0; i < (int)( sizeof( suites ) / sizeof( suites[0] ) ); ++i )
	{
		if ( lp_testListing )
		{
			s_suite = suites[i].name;
			suites[i].run();
		}
		else if ( only == NULL || strcmp( only, suites[i].name ) == 0 )
		{
			int r = suites[i].run();
			if ( r != 0 )
			{
				printf( "test FAILED: %s\n", suites[i].name );
				return 1;
			}
			printf( "test passed: %s\n", suites[i].name );
		}
	}
	if ( catalogue != NULL )
	{
		return CheckCatalogue( catalogue );
	}
	if ( list )
	{
		for ( int i = 0; i < s_count; ++i )
		{
			printf( "%s %s %s\n", s_suites[i], s_names[i], lp_testKindNames[s_kinds[i]] );
		}
		return 0;
	}
	printf( "All tests passed.\n" );
	return 0;
}
