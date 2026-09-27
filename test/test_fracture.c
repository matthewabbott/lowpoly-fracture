// SPDX-License-Identifier: MIT

#include "fracture.h"
#include "test_macros.h"

#ifndef LP_TEST_TOLERANCE
#define LP_TEST_TOLERANCE 2e-5f
#endif

// Cells must be valid convex polyhedra that tile the parent: volumes sum to the parent volume.
static int CheckTiling( const lpPoly* parent, lpShape** cells, int count, float relativeTolerance )
{
	float parentVolume;
	b3Vec3 c;
	lpPoly_ComputeMass( parent, &parentVolume, &c );

	lpPoly poly;
	float sum = 0.0f;
	for ( int i = 0; i < count; ++i )
	{
		lpShape_ToPoly( cells[i], &poly );
		ENSURE( lpPoly_IsValid( &poly, 1e-3f ) );
		sum += cells[i]->volume;
	}
	ENSURE_NEAR( sum, parentVolume, relativeTolerance * parentVolume );
	return 0;
}

static void FreeCells( lpShape** cells, int count )
{
	for ( int i = 0; i < count; ++i )
	{
		lpShape_Destroy( cells[i] );
	}
}

static lpFractureInput SlabInput( lpPoly* slab, int pattern )
{
	lpPoly_MakeBox( slab, (b3Vec3){ 1.5f, 1.0f, 0.15f }, b3Transform_identity, 0 );
	lpFractureInput input = { 0 };
	input.parent = slab;
	input.impact = (b3Vec3){ 0.3f, 0.2f, 0.15f };
	input.radius = 0.8f;
	input.fragmentSize = 0.14f;
	input.maxCells = 64;
	input.pattern = pattern;
	input.axis = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	input.stretch = 5.0f;
	input.interiorMaterial = 3;
	input.seed = 99;
	input.tolerance = 1e-4f;
	return input;
}

static int TestImpactPattern( void )
{
	lpPoly slab;
	lpFractureInput input = SlabInput( &slab, lp_patternImpact );
	lpShape* cells[LP_MAX_SITES];
	lpFractureStats stats = { 0 };
	int count = lpFracture( &input, cells, NULL, LP_MAX_SITES, &stats );
	ENSURE( count >= 8 );
	ENSURE( stats.failureCount == 0 );
	ENSURE( CheckTiling( &slab, cells, count, 1e-3f ) == 0 );

	// Fragments near the impact are smaller than far ones
	float nearVolume = 0.0f, farVolume = 0.0f;
	int nearCount = 0, farCount = 0;
	for ( int i = 0; i < count; ++i )
	{
		float d = b3Distance( cells[i]->centroid, input.impact );
		if ( d < 0.4f )
		{
			nearVolume += cells[i]->volume;
			nearCount += 1;
		}
		else if ( d > 1.2f )
		{
			farVolume += cells[i]->volume;
			farCount += 1;
		}
	}
	ENSURE( nearCount > 0 && farCount > 0 );
	ENSURE( nearVolume / (float)nearCount < farVolume / (float)farCount );
	FreeCells( cells, count );
	return 0;
}

static int TestGrainPattern( void )
{
	lpPoly slab;
	lpFractureInput input = SlabInput( &slab, lp_patternGrain );
	lpShape* cells[LP_MAX_SITES];
	lpFractureStats stats = { 0 };
	int count = lpFracture( &input, cells, NULL, LP_MAX_SITES, &stats );
	ENSURE( count >= 4 );
	ENSURE( CheckTiling( &slab, cells, count, 2e-3f ) == 0 );

	// Splinters: cells are longer along the grain (x) than across it (y)
	float along = 0.0f, across = 0.0f;
	for ( int i = 0; i < count; ++i )
	{
		b3AABB b = cells[i]->bounds;
		along += b.upperBound.x - b.lowerBound.x;
		across += b.upperBound.y - b.lowerBound.y;
	}
	ENSURE( along > 1.5f * across );
	FreeCells( cells, count );
	return 0;
}

static int TestRadialPattern( void )
{
	lpPoly pane;
	lpPoly_MakeBox( &pane, (b3Vec3){ 1.0f, 0.8f, 0.02f }, b3Transform_identity, 0 );
	lpFractureInput input = { 0 };
	input.parent = &pane;
	input.impact = (b3Vec3){ 0.2f, -0.1f, 0.02f };
	input.radius = 0.9f;
	input.fragmentSize = 0.07f;
	input.maxCells = 128;
	input.pattern = lp_patternRadial;
	input.axis = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	input.seed = 5;
	input.tolerance = 1e-4f;

	lpShape* cells[LP_MAX_SITES];
	int count = lpFracture( &input, cells, NULL, LP_MAX_SITES, NULL );
	ENSURE( count >= 12 );
	ENSURE( CheckTiling( &pane, cells, count, 1e-3f ) == 0 );
	FreeCells( cells, count );
	return 0;
}

// Same input, same bytes
static int TestFractureDeterminism( void )
{
	for ( int pattern = 0; pattern < 3; ++pattern )
	{
		lpPoly slab;
		lpFractureInput input = SlabInput( &slab, pattern );
		if ( pattern == lp_patternRadial )
		{
			input.axis = (b3Vec3){ 0.0f, 0.0f, 1.0f };
		}
		lpShape* a[LP_MAX_SITES];
		lpShape* b[LP_MAX_SITES];
		int na = lpFracture( &input, a, NULL, LP_MAX_SITES, NULL );
		int nb = lpFracture( &input, b, NULL, LP_MAX_SITES, NULL );
		ENSURE( na == nb );
		uint64_t ha = LP_HASH_INIT, hb = LP_HASH_INIT;
		for ( int i = 0; i < na; ++i )
		{
			ha = lpHashBytes( ha, a[i]->vertices, sizeof( b3Vec3 ) * (size_t)a[i]->vertexCount );
			hb = lpHashBytes( hb, b[i]->vertices, sizeof( b3Vec3 ) * (size_t)b[i]->vertexCount );
		}
		ENSURE( ha == hb );
		FreeCells( a, na );
		FreeCells( b, nb );
	}
	return 0;
}

// Many seeds and impact points: no clip failures, always a valid tiling
static int TestFractureFuzz( void )
{
	lpRandom rng;
	lpRandom_Seed( &rng, 77, 3 );
	int failures = 0;
	float maxError = 0.0f;
	for ( int trial = 0; trial < 60; ++trial )
	{
		lpPoly parent;
		b3Vec3 points[16];
		for ( int i = 0; i < 16; ++i )
		{
			points[i] = (b3Vec3){ lpRandom_Range( &rng, -1.0f, 1.0f ), lpRandom_Range( &rng, -0.6f, 0.6f ),
								  lpRandom_Range( &rng, -0.4f, 0.4f ) };
		}
		if ( lpPoly_MakeFromPoints( &parent, points, 16, 0 ) == false )
		{
			continue;
		}
		lpFractureInput input = { 0 };
		input.parent = &parent;
		input.impact = parent.vertices[trial % parent.vertexCount];
		input.radius = lpRandom_Range( &rng, 0.3f, 1.2f );
		input.fragmentSize = lpRandom_Range( &rng, 0.08f, 0.2f );
		input.maxCells = 64;
		input.pattern = trial % 3;
		input.axis = b3Normalize( (b3Vec3){ 1.0f, lpRandom_Range( &rng, -0.3f, 0.3f ), 0.0f } );
		input.stretch = 4.0f;
		input.seed = (uint64_t)trial;
		input.tolerance = LP_TEST_TOLERANCE;

		lpShape* cells[LP_MAX_SITES];
		lpFractureStats stats = { 0 };
		int count = lpFracture( &input, cells, NULL, LP_MAX_SITES, &stats );
		failures += stats.failureCount;
		if ( count > 0 )
		{
			float pv, sum = 0.0f;
			b3Vec3 pc;
			lpPoly_ComputeMass( &parent, &pv, &pc );
			for ( int i = 0; i < count; ++i )
			{
				sum += cells[i]->volume;
			}
			float err = fabsf( sum - pv ) / pv;
			maxError = err > maxError ? err : maxError;
			if ( err > 1e-3f )
			{
				printf( "  trial %d pattern %d cells %d failures %d error %.5f\n", trial, trial % 3, count, stats.failureCount, err );
			}
			ENSURE( CheckTiling( &parent, cells, count, 1e-3f ) == 0 );
		}
		FreeCells( cells, count );
	}
	printf( "  max tiling error %.6f, clip failures %d\n", maxError, failures );
	ENSURE( failures == 0 );
	return 0;
}

// Chips tile the cell they came from, and with a grain axis every chip cut contains the axis (splinters stay long)
static int TestChipCell( void )
{
	lpPoly box;
	lpPoly_MakeBox( &box, (b3Vec3){ 0.3f, 0.05f, 0.08f }, b3Transform_identity, 0 );
	lpShape* cell = lpShape_Create( &box );
	ENSURE( cell != NULL );
	b3Vec3 grain = { 1.0f, 0.0f, 0.0f };
	int total = 0;
	for ( int trial = 0; trial < 40; ++trial )
	{
		lpRandom rng;
		lpRandom_Seed( &rng, (uint64_t)trial, 5 );
		int splits = 1 + trial % 3;
		b3Vec3 axis = trial % 2 == 0 ? grain : b3Vec3_zero;
		lpShape* chips[8];
		int count = lpChipCell( cell, splits, axis, 0, 1e-6f, &rng, chips, 8 );
		ENSURE( count == 0 || ( count >= 2 && count <= splits + 1 ) );
		if ( count > 0 )
		{
			ENSURE( CheckTiling( &box, chips, count, 1e-3f ) == 0 );
			for ( int i = 0; i < count && trial % 2 == 0; ++i )
			{
				for ( int f = 0; f < chips[i]->faceCount; ++f )
				{
					if ( chips[i]->faces[f].tag == LP_TAG_CUT )
					{
						ENSURE( fabsf( b3Dot( chips[i]->faces[f].plane.normal, grain ) ) < 1e-3f );
					}
				}
			}
		}
		total += count;
		FreeCells( chips, count );
	}
	lpShape_Destroy( cell );
	printf( "  %d chips from 40 cells\n", total );
	ENSURE( total > 60 );
	return 0;
}

int FractureTest( void )
{
	RUN_TEST( TestImpactPattern );
	RUN_TEST( TestGrainPattern );
	RUN_TEST( TestRadialPattern );
	RUN_TEST( TestFractureDeterminism );
	RUN_TEST( TestFractureFuzz );
	RUN_TEST( TestChipCell );
	return 0;
}
