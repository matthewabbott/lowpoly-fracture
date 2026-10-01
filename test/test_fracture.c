// SPDX-License-Identifier: MIT

#include "fracture.h"
#include "test_macros.h"

// Cells must be valid convex polyhedra that tile the parent: volumes sum to the parent volume.
static int CheckTiling( const lpPoly* parent, lpShape** cells, int count, float relativeTolerance )
{
	float parentVolume;
	lpVec3 c;
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
	lpPoly_MakeBox( slab, (lpVec3){ 1.5f, 1.0f, 0.15f }, lpTransform_identity, 0 );
	lpFractureInput input = { 0 };
	input.parent = slab;
	input.impact = (lpVec3){ 0.3f, 0.2f, 0.15f };
	input.radius = 0.8f;
	input.fragmentSize = 0.14f;
	input.maxCells = 64;
	input.pattern = pattern;
	input.axis = (lpVec3){ 1.0f, 0.0f, 0.0f };
	input.stretch = 5.0f;
	input.interiorMaterial = 3;
	input.seed = 99;
	input.tolerance = 1e-4f;
	return input;
}

static int TestImpactPattern( void )
{
	lpPoly slab;
	lpFractureInput input = SlabInput( &slab, lp_breakImpact );
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
		float d = lpDistance( cells[i]->centroid, input.impact );
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
	lpFractureInput input = SlabInput( &slab, lp_breakGrain );
	lpShape* cells[LP_MAX_SITES];
	lpFractureStats stats = { 0 };
	int count = lpFracture( &input, cells, NULL, LP_MAX_SITES, &stats );
	ENSURE( count >= 4 );
	ENSURE( CheckTiling( &slab, cells, count, 2e-3f ) == 0 );

	// Splinters: cells are longer along the grain (x) than across it (y)
	float along = 0.0f, across = 0.0f;
	for ( int i = 0; i < count; ++i )
	{
		lpAABB b = cells[i]->bounds;
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
	lpPoly_MakeBox( &pane, (lpVec3){ 1.0f, 0.8f, 0.02f }, lpTransform_identity, 0 );
	lpFractureInput input = { 0 };
	input.parent = &pane;
	input.impact = (lpVec3){ 0.2f, -0.1f, 0.02f };
	input.radius = 0.9f;
	input.fragmentSize = 0.07f;
	input.maxCells = 128;
	input.pattern = lp_breakRadial;
	input.axis = (lpVec3){ 0.0f, 0.0f, 1.0f };
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
		if ( pattern == lp_breakRadial )
		{
			input.axis = (lpVec3){ 0.0f, 0.0f, 1.0f };
		}
		lpShape* a[LP_MAX_SITES];
		lpShape* b[LP_MAX_SITES];
		int na = lpFracture( &input, a, NULL, LP_MAX_SITES, NULL );
		int nb = lpFracture( &input, b, NULL, LP_MAX_SITES, NULL );
		ENSURE( na == nb );
		uint64_t ha = LP_HASH_INIT, hb = LP_HASH_INIT;
		for ( int i = 0; i < na; ++i )
		{
			ha = lpHashBytes( ha, a[i]->vertices, sizeof( lpVec3 ) * (size_t)a[i]->vertexCount );
			hb = lpHashBytes( hb, b[i]->vertices, sizeof( lpVec3 ) * (size_t)b[i]->vertexCount );
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
		lpVec3 points[16];
		for ( int i = 0; i < 16; ++i )
		{
			points[i] = (lpVec3){ lpRandom_Range( &rng, -1.0f, 1.0f ), lpRandom_Range( &rng, -0.6f, 0.6f ),
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
		input.axis = lpNormalize( (lpVec3){ 1.0f, lpRandom_Range( &rng, -0.3f, 0.3f ), 0.0f } );
		input.stretch = 4.0f;
		input.seed = (uint64_t)trial;
		input.tolerance = 2e-5f;

		lpShape* cells[LP_MAX_SITES];
		lpFractureStats stats = { 0 };
		int count = lpFracture( &input, cells, NULL, LP_MAX_SITES, &stats );
		failures += stats.failureCount;
		if ( count > 0 )
		{
			float pv, sum = 0.0f;
			lpVec3 pc;
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
// Masonry cuts lie on the course grid: every cut face of a run or plate is a bed joint (horizontal, at a course line)
// or a head joint (across the run, at a brick line of that course). Volume is conserved.
static int TestMasonryGrid( void )
{
	lpPoly slab;
	lpPoly_MakeBox( &slab, (lpVec3){ 1.6f, 0.8f, 0.15f }, lpTransform_identity, 0 );
	float h = 0.15f, l = 0.3f;
	lpVec3 origin = { -1.6f, -0.8f, 0.0f }; // the grid's origin: the slab's lower corner, in its own frame
	lpFractureInput input = { 0 };
	input.parent = &slab;
	input.impact = (lpVec3){ 0.1f, 0.05f, 0.15f };
	input.radius = 0.7f;
	input.fragmentSize = 0.16f;
	input.maxCells = 80;
	input.pattern = lp_breakMasonry;
	input.axis = (lpVec3){ 1.0f, 0.0f, 0.0f };
	input.courseHeight = h;
	input.brickLength = l;
	input.gridOrigin = origin;
	input.seed = 3;
	input.tolerance = 2e-5f;

	lpShape* cells[LP_MAX_SITES];
	int sites[LP_MAX_SITES];
	lpFractureStats stats = { 0 };
	int count = lpFracture( &input, cells, sites, LP_MAX_SITES, &stats );
	printf( "  %d masonry cells\n", count );
	ENSURE( count > 10 && count <= 80 );
	ENSURE( stats.failureCount == 0 );
	ENSURE( CheckTiling( &slab, cells, count, 1e-3f ) == 0 );
	float brick = h * l * 0.3f;
	for ( int i = 0; i < count; ++i )
	{
		if ( cells[i]->volume <= 1.01f * brick )
		{
			continue; // a loose brick or a chip of one
		}
		for ( int f = 0; f < cells[i]->faceCount; ++f )
		{
			const lpFace* face = cells[i]->faces + f;
			if ( face->tag != LP_TAG_CUT )
			{
				continue;
			}
			lpVec3 n = face->plane.normal;
			float d = face->plane.offset - lpDot( n, origin );
			if ( fabsf( n.y ) > 0.999f )
			{
				float k = d / ( h * n.y );
				ENSURE( fabsf( k - roundf( k ) ) < 1e-2f ); // on a course line
			}
			else
			{
				ENSURE( fabsf( n.x ) > 0.999f );
				float u = d / n.x; // the head joint's position along the run
				float inOdd = ( u - 0.5f * l ) / l, inEven = u / l;
				ENSURE( fabsf( inEven - roundf( inEven ) ) < 1e-2f || fabsf( inOdd - roundf( inOdd ) ) < 1e-2f );
			}
		}
	}
	FreeCells( cells, count );
	return 0;
}

static int TestChipCell( void )
{
	lpPoly box;
	lpPoly_MakeBox( &box, (lpVec3){ 0.3f, 0.05f, 0.08f }, lpTransform_identity, 0 );
	lpShape* cell = lpShape_Create( &box );
	ENSURE( cell != NULL );
	lpVec3 grain = { 1.0f, 0.0f, 0.0f };
	int total = 0;
	for ( int trial = 0; trial < 40; ++trial )
	{
		lpRandom rng;
		lpRandom_Seed( &rng, (uint64_t)trial, 5 );
		int splits = 1 + trial % 3;
		lpVec3 axis = trial % 2 == 0 ? grain : lpVec3_zero;
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
						ENSURE( fabsf( lpDot( chips[i]->faces[f].plane.normal, grain ) ) < 1e-3f );
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
	RUN_TEST( TestMasonryGrid );
	return 0;
}
