// SPDX-License-Identifier: MIT

#include "fcheck.h"
#include "fracture.h"
#include "test_macros.h"
#include "test_sim.h"

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
		int count = lpChipCell( cell, splits, axis, 0, 1e-6f, &rng, chips, 8, NULL );
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

// ---- the fracture checks (fcheck.h): every validator catches the fault it is for ----

// A box shape: dyadic halves and centres keep every coordinate, plane and volume exact in float
static lpShape* BoxShape( lpVec3 half, lpVec3 center )
{
	lpPoly poly;
	lpTransform xf = { center, lpQuat_identity };
	lpPoly_MakeBox( &poly, half, xf, 0 );
	return lpShape_Create( &poly );
}

// The cube [-1, 1]^3 cut into its eight octants: an exact tiling, every inner face with an exact twin
static void Octants( lpShape** cells )
{
	for ( int i = 0; i < 8; ++i )
	{
		lpVec3 c = { ( i & 1 ) ? 0.5f : -0.5f, ( i & 2 ) ? 0.5f : -0.5f, ( i & 4 ) ? 0.5f : -0.5f };
		cells[i] = BoxShape( (lpVec3){ 0.5f, 0.5f, 0.5f }, c );
	}
}

static int TestCheckCatchesFaults( void )
{
	lpPoly parent;
	lpPoly_MakeBox( &parent, (lpVec3){ 1.0f, 1.0f, 1.0f }, lpTransform_identity, 0 );
	lpShape* cells[8];
	Octants( cells );
	lpShape* whole = lpShape_Create( &parent );

	// Clean: nothing to report
	lpFractureCheck c;
	lpFractureCheck_Init( &c );
	ENSURE( lpFractureCheck_Tiling( &c, &parent, cells, 8 ) == 0.0 );
	lpFractureCheck_Overlap( &c, cells, 8 );
	lpFractureCheck_Siblings( &c, &parent, cells, 8 );
	lpFractureCheck_Cells( &c, cells, 8 );
	lpFractureCheck_Containment( &c, cells, 8, cells, 8 );
	lpFractureCheck_Chips( &c, whole, cells, 8 );
	printf( "  clean: %d violations, %d sibling faces (%d exact), %d overlap pairs\n", lpFractureCheck_Violations( &c ),
			c.siblingFaces, c.siblingExact, c.overlapPairs );
	ENSURE( lpFractureCheck_Violations( &c ) == 0 );
	ENSURE( c.siblingFaces == 24 && c.siblingExact == 24 && c.overlapPairs == 28 );
	ENSURE( c.census.closePairs == 0 && c.census.gridCollisions == 0 && c.census.maxCoordinate == 1.0 );

	// A gap: an octant missing (tiling, and three faces of its neighbours unmatched)
	lpFractureCheck_Init( &c );
	lpFractureCheck_Tiling( &c, &parent, cells, 7 );
	lpFractureCheck_Siblings( &c, &parent, cells, 7 );
	ENSURE( c.tilingViolations == 1 && c.tilingGaps == 1 && fabs( c.tilingMaxError - 0.125 ) < 1e-12 );
	ENSURE( c.siblingUnmatched == 3 && fabs( c.siblingUnmatchedArea - 3.0 ) < 1e-12 );

	// An overlapping pair: octant 0 pushed 1/8 m into octant 1
	lpShape* moved[8];
	memcpy( moved, cells, sizeof( moved ) );
	moved[0] = BoxShape( (lpVec3){ 0.5f, 0.5f, 0.5f }, (lpVec3){ -0.375f, -0.5f, -0.5f } );
	lpFractureCheck_Init( &c );
	lpFractureCheck_Overlap( &c, moved, 8 );
	ENSURE( c.overlapViolations == 1 && fabs( c.overlapMax - 0.125 ) < 1e-12 );
	// and in a chip set
	lpFractureCheck_Init( &c );
	lpFractureCheck_Chips( &c, whole, moved, 8 );
	ENSURE( c.chipViolations == 1 && c.chipMaxOverlap > 0.0 );
	lpShape_Destroy( moved[0] );

	// A mismatched sibling face: octant 0 pulled 1/1024 m away from octant 1 (a gap between faces on near planes)
	moved[0] = BoxShape( (lpVec3){ 0.5f, 0.5f, 0.5f }, (lpVec3){ -0.5f - 1.0f / 1024.0f, -0.5f, -0.5f } );
	lpFractureCheck_Init( &c );
	lpFractureCheck_Siblings( &c, &parent, moved, 8 );
	// Its faces against octants 1, 2 and 4 and theirs against it: on near planes, or offset along a shared plane
	ENSURE( c.siblingNear == 6 && c.siblingUnmatched == 0 && fabs( c.siblingMaxDistance - 1.0 / 1024.0 ) < 1e-9 );
	lpShape_Destroy( moved[0] );

	// A moved vertex: a corner of octant 0 pushed out past its own face planes
	lpPoly poly;
	lpShape_ToPoly( cells[0], &poly );
	for ( int i = 0; i < poly.vertexCount; ++i )
	{
		lpVec3 v = poly.vertices[i];
		if ( v.x == -1.0f && v.y == -1.0f && v.z == -1.0f )
		{
			poly.vertices[i].x = -1.0625f;
		}
	}
	moved[0] = lpShape_Create( &poly );
	lpFractureCheck_Init( &c );
	lpFractureCheck_Cells( &c, moved, 1 );
	ENSURE( c.convexViolations == 1 && fabs( c.maxConvexExcess - 0.0625 ) < 1e-12 && c.maxPlanarity > 0.0 );
	lpShape_Destroy( moved[0] );

	// An open cell: a face missing
	lpShape* open = BoxShape( (lpVec3){ 0.5f, 0.5f, 0.5f }, lpVec3_zero );
	open->faceCount -= 1;
	lpFractureCheck_Init( &c );
	lpFractureCheck_Cells( &c, &open, 1 );
	ENSURE( c.invalidCells == 1 );
	lpShape_Destroy( open );

	// A merged cell missing part of a source cell: octants 0 and 1 merged into a box that stops halfway across octant 1
	lpShape* merged = BoxShape( (lpVec3){ 0.75f, 0.5f, 0.5f }, (lpVec3){ -0.25f, -0.5f, -0.5f } );
	lpFractureCheck_Init( &c );
	lpFractureCheck_Containment( &c, cells, 2, &merged, 1 );
	ENSURE( c.keepersChecked == 2 && c.containViolations == 1 && fabs( c.maxContainExcess - 0.5 ) < 1e-12 );
	lpShape_Destroy( merged );

	// Chips that leave a gap
	lpFractureCheck_Init( &c );
	lpFractureCheck_Chips( &c, whole, cells, 7 );
	ENSURE( c.chipViolations == 1 && fabs( c.chipMaxTilingError - 0.125 ) < 1e-12 );

	// The census: two distinct vertices of one cell closer than the 2^-16 m grid step, on the same grid point
	lpShape* close = BoxShape( (lpVec3){ 0.5f, 0.5f, 0.5f }, (lpVec3){ 0.5f, 0.5f, 0.5f } );
	close->vertices[1] = close->vertices[0]; // the corner at the origin
	close->vertices[1].x += 1.0f / 262144.0f; // 2^-18 m away
	lpShapeCensus census;
	lpShapeCensus_Init( &census );
	lpShapeCensus_Add( &census, close );
	ENSURE( census.closePairs == 1 && census.gridCollisions == 1 && census.minEdge == 1.0 / 262144.0 );
	lpShape_Destroy( close );

	for ( int i = 0; i < 8; ++i )
	{
		lpShape_Destroy( cells[i] );
	}
	lpShape_Destroy( whole );
	return 0;
}

// What the world's jobs look like to a hook: snapshots read back, run again, and their digests compared
typedef struct RoundTrip
{
	int jobs;
	int failures;
	int patterns[5];
	lpFractureJob* copy;
} RoundTrip;

static void RoundTripHook( void* context, const lpFractureJob* job )
{
	RoundTrip* rt = context;
	if ( rt->jobs >= 60 )
	{
		return;
	}
	rt->jobs += 1;
	rt->patterns[job->input.snap ? 4 : (int)job->input.pattern] += 1;
	uint8_t bytes[LP_JOB_SNAPSHOT_MAX], again[LP_JOB_SNAPSHOT_MAX];
	int size = lpFractureJob_Write( job, bytes, (int)sizeof( bytes ) );
	bool ok = size > 0 && size <= LP_JOB_SNAPSHOT_MAX && lpFractureJob_Read( rt->copy, bytes, size );
	// Read and write are inverses, and running the job leaves its snapshot as it was
	ok = ok && lpFractureJob_Write( rt->copy, again, (int)sizeof( again ) ) == size && memcmp( bytes, again, (size_t)size ) == 0;
	if ( ok )
	{
		lpFracture_RunJob( rt->copy );
		ok = lpFractureJob_Digest( rt->copy ) == lpFractureJob_Digest( job );
		ok = ok && lpFractureJob_Write( rt->copy, again, (int)sizeof( again ) ) == size && memcmp( bytes, again, (size_t)size ) == 0;
		lpFracture_FreeJob( rt->copy );
	}
	// Short, long and corrupt bytes fail cleanly
	ok = ok && lpFractureJob_Read( rt->copy, bytes, size - 1 ) == false && lpFractureJob_Read( rt->copy, bytes, 7 ) == false;
	for ( int k = 0; k < 4 && ok; ++k )
	{
		int at = ( k * 997 + rt->jobs * 31 ) % size;
		bytes[at] ^= (uint8_t)( 1u << ( k + rt->jobs ) % 8 );
		ok = lpFractureJob_Read( rt->copy, bytes, size ) == false;
		bytes[at] ^= (uint8_t)( 1u << ( k + rt->jobs ) % 8 );
	}
	rt->failures += ok ? 0 : 1;
}

// A job's snapshot reads back into a job that runs to the same digest, in memory; truncated or corrupt bytes are refused
static int TestJobSnapshotRoundTrip( void )
{
	Sim s = CreateSimWorkers( lp_sceneWall, 1 );
	RoundTrip rt = { 0 };
	rt.copy = lpFractureJob_Create();
	lpWorld_SetFractureHook( s.world, RoundTripHook, &rt );
	for ( int tick = 0; tick < 200 && rt.jobs < 60; ++tick )
	{
		lpSceneBombard( s.world, lp_sceneWall, tick, 12 );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
	}
	printf( "  %d jobs (impact %d, grain %d, radial %d, masonry %d, snap %d) round trip, %d failures\n", rt.jobs, rt.patterns[0],
			rt.patterns[1], rt.patterns[2], rt.patterns[3], rt.patterns[4], rt.failures );
	ENSURE( rt.jobs >= 40 && rt.failures == 0 );
	ENSURE( rt.patterns[0] > 0 && rt.patterns[2] > 0 && rt.patterns[3] > 0 );
	lpFractureJob_Destroy( rt.copy );
	DestroySim( &s );
	return 0;
}

// Sums every job's stats and checks: what --check-fractures counts
typedef struct CheckRun
{
	lpFractureStats stats;
	lpFractureCheck check;
	int jobs;
} CheckRun;

static void CheckHook( void* context, const lpFractureJob* job )
{
	CheckRun* run = context;
	run->jobs += 1;
	lpFractureStats_Add( &run->stats, &job->stats );
	lpFractureJob_Validate( job, &run->check );
	uint8_t bytes[LP_JOB_SNAPSHOT_MAX];
	lpFractureJob_Write( job, bytes, (int)sizeof( bytes ) );
}

static void RunChecked( int workers, int ticks, CheckRun* run, uint64_t* hashes, uint64_t* stressHashes, lpStats* stats )
{
	Sim s = CreateSimWorkers( lp_sceneWall, workers );
	if ( run != NULL )
	{
		memset( run, 0, sizeof( *run ) );
		lpFractureCheck_Init( &run->check );
		lpWorld_SetFractureHook( s.world, CheckHook, run );
	}
	for ( int tick = 0; tick < ticks; ++tick )
	{
		lpSceneBombard( s.world, lp_sceneWall, tick, 12 );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		hashes[tick] = lpWorld_Hash( s.world );
		stressHashes[tick] = lpWorld_HashStress( s.world );
	}
	*stats = lpWorld_GetStats( s.world );
	DestroySim( &s );
}

// A hook that validates and snapshots every job changes nothing: the state hashes match a run without one, every tick
static int TestFractureHookChangesNothing( void )
{
	enum
	{
		ticks = 240
	};
	static uint64_t a[ticks], b[ticks], sa[ticks], sb[ticks];
	static CheckRun run;
	lpStats stA, stB;
	RunChecked( 1, ticks, &run, a, sa, &stA );
	RunChecked( 1, ticks, NULL, b, sb, &stB );
	for ( int i = 0; i < ticks; ++i )
	{
		if ( a[i] != b[i] || sa[i] != sb[i] )
		{
			printf( "  the hook changed the state at tick %d\n", i );
			return 1;
		}
	}
	printf( "  %d jobs validated, %d ticks equal (final %016llx)\n", run.jobs, ticks, (unsigned long long)a[ticks - 1] );
	ENSURE( run.jobs > 20 && run.check.jobs > 10 );
	return 0;
}

// The jobs' stats and the checks' counts are the same at 1 and 8 workers (only their timings differ)
static int TestFractureStatsWorkers( void )
{
	enum
	{
		ticks = 240
	};
	static uint64_t a[ticks], b[ticks], sa[ticks], sb[ticks];
	static CheckRun one, eight;
	lpStats stOne, stEight;
	RunChecked( 1, ticks, &one, a, sa, &stOne );
	RunChecked( 8, ticks, &eight, b, sb, &stEight );
	ENSURE( a[ticks - 1] == b[ticks - 1] );
	lpFractureStats x = one.stats, y = eight.stats;
	x.voronoiMs = x.mergeMs = x.hullMs = x.bondMs = x.chipMs = 0.0f;
	y.voronoiMs = y.mergeMs = y.hullMs = y.bondMs = y.chipMs = 0.0f;
	ENSURE( memcmp( &x, &y, sizeof( x ) ) == 0 ); // ints and floats only: no padding
	const lpFractureCheck* p = &one.check;
	const lpFractureCheck* q = &eight.check;
	ENSURE( one.jobs == eight.jobs && p->jobs == q->jobs && lpFractureCheck_Violations( p ) == lpFractureCheck_Violations( q ) );
	ENSURE( p->tilingMaxError == q->tilingMaxError && p->overlapTotal == q->overlapTotal && p->siblingNear == q->siblingNear &&
			p->siblingMaxDistance == q->siblingMaxDistance && p->maxConvexExcess == q->maxConvexExcess &&
			p->maxContainExcess == q->maxContainExcess && p->chipMaxTilingError == q->chipMaxTilingError &&
			p->census.minEdge == q->census.minEdge && p->census.closePairs == q->census.closePairs );
	ENSURE( stOne.planeShifts == stEight.planeShifts && stOne.clipFailures == stEight.clipFailures );
	printf( "  %d jobs, %d cells, %d clips, %d violations at 1 and 8 workers alike\n", one.jobs, x.outputCells,
			x.clips[lp_clipVoronoi].clips, lpFractureCheck_Violations( p ) );
	return 0;
}

int FractureTest( void )
{
	RUN_TEST( TestImpactPattern, OUTCOME );
	RUN_TEST( TestGrainPattern, OUTCOME );
	RUN_TEST( TestRadialPattern, OUTCOME );
	RUN_TEST( TestFractureDeterminism, DETERMINISM );
	RUN_TEST( TestFractureFuzz, MECHANISM );
	RUN_TEST( TestChipCell, MECHANISM );
	RUN_TEST( TestMasonryGrid, OUTCOME );
	RUN_TEST( TestCheckCatchesFaults, MECHANISM );
	RUN_TEST( TestJobSnapshotRoundTrip, MECHANISM );
	RUN_TEST( TestFractureHookChangesNothing, MECHANISM );
	RUN_TEST( TestFractureStatsWorkers, MECHANISM );
	return 0;
}
