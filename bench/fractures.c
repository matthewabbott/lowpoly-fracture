// SPDX-License-Identifier: MIT
// lpf_bench --check-fractures, --record-fractures and --replay-fractures (fractures.h). The bench reaches into the
// core's internals here, as the tests do: the fracture job and its hook are not in lpf.h.

#include "fractures.h"

#include "world.h"

#include <float.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

// A recording: "LPFR", a version, a line of text (the run it came from), then one record per job in the run's order:
// the snapshot's size and bytes (lpFractureJob_Write), the digest of what the job made, the tick. Little-endian.
#define LP_RECORDING_VERSION 1

static void lpWriteU32( FILE* f, uint32_t v )
{
	uint8_t b[4] = { (uint8_t)v, (uint8_t)( v >> 8 ), (uint8_t)( v >> 16 ), (uint8_t)( v >> 24 ) };
	fwrite( b, 1, 4, f );
}

static void lpWriteU64( FILE* f, uint64_t v )
{
	lpWriteU32( f, (uint32_t)( v & 0xFFFFFFFFu ) );
	lpWriteU32( f, (uint32_t)( v >> 32 ) );
}

static const char* const lp_patternNames[5] = { "impact", "grain", "radial", "masonry", "snap" };
static const char* const lp_callerNames[lp_clipCallerCount] = { "voronoi", "grain", "radial", "masonry", "snap", "chips" };

static int lpJobPattern( const lpFractureJob* job )
{
	int pattern = (int)job->input.pattern;
	return job->input.snap ? 4 : ( pattern >= 0 && pattern < 4 ? pattern : 0 );
}

static lpBenchFractures* lpBenchFractures_Create( bool check )
{
	lpBenchFractures* bf = calloc( 1, sizeof( lpBenchFractures ) );
	bf->check = check;
	lpFractureCheck_Init( &bf->total );
	for ( int k = 0; k < 5; ++k )
	{
		lpFractureCheck_Init( bf->byPattern + k );
	}
	bf->worstTiling = bf->worstOverlap = bf->worstSibling = bf->worstNear = bf->worstContain = bf->firstInvalid = -1;
	return bf;
}

// One job: its stats, and its checks when asked; index is its place in the run (or the recording)
static void lpBenchFractures_Take( lpBenchFractures* bf, const lpFractureJob* job, int index )
{
	bf->jobs += 1;
	bf->patterns[lpJobPattern( job )] += 1;
	lpFractureStats_Add( &bf->stats, &job->stats );
	if ( bf->check == false )
	{
		return;
	}
	uint64_t ticks = lpGetTicks();
	lpFractureCheck one;
	lpFractureCheck_Init( &one );
	lpFractureJob_Validate( job, &one );
	lpFractureCheck_Add( &bf->total, &one );
	lpFractureCheck_Add( bf->byPattern + lpJobPattern( job ), &one );
	if ( one.tilingMaxError > bf->worstTilingError )
	{
		bf->worstTilingError = one.tilingMaxError;
		bf->worstTiling = index;
	}
	if ( one.overlapMax > bf->worstOverlapVolume )
	{
		bf->worstOverlapVolume = one.overlapMax;
		bf->worstOverlap = index;
	}
	if ( one.siblingUnmatchedArea > bf->worstSiblingArea )
	{
		bf->worstSiblingArea = one.siblingUnmatchedArea;
		bf->worstSibling = index;
	}
	if ( one.siblingMaxDistance > bf->worstNearDistance )
	{
		bf->worstNearDistance = one.siblingMaxDistance;
		bf->worstNear = index;
	}
	if ( one.maxContainExcess > bf->worstContainExcess )
	{
		bf->worstContainExcess = one.maxContainExcess;
		bf->worstContain = index;
	}
	if ( one.invalidCells > 0 && bf->firstInvalid < 0 )
	{
		bf->firstInvalid = index;
	}
	bf->validateMs += lpGetMilliseconds( ticks );
}

static void lpBenchFractureHook( void* context, const lpFractureJob* job )
{
	lpBenchFractures* bf = context;
	int index = bf->jobs;
	lpBenchFractures_Take( bf, job, index );
	if ( bf->record != NULL )
	{
		uint8_t bytes[LP_JOB_SNAPSHOT_MAX];
		int size = lpFractureJob_Write( job, bytes, (int)sizeof( bytes ) );
		lpWriteU32( bf->record, (uint32_t)size );
		fwrite( bytes, 1, (size_t)size, bf->record );
		lpWriteU64( bf->record, lpFractureJob_Digest( job ) );
		lpWriteU32( bf->record, (uint32_t)bf->tick );
	}
}

lpBenchFractures* lpBenchFractures_Begin( lpWorld* world, bool check, const char* recordPath, const char* meta )
{
	lpBenchFractures* bf = lpBenchFractures_Create( check );
	if ( recordPath != NULL )
	{
		bf->record = fopen( recordPath, "wb" );
		if ( bf->record == NULL )
		{
			printf( "cannot write %s\n", recordPath );
		}
		else
		{
			size_t length = strlen( meta );
			fwrite( "LPFR", 1, 4, bf->record );
			lpWriteU32( bf->record, LP_RECORDING_VERSION );
			lpWriteU32( bf->record, (uint32_t)length );
			fwrite( meta, 1, length, bf->record );
		}
	}
	if ( check )
	{
		lpWorld_CensusParts( world, &bf->parts );
		bf->haveParts = true;
	}
	lpWorld_SetFractureHook( world, lpBenchFractureHook, bf );
	return bf;
}

void lpBenchFractures_End( lpBenchFractures* bf, lpWorld* world )
{
	lpWorld_SetFractureHook( world, NULL, NULL );
	if ( bf->record != NULL )
	{
		fclose( bf->record );
		bf->record = NULL;
	}
}

void lpBenchFractures_Free( lpBenchFractures* bf )
{
	free( bf );
}

// ---- printing ----

static int lpClipTotal( const lpFractureStats* s, int field )
{
	int sum = 0;
	for ( int k = 0; k < lp_clipCallerCount; ++k )
	{
		const lpClipStats* c = s->clips + k;
		sum += field == 0 ? c->clips : ( field == 1 ? c->shifts : ( field == 2 ? c->toleranceOuts : c->failures ) );
	}
	return sum;
}

static float lpMaxShift( const lpFractureStats* s )
{
	float m = 0.0f;
	for ( int k = 0; k < lp_clipCallerCount; ++k )
	{
		m = s->clips[k].maxShift > m ? s->clips[k].maxShift : m;
	}
	return m;
}

static void lpPrintBins( const char* name, const int* bins, bool faces, int max )
{
	printf( "%s max %d [", name, max );
	for ( int k = 0; k < LP_CELL_BINS; ++k )
	{
		if ( k < LP_CELL_BINS - 1 )
		{
			printf( "%s<=%d: %d", k > 0 ? ", " : "", lpCellBinLimit( faces, k ), bins[k] );
		}
		else
		{
			printf( ", more: %d", bins[k] );
		}
	}
	printf( "]" );
}

static double lpOrZero( double minimum )
{
	return minimum == DBL_MAX ? 0.0 : minimum;
}

static void lpPrintCensus( const lpShapeCensus* c )
{
	printf( "max |coordinate| %.3f m (%d vertices beyond %.0f m), min edge %.3g m, min face area %.3g m^2, faces max %d, vertices "
			"max %d, close pairs %d, grid collisions %d",
			c->maxCoordinate, c->beyondRange, LP_CHECK_RANGE, lpOrZero( c->minEdge ), lpOrZero( c->minFaceArea ), c->maxFaces,
			c->maxVertices, c->closePairs, c->gridCollisions );
}

void lpBenchFractures_Print( const lpBenchFractures* bf )
{
	const lpFractureStats* s = &bf->stats;
	printf( "        fracture jobs: %d (", bf->jobs );
	for ( int k = 0; k < 5; ++k )
	{
		printf( "%s%s %d", k > 0 ? ", " : "", lp_patternNames[k], bf->patterns[k] );
	}
	printf( "); pattern cells %d (sites %d, slivers absorbed %d, dropped %d), cells out %d\n", s->patternCells, s->sitesDrawn,
			s->sliversAbsorbed, s->cellsDropped, s->outputCells );
	int clips = lpClipTotal( s, 0 );
	int shifts = lpClipTotal( s, 1 );
	printf( "        float clips %d (", clips );
	for ( int k = 0; k < lp_clipCallerCount; ++k )
	{
		printf( "%s%s %d/%d", k > 0 ? ", " : "", lp_callerNames[k], s->clips[k].clips, s->clips[k].shifts );
	}
	printf( " clips/shifts): plane shifts %d (%.2f%%), max shift %.3f mm, tolerance outs %d, failures %d (cells skipped a "
			"plane %d)\n",
			shifts, clips > 0 ? 100.0 * shifts / clips : 0.0, 1000.0 * (double)lpMaxShift( s ), lpClipTotal( s, 2 ),
			lpClipTotal( s, 3 ), s->failureCount );
	printf( "        merge: %d tried, %d too big, %d pre-rejected, %d quickhulls, %d accepted (faces retagged %d, bridges %d) | "
			"bonds: %d by tag, %d by contact | hulls: %d built, %d failed large, %d failed other | chips: %d ghosts into %d\n",
			s->mergeTried, s->mergeTooBig, s->mergePrerejected, s->mergeHulls, s->mergeAccepted, s->mergeRetagged,
			s->mergeBridges, s->bondsByTag, s->bondsByContact, s->hullsBuilt, s->hullFailsLarge, s->hullFailsOther,
			s->ghostsChipped, s->chipsMade );
	printf( "        cells out: " );
	lpPrintBins( "faces", s->faceBins, true, s->maxFaces );
	printf( "; " );
	lpPrintBins( "vertices", s->vertexBins, false, s->maxVertices );
	printf( "\n        job cpu ms: pattern %.0f, merge %.0f, hulls %.0f, bonds %.0f, chips %.0f\n", (double)s->voronoiMs,
			(double)s->mergeMs, (double)s->hullMs, (double)s->bondMs, (double)s->chipMs );
	if ( bf->haveParts )
	{
		printf( "        parts at build: %d (%d ground); all: ", bf->parts.parts, bf->parts.groundParts );
		lpPrintCensus( &bf->parts.all );
		printf( "\n                                 without the ground: " );
		lpPrintCensus( &bf->parts.noGround );
		printf( "\n" );
	}
	if ( bf->check == false )
	{
		return;
	}
	const lpFractureCheck* c = &bf->total;
	printf( "        checks of %d jobs (%.0f ms): %d exactness violations%s\n", c->jobs, bf->validateMs,
			lpFractureCheck_Violations( c ), c->validatorOverflows > 0 ? " (VALIDATOR OVERFLOWS)" : "" );
	printf( "          a tiling: %d jobs off, %d by more than 1e-3 (max relative error %.3g)\n", c->tilingViolations, c->tilingGaps,
			c->tilingMaxError );
	printf( "          b overlap: %d of %d pairs (max %.3g m^3, %.3g of the smaller cell; total %.3g m^3)\n", c->overlapViolations,
			c->overlapPairs, c->overlapMax, c->overlapMaxRelative, c->overlapTotal );
	printf( "          c siblings: %d cut faces: %d exact, %d covered (T-junctions), %d near (max distance %.3g m), %d unmatched "
			"(%.3g m^2, max %.3g m^2)\n",
			c->siblingFaces, c->siblingExact, c->siblingCovered, c->siblingNear, c->siblingMaxDistance, c->siblingUnmatched,
			c->siblingUnmatchedArea, c->siblingMaxUnmatchedArea );
	printf( "          d cells: %d invalid (%d degenerate faces), %d not convex (max excess %.3g m, max planarity %.3g m)\n",
			c->invalidCells, c->degenerateFaces, c->convexViolations, c->maxConvexExcess, c->maxPlanarity );
	printf( "          e merge: %d keepers, %d outside their cell (max excess %.3g m)\n", c->keepersChecked, c->containViolations,
			c->maxContainExcess );
	printf( "          f chips: %d sets, %d off (max tiling error %.3g, max overlap %.3g m^3), %d mismatches\n", c->chipSets,
			c->chipViolations, c->chipMaxTilingError, c->chipMaxOverlap, c->chipMismatches );
	printf( "          g census of %d cells: ", c->census.shapes );
	lpPrintCensus( &c->census );
	printf( "\n          worst jobs (replay with --job k): tiling #%d, overlap #%d, siblings unmatched #%d, near #%d, containment "
			"#%d, first invalid #%d\n",
			bf->worstTiling, bf->worstOverlap, bf->worstSibling, bf->worstNear, bf->worstContain, bf->firstInvalid );
	printf( "          by pattern (jobs: max tiling error, max overlap m^3, near sibling faces and their max distance m, max "
			"containment excess m):\n" );
	for ( int k = 0; k < 5; ++k )
	{
		const lpFractureCheck* p = bf->byPattern + k;
		printf( "            %-8s %5d: %.3g, %.3g, %d %.3g, %.3g\n", lp_patternNames[k], p->jobs, p->tilingMaxError, p->overlapMax,
				p->siblingNear, p->siblingMaxDistance, p->maxContainExcess );
	}
}

// ---- counts as JSON ----

typedef struct lpText
{
	char* data;
	int size;
	int length;
} lpText;

static void lpAppend( lpText* t, const char* format, ... )
{
	va_list args;
	va_start( args, format );
	int room = t->size - t->length;
	int n = vsnprintf( t->data != NULL && room > 0 ? t->data + t->length : NULL, room > 0 ? (size_t)room : 0, format, args );
	va_end( args );
	t->length += n > 0 ? n : 0;
}

static void lpAppendInts( lpText* t, const char* name, const int* values, int count, int stride )
{
	lpAppend( t, "\"%s\": [", name );
	for ( int k = 0; k < count; ++k )
	{
		lpAppend( t, "%s%d", k > 0 ? ", " : "", values[k * stride] );
	}
	lpAppend( t, "], " );
}

static void lpAppendCensus( lpText* t, const char* name, const lpShapeCensus* c )
{
	lpAppend( t,
			  "\"%s\": {\"shapes\": %d, \"maxCoordinate\": %.9g, \"beyondRange\": %d, \"minEdge\": %.9g, \"minFaceArea\": %.9g, "
			  "\"closePairs\": %d, \"gridCollisions\": %d, \"maxFaces\": %d, \"maxVertices\": %d}",
			  name, c->shapes, c->maxCoordinate, c->beyondRange, lpOrZero( c->minEdge ), lpOrZero( c->minFaceArea ), c->closePairs,
			  c->gridCollisions, c->maxFaces, c->maxVertices );
}

int lpBenchFractures_Counts( const lpBenchFractures* bf, char* text, int size )
{
	lpText t = { text, size, 0 };
	const lpFractureStats* s = &bf->stats;
	lpAppend( &t, "\"jobs\": %d, ", bf->jobs );
	lpAppendInts( &t, "patterns", bf->patterns, 5, 1 );
	lpAppend( &t, "\"sitesDrawn\": %d, \"sliversAbsorbed\": %d, \"cellsDropped\": %d, \"patternCells\": %d, \"outputCells\": %d, ",
			  s->sitesDrawn, s->sliversAbsorbed, s->cellsDropped, s->patternCells, s->outputCells );
	int clips[lp_clipCallerCount], shifts[lp_clipCallerCount], outs[lp_clipCallerCount], failures[lp_clipCallerCount];
	for ( int k = 0; k < lp_clipCallerCount; ++k )
	{
		clips[k] = s->clips[k].clips;
		shifts[k] = s->clips[k].shifts;
		outs[k] = s->clips[k].toleranceOuts;
		failures[k] = s->clips[k].failures;
	}
	lpAppend( &t, "\"clipCallers\": [\"voronoi\", \"grain\", \"radial\", \"masonry\", \"snap\", \"chips\"], " );
	lpAppendInts( &t, "clips", clips, lp_clipCallerCount, 1 );
	lpAppendInts( &t, "planeShifts", shifts, lp_clipCallerCount, 1 );
	lpAppendInts( &t, "toleranceOuts", outs, lp_clipCallerCount, 1 );
	lpAppendInts( &t, "clipFailures", failures, lp_clipCallerCount, 1 );
	lpAppend( &t, "\"maxShift\": %.9g, \"failureCount\": %d, ", (double)lpMaxShift( s ), s->failureCount );
	lpAppend( &t,
			  "\"mergeTried\": %d, \"mergeTooBig\": %d, \"mergePrerejected\": %d, \"mergeHulls\": %d, \"mergeAccepted\": %d, "
			  "\"mergeRetagged\": %d, \"mergeBridges\": %d, \"bondsByTag\": %d, \"bondsByContact\": %d, \"hullsBuilt\": %d, "
			  "\"hullFailsLarge\": %d, \"hullFailsOther\": %d, \"ghostsChipped\": %d, \"chipsMade\": %d, \"maxFaces\": %d, "
			  "\"maxVertices\": %d, ",
			  s->mergeTried, s->mergeTooBig, s->mergePrerejected, s->mergeHulls, s->mergeAccepted, s->mergeRetagged,
			  s->mergeBridges, s->bondsByTag, s->bondsByContact, s->hullsBuilt, s->hullFailsLarge, s->hullFailsOther,
			  s->ghostsChipped, s->chipsMade, s->maxFaces, s->maxVertices );
	lpAppendInts( &t, "faceBins", s->faceBins, LP_CELL_BINS, 1 );
	lpAppendInts( &t, "vertexBins", s->vertexBins, LP_CELL_BINS, 1 );
	if ( bf->haveParts )
	{
		lpAppend( &t, "\"parts\": %d, \"groundParts\": %d, ", bf->parts.parts, bf->parts.groundParts );
		lpAppendCensus( &t, "partsAll", &bf->parts.all );
		lpAppend( &t, ", " );
		lpAppendCensus( &t, "partsNoGround", &bf->parts.noGround );
		lpAppend( &t, ", " );
	}
	if ( bf->check )
	{
		const lpFractureCheck* c = &bf->total;
		lpAppend( &t, "\"violations\": %d, \"checkedJobs\": %d, \"checkedPatternCells\": %d, \"checkedOutputCells\": %d, ",
				  lpFractureCheck_Violations( c ), c->jobs, c->patternCells, c->outputCells );
		lpAppend( &t, "\"tilingViolations\": %d, \"tilingGaps\": %d, \"tilingMaxError\": %.9g, ", c->tilingViolations,
				  c->tilingGaps, c->tilingMaxError );
		lpAppend( &t,
				  "\"overlapPairs\": %d, \"overlapViolations\": %d, \"overlapMax\": %.9g, \"overlapMaxRelative\": %.9g, "
				  "\"overlapTotal\": %.9g, ",
				  c->overlapPairs, c->overlapViolations, c->overlapMax, c->overlapMaxRelative, c->overlapTotal );
		lpAppend( &t,
				  "\"siblingFaces\": %d, \"siblingExact\": %d, \"siblingCovered\": %d, \"siblingNear\": %d, \"siblingUnmatched\": %d, "
				  "\"siblingMaxDistance\": %.9g, \"siblingUnmatchedArea\": %.9g, \"siblingMaxUnmatchedArea\": %.9g, ",
				  c->siblingFaces, c->siblingExact, c->siblingCovered, c->siblingNear, c->siblingUnmatched, c->siblingMaxDistance,
				  c->siblingUnmatchedArea, c->siblingMaxUnmatchedArea );
		lpAppend( &t,
				  "\"invalidCells\": %d, \"degenerateFaces\": %d, \"convexViolations\": %d, \"maxConvexExcess\": %.9g, "
				  "\"maxPlanarity\": %.9g, ",
				  c->invalidCells, c->degenerateFaces, c->convexViolations, c->maxConvexExcess, c->maxPlanarity );
		lpAppend( &t, "\"keepersChecked\": %d, \"containViolations\": %d, \"maxContainExcess\": %.9g, ", c->keepersChecked,
				  c->containViolations, c->maxContainExcess );
		lpAppend( &t,
				  "\"chipSets\": %d, \"chipViolations\": %d, \"chipMaxTilingError\": %.9g, \"chipMaxOverlap\": %.9g, "
				  "\"chipMismatches\": %d, \"validatorOverflows\": %d, ",
				  c->chipSets, c->chipViolations, c->chipMaxTilingError, c->chipMaxOverlap, c->chipMismatches,
				  c->validatorOverflows );
		lpAppendCensus( &t, "census", &c->census );
		lpAppend( &t,
				  ", \"worstJobs\": {\"tiling\": %d, \"overlap\": %d, \"siblings\": %d, \"near\": %d, \"containment\": %d, "
				  "\"invalid\": %d}, ",
				  bf->worstTiling, bf->worstOverlap, bf->worstSibling, bf->worstNear, bf->worstContain, bf->firstInvalid );
		lpAppend( &t, "\"byPattern\": {" );
		for ( int k = 0; k < 5; ++k )
		{
			const lpFractureCheck* p = bf->byPattern + k;
			lpAppend( &t,
					  "%s\"%s\": {\"jobs\": %d, \"violations\": %d, \"tilingMaxError\": %.9g, \"overlapMax\": %.9g, \"siblingNear\": %d, "
					  "\"siblingUnmatched\": %d, \"siblingMaxDistance\": %.9g, \"maxContainExcess\": %.9g}",
					  k > 0 ? ", " : "", lp_patternNames[k], p->jobs, lpFractureCheck_Violations( p ), p->tilingMaxError, p->overlapMax,
					  p->siblingNear, p->siblingUnmatched, p->siblingMaxDistance, p->maxContainExcess );
		}
		lpAppend( &t, "}, " );
	}
	lpAppend( &t, "\"countsEnd\": true" );
	return t.length;
}

// ---- replay ----

typedef struct lpReplayJob
{
	int index;
	int tick;
	int pattern;
	int cells;
	float ms[5]; // best of n: pattern, merge, hulls, bonds, chips
	float total;
} lpReplayJob;

static int lpCompareSlowest( const void* a, const void* b )
{
	const lpReplayJob* x = a;
	const lpReplayJob* y = b;
	if ( x->total != y->total )
	{
		return x->total > y->total ? -1 : 1;
	}
	return ( x->index > y->index ) - ( x->index < y->index );
}

static uint32_t lpReadU32( const uint8_t* p )
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static uint64_t lpReadU64( const uint8_t* p )
{
	return (uint64_t)lpReadU32( p ) | (uint64_t)lpReadU32( p + 4 ) << 32;
}

int lpBenchReplay( const char* path, int onlyJob, int repeat, bool check )
{
	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		printf( "cannot read %s\n", path );
		return 1;
	}
	fseek( f, 0, SEEK_END );
	long fileSize = ftell( f );
	fseek( f, 0, SEEK_SET );
	if ( fileSize < 12 || fileSize > 0x7FFFFFFF )
	{
		printf( "%s: not a recording (%ld bytes)\n", path, fileSize );
		fclose( f );
		return 1;
	}
	int size = (int)fileSize;
	uint8_t* data = malloc( (size_t)size );
	size_t read = fread( data, 1, (size_t)size, f );
	fclose( f );
	uint32_t metaLength = read == (size_t)size ? lpReadU32( data + 8 ) : 0;
	if ( read != (size_t)size || memcmp( data, "LPFR", 4 ) != 0 || lpReadU32( data + 4 ) != LP_RECORDING_VERSION ||
		 metaLength > (uint32_t)size - 12u )
	{
		printf( "%s: not a recording of version %d\n", path, LP_RECORDING_VERSION );
		free( data );
		return 1;
	}
	repeat = repeat < 1 ? 1 : repeat;
	printf( "replay %s (%.*s), best of %d\n", path, (int)metaLength, (const char*)data + 12, repeat );

	int at = 12 + (int)metaLength;
	int count = 0, same = 0, differs = 0;
	bool broken = false;
	lpBenchFractures* bf = lpBenchFractures_Create( check );
	lpFractureJob* job = lpFractureJob_Create();
	int jobCapacity = 1024;
	lpReplayJob* jobs = malloc( sizeof( lpReplayJob ) * (size_t)jobCapacity );
	int jobCount = 0;
	double stageTotals[5] = { 0.0 };
	for ( int index = 0; at < size; ++index )
	{
		if ( size - at < 4 )
		{
			printf( "record %d: truncated\n", index );
			broken = true;
			break;
		}
		int snapshotSize = (int)lpReadU32( data + at );
		if ( snapshotSize <= 0 || snapshotSize > LP_JOB_SNAPSHOT_MAX || size - at - 4 < snapshotSize + 12 )
		{
			printf( "record %d: truncated or corrupt (a snapshot of %d bytes, %d left)\n", index, snapshotSize, size - at - 4 );
			broken = true;
			break;
		}
		const uint8_t* snapshot = data + at + 4;
		uint64_t recorded = lpReadU64( snapshot + snapshotSize );
		int tick = (int)lpReadU32( snapshot + snapshotSize + 8 );
		at += 4 + snapshotSize + 12;
		if ( onlyJob >= 0 && index != onlyJob )
		{
			continue;
		}
		if ( lpFractureJob_Read( job, snapshot, snapshotSize ) == false )
		{
			printf( "record %d: corrupt snapshot\n", index );
			broken = true;
			break;
		}

		lpReplayJob r = { index, tick, lpJobPattern( job ), 0, { 0.0f }, 0.0f };
		for ( int k = 0; k < 5; ++k )
		{
			r.ms[k] = 1e30f;
		}
		for ( int n = 0; n < repeat; ++n )
		{
			lpFracture_FreeJob( job );
			lpFracture_RunJob( job );
			float stages[5] = { job->stats.voronoiMs, job->stats.mergeMs, job->stats.hullMs, job->stats.bondMs, job->stats.chipMs };
			for ( int k = 0; k < 5; ++k )
			{
				r.ms[k] = stages[k] < r.ms[k] ? stages[k] : r.ms[k];
			}
		}
		uint64_t digest = lpFractureJob_Digest( job );
		bool match = digest == recorded;
		same += match ? 1 : 0;
		differs += match ? 0 : 1;
		if ( match == false && differs <= 10 )
		{
			printf( "record %d (tick %d): DIFFERS, digest %016llx, recorded %016llx\n", index, tick, (unsigned long long)digest,
					(unsigned long long)recorded );
		}
		r.cells = job->cellCount;
		for ( int k = 0; k < 5; ++k )
		{
			r.total += r.ms[k];
			stageTotals[k] += r.ms[k];
		}
		if ( jobCount == jobCapacity )
		{
			jobCapacity *= 2;
			jobs = realloc( jobs, sizeof( lpReplayJob ) * (size_t)jobCapacity );
		}
		jobs[jobCount++] = r;
		lpBenchFractures_Take( bf, job, index );
		if ( onlyJob >= 0 )
		{
			printf( "record %d (tick %d, %s): %d cells, %d bonds, digest %016llx (%s)\n", index, tick, lp_patternNames[r.pattern],
					job->cellCount, job->bondCount, (unsigned long long)digest, match ? "same" : "DIFFERS" );
		}
		lpFracture_FreeJob( job );
		count += 1;
	}

	printf( "%d jobs replayed: %d same, %d differ%s\n", count, same, differs, broken ? "; the file is TRUNCATED or CORRUPT" : "" );
	double total = stageTotals[0] + stageTotals[1] + stageTotals[2] + stageTotals[3] + stageTotals[4];
	printf( "stage ms (each job's best of %d, summed): pattern %.1f, merge %.1f, hulls %.1f, bonds %.1f, chips %.1f; total %.1f\n",
			repeat, stageTotals[0], stageTotals[1], stageTotals[2], stageTotals[3], stageTotals[4], total );
	qsort( jobs, (size_t)jobCount, sizeof( lpReplayJob ), lpCompareSlowest );
	for ( int i = 0; i < jobCount && i < 8 && onlyJob < 0; ++i )
	{
		const lpReplayJob* r = jobs + i;
		printf( "  slowest #%d (tick %d, %s, %d cells): %.3f ms (pattern %.3f, merge %.3f, hulls %.3f, bonds %.3f, chips %.3f)\n",
				r->index, r->tick, lp_patternNames[r->pattern], r->cells, (double)r->total, (double)r->ms[0], (double)r->ms[1],
				(double)r->ms[2], (double)r->ms[3], (double)r->ms[4] );
	}
	lpBenchFractures_Print( bf );
	int violations = check ? lpFractureCheck_Violations( &bf->total ) : 0;

	free( jobs );
	lpFractureJob_Destroy( job );
	lpBenchFractures_Free( bf );
	free( data );
	if ( differs > 0 )
	{
		return 2;
	}
	if ( broken || ( onlyJob >= 0 && count == 0 ) )
	{
		return 1;
	}
	return violations > 0 ? 5 : 0;
}
