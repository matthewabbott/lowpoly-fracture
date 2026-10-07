// SPDX-License-Identifier: MIT
// lpf_bench --check-fractures, --record-fractures and --replay-fractures (fractures.h). The bench reaches into the
// core's internals here, as the tests do: the fracture job and its hook are not in lpf.h.

#include "fractures.h"

#include "world.h"
#include "xvoronoi.h"

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

// A recording read whole, and its records one by one
typedef struct lpRecording
{
	uint8_t* data;
	int size;
	int at; // the next record
	int metaLength;
} lpRecording;

// Prints why and returns false when the file is not a recording of this version
static bool lpRecording_Open( const char* path, lpRecording* rec )
{
	memset( rec, 0, sizeof( *rec ) );
	FILE* f = fopen( path, "rb" );
	if ( f == NULL )
	{
		printf( "cannot read %s\n", path );
		return false;
	}
	fseek( f, 0, SEEK_END );
	long fileSize = ftell( f );
	fseek( f, 0, SEEK_SET );
	if ( fileSize < 12 || fileSize > 0x7FFFFFFF )
	{
		printf( "%s: not a recording (%ld bytes)\n", path, fileSize );
		fclose( f );
		return false;
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
		return false;
	}
	*rec = (lpRecording){ data, size, 12 + (int)metaLength, (int)metaLength };
	return true;
}

// Record `index`: false at the end, or (with *broken set, and why printed) when the rest is truncated or corrupt
static bool lpRecording_Next( lpRecording* rec, int index, const uint8_t** snapshot, int* snapshotSize, uint64_t* digest,
							  int* tick, bool* broken )
{
	if ( rec->at >= rec->size )
	{
		return false;
	}
	if ( rec->size - rec->at < 4 )
	{
		printf( "record %d: truncated\n", index );
		*broken = true;
		return false;
	}
	int bytes = (int)lpReadU32( rec->data + rec->at );
	if ( bytes <= 0 || bytes > LP_JOB_SNAPSHOT_MAX || rec->size - rec->at - 4 < bytes + 12 )
	{
		printf( "record %d: truncated or corrupt (a snapshot of %d bytes, %d left)\n", index, bytes, rec->size - rec->at - 4 );
		*broken = true;
		return false;
	}
	*snapshot = rec->data + rec->at + 4;
	*snapshotSize = bytes;
	*digest = lpReadU64( *snapshot + bytes );
	*tick = (int)lpReadU32( *snapshot + bytes + 8 );
	rec->at += 4 + bytes + 12;
	return true;
}

int lpBenchReplay( const char* path, int onlyJob, int repeat, bool check )
{
	lpRecording rec;
	if ( lpRecording_Open( path, &rec ) == false )
	{
		return 1;
	}
	uint8_t* data = rec.data;
	repeat = repeat < 1 ? 1 : repeat;
	printf( "replay %s (%.*s), best of %d\n", path, rec.metaLength, (const char*)data + 12, repeat );

	int count = 0, same = 0, differs = 0;
	bool broken = false;
	lpBenchFractures* bf = lpBenchFractures_Create( check );
	lpFractureJob* job = lpFractureJob_Create();
	int jobCapacity = 1024;
	lpReplayJob* jobs = malloc( sizeof( lpReplayJob ) * (size_t)jobCapacity );
	int jobCount = 0;
	double stageTotals[5] = { 0.0 };
	const uint8_t* snapshot;
	int snapshotSize, tick;
	uint64_t recorded;
	for ( int index = 0; lpRecording_Next( &rec, index, &snapshot, &snapshotSize, &recorded, &tick, &broken ); ++index )
	{
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

// ---- the exact Voronoi stage against the float one (milestone 11a, C2's go/no-go) ----

typedef struct lpExactSide
{
	int jobs;
	int split[2];	  // jobs with two cells or more: float, exact
	int cells[2];	  // float pattern cells, exact cells
	double ms[2];	  // the float pattern stage and the exact stage, each job's best of n, summed
	double convertMs; // the parents made exact, best of n (C5's pieces are exact already: not in the stage)
	int convertFailures;
	int64_t parentFaces[2], parentVertices[2]; // the float parents' and the exact ones'
	uint64_t exactDigest, shapeDigest;		   // the exact cells and their float shapes, in job order
	int64_t faces[2], vertices[2]; // summed over the cells, for the means
	int maxFaces[2], maxVertices[2];
	int faceBins[2][LP_CELL_BINS], vertexBins[2][LP_CELL_BINS];
	lpXVoronoiStats stats;	 // the first run of each job: counts
	lpXVoronoiStats profile; // a run with timers per cell: the phases
	lpXVoronoiCheck check;
	// The operations' cost on the exact cells themselves: every vertex against every face plane of its cell, and every
	// vertex made again from its triple (with its doubles)
	double classifyMs, vertexMs;
	int64_t classifyCount, vertexCount;
} lpExactSide;

static void lpExactBins( lpExactSide* side, int k, int faces, int vertices )
{
	side->faces[k] += faces;
	side->vertices[k] += vertices;
	side->maxFaces[k] = faces > side->maxFaces[k] ? faces : side->maxFaces[k];
	side->maxVertices[k] = vertices > side->maxVertices[k] ? vertices : side->maxVertices[k];
	side->faceBins[k][lpCellBin( true, faces )] += 1;
	side->vertexBins[k][lpCellBin( false, vertices )] += 1;
}

static void lpExactAdd( lpExactSide* total, const lpExactSide* one )
{
	total->jobs += one->jobs;
	total->convertMs += one->convertMs;
	total->convertFailures += one->convertFailures;
	for ( int k = 0; k < 2; ++k )
	{
		total->parentFaces[k] += one->parentFaces[k];
		total->parentVertices[k] += one->parentVertices[k];
	}
	for ( int k = 0; k < 2; ++k )
	{
		total->split[k] += one->split[k];
		total->cells[k] += one->cells[k];
		total->ms[k] += one->ms[k];
		total->faces[k] += one->faces[k];
		total->vertices[k] += one->vertices[k];
		total->maxFaces[k] = one->maxFaces[k] > total->maxFaces[k] ? one->maxFaces[k] : total->maxFaces[k];
		total->maxVertices[k] = one->maxVertices[k] > total->maxVertices[k] ? one->maxVertices[k] : total->maxVertices[k];
		for ( int b = 0; b < LP_CELL_BINS; ++b )
		{
			total->faceBins[k][b] += one->faceBins[k][b];
			total->vertexBins[k][b] += one->vertexBins[k][b];
		}
	}
	const lpXVoronoiStats* s = &one->stats;
	lpXVoronoiStats* t = &total->stats;
	t->jobs += s->jobs;
	t->sitesDrawn += s->sitesDrawn;
	t->siteDuplicates += s->siteDuplicates;
	t->sitesOutside += s->sitesOutside;
	t->planeRejects += s->planeRejects;
	t->sliversAbsorbed += s->sliversAbsorbed;
	t->cellsEmpty += s->cellsEmpty;
	t->cellsDropped += s->cellsDropped;
	t->cells += s->cells;
	t->clip.clips += s->clip.clips;
	t->clip.unchanged += s->clip.unchanged;
	t->clip.cut += s->clip.cut;
	t->clip.empty += s->clip.empty;
	t->clip.overflows += s->clip.overflows;
	t->clip.touching += s->clip.touching;
	t->clip.classifications += s->clip.classifications;
	t->clip.vertices += s->clip.vertices;
	t->clip.repicked += s->clip.repicked;
	total->profile.sitesMs += one->profile.sitesMs;
	total->profile.clipMs += one->profile.clipMs;
	total->profile.roundMs += one->profile.roundMs;
	total->profile.shapeMs += one->profile.shapeMs;
	lpXVoronoiCheck* c = &total->check;
	c->jobs += one->check.jobs;
	c->cells += one->check.cells;
	c->invalid += one->check.invalid;
	c->firstInvalid = c->firstInvalid != NULL ? c->firstInvalid : one->check.firstInvalid;
	c->tilingViolations += one->check.tilingViolations;
	c->tilingMaxError = one->check.tilingMaxError > c->tilingMaxError ? one->check.tilingMaxError : c->tilingMaxError;
	c->cutFaces += one->check.cutFaces;
	c->twins += one->check.twins;
	c->unmatched += one->check.unmatched;
	c->floatMismatches += one->check.floatMismatches;
	total->exactDigest = lpMix64( total->exactDigest ^ lpMix64( one->exactDigest + 1u ) );
	total->shapeDigest = lpMix64( total->shapeDigest ^ lpMix64( one->shapeDigest + 1u ) );
	total->classifyMs += one->classifyMs;
	total->vertexMs += one->vertexMs;
	total->classifyCount += one->classifyCount;
	total->vertexCount += one->vertexCount;
}

static void lpPrintExactBins( const int* bins )
{
	printf( "[" );
	for ( int k = 0; k < LP_CELL_BINS; ++k )
	{
		printf( "%s%d", k > 0 ? ", " : "", bins[k] );
	}
	printf( "]" );
}

static void lpPrintExactSide( const char* name, const lpExactSide* side )
{
	const lpXVoronoiStats* s = &side->stats;
	const lpXClipStats* x = &s->clip;
	printf( "  %s: %d jobs (split: float %d, exact %d), cells float %d, exact %d\n", name, side->jobs, side->split[0],
			side->split[1], side->cells[0], side->cells[1] );
	printf( "    stage ms (each job's best, summed): float %.2f, exact %.2f, ratio %.2f; parents made exact %.2f ms (%d "
			"failed)\n",
			side->ms[0], side->ms[1], side->ms[0] > 0.0 ? side->ms[1] / side->ms[0] : 0.0, side->convertMs,
			side->convertFailures );
	printf( "    parents: float %lld faces, %lld vertices; exact %lld faces, %lld vertices\n", (long long)side->parentFaces[0],
			(long long)side->parentVertices[0], (long long)side->parentFaces[1], (long long)side->parentVertices[1] );
	for ( int k = 0; k < 2; ++k )
	{
		int cells = side->cells[k] > 0 ? side->cells[k] : 1;
		printf( "    %s cells: faces mean %.1f max %d ", k == 0 ? "float" : "exact", (double)side->faces[k] / cells,
				side->maxFaces[k] );
		lpPrintExactBins( side->faceBins[k] );
		printf( "; vertices mean %.1f max %d ", (double)side->vertices[k] / cells, side->maxVertices[k] );
		lpPrintExactBins( side->vertexBins[k] );
		printf( "\n" );
	}
	printf( "    exact clips %lld (unchanged %lld, cut %lld, empty %lld, overflows %lld; %lld cuts touching a vertex, %lld "
			"triples re-picked), %lld vertices classified (%.1f a clip), %lld made (%.2f a clip)\n",
			(long long)x->clips, (long long)x->unchanged, (long long)x->cut, (long long)x->empty, (long long)x->overflows,
			(long long)x->touching, (long long)x->repicked, (long long)x->classifications,
			x->clips > 0 ? (double)x->classifications / (double)x->clips : 0.0, (long long)x->vertices,
			x->clips > 0 ? (double)x->vertices / (double)x->clips : 0.0 );
	printf( "    sites: %d drawn, %d duplicates, %d outside; %d plane rejects, %d slivers absorbed, %d cells empty, %d "
			"dropped\n",
			s->sitesDrawn, s->siteDuplicates, s->sitesOutside, s->planeRejects, s->sliversAbsorbed, s->cellsEmpty,
			s->cellsDropped );
	const lpXVoronoiStats* p = &side->profile;
	double nsClassify = side->classifyCount > 0 ? 1e6 * side->classifyMs / (double)side->classifyCount : 0.0;
	double nsVertex = side->vertexCount > 0 ? 1e6 * side->vertexMs / (double)side->vertexCount : 0.0;
	double classifyMs = 1e-6 * nsClassify * (double)x->classifications;
	double vertexMs = 1e-6 * nsVertex * (double)x->vertices;
	double phases = p->sitesMs + p->clipMs + p->roundMs + p->shapeMs;
	printf( "    phases (a run with timers, %.2f ms): sites %.2f, clips %.2f (classification about %.2f at %.1f ns, new "
			"vertices about %.2f at %.1f ns, the rest %.2f), rounding %.2f, mass and shapes %.2f\n",
			phases, p->sitesMs, p->clipMs, classifyMs, nsClassify, vertexMs, nsVertex, p->clipMs - classifyMs - vertexMs,
			p->roundMs, p->shapeMs );
	const lpXVoronoiCheck* c = &side->check;
	printf( "    checks: %d cells, %d invalid%s%s; tiling max error %.3g (%d jobs past %.0e); %d cut faces: %d twins, %d "
			"unmatched, %d float mismatches\n",
			c->cells, c->invalid, c->firstInvalid != NULL ? ", first: " : "", c->firstInvalid != NULL ? c->firstInvalid : "",
			c->tilingMaxError, c->tilingViolations, LP_CHECK_TILING, c->cutFaces, c->twins, c->unmatched, c->floatMismatches );
	printf( "    digests: exact cells %016llx, their float shapes %016llx\n", (unsigned long long)side->exactDigest,
			(unsigned long long)side->shapeDigest );
}

int lpBenchExactVoronoi( const char* path, int onlyJob, int repeat )
{
	lpRecording rec;
	if ( lpRecording_Open( path, &rec ) == false )
	{
		return 1;
	}
	repeat = repeat < 1 ? 1 : repeat;
	printf( "exact voronoi against float (milestone 11a, C2) on %s (%.*s), best of %d, int128 path %s\n", path,
			rec.metaLength, (const char*)rec.data + 12, repeat, lpI128_Path() );

	lpFractureJob* job = lpFractureJob_Create();
	lpXVoronoiWork* work = malloc( sizeof( lpXVoronoiWork ) );
	lpXPoly* parents = malloc( 2 * sizeof( lpXPoly ) );
	lpExactSide* sides = calloc( 2, sizeof( lpExactSide ) ); // impact, grain
	lpShape* cells[LP_MAX_SITES];
	int cellSites[LP_MAX_SITES];
	int others = 0, count = 0;
	bool broken = false;
	const uint8_t* snapshot;
	int snapshotSize, tick;
	uint64_t recorded;
	for ( int index = 0; lpRecording_Next( &rec, index, &snapshot, &snapshotSize, &recorded, &tick, &broken ); ++index )
	{
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
		count += 1;
		int pattern = lpJobPattern( job );
		if ( pattern != 0 && pattern != 1 )
		{
			others += 1;
			continue;
		}
		lpExactSide* side = sides + pattern;
		side->jobs += 1;

		// The float pattern stage (what voronoiMs measures in a job)
		float best = 1e30f;
		int floatCells = 0;
		for ( int n = 0; n < repeat; ++n )
		{
			uint64_t ticks = lpGetTicks();
			int made = lpFracture( &job->input, cells, cellSites, LP_MAX_SITES, NULL );
			float ms = lpGetMilliseconds( ticks );
			best = ms < best ? ms : best;
			if ( n == 0 )
			{
				floatCells = made;
				side->cells[0] += made;
				side->split[0] += made >= 2 ? 1 : 0;
				for ( int c = 0; c < made; ++c )
				{
					lpExactBins( side, 0, cells[c]->faceCount, cells[c]->vertexCount );
				}
			}
			for ( int c = 0; c < made; ++c )
			{
				lpShape_Destroy( cells[c] );
			}
		}
		side->ms[0] += best;

		// The parent made exact, in the object frame
		best = 1e30f;
		lpXBuild built = lp_xBuilt;
		for ( int n = 0; n < repeat; ++n )
		{
			uint64_t ticks = lpGetTicks();
			built = lpXPoly_FromPoly( &job->poly, job->center, parents, parents + 1 );
			float ms = lpGetMilliseconds( ticks );
			best = ms < best ? ms : best;
		}
		side->convertMs += best;
		side->parentFaces[0] += job->poly.faceCount;
		side->parentVertices[0] += job->poly.vertexCount;
		side->parentFaces[1] += built == lp_xBuilt ? parents->faceCount : 0;
		side->parentVertices[1] += built == lp_xBuilt ? parents->vertexCount : 0;
		if ( built != lp_xBuilt )
		{
			printf( "record %d: the parent made exact failed (%d)\n", index, (int)built );
			side->convertFailures += 1;
			continue;
		}

		// The exact stage
		best = 1e30f;
		int made = 0;
		for ( int n = 0; n < repeat; ++n )
		{
			lpXVoronoiStats ignored = { 0 };
			uint64_t ticks = lpGetTicks();
			made = lpXVoronoi_Run( parents, &job->input, job->center, work, cells, cellSites, LP_MAX_SITES,
								   n == 0 ? &side->stats : &ignored );
			float ms = lpGetMilliseconds( ticks );
			best = ms < best ? ms : best;
			for ( int c = 0; c < made; ++c )
			{
				lpShape_Destroy( cells[c] );
			}
		}
		side->ms[1] += best;
		side->cells[1] += made;
		side->split[1] += made >= 2 ? 1 : 0;
		for ( int c = 0; c < made; ++c )
		{
			lpExactBins( side, 1, work->cells[c].faceCount, work->cells[c].vertexCount );
		}

		// What the cells are worth, and what their operations cost
		lpXVoronoi_Check( parents, work, made, cellSites, &side->check );
		for ( int c = 0; c < made; ++c )
		{
			uint64_t d = lpXPoly_Digest( work->cells + c );
			side->exactDigest = lpHashWords( side->exactDigest, &d, sizeof( d ) );
		}
		int sum = 0;
		uint64_t ticks = lpGetTicks();
		for ( int c = 0; c < made; ++c )
		{
			const lpXPoly* cell = work->cells + c;
			for ( int v = 0; v < cell->vertexCount; ++v )
			{
				for ( int f = 0; f < cell->faceCount; ++f )
				{
					sum += lpIVertex_Classify( cell->vertices + v, &cell->faces[f].plane );
				}
			}
			side->classifyCount += (int64_t)cell->vertexCount * cell->faceCount;
		}
		side->classifyMs += lpGetMilliseconds( ticks );
		ticks = lpGetTicks();
		double approx = 0.0;
		for ( int c = 0; c < made; ++c )
		{
			const lpXPoly* cell = work->cells + c;
			for ( int v = 0; v < cell->vertexCount; ++v )
			{
				const uint8_t* t = cell->triples[v];
				lpIVertex again;
				if ( lpIVertex_FromPlanes( &cell->faces[t[0]].plane, &cell->faces[t[1]].plane, &cell->faces[t[2]].plane,
										   &again ) )
				{
					double w = lpI128_ToDouble( again.w );
					approx += lpI128_ToDouble( again.x ) / w + lpI128_ToDouble( again.y ) / w + lpI128_ToDouble( again.z ) / w;
				}
			}
			side->vertexCount += cell->vertexCount;
		}
		side->vertexMs += lpGetMilliseconds( ticks );
		if ( sum == 1 && approx == 0.5 )
		{
			printf( "(never)\n" ); // keeps the measured work alive
		}

		// A run with timers per cell, for the phases
		side->profile.profile = true;
		int again = lpXVoronoi_Run( parents, &job->input, job->center, work, cells, cellSites, LP_MAX_SITES, &side->profile );
		for ( int c = 0; c < again; ++c )
		{
			side->shapeDigest = lpHashWords( side->shapeDigest, &cells[c]->digest, sizeof( cells[c]->digest ) );
			lpShape_Destroy( cells[c] );
		}
		if ( onlyJob >= 0 )
		{
			printf( "record %d (tick %d, %s): float %d cells, exact %d; exact parent %d faces, %d vertices (float %d, %d)\n",
					index, tick, lp_patternNames[pattern], floatCells, made, parents->faceCount, parents->vertexCount,
					job->poly.faceCount, job->poly.vertexCount );
		}
	}

	printf( "%d jobs read (%d of other patterns)%s\n", count, others, broken ? "; the file is TRUNCATED or CORRUPT" : "" );
	lpPrintExactSide( "impact", sides );
	lpPrintExactSide( "grain (plain bisectors on the exact side: not the float's cells)", sides + 1 );
	lpExactSide* total = calloc( 1, sizeof( lpExactSide ) );
	lpExactAdd( total, sides );
	lpExactAdd( total, sides + 1 );
	lpPrintExactSide( "impact and grain", total );
	int violations = total->check.invalid + total->check.tilingViolations + total->check.unmatched +
					 total->check.floatMismatches + (int)total->stats.clip.overflows;

	free( total );
	free( sides );
	free( parents );
	free( work );
	lpFractureJob_Destroy( job );
	free( rec.data );
	if ( broken )
	{
		return 1;
	}
	return violations > 0 ? 5 : 0;
}
