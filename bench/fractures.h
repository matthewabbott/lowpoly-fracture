// SPDX-License-Identifier: MIT
// lpf_bench's fracture tools (milestone 11a), through the core's fracture hook (src/fcheck.h does the work):
//   --check-fractures         validate every fracture job of a run, and count what the jobs did
//   --record-fractures path   every job's snapshot and output digest, from the first worker count's run
//   --replay-fractures path   run recorded jobs alone, compare their digests, time their stages (--job k, --repeat n)
#pragma once

#include "fcheck.h"

#include <stdio.h>

typedef struct lpBenchFractures
{
	bool check;
	FILE* record;
	int tick;
	int jobs;		   // every job, those that left their piece whole too; a job's index here is its record's
	int patterns[5];   // jobs by pattern: impact, grain, radial, masonry, snap
	lpFractureStats stats;
	lpFractureCheck total;
	lpPartCensus parts; // the scene's parts as built
	bool haveParts;
	double validateMs;
	// The jobs to replay alone (--replay-fractures path --job k): the worst of each check, by its job index
	int worstTiling, worstOverlap, worstSibling, worstNear, worstContain, firstInvalid;
	double worstTilingError, worstOverlapVolume, worstSiblingArea, worstNearDistance, worstContainExcess;
	// The checks by pattern (as patterns above): jobs checked, the largest tiling error, overlap and sibling distance
	lpFractureCheck byPattern[5];
} lpBenchFractures;

// Installs the hook on a world for one run: validates each job when check, writes each to recordPath (NULL: none,
// meta: a line of text for the file's header)
lpBenchFractures* lpBenchFractures_Begin( lpWorld* world, bool check, const char* recordPath, const char* meta );
void lpBenchFractures_End( lpBenchFractures* bf, lpWorld* world ); // the hook off, the recording closed
void lpBenchFractures_Free( lpBenchFractures* bf );

// What the run's jobs did and what the checks found, printed under the run's row
void lpBenchFractures_Print( const lpBenchFractures* bf );

// The counts as a JSON object's members (no timings: they must agree between worker counts); returns the length
int lpBenchFractures_Counts( const lpBenchFractures* bf, char* text, int size );

// --replay-fractures: 0 when every job replayed the same (and with check, no violation); 2 when one differs, 5 on a
// violation, 1 when the file cannot be read or is truncated or corrupt
int lpBenchReplay( const char* path, int onlyJob, int repeat, bool check );

// --replay-fractures path --exact-voronoi (milestone 11a, C2's go/no-go): every impact and grain job's float pattern
// stage against the exact Voronoi stage (src/xvoronoi.h) on the same input, each job's best of n summed, with what the
// exact cells are worth (valid, tiling, twins), their faces and vertices beside the float cells', and where the exact
// stage spends its time. 0 when the exact cells check out, 5 when not, 1 when the file cannot be read.
int lpBenchExactVoronoi( const char* path, int onlyJob, int repeat );
