// SPDX-License-Identifier: MIT
// The exact Voronoi stage (milestone 11a, C2's go/no-go): a prototype beside the float pattern, not wired into the
// engine. From a fracture job's input and its parent made exact (lpXPoly_FromPoly, in the object frame):
// - today's sites (the same draws: lpFracture_VoronoiSites) rounded to the grid, de-duplicated, and kept only strictly
//   inside the parent (exactly, in int64);
// - each cell the parent clipped exactly by its bisectors (lpIPlane_MakeBisector, tagged with the neighbour's site),
//   neighbours nearest first by exact int64 distance (index on ties), with the float loop's early out made exact: the
//   next bisector is skipped, and the rest with it, once it lies beyond the cell's reach, which is the largest
//   distance from the site to a vertex (from the doubles, within 2^-50) plus a whole grid unit of margin;
// - each cell's vertices rounded to floats, its mass in doubles from them, and its shape made as the float pattern
//   makes it; slivers absorbed as the float pattern absorbs them (up to three passes).
// Grain jobs use plain bisectors of the sites unsquashed (the metric is C5's), so their cells are not the float ones.

#pragma once

#include "fracture.h"
#include "xpoly.h"

typedef struct lpXVoronoiStats
{
	int jobs;
	int sitesDrawn;		// the float draws (the first pass)
	int siteDuplicates; // rounded onto an earlier site
	int sitesOutside;	// not strictly inside the exact parent
	int planeRejects;	// bisectors the constructor refused (never within an object)
	int sliversAbsorbed;
	int cellsEmpty;	  // never, for a site strictly inside
	int cellsDropped; // a shape refused (a float volume not positive)
	int cells;		  // what the stage returned
	int maxFaces;
	int maxVertices;
	int faceBins[LP_CELL_BINS];
	int vertexBins[LP_CELL_BINS];
	lpXClipStats clip;

	// The phases (when profile is set; timers per cell): sites (draw, rounding, sorts), clips, rounding to floats, mass
	// and shape
	bool profile;
	double sitesMs;
	double clipMs;
	double roundMs;
	double shapeMs;
} lpXVoronoiStats;

// The stage's room, about 1.8 MB: allocate it once
typedef struct lpXVoronoiWork
{
	lpXPoly scratch;
	lpXPoly cells[LP_MAX_SITES]; // the exact cells the stage returned, in its order
	int32_t sites[LP_MAX_SITES][3];
	int siteCount;
	lpVec3 rounded[LP_XPOLY_MAX_VERTICES];
	lpPoly poly;
} lpXVoronoiWork;

// As lpFracture for the Voronoi pattern: writes up to `capacity` shapes (caller owns them, in the object frame) and their
// sites, and returns the count (0 with fewer than two sites). origin: the job frame's origin in the object frame (the
// job's centre). Counted into stats.
int lpXVoronoi_Run( const lpXPoly* parent, const lpFractureInput* input, lpVec3 origin, lpXVoronoiWork* work, lpShape** cells,
					int* cellSites, int capacity, lpXVoronoiStats* stats );

// What the exact cells of a run are worth (the go/no-go's G5)
typedef struct lpXVoronoiCheck
{
	int jobs;
	int cells;
	int invalid; // cells lpXPoly_Validate refuses
	const char* firstInvalid;
	int tilingViolations; // jobs whose cells' volumes (precise) are off the parent's by more than LP_CHECK_TILING
	double tilingMaxError; // relative
	int cutFaces;		   // faces on a bisector
	int twins;			   // each with its neighbour's face on the exactly negated plane, holding the same points
	int unmatched;
	int floatMismatches; // twin points that rounded to different floats (never: one point, one float)
} lpXVoronoiCheck;

// Checks the cells of the last run (work->cells, count and cellSites as it returned them) against the parent
void lpXVoronoi_Check( const lpXPoly* parent, const lpXVoronoiWork* work, int count, const int* cellSites,
					   lpXVoronoiCheck* check );
