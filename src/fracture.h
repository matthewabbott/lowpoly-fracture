// SPDX-License-Identifier: MIT
// Fracture patterns: split one convex piece into convex cells.
//
// Every cell is a pure function of the fracture input and its cell index, so cells can be computed in
// any order on any thread and the result is bit-identical.

#pragma once

#include "poly.h"

typedef enum lpPattern
{
	// Voronoi cells, dense at the impact and coarse away from it (stone, brick, concrete, plaster)
	lp_patternImpact,

	// Voronoi computed in a space squashed along the grain, so cells come out as long splinters (wood)
	lp_patternGrain,

	// Radial wedges cut by concentric chords around the impact, in the plane of a thin pane (glass)
	lp_patternRadial,
} lpPattern;

typedef struct lpFractureInput
{
	// Piece to split, in a frame near the piece (for precision)
	const lpPoly* parent;

	b3Vec3 impact;		// impact point in the parent frame
	float radius;		// damage radius
	float fragmentSize; // edge length of the smallest fragments, near the impact
	float plateSize;	// spacing of the few large cells away from the impact
	int maxCells;
	float absorbVolume; // cells smaller than this outside the damage radius are slivers and get absorbed (0: keep)

	lpPattern pattern;
	b3Vec3 axis;	 // grain axis (grain pattern) or pane normal (radial pattern), unit length
	float stretch;	 // grain pattern: how much longer cells are along the grain (e.g. 4)

	uint8_t interiorMaterial;
	uint64_t seed;
	float tolerance;
} lpFractureInput;

typedef struct lpFractureStats
{
	int siteCount;
	int clipCount;
	int failureCount;
} lpFractureStats;

#define LP_MAX_SITES 128

// Split the parent. Writes up to `capacity` heap-allocated cell shapes (caller owns them) and returns the
// count. Returns 0 when the parent should not split (too few sites). Cells tile the parent exactly.
// cellSites (optional) receives the Voronoi site of each cell, or -1 for patterns without sites.
int lpFracture( const lpFractureInput* input, lpShape** cells, int* cellSites, int capacity, lpFractureStats* stats );

// A face two cells of one fracture share
typedef struct lpCellBond
{
	int a, b; // cell indices, a < b
	float area;
	b3Vec3 centroid;
} lpCellBond;

// Shared faces between the cells of one fracture. Voronoi cells share exact faces, found through the face tags (a
// lookup, no polygon clipping); other patterns fall back to lpShape_ContactArea.
int lpFindCellBonds( lpShape* const* cells, const int* cellSites, int count, lpCellBond* bonds, int capacity );

// Building blocks, exposed for tests and benchmarks ------------------------------------------------------

// Sites inside the parent: dense near the impact (density ~ 1/distance, the ejecta), 4-6 ring sites around the
// damage radius that shape a jagged rim, and at most three far sites that keep the rest of the piece in large plates.
typedef struct lpSiteParams
{
	b3Vec3 impact;
	float radius;
	float fragmentSize;
	float plateSize;
	int maxSites;
	b3Vec3 avoidAxis; // ring sites stay out of a 25 degree cone around this axis (the grain); zero for none
} lpSiteParams;

int lpGenerateImpactSites( const lpPoly* parent, const lpSiteParams* params, lpRandom* rng, b3Vec3* sites );

// Voronoi cell of site `index`, clipped to the parent. Returns the cell in `out`. Returns false if the
// cell is empty or a clip failed. Neighbor sites are visited nearest first, and the search stops once the
// next site is farther than twice the current cell radius (no further plane can cut).
bool lpComputeVoronoiCell( const lpPoly* parent, const b3Vec3* sites, int siteCount, int index, uint8_t material,
						   float tolerance, lpPoly* scratch, lpPoly* out, lpFractureStats* stats );
