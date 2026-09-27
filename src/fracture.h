// SPDX-License-Identifier: MIT
// Fracture patterns: split one convex piece into convex cells.
//
// Every cell is a pure function of the fracture input and its cell index, so cells can be computed in
// any order on any thread and the result is bit-identical.

#pragma once

#include "poly.h"

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

	lpPatternId pattern;
	b3Vec3 axis;	 // grain axis (grain pattern) or pane normal (radial pattern), unit length
	float stretch;	 // grain pattern: how much longer cells are along the grain (e.g. 4)

	uint8_t interiorMaterial;
	uint64_t seed;
	float tolerance;
} lpFractureInput;

typedef struct lpFractureStats
{
	int failureCount;
	float voronoiMs; // CPU time of a fracture job's stages (impact.c), measured and never branched on
	float mergeMs;
	float hullMs;
} lpFractureStats;

#define LP_MAX_SITES 128

// Split the parent. Writes up to `capacity` heap-allocated cell shapes (caller owns them) and returns the
// count. Returns 0 when the parent should not split (too few sites). Cells tile the parent exactly.
// cellSites (optional) receives the Voronoi site of each cell, or -1 for patterns without sites.
int lpFracture( const lpFractureInput* input, lpShape** cells, int* cellSites, int capacity, lpFractureStats* stats );

// A face two cells of one fracture share
typedef struct lpCellBond
{
	int a, b;		   // cell indices, a < b
	lpContact contact; // normal points from cell a to cell b
} lpCellBond;

// Shared faces between the cells of one fracture. Voronoi cells share exact faces, found through the face tags (a
// lookup, no polygon clipping); other patterns fall back to lpShape_Contact.
int lpFindCellBonds( lpShape* const* cells, const int* cellSites, int count, lpCellBond* bonds, int capacity );

// Merges touching Voronoi cells of class `mergeClass` while the convex hull of their union stays within (1 + slack)
// of the volume the cells really had: fewer, chunkier pieces for the same look (a broken log end is one piece, not
// four). The hull spills into neighbouring cells' space by at most the slack, and never over the centroid of a cell
// of another class (the ejecta), so knocked-out notches stay open. Hull faces keep the tag and material of
// a source face in their plane; the rest become cut faces. Compacts cells, cellSites and classes in place and
// returns the new count. Deterministic: pairs are tried in cell and face order.
int lpMergeCells( lpShape** cells, int* cellSites, uint8_t* classes, int count, uint8_t mergeClass, float slack,
				  uint8_t interiorMaterial );

// Splits a small ejecta cell into up to 1 + splits chips with random planes near its centroid: a blast throws a
// dirtier spray of real fragments than the Voronoi budget alone gives, at the price of a few plane clips. With a
// grain axis the planes contain it, so wood chips stay long splinters. Chips tile the cell and their new faces are
// cut faces. Writes new shapes (caller owns them) and returns their count; 0 means the cell was left whole.
int lpChipCell( const lpShape* cell, int splits, b3Vec3 grainAxis, uint8_t material, float minVolume, lpRandom* rng,
				lpShape** chips, int capacity );
