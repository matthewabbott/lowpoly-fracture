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

	lpVec3 impact;		// impact point in the parent frame
	float radius;		// damage radius
	float fragmentSize; // edge length of the smallest fragments, near the impact
	float plateSize;	// spacing of the few large cells away from the impact
	int maxCells;
	float absorbVolume; // cells smaller than this outside the damage radius are slivers and get absorbed (0: keep)

	lpPatternId pattern;
	lpVec3 axis;	 // grain axis (grain pattern) or pane normal (radial pattern), unit length
	float stretch;	 // grain pattern: how much longer cells are along the grain (e.g. 4)

	uint8_t interiorMaterial;
	uint64_t seed;
	float tolerance;

	bool snap; // overloaded beam: one tilted cut across `axis` through `impact` instead of the pattern

	// Masonry pattern: courses and bricks of this size, bricks running along `axis`, the grid anchored at gridOrigin
	// (the object frame's origin in the parent's frame) so every piece of a wall shares it
	float courseHeight;
	float brickLength;
	lpVec3 gridOrigin;
} lpFractureInput;

// Who called the float clip, for its stats
typedef enum lpClipCaller
{
	lp_clipVoronoi,
	lp_clipGrain,
	lp_clipRadial,
	lp_clipMasonry,
	lp_clipSnap,
	lp_clipChips, // ejecta chips, and the chips of bricks near a masonry hit
	lp_clipCallerCount
} lpClipCaller;

// Output cells by face count (up to 6, 8, 12, 16, 24, 32, 48, and more) and by vertex count (up to 8, 12, 16, 24, 32, 48,
// 64, and more)
#define LP_CELL_BINS 8

// The upper limit of a bin (the last has none: INT_MAX)
static inline int lpCellBinLimit( bool faces, int bin )
{
	static const int faceLimits[LP_CELL_BINS] = { 6, 8, 12, 16, 24, 32, 48, INT_MAX };
	static const int vertexLimits[LP_CELL_BINS] = { 8, 12, 16, 24, 32, 48, 64, INT_MAX };
	return faces ? faceLimits[bin] : vertexLimits[bin];
}

static inline int lpCellBin( bool faces, int count )
{
	int bin = 0;
	while ( count > lpCellBinLimit( faces, bin ) )
	{
		bin += 1;
	}
	return bin;
}

// What a fracture job did. Counted and timed, never hashed and never branched on: deterministic counts (milestone
// 11a's before-numbers, lpf_bench --check-fractures), CPU times that are not.
typedef struct lpFractureStats
{
	int failureCount; // clips that failed: the cell skipped that plane (or the cut was not made)
	lpClipStats clips[lp_clipCallerCount];

	// The pattern
	int sitesDrawn;		  // Voronoi sites placed (the first pass)
	int sliversAbsorbed;  // sites dropped because their cell was a sliver outside the damage radius
	int cellsDropped;	  // cells of the last pass that came out empty or degenerate
	int patternCells;	  // what the pattern returned

	// The keeper merge (lpMergeCells)
	int mergeTried;		  // pairs tried
	int mergeTooBig;	  // with more vertices together than a polyhedron holds
	int mergePrerejected; // rejected by the hull volume's lower bound, without a quickhull
	int mergeHulls;		  // quickhulls built
	int mergeAccepted;
	int mergeRetagged; // faces of accepted hulls that took a source face's tag by the 0.9999 rule
	int mergeBridges;  // faces of accepted hulls that found none (cut faces across the filled-in gap)

	// Bonds between the cells (lpFindCellBonds)
	int bondsByTag;		// through Voronoi face tags
	int bondsByContact; // through the lpShape_Contact fallback

	// Physics hulls of the cells that need one; a cell that gets none turns to dust
	int hullsBuilt;
	int hullFailsLarge; // the cell had more edges than Box3D's hull holds (128)
	int hullFailsOther; // degenerate

	// Chips
	int ghostsChipped;
	int chipsMade;

	// Output cells
	int outputCells;
	int maxFaces;
	int maxVertices;
	int faceBins[LP_CELL_BINS];
	int vertexBins[LP_CELL_BINS];

	float voronoiMs; // CPU time of a fracture job's stages (impact.c), measured and never branched on
	float mergeMs;
	float hullMs;
	float bondMs;
	float chipMs;
} lpFractureStats;

#define LP_MAX_SITES 128

// Split the parent. Writes up to `capacity` heap-allocated cell shapes (caller owns them) and returns the
// count. Returns 0 when the parent should not split (too few sites). Cells tile the parent exactly.
// cellSites (optional) receives the Voronoi site of each cell, or -1 for patterns without sites.
int lpFracture( const lpFractureInput* input, lpShape** cells, int* cellSites, int capacity, lpFractureStats* stats );

// The sites the Voronoi pattern (impact and grain) draws for its first pass with a capacity of LP_MAX_SITES, in the
// parent's frame (a grain's unsquashed back from the frame it is drawn in): milestone 11a's exact prototype
// (xvoronoi.h) starts from them. Writes up to LP_MAX_SITES sites and returns the count.
int lpFracture_VoronoiSites( const lpFractureInput* input, lpVec3* sites );

// A face two cells of one fracture share
typedef struct lpCellBond
{
	int a, b;		   // cell indices, a < b
	lpContact contact; // normal points from cell a to cell b
} lpCellBond;

// Shared faces between the cells of one fracture. Voronoi cells share exact faces, found through the face tags (a
// lookup, no polygon clipping); other patterns fall back to lpShape_Contact. Counted into stats (NULL: not).
int lpFindCellBonds( lpShape* const* cells, const int* cellSites, int count, lpCellBond* bonds, int capacity,
					 lpFractureStats* stats );

// Merges touching Voronoi cells of class `mergeClass` while the convex hull of their union stays within (1 + slack)
// of the volume the cells really had: fewer, chunkier pieces for the same look (a broken log end is one piece, not
// four). The hull spills into neighbouring cells' space by at most the slack, and never over the centroid of a cell
// of another class (the ejecta) or over the impact point itself, so knocked-out notches stay open and a snapped
// beam stays snapped. Hull faces keep the tag and material of
// a source face in their plane; the rest become cut faces. Compacts cells, cellSites and classes in place and
// returns the new count. Deterministic: pairs are tried in cell and face order. Counted into stats (NULL: not).
int lpMergeCells( lpShape** cells, int* cellSites, uint8_t* classes, int count, uint8_t mergeClass, float slack,
				  uint8_t interiorMaterial, lpVec3 impact, lpFractureStats* stats );

// Splits a small ejecta cell into up to 1 + splits chips with random planes near its centroid: a blast throws a
// dirtier spray of real fragments than the Voronoi budget alone gives, at the price of a few plane clips. With a
// grain axis the planes contain it, so wood chips stay long splinters. Chips tile the cell and their new faces are
// cut faces. Writes new shapes (caller owns them) and returns their count; 0 means the cell was left whole. Its
// clips are counted into stats (NULL: not).
int lpChipCell( const lpShape* cell, int splits, lpVec3 grainAxis, uint8_t material, float minVolume, lpRandom* rng,
				lpShape** chips, int capacity, lpFractureStats* stats );

// Fate of a cell after a fracture
typedef enum lpCellClass
{
	lp_cellKeep,  // stays on the parent body, bonded to its neighbours
	lp_cellPuff,  // particles only
	lp_cellGhost, // ejected as a ghost (no physics body)
	lp_cellLight, // ejected as a light debris body
	lp_cellFull,  // ejected as a full debris body
} lpCellClass;

// What a loose piece of this volume becomes, against its material's tier volumes (lpParticleVolume and the rest):
// particles below the first, a ghost below the second, light debris below the third, full debris above
static inline uint8_t lpLooseClass( float volume, float particle, float ghost, float light )
{
	return volume < particle ? lp_cellPuff : ( volume < ghost ? lp_cellGhost : ( volume < light ? lp_cellLight : lp_cellFull ) );
}

#define LP_MAX_CELL_BONDS ( LP_MAX_SITES * 24 )

// One piece to fracture during an impact: impact.c snapshots its input (phase 1), lpFracture_RunJob computes its
// cells, their fates, hulls and sibling bonds without the world (phase 2, in parallel), impact.c integrates them (3)
typedef struct lpFractureJob
{
	int piece;
	lpVec3 localImpact; // body frame
	lpVec3 center;		// piece centroid; the fracture runs in a frame centered here
	lpImpactDef impact; // what broke it: new bonds between its cells start with the damage it did there
	lpPoly poly;
	lpFractureInput input;
	float particleVolume; // tier thresholds of the piece's material, scaled
	float ghostVolume;
	float lightVolume;
	float mergeSlack; // and its merge slack and chip splits: the job runs without the world
	int chipSplits;

	int cellCount;
	lpShape* cells[LP_MAX_SITES];
	int cellSites[LP_MAX_SITES];
	uint8_t cellClass[LP_MAX_SITES];
	lpPhysHull* hulls[LP_MAX_SITES];
	int bondCount;
	lpCellBond* bonds; // LP_MAX_CELL_BONDS
	lpFractureStats stats;
} lpFractureJob;

// Phase 2 of an impact: the cells, each one's fate, the physics hulls of the cells that need one, and the bonds between
// the ones that stay. A pure function of the job's snapshot, which it leaves as it was (the cells it writes are moved
// into the body frame; the poly and the input stay in the job's frame). Zeroes the job's stats first.
void lpFracture_RunJob( lpFractureJob* job );

// Frees the cells and hulls a job still owns
void lpFracture_FreeJob( lpFractureJob* job );

// What sees each finished job (lpWorld_SetFractureHook, world.h)
typedef void lpFractureHook( void* context, const lpFractureJob* job );

// The stages of lpFracture_RunJob that the validators (fcheck.h) run again on their own copies:
// a pattern cell's fate before the merge, once it is in the body frame (ejecta inside the break radius go by volume,
// slivers outside it turn to dust, the rest stay);
uint8_t lpFracture_ClassifyCell( const lpFractureJob* job, const lpShape* cell );
// and the chips of ghost cell `cell` while the job has `cellCount` cells (0: left whole; chips holds at most 4).
int lpFracture_ChipGhost( const lpFractureJob* job, int cell, int cellCount, lpShape** chips, lpFractureStats* stats );
