// SPDX-License-Identifier: MIT
// Fracture jobs checked (milestone 11a): a job's snapshot to and from bytes, the digest of what it produced, and the
// validators that say what "fractures exactly" means, with the census of coordinates and edges the exact grid needs.
// Pure (the validators re-run a job's pattern and stages on their own copies), but for the census of a world's parts.
// No file I/O: lpf_bench --check-fractures, --record-fractures and --replay-fractures do that (bench/fractures.c).
// Validators measure in double from the float cells and run sequentially (in phase 3, or in a replay); nothing they
// compute reaches the simulation.

#pragma once

#include "fracture.h"

// ---- snapshots and digests ----

#define LP_JOB_SNAPSHOT_VERSION 1
#define LP_JOB_SNAPSHOT_MAX 4096 // bytes a snapshot takes at most

// Everything lpFracture_RunJob reads (the poly, the input, the tier thresholds, the merge slack and the chip splits),
// the job's centre and local impact, and the impact that broke it (which the integration reads), little-endian with
// exact float bits, behind a magic and a version and followed by a checksum. Writes it when it fits in capacity
// (bytes may be NULL) and returns its size either way.
int lpFractureJob_Write( const lpFractureJob* job, uint8_t* bytes, int capacity );

// Parses a snapshot into job, ready to run: its bonds array must be allocated (lpFractureJob_Create) and it must own no
// cells. False, leaving the job empty, when the bytes are short or long, fail the checksum, are of another version, or
// hold a value out of range (counts past the polyhedron's limits, an index past its vertices, a float that is not finite).
bool lpFractureJob_Read( lpFractureJob* job, const uint8_t* bytes, int size );

// A job for replays and checks, with its bonds array and no cells; destroying it frees its cells and hulls too
lpFractureJob* lpFractureJob_Create( void );
void lpFractureJob_Destroy( lpFractureJob* job );

// The digest of a finished job's output, exact bits in job order: every cell (vertices, faces with planes, tags and
// materials, loops, volume, centroid, radius, bounds), its site, and with two cells or more its class and its physics
// hull's points and planes; then every bond with its contact. A replay compares it with the recorded one.
uint64_t lpFractureJob_Digest( const lpFractureJob* job );

// Sums one job's stats into a total (counts and times added, maxima kept)
void lpFractureStats_Add( lpFractureStats* total, const lpFractureStats* one );

// ---- validators ----

// The exact grid of milestone 11a: 2^-16 m, within +-128 m of the object's origin
#define LP_CHECK_GRID ( 1.0 / 65536.0 )
#define LP_CHECK_RANGE 128.0
// Volumes are summed in double from the float vertices, so tiling is held to C5's tolerance, not to zero
#define LP_CHECK_TILING 1e-9

// Coordinates, edges and close vertices of some shapes, each in its object frame
typedef struct lpShapeCensus
{
	int shapes;
	double maxCoordinate; // the largest |x|, |y| or |z| of a vertex
	int beyondRange;	  // vertices past LP_CHECK_RANGE
	double minEdge;
	double minFaceArea;
	int closePairs;		// vertex pairs of one shape closer than the grid step
	int gridCollisions; // distinct vertices of one shape that round to the same grid point
	int maxFaces;
	int maxVertices;
} lpShapeCensus;

// What the validators found, summed over jobs. A violation is any nonzero error ("fractures exactly"); in floats nearly
// everything violates, and the counts are milestone 11a's before-numbers.
typedef struct lpFractureCheck
{
	int jobs;		  // validated: the jobs with two cells or more
	int patternCells; // what the pattern gave again
	int outputCells;

	// a. tiling: the pattern cells' volumes sum to the parent's
	int tilingViolations; // jobs off by more than LP_CHECK_TILING of the parent's volume
	int tilingGaps;		  // of those, jobs off by more than 1e-3: a cell missing, not rounding
	double tilingMaxError; // relative

	// b. overlap: pattern cells whose intersection has volume (one clipped by the other's face planes, in double)
	int overlapPairs; // pairs whose bounds meet
	int overlapViolations;
	double overlapMax; // m^3
	double overlapMaxRelative; // of the smaller cell
	double overlapTotal;

	// c. siblings: every cut face of a pattern cell (one not on the parent's surface) against the faces of the other
	// cells on its opposite plane (normals within 0.9999, offsets within 1 mm)
	int siblingFaces;
	int siblingExact;	// its twin: one face on the exactly negated plane with the same vertices
	int siblingCovered; // covered by faces on the exactly negated plane that reach past it, or by several, which meet it
						// at T-junctions (a masonry plate under its bricks, radial rings across a wedge's side): all
						// they can do
	int siblingNear;	// covered within tolerance but not exactly (a twin whose vertices differ, a plane not exactly
						// negated; uncovered under 1e-3 of it or 1e-8 m^2): a violation
	int siblingUnmatched; // a violation
	double siblingMaxDistance;	   // of a near face's vertex from the faces covering it, m
	double siblingUnmatchedArea;   // in total, m^2
	double siblingMaxUnmatchedArea;

	// d. validity of the output cells
	int invalidCells; // not a closed oriented 2-manifold with V - E + F = 2, a degenerate face, or a repeated vertex
	int degenerateFaces; // under three vertices, a vertex twice in its loop, or zero area
	int convexViolations; // cells with a vertex outside one of their face planes
	double maxConvexExcess; // m
	double maxPlanarity;	// a vertex off its own face's plane, m

	// e. merge containment: every keeper before the merge inside one cell after it
	int keepersChecked;
	int containViolations;
	double maxContainExcess; // a vertex outside the best cell's face planes, m

	// f. chips: they tile their ghost cell without overlap
	int chipSets;
	int chipViolations;
	double chipMaxTilingError; // relative
	double chipMaxOverlap;	   // m^3
	int chipMismatches;		   // chips made again that differ from the job's: never, the stage is pure

	// g. census of the output cells (in the object frame: the job's centre plus its poly)
	lpShapeCensus census;
	int validatorOverflows; // measurements that ran out of room (none expected)
} lpFractureCheck;

void lpFractureCheck_Init( lpFractureCheck* check );
void lpFractureCheck_Add( lpFractureCheck* total, const lpFractureCheck* one );
int lpFractureCheck_Violations( const lpFractureCheck* check );

// Validates a finished job (before it is integrated): runs its pattern again for a to c, the job again without chips
// for e and f when it chipped, and checks its own cells for d and g. Adds to check.
void lpFractureJob_Validate( const lpFractureJob* job, lpFractureCheck* check );

// The checks one by one, on cells in one frame (tests inject faults through these). Tiling returns the relative error.
double lpFractureCheck_Tiling( lpFractureCheck* check, const lpPoly* parent, lpShape* const* cells, int count );
void lpFractureCheck_Overlap( lpFractureCheck* check, lpShape* const* cells, int count );
void lpFractureCheck_Siblings( lpFractureCheck* check, const lpPoly* parent, lpShape* const* cells, int count );
void lpFractureCheck_Cells( lpFractureCheck* check, lpShape* const* cells, int count ); // d, and the census
void lpFractureCheck_Containment( lpFractureCheck* check, lpShape* const* inner, int innerCount, lpShape* const* outer,
								  int outerCount );
void lpFractureCheck_Chips( lpFractureCheck* check, const lpShape* cell, lpShape* const* chips, int count );

// ---- the parts a scene authored ----

typedef struct lpPartCensus
{
	int parts;
	int groundParts;
	lpShapeCensus all;
	lpShapeCensus noGround; // the ground is one big slab, far past any part
} lpPartCensus;

void lpShapeCensus_Init( lpShapeCensus* census );
void lpShapeCensus_Add( lpShapeCensus* census, const lpShape* shape );

// Every piece's shape as it stands (at scene build: every authored part before any fracture)
void lpWorld_CensusParts( const lpWorld* world, lpPartCensus* census );
