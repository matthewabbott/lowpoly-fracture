// SPDX-License-Identifier: MIT
// Exact geometry on the grid (milestone 11a): integer planes, vertices as the meeting of three planes, and the
// predicates on them, all exact. Fracture's topology decisions (side of a plane, incidence, coplanarity) are made here;
// floats only round the results for display and physics.
//
// The grid: u = 2^-16 m (15 um); a grid point has integer coordinates |p_i| <= P = 2^23 (+-128 m). A plane is
// n.x <= d, the inside where n.x < d (the same side as the float lpPlane's dot(normal, x) <= offset), with |n_i| <= A =
// 2^24 - 1, n != 0, and the plane meets the grid's range (|d| <= |n|_1 P, so |d| <= D = 3 A P < 2^48.59). Planes are
// made only by the four constructors below, which check every range and return false (and a reason) when a plane
// would leave it: the caller skips the cut and counts a reject. Their planes are canonical: gcd(n_x, n_y, n_z, d) = 1,
// so a half-space has one representation (equal planes are equal field by field) and the negation of a canonical plane
// is canonical (the other side of the same cut).
//
// The bit budget. Each quantity below is a determinant, or a step of one, of a matrix whose columns are bounded
// entrywise. A determinant is affine in each entry, so its largest magnitude over such a box is at a corner, where it
// is the product of the column bounds times the largest determinant of a +-1 matrix: 4 for order 3, 16 for order 4.
// Each step of the computation is bounded by the triangle inequality on its terms. For planes 1, 2, 3 meeting at a
// vertex, and a fourth plane 4 to classify it against:
//   cross products  c23 = n2 x n3 (and c31, c12)       |c|         <= 2 A^2                   < 2^49    (int64)
//   W = n1.c23 = det[n1; n2; n3]                       terms       <= 2 A^3                   < 2^73
//                                                      |W|, sums   <= 4 A^3                   < 2^74
//   X = d1 c23_x + d2 c31_x + d3 c12_x = det[d|ny|nz]  terms       <= 2 A^2 D = 6 A^3 P       < 2^97.59
//   (Y and Z alike)                                    |X|, sums   <= 4 A^2 D = 12 A^3 P      < 2^98.59
//   S = n4.(X, Y, Z) - d4 W = -det[n_i | d_i]          terms       <= 4 A^3 D = 12 A^4 P      < 2^122.59
//                                                      sums of n4  <= 36 A^4 P                < 2^124.17
//                                                      |S|         <= 16 A^3 D = 48 A^4 P     < 2^124.59
//   a grid point against a plane, n.p - d              |n.p - d|   <= 2 D = 6 A P             < 2^49.59 (int64)
// Every value fits a signed int128 (|v| < 2^127) with two bits to spare. The triangle inequality alone would give
// 18 A^3 P (2^99.17) for X and 48 A^4 P for S's last step; the determinant bound is what keeps X below 2^99 (the
// milestone plan's W < 2^75, numerators < 2^99 and S < 2^125 all hold). Every bound is reached, by planes whose
// normals and offsets are +-A and +-D in a Hadamard sign pattern (not canonical: the predicates need only the ranges),
// and TestPredicateBudget checks the int128 results there against a 256-bit reference.

#pragma once

#include "core.h"
#include "int128.h"

#include "lpf/lpmath.h"

#define LP_GRID_SCALE 65536.0f				 // grid units per metre: u = 2^-16 m
#define LP_GRID_RANGE ( 1 << 23 )			 // P: a grid point's |p_i| at most
#define LP_PLANE_NORMAL_MAX ( ( 1 << 24 ) - 1 ) // A: a plane's |n_i| at most
#define LP_SNAP_BITS_MAX 23					 // a snapped direction's largest component is 2^k, k <= 23

// The half-space n.x <= d (canonical when made by a constructor). It has padding: compare and hash it by fields.
typedef struct lpIPlane
{
	int32_t n[3];
	int64_t d;
} lpIPlane;

// The point where three planes meet, in homogeneous coordinates: (x/w, y/w, z/w), w > 0. Cramer's rule on the three
// planes, so the same three in any order give the same values (but not the same point from another triple: compare
// points by incidence, classifying one against the other's planes, never by cross-multiplying, which needs 173 bits).
// Fracture keeps one beside its defining plane triple (milestone 11a's plan): 64 bytes, no padding.
typedef struct lpIVertex
{
	lpI128 x, y, z, w;
} lpIVertex;

// Why a constructor made no plane
typedef enum lpGeomReject
{
	lp_geomOk,
	lp_geomOffGrid,		 // an input point is outside the grid's range
	lp_geomDegenerate,	 // no direction: equal sites, a zero or non-finite direction, a direction along the axis
	lp_geomNormalRange,	 // the reduced normal has a component beyond A
	lp_geomOffLattice,	 // two sites of a metric bisector are not on one lattice
	lp_geomParamRange,	 // a parameter out of its range (k, the axis, the stretch, the extent)
	lp_geomRejectCount
} lpGeomReject;

static inline lpIPlane lpIPlane_Negate( lpIPlane plane )
{
	lpIPlane r = { { -plane.n[0], -plane.n[1], -plane.n[2] }, -plane.d };
	return r;
}

static inline bool lpIPlane_Equal( const lpIPlane* a, const lpIPlane* b )
{
	return a->n[0] == b->n[0] && a->n[1] == b->n[1] && a->n[2] == b->n[2] && a->d == b->d;
}

// A total order (n_x, n_y, n_z, then d): -1, 0 or 1. Zero exactly when the planes are equal.
int lpIPlane_Compare( const lpIPlane* a, const lpIPlane* b );

// The plane is in range (|n_i| <= A, n != 0, it meets the grid's range) and canonical (gcd 1)
bool lpIPlane_IsValid( const lpIPlane* plane );

// True if the grid point is within the grid's range
static inline bool lpGrid_Contains( const int32_t p[3] )
{
	return p[0] >= -LP_GRID_RANGE && p[0] <= LP_GRID_RANGE && p[1] >= -LP_GRID_RANGE && p[1] <= LP_GRID_RANGE &&
		   p[2] >= -LP_GRID_RANGE && p[2] <= LP_GRID_RANGE;
}

// ---- the four constructors ----

// a. The bisector of two grid sites (the Voronoi pattern): n = 2 (b - a), d = |b|^2 - |a|^2, reduced; a is inside, b
// outside, and their midpoint on it. Sites less than 2^23 apart on every axis always get a plane (|n_i| <= A after
// reduction decides the rest).
bool lpIPlane_MakeBisector( const int32_t a[3], const int32_t b[3], lpIPlane* plane, lpGeomReject* why );

// The grain's metric, made once per job (or per object). Grain is an anisotropic metric: distances along the grain
// axis count 1/stretch as much, so Voronoi cells under it come out stretch times longer along the axis. The axis is
// snapped to a primitive integer g with |g_i| <= LP_GRAIN_AXIS_MAX; c = 1 - 1/stretch^2 is quantised as t/LP_GRAIN_Q;
// the metric is K = s I - t g g^T with s = LP_GRAIN_Q |g|^2 (s and t divided by their gcd), and the bisector of sites
// a and b is n = 2 K (b - a) / L, d = n.(a + b)/2, for (b - a) a multiple of the job's site lattice L: dividing L out is
// what keeps |n_i| <= A for sites far apart. Any two sites at most `span` lattice steps apart on every axis get a plane
// (span = floor(A / (2 |K|_inf)), |K|_inf the largest absolute row sum), so the job's lattice is the smallest power of
// two that fits its extent in span steps: no bisector of the job is ever rejected.
// Precision against the float design: the axis is at most 5.0 degrees off (at directions like (16, 1, 1); 3.6 within a
// coordinate plane); the stretch is off by about stretch^2/1024 of itself at most (1.6% at 4; wood's 3.5 becomes
// 16/sqrt(21) = 3.4915, -0.24%), and stretches up to 22 are taken (quantised ones reach 16). The span is 32767 steps
// for an axis-aligned grain at 3.5 (a 4 m piece gets a lattice of 16 u, 0.24 mm) and 139 for the worst axis, (8, 8, 7)
// (a 1 m piece gets 512 u, 7.8 mm; a 4 m one 2048 u, 3.1 cm). A finer stretch costs span in proportion (LP_GRAIN_Q).
#define LP_GRAIN_AXIS_MAX 8
#define LP_GRAIN_Q 256

typedef struct lpIMetric
{
	int32_t g[3];	 // the axis: primitive, |g_i| <= LP_GRAIN_AXIS_MAX
	int32_t s, t;	 // the metric s I - t g g^T, up to scale: t |g|^2 / s = c, s > t |g|^2
	int32_t lattice; // sites' coordinates differ by multiples of this (grid units, a power of two)
	int32_t span;	 // any two lattice sites at most this many steps apart on every axis get a plane
} lpIMetric;

// axis: the grain's direction (any length); stretch >= 1; extent: the largest coordinate difference between two of the
// job's sites on any axis (grid units, at most 2^24), which sets the lattice
bool lpIMetric_Make( lpVec3 axis, float stretch, int32_t extent, lpIMetric* metric, lpGeomReject* why );

// b. The metric bisector of two sites on the metric's lattice: a inside, b outside
bool lpIPlane_MakeMetricBisector( const lpIMetric* metric, const int32_t a[3], const int32_t b[3], lpIPlane* plane,
								  lpGeomReject* why );

// c. A plane through grid point p with a snapped normal (masonry, snap cuts, chips, authored faces): the float normal
// rounded (half away from zero) to an integer one whose largest component is 2^k, k <= 23; d = n.p; reduced. The
// snapped normal is within asin(2^-k / sqrt(2)) of the float one (two components rounded by 1/2 at most, against a
// length of 2^k at least): 1.1e-5 rad at k = 16, 8.4e-8 rad at 23. p lies on the plane exactly.
bool lpIPlane_MakeSnapped( lpVec3 normal, int k, const int32_t p[3], lpIPlane* plane, lpGeomReject* why );

// d. A plane containing the integer axis g through the grid point p (radial wedges, grain-aligned chips): n = g x s for
// `direction` snapped to s as in c, so n.g = 0 exactly and every such plane through p contains the whole line p + t g:
// they meet exactly on it. The inside is where (g x s).x < d. The plane is within about 2^-k / (sqrt(2) sin(g, s)) rad
// of the float plane containing g and the direction. |g_i| <= A; the plane always fits when 2 max|g_i| 2^k <= A (beyond
// that, the reduced normal decides).
bool lpIPlane_MakeAxial( const int32_t g[3], const int32_t p[3], lpVec3 direction, int k, lpIPlane* plane,
						 lpGeomReject* why );

// A float direction rounded to integers whose largest magnitude is 2^k (k <= 23), half away from zero. False for a zero
// or non-finite direction or k out of range.
bool lpGeom_SnapDirection( lpVec3 direction, int k, int32_t out[3] );

// The primitive integer direction with |g_i| <= LP_GRAIN_AXIS_MAX nearest the axis (sign: first nonzero component
// positive). False for a zero or non-finite axis.
bool lpGeom_SnapAxis( lpVec3 axis, int32_t g[3] );

// The grid point nearest a point given in metres (doubles), half away from zero. False outside the grid's range (or
// not finite).
bool lpGeom_GridPoint( double x, double y, double z, int32_t p[3] );

// ---- predicates ----

// The meeting point of three planes (in range, any order); false when they do not meet in one point (W = 0)
bool lpIVertex_FromPlanes( const lpIPlane* a, const lpIPlane* b, const lpIPlane* c, lpIVertex* vertex );

// n.X - d.W for a vertex of in-range planes against an in-range plane: its sign says the side (negative inside, zero
// on the plane, positive outside), and its magnitude is below 2^124.6 (the budget above)
lpI128 lpIVertex_Evaluate( const lpIVertex* vertex, const lpIPlane* plane );

static inline int lpIVertex_Classify( const lpIVertex* vertex, const lpIPlane* plane )
{
	return lpI128_Sign( lpIVertex_Evaluate( vertex, plane ) );
}

// A grid point against a plane, in int64: -1 inside, 0 on, 1 outside
static inline int lpIPlane_ClassifyPoint( const lpIPlane* plane, const int32_t p[3] )
{
	int64_t s = (int64_t)plane->n[0] * p[0] + (int64_t)plane->n[1] * p[1] + (int64_t)plane->n[2] * p[2] - plane->d;
	return s < 0 ? -1 : ( s > 0 ? 1 : 0 );
}

// ---- canonical rounding: a rational to the nearest grid point, or to the nearest float ----
//
// Both start from a double estimate and correct it exactly, so the answer is the exact nearest whatever the estimate's
// bits: x / w in double is within 2^-50 of itself (relative: lpI128_ToDouble's 2^-51 twice and the division's 2^-53),
// an estimate further than 2^-45 of itself from every rounding boundary is taken as it is (provably on the right side),
// and the rest are decided by integer comparisons in the budget below. For the grid, 2x against (2q -+ 1) w: below
// 2^99 for a vertex in the grid's range (|q| <= 2^23), below 2^126.1 at the domain's edge (|q| <= 2^50). For floats,
// x against w K 2^s, a midpoint K 2^s (K < 2^26) within 2^-22 of x / w: both sides below 2^101, the shift on whichever
// side keeps it so.

// x / w as a double (w > 0): an estimate, relative error below 2^-50
static inline double lpGeom_Ratio( lpI128 x, lpI128 w )
{
	return lpI128_ToDouble( x ) / lpI128_ToDouble( w );
}

// The grid point nearest x / w (w > 0; grid units), ties half away from zero (as lpGeom_SnapDirection, and the GPU
// lab's toy): 2x is compared with (2q -+ 1) w. For |x / w| < 2^50 (a vertex of a polyhedron in the grid's range is
// within 2^23).
int64_t lpGeom_RoundToGrid( lpI128 x, lpI128 w );

// The float nearest x / (w 2^16), in metres (w > 0), ties to even (IEEE's own rounding): x is compared with w times
// the float's midpoints with its neighbours. Any vertex of in-range planes (|x| < 2^99, 0 < w < 2^75) is a normal
// float or zero there. One exact point rounds to one float, so siblings' shared vertices are bit-identical in float.
float lpGeom_RoundToFloat( lpI128 x, lpI128 w );

static inline void lpIVertex_RoundToGrid( const lpIVertex* v, int64_t out[3] )
{
	out[0] = lpGeom_RoundToGrid( v->x, v->w );
	out[1] = lpGeom_RoundToGrid( v->y, v->w );
	out[2] = lpGeom_RoundToGrid( v->z, v->w );
}

static inline lpVec3 lpIVertex_RoundToFloat( const lpIVertex* v )
{
	lpVec3 r;
	r.x = lpGeom_RoundToFloat( v->x, v->w );
	r.y = lpGeom_RoundToFloat( v->y, v->w );
	r.z = lpGeom_RoundToFloat( v->z, v->w );
	return r;
}
