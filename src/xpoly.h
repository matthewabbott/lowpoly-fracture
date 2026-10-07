// SPDX-License-Identifier: MIT
// The exact working polyhedron (milestone 11a, C2), beside the float lpPoly that fracture still uses: faces on canonical
// integer planes (geom.h), each vertex the meeting point of three of its faces with that point cached in homogeneous
// int128. The clip is exact: no tolerance, no plane shift, no failure but capacity. Nothing in the engine uses it yet;
// C5 switches fracture over to it (and decides whether it takes the name lpPoly).
//
// Coordinates are the grid's (geom.h): grid units of 2^-16 m in the object frame, every plane in range, so every
// vertex, classification and product stays inside geom.h's bit budget. Floats only come out of it: each vertex rounded
// once to the float nearest the exact point (lpXPoly_Round), and masses in doubles from those floats (the plan's rule
// 22: thresholds are deterministic doubles from canonical floats).
//
// The defining triple. Each vertex keeps three of its faces whose planes meet only there, and the point they gave when
// the vertex was made (Cramer's rule, so the triple's order does not matter). A clip that drops a face through a kept
// vertex (only a vertex exactly on the cutting plane can lose one) picks the triple again: the cap and the two faces
// whose cap edges meet at the vertex, which always meet in one point (the cap edges are not collinear). The cached
// point is kept as it was, not recomputed from the new triple: it is still the same point (the validator checks the
// incidence), and both halves of a clip, which may re-pick differently, keep the same values for it. The planes are not
// kept by value: a vertex always has three faces of its own to name (C5's lpShape stores face triples, not planes), and
// indices cost 3 bytes against 72.

#pragma once

#include "geom.h"
#include "poly.h"

#define LP_XPOLY_MAX_VERTICES LP_POLY_MAX_VERTICES
#define LP_XPOLY_MAX_FACES LP_POLY_MAX_FACES
#define LP_XPOLY_MAX_INDICES LP_POLY_MAX_INDICES

// The faces of the box at the grid's range that lpXPoly_FromPlanes starts from: one surviving means the half-spaces
// were unbounded (or reach past the grid's range)
#define LP_TAG_RANGE ( -3 )

typedef struct lpXFace
{
	lpIPlane plane; // canonical; the polyhedron is on its inside, n.x <= d
	uint16_t first; // first index into the index array
	uint8_t count;	// vertex count of the face loop
	uint8_t material;
	int32_t tag;
} lpXFace;

// Fixed capacity, so clipping never allocates. Face loops are counter clockwise seen from outside. Every face holds
// exactly the vertices on its plane, and every vertex is strictly inside every other face's plane.
typedef struct lpXPoly
{
	int vertexCount;
	int faceCount;
	int indexCount;
	lpIVertex vertices[LP_XPOLY_MAX_VERTICES];	// the exact points, homogeneous (w > 0)
	double approx[LP_XPOLY_MAX_VERTICES][3];	// x / w in grid units (lpGeom_Ratio, within 2^-50): bounds and early outs
	uint8_t triples[LP_XPOLY_MAX_VERTICES][3];	// three faces through the vertex whose planes meet only there
	lpXFace faces[LP_XPOLY_MAX_FACES];
	uint8_t indices[LP_XPOLY_MAX_INDICES];
} lpXPoly;

// What exact clips did (counted, never branched on)
typedef struct lpXClipStats
{
	int64_t clips;
	int64_t unchanged;
	int64_t cut;
	int64_t empty;
	int64_t overflows; // out of vertices, faces or indices: the clip did nothing
	int64_t touching;  // cuts with a vertex exactly on the plane (kept, and joining the cap)
	int64_t classifications;
	int64_t vertices; // made where an edge crosses the plane
	int64_t repicked; // kept vertices whose triple lost a face
} lpXClipStats;

// Copies the used part
void lpXPoly_Copy( lpXPoly* out, const lpXPoly* in );

// Keep the part with n.x <= d. Exact in every case: a vertex on the plane stays and joins the cap; an edge crossing it
// gets a new vertex (its two faces and the plane); a face that meets the plane in a vertex or an edge only is kept when
// it has a vertex strictly inside, dropped otherwise; nothing outside the plane (a face may lie in it): unchanged, `out`
// not written; nothing strictly inside: empty. The cap (the plane, material, tag) is the last face, chained from the
// half-edges the plane left unmatched. Clipping by a plane and by its exact negation gives two halves whose caps hold
// the same vertices, value for value, in opposite orders. lp_clipOverflow when a limit is reached (`out` is then
// garbage); lp_clipFailed only for an input that is not a valid polyhedron (asserted). `out` must not alias `in`.
lpClipResult lpXPoly_Clip( const lpXPoly* in, const lpIPlane* plane, uint8_t material, int32_t tag, lpXPoly* out );

// lpXPoly_Clip, counted into stats (NULL: not counted)
lpClipResult lpXPoly_ClipCounted( const lpXPoly* in, const lpIPlane* plane, uint8_t material, int32_t tag, lpXPoly* out,
								  lpXClipStats* stats );

typedef enum lpXBuild
{
	lp_xBuilt,
	lp_xEmpty,	   // the half-spaces have no interior in common
	lp_xUnbounded, // a face of the range box survived: unbounded, or reaching past the grid's range
	lp_xOverflow,
	lp_xBadPlane, // a plane out of range or not canonical
} lpXBuild;

// The intersection of half-spaces: the box at the grid's range clipped by each in turn (materials and tags NULL: 0 and
// LP_TAG_EXTERIOR). A face's material and tag are its plane's; of planes repeated, the first one's.
lpXBuild lpXPoly_FromPlanes( const lpIPlane* planes, const uint8_t* materials, const int32_t* tags, int count, lpXPoly* out,
							 lpXPoly* scratch );

// The snap of the conversions below: a normal's largest component 2^23, within 8.4e-8 rad of the float one
#define LP_XPOLY_SNAP_BITS 23

// A box from six snapped planes (lpIPlane_MakeSnapped), each through the grid point nearest its face's centre in metres
lpXBuild lpXPoly_MakeBox( lpVec3 halfExtents, lpTransform transform, uint8_t material, lpXPoly* out, lpXPoly* scratch );

// A float polyhedron converted, the authoring path's first draft (C3 refines it: near-coplanar faces merged): each face's
// plane snapped through the grid point nearest the mean of its vertices, which are offset by `origin` first (metres:
// the poly's frame in the grid's), then lpXPoly_FromPlanes. Faces keep their material and tag; a face whose snapped
// plane does not bound the result (redundant, or repeated) is lost.
lpXBuild lpXPoly_FromPoly( const lpPoly* poly, lpVec3 origin, lpXPoly* out, lpXPoly* scratch );

// Each vertex rounded to the float nearest it, in metres (lpIVertex_RoundToFloat)
void lpXPoly_Round( const lpXPoly* poly, lpVec3* rounded );

// Volume (m^3) and centroid (metres) in doubles from the rounded vertices (the deterministic thresholds' mass)
void lpXPoly_ComputeMass( const lpXPoly* poly, const lpVec3* rounded, double* volume, double centroid[3] );

// Volume and centroid from the exact vertices taken relative to a grid point (each offset exact in int128, then one
// division in double): precise to about 2^-50 of the polyhedron's size, for checks (tiling to 1e-9 and better)
void lpXPoly_ComputeMassPrecise( const lpXPoly* poly, double* volume, double centroid[3] );

// The float polyhedron of the rounded vertices: face planes from the integer ones (unit normal, offset in metres), the
// same loops, materials and tags
void lpXPoly_ToPoly( const lpXPoly* poly, const lpVec3* rounded, lpPoly* out );

// NULL when the polyhedron is valid, else what is wrong. Topology is decided exactly: counts in range, canonical
// planes and no two faces on one plane; faces of three vertices or more, none repeated; a closed 2-manifold, each edge
// in two faces in opposite directions, V - E + F = 2; every vertex exactly on every face whose loop holds it and
// strictly inside every other face's plane; every vertex's triple three faces holding it whose planes meet in one point,
// with the cached point on all three. Then the orientation, counter clockwise from outside: a positive volume.
const char* lpXPoly_Validate( const lpXPoly* poly );

static inline bool lpXPoly_IsValid( const lpXPoly* poly )
{
	return lpXPoly_Validate( poly ) == NULL;
}

// True if vertex va of a and vertex vb of b are the same point: va is on the three planes of vb's triple (by
// incidence, never by cross-multiplying)
bool lpXPoly_SamePoint( const lpXPoly* a, int va, const lpXPoly* b, int vb );

// The exact polyhedron's digest, field by field: counts, the exact points, triples, face planes, loops, materials and
// tags (not the doubles, which only estimate)
uint64_t lpXPoly_Digest( const lpXPoly* poly );
