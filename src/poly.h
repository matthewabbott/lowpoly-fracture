// SPDX-License-Identifier: MIT
// Convex polyhedra with convex polygon faces: the one geometric primitive of the fracture core.
// Every destructible piece is one of these, and so is every fracture cell (convex ∩ convex = convex).

#pragma once

#include "core.h"

#include "lpf/lpmath.h"
#include "phys.h"

#define LP_POLY_MAX_VERTICES 128
#define LP_POLY_MAX_FACES 64
#define LP_POLY_MAX_INDICES 512

// Face tag for authored outer surface. Cut faces get a tag >= 0 naming the cut (site or plane id) while their
// fracture is being computed; afterwards they are normalised to LP_TAG_CUT so a later fracture of the same piece
// cannot mistake them for its own site indices.
#define LP_TAG_EXTERIOR ( -1 )
#define LP_TAG_CUT ( -2 )

typedef struct lpFace
{
	lpPlane plane;
	uint16_t first; // first index into the index array
	uint8_t count;	// vertex count of the face loop
	uint8_t material;
	int32_t tag;
} lpFace;

// Working polyhedron with fixed capacity, so clipping never allocates. Face loops are counter clockwise
// seen from outside.
typedef struct lpPoly
{
	int vertexCount;
	int faceCount;
	int indexCount;
	lpVec3 vertices[LP_POLY_MAX_VERTICES];
	lpFace faces[LP_POLY_MAX_FACES];
	uint8_t indices[LP_POLY_MAX_INDICES];
} lpPoly;

typedef enum lpClipResult
{
	lp_clipUnchanged,
	lp_clipCut,
	lp_clipEmpty,
	lp_clipOverflow,
	lp_clipFailed,
} lpClipResult;

void lpPoly_MakeBox( lpPoly* poly, lpVec3 halfExtents, lpTransform transform, uint8_t material );

// Convex hull of a point cloud (via Box3D's quickhull). Returns false if degenerate or too large.
bool lpPoly_MakeFromPoints( lpPoly* poly, const lpVec3* points, int count, uint8_t material );
bool lpPoly_MakeFromHull( lpPoly* poly, const lpPhysHull* hull, uint8_t material );

// Keep the part with dot(normal, x) <= offset. The new cap face gets material and tag.
// Degenerate cases (vertices within tolerance of the plane) are removed by pushing the plane outward by a
// sub-tolerance amount, which keeps the topology exact. `out` must not alias `in`.
lpClipResult lpPoly_Clip( const lpPoly* in, lpPlane plane, uint8_t material, int32_t tag, float tolerance, lpPoly* out );

void lpPoly_Translate( lpPoly* poly, lpVec3 translation );

// Apply a linear map (e.g. a non-uniform scale along the wood grain). Face planes are rebuilt with
// Newell's method. Convexity is preserved by any invertible linear map.
void lpPoly_ApplyLinear( lpPoly* poly, lpMatrix3 m );

void lpPoly_ComputeMass( const lpPoly* poly, float* volume, lpVec3* centroid );
lpAABB lpPoly_ComputeBounds( const lpPoly* poly );
float lpPoly_MaxDistanceSquared( const lpPoly* poly, lpVec3 point );

// max over face planes of the signed plane distance: exact inside and outside faces, a lower bound
// of the true distance near edges and corners.
float lpPoly_SignedDistance( const lpPoly* poly, lpVec3 point );

// Topology and geometry checks for tests: closed 2-manifold with consistent orientation, planar faces,
// convex (every vertex behind every plane), Euler characteristic 2.
bool lpPoly_IsValid( const lpPoly* poly, float tolerance );

// Compact immutable copy owned by a piece. One allocation holds all arrays.
typedef struct lpShape
{
	lpVec3* vertices;
	lpFace* faces;
	uint8_t* indices;
	int vertexCount;
	int faceCount;
	int indexCount;
	lpAABB bounds;
	lpVec3 centroid;
	float volume;
	float radius; // max distance from centroid to a vertex
} lpShape;

// Returns NULL for a degenerate polyhedron (non-positive volume).
lpShape* lpShape_Create( const lpPoly* poly );
void lpShape_Destroy( lpShape* shape );
void lpShape_ToPoly( const lpShape* shape, lpPoly* poly );
void lpShape_Translate( lpShape* shape, lpVec3 translation );

// True if the shape has a face lying on the plane (same normal and offset within tolerance).
bool lpShape_HasFaceOnPlane( const lpShape* shape, lpPlane plane, float tolerance );
float lpShape_SignedDistance( const lpShape* shape, lpVec3 point );
float lpShape_FaceArea( const lpShape* shape, int faceIndex, lpVec3* centroid );

// True if the shapes overlap or come within `margin` of each other. Catches authored parts that meet at an angle
// (roof on a gable) where there is no shared coplanar face.
bool lpShape_NearlyOverlap( const lpShape* a, const lpShape* b, float margin );

// True if the boxes overlap or are within `margin` of each other on every axis
static inline bool lpBoxesTouch( lpAABB a, lpAABB b, float margin )
{
	return !( a.lowerBound.x > b.upperBound.x + margin || b.lowerBound.x > a.upperBound.x + margin ||
			  a.lowerBound.y > b.upperBound.y + margin || b.lowerBound.y > a.upperBound.y + margin ||
			  a.lowerBound.z > b.upperBound.z + margin || b.lowerBound.z > a.upperBound.z + margin );
}

// The physics hull with the same vertices. Caller owns the result (lpPhys_DestroyHull). NULL on failure.
lpPhysHull* lpShape_CreateHull( const lpShape* shape );

// Where two pieces touch: the patch a bond sits on
typedef struct lpContact
{
	float area;
	lpVec3 centroid;
	lpVec3 normal; // unit, pointing from the first shape toward the second
	float h1, h2;  // half-extents of the patch along the tangents of lpContactBasis( normal )
} lpContact;

// Tangents for a contact normal. The same rule everywhere, so extents and bending axes agree.
static inline void lpContactBasis( lpVec3 n, lpVec3* t1, lpVec3* t2 )
{
	lpVec3 t = lpAbsFloat( n.x ) < 0.57f ? (lpVec3){ 1.0f, 0.0f, 0.0f } : (lpVec3){ 0.0f, 1.0f, 0.0f };
	*t1 = lpNormalize( lpCross( t, n ) );
	*t2 = lpCross( n, *t1 );
}

// Where a face of `a` lies on a face of `b` with opposite normals (the faces two pieces share): total area,
// area-weighted centroid, the normal of the largest patch and the extents of all patches. False if they share none.
bool lpShape_Contact( const lpShape* a, const lpShape* b, float tolerance, lpContact* contact );

// The contact a face of `shape` makes with whatever lies on it (Voronoi siblings share whole faces)
void lpShape_FaceContact( const lpShape* shape, int faceIndex, lpContact* contact );
