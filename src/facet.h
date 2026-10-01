// SPDX-License-Identifier: MIT
// Render meshes for pieces: flat-shaded triangles with one color per face. Exterior faces keep the authored
// color; freshly cut interior faces get the material's solid "interior color" function evaluated at the face
// centroid in object space, so neighbouring cuts line up (wood rings, stone speckle).

#pragma once

#include "poly.h"

#define LP_MAX_PIECE_VERTICES ( 3 * LP_POLY_MAX_INDICES )

typedef struct lpFacetParams
{
	uint32_t exteriorColor; // 0xRRGGBB
	lpVec3 axis;			// grain axis in the body frame
} lpFacetParams;

// Returns the vertex count (3 per triangle), or -1 if capacity is too small.
int lpBuildFacetMesh( const lpShape* shape, const lpFacetParams* params, lpVertex* vertices, int capacity );
