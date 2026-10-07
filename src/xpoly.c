// SPDX-License-Identifier: MIT

#include "xpoly.h"

#include <math.h>

void lpXPoly_Copy( lpXPoly* out, const lpXPoly* in )
{
	LP_ASSERT( out != in );
	size_t vertexCount = (size_t)in->vertexCount;
	out->vertexCount = in->vertexCount;
	out->faceCount = in->faceCount;
	out->indexCount = in->indexCount;
	memcpy( out->vertices, in->vertices, sizeof( lpIVertex ) * vertexCount );
	memcpy( out->approx, in->approx, sizeof( in->approx[0] ) * vertexCount );
	memcpy( out->triples, in->triples, sizeof( in->triples[0] ) * vertexCount );
	memcpy( out->faces, in->faces, sizeof( lpXFace ) * (size_t)in->faceCount );
	memcpy( out->indices, in->indices, (size_t)in->indexCount );
}

static void lpXApprox( const lpIVertex* v, double out[3] )
{
	double w = lpI128_ToDouble( v->w ); // as lpGeom_Ratio, bit for bit
	out[0] = lpI128_ToDouble( v->x ) / w;
	out[1] = lpI128_ToDouble( v->y ) / w;
	out[2] = lpI128_ToDouble( v->z ) / w;
}

// ---- the clip ----

lpClipResult lpXPoly_ClipCounted( const lpXPoly* in, const lpIPlane* plane, uint8_t material, int32_t tag, lpXPoly* out,
								  lpXClipStats* stats )
{
	LP_ASSERT( in != out );

	// Every vertex classified exactly: -1 inside, 0 on the plane, 1 outside
	int vertexCount = in->vertexCount;
	int8_t side[LP_XPOLY_MAX_VERTICES];
	int insideCount = 0;
	int outsideCount = 0;
	for ( int i = 0; i < vertexCount; ++i )
	{
		int s = lpIVertex_Classify( in->vertices + i, plane );
		side[i] = (int8_t)s;
		insideCount += s < 0 ? 1 : 0;
		outsideCount += s > 0 ? 1 : 0;
	}
	if ( stats != NULL )
	{
		stats->clips += 1;
		stats->classifications += vertexCount;
	}

	// Nothing outside (a vertex, an edge or a face may lie in the plane): the closed half-space holds it all. Nothing
	// strictly inside: what is left is a face, an edge or a vertex at most, no volume.
	if ( outsideCount == 0 )
	{
		if ( stats != NULL )
		{
			stats->unchanged += 1;
		}
		return lp_clipUnchanged;
	}
	if ( insideCount == 0 )
	{
		if ( stats != NULL )
		{
			stats->empty += 1;
		}
		return lp_clipEmpty;
	}

	// A proper cut: the plane goes through the interior, so no face lies in it, the cap is a convex polygon, and every
	// vertex on the plane is one of its corners.
	lpClipResult result = lp_clipOverflow;
	int map[LP_XPOLY_MAX_VERTICES];
	int8_t capNext[LP_XPOLY_MAX_VERTICES]; // the cap's loop: capNext[b] = a for a face's cut edge a -> b
	int8_t capFaces[LP_XPOLY_MAX_VERTICES][2]; // the faces of the two cap edges at an on-plane vertex (out indices)
	bool onCap[LP_XPOLY_MAX_VERTICES];
	int outVertexCount = 0;
	for ( int i = 0; i < vertexCount; ++i )
	{
		if ( side[i] > 0 )
		{
			map[i] = -1;
			continue;
		}
		int v = outVertexCount++;
		map[i] = v;
		out->vertices[v] = in->vertices[i];
		memcpy( out->approx[v], in->approx[i], sizeof( out->approx[v] ) );
		memcpy( out->triples[v], in->triples[i], sizeof( out->triples[v] ) ); // input faces, remapped below
		onCap[v] = side[i] == 0;
		capNext[v] = -1;
		capFaces[v][0] = -1;
		capFaces[v][1] = -1;
	}
	int keptCount = outVertexCount;

	// Edges crossing the plane strictly (one end inside, one outside), keyed by their sorted vertex pair: the new vertex
	// is made once, from the edge's two faces and the plane, after both faces have named themselves
	enum
	{
		lp_maxCrossings = LP_XPOLY_MAX_VERTICES
	};
	uint8_t crossA[lp_maxCrossings];
	uint8_t crossB[lp_maxCrossings];
	int8_t crossFaces[lp_maxCrossings][2];
	int crossCount = 0;

	int faceMap[LP_XPOLY_MAX_FACES];
	int outFaceCount = 0;
	int outIndexCount = 0;
	int capEdges = 0;
	for ( int f = 0; f < in->faceCount; ++f )
	{
		const lpXFace* face = in->faces + f;
		const uint8_t* loop = in->indices + face->first;
		int count = face->count;

		// Kept only with a vertex strictly inside: a face meeting the plane in a vertex or an edge from outside has
		// nothing left with area
		bool inside = false;
		for ( int k = 0; k < count && inside == false; ++k )
		{
			inside = side[loop[k]] < 0;
		}
		if ( inside == false )
		{
			faceMap[f] = -1;
			continue;
		}
		if ( outFaceCount + 1 >= LP_XPOLY_MAX_FACES ) // and the cap
		{
			goto done;
		}

		int first = outIndexCount;
		for ( int k = 0; k < count; ++k )
		{
			int cur = loop[k];
			int next = loop[k + 1 < count ? k + 1 : 0];
			if ( side[cur] <= 0 )
			{
				if ( outIndexCount >= LP_XPOLY_MAX_INDICES )
				{
					goto done;
				}
				out->indices[outIndexCount++] = (uint8_t)map[cur];
			}
			if ( side[cur] * side[next] < 0 )
			{
				uint8_t a = (uint8_t)( cur < next ? cur : next );
				uint8_t b = (uint8_t)( cur < next ? next : cur );
				int c = 0;
				while ( c < crossCount && ( crossA[c] != a || crossB[c] != b ) )
				{
					c += 1;
				}
				if ( c == crossCount )
				{
					if ( outVertexCount >= LP_XPOLY_MAX_VERTICES )
					{
						goto done;
					}
					crossA[c] = a;
					crossB[c] = b;
					crossFaces[c][0] = (int8_t)f;
					crossFaces[c][1] = -1;
					crossCount += 1;
					int v = outVertexCount++;
					onCap[v] = true;
					capNext[v] = -1;
					capFaces[v][0] = -1;
					capFaces[v][1] = -1;
				}
				else
				{
					LP_ASSERT( crossFaces[c][1] < 0 );
					crossFaces[c][1] = (int8_t)f;
				}
				if ( outIndexCount >= LP_XPOLY_MAX_INDICES )
				{
					goto done;
				}
				out->indices[outIndexCount++] = (uint8_t)( keptCount + c );
			}
		}

		int outCount = outIndexCount - first;
		LP_ASSERT( outCount >= 3 );
		int g = outFaceCount++;
		faceMap[f] = g;
		lpXFace* outFace = out->faces + g;
		*outFace = *face;
		outFace->first = (uint16_t)first;
		outFace->count = (uint8_t)outCount;

		// Its cut edge: the one pair of neighbours on the plane (the face meets the plane in a chord, of which a strictly
		// convex polygon has at most two corners); the cap runs it the other way
		const uint8_t* outLoop = out->indices + first;
		for ( int k = 0; k < outCount; ++k )
		{
			int a = outLoop[k];
			int b = outLoop[k + 1 < outCount ? k + 1 : 0];
			if ( onCap[a] && onCap[b] )
			{
				if ( capNext[b] >= 0 || capFaces[a][1] >= 0 )
				{
					LP_ASSERT( false ); // a vertex on two cut edges of one side: the input is not a valid polyhedron
					result = lp_clipFailed;
					goto done;
				}
				capNext[b] = (int8_t)a;
				capFaces[b][0] = (int8_t)g;
				capFaces[a][1] = (int8_t)g;
				capEdges += 1;
				break;
			}
		}
	}

	// The cap: last, after the kept faces
	int cap = outFaceCount;
	if ( cap >= LP_XPOLY_MAX_FACES || outIndexCount + capEdges > LP_XPOLY_MAX_INDICES )
	{
		goto done;
	}

	// The new vertices: each from its edge's two faces and the plane, which Cramer's rule gives the same from either
	// face's side and from the negated plane (geom.h)
	for ( int c = 0; c < crossCount; ++c )
	{
		int v = keptCount + c;
		const lpIPlane* p0 = &in->faces[crossFaces[c][0]].plane;
		LP_ASSERT( crossFaces[c][1] >= 0 );
		if ( crossFaces[c][1] < 0 )
		{
			result = lp_clipFailed; // an edge in one face only: the input is not closed
			goto done;
		}
		const lpIPlane* p1 = &in->faces[crossFaces[c][1]].plane;
		bool met = lpIVertex_FromPlanes( p0, p1, plane, out->vertices + v );
		LP_ASSERT( met ); // the edge crosses the plane strictly, so its line and the plane meet in one point
		(void)met;
		lpXApprox( out->vertices + v, out->approx[v] );
		out->triples[v][0] = (uint8_t)faceMap[crossFaces[c][0]];
		out->triples[v][1] = (uint8_t)faceMap[crossFaces[c][1]];
		out->triples[v][2] = (uint8_t)cap;
	}

	// Kept vertices' triples in the new face indices; one that lost a face lay on the plane, and takes the cap and the
	// faces of its two cap edges, whose planes meet only there
	int repicked = 0;
	for ( int v = 0; v < keptCount; ++v )
	{
		uint8_t* t = out->triples[v];
		int t0 = faceMap[t[0]], t1 = faceMap[t[1]], t2 = faceMap[t[2]];
		if ( t0 >= 0 && t1 >= 0 && t2 >= 0 )
		{
			t[0] = (uint8_t)t0;
			t[1] = (uint8_t)t1;
			t[2] = (uint8_t)t2;
			continue;
		}
		LP_ASSERT( onCap[v] && capFaces[v][0] >= 0 && capFaces[v][1] >= 0 );
		if ( capFaces[v][0] < 0 || capFaces[v][1] < 0 )
		{
			result = lp_clipFailed;
			goto done;
		}
		t[0] = (uint8_t)cap;
		t[1] = (uint8_t)capFaces[v][0];
		t[2] = (uint8_t)capFaces[v][1];
		repicked += 1;
	}

	// Chain the cap from its lowest vertex
	{
		int start = 0;
		while ( start < outVertexCount && capNext[start] < 0 )
		{
			start += 1;
		}
		int capFirst = outIndexCount;
		int v = start;
		int capCount = 0;
		do
		{
			if ( v < 0 || capCount >= capEdges )
			{
				LP_ASSERT( false ); // the cut edges do not close into one loop: the input is not a valid polyhedron
				result = lp_clipFailed;
				goto done;
			}
			out->indices[outIndexCount++] = (uint8_t)v;
			capCount += 1;
			v = capNext[v];
		}
		while ( v != start );
		if ( capCount != capEdges || capCount < 3 )
		{
			LP_ASSERT( false );
			result = lp_clipFailed;
			goto done;
		}
		lpXFace* capFace = out->faces + cap;
		capFace->plane = *plane;
		capFace->first = (uint16_t)capFirst;
		capFace->count = (uint8_t)capCount;
		capFace->material = material;
		capFace->tag = tag;
	}

	out->vertexCount = outVertexCount;
	out->faceCount = cap + 1;
	out->indexCount = outIndexCount;
	result = lp_clipCut;
	if ( stats != NULL )
	{
		stats->cut += 1;
		stats->touching += insideCount + outsideCount < vertexCount ? 1 : 0;
		stats->vertices += crossCount;
		stats->repicked += repicked;
	}

done:
	if ( stats != NULL && result == lp_clipOverflow )
	{
		stats->overflows += 1;
	}
	return result;
}

lpClipResult lpXPoly_Clip( const lpXPoly* in, const lpIPlane* plane, uint8_t material, int32_t tag, lpXPoly* out )
{
	return lpXPoly_ClipCounted( in, plane, material, tag, out, NULL );
}

// ---- building ----

// Box vertex i has coordinates (i&1 ? +r : -r, i&2 ? +r : -r, i&4 ? +r : -r); loops counter clockwise from outside,
// faces +x, -x, +y, -y, +z, -z (as poly.c's box)
static const uint8_t lp_xBoxLoops[6][4] = {
	{ 1, 3, 7, 5 }, { 0, 4, 6, 2 }, { 2, 6, 7, 3 }, { 0, 1, 5, 4 }, { 4, 5, 7, 6 }, { 0, 2, 3, 1 },
};

static const lpIPlane lp_rangePlanes[6] = {
	{ { 1, 0, 0 }, LP_GRID_RANGE },	 { { -1, 0, 0 }, LP_GRID_RANGE }, { { 0, 1, 0 }, LP_GRID_RANGE },
	{ { 0, -1, 0 }, LP_GRID_RANGE }, { { 0, 0, 1 }, LP_GRID_RANGE },  { { 0, 0, -1 }, LP_GRID_RANGE },
};

static void lpXPoly_MakeRange( lpXPoly* poly )
{
	for ( int f = 0; f < 6; ++f )
	{
		lpXFace* face = poly->faces + f;
		face->plane = lp_rangePlanes[f];
		face->first = (uint16_t)( 4 * f );
		face->count = 4;
		face->material = 0;
		face->tag = LP_TAG_RANGE;
		memcpy( poly->indices + 4 * f, lp_xBoxLoops[f], 4 );
	}
	for ( int i = 0; i < 8; ++i )
	{
		uint8_t* t = poly->triples[i];
		t[0] = ( i & 1 ) ? 0 : 1;
		t[1] = ( i & 2 ) ? 2 : 3;
		t[2] = ( i & 4 ) ? 4 : 5;
		bool met = lpIVertex_FromPlanes( &lp_rangePlanes[t[0]], &lp_rangePlanes[t[1]], &lp_rangePlanes[t[2]],
										 poly->vertices + i );
		LP_ASSERT( met );
		(void)met;
		lpXApprox( poly->vertices + i, poly->approx[i] );
	}
	poly->vertexCount = 8;
	poly->faceCount = 6;
	poly->indexCount = 24;
}

lpXBuild lpXPoly_FromPlanes( const lpIPlane* planes, const uint8_t* materials, const int32_t* tags, int count, lpXPoly* out,
							 lpXPoly* scratch )
{
	for ( int i = 0; i < count; ++i )
	{
		if ( lpIPlane_IsValid( planes + i ) == false )
		{
			return lp_xBadPlane;
		}
	}
	lpXPoly* current = out;
	lpXPoly* next = scratch;
	lpXPoly_MakeRange( current );
	for ( int i = 0; i < count; ++i )
	{
		uint8_t material = materials != NULL ? materials[i] : 0;
		int32_t tag = tags != NULL ? tags[i] : LP_TAG_EXTERIOR;
		// A plane on a face of the range box leaves the box unchanged: that face becomes the plane's
		for ( int f = 0; f < current->faceCount; ++f )
		{
			lpXFace* face = current->faces + f;
			if ( face->tag == LP_TAG_RANGE && lpIPlane_Equal( &face->plane, planes + i ) )
			{
				face->material = material;
				face->tag = tag;
			}
		}
		lpClipResult result = lpXPoly_Clip( current, planes + i, material, tag, next );
		if ( result == lp_clipCut )
		{
			lpXPoly* t = current;
			current = next;
			next = t;
		}
		else if ( result == lp_clipEmpty )
		{
			return lp_xEmpty;
		}
		else if ( result != lp_clipUnchanged )
		{
			return lp_xOverflow; // lp_clipFailed cannot happen on the box and its cuts
		}
	}
	for ( int f = 0; f < current->faceCount; ++f )
	{
		if ( current->faces[f].tag == LP_TAG_RANGE )
		{
			return lp_xUnbounded;
		}
	}
	if ( current != out )
	{
		lpXPoly_Copy( out, current );
	}
	return lp_xBuilt;
}

lpXBuild lpXPoly_MakeBox( lpVec3 h, lpTransform transform, uint8_t material, lpXPoly* out, lpXPoly* scratch )
{
	static const lpVec3 axes[6] = {
		{ 1.0f, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f },
		{ 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, -1.0f },
	};
	lpIPlane planes[6];
	uint8_t materials[6];
	for ( int f = 0; f < 6; ++f )
	{
		lpVec3 e = axes[f];
		lpVec3 centre = lpTransformPoint( transform, (lpVec3){ e.x * h.x, e.y * h.y, e.z * h.z } );
		int32_t p[3];
		if ( lpGeom_GridPoint( centre.x, centre.y, centre.z, p ) == false ||
			 lpIPlane_MakeSnapped( lpRotateVector( transform.q, e ), LP_XPOLY_SNAP_BITS, p, planes + f, NULL ) == false )
		{
			return lp_xBadPlane;
		}
		materials[f] = material;
	}
	return lpXPoly_FromPlanes( planes, materials, NULL, 6, out, scratch );
}

lpXBuild lpXPoly_FromPoly( const lpPoly* poly, lpVec3 origin, lpXPoly* out, lpXPoly* scratch )
{
	lpIPlane planes[LP_POLY_MAX_FACES];
	uint8_t materials[LP_POLY_MAX_FACES];
	int32_t tags[LP_POLY_MAX_FACES];
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		const lpFace* face = poly->faces + f;
		double m[3] = { 0.0, 0.0, 0.0 };
		for ( int k = 0; k < face->count; ++k )
		{
			lpVec3 v = poly->vertices[poly->indices[face->first + k]];
			m[0] += (double)v.x;
			m[1] += (double)v.y;
			m[2] += (double)v.z;
		}
		double inv = 1.0 / (double)face->count;
		int32_t p[3];
		if ( lpGeom_GridPoint( m[0] * inv + (double)origin.x, m[1] * inv + (double)origin.y, m[2] * inv + (double)origin.z, p ) ==
				 false ||
			 lpIPlane_MakeSnapped( face->plane.normal, LP_XPOLY_SNAP_BITS, p, planes + f, NULL ) == false )
		{
			return lp_xBadPlane;
		}
		materials[f] = face->material;
		tags[f] = face->tag;
	}
	return lpXPoly_FromPlanes( planes, materials, tags, poly->faceCount, out, scratch );
}

// ---- floats and masses ----

void lpXPoly_Round( const lpXPoly* poly, lpVec3* rounded )
{
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		rounded[i] = lpIVertex_RoundToFloat( poly->vertices + i );
	}
}

// Volume and centroid of loops over points in doubles, tetrahedra from the points' mean (as lpPoly_ComputeMass)
static void lpXMass( const lpXPoly* poly, const double ( *p )[3], double* volume, double centroid[3] )
{
	double r[3] = { 0.0, 0.0, 0.0 };
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		r[0] += p[i][0];
		r[1] += p[i][1];
		r[2] += p[i][2];
	}
	double inv = 1.0 / (double)poly->vertexCount;
	r[0] *= inv;
	r[1] *= inv;
	r[2] *= inv;
	double v6 = 0.0;
	double c[3] = { 0.0, 0.0, 0.0 };
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		const lpXFace* face = poly->faces + f;
		const uint8_t* loop = poly->indices + face->first;
		double a[3] = { p[loop[0]][0] - r[0], p[loop[0]][1] - r[1], p[loop[0]][2] - r[2] };
		for ( int k = 1; k + 1 < face->count; ++k )
		{
			double b[3] = { p[loop[k]][0] - r[0], p[loop[k]][1] - r[1], p[loop[k]][2] - r[2] };
			double d[3] = { p[loop[k + 1]][0] - r[0], p[loop[k + 1]][1] - r[1], p[loop[k + 1]][2] - r[2] };
			double t = a[0] * ( b[1] * d[2] - b[2] * d[1] ) + a[1] * ( b[2] * d[0] - b[0] * d[2] ) +
					   a[2] * ( b[0] * d[1] - b[1] * d[0] );
			v6 += t;
			for ( int i = 0; i < 3; ++i )
			{
				c[i] += t * ( a[i] + b[i] + d[i] );
			}
		}
	}
	*volume = v6 / 6.0;
	for ( int i = 0; i < 3; ++i )
	{
		centroid[i] = v6 > 0.0 ? r[i] + c[i] / ( 4.0 * v6 ) : r[i];
	}
}

void lpXPoly_ComputeMass( const lpXPoly* poly, const lpVec3* rounded, double* volume, double centroid[3] )
{
	double p[LP_XPOLY_MAX_VERTICES][3];
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		p[i][0] = (double)rounded[i].x;
		p[i][1] = (double)rounded[i].y;
		p[i][2] = (double)rounded[i].z;
	}
	lpXMass( poly, (const double( * )[3])p, volume, centroid );
}

void lpXPoly_ComputeMassPrecise( const lpXPoly* poly, double* volume, double centroid[3] )
{
	// Offsets from the grid point g nearest the first vertex: x - g w is exact (|g w| < 2^98), one rounding to double
	int64_t g[3];
	for ( int i = 0; i < 3; ++i )
	{
		g[i] = (int64_t)floor( poly->approx[0][i] + 0.5 ); // any grid point near the polyhedron
	}
	double p[LP_XPOLY_MAX_VERTICES][3];
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		const lpIVertex* v = poly->vertices + i;
		double w = lpI128_ToDouble( v->w );
		p[i][0] = lpI128_ToDouble( lpI128_Sub( v->x, lpI128_Mul( v->w, g[0] ) ) ) / w;
		p[i][1] = lpI128_ToDouble( lpI128_Sub( v->y, lpI128_Mul( v->w, g[1] ) ) ) / w;
		p[i][2] = lpI128_ToDouble( lpI128_Sub( v->z, lpI128_Mul( v->w, g[2] ) ) ) / w;
	}
	double grid[3];
	lpXMass( poly, (const double( * )[3])p, volume, grid );
	const double u = 1.0 / 65536.0;
	*volume *= u * u * u;
	for ( int i = 0; i < 3; ++i )
	{
		centroid[i] = ( (double)g[i] + grid[i] ) * u;
	}
}

void lpXPoly_ToPoly( const lpXPoly* poly, const lpVec3* rounded, lpPoly* out )
{
	out->vertexCount = poly->vertexCount;
	out->faceCount = poly->faceCount;
	out->indexCount = poly->indexCount;
	memcpy( out->vertices, rounded, sizeof( lpVec3 ) * (size_t)poly->vertexCount );
	memcpy( out->indices, poly->indices, (size_t)poly->indexCount );
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		const lpXFace* x = poly->faces + f;
		lpFace* face = out->faces + f;
		double n[3] = { (double)x->plane.n[0], (double)x->plane.n[1], (double)x->plane.n[2] };
		double length = sqrt( n[0] * n[0] + n[1] * n[1] + n[2] * n[2] ); // n.n < 2^50: exact before the root
		face->plane.normal = (lpVec3){ (float)( n[0] / length ), (float)( n[1] / length ), (float)( n[2] / length ) };
		face->plane.offset = (float)( (double)x->plane.d / length / 65536.0 );
		face->first = x->first;
		face->count = x->count;
		face->material = x->material;
		face->tag = x->tag;
	}
}

// ---- checks ----

uint64_t lpXPoly_Digest( const lpXPoly* poly )
{
	int counts[3] = { poly->vertexCount, poly->faceCount, poly->indexCount };
	uint64_t h = lpHashWords( LP_HASH_INIT, counts, sizeof( counts ) );
	h = lpHashWords( h, poly->vertices, sizeof( lpIVertex ) * (size_t)poly->vertexCount ); // four words each, no padding
	h = lpHashWords( h, poly->triples, sizeof( poly->triples[0] ) * (size_t)poly->vertexCount );
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		const lpXFace* face = poly->faces + f;
		int64_t fields[7] = { face->plane.n[0], face->plane.n[1], face->plane.n[2], face->plane.d, face->first, face->count,
							  (int64_t)face->material << 32 | (uint32_t)face->tag };
		h = lpHashWords( h, fields, sizeof( fields ) );
	}
	h = lpHashWords( h, poly->indices, (size_t)poly->indexCount );
	return lpMix64( h );
}

bool lpXPoly_SamePoint( const lpXPoly* a, int va, const lpXPoly* b, int vb )
{
	const uint8_t* t = b->triples[vb];
	for ( int i = 0; i < 3; ++i )
	{
		if ( lpIVertex_Classify( a->vertices + va, &b->faces[t[i]].plane ) != 0 )
		{
			return false;
		}
	}
	return true;
}

const char* lpXPoly_Validate( const lpXPoly* poly )
{
	int vertexCount = poly->vertexCount;
	int faceCount = poly->faceCount;
	if ( vertexCount < 4 || vertexCount > LP_XPOLY_MAX_VERTICES || faceCount < 4 || faceCount > LP_XPOLY_MAX_FACES ||
		 poly->indexCount < 12 || poly->indexCount > LP_XPOLY_MAX_INDICES )
	{
		return "counts out of range";
	}
	for ( int f = 0; f < faceCount; ++f )
	{
		if ( lpIPlane_IsValid( &poly->faces[f].plane ) == false )
		{
			return "a face's plane is out of range or not canonical";
		}
		for ( int g = 0; g < f; ++g )
		{
			if ( lpIPlane_Equal( &poly->faces[f].plane, &poly->faces[g].plane ) )
			{
				return "two faces on one plane";
			}
		}
	}

	// Loops: three vertices or more, in range, none twice; which faces hold each vertex
	uint64_t holds[LP_XPOLY_MAX_VERTICES] = { 0 };
	int edgeCount = 0;
	for ( int f = 0; f < faceCount; ++f )
	{
		const lpXFace* face = poly->faces + f;
		if ( face->count < 3 )
		{
			return "a face with fewer than three vertices";
		}
		if ( face->first + face->count > poly->indexCount )
		{
			return "a loop past the indices";
		}
		const uint8_t* loop = poly->indices + face->first;
		for ( int k = 0; k < face->count; ++k )
		{
			if ( loop[k] >= vertexCount )
			{
				return "an index past the vertices";
			}
			if ( holds[loop[k]] >> f & 1u )
			{
				return "a vertex twice in one loop";
			}
			holds[loop[k]] |= 1ull << f;
		}
		edgeCount += face->count;
	}

	// Incidence: each vertex exactly on the faces holding it, strictly inside every other face's plane; its triple three
	// of those faces whose planes meet in one point (then the cached point, on all three, is that point)
	for ( int v = 0; v < vertexCount; ++v )
	{
		if ( lpI128_Sign( poly->vertices[v].w ) <= 0 )
		{
			return "a vertex with w <= 0";
		}
		for ( int f = 0; f < faceCount; ++f )
		{
			int side = lpIVertex_Classify( poly->vertices + v, &poly->faces[f].plane );
			if ( ( holds[v] >> f & 1u ) != 0 && side != 0 )
			{
				return "a vertex off the plane of a face holding it";
			}
			if ( ( holds[v] >> f & 1u ) == 0 && side >= 0 )
			{
				return "a vertex not strictly inside the plane of a face not holding it";
			}
		}
		const uint8_t* t = poly->triples[v];
		if ( t[0] >= faceCount || t[1] >= faceCount || t[2] >= faceCount || t[0] == t[1] || t[1] == t[2] || t[0] == t[2] )
		{
			return "a triple out of range or repeating a face";
		}
		if ( ( holds[v] >> t[0] & 1u ) == 0 || ( holds[v] >> t[1] & 1u ) == 0 || ( holds[v] >> t[2] & 1u ) == 0 )
		{
			return "a triple naming a face that does not hold its vertex";
		}
		lpIVertex check;
		if ( lpIVertex_FromPlanes( &poly->faces[t[0]].plane, &poly->faces[t[1]].plane, &poly->faces[t[2]].plane, &check ) ==
			 false )
		{
			return "a triple whose planes do not meet in one point";
		}
	}

	// A closed 2-manifold: every directed edge once, its reverse once in another face
	for ( int f = 0; f < faceCount; ++f )
	{
		const lpXFace* face = poly->faces + f;
		const uint8_t* loop = poly->indices + face->first;
		for ( int k = 0; k < face->count; ++k )
		{
			int a = loop[k];
			int b = loop[k + 1 < face->count ? k + 1 : 0];
			int forward = 0;
			int reverse = 0;
			for ( int g = 0; g < faceCount; ++g )
			{
				const lpXFace* other = poly->faces + g;
				const uint8_t* otherLoop = poly->indices + other->first;
				for ( int j = 0; j < other->count; ++j )
				{
					int c = otherLoop[j];
					int e = otherLoop[j + 1 < other->count ? j + 1 : 0];
					forward += c == a && e == b ? 1 : 0;
					reverse += c == b && e == a ? ( g != f ? 1 : 2 ) : 0;
				}
			}
			if ( forward != 1 || reverse != 1 )
			{
				return "not a closed 2-manifold with consistent loops";
			}
		}
	}
	if ( vertexCount - edgeCount / 2 + faceCount != 2 )
	{
		return "Euler characteristic not 2";
	}

	// Consistent loops are all counter clockwise from outside, or all clockwise: the volume's sign says which
	double volume, centroid[3];
	lpXPoly_ComputeMassPrecise( poly, &volume, centroid );
	if ( ( volume > 0.0 ) == false )
	{
		return "loops clockwise from outside (volume not positive)";
	}
	return NULL;
}
