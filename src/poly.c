// SPDX-License-Identifier: MIT

#include "poly.h"

#include <float.h>

// Box vertex i has coordinates (i&1 ? +x : -x, i&2 ? +y : -y, i&4 ? +z : -z).
// Loops are counter clockwise seen from outside.
static const uint8_t lp_boxLoops[6][4] = {
	{ 1, 3, 7, 5 }, // +x
	{ 0, 4, 6, 2 }, // -x
	{ 2, 6, 7, 3 }, // +y
	{ 0, 1, 5, 4 }, // -y
	{ 4, 5, 7, 6 }, // +z
	{ 0, 2, 3, 1 }, // -z
};

static const lpVec3 lp_boxNormals[6] = {
	{ 1.0f, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f },
	{ 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, -1.0f },
};

void lpPoly_MakeBox( lpPoly* poly, lpVec3 h, lpTransform transform, uint8_t material )
{
	for ( int i = 0; i < 8; ++i )
	{
		lpVec3 p = { ( i & 1 ) ? h.x : -h.x, ( i & 2 ) ? h.y : -h.y, ( i & 4 ) ? h.z : -h.z };
		poly->vertices[i] = lpTransformPoint( transform, p );
	}
	poly->vertexCount = 8;

	for ( int f = 0; f < 6; ++f )
	{
		lpFace* face = poly->faces + f;
		lpVec3 n = lpRotateVector( transform.q, lp_boxNormals[f] );
		face->plane.normal = n;
		face->plane.offset = lpDot( n, poly->vertices[lp_boxLoops[f][0]] );
		face->first = (uint16_t)( 4 * f );
		face->count = 4;
		face->material = material;
		face->tag = LP_TAG_EXTERIOR;
		for ( int k = 0; k < 4; ++k )
		{
			poly->indices[4 * f + k] = lp_boxLoops[f][k];
		}
	}
	poly->faceCount = 6;
	poly->indexCount = 24;
}

bool lpPoly_MakeFromHull( lpPoly* poly, const lpPhysHull* hull, uint8_t material )
{
	if ( hull == NULL )
	{
		return false;
	}
	int vertexCount = lpPhys_GetHullVertexCount( hull );
	int faceCount = lpPhys_GetHullFaceCount( hull );
	if ( vertexCount > LP_POLY_MAX_VERTICES || faceCount > LP_POLY_MAX_FACES )
	{
		return false;
	}

	for ( int i = 0; i < vertexCount; ++i )
	{
		poly->vertices[i] = lpPhys_GetHullPoint( hull, i );
	}
	poly->vertexCount = vertexCount;

	int indexCount = 0;
	for ( int f = 0; f < faceCount; ++f )
	{
		lpFace* face = poly->faces + f;
		face->first = (uint16_t)indexCount;
		face->material = material;
		face->tag = LP_TAG_EXTERIOR;
		face->plane = lpPhys_GetHullPlane( hull, f );
		int room = LP_POLY_MAX_INDICES - indexCount;
		int count = lpPhys_GetHullFace( hull, f, poly->indices + indexCount, room < 255 ? room : 255 );
		if ( count < 0 )
		{
			return false;
		}
		indexCount += count;
		face->count = (uint8_t)count;
	}
	poly->faceCount = faceCount;
	poly->indexCount = indexCount;
	return true;
}

bool lpPoly_MakeFromPoints( lpPoly* poly, const lpVec3* points, int count, uint8_t material )
{
	lpPhysHull* hull = lpPhys_CreateHull( points, count, LP_POLY_MAX_VERTICES );
	if ( hull == NULL )
	{
		return false;
	}
	bool ok = lpPoly_MakeFromHull( poly, hull, material );
	lpPhys_DestroyHull( hull );
	return ok;
}

void lpPoly_Translate( lpPoly* poly, lpVec3 translation )
{
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		poly->vertices[i] = lpAdd( poly->vertices[i], translation );
	}
	for ( int i = 0; i < poly->faceCount; ++i )
	{
		lpPlane* plane = &poly->faces[i].plane;
		plane->offset += lpDot( plane->normal, translation );
	}
}

// Plane through a convex face loop by Newell's method: robust for any vertex order and count.
static lpPlane lpNewellPlane( const lpPoly* poly, const lpFace* face )
{
	lpVec3 n = lpVec3_zero;
	lpVec3 c = lpVec3_zero;
	for ( int k = 0; k < face->count; ++k )
	{
		lpVec3 a = poly->vertices[poly->indices[face->first + k]];
		lpVec3 b = poly->vertices[poly->indices[face->first + ( k + 1 ) % face->count]];
		n.x += ( a.y - b.y ) * ( a.z + b.z );
		n.y += ( a.z - b.z ) * ( a.x + b.x );
		n.z += ( a.x - b.x ) * ( a.y + b.y );
		c = lpAdd( c, a );
	}
	c = lpMulSV( 1.0f / (float)face->count, c );
	n = lpNormalize( n );
	return (lpPlane){ n, lpDot( n, c ) };
}

void lpPoly_ApplyLinear( lpPoly* poly, lpMatrix3 m )
{
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		poly->vertices[i] = lpMulMV( m, poly->vertices[i] );
	}
	for ( int i = 0; i < poly->faceCount; ++i )
	{
		poly->faces[i].plane = lpNewellPlane( poly, poly->faces + i );
	}
}

// The clip; stats (NULL: none) counts what it did (the integers and the largest shift never feed back into it)
static lpClipResult lpClip( const lpPoly* in, lpPlane plane, uint8_t material, int32_t tag, float tolerance, lpPoly* out,
							lpClipStats* stats )
{
	LP_ASSERT( in != out );

	int vertexCount = in->vertexCount;
	float s[LP_POLY_MAX_VERTICES];
	float minS = FLT_MAX;
	float maxS = -FLT_MAX;
	for ( int i = 0; i < vertexCount; ++i )
	{
		float si = lpDot( plane.normal, in->vertices[i] ) - plane.offset;
		s[i] = si;
		minS = si < minS ? si : minS;
		maxS = si > maxS ? si : maxS;
	}

	if ( maxS <= tolerance )
	{
		if ( stats != NULL && maxS > 0.0f )
		{
			stats->toleranceOuts += 1; // a vertex outside, within tolerance: kept whole
		}
		return lp_clipUnchanged;
	}
	if ( minS >= -tolerance )
	{
		if ( stats != NULL && minS < 0.0f )
		{
			stats->toleranceOuts += 1; // a vertex inside, within tolerance: dropped whole
		}
		return lp_clipEmpty;
	}

	// Push the plane outward until no vertex is within tolerance. After this no vertex, edge or face lies
	// in the plane, so every crossing is a proper edge crossing and the cap is a simple convex loop.
	// Technique borrowed from Nebenan (github.com/Holz231/Nebenan, MIT).
	float shift = 0.0f;
	for ( int iteration = 0; iteration < 8; ++iteration )
	{
		bool isClose = false;
		for ( int i = 0; i < vertexCount; ++i )
		{
			if ( lpAbsFloat( s[i] - shift ) < tolerance )
			{
				isClose = true;
				break;
			}
		}
		if ( isClose == false )
		{
			break;
		}
		shift += 2.0f * tolerance;
	}
	if ( stats != NULL && shift > 0.0f )
	{
		stats->shifts += 1;
		stats->maxShift = shift > stats->maxShift ? shift : stats->maxShift;
	}

	int insideCount = 0;
	for ( int i = 0; i < vertexCount; ++i )
	{
		float si = s[i] - shift;
		if ( lpAbsFloat( si ) < tolerance )
		{
			si = -tolerance;
		}
		s[i] = si;
		insideCount += si < 0.0f ? 1 : 0;
	}

	if ( insideCount == vertexCount || insideCount == 0 )
	{
		if ( stats != NULL )
		{
			stats->toleranceOuts += 1; // the shift and the snap decided it
		}
		return insideCount == vertexCount ? lp_clipUnchanged : lp_clipEmpty;
	}

	// Keep inside vertices
	int map[LP_POLY_MAX_VERTICES];
	int outVertexCount = 0;
	for ( int i = 0; i < vertexCount; ++i )
	{
		if ( s[i] < 0.0f )
		{
			map[i] = outVertexCount;
			out->vertices[outVertexCount++] = in->vertices[i];
		}
		else
		{
			map[i] = -1;
		}
	}

	// Crossing edges, keyed by the sorted vertex pair so both faces sharing an edge get the same new
	// vertex, computed in the same order (bit-identical regardless of which face comes first).
	enum
	{
		lp_maxCrossings = 128
	};
	uint8_t crossA[lp_maxCrossings];
	uint8_t crossB[lp_maxCrossings];
	int crossVertex[lp_maxCrossings];
	int crossCount = 0;

	// cap loop links: capNext[entry vertex] = exit vertex (the cap walks each cut edge backwards)
	int capNext[LP_POLY_MAX_VERTICES];
	for ( int i = 0; i < LP_POLY_MAX_VERTICES; ++i )
	{
		capNext[i] = -1;
	}

	int outFaceCount = 0;
	int outIndexCount = 0;

	for ( int f = 0; f < in->faceCount; ++f )
	{
		const lpFace* face = in->faces + f;
		int first = outIndexCount;
		int exitVertex = -1;
		int entryVertex = -1;

		for ( int k = 0; k < face->count; ++k )
		{
			int cur = in->indices[face->first + k];
			int next = in->indices[face->first + ( k + 1 ) % face->count];
			bool curIn = s[cur] < 0.0f;
			bool nextIn = s[next] < 0.0f;

			if ( curIn )
			{
				if ( outIndexCount >= LP_POLY_MAX_INDICES )
				{
					return lp_clipOverflow;
				}
				out->indices[outIndexCount++] = (uint8_t)map[cur];
			}

			if ( curIn != nextIn )
			{
				int a = cur < next ? cur : next;
				int b = cur < next ? next : cur;
				int v = -1;
				for ( int c = 0; c < crossCount; ++c )
				{
					if ( crossA[c] == a && crossB[c] == b )
					{
						v = crossVertex[c];
						break;
					}
				}
				if ( v < 0 )
				{
					if ( crossCount >= lp_maxCrossings || outVertexCount >= LP_POLY_MAX_VERTICES )
					{
						return lp_clipOverflow;
					}
					float t = s[a] / ( s[a] - s[b] );
					lpVec3 pa = in->vertices[a];
					lpVec3 pb = in->vertices[b];
					v = outVertexCount++;
					out->vertices[v] = lpAdd( pa, lpMulSV( t, lpSub( pb, pa ) ) );
					crossA[crossCount] = (uint8_t)a;
					crossB[crossCount] = (uint8_t)b;
					crossVertex[crossCount] = v;
					crossCount += 1;
				}

				if ( outIndexCount >= LP_POLY_MAX_INDICES )
				{
					return lp_clipOverflow;
				}
				out->indices[outIndexCount++] = (uint8_t)v;

				if ( curIn )
				{
					exitVertex = v;
				}
				else
				{
					entryVertex = v;
				}
			}
		}

		int count = outIndexCount - first;
		if ( count == 0 )
		{
			continue; // face entirely outside
		}

		if ( count < 3 || outFaceCount >= LP_POLY_MAX_FACES )
		{
			return count < 3 ? lp_clipFailed : lp_clipOverflow;
		}

		lpFace* outFace = out->faces + outFaceCount++;
		*outFace = *face;
		outFace->first = (uint16_t)first;
		outFace->count = (uint8_t)count;

		if ( exitVertex >= 0 || entryVertex >= 0 )
		{
			if ( exitVertex < 0 || entryVertex < 0 )
			{
				return lp_clipFailed;
			}
			// The face loop runs ... exit -> entry ... along the cut; the cap runs entry -> exit.
			if ( capNext[entryVertex] != -1 )
			{
				return lp_clipFailed;
			}
			capNext[entryVertex] = exitVertex;
		}
	}

	// Cap face: chain the cut edges into one loop
	if ( crossCount < 3 || outFaceCount >= LP_POLY_MAX_FACES || outIndexCount + crossCount > LP_POLY_MAX_INDICES )
	{
		return crossCount < 3 ? lp_clipFailed : lp_clipOverflow;
	}

	int capFirst = outIndexCount;
	int start = crossVertex[0];
	int v = start;
	int capCount = 0;
	do
	{
		if ( v < 0 || capCount >= crossCount )
		{
			return lp_clipFailed;
		}
		out->indices[outIndexCount++] = (uint8_t)v;
		capCount += 1;
		v = capNext[v];
	}
	while ( v != start );

	if ( capCount != crossCount )
	{
		return lp_clipFailed;
	}

	lpFace* cap = out->faces + outFaceCount++;
	cap->plane.normal = plane.normal;
	cap->plane.offset = plane.offset + shift;
	cap->first = (uint16_t)capFirst;
	cap->count = (uint8_t)capCount;
	cap->material = material;
	cap->tag = tag;

	out->vertexCount = outVertexCount;
	out->faceCount = outFaceCount;
	out->indexCount = outIndexCount;
	return lp_clipCut;
}

lpClipResult lpPoly_Clip( const lpPoly* in, lpPlane plane, uint8_t material, int32_t tag, float tolerance, lpPoly* out )
{
	return lpClip( in, plane, material, tag, tolerance, out, NULL );
}

lpClipResult lpPoly_ClipCounted( const lpPoly* in, lpPlane plane, uint8_t material, int32_t tag, float tolerance, lpPoly* out,
								 lpClipStats* stats )
{
	lpClipResult result = lpClip( in, plane, material, tag, tolerance, out, stats );
	if ( stats != NULL )
	{
		stats->clips += 1;
		stats->failures += result == lp_clipFailed || result == lp_clipOverflow ? 1 : 0;
	}
	return result;
}

void lpPoly_ComputeMass( const lpPoly* poly, float* volume, lpVec3* centroid )
{
	// Reference point inside the polyhedron keeps the tetrahedra small, for precision
	lpVec3 r = lpVec3_zero;
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		r = lpAdd( r, poly->vertices[i] );
	}
	r = lpMulSV( 1.0f / (float)poly->vertexCount, r );

	float v6 = 0.0f;
	lpVec3 c = lpVec3_zero;
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		const lpFace* face = poly->faces + f;
		lpVec3 a = lpSub( poly->vertices[poly->indices[face->first]], r );
		for ( int k = 1; k + 1 < face->count; ++k )
		{
			lpVec3 b = lpSub( poly->vertices[poly->indices[face->first + k]], r );
			lpVec3 d = lpSub( poly->vertices[poly->indices[face->first + k + 1]], r );
			float t = lpDot( a, lpCross( b, d ) );
			v6 += t;
			c = lpAdd( c, lpMulSV( t, lpAdd( lpAdd( a, b ), d ) ) );
		}
	}

	*volume = v6 / 6.0f;
	if ( v6 > 0.0f )
	{
		// centroid of tetra (r, a, b, d) is r + (a + b + d) / 4
		*centroid = lpAdd( r, lpMulSV( 1.0f / ( 4.0f * v6 ), c ) );
	}
	else
	{
		*centroid = r;
	}
}

lpAABB lpPoly_ComputeBounds( const lpPoly* poly )
{
	lpAABB box = { poly->vertices[0], poly->vertices[0] };
	for ( int i = 1; i < poly->vertexCount; ++i )
	{
		box.lowerBound = lpMin( box.lowerBound, poly->vertices[i] );
		box.upperBound = lpMax( box.upperBound, poly->vertices[i] );
	}
	return box;
}

float lpPoly_MaxDistanceSquared( const lpPoly* poly, lpVec3 point )
{
	float m = 0.0f;
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		float d = lpDistanceSquared( poly->vertices[i], point );
		m = d > m ? d : m;
	}
	return m;
}

static float lpPlanesDistance( const lpFace* faces, int faceCount, lpVec3 point )
{
	float m = -FLT_MAX;
	for ( int f = 0; f < faceCount; ++f )
	{
		float d = lpDot( faces[f].plane.normal, point ) - faces[f].plane.offset;
		m = d > m ? d : m;
	}
	return m;
}

float lpPoly_SignedDistance( const lpPoly* poly, lpVec3 point )
{
	return lpPlanesDistance( poly->faces, poly->faceCount, point );
}

bool lpPoly_IsValid( const lpPoly* poly, float tolerance )
{
	if ( poly->vertexCount < 4 || poly->faceCount < 4 )
	{
		return false;
	}

	int edgeCount = 0;
	int used[LP_POLY_MAX_VERTICES] = { 0 };

	for ( int f = 0; f < poly->faceCount; ++f )
	{
		const lpFace* face = poly->faces + f;
		if ( face->count < 3 || face->first + face->count > poly->indexCount )
		{
			return false;
		}
		if ( lpAbsFloat( lpLength( face->plane.normal ) - 1.0f ) > 1e-3f )
		{
			return false;
		}

		for ( int k = 0; k < face->count; ++k )
		{
			int a = poly->indices[face->first + k];
			int b = poly->indices[face->first + ( k + 1 ) % face->count];
			if ( a >= poly->vertexCount || a == b )
			{
				return false;
			}
			used[a] = 1;

			// planar
			float d = lpDot( face->plane.normal, poly->vertices[a] ) - face->plane.offset;
			if ( lpAbsFloat( d ) > tolerance )
			{
				return false;
			}

			// the reverse edge b->a must appear exactly once in another face
			int reverse = 0;
			for ( int g = 0; g < poly->faceCount; ++g )
			{
				const lpFace* other = poly->faces + g;
				for ( int j = 0; j < other->count; ++j )
				{
					int c = poly->indices[other->first + j];
					int e = poly->indices[other->first + ( j + 1 ) % other->count];
					if ( c == b && e == a )
					{
						reverse += ( g != f ) ? 1 : 2;
					}
				}
			}
			if ( reverse != 1 )
			{
				return false;
			}
			edgeCount += 1;
		}
	}

	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		if ( used[i] == 0 )
		{
			return false;
		}
		// convex: behind every plane
		for ( int f = 0; f < poly->faceCount; ++f )
		{
			float d = lpDot( poly->faces[f].plane.normal, poly->vertices[i] ) - poly->faces[f].plane.offset;
			if ( d > tolerance )
			{
				return false;
			}
		}
	}

	// Euler: V - E + F = 2, with E counted once per undirected edge
	int e = edgeCount / 2;
	if ( poly->vertexCount - e + poly->faceCount != 2 )
	{
		return false;
	}

	float volume;
	lpVec3 centroid;
	lpPoly_ComputeMass( poly, &volume, &centroid );
	return volume > 0.0f;
}

// The geometry's digest, field by field (faces have padding to keep out)
static uint64_t lpShapeDigest( const lpShape* shape )
{
	int counts[3] = { shape->vertexCount, shape->faceCount, shape->indexCount };
	uint64_t h = lpHashWords( LP_HASH_INIT, counts, sizeof( counts ) );
	h = lpHashWords( h, shape->vertices, sizeof( lpVec3 ) * (size_t)shape->vertexCount );
	for ( int i = 0; i < shape->faceCount; ++i )
	{
		const lpFace* f = shape->faces + i;
		float plane[4] = { f->plane.normal.x, f->plane.normal.y, f->plane.normal.z, f->plane.offset };
		int32_t loop[4] = { f->first, f->count, f->material, f->tag };
		h = lpHashWords( h, plane, sizeof( plane ) );
		h = lpHashWords( h, loop, sizeof( loop ) );
	}
	h = lpHashWords( h, shape->indices, (size_t)shape->indexCount );
	float mass[4] = { shape->volume, shape->centroid.x, shape->centroid.y, shape->centroid.z };
	return lpMix64( lpHashWords( h, mass, sizeof( mass ) ) );
}

lpShape* lpShape_Create( const lpPoly* poly )
{
	float volume;
	lpVec3 centroid;
	lpPoly_ComputeMass( poly, &volume, &centroid );
	if ( ( volume > 0.0f ) == false )
	{
		return NULL;
	}

	size_t header = ( sizeof( lpShape ) + 15 ) & ~(size_t)15;
	size_t vertexBytes = sizeof( lpVec3 ) * (size_t)poly->vertexCount;
	size_t faceBytes = sizeof( lpFace ) * (size_t)poly->faceCount;
	size_t indexBytes = (size_t)poly->indexCount;
	uint8_t* memory = lpAlloc( header + vertexBytes + faceBytes + indexBytes );

	lpShape* shape = (lpShape*)memory;
	shape->faces = (lpFace*)( memory + header );
	shape->vertices = (lpVec3*)( memory + header + faceBytes );
	shape->indices = memory + header + faceBytes + vertexBytes;
	shape->vertexCount = poly->vertexCount;
	shape->faceCount = poly->faceCount;
	shape->indexCount = poly->indexCount;
	memcpy( shape->vertices, poly->vertices, vertexBytes );
	memcpy( shape->faces, poly->faces, faceBytes );
	memcpy( shape->indices, poly->indices, indexBytes );

	shape->bounds = lpPoly_ComputeBounds( poly );
	shape->centroid = centroid;
	shape->volume = volume;
	shape->radius = sqrtf( lpPoly_MaxDistanceSquared( poly, centroid ) );
	shape->digest = lpShapeDigest( shape );
	return shape;
}

void lpShape_Destroy( lpShape* shape )
{
	lpFree( shape );
}

void lpShape_ToPoly( const lpShape* shape, lpPoly* poly )
{
	poly->vertexCount = shape->vertexCount;
	poly->faceCount = shape->faceCount;
	poly->indexCount = shape->indexCount;
	memcpy( poly->vertices, shape->vertices, sizeof( lpVec3 ) * (size_t)shape->vertexCount );
	memcpy( poly->faces, shape->faces, sizeof( lpFace ) * (size_t)shape->faceCount );
	memcpy( poly->indices, shape->indices, (size_t)shape->indexCount );
}

void lpShape_Translate( lpShape* shape, lpVec3 translation )
{
	for ( int i = 0; i < shape->vertexCount; ++i )
	{
		shape->vertices[i] = lpAdd( shape->vertices[i], translation );
	}
	for ( int i = 0; i < shape->faceCount; ++i )
	{
		lpPlane* plane = &shape->faces[i].plane;
		plane->offset += lpDot( plane->normal, translation );
	}
	shape->bounds.lowerBound = lpAdd( shape->bounds.lowerBound, translation );
	shape->bounds.upperBound = lpAdd( shape->bounds.upperBound, translation );
	shape->centroid = lpAdd( shape->centroid, translation );
	shape->digest = lpShapeDigest( shape );
}

bool lpShape_HasFaceOnPlane( const lpShape* shape, lpPlane plane, float tolerance )
{
	for ( int i = 0; i < shape->faceCount; ++i )
	{
		lpPlane p = shape->faces[i].plane;
		if ( lpDot( p.normal, plane.normal ) > 0.999f && lpAbsFloat( p.offset - plane.offset ) < tolerance )
		{
			return true;
		}
	}
	return false;
}

float lpShape_SignedDistance( const lpShape* shape, lpVec3 point )
{
	return lpPlanesDistance( shape->faces, shape->faceCount, point );
}

float lpShape_FaceArea( const lpShape* shape, int faceIndex, lpVec3* centroid )
{
	const lpFace* face = shape->faces + faceIndex;
	lpVec3 a = shape->vertices[shape->indices[face->first]];
	float area2 = 0.0f;
	lpVec3 c = lpVec3_zero;
	for ( int k = 1; k + 1 < face->count; ++k )
	{
		lpVec3 b = shape->vertices[shape->indices[face->first + k]];
		lpVec3 d = shape->vertices[shape->indices[face->first + k + 1]];
		float t = lpDot( face->plane.normal, lpCross( lpSub( b, a ), lpSub( d, a ) ) );
		area2 += t;
		c = lpAdd( c, lpMulSV( t, lpAdd( lpAdd( a, b ), d ) ) );
	}
	if ( centroid != NULL )
	{
		*centroid = area2 > 0.0f ? lpMulSV( 1.0f / ( 3.0f * area2 ), c ) : a;
	}
	return 0.5f * area2;
}

bool lpShape_NearlyOverlap( const lpShape* a, const lpShape* b, float margin )
{
	for ( int i = 0; i < a->vertexCount; ++i )
	{
		if ( lpShape_SignedDistance( b, a->vertices[i] ) < margin )
		{
			return true;
		}
	}
	for ( int i = 0; i < b->vertexCount; ++i )
	{
		if ( lpShape_SignedDistance( a, b->vertices[i] ) < margin )
		{
			return true;
		}
	}

	// Two convex solids can cross edge to edge with no vertex inside the other (overlapping low-poly blobs): GJK
	if ( a->vertexCount > LP_PHYS_MAX_POINTS || b->vertexCount > LP_PHYS_MAX_POINTS )
	{
		return false;
	}
	return lpPhys_HullDistance( a->vertices, a->vertexCount, b->vertices, b->vertexCount ) < margin;
}

lpPhysHull* lpShape_CreateHull( const lpShape* shape )
{
	return lpPhys_CreateHull( shape->vertices, shape->vertexCount, LP_PHYS_MAX_POINTS );
}

// ---- contact area between coplanar opposing faces ----

typedef struct lpVec2
{
	float x, y;
} lpVec2;

enum
{
	lp_maxPolygon2 = 96
};

// Clip a convex 2D polygon by the half-plane on the left of edge (a -> b) scaled by orientation sign.
static int lpClipPolygon2( const lpVec2* in, int count, lpVec2 a, lpVec2 b, float sign, lpVec2* out )
{
	int n = 0;
	for ( int i = 0; i < count; ++i )
	{
		lpVec2 p = in[i];
		lpVec2 q = in[( i + 1 ) % count];
		float sp = sign * ( ( b.x - a.x ) * ( p.y - a.y ) - ( b.y - a.y ) * ( p.x - a.x ) );
		float sq = sign * ( ( b.x - a.x ) * ( q.y - a.y ) - ( b.y - a.y ) * ( q.x - a.x ) );
		if ( sp >= 0.0f )
		{
			if ( n < lp_maxPolygon2 )
			{
				out[n++] = p;
			}
		}
		if ( ( sp >= 0.0f ) != ( sq >= 0.0f ) )
		{
			float t = sp / ( sp - sq );
			if ( n < lp_maxPolygon2 )
			{
				out[n++] = (lpVec2){ p.x + t * ( q.x - p.x ), p.y + t * ( q.y - p.y ) };
			}
		}
	}
	return n;
}

static float lpPolygonArea2( const lpVec2* p, int count, lpVec2* centroid )
{
	float a2 = 0.0f;
	float cx = 0.0f;
	float cy = 0.0f;
	for ( int i = 0; i < count; ++i )
	{
		lpVec2 u = p[i];
		lpVec2 v = p[( i + 1 ) % count];
		float c = u.x * v.y - v.x * u.y;
		a2 += c;
		cx += ( u.x + v.x ) * c;
		cy += ( u.y + v.y ) * c;
	}
	if ( centroid != NULL && a2 != 0.0f )
	{
		centroid->x = cx / ( 3.0f * a2 );
		centroid->y = cy / ( 3.0f * a2 );
	}
	return 0.5f * a2;
}

// Half-extents of points around a centroid along the contact tangents
static void lpContactExtents( lpContact* contact, const lpVec3* points, int count )
{
	lpVec3 t1, t2;
	lpContactBasis( contact->normal, &t1, &t2 );
	float lo1 = FLT_MAX, hi1 = -FLT_MAX, lo2 = FLT_MAX, hi2 = -FLT_MAX;
	for ( int k = 0; k < count; ++k )
	{
		lpVec3 d = lpSub( points[k], contact->centroid );
		float p1 = lpDot( d, t1 );
		float p2 = lpDot( d, t2 );
		lo1 = p1 < lo1 ? p1 : lo1;
		hi1 = p1 > hi1 ? p1 : hi1;
		lo2 = p2 < lo2 ? p2 : lo2;
		hi2 = p2 > hi2 ? p2 : hi2;
	}
	contact->h1 = count > 0 ? lpMaxFloat( 0.5f * ( hi1 - lo1 ), 1e-3f ) : 1e-3f;
	contact->h2 = count > 0 ? lpMaxFloat( 0.5f * ( hi2 - lo2 ), 1e-3f ) : 1e-3f;
}

void lpShape_FaceContact( const lpShape* shape, int faceIndex, lpContact* contact )
{
	const lpFace* face = shape->faces + faceIndex;
	contact->area = lpShape_FaceArea( shape, faceIndex, &contact->centroid );
	contact->normal = face->plane.normal;
	lpVec3 points[LP_POLY_MAX_VERTICES];
	int count = face->count < LP_POLY_MAX_VERTICES ? face->count : LP_POLY_MAX_VERTICES;
	for ( int k = 0; k < count; ++k )
	{
		points[k] = shape->vertices[shape->indices[face->first + k]];
	}
	lpContactExtents( contact, points, count );
}

bool lpShape_Contact( const lpShape* a, const lpShape* b, float tolerance, lpContact* contact )
{
	enum
	{
		lp_maxContactPoints = 128
	};
	float total = 0.0f;
	lpVec3 weighted = lpVec3_zero;
	float largest = 0.0f;
	lpVec3 normal = lpVec3_zero;
	lpVec3 points[lp_maxContactPoints];
	int pointCount = 0;

	for ( int fa = 0; fa < a->faceCount; ++fa )
	{
		const lpFace* faceA = a->faces + fa;
		lpVec3 n = faceA->plane.normal;
		for ( int fb = 0; fb < b->faceCount; ++fb )
		{
			const lpFace* faceB = b->faces + fb;
			if ( lpDot( n, faceB->plane.normal ) > -0.999f )
			{
				continue;
			}
			if ( lpAbsFloat( faceA->plane.offset + faceB->plane.offset ) > tolerance )
			{
				continue;
			}

			// 2D basis in the plane of A
			lpVec3 t = lpAbsFloat( n.x ) < 0.57f ? (lpVec3){ 1.0f, 0.0f, 0.0f } : (lpVec3){ 0.0f, 1.0f, 0.0f };
			lpVec3 u = lpNormalize( lpCross( t, n ) );
			lpVec3 v = lpCross( n, u );
			lpVec3 origin = lpMulSV( faceA->plane.offset, n );

			lpVec2 bufferA[lp_maxPolygon2];
			lpVec2 bufferB[lp_maxPolygon2];
			int countA = faceA->count < lp_maxPolygon2 ? faceA->count : lp_maxPolygon2;
			for ( int k = 0; k < countA; ++k )
			{
				lpVec3 p = lpSub( a->vertices[a->indices[faceA->first + k]], origin );
				bufferA[k] = (lpVec2){ lpDot( p, u ), lpDot( p, v ) };
			}

			// B's loop, projected into A's basis, runs clockwise; clip A by each of B's edges.
			lpVec2 loopB[lp_maxPolygon2];
			int countB = faceB->count < lp_maxPolygon2 ? faceB->count : lp_maxPolygon2;
			for ( int k = 0; k < countB; ++k )
			{
				lpVec3 p = lpSub( b->vertices[b->indices[faceB->first + k]], origin );
				loopB[k] = (lpVec2){ lpDot( p, u ), lpDot( p, v ) };
			}
			float signB = lpPolygonArea2( loopB, countB, NULL ) >= 0.0f ? 1.0f : -1.0f;

			lpVec2* src = bufferA;
			lpVec2* dst = bufferB;
			int count = countA;
			for ( int k = 0; k < countB && count > 0; ++k )
			{
				count = lpClipPolygon2( src, count, loopB[k], loopB[( k + 1 ) % countB], signB, dst );
				lpVec2* tmp = src;
				src = dst;
				dst = tmp;
			}

			if ( count < 3 )
			{
				continue;
			}

			lpVec2 c2 = { 0.0f, 0.0f };
			float area = lpPolygonArea2( src, count, &c2 );
			if ( area <= 0.0f )
			{
				continue;
			}

			lpVec3 c3 = lpAdd( origin, lpAdd( lpMulSV( c2.x, u ), lpMulSV( c2.y, v ) ) );
			total += area;
			weighted = lpAdd( weighted, lpMulSV( area, c3 ) );
			if ( area > largest )
			{
				largest = area;
				normal = n;
			}
			for ( int k = 0; k < count && pointCount < lp_maxContactPoints; ++k )
			{
				points[pointCount++] = lpAdd( origin, lpAdd( lpMulSV( src[k].x, u ), lpMulSV( src[k].y, v ) ) );
			}
		}
	}

	contact->area = total;
	if ( total <= 0.0f )
	{
		return false;
	}
	contact->centroid = lpMulSV( 1.0f / total, weighted );
	contact->normal = normal;
	lpContactExtents( contact, points, pointCount );
	return true;
}
