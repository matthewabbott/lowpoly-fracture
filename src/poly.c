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

static const b3Vec3 lp_boxNormals[6] = {
	{ 1.0f, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f },
	{ 0.0f, -1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }, { 0.0f, 0.0f, -1.0f },
};

void lpPoly_MakeBox( lpPoly* poly, b3Vec3 h, b3Transform transform, uint8_t material )
{
	for ( int i = 0; i < 8; ++i )
	{
		b3Vec3 p = { ( i & 1 ) ? h.x : -h.x, ( i & 2 ) ? h.y : -h.y, ( i & 4 ) ? h.z : -h.z };
		poly->vertices[i] = b3TransformPoint( transform, p );
	}
	poly->vertexCount = 8;

	for ( int f = 0; f < 6; ++f )
	{
		lpFace* face = poly->faces + f;
		b3Vec3 n = b3RotateVector( transform.q, lp_boxNormals[f] );
		face->plane.normal = n;
		face->plane.offset = b3Dot( n, poly->vertices[lp_boxLoops[f][0]] );
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

bool lpPoly_MakeFromHull( lpPoly* poly, const b3HullData* hull, uint8_t material )
{
	if ( hull == NULL || hull->vertexCount > LP_POLY_MAX_VERTICES || hull->faceCount > LP_POLY_MAX_FACES )
	{
		return false;
	}

	const b3Vec3* points = b3GetHullPoints( hull );
	const b3Plane* planes = b3GetHullPlanes( hull );
	const b3HullFace* faces = b3GetHullFaces( hull );
	const b3HullHalfEdge* edges = b3GetHullEdges( hull );

	for ( int i = 0; i < hull->vertexCount; ++i )
	{
		poly->vertices[i] = points[i];
	}
	poly->vertexCount = hull->vertexCount;

	int indexCount = 0;
	for ( int f = 0; f < hull->faceCount; ++f )
	{
		lpFace* face = poly->faces + f;
		face->first = (uint16_t)indexCount;
		face->material = material;
		face->tag = LP_TAG_EXTERIOR;
		face->plane = planes[f];

		int first = faces[f].edge;
		int edge = first;
		int count = 0;
		do
		{
			if ( indexCount >= LP_POLY_MAX_INDICES || count >= 255 )
			{
				return false;
			}
			poly->indices[indexCount++] = (uint8_t)edges[edge].origin;
			count += 1;
			edge = edges[edge].next;
		}
		while ( edge != first );
		face->count = (uint8_t)count;
	}
	poly->faceCount = hull->faceCount;
	poly->indexCount = indexCount;
	return true;
}

bool lpPoly_MakeFromPoints( lpPoly* poly, const b3Vec3* points, int count, uint8_t material )
{
	b3HullData* hull = b3CreateHull( points, count, LP_POLY_MAX_VERTICES );
	if ( hull == NULL )
	{
		return false;
	}
	bool ok = lpPoly_MakeFromHull( poly, hull, material );
	b3DestroyHull( hull );
	return ok;
}

void lpPoly_Translate( lpPoly* poly, b3Vec3 translation )
{
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		poly->vertices[i] = b3Add( poly->vertices[i], translation );
	}
	for ( int i = 0; i < poly->faceCount; ++i )
	{
		b3Plane* plane = &poly->faces[i].plane;
		plane->offset += b3Dot( plane->normal, translation );
	}
}

// Plane through a convex face loop by Newell's method: robust for any vertex order and count.
static b3Plane lpNewellPlane( const lpPoly* poly, const lpFace* face )
{
	b3Vec3 n = b3Vec3_zero;
	b3Vec3 c = b3Vec3_zero;
	for ( int k = 0; k < face->count; ++k )
	{
		b3Vec3 a = poly->vertices[poly->indices[face->first + k]];
		b3Vec3 b = poly->vertices[poly->indices[face->first + ( k + 1 ) % face->count]];
		n.x += ( a.y - b.y ) * ( a.z + b.z );
		n.y += ( a.z - b.z ) * ( a.x + b.x );
		n.z += ( a.x - b.x ) * ( a.y + b.y );
		c = b3Add( c, a );
	}
	c = b3MulSV( 1.0f / (float)face->count, c );
	n = b3Normalize( n );
	return (b3Plane){ n, b3Dot( n, c ) };
}

void lpPoly_ApplyLinear( lpPoly* poly, b3Matrix3 m )
{
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		poly->vertices[i] = b3MulMV( m, poly->vertices[i] );
	}
	for ( int i = 0; i < poly->faceCount; ++i )
	{
		poly->faces[i].plane = lpNewellPlane( poly, poly->faces + i );
	}
}

lpClipResult lpPoly_Clip( const lpPoly* in, b3Plane plane, uint8_t material, int32_t tag, float tolerance, lpPoly* out )
{
	LP_ASSERT( in != out );

	int vertexCount = in->vertexCount;
	float s[LP_POLY_MAX_VERTICES];
	float minS = FLT_MAX;
	float maxS = -FLT_MAX;
	for ( int i = 0; i < vertexCount; ++i )
	{
		float si = b3Dot( plane.normal, in->vertices[i] ) - plane.offset;
		s[i] = si;
		minS = si < minS ? si : minS;
		maxS = si > maxS ? si : maxS;
	}

	if ( maxS <= tolerance )
	{
		return lp_clipUnchanged;
	}
	if ( minS >= -tolerance )
	{
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
			if ( b3AbsFloat( s[i] - shift ) < tolerance )
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

	int insideCount = 0;
	for ( int i = 0; i < vertexCount; ++i )
	{
		float si = s[i] - shift;
		if ( b3AbsFloat( si ) < tolerance )
		{
			si = -tolerance;
		}
		s[i] = si;
		insideCount += si < 0.0f ? 1 : 0;
	}

	if ( insideCount == vertexCount )
	{
		return lp_clipUnchanged;
	}
	if ( insideCount == 0 )
	{
		return lp_clipEmpty;
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
					b3Vec3 pa = in->vertices[a];
					b3Vec3 pb = in->vertices[b];
					v = outVertexCount++;
					out->vertices[v] = b3Add( pa, b3MulSV( t, b3Sub( pb, pa ) ) );
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

void lpPoly_ComputeMass( const lpPoly* poly, float* volume, b3Vec3* centroid )
{
	// Reference point inside the polyhedron keeps the tetrahedra small, for precision
	b3Vec3 r = b3Vec3_zero;
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		r = b3Add( r, poly->vertices[i] );
	}
	r = b3MulSV( 1.0f / (float)poly->vertexCount, r );

	float v6 = 0.0f;
	b3Vec3 c = b3Vec3_zero;
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		const lpFace* face = poly->faces + f;
		b3Vec3 a = b3Sub( poly->vertices[poly->indices[face->first]], r );
		for ( int k = 1; k + 1 < face->count; ++k )
		{
			b3Vec3 b = b3Sub( poly->vertices[poly->indices[face->first + k]], r );
			b3Vec3 d = b3Sub( poly->vertices[poly->indices[face->first + k + 1]], r );
			float t = b3Dot( a, b3Cross( b, d ) );
			v6 += t;
			c = b3Add( c, b3MulSV( t, b3Add( b3Add( a, b ), d ) ) );
		}
	}

	*volume = v6 / 6.0f;
	if ( v6 > 0.0f )
	{
		// centroid of tetra (r, a, b, d) is r + (a + b + d) / 4
		*centroid = b3Add( r, b3MulSV( 1.0f / ( 4.0f * v6 ), c ) );
	}
	else
	{
		*centroid = r;
	}
}

b3AABB lpPoly_ComputeBounds( const lpPoly* poly )
{
	b3AABB box = { poly->vertices[0], poly->vertices[0] };
	for ( int i = 1; i < poly->vertexCount; ++i )
	{
		box.lowerBound = b3Min( box.lowerBound, poly->vertices[i] );
		box.upperBound = b3Max( box.upperBound, poly->vertices[i] );
	}
	return box;
}

float lpPoly_MaxDistanceSquared( const lpPoly* poly, b3Vec3 point )
{
	float m = 0.0f;
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		float d = b3DistanceSquared( poly->vertices[i], point );
		m = d > m ? d : m;
	}
	return m;
}

float lpPoly_SignedDistance( const lpPoly* poly, b3Vec3 point )
{
	float m = -FLT_MAX;
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		float d = b3Dot( poly->faces[f].plane.normal, point ) - poly->faces[f].plane.offset;
		m = d > m ? d : m;
	}
	return m;
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
		if ( b3AbsFloat( b3Length( face->plane.normal ) - 1.0f ) > 1e-3f )
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
			float d = b3Dot( face->plane.normal, poly->vertices[a] ) - face->plane.offset;
			if ( b3AbsFloat( d ) > tolerance )
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
			float d = b3Dot( poly->faces[f].plane.normal, poly->vertices[i] ) - poly->faces[f].plane.offset;
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
	b3Vec3 centroid;
	lpPoly_ComputeMass( poly, &volume, &centroid );
	return volume > 0.0f;
}

lpShape* lpShape_Create( const lpPoly* poly )
{
	float volume;
	b3Vec3 centroid;
	lpPoly_ComputeMass( poly, &volume, &centroid );
	if ( ( volume > 0.0f ) == false )
	{
		return NULL;
	}

	size_t header = ( sizeof( lpShape ) + 15 ) & ~(size_t)15;
	size_t vertexBytes = sizeof( b3Vec3 ) * (size_t)poly->vertexCount;
	size_t faceBytes = sizeof( lpFace ) * (size_t)poly->faceCount;
	size_t indexBytes = (size_t)poly->indexCount;
	uint8_t* memory = lpAlloc( header + vertexBytes + faceBytes + indexBytes );

	lpShape* shape = (lpShape*)memory;
	shape->faces = (lpFace*)( memory + header );
	shape->vertices = (b3Vec3*)( memory + header + faceBytes );
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
	memcpy( poly->vertices, shape->vertices, sizeof( b3Vec3 ) * (size_t)shape->vertexCount );
	memcpy( poly->faces, shape->faces, sizeof( lpFace ) * (size_t)shape->faceCount );
	memcpy( poly->indices, shape->indices, (size_t)shape->indexCount );
}

void lpShape_Translate( lpShape* shape, b3Vec3 translation )
{
	for ( int i = 0; i < shape->vertexCount; ++i )
	{
		shape->vertices[i] = b3Add( shape->vertices[i], translation );
	}
	for ( int i = 0; i < shape->faceCount; ++i )
	{
		b3Plane* plane = &shape->faces[i].plane;
		plane->offset += b3Dot( plane->normal, translation );
	}
	shape->bounds.lowerBound = b3Add( shape->bounds.lowerBound, translation );
	shape->bounds.upperBound = b3Add( shape->bounds.upperBound, translation );
	shape->centroid = b3Add( shape->centroid, translation );
}

bool lpShape_HasFaceOnPlane( const lpShape* shape, b3Plane plane, float tolerance )
{
	for ( int i = 0; i < shape->faceCount; ++i )
	{
		b3Plane p = shape->faces[i].plane;
		if ( b3Dot( p.normal, plane.normal ) > 0.999f && b3AbsFloat( p.offset - plane.offset ) < tolerance )
		{
			return true;
		}
	}
	return false;
}

float lpShape_SignedDistance( const lpShape* shape, b3Vec3 point )
{
	float m = -FLT_MAX;
	for ( int f = 0; f < shape->faceCount; ++f )
	{
		float d = b3Dot( shape->faces[f].plane.normal, point ) - shape->faces[f].plane.offset;
		m = d > m ? d : m;
	}
	return m;
}

float lpShape_FaceArea( const lpShape* shape, int faceIndex, b3Vec3* centroid )
{
	const lpFace* face = shape->faces + faceIndex;
	b3Vec3 a = shape->vertices[shape->indices[face->first]];
	float area2 = 0.0f;
	b3Vec3 c = b3Vec3_zero;
	for ( int k = 1; k + 1 < face->count; ++k )
	{
		b3Vec3 b = shape->vertices[shape->indices[face->first + k]];
		b3Vec3 d = shape->vertices[shape->indices[face->first + k + 1]];
		float t = b3Dot( face->plane.normal, b3Cross( b3Sub( b, a ), b3Sub( d, a ) ) );
		area2 += t;
		c = b3Add( c, b3MulSV( t, b3Add( b3Add( a, b ), d ) ) );
	}
	if ( centroid != NULL )
	{
		*centroid = area2 > 0.0f ? b3MulSV( 1.0f / ( 3.0f * area2 ), c ) : a;
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
	return false;
}

b3HullData* lpShape_CreateHull( const lpShape* shape )
{
	return b3CreateHull( shape->vertices, shape->vertexCount, B3_MAX_HULL_VERTICES );
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

float lpShape_ContactArea( const lpShape* a, const lpShape* b, float tolerance, b3Vec3* centroid, b3Vec3* normal )
{
	float total = 0.0f;
	b3Vec3 weighted = b3Vec3_zero;
	b3Vec3 bestNormal = b3Vec3_zero;
	float bestArea = 0.0f;

	for ( int fa = 0; fa < a->faceCount; ++fa )
	{
		const lpFace* faceA = a->faces + fa;
		b3Vec3 n = faceA->plane.normal;
		for ( int fb = 0; fb < b->faceCount; ++fb )
		{
			const lpFace* faceB = b->faces + fb;
			if ( b3Dot( n, faceB->plane.normal ) > -0.999f )
			{
				continue;
			}
			if ( b3AbsFloat( faceA->plane.offset + faceB->plane.offset ) > tolerance )
			{
				continue;
			}

			// 2D basis in the plane of A
			b3Vec3 t = b3AbsFloat( n.x ) < 0.57f ? (b3Vec3){ 1.0f, 0.0f, 0.0f } : (b3Vec3){ 0.0f, 1.0f, 0.0f };
			b3Vec3 u = b3Normalize( b3Cross( t, n ) );
			b3Vec3 v = b3Cross( n, u );
			b3Vec3 origin = b3MulSV( faceA->plane.offset, n );

			lpVec2 bufferA[lp_maxPolygon2];
			lpVec2 bufferB[lp_maxPolygon2];
			int countA = faceA->count < lp_maxPolygon2 ? faceA->count : lp_maxPolygon2;
			for ( int k = 0; k < countA; ++k )
			{
				b3Vec3 p = b3Sub( a->vertices[a->indices[faceA->first + k]], origin );
				bufferA[k] = (lpVec2){ b3Dot( p, u ), b3Dot( p, v ) };
			}

			// B's loop, projected into A's basis, runs clockwise; clip A by each of B's edges.
			lpVec2 loopB[lp_maxPolygon2];
			int countB = faceB->count < lp_maxPolygon2 ? faceB->count : lp_maxPolygon2;
			for ( int k = 0; k < countB; ++k )
			{
				b3Vec3 p = b3Sub( b->vertices[b->indices[faceB->first + k]], origin );
				loopB[k] = (lpVec2){ b3Dot( p, u ), b3Dot( p, v ) };
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

			b3Vec3 c3 = b3Add( origin, b3Add( b3MulSV( c2.x, u ), b3MulSV( c2.y, v ) ) );
			total += area;
			weighted = b3Add( weighted, b3MulSV( area, c3 ) );
			if ( area > bestArea )
			{
				bestArea = area;
				bestNormal = n;
			}
		}
	}

	if ( total > 0.0f )
	{
		if ( centroid != NULL )
		{
			*centroid = b3MulSV( 1.0f / total, weighted );
		}
		if ( normal != NULL )
		{
			*normal = bestNormal;
		}
	}
	return total;
}
