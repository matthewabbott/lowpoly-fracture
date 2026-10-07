// SPDX-License-Identifier: MIT
// The exact working polyhedron (src/xpoly.h, milestone 11a's C2): the clip fuzzed through vertices, edges and faces,
// half-space intersections against brute force, boxes and conversions, and the clip's cost. Part of suite `geom`.

#include "test_macros.h"
#include "xpoly.h"

#include <stdlib.h>

#define A_MAX LP_PLANE_NORMAL_MAX
#define P_MAX LP_GRID_RANGE

static uint64_t Gcd64( uint64_t a, uint64_t b )
{
	while ( b != 0 )
	{
		uint64_t r = a % b;
		a = b;
		b = r;
	}
	return a;
}

static uint64_t Abs64( int64_t x )
{
	return x < 0 ? 0u - (uint64_t)x : (uint64_t)x;
}

// The canonical plane c_0 p_0 + ... (planes as (n, d) 4-vectors): it holds every point on all of the p_i. False when
// it is no plane or out of range.
static bool CombinePlanes( const lpIPlane* const* planes, const int* c, int count, lpIPlane* out )
{
	int64_t n[3] = { 0, 0, 0 };
	int64_t d = 0;
	for ( int i = 0; i < count; ++i )
	{
		for ( int k = 0; k < 3; ++k )
		{
			n[k] += (int64_t)c[i] * planes[i]->n[k];
		}
		d += (int64_t)c[i] * planes[i]->d; // |d| < 2^49, |c| <= 3: far inside int64
	}
	uint64_t g = Gcd64( Gcd64( Gcd64( Abs64( n[0] ), Abs64( n[1] ) ), Abs64( n[2] ) ), Abs64( d ) );
	if ( n[0] == 0 && n[1] == 0 && n[2] == 0 )
	{
		return false;
	}
	for ( int k = 0; k < 3; ++k )
	{
		n[k] /= (int64_t)g;
		if ( n[k] > A_MAX || n[k] < -A_MAX )
		{
			return false;
		}
	}
	*out = (lpIPlane){ { (int32_t)n[0], (int32_t)n[1], (int32_t)n[2] }, d / (int64_t)g };
	return lpIPlane_IsValid( out );
}

static lpVec3 RandomDirection( lpRandom* rng )
{
	for ( ;; )
	{
		lpVec3 v;
		v.x = lpRandom_Range( rng, -1.0f, 1.0f );
		v.y = lpRandom_Range( rng, -1.0f, 1.0f );
		v.z = lpRandom_Range( rng, -1.0f, 1.0f );
		float l2 = lpLengthSquared( v );
		if ( l2 > 0.01f && l2 <= 1.0f )
		{
			return lpMulSV( 1.0f / sqrtf( l2 ), v );
		}
	}
}

static int32_t RandomInt( lpRandom* rng, int32_t m )
{
	return (int32_t)( lpRandom_Next( rng ) % (uint32_t)( 2 * m + 1 ) ) - m;
}

// A plane with a normal snapped to k bits through grid point p (k small: small normals, many coincidences)
static bool SnappedPlane( lpRandom* rng, const int32_t p[3], int k, lpIPlane* plane )
{
	return lpIPlane_MakeSnapped( RandomDirection( rng ), k, p, plane, NULL );
}

// A random exact polyhedron: an axis-aligned box cut a few times, or half-spaces with small normals tangent around a
// centre; anywhere in the grid's range, from a millimetre to metres
static bool RandomPoly( lpRandom* rng, lpXPoly* out, lpXPoly* scratch )
{
	lpIPlane planes[24];
	int count = 0;
	int32_t reach = 1 << ( 6 + (int)( lpRandom_Next( rng ) % 14u ) );
	int32_t centre[3];
	for ( int k = 0; k < 3; ++k )
	{
		centre[k] = RandomInt( rng, P_MAX - 2 * reach );
	}
	if ( lpRandom_Next( rng ) % 3u == 0 )
	{
		for ( int k = 0; k < 3; ++k )
		{
			int32_t h = 1 + (int32_t)( lpRandom_Next( rng ) % (uint32_t)reach );
			lpIPlane up = { { 0, 0, 0 }, (int64_t)centre[k] + h };
			lpIPlane down = { { 0, 0, 0 }, -(int64_t)centre[k] + h };
			up.n[k] = 1;
			down.n[k] = -1;
			planes[count++] = up;
			planes[count++] = down;
		}
		int cuts = (int)( lpRandom_Next( rng ) % 5u );
		for ( int i = 0; i < cuts; ++i )
		{
			int32_t p[3];
			for ( int k = 0; k < 3; ++k )
			{
				p[k] = centre[k] + RandomInt( rng, reach / 2 );
			}
			lpIPlane plane;
			if ( SnappedPlane( rng, p, 1 + (int)( lpRandom_Next( rng ) % 6u ), &plane ) &&
				 lpIPlane_ClassifyPoint( &plane, centre ) < 0 )
			{
				planes[count++] = plane;
			}
		}
	}
	else
	{
		int n = 6 + (int)( lpRandom_Next( rng ) % 14u );
		int k = 1 + (int)( lpRandom_Next( rng ) % 8u );
		for ( int i = 0; i < n; ++i )
		{
			lpVec3 dir = RandomDirection( rng );
			float r = (float)reach * lpRandom_Range( rng, 0.5f, 1.0f );
			int32_t p[3] = { centre[0] + (int32_t)( r * dir.x ), centre[1] + (int32_t)( r * dir.y ),
							 centre[2] + (int32_t)( r * dir.z ) };
			lpIPlane plane;
			if ( lpIPlane_MakeSnapped( dir, k, p, &plane, NULL ) && lpIPlane_ClassifyPoint( &plane, centre ) < 0 )
			{
				planes[count++] = plane;
			}
		}
	}
	return lpXPoly_FromPlanes( planes, NULL, NULL, count, out, scratch ) == lp_xBuilt;
}

// The other face holding edge a -> b (as b -> a), or -1
static int FaceAcross( const lpXPoly* poly, int a, int b )
{
	for ( int g = 0; g < poly->faceCount; ++g )
	{
		const lpXFace* face = poly->faces + g;
		for ( int j = 0; j < face->count; ++j )
		{
			int c = poly->indices[face->first + j];
			int e = poly->indices[face->first + ( j + 1 ) % face->count];
			if ( c == b && e == a )
			{
				return g;
			}
		}
	}
	return -1;
}

static int RandomCoefficient( lpRandom* rng )
{
	int c = (int)( lpRandom_Next( rng ) % 7u ) - 3;
	return c == 0 ? 1 : c;
}

// A plane for the fuzz: through the interior at a random small-normal direction (four in ten), through a vertex (a
// combination of its triple's planes), along an edge (of its two faces' planes), in a face (its plane or the negation),
// or through a line in a face (its plane and a random one)
static void FuzzPlane( lpRandom* rng, const lpXPoly* poly, lpIPlane* plane )
{
	int kind = (int)( lpRandom_Next( rng ) % 10u );
	if ( kind >= 4 && kind <= 5 )
	{
		int v = (int)( lpRandom_Next( rng ) % (uint32_t)poly->vertexCount );
		const uint8_t* t = poly->triples[v];
		const lpIPlane* p[3] = { &poly->faces[t[0]].plane, &poly->faces[t[1]].plane, &poly->faces[t[2]].plane };
		int c[3];
		for ( int i = 0; i < 3; ++i )
		{
			c[i] = RandomCoefficient( rng ); // one draw per statement: C leaves the order inside an initializer open
		}
		if ( CombinePlanes( p, c, 3, plane ) )
		{
			return;
		}
	}
	else if ( kind >= 6 && kind <= 7 )
	{
		int f = (int)( lpRandom_Next( rng ) % (uint32_t)poly->faceCount );
		const lpXFace* face = poly->faces + f;
		int k = (int)( lpRandom_Next( rng ) % face->count );
		int g = FaceAcross( poly, poly->indices[face->first + k], poly->indices[face->first + ( k + 1 ) % face->count] );
		const lpIPlane* p[2] = { &face->plane, &poly->faces[g].plane };
		int c[2];
		c[0] = RandomCoefficient( rng );
		c[1] = RandomCoefficient( rng );
		if ( g >= 0 && CombinePlanes( p, c, 2, plane ) )
		{
			return;
		}
	}
	else if ( kind == 8 )
	{
		const lpXFace* face = poly->faces + lpRandom_Next( rng ) % (uint32_t)poly->faceCount;
		*plane = ( lpRandom_Next( rng ) & 1u ) ? face->plane : lpIPlane_Negate( face->plane );
		return;
	}
	// Through a point between two vertices (rounded to the grid), at a small-normal direction
	int a = (int)( lpRandom_Next( rng ) % (uint32_t)poly->vertexCount );
	int b = (int)( lpRandom_Next( rng ) % (uint32_t)poly->vertexCount );
	float s = lpRandom_Float( rng );
	int32_t p[3];
	for ( int k = 0; k < 3; ++k )
	{
		double x = poly->approx[a][k] + (double)s * ( poly->approx[b][k] - poly->approx[a][k] );
		p[k] = (int32_t)floor( x + 0.5 );
	}
	lpIPlane random;
	while ( SnappedPlane( rng, p, 1 + (int)( lpRandom_Next( rng ) % 10u ), &random ) == false )
	{
	}
	if ( kind == 9 )
	{
		const lpXFace* face = poly->faces + lpRandom_Next( rng ) % (uint32_t)poly->faceCount;
		const lpIPlane* q[2] = { &face->plane, &random };
		int c[2];
		c[0] = RandomCoefficient( rng );
		c[1] = RandomCoefficient( rng );
		if ( CombinePlanes( q, c, 2, plane ) )
		{
			return;
		}
	}
	*plane = random;
}

static bool SameValue( const lpIVertex* a, const lpIVertex* b )
{
	return lpI128_Equal( a->x, b->x ) && lpI128_Equal( a->y, b->y ) && lpI128_Equal( a->z, b->z ) && lpI128_Equal( a->w, b->w );
}

// The two caps hold the same vertices, value for value, in opposite orders
static int CheckCaps( const lpXPoly* a, const lpXPoly* b )
{
	const lpXFace* ca = a->faces + a->faceCount - 1;
	const lpXFace* cb = b->faces + b->faceCount - 1;
	ENSURE( ca->count == cb->count );
	int n = ca->count;
	const uint8_t* la = a->indices + ca->first;
	const uint8_t* lb = b->indices + cb->first;
	int j = 0;
	while ( j < n && SameValue( a->vertices + la[0], b->vertices + lb[j] ) == false )
	{
		j += 1;
	}
	ENSURE( j < n );
	for ( int k = 0; k < n; ++k )
	{
		ENSURE( SameValue( a->vertices + la[k], b->vertices + lb[( j - k + n ) % n] ) );
	}
	return 0;
}

static bool CapHolds( const lpXPoly* poly, const lpIVertex* v )
{
	const lpXFace* cap = poly->faces + poly->faceCount - 1;
	for ( int k = 0; k < cap->count; ++k )
	{
		if ( SameValue( poly->vertices + poly->indices[cap->first + k], v ) )
		{
			return true;
		}
	}
	return false;
}

static int EnsureValid( const lpXPoly* poly )
{
	const char* why = lpXPoly_Validate( poly );
	if ( why != NULL )
	{
		printf( "  invalid: %s (%d vertices, %d faces)\n", why, poly->vertexCount, poly->faceCount );
	}
	ENSURE( why == NULL );
	return 0;
}

// Seeded random exact polyhedra clipped by a plane and by its exact negation, a third of the planes or more through
// existing vertices, along edges or in faces: both halves valid, their caps the same vertices, on-plane vertices in
// both caps, the volumes summing to the whole; a plane with nothing outside leaves it unchanged and its negation
// empties it. The halves are clipped again, so the polyhedra grow faces and coincidences.
static int TestXClipFuzz( void )
{
	lpXPoly* work = malloc( 4 * sizeof( lpXPoly ) );
	lpXPoly* poly = work;
	lpXPoly* halfA = work + 1;
	lpXPoly* halfB = work + 2;
	lpXPoly* scratch = work + 3;
	lpRandom rng;
	lpRandom_Seed( &rng, 0xF022ull, 1 );
	int planes = 0, touching = 0, cuts = 0, unchanged = 0, faceIn = 0, onPlane = 0;
	double worstVolume = 0.0;
	lpXClipStats stats = { 0 };
	for ( int round = 0; round < 3000; ++round )
	{
		while ( RandomPoly( &rng, poly, scratch ) == false )
		{
		}
		ENSURE( EnsureValid( poly ) == 0 );
		for ( int step = 0; step < 8; ++step )
		{
			lpIPlane plane;
			FuzzPlane( &rng, poly, &plane );
			lpIPlane negated = lpIPlane_Negate( plane );
			int on = 0, out = 0;
			for ( int i = 0; i < poly->vertexCount; ++i )
			{
				int s = lpIVertex_Classify( poly->vertices + i, &plane );
				on += s == 0 ? 1 : 0;
				out += s > 0 ? 1 : 0;
			}
			planes += 1;
			touching += on > 0 ? 1 : 0;
			uint8_t material = (uint8_t)( 1 + step );
			lpClipResult ra = lpXPoly_ClipCounted( poly, &plane, material, 1000 + step, halfA, &stats );
			lpClipResult rb = lpXPoly_ClipCounted( poly, &negated, material, 2000 + step, halfB, &stats );
			ENSURE( ra != lp_clipOverflow && ra != lp_clipFailed && rb != lp_clipOverflow && rb != lp_clipFailed );
			if ( out == 0 || out + on == poly->vertexCount )
			{
				// Nothing outside one side: that side unchanged, the other empty
				ENSURE( ( ra == lp_clipUnchanged && rb == lp_clipEmpty ) || ( ra == lp_clipEmpty && rb == lp_clipUnchanged ) );
				ENSURE( ( ra == lp_clipUnchanged ) == ( out == 0 ) );
				unchanged += 1;
				// A face in the plane: on the unchanged side, the face's plane is the cut's
				for ( int f = 0; f < poly->faceCount; ++f )
				{
					faceIn += lpIPlane_Equal( &poly->faces[f].plane, ra == lp_clipUnchanged ? &plane : &negated ) ? 1 : 0;
				}
				continue;
			}
			ENSURE( ra == lp_clipCut && rb == lp_clipCut );
			cuts += 1;
			ENSURE( EnsureValid( halfA ) == 0 );
			ENSURE( EnsureValid( halfB ) == 0 );
			const lpXFace* capA = halfA->faces + halfA->faceCount - 1;
			const lpXFace* capB = halfB->faces + halfB->faceCount - 1;
			ENSURE( lpIPlane_Equal( &capA->plane, &plane ) && lpIPlane_Equal( &capB->plane, &negated ) );
			ENSURE( capA->tag == 1000 + step && capB->tag == 2000 + step && capA->material == material );
			ENSURE( CheckCaps( halfA, halfB ) == 0 );
			for ( int i = 0; i < poly->vertexCount; ++i )
			{
				if ( lpIVertex_Classify( poly->vertices + i, &plane ) == 0 )
				{
					ENSURE( CapHolds( halfA, poly->vertices + i ) && CapHolds( halfB, poly->vertices + i ) );
					onPlane += 1;
				}
			}
			double whole, a, b, centroid[3];
			lpXPoly_ComputeMassPrecise( poly, &whole, centroid );
			lpXPoly_ComputeMassPrecise( halfA, &a, centroid );
			lpXPoly_ComputeMassPrecise( halfB, &b, centroid );
			double error = fabs( a + b - whole ) / whole;
			worstVolume = error > worstVolume ? error : worstVolume;
			ENSURE( error < 1e-12 );

			// Go on with a half, the one with more faces
			lpXPoly_Copy( poly, halfA->faceCount >= halfB->faceCount ? halfA : halfB );
		}
	}
	printf( "  %d planes: %d through vertices, edges or faces (%.0f%%); %d cuts (%lld touching, %d on-plane vertices kept, "
			"%lld triples re-picked), %d unchanged or empty (%d with a face in the plane); worst volume error %.2g\n",
			planes, touching, 100.0 * touching / planes, cuts, (long long)stats.touching, onPlane, (long long)stats.repicked,
			unchanged, faceIn, worstVolume );
	ENSURE( touching * 10 >= planes * 3 );
	ENSURE( stats.repicked > 0 && faceIn > 0 && onPlane > 0 );
	free( work );
	return 0;
}

// The brute-force vertices of half-spaces: every triple's meeting point inside (or on) every plane
typedef struct BruteVertex
{
	lpIVertex point;
	int planes[3];
} BruteVertex;

static int BruteForce( const lpIPlane* planes, int count, BruteVertex* out, int capacity )
{
	int n = 0;
	for ( int i = 0; i < count; ++i )
	{
		for ( int j = i + 1; j < count; ++j )
		{
			for ( int k = j + 1; k < count; ++k )
			{
				lpIVertex v;
				if ( lpIVertex_FromPlanes( planes + i, planes + j, planes + k, &v ) == false )
				{
					continue;
				}
				bool inside = true;
				for ( int m = 0; m < count && inside; ++m )
				{
					inside = lpIVertex_Classify( &v, planes + m ) <= 0;
				}
				// One entry per point: an earlier triple through it already named it
				bool seen = false;
				for ( int m = 0; m < n && seen == false && inside; ++m )
				{
					seen = lpIVertex_Classify( &v, planes + out[m].planes[0] ) == 0 &&
						   lpIVertex_Classify( &v, planes + out[m].planes[1] ) == 0 &&
						   lpIVertex_Classify( &v, planes + out[m].planes[2] ) == 0;
				}
				if ( inside && seen == false && n < capacity )
				{
					out[n].point = v;
					out[n].planes[0] = i;
					out[n].planes[1] = j;
					out[n].planes[2] = k;
					n += 1;
				}
			}
		}
	}
	return n;
}

static const lpIPlane lp_testRange[6] = {
	{ { 1, 0, 0 }, P_MAX }, { { -1, 0, 0 }, P_MAX }, { { 0, 1, 0 }, P_MAX },
	{ { 0, -1, 0 }, P_MAX }, { { 0, 0, 1 }, P_MAX }, { { 0, 0, -1 }, P_MAX },
};

// lpXPoly_FromPlanes against brute force on random half-space sets (tangent planes with small normals around a centre,
// with repeats, pencils through one point, and planes through the centre's neighbours): built when bounded, every
// vertex on its triple's planes and inside every input plane, every face on an input plane, every brute-force vertex
// found and no other; unbounded exactly when the brute-force vertices with the range box put three on a range plane;
// sets with no interior empty
static int TestXHalfSpaces( void )
{
	lpXPoly* work = malloc( 2 * sizeof( lpXPoly ) );
	lpXPoly* poly = work;
	lpXPoly* scratch = work + 1;
	BruteVertex* brute = malloc( 2048 * sizeof( BruteVertex ) );
	lpRandom rng;
	lpRandom_Seed( &rng, 0x4A1Full, 2 );
	int built = 0, unbounded = 0, empty = 0, vertices = 0;
	for ( int round = 0; round < 1500; ++round )
	{
		lpIPlane planes[32];
		int count = 0;
		int32_t reach = 1 << ( 4 + (int)( lpRandom_Next( &rng ) % 16u ) );
		int32_t centre[3];
		for ( int k = 0; k < 3; ++k )
		{
			centre[k] = RandomInt( &rng, P_MAX - 2 * reach );
		}
		int kind = round % 5; // 0-2 tangent sets, 3 a half-open set (unbounded), 4 a set with no interior
		int n = 4 + (int)( lpRandom_Next( &rng ) % 12u );
		int k = 1 + (int)( lpRandom_Next( &rng ) % 6u );
		for ( int i = 0; i < n; ++i )
		{
			lpVec3 dir = RandomDirection( &rng );
			if ( kind == 3 && dir.z < 0.2f )
			{
				dir.z = 0.2f + lpRandom_Float( &rng ); // every normal up: nothing bounds it downward
			}
			float r = (float)reach * lpRandom_Range( &rng, 0.3f, 1.0f );
			int32_t p[3] = { centre[0] + (int32_t)( r * dir.x ), centre[1] + (int32_t)( r * dir.y ),
							 centre[2] + (int32_t)( r * dir.z ) };
			lpIPlane plane;
			if ( lpIPlane_MakeSnapped( dir, k, p, &plane, NULL ) == false || lpIPlane_ClassifyPoint( &plane, centre ) >= 0 )
			{
				continue;
			}
			planes[count++] = plane;
			uint32_t extra = lpRandom_Next( &rng ) % 8u;
			if ( extra == 0 )
			{
				planes[count++] = plane; // a repeat
			}
			else if ( extra == 1 && kind != 3 )
			{
				// A pencil: two more planes through p, still holding the centre inside
				for ( int j = 0; j < 2; ++j )
				{
					lpIPlane other;
					if ( SnappedPlane( &rng, p, k, &other ) && lpIPlane_ClassifyPoint( &other, centre ) < 0 )
					{
						planes[count++] = other;
					}
				}
			}
		}
		if ( kind == 4 && count > 0 )
		{
			planes[count++] = lpIPlane_Negate( planes[0] ); // with the first: a plane, no interior
		}
		lpXBuild result = lpXPoly_FromPlanes( planes, NULL, NULL, count, poly, scratch );

		// Brute force with the range box: unbounded (or past the range) when a range plane holds three of its vertices
		lpIPlane all[40];
		memcpy( all, planes, sizeof( lpIPlane ) * (size_t)count );
		memcpy( all + count, lp_testRange, sizeof( lp_testRange ) );
		int bruteCount = BruteForce( all, count + 6, brute, 2048 );
		bool onRange = false;
		for ( int r = 0; r < 6; ++r )
		{
			int held = 0;
			for ( int b = 0; b < bruteCount; ++b )
			{
				held += lpIVertex_Classify( &brute[b].point, lp_testRange + r ) == 0 ? 1 : 0;
			}
			onRange = onRange || held >= 3;
		}
		if ( kind == 4 && count > 0 )
		{
			ENSURE( result == lp_xEmpty );
			empty += 1;
			continue;
		}
		if ( count < 4 || onRange )
		{
			ENSURE( result == lp_xUnbounded );
			unbounded += 1;
			continue;
		}
		ENSURE( kind != 3 );
		ENSURE( result == lp_xBuilt );
		built += 1;
		ENSURE( EnsureValid( poly ) == 0 );
		ENSURE( lpIPlane_ClassifyPoint( &poly->faces[0].plane, centre ) < 0 );
		for ( int f = 0; f < poly->faceCount; ++f )
		{
			bool input = false;
			for ( int i = 0; i < count && input == false; ++i )
			{
				input = lpIPlane_Equal( &poly->faces[f].plane, planes + i );
			}
			ENSURE( input );
		}
		for ( int v = 0; v < poly->vertexCount; ++v )
		{
			for ( int i = 0; i < count; ++i )
			{
				ENSURE( lpIVertex_Classify( poly->vertices + v, planes + i ) <= 0 );
			}
			bool found = false;
			for ( int b = 0; b < bruteCount && found == false; ++b )
			{
				found = lpIVertex_Classify( &brute[b].point, &poly->faces[poly->triples[v][0]].plane ) == 0 &&
						lpIVertex_Classify( &brute[b].point, &poly->faces[poly->triples[v][1]].plane ) == 0 &&
						lpIVertex_Classify( &brute[b].point, &poly->faces[poly->triples[v][2]].plane ) == 0;
			}
			ENSURE( found );
		}
		for ( int b = 0; b < bruteCount; ++b )
		{
			bool found = false;
			for ( int v = 0; v < poly->vertexCount && found == false; ++v )
			{
				found = lpIVertex_Classify( poly->vertices + v, all + brute[b].planes[0] ) == 0 &&
						lpIVertex_Classify( poly->vertices + v, all + brute[b].planes[1] ) == 0 &&
						lpIVertex_Classify( poly->vertices + v, all + brute[b].planes[2] ) == 0;
			}
			ENSURE( found );
		}
		ENSURE( bruteCount == poly->vertexCount );
		vertices += poly->vertexCount;
	}
	printf( "  %d sets built (%d vertices against brute force), %d unbounded, %d empty\n", built, vertices, unbounded, empty );
	ENSURE( built > 500 && unbounded > 200 && empty > 200 );
	free( brute );
	free( work );
	return 0;
}

// Boxes from snapped planes, and float polyhedra converted: valid, with the float's volume to the snap's precision;
// the canonical floats of a box with grid-point corners are the corners
static int TestXBuild( void )
{
	lpXPoly* work = malloc( 2 * sizeof( lpXPoly ) );
	lpPoly* box = malloc( sizeof( lpPoly ) );
	lpVec3* rounded = malloc( LP_XPOLY_MAX_VERTICES * sizeof( lpVec3 ) );
	lpRandom rng;
	lpRandom_Seed( &rng, 0xB0Bull, 4 );
	for ( int i = 0; i < 500; ++i )
	{
		lpVec3 h;
		h.x = lpRandom_Range( &rng, 0.01f, 3.0f ); // one draw per statement: C leaves the order inside an initializer open
		h.y = lpRandom_Range( &rng, 0.01f, 3.0f );
		h.z = lpRandom_Range( &rng, 0.01f, 3.0f );
		lpVec3 axis = RandomDirection( &rng );
		float angle = lpRandom_Range( &rng, -3.0f, 3.0f );
		lpCosSin cs = lpComputeCosSin( 0.5f * angle );
		lpTransform t = { lpVec3_zero, { lpMulSV( cs.sine, axis ), cs.cosine } };
		t.p.x = lpRandom_Range( &rng, -60.0f, 60.0f );
		t.p.y = lpRandom_Range( &rng, -60.0f, 60.0f );
		t.p.z = lpRandom_Range( &rng, -60.0f, 60.0f );
		if ( i % 4 == 0 )
		{
			t.q = lpQuat_identity;
		}
		ENSURE( lpXPoly_MakeBox( h, t, 7, work, work + 1 ) == lp_xBuilt );
		ENSURE( EnsureValid( work ) == 0 );
		ENSURE( work->vertexCount == 8 && work->faceCount == 6 && work->faces[0].material == 7 );
		double volume, centroid[3];
		lpXPoly_ComputeMassPrecise( work, &volume, centroid );
		double expected = 8.0 * (double)h.x * (double)h.y * (double)h.z;
		// Each face within its grid point's rounding (0.87 u), the float vertices' (0.25 u at 60 m) and the snapped normal's
		// (1e-7 rad) of the float's: 1.5 u over the area
		double area = 8.0 * ( (double)h.x * h.y + (double)h.y * h.z + (double)h.z * h.x );
		double slack = area * 1.5 / 65536.0;
		ENSURE_NEAR( volume, expected, slack + 1e-6 * expected );
		ENSURE_NEAR( centroid[0], t.p.x, 1e-4 );

		// The float box converted: the same, through its faces' means
		lpPoly_MakeBox( box, h, t, 3 );
		lpVec3 origin = { 1.5f, -2.0f, 0.25f };
		ENSURE( lpXPoly_FromPoly( box, origin, work, work + 1 ) == lp_xBuilt );
		ENSURE( EnsureValid( work ) == 0 && work->faceCount == 6 && work->faces[2].material == 3 );
		lpXPoly_ComputeMassPrecise( work, &volume, centroid );
		ENSURE_NEAR( volume, expected, slack + 1e-5 * expected );
		ENSURE_NEAR( centroid[1], t.p.y + origin.y, 1e-4 );
		if ( i % 4 == 0 )
		{
			// Axis-aligned at grid points: the corners are grid points, and their floats exact
			lpXPoly_Round( work, rounded );
			lpPoly_Translate( box, origin );
			for ( int v = 0; v < 8; ++v )
			{
				float best = 1e30f;
				for ( int u = 0; u < 8; ++u )
				{
					float d = lpDistanceSquared( rounded[v], box->vertices[u] );
					best = d < best ? d : best;
				}
				ENSURE( best < 1e-9f );
				ENSURE( rounded[v].x * 65536.0f == floorf( rounded[v].x * 65536.0f ) );
			}
		}
	}
	free( rounded );
	free( box );
	free( work );
	return 0;
}

// The exact clip's cost on Voronoi work (a 1 m box, 64 sites, each cell clipped by every other site's bisector) beside
// the float clip on the same work, on this build's path (the portable build prints the portable path's)
static int TestXClipCost( void )
{
	enum
	{
		lp_sites = 64
	};
	lpXPoly* work = malloc( 3 * sizeof( lpXPoly ) );
	lpXPoly* parent = work;
	lpPoly* floats = malloc( 3 * sizeof( lpPoly ) );
	lpRandom rng;
	lpRandom_Seed( &rng, 0xC0575ull, 6 );
	lpTransform t = { { 3.0f, 1.0f, -2.0f }, { { 0.0f, 0.38268343f, 0.0f }, 0.92387953f } };
	lpVec3 h = { 0.5f, 0.5f, 0.5f };
	ENSURE( lpXPoly_MakeBox( h, t, 1, parent, work + 1 ) == lp_xBuilt );
	lpPoly_MakeBox( floats, h, t, 1 );
	int32_t sites[lp_sites][3];
	lpVec3 floatSites[lp_sites];
	for ( int i = 0; i < lp_sites; ++i )
	{
		lpVec3 local;
		local.x = lpRandom_Range( &rng, -0.45f, 0.45f ); // one draw per statement
		local.y = lpRandom_Range( &rng, -0.45f, 0.45f );
		local.z = lpRandom_Range( &rng, -0.45f, 0.45f );
		floatSites[i] = lpTransformPoint( t, local );
		ENSURE( lpGeom_GridPoint( floatSites[i].x, floatSites[i].y, floatSites[i].z, sites[i] ) );
	}
	lpXClipStats stats = { 0 };
	int64_t faces = 0;
	uint64_t ticks = lpGetTicks();
	for ( int i = 0; i < lp_sites; ++i )
	{
		lpXPoly_Copy( work + 1, parent );
		lpXPoly* current = work + 1;
		lpXPoly* next = work + 2;
		for ( int j = 0; j < lp_sites; ++j )
		{
			lpIPlane plane;
			if ( j == i || lpIPlane_MakeBisector( sites[i], sites[j], &plane, NULL ) == false )
			{
				continue;
			}
			if ( lpXPoly_ClipCounted( current, &plane, 1, j, next, &stats ) == lp_clipCut )
			{
				lpXPoly* swap = current;
				current = next;
				next = swap;
			}
		}
		faces += current->faceCount;
	}
	float exactMs = lpGetMilliseconds( ticks );

	int64_t floatClips = 0, floatFaces = 0;
	ticks = lpGetTicks();
	for ( int i = 0; i < lp_sites; ++i )
	{
		floats[1] = floats[0];
		lpPoly* current = floats + 1;
		lpPoly* next = floats + 2;
		for ( int j = 0; j < lp_sites; ++j )
		{
			if ( j == i )
			{
				continue;
			}
			lpVec3 normal = lpNormalize( lpSub( floatSites[j], floatSites[i] ) );
			lpPlane plane = { normal, lpDot( normal, lpMulSV( 0.5f, lpAdd( floatSites[i], floatSites[j] ) ) ) };
			floatClips += 1;
			if ( lpPoly_Clip( current, plane, 1, j, 2e-5f, next ) == lp_clipCut )
			{
				lpPoly* swap = current;
				current = next;
				next = swap;
			}
		}
		floatFaces += current->faceCount;
	}
	float floatMs = lpGetMilliseconds( ticks );

	// The cells' vertices rounded to floats
	lpVec3* rounded = malloc( LP_XPOLY_MAX_VERTICES * sizeof( lpVec3 ) );
	int64_t roundedCount = 0;
	ticks = lpGetTicks();
	for ( int r = 0; r < 64; ++r )
	{
		lpXPoly_Round( work + 1, rounded );
		roundedCount += 3 * work[1].vertexCount;
	}
	float roundMs = lpGetMilliseconds( ticks );
	free( rounded );

	printf( "  exact clip (%s): %.0f ns a clip (%lld clips, %lld cut, %.1f vertices classified and %.2f made a clip), %.1f "
			"faces a cell; float clip: %.0f ns a clip, %.1f faces a cell; ratio %.2f; rounding %.1f ns a coordinate\n",
			lpI128_Path(), 1e6 * exactMs / (double)stats.clips, (long long)stats.clips, (long long)stats.cut,
			(double)stats.classifications / (double)stats.clips, (double)stats.vertices / (double)stats.clips,
			(double)faces / lp_sites, 1e6 * floatMs / (double)floatClips, (double)floatFaces / lp_sites,
			(double)exactMs / (double)floatMs, 1e6 * roundMs / (double)roundedCount );
	free( floats );
	free( work );
	return 0;
}

int XPolyTest( void )
{
	RUN_TEST( TestXClipFuzz, MECHANISM );
	RUN_TEST( TestXHalfSpaces, MECHANISM );
	RUN_TEST( TestXBuild, MECHANISM );
	RUN_TEST( TestXClipCost, TIMING );
	return 0;
}
