// SPDX-License-Identifier: MIT

#include "poly.h"
#include "test_macros.h"
#include "test_sim.h"

#include <float.h>
#include <stdlib.h>

static int TestBox( void )
{
	lpPoly poly;
	lpPoly_MakeBox( &poly, (lpVec3){ 1.0f, 0.5f, 0.25f }, lpTransform_identity, 0 );
	ENSURE( lpPoly_IsValid( &poly, 1e-4f ) );
	float volume;
	lpVec3 centroid;
	lpPoly_ComputeMass( &poly, &volume, &centroid );
	ENSURE_NEAR( volume, 1.0f, 1e-5f );
	ENSURE_NEAR( lpLength( centroid ), 0.0f, 1e-5f );
	return 0;
}

static int TestClipBox( void )
{
	lpPoly box, cut;
	lpPoly_MakeBox( &box, (lpVec3){ 0.5f, 0.5f, 0.5f }, lpTransform_identity, 0 );
	lpPlane plane = { { 1.0f, 0.0f, 0.0f }, 0.2f };
	ENSURE( lpPoly_Clip( &box, plane, 1, 7, 1e-5f, &cut ) == lp_clipCut );
	ENSURE( lpPoly_IsValid( &cut, 1e-4f ) );
	float volume;
	lpVec3 centroid;
	lpPoly_ComputeMass( &cut, &volume, &centroid );
	ENSURE_NEAR( volume, 0.7f, 1e-5f );
	ENSURE( cut.faceCount == 6 );
	ENSURE( cut.faces[cut.faceCount - 1].tag == 7 );

	// Plane through a face: unchanged; plane outside: empty
	ENSURE( lpPoly_Clip( &box, (lpPlane){ { 1.0f, 0.0f, 0.0f }, 0.5f }, 1, 7, 1e-5f, &cut ) == lp_clipUnchanged );
	ENSURE( lpPoly_Clip( &box, (lpPlane){ { 1.0f, 0.0f, 0.0f }, -0.5f }, 1, 7, 1e-5f, &cut ) == lp_clipEmpty );

	// Plane through an edge and the opposite edge: diagonal split into two prisms
	lpVec3 n = lpNormalize( (lpVec3){ 1.0f, 1.0f, 0.0f } );
	ENSURE( lpPoly_Clip( &box, (lpPlane){ n, 0.0f }, 1, 7, 1e-5f, &cut ) == lp_clipCut );
	ENSURE( lpPoly_IsValid( &cut, 1e-3f ) );
	lpPoly_ComputeMass( &cut, &volume, &centroid );
	ENSURE_NEAR( volume, 0.5f, 1e-4f );

	// Plane through a corner
	lpVec3 m = lpNormalize( (lpVec3){ 1.0f, 1.0f, 1.0f } );
	float d = lpDot( m, (lpVec3){ 0.5f, 0.5f, -0.5f } );
	ENSURE( lpPoly_Clip( &box, (lpPlane){ m, d }, 1, 7, 1e-5f, &cut ) == lp_clipCut );
	ENSURE( lpPoly_IsValid( &cut, 1e-3f ) );
	return 0;
}

// Random planes against random convex hulls. The two halves must be valid and add up to the whole.
static int TestClipFuzz( void )
{
	lpRandom rng;
	lpRandom_Seed( &rng, 1234, 1 );
	lpPoly poly, keep, other;
	int cuts = 0;

	for ( int trial = 0; trial < 400; ++trial )
	{
		lpVec3 points[24];
		int count = 6 + (int)( lpRandom_Next( &rng ) % 18u );
		for ( int i = 0; i < count; ++i )
		{
			// The draws as statements (rule 13: an initializer's order is unspecified)
			points[i].x = lpRandom_Range( &rng, -1.0f, 1.0f );
			points[i].y = lpRandom_Range( &rng, -0.5f, 0.5f );
			points[i].z = lpRandom_Range( &rng, -0.8f, 0.8f );
		}
		if ( lpPoly_MakeFromPoints( &poly, points, count, 0 ) == false )
		{
			continue;
		}
		ENSURE( lpPoly_IsValid( &poly, 1e-3f ) );
		float whole;
		lpVec3 c;
		lpPoly_ComputeMass( &poly, &whole, &c );

		for ( int k = 0; k < 10; ++k )
		{
			lpVec3 n;
			n.x = lpRandom_Range( &rng, -1.0f, 1.0f );
			n.y = lpRandom_Range( &rng, -1.0f, 1.0f );
			n.z = lpRandom_Range( &rng, -1.0f, 1.0f );
			n = lpNormalize( n );
			// Half the planes go exactly through a vertex, to hit the degenerate paths
			float offset = ( k % 2 == 0 ) ? lpDot( n, poly.vertices[k % poly.vertexCount] ) : lpRandom_Range( &rng, -0.5f, 0.5f );
			lpPlane plane = { n, offset };
			lpPlane flipped = { lpNeg( n ), -offset };

			lpClipResult a = lpPoly_Clip( &poly, plane, 1, 1, 1e-5f, &keep );
			lpClipResult b = lpPoly_Clip( &poly, flipped, 1, 2, 1e-5f, &other );
			ENSURE( a != lp_clipFailed && a != lp_clipOverflow );
			ENSURE( b != lp_clipFailed && b != lp_clipOverflow );

			float va = a == lp_clipCut ? 0.0f : ( a == lp_clipUnchanged ? whole : 0.0f );
			float vb = b == lp_clipCut ? 0.0f : ( b == lp_clipUnchanged ? whole : 0.0f );
			if ( a == lp_clipCut )
			{
				ENSURE( lpPoly_IsValid( &keep, 1e-3f ) );
				lpPoly_ComputeMass( &keep, &va, &c );
				cuts += 1;
			}
			if ( b == lp_clipCut )
			{
				ENSURE( lpPoly_IsValid( &other, 1e-3f ) );
				lpPoly_ComputeMass( &other, &vb, &c );
			}
			if ( a == lp_clipCut && b == lp_clipCut )
			{
				// The degeneracy push moves each cut by at most ~16 tolerances
				ENSURE_NEAR( va + vb, whole, 2e-3f * whole + 1e-4f );
			}
		}
	}
	ENSURE( cuts > 1000 );
	return 0;
}

static int TestContactArea( void )
{
	lpPoly a, b;
	lpPoly_MakeBox( &a, (lpVec3){ 0.5f, 0.5f, 0.5f }, lpTransform_identity, 0 );
	lpPoly_MakeBox( &b, (lpVec3){ 0.5f, 0.25f, 0.5f }, (lpTransform){ { 1.0f, 0.1f, 0.0f }, lpQuat_identity }, 0 );
	lpShape* sa = lpShape_Create( &a );
	lpShape* sb = lpShape_Create( &b );
	lpContact contact;
	ENSURE( lpShape_Contact( sa, sb, 1e-3f, &contact ) );
	ENSURE_NEAR( contact.area, 0.5f, 1e-4f );
	ENSURE_NEAR( contact.centroid.x, 0.5f, 1e-5f );
	ENSURE_NEAR( contact.centroid.y, 0.1f, 1e-5f );
	ENSURE_NEAR( contact.normal.x, 1.0f, 1e-5f );
	// the patch is 0.5 tall (y) and 1 deep (z): half-extents 0.25 and 0.5 in some order
	ENSURE_NEAR( contact.h1 + contact.h2, 0.75f, 1e-4f );
	ENSURE_NEAR( contact.h1 * contact.h2, 0.125f, 1e-4f );

	// Not touching
	lpShape_Translate( sb, (lpVec3){ 0.1f, 0.0f, 0.0f } );
	ENSURE( lpShape_Contact( sa, sb, 1e-3f, &contact ) == false );
	lpShape_Destroy( sa );
	lpShape_Destroy( sb );
	return 0;
}

// A shape's digest is its geometry's: two shapes of one polyhedron agree, a translated one does not, and a different
// polyhedron does not
static int TestShapeDigest( void )
{
	lpPoly a, b;
	lpPoly_MakeBox( &a, (lpVec3){ 0.5f, 0.25f, 0.75f }, lpTransform_identity, 0 );
	lpPoly_MakeBox( &b, (lpVec3){ 0.5f, 0.25f, 0.7f }, lpTransform_identity, 0 );
	lpShape* s1 = lpShape_Create( &a );
	lpShape* s2 = lpShape_Create( &a );
	lpShape* s3 = lpShape_Create( &b );
	ENSURE( s1->digest == s2->digest && s1->digest != s3->digest );
	lpShape_Translate( s2, (lpVec3){ 0.0f, 1.0f, 0.0f } );
	ENSURE( s1->digest != s2->digest );
	lpShape_Destroy( s1 );
	lpShape_Destroy( s2 );
	lpShape_Destroy( s3 );
	return 0;
}

// ---- physics hulls from known topology (milestone 11a, C4) ----

typedef struct HullTally
{
	int shapes;
	int built;		  // by both builders
	int refusedLarge; // quickhull built it, the faces were refused: over the edge limit
	int refusedOther; // quickhull built it, the faces were refused within the limits
	int onlyFaces;	  // the faces built it, quickhull did not
	int neither;
	int quickMerged; // built by both, quickhull with fewer faces than the shape has (near-coplanar ones merged)
	int quickSplit;	 // built by both, quickhull with more (a face a little off its plane split)
	int maxEdges;
	double worstVolume;	  // relative difference from quickhull's
	double worstAllowed;  // the volume's difference over what is allowed
	double worstCentroid; // distance from quickhull's, relative to the shape's radius plus its distance from the origin
} HullTally;

// The hull from a shape's faces passes Box3D's checks, keeps the shape's vertices, planes and loops in their order, and
// has the volume and centroid quickhull finds for the same points
static int CheckHullFromFaces( const lpShape* shape, HullTally* tally )
{
	lpPhysHull* quick = lpShape_CreateHull( shape );
	lpPhysHull* hull = lpShape_CreateHullFromFaces( shape );
	int edges = shape->vertexCount + shape->faceCount - 2;
	tally->shapes += 1;
	tally->maxEdges = edges > tally->maxEdges ? edges : tally->maxEdges;
	if ( hull == NULL )
	{
		bool large = edges > LP_PHYS_MAX_HULL_EDGES;
		tally->refusedLarge += quick != NULL && large ? 1 : 0;
		tally->refusedOther += quick != NULL && large == false ? 1 : 0;
		tally->neither += quick == NULL ? 1 : 0;
	}
	else
	{
		ENSURE( lpPhys_IsValidHull( hull ) );
		ENSURE( lpPhys_GetHullVertexCount( hull ) == shape->vertexCount );
		ENSURE( lpPhys_GetHullFaceCount( hull ) == shape->faceCount );
		for ( int i = 0; i < shape->vertexCount; ++i )
		{
			lpVec3 p = lpPhys_GetHullPoint( hull, i );
			ENSURE( memcmp( &p, shape->vertices + i, sizeof( p ) ) == 0 );
		}
		for ( int i = 0; i < shape->faceCount; ++i )
		{
			const lpFace* face = shape->faces + i;
			lpPlane plane = lpPhys_GetHullPlane( hull, i );
			ENSURE( memcmp( &plane, &face->plane, sizeof( plane ) ) == 0 );
			uint8_t loop[LP_POLY_MAX_VERTICES];
			ENSURE( lpPhys_GetHullFace( hull, i, loop, LP_POLY_MAX_VERTICES ) == face->count );
			ENSURE( memcmp( loop, shape->indices + face->first, face->count ) == 0 );
		}
		if ( quick == NULL )
		{
			tally->onlyFaces += 1;
		}
		else
		{
			// The volumes agree to 1e-6 up to what rounding leaves open: float vertices lie off their face's plane by
			// about an ulp of the farthest coordinate L, so two triangulations of the same faces (quickhull's from its
			// own first vertex and edges, ours from each loop's first point) differ by up to FLT_EPSILON * area * L
			lpVec3 c, qc;
			double volume = lpPhys_GetHullVolume( hull, &c );
			double quickVolume = lpPhys_GetHullVolume( quick, &qc );
			lpVec3 far = lpMax( lpAbs( shape->bounds.lowerBound ), lpAbs( shape->bounds.upperBound ) );
			double area = 0.0;
			for ( int f = 0; f < shape->faceCount; ++f )
			{
				area += lpShape_FaceArea( shape, f, NULL );
			}
			double allowed = 1e-6 * quickVolume + FLT_EPSILON * area * lpMaxFloat( far.x, lpMaxFloat( far.y, far.z ) );
			double dv = fabs( volume - quickVolume );
			double dc = (double)lpLength( lpSub( c, qc ) ) / (double)( shape->radius + lpLength( qc ) );
			tally->worstVolume = dv / quickVolume > tally->worstVolume ? dv / quickVolume : tally->worstVolume;
			tally->worstAllowed = dv / allowed > tally->worstAllowed ? dv / allowed : tally->worstAllowed;
			tally->worstCentroid = dc > tally->worstCentroid ? dc : tally->worstCentroid;
			ENSURE_NEAR( volume, quickVolume, allowed );
			ENSURE_NEAR( dc, 0.0, 1e-6 );
			tally->built += 1;
			tally->quickMerged += lpPhys_GetHullFaceCount( quick ) < shape->faceCount ? 1 : 0;
			tally->quickSplit += lpPhys_GetHullFaceCount( quick ) > shape->faceCount ? 1 : 0;
		}
	}
	if ( hull != NULL )
	{
		lpPhys_DestroyHull( hull );
	}
	if ( quick != NULL )
	{
		lpPhys_DestroyHull( quick );
	}
	return 0;
}

static void PrintTally( const char* what, const HullTally* t )
{
	printf( "  %-26s %5d shapes, %5d built by both (quickhull merged faces in %d, split in %d); refused %d over the "
			"edge limit, %d within it; %d by faces only, %d by neither; most edges %d; worst volume %.2g (%.2g of "
			"allowed), centroid %.2g\n",
			what, t->shapes, t->built, t->quickMerged, t->quickSplit, t->refusedLarge, t->refusedOther, t->onlyFaces,
			t->neither, t->maxEdges, t->worstVolume, t->worstAllowed, t->worstCentroid );
}

// A prism of n sides round the y axis: 2n vertices, n + 2 faces, 3n edges
static void MakePrism( lpPoly* poly, int n, float radius, float height )
{
	for ( int i = 0; i < n; ++i )
	{
		lpCosSin cs = lpComputeCosSin( 2.0f * LP_PI * (float)i / (float)n );
		poly->vertices[i] = (lpVec3){ radius * cs.cosine, -0.5f * height, radius * cs.sine };
		poly->vertices[n + i] = (lpVec3){ radius * cs.cosine, 0.5f * height, radius * cs.sine };
	}
	poly->vertexCount = 2 * n;
	int count = 0;
	for ( int f = 0; f < n + 2; ++f )
	{
		lpFace* face = poly->faces + f;
		face->first = (uint16_t)count;
		face->material = 0;
		face->tag = LP_TAG_EXTERIOR;
		if ( f < n ) // side f, from angle f to f + 1, outward
		{
			int g = ( f + 1 ) % n;
			poly->indices[count + 0] = (uint8_t)f;
			poly->indices[count + 1] = (uint8_t)( n + f );
			poly->indices[count + 2] = (uint8_t)( n + g );
			poly->indices[count + 3] = (uint8_t)g;
			face->count = 4;
			lpVec3 a = poly->vertices[f], b = poly->vertices[g];
			lpVec3 normal = lpNormalize( (lpVec3){ a.x + b.x, 0.0f, a.z + b.z } );
			face->plane = (lpPlane){ normal, lpDot( normal, a ) };
		}
		else // the bottom runs with the angle seen from below, the top against it seen from above
		{
			bool top = f == n + 1;
			for ( int k = 0; k < n; ++k )
			{
				poly->indices[count + k] = (uint8_t)( top ? n + ( n - 1 - k ) : k );
			}
			face->count = (uint8_t)n;
			face->plane = (lpPlane){ { 0.0f, top ? 1.0f : -1.0f, 0.0f }, 0.5f * height };
		}
		count += face->count;
	}
	poly->faceCount = n + 2;
	poly->indexCount = count;
}

// lpPhys_CreateHullFromFaces's input, the loops back to back
typedef struct FaceArrays
{
	lpVec3 points[LP_POLY_MAX_VERTICES];
	lpPlane planes[LP_POLY_MAX_FACES];
	uint8_t sizes[LP_POLY_MAX_FACES];
	uint8_t indices[LP_POLY_MAX_INDICES];
	int pointCount, faceCount;
} FaceArrays;

static void GetFaceArrays( const lpPoly* poly, FaceArrays* a )
{
	memcpy( a->points, poly->vertices, sizeof( lpVec3 ) * (size_t)poly->vertexCount );
	a->pointCount = poly->vertexCount;
	a->faceCount = poly->faceCount;
	int count = 0;
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		a->planes[f] = poly->faces[f].plane;
		a->sizes[f] = poly->faces[f].count;
		memcpy( a->indices + count, poly->indices + poly->faces[f].first, poly->faces[f].count );
		count += poly->faces[f].count;
	}
}

// True if the backend builds a hull from the arrays (which it then destroys)
static bool BuildsHull( const FaceArrays* a )
{
	lpPhysHull* hull =
		lpPhys_CreateHullFromFaces( a->points, a->pointCount, a->planes, a->sizes, a->faceCount, a->indices );
	bool valid = hull != NULL && lpPhys_IsValidHull( hull );
	if ( hull != NULL )
	{
		lpPhys_DestroyHull( hull );
	}
	return valid;
}

static void ReverseLoop( uint8_t* loop, int count )
{
	for ( int i = 0; i < count / 2; ++i )
	{
		uint8_t t = loop[i];
		loop[i] = loop[count - 1 - i];
		loop[count - 1 - i] = t;
	}
}

// The physics hull built from a shape's faces (the Box3D patch b3CreateHullFromFaces) instead of quickhull: on boxes,
// prisms, random convex hulls, boxes cut by random planes, and the authored parts and fracture cells of real scenes it
// passes Box3D's own checks, keeps the shape's vertices, planes and loops, and has quickhull's volume and centroid to
// 1e-6; bad input (open, a flipped face, inside out, a plane facing in, an index out of range, a point on no face, two
// cones meeting at points, over the edge limit) is refused
static int TestHullFromFaces( void )
{
	HullTally tally = { 0 };
	lpRandom rng;
	lpRandom_Seed( &rng, 0xC4ull, 1 );
	lpPoly* polys = malloc( 2 * sizeof( lpPoly ) );

	// Boxes, turned and moved
	for ( int trial = 0; trial < 50; ++trial )
	{
		lpVec3 h, axis;
		lpTransform t;
		h.x = lpRandom_Range( &rng, 0.01f, 2.0f ); // one draw per statement (determinism rule 13)
		h.y = lpRandom_Range( &rng, 0.01f, 2.0f );
		h.z = lpRandom_Range( &rng, 0.01f, 2.0f );
		t.p.x = lpRandom_Range( &rng, -30.0f, 30.0f );
		t.p.y = lpRandom_Range( &rng, 0.0f, 20.0f );
		t.p.z = lpRandom_Range( &rng, -30.0f, 30.0f );
		axis.x = lpRandom_Range( &rng, -1.0f, 1.0f );
		axis.y = lpRandom_Range( &rng, -1.0f, 1.0f );
		axis.z = 0.5f;
		t.q = lpMakeQuatFromAxisAngle( lpNormalize( axis ), lpRandom_Range( &rng, -LP_PI, LP_PI ) );
		lpPoly_MakeBox( polys, h, t, 0 );
		lpShape* shape = lpShape_Create( polys );
		ENSURE( CheckHullFromFaces( shape, &tally ) == 0 );
		lpShape_Destroy( shape );
	}
	ENSURE( tally.built == 50 && tally.quickMerged == 0 );
	PrintTally( "boxes", &tally );

	// Prisms up to the edge limit, and one over it, which quickhull refuses too
	memset( &tally, 0, sizeof( tally ) );
	for ( int n = 3; n <= 43; ++n )
	{
		MakePrism( polys, n, 0.5f, 0.3f );
		lpShape* shape = lpShape_Create( polys );
		ENSURE( CheckHullFromFaces( shape, &tally ) == 0 );
		lpShape_Destroy( shape );
	}
	ENSURE( tally.built == 40 && tally.neither == 1 && tally.maxEdges == 129 );
	PrintTally( "prisms of 3 to 43 sides", &tally );

	// Random convex hulls (quickhull's topology, Newell's planes)
	memset( &tally, 0, sizeof( tally ) );
	for ( int trial = 0; trial < 400; ++trial )
	{
		lpVec3 points[64];
		int count = 4 + (int)( lpRandom_Next( &rng ) % 60u );
		float scale = lpRandom_Range( &rng, 0.02f, 3.0f );
		for ( int i = 0; i < count; ++i )
		{
			points[i].x = scale * lpRandom_Range( &rng, -1.0f, 1.0f ); // one draw per statement
			points[i].y = scale * lpRandom_Range( &rng, -0.5f, 0.5f );
			points[i].z = scale * lpRandom_Range( &rng, -0.8f, 0.8f );
		}
		if ( lpPoly_MakeFromPoints( polys, points, count, 0 ) == false )
		{
			continue;
		}
		lpShape* shape = lpShape_Create( polys );
		ENSURE( CheckHullFromFaces( shape, &tally ) == 0 );
		lpShape_Destroy( shape );
	}
	ENSURE( tally.built > 350 && tally.refusedOther == 0 && tally.onlyFaces == 0 );
	PrintTally( "random convex hulls", &tally );

	// Boxes cut by random planes (the clip's topology and planes, as fracture cells have them)
	memset( &tally, 0, sizeof( tally ) );
	for ( int trial = 0; trial < 400; ++trial )
	{
		lpVec3 h;
		h.x = lpRandom_Range( &rng, 0.05f, 1.0f );
		h.y = lpRandom_Range( &rng, 0.05f, 1.0f );
		h.z = lpRandom_Range( &rng, 0.05f, 1.0f );
		lpTransform t = { { lpRandom_Range( &rng, -20.0f, 20.0f ), 0.0f, 0.0f }, lpQuat_identity };
		lpPoly_MakeBox( polys, h, t, 0 );
		int cuts = 1 + trial % 24;
		for ( int k = 0; k < cuts; ++k )
		{
			lpVec3 n;
			n.x = lpRandom_Range( &rng, -1.0f, 1.0f );
			n.y = lpRandom_Range( &rng, -1.0f, 1.0f );
			n.z = lpRandom_Range( &rng, -1.0f, 1.0f );
			n = lpNormalize( n );
			float reach = 0.6f * lpRandom_Range( &rng, 0.0f, 1.0f ) * lpMinFloat( h.x, lpMinFloat( h.y, h.z ) );
			lpPlane plane = { n, lpDot( n, t.p ) + reach };
			if ( lpPoly_Clip( polys, plane, 1, k, 1e-5f, polys + 1 ) == lp_clipCut )
			{
				polys[0] = polys[1];
			}
		}
		lpShape* shape = lpShape_Create( polys );
		ENSURE( CheckHullFromFaces( shape, &tally ) == 0 );
		lpShape_Destroy( shape );
	}
	ENSURE( tally.built > 350 && tally.refusedOther == 0 && tally.onlyFaces == 0 );
	PrintTally( "boxes cut by random planes", &tally );

	// The authored parts of real scenes, then their fracture cells and survivors after a bombardment
	const int scenes[5][2] = {
		{ lp_sceneWall, 12 }, { lp_sceneLumber, 12 }, { lp_sceneTown, 3 }, { lp_sceneRuins, 6 }, { lp_sceneKeep, 4 },
	};
	for ( int k = 0; k < 5; ++k )
	{
		Sim s = CreateSimWorkers( scenes[k][0], 4 );
		memset( &tally, 0, sizeof( tally ) );
		for ( int i = 0; i < s.world->pieces.count; ++i )
		{
			const lpShape* shape = s.world->pieces.data[i].shape;
			ENSURE( shape == NULL || CheckHullFromFaces( shape, &tally ) == 0 );
		}
		char what[64];
		snprintf( what, sizeof( what ), "%s, authored", lpSceneName( scenes[k][0] ) );
		PrintTally( what, &tally );
		ENSURE( tally.refusedOther == 0 && tally.onlyFaces == 0 );

		for ( int tick = 0; tick < 150; ++tick )
		{
			lpSceneBombard( s.world, scenes[k][0], tick, scenes[k][1] );
			lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		}
		memset( &tally, 0, sizeof( tally ) );
		for ( int i = 0; i < s.world->pieces.count; ++i )
		{
			const lpShape* shape = s.world->pieces.data[i].shape;
			ENSURE( shape == NULL || CheckHullFromFaces( shape, &tally ) == 0 );
		}
		snprintf( what, sizeof( what ), "%s, after 150 ticks", lpSceneName( scenes[k][0] ) );
		PrintTally( what, &tally );
		ENSURE( tally.refusedOther == 0 && tally.onlyFaces == 0 );
		DestroySim( &s );
	}

	// Bad input is refused: a box's arrays, spoilt one way at a time
	FaceArrays* good = malloc( 2 * sizeof( FaceArrays ) );
	FaceArrays* bad = good + 1;
	lpVec3 axis = lpNormalize( (lpVec3){ 1.0f, 2.0f, 3.0f } );
	lpTransform t = { { 1.0f, 2.0f, 3.0f }, lpMakeQuatFromAxisAngle( axis, 0.7f ) };
	lpPoly_MakeBox( polys, (lpVec3){ 0.5f, 0.4f, 0.3f }, t, 0 );
	GetFaceArrays( polys, good );
	ENSURE( BuildsHull( good ) );

	*bad = *good; // open: a face missing
	bad->faceCount = 5;
	ENSURE( BuildsHull( bad ) == false );

	*bad = *good; // a face flipped
	ReverseLoop( bad->indices, 4 );
	ENSURE( BuildsHull( bad ) == false );

	*bad = *good; // inside out: every loop reversed
	for ( int f = 0; f < 6; ++f )
	{
		ReverseLoop( bad->indices + 4 * f, 4 );
	}
	ENSURE( BuildsHull( bad ) == false );

	*bad = *good; // a plane facing in: the centroid is in front of it
	bad->planes[2] = (lpPlane){ lpNeg( good->planes[2].normal ), -good->planes[2].offset };
	ENSURE( BuildsHull( bad ) == false );

	*bad = *good; // an index out of range
	bad->indices[5] = 8;
	ENSURE( BuildsHull( bad ) == false );

	*bad = *good; // a point on no face
	bad->points[8] = (lpVec3){ 1.0f, 2.0f, 3.0f };
	bad->pointCount = 9;
	ENSURE( BuildsHull( bad ) == false );

	*bad = *good; // a loop that comes back to a point (an edge of no length)
	bad->indices[1] = bad->indices[0];
	ENSURE( BuildsHull( bad ) == false );

	*bad = *good; // a point that is not a number
	bad->points[3].y = NAN;
	ENSURE( BuildsHull( bad ) == false );

	*bad = *good; // a plane that is not of unit length
	bad->planes[0].normal = lpMulSV( 1.01f, bad->planes[0].normal );
	ENSURE( BuildsHull( bad ) == false );

	// Two copies of the box sharing two opposite corners and nothing else: closed, oriented, Euler's formula holds (14
	// points, 24 edges, 12 faces), and the geometry passes (twice the volume, the same centroid), but two fans meet at
	// each shared corner
	*bad = *good;
	for ( int i = 1; i < 7; ++i )
	{
		bad->points[7 + i] = good->points[i];
	}
	for ( int k = 0; k < 24; ++k )
	{
		uint8_t i = good->indices[k];
		bad->indices[24 + k] = i == 0 || i == 7 ? i : (uint8_t)( 7 + i );
	}
	for ( int f = 0; f < 6; ++f )
	{
		bad->planes[6 + f] = good->planes[f];
		bad->sizes[6 + f] = 4;
	}
	bad->pointCount = 14;
	bad->faceCount = 12;
	ENSURE( BuildsHull( bad ) == false );

	// Over the edge limit: a prism of 43 sides has 129 edges (quickhull refuses it too); 42 sides, 126 edges, is built
	MakePrism( polys, 43, 0.5f, 0.3f );
	GetFaceArrays( polys, bad );
	ENSURE( BuildsHull( bad ) == false );
	MakePrism( polys, 42, 0.5f, 0.3f );
	GetFaceArrays( polys, bad );
	ENSURE( BuildsHull( bad ) );

	free( good );
	free( polys );
	return 0;
}

// What C6 is for: building physics hulls from the faces costs less than quickhull on the same shapes (printed)
static int TestHullFromFacesCost( void )
{
	Sim s = CreateSimWorkers( lp_sceneTown, 4 );
	for ( int tick = 0; tick < 150; ++tick )
	{
		lpSceneBombard( s.world, lp_sceneTown, tick, 3 );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
	}
	const lpShape** shapes = malloc( sizeof( lpShape* ) * (size_t)s.world->pieces.count );
	int count = 0, vertices = 0, faces = 0;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		const lpShape* shape = s.world->pieces.data[i].shape;
		if ( shape != NULL && shape->vertexCount + shape->faceCount - 2 <= LP_PHYS_MAX_HULL_EDGES )
		{
			shapes[count++] = shape;
			vertices += shape->vertexCount;
			faces += shape->faceCount;
		}
	}
	ENSURE( count > 1000 );

	enum
	{
		lp_rounds = 5
	};
	float best[2] = { 1e9f, 1e9f };
	int built[2] = { 0, 0 };
	for ( int round = 0; round < lp_rounds; ++round )
	{
		for ( int builder = 0; builder < 2; ++builder )
		{
			built[builder] = 0;
			uint64_t ticks = lpGetTicks();
			for ( int i = 0; i < count; ++i )
			{
				lpPhysHull* hull =
					builder == 0 ? lpShape_CreateHull( shapes[i] ) : lpShape_CreateHullFromFaces( shapes[i] );
				if ( hull != NULL )
				{
					built[builder] += 1;
					lpPhys_DestroyHull( hull );
				}
			}
			float ms = lpGetMilliseconds( ticks );
			best[builder] = ms < best[builder] ? ms : best[builder];
		}
	}
	ENSURE( built[1] > 0 );
	printf( "  %d town pieces after a barrage (%.1f vertices, %.1f faces each): quickhull %.2f us a hull (%d built), "
			"from faces %.2f us (%d built): %.2fx faster, best of %d\n",
			count, (double)vertices / count, (double)faces / count, 1000.0 * best[0] / count, built[0],
			1000.0 * best[1] / count, built[1], (double)( best[0] / best[1] ), lp_rounds );
	free( (void*)shapes );
	DestroySim( &s );
	return 0;
}

int PolyTest( void )
{
	RUN_TEST( TestBox, MECHANISM );
	RUN_TEST( TestShapeDigest, MECHANISM );
	RUN_TEST( TestClipBox, MECHANISM );
	RUN_TEST( TestClipFuzz, MECHANISM );
	RUN_TEST( TestContactArea, MECHANISM );
	RUN_TEST( TestHullFromFaces, MECHANISM );
	RUN_TEST( TestHullFromFacesCost, TIMING );
	return 0;
}
