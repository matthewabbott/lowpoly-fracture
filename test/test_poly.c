// SPDX-License-Identifier: MIT

#include "poly.h"
#include "test_macros.h"

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
			points[i] = (lpVec3){ lpRandom_Range( &rng, -1.0f, 1.0f ), lpRandom_Range( &rng, -0.5f, 0.5f ),
								  lpRandom_Range( &rng, -0.8f, 0.8f ) };
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
			lpVec3 n = lpNormalize( (lpVec3){ lpRandom_Range( &rng, -1.0f, 1.0f ), lpRandom_Range( &rng, -1.0f, 1.0f ),
											  lpRandom_Range( &rng, -1.0f, 1.0f ) } );
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

int PolyTest( void )
{
	RUN_TEST( TestBox, MECHANISM );
	RUN_TEST( TestClipBox, MECHANISM );
	RUN_TEST( TestClipFuzz, MECHANISM );
	RUN_TEST( TestContactArea, MECHANISM );
	return 0;
}
