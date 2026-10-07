// SPDX-License-Identifier: MIT

#include "fracture.h"

#include <float.h>

// Sites inside the parent: dense near the impact (density ~ 1/distance, the ejecta), 4-6 ring sites around the
// damage radius that shape a jagged rim, and at most three far sites that keep the rest of the piece in large plates.
typedef struct lpSiteParams
{
	lpVec3 impact;
	float radius;
	float fragmentSize;
	float plateSize;
	int maxSites;
	lpVec3 grainAxis; // ring sites stay out of a 25 degree cone around this axis; zero for none
	float stretch;	  // the parent is squashed along grainAxis by this (> 1): distances along it count this much more
	int ringSites;	  // how many ring sites to aim for (0 for none: a broken log end is one piece)
} lpSiteParams;

static uint32_t lpFloatBits( float value )
{
	uint32_t bits;
	memcpy( &bits, &value, sizeof( bits ) );
	return bits;
}

static float lpBitsToFloat( uint32_t bits )
{
	float value;
	memcpy( &value, &bits, sizeof( value ) );
	return value;
}

// Keys are unique (the site index is in the low bits), so any correct sort gives the same order.
static void lpSortKeys( uint64_t* keys, int count )
{
	for ( int i = 1; i < count; ++i )
	{
		uint64_t key = keys[i];
		int j = i - 1;
		while ( j >= 0 && keys[j] > key )
		{
			keys[j + 1] = keys[j];
			j -= 1;
		}
		keys[j + 1] = key;
	}
}

// Voronoi cell of site `index`, clipped to the parent. Returns the cell in `out`. Returns false if the
// cell is empty or a clip failed. Neighbor sites are visited nearest first, and the search stops once the
// next site is farther than twice the current cell radius (no further plane can cut).
static bool lpComputeVoronoiCell( const lpPoly* parent, const lpVec3* sites, int siteCount, int index, uint8_t material,
						   float tolerance, lpPoly* scratch, lpPoly* out, lpFractureStats* stats, lpClipStats* clipStats )
{
	lpVec3 site = sites[index];

	uint64_t keys[LP_MAX_SITES];
	int keyCount = 0;
	for ( int j = 0; j < siteCount && keyCount < LP_MAX_SITES; ++j )
	{
		if ( j == index )
		{
			continue;
		}
		// Non-negative floats order like their bit patterns
		float d2 = lpDistanceSquared( site, sites[j] );
		keys[keyCount++] = ( (uint64_t)lpFloatBits( d2 ) << 32 ) | (uint32_t)j;
	}
	lpSortKeys( keys, keyCount );

	*out = *parent;
	lpPoly* current = out;
	lpPoly* next = scratch;
	float radiusSquared = lpPoly_MaxDistanceSquared( current, site );

	for ( int k = 0; k < keyCount; ++k )
	{
		float d2 = lpBitsToFloat( (uint32_t)( keys[k] >> 32 ) );

		// The bisector plane is at half the distance; it can only cut if that is inside the cell radius.
		if ( d2 >= 4.0f * radiusSquared )
		{
			break;
		}

		int j = (int)( keys[k] & 0xFFFFFFFFu );
		lpVec3 other = sites[j];
		lpVec3 normal = lpNormalize( lpSub( other, site ) );
		lpVec3 mid = lpMulSV( 0.5f, lpAdd( site, other ) );
		lpPlane plane = { normal, lpDot( normal, mid ) };

		lpClipResult result = lpPoly_ClipCounted( current, plane, material, j, tolerance, next, clipStats );

		if ( result == lp_clipCut )
		{
			lpPoly* t = current;
			current = next;
			next = t;
			radiusSquared = lpPoly_MaxDistanceSquared( current, site );
		}
		else if ( result == lp_clipEmpty )
		{
			return false;
		}
		else if ( result != lp_clipUnchanged )
		{
			// Numerical failure: skip this plane. The cell overlaps its neighbor slightly, which is harmless
			// for rendering and physics.
			if ( stats != NULL )
			{
				stats->failureCount += 1;
			}
		}
	}

	if ( current != out )
	{
		*out = *current;
	}
	return true;
}

// ---- site generation ----

static lpVec3 lpRandomInBox( lpRandom* rng, lpAABB box )
{
	lpVec3 p; // one draw per statement: C leaves the order inside an initializer open
	p.x = lpRandom_Range( rng, box.lowerBound.x, box.upperBound.x );
	p.y = lpRandom_Range( rng, box.lowerBound.y, box.upperBound.y );
	p.z = lpRandom_Range( rng, box.lowerBound.z, box.upperBound.z );
	return p;
}

// Uniform unit vector by rejection sampling: no trigonometry, so bit-identical everywhere.
static lpVec3 lpRandomUnitVector( lpRandom* rng )
{
	for ( int i = 0; i < 64; ++i )
	{
		lpVec3 p; // one draw per statement: C leaves the order inside an initializer open
		p.x = lpRandom_Range( rng, -1.0f, 1.0f );
		p.y = lpRandom_Range( rng, -1.0f, 1.0f );
		p.z = lpRandom_Range( rng, -1.0f, 1.0f );
		float l2 = lpLengthSquared( p );
		if ( l2 > 0.01f && l2 <= 1.0f )
		{
			return lpMulSV( 1.0f / sqrtf( l2 ), p );
		}
	}
	return (lpVec3){ 0.0f, 1.0f, 0.0f };
}

static bool lpIsTooClose( lpVec3 p, const lpVec3* sites, int count, float spacingSquared )
{
	for ( int i = 0; i < count; ++i )
	{
		if ( lpDistanceSquared( p, sites[i] ) < spacingSquared )
		{
			return true;
		}
	}
	return false;
}

static float lpOverlap1( float lo, float hi, float a, float b )
{
	float l = lo > a ? lo : a;
	float h = hi < b ? hi : b;
	return h > l ? h - l : 0.0f;
}

// Distance from the focus in unsquashed (real) space, so the damage sphere stays a sphere on a grained piece
static float lpSiteDistance( const lpSiteParams* params, lpVec3 p, lpVec3 focus )
{
	lpVec3 v = lpSub( p, focus );
	if ( params->stretch > 1.0f )
	{
		v = lpMulAdd( v, ( params->stretch - 1.0f ) * lpDot( v, params->grainAxis ), params->grainAxis );
	}
	return lpLength( v );
}

static int lpGenerateImpactSites( const lpPoly* parent, const lpSiteParams* params, lpRandom* rng, lpVec3* sites )
{
	int maxSites = params->maxSites < LP_MAX_SITES ? params->maxSites : LP_MAX_SITES;
	float radius = params->radius;
	float fragmentSize = params->fragmentSize;
	lpAABB bounds = lpPoly_ComputeBounds( parent );
	lpVec3 focus = lpClamp( params->impact, bounds.lowerBound, bounds.upperBound );
	lpVec3 extent = lpSub( bounds.upperBound, bounds.lowerBound );

	// Sites too close to the surface make thin slivers against it; keep them in by a margin, but never so much that
	// a thin plank or pane has no room left.
	float spacing = 0.7f * fragmentSize;
	float thinnest = 0.5f * lpMinFloat( extent.x, lpMinFloat( extent.y, extent.z ) );
	float margin = lpMinFloat( 0.5f * spacing, 0.45f * thinnest );
	float minRadius = lpMaxFloat( 0.15f * radius, spacing );

	// Box around the damage sphere: an ellipsoid with semi-axis radius / stretch along the grain when squashed
	lpVec3 reach = { radius, radius, radius };
	if ( params->stretch > 1.0f )
	{
		lpVec3 a = params->grainAxis;
		float f = 1.0f - 1.0f / ( params->stretch * params->stretch );
		reach = (lpVec3){ radius * sqrtf( 1.0f - f * a.x * a.x ), radius * sqrtf( 1.0f - f * a.y * a.y ),
						  radius * sqrtf( 1.0f - f * a.z * a.z ) };
	}

	// Damaged volume: that box clipped to the bounds, scaled to an ellipsoid.
	float ox = lpOverlap1( bounds.lowerBound.x, bounds.upperBound.x, focus.x - reach.x, focus.x + reach.x );
	float oy = lpOverlap1( bounds.lowerBound.y, bounds.upperBound.y, focus.y - reach.y, focus.y + reach.y );
	float oz = lpOverlap1( bounds.lowerBound.z, bounds.upperBound.z, focus.z - reach.z, focus.z + reach.z );
	float damagedVolume = 0.5236f * ox * oy * oz;
	float cellVolume = fragmentSize * fragmentSize * fragmentSize;

	// A few ring and plate sites are always reserved; the inner (ejecta) sites get the rest of the budget.
	int innerTarget = lpFloatToInt( damagedVolume / cellVolume );
	innerTarget = innerTarget < 2 ? 2 : innerTarget;
	innerTarget = innerTarget > maxSites - 9 ? maxSites - 9 : innerTarget;
	innerTarget = innerTarget < 2 ? 2 : innerTarget;

	int count = 0;
	lpAABB innerBox = {
		lpMax( bounds.lowerBound, lpSub( focus, reach ) ),
		lpMin( bounds.upperBound, lpAdd( focus, reach ) ),
	};

	int attempts = 30 * innerTarget + 64;
	for ( int a = 0; a < attempts && count < innerTarget; ++a )
	{
		lpVec3 p = lpRandomInBox( rng, innerBox );
		float d = lpSiteDistance( params, p, focus );
		if ( d > radius )
		{
			continue;
		}
		// density ~ 1/d: small fragments at the focus, larger toward the rim
		float u = lpRandom_Float( rng );
		if ( d > minRadius && u * d > minRadius )
		{
			continue;
		}
		if ( lpPoly_SignedDistance( parent, p ) > -margin || lpIsTooClose( p, sites, count, spacing * spacing ) )
		{
			continue;
		}
		sites[count++] = p;
	}

	// A handful of ring sites around the damage radius: their cells stay on the piece and give the hole (or the broken
	// end of a log) a jagged rim of a few big facets. Sites near the grain axis are skipped so the rim cuts across the
	// grain at varied angles instead of splitting the log lengthwise.
	int ringTarget = params->ringSites;
	float ringSpacing = lpMaxFloat( spacing, 0.6f * radius );
	int ringEnd = count + ringTarget < maxSites ? count + ringTarget : maxSites;
	bool avoidAxis = lpLengthSquared( params->grainAxis ) > 0.0f;
	for ( int a = 0; a < 12 * ringTarget && count < ringEnd; ++a )
	{
		lpVec3 dir = lpRandomUnitVector( rng );
		if ( avoidAxis && lpAbsFloat( lpDot( dir, params->grainAxis ) ) > 0.906f ) // within 25 degrees
		{
			continue;
		}
		if ( params->stretch > 1.0f )
		{
			dir = lpMulSub( dir, ( 1.0f - 1.0f / params->stretch ) * lpDot( dir, params->grainAxis ), params->grainAxis );
		}
		float r = radius * lpRandom_Range( rng, 0.95f, 1.5f );
		lpVec3 p = lpMulAdd( focus, r, dir );
		if ( lpPoly_SignedDistance( parent, p ) > -margin || lpIsTooClose( p, sites, count, ringSpacing * ringSpacing ) )
		{
			continue;
		}
		sites[count++] = p;
	}

	// Grained pieces get an anchor site on the grain line through the middle, just past the damage radius on each
	// side. Each claims the whole remaining end of a log or plank, which then breaks off as one piece whose broken
	// face is the few jagged planes it shares with the splinter sites.
	if ( params->stretch > 1.0f )
	{
		lpVec3 a = params->grainAxis;
		lpVec3 center = lpLerp( bounds.lowerBound, bounds.upperBound, 0.5f );
		float along = lpDot( lpSub( focus, center ), a );
		for ( int side = -1; side <= 1 && count < maxSites; side += 2 )
		{
			float t = along + (float)side * lpRandom_Range( rng, 1.15f, 1.4f ) * radius / params->stretch;
			lpVec3 p = lpMulAdd( center, t, a );
			if ( lpPoly_SignedDistance( parent, p ) > -margin || lpIsTooClose( p, sites, count, spacing * spacing ) )
			{
				continue;
			}
			sites[count++] = p;
		}
	}

	// At most three far sites: the rest of the piece stays in a few large plates (a log keeps two whole ends)
	float totalVolume = extent.x * extent.y * extent.z;
	float plate = lpMaxFloat( lpMaxFloat( 3.0f * fragmentSize, 1.2f * radius ), params->plateSize );
	int farTarget = lpFloatToInt( ( totalVolume - damagedVolume ) / ( plate * plate * plate ) );
	farTarget = farTarget < 0 ? 0 : ( farTarget > 3 ? 3 : farTarget );
	int farEnd = count + farTarget < maxSites ? count + farTarget : maxSites;
	float farSpacing = 0.8f * plate;
	for ( int a = 0; a < 16 * farTarget && count < farEnd; ++a )
	{
		lpVec3 p = lpRandomInBox( rng, bounds );
		if ( lpSiteDistance( params, p, focus ) < 1.4f * radius )
		{
			continue;
		}
		if ( lpPoly_SignedDistance( parent, p ) > -margin || lpIsTooClose( p, sites, count, farSpacing * farSpacing ) )
		{
			continue;
		}
		sites[count++] = p;
	}

	return count;
}

// ---- patterns ----

static int lpFractureVoronoi( const lpFractureInput* input, lpRandom* rng, lpShape** cells, int* cellSites, int capacity,
							  lpFractureStats* stats )
{
	lpPoly* work = lpAlloc( 3 * sizeof( lpPoly ) );
	lpPoly* parent = work;
	lpPoly* cell = work + 1;
	lpPoly* scratch = work + 2;

	*parent = *input->parent;
	lpVec3 impact = input->impact;
	lpMatrix3 unsquash = { 0 };

	bool grain = input->pattern == lp_breakGrain && input->stretch > 1.0f;
	if ( grain )
	{
		// Squash along the grain: isotropic Voronoi there becomes cells `stretch` times longer after unsquashing.
		lpVec3 a = input->axis;
		float s = 1.0f / input->stretch;
		float k = s - 1.0f;
		lpMatrix3 squash = {
			{ 1.0f + k * a.x * a.x, k * a.y * a.x, k * a.z * a.x },
			{ k * a.x * a.y, 1.0f + k * a.y * a.y, k * a.z * a.y },
			{ k * a.x * a.z, k * a.y * a.z, 1.0f + k * a.z * a.z },
		};
		float g = input->stretch - 1.0f;
		unsquash = (lpMatrix3){
			{ 1.0f + g * a.x * a.x, g * a.y * a.x, g * a.z * a.x },
			{ g * a.x * a.y, 1.0f + g * a.y * a.y, g * a.z * a.y },
			{ g * a.x * a.z, g * a.y * a.z, 1.0f + g * a.z * a.z },
		};
		lpPoly_ApplyLinear( parent, squash );
		impact = lpMulMV( squash, impact );
	}

	lpVec3 sites[LP_MAX_SITES];
	lpSiteParams params = { 0 };
	params.impact = impact;
	params.radius = input->radius;
	params.fragmentSize = input->fragmentSize;
	params.plateSize = grain ? input->plateSize / input->stretch : input->plateSize;
	params.maxSites = input->maxCells < capacity ? input->maxCells : capacity;
	params.grainAxis = grain ? input->axis : lpVec3_zero;
	params.stretch = grain ? input->stretch : 1.0f;
	// Walls get a ring of rim cells for a jagged hole; a snapped log keeps each end whole, its broken face already
	// jagged from the splinters that left it.
	params.ringSites = grain ? 0 : 5;
	int siteCount = lpGenerateImpactSites( parent, &params, rng, sites );
	lpClipStats* clipStats = stats != NULL ? stats->clips + ( grain ? lp_clipGrain : lp_clipVoronoi ) : NULL;
	if ( stats != NULL )
	{
		stats->sitesDrawn += siteCount;
	}

	// Compute all cells. Tiny cells outside the damage radius are slivers: drop their sites and recompute, which
	// hands their volume to the neighbours while keeping the tiling exact and every cell convex.
	int count = 0;
	int dropped = 0;
	for ( int pass = 0; pass < 3 && siteCount >= 2; ++pass )
	{
		count = 0;
		dropped = 0;
		for ( int i = 0; i < siteCount && count < capacity; ++i )
		{
			if ( lpComputeVoronoiCell( parent, sites, siteCount, i, input->interiorMaterial, input->tolerance, scratch,
									   cell, stats, clipStats ) == false )
			{
				dropped += 1;
				continue;
			}
			if ( grain )
			{
				lpPoly_ApplyLinear( cell, unsquash );
			}
			lpShape* shape = lpShape_Create( cell );
			if ( shape != NULL )
			{
				if ( cellSites != NULL )
				{
					cellSites[count] = i;
				}
				cells[count++] = shape;
			}
			else
			{
				dropped += 1;
			}
		}

		if ( pass == 2 || input->absorbVolume <= 0.0f || cellSites == NULL )
		{
			break;
		}

		bool drop[LP_MAX_SITES] = { false };
		int dropCount = 0;
		float r2 = input->radius * input->radius;
		for ( int c = 0; c < count; ++c )
		{
			if ( cells[c]->volume < input->absorbVolume && lpDistanceSquared( cells[c]->centroid, input->impact ) > r2 )
			{
				drop[cellSites[c]] = true;
				dropCount += 1;
			}
		}
		if ( dropCount == 0 || siteCount - dropCount < 2 )
		{
			break;
		}
		if ( stats != NULL )
		{
			stats->sliversAbsorbed += dropCount;
		}

		for ( int c = 0; c < count; ++c )
		{
			lpShape_Destroy( cells[c] );
		}
		int kept = 0;
		for ( int i = 0; i < siteCount; ++i )
		{
			if ( drop[i] == false )
			{
				sites[kept++] = sites[i];
			}
		}
		siteCount = kept;
		count = 0;
	}
	if ( stats != NULL )
	{
		stats->cellsDropped += dropped;
	}

	lpFree( work );
	return count;
}

static int lpFractureRadial( const lpFractureInput* input, lpRandom* rng, lpShape** cells, int* cellSites, int capacity,
							 lpFractureStats* stats )
{
	lpPoly* work = lpAlloc( 2 * sizeof( lpPoly ) );
	lpPoly* a = work;
	lpPoly* b = work + 1;

	lpVec3 n = input->axis;
	lpVec3 t = lpAbsFloat( n.x ) < 0.57f ? (lpVec3){ 1.0f, 0.0f, 0.0f } : (lpVec3){ 0.0f, 1.0f, 0.0f };
	lpVec3 u = lpNormalize( lpCross( t, n ) );
	lpVec3 v = lpCross( n, u );
	lpVec3 p = input->impact;

	// Wedge boundaries around the impact axis, jittered
	enum
	{
		lp_maxWedges = 12,
		lp_maxRings = 8
	};
	int wedgeCount = 6 + (int)( lpRandom_Next( rng ) % 5u );
	float step = 2.0f * LP_PI / (float)wedgeCount;
	lpVec3 dirs[lp_maxWedges + 1];
	for ( int k = 0; k < wedgeCount; ++k )
	{
		float angle = step * ( (float)k + lpRandom_Range( rng, -0.3f, 0.3f ) );
		lpCosSin cs = lpComputeCosSin( angle );
		dirs[k] = lpAdd( lpMulSV( cs.cosine, u ), lpMulSV( cs.sine, v ) );
	}
	dirs[wedgeCount] = dirs[0];

	// Concentric ring radii, growing geometrically out to the damage radius
	float rings[lp_maxRings];
	int ringCount = 0;
	float r = 1.2f * input->fragmentSize;
	while ( ringCount < lp_maxRings && r < input->radius )
	{
		rings[ringCount++] = r;
		r *= lpRandom_Range( rng, 1.6f, 2.0f );
	}

	int count = 0;
	for ( int k = 0; k < wedgeCount && count < capacity; ++k )
	{
		lpVec3 d0 = dirs[k];
		lpVec3 d1 = dirs[k + 1];
		lpVec3 perp0 = lpCross( n, d0 );
		lpVec3 perp1 = lpCross( n, d1 );
		lpVec3 mid = lpNormalize( lpAdd( d0, d1 ) );
		float cosHalf = lpDot( mid, d0 );

		lpPlane side0 = { lpNeg( perp0 ), -lpDot( perp0, p ) };
		lpPlane side1 = { perp1, lpDot( perp1, p ) };
		int32_t tag0 = 1000 + k;
		int32_t tag1 = 1000 + ( k + 1 ) % wedgeCount;

		for ( int j = 0; j <= ringCount && count < capacity; ++j )
		{
			*a = *input->parent;
			lpPoly* cur = a;
			lpPoly* nxt = b;
			bool empty = false;

			lpPlane planes[4];
			int32_t tags[4];
			int planeCount = 0;
			planes[planeCount] = side0;
			tags[planeCount++] = tag0;
			planes[planeCount] = side1;
			tags[planeCount++] = tag1;
			if ( j < ringCount )
			{
				planes[planeCount] = (lpPlane){ mid, lpDot( mid, p ) + rings[j] * cosHalf };
				tags[planeCount++] = 2000 + k * 16 + j;
			}
			if ( j > 0 )
			{
				planes[planeCount] = (lpPlane){ lpNeg( mid ), -( lpDot( mid, p ) + rings[j - 1] * cosHalf ) };
				tags[planeCount++] = 2000 + k * 16 + j - 1;
			}

			for ( int q = 0; q < planeCount; ++q )
			{
				lpClipResult result = lpPoly_ClipCounted( cur, planes[q], input->interiorMaterial, tags[q], input->tolerance, nxt,
														  stats != NULL ? stats->clips + lp_clipRadial : NULL );
				if ( result == lp_clipCut )
				{
					lpPoly* tmp = cur;
					cur = nxt;
					nxt = tmp;
				}
				else if ( result == lp_clipEmpty )
				{
					empty = true;
					break;
				}
				else if ( result != lp_clipUnchanged && stats != NULL )
				{
					stats->failureCount += 1;
				}
			}

			if ( empty == false )
			{
				lpShape* shape = lpShape_Create( cur );
				if ( shape != NULL )
				{
					if ( cellSites != NULL )
					{
						cellSites[count] = -1;
					}
					cells[count++] = shape;
				}
				else if ( stats != NULL )
				{
					stats->cellsDropped += 1;
				}
			}
		}
	}


	lpFree( work );
	return count;
}

// Keep the part of `in` behind the plane (dot(n, x) <= d) in `out`; false if nothing is left
static bool lpKeepBehind( const lpPoly* in, lpVec3 n, float d, const lpFractureInput* input, lpPoly* out, lpFractureStats* stats )
{
	lpClipResult result = lpPoly_ClipCounted( in, (lpPlane){ n, d }, input->interiorMaterial, LP_TAG_CUT, input->tolerance, out,
											  stats != NULL ? stats->clips + lp_clipMasonry : NULL );
	if ( result == lp_clipUnchanged )
	{
		*out = *in;
		return true;
	}
	if ( result == lp_clipCut )
	{
		return true;
	}
	if ( result != lp_clipEmpty && stats != NULL )
	{
		stats->failureCount += 1;
	}
	return false;
}

// The slab of `in` between two parallel planes along `n`: lo <= dot(n, x) <= hi
static bool lpKeepBetween( const lpPoly* in, lpVec3 n, float lo, float hi, const lpFractureInput* input, lpPoly* scratch,
						   lpPoly* out, lpFractureStats* stats )
{
	return lpKeepBehind( in, n, hi, input, scratch, stats ) && lpKeepBehind( scratch, lpNeg( n ), -lo, input, out, stats );
}

static int lpAddMasonryCell( const lpPoly* poly, lpShape** cells, int* cellSites, int count, int capacity )
{
	if ( count >= capacity )
	{
		return count;
	}
	lpShape* shape = lpShape_Create( poly );
	if ( shape != NULL )
	{
		cellSites[count] = -1;
		cells[count++] = shape;
	}
	return count;
}

// Brick walls break along their mortar. The parent is one solid panel until it is hit; then, in a course grid shared by
// the whole wall, the bricks within the break radius of each course come out (those near the centre as chips, the rest
// whole) and the rest of the wall is cut on mortar lines only: one plate below the damaged courses, one above, and a run
// on each side of the hole in every damaged course. The alternating offsets of the courses make the hole stair-stepped,
// and every cut is a bed or head joint, so later stress cracks follow the mortar too.
static int lpFractureMasonry( const lpFractureInput* input, lpRandom* rng, lpShape** cells, int* cellSites, int capacity,
							  lpFractureStats* stats )
{
	enum
	{
		lp_maxCourses = 48
	};
	lpVec3 up = { 0.0f, 1.0f, 0.0f };
	lpVec3 run = { input->axis.x, 0.0f, input->axis.z };
	run = lpLengthSquared( run ) > 1e-6f ? lpNormalize( run ) : (lpVec3){ 1.0f, 0.0f, 0.0f };
	float h = input->courseHeight;
	float l = input->brickLength;
	lpVec3 o = input->gridOrigin;
	const lpPoly* parent = input->parent;

	float y0 = FLT_MAX, y1 = -FLT_MAX;
	for ( int i = 0; i < parent->vertexCount; ++i )
	{
		float y = lpDot( lpSub( parent->vertices[i], o ), up );
		y0 = y < y0 ? y : y0;
		y1 = y > y1 ? y : y1;
	}
	float yi = lpDot( lpSub( input->impact, o ), up );
	float ui = lpDot( lpSub( input->impact, o ), run );
	float r = input->radius;
	int k0 = (int)floorf( y0 / h + 1e-4f );
	int k1 = (int)ceilf( y1 / h - 1e-4f ) - 1;
	if ( k1 - k0 + 1 > lp_maxCourses )
	{
		return 0; // a wall this tall in one piece: let the impact pattern handle it
	}

	// The bricks each course loses: those whose centre lies inside the damage sphere
	int first[lp_maxCourses], last[lp_maxCourses];
	int low = INT32_MAX, high = INT32_MIN, bricks = 0;
	for ( int k = k0; k <= k1; ++k )
	{
		int row = k - k0;
		first[row] = 1;
		last[row] = 0;
		float cy = ( (float)k + 0.5f ) * h;
		float dy = lpAbsFloat( cy - yi );
		float offset = ( k & 1 ) ? 0.5f * l : 0.0f;
		if ( dy < r )
		{
			float half = sqrtf( r * r - dy * dy );
			first[row] = (int)ceilf( ( ui - half - offset ) / l - 0.5f );
			last[row] = (int)floorf( ( ui + half - offset ) / l - 0.5f );
		}
		if ( first[row] > last[row] && yi >= (float)k * h && yi < (float)( k + 1 ) * h )
		{
			first[row] = last[row] = (int)floorf( ( ui - offset ) / l ); // at least the brick that was hit
		}
		if ( first[row] <= last[row] )
		{
			low = k < low ? k : low;
			high = k > high ? k : high;
			bricks += last[row] - first[row] + 1;
		}
	}
	if ( low > high )
	{
		return 0;
	}

	// Loose bricks come out in groups when there are more than the cells allow
	int courses = high - low + 1;
	int room = capacity - 2 - 2 * courses;
	int group = 1;
	while ( group < 8 && bricks / group > room - 8 )
	{
		group += 1;
	}

	lpPoly* work = lpAlloc( 4 * sizeof( lpPoly ) );
	lpPoly* slab = work;
	lpPoly* piece = work + 1;
	lpPoly* scratch = work + 2;
	lpPoly* brick = work + 3;
	int count = 0;

	if ( (float)low * h > y0 + 1e-3f && lpKeepBehind( parent, up, (float)low * h + lpDot( o, up ), input, piece, stats ) )
	{
		count = lpAddMasonryCell( piece, cells, cellSites, count, capacity ); // the wall below the hole
	}
	if ( (float)( high + 1 ) * h < y1 - 1e-3f &&
		 lpKeepBehind( parent, lpNeg( up ), -( (float)( high + 1 ) * h + lpDot( o, up ) ), input, piece, stats ) )
	{
		count = lpAddMasonryCell( piece, cells, cellSites, count, capacity ); // the wall above it
	}

	float oy = lpDot( o, up );
	float ou = lpDot( o, run );
	for ( int k = low; k <= high; ++k )
	{
		int row = k - k0;
		if ( lpKeepBetween( parent, up, (float)k * h + oy, (float)( k + 1 ) * h + oy, input, scratch, slab, stats ) == false )
		{
			continue;
		}
		if ( first[row] > last[row] )
		{
			count = lpAddMasonryCell( slab, cells, cellSites, count, capacity ); // an intact course inside the hole's span
			continue;
		}
		float offset = ( k & 1 ) ? 0.5f * l : 0.0f;
		float ua = (float)first[row] * l + offset + ou;
		float ub = (float)( last[row] + 1 ) * l + offset + ou;
		if ( lpKeepBehind( slab, run, ua, input, piece, stats ) )
		{
			count = lpAddMasonryCell( piece, cells, cellSites, count, capacity );
		}
		if ( lpKeepBehind( slab, lpNeg( run ), -ub, input, piece, stats ) )
		{
			count = lpAddMasonryCell( piece, cells, cellSites, count, capacity );
		}
		for ( int j = first[row]; j <= last[row]; j += group )
		{
			int end = j + group - 1 < last[row] ? j + group - 1 : last[row];
			float lo = (float)j * l + offset + ou;
			float hi = (float)( end + 1 ) * l + offset + ou;
			if ( lpKeepBetween( slab, run, lo, hi, input, scratch, brick, stats ) == false )
			{
				continue;
			}
			// Bricks near the centre shatter into chips; further out they come loose whole
			lpVec3 centre = lpAdd( lpMulAdd( o, 0.5f * ( lo + hi ) - ou, run ), lpMulSV( ( (float)k + 0.5f ) * h, up ) );
			lpShape* shape = lpShape_Create( brick );
			if ( shape == NULL )
			{
				continue;
			}
			lpShape* chips[4];
			int chipCount = 0;
			if ( group == 1 && lpDistance( centre, input->impact ) < 0.5f * r && count + 4 <= capacity )
			{
				chipCount = lpChipCell( shape, 2, lpVec3_zero, input->interiorMaterial, 1e-5f, rng, chips, 4, stats );
			}
			if ( chipCount > 0 )
			{
				lpShape_Destroy( shape );
				for ( int c = 0; c < chipCount; ++c )
				{
					cellSites[count] = -1;
					cells[count++] = chips[c];
				}
			}
			else if ( count < capacity )
			{
				cellSites[count] = -1;
				cells[count++] = shape;
			}
			else
			{
				lpShape_Destroy( shape );
			}
		}
	}
	lpFree( work );
	return count;
}

// A beam giving way under load: one cut across its axis through the overloaded point, tilted a little at random so
// it reads as broken, not sawn. Two cells; the blow's damage at the cut keeps them apart.
static int lpFractureSnap( const lpFractureInput* input, lpRandom* rng, lpShape** cells, int* cellSites, int capacity,
						   lpFractureStats* stats )
{
	if ( capacity < 2 )
	{
		return 0;
	}
	lpVec3 n = input->axis;
	lpVec3 t1, t2;
	lpContactBasis( n, &t1, &t2 );
	// Two draws as statements: C leaves the order of a call's arguments unspecified (MSVC and gcc on x64 took the
	// second first, clang and gcc on ARM64 the first), so draws inside one call's arguments differ by compiler
	float tilt2 = lpRandom_Range( rng, -0.35f, 0.35f );
	float tilt1 = lpRandom_Range( rng, -0.35f, 0.35f );
	n = lpNormalize( lpAdd( n, lpAdd( lpMulSV( tilt1, t1 ), lpMulSV( tilt2, t2 ) ) ) );
	lpPlane plane = { n, lpDot( n, input->impact ) };
	lpPlane flipped = { lpNeg( n ), -plane.offset };

	lpPoly* halves = lpAlloc( 2 * sizeof( lpPoly ) );
	lpClipStats* clipStats = stats != NULL ? stats->clips + lp_clipSnap : NULL;
	int count = 0;
	if ( lpPoly_ClipCounted( input->parent, plane, input->interiorMaterial, LP_TAG_CUT, input->tolerance, halves, clipStats ) ==
			 lp_clipCut &&
		 lpPoly_ClipCounted( input->parent, flipped, input->interiorMaterial, LP_TAG_CUT, input->tolerance, halves + 1,
							 clipStats ) == lp_clipCut )
	{
		for ( int i = 0; i < 2; ++i )
		{
			lpShape* shape = lpShape_Create( halves + i );
			if ( shape != NULL )
			{
				cellSites[count] = -1;
				cells[count++] = shape;
			}
		}
	}
	else if ( stats != NULL )
	{
		stats->failureCount += 1;
	}
	lpFree( halves );
	return count;
}

static int lpFracturePattern( const lpFractureInput* input, lpShape** cells, int* cellSites, int capacity, lpFractureStats* stats )
{
	lpRandom rng;
	lpRandom_Seed( &rng, input->seed, 0x5EEDu );

	if ( input->snap && cellSites != NULL )
	{
		return lpFractureSnap( input, &rng, cells, cellSites, capacity, stats );
	}
	if ( input->pattern == lp_breakRadial )
	{
		return lpFractureRadial( input, &rng, cells, cellSites, capacity, stats );
	}
	if ( input->pattern == lp_breakMasonry && cellSites != NULL && input->courseHeight > 0.0f && input->brickLength > 0.0f )
	{
		int count = lpFractureMasonry( input, &rng, cells, cellSites, capacity, stats );
		if ( count >= 2 )
		{
			return count;
		}
		for ( int i = 0; i < count; ++i )
		{
			lpShape_Destroy( cells[i] );
		}
	}
	return lpFractureVoronoi( input, &rng, cells, cellSites, capacity, stats );
}

int lpFracture( const lpFractureInput* input, lpShape** cells, int* cellSites, int capacity, lpFractureStats* stats )
{
	int count = lpFracturePattern( input, cells, cellSites, capacity, stats );
	if ( stats != NULL )
	{
		stats->patternCells += count;
	}
	return count;
}

int lpFindCellBonds( lpShape* const* cells, const int* cellSites, int count, lpCellBond* bonds, int capacity,
					 lpFractureStats* stats )
{
	int siteToCell[LP_MAX_SITES];
	for ( int i = 0; i < LP_MAX_SITES; ++i )
	{
		siteToCell[i] = -1;
	}
	for ( int i = 0; i < count; ++i )
	{
		if ( cellSites != NULL && cellSites[i] >= 0 && cellSites[i] < LP_MAX_SITES )
		{
			siteToCell[cellSites[i]] = i;
		}
	}

	int n = 0;
	int byContact = 0;
	for ( int a = 0; a < count; ++a )
	{
		bool voronoi = cellSites != NULL && cellSites[a] >= 0;
		if ( voronoi )
		{
			const lpShape* shape = cells[a];
			for ( int f = 0; f < shape->faceCount && n < capacity; ++f )
			{
				int tag = shape->faces[f].tag;
				if ( tag < 0 || tag >= LP_MAX_SITES )
				{
					continue;
				}
				int b = siteToCell[tag];
				if ( b <= a )
				{
					continue;
				}
				lpCellBond bond = { a, b };
				lpShape_FaceContact( shape, f, &bond.contact );
				if ( bond.contact.area > 1e-4f )
				{
					bonds[n++] = bond;
				}
			}
		}
		else
		{
			for ( int b = a + 1; b < count && n < capacity; ++b )
			{
				if ( lpBoxesTouch( cells[a]->bounds, cells[b]->bounds, 2e-3f ) == false )
				{
					continue;
				}
				lpCellBond bond = { a, b };
				if ( lpShape_Contact( cells[a], cells[b], 2e-3f, &bond.contact ) && bond.contact.area > 1e-4f )
				{
					bonds[n++] = bond;
					byContact += 1;
				}
			}
		}
	}

	// A merged cell's hull can bulge past the plane it shared with a neighbour, so only the neighbour still has the
	// face. Faces of the higher cell add the bonds the lower one had no face for.
	int direct = n;
	for ( int a = 0; a < count && cellSites != NULL; ++a )
	{
		if ( cellSites[a] < 0 )
		{
			continue;
		}
		const lpShape* shape = cells[a];
		for ( int f = 0; f < shape->faceCount && n < capacity; ++f )
		{
			int tag = shape->faces[f].tag;
			if ( tag < 0 || tag >= LP_MAX_SITES )
			{
				continue;
			}
			int b = siteToCell[tag];
			if ( b < 0 || b >= a )
			{
				continue;
			}
			bool known = false;
			for ( int k = 0; k < direct && known == false; ++k )
			{
				known = bonds[k].a == b && bonds[k].b == a;
			}
			if ( known )
			{
				continue;
			}
			lpCellBond bond = { b, a };
			lpShape_FaceContact( shape, f, &bond.contact );
			bond.contact.normal = lpNeg( bond.contact.normal ); // the face is a's; the bond runs from b to a
			if ( bond.contact.area > 1e-4f )
			{
				bonds[n++] = bond;
			}
		}
	}
	if ( stats != NULL )
	{
		stats->bondsByContact += byContact;
		stats->bondsByTag += n - byContact;
	}
	return n;
}

// Volume of conv(a + p) for the vertex p of b that adds the most: a's volume plus the pyramids from p over the faces
// of a that p can see, which is exact for adding one point to a convex solid. The hull of a and b contains every
// such solid, so this is a true lower bound on the hull's volume, found without building it.
static float lpHullVolumeLowerBound( const lpShape* a, const lpShape* b )
{
	float area[LP_POLY_MAX_FACES];
	for ( int f = 0; f < a->faceCount; ++f )
	{
		lpVec3 centroid;
		area[f] = lpShape_FaceArea( a, f, &centroid );
	}
	float added = 0.0f;
	for ( int k = 0; k < b->vertexCount; ++k )
	{
		float sum = 0.0f;
		for ( int f = 0; f < a->faceCount; ++f )
		{
			float d = lpDot( a->faces[f].plane.normal, b->vertices[k] ) - a->faces[f].plane.offset;
			sum += d > 0.0f ? area[f] * d : 0.0f;
		}
		added = sum > added ? sum : added;
	}
	return a->volume + added / 3.0f;
}

// Hull of two cells into `poly`, retagged from their faces. False if it is too much bigger than the cells were, or
// if it would fill in the space of a cell that leaves (a notch knocked out of a log must stay a notch).
// The faces of an accepted hull that took a source face's tag, and those that found none (counted when it is accepted)
typedef struct lpMergeTags
{
	int retagged;
	int bridges;
} lpMergeTags;

static bool lpMergePair( const lpShape* a, const lpShape* b, int siteA, int siteB, float allowedVolume,
						 const lpVec3* keepOut, int keepOutCount, uint8_t interiorMaterial, lpPoly* poly, lpVec3* points,
						 lpFractureStats* stats, lpMergeTags* tags )
{
	int n = a->vertexCount + b->vertexCount;
	if ( n > LP_POLY_MAX_VERTICES )
	{
		if ( stats != NULL )
		{
			stats->mergeTooBig += 1;
		}
		return false;
	}
	// Cheap reject before paying for a quickhull: most pairs fail the volume test, and a lower bound on the hull's
	// volume already proves it for most of them. The margin keeps rounding from ever rejecting a pair the exact test
	// would accept, so merge results are the same as without this check.
	float margin = 1.0f + 1e-3f;
	if ( lpHullVolumeLowerBound( a, b ) > margin * allowedVolume || lpHullVolumeLowerBound( b, a ) > margin * allowedVolume )
	{
		if ( stats != NULL )
		{
			stats->mergePrerejected += 1;
		}
		return false;
	}
	memcpy( points, a->vertices, sizeof( lpVec3 ) * (size_t)a->vertexCount );
	memcpy( points + a->vertexCount, b->vertices, sizeof( lpVec3 ) * (size_t)b->vertexCount );
	if ( stats != NULL )
	{
		stats->mergeHulls += 1;
	}
	if ( lpPoly_MakeFromPoints( poly, points, n, interiorMaterial ) == false )
	{
		return false;
	}
	float volume;
	lpVec3 centroid;
	lpPoly_ComputeMass( poly, &volume, &centroid );
	if ( ( volume <= allowedVolume ) == false )
	{
		return false;
	}
	for ( int k = 0; k < keepOutCount; ++k )
	{
		if ( lpPoly_SignedDistance( poly, keepOut[k] ) < 0.0f )
		{
			return false;
		}
	}

	// Authored surface stays authored and a face against a third cell still names it. Faces across the filled-in
	// gap are cut faces.
	const lpShape* sources[2] = { a, b };
	tags->retagged = 0;
	tags->bridges = 0;
	for ( int f = 0; f < poly->faceCount; ++f )
	{
		lpFace* face = poly->faces + f;
		face->tag = LP_TAG_CUT;
		face->material = interiorMaterial;
		bool found = false;
		for ( int s = 0; s < 2 && found == false; ++s )
		{
			const lpShape* src = sources[s];
			for ( int g = 0; g < src->faceCount && found == false; ++g )
			{
				const lpFace* other = src->faces + g;
				if ( other->tag == siteA || other->tag == siteB )
				{
					continue;
				}
				if ( lpDot( face->plane.normal, other->plane.normal ) > 0.9999f &&
					 lpAbsFloat( face->plane.offset - other->plane.offset ) < 2e-3f )
				{
					face->tag = other->tag;
					face->material = other->material;
					found = true;
				}
			}
		}
		tags->retagged += found ? 1 : 0;
		tags->bridges += found ? 0 : 1;
	}
	return true;
}

int lpMergeCells( lpShape** cells, int* cellSites, uint8_t* classes, int count, uint8_t mergeClass, float slack,
				  uint8_t interiorMaterial, lpVec3 impact, lpFractureStats* stats )
{
	if ( ( slack > 0.0f ) == false || cellSites == NULL || count < 2 || count > LP_MAX_SITES )
	{
		return count;
	}

	int siteToCell[LP_MAX_SITES];
	float trueVolume[LP_MAX_SITES];
	lpVec3 keepOut[LP_MAX_SITES + 1]; // centroids of the cells that do not merge, and the impact point
	int keepOutCount = 0;
	keepOut[keepOutCount++] = impact;
	for ( int i = 0; i < LP_MAX_SITES; ++i )
	{
		siteToCell[i] = -1;
	}
	for ( int i = 0; i < count; ++i )
	{
		if ( cellSites[i] >= 0 && cellSites[i] < LP_MAX_SITES )
		{
			siteToCell[cellSites[i]] = i;
		}
		trueVolume[i] = cells[i]->volume;
		if ( classes[i] != mergeClass )
		{
			keepOut[keepOutCount++] = cells[i]->centroid;
		}
	}

	lpPoly* poly = lpAlloc( sizeof( lpPoly ) );
	lpVec3 points[LP_POLY_MAX_VERTICES];
	int tried[LP_MAX_SITES]; // the version of `a` each neighbour was last tried against
	int version = 0;
	for ( int i = 0; i < LP_MAX_SITES; ++i )
	{
		tried[i] = -1;
	}
	int merged = 0;
	for ( int a = 0; a < count; ++a )
	{
		if ( cells[a] == NULL || classes[a] != mergeClass || cellSites[a] < 0 )
		{
			continue;
		}
		// Grow `a` by its neighbours until none fits; the slack is against the true volume, so it never compounds
		bool grew = true;
		while ( grew )
		{
			grew = false;
			version += 1; // a new shape for `a`: every neighbour is worth one try again
			const lpShape* shape = cells[a];
			for ( int f = 0; grew == false && f < shape->faceCount; ++f )
			{
				int tag = shape->faces[f].tag;
				int b = tag >= 0 && tag < LP_MAX_SITES ? siteToCell[tag] : -1;
				if ( b < 0 || b == a || cells[b] == NULL || classes[b] != mergeClass )
				{
					continue;
				}
				float allowed = ( 1.0f + slack ) * ( trueVolume[a] + trueVolume[b] );
				if ( tried[b] == version )
				{
					continue; // another face of a names the same neighbour: same inputs, same answer
				}
				tried[b] = version;
				if ( stats != NULL )
				{
					stats->mergeTried += 1;
				}
				lpMergeTags tags;
				if ( lpMergePair( cells[a], cells[b], cellSites[a], cellSites[b], allowed, keepOut, keepOutCount,
								  interiorMaterial, poly, points, stats, &tags ) == false )
				{
					continue;
				}
				lpShape* joined = lpShape_Create( poly );
				if ( joined == NULL )
				{
					continue;
				}
				if ( stats != NULL )
				{
					stats->mergeAccepted += 1;
					stats->mergeRetagged += tags.retagged;
					stats->mergeBridges += tags.bridges;
				}
				int siteA = cellSites[a];
				int siteB = cellSites[b];
				lpShape_Destroy( cells[a] );
				lpShape_Destroy( cells[b] );
				cells[a] = joined;
				cells[b] = NULL;
				trueVolume[a] += trueVolume[b];
				siteToCell[siteB] = a;
				for ( int c = 0; c < count; ++c )
				{
					for ( int g = 0; cells[c] != NULL && g < cells[c]->faceCount; ++g )
					{
						if ( cells[c]->faces[g].tag == siteB )
						{
							cells[c]->faces[g].tag = siteA;
						}
					}
				}
				merged += 1;
				grew = true;
			}
		}
	}
	lpFree( poly );

	if ( merged == 0 )
	{
		return count;
	}
	int kept = 0;
	for ( int i = 0; i < count; ++i )
	{
		if ( cells[i] != NULL )
		{
			cells[kept] = cells[i];
			cellSites[kept] = cellSites[i];
			classes[kept] = classes[i];
			kept += 1;
		}
	}
	return kept;
}

int lpChipCell( const lpShape* cell, int splits, lpVec3 grainAxis, uint8_t material, float minVolume, lpRandom* rng,
				lpShape** chips, int capacity, lpFractureStats* stats )
{
	lpClipStats* clipStats = stats != NULL ? stats->clips + lp_clipChips : NULL;
	enum
	{
		lp_maxChips = 4
	};
	splits = splits < lp_maxChips - 1 ? splits : lp_maxChips - 1;
	if ( splits <= 0 || capacity < 2 || cell->volume < 2.0f * minVolume )
	{
		return 0;
	}

	lpPoly* polys = lpAlloc( ( lp_maxChips + 1 ) * sizeof( lpPoly ) );
	lpPoly* scratch = polys + lp_maxChips;
	float volumes[lp_maxChips];
	lpVec3 centroids[lp_maxChips];
	lpShape_ToPoly( cell, polys );
	volumes[0] = cell->volume;
	centroids[0] = cell->centroid;
	int count = 1;
	bool grain = lpLengthSquared( grainAxis ) > 0.0f;

	for ( int s = 0; s < splits && count < capacity && count < lp_maxChips; ++s )
	{
		// Always split the biggest chip so far (lowest index on ties)
		int big = 0;
		for ( int i = 1; i < count; ++i )
		{
			big = volumes[i] > volumes[big] ? i : big;
		}
		if ( volumes[big] < 2.0f * minVolume )
		{
			break;
		}

		lpVec3 n = lpRandomUnitVector( rng );
		if ( grain )
		{
			n = lpMulSub( n, lpDot( n, grainAxis ), grainAxis );
			if ( lpLengthSquared( n ) < 1e-4f )
			{
				continue;
			}
			n = lpNormalize( n );
		}
		float reach = 0.15f * cell->radius; // no cbrtf: C-library roots are not bit-identical everywhere
		lpPlane plane = { n, lpDot( n, centroids[big] ) + lpRandom_Range( rng, -reach, reach ) };
		lpPlane flipped = { lpNeg( n ), -plane.offset };

		lpClipResult below = lpPoly_ClipCounted( polys + big, plane, material, LP_TAG_CUT, 1e-5f, scratch, clipStats );
		if ( below != lp_clipCut )
		{
			continue;
		}
		float va, vb;
		lpVec3 ca, cb;
		lpPoly_ComputeMass( scratch, &va, &ca );
		lpPoly* other = polys + count;
		if ( lpPoly_ClipCounted( polys + big, flipped, material, LP_TAG_CUT, 1e-5f, other, clipStats ) != lp_clipCut )
		{
			continue;
		}
		lpPoly_ComputeMass( other, &vb, &cb );
		if ( va < minVolume || vb < minVolume )
		{
			continue;
		}
		polys[big] = *scratch;
		volumes[big] = va;
		centroids[big] = ca;
		volumes[count] = vb;
		centroids[count] = cb;
		count += 1;
	}

	int written = 0;
	if ( count > 1 )
	{
		for ( int i = 0; i < count; ++i )
		{
			lpShape* chip = lpShape_Create( polys + i );
			if ( chip != NULL )
			{
				chips[written++] = chip;
			}
		}
	}
	lpFree( polys );
	return written;
}

// ---- the fracture job (impact.c prepares it and integrates its output) ----

// Phase 2. Must not touch the world. Cells inside the break radius are ejecta: their bonds would break anyway, so
// they skip bonding and connectivity and go straight to their tier. Puffs and ghosts need no physics hull at all.
uint8_t lpFracture_ClassifyCell( const lpFractureJob* job, const lpShape* cell )
{
	float r2 = job->input.radius * job->input.radius;
	float volume = cell->volume;
	bool ejecta = lpDistanceSquared( cell->centroid, job->localImpact ) < r2;
	// Flying ejecta are real geometry down to the tiny particle volume; a sliver left on the piece turns to dust
	return ejecta ? lpLooseClass( volume, job->particleVolume, job->ghostVolume, job->lightVolume )
				  : ( volume < job->input.absorbVolume ? lp_cellPuff : lp_cellKeep );
}

int lpFracture_ChipGhost( const lpFractureJob* job, int cell, int cellCount, lpShape** chips, lpFractureStats* stats )
{
	bool oriented = job->input.pattern == lp_breakGrain || job->input.pattern == lp_breakRadial;
	lpRandom rng;
	lpRandom_Seed( &rng, job->input.seed, 0xC41Full + (uint64_t)cell );
	int room = LP_MAX_SITES - cellCount + 1;
	return lpChipCell( job->cells[cell], job->chipSplits, oriented ? job->input.axis : lpVec3_zero, job->input.interiorMaterial,
					   job->particleVolume, &rng, chips, room < 4 ? room : 4, stats );
}

void lpFracture_RunJob( lpFractureJob* job )
{
	job->input.parent = &job->poly; // the job array may have moved since the job was prepared
	memset( &job->stats, 0, sizeof( job->stats ) );
	uint64_t ticks = lpGetTicks();
	job->cellCount = lpFracture( &job->input, job->cells, job->cellSites, LP_MAX_SITES, &job->stats );
	job->stats.voronoiMs = lpGetMillisecondsAndReset( &ticks );
	job->bondCount = 0;
	for ( int i = 0; i < job->cellCount; ++i )
	{
		job->hulls[i] = NULL; // the job slot is reused: never leave a stale hull for lpFracture_FreeJob
	}
	if ( job->cellCount < 2 )
	{
		return;
	}
	for ( int i = 0; i < job->cellCount; ++i )
	{
		lpShape_Translate( job->cells[i], job->center );
		job->cellClass[i] = lpFracture_ClassifyCell( job, job->cells[i] );
	}

	// Cells that stay on the piece merge where their union is nearly convex: a log end becomes one piece
	job->cellCount = lpMergeCells( job->cells, job->cellSites, job->cellClass, job->cellCount, lp_cellKeep, job->mergeSlack,
								   job->input.interiorMaterial, job->localImpact, &job->stats );
	job->stats.mergeMs = lpGetMillisecondsAndReset( &ticks );

	// What is still too small to carry load does not stay on the piece: it falls as debris. Structures keep chunks,
	// not crumbs, which is cheaper for physics and keeps the stress solve well conditioned (no tiny bonds). A crumb is
	// small in every direction: half a snapped plank is thin but long, and stays.
	float crumbReach = 4.0f * job->input.fragmentSize;
	for ( int i = 0; i < job->cellCount; ++i )
	{
		float volume = job->cells[i]->volume;
		if ( job->cellClass[i] == lp_cellKeep && volume < job->lightVolume && job->cells[i]->radius < crumbReach )
		{
			job->cellClass[i] = volume < job->ghostVolume ? lp_cellGhost : lp_cellLight;
		}
	}

	for ( int i = 0; i < job->cellCount; ++i )
	{
		uint8_t cls = job->cellClass[i];
		bool needsHull = cls == lp_cellKeep || cls == lp_cellLight || cls == lp_cellFull;
		job->hulls[i] = needsHull ? lpShape_CreateHull( job->cells[i] ) : NULL;
		if ( needsHull )
		{
			const lpShape* cell = job->cells[i];
			bool large = cell->vertexCount + cell->faceCount - 2 > 128; // edges, against Box3D's B3_MAX_HULL_EDGES
			job->stats.hullsBuilt += job->hulls[i] != NULL ? 1 : 0;
			job->stats.hullFailsLarge += job->hulls[i] == NULL && large ? 1 : 0;
			job->stats.hullFailsOther += job->hulls[i] == NULL && large == false ? 1 : 0;
		}
	}
	job->stats.hullMs = lpGetMillisecondsAndReset( &ticks );
	job->bondCount = lpFindCellBonds( job->cells, job->cellSites, job->cellCount, job->bonds, LP_MAX_CELL_BONDS, &job->stats );
	job->stats.bondMs = lpGetMillisecondsAndReset( &ticks );

	// Ghost ejecta break into a few real chips: a dirtier spray for a few plane clips. After the bonds, which only
	// keepers use, so the chips need none. Wood splits along the grain, glass across the pane.
	int original = job->cellCount;
	for ( int i = 0; i < original && job->chipSplits > 0; ++i )
	{
		if ( job->cellClass[i] != lp_cellGhost )
		{
			continue;
		}
		lpShape* chips[4];
		int count = lpFracture_ChipGhost( job, i, job->cellCount, chips, &job->stats );
		if ( count == 0 )
		{
			continue;
		}
		job->stats.ghostsChipped += 1;
		job->stats.chipsMade += count;
		lpShape_Destroy( job->cells[i] );
		job->cells[i] = chips[0];
		for ( int k = 1; k < count; ++k )
		{
			int c = job->cellCount++;
			job->cells[c] = chips[k];
			job->cellSites[c] = -1;
			job->cellClass[c] = lp_cellGhost;
			job->hulls[c] = NULL;
		}
	}
	job->stats.chipMs = lpGetMillisecondsAndReset( &ticks );

	// The output cells' sizes, for the bench's histograms
	job->stats.outputCells = job->cellCount;
	for ( int i = 0; i < job->cellCount; ++i )
	{
		const lpShape* cell = job->cells[i];
		job->stats.maxFaces = cell->faceCount > job->stats.maxFaces ? cell->faceCount : job->stats.maxFaces;
		job->stats.maxVertices = cell->vertexCount > job->stats.maxVertices ? cell->vertexCount : job->stats.maxVertices;
		job->stats.faceBins[lpCellBin( true, cell->faceCount )] += 1;
		job->stats.vertexBins[lpCellBin( false, cell->vertexCount )] += 1;
	}
}

void lpFracture_FreeJob( lpFractureJob* job )
{
	for ( int i = 0; i < job->cellCount; ++i )
	{
		if ( job->cells[i] != NULL )
		{
			lpShape_Destroy( job->cells[i] );
		}
		if ( job->hulls[i] != NULL )
		{
			lpPhys_DestroyHull( job->hulls[i] );
		}
	}
	job->cellCount = 0;
}
