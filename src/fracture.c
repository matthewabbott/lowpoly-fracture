// SPDX-License-Identifier: MIT

#include "fracture.h"

#include <float.h>

// Sites inside the parent: dense near the impact (density ~ 1/distance, the ejecta), 4-6 ring sites around the
// damage radius that shape a jagged rim, and at most three far sites that keep the rest of the piece in large plates.
typedef struct lpSiteParams
{
	b3Vec3 impact;
	float radius;
	float fragmentSize;
	float plateSize;
	int maxSites;
	b3Vec3 grainAxis; // ring sites stay out of a 25 degree cone around this axis; zero for none
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
static bool lpComputeVoronoiCell( const lpPoly* parent, const b3Vec3* sites, int siteCount, int index, uint8_t material,
						   float tolerance, lpPoly* scratch, lpPoly* out, lpFractureStats* stats )
{
	b3Vec3 site = sites[index];

	uint64_t keys[LP_MAX_SITES];
	int keyCount = 0;
	for ( int j = 0; j < siteCount && keyCount < LP_MAX_SITES; ++j )
	{
		if ( j == index )
		{
			continue;
		}
		// Non-negative floats order like their bit patterns
		float d2 = b3DistanceSquared( site, sites[j] );
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
		b3Vec3 other = sites[j];
		b3Vec3 normal = b3Normalize( b3Sub( other, site ) );
		b3Vec3 mid = b3MulSV( 0.5f, b3Add( site, other ) );
		b3Plane plane = { normal, b3Dot( normal, mid ) };

		lpClipResult result = lpPoly_Clip( current, plane, material, j, tolerance, next );

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

static b3Vec3 lpRandomInBox( lpRandom* rng, b3AABB box )
{
	return (b3Vec3){
		lpRandom_Range( rng, box.lowerBound.x, box.upperBound.x ),
		lpRandom_Range( rng, box.lowerBound.y, box.upperBound.y ),
		lpRandom_Range( rng, box.lowerBound.z, box.upperBound.z ),
	};
}

// Uniform unit vector by rejection sampling: no trigonometry, so bit-identical everywhere.
static b3Vec3 lpRandomUnitVector( lpRandom* rng )
{
	for ( int i = 0; i < 64; ++i )
	{
		b3Vec3 p = { lpRandom_Range( rng, -1.0f, 1.0f ), lpRandom_Range( rng, -1.0f, 1.0f ),
					 lpRandom_Range( rng, -1.0f, 1.0f ) };
		float l2 = b3LengthSquared( p );
		if ( l2 > 0.01f && l2 <= 1.0f )
		{
			return b3MulSV( 1.0f / sqrtf( l2 ), p );
		}
	}
	return (b3Vec3){ 0.0f, 1.0f, 0.0f };
}

static bool lpIsTooClose( b3Vec3 p, const b3Vec3* sites, int count, float spacingSquared )
{
	for ( int i = 0; i < count; ++i )
	{
		if ( b3DistanceSquared( p, sites[i] ) < spacingSquared )
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
static float lpSiteDistance( const lpSiteParams* params, b3Vec3 p, b3Vec3 focus )
{
	b3Vec3 v = b3Sub( p, focus );
	if ( params->stretch > 1.0f )
	{
		v = b3MulAdd( v, ( params->stretch - 1.0f ) * b3Dot( v, params->grainAxis ), params->grainAxis );
	}
	return b3Length( v );
}

static int lpGenerateImpactSites( const lpPoly* parent, const lpSiteParams* params, lpRandom* rng, b3Vec3* sites )
{
	int maxSites = params->maxSites < LP_MAX_SITES ? params->maxSites : LP_MAX_SITES;
	float radius = params->radius;
	float fragmentSize = params->fragmentSize;
	b3AABB bounds = lpPoly_ComputeBounds( parent );
	b3Vec3 focus = b3Clamp( params->impact, bounds.lowerBound, bounds.upperBound );
	b3Vec3 extent = b3Sub( bounds.upperBound, bounds.lowerBound );

	// Sites too close to the surface make thin slivers against it; keep them in by a margin, but never so much that
	// a thin plank or pane has no room left.
	float spacing = 0.7f * fragmentSize;
	float thinnest = 0.5f * b3MinFloat( extent.x, b3MinFloat( extent.y, extent.z ) );
	float margin = b3MinFloat( 0.5f * spacing, 0.45f * thinnest );
	float minRadius = b3MaxFloat( 0.15f * radius, spacing );

	// Box around the damage sphere: an ellipsoid with semi-axis radius / stretch along the grain when squashed
	b3Vec3 reach = { radius, radius, radius };
	if ( params->stretch > 1.0f )
	{
		b3Vec3 a = params->grainAxis;
		float f = 1.0f - 1.0f / ( params->stretch * params->stretch );
		reach = (b3Vec3){ radius * sqrtf( 1.0f - f * a.x * a.x ), radius * sqrtf( 1.0f - f * a.y * a.y ),
						  radius * sqrtf( 1.0f - f * a.z * a.z ) };
	}

	// Damaged volume: that box clipped to the bounds, scaled to an ellipsoid.
	float ox = lpOverlap1( bounds.lowerBound.x, bounds.upperBound.x, focus.x - reach.x, focus.x + reach.x );
	float oy = lpOverlap1( bounds.lowerBound.y, bounds.upperBound.y, focus.y - reach.y, focus.y + reach.y );
	float oz = lpOverlap1( bounds.lowerBound.z, bounds.upperBound.z, focus.z - reach.z, focus.z + reach.z );
	float damagedVolume = 0.5236f * ox * oy * oz;
	float cellVolume = fragmentSize * fragmentSize * fragmentSize;

	// A few ring and plate sites are always reserved; the inner (ejecta) sites get the rest of the budget.
	int innerTarget = (int)( damagedVolume / cellVolume );
	innerTarget = innerTarget < 2 ? 2 : innerTarget;
	innerTarget = innerTarget > maxSites - 9 ? maxSites - 9 : innerTarget;
	innerTarget = innerTarget < 2 ? 2 : innerTarget;

	int count = 0;
	b3AABB innerBox = {
		b3Max( bounds.lowerBound, b3Sub( focus, reach ) ),
		b3Min( bounds.upperBound, b3Add( focus, reach ) ),
	};

	int attempts = 30 * innerTarget + 64;
	for ( int a = 0; a < attempts && count < innerTarget; ++a )
	{
		b3Vec3 p = lpRandomInBox( rng, innerBox );
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
	float ringSpacing = b3MaxFloat( spacing, 0.6f * radius );
	int ringEnd = count + ringTarget < maxSites ? count + ringTarget : maxSites;
	bool avoidAxis = b3LengthSquared( params->grainAxis ) > 0.0f;
	for ( int a = 0; a < 12 * ringTarget && count < ringEnd; ++a )
	{
		b3Vec3 dir = lpRandomUnitVector( rng );
		if ( avoidAxis && b3AbsFloat( b3Dot( dir, params->grainAxis ) ) > 0.906f ) // within 25 degrees
		{
			continue;
		}
		if ( params->stretch > 1.0f )
		{
			dir = b3MulSub( dir, ( 1.0f - 1.0f / params->stretch ) * b3Dot( dir, params->grainAxis ), params->grainAxis );
		}
		float r = radius * lpRandom_Range( rng, 0.95f, 1.5f );
		b3Vec3 p = b3MulAdd( focus, r, dir );
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
		b3Vec3 a = params->grainAxis;
		b3Vec3 center = b3Lerp( bounds.lowerBound, bounds.upperBound, 0.5f );
		float along = b3Dot( b3Sub( focus, center ), a );
		for ( int side = -1; side <= 1 && count < maxSites; side += 2 )
		{
			float t = along + (float)side * lpRandom_Range( rng, 1.15f, 1.4f ) * radius / params->stretch;
			b3Vec3 p = b3MulAdd( center, t, a );
			if ( lpPoly_SignedDistance( parent, p ) > -margin || lpIsTooClose( p, sites, count, spacing * spacing ) )
			{
				continue;
			}
			sites[count++] = p;
		}
	}

	// At most three far sites: the rest of the piece stays in a few large plates (a log keeps two whole ends)
	float totalVolume = extent.x * extent.y * extent.z;
	float plate = b3MaxFloat( b3MaxFloat( 3.0f * fragmentSize, 1.2f * radius ), params->plateSize );
	int farTarget = (int)( ( totalVolume - damagedVolume ) / ( plate * plate * plate ) );
	farTarget = farTarget < 0 ? 0 : ( farTarget > 3 ? 3 : farTarget );
	int farEnd = count + farTarget < maxSites ? count + farTarget : maxSites;
	float farSpacing = 0.8f * plate;
	for ( int a = 0; a < 16 * farTarget && count < farEnd; ++a )
	{
		b3Vec3 p = lpRandomInBox( rng, bounds );
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
	b3Vec3 impact = input->impact;
	b3Matrix3 unsquash = { 0 };

	bool grain = input->pattern == lp_breakGrain && input->stretch > 1.0f;
	if ( grain )
	{
		// Squash along the grain: isotropic Voronoi there becomes cells `stretch` times longer after unsquashing.
		b3Vec3 a = input->axis;
		float s = 1.0f / input->stretch;
		float k = s - 1.0f;
		b3Matrix3 squash = {
			{ 1.0f + k * a.x * a.x, k * a.y * a.x, k * a.z * a.x },
			{ k * a.x * a.y, 1.0f + k * a.y * a.y, k * a.z * a.y },
			{ k * a.x * a.z, k * a.y * a.z, 1.0f + k * a.z * a.z },
		};
		float g = input->stretch - 1.0f;
		unsquash = (b3Matrix3){
			{ 1.0f + g * a.x * a.x, g * a.y * a.x, g * a.z * a.x },
			{ g * a.x * a.y, 1.0f + g * a.y * a.y, g * a.z * a.y },
			{ g * a.x * a.z, g * a.y * a.z, 1.0f + g * a.z * a.z },
		};
		lpPoly_ApplyLinear( parent, squash );
		impact = b3MulMV( squash, impact );
	}

	b3Vec3 sites[LP_MAX_SITES];
	lpSiteParams params = { 0 };
	params.impact = impact;
	params.radius = input->radius;
	params.fragmentSize = input->fragmentSize;
	params.plateSize = grain ? input->plateSize / input->stretch : input->plateSize;
	params.maxSites = input->maxCells < capacity ? input->maxCells : capacity;
	params.grainAxis = grain ? input->axis : b3Vec3_zero;
	params.stretch = grain ? input->stretch : 1.0f;
	// Walls get a ring of rim cells for a jagged hole; a snapped log keeps each end whole, its broken face already
	// jagged from the splinters that left it.
	params.ringSites = grain ? 0 : 5;
	int siteCount = lpGenerateImpactSites( parent, &params, rng, sites );

	// Compute all cells. Tiny cells outside the damage radius are slivers: drop their sites and recompute, which
	// hands their volume to the neighbours while keeping the tiling exact and every cell convex.
	int count = 0;
	for ( int pass = 0; pass < 3 && siteCount >= 2; ++pass )
	{
		count = 0;
		for ( int i = 0; i < siteCount && count < capacity; ++i )
		{
			if ( lpComputeVoronoiCell( parent, sites, siteCount, i, input->interiorMaterial, input->tolerance, scratch,
									   cell, stats ) == false )
			{
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
			if ( cells[c]->volume < input->absorbVolume && b3DistanceSquared( cells[c]->centroid, input->impact ) > r2 )
			{
				drop[cellSites[c]] = true;
				dropCount += 1;
			}
		}
		if ( dropCount == 0 || siteCount - dropCount < 2 )
		{
			break;
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

	lpFree( work );
	return count;
}

static int lpFractureRadial( const lpFractureInput* input, lpRandom* rng, lpShape** cells, int* cellSites, int capacity,
							 lpFractureStats* stats )
{
	lpPoly* work = lpAlloc( 2 * sizeof( lpPoly ) );
	lpPoly* a = work;
	lpPoly* b = work + 1;

	b3Vec3 n = input->axis;
	b3Vec3 t = b3AbsFloat( n.x ) < 0.57f ? (b3Vec3){ 1.0f, 0.0f, 0.0f } : (b3Vec3){ 0.0f, 1.0f, 0.0f };
	b3Vec3 u = b3Normalize( b3Cross( t, n ) );
	b3Vec3 v = b3Cross( n, u );
	b3Vec3 p = input->impact;

	// Wedge boundaries around the impact axis, jittered
	enum
	{
		lp_maxWedges = 12,
		lp_maxRings = 8
	};
	int wedgeCount = 6 + (int)( lpRandom_Next( rng ) % 5u );
	float step = 2.0f * B3_PI / (float)wedgeCount;
	b3Vec3 dirs[lp_maxWedges + 1];
	for ( int k = 0; k < wedgeCount; ++k )
	{
		float angle = step * ( (float)k + lpRandom_Range( rng, -0.3f, 0.3f ) );
		b3CosSin cs = b3ComputeCosSin( angle );
		dirs[k] = b3Add( b3MulSV( cs.cosine, u ), b3MulSV( cs.sine, v ) );
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
		b3Vec3 d0 = dirs[k];
		b3Vec3 d1 = dirs[k + 1];
		b3Vec3 perp0 = b3Cross( n, d0 );
		b3Vec3 perp1 = b3Cross( n, d1 );
		b3Vec3 mid = b3Normalize( b3Add( d0, d1 ) );
		float cosHalf = b3Dot( mid, d0 );

		b3Plane side0 = { b3Neg( perp0 ), -b3Dot( perp0, p ) };
		b3Plane side1 = { perp1, b3Dot( perp1, p ) };
		int32_t tag0 = 1000 + k;
		int32_t tag1 = 1000 + ( k + 1 ) % wedgeCount;

		for ( int j = 0; j <= ringCount && count < capacity; ++j )
		{
			*a = *input->parent;
			lpPoly* cur = a;
			lpPoly* nxt = b;
			bool empty = false;

			b3Plane planes[4];
			int32_t tags[4];
			int planeCount = 0;
			planes[planeCount] = side0;
			tags[planeCount++] = tag0;
			planes[planeCount] = side1;
			tags[planeCount++] = tag1;
			if ( j < ringCount )
			{
				planes[planeCount] = (b3Plane){ mid, b3Dot( mid, p ) + rings[j] * cosHalf };
				tags[planeCount++] = 2000 + k * 16 + j;
			}
			if ( j > 0 )
			{
				planes[planeCount] = (b3Plane){ b3Neg( mid ), -( b3Dot( mid, p ) + rings[j - 1] * cosHalf ) };
				tags[planeCount++] = 2000 + k * 16 + j - 1;
			}

			for ( int q = 0; q < planeCount; ++q )
			{
				lpClipResult result = lpPoly_Clip( cur, planes[q], input->interiorMaterial, tags[q], input->tolerance, nxt );
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
	b3Vec3 n = input->axis;
	b3Vec3 t1, t2;
	lpContactBasis( n, &t1, &t2 );
	n = b3Normalize( b3Add( n, b3Add( b3MulSV( lpRandom_Range( rng, -0.35f, 0.35f ), t1 ),
									  b3MulSV( lpRandom_Range( rng, -0.35f, 0.35f ), t2 ) ) ) );
	b3Plane plane = { n, b3Dot( n, input->impact ) };
	b3Plane flipped = { b3Neg( n ), -plane.offset };

	lpPoly* halves = lpAlloc( 2 * sizeof( lpPoly ) );
	int count = 0;
	if ( lpPoly_Clip( input->parent, plane, input->interiorMaterial, LP_TAG_CUT, input->tolerance, halves ) == lp_clipCut &&
		 lpPoly_Clip( input->parent, flipped, input->interiorMaterial, LP_TAG_CUT, input->tolerance, halves + 1 ) == lp_clipCut )
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

int lpFracture( const lpFractureInput* input, lpShape** cells, int* cellSites, int capacity, lpFractureStats* stats )
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
	return lpFractureVoronoi( input, &rng, cells, cellSites, capacity, stats );
}

int lpFindCellBonds( lpShape* const* cells, const int* cellSites, int count, lpCellBond* bonds, int capacity )
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
			bond.contact.normal = b3Neg( bond.contact.normal ); // the face is a's; the bond runs from b to a
			if ( bond.contact.area > 1e-4f )
			{
				bonds[n++] = bond;
			}
		}
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
		b3Vec3 centroid;
		area[f] = lpShape_FaceArea( a, f, &centroid );
	}
	float added = 0.0f;
	for ( int k = 0; k < b->vertexCount; ++k )
	{
		float sum = 0.0f;
		for ( int f = 0; f < a->faceCount; ++f )
		{
			float d = b3Dot( a->faces[f].plane.normal, b->vertices[k] ) - a->faces[f].plane.offset;
			sum += d > 0.0f ? area[f] * d : 0.0f;
		}
		added = sum > added ? sum : added;
	}
	return a->volume + added / 3.0f;
}

// Hull of two cells into `poly`, retagged from their faces. False if it is too much bigger than the cells were, or
// if it would fill in the space of a cell that leaves (a notch knocked out of a log must stay a notch).
static bool lpMergePair( const lpShape* a, const lpShape* b, int siteA, int siteB, float allowedVolume,
						 const b3Vec3* keepOut, int keepOutCount, uint8_t interiorMaterial, lpPoly* poly, b3Vec3* points )
{
	int n = a->vertexCount + b->vertexCount;
	if ( n > LP_POLY_MAX_VERTICES )
	{
		return false;
	}
	// Cheap reject before paying for a quickhull: most pairs fail the volume test, and a lower bound on the hull's
	// volume already proves it for most of them. The margin keeps rounding from ever rejecting a pair the exact test
	// would accept, so merge results are the same as without this check.
	float margin = 1.0f + 1e-3f;
	if ( lpHullVolumeLowerBound( a, b ) > margin * allowedVolume || lpHullVolumeLowerBound( b, a ) > margin * allowedVolume )
	{
		return false;
	}
	memcpy( points, a->vertices, sizeof( b3Vec3 ) * (size_t)a->vertexCount );
	memcpy( points + a->vertexCount, b->vertices, sizeof( b3Vec3 ) * (size_t)b->vertexCount );
	if ( lpPoly_MakeFromPoints( poly, points, n, interiorMaterial ) == false )
	{
		return false;
	}
	float volume;
	b3Vec3 centroid;
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
				if ( b3Dot( face->plane.normal, other->plane.normal ) > 0.9999f &&
					 b3AbsFloat( face->plane.offset - other->plane.offset ) < 2e-3f )
				{
					face->tag = other->tag;
					face->material = other->material;
					found = true;
				}
			}
		}
	}
	return true;
}

int lpMergeCells( lpShape** cells, int* cellSites, uint8_t* classes, int count, uint8_t mergeClass, float slack,
				  uint8_t interiorMaterial, b3Vec3 impact )
{
	if ( ( slack > 0.0f ) == false || cellSites == NULL || count < 2 || count > LP_MAX_SITES )
	{
		return count;
	}

	int siteToCell[LP_MAX_SITES];
	float trueVolume[LP_MAX_SITES];
	b3Vec3 keepOut[LP_MAX_SITES + 1]; // centroids of the cells that do not merge, and the impact point
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
	b3Vec3 points[LP_POLY_MAX_VERTICES];
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
				if ( lpMergePair( cells[a], cells[b], cellSites[a], cellSites[b], allowed, keepOut, keepOutCount,
								  interiorMaterial, poly, points ) == false )
				{
					continue;
				}
				lpShape* joined = lpShape_Create( poly );
				if ( joined == NULL )
				{
					continue;
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

int lpChipCell( const lpShape* cell, int splits, b3Vec3 grainAxis, uint8_t material, float minVolume, lpRandom* rng,
				lpShape** chips, int capacity )
{
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
	b3Vec3 centroids[lp_maxChips];
	lpShape_ToPoly( cell, polys );
	volumes[0] = cell->volume;
	centroids[0] = cell->centroid;
	int count = 1;
	bool grain = b3LengthSquared( grainAxis ) > 0.0f;

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

		b3Vec3 n = lpRandomUnitVector( rng );
		if ( grain )
		{
			n = b3MulSub( n, b3Dot( n, grainAxis ), grainAxis );
			if ( b3LengthSquared( n ) < 1e-4f )
			{
				continue;
			}
			n = b3Normalize( n );
		}
		float reach = 0.15f * cell->radius; // no cbrtf: C-library roots are not bit-identical everywhere
		b3Plane plane = { n, b3Dot( n, centroids[big] ) + lpRandom_Range( rng, -reach, reach ) };
		b3Plane flipped = { b3Neg( n ), -plane.offset };

		lpClipResult below = lpPoly_Clip( polys + big, plane, material, LP_TAG_CUT, 1e-5f, scratch );
		if ( below != lp_clipCut )
		{
			continue;
		}
		float va, vb;
		b3Vec3 ca, cb;
		lpPoly_ComputeMass( scratch, &va, &ca );
		lpPoly* other = polys + count;
		if ( lpPoly_Clip( polys + big, flipped, material, LP_TAG_CUT, 1e-5f, other ) != lp_clipCut )
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
