// SPDX-License-Identifier: MIT

#include "fracture.h"

#include <float.h>

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

bool lpComputeVoronoiCell( const lpPoly* parent, const b3Vec3* sites, int siteCount, int index, uint8_t material,
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
		if ( stats != NULL )
		{
			stats->clipCount += 1;
		}

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

int lpGenerateImpactSites( const lpPoly* parent, const lpSiteParams* params, lpRandom* rng, b3Vec3* sites )
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

	// Damaged volume: the cube around the focus clipped to the bounds, scaled to a sphere.
	float ox = lpOverlap1( bounds.lowerBound.x, bounds.upperBound.x, focus.x - radius, focus.x + radius );
	float oy = lpOverlap1( bounds.lowerBound.y, bounds.upperBound.y, focus.y - radius, focus.y + radius );
	float oz = lpOverlap1( bounds.lowerBound.z, bounds.upperBound.z, focus.z - radius, focus.z + radius );
	float damagedVolume = 0.5236f * ox * oy * oz;
	float cellVolume = fragmentSize * fragmentSize * fragmentSize;

	// A few ring and plate sites are always reserved; the inner (ejecta) sites get the rest of the budget.
	int innerTarget = (int)( damagedVolume / cellVolume );
	innerTarget = innerTarget < 2 ? 2 : innerTarget;
	innerTarget = innerTarget > maxSites - 9 ? maxSites - 9 : innerTarget;
	innerTarget = innerTarget < 2 ? 2 : innerTarget;

	int count = 0;
	b3AABB innerBox = {
		b3Max( bounds.lowerBound, b3Sub( focus, (b3Vec3){ radius, radius, radius } ) ),
		b3Min( bounds.upperBound, b3Add( focus, (b3Vec3){ radius, radius, radius } ) ),
	};

	int attempts = 30 * innerTarget + 64;
	for ( int a = 0; a < attempts && count < innerTarget; ++a )
	{
		b3Vec3 p = lpRandomInBox( rng, innerBox );
		float d = b3Distance( p, focus );
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
	int ringTarget = count / 3;
	ringTarget = ringTarget < 4 ? 4 : ( ringTarget > 6 ? 6 : ringTarget );
	float ringSpacing = b3MaxFloat( spacing, 0.6f * radius );
	int ringEnd = count + ringTarget < maxSites ? count + ringTarget : maxSites;
	bool avoidAxis = b3LengthSquared( params->avoidAxis ) > 0.0f;
	for ( int a = 0; a < 12 * ringTarget && count < ringEnd; ++a )
	{
		b3Vec3 dir = lpRandomUnitVector( rng );
		if ( avoidAxis && b3AbsFloat( b3Dot( dir, params->avoidAxis ) ) > 0.906f ) // within 25 degrees
		{
			continue;
		}
		float r = radius * lpRandom_Range( rng, 0.95f, 1.5f );
		b3Vec3 p = b3MulAdd( focus, r, dir );
		if ( lpPoly_SignedDistance( parent, p ) > -margin || lpIsTooClose( p, sites, count, ringSpacing * ringSpacing ) )
		{
			continue;
		}
		sites[count++] = p;
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
		if ( b3Distance( p, focus ) < 1.4f * radius )
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

	bool grain = input->pattern == lp_patternGrain && input->stretch > 1.0f;
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
	params.avoidAxis = grain ? input->axis : b3Vec3_zero;
	int siteCount = lpGenerateImpactSites( parent, &params, rng, sites );
	if ( stats != NULL )
	{
		stats->siteCount += siteCount;
	}

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
				if ( stats != NULL )
				{
					stats->clipCount += 1;
				}
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

	if ( stats != NULL )
	{
		stats->siteCount += wedgeCount * ( ringCount + 1 );
	}

	lpFree( work );
	return count;
}

int lpFracture( const lpFractureInput* input, lpShape** cells, int* cellSites, int capacity, lpFractureStats* stats )
{
	lpRandom rng;
	lpRandom_Seed( &rng, input->seed, 0x5EEDu );

	if ( input->pattern == lp_patternRadial )
	{
		return lpFractureRadial( input, &rng, cells, cellSites, capacity, stats );
	}
	return lpFractureVoronoi( input, &rng, cells, cellSites, capacity, stats );
}

static bool lpBoxesTouch( b3AABB a, b3AABB b, float margin )
{
	return !( a.lowerBound.x > b.upperBound.x + margin || b.lowerBound.x > a.upperBound.x + margin ||
			  a.lowerBound.y > b.upperBound.y + margin || b.lowerBound.y > a.upperBound.y + margin ||
			  a.lowerBound.z > b.upperBound.z + margin || b.lowerBound.z > a.upperBound.z + margin );
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
				b3Vec3 centroid;
				float area = lpShape_FaceArea( shape, f, &centroid );
				if ( area > 1e-4f )
				{
					bonds[n++] = (lpCellBond){ a, b, area, centroid };
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
				b3Vec3 centroid, normal;
				float area = lpShape_ContactArea( cells[a], cells[b], 2e-3f, &centroid, &normal );
				if ( area > 1e-4f )
				{
					bonds[n++] = (lpCellBond){ a, b, area, centroid };
				}
			}
		}
	}
	return n;
}
