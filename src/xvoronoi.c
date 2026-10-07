// SPDX-License-Identifier: MIT

#include "xvoronoi.h"

#include "fcheck.h"

#include <math.h>

// Keys are unique (the site index is in the low bits), so any correct sort gives the same order (as fracture.c's)
static void lpSortKeys64( uint64_t* keys, int count )
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

// 4 R^2 for R the largest distance from the site to a vertex plus one grid unit, in grid units: a bisector of a site
// at d2 >= 4 R^2 lies beyond the cell (a vertex at most touches it), so the clip would leave the cell unchanged. The
// doubles are within 2^-50 of the exact points (2^-26 u at the grid's edge); the unit of margin covers them and every
// rounding below for any R up to 2^50.
static double lpXReach( const lpXPoly* poly, const int32_t site[3] )
{
	double m = 0.0;
	for ( int i = 0; i < poly->vertexCount; ++i )
	{
		double dx = poly->approx[i][0] - (double)site[0];
		double dy = poly->approx[i][1] - (double)site[1];
		double dz = poly->approx[i][2] - (double)site[2];
		double d = dx * dx + dy * dy + dz * dz;
		m = d > m ? d : m;
	}
	double r = sqrt( m ) + 1.0;
	return 4.0 * r * r;
}

// The exact cell of site `index`: false when it comes out empty (never for a site strictly inside the parent)
static bool lpXVoronoiCell( const lpXPoly* parent, const int32_t ( *sites )[3], int siteCount, int index, uint8_t material,
							lpXPoly* scratch, lpXPoly* out, lpXVoronoiStats* stats )
{
	const int32_t* site = sites[index];
	uint64_t keys[LP_MAX_SITES];
	int keyCount = 0;
	for ( int j = 0; j < siteCount; ++j )
	{
		if ( j == index )
		{
			continue;
		}
		// |d_i| <= 2^24 on the grid: d2 < 3 2^48, and the key below 2^57
		int64_t dx = (int64_t)sites[j][0] - site[0];
		int64_t dy = (int64_t)sites[j][1] - site[1];
		int64_t dz = (int64_t)sites[j][2] - site[2];
		uint64_t d2 = (uint64_t)( dx * dx + dy * dy + dz * dz );
		keys[keyCount++] = d2 << 7 | (uint64_t)j;
	}
	lpSortKeys64( keys, keyCount );

	lpXPoly_Copy( out, parent );
	lpXPoly* current = out;
	lpXPoly* next = scratch;
	double reach = lpXReach( current, site );
	for ( int k = 0; k < keyCount; ++k )
	{
		uint64_t d2 = keys[k] >> 7; // below 2^53: exact in double
		if ( (double)d2 >= reach )
		{
			break;
		}
		int j = (int)( keys[k] & 127u );
		lpIPlane plane;
		if ( lpIPlane_MakeBisector( site, sites[j], &plane, NULL ) == false )
		{
			stats->planeRejects += 1;
			continue;
		}
		lpClipResult result = lpXPoly_ClipCounted( current, &plane, material, j, next, &stats->clip );
		if ( result == lp_clipCut )
		{
			lpXPoly* t = current;
			current = next;
			next = t;
			reach = lpXReach( current, site );
		}
		else if ( result == lp_clipEmpty )
		{
			return false;
		}
		// overflow (counted): the plane is skipped, as the float loop skips a failed clip
	}
	if ( current != out )
	{
		lpXPoly_Copy( out, current );
	}
	return true;
}

int lpXVoronoi_Run( const lpXPoly* parent, const lpFractureInput* input, lpVec3 origin, lpXVoronoiWork* work, lpShape** cells,
					int* cellSites, int capacity, lpXVoronoiStats* stats )
{
	uint64_t ticks = stats->profile ? lpGetTicks() : 0;
	stats->jobs += 1;

	// Today's draws, to the grid: duplicates and sites not strictly inside dropped
	lpVec3 drawn[LP_MAX_SITES];
	int drawnCount = lpFracture_VoronoiSites( input, drawn );
	stats->sitesDrawn += drawnCount;
	int siteCount = 0;
	for ( int i = 0; i < drawnCount; ++i )
	{
		int32_t* p = work->sites[siteCount];
		if ( lpGeom_GridPoint( (double)drawn[i].x + (double)origin.x, (double)drawn[i].y + (double)origin.y,
							   (double)drawn[i].z + (double)origin.z, p ) == false )
		{
			stats->sitesOutside += 1;
			continue;
		}
		bool duplicate = false;
		for ( int j = 0; j < siteCount && duplicate == false; ++j )
		{
			duplicate = work->sites[j][0] == p[0] && work->sites[j][1] == p[1] && work->sites[j][2] == p[2];
		}
		if ( duplicate )
		{
			stats->siteDuplicates += 1;
			continue;
		}
		bool inside = true;
		for ( int f = 0; f < parent->faceCount && inside; ++f )
		{
			inside = lpIPlane_ClassifyPoint( &parent->faces[f].plane, p ) < 0;
		}
		if ( inside == false )
		{
			stats->sitesOutside += 1;
			continue;
		}
		siteCount += 1;
	}
	if ( stats->profile )
	{
		stats->sitesMs += lpGetMillisecondsAndReset( &ticks );
	}

	double impact[3] = { (double)input->impact.x + (double)origin.x, (double)input->impact.y + (double)origin.y,
						 (double)input->impact.z + (double)origin.z };
	double r2 = (double)input->radius * (double)input->radius;
	double volumes[LP_MAX_SITES];
	double centroids[LP_MAX_SITES][3];
	int count = 0;
	for ( int pass = 0; pass < 3 && siteCount >= 2; ++pass )
	{
		count = 0;
		for ( int i = 0; i < siteCount && count < capacity; ++i )
		{
			lpXPoly* cell = work->cells + count;
			bool made = lpXVoronoiCell( parent, (const int32_t( * )[3])work->sites, siteCount, i, input->interiorMaterial,
										&work->scratch, cell, stats );
			if ( stats->profile )
			{
				stats->clipMs += lpGetMillisecondsAndReset( &ticks );
			}
			if ( made == false )
			{
				stats->cellsEmpty += 1;
				continue;
			}
			lpXPoly_Round( cell, work->rounded );
			if ( stats->profile )
			{
				stats->roundMs += lpGetMillisecondsAndReset( &ticks );
			}
			lpXPoly_ComputeMass( cell, work->rounded, volumes + count, centroids[count] );
			lpXPoly_ToPoly( cell, work->rounded, &work->poly );
			lpShape* shape = lpShape_Create( &work->poly );
			if ( stats->profile )
			{
				stats->shapeMs += lpGetMillisecondsAndReset( &ticks );
			}
			if ( shape == NULL )
			{
				stats->cellsDropped += 1;
				continue;
			}
			cellSites[count] = i;
			cells[count++] = shape;
		}

		if ( pass == 2 || input->absorbVolume <= 0.0f )
		{
			break;
		}
		// Slivers outside the damage radius: their sites go, and the cells are made again (as the float pattern)
		bool drop[LP_MAX_SITES] = { false };
		int dropCount = 0;
		for ( int c = 0; c < count; ++c )
		{
			double dx = centroids[c][0] - impact[0];
			double dy = centroids[c][1] - impact[1];
			double dz = centroids[c][2] - impact[2];
			if ( volumes[c] < (double)input->absorbVolume && dx * dx + dy * dy + dz * dz > r2 )
			{
				drop[cellSites[c]] = true;
				dropCount += 1;
			}
		}
		if ( dropCount == 0 || siteCount - dropCount < 2 )
		{
			break;
		}
		stats->sliversAbsorbed += dropCount;
		for ( int c = 0; c < count; ++c )
		{
			lpShape_Destroy( cells[c] );
		}
		int kept = 0;
		for ( int i = 0; i < siteCount; ++i )
		{
			if ( drop[i] == false )
			{
				memmove( work->sites[kept], work->sites[i], sizeof( work->sites[i] ) );
				kept += 1;
			}
		}
		siteCount = kept;
		count = 0;
	}
	work->siteCount = siteCount;

	for ( int c = 0; c < count; ++c )
	{
		const lpXPoly* cell = work->cells + c;
		stats->maxFaces = cell->faceCount > stats->maxFaces ? cell->faceCount : stats->maxFaces;
		stats->maxVertices = cell->vertexCount > stats->maxVertices ? cell->vertexCount : stats->maxVertices;
		stats->faceBins[lpCellBin( true, cell->faceCount )] += 1;
		stats->vertexBins[lpCellBin( false, cell->vertexCount )] += 1;
	}
	stats->cells += count;
	return count;
}

void lpXVoronoi_Check( const lpXPoly* parent, const lpXVoronoiWork* work, int count, const int* cellSites,
					   lpXVoronoiCheck* check )
{
	check->jobs += 1;
	check->cells += count;
	int siteToCell[LP_MAX_SITES];
	for ( int i = 0; i < LP_MAX_SITES; ++i )
	{
		siteToCell[i] = -1;
	}
	for ( int c = 0; c < count; ++c )
	{
		siteToCell[cellSites[c]] = c;
	}

	// Validity, and the tiling from the exact points (precise doubles)
	double total = 0.0;
	double centroid[3];
	for ( int c = 0; c < count; ++c )
	{
		const char* why = lpXPoly_Validate( work->cells + c );
		if ( why != NULL )
		{
			check->invalid += 1;
			check->firstInvalid = check->firstInvalid != NULL ? check->firstInvalid : why;
		}
		double volume;
		lpXPoly_ComputeMassPrecise( work->cells + c, &volume, centroid );
		total += volume;
	}
	if ( count >= 2 )
	{
		double whole;
		lpXPoly_ComputeMassPrecise( parent, &whole, centroid );
		double error = fabs( total - whole ) / whole;
		check->tilingMaxError = error > check->tilingMaxError ? error : check->tilingMaxError;
		check->tilingViolations += error > LP_CHECK_TILING ? 1 : 0;
	}

	// Twins: every face on a bisector against its neighbour's face on the exactly negated plane, point by point
	for ( int c = 0; c < count; ++c )
	{
		const lpXPoly* cell = work->cells + c;
		for ( int f = 0; f < cell->faceCount; ++f )
		{
			const lpXFace* face = cell->faces + f;
			if ( face->tag < 0 )
			{
				continue;
			}
			check->cutFaces += 1;
			int d = face->tag < LP_MAX_SITES ? siteToCell[face->tag] : -1;
			const lpXPoly* other = d >= 0 ? work->cells + d : NULL;
			lpIPlane negated = lpIPlane_Negate( face->plane );
			int g = 0;
			while ( other != NULL && g < other->faceCount &&
					( other->faces[g].tag != cellSites[c] || lpIPlane_Equal( &other->faces[g].plane, &negated ) == false ) )
			{
				g += 1;
			}
			if ( other == NULL || g == other->faceCount || other->faces[g].count != face->count )
			{
				check->unmatched += 1;
				continue;
			}
			const lpXFace* twin = other->faces + g;
			bool all = true;
			for ( int k = 0; k < face->count && all; ++k )
			{
				int u = cell->indices[face->first + k];
				int found = -1;
				for ( int j = 0; j < twin->count && found < 0; ++j )
				{
					int w = other->indices[twin->first + j];
					found = lpXPoly_SamePoint( cell, u, other, w ) ? w : -1;
				}
				all = found >= 0;
				if ( all )
				{
					lpVec3 a = lpIVertex_RoundToFloat( cell->vertices + u );
					lpVec3 b = lpIVertex_RoundToFloat( other->vertices + found );
					check->floatMismatches += memcmp( &a, &b, sizeof( a ) ) != 0 ? 1 : 0;
				}
			}
			check->twins += all ? 1 : 0;
			check->unmatched += all ? 0 : 1;
		}
	}
}
