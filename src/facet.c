// SPDX-License-Identifier: MIT

#include "facet.h"

#include <math.h>

static uint32_t lpHash3( int x, int y, int z, uint32_t seed )
{
	uint32_t h = seed ^ 0x9E3779B9u;
	h ^= (uint32_t)x * 0x85EBCA6Bu;
	h = ( h << 13 ) | ( h >> 19 );
	h ^= (uint32_t)y * 0xC2B2AE35u;
	h = ( h << 13 ) | ( h >> 19 );
	h ^= (uint32_t)z * 0x27D4EB2Fu;
	h ^= h >> 16;
	h *= 0x7FEB352Du;
	h ^= h >> 15;
	h *= 0x846CA68Bu;
	h ^= h >> 16;
	return h;
}

static float lpLattice( int x, int y, int z, uint32_t seed )
{
	return (float)( lpHash3( x, y, z, seed ) >> 8 ) * ( 1.0f / 16777216.0f );
}

// Trilinear value noise in [0, 1). Rendering only, so float floor is fine here.
static float lpValueNoise( lpVec3 p, uint32_t seed )
{
	float fx = floorf( p.x ), fy = floorf( p.y ), fz = floorf( p.z );
	int x = (int)fx, y = (int)fy, z = (int)fz;
	float tx = p.x - fx, ty = p.y - fy, tz = p.z - fz;
	tx = tx * tx * ( 3.0f - 2.0f * tx );
	ty = ty * ty * ( 3.0f - 2.0f * ty );
	tz = tz * tz * ( 3.0f - 2.0f * tz );

	float c000 = lpLattice( x, y, z, seed ), c100 = lpLattice( x + 1, y, z, seed );
	float c010 = lpLattice( x, y + 1, z, seed ), c110 = lpLattice( x + 1, y + 1, z, seed );
	float c001 = lpLattice( x, y, z + 1, seed ), c101 = lpLattice( x + 1, y, z + 1, seed );
	float c011 = lpLattice( x, y + 1, z + 1, seed ), c111 = lpLattice( x + 1, y + 1, z + 1, seed );

	float c00 = c000 + ( c100 - c000 ) * tx;
	float c10 = c010 + ( c110 - c010 ) * tx;
	float c01 = c001 + ( c101 - c001 ) * tx;
	float c11 = c011 + ( c111 - c011 ) * tx;
	float c0 = c00 + ( c10 - c00 ) * ty;
	float c1 = c01 + ( c11 - c01 ) * ty;
	return c0 + ( c1 - c0 ) * tz;
}

static uint32_t lpScaleColor( uint32_t rgb, float s )
{
	float r = (float)( ( rgb >> 16 ) & 0xFF ) * s;
	float g = (float)( ( rgb >> 8 ) & 0xFF ) * s;
	float b = (float)( rgb & 0xFF ) * s;
	uint32_t ri = r > 255.0f ? 255u : (uint32_t)r;
	uint32_t gi = g > 255.0f ? 255u : (uint32_t)g;
	uint32_t bi = b > 255.0f ? 255u : (uint32_t)b;
	return ( ri << 16 ) | ( gi << 8 ) | bi;
}

static uint32_t lpMixColor( uint32_t a, uint32_t b, float t )
{
	float r = (float)( ( a >> 16 ) & 0xFF ) * ( 1.0f - t ) + (float)( ( b >> 16 ) & 0xFF ) * t;
	float g = (float)( ( a >> 8 ) & 0xFF ) * ( 1.0f - t ) + (float)( ( b >> 8 ) & 0xFF ) * t;
	float bl = (float)( a & 0xFF ) * ( 1.0f - t ) + (float)( b & 0xFF ) * t;
	return ( (uint32_t)r << 16 ) | ( (uint32_t)g << 8 ) | (uint32_t)bl;
}

// 0xRRGGBB of a cut face of `material` at object-space point p
static uint32_t lpInteriorColor( const lpMaterialDef* materials, uint8_t material, lpVec3 p, lpVec3 axis )
{
	const lpMaterialDef* m = materials + material;
	uint32_t base = m->interiorColor;

	switch ( material )
	{
		case lp_wood:
		{
			// Growth rings around the grain axis, wobbled by noise, plus streaks along the grain
			lpVec3 along = lpMulSV( lpDot( p, axis ), axis );
			lpVec3 across = lpSub( p, along );
			float r = lpLength( across );
			float wobble = lpValueNoise( lpMulSV( 3.0f, p ), 11u );
			float ring = r * 14.0f + 1.5f * wobble;
			ring -= floorf( ring );
			uint32_t dark = lpScaleColor( base, 0.72f );
			float streak = lpValueNoise( lpAdd( lpMulSV( 0.6f, along ), lpMulSV( 9.0f, across ) ), 23u );
			uint32_t c = ring < 0.35f ? dark : base;
			return lpScaleColor( c, 0.88f + 0.22f * streak );
		}

		case lp_brick:
		{
			float n = lpValueNoise( lpMulSV( 7.0f, p ), 5u );
			return lpScaleColor( base, 0.82f + 0.3f * n );
		}

		case lp_stone:
		case lp_concrete:
		{
			float n = lpValueNoise( lpMulSV( 5.0f, p ), 7u );
			float speck = lpValueNoise( lpMulSV( 23.0f, p ), 9u );
			uint32_t c = lpScaleColor( base, 0.86f + 0.22f * n );
			return speck > 0.82f ? lpScaleColor( c, 0.7f ) : c;
		}

		case lp_plaster:
		{
			float n = lpValueNoise( lpMulSV( 4.0f, p ), 3u );
			return lpScaleColor( base, 0.94f + 0.08f * n );
		}

		case lp_foliage:
		{
			float n = lpValueNoise( lpMulSV( 3.0f, p ), 13u );
			return lpMixColor( base, 0x6B8F2Au, 0.5f * n );
		}

		default:
			return base;
	}
}

static uint32_t lpToRGBA( uint32_t rgb )
{
	uint32_t r = ( rgb >> 16 ) & 0xFF;
	uint32_t g = ( rgb >> 8 ) & 0xFF;
	uint32_t b = rgb & 0xFF;
	return 0xFF000000u | ( b << 16 ) | ( g << 8 ) | r;
}

static int8_t lpSnorm8( float v )
{
	float s = v * 127.0f;
	s = s > 127.0f ? 127.0f : ( s < -127.0f ? -127.0f : s );
	return (int8_t)( s < 0.0f ? s - 0.5f : s + 0.5f );
}

int lpBuildFacetMesh( const lpShape* shape, const lpFacetParams* params, lpVertex* vertices, int capacity )
{
	int count = 0;
	for ( int f = 0; f < shape->faceCount; ++f )
	{
		const lpFace* face = shape->faces + f;
		int triangleCount = face->count - 2;
		if ( count + 3 * triangleCount > capacity )
		{
			return -1;
		}

		uint32_t rgb;
		if ( face->tag == LP_TAG_EXTERIOR )
		{
			// Authored surface: a small per-plane brightness jitter gives the faceted look. Keyed by the plane, not
			// the piece, so coplanar faces of neighbouring cells match and intact surfaces stay seamless.
			lpVec3 n = face->plane.normal;
			int qx = (int)floorf( n.x * 64.0f + 0.5f );
			int qy = (int)floorf( n.y * 64.0f + 0.5f );
			int qz = (int)floorf( n.z * 64.0f + 0.5f );
			int qd = (int)floorf( face->plane.offset * 16.0f + 0.5f );
			uint32_t h = lpHash3( qx * 131 + qy, qz * 131 + qd, face->material, 0xFACEu );
			float jitter = 0.93f + 0.1f * (float)( h & 0xFF ) / 255.0f;
			rgb = lpScaleColor( params->exteriorColor, jitter );
		}
		else
		{
			lpVec3 c = lpVec3_zero;
			for ( int k = 0; k < face->count; ++k )
			{
				c = lpAdd( c, shape->vertices[shape->indices[face->first + k]] );
			}
			c = lpMulSV( 1.0f / (float)face->count, c );
			rgb = lpInteriorColor( params->materials, face->material, c, params->axis );
		}
		uint32_t color = lpToRGBA( rgb );

		lpVec3 n = face->plane.normal;
		int8_t normal[4] = { lpSnorm8( n.x ), lpSnorm8( n.y ), lpSnorm8( n.z ), 0 };

		lpVec3 a = shape->vertices[shape->indices[face->first]];
		for ( int k = 1; k + 1 < face->count; ++k )
		{
			lpVec3 b = shape->vertices[shape->indices[face->first + k]];
			lpVec3 d = shape->vertices[shape->indices[face->first + k + 1]];
			lpVec3 tri[3] = { a, b, d };
			for ( int t = 0; t < 3; ++t )
			{
				lpVertex* v = vertices + count++;
				v->position[0] = tri[t].x;
				v->position[1] = tri[t].y;
				v->position[2] = tri[t].z;
				memcpy( v->normal, normal, 4 );
				v->color = color;
			}
		}
	}
	return count;
}
