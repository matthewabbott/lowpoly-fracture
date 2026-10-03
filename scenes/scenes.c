// SPDX-License-Identifier: MIT

#include "scenes.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// C-library roots and trig differ between libms, so scenes use the core's cube root and Box3D's own trig
// (determinism rule 12)
float lpCbrt( float x );

enum
{
	lp_maxParts = 4096,
	lp_maxPoints = 8192
};

// Collects convex parts of one object, then creates it
typedef struct lpBuilder
{
	lpPartDef parts[lp_maxParts];
	int partCount;
	lpVec3 points[lp_maxPoints];
	int pointCount;
} lpBuilder;

static lpBuilder lp_builder;

static void lpBegin( void )
{
	lp_builder.partCount = 0;
	lp_builder.pointCount = 0;
}

static lpPartDef* lpBox( lpVec3 center, lpVec3 half, lpQuat q, int material, uint32_t color, bool anchored )
{
	if ( lp_builder.partCount == lp_maxParts )
	{
		return NULL;
	}
	lpPartDef* part = lp_builder.parts + lp_builder.partCount++;
	*part = lpDefaultPartDef();
	part->halfExtents = half;
	part->transform = (lpTransform){ center, q };
	part->material = (uint8_t)material;
	part->color = color;
	part->anchored = anchored;
	return part;
}

static lpPartDef* lpHull( const lpVec3* points, int count, int material, uint32_t color, bool anchored )
{
	if ( lp_builder.partCount == lp_maxParts || lp_builder.pointCount + count > lp_maxPoints )
	{
		return NULL;
	}
	lpVec3* dst = lp_builder.points + lp_builder.pointCount;
	memcpy( dst, points, sizeof( lpVec3 ) * (size_t)count );
	lp_builder.pointCount += count;

	lpPartDef* part = lp_builder.parts + lp_builder.partCount++;
	*part = lpDefaultPartDef();
	part->points = dst;
	part->pointCount = count;
	part->material = (uint8_t)material;
	part->color = color;
	part->anchored = anchored;
	return part;
}

// Creates the object from the builder's parts, with the rest of its definition from `def` (velocity, detonator,
// gravity scale)
static int lpCommitDef( lpWorld* world, lpVec3 position, lpQuat q, lpObjectDef def )
{
	def.transform.p = lpToPos( position );
	def.transform.q = q;
	def.parts = lp_builder.parts;
	def.partCount = lp_builder.partCount;
	return lpCreateObject( world, &def );
}

static int lpCommit( lpWorld* world, lpVec3 position, float yaw, bool isStatic )
{
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = isStatic;
	return lpCommitDef( world, position, lpMakeQuatFromAxisAngle( (lpVec3){ 0.0f, 1.0f, 0.0f }, yaw ), def );
}

static lpQuat lpYaw( float angle )
{
	return lpMakeQuatFromAxisAngle( (lpVec3){ 0.0f, 1.0f, 0.0f }, angle );
}

// Deterministic per-scene variation (splitmix64)
static uint64_t lpNext( uint64_t* state )
{
	uint64_t z = ( *state += 0x9E3779B97F4A7C15ull );
	z = ( z ^ ( z >> 30 ) ) * 0xBF58476D1CE4E5B9ull;
	z = ( z ^ ( z >> 27 ) ) * 0x94D049BB133111EBull;
	return z ^ ( z >> 31 );
}

static float lpUnit( uint64_t* state )
{
	return (float)( lpNext( state ) >> 40 ) * ( 1.0f / 16777216.0f );
}

// Palette
#define LP_BRICK 0xB5553Au
#define LP_BRICK_DARK 0x96452Fu
#define LP_PLASTER 0xE6DDC8u
#define LP_PLASTER_WARM 0xE8C9A0u
#define LP_BEAM 0x7A4E2Du
#define LP_PLANK 0xA8743Fu
#define LP_ROOF 0x8E3F2Eu
#define LP_ROOF_SLATE 0x5B6570u
#define LP_STONE 0x9C978Cu
#define LP_STONE_DARK 0x7F7A70u
#define LP_GLASS 0x9FD3E0u
#define LP_LEAF 0x5E9E3Eu
#define LP_LEAF_DARK 0x3F7A34u
#define LP_BARK 0x6B4A2Fu
#define LP_GRASS 0x7FA35Au
#define LP_CONCRETE 0xA8A49Au

#ifndef LPF_BUILD_ID
#define LPF_BUILD_ID "unknown"
#endif

int lpSceneDescribeSession( const lpWorld* world, int scene, int period, float timeStep, int subSteps, char* buffer, int size )
{
	int length = lpWorld_DescribeSession( world, buffer, size );
	char app[256];
	int n = snprintf( app, sizeof( app ), "build %s\nprotocol %d\nscene %s\nperiod %d\ndt %.9g\nsubsteps %d\n", LPF_BUILD_ID,
					  LP_PROTOCOL_VERSION, lpSceneName( scene ), period, (double)timeStep, subSteps );
	n = n < (int)sizeof( app ) ? n : (int)sizeof( app ) - 1;
	if ( length < size - 1 )
	{
		int fit = n < size - 1 - length ? n : size - 1 - length;
		memcpy( buffer + length, app, (size_t)fit );
		buffer[length + fit] = 0;
	}
	return length + n;
}

const char* lpSceneName( int scene )
{
	switch ( scene )
	{
		case lp_sceneWall:
			return "walls";
		case lp_sceneHouse:
			return "house";
		case lp_sceneTown:
			return "town";
		case lp_sceneTower:
			return "tower";
		case lp_scenePile:
			return "pile";
		case lp_sceneLumber:
			return "lumber";
		case lp_sceneRuins:
			return "ruins";
		case lp_sceneYard:
			return "yard";
		case lp_sceneKeep:
			return "keep";
		case lp_sceneTrack:
			return "track";
		case lp_sceneMech:
			return "mech";
		case lp_sceneContraption:
			return "contraption";
		default:
			return "?";
	}
}

int lpSceneFromName( const char* name )
{
	for ( int i = 0; i < lp_sceneCount; ++i )
	{
		if ( strcmp( name, lpSceneName( i ) ) == 0 )
		{
			return i;
		}
	}
	return atoi( name );
}

void lpAddGround( lpWorld* world, float halfSize )
{
	lpBegin();
	lpBox( (lpVec3){ 0.0f, -0.5f, 0.0f }, (lpVec3){ halfSize, 0.5f, halfSize }, lpQuat_identity, lp_ground, LP_GRASS, true );
	lpCommit( world, lpVec3_zero, 0.0f, true );
}

// Wall segment along local x in [0, length], split into panels and rows. Panels in the bottom row are anchored.
static void lpWallPanels( const lpWorld* world, lpVec3 origin, float length, float y0, float y1, float thickness, int material, uint32_t color,
						  float panelWidth, bool anchorBottom )
{
	int columns = (int)ceilf( length / panelWidth );
	columns = columns < 1 ? 1 : columns;
	float height = y1 - y0;
	int rows = (int)ceilf( height / 1.6f );
	rows = rows < 1 ? 1 : rows;
	if ( lpWorld_GetMaterial( world, material )->pattern == lp_breakMasonry )
	{
		columns = rows = 1; // one solid wall: its course grid decides where it breaks
	}
	float w = length / (float)columns;
	float h = height / (float)rows;
	for ( int r = 0; r < rows; ++r )
	{
		for ( int c = 0; c < columns; ++c )
		{
			lpVec3 center = { origin.x + ( (float)c + 0.5f ) * w, origin.y + y0 + ( (float)r + 0.5f ) * h, origin.z };
			lpBox( center, (lpVec3){ 0.5f * w, 0.5f * h, 0.5f * thickness }, lpQuat_identity, material, color,
				   anchorBottom && r == 0 && y0 <= 0.001f );
		}
	}
}

static void lpAddWall( lpWorld* world, lpVec3 base, float yaw, float length, float height, float thickness, int material,
				uint32_t color, float panelWidth )
{
	lpBegin();
	lpWallPanels( world, (lpVec3){ -0.5f * length, 0.0f, 0.0f }, length, 0.0f, height, thickness, material, color, panelWidth, true );
	lpCommit( world, base, yaw, true );
}

typedef struct lpOpening
{
	float x0, x1, y0, y1;
	bool glass;
} lpOpening;

// A wall along local x from 0 to length at depth z, with openings. The solid parts are decomposed into boxes:
// full-height strips between openings, plus a sill and a lintel for each opening.
static void lpWallWithOpenings( lpQuat q, lpVec3 origin, float length, float y0, float y1, float thickness, int material,
								uint32_t color, const lpOpening* unsorted, int openingCount, bool anchorBottom )
{
	// Openings must be walked left to right
	lpOpening openings[8];
	openingCount = openingCount < 8 ? openingCount : 8;
	memcpy( openings, unsorted, sizeof( lpOpening ) * (size_t)openingCount );
	for ( int i = 1; i < openingCount; ++i )
	{
		for ( int j = i; j > 0 && openings[j].x0 < openings[j - 1].x0; --j )
		{
			lpOpening t = openings[j];
			openings[j] = openings[j - 1];
			openings[j - 1] = t;
		}
	}
	lpVec3 ax = lpRotateVector( q, (lpVec3){ 1.0f, 0.0f, 0.0f } );
	float x = 0.0f;
	for ( int i = 0; i <= openingCount; ++i )
	{
		float stripEnd = i < openingCount ? openings[i].x0 : length;
		if ( stripEnd - x > 0.05f )
		{
			// strip [x, stripEnd] full height, split into ~1.5 m panels
			int columns = (int)ceilf( ( stripEnd - x ) / 1.5f );
			float w = ( stripEnd - x ) / (float)columns;
			for ( int c = 0; c < columns; ++c )
			{
				float cx = x + ( (float)c + 0.5f ) * w;
				int rows = (int)ceilf( ( y1 - y0 ) / 1.6f );
				float h = ( y1 - y0 ) / (float)rows;
				for ( int r = 0; r < rows; ++r )
				{
					lpVec3 center = lpAdd( origin, lpMulSV( cx, ax ) );
					center.y = y0 + ( (float)r + 0.5f ) * h;
					lpBox( center, (lpVec3){ 0.5f * w, 0.5f * h, 0.5f * thickness }, q, material, color, anchorBottom && r == 0 );
				}
			}
		}
		if ( i == openingCount )
		{
			break;
		}

		const lpOpening* o = openings + i;
		float cx = 0.5f * ( o->x0 + o->x1 );
		float hw = 0.5f * ( o->x1 - o->x0 );
		lpVec3 center = lpAdd( origin, lpMulSV( cx, ax ) );
		if ( o->y0 - y0 > 0.05f )
		{
			center.y = 0.5f * ( y0 + o->y0 );
			lpBox( center, (lpVec3){ hw, 0.5f * ( o->y0 - y0 ), 0.5f * thickness }, q, material, color, anchorBottom );
		}
		if ( y1 - o->y1 > 0.05f )
		{
			center.y = 0.5f * ( o->y1 + y1 );
			lpBox( center, (lpVec3){ hw, 0.5f * ( y1 - o->y1 ), 0.5f * thickness }, q, material, color, false );
		}
		if ( o->glass )
		{
			center.y = 0.5f * ( o->y0 + o->y1 );
			lpBox( center, (lpVec3){ hw, 0.5f * ( o->y1 - o->y0 ), 0.02f }, q, lp_glass, LP_GLASS, false );
		}
		x = o->x1;
	}
}

static void lpAddHouse( lpWorld* world, lpVec3 base, float yaw, uint64_t seed )
{
	uint64_t rng = seed;
	float width = 6.0f + 2.0f * lpUnit( &rng );
	float depth = 5.0f + 1.5f * lpUnit( &rng );
	int stories = lpUnit( &rng ) < 0.35f ? 2 : 1;
	float storyHeight = 2.8f;
	float t = 0.3f;
	bool plaster = lpUnit( &rng ) < 0.5f;
	int wallMaterial = plaster ? lp_plaster : lp_brick;
	uint32_t wallColor = plaster ? ( lpUnit( &rng ) < 0.5f ? LP_PLASTER : LP_PLASTER_WARM ) : ( lpUnit( &rng ) < 0.5f ? LP_BRICK : LP_BRICK_DARK );
	uint32_t roofColor = lpUnit( &rng ) < 0.5f ? LP_ROOF : LP_ROOF_SLATE;

	lpBegin();
	float hw = 0.5f * width;
	float hd = 0.5f * depth;

	// Foundation slab
	lpBox( (lpVec3){ 0.0f, 0.1f, 0.0f }, (lpVec3){ hw + 0.1f, 0.1f, hd + 0.1f }, lpQuat_identity, lp_concrete, LP_CONCRETE, true );

	float floorY = 0.2f;
	for ( int s = 0; s < stories; ++s )
	{
		float y0 = floorY + (float)s * ( storyHeight + 0.2f );
		float y1 = y0 + storyHeight;

		// Front (+z) with a door on the ground floor and windows
		lpOpening front[3];
		int frontCount = 0;
		float innerW = width - 2.0f * t;
		if ( s == 0 )
		{
			front[frontCount++] = (lpOpening){ 0.5f * innerW - 0.5f, 0.5f * innerW + 0.5f, y0, y0 + 2.1f, false };
			front[frontCount++] = (lpOpening){ 0.8f, 1.9f, y0 + 0.9f, y0 + 2.0f, true };
		}
		else
		{
			front[frontCount++] = (lpOpening){ 0.8f, 1.9f, y0 + 0.9f, y0 + 2.0f, true };
			front[frontCount++] = (lpOpening){ innerW - 1.9f, innerW - 0.8f, y0 + 0.9f, y0 + 2.0f, true };
		}
		// front and back walls run between the side walls
		lpWallWithOpenings( lpQuat_identity, (lpVec3){ -hw + t, 0.0f, hd - 0.5f * t }, innerW, y0, y1, t, wallMaterial,
							wallColor, front, frontCount, s == 0 );

		lpOpening back[2] = {
			{ 1.0f, 2.0f, y0 + 0.9f, y0 + 2.0f, true },
			{ innerW - 2.0f, innerW - 1.0f, y0 + 0.9f, y0 + 2.0f, true },
		};
		lpWallWithOpenings( lpQuat_identity, (lpVec3){ -hw + t, 0.0f, -hd + 0.5f * t }, innerW, y0, y1, t, wallMaterial,
							wallColor, back, 2, s == 0 );

		// Side walls (along z) span the full depth
		lpOpening side[1] = { { 0.5f * depth - 0.6f, 0.5f * depth + 0.6f, y0 + 0.9f, y0 + 2.0f, true } };
		lpQuat sideQ = lpYaw( -0.5f * LP_PI );
		lpWallWithOpenings( sideQ, (lpVec3){ -hw + 0.5f * t, 0.0f, -hd }, depth, y0, y1, t, wallMaterial, wallColor, side, 1, s == 0 );
		lpWallWithOpenings( sideQ, (lpVec3){ hw - 0.5f * t, 0.0f, -hd }, depth, y0, y1, t, wallMaterial, wallColor, side, 1, s == 0 );

		// Floor slab / ceiling: wooden planks resting on the walls
		float ceilingY = y1 + 0.1f;
		int planks = (int)( depth / 0.8f );
		float pw = depth / (float)planks;
		for ( int p = 0; p < planks; ++p )
		{
			float z = -hd + ( (float)p + 0.5f ) * pw;
			lpBox( (lpVec3){ 0.0f, ceilingY, z }, (lpVec3){ hw, 0.1f, 0.5f * pw }, lpQuat_identity, lp_wood, LP_PLANK, false );
		}
	}

	// Gable roof: ridge along x
	float topY = floorY + (float)stories * ( storyHeight + 0.2f );
	float rise = 1.6f;
	float slope = lpAtan2( rise, hd );
	float slant = sqrtf( rise * rise + hd * hd );
	int roofSegments = (int)ceilf( width / 1.6f );
	float segW = ( width + 0.6f ) / (float)roofSegments;
	for ( int side = -1; side <= 1; side += 2 )
	{
		lpQuat q = lpMakeQuatFromAxisAngle( (lpVec3){ 1.0f, 0.0f, 0.0f }, (float)side * slope );
		for ( int k = 0; k < roofSegments; ++k )
		{
			float x = -0.5f * ( width + 0.6f ) + ( (float)k + 0.5f ) * segW;
			lpVec3 center = { x, topY + 0.5f * rise + 0.08f, (float)side * 0.5f * hd };
			lpBox( center, (lpVec3){ 0.5f * segW, 0.08f, 0.5f * slant + 0.25f }, q, lp_wood, roofColor, false );
		}
	}
	// Gable ends: triangular prisms
	for ( int side = -1; side <= 1; side += 2 )
	{
		float x = (float)side * ( hw - 0.5f * t );
		lpVec3 tri[6] = {
			{ x - 0.5f * t, topY, -hd }, { x - 0.5f * t, topY, hd }, { x - 0.5f * t, topY + rise, 0.0f },
			{ x + 0.5f * t, topY, -hd }, { x + 0.5f * t, topY, hd }, { x + 0.5f * t, topY + rise, 0.0f },
		};
		lpHull( tri, 6, wallMaterial, wallColor, false );
	}

	// Corner posts
	for ( int sx = -1; sx <= 1; sx += 2 )
	{
		for ( int sz = -1; sz <= 1; sz += 2 )
		{
			float h = topY - floorY;
			lpBox( (lpVec3){ (float)sx * ( hw + 0.08f ), floorY + 0.5f * h, (float)sz * ( hd + 0.08f ) }, (lpVec3){ 0.1f, 0.5f * h, 0.1f },
				   lpQuat_identity, lp_wood, LP_BEAM, false );
		}
	}

	lpCommit( world, base, yaw, true );
}

static int lpRing( lpVec3* out, lpVec3 center, float radius, int sides, float phase )
{
	for ( int i = 0; i < sides; ++i )
	{
		float a = phase + 2.0f * LP_PI * (float)i / (float)sides;
		lpCosSin cs = lpComputeCosSin( a );
		out[i] = (lpVec3){ center.x + radius * cs.cosine, center.y, center.z + radius * cs.sine };
	}
	return sides;
}

static void lpAddTree( lpWorld* world, lpVec3 base, float height, uint64_t seed )
{
	uint64_t rng = seed;
	lpBegin();
	int segments = 3;
	float r0 = 0.22f + 0.08f * lpUnit( &rng );
	float trunkH = 0.55f * height;
	float phase = lpUnit( &rng );
	for ( int s = 0; s < segments; ++s )
	{
		float y0 = trunkH * (float)s / (float)segments;
		float y1 = trunkH * (float)( s + 1 ) / (float)segments;
		float ra = r0 * ( 1.0f - 0.25f * (float)s / (float)segments );
		float rb = r0 * ( 1.0f - 0.25f * (float)( s + 1 ) / (float)segments );
		lpVec3 pts[16];
		lpRing( pts, (lpVec3){ 0.0f, y0, 0.0f }, ra, 7, phase );
		lpRing( pts + 7, (lpVec3){ 0.0f, y1, 0.0f }, rb, 7, phase );
		lpPartDef* part = lpHull( pts, 14, lp_wood, LP_BARK, s == 0 );
		if ( part != NULL )
		{
			part->grainAxis = (lpVec3){ 0.0f, 1.0f, 0.0f };
			part->joint = lp_jointSolid; // one living tree, not timber nailed together
		}
	}

	// Canopy: a few chunky low-poly blobs
	int blobs = 2 + (int)( lpNext( &rng ) % 2u );
	lpVec3 below = { 0.0f, trunkH - 0.3f, 0.0f }; // each blob reaches into what holds it up: the trunk, then the blob below
	for ( int b = 0; b < blobs; ++b )
	{
		float cr = 0.35f * height * ( 0.8f + 0.3f * lpUnit( &rng ) );
		lpVec3 c; // one draw per statement: C leaves the order inside an initializer open
		c.x = 0.4f * ( lpUnit( &rng ) - 0.5f );
		c.y = trunkH + cr * ( 0.4f + 0.9f * (float)b );
		c.z = 0.4f * ( lpUnit( &rng ) - 0.5f );
		lpVec3 pts[14];
		for ( int i = 0; i < 14; ++i )
		{
			lpVec3 d;
			d.x = lpUnit( &rng ) - 0.5f;
			d.y = 0.8f * ( lpUnit( &rng ) - 0.5f );
			d.z = lpUnit( &rng ) - 0.5f;
			d = lpNormalize( d );
			pts[i] = lpMulAdd( c, cr * ( 0.85f + 0.3f * lpUnit( &rng ) ), d );
		}
		pts[0] = below;
		below = c;
		lpPartDef* canopy = lpHull( pts, 14, lp_foliage, b % 2 == 0 ? LP_LEAF : LP_LEAF_DARK, false );
		if ( canopy != NULL )
		{
			canopy->joint = lp_jointSolid;
		}
		cr *= 0.7f;
	}
	lpCommit( world, base, 0.0f, true );
}

// A round log lying along its local x axis (dynamic unless isStatic)
static void lpAddLog( lpWorld* world, lpVec3 center, float yaw, float length, float radius, bool isStatic )
{
	lpBegin();
	lpVec3 pts[16];
	for ( int i = 0; i < 8; ++i )
	{
		lpCosSin cs = lpComputeCosSin( 0.3926991f + 0.7853982f * (float)i );
		pts[i] = (lpVec3){ -0.5f * length, radius * cs.cosine, radius * cs.sine };
		pts[8 + i] = (lpVec3){ 0.5f * length, radius * cs.cosine, radius * cs.sine };
	}
	lpPartDef* part = lpHull( pts, 16, lp_wood, LP_BARK, false );
	if ( part != NULL )
	{
		part->grainAxis = (lpVec3){ 1.0f, 0.0f, 0.0f };
	}
	lpCommit( world, center, yaw, isStatic );
}

static void lpAddStump( lpWorld* world, lpVec3 base, float height, float radius )
{
	lpBegin();
	lpVec3 pts[16];
	lpRing( pts, (lpVec3){ 0.0f, 0.0f, 0.0f }, radius, 8, 0.2f );
	lpRing( pts + 8, (lpVec3){ 0.0f, height, 0.0f }, 0.9f * radius, 8, 0.2f );
	lpPartDef* part = lpHull( pts, 16, lp_wood, LP_BARK, true );
	if ( part != NULL )
	{
		part->grainAxis = (lpVec3){ 0.0f, 1.0f, 0.0f };
	}
	lpCommit( world, base, 0.0f, true );
}

// A small plank shed: posts, horizontal wall planks, a lean-to roof
static void lpAddShed( lpWorld* world, lpVec3 base, float yaw )
{
	lpBegin();
	float w = 4.0f, d = 3.0f, h = 2.4f;
	for ( int sx = -1; sx <= 1; sx += 2 )
	{
		for ( int sz = -1; sz <= 1; sz += 2 )
		{
			lpBox( (lpVec3){ (float)sx * 0.5f * w, 0.5f * h, (float)sz * 0.5f * d }, (lpVec3){ 0.08f, 0.5f * h, 0.08f },
				   lpQuat_identity, lp_wood, LP_BEAM, true );
		}
	}
	int rows = 8;
	float ph = h / (float)rows;
	for ( int r = 0; r < rows; ++r )
	{
		float y = ( (float)r + 0.5f ) * ph;
		uint32_t color = ( r % 2 ) ? LP_PLANK : 0x9A6A3Au;
		lpBox( (lpVec3){ 0.0f, y, -0.5f * d }, (lpVec3){ 0.5f * w - 0.08f, 0.5f * ph - 0.005f, 0.03f }, lpQuat_identity, lp_wood, color, r == 0 );
		lpBox( (lpVec3){ -0.5f * w, y, 0.0f }, (lpVec3){ 0.03f, 0.5f * ph - 0.005f, 0.5f * d - 0.08f }, lpQuat_identity, lp_wood, color, r == 0 );
		lpBox( (lpVec3){ 0.5f * w, y, 0.0f }, (lpVec3){ 0.03f, 0.5f * ph - 0.005f, 0.5f * d - 0.08f }, lpQuat_identity, lp_wood, color, r == 0 );
	}
	lpQuat tilt = lpMakeQuatFromAxisAngle( (lpVec3){ 1.0f, 0.0f, 0.0f }, 0.18f );
	for ( int k = 0; k < 5; ++k )
	{
		float x = -0.5f * w - 0.2f + ( (float)k + 0.5f ) * ( w + 0.4f ) / 5.0f;
		lpBox( (lpVec3){ x, h + 0.12f, 0.0f }, (lpVec3){ 0.5f * ( w + 0.4f ) / 5.0f, 0.05f, 0.5f * d + 0.3f }, tilt, lp_wood, LP_ROOF, false );
	}
	lpCommit( world, base, yaw, true );
}

// A semicircular arch of dry-laid voussoirs on two piers: it stands by compression alone, and falls without its
// keystone. `voussoirs` should be odd so one sits at the crown.
static void lpAddArch( lpWorld* world, lpVec3 base, float radius, float thickness, float depth, int voussoirs )
{
	lpBegin();
	float springing = 1.2f;
	float mid = radius + 0.5f * thickness;
	for ( int side = -1; side <= 1; side += 2 )
	{
		lpPartDef* pier = lpBox( (lpVec3){ (float)side * mid, 0.5f * springing, 0.0f },
								 (lpVec3){ 0.5f * thickness + 0.1f, 0.5f * springing, 0.5f * depth }, lpQuat_identity, lp_stone,
								 LP_STONE_DARK, true );
		if ( pier != NULL )
		{
			pier->joint = lp_jointDry;
		}
	}
	for ( int i = 0; i < voussoirs; ++i )
	{
		float a0 = LP_PI * (float)i / (float)voussoirs;
		float a1 = LP_PI * (float)( i + 1 ) / (float)voussoirs;
		lpCosSin c0 = lpComputeCosSin( a0 );
		lpCosSin c1 = lpComputeCosSin( a1 );
		lpVec3 pts[8];
		for ( int k = 0; k < 2; ++k )
		{
			float z = k == 0 ? -0.5f * depth : 0.5f * depth;
			pts[4 * k + 0] = (lpVec3){ radius * c0.cosine, springing + radius * c0.sine, z };
			pts[4 * k + 1] = (lpVec3){ ( radius + thickness ) * c0.cosine, springing + ( radius + thickness ) * c0.sine, z };
			pts[4 * k + 2] = (lpVec3){ radius * c1.cosine, springing + radius * c1.sine, z };
			pts[4 * k + 3] = (lpVec3){ ( radius + thickness ) * c1.cosine, springing + ( radius + thickness ) * c1.sine, z };
		}
		uint32_t color = i == voussoirs / 2 ? LP_STONE_DARK : LP_STONE;
		lpPartDef* v = lpHull( pts, 8, lp_stone, color, false );
		if ( v != NULL )
		{
			v->joint = lp_jointDry;
		}
	}
	lpCommit( world, base, 0.0f, true );
}

// Columns carrying mortared stone lintels: take a column out and the two lintels on it come down
static void lpAddColonnade( lpWorld* world, lpVec3 base, int columns, float spacing, float height )
{
	lpBegin();
	for ( int i = 0; i < columns; ++i )
	{
		lpBox( (lpVec3){ (float)i * spacing, 0.5f * height, 0.0f }, (lpVec3){ 0.2f, 0.5f * height, 0.2f }, lpQuat_identity, lp_stone,
			   LP_STONE, true );
	}
	for ( int i = 0; i + 1 < columns; ++i )
	{
		float x0 = (float)i * spacing - ( i == 0 ? 0.2f : 0.0f );
		float x1 = (float)( i + 1 ) * spacing + ( i + 2 == columns ? 0.2f : 0.0f );
		lpBox( (lpVec3){ 0.5f * ( x0 + x1 ), height + 0.175f, 0.0f }, (lpVec3){ 0.5f * ( x1 - x0 ), 0.175f, 0.25f },
			   lpQuat_identity, lp_stone, LP_STONE_DARK, false );
	}
	lpCommit( world, base, 0.0f, true );
}

// A stone wall with two mortared balconies: the short one stands easily; the long one is near its limit (root joint
// at about 94%), so one hit at its root brings it down
static void lpAddBalconies( lpWorld* world, lpVec3 base )
{
	lpBegin();
	lpBox( (lpVec3){ 0.0f, 1.5f, 0.0f }, (lpVec3){ 2.0f, 1.5f, 0.2f }, lpQuat_identity, lp_stone, LP_STONE, true );
	lpBox( (lpVec3){ -1.0f, 2.2f, 0.2f + 0.4f }, (lpVec3){ 0.5f, 0.2f, 0.4f }, lpQuat_identity, lp_stone, LP_STONE_DARK, false );
	lpBox( (lpVec3){ 1.0f, 2.2f, 0.2f + 0.625f }, (lpVec3){ 0.5f, 0.2f, 0.625f }, lpQuat_identity, lp_stone, LP_STONE_DARK, false );
	lpCommit( world, base, 0.0f, true );
}

static void lpAddFence( lpWorld* world, lpVec3 base, float yaw, float length )
{
	lpBegin();
	int posts = (int)( length / 1.8f ) + 1;
	float step = length / (float)( posts - 1 );
	for ( int i = 0; i < posts; ++i )
	{
		float x = -0.5f * length + (float)i * step;
		lpBox( (lpVec3){ x, 0.55f, 0.0f }, (lpVec3){ 0.06f, 0.55f, 0.06f }, lpQuat_identity, lp_wood, LP_BEAM, true );
	}
	for ( int i = 0; i + 1 < posts; ++i )
	{
		float x = -0.5f * length + ( (float)i + 0.5f ) * step;
		for ( int r = 0; r < 2; ++r )
		{
			lpBox( (lpVec3){ x, 0.4f + 0.45f * (float)r, 0.1f }, (lpVec3){ 0.5f * step, 0.06f, 0.04f }, lpQuat_identity, lp_wood,
				   LP_PLANK, false );
		}
	}
	lpCommit( world, base, yaw, true );
}

static void lpAddTowerAt( lpWorld* world, lpVec3 base, int levels )
{
	lpBegin();
	int sides = 10;
	float radius = 2.6f;
	float t = 0.6f;
	float levelH = 1.4f;
	float side = 2.0f * radius * lpComputeCosSin( LP_PI / (float)sides ).sine + 0.02f;
	for ( int l = 0; l < levels; ++l )
	{
		float phase = ( l % 2 ) ? 0.5f * 2.0f * LP_PI / (float)sides : 0.0f;
		for ( int s = 0; s < sides; ++s )
		{
			float a = phase + 2.0f * LP_PI * (float)s / (float)sides;
			lpCosSin cs = lpComputeCosSin( a );
			lpVec3 c = { radius * cs.cosine, ( (float)l + 0.5f ) * levelH, radius * cs.sine };
			lpQuat q = lpYaw( -a + 0.5f * LP_PI );
			uint32_t color = ( ( l + s ) % 3 == 0 ) ? LP_STONE_DARK : LP_STONE;
			lpPartDef* block = lpBox( c, (lpVec3){ 0.5f * side, 0.5f * levelH, 0.5f * t }, q, lp_stone, color, l == 0 );
			if ( block != NULL )
			{
				block->joint = lp_jointDry; // dry-stacked: friction only, so the tower hinges and falls once it leans
			}
		}
	}
	// Crenellations
	for ( int s = 0; s < sides; s += 2 )
	{
		float a = 2.0f * LP_PI * (float)s / (float)sides;
		lpCosSin cs = lpComputeCosSin( a );
		lpVec3 c = { radius * cs.cosine, (float)levels * levelH + 0.35f, radius * cs.sine };
		lpPartDef* merlon = lpBox( c, (lpVec3){ 0.35f * side, 0.35f, 0.5f * t }, lpYaw( -a + 0.5f * LP_PI ), lp_stone, LP_STONE, false );
		if ( merlon != NULL )
		{
			merlon->joint = lp_jointDry;
		}
	}
	lpCommit( world, base, 0.0f, true );
}

// ---- the keep: a mortared stone keep of about 2000 pieces, the stress solve's big-building case ----

#define LP_KEEP_HALF 7.5f	// outer half-width
#define LP_KEEP_WALL 1.2f	// wall thickness
#define LP_KEEP_COURSE 0.6f // course height
#define LP_KEEP_BLOCK 1.25f // stone length: twelve to a course across the front

// An opening in a wall run: courses first..last are left open between s0 and s1, and the next course gets a lintel
typedef struct lpKeepOpening
{
	float s0, s1;
	int first, last;
} lpKeepOpening;

// A stone of a keep wall over [a, b] along the run (x when alongX, else z); `across` is the middle of the wall
static void lpKeepStone( bool alongX, float across, float thickness, float a, float b, float y0, float y1, uint32_t color,
						 bool anchored )
{
	float mid = 0.5f * ( a + b );
	float hy = 0.5f * ( y1 - y0 );
	lpVec3 center = alongX ? (lpVec3){ mid, y0 + hy, across } : (lpVec3){ across, y0 + hy, mid };
	lpVec3 half = alongX ? (lpVec3){ 0.5f * ( b - a ), hy, 0.5f * thickness } : (lpVec3){ 0.5f * thickness, hy, 0.5f * ( b - a ) };
	lpBox( center, half, lpQuat_identity, lp_stone, color, anchored );
}

// One course of a wall run over [a, b], in running bond: joints on a grid of stone lengths from the keep's corner,
// shifted half a stone on odd courses, and a leftover shorter than a third of a stone joins its neighbour. Openings of
// this course are left out; the course above an opening gets a lintel reaching half a stone past each side. A merlon
// course keeps every other stone.
static void lpKeepCourse( bool alongX, float across, float thickness, float a, float b, int course, const lpKeepOpening* openings,
						  int openingCount, bool merlons, uint64_t* rng )
{
	float y0 = LP_KEEP_COURSE * (float)course;
	float y1 = y0 + ( merlons ? 0.7f : LP_KEEP_COURSE );
	bool anchored = course == 0;

	float cut0[16], cut1[16];
	int cuts = 0;
	for ( int i = 0; i < openingCount && cuts < 16; ++i )
	{
		const lpKeepOpening* o = openings + i;
		float s0 = o->s0, s1 = o->s1;
		if ( course == o->last + 1 )
		{
			s0 -= 0.5f * LP_KEEP_BLOCK;
			s1 += 0.5f * LP_KEEP_BLOCK;
			lpKeepStone( alongX, across, thickness, s0, s1, y0, y1, LP_STONE_DARK, anchored );
		}
		else if ( course < o->first || course > o->last )
		{
			continue;
		}
		int j = cuts++;
		for ( ; j > 0 && cut0[j - 1] > s0; --j )
		{
			cut0[j] = cut0[j - 1];
			cut1[j] = cut1[j - 1];
		}
		cut0[j] = s0;
		cut1[j] = s1;
	}

	float offset = ( course % 2 ) ? 0.5f * LP_KEEP_BLOCK : 0.0f;
	float shortest = LP_KEEP_BLOCK / 3.0f;
	int index = 0;
	float s = a;
	for ( int i = 0; i <= cuts; ++i )
	{
		float e = i < cuts ? cut0[i] : b;
		if ( e - s > 0.05f )
		{
			float start = s;
			for ( int k = (int)ceilf( ( s + shortest + LP_KEEP_HALF - offset ) / LP_KEEP_BLOCK );; ++k )
			{
				float joint = -LP_KEEP_HALF + offset + (float)k * LP_KEEP_BLOCK;
				float end = joint < e - shortest ? joint : e;
				if ( merlons == false || index % 2 == 0 )
				{
					uint32_t color = lpUnit( rng ) < 0.25f ? LP_STONE_DARK : LP_STONE;
					lpKeepStone( alongX, across, thickness, start, end, y0, y1, color, anchored );
				}
				index += 1;
				if ( end >= e )
				{
					break;
				}
				start = end;
			}
		}
		if ( i < cuts )
		{
			s = cut1[i];
		}
	}
}

// A square keep, 15 m across, of mortared stone in running bond with interleaved corners: 1.2 m walls, a door and
// windows with lintels, a cross wall with a doorway on every floor, and `floors` wooden floors (planks nailed to beams
// that rest on stone corbels) under a crenellated parapet. Four floors make about 2000 pieces. Returns the body.
int lpAddKeep( lpWorld* world, lpVec3 base, int floors )
{
	floors = floors < 1 ? 1 : ( floors > 6 ? 6 : floors );
	lpBegin();
	uint64_t rng = 0x4B454550ull;
	const float h = LP_KEEP_HALF, t = LP_KEEP_WALL, inner = LP_KEEP_HALF - LP_KEEP_WALL, cross = 0.45f;
	int top = 5 * floors + 1; // the parapet course; the merlons stand on it

	// The front (+z) has the door and a window each side on the upper floors, the back the same windows
	lpKeepOpening front[16], back[16], doorways[8];
	int frontCount = 0, backCount = 0, doorwayCount = 0;
	front[frontCount++] = (lpKeepOpening){ -4.6f, -2.6f, 0, 4 };
	for ( int f = 1; f < floors; ++f )
	{
		for ( int side = -1; side <= 1; side += 2 )
		{
			lpKeepOpening window = { 3.6f * (float)side - 0.5f, 3.6f * (float)side + 0.5f, 5 * f + 2, 5 * f + 3 };
			front[frontCount++] = window;
			back[backCount++] = window;
		}
	}
	for ( int f = 0; f < floors; ++f )
	{
		doorways[doorwayCount++] = (lpKeepOpening){ -0.5f, 0.5f, f == 0 ? 0 : 5 * f + 1, 5 * f + 3 };
	}

	for ( int c = 0; c <= top + 1; ++c )
	{
		bool merlons = c == top + 1;
		float along = c % 2 == 0 ? h : inner; // even courses: the front and back hold the corners
		float side = c % 2 == 0 ? inner : h;
		lpKeepCourse( true, h - 0.5f * t, t, -along, along, c, front, frontCount, merlons, &rng );
		lpKeepCourse( true, -h + 0.5f * t, t, -along, along, c, back, backCount, merlons, &rng );
		lpKeepCourse( false, -h + 0.5f * t, t, -side, side, c, NULL, 0, merlons, &rng );
		lpKeepCourse( false, h - 0.5f * t, t, -side, side, c, NULL, 0, merlons, &rng );
		if ( c <= 5 * floors )
		{
			lpKeepCourse( false, 0.0f, 2.0f * cross, -inner, inner, c, doorways, doorwayCount, false, &rng );
		}
	}

	// Floors: eight beams each side of the cross wall, from wall to wall on corbels, and planks across them in
	// staggered rows, their ends meeting on every other beam
	for ( int f = 1; f <= floors; ++f )
	{
		float y = LP_KEEP_COURSE * (float)( 5 * f + 1 ); // top of the planks
		for ( int side = -1; side <= 1; side += 2 )
		{
			float s = (float)side;
			for ( int m = 0; m < 8; ++m )
			{
				float z = -4.9f + 1.4f * (float)m;
				lpBox( (lpVec3){ s * 0.5f * ( cross + inner ), y - 0.21f, z }, (lpVec3){ 0.5f * ( inner - cross ), 0.15f, 0.125f },
					   lpQuat_identity, lp_wood, LP_BEAM, false );
				lpBox( (lpVec3){ s * ( inner - 0.225f ), y - 0.51f, z }, (lpVec3){ 0.225f, 0.15f, 0.175f }, lpQuat_identity, lp_stone,
					   LP_STONE_DARK, false );
				lpBox( (lpVec3){ s * ( cross + 0.225f ), y - 0.51f, z }, (lpVec3){ 0.225f, 0.15f, 0.175f }, lpQuat_identity, lp_stone,
					   LP_STONE_DARK, false );
			}
			float x0 = cross + 0.04f;
			float pitch = ( inner - x0 ) / 14.0f;
			for ( int r = 0; r < 14; ++r )
			{
				float px0 = x0 + (float)r * pitch;
				float px1 = px0 + pitch - 0.04f;
				float z0 = -inner;
				for ( int m = 1 - r % 2;; m += 2 )
				{
					bool last = m > 7;
					float joint = -4.9f + 1.4f * (float)m;
					float z1 = last ? inner : joint - 0.02f;
					lpBox( (lpVec3){ s * 0.5f * ( px0 + px1 ), y - 0.03f, 0.5f * ( z0 + z1 ) },
						   (lpVec3){ 0.5f * ( px1 - px0 ), 0.03f, 0.5f * ( z1 - z0 ) }, lpQuat_identity, lp_wood, LP_PLANK, false );
					if ( last )
					{
						break;
					}
					z0 = joint + 0.02f;
				}
			}
		}
	}
	return lpCommit( world, base, 0.0f, true );
}

// ---- the yard: things joined by links ----

static lpObjectDef lpDynamicDef( void )
{
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	return def;
}

static lpLinkDef lpLinkBetween( int type, int bodyA, int bodyB, lpVec3 anchorA, lpVec3 anchorB )
{
	lpLinkDef def = lpDefaultLinkDef( type );
	def.bodyA = bodyA;
	def.bodyB = bodyB;
	def.anchorA = lpToPos( anchorA );
	def.anchorB = lpToPos( anchorB );
	return def;
}

// A cart of volatile crates parked on a ramp and tied to a post at the top; a brick wall waits at the bottom. Cut the
// rope and it rolls down into the wall: the crash tears wheels off their axles and sets the crates off.
static void lpAddCartOnRamp( lpWorld* world, float z )
{
	float slope = 0.35f; // well past a 24-sided wheel's tipping angle (7.5 degrees), so it rolls, and fast
	lpCosSin cs = lpComputeCosSin( slope );
	lpQuat tilt = lpMakeQuatFromAxisAngle( (lpVec3){ 0.0f, 0.0f, 1.0f }, -slope );
	lpVec3 down = { cs.cosine, -cs.sine, 0.0f }; // along the ramp, downhill
	lpVec3 normal = { cs.sine, cs.cosine, 0.0f };
	lpVec3 foot = { -5.0f, 0.0f, z }; // where the ramp's surface meets the ground
	float length = 7.0f;

	// The ramp: one anchored stone slab
	lpBegin();
	lpBox( lpVec3_zero, (lpVec3){ 0.5f * length, 0.1f, 1.2f }, lpQuat_identity, lp_stone, LP_STONE, true );
	lpCommitDef( world, lpMulAdd( lpMulAdd( foot, -0.5f * length, down ), -0.1f, normal ), tilt, lpDefaultObjectDef() );

	// The post at the top that the cart is tied to
	lpVec3 top = lpMulAdd( foot, -length, down );
	lpBegin();
	float postHalf = 0.5f * ( top.y + 0.9f );
	lpBox( lpVec3_zero, (lpVec3){ 0.1f, postHalf, 0.1f }, lpQuat_identity, lp_wood, LP_BEAM, true );
	int post = lpCommit( world, (lpVec3){ top.x - 0.5f, postHalf, z }, 0.0f, true );

	// The cart: a plank bed with low sides, 1.5 m down the ramp, its wheels resting on the slab
	lpVec3 origin = lpMulAdd( lpMulAdd( top, 1.5f, down ), 0.5f, normal );
	lpBegin();
	lpBox( lpVec3_zero, (lpVec3){ 0.8f, 0.05f, 0.5f }, lpQuat_identity, lp_wood, LP_PLANK, false );
	for ( int side = -1; side <= 1; side += 2 )
	{
		lpBox( (lpVec3){ 0.0f, 0.2f, 0.47f * (float)side }, (lpVec3){ 0.8f, 0.15f, 0.03f }, lpQuat_identity, lp_wood, LP_BEAM,
			   false );
		lpBox( (lpVec3){ 0.77f * (float)side, 0.2f, 0.0f }, (lpVec3){ 0.03f, 0.15f, 0.44f }, lpQuat_identity, lp_wood, LP_BEAM,
			   false );
	}
	int cart = lpCommitDef( world, origin, tilt, lpDynamicDef() );

	// Four 24-sided wheels on axle pegs (hinges): a hard crash tears them off
	for ( int i = 0; i < 4; ++i )
	{
		lpVec3 pts[48];
		for ( int k = 0; k < 24; ++k )
		{
			lpCosSin c = lpComputeCosSin( 0.2617994f * (float)k );
			pts[k] = (lpVec3){ 0.3f * c.cosine, 0.3f * c.sine, -0.05f };
			pts[24 + k] = (lpVec3){ 0.3f * c.cosine, 0.3f * c.sine, 0.05f };
		}
		lpBegin();
		lpHull( pts, 48, lp_wood, LP_BARK, false );
		lpVec3 hub = lpAdd( origin, lpRotateVector( tilt, (lpVec3){ i < 2 ? -0.55f : 0.55f, -0.2f, i % 2 == 0 ? -0.62f : 0.62f } ) );
		int wheel = lpCommitDef( world, hub, tilt, lpDynamicDef() );
		lpLinkDef axle = lpLinkBetween( lp_linkHinge, cart, wheel, hub, hub );
		axle.axis = (lpVec3){ 0.0f, 0.0f, 1.0f };
		axle.maxForce = 6000.0f;
		axle.maxTorque = 800.0f;
		lpCreateLink( world, &axle );
	}

	// Two crates of something volatile in the bed
	for ( int k = 0; k < 2; ++k )
	{
		lpBegin();
		lpBox( lpVec3_zero, (lpVec3){ 0.2f, 0.2f, 0.2f }, lpQuat_identity, lp_wood, LP_PLANK, false );
		lpObjectDef def = lpDynamicDef();
		def.detonator = (lpDetonatorDef){ 3.5f, 1.6f, 70000.0f, 10.0f };
		lpCommitDef( world, lpAdd( origin, lpRotateVector( tilt, (lpVec3){ k == 0 ? -0.35f : 0.3f, 0.26f, 0.0f } ) ), tilt, def );
	}

	// The rope, from the post to the back of the cart
	lpVec3 tie = lpAdd( origin, lpRotateVector( tilt, (lpVec3){ -0.8f, 0.2f, 0.0f } ) );
	lpLinkDef rope = lpLinkBetween( lp_linkRope, post, cart, (lpVec3){ top.x - 0.4f, top.y + 0.35f, z }, tie );
	lpCreateLink( world, &rope );
}

// A sign hanging on two ropes from a gallows
static void lpAddHangingSign( lpWorld* world, lpVec3 base )
{
	lpBegin();
	lpBox( (lpVec3){ -1.1f, 1.6f, 0.0f }, (lpVec3){ 0.1f, 1.6f, 0.1f }, lpQuat_identity, lp_wood, LP_BEAM, true );
	lpBox( (lpVec3){ 1.1f, 1.6f, 0.0f }, (lpVec3){ 0.1f, 1.6f, 0.1f }, lpQuat_identity, lp_wood, LP_BEAM, true );
	lpBox( (lpVec3){ 0.0f, 3.3f, 0.0f }, (lpVec3){ 1.3f, 0.1f, 0.12f }, lpQuat_identity, lp_wood, LP_BEAM, false );
	int gallows = lpCommit( world, base, 0.0f, true );
	lpBegin();
	lpBox( lpVec3_zero, (lpVec3){ 0.6f, 0.35f, 0.03f }, lpQuat_identity, lp_wood, LP_PLANK, false );
	int sign = lpCommit( world, (lpVec3){ base.x, base.y + 2.0f, base.z }, 0.0f, false );
	for ( int side = -1; side <= 1; side += 2 )
	{
		float x = base.x + 0.5f * (float)side;
		lpLinkDef rope = lpLinkBetween( lp_linkRope, gallows, sign, (lpVec3){ x, base.y + 3.2f, base.z },
										(lpVec3){ x, base.y + 2.35f, base.z } );
		lpCreateLink( world, &rope );
	}
}

// A door on a hinge in its own frame (a hinge stops all collision between the door and the frame)
static void lpAddDoorway( lpWorld* world, lpVec3 base )
{
	lpBegin();
	lpBox( (lpVec3){ -0.7f, 1.15f, 0.0f }, (lpVec3){ 0.1f, 1.15f, 0.1f }, lpQuat_identity, lp_wood, LP_BEAM, true );
	lpBox( (lpVec3){ 0.7f, 1.15f, 0.0f }, (lpVec3){ 0.1f, 1.15f, 0.1f }, lpQuat_identity, lp_wood, LP_BEAM, true );
	lpBox( (lpVec3){ 0.0f, 2.4f, 0.0f }, (lpVec3){ 0.8f, 0.1f, 0.1f }, lpQuat_identity, lp_wood, LP_BEAM, false );
	int frame = lpCommit( world, base, 0.0f, true );
	lpBegin();
	lpBox( lpVec3_zero, (lpVec3){ 0.55f, 1.02f, 0.03f }, lpQuat_identity, lp_wood, LP_PLANK, false );
	int door = lpCommit( world, (lpVec3){ base.x, base.y + 1.12f, base.z }, 0.0f, false );
	lpLinkDef hinge = lpLinkBetween( lp_linkHinge, frame, door, (lpVec3){ base.x - 0.575f, base.y + 1.12f, base.z }, lpVec3_zero );
	hinge.axis = (lpVec3){ 0.0f, 1.0f, 0.0f };
	hinge.lowerAngle = -1.4f;
	hinge.upperAngle = 1.4f;
	lpCreateLink( world, &hinge );
}

// A raised drawbridge in a stone gatehouse: hinged at its foot on the sill, held up by two ropes from under the
// lintel. It leans out a little, so when the ropes go it falls open.
static void lpAddDrawbridge( lpWorld* world, lpVec3 base )
{
	lpBegin();
	lpBox( (lpVec3){ -1.3f, 1.6f, -0.3f }, (lpVec3){ 0.4f, 1.6f, 0.5f }, lpQuat_identity, lp_stone, LP_STONE, true );
	lpBox( (lpVec3){ 1.3f, 1.6f, -0.3f }, (lpVec3){ 0.4f, 1.6f, 0.5f }, lpQuat_identity, lp_stone, LP_STONE, true );
	lpBox( (lpVec3){ 0.0f, 3.4f, -0.3f }, (lpVec3){ 1.7f, 0.2f, 0.5f }, lpQuat_identity, lp_stone, LP_STONE_DARK, false );
	lpBox( (lpVec3){ 0.0f, 0.05f, -0.3f }, (lpVec3){ 0.9f, 0.05f, 0.5f }, lpQuat_identity, lp_stone, LP_STONE_DARK, true );
	int gate = lpCommit( world, base, 0.0f, true );

	lpQuat lean = lpMakeQuatFromAxisAngle( (lpVec3){ 1.0f, 0.0f, 0.0f }, 0.05f );
	lpVec3 foot = { base.x, base.y + 0.1f, base.z + 0.2f }; // the sill's front edge
	lpVec3 center = lpAdd( foot, lpRotateVector( lean, (lpVec3){ 0.0f, 1.4f, -0.06f } ) );
	lpBegin();
	lpBox( lpVec3_zero, (lpVec3){ 0.85f, 1.4f, 0.06f }, lpQuat_identity, lp_wood, LP_PLANK, false );
	int bridge = lpCommitDef( world, center, lean, lpDynamicDef() );
	lpLinkDef hinge = lpLinkBetween( lp_linkHinge, gate, bridge, foot, lpVec3_zero );
	hinge.axis = (lpVec3){ 1.0f, 0.0f, 0.0f };
	lpCreateLink( world, &hinge );
	for ( int side = -1; side <= 1; side += 2 )
	{
		lpVec3 hook = { base.x + 0.7f * (float)side, base.y + 3.2f, base.z - 0.3f };
		lpVec3 top = lpAdd( center, lpRotateVector( lean, (lpVec3){ 0.7f * (float)side, 1.4f, 0.0f } ) );
		lpLinkDef rope = lpLinkBetween( lp_linkRope, gate, bridge, hook, top );
		lpCreateLink( world, &rope );
	}
}

// A porter's rack (a base board, two uprights and a top bar) with two flasks hung from the bar on strings. Dusted to
// half its weight so the grab tool carries it; jostled too hard, the flasks swing into it and go off.
static void lpAddPorterRack( lpWorld* world, lpVec3 base )
{
	lpBegin();
	lpBox( (lpVec3){ 0.0f, 0.03f, 0.0f }, (lpVec3){ 0.35f, 0.03f, 0.25f }, lpQuat_identity, lp_wood, LP_PLANK, false );
	lpBox( (lpVec3){ -0.3f, 0.66f, 0.0f }, (lpVec3){ 0.04f, 0.6f, 0.04f }, lpQuat_identity, lp_wood, LP_BEAM, false );
	lpBox( (lpVec3){ 0.3f, 0.66f, 0.0f }, (lpVec3){ 0.04f, 0.6f, 0.04f }, lpQuat_identity, lp_wood, LP_BEAM, false );
	lpBox( (lpVec3){ 0.0f, 1.3f, 0.0f }, (lpVec3){ 0.34f, 0.04f, 0.04f }, lpQuat_identity, lp_wood, LP_BEAM, false );
	lpObjectDef def = lpDynamicDef();
	def.gravityScale = 0.5f;
	int rack = lpCommitDef( world, base, lpQuat_identity, def );
	for ( int side = -1; side <= 1; side += 2 )
	{
		float x = base.x + 0.15f * (float)side;
		lpVec3 pts[12];
		lpRing( pts, (lpVec3){ 0.0f, -0.11f, 0.0f }, 0.06f, 6, 0.0f );
		lpRing( pts + 6, (lpVec3){ 0.0f, 0.11f, 0.0f }, 0.06f, 6, 0.0f );
		lpBegin();
		lpHull( pts, 12, lp_glass, LP_GLASS, false );
		lpObjectDef flask = lpDynamicDef();
		flask.detonator = (lpDetonatorDef){ 4.0f, 1.4f, 60000.0f, 10.0f };
		int bottle = lpCommitDef( world, (lpVec3){ x, base.y + 0.9f, base.z }, lpQuat_identity, flask );
		lpLinkDef string = lpLinkBetween( lp_linkRope, rack, bottle, (lpVec3){ x, base.y + 1.26f, base.z },
										  (lpVec3){ x, base.y + 1.01f, base.z } );
		string.maxForce = 400.0f;
		lpCreateLink( world, &string );
	}
}

// ---- the track: a ring road, and cars that drive laps ----

#define LP_TRACK_RADIUS 45.0f
#define LP_TRACK_SPEED 14.0f  // m/s the scripted drivers hold
#define LP_TRACK_LOOKAHEAD 12.0f // m along the ring they steer for

// A point on the ring of radius r at angle a (x = r cos a, z = r sin a). Yaw -a turns local +z along the ring
// (counterclockwise seen from above) and local +x outward.
static lpVec3 lpRingPoint( float r, float a, float y )
{
	lpCosSin cs = lpComputeCosSin( a );
	return (lpVec3){ r * cs.cosine, y, r * cs.sine };
}

// A ramp along local z: height 0 at z0, h at z1
static void lpWedge( float halfWidth, float z0, float z1, float h, int material, uint32_t color )
{
	lpVec3 pts[6] = { { -halfWidth, 0.0f, z0 }, { halfWidth, 0.0f, z0 }, { -halfWidth, 0.0f, z1 },
					  { halfWidth, 0.0f, z1 },	{ -halfWidth, h, z1 },	 { halfWidth, h, z1 } };
	lpHull( pts, 6, material, color, true );
}

// A sheet-metal part of the car kit, with its system
static lpPartDef* lpCarPart( lpVec3 center, lpVec3 half, int material, uint32_t color, uint16_t tag, uint8_t carries,
							 uint8_t sources, uint8_t needs )
{
	lpPartDef* part = lpBox( center, half, lpQuat_identity, material, color, false );
	if ( part != NULL )
	{
		part->system = (lpPartSystem){ tag, carries, sources, needs };
	}
	return part;
}

int lpAddCar( lpWorld* world, lpVec3 base, float yaw, int style )
{
	static const uint32_t paints[4] = { 0x2F6FB5u, 0xC0392Bu, 0xE0A526u, 0x3C8D4Fu };
	uint32_t paint = paints[( style % 4 + 4 ) % 4];
	const uint8_t fuel = 1u << lp_channelFuel, power = 1u << lp_channelPower, steer = 1u << lp_channelSteer;
	const uint32_t trim = 0x3A3D42u, glass = 0x9FD3E0u;
	float ride = 0.72f; // the floor pan's bottom above the ground, about where it rests
	lpQuat q = lpYaw( yaw );

	lpBegin();
	// The floor pan carries every line; what sits on it is bolted to it
	lpCarPart( (lpVec3){ 0.0f, 0.04f, 0.0f }, (lpVec3){ 0.85f, 0.04f, 2.1f }, lp_sheetMetal, trim, lp_tagFrame,
			   fuel | power | steer, 0, 0 );
	for ( int k = 0; k < 3; ++k ) // an engine of three blocks: lose one and a third of the power goes with it
	{
		lpPartDef* block = lpCarPart( (lpVec3){ 0.0f, 0.28f, 1.25f + 0.2f * (float)k }, (lpVec3){ 0.3f, 0.2f, 0.095f }, lp_sheetMetal,
									  0x55595Eu, lp_tagEngine, fuel, power, fuel );
		if ( block != NULL )
		{
			block->joint = lp_jointMounts; // a hard crash tears it off its mounts
		}
	}
	lpPartDef* tank = lpCarPart( (lpVec3){ 0.0f, 0.2f, -1.55f }, (lpVec3){ 0.4f, 0.12f, 0.22f }, lp_sheetMetal, 0x6B3A2Au,
								 lp_tagFuelTank, 0, fuel, 0 );
	if ( tank != NULL )
	{
		tank->detonator = (lpDetonatorDef){ 14.0f, 2.2f, 150000.0f, 12.0f };
	}
	lpCarPart( (lpVec3){ -0.45f, 0.18f, 1.05f }, (lpVec3){ 0.12f, 0.1f, 0.12f }, lp_sheetMetal, trim, lp_tagSteering, power, steer,
			   power );
	// Body panels, pillars and roof, glass, bumpers
	lpCarPart( (lpVec3){ 0.0f, 0.51f, 1.5f }, (lpVec3){ 0.8f, 0.03f, 0.55f }, lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
	lpCarPart( (lpVec3){ 0.0f, 0.35f, -1.75f }, (lpVec3){ 0.8f, 0.03f, 0.4f }, lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
	for ( int side = -1; side <= 1; side += 2 )
	{
		float x = 0.82f * (float)side;
		lpCarPart( (lpVec3){ x, 0.38f, -0.25f }, (lpVec3){ 0.03f, 0.3f, 0.7f }, lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
		for ( int k = 0; k < 2; ++k )
		{
			lpCarPart( (lpVec3){ 0.72f * (float)side, 0.575f, k == 0 ? 0.5f : -1.0f }, (lpVec3){ 0.06f, 0.495f, 0.06f },
					   lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
		}
	}
	lpCarPart( (lpVec3){ 0.0f, 1.1f, -0.25f }, (lpVec3){ 0.78f, 0.03f, 0.85f }, lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
	for ( int k = 0; k < 2; ++k )
	{
		lpCarPart( (lpVec3){ 0.0f, 0.78f, k == 0 ? 0.5f : -1.0f }, (lpVec3){ 0.66f, 0.29f, 0.015f }, lp_glass, glass, lp_tagGlass, 0, 0,
				   0 );
		lpCarPart( (lpVec3){ 0.0f, 0.1f, k == 0 ? 2.18f : -2.18f }, (lpVec3){ 0.85f, 0.1f, 0.08f }, lp_rubber, 0x2A2A2Au,
				   lp_tagBumper, 0, 0, 0 );
	}
	lpVec3 origin = { base.x, base.y + ride, base.z };
	lpObjectDef carDef = lpDynamicDef();
	carDef.solveStress = true; // a crash's deceleration loads its joints: an engine can tear off its mounts
	int body = lpCommitDef( world, origin, q, carDef );

	lpWheelDef wheels[4];
	for ( int i = 0; i < 4; ++i )
	{
		bool front = i >= 2;
		lpVec3 mount = { ( i & 1 ) ? 0.78f : -0.78f, 0.0f, front ? 1.35f : -1.35f };
		wheels[i] = lpDefaultWheelDef();
		wheels[i].mount = lpToPos( lpAdd( origin, lpRotateVector( q, mount ) ) );
		wheels[i].radius = 0.36f;
		wheels[i].width = 0.24f;
		wheels[i].driveShare = front ? 0.0f : 0.5f;
		wheels[i].driveNeeds = power;
		wheels[i].steerFactor = front ? 1.0f : 0.0f;
		wheels[i].steerNeeds = steer;
		wheels[i].handbrake = front == false;
		wheels[i].material = lp_rubber;
	}
	lpVehicleDef def = lpDefaultVehicleDef();
	def.body = body;
	def.forward = lpRotateVector( q, (lpVec3){ 0.0f, 0.0f, 1.0f } );
	def.wheels = wheels;
	def.wheelCount = 4;
	def.maxDriveForce = 10000.0f;
	def.maxSpeed = 30.0f;
	def.maxBrakeForce = 15000.0f;
	return lpCreateVehicle( world, &def );
}

int lpAddCrane( lpWorld* world, lpVec3 base, float loadMass )
{
	// A steel base plate with the mast's foot set in it: the crane's weight and swing load the timber at its foot
	lpBegin();
	lpPartDef* footing = lpBox( (lpVec3){ 0.0f, 0.4f, 0.0f }, (lpVec3){ 1.2f, 0.4f, 1.2f }, lpQuat_identity, lp_metal, 0x6F7378u,
								true );
	lpPartDef* mast = lpBox( (lpVec3){ 0.0f, 4.8f, 0.0f }, (lpVec3){ 0.35f, 4.0f, 0.35f }, lpQuat_identity, lp_wood, LP_BEAM, false );
	if ( footing != NULL && mast != NULL )
	{
		footing->joint = lp_jointSolid;
		mast->joint = lp_jointSolid; // set in the base plate: the timber itself is the limit
		mast->grainAxis = (lpVec3){ 0.0f, 1.0f, 0.0f };
	}
	int tower = lpCommit( world, base, 0.0f, true );

	lpBegin();
	lpBox( lpVec3_zero, (lpVec3){ 0.9f, 0.2f, 0.9f }, lpQuat_identity, lp_sheetMetal, 0xD9A12Bu, false );
	int deck = lpCommitDef( world, lpAdd( base, (lpVec3){ 0.0f, 9.0f, 0.0f } ), lpQuat_identity, lpDynamicDef() );
	lpBegin();
	lpPartDef* beam = lpBox( lpVec3_zero, (lpVec3){ 5.0f, 0.2f, 0.2f }, lpQuat_identity, lp_wood, 0xD9A12Bu, false );
	if ( beam != NULL )
	{
		beam->grainAxis = (lpVec3){ 1.0f, 0.0f, 0.0f };
	}
	int jib = lpCommitDef( world, lpAdd( base, (lpVec3){ 5.0f, 9.4f, 0.0f } ), lpQuat_identity, lpDynamicDef() );
	float side = lpCbrt( loadMass / lpWorld_GetMaterial( world, lp_metal )->density );
	lpBegin();
	lpBox( lpVec3_zero, (lpVec3){ 0.5f * side, 0.5f * side, 0.5f * side }, lpQuat_identity, lp_metal, 0x3A3D42u, false );
	int load = lpCommitDef( world, lpAdd( base, (lpVec3){ 9.8f, 9.2f - 5.0f - 0.5f * side, 0.0f } ), lpQuat_identity, lpDynamicDef() );

	lpVec3 top = lpAdd( base, (lpVec3){ 0.0f, 8.8f, 0.0f } );
	lpLinkDef slew = lpLinkBetween( lp_linkHinge, tower, deck, top, top );
	slew.axis = (lpVec3){ 0.0f, 1.0f, 0.0f };
	slew.maxForce = 1e6f; // a slewing ring
	slew.maxTorque = 2e6f;
	slew.strength = 20000.0f;
	slew.motor = (lpMotorDef){ 40000.0f, 0.3f, 1.0f, 0, 40000.0f };
	slew.userId = lp_linkSlew;
	lpCreateLink( world, &slew );

	lpVec3 root = lpAdd( base, (lpVec3){ 0.0f, 9.4f, 0.0f } );
	lpLinkDef luff = lpLinkBetween( lp_linkHinge, deck, jib, root, root );
	luff.axis = (lpVec3){ 0.0f, 0.0f, 1.0f };
	luff.lowerAngle = -0.2f;
	luff.upperAngle = 0.6f;
	luff.maxForce = 1e6f;
	luff.maxTorque = 2e6f;
	luff.strength = 20000.0f;
	luff.motor = (lpMotorDef){ 300000.0f, 0.2f, 2.0f, 0, 300000.0f };
	luff.userId = lp_linkLuff;
	lpCreateLink( world, &luff );

	lpVec3 tip = lpAdd( base, (lpVec3){ 9.8f, 9.2f, 0.0f } );
	lpLinkDef winch = lpLinkBetween( lp_linkRope, jib, load, tip, lpAdd( base, (lpVec3){ 9.8f, 9.2f - 5.0f, 0.0f } ) );
	winch.maxForce = 1e6f; // steel cable
	winch.strength = 20000.0f;
	winch.userId = lp_linkWinch;
	lpCreateLink( world, &winch );
	return tower;
}

// ---- the hexapod: a walking mech ----

// A leg in its own plane: u outward from the hip, v up. The hip's yaw hinge is at the torso's side, the femur's pitch
// hinge 0.45 m out, the knee 1.2 m further up and out, the foot 1.6 m below and a little out from the knee.
#define LP_HEX_FEMUR_U 0.45f
#define LP_HEX_KNEE_U 1.4892f
#define LP_HEX_KNEE_V 0.6f
#define LP_HEX_FOOT_U 1.8392f
#define LP_HEX_FOOT_V -0.95f
#define LP_HEX_SOLE 0.08f // the rubber foot's half height below the tibia's tip
#define LP_HEX_RIDE 1.09f // the torso's centre above the ground, standing as built (the sole reaches 0.14 m below the tibia)

enum
{
	lp_hexBlock,
	lp_hexFemur,
	lp_hexTibia
};

// One leg segment: an object along its local x, centred at `center`, turned by q, of sheet metal: a box (the hip block),
// two halves welded at the middle (the femur: its bone, where a blast weakens it and a landing snaps it), or a box with
// a rubber sole welded to its far end (the tibia). The welds hold more than the servos can put on them.
static int lpHexSegment( lpWorld* world, lpVec3 center, lpQuat q, lpVec3 half, uint32_t color, int kind, lpQuat legQ )
{
	const uint8_t lines = ( 1u << lp_channelPower ) | ( 1u << lp_channelHydraulics ) | ( 1u << lp_channelControl );
	lpBegin();
	int halves = kind == lp_hexFemur ? 2 : 1;
	lpVec3 partHalf = { half.x / (float)halves, half.y, half.z };
	for ( int h = 0; h < halves; ++h )
	{
		float x = halves == 2 ? ( h == 0 ? -partHalf.x : partHalf.x ) : 0.0f;
		lpPartDef* part = lpBox( (lpVec3){ x, 0.0f, 0.0f }, partHalf, lpQuat_identity, lp_sheetMetal, color, false );
		if ( part != NULL )
		{
			part->grainAxis = (lpVec3){ 1.0f, 0.0f, 0.0f };
			part->system = (lpPartSystem){ lp_tagLeg, lines, 0, 0 };
			part->joint = kind == lp_hexBlock ? part->joint : lp_jointWeld;
		}
	}
	if ( kind == lp_hexTibia )
	{
		// The sole sits square to the leg's plane (not tilted with the tibia), its top at the tibia's tip, welded on
		lpQuat local = lpInvMulQuat( q, legQ );
		lpVec3 tip = { half.x, 0.0f, 0.0f };
		lpPartDef* sole = lpBox( lpAdd( tip, lpRotateVector( local, (lpVec3){ 0.0f, -LP_HEX_SOLE + 0.02f, 0.0f } ) ),
								 (lpVec3){ 0.1f, LP_HEX_SOLE, 0.1f }, local, lp_rubber, 0x2A2A2Au, false );
		if ( sole != NULL )
		{
			sole->joint = lp_jointWeld;
		}
	}
	lpObjectDef def = lpDynamicDef();
	def.inertiaRadius = 0.5f; // a slender leg on joints: Box3D holds them as stiffly as the leg's inertia allows
	def.solveStress = true;	  // a landing loads its bones: a segment cracked by a hit snaps
	return lpCommitDef( world, center, q, def );
}

int lpAddHexapod( lpWorld* world, lpVec3 base, float yaw, int style )
{
	static const uint32_t paints[4] = { 0x8A8F4Bu, 0x4F6E8Cu, 0xB5652Eu, 0x5D5F63u };
	uint32_t paint = paints[( style % 4 + 4 ) % 4];
	const uint32_t steel = 0x55595Eu, dark = 0x3A3D42u;
	const lpVec3 up = { 0.0f, 1.0f, 0.0f };
	lpQuat q = lpYaw( yaw );
	lpVec3 origin = { base.x, base.y + LP_HEX_RIDE, base.z };

	// The torso, all armor welded together: a frame (the belly skid under it) carrying every line, a deck, and between
	// them a reactor (power), a hydraulic reservoir and a computer (control), both run on power. The legs are sheet metal:
	// they are what a grenade takes
	const uint8_t power = 1u << lp_channelPower, hydraulics = 1u << lp_channelHydraulics, control = 1u << lp_channelControl;
	const uint8_t lines = power | hydraulics | control;
	lpBegin();
	lpPartDef* frame = lpBox( (lpVec3){ 0.0f, 0.0f, 0.0f }, (lpVec3){ 0.95f, 0.08f, 1.35f }, lpQuat_identity, lp_armor, steel, false );
	lpPartDef* reactor = lpBox( (lpVec3){ 0.0f, 0.33f, -0.75f }, (lpVec3){ 0.35f, 0.25f, 0.35f }, lpQuat_identity, lp_armor, dark, false );
	lpPartDef* reservoir = lpBox( (lpVec3){ 0.45f, 0.28f, 0.35f }, (lpVec3){ 0.3f, 0.2f, 0.25f }, lpQuat_identity, lp_armor, 0x7A5C2Eu,
								  false );
	lpPartDef* computer = lpBox( (lpVec3){ -0.45f, 0.23f, 0.45f }, (lpVec3){ 0.2f, 0.15f, 0.2f }, lpQuat_identity, lp_armor, 0x2E4A3Au,
								 false );
	lpBox( (lpVec3){ 0.0f, 0.61f, 0.0f }, (lpVec3){ 0.9f, 0.03f, 1.2f }, lpQuat_identity, lp_armor, paint, false );
	if ( frame != NULL && reactor != NULL && reservoir != NULL && computer != NULL )
	{
		frame->system = (lpPartSystem){ lp_tagFrame, lines, 0, 0 };
		reactor->system = (lpPartSystem){ lp_tagReactor, power, power, 0 };
		// It carries what it needs; its fluid leaks from a cut line until the valves close (3 s)
		reservoir->system = (lpPartSystem){ lp_tagReservoir, hydraulics | power, hydraulics, power, 100.0f, 3.0f };
		computer->system = (lpPartSystem){ lp_tagComputer, control | power, control, power };
	}
	lpObjectDef torsoDef = lpDynamicDef();
	torsoDef.solveStress = true;
	int torso = lpCommitDef( world, origin, q, torsoDef );

	// Legs in order around the body (right front, middle, rear, then left rear, middle, front): each one's neighbours
	// are the ones next to it in this list
	lpLimbDef limbs[6];
	for ( int leg = 0; leg < 6; ++leg )
	{
		float side = leg < 3 ? 1.0f : -1.0f;
		int row = leg < 3 ? leg : 5 - leg; // 0 front, 1 middle, 2 rear
		float splay = row == 0 ? 0.61f : ( row == 1 ? 0.0f : -0.61f ); // 35 degrees toward the front or the rear
		lpCosSin cs = lpComputeCosSin( splay );
		lpVec3 d = lpRotateVector( q, (lpVec3){ side * cs.cosine, 0.0f, cs.sine } ); // outward
		lpVec3 t = lpCross( d, up );												   // the pitch hinges' axis
		lpMatrix3 m = { d, up, t };
		lpQuat legQ = lpMakeQuatFromMatrix( &m );
		lpVec3 hip = lpAdd( origin, lpRotateVector( q, (lpVec3){ 0.95f * side, 0.0f, row == 0 ? 1.1f : ( row == 1 ? 0.0f : -1.1f ) } ) );
		lpVec3 femurRoot = lpMulAdd( hip, LP_HEX_FEMUR_U, d );
		lpVec3 knee = lpAdd( lpMulAdd( hip, LP_HEX_KNEE_U, d ), lpMulSV( LP_HEX_KNEE_V, up ) );
		lpVec3 foot = lpAdd( lpMulAdd( hip, LP_HEX_FOOT_U, d ), lpMulSV( LP_HEX_FOOT_V, up ) );

		// The hip block, the femur rising 30 degrees, the tibia down to the foot: slender, a third of the mech's weight (legs
		// as heavy as the torso shove it about as they swing)
		lpQuat femurTilt = lpMakeQuatFromAxisAngle( (lpVec3){ 0.0f, 0.0f, 1.0f }, 0.5236f );
		lpVec3 shin = { LP_HEX_FOOT_U - LP_HEX_KNEE_U, LP_HEX_FOOT_V - LP_HEX_KNEE_V, 0.0f };
		float shinLength = lpLength( shin );
		lpQuat tibiaTilt = lpMakeQuatFromAxisAngle( (lpVec3){ 0.0f, 0.0f, 1.0f }, lpAtan2( shin.y, shin.x ) );
		int block = lpHexSegment( world, lpLerp( hip, femurRoot, 0.5f ), legQ, (lpVec3){ 0.22f, 0.15f, 0.15f }, steel, lp_hexBlock, legQ );
		int femur = lpHexSegment( world, lpLerp( femurRoot, knee, 0.5f ), lpMulQuat( legQ, femurTilt ), (lpVec3){ 0.6f, 0.09f, 0.09f },
								  paint, lp_hexFemur, legQ );
		int tibia = lpHexSegment( world, lpLerp( knee, foot, 0.5f ), lpMulQuat( legQ, tibiaTilt ),
								  (lpVec3){ 0.5f * shinLength, 0.08f, 0.085f }, paint, lp_hexTibia, legQ );

		// Motorised hinges: the hip turns the leg about the vertical, the femur and the knee lift it
		int bodies[4] = { torso, block, femur, tibia };
		lpVec3 anchors[3] = { hip, femurRoot, knee };
		lpVec3 axes[3] = { lpRotateVector( q, up ), t, t };
		float limits[3] = { 0.6f, 0.9f, 1.2f };
		float caps[3] = { 10000.0f, 30000.0f, 30000.0f }; // a tripod stance loads the femur and knee about 15 kN*m, a swing the hip 12
		for ( int j = 0; j < 3; ++j )
		{
			lpLinkDef hinge = lpLinkBetween( lp_linkHinge, bodies[j], bodies[j + 1], anchors[j], anchors[j] );
			hinge.axis = axes[j];
			hinge.lowerAngle = -limits[j];
			hinge.upperAngle = limits[j];
			hinge.maxForce = 120000.0f; // walking loads them to about a third
			hinge.maxTorque = 150000.0f;
			hinge.strength = 20000.0f;
			// Gain 8 sways at 6 Hz on 4 substeps. The femur and knee jam as they are damaged
			hinge.motor = (lpMotorDef){ caps[j], 4.0f, 4.0f, hydraulics | control, 0.0f, j > 0 ? 0.6f : 0.0f };
			hinge.carries = lines;
			hinge.userId = (uint32_t)( lp_linkHexapod + 16 * leg + j );
			hinge.tearRatio = j > 0 ? 0.1f : 0.0f; // a stub of a leg segment left on a joint tears off
			limbs[leg].links[j] = lpCreateLink( world, &hinge );
		}
		limbs[leg].linkCount = 3;
		limbs[leg].foot = lpToPos( lpMulAdd( foot, -2.0f * LP_HEX_SOLE + 0.02f, up ) ); // the sole's bottom
	}
	lpRigDef def = lpDefaultRigDef();
	def.body = torso;
	def.gait.stride = 0.6f;	   // a step of about 0.85 m
	def.gait.swingTime = 0.35f; // so a tripod keeps up 2.3 m/s
	def.gait.maxSpeed = 2.3f;
	def.forward = lpRotateVector( q, (lpVec3){ 0.0f, 0.0f, 1.0f } );
	def.limbs = limbs;
	def.limbCount = 6;
	return lpCreateRig( world, &def );
}

// The crane's links by user id (-1 if gone)
static int lpFindLink( const lpWorld* world, uint32_t userId )
{
	for ( int i = 0; i < lpWorld_GetLinkCapacity( world ); ++i )
	{
		lpLinkState st = lpWorld_GetLinkState( world, i );
		if ( st.alive && st.userId == userId )
		{
			return i;
		}
	}
	return -1;
}

// One of the scene's own commands, for this tick (the world numbers them in the order they come)
static void lpSceneSubmit( lpWorld* world, lpCommand* c )
{
	c->tick = (int64_t)lpWorld_GetTick( world );
	c->peer = LP_PEER_SCENE;
	lpWorld_Submit( world, c );
}

static void lpSceneLink( lpWorld* world, int kind, int link, float value )
{
	if ( link >= 0 )
	{
		lpCommand c = { 0 };
		c.kind = (uint8_t)kind;
		c.link.link = link;
		c.link.generation = LP_ANY_GENERATION;
		c.link.value = value;
		lpSceneSubmit( world, &c );
	}
}

// The track's crane swings its load back and forth over the infield and winches it up and down
static void lpDriveCrane( lpWorld* world, int tick )
{
	int slew = lpFindLink( world, lp_linkSlew );
	int luff = lpFindLink( world, lp_linkLuff );
	int winch = lpFindLink( world, lp_linkWinch );
	lpSceneLink( world, lp_commandLinkTarget, slew, ( tick / 360 ) % 2 == 0 ? 1.2f : -1.2f );
	lpSceneLink( world, lp_commandLinkTarget, luff, 0.1f );
	// The winch reels at 1 m/s: a rope shortened at once would fling its load up
	float length = winch >= 0 ? lpWorld_GetLinkState( world, winch ).length : 0.0f;
	float goal = ( tick / 240 ) % 2 == 0 ? 5.0f : 3.0f;
	if ( winch >= 0 && length != goal )
	{
		lpSceneLink( world, lp_commandRopeLength, winch, length + lpClampFloat( goal - length, -1.0f / 60.0f, 1.0f / 60.0f ) );
	}
}

static void lpAddTrack( lpWorld* world )
{
	float r = LP_TRACK_RADIUS;
	lpAddGround( world, 120.0f );

	// Two stone kerbs across the road
	for ( int k = 0; k < 2; ++k )
	{
		float a = 0.9f + 0.1f * (float)k;
		lpBegin();
		lpBox( lpVec3_zero, (lpVec3){ 4.5f, 0.08f, 0.4f }, lpQuat_identity, lp_stone, LP_STONE, true );
		lpCommit( world, lpRingPoint( r, a, 0.08f ), -a, true );
	}

	// Loose crates in the road
	for ( int k = 0; k < 5; ++k )
	{
		float a = 1.5f + 0.02f * (float)( k % 3 );
		lpBegin();
		lpBox( lpVec3_zero, (lpVec3){ 0.35f, 0.35f, 0.35f }, lpQuat_identity, lp_wood, LP_PLANK, false );
		lpCommit( world, lpRingPoint( r - 1.5f + 1.0f * (float)k, a, 0.36f + 0.72f * (float)( k / 3 ) ), 0.3f * (float)k, false );
	}

	// A hump: up 0.7 m over 5 m, flat for 2 m, down again
	{
		float a = 2.6f;
		lpBegin();
		lpWedge( 4.0f, -6.0f, -1.0f, 0.7f, lp_stone, LP_STONE_DARK );
		lpBox( (lpVec3){ 0.0f, 0.35f, 0.0f }, (lpVec3){ 4.0f, 0.35f, 1.0f }, lpQuat_identity, lp_stone, LP_STONE_DARK, true );
		lpWedge( 4.0f, 6.0f, 1.0f, 0.7f, lp_stone, LP_STONE_DARK );
		lpCommit( world, lpRingPoint( r, a, 0.0f ), -a, true );
	}

	// A plank bridge: ramps up to two stone piers, an 8 m span of planks nailed across them
	{
		float a = 3.8f;
		lpVec3 at = lpRingPoint( r, a, 0.0f );
		lpBegin();
		lpWedge( 2.2f, -10.0f, -5.0f, 0.9f, lp_stone, LP_STONE );
		lpCommit( world, at, -a, true );
		lpBegin();
		lpWedge( 2.2f, 10.0f, 5.0f, 0.9f, lp_stone, LP_STONE );
		lpCommit( world, at, -a, true );
		lpBegin();
		for ( int side = -1; side <= 1; side += 2 )
		{
			lpBox( (lpVec3){ 0.0f, 0.4f, 4.5f * (float)side }, (lpVec3){ 2.2f, 0.4f, 0.5f }, lpQuat_identity, lp_stone, LP_STONE, true );
		}
		for ( int k = 0; k < 4; ++k )
		{
			lpPartDef* plank = lpBox( (lpVec3){ -1.35f + 0.9f * (float)k, 0.85f, 0.0f }, (lpVec3){ 0.44f, 0.05f, 5.0f },
									  lpQuat_identity, lp_wood, LP_PLANK, false );
			plank->grainAxis = (lpVec3){ 0.0f, 0.0f, 1.0f };
		}
		lpCommit( world, at, -a, true );
	}

	// A brick wall just outside the ring, for the cars that leave it
	{
		float a = 5.2f;
		lpCosSin cs = lpComputeCosSin( a );
		float yaw = lpAtan2( -cs.cosine, -cs.sine ); // turns the wall's local x along the ring's tangent
		lpAddWall( world, lpRingPoint( r + 8.0f, a, 0.0f ), yaw, 10.0f, 1.6f, 0.3f, lp_brick, LP_BRICK, 1.0f );
	}

	// A crane in the infield, and three cars spread round the ring, driving counterclockwise
	lpAddCrane( world, (lpVec3){ 0.0f, 0.0f, 0.0f }, 400.0f );
	lpAddCar( world, lpRingPoint( r, 0.0f, 0.0f ), -0.0f, 0 );
	lpAddCar( world, lpRingPoint( r, 2.1f, 0.0f ), -2.1f, 1 );
	lpAddCar( world, lpRingPoint( r, 4.5f, 0.0f ), -4.5f, 2 );
}

// Each car steers for a point a little ahead on the ring and holds the track speed
static void lpDriveTrack( lpWorld* world )
{
	int count = lpWorld_GetVehicleCapacity( world );
	for ( int v = 0; v < count; ++v )
	{
		lpVehicleState s = lpWorld_GetVehicleState( world, v );
		if ( s.controller >= 0 || s.alive == false || s.body < 0 )
		{
			continue;
		}
		float angle = lpAtan2( (float)s.position.z, (float)s.position.x );
		lpVec3 target = lpRingPoint( LP_TRACK_RADIUS, angle + LP_TRACK_LOOKAHEAD / LP_TRACK_RADIUS, 0.0f );
		float tx = target.x - (float)s.position.x;
		float tz = target.z - (float)s.position.z;
		// Signed angle from the car's heading to the target about +y: positive is to the left
		float error = lpAtan2( s.forward.z * tx - s.forward.x * tz, s.forward.x * tx + s.forward.z * tz );
		lpCommand c = { 0 };
		c.kind = lp_commandVehicleControl;
		c.vehicleControl.vehicle = v;
		c.vehicleControl.control.steer = lpClampFloat( -2.0f * error, -1.0f, 1.0f );
		c.vehicleControl.control.throttle = lpClampFloat( 0.5f * ( LP_TRACK_SPEED - s.speed ), -1.0f, 1.0f );
		lpSceneSubmit( world, &c );
	}
}

// ---- the mech yard: a hexapod on patrol over rough ground ----

#define LP_MECH_LOOKAHEAD 5.0f // m along the patrol the walkers steer for

// The patrol: a loop round the yard, north up x = 0, east, south down x = 16, west
static const lpVec3 lp_mechPatrol[4] = { { 0.0f, 0.0f, -20.0f }, { 0.0f, 0.0f, 20.0f }, { 16.0f, 0.0f, 20.0f }, { 16.0f, 0.0f, -20.0f } };

// A contraption that runs on its own: a spiral of wooden dominoes, the first leaning past its tipping point as the
// scene starts, runs outward and ends in six dominoes each 15% taller than the last; the tallest comes down on a volatile
// vial, which goes off against a brick wall. Nothing drives it: the run is
// physics alone, waking each frozen domino as it strikes it, so it is the outcome catalogue's stand-in for a trap set
// well ahead (and for physics left running far away). The vial's part has userId lp_userContraptionVial.
static lpVec3 lpDomino( lpWorld* world, lpVec3 at, lpVec3 travel, float height, bool lean, uint32_t color )
{
	lpVec3 half = { 0.065f * height, 0.5f * height, 0.25f * height }; // thin along the run
	lpQuat q = lpYaw( lpAtan2( -travel.z, travel.x ) );
	if ( lean )
	{
		q = lpMulQuat( lpMakeQuatFromAxisAngle( lpCross( (lpVec3){ 0.0f, 1.0f, 0.0f }, travel ), 0.25f ), q );
	}
	lpBegin();
	lpBox( lpVec3_zero, half, lpQuat_identity, lp_wood, color, false );
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	lpCommitDef( world, (lpVec3){ at.x, half.y + ( lean ? 0.01f : 0.0f ), at.z }, q, def );
	return at;
}

static void lpAddContraption( lpWorld* world )
{
	lpAddGround( world, 50.0f );

	// The run: an Archimedean spiral r = r0 + k theta walked outward at a fixed spacing, each domino facing along it, so
	// what comes after its last domino stands clear of it
	// Giant dominoes: a frozen one wakes only when struck at lpWorldDef.wakeSpeed (1.5 m/s), and a falling domino strikes at
	// about sqrt(g height): 1.2 m tall, they strike at 2.3 m/s
	const int count = 70;
	const float tall = 1.2f, r0 = 2.6f, k = 0.21f, spacing = 0.75f * tall; // rings 1.3 m apart
	float theta = 0.0f;
	lpVec3 travel = lpVec3_zero;
	lpVec3 last = lpVec3_zero;
	for ( int i = 0; i < count; ++i )
	{
		float r = r0 + k * theta;
		lpCosSin cs = lpComputeCosSin( theta );
		travel = lpNormalize( (lpVec3){ k * cs.cosine - r * cs.sine, 0.0f, k * cs.sine + r * cs.cosine } );
		last = lpDomino( world, (lpVec3){ r * cs.cosine, 0.0f, r * cs.sine }, travel, tall, i == 0, i % 2 == 0 ? LP_PLANK : LP_BEAM );
		theta += spacing / sqrtf( r * r + k * k );
	}

	// Six dominoes, each 15% taller, on along the last direction: a domino amplifies (a step much larger stalls, as the
	// run reaches each domino frozen and pushes it over from rest)
	float height = tall;
	for ( int i = 0; i < 6; ++i )
	{
		last = lpMulAdd( last, 0.75f * height, travel );
		height *= 1.15f;
		lpDomino( world, last, travel, height, false, i % 2 == 0 ? LP_BEAM : LP_PLANK );
	}

	// The vial where the tallest one's top comes down (at about 8 m/s)
	lpVec3 pivot = lpMulAdd( last, 0.065f * height, travel );
	lpQuat facing = lpYaw( lpAtan2( -travel.z, travel.x ) );
	lpBegin();
	lpPartDef* vial = lpBox( lpVec3_zero, (lpVec3){ 0.07f, 0.09f, 0.07f }, lpQuat_identity, lp_glass, 0x6FD68Au, false );
	vial->detonator.triggerSpeed = 4.0f; // where collisions start to register (lpWorldDef.hitSpeed)
	vial->detonator.radius = 1.8f;
	vial->detonator.energy = 120000.0f;
	vial->detonator.speed = 12.0f;
	lpObjectDef vialDef = lpDefaultObjectDef();
	vialDef.isStatic = false;
	vialDef.userId = lp_userContraptionVial;
	lpVec3 vialAt = lpMulAdd( pivot, 0.9f * height, travel );
	lpCommitDef( world, (lpVec3){ vialAt.x, 0.09f, vialAt.z }, facing, vialDef );

	// The wall the vial blows a hole in
	lpVec3 wall = lpMulAdd( (lpVec3){ vialAt.x, 0.0f, vialAt.z }, 1.2f, travel );
	lpAddWall( world, wall, lpAtan2( -travel.z, travel.x ) + 0.5f * LP_PI, 4.0f, 2.4f, 0.3f, lp_brick, LP_BRICK, 1.6f );
}

static void lpAddMechYard( lpWorld* world )
{
	lpAddGround( world, 50.0f );
	uint64_t rng = 0x3EC4ull;

	// North up x = 0: a 0.4 m concrete step, loose rubble, a hump up and down 15 degrees
	lpBegin();
	lpBox( (lpVec3){ 0.0f, 0.2f, 0.0f }, (lpVec3){ 5.0f, 0.2f, 3.0f }, lpQuat_identity, lp_concrete, LP_CONCRETE, true );
	lpCommit( world, (lpVec3){ 0.0f, 0.0f, -9.0f }, 0.0f, true );
	for ( int k = 0; k < 14; ++k )
	{
		float size = 0.12f + 0.16f * lpUnit( &rng );
		lpVec3 at; // one draw per statement
		at.x = -3.0f + 6.0f * lpUnit( &rng );
		at.y = size;
		at.z = -2.0f + 4.0f * lpUnit( &rng );
		lpBegin();
		lpBox( lpVec3_zero, (lpVec3){ size, 0.8f * size, 1.2f * size }, lpQuat_identity, lp_stone, LP_STONE_DARK, false );
		lpCommit( world, at, 6.28f * lpUnit( &rng ), false );
	}
	lpBegin();
	lpWedge( 5.0f, 6.0f, 10.0f, 1.07f, lp_stone, LP_STONE );
	lpBox( (lpVec3){ 0.0f, 0.535f, 11.0f }, (lpVec3){ 5.0f, 0.535f, 1.0f }, lpQuat_identity, lp_stone, LP_STONE, true );
	lpWedge( 5.0f, 16.0f, 12.0f, 1.07f, lp_stone, LP_STONE );
	lpCommit( world, lpVec3_zero, 0.0f, true );

	// East along z = 20: loose crates to wade through
	for ( int k = 0; k < 7; ++k )
	{
		lpBegin();
		lpBox( lpVec3_zero, (lpVec3){ 0.4f, 0.4f, 0.4f }, lpQuat_identity, lp_wood, LP_PLANK, false );
		lpCommit( world, (lpVec3){ 6.0f + 0.9f * (float)( k % 4 ), 0.4f + 0.8f * (float)( k / 4 ), 19.0f + 0.7f * (float)( k % 3 ) },
				  0.4f * (float)k, false );
	}

	// South down x = 16: a brick wall on its right, a parked car on its left
	lpAddWall( world, (lpVec3){ 12.5f, 0.0f, 0.0f }, 0.5f * LP_PI, 8.0f, 2.0f, 0.3f, lp_brick, LP_BRICK, 1.0f );
	lpAddCar( world, (lpVec3){ 20.5f, 0.0f, -8.0f }, 0.0f, 3 );

	// The mech, at the start of the patrol, facing north
	lpAddHexapod( world, (lpVec3){ 0.0f, 0.0f, -24.0f }, 0.0f, 0 );
}

// Each walker steers for a point a little further round the patrol than the nearest point on it, slowing to turn
static void lpDriveMech( lpWorld* world )
{
	for ( int ri = 0; ri < lpWorld_GetRigCapacity( world ); ++ri )
	{
		lpRigState s = lpWorld_GetRigState( world, ri );
		if ( s.controller >= 0 || s.alive == false || s.body < 0 )
		{
			continue;
		}
		lpVec3 p = { (float)s.position.x, 0.0f, (float)s.position.z };
		int segment = 0;
		float along = 0.0f, nearest = FLT_MAX;
		for ( int k = 0; k < 4; ++k )
		{
			lpVec3 a = lp_mechPatrol[k], b = lp_mechPatrol[( k + 1 ) % 4];
			lpVec3 ab = lpSub( b, a );
			float t = lpClampFloat( lpDot( lpSub( p, a ), ab ) / lpDot( ab, ab ), 0.0f, 1.0f );
			float d = lpLength( lpSub( p, lpMulAdd( a, t, ab ) ) );
			if ( d < nearest )
			{
				nearest = d;
				segment = k;
				along = t * lpLength( ab );
			}
		}
		// Walk the lookahead along the loop
		float left = along + LP_MECH_LOOKAHEAD;
		lpVec3 target = lp_mechPatrol[segment];
		for ( int k = 0; k < 4; ++k )
		{
			lpVec3 a = lp_mechPatrol[( segment + k ) % 4], b = lp_mechPatrol[( segment + k + 1 ) % 4];
			float length = lpLength( lpSub( b, a ) );
			if ( left <= length )
			{
				target = lpMulAdd( a, left / length, lpSub( b, a ) );
				break;
			}
			left -= length;
		}
		float tx = target.x - p.x, tz = target.z - p.z;
		// Signed angle from the heading to the target about +y: positive is to the left
		float error = lpAtan2( s.forward.z * tx - s.forward.x * tz, s.forward.x * tx + s.forward.z * tz );
		lpCommand c = { 0 };
		c.kind = lp_commandRigControl;
		c.rigControl.rig = ri;
		c.rigControl.control.turn = lpClampFloat( -1.5f * error, -1.0f, 1.0f );
		c.rigControl.control.forward = lpClampFloat( 1.0f - lpAbsFloat( error ), 0.2f, 1.0f );
		lpSceneSubmit( world, &c );
	}
}

void lpSceneDrive( lpWorld* world, int scene, int tick )
{
	if ( scene == lp_sceneTrack )
	{
		lpDriveTrack( world );
		lpDriveCrane( world, tick );
	}
	else if ( scene == lp_sceneMech )
	{
		lpDriveMech( world );
	}
}

static void lpAddPile( lpWorld* world, lpVec3 center, int count, uint64_t seed )
{
	uint64_t rng = seed;
	int side = (int)ceilf( lpCbrt( (float)count ) );
	int n = 0;
	for ( int y = 0; n < count; ++y )
	{
		for ( int x = 0; x < side && n < count; ++x )
		{
			for ( int z = 0; z < side && n < count; ++z, ++n )
			{
				lpBegin();
				lpVec3 p = { center.x + ( (float)x - 0.5f * (float)side ) * 0.9f, center.y + 1.0f + (float)y * 0.9f,
							 center.z + ( (float)z - 0.5f * (float)side ) * 0.9f };
				if ( lpNext( &rng ) % 3u == 0u )
				{
					lpVec3 pts[12];
					for ( int i = 0; i < 12; ++i )
					{
						lpVec3 d; // one draw per statement
						d.x = lpUnit( &rng ) - 0.5f;
						d.y = lpUnit( &rng ) - 0.5f;
						d.z = lpUnit( &rng ) - 0.5f;
						d = lpNormalize( d );
						pts[i] = lpMulSV( 0.35f * ( 0.8f + 0.4f * lpUnit( &rng ) ), d );
					}
					lpHull( pts, 12, lp_stone, LP_STONE, false );
				}
				else
				{
					lpBox( lpVec3_zero, (lpVec3){ 0.35f, 0.35f, 0.35f }, lpQuat_identity, lp_wood, LP_PLANK, false );
				}
				lpCommit( world, p, 6.2831853f * lpUnit( &rng ), false );
			}
		}
	}
}

// The tools' throwables, as templates: lp_templateFlask, then lp_templateBall
static void lpAddToolTemplates( lpWorld* world )
{
	// A chunky hexagonal bottle with a neck: it goes off when it lands hard
	lpVec3 points[20];
	for ( int i = 0; i < 6; ++i )
	{
		lpCosSin cs = lpComputeCosSin( 1.0471976f * (float)i );
		points[i] = (lpVec3){ 0.09f * cs.cosine, -0.12f, 0.09f * cs.sine };
		points[6 + i] = (lpVec3){ 0.09f * cs.cosine, 0.06f, 0.09f * cs.sine };
	}
	points[12] = (lpVec3){ 0.04f, 0.2f, 0.0f };
	points[13] = (lpVec3){ -0.03f, 0.2f, 0.035f };
	points[14] = (lpVec3){ -0.03f, 0.2f, -0.035f };
	lpPartDef part = lpDefaultPartDef();
	part.points = points;
	part.pointCount = 15;
	part.material = lp_glass;
	part.color = 0x6FD68Au;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.parts = &part;
	def.partCount = 1;
	def.detonator.triggerSpeed = 4.5f;
	def.detonator.radius = 1.8f;
	def.detonator.energy = 120000.0f;
	def.detonator.speed = 12.0f;
	lpWorld_AddTemplate( world, &def );

	// A metal cannonball, a chunky low-poly sphere of golden-spiral points
	for ( int i = 0; i < 20; ++i )
	{
		float y = 1.0f - 2.0f * ( (float)i + 0.5f ) / 20.0f;
		float r = sqrtf( 1.0f - y * y );
		float a = 2.39996323f * (float)i;
		lpCosSin cs = lpComputeCosSin( a );
		points[i] = (lpVec3){ 0.3f * r * cs.cosine, 0.3f * y, 0.3f * r * cs.sine };
	}
	part = lpDefaultPartDef();
	part.points = points;
	part.pointCount = 20;
	part.material = lp_metal;
	part.color = 0x3A3D42u;
	def = lpDefaultObjectDef();
	def.isStatic = false;
	def.parts = &part;
	def.partCount = 1;
	lpWorld_AddTemplate( world, &def );
}

void lpBuildScene( lpWorld* world, int scene )
{
	lpAddToolTemplates( world );
	switch ( scene )
	{
		case lp_sceneWall:
		{
			lpAddGround( world, 60.0f );
			lpAddWall( world, (lpVec3){ 0.0f, 0.0f, -4.0f }, 0.0f, 8.0f, 3.2f, 0.3f, lp_brick, LP_BRICK, 1.6f );
			lpAddWall( world, (lpVec3){ 0.0f, 0.0f, -9.0f }, 0.0f, 10.0f, 4.0f, 0.6f, lp_stone, LP_STONE, 2.0f );
			lpAddWall( world, (lpVec3){ -7.0f, 0.0f, -2.0f }, 0.5f * LP_PI, 5.0f, 2.6f, 0.25f, lp_plaster, LP_PLASTER, 1.5f );
			lpAddFence( world, (lpVec3){ 5.0f, 0.0f, 0.0f }, 0.5f * LP_PI, 7.0f );
			lpBegin();
			lpBox( (lpVec3){ 0.0f, 1.6f, 0.0f }, (lpVec3){ 1.0f, 0.8f, 0.02f }, lpQuat_identity, lp_glass, LP_GLASS, true );
			lpBox( (lpVec3){ 0.0f, 0.4f, 0.0f }, (lpVec3){ 1.1f, 0.4f, 0.1f }, lpQuat_identity, lp_wood, LP_BEAM, true );
			lpCommit( world, (lpVec3){ -3.0f, 0.0f, 1.0f }, 0.0f, true );
			lpAddTree( world, (lpVec3){ 8.0f, 0.0f, -6.0f }, 5.0f, 7u );
			break;
		}

		case lp_sceneHouse:
			lpAddGround( world, 60.0f );
			lpAddHouse( world, (lpVec3){ 0.0f, 0.0f, -6.0f }, 0.0f, 3u );
			lpAddTree( world, (lpVec3){ 7.0f, 0.0f, -4.0f }, 5.5f, 11u );
			lpAddFence( world, (lpVec3){ 0.0f, 0.0f, 0.5f }, 0.0f, 9.0f );
			break;

		case lp_sceneTown:
		{
			lpAddGround( world, 120.0f );
			uint64_t rng = 42;
			for ( int i = 0; i < 6; ++i )
			{
				float x = -27.0f + 11.0f * (float)i;
				lpAddHouse( world, (lpVec3){ x, 0.0f, -10.0f }, 0.0f, lpNext( &rng ) );
				lpAddHouse( world, (lpVec3){ x + 3.0f, 0.0f, 10.0f }, LP_PI, lpNext( &rng ) );
				uint64_t treeSeed = lpNext( &rng ); // drawn first, as MSVC evaluated the call's arguments (C leaves it open)
				float treeHeight = 4.5f + 2.0f * lpUnit( &rng );
				lpAddTree( world, (lpVec3){ x + 5.5f, 0.0f, -4.5f }, treeHeight, treeSeed );
				lpAddFence( world, (lpVec3){ x - 1.0f, 0.0f, 4.5f }, 0.0f, 6.0f );
			}
			lpAddTowerAt( world, (lpVec3){ 40.0f, 0.0f, 0.0f }, 7 );
			break;
		}

		case lp_sceneTower:
			lpAddGround( world, 60.0f );
			lpAddTowerAt( world, (lpVec3){ 0.0f, 0.0f, -8.0f }, 9 );
			lpAddTree( world, (lpVec3){ 7.0f, 0.0f, -5.0f }, 5.0f, 5u );
			break;

		case lp_scenePile:
			lpAddGround( world, 60.0f );
			lpAddPile( world, (lpVec3){ 0.0f, 0.0f, -6.0f }, 512, 9u );
			break;

		case lp_sceneRuins:
			lpAddGround( world, 60.0f );
			lpAddArch( world, (lpVec3){ -7.0f, 0.0f, -6.0f }, 2.0f, 0.5f, 1.0f, 9 );
			lpAddColonnade( world, (lpVec3){ 0.0f, 0.0f, -6.0f }, 4, 2.6f, 3.0f );
			lpAddBalconies( world, (lpVec3){ 12.0f, 0.0f, -6.0f } );
			break;

		case lp_sceneYard:
			lpAddGround( world, 60.0f );
			lpAddCartOnRamp( world, -8.0f );
			lpAddWall( world, (lpVec3){ -3.2f, 0.0f, -8.0f }, 0.5f * LP_PI, 3.0f, 1.4f, 0.3f, lp_brick, LP_BRICK, 1.0f );
			lpAddPorterRack( world, (lpVec3){ -0.6f, 0.0f, -6.0f } );
			lpAddHangingSign( world, (lpVec3){ 1.5f, 0.0f, -8.0f } );
			lpAddDoorway( world, (lpVec3){ 4.7f, 0.0f, -8.0f } );
			lpAddDrawbridge( world, (lpVec3){ 7.9f, 0.0f, -8.0f } );
			break;

		case lp_sceneKeep:
			lpAddGround( world, 60.0f );
			lpAddKeep( world, (lpVec3){ 0.0f, 0.0f, -10.0f }, 4 );
			break;

		case lp_sceneTrack:
			lpAddTrack( world );
			break;

		case lp_sceneMech:
			lpAddMechYard( world );
			break;

		case lp_sceneContraption:
			lpAddContraption( world );
			break;

		case lp_sceneLumber:
		{
			lpAddGround( world, 60.0f );
			// A log bridge over two stumps, the first thing in view
			lpAddStump( world, (lpVec3){ -2.2f, 0.0f, -3.0f }, 0.8f, 0.35f );
			lpAddStump( world, (lpVec3){ 2.2f, 0.0f, -3.0f }, 0.8f, 0.35f );
			lpAddLog( world, (lpVec3){ 0.0f, 1.05f, -3.0f }, 0.0f, 5.2f, 0.24f, false );
			// Logs lying about
			lpAddLog( world, (lpVec3){ -4.5f, 0.3f, 0.5f }, 0.4f, 4.0f, 0.3f, false );
			lpAddLog( world, (lpVec3){ 4.8f, 0.25f, 0.0f }, -0.3f, 3.2f, 0.25f, false );
			// A woodpile
			for ( int row = 0; row < 3; ++row )
			{
				for ( int k = 0; k < 4 - row; ++k )
				{
					float x = 6.0f + 0.42f * ( (float)k - 0.5f * (float)( 3 - row ) );
					lpAddLog( world, (lpVec3){ x, 0.2f + 0.36f * (float)row, -7.0f }, 0.5f * LP_PI, 2.0f, 0.2f, false );
				}
			}
			lpAddShed( world, (lpVec3){ -6.0f, 0.0f, -8.0f }, 0.3f );
			lpAddTree( world, (lpVec3){ 1.0f, 0.0f, -10.0f }, 6.0f, 21u );
			lpAddTree( world, (lpVec3){ 9.0f, 0.0f, -4.0f }, 5.0f, 22u );
			lpAddTree( world, (lpVec3){ -10.0f, 0.0f, -3.0f }, 5.5f, 23u );
			lpAddFence( world, (lpVec3){ 0.0f, 0.0f, 3.5f }, 0.0f, 10.0f );
			break;
		}

		default:
			lpAddGround( world, 60.0f );
			break;
	}
	lpWorld_SettleStructures( world ); // every structure starts converged instead of creaking through its first seconds
}

bool lpSceneBombard( lpWorld* world, int scene, int tick, int period )
{
	if ( period <= 0 || tick % period != period / 2 )
	{
		return false;
	}
	int shot = tick / period;
	uint64_t rng = 0xB0B0ull + (uint64_t)shot * 7919ull;

	lpVec3 origin;
	lpVec3 target;
	switch ( scene )
	{
		case lp_sceneTown:
		{
			// Walk down the street firing at both rows of houses and the tower at the end
			float x = -30.0f + (float)( ( shot * 7 ) % 75 );
			origin = (lpVec3){ x, 1.8f, 0.0f };
			float side = ( shot % 2 == 0 ) ? -1.0f : 1.0f;
			target.x = x + 6.0f * ( lpUnit( &rng ) - 0.5f ); // one draw per statement
			target.y = 0.4f + 5.0f * lpUnit( &rng );
			target.z = side * 12.0f;
			if ( x > 34.0f )
			{
				target.x = 40.0f;
				target.y = 0.5f + 8.0f * lpUnit( &rng );
				target.z = 2.0f * ( lpUnit( &rng ) - 0.5f );
			}
			break;
		}
		case lp_sceneRuins:
		{
			// Across the whole row: the arch, the colonnade and the balcony wall
			origin = (lpVec3){ 2.0f + 8.0f * ( lpUnit( &rng ) - 0.5f ), 1.8f, 10.0f };
			target.x = -10.0f + 24.0f * lpUnit( &rng );
			target.y = 0.3f + 3.7f * lpUnit( &rng );
			target.z = -6.0f;
			break;
		}
		case lp_sceneYard:
		{
			// Across the whole yard, from the cart's ramp to the gatehouse
			origin = (lpVec3){ -1.0f + 8.0f * ( lpUnit( &rng ) - 0.5f ), 1.8f, 10.0f };
			target.x = -11.0f + 20.0f * lpUnit( &rng );
			target.y = 0.3f + 3.2f * lpUnit( &rng );
			target.z = -8.0f;
			break;
		}
		case lp_sceneKeep:
		{
			// Across the keep's front, from the foot of the wall to the parapet
			origin = (lpVec3){ 8.0f * ( lpUnit( &rng ) - 0.5f ), 1.8f, 14.0f };
			target.x = -7.0f + 14.0f * lpUnit( &rng );
			target.y = 0.3f + 12.5f * lpUnit( &rng );
			target.z = -2.5f;
			break;
		}
		case lp_sceneMech:
		{
			// At the walker's legs in turn, from beside it (at the patrol's start once it is gone)
			lpRigState rig = lpWorld_GetRigState( world, 0 );
			lpLimbState leg = lpWorld_GetLimbState( world, 0, shot % 6 );
			lpVec3 at = { 0.0f, 0.8f, -20.0f };
			if ( leg.footBody >= 0 )
			{
				at = (lpVec3){ (float)leg.foot.x, (float)leg.foot.y + 0.9f, (float)leg.foot.z };
			}
			else if ( rig.body >= 0 )
			{
				at = (lpVec3){ (float)rig.position.x, (float)rig.position.y, (float)rig.position.z };
			}
			target.x = at.x + 0.4f * ( lpUnit( &rng ) - 0.5f );
			target.y = at.y + 0.3f * ( lpUnit( &rng ) - 0.5f );
			target.z = at.z + 0.4f * ( lpUnit( &rng ) - 0.5f );
			float side = shot % 2 == 0 ? 1.0f : -1.0f;
			origin = lpAdd( target, (lpVec3){ 9.0f * side, 0.8f, 3.0f * ( lpUnit( &rng ) - 0.5f ) } );
			break;
		}
		case lp_sceneTrack:
		{
			// At the cars in turn, from the infield (at the road ahead of a car that is gone)
			int count = lpWorld_GetVehicleCapacity( world );
			lpVehicleState car = lpWorld_GetVehicleState( world, count > 0 ? shot % count : 0 );
			lpVec3 at = car.body >= 0 ? lpVec3_zero : lpRingPoint( LP_TRACK_RADIUS, 0.4f * (float)shot, 0.5f );
			if ( car.body >= 0 )
			{
				at = (lpVec3){ (float)car.position.x, (float)car.position.y, (float)car.position.z };
			}
			target.x = at.x + 2.0f * ( lpUnit( &rng ) - 0.5f );
			target.y = at.y + 0.3f * lpUnit( &rng );
			target.z = at.z + 2.0f * ( lpUnit( &rng ) - 0.5f );
			origin = (lpVec3){ 0.7f * target.x, 1.8f, 0.7f * target.z };
			break;
		}
		default:
		{
			origin = (lpVec3){ 6.0f * ( lpUnit( &rng ) - 0.5f ), 1.8f, 10.0f };
			target.x = 12.0f * ( lpUnit( &rng ) - 0.5f );
			target.y = 0.3f + 4.0f * lpUnit( &rng );
			target.z = -8.0f;
			break;
		}
	}

	lpVec3 dir = lpNormalize( lpSub( target, origin ) );
	lpCommand c = { 0 };
	c.kind = lp_commandImpact;
	c.impact.origin = origin;
	c.impact.range = 60.0f;
	c.impact.piecesOnly = true;
	lpImpactDef impact = { 0 };
	impact.direction = dir;
	if ( shot % 4 == 3 && scene != lp_sceneMech ) // the mech's legs take grenades only
	{
		impact.radius = 2.0f;
		impact.energy = 250000.0f;
		impact.impulse = 18.0f;
	}
	else
	{
		impact.radius = 1.3f;
		impact.energy = 70000.0f;
		impact.impulse = 12.0f;
	}
	impact.explosion = true;
	c.impact.def = impact;
	lpSceneSubmit( world, &c );
	return true;
}
