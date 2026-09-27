// SPDX-License-Identifier: MIT

#include "scenes.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

enum
{
	lp_maxParts = 768,
	lp_maxPoints = 8192
};

// Collects convex parts of one object, then creates it
typedef struct lpBuilder
{
	lpPartDef parts[lp_maxParts];
	int partCount;
	b3Vec3 points[lp_maxPoints];
	int pointCount;
} lpBuilder;

static lpBuilder lp_builder;

static void lpBegin( void )
{
	lp_builder.partCount = 0;
	lp_builder.pointCount = 0;
}

static lpPartDef* lpBox( b3Vec3 center, b3Vec3 half, b3Quat q, int material, uint32_t color, bool anchored )
{
	if ( lp_builder.partCount == lp_maxParts )
	{
		return NULL;
	}
	lpPartDef* part = lp_builder.parts + lp_builder.partCount++;
	*part = lpDefaultPartDef();
	part->halfExtents = half;
	part->transform = (b3Transform){ center, q };
	part->material = (uint8_t)material;
	part->color = color;
	part->anchored = anchored;
	return part;
}

static lpPartDef* lpHull( const b3Vec3* points, int count, int material, uint32_t color, bool anchored )
{
	if ( lp_builder.partCount == lp_maxParts || lp_builder.pointCount + count > lp_maxPoints )
	{
		return NULL;
	}
	b3Vec3* dst = lp_builder.points + lp_builder.pointCount;
	memcpy( dst, points, sizeof( b3Vec3 ) * (size_t)count );
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

static int lpCommit( lpWorld* world, b3Vec3 position, float yaw, bool isStatic )
{
	lpObjectDef def = lpDefaultObjectDef();
	def.transform.p = position;
	def.transform.q = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, yaw );
	def.isStatic = isStatic;
	def.parts = lp_builder.parts;
	def.partCount = lp_builder.partCount;
	return lpCreateObject( world, &def );
}

static b3Quat lpYaw( float angle )
{
	return b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, angle );
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
	lpBox( (b3Vec3){ 0.0f, -0.5f, 0.0f }, (b3Vec3){ halfSize, 0.5f, halfSize }, b3Quat_identity, lp_ground, LP_GRASS, true );
	lpCommit( world, b3Vec3_zero, 0.0f, true );
}

// Wall segment along local x in [0, length], split into panels and rows. Panels in the bottom row are anchored.
static void lpWallPanels( b3Vec3 origin, float length, float y0, float y1, float thickness, int material, uint32_t color,
						  float panelWidth, bool anchorBottom )
{
	int columns = (int)ceilf( length / panelWidth );
	columns = columns < 1 ? 1 : columns;
	float height = y1 - y0;
	int rows = (int)ceilf( height / 1.6f );
	rows = rows < 1 ? 1 : rows;
	float w = length / (float)columns;
	float h = height / (float)rows;
	for ( int r = 0; r < rows; ++r )
	{
		for ( int c = 0; c < columns; ++c )
		{
			b3Vec3 center = { origin.x + ( (float)c + 0.5f ) * w, origin.y + y0 + ( (float)r + 0.5f ) * h, origin.z };
			lpBox( center, (b3Vec3){ 0.5f * w, 0.5f * h, 0.5f * thickness }, b3Quat_identity, material, color,
				   anchorBottom && r == 0 && y0 <= 0.001f );
		}
	}
}

static void lpAddWall( lpWorld* world, b3Vec3 base, float yaw, float length, float height, float thickness, int material,
				uint32_t color, float panelWidth )
{
	lpBegin();
	lpWallPanels( (b3Vec3){ -0.5f * length, 0.0f, 0.0f }, length, 0.0f, height, thickness, material, color, panelWidth, true );
	lpCommit( world, base, yaw, true );
}

typedef struct lpOpening
{
	float x0, x1, y0, y1;
	bool glass;
} lpOpening;

// A wall along local x from 0 to length at depth z, with openings. The solid parts are decomposed into boxes:
// full-height strips between openings, plus a sill and a lintel for each opening.
static void lpWallWithOpenings( b3Quat q, b3Vec3 origin, float length, float y0, float y1, float thickness, int material,
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
	b3Vec3 ax = b3RotateVector( q, (b3Vec3){ 1.0f, 0.0f, 0.0f } );
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
					b3Vec3 center = b3Add( origin, b3MulSV( cx, ax ) );
					center.y = y0 + ( (float)r + 0.5f ) * h;
					lpBox( center, (b3Vec3){ 0.5f * w, 0.5f * h, 0.5f * thickness }, q, material, color, anchorBottom && r == 0 );
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
		b3Vec3 center = b3Add( origin, b3MulSV( cx, ax ) );
		if ( o->y0 - y0 > 0.05f )
		{
			center.y = 0.5f * ( y0 + o->y0 );
			lpBox( center, (b3Vec3){ hw, 0.5f * ( o->y0 - y0 ), 0.5f * thickness }, q, material, color, anchorBottom );
		}
		if ( y1 - o->y1 > 0.05f )
		{
			center.y = 0.5f * ( o->y1 + y1 );
			lpBox( center, (b3Vec3){ hw, 0.5f * ( y1 - o->y1 ), 0.5f * thickness }, q, material, color, false );
		}
		if ( o->glass )
		{
			center.y = 0.5f * ( o->y0 + o->y1 );
			lpBox( center, (b3Vec3){ hw, 0.5f * ( o->y1 - o->y0 ), 0.02f }, q, lp_glass, LP_GLASS, false );
		}
		x = o->x1;
	}
}

static void lpAddHouse( lpWorld* world, b3Vec3 base, float yaw, uint64_t seed )
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
	lpBox( (b3Vec3){ 0.0f, 0.1f, 0.0f }, (b3Vec3){ hw + 0.1f, 0.1f, hd + 0.1f }, b3Quat_identity, lp_concrete, LP_CONCRETE, true );

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
		lpWallWithOpenings( b3Quat_identity, (b3Vec3){ -hw + t, 0.0f, hd - 0.5f * t }, innerW, y0, y1, t, wallMaterial,
							wallColor, front, frontCount, s == 0 );

		lpOpening back[2] = {
			{ 1.0f, 2.0f, y0 + 0.9f, y0 + 2.0f, true },
			{ innerW - 2.0f, innerW - 1.0f, y0 + 0.9f, y0 + 2.0f, true },
		};
		lpWallWithOpenings( b3Quat_identity, (b3Vec3){ -hw + t, 0.0f, -hd + 0.5f * t }, innerW, y0, y1, t, wallMaterial,
							wallColor, back, 2, s == 0 );

		// Side walls (along z) span the full depth
		lpOpening side[1] = { { 0.5f * depth - 0.6f, 0.5f * depth + 0.6f, y0 + 0.9f, y0 + 2.0f, true } };
		b3Quat sideQ = lpYaw( -0.5f * B3_PI );
		lpWallWithOpenings( sideQ, (b3Vec3){ -hw + 0.5f * t, 0.0f, -hd }, depth, y0, y1, t, wallMaterial, wallColor, side, 1, s == 0 );
		lpWallWithOpenings( sideQ, (b3Vec3){ hw - 0.5f * t, 0.0f, -hd }, depth, y0, y1, t, wallMaterial, wallColor, side, 1, s == 0 );

		// Floor slab / ceiling: wooden planks resting on the walls
		float ceilingY = y1 + 0.1f;
		int planks = (int)( depth / 0.8f );
		float pw = depth / (float)planks;
		for ( int p = 0; p < planks; ++p )
		{
			float z = -hd + ( (float)p + 0.5f ) * pw;
			lpBox( (b3Vec3){ 0.0f, ceilingY, z }, (b3Vec3){ hw, 0.1f, 0.5f * pw }, b3Quat_identity, lp_wood, LP_PLANK, false );
		}
	}

	// Gable roof: ridge along x
	float topY = floorY + (float)stories * ( storyHeight + 0.2f );
	float rise = 1.6f;
	float slope = atan2f( rise, hd );
	float slant = sqrtf( rise * rise + hd * hd );
	int roofSegments = (int)ceilf( width / 1.6f );
	float segW = ( width + 0.6f ) / (float)roofSegments;
	for ( int side = -1; side <= 1; side += 2 )
	{
		b3Quat q = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, (float)side * slope );
		for ( int k = 0; k < roofSegments; ++k )
		{
			float x = -0.5f * ( width + 0.6f ) + ( (float)k + 0.5f ) * segW;
			b3Vec3 center = { x, topY + 0.5f * rise + 0.08f, (float)side * 0.5f * hd };
			lpBox( center, (b3Vec3){ 0.5f * segW, 0.08f, 0.5f * slant + 0.25f }, q, lp_wood, roofColor, false );
		}
	}
	// Gable ends: triangular prisms
	for ( int side = -1; side <= 1; side += 2 )
	{
		float x = (float)side * ( hw - 0.5f * t );
		b3Vec3 tri[6] = {
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
			lpBox( (b3Vec3){ (float)sx * ( hw + 0.08f ), floorY + 0.5f * h, (float)sz * ( hd + 0.08f ) }, (b3Vec3){ 0.1f, 0.5f * h, 0.1f },
				   b3Quat_identity, lp_wood, LP_BEAM, false );
		}
	}

	lpCommit( world, base, yaw, true );
}

static int lpRing( b3Vec3* out, b3Vec3 center, float radius, int sides, float phase )
{
	for ( int i = 0; i < sides; ++i )
	{
		float a = phase + 2.0f * B3_PI * (float)i / (float)sides;
		b3CosSin cs = b3ComputeCosSin( a );
		out[i] = (b3Vec3){ center.x + radius * cs.cosine, center.y, center.z + radius * cs.sine };
	}
	return sides;
}

static void lpAddTree( lpWorld* world, b3Vec3 base, float height, uint64_t seed )
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
		b3Vec3 pts[16];
		lpRing( pts, (b3Vec3){ 0.0f, y0, 0.0f }, ra, 7, phase );
		lpRing( pts + 7, (b3Vec3){ 0.0f, y1, 0.0f }, rb, 7, phase );
		lpPartDef* part = lpHull( pts, 14, lp_wood, LP_BARK, s == 0 );
		if ( part != NULL )
		{
			part->grainAxis = (b3Vec3){ 0.0f, 1.0f, 0.0f };
			part->joint = lp_jointSolid; // one living tree, not timber nailed together
		}
	}

	// Canopy: a few chunky low-poly blobs
	int blobs = 2 + (int)( lpNext( &rng ) % 2u );
	b3Vec3 below = { 0.0f, trunkH - 0.3f, 0.0f }; // each blob reaches into what holds it up: the trunk, then the blob below
	for ( int b = 0; b < blobs; ++b )
	{
		float cr = 0.35f * height * ( 0.8f + 0.3f * lpUnit( &rng ) );
		b3Vec3 c = { 0.4f * ( lpUnit( &rng ) - 0.5f ), trunkH + cr * ( 0.4f + 0.9f * (float)b ), 0.4f * ( lpUnit( &rng ) - 0.5f ) };
		b3Vec3 pts[14];
		for ( int i = 0; i < 14; ++i )
		{
			b3Vec3 d = { lpUnit( &rng ) - 0.5f, 0.8f * ( lpUnit( &rng ) - 0.5f ), lpUnit( &rng ) - 0.5f };
			d = b3Normalize( d );
			pts[i] = b3MulAdd( c, cr * ( 0.85f + 0.3f * lpUnit( &rng ) ), d );
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
static void lpAddLog( lpWorld* world, b3Vec3 center, float yaw, float length, float radius, bool isStatic )
{
	lpBegin();
	b3Vec3 pts[16];
	for ( int i = 0; i < 8; ++i )
	{
		b3CosSin cs = b3ComputeCosSin( 0.3926991f + 0.7853982f * (float)i );
		pts[i] = (b3Vec3){ -0.5f * length, radius * cs.cosine, radius * cs.sine };
		pts[8 + i] = (b3Vec3){ 0.5f * length, radius * cs.cosine, radius * cs.sine };
	}
	lpPartDef* part = lpHull( pts, 16, lp_wood, LP_BARK, false );
	if ( part != NULL )
	{
		part->grainAxis = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	}
	lpCommit( world, center, yaw, isStatic );
}

static void lpAddStump( lpWorld* world, b3Vec3 base, float height, float radius )
{
	lpBegin();
	b3Vec3 pts[16];
	lpRing( pts, (b3Vec3){ 0.0f, 0.0f, 0.0f }, radius, 8, 0.2f );
	lpRing( pts + 8, (b3Vec3){ 0.0f, height, 0.0f }, 0.9f * radius, 8, 0.2f );
	lpPartDef* part = lpHull( pts, 16, lp_wood, LP_BARK, true );
	if ( part != NULL )
	{
		part->grainAxis = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	}
	lpCommit( world, base, 0.0f, true );
}

// A small plank shed: posts, horizontal wall planks, a lean-to roof
static void lpAddShed( lpWorld* world, b3Vec3 base, float yaw )
{
	lpBegin();
	float w = 4.0f, d = 3.0f, h = 2.4f;
	for ( int sx = -1; sx <= 1; sx += 2 )
	{
		for ( int sz = -1; sz <= 1; sz += 2 )
		{
			lpBox( (b3Vec3){ (float)sx * 0.5f * w, 0.5f * h, (float)sz * 0.5f * d }, (b3Vec3){ 0.08f, 0.5f * h, 0.08f },
				   b3Quat_identity, lp_wood, LP_BEAM, true );
		}
	}
	int rows = 8;
	float ph = h / (float)rows;
	for ( int r = 0; r < rows; ++r )
	{
		float y = ( (float)r + 0.5f ) * ph;
		uint32_t color = ( r % 2 ) ? LP_PLANK : 0x9A6A3Au;
		lpBox( (b3Vec3){ 0.0f, y, -0.5f * d }, (b3Vec3){ 0.5f * w - 0.08f, 0.5f * ph - 0.005f, 0.03f }, b3Quat_identity, lp_wood, color, r == 0 );
		lpBox( (b3Vec3){ -0.5f * w, y, 0.0f }, (b3Vec3){ 0.03f, 0.5f * ph - 0.005f, 0.5f * d - 0.08f }, b3Quat_identity, lp_wood, color, r == 0 );
		lpBox( (b3Vec3){ 0.5f * w, y, 0.0f }, (b3Vec3){ 0.03f, 0.5f * ph - 0.005f, 0.5f * d - 0.08f }, b3Quat_identity, lp_wood, color, r == 0 );
	}
	b3Quat tilt = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.18f );
	for ( int k = 0; k < 5; ++k )
	{
		float x = -0.5f * w - 0.2f + ( (float)k + 0.5f ) * ( w + 0.4f ) / 5.0f;
		lpBox( (b3Vec3){ x, h + 0.12f, 0.0f }, (b3Vec3){ 0.5f * ( w + 0.4f ) / 5.0f, 0.05f, 0.5f * d + 0.3f }, tilt, lp_wood, LP_ROOF, false );
	}
	lpCommit( world, base, yaw, true );
}

static void lpAddFence( lpWorld* world, b3Vec3 base, float yaw, float length )
{
	lpBegin();
	int posts = (int)( length / 1.8f ) + 1;
	float step = length / (float)( posts - 1 );
	for ( int i = 0; i < posts; ++i )
	{
		float x = -0.5f * length + (float)i * step;
		lpBox( (b3Vec3){ x, 0.55f, 0.0f }, (b3Vec3){ 0.06f, 0.55f, 0.06f }, b3Quat_identity, lp_wood, LP_BEAM, true );
	}
	for ( int i = 0; i + 1 < posts; ++i )
	{
		float x = -0.5f * length + ( (float)i + 0.5f ) * step;
		for ( int r = 0; r < 2; ++r )
		{
			lpBox( (b3Vec3){ x, 0.4f + 0.45f * (float)r, 0.1f }, (b3Vec3){ 0.5f * step, 0.06f, 0.04f }, b3Quat_identity, lp_wood,
				   LP_PLANK, false );
		}
	}
	lpCommit( world, base, yaw, true );
}

static void lpAddTowerAt( lpWorld* world, b3Vec3 base, int levels )
{
	lpBegin();
	int sides = 10;
	float radius = 2.6f;
	float t = 0.6f;
	float levelH = 1.4f;
	float side = 2.0f * radius * sinf( B3_PI / (float)sides ) + 0.02f;
	for ( int l = 0; l < levels; ++l )
	{
		float phase = ( l % 2 ) ? 0.5f * 2.0f * B3_PI / (float)sides : 0.0f;
		for ( int s = 0; s < sides; ++s )
		{
			float a = phase + 2.0f * B3_PI * (float)s / (float)sides;
			b3CosSin cs = b3ComputeCosSin( a );
			b3Vec3 c = { radius * cs.cosine, ( (float)l + 0.5f ) * levelH, radius * cs.sine };
			b3Quat q = lpYaw( -a + 0.5f * B3_PI );
			uint32_t color = ( ( l + s ) % 3 == 0 ) ? LP_STONE_DARK : LP_STONE;
			lpPartDef* block = lpBox( c, (b3Vec3){ 0.5f * side, 0.5f * levelH, 0.5f * t }, q, lp_stone, color, l == 0 );
			if ( block != NULL )
			{
				block->joint = lp_jointDry; // dry-stacked: friction only, so the tower hinges and falls once it leans
			}
		}
	}
	// Crenellations
	for ( int s = 0; s < sides; s += 2 )
	{
		float a = 2.0f * B3_PI * (float)s / (float)sides;
		b3CosSin cs = b3ComputeCosSin( a );
		b3Vec3 c = { radius * cs.cosine, (float)levels * levelH + 0.35f, radius * cs.sine };
		lpPartDef* merlon = lpBox( c, (b3Vec3){ 0.35f * side, 0.35f, 0.5f * t }, lpYaw( -a + 0.5f * B3_PI ), lp_stone, LP_STONE, false );
		if ( merlon != NULL )
		{
			merlon->joint = lp_jointDry;
		}
	}
	lpCommit( world, base, 0.0f, true );
}

static void lpAddPile( lpWorld* world, b3Vec3 center, int count, uint64_t seed )
{
	uint64_t rng = seed;
	int side = (int)ceilf( cbrtf( (float)count ) );
	int n = 0;
	for ( int y = 0; n < count; ++y )
	{
		for ( int x = 0; x < side && n < count; ++x )
		{
			for ( int z = 0; z < side && n < count; ++z, ++n )
			{
				lpBegin();
				b3Vec3 p = { center.x + ( (float)x - 0.5f * (float)side ) * 0.9f, center.y + 1.0f + (float)y * 0.9f,
							 center.z + ( (float)z - 0.5f * (float)side ) * 0.9f };
				if ( lpNext( &rng ) % 3u == 0u )
				{
					b3Vec3 pts[12];
					for ( int i = 0; i < 12; ++i )
					{
						b3Vec3 d = b3Normalize( (b3Vec3){ lpUnit( &rng ) - 0.5f, lpUnit( &rng ) - 0.5f, lpUnit( &rng ) - 0.5f } );
						pts[i] = b3MulSV( 0.35f * ( 0.8f + 0.4f * lpUnit( &rng ) ), d );
					}
					lpHull( pts, 12, lp_stone, LP_STONE, false );
				}
				else
				{
					lpBox( b3Vec3_zero, (b3Vec3){ 0.35f, 0.35f, 0.35f }, b3Quat_identity, lp_wood, LP_PLANK, false );
				}
				lpCommit( world, p, 6.2831853f * lpUnit( &rng ), false );
			}
		}
	}
}

void lpBuildScene( lpWorld* world, int scene )
{
	switch ( scene )
	{
		case lp_sceneWall:
		{
			lpAddGround( world, 60.0f );
			lpAddWall( world, (b3Vec3){ 0.0f, 0.0f, -4.0f }, 0.0f, 8.0f, 3.2f, 0.3f, lp_brick, LP_BRICK, 1.6f );
			lpAddWall( world, (b3Vec3){ 0.0f, 0.0f, -9.0f }, 0.0f, 10.0f, 4.0f, 0.6f, lp_stone, LP_STONE, 2.0f );
			lpAddWall( world, (b3Vec3){ -7.0f, 0.0f, -2.0f }, 0.5f * B3_PI, 5.0f, 2.6f, 0.25f, lp_plaster, LP_PLASTER, 1.5f );
			lpAddFence( world, (b3Vec3){ 5.0f, 0.0f, 0.0f }, 0.5f * B3_PI, 7.0f );
			lpBegin();
			lpBox( (b3Vec3){ 0.0f, 1.6f, 0.0f }, (b3Vec3){ 1.0f, 0.8f, 0.02f }, b3Quat_identity, lp_glass, LP_GLASS, true );
			lpBox( (b3Vec3){ 0.0f, 0.4f, 0.0f }, (b3Vec3){ 1.1f, 0.4f, 0.1f }, b3Quat_identity, lp_wood, LP_BEAM, true );
			lpCommit( world, (b3Vec3){ -3.0f, 0.0f, 1.0f }, 0.0f, true );
			lpAddTree( world, (b3Vec3){ 8.0f, 0.0f, -6.0f }, 5.0f, 7u );
			break;
		}

		case lp_sceneHouse:
			lpAddGround( world, 60.0f );
			lpAddHouse( world, (b3Vec3){ 0.0f, 0.0f, -6.0f }, 0.0f, 3u );
			lpAddTree( world, (b3Vec3){ 7.0f, 0.0f, -4.0f }, 5.5f, 11u );
			lpAddFence( world, (b3Vec3){ 0.0f, 0.0f, 0.5f }, 0.0f, 9.0f );
			break;

		case lp_sceneTown:
		{
			lpAddGround( world, 120.0f );
			uint64_t rng = 42;
			for ( int i = 0; i < 6; ++i )
			{
				float x = -27.0f + 11.0f * (float)i;
				lpAddHouse( world, (b3Vec3){ x, 0.0f, -10.0f }, 0.0f, lpNext( &rng ) );
				lpAddHouse( world, (b3Vec3){ x + 3.0f, 0.0f, 10.0f }, B3_PI, lpNext( &rng ) );
				lpAddTree( world, (b3Vec3){ x + 5.5f, 0.0f, -4.5f }, 4.5f + 2.0f * lpUnit( &rng ), lpNext( &rng ) );
				lpAddFence( world, (b3Vec3){ x - 1.0f, 0.0f, 4.5f }, 0.0f, 6.0f );
			}
			lpAddTowerAt( world, (b3Vec3){ 40.0f, 0.0f, 0.0f }, 7 );
			break;
		}

		case lp_sceneTower:
			lpAddGround( world, 60.0f );
			lpAddTowerAt( world, (b3Vec3){ 0.0f, 0.0f, -8.0f }, 9 );
			lpAddTree( world, (b3Vec3){ 7.0f, 0.0f, -5.0f }, 5.0f, 5u );
			break;

		case lp_scenePile:
			lpAddGround( world, 60.0f );
			lpAddPile( world, (b3Vec3){ 0.0f, 0.0f, -6.0f }, 512, 9u );
			break;

		case lp_sceneLumber:
		{
			lpAddGround( world, 60.0f );
			// A log bridge over two stumps, the first thing in view
			lpAddStump( world, (b3Vec3){ -2.2f, 0.0f, -3.0f }, 0.8f, 0.35f );
			lpAddStump( world, (b3Vec3){ 2.2f, 0.0f, -3.0f }, 0.8f, 0.35f );
			lpAddLog( world, (b3Vec3){ 0.0f, 1.05f, -3.0f }, 0.0f, 5.2f, 0.24f, false );
			// Logs lying about
			lpAddLog( world, (b3Vec3){ -4.5f, 0.3f, 0.5f }, 0.4f, 4.0f, 0.3f, false );
			lpAddLog( world, (b3Vec3){ 4.8f, 0.25f, 0.0f }, -0.3f, 3.2f, 0.25f, false );
			// A woodpile
			for ( int row = 0; row < 3; ++row )
			{
				for ( int k = 0; k < 4 - row; ++k )
				{
					float x = 6.0f + 0.42f * ( (float)k - 0.5f * (float)( 3 - row ) );
					lpAddLog( world, (b3Vec3){ x, 0.2f + 0.36f * (float)row, -7.0f }, 0.5f * B3_PI, 2.0f, 0.2f, false );
				}
			}
			lpAddShed( world, (b3Vec3){ -6.0f, 0.0f, -8.0f }, 0.3f );
			lpAddTree( world, (b3Vec3){ 1.0f, 0.0f, -10.0f }, 6.0f, 21u );
			lpAddTree( world, (b3Vec3){ 9.0f, 0.0f, -4.0f }, 5.0f, 22u );
			lpAddTree( world, (b3Vec3){ -10.0f, 0.0f, -3.0f }, 5.5f, 23u );
			lpAddFence( world, (b3Vec3){ 0.0f, 0.0f, 3.5f }, 0.0f, 10.0f );
			break;
		}

		default:
			lpAddGround( world, 60.0f );
			break;
	}
}

bool lpSceneBombard( lpWorld* world, int scene, int tick, int period )
{
	if ( period <= 0 || tick % period != period / 2 )
	{
		return false;
	}
	int shot = tick / period;
	uint64_t rng = 0xB0B0ull + (uint64_t)shot * 7919ull;

	b3Vec3 origin;
	b3Vec3 target;
	switch ( scene )
	{
		case lp_sceneTown:
		{
			// Walk down the street firing at both rows of houses and the tower at the end
			float x = -30.0f + (float)( ( shot * 7 ) % 75 );
			origin = (b3Vec3){ x, 1.8f, 0.0f };
			float side = ( shot % 2 == 0 ) ? -1.0f : 1.0f;
			target = (b3Vec3){ x + 6.0f * ( lpUnit( &rng ) - 0.5f ), 0.4f + 5.0f * lpUnit( &rng ), side * 12.0f };
			if ( x > 34.0f )
			{
				target = (b3Vec3){ 40.0f, 0.5f + 8.0f * lpUnit( &rng ), 2.0f * ( lpUnit( &rng ) - 0.5f ) };
			}
			break;
		}
		default:
		{
			origin = (b3Vec3){ 6.0f * ( lpUnit( &rng ) - 0.5f ), 1.8f, 10.0f };
			target = (b3Vec3){ 12.0f * ( lpUnit( &rng ) - 0.5f ), 0.3f + 4.0f * lpUnit( &rng ), -8.0f };
			break;
		}
	}

	b3Vec3 dir = b3Normalize( b3Sub( target, origin ) );
	lpRayHit hit = lpWorld_CastRay( world, origin, b3MulSV( 60.0f, dir ) );
	if ( hit.hit == false || hit.piece < 0 )
	{
		return false;
	}

	lpImpactDef impact = { 0 };
	impact.point = hit.point;
	impact.direction = dir;
	if ( shot % 4 == 3 )
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
	lpWorld_AddImpact( world, &impact );
	return true;
}
