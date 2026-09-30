// SPDX-License-Identifier: MIT

#include "scenes.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// C-library roots and trig can differ between libms; LPF_PORTABLE_MATH (a research option, off by default: it
// changes the scenes) uses the core's cube root and Box3D's own trig instead
#if defined( LPF_PORTABLE_MATH )
float lpCbrt( float x );
#define lpSceneCbrt( x ) lpCbrt( x )
#define lpSceneAtan2( y, x ) b3Atan2( y, x )
#define lpSceneSin( x ) b3ComputeCosSin( x ).sine
#else
#define lpSceneCbrt( x ) cbrtf( x )
#define lpSceneAtan2( y, x ) atan2f( y, x )
#define lpSceneSin( x ) sinf( x )
#endif

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

// Creates the object from the builder's parts, with the rest of its definition from `def` (velocity, detonator,
// gravity scale)
static int lpCommitDef( lpWorld* world, b3Vec3 position, b3Quat q, lpObjectDef def )
{
	def.transform.p = b3ToPos( position );
	def.transform.q = q;
	def.parts = lp_builder.parts;
	def.partCount = lp_builder.partCount;
	return lpCreateObject( world, &def );
}

static int lpCommit( lpWorld* world, b3Vec3 position, float yaw, bool isStatic )
{
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = isStatic;
	return lpCommitDef( world, position, b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 1.0f, 0.0f }, yaw ), def );
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
	if ( lpGetMaterial( material )->pattern == lp_breakMasonry )
	{
		columns = rows = 1; // one solid wall: its course grid decides where it breaks
	}
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
	float slope = lpSceneAtan2( rise, hd );
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

// A semicircular arch of dry-laid voussoirs on two piers: it stands by compression alone, and falls without its
// keystone. `voussoirs` should be odd so one sits at the crown.
static void lpAddArch( lpWorld* world, b3Vec3 base, float radius, float thickness, float depth, int voussoirs )
{
	lpBegin();
	float springing = 1.2f;
	float mid = radius + 0.5f * thickness;
	for ( int side = -1; side <= 1; side += 2 )
	{
		lpPartDef* pier = lpBox( (b3Vec3){ (float)side * mid, 0.5f * springing, 0.0f },
								 (b3Vec3){ 0.5f * thickness + 0.1f, 0.5f * springing, 0.5f * depth }, b3Quat_identity, lp_stone,
								 LP_STONE_DARK, true );
		if ( pier != NULL )
		{
			pier->joint = lp_jointDry;
		}
	}
	for ( int i = 0; i < voussoirs; ++i )
	{
		float a0 = B3_PI * (float)i / (float)voussoirs;
		float a1 = B3_PI * (float)( i + 1 ) / (float)voussoirs;
		b3CosSin c0 = b3ComputeCosSin( a0 );
		b3CosSin c1 = b3ComputeCosSin( a1 );
		b3Vec3 pts[8];
		for ( int k = 0; k < 2; ++k )
		{
			float z = k == 0 ? -0.5f * depth : 0.5f * depth;
			pts[4 * k + 0] = (b3Vec3){ radius * c0.cosine, springing + radius * c0.sine, z };
			pts[4 * k + 1] = (b3Vec3){ ( radius + thickness ) * c0.cosine, springing + ( radius + thickness ) * c0.sine, z };
			pts[4 * k + 2] = (b3Vec3){ radius * c1.cosine, springing + radius * c1.sine, z };
			pts[4 * k + 3] = (b3Vec3){ ( radius + thickness ) * c1.cosine, springing + ( radius + thickness ) * c1.sine, z };
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
static void lpAddColonnade( lpWorld* world, b3Vec3 base, int columns, float spacing, float height )
{
	lpBegin();
	for ( int i = 0; i < columns; ++i )
	{
		lpBox( (b3Vec3){ (float)i * spacing, 0.5f * height, 0.0f }, (b3Vec3){ 0.2f, 0.5f * height, 0.2f }, b3Quat_identity, lp_stone,
			   LP_STONE, true );
	}
	for ( int i = 0; i + 1 < columns; ++i )
	{
		float x0 = (float)i * spacing - ( i == 0 ? 0.2f : 0.0f );
		float x1 = (float)( i + 1 ) * spacing + ( i + 2 == columns ? 0.2f : 0.0f );
		lpBox( (b3Vec3){ 0.5f * ( x0 + x1 ), height + 0.175f, 0.0f }, (b3Vec3){ 0.5f * ( x1 - x0 ), 0.175f, 0.25f },
			   b3Quat_identity, lp_stone, LP_STONE_DARK, false );
	}
	lpCommit( world, base, 0.0f, true );
}

// A stone wall with two mortared balconies: the short one stands easily; the long one is near its limit (root joint
// at about 94%), so one hit at its root brings it down
static void lpAddBalconies( lpWorld* world, b3Vec3 base )
{
	lpBegin();
	lpBox( (b3Vec3){ 0.0f, 1.5f, 0.0f }, (b3Vec3){ 2.0f, 1.5f, 0.2f }, b3Quat_identity, lp_stone, LP_STONE, true );
	lpBox( (b3Vec3){ -1.0f, 2.2f, 0.2f + 0.4f }, (b3Vec3){ 0.5f, 0.2f, 0.4f }, b3Quat_identity, lp_stone, LP_STONE_DARK, false );
	lpBox( (b3Vec3){ 1.0f, 2.2f, 0.2f + 0.625f }, (b3Vec3){ 0.5f, 0.2f, 0.625f }, b3Quat_identity, lp_stone, LP_STONE_DARK, false );
	lpCommit( world, base, 0.0f, true );
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
	float side = 2.0f * radius * lpSceneSin( B3_PI / (float)sides ) + 0.02f;
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
	b3Vec3 center = alongX ? (b3Vec3){ mid, y0 + hy, across } : (b3Vec3){ across, y0 + hy, mid };
	b3Vec3 half = alongX ? (b3Vec3){ 0.5f * ( b - a ), hy, 0.5f * thickness } : (b3Vec3){ 0.5f * thickness, hy, 0.5f * ( b - a ) };
	lpBox( center, half, b3Quat_identity, lp_stone, color, anchored );
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
int lpAddKeep( lpWorld* world, b3Vec3 base, int floors )
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
				lpBox( (b3Vec3){ s * 0.5f * ( cross + inner ), y - 0.21f, z }, (b3Vec3){ 0.5f * ( inner - cross ), 0.15f, 0.125f },
					   b3Quat_identity, lp_wood, LP_BEAM, false );
				lpBox( (b3Vec3){ s * ( inner - 0.225f ), y - 0.51f, z }, (b3Vec3){ 0.225f, 0.15f, 0.175f }, b3Quat_identity, lp_stone,
					   LP_STONE_DARK, false );
				lpBox( (b3Vec3){ s * ( cross + 0.225f ), y - 0.51f, z }, (b3Vec3){ 0.225f, 0.15f, 0.175f }, b3Quat_identity, lp_stone,
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
					lpBox( (b3Vec3){ s * 0.5f * ( px0 + px1 ), y - 0.03f, 0.5f * ( z0 + z1 ) },
						   (b3Vec3){ 0.5f * ( px1 - px0 ), 0.03f, 0.5f * ( z1 - z0 ) }, b3Quat_identity, lp_wood, LP_PLANK, false );
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

static lpLinkDef lpLinkBetween( int type, int bodyA, int bodyB, b3Vec3 anchorA, b3Vec3 anchorB )
{
	lpLinkDef def = lpDefaultLinkDef( type );
	def.bodyA = bodyA;
	def.bodyB = bodyB;
	def.anchorA = b3ToPos( anchorA );
	def.anchorB = b3ToPos( anchorB );
	return def;
}

// A cart of volatile crates parked on a ramp and tied to a post at the top; a brick wall waits at the bottom. Cut the
// rope and it rolls down into the wall: the crash tears wheels off their axles and sets the crates off.
static void lpAddCartOnRamp( lpWorld* world, float z )
{
	float slope = 0.35f; // well past a 24-sided wheel's tipping angle (7.5 degrees), so it rolls, and fast
	b3CosSin cs = b3ComputeCosSin( slope );
	b3Quat tilt = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, -slope );
	b3Vec3 down = { cs.cosine, -cs.sine, 0.0f }; // along the ramp, downhill
	b3Vec3 normal = { cs.sine, cs.cosine, 0.0f };
	b3Vec3 foot = { -5.0f, 0.0f, z }; // where the ramp's surface meets the ground
	float length = 7.0f;

	// The ramp: one anchored stone slab
	lpBegin();
	lpBox( b3Vec3_zero, (b3Vec3){ 0.5f * length, 0.1f, 1.2f }, b3Quat_identity, lp_stone, LP_STONE, true );
	lpCommitDef( world, b3MulAdd( b3MulAdd( foot, -0.5f * length, down ), -0.1f, normal ), tilt, lpDefaultObjectDef() );

	// The post at the top that the cart is tied to
	b3Vec3 top = b3MulAdd( foot, -length, down );
	lpBegin();
	float postHalf = 0.5f * ( top.y + 0.9f );
	lpBox( b3Vec3_zero, (b3Vec3){ 0.1f, postHalf, 0.1f }, b3Quat_identity, lp_wood, LP_BEAM, true );
	int post = lpCommit( world, (b3Vec3){ top.x - 0.5f, postHalf, z }, 0.0f, true );

	// The cart: a plank bed with low sides, 1.5 m down the ramp, its wheels resting on the slab
	b3Vec3 origin = b3MulAdd( b3MulAdd( top, 1.5f, down ), 0.5f, normal );
	lpBegin();
	lpBox( b3Vec3_zero, (b3Vec3){ 0.8f, 0.05f, 0.5f }, b3Quat_identity, lp_wood, LP_PLANK, false );
	for ( int side = -1; side <= 1; side += 2 )
	{
		lpBox( (b3Vec3){ 0.0f, 0.2f, 0.47f * (float)side }, (b3Vec3){ 0.8f, 0.15f, 0.03f }, b3Quat_identity, lp_wood, LP_BEAM,
			   false );
		lpBox( (b3Vec3){ 0.77f * (float)side, 0.2f, 0.0f }, (b3Vec3){ 0.03f, 0.15f, 0.44f }, b3Quat_identity, lp_wood, LP_BEAM,
			   false );
	}
	int cart = lpCommitDef( world, origin, tilt, lpDynamicDef() );

	// Four 24-sided wheels on axle pegs (hinges): a hard crash tears them off
	for ( int i = 0; i < 4; ++i )
	{
		b3Vec3 pts[48];
		for ( int k = 0; k < 24; ++k )
		{
			b3CosSin c = b3ComputeCosSin( 0.2617994f * (float)k );
			pts[k] = (b3Vec3){ 0.3f * c.cosine, 0.3f * c.sine, -0.05f };
			pts[24 + k] = (b3Vec3){ 0.3f * c.cosine, 0.3f * c.sine, 0.05f };
		}
		lpBegin();
		lpHull( pts, 48, lp_wood, LP_BARK, false );
		b3Vec3 hub = b3Add( origin, b3RotateVector( tilt, (b3Vec3){ i < 2 ? -0.55f : 0.55f, -0.2f, i % 2 == 0 ? -0.62f : 0.62f } ) );
		int wheel = lpCommitDef( world, hub, tilt, lpDynamicDef() );
		lpLinkDef axle = lpLinkBetween( lp_linkHinge, cart, wheel, hub, hub );
		axle.axis = (b3Vec3){ 0.0f, 0.0f, 1.0f };
		axle.maxForce = 6000.0f;
		axle.maxTorque = 800.0f;
		lpCreateLink( world, &axle );
	}

	// Two crates of something volatile in the bed
	for ( int k = 0; k < 2; ++k )
	{
		lpBegin();
		lpBox( b3Vec3_zero, (b3Vec3){ 0.2f, 0.2f, 0.2f }, b3Quat_identity, lp_wood, LP_PLANK, false );
		lpObjectDef def = lpDynamicDef();
		def.detonator = (lpDetonatorDef){ 3.5f, 1.6f, 70000.0f, 10.0f };
		lpCommitDef( world, b3Add( origin, b3RotateVector( tilt, (b3Vec3){ k == 0 ? -0.35f : 0.3f, 0.26f, 0.0f } ) ), tilt, def );
	}

	// The rope, from the post to the back of the cart
	b3Vec3 tie = b3Add( origin, b3RotateVector( tilt, (b3Vec3){ -0.8f, 0.2f, 0.0f } ) );
	lpLinkDef rope = lpLinkBetween( lp_linkRope, post, cart, (b3Vec3){ top.x - 0.4f, top.y + 0.35f, z }, tie );
	lpCreateLink( world, &rope );
}

// A sign hanging on two ropes from a gallows
static void lpAddHangingSign( lpWorld* world, b3Vec3 base )
{
	lpBegin();
	lpBox( (b3Vec3){ -1.1f, 1.6f, 0.0f }, (b3Vec3){ 0.1f, 1.6f, 0.1f }, b3Quat_identity, lp_wood, LP_BEAM, true );
	lpBox( (b3Vec3){ 1.1f, 1.6f, 0.0f }, (b3Vec3){ 0.1f, 1.6f, 0.1f }, b3Quat_identity, lp_wood, LP_BEAM, true );
	lpBox( (b3Vec3){ 0.0f, 3.3f, 0.0f }, (b3Vec3){ 1.3f, 0.1f, 0.12f }, b3Quat_identity, lp_wood, LP_BEAM, false );
	int gallows = lpCommit( world, base, 0.0f, true );
	lpBegin();
	lpBox( b3Vec3_zero, (b3Vec3){ 0.6f, 0.35f, 0.03f }, b3Quat_identity, lp_wood, LP_PLANK, false );
	int sign = lpCommit( world, (b3Vec3){ base.x, base.y + 2.0f, base.z }, 0.0f, false );
	for ( int side = -1; side <= 1; side += 2 )
	{
		float x = base.x + 0.5f * (float)side;
		lpLinkDef rope = lpLinkBetween( lp_linkRope, gallows, sign, (b3Vec3){ x, base.y + 3.2f, base.z },
										(b3Vec3){ x, base.y + 2.35f, base.z } );
		lpCreateLink( world, &rope );
	}
}

// A door on a hinge in its own frame (a hinge stops all collision between the door and the frame)
static void lpAddDoorway( lpWorld* world, b3Vec3 base )
{
	lpBegin();
	lpBox( (b3Vec3){ -0.7f, 1.15f, 0.0f }, (b3Vec3){ 0.1f, 1.15f, 0.1f }, b3Quat_identity, lp_wood, LP_BEAM, true );
	lpBox( (b3Vec3){ 0.7f, 1.15f, 0.0f }, (b3Vec3){ 0.1f, 1.15f, 0.1f }, b3Quat_identity, lp_wood, LP_BEAM, true );
	lpBox( (b3Vec3){ 0.0f, 2.4f, 0.0f }, (b3Vec3){ 0.8f, 0.1f, 0.1f }, b3Quat_identity, lp_wood, LP_BEAM, false );
	int frame = lpCommit( world, base, 0.0f, true );
	lpBegin();
	lpBox( b3Vec3_zero, (b3Vec3){ 0.55f, 1.02f, 0.03f }, b3Quat_identity, lp_wood, LP_PLANK, false );
	int door = lpCommit( world, (b3Vec3){ base.x, base.y + 1.12f, base.z }, 0.0f, false );
	lpLinkDef hinge = lpLinkBetween( lp_linkHinge, frame, door, (b3Vec3){ base.x - 0.575f, base.y + 1.12f, base.z }, b3Vec3_zero );
	hinge.axis = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	hinge.lowerAngle = -1.4f;
	hinge.upperAngle = 1.4f;
	lpCreateLink( world, &hinge );
}

// A raised drawbridge in a stone gatehouse: hinged at its foot on the sill, held up by two ropes from under the
// lintel. It leans out a little, so when the ropes go it falls open.
static void lpAddDrawbridge( lpWorld* world, b3Vec3 base )
{
	lpBegin();
	lpBox( (b3Vec3){ -1.3f, 1.6f, -0.3f }, (b3Vec3){ 0.4f, 1.6f, 0.5f }, b3Quat_identity, lp_stone, LP_STONE, true );
	lpBox( (b3Vec3){ 1.3f, 1.6f, -0.3f }, (b3Vec3){ 0.4f, 1.6f, 0.5f }, b3Quat_identity, lp_stone, LP_STONE, true );
	lpBox( (b3Vec3){ 0.0f, 3.4f, -0.3f }, (b3Vec3){ 1.7f, 0.2f, 0.5f }, b3Quat_identity, lp_stone, LP_STONE_DARK, false );
	lpBox( (b3Vec3){ 0.0f, 0.05f, -0.3f }, (b3Vec3){ 0.9f, 0.05f, 0.5f }, b3Quat_identity, lp_stone, LP_STONE_DARK, true );
	int gate = lpCommit( world, base, 0.0f, true );

	b3Quat lean = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, 0.05f );
	b3Vec3 foot = { base.x, base.y + 0.1f, base.z + 0.2f }; // the sill's front edge
	b3Vec3 center = b3Add( foot, b3RotateVector( lean, (b3Vec3){ 0.0f, 1.4f, -0.06f } ) );
	lpBegin();
	lpBox( b3Vec3_zero, (b3Vec3){ 0.85f, 1.4f, 0.06f }, b3Quat_identity, lp_wood, LP_PLANK, false );
	int bridge = lpCommitDef( world, center, lean, lpDynamicDef() );
	lpLinkDef hinge = lpLinkBetween( lp_linkHinge, gate, bridge, foot, b3Vec3_zero );
	hinge.axis = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	lpCreateLink( world, &hinge );
	for ( int side = -1; side <= 1; side += 2 )
	{
		b3Vec3 hook = { base.x + 0.7f * (float)side, base.y + 3.2f, base.z - 0.3f };
		b3Vec3 top = b3Add( center, b3RotateVector( lean, (b3Vec3){ 0.7f * (float)side, 1.4f, 0.0f } ) );
		lpLinkDef rope = lpLinkBetween( lp_linkRope, gate, bridge, hook, top );
		lpCreateLink( world, &rope );
	}
}

// A porter's rack (a base board, two uprights and a top bar) with two flasks hung from the bar on strings. Dusted to
// half its weight so the grab tool carries it; jostled too hard, the flasks swing into it and go off.
static void lpAddPorterRack( lpWorld* world, b3Vec3 base )
{
	lpBegin();
	lpBox( (b3Vec3){ 0.0f, 0.03f, 0.0f }, (b3Vec3){ 0.35f, 0.03f, 0.25f }, b3Quat_identity, lp_wood, LP_PLANK, false );
	lpBox( (b3Vec3){ -0.3f, 0.66f, 0.0f }, (b3Vec3){ 0.04f, 0.6f, 0.04f }, b3Quat_identity, lp_wood, LP_BEAM, false );
	lpBox( (b3Vec3){ 0.3f, 0.66f, 0.0f }, (b3Vec3){ 0.04f, 0.6f, 0.04f }, b3Quat_identity, lp_wood, LP_BEAM, false );
	lpBox( (b3Vec3){ 0.0f, 1.3f, 0.0f }, (b3Vec3){ 0.34f, 0.04f, 0.04f }, b3Quat_identity, lp_wood, LP_BEAM, false );
	lpObjectDef def = lpDynamicDef();
	def.gravityScale = 0.5f;
	int rack = lpCommitDef( world, base, b3Quat_identity, def );
	for ( int side = -1; side <= 1; side += 2 )
	{
		float x = base.x + 0.15f * (float)side;
		b3Vec3 pts[12];
		lpRing( pts, (b3Vec3){ 0.0f, -0.11f, 0.0f }, 0.06f, 6, 0.0f );
		lpRing( pts + 6, (b3Vec3){ 0.0f, 0.11f, 0.0f }, 0.06f, 6, 0.0f );
		lpBegin();
		lpHull( pts, 12, lp_glass, LP_GLASS, false );
		lpObjectDef flask = lpDynamicDef();
		flask.detonator = (lpDetonatorDef){ 4.0f, 1.4f, 60000.0f, 10.0f };
		int bottle = lpCommitDef( world, (b3Vec3){ x, base.y + 0.9f, base.z }, b3Quat_identity, flask );
		lpLinkDef string = lpLinkBetween( lp_linkRope, rack, bottle, (b3Vec3){ x, base.y + 1.26f, base.z },
										  (b3Vec3){ x, base.y + 1.01f, base.z } );
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
static b3Vec3 lpRingPoint( float r, float a, float y )
{
	b3CosSin cs = b3ComputeCosSin( a );
	return (b3Vec3){ r * cs.cosine, y, r * cs.sine };
}

// A ramp along local z: height 0 at z0, h at z1
static void lpWedge( float halfWidth, float z0, float z1, float h, int material, uint32_t color )
{
	b3Vec3 pts[6] = { { -halfWidth, 0.0f, z0 }, { halfWidth, 0.0f, z0 }, { -halfWidth, 0.0f, z1 },
					  { halfWidth, 0.0f, z1 },	{ -halfWidth, h, z1 },	 { halfWidth, h, z1 } };
	lpHull( pts, 6, material, color, true );
}

// A sheet-metal part of the car kit, with its system
static lpPartDef* lpCarPart( b3Vec3 center, b3Vec3 half, int material, uint32_t color, uint16_t tag, uint8_t carries,
							 uint8_t sources, uint8_t needs )
{
	lpPartDef* part = lpBox( center, half, b3Quat_identity, material, color, false );
	if ( part != NULL )
	{
		part->system = (lpPartSystem){ tag, carries, sources, needs };
	}
	return part;
}

int lpAddCar( lpWorld* world, b3Vec3 base, float yaw, int style )
{
	static const uint32_t paints[4] = { 0x2F6FB5u, 0xC0392Bu, 0xE0A526u, 0x3C8D4Fu };
	uint32_t paint = paints[( style % 4 + 4 ) % 4];
	const uint8_t fuel = 1u << lp_channelFuel, power = 1u << lp_channelPower, steer = 1u << lp_channelSteer;
	const uint32_t trim = 0x3A3D42u, glass = 0x9FD3E0u;
	float ride = 0.72f; // the floor pan's bottom above the ground, about where it rests
	b3Quat q = lpYaw( yaw );

	lpBegin();
	// The floor pan carries every line; what sits on it is bolted to it
	lpCarPart( (b3Vec3){ 0.0f, 0.04f, 0.0f }, (b3Vec3){ 0.85f, 0.04f, 2.1f }, lp_sheetMetal, trim, lp_tagFrame,
			   fuel | power | steer, 0, 0 );
	for ( int k = 0; k < 3; ++k ) // an engine of three blocks: lose one and a third of the power goes with it
	{
		lpPartDef* block = lpCarPart( (b3Vec3){ 0.0f, 0.28f, 1.25f + 0.2f * (float)k }, (b3Vec3){ 0.3f, 0.2f, 0.095f }, lp_sheetMetal,
									  0x55595Eu, lp_tagEngine, fuel, power, fuel );
		if ( block != NULL )
		{
			block->joint = lp_jointMounts; // a hard crash tears it off its mounts
		}
	}
	lpPartDef* tank = lpCarPart( (b3Vec3){ 0.0f, 0.2f, -1.55f }, (b3Vec3){ 0.4f, 0.12f, 0.22f }, lp_sheetMetal, 0x6B3A2Au,
								 lp_tagFuelTank, 0, fuel, 0 );
	if ( tank != NULL )
	{
		tank->detonator = (lpDetonatorDef){ 14.0f, 2.2f, 150000.0f, 12.0f };
	}
	lpCarPart( (b3Vec3){ -0.45f, 0.18f, 1.05f }, (b3Vec3){ 0.12f, 0.1f, 0.12f }, lp_sheetMetal, trim, lp_tagSteering, power, steer,
			   power );
	// Body panels, pillars and roof, glass, bumpers
	lpCarPart( (b3Vec3){ 0.0f, 0.51f, 1.5f }, (b3Vec3){ 0.8f, 0.03f, 0.55f }, lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
	lpCarPart( (b3Vec3){ 0.0f, 0.35f, -1.75f }, (b3Vec3){ 0.8f, 0.03f, 0.4f }, lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
	for ( int side = -1; side <= 1; side += 2 )
	{
		float x = 0.82f * (float)side;
		lpCarPart( (b3Vec3){ x, 0.38f, -0.25f }, (b3Vec3){ 0.03f, 0.3f, 0.7f }, lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
		for ( int k = 0; k < 2; ++k )
		{
			lpCarPart( (b3Vec3){ 0.72f * (float)side, 0.575f, k == 0 ? 0.5f : -1.0f }, (b3Vec3){ 0.06f, 0.495f, 0.06f },
					   lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
		}
	}
	lpCarPart( (b3Vec3){ 0.0f, 1.1f, -0.25f }, (b3Vec3){ 0.78f, 0.03f, 0.85f }, lp_sheetMetal, paint, lp_tagPanel, 0, 0, 0 );
	for ( int k = 0; k < 2; ++k )
	{
		lpCarPart( (b3Vec3){ 0.0f, 0.78f, k == 0 ? 0.5f : -1.0f }, (b3Vec3){ 0.66f, 0.29f, 0.015f }, lp_glass, glass, lp_tagGlass, 0, 0,
				   0 );
		lpCarPart( (b3Vec3){ 0.0f, 0.1f, k == 0 ? 2.18f : -2.18f }, (b3Vec3){ 0.85f, 0.1f, 0.08f }, lp_rubber, 0x2A2A2Au,
				   lp_tagBumper, 0, 0, 0 );
	}
	b3Vec3 origin = { base.x, base.y + ride, base.z };
	lpObjectDef carDef = lpDynamicDef();
	carDef.solveStress = true; // a crash's deceleration loads its joints: an engine can tear off its mounts
	int body = lpCommitDef( world, origin, q, carDef );

	lpWheelDef wheels[4];
	for ( int i = 0; i < 4; ++i )
	{
		bool front = i >= 2;
		b3Vec3 mount = { ( i & 1 ) ? 0.78f : -0.78f, 0.0f, front ? 1.35f : -1.35f };
		wheels[i] = lpDefaultWheelDef();
		wheels[i].mount = b3ToPos( b3Add( origin, b3RotateVector( q, mount ) ) );
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
	def.forward = b3RotateVector( q, (b3Vec3){ 0.0f, 0.0f, 1.0f } );
	def.wheels = wheels;
	def.wheelCount = 4;
	def.maxDriveForce = 10000.0f;
	def.maxSpeed = 30.0f;
	def.maxBrakeForce = 15000.0f;
	return lpCreateVehicle( world, &def );
}

int lpAddCrane( lpWorld* world, b3Vec3 base, float loadMass )
{
	// A steel base plate with the mast's foot set in it: the crane's weight and swing load the timber at its foot
	lpBegin();
	lpPartDef* footing = lpBox( (b3Vec3){ 0.0f, 0.4f, 0.0f }, (b3Vec3){ 1.2f, 0.4f, 1.2f }, b3Quat_identity, lp_metal, 0x6F7378u,
								true );
	lpPartDef* mast = lpBox( (b3Vec3){ 0.0f, 4.8f, 0.0f }, (b3Vec3){ 0.35f, 4.0f, 0.35f }, b3Quat_identity, lp_wood, LP_BEAM, false );
	if ( footing != NULL && mast != NULL )
	{
		footing->joint = lp_jointSolid;
		mast->joint = lp_jointSolid; // set in the base plate: the timber itself is the limit
		mast->grainAxis = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	}
	int tower = lpCommit( world, base, 0.0f, true );

	lpBegin();
	lpBox( b3Vec3_zero, (b3Vec3){ 0.9f, 0.2f, 0.9f }, b3Quat_identity, lp_sheetMetal, 0xD9A12Bu, false );
	int deck = lpCommitDef( world, b3Add( base, (b3Vec3){ 0.0f, 9.0f, 0.0f } ), b3Quat_identity, lpDynamicDef() );
	lpBegin();
	lpPartDef* beam = lpBox( b3Vec3_zero, (b3Vec3){ 5.0f, 0.2f, 0.2f }, b3Quat_identity, lp_wood, 0xD9A12Bu, false );
	if ( beam != NULL )
	{
		beam->grainAxis = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	}
	int jib = lpCommitDef( world, b3Add( base, (b3Vec3){ 5.0f, 9.4f, 0.0f } ), b3Quat_identity, lpDynamicDef() );
	float side = lpSceneCbrt( loadMass / lpGetMaterial( lp_metal )->density );
	lpBegin();
	lpBox( b3Vec3_zero, (b3Vec3){ 0.5f * side, 0.5f * side, 0.5f * side }, b3Quat_identity, lp_metal, 0x3A3D42u, false );
	int load = lpCommitDef( world, b3Add( base, (b3Vec3){ 9.8f, 9.2f - 5.0f - 0.5f * side, 0.0f } ), b3Quat_identity, lpDynamicDef() );

	b3Vec3 top = b3Add( base, (b3Vec3){ 0.0f, 8.8f, 0.0f } );
	lpLinkDef slew = lpLinkBetween( lp_linkHinge, tower, deck, top, top );
	slew.axis = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	slew.maxForce = 1e6f; // a slewing ring
	slew.maxTorque = 2e6f;
	slew.strength = 20000.0f;
	slew.motor = (lpMotorDef){ 40000.0f, 0.3f, 1.0f, 0, 40000.0f };
	slew.userId = lp_linkSlew;
	lpCreateLink( world, &slew );

	b3Vec3 root = b3Add( base, (b3Vec3){ 0.0f, 9.4f, 0.0f } );
	lpLinkDef luff = lpLinkBetween( lp_linkHinge, deck, jib, root, root );
	luff.axis = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	luff.lowerAngle = -0.2f;
	luff.upperAngle = 0.6f;
	luff.maxForce = 1e6f;
	luff.maxTorque = 2e6f;
	luff.strength = 20000.0f;
	luff.motor = (lpMotorDef){ 300000.0f, 0.2f, 2.0f, 0, 300000.0f };
	luff.userId = lp_linkLuff;
	lpCreateLink( world, &luff );

	b3Vec3 tip = b3Add( base, (b3Vec3){ 9.8f, 9.2f, 0.0f } );
	lpLinkDef winch = lpLinkBetween( lp_linkRope, jib, load, tip, b3Add( base, (b3Vec3){ 9.8f, 9.2f - 5.0f, 0.0f } ) );
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
static int lpHexSegment( lpWorld* world, b3Vec3 center, b3Quat q, b3Vec3 half, uint32_t color, int kind, b3Quat legQ )
{
	const uint8_t lines = ( 1u << lp_channelPower ) | ( 1u << lp_channelHydraulics ) | ( 1u << lp_channelControl );
	lpBegin();
	int halves = kind == lp_hexFemur ? 2 : 1;
	b3Vec3 partHalf = { half.x / (float)halves, half.y, half.z };
	for ( int h = 0; h < halves; ++h )
	{
		float x = halves == 2 ? ( h == 0 ? -partHalf.x : partHalf.x ) : 0.0f;
		lpPartDef* part = lpBox( (b3Vec3){ x, 0.0f, 0.0f }, partHalf, b3Quat_identity, lp_sheetMetal, color, false );
		if ( part != NULL )
		{
			part->grainAxis = (b3Vec3){ 1.0f, 0.0f, 0.0f };
			part->system = (lpPartSystem){ lp_tagLeg, lines, 0, 0 };
			part->joint = kind == lp_hexBlock ? part->joint : lp_jointWeld;
		}
	}
	if ( kind == lp_hexTibia )
	{
		// The sole sits square to the leg's plane (not tilted with the tibia), its top at the tibia's tip, welded on
		b3Quat local = b3InvMulQuat( q, legQ );
		b3Vec3 tip = { half.x, 0.0f, 0.0f };
		lpPartDef* sole = lpBox( b3Add( tip, b3RotateVector( local, (b3Vec3){ 0.0f, -LP_HEX_SOLE + 0.02f, 0.0f } ) ),
								 (b3Vec3){ 0.1f, LP_HEX_SOLE, 0.1f }, local, lp_rubber, 0x2A2A2Au, false );
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

int lpAddHexapod( lpWorld* world, b3Vec3 base, float yaw, int style )
{
	static const uint32_t paints[4] = { 0x8A8F4Bu, 0x4F6E8Cu, 0xB5652Eu, 0x5D5F63u };
	uint32_t paint = paints[( style % 4 + 4 ) % 4];
	const uint32_t steel = 0x55595Eu, dark = 0x3A3D42u;
	const b3Vec3 up = { 0.0f, 1.0f, 0.0f };
	b3Quat q = lpYaw( yaw );
	b3Vec3 origin = { base.x, base.y + LP_HEX_RIDE, base.z };

	// The torso, all armor welded together: a frame (the belly skid under it) carrying every line, a deck, and between
	// them a reactor (power), a hydraulic reservoir and a computer (control), both run on power. The legs are sheet metal:
	// they are what a grenade takes
	const uint8_t power = 1u << lp_channelPower, hydraulics = 1u << lp_channelHydraulics, control = 1u << lp_channelControl;
	const uint8_t lines = power | hydraulics | control;
	lpBegin();
	lpPartDef* frame = lpBox( (b3Vec3){ 0.0f, 0.0f, 0.0f }, (b3Vec3){ 0.95f, 0.08f, 1.35f }, b3Quat_identity, lp_armor, steel, false );
	lpPartDef* reactor = lpBox( (b3Vec3){ 0.0f, 0.33f, -0.75f }, (b3Vec3){ 0.35f, 0.25f, 0.35f }, b3Quat_identity, lp_armor, dark, false );
	lpPartDef* reservoir = lpBox( (b3Vec3){ 0.45f, 0.28f, 0.35f }, (b3Vec3){ 0.3f, 0.2f, 0.25f }, b3Quat_identity, lp_armor, 0x7A5C2Eu,
								  false );
	lpPartDef* computer = lpBox( (b3Vec3){ -0.45f, 0.23f, 0.45f }, (b3Vec3){ 0.2f, 0.15f, 0.2f }, b3Quat_identity, lp_armor, 0x2E4A3Au,
								 false );
	lpBox( (b3Vec3){ 0.0f, 0.61f, 0.0f }, (b3Vec3){ 0.9f, 0.03f, 1.2f }, b3Quat_identity, lp_armor, paint, false );
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
		b3CosSin cs = b3ComputeCosSin( splay );
		b3Vec3 d = b3RotateVector( q, (b3Vec3){ side * cs.cosine, 0.0f, cs.sine } ); // outward
		b3Vec3 t = b3Cross( d, up );												   // the pitch hinges' axis
		b3Matrix3 m = { d, up, t };
		b3Quat legQ = b3MakeQuatFromMatrix( &m );
		b3Vec3 hip = b3Add( origin, b3RotateVector( q, (b3Vec3){ 0.95f * side, 0.0f, row == 0 ? 1.1f : ( row == 1 ? 0.0f : -1.1f ) } ) );
		b3Vec3 femurRoot = b3MulAdd( hip, LP_HEX_FEMUR_U, d );
		b3Vec3 knee = b3Add( b3MulAdd( hip, LP_HEX_KNEE_U, d ), b3MulSV( LP_HEX_KNEE_V, up ) );
		b3Vec3 foot = b3Add( b3MulAdd( hip, LP_HEX_FOOT_U, d ), b3MulSV( LP_HEX_FOOT_V, up ) );

		// The hip block, the femur rising 30 degrees, the tibia down to the foot: slender, a third of the mech's weight (legs
		// as heavy as the torso shove it about as they swing)
		b3Quat femurTilt = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, 0.5236f );
		b3Vec3 shin = { LP_HEX_FOOT_U - LP_HEX_KNEE_U, LP_HEX_FOOT_V - LP_HEX_KNEE_V, 0.0f };
		float shinLength = b3Length( shin );
		b3Quat tibiaTilt = b3MakeQuatFromAxisAngle( (b3Vec3){ 0.0f, 0.0f, 1.0f }, b3Atan2( shin.y, shin.x ) );
		int block = lpHexSegment( world, b3Lerp( hip, femurRoot, 0.5f ), legQ, (b3Vec3){ 0.22f, 0.15f, 0.15f }, steel, lp_hexBlock, legQ );
		int femur = lpHexSegment( world, b3Lerp( femurRoot, knee, 0.5f ), b3MulQuat( legQ, femurTilt ), (b3Vec3){ 0.6f, 0.09f, 0.09f },
								  paint, lp_hexFemur, legQ );
		int tibia = lpHexSegment( world, b3Lerp( knee, foot, 0.5f ), b3MulQuat( legQ, tibiaTilt ),
								  (b3Vec3){ 0.5f * shinLength, 0.08f, 0.085f }, paint, lp_hexTibia, legQ );

		// Motorised hinges: the hip turns the leg about the vertical, the femur and the knee lift it
		int bodies[4] = { torso, block, femur, tibia };
		b3Vec3 anchors[3] = { hip, femurRoot, knee };
		b3Vec3 axes[3] = { b3RotateVector( q, up ), t, t };
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
		limbs[leg].foot = b3ToPos( b3MulAdd( foot, -2.0f * LP_HEX_SOLE + 0.02f, up ) ); // the sole's bottom
	}
	lpRigDef def = lpDefaultRigDef();
	def.body = torso;
	def.stride = 0.6f;	   // a step of about 0.85 m
	def.swingTime = 0.35f; // so a tripod keeps up 2.3 m/s
	def.maxSpeed = 2.3f;
	def.forward = b3RotateVector( q, (b3Vec3){ 0.0f, 0.0f, 1.0f } );
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

// The track's crane swings its load back and forth over the infield and winches it up and down
static void lpDriveCrane( lpWorld* world, int tick )
{
	int slew = lpFindLink( world, lp_linkSlew );
	int luff = lpFindLink( world, lp_linkLuff );
	int winch = lpFindLink( world, lp_linkWinch );
	lpWorld_SetLinkTarget( world, slew, ( tick / 360 ) % 2 == 0 ? 1.2f : -1.2f );
	lpWorld_SetLinkTarget( world, luff, 0.1f );
	// The winch reels at 1 m/s: a rope shortened at once would fling its load up
	float length = winch >= 0 ? lpWorld_GetLinkState( world, winch ).length : 0.0f;
	float goal = ( tick / 240 ) % 2 == 0 ? 5.0f : 3.0f;
	if ( winch >= 0 && length != goal )
	{
		lpWorld_SetRopeLength( world, winch, length + b3ClampFloat( goal - length, -1.0f / 60.0f, 1.0f / 60.0f ) );
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
		lpBox( b3Vec3_zero, (b3Vec3){ 4.5f, 0.08f, 0.4f }, b3Quat_identity, lp_stone, LP_STONE, true );
		lpCommit( world, lpRingPoint( r, a, 0.08f ), -a, true );
	}

	// Loose crates in the road
	for ( int k = 0; k < 5; ++k )
	{
		float a = 1.5f + 0.02f * (float)( k % 3 );
		lpBegin();
		lpBox( b3Vec3_zero, (b3Vec3){ 0.35f, 0.35f, 0.35f }, b3Quat_identity, lp_wood, LP_PLANK, false );
		lpCommit( world, lpRingPoint( r - 1.5f + 1.0f * (float)k, a, 0.36f + 0.72f * (float)( k / 3 ) ), 0.3f * (float)k, false );
	}

	// A hump: up 0.7 m over 5 m, flat for 2 m, down again
	{
		float a = 2.6f;
		lpBegin();
		lpWedge( 4.0f, -6.0f, -1.0f, 0.7f, lp_stone, LP_STONE_DARK );
		lpBox( (b3Vec3){ 0.0f, 0.35f, 0.0f }, (b3Vec3){ 4.0f, 0.35f, 1.0f }, b3Quat_identity, lp_stone, LP_STONE_DARK, true );
		lpWedge( 4.0f, 6.0f, 1.0f, 0.7f, lp_stone, LP_STONE_DARK );
		lpCommit( world, lpRingPoint( r, a, 0.0f ), -a, true );
	}

	// A plank bridge: ramps up to two stone piers, an 8 m span of planks nailed across them
	{
		float a = 3.8f;
		b3Vec3 at = lpRingPoint( r, a, 0.0f );
		lpBegin();
		lpWedge( 2.2f, -10.0f, -5.0f, 0.9f, lp_stone, LP_STONE );
		lpCommit( world, at, -a, true );
		lpBegin();
		lpWedge( 2.2f, 10.0f, 5.0f, 0.9f, lp_stone, LP_STONE );
		lpCommit( world, at, -a, true );
		lpBegin();
		for ( int side = -1; side <= 1; side += 2 )
		{
			lpBox( (b3Vec3){ 0.0f, 0.4f, 4.5f * (float)side }, (b3Vec3){ 2.2f, 0.4f, 0.5f }, b3Quat_identity, lp_stone, LP_STONE, true );
		}
		for ( int k = 0; k < 4; ++k )
		{
			lpPartDef* plank = lpBox( (b3Vec3){ -1.35f + 0.9f * (float)k, 0.85f, 0.0f }, (b3Vec3){ 0.44f, 0.05f, 5.0f },
									  b3Quat_identity, lp_wood, LP_PLANK, false );
			plank->grainAxis = (b3Vec3){ 0.0f, 0.0f, 1.0f };
		}
		lpCommit( world, at, -a, true );
	}

	// A brick wall just outside the ring, for the cars that leave it
	{
		float a = 5.2f;
		b3CosSin cs = b3ComputeCosSin( a );
		float yaw = b3Atan2( -cs.cosine, -cs.sine ); // turns the wall's local x along the ring's tangent
		lpAddWall( world, lpRingPoint( r + 8.0f, a, 0.0f ), yaw, 10.0f, 1.6f, 0.3f, lp_brick, LP_BRICK, 1.0f );
	}

	// A crane in the infield, and three cars spread round the ring, driving counterclockwise
	lpAddCrane( world, (b3Vec3){ 0.0f, 0.0f, 0.0f }, 400.0f );
	lpAddCar( world, lpRingPoint( r, 0.0f, 0.0f ), -0.0f, 0 );
	lpAddCar( world, lpRingPoint( r, 2.1f, 0.0f ), -2.1f, 1 );
	lpAddCar( world, lpRingPoint( r, 4.5f, 0.0f ), -4.5f, 2 );
}

// Each car steers for a point a little ahead on the ring and holds the track speed
static void lpDriveTrack( lpWorld* world, int skipVehicle )
{
	int count = lpWorld_GetVehicleCapacity( world );
	for ( int v = 0; v < count; ++v )
	{
		lpVehicleState s = lpWorld_GetVehicleState( world, v );
		if ( v == skipVehicle || s.alive == false || s.body < 0 )
		{
			continue;
		}
		float angle = b3Atan2( (float)s.position.z, (float)s.position.x );
		b3Vec3 target = lpRingPoint( LP_TRACK_RADIUS, angle + LP_TRACK_LOOKAHEAD / LP_TRACK_RADIUS, 0.0f );
		float tx = target.x - (float)s.position.x;
		float tz = target.z - (float)s.position.z;
		// Signed angle from the car's heading to the target about +y: positive is to the left
		float error = b3Atan2( s.forward.z * tx - s.forward.x * tz, s.forward.x * tx + s.forward.z * tz );
		lpVehicleControl c = { 0 };
		c.steer = b3ClampFloat( -2.0f * error, -1.0f, 1.0f );
		c.throttle = b3ClampFloat( 0.5f * ( LP_TRACK_SPEED - s.speed ), -1.0f, 1.0f );
		lpWorld_SetVehicleControl( world, v, &c );
	}
}

int lpRigGrab( lpWorld* world, int rig, int limb )
{
	lpLimbState st = lpWorld_GetLimbState( world, rig, limb );
	if ( st.reaching == false || st.touching < 0 || st.footBody < 0 )
	{
		return -1;
	}
	lpLinkDef grip = lpDefaultLinkDef( lp_linkWeld );
	grip.bodyA = st.footBody;
	grip.bodyB = lpWorld_GetPieceInfo( world, st.touching ).body;
	grip.anchorA = st.foot;
	grip.maxForce = 40000.0f; // a claw's grip
	grip.maxTorque = 15000.0f;
	grip.strength = 5000.0f;
	return lpCreateLink( world, &grip );
}

// ---- the mech yard: a hexapod on patrol over rough ground ----

#define LP_MECH_LOOKAHEAD 5.0f // m along the patrol the walkers steer for

// The patrol: a loop round the yard, north up x = 0, east, south down x = 16, west
static const b3Vec3 lp_mechPatrol[4] = { { 0.0f, 0.0f, -20.0f }, { 0.0f, 0.0f, 20.0f }, { 16.0f, 0.0f, 20.0f }, { 16.0f, 0.0f, -20.0f } };

static void lpAddMechYard( lpWorld* world )
{
	lpAddGround( world, 50.0f );
	uint64_t rng = 0x3EC4ull;

	// North up x = 0: a 0.4 m concrete step, loose rubble, a hump up and down 15 degrees
	lpBegin();
	lpBox( (b3Vec3){ 0.0f, 0.2f, 0.0f }, (b3Vec3){ 5.0f, 0.2f, 3.0f }, b3Quat_identity, lp_concrete, LP_CONCRETE, true );
	lpCommit( world, (b3Vec3){ 0.0f, 0.0f, -9.0f }, 0.0f, true );
	for ( int k = 0; k < 14; ++k )
	{
		float size = 0.12f + 0.16f * lpUnit( &rng );
		b3Vec3 at = { -3.0f + 6.0f * lpUnit( &rng ), size, -2.0f + 4.0f * lpUnit( &rng ) };
		lpBegin();
		lpBox( b3Vec3_zero, (b3Vec3){ size, 0.8f * size, 1.2f * size }, b3Quat_identity, lp_stone, LP_STONE_DARK, false );
		lpCommit( world, at, 6.28f * lpUnit( &rng ), false );
	}
	lpBegin();
	lpWedge( 5.0f, 6.0f, 10.0f, 1.07f, lp_stone, LP_STONE );
	lpBox( (b3Vec3){ 0.0f, 0.535f, 11.0f }, (b3Vec3){ 5.0f, 0.535f, 1.0f }, b3Quat_identity, lp_stone, LP_STONE, true );
	lpWedge( 5.0f, 16.0f, 12.0f, 1.07f, lp_stone, LP_STONE );
	lpCommit( world, b3Vec3_zero, 0.0f, true );

	// East along z = 20: loose crates to wade through
	for ( int k = 0; k < 7; ++k )
	{
		lpBegin();
		lpBox( b3Vec3_zero, (b3Vec3){ 0.4f, 0.4f, 0.4f }, b3Quat_identity, lp_wood, LP_PLANK, false );
		lpCommit( world, (b3Vec3){ 6.0f + 0.9f * (float)( k % 4 ), 0.4f + 0.8f * (float)( k / 4 ), 19.0f + 0.7f * (float)( k % 3 ) },
				  0.4f * (float)k, false );
	}

	// South down x = 16: a brick wall on its right, a parked car on its left
	lpAddWall( world, (b3Vec3){ 12.5f, 0.0f, 0.0f }, 0.5f * B3_PI, 8.0f, 2.0f, 0.3f, lp_brick, LP_BRICK, 1.0f );
	lpAddCar( world, (b3Vec3){ 20.5f, 0.0f, -8.0f }, 0.0f, 3 );

	// The mech, at the start of the patrol, facing north
	lpAddHexapod( world, (b3Vec3){ 0.0f, 0.0f, -24.0f }, 0.0f, 0 );
}

// Each walker steers for a point a little further round the patrol than the nearest point on it, slowing to turn
static void lpDriveMech( lpWorld* world, int skipRig )
{
	for ( int ri = 0; ri < lpWorld_GetRigCapacity( world ); ++ri )
	{
		lpRigState s = lpWorld_GetRigState( world, ri );
		if ( ri == skipRig || s.alive == false || s.body < 0 )
		{
			continue;
		}
		b3Vec3 p = { (float)s.position.x, 0.0f, (float)s.position.z };
		int segment = 0;
		float along = 0.0f, nearest = FLT_MAX;
		for ( int k = 0; k < 4; ++k )
		{
			b3Vec3 a = lp_mechPatrol[k], b = lp_mechPatrol[( k + 1 ) % 4];
			b3Vec3 ab = b3Sub( b, a );
			float t = b3ClampFloat( b3Dot( b3Sub( p, a ), ab ) / b3Dot( ab, ab ), 0.0f, 1.0f );
			float d = b3Length( b3Sub( p, b3MulAdd( a, t, ab ) ) );
			if ( d < nearest )
			{
				nearest = d;
				segment = k;
				along = t * b3Length( ab );
			}
		}
		// Walk the lookahead along the loop
		float left = along + LP_MECH_LOOKAHEAD;
		b3Vec3 target = lp_mechPatrol[segment];
		for ( int k = 0; k < 4; ++k )
		{
			b3Vec3 a = lp_mechPatrol[( segment + k ) % 4], b = lp_mechPatrol[( segment + k + 1 ) % 4];
			float length = b3Length( b3Sub( b, a ) );
			if ( left <= length )
			{
				target = b3MulAdd( a, left / length, b3Sub( b, a ) );
				break;
			}
			left -= length;
		}
		float tx = target.x - p.x, tz = target.z - p.z;
		// Signed angle from the heading to the target about +y: positive is to the left
		float error = b3Atan2( s.forward.z * tx - s.forward.x * tz, s.forward.x * tx + s.forward.z * tz );
		lpRigControl c = { 0 };
		c.turn = b3ClampFloat( -1.5f * error, -1.0f, 1.0f );
		c.forward = b3ClampFloat( 1.0f - b3AbsFloat( error ), 0.2f, 1.0f );
		lpWorld_SetRigControl( world, ri, &c );
	}
}

void lpSceneDrive( lpWorld* world, int scene, int tick, int skipVehicle, int skipRig )
{
	if ( scene == lp_sceneTrack )
	{
		lpDriveTrack( world, skipVehicle );
		lpDriveCrane( world, tick );
	}
	else if ( scene == lp_sceneMech )
	{
		lpDriveMech( world, skipRig );
	}
}

static void lpAddPile( lpWorld* world, b3Vec3 center, int count, uint64_t seed )
{
	uint64_t rng = seed;
	int side = (int)ceilf( lpSceneCbrt( (float)count ) );
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

		case lp_sceneRuins:
			lpAddGround( world, 60.0f );
			lpAddArch( world, (b3Vec3){ -7.0f, 0.0f, -6.0f }, 2.0f, 0.5f, 1.0f, 9 );
			lpAddColonnade( world, (b3Vec3){ 0.0f, 0.0f, -6.0f }, 4, 2.6f, 3.0f );
			lpAddBalconies( world, (b3Vec3){ 12.0f, 0.0f, -6.0f } );
			break;

		case lp_sceneYard:
			lpAddGround( world, 60.0f );
			lpAddCartOnRamp( world, -8.0f );
			lpAddWall( world, (b3Vec3){ -3.2f, 0.0f, -8.0f }, 0.5f * B3_PI, 3.0f, 1.4f, 0.3f, lp_brick, LP_BRICK, 1.0f );
			lpAddPorterRack( world, (b3Vec3){ -0.6f, 0.0f, -6.0f } );
			lpAddHangingSign( world, (b3Vec3){ 1.5f, 0.0f, -8.0f } );
			lpAddDoorway( world, (b3Vec3){ 4.7f, 0.0f, -8.0f } );
			lpAddDrawbridge( world, (b3Vec3){ 7.9f, 0.0f, -8.0f } );
			break;

		case lp_sceneKeep:
			lpAddGround( world, 60.0f );
			lpAddKeep( world, (b3Vec3){ 0.0f, 0.0f, -10.0f }, 4 );
			break;

		case lp_sceneTrack:
			lpAddTrack( world );
			break;

		case lp_sceneMech:
			lpAddMechYard( world );
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
		case lp_sceneRuins:
		{
			// Across the whole row: the arch, the colonnade and the balcony wall
			origin = (b3Vec3){ 2.0f + 8.0f * ( lpUnit( &rng ) - 0.5f ), 1.8f, 10.0f };
			target = (b3Vec3){ -10.0f + 24.0f * lpUnit( &rng ), 0.3f + 3.7f * lpUnit( &rng ), -6.0f };
			break;
		}
		case lp_sceneYard:
		{
			// Across the whole yard, from the cart's ramp to the gatehouse
			origin = (b3Vec3){ -1.0f + 8.0f * ( lpUnit( &rng ) - 0.5f ), 1.8f, 10.0f };
			target = (b3Vec3){ -11.0f + 20.0f * lpUnit( &rng ), 0.3f + 3.2f * lpUnit( &rng ), -8.0f };
			break;
		}
		case lp_sceneKeep:
		{
			// Across the keep's front, from the foot of the wall to the parapet
			origin = (b3Vec3){ 8.0f * ( lpUnit( &rng ) - 0.5f ), 1.8f, 14.0f };
			target = (b3Vec3){ -7.0f + 14.0f * lpUnit( &rng ), 0.3f + 12.5f * lpUnit( &rng ), -2.5f };
			break;
		}
		case lp_sceneMech:
		{
			// At the walker's legs in turn, from beside it (at the patrol's start once it is gone)
			lpRigState rig = lpWorld_GetRigState( world, 0 );
			lpLimbState leg = lpWorld_GetLimbState( world, 0, shot % 6 );
			b3Vec3 at = { 0.0f, 0.8f, -20.0f };
			if ( leg.footBody >= 0 )
			{
				at = (b3Vec3){ (float)leg.foot.x, (float)leg.foot.y + 0.9f, (float)leg.foot.z };
			}
			else if ( rig.body >= 0 )
			{
				at = (b3Vec3){ (float)rig.position.x, (float)rig.position.y, (float)rig.position.z };
			}
			target = b3Add( at, (b3Vec3){ 0.4f * ( lpUnit( &rng ) - 0.5f ), 0.3f * ( lpUnit( &rng ) - 0.5f ), 0.4f * ( lpUnit( &rng ) - 0.5f ) } );
			float side = shot % 2 == 0 ? 1.0f : -1.0f;
			origin = b3Add( target, (b3Vec3){ 9.0f * side, 0.8f, 3.0f * ( lpUnit( &rng ) - 0.5f ) } );
			break;
		}
		case lp_sceneTrack:
		{
			// At the cars in turn, from the infield (at the road ahead of a car that is gone)
			int count = lpWorld_GetVehicleCapacity( world );
			lpVehicleState car = lpWorld_GetVehicleState( world, count > 0 ? shot % count : 0 );
			b3Vec3 at = car.body >= 0 ? b3Vec3_zero : lpRingPoint( LP_TRACK_RADIUS, 0.4f * (float)shot, 0.5f );
			if ( car.body >= 0 )
			{
				at = (b3Vec3){ (float)car.position.x, (float)car.position.y, (float)car.position.z };
			}
			target = b3Add( at, (b3Vec3){ 2.0f * ( lpUnit( &rng ) - 0.5f ), 0.3f * lpUnit( &rng ), 2.0f * ( lpUnit( &rng ) - 0.5f ) } );
			origin = (b3Vec3){ 0.7f * target.x, 1.8f, 0.7f * target.z };
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
	lpWorld_AddImpact( world, &impact );
	return true;
}
