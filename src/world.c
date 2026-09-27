// SPDX-License-Identifier: MIT
// The destruction world: pieces, bonds and bodies on top of a Box3D world.
//
// Invariants (checked by lpWorld_Validate in tests):
// - every live piece belongs to exactly one live body and owns one Box3D hull shape on that body
// - bonds only join pieces of the same body; a piece's bond list holds exactly its live bonds
// - body frames never change when pieces move between bodies: a split-off body is created at the parent's
//   transform, so piece geometry stays in the original object frame for its whole life (no drift, and solid
//   interior colors line up across cuts)

#include "facet.h"
#include "fracture.h"
#include "tasks.h"
#include "world.h"

#include <float.h>
#include <math.h>
#include <stdio.h>

// Strengths are damage densities: an impact of energy E and radius R delivers E (1 - d/R)^2 / (pi R^2) at
// distance d. Calibration: a rifle round (E 4 kJ, R 0.35 m, center 10.4 kJ/m^2) chips ~15 cm out of brick in one
// shot; a grenade (E 80 kJ, R 1.4 m, center 13 kJ/m^2) opens ~0.7 m in brick and ~0.3 m in stone.
static lpMaterialDef lp_materials[lp_materialCount] = {
	[lp_wood] = { "wood", 600.0f, 2000.0f, 1600.0f, 0.12f, 0.6f, 0.05f, lp_breakGrain, 5.0f, 0xE0B070u, true, 3.0e5f },
	[lp_stone] = { "stone", 2400.0f, 8000.0f, 6400.0f, 0.18f, 0.7f, 0.02f, lp_breakImpact, 1.0f, 0x9A968Cu, true, 5.0e5f },
	[lp_brick] = { "brick", 1900.0f, 3000.0f, 2400.0f, 0.14f, 0.7f, 0.02f, lp_breakImpact, 1.0f, 0xC8704Au, true, 4.0e5f },
	[lp_plaster] = { "plaster", 1200.0f, 1200.0f, 1000.0f, 0.12f, 0.6f, 0.02f, lp_breakImpact, 1.0f, 0xEEE6D2u, true, 3.0e5f },
	[lp_concrete] = { "concrete", 2400.0f, 10000.0f, 8000.0f, 0.2f, 0.7f, 0.02f, lp_breakImpact, 1.0f, 0xA5A39Cu, true, 1.0e6f },
	[lp_glass] = { "glass", 2500.0f, 300.0f, 240.0f, 0.07f, 0.4f, 0.05f, lp_breakRadial, 1.0f, 0xC6EEF2u, true, 5.0e4f },
	[lp_metal] = { "metal", 7800.0f, 1e9f, 1e9f, 0.3f, 0.5f, 0.1f, lp_breakImpact, 1.0f, 0x70757Bu, false, 1e12f },
	[lp_ground] = { "ground", 2000.0f, 1e9f, 1e9f, 1.0f, 0.8f, 0.0f, lp_breakImpact, 1.0f, 0x6E5B45u, false, 1e12f },
	[lp_foliage] = { "foliage", 150.0f, 300.0f, 240.0f, 0.35f, 0.8f, 0.0f, lp_breakImpact, 1.0f, 0x4E8C3Au, true, 2.0e5f },
};

const lpMaterialDef* lpGetMaterial( int materialId )
{
	LP_ASSERT( 0 <= materialId && materialId < lp_materialCount );
	return lp_materials + materialId;
}

void lpSetMaterial( int materialId, const lpMaterialDef* def )
{
	LP_ASSERT( 0 <= materialId && materialId < lp_materialCount );
	lp_materials[materialId] = *def;
}

lpWorldDef lpDefaultWorldDef( void )
{
	lpWorldDef def = { 0 };
	def.seed = 1;
	def.maxDebrisBodies = 1500;
	def.maxRubblePieces = 20000;
	def.fragmentScale = 1.0f;
	def.freezeRubble = true;
	def.minPieceVolume = 0.0004f; // a 7 cm cube
	def.maxDepth = 3;
	def.maxHitImpacts = 16;
	def.hitSpeed = 4.0f;
	def.killDepth = -50.0f;
	def.workerCount = 1;
	def.stressScale = 1.0f;
	return def;
}

lpObjectDef lpDefaultObjectDef( void )
{
	lpObjectDef def = { 0 };
	def.transform = b3Transform_identity;
	def.isStatic = true;
	return def;
}

lpPartDef lpDefaultPartDef( void )
{
	lpPartDef def = { 0 };
	def.halfExtents = (b3Vec3){ 0.5f, 0.5f, 0.5f };
	def.transform = b3Transform_identity;
	def.material = lp_stone;
	def.color = 0x9A968Cu;
	return def;
}

// ---- slots ----

static int lpAllocPiece( lpWorld* w )
{
	int index;
	if ( w->freePiece != -1 )
	{
		index = w->freePiece;
		w->freePiece = w->pieces.data[index].nextFree;
		uint32_t generation = w->pieces.data[index].generation;
		memset( w->pieces.data + index, 0, sizeof( lpPiece ) );
		w->pieces.data[index].generation = generation + 1;
	}
	else
	{
		index = w->pieces.count;
		lpPiece zero = { 0 };
		lpArray_Push( w->pieces, zero );
	}
	lpPiece* p = w->pieces.data + index;
	p->body = -1;
	p->nextFree = -1;
	p->shapeId = b3_nullShapeId;
	p->meshVersion = 1;
	return index;
}

static void lpFreePieceSlot( lpWorld* w, int index )
{
	lpPiece* p = w->pieces.data + index;
	lpShape_Destroy( p->shape );
	if ( p->hull != NULL )
	{
		b3DestroyHull( p->hull );
	}
	lpArray_Free( p->bonds );
	p->shape = NULL;
	p->hull = NULL;
	p->body = -1;
	p->nextFree = w->freePiece;
	w->freePiece = index;
}

static int lpAllocBond( lpWorld* w )
{
	int index;
	if ( w->freeBond != -1 )
	{
		index = w->freeBond;
		w->freeBond = w->bonds.data[index].nextFree;
	}
	else
	{
		index = w->bonds.count;
		lpBond zero = { 0 };
		lpArray_Push( w->bonds, zero );
	}
	w->bonds.data[index].nextFree = -1;
	return index;
}

static int lpAllocBody( lpWorld* w )
{
	int index;
	if ( w->freeBody != -1 )
	{
		index = w->freeBody;
		w->freeBody = w->bodies.data[index].nextFree;
		lpBody* b = w->bodies.data + index;
		uint32_t generation = b->generation;
		int* data = b->pieces.data;
		int capacity = b->pieces.capacity;
		memset( b, 0, sizeof( lpBody ) );
		b->pieces.data = data;
		b->pieces.capacity = capacity;
		b->generation = generation + 1;
	}
	else
	{
		index = w->bodies.count;
		lpBody zero = { 0 };
		lpArray_Push( w->bodies, zero );
	}
	lpBody* b = w->bodies.data + index;
	b->alive = true;
	b->nextFree = -1;
	b->createdTick = w->tick;
	return index;
}

// ---- world ----

lpWorld* lpCreateWorld( const lpWorldDef* def )
{
	lpWorld* w = lpAlloc( sizeof( lpWorld ) );
	memset( w, 0, sizeof( lpWorld ) );
	w->def = *def;
	w->freePiece = -1;
	w->freeBond = -1;
	w->freeBody = -1;
	b3World_SetHitEventThreshold( def->physics, def->hitSpeed );
	w->tasks = lpTaskPool_Create( def->workerCount );
	return w;
}

void lpDestroyWorld( lpWorld* w )
{
	bool physicsAlive = b3World_IsValid( w->def.physics );
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		if ( b->alive && physicsAlive )
		{
			b3DestroyBody( b->id );
		}
		lpArray_Free( b->pieces );
	}
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		lpPiece* p = w->pieces.data + i;
		if ( p->body >= 0 )
		{
			lpShape_Destroy( p->shape );
			if ( p->hull != NULL )
			{
				b3DestroyHull( p->hull );
			}
		}
		lpArray_Free( p->bonds );
	}
	lpArray_Free( w->pieces );
	lpArray_Free( w->bonds );
	lpArray_Free( w->bodies );
	lpArray_Free( w->impacts );
	lpArray_Free( w->nextImpacts );
	lpArray_Free( w->particles );
	lpArray_Free( w->scratchPieces );
	lpArray_Free( w->scratchBodies );
	lpArray_Free( w->scratchQueue );
	lpArray_Free( w->dirtyBodies );
	lpArray_Free( w->forces );
	lpArray_Free( w->pendingWakes );
	lpArray_Free( w->freezeCandidates );
	lpArray_Free( w->pendingDestroy );
	lpArray_Free( w->scratchLoad );
	lpArray_Free( w->stressAgain );
	for ( int i = 0; i < w->jobCapacity; ++i )
	{
		lpFree( w->jobs[i].bonds );
	}
	lpFree( w->jobs );
	lpTaskPool_Destroy( w->tasks );
	lpArray_Free( w->pulls );
	lpArray_Free( w->scratchComponents );
	lpFree( w );
}

// ---- pieces and bodies ----

static b3ShapeDef lpMakeShapeDef( int pieceIndex, uint8_t material )
{
	const lpMaterialDef* m = lpGetMaterial( material );
	b3ShapeDef def = b3DefaultShapeDef();
	def.density = m->density;
	def.baseMaterial.friction = m->friction;
	def.baseMaterial.restitution = m->restitution;
	def.baseMaterial.userMaterialId = material;
	def.enableHitEvents = m->breakable;
	def.updateBodyMass = false;
	def.userData = (void*)(intptr_t)( pieceIndex + 1 );
	return def;
}

static bool lpAttachPiece( lpWorld* w, int pieceIndex, int bodyIndex )
{
	lpPiece* p = w->pieces.data + pieceIndex;
	lpBody* b = w->bodies.data + bodyIndex;
	uint64_t t0 = b3GetTicks();
	if ( p->hull == NULL )
	{
		p->hull = lpShape_CreateHull( p->shape );
		if ( p->hull == NULL )
		{
			return false;
		}
	}
	uint64_t t1 = b3GetTicks();
	b3ShapeDef def = lpMakeShapeDef( pieceIndex, p->material );
	p->shapeId = b3CreateHullShape( b->id, &def, p->hull );
	w->stats.hullMs += b3GetMilliseconds( t0 ) - b3GetMilliseconds( t1 );
	w->stats.shapeMs += b3GetMilliseconds( t1 );
	p->body = bodyIndex;
	lpArray_Push( b->pieces, pieceIndex );
	b->volume += p->shape->volume;
	return true;
}

static void lpDetachPieceShape( lpWorld* w, int pieceIndex )
{
	lpPiece* p = w->pieces.data + pieceIndex;
	if ( B3_IS_NON_NULL( p->shapeId ) )
	{
		b3DestroyShape( p->shapeId, false );
		p->shapeId = b3_nullShapeId;
	}
}

static void lpRemoveBondFromPiece( lpPiece* p, int bondIndex )
{
	for ( int i = 0; i < p->bonds.count; ++i )
	{
		if ( p->bonds.data[i] == bondIndex )
		{
			// keep order for determinism of later traversals
			memmove( p->bonds.data + i, p->bonds.data + i + 1, sizeof( int ) * (size_t)( p->bonds.count - i - 1 ) );
			p->bonds.count -= 1;
			return;
		}
	}
	LP_ASSERT( false );
}

static void lpBreakBond( lpWorld* w, int bondIndex )
{
	lpBond* bond = w->bonds.data + bondIndex;
	LP_ASSERT( bond->alive );
	lpRemoveBondFromPiece( w->pieces.data + bond->a, bondIndex );
	lpRemoveBondFromPiece( w->pieces.data + bond->b, bondIndex );
	bond->alive = false;
	bond->nextFree = w->freeBond;
	w->freeBond = bondIndex;
	w->bondCount -= 1;
}

static void lpAddBond( lpWorld* w, int a, int b, float area, b3Vec3 centroid )
{
	lpPiece* pa = w->pieces.data + a;
	lpPiece* pb = w->pieces.data + b;
	float strengthA = lpGetMaterial( pa->material )->bondStrength;
	float strengthB = lpGetMaterial( pb->material )->bondStrength;

	int index = lpAllocBond( w );
	lpBond* bond = w->bonds.data + index;
	bond->a = a < b ? a : b;
	bond->b = a < b ? b : a;
	bond->area = area;
	bond->centroid = centroid;
	bond->health = strengthA < strengthB ? strengthA : strengthB;
	bond->strength = bond->health;
	float loadA = lpGetMaterial( pa->material )->loadStrength;
	float loadB = lpGetMaterial( pb->material )->loadStrength;
	bond->loadStrength = loadA < loadB ? loadA : loadB;
	bond->alive = true;
	lpArray_Push( pa->bonds, index );
	lpArray_Push( pb->bonds, index );
	w->bondCount += 1;
}

// Bond two pieces of one body if they share a face. Parts that meet at an angle (a roof plank on a gable) share no
// coplanar face; if they touch or interpenetrate they are welded with a nominal area instead.
static void lpTryBond( lpWorld* w, int a, int b )
{
	lpPiece* pa = w->pieces.data + a;
	lpPiece* pb = w->pieces.data + b;
	const float tolerance = 2e-3f;
	const float weldMargin = 0.03f;
	b3AABB ba = pa->shape->bounds;
	b3AABB bb = pb->shape->bounds;
	if ( ba.lowerBound.x > bb.upperBound.x + weldMargin || bb.lowerBound.x > ba.upperBound.x + weldMargin ||
		 ba.lowerBound.y > bb.upperBound.y + weldMargin || bb.lowerBound.y > ba.upperBound.y + weldMargin ||
		 ba.lowerBound.z > bb.upperBound.z + weldMargin || bb.lowerBound.z > ba.upperBound.z + weldMargin )
	{
		return;
	}

	b3Vec3 centroid, normal;
	float area = lpShape_ContactArea( pa->shape, pb->shape, tolerance, &centroid, &normal );
	if ( area >= 1e-4f )
	{
		lpAddBond( w, a, b, area, centroid );
		return;
	}

	if ( lpShape_NearlyOverlap( pa->shape, pb->shape, weldMargin ) )
	{
		float v = pa->shape->volume < pb->shape->volume ? pa->shape->volume : pb->shape->volume;
		float nominal = 0.5f * cbrtf( v * v );
		lpAddBond( w, a, b, nominal, b3MulSV( 0.5f, b3Add( pa->shape->centroid, pb->shape->centroid ) ) );
	}
}

// Emit a cosmetic particle at a body-frame point
static void lpEmitParticle( lpWorld* w, b3WorldTransform xf, b3Vec3 localPoint, b3Vec3 velocity, float size, uint32_t rgb )
{
	b3Pos p = b3TransformWorldPoint( xf, localPoint );
	lpParticle particle;
	particle.position[0] = (float)p.x;
	particle.position[1] = (float)p.y;
	particle.position[2] = (float)p.z;
	particle.velocity[0] = velocity.x;
	particle.velocity[1] = velocity.y;
	particle.velocity[2] = velocity.z;
	particle.size = size;
	particle.color = 0xFF000000u | ( ( rgb & 0xFF ) << 16 ) | ( rgb & 0xFF00 ) | ( ( rgb >> 16 ) & 0xFF );
	lpArray_Push( w->particles, particle );
}

static void lpDestroyBody( lpWorld* w, int bodyIndex, bool emitDust )
{
	lpBody* b = w->bodies.data + bodyIndex;
	LP_ASSERT( b->alive );
	b3WorldTransform xf = b3Body_GetTransform( b->id );

	for ( int i = 0; i < b->pieces.count; ++i )
	{
		int pieceIndex = b->pieces.data[i];
		lpPiece* p = w->pieces.data + pieceIndex;
		while ( p->bonds.count > 0 )
		{
			lpBreakBond( w, p->bonds.data[p->bonds.count - 1] );
		}
		if ( emitDust )
		{
			float size = cbrtf( p->shape->volume );
			lpEmitParticle( w, xf, p->shape->centroid, b3Vec3_zero, size, lpGetMaterial( p->material )->interiorColor );
		}
		p->shapeId = b3_nullShapeId; // destroyed with the body
		lpFreePieceSlot( w, pieceIndex );
	}
	b->pieces.count = 0;

	b3DestroyBody( b->id );
	b->alive = false;
	b->id = b3_nullBodyId;
	b->nextFree = w->freeBody;
	w->freeBody = bodyIndex;
}

static int lpCreateBodyInternal( lpWorld* w, b3WorldTransform xf, b3BodyType type, uint8_t kind, b3Vec3 v, b3Vec3 omega )
{
	int index = lpAllocBody( w );
	lpBody* b = w->bodies.data + index;
	b3BodyDef def = b3DefaultBodyDef();
	def.type = type;
	def.position = xf.p;
	def.rotation = xf.q;
	def.linearVelocity = v;
	def.angularVelocity = omega;
	def.userData = (void*)(intptr_t)( index + 1 );
	b->id = b3CreateBody( w->def.physics, &def );
	b->kind = kind;
	return index;
}

static void lpMarkDirty( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->dirty == false )
	{
		b->dirty = true;
		lpArray_Push( w->dirtyBodies, bodyIndex );
	}
}

static b3Vec3 lpBoxAxis( b3Vec3 h, b3Quat q, bool longest )
{
	b3Vec3 axis = { 1.0f, 0.0f, 0.0f };
	float best = h.x;
	if ( longest ? h.y > best : h.y < best )
	{
		axis = (b3Vec3){ 0.0f, 1.0f, 0.0f };
		best = h.y;
	}
	if ( longest ? h.z > best : h.z < best )
	{
		axis = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	}
	return b3RotateVector( q, axis );
}

int lpCreateObject( lpWorld* w, const lpObjectDef* def )
{
	b3BodyType type = def->isStatic ? b3_staticBody : b3_dynamicBody;
	uint8_t kind = def->isStatic ? lp_kindStructure : lp_kindDebris;
	int bodyIndex = lpCreateBodyInternal( w, def->transform, type, kind, def->linearVelocity, def->angularVelocity );
	w->bodies.data[bodyIndex].detonator = def->detonator;
	w->bodies.data[bodyIndex].armed = def->detonator.radius > 0.0f;

	lpPoly* poly = lpAlloc( sizeof( lpPoly ) );
	int first = w->bodies.data[bodyIndex].pieces.count;

	for ( int i = 0; i < def->partCount; ++i )
	{
		const lpPartDef* part = def->parts + i;
		bool ok;
		b3Vec3 extents = { 0.5f, 0.5f, 0.5f };
		if ( part->pointCount == 0 )
		{
			lpPoly_MakeBox( poly, part->halfExtents, part->transform, part->material );
			ok = true;
			extents = part->halfExtents;
		}
		else
		{
			ok = lpPoly_MakeFromPoints( poly, part->points, part->pointCount, part->material );
			if ( ok )
			{
				b3AABB box = lpPoly_ComputeBounds( poly );
				extents = b3MulSV( 0.5f, b3Sub( box.upperBound, box.lowerBound ) );
			}
		}
		if ( ok == false )
		{
			continue;
		}

		lpShape* shape = lpShape_Create( poly );
		if ( shape == NULL )
		{
			continue;
		}

		int pieceIndex = lpAllocPiece( w );
		lpPiece* p = w->pieces.data + pieceIndex;
		p->shape = shape;
		p->material = part->material;
		p->color = part->color;
		p->seed = (uint32_t)lpMix64( w->def.seed ^ ( (uint64_t)pieceIndex << 20 ) ^ w->pieceSerial++ );

		b3Quat q = part->pointCount == 0 ? part->transform.q : b3Quat_identity;
		bool glass = lpGetMaterial( part->material )->pattern == lp_breakRadial;
		if ( b3LengthSquared( part->grainAxis ) > 0.0f )
		{
			p->axis = b3Normalize( part->grainAxis );
		}
		else
		{
			p->axis = lpBoxAxis( extents, q, glass == false );
		}

		if ( part->anchored )
		{
			// Anchor through the face that points most downward (object space -y)
			int best = 0;
			float bestDot = FLT_MAX;
			for ( int f = 0; f < shape->faceCount; ++f )
			{
				float d = shape->faces[f].plane.normal.y;
				if ( d < bestDot )
				{
					bestDot = d;
					best = f;
				}
			}
			p->anchored = true;
			p->anchorPlane = shape->faces[best].plane;
		}

		if ( lpAttachPiece( w, pieceIndex, bodyIndex ) == false )
		{
			lpFreePieceSlot( w, pieceIndex );
		}
	}
	lpFree( poly );

	// Bond touching parts
	lpBody* b = w->bodies.data + bodyIndex;
	for ( int i = first; i < b->pieces.count; ++i )
	{
		for ( int j = i + 1; j < b->pieces.count; ++j )
		{
			lpTryBond( w, b->pieces.data[i], b->pieces.data[j] );
		}
	}

	if ( type == b3_dynamicBody )
	{
		b3Body_ApplyMassFromShapes( b->id );
	}
	return bodyIndex;
}

void lpWorld_AddImpact( lpWorld* w, const lpImpactDef* impact )
{
	lpArray_Push( w->impacts, *impact );
}

// ---- impacts ----

typedef struct lpOverlapContext
{
	lpWorld* world;
} lpOverlapContext;

static bool lpCollectPieceFcn( b3ShapeId shapeId, void* context )
{
	lpWorld* w = ( (lpOverlapContext*)context )->world;
	intptr_t data = (intptr_t)b3Shape_GetUserData( shapeId );
	if ( data > 0 )
	{
		lpArray_Push( w->scratchPieces, (int)( data - 1 ) );
	}
	return true;
}

static int lpCompareInt( const void* a, const void* b )
{
	int x = *(const int*)a;
	int y = *(const int*)b;
	return ( x > y ) - ( x < y );
}

// Pieces whose shapes overlap the box, sorted and unique (query order must not leak into results).
static void lpQueryPieces( lpWorld* w, b3AABB box )
{
	w->scratchPieces.count = 0;
	lpOverlapContext context = { w };
	b3World_OverlapAABB( w->def.physics, box, b3DefaultQueryFilter(), lpCollectPieceFcn, &context );
	if ( w->scratchPieces.count > 1 )
	{
		qsort( w->scratchPieces.data, (size_t)w->scratchPieces.count, sizeof( int ), lpCompareInt );
		int unique = 1;
		for ( int i = 1; i < w->scratchPieces.count; ++i )
		{
			if ( w->scratchPieces.data[i] != w->scratchPieces.data[unique - 1] )
			{
				w->scratchPieces.data[unique++] = w->scratchPieces.data[i];
			}
		}
		w->scratchPieces.count = unique;
	}
}

static void lpWakeRubble( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->kind != lp_kindRubble )
	{
		return;
	}
	b->kind = lp_kindDebris;
	b3Body_SetType( b->id, b3_dynamicBody );
	b3Body_ApplyMassFromShapes( b->id );
	b3Body_SetAwake( b->id, true );
}

static b3AABB lpInflatedBox( b3Pos center, float r )
{
	b3Vec3 c = b3ToVec3( center );
	return (b3AABB){ b3Sub( c, (b3Vec3){ r, r, r } ), b3Add( c, (b3Vec3){ r, r, r } ) };
}

// Energy density (J/m^2) delivered at distance d from an impact
static float lpImpactDensity( const lpImpactDef* impact, float d )
{
	if ( d >= impact->radius )
	{
		return 0.0f;
	}
	float x = d < 0.0f ? 0.0f : d / impact->radius;
	float f = ( 1.0f - x ) * ( 1.0f - x );
	return impact->energy * f / ( B3_PI * impact->radius * impact->radius );
}

// ---- fracture jobs ----
//
// An impact fractures its pieces in three phases, which keeps the result independent of the thread count:
// 1. choose the pieces and snapshot their inputs (sequential, in piece order)
// 2. compute cells, Box3D hulls and sibling bonds for every piece (parallel; each job is a pure function of its input)
// 3. swap parents for their cells (sequential, in job order)

static void lpPrepareFractureJob( lpWorld* w, lpFractureJob* job, int pieceIndex, b3Vec3 localImpact, const lpImpactDef* impact )
{
	lpPiece* piece = w->pieces.data + pieceIndex;
	const lpMaterialDef* m = lpGetMaterial( piece->material );
	float fragment = m->fragmentSize * w->def.fragmentScale;

	// Radius inside which bonds will break: (1 - x)^2 >= strength * pi R^2 / E
	float ratio = m->bondStrength * B3_PI * impact->radius * impact->radius / impact->energy;
	float xb = ratio < 1.0f ? 1.0f - sqrtf( ratio ) : 0.0f;
	float breakRadius = b3MaxFloat( impact->radius * xb, 1.5f * fragment );

	job->piece = pieceIndex;
	job->localImpact = localImpact;
	job->center = piece->shape->centroid;
	lpShape_ToPoly( piece->shape, &job->poly );
	lpPoly_Translate( &job->poly, b3Neg( job->center ) );
	job->minPieceVolume = w->def.minPieceVolume;
	job->cellCount = 0;
	job->bondCount = 0;
	memset( &job->stats, 0, sizeof( job->stats ) );

	lpFractureInput* input = &job->input;
	memset( input, 0, sizeof( *input ) );
	input->parent = &job->poly;
	input->impact = b3Sub( localImpact, job->center );
	input->radius = breakRadius;
	input->fragmentSize = fragment;
	input->maxCells = 64;
	input->pattern = m->pattern == lp_breakGrain ? lp_patternGrain : ( m->pattern == lp_breakRadial ? lp_patternRadial : lp_patternImpact );
	input->axis = piece->axis;
	input->stretch = m->grainStretch;
	input->interiorMaterial = piece->material;
	input->seed = lpMix64( w->def.seed ^ ( w->tick << 24 ) ^ ( (uint64_t)pieceIndex << 1 ) ^ piece->generation );
	input->tolerance = 2e-5f;
}

// Phase 2. Must not touch the world.
static void lpRunFractureJob( int index, int worker, void* context )
{
	(void)worker;
	lpFractureJob* job = (lpFractureJob*)context + index;
	job->input.parent = &job->poly; // the job array may have moved since the job was prepared
	job->cellCount = lpFracture( &job->input, job->cells, job->cellSites, LP_MAX_SITES, &job->stats );
	if ( job->cellCount < 2 )
	{
		return;
	}
	for ( int i = 0; i < job->cellCount; ++i )
	{
		lpShape_Translate( job->cells[i], job->center );
		job->hulls[i] = job->cells[i]->volume >= job->minPieceVolume ? lpShape_CreateHull( job->cells[i] ) : NULL;
	}
	job->bondCount = lpFindCellBonds( job->cells, job->cellSites, job->cellCount, job->bonds, LP_MAX_CELL_BONDS );
}

static void lpFreeJobOutput( lpFractureJob* job )
{
	for ( int i = 0; i < job->cellCount; ++i )
	{
		if ( job->cells[i] != NULL )
		{
			lpShape_Destroy( job->cells[i] );
		}
		if ( job->hulls[i] != NULL )
		{
			b3DestroyHull( job->hulls[i] );
		}
	}
	job->cellCount = 0;
}

// Phase 3: replace the parent piece by its cells. Ownership of cells and hulls moves to the new pieces.
static void lpIntegrateFractureJob( lpWorld* w, lpFractureJob* job )
{
	w->stats.clipFailures += job->stats.failureCount;
	int pieceIndex = job->piece;
	lpPiece* piece = w->pieces.data + pieceIndex;
	if ( job->cellCount < 2 || piece->body < 0 )
	{
		lpFreeJobOutput( job );
		return;
	}

	const lpMaterialDef* m = lpGetMaterial( piece->material );
	int bodyIndex = piece->body;
	lpBody* body = w->bodies.data + bodyIndex;
	b3WorldTransform xf = b3Body_GetTransform( body->id );

	// Former neighbors, then retire the parent
	int neighbors[256];
	int neighborCount = 0;
	while ( piece->bonds.count > 0 )
	{
		int bondIndex = piece->bonds.data[piece->bonds.count - 1];
		lpBond* bond = w->bonds.data + bondIndex;
		int other = bond->a == pieceIndex ? bond->b : bond->a;
		if ( neighborCount < 256 )
		{
			neighbors[neighborCount++] = other;
		}
		lpBreakBond( w, bondIndex );
	}

	uint8_t material = piece->material;
	uint32_t color = piece->color;
	b3Vec3 axis = piece->axis;
	int depth = piece->depth + 1;
	bool anchored = piece->anchored;
	b3Plane anchorPlane = piece->anchorPlane;

	uint64_t shapeTicks = b3GetTicks();
	lpDetachPieceShape( w, pieceIndex );
	w->stats.shapeMs += b3GetMilliseconds( shapeTicks );
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		if ( body->pieces.data[i] == pieceIndex )
		{
			memmove( body->pieces.data + i, body->pieces.data + i + 1, sizeof( int ) * (size_t)( body->pieces.count - i - 1 ) );
			body->pieces.count -= 1;
			break;
		}
	}
	body->volume -= piece->shape->volume;
	lpFreePieceSlot( w, pieceIndex );
	piece = NULL;

	int cellToPiece[LP_MAX_SITES];
	int children[LP_MAX_SITES];
	int childCount = 0;
	for ( int i = 0; i < job->cellCount; ++i )
	{
		lpShape* cell = job->cells[i];
		job->cells[i] = NULL;
		cellToPiece[i] = -1;

		if ( job->hulls[i] == NULL )
		{
			if ( cell->volume < w->def.minPieceVolume )
			{
				b3Vec3 away = b3Sub( cell->centroid, job->localImpact );
				b3Vec3 v = b3MulSV( 2.0f, b3Normalize( away ) );
				lpEmitParticle( w, xf, cell->centroid, b3RotateVector( xf.q, v ), cbrtf( cell->volume ), m->interiorColor );
			}
			lpShape_Destroy( cell );
			continue;
		}

		int childIndex = lpAllocPiece( w );
		lpPiece* child = w->pieces.data + childIndex;
		child->shape = cell;
		child->hull = job->hulls[i];
		job->hulls[i] = NULL;
		child->material = material;
		child->color = color;
		child->axis = axis;
		child->depth = (uint8_t)( depth > 255 ? 255 : depth );
		child->seed = (uint32_t)lpMix64( w->def.seed ^ w->pieceSerial++ );
		child->anchorPlane = anchorPlane;
		child->anchored = anchored && lpShape_HasFaceOnPlane( cell, anchorPlane, 1e-3f );

		if ( lpAttachPiece( w, childIndex, bodyIndex ) == false )
		{
			lpFreePieceSlot( w, childIndex );
			continue;
		}
		cellToPiece[i] = childIndex;
		children[childCount++] = childIndex;
	}
	job->cellCount = 0;

	uint64_t bondTicks = b3GetTicks();
	for ( int i = 0; i < job->bondCount; ++i )
	{
		lpCellBond cb = job->bonds[i];
		int a = cellToPiece[cb.a];
		int b = cellToPiece[cb.b];
		if ( a >= 0 && b >= 0 )
		{
			lpAddBond( w, a, b, cb.area, cb.centroid );
		}
	}
	for ( int i = 0; i < childCount; ++i )
	{
		for ( int n = 0; n < neighborCount; ++n )
		{
			lpTryBond( w, children[i], neighbors[n] );
		}
	}
	w->stats.bondMs += b3GetMilliseconds( bondTicks );

	if ( body->kind != lp_kindStructure )
	{
		b3Body_ApplyMassFromShapes( body->id );
	}

	w->stats.fracturesThisStep += 1;
	w->stats.cellsThisStep += childCount;
	lpMarkDirty( w, bodyIndex );
}

// Queue the blast of an armed body for the next step and remove the body then.
static void lpDetonate( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->alive == false || b->armed == false )
	{
		return;
	}
	b->armed = false;
	lpImpactDef blast = { 0 };
	blast.point = b3Body_GetWorldCenter( b->id );
	blast.radius = b->detonator.radius;
	blast.energy = b->detonator.energy;
	blast.impulse = b->detonator.speed;
	blast.explosion = true;
	lpArray_Push( w->nextImpacts, blast );
	lpArray_Push( w->pendingDestroy, bodyIndex );
}

static void lpProcessImpact( lpWorld* w, const lpImpactDef* impact )
{
	if ( impact->radius <= 0.0f || impact->energy <= 0.0f )
	{
		return;
	}

	lpQueryPieces( w, lpInflatedBox( impact->point, impact->radius ) );

	// Copy: fracturing reuses the scratch query array
	int candidateCount = w->scratchPieces.count;
	int* candidates = lpAlloc( sizeof( int ) * (size_t)( candidateCount > 0 ? candidateCount : 1 ) );
	memcpy( candidates, w->scratchPieces.data, sizeof( int ) * (size_t)candidateCount );

	// Anything frozen nearby comes back to life; volatile things caught in the blast go off (next step)
	for ( int i = 0; i < candidateCount; ++i )
	{
		lpPiece* p = w->pieces.data + candidates[i];
		if ( p->body < 0 )
		{
			continue;
		}
		lpWakeRubble( w, p->body );
		lpBody* b = w->bodies.data + p->body;
		if ( b->armed )
		{
			b3WorldTransform xf = b3Body_GetTransform( b->id );
			float d = lpShape_SignedDistance( p->shape, b3InvTransformWorldPoint( xf, impact->point ) );
			if ( lpImpactDensity( impact, d ) > 150.0f )
			{
				lpDetonate( w, p->body );
			}
		}
	}

	// Refracture struck pieces (see the phases above)
	w->jobCount = 0;
	for ( int i = 0; i < candidateCount; ++i )
	{
		int pieceIndex = candidates[i];
		lpPiece* p = w->pieces.data + pieceIndex;
		if ( p->body < 0 || p->shape == NULL )
		{
			continue;
		}
		const lpMaterialDef* m = lpGetMaterial( p->material );
		if ( m->breakable == false || p->depth >= w->def.maxDepth )
		{
			continue;
		}
		float fragment = m->fragmentSize * w->def.fragmentScale;
		if ( p->shape->radius < 1.5f * fragment )
		{
			continue;
		}

		b3WorldTransform xf = b3Body_GetTransform( w->bodies.data[p->body].id );
		b3Vec3 local = b3InvTransformWorldPoint( xf, impact->point );
		float d = lpShape_SignedDistance( p->shape, local );
		if ( lpImpactDensity( impact, d ) >= m->fractureEnergy )
		{
			if ( w->jobCount == w->jobCapacity )
			{
				int capacity = w->jobCapacity < 8 ? 8 : 2 * w->jobCapacity;
				w->jobs = lpRealloc( w->jobs, sizeof( lpFractureJob ) * (size_t)capacity );
				for ( int k = w->jobCapacity; k < capacity; ++k )
				{
					w->jobs[k].bonds = lpAlloc( sizeof( lpCellBond ) * LP_MAX_CELL_BONDS );
				}
				w->jobCapacity = capacity;
			}
			lpPrepareFractureJob( w, w->jobs + w->jobCount, pieceIndex, local, impact );
			w->jobCount += 1;
		}
	}
	lpFree( candidates );

	uint64_t cellTicks = b3GetTicks();
	lpTaskPool_ParallelFor( w->tasks, w->jobCount, lpRunFractureJob, w->jobs );
	w->stats.cellMs += b3GetMilliseconds( cellTicks );

	for ( int i = 0; i < w->jobCount; ++i )
	{
		lpIntegrateFractureJob( w, w->jobs + i );
	}
	w->jobCount = 0;

	// Damage bonds near the impact (including the fresh ones)
	lpQueryPieces( w, lpInflatedBox( impact->point, impact->radius ) );
	w->stamp += 1;
	for ( int i = 0; i < w->scratchPieces.count; ++i )
	{
		lpPiece* p = w->pieces.data + w->scratchPieces.data[i];
		if ( p->body < 0 )
		{
			continue;
		}
		b3WorldTransform xf = b3Body_GetTransform( w->bodies.data[p->body].id );
		b3Vec3 local = b3InvTransformWorldPoint( xf, impact->point );

		for ( int k = 0; k < p->bonds.count; )
		{
			int bondIndex = p->bonds.data[k];
			lpBond* bond = w->bonds.data + bondIndex;
			if ( bond->stamp == w->stamp )
			{
				k += 1;
				continue;
			}
			bond->stamp = w->stamp;
			float density = lpImpactDensity( impact, b3Distance( bond->centroid, local ) );
			bond->health -= density;
			if ( bond->health <= 0.0f )
			{
				lpBreakBond( w, bondIndex ); // removes it from p->bonds, so k stays
				lpMarkDirty( w, p->body );
			}
			else
			{
				k += 1;
			}
		}
	}

	w->stats.impactsThisStep += 1;
	if ( w->def.debugLog )
	{
		printf( "[lpf] tick %llu impact at (%.2f %.2f %.2f) r %.2f E %.0f%s: %d candidates, fractures so far %d\n",
				(unsigned long long)w->tick, (double)impact->point.x, (double)impact->point.y, (double)impact->point.z,
				(double)impact->radius, (double)impact->energy, impact->explosion ? " (blast)" : "", candidateCount,
				w->stats.fracturesThisStep );
	}
}

// ---- connectivity ----


// Weight check for a static structure. A BFS from the anchored pieces gives every piece a bond distance to the
// ground. Loads then flow from the top down: each piece splits its own weight plus everything resting on it across
// all its bonds to pieces one step closer to the ground, in proportion to bond area, so parallel supports share the
// load like a real wall. A bond whose share exceeds area * health * stressScale breaks and its share moves to the
// piece's other supports. One pass per step; the caller re-checks the body next step if anything broke, so an
// undermined structure comes down progressively. Returns the number of broken bonds.
static int lpStressPass( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	int n = body->pieces.count;
	if ( w->def.stressScale <= 0.0f || n < 2 )
	{
		return 0;
	}

	// BFS from all anchored pieces (in body order)
	w->stamp += 1;
	int stamp = w->stamp;
	w->scratchQueue.count = 0;
	lpArray_Reserve( w->scratchQueue, n );
	lpArray_Reserve( w->scratchLoad, n );
	for ( int i = 0; i < n; ++i )
	{
		int pi = body->pieces.data[i];
		lpPiece* p = w->pieces.data + pi;
		p->supportBond = -1; // reused here as the BFS depth
		if ( p->anchored )
		{
			p->mark = stamp;
			p->supportBond = 0;
			w->scratchQueue.data[w->scratchQueue.count++] = pi;
		}
	}
	for ( int head = 0; head < w->scratchQueue.count; ++head )
	{
		int pi = w->scratchQueue.data[head];
		lpPiece* p = w->pieces.data + pi;
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			lpBond* bond = w->bonds.data + p->bonds.data[k];
			int other = bond->a == pi ? bond->b : bond->a;
			lpPiece* q = w->pieces.data + other;
			if ( q->mark != stamp )
			{
				q->mark = stamp;
				q->supportBond = p->supportBond + 1;
				w->scratchQueue.data[w->scratchQueue.count++] = other;
			}
		}
	}

	// Loads in newtons, flowing from the deepest pieces toward the ground (reverse BFS order)
	int reached = w->scratchQueue.count;
	float g = b3Length( b3World_GetGravity( w->def.physics ) );
	for ( int i = 0; i < reached; ++i )
	{
		lpPiece* p = w->pieces.data + w->scratchQueue.data[i];
		w->scratchLoad.data[i] = p->shape->volume * lpGetMaterial( p->material )->density * g;
		p->loadSlot = i;
	}

	int broken = 0;
	float maxUtilization = 0.0f;
	for ( int i = reached - 1; i >= 0; --i )
	{
		int pi = w->scratchQueue.data[i];
		lpPiece* p = w->pieces.data + pi;
		int depth = p->supportBond;
		if ( depth == 0 )
		{
			continue; // anchored: the ground takes it
		}
		float load = w->scratchLoad.data[i];

		// Repeatedly split over the surviving downward bonds until none is overloaded (or none is left)
		for ( int attempt = 0; attempt < 8; ++attempt )
		{
			float totalArea = 0.0f;
			for ( int k = 0; k < p->bonds.count; ++k )
			{
				lpBond* bond = w->bonds.data + p->bonds.data[k];
				int other = bond->a == pi ? bond->b : bond->a;
				if ( w->pieces.data[other].supportBond == depth - 1 )
				{
					totalArea += bond->area;
				}
			}
			if ( totalArea <= 0.0f )
			{
				break;
			}

			bool failed = false;
			for ( int k = 0; k < p->bonds.count; )
			{
				int bi = p->bonds.data[k];
				lpBond* bond = w->bonds.data + bi;
				int other = bond->a == pi ? bond->b : bond->a;
				if ( w->pieces.data[other].supportBond != depth - 1 )
				{
					k += 1;
					continue;
				}
				float share = load * bond->area / totalArea;
				float capacity = bond->area * bond->loadStrength * b3MaxFloat( bond->health, 0.0f ) / bond->strength * w->def.stressScale;
				float utilization = capacity > 0.0f ? share / capacity : 1e9f;
				maxUtilization = utilization > maxUtilization ? utilization : maxUtilization;
				if ( share > capacity )
				{
					lpBreakBond( w, bi ); // removes it from p->bonds
					broken += 1;
					failed = true;
				}
				else
				{
					k += 1;
				}
			}
			if ( failed )
			{
				continue; // redistribute over the survivors
			}

			for ( int k = 0; k < p->bonds.count; ++k )
			{
				lpBond* bond = w->bonds.data + p->bonds.data[k];
				int other = bond->a == pi ? bond->b : bond->a;
				lpPiece* q = w->pieces.data + other;
				if ( q->supportBond == depth - 1 )
				{
					w->scratchLoad.data[q->loadSlot] += load * bond->area / totalArea;
				}
			}
			break;
		}
	}
	if ( w->def.debugLog )
	{
		printf( "[lpf] tick %llu stress: body %d, %d pieces, %d reach the ground, %d bonds broke\n", (unsigned long long)w->tick,
				bodyIndex, n, reached, broken );
		printf( "[lpf]   peak bond utilization %.2f\n", (double)maxUtilization );
	}
	return broken;
}

static void lpUpdateBody( lpWorld* w, int bodyIndex )
{
	lpBody* body = w->bodies.data + bodyIndex;
	body->dirty = false;
	if ( body->alive == false )
	{
		return;
	}
	if ( body->pieces.count == 0 )
	{
		lpDestroyBody( w, bodyIndex, false );
		return;
	}

	if ( body->kind == lp_kindStructure && lpStressPass( w, bodyIndex ) > 0 )
	{
		lpArray_Push( w->stressAgain, bodyIndex );
	}

	// Flood fill over live bonds. Components are listed in the order of their first piece in the body list.
	w->stamp += 1;
	int stamp = w->stamp;
	w->scratchQueue.count = 0;
	lpArray_Reserve( w->scratchQueue, body->pieces.count );

	w->scratchComponents.count = 0;

	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int seed = body->pieces.data[i];
		if ( w->pieces.data[seed].mark == stamp )
		{
			continue;
		}
		lpComponent fresh = { w->scratchQueue.count, 0, 0.0f, b3Vec3_zero, false };
		lpArray_Push( w->scratchComponents, fresh );
		lpComponent* c = w->scratchComponents.data + w->scratchComponents.count - 1;

		w->pieces.data[seed].mark = stamp;
		w->scratchQueue.data[w->scratchQueue.count++] = seed;
		for ( int head = c->first; head < w->scratchQueue.count; ++head )
		{
			int pi = w->scratchQueue.data[head];
			lpPiece* p = w->pieces.data + pi;
			c->count += 1;
			c->volume += p->shape->volume;
			c->centroid = b3MulAdd( c->centroid, p->shape->volume, p->shape->centroid );
			c->anchored = c->anchored || p->anchored;
			for ( int k = 0; k < p->bonds.count; ++k )
			{
				lpBond* bond = w->bonds.data + p->bonds.data[k];
				int other = bond->a == pi ? bond->b : bond->a;
				if ( w->pieces.data[other].mark != stamp )
				{
					w->pieces.data[other].mark = stamp;
					w->scratchQueue.data[w->scratchQueue.count++] = other;
				}
			}
		}
		c->centroid = b3MulSV( 1.0f / c->volume, c->centroid );
	}

	lpComponent* components = w->scratchComponents.data;
	int componentCount = w->scratchComponents.count;

	if ( componentCount <= 1 && ( body->kind != lp_kindStructure || componentCount == 0 || components[0].anchored ) )
	{
		return;
	}

	// Which components stay on this body
	bool structure = body->kind == lp_kindStructure;
	int keep = -1;
	if ( structure == false )
	{
		keep = 0;
		for ( int c = 1; c < componentCount; ++c )
		{
			if ( components[c].volume > components[keep].volume )
			{
				keep = c;
			}
		}
	}

	b3WorldTransform xf = b3Body_GetTransform( body->id );
	b3Vec3 v = b3Body_GetLinearVelocity( body->id );
	b3Vec3 omega = b3Body_GetAngularVelocity( body->id );
	b3Vec3 localCenter = b3Body_GetLocalCenter( body->id );
	bool isDynamic = b3Body_GetType( body->id ) == b3_dynamicBody;

	int movedAny = 0;
	for ( int c = 0; c < componentCount; ++c )
	{
		lpComponent* comp = components + c;
		bool stays = structure ? comp->anchored : ( c == keep );
		if ( stays )
		{
			continue;
		}

		b3Vec3 compV = v;
		if ( isDynamic )
		{
			b3Vec3 r = b3RotateVector( xf.q, b3Sub( comp->centroid, localCenter ) );
			compV = b3Add( v, b3Cross( omega, r ) );
		}
		int newIndex = lpCreateBodyInternal( w, xf, b3_dynamicBody, lp_kindDebris, compV, isDynamic ? omega : b3Vec3_zero );
		body = w->bodies.data + bodyIndex; // array may have moved
		lpBody* nb = w->bodies.data + newIndex;

		for ( int k = 0; k < comp->count; ++k )
		{
			int pi = w->scratchQueue.data[comp->first + k];
			lpPiece* p = w->pieces.data + pi;
			lpDetachPieceShape( w, pi );
			p->anchored = false;
			p->body = -1;
			lpAttachPiece( w, pi, newIndex );
		}
		b3Body_ApplyMassFromShapes( nb->id );
		body->volume -= comp->volume;
		movedAny += 1;
		w->stats.splitsThisStep += 1;

		// Frozen rubble resting on what just left must be able to fall
		b3AABB box = { comp->centroid, comp->centroid };
		for ( int k = 0; k < comp->count; ++k )
		{
			lpPiece* p = w->pieces.data + w->scratchQueue.data[comp->first + k];
			box.lowerBound = b3Min( box.lowerBound, p->shape->bounds.lowerBound );
			box.upperBound = b3Max( box.upperBound, p->shape->bounds.upperBound );
		}
		b3Vec3 c0 = b3ToVec3( b3TransformWorldPoint( xf, b3AABB_Center( box ) ) );
		float r = b3Length( b3AABB_Extents( box ) ) + 0.3f;
		lpArray_Push( w->pendingWakes, ( (lpWake){ c0, r } ) );
	}

	if ( movedAny == 0 )
	{
		return;
	}

	// Rebuild this body's piece list from the pieces that stayed
	int kept = 0;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		if ( w->pieces.data[pi].body == bodyIndex )
		{
			body->pieces.data[kept++] = pi;
		}
	}
	body->pieces.count = kept;

	if ( kept == 0 )
	{
		lpDestroyBody( w, bodyIndex, false );
	}
	else if ( isDynamic )
	{
		b3Body_ApplyMassFromShapes( body->id );
	}
}

// ---- step ----

static void lpApplyWakes( lpWorld* w )
{
	for ( int i = 0; i < w->pendingWakes.count; ++i )
	{
		lpWake wake = w->pendingWakes.data[i];
		b3Vec3 r = { wake.radius, wake.radius, wake.radius };
		lpQueryPieces( w, (b3AABB){ b3Sub( wake.center, r ), b3Add( wake.center, r ) } );
		for ( int k = 0; k < w->scratchPieces.count; ++k )
		{
			lpPiece* p = w->pieces.data + w->scratchPieces.data[k];
			if ( p->body >= 0 )
			{
				lpWakeRubble( w, p->body );
			}
		}
	}
	w->pendingWakes.count = 0;
}

static void lpApplyForces( lpWorld* w )
{
	for ( int i = 0; i < w->forces.count; ++i )
	{
		lpForce force = w->forces.data[i];
		if ( force.impulse <= 0.0f )
		{
			continue;
		}

		// One impulse per loose body near the impact. Explosions push radially and set a speed (m/s, falling off with
		// distance) so light and heavy chunks fly alike; directional hits give an impulse (N*s) capped at 12 m/s.
		// The radial push is applied slightly off center toward the blast, which gives flying chunks some spin.
		lpQueryPieces( w, lpInflatedBox( force.point, force.radius ) );
		w->stamp += 1;
		for ( int k = 0; k < w->scratchPieces.count; ++k )
		{
			lpPiece* p = w->pieces.data + w->scratchPieces.data[k];
			if ( p->body < 0 )
			{
				continue;
			}
			lpBody* b = w->bodies.data + p->body;
			if ( b->kind != lp_kindDebris || b->stamp == w->stamp )
			{
				continue;
			}
			b->stamp = w->stamp;
			b3Pos center = b3Body_GetWorldCenter( b->id );
			float d = b3Length( b3SubPos( center, force.point ) );
			if ( d > force.radius )
			{
				continue;
			}
			float f = 1.0f - d / force.radius;
			float mass = b3Body_GetMass( b->id );
			if ( force.explosion )
			{
				b3Vec3 away = d > 1e-4f ? b3MulSV( 1.0f / d, b3SubPos( center, force.point ) ) : (b3Vec3){ 0.0f, 1.0f, 0.0f };
				// A blast on a surface breaches it: bias the push along the incoming direction, plus a little lift
				away = b3Add( away, force.direction );
				away = b3Normalize( b3Add( away, (b3Vec3){ 0.0f, 0.35f, 0.0f } ) );
				float speed = force.impulse * f;
				b3Vec3 extent = b3Body_GetMaxExtent( b->id );
				b3Pos at = b3OffsetPos( center, b3MulSV( -0.3f * b3Length( extent ), away ) );
				b3Body_ApplyLinearImpulse( b->id, b3MulSV( mass * speed, away ), at, true );
			}
			else
			{
				float impulse = b3MinFloat( force.impulse * f, 12.0f * mass );
				b3Body_ApplyLinearImpulse( b->id, b3MulSV( impulse, force.direction ), center, true );
			}
		}
	}
	w->forces.count = 0;
}

static int lpCompareHits( const void* a, const void* b )
{
	const lpHitCandidate* x = a;
	const lpHitCandidate* y = b;
	if ( x->energy != y->energy )
	{
		return x->energy > y->energy ? -1 : 1;
	}
	return ( x->key > y->key ) - ( x->key < y->key );
}

static float lpShapeMass( b3ShapeId shapeId )
{
	b3BodyId body = b3Shape_GetBody( shapeId );
	return b3Body_GetType( body ) == b3_dynamicBody ? b3Body_GetMass( body ) : 0.0f;
}

// Collisions hard enough to hurt become impacts for the next step
static void lpCollectHits( lpWorld* w )
{
	b3ContactEvents events = b3World_GetContactEvents( w->def.physics );
	w->scratchHits.count = 0;
	for ( int i = 0; i < events.hitCount; ++i )
	{
		const b3ContactHitEvent* e = events.hitEvents + i;
		if ( e->approachSpeed < w->def.hitSpeed )
		{
			continue;
		}
		intptr_t da = (intptr_t)b3Shape_GetUserData( e->shapeIdA );
		intptr_t db = (intptr_t)b3Shape_GetUserData( e->shapeIdB );
		if ( da <= 0 && db <= 0 )
		{
			continue;
		}

		// Volatile objects go off on a hard enough knock
		intptr_t pieceData[2] = { da, db };
		for ( int k = 0; k < 2; ++k )
		{
			if ( pieceData[k] > 0 )
			{
				int bodyIndex = w->pieces.data[pieceData[k] - 1].body;
				lpBody* b = w->bodies.data + bodyIndex;
				if ( b->armed && e->approachSpeed >= b->detonator.triggerSpeed )
				{
					lpDetonate( w, bodyIndex );
				}
			}
		}

		float ma = lpShapeMass( e->shapeIdA );
		float mb = lpShapeMass( e->shapeIdB );
		float mass = ( ma > 0.0f && mb > 0.0f ) ? ma * mb / ( ma + mb ) : b3MaxFloat( ma, mb );
		float energy = 0.5f * mass * e->approachSpeed * e->approachSpeed;
		if ( energy < 100.0f )
		{
			continue;
		}

		lpHitCandidate hit = { energy, e->point, ( (uint64_t)( da > 0 ? da : 0 ) << 32 ) | (uint64_t)( db > 0 ? db : 0 ) };
		lpArray_Push( w->scratchHits, hit );
	}

	if ( w->scratchHits.count > 1 )
	{
		qsort( w->scratchHits.data, (size_t)w->scratchHits.count, sizeof( lpHitCandidate ), lpCompareHits );
	}

	int count = w->scratchHits.count < w->def.maxHitImpacts ? w->scratchHits.count : w->def.maxHitImpacts;
	for ( int i = 0; i < count; ++i )
	{
		lpHitCandidate hit = w->scratchHits.data[i];
		lpImpactDef impact = { 0 };
		impact.point = hit.point;
		impact.radius = b3ClampFloat( 0.06f * cbrtf( hit.energy ), 0.15f, 1.2f );
		impact.energy = hit.energy;
		lpArray_Push( w->nextImpacts, impact );
	}
}

static void lpFreezeOrKill( lpWorld* w )
{
	b3BodyEvents events = b3World_GetBodyEvents( w->def.physics );
	w->scratchBodies.count = 0;
	for ( int i = 0; i < events.moveCount; ++i )
	{
		const b3BodyMoveEvent* e = events.moveEvents + i;
		intptr_t data = (intptr_t)e->userData;
		if ( data <= 0 )
		{
			continue;
		}
		int bodyIndex = (int)( data - 1 );
		lpBody* b = w->bodies.data + bodyIndex;
		if ( b->alive == false || b->kind != lp_kindDebris )
		{
			continue;
		}
		if ( (float)e->transform.p.y < w->def.killDepth )
		{
			lpArray_Push( w->scratchBodies, bodyIndex );
		}
		else if ( e->fellAsleep && w->def.freezeRubble && b->freezePending == false )
		{
			b->freezePending = true;
			lpArray_Push( w->freezeCandidates, bodyIndex );
		}
	}

	// Freeze sleepers that are old enough. Fresh debris wedged in its hole can fall asleep before anything pushed
	// it out; freezing it at once would glue it back into the wall.
	int kept = 0;
	for ( int i = 0; i < w->freezeCandidates.count; ++i )
	{
		int bodyIndex = w->freezeCandidates.data[i];
		lpBody* b = w->bodies.data + bodyIndex;
		if ( b->alive == false || b->kind != lp_kindDebris || b->freezePending == false )
		{
			continue;
		}
		if ( b3Body_IsAwake( b->id ) )
		{
			b->freezePending = false;
			continue;
		}
		if ( w->tick - b->createdTick >= 30 )
		{
			b->freezePending = false;
			b->kind = lp_kindRubble;
			b3Body_SetType( b->id, b3_staticBody );
			continue;
		}
		w->freezeCandidates.data[kept++] = bodyIndex;
	}
	w->freezeCandidates.count = kept;

	// Kill list in index order
	if ( w->scratchBodies.count > 1 )
	{
		qsort( w->scratchBodies.data, (size_t)w->scratchBodies.count, sizeof( int ), lpCompareInt );
	}
	for ( int i = 0; i < w->scratchBodies.count; ++i )
	{
		int bodyIndex = w->scratchBodies.data[i];
		if ( w->bodies.data[bodyIndex].alive )
		{
			lpDestroyBody( w, bodyIndex, false );
			w->stats.removedThisStep += 1;
		}
	}
}

typedef struct lpBudgetEntry
{
	float volume;
	uint64_t tick;
	int body;
} lpBudgetEntry;

static int lpCompareBudget( const void* a, const void* b )
{
	const lpBudgetEntry* x = a;
	const lpBudgetEntry* y = b;
	if ( x->volume != y->volume )
	{
		return x->volume < y->volume ? -1 : 1;
	}
	if ( x->tick != y->tick )
	{
		return x->tick < y->tick ? -1 : 1;
	}
	return ( x->body > y->body ) - ( x->body < y->body );
}

// Remove the smallest, oldest bodies of a kind until at most `limit` (bodies or pieces) remain
static void lpEnforceBudget( lpWorld* w, uint8_t kind, int limit, bool countPieces )
{
	int total = 0;
	int n = 0;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		if ( b->alive && b->kind == kind )
		{
			total += countPieces ? b->pieces.count : 1;
			n += 1;
		}
	}
	if ( total <= limit )
	{
		return;
	}

	lpBudgetEntry* entries = lpAlloc( sizeof( lpBudgetEntry ) * (size_t)n );
	int k = 0;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		if ( b->alive && b->kind == kind )
		{
			entries[k++] = (lpBudgetEntry){ b->volume, b->createdTick, i };
		}
	}
	qsort( entries, (size_t)n, sizeof( lpBudgetEntry ), lpCompareBudget );
	for ( int i = 0; i < n && total > limit; ++i )
	{
		lpBody* b = w->bodies.data + entries[i].body;
		total -= countPieces ? b->pieces.count : 1;
		lpDestroyBody( w, entries[i].body, true );
		w->stats.removedThisStep += 1;
	}
	lpFree( entries );
}

void lpWorld_Pull( lpWorld* w, int piece, b3Vec3 localPoint, b3Pos target, float maxAccel, float maxMass )
{
	lpPull pull = { piece, localPoint, target, maxAccel, maxMass };
	lpArray_Push( w->pulls, pull );
}

b3Vec3 lpWorld_ToBodyFrame( const lpWorld* w, int piece, b3Pos worldPoint )
{
	const lpPiece* p = w->pieces.data + piece;
	if ( p->body < 0 )
	{
		return b3Vec3_zero;
	}
	return b3InvTransformWorldPoint( b3Body_GetTransform( w->bodies.data[p->body].id ), worldPoint );
}

b3Pos lpWorld_ToWorldFrame( const lpWorld* w, int piece, b3Vec3 localPoint )
{
	const lpPiece* p = w->pieces.data + piece;
	if ( p->body < 0 )
	{
		return b3ToPos( localPoint );
	}
	return b3TransformWorldPoint( b3Body_GetTransform( w->bodies.data[p->body].id ), localPoint );
}

// Spring-damper toward the target, mass-normalized and clamped, with gravity compensation up to maxMass
static void lpApplyPulls( lpWorld* w )
{
	for ( int i = 0; i < w->pulls.count; ++i )
	{
		lpPull pull = w->pulls.data[i];
		if ( pull.piece < 0 || pull.piece >= w->pieces.count || w->pieces.data[pull.piece].body < 0 )
		{
			continue;
		}
		int bodyIndex = w->pieces.data[pull.piece].body;
		lpBody* b = w->bodies.data + bodyIndex;
		if ( b->kind == lp_kindStructure )
		{
			continue;
		}
		lpWakeRubble( w, bodyIndex );

		b3Pos point = b3TransformWorldPoint( b3Body_GetTransform( b->id ), pull.localPoint );
		b3Vec3 v = b3Body_GetWorldPointVelocity( b->id, point );
		b3Vec3 error = b3SubPos( pull.target, point );
		b3Vec3 accel = b3Sub( b3MulSV( 60.0f, error ), b3MulSV( 14.0f, v ) );
		float a = b3Length( accel );
		if ( a > pull.maxAccel )
		{
			accel = b3MulSV( pull.maxAccel / a, accel );
		}
		float mass = b3Body_GetMass( b->id );
		float m = b3MinFloat( mass, pull.maxMass );
		b3Vec3 g = b3World_GetGravity( w->def.physics );
		b3Vec3 force = b3Sub( b3MulSV( m, accel ), b3MulSV( m, g ) );
		b3Body_ApplyForce( b->id, force, point, true );
		// A little angular damping so held things do not spin forever
		b3Vec3 omega = b3Body_GetAngularVelocity( b->id );
		b3Body_SetAngularVelocity( b->id, b3MulSV( 0.97f, omega ) );
	}
	w->pulls.count = 0;
}

void lpWorld_Step( lpWorld* w, float timeStep, int subStepCount )
{
	w->stats.impactsThisStep = 0;
	w->stats.fracturesThisStep = 0;
	w->stats.cellsThisStep = 0;
	w->stats.splitsThisStep = 0;
	w->stats.removedThisStep = 0;
	w->stats.cellMs = 0.0f;
	w->stats.hullMs = 0.0f;
	w->stats.shapeMs = 0.0f;
	w->stats.bondMs = 0.0f;
	w->stats.splitMs = 0.0f;
	w->particles.count = 0;

	uint64_t ticks = b3GetTicks();

	// Bodies that detonated last step are consumed by their own blast
	if ( w->pendingDestroy.count > 1 )
	{
		qsort( w->pendingDestroy.data, (size_t)w->pendingDestroy.count, sizeof( int ), lpCompareInt );
	}
	for ( int i = 0; i < w->pendingDestroy.count; ++i )
	{
		int bodyIndex = w->pendingDestroy.data[i];
		if ( w->bodies.data[bodyIndex].alive && ( i == 0 || w->pendingDestroy.data[i - 1] != bodyIndex ) )
		{
			lpDestroyBody( w, bodyIndex, true );
		}
	}
	w->pendingDestroy.count = 0;

	// Impacts: tool impacts in call order, then last step's collision impacts
	for ( int i = 0; i < w->nextImpacts.count; ++i )
	{
		lpArray_Push( w->impacts, w->nextImpacts.data[i] );
	}
	w->nextImpacts.count = 0;

	for ( int i = 0; i < w->impacts.count; ++i )
	{
		lpImpactDef impact = w->impacts.data[i];
		lpProcessImpact( w, &impact );
		if ( impact.impulse > 0.0f )
		{
			lpForce force = { impact.point, impact.direction, impact.explosion ? 1.5f * impact.radius : impact.radius,
							  impact.impulse, impact.explosion };
			lpArray_Push( w->forces, force );
		}
	}
	w->impacts.count = 0;

	// Split what came apart. lpUpdateBody may create bodies but never marks new ones dirty.
	uint64_t splitTicks = b3GetTicks();
	for ( int i = 0; i < w->dirtyBodies.count; ++i )
	{
		lpUpdateBody( w, w->dirtyBodies.data[i] );
	}
	w->dirtyBodies.count = 0;
	for ( int i = 0; i < w->stressAgain.count; ++i )
	{
		lpMarkDirty( w, w->stressAgain.data[i] ); // keep collapsing next step
	}
	w->stressAgain.count = 0;
	w->stats.splitMs = b3GetMilliseconds( splitTicks );

	lpApplyWakes( w );
	lpApplyForces( w );
	lpApplyPulls( w );
	w->stats.fractureMs = b3GetMillisecondsAndReset( &ticks );

	b3World_Step( w->def.physics, timeStep, subStepCount );
	w->stats.physicsMs = b3GetMillisecondsAndReset( &ticks );

	lpCollectHits( w );
	lpFreezeOrKill( w );
	lpEnforceBudget( w, lp_kindDebris, w->def.maxDebrisBodies, false );
	lpEnforceBudget( w, lp_kindRubble, w->def.maxRubblePieces, true );

	// Counters
	w->stats.structureBodies = 0;
	w->stats.debrisBodies = 0;
	w->stats.awakeDebris = 0;
	w->stats.rubbleBodies = 0;
	int pieceCount = 0;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		if ( b->alive == false )
		{
			continue;
		}
		pieceCount += b->pieces.count;
		if ( b->kind == lp_kindStructure )
		{
			w->stats.structureBodies += 1;
		}
		else if ( b->kind == lp_kindRubble )
		{
			w->stats.rubbleBodies += 1;
		}
		else
		{
			w->stats.debrisBodies += 1;
			w->stats.awakeDebris += b3Body_IsAwake( b->id ) ? 1 : 0;
		}
	}
	w->stats.pieceCount = pieceCount;
	w->stats.bondCount = w->bondCount;
	w->stats.particlesThisStep = w->particles.count;
	w->stats.updateMs = b3GetMillisecondsAndReset( &ticks );

	w->tick += 1;
	w->stats.tick = w->tick;
}

lpStats lpWorld_GetStats( const lpWorld* w )
{
	return w->stats;
}

uint64_t lpWorld_Hash( const lpWorld* w )
{
	uint64_t h = LP_HASH_INIT;
	h = lpHashBytes( h, &w->tick, sizeof( w->tick ) );
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		if ( b->alive == false )
		{
			continue;
		}
		h = lpHashBytes( h, &i, sizeof( i ) );
		h = lpHashBytes( h, &b->kind, sizeof( b->kind ) );
		h = lpHashBytes( h, &b->pieces.count, sizeof( int ) );
		h = lpHashBytes( h, b->pieces.data, sizeof( int ) * (size_t)b->pieces.count );
		b3WorldTransform xf = b3Body_GetTransform( b->id );
		b3Vec3 v = b3Body_GetLinearVelocity( b->id );
		b3Vec3 omega = b3Body_GetAngularVelocity( b->id );
		h = lpHashBytes( h, &xf, sizeof( xf ) );
		h = lpHashBytes( h, &v, sizeof( v ) );
		h = lpHashBytes( h, &omega, sizeof( omega ) );
	}
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		if ( p->body < 0 )
		{
			continue;
		}
		h = lpHashBytes( h, &p->body, sizeof( p->body ) );
		h = lpHashBytes( h, &p->shape->volume, sizeof( float ) );
		h = lpHashBytes( h, &p->shape->centroid, sizeof( b3Vec3 ) );
		h = lpHashBytes( h, p->shape->vertices, sizeof( b3Vec3 ) * (size_t)p->shape->vertexCount );
		h = lpHashBytes( h, p->bonds.data, sizeof( int ) * (size_t)p->bonds.count );
	}
	for ( int i = 0; i < w->bonds.count; ++i )
	{
		const lpBond* bond = w->bonds.data + i;
		if ( bond->alive )
		{
			h = lpHashBytes( h, &bond->a, sizeof( int ) );
			h = lpHashBytes( h, &bond->b, sizeof( int ) );
			h = lpHashBytes( h, &bond->health, sizeof( float ) );
		}
	}
	return h;
}

// ---- queries ----

lpRayHit lpWorld_CastRay( const lpWorld* w, b3Pos origin, b3Vec3 translation )
{
	lpRayHit hit = { 0 };
	hit.piece = -1;
	hit.body = -1;
	b3RayResult result = b3World_CastRayClosest( w->def.physics, origin, translation, b3DefaultQueryFilter() );
	if ( result.hit == false )
	{
		return hit;
	}
	hit.hit = true;
	hit.point = result.point;
	hit.normal = result.normal;
	intptr_t data = (intptr_t)b3Shape_GetUserData( result.shapeId );
	if ( data > 0 )
	{
		hit.piece = (int)( data - 1 );
		hit.body = w->pieces.data[hit.piece].body;
	}
	return hit;
}

// ---- rendering access ----

int lpWorld_GetPieceCapacity( const lpWorld* w )
{
	return w->pieces.count;
}

lpPieceInfo lpWorld_GetPieceInfo( const lpWorld* w, int piece )
{
	const lpPiece* p = w->pieces.data + piece;
	lpPieceInfo info = { p->body, p->meshVersion, p->generation };
	return info;
}

int lpWorld_GetBodyCapacity( const lpWorld* w )
{
	return w->bodies.count;
}

bool lpWorld_GetBodyTransform( const lpWorld* w, int body, b3WorldTransform* transform )
{
	const lpBody* b = w->bodies.data + body;
	if ( b->alive == false )
	{
		return false;
	}
	*transform = b3Body_GetTransform( b->id );
	return true;
}

uint32_t lpWorld_GetBodyGeneration( const lpWorld* w, int body )
{
	return w->bodies.data[body].generation;
}

int lpWorld_BuildPieceMesh( const lpWorld* w, int piece, lpVertex* vertices, int capacity )
{
	const lpPiece* p = w->pieces.data + piece;
	if ( p->body < 0 )
	{
		return 0;
	}
	lpFacetParams params = { p->material, p->color, p->axis, p->seed };
	return lpBuildFacetMesh( p->shape, &params, vertices, capacity );
}

int lpWorld_GetMaxPieceVertices( void )
{
	return LP_MAX_PIECE_VERTICES;
}

const lpParticle* lpWorld_GetParticles( const lpWorld* w, int* count )
{
	*count = w->particles.count;
	return w->particles.data;
}

// ---- validation ----

static bool lpFail( const char* message, int a, int b, int c )
{
	fprintf( stderr, "lpWorld_Validate: " );
	fprintf( stderr, message, a, b, c );
	fprintf( stderr, "\n" );
	return false;
}

bool lpWorld_Validate( const lpWorld* w )
{
	int bondRefs = 0;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		if ( b->alive == false )
		{
			continue;
		}
		if ( b3Body_IsValid( b->id ) == false )
		{
			return lpFail( "body %d has an invalid Box3D id", i, 0, 0 );
		}
		if ( b3Body_GetShapeCount( b->id ) != b->pieces.count )
		{
			return lpFail( "body %d: %d shapes but %d pieces", i, b3Body_GetShapeCount( b->id ), b->pieces.count );
		}
		for ( int k = 0; k < b->pieces.count; ++k )
		{
			int pi = b->pieces.data[k];
			if ( pi < 0 || pi >= w->pieces.count || w->pieces.data[pi].body != i )
			{
				return lpFail( "body %d lists piece %d that is not on it (%d)", i, pi, pi >= 0 && pi < w->pieces.count ? w->pieces.data[pi].body : -2 );
			}
		}
	}

	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		if ( p->body < 0 )
		{
			continue;
		}
		if ( w->bodies.data[p->body].alive == false )
		{
			return lpFail( "piece %d on dead body %d", i, p->body, 0 );
		}
		if ( b3Shape_IsValid( p->shapeId ) == false )
		{
			return lpFail( "piece %d has no shape", i, 0, 0 );
		}
		if ( b3Shape_GetBody( p->shapeId ).index1 != w->bodies.data[p->body].id.index1 )
		{
			return lpFail( "piece %d shape on the wrong body", i, 0, 0 );
		}
		if ( p->shape == NULL || ( p->shape->volume > 0.0f ) == false )
		{
			return lpFail( "piece %d degenerate", i, 0, 0 );
		}
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			int bi = p->bonds.data[k];
			const lpBond* bond = w->bonds.data + bi;
			if ( bond->alive == false || ( bond->a != i && bond->b != i ) )
			{
				return lpFail( "piece %d lists bad bond %d", i, bi, 0 );
			}
			int other = bond->a == i ? bond->b : bond->a;
			if ( w->pieces.data[other].body != p->body )
			{
				return lpFail( "bond %d crosses bodies %d and %d", bi, p->body, w->pieces.data[other].body );
			}
			bondRefs += 1;
		}
	}

	int liveBonds = 0;
	for ( int i = 0; i < w->bonds.count; ++i )
	{
		liveBonds += w->bonds.data[i].alive ? 1 : 0;
	}
	if ( liveBonds != w->bondCount || bondRefs != 2 * liveBonds )
	{
		return lpFail( "bond count %d, live %d, refs %d", w->bondCount, liveBonds, bondRefs );
	}
	return true;
}
