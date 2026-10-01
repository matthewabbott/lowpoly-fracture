// SPDX-License-Identifier: MIT
// Debris tiers: how fragments get cheaper as they get smaller or more numerous.
//
// full   Box3D body, collides with everything; rests as fragile rubble (static, wakes when approached or hit)
// light  Box3D body, collides with static geometry only; rests as light rubble that movers shove aside one-way
// ghost  no Box3D body: flies ballistically through everything, lands by ray cast and becomes scrap
// scrap  no Box3D body: render-only; over budget it sinks into the ground and goes
//
// Tier changes keep the body index, so references and rendering stay valid, and piece geometry stays in object
// space (a ghost's frame is com - q * localCenter).

#include "world.h"

#include <float.h>
#include <math.h>

#define LP_GRID_CELL 2.0f
#define LP_GHOST_PLAN_TICKS 8

// ---- loose-debris grid ----

void lpGridInit( lpWorld* w )
{
	w->gridSlotCount = 4096;
	w->gridHeads = lpAlloc( sizeof( int ) * (size_t)w->gridSlotCount );
	for ( int i = 0; i < w->gridSlotCount; ++i )
	{
		w->gridHeads[i] = -1;
	}
}

void lpGridFree( lpWorld* w )
{
	lpFree( w->gridHeads );
	w->gridHeads = NULL;
}

static int lpGridCoord( float x )
{
	return lpFloatToInt( floorf( x * ( 1.0f / LP_GRID_CELL ) ) ); // a body flung far away stays in range
}

static int lpGridSlotOfCell( const lpWorld* w, int ix, int iz )
{
	uint32_t h = (uint32_t)ix * 73856093u ^ (uint32_t)iz * 19349663u;
	return (int)( h & (uint32_t)( w->gridSlotCount - 1 ) );
}

static void lpGridInsert( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	int slot = lpGridSlotOfCell( w, lpGridCoord( (float)b->com.x ), lpGridCoord( (float)b->com.z ) );
	b->gridSlot = slot;
	b->gridPrev = -1;
	b->gridNext = w->gridHeads[slot];
	if ( b->gridNext >= 0 )
	{
		w->bodies.data[b->gridNext].gridPrev = bodyIndex;
	}
	w->gridHeads[slot] = bodyIndex;
}

void lpGridRemove( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->gridSlot < 0 )
	{
		return;
	}
	if ( b->gridPrev >= 0 )
	{
		w->bodies.data[b->gridPrev].gridNext = b->gridNext;
	}
	else
	{
		w->gridHeads[b->gridSlot] = b->gridNext;
	}
	if ( b->gridNext >= 0 )
	{
		w->bodies.data[b->gridNext].gridPrev = b->gridPrev;
	}
	b->gridSlot = -1;
	b->gridPrev = -1;
	b->gridNext = -1;
}

static void lpGridUpdate( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	int slot = lpGridSlotOfCell( w, lpGridCoord( (float)b->com.x ), lpGridCoord( (float)b->com.z ) );
	if ( slot != b->gridSlot )
	{
		lpGridRemove( w, bodyIndex );
		lpGridInsert( w, bodyIndex );
	}
}

void lpQueryLoose( lpWorld* w, lpAABB box )
{
	w->scratchLoose.count = 0;
	int x0 = lpGridCoord( box.lowerBound.x ), x1 = lpGridCoord( box.upperBound.x );
	int z0 = lpGridCoord( box.lowerBound.z ), z1 = lpGridCoord( box.upperBound.z );
	w->stamp += 1;
	int stamp = w->stamp;

	bool everySlot = (int64_t)( x1 - x0 + 1 ) * (int64_t)( z1 - z0 + 1 ) > (int64_t)w->gridSlotCount;
	if ( everySlot )
	{
		// Bigger than the table: walk every slot once instead
		x0 = 0;
		x1 = 0;
		z0 = 0;
		z1 = w->gridSlotCount - 1;
	}

	for ( int ix = x0; ix <= x1; ++ix )
	{
		for ( int iz = z0; iz <= z1; ++iz )
		{
			int slot = everySlot ? iz : lpGridSlotOfCell( w, ix, iz );
			for ( int i = w->gridHeads[slot]; i >= 0; i = w->bodies.data[i].gridNext )
			{
				lpBody* b = w->bodies.data + i;
				if ( b->stamp == stamp )
				{
					continue;
				}
				b->stamp = stamp;
				lpVec3 c = lpToVec3( b->com );
				if ( c.x < box.lowerBound.x || c.x > box.upperBound.x || c.y < box.lowerBound.y || c.y > box.upperBound.y ||
					 c.z < box.lowerBound.z || c.z > box.upperBound.z )
				{
					continue;
				}
				lpArray_Push( w->scratchLoose, i );
			}
		}
	}
	if ( w->scratchLoose.count > 1 )
	{
		qsort( w->scratchLoose.data, (size_t)w->scratchLoose.count, sizeof( int ), lpCompareInt );
	}
}

// ---- loose bodies ----

static lpVec3 lpVolumeCenter( const lpWorld* w, const lpBody* b )
{
	lpVec3 c = lpVec3_zero;
	float v = 0.0f;
	for ( int i = 0; i < b->pieces.count; ++i )
	{
		const lpShape* s = w->pieces.data[b->pieces.data[i]].shape;
		c = lpMulAdd( c, s->volume, s->centroid );
		v += s->volume;
	}
	return v > 0.0f ? lpMulSV( 1.0f / v, c ) : c;
}

int lpBeginGhost( lpWorld* w, lpWorldTransform xf, lpVec3 v, lpVec3 omega, float gravityScale )
{
	int index = lpAllocBody( w );
	lpBody* b = w->bodies.data + index;
	b->id = b3_nullBodyId;
	b->gravityScale = gravityScale;
	b->kind = lp_kindGhost;
	b->tier = lp_tierLight;
	b->com = xf.p;
	b->q = xf.q;
	b->v = v;
	b->omega = omega;
	b->localCenter = lpVec3_zero;
	b->planTicks = 0;
	b->landIn = -1;
	b->sinkTicks = 0;
	return index;
}

void lpAddLoosePiece( lpWorld* w, int bodyIndex, int pieceIndex )
{
	lpBreakPieceLinks( w, pieceIndex ); // a loose piece has no Box3D body to hold a joint
	lpPiece* p = w->pieces.data + pieceIndex;
	lpBody* b = w->bodies.data + bodyIndex;
	p->body = bodyIndex;
	p->shapeId = b3_nullShapeId;
	p->anchored = false;
	lpArray_Push( b->pieces, pieceIndex );
	b->volume += p->shape->volume;
}

void lpFinishLoose( lpWorld* w, int bodyIndex, lpWorldTransform xf )
{
	lpBody* b = w->bodies.data + bodyIndex;
	b->localCenter = lpVolumeCenter( w, b );
	b->q = xf.q;
	b->com = lpTransformWorldPoint( xf, b->localCenter );
	lpGridInsert( w, bodyIndex );
}

// Capture a Box3D body's motion, destroy the Box3D body, and keep the pieces as a loose body.
static void lpMakeLoose( lpWorld* w, int bodyIndex, uint8_t kind )
{
	lpBody* b = w->bodies.data + bodyIndex;
	lpWorldTransform xf = b3Body_GetTransform( b->id );
	lpVec3 lc = lpVolumeCenter( w, b );
	lpPos com = lpTransformWorldPoint( xf, lc );
	lpVec3 v = lpVec3_zero;
	lpVec3 omega = lpVec3_zero;
	if ( kind == lp_kindGhost && b3Body_GetType( b->id ) == b3_dynamicBody )
	{
		v = b3Body_GetWorldPointVelocity( b->id, com );
		omega = b3Body_GetAngularVelocity( b->id );
	}
	for ( int i = 0; i < b->pieces.count; ++i )
	{
		lpBreakPieceLinks( w, b->pieces.data[i] );
		w->pieces.data[b->pieces.data[i]].shapeId = b3_nullShapeId; // destroyed with the body
	}
	b3DestroyBody( b->id );
	b->id = b3_nullBodyId;
	b->kind = kind;
	b->tier = lp_tierLight;
	b->freezePending = false;
	b->q = xf.q;
	b->com = com;
	b->v = v;
	b->omega = omega;
	b->localCenter = lc;
	b->planTicks = 0;
	b->landIn = -1;
	b->sinkTicks = 0;
	lpGridInsert( w, bodyIndex );
}

void lpConvertToGhost( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->kind == lp_kindGhost || b->kind == lp_kindStructure )
	{
		return;
	}
	if ( b->kind == lp_kindScrap )
	{
		b->kind = lp_kindGhost;
		b->planTicks = 0;
		b->landIn = -1;
		b->sinkTicks = 0;
		return;
	}
	lpMakeLoose( w, bodyIndex, lp_kindGhost );
}

void lpConvertToScrap( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->kind == lp_kindScrap || b->kind == lp_kindStructure )
	{
		return;
	}
	if ( b->kind == lp_kindGhost )
	{
		b->kind = lp_kindScrap;
		b->v = lpVec3_zero;
		b->omega = lpVec3_zero;
		b->landIn = -1;
		return;
	}
	lpMakeLoose( w, bodyIndex, lp_kindScrap );
}

static void lpRecreateShapes( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	for ( int i = 0; i < b->pieces.count; ++i )
	{
		int pi = b->pieces.data[i];
		lpDetachPieceShape( w, pi );
		lpCreatePieceShape( w, pi, bodyIndex );
	}
}

void lpConvertToLight( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( ( b->kind != lp_kindDebris && b->kind != lp_kindRubble ) || b->tier == lp_tierLight )
	{
		return;
	}
	b->tier = lp_tierLight;
	lpRecreateShapes( w, bodyIndex );
	b3Body_SetSleepThreshold( b->id, 0.3f );
	if ( b->kind == lp_kindDebris )
	{
		lpApplyMass( b );
	}
}

void lpConvertToFull( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->kind == lp_kindStructure )
	{
		return;
	}

	if ( b->kind == lp_kindGhost || b->kind == lp_kindScrap )
	{
		lpWorldTransform xf = lpGetTransform( b );
		b3BodyDef def = b3DefaultBodyDef();
		def.type = b3_dynamicBody;
		def.position = xf.p;
		def.rotation = xf.q;
		def.linearVelocity = b->v;
		def.angularVelocity = b->omega;
		def.gravityScale = b->gravityScale;
		def.userData = (void*)(intptr_t)( bodyIndex + 1 );
		lpGridRemove( w, bodyIndex );
		b->id = b3CreateBody( w->physics, &def );
		b->kind = lp_kindDebris;
		b->tier = lp_tierFull;
		b->createdTick = w->tick;
		for ( int i = 0; i < b->pieces.count; ++i )
		{
			lpCreatePieceShape( w, b->pieces.data[i], bodyIndex );
		}
		lpApplyMass( b );
		return;
	}

	if ( b->tier == lp_tierLight )
	{
		b->tier = lp_tierFull;
		lpRecreateShapes( w, bodyIndex );
		b3Body_SetSleepThreshold( b->id, 0.05f );
	}
	lpWakeRubble( w, bodyIndex );
	if ( b->kind == lp_kindDebris )
	{
		lpApplyMass( b );
	}
}

// ---- ghosts ----

// Same as Box3D's internal b3IntegrateRotation: q2 = normalize(q1 + 0.5 * omega * q1)
static lpQuat lpIntegrateRotation( lpQuat q1, lpVec3 deltaRotation )
{
	lpQuat qd = { lpMulSV( 0.5f, deltaRotation ), 0.0f };
	qd = lpMulQuat( qd, q1 );
	lpQuat q2 = { lpAdd( q1.v, qd.v ), qd.s + q1.s };
	return lpNormalizeQuat( q2 );
}

typedef struct lpStaticRay
{
	const lpWorld* world;
	float fraction;
	lpPos point;
	lpVec3 normal;
	bool hit;
	int piece;
} lpStaticRay;

// Ghosts land on static things only (structures, rubble, the ground); they fly through moving bodies
static float lpStaticRayFcn( b3ShapeId shapeId, lpPos point, lpVec3 normal, float fraction, uint64_t userMaterialId,
							 int triangleIndex, int childIndex, void* context )
{
	(void)userMaterialId;
	(void)triangleIndex;
	(void)childIndex;
	lpStaticRay* ray = context;
	intptr_t data = (intptr_t)b3Shape_GetUserData( shapeId );
	if ( data <= 0 )
	{
		return -1.0f;
	}
	const lpBody* b = ray->world->bodies.data + ray->world->pieces.data[data - 1].body;
	if ( b->kind != lp_kindStructure && b->kind != lp_kindRubble )
	{
		return -1.0f;
	}
	int piece = (int)( data - 1 );
	if ( fraction < ray->fraction || ( ray->hit && fraction == ray->fraction && piece < ray->piece ) )
	{
		ray->fraction = fraction;
		ray->point = point;
		// A ray that starts inside a shape reports it with no normal: the ghost is already in, so it lands upward
		ray->normal = lpLengthSquared( normal ) > 0.5f ? normal : (lpVec3){ 0.0f, 1.0f, 0.0f };
		ray->hit = true;
		ray->piece = piece;
	}
	return lpNextUp( ray->fraction );
}

// Settle on the landing surface: the lowest vertex along the normal touches it. Steep surfaces (walls) deflect the
// ghost downward instead, so it does not stick to them.
static void lpLand( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	lpVec3 n = b->landNormal;
	if ( n.y < 0.5f )
	{
		float vn = lpDot( b->v, n );
		b->v = lpMulSV( 0.25f, lpMulAdd( b->v, -2.0f * vn, n ) );
		b->com = lpOffsetPos( b->landPoint, lpMulSV( 0.05f, n ) );
		b->planTicks = 0;
		b->landIn = -1;
		lpGridUpdate( w, bodyIndex );
		return;
	}

	float support = 0.0f;
	for ( int i = 0; i < b->pieces.count; ++i )
	{
		const lpShape* s = w->pieces.data[b->pieces.data[i]].shape;
		for ( int k = 0; k < s->vertexCount; ++k )
		{
			lpVec3 r = lpRotateVector( b->q, lpSub( s->vertices[k], b->localCenter ) );
			float d = -lpDot( r, n );
			support = d > support ? d : support;
		}
	}
	b->com = lpOffsetPos( b->landPoint, lpMulSV( support, n ) );
	b->v = lpVec3_zero;
	b->omega = lpVec3_zero;
	b->kind = lp_kindScrap;
	b->planTicks = 0;
	b->landIn = -1;
	lpGridUpdate( w, bodyIndex );
}

void lpStepGhosts( lpWorld* w, float timeStep )
{
	lpVec3 g = b3World_GetGravity( w->physics );
	int casts = 0;
	float plan = (float)LP_GHOST_PLAN_TICKS * timeStep;

	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		if ( b->alive == false )
		{
			continue;
		}

		if ( b->kind == lp_kindScrap )
		{
			if ( b->sinkTicks > 0 )
			{
				b->com.y -= 0.02f;
				b->sinkTicks -= 1;
				if ( b->sinkTicks == 0 )
				{
					lpDestroyBody( w, i, false );
				}
			}
			continue;
		}
		if ( b->kind != lp_kindGhost )
		{
			continue;
		}
		lpVec3 gb = lpMulSV( b->gravityScale, g );

		// Flight plan: cast along the chord of the next few ticks of the ballistic arc. The arc bows off the chord by
		// g T^2 / 8 (about 2 cm for 8 ticks), which is invisible. A hit gives the landing tick. The chord ends where
		// the stepping below really ends (velocity first, then position), a centimetre under the exact parabola, or
		// a ghost skimming the ground would end up inside it.
		if ( b->planTicks <= 0 && casts < w->def.maxGhostCastsPerStep )
		{
			casts += 1;
			float n = (float)LP_GHOST_PLAN_TICKS;
			lpVec3 chord = lpAdd( lpMulSV( plan, b->v ), lpMulSV( 0.5f * timeStep * timeStep * n * ( n + 1.0f ), gb ) );
			lpStaticRay ray = { w, 2.0f, { 0 }, lpVec3_zero, false, -1 };
			b3World_CastRay( w->physics, b->com, chord, b3DefaultQueryFilter(), lpStaticRayFcn, &ray );
			b->planTicks = LP_GHOST_PLAN_TICKS;
			b->landIn = -1;
			if ( ray.hit )
			{
				int ticks = (int)ceilf( ray.fraction * (float)LP_GHOST_PLAN_TICKS );
				b->landIn = ticks < 1 ? 1 : ticks;
				b->landPoint = ray.point;
				b->landNormal = ray.normal;
			}
		}

		b->v = lpMulAdd( b->v, timeStep, gb );
		b->com = lpOffsetPos( b->com, lpMulSV( timeStep, b->v ) );
		b->q = lpIntegrateRotation( b->q, lpMulSV( timeStep, b->omega ) );
		b->planTicks -= 1;

		if ( b->landIn > 0 )
		{
			b->landIn -= 1;
			if ( b->landIn == 0 )
			{
				lpLand( w, i );
				continue;
			}
		}
		if ( (float)b->com.y < w->def.killDepth )
		{
			lpDestroyBody( w, i, false );
			continue;
		}
		lpGridUpdate( w, i );
	}
	w->stats.ghostCasts = casts;
}

// ---- pushes ----

static float lpLooseMass( const lpWorld* w, const lpBody* b )
{
	float mass = 0.0f;
	for ( int i = 0; i < b->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + b->pieces.data[i];
		mass += p->shape->volume * lpGetMaterial( p->material )->density;
	}
	return mass > 0.01f ? mass : 0.01f;
}

static void lpLoosen( lpBody* b )
{
	if ( b->kind == lp_kindScrap )
	{
		b->kind = lp_kindGhost;
		b->sinkTicks = 0;
	}
	b->planTicks = 0;
	b->landIn = -1;
}

void lpApplyLooseForce( lpWorld* w, const lpForce* force )
{
	lpVec3 c = lpToVec3( force->point );
	lpVec3 r = { force->radius, force->radius, force->radius };
	lpQueryLoose( w, (lpAABB){ lpSub( c, r ), lpAdd( c, r ) } );
	for ( int k = 0; k < w->scratchLoose.count; ++k )
	{
		lpBody* b = w->bodies.data + w->scratchLoose.data[k];
		lpVec3 rel = lpSubPos( b->com, force->point );
		float d = lpLength( rel );
		if ( d > force->radius )
		{
			continue;
		}
		float f = 1.0f - d / force->radius;
		lpLoosen( b );
		if ( force->explosion )
		{
			lpVec3 away = d > 1e-4f ? lpMulSV( 1.0f / d, rel ) : (lpVec3){ 0.0f, 1.0f, 0.0f };
			away = lpNormalize( lpAdd( lpAdd( away, force->direction ), (lpVec3){ 0.0f, 0.35f, 0.0f } ) );
			b->v = lpMulAdd( b->v, force->impulse * f, away );
		}
		else
		{
			float dv = lpMinFloat( force->impulse * f / lpLooseMass( w, b ), 12.0f );
			b->v = lpMulAdd( b->v, dv, force->direction );
		}
	}
}

// Heavy, fast bodies plough through small rubble: light rubble and scrap in their path are flung aside without
// pushing back (one-way "styrofoam"), and fragile full rubble is woken so real physics handles the collision.
void lpShove( lpWorld* w, float timeStep )
{
	int movers = 0;
	for ( int i = 0; i < w->bodies.count && movers < 32; ++i )
	{
		lpBody* mover = w->bodies.data + i;
		if ( mover->alive == false || mover->kind != lp_kindDebris || mover->tier != lp_tierFull )
		{
			continue;
		}
		lpVec3 v = b3Body_GetLinearVelocity( mover->id );
		float speed = lpLength( v );
		if ( speed < 2.0f || b3Body_GetMass( mover->id ) < 60.0f )
		{
			continue;
		}
		movers += 1;

		lpAABB box = b3Body_ComputeAABB( mover->id );
		lpVec3 ahead = lpMulSV( 2.0f * timeStep, v );
		box.lowerBound = lpMin( box.lowerBound, lpAdd( box.lowerBound, ahead ) );
		box.upperBound = lpMax( box.upperBound, lpAdd( box.upperBound, ahead ) );
		lpVec3 moverCenter = lpToVec3( b3Body_GetWorldCenter( mover->id ) );

		lpQueryPieces( w, box );
		w->stamp += 1;
		int stamp = w->stamp;
		for ( int k = 0; k < w->scratchPieces.count; ++k )
		{
			int bi = w->pieces.data[w->scratchPieces.data[k]].body;
			lpBody* b = w->bodies.data + bi;
			if ( b->kind != lp_kindRubble || b->stamp == stamp )
			{
				continue;
			}
			b->stamp = stamp;
			lpWakeRubble( w, bi );
			if ( b->tier == lp_tierLight )
			{
				lpPos c = b3Body_GetWorldCenter( b->id );
				lpVec3 out = lpSub( lpToVec3( c ), moverCenter );
				out.y = 0.0f;
				out = lpNormalize( out );
				lpVec3 push = lpAdd( lpMulSV( 1.1f, b3Body_GetWorldPointVelocity( mover->id, c ) ), lpMulSV( 1.5f, out ) );
				push.y += 1.0f;
				b3Body_SetLinearVelocity( b->id, push );
			}
		}

		lpQueryLoose( w, box );
		for ( int k = 0; k < w->scratchLoose.count; ++k )
		{
			lpBody* b = w->bodies.data + w->scratchLoose.data[k];
			lpVec3 out = lpSub( lpToVec3( b->com ), moverCenter );
			out.y = 0.0f;
			out = lpNormalize( out );
			lpLoosen( b );
			b->v = lpAdd( lpMulSV( 1.1f, b3Body_GetWorldPointVelocity( mover->id, b->com ) ), lpMulSV( 1.5f, out ) );
			b->v.y += 1.0f;
		}
	}
}

void lpWorld_Blow( lpWorld* w, lpPos origin, lpVec3 direction, float range, float halfAngleRadians, float speed )
{
	lpBlow blow = { origin, lpNormalize( direction ), range, lpComputeCosSin( halfAngleRadians ).cosine, speed };
	lpArray_Push( w->blows, blow );
}

void lpApplyBlows( lpWorld* w )
{
	for ( int i = 0; i < w->blows.count; ++i )
	{
		lpBlow blow = w->blows.data[i];
		lpVec3 o = lpToVec3( blow.origin );
		lpVec3 e = lpMulAdd( o, blow.range, blow.direction );
		float spread = blow.range * sqrtf( lpMaxFloat( 1.0f - blow.cosAngle * blow.cosAngle, 0.0f ) ) + 0.5f;
		lpVec3 pad = { spread, spread, spread };
		lpAABB box = { lpSub( lpMin( o, e ), pad ), lpAdd( lpMax( o, e ), pad ) };

		lpQueryPieces( w, box );
		w->stamp += 1;
		int stamp = w->stamp;
		for ( int k = 0; k < w->scratchPieces.count; ++k )
		{
			int bi = w->pieces.data[w->scratchPieces.data[k]].body;
			lpBody* b = w->bodies.data + bi;
			if ( b->kind == lp_kindStructure || b->stamp == stamp )
			{
				continue;
			}
			b->stamp = stamp;
			lpPos c = b3Body_GetWorldCenter( b->id );
			lpVec3 rel = lpSubPos( c, blow.origin );
			float d = lpLength( rel );
			if ( d > blow.range || d < 1e-3f || lpDot( rel, blow.direction ) < blow.cosAngle * d )
			{
				continue;
			}
			float f = 1.0f - d / blow.range;
			lpWakeRubble( w, bi );
			float mass = b3Body_GetMass( b->id );
			float dv = blow.speed * f * lpClampFloat( 25.0f / lpMaxFloat( mass, 0.01f ), 0.05f, 1.0f );
			lpVec3 push = lpMulAdd( lpMulSV( dv, blow.direction ), 0.3f * dv, (lpVec3){ 0.0f, 1.0f, 0.0f } );
			b3Body_ApplyLinearImpulse( b->id, lpMulSV( mass, push ), c, true );
		}

		lpQueryLoose( w, box );
		for ( int k = 0; k < w->scratchLoose.count; ++k )
		{
			lpBody* b = w->bodies.data + w->scratchLoose.data[k];
			lpVec3 rel = lpSubPos( b->com, blow.origin );
			float d = lpLength( rel );
			if ( d > blow.range || d < 1e-3f || lpDot( rel, blow.direction ) < blow.cosAngle * d )
			{
				continue;
			}
			float f = 1.0f - d / blow.range;
			lpLoosen( b );
			b->v = lpMulAdd( lpMulAdd( b->v, blow.speed * f, blow.direction ), 0.3f * blow.speed * f, (lpVec3){ 0.0f, 1.0f, 0.0f } );
		}
	}
	w->blows.count = 0;
}

void lpWorld_PromoteBody( lpWorld* w, int body )
{
	if ( body >= 0 && body < w->bodies.count && w->bodies.data[body].alive )
	{
		lpConvertToFull( w, body );
	}
}

void lpWorld_SetGravityScale( lpWorld* w, int body, float scale )
{
	if ( body < 0 || body >= w->bodies.count || w->bodies.data[body].alive == false )
	{
		return;
	}
	lpBody* b = w->bodies.data + body;
	b->gravityScale = scale;
	if ( B3_IS_NON_NULL( b->id ) )
	{
		b3Body_SetGravityScale( b->id, scale );
		if ( b3Body_GetType( b->id ) == b3_dynamicBody )
		{
			b3Body_SetAwake( b->id, true );
		}
	}
	if ( b->kind == lp_kindStructure )
	{
		b->topology += 1; // its weight changed: solve it again
		lpMarkDirty( w, body );
	}
}

// ---- budgets ----

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

typedef bool lpBodyPredicate( const lpBody* b );

static bool lpIsFullDebris( const lpBody* b )
{
	return b->kind == lp_kindDebris && b->tier == lp_tierFull;
}

static bool lpIsLightDebris( const lpBody* b )
{
	return b->kind == lp_kindDebris && b->tier == lp_tierLight;
}

static bool lpIsGhost( const lpBody* b )
{
	return b->kind == lp_kindGhost;
}

static bool lpIsRubble( const lpBody* b )
{
	return b->kind == lp_kindRubble;
}

static bool lpIsStandingScrap( const lpBody* b )
{
	return b->kind == lp_kindScrap && b->sinkTicks == 0;
}

// Bodies matching the predicate, smallest and oldest first
static lpBudgetEntry* lpRankBodies( lpWorld* w, lpBodyPredicate* predicate, int* count )
{
	int n = 0;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		n += b->alive && lpBodyLinked( w, b ) == false && predicate( b ) ? 1 : 0;
	}
	lpBudgetEntry* entries = lpAlloc( sizeof( lpBudgetEntry ) * (size_t)( n > 0 ? n : 1 ) );
	int k = 0;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		if ( b->alive && lpBodyLinked( w, b ) == false && predicate( b ) )
		{
			entries[k++] = (lpBudgetEntry){ b->volume, b->createdTick, i };
		}
	}
	qsort( entries, (size_t)n, sizeof( lpBudgetEntry ), lpCompareBudget );
	*count = n;
	return entries;
}

// Over budget, bodies move down a tier instead of popping: full -> light -> ghost -> particles, rubble -> scrap,
// scrap sinks away. Per-step caps spread the cost of bursts.
void lpEnforceBudgets( lpWorld* w )
{
	int full = 0, light = 0, ghosts = 0, rubblePieces = 0, scrapPieces = 0;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		if ( b->alive == false || lpBodyLinked( w, b ) )
		{
			continue; // linked bodies are in use (a cart, a hanging sign): outside every budget
		}
		if ( b->kind == lp_kindDebris )
		{
			if ( b->tier == lp_tierFull )
			{
				full += 1;
				continue;
			}
			// Light debris still moving long after it was made (rolling, jittering) is frozen once slow
			if ( w->def.freezeRubble && w->tick - b->createdTick >= 240 && w->freezesThisStep < w->def.maxFreezesPerStep &&
				 lpLength( b3Body_GetLinearVelocity( b->id ) ) < 1.0f && lpBodyLinked( w, b ) == false &&
				 lpTouchesLinked( w, b ) == false )
			{
				b->kind = lp_kindRubble;
				b->freezePending = false;
				b3Body_SetType( b->id, b3_staticBody );
				w->freezesThisStep += 1;
				rubblePieces += b->pieces.count;
				continue;
			}
			light += 1;
		}
		else if ( b->kind == lp_kindGhost )
		{
			ghosts += 1;
		}
		else if ( b->kind == lp_kindRubble )
		{
			rubblePieces += b->pieces.count;
		}
		else if ( b->kind == lp_kindScrap && b->sinkTicks == 0 )
		{
			scrapPieces += b->pieces.count;
		}
	}

	int n;
	if ( full > w->def.maxFullDebris )
	{
		lpBudgetEntry* e = lpRankBodies( w, lpIsFullDebris, &n );
		int excess = full - w->def.maxFullDebris;
		for ( int i = 0; i < n && i < excess && i < 32; ++i )
		{
			lpConvertToLight( w, e[i].body );
			w->stats.demotionsThisStep += 1;
			light += 1;
		}
		lpFree( e );
	}
	if ( light > w->def.maxLightDebris )
	{
		lpBudgetEntry* e = lpRankBodies( w, lpIsLightDebris, &n );
		int excess = light - w->def.maxLightDebris;
		for ( int i = 0; i < n && i < excess && i < 128; ++i )
		{
			lpConvertToGhost( w, e[i].body );
			w->stats.demotionsThisStep += 1;
			ghosts += 1;
		}
		lpFree( e );
	}
	if ( ghosts > w->def.maxGhosts )
	{
		lpBudgetEntry* e = lpRankBodies( w, lpIsGhost, &n );
		int excess = ghosts - w->def.maxGhosts;
		for ( int i = 0; i < n && i < excess; ++i )
		{
			lpDestroyBody( w, e[i].body, true );
			w->stats.demotionsThisStep += 1;
		}
		lpFree( e );
	}
	if ( rubblePieces > w->def.maxRubblePieces )
	{
		lpBudgetEntry* e = lpRankBodies( w, lpIsRubble, &n );
		for ( int i = 0, done = 0; i < n && rubblePieces > w->def.maxRubblePieces && done < 64; ++i, ++done )
		{
			int pieces = w->bodies.data[e[i].body].pieces.count;
			rubblePieces -= pieces;
			scrapPieces += pieces; // counted now, so the scrap cap below holds in this same step
			lpConvertToScrap( w, e[i].body );
			w->stats.demotionsThisStep += 1;
		}
		lpFree( e );
	}
	if ( scrapPieces > w->def.maxScrapPieces )
	{
		lpBudgetEntry* e = lpRankBodies( w, lpIsStandingScrap, &n );
		for ( int i = 0; i < n && scrapPieces > w->def.maxScrapPieces; ++i )
		{
			lpBody* b = w->bodies.data + e[i].body;
			scrapPieces -= b->pieces.count;
			b->sinkTicks = 20;
		}
		lpFree( e );
	}
}

// ---- collision filter ----

// Runs on Box3D worker threads when a pair is created; reads only our own arrays, which do not change during a step.
// A light piece touches only static things (structures, rubble); a full piece touches light pieces only while the
// full piece is frozen rubble. Everything else follows the category masks.
bool lpPairFilter( int pieceA, int pieceB, void* context )
{
	const lpWorld* w = context;
	if ( pieceA < 0 || pieceB < 0 )
	{
		return true;
	}
	const lpBody* ba = w->bodies.data + w->pieces.data[pieceA].body;
	const lpBody* bb = w->bodies.data + w->pieces.data[pieceB].body;
	bool lightA = ba->kind != lp_kindStructure && ba->tier == lp_tierLight;
	bool lightB = bb->kind != lp_kindStructure && bb->tier == lp_tierLight;
	if ( lightA == lightB )
	{
		return true;
	}
	const lpBody* light = lightA ? ba : bb;
	const lpBody* other = lightA ? bb : ba;
	bool otherStatic = other->kind == lp_kindStructure || other->kind == lp_kindRubble;
	return otherStatic && light->kind == lp_kindDebris;
}
