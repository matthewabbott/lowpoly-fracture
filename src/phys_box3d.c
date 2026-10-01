// SPDX-License-Identifier: MIT
// The physics interface (phys.h) on Box3D: the only file that includes Box3D. Box3D's ids pack into the handles
// (b3Store*Id), user data is index + 1 (0 for none), and reports are re-ordered here into our own total order.

#include "phys.h"

#include "core.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <float.h>
#include <stdlib.h>

typedef struct lpPhysOrder
{
	uint64_t key;
	uint32_t tie;
	int index;
} lpPhysOrder;

struct lpPhys
{
	b3WorldId world;
	lpPhysPairFcn* pairFilter;
	void* context;
	b3ContactEvents events; // of the last lpPhys_GetHits, valid until the next step
	LP_ARRAY( b3ContactData ) contactData;
	LP_ARRAY( lpPhysOrder ) order;
	LP_ARRAY( lpPhysContact ) contacts;
	LP_ARRAY( lpPhysHit ) hits;
	LP_ARRAY( lpPhysMove ) moves;
	LP_ARRAY( int ) pieces;
};

// Identities until the handles become opaque (step 3's end)
static inline b3BodyId lpB3Body( lpPhysBody b )
{
	return b;
}

static inline lpPhysBody lpPhysBodyOf( b3BodyId id )
{
	return id;
}

static inline b3ShapeId lpB3Shape( lpPhysShape s )
{
	return s;
}

static inline lpPhysShape lpPhysShapeOf( b3ShapeId id )
{
	return id;
}

static inline b3JointId lpB3Joint( lpPhysJoint j )
{
	return j;
}

static inline lpPhysJoint lpPhysJointOf( b3JointId id )
{
	return id;
}

static inline void* lpUserData( int index )
{
	return index >= 0 ? (void*)(intptr_t)( index + 1 ) : NULL;
}

static inline int lpShapeIndex( b3ShapeId shape )
{
	intptr_t data = (intptr_t)b3Shape_GetUserData( shape );
	return data > 0 ? (int)( data - 1 ) : -1;
}

// ---- the world ----

static bool lpPairFilterB3( b3ShapeId shapeA, b3ShapeId shapeB, void* context )
{
	const lpPhys* p = context;
	return p->pairFilter( lpShapeIndex( shapeA ), lpShapeIndex( shapeB ), p->context );
}

lpPhys* lpPhys_Create( const lpPhysDef* def )
{
	lpPhys* p = lpAlloc( sizeof( lpPhys ) );
	memset( p, 0, sizeof( lpPhys ) );
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = def->gravity;
	wd.workerCount = (uint32_t)( def->workerCount > 1 ? def->workerCount : 1 );
	p->world = b3CreateWorld( &wd );
	b3World_SetHitEventThreshold( p->world, def->hitSpeed );
	p->pairFilter = def->pairFilter;
	p->context = def->context;
	if ( def->pairFilter != NULL )
	{
		b3World_SetCustomFilterCallback( p->world, lpPairFilterB3, p );
	}
	return p;
}

void lpPhys_Destroy( lpPhys* p )
{
	b3DestroyWorld( p->world );
	lpArray_Free( p->contactData );
	lpArray_Free( p->order );
	lpArray_Free( p->contacts );
	lpArray_Free( p->hits );
	lpArray_Free( p->moves );
	lpArray_Free( p->pieces );
	lpFree( p );
}

void lpPhys_Step( lpPhys* p, float timeStep, int subStepCount )
{
	p->events = ( b3ContactEvents ){ 0 };
	b3World_Step( p->world, timeStep, subStepCount );
}

b3WorldId lpPhys_Box3DWorld( const lpPhys* p );
b3WorldId lpPhys_Box3DWorld( const lpPhys* p )
{
	return p->world;
}

lpVec3 lpPhys_GetGravity( const lpPhys* p )
{
	return b3World_GetGravity( p->world );
}

lpPhysCounters lpPhys_GetCounters( const lpPhys* p )
{
	b3Counters c = b3World_GetCounters( p->world );
	lpPhysCounters out = { c.shapeCount, c.contactCount, c.awakeContactCount };
	return out;
}

// ---- bodies ----

lpPhysBodyDef lpPhys_DefaultBodyDef( void )
{
	lpPhysBodyDef def = { 0 };
	def.transform = lpTransform_identity;
	def.gravityScale = 1.0f;
	def.userData = -1;
	return def;
}

lpPhysBody lpPhys_CreateBody( lpPhys* p, const lpPhysBodyDef* def )
{
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = def->dynamic ? b3_dynamicBody : b3_staticBody;
	bd.position = def->transform.p;
	bd.rotation = def->transform.q;
	bd.linearVelocity = def->linearVelocity;
	bd.angularVelocity = def->angularVelocity;
	bd.gravityScale = def->gravityScale;
	if ( def->sleepThreshold > 0.0f )
	{
		bd.sleepThreshold = def->sleepThreshold;
	}
	bd.userData = lpUserData( def->userData );
	return lpPhysBodyOf( b3CreateBody( p->world, &bd ) );
}

void lpPhys_DestroyBody( lpPhys* p, lpPhysBody body )
{
	(void)p;
	b3DestroyBody( lpB3Body( body ) );
}

bool lpPhys_IsValidBody( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_IsValid( lpB3Body( body ) );
}

int lpPhys_GetShapeCount( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetShapeCount( lpB3Body( body ) );
}

bool lpPhys_IsDynamic( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetType( lpB3Body( body ) ) == b3_dynamicBody;
}

void lpPhys_SetDynamic( lpPhys* p, lpPhysBody body, bool dynamic )
{
	(void)p;
	b3Body_SetType( lpB3Body( body ), dynamic ? b3_dynamicBody : b3_staticBody );
}

lpWorldTransform lpPhys_GetTransform( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetTransform( lpB3Body( body ) );
}

lpPos lpPhys_GetWorldCenter( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetWorldCenter( lpB3Body( body ) );
}

lpVec3 lpPhys_GetLocalCenter( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetLocalCenter( lpB3Body( body ) );
}

lpVec3 lpPhys_GetLinearVelocity( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetLinearVelocity( lpB3Body( body ) );
}

lpVec3 lpPhys_GetAngularVelocity( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetAngularVelocity( lpB3Body( body ) );
}

lpVec3 lpPhys_GetPointVelocity( const lpPhys* p, lpPhysBody body, lpPos point )
{
	(void)p;
	return b3Body_GetWorldPointVelocity( lpB3Body( body ), point );
}

void lpPhys_SetLinearVelocity( lpPhys* p, lpPhysBody body, lpVec3 v )
{
	(void)p;
	b3Body_SetLinearVelocity( lpB3Body( body ), v );
}

void lpPhys_SetAngularVelocity( lpPhys* p, lpPhysBody body, lpVec3 omega )
{
	(void)p;
	b3Body_SetAngularVelocity( lpB3Body( body ), omega );
}

float lpPhys_GetMass( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetMass( lpB3Body( body ) );
}

lpMatrix3 lpPhys_GetInvInertia( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetWorldInverseRotationalInertia( lpB3Body( body ) );
}

void lpPhys_UpdateMass( lpPhys* p, lpPhysBody body, float inertiaRadius )
{
	(void)p;
	b3BodyId id = lpB3Body( body );
	b3Body_ApplyMassFromShapes( id );
	if ( inertiaRadius > 0.0f )
	{
		b3MassData md = b3Body_GetMassData( id );
		float add = md.mass * inertiaRadius * inertiaRadius;
		md.inertia.cx.x += add;
		md.inertia.cy.y += add;
		md.inertia.cz.z += add;
		b3Body_SetMassData( id, md );
	}
}

void lpPhys_ApplyForce( lpPhys* p, lpPhysBody body, lpVec3 force, lpPos point, bool wake )
{
	(void)p;
	b3Body_ApplyForce( lpB3Body( body ), force, point, wake );
}

void lpPhys_ApplyImpulse( lpPhys* p, lpPhysBody body, lpVec3 impulse, lpPos point, bool wake )
{
	(void)p;
	b3Body_ApplyLinearImpulse( lpB3Body( body ), impulse, point, wake );
}

bool lpPhys_IsAwake( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_IsAwake( lpB3Body( body ) );
}

void lpPhys_SetAwake( lpPhys* p, lpPhysBody body, bool awake )
{
	(void)p;
	b3Body_SetAwake( lpB3Body( body ), awake );
}

void lpPhys_SetSleepThreshold( lpPhys* p, lpPhysBody body, float speed )
{
	(void)p;
	b3Body_SetSleepThreshold( lpB3Body( body ), speed );
}

float lpPhys_GetGravityScale( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetGravityScale( lpB3Body( body ) );
}

void lpPhys_SetGravityScale( lpPhys* p, lpPhysBody body, float scale )
{
	(void)p;
	b3Body_SetGravityScale( lpB3Body( body ), scale );
}

lpAABB lpPhys_GetBounds( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_ComputeAABB( lpB3Body( body ) );
}

lpVec3 lpPhys_GetMaxExtent( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetMaxExtent( lpB3Body( body ) );
}

// ---- hulls and shapes ----

lpPhysHull* lpPhys_CreateHull( const lpVec3* points, int count, int maxVertices )
{
	return b3CreateHull( points, count, maxVertices );
}

void lpPhys_DestroyHull( lpPhysHull* hull )
{
	b3DestroyHull( hull );
}

lpPhysHullView lpPhys_GetHullView( const lpPhysHull* hull )
{
	const b3HullData* h = hull;
	lpPhysHullView view = { h->vertexCount, h->faceCount, b3GetHullPoints( h ), b3GetHullPlanes( h ) };
	return view;
}

int lpPhys_GetHullFace( const lpPhysHull* hull, int face, uint8_t* indices, int capacity )
{
	const b3HullData* h = hull;
	const b3HullFace* faces = b3GetHullFaces( h );
	const b3HullHalfEdge* edges = b3GetHullEdges( h );
	int first = faces[face].edge;
	int edge = first;
	int count = 0;
	do
	{
		if ( count >= capacity )
		{
			return -1;
		}
		indices[count++] = edges[edge].origin;
		edge = edges[edge].next;
	}
	while ( edge != first );
	return count;
}

float lpPhys_HullDistance( const lpVec3* a, int countA, const lpVec3* b, int countB )
{
	b3DistanceInput input = { 0 };
	input.proxyA = ( b3ShapeProxy ){ a, countA, 0.0f };
	input.proxyB = ( b3ShapeProxy ){ b, countB, 0.0f };
	input.transform = lpTransform_identity;
	input.useRadii = false;
	b3SimplexCache cache = { 0 };
	return b3ShapeDistance( &input, &cache, NULL, 0 ).distance;
}

lpPhysShape lpPhys_CreateHullShape( lpPhys* p, lpPhysBody body, const lpPhysShapeDef* def, const lpPhysHull* hull )
{
	(void)p;
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = def->density;
	sd.baseMaterial.friction = def->friction;
	sd.baseMaterial.restitution = def->restitution;
	sd.baseMaterial.userMaterialId = (uint64_t)def->material;
	sd.updateBodyMass = false;
	sd.userData = lpUserData( def->userData );
	sd.filter.categoryBits = def->filter.category;
	sd.filter.maskBits = def->filter.mask;
	sd.enableHitEvents = def->hitEvents;
	sd.enableCustomFiltering = def->customFilter;
	return lpPhysShapeOf( b3CreateHullShape( lpB3Body( body ), &sd, hull ) );
}

void lpPhys_DestroyShape( lpPhys* p, lpPhysShape shape )
{
	(void)p;
	b3DestroyShape( lpB3Shape( shape ), false );
}

bool lpPhys_IsValidShape( const lpPhys* p, lpPhysShape shape )
{
	(void)p;
	return b3Shape_IsValid( lpB3Shape( shape ) );
}

lpPhysBody lpPhys_GetShapeBody( const lpPhys* p, lpPhysShape shape )
{
	(void)p;
	return lpPhysBodyOf( b3Shape_GetBody( lpB3Shape( shape ) ) );
}

// ---- joints ----

lpPhysJoint lpPhys_CreateJoint( lpPhys* p, const lpPhysJointDef* def )
{
	b3JointDef base = b3DefaultWeldJointDef().base; // the common part is the same for every type
	base.bodyIdA = lpB3Body( def->bodyA );
	base.bodyIdB = lpB3Body( def->bodyB );
	base.localFrameA = def->frameA;
	base.localFrameB = def->frameB;
	base.collideConnected = def->collideConnected;

	b3JointId id;
	switch ( def->type )
	{
		case lp_physWeld:
		{
			b3WeldJointDef jd = b3DefaultWeldJointDef();
			jd.base = base;
			jd.linearHertz = def->hertz;
			jd.angularHertz = def->hertz;
			jd.linearDampingRatio = def->dampingRatio;
			jd.angularDampingRatio = def->dampingRatio;
			id = b3CreateWeldJoint( p->world, &jd );
			break;
		}
		case lp_physHinge:
		{
			b3RevoluteJointDef jd = b3DefaultRevoluteJointDef();
			jd.base = base;
			jd.enableLimit = def->lowerAngle < def->upperAngle;
			jd.lowerAngle = def->lowerAngle;
			jd.upperAngle = def->upperAngle;
			jd.enableMotor = def->enableMotor;
			id = b3CreateRevoluteJoint( p->world, &jd );
			break;
		}
		case lp_physBall:
		{
			b3SphericalJointDef jd = b3DefaultSphericalJointDef();
			jd.base = base;
			jd.enableConeLimit = def->coneAngle > 0.0f;
			jd.coneAngle = def->coneAngle;
			jd.enableMotor = def->enableMotor;
			id = b3CreateSphericalJoint( p->world, &jd );
			break;
		}
		default:
		{
			// No spring force, only the upper length limit, so it goes slack when pushed together
			b3DistanceJointDef jd = b3DefaultDistanceJointDef();
			jd.base = base;
			jd.length = def->length;
			jd.enableSpring = true;
			jd.hertz = 0.0f;
			jd.enableLimit = true;
			jd.minLength = 0.0f;
			jd.maxLength = def->length;
			id = b3CreateDistanceJoint( p->world, &jd );
			break;
		}
	}
	return lpPhysJointOf( id );
}

void lpPhys_DestroyJoint( lpPhys* p, lpPhysJoint joint, bool wakeBodies )
{
	(void)p;
	b3DestroyJoint( lpB3Joint( joint ), wakeBodies );
}

bool lpPhys_IsValidJoint( const lpPhys* p, lpPhysJoint joint )
{
	(void)p;
	return b3Joint_IsValid( lpB3Joint( joint ) );
}

void lpPhys_GetJointBodies( const lpPhys* p, lpPhysJoint joint, lpPhysBody* a, lpPhysBody* b )
{
	(void)p;
	*a = lpPhysBodyOf( b3Joint_GetBodyA( lpB3Joint( joint ) ) );
	*b = lpPhysBodyOf( b3Joint_GetBodyB( lpB3Joint( joint ) ) );
}

void lpPhys_WakeJoint( lpPhys* p, lpPhysJoint joint )
{
	(void)p;
	b3Joint_WakeBodies( lpB3Joint( joint ) );
}

void lpPhys_GetJointLoad( const lpPhys* p, lpPhysJoint joint, lpVec3* force, lpVec3* torque )
{
	(void)p;
	*force = b3Joint_GetConstraintForce( lpB3Joint( joint ) );
	*torque = b3Joint_GetConstraintTorque( lpB3Joint( joint ) );
}

float lpPhys_GetJointSeparation( const lpPhys* p, lpPhysJoint joint )
{
	(void)p;
	return b3Joint_GetLinearSeparation( lpB3Joint( joint ) );
}

float lpPhys_GetHingeAngle( const lpPhys* p, lpPhysJoint joint )
{
	(void)p;
	return b3RevoluteJoint_GetAngle( lpB3Joint( joint ) );
}

void lpPhys_SetHingeMotor( lpPhys* p, lpPhysJoint joint, float speed )
{
	(void)p;
	b3RevoluteJoint_SetMotorSpeed( lpB3Joint( joint ), speed );
}

void lpPhys_SetBallMotor( lpPhys* p, lpPhysJoint joint, lpVec3 omega )
{
	(void)p;
	b3SphericalJoint_SetMotorVelocity( lpB3Joint( joint ), omega );
}

void lpPhys_SetMotorMaxTorque( lpPhys* p, lpPhysJoint joint, float torque )
{
	(void)p;
	b3JointId id = lpB3Joint( joint );
	if ( b3Joint_GetType( id ) == b3_revoluteJoint )
	{
		b3RevoluteJoint_SetMaxMotorTorque( id, torque );
	}
	else
	{
		b3SphericalJoint_SetMaxMotorTorque( id, torque );
	}
}

float lpPhys_GetHingeMotorTorque( const lpPhys* p, lpPhysJoint joint )
{
	(void)p;
	return b3RevoluteJoint_GetMotorTorque( lpB3Joint( joint ) );
}

lpVec3 lpPhys_GetBallMotorTorque( const lpPhys* p, lpPhysJoint joint )
{
	(void)p;
	return b3SphericalJoint_GetMotorTorque( lpB3Joint( joint ) );
}

void lpPhys_SetRopeLength( lpPhys* p, lpPhysJoint joint, float length )
{
	(void)p;
	b3DistanceJoint_SetLengthRange( lpB3Joint( joint ), 0.0f, length );
}

// ---- reports ----

static int lpComparePhysOrder( const void* a, const void* b )
{
	const lpPhysOrder* x = a;
	const lpPhysOrder* y = b;
	if ( x->key != y->key )
	{
		return ( x->key > y->key ) - ( x->key < y->key );
	}
	if ( x->tie != y->tie )
	{
		return ( x->tie > y->tie ) - ( x->tie < y->tie );
	}
	return ( x->index > y->index ) - ( x->index < y->index );
}

int lpPhys_GetBodyContacts( lpPhys* p, lpPhysBody body, const lpPhysContact** contacts )
{
	b3BodyId id = lpB3Body( body );
	p->contacts.count = 0;
	*contacts = p->contacts.data;
	int capacity = b3Body_GetContactCapacity( id );
	if ( capacity == 0 )
	{
		return 0;
	}
	lpArray_Reserve( p->contactData, capacity );
	int count = b3Body_GetContactData( id, p->contactData.data, capacity );

	// Ordered by (own piece, other piece), then the other shape's index (shapes that are no piece), then report order
	p->order.count = 0;
	for ( int k = 0; k < count; ++k )
	{
		const b3ContactData* c = p->contactData.data + k;
		bool mineA = B3_ID_EQUALS( b3Shape_GetBody( c->shapeIdA ), id );
		b3ShapeId other = mineA ? c->shapeIdB : c->shapeIdA;
		int piece = lpShapeIndex( mineA ? c->shapeIdA : c->shapeIdB );
		int otherPiece = lpShapeIndex( other );
		if ( piece < 0 )
		{
			continue;
		}
		lpPhysOrder order = { ( (uint64_t)( piece + 1 ) << 32 ) | (uint64_t)( otherPiece + 1 ), (uint32_t)other.index1, k };
		lpArray_Push( p->order, order );
	}
	if ( p->order.count > 1 )
	{
		qsort( p->order.data, (size_t)p->order.count, sizeof( lpPhysOrder ), lpComparePhysOrder );
	}

	for ( int o = 0; o < p->order.count; ++o )
	{
		const b3ContactData* c = p->contactData.data + p->order.data[o].index;
		bool mineA = B3_ID_EQUALS( b3Shape_GetBody( c->shapeIdA ), id );
		int piece = (int)( p->order.data[o].key >> 32 ) - 1;
		int otherPiece = (int)( p->order.data[o].key & 0xFFFFFFFFu ) - 1;
		lpPos centerA = b3Body_GetWorldCenter( b3Shape_GetBody( c->shapeIdA ) );
		for ( int mi = 0; mi < c->manifoldCount; ++mi )
		{
			const b3Manifold* manifold = c->manifolds + mi;
			for ( int pi = 0; pi < manifold->pointCount; ++pi )
			{
				const b3ManifoldPoint* mp = manifold->points + pi;
				lpPhysContact contact = { piece,
										  otherPiece,
										  manifold->normal,
										  mineA,
										  lpOffsetPos( centerA, mp->anchorA ),
										  mp->separation,
										  mp->totalNormalImpulse };
				lpArray_Push( p->contacts, contact );
			}
		}
	}
	*contacts = p->contacts.data;
	return p->contacts.count;
}

static int lpCompareHits( const void* a, const void* b )
{
	const lpPhysHit* x = a;
	const lpPhysHit* y = b;
	if ( x->pair != y->pair )
	{
		return ( x->pair > y->pair ) - ( x->pair < y->pair );
	}
	if ( x->speed != y->speed )
	{
		return x->speed > y->speed ? -1 : 1; // the fastest first
	}
	if ( x->point.x != y->point.x )
	{
		return x->point.x < y->point.x ? -1 : 1;
	}
	if ( x->point.y != y->point.y )
	{
		return x->point.y < y->point.y ? -1 : 1;
	}
	if ( x->point.z != y->point.z )
	{
		return x->point.z < y->point.z ? -1 : 1;
	}
	return ( x->contact > y->contact ) - ( x->contact < y->contact ); // identical hits: either order acts the same
}

int lpPhys_GetHits( lpPhys* p, const lpPhysHit** hits )
{
	p->events = b3World_GetContactEvents( p->world );
	p->hits.count = 0;
	for ( int i = 0; i < p->events.hitCount; ++i )
	{
		const b3ContactHitEvent* e = p->events.hitEvents + i;
		int a = lpShapeIndex( e->shapeIdA );
		int b = lpShapeIndex( e->shapeIdB );
		uint64_t lo = (uint64_t)( a + 1 );
		uint64_t hi = (uint64_t)( b + 1 );
		lpPhysHit hit = { lo < hi ? ( lo << 32 ) | hi : ( hi << 32 ) | lo, a, b, e->approachSpeed, e->point, (uint64_t)i };
		lpArray_Push( p->hits, hit );
	}
	if ( p->hits.count > 1 )
	{
		qsort( p->hits.data, (size_t)p->hits.count, sizeof( lpPhysHit ), lpCompareHits );
	}
	*hits = p->hits.data;
	return p->hits.count;
}

bool lpPhys_GetContactCentroid( const lpPhys* p, uint64_t contact, lpPos* point )
{
	if ( contact >= (uint64_t)p->events.hitCount )
	{
		return false;
	}
	b3ContactId id = p->events.hitEvents[contact].contactId;
	if ( b3Contact_IsValid( id ) == false )
	{
		return false;
	}
	b3ContactData c = b3Contact_GetData( id );
	lpPos centerA = b3Body_GetWorldCenter( b3Shape_GetBody( c.shapeIdA ) );
	lpVec3 sum = lpVec3_zero;
	int count = 0;
	for ( int mi = 0; mi < c.manifoldCount; ++mi )
	{
		for ( int pi = 0; pi < c.manifolds[mi].pointCount; ++pi )
		{
			sum = lpAdd( sum, c.manifolds[mi].points[pi].anchorA );
			count += 1;
		}
	}
	if ( count == 0 )
	{
		return false;
	}
	*point = lpOffsetPos( centerA, lpMulSV( 1.0f / (float)count, sum ) );
	return true;
}

static int lpCompareMoves( const void* a, const void* b )
{
	const lpPhysMove* x = a;
	const lpPhysMove* y = b;
	return ( x->userData > y->userData ) - ( x->userData < y->userData );
}

int lpPhys_GetMoves( lpPhys* p, const lpPhysMove** moves )
{
	b3BodyEvents events = b3World_GetBodyEvents( p->world );
	p->moves.count = 0;
	for ( int i = 0; i < events.moveCount; ++i )
	{
		const b3BodyMoveEvent* e = events.moveEvents + i;
		intptr_t data = (intptr_t)e->userData;
		if ( data > 0 )
		{
			lpPhysMove move = { (int)( data - 1 ), e->transform, e->fellAsleep };
			lpArray_Push( p->moves, move );
		}
	}
	if ( p->moves.count > 1 )
	{
		qsort( p->moves.data, (size_t)p->moves.count, sizeof( lpPhysMove ), lpCompareMoves );
	}
	*moves = p->moves.data;
	return p->moves.count;
}

// ---- queries ----

static b3QueryFilter lpQueryFilter( lpPhysFilter filter )
{
	b3QueryFilter qf = b3DefaultQueryFilter();
	qf.categoryBits = filter.category;
	qf.maskBits = filter.mask;
	return qf;
}

static int lpCompareIndex( const void* a, const void* b )
{
	int x = *(const int*)a;
	int y = *(const int*)b;
	return ( x > y ) - ( x < y );
}

static bool lpCollectPieceFcn( b3ShapeId shapeId, void* context )
{
	lpPhys* p = context;
	int piece = lpShapeIndex( shapeId );
	if ( piece >= 0 )
	{
		lpArray_Push( p->pieces, piece );
	}
	return true;
}

int lpPhys_OverlapBox( lpPhys* p, lpAABB box, lpPhysFilter filter, const int** pieces )
{
	p->pieces.count = 0;
	b3World_OverlapAABB( p->world, box, lpQueryFilter( filter ), lpCollectPieceFcn, p );
	if ( p->pieces.count > 1 )
	{
		qsort( p->pieces.data, (size_t)p->pieces.count, sizeof( int ), lpCompareIndex );
		int unique = 1;
		for ( int i = 1; i < p->pieces.count; ++i )
		{
			if ( p->pieces.data[i] != p->pieces.data[unique - 1] )
			{
				p->pieces.data[unique++] = p->pieces.data[i];
			}
		}
		p->pieces.count = unique;
	}
	*pieces = p->pieces.data;
	return p->pieces.count;
}

typedef struct lpCastState
{
	lpPhysCastAcceptFcn* accept;
	void* context;
	lpPhysCastHit hit;
} lpCastState;

static float lpCastFcn( b3ShapeId shapeId, lpPos point, lpVec3 normal, float fraction, uint64_t userMaterialId,
						int triangleIndex, int childIndex, void* context )
{
	(void)triangleIndex;
	(void)childIndex;
	lpCastState* s = context;
	int piece = lpShapeIndex( shapeId );
	if ( s->accept != NULL && s->accept( piece, fraction, s->context ) == false )
	{
		return -1.0f;
	}
	lpPhysCastHit* h = &s->hit;
	if ( fraction < h->fraction || ( h->hit && fraction == h->fraction && piece < h->piece ) )
	{
		h->fraction = fraction;
		h->point = point;
		h->normal = normal;
		h->piece = piece;
		h->material = (int)userMaterialId;
		h->hit = true;
	}
	return lpNextUp( h->fraction ); // equal fractions still come in, so the tie rule sees them
}

lpPhysCastHit lpPhys_CastRay( const lpPhys* p, lpPos origin, lpVec3 translation, lpPhysFilter filter,
							  lpPhysCastAcceptFcn* accept, void* context )
{
	lpCastState s = { accept, context, { false, FLT_MAX, { 0 }, { 0 }, -1, -1 } };
	b3World_CastRay( p->world, origin, translation, lpQueryFilter( filter ), lpCastFcn, &s );
	return s.hit;
}

lpPhysCastHit lpPhys_CastShape( const lpPhys* p, lpPos origin, const lpVec3* points, int count, float radius,
								lpVec3 translation, lpPhysFilter filter, lpPhysCastAcceptFcn* accept, void* context )
{
	lpCastState s = { accept, context, { false, FLT_MAX, { 0 }, { 0 }, -1, -1 } };
	b3ShapeProxy proxy = { points, count, radius };
	b3World_CastShape( p->world, origin, &proxy, translation, lpQueryFilter( filter ), lpCastFcn, &s );
	return s.hit;
}
