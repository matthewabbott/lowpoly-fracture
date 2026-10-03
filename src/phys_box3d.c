// SPDX-License-Identifier: MIT
// The physics interface (phys.h) on Box3D: the only file that sees Box3D's headers (the build gives no other file
// their path). Box3D's ids pack into the handles (b3Store*Id), values cross by copy (same layouts, so exact), user
// data is index + 1 (0 for none), and reports are re-ordered here into our own total order.

#include "phys.h"

#include "core.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"

#include <float.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct lpPhysOrder
{
	uint64_t key;
	int index;
} lpPhysOrder;

typedef struct lpHitRecord
{
	lpPhysHit hit;
	int event; // in Box3D's report
} lpHitRecord;

struct lpPhys
{
	b3WorldId world;
	lpPhysPairFcn* pairFilter;
	void* context;
	b3ContactEvents events; // of the last lpPhys_GetHits, valid until the next step
	LP_ARRAY( b3ContactData ) contactData;
	LP_ARRAY( lpPhysOrder ) order;
	LP_ARRAY( lpPhysContact ) contacts;
	LP_ARRAY( lpHitRecord ) hitRecords;
	LP_ARRAY( lpPhysHit ) hits;
	LP_ARRAY( int ) hitEvents; // per hit, its event in Box3D's report
	LP_ARRAY( lpPhysMove ) moves;
	LP_ARRAY( uint64_t ) keys;
	LP_ARRAY( uint64_t ) keyScratch;
	LP_ARRAY( int ) pieces;
};

// ---- crossing the boundary ----

static inline b3Vec3 lpB3Vec( lpVec3 v )
{
	b3Vec3 out = { v.x, v.y, v.z };
	return out;
}

static inline lpVec3 lpVec( b3Vec3 v )
{
	lpVec3 out = { v.x, v.y, v.z };
	return out;
}

static inline b3Quat lpB3Quat( lpQuat q )
{
	b3Quat out = { lpB3Vec( q.v ), q.s };
	return out;
}

static inline lpQuat lpQuatOf( b3Quat q )
{
	lpQuat out = { lpVec( q.v ), q.s };
	return out;
}

static inline b3Transform lpB3Transform( lpTransform t )
{
	b3Transform out = { lpB3Vec( t.p ), lpB3Quat( t.q ) };
	return out;
}

static inline lpTransform lpTransformOf( b3Transform t )
{
	lpTransform out = { lpVec( t.p ), lpQuatOf( t.q ) };
	return out;
}

static inline lpMatrix3 lpMatrixOf( b3Matrix3 m )
{
	lpMatrix3 out = { lpVec( m.cx ), lpVec( m.cy ), lpVec( m.cz ) };
	return out;
}

static inline lpAABB lpAABBOf( b3AABB a )
{
	lpAABB out = { lpVec( a.lowerBound ), lpVec( a.upperBound ) };
	return out;
}

static inline b3AABB lpB3AABB( lpAABB a )
{
	b3AABB out = { lpB3Vec( a.lowerBound ), lpB3Vec( a.upperBound ) };
	return out;
}

static inline b3BodyId lpB3Body( lpPhysBody b )
{
	return b3LoadBodyId( b.handle );
}

static inline lpPhysBody lpPhysBodyOf( b3BodyId id )
{
	lpPhysBody b = { b3StoreBodyId( id ) };
	return b;
}

static inline b3ShapeId lpB3Shape( lpPhysShape s )
{
	return b3LoadShapeId( s.handle );
}

static inline lpPhysShape lpPhysShapeOf( b3ShapeId id )
{
	lpPhysShape s = { b3StoreShapeId( id ) };
	return s;
}

static inline b3JointId lpB3Joint( lpPhysJoint j )
{
	return b3LoadJointId( j.handle );
}

static inline lpPhysJoint lpPhysJointOf( b3JointId id )
{
	lpPhysJoint j = { b3StoreJointId( id ) };
	return j;
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

// Box3D's own assertion handler prints to stdout, which is lost when it breaks; ours says it on stderr first
static int lpBox3dAssert( const char* condition, const char* fileName, int lineNumber )
{
	fprintf( stderr, "BOX3D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );
	fflush( stderr );
	return 1;
}

lpPhys* lpPhys_Create( const lpPhysDef* def )
{
	b3SetAssertFcn( lpBox3dAssert );
	lpPhys* p = lpAlloc( sizeof( lpPhys ) );
	memset( p, 0, sizeof( lpPhys ) );
	b3WorldDef wd = b3DefaultWorldDef();
	wd.gravity = lpB3Vec( def->gravity );
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
	lpArray_Free( p->hitRecords );
	lpArray_Free( p->hits );
	lpArray_Free( p->hitEvents );
	lpArray_Free( p->moves );
	lpArray_Free( p->keys );
	lpArray_Free( p->keyScratch );
	lpArray_Free( p->pieces );
	lpFree( p );
}

void lpPhys_Step( lpPhys* p, float timeStep, int subStepCount )
{
	p->events = ( b3ContactEvents ){ 0 };
	b3World_Step( p->world, timeStep, subStepCount );
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
	def.body = -1;
	return def;
}

lpPhysBody lpPhys_CreateBody( lpPhys* p, const lpPhysBodyDef* def )
{
	b3BodyDef bd = b3DefaultBodyDef();
	bd.type = def->dynamic ? b3_dynamicBody : b3_staticBody;
	bd.position = lpB3Vec( def->transform.p );
	bd.rotation = lpB3Quat( def->transform.q );
	bd.linearVelocity = lpB3Vec( def->linearVelocity );
	bd.angularVelocity = lpB3Vec( def->angularVelocity );
	bd.gravityScale = def->gravityScale;
	if ( def->sleepThreshold > 0.0f )
	{
		bd.sleepThreshold = def->sleepThreshold;
	}
	bd.userData = lpUserData( def->body );
	return lpPhysBodyOf( b3CreateBody( p->world, &bd ) );
}

void lpPhys_DestroyBody( lpPhys* p, lpPhysBody body )
{
	b3DestroyBody( lpB3Body( body ) );
}

bool lpPhys_IsValidBody( const lpPhys* p, lpPhysBody body )
{
	return b3Body_IsValid( lpB3Body( body ) );
}

int lpPhys_GetShapeCount( const lpPhys* p, lpPhysBody body )
{
	return b3Body_GetShapeCount( lpB3Body( body ) );
}

bool lpPhys_IsDynamic( const lpPhys* p, lpPhysBody body )
{
	return b3Body_GetType( lpB3Body( body ) ) == b3_dynamicBody;
}

void lpPhys_SetDynamic( lpPhys* p, lpPhysBody body, bool dynamic )
{
	b3Body_SetType( lpB3Body( body ), dynamic ? b3_dynamicBody : b3_staticBody );
}

lpWorldTransform lpPhys_GetTransform( const lpPhys* p, lpPhysBody body )
{
	return lpTransformOf( b3Body_GetTransform( lpB3Body( body ) ) );
}

lpPos lpPhys_GetWorldCenter( const lpPhys* p, lpPhysBody body )
{
	return lpVec( b3Body_GetWorldCenter( lpB3Body( body ) ) );
}

lpVec3 lpPhys_GetLocalCenter( const lpPhys* p, lpPhysBody body )
{
	return lpVec( b3Body_GetLocalCenter( lpB3Body( body ) ) );
}

lpVec3 lpPhys_GetLinearVelocity( const lpPhys* p, lpPhysBody body )
{
	return lpVec( b3Body_GetLinearVelocity( lpB3Body( body ) ) );
}

lpVec3 lpPhys_GetAngularVelocity( const lpPhys* p, lpPhysBody body )
{
	return lpVec( b3Body_GetAngularVelocity( lpB3Body( body ) ) );
}

lpVec3 lpPhys_GetPointVelocity( const lpPhys* p, lpPhysBody body, lpPos point )
{
	return lpVec( b3Body_GetWorldPointVelocity( lpB3Body( body ), lpB3Vec( point ) ) );
}

void lpPhys_SetLinearVelocity( lpPhys* p, lpPhysBody body, lpVec3 v )
{
	b3Body_SetLinearVelocity( lpB3Body( body ), lpB3Vec( v ) );
}

void lpPhys_SetAngularVelocity( lpPhys* p, lpPhysBody body, lpVec3 omega )
{
	b3Body_SetAngularVelocity( lpB3Body( body ), lpB3Vec( omega ) );
}

float lpPhys_GetMass( const lpPhys* p, lpPhysBody body )
{
	return b3Body_GetMass( lpB3Body( body ) );
}

lpMatrix3 lpPhys_GetInvInertia( const lpPhys* p, lpPhysBody body )
{
	return lpMatrixOf( b3Body_GetWorldInverseRotationalInertia( lpB3Body( body ) ) );
}

void lpPhys_UpdateMass( lpPhys* p, lpPhysBody body, float inertiaRadius )
{
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
	b3Body_ApplyForce( lpB3Body( body ), lpB3Vec( force ), lpB3Vec( point ), wake );
}

void lpPhys_ApplyImpulse( lpPhys* p, lpPhysBody body, lpVec3 impulse, lpPos point, bool wake )
{
	b3Body_ApplyLinearImpulse( lpB3Body( body ), lpB3Vec( impulse ), lpB3Vec( point ), wake );
}

float lpPhys_GetSleepTime( const lpPhys* p, lpPhysBody body )
{
	(void)p;
	return b3Body_GetSleepTime( lpB3Body( body ) );
}

typedef struct lpContactSums
{
	uint64_t* sums;
	int bodyCount;
} lpContactSums;

// One touching contact's state, field by field (manifold points have padding), keyed by its two pieces
static void lpHashContactState( void* shapeA, void* shapeB, void* bodyA, void* bodyB, const b3ContactState* s, void* context )
{
	lpContactSums* out = context;
	int32_t head[4] = { (int32_t)(intptr_t)shapeA, (int32_t)(intptr_t)shapeB, s->manifoldCount, (int32_t)s->flags };
	uint64_t h = lpHashWords( LP_HASH_INIT, head, sizeof( head ) );
	float cache[15] = { s->cachedRotationA.v.x, s->cachedRotationA.v.y, s->cachedRotationA.v.z, s->cachedRotationA.s,
						s->cachedRotationB.v.x, s->cachedRotationB.v.y, s->cachedRotationB.v.z, s->cachedRotationB.s,
						s->cachedRelativePose.p.x, s->cachedRelativePose.p.y, s->cachedRelativePose.p.z, s->cachedRelativePose.q.v.x,
						s->cachedRelativePose.q.v.y, s->cachedRelativePose.q.v.z, s->cachedRelativePose.q.s };
	h = lpHashWords( h, cache, sizeof( cache ) );
	for ( int m = 0; m < s->manifoldCount; ++m )
	{
		const b3Manifold* manifold = s->manifolds + m;
		float impulses[11] = { manifold->normal.x,			manifold->normal.y,			 manifold->normal.z,
							   manifold->twistImpulse,		manifold->frictionImpulse.x, manifold->frictionImpulse.y,
							   manifold->frictionImpulse.z, manifold->rollingImpulse.x,	 manifold->rollingImpulse.y,
							   manifold->rollingImpulse.z,	(float)manifold->pointCount };
		h = lpHashWords( h, impulses, sizeof( impulses ) );
		for ( int k = 0; k < manifold->pointCount; ++k )
		{
			const b3ManifoldPoint* mp = manifold->points + k;
			float point[10] = { mp->anchorA.x,		mp->anchorA.y,	  mp->anchorA.z,		 mp->anchorB.x, mp->anchorB.y,
								mp->anchorB.z,		mp->normalImpulse, mp->totalNormalImpulse, mp->separation, mp->baseSeparation };
			uint32_t feature[2] = { mp->featureId, mp->persisted ? 1u : 0u };
			h = lpHashWords( h, point, sizeof( point ) );
			h = lpHashWords( h, feature, sizeof( feature ) );
		}
	}
	h = lpMix64( h );
	int a = (int)(intptr_t)bodyA - 1, b = (int)(intptr_t)bodyB - 1;
	if ( a >= 0 && a < out->bodyCount )
	{
		out->sums[a] += h;
	}
	if ( b >= 0 && b < out->bodyCount )
	{
		out->sums[b] += h;
	}
}

void lpPhys_HashContacts( lpPhys* p, bool awakeOnly, uint64_t* sums, int bodyCount )
{
	lpContactSums out = { sums, bodyCount };
	b3World_VisitContactState( p->world, awakeOnly, lpHashContactState, &out );
}

bool lpPhys_IsAwake( const lpPhys* p, lpPhysBody body )
{
	return b3Body_IsAwake( lpB3Body( body ) );
}

void lpPhys_SetAwake( lpPhys* p, lpPhysBody body, bool awake )
{
	b3Body_SetAwake( lpB3Body( body ), awake );
}

void lpPhys_SetSleepThreshold( lpPhys* p, lpPhysBody body, float speed )
{
	b3Body_SetSleepThreshold( lpB3Body( body ), speed );
}

float lpPhys_GetGravityScale( const lpPhys* p, lpPhysBody body )
{
	return b3Body_GetGravityScale( lpB3Body( body ) );
}

void lpPhys_SetGravityScale( lpPhys* p, lpPhysBody body, float scale )
{
	b3Body_SetGravityScale( lpB3Body( body ), scale );
}

lpAABB lpPhys_GetBounds( const lpPhys* p, lpPhysBody body )
{
	return lpAABBOf( b3Body_ComputeAABB( lpB3Body( body ) ) );
}

lpVec3 lpPhys_GetMaxExtent( const lpPhys* p, lpPhysBody body )
{
	return lpVec( b3Body_GetMaxExtent( lpB3Body( body ) ) );
}

// ---- hulls and shapes ----

lpPhysHull* lpPhys_CreateHull( const lpVec3* points, int count, int maxVertices )
{
	b3Vec3 local[LP_PHYS_MAX_POINTS];
	b3Vec3* copy = count <= LP_PHYS_MAX_POINTS ? local : lpAlloc( (size_t)count * sizeof( b3Vec3 ) );
	for ( int i = 0; i < count; ++i )
	{
		copy[i] = lpB3Vec( points[i] );
	}
	b3HullData* hull = b3CreateHull( copy, count, maxVertices );
	if ( copy != local )
	{
		lpFree( copy );
	}
	return (lpPhysHull*)hull;
}

void lpPhys_DestroyHull( lpPhysHull* hull )
{
	b3DestroyHull( (b3HullData*)hull );
}

int lpPhys_GetHullVertexCount( const lpPhysHull* hull )
{
	return ( (const b3HullData*)hull )->vertexCount;
}

int lpPhys_GetHullFaceCount( const lpPhysHull* hull )
{
	return ( (const b3HullData*)hull )->faceCount;
}

lpVec3 lpPhys_GetHullPoint( const lpPhysHull* hull, int vertex )
{
	return lpVec( b3GetHullPoints( (const b3HullData*)hull )[vertex] );
}

lpPlane lpPhys_GetHullPlane( const lpPhysHull* hull, int face )
{
	b3Plane plane = b3GetHullPlanes( (const b3HullData*)hull )[face];
	lpPlane out = { lpVec( plane.normal ), plane.offset };
	return out;
}

int lpPhys_GetHullFace( const lpPhysHull* hull, int face, uint8_t* indices, int capacity )
{
	const b3HullData* h = (const b3HullData*)hull;
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
	LP_ASSERT( countA <= LP_PHYS_MAX_POINTS && countB <= LP_PHYS_MAX_POINTS );
	b3Vec3 pointsA[LP_PHYS_MAX_POINTS];
	b3Vec3 pointsB[LP_PHYS_MAX_POINTS];
	for ( int i = 0; i < countA; ++i )
	{
		pointsA[i] = lpB3Vec( a[i] );
	}
	for ( int i = 0; i < countB; ++i )
	{
		pointsB[i] = lpB3Vec( b[i] );
	}
	b3DistanceInput input = { 0 };
	input.proxyA = ( b3ShapeProxy ){ pointsA, countA, 0.0f };
	input.proxyB = ( b3ShapeProxy ){ pointsB, countB, 0.0f };
	input.transform = b3Transform_identity;
	input.useRadii = false;
	b3SimplexCache cache = { 0 };
	return b3ShapeDistance( &input, &cache, NULL, 0 ).distance;
}

lpPhysShape lpPhys_CreateHullShape( lpPhys* p, lpPhysBody body, const lpPhysShapeDef* def, const lpPhysHull* hull )
{
	b3ShapeDef sd = b3DefaultShapeDef();
	sd.density = def->density;
	sd.baseMaterial.friction = def->friction;
	sd.baseMaterial.restitution = def->restitution;
	sd.baseMaterial.userMaterialId = (uint64_t)def->material;
	sd.updateBodyMass = false;
	sd.userData = lpUserData( def->piece );
	sd.filter.categoryBits = def->filter.category;
	sd.filter.maskBits = def->filter.mask;
	sd.enableHitEvents = def->hitEvents;
	sd.enableCustomFiltering = def->customFilter;
	return lpPhysShapeOf( b3CreateHullShape( lpB3Body( body ), &sd, (const b3HullData*)hull ) );
}

void lpPhys_DestroyShape( lpPhys* p, lpPhysShape shape )
{
	b3DestroyShape( lpB3Shape( shape ), false );
}

bool lpPhys_IsValidShape( const lpPhys* p, lpPhysShape shape )
{
	return b3Shape_IsValid( lpB3Shape( shape ) );
}

lpPhysBody lpPhys_GetShapeBody( const lpPhys* p, lpPhysShape shape )
{
	return lpPhysBodyOf( b3Shape_GetBody( lpB3Shape( shape ) ) );
}

// ---- joints ----

lpPhysJoint lpPhys_CreateJoint( lpPhys* p, const lpPhysJointDef* def )
{
	b3JointDef base = b3DefaultWeldJointDef().base; // the common part is the same for every type
	base.bodyIdA = lpB3Body( def->bodyA );
	base.bodyIdB = lpB3Body( def->bodyB );
	base.localFrameA = lpB3Transform( def->frameA );
	base.localFrameB = lpB3Transform( def->frameB );
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
	b3DestroyJoint( lpB3Joint( joint ), wakeBodies );
}

bool lpPhys_IsValidJoint( const lpPhys* p, lpPhysJoint joint )
{
	return b3Joint_IsValid( lpB3Joint( joint ) );
}

void lpPhys_GetJointBodies( const lpPhys* p, lpPhysJoint joint, lpPhysBody* a, lpPhysBody* b )
{
	*a = lpPhysBodyOf( b3Joint_GetBodyA( lpB3Joint( joint ) ) );
	*b = lpPhysBodyOf( b3Joint_GetBodyB( lpB3Joint( joint ) ) );
}

void lpPhys_WakeJoint( lpPhys* p, lpPhysJoint joint )
{
	b3Joint_WakeBodies( lpB3Joint( joint ) );
}

void lpPhys_GetJointLoad( const lpPhys* p, lpPhysJoint joint, lpVec3* force, lpVec3* torque )
{
	*force = lpVec( b3Joint_GetConstraintForce( lpB3Joint( joint ) ) );
	*torque = lpVec( b3Joint_GetConstraintTorque( lpB3Joint( joint ) ) );
}

float lpPhys_GetJointSeparation( const lpPhys* p, lpPhysJoint joint )
{
	return b3Joint_GetLinearSeparation( lpB3Joint( joint ) );
}

float lpPhys_GetHingeAngle( const lpPhys* p, lpPhysJoint joint )
{
	return b3RevoluteJoint_GetAngle( lpB3Joint( joint ) );
}

void lpPhys_SetHingeMotor( lpPhys* p, lpPhysJoint joint, float speed )
{
	b3RevoluteJoint_SetMotorSpeed( lpB3Joint( joint ), speed );
}

void lpPhys_SetBallMotor( lpPhys* p, lpPhysJoint joint, lpVec3 omega )
{
	b3SphericalJoint_SetMotorVelocity( lpB3Joint( joint ), lpB3Vec( omega ) );
}

void lpPhys_SetMotorMaxTorque( lpPhys* p, lpPhysJoint joint, float torque )
{
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
	return b3RevoluteJoint_GetMotorTorque( lpB3Joint( joint ) );
}

lpVec3 lpPhys_GetBallMotorTorque( const lpPhys* p, lpPhysJoint joint )
{
	return lpVec( b3SphericalJoint_GetMotorTorque( lpB3Joint( joint ) ) );
}

void lpPhys_SetRopeLength( lpPhys* p, lpPhysJoint joint, float length )
{
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

	// Ordered by (own piece, other piece): one contact per pair of shapes, every shape a piece
	p->order.count = 0;
	for ( int k = 0; k < count; ++k )
	{
		const b3ContactData* c = p->contactData.data + k;
		bool mineA = B3_ID_EQUALS( b3Shape_GetBody( c->shapeIdA ), id );
		int piece = lpShapeIndex( mineA ? c->shapeIdA : c->shapeIdB );
		int otherPiece = lpShapeIndex( mineA ? c->shapeIdB : c->shapeIdA );
		if ( piece < 0 )
		{
			continue;
		}
		lpPhysOrder order = { ( (uint64_t)( piece + 1 ) << 32 ) | (uint64_t)( otherPiece + 1 ), k };
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
		lpPos centerA = lpVec( b3Body_GetWorldCenter( b3Shape_GetBody( c->shapeIdA ) ) );
		for ( int mi = 0; mi < c->manifoldCount; ++mi )
		{
			const b3Manifold* manifold = c->manifolds + mi;
			lpVec3 push = mineA ? lpNeg( lpVec( manifold->normal ) ) : lpVec( manifold->normal ); // Box3D's runs A to B
			for ( int pi = 0; pi < manifold->pointCount; ++pi )
			{
				const b3ManifoldPoint* mp = manifold->points + pi;
				lpPos point = lpOffsetPos( centerA, lpVec( mp->anchorA ) );
				lpPhysContact contact = { piece, otherPiece, push, point, mp->separation, mp->totalNormalImpulse };
				lpArray_Push( p->contacts, contact );
			}
		}
	}
	*contacts = p->contacts.data;
	return p->contacts.count;
}

static int lpCompareHits( const void* a, const void* b )
{
	const lpHitRecord* rx = a;
	const lpHitRecord* ry = b;
	const lpPhysHit* x = &rx->hit;
	const lpPhysHit* y = &ry->hit;
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
	return ( rx->event > ry->event ) - ( rx->event < ry->event ); // identical hits: either order acts the same
}

int lpPhys_GetHits( lpPhys* p, const lpPhysHit** hits )
{
	p->events = b3World_GetContactEvents( p->world );
	p->hitRecords.count = 0;
	for ( int i = 0; i < p->events.hitCount; ++i )
	{
		const b3ContactHitEvent* e = p->events.hitEvents + i;
		int a = lpShapeIndex( e->shapeIdA );
		int b = lpShapeIndex( e->shapeIdB );
		uint64_t lo = (uint64_t)( a + 1 );
		uint64_t hi = (uint64_t)( b + 1 );
		uint64_t pair = lo < hi ? ( lo << 32 ) | hi : ( hi << 32 ) | lo;
		lpHitRecord record = { { pair, a, b, e->approachSpeed, lpVec( e->point ) }, i };
		lpArray_Push( p->hitRecords, record );
	}
	int count = p->hitRecords.count;
	if ( count > 1 )
	{
		qsort( p->hitRecords.data, (size_t)count, sizeof( lpHitRecord ), lpCompareHits );
	}
	lpArray_Reserve( p->hits, count );
	lpArray_Reserve( p->hitEvents, count );
	for ( int i = 0; i < count; ++i )
	{
		p->hits.data[i] = p->hitRecords.data[i].hit;
		p->hitEvents.data[i] = p->hitRecords.data[i].event;
	}
	p->hits.count = count;
	p->hitEvents.count = count;
	*hits = p->hits.data;
	return count;
}

bool lpPhys_GetContactCentroid( const lpPhys* p, int hit, lpPos* point )
{
	if ( hit < 0 || hit >= p->hitEvents.count || p->hitEvents.data[hit] >= p->events.hitCount )
	{
		return false;
	}
	b3ContactId id = p->events.hitEvents[p->hitEvents.data[hit]].contactId;
	if ( b3Contact_IsValid( id ) == false )
	{
		return false;
	}
	b3ContactData c = b3Contact_GetData( id );
	lpPos centerA = lpVec( b3Body_GetWorldCenter( b3Shape_GetBody( c.shapeIdA ) ) );
	lpVec3 sum = lpVec3_zero;
	int count = 0;
	for ( int mi = 0; mi < c.manifoldCount; ++mi )
	{
		for ( int pi = 0; pi < c.manifolds[mi].pointCount; ++pi )
		{
			sum = lpAdd( sum, lpVec( c.manifolds[mi].points[pi].anchorA ) );
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

int lpPhys_GetMoves( lpPhys* p, const lpPhysMove** moves )
{
	b3BodyEvents events = b3World_GetBodyEvents( p->world );
	lpArray_Reserve( p->keys, events.moveCount );
	lpArray_Reserve( p->keyScratch, events.moveCount );
	int count = 0;
	for ( int i = 0; i < events.moveCount; ++i )
	{
		intptr_t data = (intptr_t)events.moveEvents[i].userData;
		if ( data > 0 )
		{
			p->keys.data[count++] = ( (uint64_t)data << 32 ) | (uint64_t)i; // body index + 1, then report order
		}
	}
	lpRadixSort64( p->keys.data, p->keyScratch.data, count, 32 ); // one move per body, so the high halves differ
	for ( int k = 1; k < count; ++k )
	{
		LP_ASSERT( ( p->keys.data[k] >> 32 ) != ( p->keys.data[k - 1] >> 32 ) );
	}
	lpArray_Reserve( p->moves, count );
	for ( int k = 0; k < count; ++k )
	{
		const b3BodyMoveEvent* e = events.moveEvents + ( p->keys.data[k] & 0xFFFFFFFFu );
		lpPhysMove move = { (int)( p->keys.data[k] >> 32 ) - 1, lpTransformOf( e->transform ), e->fellAsleep };
		p->moves.data[k] = move;
	}
	p->moves.count = count;
	*moves = p->moves.data;
	return count;
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
	b3World_OverlapAABB( p->world, lpB3AABB( box ), lpQueryFilter( filter ), lpCollectPieceFcn, p );
	if ( p->pieces.count > 1 )
	{
		qsort( p->pieces.data, (size_t)p->pieces.count, sizeof( int ), lpCompareIndex );
		for ( int i = 1; i < p->pieces.count; ++i )
		{
			LP_ASSERT( p->pieces.data[i] != p->pieces.data[i - 1] ); // one shape per piece
		}
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

static float lpCastFcn( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
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
		h->point = lpVec( point );
		h->normal = lpVec( normal );
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
	b3World_CastRay( p->world, lpB3Vec( origin ), lpB3Vec( translation ), lpQueryFilter( filter ), lpCastFcn, &s );
	return s.hit;
}

lpPhysCastHit lpPhys_CastShape( const lpPhys* p, lpPos origin, const lpVec3* points, int count, float radius,
								lpVec3 translation, lpPhysFilter filter, lpPhysCastAcceptFcn* accept, void* context )
{
	LP_ASSERT( count <= LP_PHYS_MAX_POINTS );
	b3Vec3 copy[LP_PHYS_MAX_POINTS];
	for ( int i = 0; i < count; ++i )
	{
		copy[i] = lpB3Vec( points[i] );
	}
	lpCastState s = { accept, context, { false, FLT_MAX, { 0 }, { 0 }, -1, -1 } };
	b3ShapeProxy proxy = { copy, count, radius };
	b3QueryFilter qf = lpQueryFilter( filter );
	b3World_CastShape( p->world, lpB3Vec( origin ), &proxy, lpB3Vec( translation ), qf, lpCastFcn, &s );
	return s.hit;
}
