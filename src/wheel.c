// SPDX-License-Identifier: MIT
// Vehicles: bodies on wheels.
//
// A wheel is a link (link.c) with no joint: end 0 on the piece at its mount, end 1 on nothing. So it follows its piece
// through splits and fractures, takes blast damage, strains under its own load and tears off a chip, like any link;
// when it breaks it comes off as a wheel object at the start of the next step. A vehicle holds its controls and its
// wheels' links, never a body: a chassis cut in two keeps on each half the wheels mounted there.
//
// Each step, before the physics step:
// - steering turns toward the control at a limited rate;
// - each wheel on an awake chassis casts its tyre (a disc of rim points with a radius of half its width) down the
//   suspension from its mount, so it rolls up kerbs and ignores light debris;
// - the suspension is an explicit spring-damper along the contact normal, damped by the chassis's own motion (a kerb
//   does not jolt it);
// - the tyres' grip is a small impulse solve per chassis body, on a copy of its velocity after gravity and the springs:
//   sideways and along the tyre within a friction circle, the drive a motor toward top speed with a capped force,
//   brakes and rolling resistance capped too, and a one-way stop when the suspension bottoms out. The result goes to
//   Box3D as forces (the side force raised toward the centre of mass by rollFactor), and the opposite onto a moving
//   ground body.
// Wheels are stepped in index order and solved in (body, wheel) order with a fixed iteration count, so the result does
// not depend on anything but the simulation state. A parked chassis falls asleep; it wakes when its controls change or
// the ground under a wheel goes.

#include "world.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define LP_WHEEL_ITERATIONS 4		  // of the tyre solve per body per step
#define LP_WHEEL_HERTZ 1.4f			  // default suspension: the chassis's share on a wheel bounces at this rate
#define LP_WHEEL_ZETA 0.5f			  // default damping ratio
#define LP_WHEEL_SLIDING_GRIP 0.8f	  // grip while sliding, of the grip
#define LP_WHEEL_HANDBRAKE_GRIP 0.5f  // sideways grip of a wheel locked by the handbrake
#define LP_WHEEL_RIM 8				  // points of the tyre's cast shape
#define LP_WHEEL_RECHECK_TICKS 30	  // a structure under a wheel is re-checked at most this often for a changed load
#define LP_PI 3.14159265f

lpWheelDef lpDefaultWheelDef( void )
{
	lpWheelDef def = { 0 };
	def.radius = 0.35f;
	def.width = 0.22f;
	def.restLength = 0.5f;
	def.maxLength = 0.5f;
	def.grip = 1.2f;
	def.brakeShare = 0.25f;
	def.maxForce = 60000.0f;
	def.strength = 5000.0f;
	def.material = lp_wood;
	def.color = 0x2B2B2Bu;
	return def;
}

lpVehicleDef lpDefaultVehicleDef( void )
{
	lpVehicleDef def = { 0 };
	def.body = -1;
	def.forward = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	def.up = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	def.maxDriveForce = 8000.0f;
	def.maxSpeed = 40.0f;
	def.maxBrakeForce = 12000.0f;
	def.maxSteer = 0.6f;
	def.steerSpeed = 2.5f;
	def.rollFactor = 0.3f;
	def.rollingResistance = 0.015f;
	return def;
}

static int lpAllocWheel( lpWorld* w )
{
	int index;
	if ( w->freeWheel != -1 )
	{
		index = w->freeWheel;
		w->freeWheel = w->wheels.data[index].nextFree;
	}
	else
	{
		index = w->wheels.count;
		lpWheel zero = { 0 };
		lpArray_Push( w->wheels, zero );
	}
	lpWheel* wh = w->wheels.data + index;
	memset( wh, 0, sizeof( lpWheel ) );
	wh->link = -1;
	wh->nextFree = -1;
	wh->groundPiece = -1;
	wh->stressBody = -1;
	return index;
}

static void lpFreeWheel( lpWorld* w, int index )
{
	lpWheel* wh = w->wheels.data + index;
	wh->link = -1;
	wh->nextFree = w->freeWheel;
	w->freeWheel = index;
}

static int lpWheelMountBody( const lpWorld* w, const lpWheel* wh )
{
	return w->pieces.data[w->links.data[wh->link].ends[0].piece].body;
}

// How well what the wheel's drive (or steering) needs is fed at its mount, 0 to 1
static float lpWheelSupply( const lpWorld* w, const lpWheel* wh, uint8_t needs )
{
	return lpSupplyOf( w->pieces.data + w->links.data[wh->link].ends[0].piece, needs );
}

// The tyre's frame in the chassis body frame: x the axle (up x forward), y up, z forward, turned by steering and spin
static b3Quat lpWheelRotation( const lpVehicle* v, const lpWheel* wh )
{
	b3Matrix3 m = { b3Cross( v->up, v->forward ), v->up, v->forward };
	b3Quat base = b3MakeQuatFromMatrix( &m );
	b3Quat steer = b3MakeQuatFromAxisAngle( v->up, -wh->steer );
	b3Quat spin = b3MakeQuatFromAxisAngle( (b3Vec3){ 1.0f, 0.0f, 0.0f }, wh->spin );
	return b3MulQuat( steer, b3MulQuat( base, spin ) );
}

// Hub pose and the link's cached points (the ends of the axle, so rays and blasts find the tyre)
static void lpPoseWheel( lpWorld* w, lpWheel* wh, b3WorldTransform xf, b3BodyId body )
{
	const lpVehicle* v = w->vehicles.data + wh->vehicle;
	lpLink* l = w->links.data + wh->link;
	b3Vec3 hubLocal = b3MulAdd( l->ends[0].frame.p, -wh->length, v->up );
	wh->hub.p = b3TransformWorldPoint( xf, hubLocal );
	wh->hub.q = b3MulQuat( xf.q, lpWheelRotation( v, wh ) );
	wh->hubVelocity = b3Body_GetWorldPointVelocity( body, wh->hub.p );
	wh->bodyOmega = b3Body_GetAngularVelocity( body );
	b3Vec3 axle = b3RotateVector( wh->hub.q, (b3Vec3){ 0.5f * wh->def.width, 0.0f, 0.0f } );
	l->points[0] = b3OffsetPos( wh->hub.p, b3Neg( axle ) );
	l->points[1] = b3OffsetPos( wh->hub.p, axle );
}

int lpCreateVehicle( lpWorld* w, const lpVehicleDef* def )
{
	if ( def->body < 0 || def->body >= w->bodies.count || def->wheels == NULL || def->wheelCount < 1 ||
		 def->wheelCount > LP_MAX_VEHICLE_WHEELS || w->linkCount + def->wheelCount > w->def.maxLinks )
	{
		return -1;
	}
	const lpBody* b = w->bodies.data + def->body;
	if ( b->alive == false || B3_IS_NULL( b->id ) || b3Body_GetType( b->id ) != b3_dynamicBody )
	{
		return -1;
	}
	b3WorldTransform xf = b3Body_GetTransform( b->id );
	b3Vec3 forward = b3Normalize( def->forward );
	b3Vec3 up = b3Normalize( b3Sub( def->up, b3MulSV( b3Dot( def->up, forward ), forward ) ) );
	float sprungMass = b3Body_GetMass( b->id ) / (float)def->wheelCount;

	int index = w->vehicles.count;
	lpVehicle zero = { 0 };
	lpArray_Push( w->vehicles, zero );
	lpVehicle* v = w->vehicles.data + index;
	v->def = *def;
	v->def.wheels = NULL;
	v->forward = b3InvRotateVector( xf.q, forward );
	v->up = b3InvRotateVector( xf.q, up );
	v->alive = true;
	v->wheelCount = def->wheelCount;
	for ( int i = 0; i < LP_MAX_VEHICLE_WHEELS; ++i )
	{
		v->links[i] = -1;
	}

	for ( int i = 0; i < def->wheelCount; ++i )
	{
		int wi = lpAllocWheel( w );
		lpWheel* wh = w->wheels.data + wi;
		wh->def = def->wheels[i];
		wh->def.radius = b3MaxFloat( wh->def.radius, 0.05f );
		wh->def.width = b3ClampFloat( wh->def.width, 0.02f, 1.8f * wh->def.radius );
		wh->def.maxLength = b3MaxFloat( wh->def.maxLength, 0.01f );
		if ( wh->def.stiffness <= 0.0f )
		{
			float omega = 2.0f * LP_PI * LP_WHEEL_HERTZ;
			wh->def.stiffness = sprungMass * omega * omega;
		}
		if ( wh->def.damping <= 0.0f )
		{
			wh->def.damping = 2.0f * LP_WHEEL_ZETA * sqrtf( wh->def.stiffness * sprungMass );
		}
		wh->vehicle = index;
		wh->slot = i;
		wh->sprungMass = sprungMass;
		wh->length = wh->def.maxLength;
		int link = lpCreateWheelLink( w, def->body, wh->def.mount, wh->def.maxForce, wh->def.strength, wi );
		if ( link < 0 )
		{
			lpFreeWheel( w, wi );
			for ( int k = 0; k < i; ++k )
			{
				lpDestroyLink( w, v->links[k] ); // frees its wheel too
			}
			w->vehicles.count -= 1;
			return -1;
		}
		wh->link = link;
		v->links[i] = link;
		lpPoseWheel( w, wh, xf, b->id );
	}
	return index;
}

void lpReleaseWheel( lpWorld* w, int wheel, bool comesOff )
{
	lpWheel* wh = w->wheels.data + wheel;
	w->vehicles.data[wh->vehicle].links[wh->slot] = -1;
	if ( comesOff )
	{
		b3Vec3 axle = b3RotateVector( wh->hub.q, (b3Vec3){ 1.0f, 0.0f, 0.0f } );
		lpLostWheel lost = { wh->hub, wh->hubVelocity, b3MulAdd( wh->bodyOmega, wh->spinSpeed, axle ),
							 wh->def.radius, wh->def.width, wh->def.material, wh->def.color };
		lpArray_Push( w->lostWheels, lost );
	}
	lpFreeWheel( w, wheel );
}

// A wheel that came off becomes a 12-sided cylinder with the hub's pose, velocity and spin
void lpSpawnLostWheels( lpWorld* w )
{
	for ( int i = 0; i < w->lostWheels.count; ++i )
	{
		lpLostWheel lost = w->lostWheels.data[i];
		b3Vec3 points[24];
		for ( int k = 0; k < 12; ++k )
		{
			b3CosSin cs = b3ComputeCosSin( 2.0f * LP_PI * (float)k / 12.0f );
			points[2 * k] = (b3Vec3){ -0.5f * lost.width, lost.radius * cs.cosine, lost.radius * cs.sine };
			points[2 * k + 1] = (b3Vec3){ 0.5f * lost.width, lost.radius * cs.cosine, lost.radius * cs.sine };
		}
		lpPartDef part = lpDefaultPartDef();
		part.points = points;
		part.pointCount = 24;
		part.material = lost.material;
		part.color = lost.color;
		part.grainAxis = (b3Vec3){ 1.0f, 0.0f, 0.0f };
		lpObjectDef def = lpDefaultObjectDef();
		def.transform = lost.hub;
		def.isStatic = false;
		def.parts = &part;
		def.partCount = 1;
		def.linearVelocity = lost.velocity;
		def.angularVelocity = lost.omega;
		lpCreateObject( w, &def );
	}
	w->lostWheels.count = 0;
}

typedef struct lpWheelCast
{
	const lpWorld* world;
	int chassis; // its own body: the cast passes through it
	float fraction;
	b3Pos point;
	b3Vec3 normal;
	int piece;
	float friction;
	bool hit;
} lpWheelCast;

static float lpWheelCastFcn( b3ShapeId shapeId, b3Pos point, b3Vec3 normal, float fraction, uint64_t userMaterialId,
							 int triangleIndex, int childIndex, void* context )
{
	(void)triangleIndex;
	(void)childIndex;
	lpWheelCast* cast = context;
	intptr_t data = (intptr_t)b3Shape_GetUserData( shapeId );
	int piece = data > 0 ? (int)( data - 1 ) : -1;
	if ( piece >= 0 && cast->world->pieces.data[piece].body == cast->chassis )
	{
		return -1.0f;
	}
	if ( fraction < cast->fraction )
	{
		cast->fraction = fraction;
		cast->point = point;
		cast->normal = normal;
		cast->piece = piece;
		cast->friction = userMaterialId < lp_materialCount ? lpGetMaterial( (int)userMaterialId )->friction : 0.6f;
		cast->hit = true;
	}
	return fraction;
}

// Casts the tyre from the mount (the hub at full compression) down to full droop
static void lpCastWheel( lpWorld* w, lpWheel* wh, int chassis, b3WorldTransform xf )
{
	const lpVehicle* v = w->vehicles.data + wh->vehicle;
	const lpLink* l = w->links.data + wh->link;
	b3CosSin steer = b3ComputeCosSin( wh->steer );
	b3Vec3 left = b3Cross( v->up, v->forward );
	b3Vec3 heading = b3RotateVector( xf.q, b3Sub( b3MulSV( steer.cosine, v->forward ), b3MulSV( steer.sine, left ) ) );
	b3Vec3 down = b3RotateVector( xf.q, b3Neg( v->up ) );

	// A disc in the tyre's plane: rim points (one straight down) wrapped in half the width
	static const float c[LP_WHEEL_RIM] = { 1.0f, 0.70710678f, 0.0f, -0.70710678f, -1.0f, -0.70710678f, 0.0f, 0.70710678f };
	static const float s[LP_WHEEL_RIM] = { 0.0f, 0.70710678f, 1.0f, 0.70710678f, 0.0f, -0.70710678f, -1.0f, -0.70710678f };
	float halfWidth = 0.5f * wh->def.width;
	float rim = wh->def.radius - halfWidth;
	b3Vec3 points[LP_WHEEL_RIM];
	for ( int k = 0; k < LP_WHEEL_RIM; ++k )
	{
		points[k] = b3Add( b3MulSV( rim * c[k], heading ), b3MulSV( rim * s[k], down ) );
	}
	b3ShapeProxy proxy = { points, LP_WHEEL_RIM, halfWidth };
	b3QueryFilter filter = b3DefaultQueryFilter();
	filter.categoryBits = LP_CAT_VEHICLE;
	filter.maskBits = LP_CAT_STATIC | LP_CAT_FULL;

	lpWheelCast cast = { w, chassis, FLT_MAX, { 0 }, b3Vec3_zero, -1, 0.6f, false };
	b3Pos mount = b3TransformWorldPoint( xf, l->ends[0].frame.p );
	b3World_CastShape( w->def.physics, mount, &proxy, b3MulSV( wh->def.maxLength, down ), filter, lpWheelCastFcn, &cast );
	w->stats.wheelCasts += 1;

	wh->grounded = cast.hit;
	if ( cast.hit == false )
	{
		wh->length = wh->def.maxLength;
		wh->groundPiece = -1;
		wh->atStop = false;
		return;
	}
	wh->length = cast.fraction * wh->def.maxLength;
	wh->atStop = cast.fraction <= 0.0f;
	wh->contactPoint = cast.point;
	// A cast that starts inside the ground reports no normal: push straight up the suspension
	wh->contactNormal = b3LengthSquared( cast.normal ) > 0.5f ? cast.normal : b3Neg( down );
	wh->friction = cast.friction;
	wh->groundPiece = cast.piece;
	wh->groundGeneration = cast.piece >= 0 ? w->pieces.data[cast.piece].generation : 0;
}

// The ground piece is still there, on a Box3D body
static bool lpGroundLive( const lpWorld* w, const lpWheel* wh )
{
	if ( wh->groundPiece < 0 )
	{
		return true;
	}
	const lpPiece* p = w->pieces.data + wh->groundPiece;
	return p->generation == wh->groundGeneration && p->body >= 0 && B3_IS_NON_NULL( p->shapeId );
}

static b3BodyId lpGroundBody( const lpWorld* w, const lpWheel* wh )
{
	if ( wh->groundPiece < 0 )
	{
		return b3_nullBodyId;
	}
	const lpBody* g = w->bodies.data + w->pieces.data[wh->groundPiece].body;
	return b3Body_GetType( g->id ) == b3_dynamicBody ? g->id : b3_nullBodyId;
}

static float lpEffectiveMass( float invMass, b3Matrix3 invI, b3Vec3 r, b3Vec3 d )
{
	b3Vec3 rd = b3Cross( r, d );
	float k = invMass + b3Dot( rd, b3MulMV( invI, rd ) );
	return k > 0.0f ? 1.0f / k : 0.0f;
}

static void lpApplyImpulse( b3Vec3* v, b3Vec3* omega, float invMass, b3Matrix3 invI, b3Vec3 r, b3Vec3 impulse )
{
	*v = b3MulAdd( *v, invMass, impulse );
	*omega = b3Add( *omega, b3MulMV( invI, b3Cross( r, impulse ) ) );
}

// A structure under a wheel carries its load: check it again when the load has changed by a quarter since, or moved
// to another structure (like a link's pull, at most every 30 steps)
static void lpRecheckGround( lpWorld* w, lpWheel* wh, b3Vec3 force )
{
	int ground = -1;
	if ( wh->grounded && wh->groundPiece >= 0 )
	{
		int body = w->pieces.data[wh->groundPiece].body;
		ground = w->bodies.data[body].kind == lp_kindStructure ? body : -1;
	}
	float change = b3Length( b3Sub( force, wh->stressForce ) );
	bool changed = ground != wh->stressBody ||
				   change > 0.25f * b3MaxFloat( b3Length( force ), b3Length( wh->stressForce ) ) + 10.0f;
	if ( changed == false || ( wh->recheckTick != 0 && w->tick + 1 < wh->recheckTick + LP_WHEEL_RECHECK_TICKS ) )
	{
		return;
	}
	int bodies[2] = { ground, wh->stressBody };
	for ( int k = 0; k < 2; ++k )
	{
		if ( bodies[k] >= 0 && ( k == 0 || bodies[1] != bodies[0] ) && w->bodies.data[bodies[k]].alive &&
			 w->bodies.data[bodies[k]].kind == lp_kindStructure )
		{
			lpRequestStressCheck( w, bodies[k], false );
			wh->recheckTick = w->tick + 1;
		}
	}
	wh->stressForce = ground >= 0 ? force : b3Vec3_zero;
	wh->stressBody = ground;
}

// The tyre solve of one chassis body's grounded wheels (list[0..count), in wheel order), then the forces
static void lpSolveTyres( lpWorld* w, int bodyIndex, const lpBodyWheel* list, int count, float timeStep )
{
	const lpBody* b = w->bodies.data + bodyIndex;
	b3BodyId id = b->id;
	float mass = b3Body_GetMass( id );
	if ( mass <= 0.0f )
	{
		return;
	}
	float invMass = 1.0f / mass;
	b3Matrix3 invI = b3Body_GetWorldInverseRotationalInertia( id );
	b3Pos com = b3Body_GetWorldCenter( id );
	b3WorldTransform xf = b3Body_GetTransform( id );
	b3Vec3 v0 = b3Body_GetLinearVelocity( id );
	b3Vec3 omega0 = b3Body_GetAngularVelocity( id );
	b3Vec3 up = b3Vec3_zero;

	// Velocity at the end of the step without the tyres: gravity and the springs
	b3Vec3 v = b3MulAdd( v0, timeStep * b->gravityScale, b3World_GetGravity( w->def.physics ) );
	b3Vec3 omega = omega0;
	for ( int k = 0; k < count; ++k )
	{
		lpWheel* wh = w->wheels.data + list[k].wheel;
		const lpVehicle* vehicle = w->vehicles.data + wh->vehicle;
		up = b3RotateVector( xf.q, vehicle->up );
		wh->lambdaN = 0.0f;
		wh->lambdaF = 0.0f;
		wh->lambdaS = 0.0f;
		wh->suspension = b3Vec3_zero;
		if ( wh->grounded == false )
		{
			continue;
		}
		b3Vec3 n = wh->contactNormal;
		wh->r = b3SubPos( wh->contactPoint, com );
		b3BodyId ground = lpGroundBody( w, wh );
		wh->groundVelocity = B3_IS_NON_NULL( ground ) ? b3Body_GetWorldPointVelocity( ground, wh->contactPoint ) : b3Vec3_zero;
		b3Vec3 vc = b3Sub( b3Add( v0, b3Cross( omega0, wh->r ) ), wh->groundVelocity );

		// The spring from the cast, damped by how fast the chassis closes on the ground (a kerb does not jolt it)
		float closing = -b3Dot( vc, up );
		float force = wh->def.stiffness * ( wh->def.restLength - wh->length ) + wh->def.damping * closing;
		wh->suspension = b3MulSV( b3MaxFloat( force, 0.0f ), n );
		lpApplyImpulse( &v, &omega, invMass, invI, wh->r, b3MulSV( timeStep, wh->suspension ) );

		// Along and across the tyre, in the ground's plane
		b3CosSin steer = b3ComputeCosSin( wh->steer );
		b3Vec3 left = b3Cross( vehicle->up, vehicle->forward );
		b3Vec3 heading = b3RotateVector( xf.q, b3Sub( b3MulSV( steer.cosine, vehicle->forward ), b3MulSV( steer.sine, left ) ) );
		b3Vec3 along = b3Sub( heading, b3MulSV( b3Dot( heading, n ), n ) );
		float length = b3Length( along );
		wh->dirF = length > 1e-4f ? b3MulSV( 1.0f / length, along ) : b3Vec3_zero;
		wh->dirS = b3Cross( n, wh->dirF );
		wh->massN = lpEffectiveMass( invMass, invI, wh->r, n );
		wh->massF = lpEffectiveMass( invMass, invI, wh->r, wh->dirF );
		wh->massS = lpEffectiveMass( invMass, invI, wh->r, wh->dirS );
		wh->slip = b3AbsFloat( b3Dot( vc, wh->dirS ) );
		bool locked = vehicle->control.handbrake && wh->def.handbrake;
		wh->spinSpeed = locked ? 0.0f : b3Dot( vc, wh->dirF ) / wh->def.radius;
	}

	for ( int iteration = 0; iteration < LP_WHEEL_ITERATIONS; ++iteration )
	{
		for ( int k = 0; k < count; ++k )
		{
			lpWheel* wh = w->wheels.data + list[k].wheel;
			if ( wh->grounded == false || wh->massF == 0.0f )
			{
				continue;
			}
			const lpVehicle* vehicle = w->vehicles.data + wh->vehicle;
			const lpVehicleControl* control = &vehicle->control;
			b3Vec3 n = wh->contactNormal;
			b3Vec3 vc = b3Sub( b3Add( v, b3Cross( omega, wh->r ) ), wh->groundVelocity );

			// Bottomed out: the ground may push, never pull
			if ( wh->atStop )
			{
				float lambda = b3MaxFloat( wh->lambdaN - wh->massN * b3Dot( vc, n ), 0.0f );
				lpApplyImpulse( &v, &omega, invMass, invI, wh->r, b3MulSV( lambda - wh->lambdaN, n ) );
				wh->lambdaN = lambda;
				vc = b3Sub( b3Add( v, b3Cross( omega, wh->r ) ), wh->groundVelocity );
			}
			float normal = b3Length( wh->suspension ) * timeStep + wh->lambdaN;
			float grip = wh->def.grip * wh->friction * ( wh->sliding ? LP_WHEEL_SLIDING_GRIP : 1.0f );
			float limit = grip * normal;
			bool locked = control->handbrake && wh->def.handbrake;

			// Across the tyre: no sliding sideways
			float lambdaS = wh->lambdaS - wh->massS * b3Dot( vc, wh->dirS );
			float limitS = locked ? LP_WHEEL_HANDBRAKE_GRIP * limit : limit;
			lambdaS = b3ClampFloat( lambdaS, -limitS, limitS );

			// Along it: locked, braked, driven toward top speed, or rolling
			float target = 0.0f;
			float cap;
			float brake = b3ClampFloat( control->brake, 0.0f, 1.0f ) * vehicle->def.maxBrakeForce * wh->def.brakeShare;
			float drive = b3AbsFloat( control->throttle ) * vehicle->def.maxDriveForce * wh->def.driveShare *
						  lpWheelSupply( w, wh, wh->def.driveNeeds );
			if ( locked )
			{
				cap = limit;
			}
			else if ( brake > 0.0f )
			{
				cap = brake * timeStep;
			}
			else if ( drive > 0.0f )
			{
				target = control->throttle > 0.0f ? vehicle->def.maxSpeed : -vehicle->def.maxSpeed;
				cap = drive * timeStep;
			}
			else
			{
				cap = vehicle->def.rollingResistance * normal;
			}
			float lambdaF = wh->lambdaF + wh->massF * ( target - b3Dot( vc, wh->dirF ) );
			lambdaF = b3ClampFloat( lambdaF, -cap, cap );

			// Within the friction circle
			float total = sqrtf( lambdaF * lambdaF + lambdaS * lambdaS );
			if ( total > limit )
			{
				float scale = limit / total;
				lambdaF *= scale;
				lambdaS *= scale;
			}
			b3Vec3 impulse = b3Add( b3MulSV( lambdaF - wh->lambdaF, wh->dirF ), b3MulSV( lambdaS - wh->lambdaS, wh->dirS ) );
			lpApplyImpulse( &v, &omega, invMass, invI, wh->r, impulse );
			wh->lambdaF = lambdaF;
			wh->lambdaS = lambdaS;
		}
	}

	// The forces, for Box3D to integrate over the step
	for ( int k = 0; k < count; ++k )
	{
		lpWheel* wh = w->wheels.data + list[k].wheel;
		lpLink* l = w->links.data + wh->link;
		if ( wh->grounded == false )
		{
			wh->load = 0.0f;
			wh->sliding = false;
			l->force = b3Vec3_zero;
			lpRecheckGround( w, wh, b3Vec3_zero );
			continue;
		}
		const lpVehicle* vehicle = w->vehicles.data + wh->vehicle;
		float grip = wh->def.grip * wh->friction * ( wh->sliding ? LP_WHEEL_SLIDING_GRIP : 1.0f );
		float normal = b3Length( wh->suspension ) * timeStep + wh->lambdaN;
		float total = sqrtf( wh->lambdaF * wh->lambdaF + wh->lambdaS * wh->lambdaS );
		wh->sliding = total >= 0.999f * grip * normal && normal > 0.0f;

		float invStep = 1.0f / timeStep;
		b3Vec3 push = b3MulAdd( wh->suspension, wh->lambdaN * invStep, wh->contactNormal );
		b3Vec3 along = b3MulSV( wh->lambdaF * invStep, wh->dirF );
		b3Vec3 across = b3MulSV( wh->lambdaS * invStep, wh->dirS );
		// The side force acts nearer the centre of mass, so it rolls the body less
		b3Vec3 raised = b3Sub( wh->r, b3MulSV( ( 1.0f - vehicle->def.rollFactor ) * b3Dot( wh->r, up ), up ) );
		b3Body_ApplyForce( id, b3Add( push, along ), wh->contactPoint, false );
		b3Body_ApplyForce( id, across, b3OffsetPos( com, raised ), false );
		b3Vec3 onChassis = b3Add( b3Add( push, along ), across );
		b3BodyId ground = lpGroundBody( w, wh );
		if ( B3_IS_NON_NULL( ground ) )
		{
			b3Body_ApplyForce( ground, b3Neg( onChassis ), wh->contactPoint, true );
		}
		wh->load = b3Dot( push, wh->contactNormal );
		l->force = b3Neg( onChassis ); // links keep the force on end B: the ground
		lpRecheckGround( w, wh, l->force );
	}
}

static int lpCompareBodyWheel( const void* a, const void* b )
{
	const lpBodyWheel* x = a;
	const lpBodyWheel* y = b;
	if ( x->body != y->body )
	{
		return ( x->body > y->body ) - ( x->body < y->body );
	}
	return ( x->wheel > y->wheel ) - ( x->wheel < y->wheel );
}

void lpStepVehicles( lpWorld* w, float timeStep )
{
	if ( w->vehicles.count == 0 )
	{
		return;
	}

	// Steering turns toward the control; a changed control wakes the chassis
	for ( int i = 0; i < w->vehicles.count; ++i )
	{
		lpVehicle* v = w->vehicles.data + i;
		bool wake = v->controlChanged;
		v->controlChanged = false;
		for ( int k = 0; k < v->wheelCount; ++k )
		{
			if ( v->links[k] < 0 )
			{
				continue;
			}
			lpWheel* wh = w->wheels.data + w->links.data[v->links[k]].wheel;
			float target = b3ClampFloat( v->control.steer, -1.0f, 1.0f ) * v->def.maxSteer * wh->def.steerFactor;
			float turn = v->def.steerSpeed * timeStep * lpWheelSupply( w, wh, wh->def.steerNeeds ); // unfed, it holds
			wh->steer += b3ClampFloat( target - wh->steer, -turn, turn );
			if ( wake )
			{
				b3Body_SetAwake( w->bodies.data[lpWheelMountBody( w, wh )].id, true );
			}
		}
	}

	// Cast the wheels of awake chassis bodies, within the step's count (starting where the last step left off, so over
	// budget every wheel still gets its turn; the others keep their last contact)
	w->scratchWheels.count = 0;
	int n = w->wheels.count;
	int start = (int)( w->tick % (uint64_t)n );
	int casts = 0;
	for ( int k = 0; k < n; ++k )
	{
		int i = ( start + k ) % n;
		lpWheel* wh = w->wheels.data + i;
		if ( wh->link < 0 )
		{
			continue;
		}
		int body = lpWheelMountBody( w, wh );
		b3BodyId id = w->bodies.data[body].id;
		if ( b3Body_GetType( id ) != b3_dynamicBody )
		{
			continue;
		}
		if ( b3Body_IsAwake( id ) == false )
		{
			// Parked: nothing moves, but the ground under it may go (it holds no Box3D contact to wake it)
			b3BodyId ground = lpGroundLive( w, wh ) ? lpGroundBody( w, wh ) : b3_nullBodyId;
			if ( lpGroundLive( w, wh ) == false || ( B3_IS_NON_NULL( ground ) && b3Body_IsAwake( ground ) ) )
			{
				b3Body_SetAwake( id, true );
			}
			else
			{
				continue;
			}
		}
		b3WorldTransform xf = b3Body_GetTransform( id );
		if ( casts < w->def.maxWheelCastsPerStep || lpGroundLive( w, wh ) == false )
		{
			lpCastWheel( w, wh, body, xf );
			casts += 1;
		}
		lpBodyWheel entry = { body, i };
		lpArray_Push( w->scratchWheels, entry );
	}
	if ( w->scratchWheels.count > 1 )
	{
		qsort( w->scratchWheels.data, (size_t)w->scratchWheels.count, sizeof( lpBodyWheel ), lpCompareBodyWheel );
	}

	for ( int first = 0; first < w->scratchWheels.count; )
	{
		int body = w->scratchWheels.data[first].body;
		int last = first + 1;
		while ( last < w->scratchWheels.count && w->scratchWheels.data[last].body == body )
		{
			last += 1;
		}
		lpSolveTyres( w, body, w->scratchWheels.data + first, last - first, timeStep );
		b3BodyId id = w->bodies.data[body].id;
		b3WorldTransform xf = b3Body_GetTransform( id );
		for ( int k = first; k < last; ++k )
		{
			lpWheel* wh = w->wheels.data + w->scratchWheels.data[k].wheel;
			if ( wh->grounded == false )
			{
				wh->spinSpeed *= 0.99f;
			}
			wh->spin += wh->spinSpeed * timeStep;
			while ( wh->spin > LP_PI )
			{
				wh->spin -= 2.0f * LP_PI;
			}
			while ( wh->spin < -LP_PI )
			{
				wh->spin += 2.0f * LP_PI;
			}
			lpPoseWheel( w, wh, xf, id );
		}
		first = last;
	}
}

void lpAddWheelLoads( lpWorld* w, int bodyIndex, b3WorldTransform xf )
{
	for ( int i = 0; i < w->wheels.count; ++i )
	{
		const lpWheel* wh = w->wheels.data + i;
		if ( wh->link < 0 || wh->grounded == false || wh->groundPiece < 0 || lpGroundLive( w, wh ) == false )
		{
			continue;
		}
		lpPiece* piece = w->pieces.data + wh->groundPiece;
		if ( piece->body != bodyIndex || piece->anchored )
		{
			continue;
		}
		b3Vec3 force = b3InvRotateVector( xf.q, w->links.data[wh->link].force );
		b3Vec3 point = b3InvTransformWorldPoint( xf, wh->contactPoint );
		piece->stressLoad.f = b3Add( piece->stressLoad.f, force );
		piece->stressLoad.t = b3Add( piece->stressLoad.t, b3Cross( b3Sub( point, piece->shape->centroid ), force ) );
	}
}

// ---- API ----

void lpWorld_SetVehicleControl( lpWorld* w, int vehicle, const lpVehicleControl* control )
{
	if ( vehicle < 0 || vehicle >= w->vehicles.count )
	{
		return;
	}
	lpVehicle* v = w->vehicles.data + vehicle;
	lpVehicleControl c = { b3ClampFloat( control->throttle, -1.0f, 1.0f ), b3ClampFloat( control->brake, 0.0f, 1.0f ),
						   b3ClampFloat( control->steer, -1.0f, 1.0f ), control->handbrake };
	if ( c.throttle != v->control.throttle || c.brake != v->control.brake || c.steer != v->control.steer ||
		 c.handbrake != v->control.handbrake )
	{
		v->control = c;
		v->controlChanged = true;
	}
}

lpVehicleState lpWorld_GetVehicleState( const lpWorld* w, int vehicle )
{
	lpVehicleState s = { 0 };
	s.body = -1;
	if ( vehicle < 0 || vehicle >= w->vehicles.count )
	{
		return s;
	}
	const lpVehicle* v = w->vehicles.data + vehicle;
	s.alive = v->alive;
	s.wheelCount = v->wheelCount;
	s.control = v->control;
	int bodies[LP_MAX_VEHICLE_WHEELS];
	float power = 0.0f;
	for ( int k = 0; k < v->wheelCount; ++k )
	{
		bodies[k] = -1;
		if ( v->links[k] < 0 )
		{
			continue;
		}
		const lpWheel* wh = w->wheels.data + w->links.data[v->links[k]].wheel;
		bodies[k] = lpWheelMountBody( w, wh );
		s.attached += 1;
		s.grounded += wh->grounded ? 1 : 0;
		s.driven += wh->def.driveShare > 0.0f ? 1 : 0;
		power += wh->def.driveShare > 0.0f ? lpWheelSupply( w, wh, wh->def.driveNeeds ) : 0.0f;
		s.steerable += wh->def.steerFactor != 0.0f ? 1 : 0;
	}
	s.power = s.driven > 0 ? power / (float)s.driven : 0.0f; // what its driven wheels still get, on average
	int best = 0;
	for ( int k = 0; k < v->wheelCount; ++k )
	{
		int held = 0;
		for ( int j = 0; j < v->wheelCount && bodies[k] >= 0; ++j )
		{
			held += bodies[j] == bodies[k] ? 1 : 0;
		}
		if ( held > best || ( held == best && held > 0 && bodies[k] < s.body ) )
		{
			best = held;
			s.body = bodies[k];
		}
	}
	if ( s.body >= 0 )
	{
		b3BodyId id = w->bodies.data[s.body].id;
		b3Quat q = b3Body_GetRotation( id );
		s.forward = b3RotateVector( q, v->forward );
		s.up = b3RotateVector( q, v->up );
		s.position = b3Body_GetWorldCenter( id );
		s.speed = b3Dot( b3Body_GetLinearVelocity( id ), s.forward );
	}
	return s;
}

int lpWorld_GetVehicleCapacity( const lpWorld* w )
{
	return w->vehicles.count;
}

int lpWorld_GetVehicleWheel( const lpWorld* w, int vehicle, int i )
{
	if ( vehicle < 0 || vehicle >= w->vehicles.count || i < 0 || i >= w->vehicles.data[vehicle].wheelCount )
	{
		return -1;
	}
	return w->vehicles.data[vehicle].links[i];
}

lpWheelState lpWorld_GetWheelState( const lpWorld* w, int link )
{
	lpWheelState s = { 0 };
	s.vehicle = -1;
	s.body = -1;
	s.groundPiece = -1;
	if ( link < 0 || link >= w->links.count || w->links.data[link].alive == false || w->links.data[link].wheel < 0 )
	{
		return s;
	}
	const lpWheel* wh = w->wheels.data + w->links.data[link].wheel;
	s.alive = true;
	s.grounded = wh->grounded;
	s.vehicle = wh->vehicle;
	s.body = lpWheelMountBody( w, wh );
	s.hub = wh->hub;
	s.radius = wh->def.radius;
	s.width = wh->def.width;
	s.length = wh->length;
	s.load = wh->load;
	s.slip = wh->slip;
	s.groundPiece = wh->grounded ? wh->groundPiece : -1;
	s.contactPoint = wh->contactPoint;
	return s;
}

// ---- hash, validation ----

uint64_t lpHashVehicles( const lpWorld* w, uint64_t h )
{
	for ( int i = 0; i < w->vehicles.count; ++i )
	{
		const lpVehicle* v = w->vehicles.data + i;
		h = lpHashBytes( h, &v->control, sizeof( v->control ) );
		h = lpHashBytes( h, v->links, sizeof( int ) * (size_t)v->wheelCount );
	}
	for ( int i = 0; i < w->wheels.count; ++i )
	{
		const lpWheel* wh = w->wheels.data + i;
		if ( wh->link < 0 )
		{
			continue;
		}
		float state[4] = { wh->steer, wh->length, wh->spinSpeed, wh->load };
		bool flags[3] = { wh->grounded, wh->atStop, wh->sliding };
		h = lpHashBytes( h, state, sizeof( state ) );
		h = lpHashBytes( h, flags, sizeof( flags ) );
		h = lpHashBytes( h, &wh->groundPiece, sizeof( int ) );
	}
	return h;
}

static bool lpWheelFail( const char* message, int a, int b )
{
	fprintf( stderr, "lpWorld_Validate: " );
	fprintf( stderr, message, a, b );
	fprintf( stderr, "\n" );
	return false;
}

bool lpValidateWheel( const lpWorld* w, int link )
{
	const lpLink* l = w->links.data + link;
	if ( l->def.type != lp_linkWheel || l->wheel >= w->wheels.count )
	{
		return lpWheelFail( "link %d is a wheel of the wrong type or slot %d", link, l->wheel );
	}
	const lpWheel* wh = w->wheels.data + l->wheel;
	if ( wh->link != link || wh->vehicle < 0 || wh->vehicle >= w->vehicles.count ||
		 w->vehicles.data[wh->vehicle].links[wh->slot] != link )
	{
		return lpWheelFail( "wheel link %d and wheel %d disagree", link, l->wheel );
	}
	if ( l->ends[0].piece < 0 || l->ends[1].piece >= 0 || B3_IS_NON_NULL( l->joint ) )
	{
		return lpWheelFail( "wheel link %d has a bad end or a joint (piece %d)", link, l->ends[0].piece );
	}
	return true;
}

void lpFreeVehicles( lpWorld* w )
{
	lpArray_Free( w->wheels );
	lpArray_Free( w->vehicles );
	lpArray_Free( w->lostWheels );
	lpArray_Free( w->scratchWheels );
}
