// SPDX-License-Identifier: MIT
// Rigs: the hexapod kit standing on its servos, kinematics, IK and capability.

#include "test_macros.h"
#include "test_sim.h"

#include <math.h>
#include <stdio.h>

static int TorsoOf( const Sim* s, int rig )
{
	return lpWorld_GetRigState( s->world, rig ).body;
}

typedef struct StandReport
{
	float sink;		   // how far the torso settled below where it was built, m
	float tilt;		   // worst tilt over the last half, radians
	float separation;  // worst joint separation after the first second (the landing), m
	float torqueShare; // peak motor torque over its cap after the first second
	int worstJoint;
	float utilization; // peak link utilization
	int worstLink;
	int asleepTick;	   // first tick the torso slept (-1: never)
	float speed;	   // of the torso at the end
	float awakeMs;	   // rig time per step while awake, and asleep
	float asleepMs;
	bool valid;
} StandReport;

// The kit dropped 5 cm onto flat ground and left standing for `ticks` steps
static StandReport Stand( int subSteps, int ticks, float drop )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (b3Vec3){ 0.0f, drop, 0.0f }, 0.0f, 0 );
	b3BodyId id = s.world->bodies.data[TorsoOf( &s, rig )].id;
	float built = (float)b3Body_GetWorldCenter( id ).y;
	StandReport r = { 0 };
	r.asleepTick = -1;
	r.worstJoint = -1;
	int awake = 0, asleep = 0;
	for ( int t = 0; t < ticks; ++t )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, subSteps );
		bool sleeping = b3Body_IsAwake( id ) == false;
		float ms = lpWorld_GetStats( s.world ).rigMs;
		r.awakeMs += sleeping ? 0.0f : ms;
		r.asleepMs += sleeping ? ms : 0.0f;
		awake += sleeping ? 0 : 1;
		asleep += sleeping ? 1 : 0;
		b3Vec3 up = b3RotateVector( b3Body_GetRotation( id ), (b3Vec3){ 0.0f, 1.0f, 0.0f } );
		if ( t >= ticks / 2 )
		{
			r.tilt = fmaxf( r.tilt, b3Atan2( sqrtf( up.x * up.x + up.z * up.z ), up.y ) );
		}
		for ( int i = 0; i < lpWorld_GetLinkCapacity( s.world ); ++i )
		{
			lpLinkState st = lpWorld_GetLinkState( s.world, i );
			if ( st.alive == false )
			{
				continue;
			}
			if ( t >= 60 )
			{
				r.separation = fmaxf( r.separation, b3Joint_GetLinearSeparation( s.world->links.data[i].joint ) );
			}
			float share = st.motorCap > 0.0f ? st.motorTorque / st.motorCap : 0.0f;
			if ( t >= 60 && share > r.torqueShare )
			{
				r.torqueShare = share;
				r.worstJoint = (int)( st.userId & 15 );
			}
			if ( st.utilization > r.utilization )
			{
				r.utilization = st.utilization;
				r.worstLink = (int)( st.userId & 15 );
			}
		}
		if ( r.asleepTick < 0 && sleeping )
		{
			r.asleepTick = t;
		}
	}
	r.awakeMs /= (float)( awake > 0 ? awake : 1 );
	r.asleepMs /= (float)( asleep > 0 ? asleep : 1 );
	r.speed = b3Length( b3Body_GetLinearVelocity( id ) );
	r.sink = built - (float)b3Body_GetWorldCenter( id ).y;
	r.valid = lpWorld_Validate( s.world );
	DestroySim( &s );
	return r;
}

// Box3D holds a 3.6 t mech on 18 servos at 4 substeps: built resting on the ground or dropped 5 cm, it settles within
// 3 cm, level, well within its servos and joints, and falls asleep
static int TestKitStands( void )
{
	int subSteps[3] = { 4, 4, 8 };
	float drops[3] = { 0.0f, 0.05f, 0.05f };
	for ( int k = 0; k < 3; ++k )
	{
		StandReport r = Stand( subSteps[k], 600, drops[k] );
		printf( "  %d substeps, dropped %.2f m: sag %.3f m, tilt %.2f deg, separation %.4f m, torque/cap %.2f (joint %d) after 1 s, "
				"utilization %.2f (joint %d), asleep at tick %d; rig %.4f ms awake, %.4f asleep\n",
				subSteps[k], drops[k], r.sink - drops[k], r.tilt * 57.29578f, r.separation, r.torqueShare, r.worstJoint, r.utilization,
				r.worstLink, r.asleepTick, r.awakeMs, r.asleepMs );
		ENSURE( r.valid );
		ENSURE( r.sink - drops[k] < 0.03f && r.tilt < 0.0175f && r.separation < 0.01f && r.speed < 0.001f );
		ENSURE( r.asleepTick >= 0 && r.asleepTick < 240 );
		ENSURE( r.torqueShare < 0.7f && r.utilization < 0.4f );
	}
	return 0;
}

// IK finds the angles back from where they put the foot, to a millimetre, for every leg, from a cold start
static int TestRigIKRoundTrip( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, b3Vec3_zero, 0.0f, 0 );
	const lpRig* r = s.world->rigs.data + rig;
	uint64_t state = 0x1234;
	float worst = 0.0f;
	for ( int trial = 0; trial < 60; ++trial )
	{
		const lpLimb* limb = r->limbs + trial % 6;
		float truth[LP_MAX_LIMB_JOINTS], q[LP_MAX_LIMB_JOINTS] = { 0.0f, 0.0f, 0.0f };
		for ( int k = 0; k < 3; ++k )
		{
			state = lpMix64( state + 1 );
			float u = (float)( state >> 40 ) / 16777216.0f;
			truth[k] = limb->lower[k] + 0.1f + u * ( limb->upper[k] - limb->lower[k] - 0.2f );
		}
		b3Vec3 axes[3], origins[3];
		b3Vec3 target = lpLimbForward( s.world, limb, 3, truth, limb->foot, axes, origins );
		float residual = lpLimbIK( s.world, limb, 3, limb->foot, target, q );
		// Warm-started from a nearby pose, as each step is
		float near[3] = { truth[0] + 0.05f, truth[1] - 0.05f, truth[2] + 0.05f };
		float warm = lpLimbIK( s.world, limb, 3, limb->foot, target, near );
		worst = fmaxf( worst, warm );
		if ( trial < 3 )
		{
			printf( "  trial %d: cold %.5f m, warm %.6f m\n", trial, residual, warm );
		}
	}
	printf( "  worst warm residual %.6f m over 60 poses\n", worst );
	ENSURE( worst < 0.001f );
	DestroySim( &s );
	return 0;
}

// The model (link frames and measured angles) puts each foot where its body actually has it
static int TestRigModelMatchesBodies( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (b3Vec3){ 0.0f, 0.05f, 0.0f }, 0.3f, 0 );
	Run( &s, 120 );
	const lpRig* r = s.world->rigs.data + rig;
	b3WorldTransform xf = lpGetTransform( s.world->bodies.data + r->body );
	float worst = 0.0f;
	for ( int i = 0; i < 6; ++i )
	{
		const lpLimb* limb = r->limbs + i;
		float q[3];
		for ( int k = 0; k < 3; ++k )
		{
			q[k] = s.world->links.data[limb->def.links[k]].angle;
		}
		b3Vec3 axes[3], origins[3];
		b3Pos model = b3TransformWorldPoint( xf, lpLimbForward( s.world, limb, 3, q, limb->foot, axes, origins ) );
		lpLimbState st = lpWorld_GetLimbState( s.world, rig, i );
		worst = fmaxf( worst, b3Length( b3SubPos( model, st.foot ) ) );
	}
	printf( "  worst model foot error standing %.4f m (the joints give under load)\n", worst );
	ENSURE( worst < 0.03f );
	DestroySim( &s );
	return 0;
}

// A leg that loses its tibia stands on the end of its femur: a peg
static int TestRigStumpFoot( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (b3Vec3){ 0.0f, 0.05f, 0.0f }, 0.0f, 0 );
	Run( &s, 60 );
	const lpLimb* limb = s.world->rigs.data[rig].limbs + 1;
	const lpLink* knee = s.world->links.data + limb->def.links[2];
	int femurEnd = limb->prox[2];
	int femurBody = s.world->pieces.data[knee->ends[femurEnd].piece].body;
	b3Pos kneePoint = b3TransformWorldPoint( lpGetTransform( s.world->bodies.data + femurBody ), knee->ends[femurEnd].frame.p );
	lpDestroyLink( s.world, limb->def.links[2] );
	Run( &s, 1 );
	lpLimbState st = lpWorld_GetLimbState( s.world, rig, 1 );
	float off = b3Length( b3SubPos( st.foot, kneePoint ) );
	printf( "  after the knee went: %d joints, foot %.3f m from the femur's end, reach %.2f m, able %d\n", st.joints, off, st.reach,
			st.able );
	ENSURE( st.joints == 2 && st.attached && off < 0.05f && st.footBody == femurBody );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// IK on the stump: what it can reach it reaches, what it cannot it comes as near as it can, inside its limits
static int TestRigStumpIK( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (b3Vec3){ 0.0f, 0.05f, 0.0f }, 0.0f, 0 );
	Run( &s, 30 );
	lpDestroyLink( s.world, s.world->rigs.data[rig].limbs[1].def.links[2] );
	Run( &s, 1 );
	const lpLimb* limb = s.world->rigs.data[rig].limbs + 1;
	b3Vec3 axes[3], origins[3];
	float truth[2] = { 0.2f, -0.3f };
	b3Vec3 target = lpLimbForward( s.world, limb, 2, truth, limb->foot, axes, origins );
	float q[2] = { 0.0f, 0.0f };
	float reached = lpLimbIK( s.world, limb, 2, limb->foot, target, q );
	// Far out along the leg: the best it can do is the femur stretched toward it, at its limit
	b3Vec3 far = b3MulAdd( origins[1], 10.0f, b3Normalize( b3Sub( target, origins[1] ) ) );
	float q2[2] = { 0.0f, 0.0f };
	float shortfall = lpLimbIK( s.world, limb, 2, limb->foot, far, q2 );
	printf( "  stump IK: reachable %.5f m short, far target %.2f m short, angles %.2f %.2f within [%.2f, %.2f]\n", reached,
			shortfall, q2[0], q2[1], limb->lower[1], limb->upper[1] );
	ENSURE( reached < 0.001f );
	ENSURE( shortfall < 10.0f && shortfall > 8.0f );
	ENSURE( q2[0] >= limb->lower[0] && q2[0] <= limb->upper[0] && q2[1] >= limb->lower[1] && q2[1] <= limb->upper[1] );
	DestroySim( &s );
	return 0;
}

// A crouch lowers the torso by its share of the stand height without moving the feet; then it settles and sleeps
static int TestRigCrouch( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, b3Vec3_zero, 0.0f, 0 );
	Run( &s, 120 );
	lpRigState before = lpWorld_GetRigState( s.world, rig );
	b3Pos feet[6];
	for ( int i = 0; i < 6; ++i )
	{
		feet[i] = lpWorld_GetLimbState( s.world, rig, i ).foot;
	}
	lpRigControl c = { 0.0f, 0.0f, 0.0f, 0.5f };
	lpWorld_SetRigControl( s.world, rig, &c );
	Run( &s, 180 );
	lpRigState after = lpWorld_GetRigState( s.world, rig );
	float moved = 0.0f;
	for ( int i = 0; i < 6; ++i )
	{
		moved = fmaxf( moved, b3Length( b3SubPos( lpWorld_GetLimbState( s.world, rig, i ).foot, feet[i] ) ) );
	}
	const lpRigDef* def = &s.world->rigs.data[rig].def;
	float want = 0.5f * def->crouchDepth * def->standHeight;
	float drop = before.height - after.height;
	printf( "  crouch: dropped %.3f m (wanted %.3f), feet moved %.3f m, idle %d\n", drop, want, moved, after.idle );
	ENSURE( fabsf( drop - want ) < 0.03f && moved < 0.02f );
	Run( &s, 240 );
	ENSURE( lpWorld_GetRigState( s.world, rig ).idle );
	ENSURE( b3Body_IsAwake( s.world->bodies.data[after.body].id ) == false );
	DestroySim( &s );
	return 0;
}

// The same controls at the same ticks give the same world, whatever the worker count
static int TestRigDeterminism( void )
{
	uint64_t hashes[3];
	int workers[3] = { 1, 4, 8 };
	for ( int k = 0; k < 3; ++k )
	{
		Sim s = CreateSimWorkers( -1, workers[k] );
		int rig = lpAddHexapod( s.world, (b3Vec3){ 0.0f, 0.05f, 0.0f }, 0.4f, 1 );
		for ( int t = 0; t < 240; ++t )
		{
			if ( t == 60 || t == 150 )
			{
				lpRigControl c = { 0.0f, 0.0f, 0.0f, t == 60 ? 0.8f : 0.2f };
				lpWorld_SetRigControl( s.world, rig, &c );
			}
			Run( &s, 1 );
		}
		hashes[k] = lpWorld_Hash( s.world );
		DestroySim( &s );
	}
	printf( "  hashes %016llx %016llx %016llx\n", (unsigned long long)hashes[0], (unsigned long long)hashes[1], (unsigned long long)hashes[2] );
	ENSURE( hashes[0] == hashes[1] && hashes[0] == hashes[2] );
	return 0;
}

// What a mech costs awake (bobbing up and down on its servos), the rig's step and Box3D's, against a car driving
static int TestRigCost( void )
{
	float rigMs = 0.0f, mechPhysics = 0.0f, carPhysics = 0.0f;
	int steps = 0;
	{
		Sim s = CreateSim( -1 );
		int rig = lpAddHexapod( s.world, b3Vec3_zero, 0.0f, 0 );
		for ( int t = 0; t < 420; ++t )
		{
			lpRigControl c = { 0.0f, 0.0f, 0.0f, ( t / 60 ) % 2 == 1 ? 0.6f : 0.0f };
			lpWorld_SetRigControl( s.world, rig, &c );
			Run( &s, 1 );
			if ( t >= 60 )
			{
				lpStats st = lpWorld_GetStats( s.world );
				rigMs += st.rigMs;
				mechPhysics += st.physicsMs;
				steps += 1;
			}
		}
		ENSURE( b3Body_IsAwake( s.world->bodies.data[lpWorld_GetRigState( s.world, rig ).body].id ) );
		DestroySim( &s );
	}
	{
		Sim s = CreateSim( -1 );
		int car = lpAddCar( s.world, b3Vec3_zero, 0.0f, 0 );
		lpVehicleControl c = { 0.3f, 0.0f, 0.4f, false };
		lpWorld_SetVehicleControl( s.world, car, &c );
		for ( int t = 0; t < 420; ++t )
		{
			Run( &s, 1 );
			carPhysics += t >= 60 ? lpWorld_GetStats( s.world ).physicsMs : 0.0f;
		}
		DestroySim( &s );
	}
	printf( "  awake mech: rig %.4f ms, Box3D %.3f ms per step; a car driving: Box3D %.3f ms\n", rigMs / (float)steps,
			mechPhysics / (float)steps, carPhysics / (float)steps );
	return 0;
}

int RigTest( void )
{
	RUN_TEST( TestKitStands );
	RUN_TEST( TestRigIKRoundTrip );
	RUN_TEST( TestRigModelMatchesBodies );
	RUN_TEST( TestRigStumpFoot );
	RUN_TEST( TestRigStumpIK );
	RUN_TEST( TestRigCrouch );
	RUN_TEST( TestRigDeterminism );
	RUN_TEST( TestRigCost );
	return 0;
}
