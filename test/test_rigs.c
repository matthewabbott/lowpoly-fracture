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
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, drop, 0.0f }, 0.0f, 0 );
	lpPhysBody id = s.world->bodies.data[TorsoOf( &s, rig )].id;
	float built = (float)lpPhys_GetWorldCenter( s.world->phys, id ).y;
	StandReport r = { 0 };
	r.asleepTick = -1;
	r.worstJoint = -1;
	int awake = 0, asleep = 0;
	for ( int t = 0; t < ticks; ++t )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, subSteps );
		bool sleeping = lpPhys_IsAwake( s.world->phys, id ) == false;
		float ms = lpWorld_GetStats( s.world ).rigMs;
		r.awakeMs += sleeping ? 0.0f : ms;
		r.asleepMs += sleeping ? ms : 0.0f;
		awake += sleeping ? 0 : 1;
		asleep += sleeping ? 1 : 0;
		lpVec3 up = lpRotateVector( lpPhys_GetTransform( s.world->phys, id ).q, (lpVec3){ 0.0f, 1.0f, 0.0f } );
		if ( t >= ticks / 2 )
		{
			r.tilt = fmaxf( r.tilt, lpAtan2( sqrtf( up.x * up.x + up.z * up.z ), up.y ) );
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
				r.separation = fmaxf( r.separation, lpPhys_GetJointSeparation( s.world->phys, s.world->links.data[i].joint ) );
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
	r.speed = lpLength( lpPhys_GetLinearVelocity( s.world->phys, id ) );
	r.sink = built - (float)lpPhys_GetWorldCenter( s.world->phys, id ).y;
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
	int rig = lpAddHexapod( s.world, lpVec3_zero, 0.0f, 0 );
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
		lpVec3 axes[3], origins[3];
		lpVec3 target = lpLimbForward( s.world, limb, 3, truth, limb->foot, axes, origins );
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
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.05f, 0.0f }, 0.3f, 0 );
	Run( &s, 120 );
	const lpRig* r = s.world->rigs.data + rig;
	lpWorldTransform xf = lpGetTransform( s.world, s.world->bodies.data + r->body );
	float worst = 0.0f;
	for ( int i = 0; i < 6; ++i )
	{
		const lpLimb* limb = r->limbs + i;
		float q[3];
		for ( int k = 0; k < 3; ++k )
		{
			q[k] = s.world->links.data[limb->def.links[k]].angle;
		}
		lpVec3 axes[3], origins[3];
		lpPos model = lpTransformWorldPoint( xf, lpLimbForward( s.world, limb, 3, q, limb->foot, axes, origins ) );
		lpLimbState st = lpWorld_GetLimbState( s.world, rig, i );
		worst = fmaxf( worst, lpLength( lpSubPos( model, st.foot ) ) );
	}
	printf( "  worst model foot error standing %.4f m (the joints give under load)\n", worst );
	ENSURE( worst < 0.03f );
	DestroySim( &s );
	return 0;
}

// A leg that loses its tibia has its foot at the end of its femur (too short to stand on, it is held up)
static int TestRigStumpFoot( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.05f, 0.0f }, 0.0f, 0 );
	Run( &s, 60 );
	const lpLimb* limb = s.world->rigs.data[rig].limbs + 1;
	const lpLink* knee = s.world->links.data + limb->def.links[2];
	int femurEnd = limb->prox[2];
	int femurBody = s.world->pieces.data[knee->ends[femurEnd].piece].body;
	lpVec3 kneeLocal = knee->ends[femurEnd].frame.p; // the femur's end, in its body's frame
	lpDestroyLink( s.world, limb->def.links[2] );
	Run( &s, 1 );
	lpPos kneePoint = lpTransformWorldPoint( lpGetTransform( s.world, s.world->bodies.data + femurBody ), kneeLocal );
	lpLimbState st = lpWorld_GetLimbState( s.world, rig, 1 );
	float off = lpLength( lpSubPos( st.foot, kneePoint ) );
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
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.05f, 0.0f }, 0.0f, 0 );
	Run( &s, 30 );
	lpDestroyLink( s.world, s.world->rigs.data[rig].limbs[1].def.links[2] );
	Run( &s, 1 );
	const lpLimb* limb = s.world->rigs.data[rig].limbs + 1;
	lpVec3 axes[3], origins[3];
	float truth[2] = { 0.2f, -0.3f };
	lpVec3 target = lpLimbForward( s.world, limb, 2, truth, limb->foot, axes, origins );
	float q[2] = { 0.0f, 0.0f };
	float reached = lpLimbIK( s.world, limb, 2, limb->foot, target, q );
	// Far out along the leg: the best it can do is the femur stretched toward it, at its limit
	lpVec3 far = lpMulAdd( origins[1], 10.0f, lpNormalize( lpSub( target, origins[1] ) ) );
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
	int rig = lpAddHexapod( s.world, lpVec3_zero, 0.0f, 0 );
	Run( &s, 120 );
	lpRigState before = lpWorld_GetRigState( s.world, rig );
	lpPos feet[6];
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
		moved = fmaxf( moved, lpLength( lpSubPos( lpWorld_GetLimbState( s.world, rig, i ).foot, feet[i] ) ) );
	}
	const lpRigDef* def = &s.world->rigs.data[rig].def;
	float want = 0.5f * def->gait.crouchDepth * def->standHeight;
	float drop = before.height - after.height;
	printf( "  crouch: dropped %.3f m (wanted %.3f), feet moved %.3f m, idle %d\n", drop, want, moved, after.idle );
	ENSURE( fabsf( drop - want ) < 0.03f && moved < 0.02f );
	Run( &s, 240 );
	ENSURE( lpWorld_GetRigState( s.world, rig ).idle );
	ENSURE( lpPhys_IsAwake( s.world->phys, s.world->bodies.data[after.body].id ) == false );
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
		int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.05f, 0.0f }, 0.4f, 1 );
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
		int rig = lpAddHexapod( s.world, lpVec3_zero, 0.0f, 0 );
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
		ENSURE( lpPhys_IsAwake( s.world->phys, s.world->bodies.data[lpWorld_GetRigState( s.world, rig ).body].id ) );
		DestroySim( &s );
	}
	{
		Sim s = CreateSim( -1 );
		int car = lpAddCar( s.world, lpVec3_zero, 0.0f, 0 );
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

// ---- walking ----

typedef struct WalkReport
{
	lpVec3 start, end; // the torso's frame
	float tiltRms;	   // radians, after the first second
	float worstTilt;
	float utilization; // peak over the rig's links, after the first second
	float slip;		   // the most a planted foot moved before it lifted, m (after the first second)
	float slipMean;	   // and on average
	int strides;	   // liftoffs
	int worstJoint;	   // of the peak utilization
	float yaw;		   // heading turned, radians (unwrapped, left positive)
	bool valid;
} WalkReport;

static float Heading( const Sim* s, int rig )
{
	lpRigState st = lpWorld_GetRigState( s->world, rig );
	return lpAtan2( st.forward.x, st.forward.z ); // turning left from +z (toward +x) is positive
}

static lpVec3 ToVec( lpPos p )
{
	return (lpVec3){ (float)p.x, (float)p.y, (float)p.z };
}

// Walks the rig with a control for `ticks` steps
static WalkReport Walk( Sim* s, int rig, lpRigControl control, int ticks )
{
	WalkReport r = { 0 };
	lpWorld_SetRigControl( s->world, rig, &control );
	lpRigState st = lpWorld_GetRigState( s->world, rig );
	r.start = ToVec( st.position );
	lpPos plantedAt[LP_MAX_RIG_LIMBS];
	float moved[LP_MAX_RIG_LIMBS] = { 0 };
	bool wasPlanted[LP_MAX_RIG_LIMBS] = { 0 };
	int slips = 0;
	for ( int i = 0; i < st.limbCount; ++i )
	{
		lpLimbState ls = lpWorld_GetLimbState( s->world, rig, i );
		wasPlanted[i] = ls.planted;
		plantedAt[i] = ls.foot;
	}
	float heading = Heading( s, rig );
	double tiltSum = 0.0;
	int tiltCount = 0;
	for ( int t = 0; t < ticks; ++t )
	{
		Run( s, 1 );
		st = lpWorld_GetRigState( s->world, rig );
		float h = Heading( s, rig );
		float d = h - heading;
		d = d > 3.14159265f ? d - 6.2831853f : ( d < -3.14159265f ? d + 6.2831853f : d );
		r.yaw += d;
		heading = h;
		float tilt = lpAtan2( sqrtf( st.up.x * st.up.x + st.up.z * st.up.z ), st.up.y );
		if ( t >= 60 )
		{
			tiltSum += (double)( tilt * tilt );
			tiltCount += 1;
			r.worstTilt = fmaxf( r.worstTilt, tilt );
		}
		for ( int i = 0; i < st.limbCount; ++i )
		{
			// Slip: how far a planted foot moves once it has settled (a foot that lands late finishes its landing first)
			lpLimbState ls = lpWorld_GetLimbState( s->world, rig, i );
			const lpLimb* inner = s->world->rigs.data[rig].limbs + i;
			bool settled = ls.planted && inner->holdClock >= 0.3f;
			if ( settled && wasPlanted[i] == false )
			{
				plantedAt[i] = ls.foot;
				moved[i] = 0.0f;
			}
			else if ( settled )
			{
				lpVec3 d3 = lpSubPos( ls.foot, plantedAt[i] );
				moved[i] = fmaxf( moved[i], sqrtf( d3.x * d3.x + d3.z * d3.z ) );
			}
			else if ( wasPlanted[i] )
			{
				r.strides += 1;
				r.slip = t >= 60 ? fmaxf( r.slip, moved[i] ) : r.slip;
				r.slipMean += t >= 60 ? moved[i] : 0.0f;
				slips += t >= 60 ? 1 : 0;
			}
			ls.planted = settled;
			wasPlanted[i] = ls.planted;
			const lpLimb* limb = s->world->rigs.data[rig].limbs + i;
			for ( int k = 0; k < limb->def.linkCount && t >= 60; ++k )
			{
				lpLinkState link = lpWorld_GetLinkState( s->world, limb->def.links[k] );
				if ( link.alive && link.utilization > r.utilization )
				{
					r.utilization = link.utilization;
					r.worstJoint = k;
				}
			}
		}
	}
	r.end = ToVec( st.position );
	r.tiltRms = tiltCount > 0 ? (float)sqrt( tiltSum / tiltCount ) : 0.0f;
	r.slipMean = slips > 0 ? r.slipMean / (float)slips : 0.0f;
	r.valid = lpWorld_Validate( s->world );
	return r;
}

// Forward at full speed on flat ground: it keeps most of its top speed, straight and level, its feet gripping
static int TestRigWalksStraight( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.0f, -18.0f }, 0.0f, 0 );
	Run( &s, 30 );
	lpRigControl go = { 1.0f, 0.0f, 0.0f, 0.0f };
	Walk( &s, rig, go, 120 ); // up to speed
	WalkReport r = Walk( &s, rig, go, 600 );
	float distance = r.end.z - r.start.z;
	float speed = distance / 10.0f;
	float drift = fabsf( r.end.x - r.start.x );
	float top = s.world->rigs.data[rig].def.gait.maxSpeed;
	printf( "  %.2f m/s of %.2f, drift %.2f m over %.1f m, tilt rms %.2f deg (worst %.2f), slip %.3f m (mean %.3f), %d strides, "
			"utilization %.2f (joint %d)\n",
			speed, top, drift, distance, r.tiltRms * 57.29578f, r.worstTilt * 57.29578f, r.slip, r.slipMean, r.strides, r.utilization,
			r.worstJoint );
	ENSURE( r.valid );
	ENSURE( speed >= 0.85f * top && drift < 0.5f );
	// A toe drags a little as its load goes (the worst of 84 strides a little more)
	ENSURE( r.tiltRms < 0.035f && r.slipMean < 0.05f && r.slip < 0.12f && r.utilization < 0.5f );
	ENSURE( lpWorld_GetRigState( s.world, rig ).able == 6 );
	DestroySim( &s );
	return 0;
}

// Turning in place at full rate
static int TestRigTurns( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, lpVec3_zero, 0.0f, 0 );
	Run( &s, 30 );
	lpRigControl turn = { 0.0f, 0.0f, -1.0f, 0.0f }; // left
	Walk( &s, rig, turn, 60 );
	WalkReport r = Walk( &s, rig, turn, 240 );
	float rate = r.yaw / 4.0f;
	float moved = sqrtf( ( r.end.x - r.start.x ) * ( r.end.x - r.start.x ) + ( r.end.z - r.start.z ) * ( r.end.z - r.start.z ) );
	printf( "  %.2f rad/s of %.2f, moved %.2f m, tilt rms %.2f deg, slip %.3f m, %d strides\n", rate,
			s.world->rigs.data[rig].def.gait.maxTurn, moved, r.tiltRms * 57.29578f, r.slip, r.strides );
	ENSURE( r.valid && rate >= 0.5f && moved < 1.0f );
	DestroySim( &s );
	return 0;
}

// A static concrete ramp rising along +z from z0 to z1 by `rise`, then a flat top to z2
static void AddRamp( Sim* s, float z0, float z1, float z2, float rise )
{
	lpVec3 points[8] = { { -5.0f, 0.0f, z0 }, { 5.0f, 0.0f, z0 }, { -5.0f, 0.0f, z2 }, { 5.0f, 0.0f, z2 },
						 { -5.0f, rise, z1 }, { 5.0f, rise, z1 }, { -5.0f, rise, z2 }, { 5.0f, rise, z2 } };
	lpPartDef part = lpDefaultPartDef();
	part.points = points;
	part.pointCount = 8;
	part.material = lp_concrete;
	part.anchored = true;
	lpObjectDef def = lpDefaultObjectDef();
	def.parts = &part;
	def.partCount = 1;
	lpCreateObject( s->world, &def );
	lpWorld_SettleStructures( s->world );
}

// Up a 15 degree slope at most of its flat speed
static int TestRigClimbsSlope( void )
{
	Sim s = CreateSim( -1 );
	AddRamp( &s, 0.0f, 12.0f, 20.0f, 12.0f * 0.2679f );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.0f, -10.0f }, 0.0f, 0 );
	Run( &s, 30 );
	lpRigControl go = { 1.0f, 0.0f, 0.0f, 0.0f };
	float flatRun = 0.0f, slopeRun = 0.0f, worstTilt = 0.0f;
	int flatTicks = 0, slopeTicks = 0;
	for ( int t = 0; t < 900; ++t )
	{
		lpRigState before = lpWorld_GetRigState( s.world, rig );
		Walk( &s, rig, go, 1 );
		lpRigState st = lpWorld_GetRigState( s.world, rig );
		float z = (float)st.position.z, dz = (float)( st.position.z - before.position.z );
		if ( t > 90 && z > -7.0f && z < -3.0f )
		{
			flatRun += dz;
			flatTicks += 1;
		}
		if ( z > 3.0f && z < 9.0f )
		{
			slopeRun += dz;
			slopeTicks += 1;
			worstTilt = fmaxf( worstTilt, lpAtan2( sqrtf( st.up.x * st.up.x + st.up.z * st.up.z ), st.up.y ) );
		}
	}
	float flat = flatTicks > 0 ? 60.0f * flatRun / (float)flatTicks : 0.0f;
	float slope = slopeTicks > 0 ? 60.0f * slopeRun / (float)slopeTicks : 0.0f;
	lpRigState st = lpWorld_GetRigState( s.world, rig );
	printf( "  flat %.2f m/s, up the slope %.2f m/s (%.0f%%), worst tilt on it %.1f deg, ended at z %.1f\n", flat, slope,
			100.0f * slope / fmaxf( flat, 1e-3f ), worstTilt * 57.29578f, (float)st.position.z );
	ENSURE( slopeTicks > 0 && slope >= 0.6f * flat && st.able == 6 );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// Up onto a 0.4 m step and down off it again
static int TestRigStepsUpAndDown( void )
{
	Sim s = CreateSim( -1 );
	lpPartDef part = lpDefaultPartDef();
	part.halfExtents = (lpVec3){ 5.0f, 0.2f, 4.0f };
	part.transform.p = (lpVec3){ 0.0f, 0.2f, 4.0f };
	part.material = lp_concrete;
	part.anchored = true;
	lpObjectDef def = lpDefaultObjectDef();
	def.parts = &part;
	def.partCount = 1;
	lpCreateObject( s.world, &def );
	lpWorld_SettleStructures( s.world );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.0f, -6.0f }, 0.0f, 0 );
	Run( &s, 30 );
	lpRigControl go = { 0.6f, 0.0f, 0.0f, 0.0f };
	float topHeight = 0.0f, worstTilt = 0.0f;
	for ( int t = 0; t < 1200; ++t )
	{
		Walk( &s, rig, go, 1 );
		lpRigState st = lpWorld_GetRigState( s.world, rig );
		if ( st.position.z > 3.0f && st.position.z < 5.0f )
		{
			topHeight = fmaxf( topHeight, (float)st.position.y );
		}
		worstTilt = fmaxf( worstTilt, lpAtan2( sqrtf( st.up.x * st.up.x + st.up.z * st.up.z ), st.up.y ) );
	}
	lpRigState st = lpWorld_GetRigState( s.world, rig );
	float stand = s.world->rigs.data[rig].def.standHeight;
	printf( "  on top the torso stood %.2f m up (%.2f over the step), worst tilt %.1f deg, ended at z %.1f\n", topHeight,
			topHeight - 0.4f, worstTilt * 57.29578f, (float)st.position.z );
	ENSURE( topHeight - 0.4f > stand - 0.1f && st.position.z > 10.0f && worstTilt < 0.2f && st.able == 6 );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// Stopped from full speed it pulls up within 1.5 m, tidies its feet and falls asleep
static int TestRigStops( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.0f, -10.0f }, 0.0f, 0 );
	Run( &s, 30 );
	lpRigControl go = { 1.0f, 0.0f, 0.0f, 0.0f }, stop = { 0 };
	Walk( &s, rig, go, 240 );
	WalkReport r = Walk( &s, rig, stop, 60 );
	int asleep = -1;
	for ( int t = 0; t < 300 && asleep < 0; ++t )
	{
		Run( &s, 1 );
		lpPhysBody torso = s.world->bodies.data[lpWorld_GetRigState( s.world, rig ).body].id;
		asleep = lpPhys_IsAwake( s.world->phys, torso ) ? -1 : t + 60;
	}
	lpRigState st = lpWorld_GetRigState( s.world, rig );
	float ran = (float)st.position.z - r.start.z;
	printf( "  ran on %.2f m after the stop, asleep %d steps after it, idle %d\n", ran, asleep, st.idle );
	ENSURE( ran < 1.5f && asleep >= 0 && asleep < 240 );
	DestroySim( &s );
	return 0;
}

// Walking and turning: the same world at 1, 4 and 8 workers
static int TestRigWalkDeterminism( void )
{
	uint64_t hashes[3];
	int workers[3] = { 1, 4, 8 };
	for ( int k = 0; k < 3; ++k )
	{
		Sim s = CreateSimWorkers( -1, workers[k] );
		int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.0f, -5.0f }, 0.3f, 2 );
		lpRigControl a = { 1.0f, 0.0f, 0.3f, 0.0f }, b = { 0.4f, 0.5f, -0.6f, 0.3f };
		Walk( &s, rig, a, 150 );
		Walk( &s, rig, b, 150 );
		hashes[k] = lpWorld_Hash( s.world );
		DestroySim( &s );
	}
	printf( "  hashes %016llx %016llx %016llx\n", (unsigned long long)hashes[0], (unsigned long long)hashes[1],
			(unsigned long long)hashes[2] );
	ENSURE( hashes[0] == hashes[1] && hashes[0] == hashes[2] );
	return 0;
}

// What walking costs: the rig's step, foothold casts, and Box3D with 1, 4 and 16 walkers
static int TestRigWalkCost( void )
{
	int counts[3] = { 1, 4, 16 };
	for ( int k = 0; k < 3; ++k )
	{
		Sim s = CreateSim( -1 );
		int rigs[16];
		for ( int n = 0; n < counts[k]; ++n )
		{
			rigs[n] = lpAddHexapod( s.world, (lpVec3){ -12.0f + 8.0f * (float)( n % 4 ), 0.0f, -20.0f + 7.0f * (float)( n / 4 ) }, 0.0f, n );
		}
		lpRigControl go = { 0.8f, 0.0f, 0.2f, 0.0f };
		for ( int n = 0; n < counts[k]; ++n )
		{
			lpWorld_SetRigControl( s.world, rigs[n], &go );
		}
		float rigMs = 0.0f, physicsMs = 0.0f;
		int casts = 0;
		for ( int t = 0; t < 300; ++t )
		{
			Run( &s, 1 );
			lpStats st = lpWorld_GetStats( s.world );
			if ( t >= 60 )
			{
				rigMs += st.rigMs;
				physicsMs += st.physicsMs;
				casts += st.footCasts;
			}
		}
		printf( "  %2d walkers: rig %.4f ms (%.1f us each), Box3D %.3f ms, %.2f foot casts per step\n", counts[k], rigMs / 240.0f,
				1000.0f * rigMs / 240.0f / (float)counts[k], physicsMs / 240.0f, (float)casts / 240.0f );
		DestroySim( &s );
	}
	return 0;
}

// The model puts a bent leg's foot where the bodies have it (floating, so nothing loads the joints)
static int TestRigModelBent( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 5.0f, 0.0f }, 0.4f, 0 );
	for ( int b = 0; b < s.world->bodies.count; ++b )
	{
		if ( s.world->bodies.data[b].alive && LP_PHYS_NULL( s.world->bodies.data[b].id ) == false &&
			 lpPhys_IsDynamic( s.world->phys, s.world->bodies.data[b].id ) )
		{
			lpWorld_SetGravityScale( s.world, b, 0.0f );
		}
	}
	s.world->rigs.data[rig].alive = false; // hold the targets set here
	float angles[3] = { 0.35f, -0.4f, 0.5f };
	for ( int i = 0; i < 6; ++i )
	{
		for ( int k = 0; k < 3; ++k )
		{
			lpWorld_SetLinkTarget( s.world, s.world->rigs.data[rig].limbs[i].def.links[k], angles[k] * ( i % 2 == 0 ? 1.0f : -0.7f ) );
		}
	}
	Run( &s, 120 );
	const lpRig* r = s.world->rigs.data + rig;
	lpWorldTransform xf = lpGetTransform( s.world, s.world->bodies.data + r->body );
	float worst = 0.0f;
	for ( int i = 0; i < 6; ++i )
	{
		const lpLimb* limb = r->limbs + i;
		float q[3];
		for ( int k = 0; k < 3; ++k )
		{
			q[k] = s.world->links.data[limb->def.links[k]].angle;
		}
		lpVec3 axes[3], origins[3];
		lpPos model = lpTransformWorldPoint( xf, lpLimbForward( s.world, limb, 3, q, limb->foot, axes, origins ) );
		lpPos body = lpFootWorld( s.world, limb );
		float off = lpLength( lpSubPos( model, body ) );
		printf( "  leg %d angles %.2f %.2f %.2f: model foot %.3f m from the body's\n", i, q[0], q[1], q[2], off );
		worst = fmaxf( worst, off );
	}
	ENSURE( worst < 0.02f );
	DestroySim( &s );
	return 0;
}

// Diagnostic: one leg through a few strides
// The mech yard's patrol: round the loop over the step, the rubble and the hump, past the crates, the wall and the car,
// and back, in order, with every leg on
static int TestRigPatrols( void )
{
	Sim s = CreateSim( lp_sceneMech );
	static const float corners[4][2] = { { 0.0f, 20.0f }, { 16.0f, 20.0f }, { 16.0f, -20.0f }, { 0.0f, -20.0f } };
	int reached = 0, lapTick = -1;
	for ( int t = 0; t < 4800 && reached < 4; ++t )
	{
		lpSceneDrive( s.world, lp_sceneMech, t, -1, -1 );
		Run( &s, 1 );
		lpRigState st = lpWorld_GetRigState( s.world, 0 );
		float dx = (float)st.position.x - corners[reached][0], dz = (float)st.position.z - corners[reached][1];
		if ( dx * dx + dz * dz < 16.0f )
		{
			reached += 1;
			lapTick = t;
		}
		if ( t % 300 == 0 )
		{
			ENSURE( lpWorld_Validate( s.world ) );
		}
	}
	lpRigState st = lpWorld_GetRigState( s.world, 0 );
	printf( "  round the yard (about 110 m) in %.1f s, %d legs able, torso %.2f m over its feet\n", (float)lapTick / 60.0f, st.able, st.height );
	ENSURE( reached == 4 && st.able == 6 );
	DestroySim( &s );
	return 0;
}

// ---- damage ----

// A leg off at the hip: its first link goes, and the leg with it
static void LoseLeg( Sim* s, int rig, int limb )
{
	lpDestroyLink( s->world, s->world->rigs.data[rig].limbs[limb].def.links[0] );
}

typedef struct Hobble
{
	float speed; // m/s along where it faced when it set off
	float drift; // m across it
	float tiltRms, worstTilt;
	float height;
	int able;
	bool crawling;
	bool valid;
	float pegDepth; // how far below the torso leg 1 reaches at the end
	float fluid;	// what is left in its hydraulic reservoir, 0 to 1
} Hobble;

// The mech as built, damaged by `harm`, given `wait` steps to find its feet, then walked forward for `ticks`
static Hobble WalkDamagedAfter( void ( *harm )( Sim*, int ), int wait, int ticks )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.0f, -15.0f }, 0.0f, 0 );
	Run( &s, 30 );
	harm( &s, rig );
	Run( &s, wait );
	lpRigState st = lpWorld_GetRigState( s.world, rig );
	lpVec3 heading = lpNormalize( (lpVec3){ st.forward.x, 0.0f, st.forward.z } );
	lpRigControl go = { 1.0f, 0.0f, 0.0f, 0.0f };
	Walk( &s, rig, go, 60 );
	WalkReport w = Walk( &s, rig, go, ticks );
	lpVec3 moved = lpSub( w.end, w.start );
	Hobble h;
	h.speed = lpDot( moved, heading ) * 60.0f / (float)ticks;
	h.drift = lpAbsFloat( lpDot( moved, lpCross( (lpVec3){ 0.0f, 1.0f, 0.0f }, heading ) ) );
	h.tiltRms = w.tiltRms;
	h.worstTilt = w.worstTilt;
	st = lpWorld_GetRigState( s.world, rig );
	h.height = st.height;
	h.able = st.able;
	h.crawling = st.crawling;
	h.valid = w.valid;
	h.pegDepth = lpWorld_GetLimbState( s.world, rig, 1 ).depth;
	h.fluid = -1.0f;
	for ( int i = 0; i < lpWorld_GetPieceCapacity( s.world ) && h.fluid < 0.0f; ++i )
	{
		lpPieceInfo info = lpWorld_GetPieceInfo( s.world, i );
		h.fluid = info.body >= 0 && info.tag == lp_tagReservoir ? lpWorld_GetPiecePool( s.world, i, NULL ) : -1.0f;
	}
	DestroySim( &s );
	return h;
}

static Hobble WalkDamaged( void ( *harm )( Sim*, int ), int ticks )
{
	return WalkDamagedAfter( harm, 120, ticks );
}

static void HarmNone( Sim* s, int rig )
{
	(void)s;
	(void)rig;
}

static void HarmOneLeg( Sim* s, int rig )
{
	LoseLeg( s, rig, 1 ); // the right middle
}

static void HarmOpposite( Sim* s, int rig )
{
	LoseLeg( s, rig, 1 );
	LoseLeg( s, rig, 4 ); // both middles
}

static void HarmSameSide( Sim* s, int rig )
{
	LoseLeg( s, rig, 0 );
	LoseLeg( s, rig, 2 ); // right front and rear
}

static void HarmThreeLegs( Sim* s, int rig )
{
	LoseLeg( s, rig, 1 );
	LoseLeg( s, rig, 3 );
	LoseLeg( s, rig, 5 );
}

// A heavy shot through the lower tibia: what is left of it below the knee is a peg
static void HarmPeg( Sim* s, int rig )
{
	const lpLimb* limb = s->world->rigs.data[rig].limbs + 1;
	const lpLink* knee = s->world->links.data + limb->def.links[2];
	lpVec3 top = knee->ends[1 - limb->prox[2]].frame.p;
	lpVec3 at = lpLerp( top, limb->foot, 0.6f );
	lpImpactDef impact = { 0 };
	impact.point = lpTransformWorldPoint( lpGetTransform( s->world, s->world->bodies.data + limb->tipBody ), at );
	impact.direction = (lpVec3){ -1.0f, 0.0f, 0.0f };
	impact.radius = 0.25f;
	impact.energy = 40000.0f;
	lpWorld_AddImpact( s->world, &impact );
}

static void HarmStump( Sim* s, int rig )
{
	lpDestroyLink( s->world, s->world->rigs.data[rig].limbs[1].def.links[2] ); // the tibia falls: a femur alone is too short
}

static void HarmWeakLeg( Sim* s, int rig )
{
	for ( int k = 0; k < 3; ++k )
	{
		lpLink* l = s->world->links.data + s->world->rigs.data[rig].limbs[1].def.links[k];
		l->health = 0.4f * l->def.strength; // its servos hold 40% of their torque
	}
}

static void HarmLimpLeg( Sim* s, int rig )
{
	// The hip no longer carries the lines: what lies beyond it goes unfed, and its servos limp
	lpLink* hip = s->world->links.data + s->world->rigs.data[rig].limbs[1].def.links[0];
	hip->def.carries = 0;
	lpCarriersChanged( s->world, 0xFF );
}

static void PrintHobble( const char* what, Hobble h, float intact )
{
	printf( "  %s: %.2f m/s (%.0f%%), drift %.2f m, tilt rms %.1f deg (worst %.1f), height %.2f m, %d able%s\n", what, h.speed,
			100.0f * h.speed / intact, h.drift, 57.29578f * h.tiltRms, 57.29578f * h.worstTilt, h.height, h.able, h.crawling ? ", crawling" : "" );
}

// It keeps walking on five legs, four (two opposite; two off one side leave one leg there, and no foot lifts without
// tipping it over, so it crawls), and drags itself on three
static int TestRigLosesLegs( void )
{
	Hobble intact = WalkDamaged( HarmNone, 600 );
	Hobble one = WalkDamaged( HarmOneLeg, 600 );
	Hobble opposite = WalkDamaged( HarmOpposite, 600 );
	Hobble side = WalkDamaged( HarmSameSide, 600 );
	Hobble three = WalkDamaged( HarmThreeLegs, 600 );
	PrintHobble( "intact", intact, intact.speed );
	PrintHobble( "one leg off", one, intact.speed );
	PrintHobble( "both middles off", opposite, intact.speed );
	PrintHobble( "right front and rear off", side, intact.speed );
	PrintHobble( "three off", three, intact.speed );
	ENSURE( intact.valid && one.valid && opposite.valid && side.valid && three.valid );
	// Fewer legs swing in more turns (on five, half its pace; on four, a third): five legs make 70% of what they are asked
	ENSURE( one.able == 5 && one.speed >= 0.35f * intact.speed && one.drift < 1.5f );
	ENSURE( opposite.able == 4 && opposite.speed >= 0.2f );
	ENSURE( side.able == 4 && side.speed >= 0.1f );
	ENSURE( three.able == 3 && three.crawling && three.speed >= 0.1f && three.worstTilt < 0.52f );
	return 0;
}

// A leg shot through below the knee walks on what is left of its tibia, a peg (it reaches 1.2 m instead of 1.9, still
// enough to stand level at full height). One that lost its whole tibia holds the femur up (too short to reach the ground)
// and the rest walk on
static int TestRigWalksOnAPeg( void )
{
	Hobble intact = WalkDamaged( HarmNone, 300 );
	Hobble peg = WalkDamaged( HarmPeg, 600 );
	Hobble stump = WalkDamaged( HarmStump, 600 );
	PrintHobble( "on a peg", peg, intact.speed );
	PrintHobble( "a femur held up", stump, intact.speed );
	ENSURE( peg.valid && peg.able == 6 && peg.tiltRms < 0.087f && peg.speed >= 0.5f * intact.speed );
	ENSURE( peg.pegDepth < 1.5f );
	ENSURE( stump.valid && stump.able == 5 && stump.tiltRms < 0.087f );
	return 0;
}

static void HarmBleed( Sim* s, int rig )
{
	s->world->pools.data[0].seal = 0.0f; // its valves stuck open: the reservoir bleeds until it is dry
	LoseLeg( s, rig, 1 );
}

// A leg shot off opens a leak in its hydraulics: the valves close it in a few seconds, and it walks on having lost part of
// its fluid. With its valves stuck open it bleeds dry: its servos weaken below 30%, the body sinks, and it slows to a
// stop
static int TestRigBleedsOut( void )
{
	Hobble intact = WalkDamaged( HarmNone, 300 );
	Hobble valved = WalkDamagedAfter( HarmOneLeg, 900, 300 ); // 15 s after the shot, walked for 5
	Hobble bleeding = WalkDamagedAfter( HarmBleed, 900, 300 );
	PrintHobble( "a leg off, valves shut", valved, intact.speed );
	PrintHobble( "a leg off, bleeding", bleeding, intact.speed );
	printf( "  fluid left: %.2f with its valves, %.2f bleeding\n", valved.fluid, bleeding.fluid );
	ENSURE( valved.valid && bleeding.valid );
	// On five legs it makes 70% of the half pace five legs are asked (as in TestRigLosesLegs)
	ENSURE( intact.fluid == 1.0f && valved.fluid > 0.6f && valved.fluid < 1.0f && valved.speed >= 0.35f * intact.speed );
	ENSURE( bleeding.fluid < 0.05f && bleeding.height < intact.height - 0.3f && bleeding.speed < 0.5f * valved.speed );
	return 0;
}

// A leg with 40% of its servos' torque: the body walks lower; a leg whose lines are cut goes limp and the others walk on
static int TestRigWeakAndLimpLegs( void )
{
	Hobble intact = WalkDamaged( HarmNone, 300 );
	Hobble weak = WalkDamaged( HarmWeakLeg, 600 );
	Hobble limp = WalkDamaged( HarmLimpLeg, 600 );
	PrintHobble( "a leg at 40%", weak, intact.speed );
	PrintHobble( "a leg limp", limp, intact.speed );
	ENSURE( weak.valid && weak.able == 6 && intact.height - weak.height >= 0.1f && weak.speed >= 0.25f * intact.speed );
	ENSURE( limp.valid && limp.able == 5 && limp.speed >= 0.3f * intact.speed );
	return 0;
}

// ---- strikes and grabs ----

static lpPos Offset( lpPos p, float x, float y, float z )
{
	return lpOffsetPos( p, (lpVec3){ x, y, z } );
}

// A front leg reaches up and ahead: out of the gait at once, its foot there within a second, to 5 cm; let go, it steps
// back in and the mech stands on six again
static int TestRigReaches( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, lpVec3_zero, 0.0f, 0 );
	Run( &s, 60 );
	lpPos target = Offset( lpWorld_GetLimbState( s.world, rig, 0 ).foot, -0.4f, 0.8f, 0.5f );
	lpWorld_SetLimbTarget( s.world, rig, 0, true, target );
	int reachingAt = -1;
	float rigMs = 0.0f;
	for ( int t = 0; t < 60; ++t )
	{
		Run( &s, 1 );
		reachingAt = reachingAt < 0 && lpWorld_GetLimbState( s.world, rig, 0 ).reaching ? t : reachingAt;
		rigMs += lpWorld_GetStats( s.world ).rigMs;
	}
	float off = lpLength( lpSubPos( lpWorld_GetLimbState( s.world, rig, 0 ).foot, target ) );
	lpRigState st = lpWorld_GetRigState( s.world, rig );
	float tilt = lpAtan2( sqrtf( st.up.x * st.up.x + st.up.z * st.up.z ), st.up.y );
	lpWorld_SetLimbTarget( s.world, rig, 0, false, target );
	Run( &s, 120 );
	lpRigState after = lpWorld_GetRigState( s.world, rig );
	printf( "  reaching from step %d, the foot %.3f m from its target after 1 s, tilt %.1f deg, rig %.4f ms a step; let go: %d planted\n",
			reachingAt, off, 57.29578f * tilt, rigMs / 60.0f, after.planted );
	ENSURE( reachingAt >= 0 && reachingAt < 10 && off < 0.05f && tilt < 0.05f && after.planted == 6 );
	DestroySim( &s );
	return 0;
}

// Its right middle and rear legs gone, the right front cannot lift without tipping it over: told to reach, it leans,
// waits, and stays on its feet
static int TestRigStrikeWaitsForBalance( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, lpVec3_zero, 0.0f, 0 );
	Run( &s, 30 );
	LoseLeg( &s, rig, 1 );
	LoseLeg( &s, rig, 2 );
	Run( &s, 120 );
	lpWorld_SetLimbTarget( s.world, rig, 0, true, Offset( lpWorld_GetLimbState( s.world, rig, 0 ).foot, 0.0f, 0.8f, 0.5f ) );
	bool reached = false;
	for ( int t = 0; t < 180; ++t )
	{
		Run( &s, 1 );
		reached = reached || lpWorld_GetLimbState( s.world, rig, 0 ).reaching;
	}
	lpRigState st = lpWorld_GetRigState( s.world, rig );
	float tilt = lpAtan2( sqrtf( st.up.x * st.up.x + st.up.z * st.up.z ), st.up.y );
	printf( "  one right leg left: reaching %d, %d planted, tilt %.1f deg, height %.2f m\n", reached, st.planted, 57.29578f * tilt, st.height );
	ENSURE( reached == false && st.planted >= 3 && tilt < 0.15f ); // a foot may be stepping as it leans
	DestroySim( &s );
	return 0;
}

// A stomp onto a glass pane (a skylight 30 cm up, set on the ground at its edges): raised, then driven down through it.
// Returns the pieces it broke off
static int Stomp( float health )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, lpVec3_zero, 0.0f, 0 );
	Run( &s, 30 );
	lpPos foot = lpWorld_GetLimbState( s.world, rig, 0 ).foot;
	lpVec3 at = { (float)foot.x - 0.3f, 0.3f, (float)foot.z + 1.0f };
	lpPartDef slab = lpDefaultPartDef();
	slab.halfExtents = (lpVec3){ 0.6f, 0.03f, 0.6f };
	slab.material = lp_glass;
	slab.anchored = true;
	lpObjectDef def = lpDefaultObjectDef();
	def.transform.p = (lpPos){ at.x, at.y, at.z };
	def.parts = &slab;
	def.partCount = 1;
	lpCreateObject( s.world, &def );
	lpWorld_SettleStructures( s.world );
	for ( int k = 0; k < 3; ++k )
	{
		lpLink* l = s.world->links.data + s.world->rigs.data[rig].limbs[0].def.links[k];
		l->health = health * l->def.strength;
	}
	Run( &s, 30 );
	int before = lpWorld_GetStats( s.world ).pieceCount;
	lpWorld_SetLimbTarget( s.world, rig, 0, true, (lpPos){ at.x, at.y + 1.3f, at.z } );
	Run( &s, 50 );
	lpWorld_SetLimbTarget( s.world, rig, 0, true, (lpPos){ at.x, at.y - 0.6f, at.z } );
	Run( &s, 40 );
	int broke = lpWorld_GetStats( s.world ).pieceCount - before;
	DestroySim( &s );
	return broke;
}

// At full strength the stomp breaks at least half as much again as with its joints at half health (they jam: the leg
// swings slower and drives with half the torque)
static int TestRigStompsHarderWhole( void )
{
	int whole = Stomp( 1.0f );
	int hurt = Stomp( 0.5f );
	printf( "  a stomp on a glass pane broke off %d pieces whole, %d with its leg at half health\n", whole, hurt );
	ENSURE( whole > 0 && (float)whole >= 1.5f * (float)hurt );
	return 0;
}

// A claw grabs a 200 kg crate, lifts it half a metre, and drops it when the leg's tibia (the claw) is taken off
static int TestRigGrabs( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, lpVec3_zero, 0.0f, 0 );
	Run( &s, 30 );
	lpPos foot = lpWorld_GetLimbState( s.world, rig, 0 ).foot;
	float half = 0.5f * lpCbrt( 200.0f / lpWorld_GetMaterial( s.world, lp_metal )->density );
	lpPartDef box = lpDefaultPartDef();
	box.halfExtents = (lpVec3){ half, half, half };
	box.material = lp_metal;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = Offset( foot, 0.0f, half, 0.8f );
	def.parts = &box;
	def.partCount = 1;
	int crate = lpCreateObject( s.world, &def );
	Run( &s, 30 );
	// Reach at the crate's near face, low: the claw comes to rest against it
	lpPos face = Offset( def.transform.p, 0.0f, 0.0f, -half + 0.05f );
	lpWorld_SetLimbTarget( s.world, rig, 0, true, face );
	// Like a player: it grabs the moment the claw touches the crate (it touches the ground first, less than the crate)
	int grip = -1;
	lpLimbState st = { 0 };
	for ( int t = 0; t < 60 && grip < 0; ++t )
	{
		Run( &s, 1 );
		st = lpWorld_GetLimbState( s.world, rig, 0 );
		grip = st.touching >= 0 && lpWorld_GetPieceInfo( s.world, st.touching ).body == crate ? lpRigGrab( s.world, rig, 0 ) : -1;
	}
	lpWorldTransform xf;
	lpWorld_GetBodyTransform( s.world, crate, &xf );
	float rest = (float)xf.p.y;
	lpWorld_SetLimbTarget( s.world, rig, 0, true, Offset( st.foot, 0.0f, 0.8f, 0.0f ) );
	Run( &s, 90 );
	lpWorld_GetBodyTransform( s.world, crate, &xf );
	float lifted = (float)xf.p.y;
	lpDestroyLink( s.world, s.world->rigs.data[rig].limbs[0].def.links[2] ); // the claw's knee goes
	Run( &s, 60 );
	lpWorld_GetBodyTransform( s.world, crate, &xf );
	float dropped = (float)xf.p.y;
	printf( "  touching piece %d, grip link %d; the crate at %.2f m, lifted to %.2f, after the claw went %.2f\n", st.touching, grip, rest,
			lifted, dropped );
	ENSURE( st.touching >= 0 && grip >= 0 && lifted - rest > 0.4f && dropped < lifted - 0.3f );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// ---- bones ----

// The body a limb's joint turns (its far end): joint 1 turns the femur
static int JointBody( const Sim* s, int rig, int limb, int joint )
{
	const lpLink* l = s->world->links.data + s->world->rigs.data[rig].limbs[limb].def.links[joint];
	return s->world->pieces.data[l->ends[1].piece].body;
}

// Legs whose femur is whole (its hip and knee hinges on one body)
static int WholeFemurs( const Sim* s, int rig )
{
	int whole = 0;
	for ( int i = 0; i < s->world->rigs.data[rig].limbCount; ++i )
	{
		const lpLink* knee = s->world->links.data + s->world->rigs.data[rig].limbs[i].def.links[2];
		whole += knee->alive && s->world->pieces.data[knee->ends[0].piece].body == JointBody( s, rig, i, 1 ) ? 1 : 0;
	}
	return whole;
}

// What the stress checks of a run did: solves, of the torso, bonds broken, the worst utilization judged and when
typedef struct BoneReport
{
	int solves, torsoSolves, breaks, firstBreak, landed;
	float peak, stressMs;
	float impact, rebound; // the torso's speed down as it landed, and up after
} BoneReport;

static BoneReport RunBones( Sim* s, int rig, int steps )
{
	BoneReport r = { 0, 0, 0, -1, -1, 0.0f, 0.0f, 0.0f, 0.0f };
	float falling = 0.0f;
	for ( int t = 0; t < steps; ++t )
	{
		Run( s, 1 );
		lpStats st = lpWorld_GetStats( s->world );
		lpRigState rs = lpWorld_GetRigState( s->world, rig );
		r.solves += st.stressSolves;
		r.breaks += st.stressBreaks;
		r.firstBreak = r.firstBreak < 0 && st.stressBreaks > 0 ? t : r.firstBreak;
		r.stressMs += st.stressMs;
		for ( int j = 0; j < s->world->stressJobCount && st.stressSolves > 0; ++j )
		{
			r.torsoSolves += s->world->stressJobs[j].body == rs.body ? 1 : 0;
			r.peak = fmaxf( r.peak, s->world->stressJobs[j].peak );
		}
		// Landed: the torso stops falling
		float vy = rs.body >= 0 ? lpPhys_GetLinearVelocity( s->world->phys, s->world->bodies.data[rs.body].id ).y : 0.0f;
		falling = fminf( falling, vy );
		r.landed = r.landed < 0 && falling < -3.0f && vy > -1.0f ? t : r.landed;
		r.impact = -falling;
		r.rebound = r.landed >= 0 ? fmaxf( r.rebound, vy ) : 0.0f;
	}
	return r;
}

// Dropped 1.5 m on its feet: nothing breaks. Its bones are checked as it lands (the feet's hits jolt the legs).
static int TestRigLandsWhole( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 1.5f, 0.0f }, 0.0f, 0 );
	Run( &s, 1 );
	lpStats before = lpWorld_GetStats( s.world );
	BoneReport r = RunBones( &s, rig, 240 );
	lpStats after = lpWorld_GetStats( s.world );
	lpRigState rs = lpWorld_GetRigState( s.world, rig );
	printf( "  dropped 1.5 m, landed at step %d at %.1f m/s (rebounding at %.1f): %d stress solves (%d of the torso), the worst joint "
			"at %.2f of its limit, %d broke; pieces %d -> %d, bonds %d -> %d, links %d -> %d; %d able, %.2f m over its feet\n",
			r.landed, r.impact, r.rebound, r.solves, r.torsoSolves, r.peak, r.breaks, before.pieceCount, after.pieceCount, before.bondCount,
			after.bondCount, before.linkCount, after.linkCount, rs.able, rs.height );
	// Its servos follow it down rather than fling it back up (at 4.4 m/s, before): what rebound is left is Box3D pushing
	// the soles back out of the ground, at up to the physics engine's contact speed (3 m/s)
	ENSURE( r.landed > 0 && r.solves > 0 && r.peak < 1.0f && r.breaks == 0 && r.rebound < 3.5f );
	ENSURE( after.pieceCount == before.pieceCount && after.bondCount == before.bondCount && after.linkCount == before.linkCount );
	ENSURE( rs.able == 6 );
	DestroySim( &s );
	return 0;
}

// A blow on a femur's weld, too weak to break the sheet metal, cracks it nearly through: standing, it holds the mech up;
// dropped 1.5 m, it snaps on landing, and only that one
static int TestRigCrackedFemurSnaps( void )
{
	BoneReport r[2];
	int whole[2], able[2];
	bool legAble[2];
	float health = 0.0f;
	for ( int drop = 0; drop < 2; ++drop )
	{
		Sim s = CreateSim( -1 );
		int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, drop ? 1.5f : 0.0f, 0.0f }, 0.0f, 0 );
		int femur = JointBody( &s, rig, 1, 1 );
		lpWorldTransform xf;
		lpWorld_GetBodyTransform( s.world, femur, &xf );
		lpImpactDef crack = { 0 };
		crack.point = lpTransformWorldPoint( xf, (lpVec3){ 0.0f, 0.09f, 0.0f } ); // on top of the weld
		crack.direction = lpRotateVector( xf.q, (lpVec3){ 0.0f, -1.0f, 0.0f } );
		crack.radius = 0.3f;
		crack.energy = 4800.0f * LP_PI * crack.radius * crack.radius; // 4.8 kJ/m^2 at the centre: sheet metal takes 6
		lpWorld_AddImpact( s.world, &crack );
		Run( &s, 1 );
		const lpPiece* half = s.world->pieces.data + s.world->bodies.data[femur].pieces.data[0];
		const lpBond* weld = half->bonds.count == 1 ? s.world->bonds.data + half->bonds.data[0] : NULL;
		health = weld != NULL ? weld->health / weld->strength : 0.0f;
		r[drop] = RunBones( &s, rig, 240 );
		whole[drop] = WholeFemurs( &s, rig );
		able[drop] = lpWorld_GetRigState( s.world, rig ).able;
		legAble[drop] = lpWorld_GetLimbState( s.world, rig, 1 ).able;
		ENSURE( lpWorld_Validate( s.world ) );
		DestroySim( &s );
	}
	printf( "  the weld cracked to %.2f of its strength; standing: the worst joint at %.2f, %d femurs whole; dropped: landed at step "
			"%d, the weld snapped at %d (%d broke), %d femurs whole, %d legs able\n",
			health, r[0].peak, whole[0], r[1].landed, r[1].firstBreak, r[1].breaks, whole[1], able[1] );
	ENSURE( health > 0.03f && health < 0.1f );
	ENSURE( r[0].breaks == 0 && whole[0] == 6 && able[0] == 6 && legAble[0] && legAble[1] == false );
	ENSURE( r[1].breaks == 1 && r[1].firstBreak >= r[1].landed - 15 && r[1].firstBreak <= r[1].landed + 15 && whole[1] == 5 &&
			able[1] == 5 ); // within a quarter second of landing (within 5 steps today)
	return 0;
}

// Walking swings its legs' loads every stride: the torso's stress is solved at most twice a second, and it is cheap
static int TestRigWalkingBones( void )
{
	Sim s = CreateSim( -1 );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.0f, -18.0f }, 0.0f, 0 );
	Run( &s, 30 );
	lpRigControl go = { 1.0f, 0.0f, 0.0f, 0.0f };
	lpWorld_SetRigControl( s.world, rig, &go );
	Run( &s, 120 );
	BoneReport r = RunBones( &s, rig, 600 );
	printf( "  walking 10 s: %d stress solves (%d of the torso), the worst joint at %.2f of its limit, %d broke; stress %.4f ms a step\n",
			r.solves, r.torsoSolves, r.peak, r.breaks, r.stressMs / 600.0f );
	ENSURE( r.torsoSolves <= 20 && r.breaks == 0 && r.peak < 0.6f ); // its bones outlast its servos (the cost is printed)
	ENSURE( lpWorld_GetRigState( s.world, rig ).able == 6 );
	DestroySim( &s );
	return 0;
}

typedef struct NoWalkerReport
{
	float sink;		// m the torso dropped standing on its targets
	float standing; // m: the worst foot off its target, standing
	float lifted;	// m: the lifted foot off its target
	float others;	// m: the worst other foot off its target meanwhile
	float followed; // m the torso moved when its pose was pushed 0.1 m forward
	uint64_t hash;
} NoWalkerReport;

static float FootOff( const Sim* s, int rig, int limb, lpPos target )
{
	return lpLength( lpSubPos( lpWorld_GetLimbState( s->world, rig, limb ).foot, target ) );
}

static NoWalkerReport NoWalker( int workers )
{
	NoWalkerReport r = { 0 };
	Sim s = CreateSimWorkers( -1, workers );
	int rig = lpAddHexapod( s.world, (lpVec3){ 0.0f, 0.0f, 0.0f }, 0.0f, 0 );
	s.world->rigs.data[rig].def.walker = lp_walkerNone; // the game walks it
	lpFootTarget targets[6];
	for ( int i = 0; i < 6; ++i )
	{
		targets[i] = (lpFootTarget){ true, lpWorld_GetLimbState( s.world, rig, i ).foot, lpVec3_zero };
		lpWorld_SetFootTarget( s.world, rig, i, targets + i );
	}
	// The pose where the torso stands: the legs hold it there (unset, the pose would follow the torso as it sags)
	lpWorldTransform stand;
	lpWorld_GetBodyTransform( s.world, lpWorld_GetRigState( s.world, rig ).body, &stand );
	lpWorld_SetRigPose( s.world, rig, stand, lpVec3_zero, lpVec3_zero );
	float start = (float)lpWorld_GetRigState( s.world, rig ).position.y;
	Run( &s, 120 );
	r.sink = start - (float)lpWorld_GetRigState( s.world, rig ).position.y;
	for ( int i = 0; i < 6; ++i )
	{
		r.standing = fmaxf( r.standing, FootOff( &s, rig, i, targets[i].point ) );
	}

	// One foot up 0.25 m
	targets[0].point.y += 0.25f;
	lpWorld_SetFootTarget( s.world, rig, 0, targets + 0 );
	Run( &s, 60 );
	r.lifted = FootOff( &s, rig, 0, targets[0].point );
	for ( int i = 1; i < 6; ++i )
	{
		r.others = fmaxf( r.others, FootOff( &s, rig, i, targets[i].point ) );
	}
	targets[0].point.y -= 0.25f;
	lpWorld_SetFootTarget( s.world, rig, 0, targets + 0 );
	Run( &s, 60 );

	// The pose pushed 0.1 m forward: the legs push the torso there
	lpRigState st = lpWorld_GetRigState( s.world, rig );
	lpWorldTransform pose;
	lpWorld_GetBodyTransform( s.world, st.body, &pose );
	lpPos before = pose.p;
	pose.p = lpOffsetPos( pose.p, lpMulSV( 0.1f, st.forward ) );
	lpWorld_SetRigPose( s.world, rig, pose, lpVec3_zero, lpVec3_zero );
	Run( &s, 120 );
	lpWorld_GetBodyTransform( s.world, st.body, &pose );
	r.followed = lpDot( lpSubPos( pose.p, before ), st.forward );
	r.hash = lpWorld_Hash( s.world );
	DestroySim( &s );
	return r;
}

// A rig the game walks itself (lp_walkerNone): on foot targets where its feet are it stands, holding its height; a foot
// sent up 0.25 m gets there while the others hold; its pose pushed forward, its legs push the torso after it; and the
// same targets give the same world at 1 and 8 workers
static int TestRigNoWalker( void )
{
	NoWalkerReport one = NoWalker( 1 );
	NoWalkerReport eight = NoWalker( 8 );
	printf( "  standing: sank %.3f m, feet within %.3f m; a foot lifted to %.3f m of its target, the others within %.3f m; "
			"pushed 0.1 m, the torso moved %.3f m\n",
			(double)one.sink, (double)one.standing, (double)one.lifted, (double)one.others, (double)one.followed );
	ENSURE( one.sink < 0.03f && one.standing < 0.03f );
	ENSURE( one.lifted < 0.04f && one.others < 0.03f );
	ENSURE( one.followed > 0.07f && one.followed < 0.13f );
	ENSURE( one.hash == eight.hash );
	return 0;
}

int RigTest( void )
{
	RUN_TEST( TestKitStands, OUTCOME );
	RUN_TEST( TestRigIKRoundTrip, MECHANISM );
	RUN_TEST( TestRigModelMatchesBodies, MECHANISM );
	RUN_TEST( TestRigModelBent, MECHANISM );
	RUN_TEST( TestRigStumpFoot, MECHANISM );
	RUN_TEST( TestRigStumpIK, MECHANISM );
	RUN_TEST( TestRigCrouch, OUTCOME );
	RUN_TEST( TestRigDeterminism, DETERMINISM );
	RUN_TEST( TestRigCost, TIMING );
	RUN_TEST( TestRigWalksStraight, OUTCOME );
	RUN_TEST( TestRigTurns, OUTCOME );
	RUN_TEST( TestRigClimbsSlope, OUTCOME );
	RUN_TEST( TestRigStepsUpAndDown, OUTCOME );
	RUN_TEST( TestRigStops, OUTCOME );
	RUN_TEST( TestRigWalkDeterminism, DETERMINISM );
	RUN_TEST( TestRigWalkCost, TIMING );
	RUN_TEST( TestRigPatrols, OUTCOME );
	RUN_TEST( TestRigLosesLegs, OUTCOME );
	RUN_TEST( TestRigWalksOnAPeg, OUTCOME );
	RUN_TEST( TestRigWeakAndLimpLegs, OUTCOME );
	RUN_TEST( TestRigBleedsOut, OUTCOME );
	RUN_TEST( TestRigReaches, OUTCOME );
	RUN_TEST( TestRigStrikeWaitsForBalance, OUTCOME );
	RUN_TEST( TestRigStompsHarderWhole, OUTCOME );
	RUN_TEST( TestRigGrabs, OUTCOME );
	RUN_TEST( TestRigNoWalker, OUTCOME );
	RUN_TEST( TestRigLandsWhole, OUTCOME );
	RUN_TEST( TestRigCrackedFemurSnaps, OUTCOME );
	RUN_TEST( TestRigWalkingBones, OUTCOME );
	return 0;
}
