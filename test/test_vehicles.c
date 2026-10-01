// SPDX-License-Identifier: MIT
// Vehicles: wheels that hold a chassis up, grip, stay stable at speed, and come off when they should.

#include "test_macros.h"
#include "test_sim.h"

#include <math.h>

// A flat static strip of ground along z, from z0 to z1, top at y = 0
static void AddStrip( Sim* s, float halfWidth, float z0, float z1 )
{
	lpPartDef part = lpDefaultPartDef();
	part.halfExtents = (lpVec3){ halfWidth, 0.5f, 0.5f * ( z1 - z0 ) };
	part.material = lp_ground;
	part.anchored = true;
	lpObjectDef def = lpDefaultObjectDef();
	def.transform.p = (lpPos){ 0.0f, -0.5f, 0.5f * ( z0 + z1 ) };
	def.parts = &part;
	def.partCount = 1;
	lpCreateObject( s->world, &def );
}

static int AddStatic( Sim* s, lpVec3 center, lpVec3 half, lpQuat q, int material )
{
	lpPartDef part = lpDefaultPartDef();
	part.halfExtents = half;
	part.material = (uint8_t)material;
	part.anchored = true;
	lpObjectDef def = lpDefaultObjectDef();
	def.transform.p = (lpPos){ center.x, center.y, center.z };
	def.transform.q = q;
	def.parts = &part;
	def.partCount = 1;
	return lpCreateObject( s->world, &def );
}

typedef struct Car
{
	int body;
	int vehicle;
} Car;

// A box car facing +z: a wooden chassis (length 4 m, width 1.8 m, `height` thick, in `parts` slabs along z) on four
// wheels at its bottom corners, rear-wheel drive, front steering, handbrake on the rear. The chassis's bottom is at
// `bottom`; at rest on flat ground it sits about 0.72 m up.
static Car AddCar( Sim* s, lpVec3 at, lpQuat q, float bottom, float height, int parts, lpVec3 velocity )
{
	lpPartDef slabs[4];
	float length = 4.0f / (float)parts;
	for ( int i = 0; i < parts; ++i )
	{
		slabs[i] = lpDefaultPartDef();
		slabs[i].halfExtents = (lpVec3){ 0.9f, 0.5f * height, 0.5f * length };
		slabs[i].transform.p = (lpVec3){ 0.0f, 0.5f * height, -2.0f + ( (float)i + 0.5f ) * length };
		slabs[i].material = lp_wood;
		slabs[i].color = 0x3060A0u;
		slabs[i].grainAxis = (lpVec3){ 0.0f, 0.0f, 1.0f };
	}
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (lpPos){ at.x, at.y + bottom, at.z };
	def.transform.q = q;
	def.parts = slabs;
	def.partCount = parts;
	def.linearVelocity = velocity;
	Car car;
	car.body = lpCreateObject( s->world, &def );

	lpWheelDef wheels[4];
	for ( int i = 0; i < 4; ++i )
	{
		float x = ( i & 1 ) ? 0.8f : -0.8f;
		float z = ( i & 2 ) ? 1.4f : -1.4f; // wheels 2 and 3 in front
		lpVec3 local = { x, 0.0f, z };
		lpVec3 world = lpRotateVector( q, local );
		wheels[i] = lpDefaultWheelDef();
		wheels[i].mount = (lpPos){ at.x + world.x, at.y + bottom + world.y, at.z + world.z };
		wheels[i].driveShare = ( i & 2 ) ? 0.0f : 0.5f;
		wheels[i].steerFactor = ( i & 2 ) ? 1.0f : 0.0f;
		wheels[i].handbrake = ( i & 2 ) == 0;
	}
	lpVehicleDef vd = lpDefaultVehicleDef();
	vd.body = car.body;
	vd.forward = lpRotateVector( q, (lpVec3){ 0.0f, 0.0f, 1.0f } );
	vd.up = lpRotateVector( q, (lpVec3){ 0.0f, 1.0f, 0.0f } );
	vd.wheels = wheels;
	vd.wheelCount = 4;
	car.vehicle = lpCreateVehicle( s->world, &vd );
	return car;
}

static lpWorldTransform Pose( const Sim* s, int body )
{
	lpWorldTransform xf = { 0 };
	lpWorld_GetBodyTransform( s->world, body, &xf );
	return xf;
}

static float Upright( const Sim* s, int body )
{
	return lpRotateVector( Pose( s, body ).q, (lpVec3){ 0.0f, 1.0f, 0.0f } ).y;
}

static void Drive( Sim* s, int vehicle, float throttle, float brake, float steer, bool handbrake )
{
	lpVehicleControl c = { throttle, brake, steer, handbrake };
	lpWorld_SetVehicleControl( s->world, vehicle, &c );
}

static void Step( Sim* s )
{
	lpWorld_Step( s->world, 1.0f / 60.0f, 4 );
}

// At rest each spring carries a quarter of the car: it sags mg / 4k (g / omega^2 at the default stiffness), and the
// parked car falls asleep
static int TestWheelRestHeight( void )
{
	Sim s = CreateSim( -1 );
	Car car = AddCar( &s, (lpVec3){ 0.0f, 0.0f, 0.0f }, lpQuat_identity, 0.85f, 0.4f, 1, lpVec3_zero );
	ENSURE( car.vehicle == 0 );
	int asleepAt = -1;
	for ( int t = 0; t < 180; ++t )
	{
		Step( &s );
		if ( asleepAt < 0 && lpPhys_IsAwake( s.world->phys, s.world->bodies.data[car.body].id ) == false )
		{
			asleepAt = t;
		}
	}
	float sag = 10.0f / ( ( 2.0f * 3.14159265f * 1.4f ) * ( 2.0f * 3.14159265f * 1.4f ) );
	for ( int i = 0; i < 4; ++i )
	{
		lpWheelState ws = lpWorld_GetWheelState( s.world, lpWorld_GetVehicleWheel( s.world, car.vehicle, i ) );
		ENSURE( ws.alive && ws.grounded );
		ENSURE_NEAR( 0.5f - ws.length, sag, 0.05f * sag );
		ENSURE_NEAR( ws.load, 0.25f * lpPhys_GetMass( s.world->phys, s.world->bodies.data[car.body].id ) * 10.0f, 30.0f );
	}
	lpVehicleState vs = lpWorld_GetVehicleState( s.world, car.vehicle );
	printf( "  sag %.3f m, asleep at tick %d, %d of %d wheels grounded\n", 0.5f - lpWorld_GetWheelState( s.world, lpWorld_GetVehicleWheel( s.world, car.vehicle, 0 ) ).length,
			asleepAt, vs.grounded, vs.attached );
	ENSURE( vs.body == car.body && vs.grounded == 4 && vs.driven == 2 && vs.steerable == 2 );
	ENSURE( asleepAt >= 0 && asleepAt < 120 );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// 40 m/s down a long straight for ten seconds: it stays finite, upright, on course and at speed
static int TestWheelHighSpeedStable( void )
{
	Sim s = CreateSim( -1 );
	AddStrip( &s, 12.0f, -20.0f, 480.0f );
	Car car = AddCar( &s, (lpVec3){ 0.0f, 0.0f, 0.0f }, lpQuat_identity, 0.75f, 0.4f, 1, (lpVec3){ 0.0f, 0.0f, 40.0f } );
	Drive( &s, car.vehicle, 1.0f, 0.0f, 0.0f, false );
	float worstUp = 1.0f, worstDrift = 0.0f, lowest = 1e9f, highest = -1e9f;
	for ( int t = 0; t < 600; ++t )
	{
		Step( &s );
		lpWorldTransform xf = Pose( &s, car.body );
		ENSURE( isfinite( (float)xf.p.x ) && isfinite( (float)xf.p.y ) && isfinite( (float)xf.p.z ) );
		worstUp = fminf( worstUp, Upright( &s, car.body ) );
		worstDrift = fmaxf( worstDrift, fabsf( (float)xf.p.x ) );
		if ( t > 60 )
		{
			lowest = fminf( lowest, (float)xf.p.y );
			highest = fmaxf( highest, (float)xf.p.y );
		}
	}
	lpVehicleState vs = lpWorld_GetVehicleState( s.world, car.vehicle );
	printf( "  after 600 ticks: z %.1f m, speed %.2f m/s, drift %.3f m, worst upright %.4f, bounce %.3f m\n",
			(double)Pose( &s, car.body ).p.z, vs.speed, worstDrift, worstUp, highest - lowest );
	ENSURE( worstUp > 0.99f && worstDrift < 1.0f && highest - lowest < 0.05f );
	ENSURE( vs.speed > 38.0f && vs.attached == 4 );
	DestroySim( &s );
	return 0;
}

// Full lock at 30 m/s: it slides (the tyres give at their friction circle) instead of rolling over
static int TestWheelSteerNoFlip( void )
{
	lpWorldDef ld = lpDefaultWorldDef();
	Sim s = CreateSimDef( ld, -1 );
	AddStatic( &s, (lpVec3){ 0.0f, -0.5f, 0.0f }, (lpVec3){ 150.0f, 0.5f, 150.0f }, lpQuat_identity, lp_ground );
	Car car = AddCar( &s, (lpVec3){ 0.0f, 0.0f, -100.0f }, lpQuat_identity, 0.75f, 0.4f, 1, (lpVec3){ 0.0f, 0.0f, 30.0f } );
	Drive( &s, car.vehicle, 0.0f, 0.0f, 0.0f, false );
	Run( &s, 30 );
	Drive( &s, car.vehicle, 0.0f, 0.0f, 1.0f, false );
	float worstUp = 1.0f;
	float slip = 0.0f;
	for ( int t = 0; t < 180; ++t )
	{
		Step( &s );
		worstUp = fminf( worstUp, Upright( &s, car.body ) );
		for ( int i = 0; i < 4; ++i )
		{
			slip = fmaxf( slip, lpWorld_GetWheelState( s.world, lpWorld_GetVehicleWheel( s.world, car.vehicle, i ) ).slip );
		}
	}
	lpVehicleState vs = lpWorld_GetVehicleState( s.world, car.vehicle );
	printf( "  worst upright %.3f, speed after %.1f m/s, worst slip %.1f m/s\n", worstUp, vs.speed, slip );
	ENSURE( worstUp > 0.7f && vs.grounded >= 3 );
	DestroySim( &s );
	return 0;
}

// A 20 cm kerb at 15 m/s: the wheels roll up it (the tyre's cast is round), and the car is not launched
static int TestWheelKerb( void )
{
	Sim s = CreateSim( -1 );
	AddStrip( &s, 12.0f, -40.0f, 120.0f );
	AddStatic( &s, (lpVec3){ 0.0f, 0.1f, 20.0f + 30.0f }, (lpVec3){ 8.0f, 0.1f, 30.0f }, lpQuat_identity, lp_stone );
	Car car = AddCar( &s, (lpVec3){ 0.0f, 0.0f, 0.0f }, lpQuat_identity, 0.75f, 0.4f, 1, (lpVec3){ 0.0f, 0.0f, 15.0f } );
	Drive( &s, car.vehicle, 0.3f, 0.0f, 0.0f, false );
	Run( &s, 30 );
	float rest = (float)Pose( &s, car.body ).p.y;
	float highest = rest;
	float worstUp = 1.0f;
	for ( int t = 0; t < 120; ++t )
	{
		Step( &s );
		highest = fmaxf( highest, (float)Pose( &s, car.body ).p.y );
		worstUp = fminf( worstUp, Upright( &s, car.body ) );
	}
	lpVehicleState vs = lpWorld_GetVehicleState( s.world, car.vehicle );
	printf( "  rose %.3f m over the 0.2 m kerb, worst upright %.3f, now at z %.1f, %d wheels on\n", highest - rest, worstUp,
			(double)Pose( &s, car.body ).p.z, vs.attached );
	ENSURE( (float)Pose( &s, car.body ).p.z > 40.0f ); // it went over
	ENSURE( highest - rest < 0.5f && worstUp > 0.9f && vs.attached == 4 );
	DestroySim( &s );
	return 0;
}

// On a 15 degree slope the handbrake holds it (the rear wheels lock); released, it rolls
static int TestWheelParkedOnSlope( void )
{
	Sim s = CreateSim( -1 );
	float angle = 15.0f * 3.14159265f / 180.0f;
	lpQuat tilt = lpMakeQuatFromAxisAngle( (lpVec3){ 1.0f, 0.0f, 0.0f }, -angle ); // rises toward +z
	AddStatic( &s, (lpVec3){ 0.0f, 3.0f, 0.0f }, (lpVec3){ 6.0f, 0.5f, 20.0f }, tilt, lp_ground );
	lpVec3 top = lpAdd( (lpVec3){ 0.0f, 3.0f, 0.0f }, lpRotateVector( tilt, (lpVec3){ 0.0f, 0.5f, 0.0f } ) );
	Car car = AddCar( &s, top, tilt, 0.8f, 0.4f, 1, lpVec3_zero );
	Drive( &s, car.vehicle, 0.0f, 0.0f, 0.0f, true );
	Run( &s, 90 );
	lpPos held = Pose( &s, car.body ).p;
	Run( &s, 120 );
	float creep = lpLength( lpSubPos( Pose( &s, car.body ).p, held ) );
	Drive( &s, car.vehicle, 0.0f, 0.0f, 0.0f, false );
	Run( &s, 120 );
	lpVehicleState vs = lpWorld_GetVehicleState( s.world, car.vehicle );
	printf( "  handbrake: crept %.4f m in 2 s; released: %.2f m/s after 2 s\n", creep, vs.speed );
	ENSURE( creep < 0.01f );
	ENSURE( vs.speed < -1.0f ); // rolling back down, backwards
	DestroySim( &s );
	return 0;
}

// A wheel's mount carries its own load: a 1 m drop holds, an 8 m drop tears wheels off, and they come off as wheels
static int TestWheelBreaksOnHardLanding( void )
{
	for ( int pass = 0; pass < 2; ++pass )
	{
		Sim s = CreateSim( -1 );
		float drop = pass == 0 ? 1.0f : 8.0f;
		Car car = AddCar( &s, (lpVec3){ 0.0f, 0.0f, 0.0f }, lpQuat_identity, 0.75f + drop, 0.4f, 1, lpVec3_zero );
		int bodiesBefore = lpWorld_GetStats( s.world ).debrisBodies;
		Run( &s, 150 );
		lpVehicleState vs = lpWorld_GetVehicleState( s.world, car.vehicle );
		lpStats st = lpWorld_GetStats( s.world );
		printf( "  %.0f m drop: %d wheels on, %d debris bodies (%d before)\n", drop, vs.attached, st.debrisBodies + st.rubbleBodies,
				bodiesBefore );
		if ( pass == 0 )
		{
			ENSURE( vs.attached == 4 );
		}
		else
		{
			ENSURE( vs.attached < 4 );
			ENSURE( st.debrisBodies + st.rubbleBodies >= bodiesBefore + 4 - vs.attached ); // the wheels that came off
		}
		ENSURE( lpWorld_Validate( s.world ) );
		DestroySim( &s );
	}
	return 0;
}

// A grenade at a wheel's mount: the wheel follows its piece's cells or comes off, and the world stays valid
static int TestWheelFollowsFracture( void )
{
	Sim s = CreateSim( -1 );
	Car car = AddCar( &s, (lpVec3){ 0.0f, 0.0f, 0.0f }, lpQuat_identity, 0.75f, 0.4f, 2, lpVec3_zero );
	Run( &s, 60 );
	lpWheelState before = lpWorld_GetWheelState( s.world, lpWorld_GetVehicleWheel( s.world, car.vehicle, 3 ) );
	lpImpactDef blast = { 0 };
	blast.point = lpOffsetPos( before.hub.p, (lpVec3){ 0.0f, 0.4f, 0.0f } );
	blast.radius = 1.4f;
	blast.energy = 80000.0f;
	blast.impulse = 6.0f;
	blast.explosion = true;
	lpWorld_AddImpact( s.world, &blast );
	bool valid = true;
	for ( int t = 0; t < 120; ++t )
	{
		Step( &s );
		valid = valid && lpWorld_Validate( s.world );
	}
	lpVehicleState vs = lpWorld_GetVehicleState( s.world, car.vehicle );
	printf( "  after the grenade: %d wheels on, %d pieces\n", vs.attached, lpWorld_GetStats( s.world ).pieceCount );
	ENSURE( valid );
	ENSURE( vs.attached >= 2 );
	DestroySim( &s );
	return 0;
}

// Cut a two-slab chassis in two: each half keeps the two wheels mounted on it
static int TestWheelSplitHalfRolls( void )
{
	Sim s = CreateSim( -1 );
	Car car = AddCar( &s, (lpVec3){ 0.0f, 0.0f, 0.0f }, lpQuat_identity, 0.75f, 0.4f, 2, lpVec3_zero );
	Run( &s, 60 );
	const lpBody* b = s.world->bodies.data + car.body;
	int cut = 0;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		lpPiece* p = s.world->pieces.data + b->pieces.data[k];
		while ( p->bonds.count > 0 )
		{
			lpBreakBond( s.world, p->bonds.data[0] );
			cut += 1;
		}
	}
	lpMarkDirty( s.world, car.body );
	bool valid = true;
	for ( int t = 0; t < 90; ++t )
	{
		Step( &s );
		valid = valid && lpWorld_Validate( s.world );
	}
	int bodies[4];
	for ( int i = 0; i < 4; ++i )
	{
		bodies[i] = lpWorld_GetWheelState( s.world, lpWorld_GetVehicleWheel( s.world, car.vehicle, i ) ).body;
	}
	printf( "  %d bonds cut; wheels on bodies %d %d %d %d\n", cut, bodies[0], bodies[1], bodies[2], bodies[3] );
	ENSURE( valid && cut > 0 );
	ENSURE( lpWorld_GetVehicleState( s.world, car.vehicle ).attached == 4 );
	ENSURE( bodies[0] == bodies[1] && bodies[2] == bodies[3] && bodies[0] != bodies[2] );
	DestroySim( &s );
	return 0;
}

// Wheels load a structure they stand on: a heavy truck breaks a plank bridge a light car crosses
static int TestWheelLoadsBridge( void )
{
	float masses[2] = { 0.0f, 0.0f };
	int breaks[2] = { 0, 0 };
	int judged[2] = { 0, 0 };
	for ( int pass = 0; pass < 2; ++pass )
	{
		Sim s = CreateSim( -1 );
		AddStrip( &s, 12.0f, -30.0f, 0.0f );
		AddStrip( &s, 12.0f, 8.0f, 120.0f );
		// Two stone piers, and a deck of planks across the 8 m gap, nailed to them
		lpPartDef parts[6];
		int count = 0;
		for ( int k = 0; k < 2; ++k )
		{
			parts[count] = lpDefaultPartDef();
			parts[count].halfExtents = (lpVec3){ 3.0f, 0.5f, 0.5f };
			parts[count].transform.p = (lpVec3){ 0.0f, -0.5f, k == 0 ? -0.5f : 8.5f };
			parts[count].anchored = true;
			count += 1;
		}
		for ( int k = 0; k < 4; ++k )
		{
			parts[count] = lpDefaultPartDef();
			parts[count].halfExtents = (lpVec3){ 0.35f, 0.03f, 5.0f };
			parts[count].transform.p = (lpVec3){ -1.05f + 0.7f * (float)k, 0.03f, 4.0f };
			parts[count].material = lp_wood;
			parts[count].grainAxis = (lpVec3){ 0.0f, 0.0f, 1.0f };
			count += 1;
		}
		lpObjectDef def = lpDefaultObjectDef();
		def.parts = parts;
		def.partCount = count;
		lpCreateObject( s.world, &def );
		lpWorld_SettleStructures( s.world );

		// The light car's chassis is a thin slab; the truck's is a thick one
		Car car = AddCar( &s, (lpVec3){ 0.0f, 0.06f, -12.0f }, lpQuat_identity, 0.8f, pass == 0 ? 0.2f : 0.7f, 1, lpVec3_zero );
		masses[pass] = lpPhys_GetMass( s.world->phys, s.world->bodies.data[car.body].id );
		Drive( &s, car.vehicle, 0.4f, 0.0f, 0.0f, false );
		for ( int t = 0; t < 420; ++t )
		{
			Step( &s );
			breaks[pass] += lpWorld_GetStats( s.world ).stressBreaks;
			judged[pass] += lpWorld_GetStats( s.world ).stressJudged;
		}
		printf( "  %.0f kg: %d stress checks judged, %d joints broke under it, car at z %.1f\n", masses[pass], judged[pass],
				breaks[pass], (double)Pose( &s, car.body ).p.z );
		DestroySim( &s );
	}
	ENSURE( breaks[0] == 0 && breaks[1] > 0 );
	return 0;
}

// Driving, steering and a grenade under it give the same hash at 1, 4 and 8 workers
static uint64_t DriveUnderFire( int workers, int ticks )
{
	Sim s = CreateSimWorkers( -1, workers );
	AddStrip( &s, 20.0f, -40.0f, 200.0f );
	Car car = AddCar( &s, (lpVec3){ 0.0f, 0.0f, 0.0f }, lpQuat_identity, 0.75f, 0.4f, 2, lpVec3_zero );
	Car other = AddCar( &s, (lpVec3){ 4.0f, 0.0f, 30.0f }, lpQuat_identity, 0.75f, 0.4f, 1, lpVec3_zero );
	(void)other;
	for ( int t = 0; t < ticks; ++t )
	{
		if ( t == 10 )
		{
			Drive( &s, car.vehicle, 1.0f, 0.0f, 0.0f, false );
		}
		if ( t == 90 )
		{
			Drive( &s, car.vehicle, 1.0f, 0.0f, 0.4f, false );
		}
		if ( t == 150 )
		{
			Drive( &s, car.vehicle, 0.0f, 1.0f, -0.2f, true );
		}
		if ( t == 120 )
		{
			lpWheelState ws = lpWorld_GetWheelState( s.world, lpWorld_GetVehicleWheel( s.world, car.vehicle, 0 ) );
			lpImpactDef blast = { 0 };
			blast.point = lpOffsetPos( ws.hub.p, (lpVec3){ 0.5f, 0.0f, 0.0f } );
			blast.radius = 1.4f;
			blast.energy = 80000.0f;
			blast.impulse = 6.0f;
			blast.explosion = true;
			lpWorld_AddImpact( s.world, &blast );
		}
		Step( &s );
	}
	uint64_t h = lpWorld_Hash( s.world );
	DestroySim( &s );
	return h;
}

static int TestVehicleDeterminism( void )
{
	uint64_t a = DriveUnderFire( 1, 240 );
	uint64_t b = DriveUnderFire( 4, 240 );
	uint64_t c = DriveUnderFire( 8, 240 );
	printf( "  hashes %016llx %016llx %016llx\n", (unsigned long long)a, (unsigned long long)b, (unsigned long long)c );
	ENSURE( a == b && a == c );
	return 0;
}

// The track's scripted drivers take all three cars round the ring (kerbs, crates, a hump, a plank bridge) in under
// half a minute, upright and whole
static int TestTrackLap( void )
{
	Sim s = CreateSim( lp_sceneTrack );
	int count = lpWorld_GetVehicleCapacity( s.world );
	ENSURE( count == 3 );
	float last[3], travelled[3] = { 0.0f, 0.0f, 0.0f };
	int lapAt[3] = { -1, -1, -1 };
	for ( int v = 0; v < count; ++v )
	{
		lpVehicleState vs = lpWorld_GetVehicleState( s.world, v );
		last[v] = atan2f( (float)vs.position.z, (float)vs.position.x );
	}
	bool valid = true;
	for ( int t = 0; t < 1800; ++t )
	{
		lpSceneDrive( s.world, lp_sceneTrack, t, -1, -1 );
		Step( &s );
		for ( int v = 0; v < count; ++v )
		{
			lpVehicleState vs = lpWorld_GetVehicleState( s.world, v );
			float angle = atan2f( (float)vs.position.z, (float)vs.position.x );
			float d = angle - last[v];
			d = d > 3.14159265f ? d - 6.2831853f : ( d < -3.14159265f ? d + 6.2831853f : d );
			travelled[v] += d;
			last[v] = angle;
			if ( lapAt[v] < 0 && travelled[v] >= 6.2831853f )
			{
				lapAt[v] = t;
			}
		}
		if ( t % 30 == 0 )
		{
			valid = valid && lpWorld_Validate( s.world );
		}
	}
	for ( int v = 0; v < count; ++v )
	{
		lpVehicleState vs = lpWorld_GetVehicleState( s.world, v );
		printf( "  car %d: lap at tick %d, %.2f laps, %d wheels on, upright %.3f, speed %.1f m/s\n", v, lapAt[v],
				travelled[v] / 6.2831853f, vs.attached, vs.up.y, vs.speed );
		ENSURE( lapAt[v] > 0 && vs.attached == 4 && vs.up.y > 0.9f );
	}
	ENSURE( valid );
	DestroySim( &s );
	return 0;
}

typedef struct Crash
{
	float kept;		// of the car's volume, still on its body
	int carPieces;	// then
	int wallPieces; // brick pieces left standing
	float breach;	// brick volume knocked loose
	float power;
	float speedAfter;
} Crash;

// The kit car into a brick wall at a speed, coasting
static Crash CarIntoWall( float speed )
{
	Sim s = CreateSim( -1 );
	AddStatic( &s, (lpVec3){ 0.0f, 0.9f, 0.15f }, (lpVec3){ 4.0f, 0.9f, 0.15f }, lpQuat_identity, lp_brick );
	lpWorld_SettleStructures( s.world );
	int vehicle = lpAddCar( s.world, (lpVec3){ 0.0f, 0.0f, -8.0f }, 0.0f, 0 );
	lpVehicleState vs = lpWorld_GetVehicleState( s.world, vehicle );
	const lpBody* car = s.world->bodies.data + vs.body;
	float volume = car->volume;
	lpPhys_SetLinearVelocity( s.world->phys, car->id, (lpVec3){ 0.0f, 0.0f, speed } );
	Run( &s, 120 );
	vs = lpWorld_GetVehicleState( s.world, vehicle );
	Crash c = { 0 };
	c.kept = vs.body >= 0 ? s.world->bodies.data[vs.body].volume / volume : 0.0f;
	c.carPieces = vs.body >= 0 ? s.world->bodies.data[vs.body].pieces.count : 0;
	c.power = vs.power;
	c.speedAfter = vs.speed;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		const lpPiece* p = s.world->pieces.data + i;
		if ( p->body < 0 || p->material != lp_brick )
		{
			continue;
		}
		if ( s.world->bodies.data[p->body].kind == lp_kindStructure )
		{
			c.wallPieces += 1;
		}
		else
		{
			c.breach += p->shape->volume;
		}
	}
	DestroySim( &s );
	return c;
}

// The harder the crash, the worse for both: at 10 m/s the wall is chipped and the bumper comes off; at 30 m/s the wall
// is breached and the car's front is torn up (engine blocks torn off: less power), but most of the car is one piece
static int TestCarWallCrash( void )
{
	float speeds[3] = { 10.0f, 20.0f, 30.0f };
	Crash c[3];
	for ( int k = 0; k < 3; ++k )
	{
		c[k] = CarIntoWall( speeds[k] );
		printf( "  %.0f m/s: car keeps %.0f%% (%d pieces, power %.2f, then %.1f m/s); wall %d pieces standing, %.3f m^3 knocked "
				"loose\n",
				speeds[k], 100.0f * c[k].kept, c[k].carPieces, c[k].power, c[k].speedAfter, c[k].wallPieces, c[k].breach );
	}
	ENSURE( c[0].kept > 0.9f && c[0].power == 1.0f ); // a bumper at most
	ENSURE( c[0].kept > c[1].kept && c[1].kept > c[2].kept && c[2].kept > 0.5f );
	ENSURE( c[2].power < 1.0f && c[0].breach < c[2].breach && c[2].breach > 0.3f );
	return 0;
}

// A grenade at the fuel tank sets it off: the fuel is gone, so the engine makes no power
static int TestCarTankShot( void )
{
	Sim s = CreateSim( -1 );
	int vehicle = lpAddCar( s.world, (lpVec3){ 0.0f, 0.0f, 0.0f }, 0.0f, 1 );
	Run( &s, 30 );
	ENSURE( lpWorld_GetVehicleState( s.world, vehicle ).power == 1.0f );
	lpImpactDef im = { 0 };
	im.point = (lpPos){ 0.0f, 1.2f, -2.4f };
	im.radius = 1.4f;
	im.energy = 80000.0f;
	im.impulse = 12.0f;
	im.explosion = true;
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 1 );
	ENSURE( s.world->pendingDestroy.count >= 1 ); // the tank went off
	Run( &s, 60 );
	lpVehicleState vs = lpWorld_GetVehicleState( s.world, vehicle );
	printf( "  after the tank went off: power %.2f, %d wheels on, %d pieces\n", vs.power, vs.attached,
			lpWorld_GetStats( s.world ).pieceCount );
	ENSURE( vs.power == 0.0f );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// A heavy blow at the front of the engine knocks its front block loose: the rest of the engine still drives the car,
// at its share
static int TestCarEngineShot( void )
{
	Sim s = CreateSim( -1 );
	int vehicle = lpAddCar( s.world, (lpVec3){ 0.0f, 0.0f, 0.0f }, 0.0f, 2 );
	Run( &s, 30 );
	lpImpactDef im = { 0 };
	im.point = (lpPos){ 0.0f, 1.0f, 1.8f }; // at the front block's face: its bolts go, the floor pan does not crack
	im.direction = (lpVec3){ 0.0f, 0.0f, -1.0f };
	im.radius = 0.5f;
	im.energy = 12000.0f;
	im.impulse = 20.0f;
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 20 );
	float power = lpWorld_GetVehicleState( s.world, vehicle ).power;
	printf( "  after a blow to the engine's front block: power %.3f\n", power );
	ENSURE( power > 0.5f && power < 0.9f );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// What wheels cost per step (casts and the tyre solve), driving on open ground: 4 cars, then 64
static int TestWheelCost( void )
{
	int counts[2] = { 4, 64 };
	for ( int pass = 0; pass < 2; ++pass )
	{
		Sim s = CreateSim( -1 );
		AddStatic( &s, (lpVec3){ 0.0f, -0.5f, 100.0f }, (lpVec3){ 60.0f, 0.5f, 160.0f }, lpQuat_identity, lp_ground );
		Car cars[64];
		for ( int i = 0; i < counts[pass]; ++i )
		{
			lpVec3 at = { -42.0f + 12.0f * (float)( i % 8 ), 0.0f, 8.0f * (float)( i / 8 ) };
			cars[i] = AddCar( &s, at, lpQuat_identity, 0.75f, 0.4f, 1, (lpVec3){ 0.0f, 0.0f, 10.0f } );
			Drive( &s, cars[i].vehicle, 0.5f, 0.0f, 0.0f, false );
		}
		Run( &s, 30 );
		float ms = 0.0f;
		int casts = 0;
		for ( int t = 0; t < 120; ++t )
		{
			Step( &s );
			ms += lpWorld_GetStats( s.world ).vehicleMs;
			casts += lpWorld_GetStats( s.world ).wheelCasts;
		}
		int wheels = 4 * counts[pass];
		printf( "  %d wheels: %.3f ms per step, %.2f us per wheel, %d casts per step\n", wheels, ms / 120.0f,
				1000.0f * ms / ( 120.0f * (float)wheels ), casts / 120 );
		ENSURE( casts == 120 * wheels );
		DestroySim( &s );
	}
	return 0;
}

int VehicleTest( void )
{
	RUN_TEST( TestWheelRestHeight );
	RUN_TEST( TestWheelHighSpeedStable );
	RUN_TEST( TestWheelSteerNoFlip );
	RUN_TEST( TestWheelKerb );
	RUN_TEST( TestWheelParkedOnSlope );
	RUN_TEST( TestWheelBreaksOnHardLanding );
	RUN_TEST( TestWheelFollowsFracture );
	RUN_TEST( TestWheelSplitHalfRolls );
	RUN_TEST( TestWheelLoadsBridge );
	RUN_TEST( TestVehicleDeterminism );
	RUN_TEST( TestTrackLap );
	RUN_TEST( TestCarWallCrash );
	RUN_TEST( TestCarTankShot );
	RUN_TEST( TestCarEngineShot );
	RUN_TEST( TestWheelCost );
	return 0;
}
