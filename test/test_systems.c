// SPDX-License-Identifier: MIT
// Systems: pieces remember the object and part they came from; a part can carry its own detonator.

#include "test_macros.h"
#include "test_sim.h"

#include <math.h>

enum
{
	TagFrame = 1,
	TagTank = 2,
};

// A dynamic wooden frame (two slabs) with a small metal fuel tank bolted under its middle; `volatileTank` gives the tank
// a detonator of its own
static int AddRig( Sim* s, lpVec3 at, uint32_t userId, bool volatileTank )
{
	lpPartDef parts[3];
	for ( int k = 0; k < 2; ++k )
	{
		parts[k] = lpDefaultPartDef();
		parts[k].halfExtents = (lpVec3){ 0.8f, 0.1f, 0.5f };
		parts[k].transform.p = (lpVec3){ 0.0f, 0.0f, k == 0 ? -0.5f : 0.5f };
		parts[k].material = lp_wood;
		parts[k].system.tag = TagFrame;
		parts[k].system.carries = 0x3;
	}
	parts[2] = lpDefaultPartDef();
	parts[2].halfExtents = (lpVec3){ 0.25f, 0.15f, 0.25f };
	parts[2].transform.p = (lpVec3){ 0.0f, -0.25f, 0.0f };
	parts[2].material = lp_metal;
	parts[2].system.tag = TagTank;
	parts[2].system.sources = 0x1;
	parts[2].system.needs = 0x3; // needs above its lowest source are dropped
	if ( volatileTank )
	{
		parts[2].detonator = (lpDetonatorDef){ 12.0f, 1.5f, 60000.0f, 8.0f };
	}
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (lpPos){ at.x, at.y, at.z };
	def.parts = parts;
	def.partCount = 3;
	def.userId = userId;
	return lpCreateObject( s->world, &def );
}

// Pieces anywhere in the world of that object and part
static int CountPieces( const Sim* s, uint32_t userId, int part, float* volume )
{
	int count = 0;
	float total = 0.0f;
	for ( int i = 0; i < lpWorld_GetPieceCapacity( s->world ); ++i )
	{
		lpPieceInfo info = lpWorld_GetPieceInfo( s->world, i );
		if ( info.body >= 0 && info.userId == userId && ( part < 0 || info.part == part ) )
		{
			count += 1;
			total += info.volume;
		}
	}
	if ( volume != NULL )
	{
		*volume = total;
	}
	return count;
}

static lpImpactDef Blast( lpVec3 at, float radius, float energy )
{
	lpImpactDef im = { 0 };
	im.point = (lpPos){ at.x, at.y, at.z };
	im.radius = radius;
	im.energy = energy;
	im.impulse = 6.0f;
	im.explosion = true;
	return im;
}

// Every piece made from an object keeps its object's id, its part and tag, whatever breaks it; sources keep a share
// of their object's sources by volume, which only shrinks
static int TestPartIdentitySurvivesFracture( void )
{
	Sim s = CreateSim( -1 );
	int body = AddRig( &s, (lpVec3){ 0.0f, 1.0f, 0.0f }, 77u, false );
	const lpBody* b = s.world->bodies.data + body;
	ENSURE( b->pieces.count == 3 );
	for ( int k = 0; k < 3; ++k )
	{
		const lpPiece* p = s.world->pieces.data + b->pieces.data[k];
		lpPieceInfo info = lpWorld_GetPieceInfo( s.world, b->pieces.data[k] );
		ENSURE( info.userId == 77u && info.part == k );
		ENSURE( info.tag == ( k < 2 ? TagFrame : TagTank ) );
		ENSURE( p->carries == ( k < 2 ? 0x3 : 0x1 ) && p->needs == 0 ); // the tank carries what it feeds
		ENSURE( k < 2 || p->sourceShare == 1.0f );
	}
	lpImpactDef im = Blast( (lpVec3){ 0.6f, 1.2f, -0.5f }, 1.2f, 90000.0f );
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 30 );
	ENSURE( lpWorld_Validate( s.world ) );
	int frame = CountPieces( &s, 77u, 0, NULL ) + CountPieces( &s, 77u, 1, NULL );
	int all = CountPieces( &s, 77u, -1, NULL );
	float share = 0.0f;
	for ( int i = 0; i < lpWorld_GetPieceCapacity( s.world ); ++i )
	{
		lpPieceInfo info = lpWorld_GetPieceInfo( s.world, i );
		if ( info.body >= 0 && info.userId == 77u )
		{
			const lpPiece* p = s.world->pieces.data + i;
			ENSURE( info.tag == ( info.part < 2 ? TagFrame : TagTank ) );
			ENSURE( p->carries == ( info.part < 2 ? 0x3 : 0x1 ) );
			share += p->sourceShare;
		}
	}
	printf( "  %d pieces of the object after a blast (%d of the frame), tank share left %.3f\n", all, frame, share );
	ENSURE( frame > 2 && share > 0.0f && share <= 1.0001f );
	DestroySim( &s );
	return 0;
}

// A part with its own detonator goes off alone: its pieces are gone, the rest of the object stays (and takes the
// blast)
static int TestPartDetonatorBlowsOnlyItsPart( void )
{
	Sim s = CreateSim( -1 );
	int body = AddRig( &s, (lpVec3){ 0.0f, 0.8f, 0.0f }, 5u, true );
	Run( &s, 30 );
	ENSURE( BodyArmed( s.world, body, NULL ) );
	// A shot at the tank sets it off
	lpImpactDef im = { 0 };
	im.point = (lpPos){ 0.0f, 0.4f, 0.3f };
	im.radius = 0.35f;
	im.energy = 4000.0f;
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 1 );
	ENSURE( s.world->pendingDestroy.count == 1 );
	Run( &s, 1 );
	int tank = CountPieces( &s, 5u, 2, NULL );
	float frameVolume = 0.0f;
	int frame = CountPieces( &s, 5u, 0, &frameVolume );
	printf( "  after the tank went off: %d tank pieces, %d pieces of the first slab (%.3f m^3)\n", tank, frame, frameVolume );
	ENSURE( tank == 0 && frame > 0 );
	Run( &s, 60 );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// An object's own detonator still takes the whole object, with the blast at its centre of mass
static int TestObjectDetonatorUnchanged( void )
{
	Sim s = CreateSim( -1 );
	lpPartDef part = lpDefaultPartDef();
	part.halfExtents = (lpVec3){ 0.2f, 0.2f, 0.2f };
	part.material = lp_metal;
	lpPartDef parts[2] = { part, part };
	parts[1].transform.p = (lpVec3){ 0.4f, 0.0f, 0.0f };
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (lpPos){ 0.0f, 0.2f, 0.0f };
	def.parts = parts;
	def.partCount = 2;
	def.detonator = (lpDetonatorDef){ 4.5f, 1.8f, 120000.0f, 12.0f };
	int body = lpCreateObject( s.world, &def );
	lpPos center = lpPhys_GetWorldCenter( s.world->phys, s.world->bodies.data[body].id );
	lpImpactDef im = Blast( (lpVec3){ 0.9f, 0.2f, 0.0f }, 1.4f, 80000.0f );
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 1 );
	ENSURE( s.world->pendingDestroy.count == 1 && s.world->nextImpacts.count >= 1 );
	lpImpactDef blast = s.world->nextImpacts.data[0];
	ENSURE( blast.radius == 1.8f && lpLength( lpSubPos( blast.point, center ) ) < 0.05f );
	Run( &s, 1 );
	ENSURE( s.world->bodies.data[body].alive == false );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// A tank torn off its frame stays volatile, and goes off once: its detonator is shared by every piece made from it
static int TestTankTornOffStaysVolatile( void )
{
	Sim s = CreateSim( -1 );
	int body = AddRig( &s, (lpVec3){ 0.0f, 0.8f, 0.0f }, 9u, true );
	Run( &s, 20 );
	// Unbolt the tank: break its bonds and let the body split
	const lpBody* b = s.world->bodies.data + body;
	int tankPiece = -1;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		tankPiece = s.world->pieces.data[b->pieces.data[k]].part == 2 ? b->pieces.data[k] : tankPiece;
	}
	ENSURE( tankPiece >= 0 );
	lpPiece* tp = s.world->pieces.data + tankPiece;
	while ( tp->bonds.count > 0 )
	{
		lpBreakBond( s.world, tp->bonds.data[0] );
	}
	lpMarkDirty( s.world, body );
	Run( &s, 30 );
	int tankBody = s.world->pieces.data[tankPiece].body;
	ENSURE( tankBody >= 0 && tankBody != s.world->pieces.data[s.world->bodies.data[body].pieces.data[0]].body );
	ENSURE( BodyArmed( s.world, tankBody, NULL ) );
	ENSURE( BodyArmed( s.world, body, NULL ) == false ); // the frame is not volatile

	lpWorldTransform xf;
	lpWorld_GetBodyTransform( s.world, tankBody, &xf );
	lpImpactDef im = Blast( (lpVec3){ (float)xf.p.x + 0.8f, (float)xf.p.y, (float)xf.p.z }, 1.2f, 40000.0f );
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 1 );
	ENSURE( s.world->pendingDestroy.count == 1 );
	Run( &s, 2 );
	ENSURE( CountPieces( &s, 9u, 2, NULL ) == 0 );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

enum
{
	Fuel = 0x1, // channel 0
	Power = 0x2, // channel 1
};

// A dynamic row of 0.4 m boxes along x resting on the ground, one object; each part's system from `systems`
static int AddRow( Sim* s, lpVec3 at, const lpPartSystem* systems, int count, uint32_t userId )
{
	lpPartDef parts[16];
	for ( int k = 0; k < count; ++k )
	{
		parts[k] = lpDefaultPartDef();
		parts[k].halfExtents = (lpVec3){ 0.2f, 0.2f, 0.2f };
		parts[k].transform.p = (lpVec3){ 0.4f * (float)k, 0.0f, 0.0f };
		parts[k].material = lp_metal;
		parts[k].system = systems[k];
	}
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (lpPos){ at.x, at.y, at.z };
	def.parts = parts;
	def.partCount = count;
	def.userId = userId;
	return lpCreateObject( s->world, &def );
}

// The piece of that object and part (-1 if none)
static int PieceOf( const Sim* s, uint32_t userId, int part )
{
	for ( int i = 0; i < lpWorld_GetPieceCapacity( s->world ); ++i )
	{
		lpPieceInfo info = lpWorld_GetPieceInfo( s->world, i );
		if ( info.body >= 0 && info.userId == userId && info.part == part )
		{
			return i;
		}
	}
	return -1;
}

static float SupplyAt( const Sim* s, uint32_t userId, int part, int channel )
{
	return lpWorld_GetPieceSupply( s->world, PieceOf( s, userId, part ), channel );
}

// Cut every bond of a piece (it falls off its body at the next step)
static void Unbolt( Sim* s, int piece )
{
	lpPiece* p = s->world->pieces.data + piece;
	int body = p->body;
	while ( p->bonds.count > 0 )
	{
		lpBreakBond( s->world, p->bonds.data[0] );
	}
	lpMarkDirty( s->world, body );
}

// A tank feeds fuel down a pipe to the end; cut the pipe and the far side goes dry in the same step
static int TestSupplyCut( void )
{
	Sim s = CreateSim( -1 );
	lpPartSystem row[7] = { { 0, 0, Fuel, 0 } };
	for ( int k = 1; k < 7; ++k )
	{
		row[k] = (lpPartSystem){ 0, Fuel, 0, 0 };
	}
	AddRow( &s, (lpVec3){ 0.0f, 0.2f, 0.0f }, row, 7, 1u );
	Run( &s, 1 );
	ENSURE( SupplyAt( &s, 1u, 6, 0 ) == 1.0f && SupplyAt( &s, 1u, 6, 1 ) == 0.0f );
	ENSURE( lpWorld_GetPieceInfo( s.world, PieceOf( &s, 1u, 6 ) ).supplied == Fuel );

	// Cut between parts 3 and 4
	lpPiece* p3 = s.world->pieces.data + PieceOf( &s, 1u, 3 );
	int p4 = PieceOf( &s, 1u, 4 );
	for ( int k = 0; k < p3->bonds.count; ++k )
	{
		const lpBond* bond = s.world->bonds.data + p3->bonds.data[k];
		if ( bond->a == p4 || bond->b == p4 )
		{
			lpBreakBond( s.world, p3->bonds.data[k] );
			break;
		}
	}
	lpMarkDirty( s.world, p3->body );
	Run( &s, 1 );
	printf( "  after the cut: part 3 %.2f, part 4 %.2f, the end %.2f\n", SupplyAt( &s, 1u, 3, 0 ), SupplyAt( &s, 1u, 4, 0 ),
			SupplyAt( &s, 1u, 6, 0 ) );
	ENSURE( SupplyAt( &s, 1u, 3, 0 ) == 1.0f && SupplyAt( &s, 1u, 4, 0 ) == 0.0f && SupplyAt( &s, 1u, 6, 0 ) == 0.0f );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// An engine feeds power only while fuel reaches it
static int TestSupplyNeeds( void )
{
	Sim s = CreateSim( -1 );
	lpPartSystem row[4] = { { 0, 0, Fuel, 0 }, { 0, Fuel | Power, 0, 0 }, { 0, Fuel, Power, Fuel }, { 0, Power, 0, 0 } };
	AddRow( &s, (lpVec3){ 0.0f, 0.2f, 0.0f }, row, 4, 2u );
	Run( &s, 1 );
	ENSURE( SupplyAt( &s, 2u, 3, 1 ) == 1.0f );
	Unbolt( &s, PieceOf( &s, 2u, 0 ) ); // the tank comes off
	Run( &s, 1 );
	printf( "  without its tank: fuel at the engine %.2f, power at the end %.2f\n", SupplyAt( &s, 2u, 2, 0 ),
			SupplyAt( &s, 2u, 3, 1 ) );
	ENSURE( SupplyAt( &s, 2u, 2, 0 ) == 0.0f && SupplyAt( &s, 2u, 3, 1 ) == 0.0f );
	DestroySim( &s );
	return 0;
}

// An engine of two equal parts: lose one and half the power is left
static int TestSupplyShare( void )
{
	Sim s = CreateSim( -1 );
	lpPartSystem row[3] = { { 0, 0, Power, 0 }, { 0, 0, Power, 0 }, { 0, Power, 0, 0 } };
	AddRow( &s, (lpVec3){ 0.0f, 0.2f, 0.0f }, row, 3, 3u );
	Run( &s, 1 );
	ENSURE_NEAR( s.world->pieces.data[PieceOf( &s, 3u, 0 )].sourceShare, 0.5f, 1e-4f );
	ENSURE( SupplyAt( &s, 3u, 2, 1 ) == 1.0f );
	Unbolt( &s, PieceOf( &s, 3u, 0 ) );
	Run( &s, 1 );
	printf( "  half an engine: %.3f\n", SupplyAt( &s, 3u, 2, 1 ) );
	ENSURE_NEAR( SupplyAt( &s, 3u, 2, 1 ), 0.5f, 0.004f );
	DestroySim( &s );
	return 0;
}

// A hose between two objects carries fuel; cut it and the far one goes dry
static int TestSupplyOverLink( void )
{
	Sim s = CreateSim( -1 );
	lpPartSystem tank = { 0, 0, Fuel, 0 };
	lpPartSystem drum = { 0, Fuel, 0, 0 };
	int a = AddRow( &s, (lpVec3){ 0.0f, 0.2f, 0.0f }, &tank, 1, 4u );
	int b = AddRow( &s, (lpVec3){ 2.0f, 0.2f, 0.0f }, &drum, 1, 5u );
	lpLinkDef hose = lpDefaultLinkDef( lp_linkRope );
	hose.bodyA = a;
	hose.bodyB = b;
	hose.anchorA = (lpPos){ 0.2f, 0.2f, 0.0f };
	hose.anchorB = (lpPos){ 1.8f, 0.2f, 0.0f };
	hose.carries = Fuel;
	int link = lpCreateLink( s.world, &hose );
	ENSURE( link >= 0 );
	Run( &s, 1 );
	ENSURE( SupplyAt( &s, 5u, 0, 0 ) == 1.0f && lpWorld_GetLinkState( s.world, link ).supplied == Fuel );
	lpDestroyLink( s.world, link );
	Run( &s, 1 );
	ENSURE( SupplyAt( &s, 5u, 0, 0 ) == 0.0f && SupplyAt( &s, 4u, 0, 0 ) == 1.0f );
	DestroySim( &s );
	return 0;
}

// A car whose wheels need power from an engine on its chassis: knock the engine off and it coasts
static int AddPoweredCar( Sim* s, lpVec3 at, uint32_t userId, int* vehicle )
{
	lpPartDef parts[2];
	parts[0] = lpDefaultPartDef();
	parts[0].halfExtents = (lpVec3){ 0.9f, 0.15f, 2.0f };
	parts[0].material = lp_wood;
	parts[0].system.carries = Power;
	parts[1] = lpDefaultPartDef();
	parts[1].halfExtents = (lpVec3){ 0.4f, 0.25f, 0.4f };
	parts[1].transform.p = (lpVec3){ 0.0f, 0.4f, 1.2f };
	parts[1].material = lp_metal;
	parts[1].system.sources = Power;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (lpPos){ at.x, at.y + 0.9f, at.z };
	def.parts = parts;
	def.partCount = 2;
	def.userId = userId;
	int body = lpCreateObject( s->world, &def );
	lpWheelDef wheels[4];
	for ( int i = 0; i < 4; ++i )
	{
		wheels[i] = lpDefaultWheelDef();
		wheels[i].mount = (lpPos){ at.x + ( ( i & 1 ) ? 0.78f : -0.78f ), at.y + 0.75f, at.z + ( ( i & 2 ) ? 1.4f : -1.4f ) };
		wheels[i].driveShare = ( i & 2 ) ? 0.0f : 0.5f;
		wheels[i].driveNeeds = Power;
	}
	lpVehicleDef vd = lpDefaultVehicleDef();
	vd.body = body;
	vd.wheels = wheels;
	vd.wheelCount = 4;
	*vehicle = lpCreateVehicle( s->world, &vd );
	return body;
}

static int TestCutPowerCoasts( void )
{
	Sim s = CreateSim( -1 );
	int car;
	AddPoweredCar( &s, (lpVec3){ 0.0f, 0.0f, -30.0f }, 6u, &car );
	ENSURE( car >= 0 );
	lpVehicleControl go = { 1.0f, 0.0f, 0.0f, false };
	lpWorld_SetVehicleControl( s.world, car, &go );
	Run( &s, 90 );
	lpVehicleState before = lpWorld_GetVehicleState( s.world, car );
	ENSURE( before.power == 1.0f && before.speed > 3.0f );
	Unbolt( &s, PieceOf( &s, 6u, 1 ) );
	Run( &s, 1 );
	float cut = lpWorld_GetVehicleState( s.world, car ).speed;
	Run( &s, 90 );
	lpVehicleState after = lpWorld_GetVehicleState( s.world, car );
	printf( "  powered: %.2f m/s (power %.2f); engine knocked off at %.2f m/s, 1.5 s later %.2f m/s (power %.2f)\n",
			before.speed, before.power, cut, after.speed, after.power );
	ENSURE( after.power == 0.0f && after.speed < cut );
	DestroySim( &s );
	return 0;
}

// Supply under fire is the same at any worker count
static uint64_t SupplyUnderFire( int workers )
{
	Sim s = CreateSimWorkers( -1, workers );
	int car;
	AddPoweredCar( &s, (lpVec3){ 0.0f, 0.0f, 0.0f }, 7u, &car );
	lpPartSystem row[5] = { { 0, 0, Fuel, 0 }, { 0, Fuel | Power, 0, 0 }, { 0, Fuel, Power, Fuel }, { 0, Power, 0, 0 },
							{ 0, Power, 0, 0 } };
	AddRow( &s, (lpVec3){ 4.0f, 0.2f, 0.0f }, row, 5, 8u );
	lpVehicleControl go = { 1.0f, 0.0f, 0.3f, false };
	lpWorld_SetVehicleControl( s.world, car, &go );
	for ( int t = 0; t < 180; ++t )
	{
		if ( t == 40 || t == 100 )
		{
			lpImpactDef im = Blast( t == 40 ? (lpVec3){ 4.8f, 0.3f, 0.5f } : (lpVec3){ 0.5f, 1.2f, 3.0f }, 1.4f, 90000.0f );
			lpWorld_AddImpact( s.world, &im );
		}
		Run( &s, 1 );
	}
	uint64_t h = lpWorld_Hash( s.world );
	DestroySim( &s );
	return h;
}

static int TestSupplyDeterminism( void )
{
	uint64_t a = SupplyUnderFire( 1 );
	uint64_t b = SupplyUnderFire( 4 );
	uint64_t c = SupplyUnderFire( 8 );
	printf( "  hashes %016llx %016llx %016llx\n", (unsigned long long)a, (unsigned long long)b, (unsigned long long)c );
	ENSURE( a == b && a == c );
	return 0;
}

// What a recompute costs: 1000 carriers of all 8 channels in one object, sources in a corner
static int TestSupplyCost( void )
{
	Sim s = CreateSim( -1 );
	static lpPartDef parts[1000];
	for ( int k = 0; k < 1000; ++k )
	{
		parts[k] = lpDefaultPartDef();
		parts[k].halfExtents = (lpVec3){ 0.1f, 0.1f, 0.1f };
		parts[k].transform.p = (lpVec3){ 0.2f * (float)( k % 10 ), 0.2f * (float)( ( k / 10 ) % 10 ), 0.2f * (float)( k / 100 ) };
		parts[k].material = lp_metal;
		parts[k].system.carries = 0xFF;
		parts[k].system.sources = k == 0 ? 0xFF : 0;
	}
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (lpPos){ 0.0f, 0.1f, 0.0f };
	def.parts = parts;
	def.partCount = 1000;
	lpCreateObject( s.world, &def );
	Run( &s, 1 );
	uint64_t ticks = lpGetTicks();
	for ( int r = 0; r < 20; ++r )
	{
		s.world->supplyDirty = true;
		lpUpdateSupply( s.world );
	}
	float us = 1000.0f * lpGetMilliseconds( ticks ) / 20.0f;
	printf( "  1000 carriers, 8 channels: %.1f us per recompute (%d bonds)\n", us, s.world->bondCount );
	ENSURE( lpWorld_GetPieceSupply( s.world, s.world->bodies.data[1].pieces.data[999], 7 ) == 1.0f );
	DestroySim( &s );
	return 0;
}

// ---- pools ----

// The piece of a row's part, and its pool's level (0 to 1)
static float PoolAt( const Sim* s, uint32_t userId, int part )
{
	return lpWorld_GetPiecePool( s->world, PieceOf( s, userId, part ), NULL );
}

static void CutBetween( Sim* s, uint32_t userId, int a, int b )
{
	lpPiece* pa = s->world->pieces.data + PieceOf( s, userId, a );
	int pb = PieceOf( s, userId, b );
	for ( int k = 0; k < pa->bonds.count; ++k )
	{
		const lpBond* bond = s->world->bonds.data + pa->bonds.data[k];
		if ( bond->a == pb || bond->b == pb )
		{
			lpBreakBond( s->world, pa->bonds.data[k] );
			break;
		}
	}
	lpMarkDirty( s->world, pa->body );
}

// A tank with a pool feeds fuel down a line of 7 parts; cut 3 of them off and a leak opens for 3/7 of the pool a second,
// closing over its seal time: the level follows that, and the fuel still reaching the rest weakens once it is below 30%
static int TestPoolLeaksWhenCut( void )
{
	Sim s = CreateSim( -1 );
	lpPartSystem row[7] = { { 0, 0, Fuel, 0, 100.0f, 2.0f } };
	for ( int k = 1; k < 7; ++k )
	{
		row[k] = (lpPartSystem){ 0, Fuel, 0, 0 };
	}
	AddRow( &s, (lpVec3){ 0.0f, 0.2f, 0.0f }, row, 7, 1u );
	Run( &s, 2 );
	ENSURE( PoolAt( &s, 1u, 0 ) == 1.0f && lpWorld_GetStats( s.world ).leakingPools == 0 );
	CutBetween( &s, 1u, 3, 4 );
	Run( &s, 1 ); // the cut: the leak opens
	float leak = 0.0f;
	lpWorld_GetPiecePool( s.world, PieceOf( &s, 1u, 0 ), &leak );
	int updates = 0;
	float worst = 0.0f;
	float dt = 1.0f / 60.0f, q = 1.0f - dt / 2.0f, r0 = 100.0f * 3.0f / 7.0f;
	for ( int n = 1; n <= 360; ++n )
	{
		Run( &s, 1 );
		updates += lpWorld_GetStats( s.world ).supplyUpdates;
		float model = 100.0f - r0 * 2.0f * ( 1.0f - powf( q, (float)n ) );
		worst = fmaxf( worst, fabsf( 100.0f * PoolAt( &s, 1u, 0 ) - model ) );
	}
	float level = PoolAt( &s, 1u, 0 );
	float fuel = SupplyAt( &s, 1u, 3, 0 );
	printf( "  leak %.3f of the pool a second (3/7 = %.3f), level %.3f after 6 s (model %.3f, worst off %.2f%%), fuel down the line %.2f, "
			"%d supply updates\n",
			leak, 3.0f / 7.0f, level, 1.0f - r0 * 2.0f * ( 1.0f - powf( q, 360.0f ) ) / 100.0f, worst, fuel, updates );
	ENSURE_NEAR( leak, 3.0f / 7.0f, 0.01f );
	ENSURE( worst < 1.0f && updates <= 17 );
	ENSURE_NEAR( fuel, level / 0.3f, 0.02f ); // below 30%, the pool pushes in proportion
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// Four carriers in a ring round a pooled tank: cutting one bond of the ring leaves the line whole, and nothing leaks
static int TestPoolRingDoesNotLeak( void )
{
	Sim s = CreateSim( -1 );
	lpPartDef parts[4];
	for ( int k = 0; k < 4; ++k )
	{
		parts[k] = lpDefaultPartDef();
		parts[k].halfExtents = (lpVec3){ 0.2f, 0.2f, 0.2f };
		parts[k].transform.p = (lpVec3){ 0.4f * (float)( k % 2 ), 0.0f, 0.4f * (float)( k / 2 ) };
		parts[k].material = lp_metal;
		parts[k].system = (lpPartSystem){ 0, Fuel, 0, 0 };
	}
	parts[0].system = (lpPartSystem){ 0, 0, Fuel, 0, 100.0f, 0.0f };
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (lpPos){ 0.0f, 0.2f, 0.0f };
	def.parts = parts;
	def.partCount = 4;
	def.userId = 9u;
	lpCreateObject( s.world, &def );
	Run( &s, 2 );
	CutBetween( &s, 9u, 0, 1 ); // the ring still joins them the other way round
	Run( &s, 120 );
	float leak = 0.0f;
	float level = lpWorld_GetPiecePool( s.world, PieceOf( &s, 9u, 0 ), &leak );
	printf( "  a cut ring: level %.4f, leak %.4f\n", level, leak );
	ENSURE( level == 1.0f && leak == 0.0f && SupplyAt( &s, 9u, 1, 0 ) == 1.0f );
	DestroySim( &s );
	return 0;
}

// A heavy round into a stone conduit knocks chips out of it: a small leak, most of the pool kept
static int TestPoolChippedLineLeaksALittle( void )
{
	Sim s = CreateSim( -1 );
	lpPartDef parts[6];
	for ( int k = 0; k < 6; ++k )
	{
		parts[k] = lpDefaultPartDef();
		parts[k].halfExtents = (lpVec3){ 0.5f, 0.3f, 0.3f };
		parts[k].transform.p = (lpVec3){ 1.0f * (float)k, 0.0f, 0.0f };
		parts[k].material = lp_stone;
		parts[k].system = (lpPartSystem){ 0, Fuel, 0, 0 };
	}
	parts[0].system = (lpPartSystem){ 0, 0, Fuel, 0, 100.0f, 1.0f };
	lpObjectDef def = lpDefaultObjectDef();
	def.transform.p = (lpPos){ 0.0f, 0.3f, 0.0f };
	def.parts = parts;
	def.partCount = 6;
	def.userId = 10u;
	lpCreateObject( s.world, &def );
	lpWorld_SettleStructures( s.world );
	Run( &s, 2 );
	lpImpactDef rifle = { 0 };
	rifle.point = (lpPos){ 3.0f, 0.6f, 0.0f };
	rifle.direction = (lpVec3){ 0.0f, -1.0f, 0.0f };
	rifle.radius = 0.4f;
	rifle.energy = 20000.0f; // a heavy round: a rifle's only scratches plate this thick
	int piecesBefore = lpWorld_GetStats( s.world ).pieceCount;
	float reachBefore = s.world->pools.data[0].reach;
	lpWorld_AddImpact( s.world, &rifle );
	Run( &s, 300 );
	float level = PoolAt( &s, 10u, 0 );
	printf( "  a heavy round into the conduit: pieces %d -> %d, reach %.4f -> %.4f, level %.3f\n", piecesBefore,
			lpWorld_GetStats( s.world ).pieceCount, reachBefore, s.world->pools.data[0].reach, level );
	ENSURE( level < 1.0f && level > 0.8f );
	DestroySim( &s );
	return 0;
}

// Pools in the hash: the same cut at 1 and 4 workers leaks the same
static int TestPoolDeterminism( void )
{
	uint64_t hashes[2];
	for ( int k = 0; k < 2; ++k )
	{
		Sim s = CreateSimWorkers( -1, k == 0 ? 1 : 4 );
		lpPartSystem row[5] = { { 0, 0, Fuel, 0, 50.0f, 1.5f } };
		for ( int j = 1; j < 5; ++j )
		{
			row[j] = (lpPartSystem){ 0, Fuel, 0, 0 };
		}
		AddRow( &s, (lpVec3){ 0.0f, 0.2f, 0.0f }, row, 5, 11u );
		Run( &s, 2 );
		CutBetween( &s, 11u, 2, 3 );
		Run( &s, 120 );
		hashes[k] = lpWorld_Hash( s.world );
		DestroySim( &s );
	}
	ENSURE( hashes[0] == hashes[1] );
	return 0;
}

int SystemsTest( void )
{
	RUN_TEST( TestPartIdentitySurvivesFracture );
	RUN_TEST( TestPartDetonatorBlowsOnlyItsPart );
	RUN_TEST( TestObjectDetonatorUnchanged );
	RUN_TEST( TestTankTornOffStaysVolatile );
	RUN_TEST( TestSupplyCut );
	RUN_TEST( TestSupplyNeeds );
	RUN_TEST( TestSupplyShare );
	RUN_TEST( TestSupplyOverLink );
	RUN_TEST( TestCutPowerCoasts );
	RUN_TEST( TestSupplyDeterminism );
	RUN_TEST( TestSupplyCost );
	RUN_TEST( TestPoolLeaksWhenCut );
	RUN_TEST( TestPoolRingDoesNotLeak );
	RUN_TEST( TestPoolChippedLineLeaksALittle );
	RUN_TEST( TestPoolDeterminism );
	return 0;
}
