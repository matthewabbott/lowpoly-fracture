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
static int AddRig( Sim* s, b3Vec3 at, uint32_t userId, bool volatileTank )
{
	lpPartDef parts[3];
	for ( int k = 0; k < 2; ++k )
	{
		parts[k] = lpDefaultPartDef();
		parts[k].halfExtents = (b3Vec3){ 0.8f, 0.1f, 0.5f };
		parts[k].transform.p = (b3Vec3){ 0.0f, 0.0f, k == 0 ? -0.5f : 0.5f };
		parts[k].material = lp_wood;
		parts[k].system.tag = TagFrame;
		parts[k].system.carries = 0x3;
	}
	parts[2] = lpDefaultPartDef();
	parts[2].halfExtents = (b3Vec3){ 0.25f, 0.15f, 0.25f };
	parts[2].transform.p = (b3Vec3){ 0.0f, -0.25f, 0.0f };
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
	def.transform.p = (b3Pos){ at.x, at.y, at.z };
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

static lpImpactDef Blast( b3Vec3 at, float radius, float energy )
{
	lpImpactDef im = { 0 };
	im.point = (b3Pos){ at.x, at.y, at.z };
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
	int body = AddRig( &s, (b3Vec3){ 0.0f, 1.0f, 0.0f }, 77u, false );
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
	lpImpactDef im = Blast( (b3Vec3){ 0.6f, 1.2f, -0.5f }, 1.2f, 90000.0f );
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
	int body = AddRig( &s, (b3Vec3){ 0.0f, 0.8f, 0.0f }, 5u, true );
	Run( &s, 30 );
	ENSURE( BodyArmed( s.world, body, NULL ) );
	// A shot at the tank sets it off
	lpImpactDef im = { 0 };
	im.point = (b3Pos){ 0.0f, 0.4f, 0.3f };
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
	part.halfExtents = (b3Vec3){ 0.2f, 0.2f, 0.2f };
	part.material = lp_metal;
	lpPartDef parts[2] = { part, part };
	parts[1].transform.p = (b3Vec3){ 0.4f, 0.0f, 0.0f };
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (b3Pos){ 0.0f, 0.2f, 0.0f };
	def.parts = parts;
	def.partCount = 2;
	def.detonator = (lpDetonatorDef){ 4.5f, 1.8f, 120000.0f, 12.0f };
	int body = lpCreateObject( s.world, &def );
	b3Pos center = b3Body_GetWorldCenter( s.world->bodies.data[body].id );
	lpImpactDef im = Blast( (b3Vec3){ 0.9f, 0.2f, 0.0f }, 1.4f, 80000.0f );
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 1 );
	ENSURE( s.world->pendingDestroy.count == 1 && s.world->nextImpacts.count >= 1 );
	lpImpactDef blast = s.world->nextImpacts.data[0];
	ENSURE( blast.radius == 1.8f && b3Length( b3SubPos( blast.point, center ) ) < 0.05f );
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
	int body = AddRig( &s, (b3Vec3){ 0.0f, 0.8f, 0.0f }, 9u, true );
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

	b3WorldTransform xf;
	lpWorld_GetBodyTransform( s.world, tankBody, &xf );
	lpImpactDef im = Blast( (b3Vec3){ (float)xf.p.x + 0.8f, (float)xf.p.y, (float)xf.p.z }, 1.2f, 40000.0f );
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 1 );
	ENSURE( s.world->pendingDestroy.count == 1 );
	Run( &s, 2 );
	ENSURE( CountPieces( &s, 9u, 2, NULL ) == 0 );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

int SystemsTest( void )
{
	RUN_TEST( TestPartIdentitySurvivesFracture );
	RUN_TEST( TestPartDetonatorBlowsOnlyItsPart );
	RUN_TEST( TestObjectDetonatorUnchanged );
	RUN_TEST( TestTankTornOffStaysVolatile );
	return 0;
}
