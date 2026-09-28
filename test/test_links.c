// SPDX-License-Identifier: MIT
// Links: joints between objects that hold what they should, give under too much load, and never outlive their ends.

#include "test_macros.h"
#include "test_sim.h"

#include <math.h>

// A one-part box object; returns its body index. An anchored static box is a fixed support.
static int AddPart( Sim* s, b3Vec3 position, b3Vec3 half, int material, bool isStatic )
{
	lpPartDef part = lpDefaultPartDef();
	part.halfExtents = half;
	part.material = (uint8_t)material;
	part.anchored = isStatic;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = isStatic;
	def.transform.p = (b3Pos){ position.x, position.y, position.z };
	def.parts = &part;
	def.partCount = 1;
	return lpCreateObject( s->world, &def );
}

static float BodyY( const Sim* s, int body )
{
	b3WorldTransform xf;
	lpWorld_GetBodyTransform( s->world, body, &xf );
	return (float)xf.p.y;
}

static int Rope( Sim* s, int bodyA, b3Vec3 a, int bodyB, b3Vec3 b, float maxForce )
{
	lpLinkDef def = lpDefaultLinkDef( lp_linkRope );
	def.bodyA = bodyA;
	def.bodyB = bodyB;
	def.anchorA = (b3Pos){ a.x, a.y, a.z };
	def.anchorB = (b3Pos){ b.x, b.y, b.z };
	def.maxForce = maxForce;
	return lpCreateLink( s->world, &def );
}

// Steps, validating every tick; returns the links broken meanwhile
static int StepValidated( Sim* s, int ticks, bool* valid )
{
	int breaks = 0;
	*valid = true;
	for ( int i = 0; i < ticks; ++i )
	{
		lpWorld_Step( s->world, 1.0f / 60.0f, 4 );
		breaks += lpWorld_GetStats( s->world ).linkBreaks;
		*valid = *valid && lpWorld_Validate( s->world );
	}
	return breaks;
}

// Every type links two blocks; bad definitions are refused
static int TestLinkCreate( void )
{
	Sim s = CreateSim( -1 );
	int a = AddPart( &s, (b3Vec3){ 3.0f, 1.0f, 0.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
	int b = AddPart( &s, (b3Vec3){ 3.45f, 1.0f, 0.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
	for ( int type = 0; type < lp_linkTypeCount; ++type )
	{
		lpLinkDef def = lpDefaultLinkDef( type );
		def.bodyA = a;
		def.bodyB = b;
		def.anchorA = (b3Pos){ 3.2f, 1.0f, 0.0f };
		def.anchorB = (b3Pos){ 3.25f, 1.0f, 0.0f };
		def.axis = (b3Vec3){ 0.0f, 1.0f, 0.0f };
		int link = lpCreateLink( s.world, &def );
		ENSURE( link >= 0 );
		ENSURE( lpWorld_Validate( s.world ) );
		lpLinkState st = lpWorld_GetLinkState( s.world, link );
		ENSURE( st.alive && st.type == type && st.bodyA == a && st.bodyB == b );
		ENSURE( b3Length( b3SubPos( st.pointA, def.anchorA ) ) < 1e-4f );
		lpDestroyLink( s.world, link );
		ENSURE( lpWorld_GetLinkState( s.world, link ).alive == false );
	}

	lpLinkDef def = lpDefaultLinkDef( lp_linkWeld );
	def.anchorA = (b3Pos){ 3.2f, 1.0f, 0.0f };
	def.bodyA = a;
	def.bodyB = a;
	ENSURE( lpCreateLink( s.world, &def ) == -1 ); // one body at both ends
	def.bodyA = -1;
	def.bodyB = -1;
	ENSURE( lpCreateLink( s.world, &def ) == -1 ); // the world at both ends
	def.bodyB = b;
	def.anchorA = (b3Pos){ 13.0f, 1.0f, 0.0f };
	ENSURE( lpCreateLink( s.world, &def ) == -1 ); // nowhere near b
	lpConvertToGhost( s.world, b );
	def.anchorA = (b3Pos){ 3.25f, 1.0f, 0.0f };
	ENSURE( lpCreateLink( s.world, &def ) == -1 ); // a ghost cannot hold a joint
	ENSURE( s.world->linkCount == 0 );
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// A sign on two ropes from a fixed beam hangs still; each rope carries half its weight
static int TestSignHangs( void )
{
	Sim s = CreateSim( -1 );
	int beam = AddPart( &s, (b3Vec3){ 0.0f, 4.0f, 0.0f }, (b3Vec3){ 1.0f, 0.1f, 0.1f }, lp_wood, true );
	int sign = AddPart( &s, (b3Vec3){ 0.0f, 2.5f, 0.0f }, (b3Vec3){ 0.6f, 0.3f, 0.025f }, lp_wood, false );
	int left = Rope( &s, beam, (b3Vec3){ -0.5f, 3.9f, 0.0f }, sign, (b3Vec3){ -0.5f, 2.8f, 0.0f }, 1000.0f );
	int right = Rope( &s, beam, (b3Vec3){ 0.5f, 3.9f, 0.0f }, sign, (b3Vec3){ 0.5f, 2.8f, 0.0f }, 1000.0f );
	ENSURE( left >= 0 && right >= 0 );
	float weight = 10.0f * b3Body_GetMass( s.world->bodies.data[sign].id );
	bool valid;
	int breaks = StepValidated( &s, 300, &valid );
	lpLinkState st = lpWorld_GetLinkState( s.world, left );
	printf( "  sign weighs %.0f N; after 300 steps it is at y %.3f, rope utilization %.3f (half the weight: %.3f), %d breaks\n",
			(double)weight, (double)BodyY( &s, sign ), (double)st.utilization, (double)( 0.5f * weight / 1000.0f ), breaks );
	ENSURE( valid );
	ENSURE( breaks == 0 );
	ENSURE( st.alive && lpWorld_GetLinkState( s.world, right ).alive );
	ENSURE_NEAR( BodyY( &s, sign ), 2.5f, 0.05f );
	ENSURE_NEAR( st.utilization, 0.5f * weight / 1000.0f, 0.05f * weight / 1000.0f );
	ENSURE( s.world->bodies.data[sign].kind == lp_kindDebris ); // linked: never frozen
	DestroySim( &s );
	return 0;
}

// A block welded under a fixed point: steps until the weld gives, or -1 if it held
static int WeldBreakStep( float overload, int ticks )
{
	Sim s = CreateSim( -1 );
	int block = AddPart( &s, (b3Vec3){ 0.0f, 3.0f, 0.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
	float weight = 10.0f * b3Body_GetMass( s.world->bodies.data[block].id );
	lpLinkDef def = lpDefaultLinkDef( lp_linkWeld );
	def.bodyA = -1;
	def.bodyB = block;
	def.anchorA = (b3Pos){ 0.0f, 3.2f, 0.0f };
	def.maxForce = weight / overload;
	def.maxTorque = 0.0f;
	int link = lpCreateLink( s.world, &def );
	int broke = -1;
	for ( int tick = 0; tick < ticks && broke < 0; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		if ( lpWorld_Validate( s.world ) == false )
		{
			broke = -2;
		}
		else if ( lpWorld_GetLinkState( s.world, link ).alive == false )
		{
			broke = tick + 1;
		}
	}
	DestroySim( &s );
	return broke;
}

// Under its limit a weld holds; a little over, it creaks and gives after a while; far over, it snaps at once
static int TestWeldOverload( void )
{
	int holds = WeldBreakStep( 0.5f, 300 );
	int creeps = WeldBreakStep( 1.5f, 300 );
	int snaps = WeldBreakStep( 3.5f, 300 );
	printf( "  weld at half its limit: %s; at 1.5 times: gave at step %d; at 3.5 times: snapped at step %d\n",
			holds < 0 ? "held" : "gave", creeps, snaps );
	ENSURE( holds == -1 );
	ENSURE( creeps > 6 && creeps <= 90 );
	ENSURE( snaps > 0 && snaps <= 6 );
	return 0;
}

// A door on a vertical hinge, pushed open, swings to its limit and stays on the hinge
static int TestHingeDoorSwings( void )
{
	Sim s = CreateSim( -1 );
	int post = AddPart( &s, (b3Vec3){ 0.0f, 1.1f, 0.0f }, (b3Vec3){ 0.1f, 1.1f, 0.1f }, lp_wood, true );
	int door = AddPart( &s, (b3Vec3){ 0.65f, 1.1f, 0.0f }, (b3Vec3){ 0.5f, 1.0f, 0.025f }, lp_wood, false );
	lpLinkDef def = lpDefaultLinkDef( lp_linkHinge );
	def.bodyA = post;
	def.bodyB = door;
	def.anchorA = (b3Pos){ 0.125f, 1.1f, 0.0f };
	def.axis = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	def.lowerAngle = -1.2f;
	def.upperAngle = 1.2f;
	int hinge = lpCreateLink( s.world, &def );
	ENSURE( hinge >= 0 );
	b3BodyId id = s.world->bodies.data[door].id;
	b3Body_SetAngularVelocity( id, (b3Vec3){ 0.0f, 3.0f, 0.0f } ); // swinging about the hinge line
	b3Body_SetLinearVelocity( id, (b3Vec3){ 0.0f, 0.0f, -1.575f } );

	float widest = 0.0f, drift = 0.0f;
	bool valid = true;
	for ( int tick = 0; tick < 120; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		valid = valid && lpWorld_Validate( s.world );
		b3WorldTransform xf = b3Body_GetTransform( id );
		b3Vec3 across = b3RotateVector( xf.q, (b3Vec3){ 1.0f, 0.0f, 0.0f } );
		widest = fmaxf( widest, fabsf( atan2f( -across.z, across.x ) ) );
		b3Pos edge = b3TransformWorldPoint( xf, (b3Vec3){ -0.525f, 0.0f, 0.0f } );
		drift = fmaxf( drift, b3Length( b3SubPos( edge, def.anchorA ) ) );
	}
	printf( "  door swung to %.2f rad (limit 1.2), hinge drift %.4f m\n", (double)widest, (double)drift );
	ENSURE( valid );
	ENSURE( lpWorld_GetLinkState( s.world, hinge ).alive );
	ENSURE( widest > 1.0f && widest < 1.25f );
	ENSURE( drift < 0.02f );
	DestroySim( &s );
	return 0;
}

// With no room in the debris budgets, a hanging block stays full physics; a loose one beside it is demoted
static int TestLinkedNeverDemoted( void )
{
	lpWorldDef ld = lpDefaultWorldDef();
	ld.maxFullDebris = 0;
	ld.maxLightDebris = 0;
	Sim s = CreateSimDef( ld, -1 );
	int beam = AddPart( &s, (b3Vec3){ 0.0f, 4.0f, 0.0f }, (b3Vec3){ 0.5f, 0.1f, 0.1f }, lp_wood, true );
	int hung = AddPart( &s, (b3Vec3){ 0.0f, 2.5f, 0.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
	int loose = AddPart( &s, (b3Vec3){ 3.0f, 0.2f, 0.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
	ENSURE( Rope( &s, beam, (b3Vec3){ 0.0f, 3.9f, 0.0f }, hung, (b3Vec3){ 0.0f, 2.7f, 0.0f }, 0.0f ) >= 0 );
	bool valid;
	StepValidated( &s, 600, &valid );
	const lpBody* h = s.world->bodies.data + hung;
	const lpBody* l = s.world->bodies.data + loose;
	ENSURE( valid );
	ENSURE( h->alive && h->kind == lp_kindDebris && h->tier == lp_tierFull );
	ENSURE( l->alive == false || l->kind != lp_kindDebris || l->tier != lp_tierFull );
	DestroySim( &s );
	return 0;
}

// A body lost below the kill depth takes its links with it
static int TestKillBreaksLink( void )
{
	lpWorldDef ld = lpDefaultWorldDef();
	ld.killDepth = 1.0f;
	Sim s = CreateSimDef( ld, -1 );
	int beam = AddPart( &s, (b3Vec3){ 0.0f, 6.0f, 0.0f }, (b3Vec3){ 0.5f, 0.1f, 0.1f }, lp_wood, true );
	int block = AddPart( &s, (b3Vec3){ 0.0f, 5.0f, 0.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
	lpLinkDef def = lpDefaultLinkDef( lp_linkRope );
	def.bodyA = beam;
	def.bodyB = block;
	def.anchorA = (b3Pos){ 0.0f, 5.9f, 0.0f };
	def.anchorB = (b3Pos){ 0.0f, 5.2f, 0.0f };
	def.length = 10.0f; // slack all the way to the ground
	int rope = lpCreateLink( s.world, &def );
	ENSURE( rope >= 0 );
	bool valid;
	StepValidated( &s, 120, &valid );
	ENSURE( valid );
	ENSURE( s.world->bodies.data[block].alive == false );
	ENSURE( lpWorld_GetLinkState( s.world, rope ).alive == false );
	ENSURE( s.world->linkCount == 0 );
	DestroySim( &s );
	return 0;
}

// A little yard of linked things, knocked about by a blast
static void BuildAssembly( Sim* s )
{
	int beam = AddPart( s, (b3Vec3){ 0.0f, 4.0f, -3.0f }, (b3Vec3){ 2.0f, 0.1f, 0.1f }, lp_wood, true );
	int sign = AddPart( s, (b3Vec3){ -1.0f, 2.5f, -3.0f }, (b3Vec3){ 0.6f, 0.3f, 0.025f }, lp_wood, false );
	Rope( s, beam, (b3Vec3){ -1.5f, 3.9f, -3.0f }, sign, (b3Vec3){ -1.5f, 2.8f, -3.0f }, 2000.0f );
	Rope( s, beam, (b3Vec3){ -0.5f, 3.9f, -3.0f }, sign, (b3Vec3){ -0.5f, 2.8f, -3.0f }, 2000.0f );
	// A chain of four blocks on soft welds, hung from the beam by a ball joint
	int above = beam;
	for ( int i = 0; i < 4; ++i )
	{
		float y = 3.65f - 0.5f * (float)i;
		int block = AddPart( s, (b3Vec3){ 1.2f, y, -3.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
		lpLinkDef def = lpDefaultLinkDef( i == 0 ? lp_linkBall : lp_linkWeld );
		def.bodyA = above;
		def.bodyB = block;
		def.anchorA = (b3Pos){ 1.2f, y + 0.25f, -3.0f };
		def.hertz = 5.0f;
		def.dampingRatio = 0.7f;
		lpCreateLink( s->world, &def );
		above = block;
	}
}

static int RunAssembly( int workers, int ticks, uint64_t* hashes )
{
	Sim s = CreateSimWorkers( -1, workers );
	BuildAssembly( &s );
	for ( int tick = 0; tick < ticks; ++tick )
	{
		if ( tick == 20 )
		{
			lpImpactDef im = { 0 };
			im.point = (b3Pos){ 0.6f, 2.8f, -2.6f };
			im.radius = 1.4f;
			im.energy = 80000.0f;
			im.impulse = 12.0f;
			im.explosion = true;
			lpWorld_AddImpact( s.world, &im );
		}
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		hashes[tick] = lpWorld_Hash( s.world );
	}
	int links = s.world->linkCount;
	DestroySim( &s );
	return links;
}

// The same blast on the same linked things gives the same result on any number of workers
static int TestLinkDeterminism( void )
{
	enum
	{
		ticks = 200
	};
	static uint64_t a[ticks], b[ticks];
	int linksA = RunAssembly( 1, ticks, a );
	int linksB = RunAssembly( 4, ticks, b );
	for ( int i = 0; i < ticks; ++i )
	{
		if ( a[i] != b[i] )
		{
			printf( "  4 workers diverged from 1 worker at tick %d\n", i );
			return 1;
		}
	}
	printf( "  %d of 6 links left after the blast, final hash %016llx\n", linksA, (unsigned long long)a[ticks - 1] );
	ENSURE( linksA == linksB );
	return 0;
}

int LinkTest( void )
{
	RUN_TEST( TestLinkCreate );
	RUN_TEST( TestSignHangs );
	RUN_TEST( TestWeldOverload );
	RUN_TEST( TestHingeDoorSwings );
	RUN_TEST( TestLinkedNeverDemoted );
	RUN_TEST( TestKillBreaksLink );
	RUN_TEST( TestLinkDeterminism );
	return 0;
}
