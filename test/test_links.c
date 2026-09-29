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

// World height of a piece's centroid (a body's origin need not be anywhere near its pieces)
static float PieceY( const Sim* s, int piece )
{
	return (float)lpWorld_ToWorldFrame( s->world, piece, s->world->pieces.data[piece].shape->centroid ).y;
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

// Every type links two blocks (but wheels, which only vehicles make); bad definitions are refused
static int TestLinkCreate( void )
{
	Sim s = CreateSim( -1 );
	int a = AddPart( &s, (b3Vec3){ 3.0f, 1.0f, 0.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
	int b = AddPart( &s, (b3Vec3){ 3.45f, 1.0f, 0.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
	for ( int type = 0; type < lp_linkTypeCount; ++type )
	{
		lpLinkDef def = lpDefaultLinkDef( type );
		if ( type == lp_linkWheel )
		{
			def.bodyA = a;
			def.anchorA = (b3Pos){ 3.2f, 1.0f, 0.0f };
			ENSURE( lpCreateLink( s.world, &def ) == -1 ); // wheels are made by lpCreateVehicle
			continue;
		}
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

// A dynamic object of two bonded stone boxes; returns its body index
static int AddPair( Sim* s, b3Vec3 centerA, float halfA, b3Vec3 centerB, float halfB )
{
	lpPartDef parts[2];
	parts[0] = lpDefaultPartDef();
	parts[0].halfExtents = (b3Vec3){ halfA, halfA, halfA };
	parts[0].transform.p = centerA;
	parts[1] = lpDefaultPartDef();
	parts[1].halfExtents = (b3Vec3){ halfB, halfB, halfB };
	parts[1].transform.p = centerB;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.parts = parts;
	def.partCount = 2;
	return lpCreateObject( s->world, &def );
}

// Break every bond of the smallest piece of a body, so it splits off at the next step; returns that piece
static int SplitOffSmallest( Sim* s, int bodyIndex )
{
	const lpBody* b = s->world->bodies.data + bodyIndex;
	int smallest = b->pieces.data[0];
	for ( int k = 1; k < b->pieces.count; ++k )
	{
		int pi = b->pieces.data[k];
		smallest = s->world->pieces.data[pi].shape->volume < s->world->pieces.data[smallest].shape->volume ? pi : smallest;
	}
	lpPiece* p = s->world->pieces.data + smallest;
	while ( p->bonds.count > 0 )
	{
		lpBreakBond( s->world, p->bonds.data[p->bonds.count - 1] );
	}
	lpMarkDirty( s->world, bodyIndex );
	return smallest;
}

// A small block bonded on top of a big one hangs by a rope on the small one. When the bond breaks, the small block
// splits off onto a new body; the rope's joint is rebuilt there and still holds it while the big block falls.
static int TestLinkSurvivesSplit( void )
{
	Sim s = CreateSim( -1 );
	int beam = AddPart( &s, (b3Vec3){ 0.0f, 5.0f, 0.0f }, (b3Vec3){ 0.5f, 0.1f, 0.1f }, lp_wood, true );
	int pair = AddPair( &s, (b3Vec3){ 0.0f, 3.0f, 0.0f }, 0.15f, (b3Vec3){ 0.0f, 2.45f, 0.0f }, 0.4f );
	int rope = Rope( &s, beam, (b3Vec3){ 0.0f, 4.9f, 0.0f }, pair, (b3Vec3){ 0.0f, 3.15f, 0.0f }, 0.0f );
	ENSURE( rope >= 0 );
	bool valid;
	StepValidated( &s, 30, &valid );
	ENSURE( valid );
	int small = SplitOffSmallest( &s, pair );
	int big = s.world->bodies.data[pair].pieces.data[0] == small ? s.world->bodies.data[pair].pieces.data[1]
																	: s.world->bodies.data[pair].pieces.data[0];
	int rebuilds = 0;
	for ( int tick = 0; tick < 90; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		rebuilds += lpWorld_GetStats( s.world ).linkRebuilds;
		valid = valid && lpWorld_Validate( s.world );
	}
	lpLinkState st = lpWorld_GetLinkState( s.world, rope );
	float smallY = PieceY( &s, small ), bigY = PieceY( &s, big );
	printf( "  rope rebuilt %d time(s); the small block hangs at y %.3f, the big one lies at y %.3f\n", rebuilds,
			(double)smallY, (double)bigY );
	ENSURE( valid );
	ENSURE( st.alive && rebuilds >= 1 );
	ENSURE( st.bodyB != pair && st.bodyB == s.world->pieces.data[small].body );
	ENSURE_NEAR( smallY, 3.0f, 0.05f );
	ENSURE( bigY < 1.0f );
	DestroySim( &s );
	return 0;
}

// A stone beam on two ropes, one at each end, immune to blast damage themselves (thick enough to fracture: thinner
// pieces snap under stress instead)
static int HungPlank( Sim* s, int* left, int* right )
{
	int beam = AddPart( s, (b3Vec3){ 0.0f, 5.0f, 0.0f }, (b3Vec3){ 1.5f, 0.1f, 0.1f }, lp_wood, true );
	int plank = AddPart( s, (b3Vec3){ 0.0f, 3.0f, 0.0f }, (b3Vec3){ 1.2f, 0.3f, 0.3f }, lp_stone, false );
	for ( int k = 0; k < 2; ++k )
	{
		float x = k == 0 ? -1.1f : 1.1f;
		lpLinkDef def = lpDefaultLinkDef( lp_linkRope );
		def.bodyA = beam;
		def.bodyB = plank;
		def.anchorA = (b3Pos){ x, 4.9f, 0.0f };
		def.anchorB = (b3Pos){ x, 3.3f, 0.0f };
		def.strength = 0.0f;
		def.maxForce = 0.0f; // two tonnes of stone
		*( k == 0 ? left : right ) = lpCreateLink( s->world, &def );
	}
	return plank;
}

static int Shoot( Sim* s, b3Vec3 point, float radius, float energy )
{
	lpImpactDef im = { 0 };
	im.point = (b3Pos){ point.x, point.y, point.z };
	im.direction = (b3Vec3){ 0.0f, 0.0f, -1.0f };
	im.radius = radius;
	im.energy = energy;
	im.impulse = 5.0f;
	lpWorld_AddImpact( s->world, &im );
	return 0;
}

// Shot between its ropes, the plank fractures; each rope's end moves to the cell that holds its anchor
static int TestLinkRehomedOnFracture( void )
{
	Sim s = CreateSim( -1 );
	int left, right;
	HungPlank( &s, &left, &right );
	bool valid;
	StepValidated( &s, 20, &valid );
	lpLinkEnd before = s.world->links.data[left].ends[1];
	Shoot( &s, (b3Vec3){ 0.3f, 3.0f, 0.3f }, 0.35f, 4000.0f );
	int breaks = StepValidated( &s, 60, &valid );
	ENSURE( valid );
	ENSURE( breaks == 0 );
	for ( int k = 0; k < 2; ++k )
	{
		const lpLink* l = s.world->links.data + ( k == 0 ? left : right );
		const lpPiece* p = s.world->pieces.data + l->ends[1].piece;
		float d = lpShape_SignedDistance( p->shape, l->ends[1].frame.p );
		printf( "  %s rope: end on piece %d (generation %u), anchor %.4f m from it\n", k == 0 ? "left" : "right",
				l->ends[1].piece, p->generation, (double)d );
		ENSURE( l->alive );
		ENSURE( d < 0.01f );
	}
	lpLinkEnd after = s.world->links.data[left].ends[1];
	ENSURE( after.piece != before.piece || after.generation != before.generation ); // it really moved to a new piece
	DestroySim( &s );
	return 0;
}

// A blast at one rope's anchor blows that end of the plank out: that rope breaks, and the plank swings on the other
static int TestLinkBreaksWhenAnchorEjected( void )
{
	Sim s = CreateSim( -1 );
	int left, right;
	int plank = HungPlank( &s, &left, &right );
	bool valid;
	StepValidated( &s, 20, &valid );
	Shoot( &s, (b3Vec3){ 1.1f, 3.3f, 0.0f }, 0.6f, 30000.0f );
	StepValidated( &s, 90, &valid );
	lpLinkState l = lpWorld_GetLinkState( s.world, left );
	printf( "  left rope %s, right rope %s; the plank swings from it, its end at y %.2f\n", l.alive ? "holds" : "broke",
			lpWorld_GetLinkState( s.world, right ).alive ? "holds" : "broke", (double)l.pointB.y );
	ENSURE( valid );
	ENSURE( lpWorld_GetLinkState( s.world, right ).alive == false );
	ENSURE( l.alive );
	ENSURE( b3Length( b3SubPos( l.pointB, l.pointA ) ) < 1.6f + 0.02f ); // held within the rope's length
	ENSURE( l.pointB.y > 2.5f );
	(void)plank;
	DestroySim( &s );
	return 0;
}

// A body made a ghost loses its links at once
static int TestLinkToGhostBreaks( void )
{
	Sim s = CreateSim( -1 );
	int beam = AddPart( &s, (b3Vec3){ 0.0f, 4.0f, 0.0f }, (b3Vec3){ 0.5f, 0.1f, 0.1f }, lp_wood, true );
	int block = AddPart( &s, (b3Vec3){ 0.0f, 2.5f, 0.0f }, (b3Vec3){ 0.1f, 0.1f, 0.1f }, lp_stone, false );
	int rope = Rope( &s, beam, (b3Vec3){ 0.0f, 3.9f, 0.0f }, block, (b3Vec3){ 0.0f, 2.6f, 0.0f }, 0.0f );
	Run( &s, 5 );
	lpConvertToGhost( s.world, block );
	ENSURE( lpWorld_GetLinkState( s.world, rope ).alive == false );
	ENSURE( s.world->linkCount == 0 );
	ENSURE( lpWorld_Validate( s.world ) );
	bool valid;
	StepValidated( &s, 10, &valid );
	ENSURE( valid );
	DestroySim( &s );
	return 0;
}

// A small block bonded to a larger one is welded to a heavy stone. When the small block splits off, it alone would
// hang on a 1200 kg stone by the weld: under 2% of its mass, it tears off instead.
static int TestTinyEndTears( void )
{
	Sim s = CreateSim( -1 );
	int heavy = AddPart( &s, (b3Vec3){ 0.0f, 0.4f, 0.0f }, (b3Vec3){ 0.4f, 0.4f, 0.4f }, lp_stone, false );
	int pair = AddPair( &s, (b3Vec3){ 0.5f, 0.4f, 0.0f }, 0.1f, (b3Vec3){ 0.8f, 0.4f, 0.0f }, 0.2f );
	lpLinkDef def = lpDefaultLinkDef( lp_linkWeld );
	def.bodyA = heavy;
	def.bodyB = pair;
	def.anchorA = (b3Pos){ 0.4f, 0.4f, 0.0f };
	int weld = lpCreateLink( s.world, &def );
	ENSURE( weld >= 0 );
	bool valid;
	int breaks = StepValidated( &s, 10, &valid );
	ENSURE( valid && breaks == 0 );
	SplitOffSmallest( &s, pair );
	breaks = StepValidated( &s, 2, &valid );
	printf( "  after the split: weld %s\n", lpWorld_GetLinkState( s.world, weld ).alive ? "holds" : "tore" );
	ENSURE( valid );
	ENSURE( breaks == 1 );
	ENSURE( lpWorld_GetLinkState( s.world, weld ).alive == false );
	DestroySim( &s );
	return 0;
}

// A rifle round (a ray, then its impact where it hits)
static lpRayHit Fire( Sim* s, b3Vec3 from, b3Vec3 to )
{
	b3Pos origin = { from.x, from.y, from.z };
	lpRayHit hit = lpWorld_CastRay( s->world, origin, b3Sub( to, from ) );
	if ( hit.hit )
	{
		lpImpactDef im = { 0 };
		im.point = hit.point;
		im.direction = b3Normalize( b3Sub( to, from ) );
		im.radius = 0.35f;
		im.energy = 4000.0f;
		im.impulse = 20.0f;
		lpWorld_AddImpact( s->world, &im );
	}
	return hit;
}

// The rifle can hit a rope: one round cuts it, the sign swings on the other, and a second round drops it
static int TestRopeShotSnaps( void )
{
	Sim s = CreateSim( -1 );
	int beam = AddPart( &s, (b3Vec3){ 0.0f, 4.0f, 0.0f }, (b3Vec3){ 1.0f, 0.1f, 0.1f }, lp_wood, true );
	int sign = AddPart( &s, (b3Vec3){ 0.0f, 2.5f, 0.0f }, (b3Vec3){ 0.6f, 0.3f, 0.025f }, lp_wood, false );
	int left = Rope( &s, beam, (b3Vec3){ -0.5f, 3.9f, 0.0f }, sign, (b3Vec3){ -0.5f, 2.8f, 0.0f }, 1000.0f );
	int right = Rope( &s, beam, (b3Vec3){ 0.5f, 3.9f, 0.0f }, sign, (b3Vec3){ 0.5f, 2.8f, 0.0f }, 1000.0f );
	bool valid;
	StepValidated( &s, 10, &valid );

	lpRayHit hit = Fire( &s, (b3Vec3){ -0.5f, 3.35f, 5.0f }, (b3Vec3){ -0.5f, 3.35f, -5.0f } );
	ENSURE( hit.hit && hit.link == left && hit.piece == -1 );
	StepValidated( &s, 30, &valid );
	ENSURE( valid );
	ENSURE( lpWorld_GetLinkState( s.world, left ).alive == false );
	ENSURE( lpWorld_GetLinkState( s.world, right ).alive );
	ENSURE( PieceY( &s, s.world->bodies.data[sign].pieces.data[0] ) > 1.5f ); // swinging on the right rope

	lpLinkState r = lpWorld_GetLinkState( s.world, right );
	b3Vec3 mid = b3MulSV( 0.5f, b3Add( b3ToVec3( r.pointA ), b3ToVec3( r.pointB ) ) );
	hit = Fire( &s, (b3Vec3){ mid.x, mid.y, 5.0f }, (b3Vec3){ mid.x, mid.y, -5.0f } );
	ENSURE( hit.link == right );
	StepValidated( &s, 90, &valid );
	float y = PieceY( &s, s.world->bodies.data[sign].pieces.data[0] );
	printf( "  both ropes shot through; the sign fell to y %.2f\n", (double)y );
	ENSURE( valid );
	ENSURE( s.world->linkCount == 0 );
	ENSURE( y < 1.0f );
	DestroySim( &s );
	return 0;
}

// A metal door on its hinge (metal does not fracture, so only damage can break it): a rifle round 2 m away leaves
// the hinge alone, a grenade next to it tears it off
static int TestBlastBreaksHinge( void )
{
	Sim s = CreateSim( -1 );
	int post = AddPart( &s, (b3Vec3){ 0.0f, 1.1f, 0.0f }, (b3Vec3){ 0.1f, 1.1f, 0.1f }, lp_metal, true );
	int door = AddPart( &s, (b3Vec3){ 0.65f, 1.1f, 0.0f }, (b3Vec3){ 0.5f, 1.0f, 0.025f }, lp_metal, false );
	lpLinkDef def = lpDefaultLinkDef( lp_linkHinge );
	def.bodyA = post;
	def.bodyB = door;
	def.anchorA = (b3Pos){ 0.125f, 1.1f, 0.0f };
	def.axis = (b3Vec3){ 0.0f, 1.0f, 0.0f };
	def.maxForce = 0.0f; // no load limit: only damage breaks it here
	def.maxTorque = 0.0f;
	int hinge = lpCreateLink( s.world, &def );
	bool valid;
	StepValidated( &s, 10, &valid );
	Shoot( &s, (b3Vec3){ 2.125f, 1.1f, 0.0f }, 0.35f, 4000.0f );
	StepValidated( &s, 10, &valid );
	float afterRifle = lpWorld_GetLinkState( s.world, hinge ).health;
	ENSURE( lpWorld_GetLinkState( s.world, hinge ).alive && afterRifle == def.strength );

	lpImpactDef im = { 0 };
	im.point = (b3Pos){ 0.425f, 1.1f, 0.3f };
	im.radius = 1.4f;
	im.energy = 80000.0f;
	im.impulse = 12.0f;
	im.explosion = true;
	lpWorld_AddImpact( s.world, &im );
	StepValidated( &s, 10, &valid );
	printf( "  hinge after a rifle round 2 m away: health %.0f of %.0f; after a grenade 0.42 m away: %s\n",
			(double)afterRifle, (double)def.strength, lpWorld_GetLinkState( s.world, hinge ).alive ? "holds" : "torn off" );
	ENSURE( valid );
	ENSURE( lpWorld_GetLinkState( s.world, hinge ).alive == false );
	(void)door;
	DestroySim( &s );
	return 0;
}

// A grenade among four stone blocks, with only one fracture job per step: the rest of its work is deferred to later
// steps, but a rope passing by takes its damage once
static int TestDeferredNoDoubleDamage( void )
{
	lpWorldDef ld = lpDefaultWorldDef();
	ld.maxFractureJobsPerStep = 1;
	Sim s = CreateSimDef( ld, -1 );
	for ( int k = 0; k < 4; ++k )
	{
		float x = k == 0 ? -0.7f : k == 1 ? 0.7f : 0.0f;
		float z = k == 2 ? -0.7f : k == 3 ? 0.7f : 0.0f;
		AddPart( &s, (b3Vec3){ x, 0.3f, z }, (b3Vec3){ 0.3f, 0.3f, 0.3f }, lp_stone, false );
	}
	int postA = AddPart( &s, (b3Vec3){ -2.0f, 0.5f, 0.4f }, (b3Vec3){ 0.1f, 0.5f, 0.1f }, lp_wood, true );
	int postB = AddPart( &s, (b3Vec3){ 2.0f, 0.5f, 0.4f }, (b3Vec3){ 0.1f, 0.5f, 0.1f }, lp_wood, true );
	lpLinkDef def = lpDefaultLinkDef( lp_linkRope );
	def.bodyA = postA;
	def.bodyB = postB;
	def.anchorA = (b3Pos){ -2.0f, 1.0f, 0.4f };
	def.anchorB = (b3Pos){ 2.0f, 1.0f, 0.4f };
	lpImpactDef im = { 0 };
	im.point = (b3Pos){ 0.0f, 0.3f, 0.0f };
	im.radius = 1.4f;
	im.energy = 80000.0f;
	im.impulse = 12.0f;
	im.explosion = true;
	float d = sqrtf( 0.7f * 0.7f + 0.4f * 0.4f ); // from the blast to the rope
	float x = 1.0f - d / im.radius;
	float once = im.energy * x * x / ( B3_PI * im.radius * im.radius );
	def.strength = 1.5f * once; // survives one dose, not two
	int rope = lpCreateLink( s.world, &def );
	ENSURE( rope >= 0 );
	lpWorld_AddImpact( s.world, &im );
	bool valid;
	int deferred = 0;
	for ( int tick = 0; tick < 6; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		deferred += lpWorld_GetStats( s.world ).deferredJobs;
		valid = lpWorld_Validate( s.world );
		ENSURE( valid );
	}
	lpLinkState st = lpWorld_GetLinkState( s.world, rope );
	printf( "  %d deferred job-steps; rope health %.0f (expected %.0f after one dose)\n", deferred, (double)st.health,
			(double)( def.strength - once ) );
	ENSURE( deferred > 0 );
	ENSURE( st.alive );
	ENSURE_NEAR( st.health, def.strength - once, 0.01f * once );
	DestroySim( &s );
	return 0;
}

// A winch: shortening the rope lifts its load
static int TestSetRopeLength( void )
{
	Sim s = CreateSim( -1 );
	int beam = AddPart( &s, (b3Vec3){ 0.0f, 4.0f, 0.0f }, (b3Vec3){ 0.5f, 0.1f, 0.1f }, lp_wood, true );
	int block = AddPart( &s, (b3Vec3){ 0.0f, 2.0f, 0.0f }, (b3Vec3){ 0.2f, 0.2f, 0.2f }, lp_stone, false );
	int rope = Rope( &s, beam, (b3Vec3){ 0.0f, 3.9f, 0.0f }, block, (b3Vec3){ 0.0f, 2.2f, 0.0f }, 0.0f );
	bool valid;
	StepValidated( &s, 20, &valid );
	lpWorld_SetRopeLength( s.world, rope, 1.2f );
	StepValidated( &s, 90, &valid );
	float y = PieceY( &s, s.world->bodies.data[block].pieces.data[0] );
	printf( "  rope shortened from 1.7 m to 1.2 m: the block rose from 2.00 to %.3f\n", (double)y );
	ENSURE( valid );
	ENSURE_NEAR( y, 2.5f, 0.05f );
	DestroySim( &s );
	return 0;
}

// Volume of everything still standing as structure, not counting the ground
static float StructureVolume( const lpWorld* w )
{
	float v = 0.0f;
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		if ( p->body >= 0 && p->material != lp_ground && w->bodies.data[p->body].kind == lp_kindStructure )
		{
			v += p->shape->volume;
		}
	}
	return v;
}

// A 1.2 m stone cantilever mortared to a fixed block stands on its own; with a 420 kg block hung from its tip by a
// rope, the pull reaches its stress solve and the root joint gives
static bool CantileverWithLoadFalls( bool hang )
{
	Sim s = CreateSim( -1 );
	lpPartDef parts[2];
	parts[0] = lpDefaultPartDef();
	parts[0].halfExtents = (b3Vec3){ 0.5f, 0.5f, 0.5f };
	parts[0].transform.p = (b3Vec3){ 0.0f, 1.5f, 0.0f };
	parts[0].anchored = true;
	parts[0].joint = lp_jointMortar;
	parts[1] = lpDefaultPartDef();
	parts[1].halfExtents = (b3Vec3){ 0.6f, 0.3f, 0.2f };
	parts[1].transform.p = (b3Vec3){ 1.1f, 1.6f, 0.0f };
	parts[1].joint = lp_jointMortar;
	lpObjectDef def = lpDefaultObjectDef();
	def.parts = parts;
	def.partCount = 2;
	int cantilever = lpCreateObject( s.world, &def );
	if ( hang )
	{
		int load = AddPart( &s, (b3Vec3){ 1.6f, 0.62f, 0.0f }, (b3Vec3){ 0.28f, 0.28f, 0.28f }, lp_stone, false );
		ENSURE( Rope( &s, cantilever, (b3Vec3){ 1.6f, 1.3f, 0.0f }, load, (b3Vec3){ 1.6f, 0.9f, 0.0f }, 0.0f ) >= 0 );
	}
	float before = StructureVolume( s.world );
	bool valid;
	StepValidated( &s, 180, &valid );
	bool fell = StructureVolume( s.world ) < before - 0.1f;
	DestroySim( &s );
	return valid && fell;
}

static int TestSignPullsBeam( void )
{
	bool alone = CantileverWithLoadFalls( false );
	bool loaded = CantileverWithLoadFalls( true );
	printf( "  1.2 m cantilever alone %s; with 420 kg hung from its tip %s\n", alone ? "fell" : "stands", loaded ? "fell" : "stands" );
	ENSURE( alone == false );
	ENSURE( loaded );
	return 0;
}

// The yard scene at rest: every link holds, nothing strains
static int TestYardAtRest( void )
{
	Sim s = CreateSim( lp_sceneYard );
	int links = s.world->linkCount;
	bool valid;
	int breaks = StepValidated( &s, 300, &valid );
	float peak = 0.0f;
	for ( int i = 0; i < lpWorld_GetLinkCapacity( s.world ); ++i )
	{
		lpLinkState st = lpWorld_GetLinkState( s.world, i );
		peak = st.alive && st.utilization > peak ? st.utilization : peak;
	}
	printf( "  %d links; after 300 steps %d broke, peak load %.0f%% of a limit\n", links, breaks, (double)( 100.0f * peak ) );
	ENSURE( valid );
	ENSURE( links >= 12 );
	ENSURE( breaks == 0 && s.world->linkCount == links );
	ENSURE( peak < 0.5f );
	DestroySim( &s );
	return 0;
}

// The yard demo's first shot: the rifle cuts the cart's rope, the cart rolls down the ramp into the brick wall and
// its volatile crates go off
static int TestYardCart( void )
{
	Sim s = CreateSim( lp_sceneYard );
	Run( &s, 10 );
	int rope = -1;
	for ( int i = 0; i < lpWorld_GetLinkCapacity( s.world ); ++i )
	{
		lpLinkState st = lpWorld_GetLinkState( s.world, i );
		rope = st.alive && st.type == lp_linkRope && st.pointA.x < -11.0f ? i : rope;
	}
	ENSURE( rope >= 0 );
	int cart = lpWorld_GetLinkState( s.world, rope ).bodyB;
	int armed = 0;
	int crates[4];
	for ( int i = 0; i < s.world->bodies.count && armed < 4; ++i )
	{
		const lpBody* b = s.world->bodies.data + i;
		if ( b->alive && b->armed && b->detonator.triggerSpeed < 3.8f ) // the crates, not the rack's flasks
		{
			crates[armed++] = i;
		}
	}
	ENSURE( armed == 2 );
	b3WorldTransform start;
	lpWorld_GetBodyTransform( s.world, cart, &start );

	lpRayHit hit = Fire( &s, (b3Vec3){ -1.3f, 2.8f, 3.5f }, b3Add( (b3Vec3){ -1.3f, 2.8f, 3.5f }, (b3Vec3){ -10.03f, -0.02f, -11.5f } ) );
	ENSURE( hit.link == rope );
	float farthest = 0.0f;
	bool valid = true;
	for ( int tick = 0; tick < 240; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		b3WorldTransform xf;
		if ( lpWorld_GetBodyTransform( s.world, cart, &xf ) && s.world->bodies.data[cart].generation == 0 )
		{
			farthest = fmaxf( farthest, (float)( xf.p.x - start.p.x ) );
		}
		valid = valid && ( tick % 10 != 0 || lpWorld_Validate( s.world ) );
	}
	int wentOff = 0;
	for ( int k = 0; k < armed; ++k )
	{
		const lpBody* b = s.world->bodies.data + crates[k];
		wentOff += b->alive == false || b->armed == false ? 1 : 0;
	}
	int ropes = 0, hinges = 0;
	for ( int i = 0; i < lpWorld_GetLinkCapacity( s.world ); ++i )
	{
		lpLinkState st = lpWorld_GetLinkState( s.world, i );
		ropes += st.alive && st.type == lp_linkRope ? 1 : 0;
		hinges += st.alive && st.type == lp_linkHinge ? 1 : 0;
	}
	printf( "  the cart rolled %.1f m; %d of 2 crates went off; left: %d ropes, %d hinges\n", (double)farthest, wentOff, ropes,
			hinges );
	ENSURE( valid );
	ENSURE( farthest > 3.0f );
	ENSURE( wentOff >= 1 );
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
	RUN_TEST( TestLinkSurvivesSplit );
	RUN_TEST( TestLinkRehomedOnFracture );
	RUN_TEST( TestLinkBreaksWhenAnchorEjected );
	RUN_TEST( TestLinkToGhostBreaks );
	RUN_TEST( TestTinyEndTears );
	RUN_TEST( TestRopeShotSnaps );
	RUN_TEST( TestBlastBreaksHinge );
	RUN_TEST( TestDeferredNoDoubleDamage );
	RUN_TEST( TestSetRopeLength );
	RUN_TEST( TestSignPullsBeam );
	RUN_TEST( TestYardAtRest );
	RUN_TEST( TestYardCart );
	RUN_TEST( TestLinkDeterminism );
	return 0;
}
