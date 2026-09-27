// SPDX-License-Identifier: MIT
// Debris tiers: chunky fracture, ghosts and scrap, light debris, shoving, fragile rubble, budgets, deferred fracture.

#include "facet.h"
#include "test_macros.h"
#include "test_sim.h"

// A dynamic box object; returns its body index
static int AddBox( Sim* s, b3Vec3 position, b3Vec3 half, int material, b3Vec3 velocity )
{
	lpPartDef part = lpDefaultPartDef();
	part.halfExtents = half;
	part.material = (uint8_t)material;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = position;
	def.parts = &part;
	def.partCount = 1;
	def.linearVelocity = velocity;
	return lpCreateObject( s->world, &def );
}

static float BodyY( const Sim* s, int body )
{
	b3WorldTransform xf;
	lpWorld_GetBodyTransform( s->world, body, &xf );
	return (float)xf.p.y;
}

static int TestSliverAbsorption( void )
{
	lpPoly slab;
	lpPoly_MakeBox( &slab, (b3Vec3){ 1.5f, 1.0f, 0.15f }, b3Transform_identity, 0 );
	float slabVolume;
	b3Vec3 c;
	lpPoly_ComputeMass( &slab, &slabVolume, &c );

	for ( int seed = 0; seed < 20; ++seed )
	{
		lpFractureInput input = { 0 };
		input.parent = &slab;
		input.impact = (b3Vec3){ -0.4f + 0.04f * (float)seed, 0.1f, 0.15f };
		input.radius = 0.6f;
		input.fragmentSize = 0.16f;
		input.plateSize = 1.2f;
		input.maxCells = 28;
		input.absorbVolume = 0.002f;
		input.pattern = lp_breakImpact;
		input.axis = (b3Vec3){ 1.0f, 0.0f, 0.0f };
		input.seed = (uint64_t)seed;
		input.tolerance = 2e-5f;

		lpShape* cells[LP_MAX_SITES];
		int sites[LP_MAX_SITES];
		int count = lpFracture( &input, cells, sites, LP_MAX_SITES, NULL );
		ENSURE( count >= 4 );
		float sum = 0.0f;
		for ( int i = 0; i < count; ++i )
		{
			sum += cells[i]->volume;
			bool outside = b3Distance( cells[i]->centroid, input.impact ) > input.radius;
			ENSURE( outside == false || cells[i]->volume >= input.absorbVolume );
			lpShape_Destroy( cells[i] );
		}
		ENSURE_NEAR( sum, slabVolume, 2e-3f * slabVolume );
	}
	return 0;
}

// Shooting through the middle of a log: splinters fly off as ghosts and particles, and two rough ends of a handful
// of triangles remain.
static int TestLogEnds( void )
{
	Sim s = CreateSim( -1 );
	b3Vec3 points[16];
	for ( int i = 0; i < 8; ++i )
	{
		b3CosSin cs = b3ComputeCosSin( 0.7853982f * (float)i );
		points[i] = (b3Vec3){ -1.5f, 0.2f * cs.cosine, 0.2f * cs.sine };
		points[8 + i] = (b3Vec3){ 1.5f, 0.2f * cs.cosine, 0.2f * cs.sine };
	}
	lpPartDef part = lpDefaultPartDef();
	part.points = points;
	part.pointCount = 16;
	part.material = lp_wood;
	part.grainAxis = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (b3Vec3){ 0.0f, 0.21f, 0.0f };
	def.parts = &part;
	def.partCount = 1;
	lpCreateObject( s.world, &def );
	Run( &s, 30 );

	lpImpactDef im = { 0 };
	im.point = (b3Pos){ 0.0f, 0.41f, 0.0f };
	im.direction = (b3Vec3){ 0.0f, -1.0f, 0.0f };
	im.radius = 0.7f;
	im.energy = 25000.0f;
	lpWorld_AddImpact( s.world, &im );
	for ( int i = 0; i < 180; ++i )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		ENSURE( lpWorld_Validate( s.world ) );
	}

	// Two ends of one piece each, plus splinters in the cheap tiers (anything smaller than an end)
	static lpVertex vertices[LP_MAX_PIECE_VERTICES];
	int ends = 0;
	int splinters = 0;
	for ( int b = 0; b < s.world->bodies.count; ++b )
	{
		lpBody* body = s.world->bodies.data + b;
		if ( body->alive == false || body->kind == lp_kindStructure )
		{
			continue;
		}
		if ( body->volume < 0.05f )
		{
			splinters += 1;
			continue;
		}
		ENSURE( body->kind == lp_kindDebris || body->kind == lp_kindRubble );
		int triangles = 0;
		for ( int k = 0; k < body->pieces.count; ++k )
		{
			triangles += lpWorld_BuildPieceMesh( s.world, body->pieces.data[k], vertices, LP_MAX_PIECE_VERTICES ) / 3;
		}
		printf( "  end %d: %.3f m^3, %d pieces, %d triangles\n", ends, (double)body->volume, body->pieces.count, triangles );
		ENSURE( body->pieces.count == 1 );
		ENSURE( triangles < 60 );
		ends += 1;
	}
	printf( "  ends %d, splinters %d\n", ends, splinters );
	ENSURE( ends == 2 );
	ENSURE( splinters > 0 );
	DestroySim( &s );
	return 0;
}

// Every ghost from a blast either lands as scrap on the ground or falls out of the world
static int TestGhostLanding( void )
{
	Sim s = CreateSim( lp_sceneWall );
	lpImpactDef im = { 0 };
	im.point = (b3Pos){ -3.0f, 1.6f, 1.02f }; // the glass pane: shards are ghost sized
	im.direction = (b3Vec3){ 0.0f, 0.0f, -1.0f };
	im.radius = 1.4f;
	im.energy = 80000.0f;
	im.impulse = 12.0f;
	im.explosion = true;
	lpWorld_AddImpact( s.world, &im );
	int maxGhosts = 0;
	for ( int i = 0; i < 300; ++i )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		ENSURE( lpWorld_Validate( s.world ) );
		int ghosts = lpWorld_GetStats( s.world ).ghostBodies;
		maxGhosts = ghosts > maxGhosts ? ghosts : maxGhosts;
	}
	lpStats st = lpWorld_GetStats( s.world );
	printf( "  max ghosts %d, ghosts left %d, scrap %d\n", maxGhosts, st.ghostBodies, st.scrapBodies );
	ENSURE( maxGhosts > 5 );
	ENSURE( st.ghostBodies == 0 );
	ENSURE( st.scrapBodies > 5 );
	for ( int b = 0; b < s.world->bodies.count; ++b )
	{
		lpBody* body = s.world->bodies.data + b;
		if ( body->alive && body->kind == lp_kindScrap )
		{
			ENSURE( body->com.y > -0.05f && body->com.y < 4.2f ); // the ground, or on top of the 4 m stone wall
		}
	}
	DestroySim( &s );
	return 0;
}

// Light debris falls through a crate (it cannot push dynamic bodies) and lands on the ground
static int TestLightIgnoresDebris( void )
{
	Sim s = CreateSim( -1 );
	int crate = AddBox( &s, (b3Vec3){ 0.0f, 0.5f, 0.0f }, (b3Vec3){ 0.5f, 0.5f, 0.5f }, lp_wood, b3Vec3_zero );
	int chunk = AddBox( &s, (b3Vec3){ 0.0f, 3.0f, 0.0f }, (b3Vec3){ 0.08f, 0.08f, 0.08f }, lp_stone, b3Vec3_zero );
	lpConvertToLight( s.world, chunk );
	Run( &s, 120 );
	ENSURE( lpWorld_Validate( s.world ) );
	float y = BodyY( &s, chunk );
	printf( "  light chunk rests at y = %.3f (crate top is 1.0)\n", (double)y );
	ENSURE( y < 0.2f );
	ENSURE( BodyY( &s, crate ) > 0.45f );
	DestroySim( &s );
	return 0;
}

static float CrateX( bool withRubble )
{
	Sim s = CreateSim( -1 );
	int rubble = -1;
	if ( withRubble )
	{
		rubble = AddBox( &s, (b3Vec3){ 4.0f, 0.1f, 0.0f }, (b3Vec3){ 0.1f, 0.1f, 0.1f }, lp_stone, b3Vec3_zero );
		lpConvertToLight( s.world, rubble );
		Run( &s, 60 ); // settles and freezes
	}
	int crate = AddBox( &s, (b3Vec3){ 0.0f, 0.6f, 0.0f }, (b3Vec3){ 0.6f, 0.6f, 0.6f }, lp_stone, (b3Vec3){ 8.0f, 0.0f, 0.0f } );
	float x = 0.0f;
	for ( int i = 0; i < 45; ++i )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
	}
	b3WorldTransform xf;
	lpWorld_GetBodyTransform( s.world, crate, &xf );
	x = (float)xf.p.x;
	if ( withRubble )
	{
		lpBody* r = s.world->bodies.data + rubble;
		b3WorldTransform rx;
		lpWorld_GetBodyTransform( s.world, rubble, &rx );
		printf( "  rubble kind %d at x = %.2f (was 4.0)\n", r->kind, (double)rx.p.x );
		if ( b3AbsFloat( (float)rx.p.x - 4.0f ) < 0.05f && b3AbsFloat( (float)rx.p.z ) < 0.05f )
		{
			x = -1000.0f; // not shoved
		}
	}
	DestroySim( &s );
	return x;
}

// A heavy crate sliding through light rubble flings it aside and is not slowed by it
static int TestShove( void )
{
	float clear = CrateX( false );
	float through = CrateX( true );
	printf( "  crate x after 0.75 s: %.3f clear, %.3f through rubble\n", (double)clear, (double)through );
	ENSURE( through > -999.0f );
	ENSURE_NEAR( through, clear, 0.02f );
	return 0;
}

// Full rubble is static but fragile: something hitting it knocks it loose
static int TestFragileRubble( void )
{
	Sim s = CreateSim( -1 );
	int rock = AddBox( &s, (b3Vec3){ 0.0f, 0.3f, 0.0f }, (b3Vec3){ 0.3f, 0.3f, 0.3f }, lp_stone, b3Vec3_zero );
	Run( &s, 90 );
	ENSURE( s.world->bodies.data[rock].kind == lp_kindRubble );
	// Slides in at about 2.7 m/s after friction: above the wake speed, below the damage speed
	AddBox( &s, (b3Vec3){ -1.2f, 0.3f, 0.0f }, (b3Vec3){ 0.3f, 0.3f, 0.3f }, lp_stone, (b3Vec3){ 4.0f, 0.0f, 0.0f } );
	int woke = -1;
	for ( int i = 0; i < 60 && woke < 0; ++i )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		if ( s.world->bodies.data[rock].kind == lp_kindDebris )
		{
			woke = i;
		}
	}
	printf( "  rubble woke after %d ticks\n", woke );
	ENSURE( woke >= 0 );
	DestroySim( &s );
	return 0;
}

// With tiny budgets, bodies move down the ladder and the caps hold once the ladder has caught up
static int TestBudgetLadder( void )
{
	lpWorldDef ld = lpDefaultWorldDef();
	ld.maxFullDebris = 10;
	ld.maxLightDebris = 30;
	ld.maxGhosts = 60;
	ld.maxRubblePieces = 40;
	ld.maxScrapPieces = 50;
	Sim s = CreateSimDef( ld, lp_sceneWall );
	int demotions = 0;
	for ( int tick = 0; tick < 420; ++tick )
	{
		lpSceneBombard( s.world, lp_sceneWall, tick, 20 );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		ENSURE( lpWorld_Validate( s.world ) );
		demotions += lpWorld_GetStats( s.world ).demotionsThisStep;
	}
	Run( &s, 120 ); // let the ladder catch up
	lpStats st = lpWorld_GetStats( s.world );
	int rubblePieces = 0, scrapPieces = 0;
	for ( int b = 0; b < s.world->bodies.count; ++b )
	{
		lpBody* body = s.world->bodies.data + b;
		if ( body->alive && body->kind == lp_kindRubble )
		{
			rubblePieces += body->pieces.count;
		}
		if ( body->alive && body->kind == lp_kindScrap && body->sinkTicks == 0 )
		{
			scrapPieces += body->pieces.count;
		}
	}
	printf( "  demotions %d; full %d, light %d, ghosts %d, rubble pieces %d, scrap pieces %d\n", demotions, st.fullDebris,
			st.lightDebris, st.ghostBodies, rubblePieces, scrapPieces );
	ENSURE( demotions > 0 );
	ENSURE( st.fullDebris <= ld.maxFullDebris );
	ENSURE( st.lightDebris <= ld.maxLightDebris );
	ENSURE( st.ghostBodies <= ld.maxGhosts );
	ENSURE( rubblePieces <= ld.maxRubblePieces );
	ENSURE( scrapPieces <= ld.maxScrapPieces );
	DestroySim( &s );
	return 0;
}

static int DeferredRun( int budget, int* maxJobs, int* totalFractures, int* leftover )
{
	lpWorldDef ld = lpDefaultWorldDef();
	ld.maxFractureJobsPerStep = budget;
	Sim s = CreateSimDef( ld, lp_sceneWall );
	lpImpactDef im = { 0 };
	im.point = (b3Pos){ 0.0f, 2.0f, -8.7f }; // the stone wall
	im.radius = 2.5f;
	im.energy = 600000.0f;
	im.impulse = 10.0f;
	im.explosion = true;
	lpWorld_AddImpact( s.world, &im );
	*maxJobs = 0;
	*totalFractures = 0;
	for ( int i = 0; i < 90; ++i )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		ENSURE( lpWorld_Validate( s.world ) );
		lpStats st = lpWorld_GetStats( s.world );
		*maxJobs = st.fracturesThisStep > *maxJobs ? st.fracturesThisStep : *maxJobs;
		*totalFractures += st.fracturesThisStep;
	}
	*leftover = lpWorld_GetStats( s.world ).deferredJobs;
	DestroySim( &s );
	return 0;
}

// A small per-step fracture budget spreads a big blast over several steps; the work all drains
static int TestDeferredFracture( void )
{
	int maxA, totalA, leftA, maxB, totalB, leftB;
	ENSURE( DeferredRun( 1000, &maxA, &totalA, &leftA ) == 0 );
	ENSURE( DeferredRun( 2, &maxB, &totalB, &leftB ) == 0 );
	printf( "  unlimited: %d fractures (max %d in a step); budget 2: %d fractures (max %d in a step), %d left\n", totalA, maxA,
			totalB, maxB, leftB );
	ENSURE( maxA > 2 );
	ENSURE( maxB <= 2 ); // the budget is per step, shared by all impacts
	ENSURE( leftB == 0 );
	ENSURE( totalB > totalA / 2 );
	return 0;
}

int DebrisTest( void )
{
	RUN_TEST( TestSliverAbsorption );
	RUN_TEST( TestLogEnds );
	RUN_TEST( TestGhostLanding );
	RUN_TEST( TestLightIgnoresDebris );
	RUN_TEST( TestShove );
	RUN_TEST( TestFragileRubble );
	RUN_TEST( TestBudgetLadder );
	RUN_TEST( TestDeferredFracture );
	return 0;
}
