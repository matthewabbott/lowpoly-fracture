// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "test_sim.h"

// Rifle shots walking across the brick wall, then a grenade and a cannon-sized blast
static void Bombard( Sim* s, int tick )
{
	if ( tick < 200 && tick % 10 == 3 )
	{
		int k = tick / 10;
		lpImpactDef im = { 0 };
		im.point = (b3Pos){ -2.5f + 0.25f * (float)k, 0.6f + 0.1f * (float)( k % 7 ), -3.85f };
		im.direction = (b3Vec3){ 0.0f, 0.0f, -1.0f };
		im.radius = 0.35f;
		im.energy = 4000.0f;
		im.impulse = 20.0f;
		lpWorld_AddImpact( s->world, &im );
	}
	if ( tick == 120 )
	{
		lpImpactDef im = { 0 };
		im.point = (b3Pos){ 2.0f, 1.2f, -3.6f };
		im.radius = 1.4f;
		im.energy = 80000.0f;
		im.impulse = 12.0f;
		im.explosion = true;
		lpWorld_AddImpact( s->world, &im );
	}
	if ( tick == 150 )
	{
		lpImpactDef im = { 0 };
		im.point = (b3Pos){ 0.0f, 2.0f, -8.6f };
		im.radius = 1.8f;
		im.energy = 300000.0f;
		im.impulse = 18.0f;
		im.explosion = true;
		lpWorld_AddImpact( s->world, &im );
	}
}

static int TestWallDamage( void )
{
	Sim s = CreateSimWorkers( lp_sceneWall, 1 );
	ENSURE( lpWorld_Validate( s.world ) );
	int initialPieces = 0;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		initialPieces += s.world->pieces.data[i].body >= 0 ? 1 : 0;
	}

	int maxDebris = 0;
	int fractures = 0;
	for ( int tick = 0; tick < 300; ++tick )
	{
		Bombard( &s, tick );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		ENSURE( lpWorld_Validate( s.world ) );
		lpStats st = lpWorld_GetStats( s.world );
		fractures += st.fracturesThisStep;
		maxDebris = st.debrisBodies > maxDebris ? st.debrisBodies : maxDebris;
	}

	lpStats st = lpWorld_GetStats( s.world );
	printf( "  pieces %d -> %d, fractures %d, max debris %d, rubble %d, clip failures %d\n", initialPieces, st.pieceCount,
			fractures, maxDebris, st.rubbleBodies, st.clipFailures );
	ENSURE( st.pieceCount > 2 * initialPieces );
	ENSURE( fractures > 10 );
	ENSURE( maxDebris > 20 );
	ENSURE( st.clipFailures == 0 );
	DestroySim( &s );
	return 0;
}

static int RunHashes( int scene, int workers, int ticks, uint64_t* hashes )
{
	Sim s = CreateSimWorkers( scene, workers );
	for ( int tick = 0; tick < ticks; ++tick )
	{
		Bombard( &s, tick );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		hashes[tick] = lpWorld_Hash( s.world );
	}
	DestroySim( &s );
	return 0;
}

static int TestDeterminism( void )
{
	enum
	{
		ticks = 240
	};
	static uint64_t a[ticks], b[ticks], c[ticks];
	RunHashes( lp_sceneWall, 1, ticks, a );
	RunHashes( lp_sceneWall, 1, ticks, b );
	RunHashes( lp_sceneWall, 4, ticks, c );
	for ( int i = 0; i < ticks; ++i )
	{
		if ( a[i] != b[i] )
		{
			printf( "  rerun diverged at tick %d\n", i );
			return 1;
		}
		if ( a[i] != c[i] )
		{
			printf( "  4 workers diverged from 1 worker at tick %d\n", i );
			return 1;
		}
	}
	printf( "  final hash %016llx\n", (unsigned long long)a[ticks - 1] );
	return 0;
}

static float LooseVolume( const lpWorld* world );

// Blasts at the corners of a house knock a good part of it loose (ejected fragments, split-off chunks, rubble)
static int TestHouseCollapse( void )
{
	Sim s = CreateSimWorkers( lp_sceneHouse, 1 );
	float before = LooseVolume( s.world );
	int splits = 0;
	for ( int tick = 0; tick < 240; ++tick )
	{
		if ( tick == 10 || tick == 40 )
		{
			lpImpactDef im = { 0 };
			im.point = (b3Pos){ tick == 10 ? -3.2f : 3.2f, 1.0f, -3.5f };
			im.radius = 2.2f;
			im.energy = 400000.0f;
			im.impulse = 15.0f;
			im.explosion = true;
			lpWorld_AddImpact( s.world, &im );
		}
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		ENSURE( lpWorld_Validate( s.world ) );
		splits += lpWorld_GetStats( s.world ).splitsThisStep;
	}
	lpStats st = lpWorld_GetStats( s.world );
	float loose = LooseVolume( s.world ) - before;
	printf( "  splits %d, loose volume %.2f m^3, pieces %d, debris %d (full %d, light %d), rubble %d, ghosts %d, scrap %d\n", splits,
			(double)loose, st.pieceCount, st.debrisBodies, st.fullDebris, st.lightDebris, st.rubbleBodies, st.ghostBodies, st.scrapBodies );
	ENSURE( loose > 2.0f );
	DestroySim( &s );
	return 0;
}

// A volatile flask thrown at the brick wall goes off on impact and blows a hole
static int TestDetonator( void )
{
	Sim s = CreateSimWorkers( lp_sceneWall, 1 );
	b3Vec3 points[8];
	for ( int i = 0; i < 8; ++i )
	{
		points[i] = (b3Vec3){ ( i & 1 ) ? 0.08f : -0.08f, ( i & 2 ) ? 0.12f : -0.12f, ( i & 4 ) ? 0.08f : -0.08f };
	}
	lpPartDef part = lpDefaultPartDef();
	part.points = points;
	part.pointCount = 8;
	part.material = lp_glass;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (b3Vec3){ 1.0f, 1.5f, 0.0f };
	def.linearVelocity = (b3Vec3){ 0.0f, 1.0f, -16.0f };
	def.parts = &part;
	def.partCount = 1;
	def.detonator = (lpDetonatorDef){ 4.5f, 1.8f, 120000.0f, 12.0f };
	int flask = lpCreateObject( s.world, &def );

	int fractures = 0, cells = 0;
	for ( int tick = 0; tick < 90; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		ENSURE( lpWorld_Validate( s.world ) );
		fractures += lpWorld_GetStats( s.world ).fracturesThisStep;
		cells += lpWorld_GetStats( s.world ).cellsThisStep;
	}
	ENSURE( s.world->bodies.data[flask].alive == false || s.world->bodies.data[flask].armed == false );
	printf( "  fractures %d, cells %d, debris %d\n", fractures, cells, lpWorld_GetStats( s.world ).debrisBodies );
	ENSURE( fractures >= 1 && cells > 20 ); // one solid brick wall now: fewer pieces break, into many cells
	DestroySim( &s );
	return 0;
}

// Pulling a loose crate lifts it toward the target
static int TestPull( void )
{
	Sim s = CreateSimWorkers( lp_scenePile, 1 );
	for ( int tick = 0; tick < 120; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
	}
	// Grab the loose piece on top of the heap
	int piece = -1;
	float top = -1.0e9f;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		lpPiece* p = s.world->pieces.data + i;
		if ( p->body >= 0 && s.world->bodies.data[p->body].kind != lp_kindStructure )
		{
			float y = (float)lpWorld_ToWorldFrame( s.world, i, p->shape->centroid ).y;
			if ( y > top )
			{
				top = y;
				piece = i;
			}
		}
	}
	ENSURE( piece >= 0 );
	b3Vec3 local = s.world->pieces.data[piece].shape->centroid;
	b3Pos start = lpWorld_ToWorldFrame( s.world, piece, local );
	b3Pos target = { start.x, start.y + 3.0f, start.z + 2.0f };
	for ( int tick = 0; tick < 120; ++tick )
	{
		lpWorld_Pull( s.world, piece, local, target, 40.0f, 400.0f );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
	}
	b3Pos end = lpWorld_ToWorldFrame( s.world, piece, local );
	float error = b3Length( b3SubPos( end, target ) );
	printf( "  pulled from (%.2f %.2f %.2f) to (%.2f %.2f %.2f), error %.3f m\n", (double)start.x, (double)start.y, (double)start.z,
			(double)end.x, (double)end.y, (double)end.z, (double)error );
	ENSURE( error < 0.3f );
	DestroySim( &s );
	return 0;
}

// Hitting the same spot repeatedly refractures pieces that came from an earlier fracture. Every bond must still join
// pieces that touch (stale cut-face tags once created phantom bonds between unrelated cells).
static int TestRefractureBonds( void )
{
	Sim s = CreateSimWorkers( lp_sceneWall, 1 );
	for ( int tick = 0; tick < 40; ++tick )
	{
		if ( tick % 8 == 1 )
		{
			lpImpactDef im = { 0 };
			im.point = (b3Pos){ -1.5f + 0.05f * (float)tick, 1.5f, -3.85f };
			im.direction = (b3Vec3){ 0.0f, 0.0f, -1.0f };
			im.radius = 0.8f;
			im.energy = 9000.0f;
			lpWorld_AddImpact( s.world, &im );
		}
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		ENSURE( lpWorld_Validate( s.world ) );
		ENSURE( lpWorld_ValidateBondGeometry( s.world ) );
	}
	int deep = 0;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		deep += s.world->pieces.data[i].body >= 0 && s.world->pieces.data[i].depth >= 2 ? 1 : 0;
	}
	printf( "  pieces from a second-generation fracture: %d\n", deep );
	ENSURE( deep > 0 );
	DestroySim( &s );
	return 0;
}

static float LooseVolume( const lpWorld* world )
{
	float v = 0.0f;
	for ( int i = 0; i < world->bodies.count; ++i )
	{
		const lpBody* b = world->bodies.data + i;
		if ( b->alive && b->kind != lp_kindStructure )
		{
			v += b->volume;
		}
	}
	return v;
}

// Blasting out one side of the tower's base: with the weight check the undermined masonry comes down,
// without it (Teardown-style connectivity only) it hangs on whatever still connects it to the ground.
int WorldTest( void )
{
	RUN_TEST( TestRefractureBonds );
	RUN_TEST( TestDetonator );
	RUN_TEST( TestPull );
	RUN_TEST( TestWallDamage );
	RUN_TEST( TestDeterminism );
	RUN_TEST( TestHouseCollapse );
	return 0;
}
