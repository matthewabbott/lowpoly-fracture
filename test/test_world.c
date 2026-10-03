// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "test_sim.h"

#include "dump.h"
#include "script.h"

#include <stdlib.h>

// lpDeterminismSelfTest's hash on every platform (set from the reference build; CI checks every leg against it)
#define LP_SELF_TEST_HASH 0xe6c87a41bbafe2ddull

// The tick the contraption's vial goes off today (TestContraptionOnTime allows 10% either way)
#define LP_CONTRAPTION_TICK 2986

// Rifle shots walking across the brick wall, then a grenade and a cannon-sized blast
static void Bombard( Sim* s, int tick )
{
	if ( tick < 200 && tick % 10 == 3 )
	{
		int k = tick / 10;
		lpImpactDef im = { 0 };
		im.point = (lpPos){ -2.5f + 0.25f * (float)k, 0.6f + 0.1f * (float)( k % 7 ), -3.85f };
		im.direction = (lpVec3){ 0.0f, 0.0f, -1.0f };
		im.radius = 0.35f;
		im.energy = 4000.0f;
		im.impulse = 20.0f;
		lpWorld_AddImpact( s->world, &im );
	}
	if ( tick == 120 )
	{
		lpImpactDef im = { 0 };
		im.point = (lpPos){ 2.0f, 1.2f, -3.6f };
		im.radius = 1.4f;
		im.energy = 80000.0f;
		im.impulse = 12.0f;
		im.explosion = true;
		lpWorld_AddImpact( s->world, &im );
	}
	if ( tick == 150 )
	{
		lpImpactDef im = { 0 };
		im.point = (lpPos){ 0.0f, 2.0f, -8.6f };
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

// Two worlds in one process, stepped in turn on the same inputs, stay equal: worlds share nothing (the two-world lab
// relies on it), and following the hash down finds no difference
static int TestTwinWorlds( void )
{
	Sim a = CreateSimWorkers( lp_sceneTown, 1 );
	Sim b = CreateSimWorkers( lp_sceneTown, 4 );
	for ( int tick = 0; tick < 180; ++tick )
	{
		lpSceneBombard( a.world, lp_sceneTown, tick, 6 );
		lpSceneBombard( b.world, lp_sceneTown, tick, 6 );
		lpWorld_Step( a.world, 1.0f / 60.0f, 4 );
		lpWorld_Step( b.world, 1.0f / 60.0f, 4 );
		if ( lpWorld_Hash( a.world ) != lpWorld_Hash( b.world ) )
		{
			printf( "  the twins differ at tick %d\n", tick );
			return 1;
		}
	}
	lpLabDiff diff;
	ENSURE( lpLab_Diff( a.world, b.world, &diff, 1 ) == 0 );
	printf( "  180 ticks of the barrage side by side: equal every tick\n" );
	DestroySim( &a );
	DestroySim( &b );
	return 0;
}

// An injected desync is named: a one-ulp nudge to a crate's velocity, or to the warm start of one of its contacts, is
// followed down the hash to that body (with its generation) and its unit; repaired at once, by copying that unit's
// motion, warm starts and sleep timers from the other world, the two stay equal
static int TestDesyncNamed( void )
{
	for ( int warm = 0; warm < 2; ++warm )
	{
		Sim a = CreateSimWorkers( lp_scenePile, 1 );
		Sim b = CreateSimWorkers( lp_scenePile, 1 );
		for ( int tick = 0; tick < 60; ++tick )
		{
			lpWorld_Step( a.world, 1.0f / 60.0f, 4 );
			lpWorld_Step( b.world, 1.0f / 60.0f, 4 );
		}
		int body = -1;
		for ( int i = 0; i < lpWorld_GetBodyCapacity( b.world ) && body < 0; ++i )
		{
			lpBodyInfo info = lpWorld_GetBodyInfo( b.world, i );
			if ( info.alive && info.kind == lp_kindDebris && info.awake && lpWorld_GetBodyContacts( b.world, i, NULL, 0 ) > 0 )
			{
				body = i;
			}
		}
		ENSURE( body >= 0 );
		if ( warm )
		{
			ENSURE( lpLab_NudgeWarmStart( b.world, body, 1 ) );
		}
		else
		{
			lpLab_NudgeVelocity( b.world, body, 1 );
		}

		// Named: the body's element (velocity) or its physics engine element (warm start), in the body's unit
		lpLabDiff diffs[16];
		int count = lpLab_Diff( a.world, b.world, diffs, 16 );
		int capacity = lpWorld_GetBodyCapacity( a.world );
		int* units = malloc( sizeof( int ) * (size_t)capacity );
		ENSURE( units != NULL && lpWorld_GetUnits( a.world, units, capacity ) > 0 );
		int category = warm ? lp_hashBackend : lp_hashBodies;
		bool named = false, elsewhere = false;
		for ( int k = 0; k < count && k < 16; ++k )
		{
			int unit = lpWorld_GetElementUnit( a.world, units, diffs[k].category, diffs[k].slot );
			named = named || ( diffs[k].category == category && diffs[k].slot == body &&
							   diffs[k].generation == lpWorld_GetBodyInfo( a.world, body ).generation && unit == units[body] );
			elsewhere = elsewhere || unit != units[body];
		}
		ENSURE( count >= 1 && count <= 16 && named && elsewhere == false );
		printf( "  %s nudge on body %d: %d element(s) differ, the first %d:%d, all in its unit (%d)\n",
				warm ? "warm-start" : "velocity", body, count, diffs[0].category, diffs[0].slot, units[body] );

		// Repaired at once, it stays repaired
		int bytes = lpLab_RepairUnit( b.world, a.world, units, units[body], lp_labMotion | lp_labWarmStarts | lp_labSleep );
		ENSURE( lpLab_Diff( a.world, b.world, diffs, 16 ) == 0 );
		for ( int tick = 0; tick < 60; ++tick )
		{
			lpWorld_Step( a.world, 1.0f / 60.0f, 4 );
			lpWorld_Step( b.world, 1.0f / 60.0f, 4 );
			ENSURE( lpWorld_Hash( a.world ) == lpWorld_Hash( b.world ) );
		}
		printf( "    repaired with its unit (%d bytes): equal for the next 60 ticks\n", bytes );
		free( units );
		DestroySim( &a );
		DestroySim( &b );
	}
	return 0;
}

// A library that turns flush-to-zero on behind our back (milestone 7's E9: it changed the stress solver) changes
// nothing: the step puts the control word back first, and counts it
static int TestFpGuard( void )
{
	enum
	{
		ticks = 120
	};
	static uint64_t clean[ticks][2], broken[ticks][2];
	int breaks = 0;
	for ( int run = 0; run < 2; ++run )
	{
		Sim s = CreateSimWorkers( lp_sceneWall, 4 );
		for ( int tick = 0; tick < ticks; ++tick )
		{
			Bombard( &s, tick );
			if ( run == 1 && tick % 10 == 5 )
			{
				lpFpBreakForTest();
				breaks += 1;
			}
			lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
			uint64_t* out = run == 0 ? clean[tick] : broken[tick];
			out[0] = lpWorld_Hash( s.world );
			out[1] = lpWorld_HashStress( s.world );
		}
		int repairs = lpWorld_GetStats( s.world ).fpRepairs;
		DestroySim( &s );
		if ( run == 1 && repairs != breaks )
		{
			printf( "  %d control words broken, %d put back\n", breaks, repairs );
			lpFpGuard();
			return 1;
		}
	}
	lpFpGuard();
	for ( int i = 0; i < ticks; ++i )
	{
		if ( clean[i][0] != broken[i][0] || clean[i][1] != broken[i][1] )
		{
			printf( "  diverged at tick %d with flush-to-zero turned on between steps\n", i );
			return 1;
		}
	}
	return 0;
}

// The arithmetic every machine that plays together must share: known answers, and one hash on every platform
// (determinism rule 15; CI runs this on every leg)
static int TestDeterminismSelfTest( void )
{
	int failures = -1;
	uint64_t hash = lpDeterminismSelfTest( &failures );
	printf( "  self-test hash %016llx\n", (unsigned long long)hash );
	if ( failures != 0 )
	{
		printf( "  %d known answers wrong\n", failures );
		return 1;
	}
	return hash == LP_SELF_TEST_HASH ? 0 : 1;
}

static char* Describe( const lpWorld* world )
{
	int length = lpWorld_DescribeSession( world, NULL, 0 );
	char* text = malloc( (size_t)length + 1 );
	if ( text != NULL && ( lpWorld_DescribeSession( world, text, length + 1 ) != length || (int)strlen( text ) != length ) )
	{
		free( text );
		text = NULL;
	}
	return text;
}

// Two machines' sessions agree only when every setting, material, joint and template does: a peer whose world differs
// in one is refused by the key that names it
static int TestSessionHandshake( void )
{
	lpWorldDef def = lpDefaultWorldDef();
	lpWorld* a = lpCreateWorld( &def );
	lpWorld* same = lpCreateWorld( &def );
	def.maxStressWork += 1;
	lpWorld* work = lpCreateWorld( &def );
	def = lpDefaultWorldDef();
	lpMaterialDef materials[lp_materialCount];
	memcpy( materials, lpDefaultMaterials(), sizeof( materials ) );
	materials[lp_brick].bondStrength *= 1.01f;
	def.materials = materials;
	lpWorld* brick = lpCreateWorld( &def );
	def = lpDefaultWorldDef();
	def.workerCount = 7; // results do not depend on it
	lpWorld* workers = lpCreateWorld( &def );
	lpWorld* spawns = lpCreateWorld( &def );
	lpPartDef part = lpDefaultPartDef();
	lpObjectDef ball = lpDefaultObjectDef();
	ball.parts = &part;
	ball.partCount = 1;
	lpWorld_AddTemplate( spawns, &ball );

	char* textA = Describe( a );
	char* textSame = Describe( same );
	char* textWork = Describe( work );
	char* textBrick = Describe( brick );
	char* textWorkers = Describe( workers );
	char* textSpawns = Describe( spawns );
	ENSURE( textA && textSame && textWork && textBrick && textWorkers && textSpawns );
	char key[64];
	ENSURE( lpSessionCompare( textA, textSame, key, sizeof( key ) ) && key[0] == 0 );
	ENSURE( lpSessionCompare( textA, textWorkers, key, sizeof( key ) ) );
	ENSURE( lpSessionCompare( textA, textWork, key, sizeof( key ) ) == false && strcmp( key, "maxStressWork" ) == 0 );
	printf( "  maxStressWork + 1: refused on '%s'\n", key );
	char expected[32];
	snprintf( expected, sizeof( expected ), "material.%d", (int)lp_brick );
	ENSURE( lpSessionCompare( textA, textBrick, key, sizeof( key ) ) == false && strcmp( key, expected ) == 0 );
	printf( "  brick 1%% stronger: refused on '%s'\n", key );
	ENSURE( lpSessionCompare( textA, textSpawns, key, sizeof( key ) ) == false && strcmp( key, "template.0" ) == 0 );
	ENSURE( lpSessionCompare( textSpawns, textA, key, sizeof( key ) ) == false && strcmp( key, "template.0" ) == 0 );
	printf( "  a template only one side has: refused on '%s'\n", key );

	// An app's own lines, in another order on the other side, still agree; a truncated buffer still reports the length
	size_t n = strlen( textA );
	char* withApp = malloc( n + 64 );
	char* withAppReordered = malloc( n + 64 );
	ENSURE( withApp != NULL && withAppReordered != NULL );
	snprintf( withApp, n + 64, "%sscene town\r\ndt 0.0166666675\n", textA );
	snprintf( withAppReordered, n + 64, "dt 0.0166666675\nscene town\n%s", textA );
	ENSURE( lpSessionCompare( withApp, withAppReordered, key, sizeof( key ) ) );
	ENSURE( lpSessionCompare( withApp, textA, key, sizeof( key ) ) == false && strcmp( key, "scene" ) == 0 );
	char small[16];
	ENSURE( lpWorld_DescribeSession( a, small, sizeof( small ) ) == (int)n && strlen( small ) == sizeof( small ) - 1 );

	free( withApp );
	free( withAppReordered );
	free( textA );
	free( textSame );
	free( textWork );
	free( textBrick );
	free( textWorkers );
	free( textSpawns );
	lpDestroyWorld( a );
	lpDestroyWorld( same );
	lpDestroyWorld( work );
	lpDestroyWorld( brick );
	lpDestroyWorld( workers );
	lpDestroyWorld( spawns );
	return 0;
}


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
			im.point = (lpPos){ tick == 10 ? -3.2f : 3.2f, 1.0f, -3.5f };
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
	lpVec3 points[8];
	for ( int i = 0; i < 8; ++i )
	{
		points[i] = (lpVec3){ ( i & 1 ) ? 0.08f : -0.08f, ( i & 2 ) ? 0.12f : -0.12f, ( i & 4 ) ? 0.08f : -0.08f };
	}
	lpPartDef part = lpDefaultPartDef();
	part.points = points;
	part.pointCount = 8;
	part.material = lp_glass;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (lpVec3){ 1.0f, 1.5f, 0.0f };
	def.linearVelocity = (lpVec3){ 0.0f, 1.0f, -16.0f };
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
	ENSURE( BodyArmed( s.world, flask, NULL ) == false );
	printf( "  fractures %d, cells %d, debris %d\n", fractures, cells, lpWorld_GetStats( s.world ).debrisBodies );
	ENSURE( fractures >= 1 && cells > 20 ); // one solid brick wall now: fewer pieces break, into many cells
	DestroySim( &s );
	return 0;
}

// A detonated body whose slot is freed and reused before its blast step must not take the new body with it
static int TestDetonatorIndexReuse( void )
{
	Sim s = CreateSimWorkers( -1, 1 );
	lpPartDef part = lpDefaultPartDef();
	part.halfExtents = (lpVec3){ 0.2f, 0.2f, 0.2f };
	part.material = lp_metal; // does not fracture, so the blast only sets it off
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = false;
	def.transform.p = (lpPos){ 0.0f, 0.2f, 0.0f };
	def.parts = &part;
	def.partCount = 1;
	def.detonator = (lpDetonatorDef){ 4.5f, 1.8f, 120000.0f, 12.0f };
	int armed = lpCreateObject( s.world, &def );

	lpImpactDef im = { 0 };
	im.point = (lpPos){ 0.5f, 0.2f, 0.0f };
	im.radius = 1.4f;
	im.energy = 80000.0f;
	im.explosion = true;
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 1 );
	ENSURE( s.world->pendingDestroy.count == 1 );

	// Its slot is freed (as when its own fracture empties it) and taken by a new object far away
	lpDestroyBody( s.world, armed, false );
	def.detonator = (lpDetonatorDef){ 0 };
	def.transform.p = (lpPos){ 30.0f, 0.2f, 0.0f };
	int other = lpCreateObject( s.world, &def );
	ENSURE( other == armed );
	Run( &s, 2 );
	ENSURE( s.world->bodies.data[other].alive );
	ENSURE( lpWorld_Validate( s.world ) );
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
	lpVec3 local = s.world->pieces.data[piece].shape->centroid;
	lpPos start = lpWorld_ToWorldFrame( s.world, piece, local );
	lpPos target = { start.x, start.y + 3.0f, start.z + 2.0f };
	for ( int tick = 0; tick < 120; ++tick )
	{
		lpWorld_Pull( s.world, piece, local, target, 40.0f, 400.0f );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
	}
	lpPos end = lpWorld_ToWorldFrame( s.world, piece, local );
	float error = lpLength( lpSubPos( end, target ) );
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
			im.point = (lpPos){ -1.5f + 0.05f * (float)tick, 1.5f, -3.85f };
			im.direction = (lpVec3){ 0.0f, 0.0f, -1.0f };
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

// A vertex colour (0xAABBGGRR) is `rgb` (0xRRGGBB) shaded: the same hue, within rounding, brighter or darker
static bool Shaded( uint32_t rgba, uint32_t rgb )
{
	float c[3] = { (float)( rgba & 0xFF ), (float)( ( rgba >> 8 ) & 0xFF ), (float)( ( rgba >> 16 ) & 0xFF ) };
	float b[3] = { (float)( ( rgb >> 16 ) & 0xFF ), (float)( ( rgb >> 8 ) & 0xFF ), (float)( rgb & 0xFF ) };
	float sc = c[0] + c[1] + c[2];
	float sb = b[0] + b[1] + b[2];
	if ( c[0] == 255.0f || c[1] == 255.0f || c[2] == 255.0f )
	{
		return true; // clipped: no hue to judge
	}
	for ( int k = 0; k < 3; ++k )
	{
		if ( fabsf( c[k] / sc - b[k] / sb ) > 0.02f + 3.0f / sc )
		{
			return false;
		}
	}
	return sc > 0.4f * sb && sc < 1.25f * sb;
}

// Fragments are carved from the object and coloured like it: after a wall is shot up and blasted, every face of every
// piece is the object's own colour or its material's cut-face colour, shaded (never a flat grey, never another's)
static int TestFragmentColours( void )
{
	Sim s = CreateSimWorkers( lp_sceneWall, 1 );
	for ( int tick = 0; tick < 200; ++tick )
	{
		Bombard( &s, tick );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
	}
	lpVertex* vertices = (lpVertex*)malloc( sizeof( lpVertex ) * (size_t)lpWorld_GetMaxPieceVertices() );
	int fragments = 0, faces = 0, wrong = 0;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		const lpPiece* p = s.world->pieces.data + i;
		if ( p->body < 0 || p->material == lp_ground )
		{
			continue;
		}
		fragments += p->depth > 0 ? 1 : 0;
		int count = lpWorld_BuildPieceMesh( s.world, i, vertices, lpWorld_GetMaxPieceVertices() );
		uint32_t cut = lpWorld_GetMaterial( s.world, p->material )->interiorColor;
		for ( int v = 0; v < count; v += 3 )
		{
			faces += 1;
			bool ok = Shaded( vertices[v].color, p->color ) || Shaded( vertices[v].color, cut );
			if ( ok == false && wrong < 3 )
			{
				printf( "  piece %d (material %d): %08x is neither %06x nor %06x shaded\n", i, p->material, vertices[v].color, p->color,
						cut );
			}
			wrong += ok ? 0 : 1;
		}
	}
	free( vertices );
	printf( "  %d fragments, %d triangles, %d off colour\n", fragments, faces, wrong );
	ENSURE( fragments > 50 && wrong == 0 );
	DestroySim( &s );
	return 0;
}

// A house that lost two corners slumps or gives out, then goes quiet: no joint breaks after it, no structure left
// solving or straining, nothing moving
static int TestBuildingGoesQuiet( void )
{
	Sim s = CreateSimWorkers( lp_sceneHouse, 1 );
	int lastBreak = -1, lastMoving = -1, lastUnsettled = -1;
	for ( int tick = 0; tick < 1800; ++tick )
	{
		if ( tick == 10 || tick == 40 )
		{
			lpImpactDef im = { 0 };
			im.point = (lpPos){ tick == 10 ? -3.2f : 3.2f, 1.0f, -3.5f };
			im.radius = 2.2f;
			im.energy = 400000.0f;
			im.impulse = 15.0f;
			im.explosion = true;
			lpWorld_AddImpact( s.world, &im );
		}
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		lpStats st = lpWorld_GetStats( s.world );
		lastBreak = st.stressBreaks > 0 ? tick : lastBreak;
		lastUnsettled = st.unsettledStructures > 0 ? tick : lastUnsettled;
		lastMoving = MaxBodySpeed( s.world ) > 0.05f ? tick : lastMoving;
	}
	printf( "  the last joint broke at %.1f s; the last structure settled at %.1f s; the last body stopped at %.1f s\n",
			(double)lastBreak / 60.0, (double)lastUnsettled / 60.0, (double)lastMoving / 60.0 );
	ENSURE( lastBreak < 900 && lastUnsettled < 1200 && lastMoving < 1200 ); // quiet within 20 s of the blasts
	ENSURE( lpWorld_Validate( s.world ) );
	DestroySim( &s );
	return 0;
}

// Each world holds its own materials and joints: where glass is unbreakable a rifle round leaves the pane whole, while it
// shatters in a default world beside it; a world's tables are its own copies (and every joint's strength as given);
// auto joints are each material's default; a table out of range is refused
static int GlassPieces( const lpWorld* w )
{
	int count = 0;
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		count += w->pieces.data[i].body >= 0 && w->pieces.data[i].material == lp_glass ? 1 : 0;
	}
	return count;
}

// A pulled piece that fractures in the same step hands its slot to one of its cells: the pull named the piece, so it
// must not land on the cell. With the pull and without it, the world comes out the same.
static int TestPullSkipsReusedSlot( void )
{
	uint64_t hashes[2] = { 0 };
	for ( int run = 0; run < 2; ++run )
	{
		Sim s = CreateSimWorkers( lp_scenePile, 1 );
		Run( &s, 5 );

		// The biggest loose crate or rock of the heap, struck hard where it is and pulled in the same step
		int piece = -1;
		float biggest = 0.0f;
		for ( int i = 0; i < s.world->pieces.count; ++i )
		{
			const lpPiece* p = s.world->pieces.data + i;
			int kind = p->body >= 0 ? s.world->bodies.data[p->body].kind : -1;
			if ( ( kind == lp_kindDebris || kind == lp_kindRubble ) && p->shape->volume > biggest )
			{
				biggest = p->shape->volume;
				piece = i;
			}
		}
		ENSURE( piece >= 0 );
		uint32_t generation = s.world->pieces.data[piece].generation;
		lpPos at = lpWorld_ToWorldFrame( s.world, piece, s.world->pieces.data[piece].shape->centroid );
		lpImpactDef blow = { 0 };
		blow.point = at;
		blow.direction = (lpVec3){ 0.0f, -1.0f, 0.0f };
		blow.radius = 0.6f;
		blow.energy = 60000.0f;
		lpWorld_AddImpact( s.world, &blow );
		if ( run == 0 )
		{
			lpWorld_Pull( s.world, piece, s.world->pieces.data[piece].shape->centroid, lpOffsetPos( at, (lpVec3){ 0.0f, 3.0f, 0.0f } ),
						  40.0f, 400.0f );
		}
		Run( &s, 1 );
		ENSURE( s.world->pieces.data[piece].generation != generation ); // it fractured, and its slot went to a cell
		ENSURE( s.world->pieces.data[piece].body >= 0 );
		Run( &s, 10 );
		hashes[run] = lpWorld_Hash( s.world );
		DestroySim( &s );
	}
	ENSURE( hashes[0] == hashes[1] );
	return 0;
}

static lpCommand RoundCommand( int64_t tick, uint8_t peer, uint32_t seq, lpPos point )
{
	lpCommand c = { 0 };
	c.tick = tick;
	c.peer = peer;
	c.seq = seq;
	c.kind = lp_commandImpact;
	c.impact.def.point = point;
	c.impact.def.direction = (lpVec3){ 0.0f, 0.0f, -1.0f };
	c.impact.def.radius = 0.35f;
	c.impact.def.energy = 4000.0f;
	c.impact.def.impulse = 20.0f;
	return c;
}

// A tick's commands apply in (peer, seq) order however they were submitted, so two worlds given them in opposite orders
// agree; a late command, or one whose (tick, peer, seq) is queued already, is refused
static int TestCommandOrder( void )
{
	uint64_t hashes[2] = { 0 };
	for ( int run = 0; run < 2; ++run )
	{
		Sim s = CreateSimWorkers( lp_sceneWall, 1 );
		lpCommand rounds[3] = {
			RoundCommand( 3, 0, 0, (lpPos){ -3.0f, 1.6f, 1.0f } ), // three rounds into the pane in one tick
			RoundCommand( 3, 1, 0, (lpPos){ -2.7f, 1.4f, 1.0f } ),
			RoundCommand( 3, 0, 1, (lpPos){ -3.3f, 1.8f, 1.0f } ),
		};
		for ( int k = 0; k < 3; ++k )
		{
			ENSURE( lpWorld_Submit( s.world, rounds + ( run == 0 ? k : 2 - k ) ) );
		}
		ENSURE( lpWorld_Submit( s.world, rounds + 1 ) == false );
		Run( &s, 4 );
		int count;
		const lpCommand* applied = lpWorld_GetAppliedCommands( s.world, &count );
		ENSURE( count == 3 );
		ENSURE( applied[0].peer == 0 && applied[0].seq == 0 && applied[1].peer == 0 && applied[1].seq == 1 && applied[2].peer == 1 );
		ENSURE( lpWorld_GetStats( s.world ).commandsApplied == 3 );
		lpCommand late = RoundCommand( 2, 0, 9, (lpPos){ -3.0f, 1.6f, 1.0f } );
		ENSURE( lpWorld_Submit( s.world, &late ) == false );
		Run( &s, 20 );
		hashes[run] = lpWorld_Hash( s.world );
		DestroySim( &s );
	}
	ENSURE( hashes[0] == hashes[1] );
	return 0;
}

static lpCommand DriveCommand( int64_t tick, uint8_t peer, uint32_t seq, float throttle )
{
	lpCommand c = { 0 };
	c.tick = tick;
	c.peer = peer;
	c.seq = seq;
	c.kind = lp_commandVehicleControl;
	c.vehicleControl.vehicle = 0;
	c.vehicleControl.control.throttle = throttle;
	return c;
}

// A player's control takes a vehicle over: the scene's controls of it are dropped until that player releases it, and
// another player cannot release it. A command naming a slot whose generation moved on is dropped.
static int TestCommandReferences( void )
{
	Sim s = CreateSimWorkers( lp_sceneTrack, 1 );
	ENSURE( lpWorld_GetVehicleState( s.world, 0 ).controller == -1 );
	lpCommand player = DriveCommand( 2, 0, 0, 0.75f );
	lpCommand scene = DriveCommand( 2, LP_PEER_SCENE, 0, -1.0f );
	lpCommand sceneLater = DriveCommand( 3, LP_PEER_SCENE, 1, -1.0f );
	ENSURE( lpWorld_Submit( s.world, &scene ) && lpWorld_Submit( s.world, &player ) && lpWorld_Submit( s.world, &sceneLater ) );
	Run( &s, 3 );
	ENSURE( lpWorld_GetVehicleState( s.world, 0 ).controller == 0 );
	ENSURE( lpWorld_GetVehicleState( s.world, 0 ).control.throttle == 0.75f );
	ENSURE( lpWorld_GetStats( s.world ).commandsDropped == 1 );
	Run( &s, 1 );
	ENSURE( lpWorld_GetStats( s.world ).commandsDropped == 1 && lpWorld_GetVehicleState( s.world, 0 ).control.throttle == 0.75f );

	lpCommand release = { 0 };
	release.tick = 5;
	release.peer = 1; // not the one driving it
	release.kind = lp_commandRelease;
	release.release.vehicle = 0;
	release.release.rig = -1;
	ENSURE( lpWorld_Submit( s.world, &release ) );
	release.peer = 0;
	release.tick = 6;
	ENSURE( lpWorld_Submit( s.world, &release ) );
	lpCommand sceneAfter = DriveCommand( 7, LP_PEER_SCENE, 2, -0.5f );
	ENSURE( lpWorld_Submit( s.world, &sceneAfter ) );
	Run( &s, 2 );
	ENSURE( lpWorld_GetStats( s.world ).commandsDropped == 1 ); // peer 1's release
	Run( &s, 1 );
	ENSURE( lpWorld_GetVehicleState( s.world, 0 ).controller == -1 );
	Run( &s, 1 );
	ENSURE( lpWorld_GetVehicleState( s.world, 0 ).control.throttle == -0.5f );

	// A pull naming a piece's next generation is stale
	int piece = -1;
	for ( int i = 0; i < s.world->pieces.count && piece < 0; ++i )
	{
		piece = s.world->pieces.data[i].body >= 0 ? i : -1;
	}
	lpCommand pull = { 0 };
	pull.tick = (int64_t)lpWorld_GetTick( s.world );
	pull.kind = lp_commandPull;
	pull.pull.piece = piece;
	pull.pull.generation = s.world->pieces.data[piece].generation + 1;
	pull.pull.maxAccel = 40.0f;
	pull.pull.maxMass = 400.0f;
	ENSURE( lpWorld_Submit( s.world, &pull ) );
	Run( &s, 1 );
	ENSURE( lpWorld_GetStats( s.world ).commandsDropped == 1 && lpWorld_GetStats( s.world ).commandsApplied == 0 );
	DestroySim( &s );
	return 0;
}

// An object spawned by command from a template is the object lpCreateObject makes from the same def at that tick
static int TestTemplateSpawn( void )
{
	lpVec3 points[8];
	for ( int i = 0; i < 8; ++i )
	{
		points[i] = (lpVec3){ i & 1 ? 0.3f : -0.3f, i & 2 ? 0.25f : -0.25f, i & 4 ? 0.4f : -0.4f };
	}
	lpPartDef part = lpDefaultPartDef();
	part.points = points;
	part.pointCount = 8;
	part.material = lp_wood;
	lpObjectDef crate = lpDefaultObjectDef();
	crate.isStatic = false;
	crate.parts = &part;
	crate.partCount = 1;
	crate.transform.p = (lpPos){ 0.5f, 6.0f, 0.5f };
	crate.linearVelocity = (lpVec3){ 0.0f, -8.0f, 1.0f };
	crate.angularVelocity = (lpVec3){ 1.0f, 2.0f, 0.5f };
	uint64_t hashes[2] = { 0 };
	for ( int run = 0; run < 2; ++run )
	{
		Sim s = CreateSimWorkers( lp_scenePile, 1 );
		int template = -1;
		if ( run == 1 )
		{
			template = lpWorld_AddTemplate( s.world, &crate );
			points[0].x = 9.0f; // the world kept its own copy
		}
		Run( &s, 2 );
		if ( run == 0 )
		{
			lpCreateObject( s.world, &crate );
		}
		else
		{
			lpCommand spawn = { 0 };
			spawn.tick = 2;
			spawn.kind = lp_commandSpawn;
			spawn.spawn.templateIndex = template;
			spawn.spawn.transform = crate.transform;
			spawn.spawn.linearVelocity = crate.linearVelocity;
			spawn.spawn.angularVelocity = crate.angularVelocity;
			ENSURE( lpWorld_Submit( s.world, &spawn ) );
			points[0].x = -0.3f;
		}
		Run( &s, 60 );
		hashes[run] = lpWorld_Hash( s.world );
		DestroySim( &s );
	}
	ENSURE( hashes[0] == hashes[1] );
	return 0;
}

// The physics engine's hidden contact state, hashed per body: two worlds stepped alike agree body by body, only bodies
// with a touching contact have a sum, and summing for some bodies gives them what summing for all does
static int TestContactHash( void )
{
	uint64_t* sums[2] = { NULL, NULL };
	int counts[2] = { 0 };
	for ( int run = 0; run < 2; ++run )
	{
		Sim s = CreateSimWorkers( lp_scenePile, run == 0 ? 1 : 4 );
		Run( &s, 40 );
		counts[run] = s.world->bodies.count;
		sums[run] = calloc( (size_t)counts[run], sizeof( uint64_t ) );
		lpPhys_HashContacts( s.world->phys, NULL, 0, sums[run], counts[run] );
		if ( run == 1 )
		{
			uint8_t* odd = calloc( (size_t)counts[run], 1 );
			uint64_t* some = calloc( (size_t)counts[run], sizeof( uint64_t ) );
			for ( int i = 0; i < counts[run]; ++i )
			{
				odd[i] = (uint8_t)( i & 1 );
			}
			lpPhys_HashContacts( s.world->phys, odd, 1, some, counts[run] );
			int touching = 0;
			for ( int i = 0; i < counts[run]; ++i )
			{
				touching += sums[run][i] != 0 ? 1 : 0;
				ENSURE( some[i] == ( odd[i] ? sums[run][i] : 0 ) );
				ENSURE( sums[run][i] == 0 || s.world->bodies.data[i].kind != lp_kindStructure ); // static: no sum
			}
			printf( "  %d bodies with touching contacts\n", touching );
			ENSURE( touching > 10 );
			free( odd );
			free( some );
		}
		DestroySim( &s );
	}
	ENSURE( counts[0] == counts[1] && memcmp( sums[0], sums[1], sizeof( uint64_t ) * (size_t)counts[0] ) == 0 );
	free( sums[0] );
	free( sums[1] );
	return 0;
}

// The state hash, kept incrementally, equals a full recompute every tick in scenes that fracture, collapse, drive, walk
// and settle; and following it down (categories, buckets, elements) adds back up to it
static int TestHashIncremental( void )
{
	const int scenes[7][2] = { { lp_sceneWall, 12 }, { lp_sceneTown, 3 },  { lp_sceneKeep, 4 },		   { lp_sceneTrack, 30 },
							   { lp_sceneMech, 30 }, { lp_sceneYard, 12 }, { lp_sceneContraption, 0 } };
	for ( int k = 0; k < 7; ++k )
	{
		Sim s = CreateSimWorkers( scenes[k][0], 4 );
		char message[256];
		for ( int tick = 0; tick < 240; ++tick )
		{
			lpSceneBombard( s.world, scenes[k][0], tick, scenes[k][1] );
			lpSceneDrive( s.world, scenes[k][0], tick );
			lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
			if ( lpWorld_CheckHash( s.world, message, (int)sizeof( message ) ) == false )
			{
				printf( "  %s: %s\n", lpSceneName( scenes[k][0] ), message );
				return 1;
			}
		}
		uint64_t sums[lp_hashCategoryCount];
		lpWorld_HashCategories( s.world, sums );
		for ( int c = 0; c < lp_hashCategoryCount; ++c )
		{
			uint64_t buckets = 0, elements = 0;
			int slots = lpWorld_HashSlotCount( s.world, c );
			for ( int b = 0; b * 64 < slots; ++b )
			{
				buckets += lpWorld_HashBucket( s.world, c, b );
			}
			for ( int i = 0; i < slots; ++i )
			{
				elements += lpWorld_HashElement( s.world, c, i );
			}
			ENSURE( buckets == sums[c] && elements == sums[c] );
		}
		printf( "  %-11s 240 ticks: kept hash equals the full one every tick\n", lpSceneName( scenes[k][0] ) );
		DestroySim( &s );
	}
	return 0;
}

// Causal units: bodies that touch (but not through an anchored piece), are linked, or share a detonator are in one
// unit; the ground keeps the world from being one unit; the units' hashes and the world category add up to the whole
static int TestCausalUnits( void )
{
	const int scenes[3] = { lp_sceneYard, lp_scenePile, lp_sceneTrack };
	for ( int k = 0; k < 3; ++k )
	{
		Sim s = CreateSimWorkers( scenes[k], 1 );
		lpWorld* w = s.world;
		for ( int tick = 0; tick < 90; ++tick )
		{
			lpSceneBombard( w, scenes[k], tick, 30 );
			lpSceneDrive( w, scenes[k], tick );
			lpWorld_Step( w, 1.0f / 60.0f, 4 );
		}
		int capacity = lpWorld_GetBodyCapacity( w );
		int* units = malloc( sizeof( int ) * (size_t)capacity );
		ENSURE( units != NULL && lpWorld_GetUnits( w, units, capacity - 1 ) == -1 );
		int count = lpWorld_GetUnits( w, units, capacity );
		int* sizes = calloc( (size_t)( count > 0 ? count : 1 ), sizeof( int ) );
		ENSURE( sizes != NULL );

		// Numbered densely, in order of each unit's lowest slot; free slots in none
		int alive = 0, seen = 0;
		for ( int i = 0; i < capacity; ++i )
		{
			if ( w->bodies.data[i].alive == false )
			{
				ENSURE( units[i] == -1 );
				continue;
			}
			alive += 1;
			ENSURE( units[i] >= 0 && units[i] <= seen && units[i] < count );
			seen += units[i] == seen ? 1 : 0;
			sizes[units[i]] += 1;
		}
		ENSURE( seen == count );

		// What joins does
		int joins = 0;
		for ( int i = 0; i < w->links.count; ++i )
		{
			const lpLink* l = w->links.data + i;
			if ( l->alive && l->ends[0].piece >= 0 && l->ends[1].piece >= 0 )
			{
				ENSURE( units[w->pieces.data[l->ends[0].piece].body] == units[w->pieces.data[l->ends[1].piece].body] );
				joins += 1;
			}
		}
		for ( int i = 0; i < capacity; ++i )
		{
			const lpBody* b = w->bodies.data + i;
			if ( b->alive == false || b->kind != lp_kindDebris || LP_PHYS_NULL( b->id ) )
			{
				continue;
			}
			const lpPhysContact* contacts;
			int n = lpPhys_GetBodyContacts( w->phys, b->id, &contacts );
			for ( int c = 0; c < n; ++c )
			{
				const lpPiece* other = contacts[c].other >= 0 ? w->pieces.data + contacts[c].other : NULL;
				if ( other != NULL && other->anchored == false && w->bodies.data[other->body].kind != lp_kindRubble )
				{
					ENSURE( units[other->body] == units[i] );
					joins += 1;
				}
			}
		}

		// The ground joins nothing: it is a unit of its own
		int ground = -1, biggest = 0;
		for ( int i = 0; i < w->pieces.count && ground < 0; ++i )
		{
			ground = w->pieces.data[i].body >= 0 && w->pieces.data[i].material == lp_ground ? w->pieces.data[i].body : -1;
		}
		for ( int u = 0; u < count; ++u )
		{
			biggest = sizes[u] > biggest ? sizes[u] : biggest;
		}
		ENSURE( ground >= 0 && sizes[units[ground]] == 1 && count > 1 );

		// Every element is in one unit or none: the units' sums and the rest add up to the categories
		uint64_t* sums = malloc( sizeof( uint64_t ) * (size_t)( count > 0 ? count : 1 ) );
		ENSURE( sums != NULL );
		lpWorld_HashUnits( w, units, count, sums );
		uint64_t categories[lp_hashCategoryCount];
		lpWorld_HashCategories( w, categories );
		uint64_t whole = 0, parts = 0;
		for ( int c = 0; c < lp_hashCategoryCount; ++c )
		{
			whole += categories[c];
			for ( int i = 0; i < lpWorld_HashSlotCount( w, c ); ++i )
			{
				parts += lpWorld_GetElementUnit( w, units, c, i ) < 0 ? lpWorld_HashElement( w, c, i ) : 0;
			}
		}
		for ( int u = 0; u < count; ++u )
		{
			parts += sums[u];
		}
		ENSURE( parts == whole );
		printf( "  %-6s %4d bodies in %4d units (largest %3d), %d joins checked\n", lpSceneName( scenes[k] ), alive, count, biggest, joins );
		free( sums );
		free( sizes );
		free( units );
		DestroySim( &s );
	}
	return 0;
}

// How far one step's queries reach: an impact asked for with a 10 m radius acts within maxImpactRadius, and a command's
// ray finds a wall 200 m away but not one 300 m away (past maxRayRange), whatever range it asked for
static int TestQueryBounds( void )
{
	Sim s = CreateSim( -1 );
	lpImpactDef wide = { 0 };
	wide.point = (lpPos){ 0.0f, 0.5f, 0.0f };
	wide.direction = (lpVec3){ 0.0f, -1.0f, 0.0f };
	wide.radius = 10.0f;
	wide.energy = 1.0f;
	lpWorld_AddImpact( s.world, &wide );
	ENSURE( s.world->impacts.data[s.world->impacts.count - 1].radius == s.world->def.maxImpactRadius );
	DestroySim( &s );

	const float distances[2] = { 200.0f, 300.0f };
	for ( int k = 0; k < 2; ++k )
	{
		Sim t = CreateSim( -1 );
		lpPartDef part = lpDefaultPartDef();
		part.halfExtents = (lpVec3){ 0.5f, 2.0f, 2.0f };
		part.material = lp_stone;
		part.anchored = true;
		lpObjectDef wall = lpDefaultObjectDef();
		wall.isStatic = true;
		wall.transform.p = (lpPos){ distances[k], 2.0f, 0.0f };
		wall.parts = &part;
		wall.partCount = 1;
		lpCreateObject( t.world, &wall );
		lpCommand c = RoundCommand( (int64_t)lpWorld_GetTick( t.world ), 0, 0, (lpPos){ 0.0f, 2.0f, 0.0f } );
		c.impact.origin = (lpPos){ 0.0f, 2.0f, 0.0f };
		c.impact.def.direction = (lpVec3){ 1.0f, 0.0f, 0.0f };
		c.impact.range = 400.0f;
		ENSURE( lpWorld_Submit( t.world, &c ) );
		Run( &t, 1 );
		int impacts = lpWorld_GetStats( t.world ).impactsThisStep;
		printf( "  a ray of 400 m at a wall %.0f m away: %s\n", (double)distances[k], impacts > 0 ? "hit" : "missed" );
		ENSURE( impacts == ( k == 0 ? 1 : 0 ) );
		DestroySim( &t );
	}
	return 0;
}

static int TestWorldTables( void )
{
	lpMaterialDef materials[lp_materialCount];
	lpJointDef joints[lp_jointCount];
	memcpy( materials, lpDefaultMaterials(), sizeof( materials ) );
	memcpy( joints, lpDefaultJoints(), sizeof( joints ) );
	materials[lp_glass].breakable = false;
	joints[lp_jointMortar].tensileStrength = 1.0f;
	lpWorldDef tough = lpDefaultWorldDef();
	tough.materials = materials;
	tough.joints = joints;
	tough.workerCount = 1;
	Sim a = CreateSimDef( tough, lp_sceneWall );
	Sim b = CreateSimWorkers( lp_sceneWall, 1 );
	materials[lp_glass].breakable = true; // the world took a copy
	joints[lp_jointMortar].tensileStrength = 2.0f;
	ENSURE( lpWorld_GetMaterial( a.world, lp_glass )->breakable == false && lpWorld_GetMaterial( b.world, lp_glass )->breakable );
	ENSURE( lpWorld_GetJoint( a.world, lp_jointMortar )->tensileStrength == 1.0f );
	ENSURE( lpWorld_GetJoint( b.world, lp_jointMortar )->tensileStrength == lpDefaultJoints()[lp_jointMortar].tensileStrength );

	lpImpactDef round = { 0 };
	round.point = (lpPos){ -3.0f, 1.6f, 1.0f }; // the pane
	round.direction = (lpVec3){ 0.0f, 0.0f, -1.0f };
	round.radius = 0.35f;
	round.energy = 4000.0f;
	round.impulse = 20.0f;
	lpWorld_AddImpact( a.world, &round );
	lpWorld_AddImpact( b.world, &round );
	Run( &a, 10 );
	Run( &b, 10 );
	printf( "  a rifle round into the pane: %d glass pieces where glass is unbreakable, %d by default\n", GlassPieces( a.world ),
			GlassPieces( b.world ) );
	ENSURE( GlassPieces( a.world ) == 1 && GlassPieces( b.world ) > 10 );

	// The default joints a part asking for lp_jointAuto gets, by material
	const uint8_t expected[lp_materialCount] = {
		[lp_wood] = lp_jointNails,	   [lp_stone] = lp_jointMortar,	 [lp_brick] = lp_jointMortar,  [lp_plaster] = lp_jointMortar,
		[lp_concrete] = lp_jointMortar, [lp_glass] = lp_jointSolid,	 [lp_metal] = lp_jointSolid,	 [lp_ground] = lp_jointSolid,
		[lp_foliage] = lp_jointSolid,	[lp_sheetMetal] = lp_jointBolts, [lp_rubber] = lp_jointSolid, [lp_armor] = lp_jointSolid,
	};
	for ( int m = 0; m < lp_materialCount; ++m )
	{
		ENSURE( lpWorld_GetMaterial( b.world, m )->joint == expected[m] );
		ENSURE( lpWorld_GetMaterial( b.world, m )->cellJoint == lp_jointMortar );
	}
	DestroySim( &a );
	DestroySim( &b );

	materials[lp_stone].density = -1.0f;
	ENSURE( lpCreateWorld( &tough ) == NULL );
	return 0;
}

// Runs the contraption scene until its vial goes off (or a minute passes): the tick it went off (-1: it did not), the
// dominoes left standing, and the world's hash then
static int RunContraption( int workers, int* standing, uint64_t* hash )
{
	Sim s = CreateSimWorkers( lp_sceneContraption, workers );
	int detonator = -1;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		const lpPiece* p = s.world->pieces.data + i;
		detonator = p->body >= 0 && p->userId == lp_userContraptionVial ? p->detonator - 1 : detonator;
	}
	int fired = -1;
	for ( int tick = 0; tick < 3600 && fired < 0 && detonator >= 0; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		fired = s.world->detonators.data[detonator].armed ? -1 : tick;
	}
	// Dominoes still upright (tilted under 30 degrees)
	*standing = 0;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		const lpPiece* p = s.world->pieces.data + i;
		lpWorldTransform xf;
		if ( p->body >= 0 && p->material == lp_wood && lpWorld_GetBodyTransform( s.world, p->body, &xf ) )
		{
			*standing += lpRotateVector( xf.q, (lpVec3){ 0.0f, 1.0f, 0.0f } ).y > 0.866f ? 1 : 0;
		}
	}
	*hash = lpWorld_Hash( s.world );
	DestroySim( &s );
	return fired;
}

// A contraption left to run on its own (a domino run that tips a teetering stone onto a volatile vial) goes off on time:
// every domino falls (none stalls asleep), and the vial goes off on the same tick at 1, 4 and 8 workers, within 10% of
// when it does today (the window physics left running far away, simplified or not, must still meet)
static int TestContraptionOnTime( void )
{
	const int expected = LP_CONTRAPTION_TICK;
	int standing[3];
	uint64_t hash[3];
	int fired[3] = { RunContraption( 1, standing + 0, hash + 0 ), RunContraption( 4, standing + 1, hash + 1 ),
					 RunContraption( 8, standing + 2, hash + 2 ) };
	printf( "  the vial went off at %.2f s (tick %d; %d expected), %d dominoes left standing\n", (double)fired[0] / 60.0, fired[0],
			expected, standing[0] );
	ENSURE( fired[0] >= 0 && standing[0] == 0 );
	ENSURE( fired[1] == fired[0] && fired[2] == fired[0] && hash[1] == hash[0] && hash[2] == hash[0] );
	ENSURE( 10 * abs( fired[0] - expected ) <= expected );
	return 0;
}

// A replay script read, written and read again gives the same commands, bit for bit: a recording replays the session
// it came from. The tools expand to the ray and spawn commands they stand for.
static int TestScriptRoundTrip( void )
{
	const char* lines[] = {
		"# tick tool origin dir\n",
		"\n",
		"10 rifle 0 2.2 8 -1.5 -0.4 -11\n",
		"12 pull 1 2 3 0.25 -0.5 0.125 7\n",
		"30 drive 0 1 0 -0.25 1\n",
		"40 walk 0 0.5 0 0.1 0.3\n",
		"50 reach 0 2 1 1.5 0.25 -3\n",
		"60 grab 0 2\n",
		"70 Grenade 0 0.333333343 1e-3 0 0 -1\n",
		"74 claw 0 1 1 100 200 300\n",
		"75 impact 0 2 8 0 -0.1 -1 0.25 40000 5\n",
		"76:3 flask 0 2.2 8 0 -0.1 -1\n",
		"77:3 point 1 2 3 0 -1 0 0.5 900 2 1\n",
		"78:1 release vehicle 2\n",
	};
	lpScript a = { 0 };
	for ( int i = 0; i < (int)( sizeof( lines ) / sizeof( lines[0] ) ); ++i )
	{
		ENSURE( lpScriptParseLine( &a, lines[i] ) );
	}
	ENSURE( lpScriptParseLine( &a, "80 laser 0 0 0 0 0 1\n" ) == false );
	ENSURE( a.count == 12 );
	ENSURE( a.commands[0].kind == lp_commandImpact && a.commands[0].impact.range == 250.0f && a.commands[0].impact.def.energy == 4000.0f );
	ENSURE( a.commands[1].kind == lp_commandPull && a.commands[1].pull.piece == 7 && a.commands[1].pull.generation == LP_ANY_GENERATION );
	ENSURE( a.commands[5].kind == lp_commandClaw && a.commands[5].claw.mode == lp_clawToggle );
	ENSURE( a.commands[7].kind == lp_commandClaw && a.commands[7].seq == 7 && a.commands[8].seq == 8 );
	ENSURE( a.commands[8].kind == lp_commandImpact && a.commands[8].impact.def.energy == 40000.0f && a.commands[8].impact.def.impulse == 5.0f );
	ENSURE( a.commands[9].kind == lp_commandSpawn && a.commands[9].peer == 3 && a.commands[9].seq == 0 && a.commands[10].seq == 1 );
	ENSURE( a.commands[10].impact.range == 0.0f && a.commands[10].impact.def.explosion );
	ENSURE( a.commands[11].kind == lp_commandRelease && a.commands[11].release.vehicle == 2 && a.commands[11].release.rig == -1 );

	// Written, read back and written again: the same text (%.9g is exact, so the same fields)
	const char* paths[2] = { "lpf_test_script_a.txt", "lpf_test_script_b.txt" }; // in the working directory
	lpScript b = { 0 };
	for ( int pass = 0; pass < 2; ++pass )
	{
		const lpScript* from = pass == 0 ? &a : &b;
		FILE* f = fopen( paths[pass], "w" );
		ENSURE( f != NULL );
		for ( int i = 0; i < from->count; ++i )
		{
			ENSURE( lpScriptWrite( f, from->commands + i ) );
		}
		lpCommand scene = from->commands[0];
		scene.peer = LP_PEER_SCENE;
		ENSURE( lpScriptWrite( f, &scene ) == false ); // the scene's drivers make theirs again
		fclose( f );
		if ( pass == 0 )
		{
			ENSURE( lpScriptLoad( &b, paths[0] ) );
		}
	}
	ENSURE( b.count == a.count );
	char lineA[512], lineB[512];
	FILE* fa = fopen( paths[0], "r" );
	FILE* fb = fopen( paths[1], "r" );
	ENSURE( fa != NULL && fb != NULL );
	int count = 0;
	while ( fgets( lineA, sizeof( lineA ), fa ) )
	{
		ENSURE( fgets( lineB, sizeof( lineB ), fb ) != NULL && strcmp( lineA, lineB ) == 0 );
		count += 1;
	}
	ENSURE( fgets( lineB, sizeof( lineB ), fb ) == NULL && count == a.count );
	fclose( fa );
	fclose( fb );
	remove( paths[0] );
	remove( paths[1] );
	lpScriptFree( &a );
	lpScriptFree( &b );
	return 0;
}

// A scripted session (every tool) replays to the same state at 1 and 8 workers, tick by tick
static int TestScriptReplay( void )
{
	const char* lines[] = {
		"10 rifle   0 2.2 8   -1.5 -0.4 -11\n",
		"20 grenade 0 2.2 8    1.0 -0.6 -11\n",
		"40 hammer  1.6 1.0 -2.2  0 0 -1\n",
		"50 ball    0 2.2 8    0.2 -0.1 -1\n",
		"60 flask   0 2.2 8   -0.3 -0.05 -1\n",
		"90 cannon  0 2.2 8    0 -0.3 -11\n",
	};
	lpScript script = { 0 };
	for ( int i = 0; i < (int)( sizeof( lines ) / sizeof( lines[0] ) ); ++i )
	{
		ENSURE( lpScriptParseLine( &script, lines[i] ) );
	}
	uint64_t hashes[240];
	int pieces[2] = { 0 };
	for ( int run = 0; run < 2; ++run )
	{
		Sim s = CreateSimWorkers( lp_sceneWall, run == 0 ? 1 : 8 );
		int before = lpWorld_GetStats( s.world ).pieceCount;
		int next = 0;
		for ( int tick = 0; tick < 240; ++tick )
		{
			next = lpScriptPlay( s.world, &script, next );
			lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
			uint64_t h = lpWorld_Hash( s.world );
			if ( run == 0 )
			{
				hashes[tick] = h;
			}
			else if ( h != hashes[tick] )
			{
				printf( "  diverged at tick %d\n", tick );
				return 1;
			}
		}
		ENSURE( next == script.count );
		pieces[run] = lpWorld_GetStats( s.world ).pieceCount - before;
		ENSURE( lpWorld_Validate( s.world ) );
		DestroySim( &s );
	}
	printf( "  %d commands, %d pieces more\n", script.count, pieces[0] );
	ENSURE( pieces[0] > 20 );
	lpScriptFree( &script );
	return 0;
}

// The inspection queries agree with the stats and with each other
static int TestInspection( void )
{
	Sim s = CreateSimWorkers( lp_scenePile, 1 );
	Run( &s, 90 );
	lpStats stats = lpWorld_GetStats( s.world );
	ENSURE( lpWorld_GetTick( s.world ) == 90 );

	int kinds[5] = { 0 };
	int awake = 0;
	int contacts = 0;
	lpContactInfo found[256];
	for ( int i = 0; i < lpWorld_GetBodyCapacity( s.world ); ++i )
	{
		lpBodyInfo b = lpWorld_GetBodyInfo( s.world, i );
		if ( b.alive == false )
		{
			continue;
		}
		kinds[b.kind] += 1;
		awake += b.kind == lp_kindDebris && b.awake ? 1 : 0;
		if ( b.kind == lp_kindStructure || b.kind == lp_kindRubble )
		{
			ENSURE( lpLength( b.linearVelocity ) == 0.0f && b.awake == false );
		}
		int count = lpWorld_GetBodyContacts( s.world, i, found, 256 ); // the ground touches more: the first 256 are copied
		for ( int k = 0; k < count && k < 256; ++k )
		{
			ENSURE( lpWorld_GetPieceInfo( s.world, found[k].piece ).body == i );
			ENSURE_NEAR( lpLength( found[k].normal ), 1.0f, 1e-3f );
			ENSURE( k == 0 || found[k - 1].piece < found[k].piece ||
					( found[k - 1].piece == found[k].piece && found[k - 1].other <= found[k].other ) );
		}
		contacts += count;
	}
	ENSURE( kinds[lp_kindStructure] == stats.structureBodies && kinds[lp_kindDebris] == stats.debrisBodies );
	ENSURE( kinds[lp_kindRubble] == stats.rubbleBodies && awake == stats.awakeDebris );
	ENSURE( contacts > 0 );

	int bonds = 0;
	for ( int i = 0; i < lpWorld_GetBondCapacity( s.world ); ++i )
	{
		lpBondInfo d = lpWorld_GetBondInfo( s.world, i );
		if ( d.alive == false )
		{
			continue;
		}
		bonds += 1;
		lpPieceInfo a = lpWorld_GetPieceInfo( s.world, d.pieceA );
		lpPieceInfo b = lpWorld_GetPieceInfo( s.world, d.pieceB );
		ENSURE( d.pieceA < d.pieceB && a.body >= 0 && a.body == b.body );
		ENSURE_NEAR( lpLength( d.normal ), 1.0f, 1e-3f );
		ENSURE( d.health <= d.strength && d.area > 0.0f );
	}
	ENSURE( bonds == stats.bondCount );
	printf( "  %d bodies, %d awake, %d contact points, %d bonds\n", stats.structureBodies + stats.debrisBodies + stats.rubbleBodies,
			awake, contacts, bonds );
	DestroySim( &s );
	return 0;
}

// The JSON dump holds what the stats count
static int TestDumpWorld( void )
{
	Sim s = CreateSimWorkers( lp_sceneWall, 1 );
	Bombard( &s, 123 );
	Run( &s, 30 );
	lpStats stats = lpWorld_GetStats( s.world );
	FILE* f = fopen( "lpf_test_dump.json", "w+b" );
	ENSURE( f != NULL );
	lpDumpWorld( f, s.world );
	long size = ftell( f );
	rewind( f );
	char* text = (char*)malloc( (size_t)size + 1 );
	size_t read = fread( text, 1, (size_t)size, f );
	text[read] = 0;
	fclose( f );
	remove( "lpf_test_dump.json" );
	char expected[64];
	snprintf( expected, sizeof( expected ), "\"pieceCount\": %d,", stats.pieceCount );
	ENSURE( text[0] == '{' && strstr( text, expected ) != NULL );
	ENSURE( strstr( text, "\"bonds\": [" ) != NULL && strstr( text, "\"rigs\": [" ) != NULL );
	int pieces = 0;
	for ( const char* c = strstr( text, "\"pieces\": [" ); c != NULL && c < strstr( text, "\"bonds\": [" ); c = strstr( c + 1, "{\"index\"" ) )
	{
		pieces += *c == '{' ? 1 : 0;
	}
	printf( "  %ld bytes, %d pieces\n", size, pieces );
	ENSURE( pieces == stats.pieceCount );
	free( text );
	DestroySim( &s );
	return 0;
}

int WorldTest( void )
{
	RUN_TEST( TestRefractureBonds, MECHANISM );
	RUN_TEST( TestDetonator, OUTCOME );
	RUN_TEST( TestDetonatorIndexReuse, MECHANISM );
	RUN_TEST( TestPull, OUTCOME );
	RUN_TEST( TestPullSkipsReusedSlot, MECHANISM );
	RUN_TEST( TestCommandOrder, MECHANISM );
	RUN_TEST( TestCommandReferences, MECHANISM );
	RUN_TEST( TestTemplateSpawn, MECHANISM );
	RUN_TEST( TestContactHash, MECHANISM );
	RUN_TEST( TestHashIncremental, MECHANISM );
	RUN_TEST( TestCausalUnits, MECHANISM );
	RUN_TEST( TestWallDamage, OUTCOME );
	RUN_TEST( TestDeterminism, DETERMINISM );
	RUN_TEST( TestTwinWorlds, DETERMINISM );
	RUN_TEST( TestDesyncNamed, DETERMINISM );
	RUN_TEST( TestFpGuard, DETERMINISM );
	RUN_TEST( TestDeterminismSelfTest, DETERMINISM );
	RUN_TEST( TestSessionHandshake, MECHANISM );
	RUN_TEST( TestHouseCollapse, OUTCOME );
	RUN_TEST( TestFragmentColours, OUTCOME );
	RUN_TEST( TestBuildingGoesQuiet, OUTCOME );
	RUN_TEST( TestContraptionOnTime, OUTCOME );
	RUN_TEST( TestWorldTables, MECHANISM );
	RUN_TEST( TestQueryBounds, MECHANISM );
	RUN_TEST( TestScriptRoundTrip, MECHANISM );
	RUN_TEST( TestScriptReplay, DETERMINISM );
	RUN_TEST( TestInspection, MECHANISM );
	RUN_TEST( TestDumpWorld, MECHANISM );
	return 0;
}
