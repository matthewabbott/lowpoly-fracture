// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "test_sim.h"

#include "dump.h"
#include "script.h"

#include <stdlib.h>

// lpDeterminismSelfTest's hash on every platform (set from the reference build; CI checks every leg against it)
#define LP_SELF_TEST_HASH 0x5c6b3dd653cd0c64ull

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

static bool SameEvent( const lpScriptEvent* a, const lpScriptEvent* b )
{
	return a->tick == b->tick && a->kind == b->kind && a->origin.x == b->origin.x && a->origin.y == b->origin.y &&
		   a->origin.z == b->origin.z && a->dir.x == b->dir.x && a->dir.y == b->dir.y && a->dir.z == b->dir.z &&
		   a->index == b->index && a->limb == b->limb && a->active == b->active && a->control.throttle == b->control.throttle &&
		   a->control.brake == b->control.brake && a->control.steer == b->control.steer &&
		   a->control.handbrake == b->control.handbrake && a->walk.forward == b->walk.forward && a->walk.strafe == b->walk.strafe &&
		   a->walk.turn == b->walk.turn && a->walk.crouch == b->walk.crouch && a->radius == b->radius &&
		   a->energy == b->energy && a->impulse == b->impulse;
}

// A replay script read, written and read again gives the same events: a recording replays the session it came from
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
		"75 impact 0 2 8 0 -0.1 -1 0.25 40000 5\n",
	};
	lpScript a = { 0 };
	for ( int i = 0; i < (int)( sizeof( lines ) / sizeof( lines[0] ) ); ++i )
	{
		ENSURE( lpScriptParseLine( &a, lines[i] ) );
	}
	ENSURE( lpScriptParseLine( &a, "80 laser 0 0 0 0 0 1\n" ) == false );
	ENSURE( a.count == 8 );
	ENSURE( a.events[7].kind == lp_scriptImpact && a.events[7].energy == 40000.0f && a.events[7].impulse == 5.0f );
	ENSURE( a.events[1].kind == lp_scriptPull && a.events[1].index == 7 );
	ENSURE( a.events[6].kind == lp_scriptGrenade );

	FILE* f = fopen( "lpf_test_script.txt", "w+" ); // in the working directory: tmpfile() may want the drive's root
	ENSURE( f != NULL );
	for ( int i = 0; i < a.count; ++i )
	{
		lpScriptWrite( f, a.events + i );
	}
	rewind( f );
	lpScript b = { 0 };
	char line[256];
	while ( fgets( line, sizeof( line ), f ) )
	{
		ENSURE( lpScriptParseLine( &b, line ) );
	}
	fclose( f );
	remove( "lpf_test_script.txt" );
	ENSURE( b.count == a.count );
	for ( int i = 0; i < a.count; ++i )
	{
		ENSURE( SameEvent( a.events + i, b.events + i ) );
	}
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
		lpScriptState state = lpDefaultScriptState();
		for ( int tick = 0; tick < 240; ++tick )
		{
			lpScriptPlay( s.world, &script, tick, &state );
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
		ENSURE( state.next == script.count );
		pieces[run] = lpWorld_GetStats( s.world ).pieceCount - before;
		ENSURE( lpWorld_Validate( s.world ) );
		DestroySim( &s );
	}
	printf( "  %d events, %d pieces more\n", script.count, pieces[0] );
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
	RUN_TEST( TestWallDamage, OUTCOME );
	RUN_TEST( TestDeterminism, DETERMINISM );
	RUN_TEST( TestFpGuard, DETERMINISM );
	RUN_TEST( TestDeterminismSelfTest, DETERMINISM );
	RUN_TEST( TestHouseCollapse, OUTCOME );
	RUN_TEST( TestFragmentColours, OUTCOME );
	RUN_TEST( TestBuildingGoesQuiet, OUTCOME );
	RUN_TEST( TestContraptionOnTime, OUTCOME );
	RUN_TEST( TestWorldTables, MECHANISM );
	RUN_TEST( TestScriptRoundTrip, MECHANISM );
	RUN_TEST( TestScriptReplay, DETERMINISM );
	RUN_TEST( TestInspection, MECHANISM );
	RUN_TEST( TestDumpWorld, MECHANISM );
	return 0;
}
