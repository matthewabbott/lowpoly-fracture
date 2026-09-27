// SPDX-License-Identifier: MIT
// Stress: structures stand when they should, and fall where they are weak when they should not.

#include "test_macros.h"
#include "test_sim.h"

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

// Volume-weighted world centroid of every piece of `material`
static b3Vec3 MaterialCentroid( const lpWorld* w, int material )
{
	b3Vec3 sum = b3Vec3_zero;
	float total = 0.0f;
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		b3WorldTransform xf;
		if ( p->body < 0 || p->material != material || lpWorld_GetBodyTransform( w, p->body, &xf ) == false )
		{
			continue;
		}
		b3Vec3 c = b3ToVec3( b3TransformWorldPoint( xf, p->shape->centroid ) );
		sum = b3MulAdd( sum, p->shape->volume, c );
		total += p->shape->volume;
	}
	return total > 0.0f ? b3MulSV( 1.0f / total, sum ) : b3Vec3_zero;
}

// Every scene's structures stand on their own: nothing detaches at rest, and the stress solve settles
static int TestStructuresStand( void )
{
	for ( int scene = 0; scene < lp_sceneCount; ++scene )
	{
		Sim s = CreateSim( scene );
		Run( &s, 1 );
		float before = StructureVolume( s.world );
		int breaks = 0;
		for ( int tick = 1; tick < 240; ++tick )
		{
			lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
			breaks += tick >= 60 ? lpWorld_GetStats( s.world ).stressBreaks : 0;
		}
		lpStats st = lpWorld_GetStats( s.world );
		float after = StructureVolume( s.world );
		printf( "  %-6s structure volume %.2f -> %.2f m^3, late breaks %d, unsettled %d\n", lpSceneName( scene ),
				(double)before, (double)after, breaks, st.unsettledStructures );
		ENSURE_NEAR( after, before, 1e-4f * before + 1e-4f );
		ENSURE( breaks == 0 );
		ENSURE( st.unsettledStructures == 0 );
		DestroySim( &s );
	}
	return 0;
}

// A stone block mortared to the side of an anchored cube: the root joint's tension is 3M/(A h) = 120 kPa L^2 against
// mortar's 300 kPa, so 0.8 m stands and 2 m snaps off.
static bool CantileverFalls( float length )
{
	Sim s = CreateSim( -1 );
	lpPartDef parts[2];
	parts[0] = lpDefaultPartDef();
	parts[0].halfExtents = (b3Vec3){ 0.5f, 0.5f, 0.5f };
	parts[0].transform.p = (b3Vec3){ 0.0f, 0.5f, 0.0f };
	parts[0].anchored = true;
	parts[0].joint = lp_jointMortar;
	parts[1] = lpDefaultPartDef();
	parts[1].halfExtents = (b3Vec3){ 0.5f * length, 0.3f, 0.2f };
	parts[1].transform.p = (b3Vec3){ 0.5f + 0.5f * length, 0.6f, 0.0f };
	parts[1].joint = lp_jointMortar;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = true;
	def.parts = parts;
	def.partCount = 2;
	lpCreateObject( s.world, &def );
	float before = StructureVolume( s.world );
	Run( &s, 120 );
	bool fell = StructureVolume( s.world ) < before - 0.1f;
	DestroySim( &s );
	return fell;
}

static int TestCantileverRoot( void )
{
	bool shortFalls = CantileverFalls( 0.8f );
	bool longFalls = CantileverFalls( 2.0f );
	printf( "  0.8 m cantilever %s, 2.0 m cantilever %s\n", shortFalls ? "fell" : "stands", longFalls ? "fell" : "stands" );
	ENSURE( shortFalls == false );
	ENSURE( longFalls );
	return 0;
}

// Knock out a wedge of more than half the ring from the tower's second and third courses, like a felling cut. The
// part above has its centre of mass past the remaining support, so the dry joints on the far side open and it
// topples toward the cut instead of standing or crumbling straight down.
static int TowerWedge( int workers, int maxStressWork, float* drift, uint64_t* hash, int* peakWork )
{
	lpWorldDef ld = lpDefaultWorldDef();
	ld.workerCount = workers;
	if ( maxStressWork > 0 )
	{
		ld.maxStressWork = maxStressWork;
	}
	Sim s = CreateSimDef( ld, lp_sceneTower );
	Run( &s, 2 );
	b3Vec3 start = MaterialCentroid( s.world, lp_stone );

	// Free the wedge's pieces from their bonds; they split off as debris and are then removed
	lpBody* tower = NULL;
	int towerIndex = -1;
	for ( int i = 0; i < s.world->bodies.count; ++i )
	{
		lpBody* b = s.world->bodies.data + i;
		if ( b->alive && b->kind == lp_kindStructure && b->pieces.count > 50 )
		{
			tower = b;
			towerIndex = i;
		}
	}
	ENSURE( tower != NULL );
	int removed = 0;
	for ( int k = 0; k < tower->pieces.count; ++k )
	{
		lpPiece* p = s.world->pieces.data + tower->pieces.data[k];
		b3Vec3 c = p->shape->centroid; // the tower's body frame is the world frame shifted to (0, 0, -8)
		float angle = b3Atan2( c.x, c.z ); // 0 toward the camera (+z)
		if ( c.y > 1.4f && c.y < 4.2f && b3AbsFloat( angle ) < 1.95f )
		{
			while ( p->bonds.count > 0 )
			{
				lpBreakBond( s.world, p->bonds.data[p->bonds.count - 1] );
			}
			removed += 1;
		}
	}
	lpMarkDirty( s.world, towerIndex );
	Run( &s, 1 );
	for ( int i = 0; i < s.world->bodies.count; ++i )
	{
		lpBody* b = s.world->bodies.data + i;
		if ( b->alive && b->kind != lp_kindStructure && b->pieces.count == 1 && b->createdTick >= 2 )
		{
			lpDestroyBody( s.world, i, false );
		}
	}

	*peakWork = 0;
	for ( int tick = 0; tick < 480; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		*peakWork = s.world->stressWork > *peakWork ? s.world->stressWork : *peakWork;
	}
	b3Vec3 end = MaterialCentroid( s.world, lp_stone );
	*drift = sqrtf( ( end.x - start.x ) * ( end.x - start.x ) + ( end.z - start.z ) * ( end.z - start.z ) );
	*hash = lpWorld_Hash( s.world );
	printf( "  %d workers, stress work %d: removed %d blocks, stone centroid drifted %.2f m, peak work %d\n", workers,
			maxStressWork, removed, (double)*drift, *peakWork );
	DestroySim( &s );
	return 0;
}

static int TestTowerTopples( void )
{
	float drift1, drift4;
	uint64_t hash1, hash4;
	int work1, work4;
	ENSURE( TowerWedge( 1, 0, &drift1, &hash1, &work1 ) == 0 );
	ENSURE( TowerWedge( 4, 0, &drift4, &hash4, &work4 ) == 0 );
	ENSURE( drift1 > 1.5f );
	ENSURE( hash1 == hash4 );
	return 0;
}

// With a tiny work budget the solve is spread over many steps, the budget holds, and the tower still falls
static int TestStressBudget( void )
{
	float drift;
	uint64_t hash;
	int peak;
	const int budget = 3000;
	ENSURE( TowerWedge( 1, budget, &drift, &hash, &peak ) == 0 );
	ENSURE( drift > 1.5f );
	ENSURE( peak <= budget + 1000 ); // a single iteration may overshoot by one structure's bond count
	return 0;
}

// Rifle rounds into the brick wall break it locally; afterwards it settles and stops creaking
static int TestDamagedWallSettles( void )
{
	Sim s = CreateSim( lp_sceneWall );
	for ( int tick = 0; tick < 240; ++tick )
	{
		if ( tick < 100 && tick % 5 == 0 )
		{
			lpImpactDef im = { 0 };
			im.point = (b3Pos){ -2.0f + 0.2f * (float)( tick / 5 ), 1.0f + 0.1f * (float)( tick % 3 ), -3.85f };
			im.direction = (b3Vec3){ 0.0f, 0.0f, -1.0f };
			im.radius = 0.35f;
			im.energy = 4000.0f;
			im.impulse = 20.0f;
			lpWorld_AddImpact( s.world, &im );
		}
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
	}
	int lateBreaks = 0;
	for ( int tick = 0; tick < 120; ++tick )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		lateBreaks += lpWorld_GetStats( s.world ).stressBreaks;
	}
	lpStats st = lpWorld_GetStats( s.world );
	printf( "  late stress breaks %d, unsettled %d\n", lateBreaks, st.unsettledStructures );
	ENSURE( lateBreaks == 0 );
	ENSURE( st.unsettledStructures == 0 );
	DestroySim( &s );
	return 0;
}

// A 6 m plank on two posts: alone it stands (3 MPa at midspan against wood's 30); with a 300 kg stone on its middle
// the rigid-plank statics give about 52 MPa, so it snaps near midspan and the stone comes down. After the snap the stone
// rests on both halves as a contact load, which the solve sees.
static int BeamRun( bool loaded, float* stoneY, float* biggestWood )
{
	Sim s = CreateSim( -1 );
	lpPartDef parts[4];
	for ( int i = 0; i < 2; ++i )
	{
		parts[i] = lpDefaultPartDef();
		parts[i].halfExtents = (b3Vec3){ 0.1f, 0.5f, 0.1f };
		parts[i].transform.p = (b3Vec3){ i == 0 ? -2.8f : 2.8f, 0.5f, 0.0f };
		parts[i].material = lp_wood;
		parts[i].anchored = true;
	}
	parts[2] = lpDefaultPartDef();
	parts[2].halfExtents = (b3Vec3){ 3.0f, 0.025f, 0.1f };
	parts[2].transform.p = (b3Vec3){ 0.0f, 1.025f, 0.0f };
	parts[2].material = lp_wood;
	parts[2].grainAxis = (b3Vec3){ 1.0f, 0.0f, 0.0f };
	parts[3] = lpDefaultPartDef();
	parts[3].halfExtents = (b3Vec3){ 0.25f, 0.25f, 0.1f };
	parts[3].transform.p = (b3Vec3){ 0.0f, 1.3f, 0.0f };
	parts[3].material = lp_stone;
	parts[3].halfExtents = (b3Vec3){ 0.25f, 0.25f, 0.25f };
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = true;
	def.parts = parts;
	def.partCount = loaded ? 4 : 3;
	lpCreateObject( s.world, &def );
	Run( &s, 180 );
	*stoneY = 99.0f;
	*biggestWood = 0.0f;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		const lpPiece* piece = s.world->pieces.data + i;
		b3WorldTransform xf;
		if ( piece->body < 0 || lpWorld_GetBodyTransform( s.world, piece->body, &xf ) == false )
		{
			continue;
		}
		b3Vec3 c = b3ToVec3( b3TransformWorldPoint( xf, piece->shape->centroid ) );
		if ( piece->material == lp_stone )
		{
			*stoneY = c.y < *stoneY ? c.y : *stoneY;
		}
		if ( piece->material == lp_wood && c.y > 0.9f )
		{
			*biggestWood = piece->shape->volume > *biggestWood ? piece->shape->volume : *biggestWood;
		}
	}
	DestroySim( &s );
	return 0;
}

static int TestBeamMidspan( void )
{
	float stoneY, plankAlone, plankLoaded;
	ENSURE( BeamRun( false, &stoneY, &plankAlone ) == 0 );
	ENSURE( BeamRun( true, &stoneY, &plankLoaded ) == 0 );
	printf( "  plank alone: largest raised wood %.4f m^3; loaded: %.4f m^3, stone at y %.2f\n", (double)plankAlone,
			(double)plankLoaded, (double)stoneY );
	ENSURE( plankAlone > 0.055f ); // still the whole 0.06 m^3 plank
	ENSURE( plankLoaded < 0.04f ); // snapped (the stone resting on the halves' tips then drags them off their posts)
	ENSURE( stoneY < 0.8f );	   // the stone fell through
	return 0;
}

// A grenade into the brick wall: the wall breaks along its mortar, so every bond left between brick pieces is mortar
static int TestMasonryWallHole( void )
{
	Sim s = CreateSim( lp_sceneWall );
	lpImpactDef im = { 0 };
	im.point = (b3Pos){ 0.0f, 1.4f, -3.85f };
	im.direction = (b3Vec3){ 0.0f, 0.0f, -1.0f };
	im.radius = 1.4f;
	im.energy = 80000.0f;
	im.impulse = 12.0f;
	im.explosion = true;
	lpWorld_AddImpact( s.world, &im );
	Run( &s, 60 );
	int brickBonds = 0, mortar = 0, brickPieces = 0;
	for ( int i = 0; i < s.world->bonds.count; ++i )
	{
		const lpBond* bond = s.world->bonds.data + i;
		if ( bond->alive && s.world->pieces.data[bond->a].material == lp_brick && s.world->pieces.data[bond->b].material == lp_brick )
		{
			brickBonds += 1;
			mortar += bond->joint == lp_jointMortar ? 1 : 0;
		}
	}
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		const lpPiece* p = s.world->pieces.data + i;
		brickPieces += p->body >= 0 && p->material == lp_brick ? 1 : 0;
	}
	printf( "  %d brick pieces, %d brick bonds, %d of them mortar\n", brickPieces, brickBonds, mortar );
	ENSURE( brickPieces > 10 );
	ENSURE( brickBonds > 0 && mortar == brickBonds );
	DestroySim( &s );
	return 0;
}

int StressTest( void )
{
	RUN_TEST( TestStructuresStand );
	RUN_TEST( TestCantileverRoot );
	RUN_TEST( TestBeamMidspan );
	RUN_TEST( TestTowerTopples );
	RUN_TEST( TestStressBudget );
	RUN_TEST( TestDamagedWallSettles );
	RUN_TEST( TestMasonryWallHole );
	return 0;
}
