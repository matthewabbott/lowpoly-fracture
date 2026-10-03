// SPDX-License-Identifier: MIT
// Stress: structures stand when they should, and fall where they are weak when they should not.

#include "test_macros.h"
#include "test_sim.h"

#include <limits.h>
#include <stdlib.h>
#include <string.h>

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
// mortar's 300 kPa, so 0.8 m stands and 2 m snaps off. Gravity scale ("fairy dust") lightens what the joint carries.
static bool CantileverFalls( float length, float gravityScale )
{
	Sim s = CreateSim( -1 );
	lpPartDef parts[2];
	parts[0] = lpDefaultPartDef();
	parts[0].halfExtents = (lpVec3){ 0.5f, 0.5f, 0.5f };
	parts[0].transform.p = (lpVec3){ 0.0f, 0.5f, 0.0f };
	parts[0].anchored = true;
	parts[0].joint = lp_jointMortar;
	parts[1] = lpDefaultPartDef();
	parts[1].halfExtents = (lpVec3){ 0.5f * length, 0.3f, 0.2f };
	parts[1].transform.p = (lpVec3){ 0.5f + 0.5f * length, 0.6f, 0.0f };
	parts[1].joint = lp_jointMortar;
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = true;
	def.parts = parts;
	def.partCount = 2;
	def.gravityScale = gravityScale;
	lpCreateObject( s.world, &def );
	float before = StructureVolume( s.world );
	Run( &s, 120 );
	bool fell = StructureVolume( s.world ) < before - 0.1f;
	DestroySim( &s );
	return fell;
}

static int TestCantileverRoot( void )
{
	bool shortFalls = CantileverFalls( 0.8f, 1.0f );
	bool longFalls = CantileverFalls( 2.0f, 1.0f );
	bool dustedFalls = CantileverFalls( 2.0f, 0.1f );
	printf( "  0.8 m cantilever %s, 2.0 m cantilever %s, 2.0 m at a tenth of its weight %s\n", shortFalls ? "fell" : "stands",
			longFalls ? "fell" : "stands", dustedFalls ? "fell" : "stands" );
	ENSURE( shortFalls == false );
	ENSURE( longFalls );
	ENSURE( dustedFalls == false );
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
	lpVec3 start = MaterialCentroid( s.world, lp_stone );

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
		lpVec3 c = p->shape->centroid; // the tower's body frame is the world frame shifted to (0, 0, -8)
		float angle = lpAtan2( c.x, c.z ); // 0 toward the camera (+z)
		if ( c.y > 1.4f && c.y < 4.2f && lpAbsFloat( angle ) < 1.95f )
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
	lpVec3 end = MaterialCentroid( s.world, lp_stone );
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
	ENSURE( peak <= budget ); // structures reserve their share before anything is built
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
			im.point = (lpPos){ -2.0f + 0.2f * (float)( tick / 5 ), 1.0f + 0.1f * (float)( tick % 3 ), -3.85f };
			im.direction = (lpVec3){ 0.0f, 0.0f, -1.0f };
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
		parts[i].halfExtents = (lpVec3){ 0.1f, 0.5f, 0.1f };
		parts[i].transform.p = (lpVec3){ i == 0 ? -2.8f : 2.8f, 0.5f, 0.0f };
		parts[i].material = lp_wood;
		parts[i].anchored = true;
	}
	parts[2] = lpDefaultPartDef();
	parts[2].halfExtents = (lpVec3){ 3.0f, 0.025f, 0.1f };
	parts[2].transform.p = (lpVec3){ 0.0f, 1.025f, 0.0f };
	parts[2].material = lp_wood;
	parts[2].grainAxis = (lpVec3){ 1.0f, 0.0f, 0.0f };
	parts[3] = lpDefaultPartDef();
	parts[3].halfExtents = (lpVec3){ 0.25f, 0.25f, 0.1f };
	parts[3].transform.p = (lpVec3){ 0.0f, 1.3f, 0.0f };
	parts[3].material = lp_stone;
	parts[3].halfExtents = (lpVec3){ 0.25f, 0.25f, 0.25f };
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
		lpWorldTransform xf;
		if ( piece->body < 0 || lpWorld_GetBodyTransform( s.world, piece->body, &xf ) == false )
		{
			continue;
		}
		lpVec3 c = lpToVec3( lpTransformWorldPoint( xf, piece->shape->centroid ) );
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
	im.point = (lpPos){ 0.0f, 1.4f, -3.85f };
	im.direction = (lpVec3){ 0.0f, 0.0f, -1.0f };
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

// The ruins scene's structure whose frame sits at world x (arch -7, colonnade 0, balconies 12), not the ground
static int RuinsBody( const lpWorld* w, float x )
{
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		lpWorldTransform xf;
		if ( b->alive && b->kind == lp_kindStructure && b->pieces.count > 0 &&
			 w->pieces.data[b->pieces.data[0]].material != lp_ground && lpWorld_GetBodyTransform( w, i, &xf ) &&
			 lpAbsFloat( (float)xf.p.x - x ) < 0.01f )
		{
			return i;
		}
	}
	return -1;
}

// Volume of the pieces still in a structure body
static float BodyVolume( const lpWorld* w, int bodyIndex )
{
	float v = 0.0f;
	const lpBody* b = w->bodies.data + bodyIndex;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		v += w->pieces.data[b->pieces.data[k]].shape->volume;
	}
	return v;
}

// Knock pieces out of a structure, as if blasted away without the blast: those whose centroid (body frame) lies in
// [lo, hi] lose their bonds and anchors, split off on the next step and are removed. Returns how many.
static int KnockOut( Sim* s, int bodyIndex, lpVec3 lo, lpVec3 hi )
{
	int removed[128];
	int count = 0;
	const lpBody* body = s->world->bodies.data + bodyIndex;
	for ( int k = 0; k < body->pieces.count && count < 128; ++k )
	{
		lpPiece* p = s->world->pieces.data + body->pieces.data[k];
		lpVec3 c = p->shape->centroid;
		if ( c.x > lo.x && c.x < hi.x && c.y > lo.y && c.y < hi.y && c.z > lo.z && c.z < hi.z )
		{
			while ( p->bonds.count > 0 )
			{
				lpBreakBond( s->world, p->bonds.data[p->bonds.count - 1] );
			}
			p->anchored = false;
			removed[count++] = body->pieces.data[k];
		}
	}
	lpMarkDirty( s->world, bodyIndex );
	Run( s, 1 );
	for ( int i = 0; i < count; ++i )
	{
		int b = s->world->pieces.data[removed[i]].body;
		if ( b >= 0 && b != bodyIndex && s->world->bodies.data[b].pieces.count == 1 )
		{
			lpDestroyBody( s->world, b, false );
		}
	}
	return count;
}

// The dry-laid arch stands by compression alone; take its keystone and the span comes down, leaving the piers
static int TestArchKeystone( void )
{
	Sim s = CreateSim( lp_sceneRuins );
	Run( &s, 2 );
	int arch = RuinsBody( s.world, -7.0f );
	ENSURE( arch >= 0 );
	float whole = BodyVolume( s.world, arch );
	Run( &s, 300 );
	float stood = BodyVolume( s.world, arch );

	int removed = KnockOut( &s, arch, (lpVec3){ -0.3f, 3.0f, -1.0f }, (lpVec3){ 0.3f, 4.0f, 1.0f } );
	float before = BodyVolume( s.world, arch );
	Run( &s, 300 );
	float after = BodyVolume( s.world, arch );
	printf( "  arch volume %.3f, after 300 steps %.3f; %d keystone removed: %.3f -> %.3f m^3\n", (double)whole, (double)stood,
			removed, (double)before, (double)after );
	ENSURE_NEAR( stood, whole, 1e-4f );
	ENSURE( removed == 1 );
	ENSURE( after < 0.5f * before );
	DestroySim( &s );
	return 0;
}

// Knock out one column and the two lintels it carried come down; the other columns and lintel stand
static int TestColonnade( void )
{
	Sim s = CreateSim( lp_sceneRuins );
	Run( &s, 2 );
	int colonnade = RuinsBody( s.world, 0.0f );
	ENSURE( colonnade >= 0 );
	float column = 0.4f * 3.0f * 0.4f, lastLintel = 2.8f * 0.35f * 0.5f;
	int removed = KnockOut( &s, colonnade, (lpVec3){ 2.3f, 0.0f, -1.0f }, (lpVec3){ 2.9f, 3.0f, 1.0f } );
	Run( &s, 300 );
	float after = BodyVolume( s.world, colonnade );
	float highest = -1.0f; // the highest stone that has come loose
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		const lpPiece* p = s.world->pieces.data + i;
		lpWorldTransform xf;
		if ( p->body >= 0 && p->material == lp_stone && s.world->bodies.data[p->body].kind != lp_kindStructure &&
			 lpWorld_GetBodyTransform( s.world, p->body, &xf ) )
		{
			lpVec3 c = lpToVec3( lpTransformWorldPoint( xf, p->shape->centroid ) );
			highest = c.x > -1.0f && c.x < 9.0f && c.y > highest ? c.y : highest;
		}
	}
	printf( "  %d column removed; standing %.3f m^3 (three columns and a lintel: %.3f), highest loose stone %.2f m\n",
			removed, (double)after, (double)( 3.0f * column + lastLintel ), (double)highest );
	ENSURE( removed == 1 );
	ENSURE_NEAR( after, 3.0f * column + lastLintel, 1e-3f );
	ENSURE( highest > 0.0f && highest < 2.5f );
	DestroySim( &s );
	return 0;
}

// The structure with the most pieces
static int BiggestStructure( const lpWorld* w )
{
	int best = -1;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		if ( b->alive && b->kind == lp_kindStructure && ( best < 0 || b->pieces.count > w->bodies.data[best].pieces.count ) )
		{
			best = i;
		}
	}
	return best;
}

typedef struct KeepRun
{
	int removed;
	float before, after; // keep volume just after the knockout, and at the end
	int decided;		 // steps until the first judged solve (-1: never)
	int settled;		 // steps until nothing is left to solve or strain (-1: never)
	int breaks;			 // joints broken by stress
	uint64_t hash, solverHash;
	float stressMs; // summed over the steps
	int iterations;
	bool valid;
	int oracleSolves; // with lpWorld.stressOracle: clustered solves checked, and the worst joint error among them
	float oracleWorst;
	float oracleMeter;
	int oracleFlips, oracleJoints;
	int audits;		 // exact audits judged
	int provisional; // structures provisional at the end
} KeepRun;

// Damage the keep, then watch what the stress solve makes of it: knock out the pieces in [lo, hi] (keep frame), or, with
// an impact, blast it
typedef void KeepPrepare( lpWorld* w, int keep );

static KeepRun KeepDamageDef( lpWorldDef def, lpVec3 lo, lpVec3 hi, const lpImpactDef* impact, int steps, KeepPrepare* prepare )
{
	KeepRun r = { 0 };
	Sim s = CreateSimDef( def, lp_sceneKeep );
	Run( &s, 2 );
	int keep = BiggestStructure( s.world );
	if ( prepare != NULL )
	{
		prepare( s.world, keep );
	}
	if ( impact != NULL )
	{
		lpWorld_AddImpact( s.world, impact );
	}
	else
	{
		r.removed = KnockOut( &s, keep, lo, hi );
	}
	r.before = BodyVolume( s.world, keep );
	r.decided = -1;
	r.settled = -1;
	for ( int step = 0; step < steps; ++step )
	{
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		lpStats st = lpWorld_GetStats( s.world );
		r.breaks += st.stressBreaks;
		r.stressMs += st.stressMs;
		r.iterations += st.stressIterations;
		r.audits += st.stressAudits;
		r.provisional = st.provisionalStructures;
		const lpBody* b = s.world->bodies.data + keep;
		if ( r.decided < 0 && st.stressSolves > 0 && b->solving == false )
		{
			r.decided = step + 1;
		}
		if ( r.decided >= 0 && r.settled < 0 && st.unsettledStructures == 0 && b->dirty == false )
		{
			r.settled = step + 1;
		}
	}
	r.after = BodyVolume( s.world, keep );
	r.valid = lpWorld_Validate( s.world );
	r.oracleSolves = s.world->oracleSolves;
	r.oracleWorst = s.world->oracleWorst;
	r.oracleMeter = s.world->oracleMeter;
	r.oracleFlips = s.world->oracleFlips;
	r.oracleJoints = s.world->oracleJoints;
	r.hash = lpWorld_Hash( s.world );
	r.solverHash = lpWorld_HashStress( s.world );
	DestroySim( &s );
	return r;
}

static KeepRun KeepDamage( int workers, lpVec3 lo, lpVec3 hi, const lpImpactDef* impact, int steps )
{
	lpWorldDef def = lpDefaultWorldDef();
	def.workerCount = workers;
	return KeepDamageDef( def, lo, hi, impact, steps, NULL );
}

static lpImpactDef KeepCannon( void )
{
	lpImpactDef impact = { 0 };
	impact.point = (lpPos){ 2.0f, 6.0f, -2.5f };
	impact.direction = (lpVec3){ 0.0f, 0.0f, -1.0f };
	impact.radius = 2.0f;
	impact.energy = 250000.0f;
	impact.impulse = 18.0f;
	impact.explosion = true;
	return impact;
}


static void EnableOracle( lpWorld* w, int keep )
{
	(void)keep;
	w->stressOracle = true;
}

static void PrintKeepRun( const char* name, KeepRun r, int steps )
{
	printf( "  %s: %d pieces out, keep %.1f -> %.1f m^3, decided after %d steps, settled after %d, %d joints broke, "
			"stress %.1f ms over %d steps, %d iterations\n",
			name, r.removed, (double)r.before, (double)r.after, r.decided, r.settled, r.breaks, (double)r.stressMs, steps,
			r.iterations );
}

// Every scene is settled at load: no structure is left solving or straining, and the first step solves nothing
static int TestScenesSettle( void )
{
	for ( int scene = 0; scene < lp_sceneCount; ++scene )
	{
		Sim s = CreateSim( scene );
		lpStats loaded = lpWorld_GetStats( s.world );
		int solving = 0;
		for ( int i = 0; i < s.world->bodies.count; ++i )
		{
			const lpBody* b = s.world->bodies.data + i;
			solving += b->alive && b->kind == lp_kindStructure && ( b->solving || b->unsettled ) ? 1 : 0;
		}
		Run( &s, 1 );
		lpStats first = lpWorld_GetStats( s.world );
		printf( "  %-6s settled in %.1f ms, %d iterations; %d left solving; first step %d solves\n", lpSceneName( scene ),
				(double)loaded.settleMs, loaded.settleIterations, solving, first.stressSolves );
		ENSURE( solving == 0 );
		ENSURE( first.stressSolves == 0 && first.stressIterations == 0 );
		DestroySim( &s );
	}
	return 0;
}

// The whole ground floor of the keep's front knocked out: the wall above hangs from the corners and the cross wall,
// some joints give, and the keep stands. Prints how long the stress solve takes to decide (the milestone's target).
static const lpVec3 lp_breachLo = { -7.6f, -1.0f, 6.2f };
static const lpVec3 lp_breachHi = { 7.6f, 3.6f, 7.6f };

static int TestKeepBreach( void )
{
	const int steps = 900;
	KeepRun r = KeepDamageDef( lpDefaultWorldDef(), lp_breachLo, lp_breachHi, NULL, steps, EnableOracle );
	PrintKeepRun( "breach", r, steps );
	printf( "  reduced solves %d: worst joint off by %.3f against exact ones, %d of %d joints flipped\n", r.oracleSolves,
			(double)r.oracleWorst, r.oracleFlips, r.oracleJoints );
	ENSURE( r.valid );
	ENSURE( r.removed > 60 );
	ENSURE( r.after > 0.95f * r.before );
	ENSURE( r.decided > 0 && r.settled > 0 && r.settled < 600 ); // exact solves only: about 150 and 590 steps
	ENSURE( r.breaks > 0 && r.breaks <= 24 ); // the masonry round the breach gives (12 today, as the exact solves break)
	return 0;
}

// A cannon ball through the front wall: the cells around the hole crumble away and the keep holds. Today the solve
// takes about 1600 steps to settle under its budget (61 with an unlimited one); milestone 4 brings that down.
static int TestKeepHole( void )
{
	const int steps = 600;
	lpImpactDef impact = KeepCannon();
	KeepRun r = KeepDamageDef( lpDefaultWorldDef(), lpVec3_zero, lpVec3_zero, &impact, steps, EnableOracle );
	PrintKeepRun( "cannon hole", r, steps );
	printf( "  reduced solves %d: worst joint off by %.3f against exact ones, %d of %d joints flipped\n", r.oracleSolves,
			(double)r.oracleWorst, r.oracleFlips, r.oracleJoints );
	ENSURE( r.valid );
	ENSURE( r.after > 0.95f * r.before );
	ENSURE( r.decided > 0 && r.decided < 60 ); // exact solves only: about 100
	ENSURE( r.settled > 0 );				   // exact solves only: not within 600 steps
	ENSURE( 100 * r.oracleFlips < r.oracleJoints );
	return 0;
}

static int TestKeepDeterminism( void )
{
	KeepRun a = KeepDamage( 1, lp_breachLo, lp_breachHi, NULL, 300 );
	KeepRun b = KeepDamage( 4, lp_breachLo, lp_breachHi, NULL, 300 );
	printf( "  1 worker %016llx / %016llx, 4 workers %016llx / %016llx\n", (unsigned long long)a.hash,
			(unsigned long long)a.solverHash, (unsigned long long)b.hash, (unsigned long long)b.solverHash );
	ENSURE( a.hash == b.hash );
	ENSURE( a.solverHash == b.solverHash );
	return 0;
}

// ---- the solver on its own (solve.h): hand-built systems, no world ----

// A beam edge of a unit-square contact between node a (or the world) and node b, joined along +x
static lpStressEdge ChainEdge( int a, int b )
{
	lpStressEdge e = { 0 };
	e.a = a;
	e.b = b;
	e.bond = b;
	e.ra = (lpVec3){ 0.5f, 0.0f, 0.0f };
	e.rb = (lpVec3){ -0.5f, 0.0f, 0.0f };
	e.n = (lpVec3){ 1.0f, 0.0f, 0.0f };
	lpContactBasis( e.n, &e.t1, &e.t2 );
	e.kn = 1.0f;
	e.ks = 0.4f;
	e.kb1 = e.kb2 = 0.25f / 3.0f;
	e.kt = 0.4f * ( e.kb1 + e.kb2 );
	return e;
}

// A cantilever of unit cubes along +x from a wall at x = -0.5, node i at x = i, each weighing `weights[i]`
static void BuildChain( lpStressSystem* s, int count, const float* weights )
{
	memset( s, 0, sizeof( *s ) );
	for ( int i = 0; i < count; ++i )
	{
		lpArray_Push( s->nodes, i );
		lpArray_Push( s->edges, ChainEdge( i - 1, i ) );
	}
	s->forceScale = 1.0f;
	lpSystemResize( s );
	lpSystemFactor( s );
	lpSystemIncidence( s );
	lpVec6* x = s->vectors.data;
	for ( int i = 0; i < count; ++i )
	{
		x[i] = (lpVec6){ lpVec3_zero, lpVec3_zero };
		x[count + i] = (lpVec6){ { 0.0f, -weights[i], 0.0f }, lpVec3_zero };
	}
}

static void ChainEdgeForce( const lpStressSystem* s, const lpVec6* x, int edge, lpVec3* force, lpVec3* moment )
{
	lpEdgeForce( s->edges.data + edge, x, force, moment );
}

// Groups for the chain: nodes before `first` on their own, the rest one rigid cluster
static void ChainPartition( lpPartition* part, const lpVec3* nodeRef, int count, int first )
{
	memset( part, 0, sizeof( *part ) );
	lpArray_Reserve( part->group, count );
	part->group.count = count;
	lpVec3 sum = lpVec3_zero;
	for ( int i = 0; i < count; ++i )
	{
		part->group.data[i] = i < first ? i : first;
		if ( i < first )
		{
			lpArray_Push( part->ref, nodeRef[i] );
			lpArray_Push( part->members, 1 );
		}
		else
		{
			sum = lpAdd( sum, nodeRef[i] );
		}
	}
	lpArray_Push( part->ref, lpMulSV( 1.0f / (float)( count - first ), sum ) );
	lpArray_Push( part->members, count - first );
	part->groupCount = first + 1;
}

// A cantilever's root carries all the weight (8) and the moment of it about the wall (32); statically determinate, so a rigid
// cluster at its tip changes nothing outside the cluster, and the delta form after a load change is exact everywhere
static int TestSolveSystem( void )
{
	enum
	{
		count = 8
	};
	float weights[count];
	lpVec3 nodeRef[count];
	for ( int i = 0; i < count; ++i )
	{
		weights[i] = 1.0f;
		nodeRef[i] = (lpVec3){ (float)i, 0.0f, 0.0f };
	}
	lpStressSystem fine;
	BuildChain( &fine, count, weights );
	lpSolveState state = { 0 };
	lpSystemSolve( &fine, 1000, 1e-7, 0.05f, false, &state );
	lpVec3 force, moment;
	ChainEdgeForce( &fine, fine.vectors.data, 0, &force, &moment );
	printf( "  chain of %d: %d iterations, root force (%.4f %.4f %.4f), moment (%.4f %.4f %.4f)\n", count, state.iterations,
			(double)force.x, (double)force.y, (double)force.z, (double)moment.x, (double)moment.y, (double)moment.z );
	ENSURE( state.converged );
	ENSURE_NEAR( force.y, -8.0f, 1e-3f ); // the bond's own force: it holds the chain up by the opposite
	ENSURE_NEAR( moment.z, -32.0f, 1e-2f );

	// The same chain with nodes 3 to 7 one rigid cluster, solved for its whole solution from zero
	lpPartition part;
	ChainPartition( &part, nodeRef, count, 3 );
	lpStressSystem reduced = { 0 };
	lpSystemReduce( &fine, nodeRef, &part, &reduced );
	ENSURE( reduced.nodes.count == 4 && reduced.edges.count == 4 );
	lpVec6* y = reduced.vectors.data;
	for ( int g = 0; g < 4; ++g )
	{
		y[g] = (lpVec6){ lpVec3_zero, lpVec3_zero };
	}
	lpPartitionRestrict( &part, nodeRef, fine.vectors.data + count, count, y + 4 );
	state = (lpSolveState){ 0 };
	lpSystemSolve( &reduced, 1000, 1e-7, 0.05f, false, &state );
	lpVec6 moved[count];
	memset( moved, 0, sizeof( moved ) );
	lpPartitionProlong( &part, nodeRef, y, count, moved );
	for ( int k = 0; k < 4; ++k )
	{
		lpVec3 fk, mk, clusteredF, clusteredM;
		ChainEdgeForce( &fine, fine.vectors.data, k, &fk, &mk );
		ChainEdgeForce( &fine, moved, k, &clusteredF, &clusteredM );
		ENSURE_NEAR( clusteredF.y, fk.y, 1e-3f );
		ENSURE_NEAR( clusteredM.z, mk.z, 1e-2f );
	}

	// The delta form: a new load on node 1, corrected from the exact old solution with the tip still clustered. Every
	// edge's force matches a fresh solve under the new load, the cluster's inside ones by keeping their old forces.
	weights[1] = 3.0f;
	lpStressSystem fresh;
	BuildChain( &fresh, count, weights );
	state = (lpSolveState){ 0 };
	lpSystemSolve( &fresh, 1000, 1e-7, 0.05f, false, &state );

	lpVec6 residual[count], kx[count];
	lpSystemApply( &fine, fine.vectors.data, kx );
	for ( int i = 0; i < count; ++i )
	{
		residual[i].f = lpSub( fresh.vectors.data[count + i].f, kx[i].f );
		residual[i].t = lpSub( fresh.vectors.data[count + i].t, kx[i].t );
	}
	for ( int g = 0; g < 4; ++g )
	{
		y[g] = (lpVec6){ lpVec3_zero, lpVec3_zero };
	}
	lpPartitionRestrict( &part, nodeRef, residual, count, y + 4 );
	reduced.loadNorm2 = 0.0;
	for ( int i = 0; i < count; ++i )
	{
		reduced.loadNorm2 += (double)lpDot( fresh.vectors.data[count + i].f, fresh.vectors.data[count + i].f );
	}
	state = (lpSolveState){ 0 };
	lpSystemSolve( &reduced, 1000, 1e-7, 0.05f, false, &state );
	memcpy( moved, fine.vectors.data, sizeof( moved ) );
	lpPartitionProlong( &part, nodeRef, y, count, moved );
	float worst = 0.0f;
	for ( int k = 0; k < count; ++k )
	{
		lpVec3 fk, mk, deltaF, deltaM;
		ChainEdgeForce( &fresh, fresh.vectors.data, k, &fk, &mk );
		ChainEdgeForce( &fresh, moved, k, &deltaF, &deltaM );
		worst = lpMaxFloat( worst, lpAbsFloat( deltaF.y - fk.y ) + lpAbsFloat( deltaM.z - mk.z ) / 8.0f );
	}
	printf( "  delta form after a load change, clustered tip: %d iterations, worst edge error %.2e\n", state.iterations,
			(double)worst );
	ENSURE( state.converged );
	ENSURE( worst < 1e-3f );

	lpSystemFree( &fine );
	lpSystemFree( &fresh );
	lpSystemFree( &reduced );
	lpPartitionFree( &part );
	return 0;
}

// The reduced assembly is exactly P^T K P (on a random system and partition), and with every node its own group it
// is the fine system bit for bit
static int TestReducedAssembly( void )
{
	enum
	{
		count = 40,
		edgeCount = 120
	};
	lpRandom rng;
	lpRandom_Seed( &rng, 7, 3 );
	lpStressSystem fine = { 0 };
	lpVec3 nodeRef[count];
	for ( int i = 0; i < count; ++i )
	{
		lpArray_Push( fine.nodes, i );
		nodeRef[i] = (lpVec3){ lpRandom_Range( &rng, -3.0f, 3.0f ), lpRandom_Range( &rng, 0.0f, 6.0f ), lpRandom_Range( &rng, -3.0f, 3.0f ) };
	}
	for ( int k = 0; k < edgeCount; ++k )
	{
		lpStressEdge e = { 0 };
		e.a = k < 6 ? -1 : (int)( lpRandom_Next( &rng ) % count );
		do
		{
			e.b = (int)( lpRandom_Next( &rng ) % count );
		}
		while ( e.b == e.a );
		lpVec3 pa = e.a >= 0 ? nodeRef[e.a] : (lpVec3){ nodeRef[e.b].x, -0.5f, nodeRef[e.b].z };
		lpVec3 contact = lpLerp( pa, nodeRef[e.b], lpRandom_Range( &rng, 0.3f, 0.7f ) );
		e.bond = k;
		e.ra = lpSub( contact, pa );
		e.rb = lpSub( contact, nodeRef[e.b] );
		e.n = lpNormalize( lpSub( nodeRef[e.b], pa ) );
		lpContactBasis( e.n, &e.t1, &e.t2 );
		e.kn = lpRandom_Range( &rng, 0.2f, 2.0f );
		e.ks = 0.4f * e.kn;
		e.kb1 = lpRandom_Range( &rng, 0.02f, 0.2f );
		e.kb2 = lpRandom_Range( &rng, 0.02f, 0.2f );
		e.kt = 0.4f * ( e.kb1 + e.kb2 );
		lpArray_Push( fine.edges, e );
	}
	fine.forceScale = 1.0f;
	lpSystemResize( &fine );
	lpSystemFactor( &fine );
	lpSystemIncidence( &fine );

	// Every node its own group: the same edges and factored blocks, bit for bit
	lpPartition part = { 0 };
	for ( int i = 0; i < count; ++i )
	{
		lpArray_Push( part.group, i );
		lpArray_Push( part.ref, nodeRef[i] );
		lpArray_Push( part.members, 1 );
	}
	part.groupCount = count;
	lpStressSystem reduced = { 0 };
	lpSystemReduce( &fine, nodeRef, &part, &reduced );
	ENSURE( reduced.edges.count == fine.edges.count );
	ENSURE( memcmp( reduced.edges.data, fine.edges.data, sizeof( lpStressEdge ) * (size_t)fine.edges.count ) == 0 );
	ENSURE( memcmp( reduced.blocks.data, fine.blocks.data, sizeof( lpBlock6 ) * (size_t)count ) == 0 );

	// Random groups (a third of the nodes on their own), random reduced motions: K_r y == P^T K (P y)
	part.group.count = 0;
	part.ref.count = 0;
	part.members.count = 0;
	part.groupCount = 0;
	int groupOf[count];
	for ( int i = 0; i < count; ++i )
	{
		groupOf[i] = i % 3 == 0 ? -1 : (int)( lpRandom_Next( &rng ) % 6 );
	}
	int clusterGroup[6] = { -1, -1, -1, -1, -1, -1 };
	for ( int i = 0; i < count; ++i )
	{
		int g = groupOf[i] < 0 ? -1 : clusterGroup[groupOf[i]];
		if ( g < 0 )
		{
			g = part.groupCount++;
			lpArray_Push( part.ref, lpVec3_zero );
			lpArray_Push( part.members, 0 );
			if ( groupOf[i] >= 0 )
			{
				clusterGroup[groupOf[i]] = g;
			}
		}
		lpArray_Push( part.group, g );
		part.members.data[g] += 1;
		part.ref.data[g] = lpAdd( part.ref.data[g], nodeRef[i] );
	}
	for ( int g = 0; g < part.groupCount; ++g )
	{
		part.ref.data[g] = lpMulSV( 1.0f / (float)part.members.data[g], part.ref.data[g] );
	}
	lpSystemReduce( &fine, nodeRef, &part, &reduced );
	int m = part.groupCount;
	lpVec6 y[count], kry[count], py[count], kpy[count], ptkpy[count];
	for ( int g = 0; g < m; ++g )
	{
		y[g].f = (lpVec3){ lpRandom_Range( &rng, -1.0f, 1.0f ), lpRandom_Range( &rng, -1.0f, 1.0f ), lpRandom_Range( &rng, -1.0f, 1.0f ) };
		y[g].t = (lpVec3){ lpRandom_Range( &rng, -0.3f, 0.3f ), lpRandom_Range( &rng, -0.3f, 0.3f ), lpRandom_Range( &rng, -0.3f, 0.3f ) };
	}
	lpSystemApply( &reduced, y, kry );
	memset( py, 0, sizeof( py ) );
	lpPartitionProlong( &part, nodeRef, y, count, py );
	lpSystemApply( &fine, py, kpy );
	lpPartitionRestrict( &part, nodeRef, kpy, count, ptkpy );
	float worst = 0.0f, largest = 0.0f;
	for ( int g = 0; g < m; ++g )
	{
		worst = lpMaxFloat( worst, lpLength( lpSub( kry[g].f, ptkpy[g].f ) ) + lpLength( lpSub( kry[g].t, ptkpy[g].t ) ) );
		largest = lpMaxFloat( largest, lpLength( ptkpy[g].f ) + lpLength( ptkpy[g].t ) );
	}
	printf( "  %d nodes in %d groups, %d of %d edges left: |K_r y - P^T K P y| %.2e of %.2e\n", count, m, reduced.edges.count,
			fine.edges.count, (double)worst, (double)largest );
	ENSURE( m < count && reduced.edges.count < fine.edges.count );
	ENSURE( worst <= 1e-5f * largest );

	lpSystemFree( &fine );
	lpSystemFree( &reduced );
	lpPartitionFree( &part );
	return 0;
}

// A few stones knocked out of the keep's upper front wall, solved exactly (no clusters) and as a correction with the
// keep's lightly loaded parts moving as rigid clusters: the clustered solve decides sooner, each of its judgements is
// checked against an exact solve of the same change, and the same joints give
static const lpVec3 lp_localLo = { 2.9f, 7.1f, 6.2f };
static const lpVec3 lp_localHi = { 5.1f, 8.5f, 7.6f };

static int TestKeepLocalHit( void )
{
	lpWorldDef def = lpDefaultWorldDef();
	def.stressLargeNodes = 1 << 30;
	KeepRun fine = KeepDamageDef( def, lp_localLo, lp_localHi, NULL, 300, NULL );
	def.stressLargeNodes = lpDefaultWorldDef().stressLargeNodes;
	KeepRun clustered = KeepDamageDef( def, lp_localLo, lp_localHi, NULL, 300, EnableOracle );
	PrintKeepRun( "local hit, exact", fine, 300 );
	PrintKeepRun( "local hit, clustered", clustered, 300 );
	printf( "  clustered solves %d, worst joint off by %.3f against the exact solve (the meter read %.3f)\n", clustered.oracleSolves,
			(double)clustered.oracleWorst, (double)clustered.oracleMeter );
	ENSURE( fine.valid && clustered.valid );
	ENSURE( clustered.oracleSolves > 0 );
	ENSURE( clustered.decided > 0 && 2 * clustered.decided < fine.decided );
	ENSURE( clustered.oracleWorst < 0.3f );
	ENSURE( clustered.breaks == fine.breaks );
	return 0;
}

// The stress tests of small structures again, with everything past 4 pieces clustered (the arch, the colonnade, the
// tower, the walls and the beam are all solved as corrections on rigid clusters after their first exact solve), and
// each reduced solve checked against an exact one: the same things stand and fall, and the joints read close
static int TestDriftSmallStructures( void )
{
	lp_testLargeNodes = 4;
	lp_testOracleWorst = 0.0f;
	lp_testOracleSolves = 0;
	lp_testOracleFlips = 0;
	lp_testOracleJoints = 0;
	int failed = 0;
	int ( *tests[] )( void ) = { TestArchKeystone, TestColonnade, TestTowerTopples, TestDamagedWallSettles, TestBeamMidspan, TestMasonryWallHole };
	const char* names[] = { "arch", "colonnade", "tower", "wall", "beam", "masonry" };
	for ( int i = 0; i < 6; ++i )
	{
		float worst = lp_testOracleWorst;
		int solves = lp_testOracleSolves, flips = lp_testOracleFlips;
		lp_testOracleWorst = 0.0f;
		failed += tests[i]();
		printf( "  %s: %d reduced solves, worst joint within twice its limit off by %.3f, %d joints flipped\n", names[i],
				lp_testOracleSolves - solves, (double)lp_testOracleWorst, lp_testOracleFlips - flips );
		lp_testOracleWorst = lpMaxFloat( worst, lp_testOracleWorst );
	}
	lp_testLargeNodes = 0;
	printf( "  %d reduced solves checked, worst joint off by %.3f, %d of %d joints flipped\n", lp_testOracleSolves,
			(double)lp_testOracleWorst, lp_testOracleFlips, lp_testOracleJoints );
	ENSURE( failed == 0 );
	ENSURE( lp_testOracleSolves > 0 );
	ENSURE( lp_testOracleWorst < 1.0f ); // the dry-stacked tower: the slightest tension reads high
	ENSURE( 100 * lp_testOracleFlips < lp_testOracleJoints );
	return 0;
}

typedef struct FireRun
{
	int judged, audits; // over the fire and the calm after
	int judgedUnderFire;
	int provisionalAfterFire, provisionalAtEnd, backlogAtEnd;
	float volume; // of the keep at the end
	uint64_t hash;
} FireRun;

// The keep under the bench's bombardment (a shot every 12 ticks) for 120 ticks, then 240 calm ones
static FireRun KeepUnderFire( lpWorldDef def )
{
	FireRun r = { 0 };
	Sim s = CreateSimDef( def, lp_sceneKeep );
	int keep = BiggestStructure( s.world );
	for ( int tick = 0; tick < 360; ++tick )
	{
		if ( tick < 120 )
		{
			lpSceneBombard( s.world, lp_sceneKeep, tick, 12 );
		}
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		lpStats st = lpWorld_GetStats( s.world );
		r.judged += st.stressJudged;
		r.judgedUnderFire += tick < 120 ? st.stressJudged : 0;
		r.audits += st.stressAudits;
		if ( tick == 119 )
		{
			r.provisionalAfterFire = st.provisionalStructures;
		}
		r.provisionalAtEnd = st.provisionalStructures;
		r.backlogAtEnd = st.auditBacklog;
	}
	r.volume = BodyVolume( s.world, keep );
	r.hash = lpWorld_Hash( s.world );
	DestroySim( &s );
	return r;
}

// Under sustained fire the keep used to go unjudged: each shot restarted a solve too big to finish between shots. Now
// it is judged, on its reduced system with the whole step's budget when it solves alone, and the same at 1 and 4
// workers. (With a quarter of the budget it is judged less often, and what the judgements break comes down later.)
static int TestKeepUnderFire( void )
{
	lpWorldDef def = lpDefaultWorldDef();
	FireRun full = KeepUnderFire( def );
	def.maxStressWork /= 4;
	def.maxStressStructureWork /= 4;
	FireRun pressed = KeepUnderFire( def );
	printf( "  full budget: %d judged (%d under fire), %d audits; provisional %d after the fire, %d at the end; keep %.1f m^3\n",
			full.judged, full.judgedUnderFire, full.audits, full.provisionalAfterFire, full.provisionalAtEnd, (double)full.volume );
	printf( "  a quarter:   %d judged (%d under fire), %d audits; provisional %d after the fire, %d at the end; keep %.1f m^3\n",
			pressed.judged, pressed.judgedUnderFire, pressed.audits, pressed.provisionalAfterFire, pressed.provisionalAtEnd,
			(double)pressed.volume );
	ENSURE( full.judgedUnderFire >= 5 && pressed.judgedUnderFire > 0 );

	def.workerCount = 4;
	FireRun parallel = KeepUnderFire( def );
	ENSURE( parallel.hash == pressed.hash );
	return 0;
}

// A provisional judgement is audited once things are calm: a local hit is judged on the keep's reduced system, and
// half a second after it settles an exact solve confirms it and forms the clusters again
static int TestKeepAudit( void )
{
	KeepRun r = KeepDamageDef( lpDefaultWorldDef(), lp_localLo, lp_localHi, NULL, 300, NULL );
	printf( "  local hit: decided after %d steps, %d audits, provisional at the end %d\n", r.decided, r.audits, r.provisional );
	ENSURE( r.decided > 0 && r.decided < 10 );
	ENSURE( r.audits == 1 && r.provisional == 0 );
	return 0;
}

// ---- inertia relief: stress on moving bodies ----

// A wooden beam of `parts` equal parts along x (solid joints between them), `length` long, 0.1 m square; moving, its
// stress solved when asked, or a structure with its middle part anchored
static int AddBeam( Sim* s, lpVec3 center, float length, int parts, bool isStatic, bool anchorMiddle )
{
	lpPartDef defs[8];
	float each = length / (float)parts;
	for ( int k = 0; k < parts; ++k )
	{
		defs[k] = lpDefaultPartDef();
		defs[k].halfExtents = (lpVec3){ 0.5f * each, 0.05f, 0.05f };
		defs[k].transform.p = (lpVec3){ -0.5f * length + ( (float)k + 0.5f ) * each, 0.0f, 0.0f };
		defs[k].material = lp_wood;
		defs[k].joint = lp_jointSolid;
		defs[k].grainAxis = (lpVec3){ 1.0f, 0.0f, 0.0f };
		defs[k].anchored = anchorMiddle && k == parts / 2;
	}
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = isStatic;
	def.solveStress = isStatic == false;
	def.transform.p = (lpPos){ center.x, center.y, center.z };
	def.parts = defs;
	def.partCount = parts;
	return lpCreateObject( s->world, &def );
}

// Asks for a moving body's stress check and runs the step that does it
static void CheckNow( Sim* s, int body )
{
	lpRequestStressCheck( s->world, body, false );
	Run( s, 1 );
}

static float PeakRho( const Sim* s, int body )
{
	float peak = 0.0f;
	const lpBody* b = s->world->bodies.data + body;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		const lpPiece* p = s->world->pieces.data + b->pieces.data[k];
		for ( int n = 0; n < p->bonds.count; ++n )
		{
			peak = fmaxf( peak, s->world->bonds.data[p->bonds.data[n]].rho );
		}
	}
	return peak;
}

// The relieved loads balance: weight and contacts less the body's acceleration sum to about nothing, in force and
// torque, so the pin carries nothing
static int TestReliefBalances( void )
{
	Sim s = CreateSim( -1 );
	int body = AddBeam( &s, (lpVec3){ 0.0f, 0.3f, 0.0f }, 3.0f, 3, false, false );
	lpWorld_SetGravityScale( s.world, body, 1.0f );
	Run( &s, 5 );
	CheckNow( &s, body );
	const lpBody* b = s.world->bodies.data + body;
	lpVec3 g = lpInvRotateVector( lpPhys_GetTransform( s.world->phys, b->id ).q, (lpVec3){ 0.0f, -10.0f, 0.0f } );
	lpVec3 force = lpVec3_zero, torque = lpVec3_zero;
	float weight = 0.0f;
	for ( int k = 0; k < b->pieces.count; ++k )
	{
		const lpPiece* p = s.world->pieces.data + b->pieces.data[k];
		float m = p->shape->volume * lpWorld_GetMaterial( s.world, p->material )->density;
		lpVec3 r = lpSub( p->shape->centroid, b->reliefCenter );
		lpVec3 accel = lpAdd( lpAdd( b->reliefAccel, lpCross( b->reliefAlpha, r ) ), lpCross( b->reliefOmega, lpCross( b->reliefOmega, r ) ) );
		lpVec3 f = lpSub( lpAdd( p->stressLoad.f, lpMulSV( m, g ) ), lpMulSV( m, accel ) );
		float own = m * lpCbrt( p->shape->volume ) * lpCbrt( p->shape->volume ) / 6.0f;
		lpVec3 t = lpSub( p->stressLoad.t, lpMulSV( own, b->reliefAlpha ) );
		force = lpAdd( force, f );
		torque = lpAdd( torque, lpAdd( lpCross( r, f ), t ) );
		weight += m * 10.0f;
	}
	printf( "  pin piece %d; unbalanced force %.2e and torque %.2e N*m against a weight of %.0f N, acceleration %.3f m/s^2\n",
			b->stressPin, lpLength( force ), lpLength( torque ), weight, lpLength( b->reliefAccel ) );
	ENSURE( b->stressPin >= 0 && lpLength( force ) < 1e-3f * weight && lpLength( torque ) < 1e-3f * weight );
	DestroySim( &s );
	return 0;
}

// In free fall nothing carries anything: gravity accelerates every piece alike
static int TestReliefFreeFall( void )
{
	Sim s = CreateSim( -1 );
	int body = AddBeam( &s, (lpVec3){ 0.0f, 20.0f, 0.0f }, 3.0f, 3, false, false );
	lpPhys_SetAngularVelocity( s.world->phys, s.world->bodies.data[body].id, (lpVec3){ 0.0f, 0.0f, 0.5f } );
	Run( &s, 5 );
	CheckNow( &s, body );
	float peak = PeakRho( &s, body );
	printf( "  falling and turning: peak joint utilization %.2e\n", peak );
	ENSURE( peak < 1e-3f );
	DestroySim( &s );
	return 0;
}

// A moving beam balanced on a ridge under its middle part is loaded like the same beam with that part anchored: its
// arms hang from it either way
static int TestReliefMatchesSupported( void )
{
	Sim s = CreateSim( -1 );
	lpPartDef ridge = lpDefaultPartDef();
	ridge.halfExtents = (lpVec3){ 0.4f, 0.5f, 0.3f };
	ridge.anchored = true;
	lpObjectDef def = lpDefaultObjectDef();
	def.transform.p = (lpPos){ 0.0f, 0.5f, 0.0f };
	def.parts = &ridge;
	def.partCount = 1;
	lpCreateObject( s.world, &def );
	lpWorld_SettleStructures( s.world );
	int moving = AddBeam( &s, (lpVec3){ 0.0f, 1.05f, 0.0f }, 5.0f, 5, false, false );
	Run( &s, 20 );
	CheckNow( &s, moving );
	float relieved = PeakRho( &s, moving );
	DestroySim( &s );

	Sim t = CreateSim( -1 );
	int fixed = AddBeam( &t, (lpVec3){ 0.0f, 1.05f, 0.0f }, 5.0f, 5, true, true );
	lpWorld_SettleStructures( t.world );
	float anchored = PeakRho( &t, fixed );
	printf( "  peak joint utilization: balanced on a ridge %.4f, its middle anchored %.4f\n", relieved, anchored );
	ENSURE( anchored > 0.0f && fabsf( relieved - anchored ) < 0.1f * anchored );
	DestroySim( &t );
	return 0;
}

// Landing across a ridge bends a beam: from 4 m it snaps over the ridge, from 0.3 m it holds
static int TestReliefLandingSnaps( void )
{
	float drops[2] = { 0.3f, 4.0f };
	int bodies[2];
	for ( int k = 0; k < 2; ++k )
	{
		Sim s = CreateSim( -1 );
		lpPartDef ridge = lpDefaultPartDef();
		ridge.halfExtents = (lpVec3){ 0.1f, 0.5f, 0.5f };
		ridge.anchored = true;
		lpObjectDef def = lpDefaultObjectDef();
		def.transform.p = (lpPos){ 0.0f, 0.5f, 0.0f };
		def.parts = &ridge;
		def.partCount = 1;
		lpCreateObject( s.world, &def );
		lpWorld_SettleStructures( s.world );
		int beam = AddBeam( &s, (lpVec3){ 0.0f, 1.05f + drops[k], 0.0f }, 4.0f, 2, false, false );
		int breaks = 0;
		for ( int t = 0; t < 90; ++t )
		{
			Run( &s, 1 );
			breaks += lpWorld_GetStats( s.world ).stressBreaks;
		}
		bodies[k] = 0;
		for ( int i = 0; i < s.world->bodies.count; ++i )
		{
			const lpBody* b = s.world->bodies.data + i;
			bodies[k] += b->alive && b->kind != lp_kindStructure && b->pieces.count > 0 &&
								 s.world->pieces.data[b->pieces.data[0]].material == lp_wood
							 ? 1
							 : 0;
		}
		printf( "  %.1f m drop onto a ridge: %d joints broke, the beam in %d bodies\n", drops[k], breaks, bodies[k] );
		ENSURE( lpWorld_Validate( s.world ) );
		(void)beam;
		DestroySim( &s );
	}
	ENSURE( bodies[0] == 1 && bodies[1] >= 2 );
	return 0;
}

// A car into a wall at 30 m/s tears engine blocks off their mounts with the deceleration; at 10 m/s they hold
static int CrashEngine( float speed, int workers, uint64_t* hash )
{
	Sim s = CreateSimWorkers( -1, workers );
	lpPartDef wall = lpDefaultPartDef();
	wall.halfExtents = (lpVec3){ 4.0f, 0.9f, 0.15f };
	wall.material = lp_brick;
	wall.anchored = true;
	lpObjectDef def = lpDefaultObjectDef();
	def.transform.p = (lpPos){ 0.0f, 0.9f, 0.15f };
	def.parts = &wall;
	def.partCount = 1;
	lpCreateObject( s.world, &def );
	lpWorld_SettleStructures( s.world );
	int vehicle = lpAddCar( s.world, (lpVec3){ 0.0f, 0.0f, -8.0f }, 0.0f, 0 );
	int car = lpWorld_GetVehicleState( s.world, vehicle ).body;
	lpPhys_SetLinearVelocity( s.world->phys, s.world->bodies.data[car].id, (lpVec3){ 0.0f, 0.0f, speed } );
	Run( &s, 60 );
	car = lpWorld_GetVehicleState( s.world, vehicle ).body;
	int off = 0;
	for ( int i = 0; i < s.world->pieces.count; ++i )
	{
		const lpPiece* p = s.world->pieces.data + i;
		off += p->body >= 0 && p->tag == lp_tagEngine && p->body != car ? 1 : 0;
	}
	if ( hash != NULL )
	{
		*hash = lpWorld_Hash( s.world );
	}
	DestroySim( &s );
	return off;
}

static int TestCrashTearsEngine( void )
{
	int slow = CrashEngine( 10.0f, 1, NULL );
	int fast = CrashEngine( 30.0f, 1, NULL );
	printf( "  engine pieces torn off the car: %d at 10 m/s, %d at 30 m/s\n", slow, fast );
	ENSURE( slow == 0 && fast >= 1 );
	uint64_t a, b, c;
	CrashEngine( 30.0f, 1, &a );
	CrashEngine( 30.0f, 4, &b );
	CrashEngine( 30.0f, 8, &c );
	ENSURE( a == b && a == c );
	return 0;
}

// The highest point of any stone piece's centroid
static float StoneTop( const lpWorld* w )
{
	float top = -1.0e9f;
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		lpWorldTransform xf;
		if ( p->body >= 0 && p->material == lp_stone && lpWorld_GetBodyTransform( w, p->body, &xf ) )
		{
			float y = (float)lpTransformWorldPoint( xf, p->shape->centroid ).y;
			top = y > top ? y : top;
		}
	}
	return top;
}

// Struck with a felling cut (scripts/tower_topple.txt: cannon shots round the front of its second course), the 13 m
// stone tower goes over toward the cut and comes down: its top ends low, nothing is left creeping or perched, and no
// fragment flies faster than the blasts and the fall can throw it (a spinning body's fragments once left at hundreds of
// m/s). Dry-laid, it falls as a shower of blocks rather than as one column.
static int TestTowerFelled( void )
{
	Sim s = CreateSimWorkers( lp_sceneTower, 1 );
	lpScript script = { 0 };
	ENSURE( LoadRepoScript( &script, "tower_topple.txt" ) );
	int next = 0;
	lpVec3 start = MaterialCentroid( s.world, lp_stone );
	float fastest = 0.0f;
	float restless = 0.0f;
	for ( int tick = 0; tick < 900; ++tick )
	{
		next = lpScriptPlay( s.world, &script, next );
		lpWorld_Step( s.world, 1.0f / 60.0f, 4 );
		float speed = MaxBodySpeed( s.world );
		fastest = speed > fastest ? speed : fastest;
		restless = tick >= 840 && speed > restless ? speed : restless;
		if ( tick == 600 )
		{
			lpVec3 c = MaterialCentroid( s.world, lp_stone );
			float top = StoneTop( s.world );
			printf( "  after 10 s: the stone's centroid moved (%.2f %.2f %.2f), its top at %.2f m\n", (double)( c.x - start.x ),
					(double)( c.y - start.y ), (double)( c.z - start.z ), (double)top );
			ENSURE( c.z - start.z > 2.5f );								 // over, toward the cut (+z)
			ENSURE( lpAbsFloat( c.x - start.x ) < 0.6f * ( c.z - start.z ) ); // and not off to one side
			ENSURE( top < 5.0f );
		}
	}
	printf( "  the fastest body all along %.1f m/s; in the last second %.2f m/s\n", (double)fastest, (double)restless );
	ENSURE( fastest < 25.0f ); // a cannon's push is 18 m/s, a 13 m fall about 16
	ENSURE( restless < 0.5f );
	ENSURE( lpWorld_Validate( s.world ) );
	lpScriptFree( &script );
	DestroySim( &s );
	return 0;
}

// A wall `columns` blocks long and `rows` high, settled; returns its body. Stone, its bottom row anchored; or a bridge:
// welded steel, held only at its two ends
static int AddLongWall( Sim* s, int columns, int rows, bool bridge )
{
	int count = columns * rows;
	lpPartDef* defs = malloc( sizeof( lpPartDef ) * (size_t)count );
	for ( int r = 0; r < rows; ++r )
	{
		for ( int c = 0; c < columns; ++c )
		{
			lpPartDef* d = defs + r * columns + c;
			*d = lpDefaultPartDef();
			d->halfExtents = (lpVec3){ 0.25f, 0.25f, 0.25f };
			d->transform.p = (lpVec3){ -0.25f * (float)columns + 0.5f * (float)c + 0.25f, 0.25f + 0.5f * (float)r, 0.0f };
			d->material = bridge ? lp_armor : lp_stone;
			d->joint = bridge ? lp_jointSolid : lp_jointAuto;
			d->anchored = bridge ? c == 0 || c == columns - 1 : r == 0;
		}
	}
	lpObjectDef def = lpDefaultObjectDef();
	def.isStatic = true;
	def.parts = defs;
	def.partCount = count;
	int body = lpCreateObject( s->world, &def );
	free( defs );
	lpWorld_SettleStructures( s->world );
	return body;
}

// Bonds from the pieces `from` (count of them) to every piece of the body, through free pieces only (fixed ones carry
// nothing across, as in the solve); INT_MAX where none reaches
static void FreeHops( const lpWorld* w, int body, const int* from, int count, int* hops )
{
	const lpBody* b = w->bodies.data + body;
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		hops[i] = INT_MAX;
	}
	int* queue = malloc( sizeof( int ) * (size_t)w->pieces.count );
	int head = 0, tail = 0;
	for ( int k = 0; k < count; ++k )
	{
		hops[from[k]] = 0;
		queue[tail++] = from[k];
	}
	while ( head < tail )
	{
		int pi = queue[head++];
		const lpPiece* p = w->pieces.data + pi;
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			const lpBond* bond = w->bonds.data + p->bonds.data[k];
			int other = bond->a == pi ? bond->b : bond->a;
			if ( hops[other] == INT_MAX && w->pieces.data[other].anchored == false && w->pieces.data[other].body == body )
			{
				hops[other] = hops[pi] + 1;
				queue[tail++] = other;
			}
		}
	}
	free( queue );
	(void)b;
}

static bool SameVec6( lpVec6 a, lpVec6 b )
{
	return memcmp( &a, &b, sizeof( lpVec6 ) ) == 0;
}

// The finite speed of propagation (stressHopsPerTick): two settled bridges 60 blocks long, held at their ends, a joint
// near one end broken in the second (the whole span bends differently). Step by step, every piece whose stress state
// differs between them lies within H d bonds of the break after d steps (and the held nodes just past the region,
// whose joints and residuals a judgement writes), and the difference reaches past H (d - 1): the cone is bounded, and
// the bound is tight.
static int TestStressCone( void )
{
	enum
	{
		hopsPerTick = 4,
		columns = 60
	};
	lpWorldDef def = lpDefaultWorldDef();
	def.stressHopsPerTick = hopsPerTick;
	Sim a = CreateSimDef( def, -1 );
	Sim b = CreateSimDef( def, -1 );
	int wallA = AddLongWall( &a, columns, 2, true );
	int wallB = AddLongWall( &b, columns, 2, true );
	Run( &a, 30 );
	Run( &b, 30 );
	ENSURE( wallA == wallB && lpWorld_Hash( a.world ) == lpWorld_Hash( b.world ) );

	// The joint between the second and third blocks of the top row
	const lpBody* body = b.world->bodies.data + wallB;
	int top[2] = { -1, -1 }, broken = -1;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		const lpPiece* p = b.world->pieces.data + body->pieces.data[i];
		lpVec3 c = p->shape->centroid;
		top[0] = c.y > 0.5f && c.x > -14.5f && c.x < -14.0f ? body->pieces.data[i] : top[0];
	}
	ENSURE( top[0] >= 0 );
	const lpPiece* first = b.world->pieces.data + top[0];
	for ( int k = 0; k < first->bonds.count && broken < 0; ++k )
	{
		const lpBond* bond = b.world->bonds.data + first->bonds.data[k];
		int other = bond->a == top[0] ? bond->b : bond->a;
		const lpVec3 c = b.world->pieces.data[other].shape->centroid;
		if ( c.y > 0.5f && c.x > b.world->pieces.data[top[0]].shape->centroid.x )
		{
			broken = first->bonds.data[k];
			top[1] = other;
		}
	}
	ENSURE( broken >= 0 );
	int* hops = malloc( sizeof( int ) * (size_t)b.world->pieces.count );
	FreeHops( b.world, wallB, top, 2, hops );
	lpBreakBond( b.world, broken );
	lpMarkDirty( b.world, wallB );

	int reached = 0;
	for ( int d = 1; d <= 12; ++d )
	{
		Run( &a, 1 );
		Run( &b, 1 );
		int farthest = -1, differing = 0;
		for ( int i = 0; i < b.world->pieces.count; ++i )
		{
			const lpPiece* pa = a.world->pieces.data + i;
			const lpPiece* pb = b.world->pieces.data + i;
			if ( pb->body != wallB || pa->body != wallA || pb->anchored )
			{
				continue; // an anchored piece's joints are compared from the free piece across them
			}
			bool same = SameVec6( pa->stressX, pb->stressX ) && SameVec6( pa->stressResidual, pb->stressResidual ) &&
						pa->inFront == pb->inFront && pa->slenderRho == pb->slenderRho;
			for ( int k = 0; k < pb->bonds.count && same; ++k )
			{
				const lpBond* x = a.world->bonds.data + pb->bonds.data[k];
				const lpBond* y = b.world->bonds.data + pb->bonds.data[k];
				same = x->rho == y->rho && x->strain == y->strain;
			}
			if ( same == false )
			{
				differing += 1;
				ENSURE( hops[i] != INT_MAX );
				farthest = hops[i] > farthest ? hops[i] : farthest;
			}
		}
		printf( "  step %2d: %3d pieces differ, the farthest %2d bonds from the break (bound %d)\n", d, differing, farthest, hopsPerTick * d );
		ENSURE( farthest <= hopsPerTick * d + 1 ); // the region, and the held nodes just past it (their joints and residuals)
		if ( hopsPerTick * d < columns - 4 )
		{
			ENSURE( farthest > hopsPerTick * ( d - 1 ) ); // tight: it does travel that fast
		}
		reached = farthest;
	}
	ENSURE( reached >= 40 );
	free( hops );
	DestroySim( &a );
	DestroySim( &b );
	return 0;
}

// The drift guard: a load that crept away from what its structure's last judgement solved for, by less than the
// threshold at each look but past it in all, is a change: when the structure solves again (a joint breaks at its far
// end), the region takes in that piece too. Below the threshold it is left alone.
static int TestStressDriftGuard( void )
{
	for ( int run = 0; run < 2; ++run )
	{
		Sim s = CreateSim( -1 );
		int wall = AddLongWall( &s, 40, 3, false );
		Run( &s, 10 );
		lpBody* body = s.world->bodies.data + wall;
		int far = -1, near = -1;
		for ( int i = 0; i < body->pieces.count; ++i )
		{
			const lpPiece* p = s.world->pieces.data + body->pieces.data[i];
			far = p->shape->centroid.y > 1.0f && p->shape->centroid.x > 9.5f ? body->pieces.data[i] : far;
			near = p->shape->centroid.y > 1.0f && p->shape->centroid.x < -9.5f ? body->pieces.data[i] : near;
		}
		ENSURE( far >= 0 && near >= 0 && body->solving == false );
		lpPiece* p = s.world->pieces.data + far;
		float weight = p->shape->volume * lpMaterial( s.world, p->material )->density * lpLength( s.world->def.gravity );
		// What the last judgement solved for differs from the load now by 3% of the piece's weight (drift), or by 1%
		p->acceptedLoad.f.y = p->stressLoad.f.y + ( run == 0 ? 0.03f : 0.01f ) * weight;
		uint32_t accepted = p->accepted;
		const lpPiece* n = s.world->pieces.data + near;
		lpBreakBond( s.world, n->bonds.data[n->bonds.count - 1] );
		lpMarkDirty( s.world, wall );
		for ( int tick = 0; tick < 30 && ( tick == 0 || body->solving ); ++tick )
		{
			Run( &s, 1 );
		}
		ENSURE( body->solving == false );
		bool seeded = p->accepted != accepted;
		printf( "  drift of %s of its weight: %s\n", run == 0 ? "3%" : "1%", seeded ? "a change, solved" : "not a change" );
		ENSURE( seeded == ( run == 0 ) );
		ENSURE( run == 1 || SameVec6( p->acceptedLoad, p->stressLoad ) );
		DestroySim( &s );
	}
	return 0;
}

// Joints creak each on their own: one over its limit keeps straining while a change far away is solved (a region
// solve growing across a bridge for many steps), instead of waiting for that solve's judgement
static int TestCreakDuringFarSolve( void )
{
	lpWorldDef def = lpDefaultWorldDef();
	def.stressHopsPerTick = 4;
	Sim s = CreateSimDef( def, -1 );
	int bridge = AddLongWall( &s, 60, 2, true );
	Run( &s, 10 );
	lpBody* body = s.world->bodies.data + bridge;

	// A top joint near the right end, weakened until it reads 1.1 of its limit: it creaks
	int left = -1, creaking = -1;
	for ( int i = 0; i < body->pieces.count; ++i )
	{
		int pi = body->pieces.data[i];
		const lpPiece* p = s.world->pieces.data + pi;
		left = p->shape->centroid.y > 0.5f && p->shape->centroid.x > -14.5f && p->shape->centroid.x < -14.0f ? pi : left;
		for ( int k = 0; k < p->bonds.count && creaking < 0; ++k )
		{
			const lpBond* bond = s.world->bonds.data + p->bonds.data[k];
			const lpVec3 c = s.world->pieces.data[bond->a == pi ? bond->b : bond->a].shape->centroid;
			bool top = p->shape->centroid.y > 0.5f && c.y > 0.5f;
			creaking = top && p->shape->centroid.x > 12.0f && c.x > p->shape->centroid.x && bond->rho > 0.0f ? p->bonds.data[k] : -1;
		}
	}
	ENSURE( left >= 0 && creaking >= 0 );
	lpBond* joint = s.world->bonds.data + creaking;
	joint->health *= joint->rho / 1.1f;
	body->rejudge = true;
	lpMarkDirty( s.world, bridge );
	Run( &s, 1 );
	ENSURE( joint->rho > 1.05f && joint->rho < 1.15f && joint->strain > 0.0f && body->creaking );

	// A joint near the left end breaks: the bridge bends anew, and the region takes many steps to cross it
	const lpPiece* p = s.world->pieces.data + left;
	int broken = -1;
	for ( int k = 0; k < p->bonds.count; ++k )
	{
		const lpBond* bond = s.world->bonds.data + p->bonds.data[k];
		const lpVec3 c = s.world->pieces.data[bond->a == left ? bond->b : bond->a].shape->centroid;
		broken = c.y > 0.5f && c.x > p->shape->centroid.x ? p->bonds.data[k] : broken;
	}
	ENSURE( broken >= 0 );
	lpBreakBond( s.world, broken );
	lpMarkDirty( s.world, bridge );
	int solving = 0;
	for ( int tick = 0; tick < 6; ++tick )
	{
		float before = joint->strain;
		Run( &s, 1 );
		ENSURE( body->solving ); // still crossing the bridge
		ENSURE( joint->alive == false || joint->strain > before );
		solving += 1;
	}
	printf( "  a joint at 1.1 of its limit kept straining through %d steps of a solve far away: strain %.2f\n", solving,
			(double)joint->strain );
	DestroySim( &s );
	return 0;
}

int StressTest( void )
{
	RUN_TEST( TestSolveSystem, MECHANISM );
	RUN_TEST( TestReducedAssembly, MECHANISM );
	RUN_TEST( TestStructuresStand, OUTCOME );
	RUN_TEST( TestScenesSettle, MECHANISM );
	RUN_TEST( TestCantileverRoot, OUTCOME );
	RUN_TEST( TestBeamMidspan, OUTCOME );
	RUN_TEST( TestTowerTopples, OUTCOME );
	RUN_TEST( TestTowerFelled, OUTCOME );
	RUN_TEST( TestStressBudget, MECHANISM );
	RUN_TEST( TestDamagedWallSettles, OUTCOME );
	RUN_TEST( TestMasonryWallHole, OUTCOME );
	RUN_TEST( TestArchKeystone, OUTCOME );
	RUN_TEST( TestColonnade, OUTCOME );
	RUN_TEST( TestKeepBreach, OUTCOME );
	RUN_TEST( TestKeepHole, OUTCOME );
	RUN_TEST( TestKeepDeterminism, DETERMINISM );
	RUN_TEST( TestKeepLocalHit, MECHANISM );
	RUN_TEST( TestDriftSmallStructures, MECHANISM );
	RUN_TEST( TestKeepUnderFire, OUTCOME );
	RUN_TEST( TestKeepAudit, MECHANISM );
	RUN_TEST( TestStressCone, MECHANISM );
	RUN_TEST( TestStressDriftGuard, MECHANISM );
	RUN_TEST( TestCreakDuringFarSolve, MECHANISM );
	RUN_TEST( TestReliefBalances, MECHANISM );
	RUN_TEST( TestReliefFreeFall, OUTCOME );
	RUN_TEST( TestReliefMatchesSupported, MECHANISM );
	RUN_TEST( TestReliefLandingSnaps, OUTCOME );
	RUN_TEST( TestCrashTearsEngine, OUTCOME );
	return 0;
}
