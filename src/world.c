// SPDX-License-Identifier: MIT
// The destruction world: pieces, bonds and bodies on top of a Box3D world.
//
// Invariants (checked by lpWorld_Validate in tests):
// - every live piece belongs to exactly one live body and, unless that body is a ghost or scrap (no Box3D body),
//   owns one Box3D hull shape on it
// - bonds only join pieces of the same body; a piece's bond list holds exactly its live bonds
// - body frames never change when pieces move between bodies: a split-off body is created at the parent's
//   transform, so piece geometry stays in the original object frame for its whole life (no drift, and solid
//   interior colors line up across cuts)

#include "facet.h"
#include "fracture.h"
#include "tasks.h"
#include "world.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
// Strengths are damage densities: an impact of energy E and radius R delivers E (1 - d/R)^2 / (pi R^2) at
// distance d. Calibration: a rifle round (E 4 kJ, R 0.35 m, center 10.4 kJ/m^2) chips ~15 cm out of brick in one
// shot; a grenade (E 80 kJ, R 1.4 m, center 13 kJ/m^2) opens ~0.7 m in brick and ~0.3 m in stone.
// Tier volumes (particle / ghost / light) as cube edges: stone and brick about 4 / 9 / 22 cm, plaster 6 / 12 / 25 cm,
// wood 5 / 18 / 31 cm (splinters are long and thin, so most are ghosts), glass shards nearly all ghosts, foliage mostly
// leaves. plateSize keeps the far side of a broken piece in a few big plates (a log keeps two whole ends).
static lpMaterialDef lp_materials[lp_materialCount] = {
	[lp_wood] = { "wood", 600.0f, 2000.0f, 1600.0f, 0.14f, 0.6f, 0.05f, lp_breakGrain, 3.5f, 0xE0B070u, true,
				  3.4e-6f, 6.0e-3f, 3.0e-2f, 1.6f, 20, lp_particleSplinter, 0.3f, 1, 30e6f, 40e6f, 5e6f, 0.0f, 0.0f },
	[lp_stone] = { "stone", 2400.0f, 8000.0f, 6400.0f, 0.18f, 0.7f, 0.02f, lp_breakImpact, 1.0f, 0x9A968Cu, true,
				   3.4e-6f, 7.3e-4f, 1.06e-2f, 1.2f, 28, lp_particleChip, 0.08f, 2, 5e6f, 60e6f, 8e6f, 0.0f, 0.0f },
	[lp_brick] = { "brick", 1900.0f, 3000.0f, 2400.0f, 0.16f, 0.7f, 0.02f, lp_breakMasonry, 1.0f, 0xC8704Au, true,
				   3.4e-6f, 7.3e-4f, 2.0e-2f, 1.2f, 28, lp_particleChip, 0.08f, 2, 2e6f, 20e6f, 3e6f, 0.15f, 0.3f },
	[lp_plaster] = { "plaster", 1200.0f, 1200.0f, 1000.0f, 0.16f, 0.6f, 0.02f, lp_breakImpact, 1.0f, 0xEEE6D2u, true,
					 3.4e-6f, 1.7e-3f, 1.56e-2f, 1.2f, 24, lp_particleDust, 0.1f, 3, 1e6f, 5e6f, 1e6f, 0.0f, 0.0f },
	[lp_concrete] = { "concrete", 2400.0f, 10000.0f, 8000.0f, 0.2f, 0.7f, 0.02f, lp_breakImpact, 1.0f, 0xA5A39Cu, true,
					  3.4e-6f, 7.3e-4f, 1.06e-2f, 1.4f, 28, lp_particleChip, 0.08f, 2, 3e6f, 30e6f, 5e6f, 0.0f, 0.0f },
	[lp_glass] = { "glass", 2500.0f, 300.0f, 240.0f, 0.1f, 0.4f, 0.05f, lp_breakRadial, 1.0f, 0xC6EEF2u, true,
				   3.4e-6f, 3.4e-3f, 1.0e-2f, 0.8f, 32, lp_particleGlint, 0.0f, 3, 30e6f, 500e6f, 20e6f, 0.0f, 0.0f },
	[lp_metal] = { "metal", 7800.0f, 1e9f, 1e9f, 0.3f, 0.5f, 0.1f, lp_breakImpact, 1.0f, 0x70757Bu, false,
				   6.4e-5f, 7.3e-4f, 1.06e-2f, 1.0f, 16, lp_particleChip, 0.0f, 0, 1e12f, 1e12f, 1e12f, 0.0f, 0.0f },
	[lp_ground] = { "ground", 2000.0f, 1e9f, 1e9f, 1.0f, 0.8f, 0.0f, lp_breakImpact, 1.0f, 0x6E5B45u, false,
					6.4e-5f, 7.3e-4f, 1.06e-2f, 1.0f, 16, lp_particleDust, 0.0f, 0, 1e12f, 1e12f, 1e12f, 0.0f, 0.0f },
	[lp_foliage] = { "foliage", 150.0f, 300.0f, 240.0f, 0.4f, 0.8f, 0.0f, lp_breakImpact, 1.0f, 0x4E8C3Au, true,
					 8.0e-3f, 9.0e-2f, 0.5f, 2.5f, 12, lp_particleLeaf, 0.3f, 0, 10e6f, 10e6f, 10e6f, 0.0f, 0.0f },
	// Panels are thick stand-ins for thin sheet, so the density is a car's, not steel's. They tear into a few big
	// plates, and their crumpling soaks up most of a crash before anything breaks.
	[lp_sheetMetal] = { "sheet metal", 1200.0f, 2500.0f, 6000.0f, 0.3f, 0.5f, 0.1f, lp_breakImpact, 1.0f, 0x8A9098u, true,
						3.4e-6f, 2.0e-3f, 2.0e-2f, 1.2f, 10, lp_particleChip, 0.3f, 0, 200e6f, 200e6f, 120e6f, 0.0f, 0.0f,
						0.7f },
	[lp_rubber] = { "rubber", 900.0f, 20000.0f, 30000.0f, 0.25f, 1.0f, 0.1f, lp_breakImpact, 1.0f, 0x2A2A2Au, true,
					3.4e-6f, 2.0e-3f, 2.0e-2f, 1.2f, 8, lp_particleChip, 0.3f, 0, 20e6f, 50e6f, 20e6f, 0.0f, 0.0f, 0.6f },
	// Welded plate. Like sheet metal the parts are stand-ins for a shell, so the density is a hull's, but it takes ten
	// times the damage: a grenade (13 kJ/m^2 at its centre) or a cannon blast (20) chips at most its welds' edges
	[lp_armor] = { "armor", 1200.0f, 25000.0f, 60000.0f, 0.3f, 0.5f, 0.1f, lp_breakImpact, 1.0f, 0x6A6F75u, true,
				   3.4e-6f, 2.0e-3f, 2.0e-2f, 1.2f, 10, lp_particleChip, 0.3f, 0, 400e6f, 400e6f, 250e6f, 0.0f, 0.0f, 0.5f },
};

// Joints (Pa). Mortar is weak in tension, so masonry hinges and cracks at its joints; dry stacking holds only by
// friction; nails hold timber well in shear but pull out in tension.
static const lpJointDef lp_joints[lp_jointCount] = {
	[lp_jointAuto] = { "auto", 0.0f, 0.0f, 0.0f, 0.0f },
	[lp_jointSolid] = { "solid", 1e12f, 1e12f, 1e12f, 0.6f },
	[lp_jointMortar] = { "mortar", 0.3e6f, 15e6f, 0.3e6f, 0.6f },
	[lp_jointDry] = { "dry", 0.0f, 40e6f, 0.0f, 0.7f },
	[lp_jointNails] = { "nails", 0.5e6f, 20e6f, 1e6f, 0.5f },
	[lp_jointBolts] = { "bolts", 3e6f, 50e6f, 2.5e6f, 0.6f }, // a few bolts, not the whole face: panels come off
	[lp_jointMounts] = { "mounts", 1.5e6f, 30e6f, 1.5e6f, 0.6f },
	// A seam around a shell: the parts are solid stand-ins, so the seam holds what the shell's wall would (a fifth)
	[lp_jointWeld] = { "weld", 40e6f, 200e6f, 25e6f, 0.6f },
};

const lpJointDef* lpGetJoint( int jointId )
{
	LP_ASSERT( 0 <= jointId && jointId < lp_jointCount );
	return lp_joints + jointId;
}

// The joint a part gets when it asks for lp_jointAuto
static uint8_t lpResolveJoint( uint8_t joint, uint8_t material )
{
	if ( joint != lp_jointAuto )
	{
		return joint;
	}
	switch ( material )
	{
		case lp_stone:
		case lp_brick:
		case lp_concrete:
		case lp_plaster:
			return lp_jointMortar;
		case lp_wood:
			return lp_jointNails;
		case lp_sheetMetal:
			return lp_jointBolts;
		default:
			return lp_jointSolid;
	}
}

// The weaker of two joints: lower tensile strength, then the higher id
static uint8_t lpWeakerJoint( uint8_t a, uint8_t b )
{
	float ta = lp_joints[a].tensileStrength;
	float tb = lp_joints[b].tensileStrength;
	if ( ta != tb )
	{
		return ta < tb ? a : b;
	}
	return a > b ? a : b;
}

const lpMaterialDef* lpGetMaterial( int materialId )
{
	LP_ASSERT( 0 <= materialId && materialId < lp_materialCount );
	return lp_materials + materialId;
}

lpWorldDef lpDefaultWorldDef( void )
{
	lpWorldDef def = { 0 };
	def.gravity = (lpVec3){ 0.0f, -10.0f, 0.0f };
	def.seed = 1;
	def.maxFullDebris = 400;
	def.maxLightDebris = 1200;
	def.maxGhosts = 4000;
	def.maxRubblePieces = 20000;
	def.maxScrapPieces = 8000;
	def.fragmentScale = 1.0f;
	def.debrisScale = 1.0f;
	def.freezeRubble = true;
	def.maxDepth = 3;
	def.maxHitImpacts = 16;
	def.hitSpeed = 4.0f;
	def.wakeSpeed = 1.5f;
	def.killDepth = -50.0f;
	def.workerCount = 1;
	def.stressScale = 1.0f;
	def.maxStressWork = 60000;
	def.maxStressStructureWork = 10000;
	def.maxStressIterations = 256;
	def.maxSettleIterations = 4000;
	def.stressLargeNodes = 512;
	def.stressGlue = 0.3f;
	def.maxStressBreaks = 4;
	def.stressPatience = 30;
	def.strainRate = 1.0f;
	def.maxLinks = 4096;
	def.maxFractureJobsPerStep = 48;
	def.maxFreezesPerStep = 64;
	def.maxGhostCastsPerStep = 2048;
	def.maxWheelCastsPerStep = 1024;
	def.maxFootCastsPerStep = 256;
	return def;
}

float lpParticleVolume( const lpWorld* w, int material )
{
	return lpGetMaterial( material )->particleVolume * w->def.debrisScale;
}

float lpGhostVolume( const lpWorld* w, int material )
{
	return lpGetMaterial( material )->ghostVolume * w->def.debrisScale;
}

float lpLightVolume( const lpWorld* w, int material )
{
	return lpGetMaterial( material )->lightVolume * w->def.debrisScale;
}

lpObjectDef lpDefaultObjectDef( void )
{
	lpObjectDef def = { 0 };
	def.transform = lpTransform_identity;
	def.isStatic = true;
	def.gravityScale = 1.0f;
	return def;
}

lpPartDef lpDefaultPartDef( void )
{
	lpPartDef def = { 0 };
	def.halfExtents = (lpVec3){ 0.5f, 0.5f, 0.5f };
	def.transform = lpTransform_identity;
	def.material = lp_stone;
	def.color = 0x9A968Cu;
	return def;
}

// ---- slots ----

int lpAllocPiece( lpWorld* w )
{
	int index;
	if ( w->freePiece != -1 )
	{
		index = w->freePiece;
		w->freePiece = w->pieces.data[index].nextFree;
		uint32_t generation = w->pieces.data[index].generation;
		memset( w->pieces.data + index, 0, sizeof( lpPiece ) );
		w->pieces.data[index].generation = generation + 1;
	}
	else
	{
		index = w->pieces.count;
		lpPiece zero = { 0 };
		lpArray_Push( w->pieces, zero );
	}
	lpPiece* p = w->pieces.data + index;
	p->body = -1;
	p->nextFree = -1;
	p->shapeId = lp_nullPhysShape;
	memset( &p->stressX, 0, 5 * sizeof( lpVec6 ) );
	p->strain = 0.0f;
	p->accepted = 0;
	lpTouchPiece( w, index ); // a new piece is a seed of its structure's next solve
	return index;
}

void lpFreePieceSlot( lpWorld* w, int index )
{
	lpBreakPieceLinks( w, index );
	lpPiece* p = w->pieces.data + index;
	lpCarriersChanged( w, p->carries );
	lpShape_Destroy( p->shape );
	if ( p->hull != NULL )
	{
		lpPhys_DestroyHull( p->hull );
	}
	lpArray_Free( p->bonds );
	lpArray_Free( p->links );
	p->shape = NULL;
	p->hull = NULL;
	p->body = -1;
	p->nextFree = w->freePiece;
	w->freePiece = index;
}

static int lpAllocBond( lpWorld* w )
{
	int index;
	if ( w->freeBond != -1 )
	{
		index = w->freeBond;
		w->freeBond = w->bonds.data[index].nextFree;
	}
	else
	{
		index = w->bonds.count;
		lpBond zero = { 0 };
		lpArray_Push( w->bonds, zero );
	}
	w->bonds.data[index].nextFree = -1;
	return index;
}

int lpAllocBody( lpWorld* w )
{
	int index;
	if ( w->freeBody != -1 )
	{
		index = w->freeBody;
		w->freeBody = w->bodies.data[index].nextFree;
		lpBody* b = w->bodies.data + index;
		int* data = b->pieces.data;
		int capacity = b->pieces.capacity;
		uint32_t generation = b->generation;
		memset( b, 0, sizeof( lpBody ) );
		b->pieces.data = data;
		b->pieces.capacity = capacity;
		b->generation = generation + 1;
	}
	else
	{
		index = w->bodies.count;
		lpBody zero = { 0 };
		lpArray_Push( w->bodies, zero );
	}
	lpBody* b = w->bodies.data + index;
	b->alive = true;
	b->gravityScale = 1.0f;
	b->inertiaRadius = 0.0f;
	b->nextFree = -1;
	b->createdTick = w->tick;
	b->gridSlot = -1;
	b->gridPrev = -1;
	b->gridNext = -1;
	b->landIn = -1;
	b->stressPin = -1;
	return index;
}

// ---- world ----

lpWorld* lpCreateWorld( const lpWorldDef* def )
{
	lpWorld* w = lpAlloc( sizeof( lpWorld ) );
	memset( w, 0, sizeof( lpWorld ) );
	w->def = *def;
	w->freePiece = -1;
	w->freeBond = -1;
	w->freeBody = -1;
	w->freeLink = -1;
	w->freeWheel = -1;
	w->audit.body = -1;
	lpGuardFp( w );
	int failures = 0;
	lpDeterminismSelfTest( &failures );
	if ( failures != 0 )
	{
		fprintf( stderr, "lpCreateWorld: %d determinism self-test answers are wrong on this machine\n", failures );
	}
	lpPhysDef pd = { 0 };
	pd.gravity = def->gravity;
	pd.workerCount = def->workerCount;
	pd.hitSpeed = lpMinFloat( def->hitSpeed, def->wakeSpeed ); // hits start at the wake speed (waking fragile rubble)
	pd.pairFilter = lpPairFilter;
	pd.context = w;
	w->phys = lpPhys_Create( &pd );
	w->tasks = lpTaskPool_Create( def->workerCount );
	lpGridInit( w );
	return w;
}

void lpDestroyWorld( lpWorld* w )
{
	lpPhys_Destroy( w->phys ); // every body, shape and joint with it
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		lpBody* b = w->bodies.data + i;
		lpArray_Free( b->pieces );
		lpFreeStressSystem( b );
	}
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		lpPiece* p = w->pieces.data + i;
		if ( p->body >= 0 )
		{
			lpShape_Destroy( p->shape );
			if ( p->hull != NULL )
			{
				lpPhys_DestroyHull( p->hull );
			}
		}
		lpArray_Free( p->bonds );
		lpArray_Free( p->links );
	}
	lpArray_Free( w->links );
	lpFreeVehicles( w );
	lpFreeRigs( w );
	lpArray_Free( w->scratchLinkMoves );
	lpArray_Free( w->pieces );
	lpArray_Free( w->bonds );
	lpArray_Free( w->bodies );
	lpArray_Free( w->impacts );
	lpArray_Free( w->nextImpacts );
	lpArray_Free( w->particles );
	lpArray_Free( w->scratchPieces );
	lpArray_Free( w->scratchBodies );
	lpArray_Free( w->scratchQueue );
	lpArray_Free( w->dirtyBodies );
	lpArray_Free( w->forces );
	lpArray_Free( w->pendingWakes );
	lpArray_Free( w->freezeCandidates );
	lpArray_Free( w->pendingDestroy );
	lpArray_Free( w->detonators );
	lpArray_Free( w->pools );
	lpArray_Free( w->scratchCarriers );
	for ( int i = 0; i < w->stressJobCapacity; ++i )
	{
		lpArray_Free( w->stressJobs[i].slender );
		lpArray_Free( w->stressJobs[i].clusterGroup );
		lpArray_Free( w->stressJobs[i].groupMass );
	}
	lpFree( w->stressJobs );
	lpArray_Free( w->stressQueue );
	lpArray_Free( w->scratchOverloads );
	lpArray_Free( w->scratchLoads );
	lpArray_Free( w->scratchClusters );
	lpArray_Free( w->scratchSets );
	lpArray_Free( w->audits );
	lpArray_Free( w->stressAgain );
	for ( int i = 0; i < w->jobCapacity; ++i )
	{
		lpFree( w->jobs[i].bonds );
	}
	lpFree( w->jobs );
	lpTaskPool_Destroy( w->tasks );
	lpArray_Free( w->pulls );
	lpArray_Free( w->blows );
	lpArray_Free( w->deferred );
	lpArray_Free( w->scratchLoose );
	lpArray_Free( w->scratchHits );
	lpArray_Free( w->scratchComponents );
	lpGridFree( w );
	lpFree( w );
}

// ---- pieces and bodies ----

// Filters follow the body's tier (see world.h). Set at shape creation only; changing a filter later is as costly as
// recreating the shape.
static lpPhysShapeDef lpMakeShapeDef( int pieceIndex, uint8_t material, const lpBody* body )
{
	const lpMaterialDef* m = lpGetMaterial( material );
	lpPhysShapeDef def = { 0 };
	def.density = m->density;
	def.friction = m->friction;
	def.restitution = m->restitution;
	def.material = material;
	def.userData = pieceIndex;
	if ( body->kind == lp_kindStructure )
	{
		def.filter = ( lpPhysFilter ){ LP_CAT_STATIC, LP_CAT_ALL };
		def.hitEvents = m->breakable;
	}
	else if ( body->tier == lp_tierLight )
	{
		def.filter = ( lpPhysFilter ){ LP_CAT_LIGHT, LP_CAT_STATIC | LP_CAT_FULL };
		def.customFilter = true;
		def.hitEvents = false;
	}
	else
	{
		def.filter = ( lpPhysFilter ){ LP_CAT_FULL,
									   LP_CAT_STATIC | LP_CAT_FULL | LP_CAT_LIGHT | LP_CAT_VEHICLE | LP_CAT_CHARACTER | LP_CAT_PROJECTILE };
		def.hitEvents = true; // also wakes fragile rubble it bumps into
	}
	return def;
}

// The physics shape for a piece on a physics body, without touching the body's piece list
bool lpCreatePieceShape( lpWorld* w, int pieceIndex, int bodyIndex )
{
	lpPiece* p = w->pieces.data + pieceIndex;
	lpBody* b = w->bodies.data + bodyIndex;
	uint64_t t0 = lpGetTicks();
	if ( p->hull == NULL )
	{
		p->hull = lpShape_CreateHull( p->shape );
		if ( p->hull == NULL )
		{
			return false;
		}
	}
	w->stats.hullMs += lpGetMilliseconds( t0 );
	uint64_t t1 = lpGetTicks();
	lpPhysShapeDef def = lpMakeShapeDef( pieceIndex, p->material, b );
	p->shapeId = lpPhys_CreateHullShape( w->phys, b->id, &def, p->hull );
	w->stats.shapeMs += lpGetMilliseconds( t1 );
	return true;
}

bool lpAttachPiece( lpWorld* w, int pieceIndex, int bodyIndex )
{
	if ( lpCreatePieceShape( w, pieceIndex, bodyIndex ) == false )
	{
		return false;
	}
	lpPiece* p = w->pieces.data + pieceIndex;
	lpBody* b = w->bodies.data + bodyIndex;
	p->body = bodyIndex;
	p->cluster = 0; // clusters are its old structure's
	lpArray_Push( b->pieces, pieceIndex );
	b->volume += p->shape->volume;
	b->topology += 1;
	return true;
}

void lpDetachPieceShape( lpWorld* w, int pieceIndex )
{
	lpPiece* p = w->pieces.data + pieceIndex;
	if ( LP_PHYS_NULL( p->shapeId ) == false )
	{
		lpPhys_DestroyShape( w->phys, p->shapeId );
		p->shapeId = lp_nullPhysShape;
	}
}

static void lpRemoveBondFromPiece( lpPiece* p, int bondIndex )
{
	for ( int i = 0; i < p->bonds.count; ++i )
	{
		if ( p->bonds.data[i] == bondIndex )
		{
			// keep order for determinism of later traversals
			memmove( p->bonds.data + i, p->bonds.data + i + 1, sizeof( int ) * (size_t)( p->bonds.count - i - 1 ) );
			p->bonds.count -= 1;
			return;
		}
	}
	LP_ASSERT( false );
}

void lpBreakBond( lpWorld* w, int bondIndex )
{
	lpBond* bond = w->bonds.data + bondIndex;
	LP_ASSERT( bond->alive );
	int body = w->pieces.data[bond->a].body;
	if ( body >= 0 )
	{
		w->bodies.data[body].topology += 1;
	}
	lpRemoveBondFromPiece( w->pieces.data + bond->a, bondIndex );
	lpRemoveBondFromPiece( w->pieces.data + bond->b, bondIndex );
	lpCarriersChanged( w, w->pieces.data[bond->a].carries & w->pieces.data[bond->b].carries );
	lpTouchPiece( w, bond->a );
	lpTouchPiece( w, bond->b );
	bond->alive = false;
	bond->nextFree = w->freeBond;
	w->freeBond = bondIndex;
	w->bondCount -= 1;
}

int lpAddBond( lpWorld* w, int a, int b, const lpContact* contact, uint8_t joint )
{
	lpPiece* pa = w->pieces.data + a;
	lpPiece* pb = w->pieces.data + b;
	float strengthA = lpGetMaterial( pa->material )->bondStrength;
	float strengthB = lpGetMaterial( pb->material )->bondStrength;

	if ( pa->body >= 0 )
	{
		w->bodies.data[pa->body].topology += 1;
	}
	int index = lpAllocBond( w );
	lpBond* bond = w->bonds.data + index;
	bond->a = a < b ? a : b;
	bond->b = a < b ? b : a;
	bond->area = contact->area;
	bond->centroid = contact->centroid;
	bond->normal = a < b ? contact->normal : lpNeg( contact->normal ); // always from bond->a toward bond->b
	bond->h1 = contact->h1;
	bond->h2 = contact->h2;
	bond->joint = joint;
	bond->strain = 0.0f;
	bond->rho = 0.0f;
	bond->force = lpVec3_zero;
	bond->moment = lpVec3_zero;
	bond->health = strengthA < strengthB ? strengthA : strengthB;
	bond->strength = bond->health;
	bond->alive = true;
	lpArray_Push( pa->bonds, index );
	lpArray_Push( pb->bonds, index );
	lpCarriersChanged( w, pa->carries & pb->carries );
	lpTouchPiece( w, a );
	lpTouchPiece( w, b );
	w->bondCount += 1;
	return index;
}

static const float lp_weldMargin = 0.03f; // parts this close are welded when they share no face

// Bond two pieces of one body if they share a face. Parts that meet at an angle (a roof plank on a gable) share no
// coplanar face; if they touch or interpenetrate they are welded with a nominal area instead.
void lpTryBond( lpWorld* w, int a, int b )
{
	lpPiece* pa = w->pieces.data + a;
	lpPiece* pb = w->pieces.data + b;
	const float tolerance = 2e-3f;
	const float weldMargin = lp_weldMargin;
	if ( lpBoxesTouch( pa->shape->bounds, pb->shape->bounds, weldMargin ) == false )
	{
		return;
	}

	uint8_t joint = lpWeakerJoint( pa->joint, pb->joint );
	lpContact contact;
	if ( lpShape_Contact( pa->shape, pb->shape, tolerance, &contact ) && contact.area >= 1e-4f )
	{
		lpAddBond( w, a, b, &contact, joint );
		return;
	}

	if ( lpShape_NearlyOverlap( pa->shape, pb->shape, weldMargin ) )
	{
		// A weld: a nominal square patch facing from one centroid to the other
		float v = pa->shape->volume < pb->shape->volume ? pa->shape->volume : pb->shape->volume;
		contact.area = 0.5f * lpCbrt( v * v );
		contact.centroid = lpMulSV( 0.5f, lpAdd( pa->shape->centroid, pb->shape->centroid ) );
		lpVec3 d = lpSub( pb->shape->centroid, pa->shape->centroid );
		contact.normal = lpLengthSquared( d ) > 1e-12f ? lpNormalize( d ) : (lpVec3){ 0.0f, 1.0f, 0.0f };
		contact.h1 = 0.5f * sqrtf( contact.area );
		contact.h2 = contact.h1;
		lpAddBond( w, a, b, &contact, joint );
	}
}

lpWorldTransform lpGetTransform( const lpWorld* w, const lpBody* b )
{
	if ( b->kind == lp_kindGhost || b->kind == lp_kindScrap )
	{
		lpVec3 offset = lpRotateVector( b->q, b->localCenter );
		lpWorldTransform xf = { lpOffsetPos( b->com, lpNeg( offset ) ), b->q };
		return xf;
	}
	return lpPhys_GetTransform( w->phys, b->id );
}

// Emit a cosmetic particle at a body-frame point, coloured and shaped by the material
void lpEmitParticle( lpWorld* w, lpWorldTransform xf, lpVec3 localPoint, lpVec3 velocity, float size, uint8_t material )
{
	const lpMaterialDef* m = lpGetMaterial( material );
	uint32_t rgb = m->interiorColor;
	lpPos p = lpTransformWorldPoint( xf, localPoint );
	lpParticle particle;
	particle.position[0] = (float)p.x;
	particle.position[1] = (float)p.y;
	particle.position[2] = (float)p.z;
	particle.velocity[0] = velocity.x;
	particle.velocity[1] = velocity.y;
	particle.velocity[2] = velocity.z;
	particle.size = size;
	particle.color = 0xFF000000u | ( ( rgb & 0xFF ) << 16 ) | ( rgb & 0xFF00 ) | ( ( rgb >> 16 ) & 0xFF );
	particle.kind = m->particleKind;
	lpArray_Push( w->particles, particle );
}

void lpDestroyBody( lpWorld* w, int bodyIndex, bool emitDust )
{
	lpBody* b = w->bodies.data + bodyIndex;
	LP_ASSERT( b->alive );
	bool loose = b->kind == lp_kindGhost || b->kind == lp_kindScrap;
	lpWorldTransform xf = lpGetTransform( w, b );

	for ( int i = 0; i < b->pieces.count; ++i )
	{
		int pieceIndex = b->pieces.data[i];
		lpPiece* p = w->pieces.data + pieceIndex;
		while ( p->bonds.count > 0 )
		{
			lpBreakBond( w, p->bonds.data[p->bonds.count - 1] );
		}
		if ( emitDust )
		{
			float size = lpMinFloat( lpCbrt( p->shape->volume ), 0.3f );
			lpEmitParticle( w, xf, p->shape->centroid, loose ? b->v : lpVec3_zero, size, p->material );
		}
		p->shapeId = lp_nullPhysShape; // destroyed with the body
		lpFreePieceSlot( w, pieceIndex );
	}
	b->pieces.count = 0;

	if ( loose )
	{
		lpGridRemove( w, bodyIndex );
	}
	else
	{
		lpPhys_DestroyBody( w->phys, b->id );
	}
	lpFreeStressSystem( b );
	b->alive = false;
	b->id = lp_nullPhysBody;
	b->nextFree = w->freeBody;
	w->freeBody = bodyIndex;
}

int lpCreateBodyInternal( lpWorld* w, lpWorldTransform xf, bool dynamic, uint8_t kind, uint8_t tier, lpVec3 v, lpVec3 omega,
						  float gravityScale )
{
	int index = lpAllocBody( w );
	lpBody* b = w->bodies.data + index;
	lpPhysBodyDef def = lpPhys_DefaultBodyDef();
	def.dynamic = dynamic;
	def.transform = xf;
	def.linearVelocity = v;
	def.angularVelocity = omega;
	def.userData = index;
	if ( tier == lp_tierLight )
	{
		def.sleepThreshold = 0.3f; // light debris settles fast and freezes early
	}
	def.gravityScale = gravityScale;
	b->id = lpPhys_CreateBody( w->phys, &def );
	b->kind = kind;
	b->tier = tier;
	b->gravityScale = gravityScale;
	return index;
}

void lpMarkDirty( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->dirty == false )
	{
		b->dirty = true;
		lpArray_Push( w->dirtyBodies, bodyIndex );
	}
}

static lpVec3 lpBoxAxis( lpVec3 h, lpQuat q, bool longest )
{
	lpVec3 axis = { 1.0f, 0.0f, 0.0f };
	float best = h.x;
	if ( longest ? h.y > best : h.y < best )
	{
		axis = (lpVec3){ 0.0f, 1.0f, 0.0f };
		best = h.y;
	}
	if ( longest ? h.z > best : h.z < best )
	{
		axis = (lpVec3){ 0.0f, 0.0f, 1.0f };
	}
	return lpRotateVector( q, axis );
}

typedef struct lpSweepItem
{
	float lo, hi; // bounds along x
	int slot;	  // in the body's piece list
} lpSweepItem;

typedef struct lpPiecePair
{
	int i, j;
} lpPiecePair;

static int lpCompareSweep( const void* a, const void* b )
{
	const lpSweepItem* x = a;
	const lpSweepItem* y = b;
	if ( x->lo != y->lo )
	{
		return x->lo < y->lo ? -1 : 1;
	}
	return ( x->slot > y->slot ) - ( x->slot < y->slot );
}

static int lpComparePair( const void* a, const void* b )
{
	const lpPiecePair* x = a;
	const lpPiecePair* y = b;
	if ( x->i != y->i )
	{
		return ( x->i > y->i ) - ( x->i < y->i );
	}
	return ( x->j > y->j ) - ( x->j < y->j );
}

// Bond the touching parts of a new object (the body's pieces from `first` on). A sweep along x finds the pairs whose
// bounds touch, which are then tried in the order of a double loop over the piece list, so the bonds come out exactly
// as if every pair had been tried.
static void lpBondParts( lpWorld* w, int bodyIndex, int first )
{
	const lpBody* b = w->bodies.data + bodyIndex;
	int n = b->pieces.count - first;
	if ( n < 2 )
	{
		return;
	}
	lpSweepItem* items = lpAlloc( sizeof( lpSweepItem ) * (size_t)n );
	for ( int k = 0; k < n; ++k )
	{
		lpAABB box = w->pieces.data[b->pieces.data[first + k]].shape->bounds;
		items[k] = (lpSweepItem){ box.lowerBound.x, box.upperBound.x, first + k };
	}
	qsort( items, (size_t)n, sizeof( lpSweepItem ), lpCompareSweep );

	LP_ARRAY( lpPiecePair ) pairs = { 0 };
	for ( int k = 0; k < n; ++k )
	{
		const lpShape* a = w->pieces.data[b->pieces.data[items[k].slot]].shape;
		for ( int m = k + 1; m < n && items[m].lo <= items[k].hi + lp_weldMargin; ++m )
		{
			const lpShape* c = w->pieces.data[b->pieces.data[items[m].slot]].shape;
			if ( lpBoxesTouch( a->bounds, c->bounds, lp_weldMargin ) )
			{
				int i = items[k].slot, j = items[m].slot;
				lpPiecePair pair = { i < j ? i : j, i < j ? j : i };
				lpArray_Push( pairs, pair );
			}
		}
	}
	lpFree( items );
	if ( pairs.count > 1 )
	{
		qsort( pairs.data, (size_t)pairs.count, sizeof( lpPiecePair ), lpComparePair );
	}
	for ( int k = 0; k < pairs.count; ++k )
	{
		lpTryBond( w, b->pieces.data[pairs.data[k].i], b->pieces.data[pairs.data[k].j] );
		b = w->bodies.data + bodyIndex;
	}
	lpArray_Free( pairs );
}

// Returns the detonator + 1, or 0 for an inert one
static int lpAddDetonator( lpWorld* w, const lpDetonatorDef* def )
{
	if ( def->radius <= 0.0f )
	{
		return 0;
	}
	lpDetonator d = { *def, true };
	lpArray_Push( w->detonators, d );
	return w->detonators.count;
}

int lpCreateObject( lpWorld* w, const lpObjectDef* def )
{
	lpGuardFp( w ); // computes in float between steps, on the caller's thread
	uint8_t kind = def->isStatic ? lp_kindStructure : lp_kindDebris;
	int bodyIndex = lpCreateBodyInternal( w, def->transform, def->isStatic == false, kind, lp_tierFull, def->linearVelocity, def->angularVelocity,
										  def->gravityScale );
	int objectDetonator = lpAddDetonator( w, &def->detonator ); // shared by the parts without one of their own
	w->bodies.data[bodyIndex].solveStress = def->solveStress && def->isStatic == false;

	lpPoly* poly = lpAlloc( sizeof( lpPoly ) );
	int first = w->bodies.data[bodyIndex].pieces.count;

	for ( int i = 0; i < def->partCount; ++i )
	{
		const lpPartDef* part = def->parts + i;
		bool ok;
		lpVec3 extents = { 0.5f, 0.5f, 0.5f };
		if ( part->pointCount == 0 )
		{
			lpPoly_MakeBox( poly, part->halfExtents, part->transform, part->material );
			ok = true;
			extents = part->halfExtents;
		}
		else
		{
			ok = lpPoly_MakeFromPoints( poly, part->points, part->pointCount, part->material );
			if ( ok )
			{
				lpAABB box = lpPoly_ComputeBounds( poly );
				extents = lpMulSV( 0.5f, lpSub( box.upperBound, box.lowerBound ) );
			}
		}
		if ( ok == false )
		{
			continue;
		}

		lpShape* shape = lpShape_Create( poly );
		if ( shape == NULL )
		{
			continue;
		}

		int pieceIndex = lpAllocPiece( w );
		lpPiece* p = w->pieces.data + pieceIndex;
		p->shape = shape;
		p->material = part->material;
		p->joint = lpResolveJoint( part->joint, part->material );
		p->color = part->color;
		p->seed = (uint32_t)lpMix64( w->def.seed ^ ( (uint64_t)pieceIndex << 20 ) ^ w->pieceSerial++ );
		p->userId = def->userId;
		p->part = (uint16_t)i;
		p->tag = part->system.tag;
		p->sources = part->system.sources;
		p->carries = (uint8_t)( part->system.carries | part->system.sources ); // a source carries what it feeds
		// What it needs comes from lower channels only, so one pass over the channels in order settles every supply
		uint8_t lowest = (uint8_t)( p->sources & ( ~p->sources + 1u ) );
		p->needs = p->sources != 0 ? (uint8_t)( part->system.needs & ( lowest - 1u ) ) : part->system.needs;
		p->detonator = part->detonator.radius > 0.0f ? lpAddDetonator( w, &part->detonator ) : objectDetonator;
		p->pool = 0;
		if ( part->system.pool > 0.0f && p->sources != 0 )
		{
			lpPool pool = { part->system.pool, part->system.pool, 0.0f, lpMaxFloat( part->system.seal, 0.0f ), -1.0f, 0.0f, 16 };
			lpArray_Push( w->pools, pool );
			p->pool = w->pools.count;
		}

		lpQuat q = part->pointCount == 0 ? part->transform.q : lpQuat_identity;
		int pattern = lpGetMaterial( part->material )->pattern;
		bool glass = pattern == lp_breakRadial;
		if ( lpLengthSquared( part->grainAxis ) > 0.0f )
		{
			p->axis = lpNormalize( part->grainAxis );
		}
		else if ( pattern == lp_breakMasonry )
		{
			// Bricks run horizontally along the wall: across the wall's thinnest axis and the vertical
			lpVec3 thin = lpBoxAxis( extents, q, false );
			lpVec3 run = lpCross( (lpVec3){ 0.0f, 1.0f, 0.0f }, thin );
			p->axis = lpLengthSquared( run ) > 1e-6f ? lpNormalize( run ) : (lpVec3){ 1.0f, 0.0f, 0.0f };
		}
		else
		{
			p->axis = lpBoxAxis( extents, q, glass == false );
		}

		if ( part->anchored )
		{
			// Anchor through the face that points most downward (object space -y)
			int best = 0;
			float bestDot = FLT_MAX;
			for ( int f = 0; f < shape->faceCount; ++f )
			{
				float d = shape->faces[f].plane.normal.y;
				if ( d < bestDot )
				{
					bestDot = d;
					best = f;
				}
			}
			p->anchored = true;
			p->anchorPlane = shape->faces[best].plane;
		}

		if ( lpAttachPiece( w, pieceIndex, bodyIndex ) == false )
		{
			lpFreePieceSlot( w, pieceIndex );
		}
	}
	lpFree( poly );

	// Each source's share of the object's sources of the same channels, by volume
	lpBody* body = w->bodies.data + bodyIndex;
	for ( int k = first; k < body->pieces.count; ++k )
	{
		lpPiece* p = w->pieces.data + body->pieces.data[k];
		float total = 0.0f;
		for ( int j = first; j < body->pieces.count && p->sources != 0; ++j )
		{
			const lpPiece* q = w->pieces.data + body->pieces.data[j];
			total += q->sources == p->sources ? q->shape->volume : 0.0f;
		}
		p->sourceShare = total > 0.0f ? p->shape->volume / total : 0.0f;
		lpCarriersChanged( w, p->carries );
	}

	lpBondParts( w, bodyIndex, first );

	w->bodies.data[bodyIndex].inertiaRadius = lpMaxFloat( def->inertiaRadius, 0.0f );
	if ( def->isStatic == false )
	{
		lpApplyMass( w, w->bodies.data + bodyIndex );
	}
	else
	{
		lpMarkDirty( w, bodyIndex ); // one stress check, so a structure that cannot stand comes down
	}
	return bodyIndex;
}

void lpWorld_AddImpact( lpWorld* w, const lpImpactDef* impact )
{
	lpGuardFp( w ); // computes in float between steps, on the caller's thread
	lpArray_Push( w->impacts, *impact );
}

// ---- piece queries ----

int lpCompareInt( const void* a, const void* b )
{
	int x = *(const int*)a;
	int y = *(const int*)b;
	return ( x > y ) - ( x < y );
}

int lpCompareBodyRef( const void* a, const void* b )
{
	const lpBodyRef* x = a;
	const lpBodyRef* y = b;
	if ( x->body != y->body )
	{
		return ( x->body > y->body ) - ( x->body < y->body );
	}
	return ( x->generation > y->generation ) - ( x->generation < y->generation );
}

// Pieces whose shapes overlap the box, sorted and unique (query order must not leak into results).
void lpQueryPieces( lpWorld* w, lpAABB box )
{
	const int* pieces;
	int count = lpPhys_OverlapBox( w->phys, box, lp_physQueryAll, &pieces );
	lpArray_Reserve( w->scratchPieces, count );
	if ( count > 0 )
	{
		memcpy( w->scratchPieces.data, pieces, (size_t)count * sizeof( int ) );
	}
	w->scratchPieces.count = count;
}

void lpWakeRubble( lpWorld* w, int bodyIndex )
{
	lpBody* b = w->bodies.data + bodyIndex;
	if ( b->kind != lp_kindRubble )
	{
		return;
	}
	b->kind = lp_kindDebris;
	lpPhys_SetDynamic( w->phys, b->id, true );
	lpApplyMass( w, b );
	lpPhys_SetAwake( w->phys, b->id, true );
}

lpStats lpWorld_GetStats( const lpWorld* w )
{
	return w->stats;
}

uint64_t lpWorld_Hash( const lpWorld* w )
{
	uint64_t h = LP_HASH_INIT;
	h = lpHashBytes( h, &w->tick, sizeof( w->tick ) );
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		if ( b->alive == false )
		{
			continue;
		}
		h = lpHashBytes( h, &i, sizeof( i ) );
		h = lpHashBytes( h, &b->kind, sizeof( b->kind ) );
		h = lpHashBytes( h, &b->tier, sizeof( b->tier ) );
		h = lpHashBytes( h, &b->pieces.count, sizeof( int ) );
		h = lpHashBytes( h, b->pieces.data, sizeof( int ) * (size_t)b->pieces.count );
		if ( b->gravityScale != 1.0f )
		{
			h = lpHashBytes( h, &b->gravityScale, sizeof( b->gravityScale ) ); // only when set: old hashes stay valid
		}
		if ( b->inertiaRadius != 0.0f )
		{
			h = lpHashBytes( h, &b->inertiaRadius, sizeof( b->inertiaRadius ) );
		}
		if ( b->kind == lp_kindGhost || b->kind == lp_kindScrap )
		{
			h = lpHashBytes( h, &b->com, sizeof( b->com ) );
			h = lpHashBytes( h, &b->q, sizeof( b->q ) );
			h = lpHashBytes( h, &b->v, sizeof( b->v ) );
			h = lpHashBytes( h, &b->omega, sizeof( b->omega ) );
			// the landing plan and the sinking decide where and when it stops
			h = lpHashBytes( h, &b->planTicks, sizeof( b->planTicks ) );
			h = lpHashBytes( h, &b->landIn, sizeof( b->landIn ) );
			h = lpHashBytes( h, &b->landPoint, sizeof( b->landPoint ) );
			h = lpHashBytes( h, &b->landNormal, sizeof( b->landNormal ) );
			h = lpHashBytes( h, &b->sinkTicks, sizeof( b->sinkTicks ) );
			continue;
		}
		lpWorldTransform xf = lpPhys_GetTransform( w->phys, b->id );
		lpVec3 v = lpPhys_GetLinearVelocity( w->phys, b->id );
		lpVec3 omega = lpPhys_GetAngularVelocity( w->phys, b->id );
		LP_ASSERT( lpIsValidVec3( xf.p ) && lpIsValidVec3( v ) && lpIsValidVec3( omega ) ); // NaN in state is a bug (rule 17)
		h = lpHashBytes( h, &xf, sizeof( xf ) );
		h = lpHashBytes( h, &v, sizeof( v ) );
		h = lpHashBytes( h, &omega, sizeof( omega ) );
	}
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		if ( p->body < 0 )
		{
			continue;
		}
		h = lpHashBytes( h, &p->body, sizeof( p->body ) );
		h = lpHashBytes( h, &p->shape->volume, sizeof( float ) );
		h = lpHashBytes( h, &p->shape->centroid, sizeof( lpVec3 ) );
		h = lpHashBytes( h, p->shape->vertices, sizeof( lpVec3 ) * (size_t)p->shape->vertexCount );
		h = lpHashBytes( h, p->bonds.data, sizeof( int ) * (size_t)p->bonds.count );
		if ( ( p->carries | p->needs ) != 0 )
		{
			uint8_t masks[3] = { p->carries, p->sources, p->needs }; // only when set: old hashes stay valid
			h = lpHashBytes( h, masks, sizeof( masks ) );
			h = lpHashBytes( h, &p->sourceShare, sizeof( float ) );
			h = lpHashBytes( h, p->supply, sizeof( p->supply ) );
		}
	}
	for ( int i = 0; i < w->bonds.count; ++i )
	{
		const lpBond* bond = w->bonds.data + i;
		if ( bond->alive )
		{
			h = lpHashBytes( h, &bond->a, sizeof( int ) );
			h = lpHashBytes( h, &bond->b, sizeof( int ) );
			h = lpHashBytes( h, &bond->health, sizeof( float ) );
		}
	}
	for ( int i = 0; i < w->detonators.count; ++i )
	{
		uint8_t armed = w->detonators.data[i].armed ? 1 : 0;
		h = lpHashBytes( h, &armed, sizeof( armed ) );
	}
	return lpHashLinks( w, h );
}

uint64_t lpWorld_HashStress( const lpWorld* w )
{
	uint64_t h = LP_HASH_INIT;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		if ( b->alive == false || b->kind != lp_kindStructure )
		{
			continue;
		}
		bool flags[4] = { b->solving, b->creaking, b->unsettled, b->strainedLastCheck };
		h = lpHashBytes( h, &i, sizeof( i ) );
		h = lpHashBytes( h, flags, sizeof( flags ) );
		h = lpHashBytes( h, &b->stressSteps, sizeof( b->stressSteps ) );
		if ( b->solving )
		{
			h = lpHashBytes( h, &b->solveRz, sizeof( b->solveRz ) );
		}
	}
	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		if ( p->body < 0 )
		{
			continue;
		}
		h = lpHashBytes( h, &p->stressX, sizeof( lpVec6 ) );
		h = lpHashBytes( h, &p->stressLoad, sizeof( lpVec6 ) );
		h = lpHashBytes( h, &p->strain, sizeof( float ) );
		if ( w->bodies.data[p->body].solving )
		{
			h = lpHashBytes( h, &p->stressR, sizeof( lpVec6 ) ); // a solve in progress continues from these
			h = lpHashBytes( h, &p->stressP, sizeof( lpVec6 ) );
		}
	}
	for ( int i = 0; i < w->bonds.count; ++i )
	{
		const lpBond* bond = w->bonds.data + i;
		if ( bond->alive )
		{
			h = lpHashBytes( h, &bond->rho, sizeof( float ) );
			h = lpHashBytes( h, &bond->strain, sizeof( float ) );
		}
	}
	return h;
}

// ---- queries ----

// Closest approach of the segment from the origin along d to the segment p-q: the fraction along d and the point on
// p-q, if they pass within `radius`
static bool lpRayNearSegment( lpVec3 d, lpVec3 p, lpVec3 q, float radius, float* fraction, lpVec3* point )
{
	lpVec3 e = lpSub( q, p );
	lpVec3 r = lpNeg( p );
	float a = lpDot( d, d ), ee = lpDot( e, e ), f = lpDot( e, r ), c = lpDot( d, r ), b = lpDot( d, e );
	float s = 0.0f, t = 0.0f;
	if ( a <= 1e-12f )
	{
		return false;
	}
	if ( ee <= 1e-12f )
	{
		s = lpClampFloat( -c / a, 0.0f, 1.0f );
	}
	else
	{
		float denom = a * ee - b * b;
		s = denom > 1e-12f ? lpClampFloat( ( b * f - c * ee ) / denom, 0.0f, 1.0f ) : 0.0f;
		t = ( b * s + f ) / ee;
		if ( t < 0.0f )
		{
			t = 0.0f;
			s = lpClampFloat( -c / a, 0.0f, 1.0f );
		}
		else if ( t > 1.0f )
		{
			t = 1.0f;
			s = lpClampFloat( ( b - c ) / a, 0.0f, 1.0f );
		}
	}
	lpVec3 onRope = lpMulAdd( p, t, e );
	if ( lpLength( lpSub( lpMulSV( s, d ), onRope ) ) > radius )
	{
		return false;
	}
	*fraction = s;
	*point = onRope;
	return true;
}

// A ray that starts inside a shape ignores it
static bool lpPastOrigin( int piece, float fraction, void* context )
{
	(void)piece;
	(void)context;
	return fraction != 0.0f;
}

lpRayHit lpWorld_CastRay( const lpWorld* w, lpPos origin, lpVec3 translation )
{
	lpRayHit hit = { 0 };
	hit.piece = -1;
	hit.body = -1;
	hit.link = -1;
	lpPhysCastHit result = lpPhys_CastRay( w->phys, origin, translation, lp_physQueryAll, lpPastOrigin, NULL );
	float nearest = result.hit ? result.fraction : 2.0f; // a rope at the very end of the ray still counts
	if ( result.hit )
	{
		hit.hit = true;
		hit.point = result.point;
		hit.normal = result.normal;
		if ( result.piece >= 0 )
		{
			hit.piece = result.piece;
			hit.body = w->pieces.data[hit.piece].body;
		}
	}

	// Ropes and wheels are no physics shapes: they are hit as capsules (a thin rope, a tyre round its axle), when nearer
	// than any shape
	for ( int i = 0; i < w->links.count; ++i )
	{
		const lpLink* l = w->links.data + i;
		float fraction;
		lpVec3 point;
		float radius = l->wheel >= 0 ? w->wheels.data[l->wheel].def.radius : 0.05f;
		if ( l->alive && ( l->def.type == lp_linkRope || l->wheel >= 0 ) &&
			 lpRayNearSegment( translation, lpSubPos( l->points[0], origin ), lpSubPos( l->points[1], origin ), radius,
							   &fraction, &point ) &&
			 fraction < nearest )
		{
			nearest = fraction;
			hit.hit = true;
			hit.point = lpOffsetPos( origin, point );
			hit.normal = lpNormalize( lpNeg( translation ) );
			hit.piece = -1;
			hit.body = -1;
			hit.link = i;
		}
	}
	return hit;
}

// ---- rendering access ----

int lpWorld_GetPieceCapacity( const lpWorld* w )
{
	return w->pieces.count;
}

lpPieceInfo lpWorld_GetPieceInfo( const lpWorld* w, int piece )
{
	const lpPiece* p = w->pieces.data + piece;
	lpPieceInfo info = { p->body, p->generation, p->userId, p->part, p->tag, p->body >= 0 ? p->shape->volume : 0.0f,
						 p->body >= 0 ? lpSuppliedMask( p ) : (uint8_t)0 };
	return info;
}

int lpWorld_GetBodyCapacity( const lpWorld* w )
{
	return w->bodies.count;
}

bool lpWorld_GetBodyTransform( const lpWorld* w, int body, lpWorldTransform* transform )
{
	const lpBody* b = w->bodies.data + body;
	if ( b->alive == false )
	{
		return false;
	}
	*transform = lpGetTransform( w, b );
	return true;
}

int lpWorld_BuildPieceMesh( const lpWorld* w, int piece, lpVertex* vertices, int capacity )
{
	const lpPiece* p = w->pieces.data + piece;
	if ( p->body < 0 )
	{
		return 0;
	}
	lpFacetParams params = { p->color, p->axis };
	return lpBuildFacetMesh( p->shape, &params, vertices, capacity );
}

int lpWorld_GetMaxPieceVertices( void )
{
	return LP_MAX_PIECE_VERTICES;
}

const lpParticle* lpWorld_GetParticles( const lpWorld* w, int* count )
{
	*count = w->particles.count;
	return w->particles.data;
}

// ---- validation ----

static bool lpFail( const char* message, int a, int b, int c )
{
	fprintf( stderr, "lpWorld_Validate: " );
	fprintf( stderr, message, a, b, c );
	fprintf( stderr, "\n" );
	return false;
}

bool lpWorld_Validate( const lpWorld* w )
{
	int bondRefs = 0;
	for ( int i = 0; i < w->bodies.count; ++i )
	{
		const lpBody* b = w->bodies.data + i;
		if ( b->alive == false )
		{
			continue;
		}
		if ( b->kind == lp_kindGhost || b->kind == lp_kindScrap )
		{
			if ( LP_PHYS_NULL( b->id ) == false || b->pieces.count == 0 )
			{
				return lpFail( "loose body %d has a Box3D body or no pieces (%d)", i, b->pieces.count, 0 );
			}
		}
		else if ( lpPhys_IsValidBody( w->phys, b->id ) == false )
		{
			return lpFail( "body %d has an invalid Box3D id", i, 0, 0 );
		}
		else if ( lpPhys_GetShapeCount( w->phys, b->id ) != b->pieces.count )
		{
			return lpFail( "body %d: %d shapes but %d pieces", i, lpPhys_GetShapeCount( w->phys, b->id ), b->pieces.count );
		}
		for ( int k = 0; k < b->pieces.count; ++k )
		{
			int pi = b->pieces.data[k];
			if ( pi < 0 || pi >= w->pieces.count || w->pieces.data[pi].body != i )
			{
				return lpFail( "body %d lists piece %d that is not on it (%d)", i, pi, pi >= 0 && pi < w->pieces.count ? w->pieces.data[pi].body : -2 );
			}
		}

		// A stress system belongs to a structure; while it solves, the system is the structure's current one
		const lpStressSystem* s = b->system;
		if ( s != NULL && b->kind != lp_kindStructure && b->solveStress == false )
		{
			return lpFail( "body %d of kind %d has a stress system", i, b->kind, 0 );
		}
		if ( s != NULL && b->solving && s->built && s->topology == b->topology )
		{
			for ( int k = 0; k < s->nodes.count; ++k )
			{
				int pi = s->nodes.data[k];
				if ( pi < 0 || pi >= w->pieces.count || w->pieces.data[pi].body != i || w->pieces.data[pi].solveSlot != k )
				{
					return lpFail( "body %d's stress system has node %d on piece %d, not its own", i, k, pi );
				}
			}
			if ( s->incidentStart.count != s->nodes.count + 1 || s->vectors.count != 6 * s->nodes.count )
			{
				return lpFail( "body %d's stress system is sized wrong (%d nodes, %d vectors)", i, s->nodes.count, s->vectors.count );
			}
		}
	}

	for ( int i = 0; i < w->pieces.count; ++i )
	{
		const lpPiece* p = w->pieces.data + i;
		if ( p->body < 0 )
		{
			continue;
		}
		if ( w->bodies.data[p->body].alive == false )
		{
			return lpFail( "piece %d on dead body %d", i, p->body, 0 );
		}
		uint8_t kind = w->bodies.data[p->body].kind;
		if ( kind == lp_kindGhost || kind == lp_kindScrap )
		{
			if ( LP_PHYS_NULL( p->shapeId ) == false )
			{
				return lpFail( "loose piece %d still has a Box3D shape", i, 0, 0 );
			}
		}
		else if ( lpPhys_IsValidShape( w->phys, p->shapeId ) == false )
		{
			return lpFail( "piece %d has no shape", i, 0, 0 );
		}
		if ( kind != lp_kindGhost && kind != lp_kindScrap && lpPhys_GetShapeBody( w->phys, p->shapeId ).index1 != w->bodies.data[p->body].id.index1 )
		{
			return lpFail( "piece %d shape on the wrong body", i, 0, 0 );
		}
		if ( p->shape == NULL || ( p->shape->volume > 0.0f ) == false )
		{
			return lpFail( "piece %d degenerate", i, 0, 0 );
		}
		if ( p->cluster != 0 && ( p->anchored || kind != lp_kindStructure ) )
		{
			return lpFail( "piece %d is in cluster %d but anchored or not on a structure (kind %d)", i, p->cluster, kind );
		}
		for ( int k = 0; k < p->bonds.count; ++k )
		{
			int bi = p->bonds.data[k];
			const lpBond* bond = w->bonds.data + bi;
			if ( bond->alive == false || ( bond->a != i && bond->b != i ) )
			{
				return lpFail( "piece %d lists bad bond %d", i, bi, 0 );
			}
			int other = bond->a == i ? bond->b : bond->a;
			if ( w->pieces.data[other].body != p->body )
			{
				return lpFail( "bond %d crosses bodies %d and %d", bi, p->body, w->pieces.data[other].body );
			}
			bondRefs += 1;
		}
	}

	int liveBonds = 0;
	for ( int i = 0; i < w->bonds.count; ++i )
	{
		const lpBond* bond = w->bonds.data + i;
		if ( bond->alive == false )
		{
			continue;
		}
		liveBonds += 1;
		float len2 = lpLengthSquared( bond->normal );
		if ( lpAbsFloat( len2 - 1.0f ) > 1e-3f || ( bond->h1 > 0.0f && bond->h2 > 0.0f ) == false )
		{
			return lpFail( "bond %d has a bad contact patch (joins %d and %d)", i, bond->a, bond->b );
		}
		if ( bond->joint == lp_jointAuto || bond->joint >= lp_jointCount )
		{
			return lpFail( "bond %d has joint %d (piece %d)", i, bond->joint, bond->a );
		}
		if ( bond->rho >= w->def.stressGlue && ( w->pieces.data[bond->a].cluster != 0 || w->pieces.data[bond->b].cluster != 0 ) )
		{
			return lpFail( "bond %d is loaded past the glue share but piece %d or %d is in a rigid cluster", i, bond->a, bond->b );
		}
	}
	if ( liveBonds != w->bondCount || bondRefs != 2 * liveBonds )
	{
		return lpFail( "bond count %d, live %d, refs %d", w->bondCount, liveBonds, bondRefs );
	}
	return lpValidateLinks( w ) && lpValidateRigs( w );
}

bool lpWorld_ValidateBondGeometry( const lpWorld* w )
{
	for ( int i = 0; i < w->bonds.count; ++i )
	{
		const lpBond* bond = w->bonds.data + i;
		if ( bond->alive == false )
		{
			continue;
		}
		const lpShape* a = w->pieces.data[bond->a].shape;
		const lpShape* b = w->pieces.data[bond->b].shape;
		if ( lpShape_NearlyOverlap( a, b, 0.035f ) == false )
		{
			return lpFail( "bond %d joins pieces %d and %d that do not touch", i, bond->a, bond->b );
		}
	}
	return true;
}
