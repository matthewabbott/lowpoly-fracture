// SPDX-License-Identifier: MIT
// Internal layout of lpWorld, shared by the core's source files (world, impact, split, step, debris) and the tests.

#pragma once

#include "core.h"
#include "fracture.h"
#include "poly.h"

typedef struct lpTaskPool lpTaskPool;

typedef enum lpBodyKind
{
	lp_kindStructure, // static until pieces break loose; anchored components stay
	lp_kindDebris,	  // dynamic Box3D body
	lp_kindRubble,	  // debris that settled and was frozen static; wakes when something happens nearby
	lp_kindGhost,	  // no Box3D body: flies ballistically and passes through everything
	lp_kindScrap,	  // no Box3D body: a landed ghost, render-only
} lpBodyKind;

// How much a Box3D debris body interacts (kept when it freezes into rubble)
typedef enum lpTier
{
	lp_tierFull,  // collides with everything
	lp_tierLight, // collides with static geometry only; cannot push anything
} lpTier;

// Fate of a cell after a fracture
typedef enum lpCellClass
{
	lp_cellKeep,  // stays on the parent body, bonded to its neighbours
	lp_cellPuff,  // particles only
	lp_cellGhost, // ejected as a ghost (no Box3D body)
	lp_cellLight, // ejected as a light debris body
	lp_cellFull,  // ejected as a full debris body
} lpCellClass;

// Box3D collision categories. Light shapes also run the custom filter (debris.c, lpCustomFilter): a light piece
// touches a full piece only if the full piece is frozen static rubble.
#define LP_CAT_STATIC 0x01ull
#define LP_CAT_FULL 0x02ull
#define LP_CAT_LIGHT 0x04ull
#define LP_CAT_VEHICLE 0x08ull
#define LP_CAT_CHARACTER 0x10ull
#define LP_CAT_PROJECTILE 0x20ull
#define LP_CAT_ALL 0xFFFFFFFFFFFFFFFFull

typedef struct lpPiece
{
	lpShape* shape;	  // body frame; NULL for a free slot
	b3HullData* hull; // cached Box3D hull of the shape, reused when the piece changes body; NULL for ghost ejecta
	b3ShapeId shapeId; // null while the piece is on a ghost or scrap body
	LP_ARRAY( int ) bonds;
	b3Plane anchorPlane;
	b3Vec3 axis; // grain axis (wood) or pane normal (glass), body frame
	uint32_t color;
	uint32_t seed;
	uint32_t generation;
	int body;
	int nextFree;
	int mark;
	int groundDepth; // stress pass: BFS depth
	int loadSlot;	 // stress pass: index into scratchLoad
	uint8_t material;
	uint8_t depth;
	bool anchored;
} lpPiece;

typedef struct lpBond
{
	int a, b; // a < b
	float area;
	float health;		// J/m^2 of damage left
	float strength;		// health when intact
	float loadStrength; // N/m^2 under the weight check, when intact
	b3Vec3 centroid;	// body frame
	uint32_t lastImpact; // serial of the last impact that damaged it (deferred fractures must not damage twice)
	int nextFree;
	bool alive;
} lpBond;

typedef struct lpBody
{
	b3BodyId id; // null for ghost and scrap
	LP_ARRAY( int ) pieces;
	uint64_t createdTick;
	float volume;
	int nextFree;
	int stamp;
	uint8_t kind;
	uint8_t tier;
	bool alive;
	bool dirty;
	bool freezePending;
	bool armed;
	lpDetonatorDef detonator;

	// Ghost and scrap state. The body frame is com - q * localCenter, so piece geometry stays in object space.
	b3Pos com;
	b3Quat q;
	b3Vec3 v;
	b3Vec3 omega;
	b3Vec3 localCenter;
	int planTicks; // ticks the last landing cast still covers
	int landIn;	   // ticks until the planned landing, or -1
	b3Pos landPoint;
	b3Vec3 landNormal;
	int sinkTicks; // scrap over budget sinks into the ground, then goes

	// Loose-debris grid (ghosts and scrap), intrusive per-slot lists
	int gridSlot;
	int gridPrev;
	int gridNext;
} lpBody;

typedef struct lpPull
{
	int piece;
	b3Vec3 localPoint;
	b3Pos target;
	float maxAccel;
	float maxMass;
} lpPull;

typedef struct lpBlow
{
	b3Pos origin;
	b3Vec3 direction;
	float range;
	float cosAngle;
	float speed;
} lpBlow;

typedef struct lpWake
{
	b3Vec3 center;
	float radius;
} lpWake;

typedef struct lpForce
{
	b3Pos point;
	b3Vec3 direction;
	float radius;
	float impulse;
	bool explosion;
} lpForce;

typedef struct lpComponent
{
	int first; // into scratchQueue
	int count;
	float volume;
	b3Vec3 centroid; // body frame, volume weighted
	bool anchored;
	int material; // of the largest piece
	float largest;
} lpComponent;

typedef struct lpHitCandidate
{
	float energy;
	b3Pos point;
	uint64_t key;
} lpHitCandidate;

#define LP_MAX_CELL_BONDS ( LP_MAX_SITES * 24 )

// One piece to fracture during an impact (see impact.c, "fracture jobs")
typedef struct lpFractureJob
{
	int piece;
	b3Vec3 localImpact; // body frame
	b3Vec3 center;		// piece centroid; the fracture runs in a frame centered here
	lpPoly poly;
	lpFractureInput input;
	float particleVolume; // tier thresholds of the piece's material, scaled
	float ghostVolume;
	float lightVolume;

	int cellCount;
	lpShape* cells[LP_MAX_SITES];
	int cellSites[LP_MAX_SITES];
	uint8_t cellClass[LP_MAX_SITES];
	b3HullData* hulls[LP_MAX_SITES];
	int bondCount;
	lpCellBond* bonds; // LP_MAX_CELL_BONDS
	lpFractureStats stats;
} lpFractureJob;

// A piece whose fracture did not fit in the step's budget; it runs at the start of the next step
typedef struct lpDeferredJob
{
	int piece;
	uint32_t generation;
	uint32_t impactSerial;
	lpImpactDef impact;
} lpDeferredJob;

struct lpWorld
{
	lpWorldDef def;

	LP_ARRAY( lpPiece ) pieces;
	LP_ARRAY( lpBond ) bonds;
	LP_ARRAY( lpBody ) bodies;
	int freePiece;
	int freeBond;
	int freeBody;
	int bondCount;

	LP_ARRAY( lpImpactDef ) impacts;	 // to apply at the next step
	LP_ARRAY( lpImpactDef ) nextImpacts; // collision impacts found during a step
	LP_ARRAY( lpParticle ) particles;	 // emitted during the last step
	LP_ARRAY( lpWake ) pendingWakes;
	LP_ARRAY( lpForce ) forces;
	LP_ARRAY( int ) dirtyBodies;
	LP_ARRAY( int ) freezeCandidates;
	LP_ARRAY( int ) stressAgain; // structures that lost bonds to their own weight; re-checked next step
	LP_ARRAY( float ) scratchLoad;
	LP_ARRAY( int ) pendingDestroy; // detonated bodies, removed at the start of the next step
	LP_ARRAY( lpPull ) pulls;
	LP_ARRAY( lpBlow ) blows;
	LP_ARRAY( lpDeferredJob ) deferred;

	LP_ARRAY( int ) scratchPieces;
	LP_ARRAY( int ) scratchBodies;
	LP_ARRAY( int ) scratchLoose;
	LP_ARRAY( int ) scratchQueue;
	LP_ARRAY( lpComponent ) scratchComponents;
	LP_ARRAY( lpHitCandidate ) scratchHits;

	// Loose-debris grid: hashed 2D cells in x/z, each slot heads a list of ghost/scrap bodies
	int* gridHeads;
	int gridSlotCount;

	lpTaskPool* tasks;
	lpFractureJob* jobs;
	int jobCount;
	int jobCapacity;
	int jobsThisStep;

	uint64_t tick;
	uint64_t pieceSerial;
	uint32_t impactSerial;
	int stamp;
	int freezesThisStep;
	lpStats stats;
};

// ---- shared internals (world.c, impact.c, split.c, step.c) ----

int lpAllocBody( lpWorld* w );
int lpAllocPiece( lpWorld* w );
void lpFreePieceSlot( lpWorld* w, int index );
bool lpCreatePieceShape( lpWorld* w, int pieceIndex, int bodyIndex );
bool lpAttachPiece( lpWorld* w, int pieceIndex, int bodyIndex );
void lpDetachPieceShape( lpWorld* w, int pieceIndex );
void lpDestroyBody( lpWorld* w, int bodyIndex, bool emitDust );
int lpCreateBodyInternal( lpWorld* w, b3WorldTransform xf, b3BodyType type, uint8_t kind, uint8_t tier, b3Vec3 v, b3Vec3 omega );
void lpEmitParticle( lpWorld* w, b3WorldTransform xf, b3Vec3 localPoint, b3Vec3 velocity, float size, uint8_t material );
void lpQueryPieces( lpWorld* w, b3AABB box );
void lpWakeRubble( lpWorld* w, int bodyIndex );
b3WorldTransform lpGetTransform( const lpBody* b );
int lpCompareInt( const void* a, const void* b );

// Tier thresholds (volume, m^3) of a material, scaled by the world's debrisScale
// bonds and dirty bodies (world.c)
void lpBreakBond( lpWorld* w, int bondIndex );
void lpAddBond( lpWorld* w, int a, int b, float area, b3Vec3 centroid );
void lpTryBond( lpWorld* w, int a, int b );
void lpMarkDirty( lpWorld* w, int bodyIndex );

// impacts (impact.c), connectivity (split.c)
void lpProcessDeferred( lpWorld* w );
void lpProcessImpact( lpWorld* w, const lpImpactDef* impact );
void lpApplyForces( lpWorld* w );
void lpCollectHits( lpWorld* w );
void lpUpdateBody( lpWorld* w, int bodyIndex );

float lpParticleVolume( const lpWorld* w, int material );
float lpGhostVolume( const lpWorld* w, int material );
float lpLightVolume( const lpWorld* w, int material );

// ---- debris tiers (debris.c) ----

// A new ghost body at a body-frame transform; add pieces with lpAddLoosePiece, then call lpFinishLoose.
int lpBeginGhost( lpWorld* w, b3WorldTransform xf, b3Vec3 v, b3Vec3 omega );
void lpAddLoosePiece( lpWorld* w, int bodyIndex, int pieceIndex );
void lpFinishLoose( lpWorld* w, int bodyIndex, b3WorldTransform xf );

// In-place tier changes of a body (same body index, so rendering and references stay valid)
void lpConvertToGhost( lpWorld* w, int bodyIndex );
void lpConvertToScrap( lpWorld* w, int bodyIndex );
void lpConvertToLight( lpWorld* w, int bodyIndex );
void lpConvertToFull( lpWorld* w, int bodyIndex );

void lpGridInit( lpWorld* w );
void lpGridFree( lpWorld* w );
void lpGridRemove( lpWorld* w, int bodyIndex );

// Loose (ghost/scrap) bodies whose centre lies in the box, sorted by index, into w->scratchLoose.
void lpQueryLoose( lpWorld* w, b3AABB box );

void lpStepGhosts( lpWorld* w, float timeStep );
void lpApplyLooseForce( lpWorld* w, const lpForce* force );
void lpShove( lpWorld* w, float timeStep );
void lpApplyBlows( lpWorld* w );
void lpEnforceBudgets( lpWorld* w );
bool lpCustomFilter( b3ShapeId shapeA, b3ShapeId shapeB, void* context );

// ---- validation (tests) ----

// Full invariant check. Returns false and prints the first violation.
bool lpWorld_Validate( const lpWorld* world );

// Every live bond joins two pieces that touch (within the weld margin). Slower; for tests.
bool lpWorld_ValidateBondGeometry( const lpWorld* world );
