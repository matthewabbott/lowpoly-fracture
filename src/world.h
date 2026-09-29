// SPDX-License-Identifier: MIT
// Internal layout of lpWorld, shared by the core's source files (world, impact, split, step, debris) and the tests.

#pragma once

#include "core.h"
#include "fracture.h"
#include "poly.h"
#include "solve.h"

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

typedef struct lpOverload
{
	float rho;
	int bond;
} lpOverload;

// A slender piece's worst section at a converged solve (stress.c, lpStressSlender)
typedef struct lpSlenderCut
{
	int piece;
	float length;
	float worst;   // utilization of the worst of the cuts
	float worstAt; // where along the piece's axis, from its centroid
	float depth;   // of the section
} lpSlenderCut;

// A structure whose parts move as rigid clusters is solved on its reduced system for the correction to its last
// solution (the delta form: x = xOld + P y, P^T K P y = P^T (f - K xOld)), so the clusters bias only the change and
// the bonds inside a cluster keep their last forces. Kept on the body while the solve continues.
typedef struct lpStressReduced
{
	lpPartition partition;
	LP_ARRAY( b3Vec3 ) nodeRef; // each fine node's reference point: its piece's centroid
	lpStressSystem system;		// P^T K P; its x is the correction y
	uint32_t topology;			// of the body when built
	uint32_t clusterStamp;
	bool built;
} lpStressReduced;

// One structure's stress check in a step. Structures are solved in parallel: a job reads only its own structure's
// pieces and bonds and writes only their solve state, its body's system, and its own arrays (kept between steps).
typedef struct lpStressJob
{
	int body;
	lpStressSystem* system; // the body's
	b3WorldTransform xf;
	b3Vec3 gravity;			  // body frame
	int nodeCount, edgeCount; // counted before the build, for the budget
	int budget;				  // iterations granted this step
	bool continuing;		  // pick up the solve in progress (r, p and the system, rz in solve)
	bool cached;			  // the body's system is still the structure's: no build
	bool clustered;			  // solved on the body's reduced system for a correction
	double tolerance;		  // of the whole residual, relative to the whole load
	float nodeTolerance;	  // of each node's residual, relative to the forces through it
	lpSolveState solve;
	float peak;						  // converged: the highest joint utilization (in the system's rho)
	LP_ARRAY( lpSlenderCut ) slender; // converged: the slender pieces' worst sections
	LP_ARRAY( int ) clusterGroup;	  // scratch: the group of each cluster
	LP_ARRAY( float ) groupMass;	  // scratch
} lpStressJob;

// One end of a link: a piece and a frame in its body's frame, which never changes for the piece (new bodies are made
// at their parent's transform). Piece -1 is a world end, held by a static body of its own.
typedef struct lpLinkEnd
{
	int piece;
	uint32_t generation;
	b3Transform frame;
} lpLinkEnd;

typedef struct lpLink
{
	lpLinkDef def;
	lpLinkEnd ends[2];
	b3JointId joint;
	b3BodyId builtOn[2]; // the Box3D bodies the joint was made on; when an end's body changes, it is rebuilt
	b3BodyId anchor[2];	 // the static body of a world end
	b3Pos points[2];	 // world points of the ends, cached at the last step either end was awake
	b3Vec3 force;		 // on end B, world, N
	b3Vec3 torque;
	b3Vec3 stressForce;	  // the force and torque its structure ends were last re-checked for (world)
	b3Vec3 stressTorque;
	uint64_t recheckTick; // tick + 1 of that re-check (0: never)
	float utilization; // load over limit, smoothed
	float strain;	   // breaks at 1
	float health;	   // of def.strength, after blasts
	uint32_t lastImpact;
	uint32_t generation;
	int settle; // steps before loads are judged: a rebuilt joint starts cold
	int nextFree;
	bool alive;
} lpLink;

// A link end waiting for its fractured piece's cells to be placed (see lpDetachLinks)
typedef struct lpLinkMove
{
	int link;
	int end;
	int cell; // the cell that holds the anchor
} lpLinkMove;

typedef struct lpPiece
{
	lpShape* shape;	  // body frame; NULL for a free slot
	b3HullData* hull; // cached Box3D hull of the shape, reused when the piece changes body; NULL for ghost ejecta
	b3ShapeId shapeId; // null while the piece is on a ghost or scrap body
	LP_ARRAY( int ) bonds;
	LP_ARRAY( int ) links; // links with an end on this piece
	b3Plane anchorPlane;
	b3Vec3 axis; // grain axis (wood) or pane normal (glass), body frame
	uint32_t color;
	uint32_t seed;
	uint32_t generation;
	int body;
	int nextFree;
	int mark;
	int solveSlot;	// stress solve: node index, -1 for anchored pieces
	lpVec6 stressX; // stress solve: last solution in newtons of load (the warm start after a topology change)
	lpVec6 stressR; // stress solve: residual and search direction, so a solve continues across steps
	lpVec6 stressP;
	lpVec6 stressLoad; // contact load from what rests on it (newtons, body frame), sampled when a solve starts
	float strain; // stress overload accumulated inside the piece (slender pieces break mid-span at 1)
	int cluster;	   // the rigid cluster it moves with in its structure's stress solve (from 1), 0: a node of its own
	float slenderRho;  // slender pieces: the worst section's utilization at the last judged solve (0: not slender)
	float slenderAt;   // where along its axis, from its centroid
	float slenderDepth; // of the section
	uint32_t changed;  // w->changeSerial when its bonds, their health or its load last changed
	uint32_t accepted; // w->changeSerial when its structure's last solve was judged: changed after it, it is a seed
	uint8_t material;
	uint8_t joint; // lpJointId where this piece meets other parts (never auto)
	uint8_t depth;
	bool anchored;
} lpPiece;

typedef struct lpBond
{
	int a, b; // a < b
	float area;
	float health;		// J/m^2 of damage left
	float strength;		// health when intact
	b3Vec3 centroid;	// body frame
	b3Vec3 normal;		// unit, body frame, from piece a toward piece b
	float h1, h2;		// half-extents of the contact patch along lpContactBasis( normal )
	float strain;		// stress overload accumulated over checks; the bond breaks at 1
	float rho;			// utilization at the last converged check (1 = at its limit)
	b3Vec3 force;		// it carries at the last converged check (newtons, body frame, lpEdgeForce's sign: tension along +normal)
	b3Vec3 moment;
	uint8_t joint;		// lpJointId (never auto): solid between cells of one part
	uint32_t lastImpact; // serial of the last impact that damaged it (deferred fractures must not damage twice)
	int nextFree;
	bool alive;
} lpBond;

// A body named for good: its slot index plus the generation of that slot (indices are reused)
typedef struct lpBodyRef
{
	int body;
	uint32_t generation;
} lpBodyRef;

typedef struct lpBody
{
	b3BodyId id; // null for ghost and scrap
	LP_ARRAY( int ) pieces;
	uint64_t createdTick;
	float volume;
	int nextFree;
	int stamp;
	uint32_t generation; // bumped each time the slot is reused: (index, generation) names one body for good
	float gravityScale;	 // multiplies gravity on the body and on whatever breaks off it ("fairy dust")
	uint64_t linkStamp;	 // tick + 1 when a link end was on it at this step's sync: never frozen or demoted
	uint8_t kind;
	uint8_t tier;
	bool alive;
	bool dirty;
	bool freezePending;
	bool armed;
	bool unsettled;	   // structure: still solving, or joints straining toward a break
	bool solving;	   // structure: a solve is in progress (r, p on the pieces, solveRz here)
	bool creaking;	   // structure: converged, joints straining but none broken: checks only add strain
	bool strainedLastCheck; // structure: something was over its limit at the last converged check
	bool reloadLoads;		// structure: what rests or hangs on it changed; sample its loads at the next check
	bool rejudge;			// structure: a joint's health dropped; judge the joints again from the last solve's forces
	int stressSteps;   // structure: steps spent solving the current topology
	uint32_t topology; // bumped whenever a bond or piece of the body changes; a solve in progress restarts then
	uint32_t solveTopology;
	uint32_t splitTopology; // topology at the last split that found nothing to split off
	bool splitChecked;		// splitTopology is set
	lpStressSystem* system; // structure: its stress system, kept between checks (NULL until the first)
	lpStressReduced* reduced; // structure: its reduced system while parts of it move as rigid clusters
	uint32_t clusterStamp;	  // structure: bumped whenever its pieces' clusters change
	int clusters;			  // structure: clusters formed at its last exact solve (some may have dissolved since)
	int meterRounds;		  // structure: clusters dissolved by the residual meter since its last judgement
	int solveNodes, solveEdges;
	double solveRz;
	uint64_t hitCheckTick; // last tick a hit asked for a stress check (hits re-check a structure at most every 30)
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
	lpImpactDef impact; // what broke it: new bonds between its cells start with the damage it did there
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
	bool snap; // a stress snap inside the piece: fracture only, no bond damage around it
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
	LP_ARRAY( lpLink ) links;
	int freeLink;
	int linkCount;
	LP_ARRAY( lpLinkMove ) scratchLinkMoves;
	LP_ARRAY( int ) stressAgain; // structures that lost bonds to their own weight; re-checked next step
	LP_ARRAY( int ) stressQueue; // structures updated this step, checked together after the splits (stress.c)
	lpStressJob* stressJobs;	 // this step's solves; the first stressJobCount are in use
	int stressJobCount;
	int stressJobCapacity;
	LP_ARRAY( lpOverload ) scratchOverloads;
	LP_ARRAY( b3ContactData ) scratchContacts;
	LP_ARRAY( lpVec6 ) scratchLoads;
	LP_ARRAY( int ) scratchClusters;
	int stressWork; // bond-iterations used this step, over all structures
	LP_ARRAY( lpBodyRef ) pendingDestroy; // detonated bodies, removed at the start of the next step
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
	float lastTimeStep; // of the last physics step, to turn contact impulses into forces
	uint64_t pieceSerial;
	uint32_t impactSerial;
	uint32_t changeSerial; // stamps pieces whose bonds or loads change (lpPiece.changed)
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
int lpCreateBodyInternal( lpWorld* w, b3WorldTransform xf, b3BodyType type, uint8_t kind, uint8_t tier, b3Vec3 v, b3Vec3 omega,
						  float gravityScale );
void lpEmitParticle( lpWorld* w, b3WorldTransform xf, b3Vec3 localPoint, b3Vec3 velocity, float size, uint8_t material );
void lpQueryPieces( lpWorld* w, b3AABB box );
void lpWakeRubble( lpWorld* w, int bodyIndex );
b3WorldTransform lpGetTransform( const lpBody* b );
int lpCompareInt( const void* a, const void* b );
int lpCompareBodyRef( const void* a, const void* b ); // by body, then generation

// Tier thresholds (volume, m^3) of a material, scaled by the world's debrisScale
// bonds and dirty bodies (world.c)
void lpBreakBond( lpWorld* w, int bondIndex );
int lpAddBond( lpWorld* w, int a, int b, const lpContact* contact, uint8_t joint );
void lpTryBond( lpWorld* w, int a, int b );
void lpMarkDirty( lpWorld* w, int bodyIndex );

// impacts (impact.c), connectivity (split.c)
void lpProcessDeferred( lpWorld* w );
void lpProcessImpact( lpWorld* w, const lpImpactDef* impact );
void lpApplyForces( lpWorld* w );
void lpCollectHits( lpWorld* w );
void lpUpdateBody( lpWorld* w, int bodyIndex );
void lpUpdateDirtyBodies( lpWorld* w ); // lpUpdateBody on every dirty body, in the order they were marked

// links (link.c): an end's piece leaving Box3D breaks the link at once (lpBreakPieceLinks); lpSyncLinks rebuilds
// joints whose ends changed body, just before the physics step; lpPollLinks judges their loads just after it
void lpBreakLink( lpWorld* w, int index, bool dust );
void lpBreakPieceLinks( lpWorld* w, int piece );
// Fracture: a piece's link ends move to the cell holding their anchor (detach before the piece is freed; attach once
// the cells are placed, cellToPiece giving each kept cell's piece or -1), or the links break
void lpDetachLinks( lpWorld* w, int piece, lpShape* const* cells, int cellCount );
void lpAttachLinks( lpWorld* w, const int* cellToPiece );
void lpSyncLinks( lpWorld* w );
void lpPollLinks( lpWorld* w, float timeStep );
bool lpBodyLinked( const lpWorld* w, const lpBody* b );
// Touching a linked body that moves (a crate in a cart): it must not freeze, or the assembly would jam on it
bool lpTouchesLinked( lpWorld* w, const lpBody* b );
uint64_t lpHashLinks( const lpWorld* w, uint64_t h );
bool lpValidateLinks( const lpWorld* w );
void lpFreeLinks( lpWorld* w, bool physicsAlive );

// stress (stress.c): check every structure in w->stressQueue (solves in parallel, breaks in queue order); structures
// still solving or straining are marked dirty for the next step. Settling solves each to convergence, with no budget.
// Returns the iterations spent.
int lpCheckStructures( lpWorld* w, bool settle );
// Something changed what a structure carries (a hit, a link's pull): check it again next step with fresh loads. During
// the step's splits the dirty list is being walked, so there it waits with the structures to check again.
void lpRequestStressCheck( lpWorld* w, int bodyIndex, bool duringSplits );
// A piece's bonds, their health or its load changed (for the stress check's seeds)
static inline void lpTouchPiece( lpWorld* w, int piece )
{
	w->pieces.data[piece].changed = ++w->changeSerial;
}
void lpFreeStressSystem( lpBody* b );

float lpParticleVolume( const lpWorld* w, int material );
float lpGhostVolume( const lpWorld* w, int material );
float lpLightVolume( const lpWorld* w, int material );

// ---- debris tiers (debris.c) ----

// A new ghost body at a body-frame transform; add pieces with lpAddLoosePiece, then call lpFinishLoose.
int lpBeginGhost( lpWorld* w, b3WorldTransform xf, b3Vec3 v, b3Vec3 omega, float gravityScale );
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
