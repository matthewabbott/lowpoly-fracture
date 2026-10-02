// SPDX-License-Identifier: MIT
// Internal layout of lpWorld, shared by the core's source files (world, impact, split, step, debris) and the tests.

#pragma once

#include "core.h"
#include "fracture.h"
#include "phys.h"
#include "poly.h"
#include "solve.h"

typedef struct lpTaskPool lpTaskPool;

// Collision categories (lpPhysFilter). Light shapes also run the pair filter (debris.c, lpPairFilter): a light piece
// touches a full piece only if the full piece is frozen static rubble.
#define LP_CAT_STATIC 0x01ull
#define LP_CAT_FULL 0x02ull
#define LP_CAT_LIGHT 0x04ull
#define LP_CAT_VEHICLE 0x08ull
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
	LP_ARRAY( lpVec3 ) nodeRef; // each fine node's reference point: its piece's centroid
	lpStressSystem system;		// P^T K P; its x is the correction y
	uint32_t topology;			// of the body when built
	uint32_t clusterStamp;
	bool built;
} lpStressReduced;

// A set of pieces growing into a rigid cluster (stress.c, lpFormClusters): union-find over a structure's nodes
typedef struct lpClusterSet
{
	int parent;
	int size;
	lpAABB box;
	bool eligible;
	int id;
} lpClusterSet;

// One structure's stress check in a step. Structures are solved in parallel: a job reads only its own structure's
// pieces and bonds and writes only their solve state, its body's system, and its own arrays (kept between steps).
typedef struct lpStressJob
{
	int body;
	lpStressSystem* system; // the body's
	lpWorldTransform xf;
	lpVec3 gravity;			  // body frame
	int nodeCount, edgeCount; // counted before the build, for the budget
	int budget;				  // iterations granted this step
	bool continuing;		  // pick up the solve in progress (r, p and the system, rz in solve)
	bool cached;			  // the body's system is still the structure's: no build
	bool clustered;			  // solved on the body's reduced system for a correction
	double tolerance;		  // of the whole residual, relative to the whole load
	float nodeTolerance;	  // of each node's residual, relative to the forces through it
	lpSolveState solve;
	int dissolved;					  // clusters the residual meter dissolved (then it is solved again)
	float oracleWorst;				  // tests (lpWorld.stressOracle): the worst utilization difference from the exact solve
	float meterWorst;				  // the meter's worst reading
	int oracleFlips;
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
	lpTransform frame;
} lpLinkEnd;

typedef struct lpLink
{
	lpLinkDef def;
	lpLinkEnd ends[2];
	lpPhysJoint joint;
	lpPhysBody builtOn[2]; // the physics bodies the joint was made on; when an end's body changes, it is rebuilt
	lpPhysBody anchor[2];	 // the static body of a world end
	lpPos points[2];	 // world points of the ends, cached at the last step either end was awake
	lpVec3 force;		 // on end B, world, N
	lpVec3 torque;
	lpVec3 stressForce;	  // the force and torque its structure ends were last re-checked for (world)
	lpVec3 stressTorque;
	uint64_t recheckTick; // tick + 1 of that re-check (0: never)
	float utilization; // load over limit, smoothed
	float strain;	   // breaks at 1
	float health;	   // of def.strength, after blasts
	uint32_t lastImpact;
	uint32_t generation;
	int settle; // steps before loads are judged: a rebuilt joint starts cold
	int wheel;	// its wheel (w->wheels) when it is one, else -1
	// Motor (hinges and ball joints with def.motor): its target, what was last given to the physics (set again only
	// when it changes; a rebuilt joint gets it all again), and what it did
	float target;
	lpQuat targetRotation;
	bool targetChanged;
	bool motorApplied;
	float appliedSpeed;
	lpVec3 appliedVelocity;
	float appliedCap;
	float motorCap;
	float motorTorque;
	float feed;	 // rad/s added to a hinge servo's speed: the motion its rig wants this step (0 for other links)
	float angle; // hinges: from the pose at creation, at the last step
	int nextFree;
	bool alive;
} lpLink;

#define LP_MAX_VEHICLE_WHEELS 16
#define LP_CHANNELS 8 // supply channels (lpPartSystem)

// A vehicle's wheel (wheel.c): a link with no joint, end 0 on the mount piece, end 1 on nothing. Its axes are the
// vehicle's, in the chassis body frame, which every body split off the chassis shares.
typedef struct lpWheel
{
	int link; // -1: a free slot
	int vehicle;
	int slot; // in its vehicle's wheel list
	lpWheelDef def;
	float sprungMass; // its share of the chassis at creation: a mount body much lighter than that tears it off
	float steer;	  // radians
	float spin;		  // radians, for drawing
	float spinSpeed;
	float length; // suspension length at the last cast
	float load;	  // N on the ground
	float slip;	  // sideways speed at the contact before the solve, m/s
	bool grounded;
	bool atStop; // bottomed out: the cast started inside the ground
	lpPos contactPoint;
	lpVec3 contactNormal; // out of the ground
	int groundPiece;	  // -1: none
	uint32_t groundGeneration;
	lpWorldTransform hub; // world, at the last step its chassis was awake
	lpVec3 hubVelocity;
	lpVec3 bodyOmega;
	lpVec3 stressForce; // what its ground structure was last re-checked for (world)
	int stressBody;		// that structure (-1: none)
	uint64_t recheckTick;
	bool sliding;	// its grip gave at the last solve: it slides on with less
	float friction; // of the ground
	// This step's solve (world)
	lpVec3 suspension; // force on the chassis
	lpVec3 r;		   // contact point from the chassis's centre of mass
	lpVec3 dirF, dirS; // along and across the tyre, in the ground's plane
	lpVec3 groundVelocity;
	float massN, massF, massS;
	float lambdaN, lambdaF, lambdaS;
	int nextFree;
} lpWheel;

// A wheel on a chassis body, for grouping this step's tyre solves by body
typedef struct lpBodyWheel
{
	int body;
	int wheel;
} lpBodyWheel;

typedef struct lpVehicle
{
	lpVehicleDef def; // wheels cleared
	lpVec3 forward;	  // chassis body frame
	lpVec3 up;
	lpVehicleControl control;
	bool controlChanged;
	bool alive;
	int wheelCount;
	int links[LP_MAX_VEHICLE_WHEELS]; // each wheel's link as created (-1: it came off)
} lpVehicle;

// A limb of a rig (rig.c): a chain of hinge links from the torso outward. Each link's inner end (prox: 0 or 1) is on
// the body the link before it turns; its angle turns the outer end's frame about the inner end's z by sign * angle.
typedef struct lpLimb
{
	lpLimbDef def;
	uint8_t prox[LP_MAX_LIMB_JOINTS]; // the end of each link on the inner body
	uint32_t gen[LP_MAX_LIMB_JOINTS]; // each link's generation at creation (a slot reused by another link is not it)
	float lower[LP_MAX_LIMB_JOINTS];  // the angles its targets keep within (just inside the hinge's limits)
	float upper[LP_MAX_LIMB_JOINTS];
	lpVec3 defFoot; // the foot as created, in the frame of the last link's outer body
	// Capability, every step
	int joints;		// links on in a chain from the torso
	int rootBody;	// the body of its first link's inner end (-1: that link is gone)
	int tipBody;	// the outer body of its last joint on: the foot is on it
	uint32_t tipGeneration;
	uint32_t tipTopology;
	int tipJoints;	// joints when the foot was found
	lpVec3 foot;	// in tipBody's frame
	float strength; // of its weakest joint's servo, 0 to 1
	float reach;
	float depth;	// below the torso's frame, how far the foot reaches at its neutral point (found with the foot)
	bool attached;
	bool able;
	bool planted;
	float q[LP_MAX_LIMB_JOINTS]; // the angles last given to its servos
	// Gait (gait.c)
	lpVec3 neutral; // torso frame: where its foot rests under the torso, as created
	bool swinging;
	float swingClock; // s into its swing
	bool castLate;	  // the second foothold cast (two thirds through) is done
	bool grounded;	  // the last cast found ground
	lpPos liftoff;	  // world
	lpPos landing;
	lpPos hold;		  // world: where a planted foot is kept
	float holdClock;  // s since it was set down
	bool arrived;	  // it got there: the hold is fixed
	// Reaching (lpWorld_SetLimbTarget)
	bool reachWanted; // told to reach
	lpPos reachPoint; // world
	bool reaching;	  // out of the gait, reaching
	int touching;	  // the piece its foot touches while reaching (-1: none)
	lpFootTarget target; // lp_walkerNone: where the game drives its foot
	int groundPiece;  // under the foothold (-1: none, or not a piece)
	uint32_t groundGeneration;
	uint64_t recheckTick; // tick + 1 it last asked its ground structure for a stress check
} lpLimb;

typedef struct lpRig
{
	lpRigDef def; // limbs cleared
	lpVec3 forward; // torso body frame (the creation body's; bodies split off it share it)
	lpVec3 up;
	lpRigControl control;
	bool controlChanged;
	bool alive;
	int body; // the torso this step (-1: none)
	int limbCount;
	lpLimb limbs[LP_MAX_RIG_LIMBS];
	lpWorldTransform desired; // where the torso's frame is pushed: level, at its height, moving with the controls
	float height;			  // the torso's frame above its planted feet, this step
	bool idle;				  // targets frozen: standing still, settled
	int calm;				  // steps settled and still toward the idle latch
	bool crawling;			  // fewer than 4 able limbs, or stuck: on its belly
	bool stuck;				  // stalled with no foot able to lift: it crawls until its able limbs change
	int stall;				  // steps stalled so far
	int ableSeen;			  // able limbs when that was last reset
	bool posed;				  // lp_walkerNone: the game set `desired` (else it follows the torso)
	lpVec3 poseLinear;		  // lp_walkerNone: the pose's motion, fed forward
	lpVec3 poseAngular;
} lpRig;

// A wheel that came off, spawned as an object of its own at the start of the next step
typedef struct lpLostWheel
{
	lpWorldTransform hub;
	lpVec3 velocity;
	lpVec3 omega;
	float radius, width;
	uint8_t material;
	uint32_t color;
} lpLostWheel;

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
	lpPhysHull* hull; // cached physics hull of the shape, reused when the piece changes body; NULL for ghost ejecta
	lpPhysShape shapeId; // null while the piece is on a ghost or scrap body
	LP_ARRAY( int ) bonds;
	LP_ARRAY( int ) links; // links with an end on this piece
	lpPlane anchorPlane;
	lpVec3 axis; // grain axis (wood) or pane normal (glass), body frame
	uint32_t color;
	uint32_t seed;
	uint32_t generation;
	int body;
	int nextFree;
	int mark;
	int solveSlot;	// stress solve: node index, -1 for anchored pieces
	lpVec6 stressX; // stress solve: last solution in newtons of load (the warm start after a topology change)
	lpVec6 stressLoad; // contact load from what rests on it (newtons, body frame), sampled when a solve starts
	lpVec6 stressResidual; // newtons: what its structure's last judged solve left unbalanced at it, within its tolerance.
						   // A correction solves only for what changed since, so it does not chase this everywhere.
	float strain; // stress overload accumulated inside the piece (slender pieces break mid-span at 1)
	int cluster;	   // the rigid cluster it moves with in its structure's stress solve (from 1), 0: a node of its own
	float slenderRho;  // slender pieces: the worst section's utilization at the last judged solve (0: not slender)
	float slenderAt;   // where along its axis, from its centroid
	float slenderDepth; // of the section
	uint32_t changed;  // w->changeSerial when its bonds, their health or its load last changed
	uint32_t accepted; // w->changeSerial when its structure's last solve was judged: changed after it, it is a seed
	uint32_t userId;   // of the object it came from
	uint16_t part;	   // index of the part it came from in that object
	uint16_t tag;	   // that part's system tag
	uint8_t carries;   // that part's channels (lpPartSystem)
	uint8_t sources;
	uint8_t needs;
	float sourceShare; // of its object's sources of the same channels, by volume (split among its fracture cells)
	int detonator;	   // its part's detonator, w->detonators + 1; 0: inert
	int pool;		   // its source part's pool, w->pools + 1; 0: none
	uint8_t supply[LP_CHANNELS]; // how well each channel is fed here, of 255 (supply.c)
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
	lpVec3 centroid;	// body frame
	lpVec3 normal;		// unit, body frame, from piece a toward piece b
	float h1, h2;		// half-extents of the contact patch along lpContactBasis( normal )
	float strain;		// stress overload accumulated over checks; the bond breaks at 1
	float rho;			// utilization at the last converged check (1 = at its limit)
	lpVec3 force;		// it carries at the last converged check (newtons, body frame, lpEdgeForce's sign: tension along +normal)
	lpVec3 moment;
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
	lpPhysBody id; // null for ghost and scrap
	LP_ARRAY( int ) pieces;
	uint64_t createdTick;
	float volume;
	int nextFree;
	int stamp;
	uint32_t generation; // bumped each time the slot is reused: (index, generation) names one body for good
	float gravityScale;	 // multiplies gravity on the body and on whatever breaks off it ("fairy dust")
	float inertiaRadius; // m: rotational inertia added about every axis, as mass * r^2 (lpObjectDef.inertiaRadius)
	uint64_t linkStamp;	 // tick + 1 when a link end was on it at this step's sync: never frozen or demoted
	uint8_t kind;
	uint8_t tier;
	bool alive;
	bool dirty;
	bool freezePending;
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
	bool solveClustered; // the last solve ran on the reduced system
	bool provisional;	 // structure: judged on a reduced system; an exact audit is queued (w->audits)
	bool auditing;		 // structure: its audit is running
	uint64_t provisionalTick; // tick + 1 it became provisional

	double solveRz;
	uint64_t hitCheckTick; // last tick a hit asked for a stress check (hits re-check a structure at most every 30)
	uint64_t loadCheckTick; // moving, solving its stress: tick + 1 a link's load last asked for a check (links ask one at
							// most every 30 steps: a walker's legs swing their loads every stride)
	uint64_t joltTick;		// moving, solving its stress: tick + 1 it or a body linked to it was hit hard; for 10 steps
							// its links' loads are checked as they come (a landing's peak comes a step or two after it)
	// A moving body whose stress is solved (lpObjectDef.solveStress): on request only, exactly, pinned at one piece
	// (stressPin, the nearest its centre of mass) with its loads balanced by its own acceleration (inertia relief: a,
	// alpha and omega in its frame, about reliefCenter), sampled with its loads
	bool solveStress;
	int stressPin;
	lpVec3 reliefAccel;
	lpVec3 reliefAlpha;
	lpVec3 reliefOmega;
	lpVec3 reliefCenter;
	// Its velocities at the end of the last two steps (world), for the acceleration it actually had: a crash's contact
	// is gone by the time it is checked (the impact broke it). stepTick: the tick of the newer; stepPair: the older is
	// from the step before.
	lpVec3 stepV[2];
	lpVec3 stepOmega[2];
	uint64_t stepTick;
	bool stepPair;
	// Struck at hitTick, at hitPoint (body frame), on hitMaterial: what it did not sample enters there (the pin is the
	// piece nearest it; the struck one may be broken by then), and its crumpling spreads the stop
	lpVec3 hitPoint;
	uint8_t hitMaterial;
	uint64_t hitTick;

	// Ghost and scrap state. The body frame is com - q * localCenter, so piece geometry stays in object space.
	lpPos com;
	lpQuat q;
	lpVec3 v;
	lpVec3 omega;
	lpVec3 localCenter;
	int planTicks; // ticks the last landing cast still covers
	int landIn;	   // ticks until the planned landing, or -1
	lpPos landPoint;
	lpVec3 landNormal;
	int sinkTicks; // scrap over budget sinks into the ground, then goes

	// Loose-debris grid (ghosts and scrap), intrusive per-slot lists
	int gridSlot;
	int gridPrev;
	int gridNext;
} lpBody;

// A source part's pool (supply.c), shared by every piece made from it
typedef struct lpPool
{
	float capacity;
	float level;
	float leak;	 // per second
	float seal;	 // s a leak takes to close (0: never)
	float reach; // carrier volume its lowest channel reached at the last supply update (-1: not yet)
	float found; // this update's
	int step;	 // level in sixteenths at the last supply update
	float leakRate; // lpPartSystem's, defaults filled in
	float pressure;
} lpPool;

// A detonator of a part or an object, shared by every piece made from it: the first of them to go off disarms it
typedef struct lpDetonator
{
	lpDetonatorDef def;
	bool armed;
	bool lit;	// its fuse is burning (def.delay): it goes off when the fuse is out, from the pieces that still carry it
	float fuse; // s left
} lpDetonator;

// Pieces of a body that detonated (detonator + 1 of theirs), removed at the start of the next step
typedef struct lpPendingBlast
{
	int body;
	uint32_t generation;
	int detonator;
} lpPendingBlast;

typedef struct lpPull
{
	int piece;
	lpVec3 localPoint;
	lpPos target;
	float maxAccel;
	float maxMass;
} lpPull;

typedef struct lpBlow
{
	lpPos origin;
	lpVec3 direction;
	float range;
	float cosAngle;
	float speed;
} lpBlow;

typedef struct lpWake
{
	lpVec3 center;
	float radius;
} lpWake;

typedef struct lpForce
{
	lpPos point;
	lpVec3 direction;
	float radius;
	float impulse;
	bool explosion;
} lpForce;

typedef struct lpComponent
{
	int first; // into scratchQueue
	int count;
	float volume;
	lpVec3 centroid; // body frame, volume weighted
	bool anchored;
	int material; // of the largest piece
	float largest;
} lpComponent;

typedef struct lpHitCandidate
{
	float energy;
	lpPos point;
	uint64_t key;
} lpHitCandidate;

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
	lpWorldDef def; // def.materials and def.joints point at the world's own copies below
	lpPhys* phys;
	lpMaterialDef materials[lp_materialCount];
	lpJointDef joints[lp_jointCount];

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
	LP_ARRAY( lpWheel ) wheels;
	int freeWheel;
	LP_ARRAY( lpVehicle ) vehicles;
	LP_ARRAY( lpLostWheel ) lostWheels;
	LP_ARRAY( lpBodyWheel ) scratchWheels;
	LP_ARRAY( lpRig ) rigs;
	LP_ARRAY( int ) stressAgain; // structures that lost bonds to their own weight; re-checked next step
	LP_ARRAY( int ) stressQueue; // structures updated this step, checked together after the splits (stress.c)
	lpStressJob* stressJobs;	 // this step's solves; the first stressJobCount are in use
	int stressJobCount;
	int stressJobCapacity;
	LP_ARRAY( lpOverload ) scratchOverloads;
	LP_ARRAY( lpVec6 ) scratchLoads;
	LP_ARRAY( int ) scratchClusters;
	LP_ARRAY( lpClusterSet ) scratchSets;
	int stressWork; // bond-iterations used this step, over all structures
	LP_ARRAY( lpBodyRef ) audits; // provisional structures in the order they became so (stress.c)
	lpBodyRef audit;			  // the one being audited (body -1: none)
	int calmSteps;				  // consecutive steps with the stress budget at most half used and nothing waiting
	// Tests: every solve on a reduced system is checked against an exact fine solve of the same change, and the worst
	// joint utilization difference is kept (lpStressOracle)
	bool stressOracle;
	float oracleWorst;	// over the solves since it was reset
	float oracleMeter;	// what the meter read at that solve
	int oracleSolves;
	int oracleFlips; // joints the exact solve would strain and the reduced one not, or the other way round
	int oracleJoints; // joints checked
	LP_ARRAY( lpPendingBlast ) pendingDestroy; // detonated pieces, removed at the start of the next step
	LP_ARRAY( lpDetonator ) detonators;
	LP_ARRAY( lpPool ) pools;
	bool supplyDirty; // a carrier's connections changed: supply is recomputed before the next physics step
	LP_ARRAY( int ) scratchCarriers;
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
	int fpRepairs; // control words put back on the calling thread (lpFpGuard), since the world was made
	lpStats stats;
};

static inline const lpMaterialDef* lpMaterial( const lpWorld* w, int material )
{
	LP_ASSERT( 0 <= material && material < lp_materialCount );
	return w->materials + material;
}

static inline const lpJointDef* lpJoint( const lpWorld* w, int joint )
{
	LP_ASSERT( 0 <= joint && joint < lp_jointCount );
	return w->joints + joint;
}

// ---- shared internals (world.c, impact.c, split.c, step.c) ----

int lpAllocBody( lpWorld* w );
int lpAllocPiece( lpWorld* w );
void lpFreePieceSlot( lpWorld* w, int index );
bool lpCreatePieceShape( lpWorld* w, int pieceIndex, int bodyIndex );
bool lpAttachPiece( lpWorld* w, int pieceIndex, int bodyIndex );
void lpDetachPieceShape( lpWorld* w, int pieceIndex );
void lpDestroyBody( lpWorld* w, int bodyIndex, bool emitDust );
int lpCreateBodyInternal( lpWorld* w, lpWorldTransform xf, bool dynamic, uint8_t kind, uint8_t tier, lpVec3 v,
						  lpVec3 omega, float gravityScale );
void lpEmitParticle( lpWorld* w, lpWorldTransform xf, lpVec3 localPoint, lpVec3 velocity, float size, uint8_t material );
void lpQueryPieces( lpWorld* w, lpAABB box );
void lpWakeRubble( lpWorld* w, int bodyIndex );
lpWorldTransform lpGetTransform( const lpWorld* w, const lpBody* b );
int lpCompareInt( const void* a, const void* b );

// Puts back a changed floating-point control word on the calling thread (determinism rule 15) and counts it
static inline void lpGuardFp( lpWorld* w )
{
	w->fpRepairs += lpFpGuard() ? 1 : 0;
}
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
// Burns the lit fuses by a step, and sets off the charges whose fuse is out
void lpBurnFuses( lpWorld* w, float timeStep );
void lpUpdateBody( lpWorld* w, int bodyIndex );
void lpUpdateDirtyBodies( lpWorld* w ); // lpUpdateBody on every dirty body, in the order they were marked

// links (link.c): an end's piece leaving the physics breaks the link at once (lpBreakPieceLinks); lpSyncLinks rebuilds
// joints whose ends changed body, just before the physics step; lpPollLinks judges their loads just after it
void lpBreakLink( lpWorld* w, int index, bool dust );
void lpBreakPieceLinks( lpWorld* w, int piece );
// Fracture: a piece's link ends move to the cell holding their anchor (detach before the piece is freed; attach once
// the cells are placed, cellToPiece giving each kept cell's piece or -1), or the links break
void lpDetachLinks( lpWorld* w, int piece, lpShape* const* cells, int cellCount );
void lpAttachLinks( lpWorld* w, const int* cellToPiece );
void lpSyncLinks( lpWorld* w );
void lpDriveMotors( lpWorld* w ); // after lpSyncLinks and supply: servo speeds and torque caps
void lpPollLinks( lpWorld* w, float timeStep );
bool lpBodyLinked( const lpWorld* w, const lpBody* b );
// Touching a linked body that moves (a crate in a cart): it must not freeze, or the assembly would jam on it
bool lpTouchesLinked( lpWorld* w, const lpBody* b );
uint64_t lpHashLinks( const lpWorld* w, uint64_t h );
bool lpValidateLinks( const lpWorld* w );
// A wheel's link: end 0 on the piece nearest the mount (within reach), end 1 on nothing, no joint. Returns -1 when no
// piece of the body is near enough.
int lpCreateWheelLink( lpWorld* w, int body, lpPos mount, float maxForce, float strength, int wheel );
// Distance from the origin to the segment a-b
float lpSegmentDistance( lpVec3 a, lpVec3 b );

// vehicles (wheel.c): lpSpawnLostWheels at the start of a step (wheels that came off last step become objects);
// lpStepVehicles just before the physics step (steering, suspension casts, the tyre solve, forces)
void lpSpawnLostWheels( lpWorld* w );
void lpStepVehicles( lpWorld* w, float timeStep );
void lpReleaseWheel( lpWorld* w, int wheel, bool comesOff ); // its link is going
// Wheels standing on a structure load it (stress.c): force on the ground at the contact, world
void lpAddWheelLoads( lpWorld* w, int bodyIndex, lpWorldTransform xf );
uint64_t lpHashVehicles( const lpWorld* w, uint64_t h );

// supply (supply.c): recomputed once a step, after lpSyncLinks, when a carrier's connections changed
void lpUpdateSupply( lpWorld* w );
uint8_t lpSuppliedMask( const lpPiece* p );			  // channels fed at all here
// Before the supply update: leaks drain their pools and close; a pool that crosses a sixteenth asks for an update
void lpDrainPools( lpWorld* w, float timeStep );
uint64_t lpHashPools( const lpWorld* w, uint64_t h );
float lpSupplyOf( const lpPiece* p, uint8_t channels ); // the worst of those channels here, 0 to 1 (1 for none)
static inline void lpCarriersChanged( lpWorld* w, uint8_t channels )
{
	w->supplyDirty = w->supplyDirty || channels != 0;
}
bool lpValidateWheel( const lpWorld* w, int link );
void lpFreeVehicles( lpWorld* w );

// The physics mass from a body's shapes, plus its inertia padding (lpObjectDef.inertiaRadius): the solver softens a
// joint by the lighter body's inertia, and a slender limb has little about its long axis
static inline void lpApplyMass( lpWorld* w, const lpBody* b )
{
	lpPhys_UpdateMass( w->phys, b->id, b->inertiaRadius );
}

// rigs (rig.c): lpStepRigs after the supply update and before the servos are driven (capability, then the gait's
// stance, swings and targets: lpWalkRig in gait.c)
void lpStepRigs( lpWorld* w, float timeStep );
void lpWalkRig( lpWorld* w, lpRig* r, float timeStep );
// The rig's centre of mass: its torso and the bodies of its limbs' chains
lpPos lpRigCenter( const lpWorld* w, const lpRig* r );
// How far `point` lies inside the convex hull of the feet in `use`, seen along up (negative outside; -FLT_MAX with fewer
// than three)
float lpSupportMargin( const lpPos* feet, const bool* use, int count, lpPos point, lpVec3 up );
// A structure a foot lands on or leaves carries a changed load: it is checked again (at most every 30 steps per foot)
void lpFootMoved( lpWorld* w, lpLimb* limb );
// Drives a limb's foot to `foot` (world) as seen from the rig's pose (r->desired) moving at linear and angular, the foot
// itself moving at `motion`: its servos' targets are the IK seeded from limb->q, fed the joint speeds that keep the
// foot there; with strike > 0 (1/s) its joints go at full speed until that near their targets instead (a stomp). Every
// walker drives its feet through this.
void lpDriveFoot( lpWorld* w, const lpRig* r, lpLimb* limb, lpPos foot, lpVec3 motion, lpVec3 linear, lpVec3 angular,
				  float strike );
lpVec3 lpRigWorldUp( const lpWorld* w, const lpRig* r, lpQuat torso ); // against gravity (the rig's own up without it)
lpPos lpFootWorld( const lpWorld* w, const lpLimb* limb );
// Joint speeds that move a limb's foot at `velocity` (torso frame): damped least squares on its Jacobian
void lpLimbSpeeds( int joints, const lpVec3* axes, const lpVec3* origins, lpVec3 foot, lpVec3 velocity, float* out );
uint64_t lpHashRigs( const lpWorld* w, uint64_t h );
bool lpValidateRigs( const lpWorld* w );
void lpFreeRigs( lpWorld* w );
// A limb's kinematics in the torso frame (tests reach them too): the foot (in the tip's frame) of its first `joints`
// links at the angles q, each joint's signed axis and point into axes and origins; and IK, the angles (q, warm on
// entry) that put the foot at target within the limits, returning how far short it falls
lpVec3 lpLimbForward( const lpWorld* w, const lpLimb* limb, int joints, const float* q, lpVec3 foot, lpVec3* axes, lpVec3* origins );
float lpLimbIK( const lpWorld* w, const lpLimb* limb, int joints, lpVec3 foot, lpVec3 target, float* q );
// What a motorised link's servo can put in now: its max torque by its health and supply, its hold torque (or what a
// jam holds) unfed; and of that, what it can drive with
float lpMotorCap( const lpWorld* w, const lpLink* l );
float lpMotorDrive( const lpWorld* w, const lpLink* l );

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
// After the physics step: moving bodies that solve their stress keep their velocities (lpBody.stepV)
void lpTrackMovingBodies( lpWorld* w );

float lpParticleVolume( const lpWorld* w, int material );
float lpGhostVolume( const lpWorld* w, int material );
float lpLightVolume( const lpWorld* w, int material );

// ---- debris tiers (debris.c) ----

// A new ghost body at a body-frame transform; add pieces with lpAddLoosePiece, then call lpFinishLoose.
int lpBeginGhost( lpWorld* w, lpWorldTransform xf, lpVec3 v, lpVec3 omega, float gravityScale );
void lpAddLoosePiece( lpWorld* w, int bodyIndex, int pieceIndex );
void lpFinishLoose( lpWorld* w, int bodyIndex, lpWorldTransform xf );

// In-place tier changes of a body (same body index, so rendering and references stay valid)
void lpConvertToGhost( lpWorld* w, int bodyIndex );
void lpConvertToScrap( lpWorld* w, int bodyIndex );
void lpConvertToLight( lpWorld* w, int bodyIndex );
void lpConvertToFull( lpWorld* w, int bodyIndex );

void lpGridInit( lpWorld* w );
void lpGridFree( lpWorld* w );
void lpGridRemove( lpWorld* w, int bodyIndex );

// Loose (ghost/scrap) bodies whose centre lies in the box, sorted by index, into w->scratchLoose.
void lpQueryLoose( lpWorld* w, lpAABB box );

void lpStepGhosts( lpWorld* w, float timeStep );
void lpApplyLooseForce( lpWorld* w, const lpForce* force );
void lpShove( lpWorld* w, float timeStep );
void lpApplyBlows( lpWorld* w );
void lpEnforceBudgets( lpWorld* w );
// Debris the physics engine put to sleep is frozen into rubble once old enough; debris below the kill depth goes
void lpFreezeOrKill( lpWorld* w );
bool lpPairFilter( int pieceA, int pieceB, void* context );

// ---- validation (tests) ----

// Full invariant check. Returns false and prints the first violation.
bool lpWorld_Validate( const lpWorld* world );

// Every live bond joins two pieces that touch (within the weld margin). Slower; for tests.
bool lpWorld_ValidateBondGeometry( const lpWorld* world );
