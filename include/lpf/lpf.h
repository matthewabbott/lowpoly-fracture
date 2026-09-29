// SPDX-License-Identifier: MIT
// lpf: low-poly polygonal destruction on Box3D.
//
// Objects are compounds of convex pieces. Impacts refracture the struck pieces into convex cells, damage the
// bonds between pieces, and pieces cut off from their anchors become Box3D debris bodies. Settled debris
// freezes into static rubble. Everything is deterministic: the same calls in the same order give
// bit-identical results (see docs/determinism-rules.md).

#pragma once

#include "box3d/box3d.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- materials ----

typedef enum lpMaterialId
{
	lp_wood,
	lp_stone,
	lp_brick,
	lp_plaster,
	lp_concrete,
	lp_glass,
	lp_metal,
	lp_ground,
	lp_foliage,
	lp_sheetMetal, // car bodies: panels bolted to a floor pan, tear off in big plates, crumple (crush) in a crash
	lp_rubber,	   // tyres, bumpers
	lp_materialCount
} lpMaterialId;

typedef enum lpPatternId
{
	lp_breakImpact, // Voronoi dense at the impact, coarse away from it: stone, brick, concrete, plaster
	lp_breakGrain,	// Voronoi in a space squashed along the grain, so cells come out as long splinters: wood
	lp_breakRadial, // wedges cut by concentric chords around the impact, in the plane of a thin pane: glass
	lp_breakMasonry, // along the mortar of a course grid: loose bricks, stair-stepped holes: brick
} lpPatternId;

// Cosmetic particle look, per material
typedef enum lpParticleKind
{
	lp_particleDust,
	lp_particleChip,
	lp_particleSplinter,
	lp_particleLeaf,
	lp_particleGlint,
} lpParticleKind;

// Debris tiers by volume (m^3, per material): below particleVolume a fragment is a puff of particles; below
// ghostVolume a ghost (no collision, falls like rock, lands as render-only scrap); below lightVolume light debris
// (collides with static geometry only, cannot push anything, rests as rubble that movers shove aside); above it,
// full physics (rests as fragile rubble that wakes when anything approaches).
typedef struct lpMaterialDef
{
	const char* name;
	float density;		  // kg/m^3
	float bondStrength;	  // energy density (J/m^2) that breaks a bond between pieces
	float fractureEnergy; // energy density (J/m^2) that refractures a piece
	float fragmentSize;	  // edge of the smallest fragments at the impact, meters
	float friction;
	float restitution;
	int pattern;		// lpPatternId
	float grainStretch; // wood: cells this many times longer along the grain
	uint32_t interiorColor; // 0xRRGGBB of freshly exposed faces
	bool breakable;
	float particleVolume;
	float ghostVolume;
	float lightVolume;
	float plateSize; // spacing of the few large "plate" cells away from the impact; keeps remainders whole
	int maxCells;	 // cells per fracture
	int particleKind; // lpParticleKind
	float mergeSlack; // remaining cells merge while their convex hull is at most this much bigger (0.3 = 30%)
	int chipSplits;	  // small ejecta cells are split this many times into real chips (a dirtier mess)

	// Stress limits of the solid material, Pa. Joints between parts have their own (lpJointDef).
	float tensileStrength;
	float compressiveStrength;
	float shearStrength;

	// Masonry (lp_breakMasonry): the course grid, in the object's frame so neighbouring panels line up
	float courseHeight;
	float brickLength;

	// Share of a collision's energy its crumpling soaks up before anything breaks (sheet metal, rubber): the harder
	// of two things that hit decides
	float crush;
} lpMaterialDef;

const lpMaterialDef* lpGetMaterial( int materialId );

// How two parts are joined. A bond between parts takes the weaker joint of the two; a bond between fracture cells of
// one part is solid. Structures break at their joints first: mortar before brick, nails before timber.
typedef enum lpJointId
{
	lp_jointAuto,	// by material: mortar for stone, brick, concrete and plaster, nails for wood, bolts for sheet metal,
					// solid otherwise
	lp_jointSolid,	// as strong as the weaker of the two materials
	lp_jointMortar, // weak in tension: masonry cracks and hinges at its joints
	lp_jointDry,	// stacked with nothing between: no tension or cohesion, friction only
	lp_jointNails,
	lp_jointBolts,	// sheet metal panels on a car
	lp_jointMounts, // an engine on its mounts: tears off in a hard crash
	lp_jointCount
} lpJointId;

typedef struct lpJointDef
{
	const char* name;
	float tensileStrength;	   // Pa
	float compressiveStrength; // Pa
	float shearStrength;	   // Pa of cohesion; friction x compression adds to it
	float friction;
} lpJointDef;

const lpJointDef* lpGetJoint( int jointId );

// ---- world ----

typedef struct lpWorldDef
{
	b3WorldId physics;
	uint64_t seed;
	// Budgets. Over budget, the smallest-oldest bodies move down a tier (full -> light -> ghost -> particles,
	// rubble -> scrap, scrap sinks away) instead of popping out of existence.
	int maxFullDebris;	 // moving full-physics debris bodies
	int maxLightDebris;	 // moving light debris bodies
	int maxGhosts;		 // flying ghost bodies
	int maxRubblePieces; // frozen rubble pieces
	int maxScrapPieces;	 // landed render-only scrap pieces
	float fragmentScale; // multiplies every material's fragment size
	float debrisScale;	 // multiplies every material's tier volumes (larger = cheaper debris)
	bool freezeRubble;	 // debris that Box3D puts to sleep becomes static rubble (costs nothing to simulate)
	int maxFractureJobsPerStep; // pieces refractured per step; the rest wait for the next step (spike guard)
	int maxFreezesPerStep;		// debris frozen into rubble per step (body type changes are costly)
	int maxGhostCastsPerStep;	// landing ray casts per step
	float wakeSpeed;			// approach speed at which a hit wakes frozen rubble, m/s
	int maxDepth;		   // refracture depth limit per piece lineage
	int maxHitImpacts;	   // collision impacts processed per step
	float hitSpeed;		   // minimum approach speed for collision damage, m/s
	float killDepth;	   // bodies falling below this height are removed
	bool debugLog;		   // print every processed impact to stdout (for diagnosing tuning)
	int workerCount;	   // threads for fracture and stress work, including the caller (results do not depend on it)
	float stressScale;	   // multiplies every strength in the stress solve; 0 disables collapse under weight
	// Stress budgets, in bond-iterations (about 65 ns each). Structures solve in parallel, so the per-structure cap
	// bounds the step's stress time on enough cores and the total bounds the CPU; on one core set them equal.
	int maxStressWork;			// per step, over all structures; structures past it wait for the next step
	int maxStressStructureWork; // per structure per step; a structure that needs more keeps creaking for a few steps
	int maxStressIterations;	// per structure per step
	int maxSettleIterations;	// per structure in lpWorld_SettleStructures, which has no per-step budget
	// Structures with more pieces than this solve changes on a reduced system: their lightly loaded parts, found by
	// their last exact solve, move as rigid clusters, and only the correction to that solution is solved for
	int stressLargeNodes;
	float stressGlue; // a joint loaded to this share of its limit keeps both its pieces out of clusters
	int maxStressBreaks;   // joints a structure may lose per check below twice their limit (worse ones go at once)
	int stressPatience;	   // steps on one solve before its tolerance relaxes from 0.1% to 1%
	float strainRate;	   // how fast an overloaded joint gives: at 1, 10% over its limit lasts 10 checks
	int maxLinks;		   // live links; bodies with links are exempt from freezing and the debris budgets
	int maxWheelCastsPerStep; // wheel suspension casts per step; a wheel not cast keeps its last contact
} lpWorldDef;

lpWorldDef lpDefaultWorldDef( void );

typedef struct lpWorld lpWorld;

lpWorld* lpCreateWorld( const lpWorldDef* def );
void lpDestroyWorld( lpWorld* world );

// ---- objects ----

// Makes a part or an object explode: when it hits something at triggerSpeed or faster, or when a nearby blast reaches
// it. A thrown alchemical flask, a gas can, volatile cargo, a car's fuel tank. Zero radius means inert.
typedef struct lpDetonatorDef
{
	float triggerSpeed; // m/s approach speed (at least lpWorldDef.hitSpeed, where collisions start to register)
	float radius;
	float energy;
	float speed; // blast push, m/s at the center
} lpDetonatorDef;

// What a part is to the systems a machine or a creature runs on. There are 8 channels (bit n is channel n); the game
// names them (fuel, power, steering; blood, nerves). A bond between two parts that carry a channel carries it, and so
// does a link that carries it; a part supplied with a channel is one its sources reach through carriers. Fracture
// cells inherit all of it.
typedef struct lpPartSystem
{
	uint16_t tag;	 // the game's name for the part (an engine, a fuel tank, a heart); 0: none. The core never reads it.
	uint8_t carries; // channels it carries
	uint8_t sources; // channels it feeds (it carries them too)
	uint8_t needs;	 // channels it must be fed before it feeds its own; only those below its lowest source count
} lpPartSystem;

// One convex part of an object, in object space. A box when pointCount is zero, else the hull of points.
typedef struct lpPartDef
{
	b3Vec3 halfExtents;
	b3Transform transform;
	const b3Vec3* points;
	int pointCount;
	uint8_t material;
	uint32_t color;	  // exterior 0xRRGGBB
	b3Vec3 grainAxis; // object space; zero picks the longest box axis
	bool anchored;	  // rests on a foundation: bonded to the world through its bottom face
	uint8_t joint;	  // lpJointId where this part meets its neighbours (lp_jointAuto: by material)
	lpPartSystem system;
	lpDetonatorDef detonator; // this part goes off alone, and takes only its own pieces with it (a fuel tank)
} lpPartDef;

typedef struct lpObjectDef
{
	b3WorldTransform transform;
	bool isStatic; // structures stay static until pieces break loose
	const lpPartDef* parts;
	int partCount;
	b3Vec3 linearVelocity;
	b3Vec3 angularVelocity;
	lpDetonatorDef detonator; // the parts without one of their own share it: they go off together, as one
	float gravityScale;		  // "fairy dust": 1 is normal weight, 0 floats; everything that breaks off keeps it
	uint32_t userId;		  // the game's id for the object, kept by every piece made from it
	// A moving object whose joints carry what accelerates it (a car in a crash, a mech landing): its stress is solved
	// on hard hits and landings, against its loads less its own acceleration ("inertia relief"). What breaks off
	// keeps it.
	bool solveStress;
} lpObjectDef;

lpObjectDef lpDefaultObjectDef( void );
lpPartDef lpDefaultPartDef( void );

// Returns the body index of the object.
int lpCreateObject( lpWorld* world, const lpObjectDef* def );

// ---- links ----
//
// Box3D joints between objects: a weld, a hinge, a ball joint or a rope. A link breaks when its load stays over its
// limit (it creaks first), and it outlives the pieces it was made on: when they split off or fracture, it follows the
// piece that holds its anchor, and breaks if that piece is lost.

typedef enum lpLinkType
{
	lp_linkWeld,  // holds position and orientation (softened by hertz for wobbly assemblies)
	lp_linkHinge, // turns about an axis, optionally between two angles
	lp_linkBall,  // turns freely about a point, optionally within a cone
	lp_linkRope,  // holds two points at most `length` apart
	lp_linkWheel, // a vehicle's wheel, made by lpCreateVehicle: one end on its mount, the other on the road
	lp_linkTypeCount
} lpLinkType;

// A motorised hinge or ball joint (a muscle, a crane's slew): a servo turns it toward its target
// (lpWorld_SetLinkTarget, lpWorld_SetLinkTargetRotation) with at most maxTorque, less as the link is damaged and as
// what it needs is fed. It holds up to that torque and gives past it: a weak muscle sags. Unfed, it brakes with at most
// holdTorque (a worm gear holds, a severed muscle does not). Its own torque never counts against the link's maxTorque.
typedef struct lpMotorDef
{
	float maxTorque;  // N*m; 0 (and no holdTorque) = no motor
	float maxSpeed;	  // rad/s
	float gain;		  // 1/s: turning speed per radian from its target
	uint8_t needs;	  // supply channels it needs fed at either end (0: none)
	float holdTorque; // N*m it brakes with when unfed
} lpMotorDef;

typedef struct lpLinkDef
{
	int type;					  // lpLinkType
	int bodyA;					  // body index at creation (the piece nearest the anchor holds it); -1 = the world
	int bodyB;					  // likewise; A != B
	b3Pos anchorA;				  // world: the joint point, or the rope's end on A
	b3Pos anchorB;				  // world: the rope's end on B (ropes only)
	b3Vec3 axis;				  // hinge axis, ball cone axis (world, unit)
	float length;				  // rope: longest length; 0 = the distance between its ends at creation
	float lowerAngle, upperAngle; // hinge, radians from the pose at creation; equal = free
	float coneAngle;			  // ball: 0 = free
	float hertz, dampingRatio;	  // weld softness; 0 hertz = rigid
	float maxForce;				  // N; 0 = no limit
	float maxTorque;			  // N*m; 0 = no limit
	float strength;				  // impact damage it takes, like a bond's (J/m^2); 0 = immune to blasts
	bool collideConnected;		  // false stops ALL collision between the two bodies, not only near the link
	uint8_t carries;			  // supply channels it carries between its ends (a fuel hose, a power cable)
	lpMotorDef motor;			  // hinges and ball joints
	uint32_t userId;			  // the game's id for the link
	float tearRatio;			  // rebuilt between two moving bodies, it tears when one is under this share of the other's
								  // mass (0: 0.02); a strong servo on a chip left of a limb would whip it about
} lpLinkDef;

// Defaults for a type: the limits of a hemp rope, an iron hinge or ball joint, a bolted weld
lpLinkDef lpDefaultLinkDef( int type );

// Returns the link index, or -1: the same body at both ends, a ghost or scrap end, no piece within 0.25 m of an
// anchor, maxLinks reached, or a wheel (made by lpCreateVehicle).
int lpCreateLink( lpWorld* world, const lpLinkDef* def );
void lpDestroyLink( lpWorld* world, int link );

// Winches and cranes: a rope's longest length
void lpWorld_SetRopeLength( lpWorld* world, int link, float length );

// A motorised hinge's target angle (radians from its pose at creation), or a ball joint's target rotation of its B frame
// relative to its A frame (identity: its pose at creation). They persist, and are part of the state (hashed).
void lpWorld_SetLinkTarget( lpWorld* world, int link, float angle );
void lpWorld_SetLinkTargetRotation( lpWorld* world, int link, b3Quat rotation );

typedef struct lpLinkState
{
	bool alive;
	bool slack; // a rope whose ends are closer than its length
	int type;
	uint32_t generation;  // bumped each time the link slot is reused
	int bodyA, bodyB;	  // current bodies of the two ends (-1: the world)
	b3Pos pointA, pointB; // world points of the two ends
	b3Vec3 force;		  // on B, world, N, at the last step either end was awake
	b3Vec3 torque;
	float utilization; // load over limit, smoothed; over 1 the link strains
	float strain;	   // it breaks at 1
	float health;	   // what is left of its strength after blasts
	float length;	   // a rope's longest length
	uint8_t supplied;  // supply channels fed at either end
	uint32_t userId;
	float angle;	   // a hinge's angle from its pose at creation
	float motorTorque; // what its motor is putting in, N*m
	float motorCap;	   // what its motor can put in now: less when damaged or unfed
} lpLinkState;

// Cached at the last step: safe at any time
lpLinkState lpWorld_GetLinkState( const lpWorld* world, int link );
int lpWorld_GetLinkCapacity( const lpWorld* world ); // every link index is below this

// ---- vehicles ----
//
// A vehicle is a body on wheels. A wheel is a link (lp_linkWheel) with one end on the piece at its mount and the other
// on the road: its suspension casts the tyre down from the mount, and a small impulse solve per body gives it grip.
// Like any link it follows its piece through splits and fractures, takes blast damage and strains under load; when it
// breaks (a hard landing, a blast, its mount blown out or torn off) it comes off as a wheel of its own. A vehicle
// never names a body: cut a chassis in two and each half keeps the wheels mounted on it.

typedef struct lpWheelDef
{
	b3Pos mount;	   // world, at creation: the top of the suspension, within 0.25 m of a piece of the chassis
	float radius;	   // of the tyre
	float width;
	float restLength;  // suspension length (mount to hub) at which the spring carries nothing
	float maxLength;   // at full droop
	float stiffness;   // N/m; 0: the chassis's share of weight on this wheel bounces at 1.4 Hz
	float damping;	   // N*s/m; 0: half of critical
	float grip;		   // tyre friction, times the ground's
	float driveShare;  // of the vehicle's drive force (0: not driven)
	float brakeShare;  // of its brake force
	float steerFactor; // of its steering angle (0: fixed; negative steers the other way)
	bool handbrake;	   // locks under the handbrake, and slides sideways more easily then
	uint8_t driveNeeds; // supply channels its drive needs at its mount (0: none): it drives as well as the worst is fed
	uint8_t steerNeeds; // likewise its steering: unfed, it holds where it is
	float maxForce;	   // N the mount carries before it strains; 0: no limit
	float strength;	   // blast damage it takes, like a link's (J/m^2); 0: immune
	uint8_t material;  // of the wheel once it comes off
	uint32_t color;
} lpWheelDef;

lpWheelDef lpDefaultWheelDef( void );

typedef struct lpVehicleDef
{
	int body;			  // the chassis at creation: a dynamic body
	b3Vec3 forward;		  // world, at creation
	b3Vec3 up;			  // likewise; the suspension casts along -up
	const lpWheelDef* wheels;
	int wheelCount;		  // at most 16
	float maxDriveForce;  // N at full throttle, shared by the driven wheels
	float maxSpeed;		  // m/s the drive pushes toward
	float maxBrakeForce;  // N at full brake, shared by the braking wheels
	float maxSteer;		  // radians at full lock
	float steerSpeed;	  // radians per second
	float rollFactor;	  // how much the tyres' side forces roll the body: 1 fully, 0 not at all (arcade)
	float rollingResistance; // share of a wheel's load that slows it when it is neither driven nor braked
} lpVehicleDef;

lpVehicleDef lpDefaultVehicleDef( void );

// Returns the vehicle index, or -1: not a dynamic body, no wheels or more than 16, a mount more than 0.25 m from the
// chassis, or maxLinks reached.
int lpCreateVehicle( lpWorld* world, const lpVehicleDef* def );

typedef struct lpVehicleControl
{
	float throttle; // -1 (reverse) to 1
	float brake;	// 0 to 1
	float steer;	// -1 (left) to 1 (right)
	bool handbrake;
} lpVehicleControl;

// Persists until changed and is applied from the next step. It is simulation state (hashed): record it like any
// input, at the tick it changed.
void lpWorld_SetVehicleControl( lpWorld* world, int vehicle, const lpVehicleControl* control );

typedef struct lpVehicleState
{
	bool alive;
	int body;		// the body holding most of its attached wheels (-1: none left)
	int wheelCount; // as created
	int attached;	// wheels still on
	int grounded;
	int driven;		// attached wheels with a share of the drive
	int steerable;	// attached wheels that steer
	float power;	// share of its drive force it can deliver
	float speed;	// m/s along its forward direction, of `body`
	b3Pos position; // of `body`'s centre of mass
	b3Vec3 forward; // world directions of `body`
	b3Vec3 up;
	lpVehicleControl control;
} lpVehicleState;

lpVehicleState lpWorld_GetVehicleState( const lpWorld* world, int vehicle );
int lpWorld_GetVehicleCapacity( const lpWorld* world ); // every vehicle index is below this
// The link of its i-th wheel as created, or -1 once that wheel came off
int lpWorld_GetVehicleWheel( const lpWorld* world, int vehicle, int i );

typedef struct lpWheelState
{
	bool alive;
	bool grounded;
	int vehicle;
	int body;			  // the chassis body it is mounted on
	b3WorldTransform hub; // the tyre: its axle is the transform's x, turned by steering and spin
	float radius, width;
	float length;		  // of the suspension, mount to hub
	float load;			  // N on the ground
	float slip;			  // sideways sliding speed at the contact, m/s
	int groundPiece;	  // -1 in the air (or on ground that is not a piece)
	b3Pos contactPoint;
} lpWheelState;

// Cached at the last step the chassis was awake: safe at any time
lpWheelState lpWorld_GetWheelState( const lpWorld* world, int link );

// ---- rigs ----
//
// A rig walks a body on limbs (a mech's legs, a creature's). It makes no physics of its own: the limbs are chains of
// motorised hinges (lpCreateLink with a motor) from the body outward, and the rig only steers their servos. Its
// kinematics come from the links' frames and measured angles, so it follows its pieces through splits and fractures
// like the links do. Each limb's capability is recomputed every step: which of its joints are still on, how strong
// the weakest is (damage, supply), and where its foot is (after a break, the far end of the last segment left: a stump
// walks as a peg). A rig never names a body: the torso is the body holding most of its limbs' first links.

#define LP_MAX_RIG_LIMBS 8
#define LP_MAX_LIMB_JOINTS 3

typedef struct lpLimbDef
{
	int links[LP_MAX_LIMB_JOINTS]; // motorised hinges from the body outward, each on the body the one before it turns
	int linkCount;				   // 1 to LP_MAX_LIMB_JOINTS
	b3Pos foot;					   // world, at creation: the point it stands on, on the last link's outer body
} lpLimbDef;

typedef struct lpRigDef
{
	int body;		// the torso at creation: every limb's first link has an end on it
	b3Vec3 forward; // world, at creation
	b3Vec3 up;
	const lpLimbDef* limbs;
	int limbCount;		// 1 to LP_MAX_RIG_LIMBS, in order around the body: each limb's neighbours are the ones next to it
	float standHeight;	// of the torso's frame above its feet at full strength; 0: as created
	float crouchDepth;	// share of standHeight a full crouch lowers it by
	float stepHeight;	// a swinging foot's lift, m
	float maxSpeed;		// m/s
	float maxTurn;		// rad/s
	float swingTime;	// s a step takes
	float margin;		// m the centre of mass stays inside the planted feet when a leg lifts
} lpRigDef;

lpRigDef lpDefaultRigDef( void );

// Returns the rig index, or -1: not a dynamic body, no limbs or too many, a limb's links not motorised hinges or not
// chained from the body outward.
int lpCreateRig( lpWorld* world, const lpRigDef* def );

typedef struct lpRigControl
{
	float forward; // -1 to 1 of maxSpeed
	float strafe;  // -1 (left) to 1
	float turn;	   // -1 (left) to 1 of maxTurn
	float crouch;  // 0 to 1 of crouchDepth
} lpRigControl;

// Persists until changed and is applied from the next step. It is simulation state (hashed): record it like any input,
// at the tick it changed.
void lpWorld_SetRigControl( lpWorld* world, int rig, const lpRigControl* control );

typedef struct lpRigState
{
	bool alive;
	int body;		// the torso (-1: no limb left on anything)
	int limbCount;	// as created
	int attached;	// limbs whose first link is still on the torso
	int able;		// of those, the ones that can stand and step
	int planted;	// feet on the ground
	bool idle;		// standing still with its servos' targets frozen (it can sleep)
	float height;	// of the torso's frame above its planted feet
	float speed;	// m/s along its forward direction
	b3Pos position; // of the torso's frame
	b3Vec3 forward; // world directions of the torso
	b3Vec3 up;
	lpRigControl control;
} lpRigState;

lpRigState lpWorld_GetRigState( const lpWorld* world, int rig );
int lpWorld_GetRigCapacity( const lpWorld* world ); // every rig index is below this

typedef struct lpLimbState
{
	bool attached;	// its first link is still on the torso
	bool able;		// it can lift its foot and carry its share
	bool planted;
	int joints;		// links still on in a chain from the torso (0 to linkCount)
	float strength; // of its weakest joint's servo: damage and supply, 0 to 1
	float reach;	// from its first joint to its foot, m
	b3Pos foot;		// world
	int footBody;	// the body its foot is on (-1: detached)
} lpLimbState;

lpLimbState lpWorld_GetLimbState( const lpWorld* world, int rig, int limb );

// ---- impacts ----

typedef struct lpImpactDef
{
	b3Pos point;
	b3Vec3 direction; // unit; pushes loose pieces this way (zero for radial)
	float radius;
	float energy;  // joules
	float impulse; // directional: N*s given to loose pieces (capped at 12 m/s); explosion: outward speed at the center, m/s
	bool explosion;
} lpImpactDef;

// Queued; applied at the start of the next step, in call order.
void lpWorld_AddImpact( lpWorld* world, const lpImpactDef* impact );

// Leaf blower: wakes rubble, scrap and ghosts in a cone and pushes them along the direction (light things strongly,
// heavy things barely). Call every tick while blowing; applied at the next step.
void lpWorld_Blow( lpWorld* world, b3Pos origin, b3Vec3 direction, float range, float halfAngleRadians, float speed );

// Make a body full physics again (a thrown or launched piece). Ghost and scrap bodies get a Box3D body back.
void lpWorld_PromoteBody( lpWorld* world, int body );

// Change how strongly gravity pulls a body and whatever later breaks off it ("fairy dust" on a load to carry). On a
// structure it changes only the weight its stress solve carries.
void lpWorld_SetGravityScale( lpWorld* world, int body, float scale );

// Pull a piece toward a target like a spring (grab tool, winch). Call every tick while pulling; applied at the next
// step. localPoint is in the piece's body frame (lpRayHit gives world points: convert with lpWorld_ToBodyFrame).
// Structures cannot be pulled; frozen rubble wakes up. The pull accelerates at most maxAccel and treats bodies
// heavier than maxMass as maxMass (so a crane can lift a beam but not a house).
void lpWorld_Pull( lpWorld* world, int piece, b3Vec3 localPoint, b3Pos target, float maxAccel, float maxMass );
b3Vec3 lpWorld_ToBodyFrame( const lpWorld* world, int piece, b3Pos worldPoint );
b3Pos lpWorld_ToWorldFrame( const lpWorld* world, int piece, b3Vec3 localPoint );

void lpWorld_Step( lpWorld* world, float timeStep, int subStepCount );

// Solve every structure waiting for a stress check to convergence now, with no per-step budget (up to
// maxSettleIterations each), and judge them as a step would. For loading: a new structure starts settled instead of
// spending its first steps solving. lpBuildScene calls it. Returns the iterations spent (also in lpStats).
int lpWorld_SettleStructures( lpWorld* world );

// Hash of the full simulation state (bodies, pieces, bonds). Equal hashes after the same inputs prove determinism.
uint64_t lpWorld_Hash( const lpWorld* world );

// Hash of the stress solver's state (solutions, loads, utilizations, strains, solves in progress). A change to the
// solver that is meant to change nothing keeps it equal, not only lpWorld_Hash.
uint64_t lpWorld_HashStress( const lpWorld* world );

typedef struct lpStats
{
	int pieceCount;
	int bondCount;
	int linkCount;
	int linkBreaks;	  // this step, by load, damage or a lost end
	int linkRebuilds; // this step: links whose joint moved to a new body
	int structureBodies;
	int debrisBodies; // full + light
	int awakeDebris;
	int rubbleBodies;
	int fullDebris;
	int lightDebris;
	int ghostBodies;
	int scrapBodies;
	int demotionsThisStep;
	int deferredJobs;
	int ghostCasts;
	int impactsThisStep;
	int fracturesThisStep;
	int cellsThisStep;
	int splitsThisStep;
	int clipFailures;
	float fractureMs;
	float physicsMs;
	float updateMs;

	// Breakdown of fractureMs for this step
	float cellMs;  // the parallel phase: Voronoi cells, merges and Box3D hulls of the new pieces
	float hullMs;  // Box3D hulls built later (a piece that had none, such as a ghost getting physics back)
	float shapeMs; // Box3D shape create/destroy
	float bondMs;  // contact areas for new bonds
	float splitMs; // connectivity and new bodies

	// CPU time inside the parallel phase, summed over its jobs (so it can exceed cellMs with several workers)
	float voronoiCpuMs;
	float mergeCpuMs;
	float hullCpuMs;

	// Stress solve (stress.c)
	float stressMs;
	int stressIterations;
	int stressBreaks;
	int stressSolves;		 // structures solved this step (in parallel)
	int stressJudged;		 // of those, the ones that converged and were judged
	int stressReduced;		 // of those, the ones solved on a reduced system (rigid clusters)
	int stressDissolved;	 // clusters dissolved because holding them rigid leaked too much load (the residual meter)
	int clusteredPieces;	 // pieces in rigid clusters, over all structures
	int stressAudits;		 // exact solves this step of structures judged provisionally before
	int provisionalStructures; // judged on a reduced system (rigid clusters), waiting for an exact audit
	int auditBacklog;		   // of those, queued
	int stressWaiting;		 // structures that found this step's stress budget spent; they go first next step
	int unsettledStructures; // structures still solving or creaking toward a break
	float settleMs;			 // the last lpWorld_SettleStructures (steps leave these alone)
	int settleIterations;

	// Vehicles (wheel.c) and supply (supply.c)
	float vehicleMs;
	int wheelCasts;	   // this step
	int supplyUpdates; // this step: 1 when a carrier's connections changed
	int motorSets;	   // this step: Box3D motor setter calls (only when a servo's speed or cap changed)

	// Rigs (rig.c)
	float rigMs;
	int footCasts; // this step
} lpStats;

lpStats lpWorld_GetStats( const lpWorld* world );

// ---- queries ----

typedef struct lpRayHit
{
	b3Pos point;
	b3Vec3 normal;
	int piece;
	int body;
	int link; // a rope was hit first (piece and body are -1 then); -1 otherwise
	bool hit;
} lpRayHit;

lpRayHit lpWorld_CastRay( const lpWorld* world, b3Pos origin, b3Vec3 translation );

// ---- rendering access ----

// Vertex of a piece's flat-shaded render mesh, in the body frame.
typedef struct lpVertex
{
	float position[3];
	int8_t normal[4];	// snorm8 face normal, w unused
	uint32_t color;		// 0xAABBGGRR (little-endian RGBA8)
} lpVertex;

typedef struct lpPieceInfo
{
	int body;			 // -1 when the slot is free
	uint32_t generation; // changes when the slot is reused; piece geometry never changes otherwise
	uint32_t userId;	 // of the object it came from
	int part;			 // index of the part it came from in that object
	uint16_t tag;		 // that part's system tag
	float volume;
	uint8_t supplied;	 // supply channels fed here
} lpPieceInfo;

int lpWorld_GetPieceCapacity( const lpWorld* world );
lpPieceInfo lpWorld_GetPieceInfo( const lpWorld* world, int piece );
// How well a supply channel is fed at a piece, 0 to 1: the sum of the shares of the sources its carriers reach
float lpWorld_GetPieceSupply( const lpWorld* world, int piece, int channel );

int lpWorld_GetBodyCapacity( const lpWorld* world );
bool lpWorld_GetBodyTransform( const lpWorld* world, int body, b3WorldTransform* transform );

// Triangles (3 vertices each, no index buffer). Returns the vertex count, or -1 if capacity is too small.
int lpWorld_BuildPieceMesh( const lpWorld* world, int piece, lpVertex* vertices, int capacity );
int lpWorld_GetMaxPieceVertices( void );

// Cosmetic particles emitted during the last step (dust, tiny chips, despawned debris).
typedef struct lpParticle
{
	float position[3];
	float velocity[3];
	float size;
	uint32_t color; // 0xAABBGGRR
	int kind;		// lpParticleKind
} lpParticle;

const lpParticle* lpWorld_GetParticles( const lpWorld* world, int* count );

#ifdef __cplusplus
}
#endif
