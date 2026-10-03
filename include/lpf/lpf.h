// SPDX-License-Identifier: MIT
// lpf: low-poly polygonal destruction.
//
// Objects are compounds of convex pieces. Impacts refracture the struck pieces into convex cells, damage the
// bonds between pieces, and pieces cut off from their anchors become rigid debris bodies. Settled debris
// freezes into static rubble. Everything is deterministic: the same calls in the same order give
// bit-identical results (see docs/determinism-rules.md).

#pragma once

#include "lpf/lpmath.h"

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
	lp_armor,	   // welded plate: a mech's hull, a tank's; a grenade or a cannon only chips it
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

	uint8_t joint;	   // lpJointId between its parts and others when a part asks for lp_jointAuto (auto: solid)
	uint8_t cellJoint; // lpJointId between the cells of its course grid (lp_breakMasonry; auto: mortar)
} lpMaterialDef;

// The built-in materials (lp_materialCount of them, in lpMaterialId order): copy, tune, and give the copy to
// lpWorldDef.materials. Every world holds its own table.
const lpMaterialDef* lpDefaultMaterials( void );

// How two parts are joined. A bond between parts takes the weaker joint of the two; a bond between fracture cells of
// one part is solid. Structures break at their joints first: mortar before brick, nails before timber.
typedef enum lpJointId
{
	lp_jointAuto,	// by material (lpMaterialDef.joint): mortar for stone, brick, concrete and plaster, nails for wood,
					// bolts for sheet metal, solid otherwise
	lp_jointSolid,	// as strong as the weaker of the two materials
	lp_jointMortar, // weak in tension: masonry cracks and hinges at its joints
	lp_jointDry,	// stacked with nothing between: no tension or cohesion, friction only
	lp_jointNails,
	lp_jointBolts,	// sheet metal panels on a car
	lp_jointMounts, // an engine on its mounts: tears off in a hard crash
	lp_jointWeld,	// a welded seam of thin shell (a mech's leg): a fifth of solid steel; a blast cracks it
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

// The built-in joints (lp_jointCount of them, in lpJointId order; the auto row is a placeholder, and solid's strengths
// are unused: a solid bond holds what the weaker material does)
const lpJointDef* lpDefaultJoints( void );

// ---- world ----

typedef struct lpWorldDef
{
	lpVec3 gravity; // m/s^2 (the physics world is the lpf world's own)
	uint64_t seed;
	// The materials and joints, copied when the world is made (lp_materialCount and lp_jointCount entries, in id
	// order; NULL: the built-ins). A material's auto joints become its defaults. Names are borrowed.
	const lpMaterialDef* materials;
	const lpJointDef* joints;
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
	// A hit becomes an impact when its energy (less what crumpling soaks up) reaches hitEnergy (J); its radius is
	// hitRadiusScale x cbrt(energy), within [hitRadiusMin, hitRadiusMax] m. How hard the world hits is the game's.
	float hitEnergy;
	float hitRadiusScale;
	float hitRadiusMin;
	float hitRadiusMax;
	float pushSpeedCap; // m/s: the most a directional impact (not a blast) pushes a loose body
	// Freezing: debris the physics engine puts to sleep becomes static rubble once this many steps old (full, light);
	// light debris still moving long after it was made is frozen once slower than freezeDriftSpeed (m/s), from
	// freezeDriftAge steps
	int freezeAgeFull;
	int freezeAgeLight;
	int freezeDriftAge;
	float freezeDriftSpeed;
	float killDepth;	   // bodies falling below this height are removed
	// The pull's spring (lpWorld_Pull, the grab tool): its acceleration is stiffness x the error less damping x the
	// point's velocity, and the held body's spin is multiplied by spinKeep every step
	float pullStiffness; // 1/s^2
	float pullDamping;	 // 1/s
	float pullSpinKeep;
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
	int maxFootCastsPerStep;  // rig foothold casts per step; a foot not cast lands where it was planned
} lpWorldDef;

lpWorldDef lpDefaultWorldDef( void );

typedef struct lpWorld lpWorld;

// NULL, with a message on stderr, if the def's materials or joints are out of range
lpWorld* lpCreateWorld( const lpWorldDef* def );
void lpDestroyWorld( lpWorld* world );

// The world's own materials and joints (its copies of lpWorldDef.materials and .joints)
const lpMaterialDef* lpWorld_GetMaterial( const lpWorld* world, int material );
const lpJointDef* lpWorld_GetJoint( const lpWorld* world, int joint );

// ---- objects ----

// Makes a part or an object explode: when it hits something at triggerSpeed or faster, or when a nearby blast reaches
// it. A thrown alchemical flask, a gas can, volatile cargo, a car's fuel tank. Zero radius means inert.
typedef struct lpDetonatorDef
{
	float triggerSpeed; // m/s approach speed (at least lpWorldDef.hitSpeed, where collisions start to register)
	float radius;
	float energy;
	float speed; // blast push, m/s at the center
	float delay; // s from what sets it off to the blast (0: at once): a fuse, burning wherever its pieces go
} lpDetonatorDef;

// What a part is to the systems a machine or a creature runs on. There are 8 channels (bit n is channel n); the game
// names them (fuel, power, steering; blood, nerves). A bond between two parts that carry a channel carries it, and so
// does a link that carries it; a part supplied with a channel is one its sources reach through carriers. Fracture
// cells inherit all of it.
//
// A source can hold a pool (hydraulic fluid, blood, fuel): cut its lines and it leaks. When the carriers its lowest
// channel reaches lose volume (a severed leg, chips shot out of a line), a leak opens in proportion, draining that share
// of the pool every second and closing over `seal` seconds (valves, clotting). The source feeds fully while its pool
// holds 30% or more, then less, down to nothing when it is empty.
typedef struct lpPartSystem
{
	uint16_t tag;	 // the game's name for the part (an engine, a fuel tank, a heart); 0: none. The core never reads it.
	uint8_t carries; // channels it carries
	uint8_t sources; // channels it feeds (it carries them too)
	uint8_t needs;	 // channels it must be fed before it feeds its own; only those below its lowest source count
	float pool;		 // a source's pool (any unit; 0: none, it never runs dry)
	float seal;		 // s a leak takes to close (0: it bleeds until the pool is empty)
	float leakRate;	 // share of the pool a leak drains per second for each share of its reach lost (0: 1)
	float pressure;	 // share of its capacity a pool feeds fully down to, then less, to nothing empty (0: 0.3)
} lpPartSystem;

// One convex part of an object, in object space. A box when pointCount is zero, else the hull of points.
typedef struct lpPartDef
{
	lpVec3 halfExtents;
	lpTransform transform;
	const lpVec3* points;
	int pointCount;
	uint8_t material;
	uint32_t color;	  // exterior 0xRRGGBB
	lpVec3 grainAxis; // object space; zero picks the longest box axis
	bool anchored;	  // rests on a foundation: bonded to the world through its bottom face
	uint8_t joint;	  // lpJointId where this part meets its neighbours (lp_jointAuto: by material)
	lpPartSystem system;
	lpDetonatorDef detonator; // this part goes off alone, and takes only its own pieces with it (a fuel tank)
} lpPartDef;

typedef struct lpObjectDef
{
	lpWorldTransform transform;
	bool isStatic; // structures stay static until pieces break loose
	const lpPartDef* parts;
	int partCount;
	lpVec3 linearVelocity;
	lpVec3 angularVelocity;
	lpDetonatorDef detonator; // the parts without one of their own share it: they go off together, as one
	float gravityScale;		  // "fairy dust": 1 is normal weight, 0 floats; everything that breaks off keeps it
	uint32_t userId;		  // the game's id for the object, kept by every piece made from it
	// A moving object whose joints carry what accelerates it (a car in a crash, a mech landing): its stress is solved
	// on hard hits and landings, against its loads less its own acceleration ("inertia relief"). What breaks off
	// keeps it.
	bool solveStress;
	// m: rotational inertia added about every axis, as mass * r^2. Box3D holds a joint as stiffly as the lighter body's
	// inertia allows, so a slender limb on joints (a mech's leg) gives a few degrees under load; padded, it holds. What
	// breaks off in large parts keeps it.
	float inertiaRadius;
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
// A motor that jams (dented armor grinding on a knee): as the link is damaged (by blasts, and by the pieces at its ends
// breaking up) it turns slower, by jam times the damage, and sticks, holding with jam times the damage of its
// maxTorque even unfed.
typedef struct lpMotorDef
{
	float maxTorque;  // N*m; 0 (and no holdTorque) = no motor
	float maxSpeed;	  // rad/s
	float gain;		  // 1/s: turning speed per radian from its target
	uint8_t needs;	  // supply channels it needs fed at either end (0: none)
	float holdTorque; // N*m it brakes with when unfed
	float jam;		  // 0 to 1: how much damage jams it (0: never)
	float jamKnock;	  // share of its strength a jamming motor loses when a piece at an end breaks up (0: 0.25)
} lpMotorDef;

typedef struct lpLinkDef
{
	int type;					  // lpLinkType
	int bodyA;					  // body index at creation (the piece nearest the anchor holds it); -1 = the world
	int bodyB;					  // likewise; A != B
	lpPos anchorA;				  // world: the joint point, or the rope's end on A
	lpPos anchorB;				  // world: the rope's end on B (ropes only)
	lpVec3 axis;				  // hinge axis, ball cone axis (world, unit)
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
void lpWorld_SetLinkTargetRotation( lpWorld* world, int link, lpQuat rotation );

typedef struct lpLinkState
{
	bool alive;
	bool slack; // a rope whose ends are closer than its length
	int type;
	uint32_t generation;  // bumped each time the link slot is reused
	int bodyA, bodyB;	  // current bodies of the two ends (-1: the world)
	lpPos pointA, pointB; // world points of the two ends
	lpVec3 force;		  // on B, world, N, at the last step either end was awake
	lpVec3 torque;
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
	lpPos mount;	   // world, at creation: the top of the suspension, within 0.25 m of a piece of the chassis
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
	float slidingGrip;	 // of its grip while it slides (0: 0.8)
	float handbrakeGrip; // of its sideways grip while the handbrake locks it (0: 0.5)
	float tearRatio;	 // it tears off a mount body lighter than this share of its share of the chassis (0: 0.2)
} lpWheelDef;

lpWheelDef lpDefaultWheelDef( void );

typedef struct lpVehicleDef
{
	int body;			  // the chassis at creation: a dynamic body
	lpVec3 forward;		  // world, at creation
	lpVec3 up;			  // likewise; the suspension casts along -up
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
	uint32_t bodyGeneration;
	int controller;	// the peer whose commands drive it (-1: the scene's drivers)
	int wheelCount; // as created
	int attached;	// wheels still on
	int grounded;
	int driven;		// attached wheels with a share of the drive
	int steerable;	// attached wheels that steer
	float power;	// share of its drive force it can deliver
	float speed;	// m/s along its forward direction, of `body`
	lpPos position; // of `body`'s centre of mass
	lpVec3 forward; // world directions of `body`
	lpVec3 up;
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
	lpWorldTransform hub; // the tyre: its axle is the transform's x, turned by steering and spin
	float radius, width;
	float length;		  // of the suspension, mount to hub
	float load;			  // N on the ground
	float slip;			  // sideways sliding speed at the contact, m/s
	int groundPiece;	  // -1 in the air (or on ground that is not a piece)
	uint32_t groundGeneration;
	lpPos contactPoint;
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
	lpPos foot;					   // world, at creation: the point it stands on, on the last link's outer body
} lpLimbDef;

// The built-in walker (gait.c): a free gait for statically stable many-legged bodies. Its tuning, with the defaults of
// lpDefaultRigDef; a game that walks its rigs itself sets lpRigDef.walker to lp_walkerNone instead.
typedef struct lpGaitDef
{
	// The body and its pace
	float crouchDepth; // share of the stand height a full crouch lowers it by
	float bellyHeight; // of the torso's frame resting on its belly (crawling); 0: a fifth of the stand height
	float stepHeight;  // a swinging foot's lift, m
	float stride;	   // m a planted foot drifts from where it rests before it steps (half a stride's length)
	float maxSpeed;	   // m/s
	float maxTurn;	   // rad/s
	float swingTime;   // s a step takes
	float margin;	   // m the centre of mass stays inside the planted feet when a leg lifts
	float keepUp;	   // share of the cadence's top speed the controls may ask for
	float legPace[LP_MAX_RIG_LIMBS + 1]; // share of its top speed, by able limbs: fewer swing in more turns
	float crawlSpeed;  // share of its top speed it drags itself at on its belly
	int walkingLegs;   // fewer able limbs than this cannot lift one and stay up: it crawls
	// When a foot steps, and where it lands
	float due;		 // moving, a planted foot steps once it has drifted this share of its half-step behind its rest point
	float tidy;		 // standing still, feet further than this share of the stride step back to rest, one at a time
	float land;		 // a swing lands at most this share of the stride ahead of its rest point
	float overreach; // past its stride by this share, a planted foot stops the torso
	float minStance; // s a foot set down stays down
	float lateCast;	 // share of a swing at which the ground under its landing is cast again
	// The foothold cast: a sole-sized sphere down from above the planned foothold
	float clearance; // m above the foothold it starts
	float depth;	 // m below it it looks
	float sole;		 // m: the sphere's radius
	float missDrop;	 // m below the foothold a foot is aimed when the cast finds nothing
	// The torso's desired pose
	float shift;	// m/s the body leans toward the feet that hold it while a foot waits for balance
	float lead;		// m the pose may run ahead of the torso (the servos' gains pull it along)
	float lag;		// m it may fall behind or away sideways: a torso that runs on is held back
	float leadTurn; // rad its heading may run ahead
	float climb;	// m/s its height moves at
	float falling;	// m/s: a torso sinking faster than this is falling, and the pose follows it down
	float weakSag;	// share of its height the torso drops as its weakest planted leg's strength goes to 0
	float reachSpare; // m a leg keeps in hand below its deepest reach
	float tuck;		  // share of the stand height below the torso an unable limb's foot is held at
	// Planted feet
	float arrived;	  // m: a planted foot this near where it was set down has got there
	float arriveTime; // s it may take
	float slipped;	  // m: a held foot this far from its hold slipped or was knocked
	float easeTime;	  // s a held foot's hold takes to ease toward where the legs' geometry has it
	// Standing still, and stuck
	int calmTicks;	   // steps settled and still before its targets freeze (it can sleep)
	float calmHeight;  // m off its height that still counts as settled
	float calmTilt;	   // rad of tilt that does
	float knockHeight; // idle, knocked off its height by this many calm heights, or...
	float knockTilt;   // ...tilted by this many calm tilts, it wakes
	float stallPace;   // told to move, making less than this share of the speed asked...
	int stallTicks;	   // ...for this long, it is stuck: it crawls
	// Strikes
	float strike; // 1/s: a reaching limb's joints go at full speed until this near their target (8: 4 rad/s at 0.5 rad)
} lpGaitDef;

typedef enum lpWalkerKind
{
	lp_walkerGait, // the built-in walker (lpRigDef.gait)
	lp_walkerNone, // the game walks it: each limb's foot target and the torso's pose (lpWorld_SetFootTarget, SetRigPose)
} lpWalkerKind;

typedef struct lpRigDef
{
	int body;		// the torso at creation: every limb's first link has an end on it
	lpVec3 forward; // world, at creation
	lpVec3 up;
	const lpLimbDef* limbs;
	int limbCount;	   // 1 to LP_MAX_RIG_LIMBS, in order around the body: each limb's neighbours are the ones next to it
	float standHeight; // of the torso's frame above its feet at full strength; 0: as created
	int walker;		   // lpWalkerKind
	lpGaitDef gait;	   // the built-in walker's tuning
} lpRigDef;

lpRigDef lpDefaultRigDef( void );

// Returns the rig index, or -1: not a dynamic body, no limbs or too many, a limb's links not motorised hinges or not
// chained from the body outward.
int lpCreateRig( lpWorld* world, const lpRigDef* def );

typedef struct lpRigControl
{
	float forward; // -1 to 1 of the gait's maxSpeed (the built-in walker's controls)
	float strafe;  // -1 (left) to 1
	float turn;	   // -1 (left) to 1 of maxTurn
	float crouch;  // 0 to 1 of crouchDepth
} lpRigControl;

// Persists until changed and is applied from the next step. It is simulation state (hashed): record it like any input,
// at the tick it changed.
void lpWorld_SetRigControl( lpWorld* world, int rig, const lpRigControl* control );

// A rig the game walks itself (lp_walkerNone): what it asks of one limb. Each active limb's joints are driven toward the
// IK of its foot at `point` from the rig's pose, fed the joint speeds that move the foot at `velocity` against the pose's
// own motion. Gait, balance and footholds are the game's (the built-in walker in gait.c is one way to do them).
// Persists until changed, and is simulation state (hashed): record it like any input.
typedef struct lpFootTarget
{
	bool active;	 // false: its servos keep their last targets
	lpPos point;	 // world
	lpVec3 velocity; // world, m/s
} lpFootTarget;

void lpWorld_SetFootTarget( lpWorld* world, int rig, int limb, const lpFootTarget* target );

// lp_walkerNone: the torso's pose the feet are solved from, and its motion (fed forward). Until set, the pose follows the
// torso as it is. Persists until changed (hashed).
void lpWorld_SetRigPose( lpWorld* world, int rig, lpWorldTransform pose, lpVec3 linear, lpVec3 angular );

typedef struct lpRigState
{
	bool alive;
	int body;		// the torso (-1: no limb left on anything)
	uint32_t bodyGeneration;
	int controller;	// the peer whose commands drive it (-1: the scene's drivers)
	int limbCount;	// as created
	int attached;	// limbs whose first link is still on the torso
	int able;		// of those, the ones that can stand and step
	int planted;	// feet on the ground
	bool idle;		// standing still with its servos' targets frozen (it can sleep)
	bool crawling;	// too few able limbs to walk: on its belly, dragging itself a foot at a time
	float height;	// of the torso's frame above its planted feet
	float speed;	// m/s along its forward direction
	lpPos position; // of the torso's frame
	lpVec3 forward; // world directions of the torso
	lpVec3 up;
	lpRigControl control;
} lpRigState;

lpRigState lpWorld_GetRigState( const lpWorld* world, int rig );
int lpWorld_GetRigCapacity( const lpWorld* world ); // every rig index is below this

typedef struct lpLimbState
{
	bool attached;	// its first link is still on the torso
	bool able;		// it can lift its foot and carry its share
	bool planted;
	bool swinging;
	int joints;		// links still on in a chain from the torso (0 to linkCount)
	float strength; // of its weakest joint's servo: damage and supply, 0 to 1
	float reach;	// from its first joint to its foot, m
	float depth;	// how far below the torso's frame the foot reaches at its rest point (a stump reaches less), m
	lpPos foot;		// world
	int footBody;	// the body its foot is on (-1: detached)
	bool reaching;	// out of the gait, reaching for its target
	int touching;	// reaching: the piece its foot touches (-1: none), for a grab
	uint32_t footBodyGeneration, touchingGeneration;
	int grip;				 // the link its claw holds (-1: none)
	uint32_t gripGeneration;
} lpLimbState;

lpLimbState lpWorld_GetLimbState( const lpWorld* world, int rig, int limb );

// A limb reaching for a world point (a strike, a stomp, a grab): it leaves the gait once the others keep the centre of
// mass over their feet by the margin (until then the body leans toward them, and the limb's state says it is not
// reaching yet), and IK drives its foot at the point within what is left of its chain and its servos' caps: a weak limb
// swings slower and hits softer. Persistent and hashed, like the controls; turned off, the limb steps back into the gait.
void lpWorld_SetLimbTarget( lpWorld* world, int rig, int limb, bool active, lpPos point );

// ---- impacts ----

typedef struct lpImpactDef
{
	lpPos point;
	lpVec3 direction; // unit; pushes loose pieces this way (zero for radial)
	float radius;
	float energy;  // joules
	float impulse; // directional: N*s given to loose pieces (capped at 12 m/s); explosion: outward speed at the center, m/s
	bool explosion;
} lpImpactDef;

// Queued; applied at the start of the next step, in call order.
void lpWorld_AddImpact( lpWorld* world, const lpImpactDef* impact );

// Make a body full physics again (a thrown or launched piece). Ghost and scrap bodies get a Box3D body back.
void lpWorld_PromoteBody( lpWorld* world, int body );

// Change how strongly gravity pulls a body and whatever later breaks off it ("fairy dust" on a load to carry). On a
// structure it changes only the weight its stress solve carries.
void lpWorld_SetGravityScale( lpWorld* world, int body, float scale );

// Pull a piece toward a target like a spring (grab tool, winch). Call every tick while pulling; applied at the next
// step. localPoint is in the piece's body frame (lpRayHit gives world points: convert with lpWorld_ToBodyFrame).
// Structures cannot be pulled; frozen rubble wakes up. The pull accelerates at most maxAccel and treats bodies
// heavier than maxMass as maxMass (so a crane can lift a beam but not a house).
void lpWorld_Pull( lpWorld* world, int piece, lpVec3 localPoint, lpPos target, float maxAccel, float maxMass );

// ---- commands ----
//
// Everything from outside the simulation (a player's tools and controls, a claw, a game's spawns) enters as a command
// stamped with the tick it applies at, the peer that sent it and that peer's own sequence number. A tick's commands are
// applied as its step begins, in (peer, sequence) order, whatever order they were submitted in, so every machine of a
// session that submits the same commands computes the same world. Logic that every machine runs from world state alone
// (a scene's drivers) submits as LP_PEER_SCENE, applied after the players; anything else that does not come from world
// state must be a command (determinism rule 10). The immediate calls above stay for building a scene and for tests.
//
// A command names a slot and its generation (LP_ANY_GENERATION: whatever holds the slot when it is applied). One whose
// reference went stale is dropped, and so is a scene's control of a vehicle or rig a player drives: a player's control,
// limb or claw command takes it over until that player releases it.

#define LP_PEER_SCENE 255
#define LP_ANY_GENERATION 0xFFFFFFFFu

typedef enum lpCommandKind
{
	lp_commandImpact,		  // an impact where a ray from origin along def.direction first hits (range > 0), or at def.point
	lp_commandPull,			  // pull a piece toward a target this step (lpWorld_Pull)
	lp_commandSpawn,		  // an object from a template (lpWorld_AddTemplate), placed and moving
	lp_commandVehicleControl, // (lpWorld_SetVehicleControl)
	lp_commandRigControl,	  // (lpWorld_SetRigControl)
	lp_commandLimbTarget,	  // (lpWorld_SetLimbTarget)
	lp_commandFootTarget,	  // (lpWorld_SetFootTarget)
	lp_commandRigPose,		  // (lpWorld_SetRigPose)
	lp_commandRelease,		  // the peer lets go of a vehicle or rig: the scene's drivers take it back
	lp_commandClaw,			  // a reaching limb grabs what it touches (a weld at its foot), lets go of it, or toggles
	lp_commandLinkTarget,	  // (lpWorld_SetLinkTarget: link.value is the angle)
	lp_commandLinkRotation,	  // (lpWorld_SetLinkTargetRotation)
	lp_commandRopeLength,	  // (lpWorld_SetRopeLength: link.value is the length)
	lp_commandCreateLink,	  // (lpCreateLink)
	lp_commandDestroyLink,	  // (lpDestroyLink)
	lp_commandGravityScale,	  // (lpWorld_SetGravityScale)
	lp_commandPromote,		  // (lpWorld_PromoteBody)
	lp_commandKindCount
} lpCommandKind;

typedef enum lpClawMode
{
	lp_clawGrab,
	lp_clawRelease,
	lp_clawToggle,
} lpClawMode;

typedef struct lpCommandImpact
{
	lpImpactDef def;
	lpPos origin;
	float range; // > 0: def.point is where the ray from origin, range along def.direction, first hits (a miss does nothing)
	bool piecesOnly; // a ray that first hits a rope or a wheel does nothing
} lpCommandImpact;

typedef struct lpCommandPull
{
	int piece;
	uint32_t generation;
	lpVec3 localPoint;
	lpPos target;
	float maxAccel, maxMass;
} lpCommandPull;

typedef struct lpCommandSpawn
{
	int templateIndex;
	lpWorldTransform transform;
	lpVec3 linearVelocity, angularVelocity;
} lpCommandSpawn;

typedef struct lpCommandVehicleControl
{
	int vehicle;
	lpVehicleControl control;
} lpCommandVehicleControl;

typedef struct lpCommandRigControl
{
	int rig;
	lpRigControl control;
} lpCommandRigControl;

typedef struct lpCommandLimbTarget
{
	int rig, limb;
	bool active;
	lpPos point;
} lpCommandLimbTarget;

typedef struct lpCommandFootTarget
{
	int rig, limb;
	lpFootTarget target;
} lpCommandFootTarget;

typedef struct lpCommandRigPose
{
	int rig;
	lpWorldTransform pose;
	lpVec3 linear, angular;
} lpCommandRigPose;

typedef struct lpCommandRelease
{
	int vehicle, rig; // one of them, the other -1
} lpCommandRelease;

typedef struct lpCommandClaw
{
	int rig, limb;
	uint8_t mode; // lpClawMode
	float maxForce, maxTorque, strength; // of the grip it makes
} lpCommandClaw;

typedef struct lpCommandLink
{
	int link;
	uint32_t generation;
	float value;
	lpQuat rotation;
} lpCommandLink;

typedef struct lpCommandCreateLink
{
	lpLinkDef def;
	uint32_t generationA, generationB; // of def.bodyA and def.bodyB
} lpCommandCreateLink;

typedef struct lpCommandBody
{
	int body;
	uint32_t generation;
	float scale;
} lpCommandBody;

typedef struct lpCommand
{
	int64_t tick; // the step it applies at: lpWorld_GetTick or later
	uint32_t seq; // the peer's own count: (peer, seq) orders a tick's commands, and is unique in it. The world numbers
				  // LP_PEER_SCENE's commands itself, in the order they are submitted.
	uint8_t peer; // the player who sent it (0 to 254), or LP_PEER_SCENE
	uint8_t kind; // lpCommandKind
	// Filled when applied (lpWorld_GetAppliedCommands)
	bool dropped; // a stale reference, or a scene's control of what a player drives
	int result;	  // the body a spawn made, the link a claw or a link command made (-1: none)
	union
	{
		lpCommandImpact impact;
		lpCommandPull pull;
		lpCommandSpawn spawn;
		lpCommandVehicleControl vehicleControl;
		lpCommandRigControl rigControl;
		lpCommandLimbTarget limbTarget;
		lpCommandFootTarget footTarget;
		lpCommandRigPose rigPose;
		lpCommandRelease release;
		lpCommandClaw claw;
		lpCommandLink link;
		lpCommandCreateLink createLink;
		lpCommandBody body;
	};
} lpCommand;

// Queues a command. False, and nothing queued, if its tick has passed or that (tick, peer, seq) is queued already.
bool lpWorld_Submit( lpWorld* world, const lpCommand* command );

// The commands the last step applied, in the order it applied them, dropped ones included: for recordings
const lpCommand* lpWorld_GetAppliedCommands( const lpWorld* world, int* count );

// An object to spawn by command (lp_commandSpawn), copied whole; returns its index. Its transform and velocities are
// the command's. Register templates in the same order on every machine of a session.
int lpWorld_AddTemplate( lpWorld* world, const lpObjectDef* def );
lpVec3 lpWorld_ToBodyFrame( const lpWorld* world, int piece, lpPos worldPoint );
lpPos lpWorld_ToWorldFrame( const lpWorld* world, int piece, lpVec3 localPoint );

void lpWorld_Step( lpWorld* world, float timeStep, int subStepCount );

// Solve every structure waiting for a stress check to convergence now, with no per-step budget (up to
// maxSettleIterations each), and judge them as a step would. For loading: a new structure starts settled instead of
// spending its first steps solving. lpBuildScene calls it. Returns the iterations spent (also in lpStats).
int lpWorld_SettleStructures( lpWorld* world );

// The state hash covers every element of the simulation, in categories. Each element is hashed on its own, seeded with
// its slot and generation; a category is the sum of its elements; the hash mixes the categories.
typedef enum lpHashCategory
{
	lp_hashWorld,	   // the world's counters and the queues one step leaves the next
	lp_hashBodies,	   // a body: its kind, motion and pieces (in order)
	lp_hashPieces,	   // a piece: its geometry's digest, ids, supply, links, and the bonds it holds (with its higher neighbours)
	lp_hashStress,	   // a structure's (or a moving body's) stress solve: solutions, loads, utilizations, solves in progress
	lp_hashBackend,	   // the physics engine's own state of a body: sleep, contact warm starts and caches
	lp_hashLinks,	   // a link, its motor and its loads
	lp_hashVehicles,   // a vehicle
	lp_hashWheels,	   // a wheel
	lp_hashRigs,	   // a rig and its limbs
	lp_hashPools,	   // a pool
	lp_hashDetonators, // a detonator and its fuse
	lp_hashCategoryCount
} lpHashCategory;

// Hash of the full simulation state. Equal hashes after the same inputs prove determinism.
uint64_t lpWorld_Hash( const lpWorld* world );

// Hash of the stress solver's state (lp_hashStress). A change to the solver that is meant to change nothing keeps it
// equal, not only lpWorld_Hash.
uint64_t lpWorld_HashStress( const lpWorld* world );

// Following a mismatch down. Two machines whose hashes differ compare their categories' sums, then the buckets of
// the category that differs (bucket k sums its elements in slots 64k to 64k+63), then that bucket's elements, to name
// the slot. The hash is kept up to date incrementally: these and lpWorld_Hash rehash only what changed since.
void lpWorld_HashCategories( const lpWorld* world, uint64_t sums[lp_hashCategoryCount] );
int lpWorld_HashSlotCount( const lpWorld* world, int category ); // the world category has one slot
uint64_t lpWorld_HashBucket( const lpWorld* world, int category, int bucket );
uint64_t lpWorld_HashElement( const lpWorld* world, int category, int slot ); // 0: nothing in that slot
// The incremental hash against a full recompute (slow: for tests and lpf_bench --check-hash). False, with the element
// named in message, if something changed that was not rehashed: a bug in the hash's change tracking.
bool lpWorld_CheckHash( const lpWorld* world, char* message, int size );

// Runs the determinism self-test: arithmetic with known answers (no fused multiply-add, ties to even, no
// flush-to-zero, correctly rounded sqrt and division, the min and max conventions) and the engine's own trig and cube
// root. Returns a hash that must be equal on every machine that plays together (a session handshake compares it), and
// counts the known answers that came out wrong in *failures (may be NULL).
uint64_t lpDeterminismSelfTest( int* failures );

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
	int leakingPools;  // this step

	// Rigs (rig.c)
	float rigMs;
	int footCasts; // this step

	// Floating-point control words found changed (flush-to-zero, rounding) and put back since the world was created:
	// a library or driver on one of our threads changed it. Nonzero is worth a warning (determinism rule 15).
	int fpRepairs;
	int commandsApplied; // this step
	int commandsDropped; // this step: stale references, or a scene's control of what a player drives
	// the physics engine's, after the step
	int shapes;
	int contacts;
	int awakeContacts;
} lpStats;

lpStats lpWorld_GetStats( const lpWorld* world );

// ---- queries ----

typedef struct lpRayHit
{
	lpPos point;
	lpVec3 normal;
	int piece;
	int body;
	int link; // a rope was hit first (piece and body are -1 then); -1 otherwise
	uint32_t pieceGeneration, bodyGeneration, linkGeneration; // of the slots hit, for naming them in a command
	bool hit;
} lpRayHit;

lpRayHit lpWorld_CastRay( const lpWorld* world, lpPos origin, lpVec3 translation );

// ---- inspection: the state of things, for tools, tests and agents (reads only) ----

uint64_t lpWorld_GetTick( const lpWorld* world ); // steps taken

typedef enum lpBodyKind
{
	lp_kindStructure, // static until pieces break loose; anchored components stay
	lp_kindDebris,	  // dynamic physics body (vehicles and rigs too)
	lp_kindRubble,	  // debris that settled and was frozen static; wakes when something happens nearby
	lp_kindGhost,	  // no physics body: flies ballistically and passes through everything
	lp_kindScrap,	  // no physics body: a landed ghost, render-only
} lpBodyKind;

// How much a physics debris body interacts (kept when it freezes into rubble)
typedef enum lpTier
{
	lp_tierFull,  // collides with everything
	lp_tierLight, // collides with static geometry only; cannot push anything
} lpTier;

typedef struct lpBodyInfo
{
	bool alive;
	uint32_t generation; // changes when the slot is reused
	int kind;			 // lpBodyKind
	int tier;			 // lpTier
	bool awake;			 // debris the physics engine is moving, or a ghost in flight
	bool unsettled;		 // structure: its stress solve is still working, or joints are straining toward a break
	int pieceCount;
	float volume;				 // m^3
	lpWorldTransform transform;	 // its frame, in which its pieces' geometry is given
	lpVec3 linearVelocity;		 // of its centre of mass, world (0 for static bodies and scrap)
	lpVec3 angularVelocity;
} lpBodyInfo;

lpBodyInfo lpWorld_GetBodyInfo( const lpWorld* world, int body );

// A joint between two pieces of one body, as its last converged stress check saw it
typedef struct lpBondInfo
{
	bool alive;
	uint32_t generation; // of its slot (bonds are reused)
	int pieceA, pieceB; // pieceA < pieceB
	int joint;			// lpJointId
	float area;			// m^2
	lpPos centroid;		// world
	lpVec3 normal;		// world, unit, from pieceA toward pieceB
	float health;		// J/m^2 of damage it can still take (strength when intact)
	float strength;
	float utilization; // 1 at its limit
	float strain;	   // overload accumulated over checks; it breaks at 1
	lpVec3 force;	   // N, world: what it carries (tension along +normal)
	lpVec3 moment;	   // N m, world
} lpBondInfo;

int lpWorld_GetBondCapacity( const lpWorld* world );
lpBondInfo lpWorld_GetBondInfo( const lpWorld* world, int bond );

// A touching contact point of a body's piece
typedef struct lpContactInfo
{
	int piece;		  // the body's own piece
	int other;		  // the piece it touches, -1 for none (the ground of a test)
	lpVec3 normal;	  // unit: the way the contact pushes the piece
	lpPos point;	  // world
	float separation; // m, negative when overlapping
	float impulse;	  // total normal impulse over the last step, N s
} lpContactInfo;

// A physics body's contact points, sorted by (piece, other): copies up to capacity and returns how many there are (0
// for ghosts, scrap and free slots). Not thread-safe: it reuses the physics backend's report buffer.
int lpWorld_GetBodyContacts( const lpWorld* world, int body, lpContactInfo* contacts, int capacity );

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
	uint8_t material;	 // lpMaterialId
	uint8_t joint;		 // lpJointId where it meets other parts
	lpVec3 centroid;	 // body frame
} lpPieceInfo;

int lpWorld_GetPieceCapacity( const lpWorld* world );
lpPieceInfo lpWorld_GetPieceInfo( const lpWorld* world, int piece );
// How well a supply channel is fed at a piece, 0 to 1: the sum of the shares of the sources its carriers reach
float lpWorld_GetPieceSupply( const lpWorld* world, int piece, int channel );
// What is left in the pool of the source part a piece came from, 0 to 1 (-1: no pool); leaking, how fast (share/s)
float lpWorld_GetPiecePool( const lpWorld* world, int piece, float* leak );

int lpWorld_GetBodyCapacity( const lpWorld* world );
bool lpWorld_GetBodyTransform( const lpWorld* world, int body, lpWorldTransform* transform );

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

// ---- timing, for stats and tools (the simulation never reads the clock) ----

uint64_t lpGetTicks( void );
float lpGetMilliseconds( uint64_t startTicks ); // since startTicks
float lpGetMillisecondsAndReset( uint64_t* ticks ); // since *ticks, which becomes now

#ifdef __cplusplus
}
#endif
