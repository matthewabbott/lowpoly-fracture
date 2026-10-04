// layout.h: the toy's data layout, shared by the Slang kernels (num.slang, kernels.slang, battery.slang) and the C
// driver (DESIGN.md "Layouts"). Plain scalar structs only (E11's LSTRUCT), so the C layout, the Slang C++ twin's and
// SPIR-V std430 agree; the driver checks the sizes against the twin's at start.
//
// The dialect picks the scalar type T:
//   DIALECT_F : float   (fp32, no contraction; positions float, as Box3D)
//   DIALECT_V4: int32   (one static format per quantity, below; positions int64 as two words, 32 fractional bits)
//   DIALECT_D : double  (twin only: the accuracy reference)
// F and D ignore the formats; V4's products shift by them (num.slang's M( a, b, sh )).
#ifndef TOY_LAYOUT_H
#define TOY_LAYOUT_H

#ifdef __SLANG__
#define LSTRUCT( name ) struct name
#else
#include <stdint.h>
#define LSTRUCT( name )                                                                                                \
	typedef struct name name;                                                                                          \
	struct name
#endif

// ---------------------------------------------------------------------------------------------------------------------
// Dialect-free: AABBs, pairs, push constants, hashes (the CPU stages include only these: TOY_LAYOUT_NO_DIALECT)
// ---------------------------------------------------------------------------------------------------------------------

// Integer AABB on the 2^-10 m grid (prepareBodies), read back by the CPU's broadphase: inclusive cells.
LSTRUCT( Aabb )
{
	int32_t minx;
	int32_t miny;
	int32_t minz;
	int32_t maxx;
	int32_t maxy;
	int32_t maxz;
	int32_t pad0;
	int32_t pad1;
};

// A broadphase pair (CPU): bodies by index (bodyA < bodyB), last tick's slot of the same key (-1 when new), colour
// (-1 when not solved this tick), which bodies the solver moves (PAIR_SOLVE_*: awake dynamic bodies; a static or
// sleeping body is solved as static, Box3D's null body).
LSTRUCT( Pair )
{
	int32_t bodyA;
	int32_t bodyB;
	int32_t prevIndex;
	int32_t colour;
	int32_t flags;
	int32_t pad;
};

#define PAIR_SOLVE_A 1
#define PAIR_SOLVE_B 2

LSTRUCT( Push )
{
	uint32_t start; // into the list buffer
	uint32_t count;
	uint32_t aux;
	uint32_t pad;
};

// One 64-bit element hash as two words (no 64-bit integer in the kernels).
LSTRUCT( Hash2 )
{
	uint32_t lo;
	uint32_t hi;
};

#if !defined( TOY_LAYOUT_NO_DIALECT )

#if defined( DIALECT_F )
typedef float T;
#elif defined( DIALECT_V4 )
typedef int32_t T;
#elif defined( DIALECT_D )
typedef double T;
#else
#error "define DIALECT_F, DIALECT_V4 or DIALECT_D"
#endif

// V4 formats: bits after the binary point (Qm.n: m integer bits with the sign, n = S_*)
#define S_Q 30	// unit vectors, quaternions, rotation matrices: Q1.30 (Q2.30 range)
#define S_R 24	// points, anchors, hull points, extents, margins: Q8.24 (+-128 m)
#define S_S 22	// separations: Q9.22
#define S_D2 20 // squared distances: Q11.20
#define S_V 22	// linear velocity: Q9.22 (+-512 m/s)
#define S_W 20	// angular velocity: Q11.20 (+-2048 rad/s)
#define S_DP 26 // position delta over one step: Q5.26 (+-32 m)
#define S_P 32	// positions: int64 as two words, 32 fractional bits (+-2^31 m)
#define S_H 32	// substep h = 1/240 s: Q0.32
#define S_IH 8	// 1/h = 240: Q23.8
#define S_MS 30 // dimensionless scales and constants (mass scale, friction, Newton constants): Q1.30
#define S_A 29	// angles: Q2.29 (+-4 rad)
#define S_U 20	// (speed / max speed)^2 in the speed caps: Q11.20
#define S_VV 30 // squared velocities near rest (the sleep test): Q1.30
#define S_G 10	// the broadphase grid: 2^-10 m
#define S_BR 24 // the soft contact's bias rates and 1 / speculative distance (1/s, 1/m): Q8.24
#define S_JA 28 // joint angles, limits and servo targets: Q3.28 (+-8 rad, so a difference of two angles fits)
#define S_TQ 16 // motor torques (N m): Q15.16
#define S_J 22	// the joint's 3x3 point block: reciprocal pivots, multipliers and its inverse, Q9.22

#define HULL_MAX_VERTS 16
#define HULL_MAX_FACES 24
#define HULL_MAX_EDGES 48 // half-edges; twins are adjacent (2k, 2k + 1), as Box3D's
#define MANIFOLD_POINTS 4
#define CLIP_MAX 32 // the clipped polygon: an incident face (<= 16 vertices) cut by a reference face's side planes

// The SAT's axes and the cache's type: Box3D's b3SeparatingFeature values
#define AXIS_NONE 0
#define AXIS_FACE_A 2
#define AXIS_FACE_B 3
#define AXIS_EDGE 4

// narrowSat's verdict (SatAxis.kind)
#define SAT_CACHE_SEPARATED 1 // last tick's cached axis still separates: no contact, the cache kept
#define SAT_SEPARATED 2		  // the full SAT found a separating axis (SatAxis.type, indices, separation)
#define SAT_OVERLAP 3		  // no axis separates by more than the speculative distance: build a contact
#define SAT_RECYCLED 4		  // contact recycling: the relative pose moved less than the recycle distance

LSTRUCT( V3 )
{
	T x;
	T y;
	T z;
};

LSTRUCT( Q4 )
{
	T x;
	T y;
	T z;
	T s;
};

LSTRUCT( Sym3 )
{
	T xx;
	T xy;
	T xz;
	T yy;
	T yz;
	T zz;
};

// A position: float or double in F and D; in V4 an int64 with 32 fractional bits as a low word and a signed high
// word, so no kernel holds an int64 (the same bytes as an int64 on a little-endian machine).
#if defined( DIALECT_V4 )
LSTRUCT( Pos3 )
{
	uint32_t xlo;
	int32_t xhi;
	uint32_t ylo;
	int32_t yhi;
	uint32_t zlo;
	int32_t zhi;
};
#else
LSTRUCT( Pos3 )
{
	T x;
	T y;
	T z;
};
#endif

// ---------------------------------------------------------------------------------------------------------------------
// Shapes: convex hulls in Box3D's half-edge form, local to the body's centre of mass
// ---------------------------------------------------------------------------------------------------------------------

LSTRUCT( Hull )
{
	V3 boundsCenter; // local AABB centre, Q8.24
	V3 boundsHalf;	 // local AABB half extents, Q8.24
	V3 maxExtentV;	 // the largest |vertex| per axis, Q8.24 (Box3D's b3BodySim maxExtent: contact recycling, isFast)
	T maxExtent;	 // largest distance from the centre of mass to a vertex, Q8.24 (the sleep test, Box3D's)
	T minExtent;	 // smallest distance from the centre of mass to a face plane, Q8.24
	int32_t vertexStart;
	int32_t vertexCount;
	int32_t faceStart;
	int32_t faceCount;
	int32_t edgeStart; // into hullEdges: next | twin << 8 | origin << 16 | face << 24, indices local to the hull
	int32_t edgeCount;
};

// A face plane, n . x = offset: normal Q1.30 and offset Q8.24, recomputed from the quantised points.
LSTRUCT( HullFace )
{
	V3 normal;
	T offset;
	int32_t edge; // one half-edge of the face (local)
	int32_t pad;
};

// ---------------------------------------------------------------------------------------------------------------------
// Bodies: state, pose and mass in separate buffers
// ---------------------------------------------------------------------------------------------------------------------

// Velocities, the deltas accumulated over a step, the sleep counter (Box3D's b3BodyState).
LSTRUCT( BodyState )
{
	V3 v;  // Q9.22
	V3 w;  // Q11.20
	V3 dp; // Q5.26
	Q4 dq; // Q1.30
	int32_t sleepTicks; // ticks in a row under the sleep thresholds (finalize)
	int32_t flags;		// STATE_FAST (finalize)
};

#define STATE_FAST 1 // Box3D's b3_isFast: the body moved more than half its inner radius last tick (no contact recycling)

// The centre of mass and the orientation.
LSTRUCT( BodyPose )
{
	Pos3 p;
	Q4 q;
};

// Inverse mass and inertia, and the material. V4: mantissas, the value being mantissa * 2^-e (eM for the mass, eI for
// both inertias).
LSTRUCT( BodyMass )
{
	Sym3 invIl; // local, about the centre of mass
	Sym3 invIw; // world, written by prepareBodies
	T invMass;
	T friction;	   // Q1.30; a pair mixes them as Box3D does, sqrt( fA fB )
	T restitution; // Q1.30; a pair takes the larger
	T pad;
	int32_t eM;
	int32_t eI;
	int32_t hull;
	int32_t flags; // BODY_STATIC
};

#define BODY_STATIC 1

// ---------------------------------------------------------------------------------------------------------------------
// Contacts (step 3 on): ping-ponged manifolds with persistent feature ids and impulses
// ---------------------------------------------------------------------------------------------------------------------

LSTRUCT( ManifoldPoint )
{
	V3 anchorA; // Q8.24, world frame, from A's centre of mass
	V3 anchorB;
	T separation;	  // Q9.22
	T baseSeparation; // Q9.22: the separation at the last full narrowphase (Box3D's, for contact recycling)
	T normalImpulse;  // V4: mantissa with the manifold's impulse exponent eP
	T totalNormalImpulse;
	T normalMass; // V4: mantissa with the point's exponent
	T pad;
	uint32_t featureId; // Box3D's b3MakeFeatureId: owner1 << 24 | index1 << 16 | owner2 << 8 | index2
	int32_t shN;		// V4: the shift of normalMass x velocity -> impulse
};

// Written by narrowClip for every active pair (copyManifolds carries an inactive pair's), indexed by pair; the next
// tick reads it by prevIndex. The SAT cache lives here as data (Box3D's b3SATCache: type, indices, separation), so it
// persists while the pair does, touching or not.
LSTRUCT( Manifold )
{
	ManifoldPoint points[MANIFOLD_POINTS];
	V3 normal; // Q1.30, from A to B
	V3 frictionImpulse;
	T twistImpulse;
	T axisSeparation; // the SAT cache: the separation when it was filled (Box3D's cache.separation), Q9.22
	int32_t pointCount;
	int32_t bodyA;
	int32_t bodyB;
	int32_t eP;		  // V4: impulse exponent
	int32_t axisType; // the SAT cache: AXIS_* (Box3D's b3SeparatingFeature)
	int32_t axisA;	  // face of A (and B's support vertex in axisB), vertex of A and face of B, or edge of A and edge of B
	int32_t axisB;
	int32_t flags; // MANIFOLD_CACHE_VALID, MANIFOLD_RECYCLED
	// contact recycling (Box3D's b3Collide, its default): the poses at the last full narrowphase
	Q4 cachedRotationA; // Q1.30
	Q4 cachedRotationB;
	V3 cachedPoseP; // B's centre in A's frame, Q8.24 (b3InvMulWorldTransforms)
	Q4 cachedPoseQ; // qA* qB
};

#define MANIFOLD_CACHE_VALID 1 // Box3D's b3_relativeTransformValid: the cached poses are set
#define MANIFOLD_RECYCLED 2	   // this tick's manifold is last tick's, its separations moved with the bodies

// narrowSat's record for narrowClip, per pair (not kept between ticks): the cached feature to try first, the full SAT's
// verdict, and the three queries. Separations Q8.24 (S_R).
LSTRUCT( SatAxis )
{
	T faceASep; // the face query of A: the best face's separation
	T faceBSep;
	T edgeSep;	// the best edge pair's (when edgeA >= 0)
	T sepSep;	// SAT_SEPARATED: the separating axis's separation
	int32_t kind;	   // SAT_*
	int32_t type;	   // SAT_SEPARATED: the separating axis (AXIS_*), its indices in sepA, sepB (the cache's order)
	int32_t sepA;
	int32_t sepB;
	int32_t cacheType; // the cached feature to try first (AXIS_NONE: none): Box3D's cached-contact attempt
	int32_t cacheA;	   // face of A and B's fresh support vertex, vertex of A and face of B, or the edge pair
	int32_t cacheB;
	int32_t faceA; // face query of A: the face, B's support vertex
	int32_t vertexB;
	int32_t faceB; // face query of B: the face, A's support vertex
	int32_t vertexA;
	int32_t edgeA; // the best edge pair (-1: no pair passes the Gauss-map test)
	int32_t edgeB;
	int32_t pad;
};

// The closest call of each decision a pair's narrowphase made, per pair (diagnostics for the corpus's ties, written
// only when Params.narrowDiag is set; never hashed). Lengths Q8.24, areas Q11.20, cosines Q1.30; the format's largest
// value (F, D: 1e6) when no decision of that kind was made.
LSTRUCT( NarrowDiag )
{
	T faceAGap;	 // face query of A: best minus second-best separation (of the faces visited)
	T faceASpec; // the closest |separation - speculative distance| of the faces visited (Box3D's early exit)
	T faceBGap;
	T faceBSpec;
	T edgeGap;	 // edge query: best minus second-best separation of the pairs that pass
	T edgeSpec;
	T edgeTest;	 // the best pair's closest Gauss-map or parallel-edge test value (Q8.24)
	T cacheSpec; // the cached axis's |separation - speculative distance|
	T axisLen;	 // narrowClip: the closest decision on the axis's path (the queries it used, the face and edge
				 // preferences, the cache's consistency), Q8.24: below the tie tolerance, the axis is a tie
	T clipLen;	 // the contact's closest length decision: clip distances, kept separations, the reduction's first step
	T clipArea;	 // the reduction's closest score comparison
	T clipCos;	 // the incident face's closest comparison
	T touchLen;	 // whether a contact touches at all: the clipped separation against the speculative distance, the
				 // distances of a clip pass that ends with 2 or 3 vertices, the edge contact's segment ends
	T supportA;	  // the support vertex beside face query A's best face, against the next vertex along its normal
	T supportB;	  // the same for face query B
	T supportGap; // the same for the cached face
};

// ---------------------------------------------------------------------------------------------------------------------
// The contact solve (step 4): one constraint per pair, written by prepareContacts every tick (Box3D's
// b3ContactConstraintWide, one lane), solved per colour, its impulses stored back into the manifold (storeImpulses)
// ---------------------------------------------------------------------------------------------------------------------

// V4's impulses (normal, friction, twist, restitution; linear N s and angular N m s alike) are mantissas with the
// constraint's exponent eP, from the lighter body solved as dynamic: the reduced mass mu = 1 / (invMassA + invMassB) is
// at most the lighter mass, so mu 2^eP lies in [2^19, 2^21] and an impulse of mu x 1024 m/s fits. Effective masses
// carry their own exponents, folded into the shifts below, all computed in prepare and clamped to [1, 62] (counted).
LSTRUCT( ConstraintPoint )
{
	V3 anchorA;			  // Q8.24, world frame, from A's centre of mass (the manifold's)
	V3 anchorB;
	T baseSeparation;	  // Q9.22: separation - (rB - rA) . n (Box3D's anchor update: s = base + n . (dpB - dpA) + ...)
	T normalImpulse;	  // eP
	T totalNormalImpulse; // the relax passes' and restitution's sum, eP
	T restitutionImpulse; // eP
	T normalMass;		  // 1 / k, V4: a mantissa (its exponent folded into shN)
	T leverArm;			  // |rA - centerA|, Q8.24 (the twist friction's limit)
	T relativeVelocity;	  // the normal velocity at prepare, Q9.22 (restitution)
	T pad;
	int32_t shN; // V4: normalMass x velocity (Q9.22) -> impulse (eP)
	int32_t pad1;
};

LSTRUCT( Constraint )
{
	ConstraintPoint points[MANIFOLD_POINTS];
	V3 normal;	 // Q1.30, from A to B
	V3 tangent1; // b3Perp( normal )
	V3 tangent2; // tangent1 x normal
	V3 centerA;	 // the friction centre (the points' weighted mean), Q8.24, from A's centre of mass
	V3 centerB;
	Sym3 invIA; // the world inverse inertia, V4: a mantissa (2^-eI); zero for a body solved as static
	Sym3 invIB;
	T invMassA; // V4: a mantissa (2^-eM); zero for a body solved as static
	T invMassB;
	T tangentMassXX; // the inverse of the 2x2 tangent mass, V4: mantissas (their exponent folded into shT)
	T tangentMassXY;
	T tangentMassYY;
	T twistMass;		// 1 / (n . (IA + IB) n), V4: a mantissa (shTw)
	T frictionImpulse1; // along tangent1, eP
	T frictionImpulse2; // along tangent2
	T twistImpulse;		// eP
	T friction;			// Q1.30, sqrt( fA fB )
	T restitution;		// Q1.30, max( rA, rB )
	T pad;
	int32_t bodyA;
	int32_t bodyB;
	int32_t pointCount; // 0: the pair is coloured but has no contact this tick (nothing is solved)
	int32_t flags;		// PAIR_SOLVE_A, PAIR_SOLVE_B, CON_SOFT
	int32_t eP;			// V4: the impulses' exponent
	int32_t shMA;		// V4: invMass x impulse -> velocity (Q9.22)
	int32_t shMB;
	int32_t shIA; // V4: invI x angular impulse -> angular velocity (Q11.20)
	int32_t shIB;
	int32_t shT;  // V4: tangentMass x velocity -> impulse
	int32_t shTw; // V4: twistMass x angular velocity -> impulse
	int32_t pad1;
};

#define CON_SOFT 4 // one body is solved as static: Box3D's static softness

// ---------------------------------------------------------------------------------------------------------------------
// Revolute joints (step 6): Box3D's b3RevoluteJoint, one struct per joint holding the definition (the scene's,
// quantised), the step's prepared data (prepareJoints) and the impulses (kept from tick to tick: the warm start). Its
// index is fixed for the run. The CPU's per-tick decisions and commands come in a JointCommand per joint.
// ---------------------------------------------------------------------------------------------------------------------

LSTRUCT( Joint )
{
	// the definition
	V3 localAnchorA;   // Q8.24: the hinge point in A's body frame, from A's centre of mass
	V3 localAnchorB;
	Q4 localFrameA;	   // Q1.30: the joint frame in A's body frame; its z axis is the hinge axis
	Q4 localFrameB;
	T lowerAngle;	   // Q3.28 (S_JA)
	T upperAngle;
	T maxMotorTorque;  // N m, Q15.16 (S_TQ)
	T motorSpeed;	   // rad/s, Q11.20: the speed motor's (JOINT_MOTOR without JOINT_SERVO)
	T servoGain;	   // 1/s, Q8.24: the servo's speed is gain (target - angle), capped at servoMaxSpeed
	T servoMaxSpeed;   // rad/s, Q11.20
	// the step's (prepareJoints)
	Q4 frameAq;		   // the world joint frames (b3RevoluteJoint frameA.q, frameB.q), Q1.30
	Q4 frameBq;
	V3 frameAp;		   // the hinge point from each centre of mass, world frame, at prepare (frameA.p, frameB.p), Q8.24
	V3 frameBp;
	V3 deltaCenter;	   // pB - pA at prepare, Q8.24
	V3 axisZ;		   // the hinge axis (frameA's z), Q1.30
	V3 perpAxisX;	   // the axis block's Jacobians, Q1.30 (the solve keeps them current, for the warm start)
	V3 perpAxisY;
	Sym3 invIA;		   // V4: mantissas (2^-eI); zero for a body solved as static
	Sym3 invIB;
	Sym3 pointMass;	   // the 3x3 point block's inverse (Gaussian elimination, reciprocal pivots), V4: mantissas (shK)
	T invMassA;		   // V4: a mantissa (2^-eM); zero for a body solved as static
	T invMassB;
	T axisMassXX;	   // the 2x2 axis block's inverse, V4: mantissas (shK2)
	T axisMassXY;
	T axisMassYY;
	T axialMass;	   // 1 / (z . (IA + IB) z), V4: a mantissa (shAx)
	T angle;		   // the twist angle at prepare, Q3.28
	T speed;		   // the motor's speed this step (the servo's from the target), Q11.20
	T maxMotorImpulse; // maxMotorTorque h, eP
	// the impulses (eP), kept from tick to tick
	V3 linearImpulse;
	T perpImpulseX;
	T perpImpulseY;
	T motorImpulse;
	T lowerImpulse;
	T upperImpulse;
	T pad;
	int32_t bodyA;
	int32_t bodyB;
	int32_t flags; // JOINT_LIMIT, JOINT_MOTOR, JOINT_SERVO
	int32_t solve; // the bodies the last prepare solved as dynamic (PAIR_SOLVE_A, PAIR_SOLVE_B)
	int32_t eP;	   // V4: the impulses' exponent
	int32_t shMA;  // V4: invMass x impulse -> velocity (Q9.22)
	int32_t shMB;
	int32_t shIA;  // V4: invI x angular impulse -> angular velocity (Q11.20)
	int32_t shIB;
	int32_t shK;   // V4: pointMass x velocity (Q9.22) -> impulse
	int32_t shK2;  // V4: the axis block's inverse x angular velocity (Q11.20) -> impulse
	int32_t shAx;  // V4: axialMass x angular velocity -> impulse
};

#define JOINT_LIMIT 1 // lower and upper angle limits
#define JOINT_MOTOR 2 // a speed motor with a torque cap
#define JOINT_SERVO 4 // the motor's speed from a target angle (the JointCommand's), each step

// Per joint per tick, from the CPU: this tick's servo target (a command) and the stages' decisions
LSTRUCT( JointCommand )
{
	T target;	   // the servo's target angle, Q3.28
	T pad;
	int32_t solve; // PAIR_SOLVE_A | PAIR_SOLVE_B: the awake dynamic bodies (0: the joint is not active this tick)
	int32_t colour;
};

// ---------------------------------------------------------------------------------------------------------------------
// Params (every tolerance)
// ---------------------------------------------------------------------------------------------------------------------

LSTRUCT( Params )
{
	T h;			 // substep, Q0.32
	T inv_h;		 // Q23.8
	V3 gravityDelta; // h * g, Q9.22
	T aabbMargin;	 // Q8.24
	T linearSlop;	 // Q8.24
	T speculativeDistance;
	T sleepV;			  // the sleep threshold (m/s), Q9.22: a per-component gate on v before squaring
	T sleepW;			  // the same value in Q11.20: the gate on maxExtent * w
	T sleepSq;			  // its square, Q1.30: |v|^2 + |maxExtent w|^2 must stay under it
	T invMaxLinearSpeed;  // Q1.30
	T invMaxAngularSpeed; // Q1.30
	T oneU;				  // 1 in S_U
	T one;				  // Q1.30
	T half;
	T oneHalf;
	// the narrowphase's tolerances (Box3D's b3CollideHulls and b3ReduceManifoldPoints)
	T faceTolerance;	 // 0.5 linearSlop, Q8.24: face B over face A, and the edge's absolute preference
	T edgeRelTolerance;	 // 0.9, Q1.30: the edge contact replaces the face's when edge > 0.9 face + faceTolerance
	T parallelTolerance; // 0.005, Q1.30: B3_PARALLEL_EDGE_TOL (edges this close to parallel give no axis)
	T reduceBias;		 // 0.95, Q1.30: the reduction's pecking order
	T speculativeSq;	 // speculativeDistance^2, Q11.20
	T cacheTolerance;	 // linearSlop in Q9.22: a cached feature is kept while its separation moves less
	// the contact solve (Box3D's world defaults: contact hertz 30, damping ratio 10, the static softness at twice the
	// hertz and half the damping ratio, b3MakeSoft at h; contact speed 3 m/s; restitution threshold 1 m/s)
	T dynBiasRate;		 // massScale x biasRate, Q8.24 (S_BR)
	T dynMassScale;		 // Q1.30
	T dynImpulseScale;	 // Q1.30
	T staBiasRate;		 // the static softness's
	T staMassScale;
	T staImpulseScale;
	T negContactSpeed;	 // -contactSpeed (the push's largest overlap bias), Q9.22
	T negRestitutionThreshold; // Q9.22
	T invSpeculative;	 // 1 / speculativeDistance, Q8.24 (the friction centre's weights, Box3D's invTau)
	T minFrictionWeight; // Q9.22 (Box3D's B3_MIN_FRICTION_WEIGHT; see NOTES.md)
	T oneV;				 // 1 in Q9.22
	T twoV;				 // 2 in Q9.22
	// contact recycling (Box3D's default: 10 linearSlop; 0 turns it off) and Box3D's isFast
	T recycleDistance;	   // Q8.24, for a pair that was touching
	T recycleNonTouching;  // min( recycleDistance, speculativeDistance ), Q8.24
	T recycleAngular;	   // cos( 5 degrees )^2 (Box3D's B3_CONTACT_RECYCLE_ANGULAR_DISTANCE: 10 degrees), Q1.30
	T fastSafety;		   // Box3D's safetyFactor, 0.5, Q1.30
	T dt;				   // the step, Q0.32
	int32_t sleepTicks;	 // ticks under the thresholds before an island may sleep (30)
	int32_t sleepCap;	 // the counter stops here
	int32_t substeps;
	int32_t enableSleep;
	int32_t narrowDiag; // write NarrowDiag (the corpus)
	int32_t restitutionIterations; // Box3D's world default, 2
	// the joints (step 6): Box3D's constraint softness, b3MakeSoft( min( 60, 0.25 inv_h ), 2, h ) (b3PrepareJoint)
	T jointBiasRate;	 // 1/s, Q8.24
	T jointMassScale;	 // Q1.30
	T jointImpulseScale; // Q1.30
	T jointPad;
};

// ---------------------------------------------------------------------------------------------------------------------
// Battery (battery.slang): one input vector and one output per helper call
// ---------------------------------------------------------------------------------------------------------------------

LSTRUCT( BatIn )
{
	T a;
	T b;
	T c;
	T d;
	T e;
	T f;
	int32_t sh;
	int32_t k;
};

LSTRUCT( BatOut )
{
	T r0;
	T r1;
	T r2;
	T r3;
	uint32_t u0;
	uint32_t u1;
};

#endif // TOY_LAYOUT_NO_DIALECT

#endif
