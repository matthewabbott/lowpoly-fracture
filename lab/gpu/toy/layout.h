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
// (-1 when not solved this tick).
LSTRUCT( Pair )
{
	int32_t bodyA;
	int32_t bodyB;
	int32_t prevIndex;
	int32_t colour;
};

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

#define HULL_MAX_VERTS 16
#define HULL_MAX_FACES 24
#define HULL_MAX_EDGES 48 // half-edges; twins are adjacent (2k, 2k + 1), as Box3D's
#define MANIFOLD_POINTS 4

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
	int32_t flags;
};

// The centre of mass and the orientation.
LSTRUCT( BodyPose )
{
	Pos3 p;
	Q4 q;
};

// Inverse mass and inertia. V4: mantissas, the value being mantissa * 2^-e (eM for the mass, eI for both inertias).
LSTRUCT( BodyMass )
{
	Sym3 invIl; // local, about the centre of mass
	Sym3 invIw; // world, written by prepareBodies
	T invMass;
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
	T baseSeparation; // Q9.22
	T normalImpulse;  // V4: mantissa with the manifold's impulse exponent eP
	T totalNormalImpulse;
	T normalMass; // V4: mantissa with the point's exponent
	T pad;
	uint32_t featureId;
	int32_t shN; // V4: the shift of normalMass x velocity -> impulse
};

LSTRUCT( Manifold )
{
	ManifoldPoint points[MANIFOLD_POINTS];
	V3 normal; // Q1.30, from A to B
	V3 frictionImpulse;
	T twistImpulse;
	T axisSeparation; // the SAT cache as data: the separation of the cached axis, Q9.22
	int32_t pointCount;
	int32_t bodyA;
	int32_t bodyB;
	int32_t eP;		  // V4: impulse exponent
	int32_t axisType; // Box3D's b3SeparatingFeature
	int32_t axisA;
	int32_t axisB;
	int32_t pad;
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
	int32_t sleepTicks; // ticks under the thresholds before an island may sleep (30)
	int32_t sleepCap;	// the counter stops here
	int32_t substeps;
	int32_t enableSleep;
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
