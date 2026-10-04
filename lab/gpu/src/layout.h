// layout.h: data layout shared by the Slang kernel (solver.slang) and the C harness.
// Plain scalar structs only, so the C layout, the Slang C++ twin layout and SPIR-V std430 agree.
//
// The variant picks the scalar types:
//   VAR_F   : T = float,   TP = float    (Box3D-style fp32, no contraction)
//   VAR_I64 : T = int64_t, TP = int64_t  (Q32.32 everywhere, 128-bit products from 32-bit limbs)
//   VAR_I32 : T = int32_t, TP = int64_t  (per-quantity power-of-two scales, per-body and per-contact
//                                         exponents, 32x32->64 products, one rounding shift each)
// A fourth format only needs a new block here and in solver.slang's arithmetic section.
#ifndef LAYOUT_H
#define LAYOUT_H

#ifdef __SLANG__
#define LSTRUCT( name ) struct name
#else
#include <stdint.h>
#define LSTRUCT( name )                                                                                                \
	typedef struct name name;                                                                                          \
	struct name
#endif

#if defined( VAR_F )
typedef float T;
typedef float TP;
#elif defined( VAR_I64 )
typedef int64_t T;
typedef int64_t TP;
#elif defined( VAR_I32 ) || defined( VAR_V4 )
typedef int32_t T;
typedef int64_t TP;
#else
#error "define VAR_F, VAR_I64, VAR_I32 or VAR_V4"
#endif

#if defined( VAR_V4 )
// V4 (T10, docs/research/m7-gpu-integer.md section 1.1): block-scaled int32. Fixed formats for the
// state; inverse mass and inertia are mantissas with per-body exponents; effective masses carry
// per-point exponents; impulses carry a per-manifold exponent e_P = e_n + 10; every cross-body
// product is one 32x32->64 multiply and one shift precomputed in prepare.
#ifndef V4_SV
#define S_V 22	 // linear velocity Q9.22 (+-512 m/s)
#else
#define S_V V4_SV
#endif
#ifndef V4_SW
#define S_W 23	 // angular velocity Q8.23 (+-256 rad/s)
#else
#define S_W V4_SW
#endif
#define S_DP 26	 // delta position Q5.26 (+-32 m)
#define S_R 24	 // anchors Q8.24
#define S_S 22	 // separation Q9.22
#define S_Q 30	 // unit vectors, quaternions Q1.30 / Q2.30
#define S_H 32	 // h = 1/240 as Q0.32 (17895697)
#define S_IH 8	 // 1/h = 240, exact
#define S_BR 24	 // bias rate Q8.24
#define S_MS 30	 // massScale, impulseScale, friction, Newton constants Q1.30
#define S_U 20	 // (|w| / maxAngularSpeed)^2 in the speed clamp
#else
// I32 scales (bits after the binary point) per quantity. Unused by F and I64.
#define S_V 20	 // linear velocity, m/s        (range +-2048)
#define S_W 20	 // angular velocity, rad/s     (range +-2048)
#define S_DP 24	 // delta position in a step, m (range +-128)
#define S_R 24	 // anchors, centres, m (range +-128)
#define S_S 24	 // separations, m
#define S_Q 30	 // unit vectors and quaternion components (range +-2)
#define S_H 38	 // substep h, s (1/240 -> 1145324612)
#define S_IH 20	 // 1/h, 1/s
#define S_BR 20	 // soft bias rate, 1/s
#define S_MS 30	 // massScale, impulseScale, friction coefficient, Newton constants
#define S_U 20	 // (|w| / maxAngularSpeed)^2 in the speed clamp
#endif

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

LSTRUCT( Sym2 )
{
	T xx;
	T xy;
	T yy;
};

// Solver body state (like b3BodyState): velocities and the deltas accumulated over one step.
// Index 0 is the static body: all zero, dq identity, never written.
LSTRUCT( Body )
{
	V3 v;
	V3 w;
	V3 dp;
	Q4 dq;
	T pad0;
	T pad1;
	T pad2;
};

// Pose (like the transform in b3BodySim), updated once per step by finalize.
LSTRUCT( Pose )
{
	TP px;
	TP py;
	TP pz;
	Q4 q;
};

LSTRUCT( ContactPoint )
{
	V3 anchorA;
	V3 anchorB;
	T baseSeparation;
	T normalMass;
	int32_t shN; // V4: precomputed shift of normalMass x velocity -> impulse
	int32_t pad;
};

// One contact constraint (one manifold of 1-2 points), prepared once on the CPU.
LSTRUCT( Constraint )
{
	int32_t indexA;
	int32_t indexB;
	int32_t pointCount;
	int32_t soft; // 1: one side is static, use the static softness
	// I32 exponents (unused by F and I64): inverse mass and inertia mantissas of A and B,
	// the contact's impulse scale, normal mass and tangent mass scales. V4 stores the
	// precomputed shifts here instead: eMA/eMB/eIA/eIB = shMA/shMB/shIA/shIB, eT = shT.
	int32_t eMA;
	int32_t eMB;
	int32_t eIA;
	int32_t eIB;
	int32_t eC;
	int32_t eN;
	int32_t eT;
	int32_t pad;
	V3 normal;
	V3 tangent1;
	V3 tangent2;
	T friction;
	T invMassA;
	T invMassB;
	Sym3 invIA;
	Sym3 invIB;
	Sym2 tangentMass;
	V3 centerA;
	V3 centerB;
	ContactPoint points[2];
};

// Accumulated impulses (warm started across steps).
LSTRUCT( Impulse )
{
	T normal[2];
	T friction[2];
};

LSTRUCT( Params )
{
	T h;
	T inv_h;
	V3 gravityDelta; // h * g
	T dynBiasRate;	 // massScale * biasRate, as Box3D's push
	T dynMassScale;
	T dynImpulseScale;
	T staBiasRate;
	T staMassScale;
	T staImpulseScale;
	T negContactSpeed;
	T one;
	T half;
	T oneHalf;
	T invMaxAngularSpeed; // 1 / (0.25 pi / dt), precomputed on the CPU
	T oneU;				  // 1 in the S_U scale
};

LSTRUCT( Push )
{
	uint32_t start;
	uint32_t count;
	uint32_t pad0;
	uint32_t pad1;
};

#endif
