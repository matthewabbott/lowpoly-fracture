// rowlayout.h: T10's E11 row benchmark (docs/research/m7-gpu-integer.md section 10). One row is one
// manifold point of one contact with private copies of both bodies: Box3D's mesh push point
// (contact_solver.c:449-500) followed by the relax row (:571-610) on the updated velocities.
//
//   VAR_F   V1: fp32, no contraction
//   VAR_I64 V2: Q32.32 in int64, 128-bit products from 32-bit limbs
//   VAR_V3  V3: int32, one global power-of-two scale per quantity, 32x32->64 products
//   VAR_V4  V4: int32 block-scaled (per-body inverse mass and inertia exponents, per-point normal
//               mass exponent, per-manifold impulse exponent e_P = e_n + 10, shift bytes from prepare)
//   VAR_V4B V4b: V4 with clz normalisation of r x P and of the velocity term instead of static shifts
#ifndef ROWLAYOUT_H
#define ROWLAYOUT_H

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
#elif defined( VAR_I64 )
typedef int64_t T;
#elif defined( VAR_V3 ) || defined( VAR_V4 ) || defined( VAR_V4B )
typedef int32_t T;
#define INT32_FORMAT 1
#else
#error "define VAR_F, VAR_I64, VAR_V3, VAR_V4 or VAR_V4B"
#endif

// Fixed formats of V3, V4 and V4b (T10 section 1.1)
#define S_V 22	// linear velocity Q9.22
#define S_W 23	// angular velocity Q8.23
#define S_DP 26 // delta position Q5.26
#define S_R 24	// anchors Q8.24
#define S_S 22	// separation Q9.22
#define S_Q 30	// unit vectors, quaternions
#define S_IH 8	// inv_h = 240 (exact) at 2^8
#define S_BR 24 // bias rate Q8.24
#define S_MS 30 // massScale, impulseScale Q1.30
// V3's global scales for what V4 gives exponents
#define S_IM 26 // inverse mass: 20 kg^-1 -> 1.3e9
#define S_II 13 // inverse inertia: 1.6e5 -> 1.3e9, 1.4e-4 -> 1.2 (one lsb: the failure V3 shows)
#define S_NM 19 // normal mass: 3000 kg -> 1.6e9, 0.025 kg -> 13107
#define S_P 15	// impulse: 34000 N s -> 1.1e9, chip resting impulse 2e-3 N s -> 66

LSTRUCT( RBody )
{
	T v[3];
	T w[3];
	T dq[4]; // x, y, z, s
	T dp[3];
	T invMass;
	T invI[9]; // row major, world frame
	int32_t eM;
	int32_t eI;
	int32_t flags; // 1: dynamic
	int32_t pad;
};

LSTRUCT( RRow )
{
	RBody A;
	RBody B;
	T rA[3];
	T rB[3];
	T n[3];
	T baseSep;
	T normalMass;
	T impulse; // warm-start impulse
	T massScale;
	T impulseScale;
	T biasRate;
	T contactSpeed;
	T invH;
	int32_t eN;
	int32_t eP; // impulse scale (bits after the point): e_P in T10's sign convention is -eP
	int32_t shMA;
	int32_t shMB;
	int32_t shIA;
	int32_t shIB;
	int32_t shN;
	int32_t pad;
};

LSTRUCT( ROut )
{
	T vA[3];
	T wA[3];
	T vB[3];
	T wB[3];
	T pushImpulse;
	T impulse;
};

LSTRUCT( RPush )
{
	uint32_t count;
	uint32_t pad0;
	uint32_t pad1;
	uint32_t pad2;
};

#endif
