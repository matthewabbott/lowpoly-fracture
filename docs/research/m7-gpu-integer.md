# M7 track T10: an integer-first, deterministic, GPU-capable rigid-body core

Written 2026-09-30. The question: design a rigid-body core whose arithmetic is integer end to end, so that it is
bit-exact across GPU vendors and against a CPU twin by the definition of the operations rather than by empirical
hardware coverage, sized for the owner's decisions of 2026-09-30 (floor: a GTX 1060-class PC beside an old quad-core;
dream: tens of thousands of physical bodies on an RTX 2080-class PC; solo equals co-op; convergent lockstep).
Repo facts are `file:line` at the pinned Box3D commit; web facts carry a URL; "recalled" marks spec text I know but
could not fetch this session; "unverified" marks numbers no primary source confirmed, with what would confirm them.

## Summary

- **Integers make cross-vendor bit-exactness a property of the language specs, not of drivers.** Every operation the
  core needs (add, multiply with a 64-bit product, shift, compare, select, truncating division, floor square root)
  has one correct answer on every machine; the only ways to disagree are undefined behaviour (signed overflow, shifts
  by the width, division by zero) and compiler bugs. The first is provable away (Frama-C Eva on the C twin with
  input contracts; CBMC on single rows); the second is caught by a startup kernel battery exchanged in the session
  handshake, with the CPU twin as the identical fallback and host repair as the backstop. Nothing in the float
  case list of T5 §3 (atomics, contraction, denormals, wave-size reductions) applies: integer atomics and wave sums
  are exact and order-free, so per-body impulse totals for the stress loads become a plain `atomicAdd`.
- **The format is 32-bit mantissas with 64-bit products, plus small exponents where the dynamic range demands
  them:** 64-bit world positions (Q32.32), body-relative anchors and hull vertices in Q8.24 (dynamic bodies within
  ±32 m of their centre), velocities in fixed Q9.22 and Q8.23 (the speed caps Box3D already enforces make them
  fixed), inverse mass and inverse inertia as normalised 31-bit mantissas with a per-body exponent, impulses and
  effective masses as 31-bit mantissas with a per-manifold exponent derived from the effective mass. Cross-body
  products are one 32x32->64 multiply followed by one variable shift whose amount is fixed per (constraint, body) in
  prepare; the shift amounts are provably within [30, 63] for every mass ratio we have (a 0.05 kg chip against a
  3 t mech needs a shift of 46). Q32.32 with 128-bit products is needed nowhere in the solver; 128-bit arithmetic
  stays on the CPU for exact fracture predicates and mass properties (T4 option B), once per piece.
- **The floor GPU pays about 5x float issue cost per contact row, Turing and later about 2-4x, and that is
  affordable.** Maxwell and Pascal have no 32-bit multiplier: a 32x32->32 multiply is 3 XMAD instructions and a
  signed 32x32->64 is 7 (primary SASS counts), all at the full 128 lanes per SM. A GTX 1060 issues about 2.2 T
  integer instructions per second against roughly 0.1 T 32x32->64 products per second from an i5-6500's AVX2 units at
  peak; for a 5,000-awake-body town the whole integer step is an estimated 1.5-2.5 ms of GPU time on the 1060,
  leaving the CPU its 10 ms for fracture, stress, links and rigs (estimate; gate G4 measures).
- **Solver: port Box3D's graph-coloured soft step to integers first; AVBD is a stage-3 experiment.** The soft step is
  what our materials, tests and feel were tuned on, its CPU twin is a transcription of code we already vendor, and on
  the GPU it needs no atomics: within a colour no two constraints share a body. Colours are recomputed every tick
  from state by a fixed-round Jones-Plassmann colouring with priorities hashed from pair ids, the constraints it
  leaves uncoloured are solved by Jacobi with mass splitting (order-free because integer), so colouring stops being
  hidden state. About 3C+2 dispatches per substep, C about 12-20, roughly 220 dispatches per tick.
- **Kernel language: a C-and-HLSL shared subset compiled twice (MSVC/clang for the twin, dxc for SPIR-V and DXIL),
  not Slang's CPU target.** Slang compiles one source to SPIR-V, DXIL, MSL, CUDA, WGSL and C++ and supports 64-bit
  integers on Vulkan, D3D12, CUDA, Metal and CPU, but its CPU target is documented as preliminary ("functional, but
  not feature- or performance-complete", no barriers, atomics or groupshared, breaking changes expected). A twin
  that agents read, printf, assert and run under Eva must be plain C. Slang stays as the later front-end for Metal
  and CUDA and as a free third implementation for differential tests. API: Vulkan compute first (every vendor,
  `shaderInt64` queryable, timeline semaphores for a pipeline one tick behind, explicit readback, SDK on the machine).
- **Plan: five stages, four gates, 23-33 agent-weeks.** G1 one contact kernel bit-exact on the RTX 3060, the Intel
  UHD and the twin, within 3x of the float variant per row on the 3060; G2 the integer twin within 2.5x of Box3D at
  one worker on the town rung (1.5x with an AVX2 path); G3 a full island solve bit-exact for 600 town ticks on both
  GPUs; G4 the pipeline beating the twin at 5,000 awake bodies on a GTX 1060-class card with the whole step under
  16.7 ms. We give up Box3D (its cadence, our patches, its SIMD, its shapes we do not use), and take on exponent
  bookkeeping that the compiler cannot check and that the lint and Eva must.

Hypotheses touched: H3's "fixed point costs 2-4x" holds for a CPU-only Q32.32 design and is beside the point here:
the cost lands on a machine that is otherwise idle, and the block-scaled 32-bit format halves T4's per-multiply
estimate. H4 ("GPU physics stays off the deterministic path") is falsified in principle for integer kernels; what
survives of it is that an iGPU is still a loss and that the driver is still a JIT compiler that must be tested, not
trusted. H5 is reinforced: an integer state snapshot has no rounding or layout ambiguity, and this design deletes
three of the state audit's hidden-state items (pair set, contact ids, colours).

## 0. Why integers settle the cross-vendor question

For IEEE float, T3 showed that add, multiply, divide and sqrt are correctly rounded on CPUs and on CUDA, but on
D3D11 add and multiply may truncate, div is 1.5 ULP, denormals are flushed, `mad` may fuse; on Vulkan the float
controls are optional per device, and our own RTX 3060 reports `shaderDenormPreserveFloat32 = false` and
`shaderDenormFlushToZeroFloat32 = false` under driver 581.95 (vulkaninfo on this machine, 2026-09-30), so a float
kernel cannot even request a denormal behaviour on NVIDIA's Vulkan driver. Integers have none of this. The semantics
that matter, and where they are defined:

| operation | C17 (twin) | SPIR-V / HLSL (GPU) | rule for our subset |
|---|---|---|---|
| `+ - *` on int32/int64 | wraps only for unsigned; signed overflow is UB | wraps: the low N bits of the exact result (SPIR-V `OpIAdd`/`OpIMul`, recalled) | prove no signed overflow (Eva); use unsigned for hashes and counters |
| 32x32->64 | `(int64_t)a * b` | same; GLSL also has `imulExtended`; NVIDIA/AMD/Intel back ends strength-reduce sign-extended operands to a wide multiply (unverified on the Vulkan drivers: E11 measures) | the only multiply the solver uses |
| `/` and `%` | truncate toward zero; divisor 0 and `INT_MIN / -1` are UB | `OpSDiv` truncates, `OpSRem` takes the dividend's sign (recalled); divisor 0 gives an undefined value | divisors proven nonzero; dividends proven above `INT_MIN` |
| `>>` on signed | implementation-defined; arithmetic on MSVC, GCC, Clang (documented) | `OpShiftRightArithmetic`, defined (recalled) | pinned by the self-test; used for every floor rescale |
| shift count >= width | UB | SPIR-V: undefined; DXIL masks the count (recalled) | counts are constants or clamped by a helper |
| int -> narrower int | modular for unsigned, implementation-defined for signed (all three compilers truncate) | `OpSConvert`/`OpUConvert` truncate | only through saturating helpers |
| count leading zeros | `_BitScanReverse64` / `__builtin_clzll` (UB at 0) | `firstbithigh` (returns -1 at 0) / `findMSB` | one helper with the zero case written out |
| int -> float, float -> int | IEEE RNE (exact below 2^24), truncation after a clamp | same | only at the CPU boundary, never in kernels |

So the residual hazards are undefined behaviour in our code and bugs in compilers and drivers. Section 8 says which
is proven and which is tested.

## 1. Formats (question 1)

Design rule: every product is 32x32->64, every sum is 64-bit, every quantity that feeds the next product is narrowed
back to 32 bits by one shift, and the dynamic range that 32 bits cannot hold lives in small exponents that change
only at well-defined points (body creation; constraint prepare). The step keeps dt 1/60 with 4 substeps (h = 1/240,
stored as the Q0.32 constant 17895697, 4e-9 relative error, no worse than float's 3e-8 representation of 1/240);
h = 2^-8 with a 64 Hz tick would turn the integrations into shifts, a later option.

### 1.1 The format table

| quantity | format | range | resolution | scale rule |
|---|---|---|---|---|
| world position of a body centre; TOI pose | int64 Q32.32 m, relative to a zone origin | ±2.1e9 m | 2.3e-10 m | fixed; a zone shift is an exact 64-bit subtraction |
| delta position within a step (`b3BodyState.deltaPosition`) | int32 Q5.26 m | ±32 m | 1.5e-8 m | fixed; bound from the 400 m/s cap (`types.c:26`) x 1/60 = 6.7 m |
| body-relative anchors, hull vertices, joint frames | int32 Q8.24 m | ±128 m, invariant ±32 m for dynamic bodies | 6e-8 m | fixed; a dynamic body whose vertices exceed ±32 m from its centre is rejected at creation; static bodies use anchor 0 and a corrected `baseSeparation` (their rotation and velocity are identically zero, so `rB` cancels) |
| unit vectors: normals, tangents, axes | int32 Q1.30 | ±2 | 9.3e-10 | fixed; renormalised where produced |
| rotation (body and delta) | 4 x int32 Q2.30 (fixed3d's packed quaternion format) | ±2 | 9.3e-10 | renormalised every substep with an exact integer sqrt and one 64/32 division |
| linear velocity | int32 Q9.22 m/s | ±512 m/s | 2.4e-7 m/s | fixed; saturating (Box3D caps at 400 m/s in integrate positions; we also saturate inside the sweep and count it) |
| angular velocity | int32 Q8.23 rad/s | ±256 rad/s | 1.2e-7 rad/s | fixed; saturating (Box3D caps at 0.25 pi per step = 47 rad/s unless `allowFastRotation`, `constants.h:76`) |
| separation, speculative bias input | int32 Q9.22 m | ±512 m | 2.4e-7 m | fixed; 2.4e-7 m is 5e-5 of the 5 mm slop (`constants.h:53`) |
| inverse mass | int32 mantissa in [2^30, 2^31) x 2^e_m | any mass from 1e-3 kg to 1e9 kg | 31-bit relative | e_m per body, computed from the exact integer mass at creation (clz); never changes while the body lives; static: mantissa 0 |
| inverse inertia (local and world 3x3) | 9 x int32 mantissa x 2^e_I, local tensor normalised so its largest entry is in [2^28, 2^29) | inertias from 1e-7 to 1e9 kg m^2 | 29-31-bit relative | e_I per body, from the exact integer tensor at creation; the world tensor R I R^T keeps e_I (entries bounded by the largest eigenvalue, hence by 3x the largest local entry: two headroom bits) |
| effective masses (`normalMass`, `twistMass`, tangent 2x2, joint masses) | int32 mantissa in [2^30, 2^31) x 2^e_n | 1e-7 to 1e5 kg | 31-bit relative | e_n per manifold point (or joint row) from the 64-bit sum kNormal in prepare; the reciprocal is one normalised 64/32 division |
| impulses (normal, friction, twist, rolling, joint) | int32 mantissa x 2^e_P, e_P = e_n + 10 | ±2^31 units; unit = normalMass x 2^-21 N s (chip: 1e-8 N s; car on ground: 5e-4 N s) | | e_P per manifold, recomputed in prepare; warm-start impulses re-expressed by a floor shift when e_P changes (it moves by at most ±1 between steps because only the anchors move) |
| per-(constraint, body) shift bytes `shMA, shMB, shIA, shIB` | uint8 | [0, 63] | | computed in prepare from e_m, e_I, e_P and the fixed velocity formats; clamped and counted |
| softness (`massScale`, `impulseScale`) | int32 Q1.30 | [0, 1] | 9.3e-10 | constants from `b3MakeSoft` (`solver.h:265-307`) computed once in integer on the CPU |
| `biasRate`, `inv_h`, motor speeds | int32 Q8.24 1/s; `inv_h` = 240 exactly | | | constants |
| friction, restitution, rolling resistance, damping factors 1/(1+h c) | int32 Q1.30 | [0, 2) | | per material or body, computed once |
| angles (limits, targets, twist) | int32 Q3.28 rad | ±8 rad | 3.7e-9 rad | fixed |
| broadphase AABBs | int32 per axis at 2^-10 m from the zone origin, rounded outward | ±2,097 km | 1 mm | fixed; the fat margin (`B3_AABB_MARGIN_FRACTION`, `constants.h:103`) is added in these units |
| pair keys | uint64 = shapeA << 32 or shapeB (A < B) | | | sorted every tick |
| per-body contact impulse totals (stress loads) | int64 in units of 2^-20 N s, via two 32-bit integer atomics (the Intel UHD has no 64-bit buffer atomics: vulkaninfo `shaderBufferInt64Atomics = false`) | ±8.8e12 N s | 9.5e-7 N s | fixed |
| tick, substep, ids, colours, island labels | uint32 | | | |

### 1.2 Where Q32.32 with 128-bit products was thought unavoidable, and why it is not

T4 named four things: a 0.05 kg chip beside a 3 t mech, inertias of 1e-4 kg m^2 and below, impulses to 34,000 N s,
and dt 1/240. Each is a dynamic-range problem, and each is absorbed by an exponent rather than by wider words:

- The chip and the mech share a constraint. `kNormal = mA + mB + rnA.(iA rnA) + rnB.(iB rnB)` has terms 20 and
  3e-4 (a 2^16 ratio). Summing in int64 at the largest term's exponent keeps the mech's term to 15 bits; float keeps
  it to 8 (24 - 16). The reciprocal `normalMass` is one division of 2^62 by a 31-bit mantissa. Applying an impulse to
  the mech is `mantissa(invMass_mech) x mantissa(P) >> shift` with shift = 30 - log2(invMass_mech x normalMass) =
  30 + 16 = 46; to the chip, 30 + 0 = 30. Both in [30, 63], so a single 32x32->64 product and one shift.
- Inertia 6.3e-6 kg m^2 (inverse 1.6e5) is a mantissa 0x5000000-ish with e_I = -11; nothing is lost.
- 34,000 N s on a car-vs-static point with normalMass about 1,000 kg is 7e7 units of 5e-4 N s; the resting impulse
  per substep (66 N s) is 1.4e5 units. A chip's resting impulse (2e-3 N s) is 2e5 units of its own 1e-8 N s unit.
- dt enters only as `v h` (32x32->64, shifted into Q5.26) and as constants folded on the CPU.

What actually needs more than 64 bits, and where it goes:

1. Products of three 32-bit factors: 3x3 determinants (`b3InvertMatrix` for `rollingMass` and the angular masses,
   `b3Solve3` in the gyroscopic step `solver.c:159` and the point constraints `revolute_joint.c:587`,
   `weld_joint.c:282`, `spherical_joint.c:638`) and squared norms of cross products. Replaced by Gaussian
   elimination with a normalised 64/32 division per pivot (three pivots, 31-bit relative precision per step) and by
   renormalising a cross product to 31 bits before squaring it. Joint effective-mass matrices are inverted once per
   step in prepare with the step's anchors (Box2D v2's approximation) instead of per iteration.
2. World-position arithmetic: only 64-bit additions of shifted 32-bit deltas ever touch positions; every solver
   quantity is a delta or body-relative (Box3D already does this: `body.h:171-172`).
3. Mass properties and fracture predicates (cubic in coordinates): 128-bit on the CPU, once per piece, as T4 option
   B specifies; the resulting hull vertices are rounded once to Q8.24 and the inverse mass and inertia exponents are
   computed from the exact values with integer logic, so no float is involved anywhere in a piece's creation.
4. A 128/64 division for a Q32.32 reciprocal: not needed, because reciprocals are of normalised mantissas.

Plan B if the exponent bookkeeping proves too error-prone for agents: Q32.32 in int64 with 128-bit products on
Turing and later (4 wide multiplies plus carries per product, about 3x the block-scaled cost) and dropping Pascal.
E11's V2 variant prices that path.

### 1.3 Renormalisation rules and cross-body products

- Body exponents e_m and e_I are set at creation from exact integer mass properties and never change; a body whose
  mass changes (padding, `SetMass`) is re-created with new exponents. Static bodies have mantissa 0.
- Per-manifold exponents e_n and e_P are recomputed in prepare every step. The warm-start impulse carried from the
  previous step is shifted into the new e_P (floor); if e_P rose by k, the carried impulse loses k low bits.
- Velocities never carry exponents; every velocity update is `mantissa x mantissa >> sh` with `sh` a byte fixed in
  prepare: `sh = -(e_m + e_P + 22)` for a linear update and `sh = -(e_I + e_P - 24 + 31 + 23)` for an angular one
  after the torque impulse `r x P` has been narrowed by 31 bits; both clamped to [0, 63] with a counter on the clamp.
  A clamp at 63 means the update is below velocity resolution, which is the physically right answer.
- Sums of terms with different exponents (kNormal, the per-body torque sums in the friction row) align to the
  largest exponent with floor shifts in int64 before narrowing.
- Rounding: floor (arithmetic shift) for alignments; round-half-up (add 2^(k-1), then shift) for products that
  feed state (velocity and position integration, quaternion normalisation), so that the -0.5 ulp bias of floor
  does not accumulate over millions of substeps. Both are exact and identical on every machine.

## 2. Overflow bounds (question 2)

Bounds are component-wise so that an interval analysis (Eva) can reproduce them; norm bounds like |r| <= 32 m are
not what an interval tool sees, which is why the anchor invariant is ±2^29 units per component (32 m), not the
±2^30 the format could hold. "sat" marks a saturating narrow; every saturation increments a per-kernel counter that
the CPU reads back with the stats.

| stage | intermediate | inputs and bound | bits needed | width and action |
|---|---|---|---|---|
| rotate anchor `rot(dq, r)` | `t = 2 (u x r)`; `r + s t + u x t` | u, s Q2.30 (<= 2^30 after normalisation), r <= 2^29 | products 2^59, sums 2^61.6 | int64; narrow to Q8.24 by >> 30 (result <= |r| + rounding) |
| separation `s` | `dot(dp, n) + dot(rot(dqB,rB), n) - dot(rot(dqA,rA), n) + base` | dp <= 2^31 (Q5.26), n <= 2^30, anchors 2^29 | 2^62.2 | int64; narrow to Q9.22 sat |
| speculative bias `s inv_h` | s Q9.22 x 240 | 2^31 x 2^8 | 2^39 | int64; sat to Q10.21 (a bias beyond ±1024 m/s only ever zeroes an impulse) |
| relative normal velocity `vn` | `v + w x r` per body, difference, dot n | v 2^31, w 2^31, r 2^29: w x r products 2^60 >> 23 -> 2^37 (exceeds int32 when a 256 rad/s body has a 32 m anchor: 8,192 m/s) | 2^38 | int64 through the dot; sat to Q10.21 |
| `massScale vn + bias` | Q1.30 x Q10.21 | 2^61 >> 30 | 2^32 | sat to Q10.21 |
| `deltaImpulse` | `-m_n x (…) >> 31 - impulseScale x P >> 30` | m_n < 2^31, x < 2^31 | each term < 2^31 by construction | int64 sum, sat to int32 |
| accumulated impulse `max(P + delta, 0)` | | | 2^32 | sat to int32 (a stack heavy enough to saturate a car-scale unit is 2^31 x 5e-4 = 1e6 N s per substep: a 25,000 t load) |
| apply linear `v -= mA P n` | `P n` first: P_s x n (Q1.30) >> 30 -> int32 in units 2^eP; then `m_m x (P n)` >> shMA | 2^62 | int64; add to v with sat |
| apply angular `w -= iA (r x P)` | `r x P`: 2^29 x 2^31 = 2^60 per product, difference 2^61, >> 31 -> int32 (static shift; V4b in E11 tries a clz normalisation); `I (…)`: 3 products 2^60, sum 2^61.6, >> shIA | 2^62 | int64; add to w with sat |
| friction | `vt` like vn; tangent 2x2 solve = 4 products; cone clamp `|P|^2 > (mu Pn)^2`: squares of int32 impulses 2^62, sum 2^63 (pre-shift by 1); scale = mu Pn / |P| by `lpRsqrtQ31` (Newton, no division) | | int64 |
| twist and rolling | like friction with the 3x3 `rollingMass` applied as 9 products | | int64 |
| restitution | `armed` compares; `allowance` sums of impulses in the same e_P | 2^33 | int64, sat |
| warm start | same products as apply | | |
| integrate velocities | `v = d (v + h m f + h g)`: force in Q?: accept forces as impulses per step from the CPU (Q e_P per body), so `h m f` is one shifted product; damping d Q1.30 | 2^62 | int64 |
| gyroscopic step | `I w`, `w x (I w)` with I normalised, Jacobian entries and a 3x3 elimination | products 2^60, sums 2^62 | int64; two normalised 64/32 divisions and one back-substitution; pivot below 2^-20 of the diagonal: skip (Box3D skips at `det < 1000 FLT_MIN`, `revolute_joint.c:282`) |
| integrate positions | `dp += h v` (Q0.32 x Q9.22 >> 28), `q += 0.5 h w q` | 2^63 (pre-shift v by 1) | int64; dp sat; quaternion components sat to Q2.30 |
| quaternion normalise | sum of four squares of Q2.30 with |q| <= 1.13 (w <= 64 rad/s) or 1.53 (256 rad/s) | up to 2^63.2 | squares pre-shifted by 2; exact `isqrt64`; one 64/32 division for the reciprocal; four products |
| world inertia `R I R^T` | 27 products of Q1.30 by normalised mantissas, sums of 3 | 2^62.6 | int64; narrow by >> 30 twice with the two headroom bits |
| effective mass `kNormal` | four terms aligned to the largest of e_mA, e_mB, e_IA + 2 e_r, e_IB + 2 e_r | 2^63 at worst | int64 with a 2-bit pre-shift; normalise; reciprocal by `2^62 / mantissa` (64/32, quotient < 2^32) |
| joints (weld, revolute, spherical, distance) | same primitives; angles from `b3GetTwistAngle` via an integer atan2 (CORDIC or a Q1.30 minimax polynomial, deterministic, 1e-6 rad); rope length via `isqrt64` of a 64-bit squared distance of Q8.24 differences (ropes under 128 m) | | |
| motors and limits | `maxImpulse = maxMotorTorque h` folded on the CPU into the joint's e_P; clamps are int32 compares | | |
| broadphase (sort-based) | Morton or axis keys from AABB minima in 2^-10 m units; overlap tests are compares; SAH-style costs (if a tree is kept on the CPU) use extents >> 8 so that products stay under 2^62 | | uint64 keys |
| narrowphase: hull SAT face query | `dot(plane.n, v) + d` with n Q1.30, v Q8.24 | 2^61 | int64; separation narrowed to Q9.22 |
| narrowphase: edge query | edge cross products of Q8.24 edges (2^30 x 2^30 = 2^60 per product, difference 2^61), normalised to 31 bits by clz before the squared-length and the dot | | int64; Gauss-map tests are sign tests of such dots |
| clipping | `t = d1 / (d1 - d2)` with d1, d2 int64: both normalised to 31 bits, one 64/32 division to Q0.31; `a + t (b - a)` one product per component | | int64; vertices narrowed to Q8.24 |
| manifold reduction | polygon areas of Q8.24 points: products 2^60, sums of 4 | 2^62 | int64 |
| GJK (CPU queries and casts) | simplex barycentric solves divide by `dot(ab, ab)`: normalise numerator and divisor to 31 bits, one 64/32 division (`distance.c:223` in float) | | int64 |
| pair sort, colouring, compaction | radix sort on uint64 keys; Jones-Plassmann priorities = `lpMix64(keyA, keyB)`; prefix sums in uint32 | | exact |

Integer division and square root: `/` on nonzero divisors is exact and truncates toward zero on both sides, so any
hardware or compiler emulation agrees; 64-bit division is emulated on every GPU (tens to a hundred instructions,
unverified), so it is confined to prepare and to integrate (about 20k per tick at 5k bodies). The hot loops use
Newton iterations with a fixed count in Q0.31 (`lpRecipQ31`: y = y(2 - x y), five steps; `lpRsqrtQ31`:
y = y(3 - x y^2)/2, five steps, seeded from clz), which are deterministic and not exact, which is all a scale factor
needs. Where an exact result matters (the floor square root of the squared quaternion norm and of squared rope
lengths) `isqrt64` is Newton from a clz seed followed by a bounded fix-up (at most two compares), giving the unique
floor root; it uses 64/32 divisions and runs once per body per substep. Seeding from a hardware float sqrt (fixed3d's
trick) is banned in kernels: it would be sound because the fix-up makes the result exact regardless of the seed, but
the lint's "no float tokens" law is worth more than the few instructions.

## 3. The floor GPU (question 3)

### 3.1 Per-vendor integer throughput

Throughput is per SM (NVIDIA), per CU (AMD), per EU (Intel) or per core (Apple), per clock, unless the source gives
cycles per instruction. FP32 FMA is the reference column.

| architecture (example) | FP32 FMA | INT32 add | INT32 multiply (low) | 32x32->64 | INT64 add | 64x64->64 | source and status |
|---|---|---|---|---|---|---|---|
| NVIDIA Maxwell/Pascal (GTX 1060, GP106, cc 6.1) | 128 | 128 | no multiplier: 3 XMAD (16x16+32) at full rate, about 43/clk | 7 instructions for signed (XMADs plus IADD3), about 18/clk | 2 (IADD with carry) | 15 instructions, about 8.5/clk | SASS counts on sm_52 from allanmac's gist; Maxwell and Pascal share one ISA listing in NVIDIA's binary-utilities doc (XMAD present, absent on Volta); njuffa on XMAD throughput |
| NVIDIA Volta/Turing (RTX 2080, TU104, 7.x) | 64 | 64 (separate INT32 cores) | 64 (IMAD) | IMAD.WIDE, about 1-2 instructions (table: "extended-precision multiply-add" 64/clk, recalled) | 2 | about 4 | Volta and Turing tuning guides (64 FP32 + 64 INT32 cores per SM) |
| NVIDIA Ampere GA10x (RTX 3060, 8.6), Ada (8.9) | 128 | 64 | 64 | as Turing | 2 | about 4 | Ampere tuning guide ("2x more FP32 operations per cycle per SM than 8.0"); INT32 lanes 64 (recalled from the CUDA throughput table; the page could not be fetched) |
| NVIDIA consumer Blackwell (12.x) | 128 | 128 (unified lanes, unverified) | 128 | | | | T4's note; unverified |
| AMD RDNA 3 (RX 7000) | 64 (128 with VOPD dual issue) | 64 | about a quarter of FP32 rate, high latency; VOPD rarely applies to integer ops | `v_mad_u64_u32` quarter rate (recalled, unverified) | 2 | 4 quarter-rate ops | Chips and Cheese RDNA 3 microbenchmarks |
| AMD GCN (Vega, Steam Deck's RDNA 2 similar) | 64 | 64 | quarter rate (`v_mul_lo_u32`, recalled) | quarter rate | 2 | | unverified: the RDNA/GCN ISA guides list rates |
| Intel Gen9 (the laptop's UHD; UHD 620 class) | 8 per EU (2 SIMD-4 FPUs) | 8 | unverified | int64 supported (`shaderInt64 = true`), rate unverified; no 64-bit atomics | | | vulkaninfo on this machine; the Gen9 whitepaper is a PDF this session could not read; E11 measures |
| Intel Xe-HPG (Arc A770) | 128 per Xe-core | about 75-80 % of FP32 | "much lower" than add | | | | Chips and Cheese Arc A770 (via T4) |
| Apple M1/M2 (cycles per instruction, lower is faster) | 1 | 1 | 4 | 8 | 4.7 | 16 | philipturner/metal-benchmarks (shared multiplier across the four INT/FP32 pipes) |
| x64 Skylake core (i5-6500) | `mulps` 2/clk (8 lanes) | `vpaddq` 3/clk (12 lanes) | `vpmulld` 1/clk | `vpmuldq ymm` 2/clk = 8 products/clk; `pmuldq xmm` 2/clk (Skylake+), 1/clk on Sandy Bridge and Haswell; scalar `mul r64` 1/clk | 12 lanes/clk | `imul r64` 1/clk, latency 3 | uops.info |
| Apple M1 core (ARM64) | `fmul` 4/clk | | | `smull` 2 lanes; scalar `mul` and `umulh` 2/clk | | | dougallj tables (via T4) |

### 3.2 Pascal in detail

Maxwell and Pascal dropped the 32-bit integer multiplier; XMAD reuses the FP32 multiplier for 16x16 products with a
32-bit accumulate, so every integer multiply is a short sequence at full rate. Primary counts (sm_52; Pascal shares
the instruction set): a low 32x32 multiply is 3 XMAD, a signed 32x32->64 is 7 instructions, a 64x64->64 is 15, a
32x32+64 wide multiply-add is 10. On Kepler these were 1, 2, 4 and 2 instructions at a quarter rate; njuffa's
summary is that the XMAD sequences run at "roughly the same throughput" as Kepler's quarter-rate IMAD. Volta restored
a full-rate IMAD on separate INT32 cores.

Is exact integer arithmetic on the FP32 units a workaround? It is sound: a float add or multiply whose exact result
is an integer below 2^24 is exact under any rounding mode, contraction cannot change an exact result, and integers
are never subnormal, so 12-bit limbs in FP32 give bit-exact products on any IEEE unit. But on Pascal it buys nothing:
XMAD already is the FP32 multiplier used as a 16x16 integer unit at full rate, and a 32x32->64 through 12-bit limbs
is nine FFMAs plus carries against seven XMAD-class instructions. It is worth measuring on vendors whose integer
multiply is a quarter rate or worse (AMD RDNA, Apple, possibly Intel), where nine full-rate FFMAs beat four
quarter-rate multiplies; E11 gets an optional fifth micro-variant for it (section 10). The 26-bit-limb FP64 version is
out: consumer NVIDIA runs FP64 at 1/32.

### 3.3 The floor machine, in numbers

Per contact point per solver row (push or relax), counting Box3D's `b3PushContacts_Mesh` body
(`contact_solver.c:449-500`): two anchor rotations (18 multiplies each), four cross products (24), three dots (9),
two 3x3 matrix-vector products (18), scalars (8): about 95 multiplies and 85 adds in float, of which many fuse on
hardware that fuses. The block-scaled integer row is 95 wide products, about 60 narrowing shifts, 85 64-bit adds
(170 32-bit ops) and about 10 variable shifts: about 335 integer instructions on hardware with a one-instruction
wide multiply.

| GPU | float row (issue slots per SM-clock) | integer row | ratio | notes |
|---|---|---|---|---|
| GTX 1060 (Pascal) | 180 / 128 = 1.4 | (95 x 7 + 240) / 128 = 7.1 | about 5x | XMAD sequences; everything at 128 lanes |
| RTX 2080 (Turing) | 180 / 64 = 2.8 | (95 x 1.5 + 240) / 64 = 6.0 | about 2x | INT32 and FP32 cores are separate, so an integer kernel leaves the FP32 cores idle |
| RTX 3060 (Ampere) | 180 / 128 = 1.4 | 382 / 64 = 6.0 | about 4x | half the lanes take INT32 |
| RDNA 3 | 180 / 64 (128 dual-issued) = 2.8 (1.4) | (95 x 4 + 240) / 64 = 9.7 | 3.5-7x | quarter-rate multiply |
| Apple M-series | about 180 | 95 x 8 + 240 = 1,000 | about 5.5x | cycles per instruction from metal-benchmarks |

These are issue-slot ratios; the rows are gather-bound (about 250 B of body state and constraint data per row), so
realised ratios will be smaller. Gate G1 measures the real one on the 3060 and the UHD; a Pascal card must be bought
or borrowed for G4 (a used GTX 1060 costs little; the laptop's Ampere and the UHD bracket it).

Throughput of a GTX 1060 6GB (10 SMs, 1,280 lanes, 1,708 MHz boost, 3.9 TFLOPS FP32): 2.2 T 32-bit integer
instructions per second, about 0.3 T signed 32x32->64 products per second through XMAD, 1.1 T 64-bit adds. The old
quad-core beside it (an i5-6500: four Skylake cores, 3.2-3.6 GHz, AVX2): 4 x 3.3 GHz x 8 = 0.1 T 32x32->64 products
per second at peak from `vpmuldq`, 13 G per second scalar; 32-bit adds 0.3 T per second. So the GPU has 3-4x the
CPU's SIMD peak on the wide products that dominate, 20x on adds, and 25x its scalar rate. Against Box3D's float SSE2
solver on that CPU (8 lane-multiplies per clock per core, half of them useful in a 4-wide AoSoA solver) the integer
GPU solver is an estimated 3-6x faster on the solve stage at 5,000 awake bodies, not the 30x of yacine's CUDA claim,
and it runs while the CPU does fracture and stress. Estimated per-tick GPU time on the 1060 at 5,000 bodies and
15,000 contacts: solve and relax 30k rows x 900 slots x 2 x 4 substeps = 216 M slots, narrowphase 15k pairs x about
2,000 = 30 M, sorts and prefix sums 0.2 ms, about 220 dispatches in one command buffer at 1-3 us each (unverified for
Vulkan on NVIDIA), readback 0.5 MB: 1.5-2.5 ms at 30 % realised throughput (unverified; G4). The RTX 3060 Laptop
(30 SMs x 64 INT32 lanes x 1.4 GHz = 2.7 T integer instructions per second) is a stand-in for about half an
RTX 2080 (46 x 64 x 1.7 GHz = 5.0 T), which makes the laptop a usable proxy for the dream target.

## 4. The CPU side (question 4)

SIMD paths for the twin. Because integer lanes are exact and independent, any lane width produces the same bits as
the scalar loop, so the twin can pick SSE4.1, AVX2 or NEON at run time without a determinism review; the lint only
has to forbid horizontal reductions whose order changes results, and integer reductions are order-free anyway.

| path | 32x32->64 products per clock per core | variable per-lane shifts | 64-bit add lanes | expected solve-stage speed vs Box3D's 4-wide SSE2 float solver | notes |
|---|---|---|---|---|---|
| scalar int64 (`imul r64`, `mul r64`) | 1 | native | 4 | 4-5x slower (335 ops at about 3 IPC = 110 cycles per row vs about 23) | the reference twin, what tests run |
| SSE4.1 `pmuldq` (signed, 2 lanes of the even elements) | 2-4 (2 on Sandy Bridge and Haswell, 4 on Skylake+) | none per lane in SSE (uniform counts only): needs constraints sorted by shift class within a colour, or scalar fallback for the shift | 2 lanes x 2 | about 2x slower | SSE2 has only `pmuludq` (unsigned): a sign fix-up of 4 ops per product |
| AVX2 `vpmuldq ymm` | 8 (Skylake+, Zen 3+; 4 on Zen 2) | `vpsravd`/`vpsrlvq` per lane (0.5 cycles); 64-bit arithmetic variable shift is AVX-512 only (`vpsravq`), emulated in 3 ops | 4 lanes x 3 | about 1.3-1.6x slower (about 35 cycles per row) | the floor CPU has AVX2 (Haswell 2013 onwards); older quad-cores fall to SSE4.1 |
| AVX-512 IFMA (52-bit limbs) | 8 lanes of 52x52->104 | | | not on the floor or the laptop (Comet Lake has no AVX-512) | ignore |
| NEON `smull`/`smull2` | 2 lanes x 2 per clock on Apple | `sshl` by a per-lane vector (native) | 2 x 2 | about 2x slower | ARM64 gets determinism for free, the cross-play requirement of goals.md |

Whole-physics-stage ratios, taking the solve at 40-50 % of the stage (fixed3d's profile: contact solve 24 %, anchor
rotation 15 %, prepare 14 %, warm start 11 %): scalar twin about 2.5-3x Box3D, AVX2 twin about 1.3-1.6x (estimates;
G2 measures). Since the shipping path at the floor runs the core on the GPU, the twin's speed matters for headless
agents, tests and the GPU-less fallback; the gate is set so that the town rung stays under one frame at one worker.

fixed3d: mine, do not vendor. It is Q48.16 in int64 with 128-bit products (round-to-nearest, `FIX_SATURATE`
optional), a 47-bit fast path for division, an exact `isqrt128` seeded by a hardware double sqrt, a Q2.30 packed
quaternion, and a 40-bit inverse scale that its README calls essential for large bodies to respond to impulses; it
passes Box3D's 24 suites at 2.24x float scalar (2.13x with NEON) on an M3 Ultra at 4 workers (README, 2026-09-07),
tracks Box3D 47d7f7c against our 5643cd81, has no GPU path and no x64 numbers. Its value to us is as the worked
example of every tolerance Box3D hides (which constants had to change, where a squared length lost its sign, how the
sleep and TOI thresholds were re-expressed), its exact sqrt and division kernels as reference implementations for
our `isqrt64` tests, and its quaternion format, which this design adopts. Vendoring it would double the vendored
surface for a format we are not using and a commit we are not on.

## 5. Solver shape (question 5)

| criterion | Box3D soft step in integers (constraint-coloured Gauss-Seidel, 4 substeps, 1 solve + 1 relax) | AVBD in integers (body-coloured block descent with augmented Lagrangian) |
|---|---|---|
| stability at our counts | what we ship today; stacks and joints stable at 4 substeps with warm starting (Solver2D's conclusion, T5 §2) | unconditionally stable at any iteration count; 3-10 iterations in the paper's demos; mass ratios to 1:2000 published, our 1:60,000 unverified |
| joints and motors | weld, revolute, spherical, distance with limits and motors exist in Box3D; port them | joints with limited degrees of freedom are in the paper; motors are not discussed (unverified); the demo's `Force` interface clamps `lambda` in `[fmin, fmax]`, which is where a motor's torque cap would go |
| integer fit | every quantity above has a static format or a per-body/per-manifold exponent; the per-iteration work is products and one variable shift | `lambda` and the penalty `k` (updated by `k += beta |C|` up to the material stiffness, decayed by `gamma` when warm-started) span many decades and need per-constraint exponents; the per-body 6x6 SPD solve (LDL, 6 pivots) in integers is about 300-400 ops with Newton reciprocals and a guard against a non-positive pivot from floor rounding; the lumped geometric term needs absolute column norms |
| GPU mapping | one dispatch per colour per stage: integrate velocities, C warm starts, C solves, integrate positions, C relaxes = 3C+2 per substep; C about 12-20 with per-tick Jones-Plassmann colouring; about 220 dispatches per tick; the uncoloured remainder solved by Jacobi with mass splitting (Tonge 2012), which is order-free with integer atomics | one dispatch per body colour per iteration (about 8-15 colours) plus one dual-update dispatch; at 10 iterations about 120-160 dispatches; the per-body gather reads every constraint of the body |
| determinism | within a colour no two constraints touch a body (no atomics); colours from state each tick (priorities `lpMix64` of pair keys, a fixed number of rounds); pair lists sorted; prefix sums for compaction; contact warm starts carried by a merge-join of consecutive ticks' sorted pair arrays | same colouring machinery on bodies; per-body gathers in index order |
| the CPU twin | a transcription of `contact_solver.c`, `solver.c` and the four joint files into the shared subset: known code, known behaviour, our tests and hashes carry over with tolerances | a new 3D solver with no reference implementation beyond the paper and the 2D demo; retunes every material, the stress loads (impulses become forces), the feel tests |
| what it buys | the shipped behaviour on a GPU | a budget knob (iterations) that degrades gracefully, and possibly fewer dispatches |

Recommendation: the soft step first, because it is the engine we have and its twin is a port. AVBD is a stage-3
experiment on the same buffers, colouring and harness once the integer infrastructure exists; if the dream scale
turns out to be dispatch-bound rather than throughput-bound, that is when a body-coloured solver earns its retuning.
One change from Box3D either way: colouring is recomputed from state every tick, so it leaves the snapshot, and the
overflow colour is Jacobi rather than serial (Box3D solves it on the main thread, `contact_solver.c:2273-2307`).

## 6. The boundary (question 6)

Integer, and on the GPU with a CPU twin: bodies, shapes (hulls in Q8.24), broadphase (sorted keys), narrowphase (hull
SAT, clipping, manifolds with feature ids), colouring, the soft step with its four joint types, motors and limits,
restitution, integration, sleeping (island labels by fixed-round min-label propagation on the contact graph, or
union-find on the CPU from the readback: both deterministic), the speed caps. CPU-only integer (in the shared subset,
no GPU twin needed): GJK distance, ray casts, AABB overlaps and rounded shape casts against a CPU tree, one tick
behind; the debris ghosts' own integrator; supply and pool levels.

Float, on the CPU, under T3's dialect (no FMA, no libm beyond `sqrtf`, Box3D's `b3ComputeCosSin` and `b3Atan2`
copied into a small float math header, `/fp:precise`, `-ffp-contract=off` for GCC as well, the MXCSR/FPCR guard, the
self-test vector): the stress solve (double, order-fixed CG with its 12-decade stiffness range, T4 §1), the judging
of bonds and slender pieces, link decisions, wheels' tyre solves, rigs, IK and the gait. These read integer state
converted to float (exact for mantissas under 24 bits, else IEEE RNE, identical everywhere) and write integer inputs
by clamped truncation, which is IEEE-defined. Their determinism therefore rests on T3's dialect and on H1, exactly as
today; what changes is that the physics no longer contributes a hazard, and that the fracture is exact integer
geometry feeding integer hulls with no quickhull float round trip (T4 §4's from-topology hull builder becomes the
only builder).

Where quantisation happens, once each: hull vertices to Q8.24 at piece creation from exact rational vertices; masses
and tensors to mantissas and exponents at creation from exact integers; scene positions to Q32.32 at build time;
impulses, pulls and blasts to the body's e_P-free per-step force impulse (Q8.23 N s, an int32 per axis, saturating)
when the CPU queues them; motor targets and speeds to Q3.28 and Q8.23; material coefficients to Q1.30 at material
registration. Readback converts positions to double relative to the zone origin (Q32.32 does not fit a double at
large coordinates; a zone-relative double does) and velocities and impulses to float.

The pipeline runs one tick behind. CPU(N) reads the results of physics(N-1), does its branchy work, and writes the
inputs of physics(N) (created and destroyed bodies, type flips, impulses, motor targets, joint edits); the GPU then
runs physics(N) while CPU(N+1) already starts, reading physics(N-1) again for the work that does not need newer
state (rendering, input, network, fracture jobs queued from the last tick's hits) and waiting on the timeline
semaphore for physics(N) only where it first needs it. In the fully pipelined form CPU(N+1) consumes physics(N-1)
throughout and writes the inputs of physics(N+1), so that fracture and stress overlap the GPU completely; decisions
are then based on state two physics ticks older than the tick they affect. T5 §4 already established that every reactive system of
ours tolerates a tick of lag (hits are queued to the next step today, stress runs late by design, freezing is a
many-step signal, links settle for 3 steps). The CPU twin double-buffers its visible state so that game code sees
the identical sequence whether the core ran on the GPU or on the CPU: the lag is part of the deterministic
semantics, not an artefact of the GPU path.

## 7. Kernel language and API (question 7)

Slang. Targets: D3D11 (DXBC), D3D12 (DXIL), Vulkan (SPIR-V), OpenGL (GLSL), Metal, CUDA/OptiX (PTX), WebGPU (WGSL)
and CPU. The CPU target emits C++ compiled by MSVC, Clang or GCC, or JIT-compiled by `slang-llvm`; the docs call it
"preliminary", "functional, but not feature- or performance-complete", warn that "backwards-incompatible changes to
this target may come", and list as unsupported: barriers ("would require an ABI change"), atomics in user code,
`groupshared`, `float16_t`, and debugging into the JIT. 64-bit integer types and intrinsics: Vulkan yes, CUDA yes,
Metal yes, CPU yes, D3D12 yes with DXIL (SM 6.0), D3D11 no. So Slang's CPU output could be a twin for straight-line
kernels (our solver kernels have no barriers or groupshared), but agents would then read generated C++ with threaded
context parameters, and the Eva and CBMC proofs would run on generated code that may change shape with each Slang
release. `slangc` 2026.13.1 ships in the Vulkan SDK on this machine.

The shared subset. Kernels are written in the intersection of C17 and HLSL 2021: `int`, `uint`, `int64_t`,
`uint64_t`, structs, functions, `if`, `for` with constant bounds, no pointers, no recursion, no floats, buffers as
arrays. A 60-line shim header maps `StructuredBuffer<T>` to `const T*`, `RWStructuredBuffer<T>` to `T*`,
`[numthreads(64,1,1)] void main(uint3 id : SV_DispatchThreadID)` to `void main(lpArgs*, uint32_t index)`, and the
helpers (`lpMul64`, `lpShr`, `lpSat32`, `lpMsb64`, `lpDivU64`, `lpIsqrt64`, `lpRecipQ31`, `lpRsqrtQ31`) exist
twice with the same signatures and the same code where the syntax allows and a tested pair where it does not (clz,
the 64-bit shift emulation on D3D12). The twin runs each kernel as a loop over indices, optionally split across our
task pool by contiguous ranges: no lane-crossing, so any partition gives the same bits. dxc from the Vulkan SDK
compiles the same file to SPIR-V (`-spirv`, `Int64` capability) and to DXIL for D3D12 later; glslang is the fallback
if dxc's SPIR-V back end misbehaves on a driver (then GLSL with `GL_EXT_shader_explicit_arithmetic_types_int64` and
`imulExtended`, a second shim). Metal later through SPIRV-Cross or Slang (both in the SDK); CUDA is not needed,
Vulkan runs on the DGX Spark. Agent-friendliness: one readable source per kernel, plain C on the CPU, two compilers
we already have, a lint that is a script over the kernel directory, and no second language beyond the HLSL
attributes the shim hides.

The API. Vulkan compute first: it covers NVIDIA, AMD and Intel on Windows and Linux (D3D12 is Windows only, Metal is
Apple only), it queries the features we depend on (`shaderInt64`, `timelineSemaphore`, `bufferDeviceAddress`; both
GPUs here report all three; only the RTX 3060 reports 64-bit buffer atomics), it can run headless (a compute-only
device with no swapchain, so the tests that do have a GPU can use it), its validation layers and SPIR-V tools are on
the machine, and SPIR-V is an IR we can hash and ship (the driver still compiles it: section 8). The sandbox keeps
its D3D11 renderer; transforms return to the CPU anyway. D3D12 comes second (same HLSL, DXIL, `Int64ShaderOps`), Metal
third.

Per tick: the CPU writes the delta list (created bodies with hulls appended to an immutable hull arena, destroyed
slots, type flips, impulses, joint edits) into a host-visible staging ring (three in flight); records one command
buffer: copies, then the dispatch graph with barriers (broadphase sort, pair merge-join for warm starts, narrowphase,
colouring rounds, prepare, 4 x (integrate, C warm, C solve, integrate, C relax), restitution, store, sleep labels,
compaction, result copies); submits with timeline value N. Results (transforms, velocities, per-body impulse totals,
hit events above threshold sorted by pair key, joint forces, saturation and clamp counters) land in a host-cached
readback ring; CPU(N+1) waits for value N-1, invalidates the range, and reads. Body slots are a CPU free list
(deterministic from the command log); capacity growth is a copy command at a tick boundary. Contacts are not
persistent objects: each tick's sorted pair array is merge-joined with the previous tick's to carry warm-start
impulses and feature ids, which replaces Box3D's pair hash set and contact ids and takes three items off the state
audit's hidden-state list.

## 8. Verification (question 8)

What is proven:

- Absence of undefined behaviour in the twin. Frama-C Eva computes value intervals through each kernel from an
  entry point whose inputs are constrained by ACSL `requires` (exponent ranges, |r| components <= 2^29, velocity
  formats), with the kernel's constant-bound loops unrolled (`-eva-slevel`); it emits alarms for signed overflow
  (`-warn-signed-overflow`, default on), shifts of negative values (`-warn-left-shift-negative`,
  `-warn-right-shift-negative`), downcasts (`-warn-signed-downcast`, `-warn-unsigned-downcast`), division by zero
  and out-of-bounds accesses, and if no alarm is emitted the operation "is guaranteed not to cause a run-time error".
  Every alarm is discharged by tightening a contract or by inserting a saturating helper, never by widening a
  type. Unverified: Eva's handling of `__int128` (the design avoids it: the twin's only 128-bit code is in the CPU
  fracture predicates, outside the kernels) and Frama-C on Windows (it is an OCaml tool; WSL or a Linux CI box).
- Single-row equivalence and bound properties by bounded model checking: CBMC with `--signed-overflow-check`,
  `--undefined-shift-check`, `--div-by-zero-check`, `--conversion-check`, `--bounds-check` and `--unwind` on one
  contact row or one joint row over all inputs satisfying the contracts, with the SMT back end (`--bitwuzla` or
  `--z3`) because bit-blasted 64-bit multiplier chains are hard for SAT (unverified how far it scales; the row has
  about 100 multiplies). CBMC also checks that the two implementations of a helper (the C one and a C model of the
  HLSL one, e.g. the clz zero case) agree.
- The invariants that make the contracts true: every writer of a formatted quantity saturates into its format, so
  the contracts hold by induction over ticks. This is a lint property (all narrowing through helpers), not a proof
  obligation per kernel.

What is tested, because it cannot be proven from our side:

- Compiler and driver correctness. The same integer program compiled by MSVC, clang, dxc-to-SPIR-V on the NVIDIA and
  Intel drivers, and later DXIL and MSL, must agree on every input; differential tests run each kernel over corpora
  of 2^20 rows with edge classes (format limits, zero divisors excluded by contract, saturation triggers, negative
  operands to `/`, `%` and `>>`) on every machine with a GPU in CI. Driver updates re-run them.
- The startup battery. A canned vector (4,096 rows per kernel, plus the helper tables: shifts of negatives, division
  signs, clz of zero, `isqrt64` at 2^62-1) runs on the GPU and the twin at startup; both hashes must equal the
  build's golden hash. The session handshake exchanges (build id, T3's float self-test hash, the kernel battery hash,
  the tick-0 world hash). A peer whose GPU fails the battery runs the twin, which is bit-identical, so the session
  never depends on anyone's GPU; a peer that fails the float self-test is refused. Per-object hashes and host repair
  (T1's convergent lockstep) remain the backstop for whatever escapes.
- Physics quality: the outcome tests of goals.md with tolerances, and the bench hashes as the golden end-to-end check
  once the core is in.
- UBSan: clang `-fsanitize=undefined,integer` on the twin in CI (with `unsigned-integer-overflow` exempted inside the
  hash and counter helpers), MSVC has none; clang-cl on this machine is unverified (the wave-1 harness produced an
  `m7h_clang.exe`, but `clang-cl` is not on PATH).

Lint laws, a script over the kernel directory, run in CI: no `float`, `double`, `half` or `precise` tokens; no
`sqrt`, `rsqrt`, `sin`, `cos`, `atan`, `exp`, `log`, `pow`, `fma`, `mad` (no libm, no float atomics); `/` and `%`
only inside `lpDiv*` helpers whose divisor argument is a nonzero-proven value (a type wrapper the lint recognises);
`>>` on signed operands only inside `lpShr*`; shift counts constants or wrapped by `lpClampShift`; every cast to a
narrower type through `lpSat*` or `lpTrunc*`; no `Wave*`/subgroup operations except integer `WaveActiveSum` and
`WaveActiveMax` (exact); no `groupshared` in solver kernels (allowed in sorts and prefix sums, which are exact); no
recursion; loop bounds constants; no `InterlockedAdd` on anything but the declared integer accumulators.

## 9. Cost model and build plan (question 9)

| stage | work | gate | agent-weeks |
|---|---|---|---|
| 0 (E11, running) | the Vulkan harness; V1 float dialect, V2 Q32.32 int64, V3 scaled int32, plus V4 from section 10 | G0: integer variants hash-identical on the 3060, the UHD and the twin; V4 within 3x of V1 per row on the 3060; NVIDIA strength-reduces sign-extended multiplies (timing) | in progress |
| 1 | `lpfix.h` (the shared-subset helpers and formats), the lint, Eva and CBMC harnesses for the helpers, the startup battery; the contact row (prepare, warm start, push, solve, relax, restitution, store) in the subset | G1: the contact kernel bit-exact on both GPUs and the twin over 2^24 rows including the chip-vs-mech and car corpora; within 3x of float per row on the 3060 (5x expected on Pascal); accuracy against a float64 reference no worse than the float variant's | 3 |
| 2 | the integer CPU core: bodies, hulls, sorted broadphase, SAT narrowphase, colouring, the soft step with four joints, motors and limits, islands and sleep, GJK and casts for queries; the fracture's exact geometry feeding integer hulls (T4-B, which is needed anyway) | G2: our nine suites' outcomes pass with tolerances; `lpf_bench` town at 1 worker within 2.5x of Box3D (1.5x with the AVX2 path); identical hashes across MSVC, clang, worker counts and ARM64 if available | 8-12 |
| 3 | the Vulkan runtime (device, rings, command graph, readback), every kernel from the same sources, differential CPU/GPU tests per kernel, the battery in the handshake | G3: a full island solve (broadphase to store) bit-exact against the twin for 600 town ticks on the 3060 and the UHD; a driver update re-run | 4-6 |
| 4 | the pipeline one tick behind in `lpWorld_Step` (twin double-buffering), body streams, integer-atomic stress loads, sleeping and freezing under lag, a Pascal card on the bench | G4: at 5,000 awake bodies the GPU pipeline beats the twin on a GTX 1060-class card with the whole step under 16.7 ms at one CPU worker; on the 3060 (half an RTX 2080) 20,000 awake bodies under 8 ms of GPU time | 4-6 |
| 5 | replace Box3D: delete the vendored tree and our patches, re-baseline every bench hash and outcome test, the snapshot primitive over integer state, perf-log entries | none; the milestone ends when `tools/bench.ps1` and `lpf_test` are green on the new core | 4-6 |

Total 23-33 agent-weeks. Effort calibration: fixed3d converted all of Box3D in about three months of one engineer with
an AI collaborator (303 commits); we convert a hull-only subset (about 45k of its 69.8k lines) into an estimated
15-20k lines of the shared subset, but also write a GPU runtime and the proofs.

What we give up: Box3D's upstream cadence and Erin Catto's coming features (meshes, height fields, compounds, the
character mover, sensors, its record/replay tooling), our patches, its 4-wide float SIMD solver and its SIMD SAT, the
float precision floor near zero (a resting chip's velocity is quantised to 2.4e-7 m/s; float keeps 1e-9), full CCD
(bullets against statics keep speculative contacts and the speed cap; TOI sub-stepping is dropped in the first cut:
"plausible over causal"), and simplicity: every formula carries a range contract, the compiler checks none of them,
and the lint and Eva must. What we gain beyond the GPU: bit-exactness across x64, ARM64, every emulator, WASM and
every compiler with no flags to police in the core; integer SIMD paths that need no scalar twin; snapshots that are
plain integers; three hidden-state items gone; and a fracture pipeline that never rounds through float.

Risks not priced above: driver bugs on integer code paths (Bullet's OpenCL history), which the battery detects but
cannot fix; Intel's Windows Vulkan driver performance (sokol's changelog notes a bug, T5); the number of colours a
rubble pile needs and how much lands in the Jacobi remainder (measure on the town and keep rungs in stage 2 on the
CPU, before any GPU work); AVX2 exponent handling if the twin's speed gate fails scalar.

## 10. The fourth E11 variant (V4): block-scaled int32

Name: `V4 blockscaled`. Purpose: price the format of section 1 against V1 (float dialect), V2 (Q32.32 int64,
128-bit products) and V3 (one global scale per quantity, 64-bit products) on the same rows and the same harness, and
verify it hashes identically on the RTX 3060, the Intel UHD and the C twin.

Row layout (all little-endian int32 unless noted; one row = one manifold point of one contact):

- body A and B: `v[3]` Q9.22, `w[3]` Q8.23, `dq[4]` Q2.30, `dp[3]` Q5.26, `invMassM` (mantissa, [2^30, 2^31) or 0),
  `eM` (int8), `invI[9]` (mantissas), `eI` (int8), `flags` (dynamic bit);
- constraint: `rA[3]`, `rB[3]` Q8.24 with |component| <= 2^29, `n[3]` Q1.30, `baseSep` Q9.22, `normalMassM`, `eN`
  (int8), `impulse` (int32 in units 2^eP, eP = eN + 10), `massScale`, `impulseScale` Q1.30, `biasRate` Q8.24,
  `contactSpeed` Q9.22, `invH` = 240, `shMA, shMB, shIA, shIB` (uint8, computed by the CPU prepare from the
  exponents as in section 1.3);
- the same row in float for V1 (the CPU converts once) and in V2/V3 formats.

Kernel: exactly `b3PushContacts_Mesh`'s point body (`contact_solver.c:449-500`) followed by
`b3SolveContacts_Mesh`'s normal part without bias (`:571-610`): rotate anchors, separation, speculative or soft
bias, relative normal velocity, `deltaImpulse`, clamp, apply to both bodies; then the relax row on the updated
velocities. Outputs per row: `vA, wA, vB, wB` (int32 x 12), `impulse`, and two uint32 counters accumulated with
`InterlockedAdd`: saturations and shift clamps. V4b: the same with clz normalisation of `r x P` and of `vn` instead
of static shifts (costs about 8 ops per vector, keeps 31 bits).

Corpus: 2^20 rows seeded by PCG32 from a fixed seed: a quarter "town" (masses 1-100 kg, |r| <= 0.5 m, speeds under
20 m/s), a quarter "chip against mech" (0.05 kg, inertia 6.3e-6, against 3,000 kg, inertia 7,000, both orders), a
quarter "car crash" (1,700 kg at 20 m/s into a static, impulses reaching 34,000 N s, relax rows after a large push),
a quarter adversarial (every input at a format limit: |r| = 2^29 units, v = ±2^31, w = ±2^31, s = ±2^31, `normalMassM`
at both exponent extremes, `impulse` near ±2^31, `dq` unnormalised by up to 13 %), plus 1,024 hand-written rows
(zeros, statics, exact ties in the clamp, values that make `deltaImpulse` exactly 0).

Report: (1) the 64-bit hash of all outputs and counters on the 3060, the UHD, the MSVC twin and (if present) the
clang twin: all four equal or the variant fails; (2) ns per row at 2^20 rows for V1-V4 and V4b on each GPU, with
validation layers off, plus the ratio to V1; (3) accuracy against a float64 reference of the same row: max and RMS
relative error of the velocity deltas for V1, V3, V4, V4b, and the count of rows where V4's clamp decision
(`newImpulse == 0`) differs from the reference's; (4) the counters; (5) a multiply micro-test: `int64_t(a) *
int64_t(b)` with sign-extended 32-bit operands versus a full 64x64 multiply versus GLSL `imulExtended`, timed on both
GPUs, to see whether the drivers strength-reduce; (6) the kernel compiled from HLSL by dxc and from GLSL by glslang:
hashes must match; (7) optional V5: the same row on FP32 units with 12-bit limbs (exact integer arithmetic in
floats), timed only, to price section 3.2's workaround on the UHD.

## 11. Sources

Repo (file:line): `extern/box3d/src/solver.h:265-307` (`b3MakeSoft`); `contact_solver.c:30-297` (prepare),
`:399-515` (push), `:518-705` (solve, friction, twist, rolling), `:707-832` (restitution), `:2273-2307` (overflow);
`solver.c:65-223` (integrate velocities with the gyroscopic Newton step, integrate positions and caps);
`math_internal.h:205-217` (`b3IntegrateRotation`); `body.h:163-227` (`b3BodyState`, `b3BodySim`);
`constraint_graph.c:78-169, 216-239` (greedy colouring); `revolute_joint.c:257-609` (prepare, warm start, solve with
spring, motor, limits, collinearity and point constraints); `weld_joint.c:123, 282`; `spherical_joint.c:270-364, 638`;
`distance_joint.c:96-379`; `distance.c:176-228` (simplex solve), `:713` (`b3ShapeDistance`);
`convex_manifold.c:2021-2150` (hull SAT with cache); `dynamic_tree.c:1532`; `constants.h:44, 53, 61, 76, 80, 103,
106, 123, 127-133`; `constraint_graph.h:22, 26`; `types.c:17-39` (world defaults: 400 m/s cap, sleep 0.05 m/s,
contact 30 Hz, damping 10, speed 3 m/s). Local facts: `vulkaninfoSDK.exe` (Vulkan SDK 1.4.357) on 2026-09-30 for
both GPUs' `shaderInt64`, `shaderBufferInt64Atomics`, float controls, `timelineSemaphore`, `bufferDeviceAddress`,
`subgroupSize`; the SDK's `Bin` directory (dxc, glslang, slangc 2026.13.1, spirv-cross, spirv-val).

- fixed3d README (2026-09-07): https://raw.githubusercontent.com/mas-bandwidth/fixed3d/main/README.md ; its
  arithmetic header (`fixMul`, `fixDiv`, `fixSqrt`, `FIX_SATURATE`, the Q2.30 quaternion):
  https://raw.githubusercontent.com/mas-bandwidth/fixed3d/main/extern/fixed/include/fixed/fixed.h
- NVIDIA CUDA Binary Utilities 12.4, instruction set listings (Maxwell/Pascal with XMAD; Volta and later with IMAD):
  https://docs.nvidia.com/cuda/archive/12.4.0/cuda-binary-utilities/index.html
- allanmac, Kepler vs Maxwell integer multiply SASS counts (sm_35 vs sm_52):
  https://gist.github.com/allanmac/8973f01a2e5b2aa5a994
- njuffa on XMAD and long multiplication (NVIDIA forums):
  https://forums.developer.nvidia.com/t/long-integer-multiplication-mul-wide-u64-and-mul-wide-u128/51520
- envytools, Maxwell integer instructions (XMAD operand widths and modes):
  https://envytools.readthedocs.io/en/latest/hw/graph/maxwell/cuda/int.html
- NVIDIA Volta tuning guide (64 FP32 + 64 INT32 cores per SM): https://docs.nvidia.com/cuda/volta-tuning-guide/index.html ;
  Turing tuning guide: https://docs.nvidia.com/cuda/turing-tuning-guide/index.html ; Ampere tuning guide (8.6 has 2x
  the FP32 rate of 8.0): https://docs.nvidia.com/cuda/ampere-tuning-guide/index.html . The CUDA C++ Programming
  Guide's "Throughput of Native Arithmetic Instructions" table could not be fetched this session (the page exceeds
  the fetch limit; the new guide layout no longer has a performance chapter): the INT32 lane counts for 7.x-8.9 are
  recalled from it and marked so.
- GeForce 10 series specifications (GTX 1060 6GB: 10 SMs, 1,280 cores, 1,506/1,708 MHz, 3,470/3,935 GFLOPS):
  https://en.wikipedia.org/wiki/GeForce_10_series ; GeForce 20 series (RTX 2080: 46 SMs, 2,944 cores, 1,515/1,710 MHz):
  https://en.wikipedia.org/wiki/GeForce_20_series
- uops.info: PMULDQ xmm https://www.uops.info/html-instr/PMULDQ_XMM_XMM.html ; VPMULDQ ymm
  https://www.uops.info/html-instr/VPMULDQ_YMM_YMM_YMM.html ; IMUL r64 https://www.uops.info/html-instr/IMUL_R64_R64.html ;
  VPSRAVD ymm https://www.uops.info/html-instr/VPSRAVD_YMM_YMM_YMM.html
- Apple GPU instruction costs: https://github.com/philipturner/metal-benchmarks
- AMD RDNA 3 integer multiply (Chips and Cheese, 2023):
  https://chipsandcheese.com/p/microbenchmarking-amds-rdna-3-graphics-architecture
- Slang targets: https://shader-slang.org/slang/user-guide/targets.html ; CPU target:
  https://raw.githubusercontent.com/shader-slang/slang/master/docs/cpu-target.md ; per-target compatibility (64-bit
  integers): https://raw.githubusercontent.com/shader-slang/slang/master/docs/target-compatibility.md
- Frama-C Eva: https://frama-c.com/fc-plugins/eva.html ; the kernel's `-warn-*` options (man page):
  https://raw.githubusercontent.com/Frama-C/Frama-C-snapshot/master/man/frama-c.1 ; RTE plugin:
  https://frama-c.com/fc-plugins/rte.html
- CBMC: https://www.cprover.org/cbmc/ ; property checks and unwinding (man page):
  https://raw.githubusercontent.com/diffblue/cbmc/develop/doc/man/cbmc.1
- AVBD (Giles, Diaz, Yuksel, SIGGRAPH 2025): https://graphics.cs.utah.edu/research/projects/avbd/ ; the authors' 2D
  demo solver (primal step, dual update, penalty rule, alpha/beta/gamma):
  https://raw.githubusercontent.com/savant117/avbd-demo2d/main/source/solver.cpp ; the paper PDF exceeded the fetch
  limit; the 3D per-body 6x6 form and the 4090 timings are as summarised in T5 (docs/research/m7-gpu.md §1-2)
- Mass splitting (Tonge et al. 2012) and Jones-Plassmann colouring as used by three-avbd: via T5's sources;
  Jones and Plassmann, "A parallel graph coloring heuristic", SIAM J. Sci. Comput. 14(3), 1993 (not fetched)
- SPIR-V integer semantics (`OpIAdd`/`OpIMul` low-order bits, `OpSDiv`/`OpSRem`, shift undefinedness):
  https://registry.khronos.org/SPIR-V/specs/unified1/SPIRV.html (recalled; the page exceeded the fetch limit);
  DXIL shift-count masking and integer division by zero: recalled from the D3D functional spec, unverified: E11's
  helper rows pin the observed behaviour on both drivers
- Wave-1 reports leaned on: docs/research/m7-fixed-point.md (T4: ranges, costs, fixed3d, exact fracture),
  m7-gpu.md (T5: formulations, GPU determinism requirements, round trip, launch latencies), m7-float.md (T3: the
  dialect, GPU flag table, self-test vector), m7-model.md (T1: convergent lockstep), m7-state-audit.md (T0: Box3D's
  hidden state, origination map).
