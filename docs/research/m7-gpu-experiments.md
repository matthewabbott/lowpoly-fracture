# M7 experiment E11: a Box3D-style contact solve, bit-exact across GPU vendors and a CPU twin?

Written 2026-09-30. Question: can a Box3D-shaped soft-step contact solve produce the same bits on an NVIDIA GPU, an
Intel GPU and a CPU twin, in float and in integer (fixed-point) variants, and what does each cost? Includes T10's
fourth variant (V4, block-scaled int32, `docs/research/m7-gpu-integer.md` section 10) with its row benchmark.
Repo facts are `file:line` at the pinned Box3D commit. Everything measured here is on one laptop with two GPUs and
two driver versions; "unverified" marks claims about other hardware, with what would verify them. Code, SPIR-V,
generated C++ and raw logs are under `build/m7/gpu/` (gitignored; logs in `build/m7/gpu/logs/`).

## Summary

- **Yes, with conditions, for float and for 32-bit integers; the Q32.32 variant broke on one driver.** Over 60
  steps at 1k, 10k and 100k contacts (Box3D's warm start / push / relax in Box3D's operation order, 4 substeps,
  soft contacts at 30 Hz / 10), the RTX 3060 (driver 581.95), the Intel UHD (Gen9.5, driver 101.2137) and a CPU twin
  compiled from the same Slang source (MSVC; clang-cl checked on F at 10k and on the rows) agree bit for bit on
  every word of state for:
  - **F**, the float dialect (fp32, `NoContraction`, no GPU sqrt or division, CPU reciprocals, integer-seeded Newton
    steps for the rotation normalisation and speed cap). Exception: one signed zero in one impulse at 100k on the
    UHD. Its compare-and-branch clamp is rewritten into min/max and flips -0 to +0 under every float-control mode.
  - **I32**, per-quantity scales with per-body and per-contact exponents.
  - **V4**, T10's block-scaled int32 with precomputed shift bytes, with identical saturation counts.
- **Box3D's own float kernel shape is not bit-exact on GPUs.** The same solve with Box3D's disc friction clamp
  (one GPU sqrt and one division per contact) diverges within the first step on both GPUs and between them. sqrt
  and inversesqrt are 1 ulp off on both GPUs in every mode; division is correctly rounded only on NVIDIA, and only
  once `RoundingModeRTE` is declared.
- **Drivers decide more than the spec suggests.** Both flush fp32 subnormals by default. NVIDIA offers no preserve
  mode, but declaring `RoundingModeRTE` switches it to preserved subnormals and correctly rounded division. Both
  rewrite compare-and-branch selects into min/max, changing NaN and ±0 results; `SignedZeroInfNanPreserve` stops
  this on NVIDIA, not on Intel.
- **Q32.32 (I64) matches the twin on NVIDIA and is miscompiled on the UHD.** The stored impulse becomes the
  manifold's impulse sum, a register-allocation bug in a 64-bit kernel with 187 spills. All 16 int64 building
  blocks pass 1M-operand probes on the same driver in isolation. Integer code needs per-driver self-tests too.
- **Cost at 100k contacts (ms per step, GPU timestamps):**
  - RTX 3060: F 5.4, I32 6.2, V4 7.2, I64 12.1.
  - UHD: F 31, I32 39, V4 41, I64 312 (and wrong).
  - CPU twin (scalar Slang C++): F 118 at 1 thread, 25 at 8.
  - Box3D's own solve on 4-point box contacts: 125 ms at 1 worker, 54 at 8 (approximate reference).
  - Block-scaled int32 costs 1.16-1.35x float in the full solve, far below T4's estimates.
  - At our scenes' few thousand contacts every GPU is dispatch-bound: 1k contacts take 0.78 ms on the 3060 against
    Box3D's whole solve at 0.24 ms (8 workers).
- **T10's row benchmark (2^20 rows + 1,024 hand-made).** V1-V4b hash identically on both GPUs, both twin compilers
  and three shader front ends (Slang, dxc, glslang).
  - Per row, V4 costs 1.76x float on the 3060 and 1.32x on the UHD; V2 (Q32.32) costs 2.14x and 4.84x.
  - V4 has no clamp-decision flips outside the adversarial quarter.
  - V4's angular velocity format (±256 rad/s) saturates in realistic chip and town rows, making chip-row velocity
    changes 7.5 % rms wrong. Q11.20 removes the saturation in the full solve.
  - V4b's clz normalisation bought nothing measurable.
- **Verdicts.**
  - H4: partly falsified. A GPU solve can sit on the deterministic path across two vendors, given per-driver float
    controls and a startup self-test. The iGPU and small-scene net-loss half is confirmed.
  - H3: falsified for block-scaled int32 on GPUs (1.2-1.4x), confirmed and worse for Q32.32 on the iGPU (10x).

## 1. Setup

Machine: Windows 11, i7-10870H (8 cores), RTX 3060 Laptop GPU (Ampere GA106, driver 581.95, Vulkan 1.4.312) and
Intel UHD Graphics (Comet Lake Gen9.5 GT2, device 0x9bc4, driver 101.2137, Vulkan 1.3.215), both driven through
Vulkan from one process. MSVC 19 (VS BuildTools 2022) `/O2 /fp:precise` for every C/C++ twin (x64 SSE2 baseline:
no FMA instruction exists to contract into); clang-cl 23.1.2 for a second twin. Vulkan SDK 1.4.357 (slangc
2026.13.1, dxc, glslangValidator, spirv-dis/as/val).

What the driver reports (`logs/vulkaninfo_full.txt`):

| | RTX 3060 (581.95) | UHD (101.2137) |
|---|---|---|
| `shaderDenormPreserveFloat32` / `FlushToZero` | false / false | true / true |
| `shaderRoundingModeRTEFloat32` / `RTZ` | true / true | true / true |
| `shaderSignedZeroInfNanPreserveFloat32` | true | true |
| `shaderInt64` | true | true |
| timestamp period | 1 ns | 83.3 ns |
| subgroup size (compute) | 32 | 8 (pipeline statistics) |
| `VK_KHR_pipeline_executable_properties` | statistics only (registers) | statistics only (instruction and cycle counts); neither exposes ISA text |

Route: **one Slang source per kernel, compiled twice**: `slangc -target spirv -fp-mode precise` for the GPUs and
`slangc -target cpp` for the CPU twin, whose output (about 6,500 lines including Slang's C++ prelude) is compiled by
MSVC and linked into the harness. `-fp-mode precise` puts `NoContraction` on every float add, sub and mul (push:
314 float ops, 314 decorations; relax: 431 ops, 434 decorations; checked with `spirv-dis`). Float-control execution
modes that Slang does not emit (`RoundingModeRTE`, `SignedZeroInfNanPreserve`) are added by a SPIR-V patcher
(`tools/patch_spv.py`: disassemble, insert, reassemble, `spirv-val`). T10 chose a shared C/HLSL subset compiled by
MSVC and dxc instead; as T10 asked, V4 was also cross-compiled Slang -> HLSL -> dxc and Slang -> GLSL -> glslang
(section 6). The Slang CPU target worked for this compute-only code (no barriers, groupshared or atomics in the
solver; the V4 and row counters use a plain increment in the twin, so the 8-thread twin's counts are approximate
while its state hash is exact).

Harness (`src/harness.c`, `src/vk_util.c`, C, Vulkan only): enumerates both GPUs, creates one pipeline per entry
point per variant, records a whole 60-step run into one command buffer (one dispatch per colour per stage, a global
memory barrier after each, a timestamp between steps), reads back and bit-compares every word of the final state
(body velocities, delta pose, pose, accumulated impulses) against the CPU twin; then replays the run with one submit
per step plus a readback of bodies and poses, which gives the round-trip time and the first step whose state hash
differs from the twin's. A `--trace` mode runs the twin and one GPU dispatch by dispatch and stops at the first
differing word with a dump of the contact, both bodies and the impulses. The CPU twin runs on 1 thread or on an
8-thread spin pool (one barrier per dispatch); 1 and 8 threads always gave identical hashes.

## 2. The workload and how it maps to Box3D

Scenario (`make_scenario`, generated in double with + - * / and sqrt only, so the same bits on any IEEE machine):
N contacts (1k, 10k, 100k) between N/2 dynamic bodies plus one static body; masses log-uniform 0.05-3000 kg,
densities 150-7800 kg/m^3, box half-extents from mass/density with aspect 0.6-1.6 (edges clamped to 5 cm-3 m),
inverse inertia R diag R^T for a random orientation; speeds up to 20 m/s, spins up to 10 rad/s; 10 % of contacts
against the static body; partner within 64 body indices (memory locality); random normal, 1 or 2 manifold points,
anchors inside each box, separations -2 cm to +1 cm; friction 0.6, restitution 0. Greedy colouring in contact order
(no two contacts of a colour share a dynamic body; static does not count): 11, 11 and 13 colours; 141-165
dispatches per step.

Per step, Box3D's stage order (`solver.c:1768-1776`): 4 substeps of integrate velocities, warm start, solve (push),
integrate positions, relax; then a finalize that folds the delta pose into the pose. dt 1/60, h 1/240. Soft
coefficients from `b3MakeSoft` (`solver.h:265-307`) at contact hertz 30, damping ratio 10, and the static softness
(60 Hz, 5) for static contacts (`physics_world.c:1127-1129`), push bias `massScale * biasRate * s` clamped at
-3 m/s. Kernels transcribe `b3WarmStartContacts_Convex` (`contact_solver.c:1727-1785`), `b3PushContacts_Convex`
(`:1789-1885`) and `b3SolveContacts_Convex` (`:1887-2066`) in Box3D's operation order (`b3MulAdd` = a + s*b, dots
left to right, `b3InvRotateVectorW` for the normal in each body's delta frame, central friction at the friction
centre with the 2x2 tangent mass), one thread per contact.

Simplifications (documented; none changes the arithmetic shape of the inner loops):

- Prepare runs once on the CPU (in double, rounded to each format) and the manifolds stay fixed for all 60 steps, as
  if the narrowphase kept returning the same manifold; world inverse inertia is not re-rotated; the delta pose resets
  every step, so every contact stays active. Effective masses (`1/kNormal`, the 2x2 tangent inverse) are therefore
  CPU reciprocals: no division reaches the GPU.
- No twist or rolling friction, restitution, store-impulses stage, joints, gyroscopic term, damping or linear speed
  cap. Friction is a box clamp per tangent row (no sqrt, no division); `Fdisc` restores Box3D's disc clamp
  (`contact_solver.c:2027-2045`).
- Rotation is renormalised with three Newton steps for 1/sqrt from y = 1 (|q|^2 = 1 + (h|w|/2)^2 here) instead of
  `1/sqrtf`, and the angular speed cap (0.25 pi per step, `constants.h:76`, `solver.c:208-214`) is applied without
  sqrt or division: u = |w / maxSpeed|^2 and, above 1, six Newton steps for 1/sqrt(u) from a seed built from u's
  exponent bits (float) or leading bit (integer). Without the cap the scenario blows up (chips hit by heavy bodies
  exceed ~700 rad/s, the seed-1 Newton step diverges, NaN); that run is kept as `Fnocap`.
- All clamps and selects are written as compare-and-branch (never `min`/`max` intrinsics), because their ±0 and NaN
  rules differ by vendor (section 7).

## 3. Variants

Full-solve variants (one Slang source, `src/solver.slang`, arithmetic selected by macros; `src/layout.h`):

| name | format | products | rounding | notes |
|---|---|---|---|---|
| F | fp32 | fp32 mul, `NoContraction` | RNE | CPU reciprocals only; box friction clamp; NVIDIA default modes, Intel `DenormPreserve` |
| F-ieee | as F | as F | as F | plus `RoundingModeRTE + SignedZeroInfNanPreserve` on NVIDIA, `DenormPreserve + SignedZeroInfNanPreserve` on Intel |
| F-any | as F | as F | as F | no float-control modes on either GPU |
| Fdisc | as F | as F | as F | Box3D's disc friction clamp: one GPU `sqrt` and one GPU division per contact per relax |
| Fnocap | as F | as F | as F | no angular speed cap: the NaN run |
| I64 | Q32.32 in int64 everywhere | 4 partial products of 32-bit limbs (3 widening, 1 low), signed correction on bits 64..95 | round half up (bits 32..95 of product + 2^31) | positions Q32.32; wraps on overflow (not counted) |
| I32 | int32, a fixed power-of-two scale per quantity plus exponents: v, w Q11.20; delta position, anchors, separations Q7.24; unit vectors and quaternions Q1.30; h at 2^-38; 1/h and bias rate Q11.20; soft coefficients Q1.30; inverse mass and inertia mantissas below 2^30 with per-body exponents; impulses with a per-contact exponent (largest effective mass x 100 m/s in 28 bits); normal and tangent mass with per-contact exponents | 32x32->64, sums of products exact in int64, one shift | round half up | positions int64 Q32.32; shift = sum of exponents computed in the kernel; narrows wrap (a `CHECK_RANGE` build counts them) |
| V4 | T10's formats: v Q9.22, w Q8.23, delta position Q5.26, anchors Q8.24, separation Q9.22, unit vectors Q1.30, h = 17895697 x 2^-32, 1/h = 240 exactly, bias rate Q8.24, soft coefficients Q1.30; inverse mass mantissa in [2^30, 2^31) and inverse inertia (largest principal value in [2^28, 2^29)) with per-body exponents; normal mass mantissa in [2^30, 2^31) per point; impulse exponent e_P = e_n + 10 per manifold | one 32x32->64 multiply and one shift byte per cross-body product, bytes precomputed in prepare and clamped to [1, 63] | round half up in the kernel | narrows saturate and are counted with an atomic |

Row-benchmark variants (`src/rows.slang`, T10 section 10; one row is one manifold point with private copies of both
bodies: Box3D's mesh push point `contact_solver.c:449-500`, then the relax row `:571-610` on the updated velocities):
V1 = fp32 as F; V2 = Q32.32 as I64; V3 = int32 with one global scale per quantity (V4's fixed formats, plus inverse
mass 2^26, inverse inertia 2^13, normal mass 2^19, impulses 2^15; no exponents); V4 = T10's block-scaled int32 with
precomputed shift bytes; V4b = V4 with clz normalisation of `r x P` and of the velocity term (keeps 31 bits, about 8
extra ops per vector). The row kernel also needs a full 3x3 world inverse inertia and `b3RotateVector` of both
anchors, as the mesh path does.

## 4. Full solve: bit agreement

60 steps from the same initial state; every word of the final state compared (N = 1k: 15,523 words; 10k: 155,023;
100k: 1,550,023). "=" means bit-identical. The per-step replay compared body and pose hashes after every step with
the twin's; the step is given where they first differ. The CPU twin gave the same hash at 1 and 8 threads in every
run, and the clang-cl twin (`-ffp-contract=off`) gave the MSVC twin's hash (full solve F at 10k,
`e6ae38a69d096603`; rows V1 and V4).

| variant | N | NVIDIA vs twin | Intel vs twin | NVIDIA vs Intel | notes |
|---|---|---|---|---|---|
| F | 1k, 10k | = | = | = | |
| F | 100k | = | 1 word: signed zero | 1 word | `impulse[93030].friction[0]`: twin -0, UHD +0 (the UHD's clamp rewrite, section 7); body and pose hashes equal at every step |
| F-ieee | 1k, 10k, 100k | = | as F (1 signed zero at 100k) | as F | `SignedZeroInfNanPreserve` does not stop the UHD's -0 -> +0 |
| F-any (no modes: both GPUs flush subnormals) | 1k, 10k, 100k | = | as F | as F | flushing changed nothing here (the final state holds no subnormal word); other workloads may differ |
| Fdisc (GPU sqrt and divide) | 1k | 4,222 words differ from step 0 | 4,114 from step 0 | 4,016 | 1 ulp at the first differing step, then chaotic growth |
| Fdisc | 100k | 432,644 (28 %) | 426,541 | 414,274 | max difference 2.2e9 ulp |
| Fdisc-ieee (3060 division correctly rounded) | 100k | 402,417 | 426,541 | 431,281 | sqrt is still 1 ulp off |
| Fnocap (blow-up to NaN) | 1k | 886 from step 3 | 874 from step 3 | 12 (NaN payloads only) | both GPUs agree, the twin differs (below) |
| I64 (Q32.32) | 1k, 10k, 100k | = | 5,911 / 89,650 / 899,797 from step 0 | as Intel | **UHD driver miscompile** (below) |
| I32 (per-quantity scales + exponents) | 1k, 10k, 100k | = | = | = | |
| V4 (T10 block-scaled) | 1k, 10k, 100k | = | = | = | 8,068 / 155,326 / 1,460,705 saturations counted over 60 steps, identical on all three |

First-mismatch dumps and what they were:

- **The UHD's int64 miscompile.** Tracing I64 dispatch by dispatch (`logs/trace_I64_intel.txt`): warm start, push
  and integrate positions match for the whole first substep; the first relax dispatch stores
  `normal[0] = 0x0000000e9a09386b` where the twin (and the 3060) store `0x0000000be8c3ed26`, while every body
  velocity it wrote still matches. The stored value is exactly `normal[0] + normal[1]`
  (`0xbe8c3ed26 + 0x2b1454b45`): the driver gave the int64 running total `totalNormalImpulse` the register of the
  still-live `im.normal[0]`. Three source rewrites (sum after the loop, stores before the friction row, a
  non-unrolled loop) did not remove it, and removing the friction multiply changed the symptom rather than curing
  it. All 16 int64 building blocks pass 1M-operand probes on the same driver in isolation, so this is the compiler on
  a large 64-bit kernel (the I64 relax is 11,151 instructions with 187 spills on the UHD; F's relax is 592 with none).
  I32 and V4, whose 64-bit values live only inside one multiply-add-shift, are exact on the same driver.
- **Fnocap.** Chips spin past ~700 rad/s, the seed-1 Newton normalisation diverges and a delta rotation reaches
  1e33 by step 3 (`logs/trace_Fnocap.txt`). The separation becomes NaN; the twin's `maxT(0, NaN)` (compare and
  branch) returns NaN and the next clamp zeroes the impulse; both drivers had rewritten the branch into a `max`
  instruction, which returns 0, so a finite impulse is applied. After that the GPUs agree with each other and
  not with the twin. NaN never reaches the state of a healthy run, but the lesson is that the drivers' select
  rewrite is invisible until a NaN or a -0 appears.
- **V4b's shift by 64** (row benchmark, fixed before the final runs): for a static body the dynamic shift became
  40 + 24 - 0 = 64. The 3060 returned -1 for `(x + 2^63) >> 64`; x86 and the UHD mask the count to 0. The rows
  differed on NVIDIA only until the shift was clamped to [1, 63]. Integer determinism has its own undefined
  behaviour: shift counts >= the width, signed overflow in the C++ twin, `-INT_MIN`.

## 5. Full solve: cost

ms per 60-step run divided by 60. GPU: timestamps between steps in one command buffer, median of 3 runs after a
warm-up (fastest single step in brackets); "readback" is wall time with one submit, wait and readback of bodies and
poses per step. CPU twin: the Slang-generated scalar C++ (MSVC), 1 thread and 8 threads (spin pool, one barrier
per dispatch). Dispatches per step: 141 (1k, 10k) and 165 (100k).

| variant | N | RTX 3060 | 3060 readback | UHD | UHD readback | twin 1 thread | twin 8 threads |
|---|---|---|---|---|---|---|---|
| F | 1k | 0.78 (0.74) | 0.99 | 1.13 (0.88) | 1.24 | 1.04 | 0.74 |
| F | 10k | 1.50 (1.01) | 1.58 | 2.83 (2.66) | 3.30 | 10.3 | 2.72 |
| F | 100k | 5.37 (5.35) | 7.06 | 31.0 (29.0) | 32.7 | 117.5 | 25.5 |
| I32 | 1k | 0.94 (0.90) | 1.01 | 2.30 (2.13) | 2.63 | 1.23 | 0.82 |
| I32 | 10k | 1.27 (1.20) | 1.83 | 5.24 (5.02) | 5.54 | 13.3 | 3.25 |
| I32 | 100k | 6.22 (6.19) | 8.01 | 38.6 (37.3) | 39.5 | 148.1 | 33.5 |
| V4 | 1k | 1.37 (1.27) | 1.51 | 2.83 (2.67) | 3.17 | 2.20 | 1.42 |
| V4 | 10k | 2.00 (1.97) | 2.48 | 5.22 (4.99) | 5.64 | 22.7 | 5.47 |
| V4 | 100k | 7.23 (7.21) | 8.99 | 40.8 (39.4) | 42.0 | 234.3 | 47.3 |
| I64 | 1k | 1.80 (1.75) | 1.96 | 26.4 (wrong) | 26.9 | 6.52 | 3.77 |
| I64 | 10k | 3.06 (3.04) | 3.62 | 44.9 (wrong) | 45.1 | 65.0 | 17.3 |
| I64 | 100k | 12.05 (12.02) | 14.76 | 312 (wrong) | 314 | 690 | 153 |
| Fdisc | 100k | 5.43 | 7.04 | 31.0 | 32.9 | 118.0 | |

F at 10k on the 3060 measured 1.50 ms with a fastest step of 1.01 ms; F-ieee and F-any (the same arithmetic)
measured 1.03 and 1.02 ms (the laptop GPU's clocks). Ratios to F at 100k: on the 3060 I32 1.16x, V4 1.35x, I64 2.24x; on the UHD I32 1.24x, V4
1.31x, I64 10.1x; on the twin (1 thread) I32 1.26x, V4 1.99x, I64 5.9x (the twin's Q32.32 is the same 32-bit limb
code as the GPU's; a CPU implementation would use `_mul128`/`__int128` and be cheaper, unmeasured here). At 1k the
GPUs are latency-bound: 141 dispatches and barriers cost about 5.5 us each on the 3060.

**Box3D reference (approximate).** `src/b3ref.c` links our `box3d.lib` (msvc-release) and builds stacks of 1 m
boxes, 10 high, sleep off, dt 1/60 with 4 substeps, and reads `b3World_GetProfile` over 60 steps after 120 settling
steps (`logs/b3ref.txt`). Its contacts are box-on-box faces with 4-point manifolds (about 2.7x our points per
contact), and its integrate-velocities stage includes the gyroscopic Newton solve we omit, so compare magnitudes
only:

| contacts | Box3D workers | step | solve (all stages) | substep stages (intVel + warm + solve + intPos + relax) | contact stages (warm + solve + relax) |
|---|---|---|---|---|---|
| 1,000 | 1 / 8 | 0.87 / 0.28 | 0.80 / 0.24 | 0.64 / 0.17 | 0.38 / 0.12 |
| 10,000 | 1 / 8 | 9.14 / 1.65 | 8.37 / 1.46 | 6.56 / 1.04 | 4.01 / 0.66 |
| 100,000 | 1 / 8 | 136.6 / 60.5 | 124.9 / 53.7 | 92.9 / 32.1 | 64.7 / 26.3 |

`lpf_bench --scene pile --workers 1,8` (`logs/lpf_bench_pile.txt`, `.json`): awake contacts average 2,993 (max
3,496), physics 1.98 ms a step at 1 worker and 0.71 ms at 8 (broadphase, narrowphase and solve together), hash
`f38a3b6505d95c2b` at both worker counts. Our scenes today sit at a few thousand awake contacts, the regime where the
GPU solve is dispatch-bound (1k: 0.78 ms on the 3060 against Box3D's whole solve at 8 workers, 0.24 ms).

## 6. Row benchmark (T10's V4 specification)

2^20 rows (a quarter each of "town": 1-100 kg, |r| <= 0.5 m, under 20 m/s, 10 % static; "chip against mech": 0.05 kg
with inertia 6.3e-6 against 3,000 kg with inertia 7,000, both orders; "car crash": 1,700 kg at 15-20 m/s into a
static, warm-start impulses up to 34,000 N s; "adversarial": every input at a V4 format limit, i.e. |r| = 32 m,
v = ±512 m/s, w = ±256 rad/s, separation ±512 m, normal mass 1e-7 or 1e5 kg, impulses near ±2^31 units, delta
rotation unnormalised by up to 13 %) plus 1,024 hand-made rows (all zero; static partner at rest; warm impulse at
rest; an exact clamp tie where `lambda + delta` is exactly 0; delta exactly 0; far speculative; equal velocities so
vn is exactly 0), all exactly representable in every format. A float64 reference computes the same two rows per
row from the exact inputs. Logs: `logs/rows_*.txt`, `logs/rows_summary.csv`.

**Hashes** (64-bit FNV over all outputs and both counters). Every variant hashes identically on the RTX 3060, the
UHD and the MSVC twin; V4 also through Slang -> HLSL -> dxc (after adding Vulkan bindings, because Slang's HLSL
output carries D3D registers only) and Slang -> GLSL -> glslang; V1 also with the IEEE float-control modes.

| variant | hash | RTX 3060 ns/row | UHD ns/row | MSVC twin ns/row (1 thread) | vs V1 on 3060 / UHD | registers (3060) | instructions, spills (UHD) |
|---|---|---|---|---|---|---|---|
| V1 fp32 | `ab5661ac89d3d2de` | 1.78 | 38.7 | 125 | 1.00 / 1.00 | 61 | 463, 0 |
| V2 Q32.32 | `64a34bdbc001aa50` | 3.80 | 187.2 | 706 | 2.14 / 4.84 | 126 | 8,572, 20 |
| V3 global scales | `8dbf59a0f05e9857` | 3.07 | 53.4 | 379 | 1.73 / 1.38 | 80 | 4,051, 0 |
| V4 block-scaled | `dee0c00ab61307a0` | 3.12 | 51.2 | 401 | 1.76 / 1.32 | 80 | 4,051, 0 |
| V4b (clz) | `8dcefcdc4110e7b6` | 3.05 | 66.7 | 563 | 1.72 / 1.72 | 80 | 6,837, 53 |
| V4 via dxc | `dee0c00ab61307a0` | 3.47 | 55.3 | | | | |
| V4 via glslang | `dee0c00ab61307a0` | 3.11 | 54.6 | | | | |
| V1, RTE+SZINP (3060) / preserve+SZINP (UHD) | `ab5661ac89d3d2de` | 1.78 | 37.2 | | | | |

A second twin compiler, clang-cl 23.1.2 (`/O2 -ffp-contract=off`, `build_clang.ps1`; installed on this machine
since the brief was written), gives the same hashes for V1 and V4 (`logs/rows_V1-clang.txt`, `rows_V4-clang.txt`;
the logs' "MSVC" label is the harness's fixed text). The positive control, the same twin with `-mavx2 -mfma
-ffp-contract=fast`, differs from both GPUs in 525,622 of 1,049,600 rows (and in 52,538 of 155,023 words of the
10k full solve), so the comparison does see contraction when it happens.

Medians of 5 timed dispatches after a warm-up, validation layers off. Rows are 316 bytes in and 56 out for the
32-bit formats (568 and 112 for V2), so on the 3060 V1 runs near memory bandwidth (about 210 GB/s) and the integer
variants are partly compute-bound; the UHD's fastest V1 dispatch was 32.8 ns/row.

**Accuracy against float64** (relative error of each dynamic body's velocity and spin change, with changes under
1 mm/s or 1 mrad/s measured against 1 mm/s; rms / max over the quarter):

| quarter | V1 | V2 | V3 | V4 | V4b |
|---|---|---|---|---|---|
| town | 6.9e-5 / 8.4e-3 | 8.7e-7 / 7.4e-4 | 1.8e-2 / 8.2 | 6.2e-4 / 0.22 | 5.9e-4 / 0.22 |
| chip vs mech | 1.7e-4 / 2.1e-2 | 6.7e-5 / 2.9e-2 | 0.20 / 71 | 7.5e-2 / 1.0 | 7.5e-2 / 0.81 |
| car crash | 2.0e-5 / 9.3e-3 | 1.8e-7 / 3.2e-5 | 7.5e-2 / 0.45 | 2.8e-5 / 1.3e-2 | 2.7e-5 / 1.3e-2 |
| adversarial | 1.9e-4 / 4.2e-2 | 2.6 / 39 | 0.86 / 1.0 | 0.83 / 52 | 0.82 / 52 |
| hand-made | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |

**Clamp-decision flips** (rows where `newImpulse == 0` differs from float64, push / relax) and **saturations**:

| quarter | V1 flips | V2 flips | V3 flips | V4 flips | V4b flips | V3 sat. | V4 sat. | V4b sat. |
|---|---|---|---|---|---|---|---|---|
| town | 0 / 0 | 0 / 0 | 0 / 2 | 0 / 0 | 0 / 0 | 15 | 15 | 15 |
| chip vs mech | 0 / 0 | 0 / 0 | 3 / 79 | 0 / 0 | 0 / 0 | 115,065 | 114,906 | 114,519 |
| car crash | 0 / 0 | 0 / 0 | 0 / 308 | 0 / 0 | 0 / 0 | 168,491 | 8,940 | 8,940 |
| adversarial | 0 / 13 | 42,695 / 64,097 | 91,608 / 79,897 | 122,966 / 65,781 | 114,714 / 75,462 | 6.6 M | 9.3 M | 7.4 M |

Prepare counters: V3 could not represent 1.44 M adversarial inputs; V4 clamped 393,376 shift bytes to [1, 63] and
could not represent 131,085 inputs, all in the adversarial quarter. V4b's kernel clamped 430,865 dynamic shifts.

What the rows say:

- V4 meets T10's bar on realistic rows: identical on both GPUs, both front ends and the twin; no clamp flips in
  town, chip-vs-mech or car crash; car-crash accuracy equal to float. It costs 1.76x V1 on the 3060 and 1.32x on the
  UHD per row (V2, the Q32.32 path, 2.14x and 4.84x).
- V4 is less accurate than float in town (rms 6.2e-4 against 6.9e-5), and its angular velocity format (Q8.23,
  ±256 rad/s) saturates in realistic rows: 15 town rows (a 1 kg body hit hard spins past 256 rad/s within the
  substep, before the 47 rad/s cap in integrate positions) and 115k chip rows (a chip struck by a 3 t body reaches
  thousands of rad/s). Saturation is deterministic, so it is a fidelity cost, not a sync risk, but it makes chip
  rows 7.5 % wrong (rms). The full solve confirms it: V4 counted 3,725 saturations in 10 steps at 1k contacts, and
  the same kernel with w in Q11.20 counted none (`logs/smoke_V4w20.txt`). Recommendation to T10: w in Q11.20
  (±2048 rad/s, 9.5e-7 rad/s) or an exponent on w per body.
- V4b's clz normalisation did not buy measurable accuracy over V4 at full scale (town rms 5.9e-4 against 6.2e-4) and
  costs 30 % more on the UHD (53 spills); keep static shifts.
- V3 (one global scale per quantity) fails where T4 predicted: the mech's inverse inertia is about one lsb at 2^13,
  impulses at 2^15 cannot hold both a chip's 2e-3 N s and a car's 34,000 N s, and car crashes saturate (168k).
- V2 is the accurate one (town rms 8.7e-7) but wraps silently in the adversarial quarter (no saturation in Q32.32
  products), where its errors are the worst of all.

## 7. Float-op probe

`src/probe.slang` + `src/probe.c`, `logs/probe.txt`. 1,048,576 fixed-seed operand triples per op in eight classes
(random bit patterns including NaN and infinity; magnitudes near 1; subnormal inputs; products and sums that
underflow; signed zeros; exact add ties such as 1 + 3 x 2^-24; exact multiply ties from odd 13-bit integers;
exponents across the whole range), each op in its own dispatch, compared bit for bit with MSVC `/fp:precise` on
x64 (`sqrtf`, `/` correctly rounded; inversesqrt reference `(float)(1/sqrt((double)x))`; `fmaf` for `fma`).
"flushed" = the CPU result is subnormal and the GPU returned zero; "sub-in" = a subnormal input and a different
result. Counts are out of 1,048,576.

| op | 3060, no float-control modes | 3060, RTE + SZINP | UHD, no modes | UHD, DenormPreserve + SZINP |
|---|---|---|---|---|
| a + b | flushed 113,932 of 113,932 subnormal results; NaN payload 1,048 | exact except NaN payload | flushed all; payload 5 | exact except 5 NaN payloads |
| a * b | flushed all 179,866; payload 1,011 | exact except payload | flushed all | exact except 4 payloads |
| a * b + c (`NoContraction`) | flushed; no fusion seen (near-1 and tie classes: 0 mismatches) | exact except payload | flushed; no fusion | exact except 4 payloads |
| a / b | 209,106 at 1 ulp, 1,598 more (flush-related) | **correctly rounded** (only 15,458 NaN payloads) | 209,967 at 1 ulp, 342 at 2 ulp | 210,141 at 1 ulp, 342 at 2 ulp |
| sqrt | 131,537 at 1 ulp (max 1) | 131,537 at 1 ulp | 62,575 at 1 ulp | 62,575 at 1 ulp |
| inversesqrt | 172,740 at 1 ulp (max 1) | 172,740 at 1 ulp | 76,158 at 1 ulp | 76,158 at 1 ulp |
| `a > b ? a : b` (branch) | NaN -> number 505, ±0 3,592, flushed 127,326 | flush only | flush only | exact |
| `x < -m ? -m : (x > m ? m : x)` (branch) | NaN -> number 550, ±0 5,120, flushed | flush only | **±0 13,917**, flushed | **±0 7,262** |
| `max()`, `min()` | flushed (all modes) | flushed | flushed | exact |
| `mad()` | fused (all 131,072 near-1 and all 131,072 multiply-tie rows differ from the unfused CPU) | fused | unfused | unfused |
| `fma()` | single rounding (matches `fmaf` apart from flush and payload) | same | **not single rounding** (673,631 differ, 458,960 by more than 1 ulp) | 410,543 differ, 353,311 by more than 1 ulp |
| `-a` | flushed | exact | flushed | exact |
| `0 - a` | flushed | exact | folded to `-a` (±0, 28,794) | folded (28,794) |
| `a + 0` | folded to `a` by **slangc** (28,907 ±0), no `OpFAdd` in the SPIR-V | same | same | same |

Integer building blocks (i64 add, sub, low multiply, u32 x u32 -> u64, i32 x i32 -> i64, arithmetic and left
shifts, compares, selects, negate, `max(a - b, 0)`, a rounding shift, `>> 32`, and the Q32.32 limb product checked
against MSVC `_mul128`): **0 mismatches** on both GPUs, 16 ops x 1,048,576.

Reading the table:

- The 3060 flushes fp32 subnormals under Vulkan by default and advertises no way to request preservation, **but
  declaring `RoundingModeRTE` (which it supports) switches the driver to IEEE behaviour: subnormals preserved and
  division correctly rounded.** Nothing in the float-controls properties says so; it is observed behaviour of
  driver 581.95 (verify on other NVIDIA drivers and architectures). `SignedZeroInfNanPreserve` stops the driver's
  rewrite of compare-and-branch selects into min/max (the NaN and ±0 changes), but not the subnormal flush inside
  that min/max.
- The UHD needs `DenormPreserve`; its compare-and-branch clamp still turns -0 into +0 under every mode, it folds
  `0 - a` into `-a`, and its `fma()` is not a single rounding (a spec conformance question for this 2022 driver;
  unverified on newer Intel drivers).
- sqrt and inversesqrt are 1 ulp off on both GPUs in every mode; division is correctly rounded only on NVIDIA with
  RTE. A float kernel that must match a CPU cannot call sqrt, inversesqrt or (on Intel) divide.
- NaN bit patterns differ (NVIDIA returns canonical 0x7fffffff; x86 and the UHD mostly propagate the payload).
- Slang 2026.13.1 folds `x + 0.0f` to `x` even under `-fp-mode precise` (wrong for x = -0), before the driver sees
  it.

## 8. Multiply micro-test

`src/mulbench.comp` (GLSL, glslang), `logs/mulbench.txt`: 2^20 threads x 4 independent chains x 256 dependent
steps, per-thread operands, median of 5. Relative to an fp32 FMA chain:

| step | RTX 3060 G steps/s | x FMA | UHD G steps/s | x FMA |
|---|---|---|---|---|
| fp32 fma | 6,048 | 1.0 | 81.9 | 1.0 |
| int32 mul-add | 3,202 | 1.9 | 37.4 | 2.2 |
| `int64(int32) * int64(int32)`, `>> 20`, narrow | 1,068 | 5.7 | 14.4 | 5.7 |
| same with round-half-up and a variable shift (V4's inner op) | 905 | 6.7 | 13.3 | 6.2 |
| `imulExtended` (hi, lo), recombine | 1,569 | 3.9 | 16.9 | 4.9 |
| full `int64 * int64` | 766 | 7.9 | 5.5 | 14.8 |
| Q32.32 product from 32-bit limbs | 287 | 21.0 | 1.65 | 49.8 |

The 3060's compiler does not reduce a sign-extended 32x32->64 multiply to its widening multiply as well as
`imulExtended` does (5.7x against 3.9x, same recombination work); writing the widening multiply explicitly is worth
~30 % on the multiply-bound part. On the UHD all integer multiplies are multi-instruction and the limb Q32.32
product is 50x an FMA. The UHD's FMA chain itself runs far below its paper peak (latency-bound at 4 chains per
thread), so its absolute numbers are a lower bound.

## 9. Surprises

1. **The RTX 3060 flushes fp32 subnormals under Vulkan by default and cannot be asked to preserve them, yet
   declaring `RoundingModeRTE` turns preservation on and makes division correctly rounded.** Observed on driver
   581.95; nothing in `VkPhysicalDeviceFloatControlsProperties` hints at it.
2. **Both drivers rewrite compare-and-branch selects into min/max instructions**, whose NaN and ±0 rules differ
   from the source. `NoContraction` does not cover it; `SignedZeroInfNanPreserve` stops it on NVIDIA and not on
   Intel. This, not rounding, is what separated the GPUs from the twin in the NaN run.
3. **An integer kernel was the one that broke across vendors**: the UHD miscompiles the Q32.32 relax kernel while
   passing every int64 op in isolation. "Integer semantics are exact" (T10 section 0) is necessary, not sufficient:
   bit-exactness is still a property of each driver's compiler, and needs a per-driver self-test.
4. **Slang is usable as the single source for GPU and CPU**, with three rough edges: `x + 0.0f` is folded under
   `-fp-mode precise`; SPIR-V entry points are renamed to `main`; the HLSL target drops Vulkan binding and
   push-constant attributes. The generated C++ is plain scalar code, about 3-4x slower per contact point than
   Box3D's SSE2 solver, and correct on the first build for all variants.
5. **The UHD's `fma()` is not a single rounding** (353k-459k of 1M results more than 1 ulp from `fmaf`), and it folds `0 - a` to
   `-a`. The 3060's `mad()` fuses, as the spec allows.
6. **int64 on the UHD Gen9.5 is slow as well as fragile**: the Q32.32 row costs 4.8x the float row there (2.1x on
   the 3060), the Q32.32 limb product is 50x an FMA, and the I64 kernels spill (218 spills in push). int32 with
   64-bit products costs 1.3x float on the same GPU.
7. **V4's angular velocity range (±256 rad/s) is too narrow inside a substep** for our mass mix; Q11.20 fixes it.
8. Laptop GPU clocks moved single measurements by up to 1.5x between runs of the same kernel (F at 10k: 1.50 and
   1.03 ms); medians and fastest steps are both given.

## 10. What the results mean

**(a) The float route.** A Box3D-shaped float solve can be bit-exact on NVIDIA, Intel and an x64 CPU twin (MSVC and
clang), at 100k contacts over 60 steps, but only in a dialect narrower than Box3D's:

- add, sub and mul only on the GPU, with `NoContraction`; no GPU sqrt, inversesqrt or division (Fdisc shows one
  1-ulp sqrt diverges the solve within a step, on both GPUs and between them, even with NVIDIA's correctly rounded
  division). Reciprocals and effective masses come from the CPU (or from integer-seeded Newton iterations using
  only add and mul, as the rotation normalisation and speed cap here do). Box3D itself calls `sqrtf` and divides in
  the friction clamp, the rolling clamp, `b3NormalizeQuat` and the speed caps, so Box3D's kernels as written would
  not be bit-exact on these GPUs;
- float controls per vendor: `RoundingModeRTE + SignedZeroInfNanPreserve` on NVIDIA, `DenormPreserve +
  SignedZeroInfNanPreserve` on Intel, both checked at startup against a probe vector;
- no dependence on the sign of zero (the UHD still flips -0 in a clamp) and no NaN or infinity ever in the state
  (min/max rewrites change NaN results). Canonicalising -0 to +0 before hashing, or never producing -0 in clamps,
  closes the last word of difference seen here;
- cost: this is the cheapest deterministic variant on both GPUs (5.4 ms at 100k on the 3060, 31 ms on the UHD).

The burden is that every item above is empirical per driver: three of the four hazards found (FTZ default,
select rewrites, the UHD's -0 clamp) are driver behaviour the Vulkan spec permits or does not pin down.

**(b) The integer route.** Block-scaled int32 (I32 here, V4 from T10) was bit-exact everywhere it ran: both GPUs,
both twin compilers, three shader front ends (Slang, dxc, glslang), all three sizes, with saturation counters
equal to the event. It costs 1.16-1.35x float on the 3060 and 1.24-1.31x on the UHD in the full solve (1.7-1.8x
and 1.3x per row), well under T4's 2-4x and 10-20x estimates, because every product is one 32x32->64 multiply and
the solve is partly memory-bound. Q32.32 (I64/V2) is accurate but 2.2x float on the 3060, 10x on the UHD, and the
UHD's driver miscompiles it. What the integer route still needs: a wider angular velocity format than V4's; care
with shift counts and signed overflow (the twin is C++, where both are undefined); per-driver self-tests (the UHD
bug proves integer code is not exempt); and its accuracy is below float in town rows (V4 rms 6.2e-4 against
6.9e-5), acceptable for a game but worth T10's attention (the static `r x P` narrowing; V4b's clz did not help).

**(c) The GTX 1060 floor (not testable here).** Pascal GP106 has no separate INT32 multiply at full rate: 32-bit
`IMAD` is emulated with 16-bit `XMAD` sequences (T10 section 3.2; unverified here), so the int32 route's premium over
float should be larger than on Ampere (T10 predicts, this experiment cannot confirm). Run on a 1060 with its current
driver, unchanged binaries: `probe.exe` (does RTE also switch Pascal to preserve and correctly rounded division? do
selects get rewritten?), `run_solve.sh` (bit agreement and ms/step for F, F-ieee, I32, V4, I64 at 1k/10k/100k; a
100k F step is expected around 10-15 ms from the FP32 rate ratio, unverified), `run_rows.sh` (V4/V1 ratio on
Pascal is the number the integer route's floor depends on), `mulbench.exe` (the XMAD penalty directly). All hashes
must equal this machine's, since the scenario generator and quantisation are IEEE-portable: the Windows logs here
are the reference.

**Next on the DGX Spark (GB10 Blackwell + Grace ARM64, Linux).** Port `vk_util.c` and the harnesses' timers and
thread pool to Linux (about 60 lines: `clock_gettime`, pthreads), then: (1) the CPU twins with gcc and clang on
ARM64, `-O2 -ffp-contract=off`, which must reproduce the Windows hashes for F, I32, V4 and every row variant (the
ARM64 leg of H1 for this kernel), with `-ffp-contract=fast` as the positive control; (2) the Vulkan probe and
harnesses on GB10 (does the RTE switch hold on Blackwell? GB10's integer and FP32 lanes are unified per T4's cc 12
column, so V4's premium should shrink; unverified); (3) CUDA: `slangc -target cuda` of the same kernels, built with
`--fmad=false`, must reproduce the hashes, and with the default `--fmad=true` should not (T3 section 4).

**Next on the MacBook (Apple silicon, Metal).** Run the Vulkan harnesses through MoltenVK (SPIR-V -> MSL via
SPIRV-Cross) with fast math off (`MVK_CONFIG_FAST_MATH_ENABLED=0`; Metal's default is fast math on, T3 section 4),
and alternatively `slangc -target metal`; the probe answers Apple's subnormal, select and division behaviour, the
rows answer int64 and int32 cost on Apple GPUs (unverified: Metal supports 64-bit integers on Apple GPUs, at a rate
nobody here has measured). CPU twin with Apple clang: pass `-ffp-contract=off` explicitly (ARM64 clang contracts by
default). Also the x64 build under Rosetta 2 against the native ARM64 twin (T3 section 8.5).

**Next on this machine.** A current Intel driver for the UHD (101.2137 is the installed one): does the int64
miscompile, the -0 clamp, the unfused `fma()` survive? T10's optional V5 (exact integers on FP32 units) was not
built.

## 11. Hypotheses touched

- **H4** (GPU physics stays off the deterministic path; net loss on an iGPU): *partly falsified, partly confirmed.*
  A GPU contact solve can be on the deterministic path across two vendors and a CPU twin, in float (restricted
  dialect) and in int32 block-scaled, but only with per-driver float controls and a startup self-test, and one
  driver miscompiled the Q32.32 variant. The iGPU half is confirmed: per contact point the UHD's 100k float solve (31 ms for 150k
  points, 0.21 us a point) is slower than Box3D's whole solve at 8 workers (54 ms for about 400k points, 0.13 us), and at our scenes'
  few-thousand-contact scale every GPU here is dispatch-bound (1k: 0.78 ms on the 3060 against 0.24 ms for Box3D's
  whole solve at 8 workers), before any per-step round trip (0.1-1.7 ms of readback measured).
- **H3** (fixed point costs 2-4x): *falsified for block-scaled int32 on GPUs* (1.16-1.35x float in the full solve,
  1.3-1.8x per row, bit-exact everywhere), *confirmed and worse for Q32.32 on the iGPU* (10x, and miscompiled).
- **T10's "integers make bit-exactness a property of the language specs, not of drivers"**: *qualified.* Exact
  integer semantics removed every float hazard, but a driver bug and a shift-count UB still split vendors.

## 12. Reproducing

From the repo root in PowerShell (`build/m7/gpu` holds everything; `tools/devenv.ps1` is dot-sourced by the scripts):

```powershell
pwsh build/m7/gpu/build.ps1              # slangc SPIR-V (+ patched float modes), C++ twins, harness_*, probe, rows_*, mulbench
pwsh build/m7/gpu/build_clang.ps1        # clang-cl twins: rows_V1-clang, rows_V1-clang-fma, rows_V4-clang, harness_F-clang(-fma)
cd build/m7/gpu
./bin/probe.exe --n 1048576 --log logs/probe.txt
sh run_solve.sh                           # full-solve matrix -> logs/solve_*.txt, logs/summary.csv (about 25 min)
sh run_rows.sh                            # row benchmark -> logs/rows_*.txt, logs/rows_summary.csv
./bin/mulbench.exe logs/mulbench.txt
./bin/harness_I64.exe --n 1000 --trace 3 --gpus 2 --log logs/trace_I64_intel.txt        # the UHD miscompile
./bin/harness_Fnocap.exe --n 1000 --trace 5 --spv gen/Fnocap --log logs/trace_Fnocap.txt # the NaN divergence
cl /O2 /MT /I ../../../extern/box3d/include src/b3ref.c /Fe:bin/b3ref.exe /link ../../msvc-release/extern/box3d/src/box3d.lib
./bin/b3ref.exe 10000 8 10                 # columns, workers, layers
../../msvc-release/bin/lpf_bench.exe --scene pile --workers 1,8 --json logs/lpf_bench_pile.json
```

Harness options: `--n`, `--steps` (60), `--threads` (1,8), `--reps` (3), `--gpus` (bit mask), `--spv DIR`,
`--nvidia-tag` / `--intel-tag` (SPIR-V float-mode suffix: `""`, `.preserve`, `.rte`, `.rtesz`, `.pszinp`, `.ftz`),
`--trace STEPS`; `E11_VALIDATION=1` enables the Khronos validation layer (clean for F, Fdisc, I64, I32, V4 and the
row kernels; `logs/validation_*.txt`).

Sources:

- Box3D at the pinned commit: `extern/box3d/src/contact_solver.c:449-500, 571-706, 1426-2066`,
  `extern/box3d/src/solver.c:65-220, 1768-1776`, `extern/box3d/src/solver.h:265-307`,
  `extern/box3d/src/physics_world.c:1127-1131`, `extern/box3d/src/math_internal.h:205-217`,
  `extern/box3d/include/box3d/constants.h:53-80`.
- Vulkan float controls (`VkPhysicalDeviceFloatControlsProperties`, `DenormPreserve`, `RoundingModeRTE`,
  `SignedZeroInfNanPreserve` execution modes) and `VK_KHR_pipeline_executable_properties`:
  https://registry.khronos.org/vulkan/specs/latest/html/vkspec.html and
  https://registry.khronos.org/SPIR-V/specs/unified1/SPIRV.html (the spec text was not fetched this session; the
  properties quoted are from this machine's `vulkaninfoSDK` output, `logs/vulkaninfo_full.txt`).
- Slang options (`-fp-mode precise`, `-denorm-mode-fp32`, `-target cpp`): `slangc -h` of slangc 2026.13.1 in Vulkan
  SDK 1.4.357; https://shader-slang.org/slang/user-guide/ (not fetched this session).
- Companion reports: `docs/research/m7-float.md` (T3: GPU flag table, section 8's probe list),
  `docs/research/m7-fixed-point.md` (T4: formats and cost estimates), `docs/research/m7-gpu.md` (T5: H4, section 9's
  agreement bar), `docs/research/m7-gpu-integer.md` (T10: V4 formats, sections 1 and 10).
