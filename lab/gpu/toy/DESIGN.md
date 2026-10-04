# The dual-dialect toy: design

Milestone 11's question (docs/roadmap.md §11): can a real rigid-body step (narrowphase, persistent warm starts,
stacking, sleep, a motorised joint) run bit-identically on every GPU and CPU twin, and in which arithmetic? The toy
answers it for both candidates from **one Slang source**: the float dialect **F** and block-scaled int32 **V4**, plus
**D** (double, twin only) as the accuracy reference: the same algorithm in near-exact arithmetic. It is research code
in the GPU lab (`lab/gpu/README.md`), outside the engine and its CI. `NOTES.md` is the running log: what is done,
what each gate measured, what is next.

E11 (`lab/gpu/src/solver.slang`, `layout.h`) is the model to follow: its annotated arithmetic (`M( a, b, sh )`: the
shift is V4's format change, F and D ignore it), its layout rules, its twin glue and its harness's comparisons.

## The decision this feeds

- **Decision point 1 (after step 4b):** the stack, pile, ratio and bounce scenes on the F, V4 and D twins against
  Box3D, judged by the acceptance table below. Kill early: if V4's stacks creep or its piles do not sleep, and
  narrowing `r × P` with a clz shift does not fix it within two days, stop and write it up.
- **Decision point 2 (after step 5):** every device bit-exact against the twin, in both dialects, on the RTX 3060, the
  Intel UHD and two twin compilers (then the Spark's GB10, llvmpipe and aarch64 twins).

## One source, three dialects

`DIALECT_F`, `DIALECT_V4` or `DIALECT_D` picks the scalar types and the arithmetic in `num.slang` (and `layout.h` for
C). Kernels are written once through the helpers; no kernel tests the dialect.

- **F:** fp32, `-fp-mode precise` (NoContraction), on the GPU only add, subtract, multiply. No GPU sqrt, inversesqrt,
  division, min, max, abs, clamp, sign or fma intrinsics: every compare and select goes through helpers written as
  ternaries. Reciprocals and square roots are software: an exponent-bit seed and three Newton steps (E11's
  `rsqrtSeed`). `snap( x )` sends |x| < 2^-30 to +0 at every select and store, so a driver that flushes subnormals and
  one that keeps them both see +0. **F-plain** (no float-control execution modes at all) must be bit-exact: no
  undocumented driver modes in the decision rule (the GB10's compiler crashes on `RoundingModeRTE`).
- **V4:** int32 values with one static format per quantity, products through int64 with one rounding shift (E11's
  `rsh`: add half, arithmetic shift), saturating narrows that count (E11's `narrow`). Formats: unit vectors and
  quaternions Q1.30, points and anchors Q8.24, separations Q9.22, squared distances Q11.20, linear velocity Q9.22,
  angular velocity **Q11.20** (E11's Q8.23 saturated on chips), positions int64 with 32 fractional bits. Inverse
  masses and inertias are mantissas with per-body exponents; effective masses carry per-point exponents; shifts are
  computed in prepare and clamped to [1, 63]. int64 appears only inside one multiply-add-shift helper (E11's Intel
  miscompile involved a long-lived int64).
- **D:** double, twin only (`slangc -target cpp`), `M( a, b, sh ) = a * b`.
- **Division** only through `divClamp( n, d, ... )`: F a software reciprocal (exponent seed + three Newton steps); V4
  `lpDivQ31`, a clz-normalised reciprocal plus a fix-up that gives the exact floor quotient (it must equal C's `/` on
  every input of the battery); D real division.
- **Tolerances** (slop, speculative distance, sleep thresholds, ...) come from the Params buffer, never constants in
  kernels.
- **No atomics or groupshared memory** in kernels (counters for saturation are the one exception, as in E11, and are
  not hashed).

## Pipeline per tick (about 170 dispatches)

GPU kernels, each also built as the C++ twin and run on a thread pool (E11's `twin_glue.cpp` pattern). Kernels take
index lists (`start`, `count` into a list buffer), never "the world", so a unit can later be stepped alone (the
netcode's bubble).

1. `applyCommands`, `prepareBodies` (world inertia, integer AABB at 2^-10 m from the shape's local bounds)
2. **CPU, integers only** (one readback of the AABBs): sort-and-sweep broadphase on integer AABBs (filtering is data);
   radix-sorted pair keys `(min id, max id)`; merge-join with last tick's sorted keys (`prevIndex`, so manifolds stay
   on the GPU); greedy colouring (joints first, then pairs in key order); union-find islands and sleep (an island
   sleeps when every body's counter has reached 30 ticks under the thresholds).
3. `narrowSat`: Box3D's SAT with its cache made data: test the cached axis first, then face A, face B and edge
   queries, write an axis record (kind, indices, separation).
4. `narrowClip`: reference and incident face, Sutherland-Hodgman clipping, reduction to at most 4 points, feature ids,
   warm-start impulses matched by id from `prevIndex`'s manifold.
5. `prepareContacts` (normal and tangent masses, V4 exponents and shift bytes), `prepareJoints`.
6. Substeps (4): integrate velocities; warm start, push (soft, with bias), integrate positions, relax, per colour;
   then restitution; Box3D's convex path: per-point normal rows, central friction with a disc clamp through a software
   rsqrt, twist friction.
7. `finalize`: pose from the deltas; sleep counter; integer AABB.
8. `hashElements`: one 64-bit hash per element (body, manifold, joint), no atomics; the CPU sums them per category,
   order-free (as `src/hash.c` does).

## Layouts

- **Shapes:** general convex hulls in Box3D's half-edge form (at most 16 vertices, 24 faces, 48 half-edges), local
  points Q8.24, face planes (normal Q1.30, offset Q8.24) computed from the quantised points. Boxes are ordinary hulls;
  about 20% of pile bodies are irregular chunks (a box cut by one to three random planes in the scene generator, then
  hulled).
- **Bodies:** split into state (velocities, deltas, sleep counter), pose (int64 or float position, quaternion), and
  mass (inverse mass and inertia, with V4 exponents) buffers.
- **Manifolds:** ping-ponged between ticks, up to 4 points with persistent feature ids and impulses.
- One layout header shared by Slang and C, plain scalar structs only (E11's `LSTRUCT`), sizes checked against the
  twin's at start.

## The joint (step 6)

Revolute, after Box3D's: a 3×3 point block inverted once per step by Gaussian elimination with reciprocal pivots
(`divClamp`), a 2×2 axis block, speculative soft limits, a speed motor with a torque cap, and a servo (target angle to
speed, inside the step). A software atan2 that snaps its inputs.

## Scenes and acceptance

Each scene is compared with the D twin and with `b3ref2.c`, a Box3D program that shares the scene generator and the
settings (4 substeps, 60 Hz, Box3D's contact hertz and damping). Chaotic scenes run 8 seeds.

| scene | setup | acceptable (F and V4) |
|---|---|---|
| stack10 | 10 cubes of 0.5 m, sleep off and on | **Sleep off, 60 s:** top drift ≤ max(2× Box3D's, 2 mm); yaw ≤ 0.5°; resting speed ≤ max(3× Box3D's, 1 mm/s). **Sleep on:** asleep within 1.5× Box3D's time and stays asleep |
| pile200 | 160 boxes and 40 chunks dropped in a pit | all asleep by tick 3,600 and within 1.5× Box3D's median; no escapes; penetration ≤ 2 cm |
| arm | a servo link and a limited link sweep a stack | angle vs D ≤ 1 mrad over 2 s; gap ≤ 1 mm; limit overshoot ≤ 0.01 rad |
| chip | 0.05 kg at 10 m/s and 40 rad/s | no tunnelling; V4 saturation counters at 0; sleeps |
| ratio | 3 t on 1 kg; a plate on chips | same outcome class as Box3D |
| bounce, ramp | restitution 0.5; friction 0.6 and 0.2 | apex within 5%; the 0.6 box holds; the 0.2 box slides within 2% |
| grid:K | K piles, sleep off | cost per stage only |

Also: over the first 120 ticks, position rms against D ≤ 1e-5 m for F and ≤ 1e-4 m for V4.

## Comparing bits

- Per-element hashes read back, order-free sums per tick per category.
- Reference hash files from the Windows MSVC twin, committed (every tick to 120, then every 60th).
- `--trace T`: step the twin and a GPU dispatch by dispatch to the first differing word, naming buffer, element and
  field.
- `--dispatch-hashes`: per-dispatch hashes, to compare twins across machines.
- A battery of helper vectors (`battery.slang`) runs at the start of every run; any difference fails the run.
- **The subnormal sentinel (F):** the twin checks the CPU's floating-point status after each dispatch (MXCSR DE/UE on
  x64, FPSR on ARM64, or `fetestexcept( FE_UNDERFLOW )`); any subnormal fails the run.

SPIR-V and the twins' C++ are generated once on Windows (`lab.py gen`) and shipped, so each machine tests its own
compiler and driver. Twins build with MSVC and clang-cl on Windows, gcc and clang on aarch64, and a clang UBSan build
(signed overflow, shifts) as a gate.

## Build order

| step | work | about (lines) | gate |
|---|---|---|---|
| 1 | `num.slang` helpers + battery kernel | 600 | battery identical on the 3060, the UHD, MSVC and clang-cl; `lpDivQ31` equals C's `/` |
| 2 | layouts, scene generator, quantisation, CPU stages, twin driver | 1,500 | stack and pile scenes build and run on the twin (integration, broadphase, colouring, islands), same hashes on MSVC and clang-cl |
| 3 | `narrowSat`, `narrowClip` | 700 | on a corpus of 10k poses, F and V4 pick D's axis and feature ids outside ties |
| 4 | prepare, solve, restitution, body kernels | 800 | |
| 4b | metrics + `b3ref2` | 700 | **decision point 1** |
| 5 | Vulkan tick path, hash array, trace | 600 | **decision point 2** |
| 6 | revolute joint, arm, chip | 550 | |
| 7 | grid:K timing; Spark, then Mac | 150 | |

## Rules (docs/determinism-rules.md, plus the lab's)

- No contraction anywhere (`-ffp-contract=off`, `/fp:precise`); no fast math; no C-library transcendentals (the
  scene generator uses + - * / and sqrt only).
- No side effects inside one expression's operands or one call's arguments: draw random numbers in separate
  statements (the lab's E11 port found exactly this bug).
- Index-order iteration, total-order sorts, integer CPU stages.
- A NaN or infinity anywhere is a failure: NaN bits differ by machine (x64 0xffc00000, ARM 0x7fc00000, NVIDIA
  0x7fffffff; the lab's Fnocap run).
