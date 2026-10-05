# Milestone 11: the GPU lab's results (float dialect or block-scaled integers)

Written 2026-10-04 (overnight), and kept current through milestone 11. Question (docs/roadmap.md §11): can a rigid-body core on
the GPU be bit-identical on every GPU and CPU twin, and in which arithmetic: the float dialect F or block-scaled int32
(V4)? The code is `lab/gpu` (its README says how to run it); raw logs and a summary per machine are in
`lab/gpu/results/<machine>/`. Milestone 7's E11 (`m7-gpu-experiments.md`) is the starting point and is not repeated
here.

## Summary so far

- **E11 holds on two more GPUs, a second CPU architecture and two more compilers.** E11's unchanged kernels were
  regenerated (byte-identical) and run on the DGX Spark: the GB10 (Blackwell, Linux driver 580.82) and llvmpipe
  (Mesa's CPU Vulkan, LLVM 20), with gcc 13 and clang 18 twins on aarch64. Every twin on every compiler and ISA
  reproduces E11's Windows hashes at 1k, 10k and 100k contacts. On every GPU, F (with no float-control modes), I32 and
  V4 match their twin bit for bit.
- **The undocumented NVIDIA route is unusable.** E11 found that declaring `RoundingModeRTE` makes NVIDIA's driver
  keep fp32 subnormals and round division correctly. The GB10's probe shows the same, but the GB10's shader compiler
  **crashes** (a segfault inside `libnvidia-gpucomp`) building E11's `integratePositions` whenever RTE is declared.
  A dialect must be bit-exact with no float-control modes at all ("F-plain"), which F is.
- **NaN bits are each machine's own.** F without its angular speed cap (Fnocap) blows up and makes NaNs: x64 writes
  `0xffc00000`, ARM `0x7fc00000`, NVIDIA's GPUs `0x7fffffff`, so its hashes hold on x64 only. A float core must make
  NaN impossible, not merely tolerate it.
- **Q32.32 (I64) runs correctly on the GB10 and llvmpipe.** Its miscompile was the Intel Gen9.5 driver's alone.
- **Giving up fast math on the GPU is nearly free here.** `slangc -fp-mode fast` (contraction allowed) saves 1.7% of
  the 100k solve on the GB10 (4.73 against 4.81 ms per step), 1-3% on the RTX 3060 and the UHD (5.57 against 5.65,
  31.7 against 32.6), and nothing measurable on the row benchmark on either NVIDIA GPU. The fast-math kernels differ from the twin in about one word in ten, as they should (the positive
  control).
- **Integers cost what E11 said.** V4 costs 1.25-1.36× F per step at 100k contacts on every GPU measured, and I32
  1.07-1.20×.

## Machines and drivers

| machine | GPUs (driver) | CPU twin compilers |
|---|---|---|
| win-laptop | RTX 3060 Laptop (NVIDIA 610.60), Intel UHD 630 (101.2137) | MSVC 19.42, clang-cl 23.1 (x64) |
| spark-gb10 | NVIDIA GB10 (580.82.09), llvmpipe (Mesa, LLVM 20.1.2) | gcc 13.3, clang 18.1 (aarch64, Grace) |
| mac-m5 | Apple M5 (MoltenVK 1.4) | Apple clang 21 (arm64; x64 under Rosetta 2) |

Pending: a Pascal GPU (GTX 1060) and an AMD GPU (the owner's low-end box). The Pascal number gates milestone 12, not
this milestone.

## Bit agreement (E11's full solve, 60 steps; mismatching words against the twin at 1k / 10k / 100k contacts)

| variant | RTX 3060 | UHD 630 | GB10 | llvmpipe |
|---|---|---|---|---|
| F (no modes; Intel with DenormPreserve) | 0 / 0 / 0 | 0 / 0 / 1 (one -0, E11's) | 0 / 0 / 0 | 0 / 0 / 0 |
| F-any (no modes on any GPU) | 0 / 0 / 0 | 0 / 0 / 1 | 0 / 0 / 0 | 0 / 0 / 0 |
| F-ieee (NVIDIA RTE+SZINP, Intel preserve+SZINP) | 0 / 0 / 0 | 0 / 0 / 1 | compiler crash | not reached (the run stops at the crash) |
| Fdisc (Box3D's sqrt and division) | 4,222 / 44,465 / 432,644 | 4,114 / 43,975 / 426,541 | 4,222 / 44,465 / 432,644 | 0 / 0 / 0 |
| I32 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| V4 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 | 0 / 0 / 0 |
| I64 (Q32.32) | 0 / 0 / 0 | 5,911 / 89,650 / 899,797 (miscompiled) | 0 / 0 / 0 | 0 / 0 / 0 |

The GB10 and the RTX 3060 make the same wrong Fdisc words (the same counts): two NVIDIA generations on two driver
branches agree with each other where they disagree with IEEE. llvmpipe's sqrt and division are correctly rounded.

The twins: MSVC and clang-cl on x64, gcc and clang on aarch64, at 1 and 8 threads, give E11's hash for F, Fdisc, I32,
I64 and V4 at every size, and for the five row formats. The FMA positive control (the twin built with contraction
allowed) differs on both ISAs.

## The probe (each float operation, ~1M operand triples in eight classes)

- **The GB10's counts equal the RTX 3060's, operation by operation**, under every mode: fp32 subnormals flushed by
  default (no DenormPreserve advertised); `RoundingModeRTE` turns on preservation and correctly rounded division
  (only NaN payloads then differ); sqrt and inversesqrt 1 ulp off in every mode; `mad()` fused; select max rewritten
  unless SignedZeroInfNanPreserve.
- **llvmpipe** keeps subnormals by default; its division and sqrt are correctly rounded, inversesqrt approximate;
  `max()` and `min()` get ±0 wrong unless SignedZeroInfNanPreserve is declared; `fma()` is not a single rounding; and
  it folds `0 - a` into `-a` (wrong for a = +0) under every mode. `a + 0` is folded by Slang itself (E11).
- **The laptop under the new NVIDIA driver (610.60):** the probe log is identical to E11's under 581.95, count for
  count.

## Costs (ms per step at 100k contacts, GPU timestamps; the twin on the CPU)

| | F | I32 | V4 | I64 | V4 / F |
|---|---|---|---|---|---|
| RTX 3060 | 5.65 | 6.38 | 7.04 | 9.73 | 1.25 |
| UHD 630 | 34.2 | 41.1 | 42.7 | (wrong) | 1.25 |
| GB10 | 4.81 | 5.15 | 6.55 | 8.65 | 1.36 |
| llvmpipe (20 Grace cores) | 52.2 | 55.6 | 63.0 | 88.1 | 1.21 |
| Apple M5 | 7.01 | 5.34 | 6.26 | 14.9 | 0.89 |
| twin, Grace, 1 / 8 threads | 67.6 / 17.7 | 108.7 / 19.1 | 126.4 / 22.0 | 301.8 / 42.0 | 1.87 / 1.25 |

The laptop's twin timings in this run were taken while another build was compiling; E11's (F 118 / 25 ms at 1 / 8
threads) stand for the laptop. Multiply throughput (mulbench), relative to an fp32 fma: an int32×int32→int64 product
with a shift costs 5.7× on the RTX 3060, 8.1× on the GB10 and the UHD; Pascal, with no full-rate int32 multiply, is
predicted worse (the gate before milestone 12).

## Port notes (what made E11 portable)

- **Unsequenced random draws.** E11's `rows.c` and `probe.c` drew random numbers inside one call's arguments and one
  product's operands, so clang made a different corpus from MSVC. The port draws in the order MSVC happened to use:
  mostly right to left, but x, z, y for one call's three box extents and left to right at two probe sites (found by
  trying the orders against E11's hashes and log). This is docs/determinism-rules.md's rule against side effects in
  one call's arguments, broken in research code.
- **A reference left to the C library.** The probe's CPU reference for `max()`/`min()` was `fmaxf`/`fminf`, whose
  sign for max(-0, +0) differs between MSVC and clang; it is written out.
- **Contraction:** gcc and clang contract by default on aarch64, so every lab file is built with
  `-ffp-contract=off`, the scenario generators included.

## The dual-dialect toy

A real rigid-body step written once in Slang (`lab/gpu/toy`, its DESIGN.md and NOTES.md): the float dialect F, V4
and a double reference D; a SAT narrowphase with its cache as data, clipping and feature ids, persistent warm starts,
Box3D's soft step with friction, twist and restitution, contact recycling, a revolute joint with a motor, a servo and
limits, islands and sleep, the combinatorial stages in integer C. Built step by step by implementation agents, each
step gated, then reviewed by Fable and the Codex reviewer, whose fixes are in (below).

**Decision point 2, bits (gate 5).** The whole tick on the GPU against the CPU twin, every tick's hash of every word
the tick writes (bodies, poses, masses, AABBs, SAT records, constraints, manifolds, joints, the stages' lists), plus
a full-buffer trace of the pile and the arm on every GPU: 16 runs per dialect (stack10 for 3,600 ticks asleep and
awake, pile200 seeds 1 to 8 for 1,200 ticks, bounce, ramp, ratio, chip, the arm asleep and awake).

| | F (no float-control modes) | V4 |
|---|---|---|
| RTX 3060 | every tick identical | every tick identical |
| Intel UHD 630 | every tick identical | every tick identical |
| GB10 | every tick identical | every tick identical |
| llvmpipe | every tick identical | every tick identical |
| Apple M5 (MoltenVK; also from x64 under Rosetta 2) | every tick identical | every tick identical |
| twins: MSVC, clang-cl (x64); gcc, clang (aarch64); Apple clang (arm64, and x64 under Rosetta 2) | the reference files | the reference files |

Every row is the final code (after the reviews), with the full-buffer traces showing no differing word on any GPU
(`lab/gpu/toy/results/gate5-windows.txt`, `lab/gpu/results/spark-gb10/toy-gate.txt`). The helper battery (25 helpers) and the narrowphase corpus (10,000 hull
pairs, fresh and warm-started; F and V4 choose D's axis, points and ids on every pair outside ties) are word-identical
on all five GPUs (on the M5 up to signed zeros in F's raw helpers, which the snap absorbs). A clang UBSan build of the twins runs gates 1 to 4 clean (it found one V4 overflow, now fixed).

**Decision point 1, physics (`lab/gpu/toy/results/decision1.md`).** DESIGN.md's acceptance table against Box3D and D:

| | Box3D | D | F | V4 |
|---|---|---|---|---|
| DESIGN.md's rules as written | 17 of 18 | 17 of 18 | 24 of 27 | 24 of 28 |
| with the rows judged post hoc | 18 of 18 | 18 of 18 | 26 of 27 | 27 of 28 |

- **What passes everywhere:** a 10-box stack stands 60 s and sleeps at Box3D's tick; 200-body piles sleep on 8 seeds
  with no escapes; the ramp slides within 0.51% of Box3D; bounce, ratio and chip reach Box3D's outcomes; the arm's
  angles, hinge gaps and limit overshoot are within their bars (Box3D's servo angle is 0.58 mrad from D's, F's 0.0001,
  V4's 0.03).
- **What misses as written:** the 120-tick position rms against D (1e-5 m for F, 1e-4 m for V4) on the impact scenes:
  chip for both (F 4.05e-5, V4 1.5e-4: arithmetic in the contact solve at the first impact, amplified by the tumbles),
  ratio for V4 (1.43e-4: lost at tick 1 under a 2,000:1 mass ratio, not yet traced to a format), the arm (its stack
  parts from D's on face-on ties at tick 1) and pile200 (chaotic: D against itself moved 1e-9 m differs by 0.017 m).
  The penetration bar in pile200's original window fails for Box3D (2.04 cm) and D (3.21 cm), not for F or V4.
- **Honesty about the table:** both reviewers found that step 4b had narrowed the rms rule and moved the penetration
  window after seeing the results. The table now judges DESIGN.md's rules as written and keeps the later judgements
  as separate rows labelled post hoc.

**Costs (`lab/gpu/toy/results/grid.md`, K copies of pile200, sleep off).** At one pile (about 230 dispatches a tick)
both GPUs are dispatch-bound: 92% of the RTX 3060's kernel time is fixed cost per dispatch, 6 µs in F and 13 µs in V4
(longer kernels), so V4's tick is 1.7 times F's. At 64 piles (13,057 bodies) V4 costs the same as F per body on the
3060; the tick (32 ms on the 3060, 165 ms on the UHD) is then dominated by the single-threaded CPU stages (15 ms) and,
on the UHD, by readbacks made one region per body (66 ms). E11's solve alone: V4 1.25-1.36 times F.

**What the toy found about the tools:**
- **gcc 13.3 miscompiles a float comparison** at -O2 and -O3 (aarch64; `lab/gpu/repro/gcc13-backprop.c`, 25 lines):
  its backprop pass strips the negation in max(0, -x) because only the square of the result is used, keeps the old
  range, and VRP folds a live comparison to false. F's gcc twin lost contact points; clang, MSVC and every GPU agreed.
  The pass rewrites only floats, so V4 was immune. Cross-compiler twins caught it at once.
- **The Intel UHD 630's driver misread a variably indexed array** of manifold points once the manifold grew (661 wrong
  manifolds, in F); reading them from the buffer fixed it. E11's Intel int64 miscompile was the same driver.
- **MSVC folds `x < 0 ? -x : x` into fabs** in C (so -0 becomes +0); clang does not.
- **clang raises floating-point flags the source never raises** (a guarded division speculated, an unused vector
  lane), so the twins are built with `-ffp-exception-behavior=maytrap`.
- **Rounding bias in V4:** products rounded half up leaned an aligned stack (exact negative halves); rounding half
  away from zero cut the bottom cube's creep from 50 to 0.7 µm in 50 s.
- **F's Newton steps** written y (3/2 - x y^2 / 2) gave rsqrt(1) = 1 - 2^-24, enough to make resting orientations
  drift; written as a correction, y + y (1/2 - x y^2 / 2), they are exact at powers of four.
- **Contact recycling is load-bearing:** without Box3D's recycling an exactly aligned stack loses its warm start every
  other tick and slides, in every dialect and in Box3D itself.

## The MacBook (Apple M5, MoltenVK)

macOS 27.0.1 on an Apple M5; Vulkan through MoltenVK (Homebrew's, and LunarG's SDK 1.4.363 for its universal loader
and driver), run with `MVK_CONFIG_FAST_MATH_ENABLED=0`; Apple clang 21 twins on arm64, and the same twins built for x64
and run under Rosetta 2. Results in `lab/gpu/results/mac-m5/`.

- **E11:** F, I32, V4 and I64 bit-exact against the twin at 1k, 10k and 100k contacts; Fdisc differs (825 / 8,056 /
  83,452 words), as on NVIDIA. The M5 runs Q32.32 correctly.
- **The probe:** the M5 flushes fp32 subnormals by default (no DenormPreserve advertised), returns +0 where IEEE gives
  -0 for -0 times x, and honours NoContraction: every a*b+c difference is a product flushed before the add. E11's row
  benchmark, which does not snap, differs in 28 rows, every one a signed zero.
- **MoltenVK's fast-math switch** (`MVK_CONFIG_FAST_MATH_ENABLED=1`) changed no word of F's solve or rows: the kernels
  carry NoContraction and use no operation Metal's fast math rewrites. Slang's own fast-math build (the positive
  control) differs, as everywhere.
- **The toy:** gates 1 to 5 pass on the M5, every tick identical to the twin in F and V4 across the 16 runs, the
  full-buffer traces clean, the narrowphase corpus word-exact. F's raw helpers in the battery differ from the twin
  only in zero signs (the M5's multiply: 1,080 vectors), which F's snap at selects and stores absorbs; the battery
  counts them apart.
- **Rosetta 2:** the x64 twins reproduce every hash (E11's, the battery's, the corpus's, every scene's reference file),
  and the x64 build driving the M5 through Rosetta and MoltenVK's x64 driver is bit-exact too (E11 and the toy's 16
  runs per dialect).
- **Integer multiplies are cheap on Apple's GPU:** an int32×int32→int64 product with a shift costs 1.1-1.3 times an
  fp32 fma (5.7-8 times on the NVIDIA and Intel GPUs), and V4 runs E11's 100k solve faster than F (6.26 against 7.01
  ms per step; I32 5.34, I64 14.9). The M5's CPU runs the F twin in 24 ms at one thread (the Grace 68, the laptop 118).

## Not done

- **The CUDA leg** (the plan's optional item: F through `slangc -target cuda` and nvcc with `--fmad=false` against
  the twin, `--fmad=true` as the control) was skipped: Vulkan is the path a game ships on, and the GB10's Vulkan
  results already cover its hardware.
