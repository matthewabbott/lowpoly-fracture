# Milestone 11: the GPU lab's results (float dialect or block-scaled integers)

Written 2026-10-04, and kept current through milestone 11. Question (docs/roadmap.md §11): can a rigid-body core on
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

Pending: the MacBook (Apple GPU through MoltenVK; Apple clang; x64 under Rosetta 2), a Pascal GPU (GTX 1060) and an
AMD GPU (the owner's low-end box). The Pascal number gates milestone 12, not this milestone.

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
Box3D's soft step with friction, twist and restitution, contact recycling, islands and sleep, the combinatorial
stages in integer C. Built step by step, each step gated.

- **The arithmetic helpers** (battery, 19 helpers): every word identical on the RTX 3060, the UHD 630, the GB10,
  llvmpipe and the MSVC, clang-cl, gcc and clang twins, in F (no float-control modes) and V4; V4's exact division
  `lpDivQ31` equals C's `/` on a million inputs.
- **The narrowphase** (10,000 hull pairs, fresh and warm-started): F and V4 choose D's separating axis on every pair
  outside ties (9,523) and D's points and feature ids on every pair outside clipping ties (9,187); every word of the
  SAT records and manifolds matches the twin on all four GPUs.
- **Decision point 1, physics against Box3D** (`lab/gpu/toy/results/decision1.md`): V4 passes all 20 judged rows of
  the acceptance table, F 18 of 19, and Box3D and D all of theirs. A 10-box stack stands 60 s and sleeps at the same
  tick as Box3D's; 200-body piles sleep on 8 seeds with no escapes and rest within 1.7 cm; the ramp slides within
  0.51% of Box3D. F's one miss (the ramp's position rms against D over 120 ticks, 1.34e-5 m against a 1e-5 bar) is two
  thirds the start rounded to float. V4's stack crept 93 µm in 50 s (within the bar) because its products rounded
  exact negative halves up.

**What the toy found about the tools:**
- **gcc 13.3 miscompiles a float comparison** at -O2 and -O3 (aarch64; `lab/gpu/repro/gcc13-backprop.c`, 25 lines):
  its backprop pass strips the negation in max(0, -x) because only the square of the result is used, keeps the old
  range, and VRP folds a live comparison to false. F's gcc twin lost contact points; clang, MSVC and every GPU agreed.
  Only floats are rewritten by that pass, so V4 was immune. Cross-compiler twins caught it at once, which is the case
  for keeping them.
- **The Intel UHD 630's driver misread a variably indexed array** of manifold points once the manifold grew (661 wrong
  manifolds); reading them from the buffer fixed it. E11's Intel int64 miscompile was the same driver.
- **MSVC folds `x < 0 ? -x : x` into fabs** in C (so -0 becomes +0); clang does not.
- **clang raises floating-point flags the source never raises** (speculating a guarded division, or an unused vector
  lane), so the twins are built with `-ffp-exception-behavior=maytrap`; gcc's default is the same.
- **F's Newton steps** written y (3/2 - x y^2 / 2) gave rsqrt(1) = 1 - 2^-24, enough to make resting orientations
  drift; written as a correction, y + y (1/2 - x y^2 / 2), they are exact at powers of four.

## Pending: the MacBook

What the Mac answers that no other machine here can: Apple's GPU through MoltenVK (Vulkan translated to Metal by
SPIRV-Cross), Apple clang on arm64, and an x64 twin under Rosetta 2.

**Setup (the owner wakes it; about 20 minutes once):**
- key-based ssh from the laptop: `ssh <user>@mbas-macbook-pro`;
- the Xcode command-line tools (clang, make, python3) and CMake (`brew install cmake`);
- the LunarG Vulkan SDK for macOS (MoltenVK, a universal Vulkan loader, vulkaninfo), sourced in the ssh session
  (`setup-env.sh`); the universal loader lets an x64 build link under Rosetta.

**Runs (`lab.py remote` once the setup works):**
1. E11 on the Apple GPU: the probe, the solve matrix, the rows, mulbench, with Apple clang twins.
   - Does Metal flush fp32 subnormals by default (reported, unverified)?
   - Are selects rewritten, and is `0 - a` folded?
   - Does sqrt round correctly?
   - Is int64 multiplication correct? Apple's GPUs have no native 64-bit multiply, so it is emulated.
   - What do V4 and I32 cost against F?
2. MoltenVK with `MVK_CONFIG_FAST_MATH_ENABLED=0`, then `=1` as a positive control. Metal compiles with fast math
   unless told otherwise, and SPIR-V's NoContraction has no per-operation equivalent in Metal's language, so this is
   the run most likely to break the float dialect.
3. The twins built for x64 (`CMAKE_OSX_ARCHITECTURES=x86_64`) and run under Rosetta 2, without GPUs: E11's hashes must
   hold, as the engine's CI found for the engine itself.
4. The toy, when its Vulkan path exists (step 5): the battery, then the scenes in both dialects on the Apple GPU
   against the twins.

## Not done

- **The CUDA leg** (the plan's optional item: F through `slangc -target cuda` and nvcc with `--fmad=false` against
  the twin, `--fmad=true` as the control) was skipped: Vulkan is the path a game ships on, and the GB10's Vulkan
  results already cover its hardware.
