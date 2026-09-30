# M7 research, track T8: hands-on determinism experiments

Dates: 2026-09-29 and 30. Machine: i7-10870H (8 cores, AVX2 and FMA3), RTX 3060 Laptop GPU and Intel UHD Graphics,
Windows 11 Home 22621. Local runs used branch `sandbox` at 5445925 (engine code identical to today's `sandbox`,
d08be6a, which changed only docs). Worktree: `D:\a\programming\lowpoly-fracture\.claude\worktrees\agent-ae4c0bdc70359f14e`,
branch `ci-determinism` (pushed; it holds only the commits listed in E5, rebased onto d08be6a). Toolchains: MSVC
19.42.34435 (VS 2022 Build Tools), clang-cl 23.1.2 (winget LLVM.LLVM, already installed on 2026-09-22, so nothing
was installed), CMake 4.4.3, Ninja 1.13.2; `gh` 2.101.0 was already installed and authenticated. Raw logs:
`D:\a\programming\lowpoly-fracture\build\m7\` (gitignored). Scratch harnesses: the session scratchpad `m7\exp\`
(`m7h.c`, `rt.c`, `gpuprobe.cpp`, `build-harness.ps1`, `ladder.ps1`, `ticklogs.ps1`, `variant.ps1`).

Scope as narrowed: E4, E1, E9, E5. E2 and E10 had already run; their results are short appendices. E6 and E7 were
dropped (docs/research/m7-state-audit.md has those numbers).

## Summary

| experiment | result |
|---|---|
| E4 MSVC vs clang-cl, contraction off | identical: 12 rungs x workers 1 and 8, final world and solver hashes and every per-tick hash |
| E4 controls | MSVC `/arch:AVX2` and clang-cl `/arch:AVX2` with contraction off: identical (no FMA emitted). clang-cl with its default contraction: all 12 rungs differ |
| E4 module swap | FMA in our core only, or in Box3D only: both diverge, at scene build or at the first fracture |
| E1 Box3D SIMD vs scalar | identical on all 24 runs |
| E9 FTZ/DAZ set mid-run | world hash unchanged on 10 of 12 rungs; the stress-solver hash diverges in town (tick 577) and barrage (tick 127); barrage's world hash differs on 10 ticks, then re-converges. Denormals occur |
| E5 CI, default build | runs cluster by C library, not by compiler or CPU: UCRT x64 (MSVC, clang-cl), UCRT ARM64, glibc (5 Linux legs, x64 and ARM64, gcc and clang, and box64), Apple libm (arm64 and Rosetta). No leg outside UCRT x64 matches the Windows reference |
| E5 CI, `LPF_PORTABLE_MATH=ON` + the argument-order fix | every native leg (10: Windows x64 MSVC 19.51 and clang-cl 22, Windows ARM64 MSVC, Linux x64 gcc 13 / clang 18 / gcc `-march=native`, Linux ARM64 gcc / clang, macOS arm64 AppleClang 21, macOS x86_64 under Rosetta 2) and box64 give the Windows MSVC hashes on all 24 runs, byte-identical per tick (601 lines per run). The contraction-on control differs everywhere and fails a tuned test |
| E5 new hazard | two call sites drew random numbers inside one call's arguments (unspecified evaluation order in C): MSVC and gcc x64 differ from clang and gcc ARM64. Fixed on the branch in MSVC's order, MSVC hashes unchanged |
| E5 emulators | Rosetta 2: identical to native arm64 in both builds (and to everything in the portable build). Prism: the Windows x64 binaries identical every tick. box64 with `FASTNAN=0 FASTROUND=0`: identical to the same binary on x64, every tick. box64 defaults: identical except one transient line (track's post-build hash) that the first step erases. Both box64 and Prism route some C-library calls to native ARM64 code |

Hypotheses:

- **H1** (Box3D float with pinned flags, not pinned compilers, bit-exact across MSVC, clang, gcc on x64 and ARM64;
  residual hazards ours): **confirmed**, on this bench. Five compilers from three vendors, two ISAs and three OSes
  agree bit for bit once our three residual hazards are fixed: gcc without `-ffp-contract=off` (the ARM64 control
  shows it matters), C-library `cbrtf`/`atan2f`/`sinf` in simulation and scene code, and random draws inside a
  call's argument list. Box3D needed no patch: the NEON min/max sign-of-zero concern did not fire in 12 rungs x
  600 ticks with four ARM64 toolchains (MSVC, gcc, clang, AppleClang). Compilers were not pinned: MSVC 19.42 and
  19.51, clang-cl 22 and 23, clang 18, AppleClang 21 and gcc 13 all agree.
- **H6** (x64 SSE2 bit-exact under Rosetta 2 and Prism by default, and under box64/FEX with fast modes off):
  **confirmed for Rosetta 2, Prism and box64** (box64 with fast modes off: every tick; with defaults: final hashes
  too, with one transient post-build difference), FEX not tested. Caveat, found here: both box64 and Prism hand
  C-library calls to native ARM64 code. box64 wraps libm (its `atan2f` probe equals ARM64 glibc's, not x64 glibc's);
  under Prism, a dynamically linked `ucrtbase.dll` is not x64 code (the `/MD` probe's `cbrtf` equals ARM64 UCRT's,
  the `/MT` probe's equals x64's). The default build is exact under both only because glibc's `cbrtf` is the same
  on both ISAs (box64) and the engine links the CRT statically (Prism). A libm-free simulation is safe under both.
- **H5**: nothing here contradicts it. E9 adds that a desync hash must include the stress solver's state (the world
  hash stayed equal for 600 ticks while the solver had diverged), and that each peer needs an MXCSR/FPCR guard.
- **GPU** (Appendix A, run before the scope narrowed): D3D11 cannot match the CPU as a flag set.

## Harness

`m7h.c` builds a scene exactly as `lpf_bench` does (`b3DefaultWorldDef` with gravity -10, `lpDefaultWorldDef`,
`maxFullDebris` 400), runs `lpSceneBombard` and `lpSceneDrive` every tick, steps at 1/60 with 4 substeps, and logs
`load <world> <solver>` and then `<tick> <lpWorld_Hash> <lpWorld_HashStress>` for every tick. Its final hashes equal
`lpf_bench`'s on every rung. `build-harness.ps1 -Name <exe> -Cc cl|clang-cl -Box3d <build dir> -Core <build dir>`
links `box3d.lib` from one build and `lpf.lib`/`lpf_scenes.lib` from another (the module swap). `ladder.ps1` runs
the `tools/bench.ps1` rungs with `lpf_bench` (walls, town, pile, lumber, tower, ruins, yard, keep at period 12;
barrage = town at 3; siege = keep at 4; track and mech at 30; 600 ticks; workers 1,8); `ticklogs.ps1` runs them
with the harness. The CI uses the same rungs through `.github/determinism/ladder.sh` and `lpf_bench --hash-log`.

## E4: MSVC vs clang-cl

```powershell
pwsh tools/build.ps1                                   # msvc-release
pwsh tools/build.ps1 -Preset clang-release -Fresh      # after the two build fixes below
pwsh <scratch>\ladder.ps1 -Exe build\msvc-release\bin\lpf_bench.exe  -Out <m7>\e4\msvc
pwsh <scratch>\ladder.ps1 -Exe build\clang-release\bin\lpf_bench.exe -Out <m7>\e4\clang
pwsh <scratch>\build-harness.ps1 -Name m7h_msvc  -Cc cl       -Box3d msvc-release  -Core msvc-release
pwsh <scratch>\build-harness.ps1 -Name m7h_clang -Cc clang-cl -Box3d clang-release -Core clang-release
pwsh <scratch>\ticklogs.ps1 -Exe <scratch>\bin\m7h_msvc.exe  -Out <m7>\e4\tick_msvc
pwsh <scratch>\ticklogs.ps1 -Exe <scratch>\bin\m7h_clang.exe -Out <m7>\e4\tick_clang
```

The `clang-release` preset did not build as committed: `lpf_warnings` passed `-Wall` to clang-cl, which reads it as
`/Wall` = `-Weverything` (padding, unsafe-buffer and C89 warnings, as errors); with `/W4` for the clang-cl front end,
`-Wunused-function` then flagged `lpIsSlender` in stress.c, which nothing calls. Both fixes are on `ci-determinism`
and change no MSVC code.

Result: final world and solver hashes identical between MSVC and clang-cl on all 12 rungs at workers 1 and 8, and
equal to `bench/baseline.json`:

| rung | world hash | solver hash | max pieces |
|---|---|---|---|
| walls | 900e17c146db5348 | e198765403b14f38 | 1344 |
| town | 39824554bc44b0f7 | 00034bc1a7ed2273 | 7582 |
| pile | f38a3b6505d95c2b | cd58843c0be30a65 | 1917 |
| lumber | dbd1945ad48d4c31 | 1362aebd946f7ba6 | 312 |
| tower | dc562b211bfbebaa | e492ebbed2ffbc1d | 5648 |
| ruins | f470c1ef8ced8007 | ccf6488377a291fa | 158 |
| yard | d4b4a0b5acc2272f | 2652a68338bcf191 | 271 |
| keep | d4574cf06c5718fc | c3c990710dc66a47 | 4231 |
| barrage | c48a19d345fd6be9 | f18b035b497d51c9 | 16339 |
| siege | b8baed95b8913c82 | da32c3508db0f02e | 8766 |
| track | f640f9fe61cf977a | 5e1db4e31e745be6 | 593 |
| mech | 7541c412fbe7a82f | d79a934215670d5d | 85 |

Per tick, the 12 harness logs (601 lines each, world and solver hash) are byte-identical between the two builds.
With no mismatch there was nothing to bisect.

Controls. Variant build dirs (`variant.ps1`: `LPF_SANDBOX=OFF`, `LPF_TESTS=OFF`); FMA instructions counted with
`dumpbin /disasm` on each library (pattern `vfn?m(add|sub)[0-9]{3}[sp]s`):

| build | flags | FMAs in box3d / lpf / scenes | vs MSVC |
|---|---|---|---|
| msvc-avx2 | `-DCMAKE_C_FLAGS="/DWIN32 /D_WINDOWS /arch:AVX2"`, `/fp:precise` | 0 / 0 / 0 (45,004 VEX arithmetic ops in box3d) | identical, 24/24 |
| clang-avx2 | `CCC_OVERRIDE_OPTIONS="# ^/arch:AVX2"`, contraction off | 0 / 0 / 0 | identical, 24/24 |
| clang-avx2-contract | `CCC_OVERRIDE_OPTIONS="# ^/arch:AVX2 x/clang:-ffp-contract=off"` (clang-cl's default contraction) | 9107 / 2450 / 393 | all 12 rungs differ (e.g. town 5f80d1fd8f54d7f2, max pieces 7383), w1 = w8 |

Module swap on the contraction build (harness linked against mixed libraries, per-tick logs compared with MSVC):

| harness | FMA in | first differing line |
|---|---|---|
| m7h_fma | everything | `load`, all 12 rungs |
| m7h_fmacore | lpf and scenes only | `load`, all 12 rungs |
| m7h_fmabox3d | Box3D only | `load` on 9 rungs; walls and keep at tick 6, siege at tick 2 (their first shot) |

No module can go without the flag: a contraction difference in either half shows at scene build or at the first
fracture.

cbrtf. MSVC and clang-cl call the same Universal CRT `cbrtf`, so E4 cannot show a libm difference. The libm probe
(`.github/determinism/libm_probe.c`, same hashes with both compilers) measures UCRT against the double function
rounded to float: `cbrtf` differs for 132,700 of 491,520 inputs (27%), `sinf` for 388 and `cosf` for 370 of
589,824, `sqrtf` and `atan2f` for none. The scratch replacement became the research option `LPF_PORTABLE_MATH`
(default OFF): `lpCbrt` in src/core.c (bit-trick seed, five Newton steps in double, only `+ - * /`), and `lpCbrt`,
`b3Atan2` and `b3ComputeCosSin` in scenes.c instead of `cbrtf`, `atan2f`, `sinf`. With it ON, MSVC and clang-cl
agree on all 24 runs (new hashes, e.g. town 39a2910b5078ebf3). E5 shows what it does across OSes.

## E1: Box3D SIMD vs scalar

```powershell
pwsh <scratch>\variant.ps1 -Dir msvc-nosimd -Cc cl -Defs '-DBOX3D_DISABLE_SIMD=ON'   # extern/box3d/CMakeLists.txt:91
pwsh <scratch>\ladder.ps1 -Exe build\msvc-nosimd\bin\lpf_bench.exe -Out <m7>\e1\nosimd
```

`compile_commands.json` carries `BOX3D_DISABLE_SIMD` on 50 Box3D units (so `B3_SIMD_NONE`, src/core.h:50). All 24
runs are identical to the SSE2 build, world and solver hashes. If the scalar `b3MinW`/`b3MaxW` (`<=`) and `b3AbsW`
differ from SSE2 on the sign of a zero, it never reached a hash or a decision in 12 x 600 ticks. The NEON path is
tested by the ARM64 legs of E5: with the C-library calls gone, Windows ARM64 MSVC, Linux ARM64 gcc and clang and
macOS arm64 all match x64 on every tick.

## E9: MXCSR tampering

The harness sets `_mm_setcsr(_mm_getcsr() | 0x8040)` (FTZ and DAZ) on the calling thread before the step of tick N.

```powershell
pwsh <scratch>\ticklogs.ps1 -Exe <scratch>\bin\m7h_msvc.exe -Out <m7>\e9\ftz0_w1   -Workers 1 -Extra '--ftz-at 0'
pwsh <scratch>\ticklogs.ps1 -Exe <scratch>\bin\m7h_msvc.exe -Out <m7>\e9\ftz300_w1 -Workers 1 -Extra '--ftz-at 300'
pwsh <scratch>\ticklogs.ps1 -Exe <scratch>\bin\m7h_msvc.exe -Out <m7>\e9\ftz0_w8   -Workers 8 -Extra '--ftz-at 0'
```

Per tick, against the clean 1-worker logs:

| run | unchanged rungs | town | barrage |
|---|---|---|---|
| FTZ from tick 0, 1 worker | 10 of 12 | solver hash differs from tick 577 (23 ticks); world hash never | solver hash from tick 127 (340 ticks); world hash on ticks 323-332 only; final world hash equal |
| FTZ from tick 300, 1 worker | 10 of 12 | solver from 577 | solver from 300, world 323-332 |
| FTZ from tick 0 on the main thread only, 8 workers | 10 of 12 | solver from 578 | solver from 131 |

Denormals occur in practice: in the stress solver (structures re-solving under many blasts: barrage is town with a
blast every 3 ticks) and briefly in body or ghost state (a velocity decaying through the subnormal range and then
zeroed by sleep or landing, so the world hash re-converged). No flushed value flipped a decision within 600 ticks,
but a peer with FTZ set fails a solver-inclusive hash check at once and may fork later. The main thread's share of
the stress work is enough for an 8-worker run to diverge. So: include the solver's state in any per-tick
desync check (the world hash alone stayed equal for 600 ticks in town), and guard MXCSR/FPCR at step entry and in
every worker, as T3's `lpFpGuard` proposes.

## E5: cross-platform CI

### Branch and fixes

Branch `ci-determinism`, pushed to origin, five commits on top of `origin/sandbox` (d08be6a); its patches and
messages were checked for game names before every push. Commits: 73cdcbc (portability fixes, `--hash-log`,
`LPF_PORTABLE_MATH`, the workflow), 50f2a61 (let CMake pick the runner's Visual Studio: the Windows images now
carry Visual Studio 2026, so `-G "Visual Studio 17 2022"` failed), 7ef192c (the argument-order fix below; `lpf_test`
failures recorded rather than fatal), 7744bd8 (the Prism leg), 9f0f43b (a static-CRT libm probe on Windows).

| file | change | MSVC behaviour |
|---|---|---|
| CMakeLists.txt | `-ffp-contract=off` for GNU as well as Clang (gcc defaults to `fast`) | unchanged |
| CMakeLists.txt | clang-cl warnings: `/W4` instead of `-Wall` (which clang-cl reads as `-Weverything`) | unchanged |
| src/CMakeLists.txt | lpf links `Threads::Threads` (tasks.c uses pthreads off Windows) | unchanged |
| src/stress.c | remove the unused `lpIsSlender` (`-Wunused-function` under gcc and clang) | unchanged (dead code) |
| src/fracture.c, scenes/scenes.c | random draws taken as statements instead of inside one call's arguments (below) | unchanged (MSVC's order kept) |
| bench/main.c | `--hash-log path`: every tick's world and solver hash to `path.w<N>.txt` | unchanged unless given |
| src/core.h/.c, scenes.c, five call sites | `LPF_PORTABLE_MATH` (default OFF): `lpCbrtf` macro, `lpCbrt`, Box3D trig in scenes | unchanged when OFF |

Verified on Windows after the last engine change: `pwsh tools/build.ps1 -Test` 9/9 suites pass;
`pwsh tools/bench.ps1 -Repeat 1 -StrictSolver` reports every rung `same`, world and solver hash, exit 0. A build with
clang's GNU driver (`-Wall -Wextra -Werror`) is clean. In CI, `lpf_test` passes on all 10 native legs (it fails only
on the contraction control, in `TestWheelLoadsBridge`).

The argument-order hazard. The first CI run showed that with `LPF_PORTABLE_MATH=ON`, Linux x64 gcc and Windows ARM64
MSVC matched Windows x64 MSVC on every rung, while Linux x64 clang, Linux ARM64 gcc and clang, and macOS arm64 all
failed the same four rungs (walls from tick 43, lumber from 332, town and barrage at scene build). The split follows
the order in which compilers evaluate a call's arguments, which C leaves unspecified: MSVC, clang-cl and gcc on x64
behaved as right to left, clang on Linux and macOS and gcc on ARM64 as left to right (inferred from which builds
matched, not from compiler documentation). Two sites drew random numbers inside one call:
`lpFractureSnap` tilts its snap plane with `b3Add( b3MulSV( lpRandom_Range(...), t1 ), b3MulSV( lpRandom_Range(...), t2 ) )`
(src/fracture.c, around line 780; slender pieces snapping), and the town's tree row passes `4.5f + 2.0f * lpUnit( &rng )`
and `lpNext( &rng )` to `lpAddTree` (scenes.c, around line 1699). Taking the draws as statements in MSVC's order
(second argument first) left every MSVC hash unchanged and made all four rungs match everywhere. Braced
initialisers with several draws (`(b3Vec3){ lpUnit(), lpUnit(), lpUnit() }`, a dozen sites) are also
indeterminately sequenced in C, but every compiler here evaluated them in order; they deserve the same treatment
and a rule in docs/determinism-rules.md.

### The workflow

`.github/workflows/determinism.yml`, on push to `ci-determinism` and on `workflow_dispatch`:

- `native` (11 legs): windows-latest MSVC and clang-cl (`-T ClangCL`), windows-11-arm MSVC (ARM64), ubuntu-latest gcc
  and clang, ubuntu-latest gcc with `-march=native` (control: new instructions, contraction still off),
  ubuntu-24.04-arm gcc and clang, ubuntu-24.04-arm gcc with contraction forced on (positive control, through
  `.github/determinism/contract-launcher.sh` as `CMAKE_C_COMPILER_LAUNCHER`), macos-latest arm64, and an x86_64
  build on the same runner run under Rosetta 2 (`arch -x86_64`). Each leg builds the default and the
  `LPF_PORTABLE_MATH=ON` variants and the libm probe (`.github/determinism/CMakeLists.txt`, `libm_probe.c`), runs
  `.github/determinism/ladder.sh` for both variants (every rung at workers 1 and 8, per-tick hash logs), runs
  `lpf_test` (non-fatal) and uploads `det-<leg>`. linux-x64-gcc and windows-x64-msvc also upload their binaries.
- `emulated` (3 legs): the linux-x64-gcc binaries, unchanged, under box64 on ubuntu-24.04-arm (built from source at
  tag v0.4.4 with `-DARM_DYNAREC=ON`, the documented generic-ARM64 build; the alternative is a third-party apt
  repository), with defaults and with `BOX64_DYNAREC_FASTNAN=0 BOX64_DYNAREC_FASTROUND=0`; and the windows-x64-msvc
  binaries on windows-11-arm, where Windows runs x64 code through Prism.
- `summary`: `.github/determinism/summarize.py` diffs every leg against windows-x64-msvc (final hashes, the first
  differing tick from the per-tick logs, libm probe values) into the job summary and a `summary` artifact.

Runs (https://github.com/matthewabbott/lowpoly-fracture/actions/runs/<id>): 36698052017 (cancelled: the Visual
Studio generator name), 36698217278 (complete, before the argument-order fix), 36699101608 (native legs after the
fix; cancelled while its box64 legs ran, superseded), **36699457658 (complete, all 14 legs green: the results
below)**, 36700796929 (adds the static-CRT probe for the Prism question; its native and Prism legs repeat run 36699457658's
hashes, box64 was still running when this was written). A native leg takes 2.5-4.5
minutes, box64 7 (defaults) to 15 (strict) minutes; the whole run about 25 minutes. Artifacts and the summary
job's table are copied to `build\m7\ci\<run id>\` (`summary\summary.md` is the generated table).

### Results (run 36699457658)

Runs matching the Windows x64 MSVC reference (12 rungs x workers 1 and 8; final world and solver hash). Every leg is
deterministic across its own worker counts (no `lpf_bench` exit 2 anywhere).

| leg | toolchain | default build | `LPF_PORTABLE_MATH=ON` |
|---|---|---|---|
| windows-x64-msvc (reference) | MSVC 19.51 (VS 2026), Windows build 26100 | 24/24 = local MSVC 19.42 and bench/baseline.json | 24/24 = local |
| windows-x64-clangcl | clang-cl 22.1.3 | 24/24 | 24/24 |
| windows-arm64-msvc | MSVC 19.51 ARM64 (NEON Box3D) | 6/24 (ruins, yard, mech) | 24/24 |
| linux-x64-gcc | gcc 13.3, glibc | 0/24 | 24/24 |
| linux-x64-gcc-native | gcc 13.3 `-march=native` | 0/24 (= linux-x64-gcc) | 24/24 |
| linux-x64-clang | clang 18.1.3 | 0/24 (= linux-x64-gcc) | 24/24 (16/24 before the order fix) |
| linux-arm64-gcc | gcc 13.3 | 0/24 (= linux-x64-gcc) | 24/24 (16/24 before) |
| linux-arm64-clang | clang 18.1.3 | 0/24 (= linux-x64-gcc) | 24/24 (16/24 before) |
| linux-arm64-gcc-contract | gcc 13.3, contraction on | 0/24 | 0/24 (every rung differs at scene build) |
| macos-arm64-clang | AppleClang 21 | 0/24 | 24/24 (16/24 before) |
| macos-x64-rosetta | AppleClang 21, x86_64 under Rosetta 2 | 0/24 (= macos-arm64) | 24/24 (16/24 before, = macos-arm64) |
| box64-default | the linux-x64-gcc binary under box64 0.4.4 | 0/24; final hashes = linux-x64-gcc, every tick but one (track's post-build line) | 24/24; the same one line differs |
| box64-strict | the same, `FASTNAN=0 FASTROUND=0` | 0/24; = linux-x64-gcc every tick | 24/24, every tick |
| prism-windows-arm64 | the windows-x64-msvc binary under Prism | 24/24, every tick | 24/24, every tick |

For the portable build, all matching legs are also byte-identical per tick (601 lines per run; checked with
`diff --strip-trailing-cr` on the 24 logs of every leg). In the default build the legs group by C library: all five
glibc legs agree with each other (x64 and ARM64, gcc and clang, `-march=native`), Apple's arm64 and Rosetta legs
agree with each other, UCRT ARM64 agrees with UCRT x64 on 3 rungs (ruins, yard, mech: 6 of 24 runs), and glibc
and Apple agree with each other on 3 rungs. Where the default build first differs from the reference on glibc: at
scene build for most rungs (the settle's stress solve uses `cbrtf` piece sizes, so the solver hash differs before
the world hash), pile at tick 53, ruins at 270, yard at 79, track at 54, and mech at tick 4 in the solver state
only (its world hash stays equal for all 600 ticks).

box64: with `FASTNAN=0 FASTROUND=0`, every tick of every run equals the native x64 run of the same binary. With the
defaults, the final hashes equal too, and so does every tick except one line: the track scene's hash right after
`lpBuildScene` (e0e8e38a45667f68 instead of cea96cf47fae4fe8, in both builds, so not the C library); the first step
erases it. Some value that the first step rewrites takes a different bit pattern under the fast modes (NaN handling
and out-of-range conversions are what they skip; native ARM64 builds do not show it, so it is the emulation, not
the C source); it was not traced further. Use the strict settings.

Prism (windows-11-arm, Windows build 26200, running the windows-x64-msvc executables): every tick of every run
equals native x64, in the default build too. The libm probe explains why that needed luck. Run 36700796929 built
it twice: with the C runtime linked dynamically (CMake's default `/MD`) and statically (`/MT`, as the engine links
it, CMakeLists.txt:19). On native x64 both give the same results. Under Prism the static one still equals native x64
on all five functions, but the dynamic one does not: its `cbrtf` equals UCRT ARM64's on all 491,520 inputs and its
`cosf` matches neither x64 nor ARM64. So the `ucrtbase.dll` that Windows on ARM loads into an x64 process is not the
x64 code (consistent with Windows shipping its system DLLs as ARM64X/ARM64EC, which run natively; the mechanism is
inferred, the behaviour measured). Prism emulates x64 arithmetic bit-exactly, but an x64 build that takes its math
from a system DLL gets different results under Prism: link the CRT statically, or keep C-library math out of the
simulation (the portable build is immune either way).

C-library probe (values of the sweep that differ from Windows x64 UCRT; in brackets, how many of the leg's own
results differ from the double function rounded to float):

| function | UCRT x64 | UCRT ARM64 | glibc x64 | glibc ARM64 | Apple arm64 | Apple x86_64 (Rosetta) |
|---|---|---|---|---|---|---|
| cbrtf (491,520) | 0 (132,700) | 50,980 (125,400) | 141,760 (54,520) | 141,760 (54,520) | 132,700 (0) | 132,700 (0) |
| sqrtf | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) |
| sinf (589,824) | 0 (388) | 5,406 (5,490) | 5,406 (5,490) | 5,406 (5,490) | 5,522 (5,602) | 456 (96) |
| cosf (589,824) | 0 (370) | 3,176 (3,234) | 3,176 (3,234) | 3,176 (3,234) | 3,186 (3,248) | 610 (288) |
| atan2f (262,144) | 0 (0) | 0 (0) | 42,567 (42,567) | 42,535 (42,535) | 15,276 (15,276) | 15,276 (15,276) |

The six C libraries give four different `cbrtf`s and four different `sinf`s (UCRT ARM64's `sinf` and `cosf` equal
glibc's); glibc's `atan2f` differs between its own x64 and ARM64 builds; Apple's `sinf` differs between arm64 and
x86_64. Only `sqrtf` (correctly rounded by IEEE) agrees everywhere. Apple's `cbrtf` is correctly rounded; UCRT's
misses 27% of the time. The engine's `lpf_bench` hashes
therefore cannot match across OSes while any C-library transcendental reaches simulation or scene state, and the
portable build shows that nothing else does.

## Appendix A: E2, D3D11 compute probe (run before the scope was narrowed)

`gpuprobe.cpp` (built with `cl /O2 /EHsc /MT /fp:precise gpuprobe.cpp d3d11.lib dxgi.lib d3dcompiler.lib`):
1,048,576 operand triples from PCG32 (35% random bit patterns, 20% near 1, 10% subnormal, 5% specials, 30%
physics-scale values in [-100, 100], a quarter of operands from any class); 161,618 elements have all three
operands physics-scale. Each op group is compiled separately (a `MODE` define), because fxc otherwise shares a
subexpression between the plain and `precise` forms (the first version was confounded that way). Adapters: RTX 3060,
UHD Graphics, WARP; compile variants default (O1), O3, Od and `D3DCOMPILE_IEEE_STRICTNESS`. Output and DXBC:
`build\m7\e2\`.

Mismatches with the CPU among the physics-scale elements, default compile (IEEE_STRICTNESS in brackets where
different):

| op | NVIDIA | Intel | WARP |
|---|---|---|---|
| add, sub, mul, min, max, ftoi, itof, utof | 0 | 0 | 0 (utof: 19,509 at 1 ulp) |
| div | 46,007 (1-2 ulp) | 45,022 (1-2 ulp) | 0 |
| sqrt | 13,856 at 1 ulp (plus NaN-payload differences for negative inputs) | 6,548 at 1 ulp | 0 |
| rsqrt / rcp | 28,758 / 21,838 (1-2 ulp) | 24,573 / 18,514 | 0 |
| plain `a*b+c` | 42,032, all fused (0) | 42,032 fused (0) | 0 |
| `precise a*b+c` | 0 | 0 | 0 |
| `mad()` | 42,032 fused, also under IEEE_STRICTNESS | the same | 0 |
| a contact-row update (add, mul, max) | 50,285 (0) | 50,285 (0) | 11,471 (0) |
| the same, `precise` | 0 | 0 | 0 |
| ftou of a negative | 0 | 79,999 (0xFFFFFFFF for -1.5) | 0 |

Fixed vectors: ties round to even everywhere (`1 + 3*2^-24 = 1 + 2^-22`, `1 + 2^-24 = 1`); the contraction detector
(`a = b = 1 + 2^-12`, `c = -(1 + 2^-11)`) gives 2^-24, fused, for plain `a*b+c` on NVIDIA and Intel under default,
O3 and Od, and 0 under IEEE_STRICTNESS or `precise`; `mad()` is fused on both GPUs in every variant; subnormals
flush everywhere, WARP included, even through `min(2^-127, 2^-127)`; `min(+0, -0)` is -0 on NVIDIA and Intel but +0
on WARP; `min(NaN, 1) = 1`; `ftoi(3e9)` saturates and `ftoi(NaN) = 0` everywhere; `rsqrt(2)` is 1 ulp low on NVIDIA
only. Without `precise`, fxc also folds constants across a product (`-1.3125 * (0.8125 * x)` became
`x * -1.066406`). NVIDIA canonicalises NaN payloads; Intel's `sub` flips a NaN's sign.

For the GPU question: a D3D11 kernel of add, sub, mul, min, max and conversions, written with `precise`, with no
subnormals on either side, matched the CPU and each other on both vendors here; division, square root, rsqrt and rcp
differ by 1-2 ulp in 4-28% of cases and between vendors, and nothing of this is an API guarantee. It supports T3's
recommendation to keep the GPU off the deterministic path.

## Appendix B: E10, script round trip (run before the scope was narrowed)

`rt.c`: 10^6 samples through `"%.6f"` and `strtof`: camera coordinates in [-60, 60] m change in 9.7% of cases,
coordinates in [-8, 8] m in 68%, unit-direction components and analog controls in 96%, key-driven controls never;
`"%.9g"` round-trips all. A town run with a shot every 12 ticks from full-precision rays vs the same rays after
`%.6f` and the replay's renormalisation differs from tick 6, the first shot.

## Not done

- E6 and E7 (dropped).
- FEX: packaged (Launchpad PPA `fex-emu/fex`, noble supported, 2609.1 of 2026-09-23) but it needs an x86-64 root
  filesystem for a dynamically linked binary, so it is not in the matrix; a static x64 build would avoid that.
- `-flto` legs, WASM, an Emscripten build.
- Tracing the box64-default post-build transient in the track scene.
- The ARM64 legs ran on GitHub's hosted runners (Linux on Azure ARM64 VMs, the windows-11-arm image, macOS on
  Apple silicon); the CPU models were not recorded, and no Qualcomm laptop or DGX Spark was used.

## Recommendations from these results

1. Take the portability commits (gcc contraction flag, clang-cl warnings, Threads, dead code) and the argument-order
   fix onto `sandbox`: all are hash-neutral on MSVC.
2. Remove C-library transcendentals from simulation and scene code for real (the research option shows the way:
   `lpCbrt` and Box3D's trig), accepting a one-time hash change; then the whole bench matches across every tested
   OS, ISA, compiler and emulator.
3. Add two rules to docs/determinism-rules.md: never draw random numbers (or call anything with side effects on
   simulation state) inside another call's argument list or a braced initialiser; link the C runtime statically on
   Windows (or call no C-library math).
4. Keep `.github/workflows/determinism.yml` (it runs in about 30 minutes, box64 being the slowest leg) and hash the
   solver state per tick in any desync check (E9).
