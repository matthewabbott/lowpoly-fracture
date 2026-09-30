# M7 research, track T3: floating-point determinism across machines

Date: 2026-09-29. Scope: whether IEEE float, with pinned flags, is bit-identical across x64 and ARM64 compilers,
x64 emulators on ARM64, WASM, and GPUs, for Box3D's operation set and ours; and the "portable deterministic float
dialect" that follows. Parallel tracks: T4 fixed point, T5 GPU physics, T8 experiments (this report says what T8
should look for).

Repo facts are cited as `file:line` (Box3D at the pinned commit). Web facts carry a URL and, where it matters, a
date. Anything I could not verify is marked **unverified** with what would verify it.

## 0. Findings in one screen

1. Under `/fp:precise` (MSVC), `-ffp-contract=off` (clang, gcc), an SSE2/NEON baseline, and libm limited to
   `sqrtf`/`floorf`/`remainderf`, the arithmetic of MSVC, clang and gcc on x64 and clang/gcc on ARM64 is the same
   IEEE operation sequence, and two engines with the same design verify it in CI across all three compilers and
   several ISAs (Box2D v3.1: MSVC/clang/gcc, x64 and ARM; Jolt: clang 18, gcc 14, MSVC, emscripten; x64, ARM64/32,
   RISC-V, PPC, LoongArch, WASM, one hash for all). H1 holds for the mechanism. The residual hazards are concrete
   and listed in section 1; two of them are ours today (our CMake does not give gcc `-ffp-contract=off`; `cbrtf` in
   seven simulation call sites) and one is Box3D's (NEON `vminq/vmaxq` versus SSE `minps/maxps` disagree on the
   sign of zero, and on NaN).
2. Emulators: Rosetta 2 reproduces x86 float semantics in hardware (an M1-era precursor of FEAT_AFP: NaN sign,
   tininess, flush-to-zero all measured identical to x86) and translates SSE2 fully. box64's source implements
   MINPS/MAXPS with a compare-and-select that matches x86 exactly regardless of its fast modes; its defaults
   (`FASTNAN=1`, `FASTROUND=1`, `SSE_FLUSHTO0=0`) only change NaN sign generation, float-to-int edge cases and
   whether FTZ is honoured, none of which the dialect exercises. FEX and Prism publish nothing about SSE accuracy;
   both are open. H6: confirmed for Rosetta, supported by source for box64, open for FEX and Prism.
3. WASM: the only arithmetic nondeterminism in the spec is NaN bit patterns; relaxed SIMD must stay off.
   Emscripten's SSE2 emulation implements `_mm_min_ps`/`_mm_max_ps` as compare-and-select (x86 semantics) and the
   `cvtt` conversions with the x86 `0x80000000` sentinel, and `_mm_getcsr` reports no FTZ, ever. Box3D's wasm path
   is the SSE2 path on simd128, so bit-identity with native SSE2 is expected. Jolt's CI already includes wasm32/64
   in its single-hash check. Speed cost: **unverified** (no measurement found; T8 can build `lpf_bench` with
   Emscripten if wasm is pursued; it is low priority).
4. GPUs: only CUDA (with `--fmad=false`, `--prec-div`, `--prec-sqrt`, `--ftz=false`, all but the first default)
   offers correctly rounded add/mul/div/sqrt/fma and preserved subnormals on paper. D3D11 mandates flush-to-zero on
   every fp32 arithmetic op, lets hardware truncate add/mul (0.5 ULP "within", not RNE), gives div and sqrt 1 ULP,
   and fp32 `mad` may or may not fuse. Correctly rounded div/sqrt can be built in software (OpenCL exposes exactly
   that switch; fp64 with narrowing is another route), but CPU<->GPU bit identity on D3D11 also requires no
   subnormal ever appearing, which no CPU-side rule can promise without flushing on the CPU, and flushing on the
   CPU is emulator-hostile (box64 ignores FTZ by default; WASM cannot flush). Verdict: keep the GPU off the
   deterministic path on D3D11 (agrees with H4); CUDA or Vulkan-with-float-controls are the only routes worth an
   experiment, and only for a subnormal-free kernel.
5. The dialect (section 6) is a no-FMA dialect. An explicit-FMA dialect is the natural GPU dialect but is not
   implementable on D3D11/D3D12 fp32 (no guaranteed fused op), not on WASM (no scalar fma; relaxed `madd` is
   nondeterministic), needs macOS 15 Rosetta and Windows 24H2 Prism, and excludes the 4.4% of Steam machines
   without FMA plus most "toasters". Its CPU gain would be a few percent (Jolt reports about 8% for its whole
   determinism mode, which also removes `/fp:fast`). Not worth it.

## 1. Hazard table (question 1)

Setting assumed everywhere below: 64-bit targets only; `/fp:precise` (MSVC, VS2022 or later); `-ffp-contract=off`
on clang, clang-cl (`/clang:-ffp-contract=off`) and gcc; no `-ffast-math`, `/fp:fast`, `-Ofast`; SSE2 or NEON
baseline; default FP environment (RNE, no FTZ/DAZ, exceptions masked).

| # | Hazard | Where it bites | Fix | Cost | Evidence |
|---|---|---|---|---|---|
| 1 | FMA contraction, compiler defaults | gcc defaults to `-ffp-contract=fast` in GNU C modes (`-std=gnu17`, which CMake's `C_STANDARD 17` without `C_EXTENSIONS OFF` selects) and to `off` only in strict ISO modes; clang (14+) defaults to `on` (fusing within one statement), on every target including x64 with `-march`/`/arch:AVX2` and always on AArch64 where FMA is baseline; MSVC before VS2022 17.0 contracted under `/fp:precise` when targeting AVX2. **Our top-level CMakeLists.txt:27-37 sets the flag for Clang and MSVC only; gcc gets nothing.** Box3D's own CMake sets it for GNU and Clang (extern/box3d/CMakeLists.txt:82-89) but `add_compile_options` there is scoped to that directory, so `src/`, `scenes/`, `test/` built with gcc contract on ARM64 (and on x64 with `-march=native`). Intrinsics are not protected: `vaddq_f32(a, vmulq_f32(b, c))` (simd.h:461-464) is plain IR mul+add to both gcc and clang, so the NEON "add(mul)" comment holds only under the flag. | Change `MATCHES "Clang"` to `MATCHES "GNU|Clang"` in CMakeLists.txt:28; add `-ffp-contract=off` to docs/determinism-rules.md rule 1 (which names clang and MSVC only); keep the self-test contraction probe (section 6.4). | none | GCC doc: default off only in standards mode, fast otherwise (https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html); clang `-ffp-contract` defaults (https://clang.llvm.org/docs/UsersManual.html); MSVC VS2022 change (https://learn.microsoft.com/en-us/cpp/build/reference/fp-specify-floating-point-behavior?view=msvc-170, https://devblogs.microsoft.com/cppblog/the-fpcontract-flag-and-changes-to-fp-modes-in-vs2022/); Jolt adds `-ffp-contract=off` for clang 14+ and drops `-mfma` under CROSS_PLATFORM_DETERMINISTIC (https://raw.githubusercontent.com/jrouwe/JoltPhysics/master/Jolt/Jolt.cmake) |
| 2 | Auto-vectorisation and reassociation | A vectorised elementwise loop performs the same IEEE ops, so it is harmless. Reassociated reductions (sum order changed, `a*b+a*c -> a*(b+c)`) are only legal under fast-math: MSVC `/fp:precise` makes no algebraic transformation unless bit-identical; gcc/clang need `-fassociative-math`/`-ffast-math`. | Never enable fast-math on any target that touches state; keep `LPF_WARNINGS_AS_ERRORS`-style discipline: a CMake check that rejects `/fp:fast` and `-ffast-math` in `CMAKE_C_FLAGS`. | none | MSVC `/fp` doc (as above); GCC `-ffast-math` composition (https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html) |
| 3 | Constant folding | Folded `sqrtf(c)` and IEEE `+ - * /` on constants are correctly rounded by every compiler's folder, so they match runtime. Folded calls to non-correctly-rounded libm (`cbrtf(c)` folded via MPFR in gcc versus glibc at runtime) do not. Decimal literals: modern MSVC/clang/gcc parse correctly rounded. | Ban non-exact libm in simulation code (hazard 12); the self-test compares literal bit patterns and a folded expression against runtime. | none | Bruce Dawson on mixing compile-time and runtime evaluation (https://randomascii.wordpress.com/2013/07/16/floating-point-determinism/); literal parsing: **unverified** per compiler version, caught by the self-test |
| 4 | Float to int conversion | x86 `cvttss2si` returns the "integer indefinite" `0x80000000` on overflow or NaN; AArch64 `fcvtzs` saturates; WASM `i32.trunc_f32_s` traps (saturating only with nontrapping-fptoint); C calls it UB. Factorio hit exactly this (an out-of-range double-to-int) when it made Switch (ARM) play with PC. Our sites: `debris.c:40` grid cell from an unbounded position; `debris.c:477`; `fracture.c:631-657` (bounded by piece size); `supply.c:191`, `impact.c:402` (bounded). Box3D converts only in height-field and mesh code we do not use (`height_field.c:670-1230`, `mesh.c:396-398`). | Clamp before every conversion of an unbounded value (`debris.c:40`: clamp the coordinate to +-2^20 cells); a grep-able helper `lpFloatToInt(x, lo, hi)`. | none | Intel semantics (https://www.felixcloutier.com/x86/cvttss2si); FFF-370 (https://www.factorio.com/blog/post/fff-370); WASM trapping conversions (spec; emscripten emulates `_mm_cvttss_si32` with the x86 sentinel: https://raw.githubusercontent.com/emscripten-core/emscripten/main/system/include/compat/xmmintrin.h) |
| 5 | min/max on +-0 and NaN | x86 `minps(a,b)` returns b when either is NaN or both are zero of any sign (so `min(-0,+0) = +0`, `min(+0,-0) = -0`). AArch64 `FMIN` returns NaN if either is NaN and orders `-0 < +0`. Box3D's paths: SSE `_mm_min_ps` (simd.h:138-146, 662-670); NEON `vminq_f32/vmaxq_f32` (simd.h:466-474); scalar 3-lane `<`/`>` (simd.h:305-321, matches SSE); scalar wide `<=`/`>=` (simd.h:855-873, does **not** match SSE on +-0, only with `BOX3D_DISABLE_SIMD`). The wide contact solver does `b3MaxW(zero, s*inv_h)` and `b3MaxW(x, zero)` (contact_solver.c:1867, 1930, 1941, 2118): with `s*inv_h = -0`, SSE yields `-0`, NEON `+0`. This can only ever change the sign of a zero, but `lpWorld_Hash` hashes raw velocity bytes, so `-0` versus `+0` is a hash mismatch with no physical difference; and a `-0` that reaches a division or `atan2` could diverge for real (Box3D guards its divisions). Scalar C `a < b ? a : b` is safe on every compiler (they may only lower it to `minss`/`fcsel`, whose semantics match). | Define the dialect's min/max as `a < b ? a : b` (SSE-native) and patch Box3D's NEON wrappers to `vbslq_f32(vcltq_f32(a,b), a, b)` (2 extra instructions per min/max in the wide solver; well under 1% of the step); fix the scalar wide path to `<`/`>`. Keep NaN a hard error (validation). | <1% on ARM64 | Intel MINPS rule (https://www.felixcloutier.com/x86/minps); Arm FMIN semantics: **unverified by fetch** (Arm's site is script-rendered); Erin Catto's own caveat in the brief; T8's NEON probe verifies |
| 6 | NaN payloads and sign | x86 makes a negative default NaN, ARM a positive one; propagation priority differs; box64 with `FASTNAN=1` (default) does not emulate the x86 sign; WASM leaves NaN bits nondeterministic. Any NaN in state therefore hashes differently per platform. | NaN is a bug: Box3D validation asserts on it; add a NaN check to `lpWorld_Hash` in debug and to the self-test; never canonicalise NaN silently. | none | FEAT_AFP comparison of x86 and Arm NaN rules (https://zenn.dev/mod_poppo/articles/arm-feat_afp); box64 USAGE (https://github.com/ptitSeb/box64/blob/main/docs/USAGE.md); WASM design notes (https://raw.githubusercontent.com/WebAssembly/design/main/Nondeterminism.md) |
| 7 | Denormals (FTZ/DAZ, FPCR.FZ/AH/FIZ) | Processes start with subnormals preserved on every OS (MSVC documents the default environment). Anything that sets MXCSR bits 15/6 or FPCR.FZ on a thread that then steps the world changes results. Known offenders are third-party libraries and their init code (Dawson); Direct3D 9's default was to set the x87 unit to single precision, which is an x87 issue only (`D3DCREATE_FPU_PRESERVE`), and D3D11 changes nothing on the CPU. Unity Burst's new deterministic mode chose the opposite convention (flush everywhere); we cannot: box64 ignores FTZ by default (`BOX64_SSE_FLUSHTO0=0`) and WASM cannot flush at all. Subnormal arithmetic is also slow on x86 (microcode assists), a performance note only. | The MXCSR/FPCR guard (section 6.3): read, compare to the default, repair and count, on the stepping thread each step and in every worker (ours and Box3D's) at start and per task. | ~2 ns per check | MSVC default environment (`/fp` doc above); D3D9 flag (https://learn.microsoft.com/en-us/windows/win32/direct3d9/d3dcreate); Dawson (above); Burst flushes on all platforms (https://docs.unity3d.com/Packages/com.unity.burst@1.8/manual/compilation-burstcompile.html); box64 `BOX64_SSE_FLUSHTO0` default 0 (USAGE.md above); emscripten `_mm_getcsr` fixed to FTZ off (https://emscripten.org/docs/porting/simd.html) |
| 8 | x87 and 32-bit x86 | x87 keeps 80-bit intermediates with a per-thread precision control; gcc `-m32` defaults to x87 unless `-mfpmath=sse`; emulators approximate x87 (Prism at 64 bits by default, Rosetta in slow software, box64 "float when possible"). glibc's x86_64 `remainderf` uses x87 `fprem1`; that is exact (IEEE remainder is exact) so it stays deterministic even through an emulator's x87 path, but it is an x87 instruction in our Linux binary. | 64-bit only (Burst's deterministic mode makes the same restriction). Windows/MSVC builds have no x87 at all. | none | Dawson (above); Prism x87 setting (https://learn.microsoft.com/en-us/windows/arm/apps-on-arm-program-compat-troubleshooter); dougallj on Rosetta's x87 (https://dougallj.wordpress.com/2022/11/09/why-is-rosetta-2-fast/); box64 `X87DOUBLE` (USAGE.md) |
| 9 | Excess precision (`FLT_EVAL_METHOD`) | 0 on x64/ARM64 SSE/NEON for all three compilers. MSVC says intermediate computations "may be performed at machine precision" under `/fp:precise`, which on x64 is the operand precision for SSE scalar code; the documented x64 codegen keeps `addss`/`mulss`. | 64-bit only; T8's MSVC-versus-clang-cl hash run is the check. | none | MSVC `/fp` doc (above); gcc `-fexcess-precision` doc not fetched, **unverified**, irrelevant on 64-bit |
| 10 | Approximate instructions (`rcpps`, `rsqrtps`, `vrecpe`, `vrsqrte`) | Vendor-specific tables (Intel and AMD differ in the low bits). Compilers emit them only under fast-math/`-mrecip`. Box3D avoids them on purpose (contact_solver.c:1974, 2036). Emscripten emulates them with full-precision div/sqrt, which differs from x86 hardware, another reason to ban them. | Ban; grep for them in CI. | none | Dawson (above); emscripten table (above); contact_solver.c comments |
| 11 | libm | `sqrtf`: correctly rounded everywhere (IEEE requires it; SSE `sqrtss`; Box2D measured it deterministic). `floorf`, `ceilf`, `truncf`, `fabsf`, `copysignf`, `nextafterf`: exact. `remainderf`: exact by definition (glibc, UCRT, musl all give the same bits). `sinf`/`cosf`/`atan2f`: differ per libm (Box2D found it; Factorio and Jolt wrote their own). **`cbrtf`: not correctly rounded in any libm**; glibc states it does not aim for correct rounding except `sqrt`, `fma`, `rint`; Windows UCRT additionally dispatches some x64 math functions to FMA3 variants on capable CPUs (`_set_FMA3_enable`, page not fetched, **unverified** whether `cbrtf` is among them), so even the same binary can differ by CPU. Our sites: stress.c:134, 254, 536, 554, 1231; world.c:576, 633; impact.c:1018 (simulation); impact.c:320, split.c:149, step.c:146 (particle sizes, cosmetic); fracture.c:1199 already avoids it. Scene building: scenes.c:354 `atan2f`, 621 `sinf`, 1170 and 1620 `cbrtf`. | Replace `cbrtf` with `lpCbrt` (section 6.5); scenes use the dialect too (section 6.6); Box3D's `b3ComputeCosSin`/`b3Atan2` stay (math_functions.c:170-264, pure IEEE ops plus `sqrtf`/`remainderf`). | negligible | Box2D determinism post (https://box2d.org/posts/2024/08/determinism/); glibc manual, accuracy section (https://sourceware.org/glibc/manual/latest/html_node/Errors-in-Math-Functions.html); FFF-52 (https://factorio.com/blog/post/fff-52); Jolt Architecture.md (https://raw.githubusercontent.com/jrouwe/JoltPhysics/master/Docs/Architecture.md) |
| 12 | `qsort` instability | glibc, musl and UCRT sort differently; ties come out in different orders. Factorio's first cross-compiler desync was an ambiguous STL comparator. Box3D uses its own quicksort (extern/box3d/src/qsort.h:23-27, Sedgewick, index-based), so its tie order is fixed by source, not by libc. Ours: rule 5, every comparator breaks ties by index. | Keep rule 5; a debug check that every comparator is a total order (compare(a,b) and compare(b,a) never both 0 for a != b). | none | FFF-52 (above); Jolt replaces `std::sort` with its own `QuickSort` (Architecture.md above) |
| 13 | `-march` / `/arch` | VEX-encoded scalar and 128-bit ops (`vaddss`, `vaddps`) give the same bits as SSE; the only ISA-dependent value change is contraction (hazard 1), and MSVC before VS2022 did contract with `/arch:AVX2`. AVX-512 is out of scope. Rosetta before macOS 15 and Prism before 24H2 did not emulate AVX2 at all. | SSE2 baseline for shipped builds (also the emulator- and toaster-friendly choice); allow `/arch:AVX2` only on VS2022+ and only after T8 shows identical hashes. | none | MSVC `/fp` and VS2022 blog (above); Rosetta AVX2 (https://developer.apple.com/documentation/apple-silicon/about-the-rosetta-translation-environment); Prism 24H2 (troubleshooter page above) |
| 14 | LTO / PGO | Change inlining and code layout, which cannot change IEEE results when contraction and reassociation are off. Risk only where a TU built with fast-math is merged (gcc records per-function options, but mixed flags are a known LTO trap). | One flag set for the whole tree; T8 can add an LTO build to the matrix as a control. | none | Reasoning from the flags; **unverified** empirically |
| 15 | Compiler version drift | Erin Catto calls cross-platform determinism "brittle" under compiler updates; Jolt's strict guarantee is "same binary", yet its CI holds one hash across clang 18, gcc 14, MSVC and emscripten. Drift enters through defaults (clang moved to `-ffp-contract=on` in 14; MSVC turned contraction off in 17.0), libm (hazard 11) and bugs. | Pin toolchains for shipped multiplayer builds; run the T8 matrix in CI with a golden hash; the handshake exchanges build id plus self-test hash (section 6.4). | CI time | Box2D post; Jolt determinism_check.yml (https://github.com/jrouwe/JoltPhysics/blob/master/.github/workflows/determinism_check.yml) |
| 16 | Double precision | Our stress solver accumulates in double (stress.c). Double ops are IEEE like float; the same flags cover them. GPU double is 1/32 to 1/64 rate on consumer parts (T5). | Nothing. | none | IEEE 754 |
| 17 | Text round trips | Scripts record controls with `%.6f` (brief). Not simulation, but a replay recorded on one machine and played on another must reproduce the same floats. | Record hex floats (`%a`) or integers. | none | Dawson (base conversion differs) |

## 2. x64 emulators on ARM64 (question 2)

| Emulator | SSE2 scalar and packed bit-exact by default? | Denormals | NaN | min/max | Float to int | x87 | Switches | Evidence and confidence |
|---|---|---|---|---|---|---|---|---|
| Rosetta 2 (macOS) | Yes: Apple silicon has an x86 floating-point mode (an M1-era precursor of Armv8.7 FEAT_AFP) that Rosetta enables; measured on M4: NaN sign, NaN propagation, tininess detection and flush-to-zero all identical to x86. SSE2 is translated fully to NEON sequences; AVX/AVX2 are supported now (Apple's page lists them; AVX-512 is not; **which macOS added AVX2 unverified**, believed macOS 15). | x86 rules, including FTZ if the app sets it | x86 rules | reported covered by the same hardware mode (**unverified for MINPS specifically**) | not tested in the sources | slow software 80-bit | none | https://zenn.dev/mod_poppo/articles/arm-feat_afp (2024-25, M4 tests); https://dougallj.wordpress.com/2022/11/09/why-is-rosetta-2-fast/; Apple doc above. Confidence high. One bug report exists for Rosetta-for-Linux inside Docker (bun #19677: floats truncated to integers; root cause not stated in the issue) so "Rosetta" is two products; test both. |
| Windows 11 Prism | Unknown. Microsoft documents no SSE accuracy statement. Snapdragon X cores are Armv8.7+ so FEAT_AFP is available to it (**unverified that Prism uses it**). One unresolved MS Q&A report of wrong float output in a 32-bit x87-era toolchain app under Prism (`1.23` printed as `0.23`). | not documented | not documented | not documented | not documented | approximated at 64 bits by default; "Disable floating point optimization" forces 80-bit | per-exe emulation settings (Default/Safe/Strict/Very strict; the x87 switch; hide AVX2/FMA) | https://learn.microsoft.com/en-us/windows/arm/apps-on-arm-program-compat-troubleshooter (2025-12); https://learn.microsoft.com/en-us/answers/questions/2264223/floating-point-errors-with-prism-emulator. Confidence low; no device in the T8 lab. |
| FEX-Emu (Linux) | Unknown from documentation. FEX's release notes discuss x87 optimisation (substituting 64-bit "and hope the game won't notice") and AVX correctness (2607: zero-extension audits), nothing on SSE arithmetic semantics. FEX has an opt-in x87 reduced-precision mode (**unverified: config file not reachable**; issue #4191 questions precision-control fidelity). Whether FEX uses FEAT_AFP when present (Grace/Neoverse V2) is **unverified**. | ? | ? | ? | ? | 80-bit default, reduced-precision opt-in | `X87ReducedPrecision` (name from memory, **unverified**) | https://github.com/FEX-Emu/FEX/issues/4191; https://fex-emu.com/FEX-2408/; https://fex-emu.com/FEX-2607/. Confidence low; T8 on the DGX Spark can settle it. |
| box64 (Linux) | Yes for the dialect, by source: ADDPS/SUBPS/MULPS/DIVPS/SQRTPS map to NEON ops (exact); MINPS/MAXPS are `FCMGT` + `BIF`, which reproduces x86's "return the source when unordered or equal" rule regardless of `FASTNAN`; with `FASTNAN=0` it additionally rewrites NaN outputs to x86's negative NaN. | `BOX64_SSE_FLUSHTO0=0` (default) only tracks the FTZ flag, does not flush; `1` flushes | `FASTNAN=1` (default): NaN sign not emulated | exact | `FASTROUND=1` (default): edge cases (overflow, NaN, non-default rounding mode) not emulated; `0` emulates x86 | `X87DOUBLE=0`: float when possible; `X87_NO80BITS` | force `BOX64_DYNAREC_FASTNAN=0 BOX64_DYNAREC_FASTROUND=0` for the experiment and compare with defaults; expect identical for the dialect | https://github.com/ptitSeb/box64/blob/main/docs/USAGE.md; https://raw.githubusercontent.com/ptitSeb/box64/main/src/dynarec/arm64/dynarec_arm64_0f.c (MINPS/MAXPS lowering). Confidence medium-high pending T8. |

Game reports: Factorio is the only lockstep game with a documented ARM-versus-x86 cross-play effort (Switch, FFF-370,
2022): they diffed per-tick state CRCs across 2,417 tests and found undefined behaviour (out-of-range double-to-int
conversions), not float arithmetic. Nothing was found on Rosetta/Proton-on-ARM desyncs for Factorio or RTS games
(search budget exhausted; **unverified either way**).

## 3. WASM as a deterministic VM (question 3)

- Spec: NaN bit patterns are the only arithmetic nondeterminism (plus relaxed SIMD, threads, growth failures and
  host calls). Add/sub/mul/div/sqrt are IEEE with subnormals preserved; there is no way to flush or change rounding
  (emscripten's `_mm_getcsr` always reports FTZ off, RNE, exceptions masked; `_mm_setcsr` is unavailable).
  https://raw.githubusercontent.com/WebAssembly/design/main/Nondeterminism.md;
  https://emscripten.org/docs/porting/simd.html.
- Relaxed SIMD: `relaxed_madd` may or may not fuse, `relaxed_min/max` may return either operand on NaN or +-0,
  `relaxed_trunc` may saturate or return INT_MIN; each environment picks one behaviour and keeps it, but two
  environments may differ. Keep `-mrelaxed-simd` off; Wasmtime's `relaxed_simd_deterministic` forces the slow exact
  choice. https://raw.githubusercontent.com/WebAssembly/relaxed-simd/main/proposals/relaxed-simd/Overview.md;
  https://docs.wasmtime.dev/examples-deterministic-wasm-execution.html.
- NaN canonicalisation (Wasmtime `cranelift_nan_canonicalization`, Wasmer similar) costs on every float op and is
  only needed if NaNs may exist; the dialect forbids them, so leave it off.
- Box3D's wasm path: `core.h:68-71` defines `B3_SIMD_SSE2` on wasm and the build adds `-msimd128 -msse2`
  (extern/box3d/CMakeLists.txt:106-108); the SSE2 intrinsics are emulated by Emscripten's headers. For Box3D's op
  set: `_mm_add/sub/mul/div/sqrt_ps` native and exact; `_mm_min_ps`/`_mm_max_ps` implemented as
  `bitselect(a, b, a < b)` (x86 semantics, the `pmin` migration is a TODO); `_mm_cvttss_si32` emulated with the
  x86 sentinel; `_mm_rcp/rsqrt_ps` emulated with full-precision div/sqrt (not used). So bit identity with native
  SSE2 is expected; Jolt's single cross-platform hash includes wasm32 and wasm64 under Node, which is the same
  kind of code. Also pass `-ffp-contract=off` (wasm has no scalar fma, but keep the rule uniform).
- Scalar `(int)` casts trap on overflow in wasm unless `-mnontrapping-fptoint` (then they saturate): clamp first
  (hazard 4).
- Speed versus native for a physics engine: **unverified**; I found no Box2D/Box3D/Jolt/Rapier wasm-versus-native
  numbers within the budget. If wasm matters, T8 builds `lpf_bench` with Emscripten (`-msimd128 -msse2 -pthread`)
  and measures; expect a constant-factor slowdown, not a determinism problem.

## 4. GPU flag table (question 4)

One page, per API. "CR" = correctly rounded (IEEE RNE). Entries marked (recalled) are from memory of the spec and
were not fetched this session; T5/T8 should confirm them against the spec text.

| API | add/sub/mul | div | sqrt | fp32 fma | Forbid contraction | Denormals | Rounding mode | Evidence |
|---|---|---|---|---|---|---|---|---|
| CUDA (cc >= 2.0) | CR | CR (`--prec-div=true`, default) | CR (`--prec-sqrt=true`, default) | CR; `fmaf`/`__fmaf_rn` always fused | `--fmad=false` (**default is `true`: contraction on**); `__fadd_rn`/`__fmul_rn` intrinsics are never contracted (recalled) | preserved by default (`--ftz=false`); `--use_fast_math` sets ftz, approx div/sqrt, fmad | RNE; per-op `_rz/_ru/_rd` intrinsics | https://docs.nvidia.com/cuda/floating-point/index.html; https://docs.nvidia.com/cuda/cuda-compiler-driver-nvcc/index.html |
| D3D11 / SM5 (fxc) | within 0.5 ULP: hardware may truncate rather than round to nearest even | mul 0.5 ULP times rcp 1 ULP; a direct div must be no worse than that two-step (so about 1.5 ULP, not CR) | 1 ULP; rcp/rsq relaxed further | `mad` may be fused or unfused; must be no worse than the worst serial order; HLSL `fma()` is double only (recalled) | `precise` on the value, or `/Gis` (`D3DCOMPILE_IEEE_STRICTNESS`) marks everything precise: no reordering, no `x*0 -> 0`, no mad fusion; does not change per-op precision | **flushed to sign-preserved zero on input and output of every fp32 arithmetic op** (moves/loads excepted); fp64 preserves | none (RNE-ish, truncation allowed) | https://learn.microsoft.com/en-us/windows/win32/direct3d11/floating-point-rules; https://learn.microsoft.com/en-us/archive/blogs/marcelolr/precise-and-ieee-strictness-in-hlsl |
| D3D12 / SM6.x (dxc, DXIL) | as D3D11 base rules | as D3D11 | as D3D11 | DXIL `FMad` for float (fusing unspecified, recalled), `Fma` double only (recalled) | `precise` = no fast-math flags on `fadd/fsub/fmul/fdiv/frem/fcmp`; by default all HLSL float ops are "fast" | per-function `fp32-denorm-mode` = `any` (default) / `preserve` / `ftz` (`-denorm` option, SM6.2 feature; driver support is a capability) | none | https://raw.githubusercontent.com/microsoft/DirectXShaderCompiler/main/docs/DXIL.rst |
| Vulkan / SPIR-V | CR (recalled) | 2.5 ULP; `1.0/x` 2.5 ULP (recalled) | inherited from `1/inversesqrt`, inversesqrt 2 ULP (recalled) | `OpFma` precision "inherited from mul then add", i.e. fusing not guaranteed (recalled) | `NoContraction` decoration forbids fusing; `shader_float_controls2` / `FPFastMathMode` per instruction | may be flushed by default; `DenormPreserve`/`DenormFlushToZero` execution modes exist but per-width support is a queried property (`VkPhysicalDeviceFloatControlsProperties`), optional | `RoundingModeRTE`/`RTZ` execution modes, optional per device | https://docs.vulkan.org/spec/latest/appendices/spirvenv.html (float-control capabilities confirmed as optional; ULP table not reachable through the fetch tool) |
| Metal (MSL) | fast math **on by default** (`fastMathEnabled` default true; deprecated in macOS 15 for `MTLMathMode`) | as OpenCL-style fast/precise variants (recalled) | recalled | `fma()` single rounding (recalled) | `MTLMathMode.safe`: no transformation that could change results; `relaxed` keeps Inf/NaN; `fast` | **unverified** (spec PDF too large to fetch) | none | https://developer.apple.com/documentation/metal/mtlcompileoptions/fastmathenabled; https://developer.apple.com/documentation/metal/mtlmathmode |
| WGSL (WebGPU) | CR (recalled) | 2.5 ULP (recalled) | inherited from `1/inverseSqrt` (recalled) | `fma` accuracy inherited from `x*y+z`, so fusing not guaranteed (recalled) | none | may be flushed (recalled) | none | https://www.w3.org/TR/WGSL/ (accuracy section not reachable through the fetch tool) **all unverified** |
| OpenCL (reference point) | CR | 2.5 ULP by default; `-cl-fp32-correctly-rounded-divide-sqrt` makes `x/y`, `1/x`, `sqrt` CR (device cap `CL_FP_CORRECTLY_ROUNDED_DIVIDE_SQRT`) | same | `fma` CR by definition; `CL_FP_FMA` says whether it is hardware | `-cl-mad-enable` is opt-in (off by default) | `CL_FP_DENORM` cap; `-cl-denorms-are-zero` opt-in | RNE required (`CL_FP_ROUND_TO_NEAREST` mandatory) | https://registry.khronos.org/OpenCL/sdk/1.2/docs/man/xhtml/clBuildProgram.html; https://registry.khronos.org/OpenCL/sdk/3.0/docs/man/html/clGetDeviceInfo.html |

Software correctly rounded div and sqrt on a GPU: yes, three ways, with different preconditions.

1. With a guaranteed fused fma (CUDA, Metal, OpenCL `fma`, SPIR-V only if the device fuses): reciprocal seed,
   Newton-Raphson refinement, then the exact residual `r = fma(-q, b, a)` and one correcting `fma(r, y, q)`
   (Markstein's method; this is how CUDA's own `div.rn.f32` and OpenCL's correctly rounded mode are implemented in
   software, the existence of the OpenCL switch being the proof it is done in practice). About 8-12 ops.
   Subnormal and overflow ranges need scaling.
2. Without fma but with RNE add/mul and no contraction (D3D11 with `precise`, if the probe shows RNE not
   truncation): Dekker/Veltkamp two-product gives the exact residual from add/mul alone (about 17 flops per
   product), then the same correction. About 25-35 ops. Breaks where D3D11 flushes: any operand or residual below
   `2^-126` is zeroed, so it is only exact for values kept away from the subnormal range.
3. Via fp64: `ddiv`/`dsqrt` are IEEE on D3D11 and preserve denormals; computing the float op in double and
   narrowing is correctly rounded because double rounding is innocuous when the wide format has at least 2p+2
   bits (53 >= 50) (Figueroa, "When is double rounding innocuous?", SIGNUM 1995; not fetched). The narrowing of a
   subnormal float result may still be flushed under D3D11's rules (the probe should check `dtof` of `2^-127`).
   fp64 rate is 1/32 to 1/64 on GeForce, fine for a few divisions per constraint, not for everything.

None of the three removes the flush-to-zero problem on D3D11, and add/mul are permitted to truncate there, so
CPU<->GPU bit identity on D3D11 is not a flag away: it is a subnormal-free algorithm with software div/sqrt and a
per-vendor probe. On CUDA with `--fmad=false` the CPU no-FMA dialect maps op for op (the CUDA document itself
warns only that the *sequence* of operations may differ). On Vulkan it maps where the device reports
`DenormPreserve` + `RoundingModeRTE` for 32-bit and the shader uses `NoContraction` and software div/sqrt.

## 5. Evidence from engines (question 5)

| Engine | What they do | Where they got burned | Source |
|---|---|---|---|
| Box2D v3 (v3.1 cross-platform) | No fast-math, `-ffp-contract=off`, own `atan2`/`cos`/`sin` (`sqrtf` found deterministic via SSE), a "Falling Hinges" test whose step count and transform hash (e.g. `0x5e70e5fe`) are checked in GitHub Actions across MSVC, clang, gcc, x64 and ARM. | libm trig differed per platform; author calls the property brittle under compiler updates. | https://box2d.org/posts/2024/08/determinism/ (2024-08) |
| Box3D (vendored) | Same design: `-ffp-contract=off` for GNU and Clang (extern/box3d/CMakeLists.txt:82-89), `b3MulAddW` is add(mul) on every path with the comment that real FMA would not match the scalar path (simd.h:460-464), no `rsqrt` (contact_solver.c:1974), own trig (math_functions.c:170-264), own quicksort. | Residual: NEON versus SSE min/max tie semantics (hazard 5); `b3ComputeCosSin` comment says libm cos/sin matched in the author's tests but he does not trust it (math_functions.c:212-214). | repo |
| Jolt | `CROSS_PLATFORM_DETERMINISTIC` drops `-mfma`/`JPH_USE_FMADD`; the application must build with `/fp:precise` or `-ffp-model=precise` and `-ffp-contract=off`, use Jolt's `Sin`/`Cos`, its `QuickSort` and `Hash`; about 8% slower. CI `determinism_check.yml` runs one `-validate_hash` across Linux clang 18/gcc 14, AVX-512 under Intel SDE, ARM64/32 and RISC-V/PPC/LoongArch under QEMU, Windows MSVC 64/32-bit, macOS clang, wasm32/64 emscripten, plus double-precision builds. | Strict guarantee is stated for the same binary; broadphase queries and callback order are documented as non-deterministic (threading, not float). | https://raw.githubusercontent.com/jrouwe/JoltPhysics/master/Docs/Architecture.md; https://github.com/jrouwe/JoltPhysics/blob/master/.github/workflows/determinism_check.yml; https://raw.githubusercontent.com/jrouwe/JoltPhysics/master/Jolt/Jolt.cmake |
| Rapier | `enhanced-determinism`: determinism across platforms that strictly implement IEEE 754-2008 (mainstream CPUs and WASM); math must go through nalgebra's `ComplexField`/`RealField` (a portable libm); the current page says it cannot be combined with `simd8` (lane-width change) and that `parallel` is allowed; the older text (quoted by the brief's question) forbade `simd-stable`/`simd-nightly`/`parallel`. | The restriction is about keeping one operation order across lane widths and thread counts, not about SIMD arithmetic being inexact. | https://rapier.rs/docs/user_guides/rust/determinism/ |
| PhysX 5 | Determinism only for one platform and one build with identical API call order and time-stepping; cross-platform explicitly not promised (hardware precision and instruction reordering). `eENABLE_ENHANCED_DETERMINISM` only makes results invariant to adding non-interacting actors, at a performance cost. | Gave up on cross-platform. | https://nvidiagameworks.github.io/PhysX/4.1/documentation/physxguide/Manual/BestPractices.html |
| Unity Burst | `FloatMode.Deterministic` was "reserved for future" for years; shipped in Burst 1.8.25 (2025-09-16): 64-bit only, disables FMA-type optimisations, deterministic math implementations, **subnormals flushed to zero on all platforms**, NaN bit patterns not guaranteed, intrinsics need bit-equivalent counterparts. | Took years; chose FTZ everywhere (a convention we cannot adopt, hazard 7). | https://docs.unity3d.com/Packages/com.unity.burst@1.8/changelog/CHANGELOG.html; https://docs.unity3d.com/Packages/com.unity.burst@1.8/manual/compilation-burstcompile.html; https://discussions.unity.com/t/what-is-the-state-of-bursts-floatmode-deterministic/1668491 |
| Factorio | Float lockstep across Windows/macOS/Linux with own trig (FFF-52, 2014); a 500-player test across OSes and continents without desync (FFF-108); Switch (ARM64) cross-play with PC (FFF-370, 2022) verified by per-tick CRCs over 2,417 tests. | Ambiguous STL sort comparators (map generation); undefined behaviour surfacing on ARM (double-to-int out of range); a 2022 forum report of non-deterministic software FMA turned out to be a disassembler artefact (no FMA in the binary). | https://factorio.com/blog/post/fff-52; https://www.factorio.com/blog/post/fff-370; https://forums.factorio.com/viewtopic.php?p=703214 |
| Photon Quantum | Fixed point everywhere (`FP`, 48.16, LUT trig and sqrt); floats forbidden inside the simulation because a float-to-FP conversion "will cause desyncs 100% of the time". | Chose to avoid the whole problem class; pays in range and precision (T4's territory). | https://doc.photonengine.com/quantum/current/manual/quantum-ecs/fixed-point |
| Supreme Commander, Stormgate | **Not researched**: search budget exhausted before these; nothing claimed. | | |

Pattern: every float-lockstep success (Box2D, Jolt, Factorio) rests on the same four rules (no fast-math, no
contraction, own trig, own sort) plus a golden-hash CI; every failure story is libm, a comparator, or UB, never
IEEE add/mul/div/sqrt themselves.

## 6. The dialect for our engine (question 6)

### 6.1 Allowed and banned

Allowed in simulation code (Box3D and `src/`, and `scenes/` per 6.6):

- `+ - * /`, `sqrtf`, comparisons, and the same as 4-wide SSE2/NEON/simd128 vectors; `double` under the same rules.
- `fabsf`, `copysignf`, negation (sign-bit ops), `floorf`, `ceilf`, `truncf`, `remainderf`, `nextafterf` (all exact).
- min/max defined as `a < b ? a : b` and `a > b ? a : b` (SSE-native semantics: the second operand on ties and NaN);
  NEON via `vbslq_f32(vcltq_f32(a,b), a, b)`; never `fminf`/`fmaxf`/`vminq`/`vmaxq`/`_mm_min_ss` on `+-0`-bearing data.
- Float-to-int only after a clamp that proves the value in range; int-to-float freely.
- Box3D's `b3ComputeCosSin`, `b3Atan2`; our `lpCbrt` (6.5); PCG32; integer arithmetic.

Banned: `fma`/`fmaf` and any contraction; `rcpps`/`rsqrtps`/`vrecpe`/`vrsqrte` and `_mm_*_ss` estimate forms;
every other libm function (`sinf`, `cosf`, `tanf`, `atan2f`, `expf`, `logf`, `powf`, `cbrtf`, `fmodf`, `hypotf`);
`long double`; x87 and 32-bit x86; FTZ/DAZ/rounding-mode changes; AVX-512; wasm relaxed SIMD; NaN in state;
`qsort` without a total-order comparator; hashing structs with padding (rule 11).

### 6.2 Build flags

| Compiler | Flags |
|---|---|
| MSVC x64 and ARM64 | `/fp:precise` (never `/fp:contract`, `/fp:fast`); default `/arch` (SSE2); VS2022 17.0 or later |
| clang, clang-cl | `-ffp-contract=off` (`/clang:-ffp-contract=off`), default `-ffp-model=precise`, no `-ffast-math`, no `-mrelaxed-simd`; `-march` may be raised only with T8 evidence |
| gcc (x64, ARM64) | `-ffp-contract=off -fno-fast-math` in every directory, including ours (fix CMakeLists.txt:28 to `MATCHES "GNU|Clang"`) |
| Emscripten | `-msimd128 -msse2 -ffp-contract=off -mnontrapping-fptoint` (saturating conversions, still clamp) |
| all | 64-bit only; one flag set for every TU; CI fails if `CMAKE_C_FLAGS` contains `fast` |

### 6.3 The MXCSR/FPCR guard

`lpFpGuard(void)`: on x86 read `_mm_getcsr()`, mask out the six status flags (bits 0-5) and compare the rest to
`0x1F80` (RNE, no FTZ bit 15, no DAZ bit 6, all exceptions masked); on AArch64 read FPCR (`_ReadStatusReg` on
MSVC, `__builtin_aarch64_get_fpcr`/inline asm `mrs` on gcc and clang) and require RMode [23:22] = 0, FZ (24) = 0,
AH (1) = 0, FIZ (0) = 0. Policy: repair to the default, increment `lpStats.fpRepairs`, assert in debug builds.
Cost: a `stmxcsr`/`mrs`, a mask and a compare, about 2 ns; the repair path never runs in a healthy process.

Where: the thread calling `lpWorld_Step` at entry (step.c); our workers in `lpWorkerMain` before each task
(src/tasks.c:114-116 create them); Box3D's scheduler workers at thread start and per task pickup
(extern/box3d/src/scheduler.c:126 creates them via `b3CreateThread`, timer.c:216/374): a two-line patch recorded
in `extern/box3d/PATCHES.md`, or a `b3World` task callback if Box3D exposes one later. New threads start from the
process default on Windows and inherit the creator's environment on POSIX, so the per-worker check is what
catches a library that changed MXCSR on the main thread before the pool was created.

### 6.4 Self-test vector

A table of about 60 operations with expected bit patterns, run at startup and in CI on every platform, hashed, and
exchanged in the session handshake with the build id (all peers must match before lockstep starts):

- Literals: `0.1f == 0x3DCCCCCD`, `B3_PI`, a few subnormal literals.
- Rounding: `1 + 2^-24 == 1` (tie to even); `1 + 3*2^-24 == 1 + 2^-22` (tie, even); a truncation detector.
- Contraction detector: `a = b = 1 + 2^-12`, `c = -(1 + 2^-11)`: `a*b + c` must be `0`, fused gives `2^-24`.
- FTZ/DAZ detector: `2^-126 * 0.5f == 2^-127` and `2^-127 * 2 == 2^-126`.
- `sqrtf(2) == 0x3FB504F3`, `sqrtf` of a subnormal, `1/3`, `remainderf(7.5, 2*pi)`, `floorf(-0.5)`.
- min/max on `(+0,-0)`, `(-0,+0)` through the scalar path and the wide path (export a tiny Box3D probe or
  replicate the three implementations), and `abs(-0)` through each path.
- `b3ComputeCosSin` and `b3Atan2` at eight angles; `lpCbrt` at eight values; a compile-time-folded expression
  compared with the same expression on volatile inputs.
- Float-to-int of in-range values; the guard's reading of MXCSR/FPCR.
- The existing `lpf_bench` final hashes per scene stay the golden end-to-end check.

### 6.5 Replacing `cbrtf`

`lpCbrt(x)`: exponent-hack seed (`i/3 + 709921077` on the bits), three Newton steps
`y = y - (y*y*y - x) / (3*y*y)` in float, sign restored with `copysignf`. Pure IEEE ops, identical everywhere,
about 1e-7 relative error, not correctly rounded, which nothing needs. Or precompute `shape->size` once at shape
creation, since six of the seven simulation sites take the cube root of a piece volume. Either way `cbrtf` leaves
stress.c, world.c, impact.c; the cosmetic sites (particle sizes) may keep it but should not, for grep hygiene.

### 6.6 libm in scene building

A scene built locally on each peer from a seed is simulation state at tick 0; `scenes.c:354` (`atan2f`), `:621`
(`sinf`), `:1170` and `:1620` (`cbrtf`) would desync peers before the first step. Rule: scene building is
simulation code and uses the dialect (`b3ComputeCosSin`, `b3Atan2`, `lpCbrt`, integer `ceil` for counts). Belt and
braces: the handshake compares the tick-0 `lpWorld_Hash`, and the host can ship the built world through the
serialisation primitive T-tracks propose (H5), which removes scene code from the deterministic boundary entirely.

### 6.7 Explicit-FMA dialect versus no-FMA dialect

| | No-FMA (chosen) | Explicit FMA everywhere |
|---|---|---|
| CPU coverage | every x64 and ARM64 CPU | x64 needs FMA3 (Haswell 2013 / Piledriver 2012); Steam August 2026: FMA 95.59%, so 4.4% excluded, concentrated in low-end Celeron/Pentium/Atom parts before Alder Lake-N (2023) |
| Emulators | SSE2 only: Rosetta any version, Prism any version, box64, FEX | FMA3 emulation needs macOS 15 Rosetta and Windows 24H2 Prism; box64/FEX translate `vfmadd` to `fmla` exactly (both fused) |
| WASM | native | no scalar fma; software `fmaf` per op (exact but slow); `relaxed_madd` nondeterministic |
| GPUs | CUDA `--fmad=false`; D3D `precise`; SPIR-V `NoContraction`; Metal `safe` | CUDA/Metal/OpenCL guarantee a fused `fma`; D3D11/D3D12 fp32 do not (no fp32 fma op); SPIR-V `OpFma` precision only "inherited" (recalled); WGSL not guaranteed |
| Speed | Box3D already runs without FMA; our core is scalar | a few percent on the solver kernels (Jolt: about 8% for its whole determinism mode, which also gives up `/fp:fast`) |
| Verdict | portable, cheap, matches every engine that succeeded | breaks the emulator and toaster requirements and cannot be expressed on D3D; not worth a few percent |

## 7. Verdicts

| Question | Verdict | Confidence |
|---|---|---|
| CPU cross-platform, x64 + ARM64, MSVC/clang/gcc | Bit-identical for Box3D's and our operation set under the dialect, after three fixes: gcc gets `-ffp-contract=off` everywhere (CMakeLists.txt:28), `cbrtf` leaves simulation and scene code, NEON min/max patched to SSE semantics (or accepted as a sign-of-zero hash risk until T8 shows it never fires). Same-binary determinism across CPUs is already true (no FMA, no estimates, no UCRT dispatch in the sim path once `cbrtf` is gone). | high on mechanism (two engines' CI), medium until T8's matrix runs |
| x64 under emulation on ARM64 | Rosetta 2: bit-exact by hardware design and measurement. box64: bit-exact for the dialect by source inspection, with or without its fast modes; force them off anyway. FEX and Prism: undocumented; likely fine for SSE2 arithmetic (both must emulate MINPS and conversions somehow; FEAT_AFP is available to Prism on Snapdragon X) but unverified. | Rosetta high; box64 medium-high; FEX and Prism low |
| CPU <-> GPU and GPU cross-vendor | D3D11: not achievable as a flag set (mandatory FTZ, truncation permitted, 1 ULP div/sqrt, unfused-or-fused mad); achievable only for a subnormal-free kernel with `precise` plus software div/sqrt, validated per vendor by the probe. CUDA: achievable on paper with `--fmad=false` and defaults, NVIDIA only. Vulkan: achievable where float controls (DenormPreserve, RTE) are reported, per device. Cross-vendor on D3D11: not guaranteed; the probe will show whether the RTX 3060 and the Intel UHD even agree with each other on div and sqrt. Recommendation to T5: GPU stays off the deterministic path; cosmetic and async work only. | high |

H1 (Box3D float with pinned flags, not pinned compilers, is bit-exact across MSVC, clang, gcc on x64 and ARM64;
residual hazards are ours): **confirmed with an amendment**. The residuals are mostly ours (gcc flag, `cbrtf`,
scene libm, unclamped conversions), but one is Box3D's (NEON versus SSE min/max on +-0 and NaN; the scalar wide
path's `<=`), so "pinned flags" must include one Box3D patch.

H6 (x64 SSE2 under Rosetta 2 and Prism is bit-exact by default, and under box64/FEX with fast modes off):
**confirmed for Rosetta 2, supported for box64 (even with fast modes on, for this dialect), open for Prism and FEX.**

## 8. What the T8 experiments should look for

1. MSVC versus clang-cl, both with contraction off: expect identical `lpf_bench` hashes on every scene. If they
   differ, bisect with the self-test first (literal parsing, folding), then per-function hashes (`lpWorld_HashStress`).
2. MSVC `/arch:AVX2` (VS2022): expect identical. MSVC `/fp:contract` and clang `-ffp-contract=on -mfma`: expect
   different (positive controls; if they are identical, the benches do not exercise enough arithmetic).
3. SIMD versus scalar (`BOX3D_DISABLE_SIMD`): the scalar wide path uses `<=` in `b3MinW/b3MaxW` (simd.h:855-873)
   and `a < 0 ? -a : a` in `b3AbsW` (line 820), so expect sign-of-zero differences to appear in Box3D's internal
   state and possibly in `lpWorld_Hash`; a diff that is only in the sign of zeros confirms hazard 5, anything else
   is a new finding. Also log `b3AndW`/`b3OrW` mask semantics (0/1 floats on the scalar path) as a suspect.
4. GitHub Actions matrix: Linux gcc x64 without the CMake fix will still match (no FMA instructions without
   `-march`); Linux gcc ARM64 without the fix will differ (contraction in `src/`); with the fix, all should match
   except for hazard 5 on ARM64 (NEON min/max). Run each with `-march=native` as a control. Add a
   `-flto` build. Include `lpf_test` with the self-test vector on every leg.
5. Rosetta 2 (MacBook): run the x64 build under Rosetta and compare with the native ARM64 build and the Windows
   hash; also Docker Rosetta-for-Linux if convenient (the bun issue suggests it is a different code path).
6. box64 (DGX Spark): defaults, then `BOX64_DYNAREC_FASTNAN=0 BOX64_DYNAREC_FASTROUND=0`, then
   `BOX64_DYNAREC_X87DOUBLE=1` (exercises glibc's x87 `remainderf`); expect identical throughout. If FEX is
   installable, the same with its x87 reduced-precision option on and off.
7. D3D11 probe on the RTX 3060 and the Intel UHD, fxc and dxc, `/Gis` on and off, `/O0` and `/O3`: (a) add/mul
   RNE versus truncation (`1 + 3*2^-24` must give `1 + 2^-22`; `1 + 2^-24` must give `1`); (b) div and sqrt versus
   a CPU correctly rounded reference over random and adversarial pairs, reporting max ULP and mismatch rate per
   vendor; (c) mad fusion with `a = b = 1 + 2^-12, c = -(1 + 2^-11)` with and without `precise`; (d) subnormals:
   `2^-126 * 0.5`, and a subnormal passed through `mov`, `abs`, `min`, and a `dtof` of `2^-127`; (e) `min(-0,+0)`,
   `max(+0,-0)`, `min(NaN,1)`; (f) `ftoi` of `3e9` and NaN; (g) `rcp`/`rsq` ULP. Interpretation key: (a) failing
   means even add/mul cannot match; (b) above 0 ULP means software div/sqrt is mandatory; (c) fusing under
   `precise` means the driver ignores the flag; (d) flushing under all settings is expected and is the reason for
   the verdict; vendor disagreement on any of (b), (e), (g) is the cross-vendor answer.
8. WASM, only if pursued: Emscripten build of `lpf_bench` (`-msimd128 -msse2 -ffp-contract=off -pthread`) under
   Node; expect the native hash; measure the slowdown.

## Sources

Repo: `CMakeLists.txt:27-37`; `docs/determinism-rules.md:9-11, 54-56`; `src/CMakeLists.txt:34`;
`extern/box3d/CMakeLists.txt:76-108`; `extern/box3d/src/CMakeLists.txt:130-132`; `extern/box3d/src/core.h:39-76`;
`extern/box3d/src/simd.h:51-63, 131-146, 296-321, 430-474, 626-670, 820-873`;
`extern/box3d/src/contact_solver.c:1867-2118`; `extern/box3d/include/box3d/math_functions.h:165-217, 252-295, 662-679`;
`extern/box3d/src/math_functions.c:170-264`; `extern/box3d/src/qsort.h:23-27`; `extern/box3d/src/scheduler.c:126`;
`extern/box3d/src/timer.c:203-374`; `extern/box3d/src/height_field.c:670-1230`; `extern/box3d/src/mesh.c:396-398`;
`src/tasks.c:114-116`; `src/stress.c:134, 254, 536, 554, 1231`; `src/world.c:576, 633`; `src/impact.c:320, 402, 1018`;
`src/step.c:146`; `src/split.c:149`; `src/fracture.c:631-657, 1199`; `src/debris.c:40, 477`; `src/supply.c:191`;
`scenes/scenes.c:354, 621, 1170, 1620`.

Web (fetched 2026-09-29 unless noted):
- Box2D determinism (2024-08): https://box2d.org/posts/2024/08/determinism/
- Box2D 3.1 (2025-04): https://box2d.org/posts/2025/04/box2d-3.1/
- Jolt: https://raw.githubusercontent.com/jrouwe/JoltPhysics/master/Docs/Architecture.md;
  https://raw.githubusercontent.com/jrouwe/JoltPhysics/master/Jolt/Jolt.cmake;
  https://raw.githubusercontent.com/jrouwe/JoltPhysics/master/Build/README.md;
  https://github.com/jrouwe/JoltPhysics/blob/master/.github/workflows/determinism_check.yml
- Rapier: https://rapier.rs/docs/user_guides/rust/determinism/
- PhysX: https://nvidiagameworks.github.io/PhysX/4.1/documentation/physxguide/Manual/BestPractices.html
- Unity Burst: https://docs.unity3d.com/Packages/com.unity.burst@1.8/changelog/CHANGELOG.html;
  https://docs.unity3d.com/Packages/com.unity.burst@1.8/manual/compilation-burstcompile.html;
  https://docs.unity3d.com/Packages/com.unity.burst@1.8/api/Unity.Burst.FloatMode.html;
  https://discussions.unity.com/t/what-is-the-state-of-bursts-floatmode-deterministic/1668491
- Factorio: https://factorio.com/blog/post/fff-52; https://www.factorio.com/blog/post/fff-370;
  https://forums.factorio.com/viewtopic.php?p=703214; https://wiki.factorio.com/Desynchronization
- Photon Quantum: https://doc.photonengine.com/quantum/current/manual/quantum-ecs/fixed-point
- MSVC: https://learn.microsoft.com/en-us/cpp/build/reference/fp-specify-floating-point-behavior?view=msvc-170;
  https://devblogs.microsoft.com/cppblog/the-fpcontract-flag-and-changes-to-fp-modes-in-vs2022/
- GCC: https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html
- Clang: https://clang.llvm.org/docs/UsersManual.html
- glibc accuracy: https://sourceware.org/glibc/manual/latest/html_node/Errors-in-Math-Functions.html
- Bruce Dawson (2013): https://randomascii.wordpress.com/2013/07/16/floating-point-determinism/
- Intel semantics: https://www.felixcloutier.com/x86/minps; https://www.felixcloutier.com/x86/cvttss2si
- D3D9: https://learn.microsoft.com/en-us/windows/win32/direct3d9/d3dcreate
- Rosetta 2: https://dougallj.wordpress.com/2022/11/09/why-is-rosetta-2-fast/;
  https://zenn.dev/mod_poppo/articles/arm-feat_afp;
  https://developer.apple.com/documentation/apple-silicon/about-the-rosetta-translation-environment;
  https://github.com/oven-sh/bun/issues/19677
- Prism: https://learn.microsoft.com/en-us/windows/arm/apps-on-arm-x86-emulation;
  https://learn.microsoft.com/en-us/windows/arm/apps-on-arm-program-compat-troubleshooter;
  https://learn.microsoft.com/en-us/answers/questions/2264223/floating-point-errors-with-prism-emulator
- FEX: https://github.com/FEX-Emu/FEX/issues/4191; https://fex-emu.com/FEX-2408/; https://fex-emu.com/FEX-2607/
- box64: https://github.com/ptitSeb/box64/blob/main/docs/USAGE.md;
  https://raw.githubusercontent.com/ptitSeb/box64/main/src/dynarec/arm64/dynarec_arm64_0f.c
- Linux AFP hwcap: https://www.kernel.org/doc/html/latest/arch/arm64/elf_hwcaps.html
- WASM: https://raw.githubusercontent.com/WebAssembly/design/main/Nondeterminism.md;
  https://raw.githubusercontent.com/WebAssembly/relaxed-simd/main/proposals/relaxed-simd/Overview.md;
  https://docs.wasmtime.dev/examples-deterministic-wasm-execution.html;
  https://emscripten.org/docs/porting/simd.html;
  https://raw.githubusercontent.com/emscripten-core/emscripten/main/system/include/compat/xmmintrin.h
- GPU: https://docs.nvidia.com/cuda/floating-point/index.html;
  https://docs.nvidia.com/cuda/cuda-compiler-driver-nvcc/index.html;
  https://learn.microsoft.com/en-us/windows/win32/direct3d11/floating-point-rules;
  https://learn.microsoft.com/en-us/archive/blogs/marcelolr/precise-and-ieee-strictness-in-hlsl;
  https://raw.githubusercontent.com/microsoft/DirectXShaderCompiler/main/docs/DXIL.rst;
  https://docs.vulkan.org/spec/latest/appendices/spirvenv.html;
  https://developer.apple.com/documentation/metal/mtlcompileoptions/fastmathenabled;
  https://developer.apple.com/documentation/metal/mtlmathmode;
  https://registry.khronos.org/OpenCL/sdk/1.2/docs/man/xhtml/clBuildProgram.html;
  https://registry.khronos.org/OpenCL/sdk/3.0/docs/man/html/clGetDeviceInfo.html
- Steam hardware survey, August 2026: https://store.steampowered.com/hwsurvey/
- Not fetched, cited from the literature: S. A. Figueroa, "When is double rounding innocuous?", ACM SIGNUM
  Newsletter 30(3), 1995; Markstein, "IA-64 and Elementary Functions" (2000) for the fma division scheme.
