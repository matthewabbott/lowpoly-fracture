# The GPU lab

Research code for milestone 11's arithmetic decision (docs/roadmap.md §11): can a rigid-body core on the GPU be
bit-identical across machines, and in which arithmetic, the float dialect F or block-scaled integers (V4)? It is
outside the engine: nothing in `src/`, the tests, `tools/build.ps1` or the determinism CI builds or reads it. Read it
only for GPU work. The record of what it found is `docs/research/m11-gpu-lab.md`. Milestone 7's E11 (the first
measurement, `docs/research/m7-gpu-experiments.md`) lives here now, unchanged in behaviour: its runners reproduce
E11's hashes and logs.

## What is here

| path | what it is |
|---|---|
| `lab.py` | the one tool: `gen`, `build`, `run`, `check`, `remote` (standard-library Python) |
| `src/solver.slang`, `layout.h` | E11's contact solve (Box3D's soft step): one Slang source, compiled to SPIR-V for the GPUs and to C++ for the CPU twin, per number format (F, I32, I64, V4) |
| `src/harness.c` | runs the solve on every Vulkan GPU and on the twin (1 and 8 threads), compares every word, traces a divergence to its dispatch (`--trace`) |
| `src/rows.slang`, `rowlayout.h`, `rows.c` | one contact row in five formats (V1 float to V4b), 2^20 rows of four kinds, against a float64 reference |
| `src/probe.slang`, `probe.c` | each float and integer operation, ~1M operand triples per op in eight classes (subnormals, ties, signed zeros, ...), under each float-control mode, against the CPU |
| `src/mulbench.comp`, `mulbench.c` | multiply throughput per format (the cost of an int32×int32→int64 product) |
| `src/vk_util.c/.h` | the small Vulkan compute layer; `twin_glue.cpp`, `rows_glue.cpp`, `twin_info.h`: the twins' entry points |
| `reference.json` | E11's twin hashes, which every twin on every compiler and ISA must reproduce |
| `results/<machine>/` | logs and `summary.md` per machine (committed: they are the evidence) |
| `gen/` | generated SPIR-V and twins (`lab.py gen`; not committed) |

## Commands

```bash
python lab/gpu/lab.py gen                     # Windows, Vulkan SDK (C:\VulkanSDK\1.4.357.0 or VULKAN_SDK): kernels and twins
python lab/gpu/lab.py build                   # CMake into lab/gpu/build/<compiler>; --compiler msvc|clang-cl|gcc|clang
python lab/gpu/lab.py run --machine win-laptop --twins clang-cl    # every suite; --quick for a smoke test
python lab/gpu/lab.py check lab/gpu/results/win-laptop             # rewrite summary.md from the logs
python lab/gpu/lab.py remote consulear@spark-d683 --machine spark-gb10 --twins clang   # copy, build, run there, fetch
```

Suites: `probe`, `solve`, `rows`, `controls` (the positive controls), `mulbench`, `twins` (the twins of the
compilers named by `--twins`, without GPUs). A run takes about 15 minutes on the laptop.

## Rules this lab follows, and what broke them

- **Kernels are generated once, on Windows, and shipped.** Every machine then tests its own C compiler and its own
  GPU driver, never a different `slangc`.
- **No contraction, anywhere:** `/fp:precise` on MSVC, `-ffp-contract=off` on gcc and clang (both contract by
  default on ARM64), for the twins and for the C that builds the scenarios. The positive controls
  (`harness_F-fma` and `rows_V1-fma` let the compiler fuse; `gen/F-fast` and `V1.fast.spv` are `slangc -fp-mode
  fast`) must differ, which proves the harness would see a change.
- **No side effects inside one expression's operands or one call's arguments** (docs/determinism-rules.md). E11's
  `rows.c` and `probe.c` drew random numbers inside call arguments, so clang made a different corpus from MSVC. The
  port draws in the order MSVC happened to use (found against E11's hashes and log), so every compiler makes E11's
  corpus.
- **Pinned references.** `probe.c`'s CPU reference for `max()` and `min()` was `fmaxf`/`fminf`, which leave the sign
  of `max(-0, +0)` to the C library; it is written out. NaN payloads in the probe's CPU reference still follow the CPU
  and the compiler's operand order (the probe's `nanPay` column).
- **Float-control modes a device does not advertise are skipped**, never run (`vku_tag_supported`). SPIR-V file tags:
  `.rte`, `.szinp`, `.preserve`, `.ftz`, `.rtesz` (RTE + SignedZeroInfNanPreserve), `.pszinp` (DenormPreserve +
  SignedZeroInfNanPreserve); `--tag VENDOR=TAG` picks them per vendor (Intel gets `.preserve` by default).

`E11_TRACE_PIPELINES=1` prints each pipeline before the driver compiles it (a driver whose compiler crashes is a
result: the GB10's did on `RoundingModeRTE`). `E11_VALIDATION=1` turns on the validation layer.

## Machines

The laptop (RTX 3060 and Intel UHD 630, Windows, MSVC and clang-cl), the DGX Spark (GB10 and llvmpipe, aarch64 Linux,
gcc and clang; `ssh consulear@spark-d683`, work in `~/lpf-m11`), and later a MacBook (Apple GPU through MoltenVK) and
the owner's low-end box (Pascal, AMD). On macOS, MoltenVK is found through `VK_KHR_portability_enumeration`; run with
`MVK_CONFIG_FAST_MATH_ENABLED=0`.
