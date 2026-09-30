# M7 research, track T5: GPU rigid-body physics

Date: 2026-09-29. Question: what is the state of the art in GPU rigid bodies, and what of our destruction engine
should run on a GPU, given player-hosted multiplayer (T1 decides the model) and the toaster goal. T3 owns GPU
arithmetic rules and T4 fixed point on GPUs; this track covers float GPU physics.

## Verdicts first

- **H4 confirmed on every part.** No engine in the survey claims bit-exact results across GPUs or vendors; the best
  class anyone demonstrates is "same device, same driver, same binary". GPU results therefore cannot touch lockstep
  state, and in host-authoritative sync they would make the host's simulation unreproducible on any other machine
  (replays, tests, agents). On an iGPU the fixed costs alone (35-40 us per kernel launch on Intel iGPUs, no FP64 on
  Xe-LP, 0.44 TFLOPS on a UHD 620, one memory bus shared with the renderer) exceed what Box3D's SSE path costs on
  2-4 cores at our awake-body counts (1500 cap; physics 0.5-1 ms a step after the debris tiers).
- **The one candidate with a real gain is a cosmetic debris layer** that never feeds back: upload-only, rendered
  straight from GPU buffers, no readback (which the vendored sokol cannot do anyway). 100k pieces is realistic on a
  discrete GPU with full contacts (AVBD: 110k bricks in 9.8 ms on an RTX 4090); on a Steam Deck-class iGPU expect
  20-50k ballistic pieces colliding with static geometry only, and 5-10k with piece-piece contacts (estimates).
- **The async stress solve is not a candidate.** Its results decide which bonds break (state); it needs FP64
  accumulators (absent on Intel Xe-LP, 1/16 rate on RDNA2, 1/64 on NVIDIA consumer parts); each conjugate-gradient
  iteration needs two global reductions, so 50-200 iterations cost more in launch latency on an iGPU than the CPU
  spends on the whole solve (60k bond-iterations, about 4 ms on one core). The perf log's own plan (SoA 6-vectors,
  precomputed blocks, a parallel K.x) is the right lever.
- **Fracture clipping, the broadphase and the whole solver: no.** Details in section 5.
- **yacineMTB's CUDA Box3D** (tweet of 2026-07-06): "about 30x faster on my GPU compared to my CPU (depending on
  the situation)"; 200k bodies claimed in secondary coverage; no GPU or CPU model named; no determinism claim of any
  kind (same GPU or across vendors); no source, no writeup. It is evidence that Box3D's structure ports to CUDA, not
  evidence about determinism or toasters.

## 1. Survey

Numbers are as published; dates are of the source. "Determinism" is what the source claims, not what may be true.

| engine | solver | GPU API | scale numbers | determinism | open source | date |
|---|---|---|---|---|---|---|
| NVIDIA PhysX 5.6 GPU rigid bodies | substepped PGS/TGS (`PxSolverType`, not re-verified this session); GPU broadphase, contact gen, solver; joint projection, CCD, triggers, contact modification stay on CPU | CUDA only | "scenes with several thousand active actors" is where the GPU pays; default buffers size about 10k bodies; convex hulls capped at 64 vertices (larger fall back to CPU); Isaac Gym on PhysX: Ant, 4096 envs, 540k env-steps/s on an A100 (2021) | "limited": identical runs need the same scene, same release, same platform; nothing GPU-specific; Isaac Gym release notes list fixed GPU-pipeline sync non-determinism | BSD-3; GPU kernel source (500+ CUDA kernels) opened 2025-03-25 | docs 5.6.0 (2025); Isaac Gym paper 2021 |
| Bullet 3 OpenCL pipeline (Erwin Coumans) | PGS in graph-coloured batches, plus Jacobi with mass splitting inside a 64-thread compute unit (hybrid); constraint rows sorted for determinism | OpenCL | "100 thousand rigid bodies in real-time on a high-end desktop GPU"; README wants a Radeon 7970 / GTX 680 or better and says laptop GPUs will not do well; 50 kernels, 100% on GPU | order kept by sorting rows and batches; no cross-vendor claim | zlib; experimental, driver kernel-compile failures noted | GDC/AMD 2013 |
| MuJoCo MJX (JAX) | Newton or CG on generalized coordinates | JAX on CUDA/TPU | one humanoid: 950k steps/s on an A100 at batch 8192; 2.7M on an 8-chip TPU v5 at batch 16384; "10x slower than MuJoCo" for a single scene; degrades faster than CPU with contact complexity | none stated | Apache-2.0 | docs, 2025-26 |
| MuJoCo Warp (MJWarp) | as MJX; Newton solver; PGS "not yet supported" | NVIDIA Warp (CUDA); CPU only for debugging | MJX-Warp humanoid 2.96M steps/s (pure Warp 3.35M); hardware not stated in the docs excerpt | none stated; Warp 1.15 (2026-07-07) added a deterministic mode for float atomics: `RUN_TO_RUN` (same GPU) and `GPU_TO_GPU` | Apache-2.0 | 2025-26 |
| NVIDIA Newton (Disney, DeepMind, NVIDIA; Linux Foundation) | multi-solver: MuJoCo Warp, XPBD, VBD, Featherstone, Euler | Warp; NVIDIA Maxwell+ with CUDA 12; macOS CPU-only | no numbers published on the repo page; Isaac Lab 3.0 beta integrates it | none stated | Apache-2.0 | 2025-26 |
| Genesis | rigid (MuJoCo-derived), MPM, SPH, FEM, PBD, coupling | Quadrants (a Taichi fork, June 2025): CUDA, ROCm, Metal, Vulkan, x86, ARM | claims 43M FPS for a Franka arm on an RTX 4090; independent re-runs with contacts enabled get about 0.29M FPS, "off by 150x", 3-10x slower than other GPU simulators | none stated | Apache-2.0 | Dec 2024; critique Dec 2024-2025 |
| Brax | generalized (Featherstone-like), positional (PBD), spring pipelines | JAX on TPU/GPU | "millions of physics steps per second on TPU" | none stated | Apache-2.0 | NeurIPS 2021 |
| Isaac Lab | PhysX 5 GPU via Isaac Sim; Newton integration in 3.0 beta | CUDA | as PhysX | as PhysX | BSD-3 | 2025-26 |
| Dimforge wgrapier / wgsparkl | wgrapier: BVH broadphase + Soft-TGS; wgsparkl: MPM | WGSL/WebGPU; moving to rust-gpu in 2026 because of WGSL's limits | 93k bodies + 120k joints; a 34k-plank stack (no frame times given) | none stated | Apache-2.0/MIT; experimental | 2025 review, 2026-01-09 |
| AVBD (Giles, Diaz, Yuksel; Roblox + Utah) | augmented-Lagrangian vertex block descent: per-body 6x6 solves, bodies graph-coloured, hard constraints, joints, friction, stacking | paper: CUDA on an RTX 4090; ports in WebGPU | paper numbers (via three-avbd's table): brick ring 110k bodies 9.8 ms (4 iterations); jointed drop 34k bodies + 71k joints 16 ms (10 iterations, with cloth); brick gables 506k bodies 17.6 ms (3 iterations). three-avbd on an M4 Max in Chrome: 10.4 / 6.4 / 25.5 ms | three-avbd: "deterministic on a given device"; a GPU step matches the CPU reference to 3e-6 (not bit-exact) | paper code MIT (avbd-demo2d/3d); three-avbd open | SIGGRAPH 2025 (TOG 44(4), Aug 2025); ports 2025-26 |
| VBD (Chen, Liu, Yang, Yuksel) | block coordinate descent on vertices, vertex colouring (8 colours vs 76 for element colouring); Jacobi-style for same-colour collision pairs | CUDA, RTX 4090 | deformables of 36-48M vertices at 3.6-4.7 s per frame; handles 1:2000 mass ratios where XPBD fails | not addressed | code released | SIGGRAPH 2024 |
| Jolt | CPU soft-step PGS-family, multicore | none; a compute interface PR (#1847, Dec 2025); would target D3D12/Vulkan/Metal, not CUDA | n/a | cross-platform determinism on CPU (v5.3.0 tests it) | MIT | 2025 |
| Unreal Chaos | CPU rigid bodies (Intel CPU optimisation work) | none for rigid bodies | n/a | n/a | source-available (EULA) | UE 5.x |
| Unity | built-in PhysX 4 (CPU); Unity Physics (C#, Burst, jobs) "completely deterministic", no platform statement | none | n/a | as stated | Unity Physics package | 1.3 docs |
| Roblox | production solver: PGS family (unverified); AVBD authored by Roblox staff | n/a | n/a | n/a | n/a | the June 2026 rollout is physics *replication*, not a solver change |
| yacineMTB, CUDA Box3D | Box3D's solver "rewritten entirely in native cuda" (with an agent) | CUDA | "about 30x faster on my GPU compared to my CPU (depending on the situation)"; 200k rigid bodies (secondary coverage, unverified); GPU/CPU models unstated | none claimed | not released ("just the claim, the demo, and the CUDA flex") | 2026-07-06 |

Erin Catto's own position: no direct statement on GPU physics was found in his blog (Determinism 2024-08-27, Solver2D
2024-02-05, Announcing Box3D 2026-06-30, SIMD for Collision 2026-07-18), the Box3D README, or the two fetched
tweets. Solver2D lists "GPU friendly solvers. Jacobi, mass-splitting, etc." as untried future work. Box3D's design
is a CPU design end to end: add(mul) instead of FMA so SIMD matches scalar, per-worker bit arrays merged in order,
pair keys sorted so contact order is independent of the tree, a wide (4-lane) contact solver. SE Radio 739
(2026-09-23) is an interview about physics engines that may contain a GPU remark; unverified (not listened to).

Box3D CPU scale for reference: the July 2026 SIMD post reports the convex pile benchmark (5120 hulls of 32 points)
on a Ryzen 7950X at 2,410 ms for the run on 8 threads with SSE2 (5,292 scalar; 17,337 single-thread SSE2). A
Babylon.js port (wasm SIMD128, Ryzen 5900X, 2026-09-17) steps a 4000-body mixed pile in 20.9 ms on one thread and
6.31 ms on 8 workers, a 5050-body pyramid in 28.2 / 5.72 ms, a 1275-body pyramid in 0.98 / 0.34 ms; native SSE2
would be faster still.

## 2. Solver formulations for the GPU

| formulation | GPU shape | tall stacks | rubble piles | soft welds and motors | notes |
|---|---|---|---|---|---|
| Graph-coloured Gauss-Seidel / PGS (Box3D's soft step) | one dispatch per colour per iteration; no atomics inside a colour; the overflow colour is serial (Box3D solves it on the main thread, `contact_solver.c:2273-2307`) | good with substeps and warm starting (Solver2D's conclusion; Box3D ships it) | good | good: joints are coloured too (`constraint_graph.c:216`), motors are joint rows | Box3D uses 24 colours (`constants.h:44`), 20 for dynamic pairs, statics fill from the top, colour 23 overflows; per substep 1 solve + 1 relax iteration (`solver.c:28-29`) with stages `integrate velocities, warm start x C, solve x C, integrate positions, relax x C` (`solver.c:1240-1317`). Ported literally, a step is about (2 + 3C) dependent dispatches per substep. |
| Jacobi | one dispatch per iteration, fully parallel, order independent | jitters and converges slowly at game iteration counts (Tonge et al. 2012: PGS spreads residual by order, Jacobi is order-free but slower); Bullet 3 used it only inside a compute unit | worse than PGS at equal iterations | motors and stiff welds converge poorly | mass splitting (Tonge 2012) fixes the jitter by dividing each body's mass by its contact count in the effective mass; still needs more iterations than PGS. |
| TGS / substepped Gauss-Seidel (Macklin 2019 "small steps") | as PGS, with the position update per substep | what Box3D and Jolt do; Solver2D: soft step (a TGS variant) and XPBD stable at low cost | good | good | this is our current solver; on a GPU it inherits the per-colour dispatch structure. |
| XPBD (Macklin, Mueller, Chentanez 2016) | as Gauss-Seidel per colour, or Jacobi | comparable to soft step in Solver2D | good | joint motors exist (Mueller 2020); VBD reports XPBD failing at 1:2000 mass ratios | compliance maps directly to our soft welds. |
| VBD (2024) / body-coloured block descent | one dispatch per body colour per iteration; a 6x6 local solve per body; far fewer colours than constraint colouring | unconditionally stable at any iteration count; primal, so mass ratios are fine | good | soft: fine; hard joints need AVBD | same-colour collision pairs are handled Jacobi-style with a second buffer. |
| AVBD (2025) | as VBD plus a dual (multiplier) and stiffness update per constraint | the paper's stacking and joint demos at 3-10 iterations; the only formulation here with published GPU numbers for rigid stacks and joints at 100k+ | good | augmented Lagrangian handles infinite stiffness and stiff-to-soft ratios; joints with limited degrees of freedom are in the paper; motors are not discussed (unverified) | the best fit if anything of ours ever runs on a GPU: it degrades gracefully under a fixed iteration budget, which is what a count-capped GPU pass needs. |

For our step (1/60, 4 substeps): Box3D's soft step is already the substepped Gauss-Seidel the literature settled on
for stacks and joints; a GPU version of it keeps the same numerics but pays about 3C+2 barriers per substep. AVBD is
the formulation that fits a GPU budget model (iterations as a count, stable at any count), but it is a different
solver with different behaviour (bit-for-bit incompatibility with Box3D, retuned welds, motors unverified) and would
be a rewrite, not a port.

## 3. What deterministic GPU physics requires

On one GPU (same model, same driver, same shader binary):

- **No float atomics.** Accumulation order follows the warp scheduler and differs run to run (NVIDIA CCCL blog on
  floating-point determinism; Warp needed a whole deterministic mode, 1.15.0, 2026-07-07, that records and sorts).
  Colouring avoids atomics by construction: within a colour no two constraints touch one body, so writes are plain
  stores. Where a sum over a variable set is unavoidable (contact impulses per body for our stress loads, CG dot
  products), either gather in a fixed order or accumulate in **integer/fixed-point atomics**, which are associative
  and therefore order-independent (a 2026 MPM study, arXiv 2609.34666, found scheduling-dependent float sums flipping
  the sign of long-horizon gradients and fixed-point accumulation giving order-independent repetition; the
  reproducible-summation literature, Demmel and Nguyen, uses binned or long accumulators for the same reason).
- **Deterministic reductions:** a fixed tree shape, which means a fixed workgroup size and grid per array length;
  changing a tile size changes the rounding order.
- **Deterministic sorts and compaction:** radix sort on total keys with index tiebreaks (Box3D already sorts pair
  keys, `broad_phase.c:710-712`); pair lists built by prefix sum, not by an atomic counter, or by counter plus sort
  (Warp's second pattern).
- **Fixed colouring:** a greedy colouring in a fixed order, or Jones-Plassmann with priorities seeded from state
  (three-avbd does this and claims same-device bit repeatability).
- **No timing-dependent control flow:** no early exit on a convergence test read back mid-step, no indirect
  dispatch whose count depends on a race.
- **The driver is part of the binary.** DXIL, SPIR-V and PTX are compiled by the user's driver at run time; a driver
  update can change contraction or scheduling and break replays recorded before it. PhysX's own guarantee is "same
  platform, same release". The CPU binary we ship is fixed; a GPU kernel is not.

Across GPUs (T3 owns the arithmetic; the conclusion): basic IEEE operations are correctly rounded on all current
GPUs, so cross-vendor bit-exactness is *possible in principle* for straight-line code without contraction, fast
intrinsics, denormal flushing or wave-size-dependent reductions. Nobody in the survey claims to have done it. The
tooling is uneven: CUDA has `--fmad=false`, HLSL has `precise`, SPIR-V has `NoContraction`; **WGSL explicitly
permits reassociation and fusion (spec section 15.7.5; gpuweb issue #2402 "reassociation is always allowed")**, so
WebGPU cannot promise bit-exact results even on one machine across browser updates. Warp's `GPU_TO_GPU` mode covers
only its atomics. The practical determinism classes are therefore: **none** (cosmetic), **same GPU + driver**
(achievable with care), **cross-GPU** (unproven; an experiment, section 9).

## 4. The round trip

What our step reads from physics every tick (`src/step.c`): after `b3World_Step` (line 332) it reads joint forces
for every awake link (`lpPollLinks`, 335), body velocities for inertia relief (`lpTrackMovingBodies`, 336), ray
casts for ghost flight plans (`lpStepGhosts`, 338), transforms and contacts for shoves (`lpShove`, 339), hit events
and per-body contact data with `totalNormalImpulse` for stress loads (`lpCollectHits`, 340), and body move events
with `fellAsleep` to freeze rubble by `b3Body_SetType` (`lpFreezeOrKill`, 30-79). Before the step it creates bodies
with variable hulls of up to 128 vertices (PhysX's GPU path caps hulls at 64), rebuilds joints (`lpSyncLinks`), casts
wheel and foot shapes, reads transforms and velocities for pulls (210-226), and sets motor speeds and torques. Every
one of these is a GPU-to-CPU or CPU-to-GPU transfer if the solver lives on the GPU, and the writes change topology
(new bodies, destroyed bodies, static/dynamic flips) every tick.

Which of ours tolerate a one-step lag: collision-driven impacts already do (hits are queued to `nextImpacts` and
processed next step, 292-296); the stress solve does by design (results a step or more late); rubble freezing does
(sleep is a many-step signal); link tearing does (a 3-step settle before judging); wheels and rigs would take a tick
of latency on the forces they compute from transforms (an AI driver or a mech would not notice; the local character
is predicted outside the world in T1's model); tool ray casts could use last tick's tree. So a pipelined GPU step
that runs one tick behind is semantically feasible for the whole core. What is not feasible is a *synchronous* GPU
step at 60 Hz on an iGPU (below).

Costs, discrete vs integrated:

- **Discrete (PCIe):** a host-device copy has a fixed cost of roughly 10 us and kernel launch latency of about 5 us
  on an RTX 4090 or a Radeon 890M (Dalek, arXiv 2508.10481, OpenCL measurements). Our per-tick readback (1500
  awake bodies at about 100 B, 2-3k contacts, events) is about 0.5 MB, tens of microseconds on PCIe 4. The real
  cost is the sync: the CPU waits for the GPU's last dispatch; on D3D11 a `Map(READ)` on a staging buffer stalls
  until the GPU has finished (or `DO_NOT_WAIT` and read it a frame late). Also the CPU cannot start on the results
  until the GPU finishes, so the broadphase/narrowphase overlap Box3D gets across cores is lost (Jolt's argument).
- **Integrated (shared memory):** zero-copy is real on Intel (4096-byte aligned, 64-byte multiples) so the transfer
  cost vanishes, but launch latency is 35-40 us on Iris Xe and Arc mobile against about 5 us on the discrete parts,
  and every dispatch competes with the renderer for the same execution units and the same LPDDR bus (a Steam Deck
  has 88 GB/s for CPU and GPU together, 1.6 TFLOPS FP32; a UHD 620 has 0.44 TFLOPS). A GPU physics millisecond on a
  toaster is a millisecond taken from a 16.7 ms frame the renderer already needs.
- **PhysX's answer** is to not round-trip at all: the Direct GPU API keeps poses, velocities, forces and joints on
  the GPU, disables the CPU getters (they return stale data), and gives up scene queries, character controllers,
  vehicles, contact modification, triggers and CCD. That is the shape of a GPU-resident world: an RL simulator, not
  a game core that queries and edits its world every tick.
- **sokol:** compute passes exist in the vendored `sokol_gfx.h` (D3D11, Metal, GL 4.3, WebGPU; GLES 3.1 since
  2025-04; experimental Vulkan since 2025-12) and storage buffers can be bound as vertex buffers (multi-purpose
  buffers, 2025-05), but there is **no GPU-to-CPU readback** (still "planned" in the 2026-08-09 changelog entry).
  Through sokol, only upload-only, render-only GPU work is possible today.

## 5. What of ours could move

| subsystem | gain | determinism class | round-trip constraint | toaster verdict | effort |
|---|---|---|---|---|---|
| (a) cosmetic debris layer: mess, dust, far debris, sparks, chips that can never be promoted | large: 100k+ pieces on a discrete GPU with full contacts (AVBD: 110k in 9.8 ms on a 4090; 506k in 17.6 ms); on a Steam Deck-class iGPU an estimated 20-50k ballistic pieces against static geometry (a coarse grid/SDF or the depth buffer) and 5-10k with piece-piece contacts; removes those pieces from `Renderer_Sync` (O(everything) today) and from the core entirely | none needed: never feeds back; peers diverge visually and nothing cares | upload only (spawn batches, blast/blow force events); draw from the storage buffer; no readback, which sokol supports today | viable, sized to the GPU; on a UHD 620 keep it small and behind the toaster profile switch | medium: one compute shader (integrate, collide with a static field, settle), a spawn ring buffer, an instanced draw path; the CPU ghost/scrap tiers stay for anything a pull or blast may promote (that needs a position on the CPU) |
| (b) async stress conjugate gradient | frees 1-4 ms a step on the host in the keep and siege rungs (stress 3.28 / 5.38 ms on one worker); the budget is a count, so a GPU would just raise it | same GPU + driver at best: dot products need fixed reductions; FP64 accumulators are absent on Intel Xe-LP and slow elsewhere, so the arithmetic changes versus the CPU; results decide bond breaks (state) | tolerant: results a step or more late already | no: 2-3 dependent launches per CG iteration at 35-40 us each, 50-200 iterations, is 5-25 ms of latency for work the CPU does in 4 ms; and the iGPU is rendering | medium-high; and it breaks headless reproducibility for tests and agents |
| (c) fracture clipping | small: the parallel phase totals 14-26 ms Voronoi, 46-70 ms merge, 14-21 ms hulls per 600 ticks (perf log); spikes of 4-5 ms at 8 workers are dominated by quickhull merges and Box3D shape creation, which stay on the CPU | must be exact across peers (cells are geometry, geometry is state) | hostile: cells are needed the same step (the split follows the impact), so every fracture is a sync stall; variable-size outputs | no | high (branchy clipping, variable topology); T3/T4's exact predicates are the better robustness investment |
| (d) broadphase | none: awake contacts are 1.5-3k after the tiers; Box3D's tree with SAH and sorted pair keys is a small share of a 0.5-1 ms physics step | state | the CPU narrowphase waits for it (Jolt's objection) | no | low but pointless |
| (e) whole rigid-body solver | negative at our scale: Box3D does 4000 bodies in about 6 ms on 8 wasm workers, our physics is 0.5-1 ms a step post-tiers; a literal port is about 150-180 dependent dispatches a step (3C+2 per substep, C about 8-12 active colours, 4 substeps, plus prepare, broadphase, narrowphase, restitution, store), a floor of about 1 ms on a discrete GPU and 5-7 ms on an Intel iGPU from launch latency alone (estimate); plus the per-tick readback and dynamic topology of section 4 | same GPU + driver at best | a full sync every tick, or a one-tick pipelined lag | no | very high; a discrete-only option for worlds past 10-20k awake bodies, which the debris tiers exist to avoid |

## 6. iGPU versus SSE, the crossover, the agent cost, the API

**Is a GPU path ever a net win on an iGPU against Box3D's 4-wide SSE on 2-4 cores?** No, for anything that feeds
back. Per body-step, AVBD on an RTX 4090 costs about 90 ns (110k bodies, 9.8 ms, 4 iterations); scale by FP32
throughput to a Steam Deck (1.6 vs about 82 TFLOPS, 50x) and the same efficiency gives about 2-4k bodies in 10 ms,
while the Deck's four Zen 2 cores run Box3D at 4000 bodies in roughly 6-10 ms (the wasm numbers above on a faster
desktop CPU are 6.3 ms; native SSE2 on the Deck is in that range). Add 35-40 us per launch on Intel iGPUs and the
renderer's claim on the same units, and the GPU loses at every count we run. For upload-only cosmetic work the
comparison is different: the CPU pays nothing, and the GPU work is a fraction of a millisecond.

**At what awake-body count does a GPU solver beat an 8-core CPU?** Published numbers give a band, not a point:
PhysX says "several thousand active actors" (discrete, CUDA); Bullet 3 needed a 2013 high-end desktop GPU for its
100k; Isaac Gym's 540k env-steps/s for Ant on an A100 is about 37k bodies per 7.6 ms batched step; Box3D's 8-thread
pyramid of 5050 bodies is 5.7 ms in wasm. Estimate: a midrange discrete GPU wins somewhere between 5k and 20k awake
bodies, and only when the results stay on the GPU. Our awake cap is 1500.

**Agent-friendliness cost:** a second language (HLSL/GLSL/WGSL/CUDA) and a shader toolchain (sokol-shdc, already in
`tools/`, compiles GLSL compute to HLSL/MSL/WGSL since 2025-03, a real plus); driver-dependent bugs (Bullet's README:
OpenCL drivers failing to compile kernels; sokol's changelog: an Intel Windows Vulkan driver performance bug); no
asserts or printf in kernels, debugging through RenderDoc; and the decisive one: **headless tests and agents have no
GPU**, so `lpf_test` and `lpf_bench` need a CPU reference path that stays bit-identical with the GPU path, which is
two solvers to keep in sync inside a 600k-token budget. A cosmetic layer escapes all of this except the language:
it has no reference to match and no test beyond a screenshot.

**Which API:** CUDA is out (NVIDIA only; the DGX Sparks are servers). D3D12/Vulkan/Metal compute cover every
vendor at the cost of two or three backends (Jolt's plan). WebGPU through wgpu-native or Dawn is one C API for all
three plus the browser, but WGSL permits reassociation and fusion, and both runtimes are large foreign-language
dependencies. **sokol compute** is vendored, works on D3D11 today, compiles from one GLSL source, and cannot read
back, which is exactly the constraint set of the cosmetic layer and nothing else. For a deterministic-path kernel
the only route with a chance is a small hand-written abstraction over D3D12 and Vulkan with `precise` /
`NoContraction` and integer accumulators, which is a milestone of its own.

## 7. GPU verdict per multiplayer model (input from T1)

- **Lockstep:** only work that is bit-exact across machines may touch simulation state. No surveyed engine
  demonstrates that class on a GPU; WGSL forbids nothing, drivers JIT the kernels, and same-device is the best
  claim anywhere (three-avbd, PhysX). So: nothing on the deterministic path; the cosmetic layer is fine because it
  reads state and never writes it, and it must not consume the simulation's random stream or affect event order.
- **Host-authoritative state sync:** the host may use a GPU for anything. The gains available are still only the
  cosmetic layer (client-local in both models) and, for a host with a discrete GPU, a solver that wins past 10-20k
  awake bodies, which our tiers deliberately keep us under. A host-only GPU stress solve is possible but makes the
  host's outcomes unreproducible on the CPU (tests, replays, desync repair, H5's serializable world), for a gain the
  perf log's SoA plan can get on the CPU. Recommend against until that plan is exhausted.
- **Either model:** the cosmetic layer, sized per machine, behind the toaster profile switch.

## 8. Verdict on H4

Confirmed. GPU physics stays off the deterministic path (section 3: no cross-GPU class exists in practice, and the
driver is part of the binary), and it is a net loss on an iGPU (section 6: launch latency, no FP64 on Xe-LP, shared
bandwidth, 0.44-1.6 TFLOPS against 2-4 SSE cores at 1500 awake bodies). The cosmetic layer is the candidate and is
worth building. The async stress solve is *not* a candidate (FP64, reductions, reproducibility): that half of H4's
"perhaps" is falsified.

## 9. What a CUDA / D3D / CPU kernel-agreement experiment would need to show

To move any GPU work onto the deterministic path, an experiment on the machines at hand (RTX 3060 + Intel UHD on
the laptop, a DGX Spark GB10 for CUDA, the MacBook for Metal, a Steam Deck or any AMD part) would have to show:

1. **Bit agreement across vendors and drivers** on three kernels compiled from one source: Box3D's soft-step contact
   solve for one colour (the wide AoSoA path), one block-Jacobi CG iteration with its two dot products, and a
   pair-key radix sort with compaction. Compiled for CPU (MSVC, clang, gcc; SSE2 add(mul)), CUDA (`--fmad=false`,
   no fast intrinsics), and D3D12 and Vulkan (HLSL `precise` / SPIR-V `NoContraction`, DXC), on NVIDIA, AMD, Intel
   and Apple with at least two driver versions each, over a corpus of millions of inputs including denormals, signed
   zeros and near-ties, with every reduction as a fixed-shape tree or an integer fixed-point accumulator. One
   mismatch on one driver ends the deterministic-path question.
2. **The round trip under load:** a 1500-awake-body step with per-tick readback of transforms, contacts and events,
   pipelined one tick behind, while the sandbox renders the town scene at 60 Hz on the UHD iGPU and on the 3060;
   the wall time must be below the CPU's 0.5-1 ms.
3. **Replay stability across a driver update:** identical per-tick hashes on the barrage rung for 10k ticks before
   and after updating the GPU driver on the same machine.
4. **Headless reproduction:** an agent builds, runs and tests the path without a GPU, meaning the CPU reference in
   (1) is bit-identical and stays that way under `tools/bench.ps1`.

For the cosmetic layer the bar is different and low: 50k pieces settling on the town's rubble at under 1 ms of GPU
time on the UHD iGPU while the scene renders, with no CPU cost beyond the spawn uploads, and a screenshot that looks
like more mess than today.

## Sources

Repo (file:line): `extern/box3d/include/box3d/constants.h:44`; `extern/box3d/src/constraint_graph.h:22,26`;
`extern/box3d/src/constraint_graph.c:147,216`; `extern/box3d/src/solver.c:28-29,1240-1349`;
`extern/box3d/src/contact_solver.c:2273-2307`; `extern/box3d/src/broad_phase.c:710-712`; `src/step.c:30-79,
210-226, 292-296, 332-341`; `docs/perf-log.md` (2026-09-27 baseline and tiers, backlog, stress steps 0-5);
`docs/architecture.md` (debris tiers, stress, rendering); `extern/sokol/sokol_gfx.h` (compute passes and storage
buffer views present).

- yacineMTB, CUDA Box3D tweet, 2026-07-06: https://x.com/yacineMTB/status/2074070149737406841 (text via
  cdn.syndication.twimg.com); the 2026-07-01 "cudafied" tweet: https://x.com/yacineMTB/status/2072348861935267992;
  secondary coverage (200k bodies, no writeup, unreleased): https://daily.dev/posts/someone-built-a-200k-rigidbody-cuda-simulator-and-the-microduck-walks-zxlgb1tc9
- Erin Catto, Announcing Box3D (2026-06-30): https://box2d.org/posts/2026/06/announcing-box3d/; SIMD for Collision
  (2026-07-18): https://box2d.org/posts/2026/07/simd-for-collision/; Determinism (2024-08-27):
  https://box2d.org/posts/2024/08/determinism/; Solver2D (2024-02-05): https://box2d.org/posts/2024/02/solver2d/;
  Box3D README: https://github.com/erincatto/box3d; Box2D FAQ: https://box2d.org/documentation/md_faq.html;
  SE Radio 739 (2026-09-23, unverified content): https://se-radio.net/2026/09/se-radio-739-erin-catto-on-video-game-physics-engines/
- Box3D wasm benchmarks (2026-09-17): https://github.com/Pryme8/babylon-box3d
- PhysX 5.6 GPU rigid bodies: https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/GPURigidBodies.html;
  determinism: https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/RigidBodyDynamics.html; Direct GPU API:
  https://nvidia-omniverse.github.io/PhysX/physx/5.6.0/docs/DirectGPUAPI.html; GPU source release (2025-03-25):
  https://github.com/NVIDIA-Omniverse/PhysX/discussions/384 and
  https://www.cgchannel.com/2025/04/nvidia-open-sources-physxs-gpu-simulation-code/
- Isaac Gym paper (2021): https://arxiv.org/abs/2108.10470; Isaac Gym release notes (GPU-pipeline non-determinism
  fixes): https://docs.robotsfan.com/isaacgym/release-notes.html; Isaac Lab Newton integration:
  https://isaac-sim.github.io/IsaacLab/main/source/experimental-features/newton-physics-integration/index.html
- Bullet 3 OpenCL: README https://github.com/bulletphysics/bullet3/blob/master/README.md; AMD 2013 slides
  https://www.slideshare.net/slideshow/gs4150-erwincoumans/28535062; GDC 2013 slides
  https://slideshare.net/ecoumans/gpu-rigid-body-simulation-gdc-2013; course notes (PDF, not text-extractable
  here) https://www.multithreadingandvfx.org/course_notes/GPU_rigidbody_using_OpenCL.pdf; "100 thousand rigid
  bodies" claim: https://en.wikipedia.org/wiki/Bullet_(software)
- Mass splitting (Tonge, Benevolenski, Voroshilov, SIGGRAPH 2012): https://dl.acm.org/doi/abs/10.1145/2185520.2185601
- XPBD (2016): https://matthias-research.github.io/pages/publications/XPBD.pdf; small steps / TGS (2019):
  https://mmacklin.com/smallsteps.pdf
- VBD (SIGGRAPH 2024): https://arxiv.org/html/2403.06321v3; project page
  https://graphics.cs.utah.edu/research/projects/vbd/vbd-siggraph2024.pdf
- AVBD (SIGGRAPH 2025): project page https://graphics.cs.utah.edu/research/projects/avbd/; paper
  https://graphics.cs.utah.edu/research/projects/avbd/Augmented_VBD-SIGGRAPH25.pdf (numbers taken from the
  three-avbd comparison table); ACM https://dl.acm.org/doi/10.1145/3731195; authors' demo
  https://github.com/savant117/avbd-demo2d; three-avbd (WebGPU; 4090 and M4 Max table; same-device determinism)
  https://github.com/JackZhouSz/three-avbd; WebPhysics (2026-04-14)
  https://www.webgpu.com/showcase/webphysics-webgpu-avbd-solver/
- Roblox physics replication rollout (June 2026, networking not solver):
  https://devforum.roblox.com/t/upcoming-improvements-to-physics-replication/4675512
- MuJoCo MJX docs (numbers, hardware): https://mujoco.readthedocs.io/en/stable/mjx.html; MuJoCo Warp:
  https://github.com/google-deepmind/mujoco_warp; Newton: https://github.com/newton-physics/newton; Linux
  Foundation announcement: https://www.linuxjournal.com/content/linux-foundation-welcomes-newton-next-open-physics-engine-robotics
- NVIDIA Warp deterministic mode (1.15.0, 2026-07-07): https://github.com/NVIDIA/warp/blob/main/CHANGELOG.md
- Genesis: https://github.com/Genesis-Embodied-AI/Genesis; the corrected benchmark:
  https://stoneztao.substack.com/p/the-new-hyped-genesis-simulator-is,
  https://github.com/google-deepmind/mujoco/discussions/2303,
  https://github.com/zhouxian/genesis-speed-benchmark/blob/main/genesis_corrected_franka_benchmark.py
- Brax: https://github.com/google/brax
- Dimforge 2025 review and 2026 goals (2026-01-09): https://dimforge.com/blog/2026/01/09/the-year-2025-in-dimforge/;
  wgrapier3d: https://docs.rs/wgrapier3d/latest/wgrapier3d/; wgsparkl: https://github.com/dimforge/wgsparkl
- Jolt GPU discussion: https://github.com/jrouwe/JoltPhysics/discussions/501; v5.3.0 cross-platform determinism
  test: https://x.com/jrouwe/status/1901025550983946259
- Unreal Chaos (CPU): https://www.unrealengine.com/en-US/tech-blog/chaos-scene-queries-and-rigid-body-engine-in-ue5;
  Intel CPU optimisation paper: https://www.intel.com/content/dam/develop/external/us/en/documents/unreal-engines-new-chaos-physics-system-screams-with-in-depth-intel-cpu-optimizations.pdf
- Unity Physics design: https://docs.unity3d.com/Packages/com.unity.physics@1.3/manual/design.html
- Float atomics and reproducibility: NVIDIA CCCL determinism blog
  https://developer.nvidia.com/blog/controlling-floating-point-determinism-in-nvidia-cccl/; reproducible FP atomic
  addition https://ieeexplore.ieee.org/document/7321514; Deterministic Atomic Buffering (MICRO 2020)
  https://microarch.org/micro53/papers/738300a981.pdf; reproducible summation (Demmel, Nguyen et al.)
  https://dl.acm.org/doi/fullHtml/10.1145/3389360; fixed-point order independence in MPM (2026)
  https://arxiv.org/abs/2609.34666
- WGSL reassociation and fusion: https://www.w3.org/TR/WGSL/ (section 15.7.5);
  https://github.com/gpuweb/gpuweb/issues/2402; nvcc `--fmad`:
  https://docs.nvidia.com/cuda/cuda-compiler-driver-nvcc/index.html; HLSL `precise`:
  https://learn.microsoft.com/en-us/windows/win32/direct3dhlsl/dx-graphics-hlsl-variable-syntax; SPIR-V
  `NoContraction`: https://registry.khronos.org/SPIR-V/specs/unified1/SPIRV.html
- Launch latency (Iris Xe / Arc mobile 35-40 us; RTX 4090 and Radeon 890M about 5 us; OpenCL): Dalek,
  https://arxiv.org/abs/2508.10481; host-device copy overhead about 10 us:
  https://eunomia.dev/others/cuda-tutorial/13-low-latency-gpu-packet-processing/ and
  https://forums.developer.nvidia.com/t/pci-express-latency-and-how-to-decrease-it/20629; D3D11 staging readback:
  https://learn.microsoft.com/en-us/windows/win32/api/d3d11/nf-d3d11-id3d11devicecontext-map
- Intel zero-copy on iGPUs: https://www.intel.com/content/www/us/en/developer/articles/training/getting-the-most-from-opencl-12-how-to-increase-performance-by-minimizing-buffer-copies-on-intel-processor-graphics.html;
  Xe-LP without FP64: https://en.wikipedia.org/wiki/Intel_Xe; UHD 620 at 422-442 GFLOPS:
  https://en.wikipedia.org/wiki/Intel_Graphics_Technology; Steam Deck (8 RDNA2 CUs, 1.6 TFLOPS, LPDDR5 5500 quad
  32-bit): https://www.steamdeck.com/en/tech; Iris Xe 96 EU at 1.7-2.1 TFLOPS:
  https://www.pcgamer.com/steam-deck-performance-expectations/
- sokol compute (2025-03-03): https://floooh.github.io/2025/03/03/sokol-gfx-compute-update.html; milestone 2
  (2025-05-19): https://floooh.github.io/2025/05/19/sokol-gfx-compute-ms2.html; resource views (2025-08-17):
  https://floooh.github.io/2025/08/17/sokol-gfx-view-update.html; Vulkan backend (2025-12-01):
  https://floooh.github.io/2025/12/01/sokol-vulkan-backend-1.html; changelog (no readback; read-buffer planned):
  https://github.com/floooh/sokol/blob/master/CHANGELOG.md
- Historical: GPU Gems 3 ch. 29, particle-based rigid bodies on GPUs (2007):
  https://developer.nvidia.com/gpugems/gpugems3/part-v-physics-simulation/chapter-29-real-time-rigid-body-simulation-gpus

Unverified items, and what would verify them: yacine's 200k bodies and hardware (his demo video or code); Roblox's
production solver (a Roblox engineering post); PhysX's GPU solver type (a `PxSolverType` check in the 5.6 headers);
the SE Radio 739 content (listening to it); the iGPU crossover estimates in sections 5 and 6 (the experiment in
section 9); Bullet 3's course-notes numbers (a text extraction of the PDF; no PDF tools on this machine).
