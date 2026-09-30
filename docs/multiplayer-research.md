# Milestone 7: a deterministic, GPU-aware, multiplayer-first engine (research report)

Written 2026-09-30. This closes the research of milestone 7 ("Deep research: destruction engine architecture",
[roadmap.md](roadmap.md)). It answers the owner's question: should we rewrite Box3D, whole, in part or not at all, so
that the physics is deterministic across machines, plays well with GPUs, and makes co-op as seamless as possible?
It is judged against the north star in [goals.md](goals.md). The detail, sources and measurements live in
[research/](research/), one report per track; this file is the decision record. Read the tracks only when a decision
needs their evidence.

| track | report | what it holds |
|---|---|---|
| T0 | [m7-state-audit.md](research/m7-state-audit.md) | our state inventory, measured snapshot sizes and hash costs, where decisions originate, 24 hardening items |
| T1 | [m7-model.md](research/m7-model.md) | the multiplayer models, compared with numbers |
| T2 | [m7-case-studies.md](research/m7-case-studies.md) | 45 shipped games: how they synced physics and destruction; 55 strategies |
| T3 | [m7-float.md](research/m7-float.md) | float determinism across compilers, CPUs, emulators, WASM and GPU APIs; the dialect |
| T4 | [m7-fixed-point.md](research/m7-fixed-point.md) | fixed-point formats and costs, fixed3d, exact integer fracture geometry |
| T5 | [m7-gpu.md](research/m7-gpu.md) | GPU physics: the survey, the round trip, the cosmetic layer |
| T6 | [m7-netcode.md](research/m7-netcode.md) | 50 netcode techniques with numbers; bandwidth worked examples |
| T8 | [m7-experiments.md](research/m7-experiments.md) | measured: compilers, SIMD paths, flush-to-zero, a 14-leg CI across OSes, CPUs and emulators |
| T10 | [m7-gpu-integer.md](research/m7-gpu-integer.md) | the design of an integer-first deterministic GPU core |
| E11 | [m7-gpu-experiments.md](research/m7-gpu-experiments.md) | measured: one contact solve on an RTX 3060, an Intel UHD and a CPU twin, float and integer |
| T7 | [m7-synthesis.md](research/m7-synthesis.md) | the synthesis this report condenses: the model, the ladder, the roadmap |
| T9 | [m7-redteam.md](research/m7-redteam.md) | the red team's objections to the synthesis |

## The answer in one screen

- **Floating point was never the enemy.** IEEE arithmetic is deterministic given the same operations in the same order.
  A 14-leg CI run of this engine is byte-identical per tick on every bench rung across Windows, Linux and macOS, x64 and
  ARM64, five compilers (not pinned) and three x64 emulators on ARM64 (Rosetta 2, Prism, box64), once three hazards of
  ours are fixed: gcc's default FMA contraction, C-library transcendentals (`cbrtf`, `sinf`, `atan2f`), and random draws
  inside one call's argument list. Box3D needed no patch.
- **Model: convergent lockstep.** Every machine runs the whole simulation from one command log; the host relays inputs
  and keeps the clock; the local character is predicted outside the deterministic world; per-object hashes localise a
  divergence and the host repairs that object. The solo requirement already puts the whole simulation on the minimum
  spec, and the north star's body counts make streaming body state impossible from a player host, while lockstep's
  bandwidth does not grow with debris.
- **The one primitive every model needs:** serialise the world and rebuild it bit-exactly, plus a tick-stamped command
  log and per-object hashes that include the stress solver's state. It serves saves, late join, host migration, desync
  repair, zones and persistence, replays, and "every bug a replay".
- **The GPU can be inside the simulation, in integers.** One Box3D-style contact solve was bit-exact on an NVIDIA GPU, an
  Intel GPU and a CPU twin in block-scaled 32-bit fixed point, at 1.16 to 1.35 times the float solve's cost. Float on
  GPUs works only in a narrow, per-driver dialect; 64-bit fixed point was miscompiled by the Intel driver. Exact integer
  semantics are necessary but not sufficient: a startup self-test and a bit-identical CPU twin complete the guarantee.
- **Do not rewrite Box3D for determinism; replace it only for the GPU, and only if the GPU pays.** The ladder:
  - harden (L0);
  - build the command log and hashes (L1a) with exact integer fracture geometry beside them (L2);
  - add the snapshot (L1b);
  - then an integer rigid-body core with a CPU twin (L3), and that core on the GPU (L4). Box3D is deleted only if L4 meets its gate on a GTX 1060-class
  card.
- **The experiments to run now,** because they can change the plan within days (§7):
  - E11's unchanged binaries on a used GTX 1060 (the floor's Pascal GPU has no full-rate 32-bit integer multiply) and
    on any AMD GPU;
  - a one-tick physics lag on today's engine;
  - a feel test of a delayed first-person body.
- **The red team** found three major objections and no fatal one (§7). The plan above includes its amendments.

## 1. The multiplayer model: convergent lockstep

Chosen in [m7-model.md](research/m7-model.md) (§6, "g3") and confirmed by the synthesis (§1). What it is:

- **Clock and inputs.** The host is the clock and relays each tick's inputs as one packet (Factorio's scheme). Input
  delay is 2 to 3 ticks plus a jitter buffer. A peer whose input is late past a tolerance has its last input repeated
  and loses control briefly instead of stalling everyone (Photon Quantum's "hard tolerance").
- **The local character** is Factorio's "latency state": the canonical body lives in the deterministic world; the local
  view re-applies pending inputs to a kinematic mover each frame. Its effects (tools, pulls, grabs) enter as
  tick-stamped commands; tools are resolved on the shooter's machine. Vehicles take the input delay (a predicted
  vehicle is a later bolt-on); cranes and rigs are never predicted.
- **Hashes and repair.** An incremental per-object hash (today's `lpWorld_Hash` costs a whole step at the peaks; the
  replacement is estimated at 0.1 to 0.2 ms) that must include the stress solver's state: flush-to-zero changed the
  solver while the world hash stayed equal for 600 ticks ([m7-experiments.md](research/m7-experiments.md), E9). A
  mismatch names an object and a tick; the host ships that object's image, applied at a tick boundary on every peer.
- **Late join by snapshot, not by replaying the log.** Measured: 3.7 MB (town), 4.1 (keep), 7.3 (barrage) and 8.6 MB
  (siege) compressed at each rung's worst tick, nothing at load. Replaying the log costs 7 to 32 s of CPU per minute of
  play. Pause-on-join is the fallback for slow joiners.
- **Session-wide settings.** Every count budget, dt and substeps, the build, the content and the self-test hashes are
  agreed in the handshake. Quality is per session, bounded by the weakest peer; worker counts may differ freely.
- **Host migration is free:** every peer holds the whole state.

Why not what shipped games did: no real-time 3D destruction game has shipped full lockstep
([m7-case-studies.md](research/m7-case-studies.md)); Teardown, Rainbow Six Siege and Deep Rock Galactic send
destruction as ordered events and sync bodies as prioritised state, and pay in snapping. Their reasons were weak
clients, the slowest peer, late join without a snapshot, and a desync ending the session. Under the owner's decisions
each is answered: everyone can play solo (so no thin clients), budgets are session-wide and the clock tolerates a slow
peer, the snapshot is measured, and repair turns a desync into one object snapping. The number that decides it: at
5,000 awake bodies, state sync costs about 9.6 Mbit/s per client (106 Mbit/s of upload for a 12-player host), or about
2 Hz per body under Teardown's 1 Mbit/s cap; lockstep's host upload is 0.1 to 0.9 Mbit/s at any body count
([m7-netcode.md](research/m7-netcode.md) §12).

**Second choice:** the Teardown hybrid with the host's cell geometry shipped as commands (no client needs any
determinism). It flips only if the union of active zones for 8 to 12 spread-out players exceeds the floor while one
player's zones fit, if repairs never settle in the field, or if a bit-exactness failure appears that CI cannot
reproduce.

## 2. Determinism: verdicts

| scope | verdict |
|---|---|
| CPU across compilers, OSes, ISAs | **Confirmed** (T8): byte-identical per tick, compilers unpinned, once gcc gets `-ffp-contract=off`, libm leaves simulation and scene code, and random draws leave argument lists |
| x64 under emulation on ARM64 | **Confirmed** for Rosetta 2, Prism and box64 (with `FASTNAN=0 FASTROUND=0`); FEX untested. Prism and box64 route C-library calls to native ARM64 code, so the simulation must be libm-free or link the C runtime statically |
| Flush-to-zero set by other code | **A real hazard** (T8 E9): it changed the stress solver's results. A control-word guard at step entry and in every worker is required |
| WASM | Open, expected to hold (T3); low priority |
| GPU, float | **Conditional, per driver** (E11): add, sub and mul only, no contraction, no GPU sqrt or division (sqrt is 1 ulp off on both GPUs in every mode), per-vendor float controls (NVIDIA's RTE mode silently enables subnormals and correct division), no reliance on the sign of zero. D3D11 is out |
| GPU, integer | **Confirmed on two vendors** (E11) for block-scaled 32-bit fixed point with 64-bit products; Q32.32 in 64-bit integers was miscompiled by the Intel driver. A driver bug and a shift-by-64 case still split vendors, so a startup self-test and the CPU twin are part of the guarantee |
| Unproven | Pascal (the GTX 1060 floor), AMD, Apple GPUs, Blackwell, replay stability across a driver update, FEX, WASM |

## 3. The GPU, in brief

- **Why integers.** Integer results are specified by the language down to the bit, so bit-exactness across vendors
  becomes something we can prove (overflow freedom with Frama-C or CBMC, lint rules, sanitizers) and test on one
  machine, rather than something only a lab of GPUs and driver versions could establish. Integer atomics are exact and
  order-free, so parallel accumulation (per-body impulse totals for the stress loads) needs no colouring tricks.
- **The format** ([m7-gpu-integer.md](research/m7-gpu-integer.md)): 32-bit mantissas with 64-bit products; 64-bit world
  positions; body-relative anchors; per-body exponents for mass and inertia fixed at creation; each cross-body product
  one multiply and one precomputed shift. No 128-bit arithmetic in the solver. E11 found the angular-velocity range too
  narrow (fixed by widening it) and town rows about nine times less accurate than float, which the outcome catalogue
  must judge.
- **Measured cost** (E11, 100k contacts per step): RTX 3060 float 5.4 ms, integer 7.2 ms; Intel UHD 31 to 41 ms; the CPU
  twin 118 ms on one thread and 25 ms on eight. Below a few thousand contacts the GPU is dispatch-bound and slower than
  Box3D on the CPU, but since the twin is bit-identical, each machine may pick the GPU or the CPU per tick with no
  effect on the session.
- **The floor.** Pascal has no full-rate 32-bit integer multiply (a signed 32×32→64 product is 7 instructions); the
  prediction is about 5 times float's issue cost per contact row and 1.5 to 2.5 ms of GPU time for 5,000 awake bodies on
  a GTX 1060. Unmeasured: the first thing to buy.
- **What stays on the CPU, in float:** stress (double), links, wheels, rigs, gait, supply. Fracture moves to exact integer
  plane geometry (L2), which also removes today's tolerance hacks.
- **Kernel language and API:** a C-and-HLSL shared subset compiled twice (the twin as plain C that agents read and proof
  tools check); Slang as a third, differential implementation (it produced correct twins on the first build); Vulkan
  first, D3D12 and Metal later.

## 4. The ladder

| rung | what | effort (agent-weeks) | unlocks | gate |
|---|---|---|---|---|
| **L0 hardening** | merge `ci-determinism`; portable maths only (one hash change); the order-independence and hash-coverage fixes from T0; the control-word guard at step entry, in workers and at API entry points; the self-test vector; float-to-int clamps; exact float recording; the NEON −0 and braced-initialiser fixes; new determinism rules, including one for game code; per-tick step times in the bench's hash log; the one-tick-lag flag experiment on Box3D; the 14-leg CI kept | 1 to 2 | every later rung; CI as a standing check; the lag and spike answers | CI green and byte-identical on every leg at the new hashes; the lag experiment's verdict recorded |
| **L1a commands and hashes** | a tick-stamped command queue in the core; stable ids; the incremental per-object hash with the solver state and Box3D's contact state; session settings; a two-world lockstep test harness that measures how repairs reconverge | 2 to 3 | desync detection and triage; the two-process smoke test; the repair question answered | the incremental hash equals the full one and costs under 0.3 ms at the barrage peak; the harness names an injected desync by object and tick |
| **L1b the snapshot** | serialise and rebuild (ours lean, Box3D's state in a portable encoding or restored in place), per-object repair, join; after the simplicity review | 2 to 3 | saves, late join, migration, repair, replays with seek, zone snapshots | round trip exact on every rung; a cross-OS join measured |
| **L2 exact fracture geometry** | integer sites and bisector planes, exact classification, hulls from exact topology | 2 to 4, beside L1 | fracture bit-identical by construction; no tolerance flips; required by L3 | recorded blasts replayed with no failures; spike within 2× today's |
| **L3 integer core, CPU twin** | the solver, bodies, hulls, broadphase, narrowphase, four joints, sleep and casts in block-scaled integers, beside Box3D and switchable per world; the one-tick pipeline lag introduced on the CPU | 11 to 15 | nothing alone (a CPU loss: 2.5 to 3× Box3D scalar, 1.3 to 1.6× with AVX2); it is L4's price | the Pascal and AMD row experiments first; the outcome catalogue passes with tolerances |
| **L4 the core on the GPU** | a Vulkan runtime, every kernel from L3's sources, differential tests, the self-test in the handshake, the twin as fallback; Box3D deleted at the end | 12 to 18 | the north star's GPU inside the simulation; tens of thousands of bodies | 5,000 awake bodies under 16.7 ms on a GTX 1060-class card; 20,000 under 8 ms of GPU time on the 3060 |
| Cosmetic GPU layer (optional) | upload-only debris, dust and mess | 2 to 3 | more mess for free | none |

L0 to L4 total 29 to 44 agent-weeks in T10's units, which the red team calls uncalibrated: milestones 1 to 6 took four
calendar days, and the binding cost is verification, not typing. L1a and L1b are worth having even if L3 never passes
its gate.

## 5. Proposed roadmap order (for the owner to choose)

| new | milestone |
|---|---|
| 7 | this research, closed by **L0 hardening** (widened with the lag and spike experiments); meanwhile, buy a GTX 1060 and run E11 on it and on any AMD GPU, and run the owner's feel test of a delayed first-person body |
| 8 | engine surface and diagnostics (as written) |
| 9 | **commands and hashes (L1a)**, with exact fracture geometry (L2) in parallel; ends in a two-process lockstep smoke test |
| 10 | the simplicity review, outcome catalogue first, scoped by the GTX 1060 result (to what survives the core swap if L3 is on) |
| 11 | **the snapshot (L1b)**, then, if the floor GPU pays, the integer CPU toy and **the integer core with a CPU twin (L3)** |
| 12 | **the core on the GPU (L4)**; Box3D deleted at its end if the gate passes |
| 13 | profiling and a performance review (of the architecture that will ship) |
| 14 | large-map zones and persistence (per-zone snapshots, bake and forget, wake storms) |
| 15 | networking (thin under lockstep; may move up to right after 11's snapshot if a game needs co-op first) |
| 16 | dents |
| 17 | art polish and demo views, with the cosmetic layer |

To place (R24): the character controller (after the feel test), versioning of the save format, field telemetry and
desync triage, terrain.

The simplicity review comes before the core rewrite because its outcome catalogue is the only way to gate a rewrite that
changes every number, and because the subsystems it would simplify are the ones that stay on the CPU.

## 6. Risks, and hardware to acquire

1. **The floor GPU's integer cost is unmeasured** (at 5× float the GTX 1060 fits; at 10× it does not): a used GTX 1060,
   bought now.
2. **The weakest GPU sets the session** under lockstep once physics is on the GPU (§7): measured through the twin's cost
   at the floor.
3. **The one-tick lag may break feedback loops** (the tyres, the servos): the flag experiment in L0.
4. **The player as a rig under prediction** (§7): the feel test.
5. **AMD is untested:** any RDNA card or a Steam Deck.
6. **Driver compilers are part of the binary:** contained by the startup self-test, the twin, differential tests on
   real GPUs in CI (a self-hosted runner) and a re-run after every driver update.
7. **The integer solver's accuracy and stacking** against float: the integer toy, then the outcome catalogue.
8. **The union of active zones at 8 to 12 spread-out players** may exceed the floor: measured in the zones milestone;
   the flip condition for the hybrid.
9. **The CPU side at 20,000 bodies** (freezing, wakes, tiers, the hash, rendering sync): a synthetic 20k-body bench rung
   in the profiling milestone.

Hardware, in order: a used GTX 1060, an AMD GPU or a Steam Deck, then time on the DGX Spark (ARM64 and Blackwell) and
the MacBook (Apple GPU) for E11's listed runs.

## 7. The red team, and what it changed

A fresh reviewer attacked the synthesis: 25 objections, each with a severity, a verdict and the cheapest experiment
that would settle it ([m7-redteam.md](research/m7-redteam.md)). None was fatal.

**What it could not break:**
- lockstep's bandwidth staying flat as body counts grow (it could only move the crossover point);
- the CPU determinism evidence;
- H5, with the solver state in the hash;
- block-scaled int32 at 1.16 to 1.35 times float;
- the float GPU verdict;
- gating Box3D's deletion on the GPU paying.

**The three objections most likely to change the plan:**

1. **The weakest GPU sets the session** (R1, R17).
   - After L4, a peer whose GPU fails the self-test, or is slow (an old AMD or Pascal card), runs the CPU twin at 2.5 to
     3 times Box3D's cost. The twin cannot hold the floor's 5,000 awake bodies at 60 Hz, so lockstep drops the whole
     session to that peer's counts, and solo play on such a machine is capped the same way. The hybrid runs at the
     host's capacity instead.
   - Accepted as lockstep's price. The handshake sets the session's counts from each peer's measured capacity, GPU or
     twin.
   - The hybrid stays the documented fallback: if weak peers are common, a per-peer follower path is where it would
     come back.
   - Measure first: the siege rung at 4 workers, scaled by the twin's cost, stands in for the floor CPU's twin.
2. **The one-tick pipeline lag is unmeasured** (R16).
   - The tyre solve and the rigs' servos are 60 Hz feedback loops with stiff contacts. Nobody has checked that they
     tolerate reading physics a tick late.
   - The experiment: buffer the post-step physics reads by one tick behind a flag on today's Box3D engine, then run the
     nine suites and the track and mech rungs. It goes in L0, before any integer code is written.
3. **The player is a rig, not a capsule** (R6).
   - The goals make the player a destructible body with pools and vitals. Predicting it with a kinematic mover means
     rebasing a capsule onto a stumbling torso every tick; the alternative is leaving it unpredicted at 50 to 100 ms of
     input delay.
   - The experiment: a 6-tick delay on walk and drive events in the sandbox, with the owner at the keyboard for a day.
   - It decides the character controller, L1's command set, and whether the first-person co-op game is feasible under
     lockstep.

**Other amendments adopted:**
- **Split L1.** The command log, the incremental hash and the two-world harness come first. The hash also covers
  Box3D's contact state (R13): today warm-start divergence is invisible until a transform moves. The serialiser waits
  until after the simplicity review, which reshapes the state it would serialise (R22).
- **Buy the GTX 1060 now,** not just before L3. Its result decides whether L3 and L4 exist, and therefore what the review
  may touch (R23). NVIDIA reportedly moved Pascal to a legacy driver branch (to verify): frozen compilers help replay
  stability, but their bugs won't be fixed.
- **Widen L0** (R25):
  - merge `ci-determinism` now;
  - add per-tick step times to the bench's hash log (R2: fracture spikes under lockstep have never been measured);
  - the lag flag (R16);
  - the control-word guard at the API entry points as well as the step (R12);
  - two latent hazards (R10): a NEON −0 patch, and random draws inside braced initialisers;
  - a determinism rule for game code in the private repo (R11): AI and scripts run on the host and emit commands, or
    follow the dialect under a hash CI.
- **Late join across operating systems** needs either a portable encoding of Box3D's hidden state or a synchronised
  canonical rebuild, because Box3D's image only works within one build (R4). Priced in L1.
- **Repair may be state sync in disguise** for structures under load (R3). The two-world harness measures
  reconvergence (perturb one warm start, count ticks) before anything relies on it.
- **An integer "toy" of about 1,000 lines on the CPU** (boxes, SAT, the block-scaled step, sleep) settles stacking and
  the narrowphase before L3 commits (R15).
- **The effort estimates are uncalibrated** (R19). The whole engine, milestones 1 to 6, landed in four calendar days.
  The binding cost is verification (the outcome catalogue, proofs, GPU CI machines), not typing, so L3's first stage
  gets a hard stop.
- **Missing from the roadmap** (R24):
  - a character controller;
  - versioning of the save format;
  - field telemetry and desync triage;
  - a measurement on a floor CPU;
  - terrain (the integer core has no meshes or height fields).
- **The hybrid's declined price is recorded** (R8): roughly 15 to 20 agent-weeks against 26 to 38. It raises no Pascal,
  AMD, twin or self-test questions, but costs bandwidth as body counts grow and a second decision path in every
  reactive system.

## 8. Hypotheses

| hypothesis | verdict |
|---|---|
| H1: float with pinned flags is bit-exact across compilers and CPUs; the hazards are ours | confirmed (T8) |
| H2: lockstep for 4 players, acceptable at 12; the character predicted outside the world | confirmed with amendments (server-timed clock, per-object repair) |
| H3: fixed point everywhere costs 2 to 4× and solves what flags solve | mixed: true on the CPU; false for block-scaled int32 on GPUs (1.16 to 1.35×); integers are justified by the GPU alone |
| H4: GPU physics stays off the deterministic path | partly falsified: an integer (and a narrow float) GPU solve is deterministic across two vendors; the iGPU loss holds |
| H5: a serialisable, command-logged, hashed world matters more than a solver rewrite | confirmed and sharpened |
| H6: x64 under Rosetta 2, Prism and box64 stays bit-exact | confirmed; FEX open |
