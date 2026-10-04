# Milestone 7: a deterministic, GPU-aware, multiplayer-first engine (research report)

Written 2026-09-30. This closes the research of milestone 7 ("Deep research: destruction engine architecture",
[roadmap.md](roadmap.md)). It answers the owner's question: should we rewrite Box3D, whole, in part or not at all, so
that the physics is deterministic across machines, plays well with GPUs, and makes co-op as seamless as possible?
It is judged against the north star in [goals.md](goals.md). The detail, sources and measurements live in
[research/](research/), one report per track; this file is the decision record. Read the tracks only when a decision
needs their evidence. Milestone 11 (§10, 2026-10-04) replaced the model's prediction (the netcode inversion: the world
in delayed lockstep, each player's own bubble predicted by the core) and reopened the arithmetic on evidence.

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
| Unproven | Pascal (the GTX 1060 floor), AMD, Apple GPUs, Blackwell, FEX, WASM. Replays survived one NVIDIA driver update (581.95 to 610.60: every hash identical, [m7-gpu-experiments.md](research/m7-gpu-experiments.md) addendum); other vendors and future branches are open |

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

## 5. The roadmap that was chosen

The owner chose on 2026-09-30 to commit to the integer GPU core now, rather than gate it on the floor GPU's result, and
to hold the first co-op test early, on Box3D. The pinned order is in [roadmap.md](roadmap.md):

| # | milestone |
|---|---|
| 7 | this research, closed by **determinism hardening** (L0) |
| 8 | **the physics seam:** every Box3D call behind one interface; the one-tick-lag experiment |
| 9 | the outcome catalogue (the new core's contract), the engine surface, a seams-first review |
| 10 | **commands and hashes (L1a)** and the first two-process co-op, on Box3D |
| 11 | integer groundwork: the integer toy, exact fracture geometry (L2), the floor GPU measured (replanned in §10: the arithmetic spike, then 11a exact geometry) |
| 12 | **the integer core with its CPU twin (L3)**, beside Box3D |
| 13 | **the core on the GPU (L4)**; Box3D deleted if the gate passes |
| 14 | **the snapshot (L1b)**, written once, on the integer core |
| 15 | the character controller, as the feel test decides (in §10: the bubble) |
| 16 | networking |
| 17 | profiling and a performance review |
| 18 | large-map zones and persistence |
| 19 | creatures, part 2 (informed by the adaptive-locomotion deep dive) |
| 20 | dents |
| 21 | art polish and demo views |

The simplicity review's outcome catalogue comes before the core rewrite because it is the only way to gate a rewrite
that changes every number. The snapshot waits for the integer core so that it is written once, on plain integers,
instead of over Box3D's internal state first.

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

## 9. Ideas after the research (2026-10-01)

Three ideas from the owner, now in the [roadmap](roadmap.md):
- **Repair by causal unit** (milestone 10): replace a mismatched object's whole island, structure or assembly, plus
  every unit it touched since the last agreed tick, hidden state included, and re-simulate only those from the host's
  image. It answers the red team's R3 (repairs that re-diverge on unshipped hidden state), and it makes partial
  rollback exact: a causally closed region has nothing outside it to leak into.
- **A finite speed of propagation** on the channels that reach across space within a tick (milestone 10): the stress
  solve, supply and queries. Between islands, cause and effect already travels only as fast as matter, so this bounds
  every cone: repair regions, rollback regions and zones that may lag (milestone 18). Collapses become cascades
  spread over ticks.
- **Causally scoped rollback** (milestone 16): predict the other players' inputs and, on a wrong guess, re-simulate
  only that input's cone, so a player's own actions on the world feel instant at a cost bounded by the cone, not the
  world. It rests on the lockstep base, which lets every machine compute the truth from confirmed inputs.

## 10. Milestone 11 decisions (2026-10-04)

The owner relitigated two of milestone 7's choices with two outside reviewers (GPT-6 Astra and Fable, independent
memos) and Claude. The decisions belong to the owner and Claude.

### The netcode inversion (adopted)

**The world runs in delayed lockstep; each player's own bubble is predicted.** Every confirmed tick is applied only
when every input for it has arrived, as today. On top of that, each machine steps its own player's *bubble* ahead of
the confirmed world by the input delay d, with that player's own pending inputs, which are known. The bubble is the
avatar (the real rig, not a capsule), what it holds, and the vehicle it drives, plus every unit whose swept bounds over
d ticks reach them.

- **It supersedes §1's latency state and §9's causally scoped rollback.** The latency state predicted a kinematic
  mover rebased on the canonical body, which is wrong every tick for a stumbling rig (red team R6). Scoped rollback
  guessed *other* players' inputs and re-simulated the cone of a wrong guess. The inversion's only guess is "nothing
  outside my bubble reaches it within d ticks", so a misprediction is a boundary crossing, never a wrong input. It is
  strictly less speculative than scoped rollback and strictly more faithful than the latency state. Scoped rollback
  may return later as a layer on top of it.
- **Exact when nothing crosses the boundary.** The bubble is stepped by the core itself (the CPU twin, even when the
  world runs on the GPU), in the unit-isolated mode milestone 12 requires. With nothing crossing, the prediction is
  bit-identical to what the confirmed world will compute. Reconciliation per confirmed tick compares the bubble's unit
  hashes with the confirmed world's: equal means nothing to do; different means restore the bubble from the confirmed
  image, hidden state included (the lab's E3: state-only copies cannot catch up), and re-step it d ticks. A rig and a
  car are tens of bodies, so this costs microseconds on the twin.
- **Vehicles join the bubble from day one** (§1 had them take the input delay). Driving with 180 ms of steering
  delay is bad (lateral error grows from 130 ms, Frontiers in VR 2021); Factorio hides 30 ticks of driving this way
  (FFF-412).
- **What needs care.** The rest of the world is d ticks stale in prediction: a moving thing is off by v×d (3 m for
  debris at 20 m/s and 10 ticks). Structures under stress are never in the bubble (they are static to it).
  Avatar-avatar contact is softened in prediction. Corrections are eased by the renderer (cosmetic, so free to differ
  per machine). The bubble is capped by body count (a count, never a time) and falls back to the confirmed world
  over the cap. A blast from outside reaches the avatar as a late ragdoll.
- **Others' actions are previewed outside lockstep.** Other players' shots, swings and effects are sent unreliably
  on a separate channel as cosmetic previews, each with an id, so the confirmed event does not duplicate its effects.
  Hitscan is resolved on the shooter's machine and enters as a tick-stamped command.
- **Adaptive delay** stays (milestone 16), driven by measured arrival jitter and deadline misses, raised at once and
  lowered a tick at a time, with explicit rules for a tick whose inputs arrive late (a host-finalised repeat that
  every peer applies) and for merging inputs when the delay changes (the last wins).
- **Determinism requirements are unchanged across machines.** Within a machine the core gains a contract: a unit
  stepped alone gives the same bits as that unit stepped inside the world; a unit's image carries its hidden state;
  prediction emits only commands, never state, into the shared history.
- **The exact bubble experiment needs a world clone,** which Box3D does not have. It moves to milestone 15, on the
  snapshot (milestone 14). The owner's quick check, which needs no code: play co-op at 12 ticks of input delay with no
  prediction (`python tools/coop.py launch --delay 12 --running --allow-input`) to feel what the bubble must hide.

### The arithmetic (decided on evidence by milestone 11)

**Catto's rules are CPU rules.** His Box2D v3 post lists three things to avoid: fast math, FMA contraction and
C-library trigonometry (sqrtf is fine). This engine's 14-leg CI proves them sufficient on CPUs, with our own additions
(a control-word guard, the static C runtime, total orders, no side effects in one call's arguments). On GPUs two carry
over (fast math and contraction, as `NoContraction`) and the third expands to "own everything except add, subtract
and multiply":
- sqrt and inversesqrt are 1 ulp off on every NVIDIA GPU measured, in every mode; Vulkan requires only 2.5 ulp for
  division and 2 for inversesqrt;
- there is no control word the program owns: drivers flush fp32 subnormals by default; NVIDIA keeps them only through
  an undocumented side effect of `RoundingModeRTE`, which crashes the GB10's compiler on a real kernel;
- the compiler lives in the driver: selects are rewritten into min/max (±0 and NaN change), llvmpipe folds `0 - a` into
  `-a`, Slang folds `x + 0`;
- NaN bits differ by machine.

**Two candidates, both bit-exact.** The float dialect F (fp32, NoContraction, only add, subtract and multiply on the
GPU, software reciprocals, every select written out and snapped, no float-control modes) and block-scaled int32 V4
both matched their CPU twins on four GPUs (RTX 3060, Intel UHD 630, GB10, llvmpipe) and four twin compilers on two
ISAs ([research/m11-gpu-lab.md](research/m11-gpu-lab.md)): E11's single solve (except E11's one signed zero in F on
the UHD at 100k contacts, a select rewritten into min/max, which F's snap now absorbs) and then the toy's whole tick,
every word of it, for thousands of ticks. V4 costs 1.25 to 1.36 times F for E11's solve; for the toy's whole tick
1.7 times at one pile (dispatch-bound) and the same as F per body at 64. The asymmetry between
them: an integer driver bug is deterministic, local and loud (a startup battery catches it, and it can be reported);
a float driver that returns a legal but different result fails late, remotely and silently. Astra leaned float, Fable
and Claude integers.

**The rule.** Zero unexplained bit differences first, then acceptable outcomes on the catalogue's terms, then the
whole pipeline's cost; prefer integers if both pass while float still depends on behaviour the driver does not
promise; prefer float if V4's outcomes fail (stacks creep, piles do not sleep) and no narrowing fixes them. Milestone
11's toy supplies the evidence: decision point 1 (outcomes on the F, V4 and double twins against Box3D) and decision
point 2 (bit-exact on every device in both dialects). The Pascal rows (no full-rate int32 multiply) and AMD close the
gate before milestone 12. The verdict is recorded here when the toy has answered.

**What the toy answered (2026-10-04, overnight):**
- *Bits:* both dialects pass decision point 2 on every device to hand, with no driver mode the spec does not promise.
- *Outcomes:* both pass DESIGN.md's acceptance table on outcomes. Under its rules as written both miss the 120-tick
  position rms against the double reference on impact scenes (F three rows, V4 four), and D and Box3D miss one bar
  each; nothing separates the dialects on physics.
- *Tools:* each dialect met one bug that only it could meet: gcc 13 miscompiled a float comparison (F's twin), and
  V4's matrix inversion overflowed int32 on near-singular inputs (UBSan). Both were loud: the cross-compiler twins
  and the sanitizer caught them the first time they ran.
- *Cost:* V4 is dearer where the tick is dispatch-bound (1.7 times at one pile) and level at scale.

**Claude's recommendation, for the owner to confirm: block-scaled integers (V4)**, provisional on the Pascal and AMD
gate. The deciding difference is the one the rule names: F is bit-exact because no subnormal ever arises in a step
(snapped at every select and store, watched by the x64 twin's sentinel), but nothing a driver promises keeps an
intermediate inside an expression from going subnormal in content nobody tested, and NVIDIA flushes them by
default. That failure would be silent and remote. V4's arithmetic is exact by the spec; its risks (saturation,
formats, a joint block's condition number) are counted where they happen. The strongest case against: F met every
bar V4 met, keeps Box3D's formulas and float's culture, and is cheaper on small scenes; if the Pascal rows exceed the
gate, F (or dropping Pascal, Fable's rule) is the answer, and milestone 12's core is built so the dialect is a
compile-time choice for as long as that stays cheap.
