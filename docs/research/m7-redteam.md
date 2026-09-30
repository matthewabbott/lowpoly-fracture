# M7 T9: red team of the synthesis

Written 2026-09-30, without sight of the discussion that produced `m7-synthesis.md`. Read: the synthesis in full;
`docs/goals.md` and `CLAUDE.md`; every track report it cites (T0, T1, T2, T3, T4, T5, T6, T8, T10, E11), checking each
citation against its section; `bench/baseline.json`; `docs/roadmap.md`; `docs/determinism-rules.md`; `src/step.c`,
`include/lpf/lpf.h`, `bench/main.c` and the git log for calibration. Stance: adversarial, conceding where the evidence
holds. Yardstick: agent-friendliness first, performance second, determinism non-negotiable, and the games as goals.md
describes them (a rigging-race game, a co-op hauling game, first-person co-op in destructible buildings).

Two repo facts the synthesis does not use, and which several objections lean on:

- The whole first-party engine (23k lines, milestones 1 to 6) was built between 2026-09-26 and 2026-09-29. So
  "agent-weeks" is not a unit calibrated to this repo (see R19).
- `lpf_bench` reports only average, p95 and max step times; no per-tick series exists, so the "fracture spike under
  lockstep" question has never been measured as a catch-up debt (see R2).

## The objections

Severity: fatal (the plan as written cannot meet the goals), major (a decision or an estimate changes), minor (a gap to
close). Verdict after checking the evidence: stands (the synthesis is right), partly (right in part, wrong or unpriced
in part), falls (the claim is wrong or missing).

| id | claim attacked (synthesis section) | why it may be wrong | severity | verdict | cheapest experiment, and what result changes what |
|---|---|---|---|---|---|
| R1 | "Weak clients: the hybrid's main gift is worthless here" (§1.1) | True while every peer's CPU runs Box3D. False once L4 exists: at the north star's counts the mass of the simulation is on the GPU, the twin is 2.5 to 3x Box3D (§3.1), and a peer whose GPU fails the battery or is an AMD or Pascal part that runs slow drags every lockstep session down to the twin's counts. The hybrid runs the session at the host's capacity; lockstep at the weakest peer's. "No thin clients" was decided for solo play; it does not say a client may not do less in co-op | major | partly | Arithmetic from the synthesis's own numbers, then one measurement: `lpf_bench --scene siege --workers 4` on any old quad-core (or the i7 pinned to 4 cores), physics x2.75. If the twin cannot hold 60 Hz at the session's counts on the floor CPU, decide now whether a "twin-only peer follows" path (the hybrid's apply path for that peer) is in L1's scope |
| R2 | Server-timed lockstep with an 8-tick tolerance handles the slowest peer; fracture spikes are absorbed by count budgets (§1.1, §1.2) | A spike is paid by every peer on the same tick; the slow peer then owes ticks while the next ticks are also heavy (barrage: a blast every 3 ticks). Counts cap the spike, not the debt. Stormworks left slowest-peer sync for exactly this (T2). The catch-up figures (T6 §1c) use averages, not the spike series; no per-tick series has ever been logged | major | partly | Add per-tick step ms to `--hash-log` (an hour). On barrage and siege at 4 workers, count ticks over 16.7 ms and the longest over-budget run, scaled x1.5 for the floor CPU. Then the M9 two-process harness with one process throttled: count demotions in 600 ticks. If a floor peer is demoted during every collapse, the follower path is not an edge case and belongs in L1 |
| R3 | Repair is "the intended degradation toward follower behaviour"; a repaired object converges once it sleeps (§1.2 "Hashes and repair") | A structure repaired by its bond set alone re-diverges at the next stress check (strain, `stressX`, the solving systems are 0.4 to 3.5 MB per structure, T0 §1.6); a moving island repaired as-built loses warm starts on the repaired peer while the other bodies in the island keep divergent ones, so it re-diverges within ticks; under a barrage nothing sleeps. A peer under sustained divergence is then state-synced at state-sync bandwidth (an island of 500 bodies is 50 KB per repair) with lockstep's input delay on top. The synthesis prices repair at "sporadic KB" | major | partly | In L1's two-world harness, perturb one warm-start impulse by 1 ulp in world B at tick 200 of barrage; log ticks to detection, repair bytes per second, and whether B ever reconverges before tick 600. If it never does, repair is a lab tool and flip condition (2) is already met in the lab: build the follower path |
| R4 | Late join costs 3.7 to 8.6 MB and "joining takes seconds" (§1.2 "Late join") | Those sizes are Box3D's verbatim image, which is same-build only. A Windows host with a Linux or macOS joiner (the CI matrix's own premise) cannot use it: it needs the lean format to carry Box3D's hidden state (warm starts, sleep timers, id pools, colours: "a custom deserializer and Box3D internals knowledge", T0 §2) or the canonical rebuild, a synchronised hitch for every peer on every join. Unpriced: 20 ms of Box3D serialisation stalls the host's clock; xz -6 of 16 MB is seconds of host CPU | major | partly | Time zstd -3 and xz -6 on T0's probe dumps (an hour): expect zstd as the real choice at about 5 MB. Then, on Box3D alone, restore a lean re-encoding (no pair set, no hull registry, contacts re-created) at tick 300 and continue: if the hash diverges, cross-build join is the canonical rebuild and its hitch must be measured before "seconds" is promised |
| R5 | Input delay is uniform and tolerable for tools, cranes, trucks (§1.2 "Clock and input delay") | Pulls and held assemblies (the hauling game) are per-tick commands with 50 to 100 ms of delay on a spring: R.E.P.O.'s "round trip on every grab" that its developers "struggled with for a long time" (T2), in delay form. T1 §4 g1 already concedes a carried cart trails the predicted body by 0.3 m. Not discussed in the synthesis | minor to major | partly | One day: add a 6-tick delay to `pull`, `drive` and `walk` events in the sandbox's replay path and play the yard, track and mech demos by hand. Decides whether held objects must be predicted with the character (a second predictor) before L1 fixes the command set |
| R6 | The local character is a kinematic mover predicted outside the world, rebased onto the canonical body; "cranes and rigs are never predicted" (§1.2 "The local character") | goals.md makes the player "one of these bodies too": a rig with pools, vitals, knockout. A rig is a chain of motorised hinges walked by a gait; Factorio's latency state predicts a sprite with no physics. A capsule rebased every tick onto a torso that stumbles, sags and is shoved is a constant correction, not a rare snap. If the player is a rig, the synthesis's own rule leaves the first-person body unpredicted at 50 to 100 ms; if a mover, the impairment goals are not met by the body the player controls | major | falls as written | The same sandbox delay test on the hexapod at 100 ms, judged by a human. Then a design decision before L1: "the player is a mover carrying a damage model" or "the player is a rig". The command set, the predictor and the first-person game's feasibility under lockstep all follow from it |
| R7 | "Only lockstep's bandwidth is flat in that number": 5,000 awake bodies cost 9.6 Mbit/s per client (§1.1) | That is T6's state-sync profile for every awake body at 20 Hz. A hybrid would tier: ghosts and scrap are 52 to 69% of bodies and client-local (T0 §3); rubble is one reliable command; ballistic debris is two keyframes per flight, not 30 updates (T6 §2.4). The crossover where no uplink suffices is further out than stated. Flatness in the limit stands | minor | partly | Recompute T6 §12.1 with the tiers: only full and light debris state-synced, ghosts as keyframes. If 5,000 awake fits a median uplink at 4 players, the bandwidth argument decides only the dream's counts, not the floor's |
| R8 | The second choice is dismissed because a float GPU on the host "makes the host's outcomes unreproducible headlessly" (§1.3) | The hybrid removes the need for the L0 CI, the bit-exact snapshot, the twin, the battery, Eva and CBMC, and the Pascal and AMD questions: a float GPU solver on the host works on every vendor at 3e-6 tolerance (AVBD's class, T5). Rough price: follower mode 4 to 6 agent-weeks (T1's "largest surface") plus an as-built snapshot 2 to 3 plus a host float GPU solver 8 to 12, against L1 + L3 + L4 at 26 to 38. What it loses is real (bandwidth at the dream's counts, snapping, a second decision path per system), but the synthesis never states the counterfactual's price in its own units | minor to major | partly | None to run; a paragraph to write. The decision may stand, but the roadmap should record what was declined and at what price, so that flip conditions (1) to (3) can be judged against it |
| R9 | "Nobody has shipped it" is answered by the four rules and T8's matrix (§1.1) | The rebuttal covers arithmetic, late join and desync. It does not cover R1 (GPU heterogeneity) or R2 (spike debt), which are the reasons physics-heavy titles left slowest-peer sync (Stormworks, Planetary Annihilation: T2). The RTS precedents share none of the spike profile | minor | partly | Covered by R1 and R2 |
| R10 | "Determinism on the CPU is settled" (§2, H1 confirmed) | Settled for 12 rungs x 600 ticks (10 s of play each) on hosted runners. Not covered: long sessions, `lpf_test` hashes across legs (only pass/fail), rare paths, and two known latent hazards left in place: Box3D's NEON `b3MaxW(zero, s*inv_h)` on -0 (T3 hazard 5; "never fired" in 7,200 ticks is not "cannot fire"; a raw -0 in a velocity is a hash mismatch), and the dozen braced initialisers with several draws (T8, unsequenced in C) | minor | stands, with two cheap fixes | Take T3's 2-instruction NEON patch and rewrite the braced initialisers in L0 (an hour each). Add a nightly: 216k ticks of barrage on a Windows leg and a Linux ARM64 leg, hashes diffed. If it stays green for a month the "settled" claim earns its word |
| R11 | The six new rules cover "simulation or scene code" (§3.2 L0) | The games live in a private repo and will write AI drivers, scripts and gameplay logic that read state and write commands on every peer. One `atan2f` in a steering controller, one unstable `std::sort`, one accumulator fed from frame time, and the field desyncs while the CI stays green. The synthesis states no rule for game code and no CI can see it | major | falls (missing) | A rule, not an experiment: AI and scripts either run on the host and enter as commands (no determinism burden, trivial bandwidth: 3 cars x 60 Hz x 16 B) or obey the dialect and join a hash CI in the game repo. Default to host-run AI. Decide in L0 |
| R12 | `lpFpGuard` at step entry and in every worker (§3.2 L0) | Immediate API calls (`lpCreateObject` computes mass from shapes, `lpCreateLink` reads transforms, `lpCreateVehicle`, `lpCreateRig`) run in float on the caller's thread between steps, outside the guard, until L1 moves them to the tick's command point (T0 #19). An audio or overlay DLL that sets FTZ on the main thread diverges those calls in the L0 to L1 window | minor | partly | Put the guard on every public entry that touches state, not only the step (a macro, an hour). E9's harness with FTZ set before an `lpCreateObject` call is the unit test |
| R13 | Per-object hashes with the solver state localise a divergence to an object and a tick (§1.2) | Box3D's contact state (warm-start impulses, 1.03 MB of manifolds at the town peak) is never hashed (T0 §6 "Coverage gaps"). E9 showed the world hash blind for 600 ticks while the solver had diverged; the same holds for a warm-start difference, which surfaces only when a transform moves, by which time an island is involved. Detection latency sets repair's cost (R3) | major | partly | One day: extend Box3D's recording hash to manifold impulses (word-wise, about 0.3 ms at the peak), then perturb one impulse on one twin world and compare ticks-to-detection with and without the manifold hash. If the gap is tens of ticks, the manifold hash goes into the per-tick vector |
| R14 | "A desync in the field becomes a failing headless test" (§1.1, the north star) | Only if the field ships the command log, the last snapshot and the object id home. Repair makes a silently re-diverging peer look like a working one; nobody files the bug. No telemetry or triage item exists in any rung | minor to major | falls (missing) | Scope it into L1: every repair is logged with tick, object and cause, rate-limited, uploaded on session end with the log and snapshot; a `lpf_bench --replay` that reproduces it is the acceptance test |
| R15 | The integer core is proven for block-scaled int32 on two vendors (§2 table, §3.1 L3) | E11 proved one contact kernel with fixed manifolds prepared on the CPU in double for 60 steps. Unproven: SAT narrowphase and clipping in integers (the branchy, tolerance-laden code, with a 64/32 division per edge), feature-id stability for warm starts under quantisation, joints and motors (the mech's servos, cranes), the gyroscopic step, sleeping and islands, restitution, contact persistence via merge-join, and the thing that matters most to the feel: whether a 10-high stack stands and a 200-body rubble pile sleeps in V4's 31-bit formats. fixed3d stacks, but with 128-bit products and a 40-bit inverse scale; V4 is much narrower and 9x less accurate than float on town rows | major | partly (the gates cover it, 8 to 12 weeks in) | Before stage 2: a 1,000-line integer toy on the CPU only: boxes, SAT and clipping in Q8.24, the V4 soft step, sleep. A 10-high stack and a 200-box pile for 3,600 ticks: drift, jitter, time to sleep, against Box3D on the same scene. If stacks creep or piles never sleep, the format is plan B (Q32.32, 128-bit products, Pascal dropped) and the GPU case is re-priced |
| R16 | The one-tick pipeline lag is "deterministic semantics" that the outcome tests absorb; every reactive system tolerates a tick (§3.2 L3, T10 §6, T5 §4) | Unmeasured. The tyre solve and the rig's servos are 60 Hz feedback loops reading velocities and contacts; a tick of delay in a control loop with 4 substeps of stiff contact is a classic oscillation source. The fully pipelined form is two ticks. The predictor's casts run against a world one tick stale. T5's "would not notice" is an opinion | major | partly | The cheapest experiment in this report, on today's engine: buffer the post-step reads (`lpPollLinks`, `lpTrackMovingBodies`, `lpCollectHits`, the wheel casts, the rig's contact reads) by one tick in `step.c` behind a flag and run the nine suites and the track and mech rungs. A week, before L3, no integers. If vehicles at 40 m/s or the gait fail, the pipeline is redesigned (synchronous GPU step, or wheels and rigs on the twin) before 15-20k lines are written on the assumption |
| R17 | Driver bugs are contained by the battery and the twin fallback (§2, §5 risk 3) | The battery catches compile-time miscompiles (the Intel case is one). The containment's cost is the fallback: at the floor the twin cannot hold 5,000 awake bodies at 60 Hz (R1), so a floor machine whose driver fails the battery loses solo play at the target counts, and drags every lockstep session it joins. Pascal now sits on NVIDIA's legacy driver branch (as I recall, the R580 series is the last for Maxwell, Pascal and Volta: verify): bugs found there are permanent, though replays are stable. AMD untested, on a large share of players; the Steam Deck (RDNA 2, Linux) is the floor many own | major | partly | The synthesis's own Pascal and AMD runs, plus: run the battery and E11's binaries under the oldest driver each vendor still ships for those parts. If either vendor fails or runs over 5x float per row, the floor is redefined per vendor before L3 starts |
| R18 | V4's accuracy and angular saturation are fixable by Q11.20 and a revisited `r x P` (§3.2 L3, §5 risk 4) | Proposed, not re-run on the rows; the gate says "within what the outcome catalogue tolerates" and the catalogue does not exist | minor | partly | Re-run E11's rows with w in Q11.20 and the widening multiply (a day, the harness exists). Report rms on the town quarter; if still 5x float, the catalogue's tolerances are set with that number in hand |
| R19 | L3 + L4 cost 23 to 33 agent-weeks, "calibrated by T10" against fixed3d's three engineer-months (§3.1) | fixed3d was a mechanical port (one format, same architecture) by a human with an AI; T10 changes the architecture (per-quantity exponents, per-tick colouring, sorted broadphase, no persistent contacts, a GPU runtime, proofs). And the unit is uncalibrated here: this repo's 23k lines took four calendar days. The binding cost is not writing but verifying: the catalogue, Eva and CBMC, and CI machines with GPUs, none of which exist. On priority 1: +15 to 20k lines of contract-laden code, with Box3D vendored alongside for the whole of L3 to L4 (both engines alive for the longest stretch of the roadmap); "-45k vendored lines" is not a saving, those lines are never read today | major | partly | Stage 1 (the contact row, lint, Eva, CBMC, battery: 3 weeks) is the calibration. Set a hard stop: if it overruns 2x or Eva cannot discharge the row's alarms, the plan is re-priced before stage 2. Log tokens before and after as the review does |
| R20 | Box3D's losses are TOI and CCD, the mover, meshes, upstream (§5 risk 9) | The mover is what the predictor uses (R6). Meshes and height fields are how terrain is normally done; the hauling game asks for "terrain and caves that break", and the integer core has hulls only. Car-on-car at 30 m/s is already an open item under Box3D's speculative contacts | minor to major | partly | The synthesis's two-cars test, plus a terrain test: a 200 m x 200 m cave floor as hulls in the integer core, body count and step cost against Box3D's mesh. Decides whether terrain is pieces or a mesh shape the core must grow |
| R21 | "E11's twin at 100k contacts: 234 ms against Box3D's 125 at 1 worker" supports 2.5 to 3x (§3.1 L3) | Apples to oranges: E11's Box3D reference has 2.7x the manifold points per contact and the gyroscopic step, and E11 says "compare magnitudes only". Per point the scalar twin is about 5x. The whole-stage 2.5 to 3x rests on T10's assumption that the solve is 40 to 50% of the stage. And G-L3b gates at 1 worker on the i7, not 4 workers on the floor CPU | minor | partly | G-L3b measures it; add a 4-worker row on a quad-core to the gate |
| R22 | L1 (the serialiser) before the simplicity review, the review before the rewrite (§4.1, §4.2) | The serialiser is written over today's 432-byte bodies, 296-byte pieces and the stress scheduler's 17 flags (T0 §1.5), which the review then reshapes, and which L3 replaces again. Three serialisers. The synthesis's reason (testability) applies to the command log, the hash and the two-world harness, not to the serialiser | minor to major | partly | Split L1: command log, incremental hash with the solver state, per-object apply and the two-world harness first (small; the testability gain); the S-class serialiser after the review, over the state it leaves. Box3D's verbatim half is cheap either way |
| R23 | The Pascal run is "the one experiment to buy before committing to L3" (plan in one screen, §5) | Its result changes more than L3: if the floor GPU does not pay, L3 and L4 vanish, the review's scope changes (the physics-surface plumbing becomes reviewable), and milestone 13's profiling targets change. Yet it is scheduled three milestones away | minor | stands as an ordering error | Buy the used GTX 1060 now (about the price of a lunch for three) and run E11's unchanged binaries during L0; an RDNA card or a Steam Deck the same week |
| R24 | The revised roadmap (§4.1) | Missing: the character controller (goals.md: "not started", "for all" games; L1's command set needs its inputs; R6's decision), save schema versioning across patches (Box3D's image is same-build; "a zone left and revisited is as it was left" needs as-built loads that survive a numerics change), field telemetry (R14), the floor CPU measurement (T1's flip experiment 2, never done; the i7 stands in everywhere), terrain (R20) | major, collectively | falls (missing) | Add them as named items; the floor measurement is an afternoon on any old quad-core |
| R25 | L0 is the right next step (§4.3) | Right, and too thin. It should carry the experiments that can change the plan and cost days: R16's lag flag, R2's per-tick log, R10's two fixes, R12's guard placement, R23's card, R11's rule. And the five `ci-determinism` commits are hash-neutral and verified: merge them today rather than at L0's end | minor | stands, extended | None; a scope change |

## 1. The multiplayer model

The synthesis's case for convergent lockstep rests on four legs: solo play puts the whole simulation on the minimum
spec anyway; bandwidth is flat in the body count; T8 retired the bit-exactness risk; per-object repair turns a desync
into a correction. I tried each.

**The solo-play leg breaks under GPU heterogeneity (R1, R17).** The argument "everyone runs the whole simulation
solo, so a co-op client saves nothing by doing less" is exact while the simulation is Box3D on the CPU. The synthesis
then plans to move the mass of the simulation onto the GPU (L4) with a CPU twin at 2.5 to 3x Box3D's cost. At the
floor's own target (5,000 awake bodies at 60 Hz on an old quad-core) the twin cannot hold the frame: siege, with 1,650
awake debris, is already 17 ms at one worker on the i7, and physics is 10 ms of it. So after L4 a peer's capacity is
its GPU's, and GPUs differ far more than CPUs: a driver that fails the battery, an AMD part at quarter-rate integer
multiply, a Pascal card on a frozen driver branch. Under lockstep the session runs at the weakest peer's GPU-qualified
capacity; under the hybrid it runs at the host's, and the weak peer follows. That is precisely "the hybrid's main
gift", declared worthless in §1.1. The owner's decision was "everyone can play solo", which does not say a peer may
not do less in co-op when it cannot keep up. The synthesis does list "the twin cannot hold 60 Hz at the session's
counts" as flip condition (2), but as something "the field shows", when it is arithmetic from its own numbers today.
The cheapest check is an afternoon: `lpf_bench siege --workers 4` on any old quad-core, physics multiplied by 2.75.

**The slowest-peer answer ignores the spike debt (R2).** Server-timed ticks with an 8-tick tolerance stop a lagging
peer from stalling the others; they do not stop it from falling behind. Every peer pays the same fracture spike on the
same tick (baseline: 21 ms on the keep, 32 ms on siege at one worker), then the slow one owes ticks during a collapse
whose next ticks are also heavy. T6 §5.2 says so; the synthesis's reply is count-based budgets, which cap the spike,
not the debt. Nobody has logged a per-tick step series (`lpf_bench` gives p95 and max), so the number of over-budget
runs and their length on a floor-class machine is unknown. Stormworks left slowest-peer sync for exactly this, which
the synthesis files under "reasons that are gone". L2's exact fracture is estimated at 1 to 3x today's spike, so the
plan may make this worse before better.

**Repair is priced as sporadic kilobytes; it is state sync in disguise for anything that does not come to rest (R3,
R13).** A structure repaired by its bond set re-diverges at its next stress check, because strain, the warm start and
the solving systems (0.4 to 3.5 MB per structure) are not shipped; if they are shipped, a keep repair is seconds on a
median uplink. A moving island repaired as-built loses warm starts on one peer while the island's other bodies keep
divergent ones, so it re-diverges within ticks. Under a barrage nothing sleeps. The synthesis concedes "repaired
again until it sleeps" and calls it the intended degradation; it does not price it, and the price is the hybrid's
per-client bandwidth with lockstep's input delay added. Detection latency makes it worse: Box3D's contact state is never
hashed, so a warm-start divergence is seen only when a transform moves. E9 showed the same blindness for the stress
solver (600 ticks). The one-day experiment is to perturb one warm-start impulse in the L1 harness and count ticks to
detection and to reconvergence. If barrage never reconverges, the follower path is needed and belongs in L1, and the
"second decision path in every reactive system" that the synthesis avoided is back.

**Late join's measured sizes are for the case the CI matrix exists to avoid (R4).** The 3.7 to 8.6 MB figures are
Box3D's verbatim image, same-build only by its layout hash. A Windows host and a Linux joiner, which is the whole point
of a 14-leg CI, must use either a lean format that re-encodes Box3D's hidden state (T0: needs a custom deserialiser and
internals knowledge, unverified) or the canonical rebuild, a synchronised hitch on every peer at every join. Neither is
measured. Nor is the host's cost: 20 ms of Box3D serialisation stalls the host, which is the session's clock, and xz -6
of a 16 MB dump is seconds of CPU on a machine that has 16.7 ms per tick. zstd is the realistic codec and its size is
not in the report.

**The player's body is the unresolved contradiction (R6).** goals.md makes the player one of the impairment bodies: a
rig with pools, vitals and knockout. The synthesis predicts the local character as a kinematic mover (Factorio's latency
state, a pattern built for a sprite with no physics) and says cranes and rigs are never predicted. If the player is a
rig, the first-person body is unpredicted at 50 to 100 ms; if a mover rebased every tick onto a torso that stumbles and
is shoved, the correction is constant. Which one the player is decides the command set L1 fixes and whether the
first-person game is feasible under lockstep at all. The hybrid handles a physical player body more gracefully (Space
Engineers: only the controlled entity is dynamic on the client). This is the only place I would add a fourth flip
condition. The experiment is a day: a 6-tick delay on `walk` and `drive` events in the sandbox and a human at the
keyboard.

**What stood.** Bandwidth is flat in the body count under lockstep and is not under any streaming model; at the
dream's tens of thousands of physical bodies no home uplink streams them, and the synthesis is right that this decides
the limit. I could only narrow the crossover (R7): T6's 9.6 Mbit/s at 5,000 assumes every awake body is state-synced at
20 Hz, and a hybrid would tier (ghosts and scrap client-local, rubble one command, ballistic debris as keyframes), so the
floor's 5,000 may fit a median uplink at 4 players; the dream's counts do not. T8's cross-machine result is real and
thorough. And the counterfactual is under-described rather than wrong (R8): the hybrid would cost roughly 15 to 20
agent-weeks against L1 + L3 + L4 at 26 to 38 and would take the Pascal, AMD, battery and twin questions off the table
by running a float GPU solver on the host at AVBD-class tolerance; what it gives up (bandwidth in the limit, snapping, a
second path per system, host-bound sessions) is what the synthesis says, but the roadmap should record the declined
price so flips (1) to (3) can be judged against it.

## 2. Determinism

**The CPU verdict earns its word within its scope and the scope is narrow (R10).** Twelve rungs of 600 ticks on
hosted runners is ten seconds of play per rung, and `lpf_test` is compared across legs only as pass or fail. Two latent
hazards are left in on the strength of "never fired in 7,200 ticks": Box3D's NEON `b3MaxW(zero, s*inv_h)` gives +0
where SSE gives -0 (T3 hazard 5; a raw -0 in a hashed velocity is a mismatch), and a dozen braced initialisers draw
several random numbers in unsequenced order (T8). Both fixes are an hour. A nightly run of an hour's simulated time on
two ISAs is the cheap way to make "settled" mean more than "settled for ten seconds".

**Game code is outside every rule and every CI (R11).** The games live in a private repo and will run AI drivers,
scripts and gameplay logic on every peer. One libm call in a steering controller and the field desyncs while the matrix
stays green. The fix is a rule, not an experiment: AI and scripts run on the host and enter as commands (no determinism
burden; three cars at 60 Hz are bytes), or they obey the dialect and join a hash CI in the game repo. The synthesis
should state it in L0.

**The guard's placement has a window (R12).** Immediate API calls compute in float on the caller's thread between
steps until L1 moves them to the command point; a DLL that sets FTZ on the main thread (audio middleware is the usual
culprit) diverges them under a guard that only wraps the step. Guard every state-touching entry.

**The emulator caveat is handled (concede).** With libm out of simulation and scene code and the CRT static, Prism and
box64 are exact; `sqrtf`, `floorf` and `remainderf` are exact everywhere. The synthesis's rule is right and sufficient.

**Repair as a bug-hider (R14).** Per-object hashes do not hide systematic divergence: the root mismatches at once. What
hides it is repair without telemetry: a peer that repairs an island every few seconds is "working", and "a desync in the
field becomes a failing headless test" stays a lab property. No rung ships the log, the snapshot and the object id home.

## 3. The integer GPU core

**E11 is one kernel; the feel lives in the rest (R15).** The synthesis's "confirmed on two vendors" is true of a
contact solve on fixed manifolds prepared in double on the CPU. What stays unproven is the branchy half (integer SAT and
clipping with a 64/32 division per edge, feature-id stability for warm starts under quantisation, joints and motors,
sleeping and islands, contact persistence by merge-join) and the property the feel tests actually pin: does a 10-high
stack stand and a rubble pile sleep in V4's 31-bit mantissas? fixed3d is evidence that fixed point stacks, but with
128-bit products and a 40-bit inverse scale; V4 is narrower and 9x less accurate than float on town rows (E11 §6). The
gates cover this, eight to twelve weeks in. A 1,000-line CPU-only integer toy (boxes, SAT, the V4 step, sleep) answers
it in a week or two, before stage 2, and its failure mode (creeping stacks, piles that never sleep) is the one that
would send the design to plan B and drop Pascal, which changes the GPU case itself.

**The pipeline lag is the cheapest thing to falsify and it is not scheduled as an experiment (R16).** T10 introduces
the one-tick lag as semantics and the synthesis says the outcome tests will absorb it. The tyre solve and the rig's
servos are 60 Hz feedback loops with stiff contacts; a tick of delay is a classic oscillation source, and the fully
pipelined form is two ticks. Nothing has been measured; T5 §4's "an AI driver or a mech would not notice" is an
opinion. The experiment needs no integers: buffer the post-step reads in `step.c` by one tick behind a flag and run the
nine suites and the track and mech rungs. A week, on today's engine. If vehicles at 40 m/s or the gait fail, the
pipeline is redesigned before 15 to 20k lines are written on the assumption.

**The driver-bug surface is contained at a price the synthesis does not state (R17).** The battery catches compile-time
miscompiles (the Intel int64 case is one) and the twin is the fallback; fair. The price is the fallback's speed at the
floor (R1): a floor machine whose driver fails the battery loses solo play at the target counts and drags every session
it joins. Pascal is, as I recall, on NVIDIA's legacy branch (the R580 series being the last for Maxwell, Pascal and
Volta; verify), so a bug found there is permanent, though a replay recorded there is stable forever. AMD is untested on a
large share of players, and the Steam Deck (RDNA 2 on Linux) is the floor many own. The synthesis's two experiments are
right; add the oldest driver each vendor still ships for those parts.

**The effort unit is uncalibrated and the agent cost is understated (R19).** T10 calibrates 23 to 33 agent-weeks
against fixed3d's three engineer-months, a mechanical port with one format and the same architecture, done by a human
with an AI. T10's design changes the architecture and adds a GPU runtime and proofs. In this repo the whole first-party
engine, 23k lines and six milestones, landed in four calendar days; either the unit is a human-month or the integer
core is forty times harder per line. Either way the binding cost is verification, not writing: the outcome catalogue,
Eva and CBMC harnesses, and CI machines with GPUs, none of which exist. On priority 1 the accounting is also off:
"-45k vendored lines" is not a saving (they are never read), and Box3D stays vendored beside the new core for the whole
of L3 and L4, so two physics engines are alive for the longest stretch of the roadmap. Stage 1 is the calibration; give
it a hard stop.

**Smaller points.** V4's fixes (Q11.20, the widening multiply) are un-rerun and the gate that absorbs the residual
error names a catalogue that does not exist (R18); a day on the existing harness. E11's twin-versus-Box3D figure that
the synthesis quotes for 2.5 to 3x is apples to oranges (E11 says "compare magnitudes only"; per point the scalar twin is
about 5x), and G-L3b gates at one worker on an i7 rather than four on the floor CPU (R21). The list of Box3D losses omits
the two the games need: the mover the predictor uses, and meshes for terrain and caves that break (R20).

## 4. The ladder and the roadmap

**L1 before the review writes the serialiser three times (R22).** The serialiser is written over today's 432-byte
bodies and 296-byte pieces, which the review reshapes, which L3 replaces. The synthesis's reason for L1 first
(testability) applies to the command log, the incremental hash and the two-world harness, not to the S-class serialiser.
Split L1: the small half first, the serialiser after the review over the state it leaves.

**The decisive experiment is scheduled three milestones late (R23).** The synthesis names the Pascal run as the one
experiment to buy before L3 and puts it before milestone 11. Its result also decides whether L3 and L4 exist, what the
review may touch (the physics-surface plumbing is excluded only because L3 replaces it) and what milestone 13 profiles.
A used GTX 1060 costs little; run E11's unchanged binaries during L0.

**Missing from the roadmap (R24):** the character controller (goals.md: not started, needed by every game, and its
inputs must be in L1's command set); save schema versioning across patches (Box3D's image is same-build; persistence
across a numerics change needs an as-built load and a settle); field telemetry and desync triage (R14); the floor CPU
measurement (T1's flip experiment 2 was never run; the i7 stands in for the floor everywhere in the plan); terrain.

**L0 is right and too thin (R25).** It should carry the experiments that cost days and can change the plan: R16's lag
flag, R2's per-tick series, R10's two fixes, R12's guard placement, R11's rule, R23's card. The five `ci-determinism`
commits are hash-neutral and verified by T8; merge them today.

## 5. What the synthesis did not consider

- **GPU heterogeneity under lockstep** (R1, R17): after L4 the session's capacity is the weakest peer's GPU, and the
  twin cannot hold the floor's counts. This is the largest unstated consequence of combining lockstep with a GPU core.
- **The spike debt on the slow peer** (R2), and that L2 may raise the spike.
- **Which body the player is** (R6), and what it does to the first-person game.
- **Cross-build late join** (R4): the CI proves cross-OS sessions work; the measured join path does not serve them.
- **Detection latency of Box3D's hidden state** (R13), which sets repair's real cost.
- **Game code as simulation code** (R11).
- **Terrain** (R20) and **save versioning** (R24).
- **The frozen Pascal driver branch** (R17): permanent bugs, stable replays.

## The three objections most likely to change the plan

1. **R1 with R17: lockstep plus a GPU core makes every session run at the weakest peer's GPU-qualified capacity, and
   the twin cannot hold the floor's counts.** This is arithmetic from the synthesis's own numbers, not a field finding.
   It either adds a follower path to L1 (the surface the synthesis chose lockstep to avoid) or redefines the floor per
   vendor and accepts that a peer without a qualifying GPU plays at twin counts, which the goals do not say. An afternoon
   on an old quad-core settles the number.
2. **R16: the one-tick pipeline lag has never been tried on wheels and rigs, and it can be tried this week on today's
   engine.** If the tyre solve or the gait oscillates under a tick of delay, the pipeline design that L3 and L4 are
   built on changes before any of their 15 to 20k lines exist. No other experiment in the plan has this ratio of cost
   to consequence.
3. **R6: the player is a rig or a mover.** The synthesis assumes a mover; the goals describe a rig; the two need
   different command sets, different predictors and, for the first-person game, possibly a different model. A day in
   the sandbox with a 6-tick delay puts a human in front of the question before L1 freezes the commands.

R15 (integer stacking in V4's formats) is the fourth, and the one most likely to send the integer design to plan B.

## What the synthesis got right that I tried and failed to break

- **Bandwidth in the limit.** At the dream's counts no streaming model fits a home uplink; lockstep's flatness is the
  decisive property and the numbers hold (I could only move the crossover, R7).
- **The CPU determinism evidence.** E5's design is what the literature asks for, and its findings (libm clusters, the
  argument-order hazard, the Prism CRT routing) are exactly the failure classes every shipped lockstep game reports.
  The "every documented desync was a comparator, hidden state, UB or libm" claim checks out against T2 §4 and T3 §5,
  with the note that x87 precision and compiler promotion are arithmetic-environment hazards that 64-bit-only and the
  guard cover.
- **H5 and the per-object hash with the solver state.** Every track and every case study agrees; E9's lesson is taken.
- **Block-scaled int32 on GPUs.** E11 measured it at 1.16 to 1.35x float on two vendors with identical saturation
  counts; T4's 10 to 20x estimate is dead and the synthesis says so with the right qualifications.
- **The honest "unproven" row,** the deletion of Box3D gated on G-L4b rather than G-L3b, the lag introduced on the CPU
  before any GPU exists, and naming the Pascal run as decisive. My complaints there are about ordering, not judgement.
- **The float GPU dialect verdict** (conditional, empirical per driver): E11 §7 and §10a support every word.
- **The hybrid's dismissal on "unreproducible headlessly"** is a values call under "built by agents"; I disagree with
  its weight but could not show it wrong.
