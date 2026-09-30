# M7 T7: synthesis. The multiplayer model, the determinism verdicts, the architecture ladder and its place on the roadmap

Written 2026-09-30. This closes milestone 7 (docs/roadmap.md §7). It turns the ten track reports in this directory and
the owner's decisions of 2026-09-29 and 30 into one plan, judged against docs/goals.md ("North star", "Priorities",
"Multiplayer targets", "Feel") rather than the brief's earlier framing (toasters, 1,500 awake bodies). Where tracks
disagree, measured numbers win, and each claim cites the report and section that holds the evidence. Repo numbers are
from `bench/baseline.json` (commit cce5ea5, 600 ticks, i7-10870H, workers 1 and 8).

## The plan in one screen

- **Model: convergent lockstep** (T1 §6, g3). Every machine runs the whole simulation from one command log; the host
  relays inputs and keeps the clock; the local character is predicted outside the deterministic world; per-object
  hashes localise a divergence and the host repairs that object. Chosen because the solo requirement already puts the
  whole simulation on the minimum spec, because the north star's body counts make streaming body state impossible on
  a player host (5,000 awake bodies at 20 Hz is 9.6 Mbit/s per client, T6 §12.1, while lockstep's host upstream is
  0.1 to 0.9 Mbit/s at any count, T6 §12.3), and because T8 retired the risk T1 named as its flip condition: this
  engine is bit-exact across five compilers, two ISAs, three OSes and three emulators once three hazards of ours are
  fixed (T8 E5).
- **Determinism on the CPU is settled. On the GPU it is settled for block-scaled 32-bit integers on two vendors**
  (E11 §4, §6), conditional for a narrow float dialect, and unproven on Pascal, AMD, Apple and across driver updates.
- **The ladder:** L0 hardening (1 to 2 agent-weeks) → L1 the snapshot, command log, hash and repair primitive (3 to 5),
  with L2 exact integer fracture (2 to 4) in parallel → L3 an integer rigid-body core beside Box3D with a CPU twin
  (11 to 15) → L4 that core on the GPU, Vulkan first, Box3D deleted at the end (12 to 18). L3 has no standalone
  performance case; it is the price of L4. The cosmetic GPU layer (2 to 3) is optional and independent.
- **Roadmap:** L0 now, as milestone 7's close; L1 and L2 as a new milestone after the engine-surface pass; the
  simplicity review after that and before the core rewrite, scoped to what the rewrite keeps, with its outcome
  catalogue as the rewrite's gate; then L3, L4, profiling, zones, networking, dents, art.
- **The one experiment to buy before committing to L3:** E11's unchanged binaries on a used GTX 1060 and on any AMD
  card. The floor's cost of the integer contact row is the number the GPU case rests on, and it is unmeasured.

## 1. The multiplayer model

### 1.1 Convergent lockstep, and why the case-study objection does not apply

T1 chose it (T1 §1, §6). T2 argued against it: no shipped real-time 3D destruction game runs full lockstep, and the
physics-heavy titles that tried slowest-peer synchronisation (Stormworks, Planetary Annihilation) left it for host
authority (T2 "The short version" 1; "Questions across games" 4). T2's reasons were weak clients that should do little
physics, the slowest peer setting everyone's pace, late join without a snapshot, and a desync ending the session. Under
the owner's decisions each is gone or answered:

- *Weak clients.* Everyone must be able to play solo, so the minimum spec runs the whole simulation anyway (goals.md
  "Multiplayer targets"). A client that does less physics in co-op than solo saves nothing that matters; the hybrid's
  main gift is worthless here.
- *The slowest peer.* The weakest machine bounds a solo session and a lockstep session alike; what lockstep adds is
  that in co-op it bounds the others too. The answer is what count-based budgets already give: session-wide settings
  chosen for the weakest peer (T1 §5 "CPU and the slowest peer"), server-timed lockstep so a lagging peer loses control
  briefly instead of stalling the rest (Quantum's hard tolerance, T6 §1c, §5.2), and demotion of a hopeless peer to a
  follower (T1 §4 f/g3).
- *Late join and migration.* T0 measured what a bit-exact join costs: 3.7 MB (town), 4.1 (keep), 7.3 (barrage) and
  8.6 MB (siege) under xz at the worst tick of each rung, nothing at load, and Box3D's own snapshot is proven bit-exact
  for restore-then-continue in six scenes (T0 "Summary", §2). Host migration is free: every peer holds the state (T1 §5
  "Late join, migration, saves").
- *A desync ends the session.* Not with per-object hashes and repair: a mismatch names an object and a tick, the host
  ships that object's image, and the command log plus the last snapshot reproduce the field desync in `lpf_bench` (T1
  §4 f/g3, §5 "Desync"). That is the north star's "a desync in the field becomes a failing headless test".
- *Nobody has shipped it.* True. The float-lockstep games that shipped (Factorio across Windows, Linux, macOS and Switch
  ARM; Supreme Commander; Age of Empires) held on the same four rules every deterministic engine now uses, and every
  documented lockstep desync was a comparator, hidden state, undefined behaviour or libm, never IEEE arithmetic (T2
  "Questions" 4; T3 §5). T8 then ran the experiment on this engine: 24 runs byte-identical per tick across 10 native
  legs and 3 emulated ones (T8 E5).

The number that decides it is the north star's scale. T6's worked example (§12.1): a delta-coded body update is about
12 B, so 1,500 awake bodies at 20 Hz cost 2.9 Mbit/s per client, 5,000 cost 9.6 Mbit/s, and a 4-player host uploads
29 Mbit/s and a 12-player host 106 Mbit/s for a collapse every player sees. Teardown's answer, a priority queue capped
at 1 Mbit/s per client (T2 "Teardown multiplayer"), buys about 10,400 body updates a second: 7 Hz per body at 1,500, 2 Hz
at 5,000 and under 1 Hz at the dream's counts, which is snapping, not synchronisation. Lockstep sends inputs: 0.1 Mbit/s
of host upstream at 4 players and 0.87 at 12 (T6 §12.3), whatever is breaking. The goals ask for a world made of tens of
thousands of things that break; only lockstep's bandwidth is flat in that number.

Two properties follow from the same choice. The GPU is compatible with lockstep only if its results are bit-identical
to a CPU twin on every peer; T10 and E11 make that the design (§2 and §3.2 below), so lockstep does not forbid the north
star's GPU ambition, it requires exactly what the twin provides. And every system that reads physics (stress, links,
supply, rigs, wheels, freezing) keeps one decision path; the hybrid would give each a second "apply what the host said"
path (T0 §3 "Follower mode"; T1 §4 c), doubling the test matrix for the agents who maintain them.

### 1.2 The pieces

**Clock and input delay.** Server-timed lockstep with the host as the clock: inputs go to the host, which merges each
tick's inputs into one packet (Factorio's scheme, T1 §6); a peer whose input is late past a hard tolerance (Quantum's
default is 8 ticks) has its last input repeated and loses control briefly; a peer that falls behind runs extra ticks per
frame, which the town's 1.9 ms step at 8 workers allows and its 12.5 ms maximum at 1 worker barely does (T6 §1c "Catch
up"). Input delay: 2 to 3 ticks plus a jitter buffer, 50 to 100 ms at typical pings, applied adaptively above a ping
threshold (T6 §5.1). Tools, cranes and trucks feel it uniformly (T1 §3 "latency feel"), which the rigging-race game
tolerates because its players rig while the AI races (goals.md "Multiplayer targets").

**The local character.** Factorio's latency state (T1 §4 g1; T6 §5.6): the canonical player body lives in the
deterministic world, driven by committed inputs; each frame the local view re-applies the pending inputs to a kinematic
mover (Box3D's `b3World_CastMover` today, the integer core's casts later) from the confirmed state, a few casts costing
microseconds. Its effects enter as tick-stamped commands: tools are resolved on the shooter's machine against what it
displays and sent as "piece, local point, direction" (T6 §6.4; cheating is out of scope), pulls and pushes as the
commands T0 §4 lists. Knockouts and dents apply to the canonical body and the predictor rebases onto it, so impairment
stays inside the deterministic world. Vehicles stay in the world with input delay; Factorio hid 500 ms of car driving in
2024 (T6 §1c), so a predicted vehicle is a later bolt-on, not a model change. Cranes and rigs are never predicted (Space
Engineers' rotor lesson, T2 strategy 28).

**Hashes and repair.** `lpWorld_Hash` walks every vertex byte by byte and costs as much as a step at the town peak: 2.9
to 4.2 ms, and 6.5 ms at 16k pieces (T0 §6). T0's replacement (§6 "Incremental hash"): a geometry digest per piece
computed once, a commutative position-sensitive mix so an update is O(1) per changed element, dirty hooks at every
mutation site, per-body and per-island grouping under a small Merkle tree over body-index ranges, word-wise mixing; an
estimated 0.05 to 0.2 ms at the peaks (unverified). Peers exchange the 8-byte root every tick and descend the tree in
O(log n) round trips on a mismatch. The hash must include the stress solver's state: E9 kept the world hash equal for
600 ticks while the solver had diverged under FTZ (T8 E9). Repair is T1's f: the host sends the object's image (a body's
transform, velocity and type; a structure's bond set, from which `split.c` recomputes components; a link's, wheel's,
rig's or pool's state), applied at a tick boundary on every peer including the host, so the repaired image is the new
shared truth. Repair is as-built (warm starts and sleep timers are not shipped), so a still-moving island is repaired
again until it sleeps; that is the intended degradation toward follower behaviour for a peer that diverges often (T1 §4
f). A peer that fails the float self-test is refused; a peer whose GPU fails the kernel battery runs the twin.

**Late join: snapshot, not command log.** Replaying the log from the session start is cheap in bytes and ruinous in
CPU: one minute of town is 7 to 32 s on 8 workers, so Teardown caps its buffer and refuses joins past it (T1 §5 "Late
join"). The snapshot is T0's measured image: Box3D's verbatim serialisation (4.5 MB at the town peak, bit-exact once our
filter callback is reinstalled and `userData` rewired, T0 §2) plus a lean format of our S-class fields (about 3.9 MB, T0
§1.1), 3.7 to 8.6 MB compressed at the worst ticks of the big rungs and nothing at load. "Joining takes seconds"
(goals.md) then means a 5 to 10 s transfer at home uplinks (medians 38 to 65 Mbit/s, tails single-digit, T6 §10) and a
catch-up that a fast joiner does in 3 to 4 s and a slow one cannot: pause-on-join is the fallback (T6 §7.2). Box3D's
image is same-build only (a layout hash); sessions that mix builds use T6's canonical rebuild (every peer, host
included, restores from the same bytes at an agreed tick, §1e). The integer core removes the question, its snapshot
being plain integers (T10 "Summary").

**Session-wide settings.** Every count in `lpWorldDef` except `workerCount` and `debugLog`, the `b3WorldDef`, dt and
substeps, the build id, the content hash, T3's self-test hash and T10's kernel-battery hash travel in the handshake and
fold into the checksum, as Quantum folds its session config (T6 §5.1). Worker count is free by house rule, a real asset:
peers with different core counts stay in sync and differ only in speed (T1 §4 a). Quality is per session, chosen for the
weakest peer; only the cosmetic layer scales per machine (goals.md "Simulation scale per session").

**Spread-out players on large maps.** Lockstep peers simulate the union of every player's active zones; the hybrid's
per-client relevancy would let each client simulate only its own (T6 §8). This is lockstep's one cost that grows with
player count, and it lands on CPU, not bandwidth. Zones derive activity from shared state (roadmap §12; determinism rule
7), so the union is identical everywhere and its size is a session setting: a count of active zones, everything outside
frozen or baked. Whether 12 players spread across a facility exceed the floor is the measurement milestone 14 must make
(§5, risk 6).

### 1.3 The second choice, and what flips to it

The Teardown hybrid with cell lists (T1 §6, d): the host simulates; bodies are prioritised, eventually consistent state
at about 1 Mbit/s per client; destruction ships as the host's quantised cell geometry (150 to 250 B per piece, T6 §1b) so
no client needs any determinism; structural decisions are small reliable commands; clients run follower mode with a
predicted kinematic character; vehicles may be driver-owned. Its niche is the case lockstep handles worst: many players
spread over a large map, each seeing a disjoint few hundred bodies, on a session whose weakest peer cannot run the union
of zones while its strongest can host it.

What would flip to it, in order of likelihood: (1) milestone 14's measurement shows the union of active zones at 8 to 12
players exceeds the floor at the lowest acceptable session budget while one player's zones fit, and the games want those
sessions; (2) the field shows a repair rate that never settles (drivers failing the kernel battery so often that the
twin, not the GPU, is the common case, and the twin cannot hold 60 Hz at the session's counts); (3) a bit-exactness
failure the CI matrix cannot reproduce. Nothing else flips it: bandwidth favours lockstep at every count, the snapshot
sizes are measured, and cross-machine bit-exactness has passed its experiment. The GPU does not flip it either way:
under the hybrid the host could run any float GPU solver, but that makes the host's outcomes unreproducible headlessly
(T5 §7), which the north star's "built by agents" rules out.

## 2. Determinism verdicts

| scope | verdict | evidence |
|---|---|---|
| CPU across compilers, OSes and ISAs (x64, ARM64; MSVC, clang-cl, clang, gcc, AppleClang; Windows, Linux, macOS) | **Confirmed.** Byte-identical per tick on 24 runs (12 rungs × 2 worker counts) across 10 native legs, compilers not pinned (MSVC 19.42 and 19.51, clang-cl 22 and 23, clang 18, gcc 13, AppleClang 21), once three hazards of ours are fixed: gcc without `-ffp-contract=off`, C-library `cbrtf`, `sinf` and `atan2f` in simulation and scene code, and random draws inside a call's argument list. Box3D needed no patch: the NEON min/max concern never fired. | T8 E4, E5; T3 §1, §7 |
| x64 under emulation on ARM64 | **Confirmed for Rosetta 2, Prism and box64** (box64 with `FASTNAN=0 FASTROUND=0` every tick; its defaults differ on one transient post-build line); **FEX untested.** Caveat: Prism and box64 both route C-library calls to native ARM64 code, so the simulation must be libm-free or link the CRT statically. | T8 E5; T3 §2 |
| WASM | **Open, expected to hold.** The spec's only arithmetic nondeterminism is NaN bits; Emscripten's SSE2 shims use x86 semantics; relaxed SIMD must stay off. No build was made; low priority. | T3 §3 |
| GPU, float | **Conditional, and empirical per driver.** A Box3D-shaped solve is bit-exact on an RTX 3060, an Intel UHD and a CPU twin at 100k contacts for 60 steps only in a dialect narrower than Box3D's: add, sub and mul with `NoContraction`; no GPU sqrt, rsqrt or division (one 1-ulp sqrt diverges the solve within a step); reciprocals from the CPU; per-vendor float controls (`RoundingModeRTE` on NVIDIA, which as an undocumented side effect also preserves subnormals; `DenormPreserve` on Intel); no dependence on the sign of zero. Every condition is driver behaviour the spec does not pin. D3D11 is out (mandatory flush-to-zero, truncation permitted). | E11 §4, §7, §10a; T8 Appendix A; T3 §4 |
| GPU, integer | **Confirmed on two vendors for block-scaled int32** (T10's V4, E11's I32): bit-exact on both GPUs, both twin compilers and three shader front ends at 1k, 10k and 100k contacts, saturation counts identical, at 1.16 to 1.35× the float solve's cost. **Q32.32 in int64 is not the format:** correct on NVIDIA at 2.2× float, miscompiled by the Intel driver and 10× on it. Qualification: exact semantics are necessary, not sufficient; a driver compiler bug and a shift-count UB (`>> 64` gave −1 on the 3060, 0 on x86) each split vendors, so a per-driver startup battery and the CPU twin are part of the guarantee. | E11 §4, §5, §6, §9; T10 §0, §8 |
| Unproven | Pascal (the floor GPU: XMAD multiply cost, whether the RTE side effect holds); any AMD part; Apple GPUs through Metal or MoltenVK; Blackwell (the DGX Spark, CUDA via Slang); replay stability across a driver update; newer Intel drivers (does the int64 miscompile persist); FEX; WASM. | E11 §10c-d; T5 §9; T8 "Not done" |

## 3. The ladder

### 3.1 The rungs, priced

Effort is in agent-weeks, calibrated by T10 (fixed3d converted all of Box3D in about three engineer-months, T4 §6).
"Floor" is a GTX 1060-class GPU beside an old quad-core; "dream" an RTX 2080-class PC; E11's RTX 3060 laptop stands in
for about half an RTX 2080 (T10 §3.3).

| rung | what | effort | risk | performance at the floor and the dream | unlocks | agent-friendliness |
|---|---|---|---|---|---|---|
| **L0 hardening** | merge `ci-determinism`; portable math as the only path (one hash change); the MXCSR/FPCR guard; the self-test vector; T0's order-independence and hash-coverage items; float-to-int clamps; new rules; CI kept | 1 to 2 | low | neutral: the guard is about 2 ns; the sorts are over tens of items | the CI matrix as a standing check; every later rung | positive: rules, a 14-leg CI, a few hundred lines |
| **L1 the primitive** | serialise and rebuild (ours lean, Box3D verbatim behind a small patch); a tick-stamped command queue in the core; stable ids and generations; the incremental per-object hash with the solver state; per-object apply (repair); session settings and the handshake; a two-world lockstep harness | 3 to 5 | medium: exposing Box3D's snapshot is a patch of unverified size (T0 §9); the lean format's compression is unmeasured | hash 3 to 6.5 ms → about 0.1 to 0.2 ms per tick (estimate, T0 §6); serialise 20 ms at the town peak (measured, Box3D's side) | saves, late join, host migration, repair, replays with seek, zones' snapshots, "every bug a replay" | +2 to 3k lines; the largest testability gain available |
| **L2 exact fracture geometry** | integer sites and bisector planes; 4D homogeneous integer classification with 128-bit predicates; exact vertices rounded once; mass and bond areas in a fixed op order; hulls from topology | 2 to 4 | low to medium: silent 128-bit wrap (ship a saturating debug mode); a from-topology hull builder may need a Box3D patch | the fracture spike 1 to 3× today's on one core, 15 to 30 ms at 8 workers against 20 to 90 (estimate, T4 §4); one hash change | removes the plane-shift hack and every tolerance flip; fracture bit-identical by construction; required by L3 | +about 600 lines, one hack removed |
| **L3 integer core, CPU twin** | `lpfix.h` helpers, the lint, Eva and CBMC harnesses, the startup battery; the contact row; then bodies, Q8.24 hulls, a sorted broadphase, SAT narrowphase, per-tick colouring, the soft step with four joints, motors and limits, islands and sleep, GJK and casts; the one-tick pipeline lag as deterministic semantics; beside Box3D behind the physics surface, switchable per world | 11 to 15 (T10 stages 1 and 2) | high: solver quality against Box3D's; the twin's speed; exponent bookkeeping the compiler cannot check; TOI and CCD, the mover and upstream cadence lost | **a loss on the CPU:** physics 2.5 to 3× Box3D scalar, 1.3 to 1.6× with AVX2 (T10 §4; E11's twin at 100k contacts: 234 ms against Box3D's 125 at 1 worker); town at 1 worker stays under a frame (3.3 ms → about 6 to 7 scalar, 4 to 4.5 AVX2), siege does not (17 → about 32 to 37 scalar, 20 to 23 AVX2) | nothing on its own: it is L4's prerequisite; side gains: integer snapshots, no flags to police in the core, integer SIMD without a scalar twin | −45k vendored lines (never read); +15 to 20k first-party lines with range contracts (about +150 to 250k tokens: the plan's largest agent cost) |
| **L4 the core on the GPU** | the Vulkan runtime (device, rings, one command buffer per tick, timeline semaphores, readback); every kernel from the L3 sources; differential CPU/GPU tests; the battery in the handshake; integer-atomic stress loads; the CPU fallback; then Box3D deleted and every hash re-baselined | 12 to 18 (T10 stages 3, 4, 5) | high to medium: dispatch-bound below a few thousand bodies (E11: 0.78 ms at 1k contacts against Box3D's 0.24 at 8 workers); Pascal unmeasured; driver bugs; a second toolchain | floor: an estimated 1.5 to 2.5 ms of GPU time at 5,000 awake bodies on a 1060 (T10 §3.3, unverified; G-L4b measures); dream: 20,000 awake bodies under 8 ms of GPU time on the 3060 is the gate, consistent with E11's 7.2 ms for 100k fixed-manifold contacts | the north star's GPU inside the simulation; the dream's tens of thousands of bodies; the CPU freed for fracture, stress, links and rigs | a shader toolchain and a runtime; driver-dependent bugs; headless agents run the twin; CI needs a machine with GPUs |
| **Cosmetic GPU layer** (optional) | upload-only debris, dust and mess in a sokol compute pass, drawn from storage buffers, never read back | 2 to 3 | low | 100k pieces on a discrete GPU (AVBD-class numbers, T5 §5a); not a priority | more mess for free | one shader, nothing to match |

L0 to L4 total 29 to 44 agent-weeks; L3 plus L4 is T10's 23 to 33.

### 3.2 What each rung contains, and its gate

**L0.** Everything T8 recommends (T8 "Recommendations"): the five commits on `origin/ci-determinism` (73cdcbc to
9f0f43b: gcc's `-ffp-contract=off`, clang-cl's `/W4`, `Threads::Threads`, the dead `lpIsSlender`, the argument-order
fix, `--hash-log`, `LPF_PORTABLE_MATH`, the workflow, the Prism leg, the static-CRT probe), all hash-neutral on MSVC;
then portable math as the only path, which changes every hash once, after which the whole bench matches on every
tested OS, ISA, compiler and emulator; T3's guard (§6.3: `lpFpGuard` at step entry and in every worker, Box3D's
through `PATCHES.md`) with E9 as its unit test; T3's self-test vector (§6.4), its hash kept for the handshake. The
hash-changing T0 items go into the same re-baseline: #1 freeze candidates and #2 hit events sorted by key before
acting, #3 contact loads summed in sorted order, #5 cast ties broken by piece index. Within lockstep every peer holds
identical Box3D state, so these are not desync hazards today; they are what lets a repaired peer converge instead of
forking on report order, and they cut the dependence on the physics engine's ordering before that engine is swapped.
Hash-only additions: #8 to #11, body volume, `createdTick`, the serials. Cheap fixes: float-to-int clamps (T3 hazard
4: `debris.c:40` converts an unbounded position), `%.9g` in the sandbox recorder (#17; E10 shows `%.6f` changes 68 to
96% of values), #21, #23, a NaN check in the debug hash. Six new rules for `determinism-rules.md`: gcc in rule 1; no
side-effecting call inside another call's arguments or a braced initialiser; no C-library transcendentals in
simulation or scene code; the CRT linked statically on Windows; clamp before every float-to-int; NaN is a bug.
Optional: an Emscripten leg in CI (T3 §8 item 8). The step list and exit criteria are in §4.3.

**L1.** The serialiser over T0's S-class tables (§1.2 to §1.7), D caches rebuilt, scratch zeroed together (`stamp`,
`piece.mark`, `body.stamp`); Box3D's image verbatim through a small patch exposing `b3SerializeWorld` and a
deserialise-into-world (T0 §2 item 5), `userData` rewired to index + 1, the filter callback reinstalled (#16), `world0`
patched or the restore done in place (#15), the hull registry rebuilt from our shapes rather than shipped (3.4 MB
saved; optional). The remaining T0 items belong here: #6 (charge a continuing solve the same whether or not its system is
cached), #7 (ship a clustered solve in progress or restart it), #12 (recompute body volume canonically), #13 (fracture
seeds and ghost tumbles from keys, not slots), #14 (generations for bonds and wheels; body generations exposed), #18
(`CLAW_GRAB` and `CLAW_RELEASE`; keyed references in `pull`), #19 (immediate creations applied at the tick's command
point). The command queue is T0 §4's canonical set, stamped `(tick, peer, seq)`, floats as raw bits, applied in `(peer,
seq)` order before the step. Ids: slots stay local handles, mapped to T0 §5's global keys at the boundary, chosen so a
zone can be serialised alone (milestone 14's need: identity is opt-in, anonymous rubble bakes). The incremental hash as
in §1.2, with the Merkle tree and the solver state; `lpWorld_Hash` stays as the full-state check that validates a
restore byte for byte (T0 §6 "Coverage gaps"). Per-object apply for repair is the same code as restore; there is no
follower mode, and no reactive system gains a second decision path. Session settings as one struct with a hash. A
two-world harness in `lpf_test`: worlds A and B step the same command log; a GGPO-style sync test serialises and
restores every tick and compares (T6 §11); an injected repair must converge; an injected desync (FTZ forced on one
world) must be named by object and tick. *Gate:* the round trip on every rung (step to N, serialise, restore into a
fresh world, step both to 600, compare `lpWorld_Hash`, `lpWorld_HashStress` and the snapshots byte for byte, T0 §9); the
incremental hash equals the full hash every tick and costs under 0.3 ms at the barrage peak; the two-world harness
passes; join wall time measured at the town peak.

**L2.** T4 §4 option B as specified: sites snapped to 2^-14 m in the piece frame; bisector planes `n = 2(b − a)`,
`d = |b|² − |a|²` exact; vertices as plane triples classified by 4D integer dots within the Nehring-Wirxel 128-bit
bound; two-limb helpers for MSVC (about 150 lines); `lpPoly_Clip` and the Voronoi loop replaced (about 400 lines);
neighbour order and site rejection as integer compares, so the random-draw count no longer depends on a float compare;
sliver absorption and bond areas from exact vertices in a fixed double order; hulls from the exact topology so
quickhull's tolerances leave the loop (a from-topology `b3HullData` builder: grep `hull.c` first, patch if absent).
*Gate:* recorded fracture inputs from the town and keep blasts replayed through both paths: zero numerical failures and
no plane shifts; hash agreement across the CI matrix; the barrage rung's fracture spike within 2× today's at 8 workers.
Independent of L1; required by L3 (T10 §1.2 item 3, §6).

**L3.** T10 stages 1 and 2 (§9) with E11's amendments: angular velocity in Q11.20 or with a per-body exponent (V4's
±256 rad/s saturated in realistic rows, E11 §6); the `r × P` narrowing revisited for accuracy (V4's town rms error is
6.2e-4 against float's 6.9e-5; the clz variant did not help); explicit widening multiplies (`imulExtended`, about 30% on
the multiply-bound part, E11 §8); shift counts clamped to [1, 63] everywhere (E11 §4). Kernel language: T10's shared
C-and-HLSL subset with the shim header, so the twin is plain C that agents read and Eva and CBMC check; Slang, which
E11 showed produces correct twins first build, stays the third implementation for differential tests and the later
Metal and CUDA front end (T10 §7). Solver: Box3D's soft step transcribed, colouring recomputed per tick by fixed-round
Jones-Plassmann with the remainder solved by mass-split Jacobi (T10 §5); AVBD a later experiment on the same buffers. The
one-tick lag (T10 §6) is introduced here, on the CPU, so the outcome tests absorb it before any GPU exists. The core
runs beside Box3D behind the physics surface the brief lists (bodies, hulls, four joints, hit events, contact data,
queries, casts), switchable per world, so the bench runs both and the outcome tests compare them with tolerances.
*Gates:* G-L3a, before the full core: E11's row harness with the format fixes on the RTX 3060, the UHD and a Pascal
card, bit-exact, within 3× float per row on Pascal, accuracy on the town corpus no worse than float's or within what
the outcome catalogue tolerates; G-L3b (T10's G2): the nine suites' outcomes pass with tolerances on the integer core;
town at 1 worker within 2.5× Box3D (1.5× with AVX2); identical hashes across MSVC, clang, gcc, worker counts and ARM64.

**L4.** T10 stages 3 to 5 (§9): the Vulkan runtime (`shaderInt64`, timeline semaphores, `bufferDeviceAddress`; both
GPUs here report them; 64-bit buffer atomics only on the 3060, so per-body impulse totals use two 32-bit atomics, T10
§1.1); every kernel from the L3 sources; differential tests per kernel over corpora with edge classes on every machine
with a GPU; the startup battery (4,096 rows per kernel plus the helper tables) exchanged in the handshake, with the twin
as the identical fallback; the pipeline one tick behind in `lpWorld_Step`, the twin double-buffering; sleeping and
freezing under lag; then Box3D and its patches deleted and every bench hash and outcome re-baselined. Because twin and
GPU are bit-identical, each machine may choose per tick which one runs (below some body count the twin wins on latency)
with no effect on the session: that is the deterministic form of "the GPU carries the mass". Vulkan first (every vendor,
Windows and Linux, headless-capable, installed here); D3D12 second; Metal third (T10 §7). *Gates:* G-L4a (T10's G3): a
full island solve, broadphase to store, bit-exact against the twin for 600 town ticks on the 3060 and the UHD, re-run
after a driver update; G-L4b (G4): 5,000 awake bodies on a GTX 1060-class card with the whole step under 16.7 ms at one
CPU worker, and 20,000 under 8 ms of GPU time on the 3060. The deletion of Box3D is gated on G-L4b, not G-L3b: if the
GPU does not pay, the integer core alone is a loss and Box3D stays.

**The cosmetic layer** (T5 §5a): a sokol compute pass (vendored, D3D11 today, no readback, which is exactly this layer's
constraint), a spawn ring, a static collision field, an instanced draw; sized per machine behind the toaster profile;
never consuming the simulation's random stream or touching event order. Optional; the owner ranks it low.

**Other items found.** *Wake storms* (goals.md "Open"): under L4 a wake is a type flip in the delta list, so a storm's
cost moves to the GPU; the graceful form is a wave advancing a counted number of hops per tick in index order, which
milestone 14 designs on that mechanism. *Zones and persistence:* L1 decides ids and the snapshot's shape so a zone can
be serialised, baked (frozen pieces merged into the zone's static geometry, their bodies dropped) and restored alone;
the lifecycle's frozen state exists, baked and forgotten do not. *Stress on the GPU:* no (T5 §5b; T10 §6 keeps it a
double conjugate gradient under the dialect), but L4's integer atomics give it exact per-body impulse totals. *Memory
on the floor* (T0 #22): 432-byte bodies and 296-byte pieces carrying stress and ghost fields everywhere, hulls held
twice, 4.5 MB of fracture scratch: milestone 13's, unless L3's new body layout takes it first.

### 3.3 Dependencies and parallelism

L0 → L1 (L1 needs settled hashes). L2 is independent of L0 and L1 and runs beside L1 on its own branch. L3 needs L2
(integer hulls from exact geometry) and the outcome catalogue (the review's step 0), and benefits from L1, whose Box3D
half is a verbatim image behind one patch and cheap to drop when the integer core's snapshot replaces it. L4 needs L3.
The cosmetic layer needs nothing and nothing needs it. The Pascal and AMD experiments (§5) precede L3. Optional: L4's
D3D12 and Metal backends, WASM, the Emscripten CI leg.

### 3.4 Where the tracks disagreed

- *State-sync upstream at 12 players (T1 against T6).* T1 called 11 to 32 Mbit/s over most home uploads on an assumed
  5 to 20 Mbit/s; T6 measured 2026 medians of 38 to 65 Mbit/s, so Teardown's 1 Mbit/s per client fits a median host at
  1,500 bodies. Both hold at their scale. At the north star's counts the numbers are 9.6 Mbit/s per client at 5,000
  bodies (T6 §12.1) or sub-Hz refresh under a cap, and the argument is no longer the host's uplink but that no cap makes
  tens of thousands of bodies consistent. Lockstep is flat in the count.
- *Integer cost (T4, T10, E11).* T4 estimated 10 to 20× per multiply on GPUs for Q32.32 and measured 2.2 to 2.5× on the
  CPU (fixed3d); T10 predicted 2 to 4× per row on Turing and later and 5× on Pascal for block-scaled int32; E11
  measured 1.16 to 1.35× for the full solve on Ampere and Gen9.5, 1.76× per row on Ampere, and 2.14× (Ampere) to 4.84×
  (Intel) per row for Q32.32, 10× for the Q32.32 full solve on the iGPU. Measured wins: block-scaled int32 is cheap
  where measured; Q32.32 is the expensive and fragile format; Pascal remains T10's prediction until measured.
- *H4 (T5 against E11).* T5: no cross-GPU class exists in practice, same device and driver at best. E11 produced that
  class on two vendors for int32 with a battery. T5's iGPU verdict, its rejection of a GPU stress solve and its warning
  that the driver is a JIT compiler stand, and L4 is built on them.
- *Snapshot sizes (T1 and T6 against T0).* T1 guessed 6 plus 2 MB for town, T6 10 to 20 MB raw for lockstep; T0
  measured 16.4 MB raw and 3.65 MB xz at the town peak, 31 MB raw and 7.3 MB xz for barrage. Measured wins; the guesses
  bracketed it.
- *Kernel language (T10 against E11).* T10 rejected Slang's CPU target for the twin on readability and proof grounds;
  E11 used it and it was correct first build, with three rough edges (`x + 0` folded under precise mode, renamed entry
  points, Vulkan bindings dropped from HLSL output). Resolution in L3: the shared subset for the twin, Slang as the
  differential third implementation.

## 4. Roadmap placement

### 4.1 The revised order

| new | milestone | was | notes |
|---|---|---|---|
| 7 | Deep research (done), closed by **L0 hardening** | 7 | 1 to 2 weeks; start now |
| 8 | Engine surface and diagnostics | 8 | as written; its `--dump` shares L1's traversal of state, so keep it small and let L1 own the serialiser |
| 9 | **The primitive: snapshot, command log, hashes, repair (L1), with exact fracture geometry (L2) in parallel** | new | the H5 outcome; every later milestone uses it; ends with a two-process lockstep smoke test over a plain socket (no Steam yet) |
| 10 | Independent review: simplicity, the outcome catalogue first | 9 | scoped to what survives the core swap: fracture, split, stress, the logic of links and wheels, rigs, gait, supply, debris tiers, step order; the physics-surface plumbing (the Box3D calls in `world.c`, `link.c`, `wheel.c`, `rig.c`, `debris.c`, `step.c`) is excluded |
| 11 | **The integer core with a CPU twin (L3)**, preceded by the Pascal and AMD row experiments | new | gated by G-L3a; Box3D stays beside it |
| 12 | **The core on the GPU (L4)**; Box3D deleted at its end | new | gated by G-L4b |
| 13 | Profiling and a performance review | 11 | now profiles the architecture that will ship; the CPU side at 5,000 to 20,000 bodies (freezing, wakes, tiers, `Renderer_Sync`, memory) is the new bottleneck; the cosmetic layer as a lever |
| 14 | Large-map physics zones and persistence | 12 | per-zone snapshots from L1; bake and forget; wake storms on L4's delta list; the union-of-zones measurement (§5, risk 6) |
| 15 | Networking | 13 | thin under lockstep: transport (Steam sockets with a direct fallback), server-timed ticks, the latency state, join and migration over L1; it may move up to right after milestone 9 if a game needs co-op before the GPU core, since it depends only on L1 |
| 16 | Dents | 10 | its hull rebuild is written once, on the integer core |
| 17 | Art polish and demo views, with the cosmetic GPU layer | 14 | |

### 4.2 Why the review comes before the core rewrite

Three reasons, in order of weight. The outcome catalogue, the review's step 0, is a hard prerequisite of L3: an integer
core changes every numeric, only outcome tests with tolerances can gate it (T10's G2 says so), and the catalogue is the
review's own contract, so it must exist before L3 starts. Second, the review's targets are the subsystems that stay on
the CPU under T10's boundary (§6): fracture, stress, links, wheels, rigs, gait, supply, the debris tiers. They are what
grew by iteration in milestones 5 and 6, and a smaller version of them makes the L3 re-plumbing cheaper. Third, the
physics-surface plumbing that L3 replaces is the one thing not worth reviewing twice, so it is excluded by name. A
short second pass after L4 belongs to the profiling milestone, which already has reviewers. Against this order: 25 to
40 weeks pass before the north star's GPU exists. For it: the primitive and the catalogue are what make the rewrite
testable at all, and both are worth having even if L3 never passes its gate.

### 4.3 The next milestone: L0, hardening

Steps:

1. Merge the five commits of `origin/ci-determinism` into `sandbox` and `master` (hash-neutral: 9 of 9 suites and
   `bench.ps1 -StrictSolver` verified by T8).
2. Make portable math the only path: drop the option and the C-library call sites in `stress.c`, `world.c`, `impact.c`
   and `scenes.c`; take `cbrtf` out of the cosmetic sites too, for grep hygiene.
3. The order-independence batch: T0 #1, #2, #3, #5.
4. Hash coverage: T0 #8 to #11, body volume and `createdTick`, the serials.
5. Re-baseline `bench/baseline.json` once for steps 2 to 4 together; update the workflow's reference hashes; a perf-log
   entry with before and after step times.
6. `lpFpGuard` at step entry, in `lpWorkerMain` before each task, and in Box3D's scheduler workers (a two-line patch
   logged in `PATCHES.md`); `lpStats.fpRepairs`; E9 as a test in `lpf_test world`.
7. The self-test vector as `lpf_test determinism` and at world creation; its hash returned by a query for the handshake.
8. `lpFloatToInt` with a clamp at every unbounded conversion; a NaN check in the debug hash.
9. `%.9g` in the sandbox recorder; `piece.seed` dropped; vehicles and rigs freed.
10. Rules in `determinism-rules.md` (the six in §3.2); "Known limits" rewritten; the T0 items not taken here listed
    against L1 and L3 in the roadmap; roadmap §7 closed with a pointer to this file and the order of §4.1; the memory
    note updated.

Exit criteria: the 14-leg workflow green and byte-identical per tick on every rung at the new hashes; `pwsh
tools/build.ps1 -Test` and `pwsh tools/bench.ps1 -Repeat 3 -StrictSolver` green; the FTZ test and the self-test pass on
every native leg; the town and siege step averages within noise of the baseline.

## 5. Risks and open questions

Ranked by how much of the plan each can move.

1. **The floor GPU's integer cost is unmeasured.** T10 predicts about 5× float issue cost per row on Pascal from XMAD
   sequences (§3.2, §3.3); E11 could measure only Ampere (1.76× per row) and Gen9.5. If Pascal's ratio is 5× the 1060's
   estimate at 5,000 bodies becomes 4 to 7 ms and the floor still fits; at 10× it does not. *Retire:* a used GTX 1060
   running E11's unchanged binaries (`probe`, `run_rows`, `run_solve`, `mulbench`; E11 §10c), the Windows logs as the
   reference. Before L3.
2. **AMD is untested.** Quarter-rate integer multiply on RDNA (T4 §2; T10 §3.1) and unknown driver behaviour, on a
   large share of players. *Retire:* any RDNA card or a Steam Deck through Vulkan, the same binaries.
3. **Driver compilers are part of the binary.** The Intel driver miscompiled a large int64 kernel while passing every
   int64 op in isolation; NVIDIA's `>> 64` differs from x86; Slang folds `x + 0` (E11 §4, §9). *Retire:* never fully;
   contain with the startup battery, the twin fallback, differential tests on real GPUs in CI (a self-hosted runner:
   the laptop or the DGX Spark) and a driver-update re-run as a standing check (T5 §9 item 3).
4. **The integer solver's accuracy.** V4 is 9× less accurate than float on town rows and its angular format saturated on
   chips (E11 §6). Fidelity, not sync. *Retire:* Q11.20 or a per-body exponent for angular velocity, the `r × P`
   narrowing revisited, the rows re-run; the outcome catalogue decides what is acceptable.
5. **The GPU is dispatch-bound at today's scenes** (E11 §5, §11): it wins only past a few thousand awake bodies, which
   the debris tiers keep us under today. *Retire:* G-L4b; the per-machine, per-tick twin-or-GPU switch makes the loss
   below the threshold zero.
6. **The union of active zones at 8 to 12 players** on a large map may exceed the floor while one player's zones fit (T6
   §8): lockstep's only scale cost. *Retire:* milestone 14 measures it on a floor CPU with a spread-out script;
   mitigations are a session cap on active zones, baking outside them and follower demotion; the flip to the hybrid if
   the games need those sessions.
7. **The CPU side at 20,000 bodies.** Freezing, wakes, ghosts, tiers, the hash and `Renderer_Sync` are O(bodies) on the
   CPU; the north star moves the bottleneck there. *Retire:* a synthetic 20k-body rung in milestone 13's profiling; SoA
   layouts and the memory items (T0 #22).
8. **Join time.** "Seconds" needs a transfer of 4 to 9 MB and catch-up headroom (T6 §7.2). *Retire:* measure at the end
   of L1; pause-on-join as the fallback.
9. **Box3D features lost in L3:** TOI and CCD (car-on-car at speed, already an open item, keeps speculative contacts
   and the speed cap only), the character mover, meshes, upstream fixes. *Retire:* a two-cars-at-30 m/s outcome test
   before deletion; the mover is a few hundred lines of casts.
10. **Agent-friendliness of L3.** Fifteen to twenty thousand new first-party lines whose range contracts the compiler
    cannot check. *Retire:* the lint, Eva and CBMC harness in stage 1 before any kernel is written; token counts logged
    before and after, as the review does; the kernel directory the one place with contracts.
11. **Apple, Blackwell, FEX and WASM** are nice-to-have and unproven, and the CPU dialect has two loose ends (braced
    initialisers with several random draws, in order on every compiler seen, T8 E5; no `-flto` leg). *Retire when
    convenient:* the MacBook through MoltenVK with fast math off and `slangc -target metal`; the DGX Spark for the
    ARM64 twins, GB10's unified lanes and CUDA with `--fmad=false` (E11 §10); a static x64 build for FEX; an
    Emscripten and an LTO leg; the L0 rule.

Hardware to acquire, in order: a used GTX 1060 (decisive, cheap), an RDNA card or a Steam Deck, then time on the DGX
Spark and the MacBook for E11's listed runs.

## 6. Hypotheses

| hypothesis | verdict | evidence |
|---|---|---|
| H1: Box3D float with pinned flags, not pinned compilers, is bit-exact across MSVC, clang and gcc on x64 and ARM64; the residual hazards are ours | **Confirmed.** Five compilers, two ISAs, three OSes agree per tick on 24 runs once gcc gets `-ffp-contract=off`, libm leaves simulation and scene code, and random draws leave argument lists; Box3D unpatched; compilers unpinned | T8 E4, E5; T3 §7 |
| H2: lockstep is right for 4p and acceptable at 12p; the local character is predicted outside the deterministic world; a hybrid loses on player-host upstream at 12p | **Confirmed with amendments.** Right for 4p; acceptable at 12p only server-timed with per-object repair; the local character as Factorio's latency state is a shipped pattern; the hybrid's upstream at 1,500 bodies fits a median host (T6), but at the north star's counts no player host can stream the bodies at all | T1 §6; T6 §12, §13; T2 "Verdicts" |
| H3: fixed point everywhere costs 2 to 4×, forfeits Box3D and solves a problem flags and tests solve; exact predicates for fracture may be worth it anyway | **Mixed.** 2.2 to 2.5× confirmed for CPU-only Q48.16 (fixed3d), worse for Q32.32 limb code; **falsified for block-scaled int32 on GPUs** (1.16 to 1.35×); "forfeits Box3D" false in principle (fixed3d exists) and true by choice in L3; "flags solve it" confirmed on the CPU by T8, so integers are justified by the GPU alone; exact fracture predicates confirmed worth it | T4 §7; E11 §5, §11; T8 |
| H4: GPU physics stays off the deterministic path and is a net loss on an iGPU; the cosmetic layer and perhaps the async stress solve are the candidates | **Partly falsified.** A GPU contact solve is on the deterministic path across two vendors and a CPU twin, in int32 and in a narrow float dialect, given per-driver controls and a startup battery; the iGPU and small-scene loss is confirmed; the async stress solve is not a candidate; the cosmetic layer is valid and low priority | E11 §11; T5 §8; T10 "Summary" |
| H5: a serialisable, command-logged world with per-island or per-structure hashes is the most valuable outcome, not a solver rewrite | **Confirmed and sharpened** by every track: per-object, with the solver state in the hash, one primitive for join, migration, saves, repair and replays; Box3D's half exists and is proven, ours is missing and specified. The rewrite is valuable too, but for the GPU, not for determinism | T0 §7; T1 §6; T2, T6 "Verdicts"; T8 E9 |
| H6: x64 SSE2 under Rosetta 2 and Prism is bit-exact by default, and under box64 and FEX with fast modes off | **Confirmed for Rosetta 2, Prism and box64; FEX open.** With the caveat that Prism and box64 hand C-library calls to native ARM64 code, so a libm-free simulation or a static CRT is part of the guarantee | T8 E5; T3 §2 |
| T10: integers make cross-vendor bit-exactness a property of the language specs, not of drivers | **Qualified.** Necessary, not sufficient: a driver miscompile and a shift-count UB split vendors; the battery and the twin close the gap | E11 §9, §11 |
| T10: block-scaled int32 needs no 128-bit products in the solver | **Confirmed** on rows and the full solve (V4 exact everywhere it ran, no clamp flips on realistic rows), with the angular velocity format too narrow as specified | E11 §4, §6 |
| T10: the floor GPU pays about 5× float issue cost per row; Turing and later 2 to 4× | **Open for Pascal; Ampere measured at 1.76× per row and 1.35× for the full solve**, better than predicted | E11 §5, §6; T10 §3.3 |
| T10: drivers strength-reduce sign-extended 32×32→64 multiplies | **Partly falsified:** NVIDIA's compiler does it less well than an explicit `imulExtended` (5.7× against 3.9× an FMA) | E11 §8 |
| T10: Slang's CPU target is unfit as the twin | **Falsified in practice** for barrier-free kernels (correct first build, all variants); the readability and proof objection stands, so the shared subset is still the twin | E11 §1, §9 |
| T10: an integer GPU solve beats the CPU at 5,000 awake bodies on a GTX 1060 and reaches 20,000 on an RTX 2080 | **Open;** G-L4b. E11's 7.2 ms at 100k fixed-manifold contacts on the 3060 makes the 2080 half plausible; the floor half waits on the Pascal run | E11 §5; T10 §9 |

## Sources

The track reports in this directory, cited above by section: `m7-state-audit.md` (T0), `m7-model.md` (T1),
`m7-case-studies.md` (T2), `m7-float.md` (T3), `m7-fixed-point.md` (T4), `m7-gpu.md` (T5), `m7-netcode.md` (T6),
`m7-experiments.md` (T8), `m7-gpu-integer.md` (T10), `m7-gpu-experiments.md` (E11). Repo: `docs/goals.md`,
`docs/roadmap.md`, `docs/determinism-rules.md`, `bench/baseline.json` (cce5ea5), `origin/ci-determinism` (73cdcbc to
9f0f43b). The owner's decisions of 2026-09-29 and 30 as recorded in `docs/goals.md` ("North star", "Multiplayer
targets").
