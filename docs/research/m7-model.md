# M7 track T1: the multiplayer model decision

Written 2026-09-29. Scope: which multiplayer model lowpoly-fracture should build for (milestone 13), judged for
player-hosted co-op at 4, 8 and 12 players on toaster clients, for the three games in docs/goals.md. The other tracks
own the evidence this leans on: T3 (float determinism across CPUs and emulators), T4 (fixed point), T5 (GPU), T6
(technique catalogue), T0 (state audit), T8 (experiments). Repo numbers are from `bench/baseline.json` (commit
cce5ea5, 600 ticks, i7-10870H) and the shared brief.

## 1. The decision in one paragraph

First choice: **convergent lockstep**. Deterministic lockstep with a small input delay, inputs relayed through the
host (star), every peer simulating the whole world; the local character predicted outside the deterministic world
(Factorio's latency-hiding pattern) with its effects entering as tick-stamped commands; and per-object hashes with
host repair so that a divergence becomes a visible correction of one object instead of a dropped session. Second
choice: **the Teardown hybrid** (host-authoritative state sync, destruction shipped as reliable commands, clients in
follower mode). The choice turns on three numbers: host upstream during a big collapse (lockstep ~0.1 Mbit/s at 12p;
state sync 11–32 Mbit/s at 12p, over most home uploads), the toaster floor (lockstep puts the whole world on the
weakest peer: `siege` costs 17 ms a step at 1 worker on the i7, so the session's budgets must be set for that peer),
and engineering surface (lockstep adds one primitive, serialize/rebuild with hashes; the hybrid adds a second decision
path to every reactive system). H2 is confirmed with one amendment (pure lockstep is too brittle at 12p; repair is
what makes it acceptable), H5 is confirmed and sharpened (the primitive is per-object, and the same primitive gives
late join, host migration, saves, rollback and repair).

## 2. Models weighed

| id | model | who simulates | what must be bit-exact across peers |
|---|---|---|---|
| a | lockstep, input delay | everyone, everything | everything: Box3D, our core, scene building, game code |
| b | lockstep + rollback/prediction (GGPO, Photon Quantum) | everyone, everything, k ticks re-simulated per correction | everything, plus a per-tick snapshot/restore of everything |
| c | host-authoritative state sync (snapshots, priority accumulator) | host only; clients interpolate | nothing (host's result is the truth) |
| d | Teardown hybrid: deterministic destruction as reliable commands + host-synced float bodies | host; clients recompute destruction geometry | the fracture function only, if commands are impacts; nothing, if commands are cell lists |
| e | distributed authority (owner simulates what they touch or drive; Fiedler VR, Roblox network ownership) | each peer, its owned objects | nothing |
| f | lockstep + checksum-and-repair (per-object hashes localise a desync; host repairs that object) | everyone, everything | everything, but a miss costs a repair, not the session |
| g | hybrids: g1 = lockstep world + locally predicted character; g2 = c/d + predicted character + driver-owned vehicles; g3 = f + g1 (the first choice) | as a/f | as a/f, character excluded |

Added beyond the brief's list: g3 (f with g1's character), and the "cell list vs impact" split inside d.

## 3. The matrix

Scenario for the numbers: the friendslop worst case, one building collapsing with 1500 awake bodies (the debris cap)
while every player is inside, 60 Hz ticks. Town is the reference scene: 7.6k pieces, 3.0k bodies, 1.2k awake, step
1.9 ms avg / 6.6 ms max at 8 workers and 3.3 / 12.5 ms at 1 worker; `siege` is the heavy end: 9.2 / 27.5 ms at 8
workers, 17.1 / 52.5 ms at 1 worker; a blast refracturing 100+ pieces costs 20–90 ms once (brief; baseline
`fractureMaxMs` up to 32 ms). "Toaster" below assumes a machine 2–3x slower than the i7 at 1 worker.

| criterion | a lockstep | b lockstep+rollback | c host state sync | d Teardown hybrid | e distributed authority | f/g3 convergent lockstep |
|---|---|---|---|---|---|---|
| bit-exact scope | all sim code, all peers (x64 MSVC/clang/gcc; ARM64 and emulators if those peers join) | as a, plus snapshot/restore bit-exact | none | fracture only (impact commands) or none (cell lists) | none | as a, but a miss is repaired |
| CPU per client | whole world: town 3.3 ms (1w) → ~7–10 ms toaster; siege 17 ms → 35–50 ms toaster (below 60 Hz) | as a, times (1+k) on a correction; k≈6 ticks at 100 ms RTT → town 11–40 ms (8w), 20–75 ms (1w) per corrected frame | interpolation + a kinematic mover: ~0.1–0.3 ms | as c plus recomputing new cells (~0.2 ms per fracture) | owned objects only; the peer who triggers a collapse pays the whole collapse | as a, plus per-object hashing (~0.1 ms/tick incremental) |
| CPU on host | as a client, plus relaying ~12 × 60 small packets/s | as a | whole world + N priority queues over ~1500 bodies (~0.1 ms/client) | as c | arbitration + own objects + relay | as a client plus repair packets |
| slowest peer | sets the pace for everyone (AoE speed control) or is dropped (Factorio auto-kick); sim budgets are session-wide | worse: rollback cost stacks on the slowest peer | irrelevant to others; a slow client only lags its own view | as c | a slow owner degrades only what it owns | as a; a hopeless peer could be demoted to a follower (needs c's apply path, which f has for repair) |
| host upstream, 12p / 8p / 4p, collapse | 0.09 / 0.06 / 0.03 Mbit/s (inputs ~16 B × 60 Hz × players) | as a | full 1500 bodies @20 Hz, ~10 B delta each: 29 / 19 / 10 Mbit/s; capped at Teardown's 1 Mbit/client: 11 / 7 / 3 Mbit/s | as c plus destruction commands (~20–40 KB per 100-piece blast, reliable) | as c if relayed via host; per-owner upload if P2P | as a plus hashes (~1 kB/s) and sporadic repairs (KB) |
| client downstream | ~8 kbit/s | ~8 kbit/s | 1–3 Mbit/s during the collapse, ~15 kbit/s at rest | as c | as c | as a |
| home upload fit (5–20 Mbit/s assumed, unverified) | trivial at any count | trivial | 4p fits; 8p marginal; 12p over budget or snapping (per-body refresh ~6 Hz at 0.8 Mbit/client) | as c | as c | trivial |
| latency feel: tools, cranes, trucks | input delay ≈ RTT/2 + jitter buffer + 2–3 ticks ≈ 80–150 ms, uniform | none for the local player, corrections pop remote ones | RTT round trip ≈ 60–150 ms unless predicted on the client (needs the truck's physics client-side) | as c | driver-owned truck: immediate; cross-owner contacts pop | as a |
| latency feel: walking character | delayed unless predicted (g1: immediate) | immediate | immediate (kinematic mover, standard FPS prediction) | as c | immediate | immediate (g1) |
| player as a destructible dynamic body | yes: the canonical body lives in the deterministic world; the predictor is a mover rebased on it each tick | yes | yes, host-decided impairment arrives after RTT | as c | yes, owner-decided | as a |
| late join image | bit-exact snapshot: Box3D internal (~6 MB est. for town, same-build only) + ours (~2 MB) + catch-up | as a | as-built image: ~100 KB state + fracture log (~36 KB) rebuilt in <1 s, or ~2 MB with raw geometry | as c | as c | as a, or an as-built image that converges by repair |
| host migration | trivial: every peer holds the state | trivial | needs an as-built copy on clients (they have one) | as c | as c | trivial |
| save game | the same snapshot, or seed + command log | as a | as-built image | as c | as c | as a |
| desync risk / consequence | real (float across builds, T3); fatal (Factorio quits after 3) | as a | none by construction | none (cell lists) or fracture-only | none; ownership conflicts instead | real; consequence is one object snapping |
| detection / debuggability | per-tick hash, `lpWorld_Hash` exists; a command log replays the field desync headlessly | as a | nothing to debug but snapping | as c | ownership thrash to debug | per-object hash names the object and tick |
| large maps (roadmap 12) | every peer simulates everything; zones derived from shared state cut work for all, never bandwidth | as a | host simulates everything; per-client relevancy cuts bandwidth to the visible awake set | as c: a far collapse costs its commands only | natural | as a |
| Box3D | keep, harden flags, add a portable snapshot | fork-level: per-tick snapshot/restore | keep as is; GPU on host eligible | keep as is; GPU on host eligible | keep as is | keep, harden, portable snapshot |
| engine surface added | serialize/rebuild + hashes + input queue | as a + per-tick snapshot + resim + fracture memo | follower mode: an apply path in every reactive system | as c + destruction command path | as c + ownership in every decision | as a + per-object apply (subset of c's follower mode) |

Bandwidth arithmetic. A delta-encoded body update after Fiedler's snapshot compression is about 26 bits of position,
23 bits of orientation and 2 flag bits (~52 bits, 6.5 B) at 60 Hz [3]; at 10–20 Hz with velocities for extrapolation
and an 11–13 bit body id it is ~10 B, which is the figure used above. 1500 bodies × 10 B × 20 Hz = 300 kB/s = 2.4
Mbit/s per client; with N clients the host uploads N times that. Fiedler's cube demos ran 901 cubes at 60 Hz inside a
256 kbit/s budget after delta encoding, with ~15 kbit/s at rest [3][4]; Teardown budgets about 1 Mbit/s per client
with a per-client priority queue and reports visible snapping when the queue cannot keep up [1]. Steam's networking
sockets clamp the default send rate at 256 KB/s (2 Mbit/s) per connection [8]; it is configurable, but it is a hint
about what the transport expects. Lockstep sends inputs only, so its bandwidth is independent of the object count [2].

Rollback arithmetic (b). k = RTT × tick rate + jitter ≈ 3–4 ticks at 50 ms, 6–7 at 100 ms, 9–10 at 150 ms. Each
corrected frame re-runs k steps: town 6 × 1.9–6.6 ms = 11–40 ms at 8 workers; `siege` on a toaster 6 × 35–50 ms =
200–300 ms. Fracture inside the window is a pure function of (piece, impact, seed), so it can be memoised and replayed
free; everything else cannot. Whole-world rollback is out for our sizes on toasters; per-area prediction (Quantum's
prediction culling, unverified this session) and per-object resimulation (Unreal Chaos: Resimulation for chosen
actors, Predictive Interpolation for the rest [12]; Unity Netcode for Entities: predicted vs interpolated ghosts in
separate physics worlds bridged by proxies [11]) are the shipped forms of partial rollback, and both carry the same
caveat: objects across the boundary interact approximately.

## 4. What breaks in our engine, per model

**a (and the lockstep half of f/g3).** Everything reactive must be bit-exact on every peer, so the whole list from the
brief applies: `cbrtf` in stress.c/world.c/impact.c (not correctly rounded, differs by libm), no `-ffp-contract=off`
for GCC/clang in our CMake (Box3D sets it for itself only), libm trig in scene building (scenes are built at session
start on every peer, so they are simulation code now), Voronoi neighbour sorts and plane-clip signs that flip on one
ulp, and the places where our determinism leans on Box3D's report order (move events filling the freeze cap unsorted,
contact-data order feeding stress loads, hit-event order driving detonations). Box3D's own scope is the good news:
Box2D v3's determinism post reports identical results across MSVC, GCC and clang on x64 and ARM (M2 vs Ryzen shown)
with FMA off and libm trig replaced [9]; Box3D inherits the approach (brief: `b3MulAddW` is add(mul), own atan2 and
cos/sin, only `sqrtf`/`floorf`/`remainderf` from libm). Rapier makes the same demand explicit (`enhanced-determinism`,
IEEE 754-2008 compliant targets, no `simd8`) [10]. Session-wide settings: every count in `lpWorldDef` (lpf.h
134–168: `maxFullDebris`, `maxLightDebris`, `maxGhosts`, `maxRubblePieces`, `maxScrapPieces`, `maxFractureJobsPerStep`,
`maxFreezesPerStep`, `maxStressWork`, ...) changes the state, so the lobby must agree on them; worker count does not
(determinism at any worker count is already a house rule), which is a real asset: peers with different core counts
stay in sync and only differ in speed. Missing today: no serialize/rebuild of our world (heap pointers, Box3D ids), no
tick-stamped input queue in the core (the sandbox has one), no per-object hash (`lpWorld_Hash` is whole-world and
hashes raw vertex bytes of every piece: ~4 MB for town, milliseconds, not per tick).

**b.** Everything in a, plus a bit-exact snapshot/restore of the whole world every tick. Box3D has one internally
(`b3SerializeWorld`/`b3DeserializeIntoShell`, extern/box3d/src/world_snapshot.c:1016 and :1133, guarded by a layout
hash at :84/:1159, same build only) used for replay keyframes and backward seek; Catto states Box2D v3 offers no
rollback determinism because of internal state and no snapshot support [9], so Box3D's snapshot is the exception that
makes b possible at all. Its cost and size on town are unmeasured (T8). Our side has no snapshot. Then the resim cost
above. Not viable on toasters at our sizes.

**c.** Nothing in the deterministic core changes, but a follower mode must exist: every reactive phase in the step
order (impacts → fracture jobs, bond and link damage, splitting, stress checks, wakes/forces/blows/pulls, link sync,
pools, supply, rigs, motors, vehicles, link polling, ghosts, shove, hits, freezing) gains a second path in which the
client does not decide but applies what the host sent. Clients keep Box3D only as collision proxies for the predicted
character (kinematic bodies moved by snapshots; Box3D's mover, extern/box3d/src/mover.c, is the natural controller).
Pieces' geometry must still reach clients: as cell lists (bytes, no determinism) or as impact commands (needs the
fracture function bit-exact, including Box3D's quickhull). Stress, links, supply, rigs, wheels and freezing become
host decisions replicated as small commands: bond breaks and component splits (split.c is a pure graph operation on
the bond set, so clients can recompute components from bond-break commands), link tears, wheel losses, rig limb state
(`rig.c` already has a hash and a state block), pool levels, supply reach. Freezing is a body-type change: replicate
the type. This is the biggest surface any model adds, and it doubles the agent's test matrix (host path, follower
path).

**d.** All of c plus the destruction-command path. The decision inside d: send the impact (piece id, site, seed;
~24 B; clients recompute cells; the fracture must be bit-exact across compilers, which is the fixed-point candidate
Teardown took [1]) or send the cell list (~200 B per piece quantised; a 100-piece refracture ≈ 20–40 KB on the
reliable stream; nothing must be bit-exact). For us the cell list wins: it removes cross-machine determinism from d
entirely, and the bandwidth is a rounding error next to the body stream. The impact form only pays off for
refractures far larger than our caps.

**e.** All of c's follower paths (every peer follows for objects it does not own) plus ownership in every decision:
who runs a structure's stress check (a structure has one owner; two players hitting one building thrash it), who
resolves a truck hitting a wall owned by someone else (Fiedler's authority and ownership sequence numbers with the
host as arbiter [5]; Roblox assigns by proximity and warns the owning client can send anything [7]). The peer who
triggers a collapse simulates and uploads it from a home connection. The most decision paths of any model and the
hardest to debug; only its driver-owned-vehicle piece is worth keeping (as an option under c/d).

**f/g3.** a plus per-object hashes and a per-object apply path: set a body's transform, velocity and type; set a
structure's bond set and let split.c recompute components; set a link's, wheel's, rig's, pool's state. That apply
path is exactly the late-join/save "rebuild an object from its image" primitive, so repair costs nothing extra once
H5's primitive exists. Repair is as-built, not bit-exact (warm starts, sleep timers and contact state are lost), so a
repaired island that is still moving mismatches again next check and is repaired again until it sleeps; while awake
it is effectively host-synced (Fiedler's state synchronization: run both sides, send corrections [4]). That is the
intended failure mode: a peer that diverges often (an emulated ARM64 client, say) degrades toward a follower instead
of being kicked. What it needs beyond a: stable object ids across peers (pieces and bodies are created in tick order
on every peer, so ids can be allocated deterministically), hashes per structure/object instead of per world, a repair
cadence (hash exchange at 5–10 Hz), and the rule that a repair is applied at a tick boundary on all peers including
the host, so the repaired image is the new shared truth.

**g1 (the character).** The canonical player is a dynamic body in the deterministic world driven by committed
inputs; the local view runs a kinematic mover re-applied from the canonical state each tick with the pending inputs
(Factorio's latency state: reset to the game state, re-apply the latency queue, render both [6b]). Cost per frame: k
mover casts ≈ k × 10 µs. Its effects enter as tick-stamped commands (tool rays resolved at apply time, as the sandbox
already records them; pulls; pushes). Knockouts and dents apply to the canonical body and the predictor rebases onto
it, so the no-HP impairment system stays inside the deterministic world. What it does not do: predicted contact with
moving debris is against current-tick debris (plausible over causal). Assemblies attached to the character
(the hauling game's cart) follow the canonical body, so they trail the predicted view by the input delay (~100 ms × 3 m/s
walking ≈ 0.3 m); if that reads badly, render the cart from the predicted frame or accept it. Vehicles stay in the
deterministic world with input delay: Factorio excludes car driving from latency hiding for the same reason [6a]; for
the rigging-race game the trucks are rigging tools, not racers, and AoE-class delays of 100–250 ms were shipped for direct unit control
[13]. If a driven vehicle needs immediacy later, the fix is a client-side predicted vehicle (Rocket League's shape,
unverified this session) or driver ownership (e), both as bolt-ons, not model changes.

## 5. Judging the criteria

**Bit-exactness (question 1).** Lockstep needs the x64 triad (MSVC, clang, gcc) bit-exact, which Box2D v3's evidence
and Box3D's design make plausible and T3 must prove; ARM64 and emulated-x64 peers are where H1/H6 decide whether
lockstep is "with repair" or "with kicks". State sync needs nothing bit-exact, which is the strongest argument for d.
Fixed point everywhere (T4) buys nothing that pinned flags plus repair does not, at 2–4x cost and the loss of Box3D
(H3); fixed point inside the fracture function alone matters only for d's impact-command variant, which we would not
pick anyway.

**CPU and the slowest peer (question 2).** Lockstep's floor is the whole world on the weakest machine: town is fine
(~7–10 ms), `siege` and `keep`-class scenes (17 ms and 6 ms at 1 worker on the i7) are not on a toaster at 60 Hz
unless the session's budgets are lowered. Because every budget is a count, the ladder degrades gracefully and
identically on all peers (fewer full debris, more scrap and ghosts); the lobby sets one "quality" for the session,
chosen for the weakest peer, and AoE-style speed control or Factorio-style skipped ticks handle transient lag
[13][6a]. State sync's floor is rendering plus a mover, the best possible; its ceiling is the host, who must hold the
whole world and the packet building. For 4p among friends lockstep's floor is acceptable; at 12p with one potato it is
the model's main risk, mitigated by f's degrade-to-follower path.

**Bandwidth (question 3).** The table's arithmetic is the deciding number at 12p: 11–32 Mbit/s of host upstream for
the friendslop collapse under state sync against 0.09 Mbit/s under lockstep. At 4p state sync fits in ~3–10 Mbit/s.
On large maps with spread-out players, per-client relevancy lets d send each client only its visible awake set
(~200–400 bodies ≈ 0.3–0.6 Mbit/s), which is why d is still the right second choice for the rigging-race and hauling games; for the
friendslop building, everyone sees the collapse and the worst case stands.

**Latency (question 4).** Answered under g1 above: the pattern (lockstep world, locally predicted character, effects
as tick-stamped commands) is shipped by Factorio for its character and explicitly not for its vehicles [6a][6b];
Photon Quantum's prediction culling is the same idea for a 3D physics sim (predict an area around the local player,
verified frames elsewhere), but the manual was behind a bot wall this session, so treat that description as
unverified; per-object resimulation in Unreal [12] and predicted-vs-interpolated ghosts in Unity [11] are the
state-sync-side relatives. None of these forbids a destructible dynamic player body; the canonical body stays in the
deterministic world in every variant.

**Late join, migration, saves (question 5).** Two images exist. The as-built image (pieces' geometry or the fracture
log that regenerates it, transforms, bonds, links, vehicles, rigs, pools) is ~100 KB plus a ~36 KB fracture log for
town, rebuilds in under a second, and is enough for c/d/e and for saves everywhere. The bit-exact image (Box3D's
internal snapshot, ~6 MB estimated for town from bodies, contacts with manifolds and warm starts, shapes and trees;
plus ours) is what pure lockstep needs for a joiner to match from its first tick, and it is same-build only today
(layout hash), so cross-compiler sessions cannot use it as is. Command-log replay from the session start is cheap in
bytes (Teardown reports the command stream is far smaller than its 30–50 MB scene files [1]) but not in CPU: one
minute of town at 60 Hz is 3600 × 2–9 ms = 7–32 s on 8 workers, so Teardown bounds the buffer and disables join in
progress past it [1]; Factorio sends the whole save and has the joiner simulate up to 400 ticks to catch up [6b]. So a
snapshot primitive is unavoidable, and with f's repair the as-built image suffices even for lockstep: the joiner
converges island by island as things sleep. Host migration is where lockstep wins outright (every peer already holds
the state; Factorio's star topology still needs the host only as a relay [6a]).

**Desync (question 6).** Lockstep's risk is real and its cost in pure form is the session (Factorio auto-quits after
three [6c]; AoE spent most of its schedule on it [13]). Per-object hashing turns it into a named object at a named
tick, and the command log plus last snapshot reproduce it headlessly in `lpf_bench`, which is the agent-friendly
property this project wants: a field desync becomes a failing test. d has nothing to debug but snapping.

**Large maps (question 7).** Lockstep peers simulate everything everywhere; roadmap 12's zones must derive activity
from shared state (they already plan to), which cuts CPU identically on all peers but never bandwidth (inputs do not
scale with the world anyway). d gets both host-side zones and per-client relevancy.

**Follower mode (question 8).** Under d the destruction command should be the cell list, not the impact; stress,
links, supply, rigs, wheels and freezing become host decisions shipped as small commands and applied by a client that
never runs the decide phases. The core would grow: an "apply" entry point per system, kinematic proxies for pieces,
and the discipline that no client-side code reads Box3D results to decide anything. Under g3 the same apply entry
points exist but are used only for repair and late join, which is the smaller and more testable surface.

**Box3D (question 9).** a/f/g3: keep the vendored engine, pin flags in our CMake too (`-ffp-contract=off`, no
fast-math, SSE2 path everywhere), audit the ±0 tie in `_mm_min_ps/_mm_max_ps` and NEON NaN handling for ARM64 peers
(brief), and patch the snapshot to be portable (canonical order for contacts and warm starts, layout-independent
encoding) so late join can be bit-exact across builds; GPU physics stays off the shared path (H4 holds). c/d/e: keep
as is, and the host may even run a different or GPU-backed solver since nobody else simulates; that is the only route
by which GPU rigid bodies become eligible. No model justifies a rewrite; fixed-point everywhere is only forced if T3
finds the x64 triad cannot be made bit-exact, which the Box2D v3 evidence argues against.

## 6. Choices, hypotheses, flips

**First choice: g3, convergent lockstep.** Lockstep with input delay (2–3 ticks plus a jitter buffer, Fiedler's
playout delay [2]), inputs relayed by the host as one merged packet per tick (Factorio 1.0's scheme [6a]), the local
character predicted per g1, per-object hashes exchanged at 5–10 Hz, host repair of a mismatched object at a tick
boundary, session-wide budgets chosen for the weakest peer, and one serialize/rebuild primitive used for late join,
migration, saves and repair. Depends on: H1 (Box3D bit-exact with pinned flags across the x64 triad; ours fixable),
H5 (the primitive), H2 (confirmed below); H6 only decides how often ARM64 peers get repaired. Experiments that flip
it: (1) T8's two-process cross-compiler run (MSVC vs clang builds of `lpf_bench` on the bench scripts): if hashes
diverge after our own hazards are fixed and the cause is inside Box3D and not flag-fixable, lockstep falls to d;
(2) a toaster step measurement (`siege`, `keep`, `town` at 1 worker on a low-end CPU) with the lowest session budgets:
if the average exceeds ~12 ms at the low rung, 12p lockstep needs the follower fallback and d's case strengthens;
(3) `b3SerializeWorld` size and cost on town, and whether a restore is bit-exact across MSVC and clang builds: this
decides whether late join is bit-exact or convergent, not the model.

**Second choice: d, the Teardown hybrid with cell-list commands.** Host-authoritative bodies with a per-client
priority accumulator and ~1 Mbit/s per client, destruction as reliable cell-list commands, host-decided structural
events replicated as commands, clients in follower mode with a predicted kinematic character, driver-owned vehicles
optional. Depends on: H5 (the as-built image), H4 partially reopened (GPU on the host becomes eligible). Wins if
experiment (1) fails or if a large-map, spread-out 12p session matters more than the friendslop building. Loses on
host upstream at 12p in one building, and on engineering surface (every reactive system gains a second path).

**Verdicts.**
- H2: confirmed with an amendment. Lockstep is right for 4p; at 12p pure lockstep (a) is not acceptable because a
  single divergence or slow peer costs the session, while lockstep with per-object repair (f/g3) is. The local
  character predicted outside the deterministic world with tick-stamped effects is a shipped pattern (Factorio); the
  hybrid does lose on player-host upstream at 12p (11–32 Mbit/s vs 0.09).
- H5: confirmed and sharpened. The most valuable outcome is the serializable, command-logged world with per-object
  hashes, and the same per-object serialize/apply primitive serves late join, host migration, saves, repair, and (if
  ever wanted) rollback. Every model needs at least its as-built half.
- H1: not this track's to prove; the Box2D v3 evidence [9] and Box3D's design make it plausible on x64, and the
  residual hazards listed are ours. H3: supported by exclusion (no model needs fixed point; d's impact-command variant
  would, and we would not choose it). H4: holds under g3; partially reopened under d (host-only GPU). H6: open;
  under g3 its answer sets the repair rate for emulated peers, not the model.

## 7. Sources

Accessed 2026-09-29 unless noted. Numbers not from these are marked in the text as estimates.

1. Dennis Gustafsson, "Teardown multiplayer", 2026-03-13, https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html (deterministic fixed-point destruction as reliable commands; host-synced bodies with per-client priority queue, ~1 Mbit/client, visible snapping under load; late join by replaying a bounded command buffer; 30–50 MB scene serialisation).
2. Glenn Fiedler, "Deterministic Lockstep", 2014, https://gafferongames.com/post/deterministic_lockstep/ (inputs only, bandwidth independent of object count; 100 ms playout delay; up to 4 sim frames per render frame; cross-platform float determinism "almost impossible to guarantee").
3. Glenn Fiedler, "Snapshot Compression", 2015, https://gafferongames.com/post/snapshot_compression/ (901 cubes, 17.37 Mbit/s raw at 60 Hz to a 256 kbit/s target; ~26.1 + 23.3 bits per changed cube after delta encoding; ~15 kbit/s at rest).
4. Glenn Fiedler, "State Synchronization", 2015, https://gafferongames.com/post/state_synchronization/ (priority accumulator, ~64 updates per packet in 256 kbit/s, quantise on both sides, smoothing of corrections).
5. Glenn Fiedler, "Networked Physics in Virtual Reality", 2018, https://gafferongames.com/post/networked_physics_in_virtual_reality/ (authority and ownership sequence numbers, host as arbiter, 4 players, <256 kbit/s per player, co-op only).
6. Factorio: (a) FFF-147 "Multiplayer rewrite", 2016, https://www.factorio.com/blog/post/fff-147 (star topology with merged input packages, lag isolation, join without freeze); (b) FFF-302, 2019, https://www.factorio.com/blog/post/fff-302 (game state vs latency state re-applied each tick; catch-up of up to 400 ticks); (c) wiki "Multiplayer", https://wiki.factorio.com/Multiplayer (lockstep; latency hiding since 0.12 for character movement, not vehicles or shooting; UDP; quit after 3 desyncs; 100+ players seen).
7. Roblox, "Network ownership", https://create.roblox.com/docs/physics/network-ownership (proximity-based automatic ownership, `SetNetworkOwner`, anchored parts stay server-owned, owning clients can send bad data).
8. Valve, GameNetworkingSockets `steamnetworkingtypes.h`, https://github.com/ValveSoftware/GameNetworkingSockets (SendRateMin/Max default 256K bytes/s; 512K send buffer and max message).
9. Erin Catto, "Determinism" (Box2D v3), 2024-08, https://box2d.org/posts/2024/08/determinism/ (cross-platform tests on x64 and ARM with MSVC/GCC/clang; no FMA, own trig; no rollback determinism or snapshot support).
10. Rapier, "Determinism", https://rapier.rs/docs/user_guides/rust/determinism (local determinism by default; `enhanced-determinism` for cross-platform, IEEE 754-2008 targets, no `simd8`).
11. Unity, Netcode for Entities 1.0, "Physics", https://docs.unity3d.com/Packages/com.unity.netcode@1.0/manual/physics.html (predicted vs interpolated ghosts, separate physics worlds, proxies for interaction, prediction batching).
12. Epic, "Networked Physics Overview", https://dev.epicgames.com/documentation/en-us/unreal-engine/networked-physics-overview (Resimulation per actor with an RTT of cached state vs Predictive Interpolation; cost notes).
13. Paul Bettner and Mark Terrano, "1500 Archers on a 28.8", 2001, https://www.gamedeveloper.com/programming/1500-archers-on-a-28-8-network-programming-in-age-of-empires-and-beyond (200 ms turns, two-turn delay, speed control by the slowest machine, out-of-sync as the main schedule risk, 250–500 ms acceptable).
14. Photon, "Quantum" product page, https://www.photonengine.com/quantum (deterministic predict/rollback, input-only networking through Photon Cloud, 32 players in Stumble Guys, 30/60 Hz ticks). The Quantum manual (prediction culling, late-join snapshots, checksums) was behind a bot wall this session: unverified; a fetch of https://doc.photonengine.com/quantum/current/manual/prediction-culling from a browser would verify.
15. Wikipedia, "List of countries by Internet connection speeds" citing Ookla, March 2026, https://en.wikipedia.org/wiki/List_of_countries_by_Internet_connection_speeds (fixed median downloads: US 310, UK 172, DE 104 Mbit/s; uploads not listed). Home upload figures in this report (5–20 Mbit/s for cable/DSL hosts) are the brief's assumption: unverified; Ookla's Global Index or the FCC "Measuring Broadband America" report would verify (both refused fetches).
16. Jolt Physics README, https://github.com/jrouwe/JoltPhysics (claims deterministic simulation with inputs-only replication; its "Deterministic Simulation" section with the cross-platform flag was not retrievable: unverified).
17. Repo: `bench/baseline.json` (commit cce5ea5, 2026-09-29) for all step, piece, body and contact counts; `include/lpf/lpf.h:134-168` (session-affecting budgets), `:598-602` (`lpWorld_Hash`, `lpWorld_HashStress`); `extern/box3d/src/world_snapshot.c:80-86, 1016, 1133, 1159` (internal snapshot and layout hash); `src/poly.h:82-94` (`lpShape`); the M7 brief for Box3D internals, our hazards and the step order.

Not verified this session and cited only as context: Halo Reach's "I Shot You First" (GDC 2011) and Rocket League's
"It IS Rocket Science" (GDC 2018) talks (host-authoritative with client prediction of the local player or car; the
vault timed out), The Finals' server-side destruction (widely reported, no primary page fetched), and Photon Fusion's
Host vs Shared modes (docs behind the same bot wall).
