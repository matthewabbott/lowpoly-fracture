# M7 T0: state audit (what must be identical, shipped or originated)

Track T0 of milestone 7, 2026-09-29. A code audit of the simulation state in `lpWorld` and Box3D, of every place a state
change is decided from a Box3D float result, of the inputs, the runtime ids and the state hash. Repo facts cite
`file:line` at HEAD `5445925` (source unchanged since `f00d48c`). Numbers come from a scratch probe described in
section 0; nothing in the repo was changed except this file.

## Summary

- **Box3D's own snapshot is bit-exact for "restore, then continue", except for one thing we add.** A probe snapshotted
  Box3D mid-destruction (`b3World_StartRecording` at tick 300 of the bench schedule), recorded our API calls and steps,
  and replayed them with Box3D's player. With our custom filter callback installed, every replay diverged within 1 to
  143 frames (town, keep, mech, track, pile). With the callback removed, all six scenes replayed 120 to 200 frames with
  every per-step state hash matching, including recording at 8 workers and replaying at 1. So warm starts, contact
  caches, sleep timers, islands, colouring, trees and id pools are all in `b3SerializeWorld`. The gap is state that
  lives outside Box3D: the filter callback, which reads our `lpWorld`, and `userData`, which the snapshot zeroes.
- **Snapshot sizes at the peak (the tick with the most pieces), measured.** Town rung, 7,582 pieces: Box3D 4.54 MB,
  plus 3.4 MB of hull geometry. A naive field-complete dump of our state is 8.4 MB. Together that is 16.4 MB raw,
  4.9 MB with gzip -9 and 3.65 MB with xz. Barrage, 16,339 pieces: 31 MB raw, 10.0 MB gzip. Keep, 4,231 pieces:
  13.8 MB raw, 5.4 MB gzip. At load everything can be rebuilt from the scene, so a join at the start costs nothing.
  Piece geometry dominates our side; the stress solver's cached systems dominate the keep's (1.9 + 1.7 MB).
- **Every reactive decision originates in a Box3D float.** The origination map (section 3) lists about 50 sites in 19
  command types. Measured rates per tick at the peak rungs:
  - impacts 2.5 to 11.5 (max 17)
  - freezes 2.5 to 19 (max 64)
  - wakes 1.3 to 14 (max 243)
  - stress breaks up to 4.4 (max 151)
  - body creations up to 26 (max 339)

  About 800 to 2,100 bodies change state every tick at the peaks.
- **Static things are the same on every peer; moving things are the host's.** Structures and frozen rubble never move,
  and ghosts land only on static shapes. So a "follower" client in a hybrid model can run these itself once the host's
  freezes carry exact poses: the fracture of a commanded piece, bond damage on structures, splits, tiers by volume,
  supply and pools, link bookkeeping, ghost flight and landing. It must not decide:
  - impacts from hits
  - freezes, wakes and kills
  - stress breaks (so it needs no stress solve at all)
  - link strain
  - detonations
  - the gait
- **`lpWorld_Hash` costs about as much as a whole step at the peak.** It runs byte-wise FNV-1a over every vertex:
  2.9 to 4.2 ms at 7.6k pieces and 6.5 ms at 16.3k. Box3D's own hash takes 0.2 ms. Piece geometry never changes after
  creation, so a digest stored per piece, word-wise mixing and dirty tracking (Box3D's move events already list the
  bodies that moved) would bring it to an estimated 0.05 to 0.2 ms (unverified).
- **Hardening items: 24 (section 8).** The main ones:
  - Freeze candidates, detonations and stress re-check requests are processed in Box3D's report order.
  - Wake queries use Box3D's fat AABBs with no exact test.
  - Ties in casts are broken by tree traversal order.
  - The stress budget depends on whether a system is cached, so a restore must ship megabytes of stress systems.
  - Bonds and wheels have no generation.
  - Box3D ids embed the world slot.
  - The fracture seed uses slot indices.
  - The sandbox recorder writes `%.6f`.
  - Some decision state is not hashed: limb depth, wheel spin, detonator armed flags, ghost landing plans.
- **H5 (a serializable, command-logged world is the most valuable outcome) is supported.** Every model needs a
  snapshot, and on the Box3D side it already exists and is proven bit-exact. The missing half is ours (no serializer,
  heap pointers, slot free lists) and it is well-defined by the tables below.

## 0. Method

- **Code read:**
  - `src/world.h`, `step.c`, `world.c`, `impact.c`, `split.c`, `stress.c` (scheduling, loads, judging), `link.c`,
    `wheel.c`, `rig.c`, `gait.c`, `supply.c`, `debris.c`, `include/lpf/lpf.h`
  - the sandbox's event and record path (`app/sandbox/main.cpp`)
  - Box3D headers and excerpts: `world_snapshot.c`, `physics_world.h`, `body.h`, `contact.h`, `solver_set.h`,
    `island.h`, `constraint_graph.h`, `id_pool.h`, `broad_phase.h`, `recording.c`
- **Probe:** a scratch C program in the session scratchpad (`m7/t0probe/probe.c`, not committed).
  - It compiles Box3D, `src/` and `scenes/` from source with the repo's flags (`/O2 /fp:precise /MT`).
  - It runs each bench rung for 600 ticks at 8 workers. The bombardment period is 12 for town, keep and yard, 3 for
    barrage (town), 4 for siege (keep), and 30 for track and mech.
  - The final `lpWorld_Hash` equals `bench/baseline.json` for every rung: town `39824554bc44b0f7`, barrage
    `c48a19d345fd6be9`, keep `d4574cf06c5718fc`, siege `b8baed95b8913c82`, track `f640f9fe61cf977a`, mech
    `7541c412fbe7a82f`, yard `d4b4a0b5acc2272f`. So the numbers describe the real engine.
- **Measured at load and at the peak tick:**
  - `sizeof` of every struct
  - element counts and slot counts
  - geometry and hull bytes
  - stress systems
  - Box3D internals, read through its internal headers
  - Box3D's snapshot, through the public `b3World_StartRecording` and `b3World_StopRecording`
  - a field-complete dump of our state, with pointers zeroed
  - gzip -9 and xz -6 of both
  - hash times (best of 5)
  - per-tick event rates
- **Side effects checked:** starting and stopping a recording mid-run does not perturb the simulation. The final hash
  matched a run without it in all seven rungs.
- **Scale correction:** the brief's "town peaks at about 10.8k pieces" comes from an older `build/bench-town.json`. The
  current town rung peaks at 7,582 pieces and barrage at 16,339. The keep has 1,973 pieces and 6,296 bonds at load,
  and 4,231 pieces and 9,848 bonds at its peak. Bonds are 3.1 to 3.4 per structure piece, but only 0.2 to 0.45 per
  piece overall at the peaks, because debris has no bonds.

## 1. Our simulation state (Q1)

Classes (for a late joiner or a restore):

- **R**: rebuilt from the scene build (seed and content) or from the session config.
- **R\***: rebuilt without physics from a log of state changes (the host's command stream in a hybrid). In lockstep it
  is S, because only a full re-simulation reproduces it.
- **S**: history-dependent; must be shipped to be bit-exact.
- **D**: derived cache, rebuilt from S and R. The rebuild must be bit-identical.
- **C**: cosmetic, never fed back.

"Scratch" means within-step only. It is empty or reset at a step boundary, and snapshots are only taken between steps.

### 1.1 Measured sizes

`sizeof` (MSVC x64):

| struct | bytes | struct | bytes | struct | bytes |
|---|---|---|---|---|---|
| lpPiece | 296 | lpStressSystem | 168 | lpDeferredJob | 56 |
| lpBond | 100 | lpStressEdge | 92 | lpImpactDef | 40 |
| lpBody | 432 | lpBlock6 | 144 | lpPendingBlast | 12 |
| lpLink | 408 | lpVec6 | 24 | lpPool / lpDetonator | 28 / 20 |
| lpWheel | 320 | lpStressReduced | 256 | lpLostWheel | 68 |
| lpVehicle | 184 | lpShape header | 80 (+12/vertex, 24/face, 1/index) | lpBodyRef | 8 |
| lpRig | 2,240 (incl. 8 × lpLimb 256) | lpFractureJob | 6,504 (+135 KB cell-bond buffer) | lpWorld | 1,160 |

Element counts ("load / peak"):

| | town | barrage | keep | siege | track | mech |
|---|---|---|---|---|---|---|
| pieces | 1,150 / 7,582 | 16,339 | 1,973 / 4,231 | 8,766 | 82 / 593 | 81 / 85 |
| geometry, avg B/piece | 353 / 383 | 414 | 344 / 465 | 526 | 341 / 353 | 343 |
| our cached hulls | 0.75 / 3.41 MB | 9.46 MB | 1.26 / 3.67 MB | 8.65 MB | 0.11 MB | 0.05 MB |
| live bonds (slots) | 3,612 / 3,306 (3,872) | 4,355 (4,666) | 6,296 / 9,848 (9,992) | 14,012 | 116 / 27 | 52 / 47 |
| bodies alive | 26 / 6,142 | 14,198 | 2 / 1,323 | 4,699 | 20 / 572 | 45 / 54 |
| of which structure / debris / rubble / ghost / scrap | 26 / 1,192 / 1,756 / 530 / 2,638 | 26 / 1,600 / 2,714 / 1,255 / 8,603 | 2 / 636 / 660 / 0 / 25 | 2 / 1,558 / 1,176 / 480 / 1,483 | 9 / 47 / 74 / 66 / 376 | 4 / 27 / 23 / 0 / 0 |
| stress systems (bytes) | 25 (574 KB) / 25 (424 KB) | 25 (348 KB), 1 solving | 1 (1.23 MB) / 1 solving (1.87 MB) + reduced 1.67 MB | 2.62 + 2.63 MB | 7 (10 KB) | 13 (7 KB) |
| links / vehicles / rigs / pools / detonators | 0 | 0 | 0 | 0 | 3 / 3 / 0 / 0 / 3 | 18 (4 wheels) / 1 / 1 / 1 / 1 |
| our heap, approx | 2.8 / 17.7 MB | 38.5 MB | 4.7 / 23.3 MB | 37.7 MB | 2.2 MB | 1.3 MB |
| Box3D heap (`b3GetByteCount`) | 3.2 / 16.3 MB | 28.9 MB | 4.4 / 16.8 MB | 34.7 MB | 1.7 MB | 1.5 MB |

Our heap at the town peak includes about 4.5 MB of fracture job scratch: roughly 32 job slots of 142 KB, almost all of
it the fixed `LP_MAX_CELL_BONDS` cell-bond buffer (world.h:544, impact.c:520-533). That is scratch, not state, but it
is memory on a "toaster".

Snapshot sizes (the Box3D snapshot through `StartRecording`, and the hull registry appended by `StopRecording`):

| rung | ours naive raw | gzip -9 | xz -6 | Box3D snapshot | + hull registry | gzip -9 | xz -6 | serialize |
|---|---|---|---|---|---|---|---|---|
| town load | 1.17 MB | 0.37 MB | 0.27 MB | 0.41 MB | 1.14 MB | 0.22 MB | 0.13 MB | 1.5 ms |
| town peak (7,582) | 8.42 MB | 1.92 MB | 1.45 MB | 4.54 MB | 7.94 MB | 3.00 MB | 2.20 MB | 20 ms |
| barrage peak (16,339) | 18.7 MB | 4.74 MB | 3.55 MB | 7.16 MB | 12.6 MB | 5.28 MB | 3.78 MB | 18 ms |
| keep load | 1.98 MB | 0.62 MB | 0.46 MB | 0.68 MB | 1.95 MB | 0.24 MB | 0.11 MB | 2.3 ms |
| keep peak (4,231) | 5.41 MB | 2.04 MB | 1.60 MB | 4.73 MB | 8.42 MB | 3.37 MB | 2.46 MB | 10 ms |
| siege peak (8,766) | 11.6 MB | 4.60 MB | 3.61 MB | 9.44 MB | 16.2 MB | 6.96 MB | 5.01 MB | 15 ms |
| track peak | 0.66 MB | 0.12 MB | 0.09 MB | 0.14 MB | 0.22 MB | 0.07 MB | 0.05 MB | 0.3 ms |
| mech peak | 0.10 MB | 0.02 MB | 0.02 MB | 0.07 MB | 0.11 MB | 0.03 MB | 0.03 MB | 0.2 ms |

What makes up the naive dump at the town peak:

| part | bytes | how it breaks down |
|---|---|---|
| geometry | 2.91 MB | 70,169 vertices, 51,878 faces, 213,766 indices |
| piece structs | 2.27 MB | |
| bodies | 2.83 MB | 432 B each, mostly unused fields on ghosts and scrap |
| bonds | 0.34 MB | |

A lean format at the town peak is estimated (unverified) at about 3.9 MB for our side:

| part | bytes | how it is reached |
|---|---|---|
| geometry | 2.30 MB | shape headers are D |
| piece fields | about 0.6 MB | stress fields only on the 1,036 structure pieces |
| bonds | 0.29 MB | 89 B of state each |
| bodies | about 0.56 MB | |

Add Box3D's 4.54 MB with hulls rebuilt from our shapes instead of shipped (8.4 MB in all, against 16.4 MB naive).
Compression of the lean format was not measured.

### 1.2 World-level state and queues (owner: world.c, step.c and the module named)

| item | owner | bytes/elem | town (load/peak) | keep (load/peak) | class | notes |
|---|---|---|---|---|---|---|
| `def` (lpWorldDef: seed, budgets, thresholds, scales) | world.c:119-154 | about 140 | 1 | 1 | R (session) | Must match on every peer except `workerCount` and `debugLog`; the Box3D `b3WorldDef`, dt and substeps too. |
| `tick` | step.c:402 | 8 | 1 | 1 | S | In fracture seeds (impact.c:85), ages, rate limits, cast budget rotation (wheel.c:630), dust hashes. |
| `pieceSerial` | world.c:847, impact.c:338 | 8 | 1,150 / 8,230 | 1,973 / 4,543 | S | Seeds new pieces; the ghost tumble uses it (impact.c:369-377). |
| `impactSerial` | impact.c:696, stress.c:930 | 4 | 0 / 2,984 | 0 / 1,451 | S | Compared with `bond.lastImpact` and `link.lastImpact` so a deferred job does not damage twice. |
| `changeSerial` | world.h:790-793 | 4 | 8,414 / 46,765 | 14,565 / 147,986 | S | Relative to `piece.changed` and `piece.accepted` (stress seeds). |
| `lastTimeStep` | step.c:333 | 4 | 1 | 1 | S | Turns contact impulses into forces (stress.c:184, 217) and velocities into accelerations (stress.c:107-110). |
| `stamp` (+ `piece.mark`, `body.stamp`) | many | 4 | – | – | scratch | Safe to zero together on restore; never zero one without the other. |
| `freeze candidates` (+ `body.freezePending`) | step.c:50-84 | 4 | 0 at peak | 0 | S | The order matters: the 64-per-step cap is taken in list order, which is Box3D's move-event order. |
| `dirtyBodies` (+ `body.dirty`) | world.c:680-688 | 4 | 1 | 1 | S | The order becomes the stress queue order and so the budget order (stress.c:1527-1615). |
| `impacts` | world.c:938-941 | 40 | inputs | inputs | S (inputs) | API-queued for the next step. |
| `nextImpacts` | impact.c:497, 1020 | 40 | 2 / 16 (barrage) | 16 | S | Collision impacts and blasts found during a step, processed next step. |
| `pendingDestroy` | impact.c:498-499 | 12 | 0 | 0 | S | Pieces that detonated, sorted before use (step.c:272-285). |
| `deferred` | impact.c:588-589, stress.c:931 | 56 | 0 | 0 | S | Fracture jobs over budget and slender snaps. |
| `lostWheels` | wheel.c:217-219 | 68 | 0 | 0 | S | Spawned next step (wheel.c:225-253). |
| `audits`, `audit`, `calmSteps` | stress.c:1376-1385, 1686-1725 | 8 | 0 | 0 | S | The order of provisional structures; the calm counter decides when an audit runs. |
| `pulls`, `blows` | step.c:160-164, debris.c:627-631 | 36 | inputs | inputs | S (inputs) | Queued per tick, consumed next step. |
| `pendingWakes`, `forces`, `stressAgain`, `stressQueue`, `stressJobs`, `jobs`, `scratch*` | split.c:127, step.c:304-306, stress.c | – | 0 | 0 | scratch | Filled and consumed within one step (`pendingWakes` can hold wakes left by `lpWorld_SettleStructures` at load). |
| `supplyDirty` | world.h:738-741 | 1 | 1 | 1 | S | Or force an update on restore: it is idempotent when the topology is unchanged (supply.c:159-166). |
| loose-debris grid (`gridHeads` 4,096 slots, `body.grid*`) | debris.c:17-143 | 16 KB + 12/body | 1 | 1 | D | Rebuilt from ghost and scrap positions; queries are sorted (debris.c:139-142). |
| `particles` | world.c:598-614 | 36 | 43 / 28 | 46 | C | Dust, emitted per step. |
| `stats` | step.c:233-266 | – | – | – | scratch / C | Two within-step reads feed logic: `footCasts` (gait.c:349, 520) and `stressWaiting` (stress.c:1688). |
| `stressOracle*` | stress.c:1326-1342 | – | – | – | C (tests) | |

### 1.3 Pieces (`lpPiece`, 296 B; owners world.c, impact.c, stress.c, supply.c)

Counts: town 1,150 / 7,582; barrage 16,339; keep 1,973 / 4,231.

| field(s) | bytes | class | notes |
|---|---|---|---|
| `shape` (vertices, faces with exact planes, indices; the header's bounds, centroid, volume and radius) | 80 + arrays; about 383 avg at the town peak | S (R\* by fracture replay) | Immutable after creation. The face planes are state: fractures clip against them (impact.c:52-60), so they cannot be recomputed from the vertices. The header is D (poly.c:570-602). |
| `hull` | about 770 avg | D | `lpShape_CreateHull` of the shape; Box3D also holds its own clone in its hull database. |
| `shapeId` | 8 | S (Box3D handle) | Valid after a Box3D restore; `world0` must be patched (section 5). |
| `bonds` list, `links` list | 4 per entry | S | The order is history ("keep order", world.c:483-485): it sets the flood-fill order, component order and new-body order. |
| `body`, `generation`, `nextFree` | 12 | S | The free list is LIFO (world.c:192-237). |
| `material`, `joint`, `depth`, `anchored`, `anchorPlane`, `axis`, `userId`, `part`, `tag`, `carries`, `sources`, `needs` | about 50 | R / R\* | Part identity, copied to cells (impact.c:333-352). |
| `sourceShare` | 4 | S (R\*) | A float split by volume at each fracture (impact.c:346). |
| `detonator`, `pool` | 8 | S | Indices into append-only arrays. |
| `stressX`, `stressLoad`, `stressResidual` | 72 | S | The warm start, the sampled loads (kept while solving and the baseline for "loads changed", stress.c:249-262) and the accepted residual. |
| `stressR`, `stressP` | 48 | S while its structure is solving | Reloaded into a continuing solve (stress.c:564-574). |
| `strain`, `slenderRho`, `slenderAt`, `slenderDepth`, `cluster`, `changed`, `accepted` | 28 | S | Creaking re-judges from `slender*` with no solve (stress.c:776-780). |
| `solveSlot` | 4 | D | Set by the build. |
| `supply[8]` | 8 | D | Recomputed from topology and pools (supply.c:57-168). |
| `mark` | 4 | scratch | |
| `color` | 4 | C | Render only. |
| `seed` | 4 | dead | Read once at creation (impact.c:338, 370); drop it. |

Essential per piece: about 100 B for debris pieces and about 220 B for structure pieces, plus the geometry.

### 1.4 Bonds (`lpBond`, 100 B; world.c, impact.c, stress.c)

Counts: town 3,612 / 3,306; barrage 4,355; keep 6,296 / 9,848; siege 14,012.

| field(s) | class | notes |
|---|---|---|
| `a`, `b`, `alive`, `nextFree` | S | LIFO free list with **no generation** (world.c:239-255, 506-508). |
| `area`, `centroid`, `normal`, `h1`, `h2`, `joint` | S (R\*) | The contact patch at creation. Cell bonds come from Voronoi face tags (impact.c:166, 417-435), which are normalised away afterwards, so recomputing them with `lpShape_Contact` would not reproduce them. |
| `health`, `strength` | S | Blast damage (impact.c:667-672). |
| `strain`, `rho`, `force`, `moment` | S | Rejudging after a blast uses the stored force and moment (stress.c:763-768). |
| `lastImpact` | S | |

Essential: about 89 B.

### 1.5 Bodies (`lpBody`, 432 B; world.c, split.c, stress.c, debris.c, step.c)

Counts: town 26 / 6,142; barrage 14,198; keep 2 / 1,323. Box3D bodies: town 26 / 2,974; barrage 4,340; keep 2 / 1,298.

| field(s) | class | notes |
|---|---|---|
| `id` | S (Box3D handle) | Null for ghosts and scrap. |
| `pieces` list | S | The order is history (split.c:198-209, impact.c:272-281). |
| `alive`, `generation`, `nextFree`, `kind`, `tier`, `createdTick` | S | `createdTick` gates freezing (step.c:73-74) and ranks budgets (debris.c:787). |
| `volume` | S | Accumulated incrementally (world.c:462, impact.c:282, split.c:186, debris.c:188, step.c:147). A history-dependent float; it ranks budgets and decides tiers (split.c:219-225). |
| `gravityScale`, `inertiaRadius`, `solveStress` | S | |
| `dirty`, `freezePending` | S | Together with their lists. |
| `unsettled`, `solving`, `creaking`, `strainedLastCheck`, `reloadLoads`, `rejudge`, `stressSteps`, `solveRz`, `solveNodes`, `solveEdges`, `solveClustered`, `provisional`, `auditing`, `provisionalTick`, `clusters`, `clusterStamp`, `meterRounds` | S | The stress scheduler's state. |
| `topology`, `solveTopology`, `splitTopology`, `splitChecked` | S | Counters compared for equality. They could be renumbered consistently, but shipping them is simpler. |
| `system` (lpStressSystem) | S\* | 424 KB (town peak) to 2.6 MB (siege). The `cached` flag changes the stress budget (item 8.6), and a clustered solve in progress keeps its `x` here. |
| `reduced` (lpStressReduced) | S while a clustered solve continues | 1.67 MB (keep) and 2.63 MB (siege); `y`, `r` and `p` live only here (stress.c:1135). |
| `hitCheckTick`, `loadCheckTick`, `joltTick` | S | Rate limits on stress re-checks. |
| `stressPin`, `relief*`, `stepV[2]`, `stepOmega[2]`, `stepTick`, `stepPair`, `hitPoint`, `hitMaterial`, `hitTick` | S | Inertia relief for moving bodies that solve their stress. |
| ghosts and scrap: `com`, `q`, `v`, `omega` | S | Our own integration. |
| ghosts and scrap: `localCenter` | D | Rebuilt from the pieces. |
| ghosts and scrap: `planTicks`, `landIn`, `landPoint`, `landNormal`, `sinkTicks` | S | Not hashed. |
| `linkStamp` | D | Rewritten each step before use (link.c:647-675). |
| `stamp`, `grid*` | scratch / D | |

Essential: about 60 B for Box3D-backed debris and rubble (the rest is in Box3D), about 110 B for ghosts and scrap, and
about 150 B for structures, plus the piece list.

### 1.6 Stress systems (solve.h:36-54, stress.c)

| item | bytes | town peak | keep peak | class | notes |
|---|---|---|---|---|---|
| system (nodes, edges 92 B, 6 vectors of 24 B per node, 144 B blocks, incidence, rho, node scales and arms) | about 640 B per node | 25 systems, 424 KB | 2,845 nodes, 1.87 MB | S\* / D | A non-clustered solve is rebuildable from the pieces (x from `stressX`, stress.c:511-519; r and p from the pieces, 564-574; f from `stressLoad` and the relief). But whether it is cached decides the budget charge (stress.c:1591-1616, 1316). |
| reduced system (partition, node references, P^T K P) | similar | 0 | 1.67 MB | S while solving | The correction's y, r and p exist nowhere else; a correction's `x` stays in `system.vectors` (stress.c:511, 520-523). |
| `stressJobs[]` (slender cuts, cluster groups) | 176 each + arrays | – | – | scratch | |

### 1.7 Links, wheels, vehicles, rigs, pools, detonators

| item | bytes | counts | class | notes |
|---|---|---|---|---|
| `lpLink.def` | 120 | track 3 live (15 slots), mech 18 (22 slots), yard 2 | R\* | Except `def.length`, which `SetRopeLength` changes (S). |
| `ends[2]` (piece, generation, frame), `wheel`, `alive`, `generation`, `nextFree` | | | S | LIFO with a generation (link.c:178-199). |
| `joint`, `builtOn[2]`, `anchor[2]` | 32 | | S (Box3D handles) | `builtOn` is compared every step to decide rebuilds (link.c:676). |
| `points[2]` | 24 | | S | Cached at the last awake step. Blast distances (impact.c:619-629) and rope and wheel ray hits (world.c:1198-1217) use them. |
| `force`, `torque`, `stressForce`, `stressTorque`, `recheckTick`, `utilization`, `strain`, `health`, `lastImpact`, `settle`, `angle` | | | S | `angle` is read by rigs, and it is stale while asleep. |
| `target`, `targetRotation` | | | S (inputs) | |
| `targetChanged` | | | S | Transient. |
| `motorApplied`, `appliedSpeed`, `appliedVelocity`, `appliedCap` | | | S | Shadows of Box3D's motor settings. After a restore they must match what Box3D holds, or a setter call and a wake happen (link.c:586-641). |
| `motorCap`, `feed` | | | S | Hashed. |
| `motorTorque` | | | C | Reported only. |
| `lpWheel` (320 B): `steer`, `spinSpeed`, `length`, `load`, `grounded`, `atStop`, `sliding`, `friction`, `contactPoint`, `contactNormal`, `groundPiece`/`groundGeneration`, `hub`, `hubVelocity`, `bodyOmega`, `stressForce`/`stressBody`/`recheckTick`, `sprungMass` | | track 12, mech 4 | S | A wheel over the cast budget keeps its last contact (wheel.c:660-664). `hub`, `hubVelocity` and `bodyOmega` seed the lost wheel. |
| `lpWheel.spin` | | | S | **Not hashed**, but it sets the lost wheel's orientation (wheel.c:217, 244). |
| `lpWheel.suspension`, `r`, `dir*`, `mass*`, `lambda*`, `groundVelocity` | | | D | Rebuilt every solve. |
| `lpWheel.slip` | | | C | |
| wheel free list | | | S | LIFO, **no generation** (wheel.c:72-101). |
| `lpVehicle` (184 B): `control` | | | S (input) | |
| `lpVehicle`: `controlChanged` | | | S | Transient. |
| `lpVehicle`: `links[16]` | | | S | |
| `lpVehicle`: `def`, `forward`, `up` | | | R\* | Append-only, never freed. |
| `lpRig` (2,240 B): `control`, `controlChanged`, `desired`, `idle`, `calm`, `crawling`, `stuck`, `stall`, `ableSeen` | | mech 1 | S | |
| `lpRig`: `body`, `height`, `pace` | | | D | Recomputed each step. |
| `lpRig`: `waiting` | | | C | |
| `lpLimb` (256 B): `planted`, `swinging`, `swingClock`, `castLate`, `grounded`, `liftoff`, `landing`, `hold`, `holdClock`, `arrived`, `reaching`, `reachWanted`, `reachPoint`, `q[3]`, `groundPiece`, `groundGeneration`, `recheckTick` | | 6 per mech | S | |
| `lpLimb`: `tipBody`, `tipGeneration`, `tipTopology`, `tipJoints`, `foot`, `depth` | | | S | A cache computed from link angles when the tip changes (rig.c:229-238). `depth` is **not hashed** but gates `able` (rig.c:412) and the stand height (gait.c:434). |
| `lpLimb`: `joints`, `rootBody`, `strength`, `attached`, `able`, `touching`, `residual` | | | D | Recomputed each step. |
| `lpLimb`: `neutral`, `defFoot`, `prox`, `gen`, `lower`, `upper` | | | R\* | |
| `lpPool` (28 B): `capacity`, `level`, `leak`, `seal`, `reach`, `step` | | mech 1 | S | |
| `lpPool.found` | | | scratch | |
| `lpDetonator` (20 B): `armed` | | track 3, mech 1, yard 4 | S | **Not hashed**. |
| `lpDetonator.def` | | | R\* | |

Classes used across Q1 in one line: config and part identity are R; geometry and topology are R\* (a host log
rebuilds them without physics, lockstep cannot); everything that integrates, accumulates or rate-limits over time is S.
The derived caches (hulls, supply, the grid, a non-clustered stress system, wheel solve scratch, limb capability) are D.

## 2. Box3D's hidden state (Q2)

`b3SerializeWorld` (world_snapshot.c:1016-1131) writes the following, in order:

- a header with a layout hash over every struct copied as raw bytes (41-86): same build only
- the world scalars
- the 6 id pools
- the solver sets
- the bodies, shapes (hull geometry interned into the recording registry) and fat AABBs
- contacts with their manifolds
- joints, sensors and islands
- the 3 broadphase trees and the pair set
- the 24 graph colours
- names

Byte estimates at the town peak (counts × sizeof, summing to the measured 4.54 MB):

| state | lives in | effect on future steps | covered | bytes (town peak) |
|---|---|---|---|---|
| world config: gravity, hit and restitution thresholds, contact hertz, damping, speed, recycle distance, max speed, sleep/warm-start/continuous/speculative flags, `stepIndex`, `splitIslandId`, `inv_h`, `inv_dt`, `endEventArrayIndex` | physics_world.h | every step | yes (468-493) | about 100 B |
| custom filter callback and context (our `lpCustomFilter`, world.c:308); friction, restitution and pre-solve callbacks; task system | physics_world.h | pair creation between light and full shapes | **no** | – |
| `userData` of bodies, shapes and joints (our index + 1: world.c:406, 667; link.c:119) | b3Body, b3Shape, b3Joint | our back-references from events and queries | **no, zeroed** (1057, 537-539, 1077) | – |
| 6 id pools (free array popped LIFO, `nextIndex`; id_pool.c:19-31) | physics_world.h | every new body, shape, contact, joint and island id, which orders contacts and events | yes (1034-1040) | about 2 KB (contact 348 free, island 77, set 71) |
| solver sets: static, disabled, awake, one per sleeping island. Body sims (216 B: transform, centre, TOI pose, local centre, force and torque accumulators, inverse mass and inertia, damping, gravity scale, flags); body states (64 B: velocities, delta pose); joint sims; contact indices; island sims | solver_set.h:30-55, body.h:163-227 | integration; the awake-set order is the move-event order | yes | static 1,782 × 216 = 385 KB; awake 1,192 × (216 + 64) = 334 KB; contacts and islands 14 KB |
| bodies (136 B): `sleepTime` (the sleep timer), `sleepThreshold`, `sleepVelocity`, island links, heads of the contact and joint lists, mass, inertia, flags, generation | body.h:78-134 | sleeping (so `fellAsleep`, so our freezes); the contact-list order is the order of `b3Body_GetContactData`, and so our stress-load sums | yes | 2,977 × 136 = 405 KB |
| shapes (208 B): filter, material, hit-event flag, AABB flags, geometry | shape.h | collision filtering, fat AABBs | yes | 4,415 × 208 = 918 KB + about 50 KB of geometry references |
| hull geometry | recording registry | narrow phase | yes, as a registry | 3.4 MB. D from our `lpShape` via quickhull; ours is a second copy |
| fat AABBs | physics_world.h | which pairs get contacts; which pieces our wake queries find (item 8.4) | yes | 4,415 × 24 = 106 KB |
| contacts (216 B): flags (touching, hit event), SAT or simplex cache, recycle cache (`cachedRotationA`/`B`, `cachedRelativePose`), friction and restitution mix, island membership, per-body list links | contact.h:99-162 | the narrow phase's warm starts and contact recycling | yes | 4,769 × 216 = 1.03 MB |
| manifolds (268 B): points (56 B) with anchors, separation, `normalImpulse` (warm start), `totalNormalImpulse` (our stress loads), `featureId`, `baseSeparation`, `persisted`; normal, twist, friction and rolling impulses (warm starts) | types.h:2655-2716 | solver warm start; our loads | yes (825-873) | 1,826 × 268 = 489 KB |
| joints (72 B) and joint sims (444 B: warm-start impulses, motor, limits) | joint.h | joint solve and our link loads | yes | none in town (mech: 18) |
| islands (bodies, contact links, joint links, `constraintRemoveCount`) and the split candidate | island.h | island splitting and sleeping | yes (1096-1111) | about 41 KB |
| broadphase trees (nodes 32 B + parent 4 B, proxies 24 B, free lists) | broad_phase.h, types.h | query and cast callback order (ties, item 8.5), fat-AABB refits, pair finding | yes (284-303) | static 5,834 nodes + dynamic 3,312, about 450 KB |
| pair set (hash set of shape pairs, written raw at capacity) | broad_phase.h | membership only (derived from the contacts) | yes (241-249) | 16,384 × 16 = 262 KB |
| graph colours: body bitsets, joint sims, convex contacts, contact specs (the wide constraints are transient) | constraint_graph.h | colour assignment is history-dependent and sets the solve order | yes (443-453) | about 40 KB |
| events: move (with `fellAsleep`), contact hit and begin/end, joint | physics_world.h | read by us inside `lpWorld_Step` | no (transient) | – |
| hull database (content-keyed, reference-counted) | physics_world.c:43-81 | shared hull storage | via the registry | duplicates our `piece.hull` |

What a "restore, then continue" must reproduce to stay bit-exact:

1. **Everything `b3SerializeWorld` writes, byte for byte, in the same build.** The layout hash only catches struct-size
   drift, so the same code and flags are needed (for SIMD path equality see the brief). The probe shows this suffices
   for Box3D: with no custom filter, a snapshot taken mid-run at tick 300 of town and keep, plus a replay of the
   recorded calls, matched Box3D's per-step state hash for 120 of 120 frames. It also matched for 200 of 200 on mech
   and track (100 to 300) and on pile from load, and for town from load. That held whether the session was recorded
   at 8 workers or at 1; the player always replays at 1.
2. **The callbacks and `userData` it drops.** With our filter installed, the same replays diverged at frame 1 (town and
   keep from tick 300), 6 (track), 19 (town and pile from load) and 143 (mech). Filter decisions are not recorded
   (recording.c:398 records only the `enableCustomFiltering` flag), and the callback reads our world. Restoring our
   `lpWorld` with Box3D and reinstalling the callback closes that gap for a combined snapshot. It does not make
   Box3D's own recordings usable as a debugging log of our sessions.
3. **The same world slot.** Our stored ids embed `world0` (id.h:44-49). Restoring into a different slot means patching
   `lpBody.id`, `lpPiece.shapeId`, `lpLink.joint`, `lpLink.builtOn` and `lpLink.anchor`, or restoring in place, as the
   Box3D player does ("the world id stays stable").
4. **A step boundary.** Events are transient and consumed inside `lpWorld_Step`.
5. **An API that exposes it.** `b3SerializeWorld` and `b3DeserializeIntoShell` are internal (world_snapshot.h:16-21).
   The public path is a recording (`b3World_StartRecording` writes the snapshot) and `b3CreatePlayer` (restores it
   into a player-owned world). A small patch exposing serialize and deserialize-into-world (logged in
   extern/box3d/PATCHES.md) is the clean route (unverified effort).

Leaner options, all unverified. These parts could be skipped and rebuilt:

- the hull registry (3.4 MB, rebuilt from our shapes)
- the pair set (262 KB)
- shape records (918 KB), rebuilt from our pieces with the same creation order

That needs a custom deserializer and Box3D internals knowledge, while a verbatim snapshot compresses 2.6 to 3.6 times
anyway. For agent-friendliness, ship Box3D's image verbatim and rebuild only our side's caches.

## 3. Origination map (Q3)

Every discrete state change and where its decision comes from. "Box3D input" is the float result read, and the rate is
the per-tick average (maximum) for town / barrage / keep from the probe.

| command a host would send | site (file:line) | Box3D input | threshold / rule | rate town / barrage / keep |
|---|---|---|---|---|
| **RESOLVE_TOOL** (ray to impact point) | world.c:1175-1219; main.cpp:507-544; scenes.c:1856-1879 | `b3World_CastRayClosest` against the current world; ropes and wheels tested against cached `link.points` | closest hit; ties by tree order | inputs |
| **COLLISION_IMPACT** (point, radius, energy) | impact.c:891-1022 | hit events (`approachSpeed`, point), `b3Body_GetMass` of both (969-972), manifold anchors if crushable (982-997) | speed ≥ `hitSpeed` 4 m/s (952); energy × (1 − crush) ≥ 100 J (998); sorted by energy, top `maxHitImpacts` 16 (1007-1021); radius 0.06·cbrt(E) (1018) | impacts, all kinds: 4.8 (17) / 11.5 (17) / 2.5 (16) |
| **FRACTURE** (piece, local point, impact, seed, snap) | impact.c:538-604 | `b3World_OverlapAABB` candidates (699), then the piece's body transform (561-563) | density(d) ≥ `fractureEnergy` (564); radius ≥ 1.5 fragment (557); depth < `maxDepth` (552); nearest first, 48 jobs per step, the rest deferred (577-591); seed = mix(seed, tick, index, generation) (85) | 1.08 (23) / 3.54 (31) / 0.52 (32) fractures; 12 / 31 / 4 cells |
| ejecta motion (part of FRACTURE) | impact.c:236-240, 305-310, 369-377 | the parent's `v`, `omega`, local centre | ghost tumble from a pieceSerial seed | – |
| **SNAP_SLENDER** (a FRACTURE with snap) | stress.c:907-935, 939-966 | via stress loads | piece strain += (ρ − 1)·rate ≥ 1 (916-919) | included above |
| **DAMAGE_BOND / BREAK_BOND** (impact) | impact.c:644-688 | the piece's body transform (654-655), candidates from a fat-AABB query (646) | health −= density(distance); ≤ 0 breaks (668-672) | – |
| **BREAK_BOND** (stress) | stress.c:626-681, 1403-1417, 745-782 | loads: `totalNormalImpulse`/dt over `b3Body_GetContactData` (183-224, contact order), link forces (228-243), wheel loads (wheel.c:706-724); inertia relief from velocities (stress.c:80-144) | ρ > 1: strain += (ρ − 1)·`strainRate` (642-652); strain ≥ 1 queues; ρ ≥ 2 breaks at once, else at most `maxStressBreaks` 4 per check, worst first (660-681) | 2.96 (126) / 4.42 (98) / 0.53 (151) |
| **RECHECK_STRUCTURE** (host-internal) | impact.c:909-935; link.c:739-779; wheel.c:375-402, 577-581; gait.c:279-292 | hit speed ≥ 1.5 m/s (at most every 30 ticks) or ≥ 4 m/s for moving bodies (every 10); link force or torque change | > 25% + 10 N (link.c:743-744), at most every 30 ticks; wheel landing > 3 × sprung weight; skip if loads changed < 2% of weight (stress.c:1622-1637) | – |
| **BREAK_LINK** (load) | link.c:825-894, 801-823 | `b3Joint_GetConstraintForce` and `Torque` (861-862), less the motor torque (868-880); gated by `b3Body_IsAwake` (851) | smoothed u = u + 0.5(load − u) (888); u ≥ 2 breaks; u > 1: strain += (u − 1)·rate·10·dt ≥ 1 breaks; after 3 settle steps | track 0.02 (2), mech 0.007 (1) |
| **BREAK_LINK** (wheel load) | link.c:834-846; wheel.c:540-574 | the wheel's own force from its cast and the tyre solve; mount awake | same thresholds | – |
| **BREAK_LINK** (tear-off) | link.c:655-665, 685-689, 466-475 | `b3Body_GetMass` (from shapes) | mount < 0.2 × sprung mass; the lighter end < `tearRatio` (0.02) × the heavier | – |
| **BREAK_LINK** (blast) | impact.c:609-640 | `link.points` cached from transforms | health −= density ≤ 0 | – |
| **DETONATE** (detonator, blast point) | impact.c:716-723, 957-967, 458-500 | body transform, hit `approachSpeed`, `b3Body_GetWorldCenter` (487) | blast density > 150 J/m² at the piece; hit ≥ `triggerSpeed`; first to go disarms; in hit-event order (497) | rare |
| **FREEZE** (body, exact pose) | step.c:28-84 | move events (`fellAsleep`), `b3Body_IsAwake` (68), contact data (link.c:708-732) | not linked or touching a linked body; age ≥ 6 (light) / 30 ticks; at most 64 per step in move-event order | 6.1 (64) / 18.7 (64) / 2.5 (21) |
| **FREEZE** (light debris) | debris.c:815-825 | `b3Body_GetLinearVelocity` | age ≥ 240, \|v\| < 1 m/s, shares the cap | included |
| **KILL** | step.c:46-48, 86-98; debris.c:498-502, 443-454, 867-876; step.c:272-285 | move-event `transform.p.y` | y < `killDepth` −50; ghosts likewise; sunk scrap; ghosts over budget; detonated | 0.10 (9) / 2.3 (69) / 0.03 (8) |
| **WAKE** (rubble to debris) | impact.c:706-715; impact.c:938-951; split.c:117-127 with step.c:9-26; debris.c:564-611; debris.c:633-669; step.c:208; link.c:236-241 | fat-AABB queries (no exact test) for impacts, split regions and shoves; hit speed ≥ `wakeSpeed` 1.5 m/s; mover speed ≥ 2 m/s and mass ≥ 60 kg with its Box3D AABB | – | 3.2 (125) / 13.6 (243) / 1.3 (37) |
| **TIER** (demote, land, promote) | split.c:137-184, 215-226; debris.c:843-900; debris.c:429-506; step.c:203-206; debris.c:689-695 | volumes (pure); budgets (counts and ranking by volume, age and index: pure given the kinds); ghost landing `b3World_CastRay` against static shapes, landing at ceil(fraction × 8) ticks | – | tier-downs 0.35 / 7.0 / 0.15; landings 5.6 / 27 / 0.05 |
| **SPLIT** | split.c:10-229 | pure topology; only the new bodies' velocities come from Box3D (101-135) | – | 0.76 / 1.87 / 0.41 |
| **CREATE_BODY** (fracture ejecta, split components, lost wheels) | impact.c:365-392; split.c:153-184; wheel.c:225-253 | parent velocities; hub pose and velocity | – | 10.3 (300) / 26 (339) / 2.2 (116) |
| **LOSE_WHEEL** (pose, velocity) | wheel.c:210-222 | `hub`, `hubVelocity` and `bodyOmega` from the last awake step | – | – |
| **RIG_STATE** (desired pose, limb states, servo targets and feeds) | rig.c:370-429, 194-259, 433-484; gait.c:379-819 | torso transform and velocity, link angles (link.c:866), foothold casts (gait.c:247-276), contact manifolds (rig.c:436-481) | able: strength ≥ 0.2 and depth ≥ 0.6 × stand height (rig.c:412); arrived < 3 cm (gait.c:549); slipped > 10 cm (554); support margin (661); stuck after 90 ticks under 10% pace (694-695); falling < −1 m/s (723); idle after 30 calm ticks (805-818); touch within 0.35 m at separation < 5 cm | every tick while walking |
| **MOTOR** (continuous Box3D input) | link.c:564-643 | `b3RevoluteJoint_GetAngle` (578), frame rotations (596-597) | – | every tick |
| **VEHICLE** (continuous; sets `sliding`, `grounded`, contact) | wheel.c:596-704, 292-337, 405-583 | shape casts (318), mass, inertia, velocities | sliding at ≥ 99.9% of the friction limit (556) | every tick |
| **PUSH** (pulls, blasts, blows, shoves: continuous) | step.c:187-229; impact.c:788-844; debris.c:633-687, 564-625 | point velocity, centre, mass, extent | – | inputs and impacts |
| supply and pools | supply.c:57-198 | **none** (topology and pool levels) | – | – |

Order dependencies on Box3D's report order, already acted on:

- The freeze cap (step.c:32-84).
- Detonation order and stress re-check (dirty-list) order, both in hit-event order (impact.c:895-967).
- Float sums of contact loads in contact-data order (stress.c:187-224).
- Only the kill list and the hit impacts are sorted (step.c:87-90; impact.c:1007-1010).

**Follower mode.** A client in a host-authoritative hybrid.

Switch off, and take the host's command instead:

- `lpCollectHits`: collision impacts, detonations by hit, wakes by hit, stress re-check requests
- the freeze and kill parts of `lpFreezeOrKill`
- the light-debris freeze in `lpEnforceBudgets`
- candidate selection and budgets in `lpFractureCandidates` for dynamic bodies
- `lpCheckStructures` judging, so no stress solve runs on clients: a large CPU saving
- the break decisions of `lpPollLinks`
- `lpShove`
- query-based wakes
- `lpStepRigs` and `lpWalkRig`
- tyre solves for vehicles the client does not predict
- resolving tool rays

Still compute locally, all pure given the command stream:

- fracture of a commanded piece (cells, hulls, cell bonds, classification)
- bond and link damage from a commanded impact on structures and rubble: their transforms are static and identical once
  freezes carry exact poses
- splits and tiers by volume
- `lpSyncLinks` rebuilds, and moving link ends on a fracture
- supply and pools
- ghost flight and landing: casts hit only static shapes, so they match once static state matches. The initial
  velocity must come from the host, or ghosts must be treated as cosmetic.
- scrap sinking and the ghost and scrap budgets
- particles and meshes

Ghosts and scrap feed back only through `lpWorld_PromoteBody` and pulls (step.c:203-206). They are 52% of bodies at
the town peak (3,168 of 6,142) and 69% at the barrage peak (9,858 of 14,198). Declaring them client-local cosmetic in
a hybrid removes most bodies from sync.

## 4. Inputs (Q4)

| API (lpf.h) | kind | reads Box3D at call | sandbox record (main.cpp) | round-trips |
|---|---|---|---|---|
| `lpCreateWorld` (175): `lpWorldDef` | config | – | scene, bombard period, fragment scale on the command line | session setting |
| `lpCreateObject` (250) | immediate: Box3D body and shapes now; returns a body index | no (mass from shapes) | flask and ball tools, built from camera origin and direction at apply time (447-504), recorded `%.6f` (576) | **no** |
| `lpCreateLink` (313) / `lpDestroyLink` (314) | immediate | body transforms for the frames (link.c:230-234), wakes rubble | `grab` event (565): **a toggle kept in `app.grip`** (413-429) calls `lpRigGrab` (scenes.c:1490-1505) or destroys the grip | meaning depends on app state |
| `lpWorld_SetRopeLength` (317) | immediate: Box3D setter and wake | – | crane driver every tick from state (scenes.c:1390); not recorded | derived |
| `lpWorld_SetLinkTarget` / `Rotation` (321-322) | persistent setter, hashed | – | crane driver, gait | derived |
| `lpCreateVehicle` (401), `lpCreateRig` (497) | immediate | mass, transform, link angles (wheel.c:151-154; rig.c:319-339) | scene build | R |
| `lpWorld_SetVehicleControl` (413) | persistent setter, hashed, from the next step | – | `drive` `%.6f` (570); key values (±1, −0.6, 0) are exact | yes for keys, no for analog |
| `lpWorld_SetRigControl` (509) | persistent, hashed | – | `walk` `%.6f` (552); key values exact | yes for keys |
| `lpWorld_SetLimbTarget` (554) | persistent, hashed | – | `reach` `%.9g` (559) | yes |
| `lpWorld_AddImpact` (569) | queued, next step, call order | – | tools record the **camera ray** `%.6f` (576), resolved at apply time by `lpWorld_CastRay` against the current world (507-544) | **no**, and state-dependent |
| `lpWorld_Blow` (573) | queued per tick | – | origin and direction `%.6f` | **no** |
| `lpWorld_Pull` (586) | queued per tick | – | target and local point `%.6f`, plus a **piece slot index** picked by an unrecorded ray at grab time (717-750) | **no**; runtime id |
| `lpWorld_PromoteBody` (576), `lpWorld_SetGravityScale` (580) | immediate: tier change or Box3D setter now | – | not used by the sandbox | – |
| `lpWorld_Step` (590) | dt and substeps | – | fixed 1/60 and 4 (773) | session setting |
| `lpWorld_SettleStructures` (595) | immediate, at load | – | inside `lpBuildScene` | R |
| `lpSceneBombard`, `lpSceneDrive` | deterministic inputs read from state | ray casts and states | a command-line option, every tick (766-770) | derived |

Within a tick the sandbox applies script events in file order, then live events, then the bombardment, then the
drivers, then the step (main.cpp:755-773). Live runs apply full-precision floats while the file keeps `%.6f`, so a
recording reproduces a nearby session, not its own. Replays of the same file agree with each other, which is what
`check-determinism.ps1` tests.

**Proposed canonical command set.**

- Every command is stamped `(tick, peer, seq)` and applied at the start of its tick in `(peer, seq)` order, before the
  step. That includes immediate creations, so they always happen at the same point.
- Floats travel as raw bits.
- References are global keys (section 5), never slot indices.

| command | fields | replaces |
|---|---|---|
| `TOOL` | tool id, ray origin, direction, range | camera-ray tools. Lockstep resolves the ray at apply time; a hybrid host resolves it and sends `IMPACT`. |
| `IMPACT` | point, direction, radius, energy, impulse, explosion | resolved hits |
| `SPAWN` | template id, transform, velocity, angular velocity | flask and ball; any game projectile |
| `GRAB` / `GRAB_TARGET` / `RELEASE` | piece key, local point, target, max accel, max mass; target on change | per-tick `pull`: make the pull persistent core state like controls |
| `BLOW_SET` | on/off, origin, direction, range, half-angle, speed | the per-tick `blow` |
| `VEHICLE_CONTROL` | vehicle, throttle, brake, steer, handbrake | `drive` |
| `RIG_CONTROL` | rig, forward, strafe, turn, crouch | `walk` |
| `LIMB_TARGET` | rig, limb, active, point | `reach` |
| `CLAW_GRAB` / `CLAW_RELEASE` | rig, limb / link key | the `grab` toggle |
| `LINK_CREATE` / `LINK_DESTROY` / `ROPE_LENGTH` / `LINK_TARGET` | link def with body keys / link key / length / angle or rotation | direct calls |
| `GRAVITY_SCALE`, `PROMOTE` | body key, scale | direct calls |
| session | `lpWorldDef` without `workerCount` and `debugLog`, `b3WorldDef`, dt, substeps, build id and content hash | setup |

A hybrid adds the host's state-change stream from section 3:

- `FRACTURE`
- `BREAK_BOND`
- `BREAK_LINK`
- `DETONATE`
- `FREEZE` (with pose)
- `WAKE`
- `KILL`
- `TIER`
- `LOSE_WHEEL`
- `CREATE_BODY` (key, velocities)
- `RIG_STATE`

It also adds a dynamic-body state sync for about 800 to 2,100 bodies changing per tick at the peaks.

## 5. Runtime ids (Q5)

| handle | allocation | generation | where it leaks | stable across peers |
|---|---|---|---|---|
| piece slot | LIFO free list (world.c:192-237) | yes (201); in `lpPieceInfo` | fracture seed (impact.c:85), iteration order, pull events | lockstep: yes. Follower: only if every allocation and free happens in the host's order |
| body slot | LIFO (world.c:257-291, 652-653) | yes (271), but **not exposed**: `lpCreateObject` returns a bare index; `lpWorld_GetBodyTransform` only says alive | the game, `lpLinkDef.bodyA`/`B`, `lpVehicleDef.body` | as above |
| bond slot | LIFO (world.c:239-255) | **none** | stress breaks by index | as above; ambiguous across reuse |
| link slot | LIFO (link.c:178-199) | yes; in `lpLinkState` | the game, rigs (`limb.gen`), the `grab` toggle | as above |
| wheel slot | LIFO (wheel.c:72-101) | **none** | internal only (`link.wheel`) | as above |
| vehicle, rig | append-only, never freed (wheel.c:156-158; rig.c:362-363) | – | the game | yes |
| pool, detonator | append-only (world.c:787-796, 858-863) | – | pieces | yes |
| Box3D body, shape, joint, contact ids | LIFO id pools (id_pool.c:19-31), `{index1, world0, generation}` (id.h:44-49) | yes | `lpBody.id`, `lpPiece.shapeId`, `lpLink.joint`/`builtOn`/`anchor` | lockstep: yes. `world0` is a process-global slot and differs between processes that create worlds differently |

Seeds follow the slots. A fracture's seed is mix(world seed, tick, piece slot, piece generation) (impact.c:85), and a
ghost's tumble is seeded by `pieceSerial` (impact.c:338, 369-377). In a hybrid a client with a different slot history
gets different cells. So the seed must be in the `FRACTURE` command or be derived from a global key.

**Global keys for a hybrid.** Keep slot indices as local handles, with the index-order iteration. Map keys to slots
only at the network boundary.

- **Pieces from the scene:** `(object serial, part index)`. `userId` is the game's, not guaranteed unique.
- **Fracture cells:** `mix64(parent key, cell index)`. The cell index is deterministic because fracture is pure.
- **Bodies:** `(creating event key, ordinal)`: a split is (key of its first piece, component ordinal), ejecta take the
  key of their piece, lost wheels the key of their link.
- **Bonds:** `(min piece key, max piece key)`. This assumes at most one bond joins a pair of pieces. Scene parts are
  bonded once per unique pair (world.c:742-784); that fracture never double-bonds a pair was not proven and wants an
  assert.
- **Links:** the creating command `(tick, peer, seq)`.

Lockstep can keep slot indices, but the proposed API should expose body generations anyway.

## 6. The state hash (Q6)

**Cost model.** `lpWorld_Hash` (world.c:1012-1082, then link.c:940-973, wheel.c:844-870, rig.c:588-626,
supply.c:218-228):

- Per body: an index, kind, tier and piece list, plus either a Box3D transform and velocity read through three API
  calls, or the ghost state.
- Per piece: body, volume, centroid, **every vertex (12 B each)** and the bond list.
- Per bond: a, b and health.
- Then links, vehicles, rigs and pools.

So it is O(bodies + pieces × average vertices + bonds + links), fed byte by byte to FNV-1a: a serial xor and multiply
per byte.

| scene | `lpWorld_Hash` | bytes fed | `lpWorld_HashStress` | `b3HashWorldState` |
|---|---|---|---|---|
| town load (1,150) | 0.24 ms | 214 KB | 0.10 ms | 0.00 ms |
| town peak (7,582) | 2.9 to 4.2 ms | 1.47 MB | 0.86 to 0.99 ms | 0.17 ms |
| barrage peak (16,339) | 6.5 ms | 3.38 MB | 1.50 ms | 0.21 ms |
| keep load (1,973) | 0.44 ms | 363 KB | 0.18 ms | 0.00 ms |
| keep peak (4,231) | 1.47 ms | 0.99 MB | 0.51 ms | 0.04 ms |
| siege peak (8,766) | 3.5 ms | 2.27 MB | 1.03 ms | 0.10 ms |

About 2 to 3 ns per byte. A per-tick full hash at the town peak equals the whole average step (4.0 ms at 8 workers).
Box3D's hash (recording.c:1188-1232) mixes whole 32-bit words and covers only body transforms and velocities.

**Coverage gaps.** A desync here surfaces only through its effects, possibly many ticks later:

- In `lpWorld_HashStress` only: bond strain and ρ, stress vectors.
- Not hashed anywhere:
  - bond force and moment
  - body `volume`, `createdTick` and the stress scheduler flags
  - ghost landing plans, scrap `sinkTicks`
  - `freezeCandidates`, `deferred`, `nextImpacts`, `pendingDestroy`, `audits`, `lostWheels`, `dirtyBodies`
  - the three serials, `lastTimeStep`
  - detonator `armed`
  - link `points`, `force`, `settle` and applied motor shadows
  - wheel `spin`, contact point and normal, hub
  - limb `depth`, `neutral`, tip cache
  - pool `found`
  - all of Box3D beyond transforms (warm starts, sleep timers, caches)

  For desync detection that is acceptable. For validating a restore it is not: compare snapshots byte for byte, or
  add a full-state hash.

**Incremental hash per structure or island, dirty-only.** Measured churn per tick counts the bodies that are awake
dynamic, ghosts, created, or whose topology changed:

| scene | bodies changed per tick (max) | pieces on them per tick (max) | live pieces (avg) |
|---|---|---|---|
| town | 818 (1,779) | 1,048 (2,289) | 4,225 |
| barrage | 2,107 (3,033) | 2,785 (4,228) | 9,807 |
| keep | 407 (668) | 906 (3,543) | 3,092 |
| siege | 1,235 (2,254) | 3,388 (6,249) | 5,295 |

Pieces never move within their body frame, so a moving body changes one transform, not its pieces. What an incremental
hash needs:

1. **A geometry digest per piece**, computed once in `lpShape_Create`. Geometry is immutable; it is the bulk of the
   bytes today.
2. **A position-sensitive, commutative combination.** For example the sum over elements of `mix64(kind, index,
   fields)`. An update subtracts the old term and adds the new one, at O(1) per changed element.
3. **Dirty hooks at every mutation site:**
   - attach and free piece (world.c:451-465, 220-237)
   - add and break bond, and bond health, ρ and strain writes (world.c:492-547; impact.c:430, 668; stress.c:629, 651)
   - body alloc, destroy, kind and tier changes (world.c:257-291, 616-654; debris.c:201-341; step.c:77-78)
   - Box3D transforms: exactly the bodies in the step's move events (step.c:30), plus velocities of awake bodies
   - ghosts: all of them every tick (debris.c:429-506)
   - links, wheels, rigs and pools every step while active
   - stress vectors of the structures solved this step (stress.c:1149-1159)
4. **Grouping for localisation:**
   - per body, which is a structure
   - per Box3D island for dynamic bodies
   - a small Merkle tree over body-index ranges, so peers can binary-search a mismatch in O(log n) round trips
5. **Word-wise mixing** instead of byte-wise FNV, about 4 to 8 times faster.

Expected cost (unverified): a few thousand records of about 64 B per tick, about 0.05 to 0.2 ms at the peaks, against
3 to 6.5 ms now. Box3D's hash can likewise hash awake bodies from the move events and keep the sleeping and static
contribution cached.

## 7. Hypotheses

- **H5 (a serializable, command-logged world with per-island hashes is the most valuable outcome): supported.**
  - Every model needs a snapshot: late join, host migration, save games, rollback, desync repair.
  - Box3D already provides a bit-exact one; the probe verified mid-run restore and continue in six scenes.
  - The missing work is ours: a serializer for the S fields above, rebuilding the D caches, rewiring `userData` and the
    filter callback, patching `world0` or restoring in place, and fixing the cache-dependent stress budget.
  - Naive sizes at the peaks of the big rungs, both sides together:

    | rung | xz | gzip |
    |---|---|---|
    | town | 3.7 MB | 4.9 MB |
    | keep | 4.1 MB | 5.4 MB |
    | barrage | 7.3 MB | 10.0 MB |
    | siege | 8.6 MB | 11.6 MB |

    They are under 0.2 MB for the track and mech, and zero at load (rebuilt from the scene). Nothing here requires a
    solver rewrite.
- **H2 (a hybrid loses on player-host upstream at 12 players): numbers only, no verdict.** A hybrid host must send:
  - about 20 to 70 discrete state changes per tick at the peaks, or several hundred on a blast tick (max 243 wakes,
    339 creations)
  - state for 800 to 2,100 changing bodies per tick, unless ghosts and scrap become client-local
- **H1, H3, H4, H6:** not addressed by this track.

## 8. Hardening items

1. **Freeze candidates follow Box3D's move-event order** (step.c:32-54), and the 64-per-step cap is taken in that order
   (step.c:60-84). Sort each step's new candidates by body index (or rank by `createdTick` and index) before appending.
2. **Hit events are acted on in Box3D's report order.** Detonation blasts are pushed to `nextImpacts` in that order
   (impact.c:957-967, 497), and stress re-checks push the dirty list in that order (impact.c:909-935). The dirty list
   becomes the stress queue and budget order (stress.c:1527-1615). Collect these per event, then sort by the event key
   before acting.
3. **Contact loads are summed in `b3Body_GetContactData` order** (stress.c:187-224). Accumulate per piece in a sorted
   order: by piece, then shape pair.
4. **Wake decisions use fat AABBs with no exact test.** `b3World_OverlapAABB` reports tree (fat) AABBs
   (physics_world.c:2617-2668). The impact wakes every rubble body in the box (impact.c:699-715), split regions do too
   (split.c:117-127 with step.c:9-26), and so do shoves (debris.c:582-600). The wake set therefore depends on Box3D's
   margin and refit history. Test the transformed piece bounds exactly.
5. **Cast ties at an equal fraction keep the first reported hit**, which is the tree traversal order: wheels
   (wheel.c:279), feet (gait.c:236), ghost landings (debris.c:381), `lpWorld_CastRay` via `b3World_CastRayClosest`
   (world.c:1181). Break ties by piece index. determinism-rules.md rule 8 says the landing casts do not depend on
   callback order, which holds except for ties.
6. **The stress budget depends on whether a body's system is cached.** A rebuild is charged 2 × edges
   (stress.c:1591-1616, 1316), so a restored world without the caches solves differently. Either ship the systems
   (0.4 to 2.6 MB), or charge the same for a continuing solve whether or not it is cached.
7. **A clustered solve in progress lives only in `body.system` and `body.reduced`.** Its `x` stays in the system
   (stress.c:511, 520-523), and its y, r and p are only in the reduced system (stress.c:1135, 1149). It is 1.7 to
   2.6 MB on the keep and siege. Ship it, or make a restored correction restart.
8. **`lpLimb.depth` is not hashed.** It comes from link angles when the tip changes (rig.c:180-191, 237, 339) and gates
   `able` (rig.c:412) and the stand height (gait.c:434), yet rig.c:588-626 leaves it out. The same goes for `neutral`
   and the tip cache.
9. **`lpWheel.spin` is not hashed** (wheel.c:863-866) but sets the lost wheel's orientation (wheel.c:217, 244).
10. **Detonator `armed` flags are not hashed** (world.c:1012-1082; link.c:940-973).
11. **Ghost `planTicks`, `landIn`, `landPoint` and `landNormal`, and scrap `sinkTicks`, are not hashed**
    (world.c:1036-1043).
12. **`lpBody.volume` is an incrementally updated float** (world.c:462; impact.c:282; split.c:186; debris.c:188;
    step.c:147). It is used for tiers (split.c:219-225) and budget ranking (debris.c:787). Ship it, or recompute it
    from the pieces in one canonical order everywhere.
13. **The fracture seed uses slot index, generation and tick** (impact.c:85), and the ghost tumble uses `pieceSerial`
    (impact.c:338, 369-377). A hybrid needs the seed in the command or derived from a global key.
14. **Bonds and wheels have no generation** (world.c:239-255; wheel.c:72-101), and body generations are not exposed in
    the API (lpf.h:250, 717). Add them, or use keys (section 5).
15. **Box3D ids in our state embed `world0`** (id.h:44-49; world.h:127-129, 314, 383), and `b3SerializeWorld` zeroes
    `userData` (world_snapshot.c:537-539, 1057, 1077). A restore must rewire `userData` to index + 1 and patch or
    preserve `world0`.
16. **The custom filter callback is invisible to Box3D's recordings** (recording.c:398 records only the flag). Box3D
    replays of our sessions diverge in 1 to 143 frames. Reinstall the callback on restore. Alternatively, express the
    light-versus-frozen-rubble rule through category bits: this costs a filter change when a body freezes (world.c:395-396).
17. **The sandbox recorder writes `%.6f`** for tool rays, blows, pulls, spawns, drives and walks (main.cpp:552, 570,
    576), so recordings do not reproduce their own live session. Use `%.9g`, as `reach` does (main.cpp:558-560).
18. **The claw's `grab` event is a toggle held in `app.grip`** (main.cpp:413-429), and `pull` names a piece slot picked
    by an unrecorded ray (main.cpp:717-750). Use explicit `CLAW_GRAB` and `CLAW_RELEASE` commands and keyed
    references.
19. **`lpCreateLink`, `lpCreateVehicle` and `lpCreateRig` are immediate** and read Box3D transforms, masses and angles
    at call time (link.c:224-234; wheel.c:151-154; rig.c:319-339). Apply them only at the tick's command point. In a
    hybrid, send the resulting frames.
20. **`lpWorld_Hash` hashes every vertex byte by byte** (world.c:1061): 2.9 to 6.5 ms at the peaks. Store a geometry
    digest per piece at creation and mix word-wise (section 6).
21. **`piece.seed` is dead after creation** (impact.c:338, 370). Drop it from snapshots, or from the struct.
22. **Memory on a toaster.** `lpBody` (432 B) and `lpPiece` (296 B) carry stress and ghost fields on every element,
    while 70 to 86% of pieces are debris, ghost or scrap at the peaks. Hulls are held twice, ours plus Box3D's database:
    3.4 MB each at the town peak. Fracture job scratch is about 4.5 MB (142 KB per slot, mostly the fixed cell-bond
    buffer: world.h:544).
23. **Vehicles and rigs never die** (`alive` is never cleared, wheel.c and rig.c), and their arrays only grow. That is
    harmless now, but a long session with spawned vehicles leaks slots.
24. **Two `lpStats` counters are read by logic** within a step: `footCasts` (gait.c:349, 520) and `stressWaiting`
    (stress.c:1688). They are safe because stats reset at step start, but a snapshot or refactor must not treat
    `stats` as purely cosmetic.

## 9. Unverified, and what would verify it

- **Bit-exact restore of our side plus Box3D.** A test that serializes both at tick N, restores into fresh worlds
  (callback and `userData` rewired), steps to 600, and compares `lpWorld_Hash`, `lpWorld_HashStress` and a byte
  comparison of the snapshots against an uninterrupted run. Box3D's half is verified by the probe; ours has no
  serializer yet.
- **The lean snapshot sizes** (about 3.9 MB raw for ours at the town peak) and their compression: write the lean
  serializer and measure.
- **Incremental hash costs** (0.05 to 0.2 ms): implement it and compare against `lpWorld_Hash` in the bench.
- **The exact capacity of the fracture job scratch** (about 32 slots at the town peak): inferred from the heap
  arithmetic, not printed.
- **Whether exposing `b3SerializeWorld` / `b3DeserializeIntoShell` needs more than a header patch.**
