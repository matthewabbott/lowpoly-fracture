# Performance log

Machine: i7-10870H (8 cores / 16 threads), RTX 3060 Laptop, 16 GB, Windows 11, MSVC 19.42 Release.
Benchmark: `lpf_bench --scene <s> --workers 8 --ticks 600 --period 12` (headless, 60 Hz, a grenade or cannon blast every
12 ticks = 5 blasts per second, debris cap 1500). Step times in ms: avg / p95 / max.

## 2026-09-27 baseline (commit d4cc8ad, before debris tiers)

| scene | step avg / p95 / max | fracture avg / max | physics avg / p95 | peak pieces | awake contacts avg / max |
|---|---|---|---|---|---|
| town | 4.53 / 10.75 / 43.6 | 0.56 / 25.3 | 3.88 / 9.63 | 10,835 | 17.7k / 38.2k |
| walls | 3.09 / 6.70 / 61.6 | 0.47 / 42.1 | 2.54 / 4.80 | 3,794 | 13.8k / 25.9k |
| pile | 10.32 / 16.68 / 93.3 | 0.55 / 53.7 | 9.67 / 15.27 | 4,938 | 68.8k / 102.5k |

Notes: physics dominates; debris-on-debris contacts are the bulk (the pile averages 46 awake contacts per awake
body). Worst fracture steps are 25 to 54 ms when one blast refractures 100 to 340 pieces; there, Box3D shape
creation and body splitting cost more than the Voronoi cells.

## 2026-09-27 debris tiers (commit 41713e8)

| scene | step avg / p95 / max | awake contacts avg |
|---|---|---|
| town | 0.80 / 1.60 / 3.77 | 2.1k |
| walls | 0.87 / 1.79 / 7.2 | 2.6k |
| pile | 2.39 / 4.09 / 8.4 | 12.5k |

Single-threaded town: 12.5 ms -> 1.95 ms average step. Ghost and puff ejecta have no Box3D body, light debris only
touches static geometry, and resting debris freezes into static rubble, so contacts dropped 5 to 8 times.

## 2026-09-27 chunky ends: grain-true damage radius, anchor sites, keeper merging

| scene | step avg / p95 / max | fracture avg / max | physics avg / p95 | peak pieces | awake contacts avg / max |
|---|---|---|---|---|---|
| town | 0.74 / 1.65 / 5.22 | 0.16 / 4.16 | 0.55 / 0.87 | 4,306 | 1.5k / 3.0k |
| walls | 0.82 / 1.75 / 5.86 | 0.14 / 4.87 | 0.63 / 1.06 | 1,465 | 1.6k / 3.8k |
| pile | 0.95 / 1.81 / 5.07 | 0.10 / 4.01 | 0.76 / 1.07 | 2,064 | 3.1k / 3.7k |

Single-threaded: town 1.88, walls 1.67, pile 2.61 ms average step.

- The damage radius on wood was measured in grain-squashed space, so it reached 3.5 times too far along the grain and
  a hit log shattered along its whole length. Sites now use real-space distance.
- Grained pieces get an anchor site on each side just past the damage radius, so a log end is one cell.
- Cells that stay on the piece merge while their convex hull is within a per-material slack of their true volume
  (wood 30%, stone and brick 8%) and covers no ejecta centroid, so notches stay open. A shot log leaves two ends of
  about 40 triangles each (was 4 pieces and 108 triangles), and the pile's awake contacts fell from 12.5k to 3.1k
  because wooden crates break into fewer, bigger pieces.
- The hull merge runs in the parallel fracture phase; worst fracture steps are 4 to 5 ms at 8 workers (baseline 25
  to 54 ms).

## 2026-09-27 dirtier mess: ejecta chipping, real geometry down to 1.5 cm

Best of 3, 8 workers (1 worker in brackets). Hashes changed on purpose.

| scene | step avg | awake contacts avg | peak pieces (incl. ghosts and scrap) |
|---|---|---|---|
| walls | 0.74 (1.72) ms | 1,963 | 1,839 |
| town | 0.71 (1.71) ms | 1,410 | 5,445 |
| pile | 0.88 (2.42) ms | 3,203 | 2,057 |

- The glass-pane test now throws 140 to 160 ghost chips and leaves about 370 scrap pieces (was 34 and 77); the house
  flask demo leaves a carpet of plaster chips. Step times are within run-to-run noise of the previous baseline.
- A heuristic merge pre-check (reject pairs whose vertices poke out of each other's faces) was tried and dropped:
  it cut merge CPU by 5 to 10 times but also rejected good merges, and the extra pieces cost more in physics than the
  merge saved (pile at the strictest setting: 7.0k contacts and 1.51 ms per step, against 3.2k and 0.94 ms).
  Merging is worth its CPU. What shipped is exact: a lower bound on the hull volume plus skipping repeated tries of
  the same pair, with bit-identical results and about 12% less merge time.
- Contact counts swing by 10 to 20% between any two different simulations of the same bombardment (chaos), so
  small contact deltas need several bombard seeds before they mean anything.

## Backlog (measured 2026-09-27, commit after 7bf0f45)

Where the parallel fracture phase spends CPU time (`lpf_bench`, 600 ticks, sum over all jobs, 1 worker / 8 workers):

| scene | Voronoi cells | keeper merge | Box3D hulls |
|---|---|---|---|
| walls | 14 / 19 ms | 46 / 92 ms | 15 / 30 ms |
| town | 22 / 30 ms | 70 / 120 ms | 21 / 38 ms |
| pile | 26 / 39 ms | 51 / 160 ms | 14 / 41 ms |

Findings and candidate wins, biggest first:

- **Keeper merge is about 60% of fracture job time.** Every candidate pair runs a quickhull (`b3CreateHull`) and a
  volume check, and 60 to 90% of pairs fail it. The exact lower bound now rejects about a third of those without a
  hull (see above); the rest need a cheaper exact hull volume, e.g. an incremental hull that inserts one cell's
  vertices into the other (both are convex, so no general quickhull is needed).
- **CPU time roughly doubles at 8 workers.** Hull building and shape creation allocate on every call, so the heap
  lock is contended. Per-job scratch arenas for merge and hull building would remove most of that.
- **Direct hull builder** (deferred): build Box3D hull data straight from our cells, which already have exact
  face topology, instead of re-running quickhull. It needs a Box3D patch (`extern/box3d/PATCHES.md`) and removes at
  most the hull column above: worth it after the two items above.
- **Renderer sync is O(everything) per frame.** `Renderer_Sync` walks every piece and every body transform each
  frame. A change feed from the core (pieces whose mesh changed, bodies that moved) makes it O(changes); scrap and
  rubble never move.
- **Scrap render batching:** scrap is static, so it could be merged into static render chunks per grid cell.
- **Toaster profile:** caps, fragment scale, render scale and shadows behind one switch.

## 2026-09-27 stress solve replaces the weight check

Best of 3, 8 workers (1 worker in brackets); the tower joins the ladder.

| scene | step avg | stress avg / max | awake contacts avg |
|---|---|---|---|
| walls | 0.84 (1.86) ms | 0.12 / 2.9 ms | 1.6k |
| town | 1.35 (2.92) ms | 0.27 / 2.9 ms | 2.1k |
| pile | 0.93 (2.37) ms | 0 | 3.0k |
| tower | 2.20 (4.70) ms | 0.10 / 2.8 ms | 2.9k |

- The solve itself stays inside its work budget (40k bond-iterations, about 3 ms worst step). Cold solves: tower 85
  pieces in 19 iterations, house 48 pieces in 27; a damaged tower of 391 pieces and 1.3k bonds converges over about
  10 steps with warm starts and continuation.
- Town and walls cost more because structures now really come down under bombardment: falling chunks hit, fracture
  and make debris (town fracture avg 0.16 -> 0.49 ms, physics 0.51 -> 0.75 ms, peak pieces 5.4k -> 7.3k). Levers if
  this matters: `stressScale`, joint strengths, hit-impact caps, debris budgets.
- What made the solve converge: continuing conjugate gradient across steps instead of restarting it, a block-Jacobi
  preconditioner (6x6 per piece), and not keeping fracture crumbs on structures (kept cells below the light-debris
  size fall). Before those, a damaged tower needed 60+ steps and the old "judge anyway" fallback broke hundreds of
  joints on garbage forces.

## 2026-09-27 stress, part 2: rocking joints, slender pieces, resting loads

Best of 3, 8 workers (1 worker in brackets).

| scene | step avg | notes |
|---|---|---|
| walls | 0.56 (1.19) ms | 34% faster: rocking joints stop spurious dry-joint breaks, fewer pieces |
| town | 2.67 (5.45) ms | twice the previous step: houses under constant bombardment now also collapse under the rubble on them |
| pile | 0.87 (2.43) ms | unchanged |
| tower | 2.23 (4.93) ms | unchanged average, spikier p95 |

- **Rocking capacity:** a compressed joint holds a moment until its resultant reaches the edge of the patch (masonry
  tips over an edge; it does not crack as soon as the load leaves the middle third).
- **Slender pieces** (beams, planks) check their bending along the axis between supports and snap there with a
  tilted planar cut (a fracture job flagged as a snap: no bond damage around it, and merging never heals the spot
  that was hit).
- **Resting loads:** dynamic bodies pressing on a structure add their contact forces (sampled when a solve starts and
  frozen for it). A hit re-checks a structure at most every 30 ticks, and a check whose loads changed by under 2%
  of the structure's weight ends without a solve.
- **Creaking shortcut:** a converged structure that is only creaking adds strain from stored utilizations with no
  rebuild or solve.
- **Budget:** now 20k bond-iterations per step (measured about 60 ns each).
- **Town's stress share:** 0.74 ms of its 2.67 ms step goes to re-solving houses after each blast. That work is
  real, and houses are independent, so solving structures in parallel is the next lever (roadmap section 2,
  optional step).

## 2026-09-27 masonry pattern

Brick walls are one solid piece until hit, then break along a shared course grid (stair-stepped holes, mortar bonds).
Town got cheaper with it: 2.04 ms per step at 8 workers (was 2.67), because its brick walls are now one piece each.
Walls 0.55 ms, pile, lumber and tower unchanged.

## 2026-09-27 ruins scene

New bench rung: the ruins scene (a dry arch, a colonnade, balconies) under the default bombardment, 0.11 ms per step
at 8 workers (0.13 ms at 1). The other scenes' hashes are unchanged; their timings were within noise of the last
entry (walls 0.54, town 1.97, pile 0.83, lumber 0.20, tower 2.00 ms at 8 workers, best of 2).

Should structures be solved in parallel (milestone 2's optional step)? Measured on town with `--bombard 12`, 600
ticks: 346 ticks finish a solve; 240 of them finish one structure, 78 two, 28 three or more. The heaviest ticks are
one house (about 130 pieces) spending the whole 20k bond-iteration budget. Parallel solves per structure would save
little, so they were not built. Solve time lives inside one structure's conjugate gradient. (Wrong conclusion: the
bench's light bombardment hides the case they are for. See the next entry.)

## 2026-09-28 parallel stress solves

Structures are now checked together after the splits: each reserves its share of the budget before anything is built
(at most `maxStressStructureWork` = 10k bond-iterations for itself, `maxStressWork` = 60k for all), then all reserved
structures build and solve in parallel, and their joints are judged in queue order.

Why: under heavy bombardment (town, a blast every 3 ticks) about 10 structures waited for budget every step, and
each waiting one still sampled its loads and built its whole system before finding the budget spent (336 ms of the
1.17 s of stress time over 600 ticks). Measured on the barrage, 8 workers, 600 ticks:

| | before | after |
|---|---|---|
| stress avg / max | 1.93 / 3.16 ms | 1.00 / 2.26 ms |
| structure-steps waiting for budget | 6204 | 197 |
| structure-steps solved | 972 | 3005 |

On the standard schedule the ladder's single town run is chaotic: a different collapse order brings down a different
amount (its schedule now fractures 1185 pieces instead of 564). Over eight schedules (a blast every 8, 9, 10, 11, 12,
13, 14 and 16 ticks), town's destruction is unchanged on average (9300 vs 9304 peak pieces) and:

| mean over 8 schedules | step avg | stress avg | worst stress step |
|---|---|---|---|
| 8 workers, before | 2.73 ms | 0.92 ms | 3.34 ms |
| 8 workers, after | 2.46 ms | 0.62 ms | 2.15 ms |
| 1 worker, before | 5.06 ms | 0.92 ms | 3.05 ms |
| 1 worker, after | 5.37 ms | 1.04 ms | 5.22 ms |

One core pays for faster collapses with more stress CPU; set `maxStressWork` equal to `maxStressStructureWork` there.
The ladder gains a `barrage` rung (town, a blast every 3 ticks).

New baseline, best of 3, ms per step at 8 workers (1 worker): walls 0.48 (0.82), town 2.47 (5.61; the more destructive
outcome above), pile 0.83 (2.38), lumber 0.20 (0.27), tower 1.94 (4.18), ruins 0.11 (0.13), barrage 4.82 (11.68) with
stress at 0.99 (2.62) ms.

## 2026-09-28 links

Links (joints between objects) cost nothing in a world without them. The pre-milestone build (2dee624) and this one,
run alternately under the same conditions, gave identical hashes and step times within noise, best of 3:

| rung | before | after |
|---|---|---|
| barrage, 8 workers | 6.49 ms | 5.73 ms |
| barrage, 1 worker | 13.59 ms | 13.62 ms |
| walls, 8 workers | 0.66 ms | 0.57 ms |

(The machine was slower than when the baseline was taken, so both sides read above the ladder's numbers.)

The new yard rung (13 links: a cart on four axles, ropes, hinges) runs at about 0.12 ms per step at 8 workers. Per
step, links cost one sync pass (a validity check and two id compares each) and one poll of each awake link (two Box3D
getters). A joint is rebuilt only when an end's body changes.

## 2026-09-28 stress at scale, step 0: the keep, settling at load

The keep (`lp_sceneKeep`, `lpAddKeep`): a 15 m mortared stone keep of 1973 pieces and 6296 bonds (walls in running
bond, a cross wall, four wooden floors on corbels, lintels, merlons). It is the big-building case milestone 4 is for.
Baseline, one worker:

| | today |
|---|---|
| load (build and settle) | 152 ms: build 21 ms, settle 131 ms |
| settle | 189 cold iterations to 1e-3 (1.19M bond-iterations, about 67 ns each), of which about 50 ms is the slender check |
| without settling at load | 98 steps of one iteration each, then a 53 ms step judging it (the O(N·E) slender check) |
| a solving step at one iteration | 1.4 to 1.9 ms, for 0.4 ms of iteration work: the rebuild costs 3 to 4 iterations, the budget charges 2 |
| ground floor of the front knocked out (71 pieces) | decided after 153 steps, settled after 546; 12 joints break, the keep stands |
| a cannon hole in the front wall | decided after 29 steps, settled after 1632 (61 with an unlimited budget); the cells around the hole crumble |

Found while measuring: the relaxed tolerance (1e-2 of the whole structure's load, after `stressPatience` steps) lets
the residual gather on small fracture cells, whose joints then read utilizations of 4000 to 1.5 million and break in
dozens at once. The convergence test needs to be per node (the residual meter's normalization), not global.

Settling at load (`lpWorld_SettleStructures`, called by `lpBuildScene`) solves every new structure to convergence with
no per-step budget, in a few rounds while joints break (town's tower loses 10 dry joints at load). Every scene's first
step now solves nothing (`TestScenesSettle`). Every rung's hash but the barrage's is unchanged: small structures used
to converge in their first step with the same arithmetic, and nothing touches a static structure in between. The
barrage's first shot lands before town's tower had finished its second-round solve.

Object creation pairs parts by a sweep along x instead of testing every pair, and bonds them in the old order: every
rung's hash was unchanged with settling turned off.

## 2026-09-28 stress at scale, step 1: the solver on its own, systems kept while solving

Behaviour-preserving: every rung's simulation and solver hashes are unchanged (`bench.ps1 -StrictSolver`).
- `solve.c` holds the system and its math; `stress.c` builds and judges.
- A structure keeps its system on its body, so a step that continues a solve no longer rebuilds it (edges, stiffness,
  blocks and their factors). It only reloads x through the pieces, keeping the round trip bit for bit.
- The slender check walks each piece's incident edges instead of every edge of the structure (it was O(N·E)).
- Utilizations and slender sections are computed in the parallel phase.
- A split skips its flood fill when the body's topology has not changed since there was nothing to split off.

| keep, one worker | step 0 | step 1 |
|---|---|---|
| a solving step at one iteration | about 3.2 ms | about 0.55 ms (0.4 ms of it the iteration) |
| breach test, stress over 900 steps | 1744 ms | 300 ms |
| cannon hole test, stress over 600 steps | 920 ms | 347 ms |
| settle at load | 135 ms | 73 ms |

Ladder, best of 3 (1 / 8 workers): keep 6.05 / 3.72 → 4.53 / 2.37 ms per step (stress 2.2 → 1.1 ms); barrage 12.66 /
5.18 → 11.85 / 4.69 (stress at 8 workers 1.08 → 0.93); the rest within noise.

## 2026-09-28 stress at scale, step 2: change tracking, liveness, a per-node convergence test

Behaviour changes (hashes rebaselined):
- **A per-node convergence test** beside the global one: every node balances to 5% of its own load plus the forces
  through it (25% after patience).
- **A request for a check** (a hit, a link's pull) resamples loads instead of clearing `solving`: before, a structure
  hit mid-solve whose loads barely changed was dropped with its solve unconverged and never judged. Patience now
  counts across restarts.
- **Fracture:** kept cells start from their parent's solution moved rigidly.
- **Weakened joints:** a blast that weakens joints without breaking them has them judged again from the stored bond
  forces, with no solve. Before, nothing looked at them until the next topology change.
- **Rechecks:** load rechecks compare torques too, link rechecks force and torque vectors.
- **Budget:** a step continuing on its cached system is charged its iterations only.
- **Change stamps:** pieces carry them (seeds for the reduced solves to come; the debug log counts them).

Measured with the step-1 build plus only the new `stressJudged` stat, 8 workers, 600 ticks:

| rung | step avg | stress avg | solves / judged | steps per judgement | waits |
|---|---|---|---|---|---|
| town, before | 2.27 ms | 0.52 ms | 1193 / 569 | 2.1 | 0 |
| town, after | 1.93 ms | 0.57 ms | 1216 / 504 | 2.4 | 1 |
| barrage, before | 4.22 ms | 0.84 ms | 2964 / 1003 | 3.0 | 99 |
| barrage, after | 4.71 ms | 0.81 ms | 2418 / 891 | 2.7 | 25 |
| tower, before | 1.95 ms | 0.08 ms | 228 / 196 | 1.2 | 0 |
| tower, after | 1.87 ms | 0.06 ms | 171 / 139 | 1.2 | 0 |
| keep, before | 2.18 ms | 0.96 ms | 573 / 0 | never | 0 |
| keep, after | 2.31 ms | 1.06 ms | 594 / 0 | never | 0 |

Destruction is chaotic (town now peaks at 9017 pieces instead of 10073), so the smaller rungs are a wash. The keep's
row is the milestone's problem in one line: under a shot every 12 ticks it is never judged at all, because each shot
restarts its solve and at one iteration per step no tolerance is reachable in between. The per-node test makes the
first judgement after a cannon hole honest: 99 steps instead of 29 (the 29 was the global norm hiding the residual
on the new cells). Steps 3 to 5 give it more iterations per step (reduced systems) and a guaranteed judgement
(pressure allocation and audits).

## 2026-09-28 stress at scale, step 3: the reduced assembly and the delta form

Behaviour-preserving for everything that does not cluster, which is everything in play today: every rung's
simulation and solver hashes are unchanged (`-StrictSolver`).
- A partition of a structure's nodes into rigid groups (`lpPiece.cluster`).
- Its reduced system `PᵀKP`: edges between groups keep their stiffness and axes, with arms moved to the groups'
  centres of mass; edges inside a group are dropped.
- The delta form: the right-hand side is `Pᵀ(f − K x_old)`, from one fine pass; each member moves by `P y` on top of
  x_old.

The tests, world-free:
- `TestReducedAssembly`: 40 random nodes in 20 groups; `|K_r y − PᵀKPy|` is 9e-6 of 68. With every node its own
  group, the reduced system equals the fine one bit for bit.
- `TestSolveSystem`: a cantilever chain matches statics, clustered or not. After a load change near its root, the
  delta form with its tip clustered is exact on every edge (2e-4, the tolerance) in 12 iterations, against 33 for a
  fresh solve.

On the keep, a hand-made partition in `TestClusteredBreach` (the back half, one cluster per course) decides the
ground-floor breach in 67 steps instead of 153, and the same 12 joints break. It has not settled 600 steps later.
Only half the keep is clustered, so the reduced system still has about 3000 edges (a few iterations per step), and
each slender plank that snaps restarts the solve. Found on the way, for step 4:
- A structure straining only in slender pieces is re-solved every step instead of creaking from stored values.
- A fresh build of the keep (two fine passes) costs more than the per-structure cap, so its first step gets one
  iteration.

## 2026-09-28 slender pieces creak without a solve

A structure whose only strain was in slender pieces (a sagging plank, a loaded lintel) used to be rebuilt and solved
again every step until the piece snapped: creaking covered joints only. The worst section of each slender piece is now
kept on the piece at every judged solve, and creaking strains it from there. Town and the barrage change outcome
(chaotic: town peaks at 9980 pieces instead of 9017, the barrage at 18067 instead of 16334); the barrage's step
drops from 4.63 to 3.99 ms at 8 workers even with more coming down. The clustered keep breach's stress time drops from
536 to 212 ms.

## 2026-09-28 stress at scale, step 4: clusters in play, checked against an exact solve

Structures past 512 pieces (today only the keep) form rigid clusters after every exact solve and solve changes as
corrections on them. Tuning this took an **oracle**: tests can check every judged correction against an exact fine
solve of the same change (the worst joint utilization error, and how many joints one would strain and the other not).
It showed four things wrong with the first cut, each fixed:

| found | fix |
|---|---|
| The right-hand side `Pᵀ(f − K x_old)` carried the residual the settle was accepted with (up to 5% per node), so every correction chased it across the whole keep | subtract each piece's accepted residual: zero where nothing changed |
| Partial corrections were written back each step; a restart then took a rigid, unbalanced state as its baseline | write back only converged corrections |
| Rigid clusters one bond from a hole carried the arching load stiffly: joints at the rim off by 0.62 | seeds reach 3 bonds out: 0.27 |
| The meter normalized by each member's own load: a plank carrying only itself "changed 300%" and nothing survived | read the change against the joints (could it carry one past 0.5?) and against the cluster's most loaded member (is the cluster carrying a redistribution?) |

Also: slender pieces may cluster while lightly loaded, clusters grew to 128 pieces, and a solve continues only in the
mode (reduced or fine) it started in (a structure whose clusters all dissolved had continued a stale fine solve).

The keep, one worker, under today's budget (10k bond-iterations per structure per step):

| case | exact solves only | clustered | judged corrections vs exact |
|---|---|---|---|
| two stones out of the upper wall | decided after 41 steps | 12 steps | worst joint off by 0.13, no flips |
| the ground floor of the front out | decided 153, settled 592, 12 joints break | decided 207, settled 378, the same 12 break | 0.008, no flips |
| a cannon hole | decided 99, not settled in 600, 118 break | decided 26, settled 306, 122 break | 0.28, 2 of 65k flipped |
| small structures with everything past 4 pieces clustered (arch, colonnade, tower, walls, beam) | | same outcomes | 0.81 on the dry-stacked tower, 4 of 2870 flipped |

A local change is where clusters pay: the reduced system is a few thousand bonds instead of 6300. A breach
redistributes the front wall's weight into the corners: the meter reads it as not local, every cluster dissolves after
one round, and the fine solve that follows starts late. Under the ladder's constant bombardment the keep is never
judged either way, and the corrections' extra fine passes cost 0.16 ms per step (stress 0.99 → 1.15 ms): step 5's
ladder and audits are for that regime, and its budget allocation for the fine solves the budget starves.

## 2026-09-28 stress at scale, step 5: the budget shared, provisional judgements audited

- **The budget is shared by the structures that want to solve**: each gets the step's `maxStressWork` divided among
  them, but at least `maxStressStructureWork`. One solving alone may use all of it: the per-structure cap bounds a
  step's time when many solve in parallel, and is moot then. This is the lever: the keep's fine solves had been
  starved at one iteration per step.
- **Judgements on a reduced system are provisional.** They queue for an audit, an exact solve once things are calm
  (30 steps at most half the budget, nothing waiting) or once they have waited 300 steps. It is judged as usual and
  forms the clusters again.
- **The meter's last round:** if it still objects, every cluster goes and the solve is exact. A third reduced solve
  accepted unmetered had been off by 1.28, with 158 joints flipped.
- **Tried and dropped:** the plan's L1, solving only the patch around a change with the rest of the structure held
  where it was. Under fire the oracle found it flipping 70 to 240 joints per judgement: the held boundary carries what
  should have spread, false strain breaks joints, and the keep crumbled from ten shots (180 m³ against 45 without).
  The budget sharing made it unnecessary. L0 (no solve under extreme pressure) was not needed either.

The keep, one worker:

| case | exact only (step 0) | clusters (step 4) | now |
|---|---|---|---|
| local hit: decided | 41 | 12 | 2 (audited exactly at calm) |
| breach: decided / settled | 153 / 592 | 207 / 378 | 30 / 70, the same 12 joints |
| cannon hole: decided / settled | 99 / never in 600 | 26 / 306 | 7 / 67 |
| a shot every 12 ticks for 120 ticks: judged under fire | 0 | 0 | 10 |

Judged corrections against exact solves: breach 0.008 worst joint error, no flips; cannon hole 0.28, 1 of 58k
joints flipped; small structures all clustered 0.08, none flipped. A new ladder rung, 'siege', is the keep with a
blast every 4 ticks.

The price of deciding sooner is time per step while a big structure solves. One solving alone may now spend the whole
`maxStressWork` (60k bond-iterations, about 4 ms on one core here) where it spent 10k. Ladder, best of 3, 1 / 8
workers, against step 4:

| rung | step avg | stress avg | judged |
|---|---|---|---|
| keep | 4.24 / 2.41 → 6.79 / 4.75 ms | 1.14 → 3.28 ms | 0 → 11 (10 on reduced systems) |
| barrage | 9.92 / 4.03 → 10.65 / 4.75 ms | 0.87 → 1.32 ms (8 workers) | 1026 |
| siege (new: the keep, a blast every 4 ticks) | 19.43 / 10.29 ms | 5.38 ms | 3 |
| town | 4.56 / 2.12 → 3.55 / 1.94 ms | about the same | 502 (less comes down) |

On a slow machine, lower `maxStressWork`. The profiling milestone's first target is the cost of an iteration itself
(65 ns per bond: a structure of arrays for the 6-vectors, precomputed bond blocks, a parallel K·x for the biggest
structures). Step 6 (a nested-iteration settle, a two-level preconditioner) is not needed yet: settling the keep takes
65 to 75 ms at load, and audits finish within a few hundred calm steps.

## 2026-09-28 articulated objects, step 1: wheels and vehicles

A wheel is a link with no joint: a shape cast of the tyre down its suspension, a spring-damper, and an impulse solve
for grip per chassis body (see architecture.md, "Vehicles"). Box car tests (a 4 m wooden slab on four wheels):

| test | result |
|---|---|
| rest | sags 0.130 m (mg/4k = 0.129), asleep at tick 69 |
| 40 m/s for 10 s | 400 m, drift 1 mm, no bounce, speed held |
| full lock at 30 m/s | slides (16.7 m/s sideways at worst), stays upright (0.999) |
| 20 cm kerb at 15 m/s | the chassis rises 0.30 m and settles |
| 15 degree slope | the handbrake holds (no creep); released, it rolls back |
| drops | 1 m holds; 8 m tears all four wheels off, which come off as wheels |
| a plank bridge | a 0.9 t car crosses, a 3 t truck breaks 2 joints |

Cost (casts and the solve): 3.0 us per wheel at 16 wheels, 2.8 us at 256 (64 cars: 0.7 ms per step). Nothing changes
for worlds without vehicles: every ladder hash is the same. The ladder reads 5 to 18% slower than the baseline, but an
interleaved A/B against the previous commit on this machine today gives the same time (town, 1 worker: 3.80 vs 3.81
ms): the machine is slower than when the baseline was taken.

## 2026-09-28 articulated objects, step 2: the track, driving, the 'track' rung

The track scene: a ring road (45 m radius) with two stone kerbs, loose crates, a 0.7 m hump, a plank bridge on two
piers and a brick wall outside the bend, and three box cars (wooden floor pan, cabin, hood and boot on four wheels,
about 2 t). `lpSceneDrive` steers each for a point 12 m ahead on the ring at 14 m/s, from simulation state only. All
three lap in 1300 ticks (22 s), whole and upright. The sandbox drives: V takes the nearest car, keys become `drive`
events recorded when the controls change, a chase camera follows (`--follow` in replays). The track demo under a blast
every 12 ticks is identical at 1, 4 and 8 workers.

New ladder rung 'track' (a blast every 30 ticks at the cars): 0.50 / 0.30 ms per step at 1 / 8 workers, 487 pieces.
Under a blast every 12 ticks the wooden cars are splinters within seven seconds, wheels lying about: crash and blast
calibration is step 5's (sheet metal on bolts). Every other ladder hash is unchanged.

## 2026-09-28 articulated objects, step 3: part identity and part detonators

Pieces remember their object (`userId`), part and the part's system (tag, channels carried, fed and needed, a share of
the object's sources by volume); fracture cells inherit them. Detonators moved from bodies to parts, shared by every
piece made from one: a fuel tank goes off alone and takes only its own pieces, a tank torn off stays volatile and goes
off once. `lpPiece` grows from 264 to 288 bytes. Every ladder hash is the same, the yard's volatile crates included
(their object detonators behave exactly as before).
