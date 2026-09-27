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
