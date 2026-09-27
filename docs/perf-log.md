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
