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
