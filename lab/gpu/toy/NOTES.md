# The toy: running notes

The log of the toy's build (design in `DESIGN.md`): per step, what landed, what its gate measured (hashes, counts),
judgement calls, open issues, and what is next. Newest entry last. Agents append here before they stop.

## 2026-10-04: started

The GPU lab (E11 ported into `lab/gpu`) reproduces E11's hashes on Windows (MSVC, clang-cl) and on the Spark (gcc,
clang; GB10 and llvmpipe match their twin bit for bit in F, I32, I64 and V4). Findings that shape the toy:
- the GB10's driver (580.82) crashes compiling the F kernels when `RoundingModeRTE` is declared: F-plain only;
- NaN bits differ by machine (the lab's Fnocap): a NaN anywhere is a failure, not a value;
- unsequenced random draws made E11's corpus compiler-dependent: draw in separate statements.

## 2026-10-04: steps 1 and 2 (helpers and battery; layouts, scenes, quantisation, CPU stages, twin driver)

**What landed** (lines): `layout.h` 298 (dialect types and V4 formats, angular velocity Q11.20; hulls, bodies split
into state / pose / mass, `Aabb`, `Pair`, `Manifold` and `ManifoldPoint` for step 3, `Params`, `Push`, `Hash2`,
battery vectors), `num.slang` 748, `battery.slang` 166, `battery.c` 1,090, `battery_glue.cpp` 44, `scene.c/.h`
611/62, `quant.c/.h` 323/46, `stages.c/.h` 452/67, `kernels.slang` 176, `kernels_glue.cpp` 109, `toy.c` 757,
`rng.h` 102 (PCG32 and draws), `fpflags.h` 74 (the sentinel), `gen_toy.py` 70, `CMakeLists.txt` 50, `gate.py` 96 (both
gates in one command); about 5,300 in all. Hooks outside `toy/`: `lab.py gen` ends by calling `gen_toy.gen_toy()` when
`toy/gen_toy.py` exists (a full `lab.py gen` ran: E11's outputs byte-identical, the toy's added), and the lab's
`CMakeLists.txt` adds `toy/` when `gen/toy` exists. `gen_toy.py` also runs alone. Binaries: `battery_F/_V4/_D`,
`toy_F/_V4/_D` in `build/<compiler>/bin`, run from `lab/gpu`.

```
python lab/gpu/toy/gen_toy.py && python lab/gpu/lab.py build --compiler msvc && python lab/gpu/lab.py build --compiler clang-cl
python lab/gpu/toy/gate.py                      # gates 1 and 2, logs in build/toy-scratch/gate (about 10 s)
build/msvc/bin/toy_V4.exe --scene pile200 --ticks 600 --threads 1,8 --pos-ref build/toy-scratch/gate/pile200.D.pos
```

**Gate 1: passed.** Each dialect's battery (16 helpers, 4,096 vectors each; lpDivQ31 79,976 including a sweep of
divisors near 1 and the limits against the same numerators at 10 shift classes, and a class of quotients just under
2^31 - 1 for every divisor size; divClamp 16,384) on the RTX 3060 (driver 0x988f0000) and the UHD 630 (0x194859),
F-plain: every output word identical to the twin in every helper, saturation counts too (V4: 725 in mul, 2,100 in
sum2/dif2, ..., 17,938 in lpDivQ31). Battery hashes, equal on MSVC 19.42.34435 and clang-cl 23.1.2: **F
2b11786b345dc01f, V4 50aaf74f7d01b6fe, D 5db9d655b22c9968** (`--n 4096 --seed 20261004`, the defaults).
`lpDivQ31` equals C's `/` on every input: 0 of 79,976, and 0 of 1,063,016 at `--n 65536 --seed 7` (twin and both
GPUs, battery hash 6894d90a8ee222b1 on both compilers). The twin's floating-point status is clean after every helper
except F's snap, whose subnormal inputs set the denormal flag by design. Accuracy against C's doubles (a report): F
recip 8.8e-8 relative, rsqrt 8.9e-8, |quatNormalize| - 1 1.3e-7, divClamp 1.3e-7; V4 rsqrt 2.7e-8, |quatNormalize| - 1
2e-9, recip and divClamp below 1 lsb (truncated); atan2 2.8e-5 rad in every dialect (lpAtan2's polynomial).

**Gate 2: passed.** 600 ticks, 1 and 8 threads, MSVC and clang-cl: per-tick hashes (bodies and stages) identical
across thread counts and compilers in every dialect, floating-point sentinel clean after every dispatch, saturations
0. Run hashes (an FNV of every tick's two hashes):

| scene | D | F | V4 |
|---|---|---|---|
| stack10 | 9fd7beb412959ce4 | a31b90a2beeb8c99 | d815f7db0352baea |
| pile200 | 83d78923a458f806 | ab414d9013c1c8df | c97b8c7ef3ea182d |
| pile200, gravity 0, body 112 pushed (sleep, wake) | 17884cfbe1e2b27a | 294119e0913ca824 | 6c7dbaf910c78405 |
| chip (10 m/s, 40 rad/s) | 8025ae0dc20ebfe9 | fb8d93f952c1ce6b | 8b9e728c30c9d31f |
| ramp | 10c613893e3d9b2a | e659316259462f98 | 59fb18a494cc885a |

Sensible counts: stack10 starts with 10 pairs (ground and the 9 neighbours), 2 colours, one island of 10; the stack
falls through the ground (pairs with it come and go) and ends with the 9 chain pairs. pile200 (205 bodies: floor, 4
walls, 160 boxes, 40 chunks of up to 14 vertices / 9 faces / 42 half-edges, 16 chunks redrawn) starts with 1 pair and
199 islands; as the layers pass the floor (the walls stay clear), pairs rise to 76 (tick 56) and colours to 2, then
fall back to 1. Without gravity,
197 bodies sleep at tick 31, the pushed body wakes bodies on its way (reset to rest, asleep again 30 ticks later), and
by tick 90 only it is awake. Free fall against D, dynamic bodies (stack10 and pile200 alike):

| | ticks 1-120: rms / max | ticks 1-600 (a 500 m drop): rms / max | rotation (chip), 600 ticks: rms / max |
|---|---|---|---|
| F | 9.9e-6 / 2.5e-5 m | 6.2e-4 / 1.4e-3 m | 7.8e-6 / 1.4e-5 rad |
| V4 | 1.7e-5 / 3.8e-5 m | 4.3e-4 / 9.5e-4 m | 1.2e-7 / 2.1e-7 rad |

F's error is its float positions (Box3D's); V4's is the quantised h g (7.9e-8 m/s too much per substep, so 9.5e-4 m
after 2,400 substeps, systematic). D's body lands where y0 - g h^2 n (n + 1) / 2 says, to the printed digit.

**Findings.**
- MSVC folds `x < 0 ? -x : x` into `fabs` in C (so -0 becomes +0) and clang does not: the battery's own input generator
  made different vectors on the two compilers (404 of them) until it used `fabs`. F's snap already sends -0 to +0; D's
  snap now does too (`x == 0 ? 0 : x`), so the D twins agree whatever a compiler folds. C code in the toy uses `fabs`.
- F's Newton steps written y (3/2 - x y^2 / 2) give rsqrt(1) = 1 - 2^-24, so renormalising a unit quaternion shrank it,
  and resting orientations drifted 1.4e-5 rad in 10 s against D. Written as a correction, y + y (1/2 - x y^2 / 2), they
  are within 1.7 ulp (from 2.4) and exact at powers of four; the reciprocal likewise (1.5 ulp from 2.5, exact at powers
  of two). The drift is gone (3.7e-8 rad, constant).
- lpDivQ31's estimate lands 0 to 13 below the quotient (a Python emulation over 157k inputs, half of them quotients
  near the limit); the correction has room for 31.

**Judgement calls and deviations from DESIGN.md.**
- V4 positions are two words (low, signed high) in `Pos3`, not an int64 field: no int64 anywhere in a kernel outside
  `mas` (the sum of up to three products, the rounding shift, the counted narrow), the same bytes as an int64. The
  element hash is two 32-bit murmur3 lanes for the same reason.
- lpDivQ31( n, d, k ) = n 2^k / d truncated toward zero (C's `/`), n and d in +-(2^31 - 1), k in [0, 32]; beyond +-(2^31
  - 1), or d = 0 with n != 0, it saturates and counts. The divisor is clz-normalised; y ~ 1/x in Q1.30 for x = (dn/2 +
  1)/2^30 (so y < 2 fits) from 48/17 - 32/17 x and three Newton steps; the estimate is biased 8 low; the remainder is
  exact in two words (mas's rounded high word less the low word's top bit); five binary steps finish. divClamp( n, d,
  k, lim ) gives +-lim without counting when |n| 2^k >= lim |d| (exact, two words). F's and D's divClamp ignore k.
- V4 rsqrt( u, su, sy ): clz to x in [0.5, 2) by an even shift, the line 1.5604 - 0.4714 x, four Newton steps; su must
  be even (every format is). F rsqrt: the 0x5f3759df seed and three steps; F recip: exponent bits, 48/17 - 32/17 m,
  three steps.
- The AABB is computed in prepareBodies (the brief), not also in finalize (DESIGN's step 7): prepare runs at the start of
  a tick over last tick's awake bodies (every body on the first tick). Cells are inclusive, [floor(p) + floor(lo),
  floor(p) + floor(hi) + 1] at 2^-10 m (conservative by up to a cell). The margin is `Params.aabbMargin`, 0.05 m (Box3D's
  largest), around a fresh box each tick (no fat AABBs kept).
- Sleep: a body is under the thresholds when |v|^2 + |maxExtent w|^2 <= 0.05^2 (Box3D: |v| + |arc| <= 0.05, plus a
  position term); each component is gated first, so nothing saturates. An island sleeps when its smallest counter is
  30. Waking resets the body (a sixth kernel, `wakeBodies`: Box3D's identity state on wake); a woken body's counter
  counts as 0 that tick (the CPU's copy is stale).
- Step 2 has no manifolds, so every broadphase pair counts as touching for waking, islands and colouring
  (`stages_tick`'s `prevTouching` is NULL); step 3 passes last tick's pointCount > 0 by prevIndex. Pairs with a
  sleeping body stay in the key list, inactive, so their manifolds can carry over; static-static pairs are skipped.
  Colours: 64, then an overflow colour.
- The convex clipper is a half-space intersection (vertices from plane triples, faces sorted by a pseudo-angle, twins
  adjacent as Box3D's); chunks with an edge under 2 cm or over the limits are drawn again. Mass properties from
  tetrahedra (a box's checks against V (b^2 + c^2) / 3). Face planes are recomputed from the quantised points (Newell),
  offsets rounded up.
- Scenes: stack10's cubes are 0.5 m on a side (density 1000); pile200 is 8 layers of 5 x 5 at 0.7 m from 3 m up,
  half extents 0.1 to 0.2 m, random orientations, every fifth a chunk; bounce, ramp, ratio, chip are simple versions
  (bodies placed, materials stored, no contacts yet). Gravity (0, -10, 0).
- F's Params follow Box3D's float arithmetic (h = dt / 4, inv_h = 4 inv_dt); the speed caps are E11's (no sqrt).

**Open issues.**
- F's first-120-tick free-fall error (9.9e-6 m rms) sits at DESIGN's 1e-5 bar, from float positions alone; if
  contacts add to it, F may want V4's two-word positions.
- V4's quantised gravity delta drifts 1 mm in 10 s of free fall; a finer format for h g (or v) would remove it.
- Only the battery has run on the GPUs; the kernels' SPIR-V is generated and passes spirv-val (Vulkan 1.3) and the
  NoContraction audit, but the Vulkan tick path is step 5.
- No UBSan build yet (DESIGN's gate); signed shifts of negative values are written through uint, right shifts of
  negative values are arithmetic (implementation-defined in C++17, every compiler here).
- Debug aids: `TOY_DUMP_AABBS=T` (toy, 1 thread) and `TOY_DUMP_INPUTS=FILE` (battery).

**Step 3 starts from:** `kernels.slang` (add `narrowSat` and `narrowClip` there; bindings 12 on for the pair list and
the two manifold buffers), `layout.h`'s `Manifold` / `ManifoldPoint` (defined, unused) and the hull buffers (points
Q8.24, faces with Q1.30 normals and Q8.24 offsets, packed half-edges), already uploaded at bindings 1 to 3 but read by
no kernel yet. The pairs come from `Stages` (`pairs`, `prevIndex`, `active`; the solver's lists in `colourStart` /
`colourList`); `toy.c`'s tick has one CPU point between the prepare dispatch and the rest, where narrowphase dispatches
over the active pairs go, and `stages_tick` should get the manifolds' pointCount > 0 (by prevIndex) as `prevTouching`.
Add the narrowphase helpers to `battery.slang` as they appear, and the 10k-pose corpus as a new runner beside
`battery.c`.

## 2026-10-04: step 3 (the narrowphase: narrowSat, narrowClip; the 10k-pose corpus)

**What landed** (lines, now): `narrow.slang` 1,303 (new: the narrowphase, included by `kernels.slang`: `narrowSat`,
`narrowClip`, `copyManifolds`, the SAT queries, incident face, clipping, reduction, edge contact, the diagnostics),
`narrow.c` 1,602 (new: the corpus runner, `narrow_F/_V4/_D`), `kernels.slang` 224 (bindings 12 to 17: pairs, manifolds,
prevManifolds, satAxes, narrowDiags, manifoldHashes; `hashManifolds`), `num.slang` 877 (the division fix; `posDelta`,
frames, `unit3`, `len3`), `layout.h` 371 (`SatAxis`, `NarrowDiag`, `AXIS_*`, `SAT_*`, `CLIP_MAX`, the narrowphase's
Params), `toy.c` 863 (the wiring), `battery.slang` 249 and `battery.c` 1,208 (three helpers), `kernels_glue.cpp` 131,
`scene.c/.h` 622/70 (`scene_box_hull`, `scene_chunk_hull` public, the draws unchanged), `quant.c` 332, `gen_toy.py` 71,
`CMakeLists.txt` 57, `gate.py` 143 (gate 3; `--gates`). About 3,700 lines added (2,900 of them the two new files).

```
python lab/gpu/toy/gen_toy.py && python lab/gpu/lab.py build --compiler msvc && python lab/gpu/lab.py build --compiler clang-cl
python lab/gpu/toy/gate.py                      # gates 1, 2 and 3 (about 15 s); --gates 3 for the corpus alone
build/msvc/bin/narrow_D.exe --out build/toy-scratch/n.bin && build/msvc/bin/narrow_V4.exe --ref build/toy-scratch/n.bin --list 8
```

**Gate 3: passed.** The corpus: 10,000 pairs from the scene generator's hulls (4 floors and walls, 60 boxes with four
of stack10's cube, 60 chunks), box-box 4,066, box-chunk 2,963, chunk-chunk 2,971; faces aligned as in a stack 1,045,
faces twisted 1,949, tilted 1e-3 to 0.1 rad 1,574, within 1e-4 rad 963, edges crossing 1,997, a vertex on a face 1,484
(half exactly along the vertex's mean normal, half tipped off it), generic 988; separations deep (-0.15 to -0.04 m)
1,339, shallow 2,734, inside the speculative distance 2,255, at it (0.02 m +-2e-5) 423, just outside (to 0.03 m)
1,344, far 917. Pass 1 runs each pair fresh; pass 2 moves B by up to 0.5 mm and about 1 mrad and reads pass 1's
manifolds as last tick's, their impulses replaced by markers (so a carried impulse names the point it came from). D's
paths: pass 1, 6,976 touching and 3,017 separated; pass 2, about 6,530 keep the cached feature's contact and 2,960 stay
separated by the cached axis.

| | F | V4 |
|---|---|---|
| pass 1: axis ties / clip ties (of 10,000) | 477 / 336 | 477 / 336 |
| axis kind and index differ outside axis ties | **0** of 9,523 | **0** of 9,523 |
| point count or ids differ outside all ties | **0** of 9,187 | **0** of 9,187 |
| support vertex differs outside its own ties | 0 of 4,901 (3,762 ties) | 0 of 4,901 |
| pass 2 (pairs whose pass 1 agreed): ties | 8 + 22 of 9,518 | 9 + 20 of 9,525 |
| pass 2: axis, ids, warm-start markers differ outside ties | **0, 0, 0** | **0, 0, 0** |
| normal error rms / max | 2.1e-7 / 1.3e-5 | 1.9e-7 / 9.3e-6 |
| anchor error rms / max (pass 1) | 3.0e-7 / 2.7e-5 m | 1.4e-7 / 8.0e-6 m |
| separation error rms / max | 1.3e-7 / 4.7e-6 m | 9.6e-8 / 4.8e-7 m |
| SAT face separation error rms / max | 1.6e-7 / 3.3e-6 m | 6.2e-8 / 2.6e-7 m |
| both GPUs, both passes: SAT records, manifolds, diagnostics vs the twin | every word identical | every word identical |

GPUs: the RTX 3060 (driver 0x988f0000) and the UHD 630 (0x194859), F-plain and V4, saturations 0 as the twin's (a
preview of decision point 2: the narrowphase is bit-exact on both). Corpus hashes (manifolds and SAT records, both
passes), equal on MSVC and clang-cl: **D c99764ed09a826b7, F e9b4716289b5513d, V4 305aa1bd8412ad08**; the twins'
sentinel clean, saturations 0. The largest anchor errors are conditioning, not arithmetic: in pass 1 an edge contact
of nearly parallel edges (the closest point moves by the distance error over the sine), in pass 2 (1.0e-4 m F, 6.7e-5
V4) an aligned stack whose incident edge lies 2.3e-5 m from a side plane (the clip fraction's |d1| / (|d1| + |d2|)).

**Ties.** A tie is a decision whose closest call in D was within the tolerance: kernels record each decision's margin
in `NarrowDiag` (written when `Params.narrowDiag` is set: the corpus, not the toy). An *axis tie*: a SAT decision on
the axis's path (best against second-best face or edge pair, a separation against the speculative distance, B's face
against A's by faceTolerance, the edge against 0.9 x the clipped face plus faceTolerance, the Gauss-map and parallel
tests of the chosen pair, the cache's drift against linearSlop); or, when the axis hangs on a contact's outcome
(touching or not, the clipped separation), whether that contact touches (its clipped separation against the speculative
distance, a clip pass ending with 2 or 3 vertices, the edge contact's segment ends, a runner-up support vertex with
another incident face) or which incident face it clips. A *clip tie*: a clip distance, a kept separation or a reduction
score. The tolerance: 1e-5 m (areas 1e-5 m^2, cosines 1e-5). Why: the verdict is the same at 1e-4 and at 1e-6; at 1e-7
the first differences appear (three support vertices in F, one id set in V4, every one with D margins between 1e-7 and
1e-6 m), so the decision noise is about 1e-7 m and 1e-5 m is 100 times it, and over twice the largest error of any
decision quantity (F's SAT face separations, 3.3e-6 m). Who ties: aligned stacks (350 of 1,045, 276 to 285 of those with
other ids: every incident vertex sits on a side plane) and vertices exactly on their mean normal (333 of 1,484: a box
vertex's three faces tie for the incident face); every other pose about 1 to 2%.

**Gates 1 and 2: passed** (`gate.py`, MSVC 19.42.34435 and clang-cl 23.1.2). Battery: 19 helpers (new: `unit3/len3`,
`posDelta`, `frame`), both GPUs identical in F and V4; the 16 old helpers' outputs and hashes unchanged by the division
fix; battery hashes **F 3ad4fb70e1a79038, V4 cbbf92da69bcf218, D 1c2d51d801f3a44f**. |unit3| - 1: F 1.3e-7, V4
4.6e-9, D 3.3e-16. Toy, 600 ticks, 1 and 8 threads, both compilers, sentinel clean, saturations 0 (run hashes now cover
bodies, manifolds and stages):

| scene | D | F | V4 |
|---|---|---|---|
| stack10 | 8d3d7d843172e43a | 9cc14fa256e17b98 | dc7a6b74c85ce9af |
| pile200 | f9f1c5eec440458b | e6245cd2f41ee5fe | 617cd170f7f9802b |
| pile200, gravity 0, body 112 pushed | c18c498a624405fd | ff2bfe031741d08c | d2d3dd8ba3d76104 |
| chip | 030e4a59a2bbd406 | 01bbbad48632148b | 6c3bea66d00f7a13 |
| ramp | f085246842067e47 | d433941b6c223b9c | ce039119b913c349 |

With no solver, the bodies' per-tick hashes of stack10, pile200, chip and ramp equal step 2's in every dialect (the
scene refactor and the integration are untouched); the manifolds and stages are new. stack10 has 10 touching pairs of
4 points from tick 1 (from tick 2, when the stages see them: 2 colours, one island of 10) and peaks at 13 as the stack
sinks through the ground; pile200 peaks at 67 touching; without gravity 198 bodies sleep at tick 31 and the pushed body
wakes a sleeper through a real contact at tick 33.

**The fix from the Spark.** Every division and square root in D now divides (roots) a value selected safe before the
test that discards it (`ds = d != 0 ? d : 1`, then the select), so a compiler that evaluates it ahead of the select (clang
18 on aarch64 did, for `recipT`) raises no flag; F's software reciprocal and rsqrt get the same guard (rsqrtPos(0) would
overflow and then go NaN if speculated). The sentinel stays strict. Not yet rerun on the Spark.

**Judgement calls and deviations.**
- **Box3D's scalar path is stale.** The brief's `b3CollideHulls` at line 2534 sits under `#else` of
  `B3_SIMD_COLLIDE_HULLS 1` ("this code has gone stale, will be deleted soon"); the compiled one differs: face A unless
  B's separation is larger (no 0.5 linearSlop bias); the edge contact when the face's is empty or the edge beats the
  clipped separation by linearSlop (no 0.9 factor); a failed edge contact keeps the face's cache; the Gauss-map test
  takes products below -1e-4 m^2 (a scale-dependent tolerance); early exits at > rather than >= the speculative distance;
  bounds that skip candidates; the support vertex chosen with the index in the low mantissa bits. I followed the brief
  (the scalar path's structure and tolerances) and put every tolerance in Params (`faceTolerance`, `edgeRelTolerance`,
  `parallelTolerance`, `reduceBias`, `speculativeSq`, `cacheTolerance`), so step 4b can move to the compiled values if
  b3ref2 disagrees; the cache-clearing and the product tolerance would be code changes.
- **The cache as data.** The cache lives in the manifold (`axisType`, `axisA`, `axisB`, `axisSeparation`), carried by
  prevIndex for every pair, touching or not. `narrowSat` tests the cached axis (a face: its support, >=; an edge pair
  whose arcs still cross, >); when it still separates, the pair is done and the cache kept. Otherwise it also runs the
  full SAT and records both, and `narrowClip` tries the cached feature's contact first, kept while touching and within
  linearSlop of the cache's separation (Box3D does not refresh the cache then: the drift is measured from the last full
  SAT); only when that fails does it use the SAT's verdict. Box3D skips the SAT when the cached contact holds; the toy
  computes it anyway (a GPU cost, two dispatches instead of a dependent four; step 7's timing will say).
- **Edge contact** in unit directions: t1 = (w . (uB x m)) / sin, t2 = (w . (uA x m)) / sin, compared with the edges'
  lengths, instead of b3LineDistance's 2x2 system in |e|^2 units (whose m^4 determinant underflows V4's formats for 2 cm
  edges); the same points. The parallel-edge test unsquared (max(|cba|, |dba|) < tol |eA|, the same test, no Q11.20
  underflow); the Gauss-map signs by comparison, never by product.
- **Formats.** Separations, distances and points Q8.24 inside the narrowphase (lsb 6e-8 m); stored separations and the
  cache's Q9.22 (DESIGN), the cache's drift compared in Q9.22. Areas and squared distances Q11.20, cosines Q1.30. V4's
  `unit3` is scale-free: the largest component shifted into [2^28, 2^29), the rsqrt in Q3.28, 30 bits at any length.
- **Incident face:** b3FindIncidentFace from the support vertex's lowest-index outgoing half-edge (the layout has no
  vertex-to-edge table; Box3D's hull builder picks one); only ties can tell them apart.
- **The support vertex** stored beside a face axis is never read again (the cached path finds a fresh one), so the gate's
  "indices" are the axis's (the face, or the edge pair); the vertex is still checked, outside its own ties: 0 differ.
- **Pass 2** is compared only where pass 1 agreed on the axis and the ids (elsewhere the inputs differ).
- **Touching for the stages** is last tick's manifolds (DESIGN): a pair joins islands, waking and colouring the tick after
  its manifold first has points, and tick 1 has none. Inactive pairs (no awake body) carry their manifolds
  (`copyManifolds`); static-static pairs never appear.
- **Warm starts** as Box3D's b3UpdateContact: each new point takes the impulse of the first unclaimed old point with its
  feature id; the friction and twist impulses (and V4's eP) carry while the pair keeps touching, zero otherwise. Anchors
  are from each body's centre of mass (the toy's hulls are about it, so Box3D's later centre shift is folded in), normal
  and anchors in the world frame by A's rotation.
- **The tick** is now: prepare; CPU stages (with prevTouching); wake, `narrowSat` and `narrowClip` over the active pairs,
  `copyManifolds` over the inactive, the substeps, finalize, `hashElements` over the bodies, `hashManifolds` over every
  pair (category 2, each element seeded by its two bodies, every field and every point slot).
- **The corpus** is built in double by `narrow.c` from `scene_box_hull` / `scene_chunk_hull`, every draw in its own
  statement; its bodies are static (no masses needed) and quantised by `toy_quantize`, so the corpus hash equal on both
  compilers covers the generator too.
- `gate.py` now makes `--out` absolute: a relative one made the toy's `--pos-out` fail and crash (the binaries run from
  `lab/gpu`).

**Open issues.**
- The scalar-versus-compiled question above, for step 4b against b3ref2.
- The full SAT runs for every pair the cached axis does not separate; the clip buffers (2 x 32 vertices) sit in private
  memory; the margins are computed in every dialect even when not written (cheap; a define could drop them for timing).
- The narrowphase is bit-exact on both GPUs in the corpus, but the toy's tick path has not run on a GPU yet (step 5).
- Not yet rerun on the Spark (the division fix, the aarch64 twins, the GB10 and llvmpipe on the corpus).
- Still no UBSan build; V4's margins never subtract from their sentinel (2^31 - 1).
- `NarrowDiag` and `SatAxis` are per-pair scratch; `SatAxis` could shrink once step 5 measures bandwidth.

**Step 4 starts from:** `toy.c`'s tick, between `copyManifolds` and the substeps: `prepareContacts` over the coloured
pairs (`st.colourStart` / `st.colourList`; pairs touching last tick, so a contact that appears this tick waits a tick:
decide whether to colour by this tick's manifolds, one more CPU point and readback, or accept the delay) and
`prepareJoints`; then per substep: integrate velocities, warm start, solve (push) and relax per colour, integrate
positions; restitution; finalize. The manifold this tick writes is `mf[t & 1]` (binding 13), read by the solver and
written back with its impulses so the next tick's `narrowClip` carries them by feature id. In each `ManifoldPoint`,
narrowClip fills `anchorA`, `anchorB` (Q8.24, world, from each centre of mass), `separation` (Q9.22), `featureId` and the
carried `normalImpulse`, and zeroes `baseSeparation`, `totalNormalImpulse`, `normalMass`, `shN`: prepare fills those,
and V4's `eP` (carried today) and the impulse formats are still to be chosen; the normal is world, Q1.30, from A to B.
Params has no contact hertz, damping, restitution threshold or friction mixing yet; the bodies' world inverse inertia is
`prepareBodies`' `invIw` (mantissa with `eI`).
