# M7 track T4: fixed-point (integer) physics and geometry

Feasibility, formats, numerical pitfalls at our scale, CPU and GPU cost, and where the integer boundary should sit.
Written 2026-09-29. Sources are dated where it matters; "unverified" marks claims I could not check against a
primary source (and says what would).

## Summary

- A fixed-point Box3D already exists: **fixed3d** (Glenn Fiedler, MIT, 2026), Q48.16 in `int64`, 128-bit
  intermediates, all 24 Box3D test suites passing, the float SIMD gone. Measured **2.24x** the float runtime
  (geometric mean; 1.71–3.21x by scene) on an Apple M3 Ultra at 4 workers, 2.13x with an integer NEON narrowphase;
  an earlier run gave 2.51x (1.54–4.46x). Its own profile says the remaining gap is 64x64->128 multiplies against
  float FMA. No x64 fixed-vs-float column was published (unverified: run it on our machine, wave 2).
- For our ranges the format decides everything. Q16.16 is dead on arrival (a car's impulse overflows it). Q48.16
  loses the inertia of a 0.05 kg chip entirely (6e-6 kg·m² is 0.4 of its 1/65536 resolution) and quantises a
  3 t mech's inverse mass to 22 steps, so every Q48.16 engine keeps a second, 40-bit "inverse" scale for inverse
  mass and inertia (fixed3d does). **Q32.32 with 128-bit products survives every number we have**, from a chip's
  inertia (2.7e4 steps) to km-scale positions (±2.1e9 m), and is the format a fresh engine should use.
- Cost. Scalar x64 and ARM64 do 64x64->128 in one instruction pair (MUL r64: 2 uops, 1 per cycle; M1: MUL and
  UMULH 2 per cycle), so scalar fixed is 2–3x scalar float per multiply-heavy kernel and 6–10x Box3D's 4-wide SSE2
  solver, which has no 64-bit lane multiply to fall back on (PMULUDQ is 32x32->64; a 2-lane 64x64->128 is ~10
  uops). On GPUs a Q32.32 multiply costs 8–10 native instructions on NVIDIA (IMAD.WIDE, at half the FP32 lane
  count on Ampere/Ada/Hopper) and ~4 quarter-rate multiplies on AMD RDNA 3: **10–20x FP32 issue cost per multiply**,
  before doubled register pressure. Integer SIMD and GPU paths are bit-identical to scalar by construction, which is
  the one thing fixed point gives for free that float never will.
- The toaster budget cannot absorb it: town physics is 3–9 ms a step at 8 workers and the whole step is 12.5 ms at 1
  worker; 2.2–2.5x on the physics stage puts the 1-worker step past the 16.7 ms frame.
- The fracture geometry is a different question. Sites snapped to an integer grid make every Voronoi bisector an
  exact integer plane, and the Nehring-Wirxel 128-bit configuration (2.5 M convex mesh-plane cuts per second per
  core) fits our bit budget with room to spare. It removes the plane-shift hack in `lpPoly_Clip` and every
  tolerance-flip in the fracture, costs a few hundred lines, and is worth doing regardless of networking.
- "Quantise the state to a grid each tick so small float differences converge" is a mirage by four to six orders
  of magnitude: at the grid Fiedler found necessary for stable stacking (1/4096 m) a 1500-body world with ulp-level
  cross-platform differences flips about 80 values per tick; a grid coarse enough for an hour without a flip is
  kilometres wide.

Verdict on H3: **confirmed in cost (2.2–2.5x measured on the physics stage, not 2–4x on everything), partly
falsified on "forfeits Box3D" (fixed3d is a maintained fixed-point Box3D; what is forfeited is its SIMD, upstream
cadence and our patches), open on "solves a problem flags and tests solve" pending T3's H1, and confirmed on exact
predicates for fracture geometry being worth it for robustness alone.**

## 1. Formats against our ranges

The quantities (repo facts): fracture cells 7–20 cm (`fragmentSize` 0.10–0.30 m, `src/world.c:26-56`); tier
volumes 3.4e-6 / 7.3e-4 / 1.06e-2 m³ (1.5, 9, 22 cm cube edges); densities 150–7800 kg/m³ (`docs/materials.md`);
a 0.05 kg stone chip is a 2.8 cm cube with inertia 6.3e-6 kg·m², a 9 cm stone ghost 1.75 kg and 2.4e-3 kg·m², a
22 cm light piece 25 kg and 0.2 kg·m²; cars 1.7 t, mechs 3 t; inertia padding adds `mass * inertiaRadius²` to the
diagonal (`src/world.h:747-759`); buildings 20 m; km maps (roadmap 12); dt 1/240 (4 substeps of 1/60); contact
hertz 30, damping ratio 10, contact speed 3 m/s, restitution threshold 1 m/s, hit threshold 1.5 m/s; stress limits
0.3–500 MPa and 1e12 Pa sentinels (`src/world.c:26-70`); clip tolerance 2e-3 m and contact area threshold 1e-4 m²
(`src/world.c:557-566`).

Box3D's soft step at our settings (`extern/box3d/src/solver.h:265-307`): omega 188.5 rad/s, biasRate 9.07 /s,
massScale 0.942, impulseScale 0.0577. Damping is Padé, `1/(1+h c)` (`solver.c:86-93`). Rotation is integrated as
`normalize(q + 0.5 h w q)` (`math_internal.h:205-217`, `solver.c:219`, `solver.c:724`), so every substep does one
4-term sum of squares, one sqrt and one reciprocal per body. The contact push loop per manifold point
(`contact_solver.c:449-500`) is two quaternion rotations of anchors, one dot, two cross products, a dot, a
multiply-add, a clamp and two `I^-1 (r x P)` applications: about 45 multiplies and 40 adds, no divisions or
square roots; the divisions and square roots live in prepare (`normalMass = 1/kNormal`, `contact_solver.c:218`)
and the friction clamps (`sqrtf`, lines 636, 677), once per point per step, not per iteration.

| format | range | resolution | where it fails for us |
|---|---|---|---|
| Q16.16 (libfixmath, 0 A.D.'s CFixed_15_16) | ±32768 | 1.5e-5 | A 1.7 t car at 20 m/s carries 34 000 N·s of impulse: overflow. A chip's inverse inertia (1.6e5) overflows. Stress in Pa overflows by 10^4. Squared distances overflow past 181 m. Dead on arrival for 3D at our scale; fine for 2D fighting games and RTS unit logic (0 A.D. checks overflow in debug only). |
| Q48.16 (Photon Quantum FP, SG Physics 2D, fixed3d) | ±1.4e14 | 1.5e-5 | Chip inertia 6.3e-6 kg·m² is 0.4 of one step: rounds to 0 or 1.5e-5 (2.4x too big). A 9 cm ghost's inertia is 155 steps (0.6 % error), fine. A 3 t body's inverse mass is 22 steps (4.5 % error) unless inverses carry a wider scale (fixed3d's "40-bit inverse scale": 3.7e8 steps). Rotation below 1.5e-5 rad per substep (0.2°/s) does not move; normalisation noise random-walks orientation by ~0.8° per hour (1.5e-5 × sqrt(864 000 substeps)). Products need 128-bit intermediates or an operand-range rule: Photon's fast multiply is undetected overflow when the true product exceeds 2^31 ("stay in Int16 range" per its manual); an inverse inertia of 1.6e5 times a 0.1 N·m·s impulse already exceeds it, and inverse-scaled operands overflow the 64-bit product immediately. GJK on features under 1 cm works in squared lengths of 1e-4 m² = 6.5 steps: noisy simplex decisions on sliver hulls (risk, not measured). |
| Q32.32 (FixedMath.Net's Fix64 is Q31.32; dbox2d) | ±2.1e9 | 2.3e-10 | Nothing in the physics at our scale. Positions to ±2.1e9 m; chip inertia 2.7e4 steps; km-scale squared distances overflow past 46 km, so broadphase and GJK distances must be relative or body-local (Box3D's `deltaPosition` already is: `body.h:171-172`). Every multiply is a 64x64->128 with a 32-bit shift: no operand-range rule to police. The stress kernel's stiffnesses (`area/len`, MPa limits, 1e12 sentinels: `src/stress.c:487-491`) span 12 decades: keep it in double. |
| mixed per-quantity (state Q32.32, inverses Q24.40, dt as an exact power-of-two substep, angles Q2.61) | per quantity | per quantity | What a fresh engine should do. Every formula needs a range annotation; the compiler cannot check one. With dt = 1/256 instead of 1/240 the `v h` products become shifts (Box3D's h in Q48.16 is 273/65536, a 0.024 % error in the substep length that would otherwise be baked into every velocity integration). |
| 64-bit integer world position + float or fixed local math | ±9e18 units | unit | Solves km maps, not determinism: the local math is still float (T3's problem) or fixed (this table). Box3D's `BOX3D_DOUBLE_PRECISION` does the same with doubles at ~3 % overhead (fixed3d README, quoting Box3D's 20 000 km world). Not needed below ~2 km with float32 positions (0.12 mm resolution at 1 km). |
| block floating point (shared exponent per vector or island) | float-like | float-like | Deterministic only if the exponent choice is integer logic; costs a normalisation pass on every vector and the same 64-bit lane multiplies. No physics engine found uses it; DSP does. Not worth a prototype. |
| 128-bit fixed (fixed3d's `LUDICROUS_MODE` Q112.16 positions; ALICE-Physics, Rust) | ±2.6e33 | 1.5e-5 | fixed3d measured +1.6 % for 128-bit positions and broadphase only. A whole solver in 128-bit is 4 MULs per product: no. ALICE-Physics publishes no performance data (unverified). |

Where 128-bit intermediates are needed: every Q32.32 product (the raw product is 128 bits by definition; the
result is bits 32..95); in Q48.16 every product whose true value exceeds 2^31, which in our world includes any
inverse-scaled operand (inverse inertia × angular impulse), squared distances of far bodies, and the `K = m omega²`
style joint stiffnesses. Division: Q32.32 `(a << 32) / b` is a 128/64 division: x64 has `DIV r64` natively (it
traps on quotient overflow, so guard it); ARM64 has no 128/64 divide, so it is a Newton reciprocal or a bit-serial
loop; fixed3d's `fixDiv` takes a single hardware divide when the numerator fits in 47 bits and a 128-bit path
otherwise. Square root: integer sqrt of a 96–128-bit value; fixed3d seeds with the hardware double sqrt and repairs
to the exact floor (so a float sqrt plus a few integer ops), with a restoring shift-subtract fallback.

Where precision runs out first: in Q48.16, chip inertia and slow rotation; in Q32.32, nothing before the stress
kernel, which stays double anyway. Quaternion normalisation, restitution, TOI root finding in [0,1], and the soft
step coefficients are all comfortable in either 64-bit format (impulseScale 0.0577 is represented to 1.3e-4 relative
in Q48.16 and 4e-9 in Q32.32).

## 2. Cost against float

Hardware facts:

- x64 scalar: `MUL r64` (64x64->128) is 2 uops, latency 3–4, throughput 1 per cycle on Skylake through Alder Lake-P
  and Zen 2–4 (uops.info). `MULX` is the same unit. A float `mulss` issues 2 per cycle; `mulps` does 4 lanes 2 per
  cycle.
- x64 SSE2: `PMULUDQ` is 32x32->64 on 2 lanes, throughput 0.5 cycles on Skylake+ and Zen 3+ (uops.info). A 2-lane
  64x64->64 needs 3 PMULUDQ plus ~6 shifts/adds; a 2-lane 64x64->128 needs 4 PMULUDQ plus carry handling, ~14–18
  uops: 7–9 uops per lane-multiply against 0.125 cycles per lane for `mulps`. AVX-512 IFMA (`vpmadd52luq/huq`,
  8 lanes of 52x52->104) exists on Ice Lake, Tiger Lake, Rocket Lake, Sapphire Rapids and AMD Zen 4/5, and is not
  officially on Alder/Raptor Lake (Wikipedia AVX-512 support table); the toaster (Comet Lake i7-10870H) has no
  AVX-512 at all.
- ARM64 scalar: Apple M1 Firestorm does `MUL`, `SMULH` and `UMULH` at latency 3, 2 per cycle (dougallj tables,
  ocxtal measurements), so a 64x64->128 is two instructions at effectively 1 cycle: cheaper than x64. Cortex-A7x
  and Neoverse rates are unverified here (the Arm software optimisation guides would settle it; UMULH is usually
  lower throughput than MUL on those cores). NEON has `UMULL` (32x32->64, 2 lanes) and no 64-bit lane multiply:
  same emulation cost as SSE2. fixed3d's integer-NEON narrowphase gained 5–10 % over scalar fixed, which says all
  that needs saying about 64-bit integer SIMD on 128-bit vectors.
- NVIDIA: 32-bit `IMAD` is native; `IMAD.WIDE.U32` gives a 64-bit product. Integer lanes per SM are half the FP32
  lanes on Ampere GA10x (cc 8.6: 64 INT32 vs 128 FP32), Ada (8.9), Hopper (9.0) and datacenter Blackwell (10.0),
  and equal on consumer Blackwell (cc 12.0: 128 unified lanes) per Wikipedia's CUDA table; the CUDA C programming
  guide's "throughput of native arithmetic instructions" table is the primary source and lists 32-bit
  `mul.hi` and all 64-bit integer operations as "multiple instructions" (the table could not be fetched through
  this session's tooling; the 12.0 column is unverified). 64-bit integer add is two instructions (IADD3 with
  carry); a 64x64->128 is 4 IMAD.WIDE plus 4–6 carry adds (NVIDIA forum, njuffa; the cuda-fixnum library's
  schoolbook counts).
- AMD RDNA 3: 32-bit integer multiply runs at roughly a quarter of the FP32 rate with high latency and no VOPD
  dual issue for integer ops (Chips and Cheese microbenchmarks, 2023); `v_mad_u64_u32` is a 32x32+64->64 that is
  full rate on CDNA 3 but quarter rate on RDNA (unverified for RDNA 4).
- Intel Xe: Xe-LP's 8-wide ALU does 1 INT32 or FP32 op per lane per clock on paper (Intel Xe architecture
  guide), but Chips and Cheese measured Arc A770 integer multiply "much lower" than integer add and integer add
  at 75–80 % of FP32; the Gen 9.5 UHD on the toaster is the same lineage. Treat INT32 multiply as quarter rate or
  worse (unverified: the wave-2 D3D11 microbenchmark measures it).

Cost model for one Box3D contact iteration (45 mul, 40 add per point, no div, no sqrt):

| path | per multiply vs same-ISA float | per add | contact-iteration estimate | notes |
|---|---|---|---|---|
| x64 scalar Q32.32 vs scalar float | 2–3x (MUL 2 uops on one port + shift vs `mulss` 2/cycle) | 0.5–1x (64-bit add is 1 uop, 4/cycle) | **1.6–2.2x** | matches fixed3d's 1.7–3.2x spread |
| x64 scalar Q32.32 vs Box3D's 4-wide SSE2 solver | 8–12x | 2x | **5–8x on the solve stage** | the wide solver is what fixed point forfeits; Box3D's scalar fallback is 2–3x slower than its wide path on its own |
| SSE2 2-lane Q32.32 (PMULUDQ emulation) | 12–16x per lane | 1x | worse than scalar | do not build this; fixed3d's NEON path confirms |
| AVX-512 IFMA 8-lane 52-bit | ~2x per lane | 1x | ~2x vs wide float | not on the toaster; a Q26.26 in 52 bits fails chip inertia like Q48.16; would need the mixed format above |
| ARM64 scalar Q32.32 vs scalar float | 1.5–2x (MUL+UMULH 2/cycle) | 1x | **1.4–1.8x** | Apple silicon and Grace |
| NVIDIA Ampere/Ada/Hopper Q32.32 vs FP32 | 8–10 instr at half lane count: 16–20x | 2 instr at half: 4x | **10–13x issue cost** | plus 2x register pressure, lower occupancy; solvers are gather-bound, so realised 4–8x is plausible (unverified) |
| NVIDIA consumer Blackwell (12.0) | 8–10x | 2x | 5–7x | unified lanes halve the penalty |
| NVIDIA Q48.16 with 64-bit-only products (Photon rule) | 3 IMAD.WIDE + 2 adds at half rate: ~10x | 4x | ~7x | forbids inverse-scaled operands: inconsistent with our mass range |
| NVIDIA 32-bit fixed per quantity (IMAD.WIDE native 32x32->64) | 2x | 2x | ~2x | only if every quantity fits 32 bits with per-quantity scaling: velocities and local positions do, inverse inertia × impulse does not without a shared exponent; a research direction, not a plan |
| AMD RDNA 3 Q32.32 | 4 quarter-rate multiplies + adds: ~20x | 2x | **12–15x** | |
| Intel Xe (iGPU) Q32.32 | ~20–30x (unverified) | 2–3x | **15–25x** | the toaster's second GPU |

Transcendentals: Box3D's own `b3ComputeCosSin` is a Bhaskara rational with a normalising sqrt (`math_functions.c:216-262`:
4 multiplies, 2 divides, 1 sqrt) and `b3Atan2` a minimax polynomial (`math_functions.c:170-210`: 1 divide, ~8
multiplies), both used only in joint limits and user code, never in the contact loop; a fixed port is the same
polynomials in fixed arithmetic (0 A.D.'s `sincos_approx` reaches 5e-4 error, its `atan2_approx` 0.08 rad, which
was too rough for BEPU's Slerp: bepuphysics1int replaced FixedMath.Net's atan with an Euler series after visible
jerks). CORDIC (16–32 shift-add iterations) is the alternative when no multiplier is cheap; on x64 and ARM64 the
polynomial wins. Reciprocal and rsqrt: a 256-entry table seed plus 2–3 Newton steps of 2–3 multiplies each, ~8–10
multiplies, against a 4-cycle-throughput `divss`: 5–10x, but the solver does a few hundred of them per step, not
millions.

fixed3d's measurements (Apple M3 Ultra, arm64 clang, RelWithDebInfo, 4 workers; wall time over 199–999 steps,
minimum of 2 runs; 2026-07-11 run): convex_pile 1.54x, joint_grid 2.87x, large_pyramid 3.13x, large_world 4.46x,
many_pyramids 3.40x, rain 2.12x, trees50 1.70x, washer 2.06x, geometric mean 2.51x. The 2026-09-07 README figures
after further work: 2.24x scalar (1.71–3.21x), 2.13x with integer NEON in the narrowphase. Its profile: contact
solve ~24 %, anchor rotation ~15 %, prepare ~14 %, warm start ~11 %; "the remaining fixed-vs-float gap is the
64x64->128 lane multiplies vs float FMA". Body counts per scene are in the scene functions, not the README
(unverified; they are Box3D's standard benchmarks and run into thousands of bodies). BEPUphysics v1's fixed-point
fork reported "about 4 times slower" than float (sAm_vdP, Feb 2018), with a naive swap at 8–10x before tuning, on
the BEPU demos. Erin Catto's 2024 Box2D determinism post rejects fixed point as much slower, hard to code, world-size
limiting and overflow-prone, and reports no measurable cost for float determinism done with flags and custom trig.

Multiplier estimate for us: on the physics stage 2.2–2.5x at 4–8 workers (fixed3d, ARM64), likely 2.5–3x on x64
where the MUL unit is one per cycle (unverified until the x64 column exists); on the solve stage alone 5–8x against
the SSE2 wide path. Town: physics 3–9 ms becomes 7–22 ms at 8 workers; the 1-worker step (12.5 ms with physics as
the largest share) crosses 16.7 ms.

## 3. Existing engines and what they gave up

| engine | format and arithmetic | scale and numbers | what it gave up |
|---|---|---|---|
| Photon Quantum 2/3 (closed source) | FP = Q48.16 in 8 bytes; 5 decimal digits; manual says precision is decent within 0.01..1000; fast multiply keeps a 64-bit product, so operands must stay in Int16 range and overflow is not detected; all systems (physics 2D and 3D, navigation) in FP (Quantum manual, "Fixed Point"). Trig and sqrt via lookup tables per its API docs (unverified: the doc site blocks fetches). | Body counts unpublished; "World Size" is a map setting that bounds the broadphase; resting bodies are excluded from checks for performance. Shipped in large-player-count physics games (Stumble Guys is the usual citation; unverified from Photon). | Range (Int16 operands), resolution 1.5e-5, no rigid-body fidelity claims; 3D physics is a simpler engine than PhysX-class. |
| SG Physics 2D (Godot, MIT, 2021–) | Q48.16 in int64; range limited to ±30 000 units in practice. | Collision detection and kinematic bodies only: no rigid-body dynamics; scenes capped at 60 000 × 60 000 px; thousands of nodes hurt. | Dynamics entirely. |
| bepuphysics1int (C#, 2018) | FixedMath.Net Fix64, Q31.32, split-product multiply; determinant of inertia inversion overflowed 2^31 "rather easily", fixed by Gauss elimination; atan2 too rough for Slerp. | About 4x slower than float; world extent about ±1000 per axis; multithreading must be off for determinism. | Performance, world size, threading. |
| fixed3d (C, MIT, 2026) | Q48.16 int64 with a 128-bit multiply (native `__int128` or emulated; round-to-nearest; optional saturation), fast-path 47-bit division with `divq`, exact integer sqrt seeded by hardware double sqrt, a 40-bit inverse scale for inverse mass/inertia, optional 128-bit positions. All of Box3D: solver, GJK, trig, casts, mass, recording. | 2.24x float (M3 Ultra); all 24 test suites pass. Tracks Box3D commit 47d7f7c (ours is 5643cd81; distance unverified). | Box3D's SIMD wide solver and SAT; half the resolution budget of Q32.32 at the small end. |
| dbox2d (Go, 2025–) | Box2D v3.1 port with an optional Q32.32 build tag; checked against float traces "within measured budgets"; no fixed-vs-float timing published. | 2D. | Some float-oriented operations are forbidden in both modes. |
| box2d_fixed (91Act) | Box2D 2.x with the float type replaced by fixed. | 2D; unmaintained. | |
| libfixmath (C) | Q16.16 with int64 intermediates, optional overflow detection, lookup-table sin (2 % error) and 80 KB caches. | Embedded. | Range. |
| 0 A.D. (C++ RTS) | CFixed_15_16 with int64 products, `isqrt64`, polynomial trig (5e-4), debug-only overflow checks. | Thousands of units, no rigid bodies. | Range (±32 768 units). |
| TrueSync (Photon, Unity, discontinued) | FP type, 2D and 3D deterministic engines; the 3D one is reported to be a Jitter port (unverified). | Discontinued; Photon replaced it with Quantum. | |
| FixedPhysics.rs, ALICE-Physics (Rust) | `fixed` crate; ALICE claims 128-bit fixed point. | Fighting-game scale; no performance data (unverified). | |
| StarCraft II, RTS lockstep | Widely reported to use fixed-point unit simulation with synchronised RNG and rolling checksums (secondary sources only; no Blizzard primary found). | Thousands of units, no rigid bodies. | |
| Teardown multiplayer (2022–2026) | Only the destruction logic was rewritten in fixed-point integers, natural for voxel volumes; breakage (object hierarchies separating, joints reattaching) stayed float; transforms are state-synchronised with eventual consistency; Gustafsson later reconsidered dismissing full determinism (Voxagon blog, 2026-03-13). | Shipped. | Deterministic physics: it never had it. |
| Rapier (Rust) | Float with an `enhanced-determinism` feature (portable libm), not fixed point. | T3's territory. | |

Is there a 3D fixed-point rigid-body engine at 1500 awake bodies? fixed3d runs Box3D's pile, pyramid and rain
benchmarks (thousands of bodies, counts unverified) at 2.2–2.5x float; Quantum ships 3D physics without published
counts; nothing else found approaches it. So yes, one exists, and its price is the number above.

## 4. The narrower boundary: exact arithmetic for fracture geometry only

What the fracture does today (`src/poly.c:155-221`, `src/fracture.c:54-123`): float bisector planes from float
sites, float signed distances with a 2e-3 m tolerance, and a hack that pushes the plane outward in steps of
2 × tolerance up to 8 times until no vertex is within tolerance (up to 32 mm of geometry moved on a 7 cm cell);
neighbours ordered by the float bits of squared distance; site rejection by float squared distance; sliver
absorption, bond areas, keeper merges by float thresholds. Every one of these is a flip-prone decision (the brief's
list) and the plane shift is a robustness smell independent of determinism.

Techniques, and how each fits:

- **Integer sites and integer planes (plane-based geometry)**. Snap sites to a grid in the piece frame. The
  bisector of integer sites a, b is the exact integer plane `n = 2(b - a)`, `d = |b|² - |a|²`. Every vertex is the
  intersection of three planes, stored as three plane indices and evaluated as a 4D homogeneous integer point
  (3x3 determinants); classifying a vertex against a fourth plane is a 4D integer dot product with a sign fix
  (Nehring-Wirxel, Trettner, Kobbelt 2021: "Fast exact booleans for iterated CSG using octree-embedded BSPs", CAD
  journal, arXiv 2103.02486). Their bound: with b bits of intermediate, `n⁺⁴ v⁺ ≤ 2^(b-1)/48`; with 128-bit
  arithmetic and computed normals they allow vertex coordinates to 2^14. Our budget: piece frames within ±4 m at
  2^-14 m (61 µm) put sites in 17 bits, bisector normals in 18, offsets in 36; parent face planes snapped to
  24-bit normals fit the same 128-bit configuration. Their speed: 2.5 M convex mesh-plane cuts per second on one
  core with 128–256-bit fixed precision, one to two orders faster than GMP (their Table 1: 11–402 cycles per core
  operation against 1360–9100). On x64 this is `MUL`/`_umul128` and `adc`/`_addcarry_u64` chains; MSVC has no
  `__int128`, so a 200-line two-limb signed helper is needed (GCC/Clang get `__int128`).
- **Shewchuk adaptive predicates** (1997; public-domain C, ~4k lines; `georust/robust` is a port): exact signs of
  orient3d on float inputs with a fast filter. They would make the clip sign exact relative to a float plane, but
  our planes are themselves rounded constructions, so the guarantee is only "consistent with the rounded plane".
  Unnecessary once inputs are integers, and heavier code than the integer path.
- **Indirect predicates** (Attene 2020; Cherchi et al. 2022 "Interactive and Robust Mesh Booleans"): implicit LPI
  and TPI points evaluated by filtered predicates on float hardware, exact fallback with expansions. Header-only
  C++ under LGPL-3, requires `/fp:strict` or `-frounding-math` and its own SIMD flags: strong for general mesh
  booleans, wrong shape and licence for a C17 engine that only cuts convex cells with known planes.
- **Snap rounding** vertices to a grid after each clip: simple, deterministic if inputs are integers, but a snapped
  vertex can sit outside a plane by half a grid step, so later clips need a tolerance again: the current design with
  smaller epsilons. Teardown's fixed-point cuts are this at voxel resolution.
- **Rational or homogeneous coordinates**: what plane-based geometry is, with vertices as (x, y, z, w) integers;
  storing plane triples instead of the 4-vectors keeps the numbers small.

What integer plane-based clipping fixes: clip signs become exact (no vertex is "within tolerance", so no plane
shift; a vertex on a plane is classified exactly and the cap is a proper loop); neighbour ordering and site
rejection become integer comparisons; the number of random draws per site no longer depends on a float compare;
sliver absorption and bond areas can be evaluated from exact vertices in a fixed order of double operations
(deterministic, if not exact; exactness is not required for determinism, and comparing exact rational volumes needs
256-bit cross-multiplication that is not worth it); keeper merges likewise. It does not touch stress break
thresholds (double CG, already order-fixed; `cbrtf` is the hazard there, T3) or link thresholds (Box3D floats).

Cost: a clip is one 4D integer dot per vertex (~20–40 cycles) plus bookkeeping; a 30-vertex cell clip is 1–2 µs. A
100-piece blast with ~100 cells × ~25 candidate planes each is ~250k clips, 100–250 ms on one core against today's
20–90 ms for the whole spike on the job pool; parallel over pieces as now, 15–30 ms at 8 workers. So it is
comparable, not free; the wave-2 experiment measures it on recorded fracture inputs.

Feeding a float physics engine: each cell's vertices are rational (plane-triple) and round to float once at output;
the float hull may be an ulp off convex, and Box3D's quickhull (`b3ComputeHull`, float) re-hulls it with its own
merge tolerance, deterministically under H1. Better: build `b3HullData` from the exact face-vertex topology directly
(float vertices, Newell planes recomputed in float) so quickhull's tolerance decisions never enter; that needs a
small Box3D entry point (the brief lists quickhull at 3.2k lines; whether a from-topology constructor exists is
unverified: grep `hull.c` for a non-quickhull builder). Mass properties (volume, centroid, inertia) computed from the
exact geometry in a fixed op order and rounded to float are then deterministic pieces regardless of the physics
engine's own float. This is also the natural place to compute bond areas.

Worth it regardless of networking: yes for the clip and neighbour ordering (robustness, the plane-shift hack, sliver
consistency, and a fracture that is bit-identical across compilers and ISAs by construction); no for exact
volumes and areas (deterministic double from exact vertices is enough).

## 5. Is "quantise to a grid every tick" a mirage?

Yes, quantitatively. Let two machines start a tick from identical (quantised) state and produce values that differ
by δ because of a libm, contraction or ordering difference. Rounding to a grid of spacing g removes the difference
only when both values fall in the same cell; with a uniformly distributed phase the per-value flip probability is
|δ|/g per tick, and a flip is a divergence of size g, not δ, which then grows chaotically. With N_v tracked values
the expected time to the first flip is g/(N_v δ) ticks.

Numbers for us: 1500 awake bodies × 13 values (position, quaternion, two velocities) = 2e4, ignoring Box3D's warm-
start impulses which are hidden state and would need quantising too. δ = 1e-6 m is one ulp of a float position 10 m
from the origin, a lower bound: the solver's branches (`max(0, impulse)`, the speculative `s > 0` test, friction
cone clamps, sleep thresholds) turn ulp differences into O(1) differences within the tick.

| grid g | flips per tick | mean time to first flip |
|---|---|---|
| 1/4096 m = 2.4e-4 m (Fiedler's stability floor) | 81 | 0.012 ticks |
| 1 cm | 2 | 0.5 ticks |
| 1 m | 0.02 | 51 ticks (0.85 s) |
| needed for one hour without a flip | | g ≥ 4.3 km |

Fiedler's own finding is the other jaw of the vice: quantising below 4096 positions per metre and 15 bits per
quaternion component "forces physics objects into penetration with each other" and pops; a grid coarse enough to
absorb float noise destroys stacking. Shipped use: Fiedler's "quantize on both sides" (State Synchronization,
gafferongames.com) quantises position, velocities and orientation before each step on sender and receiver so that
both extrapolate from the same quantised state between updates. It is explicitly an approximate and lossy strategy
for a state-sync model with periodic overwrites; it does not claim, and does not produce, cross-machine determinism.
Teardown's hybrid (deterministic destruction commands plus eventually consistent transform sync) is the honest form
of "converging": the host's state overwrites divergence; nothing converges on its own. That is H5's primitive
(serialise, hash, resync), not a numerical trick.

## 6. Effort and agent-friendliness

Sizing the Box3D subset we use (line counts from `extern/box3d/src`, float-bearing lines by grep): contact solver
2395 lines (461 float lines, half of the file is the `b3FloatW` wide path), convex manifold and SIMD SAT 2840 (480),
GJK/shape cast/TOI 1924 (234), quickhull 3158 (256), joints ~4100 (~480), solver 2249 (76), body 2663 (90), shape
2427 (135), math 616 + a 1206-line header, snapshot/recording 7.2k with layouts to change. Roughly 3–4k lines of
arithmetic to convert, 1.5k lines of SIMD to delete or replace, and every tolerance constant to re-derive. fixed3d
did exactly this for all of Box3D in about three months with one engineer and an AI collaborator (benchmark dates
2026-07-11 to 2026-09-07, 303 commits).

| option | what | effort | gain | risk |
|---|---|---|---|---|
| A: none | Float everywhere; T3's flags and tests; H5's serialise/hash/resync. | 0 for this track | Keeps Box3D's SIMD and cadence and the 2.2–2.5x. | Cross-ISA and emulator bit-exactness rests on H1/H6 evidence and on our own `cbrtf`, contraction and libm hygiene. |
| B: fracture geometry only | Integer sites, integer plane-based convex clipping with 128-bit predicates; exact vertices rounded once to float; mass and bond areas from exact geometry in fixed op order. | 2–4 weeks: ~150 lines of two-limb 128-bit helpers (MSVC), ~400 lines replacing `lpPoly_Clip` and the Voronoi loop, tests against the current fracture on recorded inputs. | Removes every fracture flip and the plane-shift hack; fracture is bit-identical everywhere by construction; slivers decided consistently; quickhull tolerance out of the loop if hulls are built from topology. | Cost 1–3x today's fracture spike (measure); a from-topology hull builder may need a Box3D patch; 128-bit helpers are a new thing for agents to get wrong (silent wrap): ship them with a saturating debug mode. |
| C: our whole core | B plus fixed point in debris ghosts, stress loads, links, wheels, rigs, gait. | Months; the stress solver's 12-decade stiffness range does not fit any 64-bit fixed format and needs scaled blocks or stays double. | Nothing beyond B unless the physics is fixed too, because every core decision reads Box3D floats (hits, impulses, casts, move events). | Wasted unless D happens; the stress CG in fixed is the hardest numerical job in the repo. |
| D: physics too | Vendor fixed3d (Q48.16 + 40-bit inverses) and rebase our patches, or port ourselves to Q32.32, or write a fresh Q32.32 engine (hulls, 4 joints, sleep, CCD vs static, casts; ~20–25k lines by Box2D v3's size). | Vendor: 1–2 months to rebase patches, re-tune tolerances, re-baseline every bench hash and pass our 9 suites; port: 3 months (fixed3d's precedent); fresh: 6–12 months and a solver-quality risk against Box3D's soft step, colouring and relax passes. | Bit-exact across x64, ARM64, every emulator and every compiler with no flags, no libm audit, no SIMD-vs-scalar twin; integer SIMD and GPU paths agree with scalar for free; snapshots are plain integers. | 2.2–2.5x on the physics stage (fixed3d) breaks the 1-worker toaster budget; Q48.16 loses chip inertia unless inertia is computed wide and only inverses stored (verify in fixed3d's mass path); we inherit a three-month-old fork of a different Box3D commit; the wide solver is gone for good on SSE2/NEON. |

Agent-friendliness: fixed point is plain C with no compiler flags to police and integer tests that either match or do
not, which agents like; against that, every formula carries a range contract the compiler cannot check, overflow
wraps silently unless a saturating or asserting mode is built in (fixed3d's `FIX_SATURATE`, 0 A.D.'s debug
overflow macros), and the vendored surface grows by fixed3d's `extern/fixed` library plus its plumbing. Option B
adds ~600 lines and removes a hack; option D adds a fork to track.

## 7. Verdict on H3

"Fixed point everywhere costs 2–4x": **confirmed for the physics stage** at 2.2–2.5x measured (fixed3d, ARM64,
4 workers; x64 unverified and likely worse because MUL r64 is one per cycle), 5–8x on the contact solve stage
against Box3D's SSE2 wide path, and 10–20x per multiply on GPUs (Section 2). "Forfeits Box3D": **partly
falsified**; fixed3d is Box3D in fixed point under MIT, so what is forfeited is its SIMD, its commit cadence and our
patches, not the engine. "Solves a problem flags and tests solve": **open** until T3 reports on H1 and H6; what
fixed point uniquely buys is bit-exactness across ISAs, emulators, compilers and SIMD-vs-scalar paths without any
evidence-gathering, which is worth 2.2–2.5x only if that evidence fails. "Exact predicates for fracture geometry may
be worth it for robustness anyway": **confirmed**; option B is cheap, removes real hacks, and stands on its own.

Recommendation for the orchestrator: A now, B as a planned milestone independent of the networking decision, D only
if T3 shows float cannot be made bit-exact where cross-play needs it, in which case vendor fixed3d rather than port,
and measure it on the toaster first.

## 8. Micro-benchmark spec for wave 2

All kernels seeded (PCG32), single-threaded unless stated, MSVC and clang-cl on the i7-10870H, clang on the DGX Spark
(Grace) and the MacBook; report ns per element, median of 5 runs of 1e6 elements, and the hash of the outputs (the
integer variants must hash identically across machines; the float ones show whether they do).

1. **Contact iteration kernel.** Extract `b3PushContacts_Mesh`'s per-point body (`contact_solver.c:449-500`) into a
   standalone loop over 1e6 synthetic contact points with random anchors, normals, inverse masses spanning 3e-4 to
   20 and inverse inertias spanning 5 to 1.6e5. Variants: float scalar; float SSE2 4-wide as Box3D's wide path;
   Q32.32 scalar with `_mul128`/`__int128`; Q48.16 scalar with 128-bit products (fixed3d's `fixMul`); Q48.16 with
   64-bit products and no inverse scale (Photon's rule; count overflows). Expected: scalar fixed 1.6–2.2x scalar
   float, 5–8x wide float; the 64-bit-product variant overflows on the chip rows.
2. **GJK distance.** `b3ShapeDistance` on pairs of 32-vertex hulls at 1 cm to 1 m feature sizes, float vs Q32.32 vs
   Q48.16 (division-heavy). Expected 2–4x; watch the Q48.16 simplex decisions on 1 cm features.
3. **fixed3d itself.** Build fixed3d and float Box3D at its pinned commit on our machine; run its benchmark suite at
   1 and 8 workers to produce the missing x64 column; then a convex pile of 1500 hulls with our size and mass mix
   (2.8 cm chips beside 3 t boxes) for 500 steps; record ms per step and whether chips tumble (the inertia question)
   and whether the pile settles (sleep).
4. **GPU multiply cost.** A D3D11 compute shader (the sandbox already has the toolchain) over 4M lanes: chains of
   FP32 FMA; chains of 32x32->64 (`umul` hi/lo); Q32.32 multiply via 4 partial products with carries; measure GOPS on
   the RTX 3060 and the Intel UHD. Expected: NVIDIA Ampere 8–10 instructions per Q32.32 multiply at half lanes;
   Intel 15–30x.
5. **Exact clipping.** Record the fracture inputs of the town and keep blasts (`lpFractureInput`), then run the
   float `lpPoly_Clip` path and an integer plane-based prototype over the same inputs: cells per second, the
   count of `failureCount` numerical failures and plane shifts in the float path, and hash agreement of the integer
   path across MSVC and clang. Expected: integer within 1–3x of float; zero failures.

## Sources

Repo (file:line): `src/poly.c:155-221` (clip, tolerance, plane shift); `src/fracture.c:54-123` (Voronoi
neighbours by float bits, site loop); `src/world.c:26-70` (materials, joints), `:557-566` (contact tolerance and area
threshold); `src/world.h:747-759` (inertia padding); `src/solve.c` (double accumulators); `src/stress.c:487-491`
(stiffnesses); `extern/box3d/src/solver.h:265-307` (soft step); `contact_solver.c:218,399-515,636,677`;
`solver.c:86-102,219,724`; `math_internal.h:205-217`; `math_functions.c:170-262`; `body.h:163-227`; `simd.h:18-36`;
`include/box3d/math_functions.h:512-523`.

- fixed3d (Glenn Fiedler, MIT): https://github.com/mas-bandwidth/fixed3d ; README
  https://raw.githubusercontent.com/mas-bandwidth/fixed3d/main/README.md (2.24x, 2026-09-07);
  https://raw.githubusercontent.com/mas-bandwidth/fixed3d/main/docs/large_worlds.md ;
  https://raw.githubusercontent.com/mas-bandwidth/fixed3d/main/extern/fixed/include/fixed/fixed.h (fixMul, fixDiv,
  fixSqrt); https://raw.githubusercontent.com/mas-bandwidth/fixed3d/main/benchmark/apple_m3_ultra_fixed/README.md
  (2.51x table, 2026-07-11, profile); https://raw.githubusercontent.com/mas-bandwidth/fixed3d/main/benchmark/main.c.
- Photon Quantum fixed point: https://doc.photonengine.com/quantum/current/manual/quantum-ecs/fixed-point (Q48.16,
  5 digits, 0.01..1000, Int16 rule; read through search excerpts, the site blocks fetches);
  https://doc.photonengine.com/quantum/current/manual/physics/physics-performance ;
  https://doc-api.photonengine.com/en/quantum/current/struct_quantum_1_1_physics_body3_d.html ; TrueSync:
  https://doc.photonengine.com/truesync/current/getting-started/feature-overview.
- SG Physics 2D: https://gitlab.com/snopek-games/sg-physics-2d (README, 2021–2023).
- bepuphysics1int: https://github.com/sam-vdp/bepuphysics1int ; forum thread (Feb 2018)
  https://forum.bepuentertainment.com/viewtopic.php?t=2507.
- FixedMath.Net (Q31.32, archived 2021): https://github.com/asik/FixedMath.Net ; libfixmath:
  https://github.com/PetteriAimonen/libfixmath ; 0 A.D. `Fixed.h`:
  https://github.com/0ad/0ad/blob/master/source/maths/Fixed.h.
- dbox2d (Go, Q32.32 option): https://github.com/dhannyell/dbox2d ; box2d_fixed: https://github.com/91Act/box2d_fixed ;
  FixedPhysics.rs: https://github.com/ValorZard/FixedPhysics.rs ; ALICE-Physics:
  https://github.com/ext-sakamoro/ALICE-Physics ; Rapier determinism: https://godot.rapier.rs/docs/documentation/determinism/.
- Erin Catto, "Determinism" (Aug 2024): https://box2d.org/posts/2024/08/determinism/.
- Teardown: Voxagon blog, "The unlikely story of Teardown Multiplayer" (2026-03-13)
  https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html ; 2022 tweet
  https://twitter.com/voxagonlabs/status/1565672132213882881 ; 80.lv summary
  https://80.lv/articles/teardown-developer-breaks-down-multiplayer-and-voxel-destruction-tech.
- StarCraft II fixed point (secondary, unverified): https://www.socratopia.app/library/math-for-game-devs-en/chapter-30 ;
  https://medium.com/adequatesource/real-time-strategy-games-what-makes-them-tick-4ac36bd3de95.
- Glenn Fiedler, "State Synchronization": https://gafferongames.com/post/state_synchronization/.
- Nehring-Wirxel, Trettner, Kobbelt, "Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs", CAD 2021:
  https://arxiv.org/abs/2103.02486 (bit bounds, Table 1, 2.5 M cuts/s); EMBER (Trettner et al. 2022):
  https://www.graphics.rwth-aachen.de/media/papers/339/ember_exact_mesh_booleans_via_efficient_and_robust_local_arrangements.pdf.
- Attene, Indirect Predicates (LGPL-3): https://github.com/MarcoAttene/Indirect_Predicates ; Cherchi et al.,
  "Interactive and Robust Mesh Booleans" (2022): https://arxiv.org/abs/2205.14151.
- Shewchuk, "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates" (1997):
  https://people.eecs.berkeley.edu/~jrs/papers/robust-predicates.pdf ; port: https://github.com/georust/robust.
- uops.info: https://www.uops.info/html-instr/MUL_R64.html ; https://www.uops.info/html-instr/PMULUDQ_XMM_XMM.html.
- Apple M1 instruction tables: https://dougallj.github.io/applecpu/firestorm-int.html ;
  https://github.com/ocxtal/insn_bench_aarch64/blob/master/results/apple_m1_firestorm.md.
- AVX-512 IFMA support table: https://en.wikipedia.org/wiki/AVX-512.
- NVIDIA: CUDA C Programming Guide, arithmetic instruction throughput table
  https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#arithmetic-instructions (not fetchable here;
  columns unverified); lane counts per compute capability https://en.wikipedia.org/wiki/CUDA ; 64-bit emulation:
  https://forums.developer.nvidia.com/t/estimate-64bit-integer-instruction-throughput/65643 ;
  https://forums.developer.nvidia.com/t/long-integer-multiplication-mul-wide-u64-and-mul-wide-u128/51520 ;
  https://github.com/data61/cuda-fixnum.
- AMD RDNA 3 integer multiply (Chips and Cheese, 2023):
  https://chipsandcheese.com/p/microbenchmarking-amds-rdna-3-graphics-architecture.
- Intel: Arc A770 microbenchmarks https://chipsandcheese.com/p/microbenchmarking-intels-arc-a770 ; Xe architecture
  https://www.intel.com/content/www/us/en/docs/oneapi/optimization-guide-gpu/2023-2/intel-xe-gpu-architecture.html.
