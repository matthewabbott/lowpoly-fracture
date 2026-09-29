# Roadmap

Principle for every milestone: **buy performance headroom first, then spend it.** A full game means a large map,
several destructible cars, and containers of sloshing volatile reagents, all at once. Every milestone logs its
before/after numbers in [perf-log.md](perf-log.md).

Order (one at a time):
1. Chunky fracture + debris tiers (done)
2. Toppling and stress points (done)
3. Breakable links and assemblies (done)
4. Stress at scale (in progress)
5. Destructible vehicles
6. Profiling and hot paths
7. Large-map physics zones
8. Networking
9. Art polish

## 1. Chunky fracture + debris tiers

"Cooking the books": the fracture detail is real, but most fragments become cheap objects. A log shot through leaves
cosmetic splinters and two rough ends of a handful of triangles each.

Done: the tiers (puff, ghost, light, full), rest states, shoving, the budget ladder and the blower are described
in [architecture.md](architecture.md) ("Debris tiers"); numbers are in [perf-log.md](perf-log.md).

Grabbing or launching a piece promotes it to full. Over budget, debris is demoted down the ladder instead of
popping.

## 2. Toppling and stress points

Done: a quasi-static stress solve on each structure's bond graph (`stress.c`, see architecture.md), joints (mortar,
dry, nails, solid) with their own strengths, strain so joints creak before they give, and toppling that emerges from
no-tension joints (the tower felling test). Also done: slender pieces snapping mid-span, resting loads, and the
masonry pattern (brick walls are one solid piece until hit, then break along their mortar). The ruins scene shows
structures failing where they are weak: a dry arch falls without its keystone, a colonnade drops the two lintels on a
lost column, and a balcony near its limit breaks off after one round at its root (`scripts/ruins_demo.txt`).

Structures solve in parallel, each within its own share of the step's budget (perf-log, "parallel stress solves"),
so big events (buildings falling onto buildings, barrages) keep collapses prompt without stretching the step.

Open, taken up when measurements call for them:
- one very large structure (1000+ pieces) still solves on one thread within its share: the k-hop patch (re-solve only
  near the damage) or a parallel K·x with fixed partitions;
- ground joints, so whole walls can overturn off their foundations;
- slump (a structure that sags into a new rest pose instead of cracking), mortar relief, creak sounds.

The earlier notes below shaped it.

The idea is snapshotted stress that is checked only where things break.

- **Local re-check.** After a break, re-check a neighbourhood only (k bond hops or radius R around broken bonds).
  Skip sleeping and frozen pieces.
- **Moment check at a cut.** For the part above a damaged layer, compare the weight N and the offset e of its
  centre of mass with the support polygon of the bonds crossing the cut. Masonry rule: once e leaves the polygon,
  break the tension-side bonds first. The upper block becomes a dynamic body resting on a hinge edge, and Box3D does
  the toppling.
- **Snapshots, recomputed lazily when topology changes near damage:**
  - a bridge / articulation-point index (Tarjan, O(V+E)); breaking a bond that is not a bridge can never split a
    body, so the flood fill is skipped
  - per-bond supported mass (dominators rooted at the ground), for O(1) load checks
- **Persistence.** A bond must stay overloaded for a few steps before it breaks. This reads as creaking and spreads
  the cost, as in Red Faction: Guerrilla (a layer fails when its strength is less than the weight above it), NVIDIA
  Blast (iterative solver carried across frames) and vox3D (strided analysis).
- **Sudden loads** (landing debris, a truck on a bridge) come from hit-event impulses.
- **Upgrade path:** a Blast-style iterative solver (compression, tension and shear per bond) on a reduced support
  graph.
- **Context:** Dennis Gustafsson has been prototyping "stress analysis" for Tuxedo Labs' new engine (Aug–Sep 2026,
  "one building at 60 fps", algorithm undisclosed; the solver is "inspired by Erin's work").
  https://x.com/voxagonlabs/status/2092992959507570923

## 3. Breakable links and assemblies

Done: links (`link.c`, see architecture.md; "links" because `lpJoint*` already names bond joints) are Box3D welds,
hinges, ball joints and ropes between objects.
- They tear under load, creaking first. Loads are polled after each step, not taken from Box3D's joint events, which
  miss overflow-coloured and sleeping joints.
- Blasts damage them; the rifle can cut a rope.
- They outlive their pieces: through a split, the joint is rebuilt on the new body; through a fracture, the end
  moves to the cell holding its anchor; if that cell is blown out, the link breaks.
- Structures carry what hangs on them in their stress solve.
- "Fairy dust" (`gravityScale`) survives breaking.
- Linked bodies never freeze or count against the debris budgets, and neither does what rests on them.
- The yard scene (`scripts/yard_demo.txt`): a cart of volatile crates rolls into a wall when its rope is cut; a sign
  swings on its last rope; a drawbridge falls open; a door is blown off its hinge; a porter's rack of flasks is there
  to carry.

Open:
- wheel and slider links, and driving (vehicles milestone; Box3D's wheel reaction force is marked "probably wrong");
- segmented ropes and chains (one distance constraint per rope for now), buoyancy, link sounds;
- a grab that follows its piece through a fracture (the sandbox's grab drops then).

## 4. Stress at scale

Why: the per-structure stress budget (10k bond-iterations per step) caps practical building size. The keep (1973
pieces, 6296 bonds) gets one iteration per step: a hole in its wall takes about 1600 steps to settle, and every
solving step rebuilds the whole system. The goal is that a big building decides a local hit in about 8 to 15 steps
and degrades gracefully under a barrage, deterministically.

The idea (reviewed by GPT-6-Astra and Fable): reduce a structure to its load-bearing skeleton. Plausibility beats
causal consistency: fidelity may vary with load, as long as the engine stays deterministic.
- **Rigid clusters as super-nodes:** groups of lightly loaded pieces move as one; assembling their bonds gives
  exactly `PᵀKP`, so the existing kernels, preconditioner and conjugate gradient run unchanged.
- **Solve for the change, not the state (the Δ-form):** keep the last exact solution and solve only for the
  correction, so rigid clusters bias the redistribution, never the far field.
- **One residual pass as the meter and trigger:** after a reduced solve, one pass over all bonds gives each
  cluster's leak; a cluster that leaks too much dissolves and the solve reruns.
- **Who stays fine:** changed pieces and their neighbours, anchored and slender pieces, and both ends of any bond
  near its limit (so a lintel is never swallowed).
- **A fidelity ladder** chosen per structure from counts (L0 graph only, L1 local patch, L2 patch plus clusters,
  L3 exact), with an **audit queue** that re-solves provisional structures exactly once things calm down, and an
  age cap so chaos cannot starve it.

Steps (each measured in [perf-log.md](perf-log.md)):
0. The keep scene, settling at load, sweep bonding, the solver hash (done).
1. Split the solver out of `stress.c`, cache each body's system while it solves, the slender check over incident
   edges (behaviour-preserving: hashes identical; done).
2. Change tracking per piece, warm starts for fracture children, liveness (a request never drops a solve in progress,
   patience counts across restarts), a per-node convergence test, weakened joints judged from stored forces (done).
3. The reduced assembly and the Δ-form (done: behind a partition; nothing clusters yet).
4. Large structures take L2; clustering from exact results; drift tests against the fine solve (done: a local hit on
   the keep is decided in 12 steps instead of 41; a breach or cannon hole redistributes widely, dissolves most
   clusters and falls back to a fine solve, which the budget still starves).
5. The ladder and audits under pressure.
6. If settling or audits dominate: a nested-iteration settle and a two-level preconditioner.

## 5. Destructible vehicles

- **Driving:** raycast suspension and our own tyre friction; stable at racing speed and deterministic. When a wheel
  comes off, a physical wheel body spawns. Box3D's wheel joint (steering servo, spin motor, friction-only grip) is
  the fallback.
- **Construction:** the chassis is convex parts (frame, floor, cabin panels, hood, doors, bumpers) plus hidden
  interior "vital" parts (engine block, fuel tank, battery, wheel mounts). This works like Teardown's vital tags,
  where destroying the engine area stops the car. Driving ability comes from which vital parts are still attached.
- **Crumpling:**
  - a cage lattice in car space (about 4×3×6 nodes) is dented by impacts, capped per node
  - render vertices interpolate from the lattice (per-vertex crush weight, like GTA's vertex-colour deformation)
  - the chassis hull is rebuilt from the lattice only occasionally (`b3Shape_SetHull` + `ApplyMassFromShapes`)
  - panels sit on breakable joints; parts fracture only past a threshold
- **Fuel tank:** a detonator part. Leaking comes later with fluids.
- **Materials:** sheet metal (dents), glass (shatters), rubber.

## 6. Profiling and hot paths

Performance-maxxing with instruments instead of guesses: a per-phase profiler (fracture, split, stress build and
solve, physics, links, debris), a heavy-load bench rung per game (a race with rigging, a courier run), then the
hottest paths first. Candidates logged so far:
- the stress system: precomputed bond blocks instead of `lpAddBlock` per build, incremental updates, a structure of
  arrays for the 6-vectors, a parallel K·x for the biggest structures;
- the creak loop over stored overloads only; budget units calibrated to time;
- a spinning barrier in the task pool.

## 7. Large-map physics zones

- **Active cells** along the track, around "observers": players, AI cars, and flagged "chunk loaders" (armed
  contraptions, timers, detonators; anything the player sets up deliberately), like Minecraft's simulation distance.
- **Outside the active cells:**
  - resting islands sleep
  - loose debris is resolved by tier: snapped to the ground and frozen, or removed
  - unstable structures are settled with the stress check
  - flagged Rube Goldberg setups keep simulating
- **Determinism:** activity is derived from shared simulation state (player and car positions), never the camera.
- **Box3D:** prefer sleep over `b3Body_Disable` (re-enabling costs a spike). The static tree scales well; use
  `invokeContactCreation = false` for bulk static creation. Double precision is available beyond about 16 km.
- **Streaming:** add bodies in batches. Rendering LOD is a separate track.

## 8. Networking

Options:
- Teardown's approach: destruction is deterministic and sent as commands; bodies are server-synced with a
  priority queue.
  https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html
- Full lockstep on Box3D's determinism, if the toolchain is pinned.

A two-process lockstep experiment comes first.

## 9. Art polish

Shaders, palette, sky, ambient occlusion, character pipeline (RetroDiffusion low-poly GLB with auto-rigging, Meshy
with target poly count, Blender cleanup via its MCP), point-filtered character textures. Essential for any public
demo; deliberately last.

Effects and mess, noted while tuning debris:

- **Dirt where debris lands:** decals or vertex-colour darkening on the ground and on walls around scrap and rubble,
  so a fight leaves the place dirty even after the pieces are cleared or sunk.
- **Smoke and fire:** particle smoke and volumetric fire (Teardown-style). Fire ties into material flammability
  ([materials.md](materials.md)).
- **Dust:** soft, fading dust clouds instead of solid motes once a transparent particle pass exists.
- **Toaster profile:** one switch that lowers caps (debris, ghosts, scrap), raises the fragment scale and render
  scale, and turns off shadows, so low-end machines still get the destruction.
