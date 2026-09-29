# Roadmap

Principle for every milestone: **buy performance headroom first, then spend it.** A full game means a large map,
several destructible cars, and containers of sloshing volatile reagents, all at once. Every milestone logs its
before/after numbers in [perf-log.md](perf-log.md).

Order (one at a time):
1. Chunky fracture + debris tiers (done)
2. Toppling and stress points (done)
3. Breakable links and assemblies (done)
4. Stress at scale (done, but for step 6 if measurements ask for it)
5. Articulated objects and systems (vehicles first)
6. Creatures and mechs
7. Dents (cars and armor)
8. Profiling and hot paths
9. Large-map physics zones
10. Networking
11. Art polish

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
5. The ladder and audits under pressure (done: the budget shared by the structures that want it, provisional judgements
   audited exactly when calm; a patch level with the far field pinned was tried and dropped, the oracle showed it
   breaking joints it should not).
6. If settling or audits dominate: a nested-iteration settle and a two-level preconditioner (not needed yet: settling
   the keep takes 65 to 75 ms; moved to the profiling milestone with the cost of an iteration itself).

## 5. Articulated objects and systems (vehicles first)

Why: vehicles, cranes, mechs and monsters are one problem. Each is pieces and links whose function depends on which
parts are still attached and still fed: a car stops when its engine or fuel line goes, a mech limps when a knee
loses power, a troll hobbles on a stump. This milestone builds the shared foundation and proves it on cars; the next
one puts creatures on it.

Three layers:
- **The body:** pieces and links, as today. Muscles are capped motorised joints (a velocity servo on Box3D's hinge
  and ball motors: it holds up to its cap and sags past it, which is the limp). Stress on moving bodies ("inertia
  relief": loads balanced against the body's own acceleration) so a crash tears an engine off its mounts.
- **Systems:** parts carry up to 8 channels (fuel, power, steering; later blood, nerve, hydraulics), some parts are
  their sources. Supply is reachability from the sources over intact carrier bonds and links: a flood fill, run only
  when a carrier's topology changes, integer and deterministic. A source is live only if what it needs is supplied
  (fuel feeds the engine, the engine powers the steering). Vitals fall out of it: a vital part is a source (engine,
  heart, power core) or a carrier on every path (a spine). Detonators belong to parts, so a fuel tank blows alone.
- **Capability:** a summary each client reads (for a car: which wheels are attached, grounded, driven, steerable, and
  the engine's power), recomputed on change. Physics does the rest.

Decisions:
- **A wheel is a link** with one end on its mount piece and no Box3D joint, so it follows fractures and splits, takes
  blast damage, strains and tears off like any link. Suspension is a shape cast (a rounded disc), the tyres a small
  impulse solve per body (friction circle, fixed iteration count, link order). No wheel bodies, so no mass ratios
  across joints. A wheel that comes off spawns a physical wheel. Box3D's wheel joint is the fallback.
- **Controls are persistent, hashed state** (`lpWorld_SetVehicleControl`, `lpWorld_SetLinkTarget`), recorded only when
  they change.
- **Identity lives on each piece:** the object's user id, its part index and tag, its channel masks; fracture
  children inherit them. The core never interprets tags.

Steps (each measured in [perf-log.md](perf-log.md)):
0. This reshape.
1. Wheels and vehicles in the core: the wheel link, the cast, the tyre solve, wheels spawned when lost; stability
   tests at 40 m/s, full lock, kerbs, slopes and hard landings.
2. A track scene, a drive mode in the sandbox (chase camera, recorded controls), scripted drivers for the bench.
3. Part identity and part detonators.
4. Channels and supply.
5. A car kit (frame, sheet-metal panels on bolts, glass, engine, fuel tank, steering box) and crash calibration.
6. Muscles (motorised links, with a patch for Box3D's revolute torque getter) and a crane on the structure stress path.
7. Inertia relief: stress on moving bodies, triggered by load spikes (the cut line: it may move to milestone 6).

Deferred: pools (blood, fuel, hydraulic fluid) to milestone 6; crumpling to milestone 7.

## 6. Creatures and mechs

The hexapod mech first: statically stable, so losing a leg means planning a new gait, and a procedural gait looks
right on a machine.
- **Rigs:** a torso plus limbs, each a chain of muscle links. Each limb's capability is recomputed when its supply or
  links change: attached, its reach, its strength (the weakest link: cap × health × supply), and its foot (the
  lowest support point of whatever remains, so a stump walks as a peg).
- **Gait:** tripod with six able legs, ripple or wave with fewer; a leg lifts only if the centre of mass stays over
  the planted feet with a margin; the body lowers as strength drops. Footholds from the wheel cast, analytic leg IK.
- **Impairment:** pools (blood, hydraulic fluid, fuel) drain through severed carriers and scale strength; nerve
  channels paralyse what lies beyond a cut; damaged armor near a joint jams it (holding torque, lower speed).
- **Maimed attacks:** IK targets within the remaining chain and its caps; a weak swing hits softer on its own, since
  impacts take their damage from impulse. A grip is a link: lose the hand and the weapon drops.
- **Bones:** inertia relief snaps a cracked leg under the mech's weight.
- **Later:** bipeds (a capped upright assist whose cap shrinks with capability; hopping and crawling modes), a
  repertoire of gaits searched offline in our own deterministic sim, learned policies trained with random damage.

## 7. Dents (cars and armor)

- a cage lattice in object space (about 4×3×6 nodes for a car) is dented by impacts, capped per node
- render vertices interpolate from the lattice (per-vertex crush weight, like GTA's vertex-colour deformation)
- the hull is rebuilt from the lattice only occasionally (`b3Shape_SetHull` + `ApplyMassFromShapes`)
- dented armor around a joint narrows its limits

## 8. Profiling and hot paths

Performance-maxxing with instruments instead of guesses: a per-phase profiler (fracture, split, stress build and
solve, physics, links, debris), a heavy-load bench rung per game (a race with rigging, a courier run), then the
hottest paths first. Candidates logged so far:
- the stress system: the cost of an iteration (65 ns per bond: a structure of arrays for the 6-vectors, precomputed
  bond blocks instead of `lpAddBlock` per build), a parallel K·x for the biggest structures (one alone may now spend
  4 ms a step), a two-level preconditioner reusing the rigid clusters as its coarse space, a nested-iteration settle;
- the creak loop over stored overloads only; budget units calibrated to time;
- a spinning barrier in the task pool.

## 9. Large-map physics zones

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

## 10. Networking

Options:
- Teardown's approach: destruction is deterministic and sent as commands; bodies are server-synced with a
  priority queue.
  https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html
- Full lockstep on Box3D's determinism, if the toolchain is pinned.

A two-process lockstep experiment comes first.

## 11. Art polish

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
