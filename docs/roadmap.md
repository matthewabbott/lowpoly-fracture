# Roadmap

Principle for every milestone: **buy performance headroom first, then spend it.** A full game means a large map,
several destructible cars, and containers of sloshing volatile reagents, all at once. Every milestone logs its
before/after numbers in [perf-log.md](perf-log.md).

Order (one at a time):
1. Chunky fracture + debris tiers (done)
2. Toppling and stress points (done)
3. Breakable links and assemblies (done)
4. Stress at scale (done, but for step 6 if measurements ask for it)
5. Articulated objects and systems (vehicles first) (done)
6. Creatures and mechs (done)
7. Deep research: destruction engine architecture (research done; its hardening step next)
8. Engine surface and diagnostics
9. Independent review: simplicity
10. Dents (cars and armor)
11. Profiling and a performance review
12. Large-map physics zones
13. Networking
14. Art polish and demo views

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
   tests at 40 m/s, full lock, kerbs, slopes and hard landings (done: about 3 us per wheel per step).
2. A track scene, a drive mode in the sandbox (chase camera, recorded controls), scripted drivers for the bench
   (done: three box cars lap a ring road with kerbs, crates, a hump and a plank bridge in 22 s; the 'track' rung).
3. Part identity and part detonators (done: a fuel tank goes off alone; objects' detonators and every hash as before).
4. Channels and supply (done: a cut line goes dry in the same step, an engine without fuel makes no power, half an
   engine gives half, a hose carries fuel between objects, a car coasts once its engine is knocked off).
5. A car kit (frame, sheet-metal panels on bolts, glass, engine, fuel tank, steering box) and crash calibration (done:
   with step 7's stress on moving bodies, into a brick wall at 10, 20 and 30 m/s the car keeps 96, 79 and 75% of
   itself, and its power only at 10 m/s).
6. Muscles (motorised links, with a patch for Box3D's revolute torque getter) and a crane on the structure stress path
   (done: servos hold to their cap and sag past it, go limp unfed or brake, survive a rebuild; the track's crane loads
   its tower's footing three times harder with a 2 t load than a 400 kg one).
7. Inertia relief: stress on moving bodies, triggered by load spikes (done: measured acceleration, pinned where struck;
   a car into a wall at 30 m/s loses its engine and front wheels and keeps its cabin, at 10 m/s only a bumper).

8. Wrap-up: docs, the agent map, the memory note (done).

Deferred: pools (blood, fuel, hydraulic fluid) to milestone 6; crumpling to dents (milestone 10).

Open:
- car-on-car crashes: Box3D sweeps only against static bodies, so two fast cars could pass through each other; not
  measured yet (an `isBullet` flag per vehicle if they do);
- Box3D's wheel joint stays unused (the fallback if the shape-cast suspension ever fails a case); its force getters
  are wrong (PATCHES.md);
- the track's box scenery is placeholder art; a real track and AI drivers belong to the game.

## 6. Creatures and mechs

The hexapod mech first: statically stable, so losing a leg means planning a new gait, and a procedural gait looks
right on a machine. It is car-sized (a 2.5 x 1 x 3 m torso, about 3 t, 2 m legs, 2 to 3 m/s) and its front legs double
as arms: they stomp, and a claw grabs what it touches. Everything it does adapts to damage by itself.

- **Rigs, in the core like vehicles:** a rig creates no physics. The kit makes the leg segments and joins them with
  motorised hinges; `lpCreateRig` takes limbs, each an ordered chain of 1 to 3 of those links from the torso outward
  and a foot point. It never names a body (the torso is the body holding most root links). Controls go in as
  persistent hashed state (`lpWorld_SetRigControl`, `lpWorld_SetLimbTarget`), a capability summary comes out.
- **Kinematics:** from the links' end frames and measured angles, so it survives splits and joint rebuilds. IK is
  damped least squares on the chain as it is (fixed iterations, warm-started, limits clamped): stumps and strikes need
  nothing special. The servos get a feedforward speed from the next step's pose, so a clamped leg does not drag the
  others.
- **Capability per limb:** attached, the intact chain, its strength (the weakest link's cap over its maximum), its foot
  (after a break, the far end of the last segment left: a stump walks as a peg), its reach.
- **Gait:** stance feet stay where they actually are (slip is accepted, never fought) and the torso is pushed toward
  its pose a step ahead, level, at a height that drops as strength does and as the shortest leg requires. A free gait:
  the most stretched leg lifts if its neighbours are planted and the centre of mass stays over the other feet with a
  margin, which makes a tripod on six legs, a ripple on five and a wave on four with no replanning. Below that every
  lift fails the check, the body drops onto its belly skid and crawls. Footholds are cast like wheels. A standing mech
  sleeps.
- **Impairment:** pools (hydraulic fluid, blood, fuel) on source parts, drained by leaks in proportion to the carrier
  volume lost (chips shot out of a line leak a little, a severed leg a lot), feed their channels in proportion; nerves
  are a channel a motor needs (unfed, limp); damage at a joint jams it (slower, and it sticks).
- **Maimed attacks:** a limb target takes a limb out of the gait only while the rest keeps it balanced, so a maimed
  mech loses its strikes on its own; a weak swing hits softer on its own, since impacts take their damage from
  impulse. A grab is a weld: lose the claw and the load drops.
- **Bones:** the legs solve their stress; a cracked femur snaps on landing.
- **Not doing:** collision groups (a mech's own debris would pass through it), phase tables, analytic IK.
- **Later:** bipeds (a capped upright assist whose cap shrinks with capability; hopping), a repertoire of gaits
  searched offline in our own deterministic sim, learned policies trained with random damage.

Steps (each measured in [perf-log.md](perf-log.md)):
0. This reshape.
1. Kit physics and the rig core: the kit's bodies and servos standing (a substep sweep first), kinematics, IK,
   capability, feedforward and a tear ratio per link (done: it stands on 4 substeps with its servos at gain 4, sags
   1.5 cm and sleeps after 3 s; IK to 0.03 mm; a crouch lands within a millimetre; 11 us a step).
2. Gait: footholds, stance and swing, the free gait, the idle latch; walking straight, turning, slopes, steps, stopping
   (done: a tripod at 2.27 m/s with 1 cm of drift over 23 m, 82% of that up 15 degrees, 0.4 m steps; legs padded with
   inertia (`lpObjectDef.inertiaRadius`) so Box3D holds their joints; 13 us a step).
3. The mech scene, driving a rig in the sandbox (walk events; reach and grab come with strikes), the 'mech' rung
   (done: the patrol goes round its yard, over a step, rubble and a hump, in 58 s; V takes the mech, WASD/QE/C).
4. Damage adaptation: lost legs, pegs, weak legs, crawling (done: on five legs 41% of its pace, on four 11%, on three
   or one leg left on a side it crawls; a peg walks level; a weak leg lowers the body; the kit gets its systems and an
   armored hull, `lp_armor`).
5. Pools, nerves, jam (done: a cut line leaks its share of the pool a second until its seal closes, following the model
   to 0.01%; the mech loses a fifth of its fluid with a leg, and bleeds dry and collapses with its valves stuck open;
   a jammed joint turns at half speed at half health and holds unfed).
6. Strikes and grabs (done: a reaching limb leaves the gait only when the rest keep it balanced, reaches within 7 mm,
   stomps 4 times harder whole than at half health; a claw grabs and lifts 200 kg; F and G in the sandbox, reach and
   grab events).
7. Bones and landings (done: the kit solves its stress; femurs are welded halves (`lp_jointWeld`) a blast cracks and a
   landing snaps; hard hits jolt the bodies linked to them; the torso is solved twice a second walking; a falling rig
   follows its torso down instead of springing back up).
8. Wrap-up: docs, the agent map, the memory note (done).

Deferred: cosmetic drips where a carrier link broke.

Open:
- a hard landing rebounds at up to Box3D's `contactSpeed` (3 m/s), which pushes sunken soles back out of the ground; a
  lower world setting would change every scene's hashes, so it waits for a reason beyond the mech;
- the damaged gaits move by about 5% with small changes to the kit (a sole's mass, its shape): their tests keep a
  margin under what the gait asks (five legs make 70% of the half pace asked);
- the sandbox's strike aims from the chase camera's fixed view, so a leg strikes the ground ahead of it or what the
  crosshair is on; aiming belongs to the game;
- the kit is placeholder art; bipeds, hopping and learned gaits are under "Later" above.

## 7. Deep research: destruction engine architecture

**Research done (2026-09-30):** [multiplayer-research.md](multiplayer-research.md) is the decision record, with the
evidence in [research/](research/).
- **The model:** convergent lockstep.
- **Determinism on the CPU** holds across OSes, CPUs, compilers and emulators once three hazards of ours are fixed.
- **The GPU can join the simulation in block-scaled integers,** with a CPU twin.

**Proposed next:**
- this milestone's hardening step (L0), widened with the lag and spike experiments;
- a GTX 1060 bought now to measure the floor.

The reordered roadmap it proposes (its §5) waits for the owner's choice; until then the order above stands. The notes
below framed the research.

Before building more: how others build destruction engines and what they give up for speed, to choose the
architectural trade-offs worth making now. Each one made early saves reworking later, and performance outranks
simplicity: a trade that complicates the engine is worth it if it buys bigger cities, hordes, and more debris.

- **What must be bit-exact across machines?** The multiplayer model decides it, so it comes first. Teardown's (read
  2026-09-29, https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html): destruction is deterministic, rewritten in
  fixed-point integer math (its voxels are discrete), and sent as commands on a reliable channel ("cut hole in this
  shape at voxel coord x,y,z"); bodies are not simulated deterministically but synced from the server in floating
  point (transforms and velocities, a priority queue favouring what each player sees, about 1 Mbit per client,
  eventually consistent). Under such a hybrid, Box3D and a GPU solver may stay in float (only the host's result
  counts), and only our destruction geometry must agree everywhere: fixed-point there is our own code, far smaller
  than all of physics. Full lockstep needs every part of the simulation bit-exact on every machine: fixed-point
  everywhere, which means a physics engine of our own. A cheap first experiment: the same scripts built with MSVC and
  clang, hashes compared.
- **Our destruction leans on physics more than Teardown's:** stress breaks joints from loads the physics produced. In a
  hybrid, the host decides (these bonds break, this piece fractures here with this seed) and sends the decisions;
  clients apply them identically. The research checks what else follows (links, supply, rigs).
- **The GPU:** GPU rigid bodies (PhysX, the CUDA port of Box3D), what a fixed-point GPU solver would cost (integer
  throughput, 64-bit products), and what can go to the GPU with no determinism at all (the cosmetic layer: dust, far
  debris, mess).
- **Parallel patterns that stay deterministic:** an order fixed by the data, never by the threads (graph-coloured
  solvers as in Box2D and Box3D, Noita's checkerboard chunk updates, reductions over a tree shaped by the count alone,
  deterministic sorts, no float atomics), or arithmetic whose order does not matter (integer or fixed-point sums,
  binned "reproducible" float sums). GPUs round floats differently across vendors and drivers (fused multiply-adds,
  transcendentals, denormals), so a GPU result is bit-exact only in integer or fixed-point math. Rounding a converged
  result to a grid ("converging" to a shared answer) makes most runs agree but still flips values near a grid line:
  it narrows desyncs without removing them.
- **Destruction architectures to mine:** Teardown and Gustafsson's newer engine, NVIDIA Blast and its stress solver,
  Unreal's Chaos Destruction and its cluster hierarchies, Red Faction: Guerrilla, Frostbite, Havok, Nebenan, Photon
  Quantum's fixed-point physics, open fixed-point physics ports.
- **Trade-offs to weigh:** destruction decided by the host and sent as commands; a harder split between simulation and
  cosmetics, with most debris cosmetic (and on the GPU); systems at lower rates than physics (stress, supply, rigs),
  staggered by count; precomputed fracture patterns; structures of arrays and a job graph across phases; physics
  fidelity by distance from observers.

Output: a report with each technique's gain, cost, determinism and fit, the trade-offs we take (each placed on the
roadmap: a fixed-point destruction core, a GPU cosmetic layer, ...), and the multiplayer model chosen.

## 8. Engine surface and diagnostics

The line between the engine and the games made on it: the core holds mechanisms and data-driven definitions; the kits
in `scenes/` (the car, the crane, the hexapod) are example content, kept in the repo because the tests and the bench
need realistic loads and because building them finds the engine's gaps (milestone 6 found four). A short pass where
game policy leaked into the core:
- `lpSetJoint`, as `lpSetMaterial` does for materials (the joint table is fixed today);
- the gait as one replaceable walking policy: `rig.c` keeps the mechanism (the model, IK, capability, balance checks,
  foothold casts, per-foot targets a game can drive itself), `gait.c` is the statically stable many-legged walker, its
  tuning in a def instead of constants (the parity groups, the speed asked of five and four legs, crawling);
- other constants that are game policy move into defs (a pool's leak rate, the strike speed);
- the docs say what is engine and what is example content.

Diagnostics for agents and tests (numbers, not pictures): queries for bonds (their pieces, position, load, utilization,
health) and contacts (points and forces), and a sandbox `--dump` of the state at a tick (pieces, bonds, links, rigs)
as JSON. Debugging milestone 6 ran on traces like these, written by hand each time; the reviewers below verify
outcomes with them too.

## 9. Independent review: simplicity

The core grew by two large milestones of mechanisms tuned by iteration (special cases pile up: three tick fields decide
when a moving body's stress is checked). Agent-friendliness comes first, and it depends on how small and plain the code
is. Other models review it with the current engine as the source of truth:
0. **An outcome catalogue:** every behaviour we like, each pinned by a test with a tolerance or by a scripted scene with
   reference screenshots; the gaps found and filled first. The contract is the outcomes, not the bench hashes (a
   simpler design may change the numerics). The first draft, with the goals and the feel it serves, is in
   [goals.md](goals.md).
1. **Reviews:** independent reviewers (Fable; GPT-6 Astra, run in Codex; the Codex and Kimi reviewers set up here),
   each given the goals, the catalogue and the code, read-only, propose simpler or more concise ways to reach the same
   outcomes, per subsystem, with the size they expect to save. One of them prunes, having read
   [How Complex Systems Fail](https://how.complexsystems.fail/) first (the "bonsai agent" from the first week).
2. **Adjudication:** each proposal is built on its own; it stays if the catalogue passes, the code shrinks, and the
   bench does not regress beyond noise. The size of the core (tokens) is logged before and after.

## 10. Dents (cars and armor)

- a cage lattice in object space (about 4×3×6 nodes for a car) is dented by impacts, capped per node
- render vertices interpolate from the lattice (per-vertex crush weight, like GTA's vertex-colour deformation)
- the hull is rebuilt from the lattice only occasionally (`b3Shape_SetHull` + `ApplyMassFromShapes`)
- dented armor around a joint narrows its limits

## 11. Profiling and a performance review

Performance-maxxing with instruments instead of guesses: a per-phase profiler (fracture, split, stress build and
solve, physics, links, debris), a heavy-load bench rung per game (a race with rigging, a courier run) with a frame
budget each, then the hottest paths first.

Outcomes may change here, as long as the feel holds:
- **The feel goals,** written down in [goals.md](goals.md), so a taste reviewer can judge against them.
- **The research's survey again, against profiles** (milestone 7 made it): which of the tricks it tagged pay off where
  the time actually goes.
- **The cosmetic layer as a lever:** determinism binds only what feeds back into play. Anything that never does (the
  smallest debris, dust, far-off mess) may use what the simulation may not: camera distance, time budgets, the GPU, and
  a different result on each machine. Moving more into it buys room.
- **What the research chose** (milestone 7): the trade-offs taken, built and measured here if not before.
- **Reviews:** independent reviewers (as in milestone 9) propose changes that trade fidelity for speed; before-and-after
  footage of the same scripts goes to a taste reviewer (an Opus model) against the feel goals; what is kept is logged
  with its numbers.

Candidates logged so far:
- the stress system: the cost of an iteration (65 ns per bond: a structure of arrays for the 6-vectors, precomputed
  bond blocks instead of `lpAddBlock` per build), a parallel K·x for the biggest structures (one alone may now spend
  4 ms a step), a two-level preconditioner reusing the rigid clusters as its coarse space, a nested-iteration settle;
- the creak loop over stored overloads only; budget units calibrated to time;
- a spinning barrier in the task pool.

## 12. Large-map physics zones

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

## 13. Networking

Options:
- Teardown's approach: destruction is deterministic and sent as commands; bodies are server-synced with a
  priority queue.
  https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html
- Full lockstep on Box3D's determinism, if the toolchain is pinned.

The research (milestone 7) chooses the model; a two-process experiment of it comes first here.

## 14. Art polish and demo views

Shaders, palette, sky, ambient occlusion, character pipeline (RetroDiffusion low-poly GLB with auto-rigging, Meshy
with target poly count, Blender cleanup via its MCP), point-filtered character textures. Essential for any public
demo; deliberately last.

Demo views that make the forces legible, for short videos (the sandbox's L already colours links by load):
- a stress heat map on the pieces, and the load paths drawn along the bond graph;
- contact forces as arrows;
- a rig's support polygon, centre of mass, foot targets and swing arcs;
- supply and pools flowing, and draining through a cut.

Effects and mess, noted while tuning debris:

- **Dirt where debris lands:** decals or vertex-colour darkening on the ground and on walls around scrap and rubble,
  so a fight leaves the place dirty even after the pieces are cleared or sunk.
- **Smoke and fire:** particle smoke and volumetric fire (Teardown-style). Fire ties into material flammability
  ([materials.md](materials.md)).
- **Dust:** soft, fading dust clouds instead of solid motes once a transparent particle pass exists.
- **Toaster profile:** one switch that lowers caps (debris, ghosts, scrap), raises the fragment scale and render
  scale, and turns off shadows, so low-end machines still get the destruction.

## Research queue

Deep dives that need no code and can run whenever agents are free, placed before the milestone they inform.

### Adaptive locomotion: generated and learned gaits for damaged bodies

Informs the creatures milestone that follows the physics-core swap and the character controller (bipeds, organic
creatures, the player as a body).

**Why.** Bodies should adapt to damage on their own (goals.md, "North star"). The hexapod's procedural free gait already
does this for statically stable machines. Bipeds, organic creatures and a believable hobble need more, and procedural
gaits read as mechanical on organic bodies.

**Leads.** From a cursory look on 2026-09-30:
- **UniMate**
  - Sources: Mou et al., "One Unified Model to Animate Diverse Skeletons", https://arxiv.org/abs/2609.05415; code at
    https://github.com/Friedrich-M/UniMate under MIT.
  - What it is: text-to-motion for arbitrary skeleton topologies (bipeds, quadrupeds, birds, fish, insects, snakes),
    zero-shot, with no per-skeleton retraining. It is a flow-matching diffusion transformer that also does
    in-betweening and text-guided edits.
  - Limits:
    - Its output is kinematic (joint rotations over time), not a physics controller.
    - It animates a given skeleton; it does not build one. Building skeletons is the art pipeline's auto-rigging track.
    - It was trained on 13k clips from Mixamo, Objaverse-XL and Truebones Zoo, whose licences govern use in a
      commercial game.
- **Repertoire search** (Cully et al., "Robots that can adapt like animals", Nature 2015): thousands of gaits
  precomputed offline (MAP-Elites), then searched in a few trials after damage. This is the owner's catalogue idea with
  a published recipe.
- **Morphology-agnostic learned policies** (shared modular policies, 2020; MetaMorph, 2022, and successors): one
  physics-based controller for many bodies, zero-shot on new ones.

**Questions:**
1. Can a usable gait for a newly maimed body be produced in a second or two, or must gaits be precomputed? UniMate
   claims real-time generation, but its motion is kinematic: our servos and IK must track it, and it knows nothing of
   balance.
2. **The catalogue.** For each entity, precompute the skeletons damage can produce (each limb lost or shortened).
   Generate or search a gait for each one offline, in our deterministic sim. At run time, snap a damaged body to the
   nearest entry, treating extra limbs as dead weight.
   - How large is a catalogue per entity?
   - How are topologies matched?
   - How is a gait retargeted to the true proportions?
3. **Determinism.** A generated motion is data, so it is safe in lockstep if it is precomputed, or generated once by the
   host and sent as a command. A learned policy evaluated every tick must run in integer arithmetic (quantised
   inference) to be bit-exact across machines; that fits the integer core.
4. Which reads better as limping, hobbling and crawling, and which survives the integer core: a kinematic reference
   tracked by our IK and servos, or a learned physics controller?
5. The licences of the models and their datasets for a commercial game.

**Output:** a report in the style of milestone 7's (`docs/research/`), with a recommendation and a prototype plan for
the creatures milestone.
