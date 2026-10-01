# Roadmap

Principle for every milestone: **buy performance headroom first, then spend it.** A full game means a large map,
several destructible cars, and containers of sloshing volatile reagents, all at once. Every milestone logs its
before/after numbers in [perf-log.md](perf-log.md).

The direction since milestone 7 (agreed 2026-09-30, [multiplayer-research.md](multiplayer-research.md),
[goals.md](goals.md) "North star"): lockstep multiplayer on a simulation that is deterministic from the ground up, and
Box3D replaced by a block-scaled integer rigid-body core that runs on the GPU with a bit-identical CPU twin.

Standing practices:
- every milestone ends with a short simplicity pass by an independent reviewer (seams first: a subsystem behind a
  clean boundary with its own outcome tests can be swapped whole);
- performance is measured at 1 and 2 workers and on the Intel GPU as the low-end proxies (fewer busy cores, less
  heat), as well as at 8;
- the `determinism` CI runs on every push to `sandbox`; `master` is fast-forwarded to `sandbox` after each milestone;
- every push range is checked for game names first (the games live in their own private repository).

Order (one at a time):
1. Chunky fracture + debris tiers (done)
2. Toppling and stress points (done)
3. Breakable links and assemblies (done)
4. Stress at scale (done, but for step 6 if measurements ask for it)
5. Articulated objects and systems (vehicles first) (done)
6. Creatures and mechs (done)
7. Deep research: destruction engine architecture (done), closed by determinism hardening
8. The physics seam
9. The outcome catalogue, the engine surface and a seams-first review
10. Commands, hashes and the first co-op
11. Integer groundwork
12. The integer core with its CPU twin
13. The core on the GPU
14. The snapshot
15. The character controller
16. Networking
17. Profiling and a performance review
18. Large-map physics zones and persistence
19. Creatures, part 2
20. Dents (cars and armor)
21. Art polish and demo views

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
  near the damage; milestone 10 makes it stress's finite speed of propagation) or a parallel K·x with fixed partitions;
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

Deferred: pools (blood, fuel, hydraulic fluid) to milestone 6; crumpling to dents (now milestone 20).

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

**Done (2026-09-30):** [multiplayer-research.md](multiplayer-research.md) is the decision record, with the evidence in
[research/](research/) (ten tracks, a synthesis, a red team).
- **The model:** convergent lockstep. Every machine runs the whole simulation from one command log; the host relays
  inputs and keeps the clock; the local character is predicted outside the deterministic world; per-object hashes
  localise a divergence and the host repairs that object.
- **Determinism on the CPU** holds across Windows, Linux and macOS, x64 and ARM64, five compilers and three x64
  emulators, once three hazards of ours were fixed.
- **The GPU can join the simulation in block-scaled 32-bit integers,** bit-exact on NVIDIA, Intel and a CPU twin, at
  1.16 to 1.35 times the float solve's cost; replays survived an NVIDIA driver update.

**Closing step: determinism hardening (L0).**
- the cross-platform CI (14 legs, per-tick hashes), on every push to `sandbox` and `master`;
- portable maths only: `lpCbrt` and Box3D's trig instead of the C library's `cbrtf`, `sinf`, `atan2f`, in the core
  and the scenes; gcc gets `-ffp-contract=off`;
- what the physics engine reports is acted on in our own total order (freeze candidates, hit events, contact loads,
  cast ties);
- the hash covers the ghosts' landing plans, detonators' armed flags, wheel spin and the limbs' depth and tip cache;
- the floating-point control word is guarded (`lpFpGuard`, a Box3D scheduler patch) and the arithmetic self-tested
  (`lpDeterminismSelfTest`);
- float-to-int through a clamp where unbounded; no NaN in state (asserted); no random draws inside one call's
  arguments or one initializer;
- the sandbox records exact floats (`%.9g`) and can delay walk and drive events (`--input-delay N`) for the owner's
  feel test of a predicted first-person body; the bench writes per-tick times (`--tick-log`) and counts ticks over
  16.7 and 33 ms;
- rules 12 to 18 in [determinism-rules.md](determinism-rules.md).

## 8. The physics seam

Every Box3D call (about 266 sites in 12 core files, 83 functions) goes through one internal interface, so the integer
core can replace Box3D behind it, and so the rules about report order live in one place.
- **Own the maths:** Box3D's maths helpers (MIT) become our `lp` types in `src/`, renamed mechanically (about 2,000
  uses), hash-neutral.
- **The interface:** `src/phys.h` with a Box3D backend (`src/phys_box3d.c`): bodies, hulls, the four joints, motors,
  hit and move events, contact data, queries and casts, all returning results in our own order. No Box3D call outside
  the backend.
- **Exact wakes:** wake and shove queries test the pieces' real bounds instead of the tree's fat ones (T0 #4).
- **The lag experiment** (red team R16): a flag that makes the core read physics results one tick late, as the GPU
  pipeline will; the nine suites and the track and mech rungs decide whether the tyres, the servos and the gait
  tolerate it.

Exit: hash-neutral but for the flagged run; no Box3D call outside the backend; the lag verdict recorded.

## 9. The outcome catalogue, the engine surface and a seams-first review

The catalogue is the contract the integer core must meet, so it comes before the core is replaced.
0. **An outcome catalogue:** every behaviour we like, each pinned by a test with a tolerance or by a scripted scene with
   reference screenshots; the gaps found and filled first. The contract is the outcomes, not the bench hashes (a
   simpler design, or a new physics core, changes the numerics). The first draft, with the goals and the feel it
   serves, is in [goals.md](goals.md).
1. **The engine surface.** The core holds mechanisms and data-driven definitions; the kits in `scenes/` (the car, the
   crane, the hexapod) are example content, kept because the tests and the bench need realistic loads and because
   building them finds the engine's gaps. Where game policy leaked into the core:
   - `lpSetJoint`, as `lpSetMaterial` does for materials (the joint table is fixed today);
   - the gait as one replaceable walking policy: `rig.c` keeps the mechanism (the model, IK, capability, balance
     checks, foothold casts, per-foot targets a game can drive itself), `gait.c` is the statically stable many-legged
     walker, its tuning in a def instead of constants;
   - other constants that are game policy move into defs (a pool's leak rate, the strike speed);
   - diagnostics for agents and tests (numbers, not pictures): queries for bonds and contacts, and a sandbox `--dump`
     of the state at a tick as JSON.
2. **A seams-first review:** independent reviewers (Fable; GPT-6 Astra in Codex; the Codex and Kimi reviewers), each
   given the goals, the catalogue and the code, read-only, propose clean boundaries and simpler ways to reach the same
   outcomes per subsystem: fracture, stress, links, wheels and rigs, the debris tiers, supply. One of them prunes,
   having read [How Complex Systems Fail](https://how.complexsystems.fail/) first. The physics plumbing that milestone
   12 replaces is out of scope.
3. **Adjudication:** each proposal is built on its own; it stays if the catalogue passes, the code shrinks, and the
   bench does not regress beyond noise. The size of the core (tokens) is logged before and after.

## 10. Commands, hashes and the first co-op

The multiplayer primitive's first half (L1a in the research), and two machines playing together on today's Box3D.
- **A command queue in the core:** every input (impacts, pulls, blows, controls, limb targets, grabs, spawns)
  tick-stamped, with exact floats and keyed references, applied in `(peer, sequence)` order before the step; the
  sandbox's tools and claw become commands (T0 #18, #19).
- **Stable ids:** generations for bonds, wheels and bodies; global keys for runtime objects (T0 #13, #14).
- **The incremental hash:** a digest per piece at creation, per-object hashes grouped under a small Merkle tree,
  covering the stress solver's state and Box3D's contact state; `lpWorld_Hash` stays as the full check (T0 #20).
- **Session settings and a handshake:** the counts, the build, the content hash and the self-test hash agreed before
  play.
- **Causal units:** islands (bodies joined by contacts and joints), structures (a bond graph, coupled by its stress
  solve), linked assemblies and supply networks: the groups whose members affect each other and nothing outside within
  a tick. Hashes are grouped by unit and include each unit's hidden state (warm starts, contact caches, sleep timers,
  solver progress).
- **A finite speed of propagation on the non-local channels** (the owner's idea, 2026-10-01). Between islands, cause
  and effect already travels no faster than matter moves. The channels that reach across space within a tick get a
  speed too, a few metres per tick, tunable and perhaps per material:
  - the stress solve (the k-hop patch, growing from a change a few hops per tick);
  - supply (a pressure drop running down a line);
  - queries (a bounded range, or the path counted in the cone).

  Then a cause's reach after d ticks is bounded, so repair regions, rollback regions and zone boundaries are bounded
  too. A collapse becomes a cascade spread over ticks, which also smooths the spikes. The catalogue (milestone 9)
  judges what it does to stress outcomes, and the speed is tuned so stiff things stay stiff.
- **Repair by causal unit:** a mismatched object's whole unit is replaced, plus every unit it touched since the last
  tick all machines agreed on, hidden state included. Every machine, host included, restores those units to the host's
  image of tick T and re-simulates only them from T to now; that is exact when the units were causally closed over that
  span, and cheap because they are small.
- **A two-world harness** in `lpf_test`: two worlds step the same commands; an injected desync is named by object and
  tick; repair by causal unit is tried and its reconvergence measured (red team R3); how fast real causal cones grow in
  the bench scenes is measured.
- **The first co-op:** a two-process lockstep smoke test over a plain socket (start together, the host keeps the
  clock), with the `--input-delay` feel test's verdict applied to the walking body.

Exit: two processes stay in sync on a scripted session; an injected desync is named.

## 11. Integer groundwork

Fail fast before the integer core is built.
- **An integer toy** on the CPU (about 1,000 lines: boxes, SAT, the block-scaled soft step, a joint, sleep): does a
  stack stand, does a rubble pile settle, how far from float are the results (red team R15)?
- **Exact integer fracture geometry** (L2): integer sites and bisector planes, exact classification, hulls from exact
  topology; removes the plane-shift hack and every tolerance flip in fracture.
- **The floor GPU:** E11's binaries on a GTX 1650 or 1060-class box (the integer multiply on Pascal), and on an AMD GPU
  or a Steam Deck when one is to hand.

Exit: stacking holds; recorded blasts fracture exactly; the floor GPU's integer cost measured.

## 12. The integer core with its CPU twin

The design in [research/m7-gpu-integer.md](research/m7-gpu-integer.md): block-scaled 32-bit fixed point with 64-bit
products, 64-bit world positions, per-body exponents; Box3D's graph-coloured soft step transcribed, coloured per tick
from state; bodies, hulls, a sorted broadphase, SAT, the four joints with motors and limits, islands and sleep, GJK and
casts; the one-tick pipeline lag as deterministic semantics. It runs behind the seam beside Box3D, switchable per world,
with overflow freedom checked (lint rules, UBSan, Frama-C or CBMC on the kernels). Two requirements from milestone 10's
design:
- it can step a chosen set of causal units in isolation (repair and scoped rollback re-simulate only those);
- it keeps a short ring buffer of each changed unit's state for the last few ticks, written incrementally.

Exit: the catalogue passes with tolerances on the integer core; the twin at 1 worker within 2.5 times Box3D (1.5 with
AVX2); identical hashes across the CI matrix.

## 13. The core on the GPU

A Vulkan runtime (one command buffer per tick, timeline semaphores, readback), every kernel from the core's shared
C-and-HLSL sources, differential tests against the twin on every GPU to hand, the startup self-test battery in the
handshake with the twin as the fallback, and the GPU or the twin chosen per tick by body count (they give the same
bits). Box3D and its patches are deleted at the end if the gate passes; if it does not, Box3D stays.

Exit: 5,000 awake bodies under 16.7 ms on the low-end box; 20,000 under 8 ms of GPU time on the 3060; hashes
identical after a driver update.

## 14. The snapshot

The primitive's second half (L1b): serialise and rebuild the world bit-exactly on the integer core (plain integers,
written once): saves, late join, host migration, repair images, zone persistence, replays with seek.

Exit: the round trip exact on every rung; a join measured at the town's peak.

## 15. The character controller

The player as a destructible body (pools, vitals, knockout), predicted as the feel test decided: a kinematic latency
state rebased on the canonical body, or the body itself under input delay.

## 16. Networking

Lockstep over the snapshot and the command log: transport (Steam sockets with a direct fallback, the send rate raised
above its default), server-timed ticks with a late-input tolerance, the latency-state character, join with a pause or
a catch-up, host migration, repair by causal unit, desync reports that replay headless.

**Causally scoped rollback** (the owner's idea, 2026-10-01), an option on top of lockstep, measured against plain input
delay. Rollback netcode is itself built on deterministic lockstep: every machine predicts the other players' inputs
(usually "the same as last tick") instead of waiting for them, and corrects itself when the real ones arrive. Classic
rollback re-simulates the whole world for every wrong guess, which a destruction world cannot afford. Scoped:
- On a wrong guess, only the cone of that input (bounded by the speed of propagation over the late ticks) is restored
  to its confirmed state and re-simulated with the true input; everything outside it never depended on the guess and
  is already right.
- Each machine computes the truth itself from the confirmed inputs, so no state is shipped, and the corrections never
  enter the shared history: a late joiner replays confirmed inputs only.
- A player's own actions on the world feel instant; reality shifts under them when a guess about someone else was
  wrong.
- Destructive inputs (blasts, tools) probably keep their input delay: a wrong guess there would mean re-simulating a
  collapse.
- The guess is "the same as last tick" (players hold keys), so remote players only jump when they change what they
  press; the renderer eases the drawn position toward each correction (cosmetic, so free to differ per machine).
- **Adaptive delay** (the owner's idea): each player stamps their own inputs a few ticks ahead, so the delay can differ
  per player and per kind of input, and change during play without global agreement. It is driven by the measured cost
  of recent wrong guesses (cone sizes times re-simulated ticks) on the slowest machine: near zero for someone running
  through the woods, more for someone in a collapse, held for destructive inputs. Raised at once, lowered a tick at a
  time. Fighting games fix their delay at the match start from ping; Photon Quantum scales it with ping. Driving it by
  re-simulation cost seems new, because nobody else runs rollback where that cost varies this much.
- Without determinism the truth would have to be shipped from the host, and divergence could start anywhere, so cones
  could not be bounded: the lockstep base is what makes it work.

Exit: a 4-player session across machines.

## 17. Profiling and a performance review

Performance-maxxing with instruments instead of guesses, on the architecture that ships: a per-phase profiler
(fracture, split, stress build and solve, physics, links, debris), a heavy-load bench rung per kind of game with a frame
budget each, a synthetic 20,000-body rung (the CPU side of a GPU core: freezing, wakes, tiers, hashing, rendering sync),
then the hottest paths first.

Outcomes may change here, as long as the feel holds:
- **The feel goals,** written down in [goals.md](goals.md), so a taste reviewer can judge against them.
- **The research's survey again, against profiles:** which of the tricks it tagged pay off where the time goes.
- **The cosmetic layer as a lever:** determinism binds only what feeds back into play. Anything that never does (dust,
  sparks, far-off mess) may use what the simulation may not: camera distance, time budgets, the GPU, and a different
  result on each machine. It is not a priority.
- **Reviews:** independent reviewers propose changes that trade fidelity for speed; before-and-after footage of the
  same scripts goes to a taste reviewer against the feel goals; what is kept is logged with its numbers.

Candidates logged so far:
- the stress system: the cost of an iteration (65 ns per bond: a structure of arrays for the 6-vectors, precomputed
  bond blocks instead of `lpAddBlock` per build), a parallel K·x for the biggest structures, a two-level
  preconditioner reusing the rigid clusters as its coarse space, a nested-iteration settle;
- the creak loop over stored overloads only; budget units calibrated to time;
- a spinning barrier in the task pool;
- memory on the floor machine: 432-byte bodies and 296-byte pieces carrying stress and ghost fields everywhere, hulls
  held twice, the fracture scratch (T0 #22); vehicles and rigs that never die (T0 #23).

## 18. Large-map physics zones and persistence

- **Active cells** along the track, around "observers": players, AI cars, and flagged "chunk loaders" (armed
  contraptions, timers, detonators; anything the player sets up deliberately), like Minecraft's simulation distance.
  In lockstep every peer simulates the union of active cells: measure it for 8 to 12 spread-out players on the floor
  machine (the research's flip condition for a hybrid).
- **Outside the active cells:** resting islands sleep; loose debris is resolved by tier (snapped down and frozen, or
  removed); unstable structures are settled with the stress check; flagged Rube Goldberg setups keep simulating.
- **Persistence:** zones snapshot alone; a piece lives awake, asleep, frozen, then baked into its zone's static
  geometry, and may be forgotten; the engine offers freeze, bake, wake, delete, restore-to-authored and budgets, the
  games choose the policy. Identity is opt-in, so anonymous rubble bakes and only tagged props persist as bodies (the
  "10,000 cabbages" problem).
- **Wake storms:** a grenade among thousands of sleeping props wakes them in a graceful, counted wave.
- **Zones that lag:** with a finite speed of propagation (milestone 10), a zone whose causal cone cannot reach any
  observer in time may be simulated lazily or at a lower rate and caught up deterministically when it can; the union of
  active zones then costs less than every zone at full rate.
- **Determinism:** activity is derived from shared simulation state (player and car positions), never the camera.
- **Streaming:** add bodies in batches. Rendering LOD is a separate track.

## 19. Creatures, part 2

Bipeds and organic creatures whose locomotion adapts to damage, informed by the adaptive-locomotion deep dive in the
research queue below: generated or searched gait catalogues snapped to a maimed body, or learned controllers in
integer arithmetic; hopping; a capped upright assist that shrinks with capability.

## 20. Dents (cars and armor)

- a cage lattice in object space (about 4×3×6 nodes for a car) is dented by impacts, capped per node
- render vertices interpolate from the lattice (per-vertex crush weight, like GTA's vertex-colour deformation)
- the hull is rebuilt from the lattice only occasionally (on the physics core that ships)
- dented armor around a joint narrows its limits

## 21. Art polish and demo views

Shaders, palette, sky, ambient occlusion, character pipeline (RetroDiffusion low-poly GLB with auto-rigging, Meshy
with target poly count, Blender cleanup via its MCP), point-filtered character textures. Essential for any public
demo; deliberately last. The cosmetic GPU layer (upload-only debris, dust and mess) lands here if it is wanted.

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

**The test it must pass (the owner's worry).** Our destruction can produce far more skeletons than any catalogue
covers, so the catalogue is judged empirically:
1. Build one for a single monster.
2. Subject the monster to many largely random torments: blasts, cuts, lost and shortened limbs, in combination.
3. Measure how well "snap to the nearest catalogued skeleton" performs: how often it finds a usable gait, how far the
   chosen gait sits from the body's true one, and how the gait looks.

A scripted, seeded headless run, so the score is repeatable.

**Output:** a report in the style of milestone 7's (`docs/research/`), with a recommendation and a prototype plan for
the creatures milestone.
