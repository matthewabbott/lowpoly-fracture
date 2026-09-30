# Milestone 7, track T2: case studies of multiplayer destruction and physics

How shipped (or seriously documented) multiplayer games kept physics and destruction in sync, and every strategy worth
stealing for our case: a 4 to 12 player, player-hosted co-op destruction engine on float Box3D with a deterministic
core. Researched 2026-09-29. Generic netcode techniques belong to track T6; this report records what each game did.

**Method and limits.** Sources are developer blogs, GDC slides (the text of The Finals, Rainbow Six Siege, Space
Engineers and Rocket League decks was extracted and read), source code (BeamMP, Space Engineers 2016 snapshot, Source
SDK 2013, Noita Entangled Worlds, LunaMultiplayer, Hedgewars, 0 A.D.), engine docs and patch notes. The session's web
search budget ran out mid-research, so the rest came from direct fetches of known pages: some games are thin and say so.
Claims marked "unverified" have no primary source; what would verify them is noted. Dates are given where they matter
(this is 2026-09).

## The short version

1. **Nobody ships full rigid-body lockstep for a real-time 3D physics game with destruction.** The one candidate,
   LittleBigPlanet, ran on identical PS3 hardware and is poorly documented. Physics-heavy titles chose host or server
   authority with state sync (Space Engineers, Besiege, Trailmakers, Stormworks, The Finals, Battlefield), owner
   authority (BeamMP, Roblox's legacy model, KSP mods, Fiedler's VR demo) or predict-and-rollback on small scenes
   (Rocket League, Quantum titles, Roblox's 2025 server authority). Lockstep lives in RTS and sim games (Factorio,
   Supreme Commander, Age of Empires, StarCraft II, Spring/Recoil, Worms), where float lockstep across platforms has held.
2. **The two games closest to us converged on the same hybrid.** Teardown (2026, 12 players, player-hosted) and
   Rainbow Six Siege (2015) send destruction as ordered *input events* that every peer applies deterministically, and do
   not replicate debris physics (Siege) or replicate bodies only as prioritised, eventually consistent state (Teardown,
   about 1 Mbit/s per client). Both do join-in-progress by replaying the event log. Siege seeds the fracture RNG from the
   impact position and quantises inputs symmetrically so the host fractures from the same bits as the clients.
3. **Streaming the bodies of a collapse is affordable only with a server farm or a small scene.** The Finals streams
   server-simulated debris transforms: about 400 kbit/s per client in a large event before compression, peaks of about
   175 kbit/s after quantisation and delta compression, from dedicated servers. Crackdown 3's cloud destruction needed
   2 to 4 Mbit/s per player and many servers per match, and was cut back.
4. **Player-hosted physics titles survive by budgets, not cleverness:** a global block or complexity budget (Besiege,
   Trailmakers, Brick Rigs), player caps tied to the host's PC (Besiege 8, Trailmakers 8, Space Engineers 2 starts at 4,
   Teardown 12), rates by distance and motion (Space Engineers), clients that do little physics (Besiege, Stormworks),
   and time dilation with a visible server-speed readout (Space Engineers, Stormworks, Brick Rigs).
5. **Every model needs a world snapshot or a replayable log.** Supreme Commander and Spring could not join mid-game for
   lack of one; Teardown rejected a 30 to 50 MB scene transfer and replays a capped command log; Factorio uploads the
   map and catches up on buffered inputs; Quantum fetches a snapshot from a peer ("buddy"). Brick Rigs and Besiege
   shipped late-join destruction bugs.
6. **The friendslop games are simple and forgiven.** Lethal Company, R.E.P.O., PEAK, Content Warning and Phasmophobia
   are non-deterministic Unity games with 4 to 6 players, owner- or host-authoritative, relayed by the host or by
   Photon; R.E.P.O. keeps all prop physics and breakage on the host and pays a round trip on every grab, and sold
   millions anyway. Deep Rock Galactic sends terrain edits as a numbered operation log and replays it for late joiners,
   the Teardown and Siege pattern in shipped 4-player co-op.
7. **Fixed point is mostly chosen for reasons other than cross-platform floats:** a managed runtime (Quantum, C#),
   discrete data (Teardown's voxels, together with the long-held belief that floats are unsafe, which Gustafsson now
   calls more nuanced), simplicity (Hedgewars, 0 A.D.), huge worlds (fixed3d). The one measurement: Box3D ported to
   Q48.16 runs 2.24x slower than float Box3D, and its authors advise against it because float Box3D is already
   deterministic across platforms.

## One row per game

Abbreviations: SS = state sync, LS = lockstep, RB = predict/rollback, DA = distributed (owner) authority, cmd = input or
destruction commands, P2P = peer-to-peer, JIP = join in progress. "?" = unknown. Sources are in the notes and at the end.

| game (MP year) | engine / physics | model | deterministic | synced | authority | rates, bandwidth | late join / migration | local player | debris |
|---|---|---|---|---|---|---|---|---|---|
| Teardown (2026) | own voxel engine and physics | hybrid: cmd + SS | destruction ops (fixed point) | destruction cmds reliable; bodies, players unreliable | host (player) | ~1 Mbit/s per client, visibility priority | replay capped cmd log; no migration | ? (not described) | ? |
| The Finals (2023) | UE5 + Embark rigid body solver | SS | nothing claimed | transforms of all debris | dedicated server | destruction ~175 kbit/s peak after compression | ? | ? | server-simulated, gameplay |
| Rainbow Six Siege (2015) | Ubisoft engine, RealBlast; Havok FX | cmd for surfaces, no debris sync | surface cutting | impact events | host/server orders | 50 Hz, 60 Hz from 2017 | JIP by event replay | applies own shots at once | cosmetic, not replicated |
| Battlefield BC2 to BF6 | Frostbite (Havok, unverified) | SS | ? | part destruction events, debris spawns | dedicated server | BF4 30 Hz, BF6 60 Hz; 25 m priority bubble (BC2) | ? | ? | prioritised, culled |
| Red Faction: Guerrilla (2009) | Geo-Mod 2 + Havok | ? | ? | ? | ? | 16 players | ? | ? | ? (thin) |
| Crackdown 3 Wrecking Zone (2019) | UE4, Havok on Azure (Cloudgine) | cloud SS | ? | body state cloud to console | cloud servers | 2 to 4 Mbit/s target | ? | ? | faded, chunky after downgrade |
| Just Cause 2/3 MP mods | JC engines, reverse engineered | owner SS | no | transforms | owner | JC2-MP: 75 to 1000 ms by type, 500 m stream-out | JC3:MP: destruction not synced at all | owner | ? |
| Source / HL2DM, Garry's Mod | Source, VPhysics | SS | no | props as entities | server | ? | ? | predicted | gibs client-side temp entities |
| Minecraft | own | SS with edit prediction | no | block deltas, chunk data | server | ? | chunk data on entering view | sequence-numbered edits | n/a |
| Space Engineers (2014-) | VRAGE, Havok | SS | no | inputs up; transforms, fracture events down | server (player or dedicated) | 60 Hz sim; updates every 4 to 60 frames by distance and motion | proximity streaming | only own entity dynamic, parent-relative prediction | local, cosmetic, cap 100 |
| Besiege (2017) | Unity, PhysX | SS | no | inputs up; block clusters down | host | ? | ? | none (waits) | ? |
| Trailmakers (2017-) | Unity, PhysX | SS | no | host physics down | host | 8 players "very good PC" | ? | ? | host-simulated |
| Brick Rigs | UE4 | SS (unverified) | no | ? | host | tick = host FPS | late-join collapse bug | ? | ? |
| Scrap Mechanic | own + Bullet | SS | no | ? | host | no dedicated server (physics too costly) | ? | ? | ? |
| Stormworks | own | SS (was slowest-peer sync) | no | vehicle islands, custom codec | server/host | bandwidth cut 60 to 80% in 2022 | ? | ? | ? |
| From the Depths | Unity | SS hybrid | no | fleet state; missiles only after launch | host | ? | ? | avatar client-side | ? |
| BeamNG + BeamMP | BeamNG soft body | DA (owner) | no | pose 50 Hz, inputs 30 Hz, break groups 15 Hz | owner; server relays | server 5 to 10 Mbit/s up | ? | owner simulates | break groups as events |
| Wreckfest | ROMU | SS + prediction | no | ? | server (player or dedicated) | 2 to 24 players by host bandwidth | ? | predicted | lost tyres synced |
| KSP LunaMultiplayer | Unity | DA (locks) | no | vessel updates, events | lock holders | 50/150 ms by distance | ? | owner | owner applies damage |
| Fiedler VR demo (2018) | Unity, PhysX | DA via host | no (quantised both sides) | quantised state, at-rest bit | toucher; host arbitrates | 10 Hz sync, 90 Hz sim; 256 kbit/s per player | n/a | own avatar | n/a |
| Noita Entangled Worlds (mod) | Noita (Falling Everything, Box2D) | DA by chunk and radius | no | pixel chunk deltas, entity state | chunk/entity owner; host arbitrates | ? | host stores relinquished chunks | owner | ? |
| Factorio | own C++ | LS | whole sim, float, own trig | inputs + CRC | server (player or headless) merges inputs | 60 UPS | map upload + input catch-up | latency state rebuilt each tick | outside sim |
| Planetary Annihilation | own | SS as curves (dropped LS) | nothing | curves of visible units | server | 10 Hz; ~1 Mbit/s per client | JIP, replays, scrub | no prediction | client-derived |
| Supreme Commander / FAF | GPG engine | LS | float, x87 pinned | inputs + hash/s | host P2P | 10 Hz sim; 200 to 300 ms command delay | none | UI feedback | ? |
| Spring / Recoil (BAR) | Recoil | LS | float via streflop, x86-64 only | inputs | host relay | 100 to 200 ms buffer | replay from start | unsynced visuals | unsynced |
| Age of Empires | Genie | LS | float, x87 | inputs + checksums | P2P, host sets turn | 200 ms turns | ? | 250 ms delay | ? |
| StarCraft II | own | LS | whole sim, Galaxy fixed point | inputs | Battle.net | 22.4 loops/s | ? | delay | ? |
| Stormgate (2024) | SnowPlay + UE5 | LS + RB | whole sim (fixed or float ?) | inputs via server | relay server | 64 Hz | ? | rollback, off when CPU short | outside sim |
| Photon Quantum titles | Quantum (C#) | RB | Q48.16 fixed point | inputs | Photon relay | 30 to 60+ Hz | snapshots, buddy snapshots | predicted, zero lag | view layer (Bandit Trap) |
| Rocket League (2015) | UE3 + Bullet | SS + RB of all bodies | no | inputs up per frame, state down | dedicated server | 120 Hz physics | ? | predicted, no delay | presets vs visuals |
| Roblox | own | DA (legacy), SS + RB (2025) | no | parts, ownership | owner or server | ? | streaming | predicted near own character | ? |
| Unity Netcode for Entities | Unity Physics / Havok | SS + RB | per platform | snapshots | server | quantisation 1000 | ? | predicted ghosts | client-only physics world |
| Worms Armageddon, Hedgewars | own | LS | Hedgewars fixed point; W:A ? | inputs | P2P, host | ? | ? | turn-based | ? |

The friendslop and survival titles (co-op, player-hosted or relayed, 4 to 16 players):

| game (MP year) | engine / netcode | model | deterministic | synced | authority | rates, bandwidth | late join / migration | local player | debris, breakage |
|---|---|---|---|---|---|---|---|---|---|
| Lethal Company (2023) | Unity, Netcode for GameObjects, Steam | event + state, host relays | no | positions on change; grab/drop events | owner for players; host validates grabs | send past 0.25 to 0.49 m | none (mods, with desyncs); no migration | owner-authoritative | items have no physics: falls animated from a target |
| R.E.P.O. (2025) | Unity, Photon PUN 2 | host SS | no | object pose and velocity; grab intent up | master client simulates all props | 25 Hz | host leaves = session ends | owner-authoritative | breakage decided on host, sent as events |
| Content Warning (2024) | Unity, PUN | owner SS (inferred) | no | root pose; ragdoll local | owner (inferred) | Photon relay paid by studio | mod only | owner | ? |
| PEAK (2025) | Unity, PUN | owner SS, every client simulates ragdolls | no | hip, input bits, half-float velocities; awake items only | holder owns items | ? | mod, with ghosts | owner; remote ragdolls nudged | ? |
| Human: Fall Flat (2017) | Unity | host SS | no | every rigid body streamed; logic events | host | host must hold 60 fps | ? | not predicted (inferred) | n/a |
| Gang Beasts (2016) | Unity | server SS | no | ? | server (official or player-run) | ? | ? | input only | n/a |
| Valheim (2021) | Unity, ZNet | DA by zone | no | object revisions (deltas) | first to arrive in a zone | 20 Hz round robin, ~10 KB window | server holds state | owner | fragments cosmetic per client |
| Deep Rock Galactic (2020) | UE4 | cmd for terrain | terrain ops | numbered terrain ops, reliable | host | ? | replay the op log over the seeded cave | ? | ? |
| Phasmophobia (2020) | Unity, PUN | owner SS | no | pose of owned props | toucher requests ownership; master reclaims doors | ? | ? | owner | n/a |
| 7 Days to Die | Unity | server SS | no | block changes; falling-block entities | server | ? | ? | ? | falling blocks thinned by count |
| Sons of the Forest (2023) | Unity | ? | ? | ? | host | ? | join cost of structures cut twice in 2023 | ? | ? (thin) |
| Enshrouded, Astroneer, TABS, Mage Arena | | | | | | | | | thin: nothing found |

## Per-game notes

### Teardown multiplayer (Tuxedo Labs, shipped 2026-03-12)

The closest precedent: a destruction sandbox with full modding, 12 players, player-hosted.
- **Model** (Gustafsson's blog, 2026-03-13): destruction is deterministic; most other things are state-synced and
  eventually consistent. The 80.lv interview (2026-03-17) adds a third category, differential state streaming, for
  screens and menus.
- **What is deterministic:** the destruction logic, rewritten in fixed-point integer maths because voxel volumes are
  discrete. A destruction event is split into a stream of deterministic commands (cut a hole in this shape at these
  voxel coordinates, change the ownership of that shape, reconnect this joint to that shape). The float work around
  them (object separation, joints) runs locally on each machine. Gustafsson says full determinism was avoided because
  floats had long been considered unsafe, a view he later found "more nuanced".
- **Channels:** scene changes (destruction, spawning, recolouring) on a reliable channel; transforms, velocities and
  player positions on an unreliable one. The host (which is the server) keeps a priority queue per client that favours
  objects that player can see, within about 1 Mbit/s per client. Clients simulate everything locally and are corrected
  when state arrives; in complex scenes more objects are corrected less often and snap visibly. No smoothing is
  described.
- **Late join:** three options weighed: serialise the whole scene (30 to 50 MB is common), serialise changed objects,
  or replay the command stream. They replay commands, cap the buffer, and disable join-in-progress once it fills.
- **Scripts (mods):** each script has server and client parts with a shared state table and remote calls; old
  single-player scripts keep working in single player only; destruction calls such as `MakeHole` are server-only
  (API docs). Backwards compatibility was the most time-consuming part.
- **Cost of the project:** more than three years and dozens of developers; Gustafsson says multiplayer was harder than
  writing the whole single-player engine (X, 2026-02). Merging the multiplayer branch took about three months after a
  year of weekly manual merges; the single-process multi-instance debugging setup was lost to the merge.
- **FAQ:** up to 12 players; the host is the server; no dedicated servers; the session ends when the host leaves (no
  migration); no cross-play, even after consoles get multiplayer (2026-09-15); multiplayer costs performance and object
  spawning is limited. Why no cross-play is not stated (unverified whether determinism is the reason).
- **Community precursor:** TDMP, a reverse-engineered mod built on function hooks (MS Detours in the repo), which
  Gustafsson called janky but impressive. The owner has hands-on experience porting Teardown content to multiplayer.
- **The new engine** (year summary 2024-12-29; 80.lv 2025-01-22; Better Software Conference 2025 talk, via a
  third-party summary): a CPU solver with substepping (6 iterations against Teardown's 8), graph colouring (10 to 12
  colours, then a serial remainder), constraints radix-sorted so they are solved in the same order every run (for
  stability), and a spinning lock-free task system (9.1x at 31 threads against 1.4x with mutexes; 5000 objects from
  82 ms to 6.6 ms). He rejects GPU physics: it competes with rendering, adds latency, and gameplay needs the results
  on the CPU at once (destruction spawns new objects). Nothing public on its multiplayer or stress analysis.

### The Finals (Embark, 2023)

GDC 2024, "Engineering Mayhem" (Måns Isaksson; slide text read):
- **Destruction pipeline:** buildings are pre-fractured in Houdini; level tools generate the connections between parts
  and mark parts resting on supportive ground.
- **Structural analysis** on top of an Embark rigid body solver: a sparse direct solver (Cholesky), made affordable
  with incremental Cholesky updates. Connection forces are split into tension or compression, shear and bending, each
  scaled by a material multiplier. A connection breaks when its impulse exceeds a *baseline* plus an offset, so a
  structure does not break under the load it was built with.
- **Networking:** the server simulates, and transforms are replicated. A large destruction event averaged about
  400 kbit/s and eventually saturated the connection. Positions are quantised within the level bounds (their example:
  0.01 units in 17 plus 16 bits), then snapshot and delta compressed after Glenn Fiedler's articles (their example
  delta costs 18 bits against 33). Result: destruction peaks at about 175 kbit/s.
- **Rendering:** a GPU transform pool for the many moving parts.
- Press coverage says destruction is entirely server-side so every player sees the same world (The Ringer 2024).
  Tick rate: not published.

### Rainbow Six Siege (Ubisoft Montreal, 2015): RealBlast

GDC 2016, "The Art of Destruction" (Julien L'Heureux; slide text read):
- **Technique:** surfaces are projected to 2D, cut against cutter polygons (Weiler-Atherton), triangulated (ear
  clipping) and extruded back. Cutter shapes come from the impact and material (ellipse, spline, Voronoi, texture).
  Collision uses simplified 2D convex pieces; bullet traces use the real surface.
- **Determinism and replication:** destruction is gameplay, so it is deterministic and replicated, preferring to
  spend CPU rather than bandwidth: events are sent, not meshes. The contract: the same inputs in the same order.
  - Same inputs is not trivial: network compression must be applied locally too (symmetric compression), and gameplay
    state races must be avoided.
  - Same order is per object and guaranteed by the network layer ("by far the easiest").
  - The RNG is seeded from the impact position (assuming inputs replicate exactly), stored thread-locally, which
    clashes with time-slicing.
  - Code must tolerate events arriving in the same frame or in different frames.
- **Local player:** shooting needs instant feedback, which breaks the ordering contract. The shooter applies its own
  events at once and may end with a slightly different surface; "in practice the difference is minimal". Rollback
  (revert and re-apply local events when the host's arrive) was considered robust but rejected: the revert stack is
  unbounded under latency and each step needs a full surface backup.
- **Debris:** physics replication is hard, so dynamic objects and debris are not replicated; they are small, ignored by
  gameplay, pre-fragmented pieces, instanced and recycled, box collision only, vaporised by explosions.
- **JIP:** "easy to do JIP with events".
- **Performance:** about 6 ms per wall; asynchronous and time-sliced, which enables "pre-destruction" (cutting ahead
  of a breach animation). One bullet hole 0.33 ms on PC and 1.1 ms on PS4; an explosion through two drywall and two
  wood layers 8.1 ms on PC and 19.5 ms on PS4.
- **Later:** 50 Hz servers in 2017, 60 Hz from patch 4.1 (2017-12); explosions separate player damage (ray casts from
  the epicentre against flagged blockers) from environment destruction (Y5S1 dev blog, 2020-02-20), so gameplay damage
  does not depend on the exact fracture.

### Battlefield: Bad Company 2 to Battlefield 6 (DICE, Frostbite)

Mostly interviews and PR; no technical talk on destruction networking was found.
- BC2 (2010): destruction data went client to server to the other clients, making bandwidth "substantially higher"
  than other shooters; everything within 25 m of a player was top priority, weighted toward the view direction (Alan
  Kertz, The Ringer, 2024-02-02).
- BF3 (2011) added indestructible cover because maps flattened and Rush defenders could not hold (IGN via MP1st).
- Tick rates: BF4 30 Hz, later up to 144 Hz in a beta with caps by player count (60 Hz at 64 players, 120 Hz at 32
  or 48, 144 Hz under 20; higher rates cost bandwidth and CPU, and ragdolls flew too far); BF1 and BFV 60 Hz on PC and
  30 Hz on console; 2042 45 Hz; BF6 60 Hz, its battle royale 30 Hz until the final circle.
- BF6 (2025): part-based destruction with health transitions that swap in or spawn assets; destruction events
  re-engineered with bandwidth optimisation and "prioritization and culling of part destruction and debris spawning"
  for 64 players (EA, 2025-11-10). Rubble stays on the map; falling debris hurts players, but not instantly (framed as
  fun over randomness).
- Frostbite 2's damage masks (SIGGRAPH 2010) are spheres placed on geometry turned into a low-resolution distance
  field: a compact damage state that could be replicated as a list of spheres (inference, not stated).

### Red Faction: Guerrilla (Volition, 2009): thin

Geo-Mod 2.0 with Havok only for throwing objects about; pre-broken meshes react to stress and impact. 16 players
online; no PC/Xbox cross-play because it was too much technically. The GDC 2010 talk on its multiplayer level design
lists technical constraints but is paywalled (viewing it would verify how destruction was networked). Red Faction:
Armageddon (2011) had 4-player co-op; its shards stay dormant until triggered (Game Developer, 2010-10-29).

### Crackdown 3 Wrecking Zone (2019): cloud destruction

Cloudgine middleware on Azure ran Havok for the match and streamed body state to the Xbox. The 2015 demo gave each
building its own server, handing collapsing buildings between servers, and used several Xbox Ones' worth of compute;
the target was the state of huge numbers of objects over 2 to 4 Mbit/s. Shipped as 5v5 on about 12 Xbox Ones' worth of
compute, with debris that fades, near-indestructible pillars and frames, chunky pre-fractured pieces and no chip
damage; the campaign (including co-op) got little destruction. Reasons: design and, by implication, server cost
(Staten 2019; Digital Foundry and press comparisons, 2019-02).

### Just Cause multiplayer mods

JC3:MP (2017): owner-controlled objects; the developers say world destruction is not synchronised, so a player who
rejoins sees the base intact. JC2-MP: per-type client sync intervals (vehicle 75 ms, on foot 120 ms, mounted gun
250 ms, passenger 1000 ms), 180 ms server broadcast, 500 m stream-out distance.

### Source engine and Garry's Mod

From the Source SDK 2013 code: `prop_physics_multiplayer` picks a physics mode automatically: props below a size
threshold become client-side only, props under a mass threshold become non-solid (pushed by players, not pushing
them). Breaking props spawn their gibs as one client-side temporary entity sent only to clients in the potentially
audible set, with gib caps and an LRU manager. Garry's Mod offers client-only physics props that are never networked.

### Minecraft (Java)

Server-authoritative blocks: single block updates, batched section updates (16³), full chunk data when a chunk comes
into view. Block breaking and placing is predicted with a sequence number that the server acknowledges, after which
the client shows the server's state (protocol wiki; the version that added sequences, believed 1.19, is unverified).

### Space Engineers (Keen, VRAGE, Havok)

GDC 2023 "Predicted Physics-Based" (Jan Hlousek; slides read by the helper) and the 2016 source snapshot:
- Authoritative server (player-hosted listen server or dedicated) in a star; clients send control input; the server
  sends delta state. The simulation is not deterministic, so predicted entities are always slightly off: corrections
  fire above a threshold and are blended exponentially.
- Destruction is decided on the server and broadcast as reliable events (create a fracture piece, create a fractured
  block, remove shapes, remove a piece with a blend time) (`MySyncDestructions.cs`). Debris is local and cosmetic, capped
  at 100 (`MyDebris.cs`).
- Rates: 60 Hz simulation; updates by distance from the client's controlled entity out to 3 km (every 4 frames near
  20 m, every 60 frames near 3 km) and by motion state (4 frames while accelerating, 20 moving steadily, 60 stopped),
  with a priority that grows with frames since the last send and a per-client multiplier.
- The local player: on the client, every body except the controlled one is static and animated from the server; only
  the controlled entity is dynamic and predicted, relative to a parent (what it stands on, or a big ship it matches in
  velocity), so corrections are sent in the parent's frame. Constrained chains (rotors, pistons) are sent as
  hierarchies relative to a root. Failures and fixes: fall-and-reset loops (send the character controller's state with
  a reset), rotor chains (do not predict them: animate), small pushable objects static on the client (never parent to
  small or light bodies), too many contacts with bodies that are static here but dynamic on the server (suspend
  prediction). Removing wheel prediction made rovers feel laggy above about 100 ms.
- Server CPU (Marek Rosa's blog, 2020): "presence" tiers slow block timers away from players; only the Havok worlds
  near players are simulated on dedicated servers. Space Engineers 2 starts with 4-player player-hosted co-op because
  it is achievable (dev diary 2025-11-13), with network, rendering and physics named as the limits.

### Besiege (Spiderling, 2017), Trailmakers, Brick Rigs, Scrap Mechanic, Stormworks, From the Depths

- **Besiege:** the host simulates all physics; clients send key inputs and receive the state of "clusters" (blocks
  merged into groups), holding collider-less copies of others' machines. A visualiser shows players how their machine
  clusters so they can build network-friendly machines. "Local simulation" lets a client run its own machine against a
  snapshot of the level, shown to others as a ghost. The block budget is global (about 500 to 700 blocks across all
  players), 8 players, host-set limits; a dedicated server was judged pointless because it needs the game's CPU anyway.
  Host lag shows as client desync. A 2024 fix: bombs that had exploded on the server broke joining clients.
- **Trailmakers:** the host computes all physics, including hundreds of flying parts in crashes; 8 players since 1.1,
  needing "a very good PC" to host; a per-vehicle complexity limit the server sets; bullet caps per player. Xbox
  multiplayer was held back because it would need dedicated servers.
- **Brick Rigs:** host-set bricks per vehicle; the server browser shows "tick rate" as the host's average FPS; patch
  notes list buildings collapsing when a client joins and desynced buildings and fuel tanks (1.1, 2021); tick functions
  sorted for more deterministic logic (1.9, 2025).
- **Scrap Mechanic:** no dedicated server because the physics is too expensive for one; lag spikes when two complex
  creations touch and merge into one solve (Dev Q&A 2021); only the host sets physics quality.
- **Stormworks:** before 2020 everyone ran at the slowest player's speed and joins or big spawns stalled everyone; the
  2020 rework moved vehicle physics to the server and let players run at their own speed (tested at 12). Physics
  objects sync as islands; a custom vehicle codec cut bandwidth 60 to 80% (2022) alongside a reworked time dilation;
  clients that fell behind got less physics to do (2023); low-bandwidth clients are dropped and the player list shows
  frames behind the server.
- **From the Depths:** host-authoritative with client extrapolation; avatar movement is client-side "so is lag free";
  missiles are synced only for a window after launch, depending on how many are in flight.

### BeamNG.drive with BeamMP

From the BeamMP client source: the server is a relay with no physics (5 to 10 Mbit/s upload recommended); each client
owns its vehicles and simulates every vehicle as a full soft body. Send rates: pose every 20 ms, inputs 30 Hz,
electrics, break groups and controllers 15 Hz, powertrain 10 Hz. A remote car chases its owner's pose with capped
forces on all its nodes (backing off when fighting contacts), dead-reckons at most 0.3 s, and teleports past
speed-scaled error thresholds. Full node and beam deformation sync exists but is commented out; only break groups
(parts coming off, glass) are sent, so dents differ between screens. Clients may swap remote cars for simplified
models, and defer others' spawns and edits until they slow down. BeamNG announced official multiplayer in 2026-05
without a date or design.

### Wreckfest and KSP multiplayer mods

- **Wreckfest (Bugbear):** server-authoritative with client prediction; 2 to 24 players set by the host's bandwidth and
  CPU. 2019 patches added adaptive prediction only for the cars nearest the player, capped prediction at 1500 ms to
  save CPU, and a dead-reckoning fallback. Whether "prediction" means re-simulation is unverified.
- **KSP LunaMultiplayer:** locks: a control lock for the pilot, an update lock taken by whoever has an uncontrolled
  vessel loaded within 20 km (so debris is simulated by the nearest player), and an unloaded-update lock. Updates every
  50 ms near others, 150 ms otherwise. Remote vessels are made immortal locally: only the owner applies damage and
  re-broadcasts the vessel.

### Networked physics in VR (Glenn Fiedler for Oculus, 2018-02)

Not a shipped game, but the most careful public design of "authority follows the toucher" for a player-hosted star:
- Each cube has an authority (default, or the last player to interact) and an ownership (held while grabbed).
  Authority spreads transitively: a thrown cube takes authority over what it hits, recursively. Objects at rest return
  to default authority.
- The host arbitrates with two sequence numbers per cube (ownership beats authority), so late packets cannot undo a
  newer action. All packets go through the host.
- Sent state: quantised position and rotation, velocities omitted for resting bodies (one bit), held cubes relative to
  the avatar. The authority quantises its own state every step exactly as it will be sent, so remote stacks stay
  stable.
- Numbers: 90 Hz physics, 10 Hz state sync, priority accumulators, 100 ms jitter buffer, under 256 kbit/s per player
  and 1 Mbit/s for the host, four players, stable stacks 20 to 30 m high.

### Noita Entangled Worlds (co-op mod for a falling-sand world, 2024-)

A Rust proxy plus a Lua mod; Steam lobbies or UDP. From the proxy's source (`noita_proxy/src/net/world.rs`, `des.rs`):
- **World pixels:** 128x128 chunks. The host arbitrates authority (a map from chunk to peer and priority). A peer asks
  for authority over chunks near it; higher-priority requests take authority over lower ones by transfer. Others
  listen: a full chunk first, then batched deltas. A peer that leaves a chunk relinquishes it with its final data, which
  the host stores and saves.
- **Entities:** authority by proximity (an R-tree and a request radius); transfer and release; a disconnecting peer's
  entities are freed.
- **Failures** (PR 509): entities marked as owned locally but never synced silently stopped updating for everyone;
  items over not-yet-generated chunks fell through, fixed by removing their gravity until terrain exists.

### Factorio (Wube): float lockstep across platforms that held

- Lockstep at 60 UPS. Peer-to-peer until 0.14, then a server (player-hosted or headless) that merges every player's
  actions for a tick into one message (FFF #147, 2016). A lagging player's actions are delayed ("skipped ticks")
  instead of stalling the map; latency is renegotiated per client (FFF #302, 2019).
- Floats with their own trig functions, because libm sin, cos, asin and atan differed between Windows, Linux and macOS
  (FFF #36, 2014). The Switch port (ARM) found undefined behaviour (out-of-range double-to-int casts) that differed
  between ARM and x86; they compared the state CRC every tick across all 2,417 tests on both architectures (FFF #370,
  2022). A staff member says only one desync ever came from floats: a compiler that evaluated float products in double
  (forum, 2017).
- Late join: the map uploads in the background while play continues; the joiner then simulates the buffered inputs
  at full speed to catch up (FFF #149). A desynced client re-downloads the map (FFF #188).
- Local player: a "latency state" rebuilt every tick from the real state plus pending local actions predicts walking,
  building and mining, but not combat (FFF #83); 2.0 added car prediction and deduplicated one-time effects (FFF #412).
- Desyncs came from hidden state not saved (FFF #63, #340), a Lua serialiser changing iteration order after load
  (#340), and chunk generation that varied with CPU core count, latent from 2017 to 2024 (FFF #415). Tools: desync
  reports with both saves, floats serialised in hex, "heavy mode" that saves and loads every tick and diffs.

### Planetary Annihilation (Uber, 2014): lockstep dropped

Forrest Smith (2013): lockstep was dropped because it runs at the slowest machine's pace (many laptops with weak
GPUs), and they wanted 40 players, join-in-progress and version-proof replays. The server simulates at 10 Hz and sends
per-property curves (linear, step, pulse) only when they change and only for units a player can see; about 1 Mbit/s
per client late in large games. Nothing needs to be deterministic. Terrain craters: the brush is sent about 30 s before
impact so clients precompute the new mesh. A bouncing ball hitched under 10 Hz interpolation until extra keyframes were
sent at impacts.

### Supreme Commander, Demigod, Forged Alliance Forever

Lockstep at 10 Hz with 200 to 300 ms command delay; the whole state hashed once per second and the game ended on a
mismatch; no join-in-progress (too many units to transfer); replays broke with patches (Forrest Smith, 2011). Floats on
x87 with the precision control pinned and asserted every tick, because some Windows API calls changed it; no IEEE
problems between Intel and AMD across more than a million players (GPG's Elijah, quoted by Fiedler, 2010). A Demigod
desync took a week: a dangling pointer to a deleted component, reused by the allocator while a unit was airborne. FAF
(the community) keeps per-beat sync logs over the checksum window and still fixes same-tick ordering desyncs in 2025.

### Spring / Recoil (Total Annihilation lineage: Zero-K, Beyond All Reason)

Lockstep with only commands sent, buffered 3 to 6 frames (100 to 200 ms) on top of ping; replays are the network
queue; saves are imperfect, so there is no mid-game join except by replaying from the start. Floats made identical by
streflop (its own libm, x87 precision control, denormal handling), and the engine restricts deterministic play to
x86-64. The docs call hunting desyncs a major drawback (Recoil netcode overview).

### Age of Empires, StarCraft II, Stormgate, Worms

- **Age of Empires (2001 article):** 200 ms turns with commands two turns ahead; the host adapts turn length to ping
  and the slowest frame rate; sending unit state would have capped a 28.8k modem at about 250 moving units; checksums
  per subsystem; players found 250 ms unnoticeable and preferred consistent to variable latency.
- **StarCraft II:** 22.4 game loops per second at Faster; deterministic given the seed; the Galaxy scripting `fixed`
  type is 20.12 fixed point (engine-wide fixed point unverified).
- **Stormgate (SnowPlay):** deterministic lockstep with rollback at 64 Hz, the simulation sandboxed from UE5
  rendering, inputs through a relay; rollback switches off when the CPU lacks headroom (patch 0.1.2, 2024-10); forced
  desync cheats to test the detector. Its relay host stopped hosting games; an offline patch followed (2026-09).
- **Worms Armageddon / Hedgewars:** player-hosted lockstep with checksums in replays and multiplayer; a decompilation
  project uses 600+ replays as a regression suite. Hedgewars uses an architecture-independent fixed-point type with
  table trig.

### Photon Quantum and its titles

C# ECS with its own deterministic 2D and 3D physics in Q48.16 fixed point (usable range about ±32,768 without overflow
checks), predict/rollback on shared inputs through a game-agnostic relay; the server substitutes inputs for missing
players after a tolerance instead of stalling. Late join from snapshots, which another client can upload ("buddy"
snapshots, chosen by the server). Titles: Stumble Guys (32 players with swing physics, per Photon), LEGO Brawls,
33 Immortals, Windblown, Bandit Trap and others. Bandit Trap (Photon blog, 2026-06-29) moved shard physics, ragdolls
and visual damage to the Unity view layer and uses pre-computed shards to save CPU; replays saved after every match
reproduce bugs every time. Common desync causes per the FAQ: writing to data assets, view code writing into the
simulation, state cached in system fields (breaks rollback), floats.

### Rocket League (Psyonix, 2015)

GDC 2018 (Jared Cone): UE3 with Bullet at a fixed 120 Hz; the server is authoritative and the client predicts
everything (its car, the ball, other cars); inputs go up tagged with frame numbers; on a large difference every
physics body rewinds and re-simulates (24 frames at 200 ms). The server buffers inputs and tells clients to run slightly
faster or slower to keep the buffer level. A tiny scene: 8 cars and a ball.

### Roblox

Legacy distributed authority: parts near a player's character are owned and simulated by that client (unverifiable,
laggy at handoffs). The 2025 server-authority mode predicts ahead, rolls back and re-simulates on misprediction, and
renders other players in the past. Its techniques page describes deterministic ids for client-spawned objects (type,
source, frame, per-frame counter) so predicted spawns match the server's. That Roblox ships the AVBD solver is
unverified.

### Unity DOTS, Unreal networked physics, 0 A.D., engine-level notes

- **Unity Physics** is stateless (no warm starting) to make rollback simple, deterministic per platform with Burst's
  deterministic float mode; Netcode for Entities runs interpolated physics bodies kinematic on clients, predicted ones
  several steps per frame, and a separate client-only physics world for effects. No verified physics-heavy title.
- **Unreal (5.8 docs):** three replication modes: velocity matching, predictive interpolation, and resimulation (cache
  state per tick for the round trip, rewind and re-simulate when the server disagrees beyond a threshold; costs CPU and
  memory).
- **0 A.D.:** simulation in 15.16 fixed point; float operators on the fixed type are deliberately undefined.
- **Box2D v3** (2024-08-27): cross-platform determinism verified on MSVC, GCC and Clang, x64 and ARM, with no FMA
  contraction, no fast-math, its own trig and bitset merges; rollback determinism explicitly not provided (internal
  solver state). **fixed3d** (2026-09-07): Box3D in Q48.16 runs 2.24x slower than float (1.71x to 3.21x per
  benchmark); its authors say "probably not" for most projects.

### Friendslop co-op: Lethal Company, R.E.P.O., Content Warning, PEAK, Phasmophobia

These are the closest in genre to the first-person co-op game in destructible buildings. None is deterministic; all are host- or
owner-authoritative Unity games with small caps (4 to 6), and mods that raise the cap to 20 to 50 break UI and sync
above about 8. Much of the detail comes from third-party decompiled code on GitHub (versions uncertain), so treat
exact constants as indicative.
- **Lethal Company** (Zeekerss, 2023; Netcode for GameObjects over Steam): each player owns its movement and sends a
  reliable position update to the host whenever it has moved about 0.5 m (tighter near other players), relative to its
  parent (the ship or the elevator); the host re-broadcasts it and remote copies ease toward it. Look direction goes as
  two shorts, at most every 0.1 s, only if it changed by 3 degrees and someone is within 35 m. **Scrap has no physics:**
  the host validates a grab (first request wins) and hands ownership to the grabber; a drop sends a floor target found
  by a ray cast plus a yaw, and every client animates the fall along a curve. Enemies are simulated by their owner
  (the host by default). No late join in the base game; a mod that added it had to forbid joining while landed because
  of desyncs.
- **R.E.P.O.** (semiwork, 2025; Photon PUN 2, 6 players, 25 Hz send and serialisation rates): the master client
  simulates every physics prop; other clients hold kinematic copies. A grabbing client streams only its *intent* (grab
  point, plane point, rotation input) and the host sums spring forces from every grabber, which is how several players
  carry one object. Velocities are clamped (40 m/s, 30 rad/s). **Breakage is judged on the host** from impact tiers,
  scaled by mass and fragility, with cooldowns, and sent to everyone as events; objects are unbreakable for 5 s after
  spawning and briefly after their first grab, and the cart protects its contents. Players stream their pose and the id
  of the physics object they ride with their local position on it. The cost is a full round trip on every grab and
  throw, which the developers say they struggled with for a long time; community mods add Hermite interpolation or
  fake local ownership. The host leaving ends the session. It sold millions of copies regardless (Photon, 2025-04-15).
- **Content Warning** (Landfall, 2024; PUN): class names suggest a puppet design (send the root, simulate the active
  ragdoll locally; inferred). Landfall pays for the Photon relay traffic and asked modders to keep traffic low; a mod
  framework routes mod traffic over Steam instead.
- **PEAK** (Aggro Crab and Landfall, 2025; PUN over Photon relays, 4 players): the owner sends its hip position, look
  as half floats, movement input as one byte of flags and the ragdoll's average velocity; every client simulates every
  ragdoll with the replayed input and nudges each body part about 10% toward the interpolated hip per step, teleporting
  past 10 m. Items send pose and half-float velocities only while awake, on the ground and non-kinematic, force a
  10-frame burst after events, and are corrected half-way on receipt. A held item's authority moves to the holder, who
  keeps it after throwing.
- **Phasmophobia** (Kinetic Games, 2020; PUN): the toucher requests ownership and only the owner's body is dynamic;
  the master reclaims doors on release; the ghost takes ownership of a prop before throwing it.
- **Jank tolerance:** direct evidence is thin. R.E.P.O.'s physics were praised as the thing Lethal Company lacked,
  despite the host round trip; Aggro Crab embraces the "friendslop" label (GamesRadar+, 2025-11-01); Human: Fall Flat
  scoped its online play to friends on good connections.

### Physics characters: Human: Fall Flat and Gang Beasts

- **Human: Fall Flat** (No Brakes, online since 2017-10, 8 players): the host simulates; every movable rigid body
  streams its state and game logic passes through networked signal nodes (events). The developers ask for the best PC
  (steady 60 fps) to host with under 100 ms ping; climbing gets harder at higher ping, suggesting clients do not predict
  their own character (inferred). The creator had first thought online impossible for its physics.
- **Gang Beasts** (Boneloaf, 2016): server-authoritative physics with client input, on official AWS servers and
  player-run dedicated servers; early patches fixed frame-rate-dependent forces and "online syncing issues" that
  destabilised physics, and idle players were kicked to save server CPU.

### Survival and building: Valheim, Deep Rock Galactic, 7 Days to Die, Minecraft-likes

- **Valheim** (Iron Gate, 2021): distributed authority. The server stores and relays; each zone's objects are simulated
  by an owner client, and ownership is sticky: a peer claims objects that are unowned or whose owner is out of range,
  and releases what leaves its own area. Objects carry owner and data revisions and each peer's last-seen revision, so
  only changes are sent: 20 Hz, one peer per frame round robin, within a send-queue window of about 10 KB, sorted by
  distance minus staleness; the distant area is added only when the queue is nearly empty. **Structural integrity** is
  computed by the owner and stored in the object's data; outside the active area support is pinned to the maximum, so
  far structures never collapse; destruction sends one event and each client spawns its own cosmetic fragments. A weak
  client that owns a zone makes everyone rubber-band.
- **Deep Rock Galactic** (Ghost Ship, UE4, 4 players): terrain edits (pickaxe, drill, melt, explode, carve, remove
  floating islands, spawn debris) are numbered operations sent on a reliable multicast with their parameters (hit
  position, direction, size); a late joiner receives the full operation history and replays it over the seeded cave
  (from header dumps, 2024). The same pattern as Teardown and Siege, in shipped co-op.
- **7 Days to Die:** an explosion is resolved once into a list of block changes; falling-block entities spawn three
  frames later and are thinned to a fraction of the count (roughly count / (20 + count^0.73)); only half drop items.
- **Terraria:** the server owns NPCs, the player who fires a projectile owns it, and state is sent only when flagged
  as changed. **Sons of the Forest:** cut the cost of processing structures on join twice in 2023; fixed trees not
  falling in multiplayer after explosives. **Enshrouded, Astroneer:** nothing found on terrain sync.

## Questions across games

### 1. Destruction as commands, as body state, or as re-simulation, and what each gave up

| approach | who | what they gave up |
|---|---|---|
| destruction as ordered input events, applied deterministically on every peer | Rainbow Six Siege, Teardown, Deep Rock Galactic (terrain ops); lockstep RTS (everything); Planetary Annihilation's craters (pre-sent brush) | a bit-exact destruction kernel (Siege: seeded RNG, symmetric quantisation; Teardown: a fixed-point rewrite); a reliable ordered channel; late join by log replay, capped (Teardown); the shooter's instant feedback breaks the order (Siege accepts tiny divergence) |
| destruction as decided results (events carrying the outcome) | Space Engineers (fracture pieces), BeamMP (break groups), Battlefield (part health transitions), R.E.P.O. (host-judged breakage events), Valheim (owner-computed support, fragments spawned per client), 7 Days to Die (block-change lists), LunaMultiplayer (owner re-broadcasts the vessel) | nothing needs to be deterministic, but only coarse or pre-authored outcomes fit in a message; fine damage differs between screens (BeamMP dents) |
| body state of the debris streamed | The Finals, Crackdown 3, Trailmakers, Besiege, Human: Fall Flat (every rigid body), R.E.P.O. (props) | bandwidth (The Finals: ~400 kbit/s per client raw, ~175 kbit/s after compression, for destruction alone; Crackdown: 2 to 4 Mbit/s and a server farm); snapping when starved; host CPU |
| re-simulation (rollback) | Rocket League (all bodies), Quantum titles, Roblox 2025, Unreal resimulation, Stormgate; Siege considered it for surfaces | CPU and memory proportional to latency times scene size; Siege rejected it (unbounded revert stack, a full surface backup per step); Stormgate turns it off without CPU headroom; Box2D and Box3D do not support it (hidden solver state) |
| not synced at all | JC3:MP world destruction; Siege and Source debris | consistency: a rejoining player sees intact buildings (JC3:MP) |

### 2. How player-hosted titles coped with host upstream and host CPU

- **Fixed per-client budgets:** Teardown about 1 Mbit/s per client (so up to about 11 Mbit/s upstream for a full
  12-player host; the actual use is not published); Fiedler's VR demo 256 kbit/s per player and 1 Mbit/s for the host.
  BeamMP's relay needs 5 to 10 Mbit/s upload. For contrast, The Finals' dedicated servers peak at 175 kbit/s per client
  for destruction.
- **Prioritisation:** by visibility (Teardown), by distance and motion (Space Engineers), by a 25 m bubble and view
  direction (BC2), culling debris spawns (BF6).
- **Content budgets and player caps:** Besiege's global block budget and 8 players, Trailmakers' complexity limits and
  8 players, Brick Rigs' bricks per vehicle, Space Engineers' block limits (PCU), Teardown's object spawning limit and
  12 players, Space Engineers 2 starting at 4.
- **Host CPU:** clients do little physics (Besiege collider-less copies; Stormworks moved physics off slow clients);
  simulation only near players (Space Engineers); time dilation with a visible server speed (Space Engineers,
  Stormworks, Brick Rigs); no dedicated servers because they would need the game's own CPU (Besiege, Scrap Mechanic).
- **Move the fan-out to a paid relay:** the Photon titles (R.E.P.O., Content Warning, PEAK, Phasmophobia) send through
  Photon's servers, so the host's upstream matters less and the studio pays (Landfall asked modders to keep traffic
  low). The host still carries the physics CPU in R.E.P.O.
- **Send on change, skip sleepers, spread ownership:** Lethal Company sends positions only past a distance threshold;
  PEAK sends nothing for sleeping or held items; Valheim sends revisions within a 10 KB window at 20 Hz and spreads the
  simulation over zone owners; Human: Fall Flat asks the strongest PC to host.
- **Lockstep sidesteps upstream entirely:** Factorio's host merges inputs into one message per tick; the cost moves to
  every peer's CPU and the slowest peer (Stormworks' old model and Planetary Annihilation left lockstep for that reason).

### 3. Fixed point, and float lockstep across platforms

- **Fixed point:** Teardown (destruction only, because voxels are discrete), Photon Quantum (C#, where float code
  generation cannot be controlled), 0 A.D. (15.16), Hedgewars, StarCraft II's scripting type, fixed3d (huge worlds).
  Cost: fixed3d is 2.24x slower than float Box3D; Quantum limits the usable range.
- **Float lockstep across platforms:** Factorio (Windows, Linux, macOS, Switch ARM, Apple silicon) held, with its own
  trig, undefined-behaviour fixes and per-tick CRC comparisons of its test suite across ISAs; one float desync ever,
  from a compiler promoting to double. GPG's games held across Intel and AMD on x87 with the control word pinned.
  Spring holds on x86-64 only (streflop). Box2D v3 is verified across MSVC, GCC, Clang, x64 and ARM.
- **Where it broke:** x87 transcendentals (Age of Empires, Battlezone 2), libm (Factorio), compiler promotion
  (Factorio), undefined behaviour (Factorio on ARM), and LittleBigPlanet under the RPCS3 emulator (a commenter blames
  float precision differences between CPUs; indirect evidence, unverified).

### 4. Desync post-mortems in physics-heavy lockstep

Few physics-heavy lockstep games document desyncs; the root causes in the lockstep titles above are almost never float
hardware: hidden state not saved or restored (Factorio), iteration order changed by serialisation (Factorio), results
that depend on thread count (Factorio, latent seven years), a dangling pointer plus allocator reuse (Demigod),
same-tick event ordering (FAF), the x87 control word changed by OS calls (GPG), middleware running fewer solver
iterations on faster CPUs (GPG, via Fiedler), undefined behaviour and libm (Factorio), memory corruption and bugs in the
detector itself (Stormgate). Stormworks' early slowest-player sync is the physics-heavy cautionary tale: joins and big
spawns stalled everyone, and it was replaced with server authority.

### 5. Small tricks

See the list below: seeded fracture from the impact position, symmetric quantisation, event-log JIP with a cap,
client-side gibs, pre-fractured parts, damage states, at-rest bits, relevance by visibility and distance, authority by
proximity or touch, delayed debris damage, and more.

## Strategies big and small

Each is tagged with the model it serves (LS lockstep, HY hybrid of deterministic destruction commands and state-synced
bodies, SS state sync, any), what it gains, what it costs, and its fit for us (a 4 to 12 player player-hosted co-op
destruction engine with a deterministic core, float Box3D, stress-driven breaks, vehicles, cranes and rigs).

### Architecture

1. **Destruction as host-ordered commands, applied deterministically everywhere** (Teardown, Siege). HY/LS. Gain:
   kilobytes instead of streaming thousands of fragments; identical rubble geometry on every screen. Cost: the fracture
   kernel must be bit-exact across machines, and commands need a reliable ordered channel. Fit: high; our fracture
   (lpFractureInput to cells and bonds) is already a pure function. In a hybrid our stress judgements depend on
   float body state that differs per client, so the host must send the *decisions* (these bonds break, this piece
   fractures with this seed), as the roadmap already suspects (docs/roadmap.md:276-278).
2. **Bodies as prioritised, eventually consistent state** (Teardown, Space Engineers, Fiedler). HY/SS. Gain: Box3D
   and a GPU may stay non-deterministic across machines; no slowest-peer stall. Cost: snapping under load; host upstream
   (about 1 Mbit/s per client in Teardown); stress loads and links read float state that now differs per machine.
   Fit: medium-high for 8 to 12 players; the proven 12-player player-hosted design.
3. **Full lockstep with local-player prediction outside the simulation** (Factorio's latency state, Quantum). LS.
   Gain: tiny upstream; every system (stress, supply, rigs) stays consistent for free; replays and save games fall out.
   Cost: every peer simulates everything (a toaster peer slows all: Stormworks, Planetary Annihilation left for this);
   input delay for anything not predicted; cross-machine bit-exactness of Box3D and us. Fit: high for 4 players, open at
   12; no shipped 3D rigid-body destruction game proves it.
4. **Predict and roll back** (Rocket League, Quantum, Roblox 2025). RB. Gain: zero-latency feel. Cost: re-simulation
   of the whole scene per correction; Box3D has no rollback determinism; Siege rejected it for surfaces. Fit: low for
   the world; possible for a local character alone.
5. **Distributed authority by region or touch** (Noita EW chunks, Roblox ownership, LunaMultiplayer update locks,
   Fiedler's authority spreading through contacts). DA. Gain: spreads CPU and upstream across peers, hides latency for
   what each player touches. Cost: arbitration, flicker at handoffs, no global consistency for chains of contacts;
   stress structures spanning two owners have no single truth. Fit: low for destruction, medium for props carried by
   players in a hybrid.
6. **A replayable command log plus world snapshots** (Teardown, Factorio, Quantum, Siege). Any. Gain: late join,
   desync repair, save games, replays, bug reproduction. Cost: the log grows with destruction (Teardown caps it and
   refuses JIP); a snapshot of Box3D's hidden state. Fit: essential under every model.

### Destruction

7. **Seed fracture from the impact position** (Siege). HY/LS. Gain: no RNG state to replicate. Cost: needs bit-exact
   inputs. Fit: high; our PCG32 seeds from world seed, tick and piece already (determinism rule 3).
8. **Symmetric quantisation** (Siege; Fiedler quantises the authority's own state every step). HY/SS. Gain: the host
   fractures from the same bits the clients receive, so the host's local result cannot drift. Cost: slight precision
   loss. Fit: high: quantise impact points, directions and energies in the command before applying it on the host.
9. **Fixed point only for the destruction geometry kernel** (Teardown). HY. Gain: cross-machine agreement where it
   matters, far smaller than all of physics. Cost: a rewrite of poly.c/fracture.c arithmetic. Fit: medium; our fracture
   calls Box3D's quickhull and is float; if float is made bit-exact with pinned flags (H1), this is not needed.
10. **Pre-fractured and part-based destruction with health states** (Battlefield, Crackdown 3 shipped, The Finals,
    Red Faction, Siege's dynamic objects). SS/HY. Gain: the replicated state is a part id and a small health or state.
    Cost: less freedom. Fit: medium; our procedural fracture is the point, but bond health and a per-structure bond
    state vector are already this kind of compact state.
11. **A break baseline** (The Finals): break when the impulse exceeds the load the structure was built with plus an
    offset. Any. Gain: structures stand at load time without tuning. Fit: we already settle structures at load
    (`lpBuildScene`); worth comparing with our utilisation thresholds.
12. **Decide gameplay damage apart from the fracture outcome** (Siege's ray-cast damage, BF6's delayed debris damage).
    Any. Gain: small divergence in rubble never decides who lives. Fit: high for a hybrid.
13. **Time-sliced destruction with pre-destruction** (Siege). Any. Gain: spreads the 20 to 90 ms fracture spikes;
    cuts can start before a charge blows. Cost: latency. Fit: medium; our caps must be counts, not time (house rule).
14. **Send terrain changes early** (Planetary Annihilation sends crater brushes 30 s ahead). Any. Fit: for scripted or
    timed detonators, send the command when armed, apply at the tick.
15. **Indestructible cores** (BF3 cover, Crackdown pillars). Any. Gain: bounds both balance and simulation. Fit: a
    level-design option.

### Debris and the cosmetic layer

16. **Debris never replicated, ignored by gameplay** (Siege, Source gibs, Space Engineers, Bandit Trap on Quantum). Any.
    Gain: removes the largest body count from the network and from determinism. Cost: debris cannot matter to play.
    Fit: high for the smallest tiers; our ghosts and scrap could move to a per-machine cosmetic layer in a hybrid.
17. **Tiny props become client-only, light props non-solid** (Source's automatic physics modes). Any. Fit: a rule for
    our debris ladder.
18. **Caps, fades, LRU and vaporise-on-explosion for debris** (Source, Siege, Space Engineers cap 100, Crackdown fades).
    Any. Fit: already our budget ladder; count-based.
19. **A separate client-only physics world for effects** (Netcode for Entities; Bandit Trap's view-layer shards). HY/LS.
    Fit: matches rule 9 of our determinism rules (dust outside the simulation); extend it to far debris.

### Bandwidth and relevance

20. **Priority queue per client within a fixed bitrate, favouring what that player sees** (Teardown, Fiedler's
    priority accumulator). SS/HY. Fit: high for a hybrid; a count-based cap per tick keeps it deterministic on the host.
21. **Rates by distance and motion; at-rest bodies send one bit** (Space Engineers 4/20/60 frames; Fiedler). SS. Fit:
    high; our sleeping and frozen rubble need no traffic at all.
22. **Quantise within level bounds and delta-compress against a baseline** (The Finals: 33 to 18 bits per position).
    SS. Fit: high for body streaming.
23. **Group bodies into islands or clusters** (Stormworks islands, Besiege clusters). SS. Gain: one transform per
    welded group. Fit: high; our structures and vehicles are natural clusters.
24. **Stream-out distance and per-type send intervals** (JC2-MP: 500 m; vehicles 75 ms, passengers 1 s). SS. Fit: for
    large maps (milestone 12).
25. **Relevance by player proximity** (BC2's 25 m bubble, Minecraft chunks, Noita chunks). Any. Fit: pairs with our
    active cells around observers.

### The local player

26. **Only the controlled body is dynamic on the client; the rest are kinematic from the server** (Space Engineers).
    SS/HY. Gain: no fights between prediction and a remote pile. Fit: high for a character or a driven truck in a hybrid.
27. **Predict relative to what you stand on** (Space Engineers parents to the ship underfoot). SS/HY. Fit: high for
    players riding trucks, cranes and the mech.
28. **Do not predict joint chains** (Space Engineers, rotors and pistons). SS/HY. Fit: cranes, winches and rigs should
    be animated from the host, not predicted.
29. **A latency state rebuilt each tick from the true state plus pending inputs** (Factorio). LS. Gain: prediction
    without rollback. Fit: high for a lockstep local character; the character's effects enter as tick-stamped commands.
30. **Apply your own destructive shots at once and accept small divergence** (Siege). HY. Fit: high for tools that cut
    or blast; send the command and show local effects immediately, correct from the host's result.
31. **Input delay only above a ping threshold** (Quantum). LS/RB. Fit: for lockstep over relays.

### Ownership and authority

32. **Authority follows the toucher, spreads through contacts, returns to default at rest; the host arbitrates with
    sequence numbers** (Fiedler). DA. Fit: carried props in a friendslop game.
33. **Damage authority stays with the owner** (LunaMultiplayer's immortal remote vessels, BeamMP break groups). DA.
    Fit: low for us (the host should own destruction).
34. **Host-arbitrated chunk authority with relinquish-to-host storage** (Noita EW). DA. Fit: a pattern for far regions
    in a large map, if ever needed.
35. **Deterministic ids for client-spawned objects** (Roblox: type, source, frame, counter). Any. Fit: high: pieces
    spawned by fracture need ids that match across peers; ours come from deterministic creation order.

### Joining, recovery and operations

36. **JIP by replaying the destruction log on the pristine level, capped** (Teardown, Siege). HY. Fit: high; simpler
    than a snapshot, but its cost grows with play.
37. **Background snapshot upload plus buffered-input catch-up** (Factorio); **snapshots from a peer** (Quantum buddy).
    LS. Fit: high for lockstep; needs world serialisation.
38. **A desynced peer alone re-downloads the state** (Factorio). LS. Fit: high; the recovery path under lockstep.
39. **Skip a lagging player's inputs instead of stalling** (Factorio, Quantum hard tolerance). LS. Fit: high.
40. **Show server speed and frames behind; drop peers that cannot keep up** (Stormworks, Brick Rigs, Space Engineers).
    Any. Fit: cheap honesty.
41. **Desync tooling:** per-tick CRC across ISAs over the whole test suite, a save/load-every-tick mode, desync reports
    with both states and floats in hex (Factorio); forced-desync cheats to test the detector (Stormgate); sync logs over
    the checksum window (FAF); replays saved from every session (Bandit Trap). LS/HY. Fit: high; extends
    `tools/check-determinism.ps1` to two machines.
42. **Pin and assert the FPU state every tick** (GPG). LS. Fit: cheap; assert MXCSR (rounding, flush-to-zero) per tick.
43. **Do not depend on one relay operator** (Stormgate lost its host). Any. Fit: Steam relays are a dependency; a
    direct-connect fallback is cheap insurance.

### From the friendslop and survival games

44. **Streamed grab intent, host-side spring forces** (R.E.P.O.). SS/HY. Gain: several players carry one object and
    the physics stays on one machine. Cost: a round trip before a grab responds. Fit: high for co-op hauling
    (the hauling game); in lockstep the intent is simply a tick-stamped input, and our pulls already work this way (`lpWorld_Pull`, include/lpf/lpf.h:586).
45. **Breakage judged by one authority, with grace periods** (R.E.P.O.: unbreakable for 5 s after spawn and briefly
    after a grab, a safe cart). HY/SS. Gain: no double breaks, fewer unfair ones at spawn and hand-over. Fit: high as
    game policy on top of our impacts.
46. **Carried items without physics: animate the fall to a ray-cast target** (Lethal Company). SS. Gain: nothing to
    sync but a target and a yaw. Cost: no tumbling. Fit: a fallback for trivial props only.
47. **Puppet ragdolls: send the hip, input bits and average velocity; every client simulates the body and nudges it
    toward the target** (PEAK; Content Warning likely). SS. Fit: medium for remote player bodies in a hybrid (our rigs
    would be simulated from replayed inputs; corrections stay soft).
48. **Send only awake bodies; burst for a few frames after an event; correct half-way** (PEAK). SS. Fit: high.
49. **Per-peer revision deltas within a send window, sorted by distance and staleness** (Valheim). SS. Fit: high for
    slow-changing state (bond health, supply, pools).
50. **Send-on-change thresholds that tighten near other players; parent-relative positions** (Lethal Company; R.E.P.O.
    sends the id of the body a player rides). SS. Fit: high for characters on vehicles.
51. **Holder becomes owner and keeps it after a throw; first grab wins, validated by the host; AI takes ownership
    before acting on a prop** (PEAK, Lethal Company, Phasmophobia). DA. Fit: for carried props in a hybrid.
52. **Far structures never collapse** (Valheim pins support to the maximum outside the active area). Any. Gain: no
    simulation where no one is. Cost: conflicts with our goal that a trap set far away still goes off (docs/goals.md);
    use it only for structures with nothing armed near them.
53. **Thin falling debris by a count formula and delay it a few frames** (7 Days to Die). Any. Fit: high; a count cap
    in our debris ladder, deterministic when taken in index order.
54. **A numbered terrain-operation log replayed for late joiners** (Deep Rock Galactic). HY. Fit: high; confirms 36.
55. **Allow late join only in a safe phase** (a Lethal Company mod after desyncs while landed). Any. Fit: cheap policy
    for a first multiplayer build (join between missions or races).

## Verdicts on the hypotheses this track touches

- **H1 (Box3D float bit-exact across compilers and ISAs; hazards are ours):** supported, not proven for Box3D v0.1.
  Box2D v3 is verified across MSVC, GCC, Clang, x64 and ARM; fixed3d's authors say Box3D is deterministic across
  platforms; Factorio's cross-platform float lockstep held and every documented failure was the game's own code
  (undefined behaviour, hidden state, thread-count dependence, libm). Spring restricts itself to x86-64, a cautionary
  data point.
- **H2 (lockstep right for 4p, acceptable at 12p; hybrid loses on host upstream at 12p):** partly falsified. Teardown
  ships the hybrid at 12 players on player hosts within about 1 Mbit/s per client, paying with visible snapping, not
  with failure. No shipped 3D rigid-body destruction game runs lockstep; its risks in practice are the slowest peer and
  late join (Stormworks and Planetary Annihilation moved away; Supreme Commander could not join mid-game). The
  local-player clause is supported: Factorio predicts the local player outside the deterministic state, and Siege
  applies the shooter's own events at once.
- **H3 (fixed point everywhere costs 2 to 4x, forfeits Box3D, solves what flags solve):** confirmed at the low end:
  fixed3d measures 2.24x (1.71 to 3.21x) and its authors advise against it. Teardown's fixed point covers only a discrete
  destruction kernel, consistent with "exact arithmetic for fracture geometry may be worth it anyway".
- **H4 (GPU off the deterministic path; cosmetic layer only):** supported. Gustafsson keeps physics on the CPU (render
  contention, latency, gameplay needs results at once); shipped games keep debris outside the replicated simulation
  (Siege, Source, Space Engineers, Bandit Trap, Netcode for Entities).
- **H5 (a serialisable, command-logged world with hashes matters most):** strongly supported. Every model in this
  survey leaned on a log or a snapshot for late join, repair or replays (Teardown, Siege and Deep Rock Galactic replay
  destruction logs; Factorio and Quantum transfer snapshots), and those without one could not join mid-game (Supreme
  Commander, Spring) or shipped late-join destruction bugs (Brick Rigs, Besiege, Lethal Company mods).
- **H6 (x64 under Rosetta, Prism, box64, FEX stays bit-exact):** not tested by any game found; the only emulator case
  (LittleBigPlanet under RPCS3, a PS3 emulator with a different CPU model) desynced, which says little about x64 SSE2
  translation. Open.

## Sources

Dates are publication dates; "n.d." = undated page, read 2026-09-29.

**Teardown and Gustafsson**
- The unlikely story of Teardown Multiplayer, 2026-03-13: https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html
- 80.lv interview, 2026-03-17: https://80.lv/articles/teardown-developer-breaks-down-multiplayer-and-voxel-destruction-tech
- Teardown multiplayer FAQ, n.d.: https://teardowngame.com/faq.html
- Teardown modding API, n.d.: https://www.teardowngame.com/modding/api.html
- Steam announcement, 2026-03-12: https://store.steampowered.com/news/app/1167630/view/1826992588592355
- Gustafsson on X, 2026-02: https://x.com/voxagonlabs/status/2027632578958623140
- TDMP repo: https://github.com/TDMP-Team/TDMP-0.2.2
- Year summary (new engine), 2024-12-29: https://blog.voxagon.se/2024/12/29/year-summary.html
- 80.lv on the new physics engine, 2025-01-22: https://80.lv/articles/see-what-s-new-in-teardown-creator-s-custom-voxel-physics-engine
- BSC 2025 talk "Parallelizing the physics solver": https://www.youtube.com/watch?v=Kvsvd67XUKw (read via the
  third-party summary https://lilys.ai/en/notes/1113395; watching the talk would verify the figures)

**The Finals**
- GDC 2024 "Engineering Mayhem" slides (Isaksson): https://media.gdcvault.com/gdc2024/Slides/GDC+slide+presentations/Isaksson_Mans_Engineering+Mayham+Technical.pdf
- Session page: https://gdcvault.com/play/1034307/Engineering-Mayhem-Technical-Deep-Dive
- The Ringer on destruction, 2024-02-02: https://www.theringer.com/2024/02/02/video-games/destruction-video-games-battlefield-bad-company-red-faction-battlebit-teardown-the-finals

**Rainbow Six Siege**
- GDC 2016 "The Art of Destruction" slides: https://media.gdcvault.com/gdc2016/Presentations/LHeureux_Julien_Art_Of_Destruction.pdf
- 60 Hz servers, 2017-12-18: https://www.gamereactor.eu/rainbow-six-siege-finally-gets-60hz-servers/
- Blood Orchid tick rate, 2017-09: https://www.pcgamesn.com/rainbow-six-siege/rainbow-six-siege-blood-orchid-tick-rate
- Explosions and shrapnel (Y5S1), 2020-02-20: https://www.ubisoft.com/en-us/game/rainbow-six/siege/news-updates/1QkezaGoRkDWqcQ6duGvtk/dev-blog-explosions-shrapnel-in-y5s1

**Battlefield**
- How Battlefield 6 redefined destruction (EA), 2025-11-10: https://www.ea.com/news/how-battlefield-6-redefined-destruction
- Battlefield Labs destruction, 2025: https://www.ea.com/games/battlefield/news/battlefield-labs-destruction
- BF4 high tickrate beta (DICE PDF), c. 2015: https://eaassets-a.akamaihd.net/dice-commerce/battlefield4/assets/patch_notes/HighTickrat_BETA_R1_R4_DS.pdf
- BF3 less destruction, 2011-09-05: https://mp1st.com/news/less-destructible-environments-in-battlefield-3-dice-explains
- BFV tick rates, 2018-10-23: https://www.resetera.com/threads/battlefield-v-tick-rate-confirmed-30-hz-on-console-60-hz-on-pc.76569/
- BF6 60 Hz, 2025-10: https://mp1st.com/news/battlefield-6-server-tick-rate-60hz-enemy-visibility-improved-per-map
- BF6 debris damage, 2025-08-11: https://gamerant.com/battlefield-6-how-destruction-damage-works/
- Frostbite 2 destruction masking, SIGGRAPH 2010: https://www.slideshare.net/slideshow/siggraph10-arrdestruction-maskinginfrostbite2/4883521

**Red Faction, Crackdown 3, Just Cause, Instruments of Destruction**
- RFG interview, 2009-04-24: https://www.gamewatcher.com/interviews/red-faction-guerrilla-interview/11408
- RFG PC interview, 2009-09-14: https://worthplaying.com/article/2009/9/14/interviews/68376-red-faction-guerrilla-pc-developer-interview/
- RFG multiplayer level design, GDC 2010 (paywalled): https://gdcvault.com/play/1012330/Multiplayer-Level-Design-in-Red
- Red Faction: Armageddon destruction, 2010-10-29: https://www.gamedeveloper.com/design/the-destructible-world-building-i-red-faction-armageddon-i-
- Cloudgine interviews, 2015-10: https://gamingbolt.com/crackdown-3s-cloud-based-destruction-enhancement-detailed-was-extremely-challenging-to-achieve and https://gamingbolt.com/cloudgine-interview-crackdown-3-and-clouds-of-destruction
- Crackdown 3 demo servers, 2015-08-06: https://gameinformer.com/games/crackdown/b/xboxone/archive/2015/08/06/we-ruined-a-cloud-city-in-crackdown-3s-multiplayer.aspx
- Crackdown 3 per-building servers, 2015-08-11: https://www.criticalhit.net/gaming/gamescom-2015how-crackdown-3-is-using-the-cloud-properly-and-why-you-might-not-need-it/
- Crackdown 3 shipped scale (Staten), 2019-02-01: https://wccftech.com/crackdown-3-cloud-12-xbox-one/
- Crackdown 3 downgrade, 2019-02-12: https://www.dsogaming.com/news/crackdown-3-destruction-has-been-severely-downgraded-comparison-video-between-2015-and-2019-builds/
- JC3:MP destruction not synced, 2017-07-13: https://steamcommunity.com/app/619910/discussions/0/1457328927827817919/
- JC2-MP server configuration: https://pingperfect.com/knowledgebase/541/Just-Cause-2-Multiplayer-JC2--Server-Configuration.html
- Instruments of Destruction (single-player only): https://en.wikipedia.org/wiki/Instruments_of_Destruction

**Source, Garry's Mod, Minecraft**
- Source SDK 2013 (props.cpp, props_shared.cpp): https://github.com/ValveSoftware/source-sdk-2013
- Garry's Mod client props: https://wiki.facepunch.com/gmod/ents.CreateClientProp
- Minecraft Java protocol: https://minecraft.wiki/w/Java_Edition_protocol/Packets

**Space Engineers**
- GDC 2023 "Predicted Physics-Based" slides (Hlousek): https://media.gdcvault.com/gdc2023/Slides/Predicted+Physics+Based_Hlousek_Jan.pdf
- Source snapshot, 2016: https://github.com/KeenSoftwareHouse/SpaceEngineers (MySyncDestructions.cs, MyEntityPhysicsStateGroup.cs, MyDebris.cs)
- Multiplayer overhaul, 2018-07-19: https://blog.marekrosa.org/2018/07/space-engineers-multiplayer-overhaul/
- Server optimisations, 2020-08-06: https://blog.marekrosa.org/2020/08/space-engineers-server-optimizations/
- Space Engineers 2 dev diary, 2025-11-13: https://blog.marekrosa.org/2025/11/mareks-dev-diary-november-13-2025/
- Space Engineers 2 roadmap: https://2.spaceengineersgame.com/roadmap-2/

**Besiege, Trailmakers, Brick Rigs, Scrap Mechanic, Stormworks, From the Depths**
- Besiege Dev Blog 1, 2017-05-19: https://store.steampowered.com/news/app/346010/view/1904306376092524656
- Besiege Dev Blog 4, 2017-11-10: https://store.steampowered.com/news/app/346010/view/2143012757614964117
- Besiege V1.26, 2024-01-22: https://store.steampowered.com/news/app/346010/view/5510784213113219352
- Besiege mod docs: http://mod.besiege.co.uk/articles/APIs.html
- Trailmakers Dev Update #8, 2017-09-28: https://store.steampowered.com/news/app/585420/view/2175660043464336842
- Trailmakers 1.1 (8 players), 2020-07-01: https://store.steampowered.com/news/app/585420/view/3444639924515774646
- Trailmakers Xbox multiplayer, 2019-03-22: https://store.steampowered.com/news/app/585420/view/2543878145175732951
- Brick Rigs 1.1, 2021-11-18: https://store.steampowered.com/news/app/552100/view/4227186657141906854
- Brick Rigs 1.9, 2025-10-17: https://store.steampowered.com/news/app/552100/view/1813674388967211
- Scrap Mechanic Dev Q&A, 2021-06-12: https://store.steampowered.com/news/app/387990/view/5019805792168405658
- Stormworks new multiplayer, 2020-01-24: https://store.steampowered.com/news/app/573090/view/2597949245051514096
- Stormworks v1.4.15, 2022-04-08: https://store.steampowered.com/news/app/573090/view/4353300134229105095
- Stormworks v1.6.11, 2023-01-06: https://store.steampowered.com/news/app/573090/view/5035620387692051037
- Stormworks v1.4.9, 2022-02-25: https://store.steampowered.com/news/app/573090/view/4246335832512391266
- From the Depths, 2015-01-29: https://store.steampowered.com/news/app/268650/view/510365936292366969
- From the Depths, 2016-04-01: https://store.steampowered.com/news/app/268650/view/260454846482744410

**BeamMP, Wreckfest, KSP**
- BeamMP client: https://github.com/BeamMP/BeamMP (lua/ge/extensions/MPUpdatesGE.lua; lua/vehicle/extensions/BeamMP/positionVE.lua, nodesVE.lua)
- BeamMP server: https://github.com/BeamMP/BeamMP-Server
- BeamMP settings docs: https://docs.beammp.com/game/multiplayer-settings/
- BeamNG multiplayer announcement, 2026-05-28: https://www.beamng.com/game/news/announce/something-new-is-on-the-horizon/
- Wreckfest, 2014-10-02: https://store.steampowered.com/news/app/228380/view/521614150626562813
- Wreckfest prediction hotfixes, 2019-08-29 and 2019-09-03: https://store.steampowered.com/news/app/228380/view/2415540140457120594 and https://store.steampowered.com/news/app/228380/view/2415540140474692017
- LunaMultiplayer lock system: https://github.com/LunaMultiplayer/LunaMultiplayer/wiki/Lock-system
- LunaMultiplayer immortal vessels: https://github.com/LunaMultiplayer/LunaMultiplayer/blob/master/LmpClient/Systems/VesselImmortalSys/VesselImmortalSystem.cs

**Distributed authority**
- Fiedler, Networked Physics in Virtual Reality, 2018-02: https://gafferongames.com/post/networked_physics_in_virtual_reality/
- Noita Entangled Worlds: https://github.com/IntQuant/noita_entangled_worlds (noita_proxy/src/net/world.rs, des.rs; docs/distributed_world_sync.drawio; PR 509)
- Roblox network ownership: https://create.roblox.com/docs/physics/network-ownership
- Roblox server authority: https://create.roblox.com/docs/projects/server-authority
- Roblox techniques: https://create.roblox.com/docs/projects/techniques

**Lockstep and rollback**
- Factorio FFF #36 (2014-05-31), #47, #63, #83 (2015-04-24), #147 (2016-07-15), #149 (2016-07-29), #188 (2017-04-28),
  #302 (2019-07-05), #340 (2020-03-27), #370 (2022-09-23), #412 (2024-05-24), #415 (2024-06-14):
  https://www.factorio.com/blog/post/fff-36 (and the same pattern for each number)
- Factorio float desync comment, 2017-09-19: https://forums.factorio.com/viewtopic.php?t=52747
- Factorio desynchronization wiki: https://wiki.factorio.com/Desynchronization
- Planetary Annihilation ChronoCam, 2013-10-09: https://www.forrestthewoods.com/blog/tech_of_planetary_annihilation_chrono_cam/
- ChronoCam Q&A, 2013-10-17: https://www.forrestthewoods.com/blog/qa_planetary_annihilation_chrono_cam/
- Synchronous RTS engines and a tale of desyncs, 2011-07-09: https://www.forrestthewoods.com/blog/synchronous_rts_engines_and_a_tale_of_desyncs/
- Fiedler, Floating Point Determinism (GPG, Battlezone 2, MotoGP), 2010-02-24: https://gafferongames.com/post/floating_point_determinism/
- FAF sync logs PR, 2026-09-26: https://github.com/FAForever/fa/pull/7274
- 1500 Archers on a 28.8, 2001-03-22: https://www.gamedeveloper.com/programming/1500-archers-on-a-28-8-network-programming-in-age-of-empires-and-beyond
- StarCraft II protocol: https://github.com/Blizzard/s2client-proto/blob/master/docs/protocol.md
- Stormgate SnowPlay interview, 2022-12-22: https://screenrant.com/james-anhalt-tim-morten-interview-snowplay-technology-stormgate/
- Stormgate patch 0.1.2, 2024-10-29: https://playstormgate.com/news/stormgate-patch-notes-0-1-2
- Recoil netcode overview: https://recoilengine.org/articles/netcode-overview/
- streflop README (in Recoil): https://github.com/beyond-all-reason/spring/blob/BAR105/rts/lib/streflop/README.txt
- Worms Armageddon replay suite (HN), 2026-05-03: https://hn.algolia.com/api/v1/items/48000939
- Hedgewars fixed point: https://raw.githubusercontent.com/hedgewars/hw/master/hedgewars/uFloat.pas
- LittleBigPlanet under RPCS3 (HN comment, indirect), 2026-08-31: https://hn.algolia.com/api/v1/items/49513678
- Photon Quantum intro: https://doc.photonengine.com/quantum/current/quantum-intro
- Quantum FAQ: https://doc.photonengine.com/quantum/current/getting-started/faq
- Quantum reconnecting: https://doc.photonengine.com/quantum/current/manual/game-session/reconnecting
- Bandit Trap on Quantum, 2026-06-29: https://blog.photonengine.com/bandit-trap-photon-quantum-physics-multiplayer/
- Rocket League GDC 2018 slides (Cone): https://media.gdcvault.com/gdc2018/presentations/Cone_Jared_It_Is_Rocket.pdf
- Unity Physics design: https://docs.unity3d.com/Packages/com.unity.physics@1.3/manual/design.html
- Netcode for Entities physics: https://docs.unity3d.com/Packages/com.unity.netcode@1.3/manual/physics.html
- Unreal networked physics (5.8): https://dev.epicgames.com/documentation/en-us/unreal-engine/networked-physics-overview
- 0 A.D. Fixed.h: https://raw.githubusercontent.com/0ad/0ad/master/source/maths/Fixed.h
- Box2D determinism, 2024-08-27: https://box2d.org/posts/2024/08/determinism/
- fixed3d, 2026-09-07: https://github.com/mas-bandwidth/fixed3d

**Friendslop, physics characters, survival** (decompiled code is third-party, version uncertain; read, not copied)
- Lethal Company decompiled (PlayerControllerB.cs, GrabbableObject.cs) (a third-party decompilation on GitHub, 2024-02; link withheld)
- Lethal Company networking notes: https://lethal.wiki/dev/advanced/networking and https://lethal.wiki/dev/apis/lethallib/custom-enemies/coding-ai
- LateCompany mod: https://thunderstore.io/c/lethal-company/p/anormaltwig/LateCompany/
- MoreCompany mod: https://thunderstore.io/c/lethal-company/p/notnotnotswipez/MoreCompany/
- R.E.P.O. on Photon, 2025-04-15: https://blog.photonengine.com/r-e-p-o-multiplayer-success-powered-by-photon/
- R.E.P.O. decompiled (PhysGrabObject.cs, PhysGrabber.cs, PlayerAvatar.cs, PhysGrabObjectImpactDetector.cs, NetworkManager.cs) (a third-party decompilation on GitHub; link withheld)
- R.E.P.O. NetworkingReworked mod: https://github.com/Ian0526/NetworkingReworked
- REPONetworkTweaks mod: https://thunderstore.io/c/repo/p/BlueAmulet/REPONetworkTweaks/
- Content Warning mod networking (Mycelium): https://github.com/RugbugRedfern/Mycelium-Networking-For-Content-Warning
- PEAK decompiled (CharacterSyncer.cs, ItemPhysicsSyncer.cs, Item.cs) (a third-party decompilation on GitHub, 2025-07; link withheld)
- PEAK FAQ: https://landfall.se/peak-faq
- Phasmophobia decompiled (PhotonObjectInteract.cs, ThrowingState.cs) (a third-party decompilation on GitHub, 2020-10; link withheld)
- Human: Fall Flat beta hosting note, 2017-09-20: https://steamstore-a.akamaihd.net/news/externalpost/steam_community_announcements/2149763711090767497
- Human: Fall Flat networked objects guide, 2019-04: https://steamcommunity.com/sharedfiles/filedetails/?id=1667236910
- Gang Beasts news (2016 to 2017): https://api.steampowered.com/ISteamNews/GetNewsForApp/v2/?appid=285900&count=30&maxlength=4000&enddate=1505400000&format=json
- Valheim decompiled (ZDOMan.cs, WearNTear.cs) (a third-party decompilation on GitHub, 2025-09; link withheld)
- Valheim networking notes, 2026-09: https://supercraft.host/wiki/valheim/better_networking/
- Deep Rock Galactic header dumps (DeepCSGWorld.h, TerrainLateJoinData.h, PickaxeDigOperationData.h) (third-party header dumps in a modding repository, 2024-03; link withheld)
- 7 Days to Die decompiled (GameManager.cs) (a third-party decompilation on GitHub, 2025-07; link withheld)
- Terraria netcode (tModLoader): https://github.com/tModLoader/tModLoader/wiki/Basic-Netcode
- Sons of the Forest news: https://api.steampowered.com/ISteamNews/GetNewsForApp/v2/?appid=1326470&count=120&maxlength=6000&format=json&feeds=steam_community_announcements
- Friendslop: https://en.wikipedia.org/wiki/Friendslop
