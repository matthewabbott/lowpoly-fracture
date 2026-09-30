# M7 track T6: a catalogue of netcode techniques for physics-heavy multiplayer

Research for milestone 7 (docs/roadmap.md section 7), read 2026-09-29. Scope: techniques, with numbers, for the
multiplayer models the engine could use; game case studies are track T2's. Our target throughout: player-hosted
co-op, 4 players first, 4–8 often, 12 sometimes; destruction; about 1500 awake bodies at peak (the debris cap), a town
of about 10.8k pieces; step 1/60 s with 4 substeps; town step 4.0 ms average at 8 workers, 12.5 ms at 1 worker;
fracture spikes of 20–90 ms (brief; docs/perf-log.md).

Model abbreviations used in the table:

- **LS**: deterministic lockstep (inputs only; every peer simulates everything).
- **RB**: rollback (predict remote inputs, resimulate on correction; GGPO, Photon Quantum).
- **SS**: state synchronisation (the host simulates; clients receive body state, either interpolating it or running
  their own physics and being corrected).
- **HY**: the Teardown hybrid (destruction as deterministic or host-decided commands on a reliable channel; bodies
  synced as SS; https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html, March 2026).
- **DA**: distributed authority (each peer simulates and sends the objects it owns).

"Det." is how much determinism the technique needs: *none*, *same build* (same executable, same machine type),
*cross-platform* (bit-exact on x64, ARM64 and emulators), or *fracture only*.

Numbers marked (est.) are my arithmetic from the cited figures; numbers marked (unverified) I could not confirm from a
primary source this session, with what would verify them.

## 1. The technique table

### 1a. Body state on the wire

| Technique | Models | Gain (numbers) | Cost | Det. | Fit for us | Source |
|---|---|---|---|---|---|---|
| Bounded position quantisation | SS, HY, DA | 3×32 → 50 bits/body at 2 mm over 512×512×32 m; 60 bits at 1/4096 m (state sync needs the finer grid) | Bounds per map or sector; km maps need sector ids | none | Yes: 18–21 bits/axis per 512 m sector | Fiedler, Snapshot Compression (2015); State Synchronization (2015) |
| Smallest-three quaternion | SS, HY, DA | 128 → 29 bits (9 bits/component) for interpolation; 47 bits (15/component) when clients extrapolate | Renormalise on decode | none | Yes | same |
| Velocity: omit or quantise | SS, HY | Snapshot interpolation needs none; state sync sends 11–12 bits/component (±32–64 m/s) | Without velocity, clients cannot extrapolate or simulate | none | Send for bodies near a player, omit for far ones | Fiedler 2014/2015 |
| At-rest bit, forced rest | SS, HY, DA | 1 bit replaces velocities; 901 resting cubes cost ~15 kbit/s total with delta coding; VR demo forces rest after 16 still frames | Needs sleep agreement between host and client | none | Yes: Box3D sleep plus our rubble freeze map onto it | Fiedler, Snapshot Compression; Networked Physics in VR (2018) |
| Delta against an acked baseline | SS, HY | Changed position 26.1 bits, orientation 23.3 bits (vs 50 + 29 absolute); unchanged body 1 bit | Host keeps per-client ack state and a ring of past snapshots; baseline is about one RTT old | none | Yes | Fiedler, Snapshot Compression |
| Delta against a ballistic prediction from the baseline | SS, HY | A body that flew exactly as predicted costs 1 bit ("perfect prediction"); errors sent as offsets | Host runs the same cheap predictor per client baseline | none (the predictor must match, not the physics) | Strong: most awake debris is airborne or sliding | Fiedler, Networked Physics in VR |
| Priority accumulator and packet budget | SS, HY, DA | Hard cap on bytes: 64 cubes per packet at 60 Hz fits 256 kbit/s (≈8 B per cube update); Teardown sends everything eventually at ≈1 Mbit/s per client | Far or boring bodies update rarely: visible snapping under load | none | Essential for 1500 awake bodies | Fiedler, State Synchronization; Teardown blog |
| Entropy coding | SS, HY, LS | Arithmetic coding estimated at 25% on top of hand-tuned bit packing; Unity uses bucketed Huffman on quantised deltas (value 1 in 3 bits) | CPU per packet; models must match both ends | none | Later: pays after quantisation and deltas | Fiedler, Snapshot Compression; Unity Netcode for Entities 1.4, Compression |
| Curves / keyframes (ChronoCam) | SS, HY | An idle unit costs 0 bytes; a build from 0 to 1 is two keys instead of dozens; whole game targets 1 Mbit/s | Client state must derive from curves only; server keeps history | none | Yes for ballistic debris and our ghosts: one keyframe at launch, one at landing | Forrest Smith, Tech of PA: ChronoCam (2013) |
| Most-recent-state masks (Tribes) | SS, HY | Per-object dirty bits (≈20 per object); on loss resend only the latest value; 3 bits/packet + 1 bit/event overhead; 28.8k modem: 10 packets/s × 200 B | Bookkeeping per packet | none | Yes for sparse state (links, vehicles, pools) | Frohnmayer & Gift, TRIBES Engine Networking Model (1999–2000) |
| Snapshot interpolation buffer | SS, HY | Smooth display with no client physics; buffer ≈3 send intervals: 350 ms at 10 Hz (with 5% loss), 150 ms at 30 Hz, 85 ms at 60 Hz; Source uses 100 ms at 20 updates/s | Everything remote is seen that late; no local interaction | none | Far bodies only | Fiedler, Snapshot Interpolation; Valve, Source Multiplayer Networking |
| State sync with client physics and correction | SS, HY | Clients extrapolate between updates; jitter buffer 4–5 frames at 60 Hz | Client CPU for local physics; corrections pop | none | Yes: Teardown clients run local physics and take corrections | Fiedler, State Synchronization; Teardown blog |
| Quantise on both sides | SS, HY, DA | Host simulates from exactly the quantised state it sends, so client and host extrapolate from identical starting values; less drift, fewer pops, steadier stacks | Host writes 1500 transforms and velocities back into physics per tick; perturbation below contact slop at 1/4096 m | none | Yes, but test Box3D's reaction to per-tick SetTransform (contacts, sleep) | Fiedler, State Synchronization; Networked Physics in VR |
| Visual error smoothing | SS, HY, RB, DA | Render proxy chases the simulated body: Fiedler blends 0.95 per frame for errors ≤25 cm, 0.85 for ≥1 m; Roblox example smooths over 0.07 s and only after a jump | Render lags the simulation slightly | none | Yes (renderer-side only) | Fiedler, State Synchronization; Roblox, Server authority techniques |

### 1b. Commands and identity

| Technique | Models | Gain (numbers) | Cost | Det. | Fit for us | Source |
|---|---|---|---|---|---|---|
| Reliable ordered messages over UDP | HY, LS, SS | Ordered delivery without TCP head-of-line stalls; 32-packet ack bitfield, resend every 0.1 s, rides in the same packets as unreliable state | A queue and sequence buffers (1024 entries) | none | Yes: the destruction command channel | Fiedler, Reliable Ordered Messages (2016) |
| Large-block transfer (slices) | all | 1 KB slices, 256 slices, 256 KB blocks: 2 s at 1 Mbit/s | Separate from the realtime stream | none | Late join and saves | Fiedler, Sending Large Blocks of Data (2016) |
| Deterministic ids for runtime objects | HY, SS, RB | No id round trip: Roblox derives an instance GUID from type, source, simulation frame and a per-frame counter; Unity hashes pre-spawned ghosts by type and scene, and matches predicted spawns within a 5-tick window | Id derivation must be identical on both sides | same-build for the derivation only | Yes: piece ids from (command sequence, cell index) | Roblox, Server authority techniques (2026); Unity Netcode for Entities, Ghost spawning |
| Host-assigned ids with a dictionary | SS, HY | Tribes: ghost ids from a limited range per connection, created and deleted with guaranteed delivery | Every reference must wait for its creation | none | Fallback: explicit id lists in commands (2–3 B per new piece) | TRIBES paper |
| Command watermark on state packets | HY | State for an id the client has not created yet is held, not applied; commands carry the host-quantised transforms they depend on | A few bytes per packet | none | Yes (design; see note 1.3) | Design from Tribes ordering and Unity spawn classification |
| Ship host-quantised geometry, not seeds | HY | Clients need no deterministic fracture at all: host quantises the cells it keeps, sends vertices (≈150–250 B per low-poly piece, est.) | 15–50 KB per 100-piece blast (est.), bigger late-join payload | none | Strong: convex pieces are cheap to ship, unlike Teardown's voxels | Design; compare Teardown's 30–50 MB scene |
| Deterministic destruction commands (seed and site) | HY, LS | ≈40 B per fracture instead of geometry; command-replay late join | Fracture must be bit-exact on every client | fracture only (cross-platform) | Maybe: keep as an option once fracture is proven cross-platform | Teardown blog |

### 1c. Lockstep mechanics

| Technique | Models | Gain (numbers) | Cost | Det. | Fit for us | Source |
|---|---|---|---|---|---|---|
| Inputs only, redundant until acked | LS, RB | All inputs resent in every packet until acked: 2 s of 60 Hz inputs worst case 90 B, ≈15 B delta-coded | Input delay, determinism | cross-platform | Yes if lockstep | Fiedler, Deterministic Lockstep (2014) |
| Input delay and playout buffer | LS | 100 ms playout buffer; fighting games 1 frame, puzzle games 4–5 frames | Latency felt on everything not predicted | cross-platform | Yes with latency hiding (below) | Fiedler 2014; GGPO Developer Guide |
| Adaptive input delay | LS, RB | Quantum defaults: delay 0–60 ms, applied from 100 ms ping | Delay changes are felt | cross-platform | Yes | Photon Quantum 3, DeterministicSessionConfig API |
| Turns (commands several ticks ahead) | LS | Age of Empires: 200 ms turns, commands run two turns later; turn length adapted to the slowest machine's frame rate and ping; <250 ms unnoticed, 250–500 ms playable | Coarse latency | cross-platform | Tick-stamped commands already exist (determinism rule 10) | Bettner & Terrano, 1500 Archers on a 28.8 (2001) |
| Server-timed ticks, hard tolerance | LS, RB | A late peer does not stall others: Quantum's server expires a tick after 8 frames (default) and substitutes repeated or null input | The late player loses control briefly | cross-platform | Essential at 12 players | Quantum DeterministicSessionConfig (InputHardTolerance 8) |
| Clock sync and time dilation | LS, RB, SS | Overwatch client runs ahead by RTT/2 + one frame and speeds from 16 to ≈15.2 ms frames (5%) to refill the server buffer; Rocket League: server tells the client to run extra or fewer frames, or consumes 0–2 inputs per frame; GGPO: the peer ahead sleeps ≥3 frames, at most 9, averaged over 40 frames; Quantum sends time corrections 4 times a second | Small speed changes | none | Yes | Edgegap summary of Ford GDC 2017; Cone GDC 2018 slides; GGPO timesync.h; Quantum API |
| Slow the whole session for a bad ping | LS | Quantum can lower the simulation time scale between 100 and 300 ms ping (off by default: minimum 100%) | Everyone slows | cross-platform | Only as a last resort | Quantum DeterministicSessionConfig |
| Catch up by running extra ticks | LS, RB | Fiedler's demo allows up to 4 simulation steps per render frame; Factorio clients download the map while the server runs on, then fast-forward | Needs headroom: at our town's 4 ms step (8 workers) a 16.7 ms frame fits 4 ticks, gaining 3 per frame; at 12.5 ms (1 worker) it gains 0.3 | cross-platform | Only on fast machines; add pause-on-join (Factorio 2.0) | Fiedler 2014; Factorio FFF #149, #415 |
| Latency hiding for the local player (Factorio) | LS | A throwaway "latency state" rebuilt every tick from the confirmed state plus unconfirmed local actions: movement, building, mining, GUIs; since 2024 also cars, hiding 30 ticks (500 ms) | Mispredictions snap; no interactions with others' actions | none for the overlay | Strong precedent for H2 | Factorio FFF #83 (2015), #412 (2024) |
| Per-tick checksums and dumps | LS, RB | Quantum sends a checksum every 60 ticks by default (set 0 for release), keeps 3 verified frames for a dump; Factorio compares a cheap heuristic CRC every tick; AoE compared world checksums | Hash cost per tick | cross-platform | Yes: lpWorld_Hash exists (per-tick cost unmeasured) | Quantum Config Files; Factorio wiki/FFF #47; AoE paper |
| Per-subsystem and per-island hashes | LS, RB | Point a desync at a subsystem and structure, not "the world" | Hash vectors kept in a ring | cross-platform | Yes: extend lpWorld_Hash (note 4.4) | Design; Box3D recordings already hash per step (brief) |
| Replays and seek | LS, RB, SS | Input log + periodic keyframe snapshots gives replay and backward seek; ChronoCam seeks by curves | Snapshot memory | same build | Box3D already keyframes its replays (brief) | Box3D world_snapshot.c; ChronoCam |

### 1d. Rollback and prediction

| Technique | Models | Gain (numbers) | Cost | Det. | Fit for us | Source |
|---|---|---|---|---|---|---|
| Full rollback | RB | No input delay: GGPO predicts up to 8 frames; Quantum keeps a 60-tick window | Resimulate k ticks per frame (note 5.1): for us 5 × 4–12.5 ms per correction, plus fracture spikes inside the window | cross-platform | No for the world | GGPO sync.h; Quantum FAQ |
| Predict everything (Rocket League) | SS+RB | No input delay, 100% server authority; client rewinds all actors to the server frame | 200 ms ping at 120 Hz = 24 resimulated frames per correction, for 6 cars and a ball | none (corrections) | No: we have 1500 bodies | Cone, GDC 2018 slides |
| Prediction culling | RB | Only entities in a sphere around the local player are predicted; verified ticks simulate everything | Culled things move differently when seen | cross-platform | Only the idea (CPU near the player) | Quantum 3, Prediction Culling |
| Predict only the local avatar (and what it touches) | SS, HY, LS | Source predicts only the local player; Fusion 2.1 without forecast keeps proxies kinematic in remote time; Unity switches ghosts between predicted and interpolated by distance; Roblox predicts inside a radius that grows or shrinks with device speed | A second, small simulation for the avatar | none | Yes | Valve wiki; Fusion 2 Physics; Unity NfE Optimizations; Roblox Server authority |
| Separate cosmetic physics world | all | Unity runs a client-only physics world (debris, particles) beside the predicted one, at its own rate | None to the network | none | Yes: our dust is already outside the simulation (determinism rule 9) | Unity NfE Physics |
| Lag compensation by rewinding targets | SS, HY | Source keeps 1 s of player positions and rewinds only players to server time − latency − interpolation; Overwatch stops rewinding above ≈220 ms RTT | History per target; "shot around the corner" | none | Not needed (note 5.4) | Valve wiki; Edgegap summary of GDC 2017 |
| Shooter-decided hits | LS, HY | The shooter sends "piece P, local point, direction" at the tick it saw; everyone applies it | Trusts clients (acceptable: no competitive integrity needed) | none beyond LS | Yes for rifle and cannon | Design; cheating is out of scope (brief) |

### 1e. Late join, saves, migration, relevancy, authority

| Technique | Models | Gain (numbers) | Cost | Det. | Fit for us | Source |
|---|---|---|---|---|---|---|
| Snapshot join (buddy snapshot) | LS, RB | Quantum: a peer uploads a platform-independent snapshot of a verified tick; timeouts 15 s wait, 20 s download, 10 s upload; local snapshots accepted if <10 s old; server keeps only ≈10 min of input history | Snapshot size and upload on a player's line | same build (bit-exact) | Needed for LS | Quantum 3, Reconnecting |
| Command-log replay join | HY | Teardown replays the destruction command stream instead of a 30–50 MB scene; join is disabled when the buffer fills | Log grows with destruction | fracture only | Possible; geometry shipping is simpler for us | Teardown blog |
| Download while running, then fast-forward | LS | Nobody waits; Factorio joins can take minutes, so 2.0 added pause-on-join | Joiner may never catch up on a slow CPU | cross-platform | Yes with pause-on-join fallback | Factorio FFF #149, #415 |
| Canonical rebuild at an agreed tick | LS | All peers (host too) rebuild the world from the same serialised state at tick T: Box3D's hidden state (warm starts, pair order, id free lists) becomes identical without serialising it | A rebuild hitch for everyone (unmeasured) | same build | Candidate for join and desync repair (note 6.2) | Design |
| Save games | all | Same serialiser as join | Needs a versioned schema across patches (Box3D's snapshot is same-build only) | none for saves | Yes | Box3D world_snapshot.c (brief) |
| Host migration | LS, HY, DA | LS: every peer holds the full state, so only the timing role moves; HY: a client adopts its own approximate world (exact geometry, approximate transforms); Unity's distributed authority promotes a new session owner automatically | Agreement on the last confirmed tick (LS); pops (HY) | as the model | Cheap in both our candidate models | Unity NGO 2.x, Distributed authority; design |
| Per-client relevancy and priority | SS, HY | Tribes scopes ghosts per client; Unity scales importance by distance tiles, caps chunks per snapshot, limits send rate per type (e.g. every 10th snapshot); Teardown favours what each player sees | Relevancy per client | none | Yes | TRIBES paper; Unity NfE Optimizations; Teardown blog |
| Activity zones from shared state | LS, HY | CPU scales with active cells (roadmap 12) | In LS every peer simulates the union of everyone's zones | cross-platform in LS | Yes; weigh the union cost at 12 players | docs/roadmap.md section 12 |
| Authority follows the toucher, ownership on grab | DA, HY | Zero-latency interaction for the toucher; authority and ownership carry sequence numbers, the host arbitrates, authority returns to the host at rest; 4 players under 1 Mbit/s total | Stacks touched by two players; conflicts | none | Limited: stress needs one authority per structure | Fiedler, Networked Physics in VR |
| Automatic network ownership by proximity | DA | Roblox hands unanchored parts to nearby clients (by distance and client hardware); connected assemblies share one owner | Unverifiable client physics | none | Only for vehicles or held objects | Roblox, Network ownership |

### 1f. Transport and testing

| Technique | Models | Gain (numbers) | Cost | Det. | Fit for us | Source |
|---|---|---|---|---|---|---|
| Steam Networking Sockets with relays (SDR) | all | NAT traversal and relays handled by Steam; IP addresses hidden; max UDP payload 1300 B (≈1248 B encrypted); reliable messages up to 512 KB; default send rate 256 KB/s per connection; Nagle 5 ms | Steam dependency; relayed ping can be worse off Valve's backbone | none | Yes (the user's plan) | Steamworks SDR docs; GameNetworkingSockets headers |
| NAT traversal with relay fallback | all | Direct connections succeed over 90% of the time; two hard NATs need a relay | Relay infrastructure | none | Given by SDR | Tailscale, How NAT traversal works (2020) |
| Packets ≤1200 B, fragment only 2–4 ways | all | At 1% loss, 10 fragments lose 9.5% of packets, 100 lose 63% | Split big snapshots across packets | none | Yes | Fiedler, Packet Fragmentation (2016) |
| Sync test (rollback every frame) | RB, LS | GGPO rolls back one frame every frame and compares checksums: desyncs show up the frame they are introduced | 2× simulation | same build | Yes as a round-trip test of serialise/restore | GGPO Developer Guide |
| Save/load every tick ("heavy mode") | LS | Factorio saves and loads every tick and compares; tagged human-readable level dumps diffed per tick | Very slow ("units of FPS") | same build | Yes as a CI job | Factorio FFF #47; wiki Desynchronization |
| Cross-machine hash CI | LS, RB | Same scripts on x64 and ARM64 hosted runners, per-tick hash logs diffed | CI minutes | cross-platform | Yes: GitHub gives free arm64 Linux and Windows runners to public repos | GitHub changelog, Jan and Apr 2025 |

## 2. Notes on body state bandwidth

### 2.1 What Fiedler measured

Glenn Fiedler's 2014–2015 series networks 900 (later 901) physics cubes. Uncompressed at 60 Hz the stream is
17.37 Mbit/s. Smallest-three orientation (29 bits) brings it to about 12.4 Mbit/s, dropping velocity for resting cubes to
about 9.4, bounded positions (50 bits at 2 mm) to about 4.7 (https://gafferongames.com/post/snapshot_compression/,
2015-01-04). Delta coding against the last acknowledged snapshot then encodes a changed position in 26.1 bits and a
changed orientation in 23.3 bits on average, with separate "changed" bits, and a fully resting scene costs about
15 kbit/s. His target was 256 kbit/s; the article does not state the final figure under motion. His summary is that the
best savings come from what you do not send. For state synchronisation (clients run physics too) he needed finer
quantisation, 4096 steps per metre and 15-bit quaternion components, and capped each 60 Hz packet at 64 cube updates
for 256 kbit/s, which is about 8 bytes per cube update (https://gafferongames.com/post/state_synchronization/,
2015-01-05). In the VR demo (Oculus, 2018) he added delta coding against a ballistic prediction from the baseline: a
cube that flew as predicted costs one bit (https://gafferongames.com/post/networked_physics_in_virtual_reality/).

### 2.2 Per-body sizes used below

Two profiles, derived from those figures (est.):

- **Interpolation profile** (far bodies; clients only display them): position 1/512 m over a 512 × 512 × 64 m sector
  (18 + 18 + 15 bits), quaternion 29 bits, at-rest bit, id about 8 bits: 89 bits, 11 B absolute; about 7.5 B delta-coded
  (Fiedler's 26.1 + 23.3 bits plus flags and id).
- **State-sync profile** (near bodies; clients simulate them): position 1/4096 m (21 + 21 + 18 bits), quaternion
  47 bits, linear velocity ±64 m/s at 1/32 (36 bits), angular velocity ±32 rad/s at 1/64 (36 bits), two flags, id about
  8 bits: about 189 bits, 24 B absolute; I assume half that, 12 B, delta-coded against a ballistic prediction (est.;
  Fiedler's 256 kbit/s budget implies about 8 B per update for cubes).

Ids: send bodies in increasing id order and code the gap (a few bits when dense), or a changed-bit per body of the
baseline's list.

### 2.3 Priority accumulator, packet budget

Every tick, each body's priority (near a player, in view, moving fast, recently hit, owned by a player) is added to its
accumulator; bodies are sorted by accumulator and written until the packet budget is full; written bodies reset to
zero, the rest carry their priority into the next packet (Fiedler, State Synchronization; the VR demo clears negative
priorities to -1). This turns bandwidth into a hard cap and makes overload degrade into lower update rates, which is
what Teardown describes as snapping when corrections come less often (Teardown blog). A Steam packet carries at most
about 1232 B of plaintext (GameNetworkingSockets `steamnetworkingsockets_internal.h`: 1300 B UDP payload, 1248 B
encrypted, 16 B AES-GCM tag), so one packet holds about 100 delta-coded near bodies.

### 2.4 Curves for ballistic debris

Planetary Annihilation's ChronoCam sends every property as a curve of keyframes (linear, step and pulse curves); a
unit doing nothing costs nothing, and a construction from 0 to 1 is two keyframes
(https://www.forrestthewoods.com/blog/tech_of_planetary_annihilation_chrono_cam/). For us: when a piece leaves contact
(or becomes a ghost), send one keyframe (tick, position, orientation, both velocities: about 24 B); clients integrate it
with the same gravity; the host sends the next keyframe on contact or when its own replay of the curve drifts past a
threshold. A 1.5 s flight at 20 Hz is 30 updates, or two keyframes (est.). Our ghost tier, integrated by our own core,
is already ballistic and ends in a landing event (docs/determinism-rules.md rule 8), so it maps onto curves directly.

## 3. Quantise on both sides

State synchronisation extrapolates: between updates each client runs physics from the last state it received. If the
host keeps simulating from full-precision floats while the client starts from the rounded values, the two worlds
diverge from the first substep, and every update pops the client back. Fiedler's fix is for the authority to round its
own state to exactly what it sends, every tick, before simulating, so both sides extrapolate from bit-identical starting
values (https://gafferongames.com/post/state_synchronization/); in the VR demo both sides quantise to keep stacks from
jittering, and cubes that have barely moved for 16 frames are forced to rest
(https://gafferongames.com/post/networked_physics_in_virtual_reality/). The rounding error at 1/4096 m (0.24 mm) and
15-bit quaternion components is far below contact slop, so it does not disturb the host's stacks. Divergence does not
vanish (client and host differ in which bodies they simulate and in contacts), but it starts later and grows from zero.

For us: quantising 1500 awake bodies per tick means writing transforms and velocities back into Box3D. That this does
not reset contacts, warm starts or sleep timers is unverified: test `b3Body_SetTransform` and the velocity setters on a
stacked pile and compare step cost and stack stability. The same idea applies to geometry (note 4.3): the host
quantises the fracture output it keeps, so host and clients hold identical pieces.

## 4. Commands for destruction

### 4.1 One reliable ordered channel beside the unreliable state

Fiedler's reliable-ordered messages ride inside the same UDP packets as the unreliable state: messages get increasing
ids, each packet acks the last 32 packets by bitfield, unacknowledged messages are re-included every 0.1 s while there
is room, and the receiver delivers them strictly in id order (https://gafferongames.com/post/reliable_ordered_messages/,
2016). Tribes separated the same concerns into guaranteed events, most-recent state and ghost creation, all over one
packet notification protocol with about 3 bytes of overhead per packet (TRIBES paper). Teardown sends every
scene-altering command (destruction, spawning, recolouring) on a reliable channel and body transforms on an unreliable
one (Teardown blog). Steam's sockets provide both kinds on one connection (reliable messages up to 512 KB).

### 4.2 Ids for objects created at runtime

- **Lockstep:** ids are free: the same commands in the same order allocate the same pool slots (our pools use LIFO free
  lists, determinism rule 4).
- **Host decides, clients follow (HY):** clients never split or fracture on their own, so their pools could stay in step
  with the host's, but any client-only allocation (cosmetic, predicted) must use a separate pool. Safer: derive ids
  from the command (command sequence, cell index), or list the host's piece indices explicitly in the command (2–3 B per
  new piece).
- **Predicted creation:** Roblox derives a GUID from the instance type, the creating source, the simulation frame and a
  per-frame counter on both client and server, so a client-created instance merges with the server's copy when it
  arrives (https://create.roblox.com/docs/en-us/projects/server-authority/techniques, updated 2026-09). Unity matches
  predicted spawns to server ghosts by type and spawn tick within 5 ticks and hashes pre-spawned ghosts by type and
  scene (https://docs.unity3d.com/Packages/com.unity.netcode@1.4/manual/ghost-spawning.html).

### 4.3 Ordering: a command before its state, state before its command

Unreliable state can overtake the reliable command that created the bodies it describes (a fracture's new pieces), and
state can describe a body a later command destroys. A simple rule set (design):

1. Each state packet carries the host's command watermark (the last command sequence applied at that tick). A client
   that has not applied up to the watermark holds the packet, or applies only the bodies it knows.
2. State for an unknown id is dropped; the priority accumulator will send it again.
3. Commands carry the state they depend on: "fracture body B at tick T" includes B's quantised transform at T, so the
   client places the new pieces exactly even though its own B was only approximately there.
4. For geometry, either the command carries a seed and site (about 40 B; clients must compute bit-identical cells:
   cross-platform determinism for fracture only) or the host's quantised cells (about 150–250 B per piece for low-poly
   hulls of 20–30 vertices at 16 bits per coordinate, est.; verify by counting vertices at the town's peak). Shipping
   geometry makes clients need no determinism at all: a 100-piece blast is about 15–25 KB, 0.1–0.2 s at 1 Mbit/s.

Unreal delays RPCs whose object references are not yet resolved (the `net.DelayUnmappedRPCs` console variable; unverified
this session, verify in the engine source).

## 5. Lockstep mechanics

### 5.1 Input delay, jitter, turns and clocks

- Fiedler's lockstep demo sends only inputs, all unacknowledged ones in every packet, and buffers about 100 ms before
  playing them; TCP showed visible hitches every few seconds at 100 ms and 1% loss
  (https://gafferongames.com/post/deterministic_lockstep/, 2014).
- Age of Empires ran 200 ms communication turns, executing commands two turns after issue, and resized turns from the
  slowest machine's frame rate and the pings; players did not notice latency under 250 ms
  (https://www.gamedeveloper.com/programming/1500-archers-on-a-28-8-network-programming-in-age-of-empires-and-beyond,
  2001).
- Photon Quantum's defaults (https://doc-api.photonengine.com/en/quantum/current/class_photon_1_1_deterministic_1_1_deterministic_session_config.html):
  60 Hz, input delay 0–60 ms applied once ping exceeds 100 ms, 3 redundant input sends, hard tolerance 8 ticks, time
  corrections 4 times a second, a 60-tick rollback window, checksum every 60 ticks, optional pure-lockstep mode (it
  recommends a minimum input delay of 10 there). The session config is agreed before the start and included in the
  checksum (https://doc.photonengine.com/quantum/current/manual/config-files): the model for our session-wide
  simulation settings (debris cap, budgets), which in lockstep cannot differ per machine.
- Clocks: the peer ahead waits. GGPO averages frame advantage over 40 frames and sleeps the leading peer by half the
  difference, only if ≥3 frames, at most 9 (`src/lib/ggpo/timesync.h`, `timesync.cpp`,
  https://github.com/pond3r/ggpo). Overwatch's client runs ahead of the server by half the RTT plus one buffered
  command frame and shortens its 16 ms frames to about 15.2 ms when the server's input buffer runs dry
  (https://edgegap.com/blog/game-backend-deep-dive-overwatch-2016-netcode-architecture-rollback, a secondary summary of
  Tim Ford's GDC 2017 talk https://www.gdcvault.com/play/1024001/-Overwatch-Gameplay-Architecture-and). Rocket League's
  server buffers inputs and tells the client to run extra or fewer physics frames, or itself consumes 0, 1 or 2 inputs a
  frame ("effective but with minor desyncs") (https://media.gdcvault.com/gdc2018/presentations/Cone_Jared_It_Is_Rocket.pdf).

### 5.2 The slowest peer

Two kinds of slow: network and CPU.

- **Network.** In peer-to-peer lockstep every peer waits for the slowest input. Server-timed lockstep (Quantum, Factorio
  after its 2017 rewrite, https://www.factorio.com/blog/post/fff-149) moves the clock to the server: after the hard
  tolerance the server fills a missing input with a repeat or nothing and moves on, so a lagging player loses control
  briefly and nobody else stalls. For a player-hosted game the host is that server. The last resort is slowing the
  whole session (Quantum's time scale, 100–300 ms ping band).
- **CPU.** Lockstep makes every peer simulate the entire world, including every fracture spike and every player's active
  zone. A toaster that cannot hold 60 ticks a second falls behind; a spike of 90 ms is 5.4 ticks it must catch up by
  running several ticks per frame, which needs headroom it may not have (our 1-worker step is 12.5 ms against a
  16.7 ms tick). Mitigations: drop or pause for the peer (Factorio lets a client that cannot catch up leave without
  bothering others, FFF #149), lengthen the input delay for everyone, or keep the simulation's worst case inside the
  slowest supported machine (the budgets are counts, determinism rule 7, so they can be set per session, not per
  machine).

### 5.3 Checksums, dumps, desync reports

Quantum computes a checksum of a verified tick every `ChecksumInterval` ticks (default 60), sends it to the server for
comparison, and keeps the last 3 verified frames to produce a dump when one differs; its guidance is to turn checksums
off for release (https://doc.photonengine.com/quantum/current/manual/config-files). Factorio compares a cheap heuristic
CRC of selected state every tick; in development a special mode computes a CRC of the whole map and saves a tagged,
human-readable map every tick, and the differences are usually a single variable (https://www.factorio.com/blog/post/fff-47).
Its desync report ships the client's desynced level and the server's reference level for diffing
(https://wiki.factorio.com/Desynchronization).

### 5.4 Per-subsystem and per-structure hashes (design)

`lpWorld_Hash` covers bodies, pieces (raw vertex bytes), bonds, links, vehicles, rigs and pools; `lpWorld_HashStress`
the solver (include/lpf/lpf.h:598-602; docs/determinism-rules.md). Proposed: compute a small vector per tick (bodies,
pieces, bonds, links, vehicles, rigs, pools, stress, Box3D's own state hash), send only the combined 64-bit value (8 B
per peer per tick), keep the vectors in a ring of about 2 s, and on a mismatch exchange the vector for the first bad
tick, then per-structure or per-island hashes for the bad subsystem, then dump that structure. Keeping per-structure
hashes incremental (recompute only dirty structures) also bounds the per-tick hash cost, which is unmeasured for the
10.8k-piece town.

### 5.5 Replays and seek

Every lockstep or rollback game gets replays from its input log; seeking needs keyframe snapshots. Box3D already records
a command log with a per-step hash and uses its internal world snapshot for replay keyframes and backward seek (brief;
`extern/box3d/src/world_snapshot.c`, snapshot format version 10). Factorio uses replays as a determinism test: replaying
must reproduce the recorded CRCs (FFF #47). ChronoCam gets seek from curves without determinism.

### 5.6 Latency hiding for the local player in a lockstep world

Factorio keeps a separate "latency state": each tick it is cleared, rebuilt from the confirmed game state, and all
not-yet-confirmed local actions are applied to it; the screen and new inputs use it (https://www.factorio.com/blog/post/fff-83,
2015). It self-corrects every tick; the known artefact is a player who runs ahead, then snaps against an object another
player built. In 2024 Factorio extended it to driving cars, hiding 30 ticks (500 ms at 60 UPS), with trouble around
duplicated one-shot effects and against unpredictable obstacles (https://factorio.com/blog/post/fff-412). This is the
closest precedent for H2's local character predicted outside the deterministic world. For us: the character (and a
driven vehicle) re-simulated for the D ticks of input delay each frame against the confirmed world: a capsule mover
(Box3D's `b3World_CastMover`) for D ≈ 6 ticks is microseconds; its pose enters the world as a tick-stamped input.

## 6. Rollback and prediction

### 6.1 Cost model

Predicted ticks per frame k ≈ ceil((RTT/2) / tick) + jitter margin. At 60 Hz, 100 ms RTT and 2 ticks of margin, k ≈ 5.
A correction resimulates k ticks and first restores a snapshot (and every tick saves one). For our world (est.): 5 × 4 ms
= 20 ms at 8 workers, 5 × 12.5 ms = 62 ms at 1 worker, before snapshot copies and before any fracture spike inside the
window (20–90 ms, redone unless fracture results are cached). GGPO bounds prediction to 8 frames (`sync.h`,
`MAX_PREDICTION_FRAMES`); Rocket League pays 24 resimulated frames at 200 ms and 120 Hz for seven bodies (Cone slides);
Fiedler rejected lockstep-with-prediction for VR because hiding 250 ms at 90 Hz would mean about 25 steps per rendered
frame (VR article). Unity warns that predicting many ticks per frame can spiral and lets physics batch ticks at the
price of more mispredictions (https://docs.unity3d.com/Packages/com.unity.netcode@1.4/manual/physics.html). Verdict:
full-world rollback is out for us; rollback of one avatar is cheap.

### 6.2 Partial prediction

- Quantum's prediction culling predicts only entities inside a sphere around the local player; everything outside runs
  only in verified ticks, which always simulate everything (https://doc.photonengine.com/quantum/current/manual/prediction-culling).
- Roblox's server authority mode (docs updated 2026-09) predicts instances within a radius of the local character that
  grows or shrinks with device performance; any mismatch in synchronised attributes triggers a rollback and
  resimulation; a 60 Hz server heartbeat (https://create.roblox.com/docs/en-us/projects/server-authority).
- Unity Netcode for Entities switches ghosts between predicted and interpolated by distance, and runs cosmetic physics in
  a second, client-only world (Optimizations; Physics pages).
- Photon Fusion 2.1 offers three physics approaches: Forecast (extrapolate remote bodies into local time and run full
  local physics), Forecast Disabled (kinematic proxies in remote time; cheapest) and a predict-and-resimulate addon for
  client-server only, the most expensive because Unity's physics is not built for resimulation
  (https://doc.photonengine.com/fusion/v2/manual/physics/physics-overview, updated 2026-08).
- Unreal's networked physics has three replication modes: legacy default, predictive interpolation, and resimulation
  (client half an RTT ahead, history of at least one RTT, rewind when the server differs enough)
  (https://dev.epicgames.com/documentation/en-us/unreal-engine/networked-physics-overview). The resimulation
  thresholds and whether it resimulates islands or everything are unverified (the resimulation page would not load).

### 6.3 Smoothing corrections

Render a proxy that chases the simulated body instead of drawing the simulated body. Fiedler blends the error away
(factor 0.95 per frame for errors up to 25 cm, 0.85 from 1 m, orientation similarly) (State Synchronization). Roblox's
soccer template smooths only after the simulated ball jumps, so there is no visual lag normally (Server authority
techniques). Quantum's advice for remote avatars is acceleration instead of instant velocity changes and, if needed, a
little input delay (Mispredictions and Entity Views page).

### 6.4 Hitscan tools (rifle, cannon)

Our tools are recorded as camera rays resolved at the tick they apply (brief). Under lockstep that tick is the input
delay later than what the shooter saw; under the hybrid the host's bodies differ from the client's approximations. Valve
rewinds only players, keeping one second of history, to the time the shooter saw
(https://developer.valvesoftware.com/wiki/Source_Multiplayer_Networking, read via the Internet Archive); Overwatch stops
rewinding above about 220 ms. Our targets are mostly structures, which do not move, and co-op needs no cheat
resistance, so the cheap answer is shooter-decided hits: the shooter's client resolves the ray against what it displays
and sends "piece P, point in P's local frame, direction, tool" stamped with its tick; every peer applies that impact.
If P no longer exists at the apply tick (fractured by an earlier command), fall back to a ray cast at the apply tick
from the same origin. In lockstep this is deterministic because every peer applies the same command.

## 7. Late join, reconnection, saves, host migration

### 7.1 What others do

- **Quantum:** late joiners and reconnectors pause and request a "buddy snapshot" of the last verified tick from another
  client (the server load-balances who uploads); the snapshot is a platform-independent blob; waiting times out after
  15 s, downloads after 20 s, uploads after 10 s; a client's own snapshot is accepted if under 10 s old (up to 2 min for
  small games with an empty-room TTL); replaying from the start is not offered because the server keeps only about
  10 minutes of input history (https://doc.photonengine.com/quantum/current/manual/game-session/reconnecting).
- **Factorio:** the server saves and uploads the map while play continues; the joiner loads it and simulates queued
  ticks as fast as it can; joins on big maps can take minutes, hence the 2.0 option to pause while someone joins
  (https://www.factorio.com/blog/post/fff-415, 2024).
- **Teardown:** considered sending the whole scene (30–50 MB and more), sending changed objects, and replaying the
  deterministic command stream; chose command replay as by far the smallest, with a capped buffer that disables join in
  progress after extreme destruction (Teardown blog).
- **Planetary Annihilation:** the server keeps the full curve history, so joining and reconnecting read curves
  (ChronoCam).
- **Photon Fusion host migration:** the host periodically pushes a snapshot to the Photon cloud and a new host restarts
  from it (unverified: the Fusion host-migration page was blocked by a bot check on 2026-09-29; verify interval and
  what is lost there). Unity's distributed-authority topology promotes a new session owner automatically when the
  current one leaves (https://docs.unity3d.com/Packages/com.unity.netcode.gameobjects@2.4/manual/terms-concepts/distributed-authority.html).

### 7.2 For us (est.)

- **Hybrid join payload:** piece geometry (10.8k pieces × 150–250 B ≈ 1.6–2.7 MB at the town's peak) plus bodies,
  bonds, links, vehicles, rigs and pools (under 1 MB), roughly 1–2 MB compressed (compression ratio unverified). At
  5 Mbit/s: 2–4 s. No Box3D internals are needed: the joiner builds its own approximate physics world, which the host's
  state stream corrects. Pristine structures can come from the scene seed if scene building is deterministic in one
  build, shrinking this to the fractured pieces.
- **Lockstep join payload:** must be bit-exact, including Box3D's hidden state (warm starts, sleep timers, contact
  order, id free lists). Our pieces alone are 10.8k × 296 B structs (docs/perf-log.md:454) plus shape data; with Box3D's
  snapshot (bodies, hulls, contacts with manifolds, trees) I guess 10–20 MB raw (unmeasured). The alternative is a
  canonical rebuild: at an agreed tick every peer, the host included, serialises its own state with our schema and
  rebuilds its world from it, so the joiner (loading the same bytes) and the others end in identical Box3D state. This
  trades bytes for a synchronised hitch (unmeasured), and the same primitive repairs desyncs.
- **Catch-up:** a joiner who downloads for 10 s must then simulate 600 ticks; at 4 ms a tick (8 workers) running 4 ticks
  per 16.7 ms frame it gains about 3 ticks per frame and catches up in about 3–4 s; at 12.5 ms (1 worker) it gains
  almost nothing. Pause-on-join is the fallback for slow machines.
- **Saves:** the same schema, versioned across patches (Box3D's snapshot is same-build only).
- **Host migration:** in lockstep any peer holds the full state, so only the host's timing role moves, after the
  remaining peers agree on the last tick for which all of them hold every input (each peer keeps the recent inputs of
  all players). In the hybrid, a client promotes its own world: geometry is exact (commands), transforms approximate, so
  migration costs a round of pops and no transfer.

## 8. Interest management and determinism

- **State sync and hybrid:** relevancy is free of determinism. Tribes scopes objects per client and prioritises within
  scope; Unity scales importance by distance tiles, caps chunks per snapshot and can limit a prefab to every Nth
  snapshot; Teardown favours what each player sees. Relevancy also drives client CPU: a hybrid client only needs local
  physics near itself.
- **Lockstep:** bandwidth needs no relevancy (inputs only), but CPU does, and activity must derive from shared
  simulation state, never a camera (roadmap section 12; determinism rule 7). The catch at 12 players: every peer
  simulates the union of all players' active zones. In a facility game where players split up, a lockstep toaster pays
  for twelve players' worth of activity; a hybrid client pays for its own.
- **Rollback:** Quantum's prediction culling is relevancy for CPU, applied only to predicted ticks.
- **Hybrid commands:** destruction commands go to everyone regardless of relevancy (geometry must match everywhere to
  join, migrate and correct), unless a zone keeps its own log that a player receives on entering, which is complexity
  not worth it at 12 players.

## 9. Distributed authority

Fiedler's VR demo (four players, host as arbiter and relay) gives each cube two claims with sequence numbers: authority
(the last player to interact, spread recursively through collisions, returned to the host when the cube comes to rest,
after the host confirms) and ownership (taken on grab, stronger than authority, held until release). Players interact
with zero latency because they simulate what they touch; the budget was under 256 kbit/s per player
(https://gafferongames.com/post/networked_physics_in_virtual_reality/). Roblox hands unanchored parts to nearby clients
by distance and client hardware, and one owner covers a whole connected assembly; it cannot verify client physics
(https://create.roblox.com/docs/physics/network-ownership), and its 2025–2026 server-authority mode moves away from that
for competitive games. For us the snag is stress: a structure's break decisions need all the loads on it in one place,
so a structure cannot be split between two owners. Candidates for client authority are the few things a player
controls directly: a held object, a driven vehicle, a crane in use ("authority follows the driver"), with the host
keeping structures and rubble.

## 10. Transport for player-hosted games

- **Steam:** `ISteamNetworkingSockets` P2P over Steam Datagram Relay: rendezvous through Steam, relayed and encrypted
  traffic, IP addresses hidden; a relayed route can be faster than the default route for many players and slower for
  hosts far from Valve's data centres (https://partner.steamgames.com/doc/features/multiplayer/steamdatagramrelay). The
  open-source GameNetworkingSockets headers give a 1300 B maximum UDP payload, 1248 B encrypted payload, reliable
  messages up to 512 KB, a 512 KB send buffer, Nagle 5 ms, and a default send rate of 256 KB/s per connection (about
  2 Mbit/s), which a state stream above that must raise
  (https://github.com/ValveSoftware/GameNetworkingSockets, `steamnetworkingtypes.h`, `steamnetworkingsockets_internal.h`).
  The docs do not state a price for relay use (unverified).
- **Epic Online Services P2P:** NAT traversal with optional relays; its per-packet limit (I recall 1170 B) and relay
  terms are unverified (the docs page would not render; verify in the EOS SDK header `eos_p2p_types.h`).
- **NAT:** hole punching works for "easy" NATs; two hard (endpoint-dependent) NATs need a relay; direct connections
  succeed over 90% of the time in Tailscale's experience (https://tailscale.com/blog/how-nat-traversal-works, 2020).
- **Packet size:** keep payloads at 1000–1200 B (safe on IPv4 and IPv6) and fragment only 2–4 ways: at 1% loss a
  10-fragment packet is lost 9.5% of the time (https://gafferongames.com/post/packet_fragmentation_and_reassembly/).
- **Home upstream (Ookla Speedtest Global Index, August 2026, rolling quarter, fixed broadband medians):** global 64.7
  Mbit/s up (129.7 down); United States 63.9, United Kingdom 60.6, Germany 37.9, Australia 42.2, Canada 95.4, Japan
  136.2, France 259.0 (https://www.speedtest.net/global-index, parsed from the page's data). Medians hide a tail of
  hosts on DSL or low cable tiers with single-digit upstream (unverified: Ookla does not publish percentiles on that
  page; verify with FCC Measuring Broadband America or M-Lab data). A 1998 modem got 2 KB/s (TRIBES paper).

## 11. Desync tooling and determinism testing in CI

- **Round-trip every tick:** GGPO's sync test rolls back one frame every frame and compares checksums, so a desync shows
  at the frame that introduced it (https://github.com/pond3r/ggpo/blob/master/doc/DeveloperGuide.md); Factorio's heavy
  mode saves and loads every tick. For us this is the test for the serialise/rebuild primitive: step, serialise, rebuild
  into a second world, step both, compare hashes.
- **Replays reproduce hashes:** record a session's input log with per-tick hashes, replay it on another build or machine,
  diff the hash logs (Factorio FFF #47; our `tools/check-determinism.ps1` already does it across worker counts).
- **Cross-machine CI:** GitHub gives free hosted arm64 runners to public repositories: `ubuntu-24.04-arm` (January
  2025) and `windows-11-arm` (April 2025), both in public preview at the time
  (https://github.blog/changelog/2025-01-16-linux-arm64-hosted-runners-now-available-for-free-in-public-repositories-public-preview/,
  https://github.blog/changelog/2025-04-14-windows-arm64-hosted-runners-now-available-in-public-preview/; current
  status in 2026 unverified). The repo has no CI and its visibility is unknown (brief), so this needs the repo public or
  paid runners.
- **Mismatch triage:** per-subsystem hash vectors (note 5.4), a desync report with both sides' state (Factorio's
  desynced and reference levels), and a tagged, human-readable dump of the first differing structure.

## 12. Worked example: bandwidth at our scale

All figures (est.) from the profiles in note 2.2; Mbit/s = bodies × bytes × Hz × 8 / 10⁶, per receiving client,
payload only.

### 12.1 State sync, every awake body every update

Bytes per update per client:

| Awake bodies | Interp. delta (7.5 B) | State-sync delta (12 B) | State-sync absolute (24 B) |
|---|---|---|---|
| 100 | 0.75 KB | 1.2 KB | 2.4 KB |
| 1500 | 11 KB | 18 KB | 36 KB |
| 5000 | 38 KB | 60 KB | 120 KB |

Mbit/s per client at 10 / 20 / 30 Hz:

| Awake bodies | Interp. delta | State-sync delta | State-sync absolute |
|---|---|---|---|
| 100 | 0.06 / 0.12 / 0.18 | 0.10 / 0.19 / 0.29 | 0.19 / 0.38 / 0.58 |
| 1500 | 0.9 / 1.8 / 2.7 | 1.4 / 2.9 / 4.3 | 2.9 / 5.8 / 8.6 |
| 5000 | 3.0 / 6.0 / 9.0 | 4.8 / 9.6 / 14.4 | 9.6 / 19.2 / 28.8 |

Packet overhead: 1500 bodies at 12 B and 20 Hz fill 15 packets of 1232 B per update, 300 packets/s; at about 60 B of
IP, UDP, header and tag per packet that is 18 KB/s, 0.14 Mbit/s (5%).

Host upstream is (players − 1) × per-client. For 1500 awake bodies, state-sync delta at 20 Hz (2.9 Mbit/s a client):
4 players 8.6, 8 players 20, 12 players 32 Mbit/s. At 12 players that is half the US median uplink and 84% of
Germany's: sending everything to everyone does not fit player hosts.

### 12.2 The Teardown hybrid

Teardown caps bodies at about 1 Mbit/s per client and lets the priority queue decide who gets updated. At our scale:

- Host upstream: 3 (4 players), 7 (8 players), 11 Mbit/s (12 players), plus commands. 11 Mbit/s is 17% of the US
  median uplink, 29% of Germany's (Ookla, August 2026): fine for a median host, too much for a low-tier one, and the cap
  can be lowered per session at the price of more snapping.
- What 1 Mbit/s buys: 125 KB/s ÷ 12 B ≈ 10,400 body updates a second. Spread evenly over 1500 awake bodies, 7 Hz each.
  Prioritised: 300 bodies near or in view of that player at 20 Hz (72 KB/s) and the other 1200 at about 3.7 Hz
  (53 KB/s). Ballistic keyframes (note 2.4) cut the airborne share further, and frozen rubble costs nothing after one
  reliable "frozen at this transform" command (24 B).
- Commands: a 100-piece blast is about 40 B as a seed or 15–25 KB as quantised geometry (note 4.3); stress breaks are
  lists of bond indices (2–4 B each); splits follow deterministically from the breaks on each side. A barrage of 20
  big blasts a minute with shipped geometry averages about 50–70 kbit/s, in bursts.
- Client uplink: inputs, the avatar's pose, and the few bodies it has authority over, well under 0.1 Mbit/s.
- Client CPU: local physics only near the client; the host carries the world.

### 12.3 Lockstep, for comparison

Inputs: about 8 B per player per tick after delta coding (tool rays when used, controls on change), plus the avatar's
pose if the character lives outside the deterministic world (about 14 B at 30 Hz). The host relays to each client
the other players' inputs: at 12 players about 11 × 0.9 KB/s = 79 kbit/s per client, 0.87 Mbit/s host upstream, plus
per-packet overhead (60 packets/s × 60 B × 11 clients ≈ 0.3 Mbit/s; bundle two ticks per packet to halve it); 4 players
about 0.1 Mbit/s. Hashes add 8 B per peer per tick. Lockstep needs about a tenth of the hybrid's upstream at 12 players;
its costs are elsewhere: every peer simulates everything (note 5.2, note 8), fracture included, bit-exactly on x64,
ARM64 and emulators, and joins need a bit-exact snapshot or a canonical rebuild (note 7.2).

## 13. Verdicts on the brief's hypotheses (netcode evidence only)

- **H2 (lockstep right for 4 players, acceptable at 12; local character predicted outside the world; hybrid loses on
  host upstream at 12):** partly confirmed.
  - The local-character part has direct precedent: Factorio's latency state for players (2015) and for cars (2024,
    500 ms hidden).
  - Lockstep at 12 players is acceptable only if server-timed (hard tolerance, input substitution), so a lagging peer
    cannot stall the rest.
  - The binding constraint at 12 is CPU, not network: every peer pays for everyone's activity and every fracture spike.
  - The hybrid does need about 10× lockstep's upstream at 12 players (11 vs about 1 Mbit/s), but 11 Mbit/s fits 2026
    median uplinks (38–65 Mbit/s in DE, AU, UK, US). "Loses" holds only for the low tail of hosts, and the hybrid
    degrades by snapping rather than stalling.
- **H5 (a serialisable, command-logged world with per-structure hashes is the most valuable outcome):** confirmed from the
  netcode side. Every model needs one serialise/rebuild primitive: joins, reconnects, saves, desync repair, and (for
  rollback) every tick. Lockstep additionally needs it bit-exact including Box3D's hidden state, or a canonical rebuild
  that all peers perform. The hybrid needs only our state (geometry and transforms), not Box3D internals. Per-subsystem
  and per-structure hashes are what turn a desync into a bug report (Quantum dumps, Factorio reports).
- **H3 (fixed point everywhere solves what flags and tests solve):** one relevant finding. In the hybrid, shipping
  host-quantised geometry removes the need for cross-platform fracture determinism on clients altogether, because
  convex low-poly pieces are cheap to send (about 150–250 B each), unlike voxel volumes.

## 14. Sources

Primary unless marked; dates as published or last updated; all read 2026-09-29.

- Glenn Fiedler, Gaffer On Games: Deterministic Lockstep (2014-11-29) https://gafferongames.com/post/deterministic_lockstep/;
  Snapshot Interpolation (2014-11-30) https://gafferongames.com/post/snapshot_interpolation/; Snapshot Compression
  (2015-01-04) https://gafferongames.com/post/snapshot_compression/; State Synchronization (2015-01-05)
  https://gafferongames.com/post/state_synchronization/; Packet Fragmentation and Reassembly (2016-09-06)
  https://gafferongames.com/post/packet_fragmentation_and_reassembly/; Sending Large Blocks of Data (2016-09-12)
  https://gafferongames.com/post/sending_large_blocks_of_data/; Reliable Ordered Messages (2016-09-15)
  https://gafferongames.com/post/reliable_ordered_messages/; Networked Physics in Virtual Reality (2018-02-22)
  https://gafferongames.com/post/networked_physics_in_virtual_reality/. GDC 2015 talk (covers the 2014–15 articles):
  https://www.gdcvault.com/play/1022195/Physics-for-Game-Programmers-Networking.
- Jared Cone, It IS Rocket Science! (GDC 2018), slides: https://media.gdcvault.com/gdc2018/presentations/Cone_Jared_It_Is_Rocket.pdf.
- Tim Ford, Overwatch Gameplay Architecture and Netcode (GDC 2017): https://www.gdcvault.com/play/1024001/-Overwatch-Gameplay-Architecture-and;
  numbers taken from a secondary summary, https://edgegap.com/blog/game-backend-deep-dive-overwatch-2016-netcode-architecture-rollback
  (verify against the talk video).
- Mark Frohnmayer and Tim Gift, The TRIBES Engine Networking Model (Dynamix, about 1999–2000):
  https://www.gamedevs.org/uploads/tribes-networking-model.pdf.
- Paul Bettner and Mark Terrano, 1500 Archers on a 28.8 (2001):
  https://www.gamedeveloper.com/programming/1500-archers-on-a-28-8-network-programming-in-age-of-empires-and-beyond.
- GGPO: Developer Guide https://github.com/pond3r/ggpo/blob/master/doc/DeveloperGuide.md; `src/lib/ggpo/sync.h`,
  `timesync.h`, `timesync.cpp` in https://github.com/pond3r/ggpo.
- Photon Quantum 3 (docs updated 2026-06/07): Prediction Culling https://doc.photonengine.com/quantum/current/manual/prediction-culling;
  Config Files https://doc.photonengine.com/quantum/current/manual/config-files; DeterministicSessionConfig API
  https://doc-api.photonengine.com/en/quantum/current/class_photon_1_1_deterministic_1_1_deterministic_session_config.html;
  Reconnecting https://doc.photonengine.com/quantum/current/manual/game-session/reconnecting; Starting From Snapshot
  https://doc.photonengine.com/quantum/current/manual/game-session/starting-from-snapshot; Commands
  https://doc.photonengine.com/quantum/current/manual/commands; FAQ https://doc.photonengine.com/quantum/current/getting-started/faq;
  Mispredictions and Entity Views https://doc.photonengine.com/quantum/current/concepts-and-patterns/mispredictions-and-entity-views.
- Photon Fusion 2 Physics (updated 2026-08): https://doc.photonengine.com/fusion/v2/manual/physics/physics-overview.
  Host migration page blocked (unverified).
- Unity Netcode for Entities 1.4: https://docs.unity3d.com/Packages/com.unity.netcode@1.4/manual/optimizations.html,
  .../ghost-snapshots.html, .../compression.html, .../physics.html, .../ghost-spawning.html. Netcode for GameObjects
  distributed authority: https://docs.unity3d.com/Packages/com.unity.netcode.gameobjects@2.4/manual/terms-concepts/distributed-authority.html.
- Unreal Engine, Networked Physics Overview: https://dev.epicgames.com/documentation/en-us/unreal-engine/networked-physics-overview
  (resimulation details unverified).
- Valve, Source Multiplayer Networking: https://developer.valvesoftware.com/wiki/Source_Multiplayer_Networking (via the
  Internet Archive's 2024 copy; the live page returned 403).
- Factorio: FFF #47 CRC fun https://www.factorio.com/blog/post/fff-47; FFF #83 Hide the latency (2015-04-24)
  https://www.factorio.com/blog/post/fff-83; FFF #149 Deep down in multiplayer https://www.factorio.com/blog/post/fff-149;
  FFF #188 https://factorio.com/blog/post/fff-188; FFF #302 megapacket https://www.factorio.com/blog/post/fff-302;
  FFF #412 car latency driving (2024-05-24) https://factorio.com/blog/post/fff-412; FFF #415 (2024-06-14)
  https://www.factorio.com/blog/post/fff-415; wiki Desynchronization https://wiki.factorio.com/Desynchronization.
- Forrest Smith, The Tech of Planetary Annihilation: ChronoCam:
  https://www.forrestthewoods.com/blog/tech_of_planetary_annihilation_chrono_cam/.
- Dennis Gustafsson, Teardown multiplayer (2026-03-13): https://blog.voxagon.se/2026/03/13/teardown-multiplayer.html.
- Roblox: Server authority model and techniques (updated 2026-09-30):
  https://create.roblox.com/docs/en-us/projects/server-authority, https://create.roblox.com/docs/en-us/projects/server-authority/techniques;
  Network ownership https://create.roblox.com/docs/physics/network-ownership.
- Valve: Steam Datagram Relay https://partner.steamgames.com/doc/features/multiplayer/steamdatagramrelay;
  GameNetworkingSockets https://github.com/ValveSoftware/GameNetworkingSockets (`include/steam/steamnetworkingtypes.h`,
  `src/steamnetworkingsockets/steamnetworkingsockets_internal.h`).
- Tailscale, How NAT traversal works (2020-08-21): https://tailscale.com/blog/how-nat-traversal-works.
- Ookla Speedtest Global Index, August 2026: https://www.speedtest.net/global-index.
- GitHub changelog, arm64 hosted runners (2025-01-16, 2025-04-14): links in note 11.
- Not reached this session (GDC Vault returned 504): David Aldridge, I Shot You First: Networking the Gameplay of Halo:
  Reach (GDC 2011), https://www.gdcvault.com/play/1014345/I-Shot-You-First-Networking. Its per-client prioritisation
  by distance and view is widely cited, but I could not verify its numbers.
- Repo: docs/determinism-rules.md; docs/roadmap.md sections 7, 12, 13; docs/perf-log.md:446-454; include/lpf/lpf.h:598-602;
  extern/box3d/src/world_snapshot.c (header lines 1–40).
