# Goals: the vision, the feel, and what the engine owes its games

What the engine is for, written down so that reviewers (and agents new to the repo) judge changes against it. The
outcome catalogue ([catalogue.md](catalogue.md)) pins what it asks for; the performance review's taste reviewer judges
before-and-after footage against "Feel". Gathered from the owner's notes and our discussions (2026-09-27 to 30).

The games themselves are designed in a private sibling repository, `lowpoly-games` (not open source). If it is checked
out at `../lowpoly-games`, its `docs/games.md` has their pitches; this file describes them only by what they ask of
the engine.

## North star

Agreed with the owner on 2026-09-30. A world made entirely of things that break, where modern compute goes into how
many things are simulated and what they do rather than how they are drawn (Astro Bot's lesson: a classic kind of game
built from a vast number of objects), and where the simulation is a pure function of the players' inputs. Noita is the
touchstone, in 3D: every detail can matter to play, down to a crack in a pane of glass.

- **Everything is physical and breaks where it is weak.** Debris is real: it has physics, lingers as rubble and leaves
  a mess.
- **Bodies are systems, not hit points** (see "Bodies taken down by impairment" below): damage impairs, and bodies
  adapt on their own.
- **Big events are the point,** and far-off ones still happen on time.
- **Shared by default.** Solo and co-op run the same simulation; in co-op every machine computes the same collapse bit
  for bit, so multiplayer is mostly moving inputs around. One command log and one snapshot serve co-op, saves, late
  join, host migration, desync repair, persistence, replays and debugging; a desync in the field becomes a failing
  headless test.
- **Gameplay over spectacle.** Detail goes into the simulation first. A cosmetic layer that never feeds back (dust,
  sparks) may be nondeterministic and scale with each machine, but it is not a priority.
- **The GPU inside the simulation.** The ambition is deterministic rigid-body physics on the GPU, bit-exact across
  vendors, so that old discrete GPUs carry the mass of the simulation while the CPU does the branchy work (fracture,
  stress, links, rigs, supply). Nobody has shipped this; we mean to try. Simulation code on the GPU always has a
  bit-identical CPU twin, because headless agents and tests have no GPU.
- **Persistent where a game wants it.** Zones are the unit of simulation, streaming, persistence and snapshots: a
  baked base (authored plus settled changes) under a live layer. A piece lives awake, then asleep, then frozen (static,
  woken when disturbed), then baked into its zone's geometry, and may be forgotten. The engine offers the mechanisms
  (freeze, bake, wake, delete, restore a region to its authored state, budgets); games choose the policy (rain washes
  the mess away, NPCs tidy up, animals eat, crews rebuild).
- **Plausible over causal,** never different between machines.
- **Built by agents:** every behaviour a headless test, every bug a replay.

First targets, to be pinned by measurement:
- **Floor:** a GTX 1060-class PC with an old quad-core (a game may raise it): 4-player co-op in a destructible town
  block at 60 Hz, a building collapse leaving thousands of physical pieces, about 5,000 awake bodies.
- **Dream:** an RTX 2080-class PC: a whole district under barrage, tens of thousands of physical bodies.
- **Always:** solo equals co-op; any session replays exactly; a zone left and revisited is as it was left; joining a
  game in progress takes seconds.

Not goals: a general-purpose physics engine, photorealism, competitive play or anti-cheat, large lobbies,
integrated-GPU toasters (a game may still support them), physical exactness.

Open, to research later:
- **Wake storms:** a grenade in a warehouse of sleeping props; something more graceful than a per-step cap on wakes.
- **The "10,000 cabbages" problem:** zones full of props that must stay dynamic. A first idea: identity is opt-in, so
  anonymous rubble bakes into its zone and only what a game tags persists as a body.
- **Simulation scale per session:** in lockstep the weakest peer bounds the session's simulation; only the cosmetic
  layer scales per machine.
- **Adaptive locomotion generated for a newly maimed body:** UniMate (https://arxiv.org/abs/2609.05415,
  https://github.com/Friedrich-M/UniMate), gait repertoires and learned policies; queued as a deep dive in
  [roadmap.md](roadmap.md) ("Research queue").

## Priorities

1. **Agent-friendliness:** code that agents can build, test, verify and prune headlessly, kept small and plain (the
   whole repo well under about 600k tokens, so an agent can hold most of it).
2. **Performance:** every shortcut we can find, in every operation, down to the hardware floor above. Big events
   (buildings falling onto buildings, a barrage, a race full of wrecks, crowded city scenes, hordes, masses of explosive
   debris) are the point, not an edge case. Performance outranks simplicity: complexity that buys speed is worth it,
   and no review trades speed for size.
3. **Everything else.**

Determinism is not negotiable: same inputs, bit-identical state at any worker count, because lockstep multiplayer
depends on it ([determinism-rules.md](determinism-rules.md)).

## The games it serves

Described by what they ask of the engine, which says what the engine needs better than a list of features can.

### A rigging-race game

The players rig a track with tools, cranes and trucks while AI drivers race.

What it asks of the engine:
- vehicles that drive well, crash plausibly and lose their function with their parts (tyres, engine, fuel tank), with
  AI drivers on top;
- structures that come down where they are weak, cranes, ropes and winches, and loads that are cheap to move about;
- a whole racetrack in a detailed area: large maps whose far parts cost little, while a trap set a minute ahead still
  goes off on time;
- how much was destroyed, measurable;
- multiplayer rigging, so determinism.

### A co-op hauling game with reactive materials

What it asks of the engine:
- materials whose properties matter to play (reactivity, flammability, fragility, weight), not just to fracture;
- contact events well below damage (a jostle, a tilt, a tumble) and liquids in containers (pools are the first step);
- detonators, volatile flasks, per-part gravity scale ("fairy dust");
- assemblies a character carries or pulls (links and loads on a moving body), and a character controller;
- terrain and caves that break.

### Bodies taken down by impairment, not hit points

A body is stopped only by a drained pool (bleeding out, running out of power), a destroyed vital part, or a knockout.
Short of that it is impaired and adapts on its own: a lost leg, a jammed joint, an unfed limb.

What it asks of the engine:
- rigs that adapt their locomotion and attacks to whatever is left (done for a hexapod; bipeds and humans next);
- vital parts, pools and channels as the body's systems (a reactor, a reservoir, a computer in the mech today);
- knockout: an impulse or acceleration at a vital that impairs control for a while (not built);
- dents that restrict joints (milestone 20);
- the player as one of these bodies too.

### First-person co-op in destructible buildings

4 to 8 players, sometimes about 12, with first-person characters in buildings that come apart around them.

### Others

A 4-player co-op horde game, and smaller projects to get movement and physics feel right piecemeal. The engine is also
worth having as a destruction tech demo on its own: polish punches above its weight on social media.

## Multiplayer targets

Set by the owner on 2026-09-29, for milestone 7's choice of a multiplayer model:
- **Co-op, player-hosted.** 4 players is the main case (the hauling game, the horde game); first-person co-op games
  take 4 to 8, sometimes about 12 (12 matters somewhat, much less than 4). Large-scale multiplayer is nice to have, not
  a priority. Hacked clients and competitive integrity are not concerns.
- **In the rigging-race game the players rig and the AI races:** no player's own racing car needs low latency. The
  players' tools, cranes and trucks do.
- **Machines that play together:** x64 is required. ARM64 is nice to have; failing native ARM64, an x64 build under
  emulation on ARM64 (box64, Rosetta 2, Prism, FEX) should stay in sync. WebAssembly is nice to have, low priority.
- **Everyone can play solo,** so the minimum spec runs the whole simulation: there are no thin clients that only draw
  what a host sends.

## Feel

- **Plausible over causal.** Physics may work differently from reality, even inconsistently, as long as what the
  player sees looks plausible. A lazy resolve during chaos with the exact pass snuck in once things calm down is fine;
  a collapse that comes a beat late is fine. A result that differs between runs is not.
- **Weight and material.** Stone feels like stone, apart from hardwood, dry wood, plaster or foam. Glass is hard and
  fragile. A stone tower that will not fall, its blocks sliding off each other, feels sticky and foamy: wrong.
- **Breaking in the right place.** Buildings collapse under their own weight. Structures know where they are weakest
  (a keystone, a loaded support). A brick wall is one solid wall for most physics but breaks along its mortar. A
  building that lost a support creaks, then either slumps into a new stable pose and falls quiet, or gives out.
- **The fracture look.** Jagged low-poly spikes and splintered wood; shattered things break into simple polyhedra.
  Fragments are carved from the object and coloured like it, never grey cubes. Procedural cut-face colours and radial
  glass are core polish. Coarse holes are fine, even right for the style. Open: whether to bring back finer fragments
  for a smoother gradient, or keep only big chunks (and glass shards), Valheim-style; decided in the art pass.
- **Mess.** Destruction should leave the place dirty: debris lingers as rubble, and later, surfaces darken where it
  lands.
- **Vehicles.** They crumple rather than shatter, and have innards: a shot engine stops the car, a shattered tank
  leaks or explodes.
- **Creatures and machines.** They adapt on their own: hobble on a stump, drop onto a belly and crawl, hit softer with
  a weak limb, give up a strike that would tip them over, bleed out and slump.
- **Scale.** Big maps, big events, and things left running far away that still happen.
- **Look.** Low-poly like Valheim, Flyknight or Old School RuneScape, with cute characters. Art polish is essential
  for anything public-facing, and deliberately last.

## What the engine owes the games

| capability | for | state |
|---|---|---|
| fracture, debris tiers, rubble, budgets | all | done |
| structures that break where they are weak (stress, masonry, toppling) | all | done |
| links: ropes, hinges, welds, winches, motors, cranes | racing, hauling | done |
| vehicles with innards (wheels, engine, fuel, steering) | racing | done; AI drivers belong to the game |
| channels, supply and pools (fuel, power, hydraulics, blood) | all | done; liquids sloshing in containers not |
| rigs that adapt to damage (lost legs, pegs, weak and limp legs, crawling, strikes, grabs, bones) | impairment, racing | done for the hexapod; bipeds and characters not |
| vital points and knockout | impairment | vitals via part systems; knockout not |
| gentle contact events (jostle, tilt, tumble) and reactions | hauling | detonators and fuses done; the rest not |
| dents (crumple, joints restricted by crumpled armour) | racing, impairment | milestone 20 |
| a character controller (the player as a destructible body) | all | not started |
| large maps: zones, far events still on time | racing, hauling | milestone 18 |
| multiplayer | all | lockstep, chosen in milestone 7; a first co-op in milestone 10, networking in 16 |
| fire, smoke, flammability | hauling, racing | art pass and later |
| cutting and chopping (clean slices stopped by what is too hard, axes that notch) | impairment, all | milestone 11b |

## Outcomes to pin

The outcomes we like are pinned in [catalogue.md](catalogue.md): each behaviour in words, the test (or the reference
frames) that pins it, and its tolerance. It is the contract a redesign or a new physics core must meet; the bench
hashes pin exact behaviour only for refactors that mean to change nothing.
