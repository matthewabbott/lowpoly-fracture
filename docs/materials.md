# Materials

A catalog of what a material is, for the engine now and for the games later. Values are starting points to be
tuned by feel: stone should feel like stone, hardwood like hardwood, dry wood like kindling, plaster and foam like
nothing much. The engine table lives in `src/world.c` (`lp_materials`); the struct is `lpMaterialDef` in
`include/lpf/lpf.h`.

## Properties the engine uses today

| group | property | meaning |
|---|---|---|
| mass and contact | `density` | kg/m³; drives mass, inertia, momentum in collisions |
| | `friction`, `restitution` | Box3D contact material |
| strength | `bondStrength` | J/m² that breaks a bond between two pieces (how easily chunks come apart) |
| | `fractureEnergy` | J/m² that refractures a piece (how easily a chunk itself shatters) |
| | `tensileStrength`, `compressiveStrength`, `shearStrength` | Pa limits of the solid material in the stress solve |
| joints | `lpPartDef.joint` | how a part meets its neighbours: mortar, dry, nails or solid (table in `world.c`) |
| | `breakable` | false for ground and metal today |
| fracture shape | `pattern` | impact Voronoi, grain Voronoi (wood), radial (glass) |
| | `grainStretch` | wood: splinters this many times longer along the grain |
| | `fragmentSize` | smallest fragments at the impact, metres |
| | `plateSize`, `maxCells` | how few, how big the remaining chunks are; cells per fracture |
| | `mergeSlack` | cells that stay merge while their hull is at most this much bigger (chunky ends) |
| tiers | `particleVolume`, `ghostVolume`, `lightVolume` | size thresholds for puff / ghost / light / full debris |
| look | `interiorColor`, `particleKind` | colour of fresh cuts; dust, chip, splinter, leaf or glint particles |

## Properties to add (proposed)

Each is listed with the problem it solves. Add one when a scene or game needs it, not before.

**Hardness vs toughness.** Today one number per level (`fractureEnergy`, `bondStrength`) decides both whether a hit
does anything and how much it breaks. Real materials separate the two:

- `hardness`: energy density below which a hit does nothing at all: no chips, no bond damage. High for glass,
  stone and metal; near zero for plaster and foam.
- `toughness`: how hard it is for damage to spread once it starts. It scales the break radius and the fragment
  count. Low toughness means a hit that gets through shatters the whole piece.
- Glass is **hard and fragile**: high hardness (small knocks bounce off), very low toughness (a real hit shatters
  the pane). Hardwood is medium hardness and high toughness (dents, splinters locally, rarely snaps). Stone is high
  hardness and medium toughness (chips at the rim, breaks in big pieces). Foam and plaster are low and low: they
  crumble.

**Weight feel.** How heavy a thing *feels* is not only density:

- `massScale`: gameplay mass multiplier, for stone that barely moves when shoved even at modest size.
- `inertiaScale`: tumbling resistance; a slow, heavy roll for stone, quick flips for planks and foam.
- `linearDamping`, `angularDamping`: air drag; foam and leaves drift, stone plummets.
- The rule from the design talks: small rocks still plummet like rock. Cheap tiers change what debris can push,
  never how fast it falls.

**Deformation.**

- `ductility`: bends and crumples instead of breaking (metal sheet, car panels). Needs the vehicle crumple lattice
  (roadmap item 3).
- `brittleness`: whether a bend turns into a snap past a limit (cast iron vs steel).

**Sound and effects.**

- `soundClass`: thud, crack, clink, shatter, crunch.
- `dustAmount`, `dustColor`: how much fine dust a hit throws up (plaster and dry brick a lot, glass none, wood
  sawdust).

**Alchemy-facing** (the courier game: volatile reagents in contraptions):

- `flammability`: ignition temperature, burn rate, fuel content (dry wood burns fast, hardwood slowly, stone never).
- `heatConductivity`, `heatCapacity`: how fast heat spreads through a crate or a flask.
- `meltingPoint` / `softeningPoint`: ice melts, wax softens, glass sags near a fire.
- `porosity`, `absorbency`: soaks up spilled reagents (wood, plaster, cloth) or sheds them (glass, metal).
- `buoyancy`: follows from density vs the liquid; foam and dry wood float.
- `electricalConductivity`: metal carries current (sparks near a volatile reagent).
- `reactivity` tags: acid-soluble (stone, metal), magnetic (iron), volatile (reagents), inert (glass).

## Material table (starting values and intended feel)

"Now" rows are in the engine; the rest are candidates. Strengths are relative (1 = plaster), and hardness and
toughness are 0 to 5.

| material | density | strength (rel.) | hardness | toughness | pattern | feel |
|---|---|---|---|---|---|---|
| stone (now) | 2400 | 7 | 4 | 3 | impact | heavy, slow to tumble; chips at the rim, big blocks |
| brick (now) | 1900 | 2.5 | 3 | 2 | impact | chunky, dusty; walls open in rough holes |
| concrete (now) | 2400 | 8 | 4 | 3 | impact | like stone, greyer dust; rebar later (ductile core) |
| plaster (now) | 1200 | 1 | 1 | 1 | impact | crumbles into dust and flakes |
| hardwood | 750 | 2.5 | 2 | 5 | grain | dents, long tough splinters, rarely snaps clean |
| wood / dry softwood (now) | 600 | 1.7 | 1 | 3 | grain | splinters, snaps into two jagged ends |
| rotten wood | 400 | 0.5 | 0 | 1 | grain | punky, crumbles, lots of dust |
| glass (now) | 2500 | 0.25 | 4 | 0 | radial | hard and fragile: shrugs off knocks, then shatters, glints |
| metal (now, unbreakable) | 7800 | n/a | 5 | 5 | none | dents and crumples (ductile), never shatters |
| foam | 30 | 0.1 | 0 | 1 | impact | light, floaty, bounces, shoved by anything |
| ice | 920 | 0.8 | 2 | 1 | radial | slippery (low friction), shatters, melts near heat |
| ceramic | 2000 | 0.6 | 3 | 0 | radial | pots and flasks: clinks, then shatters into shards |
| foliage (now) | 150 | 0.25 | 0 | 1 | impact | leaves and twigs: puffs of leaves |

Glass flasks in the sandbox are the first alchemy object: a detonator part on a glass body.
