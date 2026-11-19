# Three-phalanx hand-prop: design study before CAD (2026-11-13)

Ask (Forest, by voice): a new kind of claw. The Snap-Digit is off-centre, its grip is on or off, and it has no malleability. Wanted: the four-finger hand (`hand-prop.md`, `key-hub.md`) whose fingers rotate on the base to sit closer to one another, and unfurl **with joint-by-joint control**; fully extended it must still be a propeller that flies the robot. And: study the alternatives and the components, with visuals, **before** any CAD.
Status: **study and blueprints only. No CAD, nothing simulated.** Forest's choices so far (2026-11-13): "love the plan"; individual control, clarified as **different fingers moving independently at the same time**, which replaced the latching key hub with park and dock (one servo per joint); then "optimize the design before rendering, give me blueprints of the exact design". The current design is the "Optimised design and blueprints" section; `hand3_selected.png` (key hub) is superseded. CAD has not been started: it waits for Forest's agreement on the blueprints.
Code: `claude/code/hand3_loads.py` (the numbers → `hand3_loads.json`), `hand3_study.py` (the eight sheets → `hand3_study_1..8.png`; needs matplotlib), `hand3_selected.py` (the selected design on one sheet). The sheets were sent in the chat; re-create them with `python3 hand3_loads.py && python3 hand3_study.py`.

## Selector revisited: fingers must move at the same time (2026-11-13, later)
Forest on seeing the selected sheet: "you didn't choose the one with individual control". Asked which he meant, he chose **different fingers moving independently at the same time**, not one joint after another. That rules out every one-pusher key hub (decision 6, options A-C). Independent and simultaneous means one actuator per joint, and since nothing powered can ride on the rotor, the actuators sit on the arm tip. Sheet: `hand3_selector2.py` → `hand3_selector2.png`. All masses and times below are estimates.

| Option | Actuators | Independent joints at once | Full grasp | Mass per hand | Problem |
|---|---|---|---|---|---|
| **A park and dock, one servo per joint (proposed)** | 12 + 2 spread: Feetech STS3032 (20 g, 55 N at a 6 mm spool) | all 12 | about 0.5 s, continuous, force measured from servo load | about 335 g | weight at the arm tip (key hub: about 35 g); rotor needs a parking detent; rods must clear the cage in flight |
| B park and dock, two servos per finger | 8 + 2 | base alone; middle + tip on one cord | about 0.5 s | about 250 g | not joint by joint |
| C three swashplates | 9 | 9 of 12: one finger alone drags its neighbours half as far | fast | about 260 g | not individual |
| D one finger at a time, three pushers | 3 | 3 (one finger) | about 4 s | about 95 g | fingers take turns |
| E key hub, one pusher | 1 | 1 | about 20 s | about 35 g | turned down |

- **How A works:** the hand is only used with the rotor stopped, so the rotor parks at a fixed angle. Each cord ends in a plain sliding follower on the cage (no latch teeth). Fourteen stationary servos on a carrier round the arm tip each push one follower through a rod; retracted, the rods leave a gap to the spinning cage. The TPU links stay as the series elastic element.
- **Consequences to check before CAD:** about 300 g more at the tip of each hand arm (the XL pod is 156 g and the legged landing already reaches 1.00 × stall); 14 more bus servos per hand on the 7.4 V rail; the parking detent; packaging 14 STS3032 (23.2 × 12.1 × 28.5 mm) round a Ø32 tip bay.
- **Status:** Forest: "idk optimize the design before rendering, give me blueprints of the exact design" and "remember to consider different design plans too". Done in the next section; plan A was kept.

## Optimised design and blueprints (2026-11-13, after "optimize the design … give me blueprints … consider different design plans too")
Code: `claude/code/hand3_design.py` (**single source of truth for the dimensions**, the sizing and the plan comparison → `hand3_design.json`), `hand3_blueprints.py` (six dimensioned sheets `hand3_bp_0..5.png`, sent in the chat). Still **no CAD solids and no fit check**: these are design values.

**Whole-hand plans compared** (sheet 0; all estimates, per hand):

| Plan | Servos | Mass | Pack | Blade plane from the spigot | Verdict |
|---|---|---|---|---|---|
| **A drum under the motor, 2 tiers, shafts radial** | 14 | 530 g (+374 g on the XL pod) | Ø102 × 54, 38 mm under the rotor, 16 % of the disc | 92 mm | **picked** |
| B flat pancake tiers, shafts tangential | 14 | 539 g | Ø119 × 57 | 95 mm | does not pack (needs 4 tiers) |
| C ring round the motor, 1 tier | 14 | 534 g | Ø137 × 26, 12 mm under the rotor, 29 % of the disc | 42 mm | too much disc blocked |
| D drum, 2 servos per finger | 10 | 437 g | Ø102 × 54 | 92 mm | not joint by joint |
| E key hub, one pusher | 1 | 226 g | none | 42 mm | one joint at a time |

**What the optimisation changed against the study sheets:**
- **Motor:** BrotherHobby SE 2816 810KV (Ø34 × 46.2 mm, 98.5 g, 19 × 19 M3). Its maker test on an HQ 10×5×3 gives 22.4 N at 633 W, 28.3 N at 895 W and 34.7 N at 1,303 W, so a real 10 in prop already needs about 815 W for 26.5 N; the 3110 and 2810 candidates would be at their limits. No hover-power gain over the XL claw is expected.
- **Servos sized per joint** for 5 N at the fingertip with the finger straight (peak) and 2 N held, efficiency 0.62: base 68.8 N of cord and middle 51.7 N → Feetech STS3032 on a Ø5 spool (margins 1.59 and 2.12); tip 20.7 N → Feetech SCS0009 (13.2 g, margin 2.7); spread → STS3032, 0.35 N·m per blade. Ten STS3032 + four SCS0009 = 252.8 g. The SCS0009 misses the middle joint by margin (1.05).
- **Lever arms** 6.0 / 4.5 / 3.5 mm (base / middle / tip), set by a saddle bridge on each hinge barrel; **TPU links** 3.0 / 2.6 / 1.7 mm square × 12 mm for 3 mm of stretch at peak force.
- **Knuckles:** Ø3 × 6 dowel stubs in 4 × 3 bushings at K1 and K2, Ø2 × 5 in 3 × 2 at K3; widths 15.0 / 15.0 / 12.4 mm; pull at proof speed 1,135 / 826 / 424 N; bearing 55 / 40 / 35 MPa on the cheeks (K1 is the part to pull-test; fallback: aluminium root grip); flight lock 3.0-3.1 ×.
- **Extensor:** a Ø1 elastic cord in a back groove holds the fingers straight when the rotor is stopped.
- **Hub:** plain followers (no latch teeth) at r 25.5 with fins to r 31; 14 stationary claw bars at r 48.5 reach in over the fins, 1 mm clear in flight. Rotor parks on a Hall switch + magnet index; the two spread claws centre it.
- **Stand:** the hand stands on a TPU heel over the prop nut, 110 mm from the spigot, with the fingers flat.
- **Wrap check:** Ø32 handle 81° / 75° (middle / tip), Ø38 73° / 68°, Ø70 ball 47° / 43°; a Ø16 rod needs 110° and is pinched instead.

**Open before CAD:** the 530 g hand is 3.4 × the XL pod: arm servo loads, walk and legged landing must be re-run (hand arms 2 and 6 only); thrust of the finger blades (**now calculated, next section: the finger as drawn gives only 7-8 N**); motor can length and servo shaft positions (measure); rotor parking with a drone ESC; servo bus current (18 A if all 14 stall).

## Thrust of the finger blades (2026-11-13, "calculate the thrust of the finger blades immediately")
Code: `claude/code/hand3_thrust.py` → `hand3_thrust.json`, `hand3_thrust.png` (sent in the chat). **Estimate:** blade-element momentum theory in hover with Prandtl tip loss; section polars from NeuralFoil (an XFOIL surrogate; `pip install neuralfoil`) and, for the bluff as-drawn section, Hoerner's handbook drag. No CFD, no spin test. The full run takes about 7 minutes; `python3 hand3_thrust.py chart` redraws from the json.
- **Method check:** on a moulded 10×5×3 (HQ geometry guessed) the method gives 23.0 / 26.6 / 34.0 N where the maker's table has 22.4 / 28.3 / 34.7 N. Its shaft power is about 15 % low against the maker's full-throttle point (699 against about 820 W), so all torques are multiplied by 1.17.
- **Motor limit:** SE 2816 810KV on 6S, 55.2 A: 0.63 N·m up to about 12,400 rpm (from the maker's full-throttle point).
- **Included:** knuckle gaps (no lift over 2.4 mm at each knuckle), saddle bridges and barrels as drag, download of the inner slipstream on the Ø102 servo drum (0.4-1.1 N).
- **Result: the blueprint finger does not fly.** Its section is 36-43 % thick at a Reynolds number of about 100,000, where sections thicker than about 25 % make little lift and a lot of drag.

| Blade plan | Best pitch | Max net thrust at full throttle | At | Hover share 13.8 N | 26.5 N |
|---|---|---|---|---|---|
| A as drawn (18 → 12 mm chord, 2 mm pad proud, 36-43 % thick) | 20-28° | **7.0-7.6 N** | 7,700-8,300 rpm, 510-550 W shaft | no | no (would need 1.5-1.8 kW shaft at 11,680 rpm) |
| B same chord, airfoil shape, 1 mm flush pad (31-35 %) | 18-24° | 6.9-11.3 N | 8,800-11,400 rpm | no | no |
| **C trailing fairing: printed lip, chord 30 → 24 mm, 2 mm flush pad (22 %)** | 10-12° | **18.9-24.4 N** | 10,700-11,600 rpm, 710-770 W shaft | yes: 8,700-9,100 rpm, 410-550 W electrical, figure of merit 0.35-0.48 | no |

  The range in each row is low (transition forced at 5 % chord, lift × 0.9, drag × 1.3 + 0.004; for A: handbook separated flow) to high (clean, early transition; for A: as if it were a faired section of the same thickness).
- **For scale:** the hand weighs 5.2 N; the hover share is 13.8 N with two hands on the robot (11.23 kg); the XL claw-prop ceiling is 26.5 N. The C_T 0.14 assumed in the first study was wrong for plan A.
- **Stepped twist** (each phalanx printed at its own incidence) gains under 0.5 N: not worth it.
- **Plan C costs:** fingers 30 → 24 mm wide instead of 18 → 12 (spread and finger-to-finger clearances to re-check); about 2.6 g more per finger outboard of K1 (solid 1.8 mm lip), so the K1 pull at proof speed is 1.3-1.5 kN and the cheek bearing 62-73 MPa: the aluminium root grip becomes the baseline, or the lip is printed hollow. Pitch 10-12° at the root grip tilts the curl plane by the same angle (as `hand-prop.md`, 12°).
- **Status:** Forest: "Make it work". Done in the next section (plan D); plans A-C are kept in `hand3_thrust.py` for comparison ("A as drawn" is now "A first finger").

## Rev B: the thin airfoil finger (2026-11-13, "Make it work")
Code: `claude/code/hand3_blade_opt.py` (the search → `hand3_blade_opt.json`), `hand3_design.py` (updated: **still the single source of truth**), `hand3_thrust.py` (plan D = the design file's finger), `hand3_blueprints.py` (rev B sheets, re-sent). **Still estimates and design values: no CAD, no spin test, no grasp simulation of this hand.**
- **Why not plan C:** the two fingers of a pair lie side by side when the hand is spread, with their lag pins 28.3 mm apart, so a 30 mm chord cannot fit. The search therefore made the finger thinner rather than much wider.
- **Search** (`hand3_blade_opt.py`): chord, printed thickness, pad thickness, camber and pitch, scored on the low section data. Two things carry the result: section thickness of 19-20 % (instead of 36-43 %), and fairing the knuckle hardware. Unfaired, the saddle bridges and barrels on a thin blade eat about a quarter of the motor torque (18-22 N instead of 22-26 N).

| | First finger (plan A) | Rev B finger (plan D) |
|---|---|---|
| Chord, lag pin → tip | 18 → 12 mm | **22 → 18 mm** |
| Printed thickness | 4.5 → 3.2 mm | **3.4 → 2.4 mm** |
| Pad | 2 mm, standing proud, middle 60 % | **1 mm, inlaid flush, 15-75 % of the chord** |
| Section | rounded slab, 36-43 % thick | **airfoil, 3 % camber at 40 % chord, 19-20 % thick with the pad** |
| Pitch | not set | **12° at the root grip** (all knuckle pins pitch with it) |
| Knuckle hardware | bare | **printed teardrop over each saddle bridge, bullet ends on each barrel** |
| Max net thrust, full throttle | 7.0-7.6 N | **22.3-26.5 N** at 11,600-12,300 rpm, 770-820 W shaft |
| Hover share 13.8 N | no | 8,400-9,700 rpm, 360-500 W electrical, figure of merit 0.39-0.54 |

- **Margin:** 1.6-1.9 × the hover share; the high estimate just reaches the XL claw-prop's 26.5 N, the low one does not. With six XL pods at 26.5 N and two hands the robot (11.23 kg) has a thrust-to-weight of 1.85-1.92 on these numbers (ideal supply).
- **Design speed** is now the motor's knee on 6S, 12,400 rpm (was 11,680 from an assumed C_T); proof 14,260 rpm.
- **Knuckles re-run:** K1 widened to cheek 3.2 / lug 3.6 mm on Ø3 × 8 dowels (width 17.0 mm; the chord there is 21.6). Pull at proof 1,196 / 888 / 467 N; bearing 47 / 43 / 39 MPa (cheeks) and 42 / 35 / 28 MPa (lugs); pin shear 85 / 63 / 74 MPa; flight lock 3.2-3.3 ×.
- **Root grip in 6061 aluminium for flight** (16 × 17 × 6 mm, fork faces at 12°): the lag eye carries 1,328 N, 66 MPa on the Ø4 tube, too much for a printed eye (as `hand-prop.md` already said). Printed grips only for the bench finger.
- **Servo sizing unchanged** (lever arms 6.0 / 4.5 / 3.5 mm kept: lowering the middle and tip arms gained only 0.2 N of thrust).
- **Spread:** lag swing limited to 47° (was 49°): the pair toes in 2° with 2.8 mm between the finger tips; parallel they are 6.3 mm apart.
- **Wrap:** Ø32 handle 85° / 79°, Ø38 77° / 71°, Ø70 ball 49° / 44° (thinner fingers sit closer to the object); Ø16 rod still pinched.
- **Mass:** 533 g per hand (fingers lighter, aluminium grips heavier), centre of mass 45 mm from the spigot.
- **Open:** spin test (thrust is an estimate with a ±10 % band from section data alone); printing a 2.4 mm tip phalanx with Ø1 cord grooves; pad 1 mm instead of 2 mm in the grasp; grasp simulation with 22 → 18 mm fingers and 2° toe-in; everything in "Open before CAD" above.

## Rev C: first true solids (2026-11-13, "Build")
Code: `claude/code/hand3_cad.py` (**manifold3d solids, built from `hand3_design.py`**; `python3 hand3_cad.py` → `hand3_stl/`, 28 STLs, 1 s), `hand3_cad_checks.py` (→ `hand3_cad_checks.json`, 35 s), `hand3_cad_render.py` (→ `hand3_cad_1..4.png`; needs `render.py` one folder up). Renders and `hand3_stl.zip` were sent in the chat. **The blueprint sheets (`hand3_blueprints.py`) have NOT been re-issued for rev C**: they still draw the rev B knuckles, hub and pack; the solids are the current design.
Scope: fingers, knuckles, root grips, lag tubes, hub plates, cage, followers, heel, claw bars and the pack frame are true solids. **Servos are boxes from the makers' outline sizes (shaft assumed mid-length), so the pack frame is a first solid: servo mounts wait for measured servos. Spread-cord routing and screws are not modelled. No grasp simulation yet.**

**What the solid model changed (all now in `hand3_design.py`):**
- **K1 moved from r 30 to r 36** (base phalanx 30 mm instead of 36): the pitched fork, its cheeks and the base pulley must stay outside the hub plates when the finger swings 47°. Hub plates Ø51 (were Ø56), aluminium.
- **Cord drive at each knuckle = a pulley sector on the outer part** (radius = lever arm 6.0 / 4.5 / 3.5 mm, in the gap between the lugs), with a fairlead post on the inner part. The "saddle bridge" of the blueprints could not keep the lever arm: a cord between two guide points bowstrings. The other cords still cross on the pin axis, through a Ø3.4 hole in the sector. The base cord runs above the upper hub plate straight into its lag tube.
- **Back stop = a 1.8 mm tooth under each lug that seats on a boss of the inner part at −2°.** The body end faces of a 3 mm blade are cut away by the barrels and have a lever arm of only 1.6 mm.
- **Knuckles:** barrels Ø8.5 / 7.0 / 5.5, cheeks 3.2 / 3.6 / 2.6, lugs 3.6 / 4.0 / 3.0, widths 17.0 / 18.2 / 14.0, pins Ø3 × 8, Ø3 × 8, Ø2 × 6: sized on the net section of the rings round the bushings, which the blueprints had not checked. Lug and cheek webs run 7.5 mm into each phalanx, so the pads are shorter: 13 / 15 / 18.5 mm.
- **Lag tube Ø5 × 1** (Ø4 × 0.5 was at 138 MPa in shear).
- **TPU series link moved from the follower into the claw bar** (the follower is 8 mm tall; the link needs 15 mm). Followers are plain blocks on a T-rail in the cage, at −12° / 0° / +12° from their finger; the bars stay at ±22°, so the tip and middle arms are skewed.
- **Claw bars 7 × 6 U-channels with a gusseted arm** (the 2 mm arm of the blueprints would have seen about 580 MPa at 69 N). Bars at r 46.8-53.8, outside the servos: **pack Ø110.5 over the guide sleeves** (plates Ø102). Guide sleeves 43 mm (lower tier) and 18.5 mm (upper tier): guide efficiency 0.87 / 0.70, so **the cord efficiency is now 0.52** (was 0.62) and the servo margins 1.44 (base) / 1.78 (middle) / 2.27 (tip).
- **Claw arms 1.5 mm lower** (fin top 6.5 mm under the hub plate) to clear the K1 back-stop teeth.

**Checks on the solids (`hand3_cad_checks.py`):**

| Check | Result |
|---|---|
| Parts | 28 printable or machined parts, all single watertight bodies |
| Straight finger | no overlaps; knuckle gaps 0.04-0.10 mm, grip to hub plates 0.05 / 0.10 mm |
| Fold range free of contact | K1 −2° … 93°, K2 −2° … 93°, K3 −2° … 89° (wanted −2° … 90 / 90 / 85°) |
| Spread 0 → 47° | no overlaps; grips stop on their posts at 0° and 47°; chain to hub plates ≥ 2.0 mm; the two fingers of a pair 4.4 mm apart |
| Grasp pose (spread 47°, fold 55 / 60 / 50°, bars pulled 9 mm) | no overlaps |
| Full curl 90 / 90 / 85° on all four | not a real pose: the tips run into the hub (as on the hand-prop) |
| Flight spin, every 6° | rotating to stationary ≥ 1.0 mm between printed parts (fins to claw arms 1.0, fingers to claw arms 1.35) |
| Claw stroke 0 → 13.5 mm | no overlaps; claw to fin 1.0 mm in flight; claw to claw 1.1 mm; bar to servo 0.3 mm at full stroke |
| Pack | servo to servo 1.4 mm, servo to frame 0.2 mm, no overlaps |

**Loads at proof speed (14,260 rpm) with the CAD masses:** pull 1,369 / 1,019 / 492 N at K1 / K2 / K3 (the design-file estimate was 1,165 / 888 / 467: lugs, webs, sectors and teeth add 1.9 g outboard of K1). Printed PA-CF seats: bearing ≤ 48 MPa (K1 lugs), ring net section ≤ 47 MPa (K2 cheeks). Aluminium: K1 cheeks 54 MPa, lag eye 65 MPa, hub-plate shear-out 52 MPa. Lag tube 62 MPa in shear. Lag pin 1,555 N.
**Mass from the CAD:** 570 g per hand (the estimate was 533): fingers 51.0, hub plates 29.4, cage and followers 17.1, heel and nut 4.8, claw bars 28.4, pack frame 57.4 (skeleton plates), servos 252.8, motor 98.5, spools, springs, links, screws and wiring 28 (est.). Hover share with two hands: 13.9 N.
**Thrust with the knuckle hardware as built** (frontal area beyond the airfoil 71 / 39 / 25 mm² at K1 / K2 / K3 from the solids, Cd 0.5; K1 at r 36; `hand3_thrust_cad.json`): **21.3-25.4 N at full throttle** (rev B figure: 22.3-26.5), hover share at 8,400-9,700 rpm and 380-525 W electrical. `hand3_thrust.py` itself still uses its own hardware model and would give slightly different numbers.

**Open after the first CAD:** re-issue the blueprints from the solids; measure the servos and design their mounts; spread-cord routing; grasp simulation (pads are now 13-18.5 mm long and 1 mm thick); 570 g on the arm (arm servos, walk, landing); pull-test one finger to 2.1 kN; spin test; rotor parking; printing the 2.4 mm tip phalanx with its Ø1 cord grooves; the fold stops are 3-4° beyond nominal (set the limits in the servo travel).

## The eight decisions (first study)
Decision 6 (selector) and the key-hub parts of the sections below are superseded by the two sections above; decisions 1, 2, 4, 5 and 7 stand, and decision 3's lever arms were re-sized.
| # | Decision | Options shown | Pick | Why |
|---|---|---|---|---|
| 1 | Finger | — | 3 phalanges on the 10 in blade: knuckles at r 30 / 66 / 98 mm, phalanges 36 / 32 / 29 mm | a human-finger grain: 12 curl cords + 2 spread cords |
| 2 | How a cord picks its joint | A centre-line routing, B bowden sheaths, C all cords on the grip side, D one cord + joint brakes | **A** | a cord crossing a knuckle on its axis has no lever arm there, so each cord bends one joint only; no extra parts |
| 3 | Lever arm | A inside the airfoil (2 mm), B tunnel under the pad (4.5 mm), C pulley horn (8 mm), D bowstring | **B** | 0.36 N·m per joint, cord covered by the pad; C is stronger but puts bumps in the airflow; D is the "object presses on the cord" case |
| 4 | Knuckle | A stub pins in bushings, B rolling knuckle on ligaments, C steel tape spring, D TPU flexure | **A** | exact axis (needed by 2A), stiff in blade pitch, bought parts; C snaps (the on/off feel) |
| 5 | Where the give lives | A none, B silicone cord, C printed TPU link, D steel spring | **C** | 7.5 N/mm: each 2 mm key tooth adds about 15 N, five grip levels; silicone is 10× too soft, bare Dyneema 12× too stiff |
| 6 | Selector | A 14 keys + one pusher, B 14 keys + two pushers, C screw keys, D three sliding rings + three servos, E servos on the rotor | **B** | every joint of every finger stays independent, and a four-notch ring pusher sets one joint level on all four fingers at once: full grasp about 7 s instead of about 20 s |
| 7 | Finger spread | A lag pins + two spread keys, B fixed cross, C each finger on its own | **A** | the "rotate on the base" motion; already designed and fit-checked |
| 8 | Recommended design, bought parts, printing, unknowns | | | |

## Numbers behind the picks (`hand3_loads.py`; estimates)
- **Speed:** 26.5 N (the XL claw's ceiling) on a 10 in 4-blade rotor needs about 11,700 rpm with C_T 0.14 (est. for thick untwisted blades). Proof speed 1.15× = 13,400 rpm, tip 179 m/s.
- **Spin load at proof speed** (mass outboard × ω² × its CG radius; PA-CF section + TPU pad + 0.4 g of hardware):

  | Joint | Mass outboard | Pull | Thrust moment at 26.5 N | CG offset for a lock ratio of 2 |
  |---|---|---|---|---|
  | Lag pin, r 20 | 8.0 g | 1,070 N | 485 N·mm | 1.2 mm |
  | K1 base, r 30 | 7.1 g | 1,029 N | 419 N·mm | 1.1 mm |
  | K2 middle, r 66 | 4.1 g | 769 N | 197 N·mm | 0.7 mm |
  | K3 tip, r 98 | 2.0 g | 439 N | 52 N·mm | 0.3 mm |
- **Flight lock:** a 2° preconed stop seat puts the outboard CG about 1.5 mm (K1) behind the pin line: about 2.8× the thrust moment, the same at any rpm. (`hand-prop.md` item 7 warns that the Snap-Digit's lock figure forgot the CG offset of the hinge; here the offset is the design variable.)
- **Pins:** a Ø1 mm wire pin (as in the scale fingers) would put 55-130 MPa on the plastic. Two Ø3 × 6 hardened dowel stubs (ISO 8734) in 4 × 3 mm brass bushings at K1 and K2: 26-34 MPa on the bushing seats in PA-CF; Ø2 × 5 at K3. Pin shear ≤ 170 MPa. The two stubs leave a 3 mm gap on the axis for the cords.
- **Lever arm** (cord limited to 80 N by the key teeth, 11 MPa on 3 teeth):

  | Lever arm | Joint torque | Fingertip squeeze (15 mm) | Wrap force at 70 mm | Joint angle per 2 mm tooth | Cord travel for 80° |
  |---|---|---|---|---|---|
  | 2.0 mm | 0.16 N·m | 11 N | 2 N | 57° | 2.8 mm |
  | 4.5 mm | 0.36 N·m | 24 N | 5 N | 26° | 6.3 mm |
  | 8.0 mm | 0.64 N·m | 43 N | 9 N | 14° | 11.2 mm |
- **Series elastic** (stiffness between the cord and its key):

  | Element | Stiffness | Verdict |
  |---|---|---|
  | none: 0.6 mm Dyneema, 150 mm | about 93 N/mm | one tooth = 190 N: binary |
  | silicone cord Ø3 × 15 mm | 0.7 N/mm | 56 mm of stretch for 40 N: no (the option first suggested by voice) |
  | printed TPU 95A link, 2 × 2 × 16 mm (E 30 MPa est.) | 7.5 N/mm | 15 N per tooth, five levels to 75 N; creeps |
  | steel extension spring Ø4, 0.6 mm wire, 10 coils | 3.3 N/mm | 1,600 MPa at 40 N: yields; a spring that survives is Ø6 × 25 mm |
- **Selector timing** (from `key-hub.md`: 6 key moves = 9.1 s): 14 single moves ≈ 20 s; with the ring pusher 3 curl moves + 2 spread moves ≈ 7 s. Three rings with three STS3032 servos would be about 0.3 s and continuous but lose single-finger moves and add about 120 g and 50 mm of height per hand (est.).
- **Motor:** about 0.64 N·m at 26.5 N; hover (13 N) about 270 W at a figure of merit of 0.5 (est.), no better than the XL claw. Candidates from maker data: BrotherHobby Avenger 3110 730KV (76 g with cable, 6S, load-tested on an HQ 10×5×3), T-Motor 2810 KV900 (65 g, Ø32.9 × 36.55 mm, 6S, tested on a 9.5 in tri-blade), BrotherHobby SE 2816 810KV (98.5 g, 3.5 kg on a 10×5×3).

## The recommended hand (sheet 8)
- **Finger:** three PA-CF phalanges with TPU pads. Cords on the centre line; each moves out into the tunnel under the pad just before its own knuckle (lever arm 4.5 mm). This also removes the Snap-Digit's off-centre tendon fence.
- **Knuckles:** fork and tongue, stub dowel pins in brass bushings, flat stop faces with 2° precone.
- **Give:** one printed TPU link per cord, in the hub between the cord and its key.
- **Hub:** 14 latching keys (12 curl + 2 spread) on a wider cage; a single pusher for one finger at a time and a four-notch ring pusher for all four; the motor as the wrist. The three cords of a finger go down through its lag tube (Ø3 bore), on the swing axis.
- **Printing:** phalanges flat on the back face so the layers run along the span; cord channels as open grooves closed by the pad; PA-CF for phalanges, cage and keys (0.25 mm nozzle for the 2 mm teeth), TPU 95A for pads and links. First print: one finger on a bench hub with three hand-pulled cords.

## Unknown until CAD or bench
1. Whether 14 keys fit round the motor with usable gaps (the 8-post cage has 0.77 mm between keys). Narrower curl keys or two tiers.
2. Thrust and power of thick untwisted finger blades (all aero figures here are estimates).
3. The wrist: a drone motor and ESC cannot hold an angle (open since the first key hub).
4. Grip steps: five force levels per joint, not a continuous squeeze. If that is not enough, option 6D (rings) or 6C (screw keys) is the way to a continuous one.
5. TPU link creep and its real stiffness.
6. Which arms carry this hand: neighbouring 10 in discs are about 20 mm apart in the flight pose.
7. Hinge barrels (about 7 mm across) on the blade at each knuckle: drag and how they meet the pad.

## Sources for bought-part data
- T-Motor 2810 KV900: https://uav-en.tmotor.com/2026/Motor_0126/1334.html
- BrotherHobby Avenger 3110 730KV / 900KV: https://rcdrone.top/products/src-162-brotherhobby-avenger-3110-730kv-900kv-motor-cw
- BrotherHobby SE 2816 810KV: https://rcdrone.top/products/src-170-brotherhobby-se-2816-810kv-motor-cw
- Feetech SCS0009 (23.2 × 12.1 × 25.25 mm, 13.2 g, 2.3 kg·cm stall and 0.7 kg·cm rated at 6 V, 0.1 s / 60°, 300°, 4-7.4 V): seller pages on evelta.com, aifitlab.com and rcdrone.top (found by web search 2026-11-13). STS3032 rated torque and stall current are assumed (stall / 3; 1.4 A).
