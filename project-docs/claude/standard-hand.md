# Standard hand: hand3 (decided 2026-11-14)

**FINAL (2026-11-15): hub C2, every test passes (geometry 9/9, thrust, grasp, shovel, flight 15/15). Record: `claude/hand3-final.md`; design code `claude/code/hand3e/`; tests `claude/code/tests/`; build kit `claude/hand3-build-kit.md`. Sections below are history.**


**Forest's decision (2026-11-14): "Hand3 is the new standard for this project's hand."** The four-finger hand-prop (`claude/hand3-design-study.md`, code `claude/code/hand3_design.py` = single source of truth, `hand3_cad.py`, `hand3_cad_checks.py`) is the default hand on every hand arm in this project. Designs, sims, videos and docs use it unless a task says otherwise. **Servo drum v2 and the arm mount: `claude/hand3-drum-mount.md`, `claude/code/hand3_pack2.py` (replace the rev C pack frame).**

**Rev D (2026-11-14, later the same day): Forest asked for a smaller hand ("shrink their design a bit") and picked the 9 in rotor with the slim drum. The design files are now rev D: 9 in rotor (pitch 16°), 10 servos (the fingertip follows the middle joint on one cord), drum Ø94 mm (Ø102.6 over the claw bars), 495 g, thrust 20.2-23.3 N. Full record, flight check, the full-robot renderer and the contact grasp: `claude/hand3-revD.md`. Where the sections below give rev C numbers (10 in, 14 servos, 570 / 618 g, 21.3-25.4 N), rev D replaces them. Drum v2, the arm collar, the metal STLs and the blueprints have NOT been redone for rev D yet.**

**Hub C (2026-11-14, same evening): Forest asked to rethink the hub and chose option C. The 10-servo drum, claw bars and followers are replaced by two sliding rings pulled by two STS3215 (base ring: spread + K1; curl ring: K2 + K3). Hand mass 374 g, body 49 × 48 mm block with the rings inside Ø72. Rotor and fingers stay rev D. Record, bench and sims: `claude/hand3-hub-rethink.md`; source: `claude/code/hand3_hubc.py`. Drum v2 and the build kit for the drum (`hand3-drum-mount.md`, `hand3-build-kit.md`) are superseded by it; the fingers act together, not one by one.**

## What it replaces
- The fiddlehead tentacle (`fiddlehead-hand.md`) as the default tool hand. It stays as a swappable hand on the modular bayonet connector (`hand-connector.md`), not the baseline.
- The Snap-Digit claw as the grip on hand arms (it remains the flight pod on the non-hand arms: 6 in, or XL 7.5 in).
- Earlier hand-rotor concepts (`hand-prop.md`, `key-hub.md`, one-pusher key hub): superseded.

## What the standard is (rev C numbers; rev D in `hand3-revD.md`; all estimates unless marked)
- 10 in rotor (rev D: 9 in), 4 fingers × 3 phalanges, each joint its own servo (rev D: K1 its own servo, K2 + K3 on one cord), park-and-dock cord drive: 14 stationary claw bars push 14 followers (12 curl + 2 spread) (rev D: 10 and 10). Spread 0-47°, fold 90 / 90 / 85°.
- Motor BrotherHobby SE 2816 810KV (98.5 g, 6S); servo drum under the motor: 10 × STS3032 + 4 × SCS0009 (ears snipped, rails + stop + keeper plate, drum v2) (rev D: 10 × STS3032, drum v2 not redone).
- 570 g per hand in rev C; **about 618 g with drum v2, the arm collar, keepers and screws** (`hand3-drum-mount.md`) (rev D: 495 g with the rev C style frame; drum v2 for 10 servos is open). Thrust 21.3-25.4 N at full throttle (rev D: 20.2-23.3 N) (BEMT + NeuralFoil, no spin test); hover share 13.9 N (11.0-11.3 N on the 12.5 kg robot).
- Mount: the hand **replaces the arm's tip pod** and clamps on the Ø32 tip-bay tube (collar + Ø3 cross pin + 3 × M3 into the spine). Not through the bayonet connector (motor 55 A vs 5.2 A pins). **Clocking (2026-11-14): the hand sits turned 45° about the rotor axis on the wrist, so the arm can line the grip axis up with a tool handle (`hand3-revD.md`).**
- All CAD checks pass (fold range, spread, grasp pose, flight spin, claw stroke, pack; drum v2 fit checks in `hand3_pack2.json`). Rev D: same, plus a 0.02 mm³ graze of the 16° root grip on the lower hub plate at 47° spread (needs a 0.2 mm relief).

## Rules from the full-robot sims (2026-11-14, `hand3-flight-check.md`, `hand3-revD.md`)
- The two hands' rotors are mirror images (opposite-handed); spin pattern `[1,-1,-1,1,-1,1,1,-1]`.
- In flight the hand arms stay upright (thrust only); the other six arms steer.
- A hand holds a bar across its heel, between the two finger pairs (spread 47°). Measured in the contact sim on a Ø28 bar: 15 N pull-off, 10 N along the bar, but only 0.28 N m as a lever. One hand cannot carry a pan by its handle; two hands on one shaft work.
- Two hands on one tool: each lines its grip axis up with the line through both hands, and one hand trims its place along the handle.
- Quad mode (four arms work at once) does not fly with hand3; hand work mode (the two hands work, six claw-props + EDF fly) does.
- With one cord for K2 + K3 (rev D) the elastic extensor needs about 0.02 N m of pretension at K3.

## Materials and build split (2026-11-14)
Forest has a resin (MSLA) printer with standard and carbon-fibre-filled resin, no aluminium; agreed to order metal parts as 3D prints ("we can use metal if I can get 3D printed parts for it") and to go forward with the design proposed here.
- **Order (service-printed):** 4 aluminium root grips + lower and upper aluminium hub plates per hand (AlSi10Mg metal print or 6061 CNC; `hand3_metal.py` → `hand3_metal_stl/` + order note; ream bores after: lag 5.0, shaft 5.2, K1 bushings 4.0). 12 PA12 nylon phalanges per hand (MJF or SLS, same STLs as the bench finger) + spares. **Rev D changes the root grip (16° pitch) and all three phalanges (shorter): regenerate the STLs before ordering.**
- **Buy:** lag tubes (5 × 1 stainless), dowels and brass bushings, motor, servos, Dyneema, pads (TPU or 1 mm rubber sheet), drum screws and inserts (`hand3-drum-mount.md`).
- **Print at home (resin):** drum frame v2 and arm collar (CF resin), servo keepers, cage, followers, claw bars, spools, heel (flexible resin or rubber), the bench finger (CF resin).
- **Why metal grips and hub plates:** at proof speed (14,260 rpm) the lag eye carries ~1.3 kN: 38-53 MPa in the eye before the hole's stress concentration, against 55-65 MPa tensile and 3-6 % elongation for a typical CF-filled resin (Resinify CarbonTough TDS). Metal per hand 41.6 g (resin would be 19.2 g); the flight check (`hand3-flight-check.md`, 12.66 kg robot) already used the aluminium masses, so metal does not change whether it flies.
- **Why PA12 phalanges:** PA12 (HP MJF data, recalled, to confirm with the chosen service: ~48 MPa tensile, ~20 % elongation, 1.01 g/cm3) is lighter than PA-CF (1.25), so the centrifugal loads drop to about 0.81 ×: K2 cheek ring ~38 MPa and K1 lug bearing ~39 MPa at proof speed (factor ~1.26), ~29 MPa at the 12,400 rpm design speed (~1.66), ~15 MPa in hover (~3.2). CF resin phalanges (48 MPa at proof, brittle) are for the bench only.
- Resin-only alternative (not built, est.): cap the hand at ~11,000 rpm, thicker grip eye and plates, bigger barrels; peak hand thrust ~18-20 N against the ~18 N the day mission peaks at. Not recommended.
- Heat-set inserts do not work in resin: knurled inserts glued with epoxy, or tapped holes (hand connector, tip pad, hands, drum).
- Picture of the materials: `hand3_look.py` → `hand3_look.png` (rev C).

## Consequences to carry into the main design
1. Add the hand-arm mass to `cephalo4.py` as the hand-arm variant; hand arms are 2 and 6 only. Done in the drivers for rev D (`hand3_params.py` reads the design files; flight day mission and hand work mode rerun at 495 g). Still to rerun: walk, the legged landing loads, the step program, and everything once drum v2 for 10 servos gives the real mass.
2. ~~Replace the spring-tie grips with the hand3 grasp~~ done for the shovel (2026-11-14, `hand3_mj.py`, contact + friction, `hand3-revD.md`). Dishes: not feasible one-handed. Still open: the fiddlehead comparison (105/108 shovel result), the docking grab.
3. ~~Mount: bayonet or replace the tip pod~~ decided 2026-11-14: replaces the tip pod (`hand3-drum-mount.md`).
4. ~~Servo mounts~~ done in drum v2 for rev C (set `SV_DIM` once the servos are measured); **redo drum v2 for rev D (10 servos, Ø94)**. Still open: blueprint sheets from the solids, spread-cord routing, the K2 + K3 cord routing and the K3 extensor pretension, Hall index sensor, bus wiring board, spin test, pull test, rotor parking.
5. Videos that show a hand: ~~no renderer~~ `hand3_draw.py` draws hand3 on the full robot (2026-11-14); the day-mission flight clip and the shovel video are re-rendered with it. Still to redo: chores and the docking grab.
6. Confirm the PA12 numbers with the chosen service's datasheet; pull-test one PA12 finger to 2.1 kN before the spin test.
7. Arm 2's j5 servo overheats (1.65 × rated) in the shovel pose with the hand on: revisit the pose or that servo.
