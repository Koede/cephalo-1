# Cephalo-1 arm: main design (pin hinge, 60° helix)

Status: main design as of 2026-11-09. Forest gave the arm length as about 1 m ("like the length of
my own arm") and chose the 13-joint build, which flies only with claw fans of about 16 N. The main
design is therefore: pin hinge, 60° helix, 1 m, 13 joints, STS3250 in joints 0-4, STS3215 in joints
5-12, 16 N claw fans, and (root fix, same day) a 2:1 gear on joint 0 with every move kept within
servo speed. CAD: `arm-pin-hinge/out_1m_13/`, with the geared root socket drawn in (see "Root socket") and its
bearings resized after the root bearing check (see "Root bearing check"). The 585 mm build (`out/`) is kept for
reference; the sections "Geometry at 585 mm" to "Simulation at CAD size" describe it.

## Summary

Each arm is a chain of plain pin hinges with one bus servo per joint: STS3250 in joints 0-4 and
STS3215 in joints 5-12. The two servos share one case, so every cradle is the same. Every hinge axis crosses
the arm centreline at right angles, and each axis is turned 60° about the arm from the one before
it, so the joint axes trace a helix along the arm. Three neighbouring joints therefore span three
bending directions 60° apart, which lets the arm bend in any direction and coil into a helix
without roll joints. The servo sits in a cradle in the proximal vertebra with its output shaft on
the hinge axis; the distal vertebra ends in a fork whose ears are bolted to the servo horn on one
side and turn on a printed stub axle on the other.

Because the twist per joint is constant, every moving vertebra is the same part rotated about the
arm axis. One vertebra STL is printed twelve times per arm.

## Main design at 1 m (out_1m_13)

| Item | Value |
|---|---|
| Arm length | 1000 mm, root to claw plate |
| Joints per arm | 13, one 12 V bus servo each: STS3250 in joints 0-4, STS3215 in joints 5-12 |
| Root drive | joint 0: STS3250 through a 2:1 spur stage, 9.8 N.m and 2.2 rad/s at the joint (see "Root fix") |
| Joint pitch | 76.9 mm (shortest pitch that clears: 60.5 mm) |
| Hinge axis twist | 60° per joint |
| Travel | ±45° commanded, collision-free to ±90° |
| Outer diameter | 57.1 mm |
| Printed parts | root_socket_x1 (30 g, geared), root_end_cap_x1 (1.9 g), root_bearing_cap_x1 (2.1 g), vertebra_x12 (31 g each), tip_vertebra_x1 (37 g), servo_lid_x12 (2.5 g each) |
| Arm mass outside the mantle | 1,243 g with servos and hardware (1,394 g counting the geared root: socket, caps, root servo, gear stage, bearings and idler pin; claw extra) |
| Servos, eight arms | 40 STS3250 and 64 STS3215 |
| Robot | 15.08 kg, thrust-to-weight 1.150 with 16 N claw fans (14.71 kg and 1.18 before the root fix) |
| Shape miss | 18.8 mm mean over the target shapes |
| Wave tip RMS | 22 mm at 0.25 Hz, 62 mm at 0.5 Hz; saturates at 1 Hz (38 and 106 mm before the root fix) |
| Lift-test tip RMS | 5.9 mm (345 mm before the root fix) |
| Step to the curl pose, final miss | 86 mm (386 mm before the root fix) |

Printing, assembly and the values to measure are as for the 585 mm build below; only the pitch and
the part counts change.

## Root fix (2026-11-09)

The 345 mm lift-test miss came from one moment in the test: the arm starts straight out and level,
with the fans off, and is commanded to the walking stance in one jump. Straight out, gravity needs
7.2 N.m at joint 0 and 4.4 N.m at joint 3; the STS3250 stalls at 4.9 N.m. Joint 0 gives way, the
arm folds and never reaches the stance. Started from the stance, the same arm lifts its foot to
13 mm, so walking itself was never the problem. MuJoCo also lets a servo move at any speed, which
made the jump harsher than a real servo would allow.

The fix has two parts, adopted as the main design:

1. Joint 0 drives through a 2:1 gear. The STS3250 stays; the joint gets 9.8 N.m and 2.2 rad/s.
2. No move is commanded faster than 80 % of each servo's rated speed (a cosine ease). Raising the
   arm from straight out to the stance then takes 0.77 s.

Gear stage (spec, nothing bought):

| Item | Spec |
|---|---|
| Ratio | 2:1, spur, module 1 |
| Pinion | 20T on the STS3250 horn, 22 mm OD, 10 mm face, 9 mm centre bore; its far end runs in a 695 bearing |
| Gear | 40T on the joint-0 pin, 42 mm OD, 10 mm face; a sector with teeth over ±65°, hard stops at ±55°; a 61803 sits in a recess in its inboard face, under the mesh |
| Centres | 30 mm: the servo moves 30 mm off the joint-0 axis, inside the root socket |
| Material | 7075-T6, hard-anodized (bare aluminium gears gall) |
| Bearings | one 61803 (17 x 26 x 5 mm) on the gear's spigot, one 695 (5 x 13 x 4 mm) on the pinion (a 6800 and a 685 until the root bearing check) |
| Tooth load at STS3250 stall | 490 N; Lewis bending 153 MPa on the pinion and 126 MPa on the gear, under the roughly 160 MPa fatigue strength of 7075-T6. Holding the arm straight out loads the pinion to about 112 MPa. A 12 mm face gives 128 and 105 MPa for about 4 g more per arm, which breaks the budget |
| Added mass | 46.2 g per arm, from the CAD (see "Root socket"; 45.3 g before the root bearing check); the first estimate was 45 g |
| Mass budget | thrust-to-weight stays at 1.15 or above while the stage weighs 46 g or less per arm |
| Backlash | the stage halves the servo's 0.5° backlash at the joint and adds its own, about 0.1° |

Printed PETG gears were ruled out: at this torque a module 1.5 PETG pair is loaded to about
57 MPa, above what PETG takes.

What else was simulated (`joint-study/sim/fix_root.py`, `joint-study/results/root_fix.csv`, 36 runs):

| Fix | Added per arm | Thrust-to-weight | Lift-test RMS |
|---|---|---|---|
| None (as locked in) | 0 g | 1.18 | 345 mm |
| None, but raised at servo speed | 0 g | 1.18 | 127 mm |
| **2:1 gear on joint 0, raised at servo speed (adopted)** | **45 g** | **1.151** | **5.9 mm** |
| 1.5:1 gear on joint 0, raised at servo speed | 45 g | 1.151 | 8.0 mm, curl step misses by 248 mm |
| 2:1 gear on joints 0 and 3, raised at servo speed | 90 g | 1.12 | 5.1 mm (fails the 1.15 gate) |
| Second STS3250 on joints 0 and 3, raised at servo speed | 169 g | 1.08 | 6.8 mm (fails the gate) |
| Feetech STS3095 in joint 0, raised at servo speed | 120 g | 1.11 | 7.6 mm (fails the gate) |
| Torsion spring on joint 0, raised at servo speed | 15 g | 1.17 | 203 mm |
| Claw fan 4 N only while the arm rises, raised at servo speed | 0 g | 1.18 | 14 mm |
| Claw fan 4 N while the leg swings, raised at servo speed | 0 g | 1.18 | 17 mm |
| Bigger servo or gear without the speed limit | 20 to 120 g | 1.11 to 1.17 | 273 to 359 mm |

Why the gear over the free fan option: the claw fan fixes only the raise (14 mm) and leaves the
rest of the arm as it was, while the gear also brings the step to the curl pose from 386 to 86 mm,
the 0.5 Hz wave from 106 to 62 mm and the hover sag from 4.2 to 2.2 mm. The fan trick still works
on top of the gear if the gear stage comes out heavier than its budget.

The torsion spring fails because the load at joint 0 changes with pose: a spring sized for straight
out pushes against the servo in every other pose. Bigger servos fail the thrust-to-weight gate:
the STS3095 (9.3 N.m, 194.5 g) and the XM540-W270 (10.6 N.m, 165 g) each add 90 to 120 g per arm.

Not yet done: a sim that models servo speed directly rather than through the command limit.

## Root socket (geared, 2026-11-09)

The root socket is redrawn around the 2:1 stage (`arm-pin-hinge/root_gear.py`, run after
`pin_cad.py` for the same arm). Every vertebra, the tip and the fork that bolts to joint 0 are
unchanged: the fork's horn ear now bolts to the 40T gear's hub, on the same 4 x M3, 14 mm bolt
circle the servo horn had, and its idler ear turns on an 8 mm 7075 pin of the same size as the
printed stub axle it replaces.

Layout, in the joint-0 frame (along the hinge axis, across it, along the arm; the hinge at 0):

| Part | Where and what |
|---|---|
| 40T gear | 9.5 to 19.5 mm along the hinge axis, hub to 22.5 mm; a recess in its inboard face (to 15.6 mm radius, 5.5 mm deep) takes a ring boss off the socket's gear web, and a 61803 sits between the boss and the gear's 17 mm hollow spigot, at 9.5 to 14.5 mm, right under the mesh |
| 20T pinion | same span, 30 mm below the hinge, bolted to the servo horn (4 x M3 from the pinion side, tapped horn); stub into a 695 in a screw-on bearing cap |
| STS3250 | moved 13 mm along the hinge axis and 30 mm down the arm; slides into a closed sleeve from the proximal end and is held by an end cap (no lid at joint 0) |
| Idler side | cheek plate at -22.4 to -19.4 mm with a 3 mm boss; an 8 mm 7075 pin pressed through both carries the fork's idler ear (M3 tapped for the idler screw) |
| Hard stops | two lugs on the gear web and boss meet the sector's cut edges at ±55° of joint travel |
| Mantle flange | same plane as before (37.7 to 40.7 mm below the hinge), widened on the servo side, 8 x M3 |

Checks with the socket in place (`out_1m_13/fit_report.md`): no interference at zero; joint 0 is
free from -55 to +55° (the stops; the command range is ±45°); joints 1 and 12 are free to ±90°;
joints 0 and 1 turned together are clear to ±60° both ways. The gear and pinion mesh at 30 mm
centres in the reference model.

Mass per arm, against the old socket:

| Item | Mass |
|---|---|
| Socket | 30.1 g printed (old socket 23.2 g) |
| End cap and bearing cap | 1.9 g and 2.1 g printed |
| Joint-0 lid, no longer fitted | -2.5 g |
| 40T sector gear, 7075-T6 | 15.0 g |
| 20T pinion, 7075-T6 | 6.4 g |
| Idler pin, 7075-T6 | 1.3 g |
| 61803 and 695 bearings | 7.5 g (SKF catalogue) and 2.2 g (catalogue class, check the part in hand) |
| Extra screws and inserts | about 5.4 g |
| **Added per arm** | **46.2 g**, against the 46.4 g budget; thrust-to-weight 1.150, at the gate (45.3 g and 1.151 before the root bearing check) |

Two things change outside the arm:

- The servo sits off to one side of the tube: its sleeve stands 6.8 mm proud of the 57.1 mm
  tube along the hinge axis, from the flange face to 16.7 mm below the hinge. Link 1 never
  reaches it.
- The mantle opening needs a notch on that side. Everything that passes through the mantle fits
  inside a 61 x 42 mm outline (-35.9 to 25.0 mm along the hinge axis, ±21.1 mm across, with 0.5 mm
  clearance), drawn in `out_1m_13/mantle_cutout_joint0_frame.svg`.

Gear options that were drawn and weighed (same socket, before the root bearing check added 0.9 g):

| Gear | Face | Added per arm | Thrust-to-weight |
|---|---|---|---|
| **Sector, teeth over ±65° (adopted)** | **10 mm** | **45 g** | **1.151** |
| Sector, teeth over ±65° | 12 mm | 49 g | 1.148 |
| Full gear, webbed, 6 lightening holes | 10 mm | 54 g | 1.146 |
| Full gear, webbed, 6 lightening holes | 12 mm | 59 g | 1.142 |

A sector can be cut from a stock m1 40T aluminium gear: saw off the toothless part, drill the
web. The full gear fits the socket unchanged if the extra 9 to 14 g is acceptable; the claw fan
assist during the raise (14 mm lift miss without any gear) still works on top of it.

Assembly: press the 61803 into the socket's boss (against the lip) and the 695 into the bearing
cap; press the idler pin through the cheek; slide the STS3250 with its horn in from the proximal
end and fit the end cap; drop the pinion in through the bracket hole and bolt it to the horn; fit
the bearing cap (2 x M3); push the gear's spigot into the 61803; then hang the first vertebra's
fork on the pin (PTFE washer, idler screw) and bolt its horn ear to the gear hub. There is no gear
retaining screw: the bearing's lip takes the arm's push toward the idler side, and the idler ear on
its washer takes the push the other way.

Not checked yet: backlash, which the reference model leaves at zero.

## Root bearing check (2026-11-09)

Forest asked for the root bearing to be checked. Script: `joint-study/sim/bearing_loads.py`;
numbers: `joint-study/results/bearing_loads.json`. The loads the arm puts on the root socket
(its own weight, the claw payload and the claw fan's push) were worked out for the main-design
chain in straight, curled, coiled, walking and 60 random poses. The moment about the hinge goes
through the gear mesh (tooth force at 20°); the rest goes to the two radial supports, the gear-side
bearing and the idler ear. Cases with more than 9.8 N.m about the hinge were dropped, since the
gear stage cannot hold them.

Load cases:

| Case | What |
|---|---|
| Hover | 13.2 N up at the claw, arm level or in any pose |
| Flight stress | 16 N tilted up to 30° (the study's flight stress case) |
| Rated thrust, any direction | 16 N pointed any way by the wrist gimbal, any pose |
| Full throttle, any direction | 21.4 N pointed any way, any pose |
| Walking | 21.1 N at the foot (7 feet share 15.08 kg) |
| Hard landing | 74 N at the foot (4 feet share the robot, x2 for impact) [A] |
| Gear at stall | 9.8 N.m about the hinge |

Result: the socket as first drawn fell short. When the claw fan pushes across the hinge plane, up
to 16 N.m (rated thrust) or 22 N.m (full throttle) acts out of the plane at the root, and the
supports are only 31 mm apart. The 6800 also sat 8 mm inboard of the mesh, so it carried 1.26
times the tooth force before any side load.

| Part | As first drawn | After the check |
|---|---|---|
| Gear-side bearing, worst load | 6800: 1,143 N at rated thrust, 1,276 N at full throttle | 61803: 971 N and 1,084 N |
| Its static safety (C0 / load) | 0.73 and 0.65; 1.17 to 1.33 in flight stress, landing and stall | 1.31 and 1.17; 2.1 or more in every other case |
| Pinion bearing at stall | 685: 250 N, safety 1.12 | 695: 244 N, safety 1.68 |
| Idler support, worst | printed stub: 807 N, 36 MPa bending at the cheek (printed PETG takes roughly 40 MPa along its layers, less across) | 7075 pin: 616 N, 26 MPa (7075 takes about 500 MPa) |
| Fork idler ear on the pin, worst | 32 MPa | 24 MPa at full throttle, 18 MPa at rated, 3 MPa at hover |

Ratings: 6800 C0 0.83 kN and 61803 C0 1.27 kN (SKF), 685 C0 28.5 kgf and 695 C0 41.8 kgf
(maker data tables). Hover life is not a concern: the 61803 runs at under a third of its static
rating.

What changed in the CAD to fix it, at no net cost to the 46.4 g budget:

- The gear-side bearing is now a 61803 in a recess in the gear's inboard face, centred 2.5 mm from
  the mesh plane instead of 8 mm, and 5.5 mm further from the idler ear (36.5 mm span).
- The printed idler stub is now a pressed-in 8 mm 7075 pin; the fork's idler ear is unchanged.
- The pinion bearing is a 695 instead of a 685.
- Paid for with: a 9 mm centre bore in the pinion, a 2 mm pocket in the gear's outboard face, a
  hollow gear spigot with no retaining screw, a 2 mm end cap, and no heat-set insert in the idler.

What is left:

- The fork's printed idler ear is the weakest part now: 24 MPa on the pin at full throttle pushed
  sideways, which printed PETG takes for a moment but not as a steady load. Hover loads it to 3 MPa.
- The servo's output spline carries about 280 N of side load at stall from the pinion. Feetech does
  not publish a limit for it.
- Joints 1 to 4 have no gear and see the same kind of load. Checked below (Joints 1 to 4 side-load check).
- Simplest guard for all three: the flight controller keeps each claw fan's push within 30° of
  straight up. In that envelope the root sees under 3 N.m out of plane.

## Joints 1 to 4 side-load check (2026-11-09)

Script: `joint-study/sim/joint_loads.py`; numbers: `joint-study/results/joint_loads.json`. The same load
model as the root check (outboard arm weight, 130 g tip, claw fan push; straight, curled, coiled, side
arc, s-curve, walking and 60 random poses), applied at joints 1 to 4. Poses the STS3250 cannot hold
(over 4.9 N.m about the hinge) were dropped. These joints have no gear, so everything except the moment
about the hinge goes to the fork's two ears, 49 mm apart: the horn ear (through 4 x M3 into the servo
horn, the output spline and the servo's own output bearing) and the idler ear (on the printed 8 mm stub).

Worst over joints 1 to 4:

| Case | Horn side N | Idler side N | Axial N | Printed stub bending MPa | Ear in-plane MPa | Idler bore MPa |
|---|---|---|---|---|---|---|
| Hover, any pose | 111 | 110 | 4 | 4.6 | 7.8 | 4.3 |
| Rated 16 N within 30° of up | 191 | 196 | 9 | 8.2 | 7.6 | 7.7 |
| Full 21.4 N within 30° of up | 294 | 294 | 13 | 12.3 | 4.7 | 11.5 |
| Rated within 60° of up | 244 | 249 | 13 | 10.4 | 10.3 | 9.7 |
| Rated, any direction | 409 | 408 | 27 | 17.0 | 17.0 | 15.9 |
| Full throttle, any direction | 511 | 509 | 32 | 21.3 | 7.8 | 19.9 |
| Walking stance | 33 | 27 | 9 | 1.1 | 1.8 | 1.0 |
| Hard landing [A] | 189 | 141 | 51 | 5.9 | 6.6 | 5.5 |

Printed parts: printed PETG was taken as about 45 MPa along its layers and 25 MPa across them [A], with
Forest's 2x rule. The ears pass in every case. The printed stub passes at 2x only while the claw fan's
push stays within 30° of straight up (12.3 MPa against 12.5); pushed sideways it reaches 17 to 21 MPa.

Servo output: this is the part that fails. The horn side puts the whole radial load on the STS3250's
output spline and its ball bearing, about 9 mm out from the case: 110 N in hover in any pose, 190 to
290 N with the fan within 30° of up, 410 to 510 N pushed sideways. Feetech publishes no side-load limit.
For scale, ROBOTIS rates the XM430 (82 g, 4.1 N.m, a heavier servo) at 40 N radial 10 mm from the horn.
Even hover is 2.8 times that.

What fixes it, lightest first (none drawn yet; each costs mass the per-arm budget does not have):

1. Fan guard in the flight controller (0 g): keeps each claw fan within 30° of straight up. It brings the
   printed stub inside 2x and the root under 3 N.m, but the servo output still sees 110 to 290 N.
2. 8 mm 7075 idler pin in place of the printed stub, as at the root (about +0.8 g a joint, +3 g an arm
   [A]): stub stress drops about 15-fold; it does not help the servo output.
3. Horn-side support bearing (estimated +5 to +10 g a joint, +20 to +40 g an arm [A]): a thin-section ring
   bearing between the fork's horn ear and the cradle lid, concentric with the output, so the side load
   bypasses the spline. This is the only change here that takes the load off the servo. At +32 g an arm
   T/W goes from 1.150 to about 1.13.
4. Bench test before choosing (0 g): hang 110 N and then 300 N off an STS3250 horn 10 mm out and check play
   and current draw after a few hundred cycles.

## Horn-side support bearing, joints 1 to 4 (2026-11-09)

Script: `arm-pin-hinge/horn_bearing.py`; output `arm-pin-hinge/out_1m_13/horn_bearing/` (stl/servo_lid_bearing_x4.stl,
stl/vertebra_bearing_x4.stl, assembly_joint1_bearing.stl, horn_bearing.json). A 6705 (61705) thin-section ball
bearing, 25 x 32 x 4 mm, C0 834 N (85 kgf), sits on the servo lid round the horn: its outer race in a 1.8 mm
printed ring on the lid, its inner race on a sleeve that hangs from the fork's horn ear round the servo horn
(0.3 mm clear of it). The lid top has a 0.5 mm groove under the inner race; the ear's boss flares from 10.5 to
12.5 mm radius at its root to carry the sleeve; the horn screws are re-cut through the new material. It fits the
4.5 mm between the lid top and the horn ear with 0.5 mm running gap, inside the 57.1 mm tube (new parts reach
28.3 and 28.5 mm radius against 28.55).

- Fit: no static interference; joints 1, 2 and 4 swing +-90 deg clear; joints 1+2 and 3+4 together clear to +-60 deg.
- Mass: 8.1 g per joint (bearing 6 g [A], lid +0.8 g, fork +0.9 g), 32 g per arm, 0.26 kg on the robot.
  Robot 17.18 kg with the central prop under the dome and the Tattu 16 Ah pack; rated T/W 1.061 -> 1.045.
- Bearing load (worst of joints 1 to 4, all on the bearing): hover 111 N (0.13 of C0), 16 N fan within 30 deg
  of up 191 N (0.23), any direction 409 N (0.49), full throttle sideways 511 N (0.61). Forest's 2x rule passes
  everywhere except full throttle pushed sideways (1.23).
- Lid screws (4 x M3 through the 2 mm lid): 4.6 MPa on the hole walls in hover, 21 MPa at the worst case.
- Prints: the vertebrae for joints 1 to 4 (parts 2 to 5) and their lids become their own STLs; the other 8
  vertebrae and 9 lids are as before.
- Not modelled: how the load splits between this bearing and the servo's own output bearing; that depends on
  the fits. Bench-test one joint before printing all 32.

## Flight battery (2026-11-09)

Script: `joint-study/sim/battery.py`; numbers: `joint-study/results/battery.json`. Hover draw with the
main design and no pack: about 5.3 kW at 6S (250 A). The eight claw fans take 2.2 kW at 13.2 N each. The
central EDF is assumed to run at its full 42.2 N and to give 1.45 g/W [A: Dr. Mad Thrust 90 mm 12-blade
6S class, 3.40 kg at 2,350 W; Forest's EDF is not known], so about 3.0 kW. About 150 W more for servos and
electronics [A]. Packs at 192 Wh/kg (Tattu 6S 22,000 mAh 25C: 2,544 g), 80 % used [A]:

| Pack added | Hover | Draw | Rated T/W | Full-throttle T/W |
|---|---|---|---|---|
| none | 0 | 5.3 kW, 251 A | 1.151 | 1.44 |
| 1.0 kg | 1.7 min | 5.6 kW | 1.08 | 1.35 |
| 1.5 kg | 2.4 min | 5.7 kW | 1.05 | 1.31 |
| Tattu 6S 16,000 mAh 15C, 1.93 kg | 2.9 min | 5.9 kW, 280 A | 1.02 | 1.28 |
| 2 x 6S 10,000 mAh 25C, 2.31 kg | 3.5 min | 6.0 kW, 286 A | 1.00 | 1.25 |
| Tattu 6S 22,000 mAh 25C, 2.54 kg | 3.8 min | 6.1 kW, 290 A | 0.98 | 1.23 |

Any pack added on top of 15.08 kg takes rated T/W under 1.15. The 16,000 mAh 15C pack gives only 240 A
continuous, under the 280 A hover draw. If part of the 3.45 kg body allowance is already battery, the
picture improves (1.5 kg already in it: 3.2 min at T/W 1.12 with the 1.93 kg pack). The EDF is the least
efficient lift on the robot (about 1.45 g/W against about 5 g/W for the claw props); a central prop of
about 4.5 g/W would stretch the 1.93 kg pack from 2.9 to 4.4 minutes.

Superseded on Forest's pick (Nov 9): the EDF is replaced by a central prop. See the next section.

## Central prop in place of the EDF (2026-11-09)

Script: `joint-study/sim/central_prop.py`; numbers: `joint-study/results/central_prop.json`. Forest asked for
a central prop under the dome or above it. Both were checked against the walking stances (each also with the
foot 45 mm up) and the flight pose, with the 57.1 mm tube and 20 mm margin [A].

| | Above the dome | Under the dome (recommended) |
|---|---|---|
| Motor, prop | T-Motor MN6007II KV320, 22 x 6.6 CF | T-Motor MN5212 KV420, 17 x 5.8 CF |
| ESC | 60 A class (GetFPV pairs FLAME 60A HV) | 70 A class [A] (motor max 60 A, 1,440 W) |
| Max thrust, 6S | 54.4 N, 908 W, 38 A | about 50 N, about 1.2 kW, about 52 A [A: 24 V table less 5 %] |
| Where the disc sits | 310 mm over the hub centre, a mast about 190 mm above the dome [A: dome top 120 mm] | 125 mm under the hub centre, 75 mm off the ground while walking |
| Why that size | lower disc hits the arms when they arch up over the body with a foot lifted | a 18 in disc hits the legs in stance 4 |
| Closest arm, walking | 21 mm (stance 3, foot up) | 22 mm (stance 4) |
| Flight pose | clear while the arms rise less than about 54 deg | clear while the arms droop less than about 34 deg |
| Download on the body | about 22 % of the prop's thrust (dome 6 %, arm roots 16 %) | about 4 % |
| Mass against the EDF | -0.18 kg (-0.06 kg with a ring guard) [A: EDF system 0.55 kg] | -0.15 kg |

Download: above the dome, the dome and the eight arm roots sit in the fast, contracted slipstream (about 1.7
times the speed at the disc, 212 mm radius at the arm plane), so they push down with about 22 % of the prop's
thrust. Under it, the dome sits in the slow inflow (about 0.6 times), about 4 %. The 22 in prop is far more
efficient per watt (561 W for 40 N against 762 W for 39 N on the 17 in), and the download takes that back.
Momentum disc with Cd 0.4 for the dome and 1.1 for the arm tubes [A]; this is the least certain number here.

Hover with the best split between the central prop (held at or under 90 % thrust) and the claw fans (rated 16 N):

| Pack | EDF (before) | Above, 22 in | Under, 17 in |
|---|---|---|---|
| none | 5.3 kW, T/W 1.15 | 3.2 kW, T/W 1.17 | 3.2 kW, T/W 1.20 |
| 1.0 kg | 1.6 min, 5.6 kW, T/W 1.08 | 2.7 min, 3.5 kW, T/W 1.09 | 2.6 min, 3.5 kW, T/W 1.13 |
| 1.6 kg | 2.6 min, 5.8 kW, T/W 1.04 | 4.0 min, 3.7 kW, T/W 1.05 | 4.0 min, 3.7 kW, T/W 1.09 |
| 2.3 kg | 3.5 min, 6.0 kW, T/W 1.00 | cannot hover inside the claw fans' rating | 5.4 min, 3.9 kW (186 A), T/W 1.04 |

Recommendation: under the dome. Same hover power, better T/W at every pack, the only one that carries the
2.3 kg pack, no mast, lower CG, and the arms can rise freely in flight. What it costs: the blade tips are
75 mm off the ground while walking (a step or a stone strikes them), 22 mm from the legs in stance 4, and it
throws grit on take-off. A ring guard under the dome would cover the legs and the ground for about 0.1 kg
[A]; a full duct could add thrust but eats the ground clearance, so it is not worth it here. Ground effect
at take-off (disc 0.35 radii off the ground) adds thrust while the feet are still down. Yaw: about 1.5 N.m
at hover [A: about 5,500 rpm], cancelled by tilting the claw fans under 1 deg tangentially.
Most of the hover power is still the claw fans (2.5 to 2.9 kW of 3.5 to 3.9 kW).

### Flight pack (Forest picked 2.3 kg, Nov 9)

No 6S LiPo weighs 2.3 kg; the real packs either side, from the sellers' pages (Nov 9), with the prop under the
dome. Peak draw at full throttle is about 274 A; claw fans rated 16 N, full 21.4 N.

| Pack | Mass | Energy | Current rating | Hover draw | Hover | Rated T/W |
|---|---|---|---|---|---|---|
| Tattu 6S 16,000 mAh 30C LiPo (design default) | 1.99 kg | 355 Wh | 480 A | 181 A, fans 15.6 N | 4.5 min | 1.06 |
| Tattu 6S 22,000 mAh 25C LiPo | 2.65 kg | 488 Wh | 550 A | 192 A, fans 16.4 N | 5.8 min | 1.02 |
| 2 x Tattu 6S 10,000 mAh 25C LiPo, parallel | 2.80 kg | 444 Wh | 500 A | 194 A, fans 16.6 N | 5.2 min | 1.01 |
| Tattu semi-solid 6S 33,000 mAh (350 Wh/kg) | 2.36 kg | 733 Wh | 330 A for under 3 s only; continuous not stated | 187 A | 9.0 min if it could feed it | 1.04 |
| 2 x DOGCOM semi-solid 6S 22,000 mAh 5C, parallel | 2.78 kg | 977 Wh | 220 A (440 A burst); its XT90 plugs are good for about 90 A each | 194 A, fans 16.6 N | 11.5 min | 1.01 |

Design default: the Tattu 16,000 mAh 30C, the heaviest LiPo under 2.3 kg, with five times the current it needs.
Every heavier pack pushes the claw fans past their 16 N rating at hover and takes T/W to about 1.0, where the
robot can hover but barely climb. The semi-solid packs hold two to three times the energy per kg but are rated
for 3 to 5C continuous; only two DOGCOM 22,000 mAh packs in parallel cover the hover draw, at 88 % of their
continuous rating, and the 274 A peak only as a burst. Semi-solid cells also sag more under load [A: not modelled].
Hover now runs each claw fan at about 15.6 N (was 13.2 N with the EDF and no pack), so the joints 1 to 4 hover
side load rises from about 110 N toward the 16 N rated case (about 190 N within 30 deg of up).

## Bigger claw fans and the bent claw (2026-11-09)

Scripts: `joint-study/sim/claw_fans.py` (fan x pack table, `results/claw_fans.json`), `claw_bend.py` (claw geometry,
`results/claw_bend.json`); arm loads rerun with the new tip mass and robot mass in `results/claw_fans_arm_loads.json`.
Forest asked (Nov 9): "maybe we increase the claw/fan size and if it gets too big to graps with, add a bend halfway
through to assist in grasping". Fan data from the makers' 6S bench tables (LIGPOWER pages read Nov 9); rated thrust is
75 % of full, as for today's fan; prop and ESC masses [A]. Central 17 in prop under the dome as before.

| Claw fan (per claw) | Rated / full | Tip | Best pack that passes T/W 1.15 | Hover | T/W |
|---|---|---|---|---|---|
| 2807 + 7.5 in 3-blade (today) | 16 / 21.4 N | 130 g | Tattu 16 Ah only | 4.4 min | 1.05 |
| MN4006 + 15x5 | 15.6 / 20.8 N | 169 g | none | - | 1.01 |
| V3115 640KV + 10.5 in 3-blade | 23 / 30.6 N | 210 g | 2 x DOGCOM 22 Ah | 11.6 min | 1.27 |
| V3115 900KV + 9.5 in 3-blade | 29.7 / 39.6 N | 212 g | 3 x DOGCOM (2 packs pass T/W but draw 222 A of 220) | 13.6 min | 1.45 |
| MN5008 + 15x5 | 21.2 / 28.2 N | 233 g | 2 x DOGCOM | 15.9 min | 1.18 |
| **MN5008 + 16x5.4 (pick)** | **25.1 / 33.5 N** | **237 g** | **2 x DOGCOM** | **17.0 min** | **1.35** |
| MN5008 + 17x5.8 | 27.8 / 37.1 N | 246 g | 2 x DOGCOM | 17.3 min | 1.46 |

- The 9 to 10.5 inch 3-blade FPV props are no more efficient than today's fan; the gain comes from the big 2-blade
  props on the low-KV MN5008 (about 210 to 240 W per fan at 17 N against about 395 W today).
- Pick: MN5008 KV400 + 16x5.4 carbon 2-blade with 2 x DOGCOM 22 Ah 5C semi-solid (2.78 kg). 17.0 min hover at
  2.75 kW (131 A against the packs' 220 A), T/W 1.35, robot 18.83 kg. With the bent claw below: 16.0 min, T/W 1.29,
  19.13 kg, 140 A. Three packs: 21.8 min, T/W 1.21. The 17 inch prop adds 0.3 min but takes the horn bearing to 1.06
  of its static rating at full throttle in any direction.
- Climb at rated thrust draws about 205 A, above two XT90s' 180 A continuous [A]: short climbs, or a heavier connector.
  Full throttle (about 310 A) is past the packs' 220 A continuous and inside their 440 A burst.

Arm with the 16 inch fan (237 g tip, 18.83 kg robot; the bent claw adds 38 g more):

| Check | Today | 16 inch fan | Fix |
|---|---|---|---|
| Horn bearing, rated, fan within 30 / 60° of up | 191 / 249 N | 347 / 383 N (0.42 / 0.46 of C0) | fan guard within 60° of up |
| Horn bearing, full throttle, any direction | 511 N | 755 N (0.91 of C0) | the same guard |
| Printed idler stub, rated within 30° | 8.2 MPa | 14.5 MPa (limit 12.5 at 2x) | 8 mm 7075 idler pin on joints 1 to 4, as on joint 0 |
| Holding the arm straight out, fans off | joint 3 at 0.89 of stall | joint 3 at 1.06, joint 6 at 0.99 | 4 to 5 N of fan lift while reaching out: 0.43 / 0.26 |
| Random poses the servos can hold at hover | 42 of 65 | 23 of 65 | flight controller chooses the arm shape in flight |
| Hard landing, stance poses held | 3 of 4 | 2 of 4 | land on the fans, not the feet |

Bent claw (concept, `claw_bend.py`; CAD after the walking question below):
- A 16 inch disc is too big for the old pan-tilt wrist: at the 60 mm offset it tilts only about 9° before reaching the
  arm. So the fan is fixed on the claw axis, motor base on the tip plate and prop outermost. Its push runs along the
  last link and adds no moment at the tip joints, the arm hides in the motor's wake, and the 13 joints aim it.
- The jaws can't sit past a spinning prop, so the claw goes round the disc edge: a 16 x 14 mm carbon tube runs out
  sideways from the motor mount, 25 mm behind the prop, to 233 mm from the axis (first half, 30 mm past the disc edge),
  bends 90° forward (the bend, fixed), and the two-finger jaws reach 90 mm past the prop plane (second half, 115 mm).
- Passive, not actuated: a servo at the bend would add 10 to 25 g at 233 mm for no grasp it can't already make. About
  37 g per claw, 2 % of the disc shadowed by the tube [A: about 3 % more hover power].
- Grip: a 0.5 kg object at 233 mm is 1.14 N.m on the tip joints, 0.39 of the STS3215's stall.
- Grabbing off the floor: keep the prop side tipped up. With the fingertips down, the disc clears the floor up to 11.7°
  of tilt toward the prop and 23.9° sideways; tipping away is unlimited.
- The jaws can't be the foot: 26.4 N of foot load at 233 mm is 6.15 N.m, 2.1 times the tip servo's stall. Walking has
  to land on the arm itself, a little back from the claw (as a real octopus walks), with the claw held up.

Found on the way (not caused by the fans): in the study's four walking stance poses, joints 6 and 9 (STS3215) need
1.34 and 1.20 times their stall at 17.18 kg (1.42 and 1.27 at 18.83 kg). The stance IK places the feet and keeps the
claw vertical but does not look at torque; a torque-aware stance, or walking on the arm's underside, is the next check.

## Folding fan pod (2026-11-10)

Forest picked the folding fan pod (option 1 of the powered-bend options). CAD: `arm-pin-hinge/fan_pod.py`
(`out_1m_13/fan_pod/`: `stl/palm_finger_x8.stl`, `stl/fan_pod_x8.stl`, assemblies at 0, 45 and 88°, `fan_pod.json`).
Arm loads, walking and hover: `joint-study/sim/fan_pod_loads.py` (`joint-study/results/fan_pod_arm_loads.json`).
It replaces the fixed bent claw.

How it works: the MN5008 and 16 inch prop sit on a pod hinged 34 mm above the tip plate. The hinge axis crosses the
fan axis, so thrust passes through the hinge at every angle and the pod servo carries no thrust. An STS3032 (20 g,
0.44 N.m stall) swings the pod; a thumb pad on the pod closes against a fixed finger on the palm. The pod is the
thumb, so the claw grips with the fan itself.

| Item | Value |
|---|---|
| Travel | -4 to +88° (straight up to folded aside); the empty claw closes at -5.5°; no clash over the travel |
| Opening | 4 mm at 0°, 23 mm at 30°, 46 mm at 60°, 67 mm at 88° |
| Prop plane | 102 mm above the tip plate, 23.5 mm above the finger top (21 mm prop adapter, for the parking collar) |
| Folded | disc plane 68 mm from the arm axis, 39 mm clear of the arm |
| Grip | 4.8 N at half stall (holds 0.59 kg), 9.7 N at stall (1.18 kg) [A: friction 0.5] |
| Pod weight on the servo | 0.056 N.m, 0.13 of stall |
| Gyro load folding with the prop spinning | 0.12 N.m at 0.5 rad/s (0.26 of stall): fold slowly or with the prop stopped |
| Mass | palm, finger and parking post 30.9 g, pod 13.6 g, servo 20 g, hardware, collar and pads 10 g: 74.5 g; 252.5 g at the tip with motor, prop and ESC |

Flight (2 x DOGCOM 22 Ah, central 17 in prop under the dome): robot 18.95 kg with the parking parts, 16.9 min hover at 2.77 kW (132 A),
T/W 1.34. That is the plain 16 inch fan's figure within 0.1 min, and better than the fixed bent claw (19.13 kg,
16.0 min, T/W 1.29): no carbon tube and no shadowed disc.

Arm: the same as the plain 16 inch fan within 1 %. Horn bearing 345 / 383 N rated within 30 / 60° of up (0.46 of
C0), 756 N full throttle any direction (0.91). Printed idler stub 14.4 MPa, so the 8 mm 7075 idler pins on joints
1 to 4 stay. Straight out with fans off: joint 3 at 1.08, joint 6 at 1.01 of stall; with 4 N of fan lift 0.45 / 0.27.
Moving the ESC up to joint 10 changes nothing measurable (0.02 at most), so it stays at the tip by the motor.

Walking on the folded pod (Forest, Nov 10: "Folded pod as foot"). Script: `joint-study/sim/pod_stance.py`
(`joint-study/results/pod_stance.json`). The foot is the fixed finger's tip, 42.5 mm off the arm axis and 78 mm out,
with the pod folded 88° and the prop parked blades-across. 26.9 N per foot (7 of 8 feet down, 18.95 kg).

- The study's stance poses, standing on the finger as they are: joints 6 and 9 at 1.44 and 1.24 of stall.
- Torque-aware stance: the 13 joint angles are chosen to minimise the worst joint's load, with the finger on the
  ground, the arm tip within 10° of straight down (so the parked blades stay level), the arm clear of the ground,
  the central 17 in disc and both neighbour arms, and the parked blade tips 15 mm or more above the ground and clear
  of the neighbours' blades and the central disc. The arm arches up over its root and comes down nearly vertical
  onto the finger, so the foot load runs along the lower arm instead of across its joints.

| Hub height | Every joint, nominal | Worst over the step (±45 mm sideways, 34 mm in) | Same poses, twice the foot load | Largest joint move between step poses |
|---|---|---|---|---|
| 200 mm (the study's) | 0.16 of stall | 0.43 (joints 9 to 11) | 0.99 | 28° |
| **250 mm (pick)** | **0.15** | **0.33** (joints 5, 9 to 11) | **0.73** | 51° (joint 10, one step) |

- No stronger servos, gearing or fan lift needed: joints 6 and 9 drop from 1.44 / 1.24 to 0.25 / 0.33 at most.
  Feet sit about 250 mm from the body's centre (216 mm on the inward step).
- 250 mm keeps twice the foot load (a foot slipping, uneven ground) under stall and lifts the central disc 50 mm
  higher off the ground. The poses are static: moves between them are still to be checked.

Prop parking (CAD in `fan_pod.py`): before a foot comes down the claw motor is braked to a stop (ESC brake on stop),
then the pod folds. Over the last 1.6° of the fold (86.4 to 88°) a 2-lobe index collar on the prop adapter lands on
a sprung post on the palm; the lobes' 35° slopes turn the stopped prop until the post sits in a valley, which leaves
the blades along the hinge axis, and the post then holds it there while walking.

- Collar: 11 mm radius at the valleys, 2 mm lobes, 55 to 60 mm out from the hinge. The prop adapter grows from 5 to
  21 mm so the collar sits past the bell's sweep; the post stays 4.9 mm clear of the blade roots at every fold angle
  and nothing clashes over the travel.
- Post spring 2 N [A]: 0.017 N.m to turn the prop, and 0.115 N.m on the pod servo at the end of the fold (0.40 of its
  stall with the pod's weight). The stopped MN5008's cogging and bearing drag must stay under 0.017 N.m [A: measure
  on one motor before printing; a stiffer spring or steeper lobes if not].
- The flight controller keeps a claw motor disarmed while its pod is past 10° (it reads the pod servo's position).
- Cost: about 5 g per claw (collar, longer adapter, post and spring): robot 18.95 kg, 16.9 min hover, T/W 1.34 (both
  within 0.05 min and 0.003 of the pod without parking).

Moves on the folded pod (Forest: "Yes, check the moves", Nov 10; `pod_gait.py`, `pod_fold.py`, `pod_fold_move.py`).
One arm is traced pose by pose through a full step at hub 250 mm, 7 feet down, 26.6 N on the foot, prop parked: the
90 mm stance stroke (9 poses), unload, a 45 mm lift, the swing back, set-down and reload. Each pose is solved for the
least worst need/stall with a penalty on joint moves from the last pose, and the straight servo move between poses is
checked at 1/4, 1/2 and 3/4 (load, clearance, how far the finger strays from its line). Limits are need/stall at or
under 0.67 (1/1.5 dynamic allowance), no part inside its clearance margin, and the arm not passing through itself.

| Step direction | Worst joint over the step | Worst on a move between poses | Joints 6 / 9 | Tip servos 11 / 12 | Result |
|---|---|---|---|---|---|
| Sideways (along the body's edge) | 0.37 (joint 6, end of stroke) | 0.42 | 0.37 / 0.18 | 0.35 / 0.11 | clean |
| Diagonal | 0.45 (joint 6, reload) | 0.51 | 0.45 / 0.41 | 0.42 / 0.21 | clean, with 5 extra waypoints |
| In and out, stroke centred on the stance | 0.49 (joint 11, inner end) | 0.46 | 0.32 / 0.21 | 0.49 / 0.18 | fails: inner end runs 16 mm into the next arm at pickup |
| In and out, stroke 25 mm further out | 0.41 (joint 11) | 0.49 | 0.35 / 0.28 | 0.41 / 0.20 | clean |

- Two faults turned up on the way and are now hard limits in the solver: some solved poses had the arm coiled through
  itself (up to 30 mm of overlap), and one straight servo move on the diagonal swung the parked blade 18 mm into the
  neighbour's tip link. Fixes: self-overlap and the clearance of the poses in between are constraints, and a move that
  still clashes gets solved waypoints.
- Lift, swing and set-down never go past 0.23. The reload at the front of the stroke and the ends of the stance stroke
  carry the peaks. The pod servo holds the folded pod at 0.15 of stall the whole time.
- Controller rules that come out of this: send waypoints no more than about 5 mm of foot travel apart while a foot is
  loaded (one 11 mm step let the finger stray 9 mm off its line); keep in-out strokes centred 25 mm outside the
  stance's foot point (or limit them to the outer half).

Fold and unfold on a standing robot (the arm unloads, lifts 45 mm, raises, then the pod swings 0 to 88°):
- Fold pose: the arm stands straight up off its root (joint 0 at 87°, the rest within 5° of straight). Over the whole
  fold every arm joint is at 0.01 of stall, the pod servo at 0.15 rising to 0.41 at 88° where the parking post spring
  comes in, and the turning disc stays 33 mm clear of its own arm (its nearest part), 0.5 m from the neighbours and 1 m
  above the ground. Folding with the arm low is not possible: the 406 mm disc hits the ground.
- Raising the arm: a straight servo move from the lifted foot to the fold pose peaks at 0.87 of stall (over the limit)
  and brushes the next arm. A planned path fixes it: pull the finger in toward the root while low, rise close to the
  root, then stand up (36 solved waypoints). It peaks at 0.45 (joint 5) during the pull-in, with joints 6 / 9 at 0.40 /
  0.37 and nothing touching. The tip joints unroll last, at 0.19.
- So a take-off or landing changes one arm at a time: raise, fold or unfold, lower. The other seven carry the robot.

## Claw prop clearance while walking (2026-11-09)

Script: `joint-study/sim/prop_clearance.py`; numbers: `joint-study/results/prop_clearance.json`. The four
stance poses from the study's foot IK and the same feet lifted 45 mm, with the prop disc (7 or 7.5 inch)
checked against the ground, its own arm, the two neighbouring arms and the mantle, for 600 fan directions,
with 15 mm clearance [A]. The wrist pivot height above the ground and the pivot-to-disc offset were swept,
since the claw is not designed yet.

Result: with a foot down, no fan direction within 30° of straight up is clear in any combination. The
arm's own last links come down onto the claw almost vertically and sit in the disc. The ground,
neighbouring arms and mantle are not what blocks it.

| Wrist pivot above ground | Disc offset | Share of directions clear | Steepest clear direction |
|---|---|---|---|
| 0 mm | 60 / 90 mm | none | none |
| 0 mm | 120 mm | 1 % | 40° from up, back toward the body |
| 80 mm | 90 mm | 4 to 7 % | 57° from up |
| 120 mm | 60 mm | 24 to 39 % | 78° from up |

So the claw fans stay stopped while their foot is down, and a stopped 7 inch prop still needs the wrist
pivot about 120 mm off the ground to clear it, unless the props fold [A: folding 7 inch props for a 2807
not checked]. For take-off, either the EDF and the feet start the lift and the fans spin up once each claw
is clear, or each arm first tips its claw about 40 to 60° back toward the body. The fan guard from the
joints check (within 30° of up) only applies in flight.

## Mantle ports (2026-11-09)

Script: `arm-pin-hinge/mantle_notch.py`. The mantle CAD is not in this folder, so the ports are written as
cutters to subtract from it: `out_1m_13/mantle_port_cutter_joint0_frame.stl` (one port, joint-0 frame),
`mantle_port_cutters_x8_r110.stl` (all eight at the study's 110 mm hub radius [A]),
`mantle_port_template.dxf` and `.svg` (the outline in the flange-face plane; the notch is orange, the old
opening blue), and `mantle_ports.json`. The opening is 61 x 42 mm, running from 35.9 mm on the notch side
to 25.0 mm on the other along the hinge axis, and ±21.2 mm across. Every flange hole is at least 4.5 mm from
its edge.

- The root flange's two ring holes at 30° and 330° sat 0.35 mm from the new opening; they moved to 45° and
  315° in `root_gear.py` (`FLANGE_HOLE_DEG`). Masses and checks are unchanged.
- At a 110 mm hub radius the eight root flanges overlap their neighbours (408 mm³). They clear from a hub
  radius of 124.5 mm (flange faces 83.8 mm from the centre). The real mantle's radius decides whether this
  matters.

## Claw pickup capacity (2026-11-09)

Pickups are by the claw only (Forest dropped the electromagnet on Nov 9). With the 130 g tip, the arm lifts
0.4 to 0.6 kg from the walking stance with the claw down by the body at Forest's 2x rule (1.2 to 1.7 kg to
stall). Reached out straight, it can lift almost nothing extra: joint 3 is already at 89 % of stall holding
the arm straight out with the fans off.

## Claw fans (spec, nothing bought)

The 13-joint arm flies only if each claw fan gives about 16 N, against 10.8 N today. At 15.07 kg
(with the root gear) the robot needs 147.8 N to hover; with the EDF at 42.2 N each claw fan carries
13.2 N, which is 83 % of 16 N, so the fan has to hold 13.2 N continuously, not just reach 16 N in
bursts. Before the root gear these were 14.71 kg, 144.3 N and 12.8 N.

| Item | Spec |
|---|---|
| Thrust | 16 N per fan at about 75 % throttle, 13.2 N continuous at hover |
| Class | 7 to 7.5 inch prop on a 2807 motor, 1250-1300 KV, 6S |
| Reference data | T-Motor 2807 1250KV, 7.5x4.6x3 prop, 6S bench table: 1183 g at 60 %, 1463 g at 70 %, 1739 g at 80 %, 2178 g (21.4 N) at 100 %, 44.3 A and 551 W at full; motor 52.6 g |
| Power per fan | about 270 W at hover (13.2 N, scaled from 260 W at 12.8 N), about 360 W at 16 N, 551 W at full |
| Power, eight fans | about 2.2 kW at hover, before the EDF |
| ESC | 45 A or more per fan |
| Mass at the tip | the sim allows 130 g for claw, motor, prop and ESC (90 g before); with a 53 g motor the claw, prop and ESC share about 77 g |

Why not 5 inch: a 2207 1950KV on a 5 inch prop at 6S peaks at 1544 g (15.1 N, 788 W) on the
LIGPOWER bench, so 16 N is out of reach and 13.2 N hover sits above 90 % throttle.

Tip wrist: the pan-tilt gimbal still works with the larger disc. With the arm's 57.1 mm diameter
as the obstacle, thrust-sphere coverage at a 60 mm fan offset is 0.72 for a 90 mm disc, 0.65 for
152 mm and 0.62 for 190 mm (`joint-study/results/claw_fan_coverage.json`). Keep the fan at least
60 mm out from the wrist pivot.

Not yet modelled: the power wiring along each 1 m arm (about 25 g per arm for a 16 AWG pair),
the prop's clearance to the ground and to the claw when walking, and battery capacity for 2 kW
plus the EDF.

## Geometry at 585 mm (pin_cad.py defaults)

| Item | Value |
|---|---|
| Joints per arm | 9, one 12 V bus servo each: STS3250 in joints 0-3, STS3215 in joints 4-8 |
| Joint pitch | 65.0 mm (shortest pitch that clears: 60.5 mm) |
| Hinge axis twist | 60° per joint |
| Commanded travel | ±45° per joint (collision-free to ±90°) |
| Outer diameter | 57.1 mm (set by the fork ears and the idler screw head) |
| Arm length | joint 0 axis to tip face, 585 mm |

Servo stack along the hinge axis, measured from the arm centreline (mm): idler screw head −28.3,
idler ear −26.1 to −22.9, PTFE washer, cradle floor −22.4 to −20.0, servo case −19.5 to +15.5, horn
+19.5 to +22.5, horn ear +22.5 to +26.5. The case is offset 2 mm toward the idler side so that the
horn ear and the idler screw head reach the same radius.

## Parts per arm

| Part | Qty | Printed mass | Notes |
|---|---|---|---|
| root_socket_x1.stl | 1 | 23 g | holds servo 0; 69 mm flange, 6 x M3 on a 62 mm circle into the mantle |
| vertebra_x8.stl | 8 | 23 g each | fork for the previous joint + cradle for the next servo |
| tip_vertebra_x1.stl | 1 | 32 g | fork for joint 8 + claw plate (4 x M3 inserts on a 30 mm circle) |
| servo_lid_x9.stl | 9 | 2.5 g each | clamps each servo into its cradle, 4 x M3 |

Hardware per joint (about 5 g): 4 x M3x6 button-head screws through the horn ear into the horn,
1 x M3x6 button-head screw and M3 washer retaining the idler ear, 1 x PTFE washer (8.4 x 14 x 0.5 mm)
between idler ear and cradle floor, 4 x M3 heat-set inserts for the lid, 1 x M3 heat-set insert in
the idler stub. Arm mass outside the mantle, with servos and hardware: about 784 g
(884 g counting the socket, root servo and its lid; the claw is extra).

Servos per arm: 4 x STS3250 (joints 0-3, 74.5 g, 4.9 N m stall) and 5 x STS3215 (joints 4-8, 55 g,
2.9 N m stall). For eight arms that is 32 STS3250 and 40 STS3215. The STS3250s add about 0.62 kg to
the robot and about $670 at catalog prices against an all-STS3215 build.

## Fit and collision check (pin_cad.py)

- Every servo sits fully inside its pocket with 0.5 mm clearance; no part overlaps another with all
  joints at zero.
- Each joint turns −90° to +90° before any contact, servo cases and horns included, against the
  ±45° commanded range.
- With two neighbouring joints turned together to ±60° (same and opposite sense), links two apart
  stay clear.
- Vertebra 2 equals vertebra 1 turned 60° and shifted one pitch to within 0.00 mm³, so one STL
  serves all moving vertebrae.

## Simulation at CAD size

The joint study was rerun with this CAD's diameter (57.1 mm) and per-joint mass (29 g of printed
structure and hardware) applied to every servo-per-joint design. The arm as drawn was evaluated with
four servo fits; the STS3250 shares the STS3215 case, so the printed parts do not change.

| Servos per arm | Robot | T/W | Wave tip RMS | Lift tip RMS | Flight-stress torque | Score after dynamics |
|---|---|---|---|---|---|---|
| 9 x STS3215 | 10.33 kg | 1.27 | 21.8 mm | 84.3 mm | 116 % of stall | 0.332 (18th of 26) |
| STS3250 in joints 0-2, STS3215 beyond | 10.79 kg | 1.21 | 16.4 mm | 12.5 mm | 80 % | 0.345 |
| **STS3250 in joints 0-3, STS3215 beyond (main design)** | **10.95 kg** | **1.20** | **14.6 mm** | **8.9 mm** | **72 %** | **0.413 (would rank 4th of 26)** |
| 9 x STS3250 | 11.73 kg | 1.12 | 13.5 mm | 36.9 mm | 66 % | fails the T/W gate |

- Shape miss over the seven target shapes is 26.4 mm for all four (9 DOF, ±45°).
- With nine STS3215s the root joint reaches stall when the arm is lifted against gravity, which shows
  as the 84 mm lift-test error; walking (35 % of stall) and hover are within capacity.
- STS3250s in the first four joints were adopted on 2026-11-09: the lift error falls to 8.9 mm and
  the worst flight-stress torque to 72 % of stall, for 0.62 kg on the robot. Thrust-to-weight is
  1.20 against the 1.15 gate. The printed parts are unchanged.
- At the real (longer) arm length the root torque rises with the square of the length, so the root
  servo choice is to be rechecked with the rerun.

## Arm length 1 m: options compared (2026-11-09)

At 1 m the 9-joint arm no longer flies on today's thrust: every 1 m arm with more than 7 joints
falls below the thrust-to-weight gate of 1.15. Three builds were generated with pin_cad.py and
simulated; all use one repeated vertebra and are collision-free to ±90° per joint. Forest chose (b).

| Option | (a) 1 m, today's thrust | (b) 1 m, 16 N claw fans | (c) 700 mm, today's thrust |
|---|---|---|---|
| Joints | 7 | 13 | 9 |
| Servos | STS3250 in 0-2, STS3215 in 3-6 | STS3250 in 0-4, STS3215 in 5-12 | STS3250 in 0-3, STS3215 in 4-8 |
| Pitch | 142.9 mm | 76.9 mm | 77.8 mm |
| Vertebra | 73 g | 31 g | 32 g |
| Arm outside mantle | 920 g | 1,243 g | 854 g |
| Robot mass | 11.38 kg | 14.71 kg | 11.36 kg |
| Thrust-to-weight | 1.15 | 1.18 | 1.15 |
| Shape miss (as built) | 60.0 mm | 18.8 mm | 30.7 mm |
| Wave tip RMS, 0.25 / 0.5 / 1 Hz | 20 / 97 mm / saturates | 38 / 106 mm / saturates | not run / 12 / 27 mm |
| Lift-test tip RMS | 138 mm | 345 mm | 6 mm |
| CAD | `out_1m_7/` | `out_1m_13/` | `out_700_9/` |

Notes:
- Option (b) assumes each claw fan gives 16 N instead of 10.8 N (48 % more fan thrust) and weighs
  40 g more. The fan is specified under "Claw fans" above.
- A 1 m arm needs about 4 % more thrust than today's for 9 joints, 19 % for 13 and 31 % for 16.
- A 1 m arm moves about four times slower than the 585 mm arm: the root torque grows with the
  square of length at fixed mass per metre, and every 1 m design saturates in the 1 Hz wave test.
- With today's fans, 9 joints fly up to 800 mm (all STS3215 at 800 mm, thrust-to-weight 1.18).
- Stronger root drives were modelled afterwards; see "Root fix".

Regenerate with:

    python3 pin_cad.py --length 1000 --joints 7 --root-servos 3 --out out_1m_7
    python3 pin_cad.py --length 1000 --joints 13 --root-servos 5 --out out_1m_13
    python3 pin_cad.py --length 700 --joints 9 --root-servos 4 --out out_700_9

## Printing and assembly

- Material PETG, 0.6 mm extrusion width, 3 perimeters, 4 top and bottom layers, 25 % gyroid. The
  masses above assume near-solid walls (0.95 of solid).
- Vertebra orientation: arm axis vertical, cradle up, fork ears down. The part has no flat end, so
  supports are needed: tree supports under the two ear bosses and under the bulkhead spokes inside
  the ring. The side-wall and floor windows bridge at 20 mm or less. The horn and stub holes print
  horizontal; a 0.2 mm teardrop or a drill pass cleans them up.
- Lid: flat on the bed. Socket: flange down. Tip: claw plate down.
- Heat-set inserts: 4 per cradle into the lid bosses from the lid side, 1 into the idler stub.
- Assembly per joint: drop the servo into the cradle (bus cable out through a gap between the
  bulkhead spokes), screw the lid down, fit the horn to the spline, slide the next vertebra's idler
  ear over the stub with the PTFE washer under it, bolt the horn ear to the horn with 4 screws, then
  fit the idler screw and washer. Centre the servo (position 2048) before bolting the horn so ±45°
  is symmetric.
- The bus cable runs from servo to servo inside the arm, through the bulkhead gaps; leave a service
  loop of about 30 mm per joint so ±45° bends do not pull on it.

## Values to measure before printing ([A] in pin_cad.py)

- Spline centre to the near end of the servo case (11 mm assumed).
- Horn seat above the case top (4 mm), horn thickness (3 mm), horn radius (10 mm), horn bolt circle
  and thread (14 mm, M3).
- Insert hole size for the inserts in hand (4.0 mm hole for 4.5 mm M3 inserts).

## Regenerating

    python3 pin_cad.py --length 1000 --joints 13 --root-servos 5 --out out_1m_13   # main design
    python3 pin_cad.py                          # 585 mm, 60° helix, joints = as many as fit
    python3 pin_cad.py --length 900             # real arm length; joint count follows
    python3 pin_cad.py --length 900 --joints 13 # fixed joint count
    python3 pin_cad.py --twist 90               # alternating axes instead of the helix
    python3 pin_cad.py --root-servos 0          # STS3215 in every joint (default 4 x STS3250)

The study is rerun at a new length by setting `ARM_LENGTH` in `joint-study/sim/config.py` and
running `run_study.py`, `rank.py` and `build_page.py`. At a longer arm the root joints carry more
torque than an STS3215 gives; the study already tries larger root servos (STS3250) for graded
arms, and the STS3250 shares the STS3215 cradle. If the longer arm needs more than an STS3250 at the
root, a larger root cradle is the next CAD change.

## Files

- `arm-pin-hinge/pin_cad.py`: the generator.
- `arm-pin-hinge/root_gear.py`: redraws the root socket around the 2:1 gear stage and reruns the
  checks (`python3 root_gear.py --length 1000 --joints 13 --root-servos 5 --out out_1m_13`).
  `ROOT_BRG=6800` draws the socket as it was before the root bearing check.
- `joint-study/sim/bearing_loads.py`: the root bearing check (`joint-study/results/bearing_loads.json`).
- `arm-pin-hinge/horn_bearing.py`: the horn-side support bearing for joints 1 to 4 (`out_1m_13/horn_bearing/`).
- `joint-study/sim/central_prop.py`: the central prop sizing and placement, above or under the dome.
- `joint-study/sim/claw_fans.py`, `claw_bend.py`: bigger claw fans against the packs, and the bent claw.
- `joint-study/sim/pod_gait.py`, `pod_fold.py`, `pod_fold_move.py`: one arm's moves through a step and the fold on a standing robot (results `pod_gait_*_v4*.json`, `pod_fold.json`, `pod_fold_move_fine.json`).
- `arm-pin-hinge/fan_pod.py`, `joint-study/sim/fan_pod_loads.py`, `pod_stance.py`: the folding fan pod CAD with its prop parking,
  its arm and hover checks, and the torque-aware stance on the folded pod.
- `joint-study/sim/joint_loads.py`, `battery.py`, `prop_clearance.py`: the joints 1 to 4 check, the battery
  sizing and the claw prop clearance (`joint-study/results/joint_loads.json`, `battery.json`,
  `prop_clearance.json`).
- `arm-pin-hinge/mantle_notch.py`: the mantle port cutters and templates (`out_1m_13/mantle_port_*`).
- `arm-pin-hinge/out_1m_13/`: the main design. `stl/` holds the printable parts and the gear and
  pinion and the idler pin (reference models for 7075); `assembly_*.stl` the straight, curl and wave poses and the
  geared root alone; `fit_report.md`, `fit.json` and `root_gear.json` the checks;
  `mantle_cutout_joint0_frame.svg` the mantle notch.
- `arm-pin-hinge/out/`: the 585 mm build. `out_1m_7/` and `out_700_9/`: the options not chosen.
- `joint-study/`: the study that chose this design, with the results page and write-up.
  `joint-study/sim/fix_root.py` and `joint-study/results/root_fix.csv` hold the root-fix runs.
- `arm-canted-hinge/`: the earlier canted-hinge CAD, kept for reference.
