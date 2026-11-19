# Hippos-1 legs v3: fresh architecture study (2026-11-18)

Forest: "Before raytracing and all that, let's replan the legs." Picked: **fresh architecture study**; goals **more agile** and **more horse-like**. **Forest picked concept B (horse-true fore and hind), 2026-11-18.** Next: joint-by-joint build study (below), waiting for Forest's picks. No CAD yet.

Code: `claude/code/hippos/legs_v3/legs_v3_study.py` (loads, actuator choice, agility, horse score), `legs_v3_frames.py` (animation frames, fore-hind leg clearance, kneel), `legs_v3_joints.py` (joint ranges, loads, build options for B), `act_table.json` (actuator data the study reads). Rotatable 3D model with all four concepts and seven motions: https://claude.ai/artifact/K4gSq5MYF6YC15miYVcorJ

## Basis
- Same body (0.75 scale): front hip x 219 z 368, rear hip x -150 z 378; Centauri-1 33.4 kg with today's legs (hand3 on all arms, arm tail), front share 60 %, CoM z 500 (est.).
- Cases: stand, walk 0.3 m/s, trot 0.6, fast trot 0.9, gallop 1.6 (duty 0.32, half-sine loads aimed at the CoM, swing inertia), trot + 0.25 g push, get up, jump 100 mm, drop landing from 250 mm, rear up 40 deg. Knee offload spring = 80 % of the standing knee torque on every concept.
- Rule: rated >= 1.2 x rms (stand, walk, trot), peak >= 1.1 x peak (every case), no-load speed at 39.6 V >= 1.5 x gallop joint speed. Knee belt ratio picked from 1.0-3.0 per leg.
- Actuators from `actuators.json` (research 2026-11-18, shop listings; robstride.com gave no content): RS00 5/14 N m 310 g $185, RS02 6/17 405 g $205, RS03 20/60 880 g $275, RS04 40/120 1420 g $296, RS06 11/36 621 g $255, AK70-10 8.3/24.8 621 g $399. The lightest that passes is **RS06 on every pitch and knee joint in all four concepts** (plus RS06 front / RS02 rear abduction): actuators do not separate the concepts, geometry does.

## Concepts and results
| | A current pantograph | B horse-true fore + hind | C agile two-link | D one horse-hind module |
|---|---|---|---|---|
| Legs | today's module, front turned end for end, knee motor on the femur's scapula arm | fore: scapula from a pivot 100 mm higher, humerus, column slaved to the scapula, carpus locked in stance / folded in swing; hind: femur 147, tibia 140, cannon 158 (horse proportions), spring rod; all motors in the body | thigh 215, shank 235, knee motor in the body | B's hind leg on all four, motors in the body, spring rods |
| Four legs / Centauri-1 | 9.6 / 33.8 kg | 10.0 / 34.1 | 9.5 / 33.7 | 9.8 / 34.0 |
| Jump (hoof clearance) | 152 mm | **168** | 159 | 132 |
| Body drop to gallop | **30 mm** (legs stand at 95-98 % of length) | 0 | 0 | 0 |
| Hoof impact mass, worst | 6.7 kg | 2.8 | **0.9** | 1.6 |
| Fore-hind leg gap, worst | +173 mm | +31 (gallop, with its own footfall timing) | **-30 (jump tuck, kneel: knees meet)** | +103 |
| Belly when kneeling | 48 mm | 40 (kneels on the carpi) | 66 | 83 |
| Horse score (7 checks) | 3.9 | **7.0** | 1.8 | 4.0 |
| Tightest margin / required | push, front hip x1.14 | push, front pivot x1.04; trot shoulder x1.13 | trot knee x1.10 | trot rear knee x1.11 |

- Rear-up holds the rear hips at about 26 N m = 2.4 x RS06 rated on every concept: seconds, not minutes (est.). RS03 rear hips (+0.52 kg, +$40 for the pair) would bring it to 1.3 x rated.
- B segments: fore scapula 149, humerus 111, column 275 (carpus at half), pivot (236, 468); hind femur 147, tibia 140, cannon 158, pivot (-135, 378).

## B joint build study (2026-11-18, `legs_v3_joints.py`; sheets joints_1_legs.png, joints_2_options.png)
Ranges over all seven motions (deg): fore pivot 4..120, shoulder flex 54..171, elbow 24..141, carpus 0..150; hind pivot -1..73, stifle 49..171, hock 14..136. Ground load per leg up to 493 N front, 329 N rear (gallop). Joint speeds in the gallop: fore 405 / 700 deg/s, hind 540 / 745 deg/s.

**Finding: the shoulder / stifle motor must ride on the upper segment** (coaxial with the pivot, housing on the scapula / femur). Mounted on the body it drives the lower segment's absolute angle and the pivot motor then carries both joints' torque: 48 N m peak, 24 N m rms in the trot + push (RS06: 36 / 11). On the upper segment the loads stay as the study assumed (pivot 32 peak, shoulder 39 peak).

**Packaging:** the two RS06 of the front pivot must sit one each side of the scapula (pivot inboard, shoulder motor outboard): both inboard would reach y 68 mm, inside the neck sheath block (+-74). Split, the inner face is at about y 118.

| Joint | Recommended | Numbers | Alternatives |
|---|---|---|---|
| Scapula / hip pivot | RS06 direct, outboard stub axle on a 6805 | 493 / 329 N radial | cantilever on the RS06 bearing only |
| Shoulder drive 1.5:1 | 5M belt 25 mm 36T:54T | 911 N peak (working 700, break 7980), 269 N rms | Dyneema 3 mm capstan 20/30 mm (1305 N, SF 5.4); push rod impossible (crank would sweep 177 deg) |
| Stifle drive 1:1 | parallelogram push rod, cranks 40 mm, rod 10 x 8 carbon | 814 N peak, buckling 13 kN; 29 deg transmission only at kneel / tuck | belt 48T:48T (738 N); capstan 3 mm (940 N, SF 7.4) |
| Elbow / hock (slaved) | crossed Dyneema 3 mm tendons on 30 mm drums, series spring in the stance line | elbow 22 N m -> 724 N (SF 10); hock 10 N m -> 331 N (SF 21); no dead point | spring push rod (836 / 383 N, near dead point at the ends); rigid rods rejected |
| Carpus | toggle strut lock + STS3215 on the humerus pulling a release / fold cable, spring re-extends | load folds it in 60 % of the stance samples, up to 11 N m: a stop alone cannot hold it | passive cable from the elbow (no servo) |
| Shoulder / stifle offload | mini gas spring strut | 7.3 / 5.3 N m, about 245 N on a 30 mm lever | shock cord over a cam; torsion spring rejected |
| Fetlock, hoof, abduction | as v2 (Dyneema + coil spring in the cannon, TPU shoe; front abduction behind the sheath block on torque tubes, rear RS02) | | |
| Bearings | pivot stubs 6805; shoulder and stifle 2 x 6805; elbow 2 x 6802 (6800 too small, 610 N each); hock and carpus 2 x 6800 | | |

Not done: dynamics sim (MuJoCo with the overload monitor), shell / pod rework for the higher front pivot, belt tensioner and tendon tensioning details, cable routing across the pivot, the carpus servo on the leg's 12 V bus.
