# Centauri-1 extended bounding leap and landing

Centauri-1 is Cephalo-2 docked on the Hippos-1 horse body. This folder holds a physics study of one full bound:
gather, push off, all four hooves off the ground with the fores reached fully forward and the hinds fully back,
fores land first, hinds follow, and the robot comes to rest. The study runs on the Hippos-1 rev C leg CAD
(snapshot in `legs_revC_snapshot/`, read-only copy of the leg thread's files) with motor torque and speed limits,
the gas struts, the elbow / hock tendons and a load and current monitor on every run.

Live 3D replay (drag to turn): https://claude.ai/artifact/S7ge6U6Sh6jJYmgdEjGyCh

## What existed before

Only the Nov 20 side-view plan of an extended leap (massless legs, 249 mm CoM rise, 908 mm span). It had no leg
masses, motor limits, landing or loads. Nothing from it is reused except the extension targets.

## Result (baseline, 39.6 V bus)

| | |
|---|---|
| Hind take-off to fore landing (hoof marks) | 651 mm |
| Body travel while all four hooves are off | 131 mm forward |
| Flight time | 227 ms |
| Centre of mass rise | 114 mm (apex 564 mm) |
| Full extension in the air | reached within 2.6 deg of the target (tolerance 3 deg): fores q0 57 / q1 52, hinds q0 -6 / q1 52 |
| Landing order | fores at 1.136 s, hinds 115 ms later |
| End state | at rest, level (pitch under 0.1 deg), 232 mm ahead of the start |
| Pitch during the bound | -12.6 to +14.8 deg |

The bound carries forward more than it rises (131 mm of air travel against 114 mm of rise). It is short because
all four motors are at peak torque during the push; this is the limit of the rev C actuators, not of the controller.

### Joints (one leg; the pair mirrors it)

| Joint | Motor | Peak used / available at the joint | RMS (rated) | Peak speed / no-load | Peak phase current | Range used (limits) |
|---|---|---|---|---|---|---|
| Fore pivot q0 | RobStride 03 | 60.0 / 60.0 N m | 15.1 (20) N m | 990 / 965 deg/s | 25.4 A | 7.5 to 58.6 (0 to 60) |
| Fore flex q1 | RobStride 04 + 46:22 belt | 54.5 / 54.5 N m | 13.1 (18.2) N m | 1525 / 2070 deg/s | 50.0 A | 50.9 to 123.0 (50 to 140) |
| Hind pivot q0 | RobStride 06 | 36.0 / 36.0 N m | 14.0 (11) N m | 446 / 2376 deg/s | 33.0 A | -8.6 to 35.1 (-9 to 67) |
| Hind flex q1 | RobStride 06 | 36.0 / 36.0 N m | 16.7 (11) N m | 1261 / 2376 deg/s | 33.0 A | 51.5 to 113.2 (50 to 146) |

- Every joint reaches peak torque for part of the push. The fore pivot also briefly runs slightly past its
  no-load speed in the air (the leg's own swing carries it), so it regenerates for a moment.
- The hind motors run above their rated RMS over the 3.2 s run. One leap is fine; leaping back to back would heat them.
- No joint hits a hard stop (stop load 0 N m in the baseline).

### Electrical

- Bus current peak 144 A (5.7 kW at 39.6 V) during the push, regeneration peak -24 A on landing.
- Bus energy for the whole run (stand, gather, leap, landing, settle): 1.77 kJ.

### Landing and structure

| | |
|---|---|
| Fore pair peak ground force | 1921 N (4.7 x robot weight; about 960 N per fore hoof) |
| Hind pair peak ground force | 820 N (2.0 x weight) |
| Friction used | 0.7 of 0.7 (the hooves skid briefly at touchdown) |
| Hoof skid under load | 131 mm in all (left fore + left hind), peak 0.99 m/s |
| Elbow / hock tendon peak | 742 N, safety factor 21.6 on the doubled 3 mm Dyneema line |
| Gas strut shortest | 91.4 mm (bottoms at 84 mm) |
| Fore to hind leg gap (math, 30 mm capsules) | 308 mm minimum |
| Lowest hoof during flight | 10 mm off the floor |
| Body touches the floor | never |

## Does it still land if the estimates are off? (`sensitivity.json`)

Same commands, changed robot. All nine cases land fores first and come to rest.

| Case | Span mm | Air travel mm | Flight ms | Rise mm | Pitch range deg | Fore landing N | Hard-stop hit N m |
|---|---|---|---|---|---|---|---|
| Baseline | 652 | 132 | 228 | 114 | -12.6 / 14.8 | 2054 | 0 |
| Body pitch inertia 1.0 kg m2 | 640 | 131 | 220 | 112 | -14.0 / 14.9 | 1943 | 0 |
| Body pitch inertia 1.9 kg m2 | 657 | 130 | 230 | 115 | -11.9 / 14.6 | 1928 | 0 |
| CoM 30 mm lower | 639 | 127 | 214 | 109 | -15.7 / 14.2 | 1927 | 0 |
| CoM 30 mm higher | 667 | 134 | 244 | 122 | -8.7 / 15.5 | 1958 | 67 (hind pivot) |
| Bus 36 V | 646 | 130 | 226 | 114 | -13.0 / 14.8 | 1938 | 0 |
| Bus 48 V | 661 | 136 | 230 | 115 | -12.0 / 14.8 | 1954 | 37 |
| Floor grip 0.5 | 571 | 86 | 214 | 108 | -18.0 / 14.2 | 1768 | 0 |
| Body +1.5 kg | 646 | 125 | 222 | 111 | -12.4 / 15.0 | 1910 | 41 |

(The sensitivity runs sample the monitor every 2 ms, so their baseline differs from the 1 ms record by a few per cent.)

An earlier tune that ignored these cases flipped over when the body pitch inertia was 1.0 kg m2; the final tune
was searched over the baseline, both inertias and grip 0.5 at once.

## Estimates and what is not checked

- Body (with Cephalo-2 docked) 24.18 kg, CoM placed so the whole robot's CoM sits at (87.5, 450) mm as in the
  Nov 20 study; body pitch inertia 1.4 kg m2 is an estimate (range 1.0 to 1.9 tested). Whole robot 41.65 kg.
- Motor envelope: full peak torque up to half the no-load speed, then linear to zero when driving; full peak when
  braking. No-load speed scales with bus voltage.
- RobStride 04 torque constant 2.4 N m/A and resistance 0.25 ohm are estimates (not in the act table).
- Hard stops modelled stiff (MuJoCo solref 5 ms); a softer printed stop would see less peak load.
- Dyneema break load about 8 kN per 3 mm line; floor friction 0.7.
- **Not checked:** legs against the body shell, neck and octopus. The chassis code that draws them (`chassis_v3`,
  which imports `sk3d` / `equo_v2`) is not in this repo or the project files, so the body is a plain box here.
  Leg to leg and every non-hoof contact with the floor are checked.
- Octopus arms are not moved or changed; they ride as part of the body mass.

## Method

1. `build_model.py` turns the rev C leg CAD into a planar MuJoCo model (both legs of a pair get the same command):
   masses and inertias from the CAD solids, rotor inertia reflected through each gear, tendon-slaved elbow / hock,
   hoof ball joints with their rest tilt, the 400 N gas struts as a length-dependent joint torque.
   `python3 build_model.py --mesh` also writes `leg_meshes.json` for the viewer.
2. `sim_core.py`: motor envelope, electrical model (phase current = motor torque / kt, bus power = shaft power +
   copper loss), gas strut force.
3. `leap.py`: a contact-driven phase controller per leg pair (stand, gather, push, extend, prepare / swing, land,
   settle) with PD joint control clipped to the motor envelope and pitch feedback on landing. Its 27 timing,
   pose and gain parameters are tuned by CMA-ES. The cost rewards travel in the air and forward ground travel and
   penalises a short flight, missing the extension, hinds landing first, not ending level and at rest, skidding,
   tendon load, leg-to-leg distance and hard-stop hits.
   - `python3 leap.py search N SEED [--fresh]` (env `CENT_ROBUST=a.xml,b.xml` to tune over several models)
   - `python3 leap.py run` writes `leap_result.json`, `leap_trajectory.csv` (joint angles, torques, ground forces,
     tendon loads every 5 ms) and `leap_replay.json` (viewer frames).
4. `sensitivity.py` reruns the best parameters on changed models.
5. `viewer/index.html` is the 3D replay (needs `leg_meshes.json` and `leap_replay.json` beside it).

Parameters of the final leap: `best_params.json`.
