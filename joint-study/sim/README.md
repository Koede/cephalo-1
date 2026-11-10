# Cephalo-1 tentacle joint study

Screens vertebra-joint designs for the 8-arm flying/walking octopus robot (8.63 kg, 585 mm arms
tapering 40 -> 18 mm, claw-prop at every tip, 42.2 N EDF) and ranks them on DOF, fluidity,
loads, mass and how buildable they are with a printer and hand tools.

## Run it

    python3 -m pip install -r requirements.txt
    python3 run_study.py --workers 12      # static screen of every arm configuration -> results/sweep.csv
    python3 rank.py --workers 12           # validation, MuJoCo shortlist, Pareto front, plots, top-arm geometry
    python3 tip.py                         # tip fan wrist study -> results/tip_study.json

`--limit 20` on run_study.py does a smoke run. On 4 cores the full screen takes about 30 minutes;
12 cores should take about 10.

## What is varied

- Joint type = mechanism family x variant (material, bearing, strap) x range. 31 families, 95 joint
  types (catalog/ has the sources for each family and every servo spec).
- Arm configuration = joint type x axis arrangement (planar + root yaw, alternating 90, helix 60,
  alternating 90 with a roll module every 4th node, omni 2-axis, omni + roll) x actuation
  (servo in every node, base-box tendons, base-box push-pull, 3-section tendons, root servos +
  base-box tendons) x joint count (7, 9, 13, 19, 25) x drive (4 local servos, 3 base-box drives,
  3 root drives) x landing compliance (rigid, tendon spring, TPU coupler, overtravel detent,
  magnetic breakaway).

## What is measured (metrics.py)

- Workspace and collisions: 2000 random poses per geometry; capsule self-collision (pairs far enough
  apart along the arm to touch), mantle collision and collision with the neighbouring arm.
- Fluidity: least-squares fit of the arm to 7 target shapes (270 deg curl, log-spiral curl, S-curve,
  helical coil round a 50 mm pole, sideways arc, twisted compound bend, tip hook). Mean miss in mm.
- Walking: claw-down IK to 4 foot positions over the 90 mm stride and 34 mm pull-in at 200 mm body
  height, 12.1 N per foot with 7 feet down; ground clearance of the interior nodes enforced.
- Flight: arm straight, hover thrust 5.5 N tilted 15 deg (gate) and 10.8 N tilted 30 deg (stress).
- Landing: 150 mm drop on 4 feet; foot stiffness from J K^-1 J^T with servo and tendon stiffness,
  compliance as a series spring with a stroke limit, F = mg + sqrt((mg)^2 + 2 mg h k).
- Tendons: capstan loss exp(-mu x wrap), wrap = joint angles crossed + 5 deg routing per vertebra;
  tendon stretch stiffness 2 EA/L r^2; root channel count; base-box volume in the mantle.
- Precision at the foot: backlash, deflection under load, tendon friction hysteresis, coupling.
- Mass and thrust-to-weight (8 x 10.8 N + 42.2 N), cost, assembly hours, printed and hardware
  part counts, tolerance-critical fits, supports, build risk.
- Dynamics (shortlist, MuJoCo): flown travelling-wave curl at 1 Hz, flown step to a 270 deg curl,
  foot-lift cycle; tip tracking error, jerk, drive saturation, sag under hover thrust.

Score = weighted geometric mean of the sub-scores (fluidity and build weigh double, loads 1.5x).
Hard gates: walking reachable, T/W >= 1.15, tendons fit the root vertebra, base box fits the
mantle, mechanism fits its pitch, walking and hover loads under stall, structure under its
allowable. Failing configurations keep half their score so they still show in the plots.

Every number marked [A] in config.py and at the top of metrics.py is an assumption.

## Main design

`config.MAIN_DESIGN` names the adopted arm (pin hinge, 60° helix, local STS3215). `rank.py` always
puts it on the MuJoCo shortlist and first in the 3D bench. `python3 eval_main.py` evaluates it with
four servo fits (all STS3215, STS3250 in joints 0-2 or 0-3, all STS3250) and writes
`results/main_design_variants.csv`; `build_page.py` reads `results/main_design.json` for the
"Main design" section. Servo-per-joint designs use vertebra size and structure mass measured on
the pin-hinge CAD (`chain.CLEVIS_D_SCALE`, `metrics.LOCAL_STRUCT`, `LOCAL_STRUCT_D`).
