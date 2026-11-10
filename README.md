# Cephalo-1

Cephalo-1 is an eight-armed flying and walking octopus robot. Each arm is a 1 m chain of 13
pin-hinge joints whose axes turn 60° per joint, so the arm can bend in any direction and coil
into a helix. The full design record, with every check and the reasoning behind each choice, is
in [DESIGN.md](DESIGN.md).

## Folders

- `DESIGN.md`: the main design document (arm geometry, root gear, bearings, battery, central
  prop, claw fans, folding fan pod, mantle ports, printing notes and values to measure first).
- `arm-pin-hinge/`: the CAD generators for the chosen pin-hinge arm (`pin_cad.py`,
  `root_gear.py`, `horn_bearing.py`, `fan_pod.py`, `mantle_notch.py`).
  - `out_1m_13/`: the main design. `stl/` holds the printable parts; `assembly_*.stl` are the
    straight, curl and wave poses; `fan_pod/` and `horn_bearing/` hold those sub-assemblies;
    the mantle port cutters and templates sit alongside.
  - `out/`: the earlier 585 mm build. `out_1m_7/` and `out_700_9/`: options not chosen.
- `arm-canted-hinge/`: the earlier canted-hinge arm, kept for reference.
- `joint-study/`: the joint study that picked the design.
  - `sim/`: the study and check scripts (see `sim/README.md` to run them).
  - `catalog/`: sources for each joint family and servo spec.
  - `results/`: ranked results, check outputs (JSON/CSV) and plots.
  - `writeup/`: the study write-up as an HTML page.
- `spec-sheet/`: the pin-hinge arm spec sheet, the living record for the arm (live page:
  https://claude.ai/artifact/3t777NsUV2NmNSZxF3stNS), with its section scripts, results and the
  90° alternating-axis CAD variant (`cad_alt90/`).
- `servo-clutch-rev4/`: a separate design line from the Octopus Drone project. Its rev 4 has 8 servo-in-link
  joints per arm (478 mm), winch-tensioned detent clutches and 6 in claw-props. The folder holds design docs,
  MuJoCo full-robot sims, clutch part generators and renderers. See its own README.

## Regenerating the main design

    cd arm-pin-hinge
    python3 pin_cad.py --length 1000 --joints 13 --root-servos 5 --out out_1m_13
    python3 root_gear.py --length 1000 --joints 13 --root-servos 5 --out out_1m_13

## Not in this repository

The raw sweep data from the joint study (`joint-study/results/**/sweep*.csv.gz` and
`pareto_front.csv.gz`, about 70 MB) is left out. Running `joint-study/sim/run_study.py` and
`rank.py` rebuilds it.
