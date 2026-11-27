# Hippos-1 legs (leg v3, concept B), rev C and on

Single source for the Hippos-1 leg CAD, checks and motions. Moved here on 2026-11-27 from the Octopus Drone project files
(`claude/code/hippos/legs_v3b/`), where rev B was stored with `revC.patch` on top. `hippos_legs_cad.py` here is rev B
with that patch applied; the patch header's edits 2-5 (verify run list, motion actuators, belt efficiency, motion test
limits) are applied too.

| File | What |
|---|---|
| `hippos_legs_cad.py` | every part as a manifold3d solid in the leg's rest frame + the joint tree -> `legs_cad.json` |
| `legs_cad_verify.py` | clearance sweep over the joint ranges (>= 1.5 mm between moving parts, >= 0.02 mm on declared running fits) -> `legs_cad_verify.json` |
| `legs_motion.py` | the study's motions re-solved on the CAD kinematics; loads at the CAD mass |
| `legs_motion_test.py` | motions frame by frame: slip, limits, speeds, clearances, mechanism forces -> `legs_motion_test.json` |
| `legs_cad_export.py`, `viewer/` | data and page for the rotatable 3D model |
| `legs_v3_study.py`, `legs_v3_frames.py`, `act_table.json` | the Nov 18 study the CAD reads (copied from `project-docs/claude/code/hippos/legs_v3/`) |
| `deps/` | body / chassis modules the motions read (`chassis_v3`, `chassis_v4`, `equo_v2`, `sketch_body`, `sk3d`, `centaur_study`) |
| `sling/` | front sling options study (2026-11-27) |

Run: `pip install manifold3d numpy scipy numba matplotlib`, then in this folder
`MPLBACKEND=Agg python3 hippos_legs_cad.py && python3 legs_cad_verify.py && python3 legs_motion_test.py`.

Baseline check (2026-11-27): four legs 17.47 kg; clearance sweep fore 1741 poses, hind 580, tightest 1.5 mm, none under
the rule. The octopus clearance in the motion test needs `deps/scene2_notail.npz` (render meshes), which is not here; that
check reads 99 (skipped).
