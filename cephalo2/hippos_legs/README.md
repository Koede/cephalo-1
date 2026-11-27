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

## Rev D (2026-11-27): every part real

- **Motors:** RobStride RS02/03/04/06 envelopes, faces and bolt patterns from the maker's STEP files (`hardware.MOTORS`),
  each split into housing and output rotor with a 0.1 mm running gap.
- **Fasteners:** every joint is a declared `hardware.Joint` (screw size, length, clearance and counterbore holes, heat-set
  insert, tapped hole or nut); the build fails if a screw leaves under 0.8 d of thread.
- **Shafts:** 25/12 mm 4140 tube, three DIN 6885 A 8 x 7 keys, steel spacer sleeves; 6061 keyed arms, pulleys and cranks.
- **Yaw:** two 61810 back to back, DIN 472 65 circlip, 2 mm clamp washer on inserts.
- **Servos:** Feetech STS3215 as the real case (`vendor/so-arm100/`, Apache 2.0, from TheRobotStudio's SO-ARM100), held by
  4 M2 x 6 self-tappers through a printed plate pocketed round its bottom features; spools on the output spline with an
  M3 centre screw.
- **Gas spring studs:** ISO 7379 8 mm shoulder screws, M6. Upper into a heat-set insert in a 20 mm boss on plate B's
  outer face; middle into the outer arm (tapped). Steel spacer sleeves locate the 8 mm eye at y 41..49.
- **Pins:** DIN 6799 E-clips on the elbow, hock, carpus (-y end) and fetlock pins; elbow/hock clips sit 1.6 mm off the
  arm faces, which turn on the pin. E-clip sizes are nominal; check the DIN table before ordering.
- **Hoof:** TPU shoe held to the core by 3 M3 x 8 into heat-set inserts.

Open: the gas spring is modelled 114 / 84 mm eye to eye, 30 stroke. No stock 6/15 does that (eye to eye is about
2 x stroke + 63; Lesjoefors F9002 + L-E11 eyes is 103.5 / 83.5 with 20 stroke), and a 20 or 25 mm stroke would put the
middle stud inside the 25 mm shaft. Choices: a custom-stroke strut, a lost-motion slot, or no spring.
