# Cephalo-1 arm: servo-in-joint study (2026-11-08)

**Constraint from Forest:** the tendon-driven vertebrae (and the base-box design in `base-box-design.md`) are not buildable for him. Every joint must be driven by its own servo at the joint, and that is the main constraint on joint type.

> **Update (rev 4 integration, 2026-11-08):** the recommended #5 geared double hinge does **not** package. This study's meshes let the servo bodies overlap the knuckles. An STS3215 (45.2 mm long, shaft ~11.4 mm from the output end) needs ≥ ~53 mm per joint as a single pivot. A double hinge driven at one pivot only gets half the servo torque, and the 2:1 stage needed for full torque takes ~90 mm per joint, against the 42-67 mm used here. Its m 1 Ø22 printed gears would also see ~240 N tooth loads. The masses and loads below are therefore optimistic for #3-#8. Rev 4 uses 8 servo-in-link single-pivot joints at 56 mm pitch (`cephalo1-rev4-integrated.md`).

Video: `cephalo1_servo_joint_grid.mp4`, a 3×3 grid of the same 18 s writhing command as `cephalo1_tentacle_joint_grid.mp4`. Code: `claude/code/servo_arms.py` (variants + sizing), `claude/code/writhe_grid.py` (run with `GRID_MODULE=servo_arms`).

## Sizing rule
For each joint, the smallest servo in the variant's family is chosen such that rated torque ≥ the continuous load and stall/1.25 ≥ the peak load. The choice is iterated with the resulting masses.
- **Continuous loads:** walk with EDF assist (6.2 N per foot), hover (5.5 N with ±10° vectoring), arm held out at 1 g.
- **Peak loads:** walk with fans off (12.3 N per foot), full thrust (10.8 N with ±20° vectoring), arm swung at 2 g.
- **Mass model:** servo + housing at each joint, link structure per metre, pod 100 g. These are estimates.
- **Servo data:** STS3032, STS3215 (12 V) and STS3250 from their datasheets; STS3250 rated torque assumed to be 1/3 of stall. The 4:1 printed planetary and the BLDC + cycloid modules are estimates.

## Results
| # | Joint type | DOF | Servos (auto-sized) | Arm + pod | Root Ø | Worst servo load | Shape error avg / max | Self-hit frames |
|---|---|---|---|---|---|---|---|---|
| 1 | Rev 3 U-bracket chain (current) | 9 | 3×3250, 2×3215, 4×3032 (fixed) | 704 g | 65 | **1.98×** (j5-j7 STS3032) | 11.5 / 36 | 31 |
| 2 | Servo shells (same servos) | 9 | fixed as #1 | 682 g | 48 | **1.98×** | 12.0 / 31 | 0 |
| 3 | Inline servo + bevel knuckle | 7 | 1×3250, 5×3215, 1×3032 | **601 g** | 48 | 0.77× | 12.3 / 26 | 0 |
| 4 | Cardan nodes (crossed pair in sphere) | 12 | 2×3250, 8×3215, 2×3032 | 945 g | 68 | 1.13× | 6.8 / 14 | 8 |
| 5 | **Servo geared double hinge** | 8 | 1×3250, 5×3215, 2×3032 | 619 g | 54 | 0.82× | 9.9 / 23 | **0** |
| 6 | Servo inside each rolling vertebra (geared cams) | 11 | 2×3250, 7×3215, 2×3032 | 831 g | 52 | 1.18× | 6.7 / 13 | 0 |
| 7 | Micro servo + 4:1 planetary, inline | 8 | 2×3215+4:1, 6×3032+4:1 | 620 g | 63 | 0.91× | 11.8 / 25 | 20 |
| 8 | Snake modules (servo body is the link) | 10 | 1×3250, 7×3215, 2×3032 | 714 g | 50 | 0.99× | **6.4 / 13** | 18 |
| 9 | BLDC + printed cycloid (FOC) | 6 | 1×BLDC-L, 5×BLDC-S | 674 g | 64 | 0.99× | 18.7 / 33 | 9 |

## Reading
- **The current rev 3 arm is overloaded** in the full-thrust ±20° vectoring case: its distal STS3032s (j5-j7) reach 1.3-2.0× their limit. Either that load case gets capped in firmware or those joints move up to STS3215.
- **Best balance (as modelled here): #5 servo geared double hinge.** Each knuckle is a pair of meshing gears with the servo in the link, so one servo gives up to ±120° with a rolling-contact-style moving axis. It is 85 g lighter than rev 3, tracks the shape better (9.9 vs 11.5 mm), has no self-collisions and no overloaded servo. *(Superseded: see the update at the top.)*
- **Most fluid / finest control: #8 snake modules** (10 DOF, 6.4 mm, 714 g, about the weight of rev 3), but the root servo sits at the limit (0.99×) and it self-collides in tight coils.
- **#6 servo-in-vertebra** is the sleekest-looking option with fine control, but it is 831 g and the root STS3250 is at 1.18×. It needs a stronger root (geared STS3250 or BLDC) to work.
- **#3 inline bevel** is the lightest (601 g) but has only 7 DOF.
- **#4 cardan nodes** are too heavy (945 g).

## Next steps (candidates)
- Done in rev 4: servo-in-link single-pivot joints at 56 mm pitch with the adopted clutches (`cephalo1-rev4-integrated.md`).
- Still open: printable STS3215 link cradles at 56 mm pitch; a true double hinge only if the extra range (±120° vs ±100°) proves necessary.
