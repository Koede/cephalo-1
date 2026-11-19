# XL claw-prop: the 7.5 in Snap-Digit, refactored as a hand (2026-11-12)

Ask (Forest): "The claws are incomplete … intelligently refactor the hands if you must first." Until now the 7.5 in (XL) claw-prop existed only as a thrust number in `power4.py` and as the 6 in claw scaled 1.25× with flat plates for fingers. This is the claw-prop actually sized at 7.5 in, on a real motor, with its loads checked and its use as a docking hand tested.
Status: **draft design, checked by scaling from the 6 in CAD and loads (`snapdigit.py`, `snap-digit-clawprop.md`) plus a MuJoCo grasp bench. No new CAD solids yet. Nothing printed.**
Code: `claude/code/xlclaw.py` (**single source of truth** for the XL claw: sim model, render meshes, load check → `xlclaw.json`), `xlclaw_bench.py` (two claws clasping → `xlclaw_bench.json`), `xlpod4.py` (the XL claw on the full robot, 2026-11-13).

## What changed against the 6 in claw
| Item | 6 in Snap-Digit | XL claw-prop |
|---|---|---|
| Blades, hub, palm cap, skirt | as `snapdigit.py` | **every dimension × 1.25**, tip radius 95.25 mm |
| Motor | 2207, 34 g | **T-Motor F90 2806.5 1300KV**: Ø33.4 × 34.7 mm, 46.6 g, 19 × 19 mm M3 mount, 4 mm shaft |
| Knuckle tape | 1 ply of 1/2 in tape-measure blade, 6 mm waist | **2 plies, 7.5 mm waist** |
| Root pin | M2 dowel, 3 × 2 brass bushing | **M3 × 12 hardened dowel (ISO 8734), 4 × 3 × 10 mm brass bushing** |
| Servo | STS3032, 8 mm spool, 41 N at the slider | STS3032, **6 mm spool, 55 N** (25 mm travel = 239°; the STS3032 turns 360°) |
| Pod mass | 105 g | **156 g** (est.; `power4.py` budgeted 190 g until 2026-11-13, now 156 g) |

## Loads at proof speed (`xlclaw.py`)
Speed: 22,430 rpm for 26.5 N with C_T 0.12 (same basis as the 6 in); proof 1.15× = 25,800 rpm, tip 257 m/s.
Geometric similarity: centrifugal forces scale 2.10× against the 6 in claw.
| Part | Load | Result |
|---|---|---|
| Knuckle tape (2 plies, 7.5 × 0.12 mm) | 557 N | 310 MPa, **SF 5.2** on 1600 MPa (6 in: 4.4). One ply would be SF 2.6 |
| Root pin (M3 dowel, double shear) | 1740 N | 123 MPa |
| Lug bearing on the bushing | 1740 N | 43 MPa (6 in: 34 MPa); PA-CF |
| Knuckle and root seats | — | ratios 2.3 and 3.0, unchanged: seat moment and thrust moment scale alike |
| Snap moment of the tape | 41.5 N·mm peak, 27.8 steady (est.) | the 2-ply tape needs the 55 N slider; at 41 N the knuckles stay open |

## As a docking hand (`xlclaw_bench.py`)
Two claws face to face, props stopped, both close. One claw fixed, the other on an arm-like mount (800 N/m, 6 N limit).
| Rotor index (60° = fingers between fingers) | Rotors held | Rotors free (motor off) |
|---|---|---|
| 60° | clasps | clasps, turn 4-5° |
| 50° | clasps | clasps |
| 40° | clasps | clasps, self-index to 54° |
| 30° | clasps loosely: lets go at 5 N | clasps, rotors turn 10° each |
| 20° | — | clasps, rotors turn 17° each (released hands stayed tangled once) |
| 0° (finger on finger) | — | clasps, rotors turn 26-30° |

- **The claws index themselves if the motors are off while the fingers close.** Any starting angle clasps; with the rotors held, ±20° is the limit.
- Clasped (any of the above, rotors free): 10 N sideways moves it 0.4-1.0 mm; 0.3 N·m twist 0.6°; **lets go at 20 N pulling apart** (the scaled 6 in claw: 15 N).
- What the clasp rests on: each claw's distal fingers curl round the partner's motor bell and skirt, roots at 61°, knuckles at 80°.
- Approach with the fingers open as a tulip (roots 40°). Closing from flat fingers fails when they start finger on finger.

## Render model (2026-11-13, `xlclaw_cad.py`)
Videos now draw the XL claw from the real Snap-Digit CAD (`snapdigit.py` parts × 1.25: proximal with root lug, tongue, key, tape recess and fairlead bridge; distal with seat pocket and anchor bridge; slotted hub; palm cap with stop towers; skirt; bearing ring with push rods; the steel tape strip including its fold) on the F90, in the same body frames as the MuJoCo claw. The MuJoCo contact geometry is still `xlclaw.py`'s pitched boxes.

## On the full robot (2026-11-13, `xlpod4.py`)
The rev 4 full-robot sims (`sim4.py` / `body4.py` and everything built on them) had the XL claw only as thrust, a 190 g pod and a 95 mm stick for a folded finger. `xlpod4.install()` (called right after importing `cephalo4`, before the sim and render modules) switches a script to the claw as designed here:
- **Pod mass 156 g:** `power4.PROPS["xl"]["dM_pod"]` is now 0.041 kg (was 0.075). The robot is 0.27 kg lighter than with the budget pods.
- **Fingertip as a foot:** root pin 16.25 mm off the axis, finger 79 mm from the pin, knuckle on its seat. At the 60° stand fold the tip is 56 mm out and 70 mm beyond the hub plane (the stick model: 48 / 82 mm). `cephalo4.toe()` gives it to `foot_drop()` and to `sim4`'s toe contact spheres.
- **Drawn from the CAD:** `viz4.arm_mesh` calls `xlpod4.draw` (motor body, hub and palm cap, skirt, bearing ring and push rods, three two-phalanx fingers, the tape with its fold). About 15,000 triangles per pod.
- **Fold used so far:** root fold only (2° precone in flight, 60° as feet); the knuckle curl is not driven in the full-robot sims yet.
- **Restore:** `xlpod_patch.py` applies the four small hooks to `cephalo4.py`, `sim4.py`, `viz4.py`, `power4.py` (and the two fall scripts) on a restored workspace; it is idempotent. Needs `xlclaw.py`, `xlclaw_cad.py`, `snapdigit.py`, `clawblade.py`, `mesh3d.py`.
- **Used by:** the fall bench (`fall4.py`, `falls-and-recovery.md`: all 19 falls re-run, video `cephalo1_pivot_takeoff.mp4`). Not yet: `flip4.py`, `quad4.py`, `tripod4.py`, `flex*`, `body4.py` sessions, the docking sims' robot bodies.
- **Open:** the hub plane should sit about 43 mm off the spigot (F90 34.7 mm + mount) instead of the 36 mm of the 2207 pod (`cephalo4.BLADE_OFF`); standing on three straight fingertips at a 60° root fold has not been load-checked for the XL finger (tendon holding force against the weight).

## Not done / open
1. CAD solids for the XL parts (`snapdigit.py` scaled, the F90 mount, the 2-ply tape recess, the larger bushing) and a fit/sweep check like `checks.py`.
2. Thrust of the XL claw-prop itself (the 26.5 N is the T-Motor 7.5 in table −10 % for the folding fingers, est.).
3. Tape fold moment and life with two plies; spool and tendon friction at 55 N.
4. A catch feature on the hub for more than 20 N pull-apart (not needed for docking: the reel-in uses < 10 N).

Source for the motor: [T-Motor F90 2806.5 (HobbyRC listing)](https://www.hobbyrc.co.uk/t-motor-f90-1300kv-motor).
