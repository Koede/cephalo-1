# Impact-compliant servo joints: winch-tensioned detent clutches — ADOPTED (2026-11-08)

Ask (Forest): joints that survive hits, keep range of motion, can loosen and tighten relative to each other, and give a little when the fan yanks, built from a string/elastic line and a small winch. Constraint from `servo-joint-study.md`: every joint keeps its own servo at the joint. **Decision (Forest, 2026-11-08): "clutch it is".**

## Design: one tension line per arm sets every joint's slip torque
- **Line:** 1.5 mm Dyneema from the tip coupler to the root, with a series spring (2 shock-cord loops or a printed TPU spring, ~0.5-4 kN/m). It is routed through hollow brass hinge pins (K&S tube) so bending a joint doesn't change its length.
- **Winch:** a 1220WG-N20VA-class self-locking worm gearmotor (gearbox 18 × 12 × 10 mm, 39 mm long, 3 mm D shaft, 2.4 kg·cm stall), a Ø6 mm spool and a 4:1 Dyneema block (3 × 623ZZ). Firm 60 N needs 0.45 kg·cm at the spool and tight 150 N needs 1.1 kg·cm. The worm holds tension with the power off. (This replaces the heavier JGY-370 idea; the motor mass of ~12 g is an estimate.)
- **At each clutched joint:** the servo horn carries a printed **V-cam disk**. A printed **rocker** on an M3 shoulder-bolt pivot holds a roller in the cam's V on its short arm and a 623ZZ idler on its long arm. The tension line wraps 90° under the idler, so line tension pushes the roller into the V. A support bearing carries the cam disk on the link so the roller force doesn't side-load the servo shaft.
- **Behaviour:** below the slip torque the joint is rigid. Above it the link swings on the cam up to its stop instead of back-driving the servo gears. When the load drops, the line pushes the roller back to the bottom of the V and the joint springs back.
- **Relative tightness:** M = 2T·sin(α/2)·(L_idler/L_roller)·k_cam. Each joint's lever ratio and cam rise set its share, and the winch scales them all together.

## Adopted configuration (from the sweeps below)
| Joint | Servo | Clutch | Slip torque at 60 N | Cam | Roller / support bearing | Rocker roller / idler arm |
|---|---|---|---|---|---|---|
| j0, j2 (root pitch) | STS3250 | yes, 1.2 × stall (slips only on impact spikes) | 5.9 N·m | 16 mm/rad, ±25° | 626ZZ (6×19×6) on M6 / 6802ZZ (est.) | 10 / 43 mm |
| j1 (root yaw) | STS3250 | rigid | — | — | — | — |
| j3-j8 | **all STS3215** (j5-j8 were STS3032) | yes, 0.9 × stall | 2.6 N·m | 10 mm/rad, ±40° | 624ZZ (4×13×5) on M4 / 6800ZZ (10×19×5) | 10 / 31 mm |

Printed set per arm (PETG, 4 walls + 40 % gyroid; all watertight; `cephalo1_clutch_set_v1.zip`, built by `clutch_parts.py export_adopted()`, sheet `clutch_parts_sheet.png`):
| Part | Qty | Size (mm) | Mass each (est.) |
|---|---|---|---|
| clutch_cam_STS3215 | 6 | 42 × 46 × 6 | 6.4 g |
| clutch_rocker_STS3215 | 6 | 57 × 18 × 8 | 4.9 g |
| clutch_cam_STS3250_root | 2 | 53 × 54 × 7 | 10.7 g |
| clutch_rocker_STS3250_root | 2 | 72 × 23 × 9 | 7.1 g |
| winch_cradle_N20worm | 1 | 44 × 16 × 17 | 2.9 g |
| winch_spool_6mm | 1 | 14 × 14 × 10 | 0.5 g |
| winch_block_4to1 | 1 | 36 × 12 × 6 | 1.3 g |
**The horn bolt circles in `clutch_parts.py` are placeholders.** Measure the STS3215 and STS3250 horns before printing.

## Winch modes (line tension)
| Mode | Line | Use | Side hit 6 m/s: worst gear load / limit | Recovery |
|---|---|---|---|---|
| Soft | 25 N | landing, grabbing, flying near obstacles | 0.5 | 2.4 s |
| Firm | 60 N | normal flight | ~1.0 | 0.4 s |
| Tight | 150 N | walking, carrying, pulling | ~2.3 (like rigid) | 0.4 s |
(Measured on the j3-j8 clutch set before the root clutches were added.)

## MuJoCo results (`sim_seats.py`, `tune_clutch.py`, video `cephalo1_clutch_hit_mujoco.mp4`)
One-arm tulip model, knuckle + roll wrist. The metric is gear-train load = servo torque + reflected rotor inertia × joint acceleration, against an **assumed** gear limit of 3 × stall. The reflected rotor inertias (0.003-0.02 kg·m²) are estimates. The STS gear ratio is about 1:345, so a sudden hit has to decelerate a heavy reflected rotor, and that strips gears more than steady stall torque does.

Side hits (0.2 kg object into the prop, tangential):
| Arm | 4 m/s | 6 m/s | 8 m/s | Pull 40 N |
|---|---|---|---|---|
| Rigid, j5 STS3215 | 5.8× | 8.6× | 13.3× | 0.35 |
| Rigid, j5 + j7 STS3215 | 1.4× | 2.1× | 3.2× | 0.35 |
| j5 + j7 STS3215 + clutches j3-j8 | 0.67 | 1.03 | 1.02 | 0.36 |
| Yaw clutches only (j3, j5, j7) | 0.59 | 0.98 | 0.97 | 0.65 |

Radial hits (pushing the fan toward the body, loading the pitch joints), which forced the adopted set:
| Arm | 4 m/s | 6 m/s | 8 m/s |
|---|---|---|---|
| Rigid, j5 + j7 STS3215 (j6, j8 STS3032) | 4.1× | 6.1× | — |
| Same + clutches j3-j8 (STS3032 clutch at 0.35-0.9 × stall) | 1.1× | 1.6× | 2.5× (j6, j8 clutches hit their stops) |
| All j3-j8 STS3215, rigid | — | 1.0× | 1.6× (side 8 m/s: 3.3×) |
| All j3-j8 STS3215 + clutches j3-j8 | — | 1.2× (root j0 now takes it) | 1.25× |
| **Adopted: + root pitch clutches at 1.2 × stall** | **0.45** | **0.55** | **0.67** |
Adopted set, side hits: 0.57 / 0.95 / 0.99 (4 / 6 / 8 m/s). Pulls of 20 and 40 N: 0.20 and 0.34, with essentially no slip. Recovery 0.2-0.6 s, nothing stays folded.

## Mass and flight (estimates) — the open problem
| Per arm, vs rev 3 (647 g arm) | Added |
|---|---|
| j6, j7, j8 + j5 → STS3215 (4 × +35 g) | +140 g |
| 6 STS3215 clutches (printed 11.3 g + 624 + 623 + 6800 + bolts) | ~+143 g |
| 2 root clutches (printed 17.8 g + 626 + 623 + 6802 + bolts) | ~+76 g |
| Winch (N20 worm ~12 g est. + printed 4.7 g + 3 × 623 + bolts) | ~+24 g |
| Line, spring, hollow pins | ~+13 g |
| **Total** | **~+395 g per arm → +3.2 kg robot (8.6 → ~11.8 kg)** |
With the current claw-props (EDF 42.2 + 8 × 7.8-10.8 N) T/W = **0.90-1.11: it cannot hover in the pessimistic case.**

Ways back (all estimates):
1. **Port the clutches to the servo geared double-hinge arm** (`servo-joint-study.md` #5: 619 g arm + pod, 8 joints). With its 2 STS3032 upgraded and clutches added, ~932 g vs 747 g, so the robot is ~10.5 kg. T/W is 1.02-1.25 on the current props.
2. **6 in props on 2207-class motors** (+30-40 % tip thrust, +15 g per pod): T/W 1.23-1.55 on top of #1.
3. Fewer clutches: dropping the j4/j6/j8 pitch clutches costs radial-hit protection (see above).
4. Larger EDF, PA12/CF-nylon cams with deeper pockets, or 3D-printed TPU springs in place of the support bearings: each worth tens of grams.
Recommended path: #1 + #2.

## What didn't work (kept for the record)
1. **Cone seats along the arm** (rocking + axial give at 5 places, same line): they slip only after the adjacent, weaker servo is already overpowered, and they let the arm fold under a 20-40 N pull.
2. **Root clutches at 0.9 × stall:** a 40 N pull slipped them. At 1.2 × stall they only slip on impact spikes.
3. **Telescoping wrist with its own 14 N preload:** extended 21-26 mm under a 20-40 N pull, then hit its stop (1.3-1.5×). A runaway fan is handled by the tip node cutting the ESC.
4. **Clutches on STS3032 joints:** they hit their stops in radial hits whatever the slip setting. Those joints must be STS3215.

## Open items
- Measure: STS3215/3250 gear-strip torque, reflected rotor inertia (spin-down test), horn bolt circles, N20 worm motor mass and real stall.
- After a slip the servo encoder still reads the motor side. Add a link-side AS5600 magnetic encoder per clutched joint (~2 g), or trust the spring-back.
- Firmware: winch modes (soft/firm/tight) tied to flight mode; current sensing on the winch for line tension; slip detection.
- Re-run walking and take-off loads on the heavier arm once the arm architecture (#5 + clutches) is fixed.
