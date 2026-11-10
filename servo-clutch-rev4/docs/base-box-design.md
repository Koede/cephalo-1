# Cephalo-1 base control box: per-vertebra tendon drive (2026-11-08)

Context: Forest chose fine control of every vertebra, with one base box per tentacle driving the vertebra series (see `claude/arm-joint-architecture.md`). This doc sizes and packs that box. Code: `claude/code/loads.py` (load cases), `claude/code/basebox.py` (packer), `claude/code/verify_box.py` (checks). Renders: `box_hero.png`, `box_top.png`.

## Load model (static, kinematic poses on the rolling arm, moments about each rolling contact)
- Arm: 18 rolling vertebrae, 17 tendon joints + 2-DOF root knuckle. Masses: PA12 vertebrae, TPU skin 60 g, tendons 15 g, pod 100 g (estimates).
- Continuous cases: walking with EDF assist (6.2 N per foot), hover (5.5 N per claw-prop, ±10° vectoring), arm held out at 1 g.
- Peak cases: walking with fans off (12.3 N per foot), full thrust (10.8 N, 20° vectoring), arm swung at 2 g.
- Tendon tension = joint moment / moment arm (0.8 r at the joint) × capstan friction e^(0.1·wrap) × 1.15 preload.

| Sizing rule | Servos per arm (tendon joints) | Servo mass per arm | Result |
|---|---|---|---|
| Peak loads against rated torque, original Ø36 taper | 17 × STS3215 | 935 g | Far too heavy |
| Continuous against rated, peak against stall | 13 × STS3215 + 4 × STS3032 | 795 g | Too heavy |
| Continuous against rated, bursts ≤ 1.8× continuous against stall (walk with EDF only, firmware thrust cap) | 6 × STS3215 + 11 × STS3032 | 550 g | Does not fit the sector |
| Same rule + bias springs on the pitch spools | unchanged (arm-out case then dominates) | 550 g | No gain |
| **Same rule, thicker taper Ø50 → Ø22 (r × 1.4), 30 % infill, 3 mm spools (0.8 mm Dyneema)** | **1 × STS3215 (j2) + 16 × STS3032** | **375 g** | **Chosen** |

Root knuckle: pitch 1.10 / 2.72 N·m and yaw 0.41 / 1.57 N·m (continuous / peak), so 2 × STS3250 (one per axis, inside the mantle at the porthole).

Servo data (datasheets): STS3032: 23.2 × 12.1 × 28.5 mm, 20 g, 4.5 kg·cm peak / 1.5 kg·cm rated at 6 V. STS3215 (12 V version): 45.2 × 24.7 × 35 mm, 55 g, 30 kg·cm stall / 10 kg·cm rated. STS3250: 45.2 × 24.7 × 35 mm, 74.5 g, 50 kg·cm at 12 V; its rated torque is not published, so 1/3 of stall is assumed.

Joint speed with 3 mm spools: about 100°/s at the root vertebra, faster distally (firmware target was 120°/s).

## Packing result (greedy packer + independent check)
Per-arm envelope: a 45° sector of the mantle, r ≥ 62 mm (EDF lip and airflow funnel), dome tier above the plate, and a tray under the plate (z −46 to −3 mm). A keep-out block at the porthole holds the tendon guides and knuckle shafts.

| Mantle radius | Allowed volume per sector | Parts placed (of 20) |
|---|---|---|
| 125 mm (current) | 425 cm³ | 6 |
| 140 mm | 614 cm³ | 14 (taller dome c = 210: also 14) |
| 150 mm | 756 cm³ | 15 |
| **160 mm** | **910 cm³** | **20, all placed** |

Check at R 160: everything inside the envelope, no part straddles the plate, minimum gap 1.5 mm, no overlap with the neighbouring sector. 11 of 19 straight spool-to-porthole runs are blocked by another servo and need a guide pulley. Tendon runs are 30-99 mm (mean 64). The arm wall face moves from x 104 to x 139 mm.

## Mass and flight (estimates)
| Per arm | Now (rev 3) | Base-box design |
|---|---|---|
| Servos | 413 g | 524 g (2 × STS3250, 1 × STS3215, 16 × STS3032) |
| Arm structure | ~234 g (brackets) | 249 g vertebrae + skin + tendons, ~60 g box frame/spools/pulleys, 12 g PCB, ~30 g knuckle |
| Pod | 100 g | 100 g |
| **Total** | **~747 g** | **~975 g (+228 g)** |

Robot: +1.8 kg from the arms and about +0.24 kg from the larger mantle, so ~10.7 kg (was 8.6). T/W = 1.00 (pessimistic claw-prop thrust) to 1.23 (optimistic). Hover then needs 7.85 N per claw-prop, 73-100 % of its estimated maximum. **In the pessimistic case it cannot hover.**

## Ways to win the mass back (not yet modelled)
1. Bigger claw-props / motors (6 in, 2207-class): about +30-40 % tip thrust (estimate).
2. 14 vertebrae instead of 18: 4 fewer STS3032 and lighter vertebrae (~−110 g per arm).
3. Per-vertebra drive only on the distal half; the proximal half in 2 tendon sections.
4. Custom drive boards: bare coreless micro gearmotors + one shared encoder/driver PCB instead of STS3032 cases (~−150 g per arm, custom electronics).
5. Larger central EDF.
6. SLS PA12 lattice vertebrae at ~20 % fill.

## Open items
- Guide-pulley routing for the 11 blocked tendon runs, and the slots through the plate for the under-tray servos.
- Two buses: 6 V for the STS3032s and 12 V for the STS3215/3250s. 19 servos per arm, 152 per robot.
- Software compensation for tendon path-length coupling across proximal joints.
- Check tendon friction and wear at 30-100 N on PTFE liners.
- Update `cephalo1-design-state.md` once the mantle radius change is accepted.
