# Cephalo-1 octopus drone: current design state (rev 4, as of 2026-11-08)

**Current = rev 4, the integrated design.** Full detail, sim results and BOM: `claude/cephalo1-rev4-integrated.md`. Code: `claude/code/cephalo4.py` (single source of truth for geometry, masses, thrust), `poses4.py`, `sim4.py`, `states4.py`, `viz4.py`. Video: `cephalo1_rev4_mission.mp4`.
Background docs: `servo-joint-study.md` (servo at every joint), `impact-compliant-joint.md` (clutches, adopted), `joint-concepts.md`, `arm-joint-architecture.md` / `base-box-design.md` (tendon designs, not buildable, kept for reference), `cephalo1-dynamics-results.md` (rev 3 dynamics).
Values marked (est.) are estimates.

## Current design (rev 4)
| Item | Value |
|---|---|
| Mantle base plate | 8 PETG sectors forming a ring: outer r 125 mm, inner r 45 mm, 6 mm thick (plate underside = body z 0) |
| Dome | 4 quarter shells, ellipsoid a = 125 mm, c = 150 mm, 3 mm wall (top ≈ z 156); crown vents Ø50 at r 60; arm portholes Ø62 |
| Arm root | wall face x = 104 mm, arm axis z = 39 mm; azimuths 22.5° + 45°·i |
| Central lift fan | 90 mm EDF (JP Hobby 12-blade, 1750 Kv, 6S): 450 g, 42.2 N max; split clamp under the plate |
| Arm | 8 servo-in-link single-pivot joints, p-y-p-y-p-y-p-y, axes at s = 50 + 56·k mm (50..442), spigot 478 mm. Each servo sits in its link, horn on the joint axis drives the next link through a detent clutch |
| Servos | j0 STS3250, j1-j7 STS3215 (12 V). No STS3032 in the arm |
| Clutches (adopted, clutch set v1) | j0 root cam/rocker 1.2 × stall; j1 rigid; j2-j7 STS3215 cam/rocker 0.9 × stall (at 60 N line); travel ±25° root / ±40° others; one Dyneema line + N20 worm winch per arm: soft 25 / firm 60 / tight 150 N |
| Joint limits | j0 −59/+55° (mantle), j1 ±30° (neighbours), j2-j7 ±100° (shells) |
| Tip | tip bay (6S 45 A ESC, XIAO node, IMU) 454-478 mm; no separate wrist (alternating p/y joints aim the claw axis; tip roll optional for grasping) |
| Pod | 2207 6S motor + 6 in claw-prop (3 finger-blades on root flap hinges, fold 0-60° by servo + slider + tendons), puller, 115 g (est.) |
| Masses (est.) | arm 918 g + winch 24 g; body 2.62 kg; **robot 10.15 kg** |
| Thrust | EDF 42.2 N + 8 × 6 in claw-prop 10.1 / 12.6 / 15.1 N (pess / mid / opt, unmeasured) → T/W 1.24 / 1.44 / 1.64 |
| Controller | Teensy 4.1 (3.3 V; servo bus buffer 74LVC125/126), 64 servos |
| Servo data | STS3215 12 V: 2.94 N·m stall, 0.98 rated, 55 g, 45.2 × 24.7 × 35 mm, shaft 11.4 mm from the output end (approx.). STS3250: 4.9 N·m stall, rated stall/3 (assumed), 74.5 g |

## Poses and modes (degrees, j0..j7)
- **Stand:** [−11.7, 0, 39.2, 0, 64.0, 0, −1.5, 0], plate 300 mm above the floor, feet at r 360 mm, claw fold 60° (tripod fingertips). Winch tight.
- **Walk:** ripple gait, one foot at a time (7 down), order 0, 4, 2, 6, 1, 5, 3, 7; 90 mm stride, 45 mm lift, 0.30 s swing in 0.36 s slots; fans and EDF off. Full-robot sim: worst servo 1.0 × stall momentarily (j6 in swings), rms 0.78 × rated.
- **Take-off (v5):** EDF to 42.2 N → leg pairs 2+6, 1+5, 3+7 swing up (1.8 s each: foot lift 70 mm, tuck, unfold; fans start within 25° of vertical at 6 N) → robot stands on legs 0 + 4 → collective unloads the feet (~10 N left) → lift off 40 mm → leg 4 swings up in the air (6 fans + EDF) → leg 0 swings up (7 fans + EDF). Winch firm. (The rev 3 one-leg hop tipped over in the full-robot sim.)
- **Flight ("tulip"):** [−6.9, 0, −25.8, 0, −57.0, 0, −0.3, 0], claw axes vertical, hubs at r 360 mm, 353 mm above the plate; arm-body clearance 106 mm; prop-disc gap 123 mm. Hover: EDF 80 % + 8.3 N per claw-prop. Winch firm.
- **Landing:** take-off reversed. Winch soft only during the airborne approach, firm from touch-down, tight once all feet are down.
- **Hits (0.2 kg, 6 m/s, in hover):** side hit gear load 0.99 of the assumed limit (j1, the rigid root yaw), radial 0.86; clutches slip 10-12° and are back under 1° within 0.15 s; body tilt ≤ 5°.

## Open issues
- **Claw-prop thrust (blocking):** the 6-fan steps of take-off and landing need 9.6 N per claw-prop. At the pessimistic 10.1 N the sim completes with all 6 fans saturated (no margin). Bench-test one 2207 + 6 in claw-prop; need ≥ 11 N.
- Link cradle CAD at 56 mm pitch (STS3215 + clutch rocker mounts); measure horn bolt circles.
- j1 root yaw at the gear limit in 6 m/s side hits: STS3250 at j1 is the simplest fix (+19.5 g/arm).
- Power system: 64 servos on 12 V, 8 ESCs + EDF on 6S, battery and flight time not modelled; body mass carried over from rev 3.
- Measure servo gear-strip torque, reflected rotor inertia, N20 worm mass.
- Puller-prop behaviour and spin-up time unmeasured. Gait ML (`gaitml/`) not continued.

## History (superseded, one line each)
1. Soft TPU pneumatic arms, 72 chambers: far too stiff to bend (40 kPa ≈ 1 N against the thousands of N·mm needed).
2. Tendon continuum sleeve on a nylon backbone: friction slop and 36-58 mm of sag.
3. Ducted tip fans with a ±25° tilt servo, Arduino Mega in the dome: thrust-to-weight only ~1.1-1.2.
4. Carbon spar arms, then a 16-vertebra cable spine (passive, rocks open under overload).
5. Soft TPU tube arms with tip fans as "tethered drones".
6. Ball-joint wrist with three swashplate servos (±30°) and a roll hub: blocked by the 90° drum.
7. Rev 2 alternating servo chain (461 mm), then rev 3 universal nodes (475 mm, 9 joints, U-brackets, 3 × STS3250 + 2 × STS3215 + 4 × STS3032).
8. In-line claw-prop (pod 250 → 81 g), flight v1 (60° splay), then flight v2 (fans straight up).
9. Tendon-driven rolling vertebrae with a per-arm base box (2026-11-08): not buildable for Forest; replaced by a servo at every joint.
10. Breakaway cone seats along the arm (2026-11-08): let the arm fold under pulls; replaced by per-joint detent clutches.
11. Rev 3 + clutches (2026-11-08): ~11.8 kg, T/W 0.90-1.11, could not hover in the pessimistic case.
12. #5 geared double hinge (2026-11-08): recommended by the servo study but does not package STS3215s at its 42-67 mm joint pitch (needs ~53 mm single pivot, ~90 mm with the 2:1 stage needed for full torque); replaced by rev 4 servo-in-link joints at 56 mm.

## Conventions and environment
See `claude/environment-and-conventions.md`. Body frame: origin at the centre of the plate underside, +z up, arm i at azimuth 22.5° + 45°·i. Arm frame: x along the arm from the wall face (x = 104, z = 39); positive pitch = tip down. CAD: manifold3d → STL via trimesh, overlap solids ~1 mm before unions. Checks on every animation: IK residual, joint limits, floor/body/neighbour clearance, servo load against stall/rated.
