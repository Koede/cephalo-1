# Cephalo-1 rev 4: the integrated design (2026-11-08)

Everything adopted so far, in one robot: servo-at-every-joint arms, the winch-tensioned detent clutches (clutch set v1), 6 in claw-props on 2207-class motors, and the rev 3 body. All of it is checked together in one full-robot MuJoCo model that stands, walks, takes off, hovers, takes two hits, lands and stands again.
Code: `claude/code/cephalo4.py` (single source of truth), `poses4.py`, `sim4.py`, `states4.py`, `viz4.py`. Videos: `cephalo1_rev4_mission.mp4` (re-rendered 2026-11-10 with take-off v7), `cephalo1_takeoff_v6_vs_v7.mp4`, `cephalo1_takeoff_v5_vs_v6.mp4`.

## What changed while integrating, and why
1. **The #5 geared double hinge does not package with STS3215s.** The servo study placed 8 joints 42-67 mm apart, but its meshes let the servo bodies overlap the knuckles. An STS3215 is 45.2 mm long with its shaft 11.4 mm from the output end, so a servo-in-link joint needs at least ~53 mm per joint. A double hinge driven from one pivot only delivers half the servo torque (joint angle = 2 × servo angle). Getting full torque needs a 2:1 stage, and with that it needs ~90 mm per joint. Its printed m 1 Ø22 gears would also see ~240 N tooth loads at the clutch slip torque, which is too much for PETG.
2. **Rev 4 arm = 8 servo-in-link single-pivot joints at 56 mm pitch.** The horn sits on the joint axis and drives the next link through the adopted radial cam/rocker clutch, so the printable clutch set v1 is used as designed. Shells limit j2-j7 to ±100°. The arm is 478 mm from the wall face to the spigot (rev 3 was 458 + pod).
3. **Driving one pivot of a double hinge directly (half torque) was tried on j4-j7.** j4 overloaded in the walk (1.0× stall, rms 1.23× rated), so it was dropped.
4. **No STS3032 anywhere in the arm** (as adopted for the clutches).
5. **No separate wrist.** Alternating pitch/yaw joints already point the claw-prop axis in any direction, and the IK cancels the sideways tilt that rev 3 couldn't. A tip roll (+45 g) is optional, for grasp orientation only.
6. **Take-off v4 → v5 → v6.** Hopping on one leg (rev 3 plan) tipped the robot over in the full-robot sim. The working sequence is: three leg pairs up while the robot stands on legs 0 + 4 → collective unloads the feet → lift off → leg 4 swings up in the air (6 fans + EDF hold) → leg 0 swings up (7 fans + EDF). Landing is the same in reverse.
7. **Winch modes per phase:** tight (150 N) on the ground, firm (60 N) for take-off and flight, soft (25 N) only for the airborne approach, firm again at touch-down. Soft mode with weight on two legs let the clutches slip and the robot tipped over in the sim.

## Rev 4 specification
| Item | Value |
|---|---|
| Body | rev 3 unchanged: 8 PETG plate sectors r 125, dome a 125 / c 150, 90 mm EDF (42.2 N), Teensy 4.1 |
| Arm roots | wall face x = 104, axis z = 39, azimuth 22.5° + 45°·i |
| Joints | 8 per arm, p-y-p-y-p-y-p-y, axes at s = 50, 106, 162, 218, 274, 330, 386, 442 mm; spigot 478 |
| Servos | j0 STS3250; j1-j7 STS3215 (12 V) |
| Clutches | j0 root cam/rocker 1.2 × stall; j1 rigid; j2-j7 STS3215 cam/rocker 0.9 × stall (at 60 N line) |
| Limits | j0 −59/+55° (mantle), j1 ±30° (neighbours), j2-j7 ±100° (shells) |
| Winch | one N20 worm winch + 4:1 block per arm, 1.5 mm Dyneema line; soft 25 / firm 60 / tight 150 N |
| Pod | 2207 6S motor (1750-1950 KV class) + 6 in claw-prop (3 finger-blades, fold 0-60°), puller, 45 A ESC in the tip bay |
| Arm mass (est.) | 918 g moving + 24 g winch |
| Robot mass (est.) | **10.15 kg** (body 2.62 kg is derived from rev 3: 8.6 kg − 8 × 747 g) |
| Thrust | EDF 42.2 N + 8 × claw-prop 10.1 / 12.6 / 15.1 N (pess / mid / opt, unmeasured) → **T/W 1.24 / 1.44 / 1.64** |

Arm mass table (g, est.): STS3250 74.5, 7 × STS3215 385, joint shells/bosses/pins 96, clutches 180.8 (1 root + 6), link structure 33.5, tip bay 20, pod 115, line/spring/pins 13.

## Poses (optimised in `poses4.py`, joint order j0..j7, degrees)
- **Stand:** [−11.7, 0, 39.2, 0, 64.0, 0, −1.5, 0]. Plate 300 mm above the floor, feet at r 360 mm, claw folded 60°. Static loads at 7 feet down: continuous ≤ 0.44× rated (with 30 N EDF assist), peak ≤ 0.41 of stall/1.25 (fans off).
- **Flight (tulip):** [−6.9, 0, −25.8, 0, −57.0, 0, −0.3, 0]. Claw axes vertical, hubs at r 360 mm, 353 mm above the plate. Arm-body clearance 106 mm, gap between neighbouring prop discs 123 mm. Peak static load 0.41 (full thrust ±20° vectoring).
- **Hover:** EDF at 80 % (33.8 N) + 8 claw-props at 8.3 N mean = 66 % of mid / 82 % of pess max.

## Full-robot MuJoCo mission (`sim4.py`, 39.9 s, mid thrust 12.6 N)
Model: free body + 8 × 8 stall-limited position servos (kp = stall/2°), a clutch DOF at every clutched joint (implicit spring capped at the slip torque, scaled by line tension), reflected rotor inertia 0.02 / 0.012 kg·m² (est.), toes with friction 1.0, fans as forces along the pod axis with a 40 ms lag and the 25° fail-safe, COM-referenced thrust allocation (PD attitude, altitude and xy hold; gains 60/8 on two feet, 14/4.5 in the air).

| Phase | Worst servo (× stall) | rms (× rated) | Gear load (/ 3 × stall, assumed limit) | Clutch slip | Max tilt | Notes |
|---|---|---|---|---|---|---|
| Stand | 0.51 (j0) | 0.53 | 0.28 | 0.1° | 0.0° | |
| Walk (ripple, 2 cycles, 180 mm) | 1.00 (j6, momentary in a swing) | 0.78 | 0.56 | 0.2° | 0.4° | fans off |
| Take-off | 0.47 | 0.58 | 0.18 | 0.2° | 3.8° | fans peak 12.3 N during the 6-fan step |
| Hover | 0.12 | 0.28 | 0.04 | 0.0° | 1.8° | |
| Side hit, 0.2 kg at 6 m/s, arm 2 | 1.00 (j1) | 0.88 | **0.99 (j1, rigid root yaw)** | 9.7° | 3.3° | back under 1° in 0.06 s |
| Radial hit, 0.2 kg at 6 m/s, arm 6 | 1.00 (j0) | 1.30 | 0.86 (j1) | 12.1° | 4.9° | back under 1° in 0.14 s |
| Landing | 0.65 | 0.51 | 0.24 | 3.9° | 4.5° | |

Geometry checks over every recorded frame (`states4.py`): neighbouring arms ≥ 81 mm apart, arms ≥ 21 mm from the mantle/EDF, arm structure ≥ 88 mm above the floor.

**Thrust sensitivity (same mission):**
| Claw-prop max | Result |
|---|---|
| 12.6 N (mid) | completes, no fan saturated (peak 12.3 N) |
| 11.0 N | completes; up to 4 fans saturated during the 6-fan steps |
| 10.1 N (pess) | completes but all 6 fans saturate during the 6-fan steps (tilt 6.9°): **no control margin** |
Need: the 6-fan steps require (W − EDF)/6 = 9.6 N per claw-prop. **Measure ≥ 11 N static per claw-prop before building more than one arm.** For reference, good 2207 6S motors make 1.4-1.9 kgf on conventional 5 in props (miniquadtestbench / vendor data); the folding 3-blade claw-prop is unproven.

## Bill of materials (per arm; robot = × 8)
- Servos: 1 × Feetech STS3250, 7 × STS3215 (12 V).
- Clutches (clutch set v1, `clutch_parts.py export_adopted()`): 1 × root cam + rocker (626ZZ roller on M6, 623ZZ idler, 6802ZZ support); 6 × STS3215 cam + rocker (624ZZ on M4, 623ZZ, 6800ZZ). Note: v1 prints two root sets per arm; rev 4 needs one (j2 is now an STS3215).
- Winch: 1220WG-N20VA-class self-locking worm gearmotor, printed cradle + Ø6 spool + 4:1 block (3 × 623ZZ), 1.5 mm Dyneema, 2 shock-cord loops, K&S brass tube hinge pins.
- Pod: 2207 6S motor, 45 A ESC, XIAO tip node, IMU, fold servo + slider, 3 printed 6 in finger-blades.
- Printed per arm (to design next): 8 STS3215/3250 link cradles at 56 mm pitch with horn-side fork + idler boss, tip bay.
- Robot totals: 8 × STS3250, 56 × STS3215, 8 × 2207, 8 × ESC, 8 × N20 worm winch, 1 × 90 mm EDF.

## Open items (ranked)
1. **Claw-prop thrust**: bench-test one 2207 + 6 in claw-prop (puller) for static thrust, current and spin-up time. Need ≥ 11 N.
2. **Link cradle CAD** (STS3215 at 56 mm pitch, ±100° shells, clutch rocker mounts). Measure the horn bolt circles first (clutch set v1 placeholders).
3. **j1 root yaw** reaches the assumed gear limit (0.99) in the 6 m/s side hit. Options: STS3250 at j1 (+19.5 g/arm), or a root clutch at j1. A j1 clutch at 1.2 × stall was tried in the earlier double-hinge model and did not help, because it sits above the servo stall and never slips.
4. **Power**: 64 servos on 12 V, 8 × 45 A ESCs + EDF on 6S. Servo bus current, battery mass and flight time are not modelled. The 2.62 kg body is carried over from rev 3.
5. Walking: j6 hits stall momentarily in the 0.3 s swings (rms 0.78× rated). Slower swings or EDF assist (as in rev 3) would remove it.
6. Rotor inertia, gear-strip torque and clutch parameters are still estimates (spin-down test, strip test).

## Take-off v6: fluid (2026-11-10)
Ask (Forest): make the take-off more fluid. Code: `sim4.py` (`TAKEOFF` block, `_path_v6`, `allocate`), comparison `cmp_takeoff.py`, video `takeoff_video.py` → `cephalo1_takeoff_v5_vs_v6.mp4`.

What was jerky in v5: each leg's path was three smoothstep segments that stopped at every keyframe (2 stop-and-go points per leg); j4 was commanded at 334°/s and 1987°/s² (the servo firmware limit is 120°/s, 800°/s²); fans switched on/off at 25°; the pairs, the unload, the hop and legs 4/0 ran strictly one after another; ground → flight control switched in one step.

What v6 does:
1. **Leg path:** the 70 mm foot lift (minimum-jerk over the first 25 % of the swing) and the unfold to the flight pose (smooth trapezoid from 12 %) overlap, so each leg moves in one motion. Swing time 1.9 s is the shortest that stays within 120°/s and 800°/s². A cubic spline through the same keyframes overshot j0 by 21° and needed 3.1 s, so it was dropped.
2. **Wave:** pairs start 0.9 s apart and overlap. While legs overlap, the claw-props that are up carry whatever the remaining feet should not (~10 N per foot). At 0.6 s spacing the last two stance legs reached 0.83 × stall.
3. **Fans fade in** over 0.6 s and over 25° → 12° measured against the body axis. The world-frame 25° cut stays as the fail-safe. Measuring the fade against world vertical throttled the fans whenever the body tilted, which crashed the pessimistic case.
4. **Collective** rate-limited to 40 N/s during take-off; per-fan slew limits were tried and made attitude worse.
5. **Lift-off:** gains and collective blend from ground to flight over 0.5 s; the body rises 4 cm to clear the feet, then leg 4 unfolds and leg 0 follows 1.2 s later, with a slow 3 cm rise, then the main climb (overlapping minimum-jerk curves, so the body never stops rising). Starting leg 4 0.2 s after lift-off, or leg 0 0.6 s after leg 4, crashed the pessimistic case (a foot still on the floor acted as a pivot; legs 4 and 0 both down left 6 saturated fans and no authority).
6. **Attitude-first mixing:** F = c·u + α·Fm, where u is the pure collective and Fm the roll/pitch part. The collective gives way before the moments do. The old least-squares mixer zeroed the one unsaturated fan in the 6-fan step.

| Take-off, mid thrust | v5 | v6 |
|---|---|---|
| EDF start → last leg up | 10.7 s | 7.8 s |
| Stop-and-go points inside leg swings | 16 | 0 |
| Peak commanded joint speed / accel | 334°/s / 1987°/s² | 110°/s / 793°/s² |
| RMS commanded joint jerk | 1808°/s³ | 643°/s³ |
| Peak body vertical accel / RMS jerk | 0.74 m/s² / 2.4 m/s³ | 0.62 / 1.9 |
| Max body tilt / peak tilt rate | 3.8° / 24.1°/s | 2.0° / 7.5°/s |
| Peak claw-prop thrust rate | 74 N/s | 40 N/s |
| Worst servo load | 0.47 × stall | 0.57 × stall (stance leg 4 during the wave) |
| Pessimistic thrust (10.1 N) | completes, take-off tilt 6.9° | completes, take-off tilt 2.7°, landing tilt 7.9° |

The rest of the mission is unchanged within noise: hover tilt 1.4°, side hit gear load 0.99 (j1), radial 0.74, landing tilt 2.8°; clearances ≥ 82 mm between arms, ≥ 22 mm to the body, ≥ 88 mm above the floor.

## Take-off v7: organic + fast (2026-11-10, current)
Ask (Forest): "it can do it more organically and faster". Code: `sim4.py` (`_path_v7`, v7 timeline block), `cmp_takeoff.py`, `takeoff_video.py` → `cephalo1_takeoff_v6_vs_v7.mp4`.

1. **Tentacle unfurl:** the unfold travels from the root to the tip of each arm (pitch joints j0, j2, j4, j6 start 6 % of the swing apart). The yaw joints j3 and j5 add a sideways curl (peak 9° and 6°, a sin² bell that is back to 0 at the flight pose). Every arm curls in the same rotational sense, so the robot opens like a twisting flower bud. The foot lift (30 % of the swing) and the unfold (from 8 %) overlap, so the arm never pauses: speed never drops below 12 % of its peak mid-swing. With less overlap it fell to 4 %, a visible lull where j4 reverses.
2. **Faster swings:** an arm off the floor may move at 180°/s and 1200°/s², 2/3 of the STS3215's 0.222 s/60° (270°/s) no-load speed at 12 V (Feetech/Waveshare datasheets). Loaded joints keep the 120°/s and 800°/s² limits. Swing 1.55 s (v6: 1.9 s).
3. **Rotating wave:** pairs 1+5 → 2+6 → 3+7, 0.4 s apart (v6: 2+6, 1+5, 3+7, 0.9 s apart). The first feet lift once the EDF carries about 70 %. Starting them earlier put pair 1+5's j0 at stall.
4. **Thrust-aware finish:** if (W − EDF) / (6 × measured claw-prop max) ≤ 0.85, legs 4 and 0 unfurl together 0.3 s after lift-off. Otherwise the v6 sequence runs, with leg 4 first and leg 0 after leg 4's claw-prop is back, at the loaded-joint limits. Mid thrust (0.76) takes the fast branch; pessimistic thrust (0.95) the safe one. A fast finish with the pessimistic thrust crashed.
5. **Landing** keeps the v6 leg path and timing.

| Take-off, mid thrust | v5 | v6 | v7 |
|---|---|---|---|
| EDF start → last leg up | 10.7 s | 7.8 s | **4.4 s** |
| Stop-and-go points inside swings | 16 | 0 | 0 |
| Joints moving at once (mean) | 3.4 | 4.8 | 9.0 |
| Peak commanded joint speed / accel | 334°/s / 1987°/s² | 110 / 793 | 160 / 1134 |
| RMS commanded joint jerk | 1808°/s³ | 643 | 1237 (the same smoothness per unit time as v6: jerk scales with 1/T³, and (1.9/1.55)³ × 643 = 1180) |
| Peak body vertical accel | 0.74 m/s² | 0.62 | 0.91 |
| Max body tilt / peak tilt rate | 3.8° / 24°/s | 2.0° / 7.5°/s | **0.1° / 1.1°/s** |
| Worst servo load | 0.47 × stall | 0.57 | 0.85 (stance leg 0 at j0 while the wave passes; rms over the take-off 0.92 × rated) |
| Pessimistic thrust | completes, tilt 6.9° | completes, tilt 2.7° | completes (safe branch, 6.7 s), tilt 6.3°, root clutch slips 10° once |

Clearances over the whole run: ≥ 82 mm between arms, ≥ 22 mm to the body, ≥ 88 mm above the floor. Hover, hits and the mid-thrust landing are unchanged within noise. The pessimistic landing is still marginal: it completed in this run with 8.1° tilt and a 30° clutch slip at touchdown, and failed in an intermediate run. The fix belongs in the landing (the same treatment, or symmetric legs 0 + 4), not the take-off.
