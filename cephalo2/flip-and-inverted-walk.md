# Mid-air flip and walking upside down (2026-11-11)

Code: `claude/code/flip4.py` (the run), `flip_pathopt.py` (roll path, offline), `flip4_path_opt.npy` (the path), `flip_checks.py` (clearances), `flip_video.py`. Video: `cephalo1_flip_walk_inverted.mp4`. Builds on body4 (`body-controller.md`) and the XL claw-props (`quad-mode-and-power.md`). Values marked (est.) are estimates.

## What it does
Cephalo-1 hovers upright, turns itself over in the air, lands dome-down on its feet and walks upside down.

| t (s) | Phase |
|---|---|
| 0-1.0 | Hover upright, tulip pose, 8 claw-props + EDF |
| 1.0-2.4 | EDF spools down; the claw-props carry everything (the EDF would push sideways and then down once the body tilts) |
| 2.4-6.4 | **Gimballed roll**, 180° in 4 s about the horizontal axis through arms 0 and 4 |
| 7.0-11.0 | Inverted hover, descent to 0.35 m |
| 11.0-18.0 | Inverted landing: leg pairs swing down one after another |
| 18.0-19.4 | Settle into the inverted stand |
| 19.4-28.1 | Walk upside down: 18 cm forward + 20° turn (came out 18.2 cm, 1° off course, 20.3°) |

## The two inverted poses (a useful duality)
- **Inverted flight pose = the normal stand pose** (Q_STAND in the body frame). Its claw axes point away from the dome, i.e. up once the body is upside down.
- **Inverted stand ≈ the candle flight pose**: a "spider" stance with the knees up.
  - q = [43, 0, −71, 0, −80, 0, 18, 0]°.
  - Plate 0.32 m above the floor, feet at r 0.30 m, dome crown 16 cm clear.
  - Static load 0.25 × stall on 8 feet.
- Lower stances put j0 on its +55° limit. At 0.28 m the IK lost a degree of freedom and the legs fought each other.

## Gimballed roll
The body turns over while the arms keep every thrusting claw-prop axis vertical (tentacle gimbal).
- **Arms 0 and 4** lie on the roll axis. Their bend passes from the pitch joints to the yaw joints and back, which amounts to a 180° twist of the arm.
- **Arms 1, 2 and 3** go over the top; their pitch joints counter-rotate.
- **Arms 5, 6 and 7** swing under the body:
  - their claw-props cut out (by thrust weighting) while the axes are more than 40° off vertical;
  - from 62 % of the roll they turn their axes back up and rejoin.

**Path, solved offline (`flip_pathopt.py`):** least squares at 160 steps along the roll with the body at the reference attitude, then Gaussian smoothing. Terms:
- claw axes vertical;
- joint loads above 0.6 × stall penalised, at the thrust each active claw-prop must give;
- thrust centroid over the CoM;
- body ≥ 15 mm, arm-arm ≥ 30 mm and disc ≥ 30 mm clearances;
- continuity.

**Findings that shaped it:**
- **Moment balance is the hard constraint, not reach.**
  - A first purely kinematic path put arms 1-3 on one side with 5-7 stopped. No thrust split could then hold the attitude, and the robot fell.
  - Five claw-props also cannot cancel the rotor torques by thrust differences (3 spin one way, 2 the other). Yaw is therefore held by leaning the claw axes tangentially; the allocator's yaw row is weighted low.
- **Root yaw j1 overloads mid-roll.**
  - At 90° of roll the root yaw acts as a pitch joint carrying the thrust: 1.2 × stall on an STS3215.
  - With an **STS3250 at j1** (same case, +19.5 g per arm, already proposed against 6 m/s side hits) and the load-aware path, the roll peaks at 0.94 × stall.
- **Clutches:**
  - At firm winch (60 N) the j2-j7 clutches slipped 40° under 28 N of thrust.
  - The roll now runs on the tight winch (150 N): slip ≤ 0.1°.
- **Thrust is the limit:**
  - With 2 packs (11.7 kg) the claw-props sat at their 28 N ceiling for ~1.5 s, the robot dropped 15 cm and servos touched 1.0 × stall.
  - With **1 pack** (10.9 kg): 1.1 s at the ceiling around 140-165° of roll, an altitude dip of 7 cm, attitude error ≤ 7.7°.
- **Allocation:** bounded least squares (scipy `lsq_linear`). body4's attitude-first mixer returned zero thrust when the moment demand was infeasible.

## Inverted landing
- Opposite pairs (0+4, 2+6, 1+5, 3+7) swing from the flight pose to the spider stance 1.45 s apart, so each pair is down before the next starts. The last pair swings with 6 feet planted.
- The final 40 % of each swing is IK-guided onto its floor spot.
- Planted legs hold the **planned** body pose, not the measured one. Holding the measured pose let the body drift, the load concentrated and it collapsed.
- Each planted foot takes 16 N from the claw-props; the props spin down once all 8 are down.
- Result: 0.77 × stall, slip 0.2°, no dome contact.
- **Rejected:** landing on the dome first.
  - Resting on the dome, the feet cannot reach the floor (j0 limit).
  - With the arms raised, the CoM sits above the crown's centre of curvature (0.104 m), so the body would tip.

## Walking upside down
- body4's ripple gait (`Walk`), driving the inverted body frame Rz(yaw)·Rx(180°).
- **Touchdown latch:** a foot that lands is held where it actually touched. Without it, the IK dragged it toward the planned spot, toe friction squeezed opposite legs and j4 sat at 1.0 × stall for seconds. The knees-up stance gives horizontal foot forces long levers.
  - Frames above 0.9 × stall: 183 of 368 → 2 of 260.
- **Load governor off for this gait.** Its slow-downs (rate 0.45-0.8) made the stiff, position-controlled legs rock the body: 48 frames above 0.9 with it vs 2 without. A load-sharing reflex (raise overloaded feet) was tried and made it worse.
- Result: peak 0.99 × stall for 2 frames, 95th percentile 0.68, tilt ≤ 2.3°, slip ≤ 0.1°.

## Results (1 pack, 100 % charge)

| Phase | Worst servo | 95th pct | Clutch slip | Other |
|---|---|---|---|---|
| Roll | 0.94 | 0.92 | 0.1° | attitude error ≤ 7.7°, altitude dip 7 cm, claw-props at 28 N for 1.1 s, 165 A peak, 21.8 V min |
| Inverted hover | 0.31 | 0.28 | 0.1° | 8 × ~13 N |
| Landing | 0.77 | 0.76 | 0.2° | no dome contact |
| Walk | 0.99 (2 frames) | 0.68 | 0.1° | tilt ≤ 2.3° |

Battery: 100 → 90 % over the 29 s run.

**Clearances (`flip_checks.py`):**
- arm-body ≥ 17 mm;
- arm-arm ≥ 21 mm;
- arm-floor (all but the toes) ≥ 82 mm;
- dome ≥ 160 mm above the floor while walking.
- Prop discs: ≥ 77 mm apart, except near 150° of roll. There the discs of 0/1 and 3/4 overlap by up to 17 mm in plan view, but sit 79-97 mm apart vertically (no contact; aerodynamic interference not modelled).

## Hardware implications
- **STS3250 at j1 on all arms** (now justified twice: the flip and side hits).
- **XL claw-props are required.** With 6 in claw-props, inverted flight without the EDF is impossible.
- **Dome crown pad:** not needed by this sequence (the dome never touches). Keep the TPU bumper on the EDF can for the upright lie-down.

## Open
- Thrust margin mid-roll is thin (1.1 s at the ceiling); it gets worse with two packs, payload or low charge. Not tested below full charge.
- Flip back (inverted → upright), and take-off from the inverted stand.
- Walking upside down for longer distances; terrain; governor tuning for stiff stances.
- Prop-disc plan-view overlap near 150°: add a disc-gap term weighted for |dz| < 0.12 m to the path optimiser.
- Real robot: foot touchdown latch needs foot contact sensing (servo load readback / tip IMU).
