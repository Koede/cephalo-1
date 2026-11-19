# Quad mode and the power system (2026-11-11)

Code: `claude/code/power4.py` (battery + thrust/power model), `body4.py` (CEPH_POWER / CEPH_PACKS / CEPH_SOC / CEPH_PROP), `quad_sizing.py`, `quad_pose.py`, `quad4.py` (quad-mode bench + CMA-ES training), `quad_video.py`. Video: `cephalo1_quad_mode_trained.mp4` (before/after training, side by side). Values marked (est.) are estimates.

## 1. Gravity and battery sag
- Gravity (−9.81 m/s²) has been in every MuJoCo model since sim4 (`sim4.build()`); it was verified, not added.
- New: a power model (`power4.py`). 6S 5000 mAh packs, 19 mΩ per pack (est.), LiPo rest-voltage curve, EDF fit to the JP Hobby datasheet (T_max = 42.4 N·(V/25.2)^1.51, P = 11.2 W·T^1.5), claw-prop power by momentum theory, a 12 V servo bus. Thrust ceilings now follow the loaded pack voltage every 10 ms.
- **Finding: with the current 6 in claw-props, take-off is not reliable.**
  - 1 fresh pack: T/W 1.07 on 8 claw-props, **0.86 on 6**. The take-off/landing moments with 6 claw-props up cannot hold the robot.
  - The robot cannot hover below about 60 % charge.
  - More throttle on the EDF, 2 packs, or a 12S EDF do not fix it (the 2nd pack's 0.81 kg eats its own voltage gain: T/W 1.16 / 0.94).
  - body4's TakeOff now runs a self-check (`load_fraction`: thrust needed at the voltage the pack will sag to) and refuses to take off above 92 % on 8 or 100 % on 6 claw-props.

## 2. Claw-prop overhaul: "XL" Snap-Digit (proposed)
- 7.5 in, 3-blade Snap-Digit (same finger/knuckle design scaled 1.25× in length) on a 28xx-class motor instead of the 2207.
- Basis: T-Motor 7.5-inch-class test table at 6S: 2347 g / 481 W (80 %), 2994 g / 692 W (100 %), 0.51 N·m at full thrust. Folding finger blades taken to lose 10 % (est.): **T_max 26.5 N at 21.3 V**, scaled (V/21.3)^1.5, capped 28 N; P = 5.1 W·T^1.5; reaction torque 0.0173 m × T.
- Pod 115 → 190 g (est.), robot +0.6 kg. Disc radius 95 mm.
- Result (`quad_sizing.py`): 8-prop T/W 2.31, 6-prop 1.81 with 2 packs at full charge (2.10 / 1.64 at 40 %). Take-off and landing margins stop being a problem for flight itself.
- To do: redesign the Snap-Digit at 7.5 in (blade root pin and knuckle loads scale with tip speed; re-run `snapdigit.py`), 28xx motor mount in the pod, 45 A ESC is still sufficient (~40 A peak per motor, est.), spin-up time.
- **Open: legged take-off/landing with XL pods in body4** reaches 1.00 × stall and 17° clutch slip on landing (heavier tips on the arms that are still folding). Not resolved; needs slower pair waves or the landing pose re-tuned for the heavier pod.

## 3. Quad mode: 4 arms fly, 4 arms work
- **Flyers 0, 2, 4, 6** (every other arm, a square) + EDF at 100 %. **Workers 1 + 3:** shovel (0.40 kg, 0.80 m) digging in a sand box (dig, drag out, lift a 0.20 kg scoop, swing aside, tip; 4.2 s cycle). **Workers 5 + 7:** pan (0.25 kg) held by its handle + dish brush (0.05 kg) scrubbing 3.5 cm circles at 1.2 Hz with 2.5 N press. Both chores at the same time, from a 0.80 m hover.
- Requires the XL claw-props + 2 packs: quad mode with tools puts the 4 claw-props at 76 % of their ceiling (full charge), 87 % at 60 %, 91 % at 40 %. With the 6 in claw-props it would need 195-390 % (impossible).
- **Candle flight pose** for the flyers (`quad_pose.py`): q = [5.6, 0, −63.4, 0, −45.1, 0, 12.9, 0]°, hub r 0.30 m, 0.36 m above the plate, thrust axis vertical. Worst joint 0.41 × stall at hover thrust, 0.56 at the 26.5 N peak (tulip pose: 0.60 / 0.81). Disc gap 234 mm, body clearance 17 mm.
- **Prop spin pattern** [+,−,−,+,+,−,−,+] (arms 0..7): balanced both for 8-prop flight and for the 0/2/4/6 square. Yaw uses differential reaction torque (share `mz_alloc`) plus tangential claw lean.
- Workers clutch winch tight (150 N line), flyers firm. Prop-disc collision geoms off (they collided between flyers 5 and 7 in an early run).
- Tools are free inertial bodies held by spring-damper grasp ties scaled to each tool's mass and inertia; sand and scrub forces are scripted (est.).

## 4. Training (CMA-ES, from a hand ansatz)
- 14 controller gains: position kp/kd/ki, a_max, lean_max, attitude kp/kd/ki, altitude kp/kd, heading kp/kd, arm lean time-constant scale, yaw share to prop torque.
- Ansatz = the chores4/steer4 values (lean_max 25°, a_max 1.2, attitude integral 12). Search in log-space around it (σ 0.5), 6 generations × 8, 13 s episodes. Cost: mean + ½ max position error, altitude, tilt, yaw, claw-prop saturation, servo > 0.9 × stall, grasp-tie stretch > 20 mm, crash.
- Best J per generation: 18.7 (ansatz) → 8.9 → 6.7 → 6.3 → 7.4 → 6.5 → **5.5**.
- Trained gains: kp_xy 2.87, kd_xy 7.27, ki_xy 0.54, a_max 2.43, lean_max 40° (at its bound), kp_att 40.5, kd_att 6.8, ki_att 8.7, kp_z 6.7, kd_z 2.3, kp_psi 7.5, kd_psi 4.0, tau_scale 1.02, mz_alloc 0.47. The optimizer chose much stiffer attitude (≈3×), more velocity damping, the full lean range.

| 13 s quad-mode episode (2 packs) | position error mean / max | altitude max | tilt max | yaw max | claw-props saturated | worst servo | clutch slip | min pack V |
|---|---|---|---|---|---|---|---|---|
| Ansatz, 100 % charge | 5.8 / 16.0 cm | 1.8 cm | 5.1° | 7.5° | 4 % | 0.82 | 0.2° | 22.7 V |
| **Trained, 100 %** | **1.3 / 3.2 cm** | 2.1 cm | **2.2°** | 5.5° | **0 %** | 0.82 | 0.2° | 22.7 V |
| Trained, 60 % | 2.0 / 8.0 cm | 8.9 cm | 4.2° | 7.1° | 51 % | 0.82 | 0.2° | 20.9 V |
| Trained, 40 % | 7.6 / 16.0 cm | 46.8 cm (sinks) | 6.7° | 7.8° | 88 % | 1.00 | 1.7° | 20.5 V |

- **Sustains flight with both chores at full charge**, sagging under 210 A.
- **Endurance is the real limit:** quad mode draws ~210 A (≈4.8 kW). The 13 s episode used 7.6 % of 2 × 5000 mAh, so 100 % → ~65 % (where saturation starts) is **about 1 minute of work**. Rule for the controller: quad mode only above ~70 % charge; below that, set the tools down and land on 8 claw-props.
- Not yet trained for low charge, gusts, or sand/scrub forces other than the scripted ones; the gains are specific to this task mix.

## Next
- XL Snap-Digit CAD + rotor loads; bench-test a 28xx + 7.5 in folding prop (thrust, power, spin-up).
- Fix the XL legged landing overload in body4.
- Battery-aware quad mode: a charge gate and a "set tools down, land" exit; train with SoC randomised 70-100 %.
- Larger packs (e.g. 2 × 6S 8000 mAh, +~1 kg) only make sense with the XL props; size them against the minute-scale endurance.
