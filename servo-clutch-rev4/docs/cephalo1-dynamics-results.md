# Cephalo-1 dynamics (MuJoCo) — results so far
Code: /home/claude/cephalo/dyn (sim_model.py, drop3.py, agility.py, gallop.py). Model: 8.63 kg, real link masses/COMs, 72 servos with stall + no-load-speed limits, tripod claw contacts, EDF 42.2 N and claw-prop thrust (7.8 pess / 10.8 opt N) with lag. Matches CAD FK to <1 mm. Stands at 423 mm with 10.6 N per foot; joint 6 at ~0.97x rated static.
(That code was not saved to the project; the wrist hit test below is a separate one-arm model in `claude/code/sim_wrist.py`.)

## Drops (stand pose, rigid contact, solref 4 ms — contact stiffness is crude)
2 cm: 15.7 g, GRF 12x weight, joints 0/6 at stall for 26 ms. 10 cm: 39 g, 27x. 30 cm: 60 g, 43x. 50 cm: 81 g, 54x, sag 49 mm. Braking thrust does not help (thrust at 50% is small vs impact). Conclusion: landing is the weakest point; needs compliant feet/series elasticity and a thrust-cushioned landing.

## Agility (flight pose, perfect attitude control, yaw not modelled)
T/W 1.24 (pess) to 1.52 (opt). Climb accel 2.3 / 5.0 m/s2. Roll/pitch torque at hover 4.5 / 7.5 N m, I=0.353 kg m2 -> 12.7 / 21 rad/s2; bang-bang 30 deg in ~0.40 / 0.31 s. 30 deg tilt dash covers 5.7 m in 2 s. Yaw authority needs prop reaction torque (not modelled).

## Gallop search (CMA-ES, 30 min x2, 23-param joint-space gait)
No aerial phase found. Best no-assist 0.77 m in 4 s; EDF 50% 1.24 m in 4 s. Not a gallop yet.

## Wrist hit test (2026-11-08) — `sim_wrist.py`, `sweep_wrist.py`, video `cephalo1_wrist_hit_mujoco.mp4`
Model: one rev 3 arm in the flight v2 "tulip" pose (j0 −40, j2 −35, j4 −15), mantle held fixed, position servos with stall-torque limits (STS3250 4.9, STS3215 2.9 est., STS3032 0.44 N·m), 647 g arm, 100 g pod, hover thrust 5.5 N. Wrists: geared knuckle + roll (gear mesh = equality constraint) and magnetic Ball Drive (free 50 g steel ball on roller contacts, magnet F = Fc/(1+g/2 mm)², Fc swept 15/30/60 N, driven-wheel torque ≤ 0.35 N·m, badge-reel 1.5 N, tether 130 mm). Tip fail-safe: fan cut while the pod is > 25° off vertical. Hit: 0.2 kg sphere at 2/4/6 m/s into the prop disc, sideways. Yank: upward pull on the pod ramping at 25 N/s.

| Case | Impact peak | Result |
|---|---|---|
| Knuckle, 2 m/s | 78 N | j5 swings 36°, recovers in 1.0 s. Gear tooth 0.9 N (2% of the 48 N Lewis estimate), pin 17 N, wrist servos never saturate |
| Knuckle, 4 m/s | 143 N | **j5 knocked to its 90° limit and stays**: the arm beyond j5 needs ~0.5 N·m to lift back, the STS3032 gives 0.44 |
| Knuckle, 6 m/s | 215 N | j5 and j7 swing 95° / 64°, bounces back, recovers in 1.7 s; pin 50 N |
| Magnetic 30 N, 2 m/s | 61 N | stays seated, j5 36°, recovers ~1.0 s |
| Magnetic 15/30/60 N, 4 m/s | 109–125 N | stays seated; **arm folds at j5 like the knuckle** |
| Magnetic 15 N, 6 m/s | 188 N | breaks away (sheds the hit), arm only swings 41°, ball reels back and re-seats in 0.47 s |
| Magnetic 30/60 N, 6 m/s | 180–190 N | stays seated, arm folds at j5 |
| Yank, knuckle | up to 80 N | holds; pin 104 N; root j0 reaches 4.4 of 4.9 N·m at 80 N |
| Yank, magnetic | — | breaks away at 11.6 / 26.2 / 55.5 N (≈ Fc); tether snap peaks 150–460 N (crude contact model; 1 mm Dyneema ~900 N) |

Findings:
1. **The wrist is not the weak link; the arm's j5 yaw servo is.** A ≥4 m/s side hit folds j5 (STS3032) to its stop and it cannot lift the distal arm back. With j5 upgraded to an STS3215 (2.9 N·m, +35 g per arm, +0.28 kg robot) every case recovers in < 1 s (j7 still swings up to 94° at 6 m/s but returns).
2. **Knuckle loads are tiny**: gear teeth ≤ 2 % of the printed-tooth estimate; pins ≤ 110 N (M3 steel is fine; check the printed lugs). The wrist servos never saturate.
3. **Magnet strength is a trade**: it must hold full thrust (10.8 N) with margin, so Fc ≥ ~20 N, but at 30 N it no longer breaks away to protect the arm. The magnetic ball still spins up to ~85° in its seat during a hit (traction-limited).
4. **The fan fail-safe is needed**: without it, a knocked pod's off-axis thrust back-drives the small servos.
5. **Wide flight pose with wrist-vectored fans is not feasible on rev 3**: holding fans vertical with the arms spread (j0 −20, j2 −20) needs 0.45 N·m on j6 (STS3032 stall 0.44, rated 0.15) and 0.28 on j8. Keep the tulip pose, or upgrade j5–j8.

Caveats: servo back-drive holding torque may exceed stall; contact softness, link masses, the magnet law and wheel traction are estimates; the body is held fixed.
