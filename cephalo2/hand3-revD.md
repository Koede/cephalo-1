# hand3 rev D: the smaller hand, drawn on the robot, grasping by contact (2026-11-14)

**Later the same evening the finger drive was replaced by hub C (two rings, two STS3215, 374 g): `claude/hand3-hub-rethink.md`. The rotor, fingers, renderer, contact model and design rules below still stand; the drum, the 10 servos, the masses and the flight / shovel numbers below are the drum version.**

Three requests from Forest in one session: "render the fingers as propellers", "assure they genuinely grasp", and, on seeing the result, "those two guns and fans look so off, shrink their design a bit". Forest picked **rotor 9 in + slim drum (fingertips driven by the middle cord)** from three options. Everything here is an estimate unless marked; no spin test, no hardware.

## Rev D in one table (rev C in brackets)
| | rev D | (rev C) |
|---|---|---|
| Rotor | 9 in, R 114.3 mm, pitch 16° | (10 in, R 127, pitch 12°) |
| Knuckles K1 / K2 / K3 at r | 36 / 62 / 89.5 mm | (36 / 66 / 98) |
| Servos | 10 × STS3032: 4 base, 4 middle + tip cord, 2 spread | (14: 10 × STS3032 + 4 × SCS0009) |
| Finger joints | K1 own servo; K2 + K3 on one cord (tip follows, wraps on contact) | (every joint its own servo) |
| Drum | Ø94 × 54 mm; Ø102.6 over the claw bars | (Ø102 × 54; about Ø110.6 over the bars) |
| Mass | 495 g (CAD 492 g), COM 48 mm from the spigot | (570 g, 45.5 mm) |
| Full-throttle thrust low / mid / high | 20.2 / 21.7 / 23.3 N | (21.3 / 23.35 / 25.4 N) |
| Full-throttle speed (low) | 12,550 rpm (design speed 12,400, proof 14,260 unchanged) | (12,040) |
| Electrical power at 11 N | 387 / 290 W (low / high) | (357 / 259 W) |
| Rotor q/T | 0.0295 / 0.0279 / 0.0262 m | (0.0297 / 0.0274 / 0.0250) |

- Why pitch 16°: the 9 in rotor at 12° under-loads the motor (18.1 N at 13,600 rpm). A 5-point pitch scan (12-20°, chord × 1.0 and × 1.15) puts the best low-estimate thrust at 16° with the chord unchanged. Not a full blade optimisation (`hand3_blade_opt.py` not rerun).
- Middle + tip cord: one Dyneema cord over K2 (4.5 mm lever) and K3 (3.5 mm lever), spool r 3.0 mm, travel 15.3 mm, 76.4 N at stall, 25.5 N rated, margin 1.73 on the 5 N fingertip requirement. **New requirement: the elastic extensor needs about 0.02 N m of pretension at K3**, so the middle joint closes first and the tip wraps last. Without it the tip curls first and the grasp is about half as strong (bench: pull-off 8 N instead of 15 N).
- Drum: 5 servos per tier instead of 7, so the tier's inner radius drops 18 → 14 mm (servo-to-servo gap 1.0 mm), spools at r 44.5, claw bars at r 42.8-49.8.
- CAD (`hand3_cad.py`, 24 parts, all single watertight bodies) passes the rev C checks with the same values, except: the 16° root grip grazes the lower hub plate at 47° spread (0.02 mm³: needs a 0.2 mm relief), and the smallest gap in the flight spin is still 0.5 mm (motor bell), 0.88 mm phalanx to claw bars.
- **Not redone for rev D:** servo drum v2 and the arm collar (`hand3_pack2.py`, `hand3-drum-mount.md`: still the 14-servo layout, so the "618 g with drum v2" figure no longer applies), the metal STLs and order note (`hand3_metal.py`), `hand3_look.py`, the blueprints, `hand3_loads.py`, the blade optimisation, the cord channels in the phalanx solids (still drawn for three cords), the step program. Rev C files are kept in the workspace as `*_revC*`.

## Flight with rev D (day mission, `hand3_fly.py low|mid|high day`, robot 12.51 kg)
Flies at all three thrust levels; body tilt in the air ≤ 1.9°, no refusal, pack peak 259-275 A, lowest bus 22.1 V.

| Hand thrust / its ceiling | low | mid | high |
|---|---|---|---|
| Take-off | 0.93 | 0.86 | 0.83 |
| Fly north / east | 0.92 / 0.87 | 0.86 / 0.81 | 0.81 / 0.78 |
| Hover | 0.64 | 0.59 | 0.55 |
| Landing touchdown (peak) | 1.00 | 1.00 | 0.91 |
| Time above 0.98 of the ceiling | 0.27 s | 0.13 s | 0 s |

The smaller rotor costs margin: at the low estimate the hands peak at 0.92-0.93 of their ceiling in the flight legs (0.82-0.87 with the 10 in rotor). Monitor flags are the baseline's (thermal 1.41 on arm 0 j0, EDF 1.03 of its ceiling for 0.2 s).

## Fingers drawn as the rotor (`hand3_draw.py`)
- Draws hand3 from its CAD solids on the arm tip: drum, claw bars, servos, motor, hub, cage, followers, heel, and the four fingers. With the arm's thrust on, the fingers are straight and turn with the hub: they are the rotor. Stopped, they take the pose in `hand3_draw.STATE` (spread and knuckle angles, e.g. from the sim).
- `hand3_draw.install((2, 6))` hooks into `viz4` without editing it. `bv.py` = `body_video.py` for a hand build; `quad_video_r.py` / `qv.py` = hand work mode, with a close-up along the shaft.
- Videos: `cephalo1_body_h3dfly_mid_day.mp4` (take-off, flight, landing with the hands flying) and `cephalo1_hand_work_shovel.mp4`.

## Grasp by contact (`hand3_mj.py`)
The fingers are MuJoCo bodies (spread + 3 knuckles per finger, masses from the CAD). Collision: a thin box per phalanx (blade + pad) and a barrel at each knuckle, the TPU heel and the hub plate; friction 0.8 (est.), elliptic cone, torsional contact. Drives as designed: one-way cords with the TPU link's stiffness in series, limited to servo stall; the fingers close under a force cap, then the servos hold position at 0.7 × rated.

**Bench, one hand on a bar lying on the heel between the two finger pairs** (load at which the bar moves 3 mm or 5°, Ø28 bar): pull off the heel 15.5 N, push on the heel 98 N, across 6.5 N, slide along the bar 9.9 N, cantilever moment 0.28 N m, roll about the bar 0.06 N m, twist about the rotor axis 0.17 N m (`hand3_grasp_capacity.json`, also Ø20 and Ø38). **The weak directions are the moments: one hand cannot hold a lever arm.**

**Shovel while flying (`H3_FINGERS=1`, hand work mode, six claw-props + EDF fly):** the shovel (0.40 kg + 0.20 kg scoop, Ø28 shaft) is held by finger contact and friction only from 0.7 s on (ties only while the fingers close). Over 13 s and three dig cycles: **no drop; the shaft moves at most 6.9 mm and 5.9° in the hand and comes back (no creep)**. Flight: position error mean 1.6 / max 3.2 cm, height 2.0 cm, tilt 2.4°, no prop saturation, J 8.3. Knuckle servos rms / rated 0.70 (K1) and 0.89 (cord); hand servo current peak 2.9 A per hand. Clearances: hand drum to the neighbouring arm 21 mm, to the EDF 102 mm, to the mantle plate 74 mm; arm to arm 80 mm, arm to body 18 mm.
What it took (design rules):
1. **The hand is clocked 45° on the wrist** (about the rotor axis). The grip axis (hand y: the line a bar lies along on the heel) is fixed in the hand, and the arm has no roll joint, so the clocking decides what the arm can line up with. 45° keeps every arm joint inside 80 % of its range through the dig cycle; 0° gives 92 %, 90° 98 %.
2. Grip points from a reach search (`shovel_search.py`): upper (0.137, 0.179, -0.085) m, lower (0.161, -0.121, -0.288) m in the body frame; the shaft passes ≥ 85 mm from the EDF axis.
3. Two hands on one shaft: each hand lines its grip axis up with the line through the two hands as they really are, and the upper hand trims its place along the shaft so the hands stay the grip distance apart. Without this the hands fight: 18° of twist in the hand and 12-20 mm of creep in 13 s.
4. IK for a hand arm: position + grip axis, spare joints pulled to mid-range (`ik_g` in `quad4r.py`).
Flags: arm 2's j5 servo runs hot holding the hand in this pose (thermal 1.65 × rated, lead current 1.02 × the pigtail rating, j3 at stall for an instant). The shovel pose needs a second look for the arm, not for the grasp.

**Dishes: not assured.** Holding a 0.25 kg pan by its handle with one hand needs 0.32-0.44 N m; the grasp gives 0.28 N m. In the contact run the pan dropped 0.3 s after the ties were cut. Carrying the pan on the heel, palm up, is out of the arm's reach below the body, and a first reach search found no pan place where the second hand can also scrub with its grip axis on the brush. The earlier dishes result (J 7.3) used the tie model and overstates what the hand can do. Options: two hands on a two-handled pot, a lighter dish, or a pan that rests on a surface.

## Code
`hand3_design.py` (rev D, single source of truth; merged with the 2026-11-14 STS3032 data: 4.8-6 V, 1.2 A stall), `hand3_thrust_cad.json` (rev D thrust), **`hand3_revD.diff` (the rev C → rev D patch for `hand3_cad.py` and the three-line change to `hand3_cad_checks.py`: the project copies of those two files are still rev C)**, `hand3_params.py` (the numbers the sims use, read from the design files), `hand3_mj.py` (contact model + bench), `hand3_draw.py` (renderer), `quad4r.py` + `quad_h3.py` (hand work mode, `H3_FINGERS=1` for the contact grasp), `hand3_fly.py`, `bv.py`, `quad_video_r.py`, `qv.py`, `shovel_search.py`, `hand_clear.py`, `thrust_d.py`. `body4h` and `monitor4` got small changes (extra actuators after the 64 arm servos; the monitor skips the hand's own servos).
Run: `H3_FINGERS=1 H3_SPIN=opp QUAD_FILE=quad4r.py Q_TASK=shovel python3 quad_h3.py low quad4_g.json`; `python3 hand3_mj.py`; `python3 bv.py still 41 body4_h3dfly_mid_day.npz`.
