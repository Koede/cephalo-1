# Cephalo-1: in-flight chores (shovel, dishes), landing and lying down to rest (2026-11-10)

Asks (Forest): pick a shovel up off the ground, carry it, pass it to another hand, grasp it with another one and use it; fly to a kitchen sink and do the dishes; land on its feet; lie down to rest. Delivered as video snippets. Code: `claude/code/chores4.py` (shovel + dishes), `rest4.py` (lie down), `chores_video.py` (clips). Builds on `organic-flight-steering-grab.md` (tentacle steering, grasp model).

Clips (1280×720, real time): `cephalo1_chore_1_shovel_pickup.mp4`, `_2_shovel_handover`, `_3_shovel_dig`, `_4_dishes`, `_5_landing` (cut from the rev 4 mission video), `_6_lie_down`.

## Which arms can be hands in flight
With two claw-props stopped, the other six must still balance the robot. Best static split with a 0.4 kg load (EDF at 100 %):
| Stopped claw-props | Needed per running claw-prop |
|---|---|
| one arm, or an opposite pair (2 + 6) | 10.2 N |
| 135° apart | 11.9 N |
| 90° apart | 13.9 N |
| neighbours, or any three | 15.3 N |

Mid estimate is 12.6 N, so **only an opposite pair works as two hands** (arms 2 and 6 here). A third hand in flight needs ≥ ~14 N claw-props, or the robot has to land first.

**Hand workspace under the body** (claw pointing down, arm ≥ 25 mm clear of the mantle and EDF): grasp point 0.18-0.38 m out from the centre on the arm's own side, 0.16-0.38 m below the plate. The two hands therefore cannot meet under the centre; tools must span ≥ 0.36 m between them.

## Method (`chores4.py`)
- Flight: tentacle steering (`steer4.py`), EDF 100 %, lean ≤ 25° and ≤ 1.2 m/s² while holding something, breathing off while the hands work.
- **Attitude integral (new, 12 N·m/rad·s, frozen while a claw-prop is saturated):** trims the tilt from an off-centre load. Without it the carry ran 25-38 cm off the path with 4° tilt; with it 5 cm and 0.7°.
- Hand paths: planned in the body frame, dense IK every 0.05 s (position + claw axis + clearance + joint-limit terms, strongly tied to the previous solution so the arm does not flip between postures). Peak planned joint speed 156-183°/s.
- Tip-sensor alignment (est.): in the last ~1 s before each grasp the hand is nudged (≤ 6 cm) onto the object's grip point.
- Grasp model: if the finger centre is within 3 cm of the grip point when the fingers close, the object is tied to the hand by a spring-damper plus a soft rotational spring (grip friction, est.). Finger and bristle contact are not simulated. The reported grip force and torque are what the fingers must hold.
- `sim4.build()` now accepts extra free bodies (`EXTRA_XML`) for tools.

## Shovel (0.40 kg, 0.80 m, balance point 0.50 m from the handle end; est.)
Sequence: descend to 0.415 m → arm 2 stops its prop, reaches 0.37 m below the plate and grips the shovel at its balance point (handle axis 45 mm above the ground) → lift → carry 1.2 m on 7 claw-props → arm 6 stops its prop and takes the top of the handle → both hands tilt the blade 18° down → push 4-7 cm into the sand → lever a 0.2 kg scoop out → fly it 0.35 m aside, tip it off → fly back, stab the shovel into the sand, let go → props back on.

- **Hand-over:** one claw cannot hold the shovel away from its balance point (1.7 N·m of grip torque at the handle top). So arm 6 joins as the steering hand and arm 2 stays as the lifting hand; a full swap of hands was not possible.
- Tilt is limited to ~18-22° by the hand workspace (grips 0.43 m apart). Dry sand needs ~30° to slide off, so the tip-off needs a shake or a third joint motion (not modelled).
- Sand (est.): support 1500 N/m up to 15 N and drag 40 N·s/m up to 10 N on the blade tip.

| Mid thrust | Path error | Body tilt | Worst servo | Peak claw-prop | Grip load |
|---|---|---|---|---|---|
| Pick-up (finger offset 10 mm, grasped) | 3.3 cm | 3.3° | 0.71 (arm 2 j3) | 12.2 N | 10 N, 0.09 N·m |
| Lift / carry on 7 claw-props | 9.9 / 5.0 cm | 3.7° / 0.7° | 0.43 | 12.4 N | 4.3 N |
| Hand-over (finger offset 18 mm, grasped) | 3.5 cm | 1.6° | 0.61 (arm 6 j2) | 12.4 N | 5.5 N, 0.17 N·m |
| Tilt, push in | 4.5 cm | 1.5° | 0.44 | 12.6 N (saturated briefly) | 5.2 N, 1.0 N·m |
| Lever the scoop out | 10.4 cm | 3.0° | 0.65 (arm 2 j4) | 12.6 N | 7.7 N, 1.0 N·m |
| Carry aside, tip off | 13.0 cm | 2.1° | 0.58 | 12.6 N | 7.4 N, 0.65 N·m |
| Stab back, let go | 6.7 cm | 2.9° | 1.00 for a moment (arm 2 j0, arm swinging back) | 12.6 N | 5.3 N, 1.0 N·m |
| Whole run | | | rms 0.71 × rated | | |

The ~1 N·m grip torque in the two-handed phase is the two soft ties resisting the planned tilt; a real claw would let the handle turn in the fingers.

## Dishes (0.25 kg skillet with handle, 0.06 kg dish brush; est.)
Hover at 1.22 m over a sink (counter 0.90 m). Arm 2 takes the brush from its rest; arm 6 grips the pan handle and holds the pan just above the sink grid; arm 2 scrubs three 8 cm circles inside the pan; arm 6 sets the pan on the drying rack; the brush goes back; props on.

- Both grasps made (finger offsets 9 and 11 mm). Path error ≤ 5.7 cm, tilt ≤ 2.9°, peak claw-prop 11.8 N.
- Holding the pan by its handle needs 0.4-0.7 N·m of grip torque (2.5-4 N force).
- Worst servo 0.94 × stall (arm 6 j4, moving the pan out to the rack) and 1.00 for a moment as the arms swing back to the flight pose.
- Not modelled: water, bristle pressure, and the EDF downwash onto the sink (42 N of air straight down; it will splash).

## Landing
Unchanged from the rev 4 mission (take-off v7 reversed with the v6 leg path): `sim4.py`, clip cut from `cephalo1_rev4_mission.mp4`. Landing while holding a tool has not been simulated.

## Lie down to rest (`rest4.py`, on the ground, fans off)
1. Crouch (3 s): feet stay put, arms arch up, the body sinks until the EDF can sits on the floor (plate 92 mm up). Worst servo 0.24 × stall. A printed TPU bumper ring on the EDF outlet is assumed.
2. Lie down (3.6 s): each arm lifts its foot, swings it out and is laid flat radially, 0.12 s apart around the body. The arm ends straight, sloping 15.5° down from its root, folded fingers on the floor 0.57 m out. One servo stalls for a moment and its clutch slips 43° (arm 0 j4, the fast unfold); slower would avoid it.
3. Rest: winch soft (25 N). Holding load 0.17 × stall, rms 0.48 × rated; the servos could be powered down. Footprint ≈ 1.25 m across.
Clearances: ≥ 77 mm between arms, ≥ 17.6 mm to the body, ≥ 19.7 mm above the floor.

## Clearances in flight (states4 check)
Shovel run: ≥ 82.0 mm between arms, ≥ 22.1 mm arm to body. Dishes run: ≥ 81.4 mm, ≥ 20.5 mm. Tool-to-robot collisions are not checked numerically (by eye in the clips).

## Open items
- Everything here runs at mid thrust with claw-props at or near their 12.6 N limit. The pessimistic case was not run; expect it to fail, as the ball grab did.
- Grip: can the three finger-blades hold ~1 N·m on a 28 mm handle? Fold servo + slider force is unmeasured.
- A wrist roll (the optional tip roll, +45 g) would allow steeper digging and a real tip-off.
- Digging force in flight is limited to ~10-15 N; real digging should be done standing on six legs with arms 2 and 6 (no thrust limit, any pair of arms).
- Landing and take-off with a tool in hand.
