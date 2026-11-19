# Wheel hand + hand connector rev B (Cephalo-2, 2026-11-18) — draft, stance not solved yet

Ask (Forest): a new hand that slides into the hand connector instead of the rotor, and uses the rotation the propeller would get to drive a wheel. Decisions (Forest): the motor lives **in each hand** (the connector carries motor power); **any arm** can take a wheel, designed for **four at once** (the other four keep hand3).
Code: `claude/code/handmount_b.py` (connector rev B), `claude/code/wheelhand.py` (CAD + checks), `claude/code/wheelhand_sizing.py`, `claude/code/wheelstance.py` (stance + servo loads).

## Finding that comes first
hand3's rotor motor is wired straight down the arm today (`hand3-build-kit.md`: phase leads never go through the rev A pins, 5.2 A each). So no motor hand could really be swapped. Connector rev B fixes that for every motor hand.

## Connector rev B (`handmount_b.py`) — checks pass
- **Three phase contacts:** Castle Creations 4 mm high-current bullets (75 A; hand3 peaks at 55 A). Females recessed 1.2 mm in the arm socket (touch-safe), males in the shared hand base, 8.2 mm engagement, 14 AWG leads.
- **The hand no longer twists** (bullets need a straight push). It is keyed (one rib, wrong orientations blocked) and pushes straight in; a **locking collar** on the socket turns 60° and three lip segments slide behind the hand's flange lugs. Ball detent at open and locked.
- The 5 spring pins (12 V, GND, servo data, servo return, hand ID) stay on the same pin board in the centre.
- Parts: socket 17.6 g, collar 19.3 g, hand base 15.4 g (PA-CF). Socket Ø46, collar Ø53.
- Checks on the solids: no overlap locked / pushing in / through the collar turn; a locked hand cannot be pulled out; key blocks the other clockings.
- Strength (lugs as a couple, 30 N preload, est.): factor vs 30 MPa PA-CF interlayer 4.2 standing on a wheel, 2.4 rolling over a step (2 g), 1.7 in a 3 g landing on a wheel. **Rule: land on the blade legs, not the wheels.**

## Wheel hand (`wheelhand.py`) — CAD checks pass, 289 g
| | |
|---|---|
| Motor | T-Motor F60 Pro V 1750 kV (Ø26.8 × 31.7 mm, 34.3 g, 4 mm shaft), driven by the arm's ESC through the bullets |
| Gearbox | 2 planetary stages, 36 : 1. Stage 1 m 0.5: sun 15 (brass) / 3 × 30 (POM) / ring 75. Stage 2 m 0.8: sun 12 / 3 × 24 / ring 60. Rings cut in the PA-CF housing; carriers CF resin; planet pins Ø2 / Ø3 steel |
| Output | Ø8 D-shaft on 2 × 698-2RS in the housing nose |
| Wheel | dished PA-CF hub, rim r 54 reaching back over the gearbox, 6 mm TPU tyre, Ø120 × 32. Tyre centre plane 72 mm from the socket mouth; gearbox and collar 33 mm off the floor |
| Speed | 1.5 m/s at 22 % of the motor's no-load speed; smooth sensorless from about 0.35 m/s (AM32 sine start below that, est.) |
| Torque | 0.85 N·m per wheel for a 15° slope + 1 m/s² at 6.1 A; grip limit (µ 0.8) 1.74 N·m at 11.5 A |
| Gears at the grip limit | sun roots 8.7 / 17.5 MPa (Lewis, 3 planets, load factor 1.25) |
| ESC | **current limit 13 A in wheel mode** (set from the hand-ID pin): unlimited, the motor's 40 A would push 6.6 N·m and strip the printed gears |
Checks: every pair of parts free of overlap except the intended fits; gears turned through a full tooth pitch with backlash kept (0 interference); housing prints as one body.

## Stance: not solved — the servos cannot carry it (`wheelstance.py`)
Four wheels on arms 0, 2, 4, 6 (14.5 kg robot, 36 N per wheel). Servo load as a multiple of rated torque (STS3215 0.98 N·m), best stance found by search:
| Case | Axle along the arm tip (this CAD) | Axle across the tip, arm as a column (bevel-drive variant, est.) |
|---|---|---|
| Standing still | **5.6 ×** rated (1.9 × stall) | 2.1 × rated (0.7 × stall) |
| Driving: 15° slope + 1 m/s² | 7.6 × | 6.5 × |
| Full grip, forward or braking | 10.7 × | 11.1 × |
| 0.3 g sideways | 6.7 × | 5.4 × |
Why: the arms' joints are rated about 1 N·m. With the axle along the tip the arm must turn 90° at its end and the wheel load sits on that lever. As a column the vertical load goes straight up the arm, but every drive and braking force acts on the whole leg length (0.4 m+) at the root joints. Driving on these arms needs one or more of: all 8 arms on wheels, a much lower crouch, stronger root servos, a mechanical lock of the wheel arms, or gentle flat-ground driving only (about 0.3 m/s² on the flat keeps the column variant near rated).

## Open
1. Choose the stance strategy above, then redo the stance and loads.
2. hand3 needs a rev B hand base and its motor leads moved onto bullets.
3. Print-test the collar lip and the PA-CF lugs; measure the bullet insertion force.
4. Bench-test sensorless low-speed start of the F60 through 36 : 1.
