# Chilo-1 rev 2: octopus as the head, style-A body legs, walk (2026-11-15, Cephalo-2 arms 2026-11-16)

Follows `chilo1-face-dock.md`. Body legs: **style A, arm-matched tentacle legs** (Forest's pick from `leg_styles.png`, `chilo_leg_styles.py`).
3D model (rotatable: standing, mid-walk, flying): artifact "Chilo-1 + Cephalo-2 3D model".
Code: `claude/code/chilo2.py` (model, IK), `chilo2_walk.py` (sim + monitors: `python3 chilo2_walk.py [assist|assist4]`), `chilo2_video.py` (`chunk <i> <n> <tag>`, `join <n> <tag>`), `chilo_arm_legs.py` (the arm: Cephalo-2 since 2026-11-16), `arm_leg_solve.py` (`python3 arm_leg_solve.py 0 24; python3 arm_leg_solve.py 1 36`), `chilo_c2_model.py` (GLBs: real arm + bayonet + hand3 CAD), `hand3_mesh.py` (hand3 as meshes, reads `hand3e/hand3_cad.py`, `hand3_finger.py`, `hand3_design.py`), `pack_model.py` (viewer data). Also `chilo_face_dock.py`, `geom.py`, `render.py`, `chilo_build_concepts.py`.

## Arms = Forest's Cephalo-2 arm (2026-11-16)
Forest: "make the arms my actual arm choice for the octopus".
- **Octopus** is Cephalo-2 (`cephalo2-baseline.md`), 14.81 kg (was a 12.81 kg Cephalo-1 stand-in).
- **Arm** is the rev 4 tentacle. It has 8 joints at 56 mm pitch and limits from `cephalo4.py`.
  - Current build: j0 and j1 are STS3250, j2-j7 STS3215.
  - Masses per joint come from the cephalo4 table: servo, shells 12 g, clutch (j0 38 g, j1 rigid, j2-j7 23.8 g) and 1/8 of the link structure and line.
  - Tip masses are the tip bay 24 g, the bayonet set 26 g and hand3 396 g (COM 58.4 mm out).
- **Bayonet** is end-mounted on every tip. The hand sits 9.9 mm further out.
- **hand3 hub C2 on all eight arms.**
- **Feet (arms 0/7, 1/6):** the hand is folded into a fist (fold 90/90/85) and stands on its knuckles, 127.9 mm past the hand-base face (CAD).
  - Sim contact: a 20 mm sphere, centre 117.8 mm past the spigot.
  - Standing on the heel was tried. The fingers are flat and only 18 mm above the heel plane, so the hand axis must stay within about 6° of vertical. With that constraint the side arms missed their targets by up to 155 mm and the walk without lift fell over. Rejected.
  - Fist corners sink up to 12 mm into the floor in the model at the tilted arm: the sim contact is one sphere.
  - Still to check: whether fist loads (24-36 N per foot) go into the fold stops or the cords.
- **Free arms 2-5:** fingers flat as a rotor, hand axis within 4° of up. In the 80 N case they are the props (20 N each, about 85 % of the hand3 ceiling).

### Results with the Cephalo-2 arm (25.1 kg: octopus 14.81 + centipede 10.28)
| | no lift | **80 N (4 hands)** | rev 2 stand-in, 80 N |
|---|---|---|---|
| Arm j2-j7 (STS3215): peak / p95 × stall, rms × rated | 1.00 / 1.00, 2.67 | **1.00 / 0.62, 1.60** | 1.00 / 0.42, 1.15 |
| Arm j0-j1 (STS3250): peak × stall, rms × rated | 1.00, 1.99 | **0.82, 0.65** | (j0 only) 0.69 |
| Body legs, worst group: peak × stall, rms × rated | 0.77, 0.70 | **0.43, 0.46** | 0.39, 0.38 |
| Spine pitch: rms × rated | 0.87 | 0.47 | 0.51 |
| Bus current mean / peak (A, est.) | 67 / 88 | 44 / 56 | 43 / 64 |
| Distance (asked 0.383 m) | 0.375 m | 0.339 m | 0.35 m |
| Head tilt max | 1.4° | 1.2° | 1.4° |
| Arm IK error max (low / side) | 30 / 26 mm | 23 / 26 mm | 41 / 22 mm |

- The STS3250 at j1 takes the root yaw off the list: it was the worst joint before, and it is now at 0.65 × rated rms.
- The extra 2 kg of Cephalo-2 moves the load out along the arm. The worst joint is now the outer STS3215s (j7 of the low arms with lift; j2 without), at 1.6 × rated rms.
- Body legs and spine stay well inside their servos.

## Rotor legs (2026-11-16)
Forest: "centipede legs get rotors". Every body leg now ends in the same propeller hand as the octopus: a bayonet plus hand3 (hub C2).
- **Leg:** the 195 mm shank becomes an 86 mm shank, then the socket, hand base and hand3.
  - Per leg tip: hand electronics 24 g (ESC + node, as the arm tip bay), bayonet 26 g, hand3 396 g.
  - The centipede goes from 10.28 to 14.49 kg; the whole robot is 29.3 kg.
- **Foot:** the hand3 heel (V-saddle), with the rotor flat. The fingers sit 18 mm above the floor when the shank is vertical.
  - `leg_ik` gets a third row that holds the shank vertical (`shank=`); the 4 pitch joints have one spare degree of freedom.
  - The rotor parks as an X at 45° to the body axis (`chilo_c2_model.LEG_CLOCK`).
- **Checks over the walk** (`rotor_leg_check.py`):

| | 80 N lift | no lift |
|---|---|---|
| Shank tilt max | 1.2° | 3.7° |
| Finger to floor, min | 14.9 mm | 11.2 mm |
| Neighbour leg fingers, min gap | 36.2 mm | 32.2 mm |

  - Parked with the fingers along the body axis instead, neighbours hit (-3.5 mm).
- **Walk loads, 80 N lift:**
  - leg STS3250 0.74 × stall peak, 0.43 × rated rms;
  - leg p3-p4 STS3215 0.70 × stall peak, 0.63 × rated rms;
  - arm j2-j7 1.63 × rated rms (unchanged);
  - bus current 49 A mean / 60 A peak.
- **Walk loads, no lift:** leg p3-p4 peak 0.90 × stall, 0.77 × rated rms; distance 0.376 m.
- **Flying pose** (kinematic, `chilo_c2_model.flight_pose`):
  - All 18 rotors point up: the 10 legs swing up with the shank vertical, and the 8 octopus arms hold their hands up (arms 0/7 under the head, 3/4 above it).
  - Neighbouring legs are staggered 45 mm in height, because spinning discs (Ø229) at 235 mm pitch would otherwise cross.
  - Min disc-to-disc gap 37.5 mm (L2/L3). Min disc-to-body gap 61.6 mm (arm 7 to the mantle; shells and mantle as capsules, arm links not checked).
- **Flight budget** (static, hand3 mid thrust 23.1 N, K 10.22 W/N^1.5):
  - 18 hands give 416 N for 287 N weight: T/W 1.45, hover at 69 % of the ceiling.
  - The belly rotor is in the dock, so it does not help.
  - About 650 W and 29 A per hand at hover, 11.7 kW in total.
  - With 4 × 6S 5000 (2 octopus + 2 centipede, assumed 111 Wh each), that is about 1.8 min to 80 %, and about 130 A per pack (needs ≥ 30C packs).
- **Open:**
  - motor power through the leg bayonet (same question as the octopus tips);
  - a take-off sequence (stand on the heels → swing the legs up one pair at a time while the arm hands carry);
  - flight sim.

## Model (`chilo2.py`, masses est.)
- 6 segments (head 280 mm, 5 × 200 mm, radii 125 → 75 mm), body axis 380 mm. 5 spine joints: yaw STS3215 + pitch STS3250.
- Head segment: Scabbard host hub on the face, no legs. Octopus docked rigidly. Arms 0/7 (low) and 1/6 (side) are fully simulated as legs.
- Body legs, segments 1-5 (10 legs), style A: coxa yaw STS3215 + 4 pitch joints (STS3250, STS3250, STS3215, STS3215) on 64/67/67 mm links + a 195 mm shank to a TPU ball foot. Leg ≈ 0.43 kg.
- 92 stall-limited position servos (kp = stall/2°).

## Controller (`chilo2_walk.py`)
- Body legs: head → tail wave, left/right in antiphase, duty 0.75. Arm feet: 4-beat, one at a time, duty 0.8. Cycle 2.4 s, stride 60 mm, 3.3 cm/s.
- **Arm joints are never commanded past 70 % of their range** (Forest: don't overextend the arms). IK pulls back to the solved pose. `arm_ik` can also hold the hand axis (`axis=`); that was used for the heel trial.
- **Arm-foot load balancer:** each planted arm's press depth is nudged so the arms share their worst-joint load, and all four keep the head level.
- Prop lift: the free hands, applied as a constant upward force on the octopus body (est.).
- Monitors: load per servo group (peak and p95 × stall, rms × rated); 12 V bus current (est.); arm reach and joint range; IK error; feet down; head tilt.

## Rev 2 stand-in history (12.81 kg octopus, simplified arm)
- With lift 0 / 40 / 80 N: arm STS3215 rms 2.55 / 1.89 / 1.15 × rated.
- Rejected:
  - octopus rolled 22.5° on the hub (1.7-2.3 × rated);
  - body axis at 450 mm (1.8-2.1 × rated).
- Video `chilo1_octopus_head_walk_assist4.mp4` shows that stand-in, not Cephalo-2.

## Next
1. Get the outer arm joints inside rated. Options:
   - shorter arm-foot stance (the low arms carry the most);
   - let segment 1's legs and the hub take more of the octopus;
   - STS3250 on j2 and j3 of the leg arms only.
2. Check the fist as a foot: knuckle loads into stops or cords, and a pad on the phalanx-2 backs.
3. Re-render the walk video with the Cephalo-2 arm (`chilo2_video.py` still draws the stand-in arm and body).
