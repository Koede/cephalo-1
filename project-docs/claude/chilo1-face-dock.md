# Chilo-1 face dock: the octopus becomes the centipede's head (concept, 2026-11-15)

Status: **concept sketches**, plus a static pose and load check of the planted arms. Not simulated, no CAD. Supersedes the dock ideas in `chilo1-plan.md` / the body study sheets (hub on the back, deep body).

Code: `claude/code/chilo_face_dock.py` (four-step sequence → `face_dock.png`), `chilo_face_legs.py` (first arms-as-legs sketch, superseded), `chilo_arm_legs.py` (arm kinematics, load model), `arm_leg_solve.py` (one-arm pose solve, free foot placement), `chilo_arm_legs_render.py` (→ `arm_legs.png`). They use `chilo_dock_designs.py` (octopus stand-in), `chilo_build_concepts.py`, `geom.py`, `render.py`.

## How it got here (Forest's decisions, in order)
1. Every centipede design must be able to couple Cephalo-1.
2. The body must let the octopus's Scabbard blades extend fully (224 mm below the plate), not sit above a shallow body.
3. Not a box: a shaped body.
4. **The connector goes on the centipede's face. It stands up to couple the octopus, then stands back down on all its legs.**
5. **Some of the octopus's arms then become centipede legs.**
6. **The front legs are the octopus's arms (no centipede head legs), and the planted arms must not be overextended.**

## Concept
- **Hub on the face:** the host side of the Scabbard dock (`scabbard.py`: collar r 100-118, 4 guest sockets at r 65-74.6, latch pins) faces forward on the head segment. The blades slide in along the body axis, so the head segment only needs to be about 280 mm long and 250 mm across; no deep body.
- **Body (sketch sizes):** 6 rounded segments, lengths 280 / 200 × 5 mm, radii 125 → 75 mm, about 1.3 m long, body axis 380 mm above the floor. The head segment has no legs: 10 centipede legs + 4 octopus arms.
- **Sequence:** walk up → rear up (segments 0 and 1 vertical, segment 2 at 45° with its legs planted, segments 3-5 flat) → octopus descends, blades slide 224 mm into the head, latches lock → front lowers with the octopus as the head.

## Arms as legs (`chilo_arm_legs.py`, `arm_leg_solve.py`)
Real rev 4 arm: root at the wall face (r 104, 39 mm above the plate), 8 joints p-y-p-y at s = 50…442 mm, spigot 478; j0 STS3250, j1-j7 STS3215; limits j0 −59/+55°, j1 ±30°, j2-j7 ±100°. Foot = hand3 folded (396 g, contact 100 mm past the spigot, est.). Octopus axis horizontal and forward; arm i at 22.5° + 45°·i from straight down.

"Not overextended" is enforced as hard limits in the solve:
- every joint within **70 %** of its range;
- foot no further than **80 %** of the arm's full length (578 mm) from its root;
- every joint ≥ 40 mm above the floor and clear of the head segment (≥ 25 mm ahead of the face, or ≥ 150 mm from the body axis).

The solve then minimises Σ(τ/rated)² with the foot placed freely on the floor (lateral bands: low arms |y| ≥ 140 mm, side arms ≥ 240 mm). Joint torque = floor reaction + joint and hand weights.

| Planted arm | Reach | Worst joint | Load, no assist | Load, 40 N prop lift |
|---|---|---|---|---|
| 1 / 6 (side-down, 67.5°) | 63 % | 70 % (at the margin) | 31.4 N/foot → **0.74 × rated**, 0.25 × stall | 20.9 N → 0.41 × rated |
| 0 / 7 (low, 22.5°) | 53 % | 70 % (at the margin) | 20.9 N/foot → **1.18 × rated** (j3), 0.40 × stall | 14.3 N → 0.70 × rated |
| 2 / 5 (side-up, 112.5°) | no pose found inside the limits | | | |

- **Legs = arms 0, 7, 1, 6; hands = arms 2, 3, 4, 5.**
- **Load split:** 2 × 31.4 + 2 × 20.9 = 105 N of the octopus's 126 N on the arms; the other 21 N goes through the hub into the centipede (via its spine and segment-1 legs). With 4 equal feet (31.4 N each) the low arms reach 1.95 × rated, so the controller must put more on the side arms.
- **Without assist the low arms sit above rated (1.18 ×)** while standing: fine in bursts (0.40 × stall), not for long holds. With two free hands spinning as props (40 N total, est.: hand3 thrust is unmeasured) everything is under rated (worst 0.70).
- j0 and j1 end at the 70 % margin in every solve: the arm roots want to bend further than allowed. The low arms are cramped (root only 284 mm off the floor) rather than stretched.

## Open points
- **Lowering with the octopus aboard:** 12.8 kg at the end of the raised front is far beyond the spine pitch joints and could tip the centipede forward. Proposed: the octopus keeps flying (its props are its hands) and carries its own weight until its arm-feet are down.
- **Hub share:** how much the face hub actually passes into the centipede depends on the head-spine stiffness; a limp joint puts about 90 % on the front feet. Holding more there needs about 29 N·m at that joint (est.), far beyond an STS3250.
- **Hand3 as a foot:** walking on the folded 4-finger hand (pads, finger joint loads) is unchecked; hand3 thrust is unmeasured.
- **Walking:** with only 4 arm-feet up front, lifting one in a step puts the others at up to 1.4 × stall without assist; the gait needs prop assist or the hub share.
- **Scale:** the centipede body is unsized (actuators, mass, power) at this size.

## Next
Simulate rear-up → couple → lower with the octopus's thrust assisting, then the 14-foot walk (10 centipede legs + 4 arm-feet) with loads monitored.
