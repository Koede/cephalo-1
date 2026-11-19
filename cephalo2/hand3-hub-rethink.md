# hand3 hub rethink (2026-11-14)

Forest, after rev D: "Rethink this hub, I'm not sure it's the best way to do this." This note is the rethink: what the hub has to do, what is wrong with the current one, six ways to do it, and a recommendation. **Concept study: masses and forces are first estimates from parts already in the project; only the grasp bench (option B) is simulated. Nothing here is CAD yet.**

## The job
The four fingers are the rotor blades. In flight they spin at up to 12,550 rpm and nothing may touch them. Parked, they must spread into two pairs and curl round a handle with about 15 N of pull-off. So finger actuation has to cross a joint that spins freely in flight.

## What is wrong with the current hub (rev D, "park and dock")
- **Most of the hand is finger drive.** Of 495 g: 10 servos 200 g, drum frame 48 g, claw bars 18 g, cage + followers 21 g, spools / wiring 28 g = **315 g of actuation** for 48 g of fingers. Motor 98.5 g.
- **It is the bulk you disliked:** Ø94 drum, Ø103 over the claw bars, 54 mm long, under a 229 mm rotor.
- 10 claw bars must each meet one 5 mm fin: the rotor has to park within about ±1°, and ten 1 mm gaps must hold at 12,550 rpm.
- Ten cords, ten followers, ten TPU links to route and tension.
- The weight sits 48 mm out on the wrist: arm 2's j5 servo runs at 1.65 × rated in the shovel pose, and the hands peak at 0.92-0.93 of their thrust ceiling.
- What the 10 servos buy, independent finger joints, was not used by any sim so far: every grasp closed all fingers together.

## Options
| | Idea | Actuators | Est. hand mass | Body width | Finger control | Main risk |
|---|---|---|---|---|---|---|
| A | Current: one stationary servo per channel, claw bars push followers | 10 | 495 g | Ø94-103 | every base joint + middle/tip pair on its own | weight, parts, ±1° parking |
| **B** | **One pull ring.** Each finger has one cord over all three knuckles; the four cords end on one ring that slides on the hub. A stationary yoke pulls the ring down. The ring can tilt, so the four fingers share the pull and each wraps until it meets the object | 1 (+1 for spread, or none) | **about 320 g** | about Ø60 (motor Ø34 + yoke) | all fingers together, adaptive | grip on thick / thin objects is weaker (bench below) |
| C | Two rings: B plus a second ring for the base joints, so "reach" and "curl" are separate | 2-3 | about 350 g | about Ø65 | base and curl separately, all fingers together | two concentric sliding rings in 30 mm of height |
| D | Motor-wound: a small latch locks the hub to the housing, the flight motor then turns a screw that pulls the ring. The screw holds the grip with no power | 1 small latch | about 270 g | about Ø50 | all together | a dog clutch that carries full rotor torque at 12,550 rpm; needs a sensored motor controller, not the planned ESC |
| E | Servos on the rotor (spin with the hub) | 4-10 | n/a | small | independent | 3,500 g at r 20 mm: a 13 g servo weighs 45 kg. Rejected |
| F | Give up fingers = blades: a normal folding prop with a small separate gripper beside it | 1-3 | about 280 g | prop hub + gripper | independent of flight | no longer the hand-prop; the gripper sits in the downwash |

Not listed: a pull rod through the motor shaft (the SE 2816 shaft is solid; it would need a hollow-shaft motor).

## Recommendation: B, the pull ring
- **One STS3215** (the arm servo: 2.94 N m stall, 0.98 N m rated, 55 g, already on the 12 V arm bus) on a 6 mm spool pulls the yoke: about 294 N at stall, 98 N sustained at the ring, i.e. 73 / 24.5 N per finger (rev D: 76 / 25.5 N on the middle + tip cord). Ring travel about 22 mm (6 + 4.5 + 3.5 mm levers through 90 / 90 / 85°).
- **No precise parking.** The ring and yoke are round, so they meet at any rotor angle. The rotor still has to stop roughly (±5°) so the grip axis points the right way.
- **One flight gap instead of ten:** the yoke sits 1 mm clear of the ring's flange.
- Extensor springs stage the fingers: base first, tip last (pretension about 0, 0.012, 0.024 N m at K1, K2, K3).
- Spread: either a second small servo (STS3032, 20 g) on its own ring, or the first 3 mm of the stroke swings the fingers into pairs before the curl starts (spring staged). The second is lighter but couples spread to grip; start with the separate servo.
- Mass estimate: fingers + hub plates 77, motor 98.5, heel 5, ring + cage 20, yoke + guide 15, STS3215 55, spread servo 20, mount + wiring 30 = **about 320 g (-175 g per hand, -0.35 kg on the robot)**.
- What is lost: individual finger and joint moves (pointing, pinching with two fingers, pressing a button with one). The hand becomes a gripper that also flies.

**Grasp bench for B** (`hub_b_bench.py`, same contact model and test as `hand3_mj.py`; load at which the bar moves 3 mm or 5°; rev D in brackets):
| Bar | Pull off the heel | Along the bar | Lever (cantilever) | Twist |
|---|---|---|---|---|
| Ø20 | 7.3 N (11.1) | 10.6 N (14.3) | 0.19 N m (0.27) | 0.09 N m (0.14) |
| Ø28 (shovel shaft) | 12.9 N (15.5) | 11.3 N (9.9) | 0.27 N m (0.28) | 0.13 N m (0.17) |
| Ø38 | 6.7 N (15.4) | 4.8 N (12.0) | 0.17 N m (0.36) | 0.19 N m (0.20) |

On the shovel-size handle B matches rev D. On thinner and thicker bars it holds about half as well: with one cord the base joint takes its share of the tension whether or not it is the joint that needs it. Option C (separate base ring) is the fix if that matters. The servo stays under its rated force in every test.

## If B is built, what changes downstream
- Flight: 12.16 kg robot instead of 12.51 kg; the hands' hover share drops with it. Rerun `hand3_fly.py`.
- The wrist load in the shovel pose drops by about a third; rerun hand work mode.
- Drum v2, the claw bars, followers and the 10-servo wiring go away; the metal root grips, hub plates and phalanges stay as rev D.
- New parts to design: the sliding ring on the cage, the yoke and its guide, the STS3215 mount beside the motor, the cord routing over three knuckles.
- Open questions to settle in CAD: 22 mm of ring travel in the 30 mm between the motor base and the hub; the yoke's side load on its guide; whether one STS3215 beside a Ø34 motor unbalances the wrist (it is stationary, so only the COM moves).

## Decision and result: hub C built (2026-11-14)
Forest chose **C** (two rings). It is designed, checked and simulated: `claude/code/hand3_hubc.py` (parameters, sizing, solids, checks, masses → `hand3_hubc.json`, STLs). The rev D rotor, fingers, hub plates, heel and motor are unchanged. Estimates; no hardware.

**What it is**
- Two rings slide on the cage under the hub and turn with the rotor. **Base ring (outer):** one cord per finger, past the lag pin at a 2.5 mm offset and over K1 (6 mm): the first pull swings the fingers into pairs, the rest folds K1. **Curl ring (inner):** one cord per finger over K2 (4.0 mm) and K3 (3.0 mm).
- No separate spread servo: spread is the first part of the base ring's stroke (weak lag spring 0.006 N m, K1 extensor pretension 0.02 N m, K3 0.02 N m).
- Each ring is pulled down by a stationary yoke (two lips 1 mm above the ring's flange in flight) on two Ø3 guide rods, by one **STS3215** on a double-ended Ø8 spool: 382 N stall / 128 N rated at the ring, 95.5 / 31.9 N per finger (rev D: 91.7 / 30.6 N base, 76.4 / 25.5 N curl). Strokes 13.5 mm (base) and 12.7 mm (curl), about 0.7 s each.
- The rings are rigid; every cord has its own TPU link (2 mm stretch at peak) and needs a screw anchor to trim its length: 0.3 mm of length error is about 8 N of tension.
- The two servos lie stacked under the motor: a 49 × 48 mm block, 49 mm tall. Rings and yokes fit inside Ø72; four small rod tabs reach Ø82. (Rev D: Ø94 drum, Ø103 over the claw bars.)
- **Mass 374 g, COM 54.5 mm from the spigot** (rev D 495 g, 48 mm): servos 110, motor 98.5, fingers 35, hub plates + lag tubes 34, rings + cage 22, frame 23, yokes 7, rods / spools / screws 14, collar 12, wiring 8, cords and links 6, heel 5.
- Checks (solids): no overlaps in flight, at full base stroke, full curl stroke and in the grasp pose; smallest rotating-to-stationary gap through a full turn 0.5 mm (curl ring to its yoke lip; rev D's smallest was also 0.5 mm). **The tight spot is the axial stack: 1.4 mm between the curl ring's flange at full stroke and the servo block.** Making the hand 4 mm longer removes it.

**Grasp bench, hub C** (`hand3_mj.py`, bar moves 3 mm or 5°; rev D in brackets)
| Bar | Pull off the heel | Along the bar | Lever | Twist |
|---|---|---|---|---|
| Ø20 | 9.5 N (11.1) | 12.5 N (14.3) | 0.22 N m (0.27) | 0.11 N m (0.14) |
| Ø28 | 13.5 N (15.5) | 13.0 N (9.9) | 0.29 N m (0.28) | 0.16 N m (0.17) |
| Ø38 | 6.3 N (15.4) | 4.5 N (12.0) | 0.14 N m (0.36) | 0.24 N m (0.20) |

**Correction to the table above: C does not fix thick handles.** The two rings let the base close before the curl, which helps on Ø20-28, but Ø38 stays weak. The cause is the spring staging, not the ring count: with the tip spring set to curl first ("hook"), Ø38 reaches 11.4 N / 8.2 N / 0.26 N m but Ø28 drops to 9.3 N. The staging is fixed in hardware, so the hand is tuned for 20-30 mm handles.

**On the robot (12.27 kg, was 12.51 kg)**
- Shovel while flying, contact grasp: no drop in 13 s / three dig cycles, shaft moves ≤ 9.3 mm and 5.1° in the hand, flight J 6.1, knuckle cords rms / rated 0.73 (base) and 0.66 (curl). Arm 2's j5 servo: 1.39 × rated (rev D 1.65); the lead-current and stall flags are gone. Hand to neighbouring arm ≥ 19 mm (checked with the old Ø110 drum envelope, so conservative).
- Day mission: flies at all three thrust levels. Hand thrust / ceiling in the flight legs 0.78-0.84 (low), 0.73-0.78 (mid), 0.67-0.72 (high) (rev D: 0.87-0.92 low); hover 0.64 / 0.59 / 0.55. The hands touch their ceiling for 0.30 / 0.13 / 0.03 s at lift-off and touchdown. New flags against rev D: lead current 1.13-1.34 × rating on arm 0 / arm 4 for an instant, clutch slip 1.05 on arm 6 j2 at touchdown.

**Open for hub C:** ring keys and cord anchors are not modelled in detail; yoke side load on the Ø3 rods; the arm collar under the servo block (12 g allowed); rotor parking sensor; drum v2, the claw bars, followers and the 10-servo kit (`hand3-drum-mount.md`, `hand3-build-kit.md`) are superseded if hub C stands; metal STLs unchanged from rev D.
