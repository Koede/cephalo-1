# Cephalo-1 joint concepts (brainstorm, 2026-11-07; round 2 2026-11-08)

Goal: fewest parts, most DOF per joint, and no bulk that limits range of motion. The old ball wrist (cup under a 90° drum) only let the fan pod lean about ±30° around pointing down.

Design rule: keep actuators and housings on the proximal side of each joint, so the distal side is a thin stalk and the stalk is the only thing that can collide.
Build rule (2026-11-08): printable on FDM plus hardware-store/hobby parts only (see environment-and-conventions.md).

| # | Concept | DOF | Range | Parts / catch |
|---|---|---|---|---|
| 1 | Swashplateless tip rotor (Paulos & Yim): passive skewed hinges in the prop hub; motor torque modulated once per rev gives cyclic pitch | thrust + roll + pitch moment from 1 motor | n/a | Deletes the tilt servo. Needs rotor-angle sensing and a custom ESC loop |
| 2 | Free wrist + #1: unpowered ball or gimbal at the tip; the rotor's own moments point the pod | 2–3 passive | Limited only by the stalk vs. the seat | No wrist actuators. Pod needs its own IMU and attitude loop |
| 3 | Magnetic ball seat: shallow seat, no wrapping socket lip | 3 | about ±70–80° (set by the stalk) | Breaks away in a crash and re-seats. Holding moment too low for the root |
| 4 | **Ball Drive**: omni-wheel ball wrist (inverted ballbot), 3 micro gearmotors with omni wheels behind the ball's equator | 3, unlimited roll | tilt about 115° any way (estimate) | **As drawn it has no retention: the wheels push the ball outward and the fan's own pull (5.5 N hover) lifts it off.** See round 2 |
| 5 | ABENICS cross-spherical gear | 3 | very wide; drives on one side | Printed in PLA by hobbyists; backlash; 2 drive modules (the paper used 4 motors). Root-joint candidate, too heavy for tips |
| 6 | Gear-coupled double hinge (folding-phone knuckle) | 1 per pair | about 180° | Knuckle wraps the bend, no protruding housing |
| 7 | Helical tendon routing in the spine | adds twist | as the spine | No new parts |
| 8 | Selective-lock joints: one tendon set plus thin per-joint brakes | many per motor | as each joint | Joints move in sequence |
| 9 | Bistable snap joints (binary manipulator) | 2 states each | discrete | Holds a pose with no power |
| 10 | ~~Omni-Wrist~~ (dropped 2026-11-08): Ross-Hime-style parallel spherical linkage | 2, no roll | tilt 90° any way | No gears needed, but 12 pivots whose axes must all meet at one point; printed links + bolts give compounding slop. Not practical with printer + hardware-store parts |

## Round 1 (2026-11-07): Ball Drive vs. Omni-Wrist on the 9-joint arm
Files: `cephalo1_wrists_walk_to_flight.mp4`, `cephalo1_wrists_arm_motion.mp4`. Flight pose j0 −20°, j2 −20°, wrists turn the fans up.

| Motion | Ball Drive (115°) | Omni-Wrist (90°) |
|---|---|---|
| Walk, take-off, hover (wide pose) | never at limit | never at limit |
| Arms waving: worst fan off vertical / share of fan-frames off | 29° / 22% | 54° / 32% |
| Arms swept from curled up to splayed down | 50° / 39% | 75° / 50% |
| Single-arm range sweep | 115° plus roll | 90°, no roll |

## Round 2 (2026-11-08): robustness + range, six buildable wrists
Files: `cephalo1_wrists2_hit_test.mp4` (side hit at 1.5 s, upward yank at 6.5 s; responses scripted from estimated stiffness/retention, not simulated) and `cephalo1_wrists2_range.mp4` (same commands as round 1). Code: `claude/code/wrists2.py`, `make_grid.py`.

Key geometric fact: any socket that mechanically captures a ball must wrap past the ball's equator on the pull side, so the stalk's tilt is capped below ~90° there. To keep both range and retention you need a non-contact hold (magnet), pins (serial axes), or compliance.

| Wrist | Parts | Retention (est.) | Range | Waving: worst / % off | Droop sweep | Single-arm sweep |
|---|---|---|---|---|---|---|
| Ball Drive as drawn | 3 omni wheels, 3 N20 gearmotors | ~0 N, ball lifts off under fan pull | 115°, roll ∞ | 29° / 22% | 50° / 38% | 65° / 40% |
| Captive Ball Drive | + 3 MR63 rollers on M3 bolts at 72° | >100 N | 55° at a roller, 100° between | 74° / 36% | 86° / 54% | 124° / 56% |
| Magnetic Ball Drive | 50 mm hollow steel ball, N52 Ø20×10 magnet, badge-reel tether | breaks away ~20 N (must be measured), reels back | 115°, roll ∞ | 29° / 22% | 50° / 38% | 65° / 40% |
| Roll + pitch yoke | 2 × STS3032, 2 × MR85, M3 bolts | pinned, >300 N; servo slips at ~0.4 N·m | 135°, roll ±180° | 30° / 19% (lag near the singularity) | 30° / 30% | 178° / 54% (must unwind the roll) |
| Geared knuckle + roll | 2 × STS3032, printed m1 spur gears, M3 pins | pinned, >300 N; gear teeth weakest | 170°, roll ±180° | 30° / 9% | 15° / 1% | 171° / 41% (unwinding) |
| TPU flex wrist | TPU 95A spine, 4 Dyneema tendons, 2 × STS3032 | bends and springs back | 75°, no roll | 69° / 37% | 90° / 56% | 105° / 58% |

Takeaways: the geared knuckle + roll has the best range and is pinned (hit-proof), at the cost of a roll singularity near straight-ahead and unwinding when asked to circle continuously (a slip ring would remove the unwinding). The magnetic Ball Drive keeps the Ball Drive's fluid range and turns hits into a safe break-away, if the pull force measures above ~11 N (full fan thrust). The captive ball and the flex wrist lose too much range.

Not checked yet: pod-to-arm/pod-to-pod collisions, gear tooth and pin stress, magnet pull on a thin-wall ball, servo torque during hits. Next: real dynamics of the hit in MuJoCo.
