# Cephalo-1: tentacle steering, climb/descend and grabbing in flight (2026-11-10)

Asks (Forest): turn some tentacles to fly north (for example), and fly up and down that way too; "use arms with flight as grabbing". Code: `claude/code/steer4.py` (tentacle steering vs body-tilt flight), `flymission4.py` (full mission: steer, climb, descend, grab, carry), `flymission_video.py`. Video: `cephalo1_organic_flight_grab.mp4` (1920×1080, rendered on Saturn: `C:\Users\admin\workspace\octodrone\`). Builds on `organic-midair-correction.md` (tentacle vectoring, tip-first bend, null-space breathing).

## Tentacle steering (`steer4.py`, mode `steer`)
- **Which arms lean:** to push the robot in a direction, the arms on the opposite (trailing) side lean their claw-props toward it. Arm weight wᵢ = max(0, −cos(arm azimuth − push azimuth))^1.5 + 0.05, so the arm straight behind leans most, its neighbours less, and the leading arms almost not at all.
  - To fly north, arms 4-6 (south side) lean north.
  - To brake, the north arms lean south.
  - To fly east, the west arms lean east.
- **Allocation:** sin(lean)ᵢ = wᵢ / Σ(fⱼwⱼ) · F_h, plus a small tangential term for heading hold. Lean is capped at 40° (25° while carrying).
- **Arm shape:** the lean is reached by one damped Gauss-Newton step per 10 ms on the claw-axis Jacobian of the current arm shape (large angles, not the small-angle map of flight4). It is spread over j2..j7, tip-heavy, with tip-first joint lags (0.06/0.12/0.20 s). Breathing runs in the null space.
- **Body and fans:** the body stays level, and roll/pitch remain on differential claw-prop thrust. Vectoring claw-props are faded out only beyond 42-48° from vertical.
- **Climb / descend:** collective with reference feed-forward. The arms also reshape in the null space of the claw-axis Jacobian, so no thrust axis tilts: hubs pulled 5 cm in while climbing (streamlined), flared 5 cm out while descending (umbrella).
- **Drag model:** 0.5 ρ CdA |v| v with CdA = 0.10 m² (est.), plus a feed-forward term.

**Path test** (north 2.5 m → east 2.5 m → home 3.5 m SW, min-jerk legs of 3-3.5 s, mid thrust):
| | Body tilt (quadcopter style, ≤ 15°) | Tentacle steering |
|---|---|---|
| Max path error / rms | 23.7 cm / 11.6 cm | **9.5 cm / 4.3 cm** |
| Max body tilt | 14.6° | **0.8°** |
| Max claw-prop lean from vertical | 15.1° | 42.3° (trailing arms) |
| Peak speed | 2.3 m/s | 2.1 m/s |
| Worst servo / rms | 0.11 × stall / 0.31 × rated | 0.41 × stall / 0.49 × rated |
| Peak claw-prop thrust | 9.5 N | 10.1 N |

## Grabbing in flight (`flymission4.py`)
Route: north 2.5 m → climb 0.8 m → east 2.5 m → descend onto the object → grab with arm 2 → lift → fly home with it (5.5 s) → descend. Object: 0.30 kg ball, Ø70 mm (est.), on a stand (centre z 0.40 m).

**Grab sequence (arm 2):**
1. EDF ramps to 100 %.
2. Arm 2's claw-prop spins down (0.4 s), and the breathing wave stops so the arms hold still.
3. Arm 2 swings into a reach pose (IK: grasp point 0.30 m out, 0.30 m below the plate, claw pointing down) and opens its fingers to 20°.
4. Fine alignment: the tip sensor measures the finger-to-object offset three times (before the 7 cm lowering, after it, and just before closing). Each time the body reference is shifted once, blended over 0.15 s. A continuous alignment loop was tried and oscillated against the slow position loop.
5. The fingers close (20 → 72° in 0.35 s).
6. Lift, and curl the arm to the carry pose (0.26 m out, 0.22 m below the plate; every joint ≥ 7.9° from its limit; the first try at 0.22/−0.17 put four joints on their limits).
7. Gentle flight home (lean ≤ 25°, acceleration ≤ 1.2 m/s²).

**Balance while one claw-prop is off:** the seven others need ≥ 10.05 N each in the best static split. Moving the hubs of the arms beside the gap out and the opposite arm in (null space, ±5 cm·cos Δφ) lowers that to ~9.5 N.

**Grasp model (honest label):** the grasp counts as made if the finger centre comes within 3 cm of the ball centre while the fingers close. The open tip radius is 71 mm against the ball's 35 mm, so the closing fingers would centre it. The ball is then tied to the finger centre by a 4000 N/m, 60 N·s/m spring-damper, ramped in over 0.15 s. Finger contact is not simulated; the ball's weight and swing do load the arm and the controller.

| Mid thrust (12.6 N) | Path error | Body tilt | Worst servo | Peak claw-prop |
|---|---|---|---|---|
| Fly north / east (south / west arms lean, 41°) | 10.7 cm | 0.6° | 0.43 × stall | 9.9 N |
| Climb 0.8 m (tuck) / descend (flare) | 3.8 / 3.9 cm | 0.4° | 0.23 | 10.2 N |
| Reach down (arm 2 prop off) | 3.2 cm | 0.8° | 0.54 (arm 2 j2) | 11.0 N |
| Fingers close | finger offset 1.4 cm, **grasped** | 3.1° | 0.40 | 11.6 N |
| Lift (payload) | 14.4 cm (lags in z) | 3.2° | 0.35 | 11.5 N |
| Fly home carrying (7 claw-props, 26° lean) | 9.9 cm | 3.1° | 0.43 | 11.7 N |
| Whole run | rms servo 0.77 × rated | | | 12.2 N peak |

- **Clearances:** ≥ 81.4 mm between arms, ≥ 20.6 mm to the body (arm 2 in the carry pose), ≥ 0.45 m above the floor.
- **Carry tilt:** the 3° body tilt while carrying comes from the ball hanging off-centre. The attitude loop has no integrator; adding one, or moving the carry pose closer to the centre, would remove it.
- **Thrust sensitivity:**
  - 11.0 N: the grab works (finger offset 0.5 cm), but the robot cannot climb or carry home on seven claw-props; it sags ~80 cm and arm 2 j4 hits stall momentarily.
  - 10.1 N (pessimistic): no crash (body tilt ≤ 8°, after adding the balance reshape), but the robot sinks ~40 cm at the reach and misses the ball.
  - **Grabbing and carrying 0.3 kg needs claw-props at about the mid estimate (≥ 12 N).** This raises the bench-test target from 11 N.

## Open items
- Bench-test claw-prop thrust (now ≥ 12 N for in-flight grabbing), spin-down/up time, and whether a stopped claw-prop can grip.
- Finger contact model (three blades closing on the object) and grip-force limits of the fold servo + slider.
- Attitude integrator for off-centre payloads; carry pose nearer the centre (needs a different reach arm path or the tip roll).
- Run the steering controller in the full mission (`sim4.py`) after take-off.
