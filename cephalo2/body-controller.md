# Cephalo-1 as one body: the intent-level controller (body4, 2026-11-11)

Ask (Forest): "optimize your ability to control this thing organically and fluidly, make it your body and make sure you can work your body like it's native to you".
Code: `body4.py` (controller), `body_metrics.py` (fluidity/load metrics), `body_video.py` → `cephalo1_body_mid_day.mp4`.

## What was wrong with the old way
Every behaviour so far (take-off, flight benches, steering, chores, landing, lying down) was its own script with its own fixed timeline and its own start state. Moving from one behaviour to the next meant a new simulation, and each phase change was a switch on the clock: "at t = 19.96 s start leg 0". Nothing reacted to what the body actually felt.

## How the body is controlled now
One controller runs the robot continuously. It is driven by intents, the way I would want to use my own body: "get up", "walk 18 cm and turn 30°", "take off to 0.8 m", "fly north 1.5 m and climb 0.3 m", "land", "lie down". The layers, from the top:

1. **Intents.** A queue of skills. Every skill starts from wherever the body actually is: the measured pose, velocity and which feet are down. It ends on a sensed condition, not a clock.
2. **Skills:** get up, idle (breathing), walk (any direction + turn), wave, take off, fly to (x, y, z, heading), land, lie down.
   - Ground skills move the feet through a fast whole-arm IK: an analytic-Jacobian Gauss-Newton solver, 0.3 ms per arm instead of 7 ms, with the same residual as `cephalo4.ik`.
   - The body pose is re-anchored to the measured pose at the start of every skill, so the first IK solution equals the current joint command.
   - Flight uses tentacle steering (`steer4`) with a heading reference added.
3. **Reflexes:**
   - **Touchdown is sensed.** The two reaching feet see the ground coming: a downward ToF range in each tip bay. Below 10 cm, the legs and the body descent are re-timed so the feet arrive together with the body at a gentle press, wherever the ground actually is. The pair wave starts when foot force is felt. If the floor isn't there, the body keeps sinking at 5 cm/s, feeling for it.
   - **Terrain is absorbed.** The touching feet become stance feet where they actually landed. If the ground was higher than expected, the fans lift the body to stand height over them. The other pairs are guided by IK onto the real ground for the last quarter of their path.
   - **Lift-off is sensed.** Air control blends in only once the stance feet carry < 25 N.
   - **Load governor.** Skill phases slow to as little as 0.45 × speed while any servo is above 0.80 × stall or a claw-prop is saturated, and recover at 0.8 /s. Airborne leg unfurls are exempt, because finishing them is what brings the missing claw-props back.
   - **Weight shift.** Before an arm lifts off the ground to wave, the body leans 2.5 cm away from it. The wave is timed to stay inside the servos' torque-speed line (see below).
4. **Spinal layer.** Every joint command passes a critically damped 2nd-order tracker (16 rad/s) with goal-velocity feed-forward. Its limits are 120°/s and 800°/s² for a loaded leg, 180°/s and 1200°/s² in the air. A skill switch can never step or kink a joint command. The raw skill targets jump by up to 6° in one 10 ms tick at switches; the commands the servos receive never exceed the limits.
5. **One thrust law** for ground, transition and flight:
   - A ground collective (claw-props carry what the planned feet should not) blends into the altitude loop, which has quintic velocity and acceleration feed-forward.
   - Attitude gains blend from 60/8 to 14/4.5.
   - Claw-props fade in and out by claw angle and spin-up time.
   - Attitude-first mixing (`sim4.allocate`).
6. **Senses logged every frame:** body pose and velocity, joint angles, servo load per joint, clutch slip, foot force per arm, belly (EDF bumper) force, claw-prop thrusts and saturation, governor rate, ground↔flight blend.

## Two findings that changed the design
- **Legs 0 + 4 move together, always.**
  - Moving the last two legs one after another, as the old pessimistic-thrust fallback did, shifts the centre of mass to one side. On 6 claw-props + EDF at 95 % of their maximum, there is no thrust left to correct it.
  - Re-checking the old `sim4` pessimistic take-off showed it "completed" only after the body sank 20 cm and bumped the EDF can on the floor (plate at 0.097 m) before recovering.
  - Moving legs 0 and 4 together (opposite arms, so their mass shifts cancel) needs no differential thrust. It now works at the pessimistic thrust.
  - The take-off fast branch now runs up to a 97 % six-claw-prop load (was 85 %). Above 97 %, 6 claw-props + EDF cannot hover at all.
- **Thin-margin landing descends on 8 claw-props.** When 6 claw-props + EDF would run above 85 %, the body sinks on all 8 to 15 cm above the expected floor. Only then do legs 0 + 4 reach down, while the last 15 cm are covered slowly (peak deceleration 0.13 m/s²).

## Results (full-robot MuJoCo, one continuous session: lying → get up → breathe → walk 18 cm + turn 30° → wave → take off → fly north 1.5 m + climb 0.3 m → fly east 1.5 m + descend 0.3 m + turn 90° → hover → land → lie down → rest)
| | Mid thrust (12.6 N) | Pessimistic (10.1 N) |
|---|---|---|
| Session | 55.6 s, 14 intents, completes | 57.2 s, completes |
| Worst servo | 0.86 × stall (take-off, arm 4 root) | 0.86 |
| Max clutch slip / body tilt | 0.3° / 1.0° | 0.4° / 1.0° |
| Take-off: worst servo / tilt | 0.86 / 0.6° | 0.86 / 0.6° (old `sim4`: 6.3°, 10° slip, EDF touched the floor) |
| Landing: worst servo / tilt / slip | 0.74 / 0.4° / 0.2° | 0.72 / 0.5° / 0.2° (old `sim4`: 8.1° tilt, 30° slip, failed in some runs) |
| Flight path error (max) | 6.8 cm; heading turn asked +90°, got +91.5° | 7.0 cm |
| Walk asked / done | 18 cm, 30° / 20 cm, 33° | same |
| Joints moving at once (> 10°/s), mean / max | 8.9 / 37 | 8.7 / 37 |
| Command limits held (speed / accel) | 180°/s / ~1200°/s² | same |
| Claw-prop saturated (total time) | 1.1 s | 3.3 s |
| Governor active (total time, min rate) | 1.9 s, 0.75 | 5.1 s, 0.75 |
| Clearances (min): neighbour arms / body / floor | 77 / 17.6 / 17.4 mm | same |

Per skill at mid thrust (worst servo × stall): get up 0.57, walk 0.82, wave 0.76, take off 0.86, fly 0.43-0.46, land 0.74, lie down 0.71 (the old `rest4` lie-down reached 1.0 by dragging feet; the feet now lift straight up first).

**Landing on an unexpected step** (`python3 body4.py mid step`): the robot takes off, flies 1.6 m north and lands on a 12 cm step that the landing plan doesn't know about.
- The feet see it at 10 cm, the touch is gentle, and the body ends at plate height 0.42 m (= step + 0.30).
- Worst servo 0.82 × stall, slip 0.3°, tilt 0.8°.
- With touchdown timed instead of sensed, the same landing bounced 13 cm, and the servos hit 1.0 × stall with 6.6° slip.

Flat-floor landing alone (`short` programme): worst servo 0.72 mid, 0.74 pessimistic.

## Servo torque-speed check (2026-11-11, `servo_envelope.py`)
Ask (Forest): make sure the wave can actually happen with the servos as they are, without changing servos.
- The sim servo has a stall torque limit but no speed limit, so every joint motion is checked against the real datasheet torque-speed line at 12 V: available speed = no-load speed × (1 − load / stall).
  - STS3250 (j0): 0.133 s/60° = 451°/s, 4.9 N m stall.
  - STS3215: 0.222 s/60° = 270°/s, 2.94 N m stall.
- Demand = the larger of the commanded and the actual joint speed.
- **The first wave was not possible.**
  - Lowering the arm from the raised pose in 1.0 s needed 172°/s on arm 1 j4 at 0.47 × stall: 1.20-1.32 × what the servo can do.
  - The 1.1 Hz waving itself reached 0.88 × available.
- **The wave now fits:**
  - raise 1.4 s, lower 1.6 s (were 0.9 / 1.0 s);
  - waving 0.8 Hz (was 1.1 Hz), same amplitudes (j3/j5/j7 ±14/18/22°, j2/j4/j6 ±4/6/8°).
  - Worst point is 0.65 × available (arm 1 j4, 109°/s at 0.38 × stall).
  - Worst servo 0.75 × stall, all arm 1 joints inside their limits (j0 −50…−8°, j1 0°).
  - Clearances during the wave: 81 mm to neighbour arms, 19 mm to the body, 89 mm to the floor.
  - The wave now takes 7.3 s (was 6.2 s).
- **Other motions that are over the line in the same check (mid thrust, not yet fixed):**
  - landing pair wave: 1.33 × (arm 5 j4, 168°/s at 0.53 × stall, as a pair plants);
  - lie down: 1.05 × (arm 3 j0);
  - walk: 1.04 × (arm 3 j2, 130°/s at 0.54 × stall in a swing).
  - Within the line: take-off 0.90, flight 0.82, get up 0.70.
- **Tried and rejected: enforcing the line in the spinal layer** for all joints (`CEPH_ENVLIMIT=1`, off by default). Clipped stance legs lag their IK goals, which raises their loads and tightens the clip further. Lie-down and landing went to 1.0 × stall with 2.6° slip. The over-the-line motions need slower paths of their own, like the wave got.
- Re-running the session with the longer wave also moved the pessimistic landing's timing. Its worst servo was 0.72 × stall before; now it touches 1.00 × stall at arm 0 j0, with 1.3° slip. The landing is sensitive to the state it starts from at that thrust.

## New hardware this needs (real parts)
- **One downward ToF sensor per tip bay:** Adafruit VL53L4CD breakout (#5396, ~1-1300 mm, 18° field of view, up to 100 Hz, I²C, 1.8 g, $14.95). Each tip already has its own XIAO node, so the fixed I²C address 0x29 needs no multiplexer. 8 × 1.8 g = 14 g. It looks down past the folded fingers, through a slot in the tip-bay floor (CAD to do).
- **Foot force** comes from what is already there: servo current/load readback on the STS bus (j4/j6 of a stance leg) plus the tip IMU. In the sim it is the contact force.

## Limits and open items
- The pessimistic-thrust flight works, but the claw-props run near saturation during the 6-claw-prop moments. Measured claw-prop thrust is still the main open item.
- Walking over- or undershoots the asked distance and turn by about 10 %. The body plan is followed open-loop between skills, and re-anchoring only corrects it at the next skill.
- Foot force on the real robot will be noisier than sim contact force. The touch threshold (3 N) and the ToF approach both need bench tuning.
- Not yet in the body: in-flight grabbing and two-hand chores (`chores4`), gust rejection by tentacle vectoring (`flight4`). They are written as separate scripts and are the next skills to port.

## How to drive it
`python3 body4.py mid day` (also `pess`, `CEPH_FMAX=<N>`; programmes `day`, `ground`, `short`, `step` in `PROGRAMS`). A programme is a list of intents, e.g. `[("get_up",), ("walk", 0.18, 0.0, 30.0), ("take_off", 0.8), ("fly", 0, 1.5, 0.3, 0, "north"), ("land",), ("lie_down",)]`. `python3 body_metrics.py body4_mid_day.npz` prints the metrics; `python3 body_video.py chunk <i> <n> body4_mid_day.npz`, then `join <n> body4_mid_day.npz` for the video.
