# Organic landing (v7): one continuous movement (2026-11-11)

Ask (Forest): "make the landing more organic, using multiple legs and joints at once for one continuous movement".
Code: `sim4.py` (`LAND7` block, `land_t`, landing branches in `arm_target`, `thrust`, `winch`, `mode`), `cmp_landing.py`, `landing_video.py` → `cephalo1_landing_v6_vs_v7.mp4`. The mission video `cephalo1_rev4_mission.mp4` now ends with this landing (33 → 25 s of mission time).

## What changed
The old (v6) landing was a list of separate steps: descend, stop, leg 0 down, leg 4 down, drop 6 cm, touch, then the three leg pairs one after another, each waiting for the one before. 13.1 s, never more than 2 legs moving.

The v7 landing is one movement (`CEPH_LANDING=v7`, default):
1. **One descent curve.** The body sinks from the hover to the floor on a single minimum-jerk curve (0.47 m in 2.75 s). No hover stop above the floor.
2. **Legs 0 + 4 reach down together during the descent**, starting 0.3 s after the body starts to sink and taking 2.2 s, so the feet arrive 0.25 s before the body does. The leg path is the take-off unfurl run backwards: the tip joints move first and the motion travels to the root, with the sideways spiral curl of j3/j5.
3. **Touch, then a wave.** As the two feet take load, the three leg pairs follow 0.4 s apart, rotating around the body: 3+7 → 2+6 → 1+5 (the take-off wave reversed), each a 1.55 s swing at the airborne limits (180°/s, 1200°/s²). Up to 6 legs and 26 joints move at once.
4. **Smooth hand-off of the weight.** Each claw-prop fades out by its claw angle as its arm curls down. The claw-props still up carry the weight minus the EDF minus about 10 N per planted foot, so the feet load up gradually. Flight gains blend to ground gains over 0.4 s after touch (no switch). The collective is rate-limited to 40 N/s.
5. **Winch stays firm** all the way down and goes tight once all feet are planted (v6 went soft in the air, which is where its 6° clutch slip came from).
6. **Thrust-aware fallback** (same rule as the take-off): v7 runs only if 6 claw-props + EDF carry the robot at ≤ 85 % of the measured claw-prop maximum (≥ 11.25 N per claw-prop). Below that the v6 landing runs unchanged.

## Two controller fixes this needed
- **Descent feed-forward.** The altitude loop (4 /s² position, 3.5 /s velocity gain) lags a moving reference by about 0.9 s, so the body was still 12 cm up when the plan expected it on the floor, and it then dropped at 0.36 m/s when the pair fans faded. The descent reference now feeds its velocity and acceleration forward; tracking error during the descent is about 2 cm.
- **Where the reference ends.** With legs 0 + 4 hanging unloaded the feet meet the floor with the plate at about 0.31 m, not 0.30 m. A reference ending below that drove the feet into the floor at 0.15 m/s and the robot bounced 8 cm on two springy legs with the fans near hover thrust. The reference now ends at 0.33 m (`Z_SINK7` = 0.02 m below the hover datum of stand + 0.05 m); the body settles to 0.308 m at under 1 cm/s and the feet load up as the fans fade.

## Results (full-robot MuJoCo, mid thrust 12.6 N)
| Landing | v6 | v7 organic |
|---|---|---|
| Leave hover → last foot down | 13.1 s | **5.1 s** |
| ... including EDF spool-down and winch tight | 15.3 s | 7.3 s |
| Time with no leg moving (the body is still sinking in v7) | 4.2 s | 1.0 s |
| Body hovering still before touch-down | 1.8 s | 0.1 s |
| Legs moving at once, mean / max | 1.6 / 2 | **3.0 / 6** |
| Joints moving at once (> 10°/s), mean / max | 4.3 / 6 | **10.3 / 26** |
| Peak commanded joint speed / accel | 110°/s / 793°/s² | 160 / 1132 |
| Sink rate at touch-down | 0.04 m/s | 0.01 m/s |
| Peak body vertical accel | 0.91 m/s² | 1.06 |
| RMS body vertical jerk | 2.2 m/s³ | 3.9 |
| Max body tilt (left over from the hits, at the start of the descent) | 2.7° | 2.8° |
| Worst servo load | 0.41 × stall | 0.74 (stance leg 4, j4, as the last pair comes down) |
| Max clutch slip | 6.2° | **0.2°** |
| Peak claw-prop thrust | 11.6 N | 12.6 N (the last pair, 1+5, saturates briefly while holding its share) |
| Final plate height / tilt | 0.298 m / 0.0° | 0.298 m / 0.0° |

Clearances over the whole mission are unchanged: ≥ 82 mm between arms, ≥ 22 mm to the body, ≥ 88 mm above the floor.

Other thrust cases: 15.1 N (optimistic) and 11.5 N both land the same way (worst servo 0.77 and 0.74 × stall, slip ≤ 0.3°). At the pessimistic 10.1 N the fallback v6 landing runs and is as marginal as before (8.1° tilt, 30° clutch slip at touch-down).

## Tried and rejected
- Starting the pair wave 0.25 s before touch-down (no gap at all between legs 0 + 4 arriving and the wave): 0.2 s shorter, but the body's peak vertical acceleration doubles (1.97 m/s²). Kept the wave starting at touch.
- Legs 0 + 4 at the 1.55 s swing with a 0.9 s wait first: the legs finished early and waited. The slower 2.2 s reach spans the descent instead.
- Descent reference ending 6 cm or 3.5 cm below the datum: 8 cm and 2 cm bounces on the first two feet.

## Open
- The pessimistic-thrust landing is not improved by this; it still needs ≥ 11.25 N per claw-prop to use v7.
- Touch-down is timed, not sensed. On the real robot the hand-off should be triggered by foot contact (servo load on legs 0 + 4, or the tip IMU), with the timed plan as the fallback.
- Two-leg stance for about 1.5 s after touch still relies on the claw-props for balance.

## Knobs (environment variables)
`CEPH_LANDING` v7 / v6; `CEPH_LSW04` (2.2 s, reach time of legs 0 + 4); `CEPH_LLEAD` (0.3 s); `CEPH_LGAP` (0 s, wave start after touch); `CEPH_LSW` (pair swing, default the take-off's 1.55 s); `CEPH_LSINK` (0.02 m); `CEPH_LFOOT` (10 N per planted foot).
