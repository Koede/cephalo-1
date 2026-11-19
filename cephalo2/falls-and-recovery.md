# Heavy falls and recovery (2026-11-11, reinforced the same day; corrected 2026-11-12; pivot take-off 2026-11-13)

Code: `claude/code/fall4.py` (scenarios, reflexes, recovery, both hardware sets), `rideover_parts.py` (new clutch cams and fall parts, CAD), `fall_summary.py` (batch table), `fall_checks.py` (clearances), `fall_video.py`. Videos: `cephalo1_pivot_takeoff.mp4` (the pivot take-off, current default), `cephalo1_kickstand_takeoff.mp4` (wobble style), `cephalo1_falls_reinforced.mp4` (reinforced robot, four falls, real time), `cephalo1_falls_and_recovery.mp4` (first version, rev 4 clutches). STLs: `stl_rideover/` (zip `cephalo1_rideover_parts_stl.zip`). Builds on body4 (`body-controller.md`) and the flip (`flip-and-inverted-walk.md`). Robot as for the flip: XL claw-props, 1 pack, STS3250 at j1. Values marked (est.) are estimates.

## The new claw-props on this bench (2026-11-13)
The fall bench now uses the XL claw-prop as designed in `xl-claw.md` (`xlpod4.py`; restore with `xlpod_patch.py`):
- **Pod mass 156 g** (itemised, T-Motor F90 2806.5) instead of the 190 g budget: the robot on this bench is 10.48 kg (was 10.75 kg, no senses). Thrust model unchanged (26.5 N at 21.3 V, est.).
- **Toes from the real finger:** root pin 16.25 mm off the axis, finger 79 mm from the pin. At the 60° stand fold the fingertip is 56 mm out and 70 mm beyond the hub plane. The old model was a 95 mm stick from the hub centre (48 mm out, 82 mm beyond): 12 mm too long.
- **Videos draw the claw from its CAD** (`xlclaw_cad.py`): motor body, hub, palm cap, skirt, bearing ring, two-phalanx fingers, steel tape. Fingers: root fold only (2° in flight, 60° as feet).
- **Every fall was re-run** (4 named + 15 tumbles). The lighter pods change how the tumbles land, and one of them (36 N·m at 0°, the "hit" case) now ends with two arms caught under their neighbours. That exposed a gap, and three things were added:
  - **The arms uncurl tip-first** when they rise (j6-7, then j4-5, then j1-3; 2.2 s instead of 1.5 s), so a coiled arm unwinds instead of sweeping its coil into the floor.
  - **Pre-flight check:** before the props start, every flying arm must be within 20° of its pose (servo readback). A caught arm lifts its root to 45° and swings its root yaw ±25° for up to 5 s to shake loose.
  - **Honest endings:** a pivot roll that gets more than 60° off its reference cuts the motors (the fall reflexes take over and it starts again), and "recovered" is only declared when the robot is upright and still. Before, a tumbling robot could be counted as recovered.
- **Results with the new pods:** all 19 falls recover. Of the 10 dome-down landings, **8 take off on the first try** (recovery gear 0.70-0.80, 16.7-19.5 s from the fault, attitude error ≤ 18°, five props at their ceiling for ≤ 0.7 s). Two need a second try: one clutch skipped a notch as the props came on (25 s), and the hit case still has one arm caught after the shake, loses the roll, drops from 0.5 m (gear 1.08) and takes off cleanly the second time (44 s). In that fall an arm passes 143 mm through the body in the sim (arm-body collision is not modelled), so the tangle itself is not trustworthy either way.
- Link-stop peaks in the tumble set with the new pods: j0 152 N·m, j1 131 N·m, j2-j7 84 N·m (190 g pods: 224 / 93 / 76). The design range for the cradle stays 80-220 N·m.
- Numbers in the sections below are from this re-run unless marked "190 g pods". The rev 4 comparison rows were not re-run.

## Correction (2026-11-12)
The first `cephalo1_falls_reinforced.mp4` was drawn wrong: the robot clipped through the floor and never looked recovered. Three faults were found and fixed; every number below is from the re-run.
1. **Renderer joint layout (the big one).** `states4.py` builds its own model. `fall_video.py` did not tell it about the new j1 clutch, so every joint angle past j1 was read from the wrong slot and the arms were drawn up to 0.4 m below the floor. The simulation itself was not affected. `fall_video.py` and `fall_checks.py` now assert that their layout matches the run's.
2. **Claw size.** The sim used the 6 in claw geometry (toes at 76 mm) with XL thrust and mass, while the video drew XL fingers (95 mm), 17 mm into the floor when standing. `fall4.py` now simulates the XL geometry (`FALL_XLGEOM=1`).
3. **Link contact size.** The contact capsules (17-22 mm) were slimmer than the drawn servo + clutch cam (31.5 mm), so cams sank up to 12 mm. Link contact radius is now 31.5 mm (`FALL_LINKR`).
Drawn geometry now stays within 7 mm of the floor (soft TPU contacts at impact). With the larger contact shapes, 4 of 15 tumbles at first did not recover within 40 s (3 looped in get-up → notch skip → re-index). Fix: after a notch-watchdog trip the next get-up holds the winch tight. All 15 recover again, at the price of higher gear loads in those recoveries.

## Result in one paragraph
In a tumbling fall the rev 4 clutches run out of travel, and the link's momentum goes through the printed cam stop straight into the servo gears: 1.7-3.4 × the assumed strip limit at impact (5 tumbles, refined model below). With **ride-over cams** (a V every 90°, 60° at the root, no stop), **a clutch at j1**, **TPU pads on the link stops** and **a Hall switch per clutch**, the worst gear load at impact was 0.56-0.80 (median 0.67) in 15 tumbles (5 blow strengths × 3 directions) and ≤ 0.56 in the three straight drops. All 19 runs recovered. The cost is that clutches skip notches (up to 29 per fall), so the robot re-indexes them after it settles. With the **pivot take-off** (below: circle on the dome, lean onto one leg, roll upright over it) 8 of 10 dome-down landings take off on the first try at ≤ 0.80 (10 of 10 with the earlier 190 g pods); the other two succeed on the second try. The remaining overloads are two belly-down stand-ups (1.00 and 1.16) and the dropped second-try take-off (1.08).

## What changed in the model (applies to both hardware sets)
- **Link-angle stops.** The joint limits (j0 −59/+55° against the mantle, j1 ±30° against the neighbours, j2-j7 ±100° shell to shell) act on the link angle: servo half + clutch + other half, as a MuJoCo fixed tendon with a soft TPU-pad contact (8 ms, est.). Before, each half of the servo pivot had its own limit, which stopped the servo rotor directly: there is no such stop on the real robot (the servo can turn 360°).
- **Clutch-joint armature 5e-5 kg·m²** (sim4: 5e-4, numerical). With a cam clutch there is no geared inertia between the horn and the link. At fast slips the old value coupled up to ~10 N·m of fake torque into the rotor, which made every clutch look worse than it is.
- **Geometry (2026-11-12):** XL claw size for toes, folded fingers and prop discs; link contact radius 31.5 mm (servo + clutch cam envelope).
- **Contacts:** whole-arm floor contact (per-arm collision bits; MuJoCo needs `body_contype`/`body_conaffinity` rebuilt after changing geom bits). Dome = crown sphere + ring of 8. TPU-like soft contacts (est.): EDF-can bumper and dome pad 12 ms, toe caps 10 ms, link shells 6 ms.
- **Gear-load metric:** (servo torque + rotor inertia × joint acceleration, 2 ms average) / assumed 3 × stall. 1.00 = assumed strip limit (unmeasured).
- **Not modelled:** arm-body and arm-arm collision (`fall_checks.py` reports overlap).

With the refined model the rev 4 hardware still overloads in tumbles (1.7-3.4). The "3.2" in the first version of this doc was partly the old model: the pivot-half limits added load, and a soft limit let the links swing 25° past their stops.

## The reinforcement (hardware)
| Change | What | Per arm |
|---|---|---|
| **Ride-over cams** (`rideover_parts.py`) | Same rise as clutch set v1 (10 mm/rad j2-j7, 16 mm/rad root), but the cam is a row of V's: STS3215 joints 4 notches at 90° (working travel ±40°, rounded crest at ±45°), STS3250 joints (j0, j1) 6 notches at 60° (±25°, crest ±30°). Past the crest the roller drops into the next V. The torque into the gears can never exceed the slip torque (soft 1.1 / firm 2.65 / tight 6.6 N·m for STS3215 joints). Same rockers, rollers and bearings as clutch set v1. | cams: 6 × STS3215 (4.3 g), 2 × STS3250 (7.7 g) |
| **Clutch at j1** | Same 6-notch cam as j0, root rocker with its own lever: 0.9 × stall (4.4 N·m at 60 N), roller lever 10 mm, idler lever 32.5 mm, 626ZZ roller, 623ZZ idler. In the sim it lowers the dome-first gear load from 1.31 to 1.05 even with hard stops. | +38 g (clutch hardware, est.) |
| **TPU 95A stop pads** | Pads on every link-to-link stop face (18 × 12 × 5 mm, M3 countersunk into a heat-set insert) and on the root link's stop against the mantle (28 × 18 × 8 mm). The links now end their swing on these pads, not on the cam. | 14 + 2 pads, 27 g |
| **Home-notch sensing** | A 3 × 2 mm N52 magnet pressed into each cam face at the home V. An Adafruit #158 US5881LUA Hall switch in a printed clip on the link sees it only while the roller is in the home V (±15°). US5881 needs 3.5-24 V, so it is powered from 5 V with its open-drain output pulled up to 3.3 V. The 8 switches go to an Adafruit MCP23017 breakout (#5346, I²C 0x20) on the tip XIAO's bus (VL53L4CD at 0x29). | 8 sensors + clips ≈ 8 g, expander + wiring ≈ 7 g (est.) |
| Kept from the first version | TPU bumper ring on the EDF can, dome crown pad, TPU toe caps, **no tilt failsafe** (cut the motors only on a real ESC/motor fault). | |

Mass: about +80 g per arm, **+0.64 kg** for the robot (est.). The j1 clutch hardware is in the simulated mass; pads, sensors and wiring (~0.34 kg) are not.

## Reflexes and recovery (one controller for every case)
1. **Free fall sensed** (IMU < 3 m/s², or 80 ms after a motor cut): every arm whose root is not on top braces (foot 0.30 m below the plate, 0.36 m out). Arm limits are raised to 230°/s and 2500°/s². The winch goes **soft**, so the clutches slip easily.
2. **Impact** (floor contact > 30 N): the brace targets freeze. The robot waits until it settles (0.5 s calm, max 3 s).
3. **Re-index** (new): any clutch whose Hall switch is off has skipped a notch. Its servo drives toward the link's own stop on the side it skipped to. The link reaches its stop (or the floor) first and is held there. The servo carries on, the roller rides back over the crest, and the servo stops as soon as the Hall switch reports home. Up to 25 clutches are re-indexed in 1.5-1.9 s, usually in one round.
4. **Re-seat:** the winch ramps soft → firm over 1.2 s.
5. **Recovery by tilt:**
   - **< 50°: GetUp** (body4) then stand. A **notch watchdog** stops the get-up if a roller skips again (Hall switch off for 0.3 s): the robot settles, re-indexes and starts over **with the winch tight** (4 of 15 tumbles; without the tight winch 3 of them looped forever).
   - **≥ 140° (dome-down): pivot take-off** (2026-11-13; circle on the dome, lean on one leg, roll upright over it; see the section below).
   - **50-140° (on its side): thrust righting.** The arms take the flip-path pose for this tilt (all 8 claw-props above 140°), the props pre-load and lift off, and the robot climbs to 1 m, rolls back upright along the flip path in reverse and hovers. New in this version:
     - firm winch while on the ground, tight once airborne;
     - servo torque limit 0.6 × stall until airborne (Feetech torque-limit register, ramped over 0.3 s);
     - a pinned-arm governor: an arm without thrust whose servos stall is commanded to where it is;
     - a skipped notch on the ground aborts the lift (props wind down over 0.5 s, re-index, retry).

## Results (`python3 fall4.py <case>`, `FALL_HW=rev4` for the old clutches; table from `fall_summary.py`)

**The four named falls:**

| Case | What happens | Hardware | Impact | Body (10 ms) | Gear: first fall + impact | Gear: recovery | Notches re-indexed | Recovered |
|---|---|---|---|---|---|---|---|---|
| **cut** | motors cut at 1.5 m, upright | reinforced | 4.9 m/s | 45 g | 0.55 | 0.56 | 8 | standing 13.8 s after the cut |
| | | rev 4 | 5.0 m/s | 43 g | 0.54 | 0.74 | — | standing |
| **hit** | 36 N·m blow for 0.25 s at 1.5 m, ESC failsafe at 45° | reinforced | 5.5 m/s, lands upside down | 32 g | **0.66** | 1.08 | 28 | pivot take-off on the second try (one arm caught, first roll lost), hovering 44.2 s after the blow. 190 g pods: first try, 0.81, 16.7 s |
| | | rev 4 | 5.7 m/s | 28 g | **2.55** | 0.89 | — | hovering |
| **inv** | motors cut at 1.2 m upside down | reinforced | 4.4 m/s, dome first | 32 g | 0.55 | 0.80 | 0 | pivot take-off, hovering 16.7 s after the cut |
| | | rev 4 | 4.5 m/s | 40 g | 0.86 | 0.54 | — | hovering |
| **drift** | motors cut at 1.0 m flying sideways at 2.5 m/s | reinforced | 4.4 m/s | 40 g | 0.56 | 0.50 | 15 | standing 15.2 s after the cut |
| | | rev 4 | 4.6 m/s | 43 g | 0.56 | 0.83 | — | standing |

**Tumble set** (hit case, blow 30/33/36/39/42 N·m × direction 0/45/90°; rev 4 at 45° only; 70 s allowed):

| | Runs | Recovered | Gear at impact: max / median | Gear in recovery: max / median | Runs > 1.0 anywhere |
|---|---|---|---|---|---|
| Reinforced | 15 | 15 (15-44 s) | **0.80 / 0.67** | 1.16 / 0.77 | 3 (two stand-ups at 1.00 and 1.16; the dropped second-try take-off at 1.08). The 9 dome-down landings: pivot take-off, 7 on the first try at ≤ 0.80, 2 on the second |
| Rev 4 (190 g pods, not re-run) | 5 | 5 | **3.36 / 2.48** | 1.09 / 0.94 | 5 |

## Pivot take-off: lean on one leg and roll upright over it (2026-11-13, the default)
Video: `cephalo1_pivot_takeoff.mp4` (the side-blow fall and the upside-down drop). `FALL_STYLE=pivot` (default). This is the move Forest meant: the same circle on the dome, but the leg is a pivot, not a prop. The body leans on it and rolls upright over it, so the separate hop, climb and roll of the wobble style become one movement.

**Sequence** (times for the hit case, from the blow):
1. **Arms rise (+4.7 to +6.3 s)** and **wobble (+6.3 to +9.4 s)** as in the wobble style below. The circle now stops when the low side points at the middle "under" arm of the roll path (the side that goes down in the roll); if it does not get there within one more turn, the roll axis is moved to suit the low arm (not needed in 11 runs).
2. **The leg pops out and the body leans onto it (+9.4 to +10.2 s).** The low-side arm stops its prop and swings out in 0.55 s (j0 −48°, j2 +75°, j4 +10°: its j2 housing on the floor 0.23 m from the body axis). Thrust rises from 60 to 85 % of the weight. The dome leaves the floor: the body hangs between the props and the leg, which carries 27-30 N.
3. **Pivot (+10.2 to +11.2 s).** The roll upright starts (4.0 s minimum-jerk, about the axis through the two arms 90° from the leg). The leg keeps its direction on the floor (j0 follows the roll angle), the body turns about the foot and the far side swings up: about 32° of roll with the leg still loaded (20 → 0 N). The body moves 0.15 m toward the leg and the climb starts 0.4 s into the roll.
4. **Off the leg (+11.2 s).** The leg lets go (0.14 m of climb or 60° of roll, whichever comes first), folds onto the roll path in 0.7 s, and the roll and the climb to 0.9 m finish together (+14.2 s). Hover is confirmed 2.5 s later.

**Results (the same 11 dome-down runs, 190 g pods; with the new pods see the section at the top):**

| | Wobble + one leg | **Pivot (default)** |
|---|---|---|
| First-try take-offs | 11 of 11 | **11 of 11** |
| Worst gear load in recovery | 0.60-0.81 | **0.67-0.81** |
| Time from the fault to hovering | 18.5-22.4 s | **15.2-18.5 s** |
| Props | 7 at lift-off, 23-25 N each | 7 → 5 mid-roll; at their 28 N ceiling for 0.6 s |
| Attitude error | small (level lift-off) | up to 19° mid-roll |
| Clearances (`fall_checks.py`, hit and inv) | arm-body ≥ 4 mm, arm-arm ≥ 16 mm | arm-body ≥ 4 mm, arm-arm ≥ 16 mm (in the roll: 24 / 46 mm) |
| Floor (`pen_chk.py`) | −7 / −1 mm | −7 / −8 mm (arm tips brush the floor mid-roll, ≤ 6 N) |

- **Thin thrust margin:** mid-roll the three "under" arms have their props stopped, so five props carry the robot and climb at the same time. They sit at their ceiling for 0.6 s (full pack). At lower charge this needs the leg to carry more or the roll to wait for height.
- Not tuned yet: the leg carries ~30 N of the ~65 N it can; a lower thrust share in the lean (`FALL_PIVFZ`) would lean harder on it. `FALL_PIVT`, `FALL_PIVLEAN`, `FALL_PIVREL`, `FALL_PIVCLIMB`, `FALL_PIVSHIFT` set the roll time, the lean, the release angle, the climb time and the sideways travel.

## Wobble and kickstand take-off: mode 5 made deliberate (2026-11-12; now `FALL_STYLE=wobble`)
Video: `cephalo1_kickstand_takeoff.mp4` (the side-blow fall and the upside-down drop). Used whenever the robot settles dome-down (tilt ≥ 140°). `FALL_STYLE=wobble`: circle on the dome with the props going, then one leg pops out, props the body level and it hops straight up (superseded as the default by the pivot above). `FALL_STYLE=tripod` is the first, stiffer version (three struts, no circling). `FALL_KICK=0` is the old undirected lift-off.

**Sequence, wobble style** (times for the hit case, from the blow; the robot settled at +3.3 s):
1. **Arms rise (+4.7 to +6.3 s), props off.** Root first: j0 swings each arm up off the floor, the other joints go to the inverted flight pose, then j0 settles. Servos limited to 0.6 × stall.
2. **Wobble (+6.3 to +9.4 s).** All 8 props run at 45 → 60 % of the weight, so the dome stays on the floor. The attitude reference leans 10° from level and that lean direction is swept round at 0.5 turns per second, starting where the body already leans. The body rolls round on its dome like a settling coin: 1.3 turns at 11.5° of lean in 10 of 11 runs (one run stuck after 0.3 turns and still went on).
3. **The leg pops out (+9.4 to +10.3 s).** The arm on the low side stops its prop and swings out in 0.55 s: root link down (j0 ≈ −48°, levelled by the IMU), the rest of the arm out to the side (j2 +75°, j4 +10°), so its j2 housing stands on the floor 0.23 m from the body axis. It pushes the body from 11.5° to 1.5° off level while the thrust share rises from 60 to 100 %. The flying arms get full servo torque from here (the seven props pull up to 23 N each).
4. **Lift-off on 7 props (+10.3 s)**, a hop to about 0.14 m above its rest height with the leg hanging out.
5. **The leg folds up (0.8 s) and its prop joins (by +12.3 s)**, then the climb to 0.9 m, the roll back upright (+13.9 to +17.9 s) and hover.

**Results (11 dome-down runs: hit, inv, and the 9 tumbles that land on the dome):**

| | Old lift-off (modes 3-7) | Wobble + one leg (default) | Tripod (3 struts) |
|---|---|---|---|
| First-try lift-offs | 8 of 11 | **11 of 11** | 11 of 11 |
| Worst gear load in recovery | 0.51-1.42, over 1.0 in 5 runs | **0.60-0.81** | 0.59-0.73 |
| Time from the fault to hovering | 12.7-43.6 s | 18.5-22.4 s | 15.1-19.3 s |
| Motion on the dome before lift-off | uncontrolled rocking, 1-38° swing | a steered circle, 11.5° lean, ~1.3 turns | levelled, 0.7-7.7° swing |
| Props at lift-off | 2-8, changing | 7 (up to 23-25 N each) | 8 (15 N each; 28 N in one run) |
| Clearances (`fall_checks.py`, hit and inv) | an arm 137 mm into the body | arm-body ≥ 4 mm, arm-arm ≥ 16 mm | arm-body ≥ 6 mm, arm-arm ≥ 12 mm |

- The wobble costs about 3 s against the tripod. It is there because it looks alive, and it starts the pop with the body leaning onto the leg that will prop it.
- **What went wrong on the way (wobble):** with one prop stopped the neighbours carry 22+ N, and at the 0.6 × stall ground limit their j2 servos gave way, the prop axes tipped over and the props dropped out one after another. Full servo torque from the pop onward fixed it. The climb also had to wait for the eighth prop (hop first, then climb).
- **Tripod style:** three arms (low side + the two 135° away) fold into struts with their tips pointing up, an IMU loop on the three j0 servos levels the body (within 3.5° in 9 of 11 runs), the other five arms rise, all 8 props spin up (strut tips point up, so their props thrust too) and it lifts off level. Each strut can push ~65 N (j0 only, ~7 cm lever).
- **Why struts and not feet:** an earlier version planted three feet at 0.43 m. Each foot could push only ~10 N (the load runs through seven joints), which was not enough to level the body.
- Not yet tried: settling on the side (50-140°), the real ellipsoid dome (the sim's is a crown sphere plus a ring of 8), stopping the opposite prop during the pop (`FALL_POPPAIR=1`: six balanced props at ≤ 20 N, tried once on inv, works).

## Get-up modes seen (2026-11-12; 19 reinforced runs: 4 named + 15 tumbles)
This table is the survey made before the kickstand take-off existed (`FALL_KICK=0`). None of these was scripted as a separate mode. They are what the one recovery controller did, depending on how the robot came to rest. Times are from the fault to recovered.

| # | Mode | When it happens | What it looks like | Runs | Time | Worst gear in recovery |
|---|---|---|---|---|---|---|
| 1 | **Stand-up** | settles on its belly, tilt < 50° | re-index, arms roll in, feet placed, body rises on 8 feet | 5: cut, drift, tumbles 42 N·m × 3 | 15-18 s | 0.59-0.85 |
| 2 | **Stand-up, second try tight** | as 1, but a clutch skips a notch during the get-up | watchdog stops it, re-index, get-up again with the winch tight | 3: tumbles 39 N·m × 3 | 23-28 s | 1.25-1.32 |
| 3 | **Clean inverted lift-off** | lands flat on the dome (180°) | props pre-load 2.3 s, lifts off upside down without rocking (tilt swing 1°), climbs to 1 m, rolls upright in the air | 1: inv | 12.7 s | 0.62 |
| 4 | **Rock and lift-off** | settles on the dome a little off level (155-179°) | props pulse on, the robot rocks on its dome (swing 14-24°) for 2-3.5 s until the thrust lines up, then lifts off inverted and rolls upright | 4: tumbles 30 N·m/45°, 33 N·m/0° and 45°, 36 N·m/90° | 14-17 s | 0.70-0.71 (0.51 in one) |
| 5 | **Rock, kickstand, take-off** (Forest's favourite) | as 4, when an arm without thrust reaches the floor | rocks on the dome, hops ~10 cm, an arm lands on its links and props the body clear of the floor while the other props keep running, then it climbs away and rolls upright | 2 clean: hit (= tumble 36 N·m/0°) and tumble 33 N·m/90° | 15-17 s | 0.83-1.22 |
| 6 | **Abort and retry** | the lift-off rolls it away on the ground (attitude error > 50° with no lift) | props wind down over 0.5 s, it settles, re-indexes, tries again | 2: tumble 30 N·m/0° (2 tries, the last one a 2.8 s kickstand on several arms), tumble 36 N·m/45° (4 tries) | 27-44 s | 1.12-1.42 |
| 7 | **Prop-flip, then stand-up** | a failed inverted lift-off tips it over onto its belly | the props flip it right side up on the ground, then modes 1-2 | 1: tumble 30 N·m/90° | 32.8 s | 1.22 |

**Mode 5 in detail (hit case, video 2/4, t +4.7 to +10.3 s):**
- +4.7 s: motors restart with the robot on its dome at 176°; servo torque limit 0.6 × stall, winch firm.
- +5.3 to +7.9 s: props come on in changing groups of 2-6 (only arms whose axis is within 40° of vertical thrust). The body rocks between 150° and 178° on the dome.
- +7.9 s: arm 7's links touch as it tips to 162°; the body hops to 10 cm above its rest height.
- +8.6 to +8.7 s: arm 0 (prop off, pointing away) lands on its links and holds the body at 153° tilt, 11 cm up, while 6 props run. The pinned-arm governor is what makes it hold its pose instead of pushing.
- +9.1 s on: thrust is lined up, all 8 props come on, it climbs at 0.8 m/s to 0.94 m, then rolls upright (+10.3 to +14.3 s).
- The cleanest single-arm case is tumble 33 N·m/90°: arm 3 alone props the body for 1.3 s, 9 cm up.
- It is emergent, not designed: the controller was only trying to lift off inverted. It costs gear load (1.22 in the hit case, in the lift-off).

**Not seen in these runs:** settling on the side (50-140° tilt). Every tumble ended either dome-down (≥ 155°) or belly-down (≤ 43°), so the side lift-off pose from the flip path is untested here.

**Done (2026-11-12):** mode 5 is now deliberate (wobble and kickstand take-off, section above) and replaces modes 3-7 for every dome-down landing. Modes 1-2 (belly-down stand-ups) are unchanged.

## What the links and mantle must now carry
The load the gears no longer see goes into the link stops. Peak stop torques in the tumble set (8 ms TPU pads, est.):
- **j0 root against the mantle:** median 58 N·m; three of 15 runs reached 167-224 N·m (the rest ≤ 74 N·m).
- **j1 against the neighbours:** up to 93 N·m.
- **j2-j7 shell to shell:** up to 76 N·m.

The rev 4 hardware sees similar stop loads in the same falls (up to 115 / 52 / 74 N·m): only the path changed.

Design rules for the link cradle CAD (still open):
- **Pivot loads go through the link's own bearing pair, never the servo spline.** With a stop face 40-50 mm from the axis, 80-220 N·m means a 2-5 kN contact force and a similar radial pivot load. Every joint needs a through-axle or an opposing idler bearing sized for it. For example: a pair of 6804 (20 × 32 × 7) at j0 and a pair of 6802 at the distal joints, with the stop faces at ≥ 45 mm radius. Check the bearings' static ratings (C0) against these loads.
- **Stop faces** are flat PETG bearing faces ≥ 200 mm² behind the TPU pad (≈ 10-15 MPa at 2-3 kN).
- Bolt the root stub to the mantle sector through heat-set inserts or nuts, not printed threads.

## Checks
- **Clearances (`fall_checks.py`), new pods:** cut, inv and drift: arm-body ≥ 16 mm, arm-arm ≥ 29 mm in every phase. The hit case fails this check: an arm is up to 148 mm inside the body from the impact until the second take-off (not modelled, see the top section). With the 190 g pods: arm-body ≥ 4 mm and arm-arm ≥ 12 mm in every phase of all four named falls. Before the kickstand, in the hit tumble an arm passed up to 137 mm into the body during the lift-off (arm-body collision is not modelled). On the robot those arms would rest on the dome. The dome then acts as the re-index stop, and the dome shell takes those loads.
- **Floor (`pen_chk.py`):** the lowest drawn point is −1 / −6 / −1 / −3 mm (cut / hit / inv / drift, new pods): soft-contact impact or lying; +3 mm when standing (the drawn fingertip against the toe contact sphere).
- **Body deceleration** rose slightly in straight drops (cut 43 → 45 g, 10 ms average), because the arms now give more. Pack and electronics mounts should be designed for 50-60 g.

## Design variants tried on the way (hit case unless noted)
Rows other than the baseline were run before the 2026-11-12 geometry correction; they rank the variants and are not comparable in absolute value.

| Variant | Worst gear | Note |
|---|---|---|
| Rev 4, old model | 3.20 | stop on each pivot half, numerical clutch armature |
| Rev 4, refined model | 2.55 (inv 0.86) | baseline used above (corrected geometry) |
| + j1 clutch | 3.13 (inv 1.05) | old model |
| Clutch travel ±60° | 3.10 | still a stop |
| Progressive cam (slip torque × 2.5 at the stop) | 3.6 | more torque at the stop, worse |
| TPU end-stop pad on the cam (20° compression) | 3.15 | pad bottoms out |
| Winch firm in the fall (j0 clutch 0.9 × stall) | 3.62 | stiffer clutch, harder stop |
| **Ride-over cam + j1 clutch + link stops** | **0.55-0.80** (fall + impact, 6 cases) | adopted |
| + servo torque limit 0.5 × stall in the fall | ~same | not needed |

## Open
- **Arms caught under their neighbours after a hard tumble** (new pods, 1 of 10 dome-down landings): the shake frees one of two arms; the take-off then fails once. Candidates: arm-body collision in the sim first (the tangle passes through the body), then a relax-and-retry of the brace, or taking off with the caught arm as the pivot leg.
- **Stand-up after a notch skip** is the other overload: 1.00-1.16 in 2 of 15 tumbles with the new pods (1.25-1.32 in 3 of 15 with the 190 g pods). Candidate: a get-up that does not press links into the floor.
- **Pivot take-off:** five props at their ceiling for 0.6 s mid-roll (thin at lower charge); arm tips brush the floor mid-roll; bench the strut (j0 on the j2 housing, TPU pad under it) and the dome rolling on a real floor, port it into body4, try it for side landings and as the normal take-off from the inverted stand. One run (33 N·m/90°) circled only 0.3 turns. Candidates: a ground roll-over with the arms (no props) for tilts < 140°, slower arm motion while pre-loading, or per-arm winch tension (soft on arms touching the floor).
- **Bench tests:**
  - ride-over cam: crest torque, and whether the roller drops cleanly into the next V at speed;
  - Hall switch window on the printed cam;
  - re-index against a real link stop;
  - a drop test of one arm.
- Arm-body collision in the sim; the dome shell as a load path.
- Mass +0.64 kg: re-check thrust margins (XL, 1 pack) and the flip.
- Falls at lower charge (the thrust righting needs ~28 N claw-props), with 2 packs, and from above 1.5 m.
- Port the fall reflexes, re-index and notch watchdog into body4.
