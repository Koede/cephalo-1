# Octopus Drone: environment and coding conventions (read at the start of every session)

## Setup (run first, every session)
The workspace starts empty each session. Recreate it from the project docs:
1. `mkdir -p /home/claude/octodrone && cd /home/claude/octodrone`
2. Restore code from the project: every `claude/code/*.py` and `claude/code/setup_env.sh` doc → same file name locally. For rev 4 work only `setup_env.sh`, `geom.py`, `render.py`, `cephalo4.py`, `poses4.py`, `sim4.py`, `states4.py`, `viz4.py` are needed.
3. `bash setup_env.sh` installs numpy, scipy, numba, trimesh, manifold3d, shapely, pillow, mujoco and cma (pip `--break-system-packages`), checks for ffmpeg, and prints versions.
4. Rev 4 pipeline: `python3 poses4.py` (writes `poses4.json`: stand/flight poses + static loads) → `python3 sim4.py mid` (full-robot mission, ~1 min; writes `sim4_mid.npz/.json`; `pess` or `CEPH_FMAX=<N>` for thrust sensitivity) → `python3 states4.py sim4_mid.npz` (clearance checks) → `python3 viz4.py still <t>` (check frames) → `python3 viz4.py chunk <i> <n>` per core, then `python3 viz4.py join <n>`.
5. Older studies: `python3 choreo.py` regenerates the rev 3 motion state files (`states_*.npz`) that the older video scripts read.
6. Read `claude/cephalo1-design-state.md` (current = rev 4) and `claude/cephalo1-rev4-integrated.md` before designing; the joint, clutch and dynamics docs are background.

## Code layout (claude/code/)
| File | Role |
|---|---|
| `cephalo4.py` | **Rev 4 single source of truth**: geometry (8 servo-in-link joints at 56 mm pitch), servos/clutches/winch modes, itemised masses, thrust cases and T/W, FK/IK, static joint torques, foot-force split, mantle clearance |
| `poses4.py` | Optimises the stand and flight poses for load and clearance, j0 mantle limits, static load tables → `poses4.json` |
| `sim4.py` | Full-robot MuJoCo model (free body, 8 × 8 stall-limited position servos, clutch DOFs with tension-scaled slip, toes, fan/EDF forces, COM-referenced thrust allocation) and the mission: stand → walk → take-off → hover → 2 hits → landing → stand. Writes 30 fps states (300 fps around hits) + per-phase load summary |
| `states4.py` | qpos → robot state; neighbour / body / floor clearance check over a run |
| `viz4.py` | Rev 4 meshes (servo cradles, clutch cams + rockers, pods with fold/spin), HUD (mode, mass, T/W, EDF, fans, winch, worst servo, clutch slip, tilt, per-arm bars), chunked 1920×1080 video |
| `render.py` | Headless numba z-buffer rasterizer: key/fill/rim light, planar ground shadow, outlines, fog, 2× supersampling, checker floor |
| `geom.py` | Mesh primitives (frustum, sphere with checker, torus, box, tube path), `frame_z`, `rot`, `Scene` |
| `robot.py` | Rev 3 Cephalo-1 (9-joint chain) used by the older studies |
| `wrists.py`, `wrists2.py` | Tip-wrist studies (BallDrive, OmniWrist; hobby-part wrists) |
| `choreo.py`, `make_video.py`, `make_grid.py` | Rev 3 motion scripts and 2-/6-variant video renderers |
| `tentacles.py`, `writhe_grid.py` | Tentacle joint-type comparison (9 variants), see `arm-joint-architecture.md` |
| `servo_arms.py` | Servo-at-every-joint variants and auto-sizing (`servo-joint-study.md`); its #5 double hinge does not package STS3215s (see rev 4 doc) |
| `loads.py`, `basebox.py`, `verify_box.py` | Tendon base-box load cases, packer and checks (not buildable per Forest) |
| `sim_wrist.py`, `sweep_wrist.py`, `make_hitsim_video.py` | One-arm MuJoCo wrist hit tests (rev 3) |
| `sim_seats.py`, `sweep_seats.py`, `sweep_clutch.py`, `tune_clutch.py`, `make_clutch_video.py` | One-arm clutch/seat sims and sweeps (rev 3 arm) |
| `clutch_parts.py`, `render_clutch.py`, `render_parts_sheet.py` | Printable clutch cam/rocker/winch STLs (clutch set v1, used unchanged in rev 4) and renders |

## Conventions
- **Real parts only:** every mechanism must be 3D-printable (PETG/TPU on a normal FDM printer) plus hardware-store or hobby parts (M2/M3/M4 bolts, MR/62x/68x bearings, magnets, Dyneema line, shock cord, worm gearmotors, the Feetech STS servos). Name the parts in docs and video labels. No custom micro gears or precision linkages that need tight shared axes. Size bearings against their static rating (C0). **Check packaging with the real servo bodies** (an STS3215 is 45.2 mm long, shaft ~11.4 mm from the output end): simplified meshes must not overlap.
- **Servo at every joint:** tendon/base-box drives are not buildable for Forest (`servo-joint-study.md`).
- **Units and frames:** SI metres in code; mm in docs. Body origin is the centre of the plate underside, +z up. Arm i root azimuth is 22.5° + 45°·i. Arm local frame: x along the arm, z is the arm's up. A positive pitch moves the tip down.
- **One source of truth:** rev 4 dimensions, masses and thrust live in `cephalo4.py` and must match `cephalo1-design-state.md`. When the design changes, update the doc, the code and the videos together.
- **Full-robot check:** a design change is not done until `sim4.py` completes the mission in the mid case and the pessimistic case is reported (fan saturation, tilt), and `states4.py` shows positive clearances.
- **Variant comparisons:** use identical commands for every variant. Each variant clamps to its own limits, and the HUD reports the error. Give each arm its own wrist instance, because wrists keep state.
- **Load check:** before showing a pose in a video, check the static servo torques against stall/rated. For impacts, check the gear-train load (servo torque + reflected rotor inertia × acceleration), not just the servo torque.
- **Video:** layouts are 1280×720 for 2 variants, 1920×720 for 3 variants, and 1920×1080 for 4 or 6 variants and for single-robot mission videos. All use 30 fps, libx264, crf 18, preset veryfast, yuv420p. Render checking stills first. Split the frames into contiguous chunks, one process per core; each chunk writes `*.part.mp4` and is renamed when ffmpeg exits 0. Join with the concat demuxer and `-c copy`. Slow motion is done by recording sim states densely (300 fps) and playing them at 30 fps, with a "SLOW MOTION" label.
- **Parallel work:** independent chunks or variants may run in parallel processes or agents, but always from the same code files.
- **Honesty labels:** joint limits, forces and masses that are estimates are marked as such in docs and HUDs. Kinematic videos say so; scripted impact responses say so; MuJoCo results list their assumptions.
