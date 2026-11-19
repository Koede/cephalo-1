# Cephalo-1: organic mid-air correction (2026-11-10)

Ask (Forest): "draft up organic midair correction while flying". Code: `claude/code/flight4.py` (controller + hover test bench), `flight_video.py` (side-by-side video), `render_gpu.py` (optional CUDA rasterizer). Video: `cephalo1_organic_correction.mp4` (rendered on Saturn, `C:\Users\admin\workspace\octodrone\`).

## Idea
The rev 4 flight controller corrects everything with the fans: roll/pitch by differential claw-prop thrust, position by tilting the whole robot (max 0.10 rad), and it has no yaw control at all. Rev 4 already has 6 servo joints beyond the root in every arm, so the arms can aim their own claw-props. In "tentacle vectoring", horizontal force and yaw come from tilting the thrust axes, and the body stays level. The tilt is made the way an octopus arm would make it: as a curl along the arm, not a single wrist flick.

## Controller (`flight4.py`, mode `organic`)
1. **Outer loop (100 Hz):** horizontal acceleration demand = reference feed-forward + PD (4.0 /s², 3.6 /s) + integral (1.5, clamped at 1 m/s², cancels a steady gust), limited to 2.2 m/s². Yaw moment = PD on heading (6 N m/rad, 3 N m s/rad), limited to 6 N m.
2. **Allocation to the arms:** each claw-prop i gets a horizontal axis tilt δᵢ = F_h / ΣF + t̂ᵢ · M_z / Σ(Fᵢ rᵢ): a shared push plus a tangential twist for yaw. Capped at 18° per arm.
3. **Tentacle shape:** the tilt is spread over j2..j7 by weighted least squares on the claw-axis Jacobian at the flight pose (weights 0.35 / 0.65 / 1.0 for j2-j3 / j4-j5 / j6-j7, root j0/j1 untouched). Each joint follows with its own lag: tip 0.06 s, middle 0.12 s, root-side 0.20 s. So a correction starts at the claw and settles into a smooth curve down the arm. Airborne joint speed limit 180°/s (same as take-off v7).
4. **Breathing:** a 3-4° wave (3.2 s period) travels from root to tip and around the body. It is projected into the null space of the claw-axis Jacobian, so it moves the arms without tilting any thrust axis (hover drift 1.5 cm, tilt 0.3°).
5. **Fast loops unchanged:** roll/pitch stay on attitude-first differential thrust (kp 14, kd 4.5) with the EDF at 80 %. The 25° world-frame fan cut-out becomes a 28° → 32° fade for vectoring claw-props.

## Test bench (16 s hover at plate z 0.80 m, full-robot MuJoCo, same disturbances for both controllers)
- 2.0-3.5 s gust: 8 N sideways push on the body (est.)
- 5.5 s swirl: tangential pushes on the 8 claw-props, 2 N m total for 0.25 s
- 8.5 s side hit: 0.2 kg ball at 6 m/s into arm 2's claw-prop (as in the mission)
- 11.0 s move: target slides 0.6 m sideways in 2.0 s (minimum jerk)

| Mid thrust (12.6 N) | Fan-only (current) | Organic |
|---|---|---|
| Gust: max drift / body tilt | 10.9 cm / 7.5° | 9.2 cm / **2.4°** |
| Swirl: max heading error / after 3 s | 115° / 115° (keeps turning) | **7.9° / 0.2°** |
| Side hit: max drift / tilt / heading | 4.8 cm / 2.7° / spinning | 3.4 cm / 1.5° / 3.9° |
| Move 0.6 m: max tracking error / tilt | 43.5 cm / 2.5° | **4.0 cm / 0.6°** |
| Max claw-prop axis tilt from vertical | 14.9° (in the hit) | 16.3° (in the hit), 10.6° in the move |
| Worst servo, outside the hit | 0.12 × stall | 0.14 × stall |
| Worst servo in the hit (j1, as in the mission) | 1.00 | 1.00 |
| rms servo load over the run | 0.29 × rated | 0.31 × rated |
| Peak claw-prop thrust | 9.37 N | 9.35 N |

- **Pessimistic thrust (10.1 N):** identical results for both modes; no claw-prop saturates (peak 9.35 N).
- **Clearances:** at least 82.6 mm between neighbouring arms and 23.0 mm to the body, both at the hit (states4 check).
- **Reproducibility:** Saturn (Windows, MuJoCo 4.0.0) reproduced the cloud runs digit for digit.

Notes:
- The fan-only baseline has no yaw control because prop reaction torques are not modelled. A real build with counter-rotating claw-prop pairs could get some yaw from differential prop speed. Tentacle vectoring needs no prop-direction assumption.
- The servo load stays low because a claw-prop's thrust line passes close to the outer joints. Tilting it costs little torque; the hit is still the worst case.

## Open items
- Measure the claw-prop's own reaction torque and spin-up (also the take-off/landing blocker).
- Gust model is a body force only. Real wind loads the arms and props too; repeat with distributed drag.
- Use tentacle vectoring in the mission (`sim4.py`) after take-off, and test it against the j1 hit load.
- Yaw by tentacle twist could also replace part of the landing's attitude work (the pessimistic landing is still marginal).

## Rendering on Saturn
- **Hardware:** Forest's PC, Windows, Ryzen 5 5600X (6 cores / 12 threads), RTX 3060 12 GB, Python 3.11 with numpy, numba, numba-cuda, mujoco 4.0.0, scipy, Pillow and imageio-ffmpeg.
- **Access:** reached from the cloud session through Desktop Commander. The code is copied as an xz/base64 bundle in md5-checked chunks into `C:\Users\admin\workspace\octodrone` (folder sharing was not granted).
- **Speed:** 10 parallel CPU render processes did the 480-frame 1280×720 video in 185 s, about 2.5× the cloud workspace's 2 cores.
- **GPU path:** the CUDA rasterizer (`CEPH_GPU=1`, `render_gpu.py`) matches the CPU output (1 pixel in 3072 differs at a depth tie) but measured 1.52 s/frame vs 1.20 s on CPU. Rasterizing is only a small part of each frame; most of the time is scene building and image post-processing. The GPU only pays once those move to the GPU too, or for much heavier scenes.
