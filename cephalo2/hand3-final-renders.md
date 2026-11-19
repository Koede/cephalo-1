# hand3 final on the robot: rerun and rerendered (2026-11-15)

Forest: "Make all hands be the new updated ones and rerender." Both hand arms (2 and 6) now carry **hand3 final (hub C2, `claude/hand3-final.md`)** in the sims and in the renders. Everything is simulation or estimate.

## What changed in the sims
- Hand numbers come from the final design folder (`claude/code/hand3e/`, local `hand3e/`): 396.4 g, COM 58.6 mm from the spigot, blade plane at 96 mm, rotor R 114.3 mm, full-throttle thrust 21.48 / 23.12 / 24.76 N (low / mid / high), q/T 0.0290 / 0.0273 / 0.0256, 10.22 W/N^1.5 (`hand3_params.py`, which now reads `H3_DIR`, default `hand3e`; `H3_MASS` / `H3_COM_MM` override).
- Contact model (`hand3_mj.py`): the V30 heel saddle (`H3_HEEL`, default `v:30`; `flat` = the old post) and the seat height of a bar in it; hub C2 ring forces, springs and levers from `hand3e/hand3_hubc.json`.
- Shovel (`quad4r.py`): the final grip points, upper (0.0994, 0.2335, -0.2265) m and lower (0.2105, -0.1273, -0.3255) m, dig directions from the shaft heading, `Q_IMPRATIO` (default 10).
- Flight (`body4h.py` locally): the yaw-yield fix from `tests/body4h_yawyield.diff`.
- `sim4.py` in this workspace reads `C.M_POD` when it builds each arm, so the pod-mass bug in `tests/shovel_restore.diff` does not apply here.

## Results of the rerun (robot 12.32 kg, as in the flight test)
- **Shovel, 13 s, three dig cycles:** no drop, no prop saturation, position error max 3.1 cm, J 5.6. With impratio 1000 (the setting the final test recommends) the shaft moves at most 4.4 mm / 2.0 mm in the two hands (final test: 3.0 / 2.8 mm). With the default impratio 10 the lower hand beds in to 12.5 mm in the second cycle and then holds (no creep). Ring cord rms 0.86-0.90 (base) and 0.53-0.58 (curl) of rated. Worst arm servo thermal 1.30 (arm 0 j0).
- **Day mission (`hand3_fly.py low|mid|high day`):** flies and lands at all three levels; tilt in the air ≤ 2.6°, lowest bus 22.1 V, hand ESC ≤ 0.88 of rating, hover 0.47-0.54 of the hand ceiling, time at ≥ 0.98 of the ceiling 0.13 / 0 / 0 s. Fly legs peak at 0.88 / 0.83 / 0.78 (low / mid / high): inside the 0.95 limit, but higher than the 0.79 / 0.73 / 0.68 of `tests/flight_test.py`, which runs with the documented stand / flight poses (`CEPH_SENSES=0 python3 poses4.py`) that this workspace does not use.

## Renderer
`hand3_draw.py` draws hand3 final from its CAD (rebuilt from `hand3e/`): the one-body servo frame, two STS3215, spools, idlers, eight carbon rods and the tie ring, the four yoke halves (they drop with their ring's stroke), the drive cords (redrawn each frame), hub plates, cage, the V30 heel, the spring boxes, the base and curl rings with their TPU links and trim screws (they slide with the finger pose), and the fingers (aluminium root grips, PA12 phalanges, TPU pads). The hand is clocked 45° on the wrist, as in the sim. In flight the four straight fingers turn as the rotor.

## Files
Videos: `cephalo1_body_h3dfly_mid_day.mp4` (take-off, flight, landing), `cephalo1_hand_work_shovel.mp4` (shovel by finger contact, with a close-up along the shaft); still `h3_still.png` (flight and grasp poses). Code: `hand3_draw.py`, `hand3_mj.py`, `hand3_params.py`, `quad4r.py`; the hand geometry is `claude/code/hand3e/`.

Not rerendered (older sims, other hands or no hands): the chores videos (shovel pickup, handover, dig, dishes), the docking grab, falls, flips and the earlier mission videos.
