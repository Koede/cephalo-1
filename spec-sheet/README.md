# Pin-hinge arm spec sheet

The spec sheet is the living record for the Cephalo-1 pin-hinge arm. It is updated whenever the
arm design changes, so it is the place to check the current numbers.

Live page: https://claude.ai/artifact/3t777NsUV2NmNSZxF3stNS

## Contents

- `page/`: the spec sheet page (`index.html`, `style.css`) as built by `build_page.py`.
- `sec1_geom.py` to `sec5_system.py`: the five sections of the sheet (geometry, loads,
  tolerances, structure, system), using the shared helpers in `spec_lib.py`.
- `poses.py`, `finalize_poses.py`, `stance2.py`, `hover_tilt.py`, `merge_loads.py`,
  `make_summaries.py`: the pose, stance, hover-tilt and load runs that feed the sections.
- `results/`: the outputs of those runs (JSON summaries, pose arrays and run logs).
- `cad_alt90/`: the 90° alternating-axis CAD variant used for comparison with the 60° helix.
