# Cephalo-1 rev 4: servo-in-link arms with detent clutches (Octopus Drone project)

This folder holds the design line from the Octopus Drone project. It is separate from the 1 m pin-hinge arm in the rest of this repository. It describes an eight-armed robot that walks on its arms and flies with a fan at each arm tip (plus a central 90 mm EDF). The design aims to be 3D-printable (PETG/TPU) and built from real hobby parts: Feetech STS servos, 2207 motors, bearings, M-series bolts and Dyneema line.

**Current design: rev 4** (servo-in-link arms, winch-tensioned detent clutches, 6 in claw-props). Start with:
- [`docs/cephalo1-design-state.md`](docs/cephalo1-design-state.md): the current spec, poses, open issues and design history
- [`docs/cephalo1-rev4-integrated.md`](docs/cephalo1-rev4-integrated.md): rev 4 details, full-robot MuJoCo results, BOM
- [`docs/environment-and-conventions.md`](docs/environment-and-conventions.md): setup, code layout, conventions

## Layout
- `docs/` holds the design docs. Background studies: joint concepts, arm joint architecture, servo-joint study, impact-compliant (clutch) joint, base-box design, rev 3 dynamics.
- `code/` holds the Python models, simulations and renderers. `cephalo4.py` is the single source of truth for rev 4 geometry, masses and thrust.

## Quick start (rev 4 pipeline)
```bash
cd code
bash setup_env.sh                 # numpy, scipy, numba, trimesh, manifold3d, shapely, pillow, mujoco, cma
python3 poses4.py                 # stand / flight poses + static loads -> poses4.json
python3 sim4.py mid               # full-robot MuJoCo mission -> sim4_mid.npz / .json
python3 states4.py sim4_mid.npz   # clearance checks
python3 viz4.py still 20          # render a check frame
```
Printable clutch parts (STL): `python3 clutch_parts.py` writes them to `stl_clutch/`.

Values marked (est.) in the docs are estimates, not measurements. Claw-prop thrust is the main unmeasured value.
