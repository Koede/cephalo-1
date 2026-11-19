# Cephalo-2

Working files of the Octopus Drone project's current robot, Cephalo-2: rev 4 servo arms, hand3 on all eight
arms, the hand/arm coupler (connector rev B.2), the scabbard belly rotor and its turret cameras, and the wheel
hand. This folder is a flat copy of the design workspace (code, design notes, check results and STLs). Videos,
PNG renders and caches are left out.

## Where to start

- `coupler-revB.md`: the hand/arm coupler, rev B.1 fixes and the rev B.2 optimisation.
  Code: `handmount_b.py` (geometry, checks, STLs), `coupler_opt.py`, `coupler_hand3_fit.py`, `coupler_viewer.py`.
  STLs: `handmount_b_stl/`. Live viewer: https://claude.ai/artifact/Cp2PWLZZN2RphhcBBLhQCe
- `wheelhand.md`: the wheel hand (`wheelhand.py`, `wheelhand_sizing.py`, STLs in `wheelhand_stl/`).
- `scabbard-drive.md`: the belly rotor drives (`scabbard_drive.py`).
- `turretcams.md`: the turret cameras (`turretcams.py`, STLs in `turretcams_stl/`).
- `cephalo2-baseline.md`: the Cephalo-2 baseline and its 3D viewer (`cephalo2_model.py`, `cephalo2_viewer.py`).
- `hand3/`, `hand3e/`: the hand3 design, CAD and checks (`hand3e/` is the newer copy).
- `cephalo4.py`, `sim4.py`, `body4.py`, `viz4.py`: the MuJoCo robot model, controller and renderer used by the
  movement studies (`*4.py` scripts and their `*.md` notes).
- `environment-and-conventions.md`: units, frames and the Python environment.

## Running

Python 3 with `numpy`, `manifold3d`, `trimesh` (CAD) and `mujoco` (sims). Run scripts from this folder, for example:

    python3 handmount_b.py      # coupler checks + STLs
    python3 wheelhand.py        # wheel hand checks + STLs

`monitor_out_*`, `*_runs/` and similar folders hold saved outputs of earlier sim runs.
