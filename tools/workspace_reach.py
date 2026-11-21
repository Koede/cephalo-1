#!/usr/bin/env python3
"""Reachable workspace of one arm tip (kinematic Monte Carlo over +-range/2), optional 8-arm union.

  python3 tools/workspace_reach.py --samples 100000 --png reach.png
  python3 tools/workspace_reach.py --hub-radius 110          # 8 arms on a ring, ground clearance
Prints: max reach, mean/median radius, reach in the cone around +z (the arm axis) and below the
base plane, the bounding box, and the occupied-voxel volume (10 mm voxels). --png saves XZ/XY views.
"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import build_arm
from kin import Kin


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", type=int, default=50000); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--range", type=float, default=90.0); ap.add_argument("--voxel", type=float, default=10.0)
    ap.add_argument("--hub-radius", type=float, default=None); ap.add_argument("--png", default=None)
    ap.add_argument("--length", type=float); ap.add_argument("--joints", type=int); ap.add_argument("--twist", type=float)
    a = ap.parse_args()
    k = Kin.from_arm(build_arm(a.length, a.joints, a.twist)); half = a.range / 2
    rng = np.random.default_rng(a.seed)
    P = np.array([k.tip(np.radians(rng.uniform(-half, half, k.J))) for _ in range(a.samples)])
    r = np.linalg.norm(P, axis=1)
    vox = len({tuple(v) for v in np.floor(P / a.voxel).astype(int)})
    print(f"arm length {k.length:.0f} mm; {a.samples} samples within +-{half:g} deg")
    print(f"max reach {r.max():.0f} mm, median {np.median(r):.0f}, tip z range {P[:,2].min():.0f}..{P[:,2].max():.0f}, "
          f"radial (xy) max {np.hypot(P[:,0], P[:,1]).max():.0f}")
    print(f"tips below the base plane (z<0): {(P[:,2] < 0).mean()*100:.1f}%; occupied volume ~{vox * a.voxel**3 / 1e6:.2f} L (voxel {a.voxel:g} mm)")
    if a.hub_radius:
        ang = np.arange(8) * np.pi / 4; reach = a.hub_radius + np.hypot(P[:, 0], P[:, 1]).max()
        print(f"8 arms on R={a.hub_radius:g}: furthest tip from the hub axis {reach:.0f} mm; neighbouring-arm spacing at the root {2*a.hub_radius*np.sin(np.pi/8):.0f} mm")
    if a.png:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        f, ax = plt.subplots(1, 2, figsize=(10, 5)); s = P[::max(1, len(P) // 8000)]
        ax[0].scatter(s[:, 0], s[:, 2], s=1); ax[0].set_title("tip positions, XZ"); ax[0].set_aspect("equal")
        ax[1].scatter(s[:, 0], s[:, 1], s=1); ax[1].set_title("XY"); ax[1].set_aspect("equal"); f.savefig(a.png, dpi=110)
        print("saved", a.png)


if __name__ == "__main__":
    main()
