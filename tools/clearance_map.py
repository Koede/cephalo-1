#!/usr/bin/env python3
"""Minimum distance (mm) between neighbouring links (not just overlap), per pose.

  python3 tools/clearance_map.py                       # straight, curl, wave, and each joint at +-45
  python3 tools/clearance_map.py --margin 1.0 --pose 0:30,2:-30
Surface points are sampled ~--spacing mm apart on each link mesh (trimesh) and the closest pair
found with a KD-tree: the result is the true gap plus at most ~spacing (an upper bound on the gap).
Pairs: (i, i+2) (next neighbours); (i, i+1) touch at the hinge by design, so overlap there is judged by collision.py instead. Flags gaps under --margin (default 0.5 mm = printer tolerance proxy).
"""
import argparse, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import build_arm, build_bodies, Checks, add_paths
add_paths()
from kin import Kin, named_pose
from scipy.spatial import cKDTree
import trimesh


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--margin", type=float, default=0.5); ap.add_argument("--spacing", type=float, default=1.0)
    ap.add_argument("--pose", default=None); ap.add_argument("--per-joint", type=float, default=45.0)
    a = ap.parse_args()
    from pin_cad import to_trimesh
    arm = build_arm(); bodies = build_bodies(arm); J = arm.J; k = Kin.from_arm(arm)
    meshes = [to_trimesh(b) for b in bodies]
    rng = np.random.default_rng(0)
    pts = []
    for m in meshes:
        n = max(2000, int(m.area / a.spacing ** 2))
        pts.append(np.asarray(trimesh.sample.sample_surface(m, n, seed=0)[0]))
    def gap(q, i, j):
        W = arm.pose_frames(list(q))
        A = pts[i] @ W[i][:3, :3].T + W[i][:3, 3]; B = pts[j] @ W[j][:3, :3].T + W[j][:3, 3]
        d, _ = cKDTree(B).query(A); return float(d.min())
    poses = {}
    if a.pose:
        q = np.zeros(J)
        for tok in a.pose.split(","):
            j, d = tok.split(":"); q[int(j)] = math.radians(float(d))
        poses[f"pose {a.pose}"] = q
    else:
        for nm in ("straight", "curl", "wave"): poses[nm] = named_pose(k, nm)
        for j in range(J):
            for s in (1, -1):
                q = np.zeros(J); q[j] = s * math.radians(a.per_joint); poses[f"joint {j} {s*a.per_joint:+g}"] = q
    C = Checks(f"link clearance (surface sample spacing {a.spacing:g} mm; gap is an upper bound within that); margin {a.margin:g} mm")
    for nm, q in poses.items():
        worst = min(((gap(q, i, j), (i, j)) for i, j in [(i, i + 2) for i in range(J - 1)]), key=lambda t: t[0])
        C.add(nm, worst[0] >= a.margin, f"min gap {worst[0]:.2f} mm between link {worst[1][0]} and {worst[1][1]}")
    C.exit()


if __name__ == "__main__":
    main()
