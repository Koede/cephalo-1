#!/usr/bin/env python3
"""Export an arm pose as an STL assembly and a PNG preview.

  python3 tools/pose_render.py --pose curl --out /tmp/pose
  python3 tools/pose_render.py --angles 0:30,1:-20,5:45 --png pose.png
  python3 tools/pose_render.py --pose wave --no-stl            # PNG only (skeleton + tip)
Named poses: straight, curl, wave, zigzag, coil. --angles joint:deg (others 0). The STL has the
printed links and servos placed with pin_cad.pose_frames; the PNG (matplotlib) shows the joint
axes and arm centreline from the side and the front.
"""
import argparse, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import build_arm
from kin import Kin, named_pose, axis_dir


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose", default="straight"); ap.add_argument("--angles", default=None)
    ap.add_argument("--out", default="/tmp/cephalo_pose"); ap.add_argument("--png", default=None); ap.add_argument("--no-stl", action="store_true")
    ap.add_argument("--length", type=float); ap.add_argument("--joints", type=int); ap.add_argument("--twist", type=float)
    a = ap.parse_args()
    arm = build_arm(a.length, a.joints, a.twist); k = Kin.from_arm(arm)
    if a.angles:
        q = np.zeros(k.J)
        for tok in a.angles.split(","):
            j, d = tok.split(":"); q[int(j)] = math.radians(float(d))
    else:
        q = named_pose(k, a.pose)
    os.makedirs(a.out, exist_ok=True)
    print("tip:", k.tip(q).round(1).tolist(), "mm")
    if not a.no_stl:
        import trimesh
        from pin_cad import to_trimesh
        W = arm.pose_frames(list(q)); J = arm.J
        parts = [arm.part(i) for i in range(J + 1)]
        ms = [to_trimesh(parts[i].transform(W[i][:3, :])) for i in range(J + 1)] + [to_trimesh(arm.servo_global(i).transform(W[i][:3, :])) for i in range(J)]
        f = os.path.join(a.out, "pose.stl"); trimesh.util.concatenate(ms).export(f); print("wrote", f)
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    W = k.frames(q); pts = np.array([(W[j] @ np.append(k.point(j), 1))[:3] for j in range(k.J)] + [(W[-1] @ np.append(k.point(k.J), 1))[:3]])
    fig, ax = plt.subplots(1, 2, figsize=(9, 6))
    for n, (u, v, t) in enumerate(((0, 2, "side (XZ)"), (1, 2, "front (YZ)"))):
        ax[n].plot(pts[:, u], pts[:, v], "-o", ms=3)
        for j, (p, d) in enumerate(k.world_axes(q)):
            ax[n].plot([p[u] - 25 * d[u], p[u] + 25 * d[u]], [p[2] - 25 * d[2], p[2] + 25 * d[2]], "r-", lw=1)
        ax[n].set_aspect("equal"); ax[n].set_title(t)
    out = a.png or os.path.join(a.out, "pose.png"); fig.savefig(out, dpi=110); print("wrote", out)


if __name__ == "__main__":
    main()
