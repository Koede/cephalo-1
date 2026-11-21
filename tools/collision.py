#!/usr/bin/env python3
"""Layered collision check of the arm (mesh interference, volumes in mm^3, contact > 2 mm^3).

  python3 tools/collision.py                       # levels static,sweeps,pairs  (about 10 s)
  python3 tools/collision.py --level all --samples 200 --seed 1
  python3 tools/collision.py --level mc --range 90 --samples 500
  python3 tools/collision.py --pose 0:30,1:-20,2:45     # check one pose (joint:deg, others 0)

Levels: static (all joints 0, neighbours), sweeps (each joint alone, +-range/2 incl. next-neighbours),
pairs (joint j and j+1 together, same and opposite sense, to +-60), mc (random poses within
+-range/2, links <= 3 apart), mantle (the 8 port cutters against each other, if the STL exists).
Prints the colliding link pair and the pose that reproduces it; exit 1 on any collision.
"""
import argparse, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import build_arm, build_bodies, Checks, MAIN_OUT
from geom import pose_overlaps, worst, OVERLAP_MM3


def fmt_pose(q):
    return ",".join(f"{j}:{math.degrees(v):.1f}" for j, v in enumerate(q) if abs(v) > 1e-9) or "all zero"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--level", default="static,sweeps,pairs"); ap.add_argument("--length", type=float)
    ap.add_argument("--joints", type=int); ap.add_argument("--twist", type=float); ap.add_argument("--no-gear", action="store_true")
    ap.add_argument("--range", type=float, default=90.0); ap.add_argument("--step", type=float, default=5.0)
    ap.add_argument("--samples", type=int, default=100); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--pose", default=None)
    a = ap.parse_args()
    arm = build_arm(a.length, a.joints, a.twist, geared=not a.no_gear)
    bodies = build_bodies(arm); J = arm.J; half = a.range / 2
    levels = {"static", "sweeps", "pairs", "mc", "mantle"} if a.level == "all" else set(a.level.split(","))
    C = Checks(f"collision check: {J} joints, pitch {arm.pitch:.1f} mm, twist {arm.twist:g}, +-{half:g} deg")

    if a.pose:
        q = [0.0] * J
        for tok in a.pose.split(","):
            j, d = tok.split(":"); q[int(j)] = math.radians(float(d))
        ov = pose_overlaps(arm, bodies, q, [(i, k) for i in range(J + 1) for k in range(i + 1, min(J + 1, i + 4))])
        p, v = worst(ov)
        C.add(f"pose {a.pose}", p is None, "clear" if p is None else f"link {p[0]} vs {p[1]}: {v:.1f} mm^3")
        C.exit()

    if "static" in levels:
        ov = pose_overlaps(arm, bodies, [0.0] * J)
        p, v = worst(ov); C.add("static, all joints 0", p is None, f"max {max(ov.values()):.2e} mm^3" if p is None else f"link {p[0]} vs {p[1]}: {v:.1f}")
    if "sweeps" in levels:
        angs = np.arange(a.step, half + 0.1, a.step)
        for j in range(J):
            first = None
            for sgn in (1, -1):
                for d in angs:
                    q = [0.0] * J; q[j] = sgn * math.radians(d)
                    pairs = [(j, j + 1)] + ([(j - 1, j + 1)] if j else []) + ([(j, j + 2)] if j + 2 <= J else [])
                    p, v = worst(pose_overlaps(arm, bodies, q, pairs))
                    if p: first = (sgn * d, p, v); break
                if first: break
            C.add(f"joint {j} alone to +-{half:g}", first is None, "clear" if first is None else f"{first[0]:+.0f} deg: link {first[1][0]} vs {first[1][1]} ({first[2]:.1f} mm^3)")
    if "pairs" in levels:
        for j in range(J - 1):
            first = None
            for sgn in (1, -1):
                for d in np.arange(a.step, 60.1, a.step):
                    q = [0.0] * J; q[j] = math.radians(d); q[j + 1] = sgn * math.radians(d)
                    p, v = worst(pose_overlaps(arm, bodies, q, [(j, j + 2)] + ([(j - 1, j + 1)] if j else []) + ([(j + 1, j + 3)] if j + 3 <= J else [])))
                    if p: first = (sgn, d, p, v); break
                if first: break
            C.add(f"joints {j}+{j+1} together to +-60", first is None, "clear" if first is None else f"{'same' if first[0] > 0 else 'opposite'} sense {first[1]:.0f} deg: link {first[2][0]} vs {first[2][1]}")
    if "mc" in levels:
        rng = np.random.default_rng(a.seed); hits = []
        pairs = [(i, k) for i in range(J + 1) for k in range(i + 2, min(J + 1, i + 4))]
        for n in range(a.samples):
            q = np.radians(rng.uniform(-half, half, J))
            p, v = worst(pose_overlaps(arm, bodies, q, pairs))
            if p: hits.append((v, p, q))
        if hits:
            v, p, q = max(hits, key=lambda h: h[0])
            C.add(f"random poses (n={a.samples}, seed {a.seed})", False, f"{len(hits)} colliding; worst link {p[0]} vs {p[1]} {v:.1f} mm^3 at --pose {fmt_pose(q)}")
        else:
            C.add(f"random poses (n={a.samples}, seed {a.seed})", True, "none colliding")
    if "mantle" in levels:
        import trimesh
        f = os.path.join(MAIN_OUT, "mantle_port_cutters_x8_r110.stl")
        if os.path.exists(f):
            m = trimesh.load(f)
            parts = m.split(only_watertight=False)
            C.add("mantle port cutters: count", len(parts) == 8, f"{len(parts)} bodies")
            from manifold3d import Manifold, Mesh
            def man(t): return Manifold(Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
            ms = [man(p) for p in parts]; bad = []
            for i in range(len(ms)):
                for k in range(i + 1, len(ms)):
                    v = (ms[i] ^ ms[k]).volume()
                    if v > OVERLAP_MM3: bad.append((i, k, v))
            C.add("mantle port cutters: mutual overlap", not bad, "none" if not bad else str(bad[:3]))
    C.exit()


if __name__ == "__main__":
    main()
