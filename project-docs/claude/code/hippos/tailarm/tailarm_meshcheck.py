"""Exact mesh check of every named tail pose (tailarm_checks.json) with FCL: tail meshes (root cradle, winch, arm, connector,
hand3) against the horse preview geometry and Cephalo-2 (scene2_notail.npz, NO_TAIL=1 build, mount left out), and the arm's links
against each other (joints two or more apart).  Joint angles are relative to the body, so reared poses are checked in the
horse frame too.  python3 tailarm_meshcheck.py -> tailarm_meshcheck.json"""
import sys, json
sys.path[:0] = ["/home/claude/octodrone/hippos/tailarm", "/home/claude/octodrone/hippos", "/home/claude/octodrone/ceph2"]
import numpy as np, trimesh
from trimesh.collision import CollisionManager
import tailarm as TA
M_CONN = np.array([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])
J = json.load(open("tailarm_checks.json"))
D = np.load("/home/claude/octodrone/centaur/scene2_notail.npz")
env = CollisionManager()
for k in D.files:
    if k.startswith("g") and not k.endswith("_meta"):
        T = D[k].reshape(-1, 3, 3); env.add_object(k, trimesh.Trimesh(T.reshape(-1, 3), np.arange(len(T) * 3).reshape(-1, 3), process=False))
oi = sorted(int(k[1:-2]) for k in D.files if k.startswith("o") and k.endswith("_v"))
for i in oi[:-4]: env.add_object(f"o{i}", trimesh.Trimesh(D[f"o{i}_v"], D[f"o{i}_f"], process=False))
import tailarm_checks as K                                     # for ORG (same placement as the checks)
ORG = K.ORG
out = {}
for name, p in J["poses"].items():
    if "q_deg" not in p: continue
    ms = TA.meshes(np.radians(p["q_deg"]), ORG, M_CONN)
    tail = trimesh.util.concatenate([m for _, _, m in ms])
    d, names = env.min_distance_single(tail, return_name=True)
    hit = env.in_collision_single(tail)
    out[name] = dict(min_gap_mm=round(1e3 * d, 1), nearest=names, collides=bool(hit))
    print(f"{name:42s} gap {1e3*d:7.1f} mm  nearest {names}  {'CLASH' if hit else ''}")
json.dump(out, open("tailarm_meshcheck.json", "w"), indent=1)
