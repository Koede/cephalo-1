"""Planned move from the lifted stance to the raised fold pose (Cephalo-1, Nov 10). See pod_fold.py.

A straight joint-space move between the two poses swings the arm through the ground, so the finger is led along a
path instead: pull in toward the arm root while low, then raise. Each waypoint is solved for least worst need with the
prop parked, and the straight servo moves between waypoints are checked (pod_gait.path adds waypoints where needed).
Writes results/pod_fold_move.json.
"""
import os, sys, json, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
import pod_gait as G, pod_stance as PS

def main():
    t0 = time.time(); np.random.seed(0)
    st = json.load(open(os.path.join(HERE, "..", "results", "pod_stance.json")))
    fo = json.load(open(os.path.join(HERE, "..", "results", "pod_fold.json")))
    A = G.Gait(); caps = PS.caps_for(set()); q_nom = np.radians(st["cases"]["hub 250 mm above ground"]["q_deg"][0])
    nb = G.neighbours(A, q_nom)
    q_lift = np.radians(fo["lift_rows"][-1]["q_deg"]); q_fold = np.radians(fo["fold_pose"]["q_deg"])
    f_lift = A.geom(q_lift)[2]; f_fold = A.geom(q_fold)[2]; root = A.fk(q_lift)[0][1]
    radial = (f_lift[:2] - root[:2]) / np.linalg.norm(f_lift[:2] - root[:2])
    p1 = np.r_[root[:2] + radial * 0.35, root[2] + 0.10]       # pulled in, still low
    p2 = np.r_[root[:2] + radial * 0.20, root[2] + 0.45]       # rising, close to the root
    legs = [(f_lift, p1, "pull in"), (p1, p2, "raise"), (p2, f_fold, "stand up")]
    steps = [("start", f_lift, 0.0, True, (88.0,))]
    for a, b, nm in legs:
        N = int(os.environ.get("LEG_STEPS", "6"))
        steps += [(f"{nm} {i}/{N}", a + (b - a) * i / N, 0.0, True, (88.0,)) for i in range(1, N + 1)]
    q_end, rows = G.path(A, q_lift, caps, nb, steps)
    bw, bc, bs = G.between(A, q_end, q_fold, caps, nb, 0.0, True, 88.0, n=8)
    w = max(rows, key=lambda r: max(r["worst"], r["between"]["worst"]))
    out = dict(rows=rows, last_move_to_fold_pose=dict(worst=round(bw, 3), clear_mm=round(bc * 1e3, 1),
               move_deg=round(float(np.degrees(np.abs(q_fold - q_end).max())), 1)),
               worst=round(max(max(r["worst"], r["between"]["worst"]) for r in rows), 3), worst_at=w["label"],
               least_clear_mm=min(min(r["clear_mm"], r["between"]["clear_mm"]) for r in rows),
               peak_need_by_joint=[round(max(r["need"][j] for r in rows), 3) for j in range(13)])
    json.dump(out, open(os.path.join(HERE, "..", "results", "pod_fold_move" + os.environ.get("TAG", "") + ".json"), "w"), indent=1, default=float)
    print("planned move: worst", out["worst"], "at", out["worst_at"], "| least clear", out["least_clear_mm"], "mm | all ok",
          all(r["ok"] for r in rows), "| last move", out["last_move_to_fold_pose"], "| waypoints",
          [(r["label"], r["waypoints_added"]) for r in rows if r["waypoints_added"]], f"t={time.time()-t0:.0f}s", flush=True)
    print("peak by joint", out["peak_need_by_joint"], flush=True)

if __name__ == "__main__":
    main()
