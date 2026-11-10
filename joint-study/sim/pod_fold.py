"""Folding and unfolding the fan pod on a standing robot (Cephalo-1, Nov 10). See pod_gait.py for the model.

The arm unloads and lifts its finger, then moves to a fold pose where the whole prop disc (any blade angle, the prop
is stopped but not yet parked) clears the ground, its own arm, the neighbours and the central disc at every fold
angle from 0 to 88 deg. The move from the lifted stance to the fold pose is traced in joint space. Pod servo load is
the pod's weight about the hinge, plus the parking post spring over the last 1.6 deg. Writes results/pod_fold.json.
"""
import os, sys, json, math, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
import pod_gait as G, pod_stance as PS

THS = tuple(np.arange(0.0, 88.01, 8.0)) + (88.0,)

def main():
    t0 = time.time()
    st = json.load(open(os.path.join(HERE, "..", "results", "pod_stance.json")))
    A = G.Gait(); caps = PS.caps_for(set()); q_nom = np.radians(st["cases"]["hub 250 mm above ground"]["q_deg"][0])
    nb = G.neighbours(A, q_nom); f0 = A.geom(q_nom)[2]; fw = A.fw
    up = np.array([0, 0, G.LIFT])
    q_lift, rows = G.path(A, q_nom, caps, nb, [("unload", f0, 0.0, True, (88.0,)), ("lift", f0 + up, 0.0, True, (88.0,))])
    # fold pose: free finger, full disc clear at every fold angle, least worst need over the fold.
    # Starts: the study's named shapes and random poses, ranked by their worst clearance over the fold.
    import bearing_loads as B
    P, _ = B.poses(A.ch)
    def score(q):
        c = min(np.concatenate([G.geo_cons(A, q, nb, parked=False, th=t) for t in THS[::3]]).min(), 1.0)
        n = float(np.max([np.abs(A.torques_pod(q, 0.0, t)[0]) / caps for t in THS[::3]]))
        return c, n
    cands = sorted(((score(q), nm, q) for nm, q in P.items()), key=lambda x: -x[0][0] + 0.05 * x[0][1])
    print("best starts", [(nm, round(c * 1e3), round(n, 2)) for (c, n), nm, q in cands[:6]], flush=True)
    best = None
    for (c, n), nm, x0 in cands[:5]:
        q, info = G.track(A, x0, caps, nb, None, 0.0, False, THS, w=0.0005, x0=x0)
        print("fold pose from", nm, info, f"t={time.time()-t0:.0f}s", flush=True)
        if info["ok"] and (best is None or info["need"] < best[1]["need"]): best = (q, info)
    if best is None: best = (q, info)
    q_fold, info = best
    # joint-space move from the lifted stance to the fold pose, prop parked (folded) the whole way
    move = []
    for s in np.linspace(0, 1, 11):
        q = q_lift + s * (q_fold - q_lift); g = G.geo_cons(A, q, nb)
        r = G.row(A, q, caps, 0.0); move.append(dict(s=round(float(s), 1), worst=round(max(r["need"]), 3), clear_mm=round(float(g.min() * 1e3), 1)))
    # the fold itself in the fold pose
    fold = []
    for th in np.arange(0.0, 88.01, 2.0):
        t, pod = A.torques_pod(q_fold, 0.0, th); dc = G.disc_clear(A, q_fold, nb, th)
        spring = G.SPRING_NM if th >= G.PARK_FROM else 0.0
        fold.append(dict(theta=float(th), worst_joint=round(float((np.abs(t) / caps).max()), 3), j11=round(abs(t[11]) / caps[11], 3), j12=round(abs(t[12]) / caps[12], 3),
                         pod_servo=round((abs(pod) + spring) / G.POD_STALL, 3), disc_clear_mm=dict(zip(("ground", "central", "own arm", "neighbours"), [round(float(x) * 1e3, 1) for x in dc]))))
    m, R, P, hub, ax = G.disc_pts(A, q_fold, 0.0)
    out = dict(lift_rows=rows, fold_pose=dict(ok=info["ok"], worst=round(info["need"], 3), q_deg=[round(float(v), 1) for v in np.degrees(q_fold)],
                                              finger_mm=[round(float(v) * 1e3) for v in A.geom(q_fold)[2]], tip_dir=[round(float(v), 2) for v in R[:, 2]]),
               move_to_fold_pose=move, fold=fold,
               max_joint_move_deg=round(float(np.degrees(np.abs(q_fold - q_lift).max())), 1))
    json.dump(out, open(os.path.join(HERE, "..", "results", "pod_fold.json"), "w"), indent=1, default=float)
    print("fold pose", out["fold_pose"], "max move", out["max_joint_move_deg"])
    print("move", [(r["s"], r["worst"], r["clear_mm"]) for r in move])
    print("fold", [(r["theta"], r["worst_joint"], r["pod_servo"], min(r["disc_clear_mm"].values())) for r in fold[::4]] + [(fold[-1]["theta"], fold[-1]["worst_joint"], fold[-1]["pod_servo"])])

if __name__ == "__main__":
    main()
