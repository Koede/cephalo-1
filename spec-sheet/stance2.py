"""Walking stance on the folded pod with a feasibility-first multistart: random starts -> least squares onto the
constraints -> SLSQP on need from the feasible ones. args: ARR SIDE RADIUS [feet]"""
import sys, os, json, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from scipy.optimize import least_squares
import spec_lib as S, poses as PS
arr, side_n, RF = sys.argv[1], sys.argv[2], float(sys.argv[3])
FEETN = sys.argv[4].split(",") if len(sys.argv) > 4 else ["mid", "side +45", "in 34"]
A = S.Arm(arr); ROBOT = 18.95 + 8 * S.WIRE_G_PER_ARM / 1000; FOOT = ROBOT * S.G / 7
SIDES = {"+x": [1, 0, 0], "-x": [-1, 0, 0], "+y": [0, 1, 0], "-y": [0, -1, 0]}; sd = SIDES[side_n]
FEET = {"mid": [RF, 0, -0.25], "side -45": [RF, -0.045, -0.25], "side +45": [RF, 0.045, -0.25], "in 34": [RF - 0.034, 0, -0.25]}
out = {}; store = {}
for fn in FEETN:
    foot = np.array(FEET[fn]); t0 = time.time()
    def cons(q, f):
        fp = PS.foot_point(A, f, sd); z = f["tipR"][:, 2]
        eq = list(fp - foot)
        iq = [float(z @ PS.DOWN) - math.cos(math.radians(10)), S.self_clear(f)] + list(f["marks"][1:-1, 2] - (foot[2] + S.R_OUT + 0.015)) + PS.azimuth_cons(f)
        return eq, iq
    def load(q, f): return np.zeros(3), "tip", [(PS.foot_point(A, f, sd), np.array([0, 0, FOOT]))]
    def viol(q):
        f = A.fk(q); e, i = cons(q, f); return np.concatenate([np.array(e) * 10, np.minimum(0, np.array(i)) * 10])
    rng = np.random.default_rng(1); feas = []
    for k in range(36):
        x0 = rng.uniform(-PS.LIM, PS.LIM, S.N)
        r = least_squares(viol, x0, bounds=(-PS.LIM, PS.LIM), max_nfev=400)
        if np.abs(viol(r.x)).max() / 10 < 2e-3: feas.append(r.x)
    res = PS.solve(A, load, cons, feas[:6]) if feas else dict(ok=False, worst=float("nan"), q=np.zeros(S.N), need=np.zeros(S.N))
    f = A.fk(res["q"])
    n2 = A.need(res["q"], np.zeros(3), Fpt="tip", extra=[(PS.foot_point(A, f, sd), np.array([0, 0, 2 * FOOT]))])[0] if feas else np.zeros(S.N)
    out[fn] = dict(feasible_starts=f"{len(feas)}/36", ok=bool(res["ok"]), worst=round(float(res["worst"]), 3), worst_2x=round(float(n2.max()), 3),
                   need=[round(float(x), 2) for x in res["need"]], q_deg=[round(math.degrees(x), 1) for x in res["q"]], s=round(time.time() - t0))
    if res["ok"]: store[f"{arr}_stance_{side_n}_{fn}"] = res["q"]
    print(fn, out[fn], flush=True)
tag = f"{arr}_{side_n}_{int(RF*100)}"
json.dump(dict(radius_m=RF, side=side_n, rows=out), open(os.path.join(S.HERE, "results", f"stance2_{tag}.json"), "w"), indent=1)
np.savez(os.path.join(S.HERE, "results", f"stance2_{tag}.npz"), **store)
