"""Torque-aware pose solvers shared by the spec-sheet sections.
Each pose minimises a smooth max (16-norm) of need/stall over the 13 joints under SLSQP, with the pose's geometric
constraints; a least-squares pass first moves each seed onto the constraints."""
import math
import numpy as np
from scipy.optimize import minimize, least_squares
import spec_lib as S

LIM = math.radians(45)
UP = np.array([0, 0, 1.0]); DOWN = -UP
FOOT_FINGER = (0.0425, 0.078)     # m: finger tip off the arm axis, and out from the tip plate (pod folded 88 deg)

def foot_point(A, f, side):
    R = f["tipR"]; return f["tip"] + R[:, 2] * FOOT_FINGER[1] + R @ np.asarray(side, float) * FOOT_FINGER[0]

def solve(A, load_fn, cons_fn, x0s, cap=S.CAP, maxiter=400, p=16):
    def needs(q):
        f = A.fk(q); F, Fpt, ex = load_fn(q, f)
        return A.need(q, F, Fpt=Fpt, extra=ex, cap=cap)[0], f
    def viol_vec(q):
        f = A.fk(q); eq, iq = cons_fn(q, f)
        return np.concatenate([np.asarray(eq, float) * 10, np.minimum(0.0, np.asarray(iq, float)) * 10, 1e-3 * q])
    obj = lambda q: math.log(np.sum(needs(q)[0] ** p) + 1e-300) / p + 1e-4 * np.sum(q ** 2)
    cons = [dict(type="eq", fun=lambda q: np.array(cons_fn(q, A.fk(q))[0] or [0.0])),
            dict(type="ineq", fun=lambda q: np.array(cons_fn(q, A.fk(q))[1] or [1.0]))]
    best = None
    for x0 in x0s:
        x0 = np.clip(x0, -LIM + 1e-6, LIM - 1e-6)
        try: x0 = least_squares(viol_vec, x0, bounds=(-LIM, LIM), max_nfev=300).x
        except Exception: pass
        r = minimize(obj, x0, method="SLSQP", bounds=[(-LIM, LIM)] * S.N, constraints=cons, options=dict(maxiter=maxiter, ftol=1e-9))
        q = r.x; f = A.fk(q); eq, iq = cons_fn(q, f)
        viol = max([abs(e) for e in eq] + [-min(0, i) for i in iq] + [0])
        n = needs(q)[0]; ok = viol < 2e-3
        cand = (0 if ok else 1, float(n.max()) if ok else viol, q, n, viol)
        if best is None or cand[:2] < best[:2]: best = cand
    return dict(ok=best[0] == 0, worst=float(best[3].max()), q=best[2], need=best[3], viol=best[4])

def azimuth_cons(f, deg=18.0, rmin=0.15):
    P = f["marks"][1:]; r = np.hypot(P[:, 0], P[:, 1]); t = math.tan(math.radians(deg))
    return list(np.where(r > rmin, P[:, 0] * t - np.abs(P[:, 1]), 1.0))

def seeds(n=4, rng=None, base=None):
    rng = rng or np.random.default_rng(0); out = [] if base is None else [np.asarray(base, float)]
    while len(out) < n: out.append((np.asarray(base, float) if base is not None else 0) + rng.uniform(-0.5, 0.5, S.N))
    return out
