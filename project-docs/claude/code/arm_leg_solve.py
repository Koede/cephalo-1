"""One-arm solve with the foot position free (z on the floor, lateral band, ahead of the head's flank).
python3 arm_leg_solve.py <arm> <foot_N> -> arm_leg_<arm>_<N>.json"""
import sys, json
import numpy as np
from scipy.optimize import minimize
import chilo_arm_legs as A

i = int(sys.argv[1]); Ff = float(sys.argv[2])
plate = np.array([0.315 + 0.012, 0, A.HC])
root, R0 = A.arm_frame(i, plate)
lo, hi = A.LIM[:, 0] * A.MARGIN, A.LIM[:, 1] * A.MARGIN
az = (22.5 + 45 * i + A.ROLL) % 360; az = min(az, 360 - az)              # angle from straight down
ylo = 0.10 + 0.25 * np.sin(np.radians(az))


def obj(q):
    tau = A.torques(q, root, R0, Ff)
    return float(np.sum((tau / A.RATED) ** 2)) + 0.02 * float(np.sum(q ** 2))


def clr(q):
    P, Ax, spig, tip = A.fk(q, root, R0)
    return np.array([max(p[0] - (plate[0] + 0.025), np.hypot(p[1], p[2] - A.HC) - 0.150) for p in list(P) + [spig]])


cons = [{"type": "eq", "fun": lambda q: A.fk(q, root, R0)[3][2] - A.FOOT_R},
        {"type": "ineq", "fun": lambda q: abs(A.fk(q, root, R0)[3][1]) - ylo},
        {"type": "ineq", "fun": lambda q: A.REACH_MAX * A.L_FULL - np.linalg.norm(A.fk(q, root, R0)[3] - root)},
        {"type": "ineq", "fun": clr},
        {"type": "ineq", "fun": lambda q: A.fk(q, root, R0)[0][:, 2] - 0.04}]
rng = np.random.default_rng(i); best = None
for t in range(40):
    x0 = rng.uniform(lo, hi) * 0.7
    r = minimize(obj, x0, method="SLSQP", bounds=list(zip(lo, hi)), constraints=cons, options=dict(maxiter=300, ftol=1e-8))
    if r.success and min(clr(r.x)) > -1e-4 and (best is None or r.fun < best.fun):
        best = r
q = best.x; tau = A.torques(q, root, R0, Ff)
P, Ax, spig, tip = A.fk(q, root, R0)
rng_frac = np.where(q >= 0, q / A.LIM[:, 1], q / A.LIM[:, 0])
out = dict(arm=i, foot_N=Ff, q_deg=np.degrees(q).round(1).tolist(), joints=P.round(4).tolist(), spigot=spig.round(4).tolist(), foot=tip.round(4).tolist(),
           reach=round(float(np.linalg.norm(tip - root) / A.L_FULL), 3), joint_range=round(float(rng_frac.max()), 3),
           tau_x_rated=(np.abs(tau) / A.RATED).round(2).tolist(), x_rated=round(float(np.max(np.abs(tau) / A.RATED)), 2),
           x_stall=round(float(np.max(np.abs(tau) / A.STALL)), 2))
json.dump(out, open(f"arm_leg_{i}_{int(round(Ff))}{'_r%d' % A.ROLL if A.ROLL else ''}{'_h%d' % round(A.HC * 1000) if abs(A.HC - 0.38) > 1e-6 else ''}.json", "w"))
print(json.dumps({k: v for k, v in out.items() if k not in ("joints", "spigot")}))
