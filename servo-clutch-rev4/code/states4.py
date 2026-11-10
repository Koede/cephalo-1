"""Turn sim4 qpos frames into robot states (body pose, per-arm joint + clutch angles) and run the geometric checks."""
import numpy as np, mujoco
import cephalo4 as C
import sim4 as S

_M = S.build()
JA = np.array([[_M.joint(f"q{i}_{k}a").qposadr[0] for k in range(C.NJ)] for i in range(8)])
CQ = {(i, k): _M.joint(f"c{i}_{k}").qposadr[0] for i in range(8) for k in range(C.NJ) if C.CLUTCH_FRAC[k] > 0}
IMP = _M.joint("impj").qposadr[0]


def state(qpos):
    p = qpos[0:3].copy()
    R = np.zeros(9); mujoco.mju_quat2Mat(R, qpos[3:7]); R = R.reshape(3, 3)
    q = 2 * qpos[JA]
    c = np.zeros((8, C.NJ))
    for (i, k), a in CQ.items():
        c[i, k] = qpos[a]
    return p, R, q, c, qpos[IMP:IMP + 3].copy()


def arm_pts(i, p, R, q, c, n=36):
    segs, tip = C.arm_fk(i, q, p, R, clutch=c)
    ss = np.linspace(C.S_J[0] + 0.015, C.SPIGOT, n)
    P = np.array([C.at_s(segs, s)[0] for s in ss])
    r = np.interp(ss, [0, 0.2, C.SPIGOT], [0.024, 0.022, 0.017])
    Rt, pt = tip
    pod = pt + Rt[:, 0] * np.array([0.0, 0.015, 0.03])[:, None]
    return np.vstack([P, pod]), np.concatenate([r, [0.016] * 3]), tip


def check(npz):
    Z = np.load(npz)
    worst = dict(nb=9.0, body=9.0, floor=9.0)
    at = {}
    for t, qp in zip(Z["t"], Z["qpos"]):
        p, R, q, c, _ = state(qp)
        pts = [arm_pts(i, p, R, q[i], c[i]) for i in range(8)]
        for i in range(8):
            P, r, _ = pts[i]; P2, r2, _ = pts[(i + 1) % 8]
            dd = np.linalg.norm(P[:, None] - P2[None], axis=2) - r[:, None] - r2[None]
            if dd.min() < worst["nb"]: worst["nb"] = dd.min(); at["nb"] = (t, i)
            # body clearance from just past the root pivot boss (the root stub sits in the porthole)
            db = C.body_clearance((P - p) @ R, r, np.zeros(3)).min()     # in the body frame
            if db < worst["body"]: worst["body"] = db; at["body"] = (t, i)
            fl = (P[:-3, 2] - r[:-3]).min()      # arm structure (not the toes) above the floor
            if fl < worst["floor"]: worst["floor"] = fl; at["floor"] = (t, i)
    return worst, at


if __name__ == "__main__":
    import sys
    w, a = check(sys.argv[1] if len(sys.argv) > 1 else "sim4_mid.npz")
    for k in w:
        print(f"min {k} clearance {w[k]*1000:.1f} mm at t {a[k][0]:.2f} arm {a[k][1]}")
