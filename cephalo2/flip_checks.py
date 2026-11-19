"""Collision / clearance checks on a flip4 run (python3 flip_checks.py flip4.npz): arm-body, arm-arm, arm-floor
(every arm point except the toes), prop-disc gaps between spinning claw-props, per phase."""
import sys, numpy as np
import flip4 as F4, cephalo4 as C, states4 as ST
z = np.load(sys.argv[1] if len(sys.argv) > 1 else "flip4.npz")
ph = z["phase"]; R_P = 0.0953
res = {}
for j in range(0, len(z["t"]), 2):
    p, R, q, c, _ = ST.state(z["qpos"][j])
    P = []
    for i in range(8):
        segs, tip = C.arm_fk(i, q[i], p, R, clutch=c[i])
        P.append(np.array([C.at_s(segs, s)[0] for s in np.linspace(0.07, C.SPIGOT, 24)]))
    bc = min(C.body_clearance((Pi - p) @ R, np.full(len(Pi), 0.024), np.zeros(3)).min() for Pi in P)
    aa = min(np.min(np.linalg.norm(P[a][:, None] - P[b][None], axis=2)) - 0.048 for a in range(8) for b in range(a + 1, 8))
    fl = min(Pi[:, 2].min() - 0.024 for Pi in P)
    on = [i for i in range(8) if z["fan"][j][i] > 1.0]
    H = []
    for i in on:
        segs, (Rt, pt) = C.arm_fk(i, q[i], p, R, clutch=c[i]); H.append((pt + Rt[:, 0] * C.BLADE_OFF, Rt[:, 0]))
    dg = 9.0
    for a in range(len(H)):
        for b in range(a + 1, len(H)):
            if abs(H[a][0][2] - H[b][0][2]) < 0.1: dg = min(dg, np.hypot(*(H[a][0][:2] - H[b][0][:2])) - 2 * R_P)
    dome = (p + R @ np.array([0, 0, 0.156]))[2]
    r = res.setdefault(str(ph[j]), dict(body=9, arm=9, floor=9, disc=9, dome=9))
    r["body"] = min(r["body"], bc); r["arm"] = min(r["arm"], aa); r["floor"] = min(r["floor"], fl); r["disc"] = min(r["disc"], dg); r["dome"] = min(r["dome"], dome)
print("phase      arm-body  arm-arm  arm-floor  prop-disc gap  dome crown height (mm)")
for k, r in res.items():
    print(f"{k:9s} {r['body']*1000:8.0f} {r['arm']*1000:8.0f} {r['floor']*1000:10.0f} {r['disc']*1000 if r['disc']<9 else float('nan'):12.0f} {r['dome']*1000:10.0f}")
