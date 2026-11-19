"""Clearance of the two hand3 drums (r 55 mm x 110 mm on arms 2 and 6) to the other arms, the EDF and the mantle plate over a run."""
import os, sys, numpy as np
os.environ.update(CEPH_PROP="xl", CEPH_PACKS="2", CEPH_MONITOR="0")
import cephalo4 as C, xlpod4; xlpod4.install()
import states4 as ST
Z = np.load(sys.argv[1], allow_pickle=True); R_H = 0.055
def seg_pt(a, b, P):
    ab = b - a; s = np.clip(((P - a) @ ab) / (ab @ ab), 0, 1); return np.linalg.norm(P - (a + s[:, None] * ab), axis=1)
worst = dict(arm=(9, None), edf=(9, None), plate=(9, None), hands=(9, None))
for j in range(0, len(Z["t"]), 3):
    p, R, q, c, _ = ST.state(Z["qpos"][j]); A = {}
    for i in range(8): A[i] = ST.arm_pts(i, p, R, q[i], c[i])
    H = {}
    for h in (2, 6):
        Rt, pt = A[h][2]; H[h] = (pt, pt + Rt[:, 0] * 0.110)
        for i in range(8):
            if i == h: continue
            P, r = A[i][0], A[i][1]
            d = float((seg_pt(*H[h], P) - r - R_H).min())
            if d < worst["arm"][0]: worst["arm"] = (d, (float(Z["t"][j]), h, i))
        e = np.array([p + R @ np.array([0, 0, z]) for z in np.linspace(-0.004, -0.092, 8)])
        d = float(seg_pt(*H[h], e).min() - 0.055 - R_H)
        if d < worst["edf"][0]: worst["edf"] = (d, float(Z["t"][j]))
        ring = np.array([p + R @ np.array([0.125 * np.cos(a), 0.125 * np.sin(a), 0.0]) for a in np.linspace(0, 2 * np.pi, 48)])
        d = float(seg_pt(*H[h], ring).min() - R_H)
        if d < worst["plate"][0]: worst["plate"] = (d, float(Z["t"][j]))
    s = np.linspace(0, 1, 12)[:, None]; P6 = H[6][0] + s * (H[6][1] - H[6][0])
    d = float(seg_pt(*H[2], P6).min() - 2 * R_H)
    if d < worst["hands"][0]: worst["hands"] = (d, float(Z["t"][j]))
for k, v in worst.items(): print(f"{k:6s} min clearance {v[0] * 1000:7.1f} mm at {v[1]}")
