"""Clearance checks on fall4 runs (python3 fall_checks.py): arm-body interpenetration and arm-arm proximity per phase
(the sim has no arm-body / arm-arm collision; arm-floor contact is simulated)."""
import numpy as np
import fall4 as FA, cephalo4 as C, states4 as ST
for c in ["cut", "hit", "inv", "drift"]:
    z = np.load(f"fall4_{c}.npz"); ph = z["phase"]; res = {}
    assert z["qpos"].shape[1] == ST._M.nq, "joint layout differs from the sim run"
    for j in range(0, len(z["t"]), 2):
        p, R, q, cl, _ = ST.state(z["qpos"][j])
        P = []
        for i in range(8):
            segs, tip = C.arm_fk(i, q[i], p, R, clutch=cl[i])
            P.append(np.array([C.at_s(segs, s)[0] for s in np.linspace(0.07, C.SPIGOT, 24)]))
        bc = min(C.body_clearance((Pi - p) @ R, np.full(len(Pi), 0.024), np.zeros(3)).min() for Pi in P)
        aa = min(np.min(np.linalg.norm(P[a][:, None] - P[b][None], axis=2)) - 0.048 for a in range(8) for b in range(a + 1, 8))
        r = res.setdefault(str(ph[j]), [9, 9]); r[0] = min(r[0], bc); r[1] = min(r[1], aa)
    print(c, " | ".join(f"{k}: body {v[0]*1000:.0f} arm {v[1]*1000:.0f}" for k, v in res.items()))
