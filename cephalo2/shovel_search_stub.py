
import runpy, numpy as np, os, json
ns = runpy.run_path('quad4r.py', run_name='notmain')
C = ns['C']; ik_g = ns['ik_g']; err = ns['ik_g_err']; Z0 = ns['Z0']; az = ns['az']; kf = ns['kf']; HM = ns['HM']; B4 = ns['B4']
bp = np.array([0, 0, Z0]); lead = HM.ZHEEL + 0.0145; LIMA = np.abs(C.LIM).max(1)
def offs(out_az):
    down = np.array([0, 0, -0.08]); out = 0.05 * az(out_az); up = np.array([0, 0, 0.06]); side = 0.08 * az(out_az + 90)
    keys = [(0.0, np.zeros(3)), (1.2, down), (2.0, down + out), (2.9, up + out), (3.5, up + side), (4.2, np.zeros(3))]
    return [kf(c, keys) for c in np.arange(0, 4.2, 0.07)]
def score(U, L):
    ex = (L - U) / np.linalg.norm(L - U); haz = np.degrees(np.arctan2(ex[1], ex[0])); O = offs(haz + 66)
    tot = 0; qs = {}
    for i, G in ((2, U), (6, L)):
        best = None; rng = np.random.default_rng(i)
        for s in range(8):
            q = np.radians([-40, 0, 40, 0, 20, 0, 0, 0]) + (rng.normal(0, 0.5, 8) if s else 0)
            for _ in range(120): q = ik_g(i, G + bp, ex, q, q, bp, np.eye(3), lead, w_reg=0.002)
            q0 = q.copy(); w = 0
            for o in O:
                for _ in range(4): q = ik_g(i, G + o + bp, ex, q, q, bp, np.eye(3), lead)
                e = err(i, q, G + o + bp, ex, bp, np.eye(3), lead)
                w = max(w, np.abs(q / LIMA).max() + 20 * e[0] + 0.1 * e[1])
            if best is None or w < best[0]: best = (w, q0)
        tot = max(tot, best[0]); qs[i] = best[1]
    return tot, qs, haz
ex_z = lambda U, L: ((L - U) / np.linalg.norm(L - U))[2]
rng = np.random.default_rng(2); res = []
for n in range(int(os.environ.get('N', '150'))):
    U = rng.uniform(0.20, 0.50) * az(90 + rng.uniform(-60, 60)) + np.array([0, 0, rng.uniform(-0.45, -0.05)])
    L = rng.uniform(0.20, 0.50) * az(270 + rng.uniform(-60, 60)) + np.array([0, 0, rng.uniform(-0.60, -0.20)])
    Ls = np.linalg.norm(L - U)
    if not 0.30 <= Ls <= 0.50 or L[2] > U[2] - 0.08 or L[2] + ex_z(U, L) * 0.38 + Z0 < 0.08: continue
    ex = (L - U) / Ls; s = np.linspace(-0.06, Ls + 0.38, 60); P = U + s[:, None] * ex
    if np.min(np.linalg.norm(P[:, :2], axis=1)[(P[:, 2] > -0.45)]) < 0.085: continue           # shaft clear of the EDF and its jet
    sc, qs, haz = score(U, L); res.append((sc, U.tolist(), L.tolist(), haz, {str(k): v.tolist() for k, v in qs.items()}))
    print(n, round(sc, 3), np.round(U, 3), np.round(L, 3), flush=True)
res.sort(key=lambda r: r[0]); json.dump(res[:5], open('shovel_search.json', 'w'), indent=1)
print('BEST', res[0][0], res[0][1], res[0][2], res[0][3])
