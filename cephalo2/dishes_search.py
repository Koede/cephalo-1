"""Find a pan place (body frame) where hand 2 can hold the pan from below (palm up, pan on the heel) and hand 6 can scrub from above."""
import os, sys, runpy, json, numpy as np
os.environ.update(CEPH_MONITOR="0", H3_FINGERS="1", QUAD_FILE="dishes_search_stub.py", H3_SPIN="opp", Q_TASK="shovel")
open("dishes_search_stub.py","w").write('''
import runpy, numpy as np, os, json
ns = runpy.run_path('quad4r.py', run_name='notmain')
C = ns['C']; ik_g = ns['ik_g']; err = ns['ik_g_err']; Z0 = ns['Z0']; az = ns['az']; HM = ns['HM']; LIMA = ns['LIMA']
bp = np.array([0, 0, Z0]); UP = np.array([0, 0, 1.0]); DN = -UP; I3 = np.eye(3)
def solve(i, tg, a, lead, up, n=8, w_ax=3.0, eu=True):
    best = None; rng = np.random.default_rng(i)
    for s in range(n):
        q = np.radians([-40, 0, 40, 0, 20, 0, 0, 0]) + (rng.normal(0, 0.6, 8) if s else 0)
        for _ in range(150): q = ik_g(i, tg + bp, a, q, q, bp, I3, lead, up, w_reg=0.002, w_ax=w_ax)
        e = err(i, q, tg + bp, a, bp, I3, lead, up if eu else None); w = np.abs(q / LIMA).max() + 20 * e[0] + 0.1 * e[1]
        if best is None or w < best[0]: best = (w, q)
    return best
rng = np.random.default_rng(3); res = []; GRIP = 0.14
for n in range(int(os.environ.get('N', '300'))):
    P = rng.uniform(0.14, 0.32) * az(rng.uniform(0, 360)) + np.array([0, 0, rng.uniform(-0.48, -0.15)])
    psi = rng.uniform(0, 360); phi = rng.uniform(0, 180); hb = 0.06; hd = az(psi)          # handle points from the pan centre along hd
    G = P + GRIP * hd
    if np.hypot(G[0], G[1]) < 0.10: continue
    w2, q2 = solve(2, G, hd, HM.ZHEEL + 0.0125, DN, n=5)
    if w2 > 1.0: continue
    a = az(phi); w6, q6 = solve(6, P + np.array([0, 0, hb]), a, HM.ZHEEL + 0.0125, DN, n=6, w_ax=0.4, eu=False)
    q = q6.copy()
    for th in np.linspace(0, 2 * np.pi, 24):
        tg = P + np.array([0, 0, hb]) + 0.035 * np.array([np.cos(th), np.sin(th), 0])
        for _ in range(4): q = ik_g(6, tg + bp, a, q, q, bp, I3, HM.ZHEEL + 0.0125, DN, w_ax=0.4)
        e = err(6, q, tg + bp, a, bp, I3, HM.ZHEEL + 0.0125); uz = ns['B4'].fk(6, q, bp, I3)[2][2, 0]
        w6 = max(w6, np.abs(q / LIMA).max() + 20 * e[0] + 0.1 * e[1] + (2.0 if uz > -0.6 else 0.0))
    res.append((max(w2, w6), w2, w6, P.tolist(), psi, phi, q2.tolist(), q6.tolist()))
    print(n, round(max(w2, w6), 3), round(w2, 3), round(w6, 3), np.round(P, 3), round(psi), round(phi), flush=True)
res.sort(key=lambda r: r[0]); json.dump(res[:5], open('dishes_search.json', 'w'), indent=1)
print('BEST', res[0][:6])
''')
sys.argv=["quad_h3.py","low","quad4_g.json"]
runpy.run_path("quad_h3.py", run_name="__main__")
