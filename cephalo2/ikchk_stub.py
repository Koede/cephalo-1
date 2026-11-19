
import runpy, numpy as np, os
ns = runpy.run_path('quad4r.py', run_name='notmain')
Q = ns['Quad'](); C = ns['C']; ik_g = ns['ik_g']; err = ns['ik_g_err']; AXB = ns['AXB']; UP = ns['UPREF']; Z0 = ns['Z0']
bp = np.array([0, 0, Z0]); q = {i: Q.q_ik[i].copy() for i in ns['WORK']}
print('start', {i: (np.round(np.degrees(q[i])), np.round(np.degrees(C.LIM[:,1]))) for i in q})
for wg in [float(x) for x in os.environ.get('WG','3').split(',')]:
    q = {i: Q.q_ik[i].copy() for i in ns['WORK']}; worst = {i: [0, 0, 0] for i in q}
    for t in np.arange(0, 13, 0.004):
        T = Q.hand_targets(t)[0]
        for i in q:
            q[i] = ik_g(i, T[i] + bp, AXB[i], q[i], q[i], bp, np.eye(3), Q.lead[i], UP.get(i), w_g=wg)
            e = err(i, q[i], T[i] + bp, AXB[i], bp, np.eye(3), Q.lead[i])
            lim = np.abs(q[i] / np.abs(C.LIM).max(1)).max()
            worst[i] = [max(worst[i][0], e[0] * 1e3), max(worst[i][1], e[1]), max(worst[i][2], lim)]
    print('w_g', wg, {i: np.round(v, 2) for i, v in worst.items()}, '(mm, deg, joint/limit)')
