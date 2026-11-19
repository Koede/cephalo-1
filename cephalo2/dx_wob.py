import numpy as np, fall4 as FA, cephalo4 as C
F = FA.Fall('inv'); d = F.d; oc = F.control; n = [0]
def ctl(t, dt):
    oc(t, dt)
    if 9.9 <= t <= 10.75:
        n[0] += 1
        if n[0] % 6 == 0:
            cz = np.array([d.xmat[F.pod[i]].reshape(3, 3)[2, 0] for i in range(8)])
            print('t %.2f z %.3f att %.1f Mz %5.2f axy %s sat %d gate %s axis_off %s fan %s zref %.3f' % (t, d.xpos[F.torso][2], F.att_err, F.Mz_dem, np.round(F.axy, 2), F.sat, ''.join(str(int(g)) for g in F.gate), np.round(np.degrees(np.arccos(np.clip(cz, -1, 1)))).astype(int), np.round(F.fan).astype(int), F.zplan.at(t)[0]))
F.control = ctl
F.run(10.8)
