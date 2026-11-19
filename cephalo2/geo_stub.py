
import runpy, numpy as np
ns = runpy.run_path('quad4r.py', run_name='notmain'); B4 = ns['B4']; C = ns['C']
for i in (1, 3, 5, 7, 0, 4):
    cs, ax, R, p = B4.fk(i, B4.Q_FLY, np.zeros(3), np.eye(3)); c = p + R[:, 0] * B4.BO[i]
    print(i, 'disc centre', np.round(c, 3), 'r_xy', round(float(np.hypot(c[0], c[1])), 3), 'axis', np.round(R[:, 0], 2), 'R', C.CLAW_R)
