import numpy as np, flip4 as F4, mujoco
F=F4.Flip(); d=F.d
orig=F.flip_thrust
def ft(t):
    orig(t)
    if 2.3<t<2.6 and abs((t*40)-round(t*40))<0.005:
        cz=[d.xmat[F.pod[i]].reshape(3,3)[2,0] for i in range(8)]
        print(f"{t:.3f} cz {np.round(cz,2)} gate_s {F.gate_s.round(2)} fan {F.fan.round(1)} att {F.att_err:.1f} sat {F.sat} zr {F.zplan.at(t)[0]:.2f} z {d.xpos[F.torso][2]:.2f}")
F.flip_thrust=ft
F4.T_END=2.6
F.run_flip()
