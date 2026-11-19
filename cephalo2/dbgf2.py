import numpy as np, fall4 as FA, mujoco, cephalo4 as C
F=FA.Fall("cut"); d=F.d
rec=[]; orig=F.fall_thrust
def ft(t):
    orig(t)
    if 1.64<t<1.72 and int(round(t/0.0004))%5==0:
        q=np.degrees(2*d.qpos[F.ja[0]]); cl=[np.degrees(d.qpos[F.cq[0,k]]) if (0,k) in F.cq else 0 for k in range(8)]
        fa=d.actuator_force.reshape(8,8)[0]/C.STALL
        print(f"{t:.4f} z {d.xpos[F.torso][2]:.3f} vz {d.qvel[2]:5.2f} q {np.round(q,0)} cl {np.round(cl,0)} act {np.round(fa,2)} foot {F.foot_f[0]:.0f}")
F.fall_thrust=ft
F.run(1.73)
