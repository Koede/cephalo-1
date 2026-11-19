import numpy as np, fall4 as FA, mujoco, cephalo4 as C
F=FA.Fall("cut"); d=F.d
orig=F.fall_thrust; last=[0]
def ft(t):
    orig(t)
    b,l=F.floor_forces(); tot=b+l+F.toe_f
    if tot>20 and t-last[0]>0.002 and t<2.0:
        last[0]=t
        q=np.degrees(2*d.qpos[F.ja[0]]); cl=[np.degrees(d.qpos[F.cq[0,k]]) if (0,k) in F.cq else 0 for k in range(8)]
        print(f"{t:.4f} z {d.xpos[F.torso][2]:.3f} vz {d.qvel[2]:5.2f} bump {b:5.0f} link {l:5.0f} toe {F.toe_f:5.0f} | arm0 q {np.round(q,0)} cl {np.round(cl,0)}")
F.fall_thrust=ft
F.run(1.85)
