import numpy as np, fall4 as FA, mujoco, cephalo4 as C
F=FA.Fall("hit"); m,d=F.m,F.d
st={"qv":d.qvel.copy()}; orig=F.fall_thrust
def ft(t):
    orig(t)
    qa=(d.qvel-st["qv"])/0.0004; st["qv"]=d.qvel.copy()
    if 1.775<t<1.81:
        a,k=2,4; j=F.da[a,k]
        g=(abs(d.actuator_force.reshape(8,8)[a,k])+C.ARMATURE[k]*abs(qa[j])*2)/C.GEAR_LIMIT[k]
        if g>1.0:
            jid=m.joint(f'q{a}_{k}a').id
            print(f"{t:.4f} gear {g:.2f} qa {np.degrees(2*d.qpos[F.ja[a,k]]):6.1f} range {np.degrees(m.jnt_range[jid]*2).round(0)} clutch {np.degrees(d.qpos[F.cq[a,k]]):5.1f} act {d.actuator_force.reshape(8,8)[a,k]/C.STALL[k]:5.2f} qacc {qa[j]:9.0f} cfrc {d.qfrc_constraint[j]:7.2f}")
F.fall_thrust=ft
F.run(1.82)
