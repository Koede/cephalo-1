import numpy as np, fall4 as FA, mujoco, cephalo4 as C
F=FA.Fall("cut"); d=F.d; m=F.m
orig=F.fall_thrust; st={"qv":d.qvel.copy()}
def ft(t):
    orig(t)
    qa=(d.qvel-st["qv"])/0.0004; st["qv"]=d.qvel.copy()
    if 1.70<t<1.95 and int(round(t/0.0004))%10==0:
        out=[]
        for a in (0,):
            j=F.da[a,0]; g=(abs(d.actuator_force.reshape(8,8)[a,0])+C.ARMATURE[0]*abs(qa[j])*2)/C.GEAR_LIMIT[0]
            print(f"{t:.4f} q0 {np.degrees(2*d.qpos[F.ja[a,0]]):6.1f} cmd {np.degrees(F.q_cmd[a,0]):6.1f} clutch {np.degrees(d.qpos[F.cq[a,0]]):5.1f} act {d.actuator_force.reshape(8,8)[a,0]/C.STALL[0]:5.2f} qacc {qa[j]:8.0f} qvel {d.qvel[j]:6.2f} gear {g:.2f} lim {np.degrees(m.jnt_range[m.joint(f'q{a}_0a').id]).round(1)}")
F.fall_thrust=ft
F.run(1.96)
