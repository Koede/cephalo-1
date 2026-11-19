import os, sys, numpy as np
import fall4 as FA, cephalo4 as C, body4 as B4
name = sys.argv[1]; arm, jt = int(sys.argv[2]), int(sys.argv[3]); t0, t1 = float(sys.argv[4]), float(sys.argv[5])
F = FA.Fall(name); m, d = F.m, F.d
orig = FA.mujoco.mj_step
rows = []
def step(m_, d_):
    orig(m_, d_)
    if t0 <= d_.time <= t1:
        qa = F.cq.get((arm, jt)); ca = np.degrees(d_.qpos[qa]) if qa is not None else 0
        cv = np.degrees(d_.qvel[F.cd[arm, jt]]) if qa is not None else 0
        rows.append((d_.time, ca, cv, np.degrees(d_.qpos[F.ja[arm, jt]]*2), np.degrees(d_.qvel[F.da[arm, jt]]*2),
                     d_.actuator_force.reshape(8, C.NJ)[arm, jt]))
FA.mujoco.mj_step = step
F.run(t1)
for r in rows[::10]: print("t %.4f clutch %6.1f deg %7.0f deg/s  servo %6.1f deg %7.0f deg/s  act %5.2f" % r)
