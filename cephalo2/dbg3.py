import os; os.environ["FLIP_START"]="stand"; os.environ["FLIP_WALK"]="0.18,0,0"
import numpy as np, mujoco, flip4 as F4
F4.T_END=5.0
F=F4.Flip(); F.start_inverted(); F.run_flip()
m,d=F.m,F.d
out=np.zeros(6)
for j in range(d.ncon):
    c=d.contact[j]; g=c.geom2 if c.geom1 in F.ground_ids else c.geom1
    if g in F.toe:
        mujoco.mj_contactForce(m,d,j,out); fw=c.frame.reshape(3,3).T@out[:3]
        print('arm',F.toe[g],'force world',fw.round(1))
qa=2*d.qpos[F.ja[3]]; print('arm3 q',np.degrees(qa).round(1)); print('cmd',np.degrees(F.q_cmd[3]).round(1)); print('load',F.load[3].round(2))
for (i,k),qa_ in F.cq.items():
    if i==3: print('clutch',k,round(np.degrees(d.qpos[qa_]),1))
print('qfrc_constraint arm3 j4', d.qfrc_constraint[F.da[3,4]].round(2) if hasattr(d,'qfrc_constraint') else '')
