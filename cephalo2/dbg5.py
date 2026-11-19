import os, numpy as np, flip4 as F4
F=F4.Flip()
if os.environ.get("FLIP_START")=="stand": F.start_inverted()
orig=F.land_or_walk
def low(t,dt):
    ph=F.phase; orig(t,dt)
    if ph=="stand" and F.phase=="walk":
        d=F.d
        q=2*d.qpos[F.ja]; print("max |q - qcmd| deg", np.degrees(np.abs(q-F.q_cmd)).max().round(2), "max qd_cmd", np.degrees(np.abs(F.qd_cmd)).max().round(2))
        cl=[np.degrees(d.qpos[a]) for a in F.cq.values()]; print("clutch max deg", np.abs(cl).max().round(2))
        print("load max", F.load.max().round(2), "load_f", round(F.load_f,2), "rate", round(F.rate,2), "T_line", round(F.T_line,1), "sat", F.sat)
        print("qvel max", np.abs(d.qvel).max().round(3), "body ang vel", d.qvel[3:6].round(3))
        print("gz", F.gz, "dz", F.dz)
        imp=d.qpos[F.imp_q:F.imp_q+3]; print("imp pos", imp.round(2))
F.land_or_walk=low
F4.T_END=float(os.environ.get("T","21"))
F.run_flip()
