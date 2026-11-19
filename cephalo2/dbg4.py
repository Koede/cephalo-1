import os, sys, numpy as np, flip4 as F4
F=F4.Flip()
if os.environ.get("FLIP_START")=="stand": F.start_inverted()
orig=F.land_or_walk; st={}
def low(t,dt):
    ph=F.phase; orig(t,dt)
    if ph=="stand" and F.phase=="walk":
        fm=F.measured_feet(); print("walk start: target-measured feet (mm)", (np.linalg.norm(F.feet-fm,axis=1)*1000).round(1), "bp", F.bp.round(3), "yaw", np.degrees(F.byaw).round(2))
        print("measured feet r", np.hypot(*(fm[:,:2]-F.bp[:2]).T).round(3))
F.land_or_walk=low
F4.T_END=float(os.environ.get("T","21"))
F.run_flip()
