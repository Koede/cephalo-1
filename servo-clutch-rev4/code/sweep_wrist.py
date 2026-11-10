"""Hit and yank sweeps for the two wrists (sim_wrist.py). Writes sweep_results.json and prints a table."""
import json, numpy as np
from sim_wrist import Sim

TOOTH_R = 0.010          # m1, 20T printed spur gear: pitch radius 10 mm
LEWIS_N = 48.0           # allowable tooth force: 30 MPa (PETG, est.) x 5 mm face x m1 x Y 0.32
res = {"hit": [], "yank": []}

for v in (2.0, 4.0, 6.0):
    for var, Fc in (("knuckle", None), ("magnet", 15.0), ("magnet", 30.0), ("magnet", 60.0)):
        s = Sim(var, Fc=Fc or 30.0, imp_v=v, t_hit=0.5).run(3.5, rec_dt=0.001)
        L = [l for _, l, _ in s.log]; T = np.array([t for t, _, _ in s.log])
        pk = {k: float(max(l[k] for l in L)) for k in L[0]}
        tilt = np.array([l["tilt_deg"] for l in L])
        after = np.where((T > 0.55) & (tilt > 2))[0]
        r = dict(variant=var, Fc=Fc, v=v, peak=pk, settle_s=float(T[after[-1]] - 0.5) if len(after) else 0.0)
        from sim_wrist import FLIGHT
        dev = np.degrees(np.abs(s.log[-1][2]["q"] - FLIGHT))
        maxdev = np.degrees(np.max([np.abs(sn["q"] - FLIGHT) for _, _, sn in s.log], 0))
        r["end_joint_dev_deg"] = dev.round(1).tolist(); r["max_joint_dev_deg"] = maxdev.round(1).tolist()
        r["arm_stuck"] = bool(dev.max() > 20)
        if var == "knuckle":
            sat = np.array([l["pitch_servo"] > 0.43 or l["roll_servo"] > 0.43 for l in L])
            r["servo_sat_ms"] = float(sat.sum())
            r["tooth_N"] = pk["gear_Nm"] / TOOTH_R; r["tooth_vs_allow"] = r["tooth_N"] / LEWIS_N
        else:
            r["detached"] = s.detached_at is not None
            r["reseated_after_s"] = (s.reseated_at - s.detached_at) if s.reseated_at else None
        res["hit"].append(r)
        print("hit", var, Fc, v, {k: round(x, 2) for k, x in pk.items()}, {k: r[k] for k in r if k not in ("peak", "end_joint_dev_deg")})

for var, Fc in (("knuckle", None), ("magnet", 15.0), ("magnet", 30.0), ("magnet", 60.0)):
    s = Sim(var, Fc=Fc or 30.0, imp_v=0, yank=(0.5, 25.0)).run(4.5, rec_dt=0.002)   # pull ramps up at 25 N/s
    first = {}
    for t, l, _ in s.log:
        F = max(0, (t - 0.5) * 25)
        for k, lim in (("j0", 4.85), ("j8", 0.43), ("pitch_servo", 0.43), ("roll_servo", 0.43)):
            if k in l and l[k] > lim and k not in first:
                first[k] = round(F, 1)
        if var == "magnet" and l["gap_mm"] > 4 and "detach" not in first:
            first["detach"] = round(F, 1)
    pk = {k: float(max(l[k] for _, l, _ in s.log)) for k in s.log[0][1]}
    res["yank"].append(dict(variant=var, Fc=Fc, first_limit_at_N=first, peak=pk))
    print("yank", var, Fc, first, {k: round(x, 1) for k, x in pk.items()})

json.dump(res, open("sweep_results.json", "w"), indent=1)
