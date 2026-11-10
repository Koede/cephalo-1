"""Dynamics of the main design at shorter arm lengths on today's thrust -> results/fluid_len.csv"""
import os, sys, itertools, pandas as pd
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
CASES = [(0.7, "STS3215 throughout", 9), (0.7, "STS3250 to 35 %, STS3215 beyond", 9), (0.8, "STS3215 throughout", 9), (0.8, "STS3250 to 35 %, STS3215 beyond", 7)]
def run(a):
    (L, fit, n), hz = a
    os.environ["WAVE_HZ"] = str(hz)
    import eval_main as E
    row = E.one((fit, n, L)); out = E.dyn(row)
    return dict(length_m=L, fit=fit, joints=n, wave_hz=hz, feasible=out["feasible"], fail=out.get("fail"), miss_mean_mm=out["miss_mean_mm"],
                wave_tip_rms_mm=out.get("wave_tip_rms_mm"), wave_sat_frac=out.get("wave_sat_frac"), lift_tip_rms_mm=out.get("lift_tip_rms_mm"),
                total_mass_kg=out["total_mass_kg"], tw=out["thrust_to_weight"], util_flight_max=out["util_flight_max"])
if __name__ == "__main__":
    with Pool(4, maxtasksperchild=1) as p: rows = p.map(run, list(itertools.product(CASES, [1.0, 0.5])), chunksize=1)
    df = pd.DataFrame(rows); df.to_csv(os.path.join(HERE, "results", "fluid_len.csv"), index=False)
    pd.set_option("display.width", 250); print(df.round(3).to_string())
