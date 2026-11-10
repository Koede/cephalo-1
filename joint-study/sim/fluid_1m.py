"""How fast can the 1 m main design move? Wave test at lower frequencies for the main candidates.
  python3 fluid_1m.py   -> results/fluid_1m.csv"""
import os, sys, itertools, pandas as pd
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
CASES = [("today", "STS3215 throughout", 7), ("today", "STS3250 to 35 %, STS3215 beyond", 7),
         ("more_thrust", "STS3215 throughout", 11), ("more_thrust", "STS3215 throughout", 13),
         ("more_thrust", "STS3250 throughout", 11), ("more_thrust", "STS3250 to 35 %, STS3215 beyond", 13)]
def run(a):
    (scen, fit, n), hz = a
    os.environ["WAVE_HZ"] = str(hz)
    if scen == "more_thrust": os.environ["JOINT_SCENARIO"] = "more_thrust"
    import eval_main as E, config as C
    row = E.one((fit, n, C.ARM_LENGTH)); out = E.dyn(row)
    return dict(scenario=scen, fit=fit, joints=n, wave_hz=hz, feasible=out["feasible"], wave_tip_rms_mm=out.get("wave_tip_rms_mm"),
                wave_sat_frac=out.get("wave_sat_frac"), lift_tip_rms_mm=out.get("lift_tip_rms_mm"), sag_mm=out.get("sag_mm"),
                step_settle_s=out.get("step_settle_s"), total_mass_kg=out["total_mass_kg"], tw=out["thrust_to_weight"])
if __name__ == "__main__":
    jobs = list(itertools.product(CASES, [0.25, 0.5]))
    with Pool(4, maxtasksperchild=1) as p: rows = p.map(run, jobs, chunksize=1)
    df = pd.DataFrame(rows); df.to_csv(os.path.join(HERE, "results", "fluid_1m.csv"), index=False)
    pd.set_option("display.width", 250); print(df.round(3).to_string())
