"""Evaluate the main design (pin hinge, 60 deg helix) over joint counts and servo fits at config.ARM_LENGTH.

  python3 eval_main.py            # static checks + MuJoCo for the feasible fits
  python3 eval_main.py --static   # static checks only (fast)
Writes results/main_design_variants.csv and folds the rows into results/main_design.json.
"""
import sys, json, os, argparse, numpy as np, pandas as pd
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sweep as S, rank as R, config as C

FITS = {   # label -> actuation settings; STS3250 shares the STS3215 case, STS3032 needs a smaller tip cradle
    "STS3215 throughout": dict(actuation="local", servo="STS3215_12V"),
    "STS3250 to 35 %, STS3215 beyond": dict(actuation="graded", servo="STS3215_12V", root_servo="STS3250_12V", split="long_mid"),
    "STS3250 to 25 %, STS3215 to 60 %, STS3032 beyond": dict(actuation="graded", servo="STS3032", root_servo="STS3250_12V", split="short_tip"),
    "STS3250 to 35 %, STS3215 to 85 %, STS3032 beyond": dict(actuation="graded", servo="STS3032", root_servo="STS3250_12V", split="long_mid"),
    "STS3250 throughout": dict(actuation="local", servo="STS3250_12V"),
}
def joint_counts(L):
    nmax = int(L // 0.0605)   # shortest pitch that clears in pin_cad.py: 60.5 mm
    return sorted({n for n in (7, 9, 11, 13) if n <= nmax} | {nmax})

def one(args):
    name, n, L = args; C.ARM_LENGTH = L; v = FITS[name]
    cfg = dict(family="pin_hinge", variant="M3_bolt_PETG", range_deg=90, arrangement="helix60", n_nodes=n, **v)
    job = dict(cfg=cfg, combos=[("M3_bolt_PETG", 90, v["actuation"], None, "rigid")])
    row = S.run_job(job)[0]; row["row_id"] = 0; row["label"] = name; row["joints_req"] = n; row["arm_length_m"] = L
    return row

def dyn(row):
    C.ARM_LENGTH = row.get("arm_length_m", C.ARM_LENGTH); d = R._dyn(dict(row)); out = dict(row); out.update(d); return out

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--static", action="store_true"); ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--lengths", type=float, nargs="*", default=[C.ARM_LENGTH], help="arm lengths in m")
    ap.add_argument("--tag", default="", help="suffix for the output CSV; the untagged run also updates main_design.json")
    a = ap.parse_args()
    with Pool(a.workers) as pool:
        rows = pool.map(one, [(nm, n, L) for L in a.lengths for nm in FITS for n in joint_counts(L)])
        df = pd.DataFrame(rows)
        if not a.static:
            idx = list(df.index[df.feasible])
            for i, r in zip(idx, pool.map(dyn, [df.loc[i].to_dict() for i in idx])):
                for k, v in r.items(): df.loc[i, k] = v
    if "wave_tip_rms_mm" in df:
        sd = ((np.exp(-df.wave_tip_rms_mm / 50) * np.exp(-df.lift_tip_rms_mm / 30) * (1 - 0.5 * df.wave_sat_frac) *
               np.minimum(1, 1 / df.step_settle_s.clip(lower=0.05)) * np.exp(-df.sag_mm / 60)) ** 0.2)
        df["s_dynamics"] = sd; df["final_score"] = df.score * sd ** 0.5
    cols = [c for c in ["label", "arm_length_m", "joints_req", "N", "dof", "feasible", "fail", "miss_mean_mm", "total_mass_kg", "thrust_to_weight", "util_walk",
            "util_flight_max", "wave_tip_rms_mm", "wave_sat_frac", "lift_tip_rms_mm", "sag_mm", "touchdown_v_safe", "cost_usd", "score",
            "final_score"] if c in df]
    df[cols].to_csv(os.path.join(HERE, "results", f"main_design_variants{a.tag}.csv"), index=False)
    pd.set_option("display.width", 300); print(df[cols].round(3).to_string())
    mp = os.path.join(HERE, "results", "main_design.json")
    if os.path.exists(mp) and not a.static and not a.tag:
        md = json.load(open(mp)); md["variants"] = json.loads(df[cols].to_json(orient="records")); json.dump(md, open(mp, "w"), indent=1)
