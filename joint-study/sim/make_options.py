"""Collect the arm-length options for the main design -> results/length_options.json"""
import json, os, pandas as pd
H = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(H, "results"); A = os.path.join(H, "..", "arm")
fl = pd.read_csv(os.path.join(R, "fluid_1m.csv")); fn = pd.read_csv(os.path.join(R, "fluid_len.csv"))
def fit(d): return json.load(open(os.path.join(A, d, "fit.json")))
def w(df, **k):
    q = df
    for a, b in k.items(): q = q[q[a] == b]
    return q
def opt(key, title, length, joints, servos, thrust, cad, sc=None, fit_lab=None, src="1m"):
    f = fit(cad)
    if src == "1m":
        rows = w(fl, scenario=sc, fit=fit_lab, joints=joints)
        r25 = rows[rows.wave_hz == 0.25].iloc[0]; r50 = rows[rows.wave_hz == 0.5].iloc[0]
        wave = {"0.25": r25.wave_tip_rms_mm, "0.5": r50.wave_tip_rms_mm, "1.0": "sat"}
        base = r25
    else:
        rows = w(fn, length_m=length, fit=fit_lab, joints=joints)
        r50 = rows[rows.wave_hz == 0.5].iloc[0]; r100 = rows[rows.wave_hz == 1.0].iloc[0]
        wave = {"0.25": None, "0.5": r50.wave_tip_rms_mm, "1.0": r100.wave_tip_rms_mm}
        base = r100
    return dict(key=key, title=title, length_mm=int(length * 1000), joints=joints, servos=f["servos"], thrust=thrust,
                pitch_mm=f["pitch"], vert_g=f["m_vert"], arm_out_g=f["arm_out_g"], cad=cad,
                mass_kg=float(base.total_mass_kg), tw=float(base.tw), miss_mm=float(w(pd.read_csv(os.path.join(R, "main_design_variants_lengths.csv" if src != "1m" or sc == "today" else "main_design_variants_more_thrust.csv")), label=fit_lab, N=joints)
                      .query(f"arm_length_m=={length}").miss_mean_mm.iloc[0]),
                wave=wave, lift_mm=float(base.lift_tip_rms_mm))
O = [
 opt("a", "1 m on today's thrust", 1.0, 7, None, "today's fans (10.8 N each)", "out_1m_7", "today", "STS3250 to 35 %, STS3215 beyond"),
 opt("b", "1 m with bigger claw fans", 1.0, 13, None, "16 N claw fans, 48 % more fan thrust", "out_1m_13", "more_thrust", "STS3250 to 35 %, STS3215 beyond"),
 opt("c", "700 mm on today's thrust", 0.7, 9, None, "today's fans (10.8 N each)", "out_700_9", None, "STS3250 to 35 %, STS3215 beyond", src="len"),
]
json.dump(dict(options=O, ref_585=dict(mass_kg=10.95, tw=1.197, miss_mm=26.4, joints=9)), open(os.path.join(R, "length_options.json"), "w"), indent=1, default=float)
for o in O: print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in o.items()})
