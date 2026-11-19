import json, sys, numpy as np
import hand3_thrust as TH
out = {}
for src, key in (("nf_trip", "low"), ("nf_turb", "high")):
    name = "D thin airfoil finger"; P = TH.PLANS[name]; cl, cd, conf = TH.polar_tables(name, src); prot = TH.protuberances(name)
    run = lambda rpm: TH.bemt(rpm, P["chord"], lambda r: P["pitch"], cl, cd, prot=prot, gaps=TH.GAPS)
    rf = TH.full_throttle(run); b = run(rf); T = TH.net_thrust(b)
    out[key] = dict(T_max=float(T), rpm=float(rf), P_shaft=float(b["P"]), qT=float(b["P"] / (rf * 2 * np.pi / 60) / T))
    for Tw in (11.0, 13.9):
        rp = TH.rpm_for(run, Tw); bb = run(rp); out[key][f"hover{Tw}"] = dict(rpm=float(rp), P_el=float(bb["P"] / 0.8))
    print(key, {k: (round(v, 4) if isinstance(v, float) else v) for k, v in out[key].items()}, flush=True)
json.dump(out, open(sys.argv[1], "w"), indent=1)
