import numpy as np, hand3_thrust as TH, hand3_design as H
name = "D thin airfoil finger"; P = TH.PLANS[name]; prot = TH.protuberances(name)
tabs = {k: TH.polar_tables(name, s) for k, s in (("low", "nf_trip"), ("high", "nf_turb"))}
for cs in (1.0, 1.15):
    for pitch in (12, 14, 16, 18, 20):
        row = []
        for k in ("low", "high"):
            cl, cd, _ = tabs[k]
            run = lambda rpm: TH.bemt(rpm, lambda r: P["chord"](r) * cs, lambda r: pitch, cl, cd, prot=prot, gaps=TH.GAPS)
            rf = TH.full_throttle(run); b = run(rf); T = TH.net_thrust(b); rh = TH.rpm_for(run, 11.0); bh = run(rh)
            row.append(f"{k} T {T:5.1f} N rpm {rf:6.0f} P {b['P']:4.0f} W | hover 11 N: {bh['P'] / 0.8:4.0f} W el")
        print(f"chord x{cs} pitch {pitch}: " + "  ||  ".join(row), flush=True)
