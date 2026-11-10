"""Tuned configuration: j5 and j7 as STS3215, detent clutch on every joint at 0.9 x stall (firm 60 N), +-60 deg travel."""
import json, numpy as np
import sim_seats as SS, sim_wrist as SW
from sweep_seats import summarise


def config(j7_upgrade, frac, travel):
    st = [4.9] * 3 + [2.9] * 3 + [0.44] + ([2.9] if j7_upgrade else [0.44]) + [0.44]
    SW.STALL = st
    SW.SERVO_M = [0.076] * 3 + [0.055] * 3 + [0.020] + ([0.055] if j7_upgrade else [0.020]) + [0.020]
    SW.BRACKET = 0.647 - (3 * 0.076 + 2 * 0.055 + 4 * 0.020)
    SS.ARMATURE = [0.02 if s > 4 else 0.012 if s > 2 else 0.003 for s in st]
    SS.GEAR_STRIP = [3 * s for s in st]
    SS.C_CLUTCH = [frac * s / 60.0 for s in st]
    SS.CLUTCH_RANGE = travel


if __name__ == "__main__":
    out = {}
    for label, j7, frac, travel in (("rigid_j5", False, 0.9, 45), ("rigid_j5j7", True, 0.9, 60), ("clutch_j5j7", True, 0.9, 60)):
        config(j7, frac, travel)
        on = label.startswith("clutch")
        for v in (4.0, 6.0, 8.0):
            s = SS.SeatSim(on, 60.0, imp_v=v, clutch=True).run(3.0, rec=0.001)
            r = summarise(s, 0.5); out[f"{label}_hit{v:.0f}"] = r
            print(label, "hit", v, "gear", r["gear_peak"], "worst", round(r["gear_worst"], 2), "recover", round(r["recover_s"], 2), "stuck", r["stuck"], "clutch", round(r["open_max"], 1))
        for F in (20.0, 40.0):
            s = SS.SeatSim(on, 60.0, yank=(0.5, F, 0.3), clutch=True).run(2.0, rec=0.001)
            r = summarise(s, 0.5); out[f"{label}_pull{F:.0f}"] = r
            print(label, "pull", F, "gear worst", round(r["gear_worst"], 2), "servo worst", round(r["worst_ratio"], 2), "ext", round(r["ext_max"], 1), "recover", round(r["recover_s"], 2))
    json.dump(out, open("tuned_results.json", "w"), indent=1)
