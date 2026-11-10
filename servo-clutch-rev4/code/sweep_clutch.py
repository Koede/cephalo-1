"""Rigid arm vs per-joint detent clutches (+ telescoping wrist) at two line tensions."""
import json, numpy as np
from sim_seats import SeatSim, CLUTCH_MODES
from sweep_seats import summarise

CASES = [("rigid", False, 0.0)] + [(k, True, v) for k, v in CLUTCH_MODES.items()]
out = {}
for v in (4.0, 6.0, 8.0):
    for name, on, T0 in CASES:
        s = SeatSim(on, T0, imp_v=v, clutch=True).run(3.0, rec=0.001)
        r = summarise(s, 0.5); out[f"hit{v:.0f}_{name}"] = r
        print(f"hit {v} {name:5s}", {k: (round(x, 2) if isinstance(x, float) else x) for k, x in r.items()})
for F in (20.0, 40.0):
    for name, on, T0 in CASES:
        s = SeatSim(on, T0, yank=(0.5, F, 0.3), clutch=True).run(2.0, rec=0.001)
        r = summarise(s, 0.5); out[f"pull{F:.0f}_{name}"] = r
        print(f"pull {F} {name:5s}", {k: (round(x, 2) if isinstance(x, float) else x) for k, x in r.items()})
json.dump(out, open("clutch_results.json", "w"), indent=1)
