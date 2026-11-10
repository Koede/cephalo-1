"""Fold the wider hover searches (hover_tilt_*.json) into poses_alt90.npz and results/flight_summary.json."""
import json, glob, os, math, numpy as np, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spec_lib as S
H = os.path.join(S.HERE, "results"); P = dict(np.load(os.path.join(H, "poses_alt90.npz")))
L = json.load(open(os.path.join(H, "loads_alt90.json")))
names = {22: "hover", 25: "rated", 33: "full"}
rows = {}
for fn in glob.glob(os.path.join(H, "hover_tilt_*_T*.json")):
    for r in json.load(open(fn)):
        T = int(fn.split("_T")[-1].split(".")[0]); lean = int(r["lean_allowed_deg"])
        key = f"lean{lean}_{names[T]}"
        old = rows.get(key)
        if r["ok"] and (old is None or r["worst"] < old["worst"]): rows[key] = dict(r, T_N={22: 21.53, 25: 25.10, 33: 33.47}[T])
# the earlier 3-deg searches (sec2) count too: keep whichever is lower
for nm in ("hover", "rated", "full"):
    d = L["flight"][nm]; k = f"lean3_{nm}"
    if k not in rows or d["worst"] < rows[k]["worst"]:
        rows[k] = dict(lean_allowed_deg=3, worst=d["worst"], need=d["need"], q_deg=d["q_deg"], lift_lost=0.001, T_N=d["T_N"], ok=d["ok"])
A = S.Arm("alt90")
for lean in (3, 10, 20):
    h = rows.get(f"lean{lean}_hover")
    if not h: continue
    q = np.radians(h["q_deg"]); f = A.fk(q)
    for nm, T in (("rated", 25.10), ("full", 33.47)):
        n = A.need(q, T * f["tipR"][:, 2])[0]; k = f"lean{lean}_{nm}"
        if k not in rows or float(n.max()) < rows[k]["worst"]:
            rows[k] = dict(h, T_N=T, worst=round(float(n.max()), 3), need=[round(float(x), 2) for x in n], note="the hover pose, same angles")
for k, r in rows.items():
    P[f"alt90_{k}"] = np.radians(r["q_deg"])
# the spec's reference hover (fan upright) is the best 3-deg hover
P["alt90_flight_hover"] = P["alt90_lean3_hover"]; P["alt90_flight_rated"] = P["alt90_lean3_rated"]; P["alt90_flight_full"] = P["alt90_lean3_full"]
np.savez(os.path.join(H, "poses_alt90.npz"), **P)
json.dump(rows, open(os.path.join(H, "flight_summary.json"), "w"), indent=1)
for k in sorted(rows): print(k, rows[k]["worst"], rows[k].get("lift_lost"))
