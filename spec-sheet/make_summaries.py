import json, glob, os, numpy as np, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import spec_lib as S
H = os.path.join(S.HERE, "results")
# stance summary at the chosen radius (alt90 0.30 m) plus other radii and the helix at 0.25 m
rows = []; store = {}; by_r = {}
for fn in sorted(glob.glob(os.path.join(H, "stance2_*.json"))):
    d = json.load(open(fn)); arr = os.path.basename(fn).split("_")[1]
    for foot, r in d["rows"].items():
        rr = dict(arr=arr, radius_m=d["radius_m"], side=d["side"], foot=foot, **r); rows.append(rr)
        by_r.setdefault((arr, d["radius_m"]), []).append(rr)
    p = fn.replace(".json", ".npz")
    if os.path.exists(p): store.update(dict(np.load(p)))
R0 = 0.30
main = [r for r in by_r.get(("alt90", R0), [])]
ok = [r for r in main if r["ok"]]
worst = max(ok, key=lambda r: r["worst"]) if ok else None
radii = {}
for (arr, rad), rs in sorted(by_r.items()):
    oks = [r for r in rs if r["ok"]]
    radii[f"{arr} {rad:.2f}"] = dict(cases=len(rs), feasible=len(oks), worst=max([r["worst"] for r in oks], default=None), worst_2x=max([r["worst_2x"] for r in oks], default=None))
helix = radii.get("helix60 0.25", {})
summ = dict(radius_m=R0, worst=worst["worst"] if worst else None, worst_2x=max(r["worst_2x"] for r in ok) if ok else None, need=worst["need"] if worst else None,
            worst_case=f"{worst['side']} {worst['foot']}" if worst else None, helix_worst=helix.get("worst"), radii=radii,
            rows=[dict(side=f"pod {r['side']}", foot=r["foot"], feasible_starts=r["feasible_starts"], ok=r["ok"], worst=r["worst"], worst_2x=r["worst_2x"]) for r in main],
            table_note=("Foot = the folded pod's finger tip, with the hub 250 mm above the ground and the tip axis within 10 deg of straight down; 'mid', 'side +45' (45 mm sideways) "
                        "and 'in 34' (34 mm inward) are three points of the study's step. Pod clocking = which way the folded finger points. Each case: 36 random starts moved onto the "
                        "constraints, then the best of up to six optimised. 'No pose' means none of the 36 starts reached that foot point."))
note = []
for k, v in radii.items():
    if v["worst"] is not None: note.append(f"{k.replace('alt90','no helix').replace('helix60','helix')} m: worst {v['worst']:.2f} ({v['feasible']}/{v['cases']} cases reachable)")
summ["note"] = "By radius: " + "; ".join(note) + "."
json.dump(summ, open(os.path.join(H, "stance_summary.json"), "w"), indent=1)
P = os.path.join(H, "poses_alt90.npz")
if os.path.exists(P):
    allp = dict(np.load(P)); allp = {k: v for k, v in allp.items() if "_stance_" not in k}
    # keep only the chosen radius' stance poses (stance2 files are tagged by radius)
    for fn in glob.glob(os.path.join(H, f"stance2_alt90_*_{int(R0*100)}.npz")): allp.update(dict(np.load(fn)))
    np.savez(P, **allp)
A = S.Arm("alt90"); n = A.need(np.zeros(S.N))[0]
json.dump(dict(need=[round(float(x), 3) for x in n], worst=round(float(n.max()), 3), joint=int(np.argmax(n)),
               reach_ok=None), open(os.path.join(H, "straight_need.json"), "w"), indent=1)
print(json.dumps(summ, indent=0)[:1500])
