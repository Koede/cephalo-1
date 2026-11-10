import json, glob, os, numpy as np, sys
H = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
for arr in ("alt90", "helix60"):
    out = None; store = {}
    for fn in sorted(glob.glob(os.path.join(H, f"loads_{arr}_*.json"))):
        d = json.load(open(fn))
        if out is None: out = {k: v for k, v in d.items() if k != arr}; out[arr] = {}
        out[arr].update(d[arr])
        out.setdefault("stance_foot_radius_m", d.get("stance_foot_radius_m"))
        p = fn.replace("loads_", "poses_").replace(".json", ".npz")
        if os.path.exists(p): store.update(dict(np.load(p)))
    if out is None: continue
    out.update({k: v for k, v in out[arr].items()}) if False else None
    flat = dict(out); flat.update(out[arr])
    json.dump(flat, open(os.path.join(H, f"loads_{arr}.json"), "w"), indent=1)
    np.savez(os.path.join(H, f"poses_{arr}.npz"), **store)
    print(arr, list(out[arr].keys()), len(store))
