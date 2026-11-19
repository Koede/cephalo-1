"""Summary table of the fall runs (python3 fall_summary.py): impact (fall + impact + re-index) vs recovery gear loads."""
import json, glob, os
import numpy as np

IMPACT = ("hit", "fall", "impact", "reindex")


def row(path):
    j = json.load(open(path)); ph = j["phase"]
    z = np.load(path[:-5] + ".npz"); P = z["phase"]; g = z["gear"]
    k1 = int(np.argmax(P == "assess")) if (P == "assess").any() else len(P)     # first fall: up to the first settle
    k0 = int(np.argmax(np.isin(P, ["fall"])))
    imp = float(g[k0:k1].max()); rec = float(g[k1:].max()) if k1 < len(P) else 0.0
    return dict(file=os.path.basename(path), hw=j.get("hw", "rev4"), v=j["v_imp"], decel=j["decel"], gear=j["gear"], impact=imp, recovery=rec,
                end=j["end"], reindex=j.get("reindex", 0), stop=j.get("shell_j", [0] * 8))


if __name__ == "__main__":
    print("| run | hardware | impact m/s | body g | worst gear: first fall + impact + re-index | worst gear: recovery (incl. retries) | notches re-indexed | stop torque j0 / j1 / j2-j7 max, N m | end |")
    print("|---|---|---|---|---|---|---|---|---|")
    files = [f"fall4_{c}.json" for c in ("cut", "hit", "inv", "drift")] + [f"fall4_{c}_rev4.json" for c in ("cut", "hit", "inv", "drift")]
    files += sorted(glob.glob("fall4_hit_ts[0-9]*.json")) + sorted(glob.glob("fall4_hit_tsr*.json"))
    R = []
    for f in files:
        if not os.path.exists(f): continue
        r = row(f); R.append(r); s = r["stop"]
        print(f"| {r['file'][6:-5]} | {r['hw']} | {r['v']:.1f} | {r['decel']:.0f} | {r['impact']:.2f} | {r['recovery']:.2f} | {r['reindex']} | "
              f"{s[0]:.0f} / {s[1]:.0f} / {max(s[2:]):.0f} | {r['end']} |")
    ts = [r for r in R if "_ts" in r["file"] and "tsr" not in r["file"]]; tr = [r for r in R if "tsr" in r["file"]]
    for nm, S in (("tumble set, reinforced", ts), ("tumble set, rev 4", tr)):
        if S:
            print(f"{nm}: n={len(S)} recovered {sum(r['end'] == 'done' for r in S)} | impact gear max {max(r['impact'] for r in S):.2f} "
                  f"median {np.median([r['impact'] for r in S]):.2f} | recovery gear max {max(r['recovery'] for r in S):.2f} "
                  f"median {np.median([r['recovery'] for r in S]):.2f} | >1.0 anywhere: {sum(r['gear'] > 1.0 for r in S)} | "
                  f"j0 stop max {max(r['stop'][0] for r in S):.0f} N m, j1 {max(r['stop'][1] for r in S):.0f}, j2-j7 {max(max(r['stop'][2:]) for r in S):.0f}")
