"""Full collision check: every part of the Centauri-1 / Equo-1 model against every other part (triangle-level, FCL).
Each sk3d primitive (or named group: body shell, pauldron + thigh cover, neck receiver) is one part.  Contacts are then
sorted: designed joints (bolted / pinned / pressed / sliding fits, listed with the reason) vs real clashes."""
import os, sys, io, contextlib, numpy as np, json
sys.path[:0] = ["/home/claude/octodrone/centaur/preview"]
import trimesh, fcl
from trimesh.collision import CollisionManager

def part_meshes(BP):
    tris = {}
    for pts, pid in zip(BP.sc.polys, BP.sc.pids):
        pts = np.asarray(pts)
        if len(pts) < 3: continue
        for i in range(1, len(pts) - 1):
            tris.setdefault(pid, []).append([pts[0], pts[i], pts[i + 1]])
    out = {}
    for pid, t in tris.items():
        t = np.asarray(t); v = t.reshape(-1, 3); f = np.arange(len(v)).reshape(-1, 3)
        m = trimesh.Trimesh(v, f, process=True)
        if len(m.faces): out[pid] = m
    return out

def contacts(BP):
    ms = part_meshes(BP)
    cm = CollisionManager()
    for pid, m in ms.items(): cm.add_object(str(pid), m)
    hit, names, data = cm.in_collision_internal(return_names=True, return_data=True)
    depth = {}
    for d in data:
        a, b = sorted(d.names, key=int); depth[(a, b)] = max(depth.get((a, b), 0.0), float(d.depth))
    return ms, depth

if __name__ == "__main__":
    with contextlib.redirect_stdout(io.StringIO()):
        import build_preview as BP
if __name__ == "__main__" and os.environ.get("CLASSIFY", "1") == "0":
    ms, depth = contacts(BP)
    agg = {}
    for (a, b), dp in depth.items():
        na, nb = BP.sc.pnames[int(a)], BP.sc.pnames[int(b)]
        key = tuple(sorted((na.split(":")[0] + ":" + na.split(":")[1], nb.split(":")[0] + ":" + nb.split(":")[1])))
        agg.setdefault(key, []).append((dp, na, nb))
    print(f"parts {len(ms)}, contacting pairs {len(depth)}")
    for k, v in sorted(agg.items(), key=lambda kv: -max(x[0] for x in kv[1])):
        print(f"{len(v):4d}  max depth {1e3*max(x[0] for x in v):6.1f} mm  {k[0]:28s} x {k[1]:28s}  e.g. {v[0][1]} | {v[0][2]}")

# ---------------------------------------------------------------- designed contacts: (reason) or None for a real clash
RECV_COL = ("#7b8794",)                 # receiver cartridge, rim posts (alignment)
GRAB_COL = ("#c9661f",)                 # horns, LED ring
POST_COL = ("#d9782a",)                 # grab posts: nothing of the octopus may touch them except a gripping hand
def classify(BP, a, b, depth):
    na, nb = BP.sc.pnames[int(a)], BP.sc.pnames[int(b)]
    ta, tb = na.split(":")[0], nb.split(":")[0]
    ka, kb = ":".join(na.split(":")[:2]), ":".join(nb.split(":")[:2])
    wa = na.split(":")[2] if na.count(":") >= 2 else ""; wb = nb.split(":")[2] if nb.count(":") >= 2 else ""
    ca = col_of(BP, a); cb = col_of(BP, b)
    T = {ta, tb}
    if "reach" in T: return "ghost pose (arm working-pose overlay, not hardware)"
    if T == {"octopus"}: return "inside Cephalo-1 (checked in its own design files; arm-to-arm rule covers its arms)"
    if "octopus" in T and ({ca, cb} & set(RECV_COL + GRAB_COL)) and "frame" in T: return "docking interface: bars in sheaths, horns in guest sockets, plate between rim posts"
    if "octopus" in T and "frame:neck receiver" in (ka, kb): return "docking interface: bars run in the receiver's sheaths"
    if "octopus:ring" in (ka, kb): return "dock-state LED ring set into the crown"
    if "hood" in T: return "sensor hood: sits on the crown, hinged on the front rim post"
    if T in ({"trim", "shell"}, {"trim"}): return "panel-line seams, pod caps and the LED ring set into the skin"
    if T == {"trim", "frame"} and "frame:neck receiver" in (ka, kb): return "dock LED ring set into the crown"
    leg_t = {"legs", "covers", "belts", "actuators"}
    if wa and wa == wb and T <= leg_t:
        if T == {"belts", "covers"}: return None
        if T == {"covers"} and "covers:pauldron+thigh cover" in (ka, kb) and "covers:cyl" in (ka, kb): return None
        return "same leg: joints, hubs, pulleys under belts, bones through cover ports, seam lines on covers"
    if T == {"frame"}:
        return "frame joints: rings / stringers / keel / cross tube / struts / packs on the sled rails / deck on the packs"
    if T == {"frame", "shell"} and ({ca, cb} & set(RECV_COL + GRAB_COL + POST_COL)): return "crown, rim posts and horns bolted through the collar land"
    if T == {"frame", "shell"} and "frame:neck receiver" in (ka, kb): return "crown ring is the rim of the neck opening (bonded)"
    if T in ({"shell", "tail"}, {"trim", "tail"}): return "tail root through its rump port"
    if T == {"tail"}: return "tail joints"
    if T == {"suspension"}: return "swing unit joints: pivot shaft, arms, bell-crank, pushrods, shock eyes, lockout"
    if T == {"actuators", "suspension"} and ("RL" in (wa, wb) or "RR" in (wa, wb)) and depth < 0.06:
        return "rear abduction actuator housing on the end of its swing arm"
    if T == {"actuators", "frame"} and "frame:cyl" in (ka, kb) and depth < 0.0015: return "front cross tube bolted to the hip actuator housing"
    if T == {"shell", "frame"} and depth < 0.0005: return "skin bonded on a rib (touch)"
    return None

_COLS = {}
def col_of(BP, pid):
    if id(BP) not in _COLS: _COLS[id(BP)] = {p: c[0] for c, p in zip(BP.sc.cols, BP.sc.pids)}
    return _COLS[id(BP)].get(int(pid), "")

def report(BP):
    ms, depth = contacts(BP)
    designed, real = {}, []
    for (a, b), dp in depth.items():
        why = classify(BP, a, b, dp)
        if why: designed[why] = designed.get(why, 0) + 1
        else: real.append((dp, BP.sc.pnames[int(a)], BP.sc.pnames[int(b)], col_of(BP, a), col_of(BP, b)))
    real.sort(reverse=True)
    return len(ms), designed, real

if __name__ == "__main__" and os.environ.get("CLASSIFY", "1") == "1":
    n, designed, real = report(BP)
    print(f"\n== {n} parts; designed contacts:")
    for k, v in sorted(designed.items(), key=lambda kv: -kv[1]): print(f"  {v:4d}  {k}")
    print(f"== REAL clashes: {len(real)}")
    for r in real[:40]: print(f"  {1e3*r[0]:6.1f} mm  {r[1]} ({r[3]})  x  {r[2]} ({r[4]})")
