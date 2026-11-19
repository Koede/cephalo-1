import sys, numpy as np
sys.path.insert(0, "/home/claude/octodrone/hand3e"); sys.path.insert(0, "/home/claude/octodrone")
import hand3_hubc as HC
from hand3_finger import app, T
import importlib.util
spec = importlib.util.spec_from_file_location("g", "/home/claude/octodrone/hand3e_glb.py")
src = open("/home/claude/octodrone/hand3e_glb.py").read().split("scene = trimesh.Scene()")[0]
g = {}; exec(src, g)
a = g["a"]; explode = g["explode"]
names = [n for n in sorted(a) if not a[n][0].is_empty()]
def run(view):
    P = {}
    for n in names:
        m = a[n][0]; c = np.array(m.bounding_box()).reshape(2, 3).mean(0)
        off = explode(n, c) if view == "exploded" else np.zeros(3)
        P[n] = m.translate(list(off))
    bb = {n: np.array(P[n].bounding_box()).reshape(2, 3) for n in names}
    out = []
    for i, x in enumerate(names):
        for y in names[i + 1:]:
            if (bb[x][1] < bb[y][0]).any() or (bb[y][1] < bb[x][0]).any(): continue
            v = (P[x] ^ P[y]).volume()
            if v > 0.02: out.append((round(v, 2), x, y))
    return sorted(out, reverse=True)
for view in sys.argv[1:]:
    o = run(view); print(view, len(o), "overlapping pairs")
    for r in o[:60]: print("  ", r)
