"""Independent checks of the R160 base-box layout: dense envelope test, pairwise clearance, no plate straddling,
tendon runs (spool -> arm porthole) blocked by other parts, and 8-sector neighbour clearance."""
import sys; sys.argv = ["x", "160", "150"]
import numpy as np, pickle, itertools
import basebox as b
P = pickle.load(open("pack_160_150.pkl", "rb"))
H_OF = {"STS3250": 47.0, "STS3215": 47.0, "STS3032": 36.5}
def spool(l, h, name):
    d = h - l
    if name not in H_OF: return None, None
    ax = int(np.argmin(np.abs(d - H_OF[name])))
    c = (l + h) / 2
    sgn = 1 if (b.PORT - c)[ax] >= 0 else -1
    s = c.copy(); s[ax] = (h if sgn > 0 else l)[ax] - sgn * 5
    Lax = int(np.argmax(np.where(np.arange(3) == ax, -1, d)))
    s[Lax] += 0.25 * d[Lax] * (1 if (b.PORT - c)[Lax] >= 0 else -1)
    return s, ax
def seg_box(p, q, lo, hi):
    t0, t1 = 0.0, 1.0; dd = q - p
    for i in range(3):
        if abs(dd[i]) < 1e-12:
            if p[i] < lo[i] or p[i] > hi[i]: return False
        else:
            a, c = (lo[i] - p[i]) / dd[i], (hi[i] - p[i]) / dd[i]
            t0, t1 = max(t0, min(a, c)), min(t1, max(a, c))
            if t0 > t1: return False
    return True
ok = True
g = lambda l, h: np.array(list(itertools.product(*[np.linspace(l[i], h[i], 7) for i in range(3)])))
env = [b.inside(g(l, h)).all() for l, h, *_ in P]
straddle = [not (h[2] <= b.UNDER[1] or l[2] >= 6 + b.CLR) for l, h, *_ in P]
minclr = 1e9
for (l1, h1, *_), (l2, h2, *_) in itertools.combinations(P, 2):
    gap = np.max(np.maximum(l2 - h1, l1 - h2)); minclr = min(minclr, gap)
blocked = 0; runs = []
for i, (l, h, name, c) in enumerate(P):
    s, ax = spool(l, h, name)
    if s is None: continue
    hit = [j for j, (l2, h2, *_) in enumerate(P) if j != i and seg_box(s, b.PORT, l2 - 0.5, h2 + 0.5)]
    blocked += bool(hit); runs.append((name, np.linalg.norm(b.PORT - s), hit))
# neighbour sector: rotate this layout by 45 deg and check overlap with the original (corners)
Rz = lambda a: np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
nb = 0
for l, h, *_ in P:
    C = g(l, h) @ Rz(np.radians(45)).T
    for l2, h2, *_ in P:
        if np.any(np.all((C > l2) & (C < h2), axis=1)): nb += 1
print(f"parts {len(P)} | all inside envelope {all(env)} | plate straddles {sum(straddle)} | min pairwise gap {minclr:.1f} mm")
print(f"neighbour-sector overlaps {nb} | tendon runs blocked by another part: {blocked} of {len(runs)} (need a guide pulley)")
print("tendon run length mm: min %.0f max %.0f mean %.0f" % (min(r[1] for r in runs), max(r[1] for r in runs), np.mean([r[1] for r in runs])))
