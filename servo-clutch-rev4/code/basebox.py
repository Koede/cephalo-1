"""Base control box packing for one Cephalo-1 arm sector (arm-local frame: x radial out from body centre, y tangential,
z up from plate underside; mm).  Greedy packer: axis-aligned servo envelopes (body + spool) placed inside the allowed
region (45 deg sector, dome, plate, EDF/airflow keep-out), no overlaps (1.5 mm clearance), shortest tendon run to the arm
porthole.  Two tiers: above the plate inside the dome, and an under-plate tray."""
import numpy as np, itertools, sys
R_DOME = float(sys.argv[1]) if len(sys.argv) > 1 else 125.0     # mantle radius (plate + dome a)
C_DOME = float(sys.argv[2]) if len(sys.argv) > 2 else 150.0
WALL = R_DOME - 21.0            # arm wall face (104 at R 125)
PORT = np.array([WALL, 0.0, 39.0])
R_IN = 62.0                     # EDF lip 55 + airflow funnel clearance
HALF = np.radians(22.5)
CLR = 1.5
UNDER = (-46.0, -3.0)           # under-plate tray z range
SKIN = 3.0                      # dome wall

def inside(P):
    x, y, z = P[:, 0], P[:, 1], P[:, 2]
    r = np.hypot(x, y)
    sect = np.abs(y) <= x * np.tan(HALF) - CLR - 1.0
    above = (z >= 6 + CLR) & (r <= WALL - CLR) & (z <= 6 + C_DOME * np.sqrt(np.clip(1 - (r / (R_DOME - SKIN)) ** 2, 0, 1)) - SKIN - CLR)
    under = (z >= UNDER[0]) & (z <= UNDER[1]) & (r <= R_DOME - 4)
    return sect & (r >= R_IN) & (above | under)

KEEP = [(np.array([WALL - 30, -15, 24]), np.array([WALL, 15, 54]))]   # tendon guide block + knuckle shafts at the porthole

def pts(lo, hi):
    g = [np.linspace(lo[i], hi[i], 3) for i in range(3)]
    return np.array(list(itertools.product(*g)))

def overlap(lo, hi, boxes):
    for blo, bhi, *_ in boxes:
        if np.all(lo < bhi + CLR) and np.all(hi > blo - CLR):
            return True
    return False

# parts: name, envelope (L, W, H) incl. spool on top, count
PARTS = [("STS3250", (45.2, 24.7, 35.0 + 12), 2), ("STS3215", (45.2, 24.7, 35.0 + 12), 1),
         ("STS3032", (23.2, 12.1 + 2, 28.5 + 8), 16), ("PCB", (40.0, 28.0, 8.0), 1)]

def pack():
    placed = [(k[0], k[1], "keep", None) for k in KEEP]
    xs = np.arange(R_IN, R_DOME, 2.0); ys = np.arange(-80, 80, 2.0); zs = np.concatenate([np.arange(UNDER[0], UNDER[1], 2.0), np.arange(6, 6 + C_DOME, 2.0)])
    fails = []
    for name, dims, cnt in PARTS:
        perms = sorted(set(itertools.permutations(dims)))
        for c in range(cnt):
            best = None
            for d in perms:
                d = np.array(d)
                for x in xs[::2]:
                    if x + d[0] > R_DOME: continue
                    for y in ys[::2]:
                        for z in zs:
                            lo = np.array([x, y - d[1] / 2, z]); hi = lo + d
                            ctr = (lo + hi) / 2
                            cost = np.linalg.norm(ctr - PORT) + (8 if name == "PCB" else 0) * 0
                            if best is not None and cost >= best[0]: continue
                            if not (hi[2] <= UNDER[1] or lo[2] >= 6 + CLR): continue
                            if not inside(pts(lo, hi)).all(): continue
                            if overlap(lo, hi, placed): continue
                            best = (cost, lo, hi)
            if best is None:
                fails.append(name)
            else:
                placed.append((best[1], best[2], name, c))
    return [p for p in placed if p[2] != "keep"], fails

if __name__ == "__main__":
    placed, fails = pack()
    vol = sum(np.prod(h - l) for l, h, *_ in placed) / 1000
    print(f"R_DOME {R_DOME}  wall {WALL}  placed {len(placed)}  failed {fails}  envelope vol {vol:.0f} cm3")
    under = sum(1 for l, h, *_ in placed if h[2] <= 0)
    print("under-plate parts:", under, " z range", min(l[2] for l, *_ in placed), max(h[2] for _, h, *_ in placed),
          " x range", min(l[0] for l, *_ in placed), max(h[0] for _, h, *_ in placed))
    np.save(f"pack_{int(R_DOME)}_{int(C_DOME)}.npy", np.array([(l, h) for l, h, *_ in placed]), allow_pickle=True)
    import pickle; pickle.dump(placed, open(f"pack_{int(R_DOME)}_{int(C_DOME)}.pkl", "wb"))
