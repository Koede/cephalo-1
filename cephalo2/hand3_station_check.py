"""Station A (blade + bell crank + roller) checks for hand3 hub v3, 2026-11-19.
1) Flight: every static station part (retracted) against the swept envelope of everything that spins, in (r, z) cells of 0.25 mm
   (a spinning part fills its whole circle, so this is exact range math in r and z).  Fingers straight; ring at any pull.
2) Parked, rotor turning under the roller (alignment): same check with the carriage at roller-on.
3) Lift: crank turn, pin lift, finger travel; crank vs its neighbours (range math); blade vs the held root and bead at full stroke.
Writes hand3_station_check.json"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hubc as HC, hand3_hub_v3 as V3
os.chdir(HERE)
D = 0.25
def cells(m, step=D):
    """Surface points of a manifold (about 20 per mm2) -> (r, z) cell keys.  Surfaces are enough: no static part sits wholly
    inside a spinning one."""
    import trimesh
    tm = C.to_trimesh(m); n = int(min(max(tm.area * 20, 2000), 3e6))
    pts, _ = trimesh.sample.sample_surface_even(tm, n, seed=0) if n < 4e5 else trimesh.sample.sample_surface(tm, n, seed=0)
    pts = np.r_[pts, tm.vertices]
    rz = np.c_[np.hypot(pts[:, 0], pts[:, 1]), pts[:, 2]]
    return set(map(tuple, np.floor(rz / step).astype(int)))
def swept(parts):
    s = set()
    for m in parts: s |= cells(m)
    return s
rot = [V3.base_disc(), V3.top_disc(), V3.cage_v3(), C.parts()["heel"], V3.root_block(0), V3.bead(0)] + [V3.crank(sd) for sd in (-1, 1)] + \
      [V3.latch_pin(0, sd) for sd in (-1, 1)] + list(V3.finger(0).values())
ring = V3.ring_v3(); rot_ring = [ring.translate([0, 0, -p]) for p in np.linspace(0, V3.RING_STROKE, 12)] + [V3.bead(0, p) for p in np.linspace(0, V3.RING_STROKE, 12)]
res = {}
def station_at(s):
    sp = V3.station_parts(); ps = V3.station_pose(s); out = {}
    for n, m in sp.items(): out[n] = m.translate([ps[n], 0, 0]).rotate([0, 0, V3.STATION_AZ])
    return out
import trimesh
from scipy.spatial import cKDTree
def rz_samples(m, n=300000, seed=0):
    tm = C.to_trimesh(m); p, _ = trimesh.sample.sample_surface(tm, n, seed=seed); p = np.r_[p, tm.vertices]
    return np.c_[np.hypot(p[:, 0], p[:, 1]), p[:, 2]]
ROT_RZ = np.concatenate([rz_samples(m, 200000, i) for i, m in enumerate(rot + rot_ring)])
ROT_TREE = cKDTree(ROT_RZ)
_K = lambda a: (np.floor(a[:, 0] / 0.1).astype(np.int64) << 20) + np.floor(a[:, 1] / 0.1).astype(np.int64)
FILL = np.unique(_K(ROT_RZ))
def rz_gap(stat_m, label):
    """Smallest distance in (r, z) between a static part and everything that spins; negative (area, mm2) if static samples fall
    in 0.1 mm cells the spinning parts fill."""
    q = rz_samples(stat_m, 400000, 7); n_in = int(np.isin(_K(q), FILL).sum())
    if n_in > 3: return -round(len(np.unique(_K(q)[np.isin(_K(q), FILL)])) * 0.01, 2)
    return round(float(ROT_TREE.query(q)[0].min()), 2)
for label, s in (("flight (carriage retracted)", 0.0), ("parked, roller on, rotor turning", V3.ST["roll_gap"] + 0.4)):
    for n, m in station_at(s).items():
        if label.startswith("parked") and n in ("tyre",): continue                      # the tyre touches the cage on purpose
        res[f"{label}: {n} (mm in r-z)"] = rz_gap(m, label)
# engaged (pins lifted, the hub turning under the held root): blade against the parts that turn past it: discs, cage, ring + rail
AX = [V3.base_disc(), V3.top_disc(), V3.cage_v3(), V3.ring_v3()]
AX_RZ = np.concatenate([rz_samples(m, 200000, 20 + i) for i, m in enumerate(AX)]); AX_TREE = cKDTree(AX_RZ); AX_FILL = np.unique(_K(AX_RZ))
for n in ("blade", "sensor"):
    q = rz_samples(station_at(V3.ST["stroke"])[n], 400000, 9); n_in = int(np.isin(_K(q), AX_FILL).sum())
    res[f"engaged, hub turning: {n} vs discs, cage, ring + rail (mm in r-z)"] = -1.0 if n_in > 3 else round(float(AX_TREE.query(q)[0].min()), 2)
# static drive and frame against the station: smallest distance between surface samples (KD-tree), negative if a sample lies inside
from scipy.spatial import cKDTree
import trimesh
def surf(m, per_mm2=12):
    tm = C.to_trimesh(m); n = int(min(max(tm.area * per_mm2, 1500), 2e6)); p, _ = trimesh.sample.sample_surface(tm, n, seed=1); return tm, np.r_[p, tm.vertices]
def dist(ma, mb):
    ta, pa = surf(ma); tb, pb = surf(mb)
    d = cKDTree(pb).query(pa)[0].min()
    inside = tb.contains(pa[:: max(1, len(pa) // 4000)]).any() or ta.contains(pb[:: max(1, len(pb) // 4000)]).any()
    return round(float(-d if inside else d), 2)
dr = V3.drive(0.0); dr_pull = V3.drive(V3.RING_STROKE)
for s in (0.0, V3.ST["stroke"]):
    st = station_at(s)
    for n, m in st.items():
        b = np.asarray(m.bounding_box()); near = []
        for D_ in (dr, dr_pull):
            for k, x in D_.items():
                bx = np.asarray(x.bounding_box())
                if np.all(b[:3] - 2 <= bx[3:]) and np.all(bx[:3] - 2 <= b[3:]): near.append((k, x))
        res[f"station {n} (s {s:g}) vs drive + frame (mm)"] = min(((dist(m, x), k) for k, x in near), default=(">2", "nothing within 2 mm"))
res["lift"] = dict(crank_turn_full=round(V3.crank_turn(V3.ST["stroke"]), 2), needed=round(V3.CR_ANGLE, 2),
                   lift_mm=round(np.hypot(V3.CR["pivot"][0] - V3.CR["finger"][0], V3.CR["pivot"][1] - V3.CR["finger"][2]) * np.sin(np.radians(V3.crank_turn(V3.ST["stroke"]))), 2),
                   tip_travel_mm=round(V3.ST["stroke"] - (V3.ST["tip_r0"] - (V3.CR["tip"][0] + 0.5)), 2))
# crank at rest and at full turn against its own root block, the base disc, the ring + rail and the bead (distances)
blk = V3.root_block(0); bd = V3.base_disc(); rail = V3.ring_v3(); bead = V3.bead(0)
for sd in (-1, 1):
    for turn in (0.0, V3.crank_turn(V3.ST["stroke"])):
        c = V3.crank(sd).transform(V3.crank_pose(sd, turn)[:3, :4])
        res[f"crank {sd:+d} at {turn:.0f} deg: gap to block / base disc / ring+rail / bead (mm)"] = [dist(c, x) for x in (blk, bd, rail, bead)]
json.dump(res, open("hand3_station_check.json", "w"), indent=1, default=float)
for k, v in res.items(): print(f"  {k}: {v}")
