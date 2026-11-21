"""Checks for hand3 hub v3, multi-digit control (decision D), 2026-11-20.  Writes hand3_multidigit_check.json.
1) r-z sweep: everything that turns with the rotor at a root (root + winch leg, screw, slider at both ends of its travel, straight
   finger with its bands) against every static part (8 drivers down, lift ring, frame, station at az 150 retracted, motor base).
2) Station engaged on a held root (blade at full stroke, notch over the leg): exact boolean.
3) Drivers up under parked roots (all 8 positions): exact booleans (keys in the screw slots, couplers under the legs).
4) Station against the two neighbouring drivers: exact booleans.
5) One finger curling on its own (the others straight), flight cross: first curl that touches anything (exact booleans, 5 % steps).
6) Winch loads and section (hand3_multidigit.numbers)."""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hub_v3 as V3, hand3_hubc as HC, hand3_multidigit as MD
from scipy.spatial import cKDTree

res = {}


def rz(m, n=6):
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3]; t = np.asarray(me.tri_verts)
    e = np.concatenate([v[t[:, a]] + (v[t[:, b]] - v[t[:, a]]) * f for a, b in ((0, 1), (1, 2), (2, 0)) for f in np.linspace(0, 1, n)])
    return np.c_[np.hypot(e[:, 0], e[:, 1]), e[:, 2]]


# 1) sweep
root = MD.root_d(0); fing = None
for m in V3.finger(0).values(): fing = m if fing is None else fing + m
turning = root + MD.screw() + MD.slider(0.0) + MD.slider(MD.STROKE) + fing
cranks = V3.crank(1) + V3.crank(-1)
tree = cKDTree(rz(turning + cranks + V3.latch_pin(0, 1) + V3.latch_pin(0, -1)))
st = MD.station_d()
statics = {"drivers (down)": MD.drivers(False), "lift ring (down)": MD.lift_ring(False), "frame": MD.frame_d(), "motor base": HC.hardware()["motor_base"]}
for n, m in st.items(): statics[f"station {n} (retracted)"] = m.rotate([0, 0, MD.STATION_AZ])
res["sweep_gap_mm"] = {}
for n, m in statics.items():
    d, _ = tree.query(rz(m)); res["sweep_gap_mm"][n] = round(float(d.min()), 2)
print("sweep", res["sweep_gap_mm"], flush=True)

# 2) station engaged on a held root
held = MD.root_d(MD.STATION_AZ) + MD.screw().rotate([0, 0, MD.STATION_AZ]) + MD.slider(0).rotate([0, 0, MD.STATION_AZ])
blade_in = st["blade"].translate([-(V3.ST["stroke"] - V3.ST["lost"]), 0, 0]).rotate([0, 0, MD.STATION_AZ])
res["blade_engaged_x_held_root_mm3"] = round((blade_in ^ held).volume(), 3)
tyre_in = st["tyre"].translate([-V3.ST["roll_gap"], 0, 0]).rotate([0, 0, MD.STATION_AZ])
res["tyre_on_x_held_root_mm3"] = round((tyre_in ^ held).volume(), 3)

# 3) drivers up under parked roots
up = MD.drivers(True) + MD.lift_ring(True); hits = {}
for a in MD.DRIVER_AZ:
    r = MD.root_d(a) + MD.screw().rotate([0, 0, a]) + MD.slider(MD.STROKE).rotate([0, 0, a])
    v = (up ^ r).volume()
    if v > 0.01: hits[a] = round(v, 3)
res["drivers_up_x_parked_roots_mm3"] = hits
res["drivers_up_x_frame_mm3"] = round((up ^ MD.frame_d()).volume(), 3)

# 4) station vs neighbouring drivers
nb = {}
for n, m in st.items():
    for s_ in (0.0, V3.ST["stroke"]):
        mm = m.translate([V3.station_pose(s_)[n], 0, 0]).rotate([0, 0, MD.STATION_AZ])
        v = (mm ^ (MD.drivers(False) + MD.drivers(True))).volume()
        if v > 0.01: nb[f"{n} @ {s_:.1f}"] = round(v, 2)
res["station_x_drivers_mm3"] = nb
print("2-4", res["blade_engaged_x_held_root_mm3"], res["tyre_on_x_held_root_mm3"], hits, res["drivers_up_x_frame_mm3"], nb, flush=True)

# 5) one finger curling alone
pose = V3.POSES["flight cross"]
hub = C.parts()["heel"] + V3.top_disc() + V3.base_disc()
others = None
for k, a in enumerate(pose[1:], 1):
    for m in V3.finger(a).values(): others = m if others is None else others + m
    others += MD.root_d(a)
own_root = MD.root_d(pose[0]); solo = None
base_hits = {}
for c in np.arange(0, 1.0001, 0.05):
    f = V3.finger(pose[0], tuple(c * x for x in H.FOLD_MAX)); fm = None
    for n, m in f.items():
        if n.startswith("band") or n.startswith("dowel"): continue
        fm = m if fm is None else fm + m
    v = dict(hub=(fm ^ hub).volume(), others=(fm ^ others).volume(), own_root=(fm ^ own_root).volume())
    if c == 0: base_hits = v
    new = {k_: round(x, 2) for k_, x in v.items() if x > base_hits[k_] + 0.5}
    if new: solo = dict(first_touch_curl=round(float(c), 2), with_=new); break
res["solo_curl"] = solo if solo else dict(first_touch_curl=None, note="clear to 100 %")
print("solo", res["solo_curl"], flush=True)

# 6) loads
res["winch"] = MD.numbers()
json.dump(res, open(os.path.join(HERE, "hand3_multidigit_check.json"), "w"), indent=1, default=float)
print(json.dumps(res, indent=1, default=float))
