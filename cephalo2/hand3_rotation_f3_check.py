"""Checks for hand3 finger rotation F3 (2026-11-20).  Writes hand3_rotation_f3_check.json.  Collisions by distances (r-z sweeps,
since every root can sit at any azimuth) plus exact booleans where parts stand still relative to each other.
1) Root sweep vs STATIC parts, at lift 0 (flight) and lift 1.5 (rotation stage): root + leg, screw, slider (both ends), motor,
   pinion, wipers, straight finger with bands.  Wipers vs the track ring at lift 1.5 is the intended contact (preload reported).
2) Root sweep vs ROTOR parts (roots slide relative to the hub when they rotate): base disc + gear skirt (pinion excluded: it meshes),
   top disc, cage, motor bell.  Pinion-skirt: centre distance and tip clearances.
3) Neighbours at the closest spacing (58 deg) and at 60 deg: exact booleans, straight and both curled to the same amount.
4) One finger curling alone (others straight, flight cross): first curl that touches anything (5 % steps).
5) Stage 2 (lift 5.5): drivers up under parked roots, track ring held at its stop: exact booleans.
6) Static parts among themselves: track ring / posts / drivers / lift ring / frame.
7) Loads, motor torque, current limit (hand3_rotation_f3.numbers)."""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hub_v3 as V3, hand3_hubc as HC, hand3_multidigit as MD
import hand3_rotation_f3 as F
from scipy.spatial import cKDTree

res = {}


def rz(m, n=6):
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3]; t = np.asarray(me.tri_verts)
    e = np.concatenate([v[t[:, a]] + (v[t[:, b]] - v[t[:, a]]) * f for a, b in ((0, 1), (1, 2), (2, 0)) for f in np.linspace(0, 1, n)])
    return np.c_[np.hypot(e[:, 0], e[:, 1]), e[:, 2]]


def U(L):
    out = None
    for m in L: out = m if out is None else out + m
    return out


fing = U(list(V3.finger(0).values()))
root_parts = dict(root=F.root_f3(0), screw=MD.screw(), slider_top=MD.slider(0.0), slider_bottom=MD.slider(MD.STROKE),
                  motor=F.motor(), pinion=F.pinion(), wipers=F.wipers(3), finger=fing)
PTS = {n: rz(m) for n, m in root_parts.items()}

# 1) vs static
res["static_sweep_gap_mm"] = {}
for lift in (0.0, F.STAGE1):
    st = {"drivers": MD.drivers(False).translate([0, 0, lift]), "lift ring": F.lift_ring_f3().translate([0, 0, lift]), "frame": F.frame_f3(),
          "motor base": HC.hardware()["motor_base"], "track ring + copper": (F.track_ring() + F.tracks()).translate([0, 0, F.track_dz(lift)]),
          "posts": F.posts().translate([0, 0, F.track_dz(lift)])}
    out = {}
    for sn, sm in st.items():
        sp = rz(sm); tr = cKDTree(sp); worst = (1e9, "")
        for rn, p in PTS.items():
            if rn == "wipers" and sn == "track ring + copper" and lift > 0: continue
            d, _ = tr.query(p); worst = min(worst, (float(d.min()), rn))
        out[sn] = dict(gap=round(worst[0], 2), nearest=worst[1])
    res["static_sweep_gap_mm"][f"lift {lift}"] = out
wtip = F.WIPER["tip_z"]; ring_top = F.TRACK["z"][1] + F.TRACK["cu_t"]
res["wiper"] = dict(flight_gap_mm=round(wtip - ring_top, 2), preload_mm_at_stage1=round(ring_top + F.STAGE1 - wtip, 2))
print("static", json.dumps(res["static_sweep_gap_mm"]), res["wiper"], flush=True)

# 2) vs rotor parts (root slides relative to them)
rotor = {"base disc (no skirt)": F.base_disc_f3() - F.skirt(), "gear skirt": F.skirt(), "top disc": V3.top_disc(), "cage": V3.cage_v3(),
         "motor bell": C.cyl(17, H.Z["pack1"] + 8.5, H.Z["motor_top"]) + C.cyl(2.5, H.Z["motor_top"], H.Z["shaft_tip"])}
res["rotor_sweep_gap_mm"] = {}
for sn, sm in rotor.items():
    tr = cKDTree(rz(sm)); worst = (1e9, "")
    for rn, p in PTS.items():
        if rn == "pinion" and sn == "gear skirt": continue
        d, _ = tr.query(p); worst = min(worst, (float(d.min()), rn))
    res["rotor_sweep_gap_mm"][sn] = dict(gap=round(worst[0], 2), nearest=worst[1])
ra_p = F.RP + F.GEAR_M; ra_g = F.RG + F.GEAR_M; rf_p = F.RP - 1.25 * F.GEAR_M; rf_g = F.RG - 1.25 * F.GEAR_M
res["mesh"] = dict(centre_distance=F.AX_R, pitch_sum=F.RP + F.RG, tip_clearance_pinion=round(F.AX_R - ra_p - rf_g, 3),
                   tip_clearance_skirt=round(F.AX_R - ra_g - rf_p, 3), pinion_top_to_disc=round(V3.V["z0"] - F.PINION["z_band"][1], 2),
                   face_overlap_mm=round(min(F.PINION["z_band"][1], F.SKIRT["z_band"][1]) - max(F.PINION["z_band"][0], F.SKIRT["z_band"][0]), 2))
print("rotor", json.dumps(res["rotor_sweep_gap_mm"]), res["mesh"], flush=True)


# 3) neighbours
def body(az, c):
    """Root + motor + pinion + screw + the finger curled to c (phalanges, pads), world at az."""
    parts = [F.root_f3(az), F.motor().rotate([0, 0, az]), F.pinion().rotate([0, 0, az]), MD.screw().rotate([0, 0, az]),
             F.wipers(0).rotate([0, 0, az])]
    fr = C.finger_frames(0, 0.0, tuple(f * c for f in H.FOLD_MAX)); R = C.parts()
    Rz = np.eye(4); a = np.radians(az - C.FAZ[0]); Rz[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    for j in range(3):
        A = Rz @ np.asarray(fr[j + 1], float)
        parts.append(V3.B.phalanx_v3(j).transform(A[:3, :4]))
        parts.append(R[f"pad{j + 1}"].transform(A[:3, :4]))
    return U(parts)


res["neighbours_mm3"] = {}
for sep in (58.0, 60.0):
    for c in (0.0, 0.3, 0.5, 0.7):
        v = (body(0.0, c) ^ body(sep, c)).volume()
        res["neighbours_mm3"][f"{sep:.0f} deg, both curled {int(c * 100)} %"] = round(v, 3)
print("neighbours", res["neighbours_mm3"], flush=True)

# 4) solo curl, flight cross
others = U([body(a, 0.0) for a in (90, 180, 270)]) + F.base_disc_f3() + V3.top_disc() + C.parts()["heel"]
first = None
for c in np.arange(0.05, 1.001, 0.05):
    fr = C.finger_frames(0, 0.0, tuple(f * c for f in H.FOLD_MAX)); R = C.parts(); Rz = np.eye(4); a = np.radians(-C.FAZ[0]); Rz[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    ph = U([V3.B.phalanx_v3(j).transform((Rz @ np.asarray(fr[j + 1], float))[:3, :4]) + R[f"pad{j + 1}"].transform((Rz @ np.asarray(fr[j + 1], float))[:3, :4]) for j in range(3)])
    v = (ph ^ (others + F.motor() + F.leg_f3())).volume()
    if v > 0.01: first = round(float(c), 2); res["solo_curl_touch"] = dict(curl=first, mm3=round(v, 3)); break
if first is None: res["solo_curl_touch"] = "none to 100 %"
print("solo", res["solo_curl_touch"], flush=True)

# 5) stage 2: drivers up under parked roots
up = MD.drivers(False).translate([0, 0, MD.KEY["lift"]]) + F.lift_ring_f3().translate([0, 0, MD.KEY["lift"]])
ring = (F.track_ring() + F.tracks() + F.posts()).translate([0, 0, F.STAGE1])
hits = {}
for a in MD.DRIVER_AZ:
    r = F.root_f3(a) + MD.screw().rotate([0, 0, a]) + MD.slider(MD.STROKE).rotate([0, 0, a]) + F.motor().rotate([0, 0, a]) + F.pinion().rotate([0, 0, a])
    v = (up ^ r).volume(); vr = (ring ^ r).volume()
    if v > 0.01 or vr > 0.01: hits[a] = dict(drivers=round(v, 3), track_ring=round(vr, 3))
res["stage2_hits"] = hits
key_top = MD.KEY["top_retracted"] + MD.KEY["lift"]
res["stage2_key"] = dict(key_top=key_top, slot=(MD.SCREW["z"][0], MD.SCREW["z"][0] + MD.SCREW["slot"][1]), funnel_capture_mm=F.FUNNEL["r0"] - MD.KEY["w"] / 2,
                         capture_deg=round(np.degrees((F.FUNNEL["r0"] - MD.KEY["w"] / 2) / MD.SCREW["x"]), 1))
print("stage2", hits, res["stage2_key"], flush=True)

# 6) static parts among themselves
st6 = {}
for lift in (0.0, F.STAGE1, MD.KEY["lift"]):
    dz = F.track_dz(lift)
    ring = (F.track_ring() + F.tracks()).translate([0, 0, dz]); po = F.posts().translate([0, 0, dz])
    dr = MD.drivers(False).translate([0, 0, lift]); li = F.lift_ring_f3().translate([0, 0, lift]); fr = F.frame_f3()
    st6[f"lift {lift}"] = {"ring x drivers": round((ring ^ dr).volume(), 3), "posts x drivers": round((po ^ dr).volume(), 3),
                           "posts x frame": round((po ^ fr).volume(), 3), "posts x lift ring": round((po ^ li).volume(), 3),
                           "drivers x frame": round((dr ^ fr).volume(), 3), "lift ring x frame": round((li ^ fr).volume(), 3)}
res["static_static_mm3"] = st6
print("static-static", st6, flush=True)

# 7) numbers
res["numbers"] = F.numbers()
json.dump(res, open(os.path.join(HERE, "hand3_rotation_f3_check.json"), "w"), indent=1, default=float)
print(json.dumps(res["numbers"], default=float))
