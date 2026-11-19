"""Checks for the Hippos-1 tail mount on the true solids.  python3 tailmount_checks.py -> tailmount_checks.json
Locked fit, push-in, 60 deg twist with the latch riding the keeper, rocking (tilt) limit set by the skirt in the sleeve,
latch-tab access through the windows, print overhangs, masses, and the connector loads at the real root-joint offset."""
import json, numpy as np, manifold3d as mf
import sff_core as H, tailmount as M
from sff_core import P
T = M.T
EPS = 0.01       # mm^3: below this an intersection is facet noise

S = M.rump_socket(); A = M.tail_adapter(); B = H.hand_body(); L0 = H.latch()
out = {}

def vol(a, b):
    return (a ^ b).volume()

def tail_at(theta, dz, defl=0.0, tilt=None):
    parts = {"adapter": A, "base": B, "latch": H.flex_latch(L0, defl)}
    res = {}
    for k, m in parts.items():
        m = H.place(m, theta, dz)
        if tilt is not None:
            piv, axis_az, ang = tilt
            m = m.translate((-piv).tolist()).rotate([0, 0, -axis_az]).rotate([ang, 0, 0]).rotate([0, 0, axis_az]).translate(piv.tolist())
        res[k] = m
    return res

def overlaps(tp):
    return {k: vol(S, m) for k, m in tp.items()} | {"latch_vs_adapter": vol(tp["latch"], tp["adapter"]) if False else 0.0}

# ---- 1. locked
tp = tail_at(0, 0)
out["locked"] = {k: vol(S, m) for k, m in tp.items()}
out["locked"]["skirt_radial_clearance"] = T["sl_r0"] - T["sk_r1"]
out["locked"]["servo_flange_to_sleeve_end"] = T["sl_bot"] - T["sf_z"][1]
out["locked"]["skirt_top_to_mount_flange"] = P["z_plate"] - T["sk_top"]

# ---- 2. push-in at the entry angle
th0 = -P["twist"]; dz_rest = H.ramp_dz(th0)
push = []
for dz in np.arange(45.0, dz_rest - 1e-6, -1.0).tolist() + [dz_rest]:
    tp = tail_at(th0, dz)
    push.append((dz, max(vol(S, m) for m in tp.values())))
out["push_in"] = dict(worst_overlap=max(v for _, v in push), skirt_enters_sleeve_at_dz=T["sk_top"] - T["sl_bot"],
                      lugs_reach_mouth_at_dz=P["z_lug_top"])

# ---- 3. twist -60 -> 0 with the latch riding the keeper
tw = []
for th in np.arange(-60.0, 0.01, 1.0):
    dz = H.ramp_dz(th); d = 0.0
    while d <= 1.8 and vol(S, H.place(H.flex_latch(L0, d), th, dz)) > EPS:
        d += 0.05
    tp = tail_at(th, dz, d)
    tw.append(dict(theta=th, defl=d, socket_vs_adapter=vol(S, tp["adapter"]), socket_vs_base=vol(S, tp["base"]),
                   socket_vs_latch=vol(S, tp["latch"]), latch_vs_adapter=vol(tp["latch"], tp["adapter"])))
out["twist"] = dict(max_latch_defl=max(t["defl"] for t in tw),
                    worst=dict((k, max(t[k] for t in tw)) for k in ("socket_vs_adapter", "socket_vs_base", "socket_vs_latch", "latch_vs_adapter")),
                    latch_rides_from=min((t["theta"] for t in tw if t["defl"] > 0), default=None))
# latch pushed the full 1.5 mm (release) at home: must clear its own skirt and the sleeve
tp = tail_at(0, 0, 1.6)
out["release_1p6mm"] = dict(latch_vs_adapter=vol(tp["latch"], tp["adapter"]), latch_vs_socket_ring_and_sleeve=vol(S, tp["latch"]))

# ---- 4. rocking: tilt about a line through the far lug seat, find first skirt/sleeve contact
r_seat = 0.5 * (P["r_bore"] + P["r_lug"]); z_seat = H.seat_z(r_seat)
tilt = []
for psi in range(0, 360, 30):
    a = np.radians(psi)
    piv = np.array([r_seat * np.cos(a), r_seat * np.sin(a), z_seat])
    lo, hi = 0.0, 2.0
    if vol(S, tail_at(0, 0, 0, (piv, psi + 90, hi))["adapter"]) <= EPS:
        tilt.append((psi, None)); continue
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        if vol(S, tail_at(0, 0, 0, (piv, psi + 90, mid))["adapter"]) > EPS: hi = mid
        else: lo = mid
    tilt.append((psi, hi))
angs = [t for _, t in tilt if t is not None]
L_lever = 300.0 + 60.0      # tail length + socket face to root joint (mm)
out["rocking"] = dict(per_axis_deg=tilt, worst_deg=max(angs), tip_wobble_mm=np.radians(max(angs)) * L_lever,
                      plain_sff_deg=np.degrees(P["gap"] / 14.0), plain_tip_mm=P["gap"] / 14.0 * L_lever)

# ---- 5. tab access: the tab must be visible straight through both windows at home (radial prism from the tab outward)
a0, a1 = P["az_tooth"][1] - P["tab_deg"], P["az_tooth"][1]
ray = H.sector(P["tab_r1"], T["sl_r1"] + 2, a0, a1, P["flange_bot"] + 0.5, -P["gap"] - 0.5)
out["tab_access"] = dict(blocked_by_sleeve=vol(S, ray), blocked_by_skirt=vol(A, ray),
                         window_w_mm=round(0.5 * (T["sl_r0"] + T["sl_r1"]) * np.radians(T["win_az"][1] - T["win_az"][0]), 1),
                         reach_depth_mm=T["sl_r1"] - P["tab_r1"])

# ---- 6. print overhangs (down-facing faces flatter than 45 deg, not on the bed)
def overhang(m, up, bed_z):
    t = H.to_trimesh(m); n = t.face_normals; c = t.triangles_center; ar = t.area_faces
    up = np.array(up, float)
    down = (n @ up) < -0.72                 # flatter than ~44 deg from horizontal; exact 45 deg walls pass
    onbed = np.abs((c @ up) - bed_z) < 0.05
    sel = down & ~onbed
    return dict(area_mm2=float(ar[sel].sum()), z_levels=sorted(set(np.round((c[sel] @ up), 1).tolist()))[:12])
out["print"] = dict(rump_socket=overhang(S, [0, 0, -1], -P["z_top"]), tail_adapter=overhang(A, [0, 0, 1], T["sf_z"][0]))

# ---- 7. masses and bought parts
out["mass_g"] = dict(rump_socket=S.volume() / 1000 * P["rho"], tail_adapter=A.volume() / 1000 * P["rho"],
                     hand_base=H.hand_base().volume() / 1000 * P["rho"], bought_est=5.0 + 8 * 0.4 + 8 * 1.0)

# ---- 8. loads at the real root offset (socket seat to root joint axis 60 mm)
G = 9.81; W = 30.8 * G; links = np.array([80, 70, 60, 90]) / 1000
SV = {"STS3215": (0.98, 2.94, 0.055), "STS3250": (1.57, 4.90, 0.0745)}; servos = ["STS3250", "STS3250", "STS3215", "STS3215"]
def M_conn(th, payload, tipF, off):
    Pp = [np.zeros(2)]
    for Lk, a in zip(links, th): Pp.append(Pp[-1] + Lk * np.array([-np.cos(a), np.sin(a)]))
    pts = [(0.5 * (Pp[k] + Pp[k + 1]), 0.025) for k in range(4)] + [(Pp[k], SV[servos[k]][2]) for k in range(4)] + [(Pp[4], 0.06 + payload)]
    mom = sum(m * G * (p[0] - off) for p, m in pts) + tipF[1] * (Pp[4][0] - off) - tipF[0] * Pp[4][1]
    return abs(mom)
r_lug = 0.012; F_cap = 94.8 * 5.8
cases = {"held out, 0.3 kg": (np.zeros(4), 0.3, np.zeros(2)), "kickstand straight": (np.radians([-90] * 4), 0, np.array([0, 0.1 * W])),
         "kickstand raked 15 deg": (np.radians([-75] * 4), 0, np.array([0, 0.1 * W]))}
out["loads"] = {}
for n, (th, pl, f) in cases.items():
    m = M_conn(th, pl, f, 0.060); out["loads"][n] = dict(M=m, lug_factor=F_cap / (2 * m / (3 * r_lug)))
m = 4.90; out["loads"]["root servo stalled"] = dict(M=m, lug_factor=F_cap / (2 * m / (3 * r_lug)))

json.dump(out, open("tailmount_checks.json", "w"), indent=1, default=float)
print(json.dumps(out, indent=1, default=lambda v: round(float(v), 3)))
