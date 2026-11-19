"""Turret cameras for the Scabbard turret (Cephalo-1, 2026-11-15).  Single source of truth for the camera bays.

Ask (Forest): add cameras for vision inside the existing "bucket" that stores the blades = the Scabbard turret over the
cage (scabbard.py: r 90 mm, top z 224).  The turret sits where the four eyestalk sockets were (r 80, z 112, at the
own-scabbard azimuths 0/90/180/270), so with the turret fitted these cameras are the robot's head vision.

Layout (mm, body frame of scabbard.py: origin = centre of the plate underside, +z up):
  * 4 SIDE bays, one per guest-socket sector (azimuth 45 + 90 k), above the guest sockets (top z 82) and outside the
    elevator ring (r 44-53).  Each is a printed PETG cradle hanging from the cage's top ring (r 56-82, z 212-215),
    holding a Seeed XIAO ESP32S3 Sense with the OV5640 camera (+ wide lens, est.) looking radially out through a
    window in the turret wall.  Lens centre r 87, z 190; the 21 mm board length runs along z.
  * 1 ZENITH bay at the centre of the turret top, hanging from a printed bridge across the top ring (on the 45 deg
    diagonal, between the elevator fingers), looking straight up through a window in the turret roof.
  * The lower part of each guest sector (z 82-170) and the inboard strip r 54-63 stay free: battery candidates
    (scabbard-dock.md open item 1) and the elevator belt loops (not drawn in scabbard.py).
  * Windows: dia 16 hole in the 1.6 mm shell with a printed bezel and a 20 x 1 mm clear polycarbonate disc sealed in.

python3 turretcams.py check     clearances on the solids (stow / dock / flight elevator sweep) -> turretcams_check.json
python3 turretcams.py cover     coverage on the sphere (stand / flight poses, tilt options, + tip cameras) -> turretcams_cover.json
python3 turretcams.py stl       STLs in turretcams_stl/
"""
import os, sys, json
import numpy as np
import manifold3d as m3
import scabbard as S
from scabbard import bx, zcyl, xcyl, ycyl, union, rz, hull

AZ_SIDE = [45.0, 135.0, 225.0, 315.0]
# XIAO ESP32S3 Sense stack (XIAO + Sense expansion board + camera): 21 x 17.8 x 15 mm (Seeed datasheet); the OV5640
# module is an 8.5 x 8.5 mm block, lens barrel taken as dia 8 x 4 proud (est.); heat sink 7 x 6 x 7 behind the sensor
XIAO = dict(L=21.0, W=17.8, T=15.0)
LENS_D, LENS_H, MOD = 8.0, 4.0, 8.5
HS = 7.0
SHELL_T = 1.6
R_IN = S.TURRET_R - SHELL_T     # turret wall inner radius 88.4
R_DISC = 86.6                   # outer face of the window disc (flat, vertical); the bezel's back is cut to the wall's curve
R_LENS = 84.1                   # lens front of a side camera: the tilted lens rim stays 0.4 mm behind the disc (85.6)
Z_CAM = 186.0                   # lens centre height (the turret fillet starts at 202)
TILT = -15.0                    # side camera tilt (deg, + = up), picked from `cover` (turretcams.md)
CONE = 55.0                     # half-angle of the wide lens cone (120 deg class, est.; same as tipcams4.py)
Z_ZEN = 221.0                   # zenith lens front (roof disc 221.4-222.4, roof outer 224)
# windows sized to the cone through the shell: radius >= lens radius + depth x tan(cone +- tilt)
WIN_SIDE = dict(d=31.0, dz=-5.5, disc=(35.0, 1.0))
WIN_ROOF = dict(d=18.0, disc=(22.0, 1.0))
M_CAM = 0.009          # XIAO Sense + OV5640 + antenna (eyestalks.md mass table, est.)


# ------------------------------------------------------------------ parts, built at azimuth 0 (x radial out), placed by rz
def side_stack():
    """Camera stack in its own place: lens front at x = R_LENS, z = Z_CAM.  Returns (board, module+lens, heat sink)."""
    x_mod = R_LENS - LENS_H; x_brd = x_mod - MOD + 2.0        # module 6.5 deep sits on the board face (est.)
    x0 = x_brd - (XIAO["T"] - 6.5)
    brd = bx(x0, x_brd, -XIAO["W"] / 2, XIAO["W"] / 2, Z_CAM - XIAO["L"] / 2, Z_CAM + XIAO["L"] / 2)
    mod = bx(x_brd, x_mod, -MOD / 2, MOD / 2, Z_CAM - MOD / 2, Z_CAM + MOD / 2) + xcyl(LENS_D / 2, x_mod, R_LENS, 0, Z_CAM, 24)
    hs = bx(x0 - HS, x0, -7.0, 7.0, Z_CAM - 7.0, Z_CAM + 7.0)
    return brd, mod, hs


Z_LUG = (S.Z_CAGE_TOP - 5.2, S.Z_CAGE_TOP - 0.2)      # cradle lug under the cage top ring (ring z 212-215)
R_SCREW, Y_SCREW = 66.0, 7.0                           # 2 x M2.5 per side sector: down through the zenith bridge (45/225 only), the ring, into heat-set inserts in the lug


def tilt_side(m, tilt=None):
    """Tilt a side part about the tangential axis through the lens centre (azimuth 0 frame)."""
    t = TILT if tilt is None else tilt
    return m.translate([-R_LENS, 0, -Z_CAM]).rotate([0, -t, 0]).translate([R_LENS, 0, Z_CAM]) if t else m


def side_holder():
    """Board holder (tilted with the camera): back plate, two side rails with front lips, floor with a cable exit."""
    brd, mod, hs = side_stack()
    bb = np.asarray(brd.bounding_box()).reshape(-1)
    x0b, x1b = bb[0], bb[3]
    xh = x0b - HS
    back = bx(xh - 2.0, xh, -12.0, 12.0, Z_CAM - 7.0, Z_CAM + 14.0)      # stops at the heat sink's lower edge: the tilt swings its foot inboard, toward the elevator ring
    parts = [back]
    for s_ in (1, -1):
        y0, y1 = sorted((s_ * (XIAO["W"] / 2 + 0.2), s_ * (XIAO["W"] / 2 + 2.2)))
        parts.append(bx(xh, x1b - 1.0, y0, y1, Z_CAM - XIAO["L"] / 2 - 2.0, Z_CAM + XIAO["L"] / 2 + 2.0))
        yl0, yl1 = sorted((s_ * (XIAO["W"] / 2 - 1.0), s_ * (XIAO["W"] / 2 + 0.2)))
        parts.append(bx(x1b - 2.2, x1b - 1.0, yl0, yl1, Z_CAM - XIAO["L"] / 2 - 2.0, Z_CAM + XIAO["L"] / 2 + 2.0))
    parts.append(bx(x0b, x1b - 1.0, -XIAO["W"] / 2 - 2.2, XIAO["W"] / 2 + 2.2, Z_CAM - XIAO["L"] / 2 - 2.0, Z_CAM - XIAO["L"] / 2 - 0.2))
    h = union(parts) - bx(xh - 3, x1b, -4.0, 4.0, Z_CAM - XIAO["L"] / 2 - 3, Z_CAM - XIAO["L"] / 2 + 0.1)
    return h, xh


def side_cradle(tilt=None):
    """One PETG print: a level lug under the cage top ring (2 x M2.5 from above through the ring into heat-set inserts),
    a strap down to the board holder, the holder tilted with the camera."""
    h, xh = side_holder()
    ht = tilt_side(h, tilt)
    lug = bx(58.5, 76.0, -12.0, 12.0, Z_LUG[0], Z_LUG[1])
    top_of_back = tilt_side(bx(xh - 2.0, xh, -12.0, 12.0, Z_CAM + 12.0, Z_CAM + 14.0), tilt)
    strap = hull(bx(58.5, 61.0, -12.0, 12.0, Z_LUG[0], Z_LUG[0] + 0.5), top_of_back)
    c = ht + lug + strap
    for y in (-Y_SCREW, Y_SCREW):
        c = c - zcyl(1.75, Z_LUG[0] + 0.8, Z_LUG[1] + 1, 16, R_SCREW, y)    # insert bore (M2.5 x 4 insert, dia 3.5)
    return c


def zen_stack():
    z_mod = Z_ZEN - LENS_H; z_brd = z_mod - MOD + 2.0
    z0 = z_brd - (XIAO["T"] - 6.5)
    brd = bx(-XIAO["L"] / 2, XIAO["L"] / 2, -XIAO["W"] / 2, XIAO["W"] / 2, z0, z_brd)
    mod = bx(-MOD / 2, MOD / 2, -MOD / 2, MOD / 2, z_brd, z_mod) + zcyl(LENS_D / 2, z_mod, Z_ZEN, 24)
    hs = bx(-7, 7, -7, 7, z0 - HS, z0)
    return brd, mod, hs


def zen_bridge():
    """Bridge across the top ring on the 45/225 deg diagonal (between the elevator fingers at 0/90/180/270) with a
    tray under the roof for the zenith camera; bolts to the ring with 2 x M2.5 at each end."""
    brd, mod, hs = zen_stack()
    zb = np.asarray(hs.bounding_box()).reshape(-1)[2]
    bar = bx(-56.0, 56.0, -6.0, 6.0, S.Z_CAGE_TOP + 3.0, S.Z_CAGE_TOP + 6.0) \
        + bx(56.0, 74.0, -10.0, 10.0, S.Z_CAGE_TOP + 3.0, S.Z_CAGE_TOP + 6.0) + bx(-74.0, -56.0, -10.0, 10.0, S.Z_CAGE_TOP + 3.0, S.Z_CAGE_TOP + 6.0)
    for xs in (R_SCREW, -R_SCREW):
        for y in (-Y_SCREW, Y_SCREW):
            bar = bar - zcyl(1.4, S.Z_CAGE_TOP, S.Z_CAGE_TOP + 7, 16, xs, y)        # shares the side cradle's screws
    tray = bx(-14.0, 14.0, -12.5, 12.5, zb - 2.0, zb) + bx(-14.0, -12.0, -12.5, 12.5, zb - 2.0, S.Z_CAGE_TOP + 6.0) \
        + bx(12.0, 14.0, -12.5, 12.5, zb - 2.0, S.Z_CAGE_TOP + 6.0)
    walls = bx(-12.0, 12.0, -12.5, -10.9, zb, S.Z_CAGE_TOP + 6.0) + bx(-12.0, 12.0, 10.9, 12.5, zb, S.Z_CAGE_TOP + 6.0)
    m = bar + tray + walls
    m = m - bx(-11.0, 11.0, -9.6, 9.6, zb, S.Z_CAGE_TOP + 7.0) - bx(-4, 4, -13, -9, zb - 3, zb + 4)   # pocket + cable exit
    return rz(m, 45.0)


def window_cut(side=True):
    if side:
        return xcyl(WIN_SIDE["d"] / 2, R_IN - 3, S.TURRET_R + 2, 0, Z_CAM + WIN_SIDE["dz"], 48)
    return zcyl(WIN_ROOF["d"] / 2, S.Z_TURRET - 6, S.Z_TURRET + 2, 48)


def bezel(side=True):
    """Printed ring inside the shell that holds the clear PC disc (sealed with silicone).  Side: flat disc seat, back cut
    to the wall's inner curve so the ring beds on the wall."""
    if side:
        d, t = WIN_SIDE["disc"]; zc = Z_CAM + WIN_SIDE["dz"]
        disc = xcyl(d / 2, R_DISC - t, R_DISC, 0, zc, 48)
        r = xcyl(d / 2 + 2.0, R_DISC - t - 1.2, R_IN + 2, 0, zc, 48) - xcyl(WIN_SIDE["d"] / 2 - 0.5, R_IN - 12, R_IN + 3, 0, zc, 48)
        r = (r - disc) ^ zcyl(R_IN - 0.05, zc - 30, zc + 30, 180)
        return r, disc
    d, t = WIN_ROOF["disc"]; z1 = S.Z_TURRET - SHELL_T
    disc = zcyl(d / 2, z1 - t, z1, 48)
    r = zcyl(d / 2 + 2.0, z1 - t - 1.2, z1 - 0.05, 48) - zcyl(WIN_ROOF["d"] / 2 - 0.5, z1 - 10, z1 + 1, 48) - disc
    return r, disc


def place(m, az, tilt=None):
    return rz(tilt_side(m, tilt), az)


def assembly(tilt=None):
    out = {}
    for k, az in enumerate(AZ_SIDE):
        brd, mod, hs = side_stack()
        out[f"cam{k}_board"] = place(brd, az, tilt); out[f"cam{k}_module"] = place(mod, az, tilt); out[f"cam{k}_heatsink"] = place(hs, az, tilt)
        out[f"cam{k}_cradle"] = rz(side_cradle(tilt), az)
        b, d = bezel(True); out[f"cam{k}_bezel"] = rz(b, az); out[f"cam{k}_disc"] = rz(d, az)
    brd, mod, hs = zen_stack()
    out["zen_board"], out["zen_module"], out["zen_heatsink"] = rz(brd, 45), rz(mod, 45), rz(hs, 45)
    out["zen_bridge"] = zen_bridge()
    b, d = bezel(False); out["zen_bezel"], out["zen_disc"] = b, d
    return out


def shell_with_windows():
    sh = S.mantle_solid(True, SHELL_T)
    cuts = [rz(window_cut(True), az) for az in AZ_SIDE] + [window_cut(False)]
    return sh - union(cuts)


def cam_frames(tilt=None):
    tilt = TILT if tilt is None else tilt
    """[(lens position m, optical axis)] for the 4 side cameras + zenith, body frame of cephalo4 (m)."""
    out = []
    for az in AZ_SIDE:
        a = np.radians(az); t = np.radians(tilt)
        n = np.array([np.cos(t) * np.cos(a), np.cos(t) * np.sin(a), np.sin(t)])
        out.append((np.array([R_LENS * np.cos(a), R_LENS * np.sin(a), Z_CAM]) * 1e-3, n))
    out.append((np.array([0, 0, Z_ZEN]) * 1e-3, np.array([0, 0, 1.0])))
    return out


# ------------------------------------------------------------------ checks on the solids
def window_margin():
    """Rays of the cone edge from the lens rim to the outer wall surface must pass inside the shell hole (side) / roof hole."""
    t = np.radians(TILT); worst = 1e9
    for ang in np.radians(np.arange(0, 360, 5)):
        for rim in np.radians(np.arange(0, 360, 30)):
            # cone edge direction in the camera frame (x = axis), then tilted about y
            d = np.array([np.cos(np.radians(CONE)), np.sin(np.radians(CONE)) * np.cos(ang), np.sin(np.radians(CONE)) * np.sin(ang)])
            d = np.array([d[0] * np.cos(t) - d[2] * np.sin(t), d[1], d[0] * np.sin(t) + d[2] * np.cos(t)])
            p0 = np.array([R_LENS, LENS_D / 2 * np.cos(rim), Z_CAM + LENS_D / 2 * np.sin(rim)])
            # march to the outer cylinder r 90
            for s_ in np.linspace(0, 20, 401):
                p = p0 + d * s_
                if np.hypot(p[0], p[1]) >= S.TURRET_R:
                    break
            for s2 in np.linspace(0, s_, 60):
                p = p0 + d * s2
                if np.hypot(p[0], p[1]) >= R_IN:                       # inside the shell thickness: must be inside the hole
                    off = np.hypot(p[1], p[2] - (Z_CAM + WIN_SIDE["dz"]))
                    worst = min(worst, WIN_SIDE["d"] / 2 - off)
    zen = WIN_ROOF["d"] / 2 - (LENS_D / 2 + (S.Z_TURRET - Z_ZEN) * np.tan(np.radians(CONE)))
    return dict(side=round(float(worst), 2), roof=round(float(zen), 2))


def to_tm(man):
    import trimesh
    me = man.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)


def gap(a, b, n=6000):
    """Smallest distance between two manifolds (mm): 0 or less when they touch; sampled both ways."""
    import trimesh
    if (a ^ b).volume() > 1e-3:
        return -float((a ^ b).volume())
    ta, tb = to_tm(a), to_tm(b)
    pa = np.vstack([ta.vertices, trimesh.sample.sample_surface(ta, n, seed=1)[0]])
    pb = np.vstack([tb.vertices, trimesh.sample.sample_surface(tb, n, seed=2)[0]])
    da = trimesh.proximity.ProximityQuery(tb).on_surface(pa)[1].min()
    db = trimesh.proximity.ProximityQuery(ta).on_surface(pb)[1].min()
    return float(min(da, db))


def check():
    A = assembly()
    cams = union([v for k, v in A.items() if not k.endswith("_disc") and not k.endswith("_bezel")])
    side = union([v for k, v in A.items() if k.startswith("cam") and not k.endswith(("_disc", "_bezel"))])
    zen = union([v for k, v in A.items() if k.startswith("zen") and not k.endswith(("_disc", "_bezel"))])
    shell = shell_with_windows()
    st = S.static(beta_rod=S.BETA_H, u_elev=0.0)
    res = {}
    res["side bays to cage"] = gap(side - union([A[f"cam{k}_cradle"] for k in range(4)]), S.cage_solid())
    res["cradles to cage (lug 0.2 under the top ring)"] = gap(union([A[f"cam{k}_cradle"] for k in range(4)]), S.cage_solid())
    res["zenith bay + bridge to cage (bridge sits on the ring: 0 expected)"] = gap(zen, S.cage_solid())
    res["zenith bridge to side cradles (ring between them)"] = gap(A["zen_bridge"], union([A[f"cam{k}_cradle"] for k in range(4)]))
    res["camera parts to turret shell"] = gap(cams, shell)
    res["window check: side cone clear of the shell hole (worst margin mm)"] = window_margin()
    res["side lens front to window disc"] = gap(union([A[f"cam{k}_module"] for k in range(4)]), union([A[f"cam{k}_disc"] for k in range(4)]))
    res["zenith lens front to roof disc"] = gap(A["zen_module"], A["zen_disc"])
    res["bezels to cradles"] = gap(union([A[f"cam{k}_bezel"] for k in range(4)]), union([A[f"cam{k}_cradle"] for k in range(4)]))
    res["camera parts to fold ring + rods"] = gap(cams, st["fold_ring"] + st["rods"])
    res["camera parts to latch pins"] = gap(cams, st["pins"])
    # elevator through its whole stroke (stow 194, dock 86, seated 0) and every 10 mm between
    worst = (1e9, None)
    for u in list(np.arange(0.0, S.U_STOW + 0.1, 10.0)) + [S.U_DOCK, S.U_STOW]:
        g = gap(cams, S.elevator_solid(u), n=2500)
        if g < worst[0]:
            worst = (g, float(u))
    res["camera parts to elevator, whole stroke (worst, at u)"] = worst
    # stowed blades in their scabbards (blades never enter the guest sectors above z 82)
    bl = S.blade_solid(holes=False)
    blades = union([S.place_blade(bl, k, S.BETA_H, S.U_STOW) for k in range(4)])
    res["camera parts to stowed blades"] = gap(cams, blades)
    # guest-sector volume left free under the cameras
    zlow = min(np.asarray(A[f"cam0_cradle"].bounding_box()).reshape(-1)[2], np.asarray(A["cam0_heatsink"].bounding_box()).reshape(-1)[2])
    res["guest sector left free below the cameras (z from guest socket top to cradle bottom, mm)"] = [S.GUEST_H, round(float(zlow), 1)]
    r_in = np.hypot(*np.asarray(A["cam0_cradle"].bounding_box()).reshape(-1)[[0, 1]])
    res["side bay inner edge radius (inboard strip left for belts, mm)"] = round(float(np.asarray(rz(A['cam0_cradle'], -45).bounding_box()).reshape(-1)[0]), 1)
    # printed masses
    rho = S.RHO_M["petg"]
    m_cr = S.mass_props(side_cradle(), rho)[0]; m_br = S.mass_props(zen_bridge(), rho)[0]
    m_bz = (4 * S.mass_props(bezel(True)[0], rho)[0] + S.mass_props(bezel(False)[0], rho)[0]) / 5; m_disc = (4 * np.pi * (WIN_SIDE["disc"][0] / 2) ** 2 + np.pi * (WIN_ROOF["disc"][0] / 2) ** 2) * 1.0e-9 * 1200 / 5
    res["mass_g"] = dict(cameras=5 * M_CAM * 1e3, cradles=4 * m_cr * 1e3, bridge=m_br * 1e3, bezels=5 * m_bz * 1e3, discs=5 * m_disc * 1e3,
                         screws_wire=12.0)
    res["mass_g"]["total"] = sum(res["mass_g"].values())
    for k, v in res.items():
        print(f"  {k:70s} {v if isinstance(v, (list, tuple, dict)) else round(v, 2)}")
    json.dump(res, open("turretcams_check.json", "w"), indent=1, default=float)
    return res


# ------------------------------------------------------------------ coverage on the sphere
def cover():
    import cephalo4 as C
    P4 = json.load(open("poses4.json"))
    D = sphere_dirs(4000)
    el = np.degrees(np.arcsin(D[:, 2]))
    bands = {"sphere": np.ones(len(D), bool), "horizon +-30": np.abs(el) <= 30, "above +30": el > 30, "below -30": el < -30}
    res = {}
    for pose in ("stand", "flight"):
        q = np.radians(P4[pose]); qa = np.tile(q, (8, 1))
        Ps, rs = [], []
        for i in range(8):
            P, r, (Rt, pt) = C.arm_capsules(i, qa[i], np.zeros(3))
            Ps.append(P); rs.append(r)
            pod_r = 0.060 if i in (2, 6) else 0.045                # hand3 on arms 2 and 6, XL claw pods elsewhere (spheres, est.)
            for s in (0.03, 0.07):
                Ps.append((pt + Rt[:, 0] * s)[None]); rs.append(np.array([pod_r]))
        P, r = np.vstack(Ps), np.concatenate(rs)
        tips = []
        for i in range(8):
            segs, _ = C.arm_fk(i, qa[i])
            p, R = C.at_s(segs, 0.466); nv = -R[:, 2]
            tips.append((p + nv * 0.018, np.cos(np.radians(35)) * nv - np.sin(np.radians(35)) * R[:, 0]))
        res[pose] = {}
        for tilt in (-15.0, 0.0, 15.0):
            for zen in (False, True):
                frames = cam_frames(tilt)[:4] + (cam_frames(tilt)[4:] if zen else [])
                seen = np.zeros(len(D), bool)
                for o, n in frames:
                    inside = D @ n >= np.cos(np.radians(CONE))
                    if inside.any():
                        seen[np.where(inside)[0]] |= ~blocked(o + n * 0.004, D[inside], P, r)
                tp = np.zeros(len(D), bool)
                for o, n in tips:
                    inside = D @ n >= np.cos(np.radians(CONE))
                    near = np.linalg.norm(P - o, axis=1) < 0.045
                    if inside.any():
                        tp[np.where(inside)[0]] |= ~blocked(o + n * 0.006, D[inside], P[~near], r[~near])
                key = f"tilt {tilt:+.0f}" + (" + zenith" if zen else "")
                res[pose][key] = {b: [round(100 * seen[s].mean(), 1), round(100 * (seen | tp)[s].mean(), 1)] for b, s in bands.items()}
                print(f"  {pose:7s} {key:18s} " + " | ".join(f"{b} {v[0]:5.1f} (+tips {v[1]:5.1f})" for b, v in res[pose][key].items()))
    json.dump(res, open("turretcams_cover.json", "w"), indent=1)
    return res


RAY = 1.5


def masks(pose, tilt=None, zen=True, n=4000):
    """Directions, (turret-seen, tip-seen) masks and per-camera masks for one pose of poses4.json (all arms alike)."""
    import cephalo4 as C
    q = np.radians(json.load(open("poses4.json"))[pose]); D = sphere_dirs(n)
    Ps, rs, tips = [], [], []
    for i in range(8):
        P, r, (Rt, pt) = C.arm_capsules(i, q, np.zeros(3)); Ps.append(P); rs.append(r)
        for s_ in (0.03, 0.07):
            Ps.append((pt + Rt[:, 0] * s_)[None]); rs.append(np.array([0.060 if i in (2, 6) else 0.045]))
        segs, _ = C.arm_fk(i, q); p, R = C.at_s(segs, 0.466); nv = -R[:, 2]
        tips.append((p + nv * 0.018, np.cos(np.radians(35)) * nv - np.sin(np.radians(35)) * R[:, 0]))
    P, r = np.vstack(Ps), np.concatenate(rs)
    fr = cam_frames(tilt); fr = fr if zen else fr[:4]
    per = []
    for o, nv in fr:
        m = np.zeros(len(D), bool); inside = D @ nv >= np.cos(np.radians(CONE))
        m[np.where(inside)[0]] = ~blocked(o + nv * 0.004, D[inside], P, r); per.append(m)
    tp = np.zeros(len(D), bool)
    for o, nv in tips:
        inside = D @ nv >= np.cos(np.radians(CONE)); near = np.linalg.norm(P - o, axis=1) < 0.045
        tp[np.where(inside)[0]] |= ~blocked(o + nv * 0.006, D[inside], P[~near], r[~near])
    return D, np.any(per, 0), tp, per


def sphere_dirs(n=4000):
    i = np.arange(n) + 0.5
    z = 1 - 2 * i / n; a = np.pi * (1 + 5 ** 0.5) * i
    rr = np.sqrt(1 - z * z)
    return np.stack([rr * np.cos(a), rr * np.sin(a), z], 1)


def body_hit(Q):
    """True where points Q (m) are inside the body: rev 4 dome ellipsoid, plate, turret cylinder (r 90, to z 224)."""
    x, y, z = Q[..., 0], Q[..., 1], Q[..., 2]
    rr = np.hypot(x, y)
    dome = (rr / 0.125) ** 2 + (np.clip(z - 0.006, 0, None) / 0.150) ** 2 < 1.0
    dome &= z > -0.002
    plate = (rr < 0.125) & (z > -0.001) & (z < 0.007)
    tur = (rr < S.TURRET_R * 1e-3 - 0.001) & (z < S.Z_TURRET * 1e-3 - 0.001) & (z > 0)
    hub = (rr < 0.075) & (z < 0) & (z > -0.034)
    return dome | plate | tur | hub


def blocked(o, Dd, P, r):
    v = P - o
    t = np.clip(Dd @ v.T, 0.0, RAY)
    d2 = ((v[None] - t[..., None] * Dd[:, None, :]) ** 2).sum(-1)
    hit = (d2 < r[None] ** 2).any(1)
    ts = np.linspace(0.004, RAY, 120)
    Q = o[None, None] + ts[None, :, None] * Dd[:, None, :]
    return hit | body_hit(Q).any(1)


def export_stl(folder="turretcams_stl"):
    os.makedirs(folder, exist_ok=True)
    parts = {"tcam_side_cradle_PETG": side_cradle(), "tcam_zenith_bridge_PETG": zen_bridge(), "tcam_bezel_side_PETG": bezel(True)[0],
             "tcam_bezel_roof_PETG": bezel(False)[0], "scab_mantle_turret_windows_PETG": shell_with_windows()}
    for n, m in parts.items():
        t = to_tm(m); t.export(f"{folder}/{n}.stl")
        print(f"  {n:34s} watertight {t.is_watertight} bodies {len(m.decompose())} bbox {np.round(t.extents, 1)} vol {t.volume/1000:.1f} cm3")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "check"
    {"check": check, "cover": cover, "stl": export_stl}[cmd]()
