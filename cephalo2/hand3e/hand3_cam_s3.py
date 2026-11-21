"""hand3 hub, finger rotation decision S3 (2026-11-20): one cam sleeve moves all four finger roots at once, each along its own path,
between four grip layouts.  Replaces F3's per-root motors, gear skirt, track ring, wipers and posts.

Rotor side:
  - cam sleeve (PA-CF resin, r 21.6..23.0) slides axially on the cage; four through-slots, one per root, each a series of axial
    dwells (one per layout: the root is locked by the slot walls) joined by 80 deg helical ramps;
  - each root (D's root + winch leg, AlSi10Mg) has a dia 1.5 steel pin pointing in from a boss on its leg into its slot;
  - the sleeve is moved by an M2 lead screw (lead 0.4, self-locking) in a bracket (AlSi10Mg) bolted under the base disc at az 30,
    where no root ever goes; a brass M2 nut sits in the sleeve's lug; a brass counterweight in the base disc balances the bracket.
Static side: D's 8 curl drivers + a 9th identical driver under the sleeve screw (az 30, rotor parked at 0), one lift ring, two stages:
  lift 2.0 = only the sleeve driver's key is in its screw (roots turn; curl keys stay 1 mm under the legs),
  lift 6.0 = curl keys in the winch screws (the sleeve driver's key goes deeper into its 5.5 mm coupler slot).
Layout order along the sleeve: 2 + 2  ->  cross (flight)  ->  3 + 1  ->  one side.  Root 0 never moves.
Root frame: x radial at az 0, z up.  Units mm.  est. = estimate."""
import numpy as np
import manifold3d as mf
import hand3_design as H
import hand3_hub_v3 as V3
import hand3_hubc as HC
import hand3_multidigit as MD
import hand3_bands as B
import hand3_cad as C
from hand3_finger import cyl, box, app, T, rot
import hand3_rotation_f3 as F3                                   # root_clean(), lift ring helpers (keys re-set below)

M_ = mf.Manifold
V = V3.V

MD.KEY.update(top_retracted=63.0, lift=6.0)
MD.LIFT.update(z=(22.0, 24.0))
STAGE1 = 2.0

LAYOUTS = [("2 + 2", (0, 120, 180, 300)), ("cross", (0, 90, 180, 270)), ("3 + 1", (0, 60, 120, 240)), ("one side", (0, 60, 120, 180))]
SLEEVE = dict(r=(21.6, 23.0), beta=80.0, dwell=0.6, margin=1.0, z_low=58.6, slot_w=1.6, mat="PA-CF resin")
R_C = sum(SLEEVE["r"]) / 2
PIN = dict(d=1.5, r=(21.75, 24.2))
BOSS = dict(x=(23.6, 24.25), y=2.6, dz=1.4)
LEG = dict(MD.LEG, x_in=23.6)
SCREW9 = dict(MD.SCREW, x=27.0, z=(64.0, 90.5))
AZ9 = 30.0
LUG = dict(h=3.0, x=(22.8, 29.2), y=3.0)
BRACKET = dict(back_x=(29.6, 31.0), y=(-4.6, 2.2), top_z=(90.0, 91.0), bot_z=(65.0, 66.0), x0=23.6)
TAB = dict(az=HC._hanger_az(C.FAZ[0]), w=HC.HANGER_W - 0.2, r=(20.3, 21.7), h=3.0)
CW = dict(az=AZ9 + 180.0, r=(16.2, 21.6), half=70.0)          # brass counterweight arc in the base disc (pocket from below, 1.6 deep)


def _profile():
    """Slot path: list of (u, theta_k...) from the top of the slot (u = distance below the slot's top end)."""
    segs = []; u = 0.0
    th = [list(l[1]) for l in LAYOUTS]
    pre_u = []
    for i in range(len(LAYOUTS)):
        segs.append((u, th[i])); pre_u.append(u + SLEEVE["dwell"] / 2); u += SLEEVE["dwell"]; segs.append((u, th[i]))
        if i < len(LAYOUTS) - 1:
            d = max(abs(b - a) for a, b in zip(th[i], th[i + 1]))
            u += R_C * np.radians(d) / np.tan(np.radians(SLEEVE["beta"]))
    return segs, pre_u, u


SEGS, PRE_U, SLOT_LEN = _profile()
STROKE = PRE_U[-1] - PRE_U[0]
SLEEVE_L = SLOT_LEN + 2 * SLEEVE["margin"] + LUG["h"]
PIN_Z = SLEEVE["z_low"] + SLEEVE_L - LUG["h"] - SLEEVE["margin"] - PRE_U[0]        # pin height (sleeve at layout 0)


def theta_at(u):
    """Root angles when the pin sits at slot coordinate u."""
    us = [s[0] for s in SEGS]
    return [float(np.interp(u, us, [s[1][k] for s in SEGS])) for k in range(4)]


def sleeve_dz(u):
    """Sleeve rise (from its lowest position) when the pins sit at slot coordinate u (u grows as the sleeve rises)."""
    return u - PRE_U[0]


def sleeve():
    """Cam sleeve in its lowest position (layout 0), hub frame."""
    z0 = SLEEVE["z_low"]; z1 = z0 + SLEEVE_L
    m = cyl(SLEEVE["r"][1], z0, z1, n=160) - cyl(SLEEVE["r"][0], z0 - 1, z1 + 1, n=160)
    top = z1 - LUG["h"] - SLEEVE["margin"]                                          # slot coordinate 0 sits here
    w = SLEEVE["slot_w"] / 2
    for k in range(4):
        pts = []
        for u in np.linspace(0, SLOT_LEN, 240):
            a = np.radians(theta_at(u)[k]); z = top - u
            pts.append((a, z))
        circ = [(np.cos(q), np.sin(q)) for q in np.linspace(0, 2 * np.pi, 12, endpoint=False)]
        for (a0, z0_), (a1, z1_) in zip(pts[:-1], pts[1:]):
            p = []
            for a, z in ((a0, z0_), (a1, z1_)):
                for r in (SLEEVE["r"][0] - 0.3, SLEEVE["r"][1] + 0.3):
                    for cq, sq in circ:                                         # pin-sized disc in the (tangent, z) plane
                        aa = a + w * cq / R_C
                        p.append([r * np.cos(aa), r * np.sin(aa), z + w * sq])
            m -= M_.hull_points(p)
    # lug for the nut (az 30) and anti-rotation tab (cage hanger slot at az ~17.6)
    lug = box([LUG["x"][0], SCREW9["y"] - LUG["y"], z1 - LUG["h"]], [LUG["x"][1], SCREW9["y"] + LUG["y"], z1])
    lug -= cyl(SCREW9["d"] / 2 + 0.15, z1 - LUG["h"] - 1, z1 + 1, c=(SCREW9["x"], SCREW9["y"], 0), n=20)
    nut = M_.cylinder(1.6, 2.31, 2.31, 6).translate([SCREW9["x"], SCREW9["y"], z1 - LUG["h"] + 0.7])   # M2 hex pocket (4 AF)
    m += (lug - nut).rotate([0, 0, AZ9])
    tab = box([TAB["r"][0], -TAB["w"] / 2, z1 - TAB["h"]], [TAB["r"][1], TAB["w"] / 2, z1]).rotate([0, 0, float(TAB["az"])])
    return m + tab


def nut9():
    z1 = SLEEVE["z_low"] + SLEEVE_L
    return (M_.cylinder(1.6, 2.3, 2.3, 6) - cyl(1.0, -1, 3)).translate([SCREW9["x"], SCREW9["y"], z1 - LUG["h"] + 0.7]).rotate([0, 0, AZ9])


COUPLER9 = dict(od=3.0, h=5.5, slot=0.6)                       # brass tube pinned on the screw end: 5.5 deep key slot


def screw9():
    """M2 lead screw with a slotted brass coupler tube on its lower end: the sleeve driver's key is in it at lift 2 and 6."""
    S = SCREW9; c = (S["x"], S["y"], 0); z0 = S["z"][0]
    m = cyl(S["d"] / 2, z0 + COUPLER9["h"] - 0.5, S["z"][1], c=c, n=20) + cyl(COUPLER9["od"] / 2, z0, z0 + COUPLER9["h"], c=c, n=24)
    m -= box([S["x"] - 2, S["y"] - COUPLER9["slot"] / 2, z0 - 0.1], [S["x"] + 2, S["y"] + COUPLER9["slot"] / 2, z0 + COUPLER9["h"]])
    return m.rotate([0, 0, AZ9])


def bracket():
    """AlSi10Mg bracket under the base disc at az 30: top plate (2 x M2 into the disc), back plate, bottom bearing plate."""
    S = SCREW9; Bk = BRACKET
    top = box([Bk["x0"] + 1.0, Bk["y"][0], Bk["top_z"][0]], [Bk["back_x"][1], Bk["y"][1], Bk["top_z"][1]])
    back = box([Bk["back_x"][0], Bk["y"][0], Bk["bot_z"][0]], [Bk["back_x"][1], Bk["y"][1], Bk["top_z"][1]])
    bot = box([S["x"] - 2.2, Bk["y"][0], Bk["bot_z"][0]], [Bk["back_x"][1], Bk["y"][1], Bk["bot_z"][1]])
    m = top + back + bot
    m -= cyl(S["d"] / 2 + 0.1, Bk["bot_z"][0] - 1, Bk["top_z"][1] + 1, c=(S["x"], S["y"], 0), n=20)
    m -= cyl(COUPLER9["od"] / 2 + 0.1, Bk["bot_z"][0] - 1, Bk["bot_z"][1] + 1, c=(S["x"], S["y"], 0), n=24)        # coupler bearing
    for x in (Bk["x0"] + 2.2, ):
        m -= cyl(0.8, Bk["top_z"][0] - 1, Bk["top_z"][1] + 1, c=(x, S["y"] - 2.2, 0), n=12)
    return m.rotate([0, 0, AZ9])


def base_disc_s3():
    """F3's clean base disc without the gear skirt, with the counterweight pocket (from below)."""
    m = F3.base_disc_f3() - F3.skirt()
    return m - _cw_solid(0.05)


def _cw_solid(clr=0.0):
    z0 = V["z0"] - clr
    a0, a1 = CW["az"] - CW["half"], CW["az"] + CW["half"]
    return V3.wedge(CW["r"][0] - clr, CW["r"][1] + clr, z0, z0 + 1.6 + clr, -CW["half"] - clr, CW["half"] + clr).rotate([0, 0, CW["az"]])


def counterweight():
    return _cw_solid(0.0)


def leg_s3():
    """D's winch leg, inner face trimmed to x 23.6 (sleeve clearance) up to z 90.5, with a pin boss between the cheeks."""
    m = MD.leg() - box([-50, -50, 0], [LEG["x_in"], 50, MD.LEG["z_rim"]])
    boss = box([BOSS["x"][0], -BOSS["y"], PIN_Z - BOSS["dz"]], [BOSS["x"][1], BOSS["y"], PIN_Z + BOSS["dz"]])
    m += boss
    m -= cyl(PIN["d"] / 2 - 0.02, PIN["r"][0] - 1, PIN["r"][1] - 0.4, "x", c=(0, 0, PIN_Z), n=16)                   # press-fit bore
    return m


def pin():
    return cyl(PIN["d"] / 2, PIN["r"][0], PIN["r"][1] - 0.4, "x", c=(0, 0, PIN_Z), n=16)


_RT = {}


def root_s3(az=0.0):
    if "m" not in _RT:
        S = MD.SCREW
        m = F3.root_clean() + leg_s3() - cyl(S["d"] / 2 + 0.15, S["z"][0] - 1, S["z"][1], c=(S["x"], S["y"], 0), n=20)
        xp = B.hooks(0)[0]
        m -= app(B.channel(0, xp - 1.6, H.KN[0]) - box([-50, -5, -50], [xp - 2.0, 5, 50]), T(rot("x", H.PITCH))).translate([0, 0, H.Z["blade"]])
        _RT["m"] = m
    return _RT["m"].rotate([0, 0, float(az)])


KEY9 = dict(len=7.0)                                             # long key: still in the coupler slot at lift 6 (enters at lift 1)


def driver9(lifted=0.0):
    """The 9th driver (same Pololu 30:1), under the sleeve screw at az 30, with a 7 mm key so its coupler stays clear of the
    screw's coupler at lift 6.  Sits 4 mm lower than the curl drivers, in a pocket through the lift ring (held by two tabs, est.)."""
    K = MD.KEY; N = MD.N20; x = SCREW9["x"]; y0 = SCREW9["y"]; kt = K["top_retracted"] + lifted
    coup = cyl(2.0, kt - KEY9["len"] - 2.0, kt - KEY9["len"], c=(x, y0, 0), n=20)
    key = box([x - 0.9, y0 - K["t"] / 2, kt - KEY9["len"] - 0.01], [x + 0.9, y0 + K["t"] / 2, kt])
    zb = kt - KEY9["len"] - 2.0 - N["shaft"]
    shaft = cyl(1.5, zb, kt - KEY9["len"] - 2.0, c=(x, y0, 0), n=16)
    body = box([x - N["gx"] / 2, y0 - N["gy"] / 2, zb - N["L"]], [x + N["gx"] / 2, y0 + N["gy"] / 2, zb])
    tabs = box([x - N["gx"] / 2 - 1.5, y0 - 1.5, MD.LIFT["z"][1] + lifted], [x + N["gx"] / 2 + 1.5, y0 + 1.5, MD.LIFT["z"][1] + lifted + 1.0])
    return (body + shaft + coup + key + tabs).rotate([0, 0, AZ9])


def driver9_dz(lift):
    return lift                                                                       # rides the lift ring like the others


def frame_s3():
    f = HC._parts()["frame"][0]; S = MD.SCREW; N20 = MD.N20; LI = MD.LIFT; K = MD.KEY
    cut = None
    for a, x in [(a, S["x"]) for a in MD.DRIVER_AZ] + [(AZ9, SCREW9["x"])]:
        p = box([x - N20["gx"] / 2 - 0.3, S["y"] - N20["gy"] / 2 - 0.3, LI["z"][0] - 0.3], [x + N20["gx"] / 2 + 0.3, S["y"] + N20["gy"] / 2 + 0.3, 60.0]).rotate([0, 0, float(a)])
        cut = p if cut is None else cut + p
    cut += cyl(LI["r"][1] + 0.3, LI["z"][0] - 0.3, LI["z"][1] + K["lift"] + 0.3, n=96) - cyl(LI["r"][0] - 0.3, 0, 60, n=96)
    cut += _d9_pocket(0.3, z0=10.0)
    return f - cut


def _d9_pocket(clr, z0=-50.0):
    N = MD.N20; x = SCREW9["x"]; y0 = SCREW9["y"]
    return box([x - N["gx"] / 2 - clr, y0 - N["gy"] / 2 - clr, z0], [x + N["gx"] / 2 + clr, y0 + N["gy"] / 2 + clr, 60.0]).rotate([0, 0, AZ9])


def lift_ring_s3():
    return MD.lift_ring(False) - _d9_pocket(0.3)


def numbers():
    W = 1309.0
    rho_al, rho_pa, rho_br, rho_st = 2.68e-3, 1.25e-3, 8.5e-3, 7.85e-3
    tb = np.tan(np.radians(SLEEVE["beta"])); phi = np.arctan(0.15)
    eta = np.tan(np.radians(90 - SLEEVE["beta"])) / np.tan(np.radians(90 - SLEEVE["beta"]) + phi)
    F_t = 3.0                                                                   # tangential push per moving root (est.)
    F_axial_root = F_t * tb / eta
    moving = 3
    F_ax = moving * F_axial_root
    T_screw = F_ax * SCREW9["lead"] * 1e-3 / (2 * np.pi * 0.30)               # M2, eta 0.30 (est.)
    stall = MD.N20["stall_kgcm"] * 0.0981
    mm_s = 0.75 * MD.N20["rpm"] / 60 * SCREW9["lead"]
    steps = []
    for i in range(len(LAYOUTS) - 1):
        steps.append(dict(frm=LAYOUTS[i][0], to=LAYOUTS[i + 1][0], mm=round(PRE_U[i + 1] - PRE_U[i], 2),
                          s=round((PRE_U[i + 1] - PRE_U[i]) / mm_s, 2)))
    m_br = bracket().volume() * rho_al; m_scr = screw9().volume() * rho_st; m_nut = 0.2
    r_b = 28.0
    m_lug = LUG["h"] * (LUG["x"][1] - LUG["x"][0]) * 2 * LUG["y"] * rho_pa
    unbal = (m_br + m_scr + m_nut) * r_b + m_lug * 26.0
    m_cw_need = unbal / (sum(CW["r"]) / 2)
    m_cw = counterweight().volume() * rho_br
    F_br = (m_br + m_scr + m_nut) * 1e-3 * r_b * 1e-3 * W ** 2
    bear = F_axial_root / (PIN["d"] * (SLEEVE["r"][1] - SLEEVE["r"][0]))
    m_sl = sleeve().volume() * rho_pa
    return dict(layouts=[l[0] for l in LAYOUTS], slot_len=round(SLOT_LEN, 2), stroke=round(STROKE, 2), sleeve_len=round(SLEEVE_L, 2),
                sleeve_z=(SLEEVE["z_low"], round(SLEEVE["z_low"] + SLEEVE_L + STROKE, 2)), pin_z=round(PIN_Z, 2), beta=SLEEVE["beta"],
                slot_efficiency=round(eta, 2), axial_per_root_N=round(F_axial_root, 1), axial_total_N=round(F_ax, 1),
                screw_torque_Nm=round(T_screw, 4), driver_stall_Nm=round(stall, 4), torque_margin=round(stall / T_screw, 2),
                sleeve_mm_s=round(mm_s, 2), steps=steps, pin_bearing_MPa=round(bear, 1),
                bracket_g=round(m_br, 2), bracket_set_g=round(m_br + m_scr + m_nut, 2), bracket_spin_N=round(F_br),
                counterweight_need_g=round(m_cw_need, 2), counterweight_g=round(m_cw, 2),
                residual_unbalance_N=round(abs(unbal - m_cw * (sum(CW["r"]) / 2 * np.sinc(np.radians(CW["half"]) / np.pi))) * 1e-6 * W ** 2, 1), sleeve_g=round(m_sl, 2),
                root_AlSi10Mg_g=round(root_s3(0).volume() * rho_al, 2),
                hold="at a layout the pin sits in an axial dwell: root torque goes into the slot wall, not the screw")
