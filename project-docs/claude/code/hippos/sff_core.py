"""SFF connector core geometry (copy of claude/code/handmount.py, geometry unchanged; socket cuts split out as socket_cuts()).


Connector frame (mm): z = connector axis, +z goes up into the arm (the hand hangs toward -z); z = 0 is the socket
mouth face. x = arm x (toward the tip), so the socket's mount plate runs along the arm.
Angles in degrees about +z. The hand locks by turning +TWIST degrees about +z (clockwise seen from below the arm).
The hand is drawn at home (locked); place(part, theta, dz) puts it at theta = -TWIST..0 and dz below home.

Parts (printed, PETG; PA-CF optional):
  arm_socket  ring 32 mm + mount plate, bayonet lips with a draw-in ramp, keeper for the latch, pin-board pocket
  hand_base   the base every hand shares: plug with 3 lugs (one wide = key), O-ring groove, pad-board well,
              spring latch, 4 x M3 insert holes for the hand
Bought: 5 x Mill-Max 0906-2-15-20-75-14-11-0 spring pins, O-ring 27 x 1.5 silicone, 4 x M3 x 10 button head,
        8 x M3 heat-set inserts, two round 2-layer PCBs (pin board, pad board).
"""
import os
import numpy as np
import manifold3d as mf
import trimesh

P = dict(
    R_ring=16.0,            # socket ring outer radius (32 mm ring)
    r_bore=9.7,             # socket bore
    r_barrel=9.5,           # plug barrel
    r_lug=12.0,             # lug outer radius
    r_groove=12.2,          # undercut outer radius
    twist=60.0,             # lock angle
    lug_w=30.0, key_w=40.0, # lug angular widths (lug 0 is the wide key lug)
    slot_clear=2.0,         # slot is this much wider than the lug on each side (deg)
    z_seat0=1.5,            # 45 deg conical seat: z = z_seat0 + (r - r_bore)
    z_lug_top=6.3,
    z_groove_top=6.7,
    ramp_rise=0.5, ramp_deg=25.0,   # lip surface starts 0.5 low at the slot edge and rises over 25 deg
    z_pad=5.0, pad_t=1.6, r_pad=7.0, r_well=7.15,   # pad board (rings) in the plug
    r_wire=4.5,             # wire bore through the hand base
    pogo_h0=5.004, pogo_mid=0.711, pogo_max=1.397, pogo_body=0.75, pogo_plunger=0.534, pogo_flange=0.915,
    pin_r=(0.0, 2.3, 3.6, 4.9, 6.2), pin_az=(0.0, 0.0, 144.0, 288.0, 72.0), ring_w=0.8, dot_r=1.2,
    wire_r=8.3, wire_az={"DATA": 216.0, "ID": 0.0, "AUX": 144.0, "GND": 288.0, "12V": 72.0},   # wire holes in the pin board
    pin_names=("DATA", "ID", "AUX", "GND", "12V"),
    pinboard_t=1.6, r_pinboard=10.5, r_cbore=10.6,
    z_top=10.9,             # socket top face (bolts against the tip-bay floor)
    plate=(46.0, 32.0), z_plate=6.4, plate_corner=5.0, screw_xy=(19.0, 9.0), screw_d=3.4,
    gap=0.2,                # flange top to mouth face when seated
    flange_bot=-9.7,        # underside of the hand base
    oring_id=27.0, oring_cs=1.5, og_r0=13.2, og_r1=15.3, og_depth=1.05,
    insert_r=11.5, insert_d=4.0, insert_depth=5.0,
    # latch (part of the hand base) and keeper (part of the socket)
    az_tooth=(108.5, 131.5), tooth_r=(16.4, 18.3), tooth_top=2.8,
    beam_r=(16.6, 17.8), az_root=(48.0, 58.0), root_r0=15.5,
    tab_r1=21.0, tab_deg=6.0,
    keeper_h=1.5, keeper_top=3.0, keeper_ramp=(16.0, 4.0), keeper_gap_deg=1.2,
    rho=1.27,               # PETG g/cm3
)
P["z_pinboard"] = P["z_pad"] + P["pogo_h0"] - P["pogo_mid"]          # underside of the pin board (pins at mid-stroke)
P["lug_az"] = (0.0, 120.0, 240.0)                                       # lug centres at home
P["az_keeper"] = P["az_tooth"][0] - P["keeper_gap_deg"]                # keeper's square face
N = 180


def seat_z(r):
    return P["z_seat0"] + (r - P["r_bore"])


def cyl(r, z0, z1, n=N):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([0, 0, z0])


def rev(profile, a0, a1):
    """Solid of revolution of a (r, z) polygon (counter-clockwise) from angle a0 to a1."""
    cs = mf.CrossSection([[(float(r), float(z)) for r, z in profile]])
    return mf.Manifold.revolve(cs, N, float(a1 - a0)).rotate([0, 0, float(a0)])


def sector(r0, r1, a0, a1, z0, z1):
    return rev([(r0, z0), (r1, z0), (r1, z1), (r0, z1)], a0, a1)


def lug_w(i):
    return P["key_w"] if i == 0 else P["lug_w"]


# ---------------------------------------------------------------- arm side
def keeper():
    """Ramped bump on the socket's outer wall: the latch tooth rides up the ramp and drops behind the square face."""
    aK = P["az_keeper"]; R = P["R_ring"]; h = P["keeper_h"]; L, flat = P["keeper_ramp"]
    out = []
    for hh, zt in ((h, P["keeper_top"]), (0.0, P["keeper_top"] + h)):       # 45 deg chamfer on the arm side (prints unsupported)
        pts = []
        for a in np.linspace(aK - L, aK, 33):
            pts.append(((R - 0.5) * np.cos(np.radians(a)), (R - 0.5) * np.sin(np.radians(a))))
        for a in np.linspace(aK, aK - L, 33):
            r = R + (hh if hh else 0.0) * np.clip((a - (aK - L)) / (L - flat), 0, 1)
            pts.append((r * np.cos(np.radians(a)), r * np.sin(np.radians(a))))
        out.append((pts[::-1], zt))                                       # counter-clockwise
    (p0, z0), (p1, z1) = out
    lower = mf.Manifold.extrude(mf.CrossSection([p0]), z0)
    top = mf.Manifold.extrude(mf.CrossSection([p0]), 0.01).translate([0, 0, z0 - 0.01])
    wall = mf.Manifold.extrude(mf.CrossSection([p1]), 0.01).translate([0, 0, z1 - 0.01])
    return lower + mf.Manifold.hull(top + wall)


def socket_cuts():
    """Everything the standard socket removes from its ring (bore, pin-board pocket, slots, undercuts, ramps, roofs)."""
    zt = P["z_top"]
    cut = cyl(P["r_bore"], -1, zt + 1) + cyl(P["r_cbore"], P["z_pinboard"], zt + 1)
    r0, r1 = P["r_bore"] - 0.6, P["r_groove"]
    for i, az in enumerate(P["lug_az"]):
        h = lug_w(i) / 2; s = az - P["twist"]; e = h + P["slot_clear"]
        cut += sector(r0, r1, s - e, s + e, -1, P["z_groove_top"])
        prof = [(r0, seat_z(r0)), (r1, seat_z(r1)), (r1, P["z_groove_top"]), (r0, P["z_groove_top"])]
        cut += rev(prof, s - e, az + h)
        nst = 10
        for k in range(nst):
            d = P["ramp_rise"] * (1 - (k + 0.5) / nst)
            a0 = s + e + P["ramp_deg"] * k / nst; a1 = s + e + P["ramp_deg"] * (k + 1) / nst
            cut += rev([(r0, seat_z(r0) - d), (r1, seat_z(r1) - d), (r1, P["z_groove_top"]), (r0, P["z_groove_top"])], a0 - 0.01, a1 + 0.01)
        roof = mf.Manifold.cylinder(r1 - P["r_bore"], r1, P["r_bore"], N).translate([0, 0, P["z_groove_top"]])
        cut += roof ^ sector(0.0, r1 + 1, s - e, az + h, P["z_groove_top"] - 0.01, P["z_groove_top"] + r1)
    return cut


def arm_socket():
    R = P["R_ring"]; zt = P["z_top"]
    L, W = P["plate"]; c = P["plate_corner"]
    rr = mf.CrossSection([[(-L / 2 + c, -W / 2 + c), (L / 2 - c, -W / 2 + c), (L / 2 - c, W / 2 - c), (-L / 2 + c, W / 2 - c)]]).offset(
        c, mf.JoinType.Round, 2.0, 64)
    body = cyl(R, 0, zt) + mf.Manifold.extrude(rr, zt - P["z_plate"]).translate([0, 0, P["z_plate"]]) + keeper()
    cut = socket_cuts()
    sx, sy = P["screw_xy"]
    for x in (-sx, sx):
        for y in (-sy, sy):
            cut += cyl(P["screw_d"] / 2, P["z_plate"] - 1, zt + 1, 32).translate([x, y, 0])
    return body - cut


def pin_board():
    m = cyl(P["r_pinboard"], P["z_pinboard"], P["z_pinboard"] + P["pinboard_t"])
    for a in P["wire_az"].values():
        m -= cyl(0.5, 0, 20, 16).translate([P["wire_r"] * np.cos(np.radians(a)), P["wire_r"] * np.sin(np.radians(a)), 0])
    return m


def pogo_pins(compress=None):
    cz = P["pogo_mid"] if compress is None else compress
    zb = P["z_pinboard"]; out = []
    for r, a in zip(P["pin_r"], P["pin_az"]):
        x, y = r * np.cos(np.radians(a)), r * np.sin(np.radians(a))
        tip = zb - P["pogo_h0"] + cz
        m = cyl(P["pogo_flange"], zb - 0.3, zb, 20) + cyl(P["pogo_body"], zb - 3.3, zb, 20) + cyl(P["pogo_plunger"], tip, zb - 3.2, 16)
        out.append(m.translate([x, y, 0]))
    return out


# ---------------------------------------------------------------- hand side
def hand_body():
    fb = P["flange_bot"]; ft = -P["gap"]
    body = cyl(P["R_ring"], fb, ft) + cyl(P["r_barrel"], ft - 0.5, P["z_lug_top"])
    r0, r1 = P["r_barrel"] - 0.5, P["r_lug"]
    for i, az in enumerate(P["lug_az"]):
        h = lug_w(i) / 2
        body += rev([(r0, seat_z(r0)), (r1, seat_z(r1)), (r1, P["z_lug_top"]), (r0, P["z_lug_top"])], az - h, az + h)
    cut = cyl(P["r_wire"], fb - 1, P["z_pad"]) + cyl(P["r_well"], P["z_pad"] - P["pad_t"], P["z_lug_top"] + 1)
    cut += cyl(P["og_r1"], ft - P["og_depth"], ft + 1) - cyl(P["og_r0"], ft - P["og_depth"] - 1, ft + 2)
    for k in range(4):
        a = np.radians(45 + 90 * k)
        cut += cyl(P["insert_d"] / 2, fb - 1, fb + P["insert_depth"], 32).translate([P["insert_r"] * np.cos(a), P["insert_r"] * np.sin(a), 0])
    return body - cut


def latch():
    fb = P["flange_bot"]; ft = -P["gap"]
    b0, b1 = P["beam_r"]; t0, t1 = P["tooth_r"]; a0, a1 = P["az_tooth"]; r0a, r0b = P["az_root"]
    m = sector(P["root_r0"], b1, r0a, r0b, fb, ft)
    m += sector(b0, b1, r0b - 0.5, a0 + 0.5, fb, ft)
    m += sector(t0, t1, a0, a1, fb, P["tooth_top"])
    m += sector(b1 - 0.5, P["tab_r1"], a1 - P["tab_deg"], a1, fb, ft)
    m -= rev([(t0 - 0.1, P["tooth_top"] - 0.7), (t0 + 0.7, P["tooth_top"] + 0.1), (t0 - 0.1, P["tooth_top"] + 0.1)], a0 - 1, a1 + 1)
    ca = np.radians(a1)
    wedge = mf.Manifold.cube([1.6, 1.6, 6.0], True).rotate([0, 0, a1 + 45]).translate([t0 * np.cos(ca), t0 * np.sin(ca), 1.5])
    return m - wedge


def hand_base():
    return hand_body() + latch()


def pad_board():
    return cyl(P["r_pad"], P["z_pad"] - P["pad_t"], P["z_pad"])


# ---------------------------------------------------------------- motion
def squeeze():
    return P["oring_cs"] - P["og_depth"] - P["gap"]


def ramp_dz(theta):
    prog = theta + P["twist"]
    lead = prog - P["slot_clear"]
    d_ramp = P["ramp_rise"] * (1 - np.clip(lead / P["ramp_deg"], 0, 1))
    return float(min(squeeze(), d_ramp))


def place(m, theta=0.0, dz=0.0):
    return m.rotate([0, 0, float(theta)]).translate([0, 0, -float(dz)])


def flex_latch(m, defl):
    if defl == 0:
        return m
    ar = np.radians(P["az_root"][1]); rb = sum(P["beam_r"]) / 2
    piv = np.array([rb * np.cos(ar), rb * np.sin(ar), 0.0])
    at = np.radians(sum(P["az_tooth"]) / 2); tip = np.array([P["tooth_r"][0] * np.cos(at), P["tooth_r"][0] * np.sin(at), 0.0])
    arm = np.linalg.norm(tip - piv)
    return m.translate((-piv).tolist()).rotate([0, 0, -np.degrees(defl / arm)]).translate(piv.tolist())


def to_trimesh(m):
    me = m.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)
