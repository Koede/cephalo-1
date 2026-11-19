"""Modular hand connector for Cephalo-1 (single source of truth): arm-side socket + the shared hand base.
(workspace copy of claude/code/handmount.py: geometry only; STL export and checks are in the project copy)

Connector frame (mm): z = connector axis, +z goes up into the arm (the hand hangs toward -z); z = 0 is the socket
mouth face.  The hand locks by turning +TWIST degrees about +z.
"""
import numpy as np
import manifold3d as mf

P = dict(
    R_ring=16.0, r_bore=9.7, r_barrel=9.5, r_lug=12.0, r_groove=12.2, twist=60.0,
    lug_w=30.0, key_w=40.0, slot_clear=2.0, z_seat0=1.5, z_lug_top=6.3, z_groove_top=6.7,
    ramp_rise=0.5, ramp_deg=25.0, z_pad=5.0, pad_t=1.6, r_pad=7.0, r_well=7.15, r_wire=4.5,
    pogo_h0=5.004, pogo_mid=0.711, pinboard_t=1.6, r_pinboard=10.5, r_cbore=10.6,
    z_top=10.9, plate=(46.0, 32.0), z_plate=6.4, plate_corner=5.0, screw_xy=(19.0, 9.0), screw_d=3.4,
    gap=0.2, flange_bot=-9.7, oring_cs=1.5, og_r0=13.2, og_r1=15.3, og_depth=1.05,
    insert_r=11.5, insert_d=4.0, insert_depth=5.0,
    az_tooth=(108.5, 131.5), tooth_r=(16.4, 18.3), tooth_top=2.8, beam_r=(16.6, 17.8), az_root=(48.0, 58.0), root_r0=15.5,
    tab_r1=21.0, tab_deg=6.0, keeper_h=1.5, keeper_top=3.0, keeper_ramp=(16.0, 4.0), keeper_gap_deg=1.2, rho=1.27,
)
P["z_pinboard"] = P["z_pad"] + P["pogo_h0"] - P["pogo_mid"]
P["lug_az"] = (0.0, 120.0, 240.0)
P["az_keeper"] = P["az_tooth"][0] - P["keeper_gap_deg"]
N = 90


def seat_z(r):
    return P["z_seat0"] + (r - P["r_bore"])


def cyl(r, z0, z1, n=N):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([0, 0, z0])


def rev(profile, a0, a1):
    cs = mf.CrossSection([[(float(r), float(z)) for r, z in profile]])
    return mf.Manifold.revolve(cs, N, float(a1 - a0)).rotate([0, 0, float(a0)])


def sector(r0, r1, a0, a1, z0, z1):
    return rev([(r0, z0), (r1, z0), (r1, z1), (r0, z1)], a0, a1)


def lug_w(i):
    return P["key_w"] if i == 0 else P["lug_w"]


def arm_socket():
    R = P["R_ring"]; zt = P["z_top"]
    L, W = P["plate"]; c = P["plate_corner"]
    rr = mf.CrossSection([[(-L / 2 + c, -W / 2 + c), (L / 2 - c, -W / 2 + c), (L / 2 - c, W / 2 - c), (-L / 2 + c, W / 2 - c)]]).offset(
        c, mf.JoinType.Round, 2.0, 64)
    body = cyl(R, 0, zt) + mf.Manifold.extrude(rr, zt - P["z_plate"]).translate([0, 0, P["z_plate"]])
    cut = cyl(P["r_bore"], -1, zt + 1) + cyl(P["r_cbore"], P["z_pinboard"], zt + 1)
    r0, r1 = P["r_bore"] - 0.6, P["r_groove"]
    for i, az in enumerate(P["lug_az"]):
        h = lug_w(i) / 2; s = az - P["twist"]; e = h + P["slot_clear"]
        cut += sector(r0, r1, s - e, s + e, -1, P["z_groove_top"])
        prof = [(r0, seat_z(r0)), (r1, seat_z(r1)), (r1, P["z_groove_top"]), (r0, P["z_groove_top"])]
        cut += rev(prof, s - e, az + h)
    sx, sy = P["screw_xy"]
    for x in (-sx, sx):
        for y in (-sy, sy):
            cut += cyl(P["screw_d"] / 2, P["z_plate"] - 1, zt + 1, 32).translate([x, y, 0])
    return body - cut


def hand_body():
    fb = P["flange_bot"]; ft = -P["gap"]
    body = cyl(P["R_ring"], fb, ft) + cyl(P["r_barrel"], ft - 0.5, P["z_lug_top"])
    r0, r1 = P["r_barrel"] - 0.5, P["r_lug"]
    for i, az in enumerate(P["lug_az"]):
        h = lug_w(i) / 2
        body += rev([(r0, seat_z(r0)), (r1, seat_z(r1)), (r1, P["z_lug_top"]), (r0, P["z_lug_top"])], az - h, az + h)
    cut = cyl(P["r_wire"], fb - 1, P["z_pad"]) + cyl(P["r_well"], P["z_pad"] - P["pad_t"], P["z_lug_top"] + 1)
    cut += cyl(P["og_r1"], ft - P["og_depth"], ft + 1) - cyl(P["og_r0"], ft - P["og_depth"] - 1, ft + 2)
    return body - cut


def latch():
    fb = P["flange_bot"]; ft = -P["gap"]
    b0, b1 = P["beam_r"]; t0, t1 = P["tooth_r"]; a0, a1 = P["az_tooth"]; r0a, r0b = P["az_root"]
    m = sector(P["root_r0"], b1, r0a, r0b, fb, ft)
    m += sector(b0, b1, r0b - 0.5, a0 + 0.5, fb, ft)
    m += sector(t0, t1, a0, a1, fb, P["tooth_top"])
    m += sector(b1 - 0.5, P["tab_r1"], a1 - P["tab_deg"], a1, fb, ft)
    return m


def hand_base():
    return hand_body() + latch()


LEN_HAND_BASE = -P["flange_bot"]          # 9.7 mm: the hand moves this far out from the socket mouth
