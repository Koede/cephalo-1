"""Hippos-1 tail mount on the SFF connector (single source of truth).  Draft CAD, 2026-11-16.

Same connector frame as sff_core (= handmount.py): z = connector axis, +z goes INTO the rump, z = 0 is the socket mouth;
the tail hangs toward -z.  The tail locks by turning +60 deg about +z.  Everything SFF (lugs, lips, ramp, keeper, latch,
pin board, O-ring, pad board) is the unchanged standard; only these two parts are new:

  rump_socket   standard SFF socket ring + a round mount flange (replaces the 46 x 32 plate) + a 52.6 mm sleeve
                that reaches 30 mm out of the rump; window over the latch tab.  Bolts to the rump bulkhead (4 x M3).
  tail_adapter  bolts under the STANDARD hand_base (its 4 M3 inserts): conical floor + 46.4 mm skirt that slides in the
                sleeve (0.10 mm radial), window over the latch tab, bottom flange with 4 M3 inserts for the tail's root
                servo bracket.  The tail = hand_base + tail_adapter + the 4-joint tail.
A hand without a skirt still plugs straight into rump_socket; the tail cannot plug into an arm socket's flat plate pad
only because the arm has no sleeve (the lock itself fits).
"""
import os, sys
import numpy as np
import manifold3d as mf
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sff_core as H
from sff_core import cyl, rev, sector, P

T = dict(
    c_rad=0.10,                     # skirt-in-sleeve radial clearance
    sk_r0=20.7, sk_t=2.5,           # skirt inner radius (clears the latch tooth pushed out 1.5 mm: r 19.8), wall
    sk_top=5.9,                     # skirt top (0.5 under the mount flange)
    sk_bot=-30.0,                   # skirt bottom = top of the servo flange
    sl_t=3.0, sl_bot=-29.5,         # sleeve wall, sleeve end (0.5 above the servo flange when seated)
    fl_r=32.0, fl_screw_r=29.5,     # rump mount flange (z 6.4..10.9 like the old plate), 4 x M3 clearance
    lead=1.0,                       # 45 deg lead-in chamfers, skirt top outer edge and sleeve mouth inner edge
    win_az=(116.0, 141.0), win_z=(-10.7, 0.8),   # latch-tab window (tab at az 125.5..131.5, z -9.7..-0.2, base at home)
    cone_t=3.0,                     # conical floor thickness (normal), 45 deg so it prints without supports
    boss_r=3.6, cb_d=6.2, cb_top=-15.7,           # bosses for the 4 M3 x 10 into the hand_base inserts, counterbore from below
    sf_r=(20.7, 28.0), sf_z=(-35.0, -30.0),        # servo flange (bottom), 4 x M3 heat-set inserts
    sf_ins_r=25.0, sf_ins_az=(20.0, -20.0, 160.0, 200.0), ins_d=4.0, ins_depth=5.0,
)
T["sk_r1"] = T["sk_r0"] + T["sk_t"]
T["sl_r0"] = T["sk_r1"] + T["c_rad"]
T["sl_r1"] = T["sl_r0"] + T["sl_t"]
N = H.N


def window(r_in, r_out, az0, az1, z0, z1, point):
    """Radial prism window through a cylinder wall.  point = 'low' puts a 45 deg V on the low-z edge, 'high' on the high-z
    edge (so the window prints without a flat bridge)."""
    rm = 0.5 * (r_in + r_out); w = rm * np.radians(az1 - az0); h = w / 2
    if point == "low":
        poly = [(-w / 2, z0 + h), (0, z0), (w / 2, z0 + h), (w / 2, z1), (-w / 2, z1)]
    else:
        poly = [(-w / 2, z0), (w / 2, z0), (w / 2, z1 - h), (0, z1), (-w / 2, z1 - h)]
    # extrude in y (radial), polygon in (x = tangential, z): build in XY then rotate so local y -> z
    cs = mf.CrossSection([[(float(a), float(b)) for a, b in poly]])
    pr = mf.Manifold.extrude(cs, r_out - r_in + 4).translate([0, 0, -(r_out - r_in + 4) / 2])   # local (x, y=z_model, z=radial)
    pr = pr.rotate([90, 0, 0])                                     # local y -> model z, local z -> model -y
    pr = pr.rotate([0, 0, 90]).translate([rm, 0, 0])                # radial along +x at az 0
    return pr.rotate([0, 0, 0.5 * (az0 + az1)])


def rump_socket():
    R = P["R_ring"]; zt = P["z_top"]; zp = P["z_plate"]
    body = cyl(R, 0, zt) + H.keeper()
    body += cyl(T["fl_r"], zp, zt)                                                      # round mount flange
    body += cyl(T["sl_r1"], T["sl_bot"], zp + 0.5) - cyl(T["sl_r0"], T["sl_bot"] - 1, zp)   # sleeve
    cut = H.socket_cuts()
    # 45 deg lead-in on the sleeve mouth
    cut += rev([(T["sl_r0"] - 0.01, T["sl_bot"] - 0.01), (T["sl_r0"] + T["lead"], T["sl_bot"] - 0.01), (T["sl_r0"] - 0.01, T["sl_bot"] + T["lead"])], 0, 360)
    for k in range(4):
        a = np.radians(45 + 90 * k)
        cut += cyl(P["screw_d"] / 2, zp - 1, zt + 1, 32).translate([T["fl_screw_r"] * np.cos(a), T["fl_screw_r"] * np.sin(a), 0])
    # tab window; printed top-face-down (model -z is up in print), so the low-z edge faces down: V there
    cut += window(T["sl_r0"], T["sl_r1"], *T["win_az"], *T["win_z"], point="low")
    return body - cut


def tail_adapter():
    fb = P["flange_bot"]
    r0, r1 = T["sk_r0"], T["sk_r1"]
    skirt = cyl(r1, T["sk_bot"], T["sk_top"]) - cyl(r0, T["sk_bot"] - 1, T["sk_top"] + 1)
    # 45 deg lead-in on the skirt top outer edge
    skirt -= rev([(r1 - T["lead"], T["sk_top"] + 0.01), (r1 + 0.01, T["sk_top"] - T["lead"]), (r1 + 0.01, T["sk_top"] + 0.01)], 0, 360)
    # conical floor: underside z_u(r) = (fb - 4) - (r - r_wire); thickness cone_t (normal) -> vertical cone_t * sqrt2
    zi = fb - 4.0; rw = P["r_wire"]; tv = T["cone_t"] * np.sqrt(2)
    r_end = r0 + 0.5
    floor = rev([(rw, zi), (r_end, zi - (r_end - rw)), (r_end, zi - (r_end - rw) + tv), (rw, zi + tv)], 0, 360)
    floor = floor - cyl(rw, zi - 30, fb + 1)
    # flat seat for the base underside (r <= 15.4), stepped 0.5 lower under the latch so the beam slides free
    seat = cyl(15.4, zi - 15.0, fb) - cyl(rw, zi - 16, fb + 1)       # filled down to the cone: no flat overhang under the seat
    # keep the seat only above the cone underside: intersect with the region above z_u(r)
    above = rev([(rw, zi), (40, zi - (40 - rw)), (40, fb + 1), (rw, fb + 1)], 0, 360)
    seat = seat ^ above
    bosses = mf.Manifold()
    holes = mf.Manifold()
    for k in range(4):
        a = np.radians(45 + 90 * k); x, y = P["insert_r"] * np.cos(a), P["insert_r"] * np.sin(a)
        zb = zi - (P["insert_r"] + T["boss_r"] - rw)
        bosses += cyl(T["boss_r"], zb, fb, 48).translate([x, y, 0])
        holes += cyl(1.7, zb - 1, fb + 1, 32).translate([x, y, 0])
        holes += cyl(T["cb_d"] / 2, zb - 30, T["cb_top"], 48).translate([x, y, 0])
    sf = cyl(T["sf_r"][1], *T["sf_z"]) - cyl(T["sf_r"][0], T["sf_z"][0] - 1, T["sf_z"][1] + 1)
    for adeg in T["sf_ins_az"]:                 # pattern matches tail.root_cradle (clears its servo walls)
        a = np.radians(adeg)
        holes += cyl(T["ins_d"] / 2, T["sf_z"][0] - 1, T["sf_z"][0] + T["ins_depth"], 32).translate([T["sf_ins_r"] * np.cos(a), T["sf_ins_r"] * np.sin(a), 0])
    bosses = bosses ^ above                    # boss bottoms follow the 45 deg cone underside
    body = skirt + floor + seat + bosses + sf
    body = body - holes
    # printed bottom flange down (+z up in print): the window's high-z edge faces down -> V there
    # tab window in the skirt: an open notch to the skirt top (a pointed top would catch the tab when it is pulled out)
    w_half = 0.5 * (0.5 * (r0 + r1)) * np.radians(T["win_az"][1] - T["win_az"][0])
    body = body - window(r0, r1, *T["win_az"], T["win_z"][0], T["sk_top"] + w_half + 1.0, point="high")
    return body


def tail_parts():
    return {"rump_socket": rump_socket(), "tail_adapter": tail_adapter(), "hand_base": H.hand_base()}


def export_stls(d="tailmount_stl"):
    os.makedirs(d, exist_ok=True)
    S = rump_socket().rotate([180, 0, 0]).translate([0, 0, P["z_top"]])           # print: flange top face on the bed, sleeve up
    A = tail_adapter().translate([0, 0, -T["sf_z"][0]])                           # print: servo flange on the bed, skirt up
    for n, m in (("rump_socket", S), ("tail_adapter", A)):
        H.to_trimesh(m).export(f"{d}/{n}.stl")
    return S, A


if __name__ == "__main__":
    for n, m in tail_parts().items():
        t = H.to_trimesh(m); bb = m.bounding_box()
        print(f"{n:12s} {m.status()} bodies {len(m.decompose())} watertight {t.is_watertight} mass {m.volume()/1000*P['rho']:.1f} g bbox {np.round(bb, 1)}")
    export_stls()
