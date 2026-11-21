"""hand3 hub v3, multi-digit control, decision D (2026-11-20): every finger curls on its own, all four at once.

Each root carries its own winch: a hanging leg under the grip (AlSi10Mg with the root) holds a vertical M2 lead screw; an M2 nut
in a slider runs on it, and the finger's 2:1 tackle (anchor strand + running strand, as v3) turns round the slider's eye.  The
screw is self-locking (lead 0.4 mm), so the finger holds whatever curl it was left at, in flight too.
Under the hub, 8 static drivers (Pololu 30:1 micro metal gearmotor HPCB 6 V) stand at 0/60/90/120/180/240/270/300 deg, the
places the four poses park roots on (cross, 3+1 turned -30, 2+2 turned -30, one side).  One SG92R lifts all 8 by 3.5 mm so their
blade keys enter the slots in the screws' lower ends; each driver then winds its own finger.
Removed (vs v3): base pull ring + rail, beads, 4 yoke halves, 8 rods, tie ring, 7 idlers, 2 STS3215 + spools.
Kept: latch pins, cranks, station (roller + blade + SG92R + QTR-1A) for moving roots - the station moves to az 150 (between the
drivers at 120 and 180), its blade body gets a notch for the legs.  Root frame: x radial at az 0, z up.  Units mm."""
import numpy as np
import manifold3d as mf
import hand3_design as H
import hand3_hub_v3 as V3
import hand3_hubc as HC
from hand3_finger import cyl, box

M_ = mf.Manifold
V = V3.V
STATION_AZ = 150.0
DRIVER_AZ = (0, 60, 90, 120, 180, 240, 270, 300)
SCREW = dict(d=2.0, lead=0.4, x=28.2, y=-1.2, z=(67.5, 92.0), slot=(0.6, 1.5))        # M2 x 24.5 (cut from M2 threaded rod), slot 0.6 x 1.5 deep
LEG = dict(x_in=23.0, x_rim=25.8, x_out=29.6, y=3.3, z0=66.0, z_rim=90.5, cheek=0.85)
SLIDER = dict(x=(24.4, 29.6), y=2.4, h=5.0, z_top0=86.0, eye_x=26.3)          # carries an M2 nut (4 AF, 1.6 thick)
STROKE = V3.CORD_STROKE / 2                                                     # 2:1 tackle: 11.1 mm of slider for 22.2 mm of cord
N20 = dict(name="Pololu 30:1 Micro Metal Gearmotor HPCB 6V", gx=10.0, gy=12.0, L=24.7, shaft=9.0, rpm=1000, stall_kgcm=0.6, m=9.5)
KEY = dict(t=0.5, w=3.0, top_retracted=65.0, lift=3.5)
LIFT = dict(r=(21.5, 35.5), z=(24.0, 26.0))


def leg():
    """Hanging leg (part of the root, AlSi10Mg): two cheeks (the slider runs between them), a top block into the grip and a bottom
    bridge that holds the screw's lower end.  Inside x 29.9 where the station's push fingers pass; |y| < 3.5 between the cranks."""
    L = LEG; y = L["y"]; c = L["cheek"]
    top = box([L["x_rim"], -y, L["z_rim"]], [L["x_out"], y, H.Z["blade"] - 2.0]) + box([L["x_in"], -y, SLIDER["z_top0"] + 0.2], [L["x_out"], y, L["z_rim"] + 0.01])
    cheeks = box([L["x_in"], -y, L["z0"]], [L["x_out"], -y + c, SLIDER["z_top0"] + 0.3]) + box([L["x_in"], y - c, L["z0"]], [L["x_out"], y, SLIDER["z_top0"] + 0.3])
    bottom = box([L["x_in"], -y, L["z0"]], [L["x_out"], y, SCREW["z"][0] + 1.5])
    m = top + cheeks + bottom
    m -= cyl(SCREW["d"] / 2 + 0.15, SCREW["z"][0] - 1, SCREW["z"][1], c=(SCREW["x"], SCREW["y"], 0), n=20)          # screw bore (top end blind at z 92)
    m -= cyl(1.6, L["z0"] - 1, SCREW["z"][0] + 0.01, c=(SCREW["x"], SCREW["y"], 0), n=20)                            # key way from below
    for s in (-1, 1):                                                                                     # tackle strands up into the grip
        m -= cyl(0.6, SLIDER["z_top0"] - 1, L["z_rim"] + 3.5, c=(SLIDER["eye_x"], s * 0.7, 0), n=12)
    return m


_RT = {}


def root_d(az=0.0):
    """v3 root block (latch pins, cranks, clean grip with the flexor channel and K1 band hook) + the winch leg."""
    if "m" not in _RT:
        import hand3_bands as B
        from hand3_finger import app, T, rot
        m = V3.root_block(0) + leg() - cyl(SCREW["d"] / 2 + 0.15, SCREW["z"][0] - 1, SCREW["z"][1], c=(SCREW["x"], SCREW["y"], 0), n=20)
        xp = B.hooks(0)[0]                                                   # keep K1's return band free round its proximal dowel
        m -= app(B.channel(0, xp - 1.6, H.KN[0]) - box([-50, -5, -50], [xp - 2.0, 5, 50]), T(rot("x", H.PITCH))).translate([0, 0, H.Z["blade"]])
        _RT["m"] = m
    return _RT["m"].rotate([0, 0, float(az)])


def screw():
    m = cyl(SCREW["d"] / 2, *SCREW["z"], c=(SCREW["x"], SCREW["y"], 0), n=16)
    y = SCREW["y"]
    return m - box([SCREW["x"] - 1.2, y - SCREW["slot"][0] / 2, SCREW["z"][0] - 0.1], [SCREW["x"] + 1.2, y + SCREW["slot"][0] / 2, SCREW["z"][0] + SCREW["slot"][1]])


def slider(pull=0.0):
    """Nut slider at a given slider travel (0..STROKE, down = cord in = curl)."""
    zt = SLIDER["z_top0"] - pull; S = SLIDER
    m = box([S["x"][0], -S["y"], zt - S["h"]], [S["x"][1], S["y"], zt])
    m -= cyl(SCREW["d"] / 2 + 0.05, zt - S["h"] - 1, zt + 1, c=(SCREW["x"], SCREW["y"], 0), n=16)
    m -= cyl(0.45, -3, 3, "y", c=(S["eye_x"], 0, zt - 1.0), n=12)                    # eye for the tackle
    return m


def driver(lifted=False):
    """One static driver at az 0 (caller turns it): N20 standing, shaft up, coupler with a blade key on top."""
    dz = KEY["lift"] if lifted else 0.0; x = SCREW["x"]; y0 = SCREW["y"]; kt = KEY["top_retracted"] + dz
    coup = cyl(2.0, kt - 5.0, kt - KEY["w"], c=(x, 0, 0), n=20)
    key = box([x - 0.9, y0 - KEY["t"] / 2, kt - KEY["w"] - 0.01], [x + 0.9, y0 + KEY["t"] / 2, kt])
    shaft = cyl(1.5, kt - 5.0 - N20["shaft"], kt - 5.0, c=(x, 0, 0), n=16)
    zb = kt - 5.0 - N20["shaft"]
    body = box([x - N20["gx"] / 2, y0 - N20["gy"] / 2, zb - N20["L"]], [x + N20["gx"] / 2, y0 + N20["gy"] / 2, zb])
    return body + shaft + coup + key


def drivers(lifted=False):
    return M_.batch_boolean([driver(lifted).rotate([0, 0, float(a)]) for a in DRIVER_AZ], mf.OpType.Add)


def lift_ring(lifted=False):
    dz = KEY["lift"] if lifted else 0.0
    return (cyl(LIFT["r"][1], LIFT["z"][0] + dz, LIFT["z"][1] + dz, n=96) - cyl(LIFT["r"][0], LIFT["z"][0] - 1, LIFT["z"][1] + 5, n=96))


def frame_d():
    """hub E frame with the 8 driver pockets and the lift-ring groove cut (servo bays empty now)."""
    f = HC._parts()["frame"][0]
    cut = None
    for a in DRIVER_AZ:
        p = box([SCREW["x"] - N20["gx"] / 2 - 0.3, SCREW["y"] - N20["gy"] / 2 - 0.3, LIFT["z"][0] - 0.3], [SCREW["x"] + N20["gx"] / 2 + 0.3, SCREW["y"] + N20["gy"] / 2 + 0.3, 60.0]).rotate([0, 0, float(a)])
        cut = p if cut is None else cut + p
    cut += cyl(LIFT["r"][1] + 0.3, LIFT["z"][0] - 0.3, LIFT["z"][1] + KEY["lift"] + 0.3, n=96) - cyl(LIFT["r"][0] - 0.3, 0, 60, n=96)
    sp = V3.station_parts()                                                      # the station's roller motor stands in the frame at az 150
    mb = np.asarray(sp["motor"].bounding_box())
    cut += box([mb[0] - V3.ST["roll_gap"] - 0.5, mb[1] - 0.5, mb[2] - 0.5], [mb[3] + 0.5, mb[4] + 0.5, 60.0]).rotate([0, 0, STATION_AZ])
    return f - cut


def station_d():
    """v3 station turned to az 150, the blade body notched (|y| < 3.8 up to r 36.2) so it can pass a held root's leg."""
    sp = V3.station_parts()
    b0, b1 = V3.ST["body_z"]
    sp["blade"] = sp["blade"] - box([V3.ST["body_r0"] - 1, -3.8, b0 - 1], [V3.ST["body_r0"] + V3.ST["stroke"] + 0.5, 3.8, b1 + 1])
    return sp


def tackle(az=0.0, pull=0.0):
    """Two strands from the slider eye up into the grip (world, at az)."""
    a = np.radians(az); u = np.array([np.cos(a), np.sin(a), 0.0]); n = np.array([-np.sin(a), np.cos(a), 0.0])
    z0 = SLIDER["z_top0"] - pull - 1.0; z1 = V3.Z_GAP[0] - 1.0; r = SLIDER["eye_x"]
    return [np.array([u * r + n * s * 0.7 + [0, 0, z0], u * r + n * s * 0.7 + [0, 0, z1]]) for s in (-1, 1)]


def numbers():
    W = 1309.0; F = 55.0; Fn = 2 * F                                                 # nut load with the 2:1 tackle
    dm = SCREW["d"] - 0.65 * 0.4; lam = np.arctan(SCREW["lead"] / (np.pi * dm)); phi = np.arctan(0.2)
    T = Fn * dm / 2e3 * np.tan(lam + phi); T_st = N20["stall_kgcm"] * 0.0981
    revs = STROKE / SCREW["lead"]; t = revs / (0.75 * N20["rpm"] / 60)
    mm = leg().volume() * 2.68e-3 + 0.6 + 0.35                                       # leg AlSi10Mg + screw + slider/nut (est.)
    F_c = mm * 1e-3 * 27.0e-3 * W ** 2
    lever = (H.Z["blade"] - 2.0) - 78.0
    S = 2 * LEG["y"] * (LEG["x_out"] - LEG["x_rim"]) ** 2 / 6
    sig = F_c * lever / S
    return dict(nut_load_N=Fn, screw_torque_Nm=round(T, 4), N20_stall_Nm=round(T_st, 3), torque_margin=round(T_st / T, 1),
                self_locking=bool(lam < phi), lead_angle_deg=round(np.degrees(lam), 1), curl_time_s=round(t, 1),
                leg_set_g=round(mm, 2), leg_pull_N=round(F_c), leg_root_MPa=round(sig, 1), leg_SF_AlSi10Mg_230=round(230 / sig, 2),
                root_AlSi10Mg_g=round(root_d(0).volume() * 2.68e-3, 2))
