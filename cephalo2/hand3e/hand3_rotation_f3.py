"""hand3 hub, finger rotation decision F3 (2026-11-20): every finger root turns round the hub on its own, all four at once.

Built on decision D (per-finger self-locking winch curl, 8 static curl drivers under the hub, one lift ring).  Adds, per root:
  - a Pololu 2359 (700:1 sub-micro plastic planetary, 6 x 21 mm, 1.3 g) clamped beside the winch leg, shaft UP;
  - a brass m0.3 z12 pinion (2 mm bore) meshing a z166 gear skirt moulded under the base disc's rim (pitch r 24.9, z 88.5..91);
  - two phosphor-bronze wipers on a foot plate under the leg, riding a static 5-track ring when parked.
Removed (vs D): latch pins, cranks + crank pins, the station (roller, blade, SG92R, QTR-1A), the base disc's pin slots/pockets.
The winch leg is deepened to x 30.8 (spin load of the motor) and its foot gets a V-funnel round the key way, so a rising driver
key centres the root +-1.6 mm.

Two-stage lift (one SG92R): keys sit 1.5 mm lower than in D (top 63.5).  Lift 0..1.5: the track ring rises with the lift ring
(on springs) until it meets the frame stop, wipers touch, the rotation motors can run; keys stay under the legs.  Lift 1.5..5.5:
the track ring stays, the keys enter the screw slots, the curl drivers can run.  In flight: lift 0, nothing static touches.
Root frame: x radial at az 0, z up.  Units mm.  Estimates marked est."""
import numpy as np
import manifold3d as mf
import hand3_design as H
import hand3_hub_v3 as V3
import hand3_hubc as HC
import hand3_multidigit as MD
import hand3_bands as B
from hand3_finger import cyl, box, app, T, rot

M_ = mf.Manifold
CS = mf.CrossSection
V = V3.V

# --- D constants changed by F3 (keys lower, longer lift, lift ring lower)
MD.KEY.update(top_retracted=63.5, lift=5.5)
MD.LIFT.update(z=(22.5, 24.5))
STAGE1 = 1.5                                                     # lift at which the track ring meets its stop (rotation stage)

MOTOR = dict(name="Pololu 2359 700:1 sub-micro plastic planetary gearmotor 6D x 21L, 6 V", d=6.0, L=21.0, can=9.3, shaft_d=2.0,
             m=1.3, rpm=90, stall_Nm=12 * 0.00706, A_stall=0.40, A_free=0.045, gearbox_limit_Nm=250e-5 * 9.81)
GEAR_M = 0.3
PINION = dict(z=12, w=2.0, bore=2.0, z_band=(88.5, 90.5), m=0.25)
SKIRT = dict(z=166, z_band=(88.5, 91.0), r_in=22.0)
RP = GEAR_M * PINION["z"] / 2                                    # 1.8
RG = GEAR_M * SKIRT["z"] / 2                                     # 24.9
AX_R = RP + RG                                                   # 26.7: motor axis radius
AX_Y = 6.6
AX = (float(np.sqrt(AX_R ** 2 - AX_Y ** 2)), AX_Y)              # (25.87, 6.6) in the root frame
MOT_Z = (PINION["z_band"][0] - 0.7 - MOTOR["L"], PINION["z_band"][0] - 0.7)   # 66.8 .. 87.8 (can at the bottom, gearbox on top; tabs to 66.0)
LEG = dict(MD.LEG, x_out=30.8, z_step=87.5)
Z_ROOT = H.Z["blade"] - 2.0
COLLARS = ((67.0, 69.0), (84.0, 86.0))
FUNNEL = dict(r0=2.6, r1=1.6, h=1.0)
FOOT = dict(x=(LEG["x_out"] - 0.5, 40.6), y=1.6, z=(66.0, 66.8))
TRACK = dict(r=(34.6, 40.6), z=(62.0, 63.5), centres=(35.2, 36.4, 37.6, 38.8, 40.0), w=0.8, cu_t=0.07)   # common = 35.2
WIPER = dict(t=0.15, w=0.8, tip_z=64.6)
POST = dict(r=35.0, d=2.0, az=(30.0, 150.0, 210.0, 330.0), z=(18.0, 62.5))


def gear2d(m, z, bore=None, r_in=None):
    """Spur gear outline (trapezoid teeth, 20 deg flanks approximated), as a CrossSection.  bore / r_in = hole radius."""
    rp = m * z / 2; ra = rp + m; rf = rp - 1.25 * m
    pts = []
    for i in range(z):
        a = 2 * np.pi * i / z; p = 2 * np.pi / z
        for f, r in ((-0.30, rf), (-0.17, ra), (0.17, ra), (0.30, rf)):
            pts.append((r * np.cos(a + f * p), r * np.sin(a + f * p)))
    cs = CS([pts])
    hole = bore if bore is not None else r_in
    if hole:
        cs = cs - CS.circle(hole, 64)
    return cs


def skirt():
    return M_.extrude(gear2d(GEAR_M, SKIRT["z"], r_in=SKIRT["r_in"]), SKIRT["z_band"][1] - SKIRT["z_band"][0] + 0.01).translate([0, 0, SKIRT["z_band"][0]])


def base_disc_f3():
    """v3 base disc without the latch-pin slots and pockets, with the z166 m0.3 gear skirt under its rim (one printed part)."""
    zt = V["z0"] + V["disc_t"]
    m = cyl(V["disc_r"], V["z0"], zt, n=160) + cyl(V["boss_r"], V["boss_z0"], V3.Z_GAP[1], n=64)
    for r0, r1 in V["ribs"]:
        m -= cyl(r1 + V["groove_clr"], zt - V["rib_h"] - V["groove_clr"], zt + 1, n=128) - cyl(r0 - V["groove_clr"], zt - 2, zt + 2, n=128)
    m -= cyl(V["shaft_r"], V["boss_z0"] - 1, V3.Z_GAP[1] + 1, n=32)
    for k in range(V["top_screw_n"]):
        a = np.radians(90 + 360 / V["top_screw_n"] * k)
        m -= cyl(0.8, V3.Z_GAP[1] - 5, V3.Z_GAP[1] + 1, c=(V["top_screw_r"] * np.cos(a), V["top_screw_r"] * np.sin(a), 0), n=16)
    return m + skirt()


def pinion():
    """Brass m0.3 z12 pinion on the motor shaft, root frame (at AX)."""
    return M_.extrude(gear2d(GEAR_M, PINION["z"], bore=PINION["bore"] / 2 - 0.02), PINION["w"]).translate([AX[0], AX[1], PINION["z_band"][0]])


def motor():
    """Pololu 2359 standing shaft-up: can (bottom), gearbox, 2 mm shaft up through the pinion."""
    z0, z1 = MOT_Z
    can = cyl(3.0, z0, z0 + MOTOR["can"], c=(AX[0], AX[1], 0), n=32)
    gb = cyl(3.0, z0 + MOTOR["can"], z1, c=(AX[0], AX[1], 0), n=32)
    sh = cyl(1.0, z1, PINION["z_band"][1], c=(AX[0], AX[1], 0), n=16)
    tabs = box([AX[0] - 1.2, AX[1] - 0.5, z0 - 0.8], [AX[0] + 1.2, AX[1] + 0.5, z0])
    return can + gb + sh + tabs


def root_clean():
    """v3 root block without the latch-pin bores and arm slots (pins gone)."""
    zb = H.Z["blade"]
    grip = app(V3.grip_clean(), T(t=[0, 0, zb]))
    b = V3.wedge(V["block_r"][0], V["block_r"][1], *V3.Z_GAP, -V["block_half"], V["block_half"])
    for r0, r1 in V["ribs"]:
        b += V3.wedge(r0, r1, V3.Z_GAP[0] - V["rib_h"], V3.Z_GAP[0] + 0.01, -V["block_half"], V["block_half"])
        b += V3.wedge(r0, r1, V3.Z_GAP[1] - 0.01, V3.Z_GAP[1] + V["rib_h"], -V["block_half"], V["block_half"])
    m = b + grip
    for s in (-1, 1):
        m -= cyl(0.6, V3.Z_GAP[0] - 3, zb + (1.0 if s > 0 else 6.0), c=(V["bead"]["eye_r"], s * 0.7, 0), n=16)
    m -= cyl(1.1, zb + 1.4, zb + 6.0, c=(V["bead"]["eye_r"], -0.7, 0), n=20)
    return m


def leg_f3():
    """Winch leg (AlSi10Mg with the root), deepened to x 30.8, stepped clear of the gear skirt above z 87.5, motor collars on the +y
    cheek, V-funnel under the key way, foot plate for the wipers."""
    L = LEG; y = L["y"]; c = L["cheek"]; S = MD.SCREW
    top = box([L["x_rim"], -y, L["z_step"]], [L["x_out"], y, Z_ROOT])
    step = box([L["x_in"], -y, MD.SLIDER["z_top0"] + 0.2], [L["x_out"], y, L["z_step"] + 0.01])
    cheeks = box([L["x_in"], -y, L["z0"]], [L["x_out"], -y + c, MD.SLIDER["z_top0"] + 0.3]) + box([L["x_in"], y - c, L["z0"]], [L["x_out"], y, MD.SLIDER["z_top0"] + 0.3])
    bottom = box([L["x_in"], -y, L["z0"]], [L["x_out"], y, S["z"][0] + 1.5])
    col = None
    for z0, z1 in COLLARS:
        k = cyl(3.9, z0, z1, c=(AX[0], AX[1], 0), n=40)
        col = k if col is None else col + k
    foot = box([FOOT["x"][0], -FOOT["y"], FOOT["z"][0]], [FOOT["x"][1], FOOT["y"], FOOT["z"][1]])
    m = top + step + cheeks + bottom + col + foot
    m -= cyl(3.05, MOT_Z[0] - 1, MOT_Z[1] + 1, c=(AX[0], AX[1], 0), n=40)                                       # motor bore in the collars
    m -= cyl(S["d"] / 2 + 0.15, S["z"][0] - 1, S["z"][1], c=(S["x"], S["y"], 0), n=20)
    m -= cyl(1.6, L["z0"] - 1, S["z"][0] + 0.01, c=(S["x"], S["y"], 0), n=20)
    m -= M_.cylinder(FUNNEL["h"] + 0.01, FUNNEL["r0"], FUNNEL["r1"], 32).translate([S["x"], S["y"], L["z0"] - 0.01])    # V-funnel
    for s in (-1, 1):
        m -= cyl(0.6, MD.SLIDER["z_top0"] - 1, L["z_rim"] + 3.5, c=(MD.SLIDER["eye_x"], s * 0.7, 0), n=12)
    for r in (TRACK["centres"][0], 37.0):                                                                     # wiper rivet holes (M1, est.)
        m -= cyl(0.5, FOOT["z"][0] - 1, FOOT["z"][1] + 1, c=(r, 0, 0), n=12)
    return m


_RT = {}


def root_f3(az=0.0):
    if "m" not in _RT:
        S = MD.SCREW
        m = root_clean() + leg_f3() - cyl(S["d"] / 2 + 0.15, S["z"][0] - 1, S["z"][1], c=(S["x"], S["y"], 0), n=20)
        xp = B.hooks(0)[0]
        m -= app(B.channel(0, xp - 1.6, H.KN[0]) - box([-50, -5, -50], [xp - 2.0, 5, 50]), T(rot("x", H.PITCH))).translate([0, 0, H.Z["blade"]])
        _RT["m"] = m
    return _RT["m"].rotate([0, 0, float(az)])


def wipers(k):
    """Two phosphor-bronze leaf wipers under root k's foot: common track (r 35.2) and the root's own track (r of track k+1).
    Leaves run tangentially (+y) from the rivet and bend down to the tip (est. 0.15 mm, 0.4 N preload)."""
    out = None
    for r in (TRACK["centres"][0], TRACK["centres"][k + 1]):
        leaf = box([r - WIPER["w"] / 2, -0.5, FOOT["z"][0] - WIPER["t"]], [r + WIPER["w"] / 2, 2.6, FOOT["z"][0]])
        drop = M_.batch_hull([box([r - WIPER["w"] / 2, 2.45, FOOT["z"][0] - WIPER["t"]], [r + WIPER["w"] / 2, 2.6, FOOT["z"][0]]),
                              box([r - WIPER["w"] / 2, 3.4, WIPER["tip_z"]], [r + WIPER["w"] / 2, 3.55, WIPER["tip_z"] + 0.15])])
        w = leaf + drop
        out = w if out is None else out + w
    return out


def track_ring():
    """Static printed ring (PA-CF) with 5 copper-tape tracks on top; rides the lift ring on 4 posts + springs, stops at lift 1.5."""
    m = cyl(TRACK["r"][1], *TRACK["z"], n=160) - cyl(TRACK["r"][0], TRACK["z"][0] - 1, TRACK["z"][1] + 1, n=160)
    for a in POST["az"]:
        c = (POST["r"] * np.cos(np.radians(a)), POST["r"] * np.sin(np.radians(a)), 0)
        m += cyl(1.6, TRACK["z"][0] - 1.0, TRACK["z"][0] + 0.01, c=c, n=20)
        m -= cyl(POST["d"] / 2, TRACK["z"][0] - 2.0, POST["z"][1], c=c, n=16)            # post pressed in from below
    return m


def tracks():
    out = None
    for r in TRACK["centres"]:
        t = cyl(r + TRACK["w"] / 2, TRACK["z"][1], TRACK["z"][1] + TRACK["cu_t"], n=160) - cyl(r - TRACK["w"] / 2, TRACK["z"][1] - 1, TRACK["z"][1] + 1, n=160)
        out = t if out is None else out + t
    return out


def posts():
    out = None
    for a in POST["az"]:
        p = cyl(POST["d"] / 2, *POST["z"], c=(POST["r"] * np.cos(np.radians(a)), POST["r"] * np.sin(np.radians(a)), 0), n=16)
        out = p if out is None else out + p
    return out


def lift_ring_f3():
    m = MD.lift_ring(False)
    for a in POST["az"]:
        m -= cyl(1.2, 0, 60, c=(POST["r"] * np.cos(np.radians(a)), POST["r"] * np.sin(np.radians(a)), 0), n=16)
    return m


def frame_f3():
    """hub E frame: the 8 driver pockets and the lift-ring groove (as D, lift 5.5, ring 1.5 lower), 4 post holes; no station pocket."""
    f = HC._parts()["frame"][0]; S = MD.SCREW; N20 = MD.N20; LI = MD.LIFT; K = MD.KEY
    cut = None
    for a in MD.DRIVER_AZ:
        p = box([S["x"] - N20["gx"] / 2 - 0.3, S["y"] - N20["gy"] / 2 - 0.3, LI["z"][0] - 0.3], [S["x"] + N20["gx"] / 2 + 0.3, S["y"] + N20["gy"] / 2 + 0.3, 60.0]).rotate([0, 0, float(a)])
        cut = p if cut is None else cut + p
    cut += cyl(LI["r"][1] + 0.3, LI["z"][0] - 0.3, LI["z"][1] + K["lift"] + 0.3, n=96) - cyl(LI["r"][0] - 0.3, 0, 60, n=96)
    for a in POST["az"]:
        cut += cyl(1.3, -1, 61, c=(POST["r"] * np.cos(np.radians(a)), POST["r"] * np.sin(np.radians(a)), 0), n=16)
    return f - cut


def track_dz(lift):
    return min(lift, STAGE1)


def numbers():
    W = 1309.0
    T_use = min(MOTOR["gearbox_limit_Nm"], 0.25 * MOTOR["stall_Nm"])
    push = T_use / (RP * 1e-3)
    deg_s = np.degrees(0.75 * MOTOR["rpm"] / 60 * 2 * np.pi * RP / RG)
    m_set = MOTOR["m"] + PINION["m"]
    F_mot = m_set * 1e-3 * AX_R * 1e-3 * W ** 2
    leg = leg_f3(); vol = leg.volume()
    m_leg = vol * 2.68e-3 + 0.6 + 0.35
    F_leg = m_leg * 1e-3 * 27.5e-3 * W ** 2
    S = 2 * LEG["y"] * (LEG["x_out"] - LEG["x_rim"]) ** 2 / 6
    Mb = F_leg * (Z_ROOT - 78.0) + F_mot * (Z_ROOT - sum(MOT_Z) / 2)
    sig = Mb / S
    I_lim = MOTOR["A_stall"] * MOTOR["gearbox_limit_Nm"] / MOTOR["stall_Nm"]
    return dict(motor=MOTOR["name"], pinion="brass m0.3 z12, 2 mm bore", skirt=f"z{SKIRT['z']} m0.3, pitch r {RG}", axis_r=AX_R, axis_xy=[round(AX[0], 2), AX[1]],
                usable_torque_Nm=round(T_use, 4), push_N=round(push, 1), need_N_est=3.0, root_deg_s=round(deg_s, 1),
                motor_spin_N=round(F_mot), leg_set_g=round(m_leg, 2), leg_spin_N=round(F_leg), leg_root_MPa=round(sig), leg_SF_AlSi10Mg_230=round(230 / sig, 2),
                current_limit_A=round(I_lim, 3), root_AlSi10Mg_g=round(root_f3(0).volume() * 2.68e-3, 2),
                stage1_lift=STAGE1, key_top_retracted=MD.KEY["top_retracted"], lift=MD.KEY["lift"])
