"""hand3 hub v3 (2026-11-19): finger roots that travel round the hub, locked by two axial latch pins each.
Agreed design: option F (roots travel), axial two-pin lock beside the neck, sideways swing dropped, cord route B (one pull ring,
rail + bead per finger).  Built from hand3_root_lock_bolt.json's layout with the corrections the CAD forced (see FIX below).

Frame: hand3's own (rotor axis z, hub plates at z 91..101).  A root at azimuth `az` is the finger of hand3_cad turned to az with
lag 0.  Units mm.  Estimates are marked est.

FIX (found while modelling, 2026-11-19):
 1. The cage (r 19.5-21.5) is joined to the base disc's underside exactly where the pins would come out at r 20, so the pins no
    longer pass through the disc: each hole is a slot through a 1 mm top lip with a pocket under it (closed below by the cage
    top); the pin's foot catches under the lip.
 2. The root grip is 12 mm wide (not 10), so the pins sit at +-22 deg (7.5 mm off the centre line), the block is +-28 deg wide.
 3. The pin must lift until its leg clears the disc top: lift 2.1 mm (the layout's 2.3 assumed a through hole; with the lip
    pocket it is 2.1); arm slot 4.1 deep, 1.9 mm of block left under it.
"""
import os, json
import numpy as np
import manifold3d as mf
import hand3_design as H
import hand3_cad as C
import hand3_hubc as HC
from hand3_cad import cyl, box, app, T, rot

M_ = mf.Manifold
Z = H.Z
V = dict(disc_r=25.5, z0=Z["hub_lo"], disc_t=2.0, lip_t=1.0, gap=6.0, boss_r=7.0, boss_z0=Z["motor_top"], shaft_r=2.6,
         block_r=(9.0, 24.3), block_half=28.0, ribs=((13.5, 15.5), (22.3, 24.0)), rib_h=1.0, groove_clr=0.1,
         pin_r=20.0, pin_daz=22.0, wire_d=2.0, float_=0.5, foot=0.4, lift=2.1, tab_x=28.5, step=30.0,
         slot_w=2.1, arm_slot_w=2.2, top_screw_r=4.9, top_screw_n=3,
         rail=dict(stem=(25.0, 25.6), head=(24.6, 26.0), z_stem=1.0, z_head=1.0), bead=dict(r=(24.0, 26.7), dz=(0.5, 2.8), half_w=2.0, eye_r=26.3),
         cord_r=0.35, post=(26.2, 30.0, 3.5, 102.7))
Z_GAP = (V["z0"] + V["disc_t"], V["z0"] + V["disc_t"] + V["gap"])            # 93 .. 99
Z_TOP = (Z_GAP[1], Z_GAP[1] + V["disc_t"])                                  # 99 .. 101
Z_RING_TOP = HC.Z_OUT[1]                                                    # 87: top of the base pull ring band
PIN_LAT = V["pin_r"] * np.sin(np.radians(V["pin_daz"]))
PIN_X = V["pin_r"] * np.cos(np.radians(V["pin_daz"]))
Z_LEG0 = V["z0"] + 0.1                                                      # leg bottom when locked (foot in the pocket)
Z_ARM = (Z_GAP[1] - V["lift"] - V["wire_d"], Z_GAP[1] - V["lift"])          # arm when locked: 94.9 .. 96.9


def wedge(r0, r1, z0, z1, a0, a1, n=96):
    """Annular sector between azimuths a0..a1 (deg)."""
    w = cyl(r1, z0, z1, n=n) - cyl(r0, z0 - 1, z1 + 1, n=n)
    return C.half(C.half(w, [-np.sin(np.radians(a0)), np.cos(np.radians(a0)), 0], [0, 0, 0]),
                  [np.sin(np.radians(a1)), -np.cos(np.radians(a1)), 0], [0, 0, 0])


def hole_azimuths():
    """(azimuth of the hole, azimuth of the root it serves, side) for every hole: roots on the 30 deg grid, pins at +-22 deg."""
    return [((a + s * V["pin_daz"]) % 360, a, s) for a in np.arange(0, 360, V["step"]) for s in (-1, 1)]


def slot_xy(root_az, side, x0, x1, w):
    """A slot along the root's centre line direction at the pin's lateral offset (pins move along the centre line)."""
    return app(box([x0, side * PIN_LAT - w / 2, 0.0], [x1, side * PIN_LAT + w / 2, 200.0]), T(rot("z", root_az)))


def base_disc():
    zt = V["z0"] + V["disc_t"]
    m = cyl(V["disc_r"], V["z0"], zt, n=128) + cyl(V["boss_r"], V["boss_z0"], Z_GAP[1], n=64)
    for r0, r1 in V["ribs"]:
        m -= cyl(r1 + V["groove_clr"], zt - V["rib_h"] - V["groove_clr"], zt + 1, n=128) - cyl(r0 - V["groove_clr"], zt - 2, zt + 2, n=128)
    m -= cyl(V["shaft_r"], V["boss_z0"] - 1, Z_GAP[1] + 1, n=32)
    r_in, r_out = PIN_X - 0.25 - 1.05, PIN_X - 0.25 + V["float_"] + 1.0 + 0.05    # lip slot: leg at the inner .. outer position (pin centre at PIN_X - 0.25 + out)
    for ha, ra, s in hole_azimuths():
        lip = slot_xy(ra, s, r_in, max(r_out, PIN_X - 0.25 + 1.0 + V["foot"] + 0.15), V["slot_w"]) ^ box([-60, -60, zt - V["lip_t"]], [60, 60, zt + 0.5])
        pocket = slot_xy(ra, s, r_in - 0.1, PIN_X + V["float_"] + 1.0 + V["foot"] + 0.25, V["slot_w"] + 0.3) ^ box([-60, -60, V["z0"] - 0.5], [60, 60, zt - V["lip_t"] + 0.01])
        m -= lip + pocket
    for k in range(V["top_screw_n"]):                                            # M2 thread-forming screws from the top disc
        a = np.radians(90 + 360 / V["top_screw_n"] * k); m -= cyl(0.8, Z_GAP[1] - 5, Z_GAP[1] + 1, c=(V["top_screw_r"] * np.cos(a), V["top_screw_r"] * np.sin(a), 0), n=16)
    return m


def top_disc():
    z0 = Z_TOP[0]
    m = cyl(V["disc_r"], *Z_TOP, n=128)
    for r0, r1 in V["ribs"]:
        m -= cyl(r1 + V["groove_clr"], z0 - 1, z0 + V["rib_h"] + V["groove_clr"], n=128) - cyl(r0 - V["groove_clr"], z0 - 2, z0 + 2, n=128)
    m -= cyl(V["shaft_r"], z0 - 1, Z_TOP[1] + 1, n=32)
    for k in range(V["top_screw_n"]):
        a = np.radians(90 + 360 / V["top_screw_n"] * k); c = (V["top_screw_r"] * np.cos(a), V["top_screw_r"] * np.sin(a), 0)
        m -= cyl(1.1, z0 - 1, Z_TOP[1] + 1, c=c, n=16) + cyl(1.9, Z_TOP[1] - 1.3, Z_TOP[1] + 1, c=c, n=16)
    return m


_RG = {}


def root_block(az=0.0):
    """Root block = the finger's own root grip (hand3_cad, lag 0) merged with a wedge that rides between the discs on two ribs top
    and bottom; two radial-slot bores and two arm slots for the latch pins.  Built at az 0 and turned."""
    if "m" not in _RG:
        R = C.parts(); zb = H.Z["blade"]
        grip = app(R["root_grip"], T(t=[0, 0, zb]))
        b = wedge(V["block_r"][0], V["block_r"][1], *Z_GAP, -V["block_half"], V["block_half"])
        for r0, r1 in V["ribs"]:
            b += wedge(r0, r1, Z_GAP[0] - V["rib_h"], Z_GAP[0] + 0.01, -V["block_half"], V["block_half"])
            b += wedge(r0, r1, Z_GAP[1] - 0.01, Z_GAP[1] + V["rib_h"], -V["block_half"], V["block_half"])
        post = box([V["post"][0], -V["post"][2], H.Z["blade"] + 2.0], [V["post"][1], V["post"][2], V["post"][3]])   # carries the spring box
        m = b + grip + post
        for s in (-1, 1):
            m -= slot_xy(0.0, s, PIN_X - 1.05 - 0.25, PIN_X + V["float_"] + 1.05, V["slot_w"]) ^ box([-60, -60, Z_GAP[0] - 2], [60, 60, Z_GAP[1] + 2])   # leg bore
            m -= slot_xy(0.0, s, PIN_X - 1.1 - 0.25, 40.0, V["arm_slot_w"]) ^ box([-60, -60, Z_ARM[0] - 0.05], [60, 60, Z_GAP[1] + 2])            # arm slot
        # curl cord: from the bead under the disc rim straight up into the grip's underside at r = bead eye (channel dia 1.2)
        for s in (-1, 1):                                                    # 2:1 tackle: anchor strand and running strand
            m -= cyl(0.6, Z_GAP[0] - 3, H.Z["blade"] + 1.0, c=(V["bead"]["eye_r"], s * 0.7, 0), n=16)
        _RG["m"] = m
    return _RG["m"].rotate([0, 0, float(az)])


def latch_pin(az=0.0, side=1, lifted=0.0, out=V["float_"]):
    """Bent dia 2 music wire: leg down through the block into the disc's lip slot, arm along the root out to a lift tab, a 0.4 mm
    foot at the leg's bottom pointing outward.  `out`: how far the spin has thrown it outward (0..float_); `lifted`: fork lift."""
    r = V["wire_d"] / 2; x = PIN_X + out - 0.25; y = side * PIN_LAT; zl = Z_LEG0 + lifted; za = (Z_ARM[0] + Z_ARM[1]) / 2 + lifted
    leg = cyl(r, zl, za + r, c=(x, y, 0), n=16)
    arm = cyl(r, x, V["tab_x"], "x", c=(0, y, za), n=16)
    foot = box([x, y - r * 0.8, zl], [x + r + V["foot"], y + r * 0.8, zl + 0.8])
    return (leg + arm + foot).rotate([0, 0, float(az)])


def ring_rail():
    """T-rail on top of the base pull ring's band (printed with the ring): the beads slide round it."""
    z = Z_RING_TOP; rl = V["rail"]
    return (cyl(rl["stem"][1], z - 0.01, z + rl["z_stem"], n=128) - cyl(rl["stem"][0], z - 1, z + 2, n=128)) + \
           (cyl(rl["head"][1], z + rl["z_stem"], z + rl["z_stem"] + rl["z_head"], n=128) - cyl(rl["head"][0], z, z + 3, n=128))


def bead(az=0.0, pull=0.0):
    """C-slider round the rail head (est. brass or printed PA-CF): open only along the stem; the curl cord is tied to an eye on its
    outer top edge at r eye_r, outside the disc rim.  Moves down with the ring."""
    bd = V["bead"]; z = Z_RING_TOP - pull; hw = np.degrees(bd["half_w"] / 25.3); rl = V["rail"]
    m = wedge(bd["r"][0], bd["r"][1], z + bd["dz"][0], z + bd["dz"][1], -hw, hw, n=64)
    zh = (z + rl["z_stem"] - 0.1, z + rl["z_stem"] + rl["z_head"] + 0.1)
    m -= cyl(rl["head"][1] + 0.1, *zh, n=128) - cyl(rl["head"][0] - 0.1, zh[0] - 1, zh[1] + 1, n=128)
    m -= cyl(rl["stem"][1] + 0.15, z - 1, zh[0] + 0.01, n=128) - cyl(rl["stem"][0] - 0.15, z - 2, zh[1], n=128)
    m -= cyl(0.5, -3, 3, "y", c=(bd["eye_r"], 0, z + bd["dz"][1] - 0.3), n=12)          # eyelet for the 2:1 tackle (cord turns round it)
    return m.rotate([0, 0, float(az)])


def cord_pts(az=0.0, pull=0.0):
    """Curl cord from the bead eye straight up into the root block (centre line, r = eye)."""
    a = np.radians(az); r = V["bead"]["eye_r"]
    return np.array([[r * np.cos(a), r * np.sin(a), Z_RING_TOP - pull + V["bead"]["dz"][1]], [r * np.cos(a), r * np.sin(a), H.Z["blade"]]])


def finger(az, fold=(0.0, 0.0, 0.0)):
    """Phalanges + pads of a finger whose root sits at az (hand3_cad.finger_frames with lag 0, turned)."""
    R = C.parts(); fr = C.finger_frames(0, 0.0, fold); A = T(rot("z", az - C.FAZ[0])); out = {}
    for j in range(3):
        out[f"phalanx{j + 1}"] = app(R[f"phalanx{j + 1}"], A @ fr[j + 1]); out[f"pad{j + 1}"] = app(R[f"pad{j + 1}"], A @ fr[j + 1])
    return out


def spring_box(az, k=0):
    """hub E's return-spring box, carried by the root at az on the root block's post (screwed to it, est.).  v3 has no swing, so all
    four fingers use the same ("R") box: an L box 60 deg clockwise of an R box overlaps it (35 mm3, range check), same-hand boxes
    clear by 1.6 mm at 60 deg."""
    R = C.parts(); hand = "R"
    m = R[f"spring_tray1_{hand}"] + R[f"spring_tray2_{hand}"] + R[f"spring_lid_{hand}"]
    return m.rotate([0, 0, float(az)])


POSES = {"flight cross": (0, 90, 180, 270), "2 + 2 pinch": (30, 150, 210, 330), "3 + 1 thumb": (30, 90, 150, 270), "all four on one side": (0, 60, 120, 180)}


def assembly(pose=POSES["flight cross"], fold=(0.0, 0.0, 0.0), pull=0.0, lifted=None, springs=True):
    """name -> manifold (hand3 frame).  lifted: index of the root whose pins the fork holds up."""
    out = {"base disc": base_disc(), "top disc": top_disc(), "base ring rail": ring_rail().translate([0, 0, -pull])}
    for k, az in enumerate(pose):
        out[f"root block {k}"] = root_block(az)
        for s in (-1, 1): out[f"latch pin {k}{'ab'[s > 0]}"] = latch_pin(az, s, V["lift"] if lifted == k else 0.0, 0.0 if lifted == k else V["float_"])
        out[f"bead {k}"] = bead(az, pull)
        for n, m in finger(az, fold).items(): out[f"finger {k} {n}"] = m
        if springs: out[f"spring box {k}"] = spring_box(az, k)
    return out


# ---------------------------------------------------------------- drive (decision S1: 2:1 tackle, both servos pull the one ring)
TACKLE = 2.0
K_STROKE = dict(K1=HC.LEVER[0] * np.radians(H.FOLD_MAX[0]), K23=HC.LEVER[1] * np.radians(H.FOLD_MAX[1]) + HC.LEVER[2] * np.radians(H.FOLD_MAX[2]))
CORD_STROKE = K_STROKE["K1"] + K_STROKE["K23"] + HC.STRETCH                    # one cord over K1..K3: 22.2 mm
RING_STROKE = CORD_STROKE / TACKLE                                            # 11.1 mm, both yoke pairs together
LIP_R0 = HC.R_LIP_OUT[0] + 0.6                                                # all lips moved out: beads and cords pass inside them


def yoke(which, side, pull=0.0):
    """A yoke half hooking the base ring's flange.  'out' halves are hub E's base yokes; 'in' halves are the curl servo's yokes,
    now made like the 'out' ones (lip over the base ring flange, legs 18 / 8) at their own azimuths and rods.  Lip inner edge at
    LIP_R0.  Moves down with the ring when pulling (pull + the 1 mm flight gap)."""
    az, (ax, ay), _, _ = HC.HALVES[(which, side)]
    lz = HC.Z_LIP_OUT; lf = HC.LEG["out"]; arc = HC.arc
    m = arc(LIP_R0, HC.R_LIP_OUT[1] + 1.0, lz[0], lz[1], az, HC.LIP_HALF)
    m += arc(HC.R_FL_OUT + HC.ARM_CLR, HC.R_LEG[1], lz[0] - 2.5, lz[0] + 0.5, az, HC.LEG_HALF + 2.0)
    zb0, zb1 = lz[0] - lf[0], lz[0] - lf[1]
    m += arc(HC.R_LEG[0], HC.R_LEG[1], zb0, lz[0] + 0.5, az, HC.LEG_HALF)
    bosses = [cyl(3.2, zb0, zb1, c=(*HC.rod_xy(az, k), 0)) for k in (-1, 1)]
    eye = cyl(2.2, zb0, zb0 + 4.0, c=(ax, ay, 0))
    m += M_.batch_hull(bosses + [app(box([HC.R_LEG[0], -3.0, zb0], [HC.R_LEG[0] + 1.0, 3.0, zb1]), T(rot("z", az))), eye])
    for k in (-1, 1): m -= cyl(1.6, zb0 - 1, zb1 + 1, c=(*HC.rod_xy(az, k), 0))
    m -= cyl(0.6, zb0 - 1, zb0 + 5.0, c=(ax, ay, 0)) + cyl(1.4, zb0 + 2.0, zb0 + 5.0, c=(ax, ay, 0))
    return m.translate([0, 0, -(pull + HC.GAP) if pull > 0 else 0.0])


def drive(pull=0.0):
    """name -> manifold: the static drive of hub E minus the inner (curl) ring, with the four yoke halves on the base ring."""
    hw = HC.hardware(); P = HC._parts(); out = {}
    out["frame"] = P["frame"][0]; out["tie ring"] = P["tie_ring"][0]
    for n, m in hw.items():
        if n == "motor_base": continue
        out[n] = m
    for w in ("out", "in"):
        for sd in ("near", "far"): out[f"yoke {w} {sd}"] = yoke(w, sd, pull)
    return out


def tackle_cords(az=0.0, pull=0.0):
    """2:1 tackle: the cord is anchored in the root block, runs down round the bead's eyelet and back up into the grip channel.
    Two strands at +-0.7 mm beside the root's centre line, r = bead eye."""
    a = np.radians(az); r = V["bead"]["eye_r"]; zb = Z_RING_TOP - pull + V["bead"]["dz"][1] - 0.3
    u = np.array([np.cos(a), np.sin(a), 0.0]); n = np.array([-np.sin(a), np.cos(a), 0.0])
    return [np.array([u * r + n * s * 0.7 + [0, 0, zb], u * r + n * s * 0.7 + [0, 0, Z_GAP[0] - 1.0]]) for s in (-1, 1)]


def export(d="hand3_v3_stl"):
    os.makedirs(d, exist_ok=True)
    for n, m in (("base_disc_PA-CF", base_disc()), ("top_disc_PA-CF", top_disc()), ("root_block_PA-CF", root_block(0)), ("bead_PA-CF", bead(0)),
                 ("latch_pin_music-wire", latch_pin(0, 1))):
        rho = 7.85e-3 if "wire" in n else 1.25e-3
        C.to_trimesh(m).export(f"{d}/{n}.stl"); print(f"  {n:24s} {m.volume() * rho:6.2f} g  bbox {np.round(np.asarray(m.bounding_box()), 1)}")


if __name__ == "__main__":
    export()


# ---------------------------------------------------------------- base ring and cage for v3 (S1 strength, station roller)
FLANGE_T = 3.0                                                                 # was 2: both servos pull this flange (bending 43.8 -> 19.5 MPa at stall)
Z_FL = (HC.Z_OUT[1] - 1.0 - FLANGE_T, HC.Z_OUT[1] - 1.0)                       # 83 .. 86 (top unchanged)
HANGER_Z0 = HC.lug_z()[0]                                                      # 78: hanger leg bottom (keys the ring to the cage)
CAGE_TRACK = (HC.HL - 31.5, HANGER_Z0 - RING_STROKE - 1.0)                     # continuous cage band for the roller: 59.5 .. 65.9


def ring_v3():
    """Base pull ring for v3: band, a 3 mm flange, the four hanger keys (no TPU-link lugs), and the bead rail."""
    m = cyl(HC.R_OUT[1], HC.Z_OUT[0], HC.Z_OUT[1], n=128) - cyl(HC.R_OUT[0], HC.Z_OUT[0] - 1, HC.Z_OUT[1] + 1, n=128)
    m += cyl(HC.R_FL_OUT, *Z_FL, n=128) - cyl(HC.R_OUT[0], Z_FL[0] - 1, Z_FL[1] + 1, n=128)
    for a in C.FAZ:
        ab = HC._hanger_az(a); hw = HC.adeg(HC.HANGER_W / 2, 22.0)
        m += HC._wedge_az(HC.LUG["r"][0], HC.R_OUT[0] + 0.3, HANGER_Z0, HC.Z_OUT[1], ab - hw, ab + hw)
    return m + ring_rail()


def cage_v3():
    """Cage tube with only narrow key slots for the ring's hangers (hub E's 61 deg link slots are gone): continuous below z 65.9."""
    m = cyl(HC.R_CAGE[1], HC.HL - 31.5, HC.HL, n=128) - cyl(HC.R_CAGE[0], HC.HL - 40, HC.HL + 3, n=128)
    for a in C.FAZ:
        ab = HC._hanger_az(a); hw = HC.adeg(HC.HANGER_W / 2 + 0.3, 20.5)
        m -= HC._wedge_az(HC.R_CAGE[0] - 1, HC.R_CAGE[1] + 1, CAGE_TRACK[1], HC.HL + 3, ab - hw, ab + hw)
    return m


# ---------------------------------------------------------------- bell crank on each root (station A)
# In the root's frame (x out along the root, y sideways, z up).  Pivot on a dia 1 steel pin through the grip neck; the upper finger
# lies under the latch pin's arm, the lower tip hangs outside the disc rim just under the disc, above the bead rail.
CR = dict(pivot=(27.0, 95.5), finger=(19.5, 93.85, 94.8), tip=(28.7, 87.6, 88.6), tip_y=(3.8, 5.3), jog_z=(92.3, 93.3), plate_y=(8.6, 10.0), finger_y=(6.7, 8.3), t=1.0, pin_d=1.0)
CR_ANGLE = float(np.degrees(np.arcsin(V["lift"] / np.hypot(CR["pivot"][0] - CR["finger"][0], CR["pivot"][1] - CR["finger"][2]))))   # turn for 2.1 mm lift


def crank(side=1):
    """One bell crank (PA-CF, est. 0.15 g), at rest, for the pin on `side` (+1 / -1 in y).  Built in the root frame at az 0."""
    px, pz = CR["pivot"]; y0, y1 = CR["plate_y"]; f0, f1 = CR["finger_y"]
    t0, t1 = CR["tip_y"]; j0, j1 = CR["jog_z"]
    lo = box([CR["tip"][0] - 0.5, t0, CR["tip"][1]], [CR["tip"][0] + 0.5, t1, j1])          # lower tip, close to the centre line
    lo += box([CR["tip"][0] - 0.5, t0, j0], [CR["tip"][0] + 0.5, y1, j1])                    # jog under the neck, out to the plate
    lo += box([CR["tip"][0] - 0.5, y0, j0], [CR["tip"][0] + 0.5, y1, pz - 1.0])
    piv = cyl(1.3, y0, y1, "y", c=(px, 0, pz), n=24)
    xo = 25.4                                                                  # outside the block (r 24.3 even at y 10)
    up = box([xo, y0, CR["finger"][1]], [px, y1, CR["finger"][2]])
    fing = box([CR["finger"][0], f0, CR["finger"][1]], [xo + 1.0, f1, CR["finger"][2]]) + box([xo, f0, CR["finger"][1]], [xo + 1.0, y1, CR["finger"][2]])
    m = lo + M_.batch_hull([box([CR["tip"][0] - 0.5, y0, pz - 1.2], [CR["tip"][0] + 0.5, y1, pz - 1.0]), piv]) + M_.batch_hull([piv, up]) + fing
    m -= cyl(CR["pin_d"] / 2 + 0.05, y0 - 1, y1 + 1, "y", c=(px, 0, pz), n=16)
    return m if side > 0 else m.mirror([0, 1, 0])


def crank_pose(side, turn):
    """4x4 (root frame): the crank turned by `turn` deg about its pivot (positive = tip inward, finger up)."""
    px, pz = CR["pivot"]; a = np.radians(turn); c, s = np.cos(a), np.sin(a)
    Ry = np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]])      # tip (below the pivot) moves -x, finger (inside) moves +z
    Tp = np.eye(4); Tp[0, 3], Tp[2, 3] = px, pz; Tm = np.eye(4); Tm[0, 3], Tm[2, 3] = -px, -pz
    return Tp @ Ry @ Tm


def crank_pin(side=1):
    y0 = 5.0; y1 = CR["plate_y"][1] + 0.3
    m = cyl(CR["pin_d"] / 2, y0, y1, "y", c=(CR["pivot"][0], 0, CR["pivot"][1]), n=12)
    return m if side > 0 else m.mirror([0, 1, 0])


def root_cuts_for_crank():
    """Deepen each arm slot where the crank finger lies under the pin arm, and a dia 1.05 pivot hole through the neck."""
    m = None
    for s in (-1, 1):
        c = box([CR["finger"][0] - 1.0, s * 7.5 - 1.15, CR["finger"][1] - 0.15], [V["block_r"][1] + 0.5, s * 7.5 + 1.15, V3_Z_ARM0 + 0.1])
        c += cyl(CR["pin_d"] / 2 + 0.03, -12, 12, "y", c=(CR["pivot"][0], 0, CR["pivot"][1]), n=12)
        m = c if m is None else m + c
    return m


V3_Z_ARM0 = Z_ARM[0]
_RG_OLD = root_block


def root_block(az=0.0):
    if "v3s" not in _RG:
        _RG["v3s"] = _RG_OLD(0.0) - root_cuts_for_crank()
    return _RG["v3s"].rotate([0, 0, float(az)])


# ---------------------------------------------------------------- station (static, at STATION_AZ)
STATION_AZ = 90.0
ST = dict(roll_gap=1.5, lost=2.0, tip_r0=30.1, push_y=(3.8, 5.3), push_z=(87.7, 88.5), push_r1=31.0, stem_z0=86.4,
          body_r0=33.6, body_z=(84.6, 87.6), width=11.0, prong_y=(2.3, 3.4), prong_z=(86.4, 88.3), prong_r0=29.2,
          tyre_d=20.0, tyre_z=(CAGE_TRACK[0] + 0.8, CAGE_TRACK[0] + 5.8), n20=dict(w=12.0, d=10.0, L=24.0))
ST["contact"] = ST["tip_r0"] - (CR["tip"][0] + 0.5)                                # blade travel before it meets the crank tips


def _tip_x(turn):
    """x of the crank tip's outer face (mid height) after a turn (deg)."""
    p = crank_pose(1, turn) @ np.array([CR["tip"][0] + 0.5, 0, (CR["tip"][1] + CR["tip"][2]) / 2, 1.0]); return p[0]


def crank_turn(s):
    """Crank turn (deg) for carriage travel s: solves tip x = tip x at rest - push."""
    push = max(0.0, s - ST["lost"] - ST["contact"]); x0 = _tip_x(0.0); lo, hi = 0.0, 60.0
    for _ in range(50):
        mid = (lo + hi) / 2
        if x0 - _tip_x(mid) < push: lo = mid
        else: hi = mid
    return float(lo)


def station_parts():
    """Static station at az 0 (turned to STATION_AZ by the caller), retracted.  x = radial.  Carriage = blade (on a lost-motion slot)
    + roller (on a spring).  The bead is not held: its two tackle strands are anchored in the held root and drag it round the rail.  Returns name -> manifold in their retracted places."""
    r0 = ST["tip_r0"]; w = ST["width"]; b0, b1 = ST["body_z"]
    blade = box([ST["body_r0"], -w, b0], [48.0, w, b1])                               # body under the finger, outside the ring flange
    for sgn in (-1, 1):
        y0, y1 = sorted((sgn * ST["push_y"][0], sgn * ST["push_y"][1]))
        blade += box([r0, y0, ST["push_z"][0]], [ST["push_r1"], y1, ST["push_z"][1]])                        # push finger, below z 89
        blade += box([r0 + 0.5, y0, ST["stem_z0"]], [ST["body_r0"] + 0.5, y1, ST["push_z"][0] + 0.1])     # its stem, over the flange
    spine = box([44.0, -3.0, 58.0], [48.0, 3.0, b1])
    tc = HC.R_CAGE[1] + ST["roll_gap"] + ST["tyre_d"] / 2
    tyre = cyl(ST["tyre_d"] / 2, *ST["tyre_z"], c=(tc, 0, 0), n=64) - cyl(1.5, ST["tyre_z"][0] - 1, ST["tyre_z"][1] + 1, c=(tc, 0, 0), n=16)
    n = ST["n20"]; zf = ST["tyre_z"][0] - 1.0
    motor = box([tc - n["d"] / 2, -n["w"] / 2, zf - n["L"]], [tc + n["d"] / 2, n["w"] / 2, zf]) + cyl(1.5, zf, ST["tyre_z"][1] + 1, c=(tc, 0, 0), n=16)
    motor += box([tc + n["d"] / 2, -3.0, zf - 8.0], [44.0, 3.0, zf - 2.0])               # motor arm to the carriage spine (sprung)
    servo = box([48.5, -6.0, 60.0], [60.5, 6.0, 83.0])                                    # SG92R 23 x 12 x 29 standing (est. placement)
    sensor = box([ST["body_r0"] + 0.6, -14.5, b0], [ST["body_r0"] + 3.6, -12.0, b1])        # QTR-1A on the blade, looks inward at the crank tips
    return dict(blade=blade + spine, tyre=tyre, motor=motor, servo=servo, sensor=sensor)


def station_pose(s):
    """Carriage travel s -> radial offset of each part: the roller moves first and stops on the cage (spring), the blade waits
    on its lost-motion slot until s > lost, then pushes the crank tips."""
    rb = -max(0.0, s - ST["lost"]); rr = -min(s, ST["roll_gap"])
    return dict(blade=rb, sensor=rb, servo=0.0, tyre=rr, motor=rr)


ST["stroke"] = ST["lost"] + ST["contact"] + float(_tip_x(0.0) - _tip_x(CR_ANGLE + 1.0))   # push until 1 deg past a full lift
