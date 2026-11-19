"""Hand connector rev B for Cephalo-2 (2026-11-18): motor power through the joint, so motor hands really swap.
Replaces the rev A bayonet (handmount.py), whose 5 spring pins carry 5.2 A each; hand3 had to hard-wire its rotor's
phase leads down the arm, and a wheel hand could not be swapped at all.

What changes (rev A -> rev B):
  * three motor phase contacts: Castle Creations 4 mm high-current bullets (75 A).  Female halves sit in the arm socket,
    males stand out of the hand base.  The hand's own motor (hand3 rotor, wheel motor, ...) runs off the arm's ESC.
  * the hand no longer twists: bullets need a straight push.  The hand base has a key, enters straight, and a
    LOCKING COLLAR on the socket turns 60 deg: three internal lip segments slide behind the hand's flange lugs and hold them.
    A ball detent holds the collar at locked and at open.
  * the five spring pins (12 V, GND, servo data, servo return, hand ID) stay, on the same pin board at the centre.
  * sized for the wheel hand: 9.4 N m bending + 110 N radial + 1.9 N m twist (3 g landing on a wheel, est.).
Frame (mm): z = connector axis, +z into the arm; z = 0 = socket mouth face; the hand hangs toward -z.
Rev B.1 (same day, from the assembly check): rev B's collar could neither be fitted nor hold a pulled hand (its lip
r 20.9 < socket r 21, and nothing under it to bear on).  Now: socket body r 20.3, a separate NOSE RING at the mouth
that the collar's step bears on, 3 x M3 x 20 DIN 7984 at az 30/150/270 (clear of the bullets) through ring + socket
into inserts in the tip bay, radial 3 mm ball detent, lead grooves under the hand base.  Notes: coupler-revB.md.
Parts: arm_socket + nose_ring (PA-CF, 3 x M3 into the tip bay end), collar (PA-CF), hand_base (PA-CF, shared by every hand).
python3 handmount_b.py -> checks (overlaps at open / pushed / locked, collar sweep, bullet alignment, stresses), STLs
"""
import os, json
import numpy as np
import manifold3d as mf

N = 120
P = dict(
    # socket body
    R_body=20.3, z_back=16.0, r_bore=10.6, z_bore=11.0,
    # bullets (Castle 4 mm: male pin dia 4.0, contact length 9; female barrel OD 5.6 in a 6.0 bore, est. from the part)
    r_bullet=15.4, az_bullet=(90.0, 210.0, 330.0), pin_d=4.0, pin_len=9.0, fem_bore=6.0, fem_depth=13.0, fem_recess=1.2,
    # key: rib on the plug, slot in the bore
    az_key=30.0, key_w=3.0, key_h=1.6,
    # hand base
    R_flange=20.6, r_spigot=10.35, z_spigot_top=10.4, lug_az=(0.0, 120.0, 240.0), lug_r0=20.4,
    # rev B.2 primaries (set by coupler_opt.py): lugs at the TOP of the flange, clamped between the nose ring and the lip
    lug_w=50.0, lug_r1=23.3,      # B.2 48 / 22.4; B.4 56 / 23.3; B.5 50 (lips overhang) (FEA: lug root + lip-wall fillet bands; bearing room for the fillet)
    t_lug=3.6, t_lip=3.2, t_wall=4.0,   # B.2 2.6 / 2.7 / 3.3
    t_step=2.4, ax_clr=0.1,
    lip_r0=20.9, collar_bore=20.5, twist=60.0, ramp=0.4, ramp_deg=8.0, flute=0.5,
    retain_r=23.0,
    # nose ring: separate ring at the mouth, clamped by the 3 socket screws; the collar's -z stop and the hand's seat
    z_nose=3.5, cbore_d=6.2, cbore_h=2.2,          # counterbores for M3x20 DIN 7984 low-head screws (head 5.5 x 2.0)
    # ball detent: 3 mm steel ball on a small spring in a radial blind bore in the socket body; dimples in the collar bore
    det_az=0.0, ball_d=3.0, det_bore=3.1, det_r_in=12.5, ball_rc=19.8,   # ball centre radius when seated: 1.0 proud of the body
    # pins (rev A board, unchanged)
    z_pad=5.0, pad_t=1.6, r_pad=7.0, pinboard_r=10.5, z_pinboard=9.3, pinboard_t=1.6, pin_r=(0.0, 2.3, 3.6, 4.9, 6.2),
    screws=((17.0, 30.0), (17.0, 150.0), (17.0, 270.0)), screw_d=3.4,      # 3 screws, each midway between two bullets
    hand_screw_r=13.0, hand_screw_az=(45.0, 135.0, 225.0, 315.0),   # 4 x M3 heat-set inserts (dia 4.0 x 6) for the hand's own body
    groove_w=3.2, groove_h=2.6, r_wire_cb=6.5,                       # motor-lead grooves in the hand base's bottom face
    # rev B.3 (2026-11-18, coupler_preload_study.py, option A): face O-ring in the nose-ring face, inside the bullets.  It pushes
    # the hand out until its lugs bear on the collar lips, so the 0.1 mm play cannot rattle.  NBR 70A 22 x 1.0, groove 11.0-12.4
    # rev B.3.1 corrections: the groove faces DOWN (toward the hand), so a loose ring would drop out with no hand on: the ring is
    # a 21 x 1.0, stretched 4.8 % onto the groove's Ø22 inner wall, which holds it.  And the force is tolerance-sensitive
    # (12-80 N for +-0.1 mm), so the groove is printed DEEP (0.85) and set with 0.05 mm PET shim washers under the ring after
    # measuring the printed stack: target squeeze 0.17 mm, +-0.025 after tuning.
    oring_id=21.0, oring_cs=1.0, og_r=(11.0, 12.4), og_depth=0.85, shim_step=0.05, sq_target=0.17, oring_E=5.5, mu=0.30,
    # rev B.4 (2026-11-19, coupler_fea.py): the FEA put the lug ROOT at factor 0.8-0.9 (z-tension at the sharp re-entrant
    # corner under the lug, worst at the lug's side edge where the 66 deg lip ran past the 48 deg lug).  Fixes: a root fillet
    # (gusset) under each lug, a matching chamfer on the lip's inner top edge, lips stopped lip_inset inside the lug edges,
    # a fillet where the lip meets the collar wall, a chamfer on the lug's bottom outer edge to clear it, lugs 1 mm longer.
    lug_fillet=1.0, lip_chamfer=0.9, wall_fillet=0.8, arc_n=8, lug_chamfer=0.5, lip_inset=0.0,   # B.5: unused (lips overhang the lugs, see lip_w)
    # B.4 FEA pass 2: a square lip end acts as a punch edge (flange z-tension 37-47 MPa at the lip end): the trailing end of each
    # lip is relieved lip_relief over lip_relief_deg (the leading end already has the 0.4 mm lead-in ramp), lips as wide as the lugs
    lip_relief=0.12, lip_relief_deg=8.0,
    # rev B.5 (2026-11-19, FEA B.4: collar lip-wall fillet 20-28 MPa): the lip is made as long as the insertion gaps allow
    # (lip_w = 120 - gap_w) so it overhangs each lug end and spreads the wall's load over more arc; both ends are relieved
    # from relief_in deg inside the lug edge (lip_relief at the edge, rising on over the overhang) so the lug's end never
    # bears on a full-height lip.  The lead-in ramp still sets the leading tip.
    relief_in=4.0, lip_overhang=True,
    # rev B.3.1 optimisation: six lightening pockets in the socket body from its back face, between the bullets and the screws
    pocket_r=(12.0, 19.0), pocket_z=8.0, pocket_az=(0.0, 60.0, 120.0, 180.0, 240.0, 300.0), pocket_half=(17.0, 17.0), pocket_wall=1.0,
)


def derive():
    """Everything that follows from the primaries."""
    P["gap_w"] = P["lug_w"] + 6.0                                  # only the insertion gap minimum; lips are lip_w wide
    P["lip_w"] = 120.0 - P["gap_w"] if P["lip_overhang"] else P["lug_w"] - 2 * P["lip_inset"]
    P["lug_r"] = (P["lug_r0"], P["lug_r1"]); P["lug_z"] = (-P["t_lug"], 0.0)
    P["R_nose"] = P["lug_r1"] + 0.1; P["collar_cb"] = P["lug_r1"] + 0.5
    zl = -P["t_lug"] - P["ax_clr"]; P["lip_z"] = (zl - P["t_lip"], zl); P["lip_r"] = (P["lip_r0"], P["collar_cb"])
    P["z_flange"] = (min(P["lip_z"][0] - 1.2, -7.0), 0.0)          # flange bottom 1.2 below the collar: hands bolt on flat, no boss
    P["R_collar"] = P["collar_cb"] + P["t_wall"] + P["flute"]
    P["det_z"] = P["z_nose"] + 2.0
    z1 = max(P["z_nose"] + P["t_step"], P["det_z"] + 2.3)
    P["collar_z"] = (P["lip_z"][0], z1); P["sleeve_r"] = P["collar_bore"] + 2.2
    P["retain_z"] = (z1 + 0.2, z1 + 2.2)
    return P


derive()


def cyl(r, z0, z1, n=N):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([0, 0, z0])


def rev(profile, a0, a1, n=N):
    cs = mf.CrossSection([[(float(r), float(z)) for r, z in profile]])
    return mf.Manifold.revolve(cs, n, float(a1 - a0)).rotate([0, 0, float(a0)])


def sector(r0, r1, a0, a1, z0, z1):
    return rev([(r0, z0), (r1, z0), (r1, z1), (r0, z1)], a0, a1)


def lug(az):
    """One hand-base lug, revolved: r lug_r0..lug_r1, z -t_lug..0, with a root fillet (45 deg gusset, lug_fillet) down onto the
    flange cylinder and a lug_chamfer on the bottom outer edge."""
    t, R, f, c = P["t_lug"], P["R_flange"], P["lug_fillet"], P["lug_chamfer"]
    e = 0.3                                                       # the fillet runs on e into the flange: clean boolean, no slivers
    # true arc fillet radius f, centre (R + f, -t - f), from the lug underside (R + f, -t) round to 80 deg (10 deg short of tangent
    # to the flange cylinder, so the union does not make slivers), then straight on into the flange
    arc = [(R + f - f * np.sin(np.radians(q)), -t - f + f * np.cos(np.radians(q))) for q in np.linspace(0, 80, P["arc_n"] + 1)]
    xe, ze = arc[-1]
    prof = [(P["lug_r0"], ze - e), (xe - e * np.tan(np.radians(10)) - e, ze - e)] + arc[::-1] + [(P["lug_r1"] - c, -t), (P["lug_r1"], -t + c), (P["lug_r1"], 0.0), (P["lug_r0"], 0.0)]
    return rev(prof, az - P["lug_w"] / 2, az + P["lug_w"] / 2)


def lip_seg(a0, a1, ztop, n=N):
    """One collar lip step, revolved: lip_r0..(collar_cb + 0.25 into the wall), z collar bottom..ztop, with a lip_chamfer on the
    inner top edge (clears the lug's root fillet) and a wall_fillet (45 deg) in the corner where the lip top meets the wall."""
    z0 = P["collar_z"][0]; cb = P["collar_cb"]; c, w = P["lip_chamfer"], P["wall_fillet"]
    e = 0.3                                                       # the fillet runs on e into the wall: clean boolean, no slivers
    # true arc fillet radius w, centre (cb - w, ztop + w), from the lip top round to 80 deg (10 deg short of tangent to the wall)
    arc = [(cb - w + w * np.sin(np.radians(q)), ztop + w - w * np.cos(np.radians(q))) for q in np.linspace(0, 80, P["arc_n"] + 1)]
    xe, ze = arc[-1]
    prof = [(P["lip_r0"], z0), (cb + e, z0), (cb + e, ze + e)] + arc[::-1] + [(P["lip_r0"] + c, ztop), (P["lip_r0"], ztop - c)]
    return rev(prof, a0, a1, n)


def lip_drop(m):
    """Drop of the lip top (mm) at m deg from its leading end: lead-in ramp + trailing relief."""
    seg = P["lip_w"]
    if not P["lip_overhang"]:
        return P["ramp"] * np.clip(1 - m / P["ramp_deg"], 0, 1) + P["lip_relief"] * np.clip(1 - (seg - m) / P["lip_relief_deg"], 0, 1)
    ov = (seg - P["lug_w"]) / 2                                       # overhang past each lug edge, deg
    rel = lambda e: P["lip_relief"] * np.clip((P["relief_in"] - e) / P["relief_in"], 0, None)   # e: deg inside the lug edge
    return np.maximum(P["ramp"] * np.clip(1 - m / P["ramp_deg"], 0, 1), rel(m - ov) + 0 * m) + rel(seg - m - ov)


def lip(az, nst=48):
    """One lip centred on az: the flat lip_seg revolved in nst steps, its top warped down by lip_drop (blended linearly to 0 at
    the collar bottom), so the ramp and relief are smooth and the solid stays one clean manifold."""
    seg = P["lip_w"]; z0 = P["collar_z"][0]; zl = P["lip_z"][1]
    m = lip_seg(az - seg / 2, az + seg / 2, zl, n=int(round(nst * 360 / seg)))
    def w(v):
        v = np.array(v, float); a = (np.degrees(np.arctan2(v[:, 1], v[:, 0])) - (az - seg / 2) + 180) % 360 - 180
        v[:, 2] -= lip_drop(a) * np.clip((v[:, 2] - z0) / (zl - z0), 0, None)
        return v
    return m.warp_batch(w)


def at(r, az):
    a = np.radians(az); return r * np.cos(a), r * np.sin(a)


# ---------------------------------------------------------------- arm side
def _socket_cuts():
    """Holes shared by the socket body and the nose ring (they are drilled as one stack)."""
    cut = cyl(P["r_bore"], -1, P["z_bore"]) + cyl(P["pinboard_r"] + 0.1, P["z_bore"] - 0.1, P["z_back"] + 1)     # spigot bore + pin-board seat (wires out the back)
    cut += sector(P["r_bore"] - 0.5, P["r_bore"] + P["key_h"] + 0.3, P["az_key"] - P["key_w"] / 2 / P["r_bore"] * 57.3 - 2, P["az_key"] + P["key_w"] / 2 / P["r_bore"] * 57.3 + 2, -1, P["z_bore"])
    for az in P["az_bullet"]:
        x, y = at(P["r_bullet"], az)
        cut += cyl(P["fem_bore"] / 2, P["fem_recess"], P["fem_recess"] + P["fem_depth"], 32).translate([x, y, 0])
        cut += cyl(P["pin_d"] / 2 + 0.6, -1, P["fem_recess"] + 0.01, 32).translate([x, y, 0])           # mouth: pin lead-in, female recessed 1.2 (touch-safe)
        cut += cyl(1.6, 0, P["z_back"] + 1, 16).translate([x, y, 0])                                    # 14 AWG lead exit
    for r, az in P["screws"]:
        x, y = at(r, az)
        cut += cyl(P["screw_d"] / 2, -1, P["z_back"] + 1, 24).translate([x, y, 0])                       # M3 x 20 through nose ring + body into the tip bay
        cut += cyl(P["cbore_d"] / 2, -1, P["cbore_h"], 24).translate([x, y, 0])                          # low head sits below the mouth face
    return cut


def arm_socket():
    """Socket body: z_nose .. z_back, the collar rides on it (r 20.3) between the nose ring and the retaining flange."""
    b = cyl(P["R_body"], P["z_nose"], P["z_back"]) + cyl(P["retain_r"], P["retain_z"][0], P["retain_z"][1])
    cut = _socket_cuts()
    x, y = at(P["det_r_in"], P["det_az"]); a = np.radians(P["det_az"])
    bore = mf.Manifold.cylinder(P["R_body"] - P["det_r_in"] + 1, P["det_bore"] / 2, P["det_bore"] / 2, 20).rotate([0, 90, 0]).rotate([0, 0, P["det_az"]])
    cut += bore.translate([x, y, P["det_z"]])                                                             # radial blind bore: spring + ball
    return b - cut - pockets()


def pockets():
    """Lightening pockets, open at the socket's back face (the tip bay's end face closes them): annular sectors r 12-19 from
    z 8 to the back, centred between each bullet and its neighbouring screw, trimmed to keep pocket_wall round every bore."""
    out = []
    for az in P["pocket_az"]:
        out.append(sector(P["pocket_r"][0], P["pocket_r"][1], az - P["pocket_half"][0], az + P["pocket_half"][1], P["pocket_z"], P["z_back"] + 1))
    pk = mf.Manifold.batch_boolean(out, mf.OpType.Add)
    keep = []
    for az in P["az_bullet"]:
        keep.append(cyl(P["fem_bore"] / 2 + P["pocket_wall"], 0, P["z_back"] + 2, 32).translate([*at(P["r_bullet"], az), 0]))
    for r, az in P["screws"]:
        keep.append(cyl(P["screw_d"] / 2 + P["pocket_wall"] + 0.3, 0, P["z_back"] + 2, 24).translate([*at(r, az), 0]))
    return pk - mf.Manifold.batch_boolean(keep, mf.OpType.Add)


def nose_ring():
    """Mouth ring (PA-CF), clamped between the 3 screw heads and the socket body.  Fitted after the collar: it is what
    keeps the collar on, and the hand's flange seats on its face (z = 0).  Rev B.3: O-ring face groove inside the bullets."""
    g = cyl(P["og_r"][1], -0.01, P["og_depth"]) - cyl(P["og_r"][0], -1.0, P["og_depth"] + 1.0)
    return cyl(P["R_nose"], 0.0, P["z_nose"]) - _socket_cuts() - g


def oring(free=False):
    """The 22 x 1.0 O-ring seated on its groove floor, drawn round (uncompressed): where it overlaps the hand base's flange top
    is the squeeze.  free=True: the same ring, with the hand pushed out by it before the collar turns."""
    r = cs_eff() / 2; rc = P["og_r"][0] + r; zc = depth_eff() - r
    cs = mf.CrossSection.circle(r, 32).translate([rc, zc])
    return mf.Manifold.revolve(cs, N)


def cs_eff():
    """Cross-section of the ring stretched onto the groove's inner wall (volume kept: cs x (1 - stretch / 2))."""
    st = max(2 * P["og_r"][0] / P["oring_id"] - 1, 0.0)
    return P["oring_cs"] * (1 - st / 2)


def shims():
    """Number of shim_step PET washers under the ring that bring the squeeze nearest sq_target (nominal print)."""
    want = cs_eff() - P["ax_clr"] - P["sq_target"]                 # groove depth wanted
    return int(round(max(P["og_depth"] - want, 0.0) / P["shim_step"]))


def depth_eff():
    return P["og_depth"] - shims() * P["shim_step"]


def oring_force(squeeze):
    """Face-compressed O-ring (Lindley): N, for a squeeze in mm."""
    c = cs_eff(); e = max(squeeze, 0.0) / c; rc = P["og_r"][0] + c / 2
    return P["oring_E"] * c * (1.25 * e ** 1.5 + 50 * e ** 6) * 2 * np.pi * rc


def preload():
    """Axial preload on the lugs at lock (O-ring + 5 spring pins), its print-tolerance band, and what it means."""
    c = cs_eff(); sq = c - depth_eff() - P["ax_clr"]; pogo = 5 * 0.060 * 9.81; tt = P["shim_step"] / 2
    F = oring_force(sq) + pogo; Flo = oring_force(sq - tt) + pogo; Fhi = oring_force(sq + tt) + pogo
    rl = (P["lip_r0"] + P["lug_r1"]) / 2; tr = P["ramp"] / (rl * np.radians(P["ramp_deg"])); mu = P["mu"]
    lock = F * rl * 1e-3 * (tr + mu) / (1 - mu * tr) + 0.08
    sq0 = c - P["og_depth"] - P["ax_clr"]
    return dict(shims=shims(), shim_mm=round(shims() * P["shim_step"], 2), cs_stretched_mm=round(c, 3), squeeze_mm=round(sq, 3), squeeze_pct=round(100 * sq / c),
                squeeze_full_in_pct=round(100 * (sq + P["ax_clr"]) / c), squeeze_untuned_without_shims_mm=round(sq0, 3),
                F_N=round(F, 1), F_band_N=[round(Flo, 1), round(Fhi, 1)], F_band_untuned_print_pm0p1_N=[round(oring_force(sq - 0.1) + pogo, 1), round(oring_force(sq + 0.1) + pogo, 1)],
                hand3_still_to_g=round(F / (0.3964 * 9.81), 1), hand3_still_to_g_low=round(Flo / (0.3964 * 9.81), 1),
                collar_lock_torque_Nm=round(lock, 2), collar_hold_friction_Nm=round(F * rl * 1e-3 * mu, 3),
                free_ring_standoff_mm=round(c - depth_eff(), 2), ramp_capture_margin_mm=round(P["ramp"] - (c - depth_eff() - P["ax_clr"]), 2),
                ring_stretch_pct=round(100 * max(2 * P["og_r"][0] / P["oring_id"] - 1, 0), 1))


def detent_ball(theta=0.0):
    """Where the ball sits for a collar turn theta: pushed out as far as the collar bore lets it (seated in a dimple at 0 / -twist)."""
    return mf.Manifold.sphere(P["ball_d"] / 2, 24).translate([*at(P["ball_rc"], P["det_az"]), P["det_z"]])


def collar(theta=0.0):
    """Collar at turn theta (0 = locked, -twist = open).  A C-section ring: inner STEP (z_nose .. +t_step) that bears on
    the nose ring top, WALL (t_wall) down past the nose ring and the hand's lugs, and three LIP segments under the lugs.
    A short sleeve above the step carries on the socket body and holds the detent dimples.  Turning to locked carries
    each lip segment under a lug; its top face ramps up by `ramp` over the first ramp_deg."""
    z0, z1 = P["collar_z"]; zl = P["lip_z"][1]; zs = P["z_nose"]; zt = zs + P["t_step"]; cb = P["collar_cb"]
    body = cyl(P["R_collar"], z0, zt) + cyl(P["sleeve_r"], zt - 0.01, z1) - cyl(P["collar_bore"], zs - 0.01, z1 + 1) - cyl(cb, z0 - 1, zs)
    for k in range(12):
        x, y = at(P["R_collar"] + 1.4 - P["flute"], 30 * k); body -= cyl(1.4, z0 + 1, zt - 1, 16).translate([x, y, 0])
    for k in range(3):
        body += lip(P["lug_az"][k])
    for a in (P["det_az"], P["det_az"] + P["twist"]):                         # detent dimples: locked, open
        body -= mf.Manifold.sphere(P["ball_d"] / 2 + 0.05, 24).translate([*at(P["ball_rc"], a), P["det_z"]])
    return body.rotate([0, 0, float(theta)])


def female_bullets():
    out = []
    for az in P["az_bullet"]:
        x, y = at(P["r_bullet"], az)
        out.append((cyl(2.8, P["fem_recess"], P["fem_recess"] + P["fem_depth"], 32) - cyl(2.0, P["fem_recess"] - 1, P["fem_recess"] + 9.5, 32)).translate([x, y, 0]))
    return out


# ---------------------------------------------------------------- hand side (shared base)
def hand_base():
    fz0, fz1 = P["z_flange"]
    m = cyl(P["R_flange"], fz0, fz1) + cyl(P["r_spigot"], fz1 - 0.5, P["z_spigot_top"])
    for az in P["lug_az"]:
        m += lug(az)
    m += sector(P["r_spigot"] - 0.5, P["r_bore"] + P["key_h"], P["az_key"] - P["key_w"] / 2 / P["r_bore"] * 57.3, P["az_key"] + P["key_w"] / 2 / P["r_bore"] * 57.3, fz1 - 0.5, P["z_bore"] - 1.0)
    cut = cyl(P["r_pad"] + 0.15, P["z_pad"] - P["pad_t"], P["z_spigot_top"] + 1) + cyl(3.5, fz0 - 1, P["z_pad"])        # pad board well + wire bore
    cut += cyl(P["r_wire_cb"], fz0 - 1, fz0 + P["groove_h"])                                                    # wire turn pocket under the bore
    for az in P["az_bullet"]:
        x, y = at(P["r_bullet"], az); cut += cyl(2.05, fz0 - 1, fz1 + 0.01, 32).translate([x, y, 0])               # male bullet press bore
        g = P["groove_w"] / 2                                                                                    # motor-lead groove: bullet tail -> centre pocket
        cut += mf.Manifold.cube([P["r_bullet"] + 0.5, P["groove_w"], P["groove_h"] + 1]).translate([0, -g, fz0 - 1]).rotate([0, 0, float(az)])
    for az in P["hand_screw_az"]:                                                                                # 4 x M3 inserts for the hand
        x, y = at(P["hand_screw_r"], az); cut += cyl(2.0, fz0 - 0.01, fz0 + 6.0, 24).translate([x, y, 0])
    return m - cut


def male_bullets():
    out = []
    for az in P["az_bullet"]:
        x, y = at(P["r_bullet"], az)
        out.append(cyl(P["pin_d"] / 2, P["z_flange"][0] + 1.0, P["z_flange"][1] + P["fem_recess"] + P["pin_len"] - 0.5, 32).translate([x, y, 0]))
    return out


def place(m, dz=0.0):
    return m.translate([0, 0, -float(dz)])


# ---------------------------------------------------------------- checks
def vol(a, b):
    return float((a ^ b).volume())


def screws():
    """3 x M3 x 20 DIN 7984 (low head 5.5 x 2.0): head bottom on the counterbore floor, tip 6.2 into the tip bay's inserts."""
    out = []
    for r, az in P["screws"]:
        x, y = at(r, az)
        out.append((cyl(2.75, P["cbore_h"] - 2.0, P["cbore_h"], 24) + cyl(1.45, P["cbore_h"], P["cbore_h"] + 20.0, 16)).translate([x, y, 0]))
    return out


def stresses(M, Fr, sig_allow=30.0):
    """Hand formulas (no FEA).  The hand's bending moment M is carried as a couple by the lugs (1.5 effective) + one lug's
    share of the axial preload (rev B.3: O-ring + spring pins at the stiff end of the print band; was a flat 30 N).  One lug's force F then runs lug -> lip -> collar wall -> step -> nose ring -> screws.  The collar is a
    C-clamp: lug and nose-ring reactions both act inside the wall, so the wall sees tension + bending F * e.
    Load spreads in the wall over the lug arc + 0.78 sqrt(R t) each side (shell decay length), at most one 120 deg sector."""
    w = np.radians(P["lug_w"]); rc = (P["lip_r0"] + P["lug_r1"]) / 2; cb = P["collar_cb"]
    pre = preload()["F_band_N"][1]                    # rev B.3: O-ring + pins at the stiff end of the print band
    F = M / (rc * 1.5e-3) + pre / 3.0                 # one lug's share of the preload on top of the couple (was a flat 30 N)
    root = w * P["R_flange"]; arm = rc - P["R_flange"]
    tau = F / (root * P["t_lug"])
    f = P["lug_fillet"]; toe = w * (P["R_flange"] + f)               # B.4: root section (t + fillet deep) and the fillet toe (t deep)
    s_lug = max(6 * F * arm / (root * (P["t_lug"] + f) ** 2), 6 * F * max(rc - P["R_flange"] - f, 0.1) / (toe * P["t_lug"] ** 2))
    s_lip = 6 * F * (cb - rc) / (w * cb * P["t_lip"] ** 2)
    t = P["t_wall"]; Rm = cb + t / 2; b = min(w * Rm + 1.56 * np.sqrt(Rm * t), 2 * np.pi * Rm / 3)
    s_wall = F / (b * t) + 6 * F * (Rm - rc) / (b * t ** 2)
    rn = (P["collar_bore"] + P["R_nose"]) / 2
    s_step = 6 * F * (cb - rn) / (b * P["t_step"] ** 2)
    arc = (P["lug_w"] - P["relief_in"]) if P["lip_overhang"] else (P["lip_w"] - P["ramp_deg"] - 0.5 * P["lip_relief_deg"])
    bear_lug = F / (np.radians(arc) * rc * (P["lug_r1"] - P["lug_chamfer"] - P["lip_r0"] - P["lip_chamfer"] - 0.3))   # B.4: flat lip x flat lug, 0.3 radial float
    bear_sp = Fr / (2 * P["r_spigot"] * (P["z_bore"] - P["z_nose"]) * 0.5)
    step_face = F / (0.5 * np.pi * (P["R_nose"] ** 2 - P["collar_bore"] ** 2))
    F_scr = 0.0
    for phi in np.radians(np.arange(0, 360, 5)):
        d = np.array([P["R_nose"] + r * np.cos(np.radians(az) - phi) for r, az in P["screws"]])
        F_scr = max(F_scr, M * 1e3 * d.max() / (d ** 2).sum())
    F_scr += pre / len(P["screws"])
    worst = max(tau, s_lug, s_lip, s_wall, s_step)
    return dict(lug_force_N=round(F, 1), lug_shear_MPa=round(tau, 1), lug_bending_MPa=round(s_lug, 1), lip_bending_MPa=round(s_lip, 1),
                collar_wall_MPa=round(s_wall, 1), collar_step_MPa=round(s_step, 1), lug_on_lip_bearing_MPa=round(bear_lug, 1),
                spigot_bearing_MPa=round(bear_sp, 1), step_on_nose_ring_MPa=round(step_face, 2), screw_tension_N=round(F_scr, 1),
                factor_vs_PA_CF_interlayer_30MPa=round(sig_allow / worst, 2), factor_screw_vs_M3_insert_pullout_1000N_est=round(1000 / F_scr, 1))


def checks():
    S, Nr, Hb = arm_socket(), nose_ring(), hand_base()
    Fb = mf.Manifold.batch_boolean(female_bullets(), mf.OpType.Add); Mb = mf.Manifold.batch_boolean(male_bullets(), mf.OpType.Add)
    Sc = mf.Manifold.batch_boolean(screws(), mf.OpType.Add); C0 = collar(0)
    res = {}
    # locked: collar at 0, hand home (dz 0)
    res["locked: base x socket / nose ring / collar"] = [vol(Hb, S), vol(Hb, Nr), vol(Hb, C0)]
    res["locked: males x socket / nose ring"] = [vol(Mb, S), vol(Mb, Nr)]
    res["locked: males x female barrels (sliding fit, expected 0)"] = vol(Mb, Fb)
    res["locked: collar x socket / nose ring"] = [vol(C0, S), vol(C0, Nr)]
    res["nose ring x socket (faces touch, expected 0)"] = vol(Nr, S)
    res["screws x socket / nose ring / collar / female barrels / hand base"] = [vol(Sc, S), vol(Sc, Nr), vol(Sc, C0), vol(Sc, Fb), vol(Sc, Hb)]
    res["screw head below the mouth face (mm)"] = round(0.0 - (P["cbore_h"] - 2.0), 2)
    # assembly: the collar slides on from the hand end BEFORE the nose ring is fitted, up to the retaining flange
    worst = max(vol(C0.translate([0, 0, -float(d)]), S) for d in np.linspace(40, 0, 41))
    res["assembly: collar slid on from the mouth end over the bare socket: worst overlap (expected 0)"] = worst
    res["collar retained: pulled 0.2 mm toward the hand, overlap with the nose ring (expected > 0)"] = vol(C0.translate([0, 0, -0.2]), Nr)
    res["collar retained: pushed 0.4 mm into the arm, overlap with the retaining flange (expected > 0)"] = vol(C0.translate([0, 0, 0.4]), S)
    res["collar axial play (mm)"] = round(P["retain_z"][0] - P["collar_z"][1] + 0.0, 2)
    res["axial play of a locked hand (mm): lug between nose ring and lip"] = P["ax_clr"]
    res["collar step bearing on the nose ring: annulus (mm2)"] = round(np.pi * (P["R_nose"] ** 2 - P["collar_bore"] ** 2), 1)
    # collar sweep with the hand home: open -> locked
    worst, path, det = 0.0, [], []
    ball = detent_ball()
    for th in np.linspace(-P["twist"], 0, 25):
        Ct = collar(th)
        worst = max(worst, vol(Ct, S), vol(Ct, Nr))
        dz = next((d for d in np.arange(0.0, 1.21, 0.05) if vol(Ct, place(Hb, d)) < 1e-6), None)
        path.append((round(float(th), 1), None if dz is None else round(float(dz), 2)))
        # ball: how far the collar pushes it in (0 = seated in a dimple)
        push = next((d for d in np.arange(0.0, 1.21, 0.05) if vol(Ct, ball.translate([*at(-d, P["det_az"]), 0])) < 1e-4), None)
        det.append((round(float(th), 1), None if push is None else round(float(push), 2)))
    res["collar sweep open->locked: collar x socket/nose worst overlap"] = worst
    res["collar sweep: hand stand-off needed (deg, mm); ramp draws it in to 0"] = path
    res["detent: ball pushed in by the collar (deg, mm); 0 = seated in a dimple"] = det
    spring_room = P["ball_rc"] - P["ball_d"] / 2 - P["det_r_in"]
    res["detent: spring pocket length seated / pressed (mm)"] = [round(spring_room, 2), round(spring_room - max(v for _, v in det if v is not None), 2)]
    # straight insertion with the collar open: hand from 25 mm out to home
    worst = 0.0
    for dz in np.linspace(25, 0, 26):
        h = place(Hb, dz); worst = max(worst, vol(h, S), vol(h, Nr), vol(h, collar(-P["twist"])))
    res["push-in path (collar open): worst overlap"] = worst
    # wrong clocking: rotate the hand by 120 deg -> the key must hit the socket stack
    hw = place(Hb.rotate([0, 0, 120]), -2.0 + 9.4)
    res["wrong clocking blocked by the key (overlap at 2 mm in, expected > 0)"] = vol(hw, S) + vol(hw, Nr)
    # with the collar locked, the hand cannot come out: pull it 1 mm
    res["locked hand pulled 1 mm: lug x collar overlap (expected > 0)"] = vol(place(Hb, 1.0 + P["ramp"]), C0)
    # engagement depth of each bullet
    res["bullet engagement depth (mm)"] = round(P["z_flange"][1] + P["fem_recess"] + P["pin_len"] - 0.5 - P["fem_recess"], 2)
    # rev B.3 O-ring: seated in its groove, squeezed by the hand at lock (hand pushed out onto the lips: dz = ax_clr)
    Ro = oring()
    res["O-ring x nose ring (seated in the groove, expected 0)"] = vol(Ro, Nr)
    res["O-ring x socket / collar / male bullets / screws (expected 0)"] = [vol(Ro, S), vol(Ro, C0), vol(Ro, Mb), vol(Ro, Sc)]
    res["O-ring squeeze volume at lock, hand on the lips (expected > 0)"] = round(vol(Ro, place(Hb, P["ax_clr"])), 3)
    res["locked + preloaded (hand on the lips): base x collar / nose ring (expected 0)"] = [vol(place(Hb, P["ax_clr"]), C0), vol(place(Hb, P["ax_clr"]), Nr)]
    res["O-ring groove walls: to the socket bore / to the female-bullet mouth / to the screw counterbore (mm)"] = [
        round(P["og_r"][0] - P["r_bore"], 2), round(P["r_bullet"] - (P["pin_d"] / 2 + 0.6) - P["og_r"][1], 2), round(17.0 - P["cbore_d"] / 2 - P["og_r"][1], 2)]
    res["preload"] = preload()
    # stresses at the wheel-hand design cases
    cases = json.load(open("wheelhand_sizing.json"))["connector"] if os.path.exists("wheelhand_sizing.json") else {}
    for case, v in cases.items():
        res[f"stress, {case}"] = stresses(v["bending_Nm"], v["radial_N"])
    for k, v in res.items(): print(f"  {k:72s} {v}")
    json.dump(res, open("handmount_b_check.json", "w"), indent=1, default=float)
    return res


def to_trimesh(m):
    import trimesh
    me = m.to_mesh(); return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)


def export(d="handmount_b_stl"):
    os.makedirs(d, exist_ok=True)
    for n, m in (("arm_socket_PA-CF", arm_socket()), ("nose_ring_PA-CF", nose_ring()), ("collar_PA-CF", collar(0)), ("hand_base_PA-CF", hand_base())):
        t = to_trimesh(m); t.export(f"{d}/{n}.stl")
        print(f"  {n:18s} watertight {t.is_watertight} bodies {len(m.decompose())} vol {m.volume()/1000:.2f} cm3 mass {m.volume()/1000*1.15:.1f} g bbox {np.round(t.extents, 1)}")


if __name__ == "__main__":
    checks(); export()
