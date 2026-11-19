"""Drives for the Scabbard rotor (Cephalo-2, 2026-11-15): the parts scabbard-dock.md lists in its BOM but never drew, laid
out against the real solids and checked for overlaps.  Import this BEFORE turretcams / cephalo2_model: it patches
scabbard.py (turret r 92, stow lift 185) and wraps plate_solid / elevator_solid / fold_ring with the drive features.

Frame: scabbard.py body frame, mm, +z up, origin at the plate underside centre.

FOLD DRIVE (push-rod ring on three T8x2 lead screws, one STS3215):
  * three T8x2 screws at r 95 (az 70, 190, 310), z 6..92, 20T GT2 pulleys at z 73..79, brass nuts in bosses on the fold ring
  * a GT2 belt loop at z 76 runs round the three screw pulleys and three 20T idlers at r 95 (az 10, 130, 250) and the
    servo pulley: every side stays >= 82 mm from the axis, outside the scabbards (r 79) and the push rods (r 84.5); it is
    above the fold ring's highest position (z 70.5) and the guest sockets (z 82) are inboard of it (r <= 77)
  * fold servo STS3215 in the guest sector at az 315, standing on its 45 x 25 face with the output shaft DOWN, pulley
    hanging at z 73..79; a seventh belt vertex at an idler (az 340) keeps the belt off the scabbard at az 0
ELEVATOR DRIVE (blade ring on two GT2 loops, one STS3215):
  * two vertical GT2 loops at r 50, az 66 and 204 (both on one line parallel to the 45/225 diagonal, 18 mm from it, which
    clears the zenith camera tray): 12T pulleys at the top (z 206.5, on one 4 mm shaft) and at the bottom (z 2.5, pocketed
    into the plate).  The ring is clamped to one run and slotted for the return run.
  * stow lift 194 -> 185 mm so the ring (top z 201) passes under the shaft and pulleys; the stowed tip sits 9 mm inside
    the barrel instead of 2; dock stroke 108 -> 99 (U_DOCK unchanged).
  * the shaft runs from a bearing block under the cage top ring (az 66) past two hangers on the zenith bridge to a 1:1
    miter pair at az 209, r 62; a vertical jackshaft takes it down to a 20T pulley at z 116..122, belted to the elevator
    servo (STS3215, output shaft UP) in the guest sector at az 225.
Servo footprints are placed by search (shapely) for the largest clearance to the scabbards, the ring, the wall.
"""
import numpy as np
import manifold3d as m3
import scabbard as S
from scabbard import bx, zcyl, xcyl, ycyl, union, rz, hull

# ---- design changes carried by this layout
S.TURRET_R = 92.0
S.U_STOW = 185.0
S.STROKE_DOCK = S.U_STOW - S.U_DOCK
import turretcams as TC
TC.R_IN = S.TURRET_R - TC.SHELL_T; TC.R_DISC = 88.6; TC.R_LENS = 86.1

SERVO = (45.2, 24.7, 35.0)                       # STS3215 L x W x H (H along the output axis)
R_WALL = S.TURRET_R - 1.6                        # 90.4 above z 110
Z_FOLD_BELT = 76.0; R_SCREW = 95.0; R_IDLER = 99.5; P16 = 5.09; AZ_SCREW = (70.0, 190.0, 310.0); AZ_IDLER = (10.0, 130.0, 250.0, 340.0)
P20, P12 = 6.37, 3.82                            # GT2 pulley pitch radii: 20T, 12T
Z_SHAFT = 206.5; R_LOOP = 50.0; OFF = 18.0        # elevator shaft height, loop radius, offset of the shaft line from the 45/225 diagonal
AZ_LOOP = tuple(np.degrees(np.arctan2(*reversed(p))) % 360 for p in
                [(R_LOOP * np.cos(np.radians(135 + s * np.degrees(np.arccos(OFF / R_LOOP)))), R_LOOP * np.sin(np.radians(135 + s * np.degrees(np.arccos(OFF / R_LOOP))))) for s in (-1, 1)])
Z_BOT = 2.5
E_SERVO = dict(az=225.0, z0=90.0, up=True); F_SERVO = dict(az=315.0, z0=90.0, up=False)
F_JACK_R, F_JACK_AZ, Z_FOLD_TOP = 86.0, 322.0, 86.5     # fold jackshaft: lower pulley on the belt (z 76), upper pulley at z 86.5 under the hanging servo pulley
Z_JACK_PULLEY = 90.0 + 35.0 + 4.0
JACK = None                                       # (x, y) of the jackshaft, set in layout()


def pol(r, az):
    a = np.radians(az); return np.array([r * np.cos(a), r * np.sin(a)])


def line_point(off, along):
    """Point on the elevator shaft line: perpendicular offset OFF toward az 135 from the 45/225 diagonal, `along` from the centre toward az 45."""
    d = pol(1, 45); n = pol(1, 135)
    return n * off + d * along


# ---------------------------------------------------------------- footprint search for the two servos
def sector_free(az, z0, z1, obstacles=()):
    """Shapely polygon of the free floor in a guest sector between z0 and z1 (wall at the smallest radius over that height)."""
    from shapely.geometry import Polygon, Point, box as sbox
    from shapely import affinity
    zs = np.linspace(z0, z1, 9)
    rw = min((R_WALL if z > 110 else min(R_WALL, 125 * np.sqrt(max(0, 1 - ((z - 6) / 150) ** 2)) - 1.6)) for z in zs)
    disc = Point(0, 0).buffer(rw, 256).difference(Point(0, 0).buffer(53.0 + 0.5, 256))      # wall .. elevator ring
    w, t = S.OWN
    for k in range(4):
        sc = sbox(S.R_S - t / 2 - S.WALL, -w / 2 - S.WALL, S.R_S + t / 2 + S.WALL, w / 2 + S.WALL)
        disc = disc.difference(affinity.rotate(sc, 90 * k, origin=(0, 0)))
    for (ox, oy, orad) in obstacles:
        disc = disc.difference(Point(ox, oy).buffer(orad, 32))
    return disc


def fit_servo(az, z0, z1, anchor=None, obstacles=()):
    """Best (centre xy, yaw deg) for a SERVO footprint in the sector at az: maximises the clearance to the free-polygon edge;
    with an anchor, the servo's output shaft must lie 18..34 mm from it (a short GT2 loop)."""
    from shapely.geometry import Polygon
    from shapely import affinity
    free = sector_free(az, z0, z1, obstacles)
    L, Wd = SERVO[0], SERVO[1]
    best = (-1e9, None)
    for rc in np.arange(60.0, 84.1, 1.0):
        for tc in np.arange(-20.0, 20.1, 1.0):
            c = pol(rc, az) + pol(1, az + 90) * tc
            for yaw in np.arange(0, 180, 5):
                rect = affinity.rotate(Polygon([(-L / 2, -Wd / 2), (L / 2, -Wd / 2), (L / 2, Wd / 2), (-L / 2, Wd / 2)]), az + 90 + yaw, origin=(0, 0))
                rect = affinity.translate(rect, c[0], c[1])
                if not free.contains(rect): continue
                a_ = np.radians(az + 90 + yaw); sxy = c + np.array([np.cos(a_), np.sin(a_)]) * (L / 2 - 11.4)
                if anchor is not None:
                    dd = np.linalg.norm(sxy - anchor)
                    if not (18.0 <= dd <= 34.0): continue
                d = free.exterior.distance(rect)
                for ring in free.interiors: d = min(d, ring.distance(rect))
                if d > best[0]: best = (d, (c, yaw))
    return best


_FIT = {}


def servo_pose(which):
    if which not in _FIT:
        p = E_SERVO if which == "elev" else F_SERVO
        anchor = jack_xy() if which == "elev" else pol(F_JACK_R, F_JACK_AZ)
        obst = ()
        d, (c, yaw) = fit_servo(p["az"], p["z0"], p["z0"] + SERVO[2], anchor, obst)
        _FIT[which] = dict(c=c, yaw=yaw, clear=d, az=p["az"], z0=p["z0"], up=p["up"])
    return _FIT[which]


def jack_xy():
    d = pol(1, 45)
    along = [float(np.dot(pol(R_LOOP, az), d)) for az in AZ_LOOP]
    a_min = min(along); a_j = -np.sqrt(62.0 ** 2 - OFF ** 2)
    return line_point(OFF, min(a_min, a_j))


def servo_solid(sp):
    """STS3215 box + output boss, standing on its L x W face from z0, output axis up or down; shaft 11.4 mm from one end."""
    L, Wd, Hh = SERVO
    body = bx(-L / 2, L / 2, -Wd / 2, Wd / 2, 0, Hh)
    xs = L / 2 - 11.4
    if sp["up"]:
        body = body + zcyl(5.5, Hh, Hh + 3.0, 24, xs, 0)
    else:
        body = body.translate([0, 0, 0]); body = body + zcyl(5.5, -3.0, 0.0, 24, xs, 0)
    body = body.rotate([0, 0, sp["az"] + 90 + sp["yaw"]]).translate([sp["c"][0], sp["c"][1], sp["z0"]])
    return body


def servo_shaft_xy(sp):
    L = SERVO[0]; xs = L / 2 - 11.4
    a = np.radians(sp["az"] + 90 + sp["yaw"])
    return sp["c"] + np.array([np.cos(a), np.sin(a)]) * xs


# ---------------------------------------------------------------- fold drive solids
def belt_loop_xy(points, r_p, z, w=6.0, t=1.4):
    """Flat GT2 belt (approximate: straight runs between pulley centres, offset outward by the pitch radius) at height z."""
    from shapely.geometry import Polygon, LineString
    P = np.array(points)
    poly = (Polygon(P) if len(P) > 2 else LineString(P)).buffer(r_p, 32)
    outer = np.array(poly.exterior.coords); inner = np.array(poly.buffer(-t, 32).exterior.coords)
    cs = m3.CrossSection([outer.tolist()]) - m3.CrossSection([inner.tolist()])
    return m3.Manifold.extrude(cs, w).translate([0, 0, z - w / 2])


def fold_drive():
    out = {}
    scr, pul, posts, idl = [], [], [], []
    for az in AZ_SCREW:
        x, y = pol(R_SCREW, az)
        scr.append(zcyl(4.0, S.PLATE_T, 92.0, 20, x, y)); pul.append(zcyl(P20 + 0.8, Z_FOLD_BELT - 3.5, Z_FOLD_BELT + 3.5, 24, x, y))
    for az in AZ_IDLER:
        x, y = pol(R_IDLER, az)
        posts.append(zcyl(3.0, S.PLATE_T, Z_FOLD_BELT + 5.0, 16, x, y)); idl.append(zcyl(P16 + 0.8, Z_FOLD_BELT - 3.5, Z_FOLD_BELT + 3.5, 24, x, y))
    sp = servo_pose("fold"); sxy = servo_shaft_xy(sp); fj = pol(F_JACK_R, F_JACK_AZ)
    out["fold_servo"] = servo_solid(sp)
    out["fold_servo_pulley"] = zcyl(P20 + 0.8, Z_FOLD_TOP - 3.0, Z_FOLD_TOP + 3.0, 24, sxy[0], sxy[1])
    out["fold_jackshaft"] = zcyl(2.0, Z_FOLD_BELT - 3.5, Z_FOLD_TOP + 3.0, 16, fj[0], fj[1])
    out["fold_jack_pulleys"] = zcyl(P20 + 0.8, Z_FOLD_BELT - 3.5, Z_FOLD_BELT + 3.5, 24, fj[0], fj[1]) + zcyl(P20 + 0.8, Z_FOLD_TOP - 3.0, Z_FOLD_TOP + 3.0, 24, fj[0], fj[1])
    out["fold_servo_belt"] = belt_loop_xy([fj, sxy], P20, Z_FOLD_TOP)
    # jackshaft bracket: a tab down the guest socket's outer wall and an arm between the two pulleys with a bearing boss
    tab = bx(S.GUEST_R[1] + 2.0, S.GUEST_R[1] + 4.0, -6.0, 6.0, 55.0, Z_FOLD_BELT + 3.5 + 0.5).rotate([0, 0, F_SERVO["az"]])
    arm = hull(bx(S.GUEST_R[1] + 2.0, S.GUEST_R[1] + 4.0, -6.0, 6.0, Z_FOLD_BELT + 4.0, Z_FOLD_TOP - 3.5).rotate([0, 0, F_SERVO["az"]]),
               zcyl(5.0, Z_FOLD_BELT + 4.0, Z_FOLD_TOP - 3.5, 20, fj[0], fj[1]))
    out["fold_jack_bracket"] = (tab + arm) - zcyl(2.6, 50, 100, 16, fj[0], fj[1])
    verts = sorted([(az, pol(R_SCREW, az)) for az in AZ_SCREW] + [(az, pol(R_IDLER, az)) for az in AZ_IDLER] + [(F_JACK_AZ, fj)], key=lambda v: v[0])
    out["fold_belt"] = belt_loop_xy([v[1] for v in verts], P20, Z_FOLD_BELT)
    out["lead_screws"] = union(scr); out["screw_pulleys"] = union(pul); out["idler_posts"] = union(posts); out["idlers"] = union(idl)
    return out


def nut_bosses(zt):
    """Brass T8 nuts in printed bosses on the fold ring (ring z zt+60 .. zt+66)."""
    out = []
    for az in AZ_SCREW:
        x, y = pol(R_SCREW, az)
        out.append(zcyl(10.0, zt + 60.0, zt + 66.0, 24, x, y) - zcyl(4.3, zt + 59, zt + 67, 20, x, y))
    return union(out)


# ---------------------------------------------------------------- elevator drive solids
def elevator_drive():
    global JACK
    out = {}
    d = pol(1, 45); n = pol(1, 135)
    along = [float(np.dot(pol(R_LOOP, az), d)) for az in AZ_LOOP]          # positions of the two loop pulleys along the shaft line
    a_min, a_max = min(along), max(along)
    # shaft: from a bearing block at the az-66 end (a_max side) to the miter at r 62 on the a_min side
    jack = jack_xy(); a_j = float(np.dot(jack, d)); JACK = jack
    p0 = line_point(OFF, a_max + 12.0); p1 = line_point(OFF, a_j)
    def seg(p, q, r, nseg=16):
        L = np.linalg.norm(q - p); ang = np.degrees(np.arctan2(q[1] - p[1], q[0] - p[0]))
        return xcyl(r, 0, L, 0, 0, nseg).rotate([0, 0, ang]).translate([p[0], p[1], Z_SHAFT])
    out["elev_shaft"] = seg(p0, p1, 2.0)
    # pulleys on the shaft (axis along the shaft) + bottom pulleys, belts, ring clamps
    pul = []; belts = []; bot = []; clamps = []
    for az, a in zip(AZ_LOOP, along):
        c = line_point(OFF, a)
        pul.append(xcyl(P12, -3.0, 3.0, 0, 0, 20).rotate([0, 0, 45]).translate([c[0], c[1], Z_SHAFT]))
        bot.append(xcyl(P12, -3.0, 3.0, 0, 0, 20).rotate([0, 0, 45]).translate([c[0], c[1], Z_BOT]))
        for s in (-1, 1):                                                        # two runs, offset +-P12 along the 135 direction
            q = c + n * s * P12
            belts.append(bx(-3.0, 3.0, -0.7, 0.7, Z_BOT, Z_SHAFT).rotate([0, 0, 45]).translate([q[0], q[1], 0]))
    out["elev_pulleys"] = union(pul); out["elev_bottom_pulleys"] = union(bot); out["elev_belts"] = union(belts)
    # bearing block under the cage top ring at the a_max end; two hangers from the zenith bridge
    pb = line_point(OFF, a_max + 7.5)
    out["elev_bearing_block"] = (bx(-3, 3, -5, 5, Z_SHAFT - 4.5, S.Z_CAGE_TOP).rotate([0, 0, 45]).translate([pb[0], pb[1], 0])
                                 + bx(-3, 12, -5, 5, S.Z_CAGE_TOP - 2.0, S.Z_CAGE_TOP).rotate([0, 0, 45]).translate([pb[0], pb[1], 0])) - out["elev_shaft"]
    hang = []
    for a in (-28.0, 28.0):
        ph = line_point(OFF, a)
        arm = bx(-2.5, 2.5, -(OFF - 6.0) - 2.0, 0, Z_SHAFT - 4.0, S.Z_CAGE_TOP + 3.2).rotate([0, 0, 45]).translate([ph[0], ph[1], 0])
        hang.append(arm - out["elev_shaft"])
    out["elev_hangers"] = union(hang)
    # miter pair + jackshaft + its pulley
    g_r = 4.5
    out["miter_shaft"] = m3.Manifold.cylinder(4.0, g_r, g_r * 0.55, 24).rotate([0, 90, 0]).rotate([0, 0, 225]).translate([p1[0], p1[1], Z_SHAFT]).translate([-d[0] * 2, -d[1] * 2, 0])
    out["miter_jack"] = m3.Manifold.cylinder(4.0, g_r * 0.55, g_r, 24).translate([jack[0], jack[1], Z_SHAFT - g_r - 2.0])
    out["jackshaft"] = zcyl(2.0, Z_JACK_PULLEY - 3.0, Z_SHAFT - g_r - 2.0, 16, jack[0], jack[1])
    out["jack_pulley"] = zcyl(P20 + 0.8, Z_JACK_PULLEY - 3.0, Z_JACK_PULLEY + 3.0, 24, jack[0], jack[1])
    sp = servo_pose("elev"); sxy = servo_shaft_xy(sp)
    out["elev_servo"] = servo_solid(sp)
    out["elev_servo_pulley"] = zcyl(P20 + 0.8, Z_JACK_PULLEY - 3.0, Z_JACK_PULLEY + 3.0, 24, sxy[0], sxy[1])
    out["elev_servo_belt"] = belt_loop_xy([jack, sxy], P20, Z_JACK_PULLEY)
    return out


def ring_mods(u):
    """Slots for the return runs and clamps on the drive runs, on the elevator ring at lift u."""
    zh = S.Z_H - S.X_HEAD + u; n = pol(1, 135)
    cut, add = [], []
    for az in AZ_LOOP:
        a = float(np.dot(pol(R_LOOP, az), pol(1, 45))); c = line_point(OFF, a)
        for s_ in (1, -1):                                                       # slots through the ring for both runs
            q = c + n * s_ * P12
            cut.append(bx(-4.5, 4.5, -2.2, 2.2, zh + 9, zh + 19).rotate([0, 0, 45]).translate([q[0], q[1], 0]))
        q2 = c - n * P12                                                         # drive run: clamp block over its slot (belt channel 1.6 x 6.2, screwed shut)
        add.append(bx(-5.5, 5.5, -3.6, 3.6, zh + 10, zh + 18).rotate([0, 0, 45]).translate([q2[0], q2[1], 0]) - bx(-3.1, 3.1, -0.8, 0.8, zh + 9, zh + 19).rotate([0, 0, 45]).translate([q2[0], q2[1], 0]))
    return union(cut), union(add)


# ---------------------------------------------------------------- patches into scabbard.py
_plate0, _elev0, _fold0, _cage0 = S.plate_solid, S.elevator_solid, S.fold_ring, S.cage_solid


def cage_solid():
    c = _cage0()
    for az in AZ_SCREW:
        x, y = pol(R_SCREW, az); c = c - zcyl(4.3, 0, 20, 20, x, y)
    for az in AZ_IDLER:
        x, y = pol(R_IDLER, az); c = c - zcyl(3.3, 0, 20, 16, x, y)
    for az in AZ_LOOP:
        a = float(np.dot(pol(R_LOOP, az), pol(1, 45))); q = line_point(OFF, a)
        c = c - bx(-7.0, 7.0, -5.0, 5.0, 0, 20).rotate([0, 0, 45]).translate([q[0], q[1], 0])
    return c


def plate_solid():
    p = _plate0()
    for az in AZ_LOOP:
        a = float(np.dot(pol(R_LOOP, az), pol(1, 45))); c = line_point(OFF, a)
        p = p - bx(-7.0, 7.0, -5.0, 5.0, -1, S.PLATE_T + 1).rotate([0, 0, 45]).translate([c[0], c[1], 0])
    return p


def elevator_solid(u):
    cut, add = ring_mods(u)
    return _elev0(u) - cut + add


def fold_ring(zt):
    m = (_fold0(zt) - (zcyl(200, zt + 50, zt + 80) - zcyl(96.0, zt + 49, zt + 81))) + nut_bosses(zt)     # ring r 84..96: 0.5 mm off the idler posts
    for az in AZ_SCREW:
        x, y = pol(R_SCREW, az); m = m - zcyl(4.3, zt + 59, zt + 71, 20, x, y)
    return m


S.plate_solid, S.elevator_solid, S.fold_ring, S.cage_solid = plate_solid, elevator_solid, fold_ring, cage_solid


def drive_parts():
    out = fold_drive(); out.update(elevator_drive()); return out


# ---------------------------------------------------------------- checks
def overlaps(states, verbose=True):
    """Boolean intersections (mm^3) of every drive part against the static turret parts and against the moving parts at
    each state (dicts with beta, u, rods, elev, psi); returns the worst per pair."""
    import cephalo2_model as M
    D = drive_parts()
    statics = {"cage": S.cage_solid(), "plate": S.plate_solid(), "shell": TC.shell_with_windows(), "motor": S.motor_solid(), "pins": S.four(S.latch_pin(0.0), 45.0)}
    A = TC.assembly()
    statics["cameras"] = union([v for k, v in A.items()])
    worst = {}
    def hit(a, b, name):
        v = (a ^ b).volume()
        if v > 0.5:
            worst[name] = max(worst.get(name, 0), v)
            if verbose: print(f"  OVERLAP {name}: {v:.1f} mm^3")
    names = list(D)
    for i, k in enumerate(names):
        for s, sm in statics.items(): hit(D[k], sm, f"{k} x {s}")
        for k2 in names[i + 1:]: hit(D[k], D[k2], f"{k} x {k2}")
    for st in states:
        rot = S.rotating(st["beta"], st["u"], psi=st["psi"]); rods, zt = S.rods_solid(st["rods"])
        mov = {"hub": rot["hub"], "barrels": union([v for k, v in rot.items() if k.startswith("barrel")]), "blades": union([v for k, v in rot.items() if k.startswith("blade")]),
               "rods+ring": rods + S.fold_ring(zt), "elevator": S.elevator_solid(st["elev"])}
        tag = f"u{st['u']:.0f} b{st['beta']:.0f} e{st['elev']:.0f}"
        for k in names:
            for mk, mm in mov.items(): hit(D[k], mm, f"{k} x {mk} @ {tag}")
        for mk in ("rods+ring", "elevator"):
            for s in ("cage", "plate", "cameras"): hit(mov[mk], statics[s], f"{mk} x {s} @ {tag}")
        hit(mov["elevator"], mov["blades"], f"elevator x blades @ {tag}")
    return worst


if __name__ == "__main__":
    for w in ("fold", "elev"):
        sp = servo_pose(w); print(w, "servo: centre r %.1f az %.1f, yaw %.0f, clearance %.1f mm" % (np.hypot(*sp["c"]), np.degrees(np.arctan2(sp["c"][1], sp["c"][0])) % 360, sp["yaw"], sp["clear"]))
    print("loops at az", [round(a, 1) for a in AZ_LOOP], "shaft z", Z_SHAFT, "stow lift", S.U_STOW, "dock stroke", S.STROKE_DOCK)
    states = [dict(beta=S.BETA_H, u=u, rods=S.BETA_H, elev=u, psi=0.0) for u in (S.U_STOW, 150, 100, 50, 0)]
    states += [dict(beta=S.BETA_H, u=0, rods=S.BETA_H, elev=14.0, psi=6.5)]
    states += [dict(beta=b, u=0, rods=b, elev=14.0, psi=6.5) for b in (75, 60, 40, 20, S.BETA_F)]
    states += [dict(beta=S.BETA_F, u=0, rods=None, elev=14.0, psi=p) for p in (6.5, 30, 60)]
    w = overlaps(states)
    print("overlapping pairs:", len(w))
    import json; json.dump({k: round(v, 1) for k, v in w.items()}, open("scabbard_drive_check.json", "w"), indent=1)
