#!/usr/bin/env python3
"""
root_gear.py  --  geared root socket for the Cephalo-1 main arm (joint 0 driven through a 2:1 spur stage)

The root fix (fix_root.py, Nov 9) puts a 2:1 m1 spur stage on joint 0: a 20T pinion on the STS3250
horn drives a 40T gear that turns the joint-0 pin. This script redraws the root socket around it and
reruns pin_cad.py's fit, collision and mass checks with the new socket in place. Every moving
vertebra, the tip and the fork that bolts to joint 0 stay exactly as pin_cad.py draws them: the fork's
horn ear now bolts to the 40T gear's hub, on the same 4 x M3 / 14 mm bolt circle the servo horn had.

Joint-0 local frame (as pin_cad.py): alpha along the hinge axis, beta across, zeta along the arm.
  40T gear      alpha  7.5 .. 19.5, on the hinge axis; hub r 10.5 to alpha 22.5 (fork horn ear bolts here)
                shaft r 5 to alpha 2.1 in a 6800 bearing (10 x 19 x 5) held in the socket's gear web
  20T pinion    alpha  7.5 .. 19.5, centre zeta -30 (30 mm centre distance); bolted to the servo horn;
                stub r 2.5 runs out to a 685 bearing (5 x 11 x 5) in a screw-on bearing cap
  STS3250       moved 15 mm along -alpha and 30 mm along -zeta: case alpha -34.5 .. 0.5, zeta -64.2 .. -19
                slides into a closed sleeve from the proximal end; an end cap holds it (no lid at joint 0)
  idler side    stub axle for the fork's idler ear on a cheek plate at alpha -22.4 .. -19.4, as before

  python3 root_gear.py --length 1000 --joints 13 --root-servos 5 --out out_1m_13
Requires numpy, manifold3d, trimesh. Units mm.
"""
import math, os, sys, json
import numpy as np
from manifold3d import Manifold, CrossSection
import trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pin_cad as C
from pin_cad import box, cyl_x, cyl_z, joint_affine, to_trimesh

# ---------------------------------------------------------------- gear stage
MOD = 1.0; Z_PIN, Z_GEAR = 20, 40; PA = 20.0
FACE = float(os.environ.get("GEAR_FACE", "10"))   # 10 mm keeps the stage inside the 46 g budget
GEAR_STYLE = os.environ.get("GEAR_STYLE", "sector")   # sector | webbed
SECTOR_DEG = 65.0                                  # teeth over +-65 deg about the mesh point at q = 0
STOP_DEG = 55.0                                    # joint-0 hard stops (command range is +-45)
CDIST = MOD * (Z_PIN + Z_GEAR) / 2                 # 30 mm
G_A1 = 19.5; G_A0 = G_A1 - FACE                    # gear and pinion face along alpha (7.5 .. 19.5 at 12 mm)
HUB_R, HUB_A1 = 10.5, 22.5                         # gear hub up to the fork horn ear
SHAFT_R = 5.0; SHAFT_A0 = G_A0 - 5.4               # gear shaft into the 6800 (2.1 at a 12 mm face)
B6800 = dict(name="6800", d=10.0, D=19.0, B=5.0, g=5.5, C=1720, C0=830)      # SKF 61800 catalogue: C 1.72 kN, C0 0.83 kN
B685 = dict(name="685", d=5.0, D=11.0, B=5.0, g=1.8, C=681, C0=280)         # 685: C 69.4 kgf, C0 28.5 kgf; mass [A]
B61803 = dict(name="61803", d=17.0, D=26.0, B=5.0, g=7.5, C=2030, C0=1270)  # SKF 61803 catalogue: C 2.03 kN, C0 1.27 kN, 7.5 g
B695 = dict(name="695", d=5.0, D=13.0, B=4.0, g=2.2, C=1025, C0=410)        # 695: C 104.5 kgf, C0 41.8 kgf; mass [A]
# Root bearing check (Nov 9): the 6800 under the gear shaft and the printed idler stub fall short when the claw fan pushes
# across the hinge plane. ROOT_BRG=61803 (default) puts a 61803 in a recess in the gear's inboard face, centred under the
# mesh, swaps the printed idler stub for a 7075 pin and the 685 for a 695. ROOT_BRG=6800 draws the socket as first drawn.
ROOT_BRG = os.environ.get("ROOT_BRG", "61803")
NEW = ROOT_BRG != "6800"
GB = B61803 if NEW else B6800                      # gear-side bearing
PB = B695 if NEW else B685                         # pinion bearing
BRG_A0, BRG_A1 = G_A0, G_A0 + 5.0                  # 61803 along alpha (9.5 .. 14.5), centre 12.0 against the mesh at 14.5
REC_R1 = 15.6                                      # recess in the gear's inboard face, r 8.5 .. 15.6 (rim 3.15 mm under the 18.75 mm root)
SHOULDER_R = 9.8                                   # gear shoulder for the inner race, to BRG_A1 + 0.5
BOSS_R0, BOSS_R1 = GB["D"] / 2, GB["D"] / 2 + 2.0  # socket boss ring round the outer race (r 13 .. 15)
LIP_R = 11.5                                       # outer-race lip on the web; also the screw access hole
PIN_A1 = -16.4                                     # idler pin: from the idler ear through the cheek and a 3 mm boss
AL_RHO = 2.81e-3                                    # 7075-T6, g/mm^3
D_ALPHA, D_ZETA = (19.5 - FACE) - 22.5, -CDIST   # -15 at a 12 mm face                    # servo move from its pin_cad.py place
GEAR_CLR = 0.5
WEB_A1 = G_A0 - 0.5; WEB_A0 = WEB_A1 - 3.9          # gear web (6800 seat) along alpha (3.1 .. 7.0 at 12 mm)
CHEEK_A0, CHEEK_A1 = -22.4, -19.4
BR_A0, BR_A1 = 20.5, 24.5                          # pinion bearing bracket (part of the socket)
FLANGE_HOLE_DEG = (45, 90, 270, 315)               # ring holes; 30/330 sat 0.35 mm from the mantle opening (mantle_notch.py, Nov 9)
CAP_A1 = 27.0                                      # bearing cap top
WALL_TOP = 21.8
CAP_SCREWS = [(-10.0, 10.5), (10.0, 10.5)]         # (beta, zeta from the pinion axis): gear side, clear of the flange
HW_EXTRA_G = 6.0 if os.environ.get("ROOT_BRG", "61803") == "6800" else 5.4
                   # [A] 5.4 g with the 61803: no gear retaining screw and washer, no heat-set insert in the idler stub
                   # (the idler screw threads into the 7075 pin)
                   # [A] 4 M3 pinion screws, 2 cap + 2 end-cap screws, 4 inserts, gear retaining screw
                   #     and washer, less the 4 lid screws and inserts joint 0 no longer needs

def involute_gear(z, m=MOD, pa=PA, add=1.0, ded=1.25, npts=8):
    """2-D outline of a standard external spur gear, centred at the origin"""
    rp = m * z / 2; rb = rp * math.cos(math.radians(pa)); ra = rp + add * m; rf = rp - ded * m
    inv = lambda a: math.tan(a) - a
    # half tooth angle at the base circle
    tb = math.pi / (2 * z) + inv(math.radians(pa))
    ts = np.linspace(0, math.sqrt((ra / rb) ** 2 - 1), npts)
    flank = [(rb * math.sqrt(1 + t * t), t - math.atan(t)) for t in ts]      # (r, involute angle)
    pts = []
    for k in range(z):
        c = 2 * math.pi * k / z
        # root arc from previous tooth, then the two flanks
        r0 = max(rf, 0.0)
        left = [(r, c - tb + a) for (r, a) in flank]
        right = [(r, c + tb - a) for (r, a) in reversed(flank)]
        if rf < rb:
            left = [(rf, c - tb)] + left; right = right + [(rf, c + tb)]
        for (r, a) in left + right:
            pts.append((r * math.cos(a), r * math.sin(a)))
    return pts

def gear_extrude(z, a0, a1, rot=0.0):
    """gear profile extruded along alpha (x in the joint frame), axis at the origin of beta-zeta"""
    cs = CrossSection([involute_gear(z)]).rotate(rot)
    m = Manifold.extrude(cs, a1 - a0)                 # along +z in its own frame, profile in x-y
    # map (x, y, z) -> (alpha = z + a0, beta = x, zeta = y)
    M = np.array([[0, 0, 1, a0], [1, 0, 0, 0], [0, 1, 0, 0]], float)
    return m.transform(M)


class GearedRootArm(C.PinArm):
    def __init__(s, *a, **k):
        super().__init__(*a, **k)
        sv = C.SERVO
        s.dA, s.dZ = D_ALPHA, D_ZETA
        s.hw = s.Wp / 2 + C.WALL
        # moved servo, joint-0 frame
        s.sc_a0, s.sc_a1 = s.case_lo + s.dA, s.case_hi + s.dA              # -34.5 .. 0.5
        s.sp_a0, s.sp_a1 = s.p_lo + s.dA, s.p_hi + s.dA                    # pocket
        s.sfloor = s.sp_a0 - C.FLOOR_T
        s.stop = s.sp_a1 + 2.0                                             # sleeve top ledge
        s.sz0, s.sz1 = s.zeta_lo + s.dZ, s.zeta_hi + s.dZ                  # pocket along zeta (-64.7 .. -18.5)
        s.end_z0, s.end_z1 = s.sz1, s.sz1 + 1.8                            # distal stop wall
        s.horn_a0, s.horn_a1 = s.horn_lo + s.dA, s.horn_hi + s.dA          # 4.5 .. 7.5
        assert abs(s.horn_a1 - G_A0) < 1e-6, (s.horn_a1, G_A0)
        s.flange_z0, s.flange_z1 = s.cr_lo - 3.0, s.cr_lo                  # mantle face, as before
        s.cap_z0, s.cap_z1 = s.sz0 - (2.0 if NEW else 3.0), s.sz0          # proximal end cap (2 mm with the bearing-check trim)
        s.pz = -CDIST                                                      # pinion axis zeta

    # ---------------------------------------------------------- printed: socket (one part)
    def socket_local(s):
        hw, Wp = s.hw, s.Wp
        R = s.R
        # sleeve: floor, two walls, top ledges with a slot for the spline, distal stop wall
        sl = box(s.sfloor, s.stop, -hw, hw, s.sz0, s.end_z1)
        sl = sl - box(s.sp_a0, s.sp_a1, -Wp / 2, Wp / 2, s.sz0 - 1, s.sz1)
        sl = sl - box(s.sp_a1 - 0.01, s.stop + 1, -6.2, 6.2, s.sz0 - 1, s.pz)          # spline slot
        sl = sl - cyl_x(6.2, s.sp_a1 - 0.01, s.stop + 1, 0, s.pz)
        # lightening windows in the sleeve walls and floor
        wz0, wz1 = s.sz0 + 7, s.sz1 - 6
        wa0, wa1 = s.sfloor + 4, s.sp_a1 - 4
        sl = sl - box(wa0, wa1, -hw - 1, hw + 1, wz0, wz1)
        sl = sl - box(s.sfloor - 1, s.sp_a0 + 0.01, -Wp / 2 + 3.5, Wp / 2 - 3.5, s.sz0 + 6, s.sz1 - 5)
        # end-cap bosses (M3 inserts along zeta) on the outside of the sleeve walls
        yb = hw + 2.5; ab = (s.sfloor + s.stop) / 2
        for sy in (-1, 1):
            b = cyl_z(3.5, s.sz0, s.sz0 + 10, ab, sy * yb) - cyl_z(C.INSERT_R, s.sz0 - 1, s.sz0 + 5.5, ab, sy * yb)
            web = box(ab - 2, ab + 2, min(sy * hw, sy * yb), max(sy * hw, sy * yb), s.sz0, s.sz0 + 10)
            sl = sl + b + web
        # upper chamber walls, beside the pinion, up to the bearing bracket
        uw = box(s.stop - 0.5, WALL_TOP, -hw, hw, s.sz0, s.sz1 + 0.9) - box(s.stop - 1, WALL_TOP + 1, -Wp / 2, Wp / 2, s.sz0 - 1, s.sz1 + 2)
        uw = uw - box(s.stop + 3.0, WALL_TOP - 3.0, -hw - 1, hw + 1, s.sz0 + 6.0, s.flange_z0 - 3.0)       # windows, inside the mantle
        uw = uw - box(s.stop + 3.0, G_A0 - 1.0, -hw - 1, hw + 1, s.flange_z1 + 3.0, s.sz1 - 3.0)           # windows below the pinion
        # rails: idler cheek to gear web, beside the hinge
        rails = box(CHEEK_A0, WEB_A1, -hw, hw, s.end_z0, -6.0) - box(CHEEK_A0 - 1, WEB_A1 + 1, -hw + 2.0, hw - 2.0, s.end_z0 - 1, -5)
        # gear web with the 6800 seat
        if NEW:
            web = box(WEB_A0, WEB_A1, -hw, hw, -19.5, 0.0) - box(WEB_A0 - 1, WEB_A1 + 1, -hw + 3.5, hw - 3.5, -17.0, -16.5)
            web = web + cyl_x(BOSS_R1, WEB_A0, WEB_A1) - cyl_x(LIP_R, WEB_A0 - 1, WEB_A1 + 1)   # boss ring added after the gear cut
        else:
            web = (cyl_x(B6800["D"] / 2 + 3.0, WEB_A0, WEB_A1) + box(WEB_A0, WEB_A1, -hw, hw, -19.5, 0.0))
            web = web - cyl_x(B6800["D"] / 2, G_A0 - 0.5 - B6800["B"] - 0.01, WEB_A1 + 1)      # bearing pocket from the gear side
            web = web - box(WEB_A0 - 1, WEB_A1 + 1, -hw + 3.5, hw - 3.5, -17.0, -10.5)           # lightening
        # idler cheek and stub axle
        cheek = cyl_x(15.0, CHEEK_A0, CHEEK_A1) + box(CHEEK_A0, CHEEK_A1, -hw, hw, s.end_z0, 0.0)
        cheek = cheek - box(CHEEK_A0 - 1, CHEEK_A1 + 1, -hw + 3.5, hw - 3.5, -16.0, -10.5)
        if NEW:   # 6 mm boss round a pressed-in 7075 pin (the pin is its own part, idler_pin_local)
            cheek = cheek + cyl_x(7.0, CHEEK_A1 - 0.01, PIN_A1)
            stub = Manifold()
        else:
            stub = cyl_x(C.STUB_R, s.iear_lo, CHEEK_A0 + 0.01) - cyl_x(C.INSERT_R, s.iear_lo - 1, s.iear_lo + 5.0)
        # pinion bearing bracket with a through hole for the pinion and two cap inserts
        br = box(BR_A0, BR_A1, -hw, hw, s.pz - 14.0, s.pz + 13.5) ^ cyl_z(R + 0.5, -100, 100)
        br = br - cyl_x(Z_PIN / 2 * MOD + MOD + 0.5, BR_A0 - 1, BR_A1 + 1, 0, s.pz)
        for (yb, zb) in CAP_SCREWS:
            br = br + cyl_x(3.5, BR_A0, BR_A1, yb, s.pz + zb) - cyl_x(C.INSERT_R, BR_A1 - 4.2, BR_A1 + 1, yb, s.pz + zb)
        br = br - cyl_x(Z_PIN / 2 * MOD + MOD + 0.5, BR_A0 - 1, BR_A1 + 1, 0, s.pz)
        # mantle flange: the old 6-hole ring, widened to cover the servo side, 8 holes
        fz0, fz1 = s.flange_z0, s.flange_z1
        fl = cyl_z(R + C.FLANGE_EXTRA, fz0, fz1) + box(s.sfloor - 6.0, 0.0, -hw - 6.0, hw + 6.0, fz0, fz1)
        fl = fl - (cyl_z(R - 8.0, fz0 - 1, fz1 + 1) + box(s.sfloor - 0.01, s.stop, -hw + 0.01, hw - 0.01, fz0 - 1, fz1 + 1))
        rr = R + C.FLANGE_EXTRA / 2 - 0.5
        holes = [(rr * math.cos(math.radians(t)), rr * math.sin(math.radians(t))) for t in FLANGE_HOLE_DEG]
        holes += [(s.sfloor + 6.4, sy * (hw + 3.6)) for sy in (-1, 1)] + [(s.sfloor - 3.0, sy * 9.0) for sy in (-1, 1)]
        for (a, b) in holes:
            fl = fl - cyl_z(1.7, fz0 - 1, fz1 + 1, a, b)
        sock = sl + uw + rails + web + cheek + stub + br + fl
        # keep the gear's swept disc and the pinion's teeth clear
        sock = sock - cyl_x(Z_GEAR / 2 * MOD + MOD + GEAR_CLR, G_A0 - 0.5, G_A1 + 0.5)
        sock = sock - cyl_x(HUB_R + GEAR_CLR, G_A1, HUB_A1 + 0.5)
        sock = sock - cyl_x(Z_PIN / 2 * MOD + MOD + 0.5, G_A0 - 3.5, BR_A1 + 1.0, 0, s.pz)   # pinion, and its way in from +alpha
        if NEW:
            sock = sock - cyl_x(C.STUB_R - 0.02, CHEEK_A0 - 1, PIN_A1 + 1)                     # press fit for the idler pin
            boss = (cyl_x(BOSS_R1, WEB_A1 - 0.01, BRG_A1) - cyl_x(BOSS_R0, BRG_A0, BRG_A1 + 1)
                    - cyl_x(LIP_R, WEB_A1 - 1, BRG_A0 + 0.01))                                 # outer race seat, lip at BRG_A0
            sock = sock + boss
        else:
            sock = sock - cyl_x(SHAFT_R + 0.3, SHAFT_A0 - 1, G_A0, 0, 0)                       # gear shaft
        sock = sock - cyl_x(10.5, s.horn_a0 - 0.5, s.horn_a1 + 0.01, 0, s.pz)               # horn
        if GEAR_STYLE == "sector":
            # hard stops: two lugs off the gear web, in the part of the gear's circle that has no teeth. The
            # sector's cut edges reach them at |q| = STOP_DEG, before the pinion can run off the teeth.
            for sg in (-1, 1):
                pts = [(0.0, 0.0)] + [(sg * 19.0 * math.sin(math.radians(t)), -19.0 * math.cos(math.radians(t)))
                                      for t in np.linspace(SECTOR_DEG + STOP_DEG, SECTOR_DEG + STOP_DEG + 12.0, 7)]
                if sg < 0: pts = [pts[0]] + pts[:0:-1]
                lug_a1 = (BRG_A1 + 0.5) if NEW else (G_A0 + 4.0)                # with the recess the gear's cut edge meets them at r 15.6 .. 19
                lug = Manifold.extrude(CrossSection([pts]), lug_a1 - (WEB_A1 - 0.5)).transform(
                    np.array([[0, 0, 1, WEB_A1 - 0.5], [1, 0, 0, 0], [0, 1, 0, 0]], float))
                lug = lug - cyl_x(11.0 if not NEW else BOSS_R1 - 0.01, WEB_A0 - 1, G_A1)
                base = Manifold.extrude(CrossSection([pts]), WEB_A1 - WEB_A0).transform(
                    np.array([[0, 0, 1, WEB_A0], [1, 0, 0, 0], [0, 1, 0, 0]], float)) - cyl_x(B6800["D"] / 2, WEB_A0 - 1, WEB_A1 + 1)
                sock = sock + lug + base
        # sleeve pocket (servo) and its slide-in path stay open
        sock = sock - box(s.sp_a0, s.sp_a1, -Wp / 2, Wp / 2, s.sz0 - 5, s.sz1)
        sock = sock - box(s.horn_a0 - 0.5, G_A1 + 0.5, -10.6, 10.6, s.sz0 - 5, s.pz)       # horn + pinion path
        return sock

    def socket_parts_local(s):
        """separate printed parts: proximal end cap and the pinion bearing cap"""
        hw = s.hw; yb = hw + 2.5; ab = (s.sfloor + s.stop) / 2
        ec = box(s.sfloor, s.stop, -hw, hw, s.cap_z0, s.cap_z1)
        for sy in (-1, 1):
            ec = ec + cyl_z(3.5, s.cap_z0, s.cap_z1, ab, sy * yb) + box(ab - 3.5, ab + 3.5, min(sy * hw, sy * yb), max(sy * hw, sy * yb), s.cap_z0, s.cap_z1)
            ec = ec - cyl_z(1.7, s.cap_z0 - 1, s.cap_z1 + 1, ab, sy * yb)
        ec = ec - cyl_z(4.0, s.cap_z0 - 1, s.cap_z1 + 1, s.sp_a0 + 6.0, 0)                 # bus cable
        ec = ec - box(s.sp_a0 + 10, s.sp_a1 - 4, -s.Wp / 2 + 4, s.Wp / 2 - 4, s.cap_z0 - 1, s.cap_z1 + 1)
        # bearing cap: a spigot that locates in the bracket's pinion hole, a plate above it with two
        # screws on the gear side (the mantle flange is in the way on the other side), the 685 inside
        bc = (cyl_x(14.0, BR_A1, CAP_A1, 0, s.pz) ^ cyl_z(s.R + 0.5, -100, 100)) ^ box(-100, 100, -100, 100, s.flange_z1 + 0.5, 100)
        bc = bc + cyl_x(Z_PIN / 2 * MOD + MOD + 0.3, BR_A0 + 1.1, BR_A1 + 0.01, 0, s.pz)
        bc = bc - cyl_x(PB["D"] / 2, CAP_A1 - 0.8 - PB["B"], CAP_A1 - 0.8, 0, s.pz)
        bc = bc - cyl_x(PB["d"] / 2 + 1.0, BR_A0, CAP_A1 + 1, 0, s.pz)
        bc = bc - cyl_x(PB["D"] / 2 - 1.0, BR_A0, CAP_A1 - 0.8 - PB["B"] + 0.01, 0, s.pz)   # under the outer race
        for (yb, zb) in CAP_SCREWS:
            bc = bc - cyl_x(1.7, BR_A1 - 1, CAP_A1 + 1, yb, s.pz + zb)
        return ec, bc

    # ---------------------------------------------------------- metal parts, joint-0 frame
    # (with ROOT_BRG=61803 the gear also gets a 2 mm pocket in its outboard face and the pinion a centre bore, to pay for the bearings)
    def gear_local(s, solid=False):
        style = GEAR_STYLE
        g = gear_extrude(Z_GEAR, G_A0, G_A1)
        if solid: g = cyl_x(Z_GEAR / 2 * MOD + MOD, G_A0, G_A1)
        rf = Z_GEAR / 2 * MOD - 1.25 * MOD
        if style == "sector":
            # keep teeth over +-SECTOR_DEG about -zeta (the mesh point at q = 0); the rest is cut
            # back to the hub. The cut ends double as hard stops against the pinion bracket.
            h = Z_GEAR / 2 * MOD + 5
            wedge = CrossSection([[(0, 0)] + [(h * math.sin(math.radians(a)), -h * math.cos(math.radians(a)))
                                             for a in np.linspace(-SECTOR_DEG, SECTOR_DEG, 27)]])
            wedge = Manifold.extrude(wedge, FACE + 2).transform(np.array([[0, 0, 1, G_A0 - 1], [1, 0, 0, 0], [0, 1, 0, 0]], float))
            g = (g ^ (wedge + cyl_x(9.5, G_A0 - 1, G_A1 + 1)))
        g = g + cyl_x(HUB_R, G_A1 - 0.01, HUB_A1)
        if NEW:   # recess for the socket boss and the 61803; the 17 mm core is the spigot for the inner race
            g = g - (cyl_x(REC_R1, G_A0 - 1, BRG_A1 + 0.5) - cyl_x(SHOULDER_R, BRG_A1, BRG_A1 + 1)) + cyl_x(GB["d"] / 2, G_A0, BRG_A1 + 0.01)
        else:
            g = g + cyl_x(SHAFT_R, SHAFT_A0, G_A0 + 0.01)
        if NEW and not solid:
            h = Z_GEAR / 2 * MOD + 5; sd = SECTOR_DEG - 4.0
            wedge2 = CrossSection([[(0, 0)] + [(h * math.sin(math.radians(a)), -h * math.cos(math.radians(a))) for a in np.linspace(-sd, sd, 25)]])
            wedge2 = Manifold.extrude(wedge2, 3.5).transform(np.array([[0, 0, 1, G_A1 - 2.5], [1, 0, 0, 0], [0, 1, 0, 0]], float))
            g = g - (wedge2 ^ (cyl_x(REC_R1, G_A1 - 2.5, G_A1 + 1) - cyl_x(HUB_R + 0.5, G_A1 - 3.5, G_A1 + 2)))   # outboard pocket, 2 mm web left
        if not solid and not NEW:
            web_t = 2.5
            pocket = cyl_x(rf - 2.0, G_A0 - 1, G_A1 - web_t) - cyl_x(9.5, G_A0 - 2, G_A1)
            g = g - pocket                                                          # webbed: thin web next to the hub
            for k in range(6):                                                      # lightening holes in the web
                t = math.radians(60 * k + 30); r = (rf - 2.0 + 9.5) / 2
                g = g - cyl_x(3.0, G_A1 - 4, G_A1 + 0.1, r * math.cos(t), r * math.sin(t))
        if not solid:
            for k in range(4):                                                      # M3 tapped, fork horn ear
                t = math.radians(45 + 90 * k); y, z = C.HORN_PCD / 2 * math.cos(t), C.HORN_PCD / 2 * math.sin(t)
                g = g - cyl_x(1.25, HUB_A1 - 8.0, HUB_A1 + 1, y, z)
            if NEW:   # hollow spigot; no retaining screw: the bearing lip takes the arm's push toward the idler side, the
                      # idler ear on the cheek's PTFE washer the push the other way
                g = g - cyl_x(4.5, G_A0 - 1, BRG_A1 - 0.5)
            else:
                g = g - cyl_x(1.25, SHAFT_A0 - 1, SHAFT_A0 + 8.0)                   # M3 retaining screw
        return g

    def pinion_local(s, solid=False):
        p = (cyl_x(Z_PIN / 2 * MOD - 1.25 * MOD - 0.25, G_A0, G_A1) if solid else gear_extrude(Z_PIN, G_A0, G_A1, 180.0 / Z_PIN))
        p = p.translate([0, 0, s.pz]) + cyl_x(PB["d"] / 2, G_A1 - 0.01, CAP_A1 - 0.8, 0, s.pz)
        if not solid:
            for k in range(4):
                t = math.radians(45 + 90 * k); y, z = C.HORN_PCD / 2 * math.cos(t), C.HORN_PCD / 2 * math.sin(t)
                p = p - cyl_x(1.7, G_A0 - 1, G_A1 + 1, y, s.pz + z)
            if NEW: p = p - cyl_x(4.5, G_A0 - 1, G_A1 - 1.5, 0, s.pz)              # centre bore inside the bolt circle, 1.5 mm end wall
        return p

    def idler_pin_local(s):
        """7075 pin, 8 mm, pressed through the cheek boss; the idler ear turns on it; M3 tapped for the retaining screw"""
        if not NEW: return Manifold()
        return cyl_x(C.STUB_R, s.iear_lo, PIN_A1) - cyl_x(1.25, s.iear_lo - 1, s.iear_lo + 6.0)

    def servo0_local(s):
        sv = C.SERVO
        sv_box = box(s.sc_a0, s.sc_a1, -sv["W"] / 2, sv["W"] / 2, sv["spline_from_end"] - sv["L"] + s.dZ, sv["spline_from_end"] + s.dZ)
        return sv_box + cyl_x(5.0, s.sc_a1, s.horn_a0, 0, s.pz) + cyl_x(C.HORN_R, s.horn_a0, s.horn_a1, 0, s.pz)

    # ---------------------------------------------------------- overrides used by pin_cad.main()
    def _g(s, m): return m.transform(joint_affine(0, s.twist, s.pitch))

    def part(s, i):
        if i != 0: return super().part(i)
        return s._g(s.socket_local())

    def body(s, i):
        if i == 0:
            ec, bc = s.socket_parts_local()
            return s._g(s.socket_local() + ec + bc + s.servo0_local() + s.pinion_local(solid=True) + s.idler_pin_local())
        b = s.part(i)
        if i < s.J: b = b + s.servo_global(i) + s.lid_global(i)
        if i == 1: b = b + s._g(s.gear_local(solid=True))
        elif i >= 2: b = b + s.horn_global(i - 1)
        return b


def main():
    C.PinArm = GearedRootArm
    out = None
    for k, v in enumerate(sys.argv):
        if v == "--out": out = sys.argv[k + 1]
    root_servos = 5
    for k, v in enumerate(sys.argv):
        if v == "--root-servos": root_servos = int(sys.argv[k + 1])
    C.main()                                   # STLs, sweeps, assemblies, report with the new socket in place
    import argparse
    L = float(sys.argv[sys.argv.index("--length") + 1]); J = int(sys.argv[sys.argv.index("--joints") + 1])
    arm = GearedRootArm(L, C.TWIST, J)
    stl = os.path.join(out, "stl")
    # joint 0 has no lid now: one fewer lid to print
    old = os.path.join(stl, f"servo_lid_x{arm.J}.stl")
    if os.path.exists(old): os.replace(old, os.path.join(stl, f"servo_lid_x{arm.J - 1}.stl"))
    ec, bc = arm.socket_parts_local()
    gear, pin = arm.gear_local(), arm.pinion_local()
    to_trimesh(ec).export(os.path.join(stl, "root_end_cap_x1.stl"))
    to_trimesh(bc).export(os.path.join(stl, "root_bearing_cap_x1.stl"))
    to_trimesh(gear).export(os.path.join(stl, "gear_40T_m1_7075_x1.stl"))
    to_trimesh(pin).export(os.path.join(stl, "pinion_20T_m1_7075_x1.stl"))
    ipin = arm.idler_pin_local()
    if NEW: to_trimesh(ipin).export(os.path.join(stl, "idler_pin_8mm_7075_x1.stl"))
    # assembly of the root alone (socket, caps, servo, gear, pinion), joint-0 frame
    root = [arm.socket_local(), ec, bc, gear, pin, arm.servo0_local()] + ([ipin] if NEW else [])
    trimesh.util.concatenate([to_trimesh(m) for m in root]).export(os.path.join(out, "assembly_root_gear.stl"))

    # ---------------- masses
    pg = lambda m: m.volume() * C.PETG_RHO * C.EFF_FILL
    m_sock, m_ec, m_bc = pg(arm.socket_local()), pg(ec), pg(bc)
    m_gear, m_pin = gear.volume() * AL_RHO, pin.volume() * AL_RHO
    m_ipin = ipin.volume() * AL_RHO if NEW else 0.0
    fit = json.load(open(os.path.join(out, "fit.json")))
    # the old socket and the joint-0 lid, from pin_cad.py's own numbers for this arm
    base = C.PinArm.__mro__[1](L, C.TWIST, J)
    old_sock = base.part(0).volume() * C.PETG_RHO * C.EFF_FILL
    m_lid = fit["m_lid"]
    added = (m_sock + m_ec + m_bc - old_sock - m_lid) + m_gear + m_pin + m_ipin + GB["g"] + PB["g"] + HW_EXTRA_G
    # thrust-to-weight: fix_root.py's numbers (robot 14.714 kg before the gear stage, T/W 1.179 there)
    robot0, tw0 = 14.714, 1.179
    robot = robot0 + 8 * added / 1000
    tw = tw0 * robot0 / robot
    budget = (tw0 * robot0 / 1.15 - robot0) / 8 * 1000
    # mantle cut-out: the outline of everything that passes through the mantle (socket below the flange face,
    # the end cap and the servo), projected along the arm, plus 0.5 mm
    below = (arm.socket_local() + ec + arm.servo0_local()) ^ box(-200, 200, -200, 200, -200, arm.flange_z0 - 0.05)
    proj = below.project().offset(0.5)
    bb = proj.bounds()
    polys = proj.to_polygons()
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{bb[0]-5:.1f} {bb[1]-5:.1f} {bb[2]-bb[0]+10:.1f} {bb[3]-bb[1]+10:.1f}" width="{(bb[2]-bb[0]+10)*4:.0f}" height="{(bb[3]-bb[1]+10)*4:.0f}">',
           f'<circle cx="0" cy="0" r="{arm.R:.2f}" fill="none" stroke="#999" stroke-width="0.3"/>']
    for p in polys:
        svg.append('<polygon fill="none" stroke="#000" stroke-width="0.3" points="' + " ".join(f"{x:.2f},{y:.2f}" for x, y in p) + '"/>')
    svg.append("</svg>")
    open(os.path.join(out, "mantle_cutout_joint0_frame.svg"), "w").write("\n".join(svg))

    res = dict(socket_g=m_sock, end_cap_g=m_ec, bearing_cap_g=m_bc, gear_g=m_gear, pinion_g=m_pin,
               gear_bearing=GB["name"], gear_bearing_g=GB["g"], pinion_bearing=PB["name"], pinion_bearing_g=PB["g"], idler_pin_g=m_ipin,
               hw_extra_g=HW_EXTRA_G, root_brg=ROOT_BRG,
               old_socket_g=old_sock, joint0_lid_g=m_lid, added_g_per_arm=added, estimate_g=45.0, budget_g=budget,
               robot_kg=robot, thrust_to_weight=tw,
               mantle_cutout=dict(alpha_min=bb[0], alpha_max=bb[2], beta_min=bb[1], beta_max=bb[3], arm_R=arm.R),
               bulge_mm=-(arm.sfloor) - arm.R, gear_tip_r=Z_GEAR / 2 * MOD + MOD, pinion_tip_r=Z_PIN / 2 * MOD + MOD,
               centre_distance=CDIST, servo_case_alpha=[arm.sc_a0, arm.sc_a1], servo_case_zeta=[arm.sz0 + C.CLEAR, arm.sz1 - C.CLEAR],
               socket_zeta=[arm.cap_z0, 15.0], flange_zeta=[arm.flange_z0, arm.flange_z1])
    json.dump(res, open(os.path.join(out, "root_gear.json"), "w"), indent=1)
    lines = ["", "## Geared root socket (2:1 spur stage on joint 0)", "",
             f"- Printed: socket {m_sock:.0f} g (old socket {old_sock:.0f} g), end cap {m_ec:.1f} g, pinion bearing cap {m_bc:.1f} g; joint 0 drops its lid ({m_lid:.1f} g).",
             f"- 7075-T6: 40T gear {m_gear:.1f} g ({GEAR_STYLE}{" with teeth over +-%.0f deg" % SECTOR_DEG if GEAR_STYLE == "sector" else ", 6 lightening holes"}, {FACE:.0f} mm face), 20T pinion {m_pin:.1f} g{", idler pin %.1f g" % m_ipin if NEW else ""}. Bearings: {GB['name']} {GB['g']} g, {PB['name']} {PB['g']} g. Extra hardware {HW_EXTRA_G} g [A].",
             f"- Added mass per arm: {added:.1f} g against the 45 g estimate and the {budget:.0f} g budget that keeps thrust-to-weight at 1.15.",
             f"- Robot {robot:.2f} kg, thrust-to-weight {tw:.3f}.",
             f"- Servo bulge: the servo sleeve floor sits {res['bulge_mm']:.1f} mm outside the {2*arm.R:.1f} mm tube on one side; outside the mantle that is the stretch from the flange face ({arm.flange_z1:.1f} mm) to {arm.end_z1:.1f} mm from the hinge. Link 1 never reaches it (it stays inside the tube radius along the hinge axis).",
             f"- Mantle cut-out (joint-0 frame, mm): alpha {bb[0]:.1f} to {bb[2]:.1f}, beta {bb[1]:.1f} to {bb[3]:.1f}; outline in mantle_cutout_joint0_frame.svg."]
    rep = open(os.path.join(out, "fit_report.md")).read()
    rep = rep.replace(f"lid {m_lid:.1f} g (x{arm.J})", f"lid {m_lid:.1f} g (x{arm.J - 1}; joint 0 has none)")
    rep = rep.replace("root socket", "geared root socket", 1)
    import re
    arm_all = fit["arm_out_g"] + m_sock + m_ec + m_bc + C.SERVO_ROOT["mass"] + m_gear + m_pin + m_ipin + GB["g"] + PB["g"] + C.HW_G + HW_EXTRA_G
    res["arm_all_g"] = arm_all
    json.dump(res, open(os.path.join(out, "root_gear.json"), "w"), indent=1)
    rep = re.sub(r"\(\d+ g counting the socket, root servo and its lid\)", f"({arm_all:.0f} g counting the geared root: socket, caps, root servo, gear stage and bearings)", rep)
    open(os.path.join(out, "fit_report.md"), "w").write(rep + "\n".join(lines) + "\n")
    print("\n".join(lines))

if __name__ == "__main__":
    main()
