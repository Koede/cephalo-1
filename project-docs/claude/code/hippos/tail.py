"""Hippos-1 tail, layout C (pitch - yaw - pitch - yaw), 400 mm.  Draft CAD, 2026-11-16.  Single source of truth.

Connector frame (mm) as tailmount.py: z = SFF axis, +z into the rump, the tail runs toward -z.  On the horse:
z_c = -x (backward along -z_c), x_c = up, y_c = left.  So a pitch joint turns about y_c, a yaw joint about x_c.

Chain (joint-to-joint 100 / 95 / 85, last joint to the tip socket mouth 120):
  root_cradle  bolts to the tail_adapter flange (4 x M3), holds j0 STS3095-C002 (pitch, 12 V, 2.21 / 9.32 N m rated / stall)
  link0        yoke round j0, beam, cradle for j1 STS3250 (yaw)
  link1        yoke round j1, beam, cradle for j2 STS3215 (pitch)
  link2        yoke round j2, beam, cradle for j3 STS3215 (yaw)
  link3        yoke round j3, beam, pad for the standard SFF socket (sff_core.arm_socket) at the tip, mouth facing -z
  rump_pad     disc bonded / bolted into the rump end ring: the rump_socket flange bolts to its 4 M3 inserts

Each joint: the servo case sits in the proximal link's cradle with its long side pointing back up the tail (+z); the distal
link's yoke has two plates outside the case: horn side bolted to the output horn, idler side on a 6800 bearing over the
servo's rear shaft.  Servo case sizes from the listings; shaft position, horn bolt circle and rear shaft are ESTIMATES
(measure before printing, as for the arm servos).
"""
import os, sys
import numpy as np
import manifold3d as mf
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sff_core as H
from sff_core import cyl

# (case L along the long side, W, H along the shaft, a = shaft from the distal case end, horn r, mass kg, rated, stall N m)
SERVO = {"STS3095": (65.0, 30.0, 48.0, 15.0, 12.0, 0.1945, 2.21, 9.32),
         "STS3250": (45.2, 24.7, 35.0, 12.0, 10.0, 0.0745, 1.57, 4.90),
         "STS3215": (45.2, 24.7, 35.0, 12.0, 10.0, 0.055, 0.98, 2.94)}
JOINTS = [("STS3095", "y"), ("STS3250", "x"), ("STS3215", "y"), ("STS3215", "x")]     # C: P Y P Y
LINKS = [100.0, 95.0, 85.0, 120.0]
CL = 0.3            # case clearance in the cradle
WALL = 3.0          # cradle wall
LIP_R = 17.0        # cradle side lips start this far behind the axis (clear of the yoke lobe)
PLATE_GAP = 0.5     # horn face to plate
HORN_T = 3.0
PLATE_T = 5.0       # yoke plate (6800 bearing seat on the idler side)
LOBE_R = 16.0
BRIDGE_T = 6.0
BEAM = 24.0         # beam outer square, 2.5 wall, wire channel inside
Z_FLANGE = -35.0    # tail_adapter servo flange underside
DISC_T = 4.0

def box(x0, x1, y0, y1, z0, z1):
    return mf.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def rect_hull(hx0, hy0, z0, hx1, hy1, z1):
    """solid hull between a rectangle (+-hx0, +-hy0) at z0 and (+-hx1, +-hy1) at z1 (a point if hx1 = hy1 = 0)"""
    a = box(-hx0, hx0, -hy0, hy0, z0 - 0.01, z0) if z0 < z1 else box(-hx0, hx0, -hy0, hy0, z0, z0 + 0.01)
    b = box(-max(hx1, 0.01), max(hx1, 0.01), -max(hy1, 0.01), max(hy1, 0.01), z1, z1 + 0.01) if z0 < z1 else \
        box(-max(hx1, 0.01), max(hx1, 0.01), -max(hy1, 0.01), max(hy1, 0.01), z1 - 0.01, z1)
    return mf.Manifold.hull(a + b)

def ycyl(r, y0, y1, x=0.0, z=0.0, n=64):
    return mf.Manifold.cylinder(y1 - y0, r, r, n).rotate([-90, 0, 0]).translate([x, y0, z])

def to_axis(m, axis):
    """Geometry is built for a y-axis joint; a yaw joint turns it 90 deg about z (y -> x)."""
    return m if axis == "y" else m.rotate([0, 0, -90])

def joint_z():
    """z of each joint axis.  j0 sits where the STS3095 case's proximal end meets the root cradle disc."""
    L, W, Hh, a = SERVO[JOINTS[0][0]][:4]
    z0 = Z_FLANGE - DISC_T - CL - (L - a)
    zs = [z0]
    for Lk in LINKS[:3]: zs.append(zs[-1] - Lk)
    return np.array(zs), zs[-1] - LINKS[3]

ZJ, Z_TIP = joint_z()

def d_bridge(s):
    L, W, Hh, a = SERVO[s][:4]
    return np.hypot(a, W / 2) + 3.0

# ---------------------------------------------------------------- servo envelope (for checks and the viewer)
def servo_env(k):
    s, axis = JOINTS[k]; L, W, Hh, a, rh = SERVO[s][:5]; z = ZJ[k]
    m = box(-W / 2, W / 2, -Hh / 2, Hh / 2, z - a, z + L - a)
    m += ycyl(rh, Hh / 2, Hh / 2 + HORN_T, z=z) + ycyl(5.0, -Hh / 2 - PLATE_GAP - PLATE_T + 0.5, -Hh / 2, z=z, n=32)
    return to_axis(m, axis)

# ---------------------------------------------------------------- cradle (proximal side of joint k), in the joint's own axis
def cradle(k, top_cap=True):
    s, axis = JOINTS[k]; L, W, Hh, a = SERVO[s][:4]; z = ZJ[k]
    xi, yi = W / 2 + CL, Hh / 2 + CL; ztop = z + L - a + CL
    m = box(-xi - WALL, -xi, -yi, yi, z + LIP_R, ztop)
    m += box(xi, xi + WALL, -yi, yi, z + LIP_R, ztop)
    m += box(-xi - WALL, xi + WALL, -yi - 2.2, -yi, z + LIP_R, ztop) + box(-xi - WALL, xi + WALL, yi, yi + 2.2, z + LIP_R, ztop)
    h_roof = 0.0
    if top_cap:
        xo, yo = xi + WALL, yi + 2.2; bo, bi = BEAM / 2, BEAM / 2 - 2.5
        h_roof = max(xo - bo, yo - bo, xi - bi, yi - bi) + 0.5
        m += rect_hull(xo, yo, ztop, bo, bo, ztop + h_roof) - rect_hull(xi, yi, ztop - 0.02, bi, bi, ztop + h_roof + 0.02)
    # 2 x M2.5 clamp screws through each side wall into the case's side holes (positions est.)
    for zz in (z + LIP_R + 6, ztop - 6):
        m -= box(-xi - WALL - 1, xi + WALL + 1, -1.35, 1.35, zz - 1.35, zz + 1.35)
    return to_axis(m, axis), ztop + h_roof

# ---------------------------------------------------------------- yoke (distal side of joint k)
def yoke(k):
    s, axis = JOINTS[k]; L, W, Hh, a, rh = SERVO[s][:5]; z = ZJ[k]
    db = d_bridge(s); y0 = Hh / 2 + HORN_T + PLATE_GAP; y1 = y0 + PLATE_T
    hw = LOBE_R - 2
    plate = lambda ya, yb: (ycyl(LOBE_R, ya, yb, z=z) + box(-hw, hw, ya, yb, z - db - BRIDGE_T, z))
    m = plate(y0, y1) + plate(-y1, -y0)
    m += box(-hw, hw, -y1, y1, z - db - BRIDGE_T, z - db)                        # bridge
    zb0 = z - db - BRIDGE_T; bo, bi = BEAM / 2, BEAM / 2 - 2.5
    g = max(y1 - bo, hw - bo) + 0.5
    m += rect_hull(hw, y1, zb0 + 0.01, bo, bo, zb0 - g)                         # 45 deg funnel under the bridge
    m -= rect_hull(bi, bi, zb0 - g - 0.02, 0.0, 0.0, zb0 - g + bi)              # pyramid cavity: the wire channel ends in a 45 deg point
    # horn side: centre hole + 4 x M2.5 on r 7 (horn pattern: measure); idler side: 6800 seat (19 x 5) + 14 through
    m -= ycyl(3.0, y0 - 1, y1 + 1, z=z, n=32)
    for t in range(4):
        aa = np.radians(45 + 90 * t)
        m -= ycyl(1.35, y0 - 1, y1 + 1, x=7 * np.cos(aa), z=z + 7 * np.sin(aa), n=20)
    m -= ycyl(9.5, -y1 - 1, -y1 + 4.5, z=z) + ycyl(7.0, -y1 - 1, -y0 + 1, z=z)
    return to_axis(m, axis), z - db - BRIDGE_T - g

def beam(z_top, z_bot):
    h = BEAM / 2
    m = box(-h, h, -h, h, z_bot, z_top) - box(-h + 2.5, h - 2.5, -h + 2.5, h - 2.5, z_bot - 1, z_top + 1)
    return m

# ---------------------------------------------------------------- parts
FLANGE_HOLES = [(25.0 * np.cos(np.radians(a)), 25.0 * np.sin(np.radians(a))) for a in (20, -20, 160, 200)]

def root_cradle():
    c, _ = cradle(0, top_cap=False)
    disc = cyl(28.0, Z_FLANGE - DISC_T, Z_FLANGE, 96)
    m = c + disc
    # the cradle walls run up to the disc
    s, axis = JOINTS[0]; L, W, Hh, a = SERVO[s][:4]
    for x, y in FLANGE_HOLES:
        m -= cyl(1.7, Z_FLANGE - DISC_T - 1, Z_FLANGE + 1, 24).translate([x, y, 0])
    # wire channel: from the centre (under the adapter's wire bore) out along +x past the cradle wall, 6 wide 2 deep in the top face
    m -= box(0, 30, -3, 3, Z_FLANGE - 2.0, Z_FLANGE + 1) + cyl(4.0, Z_FLANGE - 2.0, Z_FLANGE + 1, 32)
    return m

def link(k):
    """link k = yoke round joint k + beam + cradle for joint k+1 (or the tip pad for k = 3)."""
    y, zb = yoke(k)
    wire = cyl(5.0, zb - 2, zb + 35, 32)                                    # wire hole up through the funnel and the bridge
    if k < 3:
        c, zc = cradle(k + 1)
        wire += cyl(5.0, zc - WALL - 1, zc + 1, 32)                          # and through the cradle cap to the next servo
        return y + c + beam(zb + 0.5, zc - 0.5) - wire
    # tip: SFF socket pad (46 x 32, 4 x M3 inserts at (+-19, +-9)), socket mouth at Z_TIP facing -z
    zp = Z_TIP + H.P["z_top"]
    pad = box(-23, 23, -16, 16, zp, zp + 5.0)
    for sx in (-19, 19):
        for sy in (-9, 9):
            pad -= cyl(2.0, zp - 1, zp + 5.5, 24).translate([sx, sy, 0])
    pad -= cyl(5.0, zp - 1, zp + 6, 32)                                     # wire hole to the pin board
    return y + pad + beam(zb + 0.5, zp + 4.9) - wire

def tip_socket():
    """standard SFF arm socket, flipped so +z_socket = +z_c (into the tail), mouth at Z_TIP."""
    return H.arm_socket().translate([0, 0, Z_TIP])

def rump_pad():
    """Ø76 x 5 disc in the rump end ring: 4 x M3 heat-set inserts at r 29.5 (rump_socket flange), 4 x M3 clearance at r 34 to
    the end ring, Ø14 wire hole.  Sits behind the rump_socket flange (z 10.9 .. 15.9)."""
    z0 = H.P["z_top"]
    m = cyl(38.0, z0, z0 + 5.0, 96) - cyl(7.0, z0 - 1, z0 + 6, 48)
    for t in range(4):
        aa = np.radians(45 + 90 * t)
        m -= cyl(2.0, z0 - 1, z0 + 5.1, 24).translate([29.5 * np.cos(aa), 29.5 * np.sin(aa), 0])
        bb = np.radians(90 * t)
        m -= cyl(1.7, z0 - 1, z0 + 6, 24).translate([34.0 * np.cos(bb), 34.0 * np.sin(bb), 0])
    return m

def parts():
    return {"root_cradle": root_cradle(), "link0": link(0), "link1": link(1), "link2": link(2), "link3": link(3),
            "tip_socket": tip_socket(), "rump_pad": rump_pad()}

# ---------------------------------------------------------------- kinematics: pose every part
def joint_axis(k):
    return np.array([0, 1.0, 0]) if JOINTS[k][1] == "y" else np.array([1.0, 0, 0])

def rotm(axis, ang_deg):
    a = np.radians(ang_deg); x, y, z = axis; c, s = np.cos(a), np.sin(a); C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s], [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])

def pose_transforms(q):
    """4x4 transforms for link0..link3 (and their distal servos) for joint angles q (deg)."""
    T = np.eye(4); out = []
    for k in range(4):
        piv = np.array([0, 0, ZJ[k]])
        R = rotm(joint_axis(k), q[k]); Tk = np.eye(4); Tk[:3, :3] = R; Tk[:3, 3] = piv - R @ piv
        T = T @ Tk; out.append(T.copy())
    return out

def apply(m, T):
    return m.transform(T[:3, :].tolist())

if __name__ == "__main__":
    print("joint z", ZJ.round(1), "tip mouth z", round(Z_TIP, 1), "| seat -> j0", round(3.8 - ZJ[0], 1), "mm")
    for n, m in parts().items():
        t = H.to_trimesh(m)
        print(f"{n:12s} {m.status()} bodies {len(m.decompose())} watertight {t.is_watertight} vol {m.volume()/1000:.1f} cm3 bbox {np.round(m.bounding_box(), 1)}")
