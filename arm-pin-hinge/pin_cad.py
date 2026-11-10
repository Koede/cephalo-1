#!/usr/bin/env python3
"""
pin_cad.py  --  parametric pin-hinge (clevis) tentacle for Cephalo-1, the main arm design

One bus servo per joint: STS3250 in the first ROOT_SERVOS joints (4, chosen Nov 9), STS3215
beyond; the two share one case, so every cradle is the same. Each joint is a plain pin hinge whose axis crosses the arm
centreline at right angles; successive hinge axes turn TWIST degrees about the arm (60 = the
"helix60" arrangement picked in the joint study, 90 = plain alternating). The servo sits in a
cradle in the proximal vertebra with its output shaft on the hinge axis. The distal vertebra
ends in a fork: one ear is bolted to the servo horn, the other turns on a printed stub axle
on the back of the cradle, retained by an M3 button-head screw in a heat-set insert.

Because the twist per joint is constant, every moving vertebra is the same part turned about
the arm axis: print one vertebra STL N-1 times, one lid per servo, one socket and one tip.

  python3 pin_cad.py                          # defaults below (585 mm, helix60, 9 joints)
  python3 pin_cad.py --length 900             # any arm length; joint count follows the fit
  python3 pin_cad.py --length 900 --joints 13 # fixed joint count (pitch = length / joints)
  python3 pin_cad.py --twist 90               # alternating axes instead of the 60 deg helix
  python3 pin_cad.py --root-servos 0          # STS3215 in every joint

Writes out_pin/: STLs, straight/curl/coil assemblies, fit_report.md and preview.json.
Requires numpy, manifold3d, trimesh. Units mm.
Frames: arm axis z (root -> tip). Joint j sits at z = j*pitch with its axis a_j in the x-y
plane at angle j*TWIST from +x. Joint-local coordinates (alpha, beta, zeta) run along a_j,
along z x a_j, and along z.
"""
import argparse, json, math, os
import numpy as np
from manifold3d import Manifold, set_circular_segments
import trimesh

# =============================================================================
# PARAMETERS  ([A] = assumed, measure the part in hand before printing)
# =============================================================================
ARM_LENGTH = 585.0      # joint 0 axis -> tip face. Forest says the real arm is longer: pass --length
TWIST = 60.0            # deg between successive hinge axes about the arm (helix60)
RANGE = 90.0            # deg total commanded travel per joint (+-45), as in the study's pick
JOINTS = None           # None = as many as fit (length // shortest pitch that clears)
SERVO = dict(name="STS3215", L=45.2, W=24.7, H=35.0, mass=55.0,  # H runs along the output shaft
             spline_from_end=11.0,      # [A] output shaft centre from the near end of the case
             shaft_above_top=4.0)       # [A] horn seat above the case top
SERVO_ROOT = dict(SERVO, name="STS3250", mass=74.5)   # same case as the STS3215, 4.9 N m stall
ROOT_SERVOS = 4         # joints 0..ROOT_SERVOS-1 carry the STS3250 (main design: joints 0-3)
HORN_T = 3.0            # [A] horn disc thickness
HORN_R = 10.0           # [A] horn disc radius
HORN_PCD = 14.0         # [A] horn bolt circle (4 x M3)
CLEAR = 0.5             # servo -> pocket clearance per side
WALL = 1.8              # cradle side walls (3 perimeters at 0.6 mm)
FLOOR_T = 2.4           # cradle floor under the servo back (carries the idler stub)
LID_T = 2.0             # cradle lid over the servo top (spline side)
END_T = 3.0             # bulkhead at the proximal end of each cradle
RING_T = 2.4            # tube wall that joins the fork ears to the next cradle
HORN_EAR_T = 4.0        # fork ear bolted to the horn (heads sit in 1.8 mm counterbores)
IDLER_EAR_T = 3.2       # fork ear that turns on the stub axle
EAR_GAP = 0.5           # PTFE washer between idler ear and cradle floor
BOSS_R = 10.5           # ear boss radius round the hinge axis
STUB_R = 4.0            # printed stub axle radius (idler)
HEAD_H = 2.2            # M3 button head + washer standing proud of the idler ear
INSERT_R = 2.0          # M3 heat-set insert hole radius (4.0 mm hole for a 4.5 mm insert)
LID_BOSS_R = 3.5        # lid screw bosses on the cradle sides (M3 inserts)
SPOKE_W = 5.0           # bulkhead spokes joining the servo frame to the ring
WINDOW_INSET = 3.0      # lightening window inset from the edges of each cradle side wall
FLANGE_EXTRA = 6.0      # root flange overhang beyond the arm radius (mantle mount)
PETG_RHO = 1.27e-3      # g/mm^3
EFF_FILL = 0.95         # printed / solid mass for thin parts at 3-4 perimeters [A]
HW_G = 5.0              # g per joint: 4 M3x6 horn screws, M3 idler screw + washer, 6 inserts, PTFE washer
SEG = 64
set_circular_segments(SEG)

# =============================================================================
def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cyl_x(r, x0, x1, y=0.0, z=0.0):
    return Manifold.cylinder(x1 - x0, r, r, SEG).rotate([0, 90, 0]).translate([x0, y, z])

def cyl_z(r, z0, z1, x=0.0, y=0.0):
    return Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])

def tube_z(ro, ri, z0, z1):
    return cyl_z(ro, z0, z1) - cyl_z(ri, z0 - 1, z1 + 1)

def axis_dir(j, twist):
    t = math.radians(j * twist); return np.array([math.cos(t), math.sin(t), 0.0])

def joint_affine(j, twist, pitch):
    a = axis_dir(j, twist); b = np.cross([0, 0, 1.0], a)
    M = np.zeros((3, 4)); M[:, 0] = a; M[:, 1] = b; M[:, 2] = [0, 0, 1.0]; M[:, 3] = [0, 0, j * pitch]
    return M

def rot_about_line(p, a, q):
    a = np.asarray(a, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p - R @ p
    return T

def to_trimesh(m):
    g = m.to_mesh(); return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=False)

# =============================================================================
class PinArm:
    def __init__(s, length, twist, joints=None, rng=RANGE, ring_start=None):
        s.twist, s.rng, s.sv = twist, rng, SERVO
        sv = SERVO
        # ---- servo stack along the hinge axis (alpha), centred to minimise the outer radius
        s.Wp = sv["W"] + 2 * CLEAR; s.Hp = sv["H"] + 2 * CLEAR; s.Lp = sv["L"] + 2 * CLEAR
        s.zeta_lo = sv["spline_from_end"] - sv["L"] - CLEAR      # pocket, along the arm (joint-local)
        s.zeta_hi = sv["spline_from_end"] + CLEAR
        best = None
        for ct in np.arange(0.0, 30.0, 0.05):                      # ct = case top (spline face) alpha
            r = s._radius_for(ct)
            if best is None or r < best[0]: best = (r, ct)
        s.R, s.c_top = best
        s.R = math.ceil(s.R * 2 * 10) / 20                         # round OD up to 0.1 mm
        s._stack(s.c_top)
        # ---- cradle footprint along zeta and the cap radius the next ring must clear
        s.cr_lo = s.zeta_lo - END_T
        s.cr_hi = s.zeta_hi + WALL
        s.rho_cap = math.hypot(s.Wp / 2 + WALL + 0.0, s.cr_hi)
        # ---- shortest pitch: next joint's bulkhead must sit beyond the ring start
        s.ring_start = ring_start if ring_start is not None else s.rho_cap + 1.0
        s.pitch_min = s.ring_start + 2.0 - s.cr_lo                # >= 2 mm of ring before the bulkhead
        n = joints or max(2, int(length // s.pitch_min))
        s.pitch = length / n
        if s.pitch < s.pitch_min - 1e-6:
            raise SystemExit(f"{n} joints need a pitch of {s.pitch:.1f} mm but the fit needs >= {s.pitch_min:.1f} mm")
        s.J = n                                                   # joints = servos
        s.length = n * s.pitch

    def _stack(s, ct):
        sv = SERVO
        s.case_lo, s.case_hi = ct - sv["H"], ct
        s.p_lo, s.p_hi = s.case_lo - CLEAR, s.case_hi + CLEAR
        s.floor_lo = s.p_lo - FLOOR_T
        s.lid_hi = s.p_hi + LID_T
        s.horn_lo, s.horn_hi = ct + sv["shaft_above_top"], ct + sv["shaft_above_top"] + HORN_T
        s.hear_lo, s.hear_hi = s.horn_hi, s.horn_hi + HORN_EAR_T
        s.iear_hi = s.floor_lo - EAR_GAP
        s.iear_lo = s.iear_hi - IDLER_EAR_T
        s.head_lo = s.iear_lo - HEAD_H

    def _radius_for(s, ct):
        s._stack(ct)
        hw = s.Wp / 2 + WALL
        pts = [(s.hear_hi, BOSS_R), (s.iear_lo, BOSS_R), (s.head_lo, 3.5),
               (s.floor_lo, hw), (s.lid_hi, hw), (s.lid_hi, s.Wp / 2 + 2 * LID_BOSS_R)]
        return max(math.hypot(a, b) for a, b in pts)

    # ---------------------------------------------------------------- parts in joint-local frame
    def lid_boss_pos(s):
        yb = s.Wp / 2 + LID_BOSS_R - 0.6
        zs = (s.zeta_lo + 6.0, min(s.zeta_hi - 4.0, -4.0))
        return [(sy * yb, z) for sy in (-1, 1) for z in zs]

    def cradle(s, with_stub=True):
        """servo cradle for joint j in that joint's frame: floor, side walls, bulkhead, lid bosses, stub"""
        hw = s.Wp / 2 + WALL
        outer = box(s.floor_lo, s.p_hi, -hw, hw, s.cr_lo, s.cr_hi)
        pocket = box(s.p_lo, s.p_hi + 1, -s.Wp / 2, s.Wp / 2, s.zeta_lo, s.zeta_hi)
        c = outer - pocket
        # lightening windows in both side walls
        wa0, wa1 = s.floor_lo + WINDOW_INSET, s.p_hi - WINDOW_INSET
        wz0, wz1 = s.zeta_lo + WINDOW_INSET + 4, s.zeta_hi - WINDOW_INSET - 2
        if wa1 > wa0 + 4 and wz1 > wz0 + 4:
            c = c - box(wa0, wa1, -hw - 1, -s.Wp / 2 + 0.01, wz0, wz1) - box(wa0, wa1, s.Wp / 2 - 0.01, hw + 1, wz0, wz1)
        # floor window, clear of the idler stub
        c = c - box(s.floor_lo - 1, s.p_lo + 0.01, -s.Wp / 2 + 3, s.Wp / 2 - 3, s.zeta_lo + 4, -STUB_R - 4)
        # lid screw bosses (M3 inserts from the lid side)
        for (y, z) in s.lid_boss_pos():
            b = cyl_x(LID_BOSS_R, s.p_hi - 7.0, s.p_hi, y, z) - cyl_x(INSERT_R, s.p_hi - 5.5, s.p_hi + 1, y, z)
            c = c + b
        # spline end: round the cradle about the hinge axis so the next ring clears it
        cap = cyl_x(s.rho_cap, s.floor_lo - 1, s.lid_hi + 1) + box(s.floor_lo - 1, s.lid_hi + 1, -60, 60, s.cr_lo - 1, 0)
        c = c ^ cap
        if with_stub:   # idler stub axle on the back, through the idler ear, with an M3 insert
            stub = cyl_x(STUB_R, s.iear_lo, s.floor_lo + 0.01) - cyl_x(INSERT_R, s.iear_lo - 1, s.iear_lo + 5.0)
            c = c + stub
        return c

    def bulkhead(s):
        """plate at the proximal end of a cradle that joins it to the ring: outer annulus, a frame
        round the servo end, and four spokes; the gaps between spokes pass the bus cable"""
        z0, z1 = s.cr_lo, s.cr_lo + END_T
        hw = s.Wp / 2 + WALL
        ann = tube_z(s.R, s.R - RING_T, z0, z1)
        frame = box(s.floor_lo, s.p_hi, -hw, hw, z0, z1) - box(s.p_lo + 3, s.p_hi - 3, -s.Wp / 2 + 3, s.Wp / 2 - 3, z0 - 1, z1 + 1)
        sp = SPOKE_W / 2
        spokes = (box(-s.R, s.R, -sp, sp, z0, z1) + box(-sp, sp, -s.R, s.R, z0, z1)) ^ cyl_z(s.R - 0.5, z0, z1)
        return ann + frame + spokes

    def lid(s):
        hw = s.Wp / 2 + WALL
        L = box(s.p_hi, s.lid_hi, -hw, hw, s.cr_lo + END_T, s.cr_hi)
        L = L - cyl_x(6.0, s.p_hi - 1, s.lid_hi + 1)                      # spline / horn hub
        for (y, z) in s.lid_boss_pos():
            L = L - cyl_x(1.7, s.p_hi - 1, s.lid_hi + 1, y, z)
        # window between the bosses
        L = L - box(s.p_hi - 1, s.lid_hi + 1, -s.Wp / 2 + 3, s.Wp / 2 - 3, s.zeta_lo + 12, -10)
        cap = cyl_x(s.rho_cap, s.p_hi - 1, s.lid_hi + 1) + box(s.p_hi - 1, s.lid_hi + 1, -60, 60, s.cr_lo - 1, 0)
        return L ^ cap

    def fork(s, ring_to):
        """fork ears for a joint in that joint's frame, plus the ring up to ring_to (joint-local zeta)"""
        top = ring_to
        def ear(a0, a1, boss_r):
            return cyl_x(boss_r, a0, a1) + box(a0, a1, -BOSS_R, BOSS_R, 0.0, top)
        h = ear(s.hear_lo, s.hear_hi, BOSS_R)
        h = h - cyl_x(3.0, s.hear_lo - 1, s.hear_hi + 1)                    # horn screw access
        for k in range(4):
            t = math.radians(45 + 90 * k); y, z = HORN_PCD / 2 * math.cos(t), HORN_PCD / 2 * math.sin(t)
            h = h - cyl_x(1.7, s.hear_lo - 1, s.hear_hi + 1, y, z) - cyl_x(3.0, s.hear_hi - 1.8, s.hear_hi + 1, y, z)
        i = ear(s.iear_lo, s.iear_hi, BOSS_R) - cyl_x(STUB_R + 0.2, s.iear_lo - 1, s.iear_hi + 1)
        ring = tube_z(s.R, s.R - RING_T, s.ring_start, ring_to)
        f = h + i + ring
        return f ^ cyl_z(s.R, -s.R - 5, ring_to + 1)

    def servo_solid(s):
        sv = SERVO
        return (box(s.case_lo, s.case_hi, -sv["W"] / 2, sv["W"] / 2, sv["spline_from_end"] - sv["L"], sv["spline_from_end"])
                + cyl_x(5.0, s.case_hi, s.horn_lo) )

    def horn_solid(s):
        return cyl_x(HORN_R, s.horn_lo, s.horn_hi)

    # ---------------------------------------------------------------- printed parts, global frame
    def part(s, i):
        """i = 0 root socket, 1..J-1 moving vertebrae, J tip. Global (straight arm) frame."""
        P, tw = s.pitch, s.twist
        parts = []
        if i < s.J:        # holds servo i
            parts.append(s.cradle().transform(joint_affine(i, tw, P)))
            parts.append(s.bulkhead().transform(joint_affine(i, tw, P)))
        if i == 0:
            fl = cyl_z(s.R + FLANGE_EXTRA, s.cr_lo - 3.0, s.cr_lo)
            for k in range(6):
                t = math.radians(60 * k + 30); r = s.R + FLANGE_EXTRA / 2 - 0.5
                fl = fl - cyl_z(1.7, s.cr_lo - 4, s.cr_lo + 1, r * math.cos(t), r * math.sin(t))
            fl = fl - cyl_z(s.R - 8, s.cr_lo - 4, s.cr_lo + 1)
            parts.append(fl.transform(joint_affine(0, tw, P)))
        if i >= 1:         # fork for joint i-1 and the ring up to this vertebra's bulkhead (or tip face)
            ring_to = (P + s.cr_lo + 0.5) if i < s.J else P
            f = s.fork(ring_to)
            if i == s.J:   # tip plate: claw mount, 4 x M3 inserts on a 30 mm circle
                tp = cyl_z(s.R, P - 3.0, P)
                for k in range(4):
                    t = math.radians(45 + 90 * k)
                    tp = tp - cyl_z(INSERT_R, P - 4, P + 1, 15 * math.cos(t), 15 * math.sin(t))
                tp = tp - cyl_z(6, P - 4, P + 1)
                f = f + tp
                # lightening windows in the long tip ring, between the ears
                z0, z1 = s.ring_start + 6.0, P - 8.0
                if z1 > z0 + 6:
                    for k in range(2):
                        w = box(-9.0, 9.0, 0.0, s.R + 2, z0, z1).rotate([0, 0, 90 + 180 * k])
                        f = f - w
            parts.append(f.transform(joint_affine(i - 1, tw, P)))
        out = parts[0]
        for p in parts[1:]: out = out + p
        return out

    def servo_global(s, j):
        return s.servo_solid().transform(joint_affine(j, s.twist, s.pitch))

    def horn_global(s, j):
        return s.horn_solid().transform(joint_affine(j, s.twist, s.pitch))

    def lid_global(s, j):
        return s.lid().transform(joint_affine(j, s.twist, s.pitch))

    # ---------------------------------------------------------------- kinematics and checks
    def body(s, i):
        """everything that moves with link i: printed part, its servo + lid, and the horn of joint i-1"""
        b = s.part(i)
        if i < s.J: b = b + s.servo_global(i) + s.lid_global(i)
        if i >= 1: b = b + s.horn_global(i - 1)
        return b

    def pose_frames(s, qs):
        W = [np.eye(4)]
        for j in range(s.J):
            p = np.array([0, 0, j * s.pitch]); a = axis_dir(j, s.twist)
            W.append(W[-1] @ rot_about_line(p, a, qs[j]))
        return W

    def joint_sweep(s, j, bodies, step=5.0, lim=90.0):
        """free travel of joint j: link j+1 turned against link j"""
        A, B = bodies[j], bodies[j + 1]
        p = np.array([0, 0, j * s.pitch]); a = axis_dir(j, s.twist)
        res = {}
        for q in np.arange(-lim, lim + 0.1, step):
            res[float(q)] = float((A ^ B.transform(rot_about_line(p, a, math.radians(q))[:3, :])).volume())
        lo = 0.0
        for q in np.arange(0, -lim - 0.1, -step):
            if res[float(q)] > 2.0: break
            lo = float(q)
        hi = 0.0
        for q in np.arange(0, lim + 0.1, step):
            if res[float(q)] > 2.0: break
            hi = float(q)
        return lo, hi

    def pair_sweep(s, j, bodies, step=5.0, lim=60.0):
        """joints j and j+1 turned together (same sign and opposite sign); link j against link j+2"""
        A, C = bodies[j], bodies[j + 2]
        worst = 0.0; bad = []
        for sgn in (1, -1):
            for q in np.arange(step, lim + 0.1, step):
                qs = [0.0] * s.J; qs[j] = math.radians(q); qs[j + 1] = sgn * math.radians(q)
                W = s.pose_frames(qs)
                v = float((A.transform(W[j][:3, :]) ^ C.transform(W[j + 2][:3, :])).volume())
                if v > 2.0: bad.append((sgn, q)); break
        return bad

# =============================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, default=ARM_LENGTH)
    ap.add_argument("--twist", type=float, default=TWIST)
    ap.add_argument("--joints", type=int, default=JOINTS)
    ap.add_argument("--range", type=float, default=RANGE)
    ap.add_argument("--ring-start", type=float, default=None)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out"))
    ap.add_argument("--root-servos", type=int, default=ROOT_SERVOS, help="joints from the root fitted with the STS3250")
    ap.add_argument("--no-sweep", action="store_true")
    a = ap.parse_args()
    arm = PinArm(a.length, a.twist, a.joints, a.range, a.ring_start)
    os.makedirs(os.path.join(a.out, "stl"), exist_ok=True)
    J, P = arm.J, arm.pitch
    print(f"OD {2*arm.R:.1f} mm, pitch {P:.1f} (min {arm.pitch_min:.1f}), {J} joints, ring start {arm.ring_start:.1f}, cap {arm.rho_cap:.1f}")

    parts = [arm.part(i) for i in range(J + 1)]
    lid = arm.lid()
    # print copies: socket, one moving vertebra (all identical, turned), tip, lid. Exported in joint-local frames.
    def local(m, j):   # undo joint j's placement so the part sits on its own joint axis at the origin
        M = np.eye(4); M[:3, :] = joint_affine(j, arm.twist, P); Mi = np.linalg.inv(M)
        return m.transform(Mi[:3, :])
    to_trimesh(local(parts[0], 0)).export(os.path.join(a.out, "stl", "root_socket_x1.stl"))
    if J >= 2: to_trimesh(local(parts[1], 0)).export(os.path.join(a.out, "stl", f"vertebra_x{J-1}.stl"))
    to_trimesh(local(parts[J], J - 1)).export(os.path.join(a.out, "stl", "tip_vertebra_x1.stl"))
    to_trimesh(lid).export(os.path.join(a.out, "stl", f"servo_lid_x{J}.stl"))
    # identical-part check: vertebra 2 equals vertebra 1 turned by TWIST and shifted one pitch
    same = None
    if J >= 3:
        Rz = np.eye(4); t = math.radians(arm.twist); Rz[:2, :2] = [[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]]; Rz[2, 3] = P
        d = parts[2] - parts[1].transform(Rz[:3, :])
        same = float(d.volume())

    g = lambda m: m.volume() * PETG_RHO * EFF_FILL
    m_sock, m_vert, m_tip, m_lid = g(parts[0]), g(parts[1]) if J >= 2 else 0.0, g(parts[J]), g(lid)
    # mass outside the mantle: moving vertebrae + tip + lids of servos 1..J-1 + servos 1..J-1 + hardware;
    # servo 0 and its lid sit in the socket inside the mantle
    nroot = max(0, min(a.root_servos, J))
    svs = [SERVO_ROOT if j < nroot else SERVO for j in range(J)]
    arm_out = (J - 1) * (m_vert + m_lid + HW_G) + sum(sv["mass"] for sv in svs[1:]) + m_tip + HW_G
    arm_all = arm_out + m_sock + m_lid + svs[0]["mass"]
    servo_txt = (f"{SERVO_ROOT['name']} in joints 0-{nroot-1}, {SERVO['name']} in joints {nroot}-{J-1}" if 0 < nroot < J
                 else f"{svs[0]['name']} in every joint")
    bodies = [arm.body(i) for i in range(J + 1)]
    # static interference with all joints at zero
    stat = []
    for i in range(J):
        stat.append(float((bodies[i] ^ bodies[i + 1]).volume()))
    sweeps, pairs = [], []
    if not a.no_sweep:
        for j in (0, 1, J - 1) if J > 2 else (0,):
            lo, hi = arm.joint_sweep(j, bodies)
            sweeps.append((j, lo, hi))
        for j in (0, 1) if J >= 3 else ():
            pairs.append((j, arm.pair_sweep(j, bodies)))
    # assemblies
    half = math.radians(a.range / 2)
    poses = {"straight": [0.0] * J,
             "curl": [half * 0.9] * J,
             "wave": [half * 0.9 * math.sin(2 * math.pi * k / J) for k in range(J)]}
    prev = {}
    for nm, qs in poses.items():
        W = arm.pose_frames(qs)
        ms = [to_trimesh(parts[i].transform(W[i][:3, :])) for i in range(J + 1)]
        ms += [to_trimesh(arm.servo_global(i).transform(W[i][:3, :])) for i in range(J)]
        trimesh.util.concatenate(ms).export(os.path.join(a.out, f"assembly_{nm}.stl"))
        prev[nm] = [W[i][:3, :].round(4).tolist() for i in range(J + 1)]
        prev[nm + "_tip"] = (W[J] @ np.array([0, 0, J * P, 1]))[:3].round(1).tolist()
    lp = []
    for i in (0, 1, J):
        t = to_trimesh(parts[i])
        lp.append(dict(i=i, v=np.round(t.vertices, 1).ravel().tolist(), f=t.faces.ravel().tolist()))
    json.dump(dict(params=dict(length=arm.length, twist=arm.twist, pitch=P, joints=J, OD=2 * arm.R, servo=servo_txt, root_servos=nroot),
                   frames=prev, meshes=lp), open(os.path.join(a.out, "preview.json"), "w"))
    # report
    L = [f"# Pin-hinge helix arm: fit and collision report\n",
         f"- Arm length {arm.length:.0f} mm from joint 0 to the tip face: {J} joints x {P:.1f} mm pitch (shortest that clears: {arm.pitch_min:.1f} mm). Hinge axes turn {arm.twist:.0f} deg per joint.",
         f"- Servos: {servo_txt} (same case, so every cradle is the same).",
         f"- Outer diameter {2*arm.R:.1f} mm, set by the fork ears and the screw heads on the hinge axis. The servo cradle alone would fit in {2*math.hypot(arm.floor_lo, arm.Wp/2+WALL):.1f} mm.",
         f"- Servo stack along the hinge axis: idler screw head {arm.head_lo:.1f}, idler ear {arm.iear_lo:.1f} to {arm.iear_hi:.1f}, cradle floor {arm.floor_lo:.1f}, servo case {arm.case_lo:.1f} to {arm.case_hi:.1f}, horn {arm.horn_lo:.1f} to {arm.horn_hi:.1f}, horn ear to {arm.hear_hi:.1f} mm.",
         f"- Printed parts: root socket {m_sock:.0f} g, vertebra {m_vert:.0f} g (x{J-1}, all identical), tip {m_tip:.0f} g, lid {m_lid:.1f} g (x{J}).",
         f"- Outside the mantle one arm weighs about {arm_out:.0f} g with servos and hardware ({arm_all:.0f} g counting the socket, root servo and its lid).",
         f"- Static interference with every joint at zero: {max(stat):.2f} mm^3 worst pair."]
    if same is not None:
        L.append(f"- Vertebra 2 minus vertebra 1 turned {arm.twist:.0f} deg: {same:.2f} mm^3, so one STL serves every moving vertebra.")
    if sweeps:
        L += ["", "Collision-free travel (5 deg steps, overlap < 2 mm^3, servo cases and horns included):", ""]
        for j, lo, hi in sweeps:
            ok = lo <= -a.range / 2 and hi >= a.range / 2
            L.append(f"- Joint {j}: {lo:.0f} to +{hi:.0f} deg ({'covers' if ok else 'SHORT of'} the +-{a.range/2:.0f} deg command range)")
        for j, bad in pairs:
            L.append(f"- Joints {j} and {j+1} turned together, link {j} against link {j+2}: " +
                     ("clear to +-60 deg in both senses" if not bad else "first contact at " + ", ".join(f"{'same' if sg > 0 else 'opposite'} sense {q:.0f} deg" for sg, q in bad)))
    L += ["", f"Tip, straight: {prev['straight_tip']} mm; curl (all joints {a.range/2*0.9:.0f} deg): {prev['curl_tip']} mm."]
    open(os.path.join(a.out, "fit_report.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))
    json.dump(dict(OD=2 * arm.R, pitch=P, pitch_min=arm.pitch_min, joints=J, m_vert=m_vert, m_lid=m_lid, m_tip=m_tip, m_sock=m_sock,
                   arm_out_g=arm_out, arm_all_g=arm_all, servos=servo_txt, root_servos=nroot, sweeps=sweeps, pairs=[(j, b) for j, b in pairs], static=max(stat)),
              open(os.path.join(a.out, "fit.json"), "w"))

if __name__ == "__main__":
    main()
