#!/usr/bin/env python3
"""
arm_cad.py  --  parametric canted-axis hinge tentacle for Cephalo-1 (main arm design)

One bus servo per vertebra. Each joint turns about a hinge axis tilted CANT degrees from the
transverse plane toward the arm axis, so every bend also twists; successive axes alternate 90 deg
about the arm. The two vertebrae of a joint meet on a flat face normal to the hinge axis: the
servo sits in the proximal vertebra with its spline on the hinge axis, its horn bolted to the
distal vertebra, and a PTFE washer ring on a raised bearing land takes the face load.

  python3 arm_cad.py                         # defaults below
  python3 arm_cad.py --length 900 --cant 30  # any arm length / cant; node count follows
  python3 arm_cad.py --nodes 12 --pitch 80     # 12 moving vertebrae (plus the root socket)

Writes out/: one STL per vertebra and retainer, straight and curled assembly STLs, a fit
and collision report (fit_report.md) and preview.json for the results page.
Requires numpy, manifold3d, trimesh (python3 -m pip install numpy manifold3d trimesh). Units mm.
Frames: arm axis z (root -> tip), x up when mounted, joint 0 axis in the y-z plane.
"""
import argparse, json, math, os
import numpy as np
from manifold3d import Manifold, set_circular_segments
import trimesh

# =============================================================================
# PARAMETERS  ([A] = assumed, check against the part in hand)
# =============================================================================
ARM_LENGTH = 585.0        # root face to tip face. Forest says the real arm is longer: pass --length
CANT = 45.0               # deg, hinge axis tilt from the transverse plane toward the arm axis
RANGE = 120.0             # deg total joint travel the servo is commanded over (+-60)
PITCH = None              # mm per vertebra; None = shortest pitch that the fit check passes
N_NODES = None            # None = ARM_LENGTH // PITCH

SERVO = dict(name="STS3215", L=45.2, W=24.7, H=35.0,   # body; H along the output shaft
             spline_from_end=11.0,                     # [A] spline centre from the near end along L
             shaft_above_top=4.0)                      # [A] spline + horn hub above the top face
HORN_PCD = 14.0           # [A] horn bolt circle (4 x M3); measure your horn
HORN_T = 3.0              # [A] horn disc thickness
CLEAR = 0.5               # servo -> pocket clearance per side
WALL = 1.8                # shell wall (3 perimeters at 0.6 mm extrusion width)
FACE_GAP = 1.0            # gap between joint faces (PTFE washer 0.5 mm + running clearance)
LAND_W = 4.0              # bearing land radial width
LAND_H = 0.4              # bearing land height above the face (PTFE washer rides on it)
FACE_T = 3.2              # joint face plate thickness
WINDOWS = 4               # lightening windows cut in the skin of each vertebra (0 = closed skin)
RIB_W = 9.0               # width of the skin ribs left between windows
INSERT_D, INSERT_L = 4.0, 5.5   # M3 heat-set insert hole
M3_CLEAR = 3.3
CABLE_D = 6.0             # servo bus cable hole through each joint face
CABLE_OFF = 18.0          # cable hole centre from the hinge axis, on the side away from the servo body
ROOT_PCD_FRAC = 0.62      # root flange bolt circle as a fraction of OD
PETG_RHO = 1.27e-3        # g/mm^3
EFF_FILL = 0.95           # printed mass / solid mass: shells print near solid at 4 perimeters [A]
SEG = 72
set_circular_segments(SEG)

# =============================================================================
def unit(v): v = np.asarray(v, float); return v / np.linalg.norm(v)

def hinge_axis(k, cant):
    """joint k axis in its vertebra's frame: transverse y (even k) or x (odd k), tilted toward +z"""
    c = math.radians(cant)
    base = np.array([0.0, 1.0, 0.0]) if k % 2 == 0 else np.array([1.0, 0.0, 0.0])
    return unit(base * math.cos(c) + np.array([0, 0, math.sin(c)]))

def frame_from_axis(a):
    """orthonormal (u, v, a) with u the in-plane direction closest to +z (the 'tilt' direction)"""
    z = np.array([0, 0, 1.0])
    u = z - a * (a @ z)
    u = unit(u) if np.linalg.norm(u) > 1e-6 else unit(np.cross(a, [1, 0, 0]))
    v = np.cross(a, u)
    return u, v, a

def affine(R, t):
    M = np.zeros((3, 4)); M[:, :3] = R; M[:, 3] = t; return M

def place_box(size, center, R):
    return Manifold.cube(list(size), True).transform(affine(R, center))

def cyl_along(a, p0, length, r, extra=0.0):
    """cylinder of radius r from p0 along unit a for length (plus extra both ends)"""
    u, v, aa = frame_from_axis(unit(a))
    R = np.column_stack([u, v, aa])
    return Manifold.cylinder(length + 2 * extra, r, r, SEG).transform(affine(R, p0 - aa * extra))

def rot_about_line(p, a, q):
    a = unit(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    R = np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p - R @ p
    return T

def T_of(M4):
    return M4[:3, :]

# =============================================================================
class Arm:
    def __init__(s, length, cant, pitch=None, nodes=None, servo=SERVO):
        s.length, s.cant, s.sv = length, cant, servo
        Ls, Ws, Hs = servo["L"] + 2 * CLEAR, servo["W"] + 2 * CLEAR, servo["H"] + 2 * CLEAR
        s.pocket = (Ls, Ws, Hs)
        s.OD = s._min_od()
        s.R = s.OD / 2
        s.N = 10 ** 6
        s.pitch_min = s._min_pitch()
        # vertebra 0 is the root socket that sits in the mantle and holds the first servo, so the arm
        # has N - 1 moving vertebrae and N - 1 joints; the arm length runs from the socket face to the tip
        moving = nodes or max(1, int(length // s.pitch_min))
        s.pitch = pitch or max(s.pitch_min, length / moving)
        s.N = moving + 1
        s.length = moving * s.pitch

    # ---- servo placement inside vertebra k (local frame, proximal face crosses the axis at z=0)
    def servo_pose(s, k, pitch=None):
        P = pitch or getattr(s, "pitch", 0)
        a = hinge_axis(k, s.cant); u, v, _ = frame_from_axis(a)
        Ls, Ws, Hs = s.pocket
        # body centred on the arm axis in the plane normal to a; spline offset along u
        off = (Ls / 2 - (s.sv["spline_from_end"] + CLEAR))
        top = s.sv["shaft_above_top"] + HORN_T + FACE_GAP / 2      # spline top face -> joint plane
        # joint plane crosses the arm axis at z = P: centre c = (0,0,zc) and plane point c + (Hs/2+top) a
        zc = P - (Hs / 2 + top) / a[2] if a[2] > 1e-6 else P - Hs / 2 - top
        c = np.array([0, 0, zc])
        spline = c + u * off + a * (Hs / 2)             # where the hinge axis leaves the servo top
        # hinge axis line: through the spline along a; joint plane: normal a through spline + top*a
        Q = spline + a * top
        return dict(a=a, u=u, v=v, c=c, R=np.column_stack([u, v, a]), spline=spline, Q=Q)

    def _min_od(s):
        Ls, Ws, Hs = s.pocket
        best = 1e9
        a = hinge_axis(0, s.cant); u0, v0, _ = frame_from_axis(a)
        for phi in np.radians(np.arange(0, 180, 2.5)):
            u = math.cos(phi) * u0 + math.sin(phi) * v0; v = np.cross(a, u)
            P = np.array([x * u + y * v + z * a for x in (-Ls/2, Ls/2) for y in (-Ws/2, Ws/2) for z in (-Hs/2, Hs/2)])
            best = min(best, float(np.hypot(P[:, 0], P[:, 1]).max()))
        return 2 * (best + WALL)

    def _min_pitch(s):
        for P in np.arange(30, 200, 1.0):
            s.pitch = P
            if s.fit(0, quick=True)["servo_outside_mm3"] < 1.0 and s.fit(1, quick=True)["servo_outside_mm3"] < 1.0:
                return float(P)
        raise RuntimeError("no pitch up to 200 mm fits the servo")

    # ---- planes
    def prox_plane(s, k):
        if k == 0: return np.array([0, 0, 1.0]), np.zeros(3)
        prev = s.servo_pose(k - 1)
        return prev["a"], prev["Q"] - np.array([0, 0, s.pitch])

    def dist_plane(s, k):
        if k == s.N - 1: return np.array([0, 0, 1.0]), np.array([0, 0, s.pitch])
        p = s.servo_pose(k); return p["a"], p["Q"]

    def hull(s, k, shrink=0.0):
        n0, p0 = s.prox_plane(k); n1, p1 = s.dist_plane(k)
        body = Manifold.cylinder(4 * s.pitch + 4 * s.OD, s.R - shrink, s.R - shrink, SEG).translate([0, 0, -2 * s.pitch - 2 * s.OD])
        body = body.trim_by_plane(list(n0), float(n0 @ p0) + FACE_GAP / 2 + shrink)
        body = body.trim_by_plane(list(-n1), float(-n1 @ p1) + FACE_GAP / 2 + shrink)
        # keep only the piece that holds the arm axis between the faces
        parts = body.decompose()
        if len(parts) > 1:
            mid = np.array([0, 0, s.pitch / 2])
            parts.sort(key=lambda m: -m.volume()); body = parts[0]
        return body

    def servo_box(s, k):
        p = s.servo_pose(k)
        return place_box(s.pocket, p["c"], p["R"])

    def fit(s, k, quick=False):
        last = k == s.N - 1
        inner = s.hull(k, shrink=WALL)
        out = {}
        if not last:
            sb = s.servo_box(k)
            out["servo_outside_mm3"] = float((sb - inner).volume())
        else:
            out["servo_outside_mm3"] = 0.0
        if quick: return out
        # bearing land: largest disc on the distal face, centred on the hinge axis, inside the face
        if not last:
            p = s.servo_pose(k); n1, p1 = s.dist_plane(k)
            rmax = 0.0
            for r in np.arange(4, s.R, 0.5):
                ang = np.linspace(0, 2 * math.pi, 48, endpoint=False)
                pts = p["Q"][None, :] + r * (np.cos(ang)[:, None] * p["u"] + np.sin(ang)[:, None] * p["v"])
                inside = np.hypot(pts[:, 0], pts[:, 1]) <= s.R - 0.5
                # the disc must also stay on the distal side of this vertebra's proximal face
                n0, p0 = s.prox_plane(k)
                inside &= (pts @ n0 - n0 @ p0) >= FACE_GAP
                if inside.all(): rmax = r
                else: break
            out["land_r_max_mm"] = rmax
            out["spline_offset_mm"] = float(np.hypot(*p["spline"][:2]))
        return out

    # ---- solid parts
    def vertebra(s, k):
        """hollow shell: wall + face plates, a sleeve round the servo pocket, insert bosses, horn boss"""
        hull = s.hull(k)
        last = k == s.N - 1
        n0, p0 = s.prox_plane(k); n1, p1 = s.dist_plane(k)
        inner = Manifold.cylinder(4 * s.pitch + 4 * s.OD, s.R - WALL, s.R - WALL, SEG).translate([0, 0, -2 * s.pitch - 2 * s.OD])
        inner = inner.trim_by_plane(list(n0), float(n0 @ p0) + FACE_GAP / 2 + (WALL + 2 if k == 0 else FACE_T))
        inner = inner.trim_by_plane(list(-n1), float(-n1 @ p1) + FACE_GAP / 2 + (WALL + 2 if last else FACE_T))
        body = hull - inner
        if WINDOWS:
            # windows through the skin only (not the sleeve or faces); ribs stay along the arm
            skin_zone = (Manifold.cylinder(4 * s.pitch + 4 * s.OD, s.R + 1, s.R + 1, SEG).translate([0, 0, -2 * s.pitch - 2 * s.OD])
                         .trim_by_plane(list(n0), float(n0 @ p0) + FACE_GAP / 2 + FACE_T + 4)
                         .trim_by_plane(list(-n1), float(-n1 @ p1) + FACE_GAP / 2 + FACE_T + 4))
            skin_zone -= Manifold.cylinder(4 * s.pitch + 4 * s.OD, s.R - WALL - 0.01, s.R - WALL - 0.01, SEG).translate([0, 0, -2 * s.pitch - 2 * s.OD])
            w = 2 * math.pi * s.R / WINDOWS - RIB_W
            for i in range(WINDOWS):
                cut = Manifold.cube([s.R, w, 4 * s.pitch], True).translate([s.R, 0, 0.5 * s.pitch])
                body -= cut.rotate([0, 0, (i + 0.5) * 360.0 / WINDOWS]) ^ skin_zone
        if not last:
            p = s.servo_pose(k); a, u, v = p["a"], p["u"], p["v"]
            Ls, Ws, Hs = s.pocket
            sleeve = place_box((Ls + 2 * WALL, Ws + 2 * WALL, Hs + WALL + 40), p["c"] + a * (20 - WALL / 2), p["R"]) ^ hull
            ins = []
            for sx in (-1, 1):
                for sy in (-1, 1):
                    ins.append(p["Q"] + u * sx * (Ls / 2 + 3.0) + v * sy * (Ws / 2 + 3.0))
            for h in ins:
                sleeve += cyl_along(-a, h - a * 0.4, 11, 4.2) ^ hull
            body += sleeve
            body -= place_box((Ls, Ws, Hs + 60), p["c"] + a * 30, p["R"])          # pocket, open to the face
            body -= place_box((Ls + 12.5, Ws + 12.5, 4.9), p["Q"] - a * 0.45, p["R"])   # retainer recess, 2.4 deep
            for h in ins:
                body -= cyl_along(-a, h, INSERT_L + 2.9, INSERT_D / 2, 0.5)
            fr = s.fit(k)["land_r_max_mm"]
            if fr > LAND_W + 2:
                ring = cyl_along(a, p["Q"] - a * (FACE_GAP / 2 + 0.01), LAND_H, fr) - cyl_along(a, p["Q"] - a * 2, 4, fr - LAND_W)
                body += ring ^ Manifold.cylinder(4 * s.pitch, s.R, s.R, SEG).translate([0, 0, -s.pitch])
            body -= cyl_along(a, p["Q"] + u * CABLE_OFF - a * 12, 24, CABLE_D / 2)
        else:
            for i in range(4):
                t = math.pi / 4 + i * math.pi / 2; r = 0.32 * s.OD
                boss = Manifold.cylinder(9, 4.5, 4.5, 24).translate([r * math.cos(t), r * math.sin(t), s.pitch - FACE_GAP / 2 - 9])
                body += boss ^ hull
                body -= Manifold.cylinder(INSERT_L + 0.5, INSERT_D / 2, INSERT_D / 2, 24).translate([r * math.cos(t), r * math.sin(t), s.pitch - INSERT_L])
            body -= Manifold.cylinder(20, CABLE_D / 2, CABLE_D / 2, 24).translate([0, 0, s.pitch - 15])
        if k == 0:
            for i in range(4):
                t = math.pi / 4 + i * math.pi / 2; r = ROOT_PCD_FRAC * s.R
                body -= Manifold.cylinder(14, M3_CLEAR / 2, M3_CLEAR / 2, 24).translate([r * math.cos(t), r * math.sin(t), -1])
            body -= Manifold.cylinder(14, CABLE_D / 2, CABLE_D / 2, 24).translate([0, 0, -1])
        else:
            prev = s.servo_pose(k - 1)
            a, u, v = prev["a"], prev["u"], prev["v"]
            Qk = prev["Q"] - np.array([0, 0, s.pitch])
            # horn boss: closes the face gap down onto the horn, which sits flush with the previous face
            body += cyl_along(a, Qk - a * (FACE_GAP / 2), FACE_GAP / 2 + FACE_T + 0.5, 9.0)
            for i in range(4):
                t = i * math.pi / 2
                h = Qk + (HORN_PCD / 2) * (math.cos(t) * u + math.sin(t) * v)
                body -= cyl_along(a, h - a * 1, FACE_T + 3, M3_CLEAR / 2)
            body -= cyl_along(a, Qk - a * 1, FACE_T + 3, 2.0)                   # horn centre screw
            body -= cyl_along(a, Qk + u * CABLE_OFF - a * 1, FACE_T + 3, CABLE_D / 2)
        return body

    def retainer(s, k):
        """flat plate that clamps the servo top into its pocket; prints flat, sits in the recess"""
        p = s.servo_pose(k); Ls, Ws, _ = s.pocket
        plate = Manifold.cube([Ls + 12.0, Ws + 12.0, 2.2], True)
        # opening for the servo output boss (spline end) and the horn
        sx = Ls / 2 - (s.sv["spline_from_end"] + CLEAR)
        plate -= Manifold.cylinder(6, 11.0, 11.0, SEG).translate([sx, 0, -3])   # horn (20 mm) and boss (18 mm) turn inside
        for ix in (-1, 1):
            for iy in (-1, 1):
                plate -= Manifold.cylinder(6, M3_CLEAR / 2, M3_CLEAR / 2, 24).translate([ix * (Ls / 2 + 3.0), iy * (Ws / 2 + 3.0), -3])
        return plate

    # ---- kinematics and collision
    def joint_T(s, k, q):
        p = s.servo_pose(k)
        T = rot_about_line(p["spline"], p["a"], q)
        Tz = np.eye(4); Tz[2, 3] = s.pitch
        return T @ Tz

    def pose_frames(s, qs):
        W = [np.eye(4)]
        for k in range(s.N - 1):
            W.append(W[-1] @ s.joint_T(k, qs[k]))
        return W

    def reach_sweep(s, k, parts, step=10.0):
        """vertebra k against k+2 with joints k and k+1 turned together: how far the arm can curl
        before two non-adjacent vertebrae touch"""
        res = []
        for qd in np.arange(0, 180.1, step):
            for sgn in (1, -1):
                q = math.radians(qd * sgn)
                T = s.joint_T(k, q) @ s.joint_T(k + 1, q)
                vol = float((parts[k] ^ parts[k + 2].transform(T_of(T))).volume())
                res.append((qd * sgn, vol))
        lim = {1: 180.0, -1: -180.0}
        for sgn in (1, -1):
            for qd, vol in sorted([r for r in res if r[0] * sgn >= 0], key=lambda r: abs(r[0])):
                if vol >= 2.0: lim[sgn] = qd - sgn * step; break
        return lim[-1], lim[1]

    def collision_sweep(s, k, parts, step=5.0):
        """overlap volume between vertebra k and k+1 as joint k turns; returns free range (deg)"""
        A = parts[k]; res = []
        for qd in np.arange(-180, 180.1, step):
            B = parts[k + 1].transform(T_of(s.joint_T(k, math.radians(qd))))
            res.append((qd, float((A ^ B).volume())))
        free = [qd for qd, vol in res if vol < 2.0]
        lo = hi = 0.0
        for qd, vol in sorted(res, key=lambda r: abs(r[0])):
            pass
        # contiguous free band around 0
        qs = [r[0] for r in res]; ok = {r[0]: r[1] < 2.0 for r in res}
        i0 = min(range(len(qs)), key=lambda i: abs(qs[i]))
        if not ok[qs[i0]]: return 0.0, 0.0, res
        i = i0
        while i + 1 < len(qs) and ok[qs[i + 1]]: i += 1
        hi = qs[i]; i = i0
        while i - 1 >= 0 and ok[qs[i - 1]]: i -= 1
        lo = qs[i]
        return lo, hi, res

# =============================================================================
def to_trimesh(m):
    mm = m.to_mesh(); return trimesh.Trimesh(np.asarray(mm.vert_properties)[:, :3], np.asarray(mm.tri_verts), process=False)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--length", type=float, default=ARM_LENGTH)
    ap.add_argument("--cant", type=float, default=CANT)
    ap.add_argument("--pitch", type=float, default=PITCH)
    ap.add_argument("--nodes", type=int, default=N_NODES)
    ap.add_argument("--range", type=float, default=RANGE)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out"))
    ap.add_argument("--no-sweep", action="store_true")
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, "stl"), exist_ok=True)
    arm = Arm(a.length, a.cant, a.pitch, a.nodes)
    parts = [arm.vertebra(k) for k in range(arm.N)]
    rows, mass = [], 0.0
    for k, m in enumerate(parts):
        to_trimesh(m).export(os.path.join(a.out, "stl", "root_socket.stl" if k == 0 else f"vertebra_{k:02d}.stl"))
        f = arm.fit(k); g = m.volume() * PETG_RHO * EFF_FILL; mass += g
        rows.append(dict(k=k, mass_g=g, **f))
    to_trimesh(arm.retainer(0)).export(os.path.join(a.out, "stl", "servo_retainer_x%d.stl" % (arm.N - 1)))
    # collision sweep per joint
    sweeps = []
    if not a.no_sweep:
        for k in range(min(arm.N - 1, 3)):           # joints repeat with parity; 0..2 cover root, even, odd
            lo, hi, res = arm.collision_sweep(k, parts)
            r2 = arm.reach_sweep(k, parts) if k + 2 < arm.N else (None, None)
            sweeps.append(dict(joint=k, free_lo=lo, free_hi=hi, pair_lo=r2[0], pair_hi=r2[1]))
    # assemblies
    poses = {"straight": [0.0] * (arm.N - 1),
             "curl": [math.radians(a.range / 2 * 0.9) * (1 if k % 2 == 0 else 0.0) for k in range(arm.N - 1)],
             "coil": [math.radians(a.range / 2 * 0.9)] * (arm.N - 1)}
    prev = {}
    for nm, qs in poses.items():
        W = arm.pose_frames(qs)
        asm = Manifold.batch_boolean([p.transform(T_of(W[k])) for k, p in enumerate(parts)], __import__("manifold3d").OpType.Add) if False else None
        meshes = [to_trimesh(p.transform(T_of(W[k]))) for k, p in enumerate(parts)]
        trimesh.util.concatenate(meshes).export(os.path.join(a.out, f"assembly_{nm}.stl"))
        prev[nm] = [W[k][:3, :].round(4).tolist() for k in range(arm.N)]
        prev[nm + "_tip"] = (W[-1] @ np.array([0, 0, arm.pitch, 1]))[:3].round(1).tolist()
    # low-poly vertebra meshes for the page preview
    lp = []
    for k in (0, 1, 2, arm.N - 1):
        t = to_trimesh(parts[k])
        try: t = t.simplify_quadric_decimation(face_count=1500)
        except Exception: pass
        lp.append(dict(k=k, v=np.round(t.vertices, 1).ravel().tolist(), f=t.faces.ravel().tolist()))
    json.dump(dict(params=dict(length=arm.length, cant=arm.cant, pitch=arm.pitch, N=arm.N, OD=arm.OD, servo=arm.sv["name"]),
                   frames=prev, meshes=lp), open(os.path.join(a.out, "preview.json"), "w"))
    # report
    servo_g = 55.0 * (arm.N - 1)
    arm_mass = mass - rows[0]["mass_g"] + servo_g + 15 * (arm.N - 1)
    L = [f"# Canted-hinge arm: fit and collision report\n",
         f"- Arm length {arm.length:.0f} mm from the socket face: {arm.N - 1} moving vertebrae x {arm.pitch:.1f} mm pitch (shortest that fits: {arm.pitch_min:.0f} mm), plus the root socket in the mantle. Cant {arm.cant:.0f} deg, {arm.N - 1} joints, {arm.N - 1} x {arm.sv['name']}.",
         f"- Vertebra outer diameter {arm.OD:.1f} mm (servo pocket + {WALL} mm wall).",
         f"- Printed mass {mass:.0f} g per arm including the {rows[0]['mass_g']:.0f} g socket (PETG). Outside the mantle the arm weighs about {arm_mass:.0f} g with servos and hardware.",
         "", "| Part | Mass g | Servo outside pocket mm^3 | Bearing land max radius mm | Hinge axis offset from centreline mm |", "|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {'root socket' if r['k'] == 0 else 'vertebra ' + str(r['k'])} | {r['mass_g']:.0f} | {r['servo_outside_mm3']:.1f} | {r.get('land_r_max_mm', 0):.1f} | {r.get('spline_offset_mm', 0):.1f} |")
    if sweeps:
        L += ["", "Collision-free travel of each joint (5 deg steps, adjacent vertebrae, overlap < 2 mm^3):", ""]
        for sw in sweeps:
            ok = sw["free_lo"] <= -a.range / 2 and sw["free_hi"] >= a.range / 2
            L.append(f"- Joint {sw['joint']}: {sw['free_lo']:.0f} to +{sw['free_hi']:.0f} deg against its neighbour ({'covers' if ok else 'SHORT of'} the +-{a.range/2:.0f} deg command range)"
                     + (f"; with joints {sw['joint']} and {sw['joint']+1} turned together, vertebrae {sw['joint']} and {sw['joint']+2} stay clear from {sw['pair_lo']:.0f} to +{sw['pair_hi']:.0f} deg" if sw['pair_lo'] is not None else ""))
    L += ["", f"Tip position, straight: {prev['straight_tip']} mm; curl pose: {prev['curl_tip']} mm; coil pose: {prev['coil_tip']} mm."]
    open(os.path.join(a.out, "fit_report.md"), "w").write("\n".join(L) + "\n")
    print("\n".join(L))

if __name__ == "__main__":
    main()
