"""Build the full Centauri-1 scene for a Cycles render: Hippos-1 (chassis v4 shell, legs_v2 pantograph legs with
pauldron covers, actuators, belts, swing unit), the octopus (sk3d stand-in body + Cephalo-1 rev 4 arms from
arm_scale_study: front pair in the hands-meet pose, six raised with props), and the real tail CAD (hippos/tail.py +
tailmount.py) on its rump socket.  Geometry follows centaur_preview_build.py; only the tail is new.
EQUO_S=0.75 python3 render_scene.py [tail q0 q1 q2 q3 deg] -> scene.npz"""
import sys, os, io, contextlib
sys.path[:0] = ["/home/claude/octodrone", "/home/claude/octodrone/centaur", "/home/claude/octodrone/centaur/legs_v2",
                "/home/claude/octodrone/centaur/arms_scale", "/home/claude/octodrone/hippos"]
import numpy as np
import sk3d
with contextlib.redirect_stdout(io.StringIO()):
    import chassis_v3 as C3, chassis_v4 as C4, legs_v2_study as LS, arm_scale_study as AS
import tail as TL, tailmount as TM, sff_core as H
C = sk3d.COL
TAIL_Q = [float(v) for v in sys.argv[1:5]] if len(sys.argv) >= 5 else [-18.0, 12.0, 38.0, -10.0]
SOLO = os.environ.get("EQUO_SOLO", "0") == "1"

class Rec(sk3d.Scene):
    def __init__(s):
        super().__init__(); s.mat = "plastic_light"; s.mats = []
    def poly(s, pts, fc="#888888", alpha=1.0, ec=None, shade=True):
        s.polys.append(np.asarray(pts, float)); s.cols.append((fc if isinstance(fc, str) else "#%02x%02x%02x" % tuple(int(255 * x) for x in fc[:3]), alpha)); s.mats.append(s.mat)
sc = Rec()
S = C3.S
hipF, hipR = C3.HIPF, C3.HIPR
ZH = LS.H * S
# ---- body: girdle, receiver, packs (inside), shell + trim
PACK_X = -0.104
_zlo = C4.sections()[0]; PACK_Z = float(max(np.interp(xx, C4.XS, _zlo) for xx in np.linspace(PACK_X - 0.08, PACK_X + 0.08, 9))) + 0.054
C3.GIRDLE_W, C3.GIRDLE_Y, C3.PACK_Z, C3.KEEL_TAIL, C3.PACK_X = 0.118 * S, C3.Y_LEG_F - C3.ACT_OUT_IN - C3.HIPF_L, PACK_Z, 0.012, PACK_X
C3.DECK_X, C3.DECK_Z = PACK_X, PACK_Z + 0.037
sc.mat = "metal_dark"; C3.girdle(sc)
sc.mat = "shell"; C4.shell(sc, rings=False); sc.mat = "trim"; C4.shell_trim(sc)
# ---- legs
B = LS.ARCH["B"]; q = B.ik(0.0, -LS.H); P = {k: np.asarray(v) * S for k, v in LS.panto_points(B, q).items()}
def leg(xh, side, front):
    sgn = -1.0 if front else 1.0
    y_leg = C3.Y_LEG_F if front else C3.Y_LEG_R; y = side * y_leg
    W = lambda p: np.array([xh + sgn * p[0], y, ZH + p[1]])
    hip, st, hk, fe, ft = (W(P[k]) for k in ("hip", "stifle", "hock", "fet", "foot"))
    Pp, Qq = W(P["P"]), W(P["Q"])
    sc.mat = "metal_dark"
    hipact = C3.ACT[C3.SEL[("front" if front else "rear", "hip")]][:3]
    yin = side * (y_leg - C3.ACT_OUT_IN - hipact[2] / 2)
    sc.cyl([hip[0], yin - side * hipact[2] / 2, hip[2]], [hip[0], yin + side * hipact[2] / 2, hip[2]], hipact[1], C["act"], n=32)
    if front and C3.ABD_MODE == "tube":
        yt = side * C3.TT_Y
        sc.cyl([C3.TT_ACT_X0, yt, C3.TT_Z], [C3.TT_ACT_X1, yt, C3.TT_Z], C3.ABD_R, "#3a3d42", n=32)
        sc.mat = "steel"; sc.cyl([C3.TT_ACT_X1, yt, C3.TT_Z], [C3.TT_X1, yt, C3.TT_Z], C3.TT_R, "#8a9099", n=16); sc.mat = "metal_dark"
    else:
        adz = C3.ABD_DZ if front else C3.ABD_DZ_R; ad = C3.ABD_DIR_F if front else C3.ABD_DIR_R
        abd_in = C3.ABD_IN if front else C3.ABD_IN_R
        a_r, a_l = (C3.ABD_R, C3.ABD_X1 - C3.ABD_X0) if front else (C3.ABDR_R, C3.ABDR_L)
        sc.cyl([hip[0] + ad * C3.ABD_X0, side * (y_leg - abd_in), hip[2] + adz], [hip[0] + ad * (C3.ABD_X0 + a_l), side * (y_leg - abd_in), hip[2] + adz], a_r, "#3a3d42", n=32)
    dfem = (st - hip) / np.linalg.norm(st - hip)
    Y_BELT = y_leg + 0.040
    kn = hip - dfem * C3.R_K; kn = np.array([kn[0], side * y_leg, kn[2]])
    sc.cyl(kn - [0, side * 0.0245, 0], kn + [0, side * 0.0245, 0], 0.044, C["act"], n=32)
    sc.mat = "plastic_blue"; sc.cyl(np.array([hip[0], side * y_leg, hip[2]]), kn, 0.016, C["horse"], n=12)
    sc.mat = "alu"
    p1 = np.array([kn[0], side * Y_BELT, kn[2]]); p2 = np.array([st[0], side * Y_BELT, st[2]])
    sc.cyl(p1 - [0, 0.0125, 0], p1 + [0, 0.0125, 0], 0.0239, "#b0b6bd", n=24); sc.cyl(p2 - [0, 0.0125, 0], p2 + [0, 0.0125, 0], 0.0478, "#b0b6bd", n=32)
    sc.mat = "rubber"
    d = p2 - p1; d = d / np.linalg.norm(d); n_ = np.array([-d[2], 0, d[0]])
    for sgn2 in (1, -1): sc.cyl(p1 + sgn2 * n_ * 0.0245, p2 + sgn2 * n_ * 0.0485, 0.004, C["belt"], n=6)
    sc.mat = "plastic_blue"
    sc.cyl(hip, st, 0.020, C["horse"], n=14, r1=0.017); sc.cyl(st, hk, 0.011, C["horse"], n=12)
    sc.sphere(st, 0.024, C["horse"], n=8); sc.sphere(hk, 0.016, C["horse"], n=6); sc.cyl(hk, fe, 0.011, C["horse"], n=12)
    sc.cyl(fe, ft + [0, 0, 0.02], 0.009, C["horse"], n=10)
    sc.mat = "shell"; C4.leg_cover(sc, hip, st, hk, fe, side)
    sc.mat = "rubber_tpu"; sc.cyl(ft, ft + [0, 0, 0.032], 0.034, C["tpu"], n=24, r1=0.026)
    sc.mat = "carbon"
    for dy in (-0.012, 0.012): sc.cyl(Pp + [0, dy, 0], Qq + [0, dy, 0], 0.003, C["latch"], n=6)
for s_ in (1, -1):
    leg(hipF[0], s_, True); leg(hipR[0], s_, False)
# ---- octopus: stand-in body + real arms
ARM_PROPS = []
if not SOLO:
    sc.mat = "octo"
    n0 = len(sc.polys)
    sc.octopus(base=(0, 0, 0), yaw=45, bars=0.194, arms="none")
    R = sk3d.rot("y", C3.TILT)
    for k in range(n0, len(sc.polys)): sc.polys[k] = sc.polys[k] @ R.T + np.array([C3.cc[0], 0, C3.cc[1]])
    with contextlib.redirect_stdout(io.StringIO()):
        T, Hh = AS.tasks(S)
    bp, bR = Hh["oct_p"], Hh["oct_R"]
    def real_arm(az, qq):
        pts, hand, Rr = AS.arm_fk(az, qq, bp, bR)
        Pc, rc = AS.capsules(pts, n=40)
        sc.mat = "octo_arm"
        for a_, b_, ra, rb in zip(Pc[:-1], Pc[1:], rc[:-1], rc[1:]): sc.cyl(a_, b_, ra, C["octo2"], n=16, r1=rb, caps=False)
        sc.mat = "metal_dark"
        for j in range(1, 9): sc.sphere(pts[j], 0.026 if j < 5 else 0.022, C["act"], n=8)
        return pts, hand, Rr
    q4 = T["T4 hands meet"]["q"]
    real_arm(AS.AZ_R, q4); real_arm(AS.AZ_L, q4 * np.array([1, -1, 1, -1, 1, -1, 1, -1]))
    Q_GUARD = np.radians([-35, 0, -45, 0, 0, 0, 0, 0])
    for i in range(1, 7):
        pts, hand, Rr = real_arm(AS.AZ_R + 45 * i, Q_GUARD)
        ARM_PROPS.append((pts[-1] + Rr[:, 0] * 0.036, Rr[:, 0]))
        sc.mat = "prop"; sc.disc(pts[-1] + Rr[:, 0] * 0.036, 0.0762, Rr[:, 0], C["octo"], 0.28, n=40)
# ---- tail: real CAD.  Connector frame -> horse: x_c = up, y_c = right (-y_h), z_c = forward (+x_h); mouth on the rump surface
z_seat = 0.60 * S
t = C3.hits(np.array([0.0, z_seat]), np.array([1.0, 0.0])); x_rump = t[0]
M = np.array([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])            # columns: images of x_c, y_c, z_c in the horse frame
ORG = np.array([x_rump, 0.0, z_seat])
def place_c(m):
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3] / 1000.0; f = np.asarray(me.tri_verts)
    return v @ M.T + ORG, f
MESHES = []
def add(m, mat, T4=None):
    if T4 is not None: m = TL.apply(m, T4)
    v, f = place_c(m); MESHES.append((mat, v.astype(np.float32), f.astype(np.int32)))
parts = TL.parts(); Ts = TL.pose_transforms(TAIL_Q)
add(TM.rump_socket(), "pa_cf"); add(H.pin_board(), "pcb"); add(H.hand_base(), "petg_blue"); add(TM.tail_adapter(), "petg_blue")
add(parts["root_cradle"], "pa_cf"); add(TL.servo_env(0), "servo")
for k in range(4):
    add(parts[f"link{k}"], "petg_light", Ts[k])
    if k < 3: add(TL.servo_env(k + 1), "servo", Ts[k])
add(parts["tip_socket"], "pa_cf", Ts[3])
# ---- triangulate the sk3d polygons, grouped by material + colour + alpha
groups = {}
for pts, (fc, a), mat in zip(sc.polys, sc.cols, sc.mats):
    if len(pts) < 3: continue
    key = (mat, fc, round(float(a), 2)); g = groups.setdefault(key, [])
    for i in range(1, len(pts) - 1): g.append(np.r_[pts[0], pts[i], pts[i + 1]])
out = {}
for i, ((mat, fc, a), tris) in enumerate(groups.items()):
    out[f"g{i}"] = np.asarray(tris, np.float32).reshape(-1, 3)
    out[f"g{i}_meta"] = np.array([mat, fc, str(a)])
for i, (mat, v, f) in enumerate(MESHES):
    out[f"m{i}_v"] = v; out[f"m{i}_f"] = f; out[f"m{i}_meta"] = np.array([mat])
np.savez_compressed("scene_solo.npz" if SOLO else "scene.npz", **out)
ntri = sum(len(g) for g in groups.values()) + sum(len(f) for _, _, f in MESHES)
print(f"groups {len(groups)}, tail meshes {len(MESHES)}, triangles {ntri}, rump x {x_rump:.3f}, tail q {TAIL_Q}")
