"""Centauri-1 mesh scene for the Cycles render: every part that has CAD is drawn from its CAD solid.
  * Cephalo-2 (octopus), from the project CAD as cephalo2_model.py builds it: turret shell with 5 camera windows
    (turretcams), plate, scabbard cage, rotor motor, latch pins, camera bays (boards, OV5640 modules, heat sinks, cradles,
    zenith bridge, bezels, PC discs), arm portholes; the belly rotor in its "bars" mode (blades hanging, slid down
    108 mm into the neck receiver), elevator and fold rods to match; 8 rev 4 arms (servos in their cradles, clutch cams +
    rockers, idler bosses, link forks, tip bays); on every tip the connector (arm socket + hand base, handmount/SFF CAD);
    hand3 (hub C2) on all eight arms (Cephalo-2 baseline): the front pair (arms 0 and 7) grasp-ready as the
    centaur's hands, the six raised arms as fans (fingers straight = the 9 in four-blade rotor, parked).
  * Hippos-1 tail: tailmount.py rump socket + adapter, then one whole Cephalo-2 arm + hand3 (hippos/tailarm/tailarm.py);
    TAIL_POSE picks a pose from tailarm_checks.json, NO_TAIL=1 writes scene2_notail.npz (used by the tail checks).
  * Hippos-1 body and legs: there is no CAD yet, so these are the full-model preview geometry (chassis v4 shell and
    trim, girdle, legs_v2 pantograph legs, pauldron covers, actuators, belts, hooves), as in render_scene.py.
Arm poses for hand3 on every arm (2026-11-16): front pair "T4 hands meet" re-solved with the real hand (grip centre,
static servo load <= 80 % of rated, hands >= 10 mm apart); the six raised as fans at (-35, -85, 90, -50) (91 % of rated,
swept discs 17.7 mm apart).  The old Q_GUARD (-35, -45, 0, 0) held j0 at 102 % with hand3.
EQUO_S=0.75 python3 render_scene2.py -> scene2.npz"""
import sys, os, io, contextlib
sys.path[:0] = ["/home/claude/octodrone/ceph2", "/home/claude/octodrone", "/home/claude/octodrone/centaur", "/home/claude/octodrone/centaur/legs_v2",
                "/home/claude/octodrone/centaur/arms_scale", "/home/claude/octodrone/hippos"]
import numpy as np
import trimesh
import sk3d
with contextlib.redirect_stdout(io.StringIO()):
    import chassis_v3 as C3, chassis_v4 as C4, legs_v2_study as LS, arm_scale_study as AS
import tail as TL, tailmount as TM, sff_core as SFF
os.chdir("/home/claude/octodrone/ceph2")                     # xlclaw_cad cache lives there
import cephalo4 as CE, xlpod4, viz4_arm as VA, scabbard as SB, turretcams as TC, hand3_mesh as H3M
from geom import Scene, frustum
xlpod4.install()
C = sk3d.COL
TAIL_Q = [-18.0, 12.0, 38.0, -10.0]

class Rec(sk3d.Scene):
    def __init__(s):
        super().__init__(); s.mat = "plastic_light"; s.mats = []
    def poly(s, pts, fc="#888888", alpha=1.0, ec=None, shade=True):
        s.polys.append(np.asarray(pts, float)); s.cols.append((fc if isinstance(fc, str) else "#%02x%02x%02x" % tuple(int(255 * x) for x in fc[:3]), alpha)); s.mats.append(s.mat)
sc = Rec()
S = C3.S
hipF, hipR = C3.HIPF, C3.HIPR
ZH = LS.H * S
# ------------------------------------------------------------------ Hippos-1 body + legs (preview geometry, no CAD yet)
PACK_X = -0.104
_zlo = C4.sections()[0]; PACK_Z = float(max(np.interp(xx, C4.XS, _zlo) for xx in np.linspace(PACK_X - 0.08, PACK_X + 0.08, 9))) + 0.054
C3.GIRDLE_W, C3.GIRDLE_Y, C3.PACK_Z, C3.KEEL_TAIL, C3.PACK_X = 0.118 * S, C3.Y_LEG_F - C3.ACT_OUT_IN - C3.HIPF_L, PACK_Z, 0.012, PACK_X
C3.DECK_X, C3.DECK_Z = PACK_X, PACK_Z + 0.037
sc.mat = "metal_dark"; C3.girdle(sc)
sc.mat = "shell"; C4.shell(sc, rings=False); sc.mat = "trim"; C4.shell_trim(sc)
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
    sc.cyl([hip[0], yin - side * hipact[2] / 2, hip[2]], [hip[0], yin + side * hipact[2] / 2, hip[2]], hipact[1], C["act"], n=48)
    if front and C3.ABD_MODE == "tube":
        yt = side * C3.TT_Y
        sc.cyl([C3.TT_ACT_X0, yt, C3.TT_Z], [C3.TT_ACT_X1, yt, C3.TT_Z], C3.ABD_R, "#3a3d42", n=48)
        sc.mat = "steel"; sc.cyl([C3.TT_ACT_X1, yt, C3.TT_Z], [C3.TT_X1, yt, C3.TT_Z], C3.TT_R, "#8a9099", n=24); sc.mat = "metal_dark"
    else:
        adz = C3.ABD_DZ if front else C3.ABD_DZ_R; ad = C3.ABD_DIR_F if front else C3.ABD_DIR_R
        abd_in = C3.ABD_IN if front else C3.ABD_IN_R
        a_r, a_l = (C3.ABD_R, C3.ABD_X1 - C3.ABD_X0) if front else (C3.ABDR_R, C3.ABDR_L)
        sc.cyl([hip[0] + ad * C3.ABD_X0, side * (y_leg - abd_in), hip[2] + adz], [hip[0] + ad * (C3.ABD_X0 + a_l), side * (y_leg - abd_in), hip[2] + adz], a_r, "#3a3d42", n=48)
    dfem = (st - hip) / np.linalg.norm(st - hip)
    Y_BELT = y_leg + 0.040
    kn = hip - dfem * C3.R_K; kn = np.array([kn[0], side * y_leg, kn[2]])
    sc.cyl(kn - [0, side * 0.0245, 0], kn + [0, side * 0.0245, 0], 0.044, C["act"], n=48)
    sc.mat = "plastic_blue"; sc.cyl(np.array([hip[0], side * y_leg, hip[2]]), kn, 0.016, C["horse"], n=24)
    sc.mat = "alu"
    p1 = np.array([kn[0], side * Y_BELT, kn[2]]); p2 = np.array([st[0], side * Y_BELT, st[2]])
    sc.cyl(p1 - [0, 0.0125, 0], p1 + [0, 0.0125, 0], 0.0239, "#b0b6bd", n=30); sc.cyl(p2 - [0, 0.0125, 0], p2 + [0, 0.0125, 0], 0.0478, "#b0b6bd", n=60)
    sc.mat = "rubber"
    d = p2 - p1; d = d / np.linalg.norm(d); n_ = np.array([-d[2], 0, d[0]])
    for sgn2 in (1, -1): sc.cyl(p1 + sgn2 * n_ * 0.0245, p2 + sgn2 * n_ * 0.0485, 0.004, C["belt"], n=8)
    sc.mat = "plastic_blue"
    sc.cyl(hip, st, 0.020, C["horse"], n=24, r1=0.017); sc.cyl(st, hk, 0.011, C["horse"], n=20)
    sc.sphere(st, 0.024, C["horse"], n=16); sc.sphere(hk, 0.016, C["horse"], n=12); sc.cyl(hk, fe, 0.011, C["horse"], n=20)
    sc.cyl(fe, ft + [0, 0, 0.02], 0.009, C["horse"], n=16)
    sc.mat = "shell"; C4.leg_cover(sc, hip, st, hk, fe, side)
    sc.mat = "rubber_tpu"; sc.cyl(ft, ft + [0, 0, 0.032], 0.034, C["tpu"], n=40, r1=0.026)
    sc.mat = "carbon"
    for dy in (-0.012, 0.012): sc.cyl(Pp + [0, dy, 0], Qq + [0, dy, 0], 0.003, C["latch"], n=8)
for s_ in (1, -1):
    leg(hipF[0], s_, True); leg(hipR[0], s_, False)

# ------------------------------------------------------------------ mesh list: (family, hex colour, alpha, V (m), F)
OUT = []
def hexc(rgb): return "#%02x%02x%02x" % tuple(int(round(255 * float(x))) for x in rgb[:3])
def add_tm(m, fam, rgb, alpha=1.0, T=None):
    m = m.copy()
    if T is not None: m.apply_transform(T)
    OUT.append((fam, hexc(rgb) if not isinstance(rgb, str) else rgb, alpha, np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.int32)))
def man_tm(man, scale=1e-3):
    me = man.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3] * scale, np.asarray(me.tri_verts), process=False)

# ------------------------------------------------------------------ Cephalo-2 body CAD, in the docked body frame
with contextlib.redirect_stdout(io.StringIO()):
    _T, Hh = AS.tasks(S)
bp, bR = Hh["oct_p"], Hh["oct_R"]
TB = np.eye(4); TB[:3, :3] = bR; TB[:3, 3] = bp
COLS = dict(shell=(0.93, 0.93, 0.91), plate=(0.55, 0.57, 0.60), cage=(0.78, 0.80, 0.83), blade=(0.16, 0.17, 0.19), hub=(0.28, 0.29, 0.31),
            motor=(0.12, 0.30, 0.55), elev=(0.85, 0.60, 0.20), rods=(0.70, 0.72, 0.75), cradle=(0.30, 0.55, 0.85), board=(0.08, 0.40, 0.22),
            lens=(0.12, 0.12, 0.14), hs=(0.62, 0.63, 0.66), glass=(0.55, 0.78, 0.95), socket=(0.95, 0.55, 0.15), hbase=(0.30, 0.31, 0.34),
            port=(0.20, 0.21, 0.23))
FAMC = dict(shell="c2_shell", plate="c2_plate", cage="petg_light", blade="pa_cf", hub="metal_dark", motor="metal_paint", elev="petg_orange",
            rods="steel", cradle="petg_blue2", board="pcb", lens="lens", hs="alu", glass="glass", socket="petg_orange", hbase="petg_dark", port="rubber")
def body(man, key, alpha=1.0):
    add_tm(man_tm(man), FAMC[key], COLS[key], alpha, TB)
body(TC.shell_with_windows(), "shell"); body(SB.plate_solid(), "plate"); body(SB.cage_solid(), "cage"); body(SB.motor_solid(), "motor")
body(SB.four(SB.latch_pin(0.0), 45.0), "rods")
A = TC.assembly()
for key, ck in dict(cradle="cradle", bridge="cradle", bezel="cradle", board="board", module="lens", heatsink="hs", disc="glass").items():
    ms = [v for k, v in A.items() if k.endswith("_" + key)]
    if ms: body(SB.union(ms), ck, 0.35 if ck == "glass" else 1.0)
# belly rotor in "bars" mode (cephalo2_model.BLADE_MODES): blades hanging, slid 108 mm down into the receiver
rot = SB.rotating(SB.BETA_H, SB.U_DOCK, psi=0.0)
body(rot.pop("hub"), "hub"); body(SB.union([v for k, v in rot.items() if k.startswith("barrel")]), "hub")
body(SB.union([v for k, v in rot.items() if k.startswith("blade")]), "blade")
rods, zt = SB.rods_solid(SB.BETA_H); body(rods + SB.fold_ring(zt), "rods"); body(SB.elevator_solid(SB.U_DOCK), "elev")
psc = Scene()
for i in range(8):
    a = np.radians(22.5 + 45 * i); d = np.array([np.cos(a), np.sin(a), 0])
    psc.add(frustum(d * 0.100 + [0, 0, 0.039], d * 0.110 + [0, 0, 0.039], 0.031, 0.031, COLS["port"], n=40))
for V, F, Cc, _ in psc.prims: add_tm(trimesh.Trimesh(V, F, process=False), "rubber", COLS["port"], 1.0, TB)

# ------------------------------------------------------------------ arms, connectors, hands and claw-props
# T4 "hands meet" re-solved for the real hand: the reach study's hand point sat 60 mm past the spigot (stand-in hand).
# hand3 + connector reaches 159 mm (heel); its grip centre is ~130 mm out.  With 60 mm the two hand3s overlap by ~16 mm,
# so the same IK (same chest target x / z, same clearance terms) is solved at the grip centre, with the targets moved
# apart until the hands clear each other by ~10 mm.  Arms unchanged; only the pose.
AS.LEAD = 0.0099 + 0.120
_chest = Hh["sh"][:, :, 0].max(); T4Y = float(os.environ.get("T4Y", "0.080"))
with contextlib.redirect_stdout(io.StringIO()):
    _t4 = AS.solve(Hh, AS.AZ_R, np.array([_chest + 0.20, T4Y, Hh["oct_p"][2] - 0.15]), S)
assert _t4["ok"], _t4
# hand3 on every arm (426 g with the connector, COM 69 mm past the spigot): that IK pose holds j0 at 139 % and j2 at 125 % of
# rated.  Same target, same clearance terms, plus a penalty on static joint torque above 80 % of rated (cephalo4 masses).
from scipy.optimize import least_squares
POD_H3 = (0.426, 0.0099 + 0.059)
def _load(q): return np.abs(CE.joint_torques(0, q, bp, bR, POD_H3)) / CE.RATED
_tgt = np.array([_chest + 0.20, T4Y, Hh["oct_p"][2] - 0.15]); _lo, _hi = AS.LIM.T
# hand 7 is hand 0 mirrored in y = 0, so the hands clear each other by 2 x the smallest y of hand 0's mesh: keep it >= 5 mm
_HV = np.vstack([m.vertices for m, _, _ in H3M.hand(lag=0.0, fold=(28, 40, 35))]) * 1e-3
_HV = _HV[np.random.default_rng(0).choice(len(_HV), 600, replace=False)]
def _hand_pts(q):
    _segs, (Rt, pt) = CE.arm_fk(0, q, bp, bR); u = Rt[:, 0]
    return _HV @ np.stack([Rt[:, 1], Rt[:, 2], u], 1).T + pt + u * (-SFF.P["flange_bot"] + SFF.P["gap"]) * 1e-3
def _res(q, w):
    pts, hand, _R = AS.arm_fk(AS.AZ_R, q, bp, bR); P_, r_ = AS.capsules(pts)
    cl = np.minimum(AS.shell_dist(Hh, P_), AS.leg_dist(Hh, P_, 0.045 * S)) - r_ - AS.CLEAR
    hy = np.r_[_hand_pts(q)[:, 1] - 0.005, P_[:, 1] - r_ - 0.015]          # hand mesh and arm (capsule + 10 mm for the clutch cams, pins) stay off the mirror plane
    return np.concatenate([(hand - _tgt) * 10, np.minimum(cl, 0) * 20, np.minimum(hy, 0) * 40, w * np.maximum(_load(q) - 0.80, 0), 0.003 * q])
_best = None
for _st in [_t4["q"], np.radians([-55, 0, 40, 0, 20, 0, 30, 0]), np.radians([-59, 10, 60, -10, 30, 0, 20, 0]), np.radians([-40, 0, 0, 0, 60, 0, 40, 0])]:
    for _w in (1.0, 3.0):
        _s = least_squares(lambda q: _res(q, _w), np.clip(_st, _lo + 1e-6, _hi - 1e-6), bounds=(_lo, _hi), max_nfev=400)
        pts, hand, _R = AS.arm_fk(AS.AZ_R, _s.x, bp, bR); P_, r_ = AS.capsules(pts)
        _cl = min((AS.shell_dist(Hh, P_) - r_).min(), (AS.leg_dist(Hh, P_, 0.045 * S) - r_).min()); _err = np.linalg.norm(hand - _tgt)
        if _err < 0.005 and _cl >= AS.CLEAR - 0.002 and _hand_pts(_s.x)[:, 1].min() >= 0.004 and (_best is None or _load(_s.x).max() < _best[0]): _best = (_load(_s.x).max(), _s.x)
q4 = _best[1]
# raised six with hand3 as fans: Q_GUARD (-35, -45, 0, 0) holds j0 at 102 % of rated.  Scan with the rotor axis kept 80 deg up
# (sum of pitches, as Q_GUARD), swept discs >= 15 mm apart, arms >= 10 mm off the dome / turret: (-35, -85, 90, -50) at 91 %.
Q_GUARD = np.radians([-35, 0, -85, 0, 90, 0, -50, 0])
ARM_Q = {0: q4, 7: q4 * np.array([1, -1, 1, -1, 1, -1, 1, -1])}
for i in range(1, 7): ARM_Q[i] = Q_GUARD
HANDS = tuple(range(8))                      # Cephalo-2 baseline: hand3 on all eight arms (cephalo2-baseline.md)
SPIN = [float(x) for x in os.environ.get("H3_SPIN", "0,10,30,15,35,20,5,0").split(",")]   # parked rotor angle per arm (deg)
LEN_BASE = (-SFF.P["flange_bot"] + SFF.P["gap"]) * 1e-3      # hand base underside to the socket mouth (9.9 mm)
SOCK, BASE = man_tm(SFF.arm_socket()), man_tm(SFF.hand_base())
ARMFAM = {(0.92, 0.92, 0.90): "petg_light", (0.22, 0.42, 0.78): "servo_blue", (0.28, 0.29, 0.32): "metal_dark", (0.95, 0.55, 0.15): "petg_orange",
          (0.75, 0.40, 0.10): "petg_orange", (0.80, 0.45, 0.12): "petg_orange", (0.70, 0.72, 0.75): "steel", (0.20, 0.21, 0.23): "metal_dark",
          (0.55, 0.57, 0.60): "alu", (0.35, 0.36, 0.40): "metal_dark", (0.16, 0.17, 0.19): "pa_cf", (0.74, 0.76, 0.79): "alu",
          (0.13, 0.13, 0.15): "metal_dark", (0.86, 0.72, 0.16): "tape"}
def scene_out(scn):
    by = {}
    for V, F, Cc, _ in scn.prims:
        cols = np.round(np.asarray(Cc, float), 2)
        for c in np.unique(cols, axis=0):
            msk = np.all(cols == c, axis=1); Ff = F[msk]; used = np.unique(Ff)
            remap = -np.ones(len(V), int); remap[used] = np.arange(len(used))
            by.setdefault(tuple(c), []).append(trimesh.Trimesh(V[used], remap[Ff], process=False))
    for c, lst in by.items():
        fam = ARMFAM.get(tuple(np.round(c, 2)), "petg_light")
        add_tm(trimesh.util.concatenate(lst), fam, c)
TIPS = {}; ARM_IDX = {}
for i in range(8):
    n_before = len(OUT)
    asc = Scene()
    segs, (Rt, pt) = VA.arm_body(asc, i, bp, bR, ARM_Q[i])
    u = Rt[:, 0]
    Rc = np.stack([Rt[:, 1], np.cross(-u, Rt[:, 1]), -u], 1)                 # connector frame: z into the arm
    T4 = np.eye(4); T4[:3, :3] = Rc; T4[:3, 3] = pt
    add_tm(SOCK, "petg_orange", COLS["socket"], 1.0, T4); add_tm(BASE, "petg_dark", COLS["hbase"], 1.0, T4)
    p0 = pt + u * LEN_BASE
    TIPS[i] = (Rt, p0)
    if i in HANDS:
        Th = np.eye(4); Th[:3, :3] = np.stack([Rt[:, 1], Rt[:, 2], u], 1); Th[:3, 3] = p0
        Th = Th @ np.diag([1e-3, 1e-3, 1e-3, 1.0])
        famh = dict(alu="alu", steel="steel", tpu="tpu_orange", pacf="pa_cf", frame="petg_light", servo="servo_blue", motor="metal_dark", ring="petg_orange")
        fold = (28, 40, 35) if i in (0, 7) else (0, 0, 0)          # front pair grasp-ready; the six raised are fans (fingers straight)
        for m, c, k in H3M.hand(lag=0.0, fold=fold, spin=SPIN[i]):
            add_tm(m, famh[k], c, 1.0, Th)
    else:
        xlpod4.draw(asc, p0, Rt, spin=np.radians(20.0 + 37.0 * i), fold=0.0)
    scene_out(asc)
    ARM_IDX[i] = (n_before, len(OUT))

# ------------------------------------------------------------------ tail CAD on its rump socket
NO_TAIL = os.environ.get("NO_TAIL") == "1"
z_seat = 0.60 * S
t = C3.hits(np.array([0.0, z_seat]), np.array([1.0, 0.0])); x_rump = t[0]
M = np.array([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]]); ORG = np.array([x_rump, 0.0, z_seat])
TT = np.eye(4); TT[:3, :3] = M; TT[:3, 3] = ORG
def tail_add(man, fam, col):
    add_tm(man_tm(man), fam, col, 1.0, TT)
tail_add(TM.rump_socket(), "pa_cf", "#2c2f34"); tail_add(SFF.pin_board(), "pcb", "#1f6b3a"); tail_add(SFF.hand_base(), "petg_blue", "#4d77a6")
tail_add(TM.tail_adapter(), "petg_blue", "#4d77a6")
# the tail is one Cephalo-2 arm + hand3 (hippos/tailarm/tailarm.py, Forest 2026-11-16), pose from tailarm_checks.json
if not NO_TAIL:
    sys.path.insert(0, "/home/claude/octodrone/hippos/tailarm")
    import json as _json, tailarm as TA
    TAIL_POSE = os.environ.get("TAIL_POSE", "carried high (raised, curled)")
    _tq = np.radians(_json.load(open("/home/claude/octodrone/hippos/tailarm/tailarm_checks.json"))["poses"][TAIL_POSE]["q_deg"])
    _famh = dict(alu="alu", steel="steel", tpu="tpu_orange", pacf="pa_cf", frame="petg_light", servo="servo_blue", motor="metal_dark", ring="petg_orange")
    for fam, col, m in TA.meshes(_tq, ORG, M):
        if fam == "arm": fam = ARMFAM.get(tuple(np.round(col, 2)), "petg_light")
        elif fam.startswith("hand3:"): fam = _famh[fam.split(":")[1]]
        add_tm(m, fam, col)

# ------------------------------------------------------------------ export
groups = {}
for pts, (fc, a), mat in zip(sc.polys, sc.cols, sc.mats):
    if len(pts) < 3: continue
    key = (mat, fc, round(float(a), 2)); g = groups.setdefault(key, [])
    for i in range(1, len(pts) - 1): g.append(np.r_[pts[0], pts[i], pts[i + 1]])
out = {}
for i, ((mat, fc, a), tris) in enumerate(groups.items()):
    out[f"g{i}"] = np.asarray(tris, np.float32).reshape(-1, 3); out[f"g{i}_meta"] = np.array([mat, fc, str(a)])
for i, (fam, col, a, V, F) in enumerate(OUT):
    out[f"o{i}_v"] = V; out[f"o{i}_f"] = F; out[f"o{i}_meta"] = np.array([fam, col, str(a)])
np.savez_compressed(os.environ.get("SCENE_OUT", "/home/claude/octodrone/centaur/" + ("scene2_notail.npz" if NO_TAIL else "scene2.npz")), **out)
ntri = sum(len(g) for g in groups.values()) + sum(len(F) for *_, F in OUT)
print(f"preview groups {len(groups)}, CAD meshes {len(OUT)}, triangles {ntri}")
np.save("/home/claude/octodrone/centaur/tips.npy", np.array([np.r_[TIPS[i][1], TIPS[i][0][:, 0]] for i in range(8)]))
