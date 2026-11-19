"""Equo-1 v2 + Cephalo-1 full-model preview: builds the 3D geometry (sk3d primitives, metres) and the design plots' data,
then writes equo_preview.html (three.js viewer + Plotly side plots + design-rule checklist).
python3 build_preview.py   (needs ../equo_v2.py, ../legs_v2/legs_v2_study.py + legs.json, ../equo_v2_sizing.json, ../equo_v2_rescale.json)"""
import sys, os, json, io, contextlib
sys.path.insert(0, "/home/claude/octodrone"); sys.path.insert(0, "/home/claude/octodrone/centaur"); sys.path.insert(0, "/home/claude/octodrone/centaur/legs_v2")
import numpy as np
import sk3d
from matplotlib.colors import to_rgb
with contextlib.redirect_stdout(io.StringIO()):
    import equo_v2 as V
    import legs_v2_study as LS
HERE = os.path.dirname(os.path.abspath(__file__)); C = sk3d.COL

class Col(sk3d.Scene):
    """sk3d primitives without matplotlib: records polygons with a part tag and a part id (one id per top-level primitive,
    or per named group) so the full collision check can test every part against every other part."""
    def __init__(s):
        s.polys, s.cols, s.edges, s.tags, s.pids = [], [], [], [], []; s.tag = "misc"
        s.part = 0; s.depth = 0; s.grp = None; s.pnames = {}; s.where = ""; s.obstacles = []
    def poly(s, pts, fc="#888888", alpha=1.0, ec=None, shade=True):
        if s.depth == 0 and s.grp is None: s._new("poly")
        s.polys.append(np.asarray(pts, float)); s.cols.append((fc if isinstance(fc, str) else "#%02x%02x%02x" % tuple(int(255 * x) for x in fc[:3]), alpha)); s.tags.append(s.tag); s.pids.append(s.part)
    def _new(s, kind):
        s.part += 1; s.pnames[s.part] = f"{s.tag}:{kind}{':' + s.where if s.where else ''}"
    def group(s, name):
        """all primitives until ungroup() form one part (a shell, a cover, a ring of quads)"""
        s.part += 1; s.pnames[s.part] = f"{s.tag}:{name}{':' + s.where if s.where else ''}"; s.grp = s.part
    def ungroup(s): s.grp = None

_cyl_base = sk3d.Scene.cyl
def _cyl_log(s, p0, p1, r, *a, **k):
    if s.depth <= 1 and s.tag in ("actuators", "suspension", "belts"):     # depth 1: called through the part wrapper
        s.obstacles.append((np.asarray(p0, float), np.asarray(p1, float), max(r, k.get("r1") or 0)))
    return _cyl_base(s, p0, p1, r, *a, **k)
sk3d.Scene.cyl = _cyl_log

def _wrap(name):
    base = getattr(sk3d.Scene, name)
    def f(s, *a, **k):
        if s.depth == 0 and s.grp is None: s._new(name)
        s.depth += 1
        try: return base(s, *a, **k)
        finally: s.depth -= 1
    setattr(Col, name, f)
for _nm in ("box", "cyl", "ring", "sphere", "dome", "disc", "tube", "octopus"): _wrap(_nm)

sc = Col()
# ------------------------------------------------------------------ body: rib frame, skin, spine keel, packs (equo_v2)
import chassis_v3 as C3
import chassis_v4 as C4
# ---- balance: masses at scale S (bought parts fixed, printed / tube structure ~ S^2.5), packs placed on the sled
S = C3.S; S25 = S ** 2.5
_L1 = json.load(open("../legs_v2/legs.json")); _Z1 = json.load(open("../equo_v2_sizing.json"))
M_ACT = 2 * 0.880 + 10 * 0.621
SOLO = os.environ.get("EQUO_SOLO", "0") == "1"      # Equo-1 running on its own (no octopus on the collar)
MS = {"octopus (Cephalo-1)": 12.81, "four legs": M_ACT + (_L1["parts"]["R36 RS03 front hips, RS06 elsewhere, 12S"]["m_legs"] - M_ACT) * S25,
      "rib frame + skin + keel": (_Z1["totals"]["frame_new"] + 0.05) * S25, "swing unit + shocks": _Z1["totals"]["added_swing"] * (0.5 + 0.5 * S),
      "tail": _Z1["totals"]["added_tail"], "packs (2 x 6S 5000)": 1.62, "neck socket block": 0.89}
if SOLO:
    MS.pop("octopus (Cephalo-1)")
    MS["sensor hood (2 cameras, IMU, hood servo) + Pi 5"] = 0.45
MS["four legs"] += sum(2 * (C3.ACT[C3.SEL[k]][3] - C3.ACT[v][3]) for k, v in
                       {("front", "hip"): "RS03", ("rear", "abd"): "RS06"}.items())        # swaps against the leg study set
if C3.ABD_MODE == "tube": MS["four legs"] += 2 * (7.85e3 * np.pi * (0.010 ** 2 - 0.008 ** 2) * (C3.TT_X1 - C3.TT_ACT_X1) + 0.045)   # tubes + bearings
MS["electronics, wiring, rest (est.)"] = max(34.0 - sum({k: v for k, v in MS.items()}.values()), 0.5) if S == 1.0 else None
if MS["electronics, wiring, rest (est.)"] is None:
    MS["electronics, wiring, rest (est.)"] = 34.0 - (12.81 + _L1["parts"]["R36 RS03 front hips, RS06 elsewhere, 12S"]["m_legs"] + _Z1["totals"]["frame_new"] + 0.05
                                                  + _Z1["totals"]["added_swing"] + _Z1["totals"]["added_tail"] + 1.62 + 0.89)
_foot = lambda front: (C3.HIPF[0] - P_LEG0["foot"][0] * S) if front else (C3.HIPR[0] + P_LEG0["foot"][0] * S)
P_LEG0 = LS.panto_points(LS.ARCH["B"], LS.ARCH["B"].ik(0.0, -LS.H))
_shell = C4.shell_points(); _xs = _shell[:, :, 0].mean(1)
def front_share(pack_x):
    X = {"octopus (Cephalo-1)": (C3.cc + C3.n_up * 0.09)[0], "four legs": None,
         "sensor hood (2 cameras, IMU, hood servo) + Pi 5": (C3.cc + C3.n_up * 0.03)[0],
         "rib frame + skin + keel": float((_shell[:, :, 0]).mean()), "swing unit + shocks": C3.HIPR[0] + 0.10 * S,
         "tail": -0.40 * S, "packs (2 x 6S 5000)": pack_x, "neck socket block": (C3.cc - C3.n_up * 0.114)[0],
         "electronics, wiring, rest (est.)": 0.0}
    mfr = (2 * 2.96 + 2 * 2.70); legs_x = (2 * 2.96 * C3.HIPF[0] + 2 * 2.70 * C3.HIPR[0]) / mfr
    xa_f = 0.5 * (C3.TT_ACT_X0 + C3.TT_ACT_X1) if C3.ABD_MODE == "tube" else C3.HIPF[0] + C3.ABD_DIR_F * 0.0995
    xa_r = C3.HIPR[0] + C3.ABD_DIR_R * 0.0995
    legs_x += 2 * 0.621 * ((xa_f - C3.HIPF[0]) + (xa_r - C3.HIPR[0])) / mfr; X["four legs"] = legs_x
    m = sum(MS.values()); xc = sum(MS[k] * X[k] for k in MS) / m
    return (xc - _foot(False)) / (_foot(True) - _foot(False)), xc
PACK_NOM = C3.HIPR[0] + 0.09 * S
if S == 1.0:
    PACK_X = PACK_NOM
else:   # the ballast sled (Tr8 screw, 180 mm travel) slides the packs to put 62 % on the front legs, as at full size
    xs_ = np.linspace(PACK_NOM - 0.09, PACK_NOM + 0.09, 61)
    if C3.ABD_DIR_R < 0: xs_ = xs_[xs_ - 0.08 >= C3.HIPR[0] - C3.ABD_X0 + 0.005]           # pack rear end ahead of the rear abduction actuators
    if C3.ABD_MODE == "tube":                                                                    # pack front end behind the sheath block
        xs_ = xs_[xs_ + 0.08 <= C3._block_rear_x(C3.KEEL_Z) - 0.006]          # lowest pack corner sets it (block leans forward)
        if C3.TT_Y - C3.ABD_R < 0.055 + 0.004:                                   # front abduction actuators reach over the pack line
            xs_ = xs_[xs_ + 0.08 <= C3.TT_ACT_X0 - 0.005]
    sh_ = np.array([front_share(x)[0] for x in xs_])
    PACK_X = float(xs_[np.argmin(np.abs(sh_ - 0.62))])
SHARE = front_share(PACK_X)[0]
_zlo = C4.sections()[0]; PACK_Z = float(max(np.interp(xx, C4.XS, _zlo) for xx in np.linspace(PACK_X - 0.08, PACK_X + 0.08, 9))) + 0.054   # pack corners clear the belly pan along the whole pack
print(f"scale {S:.2f}: mass {sum(MS.values()):.1f} kg, packs at x {1e3*PACK_X:.0f} mm, front share {100*SHARE:.1f} %")
sc.tag = "frame"; C3.GIRDLE_W, C3.GIRDLE_Y, C3.PACK_Z, C3.KEEL_TAIL, C3.PACK_X = 0.118 * S, C3.Y_LEG_F - C3.ACT_OUT_IN - C3.HIPF_L, PACK_Z, 0.012, PACK_X; C3.DECK_X, C3.DECK_Z = PACK_X, PACK_Z + 0.025 + 0.012;   # controller rides on the packs
C3.girdle(sc)   # bulkhead inside the hip actuators, tube ends at their housings
sc.tag = "shell"; sc.group("body shell"); C4.shell(sc, rings=False); sc.ungroup(); sc.tag = "trim"; C4.shell_trim(sc)
hipF, hipR = C3.HIPF, C3.HIPR
ZH = LS.H * C3.S
# ------------------------------------------------------------------ legs (legs_v2: pantograph, RS03 front hips, RS06 elsewhere)
B = LS.ARCH["B"]; q = B.ik(0.0, -LS.H); P = {k: np.asarray(v) * C3.S for k, v in LS.panto_points(B, q).items()}   # legs scale with the horse
def leg(xh, side, front):
    sgn = -1.0 if front else 1.0                                       # front leg = the module turned end for end (x mirrored)
    y_leg = C3.Y_LEG_F if front else C3.Y_LEG_R; y = side * y_leg
    sc.where = f"{'F' if front else 'R'}{'L' if side > 0 else 'R'}"
    W = lambda p: np.array([xh + sgn * p[0], y, ZH + p[1]])
    hip, st, hk, fe, ft = (W(P[k]) for k in ("hip", "stifle", "hock", "fet", "foot"))
    Pp, Qq = W(P["P"]), W(P["Q"])
    sc.tag = "actuators"
    hipact = C3.ACT[C3.SEL[("front" if front else "rear", "hip")]][:3]
    yin = side * (y_leg - C3.ACT_OUT_IN - hipact[2] / 2)                # outer face 10 mm under the pod face
    sc.cyl([hip[0], yin - side * hipact[2] / 2, hip[2]], [hip[0], yin + side * hipact[2] / 2, hip[2]], hipact[1], C["act"], n=24)   # hip pitch
    ad = C3.ABD_DIR_F if front else C3.ABD_DIR_R
    abd_in = C3.ABD_IN if front else C3.ABD_IN_R
    if front and C3.ABD_MODE == "tube":                                  # behind the sheath block + torque tube to the hip module
        yt = side * C3.TT_Y
        sc.cyl([C3.TT_ACT_X0, yt, C3.TT_Z], [C3.TT_ACT_X1, yt, C3.TT_Z], C3.ABD_R, "#3a3d42", n=24)            # RS06 (abduction)
        sc.cyl([C3.TT_ACT_X1, yt, C3.TT_Z], [C3.TT_X1, yt, C3.TT_Z], C3.TT_R, "#8a9099", n=12)                  # 20 x 2 steel torque tube
        sc.cyl([C3.TT_X1 - 0.012, yt, C3.TT_Z], [C3.TT_X1, yt, C3.TT_Z], C3.TT_HUB_R, C["act"], n=14)          # clamp hub (6802 in the girdle behind it)
        sc.cyl([C3.TT_X1, side * (C3.TT_Y + 0.012), C3.TT_Z], [hip[0] - 0.045, side * (y_leg - C3.ACT_OUT_IN - 0.028), hip[2] + 0.02], 0.008, C["act"], n=8)   # carrier arm, outboard of the hub, to the pitch actuator housing
    else:
        adz = C3.ABD_DZ if front else C3.ABD_DZ_R
        a_r, a_l = (C3.ABD_R, C3.ABD_X1 - C3.ABD_X0) if front else (C3.ABDR_R, C3.ABDR_L)
        sc.cyl([hip[0] + ad * C3.ABD_X0, side * (y_leg - abd_in), hip[2] + adz], [hip[0] + ad * (C3.ABD_X0 + a_l), side * (y_leg - abd_in), hip[2] + adz], a_r, "#3a3d42", n=24)  # abduction (axis x), at the hip
    # thigh stack, inboard -> outboard: femur web (y 0.200 +-0.010) | belt plane 0.225 (30T + 60T, 25 mm belt) | knee actuator
    # 0.2395..0.2885 with its output face inboard driving the 30T.  One belt plane, nothing skewed.
    dfem = (st - hip) / np.linalg.norm(st - hip)
    if C3.KNEE_MODE == "scapula":     # knee RS06 in the femur plane on the shoulder-blade arm, 30T on its outboard face
        Y_BELT = y_leg + 0.040
        kn = hip - dfem * C3.R_K; kn = np.array([kn[0], side * y_leg, kn[2]])
        sc.cyl(kn - [0, side * 0.0245, 0], kn + [0, side * 0.0245, 0], 0.044, C["act"], n=24)
        sc.tag = "legs"; sc.cyl(np.array([hip[0], side * y_leg, hip[2]]), kn, 0.016, C["horse"], n=12); sc.tag = "actuators"   # shoulder-blade arm
    else:
        Y_BELT, Y_KNEE = y_leg + 0.025, y_leg + 0.064
        kn = hip + (st - hip) * 0.22; kn = np.array([kn[0], side * Y_KNEE, kn[2]])
        sc.cyl(kn - [0, side * 0.0245, 0], kn + [0, side * 0.0245, 0], 0.044, C["act"], n=24)
    BELT_C = float(np.linalg.norm(st - kn) if C3.KNEE_MODE == "scapula" else np.linalg.norm(st - hip) * 0.78)
    sc.tag = "belts"
    p1 = np.array([kn[0], side * Y_BELT, kn[2]]); p2 = np.array([st[0], side * Y_BELT, st[2]])
    sc.cyl(p1 - [0, 0.0125, 0], p1 + [0, 0.0125, 0], 0.0239, "#b0b6bd", n=20)                 # 30T
    sc.cyl(p2 - [0, 0.0125, 0], p2 + [0, 0.0125, 0], 0.0478, "#b0b6bd", n=28)                 # 60T
    d = p2 - p1; d = d / np.linalg.norm(d); n_ = np.array([-d[2], 0, d[0]])
    for sgn2 in (1, -1):
        sc.cyl(p1 + sgn2 * n_ * 0.0245, p2 + sgn2 * n_ * 0.0485, 0.004, C["belt"], n=6)
    sc.tag = "legs"
    sc.cyl(hip, st, 0.020, C["horse"], n=14, r1=0.017)                                         # femur shell
    sc.cyl(st, hk, 0.011, C["horse"], n=12)                                                    # tibia tube
    sc.sphere(st, 0.024, C["horse"], n=8); sc.sphere(hk, 0.016, C["horse"], n=6)
    sc.cyl(hk, fe, 0.011, C["horse"], n=12)                                                    # cannon tube
    sc.cyl(hk + (fe - hk) * 0.25, hk + (fe - hk) * 0.85, 0.0095, C["spring"], n=10)          # fetlock spring inside
    sc.cyl(fe, ft + [0, 0, 0.02], 0.009, C["horse"], n=10)                                     # pastern
    sc.tag = "covers"; sc.where = f"{'F' if front else 'R'}{'L' if side > 0 else 'R'}"; C4.leg_cover(sc, hip, st, hk, fe, side); sc.tag = "legs"
    sc.cyl(ft + [0, 0, 0.0], ft + [0, 0, 0.032], 0.034, C["tpu"], n=16, r1=0.026)             # hoof
    for dy in (-0.012, 0.012):
        sc.cyl(Pp + [0, dy, 0], Qq + [0, dy, 0], 0.003, C["latch"], n=6)                      # push rods (reciprocal apparatus)
    BELTS.append(BELT_C); sc.where = ""
    return ft
feet = []; BELTS = []
for s in (1, -1):
    feet.append(leg(hipF[0], s, True)); feet.append(leg(hipR[0], s, False))
# ------------------------------------------------------------------ rear swing unit + 2 goBILDA Big Bore shocks + lockout
sc.tag = "suspension"
piv = np.array([C3.TT_ACT_X0 - 0.016 if C3.ABD_MODE == "tube" else hipR[0] + 0.20 * C3.S, 0, ZH + 0.025])   # pivot: behind the front abduction actuators, 14 mm over the keel
for _s in (1, -1): sc.cyl(piv + [0, _s * 0.060, 0], piv + [0, _s * 0.090, 0], 0.012, C["act"], n=12)   # two stub axles on keel brackets (packs pass between)
# bell-crank 55 mm behind the rear hip line (the shock's front eye then stays clear of the neck receiver); the pushrod meets
# the swing arm 55 mm further from its pivot than the hip, a lever of LEV, so the crank ratio drops to keep 1.32 overall
PUSH_X = hipR[0] - 0.055
LEV = (piv[0] - PUSH_X) / (piv[0] - hipR[0])
BC_A = 0.050; BC_B = round(0.050 * 1.32 / LEV, 4)
BC_PIV = np.array([PUSH_X, 0, ZH + 0.095])
for s in (1, -1):
    sc.cyl(piv + [0, s * C3.SW_Y, 0], [hipR[0] + C3.ABD_DIR_R * C3.ABD_X0, s * C3.SW_Y, ZH + 0.025], 0.011, C["bar"], n=10)   # swing arm = carrier of the rear abduction actuator
    pv = BC_PIV + [0, s * C3.BC_Y, 0]; a_end = pv + [0.0, 0, -BC_A]; b_end = pv + [BC_B, 0, 0.0]
    sc.cyl(pv - [0, 0.008, 0], pv + [0, 0.008, 0], 0.010, C["act"], n=12)                    # crank pivot (608 bearings)
    sc.cyl(pv, a_end, 0.006, C["bar"], n=8); sc.cyl(pv, b_end, 0.006, C["bar"], n=8)        # crank arms
    sc.cyl(a_end, [PUSH_X, s * C3.SW_Y, ZH + 0.036], 0.004, C["latch"], n=6)                  # pushrod from the swing arm
    base = b_end; top = base + np.array([0.19, 0, 0.0])                                       # shock lies along the keel
    sc.cyl(base, base + (top - base) * 0.62, 0.011, "#1d1f22", n=12)                           # damper body
    sc.cyl(base + (top - base) * 0.6, top, 0.003, "#c8ced6", n=6)                              # shaft
    sc.cyl(base + (top - base) * 0.15, base + (top - base) * 0.75, 0.016, C["spring"], alpha=0.55, n=14)   # spring
LOCK = np.array([hipR[0] + 0.08, -C3.SW_Y, ZH + 0.025]) if C3.ABD_DIR_R < 0 else np.array([hipR[0] + 0.15 * C3.S, 0, ZH + 0.01])   # on the left swing arm, outside the packs
sc.cyl(LOCK - [0, 0, 0.02], LOCK + [0, 0, 0.012], 0.0025, C["latch"], n=6)  # lockout pin
sc.box(LOCK + [0, 0, 0.011 + 0.0185], (0.045, 0.025, 0.035), fc=C["act"])                    # its STS3215
# ------------------------------------------------------------------ rib frame, drawn last so it can stop at every actuator and the swing unit
sc.tag = "frame"; C4.frame(sc)
# ------------------------------------------------------------------ tail and octopus (equo_v2)
sc.tag = "tail"; C3.tail(sc)
# grab points for docking: the two side rim posts on the collar (drawn with the receiver, chassis_v3.girdle)
if SOLO:
    # sensor hood: closes the collar and sees for the horse; hinged at the front of the collar so it can swing forward 130 deg
    # (STS3215) and clear the dock while running.  2 x Raspberry Pi Camera Module 3 Wide (60 mm baseline, 20 deg down), BNO085
    sc.tag = "hood"
    _Rh = sk3d.rot("y", C3.TILT); _c0 = np.array([C3.cc[0], 0, C3.cc[1]])
    _n0 = len(sc.polys)
    sc.dome([0, 0, 0], 0.120, 0.055, "#3a3f47", n=14)
    sc.disc([0, 0, 0.001], 0.120, (0, 0, 1), "#3a3f47", 1.0)
    for yy in (-0.03, 0.03):
        sc.cyl([0.098, yy, 0.025], [0.112, yy, 0.020], 0.012, "#20242a", n=12)               # camera pods
        sc.disc([0.113, yy, 0.020], 0.007, (0.97, 0, -0.25), "#5fa8ff", 1.0)
    sc.box([0.0, 0, 0.058], (0.05, 0.05, 0.004), fc="#f2f2f2")                            # docking target (AprilTag) on top
    for _k in range(_n0, len(sc.polys)): sc.polys[_k] = np.asarray(sc.polys[_k]) @ _Rh.T + _c0
    ARMT = None
else:
    sc.tag = "octopus"
    # octopus body from the sketch kit (plate, dome, turret, hub, bars); the ARMS are drawn from Cephalo-1 rev 4's own geometry
    # (cephalo4: 8 joints p-y-p-y, 56 mm pitch, spigot 478, its joint limits) -- the horse is fitted to the arms, never the reverse
    _n0 = len(sc.polys)
    sc.octopus(base=(0, 0, 0), yaw=45, bars=0.194, arms="none")
    _R = sk3d.rot("y", C3.TILT)
    for _k in range(_n0, len(sc.polys)): sc.polys[_k] = sc.polys[_k] @ _R.T + np.array([C3.cc[0], 0, C3.cc[1]])

    # ------------------------------------------------------------------ arm working poses (from the reach check)
    sys.path.insert(0, "/home/claude/octodrone/centaur/arms_scale")
    import arm_scale_study as AS
    ARMT, _ = AS.tasks(S)
    _Hb = AS.horse(S); _bp, _bR = _Hb["oct_p"], _Hb["oct_R"]
    def _real_arm(az, q, colr):
        pts, hand, R = AS.arm_fk(az, q, _bp, _bR)
        P_, r_ = AS.capsules(pts, n=24)
        for a_, b_, ra, rb in zip(P_[:-1], P_[1:], r_[:-1], r_[1:]): sc.cyl(a_, b_, ra, colr, n=10, r1=rb, caps=False)
        for j in range(1, 9): sc.sphere(pts[j], 0.026 if j < 5 else 0.022, C["act"], n=6)          # servo joints (STS3250 / STS3215)
        return pts, hand, R
    sc.tag = "octopus"
    _q4 = ARMT["T4 hands meet"]["q"]
    _real_arm(AS.AZ_R, _q4, C["octo2"]); _real_arm(AS.AZ_L, _q4 * np.array([1, -1, 1, -1, 1, -1, 1, -1]), C["octo2"])   # mirror: yaws flip
    Q_GUARD = np.radians([-35, 0, -45, 0, 0, 0, 0, 0])                        # the other six raised, props ready (guard; arm_overlap.guard_pose)
    for i in range(1, 7):                                                      # the six others: 67.5 ... 292.5 deg
        az = AS.AZ_R + 45 * i
        pts, hand, R = _real_arm(az, Q_GUARD, C["octo2"])
        sc.disc(pts[-1] + R[:, 0] * 0.036, 0.0762, R[:, 0], C["octo"], 0.28)                 # 6 in claw-prop disc
    sc.tag = "reach"
    for k, colr in (("T3 table 750 mm", "#2f6db3"), ("T5 hand on the withers", "#7a5cc0")):
        v = ARMT[k]
        if not v["ok"]: continue
        sc.tube(list(v["pts"]), 0.014, colr, n=8)
        sc.sphere(v["hand"], 0.022, colr, n=6)
    _tx = ARMT["T3 table 750 mm"]["hand"][0] + 0.17
    sc.box([_tx, 0, 0.735], (0.40, 0.70, 0.03), fc="#c8b79a", alpha=0.6)                     # a 750 mm table
    for _dx in (-0.17, 0.17):
        for _dy in (-0.32, 0.32): sc.box([_tx + _dx, _dy, 0.36], (0.03, 0.03, 0.72), fc="#a8977a", alpha=0.6)
# ------------------------------------------------------------------ export geometry: triangles grouped by (tag, colour, alpha)
groups = {}
for pts, (fc, a), tag in zip(sc.polys, sc.cols, sc.tags):
    if len(pts) < 3: continue
    key = (tag, fc, round(float(a), 2)); g = groups.setdefault(key, [])
    for i in range(1, len(pts) - 1):
        g.extend(np.round(np.r_[pts[0], pts[i], pts[i + 1]] * 1000).astype(int).tolist())   # mm, int
geo = [dict(tag=k[0], c=k[1], a=k[2], v=v) for k, v in groups.items()]
ntri = sum(len(g["v"]) for g in geo) // 9

# ------------------------------------------------------------------ plot data
L = json.load(open("../legs_v2/legs.json")); REC = "R36 RS03 front hips, RS06 elsewhere, 12S"
Z = json.load(open("../equo_v2_sizing.json")); RS = json.load(open("../equo_v2_rescale.json"))
cases = ["stand", "walk M1", "trot M1", "trot M2", "get up", "landing 2.5 g, pair", "trot + 0.25 g push", "walk up 10 deg"]
margins = {leg: {j: {"rated": [L["margins"][REC][leg][c][j]["m_rms"] if L["margins"][REC][leg][c][j]["rms"] > 1e-6 else None for c in cases],
                     "peak": [L["margins"][REC][leg][c][j]["m_peak"] if L["margins"][REC][leg][c][j]["peak"] > 1e-6 else None for c in cases]}
                 for j in ("hip", "knee", "abd")} for leg in ("front", "rear")}
sets = [dict(name=s, rms=w["all_rms"], peak=w["all_peak"], mass=L["parts"][s]["m_legs"]) for s, w in L["worst"].items()]
belt = {k: dict(T=v["tension"], teeth=1104 / v["tension"], tension=2500 / v["tension"]) for k, v in L["belt"]["cases"].items() if "stall" not in k}
# torque-speed samples (actuator side) over the gaits
inr = LS.inertia_of("B"); ts = {}
for leg in ("front", "rear"):
    for gait in ("walk", "trot", "fast trot"):
        c = LS.m2_cycle(B, leg, gait, inr)
        ts[f"{leg} {gait}"] = dict(knee_T=(np.abs(c["tau"][:, 1]) / LS.KNEE_RATIO / LS.BELT_EFF).round(2).tolist(),
                                   knee_rpm=(np.abs(c["qd"][:, 1]) * LS.KNEE_RATIO * 30 / np.pi).round(1).tolist(),
                                   hip_T=np.abs(c["tau"][:, 0]).round(2).tolist(), hip_rpm=(np.abs(c["qd"][:, 0]) * 30 / np.pi).round(1).tolist())
Tline = np.linspace(0, 40, 41)
line06 = (480 * 39.6 / 48 - (Tline / 1.09) * 0.45 / (48 / 480)).round(1).tolist()
line03 = (195 * 39.6 / 48 - (np.linspace(0, 60, 41) / 2.36) * 0.39 / (48 / 195)).round(1).tolist()
m_oct = 12.81; m_legs = L["parts"][REC]["m_legs"]; T = Z["totals"]
mass = dict(MS)
M_TOT = sum(mass.values()); KLOAD = 34.0 / (M_TOT * S)          # joint torques ~ m g L: margins scale by this
BC_RATIO = BC_B / BC_A * LEV                  # shock travel per hip travel
k_w = 2 * RS["shock"]["per_shock_rate"] * BC_RATIO ** 2                 # N/mm at the hip
m_r = 2 * RS["shock"]["per_shock_static"] / 9.81 * ((1 - SHARE) * M_TOT) / ((1 - 0.62) * 34.0)   # rear sprung mass at this scale
BOUNCE = dict(ratio=BC_RATIO, k_wheel=k_w, f_n_Hz=np.sqrt(k_w * 1e3 / m_r) / (2 * np.pi), wheel_travel=40.0 / BC_RATIO,
              land_mm=2.5 * 2 * RS["shock"]["per_shock_static"] / k_w)
print("bounce", {k: round(float(v), 2) for k, v in BOUNCE.items()})
sw = Z["swing"]; F = np.linspace(0, 1.15 * sw["cases"]["2.5 g landing"]["F_hip"], 30)
shock = dict(F=F.round(1).tolist(), sink=np.minimum(F / (BOUNCE["k_wheel"] * 1e3) * 1e3, BOUNCE["wheel_travel"]).round(1).tolist(),
             pts={k: [v["F_hip"], round(min(v["F_hip"] / BOUNCE["k_wheel"], BOUNCE["wheel_travel"]), 1)] for k, v in sw["cases"].items()})
tail = {k: v["frac_rated"] for k, v in Z["tail"]["poses"].items()}
power = RS["power"]
# design rules (guiding principles) with the current value
w = L["worst"][REC]; b_land = belt["landing 2.5 g, pair"]
REAR_SOLO = REAR_CENT = None
if SOLO:      # what the rear legs carry alone vs with the octopus on (octopus back on, hood off, same pack position)
    _xc_s = front_share(PACK_X)[1]; _m_s = sum(MS.values())
    _x_oct = (C3.cc + C3.n_up * 0.09)[0]; _x_hood = (C3.cc + C3.n_up * 0.03)[0]
    _m_c = _m_s - 0.45 + 12.81; _xc_c = (_m_s * _xc_s - 0.45 * _x_hood + 12.81 * _x_oct) / _m_c
    _sh = lambda xc: (xc - _foot(False)) / (_foot(True) - _foot(False))
    REAR_SOLO = (1 - _sh(_xc_s)) * _m_s; REAR_CENT = (1 - _sh(_xc_c)) * _m_c
    print(f"solo: rear pair carries {REAR_SOLO:.1f} kg (with the octopus on: {REAR_CENT:.1f} kg)")
# per-joint margins with the actuator actually on each joint: need (s = 1, 34 kg) = capacity / margin in the leg study's
# set, scaled by m s / 34 (torque ~ m g L)
_CAP0 = {("front", "hip"): (20, 60)}
NEED = {}
for leg_ in ("front", "rear"):
    for j_ in ("hip", "knee", "abd"):
        c0 = _CAP0.get((leg_, j_), (11, 36)); r_ = p_ = 0.0
        for case_, v_ in L["margins"][REC][leg_].items():
            a_ = v_[j_]
            if a_["rms"] > 1e-6: r_ = max(r_, c0[0] / a_["m_rms"])
            if a_["peak"] > 1e-6: p_ = max(p_, c0[1] / a_["m_peak"])
        NEED[(leg_, j_)] = (r_ * M_TOT * S / 34.0, p_ * M_TOT * S / 34.0)
JM = {k: (C3.ACT[C3.SEL[k]][4] / v[0], C3.ACT[C3.SEL[k]][5] / v[1]) for k, v in NEED.items()}
for k, v in JM.items(): print(f"  {k[0]:5s} {k[1]:4s} {C3.SEL[k]}: need {NEED[k][0]:.1f} / {NEED[k][1]:.1f} N m -> margin {v[0]:.2f} rated, {v[1]:.2f} peak")
rules = [
    ("Leg actuators (per joint, as fitted): rated margin >= 1.2 on every case", min(v[0] for v in JM.values()), 1.2, "x"),
    ("Leg actuators (per joint, as fitted): peak margin >= 1.1 on every case", min(v[1] for v in JM.values()), 1.1, "x"),
    ("Knee belt teeth >= 1.25 at a 2.5 g landing", b_land["teeth"] * KLOAD, 1.25, "x"),
    ("Actuator speed headroom >= 1.5 over all gaits (0.8 m/s fast trot)", 2.05 * S, 1.5, "x"),
    ("Printed / tube structure >= 1.5 at landing", min(v["sf_land"] for v in L["structure"].values()) * KLOAD, 1.5, "x"),
    ("Front legs carry 55 to 65 % (horse-like; sled sets it)", 100 * SHARE, 55.0, "%") if not SOLO else
    ("Solo: rear legs carry no more than they do with the octopus on (kg on the rear pair)", REAR_SOLO, REAR_CENT, "kg max"),
    ("Trot runtime >= 30 min on two packs", power["trot"]["minutes"] * 34.0 / M_TOT, 30.0, "min"),
    ("Tail servos <= 100 % rated when used as a hand", 100 * Z["tail"]["poses"]["held out level, 0.3 kg in the gripper"]["frac_rated"][0], 100.0, "% max"),
    ("Rear bounce >= 6 Hz (bell-crank 1.32 on the two shocks)", BOUNCE["f_n_Hz"], 6.0, "Hz"),
    ("Keel tube stress margin >= 1.5", RS["keel"]["30 x 27"]["margin"] * 34.0 / (M_TOT * S), 1.5, "x"),
]
import types, audit_v4 as AU
AUD = AU.run(types.SimpleNamespace(sc=sc, LS=LS, hipF=hipF, hipR=hipR, ZH=ZH, P=P, PACK_X=PACK_X, PACK_Z=PACK_Z))
rules += [
    ("Shell clears every body-side actuator (air gap, after 2.5 mm wall)", AUD["1 actuators clearance to shell, net of wall (mm)"], 3.0, "mm"),
    ("Thigh covers clear knee actuator, belt and pulleys (air gap)", AUD["2 knee actuator / pulleys / belt inside thigh cover, net of wall (mm)"], 3.0, "mm"),
    ("Thigh and pauldron to body gap, femur swing of the gaits and kneeling + 10 deg", AUD["3 thigh-to-body swept gap (mm)"], 5.0, "mm"),
    ("Largest shell panel fits the 256 mm bed (longest side)", 256 - AUD["4 largest panel (mm)"][2], 0.0, "mm spare"),
    ("Frame, packs and girdle stay under the skin", AUD["6 frame (ribs, stringers, girdle, packs) under the skin, net of wall (mm)"], 0.0, "mm"),
]
if not SOLO:
    ARM_OUT = {k: dict(ok=v["ok"], err_mm=round(1e3 * v["err"], 1), clear_mm=round(1e3 * v["clear"], 1), bow=v.get("bow_deg"),
                       pts=(np.asarray(v["pts"]) * 1000).round().astype(int).tolist(), hand=(np.asarray(v["hand"]) * 1000).round().astype(int).tolist())
               for k, v in ARMT.items()}
    rules += [
        ("Front hands reach the floor ahead of the hooves, bowing <= 12 deg", 12 - (ARMT["T2 floor, bowing"]["bow_deg"] if ARMT["T2 floor, bowing"]["ok"] else 99), 0.0, "deg spare"),
        ("Front hands work at a 750 mm table, meet in front, reach the withers (arms clear of horse >= 15 mm)",
         float(min(1e3 * ARMT[k]["clear"] for k in ("T3 table 750 mm", "T4 hands meet", "T5 hand on the withers")) if all(ARMT[k]["ok"] for k in ("T3 table 750 mm", "T4 hands meet", "T5 hand on the withers")) else -1), 13.0, "mm"),
    ]
else:
    ARM_OUT = {}
_ax0 = np.array([*(C3.cc - C3.n_up * C3.SHEATH_L)]); _ax1 = np.array([*C3.cc]); _d = _ax1 - _ax0
_q = np.array(C3.HIPF); _aq = (_q - _ax0) @ (_d / np.linalg.norm(_d)); _tube_gap = np.linalg.norm(np.cross(np.r_[_q - _ax0, 0], np.r_[_d / np.linalg.norm(_d), 0])) - C3.recv_hw(np.clip(_aq, 0, C3.SHEATH_L)) - 0.016   # tube passes ahead/below the square block in side view
# knee belt: 30T + 60T on HTD 5M, length from the centre distance (2C + pi (D1 + D2) / 2 + (D2 - D1)^2 / 4C)
_C = max(BELTS); _D1, _D2 = 0.04775, 0.09549
BELT_L = 2 * _C + np.pi * (_D1 + _D2) / 2 + (_D2 - _D1) ** 2 / (4 * _C)
BELT_STOCK = int(np.ceil(BELT_L * 1000 / 5.0) * 5)
print(f"knee belt centre distance {1e3*_C:.0f} mm, length {1e3*BELT_L:.1f} mm -> HTD {BELT_STOCK}-5M (take up on the stifle blocks)")
if C3.ABD_MODE == "tube":
    _J = np.pi / 32 * (0.020 ** 4 - 0.016 ** 4); _tau = 36.0 * 0.010 / _J / 1e6        # RS06 peak torque in the 20 x 2 tube
    rules += [("Front abduction torque tube (20 x 2, 304 steel, 215 MPa yield) at RS06 peak", 215 / _tau, 2.0, "x")]
if not SOLO:
    import arm_overlap as AO
    _geos = [AO.arm_geo(AS.AZ_R, _q4, _bp, _bR), AO.arm_geo(AS.AZ_L, _q4 * np.array([1, -1, 1, -1, 1, -1, 1, -1]), _bp, _bR)] + \
            [AO.arm_geo(AS.AZ_R + 45 * i, Q_GUARD, _bp, _bR) for i in range(1, 7)]
    _wa, _wd, _wb = AO.report(_geos, _bp, _bR, range(2, 8))
    rules += [("Octopus arms never overlap: every arm pair (capsules), standing pose", 1e3 * _wa, 10.0, "mm"),
              ("Raised arms' prop discs clear each other", 1e3 * _wd, 10.0, "mm"),
              ("Arms clear the octopus's own dome and turret", 1e3 * _wb, 10.0, "mm")]
# static stability: centre of mass over the four hooves, margin to the support rectangle's nearest edge
_xc = front_share(PACK_X)[1]; _fx = (_foot(True), _foot(False)); _fy = min(C3.Y_LEG_F, C3.Y_LEG_R)
STAB = min(_xc - _fx[1], _fx[0] - _xc, _fy)
rules += [("Standing: centre of mass inside the four hooves, margin to the nearest edge", 1e3 * STAB, 50.0, "mm")]
import collide_all as CA          # full triangle-level check, every part against every other part
_np, CLASH_DESIGNED, CLASH_REAL = CA.report(sys.modules[__name__])
print(f"collision check: {_np} parts, {sum(CLASH_DESIGNED.values())} designed contacts, {len(CLASH_REAL)} real clashes")
for r_ in CLASH_REAL[:10]: print(f"   CLASH {1e3*r_[0]:.1f} mm  {r_[1]}  x  {r_[2]}")
rules += [(f"Nothing overlaps: every one of {_np} parts against every other (designed joints listed in the study)", len(CLASH_REAL), 0, "clashes max")]
rules += [("Front hip cross tube clears the neck sheath block", 1e3 * _tube_gap, 3.0, "mm"),
          ("Hip and abduction actuators clear the neck receiver (crown 236, throat 192 sq, sheath block 150 sq)", AUD["7 actuators to neck sheath block (mm)"], 3.0, "mm"),
          ("Neck receiver clears the frame, keel, packs and swing unit", AUD["7b receiver clears frame, keel, packs, swing unit (mm)"], 3.0, "mm"),
          ("Octopus seat: neck shell stays below the seat plane inside r 150 (plate r 125, rim posts r 139)", AUD["9 neck shell below the seat plane within r 150 (mm)"], 0.0, "mm"),
          ("Battery packs clear the actuators, swing unit and sheath block", AUD["8 packs clear actuators, swing unit, sheath block (mm)"], 3.0, "mm")]
SWEEP = json.load(open("../arms_scale/arm_scale.json"))
sweep = dict(s=[float(k) for k in SWEEP], **{f"{t}": [SWEEP[k]["tasks"][t]["err_mm"] if not SWEEP[k]["tasks"][t]["ok"] else 0 for k in SWEEP] for t in ("T1 floor, standing", "T2 floor, bowing")},
             bow=[SWEEP[k]["tasks"]["T2 floor, bowing"]["bow_deg"] if SWEEP[k]["tasks"]["T2 floor, bowing"]["ok"] else None for k in SWEEP],
             peak=[SWEEP[k]["loads"]["peak"] for k in SWEEP], speed=[SWEEP[k]["loads"]["speed"] for k in SWEEP],
             top=[1e3 * SWEEP[k]["top"] for k in SWEEP], sheath=[1e3 * SWEEP[k]["neck"]["sheath_to_hip_pod"] for k in SWEEP])
rules_out = [dict(rule=r, value=float(v), limit=float(lim), unit=u, ok=bool(v <= lim if u.endswith(" max") else v >= lim)) for r, v, lim, u in rules]
data = dict(sweep=sweep, arms=ARM_OUT, geo=geo, cases=cases, margins=margins, sets=sets, belt=belt, ts=ts, Tline=Tline.tolist(), line06=line06,
            T03=np.linspace(0, 60, 41).tolist(), line03=line03, mass=mass, shock=shock, tail=tail, rules=rules_out,
            checks={k: (float(v) if not isinstance(v, bool) else v) for k, v in C3.checks().items()}, info=dict(ntri=ntri, mass_total=sum(mass.values()), top=float(V.cc[1] * 1000 + (60 if SOLO else 224)), share=round(100 * SHARE), scale=S, solo=SOLO))
html = open(os.path.join(HERE, "template.html")).read().replace("/*DATA*/null", json.dumps(data, separators=(",", ":")))
if SOLO:
    html = html.replace("<title>Centauri-1: full-model preview</title>", "<title>Equo-1 solo: full-model preview</title>").replace(
        "<h1>Centauri-1: Cephalo-1 docked on its horse body (Equo-1)</h1>", "<h1>Equo-1 solo: Centauri-1's horse body running on its own</h1>")
open(os.path.join(HERE, "equo_solo.html" if SOLO else "equo_preview.html"), "w").write(html)
print(f"triangles {ntri}, groups {len(geo)}, html {len(html)/1e6:.2f} MB, mass {sum(mass.values()):.1f} kg")
for r in rules_out: print(("OK  " if r["ok"] else "WARN"), r["rule"], round(r["value"], 2))
