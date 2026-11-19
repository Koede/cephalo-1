"""hand3 FINAL (hub C2, claude/hand3-final.md) drawn on the full robot from its CAD solids (hand3e/), for viz4 / the videos.

draw(S, pt, Rt, spin, lag, fold, fan_on): pt, Rt = arm tip (spigot) frame, x = pod axis.
  * Stationary: servo frame (one body with bridge), two STS3215, two-groove spools, MR63 idlers, eight carbon guide rods, tie
    ring, motor base; the four yoke halves move down with their ring's stroke; the drive cords are redrawn every frame.
  * Rotating (turn with the hub by `spin`): hub plates, cage, V30 heel, motor bell, handed lag tubes, spring boxes and their
    screws; the base ring with its four TPU links, trim screws and nuts, and the curl ring with its four, slide down by their
    strokes; the fingers (aluminium root grip, three PA12 phalanges with TPU pads) take the pose (lag, fold).
  * Flight (fan_on): fingers straight = the 9 in four-blade rotor, turning with the hub; a thin disc ring marks the swept circle.
Ring strokes follow the finger pose: base ring = E_LAG x spread + lever x K1, curl ring = levers x (K2, K3) (hand3e/hand3_hubc.py).
STATE[arm] = dict(lag=(4,), fold=(4, 3)) in deg overrides the pose per frame; install(arms) hooks into viz4."""
import os, sys
import numpy as np
_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.environ.get("H3_DIR", "hand3e"))
if _DIR not in sys.path: sys.path.insert(0, _DIR)
import hand3_design as H
import hand3_cad as HC
import hand3_hubc as HB

MM = 1e-3
CLOCK = float(os.environ.get('H3_CLOCK', '45'))          # deg the hand is clocked on the wrist (as hand3_mj / quad4r)
COL = dict(alu=[0.75, 0.77, 0.80], pa12=[0.88, 0.87, 0.82], cf=[0.22, 0.23, 0.25], tpu=[0.78, 0.24, 0.22], steel=[0.60, 0.61, 0.64],
           servo=[0.16, 0.35, 0.67], motor=[0.10, 0.10, 0.11], carbon=[0.12, 0.12, 0.12], cord=[0.94, 0.78, 0.16], link=[0.90, 0.43, 0.16],
           spring=[0.35, 0.63, 0.35], ring_b=[0.30, 0.31, 0.34], ring_c=[0.40, 0.41, 0.44])
ARMS = set()
STATE = {}
PARK = {}
_M = None


def _mat(n):
    if n.startswith(("grip", "hub_")): return "alu"
    if n.startswith("ph"): return "pa12"
    if n.startswith(("pad", "heel")): return "tpu"
    if n.startswith("rlink"): return "link"
    if n.startswith(("cord", "rcord")): return "cord"
    if n.startswith(("stray", "slid")): return "spring"
    if n.startswith(("tube", "hw_rscrew", "hw_rnut", "hw_sscrew", "hw_idler")): return "steel"
    if n.startswith("hw_servo"): return "servo"
    if n.startswith("hw_motor"): return "motor"
    if n.startswith("hw_rod"): return "carbon"
    if n == "ring_out": return "ring_b"
    if n == "ring_in": return "ring_c"
    return "cf"


def _tris(m):
    me = m.to_mesh(); V = np.asarray(me.vert_properties)[:, :3] * MM; F = np.asarray(me.tri_verts).astype(np.int64)
    return V[F]


def _soup(items):
    items = [(m, c) for m, c in items if not m.is_empty()]
    T = np.concatenate([_tris(m) for m, _ in items])
    c = np.concatenate([np.tile(np.asarray(COL[c], float), (len(_tris(m)), 1)) for m, c in items])
    return T, c


def meshes():
    """Triangle soups in metres, hand frame (z = rotor axis, origin at the spigot face), from hand3_hubc.assembly() at rest."""
    global _M
    if _M is None:
        a = HB.assembly()
        lagS = [s * H.SPREAD_MAX for s in H.LAG_SIGN]
        aS = HB.assembly(lag=lagS)                                        # for the base cord's upper span at full spread
        R = HC.parts()
        grp = dict(static=[], yoke_out=[], yoke_in=[], rotor=[], base=[], curl=[], cordup0=[], cordupS=[])
        for n, (m, rot) in a.items():
            if n.startswith(("grip", "ph", "pad")) or n.startswith("cord_"): continue
            if n.startswith("rcordup"): grp["cordup0"].append((m, "cord")); grp["cordupS"].append((aS[n][0], "cord")); continue
            if not rot:
                g = "yoke_out" if n.startswith("yoke_out") else ("yoke_in" if n.startswith("yoke_in") else "static")
            elif n == "ring_out" or n.endswith(tuple(f"base{k}" for k in range(4))): g = "base"
            elif n == "ring_in" or n.endswith(tuple(f"curl{k}" for k in range(4))): g = "curl"
            else: g = "rotor"
            grp[g].append((m, _mat(n)))
        _M = {k: _soup(v) for k, v in grp.items()}
        _M["grip"] = _soup([(R["root_grip"], "alu")])
        _M["ph"] = [_soup([(R[f"phalanx{j + 1}"], "pa12"), (R[f"pad{j + 1}"], "tpu")]) for j in range(3)]
    return _M


def _place(soup, A, Rp, p):
    T, c = soup
    R = Rp @ A[:3, :3]; t = Rp @ (A[:3, 3] * MM) + p
    V = T.reshape(-1, 3) @ R.T + t
    return V, np.arange(len(V)).reshape(-1, 3), c


def strokes(lag, fold):
    """Ring strokes (mm) that go with a finger pose (deg): mean over the four fingers, clipped to the travel."""
    lv = HB.LEVER
    pb = HB.E_LAG * np.radians(np.abs(lag).mean()) + lv[0] * np.radians(max(fold[:, 0].mean(), 0.0))
    pc = lv[1] * np.radians(max(fold[:, 1].mean(), 0.0)) + lv[2] * np.radians(max(fold[:, 2].mean(), 0.0))
    return float(np.clip(pb, 0, HB.TRAVEL_B)), float(np.clip(pc, 0, HB.TRAVEL_C))


def draw(S, pt, Rt, spin, lag=None, fold=None, fan_on=False, warn=False):
    from geom import torus, frustum
    M = meshes()
    u, e1, e2 = Rt[:, 0], Rt[:, 1], Rt[:, 2]
    Rp = np.stack([e1, e2, u], 1) @ HC.rot("z", CLOCK)               # hand frame: z = pod axis, clocked on the wrist
    lag = np.zeros(4) if lag is None else np.asarray(lag, float)
    fold = np.zeros((4, 3)) if fold is None else np.asarray(fold, float).reshape(4, 3)
    if fan_on: lag = np.zeros(4); fold = np.zeros((4, 3))
    pb, pc = strokes(lag, fold)
    I = np.eye(4); Sp = HC.T(HC.rot("z", np.degrees(spin)))
    S.add(_place(M["static"], I, Rp, pt))
    for nm, pull in (("yoke_out", pb), ("yoke_in", pc)):
        S.add(_place(M[nm], HC.T(t=[0, 0, -(pull + HB.GAP) if pull > 0.2 else 0.0]), Rp, pt), same=True)
    S.add(_place(M["rotor"], Sp, Rp, pt), same=True)
    S.add(_place(M["base"], Sp @ HC.T(t=[0, 0, -pb]), Rp, pt), same=True)
    S.add(_place(M["curl"], Sp @ HC.T(t=[0, 0, -pc]), Rp, pt), same=True)
    S.add(_place(M["cordupS" if np.abs(lag).mean() > H.SPREAD_MAX / 2 else "cordup0"], Sp, Rp, pt), same=True)
    for n, (p, q) in HB.cord_runs(pb if pb > 0.2 else 0.0, pc if pc > 0.2 else 0.0).items():     # drive cords, spool -> idlers -> yoke eye
        P0 = Rp @ (np.asarray(p) * MM) + pt; P1 = Rp @ (np.asarray(q) * MM) + pt
        S.add(frustum(P0, P1, 0.00045, 0.00045, COL["cord"], n=6), same=True)
    for k in range(4):
        Fr = HC.finger_frames(k, lag[k], fold[k])
        S.add(_place(M["grip"], Sp @ Fr[0], Rp, pt), same=True)
        for j in range(3):
            S.add(_place(M["ph"][j], Sp @ Fr[j + 1], Rp, pt), same=True)
    if fan_on:
        S.add(torus(pt + u * (HC.ZB * MM), u, H.R_TIP * MM, 0.0012, [0.35, 0.37, 0.40], n=48, m=4))


def grasp_pose(fold=(55, 60, 50)):
    return np.array(H.LAG_SIGN) * H.SPREAD_MAX, np.tile(np.asarray(fold, float), (4, 1))


_CUR = [None]


def install(arms=(2, 6)):
    """Hook into viz4 without editing it: viz4.arm_mesh draws the arm, then the pod through xlpod4.draw; for a hand arm the pod
    call is answered by draw() above."""
    import viz4 as V, xlpod4 as XP
    ARMS.update(arms)
    if getattr(V, "_h3", False): return
    arm0, pod0 = V.arm_mesh, XP.draw
    def arm_mesh(S, i, *a, **k):
        _CUR[0] = i
        try: return arm0(S, i, *a, **k)
        finally: _CUR[0] = None
    def pod(S, pt, Rt, spin, fold, fan_on, warn=False):
        i = _CUR[0]
        if i in ARMS:
            st = STATE.get(i, {})
            return draw(S, pt, Rt, spin if fan_on else np.radians(PARK.get(i, 0.0)), st.get("lag"), st.get("fold"), fan_on, warn)
        return pod0(S, pt, Rt, spin, fold, fan_on, warn)
    V.arm_mesh = arm_mesh; XP.draw = pod; V._h3 = True
