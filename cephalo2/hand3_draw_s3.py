"""hand3 with hub v3 + D winches + S3 cam sleeve (2026-11-20, claude/hand3-hub-v3-cam-s3.md) drawn on the full robot from its CAD
solids, for cephalo2_model.py / viz4.  Same API as hand3_draw.py: install(arms) hooks into viz4; draw(S, pt, Rt, spin, ...).

Static (does not turn): frame, motor base, 8 curl drivers + the sleeve driver, lift ring (lift 0 = flight / parked).
Rotor (turns with the hub by `spin`): cage, motor bell, base disc + brass counterweight, top disc, heel, cam sleeve at the
layout, sleeve screw bracket, screw and nut, and per finger the root + winch leg, cam pin, lead screw, nut slider and the
finger (three phalanges, pads, return bands and dowels), straight (the 9 in rotor blades).  Layout: cross (flight)."""
import os, sys
import numpy as np
_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e")
if _DIR not in sys.path: sys.path.insert(0, _DIR)
import hand3_design as H
import hand3_cad as HC
import hand3_hubc as HB
import hand3_hub_v3 as V3
import hand3_multidigit as MD
import hand3_cam_s3 as S3

MM = 1e-3
CLOCK = float(os.environ.get('H3_CLOCK', '45'))
LAYOUT = os.environ.get("H3_LAYOUT", "cross")
COL = dict(alu=[0.70, 0.72, 0.76], pacf=[0.24, 0.25, 0.27], frame=[0.90, 0.90, 0.87], finger=[0.17, 0.17, 0.19], tpu=[0.78, 0.24, 0.22],
           steel=[0.62, 0.63, 0.66], motor=[0.42, 0.45, 0.50], brass=[0.79, 0.64, 0.16], sleeve=[0.18, 0.62, 0.42], lift=[0.94, 0.63, 0.19],
           bell=[0.10, 0.10, 0.11], band=[0.25, 0.68, 0.35], root=[0.29, 0.44, 0.75])
ARMS = set(); STATE = {}; PARK = {}
_M = None


def _tris(m):
    me = m.to_mesh(); V = np.asarray(me.vert_properties)[:, :3] * MM; F = np.asarray(me.tri_verts).astype(np.int64)
    return V[F]


def _soup(items):
    items = [(m, c) for m, c in items if not m.is_empty()]
    T = np.concatenate([_tris(m) for m, _ in items])
    c = np.concatenate([np.tile(np.asarray(COL[c], float), (len(_tris(m)), 1)) for m, c in items])
    return T, c


def meshes():
    global _M
    if _M is None:
        import hand3_cad as C
        u = dict(zip([l[0] for l in S3.LAYOUTS], S3.PRE_U))[LAYOUT]
        az = S3.theta_at(u); dz = S3.sleeve_dz(u)
        bell = C.cyl(17, H.Z["pack1"] + 8.5, H.Z["motor_top"]) + C.cyl(2.5, H.Z["motor_top"], H.Z["shaft_tip"])
        static = [(S3.frame_s3(), "frame"), (HB.hardware()["motor_base"], "motor"), (MD.drivers(False), "motor"), (S3.driver9(0.0), "motor"),
                  (S3.lift_ring_s3(), "lift")]
        rotor = [(V3.cage_v3(), "pacf"), (bell, "bell"), (S3.base_disc_s3(), "pacf"), (S3.counterweight(), "brass"), (V3.top_disc(), "pacf"),
                 (C.parts()["heel"], "pacf"), (S3.sleeve().translate([0, 0, dz]), "sleeve"), (S3.bracket(), "alu"), (S3.screw9(), "steel"),
                 (S3.nut9().translate([0, 0, dz]), "brass")]
        for a in az:
            rotor += [(S3.root_s3(a), "root"), (S3.pin().rotate([0, 0, a]), "steel"), (MD.screw().rotate([0, 0, a]), "steel"),
                      (MD.slider(0.0).rotate([0, 0, a]), "pacf")]
            for n, m in V3.finger(a).items():
                ck = "tpu" if n.startswith("pad") else ("band" if n.startswith("band") else ("steel" if n.startswith("dowel") else "finger"))
                rotor.append((m, ck))
        _M = dict(static=_soup(static), rotor=_soup(rotor))
    return _M


def _place(soup, A, Rp, p):
    T, c = soup
    R = Rp @ A[:3, :3]; t = Rp @ (A[:3, 3] * MM) + p
    V = T.reshape(-1, 3) @ R.T + t
    return V, np.arange(len(V)).reshape(-1, 3), c


def draw(S, pt, Rt, spin, lag=None, fold=None, fan_on=False, warn=False):
    from geom import torus
    M = meshes()
    u, e1, e2 = Rt[:, 0], Rt[:, 1], Rt[:, 2]
    Rp = np.stack([e1, e2, u], 1) @ HC.rot("z", CLOCK)
    Sp = HC.T(HC.rot("z", np.degrees(spin)))
    S.add(_place(M["static"], np.eye(4), Rp, pt))
    S.add(_place(M["rotor"], Sp, Rp, pt), same=True)
    if fan_on:
        S.add(torus(pt + u * (HC.ZB * MM), u, H.R_TIP * MM, 0.0012, [0.35, 0.37, 0.40], n=48, m=4))


_CUR = [None]


def install(arms=tuple(range(8))):
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
            return draw(S, pt, Rt, spin if fan_on else np.radians(PARK.get(i, 0.0)), None, None, fan_on, warn)
        return pod0(S, pt, Rt, spin, fold, fan_on, warn)
    V.arm_mesh = arm_mesh; XP.draw = pod; V._h3 = True
