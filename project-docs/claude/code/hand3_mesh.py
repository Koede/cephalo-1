"""hand3 (hub C2) as coloured triangle meshes for the Chilo-1 / Cephalo-2 viewer.
Rotor parts (fingers, root grips, lag tubes, hub plates, heel) are the project CAD solids (hand3e/).  The hub C2 drive
(two STS3215 on the base plate, frame, motor SE 2816, two sliding rings) is drawn as its envelope from hand3_hubc.json
(frame 49.2 x 36 x 53.4 mm, rings Ø81.6): the full hubc boolean model is not rebuilt here.
Pod frame: z = 0 at the hand-base face of the bayonet, +z = away from the arm (heel at z 114.2), mm."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import trimesh
import hand3_design as H
import hand3_cad as HC
from hand3_finger import to_trimesh, app, T, rot

COL = dict(pacf=(0.16, 0.17, 0.19), tpu=(0.93, 0.48, 0.16), alu=(0.72, 0.74, 0.77), steel=(0.60, 0.62, 0.65),
           servo=(0.22, 0.42, 0.78), frame=(0.88, 0.88, 0.86), motor=(0.30, 0.31, 0.34), ring=(0.80, 0.55, 0.20))
MAT = dict(root_grip="alu", lag_tube="steel", hub_lower="alu", hub_upper="alu", heel="tpu", phalanx1="pacf", phalanx2="pacf",
           phalanx3="pacf", pad1="tpu", pad2="tpu", pad3="tpu")
CLOCK = 45.0                    # hand clocked 45 deg on the arm (cephalo2_model / hand3_draw)
_cache = {}


def _tm(name):
    if name not in _cache:
        _cache[name] = to_trimesh(HC.base_parts()[name])
    return _cache[name]


def _cyl(r, z0, z1, col, r_in=None):
    if r_in:
        m = trimesh.creation.annulus(r_in, r, height=z1 - z0, sections=48)
    else:
        m = trimesh.creation.cylinder(r, z1 - z0, sections=48)
    m.apply_translation([0, 0, (z0 + z1) / 2]); return m, col


def _box(c, s, col):
    m = trimesh.creation.box(s); m.apply_translation(c); return m, col


def drive():
    out = [_cyl(25.0, 0.0, 2.5, COL["alu"])]
    out.append(_box([0, 0, 2.5 + 53.4 / 2], [49.2, 36.0, 53.4], COL["frame"]))           # printed frame (envelope)
    for sx in (-1, 1):                                                                   # 2 x STS3215 standing on the base plate
        out.append(_box([sx * 12.6, 0, 2.5 + 45.2 / 2 + 0.5], [24.7, 35.0, 45.2], COL["servo"]))
    out.append(_cyl(17.0, 58.0, 89.0, COL["motor"]))                                       # SE 2816 can
    out.append(_cyl(40.8, 66.0, 69.0, COL["ring"], r_in=36.0))                             # sliding rings (envelope Ø81.6)
    out.append(_cyl(40.8, 76.0, 79.0, COL["ring"], r_in=36.0))
    out.append(_cyl(2.5, 89.0, 104.2, COL["steel"]))
    return out


def hand(lag=0.0, fold=(0, 0, 0), with_drive=True, clock=CLOCK):
    """[(trimesh in pod-frame mm, rgb)]."""
    out = []
    for n in ("hub_lower", "hub_upper", "heel"):
        out.append((_tm(n).copy(), COL[MAT[n]]))
    for k in range(4):
        fr = HC.finger_frames(k, H.LAG_SIGN[k] * lag, fold)
        for n, A in (("root_grip", fr[0]), ("lag_tube", T(rot("z", H.FINGER_AZ[k])) @ T(t=[0, 0, HC.ZB])),
                     ("phalanx1", fr[1]), ("pad1", fr[1]), ("phalanx2", fr[2]), ("pad2", fr[2]), ("phalanx3", fr[3]), ("pad3", fr[3])):
            m = _tm(n).copy(); m.apply_transform(A); out.append((m, COL[MAT[n]]))
    if with_drive:
        out += drive()
    Rc = np.eye(4); Rc[:3, :3] = rot("z", clock)
    for m, _ in out: m.apply_transform(Rc)
    return out


if __name__ == "__main__":
    for fold in ((0, 0, 0), (90, 90, 85), (60, 90, 85), (45, 60, 60)):
        V = np.vstack([m.vertices for m, _ in hand(0.0, fold)])
        print(fold, "z max %.1f  r max %.1f" % (V[:, 2].max(), np.hypot(V[:, 0], V[:, 1]).max()))
