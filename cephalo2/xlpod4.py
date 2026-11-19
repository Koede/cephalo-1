"""The XL claw-prop (xl-claw.md: 7.5 in Snap-Digit on a T-Motor F90 2806.5) on the rev 4 full-robot model (2026-11-13).

One place that switches a script from the 6 in pod to the XL pod as it is designed in xlclaw.py / xlclaw_cad.py:
  install()   call it right after importing cephalo4 and BEFORE sim4 / body4 / flip4 / viz4 / states4 are imported:
              - prop disc radius 95.25 mm (C.CLAW_R);
              - folded-finger toe from the real finger: root pin 16.25 mm off the axis, finger 79 mm from the pin
                (C.TOE -> cephalo4.toe / foot_drop and sim4's toe contact spheres). The old model was a 95 mm stick
                from the hub centre: 12 mm longer than the finger is;
              - viz4.arm_mesh draws the pod from the CAD (C.XL_POD).
  draw(...)   the pod meshes for viz4: F90 motor body, hub + palm cap, skirt + bearing ring + push rods, three two-phalanx
              fingers (proximal with lug / tongue / fairlead, distal with the seat pocket), the steel tape with its fold.
Pod mass: power4.PROPS["xl"]["dM_pod"] (156 g itemised in xlclaw.py, was a 190 g budget); thrust model unchanged.
Fold used by the full-robot videos: root fold only (2 deg precone in flight, 60 deg as feet), knuckles on their seats.
"""
import numpy as np
import cephalo4 as C
import xlclaw as XL

MM = XL.MM
M_POD = 0.156                                          # kg, itemised in xlclaw.loads() (power4 carries it as dM_pod)
PRE = np.radians(XL.PRECONE)
_LEN = (XL.R_TIP - XL.R_PIN) * MM                      # finger length from the root pin


def toe(fold=C.FOLD_STAND):
    """(radial, axial) position of the fingertip from the hub centre for a root fold (rad), knuckle seated."""
    a = max(float(fold), PRE)
    return XL.R_PIN * MM + _LEN * np.cos(a), XL.Z_PIN * MM + _LEN * np.sin(a)


def install():
    C.CLAW_R = XL.R_TIP * MM
    C.TOE = toe
    C.XL_POD = True


def _mesh(tris, R, p, col):
    V = tris.reshape(-1, 3) @ np.asarray(R).T + p
    n = len(tris)
    return V, np.arange(3 * n).reshape(n, 3), np.tile(np.asarray(col, float), (n, 1))


BLADE = [0.16, 0.17, 0.19]; ALU = [0.74, 0.76, 0.79]; PRINT = [0.93, 0.56, 0.16]; MOTOR = [0.13, 0.13, 0.15]; RING = [0.55, 0.57, 0.60]


def draw(S, pt, Rt, spin, fold, fan_on, warn=False):
    """Adds the XL pod to scene S. pt, Rt: arm tip (spigot) frame, x = pod axis. spin (rad), fold (rad: root fold)."""
    import xlclaw_cad as XC
    from geom import frustum, torus
    m = XC.meshes()
    u, e1, e2 = Rt[:, 0], Rt[:, 1], Rt[:, 2]
    hub = pt + u * C.BLADE_OFF
    Rp = np.stack([e1, e2, u], 1)                         # pod frame: z = pod axis
    base = hub + u * XL.BELL_Z[0] * 1e-3
    if (base - pt) @ u > 1e-4:                              # printed mount between the tip bay and the motor base
        S.add(frustum(pt, base, 0.018, 0.0167, [0.88, 0.15, 0.12] if warn else [0.35, 0.36, 0.40], n=18))
    S.add(_mesh(m["stator"], Rp, hub, [0.88, 0.15, 0.12] if warn else MOTOR))
    Rr = Rp @ XL.rotm([0, 0, 1], spin)
    S.add(_mesh(m["rotor"], Rr, hub, ALU))
    a1 = max(float(fold), PRE)
    us = np.clip((a1 - PRE) / np.radians(XL.FOLD1_MAX), 0, 1) * 0.6          # root fold uses the first part of the slider travel
    S.add(_mesh(m["slider"], Rr, hub + u * (XL.SKIRT_TOP_OPEN - XL.SKIRT["travel"] * us) * MM, RING))
    pin = np.array([XL.R_PIN, 0, XL.Z_PIN]) * MM
    for k in range(3):
        Rk = Rr @ XL.rotm([0, 0, 1], np.radians(120 * k))
        Rpx = Rk @ XL.rotm([0, -1, 0], a1); ppx = hub + Rk @ pin
        Rd = Rpx @ XL.rotm(XL.AX_K, PRE); pd = ppx + Rpx @ (XL.PIV * MM - pin)
        S.add(_mesh(m["prox"], Rpx, ppx, BLADE), same=k > 0)
        S.add(_mesh(m["dist"], Rd, pd, BLADE), same=True)
        S.add(_mesh(np.concatenate([m["tape_p"] @ Rpx.T + ppx, m["tape_d"] @ Rd.T + pd, XC.free_tape((Rpx, ppx), (Rd, pd), 8)]), np.eye(3), np.zeros(3), XC.TAPE_COL), same=True)
    if fan_on:
        S.add(torus(hub + u * 0.002, u, C.CLAW_R, 0.0012, [0.35, 0.37, 0.40], n=40, m=4))
