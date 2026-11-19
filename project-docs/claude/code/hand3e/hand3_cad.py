"""hand3 rev D solids (rev C code + the rev D pack-frame radius from claude/code/hand3_revD.diff) (workspace copy of the project's claude/code/hand3_cad.py; finger functions live in hand3_finger.py,
which holds them verbatim).  Pod frame: z = 0 at the arm's spigot face, +z = thrust, rotor axis = z."""
import numpy as np
import manifold3d as mf
import hand3_design as H
from hand3_finger import (cyl, box, half, wedge, T, rot, app, about, loft, to_trimesh, root_grip, phalanx, pad)
import json

M_ = mf.Manifold
KN, RT, RL = H.KN, H.R_TIP, H.R_LAG
ZB = H.Z["blade"]; Zs = H.Z
FAZ = H.FINGER_AZ


def lag_tube():
    t = cyl(H.LAG_TUBE[0] / 2, -5.0, 7.5, c=(RL, 0, 0)) - cyl(H.LAG_TUBE[1] / 2, -6, 8, c=(RL, 0, 0))
    for z0, z1 in ((-0.8, 0.8), (5.3, 6.9)): t -= box([RL, -3, z0], [RL + 3.5, 3, z1])
    return t


def follower_az():
    return [c["az"] if c["finger"] is None else FAZ[c["finger"]] + H.FOLLOW_DAZ[c["role"]] for c in H.CHANNELS]


def stop_posts():
    out = []
    for k in range(4):
        s = H.LAG_SIGN[k]
        for lag, y in ((0.0, s * 7.55), (s * H.SPREAD_MAX, -s * 7.55)):
            p = rot("z", FAZ[k]) @ (np.array([RL, 0, 0]) + rot("z", lag) @ np.array([-4.0, y, 0]))
            out.append(cyl(1.5, ZB - 2.5 - 0.3, ZB + 2.5, c=(p[0], p[1], 0)))
    return out


def hub_holes(m):
    m -= cyl(2.6, ZB - 15, ZB + 15)                                   # (was z 80-100: absolute heights broke with STACK_DZ)
    for a in FAZ: m -= app(cyl(H.LAG_TUBE[0] / 2 + 0.02, ZB - 15, ZB + 15, c=(RL, 0, 0)), T(rot("z", a)))
    return m


def hub_lower():
    rp = H.HUB["plate_d"] / 2
    m = cyl(rp, Zs["hub_lo"], Zs["hub_lo"] + 2.5) + cyl(7.0, Zs["motor_top"], ZB + 2.5)
    for p in stop_posts(): m += p
    return hub_holes(m)


SPRING_SCREW = dict(r=13.0, tap=1.6)            # M2 x 12 holding each finger's spring box (hand3_springs.BOX["screw"]); tapped M2 in the upper plate


def hub_upper():
    m = hub_holes(cyl(H.HUB["plate_d"] / 2, ZB + 2.5, ZB + 5.0))
    for a in FAZ: m -= app(cyl(SPRING_SCREW["tap"] / 2, ZB + 2.0, ZB + 5.5, c=(SPRING_SCREW["r"], 0, 0), n=20), T(rot("z", a)))
    return m


def cage():
    r0, r1 = H.HUB["cage_r"]; ft = Zs["fin_top"]
    m = cyl(r1, Zs["cage_lo"], Zs["hub_lo"], n=96) - cyl(r0, 50, 90, n=96)
    slot = box([20.5, -1.9, ft - 8 - H.HUB["follow_travel"] - 0.5], [21.7, 1.9, 88]) + box([21.6, -1.1, ft - 8 - H.HUB["follow_travel"] - 0.5], [23.5, 1.1, 88])
    for a in follower_az(): m -= app(slot, T(rot("z", a)))
    pocket = cyl(23.6, ft + 1.5, 88, n=96) - cyl(18.4, 80, 90)
    for a in FAZ:
        w = app(half(half(pocket, [np.sin(np.radians(16)), np.cos(np.radians(16)), 0], [0, 0, 0]), [np.sin(np.radians(16)), -np.cos(np.radians(16)), 0], [0, 0, 0]), T(rot("z", a)))
        m -= w
    return m


def follower():
    ft = Zs["fin_top"]
    m = box([20.7, -1.7, ft - 8], [21.5, 1.7, ft]) + box([21.2, -0.9, ft - 8], [23.4, 0.9, ft]) + box([23.1, -2.3, ft - 8], [27.2, 2.3, ft])
    m += box([26.9, -2.3, ft - 2], [H.HUB["fin_r"][1], 2.3, ft])
    return m - cyl(0.55, ft - 9, ft + 1, c=(24.4, 0, 0)) - cyl(1.3, ft - 9, ft - 5, c=(24.4, 0, 0))


# Heel (TPU, on the rotor axis; also the foot the robot stands on).  hand3e: Ø16 post with a flat top at Z heel.
# Proposal (shovel_f, 2026-11-15): V SADDLE across the heel along the grip axis (hand y), so a handle lying on the heel is
# cradled on two flanks and cannot walk sideways across the heel (the shovel walk).  The lips stay at Z heel (the foot
# height and every sim's foot point are unchanged); the bar seats lower: a Ø28 bar's centre at 125.7 mm instead of 128.2 (apex 109.58).
# HEEL_V = None gives the hand3e heel.  Footprint: rounded rectangle 2*X_HALF (across) x 2*Y_HALF (along the grip axis).
HEEL_V = dict(alpha=30.0, w=8.0, x_half=9.0, y_half=8.5, corner_r=3.0, lip_chamfer=0.4, apex_r=0.8)   # flanks to x +-8 (Ø28 touches at x +-7), 1 mm lips; 1 mm wider each side across the grip axis than the Ø16 post. (alpha 25, w 7, x_half 8 stays inside Ø16 but lets the shaft walk with the rotor parked 5 deg off)


HEEL_LID_CLR = 0.3          # mm above the spring-box lids before the V-saddle widens past the Ø16 post


def heel_apex():
    v = HEEL_V; return Zs["heel"] - v["w"] * np.tan(np.radians(v["alpha"]))


def heel():
    bore = cyl(4.7, ZB + 4, Zs["shaft_tip"] + 0.8)
    if HEEL_V is None: return cyl(8.0, ZB + 5.0, Zs["heel"]) - bore
    v = HEEL_V; z0, z1 = ZB + 5.0, Zs["heel"]; xr, yr, cr = v["x_half"], v["y_half"], v["corner_r"]
    m = box([-xr + cr, -yr, z0], [xr - cr, yr, z1]) + box([-xr, -yr + cr, z0], [xr, yr - cr, z1])
    for sx in (-1, 1):
        for sy in (-1, 1): m += cyl(cr, z0, z1, c=(sx * (xr - cr), sy * (yr - cr), 0), n=24)
    a = np.radians(v["alpha"]); za = heel_apex(); L = 3 * xr
    cut = box([-L, -yr - 1, za], [L, yr + 1, z1 + 20])        # the V: above both flank planes through the apex line (at alpha from the heel face)
    for sx in (-1, 1): cut = half(cut, [-sx * np.sin(a), 0.0, np.cos(a)], [0, 0, za])
    m -= cut
    if v["apex_r"] > 0:                                  # round the crease (print): fill the V bottom under a circle of apex_r tangent to both flanks
        r = v["apex_r"]; zc = za + r / np.cos(a)
        m += box([-r * np.sin(a), -yr, za - 0.5], [r * np.sin(a), yr, zc - r * np.cos(a)]) - cyl(r, -yr - 1, yr + 1, "y", (0, 0, zc))
    if v["lip_chamfer"] > 0:                             # break the outer lip edges (45 deg x lip_chamfer)
        c = v["lip_chamfer"]
        for sx in (-1, 1):
            m -= half(half(box([-L, -yr - 1, z1 - 2 * c], [L, yr + 1, z1 + 1]), [sx, 0, 1.0], [sx * (xr - c), 0, z1]), [sx, 0, 0], [0, 0, 0])
    # below the spring-box lids the heel keeps the old Ø16 post: the four spring boxes wrap round it (lid top ZB + 15)
    z_lid = ZB + 15.0 + HEEL_LID_CLR
    m = (m ^ (cyl(8.0, z0 - 1, z_lid, n=96) + box([-20, -20, z_lid], [20, 20, z1 + 1]))) if z_lid < z1 else m
    return m - bore


def claw_bar(c, az_f):
    B = H.BAR; r0, r1 = B["r"]; w = B["w"] / 2
    bar = box([r0, -w, B["z0"]], [r1, w, Zs["claw_hi"] + 0.4]) - box([r1 - B["slot"][1], -B["slot"][0] / 2, B["z0"] + 1.4], [r1 + 1, B["slot"][0] / 2, Zs["claw_hi"] - 3])
    d = az_f - c["az"]
    tip = app(box([H.HUB["claw_tip_r"], -2.3, Zs["claw_lo"]], [33.0, 2.3, Zs["claw_hi"] + 0.4]), T(rot("z", d)))
    neck = app(box([32.0, -2.3, Zs["claw_lo"]], [33.5, 2.3, Zs["claw_hi"] + 0.4]), T(rot("z", d)))
    root = box([r0 - 2.6, -w, B["arm_root_z"]], [r0 + 0.5, w, Zs["claw_hi"] + 0.4])
    arm = M_.batch_hull([box([r0 - 2.6, -w, B["arm_root_z"]], [r0 - 2.0, w, Zs["claw_hi"] + 0.4]), neck]) + root + tip
    return app(bar + arm, T(rot("z", c["az"])))


def pack_frame():
    ro = H.PACK["spool_r_pos"] + 2.5                                    # 51 in rev C, 47 in rev D
    web = cyl(ro - 9, -1, 60, n=96) - cyl(13, -2, 61)
    for c in H.CHANNELS: web -= app(box([10, -1.5, -3], [ro - 6, 1.5, 62]), T(rot("z", c["az"])))
    m = cyl(ro, 0, 2.5, n=96) + cyl(ro, 26.4, 27.4, n=96) + cyl(ro, 51.5, 54, n=96) - web + cyl(10, 2, 52, n=64)
    m -= cyl(7, -1, 60)
    B = H.BAR; r0, r1 = B["r"]; w = B["w"] / 2; sw = B["sleeve_wall"]
    for c in H.CHANNELS:
        zb = 27.4 if c["tier"] == 1 else 51.5
        sl = box([r0 - sw - 0.1, -w - sw - 0.1, zb], [r1 + sw + 0.1, w + sw + 0.1, B["sleeve_top"]])
        cut = box([r0 - 0.1, -w - 0.1, 20], [r1 + 0.1, w + 0.1, 75]) + box([r0 - sw - 1, -w - 0.1, 57.0], [r0, w + 0.1, 75])
        cord = cyl(1.0, 20, 60, c=(H.PACK["spool_r_pos"], 0, 0))
        A = T(rot("z", c["az"]))
        m = m + app(sl, A) - app(cut, A) - app(cord, A)
    return m


def hardware():
    out = {}
    for c in H.CHANNELS:
        sv = H.SERVOS["SCS0009" if c["role"] == "tip" else "STS3032"]; hgt = sv["size"][2]; z0 = 3.0 if c["tier"] == 1 else 27.6
        A = T(rot("z", c["az"])); r1 = H.PACK["spool_r_pos"] - 2.0
        out["servo_" + c["name"].replace(" ", "_")] = (app(box([r1 - hgt, -6.05, z0], [r1, 6.05, z0 + 23.2]), A), False, None)
        rs = {"spread": 3.5, "middle": 3.0}.get(c["role"], 2.5)        # rev D: middle spool r 3.0 (K2 + K3 cord)
        out["spool_" + c["name"].replace(" ", "_")] = (app(cyl(rs + 1.0, r1 + 0.2, r1 + 4.2, "x", (0, 0, z0 + 11.6)), A), False, "pacf")
    out["motor_base"] = (cyl(17, Zs["pack1"], Zs["pack1"] + 8), False, None)
    out["motor_bell"] = (cyl(17, Zs["pack1"] + 8.5, Zs["motor_top"]) + cyl(2.5, Zs["motor_top"], Zs["shaft_tip"]), True, None)
    return out


# ------------------------------------------------------------------ parts / frames for hub C (same as the project's hand3_cad.py)
_P = None
_B = None
def base_parts():
    """Rotor parts WITHOUT the return-spring features (the finger solids of hand3_finger.py, hub plates, lag tube, heel)."""
    global _B
    if _B is None:
        _B = dict(root_grip=root_grip(), lag_tube=lag_tube(), hub_lower=hub_lower(), hub_upper=hub_upper(), heel=heel())
        for j in range(3): _B[f"phalanx{j + 1}"] = phalanx(j); _B[f"pad{j + 1}"] = pad(j)
    return _B


# SPRING HOOK (the one place where return-spring features enter the rotor parts).  Drop the chosen spring module into this
# folder as hand3_springs.py with a function patch_parts(base) -> dict that takes base_parts() (do not call parts() from it)
# and returns the same keys with the spring features cut / added (it may add keys, e.g. root_grip_L / root_grip_R for handed
# grips).  Everything else (hand3_hubc.assembly, the test harness, the kit) reads parts() and picks the springs up.
try:
    import hand3_springs as SPRINGS
except ImportError:
    SPRINGS = None


def parts():
    global _P
    if _P is None:
        b = dict(base_parts())
        _P = SPRINGS.patch_parts(b) if (SPRINGS is not None and hasattr(SPRINGS, "patch_parts")) else b
    return _P


def finger_frames(k, lag=0.0, fold=(0, 0, 0)):
    G = T(t=[0, 0, ZB]) @ T(rot("z", FAZ[k])) @ about("z", lag, [RL, 0, 0])
    C = G @ T(rot("x", H.PITCH)); out = [G]
    for j in range(3):
        C = C @ about("y", -fold[j], [KN[j], 0, 0]); out.append(C)
    return out


RHO = dict(pacf=1.25, tpu=1.20, alu=2.70, steel=7.85)
MAT = dict(spring_tray1_L="pacf", spring_tray1_R="pacf", spring_tray2_L="pacf", spring_tray2_R="pacf", spring_lid_L="pacf", spring_lid_R="pacf", root_grip="alu", lag_tube="steel", hub_lower="alu", hub_upper="alu", heel="tpu", phalanx1="pacf", phalanx2="pacf", phalanx3="pacf", pad1="tpu", pad2="tpu", pad3="tpu")
if __name__ == "__main__":
    rep = {n: dict(mass_g=round(float(m.volume()) * RHO[MAT.get(n, "pacf")] / 1000, 2), bodies=len(m.decompose())) for n, m in parts().items()}
    json.dump(rep, open("hand3_cad_parts.json", "w"), indent=1); print(rep)
