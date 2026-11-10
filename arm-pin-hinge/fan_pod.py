#!/usr/bin/env python3
"""
fan_pod.py  --  folding fan pod for the Cephalo-1 claw (Forest picked it Nov 10: "1")

The claw fan (T-Motor MN5008 KV400, 16x5.4 2-blade) sits on a pod that hinges off the arm's tip plate and closes like
a thumb against a fixed finger. The hinge axis crosses the fan's own axis, so the fan's thrust passes through the
hinge at every pod angle and never loads the hinge servo; the servo only moves the pod and squeezes.

  palm    : bolts to the tip plate (4 x M3 on a 30 mm circle), carries the STS3032 in a cradle and the fixed finger
  servo   : Feetech STS3032 (20 g, 4.5 kg.cm at 6 V), output shaft on the hinge axis (alpha = x here)
  pod     : yoke with two cheeks round the servo, horn side bolted to the servo horn, idler side on a 683 bearing on an
            M3 shoulder screw into the cradle's end cap; a motor plate carries the MN5008 (55.6 x 32 mm, 132 g) and a
            thumb pad beside the motor bell
  finger  : fixed column on the palm, 4 mm from the thumb pad with the pod straight (flight), TPU pads [A]

Pod angle theta: 0 = fan straight along the arm (flight), negative = closing on the finger, positive = folding aside
(up to 88 deg, where the motor clears the palm). Tip frame: origin at the tip plate face centre, z out along the arm.

  python3 fan_pod.py      # writes out_1m_13/fan_pod/ (STLs, assemblies at 0/45/88 deg, fan_pod.json)
"""
import json, math, os, sys
import numpy as np
from manifold3d import Manifold
import trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import pin_cad as C
from pin_cad import box, cyl_x, cyl_z, to_trimesh

R_ARM = 28.55
# servo: Feetech STS3032, 23.2 x 12.1 x 28.5 mm (seller's page, Nov 10); 28.5 taken along the output shaft (x),
# 23.2 along z, output 5.5 mm from the body's front end [A]
SV = dict(name="STS3032", Lx=28.5, Wy=12.1, Lz=23.2, front=5.5, g=20.0, stall_Nm=4.5 * 0.0981)
MOTOR = dict(name="T-Motor MN5008 KV400", D=55.6, H=32.0, g=132.0, holes_r=12.5)   # hole circle [A: check drawing]
PROP = dict(name="16x5.4 CF 2-blade", R=16 * 25.4 / 2, g=26.0, hub_up=21.0)          # prop plane above the motor top: 21 mm adapter leaves room for the index collar [A]
ZP = 34.0                     # hinge axis height above the tip plate
PALM_T = 4.0
PLATE_D, PLATE_T = 12.0, 3.0  # motor plate back face from the hinge, thickness
CHEEK = (16.5, 19.5)          # |x| of the pod cheeks (horn face at 16.5 [A])
CAP = (-16.0, -SV["Lx"] / 2)  # cradle end cap (idler side) in x
PAD_Y = (-36.0, -30.0)        # thumb pad (pod frame)
PAD_Z = (PLATE_D, PLATE_D + 32.0)
FING_Y = (-45.0, -40.0)       # finger (tip frame)
FING_TOP = 78.0
W_PAD = 8.0                   # half width of pad and finger in x
THETA = (-4.0, 88.0)
# prop parking: a 2-lobe index collar on the prop adapter (rotating) and a sprung post on the palm. Over the last
# degrees of the fold the collar lands on the post; the lobes' slopes turn the stopped prop until the post sits in a
# valley, which leaves the blades along x (across the swing plane), and the post then holds it there.
COLLAR = dict(r_min=11.0, lobe=2.0, a=(55.0, 60.0))     # radius at the valleys, lobe height, axial span from the hinge
POST = dict(x=2.5, y=(55.5, 59.5))                      # post half-width in x and its y span (tip frame)
POST_PRELOAD = 0.3                                       # mm: post top below the collar's valley when folded


def rot_x(m, deg, about_z=ZP):
    return m.translate([0, 0, -about_z]).rotate([-deg, 0, 0]).translate([0, 0, about_z])   # +deg folds the fan toward +y


def palm():
    p = cyl_z(24.0, 0, PALM_T)
    p = p + box(-W_PAD, W_PAD, FING_Y[0], -18.0, 0, PALM_T)                     # tab out to the finger
    for k in range(4):
        t = math.radians(45 + 90 * k)
        p = p - cyl_z(1.7, -1, PALM_T + 1, 15 * math.cos(t), 15 * math.sin(t)) - cyl_z(3.0, 2.5, PALM_T + 1, 15 * math.cos(t), 15 * math.sin(t))
    # cradle: pedestal under the servo, side straps, end cap with the idler screw insert
    z_sv0 = ZP - (SV["Lz"] - SV["front"])
    hx, hy = SV["Lx"] / 2, SV["Wy"] / 2
    cr = box(-hx, hx, -hy - 2, hy + 2, PALM_T - 0.01, z_sv0) + box(-hx, hx, -hy - 2, -hy, z_sv0 - 0.01, ZP - 8) + box(-hx, hx, hy, hy + 2, z_sv0 - 0.01, ZP - 8)
    cap = box(CAP[0], CAP[1] + 0.01, -hy - 2, hy + 2, PALM_T - 0.01, ZP) + cyl_x(6.0, CAP[0], CAP[1] + 0.01, 0, ZP)
    cap = cap - cyl_x(C.INSERT_R, CAP[0] - 1, CAP[0] + 6, 0, ZP)
    finger = box(-W_PAD, W_PAD, FING_Y[0], FING_Y[1], PALM_T - 0.01, FING_TOP) + box(-3, 3, FING_Y[0], -18.0, PALM_T - 0.01, 20.0)   # column + rib
    zc = ZP + (COLLAR["a"][0] + COLLAR["a"][1]) / 2 * math.cos(math.radians(THETA[1])) - COLLAR["r_min"] - POST_PRELOAD
    post = box(-4.0, 4.0, 18.0, POST["y"][1], 0, PALM_T) + box(-POST["x"], POST["x"], POST["y"][0], POST["y"][1], PALM_T - 0.01, zc)
    return p + cr + cap + finger + post


def servo_solid():
    z_sv0 = ZP - (SV["Lz"] - SV["front"])
    return box(-SV["Lx"] / 2, SV["Lx"] / 2, -SV["Wy"] / 2, SV["Wy"] / 2, z_sv0, ZP + SV["front"]) + cyl_x(5.0, SV["Lx"] / 2, CHEEK[0], 0, ZP)


def pod_local():
    """pod at theta = 0 (tip frame)"""
    z0 = ZP + PLATE_D
    plate = cyl_z(26.0, z0, z0 + PLATE_T)
    plate = plate - cyl_z(7.0, z0 - 1, z0 + PLATE_T + 1)
    for k in range(4):
        t = math.radians(45 + 90 * k)
        plate = plate - cyl_z(1.7, z0 - 1, z0 + PLATE_T + 1, MOTOR["holes_r"] * math.cos(t), MOTOR["holes_r"] * math.sin(t))
    cheeks = Manifold()
    for sgn in (1, -1):
        x0, x1 = (CHEEK[0], CHEEK[1]) if sgn > 0 else (-CHEEK[1], -CHEEK[0])
        ch = cyl_x(10.0, x0, x1, 0, ZP) + box(x0, x1, -10.0, 10.0, ZP, z0 + 0.01)
        ch = ch - cyl_x(3.5 if sgn < 0 else 1.0, x0 - 1, x1 + 1, 0, ZP)                     # 683 bore (idler) / centre (horn)
        if sgn > 0:
            for k in range(4):                                                              # horn screws [A: M2 on 10 mm]
                t = math.radians(45 + 90 * k)
                ch = ch - cyl_x(1.1, x0 - 1, x1 + 1, 5 * math.cos(t), ZP + 5 * math.sin(t))
        cheeks = cheeks + ch
    pad = box(-W_PAD, W_PAD, PAD_Y[0], PAD_Y[1], ZP + PAD_Z[0], ZP + PAD_Z[1]) + box(-W_PAD, W_PAD, PAD_Y[1] - 0.01, -20.0, ZP + PAD_Z[0], ZP + PAD_Z[0] + PLATE_T)
    return plate + cheeks + pad


def motor_local():
    z0 = ZP + PLATE_D + PLATE_T
    return (cyl_z(MOTOR["D"] / 2, z0, z0 + MOTOR["H"]) + cyl_z(4.0, z0 + MOTOR["H"], z0 + MOTOR["H"] + PROP["hub_up"])
            + cyl_z(COLLAR["r_min"], ZP + COLLAR["a"][0], ZP + COLLAR["a"][1]))


def prop_disc_local(t=1.0):
    z = ZP + PLATE_D + PLATE_T + MOTOR["H"] + PROP["hub_up"]
    return cyl_z(PROP["R"], z - t / 2, z + t / 2), z


def main():
    out = os.path.join(HERE, "out_1m_13", "fan_pod"); os.makedirs(os.path.join(out, "stl"), exist_ok=True)
    pa, sv, pod0, mot0 = palm(), servo_solid(), pod_local(), motor_local()
    disc0, z_prop = prop_disc_local()
    arm = cyl_z(R_ARM, -120, 0)            # the arm's last ring behind the tip plate
    fixed = pa + sv
    g = lambda m: m.volume() * C.PETG_RHO * C.EFF_FILL
    # sweep the pod over its travel: interference of pod + motor with the palm/servo/finger and the arm
    sweep = []
    for th in np.arange(THETA[0], THETA[1] + 0.01, 1.0):
        moving = rot_x(pod0 + mot0, th)
        v_fixed = (moving ^ (pa - finger_only())).volume() if False else (moving ^ fixed).volume()
        v_arm = (moving ^ arm).volume()
        sweep.append((float(th), float(v_fixed), float(v_arm)))
    bad = [s for s in sweep if s[1] > 1e-3 or s[2] > 1e-3]
    # empty-claw closing angle: first angle (going negative from 0) where the pad touches the finger
    close = None
    for th in np.arange(0, -20, -0.25):
        if (rot_x(pod0, th) ^ fixed).volume() > 1e-3: close = float(th); break
    # opening between thumb pad face and finger face at mid-pad height, by angle
    def gap(th):
        zr = (PAD_Z[0] + PAD_Z[1]) / 2; t = math.radians(th)
        return round(PAD_Y[0] * math.cos(t) + zr * math.sin(t) - FING_Y[1], 1)
    # prop disc clearances with the pod straight: to the finger top and to the arm
    prop_to_finger = z_prop - 0.5 - FING_TOP
    # prop at 88 deg folded: disc plane distance from the arm axis
    t88 = math.radians(THETA[1]); fold_plane_y = (z_prop - ZP) * math.sin(t88)
    # grip: servo torque over the pinch radius (mid pad), 2x rule on the servo, rubber pads mu 0.6 on both sides [A]
    r_pinch = math.hypot(PAD_Y[0], (PAD_Z[0] + PAD_Z[1]) / 2) / 1000
    F_half = SV["stall_Nm"] / 2 / r_pinch; F_stall = SV["stall_Nm"] / r_pinch
    hold = lambda F: 2 * 0.6 * F / 9.81
    # gyroscopic moment on the hinge when tilting the spinning fan [A: prop I = m L^2 / 12, rotor 70 g at 25 mm, 5,500 rpm hover]
    I = 0.026 * (2 * PROP["R"] / 1000) ** 2 / 12 + 0.070 * 0.025 ** 2; w = 5500 * 2 * math.pi / 60
    gyro = {f"{r} rad/s": round(I * w * r, 3) for r in (0.25, 0.5, 1.0)}
    # pod weight about the hinge (motor + prop + pod), worst case horizontal
    pod_g = g(pod0); m_pod = (pod_g + MOTOR["g"] + PROP["g"]) / 1000
    cg = (pod_g * (PLATE_D + 10) + MOTOR["g"] * (PLATE_D + PLATE_T + MOTOR["H"] / 2) + PROP["g"] * (z_prop - ZP)) / (pod_g + MOTOR["g"] + PROP["g"]) / 1000
    m_grav = m_pod * 9.81 * cg
    # parking: where the collar first meets the post (lobe tip), the post's distance from the blades, servo load
    zc_mid = (COLLAR["a"][0] + COLLAR["a"][1]) / 2
    post_top = ZP + zc_mid * math.cos(math.radians(THETA[1])) - COLLAR["r_min"] - POST_PRELOAD
    engage = None
    for th in np.arange(60, THETA[1] + 0.01, 0.1):
        t = math.radians(th); low = ZP + zc_mid * math.cos(t) - (COLLAR["r_min"] + COLLAR["lobe"])
        if low <= post_top and zc_mid * math.sin(t) >= POST["y"][0]: engage = round(float(th), 1); break
    blade_root_a = z_prop - ZP - 4.0               # blade root lower face, axial from the hinge [A: 8 mm thick hub]
    post_a_max = max(y * math.sin(math.radians(t)) + (z - ZP) * math.cos(math.radians(t))
                     for t in np.arange(THETA[0], THETA[1] + 0.1, 1.0) for y in POST["y"] for z in (PALM_T, post_top))
    spring_N = 2.0; servo_park = spring_N * zc_mid / 1000
    cam_Nm = spring_N * math.tan(math.radians(35)) * (COLLAR["r_min"] + COLLAR["lobe"] / 2) / 1000
    parking = dict(collar=COLLAR, post_top_above_tip_plate_mm=round(post_top, 1), engages_from_deg=engage,
                   post_axial_max_mm=round(post_a_max, 1), blade_root_axial_mm=round(blade_root_a, 1),
                   post_clear_of_blades_mm=round(blade_root_a - post_a_max, 1),
                   spring_N=spring_N, servo_load_Nm=round(servo_park, 3), servo_load_with_pod_weight_of_stall=round((servo_park + m_grav) / SV["stall_Nm"], 2),
                   cam_turning_torque_Nm=round(cam_Nm, 4),
                   note="valleys clocked 90 deg from the blades so the post seats with the blades along x; cam slope 35 deg [A]; "
                        "the stopped motor's cogging plus bearing drag must stay under the cam torque [A: measure]")
    hw_g = 1.0 + 3.0 + 1.0 + 1.0 + 2.0     # 683 bearing, screws, inserts, post spring leaf, aluminium collar and longer adapter [A]
    pads_g = 2.0               # TPU pads [A]
    claw_g = g(pa) + pod_g + SV["g"] + hw_g + pads_g
    res = dict(servo=SV, motor=MOTOR, prop=dict(PROP, R=round(PROP["R"], 1)), hinge_height_mm=ZP,
               travel_deg=list(THETA), empty_close_deg=close, interference_over_travel=dict(steps=len(sweep), bad=bad[:5]),
               opening_mm={str(t): gap(t) for t in (0, 15, 30, 45, 60, 75, 88)},
               prop_plane_above_tip_plate_mm=round(z_prop, 1), prop_to_finger_top_mm=round(prop_to_finger, 1),
               folded_disc_plane_from_arm_axis_mm=round(fold_plane_y, 1), folded_disc_clear_of_arm_mm=round(fold_plane_y - R_ARM, 1),
               grip=dict(pinch_radius_mm=round(r_pinch * 1000, 1), F_half_stall_N=round(F_half, 1), F_stall_N=round(F_stall, 1),
                         holds_kg_half_stall=round(hold(F_half), 2), holds_kg_stall=round(hold(F_stall), 2)),
               pod_weight_moment_Nm=round(m_grav, 3), pod_weight_of_stall=round(m_grav / SV["stall_Nm"], 2),
               gyro_Nm=gyro, gyro_of_stall_at_0p5=round(I * w * 0.5 / SV["stall_Nm"], 2),
               mass_g=dict(palm=round(g(pa), 1), pod=round(pod_g, 1), servo=SV["g"], hardware=hw_g, pads=pads_g, claw_total=round(claw_g, 1),
                           with_motor_prop=round(claw_g + MOTOR["g"] + PROP["g"], 1)),
               parking=parking,
               note="thrust passes through the hinge at every angle; servo carries the pod's weight, the squeeze and any gyroscopic moment")
    for nm, m in (("palm_finger_x8", pa), ("fan_pod_x8", pod0)):
        to_trimesh(m).export(os.path.join(out, "stl", nm + ".stl"))
    for th in (0, 45, 88):
        parts = [pa, sv, rot_x(pod0, th), rot_x(mot0, th), rot_x(prop_disc_local(0.6)[0], th), arm]
        trimesh.util.concatenate([to_trimesh(p) for p in parts]).export(os.path.join(out, f"assembly_pod_{th}deg.stl"))
    json.dump(res, open(os.path.join(out, "fan_pod.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))


def finger_only():
    return box(-W_PAD, W_PAD, FING_Y[0], FING_Y[1], PALM_T - 0.01, FING_TOP)


if __name__ == "__main__":
    main()
