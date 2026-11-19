"""hand3 hub C ("two rings", 2026-11-14): the finger drive that replaces the rev D servo drum, claw bars and followers.
Single source of truth for hub C: parameters, sizing, solids, checks, masses.  python3 hand3_hubc.py -> hand3_hubc.json, STLs.
Rotor, fingers, hub plates, heel, motor: unchanged, from hand3_design.py / hand3_cad.py (rev D).  Units mm, N, g.  Estimates.

How it works.  Two rings slide on the cage under the hub and turn with the rotor.
  BASE ring (outer): four cords, one per finger.  Each cord passes its finger's lag pin at an offset (E_LAG) and then crosses K1
      (lever 6 mm): the first pull swings the fingers into two pairs (a weak lag spring gives way first), the rest folds K1.
  CURL ring (inner): four cords, each over K2 (4.0 mm) and K3 (3.0 mm); the extensor at K3 is pre-tensioned so the tip wraps last.
Every cord ends in its own TPU link with a screw anchor (length trim).  Each ring has a flange.  Two stationary yokes, each with a
pair of lips 1 mm above its ring's flange in flight, pull the rings down; each yoke rides on two Ø3 guide rods and is pulled by
one STS3215 (the arm servo, 12 V) through two cords on a double-ended Ø8 spool.  The rings are round, so the yokes meet them at any
rotor angle: no index for the drive (the rotor still parks to about +-5 deg so the grip axis points the right way).
The two servos lie stacked under the motor, shafts across the hand axis."""
import os, sys, json
import numpy as np
import hand3_design as H
import hand3_cad as C
from hand3_cad import cyl, box, app, T, rot, to_trimesh

Z = H.Z
ETA = H.ETA                                  # spool + cord x guide, as the rev D sizing (0.52, est.)
SERVO = dict(name="Feetech STS3215", stall=2.94, rated=0.981, m=55.0, size=(45.2, 24.7, 35.0), volts="12 V", stall_A=2.7, deg=360.0)   # project arm servo
SPOOL_R = 4.0
LEVER = [6.0, 4.0, 3.0]                      # K1, K2, K3 cord levers (K2 / K3 were 4.5 / 3.5 in rev D: shorter stroke for the ring stack)
E_LAG = 2.5                                  # cord offset from the lag pin: spread torque = tension x 2.5 mm
STRETCH = 2.0                                # TPU link stretch at peak force (was 3.0): stroke again
LAG_SPRING, EXT_K1, EXT_K2, EXT_K3 = 0.006, 0.02, 0.004, 0.02     # N m pretensions: pairs first, then K1; K2 before K3
TRAVEL_B = E_LAG * np.radians(H.SPREAD_MAX) + LEVER[0] * np.radians(H.FOLD_MAX[0]) + STRETCH
TRAVEL_C = LEVER[1] * np.radians(H.FOLD_MAX[1]) + LEVER[2] * np.radians(H.FOLD_MAX[2]) + STRETCH
GAP = 1.0                                    # flight gap lip to flange, and ring to ring
# radii
R_CAGE = (19.5, 21.5); R_IN = (21.9, 23.9); R_OUT = (24.3, 26.3); R_FL_OUT = 30.0; R_FL_IN = 32.5
R_LIP_OUT = (27.1, 30.0); R_LIP_IN = (30.5, 32.5); R_LEG = (33.5, 36.0); R_ROD = 38.0
# axial stack at rest (flight)
Z_OUT = (80.0, 83.0); Z_FL_OUT = (80.0, 82.0); Z_LIP_OUT = (Z_FL_OUT[1] + GAP, Z_FL_OUT[1] + GAP + 2.0)
Z_FL_IN = (Z_OUT[0] - TRAVEL_B - GAP - 2.0, Z_OUT[0] - TRAVEL_B - GAP); Z_IN = (Z_FL_IN[0], 83.0)
Z_LIP_IN = (Z_FL_IN[1] + GAP, Z_FL_IN[1] + GAP + 2.0)
Z_SERVO_TOP = 2 * SERVO["size"][1]            # two servos stacked on their 24.7 mm side
AZ_OUT, AZ_IN = (90.0, 270.0), (0.0, 180.0)   # where the two yokes' lips and legs stand
LIP_HALF = 38.0                               # deg: each lip is an arc of +-38 deg


def sizing():
    out = {}
    for nm, trav, levers in (("base", TRAVEL_B, [LEVER[0]]), ("curl", TRAVEL_C, LEVER[1:])):
        Fs, Fr = SERVO["stall"] * ETA / (SPOOL_R * 1e-3), SERVO["rated"] * ETA / (SPOOL_R * 1e-3)
        need = H.joint_torques(H.F_TIP_PEAK)
        Fp = need[0] / (LEVER[0] * 1e-3) if nm == "base" else max(need[1] / (LEVER[1] * 1e-3), need[2] / (LEVER[2] * 1e-3))
        out[nm] = dict(travel=round(float(trav), 1), spool_deg=round(float(np.degrees(trav / SPOOL_R))), F_stall_ring=round(Fs), F_rated_ring=round(Fr),
                       F_stall_finger=round(Fs / 4, 1), F_rated_finger=round(Fr / 4, 1), F_peak_need=round(float(Fp), 1), margin=round(Fs / 4 / float(Fp), 2),
                       tpu_k=round(float(Fp) / STRETCH, 1), t_full_s=round(float(np.degrees(trav / SPOOL_R)) / 60 * 0.222, 2))
    return out


def arc(r0, r1, z0, z1, az, half):
    m = cyl(r1, z0, z1, n=96) - cyl(r0, z0 - 1, z1 + 1, n=96)
    h = np.radians(half)
    m = C.half(C.half(m, np.array([np.sin(h), np.cos(h), 0.0]), [0, 0, 0]), np.array([np.sin(h), -np.cos(h), 0.0]), [0, 0, 0])
    return app(m, T(rot("z", az)))


def cage():
    m = cyl(R_CAGE[1], 55.5, Z["hub_lo"], n=96) - cyl(R_CAGE[0], 50, 90, n=96)
    for a in C.FAZ:                                           # two slots per finger: ring lugs and cords pass through
        for d in (-8.0, 8.0): m -= app(box([R_CAGE[0] - 1, -1.3, 57.0], [R_CAGE[1] + 1, 1.3, Z["hub_lo"] - 2.0]), T(rot("z", a + d)))
    return m


def ring_in():
    m = cyl(R_IN[1], *Z_IN, n=96) - cyl(R_IN[0], Z_IN[0] - 1, Z_IN[1] + 1, n=96)
    m += cyl(R_FL_IN, *Z_FL_IN, n=96) - cyl(R_IN[0], Z_FL_IN[0] - 1, Z_FL_IN[1] + 1, n=96)
    for a in C.FAZ: m += app(box([R_CAGE[0] + 0.3, -1.0, Z_IN[1] - 4.0], [R_IN[0] + 0.2, 1.0, Z_IN[1]]), T(rot("z", a + 8.0)))      # lug: key + cord anchor
    return m


def ring_out():
    m = cyl(R_OUT[1], *Z_OUT, n=96) - cyl(R_OUT[0], Z_OUT[0] - 1, Z_OUT[1] + 1, n=96)
    m += cyl(R_FL_OUT, *Z_FL_OUT, n=96) - cyl(R_OUT[0], Z_FL_OUT[0] - 1, Z_FL_OUT[1] + 1, n=96)
    return m                                                  # keyed to the inner ring's sleeve by a flat (not modelled); cords anchor on its top face


def yoke(which):
    """Two lips over the flange + two legs down to a cross-bar with the guide bushings (stationary; slides on two rods)."""
    lip_r, lip_z, azs = (R_LIP_OUT, Z_LIP_OUT, AZ_OUT) if which == "out" else (R_LIP_IN, Z_LIP_IN, AZ_IN)
    m = None
    for a in azs:
        p = arc(lip_r[0], lip_r[1] + 1.0, lip_z[0], lip_z[1], a, LIP_HALF)                 # lip over the flange
        p += arc(lip_r[1] + 0.5, R_LEG[1], lip_z[0] - 2.5, lip_z[0] + 0.5, a, 14.0)        # arm: steps down beside the flange, clear of the pitched phalanges
        leg = arc(R_LEG[0], R_LEG[1], lip_z[0] - 18.0, lip_z[0] + 0.5, a, 14.0)           # leg
        boss = app(cyl(3.2, lip_z[0] - 18.0, lip_z[0] - 8.0, c=(R_ROD, 0, 0)) - cyl(1.55, 0, 100, c=(R_ROD, 0, 0)), T(rot("z", a)))
        tab = app(box([R_LEG[0], -3.2, lip_z[0] - 18.0], [R_ROD, 3.2, lip_z[0] - 8.0]), T(rot("z", a))) - app(cyl(1.55, 0, 100, c=(R_ROD, 0, 0)), T(rot("z", a)))
        for s in (p, leg, boss, tab): m = s if m is None else m + s
    return m


def frame():
    """Servo cradle (two STS3215 stacked, shafts along y), motor column, rod sockets.  Clamps on the arm's tip tube below z = 0."""
    L, W, Hh = SERVO["size"]
    m = box([-L / 2 - 2, -Hh / 2 - 2, 0], [L / 2 + 2, Hh / 2 + 2, 2.0])                    # base plate
    m += box([-L / 2 - 2, -Hh / 2 - 2, 0], [-L / 2, Hh / 2 + 2, Z_SERVO_TOP]) + box([L / 2, -Hh / 2 - 2, 0], [L / 2 + 2, Hh / 2 + 2, Z_SERVO_TOP])   # end walls
    m += cyl(20.0, Z_SERVO_TOP, Z["pack1"], n=64) - cyl(15.0, Z_SERVO_TOP + 2.0, Z["pack1"] - 2.5, n=64)       # motor column (19 x 19 M3 pattern on top)
    for a in AZ_OUT + AZ_IN:                                                               # rod sockets on arms from the end walls / plate
        m += app(cyl(3.0, 0, 6.0, c=(R_ROD, 0, 0)) + box([15.0, -3.0, 0], [R_ROD, 3.0, 2.0]), T(rot("z", a)))
    return m


def hardware():
    L, W, Hh = SERVO["size"]; out = {}
    for n, z0 in (("servo_base", 0.0), ("servo_curl", W)):
        out[n] = box([-L / 2, -Hh / 2, z0 + 0.1], [L / 2, Hh / 2, z0 + W - 0.1])
        for sy in (-1, 1): out[n + f"_spool{sy}"] = cyl(SPOOL_R + 1.0, sy * (Hh / 2 + 0.5), sy * (Hh / 2 + 6.5), "y", (L / 2 - 10.6 if z0 == 0 else -(L / 2 - 10.6), 0, z0 + W / 2))
    for i, a in enumerate(AZ_OUT + AZ_IN): out[f"rod{i}"] = app(cyl(1.5, 0, Z_LIP_OUT[1], c=(R_ROD, 0, 0)), T(rot("z", a)))
    out["motor_base"] = cyl(17, Z["pack1"], Z["pack1"] + 8)
    return out


RHO = dict(pacf=1.25, alu=2.70, steel=7.85)
def parts():
    return dict(cage=(cage(), "pacf", True), ring_in=(ring_in(), "pacf", True), ring_out=(ring_out(), "pacf", True),
                yoke_out=(yoke("out"), "pacf", False), yoke_in=(yoke("in"), "pacf", False), frame=(frame(), "pacf", False))


def assembly(pull_b=0.0, pull_c=0.0, lag=(0, 0, 0, 0), fold=((0, 0, 0),) * 4, spin=0.0):
    """Placed solids: name -> (solid, rotating?).  pull = ring stroke (the yoke moves GAP further to meet the flange)."""
    P = parts(); S = T(rot("z", spin)); R = C.parts(); out = {}
    out["cage"] = (app(P["cage"][0], S), True)
    out["ring_out"] = (app(P["ring_out"][0], S @ T(t=[0, 0, -pull_b])), True); out["ring_in"] = (app(P["ring_in"][0], S @ T(t=[0, 0, -pull_c])), True)
    out["yoke_out"] = (app(P["yoke_out"][0], T(t=[0, 0, -(pull_b + GAP) if pull_b > 0 else 0])), False)
    out["yoke_in"] = (app(P["yoke_in"][0], T(t=[0, 0, -(pull_c + GAP) if pull_c > 0 else 0])), False)
    out["frame"] = (P["frame"][0], False)
    for n, m in hardware().items(): out["hw_" + n] = (m, False)
    out["hw_motor_bell"] = (app(cyl(17, Z["pack1"] + 8.5, Z["motor_top"]) + cyl(2.5, Z["motor_top"], Z["shaft_tip"]), S), True)
    for n in ("hub_lower", "hub_upper", "heel"): out[n] = (app(R[n], S), True)
    for k in range(4):
        Fr = C.finger_frames(k, lag[k], fold[k])
        out[f"grip{k}"] = (app(R["root_grip"], S @ Fr[0]), True); out[f"tube{k}"] = (app(R["lag_tube"], S @ T(t=[0, 0, C.ZB]) @ T(rot("z", C.FAZ[k]))), True)
        for j in range(3):
            out[f"ph{k}_{j + 1}"] = (app(R[f"phalanx{j + 1}"], S @ Fr[j + 1]), True); out[f"pad{k}_{j + 1}"] = (app(R[f"pad{j + 1}"], S @ Fr[j + 1]), True)
    return out


def overlaps(a, skip=()):
    """Overlap volume (mm3) and smallest gap between stationary and rotating parts, and among the drive parts."""
    names = list(a); bad = {}; gap = 9e9; gpair = None
    drive = ("cage", "ring_out", "ring_in", "yoke_out", "yoke_in", "frame")
    for i, n in enumerate(names):
        for mname in names[i + 1:]:
            if not ((a[n][1] != a[mname][1]) or (n in drive and mname in drive) or (n in drive and mname.startswith("hw_")) or (mname in drive and n.startswith("hw_"))): continue
            if (n, mname) in skip or (mname, n) in skip: continue
            v = (a[n][0] ^ a[mname][0]).volume()
            if v > 0.01: bad[f"{n}|{mname}"] = round(float(v), 2)
            elif a[n][1] != a[mname][1]:
                g = a[n][0].min_gap(a[mname][0], 3.0)
                if g < gap: gap, gpair = float(g), f"{n}|{mname}"
    return bad, round(gap, 2), gpair


SKIP = {("frame", "hw_servo_base"), ("frame", "hw_servo_curl"), ("frame", "hw_motor_base"), ("frame", "hw_rod0"), ("frame", "hw_rod1"), ("frame", "hw_rod2"), ("frame", "hw_rod3"),
        ("yoke_out", "hw_rod0"), ("yoke_out", "hw_rod1"), ("yoke_in", "hw_rod2"), ("yoke_in", "hw_rod3"), ("cage", "ring_in")}

if __name__ == "__main__":
    sz = sizing(); P = parts(); mass = {}
    for n, (m, mat, _) in P.items(): mass[n] = round(float(m.volume()) * RHO[mat] / 1000, 1)
    cadp = json.load(open("hand3_cad_parts.json"))
    fingers = 4 * (cadp["root_grip"]["mass_g"] + sum(cadp[f"phalanx{j}"]["mass_g"] + cadp[f"pad{j}"]["mass_g"] for j in (1, 2, 3)))
    hubp = cadp["hub_lower"]["mass_g"] + cadp["hub_upper"]["mass_g"] + 4 * cadp["lag_tube"]["mass_g"]
    tot = dict(fingers=round(fingers, 1), hub_plates_lag_tubes=round(hubp, 1), heel_nut=4.8, motor=H.MOTOR["m"], servos=2 * SERVO["m"], frame=mass["frame"],
               yokes=round(mass["yoke_out"] + mass["yoke_in"], 1), rings_cage=round(mass["ring_in"] + mass["ring_out"] + mass["cage"], 1),
               rods_spools_screws=14.0, cords_links_springs=6.0, wiring_hall=8.0, arm_collar=12.0)
    tot["total"] = round(sum(tot.values()), 1)
    stat = tot["servos"] + tot["frame"] + tot["rods_spools_screws"] + tot["arm_collar"] + tot["wiring_hall"]
    com = (stat * Z_SERVO_TOP / 2 + tot["yokes"] * 70 + tot["rings_cage"] * 72 + tot["motor"] * (Z["pack1"] + H.MOTOR["can_L"] / 2)
           + (tot["fingers"] + tot["hub_plates_lag_tubes"] + tot["heel_nut"] + tot["cords_links_springs"]) * Z["blade"]) / tot["total"]
    chk = {}
    for nm, kw in (("flight", {}), ("base_full", dict(pull_b=TRAVEL_B, lag=[s * H.SPREAD_MAX for s in H.LAG_SIGN], fold=((90, 0, 0),) * 4)),
                   ("curl_full", dict(pull_c=TRAVEL_C, fold=((0, 90, 85),) * 4)),
                   ("grasp", dict(pull_b=TRAVEL_B, pull_c=TRAVEL_C * 0.6, lag=[s * H.SPREAD_MAX for s in H.LAG_SIGN], fold=((90, 35, 50),) * 4))):
        a = assembly(**kw)
        if nm != "flight": a = {k: v for k, v in a.items() if not k.startswith(("ph", "pad", "grip"))}     # finger poses are checked in hand3_cad_checks
        bad, gap, gp = overlaps(a, SKIP); chk[nm] = dict(overlaps=bad, min_gap_rot_stat=gap, at=gp)
    spin = 9e9
    for s in np.arange(0, 90, 7.5):
        a = assembly(spin=s); bad, gap, gp = overlaps(a, SKIP)
        if bad: spin = -1; chk["spin_overlap"] = bad; break
        spin = min(spin, gap)
    chk["flight_spin_min_gap"] = spin
    env = dict(body=f"{SERVO['size'][0] + 4:.0f} x {SERVO['size'][2] + 13:.0f} mm servo block, {Z_SERVO_TOP:.1f} mm tall; rings and yokes inside Ø{2 * R_LEG[1]:.0f}; four rod tabs to Ø{2 * (R_ROD + 3.2):.0f}",
               stack_margin=round(Z_FL_IN[0] - TRAVEL_C - Z_SERVO_TOP, 2))
    out = dict(servo=SERVO, spool_r=SPOOL_R, levers=LEVER, e_lag=E_LAG, lag_spring_Nm=LAG_SPRING, ext_k1_Nm=EXT_K1, ext_k2_Nm=EXT_K2, ext_k3_Nm=EXT_K3, stretch=STRETCH,
               base=sz["base"], curl=sz["curl"], z=dict(out=Z_OUT, fl_out=Z_FL_OUT, fl_in=Z_FL_IN, lip_out=Z_LIP_OUT, lip_in=Z_LIP_IN, servo_top=Z_SERVO_TOP),
               mass=tot, com_from_spigot=round(float(com), 1), checks=chk, envelope=env, electrical=dict(servo_stall_A=2 * SERVO["stall_A"], rail="12 V arm bus"))
    json.dump(out, open("hand3_hubc.json", "w"), indent=1, default=float)
    os.makedirs("hand3_hubc_stl", exist_ok=True)
    for n, (m, _, _) in P.items(): to_trimesh(m).export(f"hand3_hubc_stl/hubc_{n}.stl")
    print("sizing", sz); print("part mass g", mass); print("mass", tot, "com", round(float(com), 1)); print("checks", chk); print("envelope", env)
