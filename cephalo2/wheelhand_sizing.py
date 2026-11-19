"""Wheel hand for Cephalo-2: sizing from the robot's loads (2026-11-18).  Prints every number the CAD and doc use.
Four wheel hands carry the robot (the other four arms keep hand3); the wheel's axle is the arm-tip axis.
python3 wheelhand_sizing.py -> wheelhand_sizing.json"""
import json, numpy as np

G = 9.81
M_ROBOT = 14.81                     # Cephalo-2 with 8 x hand3 (cephalo2-baseline.md); wheel hands replace 4 of them (mass delta added below)
N_WHEEL = 4
R_W = 0.060                         # wheel radius (m): Ø120 TPU tyre; keeps the Ø55 gearbox and the Ø53 collar >= 32 mm off the floor
CRR = 0.03                          # rolling resistance, TPU tyre on hard floor (est.)
MU = 0.8                            # TPU on concrete / wood (est.)
GRADE = np.radians(15.0)            # design slope
ACC = 1.0                           # m/s^2 design acceleration
V_MAX = 1.5                         # m/s
# motor: T-Motor F60 Pro V 1750 kV (26.8 x 31.7 mm, 34.3 g, 12N14P, 4 mm shaft, 40.5 A peak 10 s)
KV = 1750.0; M_MOT = 0.0343; I_PEAK = 40.5; R_PH = 0.065; I0 = 1.0
KT = 60 / (2 * np.pi * KV)          # N m / A
V_PACK = 22.2
# gearbox: two planetary stages (sun / planet / ring teeth, module, face width), 3 planets each
STAGES = [dict(zs=15, zp=30, zr=75, m=0.5, b=5.0), dict(zs=12, zp=24, zr=60, m=0.8, b=8.0)]
ETA_STAGE = 0.92                    # per stage (est., printed / POM planets, greased)
KLOAD = 1.25                        # planet load-sharing factor, 3 planets (est.)


def ratio(s): return 1 + s["zr"] / s["zs"]


def lewis_Y(z):                     # Lewis form factor, 20 deg full depth (table fit)
    return 0.484 - 2.87 / z


def main():
    out = {}
    W = M_ROBOT * G
    Nw = W / N_WHEEL
    i_tot = np.prod([ratio(s) for s in STAGES]); eta = ETA_STAGE ** len(STAGES)
    for s in STAGES:
        assert s["zr"] == s["zs"] + 2 * s["zp"], s
        assert (s["zs"] + s["zr"]) % 3 == 0, s           # 3 equally spaced planets assemble
    F_des = M_ROBOT * (G * (CRR * np.cos(GRADE) + np.sin(GRADE)) + ACC) / N_WHEEL
    F_trac = MU * Nw
    T_des, T_trac = F_des * R_W, F_trac * R_W
    w_wheel = V_MAX / R_W; rpm_wheel = w_wheel * 60 / (2 * np.pi)
    rpm_noload = KV * V_PACK
    out["loads"] = dict(weight_N=W, per_wheel_N=Nw, drive_force_design_N=F_des, traction_limit_N=F_trac,
                        wheel_torque_design_Nm=T_des, wheel_torque_traction_Nm=T_trac, wheel_rpm_at_vmax=rpm_wheel)
    Tm_des = T_des / i_tot / eta; Tm_trac = T_trac / i_tot / eta
    I_des = Tm_des / KT + I0; I_trac = Tm_trac / KT + I0
    rpm_m_vmax = rpm_wheel * i_tot
    v_noload = rpm_noload / i_tot * 2 * np.pi / 60 * R_W
    rpm_min_sensorless = 2000.0                          # smooth sensorless running from about here (AM32 sine start below, est.)
    out["motor"] = dict(ratio=i_tot, eta=eta, torque_design_mNm=Tm_des * 1e3, torque_traction_mNm=Tm_trac * 1e3,
                        current_design_A=I_des, current_traction_A=I_trac, current_peak_rating_A=I_PEAK,
                        rpm_at_vmax=rpm_m_vmax, duty_at_vmax=rpm_m_vmax / rpm_noload, no_load_ground_speed_ms=v_noload,
                        min_smooth_speed_ms=rpm_min_sensorless / i_tot * 2 * np.pi / 60 * R_W,
                        P_elec_design_W=Tm_des * rpm_m_vmax * 2 * np.pi / 60 + I_des ** 2 * R_PH * 1.5,
                        stall_wheel_torque_at_40A_Nm=(I_PEAK - I0) * KT * i_tot * eta,
                        copper_loss_traction_W=I_trac ** 2 * R_PH * 1.5)
    # gear stresses at the traction-limit torque (the most the tyre can pass to the ground)
    gs = []
    T_in = Tm_trac
    for k, s in enumerate(STAGES):
        r_s = s["m"] * s["zs"] / 2 * 1e-3
        Ft = T_in / r_s / 3 * KLOAD                          # per sun-planet mesh
        sig_sun = Ft / (s["b"] * s["m"] * lewis_Y(s["zs"]))  # MPa (N / mm^2)
        sig_pl = Ft / (s["b"] * s["m"] * lewis_Y(s["zp"]))
        gs.append(dict(stage=k + 1, ratio=ratio(s), module=s["m"], face=s["b"], sun_pd=s["m"] * s["zs"], ring_pd=s["m"] * s["zr"],
                       input_torque_Nm=T_in, mesh_force_N=Ft, sun_root_MPa=sig_sun, planet_root_MPa=sig_pl,
                       planet_rpm_rel_at_vmax=rpm_m_vmax / np.prod([ratio(x) for x in STAGES[:k]]) * s["zs"] / s["zp"]))
        T_in = T_in * ratio(s) * ETA_STAGE
    out["gears"] = gs
    # bending at the connector: wheel load on a lever = the hand's length to the tyre centre
    L_lever = json.load(open("wheelhand.json"))["lever: socket mouth to tyre centre plane (mm)"] / 1e3   # from the CAD (wheelhand.py)
    for name, f in (("static", 1.0), ("rolling over a 20 mm step (2 g, est.)", 2.0), ("3 g landing on the wheels (est.)", 3.0)):
        out.setdefault("connector", {})[name] = dict(radial_N=Nw * f, bending_Nm=Nw * f * L_lever,
                                                     twist_Nm=T_trac if f > 1 else T_des)
    json.dump(out, open("wheelhand_sizing.json", "w"), indent=1, default=float)
    print(json.dumps(out, indent=1, default=lambda x: round(float(x), 3)))


if __name__ == "__main__":
    main()
