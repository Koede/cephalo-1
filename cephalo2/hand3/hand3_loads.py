"""Three-phalanx hand-prop (design study, 2026-11-13): loads and sizing numbers behind hand3-design-study.md.
No CAD here. python3 hand3_loads.py -> hand3_loads.json.  All section / mass / aero figures are estimates."""
import json, numpy as np
R_TIP, R_LAG = 0.127, 0.020                    # 10 in rotor, lag pin radius (hand-prop.md)
KN = [0.030, 0.066, 0.098]                     # knuckle radii K1 (base), K2 (middle), K3 (tip)
NB = 4
T_MAX, CT, RHO = 26.5, 0.14, 1.2               # thrust ceiling wanted (as the XL claw), CT of a thick untwisted 4-blade (est.)
D = 2 * R_TIP
n_max = np.sqrt(T_MAX / (CT * RHO * D ** 4)); rpm_max = 60 * n_max; rpm_proof = 1.15 * rpm_max
w = 2 * np.pi * rpm_proof / 60
def chord(r): return np.interp(r, [R_LAG, R_TIP], [0.018, 0.012])
def thick(r): return np.interp(r, [R_LAG, R_TIP], [0.0045, 0.0032])
rr = np.linspace(R_LAG, R_TIP, 2000); dr = rr[1] - rr[0]
lam = 1250 * 0.75 * chord(rr) * thick(rr) + 1200 * 0.6 * chord(rr) * 0.0015      # PA-CF section + 1.5 mm TPU pad, kg/m
def outboard(r0, extra=0.0004):                # + pins, bushings, cords
    m = rr >= r0; M = (lam[m]).sum() * dr + extra; rc = (lam[m] * rr[m]).sum() * dr / ((lam[m]).sum() * dr)
    return M, rc
tb = chord(rr) * rr ** 2; tb = tb / (tb.sum() * dr) * (T_MAX / NB)            # thrust per blade, dT/dr ~ c r^2
out = dict(rpm_at_26N=round(rpm_max), rpm_proof=round(rpm_proof), tip_speed_proof=round(w * R_TIP), knuckles=[])
for name, rk in zip(["lag pin", "K1 base", "K2 middle", "K3 tip"], [R_LAG] + KN):
    M, rc = outboard(rk); F = M * w ** 2 * rc
    m = rr >= rk; Ma = (tb[m] * (rr[m] - rk)).sum() * dr                       # thrust moment about the knuckle at T_MAX
    Fmax = M * (w / 1.15) ** 2 * rc
    e_need = 2.0 * Ma / Fmax                                                   # CG offset behind the pin line for a lock ratio of 2
    out["knuckles"].append(dict(name=name, r_mm=rk * 1e3, m_out_g=round(M * 1e3, 2), rc_mm=round(rc * 1e3, 1), CF_proof_N=round(F),
                                thrust_moment_Nmm=round(Ma * 1e3, 1), offset_for_lock2_mm=round(e_need * 1e3, 2),
                                precone_deg=round(float(np.degrees(e_need / (rc - rk))), 1),
                                bearing_MPa_2mm_stubs=round(F / (2 * 2.0 * 4.0), 1), shear_MPa_2mm_stubs=round(F / (2 * np.pi * 1.0 ** 2), 1),
                                bearing_MPa_1mm_wire=round(F / (2 * 1.0 * 4.0), 1)))
# tendon lever arms: torque, fingertip force, step per 2 mm tooth, travel for 80 deg
TEN = 80.0
out["lever"] = [dict(r_mm=r, torque_Nm=round(TEN * r * 1e-3, 2), tip_force_N_at_15mm=round(TEN * r / 15, 1), base_force_N_at_70mm=round(TEN * r / 70, 1),
                     deg_per_2mm_tooth=round(float(np.degrees(2.0 / r)), 1), travel_80deg_mm=round(r * np.radians(80), 1)) for r in (2.0, 4.5, 8.0)]
# series elastic candidates: stiffness N/mm and stretch at 40 N
def k_rod(E_MPa, A_mm2, L_mm): return E_MPa * A_mm2 / L_mm
spring = 79000 * 0.6 ** 4 / (8 * 3.4 ** 3 * 10)
out["elastic"] = dict(dyneema_0p6_150mm=round(k_rod(100000 * 0.5, 0.28, 150), 0), silicone_3mm_15mm=round(k_rod(1.5, 7.07, 15), 2),
                      tpu95_2x2_16mm=round(k_rod(30, 4.0, 16), 1), tpu95_2x3_12mm=round(k_rod(30, 6.0, 12), 1),
                      steel_spring_4od_0p6wire_10coils=round(spring, 2), steel_spring_stress_at_40N_MPa=round(8 * 40 * 3.4 / (np.pi * 0.6 ** 3)))
# hover power, ideal and with a figure of merit of 0.5 (thick untwisted finger blades, est.)
A = np.pi * R_TIP ** 2
out["power"] = {f"{T}N": dict(ideal_W=round(T ** 1.5 / np.sqrt(2 * RHO * A)), fm050_W=round(T ** 1.5 / np.sqrt(2 * RHO * A) / 0.5)) for T in (13.0, 26.5)}
out["torque_at_26N_Nm"] = round(out["power"]["26.5N"]["fm050_W"] / (2 * np.pi * rpm_max / 60), 2)
json.dump(out, open("hand3_loads.json", "w"), indent=1); print(json.dumps(out, indent=1))
