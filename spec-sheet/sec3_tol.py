"""Section 4: tolerances, fits, backlash and the tip-error stack-up. -> results/tol.json"""
import sys, os, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import spec_lib as S

out = {}
D = math.radians
# ---------- gear stage (m1, 20T / 40T, 30 mm centres) ----------
m, z1, z2, a_nom, alpha = 1.0, 20, 40, 30.0, D(20)
r2 = m * z2 / 2
gear = dict(module_mm=1, teeth=[z1, z2], centre_mm=a_nom, quality="ISO 1328 grade 8 to 9 (stock cut m1 aluminium gear) [A]")
rows = []
for jn in (0.03, 0.05, 0.08, 0.10):
    jt = jn / math.cos(alpha); rows.append(dict(normal_backlash_mm=jn, joint_backlash_deg=round(math.degrees(jt / r2), 3)))
gear["backlash_vs_joint_angle"] = rows
gear["centre_error_to_backlash"] = "each 0.01 mm of centre-distance error changes normal backlash by 2 x 0.01 x tan 20 deg = 0.0073 mm"
for da in (-0.10, -0.05, 0.0, 0.05, 0.10):
    gear.setdefault("centre_tolerance", []).append(dict(centre_error_mm=da, backlash_change_mm=round(2 * da * math.tan(alpha), 4)))
gear["recommend"] = ("design normal backlash 0.05 mm (0.15 deg at the joint); bore the gear and pinion bearing seats in one setup or ream "
                     "them in a drilling jig so the centre distance holds +-0.02 mm; a printed socket alone holds about +-0.1 mm, which "
                     "swings the backlash from 0 (binding) to 0.12 mm. Set the nominal centre 0.05 mm long (30.05 mm) if the seats stay as printed.")
out["gear"] = gear

# ---------- fits table ----------
CTE = dict(PETG=68e-6, Al7075=23.6e-6, steel=11.5e-6)
def loosen(d, dT, a="PETG", b="steel"): return round(d * (CTE[a] - CTE[b]) * dT, 4)
fits = [
    dict(item="61803 outer ring (26 mm) in printed socket boss", load="outer ring stationary against a mostly fixed load direction",
         fit="light press, 0.03 to 0.08 mm interference on the as-printed bore (print 25.90, ream or sand to fit)", loosens_per_40K_mm=loosen(26, 40),
         note="PETG grows 0.06 mm more than the steel ring at +40 K: a 0.03 mm press is gone by +20 K. Bond the ring with epoxy or keep a lip and a cap."),
    dict(item="61803 inner ring (17 mm) on the 7075 gear spigot", load="inner ring turns +-55 deg under a load fixed to the socket: rotating inner-ring load",
         fit="17 k5 (+0.012 / +0.001) on a solid spigot; the hollow spigot as drawn is thin, so 17 j5 (+0.005 / -0.003) plus retaining compound", loosens_per_40K_mm=loosen(17, 40, "Al7075"),
         note="aluminium grows 0.008 mm more than the ring at +40 K, which tightens this fit slightly"),
    dict(item="695 outer ring (13 mm) in the printed bearing cap", load="pinion bearing, rotating inner ring", fit="light press 0.02 to 0.05 mm, bonded", loosens_per_40K_mm=loosen(13, 40), note=""),
    dict(item="695 inner ring on the 5 mm pinion stub", load="rotating inner ring", fit="5 k5 (+0.006 / +0.001)", loosens_per_40K_mm=0.0, note=""),
    dict(item="6705 outer ring (32 mm) in the printed lid ring, joints 1 to 4", load="stationary outer ring", fit="0 to 0.05 mm interference, bonded", loosens_per_40K_mm=loosen(32, 40), note="0.07 mm of loosening at +40 K"),
    dict(item="6705 inner ring (25 mm) on the printed fork sleeve, joints 1 to 4", load="inner ring turns with the fork: rotating inner-ring load",
         fit="0.05 to 0.10 mm interference on a sanded sleeve (print 25.20, sand to fit); a printed sleeve creeps, so bond it", loosens_per_40K_mm=-loosen(25, 40),
         note="the sleeve grows into the ring when warm (tightens 0.04 mm at +40 K); the risk is creep at room temperature"),
    dict(item="8 mm 7075 idler pin in the printed cheek (joint 0) or cradle back (joints 1 to 4)", load="static press", fit="pin 8 h6 (0 / -0.009); printed hole 7.80 to 7.85 mm, pressed (0.15 to 0.2 mm into PETG) or bonded in an 8.0 mm hole",
         loosens_per_40K_mm=loosen(8, 40, "PETG", "Al7075"), note=""),
    dict(item="Fork idler ear on the 8 mm pin or stub (running)", load="oscillating pivot", fit="running clearance 0.05 to 0.10 mm diametral: print the bore at 8.0 and ream 8.05 to 8.10, or press an 8 x 10 x 4 mm PTFE or iglide bushing into a 10 mm bore",
         loosens_per_40K_mm=0.0, note="as drawn the bore is 8.4 mm (0.4 mm clearance): see the stack-up"),
    dict(item="Horn screws, 4 x M3 through the 4 mm fork horn ear", load="joint torque by clamp friction, then by the screw shanks", fit="clearance hole 3.4 mm as drawn; ream 3.1 mm (or print 3.0 and drill 3.1)",
         loosens_per_40K_mm=0.0, note="see the horn clamp check: PETG relaxes the clamp, and then the hole clearance becomes backlash"),
    dict(item="Gear and pinion centre (30 mm)", load="mesh", fit="+-0.02 mm with reamed seats; +0.05 mm nominal if printed", loosens_per_40K_mm=round(30 * (CTE["PETG"] - CTE["Al7075"]) * 40, 4),
         note="the printed socket grows 0.05 mm more than the gears at +40 K, which adds 0.04 mm of backlash when hot"),
    dict(item="Joint pitch (76.92 mm)", load="", fit="+-0.2 mm per printed vertebra (FDM, along the arm axis)", loosens_per_40K_mm=round(76.92 * CTE["PETG"] * 40, 3), note="13 in a row: +-2.6 mm worst case, +-0.7 mm RSS in arm length"),
    dict(item="PTFE thrust washer, idler ear to cradle floor", load="axial", fit="0.5 mm washer; end play 0.05 to 0.15 mm, set with 0.1 mm shims", loosens_per_40K_mm=0.0, note=""),
    dict(item="Servo in its pocket", load="", fit="0.5 mm clearance per side as drawn; the lid clamps it", loosens_per_40K_mm=0.0, note=""),
]
out["fits"] = fits

# ---------- horn clamp ----------
# M3 class 8.8 into the Feetech aluminium horn, tightened to 0.5 N.m -> preload about F = T / (0.2 d) [A]
T_tight = 0.5; Fp = T_tight / (0.2 * 0.003); mu = 0.3; r_pcd = 0.007
cap_new = 4 * Fp * mu * r_pcd
relax = 0.5   # [A] PETG under a screw head loses about half its clamp within days (creep); unbonded, no washer
out["horn_clamp"] = dict(preload_per_screw_N=round(Fp), friction=mu, torque_by_friction_new_Nm=round(cap_new, 2),
                         torque_by_friction_relaxed_Nm=round(cap_new * relax, 2), STS3250_stall_Nm=4.9, STS3215_stall_Nm=2.94,
                         slip_backlash_deg_as_drawn=round(math.degrees(0.2e-3 / r_pcd), 2), slip_backlash_deg_reamed_3p1=round(math.degrees(0.05e-3 / r_pcd), 2),
                         note="new, the clamp holds more than stall on every joint; once the PETG relaxes it holds less than an STS3250's stall, and then the 0.4 mm hole clearance adds up to 1.6 deg of play at that joint. Fix: ream the holes to 3.1 mm, add metal washers under the heads, or bond the ear to the horn.")

# ---------- stack-up ----------
SPAN = 0.049   # m between the fork ears
cases = {
    "as designed (reamed bores, horn clamp holding, gear 0.05 mm)": dict(backlash=0.5, gear=0.15, idler_c=0.08e-3, horn=0.0),
    "as drawn (8.4 mm bores), horn clamp holding": dict(backlash=0.5, gear=0.15, idler_c=0.40e-3, horn=0.0),
    "as drawn, horn clamp relaxed (3.4 mm holes slip)": dict(backlash=0.5, gear=0.15, idler_c=0.40e-3, horn=0.40e-3),
}
A = S.Arm("alt90")
def stack(q, c, n=20000, rng=np.random.default_rng(5)):
    f = A.fk(q); tip = f["tip"]
    cols_h, cols_t = [], []   # tip displacement per unit rotation about the hinge and about the two tilt axes
    amp_h, amp_t = [], []
    for j in range(S.N):
        Rf = A.joint_frame(f, j)
        lever = tip - f["o"][j]
        cols_h.append(np.cross(Rf[0], lever))
        bl = D(c["backlash"]) / (2 if j == 0 else 1) + (D(c["gear"]) if j == 0 else 0)
        bl += 2 * math.atan(c["horn"] / 2 / 0.007)       # hole slip (total play) at the horn ear
        amp_h.append(bl / 2)
        tilt = c["idler_c"] / 2 / SPAN                    # radial clearance over the ear span, about beta and zeta
        cols_t.append((np.cross(Rf[1], lever), np.cross(Rf[2], lever))); amp_t.append(tilt)
    H = np.array(cols_h); ah = np.array(amp_h)
    worst = np.sum(np.linalg.norm(H, axis=1) * ah) + sum(amp_t[j] * (np.linalg.norm(cols_t[j][0]) + np.linalg.norm(cols_t[j][1])) for j in range(S.N))
    th = rng.uniform(-1, 1, (n, S.N)) * ah
    tb = rng.uniform(-1, 1, (n, S.N)) * np.array(amp_t); tz = rng.uniform(-1, 1, (n, S.N)) * np.array(amp_t)
    d = th @ H + tb @ np.array([c_[0] for c_ in cols_t]) + tz @ np.array([c_[1] for c_ in cols_t])
    e = np.linalg.norm(d, axis=1)
    return dict(worst_case_mm=round(worst * 1000, 1), p95_mm=round(float(np.percentile(e, 95)) * 1000, 1), rms_mm=round(float(np.sqrt(np.mean(e ** 2))) * 1000, 1),
                share_from_backlash=round(float(np.sum(np.linalg.norm(H, axis=1) * ah) / worst), 2))
poses = {"straight out": np.zeros(S.N)}
try:
    P = np.load(os.path.join(S.HERE, "results", "poses_alt90.npz"))
    if "alt90_flight_hover" in P: poses["hover pose"] = P["alt90_flight_hover"]
except Exception: pass
out["stackup"] = {cn: {pn: stack(q, c) for pn, q in poses.items()} for cn, c in cases.items()}
out["stackup_note"] = ("play only: servo backlash 0.5 deg [A, no maker figure], halved by the 2:1 stage plus the stage's own; idler bore clearance tilts the "
                       "fork about the two cross axes over the 49 mm ear span; horn-hole slip once the clamp has relaxed. Uniform within each play, "
                       "20,000 samples. Gravity or thrust usually preloads every joint to one side of its play, so the repeatable error under a steady load is "
                       "far smaller than these numbers; they are what the tip can wander when the load reverses.")
# encoder and deadband
enc = 2 * math.pi / 4096
out["servo_resolution"] = dict(counts=4096, step_deg=round(math.degrees(enc), 4), deadband_counts_default="1 to 2 [A]",
                               tip_shift_from_one_count_at_joint1_mm=round(enc * 0.923 * 1000, 2))
# pitch tolerance
out["length_tolerance"] = dict(per_joint_mm=0.2, worst_mm=round(0.2 * 13, 1), rss_mm=round(0.2 * math.sqrt(13), 2),
                               thermal_mm_per_10K=round(1000 * 60e-6 * 10, 2), note="PETG and servo cases taken together at 60e-6 per K [A]")
json.dump(out, open(os.path.join(S.HERE, "results", "tol.json"), "w"), indent=1)
print(json.dumps(out["stackup"], indent=1)); print(out["horn_clamp"])
