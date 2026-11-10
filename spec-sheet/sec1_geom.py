"""Section 1-2: mass budget, CG, inertia; reach, workspace, curl, shape fits, resolution. -> results/geom.json"""
import sys, os, json, math, types
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/mnt/project-files/octopus-drone/joint-study/sim")
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
import numpy as np
from scipy.optimize import least_squares
import spec_lib as S
import metrics as MET

out = {}
# ---------- mass budget ----------
rows = []
for k in range(S.N):
    rows.append(dict(node=k, between=f"joint {k} to {k+1}" if k < 12 else "joint 12 to tip plate", g=round(S.MASS[k] * 1000, 1),
                     cg_mm=round(S.CGD[k] * 1000, 1), servo_carried=(S.servo_name(k + 1) if k < 12 else "none")))
arm_moving = S.MASS.sum() * 1000; root_fixed = sum(S.ROOT_FIXED_G.values())
robot_kg = 18.95   # DESIGN.md: folding pod, 2 x DOGCOM, central 17 in prop
out["mass"] = dict(nodes=rows, arm_links_g=round(arm_moving, 1), tip_pod_g=S.TIP_G, root_fixed_g=round(root_fixed, 1),
                   root_fixed_detail=S.ROOT_FIXED_G, gear_on_joint0_g=S.GEAR_G, arm_total_g=round(arm_moving + S.TIP_G + root_fixed, 1),
                   eight_arms_kg=round(8 * (arm_moving + S.TIP_G + root_fixed) / 1000, 2), robot_kg=robot_kg,
                   arm_share_of_robot=round(8 * (arm_moving + S.TIP_G + root_fixed) / 1000 / robot_kg, 3),
                   servos_per_arm=dict(STS3250=5, STS3215=8, STS3032=1), servo_g_per_arm=5 * 74.5 + 8 * 55 + 20,
                   wiring_g_per_arm=S.WIRE_G_PER_ARM)
A = S.Arm("alt90"); q0 = np.zeros(S.N); f = A.fk(q0)
mt = S.MASS.sum() + A.m_tip
tcg, _ = A.tip_points(f)
cg = (np.sum(S.MASS[:, None] * f["cg"], 0) + A.m_tip * tcg) / mt
cg_root = float(np.linalg.norm(cg - A.base))
I_root = sum(S.MASS[k] * np.linalg.norm(f["cg"][k] - A.base) ** 2 + S.node_inertia(k)[0, 0] for k in range(S.N)) + A.m_tip * np.linalg.norm(tcg - A.base) ** 2
M, _ = A.mass_matrix(q0)
out["mass"].update(cg_from_root_straight_mm=round(cg_root * 1000, 1), moving_kg=round(mt, 3),
                   inertia_about_root_pitch_kgm2=round(float(I_root), 4),
                   joint_space_diag_kgm2=[round(float(x), 4) for x in np.diag(M)],
                   static_moment_straight_Nm=round(float(mt * S.G * cg_root), 2),
                   mass_per_m_g=round(arm_moving / 1.0, 1))

# ---------- reach and workspace ----------
rng = np.random.default_rng(3); lim = math.radians(45)
def mc(arr, n=40000):
    Aa = S.Arm(arr); P = []
    for _ in range(n):
        q = rng.uniform(-lim, lim, S.N); q[0] = rng.uniform(-math.radians(45), math.radians(45))
        P.append(Aa.fk(q)["tip"])
    P = np.array(P) - Aa.base
    vox = 0.025; keys = {tuple(x) for x in np.floor(P / vox).astype(int)}
    r = np.linalg.norm(P, axis=1)
    return dict(samples=n, voxels_25mm=len(keys), volume_m3=round(len(keys) * vox ** 3, 3), reach_max_m=round(float(r.max()), 3),
                reach_p50_m=round(float(np.median(r)), 3), reach_min_m=round(float(r.min()), 3),
                z_range_m=[round(float(P[:, 2].min()), 3), round(float(P[:, 2].max()), 3)],
                y_range_m=[round(float(P[:, 1].min()), 3), round(float(P[:, 1].max()), 3)])
out["workspace"] = {arr: mc(arr) for arr in ("alt90", "helix60")}
out["workspace"]["note"] = "tip plate positions from 40,000 random poses within the +-45 deg command range; volume = occupied 25 mm voxels"

# planar curl: pitch joints all at 45 deg, yaw joints straight
q = np.zeros(S.N); q[0::2] = math.radians(45); f = A.fk(q)
P = f["marks"] - A.base
pitch_span = 2 * S.PITCH
out["curl"] = dict(pitch_joints=7, yaw_joints=6, bend_per_pitch_joint_deg=45, total_pitch_bend_deg=7 * 45, total_yaw_bend_deg=6 * 45,
                   bend_radius_mm=round(pitch_span / math.radians(45) * 1000, 1),
                   note="pitch joints sit two pitches (153.8 mm) apart, so a uniform 45 deg curl has a 196 mm radius; the helix spreads every joint over three directions",
                   tip_after_full_pitch_curl_mm=[round(float(x) * 1000, 1) for x in P[-1]],
                   chord_root_to_tip_mm=round(float(np.linalg.norm(P[-1])) * 1000, 1),
                   self_clear_mm=round(S.self_clear(f) * 1000, 1))
# ---------- shape fits ----------
def fit_shapes(arr):
    Aa = S.Arm(arr)
    ch = types.SimpleNamespace(P=np.full(S.N, S.PITCH), base_R=S.BASE_R, base_pos=Aa.base)
    T = MET.target_shapes(ch); res = {}
    for nm, tgt in T.items():
        best = None
        for seed in range(4):
            x0 = np.random.default_rng(seed).uniform(-0.3, 0.3, S.N) if seed else np.zeros(S.N)
            fun = lambda q: np.concatenate([(Aa.fk(q)["marks"][1:] - tgt[1:]).ravel(), 1e-3 * q])
            r = least_squares(fun, x0, bounds=(-lim, lim), max_nfev=400)
            m = Aa.fk(r.x)["marks"]; e = float(np.sqrt(np.mean(np.sum((m[1:] - tgt[1:]) ** 2, 1))))
            if best is None or e < best[0]: best = (e, r.x, float(np.linalg.norm(m[-1] - tgt[-1])), S.self_clear(Aa.fk(r.x)))
        res[nm] = dict(rms_mm=round(best[0] * 1000, 1), tip_miss_mm=round(best[2] * 1000, 1), self_clear_mm=round(best[3] * 1000, 1),
                       q_deg=[round(math.degrees(x), 1) for x in best[1]])
    res["mean_rms_mm"] = round(float(np.mean([v["rms_mm"] for v in res.values()])), 1)
    return res
out["shapes"] = {arr: fit_shapes(arr) for arr in ("alt90", "helix60")}
out["shapes"]["note"] = "seven target shapes from the joint study (arc curl, spiral curl, s-curve, helix coil, side arc, twisted bend, tip hook); RMS over the 13 joint centres, least squares within +-45 deg"
# ---------- resolution ----------
enc = 2 * math.pi / 4096
Jt, Jc, Jw, f = A.jac_points(q0)
res_tip = np.abs(Jt).sum(1) * enc * 0.5   # worst case, every joint half a count off
out["resolution"] = dict(encoder_counts_per_rev=4096, step_deg=round(math.degrees(enc), 4), joint0_step_deg=round(math.degrees(enc) / 2, 4),
                         tip_step_from_joint0_straight_mm=round(enc / 2 * 1.0 * 1000, 3),
                         tip_worst_quantisation_straight_mm=[round(float(x) * 1000, 2) for x in res_tip],
                         tip_rss_quantisation_straight_mm=round(float(np.sqrt(np.sum((Jt * enc / np.sqrt(12)) ** 2))) * 1000, 2))
# manipulability straight and curled
def manip(q):
    Jt, *_ = A.jac_points(q); s = np.linalg.svd(Jt, compute_uv=False); return [round(float(x), 3) for x in s]
out["manipulability"] = dict(straight_singular_values_m_per_rad=manip(q0), curl40_singular_values=manip(np.full(S.N, math.radians(20))),
                             note="tip velocity per joint-space unit; the straight arm has no axial (radial) component, as for any chain of transverse hinges")
json.dump(out, open(os.path.join(S.HERE, "results", "geom.json"), "w"), indent=1)
print(json.dumps({k: (v if k != "shapes" else {a: v[a]["mean_rms_mm"] for a in ("alt90", "helix60")}) for k, v in out.items() if k != "mass"}, indent=0)[:3000])
print({k: v for k, v in out["mass"].items() if k != "nodes"})
