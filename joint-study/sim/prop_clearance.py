"""Claw prop clearance while walking, for the Cephalo-1 main arm (1 m, 13 joints, pin hinge 60 deg helix, 7-7.5 in claw props).

The walking stances come from the study's foot IK (ripple gait, 7 feet planted, claw vertical within 20 deg), with the
foot 45 mm up for the swing. The claw fan sits on the pan-tilt wrist at the tip: the wrist pivot is CLAW_H above the
ground (the claw's fingers reach below it), and the prop disc is centred FAN_OFFSET out from the pivot along the fan
axis u. For every fan direction u on a sphere, the disc (hub radius 12 mm to the blade tip) is checked against:
  - the ground plane,
  - this arm (57.1 mm tube along the arm centreline),
  - the two neighbouring arms in the same pose (turned +-45 deg about the hub; a stand-in for their own stance),
  - the mantle (sphere of 100 mm radius at the hub centre, the study's collision sphere [A]).
A direction counts as clear when every gap is at least MARGIN. Writes results/prop_clearance.json.

  python3 prop_clearance.py
"""
import os, sys, math, json
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
import config as C, chain as CH, metrics as M
import bearing_loads as B

ARM_R = 0.02855               # m, tube radius (57.1 mm OD)
HUB_SPHERE = 0.100            # m, mantle collision sphere [A]
MARGIN = 0.015                # m, clearance wanted round a spinning prop [A]
CLAW_HS = (0.0, 0.04, 0.08, 0.12)        # m, wrist pivot above the ground in stance [A: claw not designed yet]
FAN_OFFSETS = (0.060, 0.090, 0.120)      # m, pivot to prop disc along the fan axis (60 mm today)
PROPS = {"7 in": 0.0889, "7.5 in": 0.09525}
HUB_R = 0.012

def disc_points(c, u, R, n_r=7, n_t=48):
    a = np.cross(u, [1.0, 0, 0]);
    if np.linalg.norm(a) < 0.1: a = np.cross(u, [0, 1.0, 0])
    a /= np.linalg.norm(a); b = np.cross(u, a)
    r = np.linspace(HUB_R, R, n_r)[:, None]; t = np.linspace(0, 2 * math.pi, n_t, endpoint=False)[None, :]
    P = c + (r * np.cos(t))[..., None] * a + (r * np.sin(t))[..., None] * b
    return P.reshape(-1, 3)

def seg_dist(P, A, Bp):
    """distance from each point in P to the polyline with vertices A (n,3) -> min over segments"""
    d = np.full(len(P), np.inf)
    for p0, p1 in zip(A[:-1], A[1:]):
        v = p1 - p0; L2 = v @ v
        s = np.clip(((P - p0) @ v) / max(L2, 1e-12), 0, 1)
        d = np.minimum(d, np.linalg.norm(P - (p0 + s[:, None] * v), axis=1))
    return d

def rotz(P, ang):
    c, s = math.cos(ang), math.sin(ang); R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    return P @ R.T

def stance_poses(ch, claw_h):
    """IK for the four stance feet and the same feet lifted 45 mm; the arm tip (wrist pivot) is claw_h above the foot"""
    out = []; x = M._x0(ch)
    for i, f in enumerate(M.FEET):
        for lift, nm in ((0.0, "stance"), (0.045, "swing, foot up 45 mm")):
            tgt = f + np.array([0, 0, claw_h + lift])
            q, ok, perr, ang, pen = M.ik_foot(ch, tgt, x)
            if lift == 0.0: x = q
            out.append(dict(name=f"{nm} {i+1}", q=q, ok=bool(ok), perr_mm=perr * 1e3, claw_tilt_deg=ang, ground_z=f[2], lift=lift))
    return out

def check(ch, pose, R, offset, dirs):
    m = CH.run_fk(ch, pose["q"])[0]
    piv = m[-1]; t = m[-1] - m[-2]; t /= np.linalg.norm(t)
    # this arm, minus the last 40 mm before the pivot (the wrist itself, sized with the claw)
    keep = np.linalg.norm(m - piv, axis=1) > 0.040
    own = m[keep]
    if len(own) < len(m): own = np.vstack([own, piv - 0.040 * t])
    nbrs = [rotz(m, math.radians(a)) for a in (45.0, -45.0)]
    gz = pose["ground_z"]
    res = np.zeros((len(dirs), 4))
    for k, u in enumerate(dirs):
        P = disc_points(piv + offset * u, u, R)
        res[k, 0] = P[:, 2].min() - gz
        res[k, 1] = seg_dist(P, own, None).min() - ARM_R
        res[k, 2] = min(seg_dist(P, n, None).min() for n in nbrs) - ARM_R
        res[k, 3] = np.linalg.norm(P, axis=1).min() - HUB_SPHERE
    return res, t

def main():
    ch, masses = B.build()
    dirs = B.sphere(600)
    out = {"assumptions": dict(margin_mm=MARGIN * 1e3, hub_sphere_mm=HUB_SPHERE * 1e3, arm_od_mm=2e3 * ARM_R,
                               note="wrist pivot at the arm tip; claw not designed yet, so its height is swept")}
    rows = []
    for ch_h in CLAW_HS:
        poses = stance_poses(ch, ch_h)
        for pn, R in PROPS.items():
            for off in FAN_OFFSETS:
                clear_all = np.ones(len(dirs), bool); up_ok = []; worst = []
                per_pose = []
                for p in poses:
                    res, t = check(ch, p, R, off, dirs)
                    clr = (res >= MARGIN).all(1)
                    clear_all &= clr
                    per_pose.append(dict(pose=p["name"], ik_ok=p["ok"], claw_tilt_deg=round(p["claw_tilt_deg"], 1),
                                         frac_clear=round(float(clr.mean()), 3),
                                         best_min_gap_mm=round(float(res.min(1).max() * 1e3), 1),
                                         up30_clear=bool((clr & (dirs[:, 2] >= math.cos(math.radians(30)))).any()),
                                         straight_up_gaps_mm=[round(float(x * 1e3), 1) for x in res[np.argmax(dirs[:, 2])]]))
                upmask = dirs[:, 2] >= math.cos(math.radians(30))
                best = int(np.argmax(np.where(clear_all, dirs[:, 2], -9))) if clear_all.any() else None
                rows.append(dict(claw_h_mm=ch_h * 1e3, prop=pn, offset_mm=off * 1e3,
                                 frac_dirs_clear_in_every_pose=round(float(clear_all.mean()), 3),
                                 any_dir_within_30deg_of_up_clear_in_every_pose=bool((clear_all & upmask).any()),
                                 highest_clear_dir=None if best is None else [round(float(x), 3) for x in dirs[best]],
                                 highest_clear_dir_tilt_from_up_deg=None if best is None else round(math.degrees(math.acos(dirs[best][2])), 1),
                                 poses=per_pose))
                r = rows[-1]
                print(f"claw {ch_h*1e3:4.0f} mm  {pn:6s} offset {off*1e3:3.0f} mm: clear dirs {r['frac_dirs_clear_in_every_pose']:5.3f}, "
                      f"up30 {r['any_dir_within_30deg_of_up_clear_in_every_pose']}, steepest clear {r['highest_clear_dir_tilt_from_up_deg']} deg from up"
                      f"  | IK ok {sum(p['ik_ok'] for p in per_pose)}/{len(per_pose)}")
    out["rows"] = rows
    json.dump(out, open(os.path.join(HERE, "..", "results", "prop_clearance.json"), "w"), indent=1)

if __name__ == "__main__":
    main()
