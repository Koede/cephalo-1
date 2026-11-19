"""Rotor-leg checks over the walk: finger clearance to the floor and between neighbouring leg hands.
Fingers modelled as 4 segments (r 20 -> 114.3 mm) in the blade plane, parked as an X (clock 45 deg)."""
import sys, json, numpy as np, mujoco
import chilo2 as C, chilo_arm_legs as A
PARK = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0     # finger azimuth in the leg frame (rotor parked)
tag = sys.argv[1] if len(sys.argv) > 1 else "_assist4"
m = C.build(); d = mujoco.MjData(m); z = np.load(f"chilo2_walk{tag}.npz")
legs = [f"{'L' if s > 0 else 'R'}{seg}" for seg in range(1, 6) for s in (1, -1)]
def fingers(n):
    b = m.body(f"{n}_l4").id; R = d.xmat[b].reshape(3, 3); u = R[:, 0]
    sp = d.site_xpos[m.site(f"{n}_ft").id]; c = sp + u * (A.BAYONET + 0.096)
    out = []
    for k in range(4):
        a = np.radians(PARK + 90 * k); e = np.cos(a) * R[:, 1] + np.sin(a) * R[:, 2]
        out.append((c + e * 0.020, c + e * 0.1143))
    return u, out
def segdist(p1, q1, p2, q2):
    s = np.linspace(0, 1, 15); P = p1 + (q1 - p1) * s[:, None]; Q = p2 + (q2 - p2) * s[:, None]
    return np.min(np.linalg.norm(P[:, None] - Q[None], axis=2))
tilt, floor, pair = 0, 1, 1; worst = {}
for f in range(0, len(z["t"]), 3):
    d.qpos[:] = z["qpos"][f]; mujoco.mj_kinematics(m, d)
    H = {}
    for n in legs:
        u, F = fingers(n); H[n] = F
        tilt = max(tilt, np.degrees(np.arccos(-u[2])))
        lo = min(min(a[2], b[2]) for a, b in F) - 0.0015          # blade half-thickness
        if lo < floor: floor = lo; worst["floor"] = (n, float(z["t"][f]))
    for side in "LR":
        for seg in range(1, 5):
            a, b = f"{side}{seg}", f"{side}{seg + 1}"
            g = min(segdist(*fa, *fb) for fa in H[a] for fb in H[b]) - 0.004
            if g < pair: pair = g; worst["pair"] = (a, b, float(z["t"][f]))
out = dict(shank_tilt_max_deg=round(float(tilt), 1), finger_floor_clear_min_mm=round(floor * 1e3, 1), neighbour_finger_gap_min_mm=round(pair * 1e3, 1), worst=worst)
print(json.dumps(out)); json.dump(out, open(f"rotor_leg_check{tag}.json", "w"))
