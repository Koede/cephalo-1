"""Tip fan joint study: which wrist lets the claw-prop thrust point in the most directions.

Frame: z = tentacle axis at the tip (pointing out), the tentacle body lies along -z behind the wrist.
A thrust direction n is usable if the mechanism can point the fan there, the fan disc does not hit the
tentacle, and it is not inside a singular (keyhole) band. Coverage is the usable fraction of the sphere.
Mechanism numbers are screening estimates from catalog/servo_and_tip_sources.json (ranges stated there,
moving mass / parts / difficulty estimated); they are inputs to argue with."""
import math, json, os
import numpy as np
import config as C

FAN_DISC_D = 0.090      # m, prop or duct outer diameter [A]
CLEAR = 0.006           # m clearance to the tentacle surface
R_TENT = C.D_TIP / 2

# id, dof, cone (max angle of fan axis from tentacle axis, deg), roll_about_arm (bool: can swing the cone around),
# singular bands as (axis, half-width deg), actuators, actuator location, moving mass g, slew deg/s, parts, difficulty 0-1,
# pivot-to-disc offset mm
MECH = [
 dict(id="pan_tilt_servo_gimbal", name="Pan-tilt servo gimbal (roll about arm + tilt)", cone=120, sing=[("z", 10)], act=2, at_tip=2, mass=55, slew=400, parts=6, diff=0.10, off=28),
 dict(id="bevel_differential_wrist", name="Bevel differential wrist (both servos behind the joint)", cone=90, sing=[("z", 10)], act=2, at_tip=0, mass=35, slew=350, parts=10, diff=0.35, off=30),
 dict(id="rss_pushrod_ball_joint", name="2-RSS pushrod ball joint", cone=40, sing=[], act=2, at_tip=0, mass=25, slew=300, parts=8, diff=0.15, off=22),
 dict(id="hooke_plus_roll", name="Double Hooke joint with roll", cone=90, sing=[], act=3, at_tip=1, mass=60, slew=300, parts=12, diff=0.35, off=35),
 dict(id="omni_wrist_III", name="Omni-Wrist III style printed linkage", cone=92.5, sing=[], act=2, at_tip=0, mass=70, slew=250, parts=22, diff=0.70, off=40),
 dict(id="rosheim_double_gimbal_wrist", name="Rosheim double-gimbal CV wrist", cone=90, sing=[], act=3, at_tip=0, mass=65, slew=250, parts=18, diff=0.60, off=38),
 dict(id="milenkovic_2axis_nonsingular", name="Milenkovic 2-axis CV wrist", cone=100, sing=[], act=2, at_tip=0, mass=50, slew=300, parts=14, diff=0.55, off=34),
 dict(id="rolling_spm_printed", name="Printed pure-rolling spherical parallel wrist", cone=75, sing=[], act=3, at_tip=0, mass=60, slew=300, parts=16, diff=0.55, off=32),
 dict(id="agile_eye_3rrr_spm", name="Agile Eye 3-RRR spherical wrist", cone=70, sing=[("cone_edge", 5)], act=3, at_tip=0, mass=45, slew=600, parts=12, diff=0.45, off=30),
 dict(id="ball_drive_omniwheel_sphere", name="Ball Drive: sphere turned by 3 omni-wheels", cone=65, sing=[], act=3, at_tip=3, mass=110, slew=200, parts=20, diff=0.55, off=30),
 dict(id="hog_drive_hemisphere", name="HOG-style tilting hemisphere (2 servos)", cone=45, sing=[], act=2, at_tip=2, mass=60, slew=250, parts=10, diff=0.30, off=26),
 dict(id="cable_spherical_joint", name="Tendon-driven ball at the tip", cone=50, sing=[], act=2, at_tip=0, mass=15, slew=150, parts=4, diff=0.30, off=20, remote=True),
 dict(id="thrust_vector_rotor_tilt", name="Single-axis 360 deg fan tilt (Voliro style)", cone=180, sing=[("y_plane_only", 0)], act=1, at_tip=1, mass=30, slew=500, parts=5, diff=0.10, off=26, one_axis=True),
 dict(id="tilt360_plus_arm_roll", name="Single-axis 360 deg fan tilt + tentacle roll module", cone=180, sing=[("z", 6)], act=2, at_tip=1, mass=45, slew=400, parts=8, diff=0.20, off=26),
 dict(id="abenics_spherical_gear", name="ABENICS spherical-gear ball", cone=110, sing=[], act=4, at_tip=4, mass=160, slew=150, parts=24, diff=0.85, off=40),
 dict(id="duta_stanisic_spherical_wrist", name="Duta-Stanisic semicircular-link wrist", cone=150, sing=[], act=2, at_tip=0, mass=90, slew=200, parts=20, diff=0.85, off=40),
 dict(id="three_roll_wrist", name="Canted three-roll wrist", cone=150, sing=[("cone_edge", 8)], act=3, at_tip=0, mass=90, slew=250, parts=20, diff=0.80, off=40),
]

def fib_sphere(n=4000):
    i = np.arange(n) + 0.5; phi = np.arccos(1 - 2 * i / n); th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)

def disc_hits_tentacle(n, off):
    """fan disc centred at off*n, normal n, radius rf; tentacle = cylinder radius R_TENT along -z from the pivot"""
    rf = FAN_DISC_D / 2
    u = np.cross(n, [1.0, 0, 0]) if abs(n[0]) < 0.9 else np.cross(n, [0, 1.0, 0]); u /= np.linalg.norm(u)
    w = np.cross(n, u)
    ang = np.linspace(0, 2 * math.pi, 48, endpoint=False)
    for rr in (rf, 0.6 * rf, 0.25 * rf):
        pts = off * n + rr * (np.outer(np.cos(ang), u) + np.outer(np.sin(ang), w))
        bad = (pts[:, 2] < -0.004) & (np.hypot(pts[:, 0], pts[:, 1]) < R_TENT + CLEAR)
        if bad.any(): return True
    # pivot housing itself: thrust aimed straight back into the tentacle
    return n[2] < -math.cos(math.atan2(R_TENT + CLEAR + rf, off + 0.03))

def evaluate(m, dirs):
    cone = math.radians(m["cone"]); off = m["off"] * 1e-3
    ang = np.arccos(np.clip(dirs[:, 2], -1, 1))
    reach = ang <= cone + 1e-9
    if m.get("one_axis"):
        reach &= np.abs(dirs[:, 1]) < math.sin(math.radians(4))   # a great circle only
    hit = np.array([disc_hits_tentacle(d, off) for d in dirs])
    usable = reach & ~hit
    sing = np.zeros(len(dirs), bool)
    for kind, w in m["sing"]:
        if kind == "z": sing |= ang < math.radians(w)
        if kind == "cone_edge": sing |= ang > cone - math.radians(w)
    smooth = usable & ~sing
    cov = usable.mean(); cov_s = smooth.mean()
    up_needed = dirs[:, 2] > -2.0   # all directions
    # added root moment from tip mass at 585 mm
    d_root = (m["mass"] - 30) * 1e-3 * C.G * C.ARM_LENGTH
    remote_pen = 0.6 if m.get("remote") else 1.0     # tip tendons run the full tentacle: friction + coupling
    build = (1 - 0.7 * m["diff"]) * (1 / (1 + m["parts"] / 25)) ** 0.5
    score = (cov_s ** 1.0) * (min(1, m["slew"] / 300) ** 0.3) * math.exp(-max(d_root, 0) / 0.6) * build ** 0.8 * remote_pen
    return dict(id=m["id"], name=m["name"], coverage_sphere=cov, coverage_smooth=cov_s, cone_deg=m["cone"],
                blocked_by_disc=float((reach & hit).mean()), actuators=m["act"], actuators_at_tip=m["at_tip"],
                moving_mass_g=m["mass"], extra_root_moment_Nm=d_root, slew_dps=m["slew"], parts=m["parts"],
                difficulty=m["diff"], score=score)

def geometric_limit(off):
    """best possible coverage for any wrist with this pivot-to-disc offset (the tentacle is the obstacle)"""
    dirs = fib_sphere(3000)
    hit = np.array([disc_hits_tentacle(d, off) for d in dirs])
    return float((~hit).mean())

def run(out_dir):
    dirs = fib_sphere(2000)
    rows = sorted((evaluate(m, dirs) for m in MECH), key=lambda r: -r["score"])
    lim = {f"{o} mm": geometric_limit(o / 1000) for o in (20, 30, 40, 60)}
    with open(os.path.join(out_dir, "tip_study.json"), "w") as f:
        json.dump(dict(fan_disc_d_mm=FAN_DISC_D * 1e3, geometric_limit_by_offset=lim, mechanisms=rows), f, indent=1)
    return rows, lim

if __name__ == "__main__":
    os.makedirs("results", exist_ok=True)
    rows, lim = run("results")
    print("max coverage by offset", lim)
    for r in rows: print(f'{r["id"]:32s} cov {r["coverage_sphere"]:.2f} smooth {r["coverage_smooth"]:.2f} disc-block {r["blocked_by_disc"]:.2f} dM {r["extra_root_moment_Nm"]:.2f} score {r["score"]:.3f}')
