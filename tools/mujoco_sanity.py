#!/usr/bin/env python3
"""MuJoCo cross-check of the arm's kinematic model (tools/kin.py) and basic dynamics sanity.

  python3 tools/mujoco_sanity.py                 # 13-joint 1 m helix60 arm
  python3 tools/mujoco_sanity.py --poses 50 --seconds 2
Builds an MJCF chain from the same joint axes/pitch as the CAD (hinge k about (cos k*twist, sin k*twist, 0)), then:
 1. FK: MuJoCo tip position == Kin.tip for random poses (<0.01 mm)  [validates axes and composition order]
 2. Jacobian: mj_jacSite translational Jacobian == Kin.jacobian[:3]
 3. Joint limits: a driven pose beyond the range stays inside +-range/2 (hinge limits on)
 3b. contacts are disabled (links overlap at the hinges), limits are soft constraints
 4. No NaN/inf over --seconds of free swing under gravity from a bent pose
 5. Energy: with damping 0, no joint limits (they are inelastic) and a small timestep the total energy drifts < 1 %
 6. Determinism: two runs from the same state give identical results
Needs `pip install mujoco`. Exit 1 on any failure. Uses uniform link mass; it checks the model, not the real servo dynamics
(joint-study/sim/dynamics.py does that with the sourced masses and servo curves).
"""
import argparse, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Checks
from kin import Kin, axis_dir


def mjcf(k, half_deg, link_kg=0.12, damping=0.0, dt=0.0005, limited=True):
    L = [f'<mujoco><option timestep="{dt}" gravity="0 0 -9810" integrator="RK4"><flag energy="enable" contact="disable"/></option><worldbody>']
    ind = ""
    for j in range(k.J):
        pos = "0 0 0" if j == 0 else f"0 0 {k.pitch}"
        ax = axis_dir(j, k.twist)
        L.append(f'{ind}<body name="l{j+1}" pos="{pos}"><joint name="j{j}" type="hinge" axis="{ax[0]:.12f} {ax[1]:.12f} 0" range="{-half_deg} {half_deg}" limited="{str(limited).lower()}" damping="{damping}"/>'
                 f'<geom type="capsule" fromto="0 0 0 0 0 {k.pitch}" size="12" mass="{link_kg}"/>')
        ind += " "
    L.append(f'{ind}<site name="tip" pos="0 0 {k.pitch}" size="2"/>')
    L.append("</body>" * k.J + "</worldbody></mujoco>")
    return "".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joints", type=int, default=13); ap.add_argument("--length", type=float, default=1000.0); ap.add_argument("--twist", type=float, default=60.0)
    ap.add_argument("--range", type=float, default=90.0); ap.add_argument("--poses", type=int, default=20); ap.add_argument("--seconds", type=float, default=1.0); ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    try: import mujoco
    except ImportError: sys.exit("mujoco not installed: pip install mujoco")
    k = Kin(a.joints, a.length / a.joints, a.twist); half = a.range / 2; rng = np.random.default_rng(a.seed)
    C = Checks(f"MuJoCo {mujoco.__version__}: {k.J}-joint arm, {a.length:g} mm, twist {a.twist:g}, +-{half:g} deg")
    m = mujoco.MjModel.from_xml_string(mjcf(k, half)); d = mujoco.MjData(m); sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "tip")
    ferr = jerr = 0.0
    for _ in range(a.poses):
        q = np.radians(rng.uniform(-half, half, k.J)); d.qpos[:] = q; mujoco.mj_forward(m, d)
        ferr = max(ferr, float(np.linalg.norm(d.site_xpos[sid] - k.tip(q))))
        Jp = np.zeros((3, m.nv)); mujoco.mj_jacSite(m, d, Jp, None, sid); jerr = max(jerr, float(np.abs(Jp - k.jacobian(q)[:3]).max()))
    C.add("FK matches kin.py", ferr < 0.01, f"max tip error {ferr:.2e} mm over {a.poses} poses")
    C.add("Jacobian matches kin.py", jerr < 1e-6, f"max |dJ| {jerr:.2e}")
    d = mujoco.MjData(m); d.qpos[:] = np.radians(half + 30); mujoco.mj_forward(m, d)
    for _ in range(2000): mujoco.mj_step(m, d)
    C.add("joint limits respected", bool((np.abs(d.qpos) <= np.radians(half) + np.radians(5)).all()), f"max |q| {np.degrees(np.abs(d.qpos).max()):.1f} deg (limit {half:g}; MuJoCo limits are soft, 5 deg overshoot allowed)")
    m2 = mujoco.MjModel.from_xml_string(mjcf(k, 180, limited=False))   # no limits: they are inelastic stops; d = mujoco.MjData(m2); d.qpos[:] = np.radians(rng.uniform(-40, 40, k.J)); mujoco.mj_forward(m2, d)
    e0 = d.energy[0] + d.energy[1]; steps = int(a.seconds / m2.opt.timestep); ok = True
    for _ in range(steps):
        mujoco.mj_step(m2, d); ok &= bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
    e1 = d.energy[0] + d.energy[1]
    C.add("no NaN/inf over the run", ok, f"{steps} steps")
    pot_scale = abs(mujoco.mj_getTotalmass(m2)) * 9810.0 * a.length          # model units are mm, kg, s
    C.add("energy drift < 1 % of m g L", abs(e1 - e0) < 0.01 * pot_scale, f"E0 {e0:.4g} -> {e1:.4g} (m g L = {pot_scale:.4g} kg mm^2/s^2), drift {abs(e1-e0)/pot_scale*100:.4f} %")
    def run():
        dd = mujoco.MjData(m2); dd.qpos[:] = np.radians(np.linspace(-30, 30, k.J)); [mujoco.mj_step(m2, dd) for _ in range(500)]; return dd.qpos.copy()
    C.add("deterministic", np.array_equal(run(), run()), "two identical runs")
    C.exit()


if __name__ == "__main__":
    main()
