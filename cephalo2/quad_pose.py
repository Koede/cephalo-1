"""Flight pose for the 4 flying arms in quad mode (python3 quad_pose.py): a near-vertical 'candle' so the 7.5 in claw-prop's
thrust line passes close to the pitch-joint axes; compared with the tulip flight pose, at the quad-mode thrust."""
import numpy as np, json, cephalo4 as C, power4 as PW
C.M_POD += PW.PROPS["xl"]["dM_pod"]
Q_FLY = np.radians(json.load(open("poses4.json"))["flight"])
BP, BR = np.zeros(3), np.eye(3)
T_HOVER, T_PEAK = 21.5, 26.5          # N per flying claw-prop: quad mode with tools / peak at the ceiling


def hub(i, q):
    _, (R, p) = C.arm_fk(i, q, BP, BR)
    return p + R[:, 0] * C.BLADE_OFF, R[:, 0]


def loads(i, q, T, lean=0.0):
    h, a = hub(i, q)
    if lean:                                          # thrust axis leaned tangentially (steering / yaw)
        az = np.radians(22.5 + 45 * i); t = np.array([-np.sin(az), np.cos(az), 0])
        a = np.cos(lean) * a + np.sin(lean) * t
    tau = C.joint_torques(i, q, BP, BR, T * a, h)
    return np.abs(tau) / C.STALL


def candle(i, r_h, z_h, q0):
    az = np.radians(22.5 + 45 * i)
    tgt = np.array([r_h * np.cos(az), r_h * np.sin(az), z_h])
    return C.ik(i, tgt, np.array([0, 0, 1.0]), q0, q0, BP, BR, lead=C.BLADE_OFF, w_ax=2.0)


if __name__ == "__main__":
    best = None
    q0 = np.radians([-50, 0, -30, 0, 0, 0, 0, 0])
    for r_h in np.arange(0.16, 0.31, 0.02):
        for z_h in np.arange(0.36, 0.50, 0.02):
            q = candle(0, r_h, z_h, q0)
            h, a = hub(0, q)
            if np.linalg.norm(h - [r_h * np.cos(np.radians(22.5)), r_h * np.sin(np.radians(22.5)), z_h]) > 0.01 or a[2] < 0.98: continue
            segs, tip = C.arm_fk(0, q, BP, BR)
            P = np.array([C.at_s(segs, s)[0] for s in np.linspace(0.06, C.SPIGOT, 30)])
            clr = C.body_clearance(P, np.full(len(P), 0.024), np.zeros(3)).min()
            if clr < 0.015: continue
            worst = max(loads(0, q, T_PEAK).max(), loads(0, q, T_PEAK, np.radians(15)).max())
            if best is None or worst < best[0]: best = (worst, r_h, z_h, q, clr)
    worst, r_h, z_h, q, clr = best
    print(f"candle pose: hub r {r_h:.2f} m, {z_h:.2f} m above the plate, body clearance {clr * 1000:.0f} mm, disc gap to the next flyer "
          f"{(r_h * np.sqrt(2) - 2 * PW.PROPS['xl']['R']) * 1000:.0f} mm | q = {np.degrees(q).round(1).tolist()}")
    for nm, qq in (("tulip (current flight pose)", Q_FLY), ("candle", q)):
        for T, lab in ((T_HOVER, "hover"), (T_PEAK, "peak")):
            l0 = loads(0, qq, T); l1 = loads(0, qq, T, np.radians(15))
            print(f"  {nm:28s} {lab:5s} {T:4.1f} N: worst joint {l0.max():.2f} x stall (j{l0.argmax()}), leaning 15 deg {l1.max():.2f} (j{l1.argmax()})")
    json.dump(dict(q_candle=np.degrees(q).tolist(), r_h=float(r_h), z_h=float(z_h)), open("quad_pose.json", "w"), indent=1)
