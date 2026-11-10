"""Joint loads and actuator sizing for the per-vertebra tendon arm (RollingEach) driven from a base box.
Static load cases (kinematic poses + external forces), moments about each rolling contact, tendon tension, servo pick."""
import numpy as np
from tentacles import RollingEach, Tr, Rt, H, Y, Z, X
from writhe_grid import SF
G = 9.81
arm = RollingEach(SF)
n = arm.n                      # 19 joints: root p, root y, 17 vertebra joints
# ---- masses (estimates): PA12 vertebrae 45 % solid, TPU skin 60 g spread by radius, tendons 15 g, pod 100 g
rho = 1010.0
m_v = rho * 0.45 * np.pi * arm.r ** 2 * arm.h
m_skin = 0.060 * arm.r / arm.r.sum()
m_vert = m_v + m_skin + 0.015 / arm.NV
M_POD = 0.100
BASE = H(R=np.eye(3), p=[0.104, 0, 0.039])        # arm 0 root on the body (body frame), x radial out

def pose(root_p, root_y, pitch_total, yaw_total=0.0, shape=1.0):
    q = np.zeros(n); q[0] = np.radians(root_p); q[1] = np.radians(root_y)
    pi = [k for k in range(2, n) if arm.typ[k] == "p"]; yi = [k for k in range(2, n) if arm.typ[k] == "y"]
    w = np.linspace(1, shape, len(pi)); q[pi] = np.radians(pitch_total) * w / w.sum()
    w = np.linspace(1, shape, len(yi)); q[yi] = np.radians(yaw_total) * w / w.sum()
    return q

def torques(q, F_tip_world, g_mult=1.0, foot_off=0.0):
    frames, tip = arm.fk(q, BASE)
    # point loads: vertebra centres + pod + tip force
    pts, F = [], []
    for v in range(arm.NV):
        M = frames[v + 2]; off = 0.015 if v == 0 else 0.0
        pts.append(M[:3, :3] @ (X * (off + arm.h[v] / 2)) + M[:3, 3]); F.append([0, 0, -G * m_vert[v] * g_mult])
    tp = tip[:3, 3] + tip[:3, 0] * 0.03
    pts.append(tp); F.append([0, 0, -G * M_POD * g_mult])
    pts.append(tip[:3, 3] + tip[:3, 0] * foot_off); F.append(F_tip_world(tip))
    pts, F = np.array(pts), np.array(F)
    s_pts = np.concatenate([[0.045 + arm.cum[v] + arm.h[v] / 2 for v in range(arm.NV)], [9, 9]])
    tau = np.zeros(n)
    for j in range(n):
        Mj = frames[j + 1]; c = Mj[:3, 3]; a = Mj[:3, 1] if arm.typ[j] == "p" else Mj[:3, 2]
        sel = s_pts > arm.s[j]
        tau[j] = a @ np.cross(pts[sel] - c, F[sel]).sum(0)
    return tau, tip

cases = {
  "stand, 12.3 N per foot (fans off)": (pose(30, 0, 60, shape=2.0), lambda tip: [0, 0, 12.3], 1.0, 0.07),
  "flight tulip, 10.8 N max thrust":   (pose(-40, 0, -50, shape=1.5), lambda tip: tip[:3, 0] * 10.8, 1.0, 0.034),
  "flight, thrust vectored 30 deg sideways": (pose(-40, 0, -50, shape=1.5),
        lambda tip: (np.cos(np.radians(30)) * tip[:3, 0] + np.sin(np.radians(30)) * tip[:3, 1]) * 10.8, 1.0, 0.034),
  "reach out horizontal, 3 g": (pose(0, 0, 0), lambda tip: [0, 0, 0], 3.0, 0.0),
}
TAU = {}
for name, (q, f, gm, fo) in cases.items():
    TAU[name], _ = torques(q, f, gm, fo)
tau_max = np.max(np.abs(np.array(list(TAU.values()))), axis=0)
# ---- tendon tension: moment arm 0.75 r at the joint, capstan friction through proximal joints in the worst pose, 20 % preload
r_j = np.concatenate([[np.nan, np.nan], 0.5 * (arm.r[:-1] + arm.r[1:])])
d_j = 0.75 * r_j
mu = 0.10                                      # PTFE-lined channel (est.)
wrap = np.array([np.sum(np.abs(np.radians(45)) * (np.arange(n) < j) * (np.arange(n) >= 2) * 0.5) for j in range(n)])  # half the proximal joints at 45 deg
fric = np.exp(mu * wrap)
T = tau_max / d_j * fric * 1.2
# ---- servos (datasheet: rated/stall torque N m, mass g, dims mm). STS3250 rated torque not published -> 1/3 stall (est.)
SERVOS = [("STS3032", 0.147, 0.441, 20.0, (23.2, 12.1, 28.5), "6 V"),
          ("STS3215", 0.981, 2.94, 55.0, (45.2, 24.7, 35.0), "12 V"),
          ("STS3250", 1.63, 4.90, 74.5, (45.2, 24.7, 35.0), "12 V")]
R_SPOOL_MIN = 0.004
pick = []
for j in range(2, n):
    for sv in SERVOS:
        rs = sv[1] / T[j]
        if rs >= R_SPOOL_MIN:
            pick.append((j, sv, min(rs, 0.008))); break
    else:
        pick.append((j, None, None))
if __name__ == "__main__":
    np.set_printoptions(precision=2, suppress=True)
    print("vertebra masses g:", np.round(m_vert * 1000, 1), "sum", round(m_vert.sum() * 1000), "+ pod", M_POD * 1000)
    for name, t in TAU.items():
        print(f"{name:42s}", np.round(t, 2))
    print("root knuckle |tau| max (N m): pitch %.2f  yaw %.2f" % (tau_max[0], tau_max[1]))
    tot_m = 0; vol = 0
    print(" j  type  s(mm)  r(mm)  |tau|Nm  T(N)  servo    spool r(mm)  tension cap rated/stall (N)")
    for j, sv, rs in pick:
        print(f"{j:2d}  {arm.typ[j]}   {arm.s[j]*1000:5.0f}  {r_j[j]*1000:5.1f}  {tau_max[j]:6.2f}  {T[j]:5.0f}  {sv[0] if sv else 'NONE':8s} {rs*1000 if rs else 0:5.1f}"
              f"      {sv[1]/rs if sv else 0:5.0f} / {sv[2]/rs if sv else 0:5.0f}")
        if sv: tot_m += sv[3]; vol += np.prod(sv[4]) / 1000
    print("tendon servos: mass %.0f g, volume %.0f cm3" % (tot_m, vol))
