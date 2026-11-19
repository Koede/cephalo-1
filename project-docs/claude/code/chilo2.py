"""Chilo-1 rev 2 (face dock, octopus as the head, style-A arm-matched legs): single source of truth + MuJoCo model.

Body: 6 rounded segments (head 280 mm, then 5 x 200 mm, radii 125 -> 75 mm), body axis 380 mm above the floor,
5 spine joints (yaw STS3215 + pitch STS3250 at one pivot).  Head segment has no legs: Cephalo-1 is docked on its face
(Scabbard host hub) and its arms 0/7 (low) and 1/6 (side) are the front legs; arms 2-5 are hands (rigid here).
Body legs (segments 1-5), style A: coxa yaw STS3215 + 4 pitch joints (STS3250, STS3250, STS3215, STS3215) on
56-67 mm links, the same link cradles as the octopus arm; foot = TPU ball.  Masses est.
SI metres.  World x forward, z up.
"""
import numpy as np
import mujoco
import chilo_arm_legs as A

HC = 0.38
SEG_L = [0.28, 0.20, 0.20, 0.20, 0.20, 0.20]
SEG_R = [0.125, 0.112, 0.100, 0.095, 0.085, 0.075]
GAP = 0.035
# segment centres (x) as in chilo_face_dock.chain([0, 0, 0])
SEG_X = [0.645, 0.370, 0.135, -0.100, -0.335, -0.570]
PIVOT_X = [None, 0.4875, 0.2525, 0.0175, -0.2175, -0.4525]       # pivot between segment i-1 and i
FACE_X = 0.785
PLATE = np.array([FACE_X + 0.012, 0.0, HC])

#        rated  stall  mass   armature  stall current (A, est.)
SV = {"STS3215": (0.981, 2.94, 0.055, 0.012, 2.7), "STS3250": (1.63, 4.90, 0.0745, 0.020, 3.8)}
I_IDLE = 0.12                                                    # A per servo, holding/electronics (est.)

# ---- body leg, style A (leg plane: dy outward from the hip, dz up), nominal standing shape
# Rotor legs (2026-11-16, Forest: "centipede legs get rotors"): every body leg ends in a bayonet + hand3 (the same propeller hand
# as the octopus arms).  The leg stands on the hand3 heel (Z heel 114.2 mm + 9.9 mm bayonet = 124.1 mm past the leg's
# spigot) with the fingers flat as a rotor, 18 mm above the floor; the shank is held vertical (leg_ik) so they stay clear.
LEG_HAND = A.BAYONET + 0.1142                                     # leg spigot -> heel contact
R_HEEL = 0.009                                                    # heel contact sphere (V-saddle footprint 18 x 17 mm)
LEG_PTS = np.array([(0.0, 0.0), (0.04, 0.0), (0.085, -0.045), (0.115, -0.105), (0.13, -0.17), (0.13, -(HC - LEG_HAND))])
LEG_SERVOS = ["STS3215", "STS3250", "STS3250", "STS3215", "STS3215"]      # coxa yaw, p1..p4
LEG_LEN = np.linalg.norm(np.diff(LEG_PTS[1:], axis=0), axis=1)          # 4 links after the coxa link
LEG_TH0 = np.arctan2(-np.diff(LEG_PTS[1:], axis=0)[:, 1], np.diff(LEG_PTS[1:], axis=0)[:, 0])   # nominal absolute link angles (down +)
LEG_LIM = np.radians([[-40, 40], [-90, 90], [-100, 100], [-100, 100], [-100, 100]])
M_LEGJ = [0.055, 0.0945, 0.0945, 0.075, 0.075]; M_FOOTLINK = 0.020          # shank 86 mm now (was 195 mm, 35 g)
# leg tip (offset from the leg spigot along the shank, kg): hand electronics (ESC 6S 45 A + node, as the arm tip bay),
# bayonet set, hand3 hub C2 (396 g, COM 58.4 mm past its base face)
LEG_TIP_MASSES = [(-0.012, 0.024), (0.003, 0.026), (A.BAYONET + 0.0584, 0.396)]
R_FOOT = 0.015

# ---- segments
M_SHELL = [0.50, 0.45, 0.45, 0.45, 0.40, 0.35]; M_HUB = 0.40; M_PACK = 0.81; PACK_SEGS = (2, 3); M_ELEC = 0.07; M_SPINE = 0.17
# ---- octopus
M_OCTO = A.M_OCTO                       # Cephalo-2: 14.81 kg
ARM_LEGS = [0, 7, 1, 6]; ARM_HANDS = [2, 3, 4, 5]
M_ARM_MOVING = float(np.sum(A.M_JOINT) + A.M_TIP)                          # Cephalo-2 arm: joints + links + tip bay + bayonet + hand3
M_OCTO_BODY = M_OCTO - 4 * M_ARM_MOVING                                    # rigid part incl. the four hand arms
Z_OCTO_COM = 0.10
DT = 0.001
D2 = np.radians(2.0)


def leg_fk_planar(q4, psi=0.0):
    """Leg-local planar FK (dy outward, dz) of the joints from p1 on, pitch q4 (deltas from nominal, down +)."""
    p = LEG_PTS[1].copy(); pts = [p.copy()]; th = 0.0
    for k in range(4):
        th = LEG_TH0[k] + q4[k] if k == 0 else th + (LEG_TH0[k] - LEG_TH0[k - 1]) + q4[k]
        p = p + LEG_LEN[k] * np.array([np.cos(th), -np.sin(th)]); pts.append(p.copy())
    return np.array(pts)


def leg_shank_angle(q4):
    """Absolute angle of the shank (down +, 90 deg = vertical, foot under the knee)."""
    return LEG_TH0[3] + float(np.sum(q4))


W_SHANK = 0.08            # m per rad: weight of the shank-vertical row in leg_ik


def leg_ik(target_local, q_prev, iters=6, shank=np.pi / 2):
    """target_local = leg spigot in the hip frame (x fwd, y outward, z up) -> [yaw, p1..p4] (deltas from nominal).
    The shank is held at `shank` (vertical by default) so the hand3 rotor stays flat over the floor."""
    x, y, z = target_local
    psi = np.arctan2(x, y); r = np.hypot(x, y)
    q = q_prev[1:].copy()
    tgt = np.array([r, z, W_SHANK * shank])
    pose = lambda q: np.r_[leg_fk_planar(q)[-1], W_SHANK * leg_shank_angle(q)]
    for _ in range(iters):
        f = pose(q)
        e = tgt - f
        if np.linalg.norm(e) < 1e-5: break
        J = np.zeros((3, 4)); h = 1e-5
        for k in range(4):
            dq = q.copy(); dq[k] += h; J[:, k] = (pose(dq) - f) / h
        lam = 1e-4; W = np.diag([1.0, 1.0, 1.6, 2.0])
        Wi = np.linalg.inv(W)
        G = Wi @ J.T @ np.linalg.inv(J @ Wi @ J.T + lam * np.eye(3))
        dq = G @ e - 0.02 * (np.eye(4) - G @ J) @ q
        q = np.clip(q + dq, LEG_LIM[1:, 0], LEG_LIM[1:, 1])
    return np.concatenate([[psi], q])


ARM_MARGIN = 0.70        # never command an arm joint past 70 % of its range (Forest: don't overextend the arms)


HAND_TILT_MAX = np.radians(6.0)   # planted hand3 axis within 6 deg of vertical: it stands on its heel with the fingers flat,
                                  # 18 mm above the floor at 0 deg (heel 114.2, blade plane 96), >= 6 mm at 6 deg
W_AX = 0.10                       # m per rad: weight of the hand-axis rows in the arm IK


def arm_ik(target, q_prev, root, R0, q_nom, iters=4, axis=None):
    """Foot on target (and hand axis toward `axis` if given: heel standing), joints <= 70 % of range."""
    q = q_prev.copy(); lo, hi = A.LIM[:, 0] * ARM_MARGIN, A.LIM[:, 1] * ARM_MARGIN

    def pose(q):
        _, _, sp, tp = A.fk(q, root, R0); u = (tp - sp) / A.HAND
        return np.concatenate([tp, W_AX * u]) if axis is not None else tp
    goal = np.concatenate([target, W_AX * axis]) if axis is not None else target
    for _ in range(iters):
        f = pose(q); e = goal - f
        if np.linalg.norm(e) < 1e-5: break
        J = np.zeros((len(goal), 8)); h = 1e-5
        for k in range(8):
            dq = q.copy(); dq[k] += h; J[:, k] = (pose(dq) - f) / h
        Jp = J.T @ np.linalg.inv(J @ J.T + 1e-4 * np.eye(len(goal)))
        q = np.clip(q + Jp @ e + 0.25 * (np.eye(8) - Jp @ J) @ (q_nom - q), lo, hi)
    return q


def _quat(R):
    q = np.zeros(4); mujoco.mju_mat2Quat(q, R.reshape(-1)); return q


def _act(name, joint, sv):
    st = SV[sv][1]; kp = st / D2
    return f'<position name="a_{name}" joint="{joint}" kp="{kp:.2f}" kv="{kp * 0.02:.3f}" forcerange="{-st} {st}" forcelimited="true" ctrlrange="-3.3 3.3"/>'


def leg_xml(seg, s, acts):
    n = f"{'L' if s > 0 else 'R'}{seg}"
    y = s * SEG_R[seg] * 0.92
    rel = np.array([0.0, y, 0.0])
    x = [f'<body name="{n}_hip" pos="{rel[0]} {rel[1]} {rel[2]}" euler="0 0 {s * np.pi / 2:.6f}">',
         f'<joint name="{n}_q0" type="hinge" axis="0 0 1" range="{LEG_LIM[0, 0]} {LEG_LIM[0, 1]}" armature="{SV[LEG_SERVOS[0]][3]}" damping="0.05"/>',
         f'<geom type="capsule" fromto="0 0 0 0.04 0 0" size="0.016" mass="{M_LEGJ[0]}" contype="0" conaffinity="0" rgba="0.95 0.62 0.27 1"/>',
         f'<body name="{n}_l1" pos="0.04 0 0" euler="0 {LEG_TH0[0]:.6f} 0">']
    acts.append(_act(f"{n}_q0", f"{n}_q0", LEG_SERVOS[0]))
    for k in range(4):
        L = LEG_LEN[k]
        x.append(f'<joint name="{n}_q{k + 1}" type="hinge" axis="0 1 0" range="{LEG_LIM[k + 1, 0]} {LEG_LIM[k + 1, 1]}" armature="{SV[LEG_SERVOS[k + 1]][3]}" damping="0.05"/>')
        x.append(f'<geom type="capsule" fromto="0 0 0 {L:.4f} 0 0" size="0.015" mass="{(M_LEGJ[k + 1] if k < 3 else M_LEGJ[4] + M_FOOTLINK):.4f}" contype="0" conaffinity="0" rgba="0.95 0.62 0.27 1"/>')
        acts.append(_act(f"{n}_q{k + 1}", f"{n}_q{k + 1}", LEG_SERVOS[k + 1]))
        if k < 3:
            x.append(f'<body name="{n}_l{k + 2}" pos="{L:.4f} 0 0" euler="0 {LEG_TH0[k + 1] - LEG_TH0[k]:.6f} 0">')
        else:
            for off, mt in LEG_TIP_MASSES:
                x.append(f'<geom type="sphere" pos="{L + off:.4f} 0 0" size="0.015" mass="{mt:.4f}" contype="0" conaffinity="0" rgba="0.2 0.2 0.22 1"/>')
            x.append(f'<geom name="{n}_foot" type="sphere" pos="{L + LEG_HAND - R_HEEL:.4f} 0 0" size="{R_HEEL}" mass="0.0" contype="0" conaffinity="1" friction="1.0 0.01 0.001" rgba="0.1 0.1 0.1 1"/>')
            x.append(f'<site name="{n}_ft" pos="{L:.4f} 0 0"/>')
    x += ["</body>"] * 4 + ["</body>"]
    return "\n".join(x)


def arm_xml(i, acts):
    root, R0 = A.arm_frame(i, PLATE)
    rel = root - np.array([SEG_X[0], 0, HC])
    q = _quat(R0)
    x = [f'<body name="arm{i}_root" pos="{rel[0]:.5f} {rel[1]:.5f} {rel[2]:.5f}" quat="{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}">']
    s_prev = 0.0
    for k in range(8):
        x.append(f'<body name="arm{i}_j{k}" pos="{A.S[k] - s_prev:.4f} 0 0">'); s_prev = A.S[k]
        ax = "0 1 0" if k % 2 == 0 else "0 0 1"; sv = A.SERVO[k]
        x.append(f'<joint name="arm{i}_q{k}" type="hinge" axis="{ax}" range="{A.LIM[k, 0]:.5f} {A.LIM[k, 1]:.5f}" armature="{SV[sv][3]}" damping="0.05"/>')
        seg_len = (A.S[k + 1] - A.S[k]) if k < 7 else (A.SPIGOT - A.S[7])
        x.append(f'<geom type="capsule" fromto="0 0 0 {seg_len:.4f} 0 0" size="0.016" mass="{A.M_JOINT[k]:.4f}" contype="0" conaffinity="0" rgba="0.95 0.62 0.27 1"/>')
        acts.append(_act(f"arm{i}_q{k}", f"arm{i}_q{k}", sv))
    L7 = A.SPIGOT - A.S[7]
    for off, mt in A.TIP_MASSES:
        x.append(f'<geom type="sphere" pos="{L7 + off:.4f} 0 0" size="0.02" mass="{mt:.4f}" contype="0" conaffinity="0" rgba="0.2 0.2 0.22 1"/>')
    x.append(f'<geom name="arm{i}_foot" type="sphere" pos="{L7 + A.HAND:.4f} 0 0" size="{A.FOOT_R}" mass="0.0" contype="0" conaffinity="1" friction="1.0 0.01 0.001" rgba="0.1 0.1 0.1 1"/>')
    x.append(f'<site name="arm{i}_ft" pos="{L7 + A.HAND:.4f} 0 0"/>')
    x += ["</body>"] * 8 + ["</body>"]
    return "\n".join(x)


def seg_xml(i, acts):
    m = M_SHELL[i] + M_ELEC + (M_PACK if i in PACK_SEGS else 0) + (M_HUB if i == 0 else 0)
    x = [f'<geom name="seg{i}_shell" type="capsule" fromto="{-SEG_L[i] / 2 + SEG_R[i]:.4f} 0 0 {SEG_L[i] / 2 - SEG_R[i]:.4f} 0 0" size="{SEG_R[i]}" '
         f'mass="{m:.4f}" contype="0" conaffinity="1" rgba="0.8 0.82 0.85 1"/>' if SEG_L[i] / 2 > SEG_R[i] else
         f'<geom name="seg{i}_shell" type="sphere" size="{SEG_R[i]}" mass="{m:.4f}" contype="0" conaffinity="1" rgba="0.8 0.82 0.85 1"/>']
    if i > 0:
        for s in (1, -1):
            x.append(leg_xml(i, s, acts))
    if i == 0:
        oc = PLATE - np.array([SEG_X[0], 0, HC]) + np.array([Z_OCTO_COM, 0, 0])
        x.append(f'<body name="octo" pos="{oc[0]:.4f} 0 0"><inertial pos="0 0 0" mass="{M_OCTO_BODY:.4f}" diaginertia="0.10 0.12 0.12"/>'
                 f'<geom name="octo_shell" type="sphere" size="0.14" mass="0" contype="0" conaffinity="0" rgba="0.9 0.5 0.2 0.0"/></body>')
        for a in ARM_LEGS:
            x.append(arm_xml(a, acts))
    return "\n".join(x)


def chain_xml(i, acts):
    if i == 0:
        head = f'<body name="seg0" pos="{SEG_X[0]} 0 {HC}"><freejoint name="root"/>'
    else:
        piv = PIVOT_X[i] - SEG_X[i]
        head = (f'<body name="seg{i}" pos="{SEG_X[i] - SEG_X[i - 1]:.4f} 0 0">'
                f'<joint name="sy{i}" type="hinge" axis="0 0 1" pos="{piv:.4f} 0 0" range="-0.6 0.6" armature="{SV["STS3215"][3]}" damping="0.05"/>'
                f'<joint name="sp{i}" type="hinge" axis="0 1 0" pos="{piv:.4f} 0 0" range="-0.6 0.6" armature="{SV["STS3250"][3]}" damping="0.05"/>'
                f'<geom type="sphere" pos="{piv:.4f} 0 0" size="0.04" mass="{M_SPINE}" contype="0" conaffinity="0" rgba="0.7 0.72 0.75 1"/>')
        acts.append(_act(f"sy{i}", f"sy{i}", "STS3215")); acts.append(_act(f"sp{i}", f"sp{i}", "STS3250"))
    return head + "\n" + seg_xml(i, acts) + "\n" + (chain_xml(i + 1, acts) if i < 5 else "") + "</body>"


def build():
    acts = []
    body = chain_xml(0, acts)
    xml = f"""<mujoco model="chilo1_rev2">
<compiler angle="radian"/>
<option timestep="{DT}" gravity="0 0 -9.81" integrator="implicitfast"/>
<default><geom solref="0.004 1" solimp="0.9 0.95 0.001"/><joint limited="true"/></default>
<worldbody>
<geom name="floor" type="plane" size="20 20 0.1" contype="1" conaffinity="1" friction="1.0 0.01 0.001" rgba="0 0 0 0"/>
{body}
</worldbody>
<actuator>{''.join(acts)}</actuator>
</mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    return m


def servo_of_actuator(m):
    out = []
    for a in range(m.nu):
        n = m.actuator(a).name[2:]
        if n.startswith("arm"):
            out.append(A.SERVO[int(n[-1])])
        elif n.startswith(("sy",)):
            out.append("STS3215")
        elif n.startswith(("sp",)):
            out.append("STS3250")
        else:
            out.append(LEG_SERVOS[int(n[-1])])
    return out


if __name__ == "__main__":
    m = build()
    print("bodies %d  dofs %d  actuators %d  total mass %.2f kg (octopus %.2f, centipede %.2f)" % (
        m.nbody, m.nv, m.nu, m.body_subtreemass[1], M_OCTO, m.body_subtreemass[1] - M_OCTO))
    print("leg links mm", (LEG_LEN * 1e3).round(1), " nominal link angles deg", np.degrees(LEG_TH0).round(1))
