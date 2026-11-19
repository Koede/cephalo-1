s=open('body4.py').read()
def rep(a,b,n=1):
    global s
    assert s.count(a)==n, (s.count(a), a[:70]); s=s.replace(a,b)
rep('''XMASS = (PACKS - 1) * PW.M_PACK + float(os.environ.get("CEPH_XMASS", "0"))
if XMASS:
    S4.M_TORSO += XMASS; C.M_ROBOT += XMASS; C.W_ROBOT = C.M_ROBOT * C.G''','''XMASS = (PACKS - 1) * PW.M_PACK + float(os.environ.get("CEPH_XMASS", "0"))
if XMASS:
    S4.M_TORSO += XMASS; C.M_ROBOT += XMASS; C.W_ROBOT = C.M_ROBOT * C.G
# claw-prop variant (power4.PROPS): "std" 6 in on a 2207, "xl" 7.5 in on a 28xx (heavier pod, CEPH_PROP=xl)
PROP = os.environ.get("CEPH_PROP", "std")
PROP_P = PW.PROPS[PROP]
if PROP_P["dM_pod"]:
    C.M_POD += PROP_P["dM_pod"]; C.M_ROBOT += 8 * PROP_P["dM_pod"]; C.W_ROBOT = C.M_ROBOT * C.G
if PROP != "std":
    F_MAX = PW.claw_max(PROP, PW.V_REF)
# prop reaction torque about each claw axis (on by default for xl): the spin pattern balances 8-prop flight and the
# 4-prop quad mode on arms 0/2/4/6 alike
PROP_TORQUE = bool(int(os.environ.get("CEPH_PROPTORQUE", "1" if PROP != "std" else "0")))
SPIN = np.array([1, -1, -1, 1, 1, -1, -1, 1], float)''')
rep('''        self.Wd = np.diag(W_J)
        # senses''','''        self.Wd = np.diag(W_J)
        # controller gains (trainable; defaults = the hand-tuned steer4 / sim4 values)
        self.g = dict(kp_xy=KP, kd_xy=KD, ki_xy=KI, a_max=A_MAX, lean_max=LEAN_MAX, kp_psi=KP_PSI, kd_psi=KD_PSI,
                      kp_att=14.0, kd_att=4.5, ki_att=0.0, kp_z=4.0, kd_z=3.5, tau_scale=1.0, mz_alloc=1.0)
        self.iatt = np.zeros(2); self.payload = 0.0
        # senses''')
rep('''        self.I = np.clip(self.I + KI * e * dt, -1.0, 1.0)
        vel = d.qvel[0:2]
        acc = a + KP * e + KD * (v - vel) + self.I + 0.5 * RHO * CDA * np.linalg.norm(vel) * vel / C.M_ROBOT
        n = np.linalg.norm(acc)
        if n > A_MAX: acc *= A_MAX / n
        Fh = C.M_ROBOT * acc''','''        g_ = self.g
        self.I = np.clip(self.I + g_["ki_xy"] * e * dt, -1.0, 1.0)
        vel = d.qvel[0:2]
        acc = a + g_["kp_xy"] * e + g_["kd_xy"] * (v - vel) + self.I + 0.5 * RHO * CDA * np.linalg.norm(vel) * vel / C.M_ROBOT
        n = np.linalg.norm(acc)
        if n > g_["a_max"]: acc *= g_["a_max"] / n
        Fh = (C.M_ROBOT + self.payload) * acc''')
rep('''        Mz = float(np.clip(KP_PSI * wrap(yr - yaw) + KD_PSI * (vyr - wW[2]), -MZ_MAX, MZ_MAX))''','''        Mz = float(np.clip(g_["kp_psi"] * wrap(yr - yaw) + g_["kd_psi"] * (vyr - wW[2]), -MZ_MAX, MZ_MAX))
        self.Mz_dem = Mz''')
rep('''        lean = lean + (tan * Mz / max((f * rn * g).sum(), 1e-6)) * g[:, None]
        ln = np.linalg.norm(lean, axis=1)
        lean *= (np.minimum(ln, np.sin(LEAN_MAX)) / np.maximum(ln, 1e-9))[:, None]''','''        Mz_lean = Mz * (1 - g_["mz_alloc"]) if PROP_TORQUE else Mz     # with prop torque, part of yaw goes to differential thrust
        lean = lean + (tan * Mz_lean / max((f * rn * g).sum(), 1e-6)) * g[:, None]
        ln = np.linalg.norm(lean, axis=1)
        lean *= (np.minimum(ln, np.sin(g_["lean_max"])) / np.maximum(ln, 1e-9))[:, None]''')
rep('''            self.dq_air[i] += (tgt - Q_FLY - self.dq_air[i]) * np.minimum(1, dt / TAU_J)
            self.goal[i] = Q_FLY + self.dq_air[i]''','''            q_base = getattr(self, "q_base", None)
            qb = Q_FLY if q_base is None else q_base[i]
            self.dq_air[i] += (tgt - qb - self.dq_air[i]) * np.minimum(1, dt / (TAU_J * g_["tau_scale"]))
            self.goal[i] = qb + self.dq_air[i]''')
rep('''            Fa = C.M_ROBOT * (C.G + azr + 4.0 * (zr - pb[2]) + 3.5 * (vzr - d.qvel[2])) / max(ez[2], 0.7) - edf * ez[2]''','''            g_ = self.g
            Fa = (C.M_ROBOT + self.payload) * (C.G + azr + g_["kp_z"] * (zr - pb[2]) + g_["kd_z"] * (vzr - d.qvel[2])) / max(ez[2], 0.7) - edf * ez[2]''')
rep('''            kp, kd = 60.0 + (14.0 - 60.0) * air, 8.0 + (4.5 - 8.0) * air
            Mx = -kp * (roll_err + ty) - kd * wW[0]
            My = -kp * (pitch_err - tx) - kd * wW[1]''','''            kp, kd = 60.0 + (g_["kp_att"] - 60.0) * air, 8.0 + (g_["kd_att"] - 8.0) * air
            Mx = -kp * (roll_err + ty) - kd * wW[0]
            My = -kp * (pitch_err - tx) - kd * wW[1]
            if g_["ki_att"] > 0 and air > 0.99:      # trims a steady tilt from off-centre payloads (frozen while saturated)
                if self.sat == 0:
                    self.iatt = np.clip(self.iatt + g_["ki_att"] * np.array([roll_err + ty, pitch_err - tx]) * DT, -3.0, 3.0)
                Mx -= self.iatt[0]; My -= self.iatt[1]''')
rep('''            A = np.zeros((3, len(on))); fmax = self.fmax_eff * wgt[on]
            for j, i in enumerate(on):
                ax = d.xmat[self.pod[i]].reshape(3, 3)[:, 0]; r = d.xpos[self.pod[i]] + ax * C.BLADE_OFF - com
                mo = np.cross(r, ax); A[:, j] = [ax[2], mo[0], mo[1]]
            F = self.allocate(A, np.array([F_need, Mx - M_edf[0], My - M_edf[1]]), wgt[on], fmax)''','''            nr = 4 if PROP_TORQUE else 3
            A = np.zeros((nr, len(on))); fmax = self.fmax_eff * wgt[on]
            for j, i in enumerate(on):
                ax = d.xmat[self.pod[i]].reshape(3, 3)[:, 0]; r = d.xpos[self.pod[i]] + ax * C.BLADE_OFF - com
                mo = np.cross(r, ax) - (PROP_P["q_per_T"] * SPIN[i] * ax if PROP_TORQUE else 0)
                A[:, j] = [ax[2], mo[0], mo[1], mo[2]][:nr]
            b = [F_need, Mx - M_edf[0], My - M_edf[1]]
            if PROP_TORQUE: b.append(getattr(self, "Mz_dem", 0.0) * self.g["mz_alloc"])
            F = self.allocate_n(A, np.array(b), wgt[on], fmax)''')
rep('''        for i in range(8):
            if self.fan[i] > 1e-3:
                R = d.xmat[self.pod[i]].reshape(3, 3); f = self.fan[i] * R[:, 0]
                d.xfrc_applied[self.pod[i], :3] = f
                d.xfrc_applied[self.pod[i], 3:] = np.cross(R[:, 0] * C.BLADE_OFF + d.xpos[self.pod[i]] - d.xipos[self.pod[i]], f)''','''        for i in range(8):
            if self.fan[i] > 1e-3:
                R = d.xmat[self.pod[i]].reshape(3, 3); f = self.fan[i] * R[:, 0]
                d.xfrc_applied[self.pod[i], :3] = f
                d.xfrc_applied[self.pod[i], 3:] = np.cross(R[:, 0] * C.BLADE_OFF + d.xpos[self.pod[i]] - d.xipos[self.pod[i]], f)
                if PROP_TORQUE:                      # rotor reaction torque on the pod, opposite to the spin
                    d.xfrc_applied[self.pod[i], 3:] -= PROP_P["q_per_T"] * SPIN[i] * self.fan[i] * R[:, 0]''')
rep('''    # ---------------------------------------------------------------- reflexes''','''    @staticmethod
    def allocate_n(A, b, w, fmax):
        """sim4.allocate (attitude first, collective gives way) generalised to any number of moment rows."""
        Wd = np.diag(np.maximum(w, 1e-3))
        P = Wd @ np.linalg.pinv(A @ Wd)
        e1 = np.zeros(len(b)); e1[0] = 1.0
        bm = b.copy(); bm[0] = 0.0
        u = P @ e1; Fm = P @ bm
        def c_range(al):
            lo, hi = -np.inf, np.inf
            for ui, fi, mx in zip(u, al * Fm, fmax):
                if abs(ui) < 1e-9:
                    if fi < -1e-9 or fi > mx + 1e-9: return None
                    continue
                a1, a2 = (0 - fi) / ui, (mx - fi) / ui
                lo, hi = max(lo, min(a1, a2)), min(hi, max(a1, a2))
            return (lo, hi) if lo <= hi else None
        al, r = 1.0, c_range(1.0)
        if r is None:
            a_lo, a_hi = 0.0, 1.0
            for _ in range(20):
                mm = 0.5 * (a_lo + a_hi)
                if c_range(mm) is None: a_hi = mm
                else: a_lo = mm
            al, r = a_lo, c_range(a_lo)
        if r is None:
            return np.clip(b[0] * u, 0, fmax)
        c = float(np.clip(b[0], r[0], r[1]))
        return np.clip(c * u + al * Fm, 0, fmax)

    # ---------------------------------------------------------------- reflexes''')
rep('''                    self.bat.step(self.edf, self.fan, self.load, CTRL_DT)
                    self.fmax_eff = F_MAX * self.bat.claw_scale(); self.edf_cap = min(C.EDF_MAX, self.bat.edf_max())''','''                    self.bat.step(self.edf, self.fan, self.load, CTRL_DT, k_claw=PROP_P["K"])
                    self.fmax_eff = F_MAX * self.bat.claw_scale() if PROP == "std" else PW.claw_max(PROP, self.bat.V)
                    self.edf_cap = min(C.EDF_MAX, self.bat.edf_max())''')
open('body4.py','w').write(s)
p=open('power4.py').read()
a='''    def step(self, edf_thrust, fan_thrusts, servo_loads, dt):
        P = EDF_K * max(edf_thrust, 0) ** 1.5 + CLAW_K * np.sum(np.clip(fan_thrusts, 0, None) ** 1.5)'''
if a in p:
    p=p.replace(a,'''    def step(self, edf_thrust, fan_thrusts, servo_loads, dt, k_claw=CLAW_K):
        P = EDF_K * max(edf_thrust, 0) ** 1.5 + k_claw * np.sum(np.clip(fan_thrusts, 0, None) ** 1.5)''')
open('power4.py','w').write(p)
print("patched")
