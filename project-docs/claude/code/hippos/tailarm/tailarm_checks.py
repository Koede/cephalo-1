"""Checks for the arm tail (tailarm.py).  python3 tailarm_checks.py -> tailarm_checks.json (about 5 min)
1 masses; 2 joint ranges against the rump / hind legs (straight arm, envelope r 31.5 + hand3); 3 named poses solved with
the same IK as the octopus arms (position / axis targets + clearance to the horse + a penalty on static servo torque
above 80 % of rated); 4 servo loads, bending through the SFF seat, lug margin; 5 current through the SFF (12 V + AUX);
6 kickstand standing and reared 30 / 45 deg (hand3 heel on the floor, 10 % of Centauri-1 on it).
Horse + octopus = their render surfaces (scene2_notail.npz from `NO_TAIL=1 render_scene2.py`) sampled into a KD-tree
(arm_scale_study's shell distance ignores everything behind the rump); the final poses are mesh-checked with FCL in
tailarm_meshcheck.py.  Kickstand: a planar grid (yaw joints 0) and the full IK, best of the two.  Joint limits: the arm's link shells allow +-100 deg; the octopus j0 / j1 limits came from
its mantle and neighbours, so here j0 / j1 are limited by the horse only (checked)."""
import sys, io, contextlib, json
sys.path[:0] = ["/home/claude/octodrone/hippos/tailarm", "/home/claude/octodrone/hippos", "/home/claude/octodrone/ceph2", "/home/claude/octodrone",
                "/home/claude/octodrone/centaur", "/home/claude/octodrone/centaur/legs_v2", "/home/claude/octodrone/centaur/arms_scale"]
import numpy as np
from scipy.optimize import least_squares
with contextlib.redirect_stdout(io.StringIO()):
    import chassis_v3 as C3, arm_scale_study as AS
import cephalo4 as C, tailarm as TA

S = C3.S; Hh = AS.horse(S)
G = 9.81
M_CONN = np.array([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])             # connector -> horse (tail mount)
Z_SEAT_H = 0.60 * S
X_RUMP = float(C3.hits(np.array([0.0, Z_SEAT_H]), np.array([1.0, 0.0]))[0])
ORG = np.array([X_RUMP, 0.0, Z_SEAT_H])
SEAT_P = ORG + M_CONN @ np.array([0, 0, TA.SEAT]) * 1e-3
HIP_R = np.array([C3.HIPR[0], 0.0, C3.HIPR[1]])                         # rear hip (horse x, z)
M_CENT = 30.8 + 2.0 + 0.6                                                # Centauri-1: 30.8 + hand3 on all eight arms + the heavier tail (est.)
LIM = np.radians([[-100, 100]] * 8)
RATED, STALL = C.RATED, C.STALL
AMP = {"STS3250": (0.28, 4.2, 4.90), "STS3215": (0.18, 2.7, 2.94)}      # no-load A, stall A, stall N m at 12 V (est. line)


def body_pose(phi_deg):
    """Horse pitched nose-up by phi about the rear hip: world = Rb (h - hip) + hip."""
    a = np.radians(phi_deg); Rb = np.array([[np.cos(a), 0, -np.sin(a)], [0, 1, 0], [np.sin(a), 0, np.cos(a)]])
    return Rb


def root(phi=0.0):
    R0, p0 = TA.frames(ORG, M_CONN)
    Rb = body_pose(phi)
    return Rb @ R0, Rb @ (p0 - HIP_R) + HIP_R, Rb


def _surface_tree():
    """Horse (preview shell, trim, covers, legs, actuators) + Cephalo-2 meshes from scene2.npz (NO_TAIL build), sampled
    densely (about 3 mm) into a KD-tree: unsigned distance to the nearest surface.  The tail mount meshes are left out."""
    import trimesh
    from scipy.spatial import cKDTree
    D = np.load("/home/claude/octodrone/centaur/scene2_notail.npz")
    tris = [D[k].reshape(-1, 3, 3) for k in D.files if k.startswith("g") and not k.endswith("_meta")]
    oi = sorted(int(k[1:-2]) for k in D.files if k.startswith("o") and k.endswith("_v"))
    for i in oi[:-4]:
        tris.append(D[f"o{i}_v"][D[f"o{i}_f"]])
    T = np.concatenate(tris).astype(float)
    m = trimesh.Trimesh(T.reshape(-1, 3), np.arange(len(T) * 3).reshape(-1, 3), process=False)
    n = int(m.area / 6e-6)
    pts = trimesh.sample.sample_surface(m, min(n, 3_000_000), seed=1)[0]
    return cKDTree(np.vstack([pts, T.reshape(-1, 3)]))
TREE = _surface_tree()


def clear(q, R0, p0, Rb, hand_floor=True):
    """Smallest clearance of the arm + hand envelope to the horse / octopus surfaces (horse frame), and to the floor (world).
    hand_floor False: the hand3 envelope is left out of the floor check (kickstand: the hand stands on its heel)."""
    P, r = TA.capsules(q, R0, p0)
    Ph = (P - HIP_R) @ Rb + HIP_R                                         # back to the horse frame
    d = TREE.query(Ph)[0] - r
    zf = P[:, 2] - r
    if not hand_floor: zf = zf[:-5]
    return d, zf


def load(q, R0, p0, F=None, at=None):
    t, _ = TA.torques(q, R0, p0, F, at)
    return np.abs(t) / RATED, t


def solve(R0, p0, Rb, tgt=None, axis=None, F=None, floor=True, q0s=None, w_load=2.0, heel_floor=False):
    lo, hi = LIM.T
    def res(q):
        h, u = TA.heel(q, R0, p0)
        r = []
        if tgt is not None: r.append((h - tgt) * 10)
        if axis is not None: r.append((u - axis) * 1.0)
        d, z = clear(q, R0, p0, Rb, hand_floor=not heel_floor)
        r.append(np.minimum(d - 0.012, 0) * 20)
        if floor: r.append(np.minimum(z - (0.0 if heel_floor else 0.03), 0) * 20)
        L, _ = load(q, R0, p0, F, h if F is not None else None)
        r.append(w_load * np.maximum(L - 0.80, 0)); r.append(0.004 * q)
        return np.concatenate(r)
    best = None
    for q0 in (q0s or [np.zeros(8)]):
        s = least_squares(res, np.clip(q0, lo + 1e-6, hi - 1e-6), bounds=(lo, hi), max_nfev=600)
        h, u = TA.heel(s.x, R0, p0)
        L, _ = load(s.x, R0, p0, F, h if F is not None else None)
        err = 0.0 if tgt is None else np.linalg.norm(h - tgt)
        d, z = clear(s.x, R0, p0, Rb, hand_floor=not heel_floor)
        key = (err > 0.006 or d.min() < 0.010, L.max())
        if best is None or key < best[0]: best = (key, s.x, err, d.min(), L)
    return best[1], best[2], best[3], best[4]


def seat_moment(q, R0, p0, F=None, at=None):
    _, (P, Fp) = TA.torques(q, R0, p0, F, at)
    M = np.cross(P - SEAT_P, Fp).sum(0)
    M += np.cross(p0 + R0[:, 0] * 0.017 - SEAT_P, np.array([0, 0, -G * (0.0745 + 0.024 + 0.018)]))   # j0 servo, winch, root cradle
    return float(np.linalg.norm(M))


def amps(tau):
    return sum(AMP[n][0] + (AMP[n][1] - AMP[n][0]) * abs(t) / AMP[n][2] for n, t in zip(C.SERVO, tau)) + 0.5      # + hand3 servos holding (est.)


def report(name, q, R0, p0, Rb, F=None, at=None, extra=None):
    L, t = load(q, R0, p0, F, at)
    d, z = clear(q, R0, p0, Rb, hand_floor=F is None)
    h, u = TA.heel(q, R0, p0)
    Ms = seat_moment(q, R0, p0, F, at)
    out = dict(q_deg=np.degrees(q).round(1).tolist(), pct_rated=(100 * L).round(0).tolist(), worst_pct_rated=round(100 * L.max()),
               worst_pct_stall=round(100 * (np.abs(t) / STALL).max()), clear_horse_mm=round(1e3 * d.min(), 1), lowest_mm=round(1e3 * z.min(), 1),
               heel_mm=(1e3 * h).round(0).tolist(), hand_axis=u.round(2).tolist(), M_seat_Nm=round(Ms, 2),
               lug_factor=round(94.8 * 5.8 / (2 * Ms / (3 * 0.012)), 2), amps=round(amps(t), 2))
    if extra: out.update(extra)
    print(f"{name:34s} worst {out['worst_pct_rated']:4d} % rated ({out['worst_pct_stall']:3d} % stall)  clear {out['clear_horse_mm']:6.1f} mm  "
          f"low {out['lowest_mm']:6.0f}  seat {Ms:.2f} N m (lugs x{out['lug_factor']})  {out['amps']:.1f} A")
    return out


if __name__ == "__main__":
    res = {}
    # ---- 1 masses
    m_arm = sum(v for _, v in C.arm_mass_table()) if hasattr(C, "arm_mass_table") else None
    segs, tip = TA.fk(np.zeros(8), *TA.frames(ORG, M_CONN))
    P, m, s = C.arm_points(segs, tip, TA.POD_H3)
    rc = TA.root_cradle_arm().volume() * 1.27e-3 * 0.7 / 1e3
    mass = dict(arm_with_tipbay_and_line=round(float(m[:-1].sum()), 3), hand3_with_connector=TA.POD_H3[0], winch=0.024, root_cradle=round(rc, 4),
                mount_socket_base_adapter=0.050)
    mass["total"] = round(sum(mass.values()), 3); mass["old_tail_total"] = 0.712
    res["mass_kg"] = mass; print("mass", mass)
    R0, p0, Rb = root(0.0)
    # ---- 2 ranges: sweep j0 (pitch) and j1 (yaw) with the rest straight / with the rest curled up
    rng = {}
    for k in (0, 1):
        lim = {}
        for sg in (1, -1):
            free = 0
            for a in range(0, 101, 2):
                q = np.zeros(8); q[k] = np.radians(sg * a)
                d, z = clear(q, R0, p0, Rb)
                if d.min() < 0.0: break
                free = a
            lim["+" if sg > 0 else "-"] = free
        rng[f"j{k}"] = lim
    res["free_range_straight_deg"] = rng; print("ranges (straight arm vs horse):", rng)
    # ---- 3 named poses
    P_ = {}
    q_carry = np.radians([-45, 0, 25, 0, 25, 0, 25, 0])
    P_["reference: first render pose (curl, out back)"] = report("reference: first render pose", q_carry, R0, p0, Rb)
    tgt = np.array([X_RUMP - 0.10, 0.0, 0.90])
    q, e, d, L = solve(R0, p0, Rb, tgt=tgt, q0s=[np.radians([-80, 0, -10, 0, 30, 0, 40, 0]), np.radians([-60, 0, -30, 0, 0, 0, 60, 0]), np.radians([-90, 0, 0, 0, 20, 0, 20, 0])], w_load=4.0)
    P_["carried high (raised, curled)"] = report("carried high (raised, curled)", q, R0, p0, Rb, extra=dict(target_err_mm=round(1e3 * e, 1)))
    P_["held straight back, level"] = report("held straight back, level", np.zeros(8), R0, p0, Rb)
    # rest: the lowest-load pose with the heel at least 80 mm off the floor and clear of the horse
    q, e, d, L = solve(R0, p0, Rb, q0s=[np.radians([80, 0, -20, 0, -40, 0, -30, 0]), np.radians([60, 0, -40, 0, -40, 0, -20, 0]), np.radians([90, 0, -60, 0, -60, 0, 0, 0])], w_load=4.0)
    P_["rest (hanging, least load)"] = report("rest (hanging, least load)", q, R0, p0, Rb)
    # hand to the side: heel 0.30 m out to the left at rump height
    tgt = np.array([X_RUMP - 0.20, 0.30, 0.42])
    q, e, d, L = solve(R0, p0, Rb, tgt=tgt, q0s=[np.radians([20, 60, 0, 30, 0, 0, 0, 0]), np.radians([-20, 80, 20, 0, 20, 0, 0, 0])])
    P_["hand to the side (y 0.30)"] = report("hand to the side (y 0.30)", q, R0, p0, Rb, extra=dict(target_err_mm=round(1e3 * e, 1)))
    # over the back: hand above the croup, towards the octopus (hand-over point 0.25 m behind the octopus plate)
    tgt = np.array([Hh["oct_p"][0] - 0.40, 0.0, 0.74])
    q, e, d, L = solve(R0, p0, Rb, tgt=tgt, q0s=[np.radians([-90, 0, -60, 0, 40, 0, 40, 0]), np.radians([-100, 0, -30, 0, 30, 0, 60, 0]), np.radians([-80, 0, -80, 0, 30, 0, 30, 0])])
    P_["over the back to the octopus"] = report("over the back to the octopus", q, R0, p0, Rb, extra=dict(target_err_mm=round(1e3 * e, 1), target_mm=(1e3 * tgt).round(0).tolist()))
    # ---- 6 kickstand: heel on the floor, hand axis straight down, 10 % of Centauri-1 up through the heel.  Planar (yaw
    # joints 0): grid over j0 / j2 / j4, j6 sets the hand axis vertical; keep heel within 4 mm of the floor, then the
    # lowest worst-joint load that clears the horse / octopus by >= 12 mm and keeps the arm off the floor.
    Fk = np.array([0, 0, 0.10 * M_CENT * G])
    sj = C.S_J; SP = C.SPIGOT
    for phi in (0.0, 30.0, 45.0):
        R0p, p0p, Rbp = root(phi)
        x0, z0 = p0p[0], p0p[2]; th0 = np.arctan2(-R0p[2, 0], R0p[0, 0] * -1.0)          # arm x direction angle below the -x axis
        g0, g2, g4 = np.meshgrid(np.radians(np.arange(0, 101, 2)), np.radians(np.arange(-100, 101, 4)), np.radians(np.arange(-100, 101, 4)), indexing="ij")
        g0, g2, g4 = g0.ravel(), g2.ravel(), g4.ravel()
        base = np.arctan2(-R0p[2, 0], -R0p[0, 0])                                             # direction of arm x: angle below the -x axis
        g6 = np.pi / 2 - base - (g0 + g2 + g4)
        ok = np.abs(g6) <= np.radians(100)
        g0, g2, g4, g6 = g0[ok], g2[ok], g4[ok], g6[ok]
        ang = [base + g0, base + g0 + g2, base + g0 + g2 + g4, base + g0 + g2 + g4 + g6]       # segment directions (below -x)
        seg = [(0, sj[2]), (sj[2], sj[4]), (sj[4], sj[6]), (sj[6], SP + TA.HEEL)]
        hx = x0 + sum(-(b_ - a_) * np.cos(t) for (a_, b_), t in zip(seg, ang))
        hz = z0 + sum(-(b_ - a_) * np.sin(t) for (a_, b_), t in zip(seg, ang))
        cand = np.where(np.abs(hz) < 0.004)[0]
        best = None
        for c in cand:
            q = np.array([g0[c], 0, g2[c], 0, g4[c], 0, g6[c], 0])
            d, z = clear(q, R0p, p0p, Rbp, hand_floor=False)
            if d.min() < 0.012 or z.min() < 0.0: continue
            h, u = TA.heel(q, R0p, p0p)
            L, _ = load(q, R0p, p0p, Fk, h)
            if best is None or L.max() < best[0]: best = (L.max(), q, h)
        # the full IK (yaw joints free) for the same task, a few heel positions behind the rump; keep the better of the two
        xr = (Rbp @ (np.array([X_RUMP, 0, 0]) - HIP_R) + HIP_R)[0]
        for dx in ((-0.35, -0.45, -0.55) if phi > 0 else ()):
            q, e, dmin, L = solve(R0p, p0p, Rbp, tgt=np.array([xr + dx, 0.0, 0.0]), axis=np.array([0, 0, -1.0]), F=Fk, heel_floor=True,
                                  q0s=[np.radians([80, 0, 0, 0, 0, 0, 0, 0]), np.radians([40, 0, 30, 0, 20, 0, 0, 0]), np.radians([60, 0, -40, 0, 40, 0, 30, 0])])
            if e < 0.004 and dmin >= 0.012 and (best is None or L.max() < best[0]): best = (L.max(), q, TA.heel(q, R0p, p0p)[0])
        name = f"kickstand {'standing' if phi == 0 else f'reared {phi:.0f} deg'}"
        if best is None:
            P_[name] = dict(feasible=False); print(name, "no pose found"); continue
        q, h = best[1], best[2]
        xr = (Rbp @ (np.array([X_RUMP, 0, 0]) - HIP_R) + HIP_R)[0]
        P_[name] = report(name, q, R0p, p0p, Rbp, F=Fk, at=h, extra=dict(heel_behind_rump_mm=round(1e3 * (h[0] - xr)), F_heel_N=round(Fk[2], 1),
                                                                         candidates=int(len(cand))))
    res["poses"] = P_
    res["current"] = dict(pins_12V="12 V + AUX as a second 12 V feed (rump switches AUX on after reading the tail ID)", limit_A=10.4,
                          stall_all_servos_A=round(4.2 + 7 * 2.7, 1), firmware="STS protection current per servo so the sum stays <= 10.4 A")
    json.dump(res, open("tailarm_checks.json", "w"), indent=1, default=float)
