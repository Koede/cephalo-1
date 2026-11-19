"""Linear-elastic FEA of the hand/arm coupler rev B.2 (handmount_b.py) in the wheel-hand load cases (2026-11-18).
Checks the hand formulas of handmount_b.stresses().  python3 coupler_fea.py [maxvol ...] -> coupler_fea.json, coupler_fea.png

Model (mm, N, MPa):
  * three bodies, quadratic tets (scikit-fem P2, tetgen meshes of the CAD solids): hand base (with lugs), locking collar
    at 0 deg (locked), nose ring.  PA-CF isotropic E 3500 MPa, nu 0.35 (est.).
  * the socket body + tip bay are rigid: the nose ring's top face (r <= R_body, clamped against the socket by the screws)
    and its three screw bores are fixed.
  * contacts are frictionless, compression-only (penalty, active set), closed with no gap (the 0.1 mm axial play and the
    radial fits are taken as closed on the loaded side; the lip's lead-in ramp keeps its own extra gap):
      hand top face (flange + lugs, z 0)  <-> nose ring underside        (hand pushed into the arm)
      collar lip tops                    <-> hand lug undersides        (hand pulled out)
      collar step underside (z 3.5)      <-> nose ring top               (collar pulled toward the hand)
      hand spigot (r 10.35)              <-> rigid socket bore, radial
      collar bore above the step (r 20.5) <-> rigid socket body, radial
  * load on the hand base's bottom face (where a hand bolts on): the case's radial force as a uniform shear plus the
    case's bending moment as a linear normal traction.  Direction swept round the axis (lugs at 0/120/240 deg).
  * twist (1.7 N m) is not applied: the key carries it; the hand formulas leave it out too.
"""
import os, sys, json, time
import numpy as np
import scipy.sparse as sp
import tetgen
from skfem import MeshTet, Basis, ElementTetP2, ElementVector, asm
from skfem.models.elasticity import linear_elasticity, lame_parameters
import handmount_b as HB

P = HB.P
P["arc_n"] = 4                               # coarser fillet arcs for the FEA mesh (CAD: 8)
E, NU = 3500.0, 0.35
LAM, MU = lame_parameters(E, NU)
REV = os.environ.get("REV", "B.4")
CASE = "3 g landing on the wheels (est.)"
LOAD = json.load(open("wheelhand_sizing.json"))["connector"][CASE]
TOL = 0.02


# ---------------------------------------------------------------- meshing
NSEG = 64                                   # circle segments for the FEA solids (the CAD uses 120)
_cyl, _rev = HB.cyl, HB.rev
HB.cyl = lambda r, z0, z1, n=NSEG: _cyl(r, z0, z1, int(np.clip(2 * np.pi * r / 1.2, 12, NSEG)))     # ~1.2 mm facets, 12..NSEG
HB.rev = lambda profile, a0, a1, n=NSEG: _rev(profile, a0, a1, max(4, int(round(max(NSEG, min(n, 3 * NSEG)) * abs(a1 - a0) / 360))))


def collar_fea():
    """collar(0) as in handmount_b, without the grip flutes and detent dimples (outer radius = the wall at a flute,
    cb + t_wall) and with the real stepped lip top (B.4: lead-in ramp and trailing relief; B.2 runs had it flat)."""
    z0, z1 = P["collar_z"]; zl = P["lip_z"][1]; zs = P["z_nose"]; zt = zs + P["t_step"]; cb = P["collar_cb"]
    body = HB.cyl(cb + P["t_wall"], z0, zt) + HB.cyl(P["sleeve_r"], zt, z1) - HB.cyl(P["collar_bore"], zs, z1 + 1) - HB.cyl(cb, z0 - 1, zs)
    for k in range(3):
        body += HB.lip(P["lug_az"][k], nst=16)                       # B.4: the real stepped lip (lead-in ramp + trailing relief)
    return body


def hand_fea():
    """hand_base() load path: flange, lugs, spigot, key rib, wire bore, pad well, the three bullet bores.  Lead grooves,
    the wire pocket and the four insert holes in the bottom face are left out (they are under the loaded face)."""
    fz0, fz1 = P["z_flange"]
    m = HB.cyl(P["R_flange"], fz0, fz1) + HB.cyl(P["r_spigot"], fz1 - 0.5, P["z_spigot_top"])
    for az in P["lug_az"]:
        m += HB.lug(az)
    m += HB.sector(P["r_spigot"] - 0.5, P["r_bore"] + P["key_h"], P["az_key"] - P["key_w"] / 2 / P["r_bore"] * 57.3, P["az_key"] + P["key_w"] / 2 / P["r_bore"] * 57.3, fz1 - 0.5, P["z_bore"] - 1.0)
    m -= HB.cyl(P["r_pad"] + 0.15, P["z_pad"] - P["pad_t"], P["z_spigot_top"] + 1) + HB.cyl(3.5, fz0 - 1, P["z_pad"])
    for az in P["az_bullet"]:
        x, y = HB.at(P["r_bullet"], az); m -= HB.cyl(2.05, fz0 - 1, fz1 + 1).translate([x, y, 0])
    return m


def tet_mesh(man, maxvol):
    import pymeshfix
    man = man.simplify(1e-3)                                     # drops the booleans' zero-area triangles (else meshfix eats patches)
    me = man.to_mesh(); V = np.asarray(me.vert_properties)[:, :3].astype(float); F = np.asarray(me.tri_verts).astype(np.int32)
    fx = pymeshfix.MeshFix(V, F); fx.repair()                    # tetgen rejects the boolean's near-coplanar slivers
    tg = tetgen.TetGen(np.asarray(fx.points, float), np.asarray(fx.faces, np.int32))
    res = tg.tetrahedralize(switches=f"pq2.0a{maxvol}Q"); nodes, elem = np.asarray(res[0], float), np.asarray(res[1], int)
    a, b, c, d = (nodes[elem[:, k]] for k in range(4))
    vol = np.einsum("ij,ij->i", np.cross(b - a, c - a), d - a) / 6
    neg = vol < 0
    elem[neg, 0], elem[neg, 1] = elem[neg, 1].copy(), elem[neg, 0].copy()
    err = abs(np.abs(vol).sum() - man.volume()) / man.volume()
    assert err < 0.01, f"mesh volume off by {100 * err:.2f} %"
    return MeshTet(nodes.T, elem.T)


class Body:
    def __init__(self, name, man, maxvol, off):
        t0 = time.time()
        self.name = name; self.m = tet_mesh(man, maxvol)
        self.b = Basis(self.m, ElementVector(ElementTetP2()))
        self.K = asm(linear_elasticity(LAM, MU), self.b)
        self.off = off; self.n = self.b.N
        # boundary facets: vertices, centroid, outward unit normal, area
        bf = self.m.boundary_facets(); tri = self.m.facets[:, bf].T; X = self.m.p.T
        A, B, C = X[tri[:, 0]], X[tri[:, 1]], X[tri[:, 2]]
        nrm = np.cross(B - A, C - A); area = 0.5 * np.linalg.norm(nrm, axis=1); nrm /= (2 * area)[:, None]
        cen = (A + B + C) / 3
        el = self.m.f2t[0, bf]; ec = X[self.m.t[:, el]].mean(axis=0) if False else self.m.p[:, self.m.t[:, el]].mean(axis=1).T
        flip = np.einsum("ij,ij->i", nrm, cen - ec) < 0; nrm[flip] *= -1
        self.bf, self.tri, self.cen, self.nrm, self.area = bf, tri, cen, nrm, area
        print(f"  {name}: {self.m.t.shape[1]} tets, {self.n} dofs ({time.time() - t0:.1f} s)", flush=True)

    def vdof(self, i):                       # global dofs of vertex node(s) i: (len, 3)
        return self.b.nodal_dofs[:, i].T + self.off

    def facets(self, pred):
        return np.where(pred(self.cen, self.nrm))[0]

    def slaves(self, fi):
        """vertex nodes of the facets fi and their tributary areas."""
        nodes = self.tri[fi].ravel(); w = np.repeat(self.area[fi] / 3, 3)
        u, inv = np.unique(nodes, return_inverse=True); a = np.bincount(inv, w)
        return u, a


r_of = lambda X: np.hypot(X[:, 0], X[:, 1])


def pair_planar(S, fs, M, fm, normal_z, g0_fun=None):
    """Slave vertex nodes of S's facets fs projected along z onto M's facets fm (barycentric).  Returns rows for the
    penalty: (slave dofs z, master dofs z (3), weights (3), area, g0, sign)."""
    sn, sa = S.slaves(fs); Xs = S.m.p[:, sn].T
    Tm = M.tri[fm]; Xm = M.m.p.T
    from scipy.spatial import cKDTree
    cm = Xm[Tm].mean(axis=1)[:, :2]; kd = cKDTree(cm)
    out = []
    _, cand = kd.query(Xs[:, :2], k=min(12, len(cm)))
    for s, (x, c) in enumerate(zip(Xs, cand)):
        for k in np.atleast_1d(c):
            a, b, cc = Xm[Tm[k]]
            v0, v1, v2 = b[:2] - a[:2], cc[:2] - a[:2], x[:2] - a[:2]
            d = v0[0] * v1[1] - v0[1] * v1[0]
            if abs(d) < 1e-12: continue
            l1 = (v2[0] * v1[1] - v2[1] * v1[0]) / d; l2 = (v0[0] * v2[1] - v0[1] * v2[0]) / d; l0 = 1 - l1 - l2
            if min(l0, l1, l2) >= -1e-6:
                zm = l0 * a[2] + l1 * b[2] + l2 * cc[2]
                g0 = 0.0 if g0_fun is None else g0_fun(x[2], zm)
                out.append((S.vdof(sn[s])[2], M.vdof(Tm[k])[:, 2], np.array([l0, l1, l2]), sa[s], g0))
                break
    return out, len(sn)


def asm_penalty(rows, ntot, signs, kpen, active):
    """penetration p = sign * (u_s - sum w u_m) - g0 > 0 means compression.  K += k a B^T B for active rows."""
    I, J, V = [], [], []; f = np.zeros(ntot)
    for r, (ds, dm, w, a, g0) in enumerate(rows):
        if not active[r]: continue
        idx = np.r_[ds, dm]; c = signs[r] * np.r_[1.0, -w]; k = kpen * a
        I.append(np.repeat(idx, len(idx))); J.append(np.tile(idx, len(idx))); V.append(k * np.outer(c, c).ravel())
        f[idx] += k * g0 * c                    # gap: force only once the gap is closed
    if not I: return sp.csr_matrix((ntot, ntot)), f
    return sp.csr_matrix((np.concatenate(V), (np.concatenate(I), np.concatenate(J))), shape=(ntot, ntot)), f


def pen(rows, u, signs):
    return np.array([s_ * (u[ds] - w @ u[dm]) - g0 for (ds, dm, w, a, g0), s_ in zip(rows, signs)])


def radial_rows(S, fs, sign):
    """slave vertex nodes on a cylinder against a rigid surface: p = sign * n_r . u."""
    sn, sa = S.slaves(fs); X = S.m.p[:, sn].T; n = np.c_[X[:, 0], X[:, 1], 0 * X[:, 0]] / r_of(X)[:, None]
    return [(S.vdof(i), n[k], sa[k]) for k, i in enumerate(sn)]


def asm_radial(rows, ntot, sign, kpen, active):
    I, J, V = [], [], []
    for r, (d, n, a) in enumerate(rows):
        if not active[r]: continue
        c = sign * n; k = kpen * a
        I.append(np.repeat(d, 3)); J.append(np.tile(d, 3)); V.append(k * np.outer(c, c).ravel())
    if not I: return sp.csr_matrix((ntot, ntot))
    return sp.csr_matrix((np.concatenate(V), (np.concatenate(I), np.concatenate(J))), shape=(ntot, ntot))


def pen_r(rows, u, sign):
    return np.array([sign * (n @ u[d]) for d, n, a in rows])


# ---------------------------------------------------------------- model
def build(maxvol):
    _ML.clear()
    print(f"mesh maxvol {maxvol} mm3", flush=True)
    H = Body("hand base", hand_fea(), maxvol, 0)
    Cl = Body("collar", collar_fea(), maxvol, H.n)
    N = Body("nose ring", HB.nose_ring(), maxvol, H.n + Cl.n)
    ntot = H.n + Cl.n + N.n
    K = sp.block_diag([H.K, Cl.K, N.K]).tocsr()
    zl0, zl1 = P["lip_z"]; zs = P["z_nose"]; cb = P["collar_cb"]
    zc = lambda z0: (lambda c, n: (np.abs(c[:, 2] - z0) < TOL))
    # contacts are paired both ways (each face's nodes against the other face), each pass with half the area
    def two(S, fs_pred, Mb, fm_pred, sign_first, g0=None):
        a, na = pair_planar(S, S.facets(fs_pred), Mb, Mb.facets(fm_pred), 1, g0_fun=g0)
        b, nb = pair_planar(Mb, Mb.facets(fm_pred), S, S.facets(fs_pred), 1, g0_fun=(None if g0 is None else (lambda zs_, zm: g0(zm, zs_))))
        rows = [(ds, dm, w, ar / 2, gg) for ds, dm, w, ar, gg in a] + [(ds, dm, w, ar / 2, gg) for ds, dm, w, ar, gg in b]
        sgn = np.r_[np.full(len(a), sign_first), np.full(len(b), -sign_first)]
        return rows, sgn, (len(a), na, len(b), nb)
    zlug = P["lug_z"][0]
    # contact 1: hand top (z 0, normal +z) vs nose ring bottom (z 0, normal -z); penetration = (u_hand - u_nose).z
    c1, s1, i1 = two(H, lambda c, n: (np.abs(c[:, 2]) < TOL) & (n[:, 2] > 0.9), N, lambda c, n: (np.abs(c[:, 2]) < TOL) & (n[:, 2] < -0.9), 1)
    # contact 2: collar lip tops vs hand lug undersides; penetration = (u_collar - u_hand).z - gap (the 0.1 mm play taken as closed)
    c2, s2, i2 = two(Cl, lambda c, n: (n[:, 2] > 0.95) & (c[:, 2] > zl0 + 0.05) & (c[:, 2] < zl1 + TOL) & (r_of(c) < cb + 0.3),
                     H, lambda c, n: (np.abs(c[:, 2] - zlug) < TOL) & (n[:, 2] < -0.9), 1,
                     g0=lambda z_lip, z_lug: max(z_lug - z_lip - P["ax_clr"], 0.0))
    # contact 3: collar step underside (z 3.5) vs nose ring top; penetration = (u_nose - u_collar).z
    c3, s3, i3 = two(Cl, lambda c, n: (np.abs(c[:, 2] - zs) < TOL) & (n[:, 2] < -0.9), N, lambda c, n: (np.abs(c[:, 2] - zs) < TOL) & (n[:, 2] > 0.9), -1)
    # radial: spigot (outward) and collar bore above the step (inward)
    rs = radial_rows(H, H.facets(lambda c, n: (np.abs(r_of(c) - P["r_spigot"]) < 0.05) & (np.abs(n[:, 2]) < 0.1) & (c[:, 2] > 0.2)), 1)
    rc = radial_rows(Cl, Cl.facets(lambda c, n: (np.abs(r_of(c) - P["collar_bore"]) < 0.05) & (np.abs(n[:, 2]) < 0.1) & (c[:, 2] > zs + 0.1)), -1)
    print(f"  contact pairs (found/slaves, both ways): hand-nose {i1}, lip-lug {i2}, step-nose {i3}, spigot {len(rs)}, collar bore {len(rc)}", flush=True)
    # fixed: nose ring top r <= R_body, screw bores
    fx = N.facets(lambda c, n: (np.abs(c[:, 2] - zs) < TOL) & (n[:, 2] > 0.9) & (r_of(c) <= P["R_body"] + 0.05))
    scr = np.array([HB.at(r, az) for r, az in P["screws"]])
    fx2 = N.facets(lambda c, n: (np.min(np.linalg.norm(c[:, None, :2] - scr[None], axis=2), axis=1) < P["screw_d"] / 2 + 0.05) & (np.abs(n[:, 2]) < 0.2))
    fixed = N.b.get_dofs(N.bf[np.r_[fx, fx2]]).all() + N.off
    M = dict(H=H, Cl=Cl, N=N, K=K, ntot=ntot)
    M["Bnull"] = rigid_modes(M)
    return dict(M, H=H, Cl=Cl, N=N, K=K, ntot=ntot, c1=c1, c2=c2, c3=c3, s1=s1, s2=s2, s3=s3, rs=rs, rc=rc, fixed=np.unique(fixed))


def load_vector(M, az_deg):
    """radial force + bending moment on the hand base's bottom face; force along direction az."""
    H = M["H"]; z0 = P["z_flange"][0]
    fb = H.facets(lambda c, n: (np.abs(c[:, 2] - z0) < TOL) & (n[:, 2] < -0.9))
    F, Mb = LOAD["radial_N"], LOAD["bending_Nm"] * 1e3
    d = np.array([np.cos(np.radians(az_deg)), np.sin(np.radians(az_deg)), 0.0])
    cen = H.cen[fb]; a = H.area[fb]; A = a.sum(); xc = (cen * a[:, None]).sum(0) / A
    x = (cen - xc) @ d; I = (a * x * x).sum()
    lever = Mb / F
    f = np.zeros(M["ntot"])
    # P2 facet: consistent load for a linear traction is close to lumping on the 3 vertices of each facet (fine mesh)
    for k, fi in enumerate(fb):
        t = F / A * d + np.array([0, 0, lever * F * x[k] / I])
        for v in H.tri[fi]: f[H.vdof(v)] += t * a[k] / 3
    return f, dict(face_area=A, I=I, lever_mm=lever)


_ML = {}


def amg_solve_gpu(A, b, B, x0=None, tol=1e-9):
    """Same solve on an NVIDIA GPU (CuPy): the smoothed-aggregation hierarchy is still built once on the CPU by pyamg, then
    copied to the GPU; the V-cycle uses damped Jacobi smoothing (2 sweeps each way, symmetric, so PCG stays valid) and a dense
    coarse solve, and the finest level always uses the current matrix A.  Set FEA_GPU=1 to use it."""
    import pyamg, cupy as cp, cupyx.scipy.sparse as cs
    key = ("gpu", A.shape[0])
    if key not in _ML:
        _ML.clear()
        ml = pyamg.smoothed_aggregation_solver(A.tobsr(blocksize=(3, 3)), B=B, symmetry="symmetric", strength=("symmetric", {"theta": 0.0}),
                                               smooth=("jacobi", {"omega": 4.0 / 3.0}), max_coarse=500)
        lv = []
        for L in ml.levels[:-1]:
            lv.append(dict(A=cs.csr_matrix(L.A.tocsr()), P=cs.csr_matrix(L.P.tocsr()), R=cs.csr_matrix(L.R.tocsr())))
        Ac = ml.levels[-1].A.toarray()
        _ML[key] = dict(lv=lv, Ac_inv=cp.asarray(np.linalg.pinv(Ac)))
        print(f"      gpu hierarchy: {len(lv) + 1} levels, coarse {Ac.shape[0]}", flush=True)
    H = _ML[key]; lv = H["lv"]
    lv[0]["A"] = cs.csr_matrix(A.tocsr())                                         # finest level: the current contact state
    for L in lv:                                                                 # Jacobi weight from the spectral radius of D^-1 A
        if "dinv" not in L or L is lv[0]:
            d = L["A"].diagonal(); L["dinv"] = 1.0 / d
            v = cp.random.default_rng(1).standard_normal(L["A"].shape[0])
            for _ in range(20):
                v = L["dinv"] * (L["A"] @ v); lam = float(cp.linalg.norm(v)); v /= lam
            L["w"] = 1.0 / (1.1 * lam) * 4.0 / 3.0 * 0.75
    def vcycle(k, r):
        if k == len(lv):
            return H["Ac_inv"] @ r
        L = lv[k]; x = L["w"] * L["dinv"] * r
        for _ in range(1):
            x += L["w"] * L["dinv"] * (r - L["A"] @ x)
        rc = L["R"] @ (r - L["A"] @ x); x += L["P"] @ vcycle(k + 1, rc)
        for _ in range(2):
            x += L["w"] * L["dinv"] * (r - L["A"] @ x)
        return x
    Ag = lv[0]["A"]; bg = cp.asarray(b); x = cp.asarray(x0) if x0 is not None else cp.zeros_like(bg)
    r = bg - Ag @ x; z = vcycle(0, r); p = z.copy(); rz = float(r @ z); nb = float(cp.linalg.norm(bg)); it = 0
    while it < 6000:
        q = Ag @ p; a = rz / float(p @ q); x += a * p; r -= a * q; it += 1
        if float(cp.linalg.norm(r)) < tol * nb: break
        z = vcycle(0, r); rz2 = float(r @ z); p = z + (rz2 / rz) * p; rz = rz2
    xh = cp.asnumpy(x); res = np.linalg.norm(A @ xh - b) / np.linalg.norm(b)
    print(f"      gpu cg {it} it, rel. residual {res:.1e}", flush=True)
    assert res < 1e-6, "solver did not converge"
    return xh


def amg_solve(A, b, B, x0=None, tol=1e-9):
    """CG with a smoothed-aggregation AMG preconditioner; the hierarchy is built once per model (first contact state)
    and reused while the active set changes (it only has to be a good preconditioner, the CG solves the true system)."""
    if os.environ.get("FEA_GPU"):
        return amg_solve_gpu(A, b, B, x0, tol)
    import pyamg
    from scipy.sparse.linalg import cg
    key = A.shape[0]
    if key not in _ML:
        _ML.clear(); _ML[key] = pyamg.smoothed_aggregation_solver(A.tobsr(blocksize=(3, 3)), B=B, symmetry="symmetric", strength=("symmetric", {"theta": 0.0}),
                                                                  smooth=("jacobi", {"omega": 4.0 / 3.0}), max_coarse=500)
    ml = _ML[key]
    it = [0]
    x, info = cg(A, b, x0=x0, rtol=tol, maxiter=3000, M=ml.aspreconditioner(cycle="V"), callback=lambda xk: it.__setitem__(0, it[0] + 1))
    r = np.linalg.norm(A @ x - b) / np.linalg.norm(b)
    print(f"      cg {it[0]} it, rel. residual {r:.1e}", flush=True)
    assert r < 1e-6, "solver did not converge"
    return x


def rigid_modes(M):
    """near-null space for AMG: 6 rigid modes per body (dofs interleaved x, y, z)."""
    out = np.zeros((M["ntot"], 6 * 3)); k = 0
    for key in ("H", "Cl", "N"):
        B = M[key]; X = B.b.doflocs.T; c = np.arange(B.n) % 3; o = B.off; sl = slice(o, o + B.n)
        for j in range(3): out[sl, k + j] = (c == j)
        x, y, z = X[:, 0], X[:, 1], X[:, 2]
        out[sl, k + 3] = np.where(c == 0, -y, np.where(c == 1, x, 0)); out[sl, k + 4] = np.where(c == 1, -z, np.where(c == 2, y, 0))
        out[sl, k + 5] = np.where(c == 2, -x, np.where(c == 0, z, 0)); k += 6
    return out


def solve(M, f, kpen):
    K0 = M["K"]; ntot = M["ntot"]; fixed = M["fixed"]
    soft = sp.diags(np.r_[np.full(M["H"].n + M["Cl"].n, 1e-4), np.zeros(M["N"].n)])        # removes the collar / hand spin about z (N/mm per dof)
    act = {k: np.ones(len(M[k]), bool) for k in ("c1", "c2", "c3", "rs", "rc")}
    sg = {"c1": M["s1"], "c2": M["s2"], "c3": M["s3"]}
    free = np.setdiff1d(np.arange(ntot), fixed); u0 = None
    for it in range(25):
        Kc = K0 + soft; fc = f.copy()
        for k in ("c1", "c2", "c3"):
            Kp, fp = asm_penalty(M[k], ntot, sg[k], kpen, act[k]); Kc = Kc + Kp; fc = fc + fp
        Kc = Kc + asm_radial(M["rs"], ntot, 1, kpen, act["rs"]) + asm_radial(M["rc"], ntot, -1, kpen, act["rc"])
        u = np.zeros(ntot)
        A = Kc[free][:, free].tocsr()
        u[free] = amg_solve(A, fc[free], M["Bnull"][free], u0[free] if u0 is not None else None)
        u0 = u.copy()
        changed = 0
        for k in ("c1", "c2", "c3"):
            p = pen(M[k], u, sg[k]); new = p > -1e-6 if it == 0 else (p > 0) | (act[k] & (p > -1e-5))
            changed += int((new != act[k]).sum()); act[k] = new
        for k, s in (("rs", 1), ("rc", -1)):
            p = pen_r(M[k], u, s); new = (p > 0) | (act[k] & (p > -1e-5)); changed += int((new != act[k]).sum()); act[k] = new
        print(f"    contact iteration {it}: {changed} changes; active " + " ".join(f"{k} {int(v.sum())}" for k, v in act.items()), flush=True)
        if changed == 0 and it > 0: break
    forces = {k: round(float(kpen * sum(row[3] * p_ for row, p_, on in zip(M[k], pen(M[k], u, sg[k]), act[k]) if on)), 1) for k in ("c1", "c2", "c3")}
    return u, act, forces


# ---------------------------------------------------------------- stresses
def stresses(B, u):
    ub = u[B.off:B.off + B.n]
    g = B.b.interpolate(ub).grad                                         # (3, 3, nel, nqp)
    eps = 0.5 * (g + np.swapaxes(g, 0, 1)); tr = np.einsum("iijk->jk", eps)
    sig = 2 * MU * eps + LAM * tr[None, None] * np.eye(3)[:, :, None, None]
    S = np.moveaxis(sig, (0, 1), (-2, -1)).reshape(-1, 3, 3)
    ev = np.linalg.eigvalsh(S)
    vm = np.sqrt(0.5 * ((ev[:, 0] - ev[:, 1]) ** 2 + (ev[:, 1] - ev[:, 2]) ** 2 + (ev[:, 2] - ev[:, 0]) ** 2))
    X = np.moveaxis(B.b.global_coordinates().value, 0, -1).reshape(-1, 3)
    return X, vm, ev[:, 2], S[:, 2, 2]


def elem_mean(B, v):
    """element mean of a quadrature-point field (P2 tets: 4+ points per element), and the element centroids."""
    nel = B.m.t.shape[1]; return v.reshape(nel, -1).mean(axis=1), B.m.p[:, B.m.t].mean(axis=1).T


def regions(name, X):
    r, z = r_of(X), X[:, 2]; cb = P["collar_cb"]
    if name == "hand base":
        return {"lugs (incl. root at the flange rim)": (r > P["R_flange"] - 1.0) & (z > P["lug_z"][0] - 0.3),
                "flange / spigot (rest)": ~((r > P["R_flange"] - 1.0) & (z > P["lug_z"][0] - 0.3))}
    if name == "collar":
        lip = (z < P["lip_z"][1] + 0.05) & (r < cb + 0.3)
        wall = (r >= cb) & (z < P["z_nose"])
        step = (z >= P["z_nose"]) & (z <= P["z_nose"] + P["t_step"]) & (r < cb + 0.5)
        return {"lip": lip, "wall (below the step)": wall, "step / wall corner": step, "sleeve": z > P["z_nose"] + P["t_step"]}
    return {"nose ring": np.ones(len(X), bool)}


HAND = {"lugs (incl. root at the flange rim)": ("lug bending", 14.7), "lip": ("lip bending", 14.6), "wall (below the step)": ("collar wall", 15.0),
        "step / wall corner": ("collar step", 11.5)}


def run(maxvol, azs=(180.0, 0.0, 90.0), kpen_h=0.25):
    M = build(maxvol); kpen = E / kpen_h                              # penalty: E / 0.25 mm per unit area
    res = dict(maxvol=maxvol, dofs=int(M["ntot"]), tets={b: int(M[k].m.t.shape[1]) for b, k in (("hand", "H"), ("collar", "Cl"), ("nose", "N"))}, cases={})
    keep = None
    for az in azs:
        t0 = time.time(); f, info = load_vector(M, az)
        u, act, cforce = solve(M, f, kpen)
        reg = {}
        for B in (M["H"], M["Cl"], M["N"]):
            X, vm, s1, szz = stresses(B, u)
            vme, Xe = elem_mean(B, vm); s1e, _ = elem_mean(B, s1); sze, _ = elem_mean(B, szz)
            rege = regions(B.name, Xe)
            for rn, m in regions(B.name, X).items():
                if not m.any(): continue
                me_ = rege[rn]
                reg[f"{B.name}: {rn}"] = dict(vm_elem_max=round(float(vme[me_].max()), 2), s1_elem_max=round(float(s1e[me_].max()), 2), szz_elem_max=round(float(sze[me_].max()), 2),
                                             at_s1_elem_max=[round(float(v), 1) for v in Xe[np.where(me_)[0][np.argmax(s1e[me_])]]],
                                             vm_p999=round(float(np.percentile(vm[m], 99.9)), 2), s1_p999=round(float(np.percentile(s1[m], 99.9)), 2),
                                             vm_max=round(float(vm[m].max()), 2), vm_p99=round(float(np.percentile(vm[m], 99)), 2),
                                             s1_max=round(float(s1[m].max()), 2), s1_p99=round(float(np.percentile(s1[m], 99)), 2),
                                             szz_max=round(float(szz[m].max()), 2), szz_p99=round(float(np.percentile(szz[m], 99)), 2),
                                             at_vm_max=[round(float(v), 1) for v in X[np.where(m)[0][np.argmax(vm[m])]]],
                                             at_s1_max=[round(float(v), 1) for v in X[np.where(m)[0][np.argmax(s1[m])]]])
        H = M["H"]; z0 = P["z_flange"][0]; Xh = H.m.p.T
        rim = np.where((np.abs(Xh[:, 2] - z0) < TOL) & (np.abs(r_of(Xh) - P["R_flange"]) < 0.3))[0]
        uz = u[H.vdof(rim)[:, 2]]; d = np.array([np.cos(np.radians(az)), np.sin(np.radians(az))]); xr = Xh[rim, :2] @ d
        tilt = np.polyfit(xr, uz, 1)[0]
        ur = u[H.vdof(rim)]; ux_mean = float((ur[:, :2] @ d).mean())
        res["cases"][f"force along az {az:.0f}"] = dict(regions=reg, rim_uz_max_mm=round(float(np.abs(uz).max()), 4), tilt_deg=round(float(np.degrees(tilt)), 4),
                                                        rot_stiffness_Nm_per_deg=round(LOAD["bending_Nm"] / max(abs(np.degrees(tilt)), 1e-9), 1),
                                                        rim_shift_along_force_mm=round(ux_mean, 4), lever_mm=round(info["lever_mm"], 1),
                                                        active={k: int(v.sum()) for k, v in act.items()}, contact_force_N=cforce, s=round(time.time() - t0, 1))
        print(f"  contact resultants (N): hand->nose ring {cforce['c1']}, lips<-lugs {cforce['c2']}, collar step->nose ring {cforce['c3']}", flush=True)
        print(f"  az {az:.0f}: {time.time() - t0:.0f} s, tilt {np.degrees(tilt):.4f} deg", flush=True)
        for k, v in reg.items(): print(f"     {k:42s} vM max {v['vm_max']:6.2f} elem {v['vm_elem_max']:6.2f} p99.9 {v['vm_p999']:6.2f} | s1 max {v['s1_max']:6.2f} elem {v['s1_elem_max']:6.2f} p99.9 {v['s1_p999']:6.2f} | szz elem {v['szz_elem_max']:6.2f}  at {v['at_vm_max']}", flush=True)
        if keep is None or az == azs[0]: keep = (az, M, u)
    return res, keep


def figure(keep, path="coupler_fea.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    az, M, u = keep
    fig = plt.figure(figsize=(16, 9), facecolor="white")
    fig.text(0.02, 0.96, f"Coupler rev {REV} FEA, {CASE}: force along az {az:.0f} deg (hand pulled out on the az {(az + 180) % 360:.0f} side)", fontsize=15, fontweight="bold")
    fig.text(0.02, 0.925, "von Mises stress (MPa) at the quadrature points; P2 tets, PA-CF E 3.5 GPa (est.), frictionless compression-only contacts, socket rigid", fontsize=10, color="#4a5058")
    vmax = 20.0
    for k, (key, title) in enumerate((("H", "hand base"), ("Cl", "collar"), ("N", "nose ring"))):
        B = M[key]; X, vm, s1, szz = stresses(B, u)
        for j, (view, sel) in enumerate((("section through az 0 (x-z, y~0)", np.abs(X[:, 1]) < 0.8), ("plan, all points (x-y)", np.ones(len(X), bool)))):
            ax = fig.add_axes([0.03 + 0.325 * k, 0.50 - 0.45 * j, 0.30, 0.38])
            o = np.argsort(vm[sel])
            if j == 0: sc = ax.scatter(X[sel, 0][o], X[sel, 2][o], c=vm[sel][o], s=3, cmap="turbo", vmin=0, vmax=vmax); ax.set_xlabel("x mm"); ax.set_ylabel("z mm (+ into the arm)")
            else: sc = ax.scatter(X[sel, 0][o], X[sel, 1][o], c=vm[sel][o], s=1.5, cmap="turbo", vmin=0, vmax=vmax); ax.set_xlabel("x mm"); ax.set_ylabel("y mm")
            ax.set_aspect("equal"); ax.set_title(f"{title}: {view}", fontsize=10); ax.tick_params(labelsize=8)
    cb = fig.colorbar(sc, ax=fig.axes, fraction=0.015, pad=0.01); cb.set_label("von Mises, MPa (clipped at 20)")
    fig.savefig(path, dpi=110); print("figure", path)


if __name__ == "__main__":
    # python3 coupler_fea.py <maxvol> <az,az,...> <tag>   one mesh -> coupler_fea_<tag>.json (+ .npz for the figure)
    # python3 coupler_fea.py merge <tag> <tag>           -> coupler_fea.json, coupler_fea.png
    if sys.argv[1] == "merge":
        runs = [json.load(open(f"coupler_fea_{t}.json")) for t in sys.argv[2:]]
        out = dict(case=CASE, load=LOAD, E_MPa=E, nu=NU, hand_formula=json.load(open("handmount_b_check.json")).get(f"stress, {CASE}"), runs=runs)
        json.dump(out, open("coupler_fea.json", "w"), indent=1, default=float)
    else:
        v, azs, tag = float(sys.argv[1]), tuple(float(a) for a in sys.argv[2].split(",")), sys.argv[3]
        r, keep = run(v, azs=azs)
        json.dump(r, open(f"coupler_fea_{tag}.json", "w"), indent=1, default=float)
        if len(sys.argv) > 4: figure(keep, sys.argv[4])
