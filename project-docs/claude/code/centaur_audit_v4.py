"""Purpose audit for the v4 shell (every feature has a job and room to do it):
 1 hardware at body height sits inside the shell with >= 6 mm (wall 2.5 + 3.5 clearance)
 2 knee actuators + belt pulleys sit inside the thigh covers with >= 3 mm
 3 thigh covers never touch the body through the stride (hip pitch -35..+35 deg)
 4 every shell panel fits the 256 mm bed
 5 no empty bulge: pod face within 12 mm of the hip actuator face"""
import numpy as np, io, contextlib, json
from matplotlib.path import Path
import chassis_v4 as C4
P = C4.shell_points(); NS = len(P)
def sec_poly_at_x(x, z):
    # build lateral half-width profile at plane x by intersecting shell quads: collect points near x
    pts = P.reshape(-1, 3); sel = np.abs(pts[:, 0] - x) < 0.012
    return pts[sel]
XS = C4.XS
def inside_shell(p):
    """clearance to the shell, m (>0 inside), in the exact vertical section at x (slices are planes x = const)"""
    if p[0] <= XS[0] or p[0] >= XS[-1]: return -1.0
    i = np.searchsorted(XS, p[0]) - 1; f = (p[0] - XS[i]) / (XS[i + 1] - XS[i])
    poly = ((1 - f) * P[i, :-1] + f * P[i + 1, :-1])[:, 1:]
    q = np.array([p[1], p[2]]); seg = np.r_[poly, poly[:1]]
    a_, b_ = seg[:-1], seg[1:]; t = np.clip(np.einsum("ij,ij->i", q - a_, b_ - a_) / np.einsum("ij,ij->i", b_ - a_, b_ - a_), 0, 1)
    d = np.min(np.linalg.norm(a_ + t[:, None] * (b_ - a_) - q, axis=1))
    return d if Path(poly).contains_point(q) else -d
def run(BP):
    res = {}
    # 1
    for tag in ("actuators", "suspension"):
        ps = np.unique(np.round(np.vstack([np.asarray(p) for p, t in zip(BP.sc.polys, BP.sc.tags) if t == tag]), 4), axis=0)
        ps = ps[np.abs(ps[:, 1]) < max(C4.Y_POD_F, C4.Y_POD_R) - 0.001]   # body-side hardware: everything inboard of the pod faces
        m = np.array([inside_shell(p) for p in ps], dtype=float)
        res[f"1 {tag} clearance to shell, net of wall (mm)"] = round(1e3 * (np.nanmin(m) - C4.WALL), 1); res[f"1 {tag} worst at (mm)"] = (1e3 * ps[np.nanargmin(m)]).round().tolist()
    # 2 thigh hardware (knee actuators + belts as built) inside the thigh covers, in the femur frame
    PP = BP.P                                      # leg points as built (scaled with the horse)
    worst = 9
    for front in (True, False):
        xh = BP.hipF[0] if front else BP.hipR[0]; sgn = -1.0 if front else 1.0
        yl = C4.C3.Y_LEG_F if front else C4.C3.Y_LEG_R; YT = yl + C4.THIGH_OFF
        Wp = lambda p: np.array([xh + sgn * p[0], yl, BP.ZH + p[1]])
        hip, st = Wp(PP["hip"]), Wp(PP["stifle"]); L = np.linalg.norm(st - hip); d = (st - hip) / L
        u = np.cross([0, 1.0, 0], d); u /= np.linalg.norm(u)
        for tag in ("actuators", "belts"):
            ps = np.vstack([np.asarray(p) for p, t in zip(BP.sc.polys, BP.sc.tags) if t == tag])
            oth = BP.hipR[0] if front else BP.hipF[0]
            ps = ps[(ps[:, 1] > yl - 0.028) & (np.abs(ps[:, 0] - xh) < np.abs(ps[:, 0] - oth)) & (ps[:, 2] < BP.ZH + 0.20)]
            for p in ps:
                t = (p - hip) @ d / L
                T0 = C4.cover_t0(L)
                if t < T0 - 0.6 or t > 1.6: continue
                tt = np.clip(t, 0, 1); rs = C4.R_THIGH[0] + (C4.R_THIGH[1] - C4.R_THIGH[0]) * tt
                a_ = abs((p - hip) @ u) / rs; b_ = abs(p[1] - YT) / C4.THIGH_RY
                t1 = 1 + C4.STIFLE_EXT / L
                if t > t1: a_ = np.hypot(a_, (t - t1) * L / C4.BOT_ROUND)
                if t < T0: a_ = np.hypot(a_, (T0 - t) * L / C4.TOP_ROUND)
                e = (a_ ** C4.THIGH_E + b_ ** C4.THIGH_E) ** (1 / C4.THIGH_E)
                worst = min(worst, (1 - e) * min(rs, C4.THIGH_RY) - C4.WALL)
    res["2 knee actuator / pulleys / belt inside thigh cover, net of wall (mm)"] = round(1e3 * worst, 1)
    fp = np.vstack([np.asarray(p) for p, t, c in zip(BP.sc.polys, BP.sc.tags, BP.sc.cols) if t == "frame" and c[1] > 0.05 and c[0] not in ("#7b8794", "#c9661f", "#d9782a")])   # receiver / horns / grab posts (above the seat by design) checked on their own
    fm = np.array([inside_shell(p) for p in fp])
    res["6 frame (ribs, stringers, girdle, packs) under the skin, net of wall (mm)"] = round(1e3 * (fm.min() - C4.WALL), 1)
    # 7 actuators vs the neck sheath block (fixed size: r 82, 228 deep)
    C3_ = C4.C3; e0 = np.array([C3_._end[0], 0, C3_._end[1]]); e1 = np.array([C3_.cc[0], 0, C3_.cc[1]]); dd = e1 - e0
    ap = np.vstack([np.asarray(p) for p, t in zip(BP.sc.polys, BP.sc.tags) if t == "actuators"])
    tt = np.clip((ap - e0) @ dd / (dd @ dd), 0, 1)
    u_ = np.array([C3_.n_up[1], 0, -C3_.n_up[0]]); v_ = np.array([0, 1.0, 0]); w_ = dd / np.linalg.norm(dd); Lb = np.linalg.norm(dd)
    def block_sdf(q):                       # square prism, half-width SHEATH_HW, from the sheath end to the collar
        r = q - e0; a = r @ w_; b = r @ u_; c = r @ v_
        hw = np.array([C3_.recv_hw(np.clip(x, 0, C3_.SHEATH_L)) for x in a])      # stepped receiver: crown, throat, sheath block
        d = np.stack([np.abs(b) - hw, np.abs(c) - hw, np.maximum(-a, a - Lb)], 1)
        return np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(d.max(1), 0)
    res["7 actuators to neck sheath block (mm)"] = round(1e3 * block_sdf(ap).min(), 1)
    sp = np.vstack([np.asarray(p) for p, t, c in zip(BP.sc.polys, BP.sc.tags, BP.sc.cols) if t in ("suspension",) or (t == "frame" and c[0] in ("#15171a", "#7a5cc0", "#9aa7b4"))])
    res["7b receiver clears frame, keel, packs, swing unit (mm)"] = round(1e3 * block_sdf(sp).min(), 1)
    # 8 packs vs everything around them (box SDF against the other parts' vertices + the sheath block)
    pc = np.array([BP.PACK_X, 0, BP.PACK_Z]); ph = np.array([0.08, 0.055, 0.025])
    def box_sdf(q):
        d = np.abs(q - pc) - ph; return np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(d.max(1), 0)
    others = np.vstack([np.asarray(p) for p, t, c in zip(BP.sc.polys, BP.sc.tags, BP.sc.cols) if t in ("actuators", "suspension", "belts")])
    gg = np.linspace(-1, 1, 7)
    blk = np.array([e0 + dd * t + C3_.recv_hw(t * Lb) * (a * u_ + b * v_) for t in np.linspace(0, 1, 24) for a in gg for b in gg if max(abs(a), abs(b)) == 1])
    res["8 packs clear actuators, swing unit, sheath block (mm)"] = round(1e3 * min(box_sdf(others).min(), box_sdf(blk).min()), 1)
    # 9 octopus seat: nothing of the neck shell rises above the seat plane inside r 150 of the neck axis (plate r 125,
    #   rim posts to r 139); 10 sheath ends to the belly pan (drains + key-shaft T-key access), along the neck axis
    w2 = np.array([C3_.n_up[0], 0, C3_.n_up[1]]); cc3 = np.array([C3_.cc[0], 0, C3_.cc[1]])
    Pf = P.reshape(-1, 3); h = (Pf - cc3) @ w2; rr = np.linalg.norm((Pf - cc3) - h[:, None] * w2[None, :], axis=1)
    near = rr < 0.150
    res["9 neck shell below the seat plane within r 150 (mm)"] = round(-1e3 * h[near].max(), 1) if near.any() else 99.0
    xs_ = C3_._end[0]; zl = np.interp(xs_, C4.XS, C4.sections()[0])
    res["10 sheath ends above the belly pan (mm)"] = round(1e3 * (C3_._end[1] - zl), 1)
    # 3 swept gap thigh vs body: inner face of the thigh at y = Y_THIGH - THIGH_RY, body max |y| = Y_POD
    # swept sector of each thigh in side view: stand pose +-35 deg about the hip, out to the cover's end (+ section depth)
    Lth = np.linalg.norm(np.asarray(BP.P["stifle"]) - np.asarray(BP.P["hip"])) + C4.STIFLE_EXT + 0.06
    pts = P.reshape(-1, 3)
    def gap(front):
        sgn = -1.0 if front else 1.0; xh = BP.hipF[0] if front else BP.hipR[0]
        st = np.array([xh + sgn * BP.P["stifle"][0], BP.ZH + BP.P["stifle"][1]]); hp = np.array([xh, BP.ZH])
        th0 = np.arctan2(*(st - hp)[::-1]); v = pts[:, [0, 2]] - hp
        ang = (np.arctan2(v[:, 1], v[:, 0]) - th0 + np.pi) % (2 * np.pi) - np.pi
        rad = np.linalg.norm(v, axis=1)
        # femur swing from the gait study (walk, trot, fast trot: -15.1..+19.6 deg) and kneeling (+27.9), + 10 deg each way;
        # leg frame = world for the rear legs, mirrored (sign flipped) for the front legs
        lo_, hi_ = np.radians(-15.1 - 10), np.radians(27.9 + 10)
        if front: lo_, hi_ = -hi_, -lo_
        half = np.arcsin(np.clip(C4.R_THIGH[0] / np.maximum(rad, 1e-3), 0, 1))
        inside = (ang > lo_ - half) & (ang < hi_ + half) & (rad < Lth + C4.R_THIGH[0])
        if C4.C3.KNEE_MODE == "scapula":            # the pauldron above the hip sweeps the opposite way round
            angu = (ang + np.pi + np.pi) % (2 * np.pi) - np.pi
            inside |= (angu > lo_ - half) & (angu < hi_ + half) & (rad < C4.C3.R_K + 0.044 + 0.0055 + C4.TOP_ROUND)
            inside |= rad < C4.R_THIGH[0] * 1.08
        yl = C4.C3.Y_LEG_F if front else C4.C3.Y_LEG_R
        return yl + C4.THIGH_OFF - C4.THIGH_RY - np.abs(pts[inside, 1]).max()
    g_f, g_r = gap(True), gap(False)
    res["3 thigh-to-body swept gap (mm)"] = round(1e3 * min(g_f, g_r), 1)
    # 4 panels
    pn = C4.panels(P); big = [(n, e.tolist()) for n, e in pn if e[2] > 1e3 * C4.BED]
    res["4 shell panels"] = len(pn); res["4 panels over the bed"] = big
    res["4 largest panel (mm)"] = max((e.tolist() for _, e in pn), key=lambda e: e[2])
    # 5 pod face vs actuator face
    res["5 pod face - actuator face (mm)"] = round(1e3 * (C4.Y_POD - C4.Y_ACT_OUT), 1)
    return res
if __name__ == '__main__':
    with contextlib.redirect_stdout(io.StringIO()):
        import build_preview as BP
    res = run(BP)
    for k, v in res.items(): print(f"{k:55s} {v}")
    json.dump(res, open("audit_v4.json", "w"), indent=1, default=str)
