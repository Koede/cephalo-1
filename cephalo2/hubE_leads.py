"""Cephalo-2 rev B.7: motor phase leads from the coupler bullets to hand3 hub E's motor, option F of
hubE_lead_options_study (14 AWG down the empty back face of the servo stack, inside a screwed-on cover rail).

One place for the lead geometry, used by the CAD (handmount_b.hand_base grooves, hand3_hubc.frame cuts), the viewer and the
motion sim (meshes), and the clearance check (this file's check()).

Frames.  "Aligned" frame (x', y', zh): hand3's own frame seen from the coupler side, x' = -x_hand, y' = y_hand, zh = hand3
height (0 = frame plate bottom = hand-base bottom face; the hand base is at zh < 0, the motor at zh 58).  The servo stack's
back face is y' = -17.5, so s = -y' is the distance out toward the back.  Connector frame = Rz(PSI) of the aligned frame with
z = z_flange[0] - zh; PSI = lead_exit_az - 270 puts the exit in the middle of a lug gap of the hand base.

Route of each 14 AWG lead: side-entry in its male bullet cup -> groove in the hand-base bottom face -> its lane in the exit
slot -> dive (45 deg down between the collar lip and the servo corner, through a notch in the frame plate) -> up the back
face inside the rail -> inline solder joint to the motor's own 18 AWG lead, cut to a short stub -> through a notch in the
top block -> motor base.  The 5 signal wires (26 AWG, est.) share the slot and the rail in their own lane.
Estimates: SE 2816 lead exit height, STS3215 rear axle boss (r 4.5 x 2 proud) and port positions.  Units mm."""
import json
import numpy as np
import manifold3d as mf
import handmount_b as HB

P = HB.P
PSI = P["lead_exit_az"] - 270.0
FZ0 = P["z_flange"][0]
RW, RS, RSIG = P["wire_od"] / 2, P["stub_od"] / 2, P["sig_od"] / 2
ZG = -(0.15 + RW)                                   # lead centre height in its groove (0.15 off the frame plate)
LANE = P["lanes"]                                   # x' of each lane in the exit slot and the rail
SV = P["lead_sv"]                                   # s of the vertical run
RB = P["lead_bend_r"]
JOINT_Z = dict(B=43.0, A=43.0, C=32.5); JOINT_R, JOINT_L = 1.9, 9.0      # inline lap solder joint 14 -> 18 AWG + heat shrink, dia 3.8 (est.)
STUB_X = dict(B=-2.5, A=0.0, C=2.5); STUB_S = 19.4; Z_EXIT = 59.3        # 18 AWG stubs at the motor base (est.)
RAIL = dict(x=(min(LANE.values()) - RW - 0.25, max(LANE.values()) + RW + 0.25), s=(18.0, 22.8), wall=1.2, z=(4.5, 48.9), ear_t=1.5)
BOSS = dict(lo=dict(x=(8.6, 12.6), s=(18.0, 22.4), z=(0.0, 4.5)), hi=dict(x=(-14.4, -9.4), s=(18.0, 22.4), z=(48.9, 53.4)), pilot=0.85)
PLATE_NOTCH = dict(x=(RAIL["x"][0] - 0.2, RAIL["x"][1] + 0.2), s=(15.0, 18.1))
BLOCK_NOTCH = dict(x=(-4.2, 4.2), s=(15.3, 21.0), z=(53.4, 58.1), z_out=51.3)   # beyond the bridge edge (s > 18) it starts at z_out
SERVO_BOSS = [(-12.0, 14.35), (12.0, 39.05)]         # (x', zh) of the STS3215 rear axle bosses, r 4.5, 2 proud (est.)


def bullet_xy():
    """Male bullet axes in the aligned frame, keyed B / A / C by where their lane is."""
    out = {}
    for az in P["az_bullet"]:
        a = np.radians(az - PSI); p = P["r_bullet"] * np.array([np.cos(a), np.sin(a)])
        out["A" if p[1] > 5 else ("B" if p[0] < 0 else "C")] = p
    return out


def turtle(p0, h0, moves, step=0.2):
    """Plan polyline from p0 heading h0 (deg): ("L", length) or ("A", radius, signed turn deg, + = anticlockwise)."""
    p = np.array(p0, float); h = np.radians(h0); pts = [p.copy()]
    for m in moves:
        if m[0] == "L":
            n = max(int(np.ceil(m[1] / step)), 1); d = np.array([np.cos(h), np.sin(h)])
            for i in range(1, n + 1): pts.append(p + d * m[1] * i / n)
            p = pts[-1].copy()
        else:
            R, turn = m[1], np.radians(m[2]); sg = np.sign(turn); c = p + sg * R * np.array([-np.sin(h), np.cos(h)])
            n = max(int(np.ceil(abs(turn) * R / step)), 2)
            for i in range(1, n + 1):
                a = h + turn * i / n; pts.append(c + sg * R * np.array([np.sin(a), -np.cos(a)]))
            p = pts[-1].copy(); h += turn
    return np.array(pts)


def dive(x, zg, s0, step=0.2):
    """(x', y', zh) points from the end of the groove run (s0) through the dive to the start of the vertical run."""
    al = np.radians(P["lead_alpha"]); sA = P["lead_sA"]; d = P["lead_d"]; pts = []
    for s in np.arange(s0, sA, step): pts.append((s, zg))
    for q in np.linspace(0, al, 14): pts.append((sA + RB * np.sin(q), zg + RB * (1 - np.cos(q))))
    s1, z1 = pts[-1]
    for t in np.linspace(0, d, max(int(d / step), 2))[1:]: pts.append((s1 + t * np.sin(al) * 0 + t * np.cos(al), z1 + t * np.sin(al)))
    s2, z2 = pts[-1]
    for q in np.linspace(al, np.pi / 2, 14)[1:]: pts.append((s2 + RB * (np.sin(q) - np.sin(al)), z2 + RB * (np.cos(al) - np.cos(q))))
    return [(x, -s, z) for s, z in pts]


def derive_dive():
    """Solve lead_sA so the vertical run lands on lead_sv."""
    al = np.radians(P["lead_alpha"])
    P["lead_sA"] = SV - RB * np.sin(al) - P["lead_d"] * np.cos(al) - RB * (1 - np.sin(al))
    return P["lead_sA"]


derive_dive()


def paths(step=0.2):
    """name -> dict(pts (N,3) aligned frame, r).  Power leads B/A/C (14 AWG), their 18 AWG stubs, and the signal bundle."""
    b = bullet_xy(); out = {}
    plan = dict(
        B=turtle(b["B"], 0.0, [("L", (LANE["B"] - RB) - b["B"][0]), ("A", RB, -90.0)], step),
        C=turtle(b["C"], 180.0, [("L", b["C"][0] - (LANE["C"] + RB)), ("A", RB, 90.0)], step))
    th = np.degrees(np.arccos(1 - (b["A"][0] - LANE["A"]) / (2 * RB)))          # S-bend that shifts lead A sideways onto its lane
    plan["A"] = turtle(b["A"], -90.0, [("L", b["A"][1] + 4.5), ("A", RB, -th), ("A", RB, th)], step)
    plan["sig"] = turtle((LANE["sig"], -1.0), -90.0, [("L", 2.0)], step)
    for k, pl in plan.items():
        r = RSIG if k == "sig" else RW; zg = -(0.15 + r)
        pts = [(x, y, zg) for x, y in pl] + dive(LANE[k], zg, -pl[-1][1] + step, step)
        ztop = P["sig_ztop"] if k == "sig" else JOINT_Z[k]
        z0 = pts[-1][2]
        pts += [(LANE[k], -SV, z) for z in np.arange(z0 + step, ztop, step)]
        out["sig" if k == "sig" else f"lead {k}"] = dict(pts=np.array(pts), r=r)
        if k == "sig":
            continue
        out[f"joint {k}"] = dict(pts=np.array([(LANE[k], -SV, z) for z in np.arange(ztop - JOINT_L / 2, ztop + JOINT_L / 2 + 1e-6, step)]), r=JOINT_R)
        # 18 AWG stub: slants from the joint to its place at the top-block notch, up, then a bend into the motor base
        za, zb, zc = ztop + JOINT_L / 2, 50.0, Z_EXIT - 2.5
        zs = max(za, 47.8)                                            # stay in the lane past the neighbours' joints, then slant
        st = [(LANE[k], -SV, z) for z in np.arange(za, zs, step)]
        st += [(LANE[k] + (STUB_X[k] - LANE[k]) * t, -(SV + (STUB_S - SV) * t), zs + (zb - zs) * t) for t in np.linspace(0, 1, int((zb - zs) / step) + 2)]
        st += [(STUB_X[k], -STUB_S, z) for z in np.arange(zb + step, zc, step)]
        st += [(STUB_X[k], -(STUB_S - 2.5 * (1 - np.cos(q))), zc + 2.5 * np.sin(q)) for q in np.linspace(0, np.pi / 2, 16)[1:]]
        st += [(STUB_X[k], -s, Z_EXIT) for s in np.arange(STUB_S - 2.5 - step, 16.0, -step)]
        out[f"stub {k}"] = dict(pts=np.array(st), r=RS)
    return out


# ---------------------------------------------------------------- frames
def to_conn(pts, dz=0.0):
    """Aligned (x', y', zh) -> connector frame."""
    pts = np.atleast_2d(np.asarray(pts, float)); a = np.radians(PSI); c, s = np.cos(a), np.sin(a)
    return np.c_[c * pts[:, 0] - s * pts[:, 1], s * pts[:, 0] + c * pts[:, 1], FZ0 - pts[:, 2] + dz]


def to_hand(pts):
    """Aligned -> hand3's own frame."""
    pts = np.atleast_2d(np.asarray(pts, float)); return np.c_[-pts[:, 0], pts[:, 1], pts[:, 2]]


def hand3_to_conn_matrix(dz=0.0):
    """4x4: hand3 frame -> connector frame (end for end about y, clocked PSI about z, on the hand-base bottom face)."""
    a = np.radians(PSI); R = np.eye(4); R[:2, :2] = [[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]
    T = np.eye(4); T[2, 3] = FZ0 + dz
    return T @ R @ np.diag([-1.0, 1.0, -1.0, 1.0])


def box_al(x, s, z):
    """Box given in the aligned frame (x' range, s range, zh range), built in hand3's own frame."""
    return mf.Manifold.cube([x[1] - x[0], s[1] - s[0], z[1] - z[0]]).translate([-x[1], -s[1], z[0]])


def capsule_chain(pts, r, every=1.0, n=12):
    """Swept round wire as a union of hulled spheres (pts in the frame you want the mesh in)."""
    pts = np.asarray(pts, float); keep = [0]; acc = 0.0
    for i in range(1, len(pts)):
        acc += np.linalg.norm(pts[i] - pts[i - 1])
        if acc >= every or i == len(pts) - 1: keep.append(i); acc = 0.0
    sp = [mf.Manifold.sphere(float(r), n).translate([float(v) for v in pts[i]]) for i in keep]
    return mf.Manifold.batch_boolean([mf.Manifold.batch_hull([sp[i], sp[i + 1]]) for i in range(len(sp) - 1)], mf.OpType.Add)


# ---------------------------------------------------------------- parts (hand3 frame)
def rail():
    """Cover rail: PA-CF U-channel, open toward the servo backs, two ears on the frame's rail bosses (2 x M2 x 6 thread-forming:
    6 long so neither tip leaves its boss toward the collar or the curl ring)."""
    (x0, x1), (s0, s1), w, (z0, z1), et = RAIL["x"], RAIL["s"], RAIL["wall"], RAIL["z"], RAIL["ear_t"]
    m = box_al((x0 - w, x1 + w), (s0, s1 + w), (z0, z1)) - box_al((x0, x1), (s0 - 1, s1), (z0 - 1, z1 + 1))
    lo, hi = BOSS["lo"], BOSS["hi"]
    m += box_al((x1 + w - 0.01, lo["x"][1]), lo["s"], (lo["z"][1], lo["z"][1] + et))          # bottom ear, on top of the lower boss
    m += box_al((hi["x"][0], x0 - w + 0.01), hi["s"], (hi["z"][0] - et, hi["z"][0]))          # top ear, under the upper boss
    for b, zr in ((lo, (lo["z"][1] - 1, lo["z"][1] + et + 1)), (hi, (hi["z"][0] - et - 1, hi["z"][0] + 1))):
        xc, sc = np.mean(b["x"]), np.mean(b["s"])
        m -= mf.Manifold.cylinder(zr[1] - zr[0], 1.15, 1.15, 20).translate([-xc, -sc, zr[0]])    # dia 2.3 clearance hole
    for xb, zb in SERVO_BOSS:                                                                   # relief over a servo's rear axle boss
        if x0 - w < xb + 4.7 and x1 + w > xb - 4.7:
            m -= mf.Manifold.cylinder(2.3, 4.9, 4.9, 32).rotate([90, 0, 0]).translate([-xb, -s0 + 0.01, zb])
    for zc in P["sig_ports"]:                                                                   # windows for the signal wires to the servo ports
        m -= box_al((x0 - w - 1, x0 + 0.01), (s0 - 0.1, s0 + 2.6), (zc - 1.6, zc + 1.6))
    return m


def rail_screws():
    out = []
    for b, sg, zf in ((BOSS["lo"], -1, BOSS["lo"]["z"][1] + RAIL["ear_t"]), (BOSS["hi"], 1, BOSS["hi"]["z"][0] - RAIL["ear_t"])):
        xc, sc = np.mean(b["x"]), np.mean(b["s"])
        head = mf.Manifold.cylinder(1.3, 1.9, 1.9, 20).translate([0, 0, zf if sg < 0 else zf - 1.3])
        shank = mf.Manifold.cylinder(6.0, 1.0, 1.0, 16).translate([0, 0, zf - 6.0 if sg < 0 else zf])
        out.append((head + shank).translate([-xc, -sc, 0]))
    return out


def frame_cuts():
    """What rev B.7 removes from hub E's frame (hand3 frame): plate notch for the dive, top-block notch for the stubs."""
    n, b = PLATE_NOTCH, BLOCK_NOTCH
    return box_al(n["x"], n["s"], (-1.0, 3.0)) + box_al(b["x"], b["s"], b["z"]) + box_al(b["x"], (18.0, b["s"][1]), (b["z_out"], b["z"][1]))


def frame_adds():
    """Two rail bosses on the frame's back edges, each with a dia 1.7 pilot for an M2 thread-forming screw."""
    m = None
    for b in (BOSS["lo"], BOSS["hi"]):
        q = box_al(b["x"], (b["s"][0] - 0.3, b["s"][1]), b["z"]) - mf.Manifold.cylinder(b["z"][1] - b["z"][0] + 2, BOSS["pilot"], BOSS["pilot"], 16).translate([-np.mean(b["x"]), -np.mean(b["s"]), b["z"][0] - 1])
        m = q if m is None else m + q
    return m


def lead_meshes(frame="hand", dz=0.0):
    """name -> (manifold, kind) in hand3's frame ("hand") or the connector frame ("conn")."""
    out = {}
    for n, d in paths(0.25).items():
        al = d["pts"]
        if n.startswith("lead"):          # drawn from where the insulation starts: outside the bullet cup, up to the joint sleeve
            al = al[(np.linalg.norm(al[:, :2] - bullet_xy()[n[-1]], axis=1) > P["bul_od"] / 2 + d["r"] + 0.3) & (al[:, 2] < JOINT_Z[n[-1]] - JOINT_L / 2 - d["r"] - 0.3)]
        if n.startswith("joint"): al = al[(al[:, 2] > al[:, 2].min() + d["r"]) & (al[:, 2] < al[:, 2].max() - d["r"])]
        if n.startswith("stub"):          # from the joint sleeve to the motor base's wall
            al = al[(al[:, 2] > JOINT_Z[n[-1]] + JOINT_L / 2 + d["r"] + 0.3) & ~((al[:, 2] > 58.0) & (np.hypot(al[:, 0], al[:, 1]) < 17.0 + d["r"] + 0.3))]
        pts = to_hand(al) if frame == "hand" else to_conn(al, dz)
        out[n] = capsule_chain(pts, d["r"], every=0.8 if d["r"] > 1.2 else 0.6)
    return out


# ---------------------------------------------------------------- clearance check (math)
def _seg_d(p, a, b):
    ab = b - a; t = np.clip(((p - a) @ ab) / (ab @ ab), 0, 1); return np.linalg.norm(p - a - np.outer(t, ab), axis=1)


def poly_dist(pts2, poly):
    """Distance from 2-D points to a convex or concave polygon (0 inside)."""
    poly = np.asarray(poly, float); n = len(poly); d = np.full(len(pts2), 1e9); inside = np.zeros(len(pts2), bool)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]; d = np.minimum(d, _seg_d(pts2, a, b))
        c = ((a[1] > pts2[:, 1]) != (b[1] > pts2[:, 1])) & (pts2[:, 0] < (b[0] - a[0]) * (pts2[:, 1] - a[1]) / (b[1] - a[1] + 1e-300) + a[0]); inside ^= c
    return np.where(inside, 0.0, d)


def box_dist(p, lo, hi):
    q = np.maximum(np.maximum(np.asarray(lo) - p, p - np.asarray(hi)), 0.0); return np.linalg.norm(q, axis=1)


def check():
    """Smallest gap from every lead to everything it passes.  Negative = collision."""
    import sys, os
    here = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(here, "hand3e"))
    import hand3_hubc as C
    res = {}; pa = paths(0.1)
    def note(k, v, who):
        if k not in res or v < res[k][0]: res[k] = (round(float(v), 3), who)
    # collar: lips taken as a full ring (they sweep it when the collar turns), plus the wall, in (r, z) of the connector frame
    z0 = P["collar_z"][0]; zl = P["lip_z"][1]; cb = P["collar_cb"]; cbt = P["lip_chamfer_bot"]
    lip = [(P["lip_r0"] + cbt, z0), (P["R_collar"], z0), (P["R_collar"], zl), (P["lip_r0"], zl), (P["lip_r0"], z0 + cbt)]
    # hand frame boxes: servos, frame plate (three boxes round the notch), bridge
    hl, hw = C.SL / 2, C.SH / 2; n = PLATE_NOTCH; xw = hl + C.WALL_T
    servo = [((-hl, -hw, C.Z_CURL), (hl, hw, C.Z_CURL + C.SW)), ((-hl, -hw, C.Z_BASE), (hl, hw, C.Z_BASE + C.SW))]
    plate = [((-xw, -n["s"][0], 0), (xw, C.Y_HALF, C.PLATE_T)), ((-xw, -C.Y_HALF, 0), (-n["x"][1], -n["s"][0], C.PLATE_T)), ((-n["x"][0], -C.Y_HALF, 0), (xw, -n["s"][0], C.PLATE_T))]
    bridge = [((-xw, -C.Y_HALF, C.Z_SERVO_TOP), (xw, C.Y_HALF, C.Z_SERVO_TOP + C.BRIDGE_T))]
    cords = [(np.array(p), np.array(q)) for pb, pc in ((0.0, 0.0), (C.TRAVEL_B, C.TRAVEL_C)) for p, q in C.cord_runs(pb, pc).values()]
    zin_lo = C.Z_FL_IN[0] - C.TRAVEL_C; zcage = C.Z["cage_lo"]; bn = BLOCK_NOTCH
    for name, d in pa.items():
        al, r = d["pts"], d["r"]; h = to_hand(al); w = to_conn(al)
        rr = np.hypot(w[:, 0], w[:, 1])
        note("collar (lips as a full ring, wall)", (poly_dist(np.c_[rr, w[:, 2]], lip) - r).min(), name)
        for nm, boxes in (("servo bodies", servo), ("frame plate (with its notch)", plate), ("frame bridge", bridge)):
            note(nm, min((box_dist(h, lo, hi) - r).min() for lo, hi in boxes), name)
        note("moving cords", min((_seg_d(h, p, q) - r - C.CORD_R).min() for p, q in cords), name)
        rh = np.hypot(h[:, 0], h[:, 1]); z = h[:, 2]
        # top block r 20 (zh 51.4 .. 58) outside its notch
        zlo = np.where(-al[:, 1] > 18.0, -1e9, bn["z"][0])           # beyond the bridge edge the notch is the block's full height
        inn = (al[:, 0] > bn["x"][0]) & (al[:, 0] < bn["x"][1]) & (-al[:, 1] > bn["s"][0]) & (z > zlo)
        blk = np.where(inn, np.minimum.reduce([al[:, 0] - bn["x"][0], bn["x"][1] - al[:, 0], -al[:, 1] - bn["s"][0], np.where(-al[:, 1] > 18.0, 9.9, z - zlo)]),
                       np.hypot(np.maximum(rh - 20.0, 0), np.maximum(np.maximum(C.Z_SERVO_TOP - z, z - C.Z["pack1"]), 0)))
        blk = np.maximum(blk, np.maximum(z - C.Z["pack1"], C.Z_SERVO_TOP - z)) - r     # above or below the block nothing of it is nearer than that
        note("top block (with its notch)", blk.min(), name)
        note("curl pull ring, fully pulled (spins)", (np.hypot(np.maximum(C.R_IN[0] - rh, 0), np.maximum(zin_lo - z, 0)) - r).min(), name)
        note("rotor cage (spins)", (np.hypot(np.maximum(C.R_CAGE[0] - rh, 0), np.maximum(zcage - z, 0)) - r).min(), name)
        if not name.startswith("stub"):                           # inside the rail: walls at its x and s limits
            m = (z > RAIL["z"][0]) & (z < RAIL["z"][1])
            if m.any():
                note("rail inner walls", min((al[m, 0] - RAIL["x"][0] - r).min(), (RAIL["x"][1] - al[m, 0] - r).min(), (RAIL["s"][1] + al[m, 1] - r).min()), name)
    # leads against each other (different wires, sampled centre-lines)
    names = list(pa)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = names[i], names[j]
            if a.split()[-1] == b.split()[-1] and "sig" not in (a, b): continue          # lead / joint / stub of one phase
            A, B = pa[a]["pts"][::3], pa[b]["pts"][::3]
            dm = np.sqrt(((A[:, None, :] - B[None, :, :]) ** 2).sum(-1)).min() - pa[a]["r"] - pa[b]["r"]
            note("lead to lead (touching side by side = 0)", dm, f"{a} | {b}")
    # hand base: inserts and bullets against the grooves
    b = bullet_xy(); ins = [P["hand_screw_r"] * np.array([np.cos(np.radians(a - PSI)), np.sin(np.radians(a - PSI))]) for a in P["hand_screw_az"]]
    for name, d in pa.items():
        if name.startswith(("stub", "joint")): continue
        al = d["pts"][d["pts"][:, 2] < 0.5]; gw = d["r"] + 0.15
        for q in ins: note("hand-base insert bores (wall left)", (np.linalg.norm(al[:, :2] - q, axis=1) - gw - 2.0).min(), name)
        for k, q in b.items():
            if k in name: continue
            note("other bullets' bores (wall left)", (np.linalg.norm(al[:, :2] - q, axis=1) - gw - P["male_bore"] / 2).min(), name)
    # exit slot against the lugs (deg of full rim left beside the slot at the flange radius)
    a0 = 270 - np.degrees(np.arcsin((-PLATE_NOTCH["x"][0] + 0.0) / P["R_flange"])) + PSI; a1 = 270 + np.degrees(np.arcsin(PLATE_NOTCH["x"][1] / P["R_flange"])) + PSI
    lug_end = max(az + P["lug_w"] / 2 for az in P["lug_az"] if az + P["lug_w"] / 2 < a0 + 1); lug_start = min((az - P["lug_w"] / 2) % 360 for az in P["lug_az"] if (az - P["lug_w"] / 2) % 360 > a1 - 1)
    res["exit slot to the lug ends at the rim (mm of full rim, each side)"] = (round(float(min(a0 - lug_end, lug_start - a1) * np.pi / 180 * P["R_flange"]), 2), f"slot az {a0:.1f}..{a1:.1f}")
    # rail against the rest (range boxes, aligned frame)
    x0, x1 = RAIL["x"][0] - RAIL["wall"], RAIL["x"][1] + RAIL["wall"]; s1 = RAIL["s"][1] + RAIL["wall"]
    r_out = float(np.hypot(max(abs(BOSS["hi"]["x"][0]), abs(BOSS["lo"]["x"][1])), BOSS["lo"]["s"][1]))
    res["rail + bosses below the curl ring's lowest sweep (axial)"] = (round(float(zin_lo - BOSS["hi"]["z"][1]), 2), f"ring bottom zh {zin_lo:.1f}, r >= {C.R_IN[0]}; rail parts reach r {r_out:.1f}")
    res["rail to the moving cords"] = (round(float(min(_seg_d(np.array([[-x, -s, z] for x in (x0, x1, BOSS['lo']['x'][1], BOSS['hi']['x'][0]) for s in (18.0, s1) for z in np.arange(0, 53.5, 0.5)]), p, q).min() for p, q in cords) - C.CORD_R), 2), "")
    res["rail below the collar (axial, rail top edge is at zh 0)"] = (round(float(-(FZ0 - z0)) - 0.0, 2), "bosses start at zh 0")
    res["rail screw heads to the servo rear bosses, est. (in x')"] = (round(float(min(abs(np.mean(BOSS["lo"]["x"]) - SERVO_BOSS[0][0]), abs(np.mean(BOSS["hi"]["x"]) - SERVO_BOSS[1][0])) - 4.5 - 1.9), 2), "")
    return res


if __name__ == "__main__":
    res = check(); bad = {k: v for k, v in res.items() if v[0] < -1e-6}
    L = {n: float(np.linalg.norm(np.diff(d["pts"], axis=0), axis=1).sum()) for n, d in paths(0.1).items()}
    R14, R18 = 8.29e-3, 20.9e-3; I = 36.0
    loss = sum(I * I * (L[f"lead {k}"] * R14 + L[f"stub {k}"] * R18) * 1e-3 for k in "BAC")
    out = dict(psi_deg=PSI, lead_sA=round(P["lead_sA"], 3), lengths_mm={k: round(v, 1) for k, v in L.items()}, loss_hover_W=round(loss, 2),
               loss_max_W=round(loss * (54.4 / 36.0) ** 2, 2), clearances_mm={k: dict(gap=v[0], at=v[1]) for k, v in res.items()}, collisions=list(bad))
    json.dump(out, open("hubE_leads.json", "w"), indent=1)
    print("PSI", PSI, " lead_sA", round(P["lead_sA"], 2), " loss at hover", round(loss, 2), "W")
    print("lengths:", out["lengths_mm"])
    for k, v in sorted(res.items(), key=lambda kv: kv[1][0]): print(f"  {v[0]:7.2f}  {k}   [{v[1]}]")
    print("collisions:", len(bad))
