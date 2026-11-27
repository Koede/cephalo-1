#!/usr/bin/env python3
"""Clearance of the rev B leg assembly, every part (structure, motors, belts, pulleys, bearings, pins, lines, springs, toggle).
Rules:
  parts on the same joint node (rigid to each other)  -> no shared volume (overlap <= 0.5 mm3)
  parts on different nodes                             -> exact solid-to-solid distance >= CLR (1.5 mm)
  declared running fits / belt wraps / line on drum    -> distance >= 0.02 mm (modelled at FIT = 0.05 mm)
  declared elastic contacts (TPU centring ring, buffer)-> reported, contact allowed
Distances are manifold min_gap (exact, no booleans), pruned by bounding-box intervals.  Lines, the gas spring and the toggle
links are rebuilt at every pose from the joint tree (they are not rigid to one node).
    python3 legs_cad_verify.py static | limits | range | all"""
import json, itertools, os, sys
import numpy as np
import hippos_legs_cad as C
HERE = os.path.dirname(os.path.abspath(__file__)); CLR = 1.5; RUN_MIN = 0.02
RUN = [("output rotor", " RS0"), ("abduction front stub", "abduction bearing"), ("pivot motor", "pivot output adapter"), ("yaw bearings", "yaw yoke"), ("stub bearing", "joint motor"), ("yaw belt", "yaw yoke"),
       ("abduction stubs", "abduction motor"), ("abduction stubs", "abduction bearing"), ("drive pulley", "joint motor"), ("drive crank", "joint motor"),
       ("shoulder belt", "drive pulley"), ("shoulder belt", "shoulder pulley"), ("shoulder belt", "belt idler"), ("bearings 2 x 6805", "shaft 25"),
       ("tendons", "scapula"), ("tendons", "femur"), ("tendons", "forearm"), ("tendons", "hind cannon"),
       ("pin bearings 2 x 6800", "elbow pin"), ("pin bearings 2 x 6800", "hock pin"), ("carpus bearings", "carpus pin"), ("fetlock pin", "pastern"),
       ("ball joint", "hoof core"), ("push rod", "drive crank"), ("push rod", "driven crank"),
       ("toggle link 1", "toggle pin A"), ("toggle link 2", "toggle pin B"), ("toggle link 1", "toggle link 2"), ("toggle link", "carpus pin"),
       ("flexor line", "flexor spool"), ("flexor line", "pastern"), ("release line", "release spool"), ("release line", "toggle link")]
ELASTIC = [("pastern", "centring ring"), ("pastern", "fetlock buffer")]
RANGE = dict(fore=dict(q0=(0, 60), q1=(50, 140), fold=(0, 120), yaw=(-25, 25), abd=(-10, 10), fet=(0, 30), hp=(-15, 15), hr=(-15, 15)),
             hind=dict(q0=(-9, 67), q1=(50, 146), yaw=(-25, 25), abd=(-10, 10), fet=(0, 30), hp=(-15, 15), hr=(-15, 15)))


def has(pairs, a, b):
    return any((x in a and y in b) or (x in b and y in a) for x, y in pairs)


def knee_of(L, T, t, p):
    """Toggle knee in leg coords (and in the lower node's rest coords), fold branch or lock branch."""
    Ta, Tb = T[t["a"][0]], T[t["b"][0]]
    a = np.array(t["a"][1], float); b = (np.linalg.inv(Ta) @ Tb @ np.array(list(t["b"][1]) + [1.0]))[:3]
    d = b - a; d[1] = 0; ln = np.linalg.norm(d)
    if ln > 2 * t["l"] + 1e-6:
        return None, None, None, ln
    h = np.sqrt(max(t["l"] ** 2 - ln * ln / 4, 0.0)); dn = d / ln; perp = np.array([dn[2], 0.0, -dn[0]])
    sgn = -1.0 if p.get("fold", 0.0) < t["lock_below"] else 1.0
    k = (a + b) / 2 + sgn * perp * h
    return a, b, k, ln


def extras(kind, p, T):
    """Pose-dependent solids in leg coords: [(name, solid)]."""
    L = C.FORE if kind == "fore" else C.HIND; out = []; knees = []
    for t in L.toggles:
        a, b, k, ln = knee_of(L, T, t, p)
        if k is None:
            raise ValueError("toggle over-extended: anchors %.2f apart, links 2 x %.2f" % (ln, t["l"]))
        knees.append((t, k)); Ta = T[t["a"][0]][:3, :4]
        def link(p0, p1, yy, knee_bore):
            m = C.hull(C.cyl_y(6.5, yy[0], yy[1], p0[0], p0[2]), C.cyl_y(t["r"], yy[0], yy[1], (p0[0] + p1[0]) / 2, (p0[2] + p1[2]) / 2), C.cyl_y(5, yy[0], yy[1], p1[0], p1[2]))
            m = m - C.hole_y(4, yy[0] - 1, yy[1] + 1, p0[0], p0[2])
            return m - C.hole_y(3, yy[0] - 1, yy[1] + 1, p1[0], p1[2]) if knee_bore else m
        l1 = link(a, k, t["y1"], False) + C.cyl_y(3, t["ypin"][0], t["ypin"][1], k[0], k[2])
        l2 = link(b, k, t["y2"], True)
        out.append((t["name"] + ", toggle link 1 (+ knee pin)", l1.transform(Ta))); out.append((t["name"] + ", toggle link 2", l2.transform(Ta)))
    for s in L.struts:
        def pt(e, other=None):
            if e[0] == "knee":
                t, k = knees[e[1]]; q = np.array([k[0], e[2], k[2], 1.0]); return (T[t["a"][0]] @ q)[:3]
            if e[0] == "tan":                                  # tangent point on a spool, seen from the line's other end
                Tn = T[e[1]]; o = (np.linalg.inv(Tn) @ np.append(other, 1.0))[:3]; c = np.array(e[2], float); r = e[3]
                v = np.array([o[0] - c[0], o[2] - c[2]]); Lv = np.linalg.norm(v); al = np.arccos(min(r / Lv, 1.0)); be = np.arctan2(v[1], v[0])
                cand = [np.array([c[0] + r * np.cos(be + s_ * al), c[1], c[2] + r * np.sin(be + s_ * al)]) for s_ in (1, -1)]
                q = min(cand, key=lambda u: u[0]) if e[4] == "minx" else max(cand, key=lambda u: u[2])
                return (Tn @ np.append(q, 1.0))[:3]
            return (T[e[0]] @ np.array(list(e[1]) + [1.0]))[:3]
        b = pt(s["b"]); a = pt(s["a"], b); d = b - a; ln = np.linalg.norm(d); u = d / ln; ta, tb = s.get("trim", (0, 0))
        if ln - ta - tb > 1.0:
            out.append((s["name"], C.rod(a + u * ta, b - u * tb, s["r"], 12)))
    return out


def posed(kind, p):
    L = C.FORE if kind == "fore" else C.HIND; T = C.fk(C.TREE[kind], p); items = []
    for q in L.parts:
        m = q["solid"].transform(T[q["node"]][:3, :4]); bb = m.bounding_box()
        items.append((q["name"], q["node"], m, np.array(bb[:3]), np.array(bb[3:])))
    for i, (nm, m) in enumerate(extras(kind, p, T)):
        bb = m.bounding_box(); items.append((nm, "x%d" % i, m, np.array(bb[:3]), np.array(bb[3:])))
    return items, T


def check(kind, p, clr=CLR, only=None, nodes=None):
    """-> (worst general gap, pair), [violations (gap, need, a, b)], {elastic pair: gap}."""
    items, _ = posed(kind, p); worst = (99.0, "", ""); bad = []; el = {}
    for (na, ka, a, lo_a, hi_a), (nb, kb, b, lo_b, hi_b) in itertools.combinations(items, 2):
        if ka == kb: continue
        if only and not only(na, nb): continue
        if nodes and not nodes(ka, kb): continue
        need = RUN_MIN if has(RUN, na, nb) else clr
        if has(ELASTIC, na, nb):
            if np.any(lo_a - hi_b > 3) or np.any(lo_b - hi_a > 3): continue
            g = a.min_gap(b, 3.0); k = (na, nb); el[k] = min(el.get(k, 9), g); continue
        lim = max(need, min(worst[0], 12.0) if need == clr else need)
        if np.any(lo_a - hi_b > lim) or np.any(lo_b - hi_a > lim): continue
        g = a.min_gap(b, lim)
        if need == clr and g < worst[0]: worst = (g, na, nb)
        if g < need: bad.append((round(float(g), 3), need, na, nb))
    return worst, bad, el


def static():
    out = {}
    for kind, L in (("fore", C.FORE), ("hind", C.HIND)):
        rows = []
        for a, b in itertools.combinations(L.parts, 2):
            if a["node"] != b["node"]: continue
            ba, bb = a["solid"].bounding_box(), b["solid"].bounding_box()
            if any(ba[i] > bb[i + 3] or bb[i] > ba[i + 3] for i in range(3)): continue
            v = (a["solid"] ^ b["solid"]).volume()
            if v > 0.5: rows.append((round(v, 1), a["name"], b["name"]))
        out[kind] = rows; print(kind, "same-node overlaps:", rows if rows else "none", flush=True)
    return out


def sweep(kind, base, var, vals, clr=CLR):
    rows = []
    for v in vals:
        p = dict(base); p.update({var: v}) if isinstance(var, str) else p.update(dict(zip(var, v)))
        try:
            w, bad, el = check(kind, p, clr)
        except ValueError as e:
            rows.append((v, -1.0, str(e), "", [])); continue
        rows.append((v, round(float(w[0]), 2), w[1][:34], w[2][:34], bad))
    return rows


def first_fail(kind, base, var, vals, lo=None):
    """Sweep one joint; -> (last value that passes, first failing value, limiting pairs)."""
    ok = None
    for r in sweep(kind, base, var, vals):
        if lo is not None and r[0] < lo: continue
        if r[4] or r[1] < CLR:
            return ok, r[0], [(b[0], b[2][:40], b[3][:40]) for b in r[4][:3]]
        ok = r[0]
    return ok, None, []


BODY_SIDE = ("rocker", "cradle", "yoke")


def grid(kind, quick=False):
    """Whole joint space.  A: body-side parts (carrier, cradle, yoke and what they carry) against everything, over pitch, flex,
    twist, sideways swing and fold.  B: the leg's own parts against each other over flex, fold, fetlock and hoof tilt."""
    R = RANGE[kind]; st = C.STAND[kind]; res = dict(worst=(99.0, "", "", None), bad={}, elastic={}, poses=0)
    def run(p, nodes):
        try:
            w, bad, el = check(kind, p, nodes=nodes)
        except ValueError as e:
            res["bad"][("toggle", str(e))] = (-1.0, dict(p)); return
        res["poses"] += 1
        if w[0] < res["worst"][0]: res["worst"] = (round(float(w[0]), 2), w[1], w[2], dict(p))
        for g, need, a, b in bad:
            if (a, b) not in res["bad"] or g < res["bad"][(a, b)][0]: res["bad"][(a, b)] = (g, dict(p))
        for k, g in el.items():
            if k not in res["elastic"] or g < res["elastic"][k][0]: res["elastic"][k] = (round(float(g), 2), dict(p))
    lin = lambda lo, hi, n: [round(float(x), 1) for x in np.linspace(lo, hi, n)]
    A = lambda ka, kb: (ka in BODY_SIDE) != (kb in BODY_SIDE) or (ka in BODY_SIDE and kb in BODY_SIDE)
    B = lambda ka, kb: ka not in BODY_SIDE and kb not in BODY_SIDE
    folds = (0.0, R["fold"][1]) if kind == "fore" else (0.0,)
    for q0 in lin(*R["q0"], 3 if quick else 5):
        for q1 in lin(*R["q1"], 3 if quick else 6):
            for yaw in lin(*R["yaw"], 3 if quick else 5):
                for abd in lin(*R["abd"], 3):
                    for fold in folds:
                        run(dict(q0=q0, q1=q1, yaw=yaw, abd=abd, fold=fold), A)
    for q1 in lin(*R["q1"], 10 if quick else (R["q1"][1] - R["q1"][0]) // 2 + 1):
        for fold in (lin(0, R["fold"][1], 5 if quick else 13) if kind == "fore" else (0.0,)):
            run(dict(q0=st["q0"], q1=q1, fold=fold), B)
    for q1 in (R["q1"][0], st["q1"], R["q1"][1]):
        for fold in ((0.0, 60.0, R["fold"][1]) if kind == "fore" else (0.0,)):
            for fet in lin(*R["fet"], 3):
                for hp in lin(*R["hp"], 3):
                    for hr in lin(*R["hr"], 3):
                        run(dict(q0=st["q0"], q1=q1, fold=fold, fet=fet, hp=hp, hr=hr), B)
    return res


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "all"
    if mode in ("static", "all"):
        static()
    if mode in ("limits", "all"):
        for kind in ("fore", "hind"):
            st = C.STAND[kind]; R = RANGE[kind]
            w, bad, el = check(kind, st); print("==", kind, "stand: tightest", round(w[0], 2), w[1][:30], "|", w[2][:30], "| under the rule:", bad, "| elastic:", {k[0][:16] + "/" + k[1][:16]: round(v, 2) for k, v in el.items()})
            for q0 in (R["q0"][0], st["q0"], R["q0"][1]):
                print("  q0 %5.1f: flex" % q0, first_fail(kind, dict(q0=q0), "q1", range(R["q1"][0], 176, 2)), flush=True)
            if kind == "fore":
                for q1 in (R["q1"][0], st["q1"], 110, R["q1"][1]):
                    print("  q1 %5.1f: fold" % q1, first_fail(kind, dict(q0=st["q0"], q1=q1), "fold", range(0, 151, 3)), flush=True)
    if mode in ("range", "all"):
        out = {}
        for kind in ("fore", "hind"):
            r = grid(kind, quick="quick" in sys.argv)
            print("==", kind, "poses", r["poses"], "| tightest moving pair", r["worst"][0], r["worst"][1][:36], "|", r["worst"][2][:36], r["worst"][3])
            for k, (g, p) in sorted(r["bad"].items(), key=lambda kv: kv[1][0]):
                print("   UNDER THE RULE %6.2f  %s | %s  at %s" % (g, k[0][:40], k[1][:40], p))
            for k, (g, p) in r["elastic"].items():
                print("   elastic contact %5.2f  %s | %s  at %s" % (g, k[0][:36], k[1][:36], p))
            out[kind] = dict(poses=r["poses"], tightest=r["worst"][0], tightest_pair=r["worst"][1:3], tightest_at=r["worst"][3],
                             under_rule=[dict(gap=g, a=k[0], b=k[1], at=p) for k, (g, p) in r["bad"].items()],
                             elastic=[dict(gap=g, a=k[0], b=k[1], at=p) for k, (g, p) in r["elastic"].items()])
        json.dump(dict(rule_mm=CLR, running_fit_mm=RUN_MIN, range=RANGE, result=out), open(os.path.join(HERE, "legs_cad_verify.json"), "w"), indent=1, default=float)
