"""Coupler rev B.5 collision check by range math (no mesh booleans), 2026-11-19.

Every solid feature is a range box in cylindrical coordinates: an angle range (deg), a radius range (mm) and a height range
that may vary with r (chamfers and arc fillets are exact z(r) formulas).  Two features collide only if their angle, radius
AND height ranges all overlap; touching (overlap 0) is a seat, not a collision.  Off-axis round features (bullets, screws,
detent bore) are checked against each other with exact centre-distance math, and against revolved features through their
bounding ranges (r +- rho, az +- asin(rho / r)), which is conservative.

States: locked (collar theta 0) with the hand on the nose ring (dz 0) and pushed out onto the lips (dz -ax_clr); the collar
turning from open to locked (theta -60 .. 0) with the hand seated; and the hand pushed in from 25 mm out with the collar open.
Output: coupler_ranges.json (every pair checked, its clearance in mm, and which range separates it)."""
import json
import numpy as np
import handmount_b as H

P = H.P
EPS = 1e-6
t, f, c, w = P["t_lug"], P["lug_fillet"], P["lip_chamfer"], P["wall_fillet"]
R, cb, zl = P["R_flange"], P["collar_cb"], P["lip_z"][1]
zt = P["z_nose"] + P["t_step"]
const = lambda v: (lambda r: np.full_like(r, v, dtype=float))


class Rev:
    """Revolved range box: angle ranges [(a0, a1)], r0..r1, z from zlo(r) to zhi(r)."""
    def __init__(s, name, part, angs, r0, r1, zlo, zhi, holes=()):
        s.name, s.part, s.angs, s.r0, s.r1, s.holes = name, part, angs, r0, r1, list(holes)
        s.zlo = zlo if callable(zlo) else const(zlo); s.zhi = zhi if callable(zhi) else const(zhi)

    def moved(s, dz=0.0, dth=0.0):
        return Rev(s.name, s.part, [(a + dth, b + dth) for a, b in s.angs], s.r0, s.r1,
                   lambda r, g=s.zlo: g(r) + dz, lambda r, g=s.zhi: g(r) + dz, [h.moved(dz, dth) for h in s.holes])


class Cyl:
    """Off-axis round feature: centre at (rc, az), radius rho, z0..z1."""
    def __init__(s, name, part, rc, az, rho, z0, z1):
        s.name, s.part, s.rc, s.az, s.rho, s.z0, s.z1 = name, part, rc, az, rho, z0, z1; s.holes = []

    def moved(s, dz=0.0, dth=0.0):
        return Cyl(s.name, s.part, s.rc, s.az + dth, s.rho, s.z0 + dz, s.z1 + dz)

    def as_rev(s):
        da = np.degrees(np.arcsin(min(s.rho / s.rc, 1.0))) if s.rc > 0 else 180.0
        return Rev(s.name, s.part, [(s.az - da, s.az + da)], max(s.rc - s.rho, 0.0), s.rc + s.rho, s.z0, s.z1)


FULL = [(-180.0, 180.0)]


def ang_gap(A, B):
    """Smallest angular gap (deg) between two sets of arcs; negative = overlap (by that many deg)."""
    best = 1e9
    for a0, a1 in A:
        for b0, b1 in B:
            if a1 - a0 >= 360 - EPS or b1 - b0 >= 360 - EPS:
                return -min(a1 - a0, b1 - b0)
            ca, cb_, ha, hb = (a0 + a1) / 2, (b0 + b1) / 2, (a1 - a0) / 2, (b1 - b0) / 2
            d = abs((ca - cb_ + 180) % 360 - 180)
            best = min(best, d - ha - hb)
    return best


def zr(F):
    if isinstance(F, Cyl):
        return F.z0, F.z1
    r = np.linspace(F.r0, F.r1, 50); return float(np.min(F.zlo(r))), float(np.max(F.zhi(r)))


def inside(A, Hh, host):
    """Margin (mm) of A's part that lies in host's r range fitting inside hole Hh, in plan/angle and radius; and the
    z interval of A that this hole covers.  None if A is not inside the hole in plan."""
    if isinstance(A, Cyl) and isinstance(Hh, Cyl):
        xa = np.array([A.rc * np.cos(np.radians(A.az)), A.rc * np.sin(np.radians(A.az))])
        xh = np.array([Hh.rc * np.cos(np.radians(Hh.az)), Hh.rc * np.sin(np.radians(Hh.az))])
        m = Hh.rho - (np.linalg.norm(xa - xh) + A.rho)
        return (m, (Hh.z0, Hh.z1)) if m > -EPS else None
    A = A.as_rev() if isinstance(A, Cyl) else A; Hh = Hh.as_rev() if isinstance(Hh, Cyl) else Hh
    r0, r1 = max(A.r0, host.r0), min(A.r1, host.r1)
    if r1 <= r0:
        return (1e9, (-1e9, 1e9))
    mr = min(r0 - Hh.r0, Hh.r1 - r1)
    (a0, a1), (h0, h1) = A.angs[0], Hh.angs[0]
    d = abs(((a0 + a1) / 2 - (h0 + h1) / 2 + 180) % 360 - 180)
    ma = np.radians((h1 - h0) / 2 - d - (a1 - a0) / 2) * r0
    m = min(mr, ma)
    return (m, zr(Hh)) if m > -EPS else None


def clearance(A, B):
    """(clearance mm, separating range).  Negative = collision depth.  A feature that sits in a hole of the other part
    is checked for fitting inside that hole (its covered height must be spanned by the hole(s))."""
    cl, sep = _clearance(A, B)
    if cl > -EPS:
        return cl, sep
    for X, Y in ((A, B), (B, A)):
        holes = getattr(Y, "holes", [])
        if not holes:
            continue
        yz = zr(Y); xz = zr(X); z0, z1 = max(xz[0], yz[0]), min(xz[1], yz[1])
        segs = [(inside(X, h, Y if isinstance(Y, Rev) else Y.as_rev()), h) for h in holes]
        segs = [(m, iv) for (res, h) in segs if res is not None for (m, iv) in [res]]
        if not segs:
            continue
        cover = sorted(iv for _, iv in segs); zc = z0; mz = 1e9
        for lo, hi in cover:
            if lo <= zc + EPS and hi > zc:
                mz = min(mz, zc - lo if zc == z0 else 1e9); zc = hi
        if zc >= z1 - EPS:
            m = min(m for m, _ in segs)
            return (round(min(m, zc - z1 + 1e3 * 0), 3), "fit in hole")
    return cl, sep


def _clearance(A, B):
    """(clearance mm, separating range).  Negative = collision depth."""
    if isinstance(A, Cyl) and isinstance(B, Cyl):
        xa = np.array([A.rc * np.cos(np.radians(A.az)), A.rc * np.sin(np.radians(A.az))])
        xb = np.array([B.rc * np.cos(np.radians(B.az)), B.rc * np.sin(np.radians(B.az))])
        dp = np.linalg.norm(xa - xb) - A.rho - B.rho; dz = max(B.z0 - A.z1, A.z0 - B.z1)
        return (max(dp, dz), "plan" if dp >= dz else "z")
    A = A.as_rev() if isinstance(A, Cyl) else A; B = B.as_rev() if isinstance(B, Cyl) else B
    seps = []
    ga = ang_gap(A.angs, B.angs)
    if ga > -EPS:
        seps.append((np.radians(ga) * max(min(A.r0, B.r0), 0.5), "angle"))
    gr = max(B.r0 - A.r1, A.r0 - B.r1)
    if gr > -EPS:
        seps.append((gr, "radius"))
    lo, hi = max(A.r0, B.r0), min(A.r1, B.r1)
    if hi > lo:
        r = np.linspace(lo, hi, 400)
        gz = np.max([np.min(np.maximum(B.zlo(r) - A.zhi(r), A.zlo(r) - B.zhi(r)))])
        seps.append((float(gz), "height"))
    else:                                                     # radius ranges apart: the height gap of the whole boxes also counts
        za, zb = zr(A), zr(B); seps.append((max(zb[0] - za[1], za[0] - zb[1]), "height"))
    return max(seps)


def gusset_lo(r):     # lug root arc fillet: material above the arc, centre (R + f, -t - f)
    return -t - f + np.sqrt(np.clip(f ** 2 - (R + f - r) ** 2, 0, None))


def lug_lo(r):        # lug underside with the lug_chamfer on its bottom outer edge
    return -t + np.clip(r - (P["lug_r1"] - P["lug_chamfer"]), 0, None)


def lip_hi(r):        # lip top: lip_chamfer at the inner edge, arc wall fillet (radius w) at the outer edge
    z = np.full_like(r, zl)
    z = np.minimum(z, zl - c + (r - P["lip_r0"]))
    m = r > cb - w
    z[m] = zl + w - np.sqrt(np.clip(w ** 2 - (r[m] - (cb - w)) ** 2, 0, None))
    return z


def parts():
    hand, col, arm = [], [], []
    hand += [Rev("hand flange", "hand", FULL, 0.0, R, P["z_flange"][0], 0.0),
             Rev("hand spigot", "hand", FULL, 0.0, P["r_spigot"], -0.5, P["z_spigot_top"])]
    kw = np.degrees(P["key_w"] / 2 / P["r_bore"])
    hand.append(Rev("hand key", "hand", [(P["az_key"] - kw, P["az_key"] + kw)], P["r_spigot"] - 0.5, P["r_bore"] + P["key_h"], -0.5, P["z_bore"] - 1.0))
    for az in P["lug_az"]:
        a = [(az - P["lug_w"] / 2, az + P["lug_w"] / 2)]
        hand += [Rev(f"lug {az:.0f}", "hand", a, P["lug_r0"], P["lug_r1"], lug_lo, 0.0),
                 Rev(f"lug {az:.0f} root fillet", "hand", a, R, R + f, gusset_lo, -t)]
    for az in P["az_bullet"]:
        hand.append(Cyl(f"male bullet pin {az:.0f}", "hand", P["r_bullet"], az, P["pin_d"] / 2, 0.0, P["pin_len"]))
    z0 = P["collar_z"][0]
    col += [Rev("collar wall", "collar", FULL, cb, P["R_collar"], z0, zt),
            Rev("collar step", "collar", FULL, P["collar_bore"], P["R_collar"], P["z_nose"], zt),
            Rev("collar sleeve", "collar", FULL, P["collar_bore"], P["sleeve_r"], zt, P["collar_z"][1])]
    dimp = [Cyl(f"dimple {a:.0f}", "collar", P["ball_rc"], a, P["ball_d"] / 2 + 0.05, P["det_z"] - P["ball_d"] / 2 - 0.05, P["det_z"] + P["ball_d"] / 2 + 0.05)
            for a in (P["det_az"], P["det_az"] + P["twist"])]
    for x in col:
        x.holes = dimp
    for az in P["lug_az"]:
        col.append(Rev(f"lip {az:.0f}", "collar", [(az - P["lip_w"] / 2, az + P["lip_w"] / 2)], P["lip_r0"], cb, z0, lip_hi))
    kwc = np.degrees(P["key_w"] / 2 / P["r_bore"]) + 2.0
    holes = [Rev("keyway", "arm", [(P["az_key"] - kwc, P["az_key"] + kwc)], P["r_bore"] - 0.5, P["r_bore"] + P["key_h"] + 0.3, -1.0, P["z_bore"])]
    for az in P["az_bullet"]:
        holes += [Cyl(f"bullet mouth {az:.0f}", "arm", P["r_bullet"], az, P["mouth_r"], -1.0, P["fem_recess"] + 0.01),
                  Cyl(f"female bore {az:.0f}", "arm", P["r_bullet"], az, P["fem_bore"] / 2, P["fem_recess"], P["z_back"] + 1.0)]
    holes.append(Cyl("detent bore", "arm", (P["det_r_in"] + P["R_body"]) / 2, P["det_az"], P["det_bore"] / 2, P["det_z"] - P["det_bore"] / 2, P["det_z"] + P["det_bore"] / 2))
    arm += [Rev("nose ring", "arm", FULL, P["r_bore"], P["R_nose"], 0.0, P["z_nose"], holes),
            Rev("socket body", "arm", FULL, P["r_bore"], P["R_body"], P["z_nose"], P["z_back"], holes),
            Rev("retaining flange", "arm", FULL, P["r_bore"], P["retain_r"], P["retain_z"][0], P["retain_z"][1], holes)]
    for r, az in P["screws"]:
        arm += [Cyl(f"screw head {az:.0f}", "arm", r, az, 2.75, P["cbore_h"] - 2.0, P["cbore_h"]),       # DIN 7984 M3: head d 5.5, k 2.0
                Cyl(f"screw shank {az:.0f}", "arm", r, az, 1.5, P["cbore_h"], P["cbore_h"] + 20.0)]
    for az in P["az_bullet"]:
        arm.append(Cyl(f"female barrel {az:.0f}", "arm", P["r_bullet"], az, P["bul_od"] / 2, P["fem_recess"], P["fem_recess"] + P["bul_len"]))
    arm.append(Cyl("detent ball", "arm", P["ball_rc"], P["det_az"], P["ball_d"] / 2, P["det_z"] - P["ball_d"] / 2, P["det_z"] + P["ball_d"] / 2))
    return hand, col, arm


def wire_paths():
    """Lead paths as chains of round sections (Cyl), so the same range math applies.  Connector frame: hand base bottom
    face at z_flange[0]; hand3's pack frame below it, its clear centre channel r < 7 (hand3_cad: nothing inside r 7 from
    the pack frame to the motor base at hand3 z 54)."""
    rw = P["wire_od"] / 2; fz0 = P["z_flange"][0]; gz = (fz0 + 0.15, fz0 + 0.15 + P["wire_od"])        # wire lies on the pack-frame disc
    hand, hand_hw, arm, tipbay = [], [], [], []
    for az in P["az_bullet"]:
        # side-entry from the male cup's saddle, then radially inward along the groove to the centre pocket
        for k, r in enumerate(np.arange(P["r_bullet"] - P["bul_od"] / 2 - 0.2, P["lead_rv"] + P["lead_bend_r"] - 0.01, -0.3)):
            hand.append(Cyl(f"hand lead {az:.0f} @{k}", "hand", r, az, rw, *gz))
        # 90 deg bend, radius lead_bend_r (around a mandrel), from horizontal to vertical at r = lead_rv, then straight down.
        # Each sampled cross-section is exact: it spans +- rw cos q in r and +- rw sin q in z (q = bend angle), +- rw sideways.
        Rb = P["lead_bend_r"]; zc = (gz[0] + gz[1]) / 2; r_v = P["lead_rv"]
        for k, q in enumerate(np.linspace(0, 90, 31)):
            c, sn = np.cos(np.radians(q)), np.sin(np.radians(q)); rr = r_v + Rb * c; zz = zc - Rb * sn
            da = np.degrees(np.arcsin(min(rw / rr, 1.0)))
            dr, dz = rw * c + 0.05, rw * sn + 0.05                     # 0.05: half the sampling step
            hand.append(Rev(f"hand lead {az:.0f} bend @{k}", "hand", [(az - da, az + da)], rr - dr, rr + dr, zz - dz, zz + dz))
        hand.append(Cyl(f"hand lead {az:.0f} down @0", "hand", r_v, az, rw, fz0 - 60.0, zc - Rb))
        # arm side: female cup tail 1.5 past the socket back into the tip bay, ESC lead soldered on axis
        arm.append(Cyl(f"female tail {az:.0f}", "arm", P["r_bullet"], az, P["bul_od"] / 2, P["z_back"], P["fem_recess"] + P["bul_len"]))
        arm.append(Cyl(f"ESC lead {az:.0f}", "arm", P["r_bullet"], az, rw, P["fem_recess"] + P["bul_len"], P["fem_recess"] + P["bul_len"] + 8.0))
    for az in P["hand_screw_az"]:          # M3 heat-set insert (OD 4.0, 6 long) + M3 x 8 DIN 7991 countersunk from the pack side
        hand_hw.append(Cyl(f"hand insert {az:.0f}", "hand", P["hand_screw_r"], az, 2.0, fz0, fz0 + 6.0))
        hand_hw.append(Cyl(f"hand screw head {az:.0f}", "hand", P["hand_screw_r"], az, 3.0, fz0 - 2.5, fz0 - 1.0))
    # hand3 pack frame below the hand base: hub disc r 7 .. 17.5, 2.5 thick, then the frame; centre channel r < 7 open
    ch = P["pf_chamfer"]                                                # B.6: 45 deg chamfer on the tube's inner top edge (lead bend)
    top = lambda rr: fz0 - np.clip(ch - (rr - 7.0), 0, None)
    hand_hw.append(Rev("hand3 pack-frame disc", "hand3", FULL, 7.0, 17.5, fz0 - 2.5, top))
    hand_hw.append(Rev("hand3 servos (r >= 15.3)", "hand3", FULL, 15.3, 42.9, fz0 - 50.8, fz0 - 3.0))
    hand_hw.append(Rev("hand3 pack-frame centre tube (r 7-10)", "hand3", FULL, 7.0, 10.0, fz0 - 50.0, top))
    # tip bay end face (arm side, behind the socket): plate z_back .. +2.5 with 3 x M3 insert bosses at the screw positions,
    # and a hole for each female tail (B.6: three Ø6.0 at r_bullet; the old spec had only a centre Ø21 opening)
    for r, az in P["screws"]:
        tipbay.append(Cyl(f"tip-bay insert boss {az:.0f}", "arm", r, az, 3.2, P["z_back"], P["z_back"] + 8.0))
    fits = {"male body in its bore (radial)": (P["male_bore"] - P["bul_od"]) / 2, "female body in its bore (radial)": (P["fem_bore"] - P["bul_od"]) / 2,
            "pin in the mouth lead-in (radial)": P["mouth_r"] - P["pin_d"] / 2, "lead in its groove (each side)": (P["groove_w"] - P["wire_od"]) / 2,
            "lead in its groove (depth, closed by the pack-frame disc)": P["groove_h"] - P["wire_od"] - 0.15,
            "3 leads going down hand3's r 7 channel (7 - r_v - wire r)": 7.0 - P["lead_rv"] - P["wire_od"] / 2,
            "leads side by side at r_v (gap between leads)": round(float(2 * P["lead_rv"] * np.sin(np.radians(60)) - P["wire_od"]), 3),
            "lead bend radius / wire OD": round(P["lead_bend_r"] / P["wire_od"], 2),
            "bullet engagement (mm)": P["pin_len"] - P["fem_recess"],
            "female tail past the socket back, into the tip bay (mm)": P["fem_recess"] + P["bul_len"] - P["z_back"],
            "tip-bay hole for each tail: dia 6.0 vs body 5.0 (radial)": 0.5,
            "wheel hand: 3 x 20 AWG F60 Pro V leads (OD 1.8, bundle dia 3.9) through its dia 6.8 back-plate hole (radial)": round(3.4 - 0.9 * (1 + 2 / np.sqrt(3)), 2)}
    return dict(hand=hand, hand_hw=hand_hw, arm=arm, tipbay=tipbay), fits


def check():
    hand, col, arm = parts(); out = []
    def run(state, A, B, skip=()):
        for a in A:
            for b in B:
                if (a.name, b.name) in skip or any(s in a.name and s2 in b.name for s, s2 in skip):
                    continue
                cl, sep = clearance(a, b)
                out.append(dict(state=state, a=a.name, b=b.name, clearance_mm=round(float(cl), 3), separated_by=sep, ok=bool(cl > -EPS)))
    seat = (("male bullet", "female barrel"), ("hand flange", "nose ring"), ("lug", "nose ring"), ("detent ball", "socket body"),
            ("screw", "nose ring"), ("screw", "socket body"), ("screw", "retaining flange"), ("female barrel", "nose ring"), ("female barrel", "socket body"))
    arm_noself = [x for x in arm if not x.name.startswith(("nose ring", "socket body", "retaining"))]
    # locked, hand seated on the nose ring and pushed out onto the lips
    for dz in (0.0, -P["ax_clr"]):
        hs = [h.moved(dz) for h in hand]
        run(f"locked, hand dz {dz}", hs, col + arm, skip=seat)
    # collar turning open -> locked with the hand seated
    for th in np.linspace(-P["twist"], 0, 13):
        run(f"collar turning, theta {th:.0f}", hand, [x.moved(dth=th) for x in col], skip=seat)
    # hand pushed in with the collar open
    co = [x.moved(dth=-P["twist"]) for x in col]
    for dz in np.linspace(-25, 0, 26):
        hs = [h.moved(dz) for h in hand if not h.name.startswith("male bullet")]
        run(f"inserting, hand dz {dz:.0f}", hs, co + [a for a in arm if a.name in ("nose ring", "socket body")], skip=seat)
    # collar against the arm at every turn
    for th in (-P["twist"], 0.0):           # at the two detent stops the ball sits in a dimple; between them it is pushed back into its bore
        run(f"collar vs arm, theta {th:.0f}", [x.moved(dth=th) for x in col], arm, skip=(("collar step", "nose ring"),))
    run("collar vs arm, theta -30 (ball retracted)", [x.moved(dth=-P["twist"] / 2) for x in col], [a for a in arm if a.name != "detent ball"], skip=(("collar step", "nose ring"),))
    # inside the arm: screws, barrels, detent among themselves
    run("arm internals", arm_noself, arm_noself, skip=())
    # wiring: motor phase leads (14 AWG silicone, OD wire_od) arm -> bullets -> hand -> motor
    wires, wfits = wire_paths()
    hw = [x for x in hand if not x.name.startswith(("male bullet",))]
    run("wiring, hand side", wires["hand"], hw + wires["hand_hw"], skip=(("hand lead", "hand flange"),))
    run("wiring, hand side (leads vs each other)", wires["hand"], wires["hand"], skip=())
    run("wiring, arm side", wires["arm"], [a for a in arm if not a.name.startswith(("female barrel", "nose ring", "socket body", "retaining"))] + wires["tipbay"], skip=())
    lead = lambda n: " ".join(n.split(" ")[:3]) if n.startswith("hand lead") else n
    out = [o for o in out if o["a"] != o["b"] and lead(o["a"]) != lead(o["b"])]
    # fits that must be clearance fits (inner part inside its bore)
    fits = {"spigot in bore (radial)": P["r_bore"] - P["r_spigot"], "hand flange inside the lips (radial)": P["lip_r0"] - R,
            "lug tips inside the collar wall (radial)": cb - P["lug_r1"], "collar bore on the socket body (radial)": P["collar_bore"] - P["R_body"],
            "collar wall outside the nose ring (radial)": cb - P["R_nose"], "lug underside above the lip top (axial, seated)": -t - zl,
            "lugs pass the open lips (deg each side)": (120 - P["lip_w"]) / 2 - P["lug_w"] / 2}
    fits.update(wfits)
    return out, fits


if __name__ == "__main__":
    out, fits = check()
    bad = [o for o in out if not o["ok"]]
    worst = {}
    for o in out:
        k = (o["a"].split(" ")[0] + " " + o["a"].split(" ")[1], o["b"].split(" ")[0] + " " + o["b"].split(" ")[1])
        if k not in worst or o["clearance_mm"] < worst[k]["clearance_mm"]:
            worst[k] = o
    tight = sorted(worst.values(), key=lambda o: o["clearance_mm"])[:15]
    json.dump(dict(pairs_checked=len(out), collisions=bad, fits_mm=fits, tightest=tight), open("coupler_ranges.json", "w"), indent=1)
    print("pairs checked:", len(out), " collisions:", len(bad))
    for o in bad[:20]:
        print("  COLLIDE", o)
    print("fits (mm / deg):", {k: round(v, 3) for k, v in fits.items()})
    print("tightest:")
    for o in tight:
        print(f"  {o['clearance_mm']:7.3f}  {o['separated_by']:7s} {o['a']} | {o['b']}  [{o['state']}]")
