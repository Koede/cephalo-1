"""Wheel hand for Cephalo-2 (2026-11-18): a drive wheel that plugs into hand connector rev B instead of a rotor hand.
The hand carries its own motor; the arm's ESC (the one that spins a hand3 rotor) drives it through the three bullet
contacts, and the hand-ID pin tells the robot to switch that ESC to wheel mode (current limit 13 A, sine start).
Sizing: wheelhand_sizing.py.  Connector: handmount_b.py.

Stack along the connector axis (mm, connector frame: +z into the arm, hand toward -z, z = 0 the socket mouth):
  hand base (rev B)             z +10.4 .. -9.0      shared part, 4 x M3 into the housing's back plate
  housing back plate            z  -9.0 .. -12.0     motor mount, 16 x 16 M3 (F60 pattern)
  motor T-Motor F60 Pro V 1750 kV                    stator base on the plate, bell turns inside the housing
  stage 1 planetary m 0.5       sun 15 (brass, on the 4 mm shaft) / 3 planets 30 / ring 75 cut in the housing: 6 : 1
  carrier 1 = sun 2             one CF-resin part: planet pins dia 2, sun 2 m 0.8 12 T on its front
  stage 2 planetary m 0.8       sun 12 / 3 planets 24 / ring 60 in the housing: 6 : 1           total 36 : 1
  carrier 2                     planet pins dia 3, D-bore onto the dia 8 output shaft
  nose                          2 x 698-2RS (8 x 19 x 6) on the output shaft
  wheel                         dished PA-CF hub on the shaft (D-flat + M5 nut), rim r 54 reaching back over the gearbox, 6 mm TPU tyre to r 60, 32 wide
python3 wheelhand.py -> checks (gear centres and backlash, overlaps of every pair, ground clearance, lever arm, masses),
STLs in wheelhand_stl/, wheelhand.json
"""
import os, json
import numpy as np
import manifold3d as mf
import handmount_b as HB

N = 96
M = dict(d=26.8, L=31.7, base_t=4.0, shaft_d=4.0, shaft_out=6.5, m=34.3, mount=16.0)
S1 = dict(zs=15, zp=30, zr=75, m=0.5, b=5.0)
S2 = dict(zs=12, zp=24, zr=60, m=0.8, b=8.0)
BRG = dict(d=8.0, D=19.0, B=6.0, m=0.0072)          # 698-2RS
WHEEL = dict(r_hub=12.0, r_rim=54.0, r_tyre=60.0, w=32.0, spokes=6, rim_t=2.5, web_t=3.0)
WALL = 2.4
BACKLASH = 0.05                                      # mm of tooth thinning per flank (print clearance, est.)


def cyl(r, z0, z1, n=N, x=0.0, y=0.0):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])


def ring(r0, r1, z0, z1, n=N):
    return cyl(r1, z0, z1, n) - cyl(r0, z0 - 1, z1 + 1, n)


# ---------------------------------------------------------------- involute gears
def involute_profile(z, m, addendum=1.0, dedendum=1.25, pa=20.0, thin=0.0, npts=8):
    """Outline of an external involute spur gear (x, y), counter-clockwise."""
    rp = m * z / 2; rb = rp * np.cos(np.radians(pa)); ra = rp + addendum * m; rf = max(rp - dedendum * m, 0.3)
    inv = lambda a: np.tan(a) - a
    # half tooth angle at the pitch circle, thinned for backlash
    t_half = (np.pi / (2 * z)) - thin / rp
    a_p = np.arccos(rb / rp)
    pts = []
    for k in range(z):
        c = 2 * np.pi * k / z
        rs = np.linspace(max(rb, rf), ra, npts)
        flank = []
        for r in rs:
            a = np.arccos(np.clip(rb / r, -1, 1))
            th = t_half + inv(a_p) - inv(a)
            flank.append((r, th))
        left = [(r * np.cos(c - th), r * np.sin(c - th)) for r, th in flank]
        right = [(r * np.cos(c + th), r * np.sin(c + th)) for r, th in flank[::-1]]
        th_root = t_half + inv(a_p) + (np.pi / z - t_half - inv(a_p)) * 0.5
        pts.append((rf * np.cos(c - th_root), rf * np.sin(c - th_root)))
        if rb > rf:
            pts.append((rb * np.cos(c - flank[0][1]) * 0.999, rb * np.sin(c - flank[0][1]) * 0.999))
        pts += left + right
        if rb > rf:
            pts.append((rb * np.cos(c + flank[0][1]) * 0.999, rb * np.sin(c + flank[0][1]) * 0.999))
        pts.append((rf * np.cos(c + th_root), rf * np.sin(c + th_root)))
    return pts


def spur(z, m, z0, z1, thin=BACKLASH, bore=0.0):
    g = mf.Manifold.extrude(mf.CrossSection([involute_profile(z, m, thin=thin)]), z1 - z0).translate([0, 0, z0])
    return g - cyl(bore / 2, z0 - 1, z1 + 1, 32) if bore else g


def internal(z, m, z0, z1, r_out, thin=BACKLASH):
    """Internal ring gear: the tooth spaces are the teeth of an external gear with addendum and dedendum swapped."""
    cutter = mf.Manifold.extrude(mf.CrossSection([involute_profile(z, m, addendum=1.25, dedendum=1.0, thin=-thin)]), z1 - z0 + 2).translate([0, 0, z0 - 1])
    return (cyl(r_out, z0, z1) - cutter).rotate([0, 0, 180.0 / z])     # a ring tooth (not a space) faces each planet's space on the +x axis


# ---------------------------------------------------------------- layout (z stations)
Z = {}
Z["plate"] = (HB.P["z_flange"][0] - 3.0, HB.P["z_flange"][0])            # bolts flat under the hand base (rev B.2)
Z["motor"] = (Z["plate"][0] - M["L"], Z["plate"][0])                        # stator base at the plate, bell down to -43.7
Z["shaft_tip"] = Z["motor"][0] - M["shaft_out"]
Z["s1"] = (Z["shaft_tip"] + 0.5 - 0.0 - S1["b"], Z["shaft_tip"] + 0.5)     # sun 1 face on the shaft end
Z["c1"] = (Z["s1"][0] - 3.0, Z["s1"][0] - 0.6)                              # carrier 1 plate
Z["s2"] = (Z["c1"][0] - S2["b"], Z["c1"][0])                                # sun 2 (on carrier 1) / planets 2
Z["c2"] = (Z["s2"][0] - 3.5, Z["s2"][0] - 0.6)                              # carrier 2 plate
Z["nose"] = (Z["c2"][0] - 2 * BRG["B"] - 3.0, Z["c2"][0] - 0.6)
Z["web"] = (Z["nose"][0] - 1.0 - WHEEL["web_t"], Z["nose"][0] - 1.0)          # the wheel's web sits just past the nose
Z["hub"] = (Z["web"][0], Z["web"][0] + WHEEL["w"])                               # rim + tyre reach back over the gearbox (dished wheel)
A1 = S1["m"] * (S1["zs"] + S1["zp"]) / 2; A2 = S2["m"] * (S2["zs"] + S2["zp"]) / 2
R_RING1 = S1["m"] * S1["zr"] / 2 + 1.25 * S1["m"] + WALL
R_RING2 = S2["m"] * S2["zr"] / 2 + 1.25 * S2["m"] + WALL


def housing():
    """One PA-CF print, front down: back plate (motor + hand base screws), motor can, stage 1 ring, stage 2 ring, nose
    with the two bearing seats.  Rings are cut in the housing (printed teeth; or press-in POM rings, same geometry)."""
    zp0, zp1 = Z["plate"]
    h = cyl(HB.P["R_flange"], zp0, zp1)                                        # back plate
    h += ring(M["d"] / 2 + 0.9, M["d"] / 2 + 0.9 + WALL, Z["motor"][0] - 1.0, zp0)   # motor can
    h += cyl(R_RING1, Z["c1"][0] - 0.5, Z["motor"][0] - 0.99) - cyl(M["d"] / 2 + 0.9, Z["motor"][0] - 1.5, Z["motor"][0])
    h -= cyl(S1["m"] * S1["zr"] / 2 + 1.25 * S1["m"] + 0.3, Z["c1"][0] - 1, Z["s1"][0] + 0.01)    # carrier 1 room below ring 1
    h = h - cyl(M["d"] / 2 + 0.9, Z["s1"][1], Z["motor"][0] + 0.01)
    h = h - cyl(S1["m"] * S1["zr"] / 2 + 1.25 * S1["m"] + 0.01, Z["s1"][0] - 0.01, Z["s1"][1] + 0.01) + internal(S1["zr"], S1["m"], Z["s1"][0], Z["s1"][1], S1["m"] * S1["zr"] / 2 + 1.25 * S1["m"] + 0.3)
    h -= cyl(S1["m"] * S1["zr"] / 2 - 1.0 * S1["m"] - 0.2, Z["s1"][1] - 0.01, Z["motor"][0] + 0.02)   # room for the planet tops above ring 1
    h += cyl(R_RING2, Z["c2"][0] - 0.5, Z["c1"][0]) - cyl(S2["m"] * S2["zr"] / 2 + 1.25 * S2["m"] + 0.3, Z["c2"][0] - 1, Z["c1"][0] + 0.01)
    h = h - cyl(S2["m"] * S2["zr"] / 2 + 1.25 * S2["m"] + 0.01, Z["s2"][0] - 0.01, Z["s2"][1] + 0.01) + internal(S2["zr"], S2["m"], Z["s2"][0], Z["s2"][1], S2["m"] * S2["zr"] / 2 + 1.25 * S2["m"] + 0.3)
    h -= cyl(S2["m"] * S2["zr"] / 2 + 1.25 * S2["m"] + 0.3, Z["c2"][0] - 0.01, Z["s2"][0] + 0.01)  # carrier 2 room
    h -= cyl(S2["m"] * S2["zr"] / 2 + 1.25 * S2["m"] + 0.3, Z["s2"][1] - 0.01, Z["c1"][0] + 0.01)  # carrier 1 runs in the ring-2 bore top
    # nose: cone from ring 2 down to the bearing tube
    nose = mf.Manifold.cylinder(Z["c2"][0] - 0.5 - Z["nose"][0], BRG["D"] / 2 + WALL, R_RING2, N).translate([0, 0, Z["nose"][0]])
    h += nose - cyl(BRG["D"] / 2 + 0.02, Z["nose"][0] - 1, Z["c2"][0] + 0.01) + ring(BRG["d"] / 2 + 1.5, BRG["D"] / 2 + 0.3, Z["nose"][0] + BRG["B"], Z["nose"][0] + BRG["B"] + 1.0)
    # joining flanges: motor can to ring 1 (around the bell, above the planets), ring 1 to ring 2 (above carrier 1)
    h += ring(M["d"] / 2 + 1.6, R_RING1, Z["motor"][0] - 1.5, Z["motor"][0] + 2.0)
    h += ring(S2["m"] * S2["zr"] / 2 + 1.25 * S2["m"] + 0.3, R_RING2, Z["c1"][0] - 1.0, Z["c1"][0] + 1.5) + ring(S1["m"] * S1["zr"] / 2 + 1.25 * S1["m"] + 0.3, R_RING2, Z["c1"][0] + 0.4, Z["c1"][0] + 1.5)
    # holes: motor screws, hand-base screws, motor leads, vent
    s = M["mount"] / 2
    for x, y in ((s, 0), (-s, 0), (0, s), (0, -s)):
        h -= cyl(1.7, zp0 - 1, zp1 + 1, 24, x, y) + cyl(3.0, zp1 - 1.2, zp1 + 1, 24, x, y)
    for az in HB.P["hand_screw_az"]:                                                               # hand base inserts (handmount_b rev B.1)
        a = np.radians(az); r = HB.P["hand_screw_r"]; h -= cyl(1.7, zp0 - 1, zp1 + 1, 24, r * np.cos(a), r * np.sin(a))
    h -= cyl(3.4, zp0 - 1, zp1 + 1, 24)                                                            # motor leads up into the hand base pocket, then out along its grooves to the bullets
    return h


def motor():
    zb, zt = Z["motor"]
    return cyl(M["d"] / 2, zb, zt - M["base_t"]) + cyl(M["d"] / 2 - 1.5, zt - M["base_t"], zt) + cyl(M["shaft_d"] / 2, Z["shaft_tip"], zb, 24)


def stage1():
    out = {"sun1": spur(S1["zs"], S1["m"], *Z["s1"], bore=M["shaft_d"])}
    pl = []
    for k in range(3):
        a = np.radians(120 * k); x, y = A1 * np.cos(a), A1 * np.sin(a)
        p = spur(S1["zp"], S1["m"], *Z["s1"], bore=2.2).rotate([0, 0, 180.0 / S1["zp"] + np.degrees(a) * (S1["zs"] / S1["zp"])])
        pl.append(p.translate([x, y, 0]))
    out["planets1"] = mf.Manifold.batch_boolean(pl, mf.OpType.Add)
    c = cyl(A1 + 4.0, *Z["c1"]) - cyl(2.2, Z["c1"][0] - 1, Z["c1"][1] + 1)
    pins = []
    for k in range(3):
        a = np.radians(120 * k); pins.append(cyl(1.0, Z["c1"][1], Z["s1"][1], 16, A1 * np.cos(a), A1 * np.sin(a)))
    out["pins1"] = mf.Manifold.batch_boolean(pins, mf.OpType.Add)
    out["carrier1_sun2"] = c + spur(S2["zs"], S2["m"], Z["s2"][0], Z["c1"][0], bore=0.0) - out["pins1"]
    return out


def stage2():
    out = {}
    pl, pins = [], []
    for k in range(3):
        a = np.radians(120 * k); x, y = A2 * np.cos(a), A2 * np.sin(a)
        p = spur(S2["zp"], S2["m"], *Z["s2"], bore=3.2).rotate([0, 0, 180.0 / S2["zp"] + np.degrees(a) * (S2["zs"] / S2["zp"])])
        pl.append(p.translate([x, y, 0]))
        pins.append(cyl(1.5, Z["c2"][1], Z["s2"][1] - 0.6, 16, x, y))
    out["planets2"] = mf.Manifold.batch_boolean(pl, mf.OpType.Add)
    out["pins2"] = mf.Manifold.batch_boolean(pins, mf.OpType.Add)
    c = cyl(A2 + 5.0, *Z["c2"])
    dflat = cyl(BRG["d"] / 2, Z["c2"][0] - 1, Z["c2"][1] + 1, 32) - mf.Manifold.cube([20, 20, 40]).translate([BRG["d"] / 2 - 0.8, -10, -100])
    out["carrier2"] = c - dflat - out["pins2"]
    return out


def shaft_and_bearings():
    z_top = Z["c2"][1]; z_bot = Z["web"][0] - 8.0
    sh = cyl(BRG["d"] / 2, z_bot, z_top, 32) - mf.Manifold.cube([20, 20, 200]).translate([BRG["d"] / 2 - 0.8, -10, -150])     # D-flat along the shaft
    b1 = ring(BRG["d"] / 2, BRG["D"] / 2, Z["nose"][0] + BRG["B"] + 1.0, Z["nose"][0] + 2 * BRG["B"] + 1.0)
    b2 = ring(BRG["d"] / 2, BRG["D"] / 2, Z["nose"][0], Z["nose"][0] + BRG["B"])
    nut = cyl(4.6, z_bot, z_bot + 4.0, 6) - cyl(2.5, z_bot - 1, z_bot + 5, 16)
    return {"shaft": sh, "bearings": b1 + b2, "nut": nut}


def wheel():
    """Dished wheel: PA-CF hub (boss on the D-shaft, web with six windows, drum rim r 51.5..54 that reaches back over
    the gearbox) and a 6 mm TPU tyre with 36 tread grooves.  Tyre centre plane over stage 2: short lever on the connector."""
    zw0, zw1 = Z["web"]; z0, z1 = Z["hub"]
    boss = cyl(WHEEL["r_hub"], zw0 - 4.0, zw1, 48)
    web = cyl(WHEEL["r_rim"] - 0.01, zw0, zw1)
    for k in range(WHEEL["spokes"]):
        a = np.radians(360 / WHEEL["spokes"] * (k + 0.5)); r = 33.0
        web -= cyl(10.5, zw0 - 1, zw1 + 1, 40, r * np.cos(a), r * np.sin(a))
    rim = ring(WHEEL["r_rim"] - WHEEL["rim_t"], WHEEL["r_rim"], z0, z1) + ring(WHEEL["r_rim"] - 0.01, WHEEL["r_rim"] + 1.5, z1 - 1.5, z1) + ring(WHEEL["r_rim"] - 0.01, WHEEL["r_rim"] + 1.5, z0, z0 + 1.5)
    bore = cyl(BRG["d"] / 2 + 0.05, zw0 - 5, zw1 + 1, 32) - mf.Manifold.cube([20, 20, 200]).translate([BRG["d"] / 2 - 0.8 + 0.05, -10, -150])
    hubpart = boss + web + rim - bore
    tyre = ring(WHEEL["r_rim"], WHEEL["r_tyre"], z0 + 1.5, z1 - 1.5) + ring(WHEEL["r_rim"] + 1.5, WHEEL["r_tyre"], z0, z0 + 1.51) + ring(WHEEL["r_rim"] + 1.5, WHEEL["r_tyre"], z1 - 1.51, z1)
    for k in range(36):
        tyre -= mf.Manifold.cube([3.0, 2.4, WHEEL["w"] + 2]).translate([WHEEL["r_tyre"] - 1.2, -1.2, z0 - 1]).rotate([0, 0, 10 * k])
    return {"wheel_hub": hubpart, "tyre": tyre}


def parts():
    out = {"hand_base": HB.hand_base(), "male_bullets": mf.Manifold.batch_boolean(HB.male_bullets(), mf.OpType.Add),
           "housing": housing(), "motor": motor()}
    out.update(stage1()); out.update(stage2()); out.update(shaft_and_bearings()); out.update(wheel())
    return out


RHO = dict(hand_base=1.15, housing=1.15, sun1=8.5, planets1=1.42, carrier1_sun2=1.25, planets2=1.42, carrier2=1.25, pins1=7.85, pins2=7.85,
           shaft=7.85, nut=7.85, wheel_hub=1.15, tyre=1.20)          # g/cm^3: PA-CF, brass sun, POM planets, CF resin carriers, steel, TPU


def checks():
    P = parts(); res = {}
    # gear geometry
    res["stage 1: centre distance / ring - sun check"] = [A1, S1["m"] * (S1["zr"] - S1["zp"]) / 2]
    res["stage 2: centre distance / ring - sun check"] = [A2, S2["m"] * (S2["zr"] - S2["zp"]) / 2]
    res["planet tip clearance to neighbours, stage 1 / 2 (mm)"] = [round(2 * A1 * np.sin(np.pi / 3) - (S1["zp"] + 2) * S1["m"], 2), round(2 * A2 * np.sin(np.pi / 3) - (S2["zp"] + 2) * S2["m"], 2)]
    # pairwise overlaps of everything that must not touch (meshing pairs are allowed only their backlash: tested separately)
    allowed = {("sun1", "motor"), ("pins1", "carrier1_sun2"), ("pins1", "planets1"), ("pins2", "carrier2"), ("pins2", "planets2"), ("shaft", "carrier2"), ("shaft", "bearings"),
               ("shaft", "wheel_hub"), ("shaft", "nut"), ("bearings", "housing"), ("hand_base", "male_bullets"), ("tyre", "wheel_hub"), ("nut", "wheel_hub")}
    names = list(P); bad = {}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if (a, b) in allowed or (b, a) in allowed: continue
            v = (P[a] ^ P[b]).volume()
            if v > 0.05: bad[f"{a} x {b}"] = round(v, 2)
    res["unexpected overlaps (mm^3)"] = bad
    # rotate every gear through one tooth pitch of the sun 1 and check meshing pairs stay apart (backlash, no interference)
    worst = {}
    for step in range(6):
        ang1 = 360.0 / S1["zs"] * step / 6
        sun1 = P["sun1"].rotate([0, 0, ang1])
        # carrier 1 turns 1/6 as fast; planets spin relative to it by -zs/zp of the sun's relative turn
        c1 = ang1 / (1 + S1["zr"] / S1["zs"]); rel = ang1 - c1; pl_spin = -rel * S1["zs"] / S1["zp"]
        pl = []
        for k in range(3):
            a = np.radians(120 * k)
            p = spur(S1["zp"], S1["m"], *Z["s1"], bore=2.2).rotate([0, 0, 180.0 / S1["zp"] + np.degrees(a) * (S1["zs"] / S1["zp"]) + pl_spin])
            pl.append(p.translate([A1 * np.cos(a), A1 * np.sin(a), 0]).rotate([0, 0, c1]))
        pl = mf.Manifold.batch_boolean(pl, mf.OpType.Add)
        worst["sun1 x planets1"] = max(worst.get("sun1 x planets1", 0), (sun1 ^ pl).volume())
        worst["planets1 x ring1 (housing)"] = max(worst.get("planets1 x ring1 (housing)", 0), (pl ^ P["housing"]).volume())
    res["meshing through one pitch: interference (mm^3, 0 = backlash kept)"] = {k: round(v, 3) for k, v in worst.items()}
    # stance numbers
    z_tyre = (Z["hub"][0] + Z["hub"][1]) / 2
    res["lever: socket mouth to tyre centre plane (mm)"] = round(-z_tyre, 1)
    res["ground clearance with the axle level: housing / collar under the wheel (mm)"] = [round(WHEEL["r_tyre"] - R_RING2, 1), round(WHEEL["r_tyre"] - HB.P["R_collar"], 1)]
    res["rim inner radius vs housing outer radius (mm)"] = [WHEEL["r_rim"] - WHEEL["rim_t"], round(R_RING2, 2)]
    res["length: socket mouth to the hub nut (mm)"] = round(-(Z["web"][0] - 8.0), 1)
    # masses
    mass = {k: round(P[k].volume() / 1000 * RHO[k], 1) for k in RHO}
    mass["motor (F60 Pro V 1750 kV, with leads)"] = M["m"]; mass["2 x 698-2RS"] = 2 * BRG["m"] * 1e3
    mass["3 x 4 mm bullets, M3 screws, wire, grease (est.)"] = 12.0
    mass["total"] = round(sum(mass.values()), 1)
    res["mass (g)"] = mass
    res["Z stations"] = Z
    for k, v in res.items(): print(f"  {k}: {v}")
    json.dump(res, open("wheelhand.json", "w"), indent=1, default=float)
    return res


def export(d="wheelhand_stl"):
    import trimesh
    os.makedirs(d, exist_ok=True)
    P = parts()
    for n in ("housing", "carrier1_sun2", "carrier2", "planets1", "planets2", "sun1", "wheel_hub", "tyre"):
        m = P[n]
        if n.startswith("planets"): m = m.decompose()[0]
        me = m.to_mesh(); t = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)
        t.export(f"{d}/wheelhand_{n}.stl")
        print(f"  {n:15s} watertight {t.is_watertight} bodies {len(m.decompose())} bbox {np.round(t.extents, 1)}")


if __name__ == "__main__":
    checks(); export()
