"""Chilo-1 body design study (3D sketches, no CAD): shell shape, battery packaging, dock-hub placement.
python3 chilo_body_study.py -> sheet_chilo_body.png"""
import numpy as np, sk3d
import chilo_shell as SH
C = sk3d.COL
SHELL, SHELL2, PACKC, HUBC = "#2a6f73", "#3c8f93", C["pack"], "#cfd3d8"
TV, TF = SH.tergite()


def legs(sc, x, yc, h=0.14, dy=0.14, col="#e07a22"):
    for s in (1, -1):
        hip = np.array([x, s * yc, 0]); knee = np.array([x, s * (yc + 0.05 + 0.105), 0.01]); foot = np.array([x, s * (yc + dy), -h])
        sc.box(hip + [0, s * 0.03, 0], (0.025, 0.045, 0.035), fc=C["act"], sub=1)
        sc.tube([hip + [0, s * 0.05, 0], knee, foot], 0.010, col)
        sc.sphere(foot, 0.013, C["act"], n=5)


def box_body(sc, n=3, pitch=0.12):
    for i in range(n):
        x = -pitch * i
        sc.box((x, 0, 0), (0.084, 0.112, 0.060), fc=SHELL)
        sc.box((x - 0.004, 0, 0.033), (0.108, 0.136, 0.006), fc=SHELL2)
        legs(sc, x, 0.06)


def tergite_body(sc, n=3, pitch=0.12, pack=True):
    for i in range(n):
        x = -pitch * i
        V = TV + [x, 0, 0]
        sc.mesh(V, TF, SHELL)
        PV, PF = SH.sternite(); sc.mesh(PV + [x, 0, 0], PF, SHELL2)
        if pack:
            sz, c = SH.pack_box(); sc.box(np.array(c) + [x, 0, 0], sz, fc=PACKC, sub=1)
        legs(sc, x, 0.06)


def thorax(sc, x0=-0.30, hub=True, rider=False):
    # fused double segment: wider carapace (same loft stretched), 2 packs in the corners, hub on a 40 mm pedestal
    V = TV.copy(); V[:, 0] *= 1.65; V[:, 1] *= 1.42; V[:, 2] *= 1.17
    sc.mesh(V + [x0, 0, 0.005], TF, SHELL)
    sc.box((x0, 0, -0.028), (0.20, 0.13, 0.014), fc=SHELL2, sub=1)
    for sx, sy in ((1, 1), (-1, -1)):
        sc.box((x0 + sx * 0.075, sy * 0.058, -0.006), (0.035, 0.075, 0.030), fc=PACKC, sub=1)
    for xo in (0.06, -0.06):
        legs(sc, x0 + xo, 0.085, dy=0.115)
    if hub:
        zt = 0.035 + 0.005; zc = 0.075
        sc.cyl((x0, 0, zt), (x0, 0, zc - 0.015), 0.090, HUBC, n=28)                      # pedestal
        sc.ring((x0, 0), 0.100, 0.118, zc - 0.015, zc, "#9aa0a8")                        # collar
        sc.cyl((x0, 0, zc - 0.034 - 0.01), (x0, 0, zc - 0.034), 0.0475, C["act"], n=24)   # hub pad (centring cones)
        for k in range(4):
            a = np.pi / 2 * k
            sc.box((x0 + 0.07 * np.cos(a), 0.07 * np.sin(a), zc - 0.041), (0.018, 0.044, 0.082), sk3d.rot("z", np.degrees(a)), C["act"], sub=1)
        for k in range(4):
            a = np.pi / 4 + np.pi / 2 * k
            sc.cyl((x0 + 0.026 * np.cos(a), 0.026 * np.sin(a), zc - 0.034), (x0 + 0.026 * np.cos(a), 0.026 * np.sin(a), zc - 0.027), 0.009, C["latch"], n=10, r1=0.002)
    if rider:
        b = np.array([x0, 0, 0.075]); Z = np.array([0, 0, 1.0])
        sc.cyl(b, b + 0.006 * Z, 0.125, C["frame"], n=32); sc.dome(b + 0.006 * Z, 0.125, 0.150, C["octo"], n=9)
        sc.cyl(b + 0.100 * Z, b + 0.224 * Z, 0.090, C["octo2"], n=28)
        for k in range(4):
            a = np.pi / 2 * k
            sc.box((x0 + 0.07 * np.cos(a), 0.07 * np.sin(a), 0.075 - 0.035), (0.013, 0.037, 0.070), sk3d.rot("z", np.degrees(a)), C["bar"], sub=1)
        for i in range(8):
            a = np.radians(22.5 + 45 * i); er = np.array([np.cos(a), np.sin(a), 0]); root = b + 0.125 * er + 0.02 * Z
            sc.tube([root, root + 0.15 * er + 0.03 * Z, root + 0.24 * er + 0.15 * Z, root + 0.24 * er + 0.28 * Z], 0.016, C["octo2"])


fig = sk3d.figure(16, 10, "Chilo-1 body study: shell, battery packaging, dock hub for Cephalo-1 (sketches, masses est.)")
# ---- row 1: shells
for k, (title, fn) in enumerate((("A  box segments (rev 1 model)", lambda sc: box_body(sc)),
                                 ("B  tergite roof-tile shell, 1 pack per segment", lambda sc: tergite_body(sc)),
                                 ("C  = B + fused thorax carrying the dock hub  (chosen)", lambda sc: (tergite_body(sc, n=2), thorax(sc, x0=-0.30))))):
    sc = sk3d.Scene(fig, [0.0 + 0.33 * k, 0.50, 0.34, 0.42], elev=22, azim=-55, title=title)
    sc.ground((-0.55, -0.3), (0.15, 0.3), z=-0.14)
    fn(sc)
    sc.done((-0.55, -0.3, -0.14), (0.15, 0.3, 0.16))
# ---- row 2: dock placement
for k, (title, x0, rider) in enumerate((("H1  hub on the head segment: the octopus as the head", 0.0, True),
                                        ("H2  hub on the thorax (segments 2+3 fused): rider over the middle legs  (chosen)", -0.30, True))):
    sc = sk3d.Scene(fig, [0.0 + 0.345 * k, 0.04, 0.36, 0.44], elev=18, azim=-62, title=title)
    sc.ground((-0.75, -0.4), (0.3, 0.4), z=-0.14)
    for i in range(6):
        x = -0.12 * i
        if x0 == -0.30 and i in (2, 3): continue
        sc.mesh(TV + [x, 0, 0], TF, SHELL); legs(sc, x, 0.06)
    if x0 == -0.30: thorax(sc, -0.30, rider=rider)
    else:
        thorax(sc, 0.0, rider=rider)
    sc.done((-0.75, -0.4, -0.14), (0.3, 0.4, 0.45))
sk3d.panel_text(fig, 0.70, 0.47, [
    "Shell: each tergite is a PETG superellipse loft, 138 mm long,",
    "overlapping the next by 18 mm (roof tiles), with a lateral flare",
    "roofing the coxa servos; a sternite pan carries the servos and",
    "one 6S 1300 mAh pack (75 x 35 x 30, 200 g est.).  75 + 40 g per",
    "segment vs 120 g box.  A 6S 5000 pack (155 mm) does not fit any",
    "120 mm segment: 6 x 1300 mAh = 7.8 Ah total, 1.2 kg (-0.42 kg).",
    "",
    "Dock hub = HOST side of the Scabbard dock (scabbard.py):",
    "collar r 100-118 (plate seat), hub pad r 47.5 with 4 centring",
    "cones at r 26, 4 guest sockets 36 x 9.6 at r 65-74.6, 82 deep,",
    "dia 6 latch pins + STS3032 release ring.  PETG, ~400 g est.",
    "The socket circle (r 75) is wider than a 120 mm segment, so the",
    "hub needs the 170 mm thorax and a 40 mm pedestal.",
    "",
    "H1 puts 12.8 kg on the two front leg pairs and the first spine",
    "pitch joint (above STS3250 stall).  H2 spreads it over 4 leg",
    "pairs and no spine joint carries it.  Cost: one spine joint",
    "(4 left), which also solves the long-pack problem.",
    "",
    "Docked walk (MuJoCo, 20.8 kg): stance 200 mm high, feet 100 mm",
    "out; STS3215 tibias ran at 1.45 x rated rms -> tibias are now",
    "STS3250 (+0.23 kg): 0.65 x stall peak, 1.03 x rated rms.",
], fs=9.6, title="Notes and loads")
sk3d.save(fig, "sheet_chilo_body.png")
