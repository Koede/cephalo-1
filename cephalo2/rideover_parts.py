"""Ride-over clutch cams + fall reinforcement parts (2026-11-11), replacing the single-V cams with hard stops of clutch set v1.

Why: in a tumbling fall the arm links swing far past the cam's +-40 deg (+-25 deg root) travel. At the printed stop
the link's momentum went straight into the servo gears (3-4.6 x the assumed strip limit in fall4.py). A ride-over cam
has a V every 2 (travel + 5 deg): past the crest the roller drops into the next V instead of hitting a stop, so the
torque into the gears can never exceed the crest torque. The link then stops on its own shell stop (TPU pad), which
the servo does not feel through the gear train.

Parts (python3 rideover_parts.py -> stl_rideover/ + rideover_parts.json + rideover_cams.png):
  cam_rideover_STS3215     j2-j7: 4 notches (90 deg pitch), rise 10 mm/rad over +-40 deg, rounded crest at +-45 deg
  cam_rideover_STS3250     j0 and j1: 6 notches (60 deg pitch), rise 16 mm/rad over +-25 deg, crest at +-30 deg
  rocker_j1_STS3250        j1 (new clutch, 0.9 x stall at 60 N): root rocker geometry with its own lever ratio
  hall_clip                holds an Adafruit #158 US5881LUA Hall switch facing the cam face (home-notch sensor)
  stop_pad_link            TPU 95A pad on each link-to-link stop face (j1-j7)
  stop_pad_root            TPU 95A pad on the root link's stop face against the mantle (j0)
Home-notch sensing: one 3 x 2 mm N52 disc magnet pressed into each cam face at the home V; the Hall switch on the
link sees it only while the roller sits in the home V (+-15 deg). US5881 runs from 3.5-24 V: power it from 5 V, its
open-drain output pulled up to 3.3 V. The 8 switches of an arm go to an Adafruit MCP23017 breakout (#5346, I2C 0x20)
on the tip XIAO's I2C bus (shared with the VL53L4CD at 0x29).
"""
import json, os
import numpy as np, trimesh, manifold3d as m3d

ALPHA = np.radians(90); T_FIRM = 60.0
HORN_BC, HORN_HOLE, HORN_BORE = 14.0, 2.9, 8.5         # STS3215 horn (placeholder, measure)
GAP = np.radians(5.0)                                  # crest blend beyond the working travel


def profile(R0, k_mm, travel, n_notch, n=1440):
    """Polar cam profile: n_notch V's; inside +-travel the radius rises k_mm per rad (constant slip torque), then a
    parabolic blend to a zero-slope crest halfway between notches."""
    P = 2 * np.pi / n_notch
    assert abs(P / 2 - (travel + GAP)) < 1e-6, "pitch must equal 2 (travel + gap)"
    pts = []
    for a in np.linspace(-np.pi, np.pi, n, endpoint=False):
        psi = abs((a + P / 2) % P - P / 2)
        if psi <= travel:
            r = R0 + k_mm * psi
        else:
            u = psi - travel
            r = R0 + k_mm * travel + k_mm * u - k_mm * u * u / (2 * GAP)
        pts.append((r * np.cos(a), r * np.sin(a)))
    return pts


def cam(R0, k_mm, travel, n_notch, thick, bore, holes, mag_r):
    body = m3d.Manifold.extrude(m3d.CrossSection([profile(R0, k_mm, travel, n_notch)]), thick)
    body = body - m3d.Manifold.cylinder(thick + 2, bore / 2, bore / 2, 48).translate((0, 0, -1))
    for (r, d) in holes:
        for i in range(4):
            a = np.pi / 4 + i * np.pi / 2
            body = body - m3d.Manifold.cylinder(thick + 2, d / 2, d / 2, 24).translate((r * np.cos(a), r * np.sin(a), -1))
    # magnet pocket (3 x 2 mm N52 disc, press fit) on the top face at the home notch (angle 0)
    body = body - m3d.Manifold.cylinder(2.3, 1.6, 1.6, 32).translate((mag_r, 0, thick - 2.2))
    # home-notch witness mark: small groove on the rim face so the home V can be found by eye
    body = body - m3d.Manifold.cube((1.0, 1.0, 1.0)).translate((mag_r + 2.5, -0.5, thick - 0.6))
    return body


def lighten(body, rm, rr, thick, n_notch, skip=()):
    for j in range(n_notch):
        a = (j + 0.5) * 2 * np.pi / n_notch                  # under the crests (most material)
        body = body - m3d.Manifold.cylinder(thick + 2, rr, rr, 32).translate((rm * np.cos(a), rm * np.sin(a), -1))
    return body


def joint_params(stall, frac, k_mm, l_roller):
    M = frac * stall; P = M / (k_mm / 1000); Fl = 2 * T_FIRM * np.sin(ALPHA / 2); ratio = P / Fl
    return dict(M=M, k_cam_mm=k_mm, P=P, ratio=ratio, l_roller=l_roller, l_idler=ratio * l_roller)


def rocker_root(p, thick=9.0):
    """As clutch_parts.rocker_root: 626ZZ roller (6x19x6) on M6, M3 shoulder-bolt pivot, 623ZZ idler on M3."""
    lr, li = p["l_roller"], p["l_idler"]
    disc = lambda x, r: m3d.CrossSection.circle(r, 48).translate((x, 0))
    cs = m3d.CrossSection.hull(m3d.CrossSection.batch_boolean([disc(-lr, 11.5), disc(0, 5.5), disc(li, 7.5)], m3d.OpType.Add))
    body = m3d.Manifold.extrude(cs, thick)
    for x, r in ((-lr, 3.15), (0.0, 1.65), (li, 1.65)):
        body = body - m3d.Manifold.cylinder(thick + 2, r, r, 32).translate((x, 0, -1))
    body = body - m3d.Manifold.cylinder(6.2, 9.55, 9.55, 64).translate((-lr, 0, thick - 6.1))
    body = body - m3d.Manifold.cylinder(4.2, 5.05, 5.05, 64).translate((li, 0, thick - 4.1))
    if li > 30:
        body = body - m3d.Manifold.cube((li - 22, 5, thick + 2)).translate((8, -2.5, -1))
    return body


def hall_clip():
    """Clip for a US5881LUA (TO-92UA: 4.1 x 3.0 x 1.5 mm body, 3 leads): branded face toward the cam face, 1.0 mm
    window; M2 screw to the link cradle. 12 x 9 x 6 mm."""
    b = m3d.Manifold.cube((12, 9, 6))
    b = b - m3d.Manifold.cube((4.4, 1.8, 3.3)).translate((2.0, 3.6, 6 - 3.3 + 0.01))       # sensor body pocket
    b = b - m3d.Manifold.cube((4.4, 1.0, 2.0)).translate((2.0, 3.6 + 1.8, 6 - 2.0 + 0.01))  # lead exit
    b = b - m3d.Manifold.cube((3.0, 1.0, 1.0)).translate((2.7, -0.01, 6 - 1.6))            # wire channel
    b = b - m3d.Manifold.cylinder(8, 1.1, 1.1, 24).translate((9.5, 4.5, -1))              # M2 (self-tap into PETG)
    return b


def stop_pad(L, W, H, hole=3.4, cs=6.4):
    """TPU 95A stop pad, 2 mm chamfer on the strike face, M3 countersunk screw into a heat-set insert."""
    pad = m3d.Manifold.cube((L, W, H))
    for s in (0, 1):
        wedge = m3d.Manifold.cube((L + 2, 3, 3)).rotate((45, 0, 0)).translate((-1, s * W - 0, H - 0.0))
        pad = pad - wedge
    pad = pad - m3d.Manifold.cylinder(H + 2, hole / 2, hole / 2, 24).translate((L / 2, W / 2, -1))
    pad = pad - m3d.Manifold.cylinder(1.6, hole / 2, cs / 2, 24).translate((L / 2, W / 2, H - 1.59))
    return pad


def to_tm(man):
    mesh = man.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=True)


def build():
    parts = {}
    c215 = cam(14.0, 10.0, np.radians(40), 4, 6.0, HORN_BORE, [(HORN_BC / 2, HORN_HOLE + 0.3)], 11.0)
    parts["cam_rideover_STS3215"] = (lighten(c215, 17.0, 2.2, 6.0, 4), 6, "PETG", "j2-j7 (replaces clutch_cam_STS3215)")
    c250 = cam(18.0, 16.0, np.radians(25), 6, 7.0, 10.0, [(9.0, 3.2)], 14.5)
    parts["cam_rideover_STS3250"] = (lighten(c250, 21.5, 2.4, 7.0, 6), 2, "PETG", "j0 + j1 (replaces clutch_cam_STS3250_root; j1 new)")
    pj1 = joint_params(4.9, 0.9, 16.0, 10.0)
    parts["rocker_j1_STS3250"] = (rocker_root(pj1), 1, "PETG", "j1 (new clutch)")
    parts["hall_clip"] = (hall_clip(), 8, "PETG", "every clutched joint")
    parts["stop_pad_link"] = (stop_pad(18, 12, 5), 14, "TPU 95A", "j1-j7, both directions")
    parts["stop_pad_root"] = (stop_pad(28, 18, 8), 2, "TPU 95A", "j0 up/down stop against the mantle")
    os.makedirs("stl_rideover", exist_ok=True)
    rep = {}
    for nm, (man, q, mat, use) in parts.items():
        tm = to_tm(man); tm.export(f"stl_rideover/{nm}.stl")
        dens = 1.27 * 0.65 if mat == "PETG" else 1.21 * 1.0
        rep[nm] = dict(qty_per_arm=q, material=mat, use=use, watertight=bool(tm.is_watertight),
                       extents_mm=np.round(tm.extents, 1).tolist(), mass_g=round(tm.volume / 1000 * dens, 2))
    def crest(R0, k, tr): return R0 + k * tr + k * GAP / 2
    rep["_cam_data"] = {
        "STS3215 j2-j7": dict(notches=4, pitch_deg=90, travel_deg=40, crest_deg=45, R0_mm=14, rise_mm_per_rad=10,
                              crest_rise_mm=round(crest(14, 10, np.radians(40)) - 14, 2)),
        "STS3250 j0/j1": dict(notches=6, pitch_deg=60, travel_deg=25, crest_deg=30, R0_mm=18, rise_mm_per_rad=16,
                              crest_rise_mm=round(crest(18, 16, np.radians(25)) - 18, 2)),
        "j1 rocker (0.9 x 4.9 N m at 60 N)": {k: round(float(v), 2) for k, v in pj1.items()}}
    json.dump(rep, open("stl_rideover/rideover_parts.json", "w"), indent=1)
    return rep


def figure():
    """Old single-V cam with stop vs new ride-over cam, both STS3215, roller path and torque vs slip angle."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(15, 5.2))
    a = np.linspace(-np.pi, np.pi, 1441)
    old = []
    for x in a:
        ph = abs(x); pm = np.radians(40)
        r = 14 + 10 * ph if ph <= pm else (21.0 + 4.0 if ph <= pm + np.radians(12) else 25.0 - 2.0 * min(1, (ph - pm - np.radians(12)) / np.radians(20)))
        old.append(r)
    new = np.array(profile(14.0, 10.0, np.radians(40), 4, n=1441))
    for k, (title, xy) in enumerate([("rev 4 cam: one V, printed stop at 40 deg", np.c_[np.array(old) * np.cos(a), np.array(old) * np.sin(a)]),
                                     ("ride-over cam: 4 V's, crest at 45 deg", new)]):
        A = ax[k]; A.fill(xy[:, 0], xy[:, 1], color="#e8954a", alpha=0.9); A.plot(xy[:, 0], xy[:, 1], "k", lw=0.8)
        A.add_patch(plt.Circle((0, 0), 4.25, color="w")); A.add_patch(plt.Circle((14 + 6.5, 0), 6.5, fill=False, lw=1.5, color="#3060c0"))
        A.text(14 + 6.5, -9, "roller\n(624ZZ)", ha="center", fontsize=9, color="#3060c0")
        if k == 1: A.plot([11], [0], "o", ms=6, color="#c03030"); A.text(11, 2.2, "magnet", ha="center", fontsize=8, color="#c03030")
        A.set_aspect("equal"); A.set_title(title, fontsize=11); A.set_xlim(-30, 30); A.set_ylim(-30, 30); A.axis("off")
    phi = np.linspace(-150, 150, 1201); tb = 2.65
    t_old = np.where(np.abs(phi) <= 40, tb * np.sign(phi), np.nan)
    psi = (phi + 45) % 90 - 45
    t_new = np.where(np.abs(psi) <= 40, tb * np.sign(psi), tb * np.sign(psi) * (1 - (np.abs(psi) - 40) / 5))
    A = ax[2]
    A.plot(phi, t_old + 0.12, lw=4, color="#999", label="rev 4: hard stop at 40 deg -> link momentum into the gears")
    for sg in (-1, 1):
        A.plot([40 * sg, 40 * sg], [2.77 * sg, 10 * sg], lw=4, color="#999")
        A.annotate("stop: unbounded", (40 * sg, 9.3 * sg), xytext=(70 * sg - 22, 6.0 * sg), fontsize=8, color="#555",
                   arrowprops=dict(arrowstyle="->", color="#555"))
    A.plot(phi, t_new, lw=1.8, color="#e8954a", label="ride-over: torque never exceeds the slip torque")
    A.axhline(8.82, color="#c03030", ls="--", lw=1); A.text(-10, 8.0, "assumed gear strip limit (3 x stall)", fontsize=8, color="#c03030")
    A.set_xlabel("clutch slip (link vs horn), deg"); A.set_ylabel("torque on the servo output, N m (firm winch)")
    A.set_ylim(-10, 12.5); A.legend(fontsize=7.5, loc="upper left"); A.grid(alpha=0.3)
    A.set_title("torque into the gear train vs slip", fontsize=11)
    fig.tight_layout(); fig.savefig("rideover_cams.png", dpi=110)


if __name__ == "__main__":
    print(json.dumps(build(), indent=1)); figure()
