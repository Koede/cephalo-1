#!/usr/bin/env python3
"""Hippos-1 front sling: options study (2026-11-27).
The front sling lets each front leg's carrier (the two hip brackets that hold the abduction shaft, abduction motor and
the whole leg) move up and down against a spring, like the muscle sling that hangs a horse's chest between its shoulders.
Rev C has it open: the carriers are rigid on the body, and the joint tree has an unused trailing "rocker" node.

Per option this computes, for each load case on one front leg: the spring / shock force, how far the sling moves, how much
fore-aft hoof force (braking, driving) moves it ("coupling"), and the forces in whatever guides it.  Geometry comes from the
rev C CAD (carrier, pivot, neighbouring hind stack).  python3 sling_study.py -> sling_study.json, sling_options.png"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.dirname(HERE))
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import hippos_legs_cad as C

G = 9.81
M_TOT = 41.65                       # Centauri-1, rev C (kg)
SHARE_F = 0.60                      # front share of the weight (study)
M_LEG_F = 5.196                     # one front leg incl. carrier, rev C CAD (moves with the sling)
PIV = C.LEG["fore"]["pivot"]        # world (mm): x 236, y 178.5, z 467.5
H_PIV = PIV[2]                      # pivot above the ground
W_LEG = M_TOT * G * SHARE_F / 2     # standing load on one front hoof
M_SPRUNG = M_TOT * SHARE_F / 2 - M_LEG_F

# carrier footprint (rev C, rest pose, relative to the leg pivot): brackets at x -254 (rear, abduction motor) and +98 (front)
X_BR = (-254.0 + -246.0) / 2, (88.0 + 98.0) / 2
GAP_TO_HIND = 19.0                   # world x: front carrier ends at -18, hind upper stack starts at -37

MT = json.load(open(os.path.join(os.path.dirname(HERE), "legs_motion_test.json")))
GRF = {g: MT["forces"][g]["grf_peak_N"]["fore"] for g in ("walk", "trot", "gallop")}
# load cases on one front hoof: Fz up (N), Fx along +x (forward, N), hoof x from the pivot (mm).  Fy for roll.
CASES = {
    "stand":                 dict(Fz=W_LEG,       Fx=0.0,   x=0.0,   Fy=0.0),
    # peaks from the rev C motion test (legs_motion_test.json, grf_peak_N fore); fore-aft share and hoof position est.
    "walk peak":             dict(Fz=GRF["walk"], Fx=-0.10 * GRF["walk"], x=60.0, Fy=0.10 * GRF["walk"]),
    "trot peak":             dict(Fz=GRF["trot"], Fx=-0.20 * GRF["trot"], x=80.0, Fy=0.15 * GRF["trot"]),
    "gallop peak":           dict(Fz=GRF["gallop"], Fx=-0.35 * GRF["gallop"], x=120.0, Fy=0.15 * GRF["gallop"]),
    "leap landing, fores":   dict(Fz=450.0, Fx=-135.0, x=100.0, Fy=60.0),
    "design max (grip 0.7)": dict(Fz=750.0, Fx=-525.0, x=150.0, Fy=225.0),
}

# spring target (per front leg, at the hip): 25 mm travel, 5 mm static sag, gallop peak near the end of travel
TRAVEL = 25.0
SAG = 5.0
K_W = (CASES["gallop peak"]["Fz"] - W_LEG) / (TRAVEL - SAG - 1.0) * 1.0   # N/mm, linear past the preload
PRELOAD = W_LEG - K_W * SAG
F_N = np.sqrt(K_W * 1e3 / M_SPRUNG) / (2 * np.pi)


def defl(F):            # mm at the hip, linear spring with preload, bump stop at TRAVEL
    return float(np.clip((F - PRELOAD) / K_W, 0, TRAVEL))


def m_pivot(c):          # moment of the ground force about the leg pivot (about +y), N m; hoof at (x, -H_PIV)
    return (c["Fz"] * c["x"] + (-c["Fx"]) * H_PIV) / 1e3


def trailing(c, piv, mr):
    """Rigid arm about a y axis at piv (x, z rel. leg pivot).  Generalised force at the hip = M about piv / arm length."""
    rx, rz = c["x"] - piv[0], -H_PIV - piv[1]
    M = c["Fz"] * rx + c["Fx"] * rz                 # N mm, + compresses
    arm = -piv[0]
    Fh = M / arm                                    # equivalent vertical load at the hip
    return dict(F_hip=Fh, shock=Fh * mr, coupling=abs(rz / arm))


def options():
    O = {}
    # A: vertical slides at both brackets, coil-over between body and carrier
    sp = X_BR[1] - X_BR[0]
    O["A"] = dict(name="Twin vertical slides at the two hip brackets + one coil-over",
                  how="each bracket rides on a short vertical guide fixed to the body; the carrier moves straight up and down; one coil-over (vertical, inboard of the carrier) carries the load",
                  rows={k: dict(shock=c["Fz"], defl=defl(c["Fz"]), coupling=0.0,
                                guide=abs(m_pivot(c)) * 1e3 / sp + abs(c["Fx"]) / 2, roll_Nm=c["Fy"] * H_PIV / 1e3) for k, c in CASES.items()},
                  guide_note=f"pitch moment taken as a couple over the {sp:.0f} mm bracket spacing",
                  parts="2 x MGN12H or MGN15H rail + 2 blocks per bracket (or 12 mm rod + 2 igus plain bushes), 1 coil-over, urethane bump stop",
                  mass=0.42, cost=110, fits="inside the carrier's own length; needs about 40 mm of body frame inboard of the brackets for rails")
    # B: short parallelogram above the abduction motor
    s = 90.0
    O["B"] = dict(name="Parallelogram links (4-bar) over the abduction motor + coil-over on a bell crank",
                  how="two 100 mm links, one above the other, hang the carrier from the body; it moves on a 100 mm arc that stays level",
                  rows={k: dict(shock=c["Fz"], defl=defl(c["Fz"]), coupling=0.0,
                                guide=abs(m_pivot(c) + c["Fz"] * (-X_BR[0]) / 1e3) * 1e3 / s, roll_Nm=c["Fy"] * H_PIV / 1e3) for k, c in CASES.items()},
                  guide_note=f"link force = moment about the links' end / {s:.0f} mm spacing",
                  parts="2 carbon or aluminium links with 4 rod ends (M8), bell crank on 2 x 6800, 1 coil-over",
                  mass=0.48, cost=95, fits="needs a 100 mm x 90 mm window above the abduction motor; the yaw stack sweeps there (check)")
    # C: trailing rocker, Nov 19 assumption (pivot as in the CAD tree, x -310 z -70)
    piv = np.array([-310.0, -70.0])
    O["C"] = dict(name="Trailing rocker + coil-over (the Nov 19 assumption)",
                  how="the carrier is the end of an arm that swings about a pivot 310 mm behind the hip",
                  rows={k: (lambda t: dict(shock=t["shock"], defl=defl(t["F_hip"]), coupling=t["coupling"], guide=None,
                                           roll_Nm=c["Fy"] * H_PIV / 1e3))(trailing(c, piv, 4.5)) for k, c in CASES.items()},
                  guide_note="", parts="rocker arm on 2 x 6802, coil-over at 4.5:1", mass=0.55, cost=85,
                  fits=f"pivot lands inside the hind leg stack (front carrier ends {GAP_TO_HIND:.0f} mm from it): would have to move inboard")
    # D: leading rocker to the chest
    piv = np.array([150.0, 20.0])
    O["D"] = dict(name="Leading rocker hung from the chest + coil-over",
                  how="the arm pivots on the chest 150 mm ahead of the hip and swings back to the carrier",
                  rows={k: (lambda t: dict(shock=t["shock"], defl=defl(t["F_hip"]), coupling=t["coupling"], guide=None,
                                           roll_Nm=c["Fy"] * H_PIV / 1e3))(trailing(c, piv, 1.5)) for k, c in CASES.items()},
                  guide_note="", parts="rocker on 2 x 6802, coil-over at 1.5:1", mass=0.50, cost=85,
                  fits="room at the chest under the pod; short arm")
    # E: elastomer
    O["E"] = dict(name="Rubber / urethane mounts under both brackets",
                  how="each bracket sits on two bonded rubber mounts; no guide, the rubber carries everything",
                  rows={k: dict(shock=c["Fz"], defl=min(5.0, c["Fz"] / (W_LEG / 2.0)), coupling=0.4, guide=None, roll_Nm=c["Fy"] * H_PIV / 1e3)
                        for k, c in CASES.items()},
                  guide_note="", parts="4 x M6 cylindrical rubber mounts (55 Shore A) per side", mass=0.10, cost=20,
                  fits="fits as is; about 5 mm of motion")
    # F: flexure leaves
    O["F"] = dict(name="Parallel spring-steel leaves (flexure)",
                  how="two flat spring-steel leaves, one above the other, hang each bracket: the leaves are spring and guide at once",
                  rows={k: dict(shock=c["Fz"], defl=min(15.0, defl(c["Fz"])), coupling=0.0, guide=abs(m_pivot(c)) * 1e3 / 100.0,
                                roll_Nm=c["Fy"] * H_PIV / 1e3) for k, c in CASES.items()},
                  guide_note="travel cut to 15 mm: 25 mm puts 750 MPa in a 200 mm leaf", parts="4 leaves 1095 spring steel 200 x 40 x 2 mm, clamps",
                  mass=0.30, cost=40, fits="200 mm leaves along the body: no room behind the carrier (hind stack)")
    # G: one shared heave sling
    O["G"] = dict(name="One shared sling for both shoulders (heave only, roll held by links)",
                  how="left and right carriers are tied through a rocker to one shock: the chest drops between both shoulders, as in a horse; a leg loaded alone moves both",
                  rows={k: dict(shock=c["Fz"], defl=defl(c["Fz"] / 2) , coupling=0.0, guide=abs(m_pivot(c)) * 1e3 / (X_BR[1] - X_BR[0]), roll_Nm=c["Fy"] * H_PIV / 1e3) for k, c in CASES.items()},
                  guide_note="as A, plus a cross rocker", parts="A's guides + cross rocker + 1 shock", mass=0.70, cost=150,
                  fits="the cross rocker crosses the body under the neck sheath block: room not known")
    # H: none (virtual)
    O["H"] = dict(name="No sling: rigid brackets, give from the leg motors",
                  how="as built; the body bobs only as much as the leg motors are told to give",
                  rows={k: dict(shock=0.0, defl=0.0, coupling=0.0, guide=None, roll_Nm=c["Fy"] * H_PIV / 1e3) for k, c in CASES.items()},
                  guide_note="", parts="none", mass=0.0, cost=0, fits="as built")
    return O


O = options()
out = dict(basis=dict(M_TOT=M_TOT, SHARE_F=SHARE_F, W_LEG=W_LEG, M_SPRUNG=M_SPRUNG, K_W=K_W, PRELOAD=PRELOAD, F_N=F_N, TRAVEL=TRAVEL, SAG=SAG,
                      pivot_world=PIV.tolist(), bracket_x=X_BR, gap_to_hind=GAP_TO_HIND, cases=CASES), options=O)
json.dump(out, open(os.path.join(HERE, "sling_study.json"), "w"), indent=1, default=float)
print(f"front leg stand {W_LEG:.0f} N, sprung {M_SPRUNG:.1f} kg, wheel rate {K_W:.1f} N/mm, preload {PRELOAD:.0f} N, bounce {F_N:.1f} Hz")
for k, o in O.items():
    r = o["rows"]
    print(f"\n{k}: {o['name']}  (+{o['mass']:.2f} kg/side, ~${o['cost']}/side)")
    for c, v in r.items():
        g = "" if v.get("guide") is None else f"  guide {v['guide']:.0f} N"
        print(f"   {c:24s} spring {v['shock']:6.0f} N  moves {v['defl']:5.1f} mm  coupling {v['coupling']:.2f}{g}")
