"""Hippos-1 tail built from the Cephalo-2 arm modules (study, 2026-11-16).
Arm makeup (cephalo4 rev 4): servo-in-link joints p-y-p-y..., 56 mm pitch, first joint 50 mm from the root face, spigot 36 mm
past the last joint (L = 30 + 56 n), STS3250 at j0, STS3215 beyond, detent clutches on j0 and j2.. (j1 rigid), winch line.
Root face = the tail_adapter servo flange (z_c -35), so j0 sits 85 mm behind the SFF seat.  Options: n = 6 / 7 / 8 joints,
tip = bare SFF socket (swappable hands; checked with the old 60 g hand + 0.3 kg) or hand3 (426 g with connector, COM 69 mm).
Statics with the cephalo4 masses."""
import sys, json
sys.path.insert(0, "/home/claude/octodrone/ceph2")
import numpy as np
import cephalo4 as C
G = 9.81; MM = 1e-3
SEAT, FLANGE = 3.8, -35.0
SV = {"STS3215": dict(rated=0.981, stall=2.94, m=0.055, I0=0.18, Is=2.7), "STS3250": dict(rated=1.63, stall=4.90, m=0.0745, I0=0.28, Is=4.2),
      "STS3095": dict(rated=2.21, stall=9.32, m=0.1945, I0=0.6, Is=9.8)}
M_MOUNT = 0.050                       # rump socket + hand base + tail adapter (printed share, tailmount.py)
M_WINCH = 0.024                       # N20 worm winch + block (cephalo4 M_WINCH), at the root
def option(n, tip):
    servo = ["STS3250"] + ["STS3215"] * (n - 1); typ = "pypypypy"[:n]
    sj = (50 + 56 * np.arange(n)) * MM; L = (30 + 56 * n) * MM
    pts = []                                                     # (s from the root face, mass)
    for k, nm in enumerate(servo):
        pts.append((sj[k] - (45.2 / 2 - 11.4) * MM, SV[nm]["m"]))
        pts.append((sj[k] - 0.002, 0.006 + (C.M_CLUTCH[nm] if C.CLUTCH_FRAC[k] > 0 else 0))); pts.append((sj[k] + 0.002, 0.006))
    for x in np.linspace(0, L, 20)[:-1] + L / 38: pts.append((x, (C.LAM * L + C.M_LINE * L / 0.478) / 19))
    pts.append((L - 0.012, C.M_TIPBAY))
    if tip == "hand3": pts.append((L + 0.0099 + 0.059, 0.426)); reach_tip = L + 0.0099 + 0.149
    else: pts.append((L + 0.005, 0.016)); pts.append((L + 0.0099 + 0.040, 0.060 + 0.300)); reach_tip = L + 0.0109
    s = np.array([p[0] for p in pts]); m = np.array([p[1] for p in pts])
    d0 = (SEAT - FLANGE) * MM                                    # seat to root face
    # held straight back, level: pitch joints carry the moment of everything outboard; yaw joints carry none
    tau = np.array([(m[s > sj[k]] * G * (s[s > sj[k]] - sj[k])).sum() if typ[k] == "p" else 0.0 for k in range(n)])
    rated = np.array([SV[x]["rated"] for x in servo]); stall = np.array([SV[x]["stall"] for x in servo])
    M_seat = (m * G * (s + d0)).sum() + (M_WINCH + 0.040) * G * 0.02 + M_MOUNT * G * 0.0
    amps = [SV[x]["I0"] + (SV[x]["Is"] - SV[x]["I0"]) * t / SV[x]["stall"] for x, t in zip(servo, tau)]
    mass = m.sum() + M_WINCH + M_MOUNT - (0.300 if tip != "hand3" else 0)
    return dict(n=n, tip=tip, L_mm=round(L * 1e3), reach_below_seat_mm=round((d0 + reach_tip) * 1e3), mass_g=round(mass * 1e3),
                held_level_pct_rated=[round(100 * t / r) for t, r in zip(tau, rated)], worst_pct_rated=round(100 * (tau / rated).max()),
                worst_pct_stall=round(100 * (tau / stall).max()), M_seat_Nm=round(M_seat, 2), lug_factor=round(94.8 * 5.8 / (2 * M_seat / (3 * 0.012)), 2),
                amps_held=round(sum(amps), 2), stall_amps_all=round(sum(SV[x]["Is"] for x in servo), 1))
rows = [option(n, t) for n in (6, 7, 8) for t in ("socket", "hand3")]
for r in rows: print(r)
json.dump(rows, open("tailarm_study.json", "w"), indent=1)
