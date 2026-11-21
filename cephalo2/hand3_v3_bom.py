"""hand3 hub v3 parts list (2026-11-20, rotation decision F3 on top of multi-digit decision D): printed parts with masses from the CAD, bought parts with
quantities, and the hub E parts that v3 D no longer has.  PA-CF 1.25 g/cm3, TPU 1.21, steel 7.85.  Writes hand3_v3_bom.json.  Masses of printed parts are
solid-model volumes (100 % infill): an upper bound."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import hand3_hub_v3 as V3, hand3_hubc as HC, hand3_cad as C, hand3_multidigit as MD, hand3_rotation_f3 as F

RHO = dict(pacf=1.25e-3, tpu=1.21e-3, steel=7.85e-3, alsi=2.68e-3)


def g(m, mat="pacf"):
    return round(m.volume() * RHO[mat], 2)


def printed():
    R = C.parts()
    rows = [("base disc + z166 gear skirt", 1, g(F.base_disc_f3()), "PA-CF resin"), ("top disc", 1, g(V3.top_disc()), "PA-CF"),
            ("root block + winch leg + motor collars + foot", 4, g(F.root_f3(0), "alsi"), "AlSi10Mg (ordered metal print)"), ("nut slider", 4, g(MD.slider(0)), "PA-CF"),
            ("cage v3", 1, g(V3.cage_v3()), "PA-CF"), ("frame (driver pockets, post holes)", 1, g(F.frame_f3()), "PA-CF"),
            ("lift ring (2 stages)", 1, g(F.lift_ring_f3()), "PA-CF"), ("track ring", 1, g(F.track_ring()), "PA-CF"), ("heel", 1, g(R["heel"]), "PA-CF")]
    for j in (1, 2, 3):
        rows += [(f"phalanx {j} (band sector + clevises)", 4, g(V3.B.phalanx_v3(j - 1)), "PA-CF"), (f"pad {j}", 4, g(R[f"pad{j}"], "tpu"), "TPU 95A")]
    return rows


BOUGHT = [("Pololu 2359 700:1 sub-micro plastic planetary gearmotor 6Dx21L", 4, 1.3, "root rotation, one per root"),
          ("brass spur pinion m0.3 z12, 2 mm bore", 4, 0.25, "on the 2359 shafts (est. mass)"),
          ("Pololu 30:1 Micro Metal Gearmotor HPCB 6V", 8, 9.5, "static curl drivers under the parked roots"),
          ("M2 threaded rod, cut to 24.5 mm, slotted end", 4, 0.6, "root lead screws (est. mass)"),
          ("M2 brass hex nut", 4, 0.2, "in the sliders (est.)"),
          ("driver coupler with blade key (chamfered tip), 3 mm D bore", 8, 0.4, "brass or printed (est.)"),
          ("TowerPro SG92R micro servo", 1, 9.0, "two-stage lift ring"),
          ("phosphor-bronze leaf 0.15 x 0.8, cut and bent", 8, 0.02, "wipers, 2 per root (est.)"), ("M1 rivet / screw", 8, 0.01, "wiper mounts (est.)"),
          ("copper foil tape 0.8 mm strip", 1, 0.3, "5 tracks on the track ring (est.)"),
          ("steel rod dia 2 x 44.5", 4, 1.09, "track ring posts"), ("compression spring 2.5 OD x 10", 4, 0.1, "track ring on the lift ring (est.)"),
          ("Adafruit 2448 TB6612 motor driver", 1, 2.0, "static side: 4 rotation tracks 0 / 12 V (est. mass)"),
          ("6 V regulator for the common track (e.g. Pololu D24V10F6)", 1, 1.0, "static side (est.)"),
          ("Adafruit 3297 DRV8833 dual motor driver", 4, 1.5, "static side: 8 curl drivers"),
          ("knuckle pins and bushings (hand3)", 12, None, "unchanged from hand3"),
          ("Dyneema 0.5 mm braid", 1, 0.4, "est. 2 m: four single flexor cords + tackles"),
          ("NBR O-ring 9 x 1.5", 4, 0.07, "K1 return band"), ("silicone O-ring 8 x 1", 4, 0.03, "K2 return band"),
          ("silicone O-ring 8 x 1.5", 4, 0.06, "K3 return band"), ("hardened steel dowel pin dia 1 x 5 (ISO 8734)", 24, 0.03, "band hooks, press fit"),
          ("M2 x 6 thread-forming screw", 2, 0.2, "B.7 lead rail"), ("M3 x 8 screw", 4, 0.5, "frame to coupler hand base")]

REMOVED = ["inner (curl) pull ring", "TPU series links (base and curl, 4 each)", "lag tubes (4)", "lag-spring peg, post and O-rings (4)",
           "hub stop posts", "hub_lower / hub_upper plates (now base and top discs)", "ring link lugs and cage 61 deg link slots",
           "root-grip lag bore, k=1/k=2 cord lanes and spring-cord paths", "K1 cord knot (one cord now runs K1..K3)",
           "latch-pin tab ends past the crank (tab now ends at r 21.5)",
           "the shared pull drive (decision D): base pull ring + rail, beads, 4 yoke halves, 8 carbon rods, tie ring, 7 MR63 idlers, 2 STS3215 + spools",
           "hub E return-spring boxes: 2 trays + lid per finger (0.94 g), their screw, the root post, knuckle saddles, extensor cords and bores",
           "decision F3: 8 latch pins, 8 bell cranks + pivots, the station (roller N20 298:1, tyre, blade + spine, SG92R, QTR-1A), base disc pin slots and pockets"]

PENDING = []


if __name__ == "__main__":
    rows = printed()
    bought = BOUGHT
    tot_p = sum(q * m for _, q, m, _ in rows); tot_b = sum(q * m for _, q, m, _ in bought if m)
    out = dict(printed=[dict(part=n, qty=q, g_each=m, material=mat) for n, q, m, mat in rows],
               bought=[dict(part=n, qty=q, g_each=m, note=note) for n, q, m, note in bought],
               removed_vs_hubE=REMOVED, pending=PENDING, total_printed_g=round(tot_p, 1), total_bought_g=round(tot_b, 1))
    json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3_v3_bom.json"), "w"), indent=1)
    for n, q, m, mat in rows: print(f"  {q:2d} x {n:38s} {m:6.2f} g  {mat}")
    for n, q, m, note in bought: print(f"  {q:2d} x {n:50s} {m if m else '-':>6} g  {note}")
    print("printed", round(tot_p, 1), "g   bought", round(tot_b, 1), "g")
