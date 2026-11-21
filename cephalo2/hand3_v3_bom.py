"""hand3 hub v3 parts list (2026-11-20, multi-digit decision D): printed parts with masses from the CAD, bought parts with
quantities, and the hub E parts that v3 D no longer has.  PA-CF 1.25 g/cm3, TPU 1.21, steel 7.85.  Writes hand3_v3_bom.json.  Masses of printed parts are
solid-model volumes (100 % infill): an upper bound."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import hand3_hub_v3 as V3, hand3_hubc as HC, hand3_cad as C, hand3_multidigit as MD

RHO = dict(pacf=1.25e-3, tpu=1.21e-3, steel=7.85e-3, alsi=2.68e-3)


def g(m, mat="pacf"):
    return round(m.volume() * RHO[mat], 2)


def printed():
    R = C.parts(); sp = MD.station_d()
    rows = [("base disc", 1, g(V3.base_disc()), "PA-CF"), ("top disc", 1, g(V3.top_disc()), "PA-CF"),
            ("root block + winch leg", 4, g(MD.root_d(0), "alsi"), "AlSi10Mg (ordered metal print)"), ("nut slider", 4, g(MD.slider(0)), "PA-CF"),
            ("bell crank", 8, g(V3.crank(1)), "PA-CF"), ("cage v3", 1, g(V3.cage_v3()), "PA-CF"),
            ("frame (driver pockets)", 1, g(MD.frame_d()), "PA-CF"), ("driver lift ring", 1, g(MD.lift_ring()), "PA-CF"),
            ("heel", 1, g(R["heel"]), "PA-CF")]
    for j in (1, 2, 3):
        rows += [(f"phalanx {j} (band sector + clevises)", 4, g(V3.B.phalanx_v3(j - 1)), "PA-CF"), (f"pad {j}", 4, g(R[f"pad{j}"], "tpu"), "TPU 95A")]
    rows += [("station blade + spine (notched)", 1, g(sp["blade"]), "PA-CF")]
    return rows


BOUGHT = [("Pololu 30:1 Micro Metal Gearmotor HPCB 6V", 8, 9.5, "static drivers under the parked roots"),
          ("M2 threaded rod, cut to 24.5 mm, slotted end", 4, 0.6, "root lead screws (est. mass)"),
          ("M2 brass hex nut", 4, 0.2, "in the sliders (est.)"),
          ("driver coupler with blade key, 3 mm D bore", 8, 0.4, "brass or printed (est.)"),
          ("TowerPro SG92R micro servo", 2, 9.0, "station carriage + driver lift ring"),
          ("Adafruit 3297 DRV8833 dual motor driver", 4, 1.5, "static side: 8 drivers"),
          ("music wire dia 2 (latch pin, bent)", 8, 0.19, "travel latches"), ("steel pin dia 1 x 5 (crank pivot)", 8, 0.03, "est."),
          ("knuckle pins and bushings (hand3)", 12, None, "unchanged from hand3"),
          ("Dyneema 0.5 mm braid", 1, 0.4, "est. 2 m: four single flexor cords + tackles"),
          ("Pololu 298:1 Micro Metal Gearmotor HPCB 12 V (N20)", 1, 9.5, "station roller drive"),
          ("dia 20 rubber tyre / O-ring on hub", 1, 1.0, "est."), ("Pololu QTR-1A reflectance sensor", 1, 0.3, "park index + spin interlock"),
          ("NBR O-ring 9 x 1.5", 4, 0.07, "K1 return band"), ("silicone O-ring 8 x 1", 4, 0.03, "K2 return band"),
          ("silicone O-ring 8 x 1.5", 4, 0.06, "K3 return band"), ("hardened steel dowel pin dia 1 x 5 (ISO 8734)", 24, 0.03, "band hooks, press fit"),
          ("M2 x 6 thread-forming screw", 2, 0.2, "B.7 lead rail"), ("M3 x 8 screw", 4, 0.5, "frame to coupler hand base")]

REMOVED = ["inner (curl) pull ring", "TPU series links (base and curl, 4 each)", "lag tubes (4)", "lag-spring peg, post and O-rings (4)",
           "hub stop posts", "hub_lower / hub_upper plates (now base and top discs)", "ring link lugs and cage 61 deg link slots",
           "root-grip lag bore, k=1/k=2 cord lanes and spring-cord paths", "K1 cord knot (one cord now runs K1..K3)",
           "latch-pin tab ends past the crank (tab now ends at r 21.5)",
           "the shared pull drive (decision D): base pull ring + rail, beads, 4 yoke halves, 8 carbon rods, tie ring, 7 MR63 idlers, 2 STS3215 + spools",
           "hub E return-spring boxes: 2 trays + lid per finger (0.94 g), their screw, the root post, knuckle saddles, extensor cords and bores"]

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
