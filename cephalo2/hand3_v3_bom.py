"""hand3 hub v3 parts list (2026-11-20): printed parts with masses from the v3 CAD, bought parts with quantities, and the hub E
parts that v3 no longer has.  PA-CF 1.25 g/cm3, TPU 1.21, steel 7.85.  Writes hand3_v3_bom.json.  Masses of printed parts are
solid-model volumes (100 % infill): an upper bound."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import hand3_hub_v3 as V3, hand3_hubc as HC, hand3_cad as C

RHO = dict(pacf=1.25e-3, tpu=1.21e-3, steel=7.85e-3)


def g(m, mat="pacf"):
    return round(m.volume() * RHO[mat], 2)


def printed():
    P = HC._parts(); R = C.parts(); sp = V3.station_parts()
    rows = [("base disc", 1, g(V3.base_disc()), "PA-CF"), ("top disc", 1, g(V3.top_disc()), "PA-CF"),
            ("root block (grip merged, cleaned)", 4, g(V3.root_block(0)), "PA-CF"), ("bead (C-slider)", 4, g(V3.bead(0)), "PA-CF"),
            ("bell crank", 8, g(V3.crank(1)), "PA-CF"), ("base ring v3 (3 mm flange, rail)", 1, g(V3.ring_v3()), "PA-CF"),
            ("cage v3", 1, g(V3.cage_v3()), "PA-CF")]
    for w, s in HC.YOKES: rows.append((f"yoke half {w}/{s}", 1, g(V3.yoke(w, s)), "PA-CF"))
    rows += [("spool", 2, g(P["spool"][0]), "PA-CF"), ("tie ring", 1, g(P["tie_ring"][0]), "PA-CF"), ("frame", 1, g(P["frame"][0]), "PA-CF"),
             ("heel", 1, g(R["heel"]), "PA-CF")]
    for j in (1, 2, 3):
        rows += [(f"phalanx {j} (v3: band sector + clevises)", 4, g(V3.B.phalanx_v3(j - 1)), "PA-CF"), (f"pad {j}", 4, g(R[f"pad{j}"], "tpu"), "TPU 95A")]
    rows += [("station blade + spine", 1, g(sp["blade"]), "PA-CF")]
    return rows


BOUGHT = [("Feetech STS3215 servo (12 V)", 2, 55.0, "both pull the one base ring (2:1 tackle)"),
          ("MR63ZZ bearing (idler)", 7, 0.3, "est. mass"), ("carbon rod dia 3", 8, 0.6, "yoke guides, est. length 45 mm"),
          ("music wire dia 2 (latch pin, bent)", 8, None, "mass from CAD"), ("steel pin dia 1 x 5 (crank pivot)", 8, 0.03, "est."),
          ("knuckle pins and bushings (hand3)", 12, None, "unchanged from hand3"),
          ("Dyneema 0.5 mm braid", 1, 0.5, "est. 3 m: four single flexor cords + tackles + servo cords"),
          ("Pololu 298:1 Micro Metal Gearmotor HPCB 12 V (N20)", 1, 9.5, "station roller drive"),
          ("dia 20 rubber tyre / O-ring on hub", 1, 1.0, "est."), ("TowerPro SG92R micro servo", 1, 9.0, "station carriage"),
          ("Pololu QTR-1A reflectance sensor", 1, 0.3, "spin interlock"),
          ("NBR O-ring 9 x 1.5", 4, 0.07, "K1 return band"), ("silicone O-ring 8 x 1", 4, 0.03, "K2 return band"),
          ("silicone O-ring 8 x 1.5", 4, 0.06, "K3 return band"), ("hardened steel dowel pin dia 1 x 5 (ISO 8734)", 24, 0.03, "band hooks, press fit"),
          ("M2 x 6 thread-forming screw", 2, 0.2, "B.7 lead rail"), ("M3 x 8 screw", 4, 0.5, "frame to coupler hand base")]

REMOVED = ["inner (curl) pull ring", "TPU series links (base and curl, 4 each)", "lag tubes (4)", "lag-spring peg, post and O-rings (4)",
           "hub stop posts", "hub_lower / hub_upper plates (now base and top discs)", "ring link lugs and cage 61 deg link slots",
           "root-grip lag bore, k=1/k=2 cord lanes and spring-cord paths", "K1 cord knot (one cord now runs K1..K3)",
           "latch-pin tab ends past the crank (tab now ends at r 21.5)",
           "hub E return-spring boxes: 2 trays + lid per finger (0.94 g), their screw, the root post, knuckle saddles, extensor cords and bores"]

PENDING = []


if __name__ == "__main__":
    rows = printed()
    pin = round(V3.latch_pin(0, 1).volume() * RHO["steel"], 2)
    bought = [(n, q, pin if (m is None and "music" in n) else m, note) for n, q, m, note in BOUGHT]
    tot_p = sum(q * m for _, q, m, _ in rows); tot_b = sum(q * m for _, q, m, _ in bought if m)
    out = dict(printed=[dict(part=n, qty=q, g_each=m, material=mat) for n, q, m, mat in rows],
               bought=[dict(part=n, qty=q, g_each=m, note=note) for n, q, m, note in bought],
               removed_vs_hubE=REMOVED, pending=PENDING, total_printed_g=round(tot_p, 1), total_bought_g=round(tot_b, 1))
    json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3_v3_bom.json"), "w"), indent=1)
    for n, q, m, mat in rows: print(f"  {q:2d} x {n:38s} {m:6.2f} g  {mat}")
    for n, q, m, note in bought: print(f"  {q:2d} x {n:50s} {m if m else '-':>6} g  {note}")
    print("printed", round(tot_p, 1), "g   bought", round(tot_b, 1), "g")
