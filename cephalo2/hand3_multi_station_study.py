"""hand3 hub v3: several parking stations under the hub, so several roots move at once and the palm stays free (user, 2026-11-20:
"multiple stations underneath").  Options study.

Back to the v3 station mechanism (latch pins + cranks on each root, a blade per station that lifts a root's pins and holds it,
ONE roller that turns the rotor under every held root).  New: more blades.  While the roller turns the hub, every held root stays
put in the world and so moves round the hub; each blade lets go on its own when its root has gone far enough (pins drop at the
next 30 deg hole), so held roots move different amounts in one turn.  The whole pattern may also turn freely.
Where a blade can stand: every 5 deg, blade + SG92R + sensor (and the roller where it is) checked against the static drive at
both ring positions (exact booleans, hand3_hub_v3.drive).  Then a planner (Dijkstra over hub-frame poses on the 30 deg grid,
rounds of align -> engage -> turn -> release) gives the time of every pose change for each station set.
Times: rotor 0.24 s per 30 deg (N20 298:1 roller, hand3_station_study), settle 0.3 s, engage 0.6 s, release 0.5 s, retract 0.4 s.
Writes hand3_multi_station_study.json/.png."""
import os, sys, json, itertools, heapq
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle, Polygon
import hand3_hub_v3 as V3

HERE = os.path.dirname(os.path.abspath(__file__))
POSES = {k: tuple(v) for k, v in V3.POSES.items()}
MIN_SEP = 60                                   # on the 30 deg grid (58 deg needed)
T30 = 0.24; T_SETTLE, T_ENG, T_REL, T_RET = 0.3, 0.6, 0.5, 0.4


def free_azimuths():
    sp = V3.station_parts(); blade = sp["blade"] + sp["servo"] + sp["sensor"]; roll = sp["tyre"] + sp["motor"]
    stat = None
    for pull in (0.0, V3.RING_STROKE):
        for k, m in V3.drive(pull).items():
            if pull == 0.0 or k.startswith("yoke"): stat = m if stat is None else stat + m
    fb, fr = [], []
    for az in range(0, 360, 5):
        if (blade.rotate([0, 0, az]) ^ stat).volume() < 0.05: fb.append(az)
        if ((blade + roll).rotate([0, 0, az]) ^ stat).volume() < 0.05: fr.append(az)
    return fb, fr


def norm(state):
    return tuple(sorted(a % 360 for a in state))


def pattern(state):
    s = sorted(a % 360 for a in state); return min(tuple((x - s[i]) % 360 for x in s[i:] + s[:i]) for i in range(4))


def ok_path(start, moves, steps=12):
    """moves: per-root relative moves (deg, same sign); all move together until each reaches its amount."""
    st = np.array(start, float); mv = np.array(moves, float); mx = np.abs(mv).max()
    for f in np.linspace(0, 1, steps + 1):
        th = f * mx; cur = st + np.sign(mv) * np.minimum(np.abs(mv), th)
        for i in range(4):
            for j in range(i + 1, 4):
                if abs((cur[i] - cur[j] + 180) % 360 - 180) < MIN_SEP - 1e-6: return False
    return True


def plan(a, b, S):
    """Fastest sequence of rounds from pose a to pose b (any turn of the pattern), stations S (world az)."""
    tgt = pattern(b); start = (norm(a), 0)
    dist = {start: 0.0}; prev = {}; pq = [(0.0, start)]
    while pq:
        d, (st, psi) = heapq.heappop(pq)
        if d > dist.get((st, psi), 1e9): continue
        if pattern(st) == tgt:
            path = []; k = (st, psi)
            while k in prev: path.append(prev[k][1]); k = prev[k][0]
            return d, path[::-1]
        for psi2 in range(0, 360, 30):
            at = [k for k, x in enumerate(st) if (x + psi2) % 360 in S]
            if not at: continue
            dpsi = abs((psi2 - psi + 180) % 360 - 180); t_al = dpsi / 30 * T30 + T_SETTLE
            for n in range(1, len(at) + 1):
                for H_ in itertools.combinations(at, n):
                    for sgn in (1, -1):
                        for amts in itertools.product((30, 60, 90, 120), repeat=n):
                            mv = [0.0] * 4
                            for k, m in zip(H_, amts): mv[k] = -sgn * m          # held roots move against the hub turn
                            if not ok_path(st, mv): continue
                            new = norm([x + m for x, m in zip(st, mv)]); th = max(amts)
                            cost = t_al + T_ENG + th / 30 * T30 + T_REL + T_RET
                            key = (new, (psi2 + sgn * th) % 360)
                            if d + cost < dist.get(key, 1e9):
                                dist[key] = d + cost; prev[key] = ((st, psi), dict(psi=psi2, held=list(H_), moves=mv)); heapq.heappush(pq, (d + cost, key))
    return None, None


SETS = {"1 station (now)": (90,), "2 opposite": (90, 270), "3 (0, 90, 270)": (0, 90, 270), "4 at 90 deg": (0, 90, 180, 270),
        "4 (90, 240, 270, 300)": (90, 240, 270, 300), "6 (0, 90, 180, 240, 270, 300)": (0, 90, 180, 240, 270, 300)}
SEQ = [("flight cross", "3 + 1 thumb"), ("3 + 1 thumb", "2 + 2 pinch"), ("2 + 2 pinch", "all four on one side"), ("all four on one side", "flight cross")]


def numbers():
    fb, fr = free_azimuths(); out = dict(free_blade_az=fb, free_roller_az=fr, sets={})
    for name, S in SETS.items():
        bad = [s for s in S if s not in fb]
        sp = V3.station_parts(); one = sp["blade"] + sp["servo"] + sp["sensor"]; roll = sp["tyre"] + sp["motor"]
        clash = []
        for i, a in enumerate(S):
            for b in S[i + 1:]:
                A = one.rotate([0, 0, a]) + (roll.rotate([0, 0, a]) if a == 90 else one.rotate([0, 0, a]))
                Bm = one.rotate([0, 0, b]) + (roll.rotate([0, 0, b]) if b == 90 else one.rotate([0, 0, b]))
                v = (A ^ Bm).volume()
                if v > 0.05: clash.append((a, b, round(v, 1)))
        row = dict(stations=S, not_free=bad, clash=clash, extra_g=round((len(S) - 1) * (9.0 + 2.0), 1), trans={})
        tot = 0.0
        for a, b in SEQ:
            t, p = plan(POSES[a], POSES[b], set(S)); row["trans"][f"{a} -> {b}"] = dict(s=round(t, 2), rounds=len(p), plan=p); tot += t
        allp = [plan(POSES[a], POSES[b], set(S))[0] for a in POSES for b in POSES if a != b]
        row["sim_sequence_s"] = round(tot, 1); row["worst_change_s"] = round(max(allp), 2); row["mean_change_s"] = round(float(np.mean(allp)), 2)
        out["sets"][name] = row; print(name, row["sim_sequence_s"], row["worst_change_s"], row["mean_change_s"], bad, clash, flush=True)
    return out


def figure(N):
    fig = plt.figure(figsize=(22, 11)); names = list(SETS)
    fb = set(N["free_blade_az"])
    for i, name in enumerate(names):
        ax = fig.add_axes([0.01 + i * 0.165, 0.47, 0.16, 0.42]); ax.set_aspect("equal"); ax.set_xlim(-58, 58); ax.set_ylim(-58, 58); ax.axis("off")
        for az in range(0, 360, 5):
            ax.add_patch(Wedge((0, 0), 50, az - 2.5, az + 2.5, width=20, fc="#dff3e6" if az in fb else "#f6dada", ec="none"))
        ax.add_patch(Circle((0, 0), V3.V["disc_r"], fc="#e6e6e6", ec="#999"))
        for a in POSES["flight cross"]:
            ax.add_patch(Wedge((0, 0), V3.V["block_r"][1], a - V3.V["block_half"], a + V3.V["block_half"], width=V3.V["block_r"][1] - V3.V["block_r"][0], fc="#7fa7d9", ec="#335", lw=0.5))
        for s in SETS[name]:
            c, sn = np.cos(np.radians(s)), np.sin(np.radians(s))
            ax.add_patch(Polygon([[c * 30 - sn * 5.5, sn * 30 + c * 5.5], [c * 48 - sn * 5.5, sn * 48 + c * 5.5], [c * 48 + sn * 5.5, sn * 48 - c * 5.5], [c * 30 + sn * 5.5, sn * 30 - c * 5.5]],
                                 fc="#f0a030", ec="k", lw=0.6))
        r = N["sets"][name]
        ax.set_title(f"{name}\nsim sequence {r['sim_sequence_s']} s, worst change {r['worst_change_s']} s", fontsize=9.5,
                     color="#2a7" if name == REC else "k", fontweight="bold" if name == REC else "normal")
    rows = [["", *names]]
    rows.append(["pose changes in the sim (4)", *[f"{N['sets'][n]['sim_sequence_s']} s" for n in names]])
    rows.append(["worst / mean pose change", *[f"{N['sets'][n]['worst_change_s']} / {N['sets'][n]['mean_change_s']} s" for n in names]])
    rows.append(["blades at free azimuths", *["yes" if not N["sets"][n]["not_free"] else f"no: {N['sets'][n]['not_free']}" for n in names]])
    rows.append(["stations clear of each other", *["yes" if not N["sets"][n]["clash"] else "no: servos 30 deg apart clash\n(needs a new servo layout)" for n in names]])
    rows.append(["added static mass (SG92R + blade each)", *[f"+{N['sets'][n]['extra_g']} g" for n in names]])
    rows.append(["palm", *["free (fingers' roots only)"] * len(names)])
    rows.append(["roots moving together", *[("one at a time" if len(SETS[n]) == 1 else f"up to {len(SETS[n])}") for n in names]])
    ax = fig.add_axes([0.01, 0.02, 0.98, 0.36]); ax.axis("off")
    t = ax.table(cellText=rows, loc="center", cellLoc="center"); t.auto_set_font_size(False); t.set_fontsize(8.5); t.scale(1, 2.4)
    k = names.index(REC) + 1
    for r in range(len(rows)): t[r, k].set_facecolor("#dff3e6")
    fig.suptitle("hand3 hub v3 - several stations under the hub (orange blades; green = azimuths where a blade clears the drive).  "
                 "Times from a planner over the 30 deg grid.  Recommended: " + REC, fontsize=12.5)
    fig.savefig(os.path.join(HERE, "hand3_multi_station_study.png"), dpi=100)


REC = None
if __name__ == "__main__":
    N = numbers()
    ok = {n: r for n, r in N["sets"].items() if not r["not_free"] and not r["clash"]}
    REC = min(ok, key=lambda n: (ok[n]["sim_sequence_s"] + 0.05 * ok[n]["extra_g"]))
    N["recommended"] = REC
    json.dump(N, open(os.path.join(HERE, "hand3_multi_station_study.json"), "w"), indent=1, default=float)
    figure(N)
