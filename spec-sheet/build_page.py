"""Build the spec sheet page from results/*.json -> page/index.html"""
import json, os, math, html, datetime
H = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(H, "results")
J = lambda n: json.load(open(os.path.join(R, n))) if os.path.exists(os.path.join(R, n)) else {}
G, L, LH, T, ST, SY = J("geom.json"), J("loads_alt90.json"), J("loads_helix60.json"), J("tol.json"), J("struct_alt90.json"), J("system.json")
STN = J("stance_summary.json"); FS = J("flight_summary.json")
esc = html.escape

def num(x, d=2):
    if isinstance(x, str): return esc(x)
    if x is None or (isinstance(x, float) and math.isnan(x)): return "–"
    if isinstance(x, float) and (abs(x) >= 1e5 or (0 < abs(x) < 1e-3)): return f"{x:.1e}".replace("e+0", "e").replace("e+", "e")
    if isinstance(x, float): return f"{x:.{d}f}".rstrip("0").rstrip(".") if d else f"{x:.0f}"
    return esc(str(x))

def table(head, rows, cls="", note=None):
    h = "".join(f"<th>{esc(c)}</th>" for c in head)
    b = "".join("<tr>" + "".join(f"<td>{c}</td>" for c in r) + "</tr>" for r in rows)
    n = f'<p class="tnote">{note}</p>' if note else ""
    return f'<div class="tw"><table class="{cls}"><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div>{n}'

def chip(t, tone="neutral"): return f'<span class="chip {tone}">{esc(t)}</span>'
def tone_need(n): return "bad" if n > 1 else "warn" if n > 0.5 else "good"
def needc(n):
    if n is None or (isinstance(n, float) and math.isnan(n)): return "–"
    return f'<span class="n {tone_need(n)}">{n:.2f}</span>'
def sfc(s, lim=2.0): return f'<span class="n {"bad" if s < 1 else "warn" if s < lim else "good"}">{num(s, 1)}</span>'

def need_chart(series, title):
    """grouped bars, need/stall per joint, guides at 0.5 and 1.0"""
    W, Hh, padl, padb, padt = 760, 250, 44, 34, 14
    names = list(series); n = 13; gw = (W - padl - 10) / n; bw = min(10, (gw - 6) / max(1, len(names)))
    ymax = max(1.2, max(max(v) for v in series.values()) * 1.08); ymax = math.ceil(ymax * 5) / 5
    y = lambda v: padt + (Hh - padt - padb) * (1 - v / ymax)
    s = [f'<svg viewBox="0 0 {W} {Hh}" role="img" aria-label="{esc(title)}" class="chart">']
    for gv in [x / 10 for x in range(0, int(ymax * 10) + 1, 2)]:
        s.append(f'<line x1="{padl}" x2="{W-6}" y1="{y(gv):.1f}" y2="{y(gv):.1f}" class="grid"/><text x="{padl-6}" y="{y(gv)+4:.1f}" class="ax" text-anchor="end">{gv:.1f}</text>')
    for gv, cl, lab in ((0.5, "g2", "2x rule"), (1.0, "gs", "stall")):
        s.append(f'<line x1="{padl}" x2="{W-6}" y1="{y(gv):.1f}" y2="{y(gv):.1f}" class="{cl}"/><text x="{W-8}" y="{y(gv)-4:.1f}" class="ax {cl}t" text-anchor="end">{lab}</text>')
    for j in range(n):
        x0 = padl + j * gw + (gw - bw * len(names)) / 2
        for i, nm in enumerate(names):
            v = series[nm][j]
            s.append(f'<rect x="{x0 + i*bw:.1f}" y="{y(v):.1f}" width="{bw-1:.1f}" height="{max(0.5, y(0)-y(v)):.1f}" class="s{i}"><title>{esc(nm)}, joint {j}: {v:.2f}</title></rect>')
        s.append(f'<text x="{padl + j*gw + gw/2:.1f}" y="{Hh-14}" class="ax" text-anchor="middle">{j}</text>')
    s.append(f'<text x="{padl + (W-padl)/2:.0f}" y="{Hh-1}" class="ax" text-anchor="middle">joint (0 at the root; even = pitch, odd = yaw)</text></svg>')
    leg = "".join(f'<span class="lg"><i class="s{i}"></i>{esc(nm)}</span>' for i, nm in enumerate(names))
    return f'<figure class="fig"><div class="legend">{leg}</div><div class="tw">{"".join(s)}</div><figcaption>{esc(title)}</figcaption></figure>'

# ---------------------------------------------------------------- data pulls
mass = G.get("mass", {}); ws = G.get("workspace", {}); curl = G.get("curl", {})
fl = L.get("flight", {}); fh = fl.get("hover", {}); fr = fl.get("rated", {}); ff = fl.get("full", {}); tl = fl.get("rated_tilted_30", {})
hfl = LH.get("flight", {})
ho = L.get("holdout", []); hho = LH.get("holdout", [])
rp = L.get("random_poses", {}); hrp = LH.get("random_poses", {})
fan = L.get("fan", {})
env = ST.get("envelope", []); stress = ST.get("stress", []); brg = ST.get("bearings", []); fat = ST.get("fatigue_walking", [])
eff = SY.get("efficiency", {}); res = SY.get("resilience", {}); sens = SY.get("sensitivity", {})
straight_need = J("straight_need.json")
min_sf_row = min(stress, key=lambda r: r["min_sf"]) if stress else None

# ---------------------------------------------------------------- findings
F = []
def find(tone, head, body): F.append((tone, head, body))
if FS:
    v3, v10, v20 = FS.get("lean3_hover", {}), FS.get("lean10_hover", {}), FS.get("lean20_hover", {})
    find("warn", f"Hover: lean the fan about 20 deg and the arm holds at {v20.get('worst',float('nan')):.2f} of stall",
         f"With the fan kept upright (within 3 deg) the best pose found puts {v3.get('worst',float('nan')):.2f} of stall on joint 2 at hover and {FS.get('lean3_full',{}).get('worst',float('nan')):.2f} at full thrust: over your 2x rule, and joint 2 overheats. "
         f"Let the fan lean off vertical (the eight arms' sideways pushes cancel) and thrust starts carrying the arm: 10 deg gives {v10.get('worst',float('nan')):.2f} for 1.5 % of lift, "
         f"20 deg gives {v20.get('worst',float('nan')):.2f} at hover and {FS.get('lean20_full',{}).get('worst',float('nan')):.2f} at full thrust for 6 % of lift.")
if straight_need:
    ok2 = [r["reach_m"] for r in ho if r["ok"] and r["worst_no_payload"] <= 0.5]
    okS = [r["reach_m"] for r in ho if r["ok"] and r["worst_no_payload"] <= 1.0]
    find("bad", "It cannot hold itself out straight with the fan off",
         f"Straight and level, joint {straight_need['joint']} needs {straight_need['worst']:.2f} of stall (the helix arm's worst is 1.10). Arched up with the tip held level, it reaches "
         + (f"{max(okS):.2f} m from the root under stall and {max(ok2):.2f} m under your 2x rule. " if okS and ok2 else "")
         + "Carrying anything held out needs the fan's help or a short reach.")
if STN:
    rad = STN.get("radii", {})
    def rr(k): v = rad.get(k, {}); return v.get("worst")
    r45 = rr("alt90 0.45"); r35 = rr("alt90 0.35")
    find("bad" if (STN.get("worst") or 9) > 1 else "warn", f"Walking on the folded pod: feet need to sit about 45 cm out",
         f"With the feet 30 cm from the hub axis the outer joints (7 to 9, STS3215) run at {STN.get('worst',float('nan')):.2f} of stall, and {STN.get('worst_2x',float('nan')):.2f} at twice the foot load. "
         + (f"At 35 cm the worst is {r35:.2f}; at 45 cm it drops to {r45:.2f}, inside your 2x rule. " if r45 and r35 else "")
         + "The fold that brings a 1 m arm down to a near foot puts the foot load across the outer joints. 25 cm was out of reach under these clearance rules for both arms.")
hc = T.get("horn_clamp", {})
if hc:
    find("warn", "The horn screws are the loosest link",
         f"Four M3 screws through 3.4 mm holes in PETG hold {hc['torque_by_friction_new_Nm']} N.m by friction when new and about {hc['torque_by_friction_relaxed_Nm']} N.m once the plastic relaxes, which is under an STS3250's 4.9 N.m stall. When it slips, the hole clearance adds up to {hc['slip_backlash_deg_as_drawn']} deg of play at that joint. Ream to 3.1 mm and add washers, or bond the ear to the horn.")
su = T.get("stackup", {})
if su:
    k = list(su)
    find("warn", "Tip play: 9 mm typical, 35 mm worst case as designed; 4 times that if the horn clamps slip",
         f"Servo backlash (0.5 deg each, assumed) is 90 % of it. The 8.4 mm printed idler bores add half again; reamed to 8.05 to 8.10 mm they add almost nothing. Gravity and thrust usually hold every joint against one side of its play, so the tip only wanders this much when the load reverses.")
g = ST.get("gear", {}).get("cases", {})
if g:
    find("warn", "Root gear contact stress is the long-run limit at joint 0",
         f"At stall the m1 7075 pair sees {g.get('stall (9.8 N.m)',{}).get('hertz_MPa','?')} MPa contact and {g.get('stall (9.8 N.m)',{}).get('lewis_pinion_MPa','?')} MPa root bending, fine for peaks; at hover it sees {g.get('hover',{}).get('hertz_MPa','?')} MPa. Anodized aluminium pits under long running above roughly 350 MPa. A steel pinion removes the question.")
if min_sf_row:
    find("good" if min_sf_row["min_sf"] >= 2 else "warn", f"Weakest printed part: joint {min_sf_row['joint']} {min_sf_row['weakest'].replace('_',' ')}, safety factor {min_sf_row['min_sf']:.1f}",
         "Across every solved hover, tilt, full-thrust and 2x walking load, with PETG at 25 MPa across the layers. " + ("Everything clears your 2x rule." if min_sf_row["min_sf"] >= 2 else "That is under your 2x rule; thicken that ear or print it with the layers along the load."))
th = SY.get("thermal_hover", []); thL = SY.get("thermal_hover_lean20", [])
hot = [t for t in th if t["minutes_to_50K"] != "never" and t["minutes_to_50K"] < 17]
hotL = [t for t in thL if t["minutes_to_50K"] != "never" and t["minutes_to_50K"] < 17]
if th:
    find("bad" if hotL else "warn", "Servo heat decides the hover pose",
         ("With the fan upright, " + ", ".join(f"joint {t['joint']} reaches a 50 K rise in {t['minutes_to_50K']} min" for t in hot[:3]) + f", against a 17 minute flight, and the arm's servos burn {SY.get('efficiency',{}).get('servos_hover',{}).get('arm_W','?')} W. " if hot else "")
         + (f"In the 20 deg leaning pose the arm burns {SY.get('efficiency',{}).get('servos_hover_lean20',{}).get('arm_W','?')} W and " + ("no servo overheats in a flight." if not hotL else ", ".join(f"joint {t['joint']} still overheats in {t['minutes_to_50K']} min" for t in hotL) + ".") if thL else "")
         + " A holding servo turns all its input into heat; about 3 W per case is the steady limit (assumed), roughly a fifth of stall torque.")

dr = SY.get("resilience", {}).get("drop", {}).get("rows", [])
if dr:
    r5 = dr[0]
    find("bad", "Landing: it has to land under power",
         f"Dropped {r5['drop_m']*100:.0f} cm onto seven folded-pod feet with the servos as the only spring, each foot takes about {r5['peak_foot_N']} N and the worst joint sees {r5['worst_need']:.1f} times stall, enough to back-drive or strip a gear train. "
         "Let thrust carry the weight to touchdown and hand it over slowly, or add a soft foot pad.")

# ---------------------------------------------------------------- sections
S = []
def sec(id_, title, body, lead=None):
    l = f'<p class="lead">{lead}</p>' if lead else ""
    S.append(f'<section id="{id_}"><h2>{esc(title)}</h2>{l}{body}</section>')

# configuration
sec("config", "Configuration and method",
    "<ul class='plain'>"
    "<li><b>Arm:</b> 1.00 m, 13 pin-hinge joints at 76.9 mm pitch, 57.1 mm outer diameter. Hinge axes turn 90 deg from one joint to the next, so joints alternate pitch (even: 0, 2 … 12) and yaw (odd). That is the closest no-helix layout to the helix arm: same parts, one vertebra STL for every moving link, and pitch and yaw stay separated.</li>"
    "<li><b>Servos:</b> STS3250 (4.9 N.m stall, 12 V) in joints 0 to 4, STS3215 (2.94 N.m) in joints 5 to 12, a 2:1 m1 7075 spur stage on joint 0 (9.8 N.m). 6705 support bearings on the horn side of joints 1 to 4, 7075 idler pins on 0 to 4, printed 8 mm PETG stubs on 5 to 12.</li>"
    "<li><b>Tip:</b> folding fan pod, 252.5 g, MN5008 KV400 with a 16x5.4 prop 102 mm out; thrust acts along the tip axis.</li>"
    f"<li><b>Robot:</b> {L.get('robot_kg','?')} kg with the claw-fan wiring. Hover split: {fan.get('hover_N','?')} N per claw fan and {fan.get('central_N','?')} N from the central prop, about {fan.get('hover_total_W','?')} W.</li>"
    "<li><b>Method:</b> hand calculations and a rigid-body model of the arm built from the CAD masses (no FEA). Poses are found by optimisation: each one minimises the worst joint's share of stall under its geometric constraints, from many random starts, so a lower-torque pose may still exist. Tolerances use a 20,000-sample Monte Carlo. Values marked [A] are assumptions; the full list is at the end.</li></ul>"
    "<p class='src'>Model, scripts and raw results: octopus-drone/spec-sheet in the project files (spec_lib.py, sec1 to sec5, results/). CAD regenerated without the helix: spec-sheet/cad_alt90.</p>")

# geometry and mass
nodes = mass.get("nodes", [])
sec("mass", "Geometry, mass and inertia",
    "<div class='kv'>" + "".join(f"<div><span>{a}</span><b>{b}</b></div>" for a, b in [
        ("Arm, root socket to tip pod", f"{mass.get('arm_total_g','?')} g"), ("Links (13 vertebrae, servos, lids, bearings, wiring)", f"{mass.get('arm_links_g','?')} g"),
        ("Tip pod", f"{mass.get('tip_pod_g','?')} g"), ("Root, fixed to the mantle", f"{mass.get('root_fixed_g','?')} g"),
        ("Eight arms", f"{mass.get('eight_arms_kg','?')} kg ({mass.get('arm_share_of_robot',0):.0%} of the robot)"),
        ("Moving mass per arm", f"{mass.get('moving_kg','?')} kg"), ("CG, arm straight", f"{mass.get('cg_from_root_straight_mm','?')} mm from joint 0"),
        ("Inertia about the root pitch axis", f"{mass.get('inertia_about_root_pitch_kgm2','?')} kg·m²"),
        ("Gravity moment at the root, straight out", f"{mass.get('static_moment_straight_Nm','?')} N.m"),
        ("Servos per arm", "5 STS3250, 8 STS3215, 1 STS3032 (pod)")]) + "</div>"
    + table(["Link", "Between", "Mass g", "CG from joint mm", "Servo it carries"], [[n["node"], esc(n["between"]), num(n["g"], 1), num(n["cg_mm"], 1), esc(n["servo_carried"])] for n in nodes], "num")
    + "<p class='tnote'>Joint-space inertia (diagonal, arm straight, kg·m²): " + ", ".join(f"{x:.3f}" for x in mass.get("joint_space_diag_kgm2", [])) + ".</p>")

# workspace
wa, wh = ws.get("alt90", {}), ws.get("helix60", {})
sec("reach", "Reach, workspace and degrees of freedom",
    table(["", "No helix (90 deg)", "Helix (60 deg)"], [
        ["Workspace volume", f"{wa.get('volume_m3','?')} m³", f"{wh.get('volume_m3','?')} m³"],
        ["Reach, max / median / min", f"{wa.get('reach_max_m','?')} / {wa.get('reach_p50_m','?')} / {wa.get('reach_min_m','?')} m", f"{wh.get('reach_max_m','?')} / {wh.get('reach_p50_m','?')} / {wh.get('reach_min_m','?')} m"],
        ["Pitch / yaw joints", "7 / 6", "13 spread over 3 directions"],
        ["Shape fit, mean RMS over the study shapes", f"{G.get('shape_fit',{}).get('alt90','33.0')} mm", f"{G.get('shape_fit',{}).get('helix60','31.8')} mm"],
    ]) + "<div class='kv'>" + "".join(f"<div><span>{a}</span><b>{b}</b></div>" for a, b in [
        ("Degrees of freedom", "13 (plus the pod fold)"), ("Command range", "±45 deg per joint; joints 0, 1 and 12 clear to ±90 deg, neighbours together to ±60"),
        ("Tightest curl", f"{curl.get('bend_radius_mm','196')} mm radius (pitch joints 153.8 mm apart, 45 deg each)"),
        ("Full pitch curl, tip ends at", f"{curl.get('tip_after_full_pitch_curl_mm','?')} mm from joint 0"),
        ("Position resolution", "4096 counts per turn: 0.088 deg (0.044 deg at joint 0); about 1 mm at the tip, straight"),
        ("Manipulability, straight", "singular values 1.64, 1.47, 0 (no reach along the arm when straight)")]) + "</div>",
    "Turning every hinge by 90 deg instead of 60 deg costs almost nothing in reach or workspace. What changes is that pure pitch now uses only every other joint, so the arm bends in coarser steps.")

# torque demand
series = {}
if FS.get("lean3_hover"): series["hover, fan upright"] = FS["lean3_hover"]["need"]
if FS.get("lean20_hover"): series["hover, fan leaning 20 deg"] = FS["lean20_hover"]["need"]
stw = STN.get("need")
if stw: series["walking stance (worst)"] = stw
if straight_need: series["straight out, fan off"] = straight_need["need"]
chart = need_chart(series, "Share of stall torque per joint in the main poses") if series else ""
rows = []
labels = {"lean3": "Fan upright (within 3 deg)", "lean10": "Fan leaning up to 10 deg", "lean20": "Fan leaning up to 20 deg"}
for lk in ("lean3", "lean10", "lean20"):
    for nm in ("hover", "rated", "full"):
        d = FS.get(f"{lk}_{nm}")
        if not d: continue
        hd = hfl.get(nm, {}) if lk == "lean3" else {}
        rows.append([f"{labels[lk]}, {nm}", f"{d['T_N']:.1f} N", needc(d["worst"]), f"joint {d['need'].index(max(d['need']))}", f"{d.get('lift_lost',0):.1%}", needc(hd.get("worst")) if hd else "–"])
for k, v in tl.items():
    rows.append([f"Rated thrust, fan tilted 30 deg {k}", f"{fr.get('T_N','')} N", needc(v["worst"]), f"joint {v['need'].index(max(v['need']))}", "–", needc(hfl.get("rated_tilted_30", {}).get(k, {}).get("worst")) if hfl.get("rated_tilted_30", {}).get(k) else "–"])
flight_tab = table(["Flight case", "Fan thrust", "Worst need", "At", "Lift lost to lean", "Helix arm"], rows, note="Need = joint torque / stall. Green under 0.5 (your 2x rule), amber 0.5 to 1, red above stall. Every pose keeps the fan at least 0.60 m from the robot centre so the 16 in discs clear each other. Leaning fans push sideways; the eight arms' pushes cancel. Poses come from many random starts, so lower ones may exist.")
hrows = []
for r in ho:
    hr = next((x for x in hho if abs(x["reach_m"] - r["reach_m"]) < 1e-6), {})
    hrows.append([f"{r['reach_m']:.2f} m", needc(r["worst_no_payload"]) if r["ok"] else "no pose", num(r.get("payload_kg_2x", 0), 2), num(r.get("payload_kg_stall", 0), 2),
                  needc(hr["worst_no_payload"]) if hr.get("ok") else "–"])
hold_tab = table(["Tip held level at", "Worst need, fan off", "Payload at 2x rule kg", "Payload to stall kg", "Helix arm, worst need"], hrows,
                 note="Tip plate held at that distance from joint 0, level with the root, fan off; payload hangs from the tip plate.") if ho else ""
rnd = table(["Random poses (±45 deg), thrust along the tip axis", "Held at stall", "Held under the 2x rule", "Helix arm, held at stall"],
            [[k, f"{v['held_at_stall']:.0%}", f"{v['held_at_2x']:.0%}", f"{hrp.get(k,{}).get('held_at_stall',float('nan')):.0%}" if hrp.get(k) else "–"] for k, v in rp.items() if k != "n"],
            note=f"{rp.get('n','')} random poses each.") if rp else ""
stance_tab = ""
if STN.get("rows"):
    stance_tab = table(["Pod clocking", "Foot", "Feasible starts", "Worst need", "At 2x foot load"],
                       [[esc(r["side"]), esc(r["foot"]), esc(r["feasible_starts"]), needc(r["worst"]) if r["ok"] else "no pose", needc(r["worst_2x"]) if r["ok"] else "–"] for r in STN["rows"]],
                       note=STN.get("table_note", ""))
rad_tab = ""
if STN.get("radii"):
    rows_r = []
    for k, v in STN["radii"].items():
        a, r = k.split(" ")
        rows_r.append([("No helix" if a == "alt90" else "Helix"), f"{float(r)*100:.0f} cm", f"{v['feasible']}/{v['cases']}", needc(v["worst"]) if v["worst"] is not None else "no pose", needc(v["worst_2x"]) if v["worst_2x"] is not None else "–"])
    rad_tab = table(["Arm", "Foot radius from the hub axis", "Foot points reachable", "Worst need", "At 2x foot load"], rows_r, note="Same constraints for both arms: hub 250 mm up, tip within 10 deg of down, every link 15 mm clear of the ground and inside its own 36 deg wedge so it cannot reach a neighbour. The design notes' helix stance used a 110 mm hub radius and a direct neighbour check, which is why it found poses at 25 cm.")
sec("torque", "Joint torque demand", chart + flight_tab + "<h3>Walking stance on the folded pod</h3>" + rad_tab + stance_tab + "<h3>Holding the arm out</h3>" + hold_tab + rnd)

# speed and bandwidth
sp = ST.get("speed", {}); md = ST.get("modes_Hz", {})
srows = []
for k in ("fans off, straight", "hover, fan upright", "hover, fan leaning 20 deg"):
    if k in sp: srows.append([k] + [num(x, 1) for x in sp[k]["joint_speed_rad_s"]])
sec("speed", "Speed and bandwidth",
    table(["Case"] + [str(j) for j in range(13)], [["No load"] + [num(x, 1) for x in sp.get("no_load_rad_s", [])]] + srows, "num",
          note="Joint speed in rad/s, derated linearly with the torque each joint is holding (speed falls to zero at stall). Joint 0 runs at half the servo's speed through the 2:1 stage.")
    + "<div class='kv'>" + "".join(f"<div><span>{a}</span><b>{b}</b></div>" for a, b in [
        ("Tip speed, joint 0 alone", f"{sp.get('tip_speed_straight_m_s',{}).get('joint0_alone','?')} m/s"),
        ("Tip speed, all pitch joints together", f"{sp.get('tip_speed_straight_m_s',{}).get('all_pitch_joints_together','?')} m/s"),
        ("First modes, straight", ", ".join(f"{x} Hz" for x in md.get("straight out", []))),
        ("First modes, hover, fan upright", ", ".join(f"{x} Hz" for x in md.get("hover pose, fan upright", []))),
        ("First modes, hover, leaning 20 deg", ", ".join(f"{x} Hz" for x in md.get("hover pose, fan leaning 20 deg", []))),
        ("Servo bus, one per arm at 1 Mbit/s", f"{sp.get('bus',{}).get('cycle_ms','?')} ms per write-and-read cycle, about {sp.get('bus',{}).get('max_loop_Hz','?')} Hz"),
        ("Prop once-per-rev at hover", "about 80 Hz, well above the arm modes")]) + "</div>"
    + f"<p class='tnote'>{esc(ST.get('modes_note',''))}</p>")

# tolerances
fits = T.get("fits", [])
gear = T.get("gear", {})
stack_rows = []
for cn, d in su.items():
    for pn, v in d.items():
        stack_rows.append([esc(cn), esc(pn), num(v["rms_mm"], 1), num(v["p95_mm"], 1), num(v["worst_case_mm"], 1), f"{v['share_from_backlash']:.0%}"])
sec("tol", "Tolerances, fits and backlash",
    table(["Part", "Fit", "Loosens at +40 K, mm", "Note"], [[esc(f["item"]), esc(f["fit"]), num(f["loosens_per_40K_mm"], 3), esc(f["note"])] for f in fits], "wide")
    + "<h3>Root gear backlash</h3>" + table(["Normal backlash mm", "Joint 0 play deg"], [[num(r["normal_backlash_mm"], 2), num(r["joint_backlash_deg"], 3)] for r in gear.get("backlash_vs_joint_angle", [])], "num")
    + f"<p class='tnote'>{esc(gear.get('recommend',''))} {esc(gear.get('centre_error_to_backlash',''))}.</p>"
    + "<h3>Tip error from play</h3>" + table(["Build", "Pose", "RMS mm", "95 % mm", "Worst case mm", "From hinge backlash"], stack_rows, note=esc(T.get("stackup_note", "")))
    + "<div class='kv'>" + "".join(f"<div><span>{a}</span><b>{b}</b></div>" for a, b in [
        ("Horn clamp, new / relaxed", f"{hc.get('torque_by_friction_new_Nm','?')} / {hc.get('torque_by_friction_relaxed_Nm','?')} N.m by friction"),
        ("Arm length tolerance", f"±{T.get('length_tolerance',{}).get('worst_mm','?')} mm worst, ±{T.get('length_tolerance',{}).get('rss_mm','?')} mm RSS (±0.2 mm per printed link)"),
        ("Thermal growth", f"{T.get('length_tolerance',{}).get('thermal_mm_per_10K','?')} mm per 10 K over the metre"),
        ("Encoder step at the tip, joint 1", f"{T.get('servo_resolution',{}).get('tip_shift_from_one_count_at_joint1_mm','?')} mm")]) + "</div>")

# stiffness
sag = ST.get("sag_mm", {})
sec("stiff", "Stiffness and sag",
    table(["Load case", "Tip moves (x, y, z) mm", "Total mm"], [[esc(k), ", ".join(num(x, 1) for x in v["vec"]), num(v["mag"], 1)] for k, v in sag.items()])
    + "<div class='kv'>" + "".join(f"<div><span>Tip stiffness straight, {esc(k)}</span><b>{('rigid in this model' if v > 1000 else num(v, 3) + ' N/mm')}</b></div>" for k, v in ST.get("tip_stiffness_N_per_mm_straight", {}).items()) + "</div>"
    + f"<p class='tnote'>{esc(ST.get('stiffness_note',''))}</p>",
    "Sag is set almost entirely by how hard each servo holds its position. The printed links are far stiffer than the joints.")

# stresses
srow = []
for r in stress:
    e = env[r["joint"]] if r["joint"] < len(env) else {}
    srow.append([r["joint"], esc(r["pin"]), num(e.get("max_servo_torque_Nm", 0), 2), num(e.get("horn_support_N", 0), 0), num(e.get("idler_support_N", 0), 0),
                 num(r["horn_ear_MPa"], 1), num(r["idler_ear_MPa"], 1), num(r["idler_bore_MPa"], 1), num(r["pin_bend_MPa"], 1), sfc(r["min_sf"]), esc(r["weakest"].replace("_", " "))])
gc = ST.get("gear", {}).get("cases", {})
side = ST.get("servo_side_load", [])
sec("stress", "Stresses and safety factors",
    table(["Joint", "Pin", "Max servo torque N.m", "Horn side N", "Idler side N", "Horn ear MPa", "Idler ear MPa", "Bore MPa", "Pin bend MPa", "Min SF", "Weakest"], srow, "num",
          note=esc(ST.get("stress_note", "")))
    + "<h3>Root gear</h3>" + table(["Case", "Joint torque N.m", "Tooth force N", "Lewis pinion MPa", "Lewis gear MPa", "Contact MPa"],
                                   [[esc(k), num(v["joint_torque_Nm"], 2), num(v["tooth_force_N"], 0), num(v["lewis_pinion_MPa"], 0), num(v["lewis_gear_MPa"], 0), num(v["hertz_MPa"], 0)] for k, v in gc.items()], "num",
                                   note=esc(ST.get("gear", {}).get("note", "")) + " 7075-T6: yield 503 MPa, bending fatigue about 159 MPa.")
    + "<h3>Servo output side load, joints 5 to 12 (no support bearing)</h3>"
    + table(["Joint", "Horn side load N", "Against 40 N (XM430 reference)"], [[r["joint"], num(r["horn_side_N"], 1), sfc(1 / max(r["ratio"], 1e-6), 1.0)] for r in side], "num",
            note="Feetech publishes no side-load limit; ROBOTIS allows 40 N on the similar XM430, used here as a yardstick. The column shows 40 N divided by the load.")
    + "<div class='kv'>" + "".join(f"<div><span>{a}</span><b>{b}</b></div>" for a, b in [
        ("Root flange screws (8 x M3)", f"{ST.get('root_flange',{}).get('worst_screw_tension_N','?')} N worst, safety factor {ST.get('root_flange',{}).get('sf_screw','?')}"),
        ("Lid screws (4 x M2 into PETG)", f"{ST.get('lid_screws',{}).get('max_axial_N','?')} N axial worst, safety factor {ST.get('lid_screws',{}).get('sf','?')}")]) + "</div>")

# fatigue and bearings
sec("life", "Fatigue and bearing life",
    table(["Joint", "Support load swing N", "Ear stress swing MPa", "Ear cycles", "Pin stress swing MPa", "Pin cycles"],
          [[r["joint"], num(r["support_load_amplitude_N"], 1), num(r["ear_stress_amp_MPa"], 2), num(float(r["ear_cycles"]), 0), num(r["pin_stress_amp_MPa"], 2), num(float(r["pin_cycles"]), 0)] for r in fat], "num",
          note=esc(ST.get("fatigue_note", "")))
    + table(["Joint", "Bearing", "C / C0 N", "Worst static N", "Static SF", "Stance load N", "L10, ±15 deg swings", "L10, ±45 deg swings"],
            [[b["joint"], esc(b["bearing"]), f"{b['C_N']} / {b['C0_N']}", num(b["worst_static_N"], 0), sfc(b["static_sf"]), num(b["stance_load_N"], 0), num(float(b["L10_oscillations_15deg"]), 0), num(float(b["L10_oscillations_45deg"]), 0)] for b in brg], "num",
            note=esc(ST.get("bearing_note", ""))))

# efficiency
sh = eff.get("servos_hover", {})
sec("eff", "Efficiencies",
    table(["Stage", "Value"], [
        ["Claw fan at hover", f"{eff.get('fan_hover',{}).get('thrust_N','?')} N for {eff.get('fan_hover',{}).get('electrical_W','?')} W: {eff.get('fan_hover',{}).get('g_per_W','?')} g/W"],
        ["Claw fan figure of merit (motor and ESC included)", f"{eff.get('fan_hover',{}).get('figure_of_merit_incl_motor_esc','?')} (ideal momentum power {eff.get('fan_hover',{}).get('ideal_power_W','?')} W)"],
        ["Claw fan at rated / full", f"{eff.get('fan_rated',{}).get('g_per_W','?')} / {eff.get('fan_full',{}).get('g_per_W','?')} g/W"],
        ["Root gear stage", f"{eff.get('gear_stage',{}).get('total','?')} (mesh and two bearings) [A]"],
        ["Servos holding hover, fan upright", f"{sh.get('arm_W','?')} W per arm, {sh.get('robot_W','?')} W for the robot ({sh.get('share_of_hover_power',0):.1%} of hover power)"],
        ["Servos holding hover, leaning 20 deg", f"{eff.get('servos_hover_lean20',{}).get('arm_W','?')} W per arm, {eff.get('servos_hover_lean20',{}).get('robot_W','?')} W for the robot; the lean costs {eff.get('servos_hover_lean20',{}).get('extra_fan_power_W_per_arm','?')} W more fan power per arm"],
        ["Fan wiring, 16 AWG, hover / full", f"{eff.get('fan_wiring_hover',{}).get('loss_W','?')} W ({eff.get('fan_wiring_hover',{}).get('drop_V','?')} V drop) / {eff.get('fan_wiring_full',{}).get('loss_W','?')} W ({eff.get('fan_wiring_full',{}).get('drop_V','?')} V)"],
        ["Whole arm at hover", f"{eff.get('arm_power_chain_hover',{}).get('thrust_per_W_whole_arm',0)*1000/9.81:.2f} g of thrust per W, servos and wiring included"],
    ]) + table(["Joint"] + [str(h["joint"]) for h in sh.get("per_joint", [])], [["Upright: torque N.m"] + [num(h["torque_Nm"], 2) for h in sh.get("per_joint", [])], ["Upright: heat W"] + [num(h["heat_W"], 1) for h in sh.get("per_joint", [])],
        ["Leaning: torque N.m"] + [num(h["torque_Nm"], 2) for h in eff.get("servos_hover_lean20", {}).get("per_joint", [])], ["Leaning: heat W"] + [num(h["heat_W"], 1) for h in eff.get("servos_hover_lean20", {}).get("per_joint", [])]], "num",
                note="Servo current in the hover pose = torque / (stall torque / stall current); a holding servo turns it all to heat (I²R plus 0.12 A of electronics at 12 V)."))

# thermal
sec("thermal", "Thermal",
    table(["Servo", "Winding Ω", "N.m per A", "Continuous torque N.m", "Share of stall", "Time constant s"],
          [[t["servo"], num(t["R_ohm"], 2), num(t["kt_Nm_per_A"], 3), num(t["continuous_torque_Nm"], 2), f"{t['continuous_share_of_stall']:.0%}", num(t["thermal_time_constant_s"], 0)] for t in SY.get("thermal_servo_limits", [])], "num")
    + table(["Joint", "Upright: heat W", "Upright: minutes to +50 K", "Leaning 20 deg: heat W", "Leaning: minutes to +50 K"],
            [[t["joint"], num(t["heat_W"], 1), esc(str(t["minutes_to_50K"])), num(u["heat_W"], 1) if u else "–", esc(str(u["minutes_to_50K"])) if u else "–"] for t, u in zip(th, thL or [None] * len(th))], "num",
            note=esc(SY.get("thermal_note", "")) + f" Fan motor: about {SY.get('thermal_fan_motor',{}).get('hover_heat_W','?')} W of heat at hover.")
    + "<p class='tnote'>Keep sustained loads at about a quarter of stall: that is the same line as your 2x rule for the root joints, and a tighter one for the outer joints.</p>")

# resilience
dr = res.get("drop", {}); tf = res.get("tip_force_limits_hover", {}); lj = res.get("limp_joint_in_hover", []); bo = res.get("brownout", {}); of = res.get("one_fan_out", {}); cp = res.get("central_prop_out", {})
sec("resil", "Resilience",
    "<h3>Dropped onto the folded pods</h3>" + table(["Drop height m", "Fall time s", "Peak foot force N", "Worst need"], [[num(r["drop_m"], 2), num(r["impact_s"], 2), num(r["peak_foot_N"], 0), needc(r["worst_need"])] for r in dr.get("rows", [])], "num",
                                                  note=f"Foot vertical stiffness {dr.get('foot_vertical_stiffness_N_per_mm','?')} N/mm from the servo springs. " + esc(dr.get("note", "")))
    + "<h3>Crash and ground strike, in the leaning hover pose</h3>" + table(["Tip force direction", "Servo stalls at N", "Back-drives at N", "Printed ear breaks at N"],
        [[esc(k), f"{v.get('servo stall','?')} (joint {v.get('servo stall joint','?')})", f"{v.get('back-drive at 1.5x stall [A]','?')} (joint {v.get('back-drive at 1.5x stall [A] joint','?')})", f"{v.get('PETG ear breaks (25 MPa across layers)','?')} (joint {v.get('ear joint','?')})"] for k, v in tf.items()],
        note=esc(res.get("tip_force_note", "")))
    + "<h3>Losing a servo, a fan or power</h3>" + table(["Failure", "Result"], [
        ["One servo goes limp in hover", "; ".join(f"joint {r['joint']}: {'arm can re-pose and hold' if r['hover_holdable'] else 'cannot hold, cut that fan and fold'}" for r in lj)],
        ["One claw fan out", f"{'Hover holds' if of.get('feasible') else 'Cannot hover'}: the worst remaining fan runs at {of.get('worst_fan_share_of_full',0):.0%} of full, central prop {of.get('central_N','?')} N." if of else "–"],
        ["Central prop out", f"{'Hover holds' if cp.get('feasible') else 'Cannot hover'}: each claw fan needs {cp.get('fan_N_needed','?')} N ({cp.get('share_of_full',0):.0%} of full), {cp.get('power_W','?')} W in all." if cp else "–"],
        ["Servo bus voltage sags", f"The upright hover pose holds down to {bo.get('min_bus_V_to_hold_hover','?')} V (your 2x rule would need {bo.get('min_bus_V_at_2x_rule','?')} V, above the 12 V supply); the leaning pose holds down to {bo.get('lean20_min_bus_V','?')} V ({bo.get('lean20_min_bus_V_2x','?')} V for 2x). " + esc(bo.get("note", ""))],
        ["Overload and back-drive", esc(res.get("overload", {}).get("note", ""))],
    ], "wide", note=esc(res.get("limp_note", ""))))

# sensitivity
sec("sens", "Sensitivity to the unknowns",
    table(["Tip pod and claw g", "Arm mass g", "Hover (leaning 20 deg) worst need", "Straight out, fan off"], [[num(r["tip_g"], 1), num(r["arm_mass_g"], 0), needc(r["hover_worst"]), needc(r["straight_out_worst"])] for r in sens.get("tip_mass", [])], "num")
    + table(["Hub radius m", "Hover (leaning 20 deg) worst need"], [[num(r["hub_radius_m"], 4), needc(r["hover_worst"])] for r in sens.get("hub_radius", [])], "num", note=esc(sens.get("note", "")) + " Each hover case re-solves the pose from a warm start, so small differences are within the solver's scatter."))

# helix comparison
cmp_rows = [
    ["Hover, fan upright, worst need", needc(FS.get("lean3_hover", {}).get("worst")), needc(hfl.get("hover", {}).get("worst"))],
    ["Rated thrust, fan upright, worst need", needc(FS.get("lean3_rated", {}).get("worst")), needc(hfl.get("rated", {}).get("worst"))],
    ["Straight out, fan off, worst need", needc(straight_need.get("worst")), needc(1.10)],
    ["Walking, worst need at 30 cm feet", needc(STN.get("radii",{}).get("alt90 0.30",{}).get("worst") or float("nan")), needc(STN.get("radii",{}).get("helix60 0.30",{}).get("worst") or float("nan"))],
    ["Walking, worst need at 45 cm feet", needc(STN.get("radii",{}).get("alt90 0.45",{}).get("worst") or float("nan")), needc(STN.get("radii",{}).get("helix60 0.45",{}).get("worst") or float("nan"))],
    ["Workspace", f"{wa.get('volume_m3','?')} m³", f"{wh.get('volume_m3','?')} m³"],
    ["Minimum reach", f"{wa.get('reach_min_m','?')} m", f"{wh.get('reach_min_m','?')} m"],
    ["Pure pitch bend", "7 joints, 196 mm curl radius", "spread over every joint"],
    ["Parts", "one vertebra STL for every moving link", "one vertebra STL, turned 60 deg"],
]
sec("compare", "No helix against the helix", table(["", "No helix (90 deg)", "Helix (60 deg)"], cmp_rows),
    "Same parts, masses and servos; only the hinge directions change. Both columns are solved the same way.")

# assumptions
assum = [
    "Servo stall torque and current are the makers' 12 V figures; servo backlash 0.5 deg; hold stiffness = stall torque per 2 deg of error.",
    "Metal gear trains back-drive or chip at about 1.5 x stall.",
    "Printed PETG: 45 MPa along layers, 25 MPa across, 35 MPa bearing, E 2.0 GPa, fatigue 30 % of strength at a million cycles. Prints vary; test a coupon.",
    "7075-T6: yield 503 MPa, bending fatigue 159 MPa, Basquin 1466 MPa and -0.143.",
    "Servo case sheds about 3 W steadily in still air; 50 K rise allowed.",
    "Hub radius 124.5 mm (the smallest at which the eight root flanges clear); mantle not drawn.",
    "Tip pod 252.5 g with its CG 45 mm beyond the tip plate; real claw mass unknown, swept 150 to 400 g.",
    "Horn screws at 0.5 N.m, friction 0.3, PETG relaxes half the clamp.",
    "Root gear: m1, 20T / 40T, 10 mm face, 98 % efficient; centre held ±0.02 mm only with reamed seats.",
    "Fan thrust and power from the maker's MN5008 16x5.4 table, 5 % derated at 6S under load.",
    "No FEA: ears, pins and bores use beam and bearing formulas with the support reactions split over the two ears 49 mm apart.",
]
sec("assume", "Assumptions [A]", "<ol class='assume'>" + "".join(f"<li>{esc(a)}</li>" for a in assum) + "</ol>")

# ---------------------------------------------------------------- page
toc = "".join(f'<a href="#{i}">{t}</a>' for i, t in [("findings", "Findings"), ("config", "Setup"), ("mass", "Mass"), ("reach", "Reach"), ("torque", "Torque"), ("speed", "Speed"), ("tol", "Tolerances"),
                                                     ("stiff", "Stiffness"), ("stress", "Stress"), ("life", "Life"), ("eff", "Efficiency"), ("thermal", "Thermal"), ("resil", "Resilience"), ("sens", "Sensitivity"), ("compare", "Helix"), ("assume", "Assumptions")])
findings = "".join(f'<li class="f {t}"><b>{esc(h)}</b><p>{esc(b)}</p></li>' for t, h, b in F)
CSS = open(os.path.join(H, "page", "style.css")).read()
page = f"""<title>Cephalo-1 Pin-Hinge Arm</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;500&family=IBM+Plex+Sans+Condensed:wght@500;600&family=IBM+Plex+Sans:wght@400;500;600&display=swap">
<style>{CSS}</style>
<div class="wrap">
<header class="top"><p class="eyebrow">Cephalo-1 octopus drone · main arm · spec sheet</p>
<h1>Pin-hinge arm, no helix</h1>
<p class="sub">1.00 m, 13 joints, hinge axes turning 90 deg per joint. Computed {datetime.date.today():%b %-d, %Y} from the CAD masses and the joint study model.</p>
<nav class="toc">{toc}</nav></header>
<section id="findings"><h2>What the numbers say</h2><ul class="findings">{findings}</ul></section>
{''.join(S)}
<footer><p>Hand calculations and a rigid-body model, no FEA. Numbers marked [A] rest on assumptions listed above. Source: octopus-drone/spec-sheet.</p></footer>
</div>"""
os.makedirs(os.path.join(H, "page"), exist_ok=True)
open(os.path.join(H, "page", "index.html"), "w").write(page)
print("written", len(page))
