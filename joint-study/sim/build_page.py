#!/usr/bin/env python3
"""Build the self-contained preview page from results/ (python3 build_page.py OUT.html)."""
import os, sys, json, base64, html
import pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); RES = os.path.join(HERE, "results") if os.path.isdir(os.path.join(HERE, "results")) else os.path.join(HERE, "..", "results")
_vp = [os.path.join(HERE, "page", "viewer.js"), os.path.join(HERE, "..", "page", "viewer.js")]
VIEWER = open(next(p for p in _vp if os.path.exists(p))).read()
out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(RES, "cephalo-joint-study.html")
summ = json.load(open(os.path.join(RES, "summary.json")))
arms = json.load(open(os.path.join(RES, "top_arms_geometry.json")))
short = pd.read_csv(os.path.join(RES, "shortlist_dynamics.csv"))
tip = json.load(open(os.path.join(RES, "tip_study.json")))
scen = json.load(open(os.path.join(RES, "scenario_comparison.json"))) if os.path.exists(os.path.join(RES, "scenario_comparison.json")) else None
val = pd.read_csv(os.path.join(RES, "validation_vs_earlier_grid.csv"))
MAIN = json.load(open(os.path.join(RES, "main_design.json"))) if os.path.exists(os.path.join(RES, "main_design.json")) else None
FIND = json.load(open(os.path.join(RES, "findings.json"))) if os.path.exists(os.path.join(RES, "findings.json")) else dict(lead="", items=[])
e = html.escape
def f(x, n=1):
    try: return f"{float(x):.{n}f}"
    except Exception: return "–"
def img(name, alt):
    p = os.path.join(RES, name)
    if not os.path.exists(p): return ""
    b = base64.b64encode(open(p, "rb").read()).decode()
    return f'<figure class="plot"><img src="data:image/png;base64,{b}" alt="{e(alt)}"><figcaption>{e(alt)}</figcaption></figure>'
ARCH_SHORT = {"servo in every node": "servo per node", "graded servo chain": "graded servos", "base-box tendons": "tendons",
              "3-section tendons": "3-section", "root servos + base-box tendons": "hybrid", "base-box push-pull": "push-pull"}

main_html = ""
LOPT = json.load(open(os.path.join(RES, "length_options.json"))) if os.path.exists(os.path.join(RES, "length_options.json")) else None
if LOPT:
    def wv(v): return "saturates" if v == "sat" else ("–" if v is None else f(v))
    orow = "".join(f"<tr{' class=hl' if o['key'] == 'b' else ''}><td><b>({o['key']}) {e(o['title'])}</b><span class=sub>{e(o['servos'])} · {e(o['thrust'])}</span></td><td class=num>{o['length_mm']}</td><td class=num>{o['joints']}</td>"
                   f"<td class=num>{f(o['pitch_mm'])}</td><td class=num>{f(o['arm_out_g'],0)}</td><td class=num>{f(o['mass_kg'],2)}</td><td class=num>{f(o['tw'],2)}</td><td class=num>{f(o['miss_mm'])}</td>"
                   f"<td class=num>{wv(o['wave']['0.25'])}</td><td class=num>{wv(o['wave']['0.5'])}</td><td class=num>{wv(o['wave']['1.0'])}</td><td class=num>{f(o['lift_mm'])}</td></tr>" for o in LOPT["options"])
    r5 = LOPT["ref_585"]
    main_html = f"""<section aria-labelledby="h-main"><h2 id="h-main">Main design: 1 m arm, 13 joints, 16 N claw fans, geared root</h2>
  {img('main_design.png','Pin-hinge vertebra and a 9-joint arm (585 mm build); the 1 m and 700 mm builds use the same vertebra stretched to their pitch')}
  <p>The main design is the pin hinge in a 60° helix, 1 m long with 13 joints: STS3250 servos in joints 0 to 4 and STS3215 beyond, one 31 g vertebra printed twelve times per arm, collision-free to ±90° per joint. On today's 10.8 N claw fans no 1 m arm with more than 7 joints passes the thrust-to-weight gate (1.15), so this build was chosen together with claw fans of about 16 N. Joint 0 now drives through a 2:1 gear (see the root fix below). The highlighted row is the main design before that fix; the other two are the options that were compared.</p>
  <div class="tbl"><table><thead><tr><th>Option</th><th>Length mm</th><th>Joints</th><th>Pitch mm</th><th>Arm g</th><th>Robot kg</th><th>T/W</th><th>Shape miss mm (as built)</th><th>Wave RMS 0.25 Hz</th><th>0.5 Hz</th><th>1 Hz</th><th>Lift RMS mm</th></tr></thead><tbody>{orow}</tbody></table></div>
  <div class="stats"><span><b>16 N</b> per claw fan, 13.2 N at hover</span><span>7 to 7.5 inch prop, 2807 motor, 6S</span><span>about <b>270 W</b> per fan at hover, 2.2 kW for eight</span><span>45 A ESC</span><span>130 g tip budget</span></div>
  <p class="note">Claw fan spec, sized from the T-Motor 2807 1250KV bench table on a 7.5 inch prop at 6S: 16 N at about 76 % throttle and 21.4 N at full. A 5 inch 2207 tops out at 15.1 N, so it cannot hold 13.2 N for hover with margin. The pan-tilt tip wrist keeps 0.62 thrust-sphere coverage with a 190 mm disc at a 60 mm offset (0.72 for the old 90 mm disc). Nothing has been bought.</p>
  <p class="note">Arm g is the arm outside the mantle, from the CAD. Shape miss is for the arm as built, with every joint at the same pitch, so it runs a little above the sweep's figure for the same joint count. Wave RMS is the tip error in mm when the arm tracks a travelling bend at that frequency; "saturates" means the servos hit their torque limit for most of the test, and a dash means that frequency was not run. For reference, the 585 mm build: {r5['joints']} joints, {f(r5['mass_kg'],2)} kg, T/W {f(r5['tw'],2)}, {f(r5['miss_mm'])} mm miss. Extra thrust a 1 m arm needs over today's: about 4 % for 9 joints, 19 % for 13, 31 % for 16. CAD and STLs for all three are in <code>octopus-drone/arm-pin-hinge/</code> (<code>out_1m_7</code>, <code>out_1m_13</code>, <code>out_700_9</code>).</p>
</section>"""
RF = json.load(open(os.path.join(RES, "root_fix.json"))) if os.path.exists(os.path.join(RES, "root_fix.json")) else None
BL = json.load(open(os.path.join(RES, "bearing_loads.json"))) if os.path.exists(os.path.join(RES, "bearing_loads.json")) else None
bearing_html = ""
if BL:
    old, new = BL["as drawn"]["rating"], BL["after the bearing check"]["rating"]
    def sf(x): return f"{x:.2f}" if x >= 1.0 else f"<span class=bad>{x:.2f}</span>"
    br = "".join(f"<tr><td>{e(k)}</td><td class=num>{old[k]['gear_brg_N']:,}</td><td class=num>{sf(old[k]['gear_brg_s0'])}</td><td class=num>{new[k]['gear_brg_N']:,}</td><td class=num>{sf(new[k]['gear_brg_s0'])}</td>"
                 f"<td class=num>{old[k]['idler_bend_MPa']:.0f}</td><td class=num>{new[k]['idler_bend_MPa']:.0f}</td><td class=num>{sf(old[k]['pin_brg_s0'])}</td><td class=num>{sf(new[k]['pin_brg_s0'])}</td></tr>" for k in old)
    bearing_html = ('<div class="tbl"><table><thead><tr><th>Load case</th><th>6800 N</th><th>6800 safety</th><th>61803 N</th><th>61803 safety</th>'
                    '<th>PETG stub MPa</th><th>7075 pin MPa</th><th>685 safety</th><th>695 safety</th></tr></thead><tbody>' + br + '</tbody></table></div>')
JL = json.load(open(os.path.join(RES, "joint_loads.json"))) if os.path.exists(os.path.join(RES, "joint_loads.json")) else None
joints_html = ""
if JL:
    rated = JL["rated"]; cases = list(rated["joint 1"])
    def wst(c, k): return max(rated[j][c][k] for j in rated if c in rated[j])
    def stub(x): return f"{x:.1f}" if x <= 12.5 else f"<span class=bad>{x:.1f}</span>"
    jr = "".join(f"<tr><td>{e(c.replace(' deg', '°'))}</td><td class=num><span class=bad>{wst(c,'horn_N')}</span></td><td class=num>{wst(c,'idler_N')}</td><td class=num>{stub(wst(c,'stub_bend_MPa'))}</td>"
                 f"<td class=num>{max(wst(c,'horn_ear_MPa'), wst(c,'idler_ear_MPa')):.1f}</td></tr>" for c in cases)
    joints_html = ('<h3>Joints 1 to 4 side-load check</h3><div class="tbl"><table><thead><tr><th>Case (worst of joints 1 to 4)</th><th>Servo output radial N</th><th>Idler side N</th>'
                   '<th>Printed stub MPa</th><th>Fork ears MPa</th></tr></thead><tbody>' + jr + '</tbody></table></div>'
                   '<p class="note">Joints 1 to 4 have no gear, so the side load goes through the fork\'s two ears, 49 mm apart: one on the servo horn (the STS3250 output spline and its ball bearing), one on a printed 8 mm stub. Printed parts were rated against about 45 MPa along the layers and 25 MPa across them with Forest\'s 2x rule; the stub passes only with the claw fan held within 30° of straight up. The servo output carries 110 N even in hover; Feetech publishes no limit, and ROBOTIS rates its heavier XM430 at 40 N radial 10 mm from the horn. Taking that load off the servo needs a thin-section bearing between the horn ear and the cradle lid (roughly +20 to +40 g per arm) or a bench test first. Script: <code>joint-study/sim/joint_loads.py</code>, <code>results/joint_loads.json</code>.</p>')
if RF and main_html:
    def gate(t): return f"{t:.3f}" if t >= RF["gate"] else f"<span class=bad>{t:.3f}</span>"
    rrow = "".join(f"<tr{' class=hl' if r['pick'] else ''}><td>{'<b>' if r['pick'] else ''}{e(r['variant'])}{'</b>' if r['pick'] else ''}</td><td class=num>{f(r['added_g'],0)}</td><td class=num>{gate(r['tw'])}</td>"
                   f"<td class=num>{f(r['lift_mm'])}</td><td class=num>{f(r['curl_mm'])}</td><td class=num>{'–' if r['wave05'] is None else f(r['wave05'])}</td><td class=num>{f(r['root_speed'],1)}</td></tr>" for r in RF["rows"])
    main_html = main_html.replace("</section>", f"""  <h3>Root fix: 2:1 gear on joint 0</h3>
  <p>The 345 mm lift-test miss came from the start of the test: the arm begins straight out and level with the fans off and is commanded to the walking stance in one jump. Straight out, gravity needs 7.2 N·m at joint 0 against the STS3250's 4.9 N·m, so the arm folds and never reaches the stance. Started from the stance, the same arm lifts its foot to 13 mm. The adopted fix drives joint 0 through a 2:1 spur stage (9.8 N·m, 2.2 rad/s at the joint) and keeps every command under 80 % of the servo's rated speed. 36 variants were run; the main ones are below.</p>
  <div class="stats"><span>lift <b>5.9 mm</b> (was 345)</span><span>curl step <b>86 mm</b> (was 386)</span><span>wave 0.5 Hz <b>62 mm</b> (was 106)</span><span>+46.2 g per arm in CAD, robot 15.08 kg</span><span>T/W <b>1.150</b></span></div>
  <div class="tbl"><table><thead><tr><th>Fix</th><th>Added g per arm</th><th>T/W</th><th>Lift RMS mm</th><th>Curl step miss mm</th><th>Wave 0.5 Hz mm</th><th>Slowest root joint rad/s</th></tr></thead><tbody>{rrow}</tbody></table></div>
  <p class="note">Gear stage (spec, nothing bought): module 1, 20T pinion on the STS3250 horn, 40T sector gear (teeth over ±65°, hard stops at ±55°) on the joint-0 pin, 30 mm centres, 10 mm face, 7075-T6 hard-anodized, a 61803 bearing under the gear and a 695 on the pinion. At STS3250 stall the teeth see 490 N, about 153 MPa bending on the pinion. The root socket is now drawn around it: the stage adds 46.2 g per arm in the CAD, inside the 46.4 g budget that keeps T/W at 1.15. A claw fan giving 4 N only while the arm rises gets the lift to 14 mm with no hardware but leaves the curl step and the wave as they were. A torsion spring fights the servo in every pose but one, and bigger servos (Feetech STS3095, ROBOTIS XM540-W270) or a second STS3250 push T/W below 1.15 (red). Runs: <code>joint-study/sim/fix_root.py</code>, <code>results/root_fix.csv</code>.</p>
  <h3>Root socket, redrawn around the gear</h3>
  <div class="stats"><span>added <b>46.2 g</b> per arm (budget 46.4)</span><span>joint 0 free <b>±55°</b> to the stops</span><span>joints 0+1 clear to <b>±60°</b></span><span>servo bulge <b>6.8 mm</b> on one side</span></div>
  <table><thead><tr><th>Part</th><th class=num>g</th><th>Note</th></tr></thead><tbody>
  <tr><td>Socket, printed</td><td class=num>30.1</td><td>was 23.2; servo slides into a sleeve 30 mm below the hinge</td></tr>
  <tr><td>End cap and bearing cap, printed</td><td class=num>4.0</td><td>joint 0 no longer has a lid (−2.5 g)</td></tr>
  <tr><td>40T sector gear, 7075-T6</td><td class=num>15.0</td><td>fork's horn ear bolts to its hub, same 4 × M3 circle; a 61803 sits in its inboard face</td></tr>
  <tr><td>20T pinion and idler pin, 7075-T6</td><td class=num>7.7</td><td>pinion bolted to the servo horn, far end in a 695; 8 mm pin for the fork's idler ear</td></tr>
  <tr><td>Bearings and hardware</td><td class=num>15.1</td><td>61803, 695, screws and inserts</td></tr>
  </tbody></table>
  <p class="note">The vertebrae, tip and fork are unchanged. The mantle opening needs a notch on the servo side: everything that passes through it fits a 61 × 42 mm outline (<code>out_1m_13/mantle_cutout_joint0_frame.svg</code>). A full webbed gear instead of the sector fits the same socket at 54 to 59 g per arm (T/W 1.146 to 1.142). Script: <code>arm-pin-hinge/root_gear.py</code>.</p>
  <h3>Root bearing check</h3>
  <div class="stats"><span>6800 static safety <b>0.73</b> → 61803 <b>1.31</b> at 16 N sideways</span><span>idler support <b>36 → 26 MPa</b> (PETG → 7075)</span><span>pinion bearing <b>1.12 → 1.68</b> at stall</span><span>mass <b>+0.9 g</b> per arm, inside budget</span></div>
  {bearing_html}
  <p class="note">When the claw fan pushes across the hinge plane at the root, up to 16 N.m (rated thrust) or 22 N.m (full throttle) acts out of that plane, and the socket as first drawn carried it on a 6800 and a printed stub 31 mm apart, with the 6800 8 mm inboard of the mesh. The fix: a 61803 in a recess in the gear's inboard face, 2.5 mm from the mesh and 36.5 mm from the idler ear; an 8 mm 7075 pin for the idler ear; a 695 on the pinion. Static safety is C0 over the worst load (catalogue ratings). Still weak: the fork's printed idler ear (24 MPa at full throttle sideways) and the servo spline's side load (about 280 N at stall, no published limit). Joints 1 to 4 are checked below. Keeping each claw fan's push within 30° of straight up holds the root under 3 N.m out of plane. Script: <code>joint-study/sim/bearing_loads.py</code>, <code>results/bearing_loads.json</code>.</p>
  {joints_html}
</section>""", 1)

rows = []
for i, r in enumerate(short.head(12).itertuples(), 1):
    servo = r.local_servo if isinstance(r.local_servo, str) and r.local_servo else (r.remote_drive if isinstance(r.remote_drive, str) else "")
    if r.actuation == "graded": servo = f"{r.root_servo} → STS3215 → {r.local_servo}"
    rows.append(f"<tr><td class=num>{i}</td><td><b>{e(r.family_name)}</b><span class=sub>{e(str(r.variant))} · {e(r.arrangement)} · {e(str(r.compliance))}</span></td>"
                f"<td>{e(ARCH_SHORT.get(r.arch, r.arch))}<span class=sub>{e(str(servo).replace('_12V',''))}</span></td>"
                f"<td class=num>{int(r.N)}</td><td class=num>{int(r.dof)}</td><td class=num>{f(r.miss_mean_mm)}</td><td class=num>{f(r.wave_tip_rms_mm)}</td>"
                f"<td class=num>{f(r.util_walk,2)}</td><td class=num>{f(r.touchdown_v_safe,2)}</td><td class=num>{f(r.total_mass_kg,2)}</td>"
                f"<td class=num>{f(r.cost_usd,0)}</td><td class=num>{f(r.assembly_h,0)}</td><td class=num><b>{f(r.final_score,3)}</b></td></tr>")
top_table = "\n".join(rows)

mech = sorted(tip["mechanisms"], key=lambda m: -m.get("score", 0))
tip_rows = "\n".join(
    f"<tr><td><b>{e(m.get('name', m.get('id','')))}</b></td><td class=num>{m.get('actuators','')} ({m.get('actuators_at_tip','')} at tip)</td>"
    f"<td class=num>{f(m.get('coverage_sphere',0),2)}</td><td class=num>{f(m.get('moving_mass_g',0),0)}</td><td class=num>{f(m.get('extra_root_moment_Nm',0),2)}</td><td class=num>{f(m.get('score',0),3)}</td></tr>"
    for m in mech[:12])

scen_html = ""
if scen:
    b = {r["arch"]: r for r in scen["baseline"]}; o = {r["arch"]: r for r in scen["tendon_optimistic"]}; m = {r["arch"]: r for r in scen.get("more_thrust", [])}
    sr = []
    for a in b:
        sr.append(f"<tr><td><b>{e(a)}</b></td><td class=num>{b[a]['feasible']:,} / {b[a]['configs']:,}</td><td class=num>{f(b[a]['best_score'],3) if b[a]['best_score'] else '–'}</td>"
                  f"<td class=num>{o.get(a,{}).get('feasible',0):,}</td><td class=num>{f(o.get(a,{}).get('best_score'),3) if o.get(a,{}).get('best_score') else '–'}</td>"
                  f"<td class=num>{m.get(a,{}).get('feasible',0):,}</td><td class=num>{f(m.get(a,{}).get('best_score'),3) if m.get(a,{}).get('best_score') else '–'}</td><td class=num>{f(b[a]['min_walk_util'],2)}</td><td>{e(', '.join(list(b[a]['top_fail'])[:2]))}</td></tr>")
    scen_html = "\n".join(sr)

val_rows = "\n".join(f"<tr><td>{e(str(r[0]))}</td>" + "".join(f"<td class=num>{f(x)}</td>" for x in r[1:]) + "</tr>" for r in val.itertuples(index=False))
val_head = "".join(f"<th>{e(c.replace('_',' '))}</th>" for c in val.columns)
def _btn(a):
    lab = a["label"]
    pre = "Most DOF: " if lab.startswith("Most DOF") else ("Main: " if lab.startswith("Main design") else "")
    core = lab.replace("Most DOF (feasible) | ", "").replace("Main design | ", "").split(" | ")[0].split(" (")[0]
    return pre + core
arm_buttons = "".join(f'<button type="button" data-arm="{i}" aria-pressed="{str(i==0).lower()}">{e(_btn(a))}</button>' for i, a in enumerate(arms))
POSES = [("walking_stance", "Walking stance"), ("straight", "Straight"), ("arc_curl", "270° curl"), ("spiral_curl", "Spiral"),
         ("helix_coil", "Coil on pole"), ("s_curve", "S-curve"), ("twisted_bend", "Twisted bend")]
pose_buttons = "".join(f'<button type="button" data-pose="{k}" aria-pressed="{str(k=="walking_stance").lower()}">{v}</button>' for k, v in POSES)
find_items = "".join(f"<li>{x}</li>" for x in FIND["items"])
fc = summ.get("fail_counts", {})
fail_list = " · ".join(f"{e(k.replace('_',' '))} {v:,}" for k, v in list(fc.items())[:8])

page = f"""<title>Cephalo-1 Joint Study</title>
<meta name="claude:viewport-fit-sides" content="cover">
<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Barlow+Condensed:wght@500;600&family=IBM+Plex+Sans:wght@400;500;600&family=IBM+Plex+Mono:wght@400;500&display=swap">
<style>
/* layout: lab-notebook column, 3D bench on top, tables below; colours keyed to PETG prints and servo housings */
:root {{
  --bg:#f3f5f4; --panel:#ffffff; --fg:#17201d; --muted:#5d6a66; --line:#d5dcd9;
  --arm:#d9822b; --servo:#2a6fdb; --target:#0f8b8d; --accent:#0f6f71; --paper:#ffffff; --bad:#b4372a;
  --display:"Barlow Condensed","Arial Narrow",sans-serif; --body:"IBM Plex Sans",system-ui,sans-serif; --mono:"IBM Plex Mono",ui-monospace,monospace;
}}
@media (prefers-color-scheme: dark) {{ :root:not([data-theme="light"]) {{
  --bg:#111715; --panel:#18201e; --fg:#e4ebe8; --muted:#93a29d; --line:#2b3633; --arm:#f0a050; --servo:#6d9cf0; --target:#3fc1c2; --accent:#5cc9ca; --paper:#f4f6f5; --bad:#f07a6a; color-scheme:dark }} }}
:root[data-theme="dark"] {{ --bg:#111715; --panel:#18201e; --fg:#e4ebe8; --muted:#93a29d; --line:#2b3633; --arm:#f0a050; --servo:#6d9cf0; --target:#3fc1c2; --accent:#5cc9ca; --paper:#f4f6f5; --bad:#f07a6a; color-scheme:dark }}
body {{ background:var(--bg); color:var(--fg); font:15px/1.55 var(--body); }}
.wrap {{ max-width:1080px; margin:0 auto; padding-block:28px 64px; padding-left:max(16px, env(safe-area-inset-left,0px)); padding-right:max(16px, env(safe-area-inset-right,0px)); display:grid; gap:36px; }}
header {{ display:grid; gap:8px; }}
.eyebrow {{ font:500 12px/1 var(--mono); letter-spacing:.08em; text-transform:uppercase; color:var(--muted); }}
h1 {{ font:600 clamp(34px,6vw,54px)/1 var(--display); letter-spacing:.01em; margin:0; text-wrap:balance; }}
h2 {{ font:600 26px/1.1 var(--display); margin:0 0 10px; letter-spacing:.02em; text-wrap:balance; }}
.lead {{ font-size:17px; max-width:68ch; margin:0; }}
.stats {{ display:flex; flex-wrap:wrap; gap:8px 22px; font:400 13px/1.4 var(--mono); color:var(--muted); }}
.stats b {{ color:var(--fg); font-weight:500; }}
section {{ display:grid; gap:12px; min-width:0; }}
p, li {{ max-width:72ch; }}
ul.find {{ margin:0; padding-left:20px; display:grid; gap:6px; }}
.bench {{ display:grid; grid-template-columns:minmax(0,1fr) 260px; gap:16px; }}
@media (max-width:760px) {{ .bench {{ grid-template-columns:1fr; }} }}
#stage {{ height:440px; background:var(--panel); border:1px solid var(--line); border-radius:6px; overflow:hidden; position:relative; }}
#stage canvas {{ display:block; }}
.fallback {{ padding:24px; color:var(--muted); }}
.ctl {{ display:grid; gap:14px; align-content:start; min-width:0; }}
.ctl h3 {{ font:500 12px/1 var(--mono); letter-spacing:.08em; text-transform:uppercase; color:var(--muted); margin:0 0 6px; }}
.btns {{ display:flex; flex-wrap:wrap; gap:6px; }}
.btns button {{ font:500 13px/1.2 var(--body); color:var(--fg); background:transparent; border:1px solid var(--line); border-radius:4px; padding:6px 10px; cursor:pointer; text-align:left; }}
.btns button[aria-pressed="true"] {{ background:var(--accent); border-color:var(--accent); color:var(--bg); }}
.btns button:focus-visible {{ outline:2px solid var(--accent); outline-offset:2px; }}
#armLabel {{ font:400 12px/1.45 var(--mono); color:var(--muted); overflow-wrap:anywhere; }}
.key {{ display:flex; flex-wrap:wrap; gap:6px 14px; font-size:13px; color:var(--muted); }}
.key i {{ display:inline-block; width:10px; height:10px; border-radius:2px; margin-right:6px; vertical-align:-1px; }}
.tbl {{ overflow-x:auto; border:1px solid var(--line); border-radius:6px; background:var(--panel); }}
table {{ border-collapse:collapse; width:100%; font-size:13px; }}
th, td {{ padding:8px 10px; border-bottom:1px solid var(--line); text-align:left; vertical-align:top; }}
th {{ font:500 11px/1.3 var(--mono); letter-spacing:.05em; text-transform:uppercase; color:var(--muted); white-space:nowrap; }}
td.num {{ font-family:var(--mono); font-variant-numeric:tabular-nums; text-align:right; white-space:nowrap; }}
tr.hl td {{ background:color-mix(in srgb, var(--accent) 14%, transparent); }}
tr:last-child td {{ border-bottom:0; }}
.sub {{ display:block; color:var(--muted); font-size:12px; }}
.note {{ font-size:13px; color:var(--muted); }}
.plots {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(300px,1fr)); gap:16px; }}
figure.plot {{ margin:0; background:var(--paper); border:1px solid var(--line); border-radius:6px; padding:8px; min-width:0; }}
figure.plot img {{ display:block; width:100%; height:auto; }}
figure.plot figcaption {{ font-size:12px; color:#4a5552; padding:6px 4px 0; }}
code {{ font-family:var(--mono); font-size:13px; }}
@media (prefers-reduced-motion: reduce) {{ * {{ scroll-behavior:auto; }} }}
.bad {{ color:var(--bad); font-weight:600 }}
section > h3 {{ font:600 21px/1.2 var(--display); margin:28px 0 8px; }}
</style>
<div class="wrap">
<header>
  <div class="eyebrow">Cephalo-1 · 8 arms · pin hinge, 60° helix · arm length 1 m (Forest, Nov 9)</div>
  <h1>Cephalo-1 Joint Study</h1>
  <p class="lead">{FIND['lead']}</p>
  <div class="stats"><span><b>{summ['configs']:,}</b> arm designs</span><span><b>{summ['joint_types']}</b> joint types</span><span><b>{summ['families']}</b> families</span>
  <span><b>{summ['feasible']:,}</b> pass every gate</span><span><b>{summ.get('pareto_size','–')}</b> on the Pareto front</span><span>Spearman vs earlier grid <b>{f(summ.get('validation_spearman',0),2)}</b></span></div>
</header>
{main_html}
<section aria-labelledby="h-find"><h2 id="h-find">What the run found</h2><ul class="find">{find_items}</ul></section>
<section aria-labelledby="h-bench"><h2 id="h-bench">Arm bench</h2>
  <div class="bench">
    <div id="stage" role="img" aria-label="3D view of the selected arm on the hub"></div>
    <div class="ctl">
      <div><h3>Arm</h3><div class="btns">{arm_buttons}</div></div>
      <div><h3>Pose</h3><div class="btns">{pose_buttons}</div></div>
      <div id="armLabel"></div>
      <div class="key"><span><i style="background:var(--servo)"></i>servo node</span><span><i style="background:var(--arm)"></i>passive / tendon node</span><span><i style="background:var(--target)"></i>target shape</span></div>
      <p class="note">Drag to orbit, scroll to zoom. Poses are the solver's best fit to each target shape; walking stance is the claw-down foot pose.</p>
    </div>
  </div>
</section>
<section aria-labelledby="h-top"><h2 id="h-top">Top arms after dynamics</h2>
  <div class="tbl"><table><thead><tr><th>#</th><th>Joint</th><th>Drive</th><th>Nodes</th><th>DOF</th><th>Shape miss mm</th><th>Wave tip RMS mm</th><th>Walk torque util</th><th>Safe touchdown m/s</th><th>Robot kg</th><th>Cost $</th><th>Build h</th><th>Score</th></tr></thead>
  <tbody>{top_table}</tbody></table></div>
  <p class="note">At 1 m every arm saturates in the 1 Hz wave test, so the dynamics factor is small for all of them and the order mostly follows the static score; the slower wave tests in the main-design table above are the fairer measure at this length. Score is the static composite (DOF, fluidity, loads, precision, mass, build effort) times the MuJoCo dynamics factor. Walk util is peak joint torque over capacity in claw-down stance (under 1 passes). Safe touchdown is the vertical speed the arm absorbs with fans holding hover weight; an unpowered 150 mm drop breaks every design.</p>
</section>
<section aria-labelledby="h-arch"><h2 id="h-arch">Architecture check at 1 m: baseline, low-friction tendons, bigger claw fans</h2>
  <div class="tbl"><table><thead><tr><th>Architecture</th><th>Feasible (baseline)</th><th>Best score</th><th>Feasible (μ 0.07, 300 N, 400 cm³)</th><th>Best score</th><th>Feasible (16 N claw fans)</th><th>Best score</th><th>Min walk util</th><th>Main failures</th></tr></thead><tbody>{scen_html}</tbody></table></div>
  <p class="note">The second scenario gives tendons PTFE-lined friction, 300 N line tension and two thirds more mantle space, to see whether the remote-drive families are ruled out by the assumptions or by the physics. The third gives each claw fan 16 N instead of 10.8 N and charges 40 g more per claw. Gate failures over the whole baseline sweep: {fail_list}.</p>
</section>
<section aria-labelledby="h-tip"><h2 id="h-tip">Tip fan joint</h2>
  <div class="tbl"><table><thead><tr><th>Wrist</th><th>Actuators</th><th>Thrust-sphere coverage</th><th>Moving mass g</th><th>Added root moment N·m</th><th>Score</th></tr></thead><tbody>{tip_rows}</tbody></table></div>
  <p class="note">Coverage is the share of thrust directions reachable without the 90 mm fan disc hitting the tentacle. The geometric ceiling is 0.56 at 20 mm offset and 0.83 at 60 mm, so offset matters as much as the mechanism.</p>
</section>
<section aria-labelledby="h-plots"><h2 id="h-plots">Plots</h2>
  <div class="plots">{img('dof_vs_fluidity.png','DOF against shape fluidity, all feasible designs')}{img('top15_subscores.png','Sub-scores of the top 15')}{img('family_by_architecture.png','Best score per family and architecture')}{img('mass_vs_dof.png','Robot mass against DOF')}</div>
</section>
<section aria-labelledby="h-val"><h2 id="h-val">Check against the earlier 9-type grid</h2>
  <div class="tbl"><table><thead><tr>{val_head}</tr></thead><tbody>{val_rows}</tbody></table></div>
</section>
<section aria-labelledby="h-run"><h2 id="h-run">Run it on Saturn</h2>
  <p>Code, catalog and CSVs are in the project folder under <code>octopus-drone/joint-study/</code>. On Saturn: <code>pip install -r requirements.txt</code>, then <code>python3 run_study.py</code> (12 workers by default), <code>python3 rank.py</code> and <code>python3 build_page.py</code>. Add <code>JOINT_SCENARIO=tendon_optimistic</code> for the low-friction case.</p>
</section>
</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script>window.ARMS = {json.dumps(arms)};</script>
<script>{VIEWER}</script>
"""
open(out, "w").write(page)
print("wrote", out, len(page) // 1024, "KB")
