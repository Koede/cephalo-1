"""Revolver-barrel multi-tool hand for Cephalo-2: options design study (before any CAD).

Question: how is the chosen tool stored and brought onto the centre axis, so that it comes out of a centre barrel,
is driven there, and goes back to its own slot?  (Forest, 2026-11-20/21: tools stored inside, one extended at a time,
"a center barrel that will rack in the right tool and retract it back into its correct slot".)

Frame: z = tool / wrist axis, z = 0 = bottom face of the coupler's hand base (rev B.7), +z away from the arm.  mm, g, N.
Every option shares one front end (see FRONT): five tool cartridges (printed sleeve + stock 1/4 in hex tool), a spinning
spindle (the barrel) on two thin bearings, driven 1:1 by the spin servo as in the selector-cam hand.

Outputs: toolhand_barrel_study.json (numbers), toolhand_barrel_study.png (figure), toolhand_barrel_scene.json (3D sketch
primitives for the rotatable page).  Numbers marked est. are estimates.
"""
import json, math
import numpy as np

G = 9.81
RHO = 1.20e-3          # g/mm^3, carbon-fibre-filled resin (est.)

# ------------------------------------------------------------------ shared requirements and parts
REQ = dict(
    mass_target_g=335, mass_cap_g=350,
    tool_change_target_s=1.0, spin_torque_sustained_Nm=1.0, spin_torque_peak_Nm=1.96,
    push_on_tool_N=8.0, pull_on_tool_N=15.0, side_load_at_tip_N=5.0,
    pin_current_A=5.2, coupler_flange_r=20.6, collar_r=28.3, collar_hang=4.5,
    arm_continuous_Nm=1.0,
)
SERVOS = {   # datasheet values as read for the selector-cam hand's BOM (2026-11-20)
    "ST3215-HS": dict(mass=68.0, V=12.0, stall_Nm=1.96, rated_Nm=0.65, rpm=106.0, stall_A=2.4, box=(45.2, 24.7, 35.0)),
    "STS3032": dict(mass=20.0, V=6.0, stall_Nm=0.44, rated_Nm=0.147, rpm=111.0, stall_A=1.2, box=(23.2, 12.1, 28.5)),
}
TOOLS = [  # name, mass of the bought tool (g, est. where the page gave none), length that sticks out of the cartridge when stored
    ("Bit holder + Phillips bit (Wera 893/4/1 K)", 22.0 + 4.0, 55.0),
    ("1/4 in square socket drive (Wera 870/1) + socket", 9.0 + 12.0, 38.0),
    ("Mini keyless chuck 0.3-3.2 + drill / scriber", 18.0 + 2.0, 55.0),
    ("Magnet poker with hook / pry tip", 8.0, 40.0),
    ("Spare bit held straight in a magnetic cartridge (1/4 in insert bit + 6x3 mm magnet)", 4.0 + 0.6, 22.0),
]
CART = dict(D=15.0, L=24.0, mass=5.5, key_r=7.0, keys=2, key_w=3.0)       # printed cartridge sleeve (holds the 1/4 in hex shank)
CH_L = CART["L"] + max(t[2] for t in TOOLS) + 3.0                          # chamber length (82 mm)
CH_BORE = CART["D"] + 0.6
FRONT = dict(sp_L=30.0, sp_OD=21.0, lip_bore=12.6, brg="6704-2RS 20x27x4 (est. 5 g)", brg_mass=5.0, brg_n=2,
             gear_mass=6.0, tool_out=CART["L"] + 55.0 - 30.0 - 0.0)          # tool tip beyond the spindle nose when deployed
PITCH = CART["D"] + 2.0                                                    # chamber centre spacing (1 mm wall each side)
N = len(TOOLS)
E_RING = PITCH / (2 * math.sin(math.pi / N))                               # radius of a 5-chamber ring (14.5 mm)
TOOL_MASS = sum(t[1] for t in TOOLS) + N * CART["mass"]


def shell_mass(R, L, t=1.6):
    """Printed thin-walled housing: side wall + two end walls (est.)."""
    return RHO * (2 * math.pi * R * L * t + 2 * math.pi * R * R * t * 0.6)


def stroke_time(d, v, a=3000.0):
    """Trapezoidal move: distance d (mm) at top speed v (mm/s), acceleration a (mm/s^2)."""
    if d <= v * v / a:
        return 2 * math.sqrt(d / a)
    return d / v + v / a


def turn_time(deg, rpm, settle=0.05):
    return deg / (rpm * 6.0) + settle


# ------------------------------------------------------------------ 3D sketch primitives
def cyl(p0, p1, r, c, label=""):
    return dict(t="cyl", p0=[float(x) for x in p0], p1=[float(x) for x in p1], r=float(r), c=c, label=label)


def box(c0, size, c, label=""):
    return dict(t="box", c=[float(x) for x in c0], s=[float(x) for x in size], col=c, label=label)


COL = dict(base="#8a8f98", housing="#b9c2cc", servo="#2f3a4a", tool="#d08a2c", tool_on="#e0522f", spindle="#3d7fd1",
           cam="#7a4fc9", feed="#2e9c6a", warn="#d23c3c")


def coupler_prims():
    return [cyl((0, 0, -8.3), (0, 0, 0), REQ["coupler_flange_r"], COL["base"], "coupler hand base (rev B.7)"),
            cyl((0, 0, 0), (0, 0, REQ["collar_hang"]), 20.3, COL["base"], "clearance boss under the collar")]


def front_prims(z0, out=True):
    """Spindle (barrel) from z0 and the deployed tool (if out)."""
    s = [cyl((0, 0, z0), (0, 0, z0 + FRONT["sp_L"]), FRONT["sp_OD"] / 2, COL["spindle"], "spinning barrel (spindle)")]
    if out:
        s.append(cyl((0, 0, z0 + FRONT["sp_L"]), (0, 0, z0 + FRONT["sp_L"] + FRONT["tool_out"]), 5.25, COL["tool_on"], "tool out"))
    return s


# ------------------------------------------------------------------ options
def opt_keep():
    """O0: keep the selector-cam fingers as built (numbers from that build's checks, 2026-11-21)."""
    s = coupler_prims()
    s.append(cyl((0, 0, 4.5), (0, 0, 108), 25.0, COL["housing"], "spin servo + drum housing"))
    for k in range(4):
        a = math.radians(90 * k); x, y = 25.5 * math.cos(a), 25.5 * math.sin(a)
        if k == 0:
            s.append(cyl((x * 0.2, y * 0.2, 108), (0, 0, 194), 5.25, COL["tool_on"], "raised finger"))
        else:
            s.append(cyl((x, y, 108), (x * 1.25, y * 1.25, 40), 5.25, COL["tool"], "folded finger"))
    return dict(key="O0", name="Keep the selector cam (as built)", short="Keep the selector cam",
                principle="Four fingers on a spinning wrist; one drum cam swings one finger up onto the axis, the rest fold back outside.",
                mass_g=322.0, tool_tip_z=194.0, env_r=33.0, env_len=194.0, com_z=88.0, com_x=0.0,
                change_s=(1.1, 6.2), park_to_tool_s=1.1, motors=["ST3215-HS spin", "STS3032 selector (on the rotor)"],
                stored_inside=False, from_centre=True, kill=[], risk="Waits up to 6.2 s; tools ride outside; servo bus through a slip ring untested.",
                pros=["Built and checked: 322 g, $167", "Works today in the desk-test replay"],
                cons=["Passes every tool on the way: 1.1-6.2 s", "Tools are outside, not stored", "Selector servo on the spinning rotor needs a slip ring"],
                scene=s)


def opt_revolver():
    """O1: offset revolver cylinder.  Chambers round an axis offset by E from the tool axis; turning the cylinder brings one
    chamber onto the centre line, a belt carriage pushes the cartridge straight forward into the barrel, and pulls it back."""
    e = E_RING; Rcyl = e + CH_BORE / 2 + 2.0
    z_sel = (REQ["collar_hang"] + 1.0, REQ["collar_hang"] + 1.0 + SERVOS["STS3032"]["box"][2])
    z_cyl = (z_sel[1] + 2.0, z_sel[1] + 2.0 + CH_L)
    z_sp = z_cyl[1] + 2.0
    tip = z_sp + FRONT["sp_L"] + FRONT["tool_out"]
    s = coupler_prims()
    s.append(box((-e, 0, sum(z_sel) / 2), (23.2, 12.1, z_sel[1] - z_sel[0]), COL["servo"], "selector servo STS3032 (turns the cylinder)"))
    s.append(cyl((-e, 0, z_cyl[0]), (-e, 0, z_cyl[1]), Rcyl, COL["housing"], "revolver cylinder, 5 chambers"))
    for k in range(N):
        a = math.radians(72 * k); x, y = -e + e * math.cos(a), e * math.sin(a)
        if k == 0:
            continue
        s.append(cyl((x, y, z_cyl[0] + 1), (x, y, z_cyl[0] + 1 + CART["L"] + TOOLS[k][2]), 5.25, COL["tool"], "stored tool"))
    rail = (z_cyl[0], z_cyl[0] + CH_L + 8.0 + 12.0)  # carriage travel: chamber rear to the barrel seat, + carriage length
    s.append(box((Rcyl - e + 6.0, 0, sum(rail) / 2), (6.0, 14.0, rail[1] - rail[0]), COL["feed"], "belt carriage rail (feed)"))
    s.append(box((Rcyl - e + 6.0, 0, REQ["collar_hang"] + 1.0 + 14.25), (12.1, 23.2, 28.5), COL["servo"], "feed servo STS3032 (2:1 to a 30T GT2 pulley)"))
    y_spin = -(FRONT["sp_OD"] / 2 + 14.0)                 # spin servo on the -y side, clear of the feed rail on +x
    s.append(box((0, y_spin, z_cyl[1] + 1.0 + 22.6), (35.0, 24.7, 45.2), COL["servo"], "spin servo ST3215-HS (1:1 gear)"))
    s += front_prims(z_sp)
    stroke = CH_L + 8.0                                   # cartridge rear from the back of its chamber to the spindle seat
    v = SERVOS["STS3032"]["rpm"] / 60.0 * 2.0 * 60.0     # 2:1 step-up to a 30T GT2 pulley (60 mm per pulley turn)
    t_feed = stroke_time(stroke, v)
    t_any = 2 * t_feed + turn_time(144.0, SERVOS["STS3032"]["rpm"])
    t_park = t_feed + turn_time(144.0, SERVOS["STS3032"]["rpm"])
    # masses (est.)
    # skeleton cylinder: five slotted 1 mm tubes between two 2 mm end plates (60 % solid), on a 6 mm hub axle
    m_cyl = RHO * (N * math.pi * (CH_BORE + 1.0) * 1.0 * CH_L * 0.85 + 2 * math.pi * Rcyl ** 2 * 2.0 * 0.6 + math.pi * 9 * CH_L)
    # open frame instead of a full shroud: rear plate, front bulkhead, three 4 mm tie rods (est.)
    m_frame = RHO * (2 * math.pi * (Rcyl + 3) ** 2 * 2.0 * 0.5 + 3 * math.pi * 4 * (CH_L + 6)) + shell_mass(FRONT["sp_OD"] / 2 + 6, 34)
    m_struct = m_cyl + m_frame + 14.0 + 10.0 + 8.0                                            # + rail, carriage, belt/pulleys/gears (est.)
    m_bought = SERVOS["ST3215-HS"]["mass"] + 2 * SERVOS["STS3032"]["mass"] + 3.0 + FRONT["brg_n"] * FRONT["brg_mass"] + 1.0 + 6.0 + 4.0  # reg, bearings, ball plunger, screws, pawls
    m = TOOL_MASS + m_struct + m_bought + FRONT["gear_mass"]
    com_z = 0.55 * z_cyl[1]
    com_x = (-e * (TOOL_MASS + 0.5 * m_struct + SERVOS["STS3032"]["mass"]) + (Rcyl - e + 6.0) * (SERVOS["STS3032"]["mass"] + 24.0)) / m
    com_y = y_spin * SERVOS["ST3215-HS"]["mass"] / m
    com_x = math.hypot(com_x, com_y)                       # off-axis distance of the centre of mass
    return dict(key="O1", name="Revolver cylinder (offset), straight push", short="Revolver cylinder",
                principle="Five chambers in a cylinder whose axis sits %.1f mm off the tool axis. Turning it brings one chamber onto "
                          "the centre line behind the barrel; a belt carriage pushes that cartridge straight forward into the barrel "
                          "and later pulls it back into the same chamber." % e,
                mass_g=m, tool_tip_z=tip, env_r=e + Rcyl + 2.5, env_len=tip, com_z=com_z, com_x=com_x,
                change_s=(t_any, t_any), park_to_tool_s=t_park, stroke=stroke, feed_speed=v,
                motors=["ST3215-HS spin (1:1 gear to the barrel)", "STS3032 selector (cylinder)", "STS3032 feed (2:1 to a GT2 belt)"],
                stored_inside=True, from_centre=True, kill=[],
                risk="Body is lopsided: it reaches %.0f mm on the cylinder side and %.0f mm on the other." % (e + Rcyl + 2.5, Rcyl - e + 12),
                pros=["Straight push: no bend for a long tool to jam in", "Any tool to any tool in the same time, never passes another",
                      "No electronics on a turning part: no slip ring", "Closest to a real revolver: proven mechanism"],
                cons=["No angling cam: the cylinder turns the tool onto the centre line instead", "Lopsided body (centre of mass %.0f mm off axis)" % abs(com_x),
                      "Three servos"],
                scene=s)


def opt_cone():
    """O2: converging cone (the literal 'cam angles it in').  Five straight inclined slots meet at the barrel's rear mouth.
    A cartridge pushed down its slot enters the barrel tilted and straightens as it goes in (front end held by the barrel,
    rear end by the slot: a trammel).  A flexible push rod (spring-steel wire in a PTFE liner) aimed by a turret pushes it."""
    best = None
    for beta in np.arange(8.0, 26.0, 1.0):
        b = math.radians(beta)
        gap = E_RING / math.sin(b)                               # distance from the apex where neighbouring slots separate
        r_rear = (gap + CH_L) * math.sin(b) + CH_BORE / 2 + 2.0
        length = (gap + CH_L) * math.cos(b)
        vol = math.pi * r_rear ** 2 * length / 3.0               # cone envelope
        if best is None or vol < best[3]:
            best = (beta, gap, r_rear, vol, length)
    beta, gap, r_rear, vol, length = best
    z_ap = REQ["collar_hang"] + 1.0 + 30.0 + length              # turret + push-rod drive behind the cone
    z_sp = z_ap
    tip = z_sp + FRONT["sp_L"] + FRONT["tool_out"]
    b = math.radians(beta)
    s = coupler_prims()
    s.append(box((0, 0, REQ["collar_hang"] + 16), (24, 24, 28), COL["servo"], "push-rod drive + turret servos"))
    s.append(cyl((0, 0, z_ap - length), (0, 0, z_ap), r_rear, COL["housing"], "cone magazine (envelope)"))
    for k in range(N):
        a = math.radians(72 * k); u = np.array([math.sin(b) * math.cos(a), math.sin(b) * math.sin(a), -math.cos(b)])
        p0 = np.array([0, 0, z_ap]) + gap * u; p1 = p0 + (CART["L"] + TOOLS[k][2]) * u
        s.append(cyl(p1, p0, 5.25, COL["tool"], "stored tool (slot at %.0f deg)" % beta))
    s.append(cyl((0, 0, z_ap - 6), (0, 0, z_ap + 2), 10.0, COL["cam"], "funnel mouth (the cam that straightens the tool)"))
    s.append(box((FRONT["sp_OD"] / 2 + 13.5, 0, z_sp + 23.6), (24.7, 35.0, 45.2), COL["servo"], "spin servo ST3215-HS"))
    s += front_prims(z_sp)
    stroke = gap + CH_L + 8.0
    v = 150.0                                                   # pinch-roller rod drive on a bus servo (est.)
    t_feed = stroke_time(stroke, v)
    t_any = 2 * t_feed + turn_time(144.0, SERVOS["STS3032"]["rpm"])
    m_struct = RHO * (math.pi * r_rear * math.hypot(r_rear, length) * 1.8 + N * math.pi * (CH_BORE / 2 + 1) * 2 * CH_L * 1.0) + 30.0
    m_bought = SERVOS["ST3215-HS"]["mass"] + 2 * SERVOS["STS3032"]["mass"] + 3.0 + FRONT["brg_n"] * FRONT["brg_mass"] + 12.0
    m = TOOL_MASS + m_struct + m_bought + FRONT["gear_mass"]
    return dict(key="O2", name="Converging cone + funnel cam (tool angles in)", short="Cone + funnel cam",
                principle="Five straight slots tilted %.0f deg meet at the barrel's rear mouth. The cartridge slides down its slot, "
                          "its nose enters the barrel and it straightens as it goes in (nose held by the barrel, tail by the slot). "
                          "A spring-steel push rod in a PTFE liner, aimed at the chosen slot by a turret, pushes and pulls it." % beta,
                mass_g=m, tool_tip_z=tip, env_r=r_rear, env_len=tip, com_z=0.5 * z_ap, com_x=0.0,
                change_s=(t_any, t_any), park_to_tool_s=t_feed + turn_time(144.0, SERVOS["STS3032"]["rpm"]), stroke=stroke, feed_speed=v,
                beta=beta, gap=gap, motors=["ST3215-HS spin", "STS3032 turret", "STS3032 push-rod drive"],
                stored_inside=True, from_centre=True,
                kill=["Size: smallest cone (tilt %.0f deg) is %.0f mm across and puts the tool tip %.0f mm out" % (beta, 2 * r_rear, tip),
                      "Mass %.0f g est. against the 350 g cap" % m],
                risk="The tilted cartridge can wedge in the funnel; a push rod in compression can buckle over %.0f mm." % stroke,
                pros=["Exactly the cam-angled entry described on Nov 21", "Symmetric round body", "Magazine never moves"],
                cons=["Neighbouring slots only separate %.0f mm from the apex, so the cone is long and wide" % gap,
                      "Longest stroke (%.0f mm), slowest change" % stroke, "Push rod and funnel are the least proven parts"],
                scene=s)


def opt_breech():
    """O3: fixed ring + turning breech.  Cartridges sit in a fixed ring of slots, spring-fed toward the centre.  The barrel
    continues back as a breech tube with a side window; the spin servo turns the window to face the chosen slot, the cartridge
    drops in, an on-axis rack pushes it forward.  Putting it back needs a ramp that drives it out sideways again."""
    R = PITCH + 2.0                                              # slots just outside the breech tube
    Rh = R + CH_BORE / 2 + 2.5
    z_rack = (REQ["collar_hang"] + 1.0, REQ["collar_hang"] + 1.0 + CH_L + 10.0)   # rack stored behind the breech
    z_br = (z_rack[1] + 2.0, z_rack[1] + 2.0 + CH_L)
    z_sp = z_br[1]
    tip = z_sp + FRONT["sp_L"] + FRONT["tool_out"]
    s = coupler_prims()
    s.append(box((0, 0, sum(z_rack) / 2), (8, 8, z_rack[1] - z_rack[0]), COL["feed"], "on-axis push rack (stored behind)"))
    s.append(box((14, 0, z_rack[0] + 14.5), (12.1, 23.2, 28.5), COL["servo"], "feed servo STS3032"))
    s.append(cyl((0, 0, z_br[0]), (0, 0, z_br[1]), Rh, COL["housing"], "fixed ring of slots"))
    s.append(cyl((0, 0, z_br[0]), (0, 0, z_br[1]), CH_BORE / 2 + 1.5, COL["spindle"], "turning breech with side window"))
    for k in range(N):
        a = math.radians(72 * k); x, y = R * math.cos(a), R * math.sin(a)
        s.append(cyl((x, y, z_br[0] + 1), (x, y, z_br[0] + 1 + CART["L"] + TOOLS[k][2]), 5.25, COL["tool"], "stored tool (spring-fed)"))
    s.append(box((FRONT["sp_OD"] / 2 + 13.5, 0, z_sp + 23.6), (24.7, 35.0, 45.2), COL["servo"], "spin servo ST3215-HS (also selects)"))
    s += front_prims(z_sp)
    stroke = CH_L + 8.0
    v = SERVOS["STS3032"]["rpm"] / 60.0 * 2.0 * 60.0
    t_feed = stroke_time(stroke, v)
    t_any = 2 * t_feed + turn_time(144.0, SERVOS["ST3215-HS"]["rpm"] * 6) + 0.3
    m_struct = RHO * math.pi * (Rh ** 2 - N * (CH_BORE / 2) ** 2) * CH_L * 0.5 + shell_mass(Rh + 2, CH_L) + 25.0
    m_bought = SERVOS["ST3215-HS"]["mass"] + SERVOS["STS3032"]["mass"] + 3.0 + FRONT["brg_n"] * FRONT["brg_mass"] + 8.0 + 5.0
    m = TOOL_MASS + m_struct + m_bought + FRONT["gear_mass"]
    return dict(key="O3", name="Fixed ring + turning breech (spin motor selects)", short="Turning breech",
                principle="Cartridges sit in a fixed ring, spring-fed toward the centre. The barrel runs back as a breech tube with "
                          "a side window; the spin servo turns the window to the chosen slot, the cartridge drops in and an on-axis "
                          "rack pushes it out. To put it back, a ramp must drive it sideways into its slot against the spring.",
                mass_g=m, tool_tip_z=tip, env_r=Rh + 2.5, env_len=tip, com_z=0.55 * z_br[1], com_x=0.0,
                change_s=(t_any, t_any), park_to_tool_s=t_feed + 0.3, stroke=stroke, feed_speed=v,
                motors=["ST3215-HS spin + select", "STS3032 feed (on-axis rack)"], stored_inside=True, from_centre=True,
                kill=["Length: the push rack has to sit behind the breech, so the tool tip is %.0f mm out" % tip,
                      "Return path: no simple way to drive a cartridge sideways out of a turning breech back into its own slot"],
                risk="Sideways spring feed and cam-back can jam or put a tool in the wrong slot.",
                pros=["One servo fewer: the spin servo also selects", "Round, symmetric body"],
                cons=["Longest hand (rack behind the breech)", "Putting the tool back into its own slot is the hard, unproven part",
                      "Springs push every stored tool against the breech while it spins"],
                scene=s)


def opt_tray():
    """O4: sliding tray ('linear revolver'): five chambers side by side in a flat tray that slides sideways to line one up."""
    W = N * PITCH + 4.0; travel = (N - 1) / 2 * PITCH
    z_tr = (REQ["collar_hang"] + 1.0 + 20.0, REQ["collar_hang"] + 21.0 + CH_L)
    z_sp = z_tr[1] + 2.0
    tip = z_sp + FRONT["sp_L"] + FRONT["tool_out"]
    s = coupler_prims()
    s.append(box((0, 0, z_tr[0] - 2.0), (W + 2 * travel + 6, CH_BORE + 6, 4.0), COL["housing"], "tray guide rail (full travel)"))
    for k in range(N):
        x = (k - 2) * PITCH
        s.append(cyl((x, 0, z_tr[0] + 1), (x, 0, z_tr[0] + 1 + CART["L"] + TOOLS[k][2]), 5.25, COL["tool"] if k != 2 else COL["tool_on"], "tray chamber"))
    s.append(box((W / 2 + travel + 16, 0, z_tr[0] + 14.0), (23.2, 12.1, 28.5), COL["servo"], "tray servo (rack and pinion)"))
    s += front_prims(z_sp)
    m = TOOL_MASS + 210.0
    return dict(key="O4", name="Sliding tray (linear revolver)", short="Sliding tray",
                principle="Five chambers side by side in a flat tray that slides sideways to line one up behind the barrel; "
                          "a carriage pushes it in as in O1.",
                mass_g=m, tool_tip_z=tip, env_r=(W / 2 + travel + 3), env_len=tip, com_z=0.5 * z_tr[1], com_x=0.0,
                change_s=(2 * stroke_time(CH_L + 8, 222.0) + 0.45, 2 * stroke_time(CH_L + 8, 222.0) + 0.45),
                park_to_tool_s=stroke_time(CH_L + 8, 222.0) + 0.45, motors=["ST3215-HS spin", "STS3032 tray", "STS3032 feed"],
                stored_inside=True, from_centre=True,
                kill=["Envelope: the tray needs %.0f mm of width to slide through (hand base is 41 mm)" % (W + 2 * travel)],
                risk="Sticks out %.0f mm each side; snags and hits the other arms." % (W / 2 + travel),
                pros=["Simplest motions (two straight slides)", "Flat: could lie along the arm"],
                cons=["%.0f mm wide sweep" % (W + 2 * travel), "Heavy guide for the full travel", "Ends hit neighbouring arms in a fold"],
                scene=s)


# ------------------------------------------------------------------ shared checks (front end, electrics)
def shared_checks():
    T = REQ["spin_torque_peak_Nm"] * 1000.0
    key_F = T / CART["key_r"] / CART["keys"]
    key_tau = key_F / (CART["key_w"] * CART["L"] * 0.8)
    hex_p = T / (6 * 2.5) / (CART["L"] * 0.8 * 1.0)           # 1/4 in hex shank in the printed hex bore, 6 flats, 2.5 mm lever, 1 mm band
    side_M = REQ["side_load_at_tip_N"] * (FRONT["tool_out"] + FRONT["sp_L"] / 2)
    brg_F = side_M / (FRONT["sp_L"] - 4.0)                       # two bearings at the spindle ends
    i12 = SERVOS["ST3215-HS"]["stall_A"] + 2 * SERVOS["STS3032"]["stall_A"] * 6.0 / 12.0 / 0.9
    return {
        "cartridge key shear at 1.96 N m (MPa)": round(key_tau, 2),
        "hex shank bearing in the printed cartridge at 1.96 N m (MPa)": round(hex_p, 2),
        "printed allowable used (MPa)": 30.0,
        "spindle bearing radial load from 5 N at the tool tip (N)": round(brg_F, 1),
        "6704 static rating C0r (N, est.)": 1800.0,
        "12 V pin current, all three servos stalled (A)": round(i12, 2),
        "12 V pin rating (A, assumed as on coupler rev B)": REQ["pin_current_A"],
        "push load path": "work push goes into spring pawls behind the cartridge's rear collar in the barrel (not into the feed)",
        "pull load path": "pull on the tool goes into the barrel's front lip (bore %.1f < cartridge %.1f)" % (FRONT["lip_bore"], CART["D"]),
    }


def sketch_overlaps(scene):
    """Coordinate-range (bounding box) overlap check: every bought part (box) against every other primitive."""
    def bb(p):
        if p["t"] == "box":
            c, h = np.array(p["c"]), np.array(p["s"]) / 2
            return c - h, c + h
        a, b = np.array(p["p0"]), np.array(p["p1"]); r = np.array([p["r"], p["r"], 0.0])
        return np.minimum(a, b) - r, np.maximum(a, b) + r
    hits = []
    for a in [p for p in scene if p["t"] == "box"]:
        for b in scene:
            if a is b:
                continue
            (l1, h1), (l2, h2) = bb(a), bb(b)
            if (np.minimum(h1, h2) - np.maximum(l1, l2) > 0).all():
                hits.append("%s | %s" % (a["label"], b["label"]))
    return hits


def main():
    opts = [opt_revolver(), opt_keep(), opt_cone(), opt_breech(), opt_tray()]
    rec = "O1"
    for o in opts:
        o["sketch_overlaps"] = sketch_overlaps(o["scene"])
        o["moment_Nm"] = o["mass_g"] / 1000.0 * G * o["com_z"] / 1000.0          # bending moment at the coupler, hand held flat
    out = dict(question="How is the chosen tool stored and brought onto the centre axis?", recommended=rec,
               requirements=REQ, shared_front_end=dict(chamber_length=CH_L, chamber_bore=CH_BORE, ring_radius=round(E_RING, 2),
                                                        tools=[t[0] for t in TOOLS], tools_and_cartridges_g=TOOL_MASS, **FRONT),
               shared_checks=shared_checks(),
               options=[{k: (round(v, 2) if isinstance(v, float) else v) for k, v in o.items() if k != "scene"} for o in opts])
    json.dump(out, open("toolhand_barrel_study.json", "w"), indent=1, default=lambda x: round(float(x), 2))
    json.dump([dict(key=o["key"], name=o["name"], prims=o["scene"]) for o in opts], open("toolhand_barrel_scene.json", "w"))
    figure(opts, rec)
    page = open("toolhand_barrel_page.template.html").read()
    page = page.replace("/*SCENE*/", open("toolhand_barrel_scene.json").read()).replace("/*STUDY*/", open("toolhand_barrel_study.json").read())
    open("toolhand_barrel_page.html", "w").write(page)
    for o in opts:
        print("%s %-42s %6.0f g  tip %4.0f  env r %4.0f  change %.2f s  M %.2f N m  kill %s" % (
            o["key"], o["name"][:42], o["mass_g"], o["tool_tip_z"], o["env_r"], o["change_s"][1], o["moment_Nm"], bool(o["kill"])))
    print(json.dumps(out["shared_checks"], indent=1))


def wrap(t, w=46):
    import textwrap
    return "\n".join(textwrap.wrap(t, w))


def figure(opts, rec):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    fig = plt.figure(figsize=(22, 14))
    gs = fig.add_gridspec(2, len(opts), height_ratios=(1.25, 1.0))
    fig.suptitle("Cephalo-2 multi-tool hand: how is the chosen tool stored and brought onto the centre axis?  "
                 "(3D sketches to one scale, mm; z = 0 at the coupler's hand base)", fontsize=14)
    lim = 120
    for i, o in enumerate(opts):
        ax = fig.add_subplot(gs[0, i], projection="3d")
        for p in o["scene"]:
            if p["t"] == "cyl":
                p0, p1 = np.array(p["p0"]), np.array(p["p1"]); d = p1 - p0; L = np.linalg.norm(d)
                if L < 1e-6: continue
                w = d / L; a = np.array([1, 0, 0]) if abs(w[0]) < 0.9 else np.array([0, 1, 0])
                u = np.cross(w, a); u /= np.linalg.norm(u); v = np.cross(w, u)
                th = np.linspace(0, 2 * np.pi, 20)
                ring = lambda c: [c + p["r"] * (math.cos(t) * u + math.sin(t) * v) for t in th]
                r0, r1 = ring(p0), ring(p1)
                quads = [[r0[j], r0[j + 1], r1[j + 1], r1[j]] for j in range(len(th) - 1)]
                ax.add_collection3d(Poly3DCollection(quads, facecolor=p["c"], edgecolor="none", alpha=0.45 if p["c"] == COL["housing"] else 0.92))
            else:
                c, s = np.array(p["c"]), np.array(p["s"]) / 2
                V = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * s + c
                F = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
                ax.add_collection3d(Poly3DCollection([[V[k] for k in f] for f in F], facecolor=p["col"], edgecolor="none", alpha=0.8))
        ax.set_xlim(-lim / 2 - 30, lim / 2 + 30); ax.set_ylim(-lim / 2 - 30, lim / 2 + 30); ax.set_zlim(-10, 250)
        ax.set_box_aspect((1, 1, 1.8)); ax.view_init(18, -60)
        ax.set_title(("%s  %s%s" % (o["key"], o["short"], "  (Recommended)" if o["key"] == rec else "")), fontsize=12,
                     color="#1d6b3a" if o["key"] == rec else ("#a02020" if o["kill"] else "black"))
        ax.tick_params(labelsize=7)
    rows = [("Mass (g, est.)", lambda o: "%.0f" % o["mass_g"]),
            ("Tool tip from base (mm)", lambda o: "%.0f" % o["tool_tip_z"]),
            ("Widest reach from axis (mm)", lambda o: "%.0f" % o["env_r"]),
            ("Tool change any->any (s)", lambda o: ("%.1f-%.1f" % o["change_s"]) if o["change_s"][0] != o["change_s"][1] else "%.1f" % o["change_s"][1]),
            ("Stored inside / from centre", lambda o: "%s / %s" % ("yes" if o["stored_inside"] else "no", "yes" if o["from_centre"] else "no")),
            ("Motors", lambda o: "%d" % len(o["motors"])),
            ("Moment at coupler, flat (N m)", lambda o: "%.2f" % o["moment_Nm"]),
            ("Kill check", lambda o: wrap("; ".join(o["kill"])) if o["kill"] else "passes"),
            ("Main risk", lambda o: wrap(o["risk"]))]
    ax = fig.add_subplot(gs[1, :]); ax.axis("off")
    cells = [[f(o) for o in opts] for _, f in rows]
    tb = ax.table(cellText=cells, rowLabels=[r[0] for r in rows], colLabels=["%s %s" % (o["key"], o["short"]) for o in opts],
                  loc="upper center", cellLoc="center")
    tb.auto_set_font_size(False); tb.set_fontsize(9); tb.scale(1, 2.0)
    for (r, c), cell in tb.get_celld().items():
        if r in (len(rows) - 1, len(rows)):
            cell.set_height(cell.get_height() * 2.4)
    for j, o in enumerate(opts):
        if o["key"] == rec:
            for i in range(len(rows) + 1):
                tb[(i, j)].set_facecolor("#dff2e3")
    plt.tight_layout(rect=(0, 0, 1, 0.95))
    plt.savefig("toolhand_barrel_study.png", dpi=110)


if __name__ == "__main__":
    main()
