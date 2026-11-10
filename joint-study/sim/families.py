"""Vertebra-joint families for the Cephalo-1 tentacle study.

Each family is a joint mechanism. A "joint type" is a family plus one mechanism variant
(material, bearing, range). An "arm configuration" is a joint type placed in a 585 mm arm with
an arrangement (axis pattern), an actuation architecture, a joint count and a landing-compliance option.

Screening values (strength index, friction, elastic stiffness, part counts) are engineering
estimates for FDM parts at D = 40 mm and scale with diameter; they are stated so they can be argued
with, and every number feeds the ranking only through the formulas in metrics.py.

kin:  pin      single pivot per bending DOF
      rolling  rolling-contact / gear-meshed: two virtual pivots, each takes half the angle
      ball     concentric axes at one point (2 or 3 rotations)
      flex     distributed compliant bend: modelled as 2 sub-pivots
      disc     disc on elastic backbone: 2 sub-pivots + small axial
      kresling bend + coupled contraction/twist
strength_k: M_cap = 20 MPa * strength_k * D^3 (PETG allowable with margin)
k_el: elastic return stiffness at D = 40 mm, N*m/rad (scaled by (D/40mm)^3)
"""
FAMILIES = [
 # ---------------- rigid pivots ----------------
 dict(id="pin_hinge", name="Pin hinge (clevis)", kin="pin", bend=1, twist=0, ext=0,
      ranges=[60, 90, 120], pitch_D=1.0, act=["local", "tendon", "hybrid", "section", "pushpull"],
      variants=[dict(v="printed_pin_PLA", fr=0.10, sk=0.0025, hw={"pin_printed": 1}, tol=0.5, mins=6),
                dict(v="M3_bolt_PETG", fr=0.07, sk=0.0035, hw={"m3_bolt": 1, "nut": 1}, tol=0.3, mins=7),
                dict(v="MR63_bearings_PETG", fr=0.02, sk=0.0035, hw={"bearing": 2, "m3_bolt": 1, "nut": 1}, tol=0.4, mins=10)],
      k_el=0.0, printed=2, supports=0, risk=0.2, landing=0, src=["pin_hinge_same_plane", "pin_hinge_alternating"]),
 dict(id="servo_shell", name="Servo-shell vertebra (servo case is the link)", kin="pin", bend=1, twist=0, ext=0,
      ranges=[90, 120], pitch_D=1.25, act=["local"],
      variants=[dict(v="horn+idler_PETG", fr=0.03, sk=0.004, hw={"bearing": 1, "m3_bolt": 4}, tol=0.3, mins=12)],
      k_el=0.0, printed=2, supports=0, risk=0.15, landing=0, src=["servo_per_joint"], prior="servo shells"),
 dict(id="servo_chain_bracket", name="Servo chain with U-brackets (current Cephalo-1 arm)", kin="pin", bend=1, twist=0, ext=0,
      ranges=[90, 120], pitch_D=1.45, act=["local"],
      variants=[dict(v="printed_U_bracket_PETG", fr=0.04, sk=0.004, hw={"bearing": 1, "m3_bolt": 6}, tol=0.3, mins=12)],
      k_el=0.0, printed=2, supports=0, risk=0.2, landing=0, src=["servo_per_joint"], prior="current servo chain"),
 dict(id="universal", name="Universal (Cardan) joint", kin="ball", bend=2, twist=0, ext=0,
      ranges=[45, 60], pitch_D=1.1, act=["local", "tendon", "hybrid", "section"],
      variants=[dict(v="printed_cross_PETG", fr=0.08, sk=0.0025, hw={"pin_printed": 2}, tol=0.6, mins=12),
                dict(v="steel_pins_PETG", fr=0.06, sk=0.0035, hw={"dowel": 2}, tol=0.5, mins=12),
                dict(v="bearings_PETG", fr=0.02, sk=0.0035, hw={"bearing": 4, "dowel": 2}, tol=0.6, mins=18)],
      k_el=0.0, printed=3, supports=0, risk=0.25, landing=0, src=["universal_cardan"]),
 dict(id="diff_bevel", name="Bevel differential 2-DOF (2 servos share pitch+roll)", kin="ball", bend=1, twist=1, ext=0,
      ranges=[90], pitch_D=1.6, act=["local"],
      variants=[dict(v="printed_bevels_PETG", fr=0.07, sk=0.003, hw={"bearing": 4, "m3_bolt": 6}, tol=0.8, mins=30),
                dict(v="printed_bevels_PA", fr=0.06, sk=0.004, hw={"bearing": 4, "m3_bolt": 6}, tol=0.8, mins=30)],
      k_el=0.0, printed=5, supports=1, risk=0.35, landing=0, diff=True, src=["diff_bevel"]),
 dict(id="cycloidal_module", name="Printed cycloidal-drive module per node", kin="pin", bend=1, twist=0, ext=0,
      ranges=[90, 150], pitch_D=1.5, act=["local"],
      variants=[dict(v="PETG_cycloid_N20motor", fr=0.05, sk=0.005, hw={"bearing": 3, "dowel": 6, "m3_bolt": 4}, tol=0.8, mins=45)],
      k_el=0.0, printed=6, supports=0, risk=0.3, landing=0, servo_override="CYCLO", src=["servo_per_joint"], prior="cycloidal modules"),
 dict(id="canted_hinge", name="Canted-axis hinge (bend-twist coupled, helical coiling)", kin="pin", bend=1, twist=0, ext=0,
      ranges=[90, 120], pitch_D=1.19, act=["local", "tendon"], cant=45,   # pitch_D from arm_cad.py fit check
      shell_struct=(0.054, 0.0666, 0.0836),   # windowed PETG shell per vertebra, from arm_cad.py
      variants=[dict(v="M3_bolt_PETG", fr=0.07, sk=0.003, hw={"m3_bolt": 1, "nut": 1}, tol=0.4, mins=8),
                dict(v="M3_bolt_PETG_cant30", pitch_D=1.65, cant=30, fr=0.07, sk=0.003, hw={"m3_bolt": 1, "nut": 1}, tol=0.4, mins=8),
                dict(v="M3_bolt_PETG_cant20", pitch_D=2.39, cant=20, fr=0.07, sk=0.003, hw={"m3_bolt": 1, "nut": 1}, tol=0.4, mins=8),
                dict(v="M3_bolt_PETG_cant15", pitch_D=3.15, cant=15, fr=0.07, sk=0.003, hw={"m3_bolt": 1, "nut": 1}, tol=0.4, mins=8),
                dict(v="bearings_PETG", fr=0.02, sk=0.003, hw={"bearing": 2, "m3_bolt": 1}, tol=0.5, mins=11)],
      k_el=0.0, printed=2, supports=1, risk=0.25, landing=0, src=["pin_hinge_alternating"]),
 dict(id="roll_bearing", name="Axial roll joint (twist module inserted every k nodes)", kin="twist", bend=0, twist=1, ext=0,
      ranges=[150], pitch_D=0.6, act=["local", "tendon"], addon=True,
      variants=[dict(v="608_bearing", fr=0.02, sk=0.004, hw={"bearing": 1}, tol=0.4, mins=10)],
      k_el=0.0, printed=2, supports=0, risk=0.2, landing=0, src=["servo_per_joint"]),
 # ---------------- rolling contact ----------------
 dict(id="rolling_1d", name="Rolling-contact joint, crossed straps (CORE-style)", kin="rolling", bend=1, twist=0, ext=0,
      ranges=[60, 90], pitch_D=0.75, act=["tendon", "section", "hybrid", "pushpull", "local"],
      variants=[dict(v="Dyneema_straps_PETG", fr=0.015, sk=0.005, hw={"strap": 2}, tol=0.3, mins=10),
                dict(v="TPU_flexure_straps_PETG", fr=0.02, sk=0.004, hw={}, tol=0.4, mins=6),
                dict(v="pinned_gear_guide_PETG", fr=0.03, sk=0.005, hw={"pin_printed": 2}, tol=0.5, mins=9)],
      k_el=0.02, printed=2, supports=0, risk=0.15, landing=0, src=["rolling_contact_1d"], prior="base-box rolling vertebrae"),
 dict(id="rolling_2d", name="Two-axis rolling contact (stacked orthogonal rolling faces)", kin="rolling", bend=2, twist=0, ext=0,
      ranges=[45, 60], pitch_D=1.0, act=["tendon", "section", "hybrid", "pushpull"],
      variants=[dict(v="Dyneema_straps_PETG", fr=0.02, sk=0.004, hw={"strap": 4}, tol=0.4, mins=16),
                dict(v="spherical_rolling_PETG", fr=0.03, sk=0.003, hw={"strap": 3}, tol=0.6, mins=14)],
      k_el=0.02, printed=3, supports=0, risk=0.2, landing=0, src=["rolling_contact_2d"]),
 dict(id="gear_rolling", name="Gear-meshed rolling (geared double hinge, 2:1)", kin="rolling", bend=1, twist=0, ext=0,
      ranges=[90, 120], pitch_D=0.95, act=["local", "tendon", "hybrid", "section"],
      variants=[dict(v="printed_spur_PETG", fr=0.05, sk=0.004, hw={"pin_printed": 2, "link_plate": 2}, tol=0.7, mins=14),
                dict(v="printed_spur_PA", fr=0.04, sk=0.005, hw={"m3_bolt": 2, "link_plate": 2}, tol=0.7, mins=14)],
      k_el=0.0, printed=4, supports=0, risk=0.25, landing=0, src=["gear_meshed_rolling"], prior="geared double hinges"),
 dict(id="rolling_monolithic", name="One-piece printed rolling (print-in-place TPU hinge)", kin="rolling", bend=1, twist=0, ext=0,
      ranges=[45, 60], pitch_D=0.75, act=["tendon", "section"],
      variants=[dict(v="TPU95A", fr=0.04, sk=0.0015, hw={}, tol=0.2, mins=2),
                dict(v="PETG_print_in_place", fr=0.08, sk=0.002, hw={}, tol=0.6, mins=2)],
      k_el=0.25, printed=1, supports=0, risk=0.35, landing=1, src=["tpu_monolithic_tentacle"], prior="one-piece printed rolling"),
 # ---------------- balls ----------------
 dict(id="ball_printed", name="Printed ball-and-socket (snap fit)", kin="ball", bend=2, twist=1, ext=0,
      ranges=[30, 40], pitch_D=0.9, act=["tendon", "section", "hybrid"], twist_uncontrolled=True,
      variants=[dict(v="PETG_snap", fr=0.12, sk=0.002, hw={}, tol=0.8, mins=4),
                dict(v="PETG_split_socket_bolted", fr=0.10, sk=0.003, hw={"m3_bolt": 2}, tol=0.6, mins=8)],
      k_el=0.0, printed=2, supports=1, risk=0.3, landing=0, src=["ball_socket_printed"]),
 dict(id="ball_magnetic", name="Magnetic ball joint (breakaway)", kin="ball", bend=2, twist=1, ext=0,
      ranges=[35, 45], pitch_D=0.9, act=["tendon", "section"], twist_uncontrolled=True,
      variants=[dict(v="steel_ball_N52_ring", fr=0.05, sk=0.0008, hw={"steel_ball": 1, "magnet": 1}, tol=0.3, mins=5)],
      k_el=0.0, printed=2, supports=0, risk=0.2, landing=1, src=["ball_socket_magnetic"]),
 dict(id="ball_pushrod", name="Ball joint driven by 2 pushrods (2-RSS)", kin="ball", bend=2, twist=0, ext=0,
      ranges=[30, 35], pitch_D=1.6, act=["local"],
      variants=[dict(v="M2_ball_links", fr=0.06, sk=0.003, hw={"ball_link": 4, "rod": 2}, tol=0.6, mins=25)],
      k_el=0.0, printed=3, supports=1, risk=0.3, landing=0, servos_per_node=2, src=["pantograph_parallel_continuum"]),
 dict(id="omniwrist_vertebra", name="Omni-Wrist-style 2-DOF linkage used as a vertebra", kin="ball", bend=2, twist=0, ext=0,
      ranges=[60, 90], pitch_D=2.0, act=["local", "tendon"],
      variants=[dict(v="printed_links_bearings", fr=0.05, sk=0.002, hw={"bearing": 8, "dowel": 8}, tol=0.9, mins=60)],
      k_el=0.0, printed=8, supports=1, risk=0.4, landing=0, src=["omni_wrist"]),
 # ---------------- compliant ----------------
 dict(id="cross_axis_flexure", name="Cross-axis flexural pivot", kin="flex", bend=1, twist=0, ext=0,
      ranges=[30, 45], pitch_D=0.8, act=["tendon", "section", "pushpull"],
      variants=[dict(v="PETG_leaves", fr=0.0, sk=0.0010, hw={}, tol=0.3, mins=2, k_el_mult=1.0),
                dict(v="TPU95A_leaves", fr=0.01, sk=0.0005, hw={}, tol=0.2, mins=2, k_el_mult=0.25, rng_mult=1.4),
                dict(v="PP_leaves", fr=0.0, sk=0.0010, hw={}, tol=0.4, mins=2, k_el_mult=0.6, rng_mult=1.3)],
      k_el=0.6, printed=1, supports=0, risk=0.45, landing=1, src=["cross_axis_flexure"], prior="cross-axis flexures"),
 dict(id="notch_flexure", name="Notch / living-hinge flexure", kin="flex", bend=1, twist=0, ext=0,
      ranges=[20, 30], pitch_D=0.6, act=["tendon", "section", "pushpull"],
      variants=[dict(v="PETG_circular_notch", fr=0.0, sk=0.0012, hw={}, tol=0.3, mins=1, k_el_mult=1.0),
                dict(v="TPU95A_notch", fr=0.01, sk=0.0005, hw={}, tol=0.2, mins=1, k_el_mult=0.25, rng_mult=1.6),
                dict(v="PP_living_hinge", fr=0.0, sk=0.0010, hw={}, tol=0.4, mins=1, k_el_mult=0.5, rng_mult=1.5)],
      k_el=0.8, printed=1, supports=0, risk=0.45, landing=1, src=["compliant_notch_flexure"]),
 dict(id="cross_notch_2d", name="Orthogonal double-notch flexure segment", kin="flex", bend=2, twist=0, ext=0,
      ranges=[20, 30], pitch_D=0.8, act=["tendon", "section"],
      variants=[dict(v="PETG", fr=0.0, sk=0.0010, hw={}, tol=0.3, mins=1, k_el_mult=1.0),
                dict(v="TPU95A", fr=0.01, sk=0.0004, hw={}, tol=0.2, mins=1, k_el_mult=0.25, rng_mult=1.5)],
      k_el=1.0, printed=1, supports=0, risk=0.5, landing=1, src=["cross_notch_2d"]),
 dict(id="disc_spine_tpu", name="Vertebral discs on a printed TPU backbone", kin="disc", bend=2, twist=0, ext=1,
      ranges=[20, 30], pitch_D=0.5, act=["tendon", "section"],
      variants=[dict(v="TPU95A_rod_PETG_discs", fr=0.02, sk=0.0012, hw={}, tol=0.2, mins=3, k_el_mult=1.0)],
      k_el=0.5, printed=1, supports=0, risk=0.3, landing=1, src=["vertebral_disc_spine"]),
 dict(id="tpu_spine", name="Continuous TPU spine (one-piece)", kin="flex", bend=2, twist=0, ext=0,
      ranges=[15, 25], pitch_D=0.5, act=["section", "tendon"],
      variants=[dict(v="TPU95A", fr=0.03, sk=0.0004, hw={}, tol=0.1, mins=1, k_el_mult=0.6),
                dict(v="TPU85A", fr=0.04, sk=0.0003, hw={}, tol=0.1, mins=1, k_el_mult=0.3, rng_mult=1.3)],
      k_el=0.6, printed=1, supports=0, risk=0.3, landing=1, src=["tpu_monolithic_tentacle"], prior="TPU spine"),
 dict(id="spirob_wedge", name="Log-spiral wedge segments (SpiRob-style)", kin="pin", bend=1, twist=0, ext=0,
      ranges=[25, 35], pitch_D=0.7, act=["tendon", "section"], log_taper=True,
      variants=[dict(v="PETG_wedges_TPU_web", fr=0.03, sk=0.0015, hw={}, tol=0.3, mins=2, k_el_mult=0.5)],
      k_el=0.3, printed=1, supports=0, risk=0.3, landing=1, src=["wedge_tentacle_spirob"]),
 dict(id="helical_spring", name="Printed helical-spring segment", kin="flex", bend=2, twist=0, ext=1,
      ranges=[15, 25], pitch_D=0.7, act=["tendon", "section"],
      variants=[dict(v="PETG_coil", fr=0.01, sk=0.0006, hw={}, tol=0.3, mins=2, k_el_mult=1.0),
                dict(v="TPU_coil", fr=0.02, sk=0.0003, hw={}, tol=0.2, mins=2, k_el_mult=0.3, rng_mult=1.3)],
      k_el=0.5, printed=1, supports=1, risk=0.4, landing=1, src=["helical_spring_joint"]),
 dict(id="kresling", name="Kresling origami module (bend + contract + twist)", kin="kresling", bend=2, twist=0, ext=1,
      ranges=[12, 18], pitch_D=0.8, act=["tendon", "section"],
      variants=[dict(v="TPU_hinged_PETG_panels", fr=0.05, sk=0.0004, hw={}, tol=0.4, mins=8, k_el_mult=1.0)],
      k_el=0.4, printed=1, supports=0, risk=0.5, landing=1, src=["kresling_origami"]),
 dict(id="yoshimura", name="Yoshimura / waterbomb origami segment", kin="flex", bend=2, twist=0, ext=1,
      ranges=[15, 20], pitch_D=0.8, act=["tendon", "section"],
      variants=[dict(v="TPU_hinged_PETG_panels", fr=0.05, sk=0.0004, hw={}, tol=0.4, mins=10, k_el_mult=1.0)],
      k_el=0.4, printed=1, supports=0, risk=0.5, landing=1, src=["yoshimura_or_waterbomb_origami"]),
 dict(id="tensegrity", name="Tensegrity segment (struts + cables)", kin="ball", bend=2, twist=1, ext=0,
      ranges=[20, 30], pitch_D=1.0, act=["tendon", "section"], twist_uncontrolled=True,
      variants=[dict(v="PETG_struts_Dyneema", fr=0.02, sk=0.0008, hw={"cable": 6}, tol=0.5, mins=25, k_el_mult=1.0)],
      k_el=0.3, printed=3, supports=0, risk=0.4, landing=1, src=["tensegrity_segment"]),
 dict(id="chain_links", name="Interlocking chain / drag-chain links", kin="pin", bend=1, twist=0, ext=0,
      ranges=[15, 25], pitch_D=0.7, act=["tendon", "section"],
      variants=[dict(v="PETG_snap_links", fr=0.10, sk=0.002, hw={}, tol=0.6, mins=2)],
      k_el=0.0, printed=1, supports=0, risk=0.3, landing=0, src=["cable_tie_or_chain"]),
 dict(id="notched_tube", name="Asymmetric notched tube (printed)", kin="flex", bend=1, twist=0, ext=0,
      ranges=[10, 15], pitch_D=0.4, act=["tendon", "section"],
      variants=[dict(v="PETG", fr=0.0, sk=0.0010, hw={}, tol=0.3, mins=1, k_el_mult=1.0)],
      k_el=0.8, printed=1, supports=0, risk=0.5, landing=1, src=["concentric_tube_or_rod"]),
 dict(id="twisted_string_hinge", name="Pin hinge driven by a local twisted-string actuator", kin="pin", bend=1, twist=0, ext=0,
      ranges=[60, 90], pitch_D=1.4, act=["local"],
      variants=[dict(v="N20_motor_Dyneema", fr=0.05, sk=0.0035, hw={"m3_bolt": 1, "dc_motor": 1, "encoder": 1}, tol=0.4, mins=35)],
      k_el=0.0, printed=3, supports=0, risk=0.4, landing=1, servo_override="TSA", src=["twisted_string"]),
 dict(id="scissor_ext", name="Scissor / Sarrus extension module (adds elongation)", kin="pin", bend=0, twist=0, ext=1,
      ranges=[0], pitch_D=0.8, act=["local", "tendon"], addon=True,
      variants=[dict(v="printed_sarrus", fr=0.05, sk=0.002, hw={"pin_printed": 6}, tol=0.5, mins=20)],
      k_el=0.0, printed=6, supports=0, risk=0.3, landing=0, src=["scissor_or_sarrus"]),
 dict(id="bellows_pneumatic", name="Pneumatic bellows segment (excluded by motors-only rule; reference)", kin="flex", bend=2, twist=0, ext=1,
      ranges=[30, 45], pitch_D=0.9, act=["section"],
      variants=[dict(v="TPU_printed_bellows", fr=0.03, sk=0.0004, hw={"valve": 2, "tube": 2}, tol=0.6, mins=20, k_el_mult=1.0)],
      k_el=0.2, printed=1, supports=0, risk=0.6, landing=1, src=["bellows_pneumatic"], excluded="Forest chose motors, not pneumatics"),
]

# Arrangements of bending axes along the arm
ARRANGEMENTS_1DOF = ["planar", "alt90", "helix60", "alt90_roll4"]
ARRANGEMENTS_2DOF = ["omni", "omni_roll4"]

# Actuation architectures
ACTUATIONS = {
    "local":    "servo/motor inside each vertebra (bus daisy-chain)",
    "tendon":   "per-joint antagonistic tendon pair from the arm's base box (Forest's base-box direction)",
    "pushpull": "per-joint push-pull Bowden cable from the base box (one cable per DOF)",
    "section":  "3 sections x 2 bend DOF, tendons from the base (continuum style, coupled joints)",
    "hybrid":   "local high-torque servos in the 2 root nodes, base-box tendons for the rest",
}

# Landing / overtravel compliance options (Forest: 'overextend slightly and snap back')
COMPLIANCE = {
    "rigid":       dict(stroke_deg=0,  k_rel=1.0,  parts=0, note="no added compliance"),
    "tendon_spring": dict(stroke_deg=8, k_rel=0.15, parts=2, note="compression spring in series with each tendon at the base box"),
    "tpu_coupler": dict(stroke_deg=6,  k_rel=0.25, parts=1, note="TPU coupler between servo horn / drive and link"),
    "overtravel_detent": dict(stroke_deg=10, k_rel=0.10, parts=3, note="spring-preloaded overtravel that snaps back (preload holds normal loads)"),
    "magnetic_breakaway": dict(stroke_deg=25, k_rel=0.05, parts=2, note="magnet-held seat that releases on impact and re-seats"),
}
