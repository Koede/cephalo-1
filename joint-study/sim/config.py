"""Cephalo-1 joint study: robot-level parameters.

Values marked [F] come from Forest's Cephalo-1 work (whole-robot MuJoCo model and notes).
Values marked [A] are assumptions of this study; change them here and re-run.
Units: SI (m, kg, N, N*m, rad) unless a name says otherwise.
"""
import math

G = 9.81

# ---- robot ----
N_ARMS = 8                      # [F]
MASS_TOTAL_REF = 8.63           # kg [F] current robot with 9-servo arms
ARM_MASS_REF = 0.647            # kg [F] current servo arm
MASS_NON_ARM = MASS_TOTAL_REF - N_ARMS * ARM_MASS_REF   # body, EDF, batteries, claws/props (~3.45 kg)
EDF_THRUST = 42.2               # N [F]
TIP_THRUST_MAX = 10.8           # N [F] claw-prop max (range 7.8-10.8)
TIP_THRUST_HOVER = 5.5          # N [F] hover per fan
TIP_THRUST_HOP = 7.4            # N [F]
ARM_LENGTH = 1.000              # m [F] Forest, Nov 9: "probably a meter" (was 0.585 assumed)
D_ROOT = 0.040                  # m [F] taper from proposal (root vertebra 36-40 mm)
D_TIP = 0.018                   # m [F]
HUB_RADIUS = 0.110              # m [A] arm roots on this circle
TIP_PAYLOAD_MASS = 0.090        # kg [A] claw + fan motor + prop + ESC at the tip, excluding the tip joint

# ---- walking (ripple gait) ----
FEET_ON_GROUND = 7              # [F]
FOOT_LOAD = MASS_TOTAL_REF * G / FEET_ON_GROUND   # N, re-computed per design with its own mass
STRIDE = 0.090                  # m [F]
STANCE_PULL_IN = (0.030, 0.038) # m [F]
BODY_HEIGHT = 0.200             # m [A] hub plane above ground while walking
FOOT_RADIAL = 0.300             # m [A] foot distance outward from the arm root, at mid-stance
CLAW_VERTICAL_TOL = math.radians(20)  # [F claw vertical] tolerance used in IK

# ---- flight ----
FLIGHT_TILT_NOM = math.radians(15)   # [A] tip-thrust tilt off the arm axis for balancing
FLIGHT_TILT_MAX = math.radians(30)   # [A] stress case

# ---- landing ----
DROP_HEIGHT = 0.15              # m [A]
LANDING_FEET = 4                # [A] uneven landing: 4 feet share the impact

# ---- tendons (base-box drives) ----
TENDON_MU = 0.10                # Dyneema in PTFE [A, literature 0.05-0.15]
TENDON_EA = 30e3                # N, 1.5 mm Dyneema braid effective EA [A]
TENDON_SWL = 150.0              # N safe working load per tendon [A] (break ~1 kN, knots/terminations derate)
TENDON_CHANNEL_OD = 3.0e-3      # m PTFE liner OD for 1.5 mm line [F: Dyneema 1.5 mm in PTFE]
PULLEY_RADIUS = 0.010           # m base-box drive pulley [A]

# ---- safety ----
SF_REQUIRED = 2.0               # Forest's robot-design rule: >= 2x on load calcs

# Main arm design chosen by Forest on 2026-11-08 (pin hinge, 60 deg helix, one STS3215 per joint).
# rank.py always runs it through MuJoCo so the page can show it wherever it lands.
MAIN_DESIGN = dict(family="pin_hinge", variant="M3_bolt_PETG", arrangement="helix60", actuation="local",
                   local_servo="STS3215_12V", compliance="rigid")
# servo fit chosen Nov 9: STS3250 in joints 0-3 (graded band to 35 % of the arm), STS3215 beyond. Same case, so the
# printed arm is the one above; eval_main.py scores this fit, and the sweep row above stands in for its geometry.
MAIN_SERVOS = dict(actuation="graded", servo="STS3215_12V", root_servo="STS3250_12V", split="long_mid")
MAIN_LABEL = "Pin hinge, 60° helix | 1 m, 13 joints | STS3250 in joints 0-4 (2:1 gear on joint 0), STS3215 beyond | 16 N claw fans"
# Root fix (Nov 9): joint 0 drives through a 2:1 spur stage (9.8 N.m at the joint, 2.2 rad/s), and the arm is never
# commanded faster than 80 % of each servo's rated speed. From fix_root.py, results/root_fix.csv.
ROOT_FIX = dict(gear={0: 2.0}, stage="m1 spur, 20T pinion on the STS3250 horn, 40T sector gear (teeth over +-65 deg, stops at +-55 deg) on the joint-0 pin, 30 mm centres, 10 mm face, both 7075-T6 hard-anodized; 61803 in the gear's inboard recess under the mesh, 695 on the pinion, 8 mm 7075 idler pin (6800, 685 and a printed stub until the Nov 9 root bearing check)",
                added_kg_per_arm=0.045, added_kg_per_arm_cad=0.0462,   # sims ran at 0.045; root_gear.py measures 46.2 g after the bearing check (45.3 before)
                socket=dict(script="arm-pin-hinge/root_gear.py", socket_g=30.1, end_cap_g=1.9, bearing_cap_g=2.1, gear_g=15.0, pinion_g=6.4, idler_pin_g=1.3,
                            bearings=dict(gear="61803", pinion="695"), bearing_check="joint-study/sim/bearing_loads.py",
                            joint0_travel_deg=55, servo_bulge_mm=6.8, mantle_cutout_mm=(-35.9, 25.0, -21.1, 21.1)), raise_rule="command speed <= 0.8 x rated servo speed (cosine ease)",
                robot_kg=15.08, thrust_to_weight=1.150, lift_rms_mm=5.9, wave_rms_mm={"0.25": 22.2, "0.5": 62.5}, curl_step_final_mm=85.9, sag_mm=2.2)
# Nov 9: Forest picked the 1 m arm with 13 joints. It flies only with the bigger claw fans, so the main design is
# scored in the more_thrust scenario (JOINT_SCENARIO=more_thrust: TIP_THRUST_MAX 16 N, TIP_PAYLOAD_MASS 130 g).
MAIN_JOINTS = 13
MAIN_SCENARIO = "more_thrust"
# Claw fan spec for the main design (design only, nothing bought). Sized from the T-Motor 2807 1250KV bench table
# (6S, 7.5x4.6x3 prop): 16 N at about 76 % throttle, 21.4 N at full; hover per fan 13.2 N at about 270 W with the root gear (12.8 N, 260 W before).
CLAW_FAN = dict(thrust_max_N=16.0, hover_N=13.2, prop_in=(7.0, 7.5), motor="2807, 1250-1300 KV, 6S", motor_g=52.6,
                tip_budget_g=130, esc_A=45, hover_W=270, peak_W=551)
