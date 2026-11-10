---
name: mass-budget
description: Mass breakdown and budget for a cephalo-1 arm and the robot - printed parts, servos, hardware, geared root additions, added mass versus the 46.4 g budget, robot mass and thrust-to-weight. Use for weight questions or when a part changes.
---
# mass-budget

Read `arm-pin-hinge/out_1m_13/fit.json` (`m_vert`, `m_lid`, `m_tip`, `m_sock`, `arm_out_g`, `arm_all_g`) and `root_gear.json` (`added_g_per_arm`, `budget_g`, `robot_kg`, `thrust_to_weight`, gear/bearing masses). Present: per-part mass x count x 8 arms, servos (STS3250 74.5 g x5, STS3215 55 g x8 per arm), hardware 5 g/joint, geared-root added mass vs budget, robot kg, T/W vs 1.15. Model: PETG 1.27 g/cm^3 x 0.95 fill. Machine-check with `python3 tools/check_gates.py`. For what-ifs (swap a servo, add a pack) recompute `robot = 15.084 + 8*delta/1000` and `T/W = 1.15 * 15.084 / robot` as `root_gear.py` does, and say which assumptions are `[A]`.
