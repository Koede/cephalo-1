# Cephalo-1 arm joints: printable, sleek, fluid (proposal, 2026-11-08)

Problem: the rev 3 arm carries 9 Feetech servos in printed U-brackets. Nodes are up to 65 mm across, arm + pod ~747 g, the servos step in position mode, and nothing is backdrivable. That makes the arm bulky, heavy at the tip, stiff in a crash and jerky to watch.

## Direction set by Forest (2026-11-08)
Fine control of every vertebra. Each tentacle gets its own **base control box** at the root that drives the whole series of vertebrae. This replaces the 3-section coupling proposed below; the 3-section version stays as the low-motor-count fallback.

## Writhing comparison video (2026-11-08)
`cephalo1_tentacle_joint_grid.mp4`: a 3×3 grid, one 475 mm tentacle per joint type, the same 18 s curvature command to all of them (wave, turning wave, tip coil, corkscrew, whip). Each variant maps the curvature onto its own joints, couples joints that share tendons, and clamps to its own limits (estimates). Kinematic only. Code: `claude/code/tentacles.py`, `claude/code/writhe_grid.py` (uses render.py and geom.py).

| Variant | DOF | Shape error avg / max (mm) | Frames with a joint at its limit | Self-hit frames |
|---|---|---|---|---|
| 1 Servo chain (rev 3) | 9 | 11.5 / 36 | 66 % | 24 |
| 2 Servo shells | 9 | 12.0 / 31 | 76 % | 0 |
| 3 Cycloidal BLDC modules | 6 | 18.7 / 33 | 67 % | 0 |
| 4 Geared double hinges | 8 | 11.7 / 22 | 72 % | 0 |
| 5 Cross-axis flexures (±25°) | 17 | 17.9 / 52 | 91 % | 0 |
| 6 TPU continuum, 2 sections | 4 | 40.4 / 76 | 45 % | 0 |
| 7 Rolling vertebrae, 3 sections | 8 | 18.0 / 36 | 57 % | 0 |
| **8 Rolling, every vertebra driven** | **19** | **6.1 / 16** | 69 % | 0 |
| 9 Monolithic CRCJ (±30°) | 8 | 21.7 / 53 | 78 % | 0 |

Shape error = mean distance between the commanded and actual backbone at 24 points. The at-limit count includes the root joints, which clip often. Self-hit uses the simple collision radii in tentacles.py.

## Options surveyed
| # | Approach | Printable | Sleek | Fluid | Verdict |
|---|---|---|---|---|---|
| 1 | Keep servos, replace U-brackets with tapered shells around the servo case, idler bearing opposite the horn | yes | medium (~45 mm nodes) | low (position steps) | Cheap upgrade |
| 2 | Printed cycloidal / quasi-direct-drive BLDC joint modules | yes | medium | high (backdrivable) | Best for the root joints only |
| 3 | Rolling-contact joints (two cylindrical cams + crossed straps), tendon-driven from the base | yes, very | high | high | **Core of the design** |
| 4 | Monolithic compliant rolling-contact joint (straps printed in) | yes | high | high | Lower range (±30°) |
| 5 | Monolithic TPU / PA12 continuum backbone | yes | highest | high | Too few DOF and too soft |
| 6 | Gear-coupled double hinge | yes | high | medium | Good shape tracking, blocky |
| 7 | Cross-axis flexure pivots | yes | high | medium | ±25° only, fatigue |
| 8 | ABENICS cross-spherical gear root | yes | medium | medium | Alternative root |
| 9 | Silicone-cast joints | partly | high | high | Too compliant for walking |

## Architecture: "base box drives every vertebra"
1. **Root (2 DOF):** pitch + yaw knuckle driven from the base box (printed cycloidal BLDC or STS3250 through a bevel differential).
2. **Arm body:** 18 rolling-contact vertebrae, taper Ø36 → Ø16 mm, h 28 → 20 mm, cam radius 1.3 r, rolling planes alternate pitch / yaw, ±45° per joint (est.).
3. **Per-vertebra drive:** one antagonistic tendon pair per joint, all 17 pairs anchored in the base box; 17 tendon actuators + 2 root = 19 per arm. Distal tendons pass through the proximal joints, so the base box firmware compensates path-length coupling. Tendon size falls toward the tip (needed tension drops with lever arm), e.g. 1.5 mm Dyneema at the root and 0.5 mm at the tip (to size).
4. **Vertebrae:** PA12/PA-CF or PETG; crossed Dyneema or 0.2 mm spring-steel straps; PTFE-lined tendon channels through the contact line, so antagonistic pairs stay taut.
5. **Skin:** printed TPU bellows or scale rings.
6. **Compliance:** series springs in the tendon lines inside the base box for soft landings and torque control.

## Open items
- **Count:** 19 actuators × 8 arms = 152 actuators, versus 72 today. Each base box has to fit 19 small linear or capstan drives. This needs a layout and mass check, and 34 tendons have to pass through the Ø36 root vertebra.
- Tendon friction accumulates for the distal joints, which pass through up to 16 proximal joints.
- Walking loads at the stand pose versus tendon tension and vertebra crush strength (run in MuJoCo).
- Fluid motion also needs firmware: stream setpoints at ≥100 Hz and use the input shaper.
