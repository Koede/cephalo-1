# Cephalo-2 baseline (2026-11-15)

Forest named the current design **Cephalo-2** and chose hand3 on all eight arms. Cephalo-1 (rev 4 and everything simulated and rendered so far) stays as it is: its videos and sims show the robot they were made for.
3D model (rotatable, standing / flying, X-ray turret): artifact "Cephalo-2 3D model". Belly-rotor deployment video: `cephalo2_belly_rotor_deploy.mp4` (`rotor_video.py`, kinematic). Code: `claude/code/cephalo2_model.py` (GLBs from the CAD), `claude/code/cephalo2_viewer.py` (page).

## What it is
| System | Cephalo-2 | Source |
|---|---|---|
| Arms | rev 4 tentacles: 8 × 8 servo joints (STS3250 / STS3215), clutches, winch line, tip camera in every tip bay | `cephalo4.py`, `tipcams.md` |
| Hands | **hand3 final (hub C2) on all eight arms**: four fingers that are also a 9 in rotor | `hand3-final.md`, `hand3e/` |
| Hand coupling | **bayonet connector on every tip**: 32 mm quarter-turn, 3 lugs, ball detent, O-ring, 5 spring pins (12 V, GND, servo data, servo return, hand ID) | `hand-connector.md`, `handmount.py` |
| Belly rotor | Scabbard rotor in place of the EDF: 4 blades, 22 in, stowed upright in the turret, legs at 60°, dock lock, centaur bars. Drives (lead screws, belts, two STS3215) laid out, fit-checked and **accepted** (2026-11-16): turret r 92, stow lift 185, dock stroke 99 | `scabbard-dock.md`, `scabbard.py`, `scabbard-drive.md`, `scabbard_drive.py` |
| Vision | 5 turret cameras (4 side, 1 up) in place of the eyestalks; 8 tip cameras | `turretcams.md`, `tipcams.md` |
| Power | 2 × 6S 5000 mAh | as rev 4 + 2 packs |

Integration change: the bayonet was designed to hang under the tip bay. On Cephalo-2 it sits on the arm-tip axis (end mount), so every hand sits 9.9 mm further out (hand base 9.7 + 0.2 gap). Fit on the tip bay still to check.

## Mass (est.)
| Item | kg |
|---|---|
| Scabbard robot (rev 4 + XL pods + 2 packs + rotor and turret, EDF out), from the current `cephalo4` mass table | 13.20 |
| − eyestalks | −0.36 |
| + turret cameras | +0.11 |
| + 8 bayonet sets (socket + hand base printed, pins, boards, O-ring, est.) | +0.21 |
| + 8 × (hand3 0.396 − XL pod 0.190) | +1.65 |
| **Cephalo-2** | **14.81** (W 145 N) |
(`scabbard-dock.md` quotes 12.81 kg for its robot; the current `cephalo4` table gives 13.20 for the same build. The difference is not traced yet.)

## Flight budget (static, before any sim)
hand3 full-throttle thrust low / mid / high 21.5 / 23.1 / 24.8 N, K 10.22 W/N^1.5 (`hand3_thrust_cad.json`); belly rotor 54.8 N max (est.), K 3.45 W/N^1.5 (from `scabbard.py`: disc 0.194 m², FM 0.50, η 0.85).
| Belly rotor | Each hand | Share of hand ceiling (low / mid / high) | Hover power | Rotor torque | Hand yaw authority (mid) |
|---|---|---|---|---|---|
| off | 18.2 N | 85 / 79 / 73 % | 6.3 kW | 0 | 1.08 N m |
| 27 N | 14.7 N | 69 / 64 / 60 % | 5.1 kW | 1.32 N m | 1.83 N m |
| 47 N (85 %) | 12.3 N | 57 / 53 / 50 % | 4.6 kW | 2.24 N m | 2.35 N m |

What this says:
1. **It flies on the hands alone, but thinly** (73-85 % of ceiling just to hover). The belly rotor is what gives it margin.
2. **The rev 4 take-off cannot be used.** It unfurls legs in pairs on six props: six hand3 would need 24.2 N each, above the 23.1 N ceiling. And the belly rotor cannot spin while the legs stand inside its disc. The take-off that fits: stand on the rotor's blade legs (60°), lift all eight arms into the flight pose with their rotors spinning, lift off on eight hands, then swing the blades to flight and spin up. Landing is the reverse.
3. **Endurance is short.** hand3 rotors cost twice the power of the XL claw-props per thrust (K 10.2 against 5.1). Hover on hands + rotor is about 4.6 kW: 2 × 111 Wh gives about 2.3 min to 80 % use. Per hand 36 A at hover (ESC limit 55 A).
4. **Yaw**: at high rotor thrust its torque (2.2 N m) uses nearly all the hands' differential-torque authority. Tilting the tentacles tangentially (tentacle steering) has to add yaw. Not modelled yet.

## Next
1. Port the belly rotor and the eight hands into the full-robot sim (`body4h.py`): rotor as the central lift, hub pad as the belly contact, hand3 masses and thrust on all arms, the blade-leg take-off and landing above.
2. Re-run the day mission as Cephalo-2 and render it.
3. Trace the 12.81 / 13.20 kg difference; check the bayonet's end-mount on the tip bay.
