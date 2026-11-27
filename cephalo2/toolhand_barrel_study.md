# Multi-tool hand, revolver barrel: design study (2026-11-27)

Question: how is the chosen tool stored and brought onto the centre axis, so that it comes out through a centre barrel, is driven there, and goes back to its own slot?

Source: `toolhand_barrel_study.py` (numbers in `toolhand_barrel_study.json`, figure `toolhand_barrel_study.png`, 3D sketch primitives `toolhand_barrel_scene.json`). Sketches only: no CAD has been built yet. Numbers marked est. are estimates.

## What it has to do
- Mount on the coupler rev B.7 hand base (Ø41.2 flange, z = 0 at its bottom face, collar hangs 4.5 mm). 12 V on the pogo pins, 5.2 A per pin assumed.
- Store five tools inside the body, put one out through a centre barrel, spin it, and put it back in its own slot. Target 335 g, cap 350 g, about 1 s per change.
- Loads on the tool: 8 N push (screwing, drilling), 15 N pull (magnet, hook), 1.0 N·m sustained spin and 1.96 N·m peak (ST3215-HS stall), 5 N side load at the tip.
- Parts: off-the-shelf or printed in standard or carbon-fibre resin; no aluminium.

## Shared front end (every option)
- Each tool sits in a printed cartridge (Ø15 × 24, 1/4 in hex bore, two keys). The five tools: Wera 893/4/1 K bit holder with a Phillips bit; Wera 870/1 square drive with a socket; Loh Torng mini keyless chuck 0.3–3.2; magnet poker with hook and pry tip; a spare insert bit held straight in a magnetic cartridge. Tools and cartridges together: 107 g est.
- The barrel is a spindle 30 mm long on two 6704-2RS bearings, driven 1:1 by the ST3215-HS through an m2 z12 gear pair (as on the selector-cam hand). Its front lip (bore 12.6) takes pull loads; spring pawls behind the cartridge's rear collar take push loads, so the feed carries no work load.
- Checks at 1.96 N·m: key shear 2.4 MPa, hex shank bearing 6.8 MPa against 30 MPa allowed for printed resin. Spindle bearing load from 5 N at the tip: 12 N against a 1,800 N static rating (est.). All three servos stalled draw 3.7 A on the 12 V pin (rated 5.2 A).

## Options
| | Mass (g, est.) | Tool tip from base (mm) | Widest reach (mm) | Change any→any (s) | Motors | Kill check |
|---|---|---|---|---|---|---|
| **O1 Revolver cylinder (recommended)** | 325 | 199 | 41 | 1.2 (0.75 from parked) | 3 | passes |
| O0 Keep the selector cam (as built) | 322 | 194 | 33 | 1.1–6.2 | 2 | passes, but tools ride outside |
| O2 Cone + funnel cam (tool angles in) | 349 | 299 | 36 | 3.0 | 3 | killed: 71 mm across, tip 299 mm out |
| O3 Fixed ring + turning breech | 376 | 260 | 32 | 1.3 | 2 | killed: length, and no clean way back into its slot |
| O4 Sliding tray | 317 | 188 | 82 | 1.4 | 3 | killed: needs 157 mm of width to slide |

- **O1 Revolver cylinder.** Five chambers in a skeleton cylinder whose axis sits 14.5 mm off the tool axis. An STS3032 turns the cylinder (up to 144°) to bring one chamber onto the centre line behind the barrel. A second STS3032 drives a GT2 belt carriage (2:1 up to a 30T pulley, 222 mm/s) whose finger reaches through the chamber's side slot and pushes the cartridge straight forward 90 mm into the barrel, then pulls it back into the same chamber. No electronics turn, so no slip ring. Centre of mass sits 7 mm off axis; the body reaches 41 mm on the cylinder side.
- **O0 Keep the selector cam.** Built and checked (322 g, $167) but it passes every tool on the way (up to 6.2 s), the tools ride outside, and its selector servo sits on the spinning rotor behind a slip ring.
- **O2 Cone + funnel cam.** The literal "a cam angles it into the barrel": five tilted slots meet at the barrel's rear mouth and the tool straightens as it goes in. Neighbouring slots only separate 104 mm from the apex at the best tilt (8°), so the magazine is long; a long push rod can buckle and a tilted cartridge can wedge in the funnel.
- **O3 Fixed ring + turning breech.** Saves a servo (the spin servo also selects) but the push rack has to sit behind the breech, and there is no simple way to drive a cartridge sideways back into its own slot.
- **O4 Sliding tray.** Simplest motions, but sweeps 78 mm each side of the axis.

## Recommendation
O1. It is the only new option that passes every kill check, it keeps all electronics stationary, and every tool takes the same straight path. It replaces the angling cam from the Nov 21 description with the cylinder turning the tool onto the centre line, which is what a revolver does.

## Decision
Waiting on Forest (asked 2026-11-27). Rotatable 3D page: https://claude.ai/artifact/Hoxti5EzgHeK2ccrwHDQME
