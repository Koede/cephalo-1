# Scabbard rotor drives: the mechanism behind the belly-rotor video (2026-11-15)

**Status: accepted (Forest, 2026-11-16, "perfect, keep this").** These drives, the turret at r 92, the 185 mm stow lift and the 99 mm dock stroke are part of the Cephalo-2 baseline.

Ask (Forest): the blades slide out, swing out 90° and spin on a shared rotating ring at the base of the turret; the video of it must show real mechanisms, not a render trick.
The slide, swing and spin were already in the scabbard CAD (`scabbard.py`: elevator ring and fingers, push rods on the trunnion cams, trunnions on the hub ring, blades). What was missing were the drives that move them: `scabbard-dock.md` carried the lead screws, belts and servos in the mass table and BOM but never drew or placed them. This doc places them, checks them against every other part over the whole sequence, and sizes them.
Code: `claude/code/scabbard_drive.py` (single source of truth for the drives; patches `scabbard.py` on import), `claude/code/rotor_video.py` (the video). Video: `cephalo2_belly_rotor_deploy.mp4` (1920×1080, 32.6 s, kinematic). Check output: `scabbard_drive_check.json`.

## Three design changes, carried in `scabbard_drive.py`
| Change | Was | Now | Why |
|---|---|---|---|
| Turret radius | 90 | **92** mm | the two STS3215 stand in the guest sectors between the scabbards; at r 90 the body (35 mm radially) missed the wall by 0.6 mm. Turret cameras move out 2 mm with it (`turretcams` lens 86.1, disc 88.6); their checks were re-run inside the drive check. |
| Stow lift | 194 | **185** mm | the elevator ring (top z 201 stowed) must pass under the elevator shaft (z 206.5) and its pulleys. Stowed blade tip 9 mm inside the barrel instead of 2. |
| Dock stroke | 108 | **99** mm | follows from the stow lift (dock position U_DOCK 86 unchanged, so the dock itself is unchanged). |

## Fold drive: rod ring on three lead screws
- **Three T8×2 lead screws** at r 95 (az 70, 190, 310), z 6-92, in thrust bearings through the cage base flange. Brass T8 nuts in printed bosses on the fold ring; the ring is r 84-96 (was 97) to clear the idler posts by 0.5 mm.
- **One GT2 belt (6 mm) at z 76** runs round 20T pulleys on the three screws, 16T idlers on posts at r 99.5 (az 10, 130, 250, 340) and a 20T pulley on a short vertical **jackshaft at az 322, r 86**. Every side of the belt stays ≥ 82 mm from the axis, outside the scabbards (r 79) and the push rods (r 84.5, and the rods never rise above z 70.5); the idlers clear the dome inner wall by 2 mm at z 79.5.
- The jackshaft (Ø4, z 72.5-89.5) carries a second 20T pulley at z 86.5, belted to the **fold servo STS3215** hanging its output shaft down in the guest sector at az 315 (body z 90-125, r 57.7-82.3; 2.9 mm from the scabbards and the wall). A printed bracket down the guest socket's outer wall holds the jackshaft bearing between its two pulleys.
- Loads: deploying in the air the rods push only the blade springs and weight, 118 N in all → 0.13 × rated on the servo. Standing on the four blade legs the rods carry 964 N: the T8×2 screws (lead angle 5.2°, below the friction angle) hold that with the servo off; the servo would need 1.04 × rated to *move* under it, so the legs are set in the air before touchdown, as in the take-off plan. A 2.5 g landing (2.4 kN on the rods) goes into the locked screws, 804 N each.
- Belt: the servo's rated torque on a 20T pulley is 154 N tight side; a 6 mm GT2 is good to about 100 N continuous. **Use 9 mm GT2 for the fold loop** (same layout, pulleys 9 mm wide), or accept 0.65 × rated.

## Elevator drive: blade ring on two belt loops
- **Two vertical GT2 loops at r 50**, azimuths 66 and 204: both lie on one line parallel to the 45/225 diagonal, 18 mm to its 135° side, which runs past the zenith camera tray (walls at ±12.5) with 1 mm. 12T pulleys top and bottom; the bottom pulleys sit in pockets in the plate (z −1.3 to 6.3; hub top plate at −3 is 1.7 mm clear, the seated ring at z 8 is 1.7 mm clear).
- The ring is **slotted for both runs** at each loop and a printed clamp (belt channel 1.6 × 6.2) bridges the drive-run slot. The fingers' webs in the scabbard slots keep the ring square.
- **One Ø4 shaft at z 206.5** carries the two top pulleys: a bearing block under the cage top ring at the az-66 end, two hangers from the zenith bridge, and at the az-204 end a **1:1 miter pair** (r 4.5) to a vertical **jackshaft at az 209, r 62** down to a 20T pulley at z 126-132, belted to the **elevator servo STS3215** (output up) in the guest sector at az 225 (body z 90-125; 3.2 mm clear). Clearances: shaft to stowed ring 3.5 mm, pulleys to roof 1.6 mm, shaft to cage top ring 3.5 mm, jackshaft to the az-225 camera holder 4.9 mm.
- Loads: lifting the four blades and the ring is 2.5 N; the 60 N dock cinch is 0.23 N m at the 12T pulleys, 4.3 × under rated. Full stroke 7.4 s at no-load speed, about 15 s under load (the video shows 5.5 s; a real deployment is slower). Belt: 30 N per loop on a 12T (5 teeth in mesh) is at the limit of a 6 mm GT2: **9 mm belt and pulleys here too**, or cinch at 40 N.

## Fit check (`python3 scabbard_drive.py`, ~4 min)
Boolean intersections of every drive part against the shell, cage, plate, motor, latch pins, the five cameras, and against the moving parts (hub, trunnions, blades, rods + fold ring, elevator) at 15 states along the sequence: slide (u 185, 150, 100, 50, 0), unhook (ring to 14, hub +6.5°), swing (droop 90, 75, 60, 40, 20, 1), spin (hub 6.5, 30, 60°). Also the moving parts against the cage, plate and cameras, and the elevator against the blades.
**Result: no collisions.** The 11 pairs that still report overlap are parts that belong inside each other: shafts in pulleys, screws in pulleys, posts in idlers, pulleys on servo bosses, belts on pulleys (the belt is drawn as a buffer), hangers embedded in the bridge, the two miter cones at their mesh.
Not modelled: the two thrust bearings per screw, the servo mounting screws, belt tensioners (the loops are drawn tight), wiring, and the latch release ring of the dock (unchanged from `scabbard-dock.md`).

## Sequence as rendered (`rotor_video.py`)
| t (s) | What moves | Drive |
|---|---|---|
| 0-1.5 | stowed | - |
| 1.5-7 | ring slides the blades down 185 mm through the trunnions | elevator servo, belts |
| 7-8 | heads seat; hub turns 6.5°, fingers lift 14 mm and unhook | rotor motor, elevator |
| 8-11.5 | rods rise; the trunnion springs swing the blades out 89° | fold servo, screws |
| 11.5-12.3 | rods retract clear of the rotor | fold servo |
| 12.3-19.8 | hub ring spins all four blades (4320 rpm, drawn at 1/24) | MN605-S |
| 19.8-32.6 | spin down, park on the index, rods down, swing back, rehook, lift, stowed | reverse |
The robot hovers on its eight hand3 rotors; the hover is posed, not simulated (that is the Cephalo-2 mission sim, still to do).

## Bought parts added
2 × STS3215 (one each; already in the scabbard BOM) · 3 × T8×2 lead screw 90 mm + brass nuts + 3 pairs of 8 mm thrust bearings · 3 × 20T + 4 × 16T GT2 pulleys (fold), 4 × 12T + 2 × 20T (elevator) + 2 × 20T (jackshafts) · GT2 belts: fold loop (length from CAD, ~640 mm), elevator 2 × ~410 mm closed loops, 2 short jackshaft loops · 1:1 miter gear pair, 4 mm bore · Ø4 steel shaft 150 mm, 2 × Ø4 shafts 60 mm · 6 × MR84 bearings · M2.5/M3 screws.
Print: fold-ring with nut bosses (PETG), 2 jackshaft brackets, bearing block, 2 hangers, 2 ring clamps, plate pockets (in the plate), cage flange bores (in the cage).
