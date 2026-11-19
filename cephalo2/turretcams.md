# Scabbard turret cameras: vision inside the blade-storage turret (2026-11-15)

Ask (Forest): add cameras for vision inside the existing "bucket" that stores the blades. That bucket is the **Scabbard turret** (`scabbard-dock.md`): the r 90 mm, 224 mm tall turret over the scabbard cage.
Code: `claude/code/turretcams.py` (**single source of truth**: bays, windows, checks, coverage), `claude/code/turretcams_fig.py`. It imports `scabbard.py`. Sheet: `cephalo1_turretcams.png`. STLs: `turretcams_stl/`, 5 parts, all single watertight bodies.
Status: **accepted (Forest, 2026-11-15): with the Scabbard turret fitted, these five cameras are the robot's head vision and the eyestalks are dropped.** CAD v1, fit-checked on the solids. On the plain rev 4 dome (no turret) the eyestalks stay.

## The finding that drives this
**The turret takes the eyestalks' place.** The four stalk sockets sit at r 80, z 112, at azimuths 0/90/180/270 (`eyestalks.md`). Those are the own-scabbard azimuths, inside the turret wall (r 90 from z 110 up). The pan servo pocket (z 83-112, r ~68-92) runs into the scabbard wall at r 79. `scabbard_checks.py` never checked the stalks. So with the turret fitted, the robot has no eyestalks and these cameras are its head vision. The eyestalks still work on the plain rev 4 dome. Moving them out onto the turret wall would be a separate redesign.

## Layout (mm, scabbard.py frame: origin at the centre of the plate underside)
| Item | Value |
|---|---|
| Side cameras | 4, one per guest-socket sector (azimuth 45 + 90k), above the guest sockets |
| Side lens | r 84.1, z 186, axis tilted **15° down**, cone ±55° (120° class lens, est.) |
| Zenith camera | 1, at the turret centre, lens front z 221, looking straight up |
| Camera | Seeed XIAO ESP32S3 Sense + OV5640 module for it (stack 21 × 17.8 × 15) with the heat sinks from its kit. Same node as the eyes and tip cameras. Wide lens to be picked (est.) |
| Side cradle (×4) | PETG, one print (21 × 24 × 35 box, 4.7 cm³). A level lug under the cage top ring (z 207-212) holds 2 × M2.5 heat-set inserts. A strap runs down to a tilted holder: back plate, side rails with front lips, floor with a USB-C exit. The stack snaps in from the front |
| Zenith bridge | PETG, one print across the top ring on the 45/225° diagonal, between the elevator fingers. A tray under the roof holds the zenith stack. At 45° and 225° the same M2.5 screws clamp bridge, ring and cradle lug together |
| Side window | Ø31 hole in the 1.6 mm shell, centre 5.5 mm below the lens. Clear PC disc 35 × 1, vertical, in a printed bezel whose back is cut to the wall's inner curve. Seal with silicone |
| Roof window | Ø18 hole, PC disc 22 × 1 in a bezel |
| Left free | The guest sector from the socket top (z 82) to the cradle bottom (z 177): room for the slim packs (scabbard open item 1). The inboard strip r 53-55 is also free for the elevator belt loops, which `scabbard.py` does not draw |

Why 15° down plus a zenith camera (`turretcams.py cover`, share of the sphere seen by the turret alone):
| Option | Stand | Flight | Stand, below −30° |
|---|---|---|---|
| level, 4 side | 72 % | 57 % | 36 % |
| level + zenith | 83 % | 71 % | 35 % |
| 15° down + zenith (**pick**) | **87 %** | **77 %** | **58 %** |
| 15° up + zenith | 77 % | 66 % | 13 % |

## Checks on the solids (`python3 turretcams.py check`, about 3 min)
| Against | Smallest gap, mm |
|---|---|
| Elevator ring + fingers, whole 194 mm stroke (every 10 mm + dock 86) | 2.3 (holder foot at u 170) |
| Stowed blades | 17.2 |
| Cage scabbards and guest sockets | 10.6 |
| Cage top ring | 0.2 under it (screws clamp); bridge sits on it |
| Turret shell | 2.8 |
| Lens to window disc | 0.5 side (tilted lens rim), 0.4 roof |
| View cone (lens rim, ±55°) to the edge of the shell hole | 0.7 side, 0.7 roof |
| Latch pins, fold ring and rods | 165, 142 |
| Bridge to side cradles | 3.2 (the ring sits between them) |
Docked or on the horse, the partner's blades or the horns reach z 70 inside the guest sockets. The sockets end at z 82, and the cameras sit above z 177.

## What they see (`python3 turretcams.py cover`)
Method as in `tipcams4.py`: 4000 directions. A ray from the lens must run 1.5 m without touching an arm capsule, a pod (hand3 on arms 2 and 6 as r 60 mm spheres, XL claw pods r 45, est.), the dome, the plate, the hub or the turret. Props count as see-through. Poses come from `poses4.json`.

| Pose | Turret | + tip cameras | Horizon ±30° | Above +30° | Below −30° |
|---|---|---|---|---|---|
| Stand | 87 % | 89 % | 97 % (99 %) | 95 % (99 %) | 58 % (58 %) |
| Flight (tulip) | 77 % | **94 %** | 77 % (92 %) | 92 % (92 %) | 63 % (**100 %**) |
For comparison, from `eyestalks.md` / `tipcams.md`: eyestalks alone 81.5 % in both poses; eyestalks + tip cameras 100 % in flight.
- **Stand: better than the stalks** (87 % vs 81.5 %). The 15° tilt sees further down than the stalks' −15° limit.
- **Flight: worse in the horizon band.** The tulip arms rise past the turret (hubs at z 353), and fixed cameras cannot lean round them the way a stalk pans. The tip cameras fill most of it: 94 % total vs 100 % with stalks. What stays unseen is a band just above the horizon behind each arm pair.
- Docking: the belly view still comes from the tip cameras (100 % below −30° in flight). The turret cannot see under the plate.
- Neighbouring side cameras overlap about 20° at the diagonals (axes 90° apart, ±55° cones): a 119 mm baseline stereo strip there, est.

## Mass and parts
+106 g (est.): 5 cameras 45, 4 cradles 24, bridge 13, bezels + discs 12, screws + wire 12. With the eyestalks gone under the turret (−357 g) the robot is **251 g lighter** than rev 4 + stalks + turret.
Buy: 5 × Seeed XIAO ESP32S3 Sense, 5 × OV5640 camera for XIAO ESP32S3 Sense (Seeed 114993115, with heat sinks; wide lens to source), 1 mm clear polycarbonate sheet (5 discs), 8 × M2.5 screws (4 × 6 mm through the ring, 4 × 10 mm through bridge and ring at 45/225°), 8 × M2.5 × 4 heat-set inserts (in the cradle lugs), silicone sealant.
Print: 4 cradles (flat on the back plate), 1 bridge (flat), 5 bezels, the turret shell with windows (`scab_mantle_turret_windows_PETG.stl`, replaces `scab_mantle_turret`).

## Open items
1. Elevator belt loops and their drive are not drawn in `scabbard.py`. The cameras leave r 53-55 and the zone below z 177 free, but the belt path must be confirmed.
2. Heat: 5 ESP32S3 + OV5640 boards (about 0.5-1 W each, est.) in a closed turret. The OV5640 needs its heat sink. Add 2-4 small vents in the roof between the windows, or run the cameras at low duty when idle.
3. Video: 5 turret + 8 tip cameras is too much Wi-Fi video. Stream only the cameras the mode needs, as in `tipcams.md`. The PETG turret is RF-transparent; route the u.FL antennas to the shell.
4. Pick the 120° class lens for the XIAO OV5640 connector. Measure its field of view and redo `cover` with the real cone.
5. Vibration: the cradles hang from the cage, which carries the rotor motor well. Add a TPU washer under each lug if images wobble with the big rotor spinning.
6. Decided: the turret cameras replace the eyestalks when the turret is fitted. Mounting stalks on the turret wall stays an option, not planned.
7. Port into `body4.py`'s mass table together with the turret.
