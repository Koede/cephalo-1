# Hippos-1: the horse body of Centauri-1 (renamed from Equo-1, 2026-11-16)

**Name (Forest, 2026-11-16): Equo-1 is now Hippos-1** (Greek hippos, horse, in line with Cephalo and Chilo). Centauri-1 is still the whole centaur (Cephalo-2 docked on Hippos-1).

Every "Equo-1" in older docs means Hippos-1. File names and switches keep the old spelling so nothing breaks: `centaur_equo_v2*.py`, `equo_solo.html`, `EQUO_S`, `EQUO_SOLO`, `EQUO_KNEE`, `EQUO_ABDMODE`, `EQUO_YF`, `EQUO_ABDF`.

Full history and numbers: `claude/centaur-design-study.md` (sections 4d to 4u). Previews (renamed): Centauri-1 https://claude.ai/artifact/AXfmVfRruZk4Hk83MwTXKn, Hippos-1 solo https://claude.ai/artifact/BWvpn7xrcgpAHn7f5R2LhS.

## Connector naming (Forest, 2026-11-16)
- **SFF connector** = the project's standard small connector: the 32 mm push-and-twist hand connector of `hand-connector.md` (3 lugs, 60 deg twist, cone seats, printed latch, O-ring, 5 spring pins: DATA, ID, AUX, GND, 12 V). Anything small and swappable (hands, the tail) uses it.
- **Bayonet** = reserved for big structural connections (the docks).

## Where Hippos-1 stands (end of 4u)
- 0.75 scale: hips 368 mm, top of octopus 771 mm docked. Body 604 mm long, 294 mm wide, 476 mm over the pauldrons; leg track 150 mm front and rear.
- Legs: three-segment pantograph horse leg, RS06 on hips, knees and front abduction (torque tubes behind the sheath block), RS02 rear abduction, 12S bus. Knee RS06 in "pauldron" (scapula) position, HTD 595-5M 25 mm belt.
- Rear swing unit on bell-crank shocks (2 x goBILDA 190 mm), bounce 6.2 Hz; lockout pin on the left swing arm. Detachable tail on the SFF connector: one Cephalo-2 arm + hand3 (below).
- Mass: Hippos-1 18.5 kg solo, Centauri-1 about 30.8 kg before the tail below (+0.3 kg, est. 31.1 kg, front share about 59.6 %, not yet re-run in the preview).
- Neck receiver (crown 236, throat 192, sheath block 150 square), grab posts at R 141, sensor hood for solo running.
- Collision check: docked 504 parts, solo 247 parts, 0 real clashes. Mid-run dock in the fast trot, arm-assisted capture x3.88.

## Full render (2026-11-16)
**v2, CAD meshes (`claude/code/centaur_render_scene2.py` -> `scene2.npz`, ~0.98 M triangles; six views hero, rear, side, hands, head, tail; 1632 x 1020, 64 samples, about 15 min on 2 cores).** Every part that has CAD is drawn from its solid:
- Cephalo-2 as `cephalo2_model.py` builds it: turret shell with the 5 camera windows, plate, scabbard cage, rotor motor, latch pins, camera bays (boards, OV5640 modules, heat sinks, cradles, zenith bridge, bezels, PC discs), portholes; belly rotor in "bars" mode (blades hanging, 108 mm down into the receiver) with elevator and fold rods; the 8 rev 4 arms (servos in cradles, clutch cams + rockers, idler bosses, forks, tip bays); the connector (arm socket + hand base) on every tip; **hand3 (hub C2, the four-finger fan) on all eight arms** (Forest, 2026-11-16: "all octopus arms the 4 finger version of the fans"; this is the Cephalo-2 baseline, `cephalo2_model.py` now defaults to `C2_HANDS=all`): front pair grasp-ready, the six raised as fans (fingers straight, rotor parked).
- Hippos-1 tail = one Cephalo-2 arm + hand3 on the SFF tail mount (`hippos/tailarm/tailarm.py`, `tailmount.py`); pose from `tailarm_checks.json` (`TAIL_POSE`, default carried high). `NO_TAIL=1` writes `scene2_notail.npz` for the tail checks.
- Hippos-1 body and legs: **no CAD exists yet**, so these stay the preview geometry (chassis v4 shell and trim, girdle, legs_v2, pauldron covers, actuators, belts, hooves).
- The Cephalo-2 modules were rebuilt in a workspace folder `ceph2/` as geometry-only subsets of the project files (geom, cephalo4, viz4 arm_mesh, scabbard, turretcams, xlpod4, xlclaw, xlclaw_cad, snapdigit, clawblade, mesh3d, hand3_design, hand3_finger, hand3_cad, hand3_mesh; handmount = `hippos/sff_core.py`). Functions copied verbatim, no geometry changed; not saved again (the project copies are the source).

**Finding: the T4 "hands meet" pose was solved with a stand-in hand.** `arm_scale_study` puts the hand point 60 mm past the spigot (LEAD). hand3 + connector reaches 159 mm (heel), grip centre about 130 mm. With the old pose the two hand3s overlap by about 16 mm. Re-solved with the same IK, chest target x / z and clearance terms, at the grip centre (LEAD 129.9 mm) with the targets 76 mm either side of the centre line: hands 11.9 mm apart, arms 15.8 mm or more from the horse shell, neighbouring arms 48.7 mm or more apart with the XL claw-props drawn (FCL on the meshes). Arms unchanged, only the pose. **The preview, its rules and its collision check still use the old T4 pose and the stand-in hand: open item 6.**

**hand3 on all eight arms: servo loads and the poses it forced (2026-11-16).** Static joint torque with the cephalo4 masses, hand3 426 g with its connector at 69 mm past the spigot (XL pod 182 g):
- Old poses with hand3: front pair (T4) j0 **139 %**, j2 125 % of rated; raised six (Q_GUARD -35/-45/0/0) j0 **102 %**. (With XL props the same poses were 104 % / 71 %.)
- **Front pair, new T4:** same IK target (grip centre 130 mm past the spigot, 0.20 m ahead of the chest), plus a penalty on torque above 80 % of rated and a mirror-plane term (hand mesh and arm + 10 mm for the clutch cams stay 5 mm off y = 0). Targets 80 mm either side of the centre line: q (-59, -3.6, 59.4, -94.6, 84.5, -8.6, 5.0, 100.0), worst joint **80 %** of rated, hands **10.6 mm** apart, hands reach forward side by side. j0 and j7 sit on their limits.
- **Raised six, new fan pose (-35, -85, 90, -50):** scanned with the rotor axis kept as in Q_GUARD (sum of pitches -80), swept 9 in discs >= 15 mm apart, arm >= 10 mm off dome and turret: worst **91 %** of rated (j0 of arms 1 and 6), swept discs **17.7 mm** apart, discs >= 17.5 mm from the neighbouring parked hands.
- Fast-trot collar heave (6.3 mm p-p at 5.61 Hz) adds 0.4 g: peaks about **127 % of rated (42 % of stall)**, below every clutch (0.9 / 1.2 x stall). Rated is a continuous figure, so a long trot with the fans raised needs a thermal check in the dynamics sim.
- All arms >= 15.8 mm from the horse shell; neighbouring arms >= 41 mm apart. Loads in `arm_loads_hand3.json`.
- **Mass:** Cephalo-2 with hand3 on all eight is 14.81 kg (est., `cephalo2-baseline.md`); the centaur study and the leg margins use 12.81 kg. Open item 7.

v1 (`claude/code/centaur_render_scene.py` -> `scene.npz`): preview geometry, octopus stand-in, capsule arms, tail CAD. Renderer for both: `claude/code/centaur_render_blend.py` (Blender 5 bpy; `python3 centaur_render_blend.py scene2.npz prefix 64 hero rear side hands head tail`, `RES_PCT` env; material families by name, colour from the CAD).

## Detachable tail on the SFF connector
### Mount design study (2026-11-16, `claude/code/hippos_tail_sff_study.py`)
Ask (Forest): make the tail a connector of its own, push in and twist to secure, using the hand connector. Picked (Forest, "go with that"): socket on the rump, the whole tail comes off; twist kept at the SFF's 60 deg (Forest said 45) so hands and tail stay interchangeable; a skirt-in-sleeve against rocking; current limited to the one 12 V pin; a second SFF socket at the tail tip.

### Mount draft CAD v1 (2026-11-16)
Code: `claude/code/hippos/tailmount.py` (**single source of truth**), `sff_core.py` (the SFF geometry, a copy of `handmount.py` with the socket cuts split out, geometry unchanged: masses check 9.3 / 10.6 g), `tailmount_checks.py` (-> `tailmount_checks.json`, 15 s), `tailmount_section.py`, `tailmount_viewer.py` + template. Mount-only model: https://claude.ai/artifact/JznPgv8RXhs5JXzK5EsDcq. Status: draft CAD, fit-checked on the true solids; not printed, no FEA.

| Part | What it is | Mass (PETG, solid) | Print |
|---|---|---|---|
| `rump_socket` | standard SFF socket ring + round mount flange Ø64 (4 x M3 at r 29.5 into the rump pad) + sleeve bore Ø46.6 / OD 52.6 reaching 29.5 mm out of the mouth, 45 deg lead-in, latch-tab window | 40.3 g | flange face on the bed, no supports |
| standard `hand_base` | unchanged SFF base | 10.6 g | as the hand kit |
| `tail_adapter` | bolts under the base (4 x M3 x 10, Ø6.2 counterbores from below); 45 deg conical floor; skirt Ø41.4 / Ø46.4; latch-tab notch open to the skirt top; bottom flange Ø56 with 4 x M3 heat-set inserts at r 25, az +-20 and 180 +-20 (matches the root cradle, clear of its servo walls) | 33.1 g | servo flange on the bed, no supports |

Checks: locked no overlap; push-in guided by the sleeve for 35 mm, no contact; twist -60 -> 0 deg no contact, latch flexes 1.45 mm; latch pulled 1.6 mm clears skirt and sleeve; rocking +-0.19 deg (plain SFF about 4x more); release tab reachable through 10.8 mm windows; no supports. (Fixes the checks forced: open notch over the release tab; bosses and seat follow the 45 deg cone.)

### Tail v2: one Cephalo-2 arm + hand3 (2026-11-16, current)
**Forest: "The tail should be the same exact makeup as the arms of the octopus."** Study (`claude/code/hippos/tailarm/tailarm_study.py`, renders A / B / C): 6 joints + socket (366 mm), 7 joints + socket (422 mm), a whole arm (8 joints + hand3). **Picked: C, a whole octopus arm with hand3; AUX pin as a second 12 V feed.**

Code: `claude/code/hippos/tailarm/tailarm.py` (**single source of truth for the tail**), `tailarm_checks.py` (-> `tailarm_checks.json`, about 5 min), `tailarm_meshcheck.py` (FCL), `tailarm_study.py`; `cephalo4_arm.py` (save as `cephalo4.py` on the path: the cephalo4 geometry + masses with a `pod` argument on `arm_points` / `joint_torques`) and `viz4_arm.py` (viz4's arm body without the pod). STL: `tailarm_stl/tail_root_cradle_STS3250_PETG.stl`. Status: **the arm is the Cephalo-2 arm as defined (layout, real servos and clutch parts); its link cradle CAD is the open item shared with the octopus (fall-load rules). Root cradle = true solid, fit-checked.**

- **What it is:** 8 servo-in-link joints p-y-p-y-p-y-p-y at 56 mm, STS3250 j0 + 7 x STS3215, clutches on j0 (1.2 x stall) and j2-j7 (0.9 x stall), j1 rigid, winch line with an N20 worm winch at the root, tip camera, SFF socket + hand base, hand3 (hub C2, used as a hand: its rotor never spins on the tail, the SFF cannot carry the 55 A motor current). Mounted on the unchanged SFF tail mount (rump socket, hand base, tail adapter): the arm's root face is the adapter servo flange (z_c -35), **j0 85 mm behind the SFF seat**, pitching about the horse's y axis. Spigot 478 mm, hand3 heel 637 mm from the root face.
- **New part, `root_cradle_arm`** (PETG, 17.6 g at 70 %): Ø56 x 4 disc on the adapter flange (4 x M3 at r 25), walls that grip the STS3250 case on its proximal half (2 x M2.5 clamp slots per side), case-stop ledges, N20 winch bay (12 x 34 x 10 envelope, est.) between the disc and the case on two strap rails, Ø8 wire / line hole. Clearances on the solids: j0 clutch cam (r 25) 2.5 mm, case 0.1 mm (seated), winch 0.3 mm, adapter / rump socket 0 overlap. One watertight body, printed disc face down, 175 mm2 of small overhangs (ledges, slot roofs).
- **Mass:** arm with tip bay and line 0.807 + hand3 with connector 0.426 + winch 0.024 + root cradle 0.018 + mount 0.050 = **1.33 kg** (old tail 0.712). Centauri-1 now about 33.4 kg (est.).
- **Ranges:** the octopus j0 / j1 limits came from its mantle and neighbours; on the rump a straight arm swings the full +-100 deg of the link shells in pitch and yaw without touching the horse.

Poses (`tailarm_checks.json`; same IK as the octopus arms with the load term at 80 % of rated; horse + Cephalo-2 surfaces as a KD-tree; every pose then mesh-checked with FCL, no clash):

| Pose | Worst servo (% rated) | Seat moment | SFF lugs | Current | Clearance (mesh) |
|---|---|---|---|---|---|
| carried high (raised, curled; render pose) | 80 | 0.86 N m | x11.5 | 4.9 A | 27.8 mm |
| rest (hanging, least load) | 80 | 2.41 | x4.1 | 4.7 | 27.8 |
| hand to the side (0.30 m out) | 80 | 2.78 | x3.6 | 6.0 | 27.8 |
| over the back towards the octopus | 80 | 0.81 | x12.2 | 6.2 | 15.5 |
| kickstand standing (heel on the floor, 33 N = 10 % of Centauri-1) | **54** | 2.65 | x3.7 | 3.5 | 27.8 |
| kickstand reared 30 deg | 80 | 1.49 | x6.7 | 5.8 | 27.8 |
| kickstand reared 45 deg | 80 | 0.88 | x11.2 | 3.9 | 27.8 |
| held straight back, level (not a working pose) | **249** (83 % stall) | 4.73 | x2.1 | 8.9 | 27.8 |

- Like the octopus arms, it works hanging, raised or curled; **straight out level overloads j0 (249 %)**: firmware keeps it out of that pose except briefly. hand3 stays >= 65 mm from its own arm in every pose.
- **Kickstand:** standing needs an S-fold (the column is 637 mm for a 450 mm drop): planar grid search gives 54 % of rated; rearing uses the yaw joints (full IK). The hand stands on its heel as the octopus does.
- **Power:** 12 V pin + AUX as a second 12 V feed (10.4 A, AUX switched on by the rump after reading the tail ID). Poses draw 3.5-6.2 A; all eight servos at stall would be 23.1 A, so the STS protection currents must keep the sum <= 10.4 A.

Superseded: tail v1 (`hippos/tail.py`, 4 joints STS3095 / STS3250 / 2 x STS3215, 400 mm, layout C, 712 g, its links, tip socket and checks `tail_checks.py`; Forest picked 400 mm + kickstand + layout C on 2026-11-16 the same day). The SFF tail mount (rump socket, adapter, rump pad) is unchanged and carries the arm.

Open (tail):
1. Arm link cradle CAD (shared with Cephalo-2): bearing pair at every joint, stop faces >= 45 mm out, TPU stop pads (`falls-and-recovery.md` rules).
2. Measure the STS3250 (clamp holes, shaft position) and the N20 winch block, then re-run the root cradle checks.
3. Dynamics: swinging, kickstand touchdown, trot with the tail raised (thermal), current limits per servo.
4. Mass budget / front share and leg margins with the 1.33 kg tail and the 14.81 kg Cephalo-2 (open item 7).
5. Print one mount + root cradle first (sleeve fit, click, rocking, case seat).

## Open items
1. **Dynamics sim.** All leg numbers are static/quasi-static. Needed: MuJoCo model of Hippos-1 solo and docked, walk / trot / fast trot, kneel and get up, landing; with the overload monitor. Covers the pauldron leg inertia, the swing unit and the tail.
2. Belt stock: HTD 595-5M x 25 mm PU/steel to confirm with a supplier.
3. Shock spring rate (unpublished): weigh on the bench.
4. Real-kinematics arm checks beyond the poses already solved (arm against arm in all centaur poses).
5. Then CAD of the agreed design, neck receiver first (interface to the octopus).
6. **Put hand3 on all eight arms into the centaur arm study and preview:** hand3 + connector (159 mm, 426 g) instead of LEAD 60 mm and the 6 in discs; use the two new poses above; re-solve T1-T3 and T5 with the load term and re-run the rules and the collision check.
7. **Re-run the horse with the 14.81 kg Cephalo-2** (was 12.81): front share, leg actuator margins, belt, shocks, dock capture.
