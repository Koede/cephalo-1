# Hand/arm coupler — connector rev B.1 (2026-11-18)

Cephalo-2's tool-change joint at every arm tip. Motor power passes through it, so motor hands (hand3 rotor, wheel hand) swap without re-wiring. Code: `claude/code/handmount_b.py` (geometry, checks, STLs), `claude/code/coupler_hand3_fit.py` (hand3 on the coupler), `claude/code/coupler_viewer.py` + `coupler_viewer_template.html` (3D viewer). Viewer: https://claude.ai/artifact/Cp2PWLZZN2RphhcBBLhQCe

## How it works
- **Push straight in.** A key rib on the hand's spigot sets the clocking. Three Castle Creations 4 mm bullets (75 A) carry the motor phases. The five spring pins (12 V, GND, servo data, servo return, hand ID) are kept from rev A.
- **Turn the collar 60°.** Three internal lip segments slide behind the three flange lugs. A 0.6 mm ramp draws the hand home over the first 25°. The lugs then sit on a flat seat, so load cannot back-drive the collar.
- **Detent.** A 3 mm steel ball on a spring sits in a radial blind bore in the socket body (az 0, z 8.5). It clicks into dimples in the collar bore at "open" and at "locked".

## What changed from rev B → B.1 (found by checking how it goes together)
1. **The rev B collar could not be assembled, and it did not hold the hand.** In rev B, the collar's lip (r 20.9) was smaller than the socket body (r 21). The collar could not slide on from either end. Under a pull-out load there was no face for the collar to bear on, so the collar would have come away with the hand.
   - **Fix:** the socket body is now r 20.3 and the collar bore is r 20.5.
   - A separate **nose ring** (Ø47 × 3.5) sits at the mouth. The collar slides on from the hand end first. The nose ring then goes on, clamped by the socket screws.
   - The collar's internal step (414 mm² annulus) bears on the nose ring. That is the load path from lugs → lip → collar → nose ring → screws → tip bay.
   - The hand's flange seats on the nose-ring face.
   - The collar has 0.2 mm axial play between the nose ring and the retaining flange.
2. **Socket screws now clear the bullets.** The first layout had 4×M3 at az 45/90 spacing, and those screw heads cut into two female bullet barrels.
   - It is now **3 × M3×20 DIN 7984 low-head** screws at r 17, az 30/150/270, each midway between two bullets.
   - The heads sit 0.2 mm below the mouth face, in Ø6.2 counterbores.
   - They thread into M3 heat-set inserts in the tip-bay end face. The tip-bay interface is now a Ø40 face with 3×M3 inserts at r 17, which replaces rev A's 46×32 plate.
3. **Radial ball detent** replaces the axial plunger hole in the 2 mm retaining flange, which was too short for a spring.
   - The spring pocket is 5.8 mm seated and 5.0 mm when pressed. The ball travel is 0.8 mm.
   - Spring: Ø2.5 OD, about 8 mm free length, about 1 N/mm (assortment part). That gives about 2–3 N on the ball and about 0.08 N·m of click torque (est.).
4. **Hand base:**
   - The hand's 4×M3 inserts moved to az 45/135/225/315 at r 13.
   - Three 3.2 × 2.6 lead grooves run in the bottom face from each bullet to a Ø13 centre pocket. The motor leads leave the bullets' solder cups, run along the grooves and go up the hand's centre bore.
   - The wheel hand housing was updated to the new insert positions; its checks still pass.

## Hand3 on rev B.1
- The tier-1 hand3 servos stand 0.5 mm above the pack plate and reach in to r 14 all the way round. A scan over every clocking found no position where cap-head screws at r 13 clear them.
  - So hand3 bolts on with **4 × M3×12 DIN 7991 countersunk** screws, flush in its pack plate.
- The collar's lip hangs 3.5 mm below the hand base, and hand3's Ø94 pack plate would hit it.
  - Hand3's pack frame (rev B.1) gets a **Ø40.6 × 4.5 boss** under a Ø35 hub disc. This leaves 1.0 mm between the turning collar and the pack plate.
  - Hand3 sits 4.5 mm further outboard. The pack frame gains about 7 g (54.5 g total).
- Checks, with the collar locked and open: no overlaps between hand3 servos, spools, frame or screws and the coupler. The lead pocket (r 6.5) lines up with hand3's spine bore (r 7).

## Check results (`handmount_b_check.json`)
- **Fits:**
  - Zero overlap locked, along the push-in path and through the whole collar sweep.
  - A wrong clocking is blocked by the key (13 mm³ clash).
  - A locked hand pulled 1 mm hits the collar (183 mm³).
  - The collar is retained both ways (83 / 68 mm³ clash when moved ±0.2 / 0.4 mm).
  - Bullet engagement is 8.2 mm.
- **Stress** (wheel-hand cases from `wheelhand_sizing.json`):

| case | lug force | worst stress | factor vs 30 MPa PA-CF interlayer | screw tension | factor vs 1 kN insert pull-out (est.) |
|---|---|---|---|---|---|
| static | 110 N | 7.2 MPa | 4.2 | 61 N | 16 |
| 20 mm step, 2 g | 190 N | 12.4 MPa | 2.4 | 112 N | 9 |
| 3 g landing on wheels | 270 N | 17.6 MPa | 1.7 | 162 N | 6 |

- The collar step bears at only 1.3 MPa on the nose ring.
- **Masses (PA-CF):**

| part | mass |
|---|---|
| socket body | 12.8 g |
| nose ring | 5.0 g |
| collar | 18.6 g |
| hand base | 14.9 g |

## Bill of materials (per arm + per hand)
- **Arm side:**
  - socket body, nose ring and collar (PA-CF)
  - 3 × female 4 mm bullets
  - 3 × M3×20 DIN 7984
  - 3 × M3 heat-set inserts in the tip bay
  - 3 mm steel ball and spring
  - rev A pin board with 5 pogo pins
- **Each hand:**
  - hand base (PA-CF)
  - 3 × male 4 mm bullets
  - 4 × M3 heat-set inserts (Ø4 × 6)
  - pad board
  - its own body screws (hand3: 4 × M3×12 DIN 7991)

## Still open
- Print and bench test:
  - lip/lug fit after printing (PA-CF shrinkage)
  - bullet insertion force: 3 bullets at about 15–25 N each (est.), so about 60 N to push in — check it is acceptable by hand
  - detent feel
  - whether the collar stays put under vibration
- Tip bay CAD: the end face needs the 3 inserts and a Ø21 wire opening. So far the tip bay is drawn as a box in the sims.
- STLs are in `handmount_b_stl/`: socket, nose ring, collar, hand base, hand3 pack frame rev B.1.

## Rev B.2 — optimised (2026-11-18, `coupler_opt.py`)
Supersedes the B.1 dimensions above. Sweep over lug width and lug outer radius, thicknesses raised until every hand-calc stress has factor >= 2.0 in the 3 g wheel landing; mass from the CAD.
- **Stress model gained the collar-wall check.** The collar is a C-clamp (lug and nose-ring reactions both act inside the wall). Under that check the B.1 wall was at about 40 MPa in the 3 g landing: factor about 0.75, not the 1.7 reported.
- **Lugs moved to the top of the flange**, clamped between nose ring and lip. Collar is 13.2 mm long (was 26.3) and ends 1.6 mm above the hand base's bottom face, so hands bolt on flat: hand3's 4.5 mm boss is gone (pack frame 48.3 g, M3x8 DIN 7991).
- Chosen: lug 48 deg, lug r 20.4-22.4, t_lug 2.6, t_lip 2.7, wall 3.3, step 2.4, nose ring r 22.5, collar OD 53.4, flange 7.0 thick, ramp 0.4 mm over 8 deg, detent at z 5.5, retaining flange z 8.0-10.0.
- 3 g landing: lug 14.7, lip 14.6, wall 15.0, step 11.5 MPa -> factor 2.01; screws 169 N (factor 5.9 est.).
- Mass: socket 12.8, nose ring 4.4, collar 9.6, hand base 12.2 g = 39.0 g (B.1: 51.3 g, -24 %).
- All fit checks pass (locked, push-in, sweep, key, pull-out, assembly, hand3, wheel hand; wheel lever 70.2 mm).
- Open: 0.1 mm nominal axial play of a locked hand (no spring element; rev A had an O-ring) - tune on prints or add one; lug-on-lip radial overlap is 1.5 mm; stresses are hand formulas, no FEA; socket body itself (12.8 g) not optimised (set by bullet length and pin board).

## Rev B.3 — O-ring preload against rattle (2026-11-18, `coupler_preload_study.py`, Forest picked option A)
Supersedes B.2 only in the nose ring. The lugs, lips, ramp, collar and hand base are unchanged.
- **Problem:** a locked hand had 0.1 mm of axial play, and only the 5 spring pins (about 2.9 N) pushed it one way. That keeps a hand3 seated only up to about 0.7 g, so it would rattle on every load reversal.
- **Study (`coupler_preload_study.png`):**
  - 0: spring pins only. 3 N.
  - **A: O-ring in the nose-ring face. 38 N.**
  - B: steel wave spring under the lugs. 21 N, insensitive to print error, but a custom size.
  - C: printed flexure lips. 78 N, but PA-CF creeps and it weakens the lip hot spot.
  - D: TPU pads on the lips. 5.6 kN/mm stiff, so print error decides everything, and the pads slide under load.
- **Built (A):**
  - NBR 70A O-ring 22 × 1.0, in a face groove r 11.0-12.4 × 0.73 deep in the nose ring (inside the bullets, outside the socket bore).
  - It presses on the hand base's flange top and pushes the hand out until the lugs bear on the collar lips.
  - One ring per arm. Nothing changes on the hands. It also dust-seals the spring-pin well.
- **Numbers** (Lindley face-seal formula; 2.9 N of spring pins included; estimates):

| | value |
|---|---|
| squeeze at lock / hand pushed fully in | 0.17 mm (17 %) / 27 % |
| preload, nominal (±0.1 mm print error) | 38 N (12-80 N) |
| hand3 stays seated up to | 9.8 g (3.1 g at the soft end) |
| collar torque to lock (ramp + detent) | 0.45 N·m by hand |
| friction then holding the collar (on top of the detent) | 0.25 N·m |
| free ring stand-off before the turn / ramp still catches the lug | 0.27 mm / 0.23 mm margin |
| groove walls to socket bore / bullet mouth / screw counterbore | 0.4 / 0.4 / 1.5 mm |

- **Checks (`handmount_b.py`):**
  - The O-ring clears the nose ring, socket, collar, bullets and screws.
  - Squeeze volume at lock is 8.6 mm³.
  - The preloaded locked hand clears the collar and nose ring.
  - All B.2 checks still pass.
- **Stress model:** the old flat 30 N preload on a lug is replaced by one lug's share of the O-ring preload at the stiff end of the band (80 N / 3). 3 g landing: lug 14.5, lip 14.4, wall 14.8, step 11.3 MPa, **factor 2.03**. Screws 186 N (factor 5.4, est.).
- **Mass:** nose ring 4.3 g (was 4.4 g), plus a 0.1 g O-ring.
- **Tuning on the bench:** the force is tolerance-sensitive (12-80 N for ±0.1 mm). Print the nose ring, measure the gap, and reprint it at the right groove depth (4 g, 0.05 mm steps). The 0.4 mm groove walls are one extrusion line: check they print.
- **STLs:** `coupler_revB4_stl.zip` (rev B.4) (socket, nose ring with groove, collar, hand base, hand3 pack frame).
- **Viewer:** https://claude.ai/artifact/Cp2PWLZZN2RphhcBBLhQCe (now with the O-ring).
- **Buy, per arm:** 1 × O-ring 22 × 1.0 NBR 70A, plus spares.

## Rev B.3.1 — corrections and lightening (2026-11-18)

**Corrections**
- **O-ring would fall out with no hand on.** The groove faces down, toward the hand, so an unstretched 22 × 1.0 ring sat loose. It is now a **21 × 1.0 NBR 70A**, stretched 4.8 % onto the groove's Ø22 inner wall, which holds it. The effective cross-section is 0.976 mm.
- **Preload was tolerance-sensitive** (12–80 N for ±0.1 mm of print error). The groove is now printed **0.85 deep** and set with **0.05 mm PET shim washers** (Ø22.2 × 24.6) under the ring. Measure the printed stack, then add shims until the squeeze is 0.17 mm. At the nominal print that takes 3 shims.

**Optimisation**
- Six blind lightening pockets go into the socket body from its back face (r 12–19, z 8 → 16). They sit between the bullets and the screws, with at least 1.0 mm round every bore.
- Socket mass: **12.8 → 9.3 g**. Walls: 1.0 mm to the bullet bores, 1.3 mm to the OD, 1.4 mm to the spigot bore.

| | B.3 | B.3.1 |
|---|---|---|
| Squeeze | 0.17 mm (17 %) | 0.176 mm (18 %), ±0.025 after shimming |
| Preload | 38 N (12–80) | 41 N (33–50) |
| hand3 held still to | 9.8 g (3.1 low) | 10.5 g (8.4 low) |
| Collar lock / hold torque | 0.45 / 0.25 N·m | 0.48 / 0.27 N·m |
| 3 g landing factor (PA-CF interlayer) | 2.03 | 2.11 |
| Screw factor | 5.4 | 5.7 |
| Socket mass | 12.8 g | 9.3 g |

All the interference and clearance checks pass (`handmount_b_check.json`).

**Buy list change:** O-ring 21 × 1.0 NBR 70A (was 22 × 1.0), plus 0.05 mm PET film for the shims.

## Rev B.4 — FEA-driven lug and lip fixes (2026-11-19)

**What the FEA found on B.2/B.3** (`coupler_fea.py`, results in `fea_B2/`):
- Setup: P2 tets, PA-CF E 3.5 GPa, frictionless compression-only contacts, socket rigid, 3 g wheel landing (109 N radial + 7.87 N·m).
- The hand formulas missed the **lug root**. The lip pushes each lug up like a cantilever, and its root (a sharp corner) carried about 32 MPa of layer-normal tension. That is a factor of 0.8–0.9 against 30 MPa interlayer strength.
- The 66° lip also ran past the 48° lug, so the lug's free corner carried load.
- Collar wall: factor 1.75–1.9. Lip, step and nose ring: fine.

**Changes**

| | B.3.1 | B.4 |
|---|---|---|
| Lug width / length (lug_w, lug_r1) | 48°, 22.4 | **56°, 23.3** |
| Lug thickness (t_lug) | 2.6 | **3.6** |
| Lip / wall thickness | 2.7 / 3.3 | **3.2 / 4.0** |
| Lug root | sharp | **R1.0 arc fillet** + 0.5 chamfer on the bottom outer edge |
| Lip | 66° wide, sharp ends | **as wide as the lug (56°)**, 0.9 chamfer on the inner top edge (clears the root fillet), **R0.8 arc fillet** to the wall, trailing end relieved 0.12 mm over 8° (the leading end keeps the 0.4 mm lead-in ramp) |
| Hand-formula factor, 3 g landing | 2.11 | **2.52** (lug-root formula now counts the fillet) |
| Mass: socket / nose ring / collar / hand base | 9.3 / 4.3 / 9.5 / 12.2 g | 9.3 / 4.8 / 13.3 / 14.2 g (**+6.3 g**) |

All the clearance and interference checks pass (`handmount_b_check.json`). That covers the push-in path, the 60° collar sweep, the ramp capture, the detent, O-ring seating and the locked/preloaded state.

**FEA on B.4** (coarse mesh about 390k DOF; `coupler_fea_B4coarse.json`; sections in `coupler_fea_B4_section_az90.png` / `_az180.png`):

The peak numbers in the JSON are **artefacts**:
- isolated single quadrature points at the 10° kink where each arc fillet ends (kept so the booleans mesh cleanly);
- one point at the top-face edge.

They grow as the mesh is refined, and the sections show the material around them at 0–5 MPa. The engineering values are the stress **bands along the fillets**, read off the sections:

| Location | B.2 (sharp) | B.4 band | Factor vs 30 MPa interlayer |
|---|---|---|---|
| Hand-base lug-root fillet, σzz / σ1 | ≈ 32 | ≈ 15–22 | ≈ 1.4–2.0 |
| Collar lip-to-wall fillet, σzz / σ1 | 15–17 | ≈ 20–28 | ≈ 1.1–1.5 |
| Collar lip, step, nose ring | ≤ 15 | ≤ 15 | ≥ 2 |

**Open item: the collar is the weak link in a 3 g landing.**
- In B.2 its wall corner looked better only because the 66° lip spread the load over more arc. With the lip now as wide as the lug, the wall carries the same force over a shorter arc.
- Options, not yet chosen:
  1. Thicker wall and a larger wall fillet. About +2 g; the collar OD grows.
  2. Print the collar in an isotropic process (SLS/MJF PA12, about 48 MPa, no layer-normal weakness).
  3. Go back to a 66° lip with a relief at **both** ends, so the extra length beyond the lug carries no load but still spreads the wall stress.
- The 30 MPa interlayer figure for PA-CF is an estimate. A pull test on a printed lug/collar pair is the real check.
