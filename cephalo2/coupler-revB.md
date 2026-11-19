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
- **STLs:** `coupler_revB6_stl.zip` (rev B.6) (socket, nose ring with groove, collar, hand base, hand3 pack frame).
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

## Rev B.5 — lips overhang the lugs (2026-11-19, option 3)

The FEA on B.4 put the **collar lip-to-wall fillet** at about 20–28 MPa: when the lip is only as wide as the lug, the collar wall carries the load over a short arc. In B.5 each lip is as long as the insertion gaps allow, and both ends are relieved so the lug's end never bears on a full-height lip.

- Lugs are **50°** (B.4: 56°), gaps 56° (lug + 6° clearance), lips **64°**, so each lip overhangs each lug end by 7°. A 66° lip with 56° lugs would not pass the insertion gap, so the lugs had to narrow.
- Relief at both ends: it starts `relief_in` = 4° inside the lug edge, reaches 0.12 mm at the lug edge and keeps rising over the overhang. The leading tip keeps the 0.4 mm lead-in ramp (whichever drop is larger applies).
- Everything else as in B.4.
- All the checks pass (`handmount_b_check.json`). Hand-formula factor 2.35. Preload unchanged: 41 N (33–50).
- Mass: socket 9.3 / nose ring 4.8 / collar 13.4 / hand base 14.2 g.

**FEA** (coarse mesh, about 400k DOF; `fea_B5coarse.log`, `coupler_fea_B5coarse.json`; section `coupler_fea_B5_section_az90.png`):

| σ1 p99.9, same mesh (MPa) | B.4 at 90° | B.5 at 90° | B.4 at 180° | B.5 at 180° |
|---|---|---|---|---|
| Collar wall | 25.7 | 23.0 | 28.7 | 22.7 |
| Collar lip | 22.0 | 22.1 | 32.2 | 21.2 |
| Hand-base lugs | 23.5 | 24.9 | 55.8 | 21.7 |
| Hand-base flange | 22.3 | 32.9* | 29.6 | 23.0 |

\* On the fillet-end kink: a single-point artefact (see the section).

Fillet bands read off the sections, as engineering values:

| Location | B.2 | B.4 | **B.5** | Factor vs 30 MPa interlayer (B.5) |
|---|---|---|---|---|
| Collar lip-to-wall fillet | 15–17 (sharp corner, 66° lip) | 20–28 | **≈ 15–20** | ≈ 1.5–2.0 |
| Hand-base lug-root fillet, σzz | ≈ 32 (sharp corner) | 15–22 | **≈ 15–22** | ≈ 1.4–2.0 |

**Where this stands**
- Both critical fillets now sit at factor ≈ 1.4–2.0 against the *estimated* 30 MPa PA-CF interlayer strength, in the *estimated* 3 g wheel landing. That's borderline rather than comfortable.
- These are coarse-mesh, linear-elastic numbers. The next real data point should be a printed pull test of a lug and collar pair (bench), not another FEA round.
- If the printed parts come up short, SLS/MJF PA12 for the collar and hand base (isotropic, about 48 MPa) adds margin with no geometry change.

## Face rethink (2026-11-19): the bullets stay

**Question:** should the three motor-phase bullets be in the face at all? They force a straight push, which is why the collar exists. Studies: `coupler_face_study.png`, `coupler_face_study2.png`.

**Options considered:**

| Option | What it is | Outcome |
|---|---|---|
| B | Gold contact rings on the hand + spring pins; the hand twists, no collar | Out: needs a custom gold-plated PCB |
| B2 / C2 | Mill-Max spring pins onto Mill-Max gold target studs (stock Digi-Key/Mouser parts) | See below |
| D | One Amass MR60 plug in the face | Not chosen |
| F | MR60 plug on a lead, plugged by hand, with a twist bayonet | Not chosen |

**Decision (Forest):** keep the 4 mm bullets (rev B.5). Rule: every part must be easy to buy.
- 4 mm gold bullets are the most available part: Castle or generic, from RC shops or Amazon.
- They carry the 36 A hover current with well under 1 W of loss. Six spring pins in parallel per phase (up to 20 mΩ contact resistance each) would waste up to about 13 W per hand inside the coupler.

**Next:** print a lug and collar pair for the pull test (bench).

## Will B.5 hold? Datasheet check instead of a pull test (2026-11-19)

The B.5 FEA stress bands in the 3 g wheel landing (est.) are compared against published Z (layer-bond) tensile strengths. Worst band: 22 MPa at the hand-base lug-root fillet. Script and output: `coupler_strength_margin.py` / `.json`.

**Knock-downs** (estimates):
- Real prints reach 0.8 of the datasheet bar strength.
- Repeated landings (10³–10⁴): fatigue allows 0.5 of the static strength.

| Material (Z tensile, datasheet) | Condition | One 3 g landing, SF | Breaks at (single landing) | Landings it takes indefinitely |
|---|---|---|---|---|
| Bambu PAHT-CF (PA12-CF), 64 MPa | dry | 2.3 | 7.0 g | ≤ 3.5 g |
| | humid (−15 %) | 2.0 | 5.9 g | ≤ 3.0 g |
| Bambu PAHT-CF, older TDS 47 MPa | dry | 1.7 | 5.1 g | ≤ 2.6 g |
| | humid | 1.45 | 4.4 g | ≤ 2.2 g |
| Polymaker Fiberon PA6-CF20, 54 / 25.5 MPa | dry | 2.0 | 5.9 g | ≤ 2.9 g |
| | **humid** | **0.93** | **2.8 g** | **≤ 1.4 g** |

**Verdict**
- **In Bambu PAHT-CF, B.5 holds.**
  - A single 3 g landing has a safety factor of 1.5–2.3.
  - It only breaks in a 4.4–7 g crash.
  - It survives unlimited repeated landings up to roughly 2.2–3.5 g.
- **In PA6-CF it does not hold once the nylon has taken up moisture**: it breaks at about 2.8 g.
- **Material spec:** collar and hand base in **Bambu PAHT-CF** (PA12 base, low water uptake), printed axis-vertical as modelled. Not PA6-CF.
- The fatigue margin is about 1 at 3 g, so treat a 3 g landing as a hard landing and not the routine case. Routine landings are softer and have proportionally more margin.

## Collision checks by range math (2026-11-19, standing rule)

Collisions are now checked with math, not mesh booleans (`coupler_ranges.py` → `coupler_ranges.json`, runs in 0.2 s).

**Method**
- Every solid feature is a range box: angle × radius × height. Height is an exact z(r) formula where there's a chamfer or an arc fillet.
- Two parts collide only if all three ranges overlap. Touching (overlap 0) is a seat.
- Off-axis round parts (bullets, screws, detent ball) are checked against each other by centre-distance math. Against revolved parts they use their bounding ranges.
- A part that sits in a hole (key in the keyway, male bullet in its mouth/bore, ball in a dimple) must fit inside that hole's range. The margin is reported.

**States checked**
- Locked, hand seated and pushed onto the lips.
- Collar turning −60° → 0° in 5° steps.
- Hand pushed in from 25 mm out with the collar open.
- Collar against the arm at both detent stops, and at mid-turn with the ball retracted.
- Inside the arm.

**B.5 result:** 3365 pairs, **0 collisions**.

| Tightest | Clearance |
|---|---|
| Lug on lip (seat at preload) | 0 |
| Key entering the nose ring | touches the face, then enters the keyway with 0.3 |
| Ball in its dimples | 0.05 |
| Lug tip to socket body | 0.1 (radius) |
| Hand flange to screw heads | 0.2 |
| Collar on socket body | 0.2 |
| Sleeve to retaining flange | 0.2 |
| Male bullets in their bores | 0.6 |

**Fits (mm):**
- spigot 0.25
- flange inside lips 0.3
- lug tips inside the wall 0.5
- collar bore 0.2
- wall outside nose ring 0.4
- lug above lip, seated 0.1
- lugs pass the open lips: 3° each side

## Rev B.6 — the bullets wired end to end (2026-11-19)

The three bullets carry the motor phases from the arm's tip-bay ESC to the hand's motor. B.5 modelled the bullets and their wire paths with guessed sizes. The real parts did not fit the model, so the whole path is now sized from datasheet dimensions and checked by range math (`coupler_ranges.py`: 25,826 pairs, **0 collisions**).

**Real parts**
- **Castle 4 mm CC bullet:** 17 mm long, 5 mm OD body, 4 mm pin, 13–16 AWG, 75 A.
- **14 AWG silicone:** 3.5 mm OD, 60 A.
- **T-Motor F60 Pro V (wheel hand):** its own 20 AWG leads, 150 mm.

**What was wrong in B.5 and what changed**

| Item | B.5 | B.6 |
|---|---|---|
| Female bullet | 13 mm body, Ø6 bore, separate Ø3.2 lead hole | 17 mm body in a Ø5.2 bore **through** the socket. Mouth recessed 0.5. The solder cup sits 1.5 mm past the back face, inside the tip bay |
| ESC side | lead hole too small for 14 AWG (3.5 OD) | the ESC phase leads are soldered straight onto the female cups in the tip bay |
| Tip-bay end face | only a centre Ø21 opening, which the leads at r 15.4 would hit | **adds 3 × Ø6.0 holes at r 15.4, az 90/210/330** for the female tails |
| Male bullet | Ø4.1 bore (the body is Ø5) | 5 mm body flush in a Ø5.05 bore (epoxy), 9 mm pin above the face. Engagement 8.5 mm |
| Hand lead grooves | 3.2 × 2.6 (wire is 3.5) | **3.8 × 3.8**, closed by hand3's pack-frame disc |
| Lead route in the hand | not checked | see below |
| Hand base screws | 4 inserts at 45/135/225/315: the leads at 210/330 ran 0.1–0.4 mm into the 225/315 inserts (4-fold vs 3-fold spacing can never be more than 15° apart) | **3 inserts at 30/150/270**, midway between the leads. The countersunk heads clear hand3's servos by height (heads z ≤ 2.5, servos z ≥ 3.0) at any clocking, so DELTA stays 0 |
| hand3 pack frame | centre tube r 7–10 with a square top edge | **1.5 mm × 45° chamfer** on the tube's inner top edge, for the lead bends |

**Hand-side lead route (hand3)**
1. Side-entry solder into the male cup's saddle, with the saddle opening facing the centre.
2. Along the groove to the centre pocket.
3. 90° bend at R 3.0 (around a Ø6 mandrel; 0.86 × OD, fine-strand silicone, static).
4. Vertical at r 2.4.
5. Down hand3's clear r < 7 centre channel to the motor base (hand3 z 54). Nothing in hand3 sits inside r 7 below z 54; servos start at r 15.3.

**Clearances on the hand-side route**
- 0.15 mm to the pack-frame disc top. This is just the lead's lay height.
- ≥ 0.3 mm between leads (exact gap 0.66).
- 2.85 mm to the tube wall going down.

**Wheel hand:** the motor's own 20 AWG leads (Ø3.9 bundle) go up through its Ø6.8 back-plate hole (1.46 mm margin) into the same pocket and grooves, and are soldered into the male cups (tin them heavily: the cup is sized for 13–16 AWG). The wheel-mode limit is 13 A.

**Electrical path, end to end:** tip-bay ESC (45 A) → 14 AWG leads → female cups → 8.5 mm bullet engagement → male cups → 14 AWG (hand3) or 20 AWG (wheel) → motor.

**Masses (PA-CF / PAHT-CF):** socket 9.4, nose ring 4.9, collar 13.4, hand base 13.8 g.

**Files**
- STLs: `coupler_revB6_stl.zip`, including the chamfered hand3 pack frame.
- The viewer shows the phase leads (red) and the tip-bay holes.

**Buy list:** 3 × Castle 4 mm CC bullet pairs per arm; 14 AWG silicone (about 0.1 m per hand3, plus ESC leads); 3 M3 heat-set inserts and 3 M3 × 8 DIN 7991 per hand; epoxy for the bullet bodies.

## Motion sim: coupler + hand3 hub E (2026-11-19)

`coupler_hand_sim.py` builds an interactive three.js page from the real CAD solids, with a 30 s loop:
1. approach
2. push in (the bullets engage)
3. collar turns 60° to lock
4. rotor spin-up
5. hover (the four fingers are the rotor fins)
6. spin-down
7. spread (finger lag)
8. curl (K1/K2/K3)
9. hold
10. open
11. unlock
12. pull out

**How it is built**
- Every rigid part is exported once.
- Each frame stores only the part poses, computed with `hand3_hubc.assembly()`'s own transforms: spin, `finger_frames(lag, fold)`, and ring strokes (base = e_lag × spread + lever × K1, curl = levers × K2, K3).
- hand3 is mounted turned end for end about y (a rotation, not a mirror).
- Cords, TPU links and trim screws change shape with the pull, so they are not drawn.
- The page shows a load readout (est.):
  - rpm (hover about 10,900; full throttle 12,472);
  - phase current per bullet (36 A hover);
  - thrust through the coupler (16.3 N hover);
  - ring strokes against their travel;
  - servo state and finger pull (34.6 N rated / 103.7 N stall).
- Page: `coupler_sim_site/`. Key frames: `coupler_hand_sim_keyframes.png`.

**Fingers:** fully curled, they close to r 45.5 mm, about 97 mm below the coupler, and curl away from the arm. Nothing reaches back toward the coupler.

**hand3 hub E brought onto the coupler**
- hub E (the final hand3) had its frame-plate screws at r 9.5, az 90/210/330, right on the bullet/lead lines, and only a Ø8 centre hole.
- The plate now bolts to the coupler's 3 inserts at **r 13, az 30/150/270**. That set maps onto itself under the end-for-end turn (az → 180 − az).
- The centre hole is now **r 8**.
- Range check against hub E's real geometry (`coupler_hubE_ranges.py` cuts each static part into 2 mm bands): **91,148 pairs, 0 collisions**. The leads clear the plate-hole edge by 3.85 mm.
- The hub E change is in the workspace copy `hand3e/hand3_hubc.py`.
- The older rev D pack-frame chamfer in `coupler_hand3_fit.py` only applies to rev D.

## Correction and open item: motor leads through hand3 hub E (2026-11-19)

**Correction.** The "0 collisions vs hub E" result above was wrong.
- `coupler_hubE_ranges.py` took each band's inner radius from polygon vertices. That misses parts that span the axis, and hub E's two STS3215 servos lie across the centre (±22.6 × ±17.5 mm, z 2–51). It is fixed: a band that encloses the axis now starts at r = 0.
- With the fix, `coupler_ranges.py` reports the leads running into the curl servo.
- **There is no centre channel in hub E.**

**Whole-sim overlap check** (`coupler_sim_overlap.py`: occupancy grids, 0.5 mm cells, every 1.5 s plus the key states) also found:
- **Leads × servos:** the item above.
- **Empty-hand curl:** at full spread + full curl (90/90/85°) the opposite fingertips pass through each other (64 mm³) and into the hub and heel. Those limits are for wrapping an object; an empty hand must stop where the tips meet.
- **Detent ball:** it does not retract while the collar turns.
- **Tip-bay sketch:** it has no insert bores for the 3 screws.
- **Leads in their solder cups:** drawn inside the cups.
- **O-ring:** shows its intended 0.17 mm squeeze.

**Where the leads can go** (`coupler_lead_route.py`: 1 mm free-space grid; rotor, cage and rings swept through a turn and their full stroke; Dijkstra; frame cells allowed at a cost):
- **14 AWG (3.5 OD):** no path even through the frame.
- **18 AWG:** a path exists with 0.3 mm clearance. The SE 2816 ships with 18 AWG × 25 cm leads.
- Phase 90 runs down the +y corridor; phases 210 and 330 run down the −y corridor (x ±8.5). The corridors are at y ±23.5, beside the servo faces and below the rings' lowest point (z 47).
- Hub E would need two 5–6 mm tunnels in its frame top block (z 51.4–55) into the centre hole under the motor, plus two plate notches.
- The hand-base grooves would run outward to the corridors.
- Study: `hubE_lead_route_study.png`. **Awaiting Forest's go-ahead before building it.**

## Motor leads through hub E: wider options study (2026-11-19)

Files: `hubE_lead_options_study.py`, `.png`, `.json`.

Two findings changed the picture from the first route search:

- **The first search left the moving cords out.** The two "far" cords cross the front (+y) face of the servo stack at y 22.5
  (z 10.4 and z 35). The front-corridor lead of the first route (y 23.5) would rub both of them (gap -0.5 mm). That route is rejected.
- **The back (-y) face of the servo stack is empty.** No spools, no cords (nearest cord 26 mm away), free space out to 12+ mm
  over the full height z 2 to 51. Three 14 AWG leads fit there side by side with no change to the hand outline.

14 AWG could not be routed before only because of the last 8 mm: the gap between the motor base (r 17) and the swept cage (r 19.5)
is 2.5 mm, too small for a 3.5 mm wire. The motor's own 18 AWG leads (2.3 mm) fit that gap, so the 14 AWG is joined to short motor
stubs on the back face, below the cage.

Loss at 36 A hover, 3 phases, 80 mm run + 20 mm motor stub (est.): A uncut 18 AWG 8.1 W; B copper strip 3.3 W; **C 14 AWG 4.2 W
(recommended)**; D 16 AWG 5.7 W; E split front/back 4.2 W but 0.7 mm from a moving cord; F C in a printed rail 4.2 W; G old corridors
rejected; H servo refactor not needed. Decision pending.

## Rev B.7: motor leads in a cover rail on hub E's back face (2026-11-19)

Chosen from `hubE_lead_options_study.png`: **option F** (14 AWG down the back face inside a printed rail).
Files: `hubE_leads.py` (one place for the lead geometry, the rail, the frame cuts and the clearance math), `hubE_leads.json`.

**Route of each phase.** Male bullet cup (side entry) -> groove in the hand-base bottom face -> its lane in one exit slot ->
45 deg dive between the collar lip and the servo corner, through a notch in hub E's frame plate -> up the back face of the servo
stack inside the rail -> inline lap joint to the motor's own 18 AWG lead, cut to a 14 to 25 mm stub -> through a notch in the top
block -> motor base. The 5 signal wires share the slot and the rail in their own lane.

**What changed**

| Part | Change |
|---|---|
| Hand base (hand3 variant) | Grooves follow the new paths; one exit slot through the rim at az 268 to 308 (middle of a lug gap, 1.0 mm of full rim left to the nearest lug end); centre pocket r 6.5 -> 4.5 (signal wires only); 4 M3 inserts at az 50 / 170 / 240 / 355 (were 3). 13.1 g (was 13.8). |
| Collar | 1.2 mm chamfer on the lips' bottom inner edge (not a bearing face). Gives the diving leads 0.7 mm to the lip and 0.7 mm to the servo corner. |
| hand3 mount | Clocked 20 deg on the coupler so the back face points at the lug gap. |
| Hub E frame | No centre hole; 4 countersunk holes; plate notch 14.5 x 3; top-block notch 8.4 wide; two bosses for the rail screws. 27.7 g. |
| New: lead rail | PA-CF U-channel 15.4 x 6 x 44.4 mm, 1.75 g, 2 x M2 x 6 thread-forming screws. Reliefs for the servo rear axle boss and the signal wires. |

**Numbers.** Lead lengths 45 to 80 mm of 14 AWG plus 14 to 25 mm of 18 AWG. Loss 3.6 W at 36 A hover, 8.2 W at 54.4 A max (est.).

**Checks**
- `hubE_leads.py`: exact distance along every lead to collar, servos, plate, bridge, top block, cords, spinning cage and curl ring,
  rail walls, inserts, other bullets: **0 collisions**. Tightest real gaps: top block notch 0.15, frame bridge 0.25, servo 0.71,
  collar 0.72, spinning curl ring (fully pulled) 1.19, spinning cage 1.20, moving cords 12.6 mm.
- `coupler_ranges.py`: 3442 pairs, **0 collisions** (lead results included).
- `coupler_sim_overlap.py` (whole motion sim, 0.5 mm cells): **only the O-ring squeeze is left** (1.75 mm3, intended).

**Other fixes in the sim so that nothing overlaps**
- Empty hand: opposite fingertips cross from 72 % of full curl at full spread. The sim now stops at 70 % ("until the tips meet").
- Detent ball backs 0.8 mm into its bore while the collar turns.
- Tip bay sketch has insert bosses and bores for the 3 socket screws.
- Rail screws are M2 x 6, not x 8: an 8 mm screw would poke out of its boss toward the collar (bottom) and the curl ring (top).

**Estimates to measure on the real parts**
- SE 2816 lead exit height on the motor base (modelled 1.3 mm above the mount face). The spinning cage starts 3 mm above it.
- STS3215 rear axle boss size and the positions of its two bus ports (rail windows at zh 12 and 37 are placeholders).
- Joint size (lap joint 14 -> 18 AWG under heat shrink, taken as dia 3.8 x 9).

**Open**: the wheel hand keeps the B.6 hand base (radial grooves to the centre); the B.7 grooves are the hand3 variant.

## hand3 independent fingers: option F chosen, root-lock layout study (2026-11-19)

Decision: **F, roots travel round the hub** (from `hand3_indep_fingers_study.png`). Rule added by the user: joints must stay steady
under rotation, so nothing on the rotor may be held by a servo or friction while it spins. User's idea: pins that slide out through
little holes and couple the parts crosswise. Study: `hand3_root_lock_study.py/.png/.json` (estimates). Recommended L1: dia 3 steel
dowel in each root carrier, thrown outward through one of 24 holes (15 deg) in a rim wall at r 24; 22.8 N seat force at 12,500 rpm,
6.5 N side load at full motor torque, release by a parked pusher. 15 deg holes need a hole circle of r 17.2 or more, so they cannot
sit on the centre spindle. Decision on the lock pending.

**Root lock, user's correction (2026-11-19):** the pins are to be **axial** (parallel to the shaft), **two per root, one on each
side of the root's centre line**. First numbers (hand calc, est.): two dia 3 steel dowels at +-15 deg on an r 18 hole circle (24 holes,
web 1.7 mm, 9.3 mm apart, 1.3 mm clear of the dia 5 swing pin). If the pair took the whole 826 N pull: 29 MPa shear per pin (double
shear), 34 MPa hole bearing in 2 mm leaves. Under spin the pull loads the pins sideways, so about 124 N of friction (mu 0.15) resists
withdrawal against under 1 N of axial load; a spring holds them at rest. Open: positive keeper for the spinning state, and how the
pins are withdrawn and the root indexed while parked. Not drawn yet.

**Axial root lock, to-scale layout (2026-11-19):** `hand3_root_lock_axial.py/.png/.json`. Each root sits on a shoe (two arc leaves,
46 deg wide) between a base disc and a top disc; the finger's 826 N pull goes from the shoe edges into rim lips on both discs
(14 MPa); two dia 3 axial pins at +-15 deg on r 17.7 (24 holes, web 1.58 mm) stop it moving round; a bridge with a tab joins the pin
pair; a parked fork lifts the tab 2.9 mm and holds the shoe while a roller turns the hub in 15 deg steps. Roots no closer than 60 deg.
Open: (1) spinning keeper: the pull is on the lips so the pins are not jammed, only a 3 N spring holds them; (2) stack 13 mm leaves
0.2 mm of shaft for the nut; (3) 3 + 1 and one-sided poses must never spin (826 / 1430 N); 2 + 2 is balanced; (4) cords to a root
that has moved; (5) roller and fork parts not chosen.

**Root lock, crystallised concept "bolt-lock shoe" (2026-11-19, deliberation; hand estimates, not drawn or checked yet):**
1. Keeper: pins stay truly axial. Each pin gets a 0.3 mm neck at the base disc's lower face and the 12 holes are radial slots
   (3.05 x 3.45). A spring and, when spinning, the finger's own 826 N pull push the shoe 0.35 mm outward onto the rim lips, so the
   neck hooks under the slot edge: the pins cannot lift. Release when parked: the fork first nudges the shoe 0.35 mm inward, then lifts.
2. Steps: 30 deg, 12 holes (cross at 0/90/180/270; 2 + 2, 3 + 1 and one-sided are all multiples of 30 deg). Web between holes 6.2 mm.
3. Height: hub_lo 91 -> 90, discs 2.5 -> 2.0, leaves 1.5 -> 1.25: stack 11.5 mm, 2.7 mm of shaft left for the heel nut (today 3.2);
   plus 4 screws through the top disc into the centre boss.
4. Spin interlock with no wires on the rotor: one fixed reflective sensor at the fork station; before spin-up the roller turns the
   rotor once and the firmware checks four tabs, all down, 90 deg apart (roller servo encoder).
5. Actuators: one small servo on a 3-position cam (out / roller in / fork lift), one continuous-rotation servo for the roller.
6. Cords: each ring gets a top bead flange; a C-clip traveller per finger slides round it (ring-and-traveller); the cord runs from
   the traveller through an eyelet in the shoe tab. Same cord length at any azimuth. Least mature part.

**Root lock v3, "rail block + two axial latch pins" (2026-11-19):** `hand3_root_lock_bolt.py/.png/.json`, to scale with interval checks.
Found while laying out the bolt-lock shoe: the swinging root grip (lag) sweeps the space the pins need. A numeric sweep of the CAD
grip gives a shoe 56 to 68 deg wide for pins clear of it, so roots could not sit 60 deg apart, and lift tabs cannot cross the grip.
Proposal: drop the sideways swing; the finger is fixed in a root block and root travel gives the 2 + 2 pose instead.
Block: PA-CF wedge 54 deg wide between two 2 mm discs, 6 mm gap (stack 10 mm = today, so shaft and nut are unchanged); two ribs top
and bottom in ring grooves carry the 826 N (11.8 MPa bearing; neck 13.8 MPa). Lock: two L-shaped dia 2 music-wire pins 7.2 mm either
side of the centre line, legs into 24 slot holes on r 20 (web 2.1 mm), arms out past the rim as lift tabs; a 0.4 mm foot on each leg
catches under the disc when the spin throws the pin 0.5 mm outward (15 N). 30 deg steps. 20 N sideways on a fingertip: 14 MPa at
the hole edges. Open: cord path from the curl ring (it sits under the disc); spin interlock for 3 + 1 / one-sided poses; fork,
roller and sensor parts; the decision to drop the swing (pending).

Decision (2026-11-19): **drop the sideways swing (lag)**; finger fixed in its root block; root travel gives 2 + 2. Next: cord path layout (today's cords run up inside the hollow swing pin at r 20, a path that disappears), then CAD of the v3 hub.

**Cord route study (2026-11-19):** `hand3_cord_route_study.py/.png/.json`. With roots that travel, the one-piece base disc (out to
r 25.5) means anything below it that follows a root must go round the rim; 3.2 mm of height under the disc above the rings.
Recommended B: keep only the outer (base) pull ring with a music-wire hoop on top; each finger's cord ends in a C-clip bead that
slides round the hoop (ring-and-traveller), goes out under the rim and up into the finger root; K1 to K3 curl together on one cord;
the inner ring and one STS3215 go (-55 g). Needs the static yoke lip's inner edge moved out 0.6 mm (r 27.1 -> 27.7). A (two hoops,
separate K1) works but is tight; C hanger arm, D Bowden, F rod through the shaft rejected; E ball-in-cradles is the fallback.

**GPU FEA on Saturn (2026-11-19):** `coupler_fea.py` has an optional CuPy solve (FEA_GPU=1): pyamg hierarchy built on the CPU,
Jacobi-smoothed V-cycle PCG on the RTX 3060. Test run (206k + 99k + 41k dofs, one load case): about 300 to 350 CG iterations per
contact step, whole case in about 4 minutes on Saturn.
