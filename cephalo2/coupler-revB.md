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
