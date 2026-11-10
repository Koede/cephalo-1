# Pin-hinge helix arm: fit and collision report

- Arm length 1000 mm from joint 0 to the tip face: 13 joints x 76.9 mm pitch (shortest that clears: 60.5 mm). Hinge axes turn 60 deg per joint.
- Servos: STS3250 in joints 0-4, STS3215 in joints 5-12 (same case, so every cradle is the same).
- Outer diameter 57.1 mm, set by the fork ears and the screw heads on the hinge axis. The servo cradle alone would fit in 53.5 mm.
- Servo stack along the hinge axis: idler screw head -28.3, idler ear -26.1 to -22.9, cradle floor -22.4, servo case -19.5 to 15.5, horn 19.5 to 22.5, horn ear to 26.5 mm.
- Printed parts: geared root socket 30 g, vertebra 31 g (x12, all identical), tip 37 g, lid 2.5 g (x12; joint 0 has none).
- Outside the mantle one arm weighs about 1243 g with servos and hardware (1394 g counting the geared root: socket, caps, root servo, gear stage and bearings).
- Static interference with every joint at zero: 0.00 mm^3 worst pair.
- Vertebra 2 minus vertebra 1 turned 60 deg: -0.00 mm^3, so one STL serves every moving vertebra.

Collision-free travel (5 deg steps, overlap < 2 mm^3, servo cases and horns included):

- Joint 0: -55 to +55 deg (covers the +-45 deg command range)
- Joint 1: -90 to +90 deg (covers the +-45 deg command range)
- Joint 12: -90 to +90 deg (covers the +-45 deg command range)
- Joints 0 and 1 turned together, link 0 against link 2: clear to +-60 deg in both senses
- Joints 1 and 2 turned together, link 1 against link 3: clear to +-60 deg in both senses

Tip, straight: [0.0, 0.0, 1000.0] mm; curl (all joints 40 deg): [429.1, -305.1, 613.3] mm.

## Geared root socket (2:1 spur stage on joint 0)

- Printed: socket 30 g (old socket 23 g), end cap 1.9 g, pinion bearing cap 2.1 g; joint 0 drops its lid (2.5 g).
- 7075-T6: 40T gear 15.0 g (sector with teeth over +-65 deg, 10 mm face), 20T pinion 6.4 g, idler pin 1.3 g. Bearings: 61803 7.5 g, 695 2.2 g. Extra hardware 5.4 g [A].
- Added mass per arm: 46.2 g against the 45 g estimate and the 46 g budget that keeps thrust-to-weight at 1.15.
- Robot 15.08 kg, thrust-to-weight 1.150.
- Servo bulge: the servo sleeve floor sits 6.8 mm outside the 57.1 mm tube on one side; outside the mantle that is the stretch from the flange face (-37.7 mm) to -16.7 mm from the hinge. Link 1 never reaches it (it stays inside the tube radius along the hinge axis).
- Mantle cut-out (joint-0 frame, mm): alpha -35.9 to 25.0, beta -21.1 to 21.1; outline in mantle_cutout_joint0_frame.svg.
