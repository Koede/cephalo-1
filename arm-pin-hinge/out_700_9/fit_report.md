# Pin-hinge helix arm: fit and collision report

- Arm length 700 mm from joint 0 to the tip face: 9 joints x 77.8 mm pitch (shortest that clears: 60.5 mm). Hinge axes turn 60 deg per joint.
- Servos: STS3250 in joints 0-3, STS3215 in joints 4-8 (same case, so every cradle is the same).
- Outer diameter 57.1 mm, set by the fork ears and the screw heads on the hinge axis. The servo cradle alone would fit in 53.5 mm.
- Servo stack along the hinge axis: idler screw head -28.3, idler ear -26.1 to -22.9, cradle floor -22.4, servo case -19.5 to 15.5, horn 19.5 to 22.5, horn ear to 26.5 mm.
- Printed parts: root socket 23 g, vertebra 32 g (x8, all identical), tip 38 g, lid 2.5 g (x9).
- Outside the mantle one arm weighs about 854 g with servos and hardware (954 g counting the socket, root servo and its lid).
- Static interference with every joint at zero: 0.00 mm^3 worst pair.
- Vertebra 2 minus vertebra 1 turned 60 deg: 0.00 mm^3, so one STL serves every moving vertebra.

Collision-free travel (5 deg steps, overlap < 2 mm^3, servo cases and horns included):

- Joint 0: -90 to +90 deg (covers the +-45 deg command range)
- Joint 1: -90 to +90 deg (covers the +-45 deg command range)
- Joint 8: -90 to +90 deg (covers the +-45 deg command range)
- Joints 0 and 1 turned together, link 0 against link 2: clear to +-60 deg in both senses
- Joints 1 and 2 turned together, link 1 against link 3: clear to +-60 deg in both senses

Tip, straight: [0.0, 0.0, 700.0] mm; curl (all joints 40 deg): [322.9, -189.8, 423.7] mm.
