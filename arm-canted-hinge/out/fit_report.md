# Canted-hinge arm: fit and collision report

- Arm length 585 mm from the socket face: 7 moving vertebrae x 83.6 mm pitch (shortest that fits: 80 mm), plus the root socket in the mantle. Cant 45 deg, 7 joints, 7 x STS3215.
- Vertebra outer diameter 66.6 mm (servo pocket + 1.8 mm wall).
- Printed mass 418 g per arm including the 50 g socket (PETG). Outside the mantle the arm weighs about 858 g with servos and hardware.

| Part | Mass g | Servo outside pocket mm^3 | Bearing land max radius mm | Hinge axis offset from centreline mm |
|---|---|---|---|---|
| root socket | 50 | 0.1 | 29.5 | 4.5 |
| vertebra 1 | 54 | 0.1 | 29.5 | 4.5 |
| vertebra 2 | 54 | 0.1 | 29.5 | 4.5 |
| vertebra 3 | 54 | 0.1 | 29.5 | 4.5 |
| vertebra 4 | 54 | 0.1 | 29.5 | 4.5 |
| vertebra 5 | 54 | 0.1 | 29.5 | 4.5 |
| vertebra 6 | 54 | 0.1 | 29.5 | 4.5 |
| vertebra 7 | 42 | 0.0 | 0.0 | 0.0 |

Collision-free travel of each joint (5 deg steps, adjacent vertebrae, overlap < 2 mm^3):

- Joint 0: -180 to +180 deg against its neighbour (covers the +-60 deg command range); with joints 0 and 1 turned together, vertebrae 0 and 2 stay clear from -180 to +180 deg
- Joint 1: -180 to +180 deg against its neighbour (covers the +-60 deg command range); with joints 1 and 2 turned together, vertebrae 1 and 3 stay clear from -180 to +180 deg
- Joint 2: -180 to +180 deg against its neighbour (covers the +-60 deg command range); with joints 2 and 3 turned together, vertebrae 2 and 4 stay clear from -180 to +180 deg

Tip position, straight: [0.0, 0.0, 668.6] mm; curl pose: [201.6, 400.3, 268.3] mm; coil pose: [281.1, 293.8, 347.2] mm.
