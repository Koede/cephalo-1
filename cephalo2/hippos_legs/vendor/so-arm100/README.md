# STS3215 servo shape

`sts3215_03a_no_horn_v1.stl` is the Feetech STS3215 (03a) case without its horn, copied unchanged from
TheRobotStudio's SO-ARM100 repository (`Simulation/SO101/assets/`, commit 141aea7bcac2744d92998e30268c413202a56f96),
which is published under the Apache License 2.0 (copy in `LICENSE`). Units are metres; the output spline points
along +z and sits at x = +12.5 mm.

`hardware.sts3215()` loads it for the leg CAD. The mesh is simplified inside (solid, mounting holes only 1.5 mm
deep), so the CAD cuts the M2 screw holes deeper itself.
