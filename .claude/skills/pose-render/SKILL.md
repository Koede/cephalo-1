---
name: pose-render
description: Export a cephalo-1 arm pose (named or joint:deg list) as an STL assembly and a side/front PNG with joint axes. Use to look at a pose or generate previews (the repo has no generator for render.png).
---
# pose-render

`python3 tools/pose_render.py [--pose straight|curl|wave|zigzag|coil | --angles 0:30,1:-20] [--out DIR] [--png file] [--no-stl]`

Prints the tip position, writes `pose.stl` (printed links + servos placed with `pin_cad.pose_frames`) and `pose.png`. For a photo-style render, load the STL in the Three.js viewer tool or Figma/Blender; the PNG is a schematic. Pair with `/singularity-scan` or `/collision-check` output (`--pose` strings use the same `joint:deg` format).
