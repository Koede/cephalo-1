#!/usr/bin/env bash
# Octopus Drone (Cephalo-1) workspace setup. Run once at the start of every session:
#   bash setup_env.sh
set -e
pip install --break-system-packages -q numpy scipy numba trimesh manifold3d shapely pillow mujoco cma
command -v ffmpeg >/dev/null || echo "WARNING: ffmpeg missing"
python3 - <<'EOF'
import numpy, scipy, numba, trimesh, manifold3d, shapely, PIL, mujoco
print("env ok: numpy", numpy.__version__, "| scipy", scipy.__version__, "| numba", numba.__version__,
      "| trimesh", trimesh.__version__, "| pillow", PIL.__version__, "| mujoco", mujoco.__version__)
EOF
