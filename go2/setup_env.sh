#!/usr/bin/env bash
# Create the `go2` conda env and install the SDK from this checkout.
#
#   go2/setup_env.sh              # SDK only (the computer at the robot)
#   go2/setup_env.sh --with-sim   # also the simulator, for testing without a robot
#
# The simulator is cloned next to this repo, where go2/sim_headless.py looks for it.
set -euo pipefail

SIM_COMMIT=1eb6642
ENV_NAME="${GO2_ENV_NAME:-go2}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SIM_DIR="$(dirname "$REPO_DIR")/unitree_mujoco"

WITH_SIM=0
case "${1:-}" in
  "") ;;
  --with-sim) WITH_SIM=1 ;;
  *) echo "usage: $0 [--with-sim]" >&2; exit 2 ;;
esac

if ! conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
  conda create -y -n "$ENV_NAME" python=3.10 pip
fi
ENV_PY="$(conda run -n "$ENV_NAME" python -c 'import sys; print(sys.executable)')"
export PYTHONNOUSERSITE=1  # stop ~/.local packages shadowing the env

"$ENV_PY" -m pip install -e "$REPO_DIR"
"$ENV_PY" -m pip install matplotlib==3.10.9  # for go2/lidar_map.py

if [ "$WITH_SIM" = 1 ]; then
  if [ ! -d "$SIM_DIR/.git" ]; then
    git clone https://github.com/unitreerobotics/unitree_mujoco.git "$SIM_DIR"
  fi
  if ! git -C "$SIM_DIR" cat-file -e "$SIM_COMMIT^{commit}" 2>/dev/null; then
    git -C "$SIM_DIR" fetch --unshallow origin 2>/dev/null || git -C "$SIM_DIR" fetch origin
  fi
  git -C "$SIM_DIR" checkout --quiet "$SIM_COMMIT"
  "$ENV_PY" -m pip install mujoco==3.17.0 pygame==2.6.1
fi

"$ENV_PY" -c "from unitree_sdk2py.go2.sport.sport_client import SportClient; print('SDK imports OK')"
echo "Done. Use: conda activate $ENV_NAME; export PYTHONNOUSERSITE=1"
