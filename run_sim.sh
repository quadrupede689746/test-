#!/usr/bin/env bash

set -eo pipefail

# Workspace location
WORKSPACE="$HOME/robodog_ws"

# Load ROS 2 Humble and this workspace
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

# Activate the project's Python environment
source "$WORKSPACE/.venv/bin/activate"

# Make the OpenQuadruped model:// resources available to Gazebo Classic.
# Gazebo expects the directory containing the open_quadruped/ model folder.
OPEN_QUADRUPED_SHARE="$(ros2 pkg prefix --share open_quadruped)"
OPEN_QUADRUPED_MODEL_ROOT="$(dirname "$OPEN_QUADRUPED_SHARE")"
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:${OPEN_QUADRUPED_MODEL_ROOT}${GAZEBO_MODEL_PATH:+:${GAZEBO_MODEL_PATH}}"
export GAZEBO_MODEL_DATABASE_URI=""

echo "======================================"
echo " Open Quadruped Simulation"
echo "======================================"
echo "Python: $(which python3)"
echo "NumPy: $(python3 -c 'import numpy; print(numpy.__version__)')"
echo "SciPy: $(python3 -c 'import scipy; print(scipy.__version__)')"
echo "Gazebo model path: $GAZEBO_MODEL_PATH"
echo "======================================"

# Launch simulation. Press Ctrl+C once and wait for ROS 2 launch to shut down
# Gazebo's server and GUI before starting another instance.
exec ros2 launch robodog_sim sim.launch.py
