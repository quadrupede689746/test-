#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$WS_DIR"
colcon build --symlink-install
source "$WS_DIR/install/setup.bash"
echo "Build complete. Source this workspace with: source $WS_DIR/install/setup.bash"
