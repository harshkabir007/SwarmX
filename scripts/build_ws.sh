#!/usr/bin/env bash
# Build the SwarmX ROS 2 workspace (run as your normal user after scripts/install_deps.sh).
set -eo pipefail
cd "$(dirname "$0")/../ros2_ws"
# ROS 2 Jazzy uses the system Python 3.12 - keep pyenv/conda shims out of the build
PATH=$(echo "$PATH" | tr ':' '\n' | grep -vE 'pyenv|conda' | paste -sd: -)
export PATH
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install "$@"
echo
echo "Done. In every new shell:  source $(pwd)/install/setup.bash"
