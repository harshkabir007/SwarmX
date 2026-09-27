#!/usr/bin/env bash
# Headless Gazebo end-to-end test in the SwarmX ROS container.
#   ROBOTS=3 TASKS=6 LOC=static EXEC=direct SCEN=random TIMEOUT=600 bash scripts/gazebo_e2e.sh
# Builds the workspace inside the container (sources mounted read-only), launches the fleet,
# and passes when every task is delivered with zero ground-truth contacts.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-swarmx-deps:jazzy}"
docker image inspect "$IMAGE" >/dev/null 2>&1 || docker build -f "$ROOT/docker/Dockerfile.deps" -t "$IMAGE" "$ROOT/docker"
exec docker run --rm \
  -e ROBOTS="${ROBOTS:-3}" -e TASKS="${TASKS:-6}" -e LOC="${LOC:-static}" -e EXEC="${EXEC:-direct}" \
  -e SCEN="${SCEN:-random}" -e TIMEOUT="${TIMEOUT:-600}" -e PALLET="${PALLET:-}" -e PALLET_AT="${PALLET_AT:-60}" \
  -v "$ROOT/ros2_ws/src:/ws/src:ro" -v "$ROOT/scripts:/scripts:ro" "$IMAGE" bash -c '
    source /opt/ros/jazzy/setup.bash
    cd /ws && colcon build > /tmp/build.log 2>&1 || { tail -30 /tmp/build.log; exit 1; }
    source install/setup.bash
    source install/swarmx_bringup/share/swarmx_bringup/config/zenoh/swarmx_zenoh.env
    export LIBGL_ALWAYS_SOFTWARE=1
    ros2 launch swarmx_bringup sim_fleet.launch.py robots:=$ROBOTS tasks:=$TASKS localization:=$LOC executor:=$EXEC \
       scenario:=$SCEN gui:=false rviz:=false headless_rendering:=true > /tmp/launch.log 2>&1 &
    python3 /scripts/gazebo_e2e_poll.py "$TIMEOUT" "$TASKS"; rc=$?
    echo "--- errors in launch log:"; grep -E "CONTACT|Traceback|process has died" /tmp/launch.log | head -10 || true
    exit $rc'
