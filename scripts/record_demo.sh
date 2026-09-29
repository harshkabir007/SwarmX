#!/usr/bin/env bash
# Record a complete SwarmX Gazebo run: a top-down Gazebo video and a dashboard video.
#   ROBOTS=20 TASKS=40 OUT=~/Desktop/SwarmX_videos bash scripts/record_demo.sh
# Needs Docker (images swarmx-rec:jazzy, built from docker/Dockerfile.rec) and Chrome on the host.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROBOTS="${ROBOTS:-20}"; TASKS="${TASKS:-40}"; SCEN="${SCEN:-random}"
OUT="${OUT:-$HOME/Desktop/SwarmX_videos}"; TAG="swarmx_${ROBOTS}robots_${SCEN}"
FRAMES="$(mktemp -d)"
mkdir -p "$OUT"
docker image inspect swarmx-rec:jazzy >/dev/null 2>&1 || {
  docker image inspect swarmx-deps:jazzy >/dev/null 2>&1 || docker build -f "$ROOT/docker/Dockerfile.deps" -t swarmx-deps:jazzy "$ROOT/docker"
  docker build -f "$ROOT/docker/Dockerfile.rec" -t swarmx-rec:jazzy "$ROOT/docker"; }
GPU=(); [ -e /dev/dri ] && GPU=(--device /dev/dri --group-add "$(getent group video | cut -d: -f3)" --group-add "$(getent group render | cut -d: -f3)")

docker rm -f swarmx-record >/dev/null 2>&1 || true
docker run -d --name swarmx-record --network host --ipc=host "${GPU[@]}" \
  -e ROBOTS="$ROBOTS" -e TASKS="$TASKS" -e SCEN="$SCEN" -e TAG="$TAG" -e HUID="$(id -u)" -e HGID="$(id -g)" \
  -v "$ROOT/ros2_ws/src:/ws/src:ro" -v "$ROOT/scripts:/scripts:ro" -v "$OUT:/out" swarmx-rec:jazzy bash -c '
    source /opt/ros/jazzy/setup.bash
    cd /ws && colcon build > /tmp/build.log 2>&1 || { tail -30 /tmp/build.log; exit 1; }
    source install/setup.bash
    source install/swarmx_bringup/share/swarmx_bringup/config/zenoh/swarmx_zenoh.env
    ros2 launch swarmx_bringup sim_fleet.launch.py robots:=$ROBOTS tasks:=$TASKS scenario:=$SCEN localization:=static \
      executor:=direct gui:=false rviz:=false headless_rendering:=true spawn_delay:=1.5 > /tmp/launch.log 2>&1 &
    until gz service -l 2>/dev/null | grep -q "/world/swarmx_warehouse/create"; do sleep 2; done
    sleep 3
    python3 /scripts/record/gazebo_recorder.py --out /out/${TAG}_gazebo_topview.mp4 --tasks $TASKS --robots $ROBOTS; rc=$?
    grep -E "CONTACT|Traceback|process has died" /tmp/launch.log | head -10 || true
    chown $HUID:$HGID /out/${TAG}_gazebo_topview.mp4
    exit $rc' >/dev/null
echo "simulation + Gazebo recorder started (docker logs -f swarmx-record)"
python3 "$ROOT/scripts/record/dashboard_recorder.py" --frames "$FRAMES" --tasks "$TASKS" || true
docker wait swarmx-record >/dev/null || true
docker logs swarmx-record 2>&1 | tail -5
# dashboard video: same length as the Gazebo video so the two play side by side in sync
N=$(ls "$FRAMES" | wc -l)
GZ_DUR=$(docker run --rm -v "$OUT:/out" swarmx-rec:jazzy ffprobe -v error -show_entries format=duration -of csv=p=0 "/out/${TAG}_gazebo_topview.mp4" 2>/dev/null || echo 0)
FPS=$(python3 -c "d=float('${GZ_DUR:-0}' or 0); n=$N; print(max(2, round(n/d, 3)) if d > 1 else 10)")
docker run --rm -v "$FRAMES:/frames:ro" -v "$OUT:/out" swarmx-rec:jazzy bash -c "
  ffmpeg -y -loglevel error -framerate $FPS -i /frames/%05d.jpg -c:v libx264 -crf 20 -pix_fmt yuv420p \
    -vf 'scale=trunc(iw/2)*2:trunc(ih/2)*2' -movflags +faststart /out/${TAG}_dashboard.mp4 && chown $(id -u):$(id -g) /out/${TAG}_dashboard.mp4"
rm -rf "$FRAMES"
docker rm -f swarmx-record >/dev/null 2>&1 || true
ls -la "$OUT"
