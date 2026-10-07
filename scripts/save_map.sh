#!/usr/bin/env bash
# Pi: save the current SLAM map into maps/<name>/ while app 2 is running.
# Run in a second Pi SSH terminal:  bash ~/NALA/scripts/save_map.sh <name>
# Files:
#   map.pgm, map.yaml          occupancy grid (Nav2 map server, base station picker on the PC)
#   map.posegraph, map.data    slam_toolbox pose graph (localization in stage 3)
# Then copy the folder to the PC (PC terminal, in the repo folder):
#   scp -r nala@nala.local:NALA/maps/<name> maps/
set -eo pipefail   # no -u: the ROS setup scripts use unset variables

name="${1:?Usage: save_map.sh <name>}"
NALA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dir="$NALA_DIR/maps/$name"

if [ -e "$dir" ]; then
  echo "$dir already exists. Use another name (saved maps are never overwritten)."
  exit 1
fi
source "$NALA_DIR/config/ros_env.sh"
mkdir -p "$dir"
trap 'echo "Saving failed. Is app 2 running?"; rm -rf "$dir"' ERR

echo "== Occupancy grid: map.pgm, map.yaml"
ros2 run nav2_map_server map_saver_cli -f "$dir/map"

echo "== slam_toolbox pose graph: map.posegraph, map.data"
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$dir/map'}"

for f in map.pgm map.yaml map.posegraph map.data; do
  [ -f "$dir/$f" ] || false   # missing file -> ERR trap
done
echo "Saved: $dir"
echo "Copy it to the PC (PC terminal, in the repo folder):"
echo "  scp -r nala@nala.local:NALA/maps/$name maps/"
