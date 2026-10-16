#!/usr/bin/env bash
# Start the touchscreen panel (app 3) on the robot screen.
# Started after login by ~/.config/autostart/nala-panel.desktop (scripts/setup_pi_screen.sh).
# By hand:  bash ~/NALA/scripts/start_panel.sh
# Extra ROS arguments are passed on, e.g. -p fullscreen:=false -p hardware:=false (simulation on the PC).
set -eo pipefail   # no -u: the ROS setup scripts use unset variables

NALA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$NALA_DIR/config/ros_env.sh"
exec ros2 run nala_panel panel --ros-args --params-file "$NALA_DIR/config/panel.yaml" \
  -p repo_dir:="$NALA_DIR" "$@"
