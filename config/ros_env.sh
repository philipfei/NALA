# ROS environment for NALA. Source it on the PC and on the Pi (same settings on both):
#   source <repo>/config/ros_env.sh
# The Pi does this in ~/.bashrc (scripts/setup_pi.sh). The PC does it by hand in each terminal.

NALA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [ -f "$NALA_DIR/install/setup.bash" ]; then
  source "$NALA_DIR/install/setup.bash"
fi

export ROS_DOMAIN_ID=0                          # must be the same on PC and Pi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp    # the PC has only Cyclone DDS installed
# A robot network from cyclonedds.xml (its /24 networks) is up: use it. Without one (no WiFi at boot), the
# loopback config, so the robot screen and the apps still run on the Pi (user, 2026-10-20; without any listed
# network Cyclone could not create a node and the panel did not start).
nala_robot_network() {
  local net
  for net in $(sed -n 's/.*NetworkInterface address="\([0-9]*\.[0-9]*\.[0-9]*\)\.0".*/\1/p' "$NALA_DIR/config/cyclonedds.xml"); do
    ip -4 -o addr show 2>/dev/null | grep -q " inet ${net//./\\.}\." && return 0
  done
  return 1
}
if [ -n "${NALA_CYCLONEDDS_URI:-}" ]; then
  # Fixed by start_panel.sh: everything the panel starts (the apps, save_map.sh) uses the panel's choice, also
  # when the WiFi comes up later.
  export CYCLONEDDS_URI="$NALA_CYCLONEDDS_URI"
elif nala_robot_network; then
  export CYCLONEDDS_URI="file://$NALA_DIR/config/cyclonedds.xml"
else
  export CYCLONEDDS_URI="file://$NALA_DIR/config/cyclonedds_local.xml"
  echo "ros_env.sh: no robot network (config/cyclonedds.xml): ROS on this computer only (loopback)" >&2
fi
