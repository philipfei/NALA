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
export CYCLONEDDS_URI="file://$NALA_DIR/config/cyclonedds.xml"
