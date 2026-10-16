# Source this (after colcon build) before run_sim.py:  source tools/sim/env.sh
# Simulation and the app 3 ROS tests; the robot and the PC use config/ros_env.sh (domain 0, robot network).
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NALA_ROOT="$(cd "$SIM_DIR/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$NALA_ROOT/install/setup.bash"
# Own domain, local only, so a simulation never talks to a real robot on the network.
export ROS_DOMAIN_ID="${SIM_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# Cyclone DDS like the robot (user decision 2026-10-16), on loopback only, with big messages and a big
# receive buffer (cyclonedds_sim.xml). Fast DDS was used before; the PC has only Cyclone installed.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$SIM_DIR/cyclonedds_sim.xml"
