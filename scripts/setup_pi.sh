#!/usr/bin/env bash
# One-time setup of the Raspberry Pi (Ubuntu 24.04, user nala). Safe to run again.
# Run in the Pi SSH terminal:  bash ~/NALA/scripts/setup_pi.sh
# Then reboot:                 sudo reboot
# Package list: requirements.yaml (apt.pi). Keep both in sync.
set -euo pipefail

ROS_APT_SOURCE_VERSION=1.3.0   # same version as on the PC (the Pi follows the PC)
NALA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APT="sudo DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get -y -o DPkg::Lock::Timeout=600"

echo "== Update the system"
# The Pi image has no noble-updates suite, but ships packages from it (e.g. libacl1).
# Without it, ROS dependencies cannot be installed. Use the same suites as the PC.
sudo sed -i 's/^Suites: noble$/Suites: noble noble-updates noble-backports/' /etc/apt/sources.list.d/ubuntu.sources
$APT update
$APT upgrade

echo "== ROS 2 apt source"
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  codename="$(. /etc/os-release && echo "$VERSION_CODENAME")"
  deb="/tmp/ros2-apt-source.deb"
  curl -fsSL -o "$deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${codename}_all.deb"
  $APT install "$deb"
  $APT update
fi

echo "== ROS 2 Jazzy and tools"
# Cyclone DDS first: then ros-base uses it and does not pull in Fast DDS (same as the PC).
$APT install ros-jazzy-rmw-cyclonedds-cpp
$APT install ros-jazzy-ros-base python3-serial python3-pytest python3-colcon-common-extensions git avahi-daemon

echo "== Xbox controller (app 1)"
$APT install ros-jazzy-joy ros-jazzy-teleop-twist-joy
# joy reads /dev/input/event*, which belongs to the group input.
sudo usermod -aG input "$USER"

echo "== LiDAR, robot model, SLAM, map saver (app 2)"
$APT install ros-jazzy-rplidar-ros ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server

echo "== UART to the MCU (/dev/ttyS0)"
sudo usermod -aG dialout "$USER"
# The Linux serial console uses the same UART. Remove it from the kernel command line
# and stop the login prompt on it.
sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt
sudo systemctl mask serial-getty@ttyS0.service

echo "== ROS environment in ~/.bashrc"
line="source $NALA_DIR/config/ros_env.sh"
grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc

echo "Done. Reboot now: sudo reboot"
