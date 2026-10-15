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
$APT install ros-jazzy-ros-base python3-serial python3-pytest python3-colcon-common-extensions git avahi-daemon \
  openssh-server

echo "== Xbox controller (app 1)"
$APT install ros-jazzy-joy ros-jazzy-teleop-twist-joy
# joy reads /dev/input/event*, which belongs to the group input.
sudo usermod -aG input "$USER"

echo "== LiDAR, robot model, SLAM, map saver, RViz (app 2)"
$APT install ros-jazzy-rplidar-ros ros-jazzy-robot-state-publisher ros-jazzy-xacro \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-rviz2

echo "== IMU and EKF (app 2), I2C battery monitor"
$APT install ros-jazzy-robot-localization python3-smbus2 i2c-tools
# Firmware compile check only (the teammate builds and flashes with Microchip Studio).
$APT install gcc-avr avr-libc
# /dev/i2c-1 belongs to dialout on Ubuntu (user added below); if a group i2c exists, join it too.
if getent group i2c >/dev/null; then sudo usermod -aG i2c "$USER"; fi

echo "== USB serial adapters (LiDAR, IMU)"
# Ubuntu Desktop: brltty takes CH340 adapters away (no /dev/ttyUSB*), and ModemManager
# sends modem commands to new serial ports. Neither is needed on the robot.
$APT remove brltty
if systemctl list-unit-files ModemManager.service >/dev/null 2>&1; then
  sudo systemctl disable --now ModemManager.service || true
fi
# Fixed name /dev/ttyIMU for the CH340G adapter of the IMU (the only CH340 on the robot;
# the LiDAR adapter is a CP2102).
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyIMU", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-nala.rules >/dev/null
sudo udevadm control --reload
sudo udevadm trigger

echo "== UART on GPIO14/15 for the MCU (/dev/ttyAMA0)"
sudo usermod -aG dialout "$USER"
# The Linux serial console uses the GPIO UART. Remove it from the kernel command line
# and stop the login prompts on it.
sudo sed -i 's/console=serial0,115200 //' /boot/firmware/cmdline.txt
sudo systemctl mask serial-getty@ttyS0.service serial-getty@ttyAMA0.service
# The stable PL011 UART goes to GPIO14/15 (MCU), Bluetooth (Xbox controller) moves to the
# mini UART. core_freq + core_freq_min fix the VPU clock, so the mini UART baud rate does not
# change with the CPU load. Fixed at 500 MHz, not 250: on the Pi 4 the display driver (vc4 KMS)
# needs a core clock of at least 500 MHz; with 250 it logged kernel warnings ~9 times per second
# and filled the SD card (2026-10-15). I2C bus 1 on GPIO2/3 is for the INA260 battery monitor.
config=/boot/firmware/config.txt
sudo sed -i '/^core_freq=250$/d' "$config"
for setting in enable_uart=1 dtoverlay=miniuart-bt core_freq=500 core_freq_min=500 dtparam=i2c_arm=on; do
  grep -qxF "$setting" "$config" || echo "$setting" | sudo tee -a "$config" >/dev/null
done

echo "== ROS environment in ~/.bashrc"
line="source $NALA_DIR/config/ros_env.sh"
grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc

echo "Done. Reboot now: sudo reboot"
