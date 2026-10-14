#!/usr/bin/env bash
# One-time setup of the robot screen for app 3 (touchscreen panel) on Ubuntu Desktop 24.04. Safe to run again.
# Run after scripts/setup_pi.sh, in a terminal on the Pi (or over SSH):  bash ~/NALA/scripts/setup_pi_screen.sh
# Then reboot:                                                          sudo reboot
# Screen: Waveshare 7 inch HDMI LCD (C), 1024 x 600, touch over USB (works without a driver).
# Package list: requirements.yaml (apt.pi.stage_3). Keep both in sync.
set -euo pipefail

NALA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APT="sudo DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get -y -o DPkg::Lock::Timeout=600"

echo "== App 3: Nav2, the panel and the path planner"
$APT install ros-jazzy-navigation2 python3-pyqt5 python3-matplotlib python3-scipy python3-pil \
  python3-numpy python3-yaml

echo "== Log in $USER automatically (GNOME, GDM)"
sudo mkdir -p /etc/gdm3
if ! sudo grep -q '^AutomaticLoginEnable *= *true' /etc/gdm3/custom.conf 2>/dev/null; then
  sudo sed -i '/^\[daemon\]/a AutomaticLoginEnable=true\nAutomaticLogin='"$USER" /etc/gdm3/custom.conf
fi

echo "== Keep the screen on, no lock screen"
# gsettings needs the desktop session's D-Bus; over SSH it is reached with dbus-launch-free 'sudo -u' tricks,
# so these settings are also written as an autostart command that runs at every login.
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/nala-screen-on.desktop <<DESKTOP
[Desktop Entry]
Type=Application
Name=NALA screen always on
Exec=sh -c 'gsettings set org.gnome.desktop.session idle-delay 0; gsettings set org.gnome.desktop.screensaver lock-enabled false; gsettings set org.gnome.settings-daemon.plugins.power idle-dim false'
DESKTOP

echo "== Start the panel after login"
cat > ~/.config/autostart/nala-panel.desktop <<DESKTOP
[Desktop Entry]
Type=Application
Name=NALA panel
Exec=bash $NALA_DIR/scripts/start_panel.sh
DESKTOP

echo "Done. Build the workspace if needed (colcon build --symlink-install --base-paths src), then: sudo reboot"
