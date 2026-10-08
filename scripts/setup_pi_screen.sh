#!/usr/bin/env bash
# One-time setup of the robot screen for app 3 (touchscreen panel). Safe to run again.
# Run after scripts/setup_pi.sh, in the Pi SSH terminal:  bash ~/NALA/scripts/setup_pi_screen.sh
# Then reboot:                                            sudo reboot
# Screen: Waveshare 7 inch HDMI LCD (C), 1024 x 600, touch over USB (works without a driver).
# Package list: requirements.yaml (apt.pi.stage_3). Keep both in sync.
set -euo pipefail

NALA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APT="sudo DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get -y -o DPkg::Lock::Timeout=600"

echo "== Small desktop (Xfce) with automatic login of $USER"
$APT install --no-install-recommends xserver-xorg xinit xfce4 lightdm lightdm-gtk-greeter x11-xserver-utils
sudo mkdir -p /etc/lightdm/lightdm.conf.d
printf '[Seat:*]\nautologin-user=%s\nautologin-session=xfce\nuser-session=xfce\n' "$USER" |
  sudo tee /etc/lightdm/lightdm.conf.d/50-nala-autologin.conf >/dev/null
sudo systemctl set-default graphical.target

echo "== App 3: Nav2, the panel and the path planner"
$APT install ros-jazzy-navigation2 python3-pyqt5 python3-matplotlib python3-scipy python3-pil \
  python3-numpy python3-yaml

echo "== Start the panel after login and keep the screen on"
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/nala-panel.desktop <<DESKTOP
[Desktop Entry]
Type=Application
Name=NALA panel
Exec=bash $NALA_DIR/scripts/start_panel.sh
DESKTOP
cat > ~/.config/autostart/nala-screen-on.desktop <<DESKTOP
[Desktop Entry]
Type=Application
Name=NALA screen always on
Exec=xset s off -dpms
DESKTOP

echo "Done. Build the workspace if needed (colcon build --symlink-install --base-paths src), then: sudo reboot"
