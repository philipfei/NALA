#!/usr/bin/env bash
# Keep the robot screen on in GNOME: no blanking, no dimming, no lock screen, no suspend (app 3 panel).
# Runs at every login from ~/.config/autostart/nala-screen-on.desktop (scripts/setup_pi_screen.sh),
# inside the GNOME session: gsettings needs the session bus (it does not work over SSH).
set -euo pipefail

gsettings set org.gnome.desktop.session idle-delay 0
gsettings set org.gnome.desktop.screensaver idle-activation-enabled false
gsettings set org.gnome.desktop.screensaver lock-enabled false
gsettings set org.gnome.settings-daemon.plugins.power idle-dim false
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-ac-type 'nothing'
gsettings set org.gnome.settings-daemon.plugins.power sleep-inactive-battery-type 'nothing'
