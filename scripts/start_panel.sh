#!/usr/bin/env bash
# Start the touchscreen panel (app 3) on the robot screen.
# Started after login by ~/.config/autostart/nala-panel.desktop (scripts/setup_pi_screen.sh).
# By hand:  bash ~/NALA/scripts/start_panel.sh   (in a terminal on the Pi desktop, or over SSH: then it uses
#           the desktop of the logged-in user nala)
# Extra ROS arguments are passed on, e.g. -p fullscreen:=false -p hardware:=false (simulation on the PC).
set -eo pipefail   # no -u: the ROS setup scripts use unset variables

NALA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WAIT_S=10          # s, at login GNOME may still be starting XWayland

# The Pi desktop is GNOME on Wayland. PyQt5 runs on its XWayland (the xcb plugin comes with python3-pyqt5).
export QT_QPA_PLATFORM=xcb
# Inside the GNOME session DISPLAY, XAUTHORITY and XDG_RUNTIME_DIR are set and kept.
# Over SSH they are not: then the first local X display and the XWayland login file are used.
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"

x_socket() {   # the socket of DISPLAY (":1" or ":1.0" -> /tmp/.X11-unix/X1), or of the first local display
  if [ -n "${DISPLAY:-}" ]; then
    local number="${DISPLAY#:}"
    number="${number%%.*}"
    echo "/tmp/.X11-unix/X${number}"
    return 0
  fi
  local socket
  for socket in /tmp/.X11-unix/X*; do
    if [ -S "$socket" ]; then echo "$socket"; return 0; fi
  done
  return 0
}

xwayland_auth() {   # the XWayland login file of this user, empty if there is none yet
  local file
  for file in "$XDG_RUNTIME_DIR"/.mutter-Xwaylandauth.*; do
    if [ -f "$file" ]; then echo "$file"; return 0; fi
  done
  return 0
}

case "${DISPLAY:-:}" in
  :*) ;;
  *) echo "start_panel.sh: DISPLAY=$DISPLAY is not a local display (X forwarding?). Unset it to use the robot screen." >&2
     exit 1 ;;
esac

deadline=$((SECONDS + WAIT_S))
while :; do
  socket="$(x_socket)"
  auth="${XAUTHORITY:-$(xwayland_auth)}"
  if [ -z "$socket" ] || [ ! -S "$socket" ]; then
    problem="no X display socket (${socket:-/tmp/.X11-unix/X*})"
  elif [ -z "$auth" ]; then
    problem="no XWayland login file in $XDG_RUNTIME_DIR (is user nala logged in on the desktop?)"
  elif ! python3 -c 'import socket, sys; s = socket.socket(socket.AF_UNIX); s.settimeout(2.0); s.connect(sys.argv[1])' \
      "$socket" 2>/dev/null; then
    problem="cannot connect to $socket"
  else
    break
  fi
  if [ "$SECONDS" -ge "$deadline" ]; then
    echo "start_panel.sh: no display after ${WAIT_S} s: $problem" >&2
    exit 1
  fi
  sleep 0.5
done
export DISPLAY="${DISPLAY:-:${socket##*/X}}"
export XAUTHORITY="$auth"

source "$NALA_DIR/config/ros_env.sh"
exec ros2 run nala_panel panel --ros-args --params-file "$NALA_DIR/config/panel.yaml" \
  -p repo_dir:="$NALA_DIR" "$@"
