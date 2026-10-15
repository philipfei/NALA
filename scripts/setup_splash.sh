#!/usr/bin/env bash
# Boot splash of the Pi: the prayer picture (firmware/prayer.jpg) as a Plymouth theme. Safe to run again.
# Run in the Pi SSH terminal:  bash ~/NALA/scripts/setup_splash.sh
# It shows at the next boot, on a connected display (the Pi firmware's rainbow screen comes first).
# Package list: requirements.yaml (apt.pi). Keep both in sync.
set -euo pipefail

NALA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APT="sudo DEBIAN_FRONTEND=noninteractive NEEDRESTART_MODE=a apt-get -y -o DPkg::Lock::Timeout=600"
THEME_DIR=/usr/share/plymouth/themes/nala
CONFIG_TXT=/boot/firmware/config.txt
[ -f "$CONFIG_TXT" ] || CONFIG_TXT=/boot/config.txt
CMDLINE_TXT=/boot/firmware/cmdline.txt
[ -f "$CMDLINE_TXT" ] || CMDLINE_TXT=/boot/cmdline.txt

echo "== Packages"
# On Ubuntu the script plugin (script.so) is part of the package plymouth
# (there is no plymouth-plugins-script package, that is the Fedora name).
$APT install plymouth plymouth-label python3-pil
if ! ls /usr/lib/*/plymouth/script.so >/dev/null 2>&1; then
  echo "ERROR: the Plymouth script plugin (script.so) is missing; the theme would fall back to text." >&2
  exit 1
fi

echo "== Theme files"
sudo mkdir -p "$THEME_DIR"
# Plymouth reads only PNG. The picture is a JPEG: convert it, at most 1080 pixels high (smaller initramfs).
python3 - "$NALA_DIR/firmware/prayer.jpg" /tmp/nala_prayer.png <<'PY'
import sys
from PIL import Image
img = Image.open(sys.argv[1]).convert('RGB')
if img.height > 1080:
    img = img.resize((round(img.width * 1080 / img.height), 1080), Image.LANCZOS)
img.save(sys.argv[2])
PY
sudo install -m 644 /tmp/nala_prayer.png "$THEME_DIR/prayer.png"
sudo install -m 644 "$NALA_DIR/scripts/splash/nala.plymouth" "$NALA_DIR/scripts/splash/nala.script" "$THEME_DIR/"
rm -f /tmp/nala_prayer.png

echo "== Make it the default theme"
sudo update-alternatives --install /usr/share/plymouth/themes/default.plymouth default.plymouth \
  "$THEME_DIR/nala.plymouth" 200
sudo update-alternatives --set default.plymouth "$THEME_DIR/nala.plymouth"

echo "== Boot settings"
# The Pi bootloader loads the initramfs (where the theme lives) only with this line.
if ! grep -q '^initramfs initrd.img followkernel' "$CONFIG_TXT"; then
  echo "Adding 'initramfs initrd.img followkernel' to $CONFIG_TXT"
  sudo cp "$CONFIG_TXT" "$CONFIG_TXT.bak"
  printf '\n[all]\ninitramfs initrd.img followkernel\n' | sudo tee -a "$CONFIG_TXT" >/dev/null
fi
# Plymouth shows a picture only with 'splash' on the kernel command line.
if ! grep -qw splash "$CMDLINE_TXT"; then
  echo "Adding 'quiet splash' to $CMDLINE_TXT"
  sudo cp "$CMDLINE_TXT" "$CMDLINE_TXT.bak"
  sudo sed -i '1 s/$/ quiet splash/' "$CMDLINE_TXT"
fi

echo "== Rebuild the initramfs (about 1 minute; flash-kernel copies it to /boot/firmware)"
sudo update-initramfs -u

echo "== Check"
plymouth-set-default-theme
echo "Done. The splash shows at the next boot on a connected display."
