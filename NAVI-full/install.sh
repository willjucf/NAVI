#!/bin/bash
# Run once on the Pi: starts NAVI on the LCD every time the Pi boots.
# Undo with: rm ~/.config/autostart/navi.desktop
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
chmod +x "$DIR/start_navi.sh" "$DIR/navi_app.py"
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/navi.desktop <<DESKTOP
[Desktop Entry]
Type=Application
Name=NAVI
Exec=$DIR/start_navi.sh
Terminal=false
X-GNOME-Autostart-enabled=true
DESKTOP
echo "NAVI will start at boot: ~/.config/autostart/navi.desktop -> $DIR/start_navi.sh"
echo "The Pi must boot to the desktop with autologin: sudo raspi-config,"
echo "System Options > Boot / Auto Login > Desktop Autologin."
