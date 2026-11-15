#!/bin/bash
# Run once on the Pi. Puts a NAVI icon on the desktop and starts NAVI on the
# LCD every time the Pi boots. Both run the full system (ESP32, ToF, camera).
#
#   ./install.sh                 desktop icon + start at boot
#   ./install.sh --no-autostart  desktop icon only
#
# Undo with: rm ~/Desktop/NAVI.desktop ~/.config/autostart/navi.desktop
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
chmod +x "$DIR/start_navi.sh" "$DIR/navi_app.py"

entry() {
    cat <<DESKTOP
[Desktop Entry]
Type=Application
Name=NAVI
Comment=NAVI assisted driving
Exec=$DIR/start_navi.sh
Icon=$DIR/navi_icon.svg
Terminal=false
DESKTOP
}

mkdir -p ~/Desktop
entry > ~/Desktop/NAVI.desktop
chmod +x ~/Desktop/NAVI.desktop
# Launch desktop icons without the "execute or open?" prompt.
LIBFM=~/.config/libfm/libfm.conf
if [ -f "$LIBFM" ] && grep -q '^quick_exec=0' "$LIBFM"; then
    sed -i 's/^quick_exec=0/quick_exec=1/' "$LIBFM"
    pkill -9 -x pcmanfm || true   # lwrespawn restarts it with the new setting
fi
echo "Desktop icon: ~/Desktop/NAVI.desktop (double tap to start NAVI)"

if [ "$1" != "--no-autostart" ]; then
    mkdir -p ~/.config/autostart
    { entry; echo "X-GNOME-Autostart-enabled=true"; } > ~/.config/autostart/navi.desktop
    echo "NAVI will start at boot: ~/.config/autostart/navi.desktop -> $DIR/start_navi.sh"
    echo "The Pi must boot to the desktop with autologin: sudo raspi-config,"
    echo "System Options > Boot / Auto Login > Desktop Autologin."
fi
