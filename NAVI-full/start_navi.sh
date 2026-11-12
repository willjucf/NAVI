#!/bin/bash
# Starts NAVI on the LCD at login and restarts it if it crashes.
# SHUTDOWN on the screen, q, or pkill -f navi_app.py exits cleanly and ends
# this too. Output goes to output/navi.log (last boot in navi.prev.log).

cd "$(dirname "$0")"
export DISPLAY="${DISPLAY:-:0}"
mkdir -p output
LOG=output/navi.log
[ -f "$LOG" ] && mv -f "$LOG" output/navi.prev.log

# Keep the LCD on: no screensaver or power saving.
xset s off -dpms s noblank 2>/dev/null
# Hide the mouse pointer over the touchscreen, if unclutter is installed.
if command -v unclutter >/dev/null && ! pgrep -x unclutter >/dev/null; then
    unclutter -idle 1 &
fi

while true; do
    echo "=== start $(date '+%F %T') ===" >> "$LOG"
    python3 -u navi_app.py "$@" >> "$LOG" 2>&1
    code=$?
    [ $code -eq 0 ] && break
    echo "=== exited with $code, restarting in 3 s ===" >> "$LOG"
    sleep 3
done
