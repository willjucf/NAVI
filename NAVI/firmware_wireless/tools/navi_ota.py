"""Send new firmware to the NAVI ESP32 over WiFi (needs the OTA-capable build).

    python3 navi_ota.py build/navi_firmware.bin [--ip 172.20.10.4]

Sends the app image to port 3334. The ESP writes it to its spare slot, checks
it, and reboots into it. If anything goes wrong it keeps running the old
firmware. The reboot drops the motor relay, like any reset.
"""

import argparse
import socket
import struct
import sys

ap = argparse.ArgumentParser()
ap.add_argument("image", help="app .bin (build/navi_firmware.bin)")
ap.add_argument("--ip", default="172.20.10.4")
ap.add_argument("--port", type=int, default=3334)
args = ap.parse_args()

data = open(args.image, "rb").read()
if data[:1] != b"\xe9":
    sys.exit("not an ESP32 app image (first byte should be 0xE9)")

print(f"sending {len(data)} bytes to {args.ip}:{args.port}...")
with socket.create_connection((args.ip, args.port), timeout=10) as s:
    s.settimeout(60)
    s.sendall(struct.pack("<I", len(data)))
    sent = 0
    for off in range(0, len(data), 4096):
        s.sendall(data[off:off + 4096])
        sent = min(off + 4096, len(data))
        print(f"\r{sent * 100 // len(data):3d}%", end="", flush=True)
    print()
    reply = s.recv(128).decode(errors="replace").strip()
print(reply or "no reply (connection closed)")
sys.exit(0 if reply.startswith("OK") else 1)
