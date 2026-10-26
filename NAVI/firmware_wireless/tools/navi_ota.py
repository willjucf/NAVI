"""Send new firmware to the NAVI ESP32 over WiFi or Bluetooth.

    python3 navi_ota.py build/navi_firmware.bin --ip 172.20.10.4     WiFi (fast)
    .venv/bin/python navi_ota.py build/navi_firmware.bin --ble        Bluetooth

The ESP writes the image to its spare slot, checks it and reboots into it. If
anything goes wrong it keeps running the old firmware. The reboot drops the
motor relay, like any reset. Bluetooth needs bleak (in tools/.venv) and takes a
few minutes; WiFi takes seconds.
"""

import argparse
import asyncio
import socket
import struct
import sys
import time

BLE_NAME = "NAVI-ESP32"
OTA_CTRL = "d0f20002-8a4e-4b5a-9c1e-4e4156494f54"
OTA_DATA = "d0f20003-8a4e-4b5a-9c1e-4e4156494f54"

# ATT error codes the firmware returns (wireless.cpp).
BLE_ERRORS = {
    0x80: "another update is already running",
    0x81: "image size is 0 or too big for the slot",
    0x82: "the ESP could not start the update",
    0x83: "flash write failed",
    0x84: "the ESP rejected the image (corrupt or wrong chip)",
    0x85: "update not started, or END before all the data",
}


def progress(done, total, started):
    rate = done / max(time.time() - started, 0.001) / 1024
    print("\r%3d%%  %d / %d bytes  %.1f KB/s" % (done * 100 // total, done, total, rate),
          end="", flush=True)


def send_wifi(data, ip, port):
    print("WiFi: sending %d bytes to %s:%d" % (len(data), ip, port))
    started = time.time()
    with socket.create_connection((ip, port), timeout=10) as s:
        s.settimeout(60)
        s.sendall(struct.pack("<I", len(data)))
        for off in range(0, len(data), 4096):
            s.sendall(data[off:off + 4096])
            progress(min(off + 4096, len(data)), len(data), started)
        print()
        reply = s.recv(128).decode(errors="replace").strip()
    print(reply or "no reply (connection closed)")
    return reply.startswith("OK")


def ble_error(exc):
    text = str(exc)
    for code, why in BLE_ERRORS.items():
        if "0x%02x" % code in text.lower() or "(%d)" % code in text:
            return why
    return text


async def send_ble(data):
    from bleak import BleakClient, BleakScanner

    print("Bluetooth: looking for %s..." % BLE_NAME)
    dev = await BleakScanner.find_device_by_name(BLE_NAME, timeout=15)
    if dev is None:
        print("%s not found. Is it powered, and is nothing else connected to it?" % BLE_NAME)
        return False
    async with BleakClient(dev) as client:
        chunk = max(20, min(client.mtu_size - 3, 509))
        print("connected, MTU %d, %d-byte chunks, %d bytes to send"
              % (client.mtu_size, chunk, len(data)))
        try:
            await client.write_gatt_char(OTA_CTRL, b"BEGIN %d" % len(data), response=True)
            started = time.time()
            for off in range(0, len(data), chunk):
                await client.write_gatt_char(OTA_DATA, data[off:off + chunk], response=True)
                if off // chunk % 20 == 0 or off + chunk >= len(data):
                    progress(min(off + chunk, len(data)), len(data), started)
            print()
            await client.write_gatt_char(OTA_CTRL, b"END", response=True)
        except Exception as exc:
            print("\nFAILED: %s" % ble_error(exc))
            try:
                await client.write_gatt_char(OTA_CTRL, b"ABORT", response=True)
            except Exception:
                pass
            return False
    print("OK, the ESP is rebooting into the new firmware")
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("image", help="app .bin (firmware_wireless/build/navi_firmware.bin)")
    how = ap.add_mutually_exclusive_group()
    how.add_argument("--ip", help="ESP's WiFi address (it logs it every 10 s)")
    how.add_argument("--ble", action="store_true", help="send over Bluetooth instead")
    ap.add_argument("--port", type=int, default=3334)
    args = ap.parse_args()

    data = open(args.image, "rb").read()
    if data[:1] != b"\xe9":
        sys.exit("not an ESP32 app image (first byte should be 0xE9)")

    if args.ble:
        ok = asyncio.run(send_ble(data))
    else:
        ok = send_wifi(data, args.ip or "172.20.10.4", args.port)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
