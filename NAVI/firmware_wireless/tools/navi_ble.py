"""Bluetooth terminal for the NAVI ESP32 (temporary wireless firmware).

Connects to NAVI-ESP32 over BLE (Nordic UART Service), prints every log line
the ESP sends, and sends what you type. Each line is one BLE write, so a
command like i2cscan or a packet like <0,0,1> goes through as is.

    .venv/bin/python navi_ble.py                  interactive, Ctrl-C to quit
    .venv/bin/python navi_ble.py --scan           list nearby BLE devices
    .venv/bin/python navi_ble.py --send i2cscan --seconds 8

Any valid <L,R,F> packet closes the motor relay, so keep motors disconnected
on the bench.
"""

import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

DEVICE_NAME = "NAVI-ESP32"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # we write here
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # ESP notifies here


async def scan(seconds):
    print(f"scanning {seconds} s...")
    found = await BleakScanner.discover(timeout=seconds, return_adv=True)
    for dev, adv in sorted(found.values(), key=lambda x: -x[1].rssi):
        mark = "  <-- NAVI" if (dev.name or adv.local_name) == DEVICE_NAME else ""
        print(f"{adv.rssi:4d} dBm  {dev.name or adv.local_name or '?':24s} {dev.address}{mark}")


def on_notify(_char, data: bytearray):
    sys.stdout.write(data.decode(errors="replace"))
    sys.stdout.flush()


async def run(sends, seconds):
    print(f"looking for {DEVICE_NAME}...")
    dev = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15)
    if dev is None:
        sys.exit(f"{DEVICE_NAME} not found. Is the MCU powered and booted (tap EN)? "
                 "Only one client can be connected at a time.")

    async with BleakClient(dev) as client:
        print(f"connected to {DEVICE_NAME} ({dev.address}), mtu {client.mtu_size}")
        await client.start_notify(NUS_TX, on_notify)

        async def send(line):
            await client.write_gatt_char(NUS_RX, line.encode(), response=True)
            print(f">>> {line}")

        if sends or seconds:
            for line in sends:
                await send(line)
            await asyncio.sleep(seconds or 5)
            return

        print("type a command (i2cscan) or a packet (<0,0,1>), Ctrl-C to quit")
        loop = asyncio.get_running_loop()
        while client.is_connected:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                break
            line = line.strip()
            if line:
                await send(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--scan", action="store_true", help="list nearby BLE devices")
    ap.add_argument("--send", action="append", default=[], metavar="LINE",
                    help="send LINE after connecting (repeatable)")
    ap.add_argument("--seconds", type=float, default=0,
                    help="listen this long, then exit (non interactive)")
    args = ap.parse_args()
    try:
        if args.scan:
            asyncio.run(scan(args.seconds or 6))
        else:
            asyncio.run(run(args.send, args.seconds))
    except KeyboardInterrupt:
        print("\ndisconnected")


if __name__ == "__main__":
    main()
