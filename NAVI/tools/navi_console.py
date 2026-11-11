"""Readable live console for the NAVI ESP32.

The ESP prints a status line twice a second, which scrolls everything else
away. This viewer keeps the latest status in a fixed panel at the top, scrolls
only the events below it (halts, errors, connects, command output), collapses
repeats, and keeps a command line at the bottom that output never overwrites.

It only changes what you see. The ESP's output is untouched, nothing is sent
unless you type it, and every raw line is saved to output/console_<time>.log.

    python3 tools/navi_console.py                    serial (the CP2102, or --port)
    python3 tools/navi_console.py --ip 172.20.10.4   WiFi
    python3 tools/navi_console.py --ble              Bluetooth (needs bleak)

Type a line and press Enter to send it, such as i2cscan or <0,0,1>.
Console commands: /pause  /raw (show every line)  /clear  /quit
PgUp / PgDn scroll the events.
"""

import argparse
import collections
import curses
import datetime
import glob
import os
import queue
import re
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
NAVI_DIR = os.path.dirname(HERE)
sys.path.insert(0, NAVI_DIR)

ANSI = re.compile(r"\x1b\[[0-9;]*m")
STATUS = re.compile(r"\bnavi: state=")
HINT = re.compile(r"\bwireless: wifi (at|down)")
STAMP = re.compile(r"^([IWE]) \((\d+)\) ")
FIELD = re.compile(r"(\w+)=(\S+)")
MAX_EVENTS = 2000


# Transports. Each reads lines into a queue on its own thread and can send a
# line. They reconnect by themselves, so a reboot or a dropped link just shows
# as disconnected until it comes back.

class Transport:
    def __init__(self):
        self.lines = queue.Queue()
        self.connected = False
        self.stop = threading.Event()

    def _feed(self, raw, buf):
        buf += raw
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            self.lines.put(line.decode(errors="replace"))
        return buf

    def close(self):
        self.stop.set()


class SerialTransport(Transport):
    def __init__(self, port, baud):
        super().__init__()
        self.port, self.baud = port, baud
        self.link = None
        threading.Thread(target=self._run, daemon=True).start()

    def describe(self):
        return "serial %s" % self.port

    def _run(self):
        from esp_serial import open_esp_serial
        buf = b""
        while not self.stop.is_set():
            try:
                # reopens until the CP2102 reads clean, never resets the ESP
                link = open_esp_serial(self.port, self.baud, timeout=0.1,
                                       log=lambda m: self.lines.put(m))
                self.link, self.connected = link, True
                while not self.stop.is_set():
                    buf = self._feed(link.read(512), buf)
            except Exception:
                pass
            self.connected = False
            self.link = None
            time.sleep(1)

    def send(self, text):
        if self.link:
            self.link.write(text.encode())


class TcpTransport(Transport):
    def __init__(self, ip, port):
        super().__init__()
        self.ip, self.port = ip, port
        self.sock = None
        threading.Thread(target=self._run, daemon=True).start()

    def describe(self):
        return "WiFi %s:%d" % (self.ip, self.port)

    def _run(self):
        buf = b""
        while not self.stop.is_set():
            try:
                sock = socket.create_connection((self.ip, self.port), timeout=5)
                sock.settimeout(1)
                self.sock, self.connected = sock, True
                while not self.stop.is_set():
                    try:
                        data = sock.recv(2048)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    buf = self._feed(data, buf)
            except OSError:
                pass
            self.connected = False
            self.sock = None
            time.sleep(2)

    def send(self, text):
        if self.sock:
            self.sock.sendall(text.encode())


class BleTransport(Transport):
    NAME = "NAVI-ESP32"
    RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
    TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

    def __init__(self):
        super().__init__()
        import asyncio
        self.asyncio = asyncio
        self.loop = asyncio.new_event_loop()
        self.client = None
        threading.Thread(target=self.loop.run_until_complete, args=(self._run(),),
                         daemon=True).start()

    def describe(self):
        return "Bluetooth %s" % self.NAME

    async def _run(self):
        from bleak import BleakClient, BleakScanner
        buf = [b""]

        def on_notify(_char, data):
            buf[0] = self._feed(bytes(data), buf[0])

        while not self.stop.is_set():
            try:
                dev = await BleakScanner.find_device_by_name(self.NAME, timeout=10)
                if dev is not None:
                    async with BleakClient(dev) as client:
                        await client.start_notify(self.TX, on_notify)
                        self.client, self.connected = client, True
                        while client.is_connected and not self.stop.is_set():
                            await self.asyncio.sleep(0.2)
            except Exception:
                pass
            self.connected = False
            self.client = None
            await self.asyncio.sleep(2)

    def send(self, text):
        if self.client:
            self.asyncio.run_coroutine_threadsafe(
                self.client.write_gatt_char(self.RX, text.encode(), response=True), self.loop)


# What is shown

class View:
    def __init__(self, log_path):
        self.status = {}
        self.status_at = None
        self.hint = ""
        self.events = collections.deque(maxlen=MAX_EVENTS)   # [text, count, level]
        self.raw = False
        self.paused = False
        self.scroll = 0            # lines back from the newest
        self.new_while_paused = 0
        self.log = open(log_path, "a", buffering=1)
        self.log_path = log_path

    def add(self, line):
        line = ANSI.sub("", line).rstrip("\r")
        if not line.strip():
            return
        self.log.write("%s %s\n" % (datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3], line))

        if STATUS.search(line):
            self.status = dict(FIELD.findall(line.split("navi: ", 1)[1]))
            self.status_at = time.time()
            if not self.raw:
                return
        elif HINT.search(line):
            self.hint = line.split("wireless: ", 1)[1]
            if not self.raw:
                return

        m = STAMP.match(line)
        level = m.group(1) if m else "I"
        body = line[m.end():] if m else line
        if "HALTED" in body:
            level = "E"
        if self.events and self.events[-1][0] == body:
            self.events[-1][1] += 1                        # collapse repeats
        else:
            self.events.append([body, 1, level])
            if self.paused or self.scroll:
                self.new_while_paused += 1
                if self.scroll:
                    self.scroll += 1                       # keep the view still


def draw(scr, view, transport, typed):
    h, w = scr.getmaxyx()
    scr.erase()

    def put(y, x, text, attr=0):
        if 0 <= y < h and x < w:
            scr.addnstr(y, x, text, max(0, w - x - 1), attr)

    link = "connected" if transport.connected else "waiting..."
    put(0, 0, (" NAVI console | %s | %s | log %s "
               % (transport.describe(), link, os.path.basename(view.log_path))).ljust(w),
        curses.A_REVERSE)

    s = view.status
    if s:
        age = time.time() - view.status_at
        state = s.get("state", "?")
        state_attr = curses.color_pair(3) if state == "NONE" else curses.color_pair(1)
        put(1, 1, "State ", curses.A_BOLD)
        put(1, 7, "%-14s" % ("RUNNING" if state == "NONE" else state), state_attr | curses.A_BOLD)
        put(1, 23, "PWM %-9s target %-9s stop flag %s"
            % (s.get("pwm", "?"), s.get("target", "?"), s.get("flag", "?")))
        put(2, 1, "USS %-14s tilt %-18s frames good/bad %s"
            % (s.get("uss", "?"), s.get("tilt", "?"), s.get("frames", "?")))
        put(2, w - 22, "updated %4.1fs ago" % age,
            curses.color_pair(1) if age > 2 else curses.A_DIM)
    else:
        put(1, 1, "waiting for the first status line...", curses.A_DIM)
    put(3, 1, ("Link: " + view.hint) if view.hint else "", curses.A_DIM)

    flags = []
    if view.raw:
        flags.append("RAW: all lines")
    if view.paused:
        flags.append("PAUSED")
    if view.scroll:
        flags.append("scrolled back %d" % view.scroll)
    if view.new_while_paused:
        flags.append("+%d new" % view.new_while_paused)
    put(4, 0, ("-- events %s" % ("[" + ", ".join(flags) + "] " if flags else "")).ljust(w, "-"),
        curses.A_DIM)

    rows = h - 7
    if rows > 0:
        events = list(view.events)
        if view.paused and view.scroll == 0:
            events = events[:len(events) - view.new_while_paused] if view.new_while_paused else events
        end = len(events) - view.scroll
        shown = events[max(0, end - rows):max(0, end)]
        for i, (body, count, level) in enumerate(shown):
            attr = {"E": curses.color_pair(1), "W": curses.color_pair(2)}.get(level, 0)
            text = body + ("  (x%d)" % count if count > 1 else "")
            put(5 + i, 1, text, attr)

    put(h - 2, 0, " Enter: send   /pause  /raw  /clear  /quit   PgUp/PgDn: scroll".ljust(w),
        curses.A_REVERSE)
    put(h - 1, 0, "> " + typed)
    scr.move(h - 1, min(w - 1, 2 + len(typed)))
    scr.refresh()


def run(scr, transport, view):
    curses.curs_set(1)
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_RED, -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_GREEN, -1)
    scr.timeout(100)
    typed = ""
    last_draw = 0
    while True:
        while True:
            try:
                view.add(transport.lines.get_nowait())
            except queue.Empty:
                break

        key = scr.getch()
        h = scr.getmaxyx()[0]
        if key in (curses.KEY_ENTER, 10, 13):
            cmd, typed = typed.strip(), ""
            if cmd in ("/quit", "/q"):
                return
            elif cmd == "/pause":
                view.paused = not view.paused
                view.new_while_paused = 0
            elif cmd == "/raw":
                view.raw = not view.raw
            elif cmd == "/clear":
                view.events.clear()
                view.scroll = view.new_while_paused = 0
            elif cmd:
                transport.send(cmd + "\n")
                view.events.append([">>> " + cmd, 1, "W"])
        elif key in (curses.KEY_BACKSPACE, 127, 8):
            typed = typed[:-1]
        elif key == curses.KEY_PPAGE:
            view.scroll = min(len(view.events), view.scroll + max(1, h - 8))
        elif key == curses.KEY_NPAGE:
            view.scroll = max(0, view.scroll - max(1, h - 8))
            if view.scroll == 0 and not view.paused:
                view.new_while_paused = 0
        elif 32 <= key < 127:
            typed += chr(key)

        now = time.time()
        if key != -1 or now - last_draw > 0.1:
            draw(scr, view, transport, typed)
            last_draw = now


def default_port():
    try:
        import navi_config
        port = navi_config.SERIAL_PORT
    except Exception:
        port = "/dev/ttyUSB0"
    if not os.path.exists(port):
        mac = sorted(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.SLAB*"))
        if mac:
            port = mac[0]
    return port


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    how = ap.add_mutually_exclusive_group()
    how.add_argument("--port", help="serial port (default: the CP2102)")
    how.add_argument("--ip", help="ESP's WiFi address")
    how.add_argument("--ble", action="store_true", help="connect over Bluetooth")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tcp-port", type=int, default=3333)
    args = ap.parse_args()

    if args.ip:
        transport = TcpTransport(args.ip, args.tcp_port)
    elif args.ble:
        transport = BleTransport()
    else:
        transport = SerialTransport(args.port or default_port(), args.baud)

    out = os.path.join(NAVI_DIR, "output")
    os.makedirs(out, exist_ok=True)
    view = View(os.path.join(out, "console_%s.log" % datetime.datetime.now().strftime("%Y%m%d-%H%M%S")))
    try:
        curses.wrapper(run, transport, view)
    except KeyboardInterrupt:
        pass
    finally:
        transport.close()
        print("Full log: %s" % view.log_path)


if __name__ == "__main__":
    main()
