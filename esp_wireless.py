"""WiFi and Bluetooth links to the ESP32 for bench testing.

The firmware takes the same <L,R,F> packets on WiFi (TCP 3333) and BLE (Nordic UART
Service) as on its UART, through the same watchdog and safety gates. Each link here
behaves like the serial port in navi_control.py: write(), read() and close(), and it
reconnects by itself. The ESP32 takes one client per link.
"""

import concurrent.futures
import socket
import threading
import time

TCP_PORT = 3333
BLE_NAME = "NAVI-ESP32"
NUS_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"   # packets go here
NUS_TX = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"   # log comes back here


class _Link:
    def __init__(self):
        self.connected = False
        self._closed = threading.Event()
        self._rx = bytearray()
        self._rx_lock = threading.Lock()

    def _received(self, data):
        with self._rx_lock:
            self._rx += data
            if len(self._rx) > 65536:          # nobody reading, keep the tail
                del self._rx[:-65536]

    def read(self, n=256):
        with self._rx_lock:
            out = bytes(self._rx[:n])
            del self._rx[:n]
        return out

    def close(self):
        self._closed.set()


def find_esp_ip(port=TCP_PORT, timeout=0.4):
    """Find the ESP32 on the local /24 by its open log port."""
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("10.255.255.255", 1))
        mine = probe.getsockname()[0]
    finally:
        probe.close()
    base = mine.rsplit(".", 1)[0]

    def try_host(i):
        ip = "%s.%d" % (base, i)
        if ip == mine:
            return None
        try:
            with socket.create_connection((ip, port), timeout=timeout):
                return ip
        except OSError:
            return None

    with concurrent.futures.ThreadPoolExecutor(max_workers=64) as pool:
        for ip in pool.map(try_host, range(1, 255)):
            if ip:
                return ip
    return None


class WifiLink(_Link):
    """Packets over TCP to the ESP's log port."""

    def __init__(self, ip=None, port=TCP_PORT):
        super().__init__()
        self.ip = ip or find_esp_ip(port)
        if self.ip is None:
            raise RuntimeError("no ESP32 found on port %d on this network. Is it "
                               "powered and on the same hotspot? It takes one WiFi "
                               "client at a time, so close any console using "
                               "--ip (use --ble for the console instead)." % port)
        self.port = port
        self._sock = None
        threading.Thread(target=self._run, daemon=True, name="esp-wifi").start()

    def describe(self):
        return "WiFi %s:%d" % (self.ip, self.port)

    def _run(self):
        while not self._closed.is_set():
            try:
                sock = socket.create_connection((self.ip, self.port), timeout=3)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(0.5)
                self._sock, self.connected = sock, True
                while not self._closed.is_set():
                    try:
                        data = sock.recv(2048)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    self._received(data)
            except OSError:
                pass
            self.connected = False
            self._sock = None
            self._closed.wait(1.0)

    def write(self, data):
        sock = self._sock
        if sock is None:
            raise ConnectionError("WiFi link to the ESP32 is down")
        sock.sendall(data)

    def close(self):
        super().close()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass


class BleLink(_Link):
    """Packets over the ESP's Bluetooth UART service."""

    def __init__(self, name=BLE_NAME):
        super().__init__()
        import asyncio
        try:
            import bleak  # noqa: F401
        except ImportError as exc:
            raise RuntimeError("bleak is not installed. On the Pi: "
                               "sudo apt install python3-bleak") from exc
        self.name = name
        self._asyncio = asyncio
        self._loop = asyncio.new_event_loop()
        self._client = None
        self._pending = None
        threading.Thread(target=self._loop.run_until_complete, args=(self._run(),),
                         daemon=True, name="esp-ble").start()

    def describe(self):
        return "Bluetooth %s" % self.name

    async def _run(self):
        from bleak import BleakClient, BleakScanner
        while not self._closed.is_set():
            try:
                dev = await BleakScanner.find_device_by_name(self.name, timeout=10)
                if dev is not None:
                    async with BleakClient(dev) as client:
                        await client.start_notify(NUS_TX, lambda _c, d: self._received(bytes(d)))
                        self._client, self.connected = client, True
                        while client.is_connected and not self._closed.is_set():
                            await self._asyncio.sleep(0.2)
            except Exception:
                pass
            self.connected = False
            self._client = None
            await self._asyncio.sleep(1.0)

    def write(self, data):
        client = self._client
        if client is None:
            raise ConnectionError("Bluetooth link to the ESP32 is down")
        # Never queue behind a slow write. The next packet is 50 ms away and
        # is newer anyway, so a busy link drops this one.
        if self._pending is not None and not self._pending.done():
            raise ConnectionError("Bluetooth link busy, packet dropped")
        self._pending = self._asyncio.run_coroutine_threadsafe(
            client.write_gatt_char(NUS_RX, data, response=False), self._loop)


class BothLinks:
    """Same packet on WiFi and Bluetooth, so either one dropping is covered."""

    def __init__(self, ip=None):
        self.links = [WifiLink(ip), BleLink()]

    def describe(self):
        return " + ".join(link.describe() for link in self.links)

    @property
    def connected(self):
        return any(link.connected for link in self.links)

    def write(self, data):
        sent = 0
        for link in self.links:
            try:
                link.write(data)
                sent += 1
            except Exception:
                pass
        if not sent:
            raise ConnectionError("both wireless links to the ESP32 are down")

    def read(self, n=256):
        # Both carry the same log. Read the WiFi copy, drain the other.
        out = self.links[0].read(n)
        self.links[1].read(65536)
        return out

    def close(self):
        for link in self.links:
            link.close()


def open_link(kind, ip=None):
    """kind is "wifi", "ble" or "both"."""
    if kind == "wifi":
        return WifiLink(ip)
    if kind == "ble":
        return BleLink()
    if kind == "both":
        return BothLinks(ip)
    raise ValueError("unknown link %r" % kind)


def wait_connected(link, timeout=20.0):
    """Wait for the first connection so startup reports the real state."""
    end = time.time() + timeout
    while time.time() < end and not link.connected:
        time.sleep(0.1)
    return link.connected
