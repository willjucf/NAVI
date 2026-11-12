"""NAVI depth: Arducam ToF capture and left / center / right zones.

The Arducam SDK is imported lazily and a simulated source is included, so this
module imports anywhere.

    python3 navi_depth.py            live Arducam
    python3 navi_depth.py --sim      simulated obstacle
"""

import argparse
import multiprocessing
import threading
import time

import numpy as np

import navi_config as cfg

CLEAR = "CLEAR"
CAUTION = "CAUTION"
BLOCKED = "BLOCKED"
UNKNOWN = "UNKNOWN"

ZONE_NAMES = ("left", "center", "right")


# Zone analysis, pure numpy so it can be tested with no camera

def zone_stats(zone_mm):
    """Reduce one zone to a distance and a verdict.

        Distance is the ZONE_PERCENTILE (5th) percentile of every non-zero reading up to
        DEPTH_MAX_MM, close ones included, so an object filling ~5% of the zone counts.
        Too few usable pixels returns UNKNOWN, which callers treat as blocked.
    """
    total = zone_mm.size
    if total == 0:
        return {"distance_mm": None, "verdict": UNKNOWN, "valid_frac": 0.0,
                "valid_px": 0, "total_px": 0}

    usable = zone_mm[(zone_mm > 0) & (zone_mm <= cfg.DEPTH_MAX_MM)]
    valid_frac = float(usable.size) / float(total)

    if valid_frac < cfg.ZONE_MIN_VALID_FRAC:
        return {"distance_mm": None, "verdict": UNKNOWN, "valid_frac": valid_frac,
                "valid_px": int(usable.size), "total_px": int(total)}

    distance = int(round(float(np.percentile(usable, cfg.ZONE_PERCENTILE))))
    if distance < cfg.ZONE_BLOCKED_MM:
        verdict = BLOCKED
    elif distance < cfg.ZONE_CAUTION_MM:
        verdict = CAUTION
    else:
        verdict = CLEAR

    return {"distance_mm": distance, "verdict": verdict, "valid_frac": valid_frac,
            "valid_px": int(usable.size), "total_px": int(total)}


def zone_map(depth_mm):
    """Split a depth map (mm) into left, center and right zones and judge each."""
    depth_mm = np.asarray(depth_mm)
    if depth_mm.ndim != 2:
        raise ValueError("depth map must be 2D, got shape %r" % (depth_mm.shape,))

    w = depth_mm.shape[1]
    bounds = (0, w // 3, 2 * w // 3, w)
    zones = {}
    for i, name in enumerate(ZONE_NAMES):
        zones[name] = zone_stats(depth_mm[:, bounds[i]:bounds[i + 1]])

    center = zones["center"]
    path_blocked = center["verdict"] in (BLOCKED, UNKNOWN)
    if center["verdict"] == BLOCKED:
        reason = "CENTER %dmm" % center["distance_mm"]
    elif center["verdict"] == UNKNOWN:
        reason = "CENTER NO VALID DEPTH"
    else:
        reason = ""

    # Which side has more room when the path is blocked. Reported only.
    suggestion = None
    if path_blocked:
        left_d = zones["left"]["distance_mm"]
        right_d = zones["right"]["distance_mm"]
        if left_d is None and right_d is None:
            suggestion = None
        elif right_d is None or (left_d is not None and left_d > right_d):
            suggestion = "left"
        else:
            suggestion = "right"

    return {
        "zones": zones,
        "path_blocked": path_blocked,
        "reason": reason,
        "suggested_turn": suggestion,
        "t": time.time(),
    }


# Depth sources

def find_tof_video_index():
    """The /dev/videoN index of the Pi 5 CSI receiver, or cfg.DEPTH_VIDEO_INDEX."""
    import glob
    import os
    import re
    found = []
    for path in glob.glob("/sys/class/video4linux/video*"):
        try:
            with open(os.path.join(path, "name")) as fh:
                name = fh.read().strip()
        except OSError:
            continue
        if name.startswith("rp1-cfe-csi2_ch0"):
            found.append(int(re.sub(r"\D", "", os.path.basename(path))))
    return min(found) if found else cfg.DEPTH_VIDEO_INDEX


class ArducamSource:
    """The real Arducam ToF camera on the Pi ribbon connector."""

    def __init__(self):
        try:
            import ArducamDepthCamera as ac
        except ImportError as exc:
            raise RuntimeError(
                "ArducamDepthCamera is not installed. It is a Pi only library. "
                "Use --sim to exercise the zone logic without hardware."
            ) from exc
        self._ac = ac
        self.cam = ac.ArducamCamera()

        # open() takes the /dev/videoN index, not the CSI port. With the webcam plugged in the ToF is video2.
        self.index = find_tof_video_index()
        ret = self.cam.open(ac.Connection.CSI, self.index)
        if ret != 0:
            raise RuntimeError("Arducam open failed with code %s. Check the "
                               "ribbon cable seating and that dtoverlay is set."
                               % ret)
        ret = self.cam.start(ac.FrameType.DEPTH)
        if ret != 0:
            self.cam.close()
            raise RuntimeError("Arducam start failed with code %s" % ret)
        # Widest range (4 m).
        try:
            self.cam.setControl(ac.Control.RANGE, cfg.DEPTH_MAX_MM)
        except Exception:
            pass

    def read(self):
        """One depth map in mm, or None if no frame was ready."""
        frame = self.cam.requestFrame(200)
        if frame is None:
            return None
        try:
            depth = np.array(frame.depth_data, dtype=np.float32)
            confidence = np.asarray(frame.confidence_data, dtype=np.float32)
            # low confidence pixels count as no reading (DEPTH_MIN_CONFIDENCE)
            depth[confidence < cfg.DEPTH_MIN_CONFIDENCE] = 0
        finally:
            self.cam.releaseFrame(frame)
        return depth

    def describe(self):
        return "Arducam ToF over CSI ribbon, %dx%d on /dev/video%d" % (
            cfg.DEPTH_WIDTH, cfg.DEPTH_HEIGHT, self.index)

    def close(self):
        try:
            self.cam.stop()
        finally:
            self.cam.close()


class SimDepthSource:
    """Synthetic depth: a box sweeping across a wall, closest when centered."""

    PERIOD = 120

    def __init__(self, width=None, height=None, seed=0):
        self.w = width or cfg.DEPTH_WIDTH
        self.h = height or cfg.DEPTH_HEIGHT
        self.i = 0
        self.rng = np.random.default_rng(seed)

    def read(self):
        self.i += 1
        depth = np.full((self.h, self.w), 3000.0, dtype=np.float32)

        phase = (self.i % self.PERIOD) / float(self.PERIOD)
        cx = int(phase * self.w)

        # Triangle wave, 0 at the ends of the sweep and 1 in the middle.
        nearness = 1.0 - abs(2.0 * phase - 1.0)
        distance = 2600.0 - 2200.0 * nearness

        half = max(1, self.w // 6)          # full box is one third of the width
        x0, x1 = max(0, cx - half), min(self.w, cx + half)
        y0, y1 = int(0.1 * self.h), int(0.9 * self.h)
        depth[y0:y1, x0:x1] = distance

        # scattered dropouts like the real sensor gives
        holes = self.rng.random(depth.shape) < 0.02
        depth[holes] = 0.0
        return depth

    def describe(self):
        return "simulated depth source, %dx%d" % (self.w, self.h)

    def close(self):
        pass


# Background reader

def _arducam_worker(conn, stop_evt):
    """Child process that owns the Arducam and sends back zone summaries.

        The SDK holds the GIL while waiting for a frame, which stalled the vision thread.
    """
    # Ctrl C hits the whole process group. The parent handles shutdown through
    # stop_evt, so ignore it here instead of dying mid read with a traceback.
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        src = ArducamSource()
    except Exception as exc:
        conn.send(("error", str(exc)))
        conn.close()
        return
    conn.send(("ready", src.describe()))
    try:
        while not stop_evt.is_set():
            depth = src.read()
            if depth is None:
                continue
            conn.send(("zones", zone_map(depth)))
    except (BrokenPipeError, EOFError, OSError):
        pass                          # parent went away
    except Exception as exc:
        try:
            conn.send(("error", repr(exc)))
        except Exception:
            pass
    finally:
        src.close()


class DepthReader:
    """Polls a depth source in the background and keeps only the latest zones.

        The Arducam runs in a child process; injected and simulated sources use a thread.
    """

    def __init__(self, source=None, sim=False):
        self._proc = None
        self._conn = None
        self._proc_stop = None
        self._describe = None
        if source is not None:
            self.source = source
        elif sim:
            self.source = SimDepthSource()
        else:
            self.source = None
            self._start_process()
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.frames = 0
        self.error = None

    def _start_process(self):
        # spawn, not fork. The webcam and other threads may already be open
        # here and a forked child would inherit them.
        ctx = multiprocessing.get_context("spawn")
        recv_end, send_end = ctx.Pipe(duplex=False)
        self._proc_stop = ctx.Event()
        self._proc = ctx.Process(target=_arducam_worker, name="depth-proc",
                                 args=(send_end, self._proc_stop), daemon=True)
        self._proc.start()
        send_end.close()
        self._conn = recv_end
        if not recv_end.poll(20.0):
            self._proc.terminate()
            raise RuntimeError("Arducam process did not start within 20 s")
        kind, payload = recv_end.recv()
        if kind != "ready":
            self._proc.join(timeout=2.0)
            raise RuntimeError(payload)
        self._describe = payload + ", own process"

    def describe(self):
        return self._describe if self._proc else self.source.describe()

    def start(self):
        self._thread = threading.Thread(target=self._run, name="depth",
                                        daemon=True)
        self._thread.start()
        return self

    def _run(self):
        if self._proc is not None:
            self._run_process()
            return
        try:
            while not self._stop.is_set():
                depth = self.source.read()
                if depth is None:
                    self._stop.wait(0.005)
                    continue
                summary = zone_map(depth)
                with self._lock:
                    self._latest = summary
                    self.frames += 1
        except Exception as exc:
            self.error = exc

    def latest(self):
        with self._lock:
            return self._latest

    def _run_process(self):
        while not self._stop.is_set():
            try:
                if not self._conn.poll(0.1):
                    continue
                kind, payload = self._conn.recv()
            except (EOFError, OSError) as exc:
                self.error = exc
                return
            if kind == "zones":
                with self._lock:
                    self._latest = payload
                    self.frames += 1
            elif kind == "error":
                self.error = RuntimeError(payload)
                return

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._proc is not None:
            self._proc_stop.set()
            self._proc.join(timeout=3.0)
            if self._proc.is_alive():
                self._proc.terminate()
            self._conn.close()
        else:
            self.source.close()


# Entry point

def format_row(summary):
    """One console line for the three zones."""
    parts = []
    for name in ZONE_NAMES:
        z = summary["zones"][name]
        mm = z["distance_mm"]
        parts.append("%-6s %7s %-8s" % (
            name.upper(),
            "n/a" if mm is None else "%dmm" % mm,
            z["verdict"]))
    tail = "PATH BLOCKED" if summary["path_blocked"] else "path clear"
    if summary["suggested_turn"]:
        tail += ", more room %s" % summary["suggested_turn"]
    return " | ".join(parts) + " || " + tail


def main():
    ap = argparse.ArgumentParser(description="NAVI Arducam ToF depth zones")
    ap.add_argument("--sim", action="store_true",
                    help="use a synthetic depth source instead of the camera")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after N seconds, 0 means run until Ctrl C")
    args = ap.parse_args()

    try:
        source = SimDepthSource() if args.sim else ArducamSource()
    except RuntimeError as exc:
        print("ERROR: %s" % exc)
        return 1

    print("Depth source: %s" % source.describe())
    print("Zone thresholds: blocked under %dmm, caution under %dmm, "
          "readings past %dmm ignored"
          % (cfg.ZONE_BLOCKED_MM, cfg.ZONE_CAUTION_MM, cfg.DEPTH_MAX_MM))
    print("Move an object toward and away from the camera. Ctrl C to stop.\n")

    started = time.perf_counter()
    frames = 0
    try:
        while True:
            if args.seconds and time.perf_counter() - started >= args.seconds:
                break
            depth = source.read()
            if depth is None:
                time.sleep(0.005)
                continue
            frames += 1
            summary = zone_map(depth)
            elapsed = time.perf_counter() - started
            print("\r%s  %5.1f fps " % (format_row(summary),
                                        frames / elapsed if elapsed else 0.0),
                  end="")
            if args.sim:
                time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        elapsed = time.perf_counter() - started
        print("\n\n%d depth frames in %.1f s, %.1f fps"
              % (frames, elapsed, frames / elapsed if elapsed else 0.0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
