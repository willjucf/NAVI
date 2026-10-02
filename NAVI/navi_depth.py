"""NAVI depth sensing. Arducam Time of Flight capture and navigation zones.

Implements report 3.7.4. A ToF camera gives a depth map, and the useful part is
knowing if an obstacle is in the chair's path or off to one side. So the map is
split into left, center and right zones, each reduced to one distance and one
verdict the control loop can act on.

ArducamDepthCamera is Pi only, so it is imported lazily and a simulated source
is included. That keeps this module importable and testable anywhere.

Run it directly to watch the zones update:

    python3 navi_depth.py            live Arducam
    python3 navi_depth.py --sim      synthetic moving obstacle, no hardware
"""

import argparse
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
    """Reduce one zone of the depth map to a distance and a verdict.

    Distance is the median of valid pixels, not the mean or minimum. Dropouts
    and reflections show up as lone extreme values and both of those chase
    them. The minimum would flag a hazard from one bad pixel and stop the chair
    constantly.

    Too few valid pixels returns UNKNOWN, not a number. Blind is not clear, so
    the caller treats UNKNOWN as blocked, same as no detection meaning STOP in
    6.11.2.
    """
    total = zone_mm.size
    if total == 0:
        return {"distance_mm": None, "verdict": UNKNOWN, "valid_frac": 0.0,
                "valid_px": 0, "total_px": 0}

    valid = zone_mm[(zone_mm >= cfg.DEPTH_MIN_MM) & (zone_mm <= cfg.DEPTH_MAX_MM)]
    valid_frac = float(valid.size) / float(total)

    if valid_frac < cfg.ZONE_MIN_VALID_FRAC:
        return {"distance_mm": None, "verdict": UNKNOWN, "valid_frac": valid_frac,
                "valid_px": int(valid.size), "total_px": int(total)}

    distance = int(round(float(np.median(valid))))
    if distance < cfg.ZONE_BLOCKED_MM:
        verdict = BLOCKED
    elif distance < cfg.ZONE_CAUTION_MM:
        verdict = CAUTION
    else:
        verdict = CLEAR

    return {"distance_mm": distance, "verdict": verdict, "valid_frac": valid_frac,
            "valid_px": int(valid.size), "total_px": int(total)}


def zone_map(depth_mm):
    """Split a depth map into left, center and right zones and judge each.

    depth_mm is a 2D array in mm, as the Arducam library returns. Splitting by
    columns makes it steerable: report 3.7.4 wants to know if the center is
    blocked and which side has more room.
    """
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

    # When the path is blocked, say which side has more room. Not acted on
    # yet since turns need motors to verify, but it is free to report and a
    # turn decision will need it.
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

        # The library API changed between releases. Current builds take a
        # connection type and device index in open().
        ret = self.cam.open(ac.Connection.CSI, 0)
        if ret != 0:
            raise RuntimeError("Arducam open failed with code %s. Check the "
                               "ribbon cable seating and that dtoverlay is set."
                               % ret)
        ret = self.cam.start(ac.FrameType.DEPTH)
        if ret != 0:
            self.cam.close()
            raise RuntimeError("Arducam start failed with code %s" % ret)
        # Widest range so the 2 to 4 m window in 3.7.4 is inside what the
        # sensor is set up for.
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
            depth = np.asarray(frame.depth_data, dtype=np.float32)
        finally:
            self.cam.releaseFrame(frame)
        return depth

    def describe(self):
        return "Arducam ToF over CSI ribbon, %dx%d" % (cfg.DEPTH_WIDTH,
                                                       cfg.DEPTH_HEIGHT)

    def close(self):
        try:
            self.cam.stop()
        finally:
            self.cam.close()


class SimDepthSource:
    """Synthetic depth maps for tests and bench work with no ToF.

    A flat wall with one box that sweeps left to right while coming closer and
    then backing off, so every verdict and turn suggestion gets hit the same
    way every run.

    It is closest exactly when it is centered. That is on purpose so the sim
    actually produces a blocked path and --sim shows the stop, not just clear.

    It spans one third of the width and most of the height so it dominates the
    center zone when centered. Smaller and the background would outvote it in
    the median and it would never read as blocked.
    """

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

class DepthReader:
    """Polls a depth source on its own thread and keeps the latest zones.

    Vision calls latest() every frame and must never block, so only the newest
    result is kept (same single slot idea as report 6.14).
    """

    def __init__(self, source=None, sim=False):
        if source is not None:
            self.source = source
        else:
            self.source = SimDepthSource() if sim else ArducamSource()
        self._latest = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self.frames = 0
        self.error = None

    def describe(self):
        return self.source.describe()

    def start(self):
        self._thread = threading.Thread(target=self._run, name="depth",
                                        daemon=True)
        self._thread.start()
        return self

    def _run(self):
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

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
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
          "valid window %d to %dmm"
          % (cfg.ZONE_BLOCKED_MM, cfg.ZONE_CAUTION_MM,
             cfg.DEPTH_MIN_MM, cfg.DEPTH_MAX_MM))
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
