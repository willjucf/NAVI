"""Test fixtures: synthetic frames and depth maps, and fakes for the camera and serial port."""

import os
import sys
import threading
import time
import unittest

import numpy as np

# Make the NAVI modules importable no matter which directory the runner started
# in, so the same command works on Windows and on the Pi.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

try:
    import cv2
    HAVE_CV2 = True
except ImportError:
    cv2 = None
    HAVE_CV2 = False


requires_cv2 = unittest.skipUnless(
    HAVE_CV2,
    "OpenCV is not installed here. These run on the Pi, which has "
    "python3-opencv, and are skipped elsewhere.")


# Synthetic camera frames

FRAME_W, FRAME_H = 640, 480

# BGR, which is the order OpenCV uses.
BGR_GREEN = (40, 200, 40)
BGR_RED = (40, 40, 200)
BGR_GREY = (90, 90, 90)


def blank_frame(w=FRAME_W, h=FRAME_H, color=BGR_GREY):
    """A featureless background that matches no threshold."""
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:, :] = color
    return frame


def add_blob(frame, color, cx_frac=0.5, size_frac=0.25):
    """Paint a square into the frame. cx_frac is the horizontal center (0..1), size_frac its width."""
    h, w = frame.shape[:2]
    half = max(1, int(size_frac * w / 2))
    cx = int(cx_frac * w)
    cy = h // 2
    x0, x1 = max(0, cx - half), min(w, cx + half)
    y0, y1 = max(0, cy - half), min(h, cy + half)
    frame[y0:y1, x0:x1] = color
    return frame


def green_frame(cx_frac=0.5, size_frac=0.25):
    return add_blob(blank_frame(), BGR_GREEN, cx_frac, size_frac)


def red_frame(cx_frac=0.5, size_frac=0.25):
    return add_blob(blank_frame(), BGR_RED, cx_frac, size_frac)


def red_and_green_frame():
    """Both markers at once."""
    frame = blank_frame()
    add_blob(frame, BGR_GREEN, cx_frac=0.25, size_frac=0.25)
    add_blob(frame, BGR_RED, cx_frac=0.75, size_frac=0.25)
    return frame


def glare_frame():
    """Bright, nearly grey patch. The saturation floor must reject it as green."""
    frame = blank_frame(color=(200, 200, 200))
    add_blob(frame, (250, 252, 250), cx_frac=0.5, size_frac=0.4)
    return frame


def speckle_frame(color=BGR_GREEN, count=400, seed=1):
    """Scattered single pixels of a target colour."""
    frame = blank_frame()
    rng = np.random.default_rng(seed)
    h, w = frame.shape[:2]
    ys = rng.integers(0, h, count)
    xs = rng.integers(0, w, count)
    frame[ys, xs] = color
    return frame


# Synthetic depth maps

def flat_depth(distance_mm=3000.0, w=None, h=None):
    import navi_config as cfg
    w = w or cfg.DEPTH_WIDTH
    h = h or cfg.DEPTH_HEIGHT
    return np.full((h, w), float(distance_mm), dtype=np.float32)


def depth_with_obstacle(zone, distance_mm=400.0, background_mm=3000.0):
    """A depth map with one of the three zones occupied.

    zone is left, center or right.
    """
    import navi_config as cfg
    depth = flat_depth(background_mm)
    w = depth.shape[1]
    bounds = {"left": (0, w // 3),
              "center": (w // 3, 2 * w // 3),
              "right": (2 * w // 3, w)}
    x0, x1 = bounds[zone]
    depth[:, x0:x1] = float(distance_mm)
    return depth


# Fake hardware

class FakeSource:
    """Fake camera returning a fixed list of frames. block=True simulates a stalled camera."""

    def __init__(self, frames, loop=True, label="fake"):
        self.frames = list(frames)
        self.loop = loop
        self.label = label
        self.i = 0
        self.released = False
        self.block = False

    def read(self):
        if self.block:
            time.sleep(0.005)
            return None, None
        if not self.frames:
            return None, None
        if self.i >= len(self.frames) and not self.loop:
            return None, None
        frame = self.frames[self.i % len(self.frames)]
        self.i += 1
        return frame.copy(), self.label

    def describe(self):
        return "fake source, %d frame(s)" % len(self.frames)

    def release(self):
        self.released = True


class FakeSerial:
    """Fake serial port that records every byte written."""

    def __init__(self, fail_after=None):
        self.writes = []
        self.closed = False
        self.fail_after = fail_after
        self._lock = threading.Lock()

    def write(self, data):
        with self._lock:
            if self.fail_after is not None and len(self.writes) >= self.fail_after:
                raise IOError("simulated serial failure")
            self.writes.append(bytes(data))
            return len(data)

    def packets(self):
        """Everything written, decoded and split into whole frames."""
        with self._lock:
            blob = b"".join(self.writes).decode("ascii", "replace")
        out = []
        depth = 0
        current = ""
        for ch in blob:
            if ch == "<":
                depth, current = 1, "<"
            elif depth:
                current += ch
                if ch == ">":
                    out.append(current)
                    depth, current = 0, ""
        return out

    def close(self):
        self.closed = True


class FakeDepthSource:
    """Stands in for the Arducam. Returns depth maps from a fixed list."""

    def __init__(self, maps):
        self.maps = list(maps)
        self.i = 0
        self.closed = False

    def read(self):
        if not self.maps:
            return None
        depth = self.maps[self.i % len(self.maps)]
        self.i += 1
        return depth

    def describe(self):
        return "fake depth source"

    def close(self):
        self.closed = True


def wait_until(predicate, timeout=2.0, interval=0.005):
    """Poll until predicate is true or the timeout passes. Returns whether it was met."""
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()
