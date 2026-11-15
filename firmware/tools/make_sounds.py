#!/usr/bin/env python3
"""Make the clips for the firmware's "say" command.

    python3 tools/make_sounds.py                    all clips
    python3 tools/make_sounds.py red                one clip
    python3 tools/make_sounds.py red "Stop. Red."   new phrase for one clip

A file in main/sounds/src/<name>.<ext> is used instead of the spoken phrase.
Writes main/sounds/<name>.raw (8 kHz mono, unsigned 8-bit). Needs ffmpeg with flite.
New clips also go in CLIPS (main/sound.cpp) and EMBED_FILES (main/CMakeLists.txt).
"""
import subprocess
import sys
from pathlib import Path

import numpy as np

PHRASES = {
    "estop": "Emergency stop.",
    "obstacle": "Obstacle ahead.",
    "red": "Red detected.",
    "tilt": "Chair tilted.",
    "link": "Link lost.",
    "resume": "Resuming.",
}

OUT = Path(__file__).resolve().parent.parent / "main" / "sounds"
SRC = OUT / "src"

# Same treatment for a phrase or a recording: cut the bass the speaker can't
# play and the treble 8 kHz can't hold, then 8 kHz mono 16 bit for numpy.
FILTER = ["-af", "highpass=f=250,lowpass=f=3500",
          "-ar", "8000", "-ac", "1", "-f", "s16le", "-"]


def recording(name):
    hits = sorted(SRC.glob(name + ".*")) if SRC.is_dir() else []
    return hits[0] if hits else None


def make(name, text=None):
    source = None if text else recording(name)
    if source:
        args = ["-i", str(source)]
        what = source.name
    else:
        text = text or PHRASES[name]
        safe = text.replace("\\", "").replace("'", "").replace(":", " ")
        args = ["-f", "lavfi", "-i", f"flite=text='{safe}'"]
        what = repr(text)
    pcm = subprocess.run(
        ["ffmpeg", "-hide_banner", "-loglevel", "error"] + args + FILTER,
        check=True, capture_output=True).stdout
    x = np.frombuffer(pcm, dtype=np.int16).astype(np.float32)
    x /= max(np.abs(x).max(), 1.0)                 # loudest point = full scale
    u8 = np.clip(np.round(x * 127 + 128), 0, 255).astype(np.uint8)
    out = OUT / (name + ".raw")
    out.write_bytes(u8.tobytes())
    print(f"{out.name}: {len(u8) / 8000:.1f} s, {what}")


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    names = [sys.argv[1]] if len(sys.argv) > 1 else list(PHRASES)
    for name in names:
        if name not in PHRASES:
            sys.exit("unknown clip %r, one of: %s" % (name, ", ".join(PHRASES)))
    if len(sys.argv) == 3:
        make(names[0], sys.argv[2])
    else:
        for name in names:
            make(name)
