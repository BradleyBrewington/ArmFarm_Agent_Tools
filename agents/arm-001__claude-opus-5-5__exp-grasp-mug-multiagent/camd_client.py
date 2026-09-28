#!/usr/bin/env python3
"""Reader for the camera daemon's /dev/shm slots. The one way any process
gets a camera frame while camd runs (nothing else may open /dev/v4l/*).

Library:
    from camd_client import read_frame, read_jpeg, alive
    img, meta = read_frame("top")            # BGR ndarray (720p), staleness-checked
    jpeg, meta = read_jpeg("wrist")          # compressed bytes, no decode
    alive()                                  # True iff both slots are fresh

Raises CamdNotRunning (no slot file) or CamdStale (frame older than max_age).
A healthy camd keeps age well under 100 ms; anything older means the daemon
is dead or a camera is wedged — fail loudly, never aim off an old frame.

CLI:
    ./camd_client.py snap top [out.jpg]      # save latest frame
    ./camd_client.py status                  # one line per cam
    ./camd_client.py watch [seconds]         # soak reader: rates + staleness
"""

import json
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np

DIR = Path("/dev/shm/so101_camd")
MAGIC = b"CMD1"
MAX_AGE = 0.25   # default staleness bound, seconds


class CamdError(RuntimeError):
    pass


class CamdNotRunning(CamdError):
    pass


class CamdStale(CamdError):
    pass


def read_jpeg(cam, max_age=MAX_AGE):
    """Latest compressed frame -> (jpeg_bytes, meta dict)."""
    try:
        data = (DIR / f"{cam}.frame").read_bytes()
    except FileNotFoundError:
        raise CamdNotRunning(f"no slot for '{cam}' — start ./camd.py") from None
    if data[:4] != MAGIC:
        raise CamdError(f"bad magic in {cam} slot")
    n = struct.unpack("<I", data[4:8])[0]
    meta = json.loads(data[8:8 + n])
    age = time.monotonic() - meta["t_mono"]
    if max_age is not None and age > max_age:
        raise CamdStale(f"{cam} frame is {age*1000:.0f} ms old (camd dead or camera wedged)")
    meta["age"] = age
    return data[8 + n:], meta


def read_frame(cam, max_age=MAX_AGE):
    """Latest frame decoded to BGR (h, w, 3) -> (img, meta dict)."""
    jpeg, meta = read_jpeg(cam, max_age=max_age)
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise CamdError(f"{cam}: JPEG decode failed")
    return img, meta


def alive(max_age=0.5):
    """True iff both cameras have fresh frames."""
    try:
        for cam in ("top", "wrist"):
            read_jpeg(cam, max_age=max_age)
        return True
    except CamdError:
        return False


# ---------------------------------------------------------------- CLI

def _status():
    for cam in ("top", "wrist"):
        try:
            jpeg, m = read_jpeg(cam, max_age=None)
            age = time.monotonic() - m["t_mono"]
            print(f"{cam}: seq={m['seq']} age={age*1000:.0f}ms fps={m['fps']} "
                  f"jpeg={len(jpeg)} B raw={m['raw']}")
        except CamdError as e:
            print(f"{cam}: {e}")


def _watch(secs):
    """Soak reader: poll both cams, report seq rate / staleness / dupes."""
    t0 = time.monotonic()
    last_seq = {"top": None, "wrist": None}
    got = {"top": 0, "wrist": 0}
    dup = {"top": 0, "wrist": 0}
    worst = {"top": 0.0, "wrist": 0.0}
    stale_events = 0
    next_report = t0 + 10.0
    while time.monotonic() - t0 < secs:
        for cam in ("top", "wrist"):
            try:
                _, m = read_jpeg(cam, max_age=MAX_AGE)
            except CamdStale:
                stale_events += 1
                continue
            except CamdNotRunning:
                print(f"{time.monotonic()-t0:7.1f}s  {cam}: CAMD NOT RUNNING")
                stale_events += 1
                time.sleep(0.5)
                continue
            worst[cam] = max(worst[cam], m["age"])
            if m["seq"] != last_seq[cam]:
                if last_seq[cam] is not None and m["seq"] == last_seq[cam]:
                    dup[cam] += 1
                got[cam] += 1
                last_seq[cam] = m["seq"]
        time.sleep(1.0 / 60)   # poll faster than the cams so nothing is missed
        if time.monotonic() >= next_report:
            el = time.monotonic() - t0
            print(f"{el:7.1f}s  " + "  ".join(
                f"{c}: {got[c]/el:5.1f} new/s worst_age={worst[c]*1000:3.0f}ms"
                for c in ("top", "wrist")) + f"  stale_events={stale_events}",
                flush=True)
            next_report += 10.0
    el = time.monotonic() - t0
    print("FINAL  " + "  ".join(
        f"{c}: {got[c]} frames ({got[c]/el:.1f}/s) worst_age={worst[c]*1000:.0f}ms"
        for c in ("top", "wrist")) + f"  stale_events={stale_events}", flush=True)


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ("snap", "status", "watch"):
        print(__doc__)
        sys.exit(1)
    cmd = sys.argv[1]
    if cmd == "status":
        _status()
    elif cmd == "snap":
        cam = sys.argv[2]
        out = sys.argv[3] if len(sys.argv) > 3 else f"/tmp/camd_{cam}.jpg"
        img, m = read_frame(cam)
        cv2.imwrite(out, img)
        print(f"{out}  seq={m['seq']} age={m['age']*1000:.0f}ms {img.shape[1]}x{img.shape[0]}")
    elif cmd == "watch":
        _watch(float(sys.argv[2]) if len(sys.argv) > 2 else 60.0)


if __name__ == "__main__":
    main()
