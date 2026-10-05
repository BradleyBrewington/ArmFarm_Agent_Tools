#!/usr/bin/env python3
"""Refit the cube pixel -> base_link map from successful picks.

Every successful pick logs the cube's top-camera pixel (before the pick) and the cube
position the wrist servo converged to, which is far more accurate than the fingertip
calibration. Fits the CUBE_PLANE_Z homography in top_plane_maps.json (robust, RANSAC).
"""
import json
import sys

import numpy as np

import vision

LOG = vision.HERE / "episodes_log.jsonl"


def samples():
    out = []
    for line in LOG.read_text().splitlines():
        e = json.loads(line)
        p = e.get("pick") or {}
        s = p.get("servo") or {}
        # only first-try picks: the logged pixel is from the same look as the servo result
        if e.get("success") and p.get("attempts") == 1 and p.get("px") and "x" in s:
            out.append((p["px"][0], p["px"][1], s["x"], s["y"]))
    return out


def main():
    S = samples()
    if len(S) < 12:
        print(f"only {len(S)} samples; need 12"); return 1
    before = np.array([np.hypot(*(np.array(vision.cube_robot(s[:2])) - s[2:])) for s in S])
    H, err = vision.fit_plane_map(S, vision.CUBE_PLANE_Z)
    print(f"{len(S)} samples: old map err mean {1000 * before.mean():.1f} mm max {1000 * before.max():.1f}; "
          f"refit rms {1000 * np.sqrt((err ** 2).mean()):.1f} mm max {1000 * err.max():.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
