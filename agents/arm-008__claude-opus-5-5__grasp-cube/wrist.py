"""Wrist-camera cube detection (used to square the jaws to the cube before descending)."""
import json
import math
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
DARK = 60


def detect(img):
    """Cube blob in the wrist image (jaws at the bottom are excluded). Returns dict or None."""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 2] < DARK).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    h, w = m.shape
    best = None
    for i in range(1, n):
        x, y, bw, bh, area = st[i]
        if area < 4000 or area > 120000:
            continue
        if y + bh >= h - 3:            # touches the bottom: a jaw (or cube merged with one)
            continue
        if x <= 2 or x + bw >= w - 3 or y <= 2:
            continue
        cnt = cv2.findContours((lab == i).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
        rect = cv2.minAreaRect(cnt)
        fill = area / max(rect[1][0] * rect[1][1], 1)
        if fill < 0.7:
            continue
        d = {"px": (float(cen[i][0]), float(cen[i][1])), "area": int(area), "angle": float(rect[2]), "fill": fill}
        if best is None or area > best["area"]:
            best = d
    return best


def fold90(a):
    a = a % 90.0
    return a - 90.0 if a >= 45 else a
