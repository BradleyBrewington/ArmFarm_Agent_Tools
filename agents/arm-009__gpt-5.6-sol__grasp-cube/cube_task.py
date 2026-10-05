#!/usr/bin/env python3
"""Recorded black-cube pick/place loop for this armfarm station."""
from __future__ import annotations

import json
import math
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calibrate_workspace as cw
import camd_client
import fk
import recording

ARM = tuple(cw.ARM_JOINTS)


def transform(joints):
    """Full base_link -> gripper_frame_link transform for calibrated degrees."""
    def rot(axis, angle):
        axis = np.asarray(axis, dtype=float); axis /= np.linalg.norm(axis)
        return Rotation.from_rotvec(axis * angle).as_matrix()
    tree = ET.fromstring(fk.URDF)
    by_child = {j.find("child").get("link"): j for j in tree.findall("joint")}
    chain, link = [], "gripper_frame_link"
    while link != "base_link":
        joint = by_child[link]; chain.append(joint); link = joint.find("parent").get("link")
    T = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        r, p, y = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        F = np.eye(4); F[:3, :3] = rot([0, 0, 1], y) @ rot([0, 1, 0], p) @ rot([1, 0, 0], r); F[:3, 3] = xyz
        T = T @ F
        if joint.get("type") != "fixed":
            M = np.eye(4); M[:3, :3] = rot(np.fromstring(joint.find("axis").get("xyz"), sep=" "), math.radians(joints[joint.get("name")]))
            T = T @ M
    return T


def pose_ik(xyz, reference, calibration):
    """IK that preserves the reference gripper orientation."""
    target_R = transform(reference)[:3, :3]
    names = list(ARM[:4])
    x0 = np.array([reference[j] for j in names], dtype=float)
    bounds = [], []
    for j in names:
        c = calibration[j]; half = (c.range_max - c.range_min) * 180 / 4095
        bounds[0].append(-half); bounds[1].append(half)
    def residual(q):
        pose = dict(reference); pose.update(dict(zip(names, q)))
        T = transform(pose)
        position = (T[:3, 3] - xyz) * 1000.0
        orientation = Rotation.from_matrix(target_R.T @ T[:3, :3]).as_rotvec() * 120.0
        return np.r_[position, orientation]
    result = least_squares(residual, x0, bounds=bounds, max_nfev=1000, xtol=1e-11, ftol=1e-11, gtol=1e-11)
    pose = dict(reference); pose.update(dict(zip(names, result.x)))
    miss_mm = float(np.linalg.norm(transform(pose)[:3, 3] - xyz) * 1000)
    if not result.success or miss_mm > 2.0:
        raise RuntimeError(f"pose IK failed: {result.message}; position miss {miss_mm:.2f} mm")
    return pose


def find_cube():
    image, meta = camd_client.read_frame("top")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, 75)
    mask[:80] = 0
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour); x, y, w, h = cv2.boundingRect(contour)
        if 900 < area < 12000 and .65 < w / max(h, 1) < 1.45:
            candidates.append((area, x + w / 2, y + h / 2, w, h))
    if not candidates:
        raise RuntimeError("black cube not found in fresh top-camera frame")
    area, x, y, w, h = max(candidates)
    return {"pixel": [x, y], "box": [w, h], "area": area, "frame_seq": meta["seq"]}


def home(bus, home_target, seconds=1.5):
    # Deliberately command only the four constrained home joints.
    cw.move(bus, home_target, seconds); return cw.settle(bus, home_target, .5)


def run_once(place_xyz=None):
    home_target = cw.load_home(HERE.parent / "home_pose.json")
    port = recording.serial_port()
    with cw.connected_bus(port) as bus:
        cw.enable_at_current_position(bus)
        current = bus.sync_read("Present_Position", list(cw.JOINTS))
        home(bus, home_target)
        reference = bus.sync_read("Present_Position", list(ARM))
        start_T = transform(reference)
        # Station alignment measured from low-height top-camera probes: the
        # initial cube is 60 mm farther in base X and 10 mm lower in base Y
        # than the home tool projection.
        pick_xy = start_T[:2, 3] + np.array([.060, -.010])
        approach = pose_ik(np.r_[pick_xy, .085], reference, bus.calibration)
        grasp = pose_ik(np.r_[pick_xy, .042], reference, bus.calibration)
        cube_before = find_cube()
        episode = recording.start_recording("Pick up black cube, bring it to home, and confirm grasp")
        success = False; notes = ""
        try:
            cw.move(bus, {"gripper": 65.0}, .5)
            cw.move(bus, approach, 1.0)
            cw.move(bus, grasp, .8); cw.settle(bus, grasp, .2)
            cw.move(bus, {"gripper": 0.0}, .7); cw.settle(bus, {"gripper": 0.0}, .3)
            held = bus.sync_read("Present_Position", ["gripper"])["gripper"]
            fault = cw.fault_bits(bus, "gripper")
            cw.move(bus, approach, .7)
            home(bus, home_target, 1.2)
            # Empty closure reaches approximately 0%; a retained opening is contact evidence.
            success = held >= 5.0 and fault in (0, cw.OVERLOAD)
            notes = f"contact gripper={held:.2f}%, fault_bits={fault}; cube_before={cube_before['pixel']}"
        except Exception as exc:
            notes = f"pick exception: {type(exc).__name__}: {exc}"
            raise
        finally:
            stopped = recording.stop_recording(success, notes)
            print(json.dumps({"episode": episode, "stopped": stopped, "success": success, "notes": notes}, indent=2), flush=True)
        if not success:
            cw.move(bus, {"gripper": 65.0}, .5)
            raise RuntimeError("grasp validation failed: " + notes)
        if place_xyz is None:
            place_xyz = np.r_[pick_xy, .042]
        place_xyz = np.asarray(place_xyz, dtype=float)
        place_approach = pose_ik(np.r_[place_xyz[:2], .085], reference, bus.calibration)
        place = pose_ik(place_xyz, reference, bus.calibration)
        cw.move(bus, place_approach, 1.0); cw.move(bus, place, .8)
        cw.move(bus, {"gripper": 65.0}, .5)
        cw.move(bus, place_approach, .7)
        home(bus, home_target, 1.2)
        return {"success": True, "gripper_contact_percent": held, "pick_xyz": [*pick_xy, .042], "place_xyz": place_xyz.tolist()}


if __name__ == "__main__":
    try:
        destination = [float(x) for x in sys.argv[1:]] if len(sys.argv) > 1 else None
        if destination is not None and len(destination) != 3:
            raise ValueError("usage: cube_task.py [PLACE_X PLACE_Y PLACE_Z]")
        print(json.dumps(run_once(destination), indent=2))
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
        raise
