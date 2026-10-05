#!/usr/bin/env python3
"""Arm control helpers for the grasp-cube task (robot venv).

    from arm import Arm
    with Arm() as a:
        a.joints()                 # calibrated degrees / gripper percent
        a.move({...}, seconds)     # smooth, clamped, stall-guarded (calibrate_workspace.move)
        a.home()                   # four home joints only
        a.gripper(60)

CLI:  python arm.py read | home | grip PCT | move JSON SECONDS
"""
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import calibrate_workspace as cw  # noqa: E402
from fk import forward  # noqa: E402

HOME_FILE = HERE.parent / "home_pose.json"
ARM = cw.ARM_JOINTS
ALL = cw.JOINTS


class Arm:
    def __init__(self, port=None):
        self.port = port or os.environ.get("ARMFARM_SERIAL_PORT", "auto")
        self._ctx = None
        self.bus = None
        self.home_pose = cw.load_home(HOME_FILE)

    def __enter__(self):
        self._ctx = cw.connected_bus(self.port, ALL)
        self.bus = self._ctx.__enter__()
        cw.preflight(self.bus, [self.joints()])
        cw.enable_at_current_position(self.bus, ALL)
        return self

    def __exit__(self, *exc):
        return self._ctx.__exit__(*exc)

    def joints(self):
        return dict(self.bus.sync_read("Present_Position", list(ALL)))

    def tip(self, joints=None):
        j = joints or self.joints()
        return forward({k: j[k] for k in ARM})

    def move(self, target, seconds):
        cw.move(self.bus, target, seconds)

    def wait(self, target, tol=3.0, timeout=1.5, joints=None):
        """Wait until the listed joints are within tol of target. Returns final error dict."""
        joints = [j for j in (joints or target) if j != "gripper"]
        goal = cw.within_stops(cw.clamp_target({j: target[j] for j in joints}, self.bus.calibration), self.bus)
        deadline = time.monotonic() + timeout
        while True:
            now = self.bus.sync_read("Present_Position", joints)
            err = {j: abs(now[j] - goal[j]) for j in joints}
            if max(err.values()) <= tol or time.monotonic() > deadline:
                return err
            time.sleep(0.02)

    def home(self, seconds=1.2):
        self.move(self.home_pose, seconds)
        return self.wait(self.home_pose, tol=4.0, timeout=2.0)

    def gripper(self, pct, seconds=0.4):
        self.move({"gripper": pct}, seconds)

    def gripper_pos(self):
        return float(self.bus.sync_read("Present_Position", ["gripper"])["gripper"])


def main():
    op = sys.argv[1] if len(sys.argv) > 1 else "read"
    with Arm() as a:
        if op == "home":
            print(json.dumps(a.home()))
        elif op == "grip":
            a.gripper(float(sys.argv[2]))
            time.sleep(0.3)
        elif op == "move":
            t = json.loads(sys.argv[2])
            a.move(t, float(sys.argv[3]) if len(sys.argv) > 3 else 1.5)
            print(json.dumps(a.wait(t)))
        j = a.joints()
        print(json.dumps({"joints": j, "tip": a.tip(j)}, indent=1))


if __name__ == "__main__":
    main()
