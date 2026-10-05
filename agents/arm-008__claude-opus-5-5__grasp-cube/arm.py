#!/usr/bin/env python3
"""Arm control helpers for the grasp-cube task (robot venv).

    from arm import Arm
    with Arm() as a:
        a.joints()                 # calibrated degrees / gripper percent
        a.move({...}, seconds)     # smooth, clamped, stall-guarded, light bus traffic
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
GRIP_TORQUE = 230      # gripper Torque_Limit (of 1000); Overload_Torque is 25%
STEP_S = 0.03         # control period of Arm.move (bus shared with the recorder)


def _retry_bus(bus, tries=4):
    """Retry sync reads/writes on transient 'no status packet' errors (the recorder shares
    the serial relay and an occasional reply is lost)."""
    for name in ("sync_read", "sync_write"):
        fn = getattr(bus, name)

        def wrapped(*a, _fn=fn, **k):
            for i in range(tries):
                try:
                    return _fn(*a, **k)
                except ConnectionError:
                    if i == tries - 1:
                        raise
                    time.sleep(0.01)
        setattr(bus, name, wrapped)


class Arm:
    def __init__(self, port=None):
        self.port = port or os.environ.get("ARMFARM_SERIAL_PORT", "auto")
        self._ctx = None
        self.bus = None
        self.home_pose = cw.load_home(HOME_FILE)

    def __enter__(self):
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
        self.bus = FeetechMotorsBus(port=self.port, motors={
            j: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if j == "gripper" else MotorNormMode.DEGREES)
            for i, j in enumerate(ALL)})
        # handshake=False and the exported calibration file: a motor reporting overload
        # (e.g. the gripper squeezing the cube) must not stop us from connecting.
        self.bus.connect(handshake=False)
        _retry_bus(self.bus)
        cal = json.loads(Path(os.environ.get("ARMFARM_MOTOR_CALIBRATION",
                                             HERE.parent / "config/motors/arm-008.json")).read_text())
        self.bus.calibration = {j: MotorCalibration(**cal[j]) for j in ALL}
        for j in ALL:
            if cw.fault_bits(self.bus, j) & cw.OVERLOAD:
                if j == "gripper":
                    self.relax_gripper()
                else:
                    cw.recover_overload(self.bus, j)
        # Gripper overload protection trips above Overload_Torque (25%) for ~2 s, dropping the cube.
        # Cap the (RAM) torque limit below it so a sustained squeeze is safe.
        cw.write_register(self.bus, "Torque_Limit", "gripper", GRIP_TORQUE)
        cw.preflight(self.bus, [self.joints()])
        cw.enable_at_current_position(self.bus, ALL)
        if self.gripper_pos() < 50:   # may be holding the cube: keep squeezing so it does not slip
            self.hold()
        return self

    def __exit__(self, *exc):
        if self.bus.is_connected:
            self.bus.disconnect(disable_torque=False)

    def relax_gripper(self, ticks=40):
        """Clear a gripper overload by retargeting just inside the current position (keeps the grip).
        Raw position decreases as the gripper closes on this arm."""
        raw, _ = cw.read_register(self.bus, "Present_Position", "gripper")
        cw.write_register(self.bus, "Goal_Position", "gripper", raw - ticks)
        cw.recover_overload(self.bus, "gripper")

    def hold(self):
        """Keep squeezing (goal fully closed); the torque cap keeps this below overload."""
        cw.stops(self.bus).pop("gripper", None)
        self.bus.sync_write("Goal_Position", {"gripper": 0.0}, normalize=True)
        return self.gripper_pos()

    def joints(self):
        return dict(self.bus.sync_read("Present_Position", list(ALL)))

    def tip(self, joints=None):
        j = joints or self.joints()
        return forward({k: j[k] for k in ARM})

    def move(self, target, seconds):
        """Smooth (smoothstep) move with light bus traffic: the episode recorder shares this serial
        relay, and calibrate_workspace.move (write + read + fault reads every 20 ms) caused
        recorder feedback gaps. Goals are clipped to calibration and known stops; arm joints are
        stall-checked every few steps (held at a physical stop like calibrate_workspace does)."""
        # calibrate_workspace treats gripper contact as a physical stop; that would keep the
        # gripper from ever closing past the last contact point, so forget it every move.
        cw.stops(self.bus).pop("gripper", None)
        target = cw.clamp_target(target, self.bus.calibration)
        start = cw.clamp_target(self.bus.sync_read("Present_Position", list(target)), self.bus.calibration)
        guard = cw.StallGuard(self.bus, target)
        t0 = time.monotonic()
        step = 0
        while True:
            f = min((time.monotonic() - t0) / max(seconds, 1e-3), 1.0)
            a = f * f * (3 - 2 * f)
            goal = cw.within_stops({j: start[j] + a * (target[j] - start[j]) for j in target}, self.bus)
            self.bus.sync_write("Goal_Position", goal, normalize=True)
            if f >= 1.0:
                break
            step += 1
            if step % 5 == 0 and len(guard.joints):
                guard.observe(goal, self.bus.sync_read("Present_Position", list(target)))
            time.sleep(STEP_S)

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
            time.sleep(0.04)

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
