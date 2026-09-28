#!/usr/bin/env python3
"""python ik.py X Y Z: base_link metres in, five arm joint angles in degrees out."""
import contextlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shlex
import subprocess
import sys
import xml.etree.ElementTree as ET

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
HOST = "protopi5"
PI_PYTHON = "/home/protopi5/miniforge3/envs/lerobot/bin/python"

URDF = '<robot name="so101_new_calib">\n\n  \n  <material name="3d_printed">\n    <color rgba="1.0 0.82 0.12 1.0" />\n  </material>\n  <material name="sts3215">\n    <color rgba="0.1 0.1 0.1 1.0" />\n  </material>\n\n  \n  <link name="base_link">\n    <inertial>\n      <origin xyz="0.0137179 -5.19711e-05 0.0334843" rpy="0 0 0" />\n      <mass value="0.147" />\n      <inertia ixx="0.000114686" ixy="-4.59787e-07" ixz="4.97151e-06" iyy="0.000136117" iyz="9.75275e-08" izz="0.000130364" />\n    </inertial>\n    \n    </link>\n\n  \n  <link name="shoulder_link">\n    <inertial>\n      <origin xyz="-0.0307604 -1.66727e-05 -0.0252713" rpy="0 0 0" />\n      <mass value="0.100006" />\n      <inertia ixx="8.3759e-05" ixy="7.55525e-08" ixz="-1.16342e-06" iyy="8.10403e-05" iyz="1.54663e-07" izz="2.39783e-05" />\n    </inertial>\n    \n    </link>\n\n  \n  <link name="upper_arm_link">\n    <inertial>\n      <origin xyz="-0.0898471 -0.00838224 0.0184089" rpy="0 0 0" />\n      <mass value="0.103" />\n      <inertia ixx="4.08002e-05" ixy="-1.97819e-05" ixz="-4.03016e-08" iyy="0.000147318" iyz="8.97326e-09" izz="0.000142487" />\n    </inertial>\n    \n    </link>\n\n  \n  <link name="lower_arm_link">\n    <inertial>\n      <origin xyz="-0.0980701 0.00324376 0.0182831" rpy="0 0 0" />\n      <mass value="0.104" />\n      <inertia ixx="2.87438e-05" ixy="7.41152e-06" ixz="1.26409e-06" iyy="0.000159844" iyz="-4.90188e-08" izz="0.00014529" />\n    </inertial>\n    \n    </link>\n\n  \n  <link name="wrist_link">\n    <inertial>\n      <origin xyz="-0.000103312 -0.0386143 0.0281156" rpy="0 0 0" />\n      <mass value="0.079" />\n      <inertia ixx="3.68263e-05" ixy="1.7893e-08" ixz="-5.28128e-08" iyy="2.5391e-05" iyz="3.6412e-06" izz="2.1e-05" />\n    </inertial>\n    \n    </link>\n\n  \n  <link name="gripper_link">\n    <inertial>\n      <origin xyz="0.000213627 0.000245138 -0.025187" rpy="0 0 0" />\n      <mass value="0.087" />\n      <inertia ixx="2.75087e-05" ixy="-3.35241e-07" ixz="-5.7352e-06" iyy="4.33657e-05" iyz="-5.17847e-08" izz="3.45059e-05" />\n    </inertial>\n    \n    </link>\n\n  \n  <link name="gripper_frame_link">\n    <origin xyz="0 0 0" rpy="0 -0 0" />\n    <inertial>\n      <origin xyz="0 0 0" rpy="0 0 0" />\n      <mass value="1e-9" />\n      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" />\n    </inertial>\n  </link>\n\n  <joint name="gripper_frame_joint" type="fixed">\n    <origin xyz="-0.0079 -0.000218121 -0.0981274" rpy="0 3.14159 0" />\n    <parent link="gripper_link" />\n    <child link="gripper_frame_link" />\n    <axis xyz="0 0 0" />\n  </joint>\n\n  \n  <link name="moving_jaw_so101_v1_link">\n    <inertial>\n      <origin xyz="-0.00157495 -0.0300244 0.0192755" rpy="0 0 0" />\n      <mass value="0.012" />\n      <inertia ixx="6.61427e-06" ixy="-3.19807e-07" ixz="-5.90717e-09" iyy="1.89032e-06" iyz="-1.09945e-07" izz="5.28738e-06" />\n    </inertial>\n    \n    </link>\n\n  \n  <joint name="gripper" type="revolute">\n    <origin xyz="0.0202 0.0188 -0.0234" rpy="1.5708 -5.24284e-08 -1.41553e-15" />\n    <parent link="gripper_link" />\n    <child link="moving_jaw_so101_v1_link" />\n    <axis xyz="0 0 1" />\n    <limit effort="10" velocity="10" lower="-0.174533" upper="1.74533" />\n  </joint>\n\n  <transmission name="gripper_trans">\n    <type>transmission_interface/SimpleTransmission</type>\n    <joint name="gripper">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n    </joint>\n    <actuator name="motor6">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n      <mechanicalReduction>1</mechanicalReduction>\n    </actuator>\n  </transmission>\n\n  \n  <joint name="wrist_roll" type="revolute">\n    <origin xyz="5.55112e-17 -0.0611 0.0181" rpy="1.5708 0.0486795 3.14159" />\n    <parent link="wrist_link" />\n    <child link="gripper_link" />\n    <axis xyz="0 0 1" />\n    <limit effort="10" velocity="10" lower="-2.74385" upper="2.84121" />\n  </joint>\n\n  <transmission name="wrist_roll_trans">\n    <type>transmission_interface/SimpleTransmission</type>\n    <joint name="wrist_roll">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n    </joint>\n    <actuator name="motor5">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n      <mechanicalReduction>1</mechanicalReduction>\n    </actuator>\n  </transmission>\n\n  \n  <joint name="wrist_flex" type="revolute">\n    <origin xyz="-0.1349 0.0052 3.62355e-17" rpy="4.02456e-15 8.67362e-16 -1.5708" />\n    <parent link="lower_arm_link" />\n    <child link="wrist_link" />\n    <axis xyz="0 0 1" />\n    <limit effort="10" velocity="10" lower="-1.65806" upper="1.65806" />\n  </joint>\n\n  <transmission name="wrist_flex_trans">\n    <type>transmission_interface/SimpleTransmission</type>\n    <joint name="wrist_flex">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n    </joint>\n    <actuator name="motor4">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n      <mechanicalReduction>1</mechanicalReduction>\n    </actuator>\n  </transmission>\n\n  \n  \n  <joint name="elbow_flex" type="revolute">\n    <origin xyz="-0.11257 -0.028 1.73763e-16" rpy="-3.63608e-16 8.74301e-16 1.5708" />\n    <parent link="upper_arm_link" />\n    <child link="lower_arm_link" />\n    <axis xyz="0 0 1" />\n    <limit effort="10" velocity="10" lower="-1.69" upper="1.69" />\n  </joint>\n\n  <transmission name="elbow_flex_trans">\n    <type>transmission_interface/SimpleTransmission</type>\n    <joint name="elbow_flex">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n    </joint>\n    <actuator name="motor3">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n      <mechanicalReduction>1</mechanicalReduction>\n    </actuator>\n  </transmission>\n\n  \n  <joint name="shoulder_lift" type="revolute">\n    <origin xyz="-0.0303992 -0.0182778 -0.0542" rpy="-1.5708 -1.5708 0" />\n    <parent link="shoulder_link" />\n    <child link="upper_arm_link" />\n    <axis xyz="0 0 1" />\n    <limit effort="10" velocity="10" lower="-1.74533" upper="1.74533" />\n  </joint>\n\n  <transmission name="shoulder_lift_trans">\n    <type>transmission_interface/SimpleTransmission</type>\n    <joint name="shoulder_lift">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n    </joint>\n    <actuator name="motor2">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n      <mechanicalReduction>1</mechanicalReduction>\n    </actuator>\n  </transmission>\n\n  \n  <joint name="shoulder_pan" type="revolute">\n    <origin xyz="0.0388353 -8.97657e-09 0.0624" rpy="3.14159 4.18253e-17 -3.14159" />\n    <parent link="base_link" />\n    <child link="shoulder_link" />\n    <axis xyz="0 0 1" />\n    <limit effort="10" velocity="10" lower="-1.91986" upper="1.91986" />\n  </joint>\n\n  <transmission name="shoulder_pan_trans">\n    <type>transmission_interface/SimpleTransmission</type>\n    <joint name="shoulder_pan">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n    </joint>\n    <actuator name="motor1">\n      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>\n      <mechanicalReduction>1</mechanicalReduction>\n    </actuator>\n  </transmission> \n\n</robot>'


def remote(function, arguments):
    payload = {"source": Path(__file__).read_text(encoding="utf-8"), "function": function, "arguments": arguments}
    runner = "import contextlib,json,sys; d=json.load(sys.stdin); n={'__name__':'remote_tool','__file__':'standalone.py'}; exec(d['source'],n); " + "\nwith contextlib.redirect_stdout(sys.stderr): r=n[d['function']](*d['arguments'])\nprint(json.dumps(r,allow_nan=False))"
    result = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10", HOST,
                             shlex.join([PI_PYTHON, "-c", runner])], input=json.dumps(payload),
                            capture_output=True, text=True, timeout=60)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or "Pi execution failed")
    return json.loads(result.stdout)


def solve(x, y, z):
    import numpy as np
    import placo

    target = np.asarray([x, y, z], dtype=float)
    if not np.isfinite(target).all():
        raise ValueError("Coordinates must be finite metres")
    limits = {}
    for joint in ET.fromstring(URDF).findall("joint"):
        if joint.get("name") in JOINTS:
            lim = joint.find("limit")
            limits[joint.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    robot = placo.RobotWrapper(".", placo.Flags.ignore_collisions, URDF)
    solver = placo.KinematicsSolver(robot)
    solver.mask_fbase(True)
    solver.mask_dof("gripper")
    solver.enable_joint_limits(True)
    solver.enable_velocity_limits(True)
    solver.dt = .05
    for joint in JOINTS:
        robot.set_velocity_limit(joint, 1.5)
    task = solver.add_position_task("gripper_frame_link", target)
    task.configure("position", "soft", 1.)
    solver.add_regularization_task(1e-7)
    pan = math.degrees(-math.atan2(y, x - .0388353))
    starts = [[pan, -30, 45, 45, 0], [pan, 60, -60, 30, 0],
              [pan, 0, 0, 0, 0], [pan, -60, 60, 90, 0]]
    for start in starts:
        for joint, degrees in zip(JOINTS, start):
            robot.set_joint(joint, float(np.clip(math.radians(degrees), *limits[joint])))
        robot.set_joint("gripper", 0.)
        robot.update_kinematics()
        for _ in range(300):
            solver.solve(True)
            robot.update_kinematics()
            position = robot.get_T_world_frame("gripper_frame_link")[:3, 3]
            angles = {j: math.degrees(robot.get_joint(j)) for j in JOINTS}
            if not np.isfinite(list(angles.values())).all():
                break
            if float(np.linalg.norm(position - target)) <= .001:
                return angles
    raise ValueError("Target is unreachable within SO101 joint limits")


def main():
    if len(sys.argv) != 4:
        raise ValueError("Usage: python ik.py X Y Z (metres in base_link)")
    xyz = [float(v) for v in sys.argv[1:]]
    if not all(math.isfinite(v) for v in xyz):
        raise ValueError("Coordinates must be finite metres")
    with contextlib.redirect_stdout(sys.stderr):
        result = solve(*xyz) if importlib.util.find_spec("placo") else remote("solve", xyz)
    print(json.dumps(result, allow_nan=False))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
