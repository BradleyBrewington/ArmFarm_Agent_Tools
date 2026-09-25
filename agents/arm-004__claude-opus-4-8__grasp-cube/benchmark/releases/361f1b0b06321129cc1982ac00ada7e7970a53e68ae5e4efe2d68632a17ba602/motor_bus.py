"""The motor-bus functions used by the original arm-003 ACT runner."""
from contextlib import contextmanager
from act_runtime import JOINTS


@contextmanager
def connected_bus(port, joints=JOINTS):
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus
    if port == "auto":
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        if len(ports) != 1:
            raise ValueError(f"Specify --port: expected one local serial device, found {[p.device for p in ports]}")
        port = ports[0].device
    if not joints or not set(joints).issubset(JOINTS):
        raise ValueError("Specify one or more known joints")
    bus = FeetechMotorsBus(port=port, motors={
        j: Motor(i + 1, "sts3215", MotorNormMode.RANGE_0_100 if j == "gripper" else MotorNormMode.DEGREES)
        for i, j in enumerate(JOINTS) if j in joints})
    try:
        bus.connect()
        bus.calibration = bus.read_calibration()
        for j in joints:
            c = bus.calibration[j]
            if not 0 <= c.range_min < c.range_max <= 4095:
                raise ValueError(f"Invalid live motor calibration for {j}")
        yield bus
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def read_register(bus, name, joint):
    motor = bus.motors[joint]
    addr, length = bus.model_ctrl_table[motor.model][name]
    value, comm, error = bus._read(addr, length, motor.id, num_retry=2, raise_on_error=False)
    if not bus._is_comm_success(comm):
        raise ConnectionError(f"{joint}: could not read {name}: {bus.packet_handler.getTxRxResult(comm)}")
    return value, error


def write_register(bus, name, joint, value):
    motor = bus.motors[joint]
    addr, length = bus.model_ctrl_table[motor.model][name]
    comm, error = bus._write(addr, length, motor.id, int(value), num_retry=2, raise_on_error=False)
    if not bus._is_comm_success(comm):
        raise ConnectionError(f"{joint}: could not write {name}: {bus.packet_handler.getTxRxResult(comm)}")
    return error
