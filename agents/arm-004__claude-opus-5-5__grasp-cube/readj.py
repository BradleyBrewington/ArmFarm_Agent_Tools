import sys, os, json
sys.path.insert(0, 'tools')
import calibrate_workspace as cw
with cw.connected_bus(os.environ['ARMFARM_SERIAL_PORT']) as bus:
    print(json.dumps(bus.sync_read("Present_Position"), indent=1))
    print(json.dumps(bus.sync_read("Torque_Enable", normalize=False)))
    print({j: (c.range_min, c.range_max) for j, c in bus.calibration.items()})
