"""Fit table->robot map from fingertip observations at known robot XY (closed gripper, near table)."""
import json, time, cv2, numpy as np
import arm, vision as v
from snap import snap

PTS = [(0.16, -0.12), (0.16, 0.0), (0.16, 0.12), (0.22, -0.12), (0.22, 0.0), (0.22, 0.12), (0.26, 0.10), (0.26, -0.10)]
Z = 0.025
obs = []
with arm.bus() as b:
    arm.gripper(b, 0)
    for i, (x, y) in enumerate(PTS):
        arm.goto(b, x, y, 0.07, roll=83, speed=60)
        arm.goto(b, x, y, Z, roll=83, speed=40, correct=2)
        time.sleep(0.4)
        p = arm.tip(b); img = snap("top"); t = v.arm_tip_px(img)
        cv2.imwrite(f"/tmp/ct{i}.jpg", img)
        obs.append({"robot": p.tolist(), "px": t})
        print(i, p.round(4), t, flush=True)
        arm.goto(b, x, y, 0.07, roll=83, speed=60, correct=0)
    arm.home(b, speed=60)
tab = [v.px_to_table([o["px"]], o["robot"][2])[0] for o in obs]
m = v.fit_rigid(tab, [o["robot"][:2] for o in obs])
m["source"] = "fingertip"; m["obs"] = obs
print(json.dumps({k: m[k] for k in ("A", "b", "reflect", "residual_mm")}, indent=1))
v.ROBOT_MAP.write_text(json.dumps(m, indent=1))
