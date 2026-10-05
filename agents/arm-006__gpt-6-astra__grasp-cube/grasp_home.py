from pathlib import Path
import time,json
import calibrate_workspace as c,recording
from task_motion import snap
with c.connected_bus(recording.serial_port()) as b:
 g=c.AdaptiveGrip(b);b.adaptive_grip=g
 try:
  print(g.acquire(),flush=True)
  lift={'shoulder_lift':-35,'elbow_flex':65,'wrist_flex':-30,'gripper':0}
  c.move(b,lift,3);c.settle(b,lift,.4)
  home=c.load_home(Path('home_pose.json'))
  c.enable_at_current_position(b,list(home));c.move(b,home,3);print(c.settle(b,home,.5),flush=True)
  snap();print('AT_HOME_REQUIRES_VISUAL_CONFIRMATION',flush=True)
  for _ in range(50):g.tick();time.sleep(.1)
  print(g.sample(),flush=True)
 except Exception:
  g.stop();snap();raise
