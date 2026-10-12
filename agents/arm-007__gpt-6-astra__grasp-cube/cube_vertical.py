"""Plan a short vertical tool path at fixed pan and pitch using the supplied FK."""
import numpy as np
from scipy.optimize import least_squares
from fk import forward
import cube_cycles as x

def vertical_path(q,height=.08,n=5):
 pitch=sum(q[k] for k in ('shoulder_lift','elbow_flex','wrist_flex'))
 point=forward({**q,'wrist_roll':-30})
 guess=[q['shoulder_lift']-10,q['elbow_flex']-20]
 poses=[]
 for z in np.linspace(point['z']+height,point['z'],n):
  def residual(v):
   p=forward({**q,'shoulder_lift':v[0],'elbow_flex':v[1],'wrist_flex':pitch-v[0]-v[1],'wrist_roll':-30})
   return [p['x']-point['x'],p['y']-point['y'],p['z']-z]
  result=least_squares(residual,guess,bounds=([-80,-100],[90,100]))
  if np.linalg.norm(result.fun)>.001:raise RuntimeError('Vertical pose not solved')
  a,e=result.x;guess=result.x
  poses.append({**q,'shoulder_lift':float(a),'elbow_flex':float(e),'wrist_flex':float(pitch-a-e)})
 return poses

def descend(b,q):
 path=vertical_path(q)
 for p in path:
  clipped=x.c.clamp_target(p,b.calibration)
  if any(abs(clipped[k]-v)>.01 for k,v in p.items()):raise RuntimeError('Vertical path exceeds live calibration')
 x.move(b,path[0],3)
 for p in path[1:]:x.move(b,p,1)
 return path
