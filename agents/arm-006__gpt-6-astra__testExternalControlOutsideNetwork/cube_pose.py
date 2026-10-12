import sys,json
import fk
from scipy.optimize import least_squares

def pose(x,z,pan):
 def err(a):
  p=fk.forward(dict(zip(fk.JOINTS,[pan,*a,0])));return [p['x']-x,p['z']-z,(sum(a)-90)/300]
 r=least_squares(err,[10,10,70],bounds=([-95,-95,-95],[95,95,100]))
 if max(abs(v) for v in err(r.x))>.003:raise ValueError('Pose unreachable')
 return dict(zip(fk.JOINTS[:4],[pan,*map(float,r.x)]))
if __name__=='__main__': print(json.dumps(pose(*map(float,sys.argv[1:]))))
