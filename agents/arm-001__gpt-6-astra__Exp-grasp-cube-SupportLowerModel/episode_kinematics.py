import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from fk import URDF,JOINTS
root=ET.fromstring(URDF);bychild={j.find('child').get('link'):j for j in root.findall('joint')}
chain=[];link='gripper_frame_link'
while link!='base_link':
 j=bychild[link];chain.insert(0,j);link=j.find('parent').get('link')
fixed=[]
for j in chain:
 o=j.find('origin');T=np.eye(4);T[:3,:3]=Rotation.from_euler('xyz',np.fromstring(o.get('rpy','0 0 0'),sep=' ')).as_matrix();T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ')
 fixed.append((j.get('name'),T,np.fromstring(j.find('axis').get('xyz'),sep=' ') if j.get('type')!='fixed' else None))
def matrix(q):
 T=np.eye(4)
 for j,F,a in fixed:
  T=T@F
  if a is not None:
   M=np.eye(4);M[:3,:3]=Rotation.from_rotvec(a*np.deg2rad(q[j])).as_matrix();T=T@M
 return T
def solve(xyz,start,roll=56,direction=None):
 names=JOINTS[:4];lo=[-105,-100,-95,-95];hi=[105,100,97,95]
 def residual(v):
  q=dict(zip(names,v));q['wrist_roll']=roll;T=matrix(q)
  r=list((T[:3,3]-xyz)*100)
  if direction is not None:r.extend((T[:3,2]-direction)*3)
  return r
 opt=least_squares(residual,[start[j] for j in names],bounds=(lo,hi),max_nfev=300)
 q=dict(zip(names,opt.x));q['wrist_roll']=roll;T=matrix(q)
 return {'q':q,'xyz':T[:3,3].tolist(),'direction':T[:3,2].tolist(),'error':float(np.linalg.norm(T[:3,3]-xyz))}
if __name__=='__main__':
 import json,sys
 q=json.loads(open('/tmp/arm_moves.jsonl').readlines()[-1])['q']
 print('current',matrix(q).tolist())
 for p in [[.29,.005,.06],[.30,.005,-.025],[.30,.005,-.06]]:
  print(solve(np.array(p),q,direction=np.array([0,0,-1])))
