"""No-motion two-camera diagnostic with before/after motor feedback."""
import time,json,cv2,numpy as np
from pathlib import Path
from episode_motion import bus_open,state
import camd_client
root=Path('tools/no_motion_'+str(time.time_ns()));root.mkdir()
with bus_open() as b:
 before=state(b);frames=[]
 for i in range(12):
  rec={'time':time.time()}
  for role in ['top','wrist']:
   data,meta=camd_client.read_jpeg(role);p=root/f'{i:02}_{role}.jpg';p.write_bytes(data)
   rec[role]={'image':str(p),'metadata':meta}
  frames.append(rec);time.sleep(.3)
 after=state(b)
def track(role,roi):
 x,y,w,h=roi;ref=cv2.imread(frames[0][role]['image'],0)[y:y+h,x:x+w];out=[]
 for f in frames:
  img=cv2.imread(f[role]['image'],0);patch=img[y-30:y+h+30,x-30:x+w+30]
  _,score,_,loc=cv2.minMaxLoc(cv2.matchTemplate(patch,ref,cv2.TM_CCOEFF_NORMED));out.append({'shift_px':[loc[0]-30,loc[1]-30],'score':score})
 return out
result=dict(diagnostic='No motion commands; torque unchanged',before=before,after=after,frames=frames,top_mug=track('top',(475,260,220,220)),top_table_edge=track('top',(35,280,100,300)),top_arm=track('top',(415,35,155,180)))
(root/'result.json').write_text(json.dumps(result,indent=2));print(root);print(json.dumps({k:result[k] for k in ['top_mug','top_table_edge','top_arm']}));print('joint_delta', {j:after['q'][j]-before['q'][j] for j in before['q']})
