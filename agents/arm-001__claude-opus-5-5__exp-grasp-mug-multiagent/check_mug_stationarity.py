"""Read-only camera drift measurement; no motor commands."""
import cv2,time,json
from pathlib import Path
import camd_client
root=Path('tools/stationarity_'+str(time.time_ns()));root.mkdir()
frames=[]
for i in range(5):
 b,m=camd_client.read_jpeg('top');(root/f'{i}_top.jpg').write_bytes(b)
 frames.append(cv2.imdecode(__import__('numpy').frombuffer(b,dtype='uint8'),cv2.IMREAD_GRAYSCALE))
 time.sleep(1)
r=[]
for i,f in enumerate(frames):
 vals={}
 for name,(x,y,w,h) in {'mug':(475,260,205,210),'table_edge':(30,280,100,300)}.items():
  template=frames[0][y:y+h,x:x+w];search=f[max(0,y-20):y+h+20,max(0,x-20):x+w+20]
  _,score,_,loc=cv2.minMaxLoc(cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED))
  vals[name]={'shift_px':[loc[0]-20,loc[1]-20],'score':score}
 r.append(vals)
(root/'measurements.json').write_text(json.dumps(r,indent=2));print(root);print(json.dumps(r))
