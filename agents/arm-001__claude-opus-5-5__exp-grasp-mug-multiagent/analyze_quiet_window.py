"""Read-only image analysis for stationary mug/table/arm sequences."""
import json,sys,cv2,numpy as np
from pathlib import Path

def analyze(path):
 r=json.load(open(Path(path)/'result.json'));frames=r['frames'];out=[]
 refs={}
 rois={'table_edge':(35,280,100,300),'arm':(415,35,155,180)}
 for label,(x,y,w,h) in rois.items():refs[label]=cv2.imread(frames[0]['top']['image'],0)[y:y+h,x:x+w]
 for f in frames:
  gray=cv2.imread(f['top']['image'],0);x,y,w,h=(470,250,240,230);roi=gray[y:y+h,x:x+w];mask=(roi<65).astype('uint8');n,lab,stats,cents=cv2.connectedComponentsWithStats(mask)
  i=1+np.argmax(stats[1:,4]);yy,xx=np.where(lab==i);pts=np.column_stack([xx,yy]);vals,vec=np.linalg.eigh(np.cov(pts.T));v=vec[:,-1];angle=float(np.degrees(np.arctan2(v[1],v[0])))
  entry={'time':f['time'],'centroid_px':(cents[i]+[x,y]).tolist(),'dark_area_px':int(stats[i,4]),'silhouette_axis_deg':angle,'bbox_px':(stats[i,:4]+[x,y,0,0]).tolist()}
  for label,(x,y,w,h) in rois.items():
   _,score,_,loc=cv2.minMaxLoc(cv2.matchTemplate(gray[y-25:y+h+25,x-25:x+w+25],refs[label],cv2.TM_CCOEFF_NORMED));entry[label]={'shift_px':[loc[0]-25,loc[1]-25],'score':score}
  out.append(entry)
 cent=np.array([a['centroid_px'] for a in out]);return {'frames':out,'centroid_range_xy_px':np.ptp(cent,axis=0).tolist(),'centroid_max_distance_from_first_px':float(np.linalg.norm(cent-cent[0],axis=1).max()),'centroid_std_xy_px':cent.std(axis=0).tolist(),'segmentation':'Largest dark component grayscale<65 in top ROI470,250,240,230; silhouette includes handle, excludes most shadow. Manual images reviewed separately.'}
p=Path(sys.argv[1]);result={'quiet':analyze(p),'earlier':analyze('tools/no_motion_1790196095155360337')};(p/'comparison.json').write_text(json.dumps(result,indent=2));print(json.dumps({k:{j:v for j,v in a.items() if j!='frames'} for k,a in result.items()}))
