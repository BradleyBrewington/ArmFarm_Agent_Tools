"""Record a broad visible-table candidate grid; never certify reach from visibility."""
import json,cv2,numpy as np,datetime
from pathlib import Path
c=json.load(open('calibration/20260918T214001_327909148/workspace_map.json'))
p=np.array([[125,50],[1080,50],[1270,710],[25,710]],float)
u=cv2.undistortPoints(p.reshape(-1,1,2),np.array(c['camera_matrix']),np.array(c['dist_coeffs']),P=np.array(c['camera_matrix'])).reshape(-1,2)
h=np.c_[u,np.ones(len(u))]@np.array(c['homography_undistorted_pixel_to_table_m']).T
poly=h[:,:2]/h[:,2:]
cells=[]
for i in range(19):
 for j in range(12):
  x=-.35+i*.05;y=-.30+j*.05
  cells.append(dict(id=f'table_{i:02}_{j:02}',bounds_m=[x,x+.05,y,y+.05],status='untested_geometry_and_grasp',center_in_visible_polygon=bool(cv2.pointPolygonTest(poly.astype(np.float32),(x+.025,y+.025),False)>=0)))
out=dict(updated_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),frame='checkerboard_table',cell_size_m=.05,bounds_m={'x':[-.35,.60],'y':[-.30,.30]},visible_table_polygon_raw_px=p.tolist(),visible_table_polygon_m=poly.tolist(),calibration='calibration/20260918T214001_327909148/workspace_map.json',evidence='tools/mug_observations/1790194356873441986_top.jpg',status='candidate survey only; edge extrapolation, obstacles, reach and camera stability require verification',accepted_usable_mask=None,verified_cells=[],cells=cells)
Path('tools/mug_table_grid.json').write_text(json.dumps(out,indent=2))
print('Recorded',len(cells),'untested cells')
