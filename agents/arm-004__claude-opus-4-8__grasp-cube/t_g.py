import sys, json; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, grasp, vision, cv2, time
from camd_client import read_frame
off=float(sys.argv[1]) if len(sys.argv)>1 else grasp.JAW_OFFSET
t,_=read_frame('top'); c=vision.top_cube(t)[0]; cx,cy=vision.pix_to_base([(c['cx'],c['cy'])])[0]; yaw=vision.top_cube_yaw(t,c)
with arm.bus() as b:
    r=grasp.grasp(b,cx,cy,yaw,off=off); print(r)
    w,_=read_frame('wrist'); cv2.imwrite('tools/g_lift.jpg',w); t2,_=read_frame('top'); cv2.imwrite('tools/top.jpg',t2)
