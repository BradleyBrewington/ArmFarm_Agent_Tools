import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, servo, time, cv2, vision, numpy as np
from camd_client import read_frame
x,y,z=map(float,sys.argv[1:4]); roll=float(sys.argv[4]) if len(sys.argv)>4 else None
with arm.bus() as b:
    arm.gripper(b,70)
    if roll is None:
        arm.goto(b,x,y,0.08,speed=40,correct=2); time.sleep(0.3)
        d,_=servo.detect(); roll=servo.ROLL_CENTER-d['angle']/servo.ROLL_GAIN; print('angle',d['angle'],'roll',roll)
    arm.goto(b,x,y,0.08,roll=roll,speed=40,correct=2); time.sleep(0.3)
    d,w8=servo.detect(); print('hover px',d and (d['cx'],d['cy'],d['angle']))
    arm.goto(b,x,y,z,roll=roll,speed=25,correct=3); time.sleep(0.4)
    print('tool',arm.tool_xyz(b).round(4))
    w,_=read_frame('wrist'); t,_=read_frame('top')
    cv2.imwrite('tools/al_w8.jpg',w8); cv2.imwrite('tools/al_w.jpg',w); cv2.imwrite('tools/al_t.jpg',t)
    c=vision.top_cube(t); cx,cy=(int(c[0]['cx']),int(c[0]['cy'])) if c else (900,300)
    crop=t[max(0,cy-200):cy+150, max(0,cx-200):cx+200]
    cv2.imwrite('tools/al_pair.jpg',np.hstack([cv2.resize(w,(560,315)),cv2.resize(crop,(int(crop.shape[1]*315/crop.shape[0]),315))]))
    arm.goto(b,x,y,0.08,roll=roll,speed=30,correct=0)
