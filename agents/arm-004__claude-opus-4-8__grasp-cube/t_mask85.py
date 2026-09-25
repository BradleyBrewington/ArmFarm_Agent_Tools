import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, time, cv2, numpy as np
from camd_client import read_frame
ims=[]
with arm.bus() as b:
    arm.gripper(b,85)
    for (x,y,r) in [(0.16,0.03,90.0),(0.15,-0.02,60.0)]:
        arm.goto(b,x,y,0.06,roll=r,speed=40,correct=0)
        arm.goto(b,x,y,-0.005,roll=r,speed=20,correct=2); time.sleep(0.4)
        w,_=read_frame('wrist'); ims.append(cv2.GaussianBlur(cv2.cvtColor(w,cv2.COLOR_BGR2GRAY),(5,5),0)); cv2.imwrite(f'tools/m85_{len(ims)}.jpg',w)
        arm.goto(b,x,y,0.07,roll=r,speed=30,correct=0)
m=((ims[0]<80)&(ims[1]<80)).astype(np.uint8); m=cv2.dilate(m,np.ones((25,25),np.uint8))
cv2.imwrite('tools/jaw_mask85.png',m*255); print('mask frac',m.mean())
