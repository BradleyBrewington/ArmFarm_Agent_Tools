import sys; sys.path.insert(0,'work'); sys.path.insert(0,'tools'); import arm, servo, time, vision, kin, numpy as np
from camd_client import read_frame
import cv2
with arm.bus() as b:
    for i,r in enumerate([83.4, 93.4, 73.4]):
        arm.goto(b,0.35,0.11,0.08,roll=r,speed=40,correct=2); time.sleep(0.3)
        img,_=read_frame('wrist'); d=vision.wrist_cube(img); cv2.imwrite(f'tools/roll{i}.jpg',img)
        T=kin.pose(arm.joints(b)); print(r, d and (round(d['cx']),round(d['cy']),round(d['angle'],1)), 'toolx',T[:3,0].round(3),'tooly',T[:3,1].round(3), arm.joints(b)['shoulder_pan'])
