import cv2
import numpy as np
import yaml
import sys
from collections import deque

myFile = "amiro_sick_2_ekf.txt"
fileContent = deque()
objs = None
with open(myFile) as stream:
    data = stream.read()
    objs = data.split('---')

for obj in objs:
    fileContent.append(yaml.load(obj))

im = cv2.imread('./../gt_creator/gt_maps/gt_wallthickness_15_resolution_0.05.png')

for cont in fileContent:
    if cont != None:
        cv2.circle(im,(int(round(180+cont['pose']['pose']['position']['x']/0.019)),int(round(170-cont['pose']['pose']['position']['y']/0.019))),1,(154,198,0),-1)
        cv2.imshow('trajectory',im)
        cv2.waitKey(1)

cv2.imshow('trajectory',im)
cv2.imwrite('amiro_sick_2_ekf.png',im)
cv2.waitKey(0)
cv2.destroyAllWindow()
