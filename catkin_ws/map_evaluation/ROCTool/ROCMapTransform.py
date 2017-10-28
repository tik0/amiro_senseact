import os
import numpy as np
import cv2
from collections import deque
import ntpath
from matplotlib import pyplot as plt

right_clicks = deque()

def mouse_callback(event, x, y, flags, params):
    if event == 7:
        global right_clicks
        right_clicks.append((x,y))
        print right_clicks

def printClicks(im):
    imC = cv2.cvtColor(im, cv2.COLOR_GRAY2BGR)
    for p in right_clicks:
        cv2.circle(imC,p,1,(0,0,255),1)
        cv2.circle(imC,p,8,(255,0,0),2)
    return imC

def transformMap(groundTruthFile, saveFile, testPath, saveFolder, poorMapsfolder,hit,miss,unknown,feature='sift'):
    if feature == 'orb':
        orb = cv2.ORB_create()
    if feature == 'sift':
        sift = cv2.xfeatures2d.SIFT_create()

    print "Load Images: "
    print "Loading: %s"%(groundTruthFile)
    gtImage = cv2.imread(groundTruthFile,0)
    print "Loaded: %s"%(groundTruthFile)
    cv2.imwrite(saveFile,gtImage)
    height, width = gtImage.shape
    if feature == 'orb':
        kp1, des1 = orb.detectAndCompute(gtImage,None)
    if feature == 'sift':
        kp1, des1 = sift.detectAndCompute(gtImage,None)
    des1 = np.asarray(des1,np.float32)


    for fileN in os.listdir(testPath):
        if not fileN.endswith(".pgm"):
            continue
        imFile = "%s%s"%(testPath,fileN)
        imFileW = "%s%s"%(saveFolder,fileN)
        imFileP = "%s%s"%(poorMapsfolder,fileN)
        print "Loading: %s"%(imFile)
        im = cv2.imread(imFile,0)
        print "Loaded!"
        if feature == 'orb':
            kp2, des2 = orb.detectAndCompute(im,None)
        if feature == 'sift':
            kp2, des2 = sift.detectAndCompute(im,None)
        des2 = np.asarray(des2,np.float32)
        print "Featur points detected."
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des2,des1,k=2)
        print "Points matched."

        good = deque()
        for m,n in matches:
            if m.distance < 0.7*n.distance:
                good.append(m)
        print len(good)
        if len(good)<1:
            print "Not enough points!"
            cv2.imwrite(imFileP,im)
            continue
        dst_pts = np.float32([ kp2[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
        src_pts = np.float32([ kp1[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
        M = cv2.estimateRigidTransform(src_pts, dst_pts, False)
        if M!=None:
            M = np.asarray(M,np.float32)
            M = cv2.invertAffineTransform(M)
            imW = cv2.warpAffine(im,M,(width,height),cv2.INTER_NEAREST)
            imW = imW-hit
            imW = cv2.threshold(imW,unknown-2-hit,255,3)
            # (255-(imW+hit))-(255-miss)
            imW = miss-hit-imW[1]
            imW = cv2.threshold(imW,miss-unknown,255,3)
            # 255-(imw[1]+(255-miss))
            imW = miss-imW[1]
            cv2.imwrite(imFileW,imW)
        else:
            print "No homography found!"
            cv2.imwrite(imFileP,im)




'''
        # transform = cv2.getAffineTransform(np.array(right_clicks,dtype=np.float32),np.array(targetPoints,dtype=np.float32))
        imW = cv2.warpAffine(im,transform,(width,height),cv2.INTER_NEAREST)
        imW = imW-hit
        imW = cv2.threshold(imW,unknown-2-hit,255,3)
        # (255-(imW+hit))-(255-miss)
        imW = miss-hit-imW[1]
        imW = cv2.threshold(imW,miss-unknown,255,3)
        # 255-(imw[1]+(255-miss))
        imW = miss-imW[1]
        cv2.imwrite(imFileW,imW)
'''
