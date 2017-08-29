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

def transformMap(groundTruthFile, saveFile, testPath, saveFolder,hit,miss,unknown):
    testImages = deque()
    testFiles = deque()
    targetPoints = deque()

    cv2.namedWindow('Modified',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Modified',800,600)
    cv2.namedWindow('Original',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Original',800,600)
    cv2.setMouseCallback('Original',mouse_callback)
    # cv2.setMouseCallback('Modified',mouse_callback)
    print "Load Images: "
    print "Loading: %s"%(groundTruthFile)
    gtImage = cv2.imread(groundTruthFile,0)
    print "Loaded: %s"%(groundTruthFile)
    right_clicks.clear()
    cv2.imshow('Original',gtImage)
    minx = miny = width = height = 0
    while True:
        print "Choose 2 points to specify a rectangle!"
        while True:
            key = cv2.waitKey(1)&0xFF
            cv2.imshow('Original',printClicks(gtImage))
            if  key == ord('c'):
                right_clicks.clear()
                print right_clicks
            if key == ord('l'):
                right_clicks.pop()
                print right_clicks
            if key == ord('q'):
                skip = True
                break
            if key == ord('n'):
                if len(right_clicks)==2:
                    print "Rectangle set!"
                    break
                else:
                    print "Expecting exactly 2 points, %i are given!"%len(right_clicks)
        minx = min(right_clicks[0][0],right_clicks[1][0])
        miny = min(right_clicks[0][1],right_clicks[1][1])
        width = abs(right_clicks[0][0]-right_clicks[1][0])
        height = abs(right_clicks[0][1]-right_clicks[1][1])
        gtImageW = gtImage[miny:miny+height,minx:minx+width]
        cv2.imshow('Modified',gtImageW)
        print minx, miny, width, height
        commit = False
        while True:
            key = cv2.waitKey(1)&0xFF
            if key == ord('b'):
                print "Abort!"
                break
            if key == ord('n'):
                commit = True
                print "Rectangle accepted!"
                break
        if commit:
            break
    right_clicks.clear()
    cv2.imwrite(saveFile,gtImageW)
    print "Choose 3 reference points and remember the order!"
    while True:
        key = cv2.waitKey(1)&0xFF
        cv2.imshow('Original',printClicks(gtImage))
        if  key == ord('c'):
            right_clicks.clear()
            print right_clicks
        if key == ord('l'):
            right_clicks.pop()
            print right_clicks
        if key == ord('q'):
            skip = True
            break
        if key == ord('n'):
            if len(right_clicks)==3:
                print "Reference points are set!"
                break
            else:
                print "Expecting exactly 3 points, %i are given!"%len(right_clicks)
    for i in right_clicks:
        targetPoints.append((i[0]-minx,i[1]-miny))
    print targetPoints
    for fileN in os.listdir(testPath):
        skip = False
        if not fileN.endswith(".pgm"):
            continue
        imFile = "%s%s"%(testPath,fileN)
        imFileW = "%s%s"%(saveFolder,fileN)
        print "Loading: %s"%(imFile)
        im = cv2.imread(imFile,0)
        testImages.append(im)
        testFiles.append(ntpath.basename(fileN))
        print "Loaded: %s"%(imFile)
        right_clicks.clear()
        while True:
            print "Select the 3 equivalent points in same order as you did in the reference picture!"
            cv2.imshow('Original',im)
            while True:
                key = cv2.waitKey(1)&0xFF
                cv2.imshow('Original',printClicks(im))
                # if key == ord('1') and len(right_clicks)>0:
                if key == ord('c'):
                    right_clicks.clear()
                    print right_clicks
                if key == ord('l'):
                    right_clicks.pop()
                    print right_clicks
                if key == ord('q'):
                    skip = True
                    break
                if key == ord('n'):
                    if len(right_clicks)==3:
                        print "Reference points are set!"
                        break
                    else:
                        print "Expecting exactly 3 points, %i are given!"%len(right_clicks)
            if skip:
                break
            # transform = cv2.getPerspectiveTransform(np.array(right_clicks,dtype=np.float32),np.array(targetPoints,dtype=np.float32))
            # transform, status = cv2.findHomography(np.array(right_clicks,dtype=np.float32),np.array(targetPoints,dtype=np.float32))
            # imW = cv2.warpPerspective(im,transform,(width,height))
            transform = cv2.getAffineTransform(np.array(right_clicks,dtype=np.float32),np.array(targetPoints,dtype=np.float32))
            imW = cv2.warpAffine(im,transform,(width,height),cv2.INTER_NEAREST)
            imW = imW-hit
            imW = cv2.threshold(imW,unknown-2-hit,255,3)
            # (255-(imW+hit))-(255-miss)
            imW = miss-hit-imW[1]
            imW = cv2.threshold(imW,miss-unknown,255,3)
            # 255-(imw[1]+(255-miss))
            imW = miss-imW[1]
            cv2.imshow('Modified',imW)
            commit = False
            while True:
                key = cv2.waitKey(1)&0xFF
                if key == ord('b'):
                    print "Abort!"
                    break
                if key == ord('n'):
                    print "Reference points accepted!"
                    commit = True
                    break
                if key == ord('q'):
                    skip = True
                    break
            if commit:
                break
            if skip:
                break
        if skip:
            continue
        cv2.imwrite(imFileW,imW)
        right_clicks.clear()
    cv2.destroyAllWindows()

def transformMap2(groundTruthFile, saveFile, testPath, saveFolder, poorMapsfolder,hit,miss,unknown,feature='sift'):
    testImages = deque()
    testFiles = deque()
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
        # flann = cv2.FlannBasedMatcher()
        matches = flann.knnMatch(des2,des1,k=2)
        # matches = flann.knnMatch(des2,des1)
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
        # M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
        # M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC)
        if M!=None:
            # print "Homography found!"
            M = np.asarray(M,np.float32)
            M = cv2.invertAffineTransform(M)
            # print M
            # imW = cv2.warpPerspective(im,M,(width,height),flags=cv2.INTER_NEAREST+cv2.WARP_FILL_OUTLIERS+cv2.WARP_INVERSE_MAP)
            imW = cv2.warpAffine(im,M,(width,height),cv2.INTER_NEAREST)
            imW = imW-hit
            imW = cv2.threshold(imW,unknown-2-hit,255,3)
            # (255-(imW+hit))-(255-miss)
            imW = miss-hit-imW[1]
            imW = cv2.threshold(imW,miss-unknown,255,3)
            # 255-(imw[1]+(255-miss))
            imW = miss-imW[1]
            cv2.imwrite(imFileW,imW)
            # ========
            # matchesMask = mask.ravel().tolist()
            # h,w = im.shape
            # pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
            # dst = cv2.perspectiveTransform(pts,M)
            # try:
            #     img2 = cv2.polylines(gtImage,[np.int32(dst)],True,255,3, cv2.LINE_AA)
            #     draw_params = dict(matchColor = (0,255,0), # draw matches in green color
            #         singlePointColor = None,
            #         matchesMask = matchesMask, # draw only inliers
            #         flags = 2)
            #     img3 = cv2.drawMatches(im,kp2,gtImage,kp1,good,None,**draw_params)
            #     plt.imshow(img3, 'gray'),plt.show()
            #     plt.imshow(imW, 'gray'),plt.show()
            # except:
            #     continue
            #     print "Error"

            # ========
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
