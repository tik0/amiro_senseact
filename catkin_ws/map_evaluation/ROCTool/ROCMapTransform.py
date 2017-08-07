import os
import numpy as np
import cv2
import math
from collections import deque
import ntpath
from time import sleep
import ROCPlot

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
