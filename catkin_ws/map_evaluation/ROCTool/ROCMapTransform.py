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

def transformMap(groundTruthFile, saveFile, testPath, saveFolder):
    testImages = deque()
    testFiles = deque()
    # ROCs = deque()
    targetPoints = deque()
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image',800,600)
    cv2.setMouseCallback('image',mouse_callback)
    print "Load Images: "
    print "Loading: %s"%(groundTruthFile)
    gtImage = cv2.imread(groundTruthFile,0)
    print "Loaded: %s"%(groundTruthFile)
    right_clicks.clear()
    cv2.imshow('image',gtImage)
    minx = miny = width = height = 0
    while True:
        while True:
            key = cv2.waitKey(1)&0xFF
            cv2.imshow('image',printClicks(gtImage))
            if  key == ord('c'):
                right_clicks.clear()
                print right_clicks
            if key == ord('l'):
                right_clicks.pop()
                print right_clicks
            if key == ord('q'):
                skip = True
                break
            if key == ord('n') and len(right_clicks)==2:
                break
        minx = min(right_clicks[0][0],right_clicks[1][0])
        miny = min(right_clicks[0][1],right_clicks[1][1])
        width = abs(right_clicks[0][0]-right_clicks[1][0])
        height = abs(right_clicks[0][1]-right_clicks[1][1])
        gtImage = gtImage[miny:miny+height,minx:minx+width]
        cv2.imshow('image',gtImage)
        print minx, miny, width, height
        commit = False
        while True:
            key = cv2.waitKey(1)&0xFF
            if key == ord('b'):
                break
            if key == ord('n'):
                commit = True
                break
        if commit:
            break
    cv2.imwrite(saveFile,gtImage)
    right_clicks.clear()
    while True:
        key = cv2.waitKey(1)&0xFF
        cv2.imshow('image',printClicks(gtImage))
        if  key == ord('c'):
            right_clicks.clear()
            print right_clicks
        if key == ord('l'):
            right_clicks.pop()
            print right_clicks
        if key == ord('q'):
            skip = True
            break
        if key == ord('n') and len(right_clicks)==4:
            break
    for i in right_clicks:
        targetPoints.append((i[0],i[1]))
    right_clicks.clear()
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
        cv2.imshow('image',im)
        while True:
            while True:
                key = cv2.waitKey(1)&0xFF
                cv2.imshow('image',printClicks(im))
                if  key == ord('c'):
                    right_clicks.clear()
                    print right_clicks
                if key == ord('l'):
                    right_clicks.pop()
                    print right_clicks
                if key == ord('q'):
                    skip = True
                    break
                if key == ord('n') and len(right_clicks)==4:
                    break
            if skip:
                continue
            transform = cv2.getPerspectiveTransform(np.array(right_clicks,dtype=np.float32),np.array(targetPoints,dtype=np.float32))
            im = cv2.warpPerspective(im,transform,(width,height),cv2.INTER_NEAREST)
            cv2.imshow('image',im)
            commit = False
            while True:
                key = cv2.waitKey(1)&0xFF
                if key == ord('b'):
                    break
                if key == ord('n'):
                    commit = True
                    break
            if commit:
                break
        cv2.imwrite(imFileW,im)
        # fpr, fnr = calcROC(gtImage,im,free,occupied)
        # print "FPR: %f\tFNR: %f"%(fpr,fnr)
        # ROCs.append([fileN,[fpr,fnr]])
        right_clicks.clear()
        # if plot:
        #     ROCPlot.drawROCCurve(ROCs)
    cv2.destroyAllWindows()
    # return ROCs

def loadImages(groundTruthFile, testPath):
    print "Load Images: "
    print "Loading: %s"%(groundTruthFile)
    gtImage = cv2.imread(groundTruthFile,0)
    print "Loaded: %s"%(groundTruthFile)
    testImages = deque()
    testFiles = deque()
    for fileN in os.listdir(testPath):
        if fileN.endswith(".pgm"):
            imFile = "%s%s"%(testPath,fileN)
            print "Loading: %s"%(imFile)
            testImages.append(cv2.imread(imFile,0))
            testFiles.append(ntpath.basename(fileN))
            print "Loaded: %s"%(imFile)
    return gtImage, testImages, testFiles
