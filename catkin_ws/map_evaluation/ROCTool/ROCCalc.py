import os
import numpy as np
import cv2
import math
from collections import deque
import ntpath

def calculateROCs(groundTruthFile, testPath, free, occupied):
    gtImage, testImages, testFiles = loadImages(groundTruthFile, testPath)
    ROCs = deque()
    rx,ry = gtImage.shape[0:2]
    for i in range(0,len(testFiles)):
        im = testImages[i]
        fileN = testFiles[i]
        print "Calculate ROC for: %s and %s:"%(groundTruthFile,fileN)
        fpr, fnr = calcROC(gtImage,im,free,occupied)
        ROCs.append([fileN,[fpr,fnr]])
        print "FPR: %f\tFNR: %f"%(fpr,fnr)
    return ROCs

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

def calcROC(gtImage, testImage, free, occupied):
    gtFree = np.equal(gtImage,free)
    gtOccu = np.equal(gtImage,occupied)
    testFree = np.equal(testImage,free)
    testOccu = np.equal(testImage,occupied)

    tpMat = np.logical_and(gtOccu,testOccu)
    tnMat = np.logical_and(gtFree,testFree)
    fpMat = np.logical_and(gtFree,testOccu)
    fnMat = np.logical_and(gtOccu,testFree)

    tn = np.sum(tnMat)
    tp = np.sum(tpMat)
    fp = np.sum(fpMat)
    fn = np.sum(fnMat)

    tpr = tp / float(tp+fn)
    fpr = fp / float(fp+tn)
    return (fpr,tpr)
