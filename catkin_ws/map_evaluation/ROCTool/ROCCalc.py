import os
import numpy as np
import cv2
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
        fpr, fnr, mr = calcROC(gtImage,im,free,occupied)
        ROCs.append([fileN,[fpr,fnr],mr])
        print "FPR: %f\tFNR: %f\t Matchrate: %f"%(fpr,fnr,mr)
    return ROCs

def loadImages(groundTruthFile, testPath):
    print "Load Images: "
    print "Loading: %s"%(groundTruthFile)
    gtImage = cv2.imread(groundTruthFile,0)
    print "Loaded: %s"%(groundTruthFile)
    testImages = deque()
    testFiles = deque()
    for fileN in os.listdir(testPath):
        imFile = "%s%s"%(testPath,fileN)
        if fileN.endswith(".pgm") and imFile!=groundTruthFile:
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
    if tp+fn>0:
        tpr = float(tp) / float(tp+fn)
    else:
        tpr = float(0)
    if fp+tn>0:
        fpr = float(fp) / float(fp+tn)
    else:
        fpr = float(0)
    match = np.equal(gtImage,testImage)
    missmatch = np.logical_not(match)
    mt = np.sum(match)
    mmt = np.sum(missmatch)
    mtr = float(tp)/float(tp+fp)
    return (fpr,tpr,mtr)
