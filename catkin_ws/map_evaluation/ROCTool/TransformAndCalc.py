import os
import numpy as np
import cv2
from collections import deque
import ntpath
import ROCCalc
import math

def transformAndCalcBest(groundTruthFolder, testPath, saveFolder, poorMapsFolder,hit,miss,unknown,feature='sift'):
    groundTruthList = deque()
    ROCs = deque()
    FLANN_INDEX_KDTREE = 0

    out = 1

    if feature == 'orb':
        feature_obj = cv2.ORB_create()
    if feature == 'sift':
        feature_obj = cv2.xfeatures2d.SIFT_create()

    for fileN in os.listdir(groundTruthFolder):
        if not fileN.endswith(".png"):
            continue
        imFile = os.path.join(groundTruthFolder,fileN)
        print("Loading: %s"%(imFile))
        im = cv2.imread(imFile,0)
        print("Loaded!")
        kp, des = feature_obj.detectAndCompute(im,None)
        des = np.asarray(des,np.float32)
        groundTruthList.append((fileN,os.path.join(saveFolder,os.path.splitext(fileN)[0]),im,des,kp))

    if not os.path.isdir(saveFolder):
        os.makedirs(saveFolder)

    # for gt_file,gt_folder,gt_im,gt_des,gt_kp in groundTruthList:
    #     if not os.path.isdir(gt_folder):
    #         os.makedirs(gt_folder)

    for fileN in os.listdir(testPath):
        if not fileN.endswith(".pgm"):
            continue
        imFile = os.path.join(testPath,fileN)
        imFileP = os.path.join(poorMapsFolder,fileN)
        if out>0:
            print("Loading: %s"%(imFile))
        im = cv2.imread(imFile,0)
        if out>0:
            print("Loaded!")
        kp, des = feature_obj.detectAndCompute(im,None)
        des = np.asarray(des,np.float32)

        best_fpr=best_tpr=best_mtr=best_val=0
        best_file=best_folder=None
        best_im=None

        for gt_file,gt_folder,gt_im,gt_des,gt_kp in groundTruthList:
            if out>1:
                print("With: %s"%(gt_file))
            height, width = gt_im.shape
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks = 50)
            flann = cv2.FlannBasedMatcher(index_params, search_params)
            matches = flann.knnMatch(des,gt_des,k=2)
            good = deque()
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)
            if out>1:
                print("Found %f points"%(len(good)))
            if len(good)<1:
                if out>1:
                    print("Not enough points!")
                continue
            dst_pts = np.float32([kp[m.queryIdx].pt for m in good]).reshape(-1,1,2)
            src_pts = np.float32([gt_kp[m.trainIdx].pt for m in good]).reshape(-1,1,2)
            M = cv2.estimateRigidTransform(src_pts,dst_pts,False)
            if M!=None:
                M = np.asarray(M,np.float32)
                M = cv2.invertAffineTransform(M)
                imW = cv2.warpAffine(im,M,(width,height),cv2.INTER_NEAREST)
                imW = imW-hit
                imW = cv2.threshold(imW,unknown-2-hit,255,3)
                imW = miss-hit-imW[1]
                imW = cv2.threshold(imW,miss-unknown,255,3)
                imW = miss-imW[1]
                fpr, tpr, mtr = ROCCalc.calcROC(gt_im,imW,miss,hit)

                new_best_val = ((tpr-fpr)/2.0)*math.sqrt(2)
                if (new_best_val>best_val):
                    best_val = new_best_val
                    best_fpr = fpr
                    best_tpr = tpr
                    best_mtr = mtr
                    best_file = gt_file
                    best_folder = gt_folder
                    best_im = imW
            else:
                if out>1:
                    print("No homography found!")

        if (best_im!=None):
            ROCs.append([fileN,[best_fpr,best_tpr],best_mtr,best_file])
            if not os.path.isdir(best_folder):
                os.makedirs(best_folder)
            im_file = os.path.join(best_folder,fileN)
            # print("\n")
            # print(im_file)
            # print("\n")
            cv2.imwrite(im_file,best_im)
        else:
            if not os.path.isdir(poorMapsFolder):
                os.makedirs(poorMapsFolder)
            im_file = os.path.join(poorMapsFolder,fileN)
            cv2.imwrite(im_file,im)
        # break
    return ROCs
