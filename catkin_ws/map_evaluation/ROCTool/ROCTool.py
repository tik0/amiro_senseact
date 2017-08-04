#! /usr/bin/env python
import ROCCalc
import ROCPlot
import ROCMapTransform
import cv2

free = 254
occupied = 0

# gtFile = "./gmapping/amiro_hokuyo_controller_speed1_gmapping_default.pgm"
gtFile = "../gmapping/amiro_sick_controller_speed1_gmapping_default.pgm"
gtWarped = "../warped/amiro_sick_controller_speed1_gmapping_default.pgm"
testFolder = "../gmapping/"
saveFolder = "../warped/"
ROCMapTransform.transformMap(gtFile,gtWarped,testFolder,saveFolder)
ROCs = ROCCalc.calculateROCs(gtFile,testFolder,free,occupied)
ROCPlot.drawROCCurve(ROCs)
