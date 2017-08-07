#! /usr/bin/env python
import ROCCalc
import ROCPlot
import ROCMapTransform
import ROCSaveToFile
# import cv2

free = 254
occupied = 0
unknown = 206

# gtFile = "./gmapping/amiro_hokuyo_controller_speed1_gmapping_default.pgm"
gtFile = "../gmapping/amiro_sick_controller_speed1_gmapping_particles_100.pgm"
gtTransformed = "../transformed/gt.pgm"
testFolder = "../gmapping/"
saveFolder = "../transformed/"
ROCMapTransform.transformMap(gtFile,gtTransformed,testFolder,saveFolder,occupied,free,unknown)
ROCs = ROCCalc.calculateROCs(gtTransformed,saveFolder,free,occupied)
ROCPlot.drawROCCurve(ROCs)
ROCSaveToFile.saveToCSV("rocs.csv",ROCs)
