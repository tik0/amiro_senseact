#! /usr/bin/env python
import ROCCalc
import ROCPlot
import ROCMapTransform
import ROCSaveToFile
import TransformAndCalc
# import cv2

free = 254
occupied = 0
unknown = 206

# gtFile = "../gmapping/amiro_sick_controller_speed1_gmapping_particles_100.pgm"
# gtTransformed = "../transformed/gt.pgm"
# testFolder = "../gmapping/"
# saveFolder = "../transformed/"
gtFile = "../gt/stitched_cut.png"
gtPath = "../gt_creator/gt_maps/"
gtTransformed = "../transformed/gt.pgm"
testFolder = "../maps/"
saveFolder = "../transformed/"
poorMapsFolder = "../notUsed/"
ROCs = TransformAndCalc.transformAndCalcBest(gtPath,testFolder,saveFolder,poorMapsFolder,occupied,free,unknown)
# ROCMapTransform.transformMap(gtFile,gtTransformed,testFolder,saveFolder,poorMapsFolder,occupied,free,unknown)
# ROCs = ROCCalc.calculateROCs(gtTransformed,saveFolder,free,occupied)
ROCSaveToFile.saveToCSV("./../Auswertung/rocs.csv",ROCs)
ROCPlot.drawROCCurve(ROCs)
