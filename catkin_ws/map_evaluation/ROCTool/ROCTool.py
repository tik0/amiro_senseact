import ROCCalc
import ROCPlot
import cv2

free = 254
occupied = 0

# gtFile = "./gmapping/amiro_hokuyo_controller_speed1_gmapping_default.pgm"
gtFile = "./gmapping/amiro_sick_controller_speed1_gmapping_default.pgm"
testFolder = "./gmapping/"

ROCs = ROCCalc.calculateROCs(gtFile,testFolder,free,occupied)
ROCPlot.drawROCCurve(ROCs)
