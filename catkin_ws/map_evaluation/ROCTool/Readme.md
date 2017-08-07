# ROCTool
This tool calculates the receiver operator curve for multiple images based on a given ground-truth image. It consists of four python scripts to apply a affine transform on the test images, to calculate the true-positive-rate and false-negative-rate, to save the calculated values in a csv-file and to plot the calculated values.

## Dependencies
Python 2.7:
- ROCMapTransform/ROCCalc/ROCSaveToFile:
  - os
  - numpy
  - cv2
  - from collections: deque
  - ntpath


- ROCPlot:
  - mpl_toolkits
  - matplotlib
  - numpy

## ROCMapTransform
This tool creates to OpenCV windows. The first one shows the original image the second one shows the transformed image. With a double left click you can select a point which is then highlighted in the original image. To delete the last point you can press 'l'. To delete all point you can press 'c'. At the beginning you have to select two points to define a rectangle which fits the ground truth map/image. To delete the last point press 'l', to delete all points press 'c'. To confirm the points press 'n' and view the resulting image in the second window. If you don't like it press 'b' and correct the points. When the rectangle is defined select three points which you can easily detect in the test images as reference points to perform an affine transformation. Again confirm the points with 'n'. For each test image mark the reference points in the same order you did before in the ground truth image and confirm them with 'n'. The transformed image can bee seen in the second window. If you do not like the chosen points press 'b' and use 'l' to delete the last and 'c' to delete all points. When you are happy with the transformed image press 'n' again and continue with next test image. If you do not want to use one of the images press 'q' to skip it.
  - transformMap(groundTruthFile, saveFile, testPath, saveFolder,hit,miss,unknown)
    - groundTruthFile - filepath to the ground-truth image
    - testPath - path to the folder containing the test images

## ROCCalc
This tool calculates for a given ground truth image and all test images the true-positive-rate and the false-negative-rate and returns a list of tuples of filename and calculated values.
### external functions:
  - calculateROCs(groundTruthFile, testPath, free, occupied)
    - groundTruthFile - filepath to the ground-truth image
    - testPath - path to the folder containing the test images
    - free - gray value for free cells (miss)
    - occupied - gray value for occupied cells (hit)
    - return: deque(filename,(tpr,fnr))

### internal functions:
  - loadImages(groundTruthFile, testPath)
    - groundTruthFile - filepath to the ground-truth image
    - testPath - path to the folder containing the test images
    - return: (np.array(dtype=uint_8), deque(np.array(dtype=uint_8)), deque(filenames))


  - calcROC(gtImage, testImage, free, occupied)
    - gtImage - numpy array for the ground truth image
    - testImage - numpy array for one test image
    - free - gray value for free cells (miss)
    - occupied - gray value for occupied cells (hit)

## ROCSaveToFile
This tool saves the list of tuples returned by ROCCalc in a csv file.
  - saveToCSV(filename,ROCs)
    - filename - filename for the csv file
    - ROCs - deque(filename,(tpr,fnr)) from ROCCalc.calculateROCs

## ROCPlot
This tool creates a first plot of the calculated roc values.
  - drawROCCurve(ROCs)
    - ROCs - deque(filename,(tpr,fnr)) from ROCCalc.calculateROCs
