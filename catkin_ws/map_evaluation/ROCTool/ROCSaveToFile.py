import os
import numpy as np
import math
from collections import deque
import ntpath

def saveToCSV(filename,ROCs):
    csvFile = open(filename,"w")
    csvFile.write("fpr,fnr,filename\n")
    for r in ROCs:
        csvFile.write("%d,%d,%s\n"%(r[1][0],r[1][1],r[0]))
    csvFile.close()
