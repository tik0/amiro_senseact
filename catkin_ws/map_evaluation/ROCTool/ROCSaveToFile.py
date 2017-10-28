from collections import deque

def saveToCSV(filename,ROCs):
    csvFile = open(filename,"w")
    csvFile.write("fpr,tpr,mr,filename\n")
    for r in ROCs:
        csvFile.write("%f,%f,%f,%s,%s\n"%(r[1][0],r[1][1],r[2],r[0],r[3]))
    csvFile.close()
