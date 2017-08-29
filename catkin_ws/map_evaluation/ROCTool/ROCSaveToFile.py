from collections import deque

def saveToCSV(filename,ROCs):
    csvFile = open(filename,"w")
    csvFile.write("fpr,fnr,mr,filename\n")
    for r in ROCs:
        csvFile.write("%f,%f,%f,%s\n"%(r[1][0],r[1][1],r[2],r[0]))
    csvFile.close()
