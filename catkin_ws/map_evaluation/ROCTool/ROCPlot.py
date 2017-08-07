from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np

def drawROCCurve(ROCs):
    # plt.figure('ROC curve')
    plt.figure(figsize=(15,5),dpi=100)
    lw = 2
    ax = plt.subplot(111)
    # fig,ax = plt.subplots(figsize=(12,12))
    minx=miny=0
    maxx=maxy=1
    for roc in ROCs:
        minx = max(minx,roc[1][0])
        miny = max(miny,roc[1][1])
        maxx = min(maxx,roc[1][0])
        maxy = min(maxy,roc[1][1])
        ax.plot(roc[1][0],roc[1][1], marker='o', markersize=7, lw=lw, label=roc[0])
    ax.plot([-1, 2], [-1, 2], color='navy', lw=lw, linestyle='--')
    minx=max(minx-0.2,0)
    miny=max(miny-0.2,0)
    maxx=min(maxx+0.2,1.1)
    maxy=min(maxy+0.2,1.1)

    box = ax.get_position()
    c=0.3
    ax.set_position([box.y0, box.y0, c*box.width, box.height])
    plt.xlim([minx, maxx])
    plt.ylim([miny, maxy])
    plt.xlabel('False Positive Rate')
    plt.ylabel('True Positive Rate')
    ax.legend(loc="center left",fontsize=10,bbox_to_anchor=(1.05, box.height))
    plt.title('Receiver operating characteristic example')
    plt.show()
