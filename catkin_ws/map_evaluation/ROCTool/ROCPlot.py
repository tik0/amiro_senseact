from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np

def drawROCCurve(ROCs):
    plt.figure()
    lw = 2
    ax = plt.subplot(111)
    for roc in ROCs:
        ax.plot(roc[1][0],roc[1][1], marker='o', markersize=7, lw=lw, label=roc[0])
    ax.plot([0, 1], [0, 1], color='navy', lw=lw, linestyle='--')

    box = ax.get_position()
    c=0.3
    ax.set_position([box.x0, box.y0+(1-c)*box.height, box.width, c*box.height])
    plt.xlim([0.0, 1.01])
    plt.ylim([0.0, 1.01])
    plt.xlabel('False Positive Rate')
    plt.ylabel('True Positive Rate')
    ax.legend(loc="center left",fontsize=10,bbox_to_anchor=(-.15, -1.5))
    plt.title('Receiver operating characteristic example')
    plt.show()
