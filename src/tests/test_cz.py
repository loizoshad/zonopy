import numpy as np
import matplotlib.pyplot as plt

from utils.sets.zonotopes import Zonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.environments.environments import SamplesCZ
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations

'''
Run this script to test all the supported Constrained Zonotope operations.

TODO: Add a test for set membership
'''




##############################################################################
#                              Original Sets                                 #
##############################################################################
vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

cz1 = SamplesCZ().set_1
cz2 = SamplesCZ().set_2
vis.vis_cz([cz1, cz2], show_edges = True)
plt.show()


##############################################################################
#                              Minkowski Sum                                 #
##############################################################################
vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

cz3 = ZonoOperations().ms_cz_cz(cz1, cz2)
vis.vis_cz([cz1, cz2, cz3], show_edges = True)
plt.show()

##############################################################################
#                            Linear Transformation                           #
##############################################################################
vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

M = np.array([
    [1.0, 1.0],
    [0.0, 1.0]
])
cz4 = ZonoOperations().lt_cz(M, cz1)
vis.vis_cz([cz1, cz4], show_edges = True)
plt.show()

##############################################################################
#                              Intersection                                  #
##############################################################################
vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

cz5 = ZonoOperations().intersection_cz_cz(cz1, cz2)
vis.vis_cz([cz1, cz2, cz5], show_edges = True)
plt.show()






