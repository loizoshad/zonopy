import matplotlib.pyplot as plt

from utils.environments.environments import SamplesHZ
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations

'''
Run this script to test the union and intersection operations on Hybrid zonotopes.
In addition, this script provides information on how the order of the zonotope evolves.
'''

colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]

##############################################################################
#                              Original Sets                                 #
##############################################################################
hz = SamplesHZ().set_4

vis = ZonoVisualizer()
vis.vis_hz([hz], title = 'Third', colors = colors, legend_labels=['$HZ_{15}$'], add_legend=True)
plt.xlim(-14, 14)
plt.ylim(-10, 10)
plt.show()