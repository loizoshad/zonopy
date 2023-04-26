import matplotlib.pyplot as plt
import numpy as np
import time

from utils.samples.samples import SamplesHZ, SamplesVis
from utils.visualization import ZonoVisualizer, AuxiliaryVisualizer
from utils.operations.operations import ZonoOperations

'''
Run this script to test the functionality of the BRS methods when the admissible set of states
is non-convex and contains obstacles internally to it.
'''

##############################################################################
#                               Initialize                                   #
##############################################################################

vis = ZonoVisualizer()
op = ZonoOperations(visualizer = vis)

obs = SamplesHZ().obstacles
road_line, road_line_vis, road_line_color = SamplesHZ().road_line
road, road_vis, road_color = SamplesHZ().road
parking = SamplesHZ().park_1


AuxiliaryVisualizer().vis_patches(SamplesVis().road_line)

colors = [
    road_color,                     # Road (Gray)
    (0.949, 0.262, 0.227, 1.0),     # Obstacle (Red)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 1.0)      # BRS (Blue)
]
##############################################################################
#                                  BRS                                       #
##############################################################################
# Dynamic Model
A = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])

B = np.array([
    [0.5, 0.5],
    [0.0, 0.5]
])

# D = np.block([A, B])
# N = 1
# start_time = time.perf_counter()
# brs = op.brs_hz(X = road, T = parking, D = D, N = N, visualize = True)
# end_time = time.perf_counter()
# print(f'N = {N} \t ng = {brs.ng} \t nc = {brs.nc} \t nb = {brs.nb}')
# print(f'Compute and plot BRS took: {end_time - start_time} seconds')

# Visualize Environment
vis.vis_hz([road_vis, obs, parking],
        title = 'Environment', 
        colors = colors, 
        legend_labels=['$\mathscr{O}$', '$\mathscr{x}$', '$\mathscr{P}$'],
        add_legend=True)



plt.grid(False)
plt.show()

