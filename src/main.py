import matplotlib.pyplot as plt
import numpy as np
import time

from utils.samples.samples import SamplesHZ, SamplesVis, ParkEnv1
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
env = ParkEnv1()

# obs, obs_color = SamplesHZ().obstacles
# road_line, road_line_vis, road_line_color = SamplesHZ().road_line
# road, road_vis, road_color = SamplesHZ().road
# parking, parking_color = SamplesHZ().park_1

# colors = [road_color] + [parking_color]
# hz = [road_vis] + [parking]

hz, hz_vis, colors = env.get_sets()

##############################################################################
#                                  BRS                                       #
##############################################################################
# # Dynamic Model
# A = np.array([
#     [1.0, 0.0],
#     [0.0, 1.0]
# ])

# B = np.array([
#     [0.1, 0.1],
#     [0.0, 0.1]
# ])

# D = np.block([A, B])
# N = 2
# start_time = time.perf_counter()
# brs = op.brs_hz(X = road, T = parking, D = D, N = N, visualize = True)
# end_time = time.perf_counter()
# print(f'N = {N} \t ng = {brs.ng} \t nc = {brs.nc} \t nb = {brs.nb}')
# print(f'Compute and plot BRS took: {end_time - start_time} seconds')

# Visualize Environment
vis.vis_hz(hz_vis,
        title = 'Environment', 
        colors = colors, 
        legend_labels=['$\mathscr{O}$', '$\mathscr{x}$', '$\mathscr{P}$'],
        add_legend=False)




AuxiliaryVisualizer().vis_patches()
# AuxiliaryVisualizer().vis_images()

# plt.xlim(-14, 14)
# plt.ylim(-10, 10)        
plt.grid(False)
plt.show()

