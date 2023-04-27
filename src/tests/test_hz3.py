import matplotlib.pyplot as plt
import numpy as np
import time

from utils.environments.environments import SamplesHZ
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations

'''
Run this script to test the functionality of the BRS methods when the admissible set of states
is non-convex and contains obstacles internally to it.
'''

##############################################################################
#                               Initialize                                   #
##############################################################################
colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]
op = ZonoOperations()

obs = SamplesHZ().obstacles_old
road, road_vis = SamplesHZ().roads_old
parking = SamplesHZ().park_old_1

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

D = np.block([A, B])
N = 4
brs = op.brs_hz(X = road, T = parking, D = D, N = N)

print(f'N = {N} \t ng = {brs.ng} \t nc = {brs.nc} \t nb = {brs.nb}')
##############################################################################
#                                Visualize                                   #
##############################################################################
vis = ZonoVisualizer()
start_time = time.perf_counter()
vis.vis_hz([obs, road_vis, parking, brs],
           title = 'BRS', 
           colors = colors, 
           legend_labels=['$\mathscr{O}$', '$\mathscr{x}$', '$\mathscr{P}$', '$\mathscr{BRS}$'],
           add_legend=True)
end_time = time.perf_counter()
print(f'Plotting took: {end_time - start_time} seconds')
plt.show()





