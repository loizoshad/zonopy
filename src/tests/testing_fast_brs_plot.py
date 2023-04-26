import matplotlib.pyplot as plt
import numpy as np
import time

from utils.samples.samples import SamplesHZ
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
    (0.949, 0.262, 0.227, 1.0),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 1.0)      # BRS (Blue)
]
vis = ZonoVisualizer()
op = ZonoOperations(visualizer = vis)

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
N = 2
start_time = time.perf_counter()
brs = op.brs_hz(X = road, T = parking, D = D, N = N, visualize = True)
end_time = time.perf_counter()
print(f'N = {N} \t ng = {brs.ng} \t nc = {brs.nc} \t nb = {brs.nb}')
print(f'Compute and plot BRS took: {end_time - start_time} seconds')

# Visualize Environment
vis.vis_hz([obs, road_vis, parking],
        title = 'Environment', 
        colors = colors, 
        legend_labels=['$\mathscr{O}$', '$\mathscr{x}$', '$\mathscr{P}$'],
        add_legend=True)

# Save the figure
name = f'brs_N_{N}'
# Set the size of the figure to be saved
plt.gcf().set_size_inches(15, 8)
plt.savefig(f'./results/{name}.pdf', dpi=300)


plt.show()

