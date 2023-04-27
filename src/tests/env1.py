import matplotlib.pyplot as plt
import numpy as np
import time

from utils.samples.samples import SamplesHZ, SamplesVis, ParkEnv1
from utils.visualization import ZonoVisualizer, AuxiliaryVisualizer
from utils.operations.operations import ZonoOperations

'''
Run this script to test the first environment
'''

##############################################################################
#                               Initialize                                   #
##############################################################################

env = ParkEnv1()
vis = ZonoVisualizer(env = env)
op = ZonoOperations(visualizer = vis)


hz, hz_vis, colors = env.get_sets()
road = hz[0]; parking = op.union_hz_hz(hz[1], hz[2])

##############################################################################
#                                  BRS                                       #
##############################################################################
# Dynamic Model
A = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])

B = np.array([
    [0.1, 0.0],
    [0.0, 0.1]
])

D = np.block([A, B])
N = 1
start_time = time.perf_counter()
brs = op.brs_hz(X = road, T = parking, D = D, N = N, visualize = True)
end_time = time.perf_counter()
print(f'N = {N} \t ng = {brs.ng} \t nc = {brs.nc} \t nb = {brs.nb}')
print(f'Compute and plot BRS took: {end_time - start_time} seconds')

# Visualize Environment
vis.vis_hz(hz_vis,
        title = 'Environment', 
        colors = colors, 
        legend_labels=['$\mathscr{O}$', '$\mathscr{x}$', '$\mathscr{P}$'],
        add_legend=False)




AuxiliaryVisualizer().vis_patches()
AuxiliaryVisualizer().vis_images()       
plt.grid(False)


# Save the figure
name = f'brs_N_{N}'
# Set the size of the figure to be saved
plt.gcf().set_size_inches(15, 8)
plt.savefig(f'./results/env1/{name}.pdf', dpi=300)

plt.show()

