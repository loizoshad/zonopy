'''
In this problem we assume that the goal of eventually exiting the parking slot is a static goal
that means that the LTL specification is static in terms of that goal.
Therefore, we can precompute the full BRS offline and optimize the HZ representation.    
'''

import matplotlib.pyplot as plt
import numpy as np
import time

from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.environments.static_env_final import Environment

from utils.sets.hybrid_zonotopes import HybridZonotope


np.set_printoptions(edgeitems=10, linewidth=10000)
obs_color  = (0.949, 0.262, 0.227, 0.6)
brs_color  = (0.423, 0.556, 0.749, 0.5)
road_color = (0.717, 0.694, 0.682, 0.5)

################################################################################################
# Step 0: Initialize Environment
################################################################################################
# N = 150
N = 145

print(f'******************************************************************')
print(f'N = {N}')

zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)    # Object for visualization
env = Environment(vis, N)                              # Environment object

################################################################################################
# Step 1: Compute Full BRS From The Exit Of The Parking Slot
################################################################################################
brs = env.compute_brs(N = N)
print(f'brs: ng = {brs.ng}, nc = {brs.nc}, nb = {brs.nb}')









###############################################################################################
# Step 2: Visualize Results 
###############################################################################################
# print(f'state space: ng = {env.state_space.ng}, nc = {env.state_space.nc}, nb = {env.state_space.nb}')
# print(f'target: ng = {env.targets[0].ng}, nc = {env.targets[0].nc}, nb = {env.targets[0].nb}')
# env.vis.vis_hz_4d([env.state_space], colors = obs_color, show_edges=True, zorder=11)
# env.vis.vis_hz_4d(env.targets, colors = obs_color, show_edges=True, zorder=11)

# env.vis_env()
# start_time_2 = time.perf_counter()
# env.vis_brs()
# # env.vis_safe_space(brs)
# # env.vis_safe_space(full_safe_space)
# end_time_2 = time.perf_counter()

# print(f'tv = {end_time_2 - start_time_2}')  # Visulaitzaiton time
# print(f'******************************************************************')


# plt.show()
# name = f'safe_space_{N}'
# env.vis.fig.savefig(f'./results/static_final/target1/pdf/{name}.pdf', dpi=300)
# env.vis.fig.savefig(f'./results/static_final/target1/png/{name}.png', dpi=300)






