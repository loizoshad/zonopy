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
from utils.environments.dynamic_env_complex_final import Environment

from utils.cars import Car1, Car2, Car3, Car4, Car5
from utils.sets.hybrid_zonotopes import HybridZonotope


np.set_printoptions(edgeitems=10, linewidth=10000)
obs_color  = (0.949, 0.262, 0.227, 0.6)
brs_color  = (0.423, 0.556, 0.749, 0.5)
road_color = (0.717, 0.694, 0.682, 0.5)

Gc = np.diag(np.array([ 0.2, 0.05]))
c = np.array([ [1.6], [0.0] ])
Gb = np.zeros((2, 0)); Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
exit_space = HybridZonotope(Gc, Gb, c, Ac, Ab, b)


################################################################################################
# Step 0: Initialize Environment
################################################################################################
t = 51
N = 30

zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)    # Object for visualization
env = Environment(vis, t = t)                              # Environment object

################################################################################################
# Step 1: Compute Full BRS From The Exit Of The Parking Slot
################################################################################################
brs = env.static_brs

###############################################################################################
# Step 1: Remove The Non-Safe Space Due To Dynamic Obstacles
###############################################################################################
start_time = time.perf_counter()
full_safe_space = env.compute_full_safe_space(N)
end_time = time.perf_counter()
print(f'******************************************************************')
print(f't = {t}')
print(f'ng = {full_safe_space.ng}'); print(f'nc = {full_safe_space.nc}'); print(f'nb = {full_safe_space.nb}')
print(f'tc = {end_time - start_time}')  # Computation time

###############################################################################################
# Step 2: Visualize Results 
###############################################################################################
# env.vis.vis_hz_4d(env.cars[0].conflict_zone, colors = obs_color, show_edges=True, zorder=11)
# env.vis.vis_hz_4d([env.cars[0].current_road], colors = obs_color, show_edges=True, zorder=11)
# env.vis.vis_hz_4d([env.cars[0].state_spaceFRS], colors = obs_color, show_edges=True, zorder=11)
# env.vis.vis_hz_4d([env.cars[0].related_conflict_area], colors = obs_color, show_edges=True, zorder=11)

env.vis_env()
start_time_2 = time.perf_counter()
env.vis_safe_space(full_safe_space)
end_time_2 = time.perf_counter()
env.vis_exit_space()

print(f'tv = {end_time_2 - start_time_2}')  # Visulaitzaiton time
print(f'******************************************************************')




plt.show()
name = f'safe_space_{t}'
env.vis.fig.savefig(f'./results/animation/pdf/{name}.pdf', dpi=300)
env.vis.fig.savefig(f'./results/animation/png/{name}.png', dpi=300)






