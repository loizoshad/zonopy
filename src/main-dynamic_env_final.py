'''
In this problem we assume that the goal of eventually exiting the parking slot is a static goal
that means that the LTL specification is static in terms of that goal.
Therefore, we can precompute the full BRS offline and optimize the HZ representation.    
'''

import matplotlib.pyplot as plt
import numpy as np

from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.environments.dynamic_env_complex_final import Environment

from utils.cars import Car1, Car2
from utils.sets.hybrid_zonotopes import HybridZonotope


np.set_printoptions(edgeitems=10, linewidth=10000)
obs_color  = (0.949, 0.262, 0.227, 0.6)
brs_color  = (0.423, 0.556, 0.749, 0.5)
road_color = (0.717, 0.694, 0.682, 0.5)


################################################################################################
# Step 0: Initialize Environment
################################################################################################
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)    # Object for visualization
env = Environment(vis)                              # Environment object

N = 10
print(f'N = {N}')

################################################################################################
# Step 1: Compute Full BRS From The Exit Of The Parking Slot
################################################################################################
brs = env.static_brs

###############################################################################################
# Step 1: Remove The Non-Safe Space Due To Dynamic Obstacles
###############################################################################################
full_safe_space = env.compute_full_safe_space(N)
print(f'full_safe_space: ng = {full_safe_space.ng}, nc = {full_safe_space.nc}, nb = {full_safe_space.nb}')

# car = Car1()
# obs = car.initial_space4D
# obs_pos = HybridZonotope(obs.Gc[0:2, :],  obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)

# for i in range(N):
#     # print(f'******************************************************************')
#     print(f'Computing obs for step {i}')
#     obs = zono_op.one_step_frs_hz_v3(X = car.state_spaceFRS, U = car.input_space, I = obs, A = car.dynamics.A, B = car.dynamics.B)
#     obs_pos = HybridZonotope(obs.Gc[0:2, :],  obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
    
#     obs_pos = zono_op.oa_hz_to_cz(obs_pos)
#     # obs_pos = zono_op.oa_cz_to_hypercube_tight_2d(obs_pos, bounds = car.bounds)
#     obs_pos = zono_op.redundant_g_cz(obs_pos)
#     obs_pos = zono_op.cz_to_hz(obs_pos)


###############################################################################################
# Step 2: Perform Model Checking For All Cars In The Environment
###############################################################################################


###############################################################################################
# Step 3: Visualize Results 
###############################################################################################
# env.vis_env()
# env.vis.vis_hz_4d([env.car2.state_spaceFRS], colors = obs_color, show_edges=True, zorder=11)

env.vis_safe_space(full_safe_space)
# env.vis_safe_space(obs_pos)

plt.show()
name = f'safe_space_{N}'
env.vis.fig.savefig(f'./results/dynamic_env_final/safe_space_car_1_2/{name}.pdf', dpi=300)
# name = f'obs_{N}'
# env.vis.fig.savefig(f'./results/dynamic_env_final/obs_car3/{name}.pdf', dpi=300)
plt.close(env.vis.fig); env.vis.new_fig()



###############################################################################################
# Step 4: Move To Next Time Step
# Move each vehicle one time-step forward
# ...
###############################################################################################










# N = 1

# for i in range(N):
#     print(f'Iteration = {i}')

#     # Visualize
#     print(f'brs: ng = {brs.ng}, nc = {brs.nc}, nb = {brs.nb}')
#     env.vis_safe_space(brs)
#     plt.show()
#     # name = f'brs_{i}'
#     # env.vis.fig.savefig(f'./results/dynamic_env_final/brs/{name}.pdf', dpi=300)
#     # plt.close(env.vis.fig); env.vis.new_fig()

