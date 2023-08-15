'''
In this script we test the dynamic_env_complex_v2 environment.

More precisely we test the full functionality of:
    - computating the BRS of the ego vehicle in that environment.
    - Computing the safe space given the non-safe space resulting from the non-ego vehicles.

    - Combine those to compute the full safe space of the ego vehicle.
'''

import matplotlib.pyplot as plt
import numpy as np
import time

from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope

from utils.environments.dynamic_env_complex_full import Ego, NonEgo
from utils.ego_model_2d import DynamicsModel
from utils.ego_model_4d import DynamicsModel as DynamicsModel4D

np.set_printoptions(edgeitems=10, linewidth=10000)
obs_color  = (0.949, 0.262, 0.227, 0.6)
brs_color  = (0.423, 0.556, 0.749, 0.5)
road_color = (0.717, 0.694, 0.682, 0.5)


################################################################################################
# Initialize ego and non-ego vehicle spaces
################################################################################################
dynamics = DynamicsModel()                              # Object for the dynamics of the ego vehicle
dynamics4d = DynamicsModel4D()                          # Object for the dynamics of the non-ego vehicle
zono_op = ZonoOperations()                              # Object for zonotope operations
vis = ZonoVisualizer(zono_op = zono_op)                 # Object for visualization
ego_env = Ego(zono_op = zono_op, visualizer = vis)      # Object for the ego vehicle

# Ego vehicle space
space = ego_env.state_space
parking = ego_env.parking
input = ego_env.input
brs_safe_space = ego_env.parking    # Initialize BRS Safe Space
# Non-Ego vehicle(s) space
ne_env_1 = NonEgo(visualizer = vis, car = 1); ne_space_1 = ne_env_1.state_space; ne_1 = ne_env_1.car; ne_1_obs = ne_1
ne_env_2 = NonEgo(visualizer = vis, car = 2); ne_space_2 = ne_env_2.state_space; ne_2 = ne_env_2.car; ne_2_obs = ne_2
ne_env_3 = NonEgo(visualizer = vis, car = 3); ne_space_3 = ne_env_3.state_space; ne_3 = ne_env_3.car; ne_3_obs = ne_3
ne_env_4 = NonEgo(visualizer = vis, car = 4); ne_space_4 = ne_env_4.state_space; ne_4 = ne_env_4.car; ne_4_obs = ne_4

extra_space = ego_env.extra_space # TODO: THIS IS ONLY USED FOR VISUALIZATION NOW BEFORE THE ENV IS ACTUALLY DESIGNED IN VECTORNATOR


# ################################################################################################
# # Step 1: Compute the full BRS from the parking spot
# ################################################################################################
### TODO: Here we assumed that the full BRS has been computed:
lw = 8 * 0.05; ll_h = 2.8
nx = 2; ng = 2; nc = 0; nb = 0
c_road_h = np.array([ [0.0], [0.0] ])
Gc_road_h = np.diag(np.array([ ll_h/2  , lw/2 ]))
Gb_road_h = np.zeros((nx, nb)); Ac_road = np.zeros((nc, ng)); Ab_road_h = np.zeros((nc, nb)); b_road = np.zeros((nc, 1))
brs = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)
safe_space_1 = brs; safe_space_2 = brs; safe_space_3 = brs; safe_space_4 = brs

# ################################################################################################
# # Step 2: Compute all the FRS of the obstacles
# ################################################################################################
total_time = 0
N = 4
for i in range(N):
    print(f'*********************************************************')
    print(f'Iteration {i}')

    # ################################################################################################
    # # Obstacles & Safe Space
    # ################################################################################################
    start_time = time.perf_counter()
    #
    ne_1_obs = ego_env.get_obstacle(X = ne_space_1, U = input, I = ne_1_obs, A = dynamics4d.A, B = dynamics4d.B, W = dynamics4d.W, bounds = ne_env_1.bounds)
    # safe_space_1 = ego_env.get_safe_space(obs = ne_1_obs, safe_space = safe_space_1, A = dynamics.A, B = dynamics.B, i = i)
    safe_space_1 = ego_env.get_safe_space_v2(obs = ne_1_obs, safe_space = safe_space_1, A = dynamics.A, B = dynamics.B, i = i, bounds = ne_env_1.bounds[0:2, :])
    # #
    ne_2_obs = ego_env.get_obstacle(X = ne_space_2, U = input, I = ne_2_obs, A = dynamics4d.A, B = dynamics4d.B, W = dynamics4d.W, bounds = ne_env_2.bounds)
    # safe_space_2 = ego_env.get_safe_space(obs = ne_2_obs, safe_space = safe_space_2, A = dynamics.A, B = dynamics.B, i = i)
    safe_space_2 = ego_env.get_safe_space_v2(obs = ne_2_obs, safe_space = safe_space_2, A = dynamics.A, B = dynamics.B, i = i, bounds = ne_env_2.bounds[0:2, :])
    #
    ne_3_obs = ego_env.get_obstacle(X = ne_space_3, U = input, I = ne_3_obs, A = dynamics4d.A, B = dynamics4d.B, W = dynamics4d.W, bounds = ne_env_3.bounds)
    # safe_space_3 = ego_env.get_safe_space(obs = ne_3_obs, safe_space = safe_space_3, A = dynamics.A, B = dynamics.B, i = i)
    safe_space_3 = ego_env.get_safe_space_v2(obs = ne_3_obs, safe_space = safe_space_3, A = dynamics.A, B = dynamics.B, i = i, bounds = ne_env_3.bounds[0:2, :])
    #
    ne_4_obs = ego_env.get_obstacle(X = ne_space_4, U = input, I = ne_4_obs, A = dynamics4d.A, B = dynamics4d.B, W = dynamics4d.W, bounds = ne_env_4.bounds)
    # safe_space_4 = ego_env.get_safe_space(obs = ne_4_obs, safe_space = safe_space_4, A = dynamics.A, B = dynamics.B, i = i)
    safe_space_4 = ego_env.get_safe_space_v2(obs = ne_4_obs, safe_space = safe_space_4, A = dynamics.A, B = dynamics.B, i = i, bounds = ne_env_4.bounds[0:2, :])
    #
    end_time = time.perf_counter()
    total_time = total_time + (end_time - start_time)
    
    ###############################################################################################
    # Plotting
    ###############################################################################################

    ### SAFE SPACE
    ## Viualize environment
    vis.vis_hz_4d([parking], colors = obs_color, show_edges=True, zorder=1)
    vis.vis_hz(extra_space, colors = road_color, show_edges=False, zorder=1)
    vis.vis_hz_4d([ne_space_1], colors = road_color, show_edges=False, zorder=1)
    vis.vis_hz_4d([ne_space_2], colors = road_color, show_edges=False, zorder=1)
    vis.vis_hz_4d([ne_space_3], colors = road_color, show_edges=False, zorder=1)
    vis.vis_hz_4d([ne_space_4], colors = road_color, show_edges=False, zorder=1)

    safe_space = zono_op.intersection_hz_hz(safe_space_1, safe_space_2)
    safe_space = zono_op.intersection_hz_hz(safe_space, safe_space_3)
    safe_space = zono_op.intersection_hz_hz(safe_space, safe_space_4)

    print(f'total time = {total_time}')
    print(f'safe_space : ng = {safe_space.ng}\t nc = {safe_space.nc}\t nb = {safe_space.nb}')

    ## FINAL SAFE SPACE
    # ego_env.grid = ne_env_1.vis.vis_hz_brs_v2(hz = safe_space, brs_settings=ego_env.brs_settings)
    ego_env.grid = ego_env.vis_safe_space(hz = safe_space)
    plt.show()
    name = f'safe_space_{i}'; ego_env.vis.fig.savefig(f'./results/dynamic_env_complex_full/safe_space_v2/{name}.pdf', dpi=300)
    plt.close(ego_env.vis.fig); ego_env.vis.new_fig()

