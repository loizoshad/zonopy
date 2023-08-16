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

from utils.environments.rci_demo_env import Ego, NonEgo
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
input = ego_env.input
# Non-Ego vehicle(s) space
ne_env_1 = NonEgo(visualizer = vis, car = 1); ne_space_1 = ne_env_1.state_space; ne_1 = ne_env_1.car; ne_1_obs = ne_1


# ################################################################################################
# # Step 1: Compute the full BRS from the parking spot: TODO: Here we assumed that the full BRS has been computed:
# ################################################################################################
lw = 8 * 0.05; ll_h = 2.8
nx = 2; ng = 2; nc = 0; nb = 0
c_road_h = np.array([ [0.0], [0.0] ])
Gc_road_h = np.diag(np.array([ ll_h/2  , lw/2 ]))
Gb_road_h = np.zeros((nx, nb)); Ac_road = np.zeros((nc, ng)); Ab_road_h = np.zeros((nc, nb)); b_road = np.zeros((nc, 1))
brs = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)
safe_space_1 = brs

# ################################################################################################
# # Step 2: Compute all the FRS of the obstacles
# ################################################################################################

# Step 1: Over-approximate obstacle with a constrained zonotope
obs = zono_op.oa_cz_to_hypercube_tight_4d(zono_op.oa_hz_to_cz(ne_1_obs))
obs = ConstrainedZonotope(obs.G[0:2, :], obs.C[0:2, :], obs.A, obs.b)
obs = zono_op.redundant_g_cz(obs)
obs_brs = zono_op.cz_to_hz(zono_op.oa_cz_to_hypercube_tight_4d(zono_op.oa_hz_to_cz(ne_1_obs)))
obs_brs = HybridZonotope(obs_brs.Gc[0:2, :], obs_brs.Gb[0:2, :], obs_brs.C[0:2, :], obs_brs.Ac, obs_brs.Ab, obs_brs.b)

# Step 2: Compute the complement of the obstacle
obs_compl_brs = zono_op.complement_cz_to_hz(obs)
obs_compl_brs = zono_op.intersection_hz_hz(obs_compl_brs, HybridZonotope(space.Gc[0:2, :], space.Gb[0:2, :], space.C[0:2, :], space.Ac, space.Ab, space.b))

total_time = 0
N = 31
for i in range(N):
    print(f'*********************************************************')
    print(f'Iteration {i}')

    # ################################################################################################
    # # Obstacles & Safe Space
    # ################################################################################################
    
    # ne_1_obs = ego_env.get_obstacle(X = ne_space_1, U = input, I = ne_1_obs, A = dynamics4d.A, B = dynamics4d.B, W = dynamics4d.W, bounds = ne_env_1.bounds)
    # safe_space = ego_env.get_safe_space(obs = ne_1_obs, safe_space = safe_space, A = dynamics.A, B = dynamics.B, i = i)
    # # safe_space_1 = ego_env.get_safe_space_v2(obs = ne_1_obs, safe_space = safe_space_1, A = dynamics.A, B = dynamics.B, i = i, bounds = ne_env_1.bounds[0:2, :])
    # end_time = time.perf_counter()
    # total_time = total_time + (end_time - start_time)
    obs_compl_brs_temp = zono_op.one_step_brs_hz(X = space, T = obs_compl_brs, D = np.block([dynamics.A, dynamics.B]))
    obs_compl_brs = zono_op.intersection_hz_hz(obs_compl_brs, obs_compl_brs_temp)
    print(f'obs_compl_brs: ng = {obs_compl_brs.ng}\t nc = {obs_compl_brs.nc}, nb = {obs_compl_brs.nb}')
    
    ###############################################################################################
    # Plotting
    ###############################################################################################

    ### SAFE SPACE
    ## Viualize environment
    ego_env.vis_background()
    # vis.vis_hz_4d([ne_1_obs], colors = obs_color, show_edges=True, zorder=30)
    # vis.vis_hz_4d([space], colors = brs_color, show_edges=True, zorder=30)


    # print(f'total time = {total_time}')
    # print(f'safe_space : ng = {safe_space.ng}\t nc = {safe_space.nc}\t nb = {safe_space.nb}')

    ## FINAL SAFE SPACE
    # ego_env.grid = ne_env_1.vis.vis_hz_brs_v2(hz = safe_space, brs_settings=ego_env.brs_settings)
    # ego_env.grid = ego_env.vis_safe_space(hz = safe_space)
    ego_env.grid = ego_env.vis_safe_space(hz = obs_compl_brs)

    plt.show()
    # name = f'obs_compl_brs{i}'; ego_env.vis.fig.savefig(f'./results/rci_demo/obs_compl_brs/{name}.pdf', dpi=300)
    name = f'obs_compl_brs{i}'; ego_env.vis.fig.savefig(f'./results/rci_demo/obs_compl_brs_h/obs_compl_brs_{i}.pdf', dpi=300)
    plt.close(ego_env.vis.fig); ego_env.vis.new_fig()

