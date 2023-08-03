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
obs_color = (0.949, 0.262, 0.227, 0.6)
brs_color = (0.423, 0.556, 0.749, 0.5)

dynamics = DynamicsModel()                              # Object for the dynamics of the ego vehicle
dynamics4d = DynamicsModel4D()                          # Object for the dynamics of the non-ego vehicle
zono_op = ZonoOperations()                              # Object for zonotope operations
vis = ZonoVisualizer(zono_op = zono_op)                 # Object for visualization
ego_env = Ego(zono_op = zono_op, visualizer = vis)      # Object for the ego vehicle
non_ego_env = NonEgo(visualizer = vis)                  # Object for the non-ego vehicle(s)


################################################################################################
# Auxiliary methods
################################################################################################
def get_obstacle(road_down, road_right, input, car_down, car_right, A, B, W):
    car_down_1 = zono_op.one_step_frs_hz(X = road_down, U = input, I = car_down, A = A, B = B, W = W)     # Car down  # V3

    car_right_1 = zono_op.one_step_frs_hz(X = road_right, U = input, I = car_right, A = A, B = B, W = W)  # Car right # V3
    car_right_2 = zono_op.one_step_frs_hz(X = road_right, U = input, I = car_down, A = A, B = B, W = W)  # Car right # V3
    
    car_down = car_down_1
    car_down  = zono_op.cz_to_hz( zono_op.oa_cz_to_hypercube_tight_4d( zono_op.oa_hz_to_cz(car_down)))      # Simplify car down
    car_right_1 = zono_op.cz_to_hz( zono_op.oa_cz_to_hypercube_tight_4d( zono_op.oa_hz_to_cz(car_right_1)))     # Simplify car right_1
    car_right_2 = zono_op.cz_to_hz( zono_op.oa_cz_to_hypercube_tight_4d( zono_op.oa_hz_to_cz(car_right_2)))     # Simplify car right_2
    car_right = zono_op.union_hz_hz_v2(car_right_1, car_right_2)

    return car_down, car_right

def get_safe_space_down(car_down, space) -> HybridZonotope:
    G_d = np.zeros((2, 2))
    G_d[0, 0] = car_down.Gc[0, 0]; G_d[1, 1] = car_down.Gc[1, 1]
    C_d = np.zeros((2, 1))
    C_d[0, 0] = car_down.C[0, 0]; C_d[1, 0] = car_down.C[1, 0]
    cz_down = ConstrainedZonotope(G_d, C_d, np.zeros((0, G_d.shape[1])), np.zeros((0, 1)))
    safe_down = zono_op.complement_cz_to_hz(cz_down)

    # Add the velocity states
    Gc_d = np.block([
        [safe_down.Gc, np.zeros((safe_down.Gc.shape[0], 2))],
        [np.zeros((2, safe_down.Gc.shape[1])), np.eye(2)]
    ])
    #
    Gb_d = np.block([
        [safe_down.Gb],
        [np.zeros((2, safe_down.Gb.shape[1]))]
    ])
    #
    C_d = np.zeros((4, 1))
    C_d[0, 0] = safe_down.C[0, 0]; C_d[1, 0] = safe_down.C[1, 0]
    C_d[2, 0] = 0.0; C_d[3, 0] = 0.0  # Velocity states
    #
    Ac_d = np.block([
        [safe_down.Ac, np.zeros((safe_down.Ac.shape[0], 2))],
    ])
    Ab_d = safe_down.Ab
    b_d = safe_down.b

    safe_down = HybridZonotope(Gc_d, Gb_d, C_d, Ac_d, Ab_d, b_d)

    safe_space = safe_down
    safe_space = zono_op.intersection_hz_hz(safe_space, space)

    return safe_space

def get_safe_space_right(car_right, space):
    G_r = np.zeros((2, 2))
    G_r[0, 0] = car_right.Gc[0, 0]; G_r[1, 1] = car_right.Gc[1, 1]
    C_r = np.zeros((2, 1))
    C_r[0, 0] = car_right.C[0, 0]; C_r[1, 0] = car_right.C[1, 0]
    cz_right = ConstrainedZonotope(G_r, C_r, np.zeros((0, G_r.shape[1])), np.zeros((0, 1)))
    safe_right = zono_op.complement_cz_to_hz(cz_right)

    # Add the velocity states
    Gc_r = np.block([
        [safe_right.Gc, np.zeros((safe_right.Gc.shape[0], 2))],
        [np.zeros((2, safe_right.Gc.shape[1])), np.eye(2)]
    ])
    #
    Gb_r = np.block([
        [safe_right.Gb],
        [np.zeros((2, safe_right.Gb.shape[1]))]
    ])
    #
    C_r = np.zeros((4, 1))
    C_r[0, 0] = safe_right.C[0, 0]; C_r[1, 0] = safe_right.C[1, 0]
    C_r[2, 0] = 0.0; C_r[3, 0] = 0.0  # Velocity states
    #
    Ac_r = np.block([
        [safe_right.Ac, np.zeros((safe_right.Ac.shape[0], 2))],
    ])
    Ab_r = safe_right.Ab
    b_r = safe_right.b

    safe_right = HybridZonotope(Gc_r, Gb_r, C_r, Ac_r, Ab_r, b_r)

    safe_space = safe_right
    safe_space = zono_op.intersection_hz_hz(safe_space, space)

    return safe_space

# def get_safe_space(car_right, car_down, space):
    if zono_op.is_empty_cz(zono_op.oa_hz_to_cz(car_right)):
        # print(f'CAR RIGHT IS EMPTY')
        safe_space = get_safe_space_down(car_down, space)
    elif zono_op.is_empty_cz(zono_op.oa_hz_to_cz(car_down)):
        # print(f'ROAD DOWN IS EMPTY')
        safe_space = get_safe_space_down(car_right, space)
    else:
        # print(f'NEITHER CAR IS EMPTY')
        safe_space_down = get_safe_space_down(car_down, space)
        safe_space_right = get_safe_space_right(car_right, space)
        safe_space = zono_op.intersection_hz_hz(safe_space_down, safe_space_right)

    return safe_space

def get_safe_space(car_right, car_down, space):
    # Extract only the position states of each space
    car_down_2d = HybridZonotope(car_down.Gc[0:2, :], car_down.Gb[0:2, :], car_down.C[0:2, :], car_down.Ac, car_down.Ab, car_down.b)    
    car_right_2d = HybridZonotope(car_right.Gc[0:2, :], car_right.Gb[0:2, :], car_right.C[0:2, :], car_right.Ac, car_right.Ab, car_right.b)    

    # Compute the union of the two hybrid zonotopes
    car_hz = zono_op.union_hz_hz_v2(car_down_2d, car_right_2d)

    # Obtain cz over-approximation
    car_cz = zono_op.oa_cz_to_hypercube_tight( zono_op.oa_hz_to_cz(car_hz))

    # Compute the complement of it
    safe_space = zono_op.complement_cz_to_hz(car_cz)

    # TODO: Compute intersection with the position states of the space
    space_2d = HybridZonotope(space.Gc[0:2, :], space.Gb[0:2, :], space.C[0:2, :], space.Ac, space.Ab, space.b)
    safe_space = zono_op.intersection_hz_hz(safe_space, space_2d)

    return safe_space


################################################################################################
# Initialize ego and non-ego vehicle spaces
################################################################################################
# Ego vehicle space
space = ego_env.state_space(options = 'brs_outer')
target = ego_env.parking(options = 'outer')
input = ego_env.input()
brs_safe_space = ego_env.parking(options = 'outer') # Initialize BRS Safe Space
# Non-Ego vehicle(s) space
non_ego_space = non_ego_env.state_space(options = 1)
non_ego = non_ego_env.car(options = 1)

## Viualize Ego vehicle space
# ego_env.vis_background()
vis.vis_hz_4d([space], colors = brs_color, show_edges=True, zorder=100)
# plt.show()
## Viualize Non-Ego vehicle space
# ego_env.vis_background()
vis.vis_hz_4d([non_ego_space], colors = brs_color, show_edges=True, zorder=100)
vis.vis_hz_4d([non_ego], colors = obs_color, show_edges=True, zorder=100)
plt.show()







# ################################################################################################
# # Main loop
# ################################################################################################
# N = 101
# for i in range(N):
#     print(f'*********************************************************')
#     print(f'Iteration {i}')

#     ################################################################################################
#     # BRS
#     ################################################################################################
#     target = zono_op.one_step_brs_hz(X = space, T = target, D = np.block([dynamics.A, dynamics.B]))
#     print(f'brs : ng = {target.ng}\t nc = {target.nc}\t nb = {target.nb}')

#     ################################################################################################
#     # Obstacles & Safe Space
#     ################################################################################################
#     car_down, car_right = get_obstacle(road_down, road_right, input, car_down, car_right, dynamics4d.A, dynamics4d.B, dynamics4d.W)
#     safe_space = get_safe_space(car_right, car_down, space)
#     print(f'safe_space : ng = {safe_space.ng}\t nc = {safe_space.nc}\t nb = {safe_space.nb}')

#     ################################################################################################
#     # Final Safe BRS Space
#     ################################################################################################
#     # Compute the union between the BRS at time step 'i'
#     brs_safe_space = zono_op.union_hz_hz_v2(brs_safe_space, target)

#     # Compute the intersection with the complement of the moving obstacle at time 'i'
#     brs_safe_space = zono_op.intersection_hz_hz(brs_safe_space, safe_space)
#     print(f'brs_safe_space : ng = {brs_safe_space.ng}\t nc = {brs_safe_space.nc}\t nb = {brs_safe_space.nb}')

#     ################################################################################################
#     # Plotting
#     ################################################################################################
#     #########################################################    
#     # ### BRS
#     # ego_env.vis_background()
#     # brs_vis = HybridZonotope(target.Gc[0:2, :], target.Gb[0:2, :], target.C[0:2, :], target.Ac, target.Ab, target.b)

#     # ego_env.grid = ego_env.vis.vis_hz_brs_v2(hz = brs_vis, brs_settings=ego_env.brs_settings)

#     # # plt.show()
#     # ego_env.vis.ax.set_title(f'iter = {i}, ng = {target.ng}, nc = {target.nc}, nb = {target.nb}', fontsize=16)
#     # name = f'brs_N_{i}'
#     # ego_env.vis.fig.savefig(f'./results/dynamic_env_complex_full/brs/{name}.pdf', dpi=300)
#     # plt.close(ego_env.vis.fig)
#     # ego_env.vis.new_fig()
#     #########################################################
#     # ## OBSTACLES
#     # obs = zono_op.union_hz_hz_v2(car_down, car_right)
#     # print(f'Obs. : ng = {obs.ng}\t nc = {obs.nc}\t nb = {obs.nb}')
#     # ego_env.vis_background()
#     # # static_obs_vis = HybridZonotope(car_down.Gc[0:2, :], car_down.Gb[0:2, :], car_down.C[0:2, :], car_down.Ac, car_down.Ab, car_down.b)
#     # # vis.vis_hz([static_obs_vis], colors = obs_color, show_edges=True, zorder=100)
#     # # static_obs_vis = HybridZonotope(car_right.Gc[0:2, :], car_right.Gb[0:2, :], car_right.C[0:2, :], car_right.Ac, car_right.Ab, car_right.b)
#     # # vis.vis_hz([static_obs_vis], colors = obs_color, show_edges=True, zorder=100)
#     # static_obs_vis = HybridZonotope(car_down.Gc[0:2, :], car_down.Gb[0:2, :], car_down.C[0:2, :], car_down.Ac, car_down.Ab, car_down.b)
#     # non_ego_env.grid = non_ego_env.vis.vis_hz_brs_v2(hz = static_obs_vis, brs_settings=non_ego_env.brs_settings)
#     # static_obs_vis = HybridZonotope(car_right.Gc[0:2, :], car_right.Gb[0:2, :], car_right.C[0:2, :], car_right.Ac, car_right.Ab, car_right.b)
#     # non_ego_env.grid = non_ego_env.vis.vis_hz_brs_v2(hz = static_obs_vis, brs_settings=non_ego_env.brs_settings)
#     # name = f'obs_N_{i}'
#     # plt.show()
#     # ego_env.vis.fig.savefig(f'./results/dynamic_env_complex_full/obstacles/{name}.pdf', dpi=300)
#     # plt.close(ego_env.vis.fig)
#     # ego_env.vis.new_fig()
#     #########################################################
#     # ## OBS COMPLEMENT
#     # # Plot safe space
#     # print(f'safe_space. : ng = {safe_space.ng}\t nc = {safe_space.nc}\t nb = {safe_space.nb}')
#     # ego_env.vis_background()
#     # safe_space_vis = HybridZonotope(safe_space.Gc[0:2, :], safe_space.Gb[0:2, :], safe_space.C[0:2, :], safe_space.Ac, safe_space.Ab, safe_space.b)
#     # # vis.vis_hz([safe_space_vis], colors = obs_color, show_edges=True, zorder=100)
#     # non_ego_env.grid = non_ego_env.vis.vis_hz_brs_v2(hz = safe_space_vis, brs_settings=non_ego_env.brs_settings)
#     # plt.show()
#     # ego_env.vis.ax.set_title(f'iter = {i}, Complement of Obstacle', fontsize=16)
#     # name = f'obs_compl_N_{i}'
#     # ego_env.vis.fig.savefig(f'./results/dynamic_env_complex_full/obs_compl/{name}.pdf', dpi=300)
#     # plt.close(ego_env.vis.fig)
#     # ego_env.vis.new_fig()
#     #########################################################
#     ### BRS SAFE SPACE
#     ego_env.vis_background()
#     brs_safe_space_vis = HybridZonotope(brs_safe_space.Gc[0:2, :], brs_safe_space.Gb[0:2, :], brs_safe_space.C[0:2, :], brs_safe_space.Ac, brs_safe_space.Ab, brs_safe_space.b)

#     ego_env.grid = ego_env.vis.vis_hz_brs_v2(hz = brs_safe_space_vis, brs_settings=ego_env.brs_settings)

#     plt.show()
#     ego_env.vis.ax.set_title(f'iter = {i}, ng = {target.ng}, nc = {target.nc}, nb = {target.nb}', fontsize=16)
#     name = f'brs_N_{i}'
#     ego_env.vis.fig.savefig(f'./results/dynamic_env_complex_full/brs_safe_space/{name}.pdf', dpi=300)
#     plt.close(ego_env.vis.fig)
#     ego_env.vis.new_fig()








