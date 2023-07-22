import matplotlib.pyplot as plt
import numpy as np
import time
import copy

from utils.environments.dynamic_env2 import DynamicEnv2
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.ego_model_2d import DynamicsModel

colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]


np.set_printoptions(edgeitems=10, linewidth=10000)
marker_size = 0.5 * 39.36           # 0.2m in inches
marker_size = marker_size**2        # area of the marker
first_time = True

options = 'outer'
reduction_options = 'reduced'

# Initialize objects
dynamics = DynamicsModel()
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)
env = DynamicEnv2(zono_op = zono_op, dynamics = dynamics, visualizer = vis, options = options)

# Create the space
space = env.state_space             # Safe state space
target = env.target_space           # Free parking spots
input = env.input_space
obs = env.initial_space   # Initial space of non-ego vehicles

method = 2
total_time = 0
N = 11
print(f'Method {method}')
total_time = 0
for i in range(N):
    print(f'*********************************************************')
    print(f'Iteration {i}')

    start_time = time.perf_counter()

    # obs = zono_op.one_step_frs_hz(X = space, U = input, I = obs, A = env.A, B = env.B, W = env.W)
    obs = zono_op.one_step_brs_hz_v2(X = space, U = input, T = obs, A = env.A, B = env.B)
    # obs = zono_op.intersection_hz_hz(obs, space)

    if method == 1: 
        cz = zono_op.oa_hz_to_cz(obs)
        cz = zono_op.redundant_c_g_cz(cz)
        obs = zono_op.cz_to_hz(cz)
    elif method == 2:
        cz = zono_op.oa_hz_to_cz(obs)
        cz = zono_op.redundant_c_g_cz(cz)
        cz = zono_op.oa_cz_to_hypercube_tight_v2(cz)
        hz = zono_op.cz_to_hz(cz)
        obs = zono_op.cz_to_hz(cz)
        obs = zono_op.intersection_hz_hz(hz, space)
        # hz = zono_op.complement_cz_to_hz(cz)
        # hz = zono_op.intersection_hz_hz(hz, space)
    end_time = time.perf_counter()

    print(f'Obstacle: ng = {obs.ng}\t nc = {obs.nc}\t nb = {obs.nb}')
    print(f'time = {end_time - start_time}')
    total_time += (end_time - start_time)
    env.vis_background()
    vis.vis_hz([obs], colors = colors, show_edges=True, zorder=100)
    # env.grid = env.vis.vis_hz_brs(hz = obs, brs_settings=env.brs_settings)
    # vis.vis_hz([hz], colors = colors, show_edges=True, zorder=100)
    # env.grid = env.vis.vis_hz_brs(hz = hz, brs_settings=env.brs_settings)

    plt.show()
    env.vis.ax.set_title(f'Method {method}, ng = {obs.ng}, nc = {obs.nc}, nb = {obs.nb}', fontsize=16)
    name = f'frs_N_{i}'
    env.vis.fig.savefig(f'./results/temp/method_{method}_v2/{name}.pdf', dpi=300)
    # env.vis.fig.savefig(f'./results/temp/compl_2/{name}.pdf', dpi=300)
    plt.close(env.vis.fig)
    env.vis.new_fig()


print(f'total time = {total_time}')