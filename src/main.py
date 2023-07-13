import matplotlib.pyplot as plt
import numpy as np

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

space = zono_op.red_hz_scott(space)

env.vis_background()


# N = 100
# for i in range(N):
    # print(f'*********************************************************')
    # print(f'Iteration {i}')

obs = zono_op.one_step_frs_hz(X = space, U = input, I = obs, A = env.A, B = env.B, W = env.W)
obs = zono_op.one_step_frs_hz(X = space, U = input, I = obs, A = env.A, B = env.B, W = env.W)
cz = zono_op.oa_hz_to_cz(obs); cz = zono_op.reduce_c_cz(cz); cz = zono_op.reduce_g_cz(cz); hz = zono_op.cz_to_hz(cz)
cz = zono_op.red_cz_ragh_v3(cz)
obs_compl = zono_op.complement_cz_to_hz(cz)#; obs_compl = zono_op.intersection_hz_hz(obs_compl, space)


print(f'Obstacle: ng = {obs.ng}\t nc = {obs.nc}\t nb = {obs.nb}')
print(f'Obs red.: ng = {hz.ng}\t nc = {hz.nc}\t nb = {hz.nb}')
print(f'Obs_Comp: ng = {obs_compl.ng}\t nc = {obs_compl.nc}\t nb = {obs_compl.nb}')


# vis.vis_hz([hz], colors = colors, show_edges=True, zorder=20)
# vis.vis_hz([obs], colors = colors, show_edges=True, zorder=20)
# vis.vis_hz([obs_compl], colors = colors, show_edges=True, zorder=20)


# env.vis_background()
# env.grid = env.vis.vis_hz_brs(hz = obs, brs_settings=env.brs_settings)
# env.grid = env.vis.vis_hz_brs(hz = hz, brs_settings=env.brs_settings)
env.grid = env.vis.vis_hz_brs(hz = obs_compl, brs_settings=env.brs_settings)


plt.show()
# name = f'frs_N_{i}'
# env.vis.fig.savefig(f'./results/dynamic2/obs_reduced/{name}.pdf', dpi=300)
# plt.close(env.vis.fig)
# env.vis.new_fig()

