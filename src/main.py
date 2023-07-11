import matplotlib.pyplot as plt
import numpy as np

from utils.environments.dynamic_env2 import DynamicEnv2
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.ego_model_2d import DynamicsModel

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


N = 101
for i in range(N):
    print(f'*********************************************************')
    print(f'Iteration {i}')

    obs = zono_op.one_step_frs_hz(X = space, U = input, I = obs, A = env.A, B = env.B, W = env.W)
    # Complement of moving obstacle
    # z = zono_op.oa_hz_to_z(obs)
    # z = zono_op.reduce_g_z(z)

    # ## V1
    # cz = zono_op.oa_hz_to_cz(obs)
    # cz = zono_op.red_cz_scott(cz)
    # hz = zono_op.cz_to_hz(cz)
    # # hz = zono_op.intersection_hz_hz(hz, space)

    ## V2
    # hz = zono_op.red_hz_scott(obs)
    # cz = zono_op.oa_hz_to_cz_v2(hz)
    # cz = zono_op.reduce_c_cz(cz)
    # hz = zono_op.cz_to_hz(cz)
    # hz = zono_op.intersection_hz_hz(hz, space)

    ## V3
    hz = zono_op.red_hz_scott(obs)
    cz = zono_op.oa_hz_to_cz(hz)
    cz = zono_op.red_cz_scott(cz)
    hz = zono_op.cz_to_hz(cz)

    # cz = zono_op.oa_hz_to_cz(obs)
    # hz = zono_op.cz_to_hz(cz)
    # obs_compl = zono_op.complement_cz_to_hz(cz)
    # obs_compl = zono_op.intersection_hz_hz(space, obs_compl)


    print(f'Obstacle: ng = {obs.ng}\t nc = {obs.nc}\t nb = {obs.nb}')
    print(f'Obs red.: ng = {hz.ng}\t nc = {hz.nc}\t nb = {hz.nb}')
    # print(f'Obs_Comp: ng = {obs_compl.ng}\t nc = {obs_compl.nc}\t nb = {obs_compl.nb}')


    env.vis_background()
    # env.grid = env.vis.vis_hz_brs(hz = obs, brs_settings=env.brs_settings)
    env.grid = env.vis.vis_hz_brs(hz = hz, brs_settings=env.brs_settings)
    # env.grid = env.vis.vis_hz_brs(hz = obs_compl, brs_settings=env.brs_settings)

    # Save and clear figure
    name = f'frs_N_{i}'
    plt.show()
    env.vis.fig.savefig(f'./results/dynamic2/v3/{name}.pdf', dpi=300)

    plt.close(env.vis.fig)

    env.vis.new_fig()

