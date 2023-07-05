import matplotlib.pyplot as plt
import numpy as np

from utils.environments.dynamic_env2 import DynamicEnv2
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.ego_model_4d import DynamicsModel
from utils.environments.samples import SamplesHZ


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
initial_space = env.initial_space   # Initial space of non-ego vehicles

space = zono_op.red_hz_scott(space)

env.vis_background()
print(f'************************************')
# print(f'{options} Environment')
# print(f'ng = {space.ng}\t nc = {space.nc}\t nb = {space.nb}')



hz = SamplesHZ().set_i
cz = zono_op.oa_hz_to_cz(hz)
# cz = ConstrainedZonotope(2*np.eye(2), np.array([[0.0], [0.0]]), np.zeros((0, 2)), np.zeros((0, 1)))

print(f'cz.ng = {cz.ng}\t cz.nc = {cz.nc}')
# cz = zono_op.reduce_gc_cz(cz)
z = zono_op.oa_cz_to_z(cz)
z = zono_op.reduce_g_z(z)
cz = zono_op.z_to_cz(z)
hz = zono_op.z_to_hz(z)
print(f'hz.ng = {hz.ng}\t hz.nc = {hz.nc}\t hz.nb = {hz.nb}')


hz_compl = zono_op.complement_cz_to_hz(cz)
hz_compl = zono_op.intersection_hz_hz(space, hz_compl)
print(f'hz_compl.ng = {hz_compl.ng}\t hz_compl.nc = {hz_compl.nc}\t hz_compl.nb = {hz_compl.nb}')







N = 101
for i in range(N):
    print(f'*********************************************************')
    print(f'Iteration {i}')

    initial_space = zono_op.one_step_frs_hz(X = space, U = input, I = initial_space, A = env.A, B = env.B, W = env.W)
    print(f'ng = {initial_space.ng}\t nc = {initial_space.nc}\t nb = {initial_space.nb}')
    
    env.vis_background()

    if first_time:
        if options == 'inner':
            env.vis.ax.scatter(0.25, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.35, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.45, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.55, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.65, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            first_time = False
        elif options == 'outer':
            env.vis.ax.scatter(1.85, 0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, 0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.25, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            first_time = False
        elif options == 'full':
            env.vis.ax.scatter(0.25, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.35, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.45, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.55, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.65, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, 0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, 0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.25, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            first_time = False


    # env.grid = env.vis.vis_hz_brs(hz = target, brs_settings=env.brs_settings)
    # env.grid = env.vis.vis_hz_brs(hz =initial_space, brs_settings=env.brs_settings)
    env.grid = env.vis.vis_hz_brs(hz = hz_compl, brs_settings=env.brs_settings)

    # Save and clear figure
    name = f'frs_N_{i}'
    plt.show()
    # env.vis.fig.savefig(f'./results/dynamic2/frs/{options}/{reduction_options}/v2/{name}.pdf', dpi=300)

    plt.close(env.vis.fig)

