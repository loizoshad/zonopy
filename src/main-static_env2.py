import matplotlib.pyplot as plt
import numpy as np

from utils.environments.static_env2 import StaticEnv2, ParamBRS
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.ego_model_2d import DynamicsModel

marker_size = 0.5 * 39.36           # 0.2m in inches
marker_size = marker_size**2        # area of the marker
first_time = True


options = 'outer'

# Initialize objects
dynamics = DynamicsModel()
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)
env = StaticEnv2(zono_op = zono_op, dynamics = dynamics, visualizer = vis, options = options)

# Create the space
space = env.state_space
target = env.target_space
input = env.input_space


N = 11
for i in range(N):
    target = zono_op.one_step_brs_hz_v2(X = space, T = target, U = input, A = env.A, B = env.B)
    
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


    env.grid = env.vis.vis_hz_brs(hz = target, brs_settings=env.brs_settings)

    # Save and clear figure
    name = f'brs_N_{i}'
    plt.show()
    # env.vis.fig.savefig(f'./results/static2/{options}/{name}.pdf', dpi=300)
    env.vis.fig.savefig(f'./results/static3/{options}/{name}.pdf', dpi=300)

    plt.close(env.vis.fig)






