import matplotlib.pyplot as plt
import numpy as np

from utils.environments.dynamic_env1 import DynamicEnv1
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.ego_model_4d import DynamicsModel

marker_size = 0.5 * 39.36           # 0.2m in inches
marker_size = marker_size**2        # area of the marker
first_time = True

options = 'outer'
reduction_options = 'reduced'

# Initialize objects
dynamics = DynamicsModel()
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)
env = DynamicEnv1(zono_op = zono_op, dynamics = dynamics, visualizer = vis, options = options)

# Create the space
space = env.state_space             # Safe state space
target = env.target_space           # Free parking spots
input = env.input_space
initial_space = env.initial_space   # Initial space of non-ego vehicles

env.vis_background()

print(f'ORIGINAL Sets SET')
print(f'space: ng = {space.ng}\t nc = {space.nc}\t nb = {space.nb}')
print(f'input: ng = {input.ng}\t nc = {input.nc}\t nb = {input.nb}')
print(f'init : ng = {initial_space.ng}\t nc = {initial_space.nc}\t nb = {initial_space.nb}')

# After reduction
space = zono_op.red_hz_scott(space)
print(f'After reduction') 
print(f'space: ng = {space.ng}\t nc = {space.nc}\t nb = {space.nb}')

space = zono_op.red_hz_scott(space)
print(f'After reduction') 
print(f'space: ng = {space.ng}\t nc = {space.nc}\t nb = {space.nb}')



# For plotting
colors = [
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5),     # BRS (Blue)
    (0.423, 0.556, 0.749, 0.5)     # BRS (Blue)
]
from utils.environments.static_env1 import StaticEnv1, ParamBRS
from utils.ego_model_2d import DynamicsModel
from utils.sets.hybrid_zonotopes import HybridZonotope
dynamics2 = DynamicsModel()
brs_plot_params = ParamBRS(dynamics = dynamics2, space = 'outer')
vis.brs_plot_settings(brs_plot_params)
# vis.ax.set_xlim(-7.0, 7.0); vis.ax.set_ylim(-12, 14)
vis.ax.set_xlim(-5.0, 5.0); vis.ax.set_ylim(-2.8, 2.8)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)
vis.ax.set_title(f'ng = {space.ng}. nc = {space.nc}, nb = {space.nb}', fontsize=16)


space = HybridZonotope(space.Gc[:2,:], space.Gb[:2,:], space.C[:2,:], space.Ac, space.Ab, space.b)
vis.vis_hz([space], colors = colors, show_edges=True, zorder = 20)

plt.show()








# N = 101
# for i in range(N):
#     print(f'*********************************************************')
#     print(f'Iteration {i}')

#     # target = zono_op.one_step_brs_hz_v2(X = space, T = target, U = input, A = env.A, B = env.B)
#     initial_space = zono_op.one_step_frs_hz(X = space, U = input, I = initial_space, A = env.A, B = env.B, W = env.W)

#     A = np.block([initial_space.Ac, initial_space.Ab, initial_space.b])
#     r = np.linalg.matrix_rank(A)
#     print(f'Number of linearly independent constraints: {r}')

#     if reduction_options == 'reduced':
#         print(f'ng = {initial_space.ng}\t nc = {initial_space.nc}\t nb = {initial_space.nb}\t BEFORE')        
#         initial_space = zono_op.ua_hz(initial_space, 100)

#     print(f'ng = {initial_space.ng}\t nc = {initial_space.nc}\t nb = {initial_space.nb}')
    
#     # Print out all the constraints of the initial_space for debugging purposes
#     A = np.block([initial_space.Ac, initial_space.Ab, initial_space.b])
    
#     env.vis_background()

#     if first_time:
#         if options == 'inner':
#             env.vis.ax.scatter(0.25, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(0.35, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(0.45, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(0.55, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(0.65, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             first_time = False
#         elif options == 'outer':
#             env.vis.ax.scatter(1.85, 0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(1.85, 0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(1.85, -0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(1.85, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(1.85, -0.25, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             first_time = False
#         elif options == 'full':
#             env.vis.ax.scatter(0.25, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(0.35, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(0.45, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(0.55, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(0.65, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(1.85, 0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(1.85, 0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(1.85, -0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(1.85, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             env.vis.ax.scatter(1.85, -0.25, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
#             first_time = False


#     env.grid = env.vis.vis_hz_brs(hz =initial_space, brs_settings=env.brs_settings)


#     # Save and clear figure
#     name = f'frs_N_{i}'
#     plt.show()
#     # env.vis.fig.savefig(f'./results/dynamic1/{options}/{name}.pdf', dpi=300)
#     env.vis.fig.savefig(f'./results/dynamic1/frs1/{name}.pdf', dpi=300)

#     plt.close(env.vis.fig)

