import matplotlib.pyplot as plt
import numpy as np
import time

from utils.environments.env3 import ParkEnv3, ParamBRS
from utils.visualization import ZonoVisualizer, AuxiliaryVisualizer
from utils.operations.operations import ZonoOperations


##############################################################################
#                               Initialize                                   #
##############################################################################

# For full
marker_size = 0.5 * 39.36           # 0.2m in inches
marker_size = marker_size**2        # area of the marker
first_time = True

# vis_1 = ZonoVisualizer()
env = ParkEnv3()

# aux_vis = AuxiliaryVisualizer(visualizer = vis_1)
op = ZonoOperations()
params = 'full'

D = np.block([env.A, env.B])


# Inner Space
road_park_1, road_park_1_vis, road_park_1_colors = env.road_park_1
road_park_2, road_park_2_vis, road_park_2_colors = env.road_park_2
road_inner, road_inner_vis, road_inner_colors = env.road_inner
road_park = op.union_hz_hz(road_park_1, road_park_2)
space_inner = op.union_hz_hz(road_inner, road_park)
target_inner, park_2_vis, park_2_colors = env.park_2
# Compute BRS starting from the inner space and target set the parking spot 2
brs_plot_params = ParamBRS(space = 'inner')
space = space_inner
target = target_inner

# Outer Space
space_outer, road_outer_vis, road_outer_colors = env.road_outer
target_outer, park_1_vis, park_1_colors = env.park_1
# Compute BRS starting from the inner space and target set the parking spot 2
# brs_plot_params = ParamBRS(space = 'outer')
# space = space_outer
# target = target_outer


# # Full
# brs_plot_params = ParamBRS(space = params)
# target = op.union_hz_hz(target_inner, target_outer)
# space = op.union_hz_hz(space_inner, space_outer)



env.vis.init_brs_plot(brs_plot_params)


N = 100
for i in range(N):
    # print(f'N: {i}')
    target = op.one_step_brs_hz(X = space, T = target, D = D)
    # print(f'Visualizing BRS {i}')
    # Visualize this time step and save the figure
    env.vis_background()
    
    # if first_time:
    #     env.ax.scatter(0.25, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     env.ax.scatter(0.35, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     env.ax.scatter(0.45, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     env.ax.scatter(0.55, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     env.ax.scatter(0.65, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     env.ax.scatter(1.85, 0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     env.ax.scatter(1.85, 0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     env.ax.scatter(1.85, -0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     env.ax.scatter(1.85, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     env.ax.scatter(1.85, -0.25, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
    #     first_time = False

    # # Plot initial_points
    # for j in range(len(brs_plot_params.initial_points)):
    #     env.ax.scatter( brs_plot_params.initial_points[j][0], brs_plot_params.initial_points[j][1], color = 'black', marker = 'x', s = 10, zorder = 10)
        
    # env.vis.vis_hz_brs(
    #     hz = target,
    #     colors = [(0.835, 0.909, 0.831, 0.5)],
    #     add_legend = False
    # )

    # Save and clear figure
    name = f'brs_N_{i}'
    plt.show()
    # env.fig.savefig(f'./results/env3/{params}/{name}.pdf', dpi=300)
    # env.fig.savefig(f'./results/env3/{params}/{name}.pdf', dpi=300)
    env.fig.savefig(f'./results/env3/brs_N_0.pdf', dpi=300)


    plt.close(env.fig)

