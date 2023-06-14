import matplotlib.pyplot as plt
import numpy as np
import time

from utils.environments.static_env1 import StaticEnv1, ParamBRS
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.dynamics_model_2d import DynamicsModel


##############################################################################
#                               Initialize                                   #
##############################################################################
dynamics = DynamicsModel()
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)

# EXTRA  # For full
marker_size = 0.5 * 39.36           # 0.2m in inches
marker_size = marker_size**2        # area of the marker
first_time = True

env = StaticEnv1(zono_op = zono_op, dynamics = dynamics, visualizer = vis)

params = 'inner'    # 'inner', 'outer', 'full'

D = np.block([env.A, env.B])


if params == 'inner':
    road_park_1, road_park_1_vis, road_park_1_colors = env.road_park_1
    road_park_2, road_park_2_vis, road_park_2_colors = env.road_park_2
    road_inner, road_inner_vis, road_inner_colors = env.road_inner
    road_park = zono_op.union_hz_hz(road_park_1, road_park_2)
    space_inner = zono_op.union_hz_hz(road_inner, road_park)
    target_inner, park_2_vis, park_2_colors = env.park_2    
    # Compute BRS starting from the inner space and target set the parking spot 2
    brs_plot_params = ParamBRS(dynamics = dynamics, space = 'inner')
    space = space_inner
    target = target_inner

if params == 'outer':
    space_outer, road_outer_vis, road_outer_colors = env.road_outer
    target_outer, park_1_vis, park_1_colors = env.park_1    
    # Compute BRS starting from the inner space and target set the parking spot 2
    brs_plot_params = ParamBRS(dynamics = dynamics, space = 'outer')
    space = space_outer
    target = target_outer

if params == 'full':
    # Inner Space
    road_park_1, road_park_1_vis, road_park_1_colors = env.road_park_1
    road_park_2, road_park_2_vis, road_park_2_colors = env.road_park_2
    road_inner, road_inner_vis, road_inner_colors = env.road_inner
    road_park = zono_op.union_hz_hz(road_park_1, road_park_2)
    space_inner = zono_op.union_hz_hz(road_inner, road_park)
    target_inner, park_2_vis, park_2_colors = env.park_2
    # Outer Space
    space_outer, road_outer_vis, road_outer_colors = env.road_outer
    target_outer, park_1_vis, park_1_colors = env.park_1    
    # Full
    brs_plot_params = ParamBRS(dynamics = dynamics, space = params)
    target = zono_op.union_hz_hz(target_inner, target_outer)
    space = zono_op.union_hz_hz(space_inner, space_outer)



env.vis.brs_plot_settings(brs_plot_params)


N = 11
for i in range(N):
    target = zono_op.one_step_brs_hz(X = space, T = target, D = D)
    
    env.vis_background()
    
    if first_time:
        if params == 'inner':
            env.vis.ax.scatter(0.25, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.35, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.45, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.55, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.65, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            first_time = False
        elif params == 'outer':
            env.vis.ax.scatter(1.85, 0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, 0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.25, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            first_time = False
        elif params == 'full':
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

    # # Plot initial_points
    # for j in range(len(brs_plot_params.initial_points)):
    #     env.vis.ax.scatter( brs_plot_params.initial_points[j][0], brs_plot_params.initial_points[j][1], color = 'black', marker = 'x', s = 10, zorder = 10)
    

    env.vis.vis_hz_brs(hz = target)

    # Save and clear figure
    name = f'brs_N_{i}'
    plt.show()
    # env.vis.fig.savefig(f'./results/static/{params}/{name}.pdf', dpi=300)

    plt.close(env.vis.fig)







