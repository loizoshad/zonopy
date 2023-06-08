import matplotlib.pyplot as plt
import numpy as np
from sympy import Matrix
import time

from utils.environments.static_env2 import StaticEnv2, ParamBRS
from utils.visualization import ZonoVisualizer, AuxiliaryVisualizer
from utils.operations.operations import ZonoOperations
from utils.dynamics_model import DynamicsModel
from utils.sets.hybrid_zonotopes import HybridZonotope


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

env = StaticEnv2(zono_op = zono_op, dynamics = dynamics, visualizer = vis)

params = 'outer'    # 'inner', 'outer', 'full'


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
    space_outer = env.get_state_space_outer()
    target_outer = env.get_target_space_outer()
    u_outer = env.get_input_space_outer()
    # Compute BRS starting from the inner space and target set the parking spot 1
    brs_plot_params = ParamBRS(dynamics = dynamics, space = 'outer')
    space = space_outer
    target = target_outer
    u_space = u_outer

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

    target = zono_op.one_step_brs_hz_w(X = space, U = u_space, T = target, A = env.A, B = env.B, W = env.W)

    A_b = np.block([
        target.Ac, target.Ab, target.b
    ])

    print(f'i = {i}\t g = {target.ng}\t nc = {target.nc}\t nb = {target.nb}')

    ## Remove reduntant rows

    # Convert the matrix to a sympy Matrix object
    sympy_matrix = Matrix(A_b)
    # Compute the row echelon form
    rref = sympy_matrix.rref()

    # Convert back to numpy array
    A_b = np.array(rref[0].tolist()).astype(np.float64)

    # Remove all zero rows
    A_b = A_b[~np.all(A_b == 0, axis=1)]


    # Reconstruct Ac, Ab, and b using the reduced A_b    
    Ac = A_b[:, :target.Ac.shape[1]]
    Ab = A_b[:, target.Ac.shape[1]:-1]
    b = A_b[:, -1]
    b = b.reshape((b.shape[0], 1))  # Reshape b to be a column vector

    target = HybridZonotope(target.Gc, target.Gb, target.C, Ac, Ab, b)




    # print(f'Visualizing BRS {i}')
    # Visualize this time step and save the figure
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
        
    env.vis.vis_hz_brs(
        hz = target,
        colors = [(0.835, 0.909, 0.831, 0.5)]
    )

    # Save and clear figure
    name = f'brs_N_{i}'
    plt.show()
    # env.vis.fig.savefig(f'./results/static1/{params}/{name}.pdf', dpi=300)
    env.vis.fig.savefig(f'./results/testing/{params}/{name}.pdf', dpi=300)

    plt.close(env.vis.fig)







