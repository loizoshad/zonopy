import matplotlib.pyplot as plt
import numpy as np
import time

from utils.environments.env3 import ParkEnv3, ParamBRS
from utils.visualization import ZonoVisualizer, AuxiliaryVisualizer
from utils.operations.operations import ZonoOperations


##############################################################################
#                               Initialize                                   #
##############################################################################




# vis_1 = ZonoVisualizer()
env = ParkEnv3()

# aux_vis = AuxiliaryVisualizer(visualizer = vis_1)
op = ZonoOperations()


D = np.block([env.A, env.B])


# # Inner Space
# road_park_1, road_park_1_vis, road_park_1_colors = env.road_park_1
# road_park_2, road_park_2_vis, road_park_2_colors = env.road_park_2
# road_inner, road_inner_vis, road_inner_colors = env.road_inner
# road_park = op.union_hz_hz(road_park_1, road_park_2)
# space = op.union_hz_hz(road_inner, road_park)
# target, park_2_vis, park_2_colors = env.park_2
# # Compute BRS starting from the inner space and target set the parking spot 2
# brs_plot_params = ParamBRS(space = 'inner')


# Outer Space
road_outer, road_outer_vis, road_outer_colors = env.road_outer
space = road_outer
target, park_1_vis, park_1_colors = env.park_1
# Compute BRS starting from the inner space and target set the parking spot 2
brs_plot_params = ParamBRS(space = 'outer')



env.vis.init_brs_plot(brs_plot_params)


N = 500
for i in range(N):
    target = op.one_step_brs_hz(X = space, T = target, D = D)

    # Visualize this time step and save the figure
    env.vis_background()
    
    # # Plot initial_points
    # for j in range(len(brs_plot_params.initial_points)):
    #     env.ax.scatter( brs_plot_params.initial_points[j][0], brs_plot_params.initial_points[j][1], color = 'black', marker = 'x', s = 10, zorder = 10)
        
    env.vis.vis_hz_brs(
        hz = target,
        colors = [(0.835, 0.909, 0.831, 0.5)],
        add_legend = False
    )

    # Save and clear figure
    name = f'brs_N_{i}'
    plt.show()
    env.fig.savefig(f'./results/env3/outer/{name}.pdf', dpi=300)

    plt.close(env.fig)

