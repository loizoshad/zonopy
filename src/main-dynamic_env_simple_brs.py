'''
In this script we test the dynamic_env_simple_v2 environment. More precisely we test the computation of the BRS of the ego vehicle in that environment.
'''

import matplotlib.pyplot as plt
import numpy as np
import time

from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations

# This works for the one_step_brs_hz_v2
# from utils.environments.dynamic_env_simple import Ego
# from utils.ego_model_4d import DynamicsModel

# This works for the one_step_brs_hz
from utils.environments.dynamic_env_simple import Ego
from utils.ego_model_2d import DynamicsModel

np.set_printoptions(edgeitems=10, linewidth=10000)
obs_color = (0.949, 0.262, 0.227, 0.6)
brs_color = (0.423, 0.556, 0.749, 0.5)
marker_size = 0.25 * 39.36           # 0.2m in inches
marker_size = marker_size**2        # area of the marker
first_time = True


dynamics = DynamicsModel()

# Initialize objects
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)
ego_env = Ego(zono_op = zono_op, visualizer = vis)








# Create the space
space = ego_env.state_space(options = 'brs_outer')           # Safe state space
target = ego_env.parking(options = 'outer')
input = ego_env.input()



# ego_env.vis_background()
# vis.vis_hz_4d([space], colors = brs_color, show_edges=True, zorder=100)
# plt.show()




N = 101
for i in range(N):
    print(f'*********************************************************')
    print(f'Iteration {i}')


    # Compute BRS
    target = zono_op.one_step_brs_hz(X = space, T = target, D = np.block([dynamics.A, dynamics.B]))


    # Plot BRS
    print(f'target. : ng = {target.ng}\t nc = {target.nc}\t nb = {target.nb}')
    ego_env.vis_background()
    target_vis = HybridZonotope(target.Gc[0:2, :], target.Gb[0:2, :], target.C[0:2, :], target.Ac, target.Ab, target.b)

    start_time = time.perf_counter()
    # ego_env.grid = ego_env.vis.vis_hz_brs(hz = target_vis, brs_settings=ego_env.brs_settings)
    ego_env.grid = ego_env.vis.vis_hz_brs_v2(hz = target_vis, brs_settings=ego_env.brs_settings)
    end_time = time.perf_counter()
    print(f'Plotting time: {end_time - start_time:.4f} seconds')

    plt.show()
    ego_env.vis.ax.set_title(f'iter = {i}, ng = {target.ng}, nc = {target.nc}, nb = {target.nb}', fontsize=16)
    name = f'brs_N_{i}'
    # ego_env.vis.fig.savefig(f'./results/dynamic_env_simple/brs_outer2/{name}.pdf', dpi=300)
    
    # plt.close(ego_env.vis.fig)
    # ego_env.vis.new_fig()






