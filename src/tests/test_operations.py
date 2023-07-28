import matplotlib.pyplot as plt
import numpy as np
import math
from sympy import Matrix

import time

from utils.environments.samples import SamplesZ, SamplesHZ, SamplesCZ
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.ego_model_4d import DynamicsModel
from utils.environments.dynamic_env_temp import DynamicEnv
from utils.environments.dynamic_env_temp import StateSpaceSafe, DynamicObstacleSpace



np.set_printoptions(edgeitems=10, linewidth=10000)

'''
This test script tests the methods: 
    oa_constr_hz_scott
'''

options = 'outer'
reduction_options = 'reduced'

# Initialize objects
dynamics = DynamicsModel()
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)
env = DynamicEnv(zono_op = zono_op, dynamics = dynamics, visualizer = vis, options = options)


colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]
obs_color = (0.949, 0.262, 0.227, 0.6)
road_color = (0.717, 0.694, 0.682, 0.5)
blue_color = (0.423, 0.556, 0.749, 0.5)







### Road
road_d = StateSpaceSafe().road_down
road_r = StateSpaceSafe().road_right
road = zono_op.union_hz_hz_v2(road_d, road_r)
# road = road_d



### Obstacles
car_d = DynamicObstacleSpace().car_outer_d
#
A = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])
B = np.array([
    [0.5, 0.0],
    [0.0, 0.5]
])
W = HybridZonotope(np.zeros((2, 2)), np.zeros((2, 0)), np.zeros((2, 1)), np.zeros((0, 2)), np.zeros((0, 0)), np.zeros((0, 1)))

# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)
# car_d = zono_op.one_step_frs_hz(X = road, U = env.input_space, I = car_d, A = A, B = B, W = W)

#
cz = zono_op.oa_hz_to_cz(car_d)
cz = zono_op.oa_cz_to_hypercube_tight(cz)
car_d = zono_op.cz_to_hz(cz)
# car_d = zono_op.intersection_hz_hz(car_d, road)


### Intersection
# inters = zono_op.intersection_hz_hz(car_d, road)



### Plot
env.vis_background()
vis.vis_hz([road], colors = road_color, show_edges=True, zorder=100)
vis.vis_hz([car_d], colors = obs_color, show_edges=True, zorder=101)
# vis.vis_hz([inters], colors = blue_color, show_edges=True, zorder=101)


plt.show()
env.vis.fig.savefig(f'./tests/plots/0.pdf', dpi=300)













