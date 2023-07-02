import matplotlib.pyplot as plt
import numpy as np
import math
from sympy import Matrix

from utils.environments.samples import SamplesHZ
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.environments.static_env1 import StaticEnv1
from utils.ego_model_2d import DynamicsModel
from utils.environments.static_env1 import StaticEnv1, ParamBRS



np.set_printoptions(edgeitems=10, linewidth=10000)

'''
This test script tests the methods: 
    oa_constr_hz_scott
'''


colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]


##############################################################################
#                              Original Sets                                 #
##############################################################################
zono_op = ZonoOperations()
dynamics = DynamicsModel()
vis = ZonoVisualizer(zono_op)
brs_plot_params = ParamBRS(dynamics = dynamics, space = 'outer')
env = StaticEnv1(zono_op = zono_op, dynamics = DynamicsModel(), visualizer = vis)
hz1 = SamplesHZ().set_a
hz2 = SamplesHZ().set_b
hz3 = SamplesHZ().set_c

hz = zono_op.union_hz_hz(hz2, hz3)
# hz = zono_op.union_hz_hz(hz, hz3)


param = 'reduced'
max_iter = 2

print(f'*************************************')
print(f'reduced {max_iter} times')
print(f'Before reduction \nshape of Gc: {hz.Gc.shape} (ng = {hz.ng})')
print(f'shape of Ac: {hz.Ac.shape} (nc = {hz.nc})\nshape of Ab: {hz.Ab.shape} (nb = {hz.nb})')
if param == 'reduced':
    for i in range(max_iter):
        hz = zono_op.red_hz_scott(hz)

print(f'After reduction \nshape of Gc: {hz.Gc.shape} (ng = {hz.ng})')
print(f'shape of Ac: {hz.Ac.shape} (nc = {hz.nc})\nshape of Ab: {hz.Ab.shape} (nb = {hz.nb})')



vis.brs_plot_settings(brs_plot_params)
vis.ax.set_xlim(-7.0, 7.0); vis.ax.set_ylim(-12, 14)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)
vis.ax.set_title(f'{param} - ng = {hz.nc}', fontsize=16)

vis.vis_hz([hz], colors = colors, show_edges=True)

plt.show()















