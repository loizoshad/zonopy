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


'''
This test script tests the methods: 
    reduce_gc_hz
    ua_hz
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
hz = SamplesHZ().set_f


param = 'under_approximate'
N = 3



print(f'Before reduction \nshape of Gc: {hz.Gc.shape} (ng = {hz.ng})')
print(f'shape of Ac: {hz.Ac.shape} (nc = {hz.nc})\nshape of Ab: {hz.Ab.shape} (nb = {hz.nb})')
print(f'Gc = \n{hz.Gc}')
print(f'Ac = \n{hz.Ac}')
if param == 'reduced':
    hz = zono_op.reduce_gc_hz(hz)
elif param == 'under_approximate':
    hz = zono_op.ua_hz(hz, N)
print(f'After reduction \nshape of Gc: {hz.Gc.shape} (ng = {hz.ng})')
print(f'shape of Ac: {hz.Ac.shape} (nc = {hz.nc})\nshape of Ab: {hz.Ab.shape} (nb = {hz.nb})')
print(f'Gc = \n{hz.Gc}')
print(f'Ac = \n{hz.Ac}')


vis.brs_plot_settings(brs_plot_params)
vis.ax.set_xlim(-5.0, 5.0); vis.ax.set_ylim(-6, 6)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)
vis.ax.set_title(f'{param} - ng = {hz.Gc.shape[1]}', fontsize=16)

vis.vis_hz([hz], colors = colors, show_edges=True)

plt.show()















