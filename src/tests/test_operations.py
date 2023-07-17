import matplotlib.pyplot as plt
import numpy as np
import math
from sympy import Matrix

from utils.environments.samples import SamplesZ, SamplesHZ, SamplesCZ
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
cz1 = SamplesCZ().set_1; cz2 = SamplesCZ().set_2
cz3 = SamplesCZ().set_3; cz4 = SamplesCZ().set_4
cz = zono_op.intersection_cz_cz(cz1, cz2)
cz = zono_op.intersection_cz_cz(cz3, cz4)

cz = zono_op.redundant_c_g_cz(cz, options = 'fast')
hz = zono_op.cz_to_hz(cz)








# hz = zono_op.red_hz_scott(hz)
# cz = zono_op.oa_hz_to_cz(hz)
# cz = zono_op.red_cz_scott_v2(cz)
# hz = zono_op.cz_to_hz(cz)
# print(f'After : ng = {hz.ng}, nc = {hz.nc}, nb = {hz.nb}')













vis.brs_plot_settings(brs_plot_params)
vis.ax.set_xlim(-7.0, 7.0); vis.ax.set_ylim(-12, 14)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)
vis.ax.set_title(f'ng = {hz.ng}, nc = {hz.nc}, nb = {hz.nb}', fontsize=16)

vis.vis_hz([hz], colors = colors, show_edges=True)

plt.show()















