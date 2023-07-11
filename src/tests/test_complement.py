import matplotlib.pyplot as plt
import numpy as np
import math
from sympy import Matrix

from utils.environments.samples import SamplesHZ
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope

from utils.environments.static_env1 import StaticEnv1, ParamBRS
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.ego_model_2d import DynamicsModel



np.set_printoptions(edgeitems=10, linewidth=10000)

'''
This test script tests the functionality of computing the complement of a constrained zonotope.
The constrained zonotope is a subset a specified Hybrid Zonotope.
The result of the complement is represented as a Hybrid zonotope.

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
dynamics = DynamicsModel()
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)
env = StaticEnv1(zono_op = zono_op, dynamics = dynamics, visualizer = vis)
hz1 = SamplesHZ().set_a
hz2 = SamplesHZ().set_b

# space = HybridZonotope(5*np.eye(2), np.zeros((2, 0)), np.array([[0.0], [0.0]]), np.zeros((0, 2)), np.zeros((0, 0)), np.zeros((0, 1)))
space = ConstrainedZonotope(5*np.eye(2), np.array([[0.0], [0.0]]), np.zeros((0, 2)), np.zeros((0, 1)))

hz = zono_op.union_hz_hz(hz1, hz2)
cz = zono_op.oa_hz_to_cz(hz)
# cz = ConstrainedZonotope(2*np.eye(2), np.array([[0.0], [0.0]]), np.zeros((0, 2)), np.zeros((0, 1)))

print(f'cz.ng = {cz.ng}\t cz.nc = {cz.nc}')
# cz = zono_op.reduce_g_cz(cz)
z = zono_op.oa_cz_to_z(cz)
z = zono_op.reduce_g_z(z)
cz = zono_op.z_to_cz(z)
hz = zono_op.z_to_hz(z)
print(f'hz.ng = {hz.ng}\t hz.nc = {hz.nc}\t hz.nb = {hz.nb}')


hz_compl = zono_op.complement_cz_to_hz(cz)
print(f'hz_compl.ng = {hz_compl.ng}\t hz_compl.nc = {hz_compl.nc}\t hz_compl.nb = {hz_compl.nb}')

print(f'*************************************')


# vis.ax.set_xlim(-7.0, 7.0); vis.ax.set_ylim(-12, 14)
# vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
# vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
# vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
# vis.ax.grid(True)

# vis.vis_hz([hz, hz_compl], colors = colors, show_edges=True)
# vis.vis_hz([hz, hz_cz], colors = colors, show_edges=True)
# vis.vis_hz([hz_compl], colors = colors, show_edges=True)

# plt.show()















