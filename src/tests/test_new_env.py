import matplotlib.pyplot as plt
import numpy as np

from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.environments.static_env1 import StaticEnv1
from utils.ego_model_2d import DynamicsModel
from utils.environments.dynamic_env3 import DynamicEnv3


np.set_printoptions(edgeitems=10, linewidth=10000)


colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]

# Initialize objects
dynamics = DynamicsModel()
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)
env = DynamicEnv3(zono_op = zono_op, dynamics = dynamics, visualizer = vis, options = 'inner')

space = env.state_space

print(f'ng = {space.ng}, nc = {space.nc}, nb = {space.nb}')
print(f'C = {space.C.T}')
print(f'Gc = \n{space.Gc}')
print(f'Gb = \n{space.Gb}')
print(f'Ac = \n{space.Ac}')
print(f'Ab = \n{space.Ab}')
print(f'b = {space.b.T}')


##############################################################################
#                              Original Sets                                 #
##############################################################################
n = 2; ng = 6; nc = 3; nb = 3

C = np.array([  [0.0], 
                [0.0] ])

Gc = np.array([
    [0.5, -0.5, 0.0, 0.0,  0.0, 0.0],
    [0.0,  0.0, 0.0, 0.5, -0.5, 0.0]
])

# Gb = np.zeros((n, nb))
Gb = np.array([
    [2.0, 0.0,  1.0],
    [0.0, 2.0, -1.0]
])


Ac = np.array([
    [1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0, 0.0, 1.0],
    [0.0, 1.0, 1.0, 0.0, 0.0, 0.0]
])
Ab = np.array([
    [ 0.0,  0.0,  1.0],
    [ 0.0,  0.0,  1.0],
    [ 0.0,  0.0, -1.0]
])
b = np.array([
    [1.0],
    [1.0],
    [1.0]
])


hz = HybridZonotope(Gc, Gb, C, Ac, Ab, b)




env.vis.ax.set_xlim(-4.0, 4.0); vis.ax.set_ylim(-4.0, 4.0)
env.vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
env.vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
env.vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
env.vis.ax.grid(True)
env.vis.ax.set_title(f'ng = {hz.ng}, nc = {hz.nc}, nb = {hz.nb}', fontsize=16)

env.vis.vis_hz([hz], colors = colors, show_edges=True)
# env.vis.vis_hz([space], colors = colors, show_edges=True)

plt.show()















