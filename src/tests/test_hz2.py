from scipy.spatial import ConvexHull
import numpy as np
import matplotlib.pyplot as plt

from utils.sets.zonotopes import Zonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations

'''
Run this script to test the basic functionality of the BRS methods.
'''
colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]

op = ZonoOperations()

# Admissible set of states and inputs
n = 4   # position(x, y) and control input (u)
ng = 4; nc = 0; nb = 0
Gc = np.array([
    [10.0,  0.0, 0.0, 0.0],
    [ 0.0, 10.0, 0.0, 0.0],
    [ 0.0,  0.0, 0.8, 0.0],
    [ 0.0,  0.0, 0.0, 0.6]
])
Gb = np.zeros((n, nb))
c = np.array([ [0.0], 
               [0.0],
               [1.0],
               [0.0]
])
Ac = np.zeros((nc, ng))
Ab = np.zeros((nc, nb))
b = np.zeros((nc, 1))
X = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

# For visulization purposes we only want the first 2 dimensions
Xvis = HybridZonotope(Gc[:2, :], Gb[:2, :], c[:2, :], Ac, Ab, b)

# Target set
n = 2   # position(x, y)
ng = 2; nc = 0; nb = 0
Gc = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])
Gb = np.zeros((n, nb))
c = np.array([ [9.0], 
               [9.0]
])
Ac = np.zeros((nc, ng))
Ab = np.zeros((nc, nb))
b = np.zeros((nc, 1))
T = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

# Dynamic Model
A = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])

B = np.array([
    [1.0, 0.3],
    [0.0, 1.0]
])

D = np.block([A, B])
# one-step backward reachable set
# brs = op.one_step_brs_hz(X, T, D)
N = 2   # Horizon
brs = op.brs_hz(X, T, D, N)


# Visualize
print(f'N = {N} \t ng = {brs.ng} \t nc = {brs.nc} \t nb = {brs.nb}')
vis = ZonoVisualizer()

vis.vis_hz([Xvis, T, brs], title = 'Backward Reachable Set', colors = colors, legend_labels=['$\mathscr{X}$', '$\mathscr{T}$', '$\mathscr{BRS}$'], add_legend=True)
# vis.vis_hz([brs], title = 'Backward Reachable Set', colors = colors, legend_labels=['$\mathscr{X}$', '$\mathscr{T}$', '$\mathscr{BRS}$'], add_legend=True)
plt.show()





















