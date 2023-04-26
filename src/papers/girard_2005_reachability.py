'''
This code is an attempt to replicate the work of Antoine Girard in his paper
'Reachability of Uncertain Linear Systems Using Zonotopes, 2005'
'''

import sys
sys.path.append('../')      # TODO: GET RID OF THIS!!!
sys.path.append('.')        # TODO: GET RID OF THIS!!!


import numpy as np
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt

from utils.sets.zonotopes import Zonotope
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.operations.fra import FRA

vis = ZonoVisualizer()
fra = FRA()

# Dynamics model: x_dot = Ax + u, inf_norm(u) <= mu
dt = 0.02; max_ord = 5; mu = 0.05; T = 2; solver = 'ag'
fra.solver_options(mu = mu, dt = dt, max_ord = max_ord, T = T, solver = solver)

A = np.array([  [-1.0, -4.0],
                [ 4.0, -1.0] ])

# Initial set: z0 = [0.9, 1.1] X [-0.1, 0.1]
c0 = np.array([ [1.0], [0.0] ])
g0 = np.array([ [0.1, 0.0],
                [0.0, 0.1] ])
z0 = Zonotope(c0, g0)

# Solve the FRA problem
R, R_approx = fra.solve(A, z0)

# Visualize the reachable set at each time step
reach_sets = [z0]
labels = ['I']
# Loop through every other time step
N = int(T/dt)
for i in range(0, N, 1):
    reach_sets.append(R[i])
    labels.append(f'R[{i + 1}]')

vis.vis_z(zonotopes = reach_sets, title='Girard (2005) Forward Reachability Analysis', legend_labels = labels, add_legend = False)
