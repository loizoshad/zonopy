import sys
sys.path.append('../')      # TODO: GET RID OF THIS!!!
sys.path.append('.')        # TODO: GET RID OF THIS!!!

'''
Run this script to test all the supported Zonotope operations.
'''

import numpy as np
import matplotlib.pyplot as plt

from utils.sets.zonotopes import Zonotope
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations

vis = ZonoVisualizer()
op = ZonoOperations()

c1 = np.array([ [0],
                [0]
            ])
g1 = np.array([ [1, 0],
                [0, 1]
            ])
c2 = np.array([ [0],
                [0]
            ])
g2 = np.array([ [1, 0],
                [1, 1]
            ])

A = np.array([  [1, 1],
                [1, 0]
            ])

z1 = Zonotope(c1, g1)
z2 = Zonotope(c2, g2)

# Minkowski Sum
z3 = op.ms_z_z(z1, z2)

# Linear Transformation
z4 = op.lt_z(A, z3)

vis.vis_z([z1, z2, z3, z4], title = 'Minkowski Sum', legend_labels=['$z_1$', '$z_2$', '$z_1 + z_2$', '$A(z_1 + z_2)$'])
plt.show()