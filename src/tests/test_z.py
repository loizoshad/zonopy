'''
Run this script to test all the supported Zonotope operations.
'''


import numpy as np
import matplotlib.pyplot as plt

from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.environments.environments import SamplesZ


op = ZonoOperations()
vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

A = np.array([  [1, 1],
                [1, 0]
            ])

z1 = SamplesZ().set_1
z2 = SamplesZ().set_2

z3 = op.ms_z_z(z1, z2)  # Minkowski Sum
z4 = op.lt_z(A, z3)     # Linear Transformation

vis.vis_z([z1, z2, z3, z4])
plt.show()


