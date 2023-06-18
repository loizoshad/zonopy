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



def reduce_hz(hz: HybridZonotope) -> HybridZonotope:
    '''
    Reduces the number of continuous generators of a Hybrid Zonotope.
    '''

    # threshold = 1e-7
    max_angle = 0.05 * math.pi / 180
    threshold = 1 - math.sin(max_angle)

    # Step 1: Stack Gc and Ac
    G = np.block([
        [hz.Gc],
        [hz.Ac]
    ])


    n_row = G.shape[0]; ng = G.shape[1]

    # Loop through all the rows of Gc
    i = 0; j = 0; k = 0

    while i < ng - k:
        g1 = G[:, i]
        g1_mag = np.linalg.norm(g1)    # Magnitude of g1
        g1_unit = g1 / g1_mag           # Unit vector of g1

        if np.abs(g1_mag) <= 0.001:
            G = np.delete(G, i, axis=1)
            k += 1
            continue

        j = 0
        while j < ng - k:
            if i == j:
                j += 1
                continue

            g2 = G[:, j]
            g2_mag = np.linalg.norm(g2)     # Magnitude of g2
            g2_unit = g2 / g2_mag            # Unit vector of g2

            dot_product = np.dot(g1_unit.T, g2_unit) # Dot product between g1 and g2 unit vectors

            if np.abs(dot_product) >= threshold or (g2_mag <= 0.001):
                print(f'Detected parallel generators')
                print(f'g1 = \n{g1.reshape(-1, 1)}\n g2 = \n{g2.reshape(-1, 1)}')
                G[:, i - k] = g1 + g2
                G = np.delete(G, j, axis=1)   # Remove the second generator
                k += 1

            j += 1

        i +=1

    Gc = G[:hz.dim, :]
    Ac = G[hz.dim:, :]

    return HybridZonotope(Gc, hz.Gb, hz.C, Ac, hz.Ab, hz.b)





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

hz_a = SamplesHZ().set_a
hz_b = SamplesHZ().set_b
hz_c = SamplesHZ().set_c
hz_d = SamplesHZ().set_d

hz_e = SamplesHZ().set_e
hz_f = SamplesHZ().set_f
hz_g = SamplesHZ().set_g

param = 'reduced'
hz = hz_f

# hz = zono_op.union_hz_hz(hz_a, hz_b)
# hz_abc = zono_op.union_hz_hz(hz_ab, hz_c)
# hz = zono_op.union_hz_hz(hz_abc, hz_d)



print(f'Before reduction \nshape of Gc: {hz.Gc.shape} (ng = {hz.ng})')
print(f'shape of Ac: {hz.Ac.shape} (nc = {hz.nc})\nshape of Ab: {hz.Ab.shape} (nb = {hz.nb})')
print(f'Gc = \n{hz.Gc}')
print(f'Ac = \n{hz.Ac}')
# print(f'Ab = \n{hz.Ab}')
# print(f'b = \n{hz.b}')
if param == 'reduced':
    hz = reduce_hz(hz)
print(f'After reduction \nshape of Gc: {hz.Gc.shape} (ng = {hz.ng})')
print(f'shape of Ac: {hz.Ac.shape} (nc = {hz.nc})\nshape of Ab: {hz.Ab.shape} (nb = {hz.nb})')
print(f'Gc = \n{hz.Gc}')
print(f'Ac = \n{hz.Ac}')
# print(f'Ab = \n{hz.Ab}')
# print(f'b = \n{hz.b}')




# vis = ZonoVisualizer(zono_op)
# vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
# vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
# vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
# vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
# vis.ax.grid(True)
# vis.vis_hz([hz_a, hz_b], colors = colors, show_edges=True)
# plt.show()

vis.brs_plot_settings(brs_plot_params)
vis.ax.set_xlim(-5.0, 5.0); vis.ax.set_ylim(-6, 6)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)
vis.ax.set_title(f'{param}', fontsize=16)

vis.vis_hz([hz], colors = colors, show_edges=True)
# vis.vis_hz_brs(hz = hz)

plt.show()















