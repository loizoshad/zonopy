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



def reduce_gc_hz(hz: HybridZonotope) -> HybridZonotope:
    '''
    Reduces the number of continuous generators of a Hybrid Zonotope.
    '''

    # threshold = 1e-7
    max_angle = 0.05 * math.pi / 180
    # max_angle = 0.5 * math.pi / 180
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
                G[:, i - k] = g1 + g2
                G = np.delete(G, j, axis=1)   # Remove the second generator
                k += 1

            j += 1

        i +=1

    Gc = G[:hz.dim, :]
    Ac = G[hz.dim:, :]

    return HybridZonotope(Gc, hz.Gb, hz.C, Ac, hz.Ab, hz.b)


def under_approximate_hz(hz: HybridZonotope, N) -> HybridZonotope:
    '''
    Reduces the number of continuous generators of a Hybrid Zonotope.

    This method first removes all parallel generators of the lifted hybrid zonotope.

    Then it makes use of the work in [1] to further reduce the number of continuous generators
    by under-approximation of maximum N generators
    '''

    # max_angle = 0.05 * math.pi / 180
    max_angle = 0.5 * math.pi / 180
    threshold = 1 - math.sin(max_angle)

    # Step 1: Stack Gc and Ac
    G = np.block([
        [hz.Gc],
        [hz.Ac]
    ])

    ng = G.shape[1]

    # Loop through all the rows of Gc
    i = 0; j = 0; k = 0


    objective = []

    while i < ng:
        g1 = G[:, i]
        g1_mag = np.linalg.norm(g1)     # Magnitude of g1
        g1_unit = g1 / g1_mag           # Unit vector of g1

        j = 0
        while j < ng:
            if i == j:
                j += 1
                continue

            g2 = G[:, j]

            objective.append( ( g1_mag * np.linalg.norm(g2 - np.dot(g1_unit, np.dot(g2.T, g1_unit)) ),
                                i,
                                j))

            j += 1
        i +=1

    # Sort the objective list from min to max
    objective = sorted(objective, key=lambda x: x[0])   # This has a cost of O(n log n)


    remaining_indices = set(range(G.shape[1]))  # Set to keep track of the remaining generator indices
    updated_generators = []                     # List to store the updated generators

    # Perform generator elimination until 'N' generators are left
    while len(remaining_indices) > N:
        # Find the pair with the smallest objective value
        _, i, j = objective.pop(0)

        # Check if generators still exist
        if i in remaining_indices and j in remaining_indices:
            # Replace g1 by g1 + g2
            G[:, i] += G[:, j]
            remaining_indices.remove(j)

    # Create the updated matrix with the remaining vectors
    for i in range(G.shape[1]):
        if i in remaining_indices:
            updated_generators.append(G[:, i])

    G = np.column_stack(updated_generators)

    Gc = G[:hz.dim, :]
    Ac = G[hz.dim:, :]

    return HybridZonotope(Gc, hz.Gb, hz.C, Ac, hz.Ab, hz.b)

def under_approximate_hz_v2(hz: HybridZonotope, N) -> HybridZonotope:
    '''
    TODO:

    THIS IS SUPPOSED TO BE A FASTER VERSION

    Reduces the number of continuous generators of a Hybrid Zonotope.

    This method first removes all parallel generators of the lifted hybrid zonotope.

    Then it makes use of the work in [1] to further reduce the number of continuous generators
    by under-approximation of maximum N generators
    '''

    # max_angle = 0.05 * math.pi / 180
    max_angle = 0.5 * math.pi / 180
    threshold = 1 - math.sin(max_angle)

    # Step 1: Stack Gc and Ac
    G = np.block([
        [hz.Gc],
        [hz.Ac]
    ])

    ng = G.shape[1]


    gen_norms = np.linalg.norm(G, axis=0)   # Compute the norms of all generators
    normalized_G = G / gen_norms            # Compute the normalized generator matrix




    # Loop through all the rows of Gc
    i = 0; j = 0; k = 0

    objective = []

    while i < ng:
        g1 = G[:, i]
        g1_mag = np.linalg.norm(g1)     # Magnitude of g1
        g1_unit = g1 / g1_mag           # Unit vector of g1

        j = 0
        while j < ng:
            if i == j:
                j += 1
                continue

            g2 = G[:, j]

            objective.append( ( g1_mag * np.linalg.norm(g2 - np.dot(g1_unit, np.dot(g2.T, g1_unit)) ),
                                i,
                                j))

            j += 1
        i +=1

    # Sort the objective list from min to max
    objective = sorted(objective, key=lambda x: x[0])   # This has a cost of O(n log n)


    remaining_indices = set(range(G.shape[1]))  # Set to keep track of the remaining generator indices
    updated_generators = []                     # List to store the updated generators

    # Perform generator elimination until 'N' generators are left
    while len(remaining_indices) > N:
        # Find the pair with the smallest objective value
        _, i, j = objective.pop(0)

        # Check if generators still exist
        if i in remaining_indices and j in remaining_indices:
            # Replace g1 by g1 + g2
            G[:, i] += G[:, j]
            remaining_indices.remove(j)

    # Create the updated matrix with the remaining vectors
    for i in range(G.shape[1]):
        if i in remaining_indices:
            updated_generators.append(G[:, i])

    G = np.column_stack(updated_generators)

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
hz = SamplesHZ().set_f


param = 'under_approximate_hz'
N = 5



print(f'Before reduction \nshape of Gc: {hz.Gc.shape} (ng = {hz.ng})')
print(f'shape of Ac: {hz.Ac.shape} (nc = {hz.nc})\nshape of Ab: {hz.Ab.shape} (nb = {hz.nb})')
print(f'Gc = \n{hz.Gc}')
print(f'Ac = \n{hz.Ac}')
if param == 'reduced':
    hz = reduce_gc_hz(hz)
elif param == 'under_approximate':
    hz = under_approximate_hz(hz, N)
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















