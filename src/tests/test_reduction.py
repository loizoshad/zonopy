import matplotlib.pyplot as plt
import numpy as np
from sympy import Matrix

from utils.environments.environments import SamplesHZ
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.environments.static_env1 import StaticEnv1
from utils.dynamics_model import DynamicsModel
from utils.environments.static_env1 import StaticEnv1, ParamBRS

def reduce_constraints_hz(hz: HybridZonotope) -> HybridZonotope:
    # Convert the matrix to a sympy Matrix object
    sympy_matrix = Matrix(np.block([
        hz.Ac, hz.Ab, hz.b
    ]))

    rref = sympy_matrix.rref()                              # Compute the row echelon form
    rref = np.array(rref[0].tolist()).astype(np.float64)    # Convert back to numpy array
    # Remove all rows of rref that are all zeros except the last column
    rref = rref[~np.all(rref[:, :-1] == 0, axis=1)]

    Ac = rref[:, :hz.Ac.shape[1]]
    Ab = rref[:, hz.Ac.shape[1]:-1]
    b = rref[:, -1]
    b = b.reshape((b.shape[0], 1))  # Reshape b to be a column vector





    return HybridZonotope(hz.Gc, hz.Gb, hz.C, Ac, Ab, b)
    

def reduce_generators_hz(hz: HybridZonotope) -> HybridZonotope:
    '''
    This method removes redundant continuous generators from a hybrid zonotope.
    '''

    threshold = 1e-7
    Gc = hz.Gc; Gb = hz.Gb
    Ac = hz.Ac; Ab = hz.Ab
    n = Gc.shape[0]; ng = Gc.shape[1]; nb = Gb.shape[1]

    ## Remove redundant continuous generators

    # Loop through all the columns of Gc
    i = 0; j = 0; k = 0
    while i < ng - k:
        g1 = Gc[:, i].reshape((n, 1))  # Get the first generator
        g1_mag = np.linalg.norm(g1)     # Magnitude of g1
        
        if np.abs(g1_mag) <= threshold:
            Gc = np.delete(Gc, i, axis=1)
            Ac = np.delete(Ac, i, axis=1)
            k += 1
            continue

        j = 0
        while j < ng - k:
            print(f'********************************************************')
            print(f'i = {i}\t j = {j}')

            if i == j:
                j += 1
                continue
            
            g2 = Gc[:, j].reshape((n, 1))  # Get the second generator

            dot_product = np.dot(g1.T, g2)  # Dot product between g1 and g2
            g2_mag = np.linalg.norm(g2)     # Magnitude of g2


            if g2_mag <= threshold:
                Gc = np.delete(Gc, j, axis=1)
                Ac = np.delete(Ac, j, axis=1)
                k += 1
                break


            if np.abs(np.abs(dot_product) - g1_mag*g2_mag) <= threshold:
                print(f'BELOW THRESHOLD DETECTED:')
                print(f'dot_product = {dot_product}')
                print(f'g1_mag = {g1_mag}\t g2_mag = {g2_mag}')
                print(f'np.abs(np.abs(dot_product) - g1_mag*g2_mag) = {np.abs(np.abs(dot_product) - g1_mag*g2_mag)}')
                print(f'before removal')
                # print(f'Detected redundant generator')
                print(f'g1 = {g1.T}\t g2 = {g2.T}')
                print(f'Gc = \n{Gc}')
                # Gc[:, i] = g1 + g2              # Add the second generator to the first
                Gc[:, i] = g1.reshape((n,)) + g2.reshape((n,))  # Add the second generator to the first
                Gc = np.delete(Gc, j, axis=1)   # Remove the second generator

                print(f'after removal')
                print(f'Gc = \n{Gc}')

                '''
                Since the constraints depend on the number of continuous generators we need to adjust the Ac matrix as well.
                '''
                Ac[:, i] = Ac[:, i] + Ac[:, j]  # Add the second generator constraint coefficients to the first
                Ac = np.delete(Ac, j, axis=1)   # Remove the second generator constraint coefficients

                k += 1
                break

            print(f'--------------------------------------------------------')
            j += 1

        i +=1




    ## Remove redundant binary generators

    # Loop through all the columns of Gb
    i = 0; j = 0; k = 0
    while i < nb - k:
        g1 = Gb[:, i].reshape((n, 1))  # Get the first generator
        g1_mag = np.linalg.norm(g1)     # Magnitude of g1
        
        if np.abs(g1_mag) <= threshold:
            Gb = np.delete(Gb, i, axis=1)
            Ab = np.delete(Ab, i, axis=1)
            k += 1
            continue

        j = 0
        while j < nb - k:
            print(f'********************************************************')
            print(f'i = {i}\t j = {j}')

            if i == j:
                j += 1
                continue
            
            g2 = Gb[:, j].reshape((n, 1))  # Get the second generator

            dot_product = np.dot(g1.T, g2)  # Dot product between g1 and g2
            g2_mag = np.linalg.norm(g2)     # Magnitude of g2


            if g2_mag <= threshold:
                Gb = np.delete(Gb, j, axis=1)
                Ab = np.delete(Ab, j, axis=1)
                k += 1
                break


            if np.abs(np.abs(dot_product) - g1_mag*g2_mag) <= threshold:
                print(f'BELOW THRESHOLD DETECTED:')
                print(f'dot_product = {dot_product}')
                print(f'g1_mag = {g1_mag}\t g2_mag = {g2_mag}')
                print(f'np.abs(np.abs(dot_product) - g1_mag*g2_mag) = {np.abs(np.abs(dot_product) - g1_mag*g2_mag)}')
                print(f'before removal')
                # print(f'Detected redundant generator')
                print(f'g1 = {g1.T}\t g2 = {g2.T}')
                print(f'Gb = \n{Gb}')
                # Gb[:, i] = g1 + g2              # Add the second generator to the first
                Gb[:, i] = g1.reshape((n,)) + g2.reshape((n,))  # Add the second generator to the first
                Gb = np.delete(Gb, j, axis=1)   # Remove the second generator

                print(f'after removal')
                print(f'Gb = \n{Gb}')

                '''
                Since the constraints depend on the number of continuous generators we need to adjust the Ac matrix as well.
                '''
                Ab[:, i] = Ab[:, i] + Ab[:, j]  # Add the second generator constraint coefficients to the first
                Ab = np.delete(Ab, j, axis=1)   # Remove the second generator constraint coefficients

                k += 1
                break

            print(f'--------------------------------------------------------')
            j += 1

        i +=1




    # ## Remove redundant binary generators

    # # Loop through all the columns of Gb
    # i = 0; j = 0; k = 0
    # while i < nb - k:
    #     g1 = Gb[:, i].reshape((n, 1))  # Get the first generator

    #     while j < nb - k:
    #         if i == j:
    #             j += 1
    #             continue

    #         g2 = Gb[:, j].reshape((n, 1))  # Get the second generator

    #         dot_product = np.dot(g1.T, g2)  # Dot product between g1 and g2
    #         g1_mag = np.linalg.norm(g1)     # Magnitude of g1
    #         g2_mag = np.linalg.norm(g2)     # Magnitude of g2

    #         if np.abs(dot_product) == g1_mag*g2_mag:
    #             Gb[:, i] = g1.reshape((n,)) + g2.reshape((n,))              # Add the second generator to the first
    #             Gb = np.delete(Gb, j, axis=1)   # Remove the second generator

    #             '''
    #             Since the constraints depend on the number of binary generators we need to adjust the Ab matrix as well.
    #             '''
    #             Ab[:, i] = Ab[:, i] + Ab[:, j]  # Add the second generator constraint coefficients to the first
    #             Ab = np.delete(Ab, j, axis=1)   # Remove the second generator constraint coefficients

    #             k += 1
    #             break

    #         j += 1
            
    #     i += 1

    return HybridZonotope(Gc, Gb, hz.C, Ac, Ab, hz.b)


def reduce_hz(hz :HybridZonotope) -> HybridZonotope:
    hz = reduce_generators_hz(hz)
    # hz = reduce_constraints_hz(hz)
    return hz

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

param = 'original'
param = 'reduced'
hz = zono_op.union_hz_hz(hz_a, hz_b)
# hz_abc = zono_op.union_hz_hz(hz_ab, hz_c)
# hz = zono_op.union_hz_hz(hz_abc, hz_d)



print(f'Before reduction \nshape of Gc: {hz.Gc.shape} (ng = {hz.ng})')
print(f'shape of Ac: {hz.Ac.shape} (nc = {hz.nc})\nshape of Ab: {hz.Ab.shape} (nb = {hz.nb})')
print(f'Gc = \n{hz.Gc}')
print(f'Ac = \n{hz.Ac}')
print(f'Ab = \n{hz.Ab}')
print(f'b = \n{hz.b}')
hz = reduce_hz(hz)
print(f'After reduction \nshape of Gc: {hz.Gc.shape} (ng = {hz.ng})')
print(f'shape of Ac: {hz.Ac.shape} (nc = {hz.nc})\nshape of Ab: {hz.Ab.shape} (nb = {hz.nb})')
print(f'Gc = \n{hz.Gc}')
print(f'Ac = \n{hz.Ac}')
print(f'Ab = \n{hz.Ab}')
print(f'b = \n{hz.b}')

# vis = ZonoVisualizer(zono_op)
# vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
# vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
# vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
# vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
# vis.ax.grid(True)
# vis.vis_hz([hz_a, hz_b], colors = colors, show_edges=True)
# plt.show()

vis.brs_plot_settings(brs_plot_params)
vis.ax.set_xlim(-0.75, 0.75); vis.ax.set_ylim(-0.4, 0.4)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)
# vis.vis_hz([hz], colors = colors, show_edges=True)
vis.ax.set_title(f'{param}', fontsize=16)
vis.vis_hz_brs(
    hz = hz,
    colors = [(0.835, 0.909, 0.831, 0.5)]
)
plt.show()















