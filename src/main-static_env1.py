import matplotlib.pyplot as plt
import numpy as np
import time

from utils.environments.static_env1 import StaticEnv1, ParamBRS
from utils.visualization import ZonoVisualizer, AuxiliaryVisualizer
from utils.operations.operations import ZonoOperations
from utils.dynamics_model import DynamicsModel


from utils.sets.hybrid_zonotopes import HybridZonotope
from sympy import Matrix

def reduce_constraints_hz(hz: HybridZonotope) -> HybridZonotope:
    # Convert the matrix to a sympy Matrix object
    sympy_matrix = Matrix(np.block([
        hz.Ac, hz.Ab, hz.b
    ]))

    rref = sympy_matrix.rref()                              # Compute the row echelon form
    rref = np.array(rref[0].tolist()).astype(np.float64)    # Convert back to numpy array
    rref = rref[~np.all(rref == 0, axis=1)]                 # Remove all zero rows

    Ac = rref[:, :hz.Ac.shape[1]]
    Ab = rref[:, hz.Ac.shape[1]:-1]
    b = rref[:, -1]
    b = b.reshape((b.shape[0], 1))  # Reshape b to be a column vector

    return HybridZonotope(hz.Gc, hz.Gb, hz.C, Ac, Ab, b)
    
def reduce_generators_hz(hz: HybridZonotope) -> HybridZonotope:
    '''
    This method removes redundant continuous generators from a hybrid zonotope.
    '''

    Gc = hz.Gc; Gb = hz.Gb
    Ac = hz.Ac; Ab = hz.Ab
    n = Gc.shape[0]; ng = Gc.shape[1]; nb = Gb.shape[1]

    ## Remove redundant continuous generators

    # Loop through all the columns of Gc
    i = 0; j = 0; k = 0
    while i < ng - k:
        g1 = Gc[:, i].reshape((n, 1))  # Get the first generator

        while j < ng - k:
            if i == j:
                j += 1
                continue
            
            g2 = Gc[:, j].reshape((n, 1))  # Get the second generator

            dot_product = np.dot(g1.T, g2)  # Dot product between g1 and g2
            g1_mag = np.linalg.norm(g1)     # Magnitude of g1
            g2_mag = np.linalg.norm(g2)     # Magnitude of g2


            if np.abs(dot_product) == g1_mag*g2_mag:
                # print(f'Detected redundant generator')
                # print(f'g1 = {g1.T}\t g2 = {g2.T}')
                # Gc[:, i] = g1 + g2              # Add the second generator to the first
                Gc[:, i] = g1.reshape((n,)) + g2.reshape((n,))  # Add the second generator to the first
                Gc = np.delete(Gc, j, axis=1)   # Remove the second generator

                '''
                Since the constraints depend on the number of continuous generators we need to adjust the Ac matrix as well.
                '''
                Ac[:, i] = Ac[:, i] + Ac[:, j]  # Add the second generator constraint coefficients to the first
                Ac = np.delete(Ac, j, axis=1)   # Remove the second generator constraint coefficients

                k += 1
                break

            j += 1

        i +=1



    ## Remove redundant binary generators

    # Loop through all the columns of Gb
    i = 0; j = 0; k = 0
    while i < nb - k:
        g1 = Gb[:, i].reshape((n, 1))  # Get the first generator

        while j < nb - k:
            if i == j:
                j += 1
                continue

            g2 = Gb[:, j].reshape((n, 1))  # Get the second generator

            dot_product = np.dot(g1.T, g2)  # Dot product between g1 and g2
            g1_mag = np.linalg.norm(g1)     # Magnitude of g1
            g2_mag = np.linalg.norm(g2)     # Magnitude of g2

            if np.abs(dot_product) == g1_mag*g2_mag:
                Gb[:, i] = g1.reshape((n,)) + g2.reshape((n,))              # Add the second generator to the first
                Gb = np.delete(Gb, j, axis=1)   # Remove the second generator

                '''
                Since the constraints depend on the number of binary generators we need to adjust the Ab matrix as well.
                '''
                Ab[:, i] = Ab[:, i] + Ab[:, j]  # Add the second generator constraint coefficients to the first
                Ab = np.delete(Ab, j, axis=1)   # Remove the second generator constraint coefficients

                k += 1
                break

            j += 1
            
        i += 1

    return HybridZonotope(Gc, Gb, hz.C, Ac, Ab, hz.b)

def reduce_hz(hz :HybridZonotope) -> HybridZonotope:
    hz = reduce_constraints_hz(hz)
    # hz = reduce_generators_hz(hz)
    return hz


##############################################################################
#                               Initialize                                   #
##############################################################################
dynamics = DynamicsModel()
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)

# EXTRA  # For full
marker_size = 0.5 * 39.36           # 0.2m in inches
marker_size = marker_size**2        # area of the marker
first_time = True

env = StaticEnv1(zono_op = zono_op, dynamics = dynamics, visualizer = vis)

params = 'outer'    # 'inner', 'outer', 'full'

D = np.block([env.A, env.B])


if params == 'inner':
    road_park_1, road_park_1_vis, road_park_1_colors = env.road_park_1
    road_park_2, road_park_2_vis, road_park_2_colors = env.road_park_2
    road_inner, road_inner_vis, road_inner_colors = env.road_inner
    road_park = zono_op.union_hz_hz(road_park_1, road_park_2)
    space_inner = zono_op.union_hz_hz(road_inner, road_park)
    target_inner, park_2_vis, park_2_colors = env.park_2    
    # Compute BRS starting from the inner space and target set the parking spot 2
    brs_plot_params = ParamBRS(dynamics = dynamics, space = 'inner')
    space = space_inner
    target = target_inner

if params == 'outer':
    space_outer, road_outer_vis, road_outer_colors = env.road_outer
    target_outer, park_1_vis, park_1_colors = env.park_1    
    # Compute BRS starting from the inner space and target set the parking spot 2
    brs_plot_params = ParamBRS(dynamics = dynamics, space = 'outer')
    space = space_outer
    target = target_outer

if params == 'full':
    # Inner Space
    road_park_1, road_park_1_vis, road_park_1_colors = env.road_park_1
    road_park_2, road_park_2_vis, road_park_2_colors = env.road_park_2
    road_inner, road_inner_vis, road_inner_colors = env.road_inner
    road_park = zono_op.union_hz_hz(road_park_1, road_park_2)
    space_inner = zono_op.union_hz_hz(road_inner, road_park)
    target_inner, park_2_vis, park_2_colors = env.park_2
    # Outer Space
    space_outer, road_outer_vis, road_outer_colors = env.road_outer
    target_outer, park_1_vis, park_1_colors = env.park_1    
    # Full
    brs_plot_params = ParamBRS(dynamics = dynamics, space = params)
    target = zono_op.union_hz_hz(target_inner, target_outer)
    space = zono_op.union_hz_hz(space_inner, space_outer)



env.vis.brs_plot_settings(brs_plot_params)


N = 11
for i in range(N):
    # print(f'N: {i}')
    target = zono_op.one_step_brs_hz(X = space, T = target, D = D)
    
    # print(f'Visualizing BRS {i}')
    # Visualize this time step and save the figure
    env.vis_background()
    
    if first_time:
        if params == 'inner':
            env.vis.ax.scatter(0.25, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.35, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.45, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.55, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.65, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            first_time = False
        elif params == 'outer':
            env.vis.ax.scatter(1.85, 0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, 0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.25, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            first_time = False
        elif params == 'full':
            env.vis.ax.scatter(0.25, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.35, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.45, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.55, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(0.65, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, 0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, 0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.05, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.15, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            env.vis.ax.scatter(1.85, -0.25, marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11, edgecolors = 'face')
            first_time = False

    # # Plot initial_points
    # for j in range(len(brs_plot_params.initial_points)):
    #     env.vis.ax.scatter( brs_plot_params.initial_points[j][0], brs_plot_params.initial_points[j][1], color = 'black', marker = 'x', s = 10, zorder = 10)
    
    print(f'Before reduction \nshape of Gc: {target.Gc.shape} (ng = {target.ng})')
    print(f'shape of Ac: {target.Ac.shape} (nc = {target.nc})\nshape of Ab: {target.Ab.shape} (nb = {target.nb})')
    target = reduce_hz(target)
    print(f'After reduction \nshape of Gc: {target.Gc.shape} (ng = {target.ng})')
    print(f'shape of Ac: {target.Ac.shape} (nc = {target.nc})\nshape of Ab: {target.Ab.shape} (nb = {target.nb})')

    env.vis.vis_hz_brs(
        hz = target,
        colors = [(0.835, 0.909, 0.831, 0.5)]
    )

    # Save and clear figure
    name = f'brs_N_{i}'
    plt.show()
    # env.vis.fig.savefig(f'./results/static1/{params}/{name}.pdf', dpi=300)
    # env.vis.fig.savefig(f'./results/testing/{params}/{name}.pdf', dpi=300)

    env.vis.fig.savefig(f'./results/testing/{params}/reduced/{name}.pdf', dpi=300)

    plt.close(env.vis.fig)







