import matplotlib.pyplot as plt
import numpy as np
import time

from utils.environments.environments import SamplesHZ, SamplesVis, ParkEnv1, ParkEnv2
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations

'''
Run this script to test the first environment
'''

##############################################################################
#                               Initialize                                   #
##############################################################################
# Dynamic Model
'''
We are aiming for a velocity of 0.5 m/s. And a time step of 0.2 seconds.
So it means that to get 0.5 m/s we need to move 0.1 m in each time step
(Actually at the moment I am using 0.05 m in each time step)
'''

A = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])

B = np.array([
    [0.05, 0.0],
    [0.0, 0.05]
])


D = np.block([A, B])

for N in range(0, 101):

    env_1 = ParkEnv2(road='outer')
    vis = ZonoVisualizer()
    op_1 = ZonoOperations(visualizer = vis)

    hz_1, hz_vis, colors_1 = env_1.get_sets()
    road_1 = hz_1[0]; parking_1 = hz_1[1]

    env_2 = ParkEnv2(road='inner')
    op_2 = ZonoOperations(visualizer = vis)

    hz_2, hz_vis_2, colors_2 = env_2.get_sets()
    road_2 = hz_2[0]; parking_2 = hz_2[1]

    hz_vis = [hz_vis[0], hz_vis_2[0], hz_vis[1], hz_vis_2[1]]
    colors = [colors_1[0], colors_2[0], colors_1[1], colors_2[1]]
    ##############################################################################
    #                                  BRS                                       #
    ##############################################################################

    start_time_total = time.perf_counter()

    # OUTER ROAD
    print(f'### HORIZON N: {N}')
    print(f'- OUTER ROAD:')
    start_time = time.perf_counter()
    brs_1 = op_1.brs_hz(X = road_1, T = parking_1, D = D, N = N, visualize = True, env = env_1)
    end_time = time.perf_counter()
    print(f'  - ng = {brs_1.ng} \t nc = {brs_1.nc} \t nb = {brs_1.nb}')
    print(f'  - Compute and plot BRS-OUTER took: {end_time - start_time} seconds')

    # INNER ROAD
    print(f'- INNER ROAD:')
    start_time = time.perf_counter()
    brs_2 = op_2.brs_hz(X = road_2, T = parking_2, D = D, N = N, visualize = True, env = env_2)
    end_time = time.perf_counter()
    print(f'  - ng = {brs_2.ng} \t nc = {brs_2.nc} \t nb = {brs_2.nb}')
    print(f'  - Compute and plot BRS-INNER took: {end_time - start_time} seconds')

    end_time_total = time.perf_counter()
    print(f'- Total time: {end_time_total - start_time_total} seconds')


    # Visualize Environment
    vis.vis_hz(hz_vis,
            title = 'Environment', 
            colors = colors, 
            legend_labels=['$\mathscr{O}$', '$\mathscr{x}$', '$\mathscr{P}$'],
            add_legend=False)




    vis.vis_patches()
    vis.vis_images()       
    # plt.grid(False)


    # # Save the figure
    # name = f'_brs_N_{N}'
    # # Set the size of the figure to be saved
    # plt.gcf().set_size_inches(15, 8)
    # plt.savefig(f'./results/env2/{name}.pdf', dpi=300)

    vis.plot_things(N)

    # Delete all objects
    del env_1, env_2, vis, op_1, op_2



# plt.show()

