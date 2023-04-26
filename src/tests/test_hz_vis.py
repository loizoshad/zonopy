import matplotlib.pyplot as plt

import numpy as np

from utils.operations.operations import ZonoOperations
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.samples.samples import SamplesHZ
from utils.visualization import ZonoVisualizer


##############################################################################
#                               Initialize                                   #
##############################################################################
colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]
zono_op = ZonoOperations()
vis = ZonoVisualizer()
samples = SamplesHZ()

# Sets
p1 = samples.park_old_1                      # Parking spots
road = samples.roads_old                    # Road
road_vis = HybridZonotope(road.Gc[:2, :], road.Gb[:2, :], road.C[:2, :], road.Ac, road.Ab, road.b)
obs = samples.obstacles_old                 # Obstacle
state_space, _ = samples.space_old             # State space


##############################################################################
#                             Compute BRS                                    #
##############################################################################
# Dynamic Model
A = np.array([
    [1.0, 0.0],
    [0.0, 1.0]
])

B = np.array([
    [1.0, 1.0],
    [0.0, 1.0]
])

D = np.block([A, B])
N = 500
brs = zono_op.brs_hz(X = road, T = p1, D = D, N = N)

print(f'N = {N}')

print(f'ORIGINAL BRS')
print(f'BRS: ng = {brs.ng}, nc = {brs.nc}, nb = {brs.nb}')


##############################################################################
#                       Intersection with state space                        #
##############################################################################
# Intersection of BRS with state space
compute_intersection = False
if compute_intersection:
    brs = zono_op.intersection_hz_hz(brs, state_space)
    print(f'INTERSECTION BRS')
    print(f'BRS: ng = {brs.ng}, nc = {brs.nc}, nb = {brs.nb}')

##############################################################################
#                             Visualization                                  #
##############################################################################
# vis.vis_hz([obs,
#             road_vis,
#             p1,
#             brs],
#             title = 'Original Trees', 
#             colors = colors, 
#             legend_labels=['$Obs', 'Road', 'Parking', 'State space'],
#             add_legend=True)

# plt.show()










