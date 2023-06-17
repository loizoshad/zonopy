import time

import matplotlib.pyplot as plt
import numpy as np
from utils.environments.environments import SamplesHZ
from utils.operations.operations import TreeOperations, ZonoOperations
from utils.tlt.nodes import AND, OR, UNTIL, set_node
from utils.tlt.tree import Tree
from utils.visualization import TreeVisualizer, ZonoVisualizer

##############################################################################
#                               Initialize                                   #
##############################################################################
colors = [
    (0.949, 0.262, 0.227, 0.6),  # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),  # Road (Gray)
    (0.423, 0.556, 0.749, 0.5),  # Tree (Blue)
]
zono_op = ZonoOperations()
vis = ZonoVisualizer()
tree_op = TreeOperations()
tree_vis = TreeVisualizer()
samples = SamplesHZ()

# Sets
p1 = samples.park_old_1
p2 = samples.park_old_2  # Parking spots
road, road_vis = samples.roads_old  # Road
obs = samples.obstacles_old  # Obstacle


##############################################################################
#                              Construct TLT                                 #
##############################################################################
start_time = time.perf_counter()
## Step 1: Obtain weak-until positive normal form of LTL specification:
# # LTL = true U (p1 OR p2)

## Step 2: Construct a tree for each atomic proposition

SUBSCRIPT_1_SYMBOL = "\u2081"
SUBSCRIPT_2_SYMBOL = "\u2082"
DOUBLE_STRUCK_X_SYMBOL = "\U0001D54F"

# TLT for atomic proposition: true
ap_true = set_node(road, f"{DOUBLE_STRUCK_X_SYMBOL}")
tlt_ap_true = Tree(ap_true)

# TLT for atomic proposition: p1
ap_p1 = set_node(p1, f"p{SUBSCRIPT_1_SYMBOL}")
tlt_ap_p1 = Tree(ap_p1)

# TLT for atomic proposition: p2
ap_p2 = set_node(p2, f"p{SUBSCRIPT_2_SYMBOL}")
tlt_ap_p2 = Tree(ap_p2)


## Step 3: Construct a tree for each operator

# TLT for operator: OR : (p1 OR p2)
tree_p1_or_p2 = tree_op.attach_trees("OR", tlt_ap_p1, tlt_ap_p2)

# TLT for operator: UNTIL : (true U (p1 OR p2))
N = 100
tlt_true_until_p1orp2 = tree_op.attach_trees("UNTIL", tlt_ap_true, tree_p1_or_p2, N=N)

end_time = time.perf_counter()

print(
    f"N = {N}, \t ng = {tlt_true_until_p1orp2.root.set.ng}, \t nc = {tlt_true_until_p1orp2.root.set.nc}, \t nb = {tlt_true_until_p1orp2.root.set.nb}"
)
print(f"Elapsed time: {end_time - start_time} seconds")

#############################################################################
#                             Visualize TLT                                 #
#############################################################################
# visualize the OR tree
tree_vis.vis_tree(tlt_true_until_p1orp2)

# vis.vis_hz([obs,
#             road_vis,
#             tlt_true_until_p1orp2.root.set],
#             title = 'Original Trees',
#             colors = colors,
#             legend_labels=['$Obs', 'Road', 'Tree'],
#             add_legend=True)
# plt.show()
