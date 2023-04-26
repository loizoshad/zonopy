import matplotlib.pyplot as plt

import numpy as np

from utils.operations.operations import ZonoOperations
from utils.operations.operations import TreeOperations
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.samples.samples import SamplesHZ
from utils.tlt.nodes import OR, AND, UNTIL, set_node, WUNTIL, NEXT
from utils.tlt.tree import Tree
from utils.visualization import ZonoVisualizer
from utils.visualization import TreeVisualizer

'''
This is just an example where we generate a TLT for the parking spots.
'''

##############################################################################
#                               Initialize                                   #
##############################################################################
colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot 1 (Green)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot 2 (Green)
    (0.423, 0.556, 0.749, 0.5)      # Tree (Blue)
]
zono_op = ZonoOperations()
vis = ZonoVisualizer()
tree_op = TreeOperations()
tree_vis = TreeVisualizer()
samples = SamplesHZ()

# Sets
p1 = samples.park_old_1; p2 = samples.park_old_2  # Parking spots
road, road_vis = samples.roads_old                    # Road
obs = samples.obstacles_old                 # Obstacle

##############################################################################
#                              Construct TLT                                 #
##############################################################################
## Step 1:

# Trees
r1 = set_node(p1, 'R1')
r2 = set_node(p2, 'R2')
x2 = set_node(p1, 'X2')
x3 = set_node(p2, 'X3')
u1 = UNTIL(r1, x2)
u2 = UNTIL(r2, x3)
p1_tree = Tree(r1, x2, u1)
p2_tree = Tree(r2, x3, u2)
# # visualize the two trees
# tree_vis.vis_tree(p1_tree)
# tree_vis.vis_tree(p2_tree)


##############################################################################
#                               Initialize                                   #
##############################################################################
# Create a third tree by computing the OR of the two trees
p1_or_p2_tree = tree_op.attach_trees('OR', p1_tree, p2_tree)





# visualize the OR tree
tree_vis.vis_tree(p1_or_p2_tree)
vis.vis_hz([obs,
            road_vis,
            p1_or_p2_tree.root.set],
            title = 'Original Trees', 
            colors = colors, 
            legend_labels=['$Obs', 'Road', 'p1', 'p2', 'OR Tree'],
            add_legend=True)

plt.show()










