import matplotlib.pyplot as plt


from utils.environments.environments import SamplesHZ
from utils.operations.operations import TreeOperations
from utils.tlt.nodes import OR, AND, UNTIL, set_node
from utils.visualization import ZonoVisualizer
from utils.visualization import TreeVisualizer
from utils.tlt.tree import Tree

'''
Run this script to visualize an example of an automatically generated TLT
alongside its corresponding hybrid zonotopes. 
'''


colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # Tree (Blue)
]

hz1 = SamplesHZ().set_1
hz2 = SamplesHZ().set_2



# Tree 1
R1 = set_node(hz1, 'R1')
X11 = set_node(hz1, 'X11')
until1 = UNTIL(R1, X11)

# Tree 2
R2 = set_node(hz2, 'R2')
X21 = set_node(hz2, 'X21')
until2 = UNTIL(R2, X21)


tree1 = Tree(R1, X11, until1)
tree2 = Tree(R2, X21, until2)

# # visualize the two trees
# TreeVisualizer().vis_tree(Tree(R1, X11, until1, R2, X21, until2))


# Create a third tree by computing the OR of the two trees
or_tree = TreeOperations().attach_trees('AND', tree1, tree2)


# visualize the OR tree
TreeVisualizer().vis_tree(or_tree)


# Visualize the hybrid zonotopes of tree1, tree2, and or_tree
vis = ZonoVisualizer()

vis.vis_hz([hz1, hz2],
           title = 'Original Trees', 
           colors = colors, 
           legend_labels=['$HZ_{1}', 'HZ_{2}'],
           add_legend=True)

plt.show()


vis = ZonoVisualizer()

vis.vis_hz([or_tree.root.set],
           title = 'AND Tree', 
           colors = colors, 
           legend_labels=['Intersection'],
           add_legend=True)

plt.show()









