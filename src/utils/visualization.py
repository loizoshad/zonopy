import graphviz

import numpy as np
from scipy.spatial import ConvexHull
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon, FancyArrowPatch, FancyArrow, FancyBboxPatch, Rectangle
import matplotlib.pyplot as plt
import matplotlib.image as image
import itertools
import pypoman

# Supported Sets
from utils.sets.zonotopes import Zonotope
from utils.operations.operations import ZonoOperations
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
# TLT
from utils.tlt.nodes import OR, AND, UNTIL, set_node, WUNTIL

import time
import math
 



class ZonoVisualizer:
    '''
    This class allows the visualization of sets in 2D.
    '''

    min_x = 0; max_x = 0; min_y = 0; max_y = 0  # Plot limits

    def __init__(self, zono_op) -> None:
        
        self.colors = [
            (0.423, 0.556, 0.749, 0.5),
            (0.835, 0.909, 0.831, 0.5),
            (0.882, 0.835, 0.905, 0.5),
            (1.000, 0.901, 0.800, 0.5)
        ]
        self.zono_op = zono_op

        self.fig, self.ax = plt.subplots()
        self.manager = plt.get_current_fig_manager()
        self.manager.window.attributes('-zoomed', True)
        self.ax.spines['right'].set_visible(False); self.ax.spines['left'].set_visible(False)
        self.ax.spines['top'].set_visible(False); self.ax.spines['bottom'].set_visible(False)
        self.ax.get_xaxis().set_visible(False); self.ax.get_yaxis().set_visible(False)
        self.ax.set_xlim(-2.5, 2.5); self.ax.set_ylim(-1.4, 1.4)


    def save(self, path: str) -> None:
        self.fig.savefig(path, dpi=300)

    def brs_plot_settings(self, settings):
        self.brs_sett = settings


    def vis_z(self, zonotopes: list):
        '''
        Visualizes a list of zonotopes
        '''
        # Assert that the zonotopes are 2D
        assert all([z.dim == 2 for z in zonotopes]), f'Zonotope(s) must be 2D (i.e., z.dim = 2)'
        

        # Initialize the list of patches
        patches = []

        # Add the patches to the list        
        for z in zonotopes:
            vertices = z.g2v()
            patches.append(Polygon([(vertices[0, i], vertices[1, i]) for i in range(vertices.shape[1])], closed=True))

        # Choose 'len(patches)' random colors from the list of colors with replacement
        colors = []
        for i in range(len(patches)):
            random_index = np.random.randint(0, len(self.colors))
            colors.append(self.colors[random_index])

        # Add the patches to the plot
        i = 0; handles = []
        for p in patches:
            p.set_facecolor(colors[i]); p.set_edgecolor('black')
            self.ax.add_patch(p)
            handles.append(p)  # For the legend
            i += 1


    def vis_cz(self, czonotopes: list, colors = [(0.423, 0.556, 0.749, 1.0)], zorder = None, show_edges = False):
        '''
        Visualizes the exact shape of a list of constrained zonotopes
        '''
        # Assert that the zonotopes are 2D
        assert all([cz.dim == 2 for cz in czonotopes]), f'Constrained Zonotope(s) must be 2D (i.e., cz.dim = 2)'    

        for cz in czonotopes:
            vertices, _, empty = cz.g2v()

            if empty:
                continue

            # # Plot the vertices
            # self.ax.scatter(vertices[:, 0], vertices[:, 1], color='red')

            # Plot the edges
            if show_edges:
                hull = ConvexHull(vertices)
                for simplex in hull.simplices:
                    self.ax.plot(vertices[simplex, 0], vertices[simplex, 1], 'k-')

            # Fill the interior of the Polytope
            zorder = 1 if zorder is None else zorder
            color = [0.423, 0.556, 0.749, 1.0]

            poly = Polygon(vertices, closed = True, fill = True, facecolor = (color[0], color[1], color[2]),  alpha = color[3], zorder = zorder)
            self.ax.add_patch(poly)
 

    def vis_hz(self, hzonotopes: list, colors = [(0.835, 0.909, 0.831, 0.5)], zorder = None, show_edges = False):
        '''
        Visualizes a list of hybrid zonotoped

        Step 1: For each hybrid zonotope, enumerate all possible binary combinations for its binary generators.
        Step 2: Each binary combination corresponds to a constrained zonotope.
            Step 2.1: For each binary combination create an constrained zonotope object out of it.
                CZ = (hz.Gc, hz.C + hz.Gb @ weights, hz.Ac, hz.b - hz.Ab @ weights)
        Step 3: Visualize the list of constrained zonotopes.

        '''
        assert all([hz.dim == 2 for hz in hzonotopes]), f'Hybrid Zonotope(s) must be 2D (i.e., hz.dim = 2)'

        i = 0
        for hz in hzonotopes:
            cz = [] # List that will hold all the constrained zonotopes

            b_combs = np.array(list(itertools.product([-1, 1], repeat = hz.nb)))    # Step 1: Enumerate all possible binary combinations

            # Iterate over all the binary combinations
            for b in b_combs:
                b = b.reshape(-1, 1)
                cz.append(ConstrainedZonotope(hz.Gc, hz.C + hz.Gb @ b, hz.Ac, hz.b - hz.Ab @ b))
 
            self.vis_cz(cz, colors = colors[i], zorder = zorder, show_edges = show_edges)
            i += 1


    def vis_hz_brs(self, hz):
        '''
        Visualizes a backward reachable set repsented by a hybrid zonotope
        '''
        assert hz.dim == 2, f'Hybrid Zonotope must be 2D (i.e., hz.dim = 2)'        

        for x_i, x in enumerate(self.brs_sett.x_space):
            for y_i, y in enumerate(self.brs_sett.y_space):

                p = np.array([[x], [y]])

                # Check if the point 'p' is contained in any of the constrained zonotopes
                if self.brs_sett.is_already_contained[y_i, x_i] == 1:
                    continue
                
                '''
                Check if the point 'p' is close enough to any of the points that where already contained during last iteration.
                We do this because this method is used specifically for plotting backward reachable sets (brs).
                Therefore, it redundant to check points that are too far away from the brs from last iteration
                as the evolution of the brs is limited by the model dynamics.
                '''

                close_enough = False
                if 1 in self.brs_sett.is_already_contained[y_i, max(0, x_i - self.brs_sett.max_dist_y):x_i ]:
                    close_enough = True
                elif 1 in self.brs_sett.is_already_contained[y_i, x_i + 1:min(self.brs_sett.samples_y - 1, x_i + self.brs_sett.max_dist_y + 1)]:
                    close_enough = True
                elif 1 in self.brs_sett.is_already_contained[y_i + 1:min(self.brs_sett.samples_x - 1, y_i + self.brs_sett.max_dist_x + 1), x_i]:
                    close_enough = True
                elif 1 in self.brs_sett.is_already_contained[max(0, y_i - self.brs_sett.max_dist_x):y_i, x_i]:
                    close_enough = True
                else:
                    for q in range(self.brs_sett.max_dist_diag):                    
                        if self.brs_sett.is_already_contained[ min(self.brs_sett.samples_y - 1, y_i + q),   max(0, x_i - q)]:
                            close_enough = True
                            break 
                        # Move top and to the right (diagonally) of the point 'p'
                        if self.brs_sett.is_already_contained[ min(self.brs_sett.samples_y - 1, y_i + q),   min(self.brs_sett.samples_x - 1, x_i + q)]:
                            close_enough = True
                            break
                        # Move bottom and to the left (diagonally) of the point 'p'
                        if self.brs_sett.is_already_contained[ max(0, y_i - q),   max(0, x_i - q)]:
                            close_enough = True
                            break
                        # Move bottom and to the right (diagonally) of the point 'p'
                        if self.brs_sett.is_already_contained[ max(0, y_i - q),   min(self.brs_sett.samples_x - 1, x_i + q)]:
                            close_enough = True
                            break
                
                if close_enough:
                    # If the point made it until here, it means it needs to be checked
                    if self.zono_op.is_inside_hz(hz, p):
                        self.brs_sett.is_already_contained[y_i, x_i] = 1
                        # Add the point p in the list of already contained points
                        self.brs_sett.already_contained_points = np.append(self.brs_sett.already_contained_points, p.reshape(1, -1), axis = 0)
                        
                        marker_size = 0.5 * 39.36           # 0.2m in inches
                        marker_size = marker_size**2        # area of the marker

                        self.ax.scatter(p[0], p[1], marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11,
                                        edgecolors = 'face'
                                        )







class TreeVisualizer:
    '''
    This class allows the visualization of trees
    '''
    dot = graphviz.Digraph()
    
    def vis_tree(self, tree):
        '''
        Visualizes a tree
        '''
        nodes = tree.nodes
        for i, n in enumerate(nodes):
            shape = 'square' if type(n) == set_node else 'circle'
            self.dot.node(n.name, n.label, {'color': 'black', 'style': 'filled', 'shape': shape, 'fillcolor': '#F5F5F5', 'fontcolor': 'black',})

        for i, n in enumerate(nodes):
            if isinstance(n, set_node):
                if n.is_root:
                    continue
            if n.parent:
                self.dot.edge(n.parent.name, n.name)

                  
        # Save and open graph
        self.dot.render('test-output/round-table.gv')
        self.dot.view()












