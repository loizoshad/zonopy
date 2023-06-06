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


    def vis_hz_brs(self, hz, colors = [(0.835, 0.909, 0.831, 0.5)]):
        '''
        Visualizes a backward reachable set repsented by a hybrid zonotope
        '''
        assert hz.dim == 2, f'Hybrid Zonotope must be 2D (i.e., hz.dim = 2)'        

        self.zono_op.warm_start = None

        is_close_enough_total = 0.0
        already_contained_checks_total = 0.0

        start_time_loop = time.perf_counter()
        for x_i, x in enumerate(self.brs_sett.x_space):
            for y_i, y in enumerate(self.brs_sett.y_space):
                already_contained_checks_time = time.perf_counter()

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
                
                already_contained_checks_end = time.perf_counter()
                already_contained_checks_total = already_contained_checks_total + (already_contained_checks_end - already_contained_checks_time)


                start_time_is_close_enough = time.perf_counter()
                if close_enough:
                    # If the point made it until here, it means it needs to be checked
                    start_time = time.perf_counter()
                    if self.zono_op.is_inside_hz(hz, p):
                        self.brs_sett.is_already_contained[y_i, x_i] = 1
                        # Add the point p in the list of already contained points
                        self.brs_sett.already_contained_points = np.append(self.brs_sett.already_contained_points, p.reshape(1, -1), axis = 0)

                        # # Add to the plot a circle of radius 'step'
                        # self.ax.scatter(p[0], p[1], marker = '.', s = 350, color = '#66B2FF', alpha = 0.9, zorder = 11)
                        
                        marker_size = 0.5 * 39.36           # 0.2m in inches
                        marker_size = marker_size**2        # area of the marker

                        # Offset to the center of the cell
                        px = p[0] + self.brs_sett.x_step/2
                        py = p[1]
                        # self.ax.scatter(px, py, marker = 's', s = marker_size, color = '#66B2FF', alpha = 0.9, zorder = 11)
                        self.ax.scatter(p[0], p[1], marker = 's', s = marker_size, color = '#4F94DA', alpha = 1.0, zorder = 11,
                                        edgecolors = 'face'
                                        )



                end_time_is_close_enough = time.perf_counter()

                is_close_enough_total += end_time_is_close_enough - start_time_is_close_enough

        end_time_loop = time.perf_counter()

        full_loop_total = end_time_loop - start_time_loop

        portion_already_contained = (already_contained_checks_total)/(full_loop_total)*100
        portion = (is_close_enough_total)/(full_loop_total)*100
        
        # print(f'############################################################')
        # print(f'Full Loop time = {full_loop_total}')
        # print(f'is_close_enough total time = {is_close_enough_total}')
        # print(f'already_contained_checks total time = {already_contained_checks_total}')
        # print(f'is_close_enough portion = {portion:.2f}%')
        # print(f'already_contained_checks portion = {portion_already_contained:.2f}%')
        # print(f'############################################################')


    # Auxiliary methods:
    def vis_cz2(self, czonotopes: list, title = '', xlabel = r'$x_1$', ylabel = r'$x_2$', colors = [(0.423, 0.556, 0.749, 0.5)], legend_labels = [], add_legend = True):
        '''
        Visualizes an approximatino of the shape of a list of constrained zonotopes
        '''
        # Assert that the zonotopes are 2D
        assert all([cz.dim == 2 for cz in czonotopes]), f'Constrained Zonotope(s) must be 2D (i.e., cz.dim = 2)'    

        for cz in czonotopes:
            # start_time = time.perf_counter()
            vertices, empty = self.approx_shape(cz)
            # end_time = time.perf_counter()

            # print(f'time = {end_time - start_time}')

            if empty:
                continue

            # Fill the interior of the Polytope
            colors = np.array(colors).reshape(-1, 4)            
            random_index = np.random.randint(0, len(colors))   # Set color
            color = colors[random_index]
            poly = Polygon(vertices, closed = True, fill = True, facecolor = (color[0], color[1], color[2]), edgecolor = (color[0], color[1], color[2]), alpha = 0.6)
            self.ax.add_patch(poly)


        self.ax.set_xlim(-14, 14)
        self.ax.set_ylim(-10, 10)

        # Add legend
        if add_legend:
            self.ax.legend(legend_labels, loc='upper left')

    def approx_shape(self, cz: ConstrainedZonotope):
        '''
        This method is not really used in this work at the moment, and it should most probably be removed in the future.
        It is just needed for vis_cz2, which is itself not used.
        '''
        vertices = []
        step_angle = 10  # degrees
        step_size = 0.1 # meters
        angles = np.arange(0, 2*np.pi, step_angle*np.pi/180)

        # Loop through a list of angles between 0 and 2*pi
        for angle in angles:
            point = cz.C
            max_point = cz.C
            while True:
                step = step_size * np.array([
                    [np.cos(angle)],
                    [np.sin(angle)]
                ])
                point = point + step

                if self.zono_op.is_inside_cz(cz, point):
                    max_point = point
                else:
                    # Concatenate vertices with new max_point
                    vertices.append(max_point.reshape(-1))
                    break

        if len(vertices) == 0:
            empty = True
        else:
            empty = False   

        return vertices, empty    




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















class AuxiliaryVisualizer:

    def __init__(self, visualizer) -> None:
        self.vis = visualizer

    def vis_env(self):
        '''
        Add to the figure the park_env.pdf file
        '''

        # Read the park_env.png image
        env = image.imread('./utils/park_env.png')

        print(f'env.shape = {env.shape}')
        print(f'env = {env}')

        # Find any zero values in the env
        zero_indices = np.where(env == 0)
        print(f'zero_indices = {zero_indices}')

        # Add the image to the figure
        self.vis.self.ax.imshow(env, extent = [-2.1, 1.5, -1.0, 1.0], zorder = 1000000)


    def vis_patches(self):
        '''
        Visualizes a list of patches
        '''
        # Define a polygon for the road line
        vertices = np.array([
            [-2.1,  1.0],
            [ 1.5,  1.0],
            [ 1.5, -1.0],
            [-2.1, -1.0]
        ])

        # Create a line between the first two vertices
        road_line = Polygon(vertices, closed = True, fill = False, zorder = 12, color = 'white', linestyle = '--', linewidth = 4)
        self.vis.self.ax.add_patch(road_line)

        # Parking spot 1 line
        vertices = np.array([
            [1.910, 0.6],
            [1.910, 0.6],
            [1.910, 0.2],
            [1.910, 0.2]
        ])
        parking_spot_1_line = Polygon(vertices, closed = True, fill = False, zorder = 12, color = 'white', linestyle = '--', linewidth = 2)
        self.vis.self.ax.add_patch(parking_spot_1_line)
        # Parking spot 2 line
        vertices = np.array([
            [1.910, -0.2],
            [1.910, -0.2],
            [1.910, -0.6],
            [1.910, -0.6]
        ])
        parking_spot_2_line = Polygon(vertices, closed = True, fill = False, zorder = 12, color = 'white', linestyle = '--', linewidth = 2)
        self.vis.self.ax.add_patch(parking_spot_2_line)

        # Parking spot 3 line
        vertices = np.array([
            [-0.9, 0.6],
            [-0.5, 0.6],
            [-0.5, 0.6],
            [-0.9, 0.6]
        ])
        parking_spot_3_line = Polygon(vertices, closed = True, fill = False, zorder = 12, color = 'white', linestyle = '--', linewidth = 2)
        self.vis.self.ax.add_patch(parking_spot_3_line)
        
        # Parking spot 4 line
        vertices = np.array([
            [-0.105, 0.6],
            [0.305, 0.6],
            [-0.105, 0.6],
            [0.305, 0.6]
        ])
        parking_spot_4_line = Polygon(vertices, closed = True, fill = False, zorder = 12, color = 'white', linestyle = '--', linewidth = 2)
        self.vis.self.ax.add_patch(parking_spot_4_line)     

        # Add text to the parking spots
        self.vis.self.ax.text( 2.2,  0.4, r'$\mathbb{P}_{1}$', fontsize = 30, zorder = 12, color = 'white', horizontalalignment = 'center', verticalalignment = 'center')
        self.vis.self.ax.text( 2.2, -0.4, r'$\mathbb{P}_{2}$', fontsize = 30, zorder = 12, color = 'white', horizontalalignment = 'center', verticalalignment = 'center')
        self.vis.self.ax.text(-0.7,  0.3, r'$\mathbb{P}_{3}$', fontsize = 30, zorder = 12, color = 'white', horizontalalignment = 'center', verticalalignment = 'center', rotation = 90)
        self.vis.self.ax.text( 0.1,  0.3, r'$\mathbb{P}_{4}$', fontsize = 30, zorder = 12, color = 'white', horizontalalignment = 'center', verticalalignment = 'center', rotation = 90)

        ##############################
        # Add the road arrows        #
        ##############################
        arrow = FancyArrowPatch(
            (1.7, -0.75), # Start point
            (1.7, -0.45), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (1.7, 0.45), # Start point
            (1.7, 0.75), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        #
        self.vis.self.ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-2.3, -0.45), # Start point
            (-2.3, -0.75), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (-2.3, 0.75), # Start point
            (-2.3, 0.45), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)          
        #
        self.vis.self.ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.45, -1.2), # Start point
            (-1.15, -1.2), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (0.55, -1.2), # Start point
            (0.85, -1.2), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)    
        #      
        self.vis.self.ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.45, 0.8), # Start point
            (-1.15, 0.8), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (0.55, 0.8), # Start point
            (0.85, 0.8), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)  
        #
        self.vis.self.ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.15, 1.2), # Start point
            (-1.45, 1.2), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (0.85, 1.2), # Start point
            (0.55, 1.2), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow) 
        #
        self.vis.self.ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.15, -0.8), # Start point
            (-1.45, -0.8), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (0.85, -0.8), # Start point
            (0.55, -0.8), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)  
        #
        self.vis.self.ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.9, -0.15), # Start point
            (-1.9,  0.15), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (1.3,  0.15), # Start point
            (1.3, -0.15), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2, # Width of the arrow
            zorder = 11
        )
        self.vis.self.ax.add_patch(arrow)

        ##############################
        # Add the perimeter          #
        ##############################
        bounding_box = FancyBboxPatch(
            (-2.425, -1.325),
            width = 4.85,
            height = 2.65,
            boxstyle = 'round, pad=0.1',
            fill = False,
            ec = 'white',
            linewidth = 10,
            zorder = 14)
        self.vis.self.ax.add_patch(bounding_box)


        ##############################
        # Add the obstacles          #
        ##############################
        rect = Rectangle((1.9, 0.60),
                            width = 0.6,
                            height = 0.8,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 13
                        )
        self.vis.self.ax.add_patch(rect)

        rect = Rectangle((1.9, -0.2),
                            width = 0.6,
                            height = 0.4,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 13
                        )
        self.vis.self.ax.add_patch(rect)

        rect = Rectangle((1.9, -1.4),
                            width = 0.6,
                            height = 0.8,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 13
                        )
        self.vis.self.ax.add_patch(rect)        
        

        # INTERNAL OBSTACLES

        rect = Rectangle((-1.7, 0.0),
                            width = 0.8,
                            height = 0.6,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 13
                        )
        self.vis.self.ax.add_patch(rect)

        rect = Rectangle((-0.5, 0.0),
                            width = 0.4,
                            height = 0.6,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 13
                        )
        self.vis.self.ax.add_patch(rect)        

        rect = Rectangle((0.3, 0.0),
                            width = 0.8,
                            height = 0.6,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 13
                        )
        self.vis.self.ax.add_patch(rect)

        rect = Rectangle((-1.7, -0.6),
                            width = 2.8,
                            height = 0.6,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 13
                        )
        self.vis.self.ax.add_patch(rect)


