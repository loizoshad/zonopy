import graphviz

import numpy as np
from scipy.spatial import ConvexHull
from matplotlib.collections import PatchCollection
from matplotlib.patches import Polygon, FancyArrowPatch, FancyArrow, FancyBboxPatch, Rectangle
import matplotlib.pyplot as plt
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
 

fig, ax = plt.subplots()        # Initialize the plot
manager = plt.get_current_fig_manager()
manager.window.attributes('-zoomed', True)        
ax.grid()                          # Add a grid
ax.spines['left'].set_edgecolor('white')
ax.spines['bottom'].set_edgecolor('white')
ax.spines['right'].set_visible(False)
ax.spines['top'].set_visible(False)

class ZonoVisualizer:
    '''
    This class allows the visualization of sets in 2D.
    '''

    min_x = 0; max_x = 0; min_y = 0; max_y = 0  # Plot limits

    def __init__(self) -> None:
        
        self.colors = [
            (0.423, 0.556, 0.749, 0.5),
            (0.835, 0.909, 0.831, 0.5),
            (0.882, 0.835, 0.905, 0.5),
            (1.000, 0.901, 0.800, 0.5)
        ]
        self.zono_op = ZonoOperations()         # Initialize the zonotope operations class
        # self.init_brs_plot()                    # Initialize the BRS plot environment

        ax.set_xlim(-2.5, 2.5)
        ax.set_ylim(-1.4, 1.4)
        # ax.set_xlim(-14, 14)
        # ax.set_ylim(-10, 10)

    def init_brs_plot(self, env):
        #####################################################################
        #                           Initialization                          #
        #####################################################################        
        if env is not None:
            x_min = env.x_min; x_max = env.x_max; y_min = env.y_min; y_max = env.y_max
            self.samples_x = env.samples_x; self.samples_y = env.samples_y
            self.already_contained_points = env.initial_points
            self.x_step = env.x_step; self.y_step = env.y_step
            self.max_dist_x = env.max_dist_x; self.max_dist_y = env.max_dist_y; self.max_dist_diag = env.max_dist_diag

            # Discretize the x-y state space
            self.x_space = np.linspace(x_min, x_max, self.samples_x)
            self.y_space = np.linspace(y_min, y_max, self.samples_y)          

            # Associated flag for already contained points
            self.is_already_contained = np.zeros((self.samples_y, self.samples_x))

            # Update the flag for already contained points
            for p in self.already_contained_points:
                x_idx = np.argmin(np.abs(self.x_space - p[0]))
                y_idx = np.argmin(np.abs(self.y_space - p[1]))
                self.is_already_contained[y_idx, x_idx] = 1


        else:   # Default environment
            x_min = -7.9; x_max = 7.9; y_min = -8.9; y_max = 8.9; self.samples_x = 79; self.samples_y = 89

            # Discretize the x-y state space
            self.x_space = np.linspace(x_min, x_max, self.samples_x)
            self.y_space = np.linspace(y_min, y_max, self.samples_y)
            self.x_step = (x_max - x_min) / (self.samples_x)
            self.y_step = (y_max - y_min) / (self.samples_y)

            # Associated flag for already contained points
            self.is_already_contained = np.zeros((self.samples_y, self.samples_x))

            # Initialize vertex list
            # TODO: First implement the functionality to obtain the vertices of the target sets (i.e., the parking spots)
            self.already_contained_points = np.array([
                [8.0, 9.0],
                [8.0, 6.5],
                [8.0, -9.0],
                [8.0, -6.5]
            ])        
            # Update the flag for already contained points
            for p in self.already_contained_points:
                x_idx = np.argmin(np.abs(self.x_space - p[0]))
                y_idx = np.argmin(np.abs(self.y_space - p[1]))
                self.is_already_contained[y_idx, x_idx] = 1

            # Add all the points between points [8.0, 9.0] and [8.0, 6.5] as already contained
            for i in range(1, 6):
                x_idx = np.argmin(np.abs(self.x_space - 8.0))
                y_idx = np.argmin(np.abs(self.y_space - (9.0 - i * 0.5)))
                self.is_already_contained[y_idx, x_idx] = 1

            # Add all the points between points [8.0, -9.0] and [8.0, -6.5] as already contained
            for i in range(1, 6):
                x_idx = np.argmin(np.abs(self.x_space - 8.0))
                y_idx = np.argmin(np.abs(self.y_space - (-9.0 + i * 0.5)))
                self.is_already_contained[y_idx, x_idx] = 1

            # Auxiliary variables
            self.max_dist = 1.0  # Assumed maximum possible propagation distance of the brs in a single iteration
            self.max_dist_x = math.ceil(self.max_dist / self.x_step) # Maximum number of steps in the x direction
            self.max_dist_y = math.ceil(self.max_dist / self.y_step) # Maximum number of steps in the y direction
            self.max_dist_diag = math.ceil( 1.1 * self.max_dist_x)




    def vis_z(self, zonotopes: list, title = '', xlabel = r'$x_1$', ylabel = r'$x_2$', legend_labels = [], add_legend = True):
        '''
        Visualizes a list of zonotopes
        '''
        # Assert that the zonotopes are 2D
        assert all([z.dim == 2 for z in zonotopes]), f'Zonotope(s) must be 2D (i.e., z.dim = 2)'
        
        # self.ax.set_title(title)                # Add the title
        # self.ax.set_xlabel(f'State - {xlabel}') # Add the x label
        # self.ax.set_ylabel(f'State - {ylabel}') # Add the y label
        # self.ax.grid()                          # Add a grid        

        # Initialize the list of patches
        patches = []

        # Add the patches to the list        
        for z in zonotopes:
            vertices = z.g2v()
            polygon = Polygon([(vertices[0, i], vertices[1, i]) for i in range(vertices.shape[1])], closed=True)
            patches.append(polygon)

            # Find the min and max of the vertices of all zonotopes to set the axis limits
            self.min_x = min(np.min(vertices[0, :]), self.min_x); self.max_x = max(np.max(vertices[0, :]), self.max_x)
            self.min_y = min(np.min(vertices[1, :]), self.min_y); self.max_y = max(np.max(vertices[1, :]), self.max_y)

        x_margin = 0.2 * (self.max_x - self.min_x); y_margin = 0.2 * (self.max_y - self.min_y)
        ax.set_xlim(self.min_x - x_margin, self.max_x + x_margin)
        ax.set_ylim(self.min_y - y_margin, self.max_y + y_margin)


        # Choose 'len(patches)' random colors from the list of colors with replacement
        colors = []
        for i in range(len(patches)):
            random_index = np.random.randint(0, len(self.colors))
            colors.append(self.colors[random_index])

        # Add the patches to the plot
        i = 0; labels = []; handles = []
        for p in patches:
            p.set_facecolor(colors[i]); p.set_edgecolor('black')
            ax.add_patch(p)
            if add_legend:
                labels.append(legend_labels[i])
            handles.append(p)  # For the legend
            i += 1

        if add_legend:
            ax.legend(handles, labels, loc='upper left')    # Add the legend
        
        # plt.show()

    def vis_cz(self, czonotopes: list, title = '', xlabel = r'$x_1$', ylabel = r'$x_2$', colors = [(0.423, 0.556, 0.749, 0.5)], legend_labels = [], add_legend = True):
        '''
        Visualizes the exact shape of a list of constrained zonotopes
        '''
        # Assert that the zonotopes are 2D
        assert all([cz.dim == 2 for cz in czonotopes]), f'Constrained Zonotope(s) must be 2D (i.e., cz.dim = 2)'    

        for cz in czonotopes:
            # start_time = time.perf_counter()
            vertices, _, empty = cz.g2v()
            # end_time = time.perf_counter()

            # print(f'time = {end_time - start_time}')

            if empty:
                continue

            # # Plot the vertices
            # ax.scatter(vertices[:, 0], vertices[:, 1], color='red')

            # # Plot the edges
            # hull = ConvexHull(vertices)
            # for simplex in hull.simplices:
            #     ax.plot(vertices[simplex, 0], vertices[simplex, 1], 'k-')


            # Fill the interior of the Polytope
            colors = np.array(colors).reshape(-1, 4)            
            random_index = np.random.randint(0, len(colors))   # Set color
            color = colors[random_index]
            poly = Polygon(vertices, closed = True, fill = True, facecolor = (color[0], color[1], color[2]),  alpha = color[3])
            ax.add_patch(poly)

        #     # Find the min and max of the vertices of all zonotopes to set the axis limits
        #     self.min_x = min(np.min(vertices[:, 0]), self.min_x); self.max_x = max(np.max(vertices[:, 0]), self.max_x)
        #     self.min_y = min(np.min(vertices[:, 1]), self.min_y); self.max_y = max(np.max(vertices[:, 1]), self.max_y)

        # x_margin = 0.2 * (self.max_x - self.min_x); y_margin = 0.2 * (self.max_y - self.min_y)
        # ax.set_xlim(self.min_x - x_margin, self.max_x + x_margin)
        # ax.set_ylim(self.min_y - y_margin, self.max_y + y_margin)       

        # Add legend
        if add_legend:
            ax.legend(legend_labels, loc='upper left')    

        ax.set_title(title)    

        # plt.show()

    def vis_hz(self, hzonotopes: list, title = '', xlabel = r'$x_{1}', ylabel = r'x_{2}', colors = [(0.835, 0.909, 0.831, 0.5)], legend_labels = [], add_legend = True):
        '''
        Visualizes a list of hybrid zonotoped

        Step 1: For each hybrid zonotope, enumerate all possible binary combinations for its binary generators.
        Step 2: Each binary combination corresponds to a constrained zonotope.
            Step 2.1: For each binary combination create an constrained zonotope object out of it.
                CZ = (hz.Gc, hz.C + hz.Gb @ weights, hz.Ac, hz.b - hz.Ab @ weights)
        Step 3: Visualize the list of constrained zonotopes.

        '''
        assert all([hz.dim == 2 for hz in hzonotopes]), f'Hybrid Zonotope(s) must be 2D (i.e., hz.dim = 2)'

        # cz = [] # List that will hold all the constrained zonotopes
        i = 0
        for hz in hzonotopes:
            cz = [] # List that will hold all the constrained zonotopes

            # Step 1: Enumerate all possible binary combinations for the binary generators hz.nb
            b_combs = np.array(list(itertools.product([-1, 1], repeat = hz.nb)))
            # weights = np.array([-1 if x == -1 else 1 for x in b_combs[i]]).reshape(-1, 1)
            
            # Iterate over all the binary combinations
            # start_time = time.perf_counter()
            for b in b_combs:
                b = b.reshape(-1, 1)
                # Step 2.1: Create a constrained zonotope object out of the binary combination
                cz.append(ConstrainedZonotope(hz.Gc, hz.C + hz.Gb @ b, hz.Ac, hz.b - hz.Ab @ b))
            self.vis_cz(cz, title = title, xlabel = r'qq', ylabel = r'yy', colors = colors[i], legend_labels = legend_labels, add_legend = add_legend)
            # end_time = time.perf_counter()
            # print(f'Decomp+Vis time = {end_time - start_time}')
            i = i + 1

            # Just making the plots a bit more beautiful
            vert1 = np.array([[-11, 9], [-11, 10], [11, 10], [11, 9]])
            vert2 = np.array([[-11, -9], [-11, -10], [11, -10], [11, -9]])
            poly = Polygon(vert1, closed = True, fill = True, facecolor = 'white', alpha = 1.0)
            ax.add_patch(poly)
            poly = Polygon(vert2, closed = True, fill = True, facecolor = 'white', alpha = 1.0)
            ax.add_patch(poly)

    def vis_hz_brs(self, hz, title = '', xlabel = r'$x_{1}', ylabel = r'x_{2}', colors = [(0.835, 0.909, 0.831, 0.5)], legend_labels = [], add_legend = True):
        '''
        Visualizes a backward reachable set repsented by a hybrid zonotope
        '''
        assert hz.dim == 2, f'Hybrid Zonotope must be 2D (i.e., hz.dim = 2)'        

        self.zono_op.warm_start = None

        for x_i, x in enumerate(self.x_space):
            for y_i, y in enumerate(self.y_space):
                p = np.array([[x], [y]])

                # print(f'x = {x}, y = {y}')

                # Check if the point 'p' is contained in any of the constrained zonotopes
                if self.is_already_contained[y_i, x_i] == 1:
                    continue
                
                '''
                Check if the point 'p' is close enough to any of the points that where already contained during last iteration.
                We do this because this method is used specifically for plotting backward reachable sets (brs).
                Therefore, it redundant to check points that are too far away from the brs from last iteration
                as the evolution of the brs is limited by the model dynamics.
                '''
                close_enough = False
                if 1 in self.is_already_contained[y_i, max(0, x_i - self.max_dist_y):x_i ]:
                    close_enough = True
                elif 1 in self.is_already_contained[y_i, x_i + 1:min(self.samples_y - 1, x_i + self.max_dist_y + 1)]:
                    close_enough = True
                elif 1 in self.is_already_contained[y_i + 1:min(self.samples_x - 1, y_i + self.max_dist_x + 1), x_i]:
                    close_enough = True
                elif 1 in self.is_already_contained[max(0, y_i - self.max_dist_x):y_i, x_i]:
                    close_enough = True
                else:
                    for q in range(self.max_dist_diag):                    
                        if self.is_already_contained[ min(self.samples_y - 1, y_i + q),   max(0, x_i - q)]:
                            close_enough = True
                            break 
                        # Move top and to the right (diagonally) of the point 'p'
                        if self.is_already_contained[ min(self.samples_y - 1, y_i + q),   min(self.samples_x - 1, x_i + q)]:
                            close_enough = True
                            break
                        # Move bottom and to the left (diagonally) of the point 'p'
                        if self.is_already_contained[ max(0, y_i - q),   max(0, x_i - q)]:
                            close_enough = True
                            break
                        # Move bottom and to the right (diagonally) of the point 'p'
                        if self.is_already_contained[ max(0, y_i - q),   min(self.samples_x - 1, x_i + q)]:
                            close_enough = True
                            break
                    
                if close_enough:
                    # If the point made it until here, it means it needs to be checked
                    if self.zono_op.is_inside_hz(hz, p):
                        self.is_already_contained[y_i, x_i] = 1
                        # Add the point p in the list of already contained points
                        self.already_contained_points = np.append(self.already_contained_points, p.reshape(1, -1), axis = 0)

                        # Add to the plot a circle of radius 'step'
                        plt.scatter(p[0], p[1], marker = '.', s = 400, color = '#4783FC', alpha = 1.0)

                        # Just to fill in the gaps between the circles
                        plt.scatter(p[0], p[1] + self.y_step/3, marker = '.', s = 400, color = '#4783FC', alpha = 1.0) 
                        plt.scatter(p[0] - self.x_step/3, p[1], marker = '.', s = 400, color = '#4783FC', alpha = 1.0)




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
        road_line = Polygon(vertices, closed = True, fill = False, color = 'white', linestyle = '--', linewidth = 4)
        ax.add_patch(road_line)

        # Parking spot 1 line
        vertices = np.array([
            [1.910, 0.6],
            [1.910, 0.6],
            [1.910, 0.2],
            [1.910, 0.2]
        ])
        parking_spot_1_line = Polygon(vertices, closed = True, fill = False, color = 'white', linestyle = '--', linewidth = 2)
        ax.add_patch(parking_spot_1_line)
        # Parking spot 2 line
        vertices = np.array([
            [1.910, -0.2],
            [1.910, -0.2],
            [1.910, -0.6],
            [1.910, -0.6]
        ])
        parking_spot_2_line = Polygon(vertices, closed = True, fill = False, color = 'white', linestyle = '--', linewidth = 2)
        ax.add_patch(parking_spot_2_line)

        # Parking spot 3 line
        vertices = np.array([
            [-0.9, 0.6],
            [-0.5, 0.6],
            [-0.5, 0.6],
            [-0.9, 0.6]
        ])
        parking_spot_3_line = Polygon(vertices, closed = True, fill = False, color = 'white', linestyle = '--', linewidth = 2)
        ax.add_patch(parking_spot_3_line)
        
        # Parking spot 4 line
        vertices = np.array([
            [-0.105, 0.6],
            [0.305, 0.6],
            [-0.105, 0.6],
            [0.305, 0.6]
        ])
        parking_spot_4_line = Polygon(vertices, closed = True, fill = False, color = 'white', linestyle = '--', linewidth = 2)
        ax.add_patch(parking_spot_4_line)     

        # Add text to the parking spots
        ax.text( 2.2,  0.4, r'$\mathbb{P}_{1}$', fontsize = 30, color = 'white', horizontalalignment = 'center', verticalalignment = 'center')
        ax.text( 2.2, -0.4, r'$\mathbb{P}_{2}$', fontsize = 30, color = 'white', horizontalalignment = 'center', verticalalignment = 'center')
        ax.text(-0.7,  0.3, r'$\mathbb{P}_{3}$', fontsize = 30, color = 'white', horizontalalignment = 'center', verticalalignment = 'center', rotation = 90)
        ax.text( 0.1,  0.3, r'$\mathbb{P}_{4}$', fontsize = 30, color = 'white', horizontalalignment = 'center', verticalalignment = 'center', rotation = 90)
 
        ##############################
        # Add the road arrows        #
        ##############################
        arrow = FancyArrowPatch(
            (1.7, -0.75), # Start point
            (1.7, -0.45), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (1.7, 0.45), # Start point
            (1.7, 0.75), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        #
        ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-2.3, -0.45), # Start point
            (-2.3, -0.75), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (-2.3, 0.75), # Start point
            (-2.3, 0.45), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)          
        #
        ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.45, -1.2), # Start point
            (-1.15, -1.2), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (0.55, -1.2), # Start point
            (0.85, -1.2), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)    
        #      
        ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.45, 0.8), # Start point
            (-1.15, 0.8), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (0.55, 0.8), # Start point
            (0.85, 0.8), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)  
        #
        ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.15, 1.2), # Start point
            (-1.45, 1.2), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (0.85, 1.2), # Start point
            (0.55, 1.2), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow) 
        #
        ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.15, -0.8), # Start point
            (-1.45, -0.8), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (0.85, -0.8), # Start point
            (0.55, -0.8), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)  
        #
        ax.add_patch(arrow)     
        arrow = FancyArrowPatch(
            (-1.9, -0.15), # Start point
            (-1.9,  0.15), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)
        arrow = FancyArrowPatch(
            (1.3,  0.15), # Start point
            (1.3, -0.15), # End point
            arrowstyle = '->', # Arrow style
            mutation_scale = 40, # Size of the arrow
            color = (1.0, 1.0, 1.0, 0.5), # Color of the arrow
            linewidth = 2 # Width of the arrow
        )
        ax.add_patch(arrow)

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
            zorder = 101)
        ax.add_patch(bounding_box)


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
                            zorder = 100
                        )
        ax.add_patch(rect)

        rect = Rectangle((1.9, -0.2),
                            width = 0.6,
                            height = 0.4,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 100
                        )
        ax.add_patch(rect)

        rect = Rectangle((1.9, -1.4),
                            width = 0.6,
                            height = 0.8,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 100
                        )
        ax.add_patch(rect)        
        

        # INTERNAL OBSTACLES

        rect = Rectangle((-1.7, 0.0),
                            width = 0.8,
                            height = 0.6,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 100
                        )
        ax.add_patch(rect)

        rect = Rectangle((-0.5, 0.0),
                            width = 0.4,
                            height = 0.6,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 100
                        )
        ax.add_patch(rect)        

        rect = Rectangle((0.3, 0.0),
                            width = 0.8,
                            height = 0.6,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 100
                        )
        ax.add_patch(rect)

        rect = Rectangle((-1.7, -0.6),
                            width = 2.8,
                            height = 0.6,
                            linewidth = 2,
                            facecolor = 'green',
                            fill = True,
                            hatch = '.....',
                            alpha = 0.8,
                            zorder = 100
                        )
        ax.add_patch(rect)        