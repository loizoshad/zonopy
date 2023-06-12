import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math

from utils.sets.zonotopes import Zonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations
from utils.visualization import ZonoVisualizer

'''
This environment includes disturbances

This environment supports the second approach for computing backward reachable sets
'''



class ParamBRS:
    def __init__(self, dynamics, options = None):
        # Dynamics Model
        self.A = dynamics.A
        self.B = dynamics.B

        if options == 'inner':
            self.x_min = -2.05              # Min x bound
            self.x_max = 1.55               # Max x bound
            self.y_min = -0.95              # Min y bound
            self.y_max = 1.05               # Max y bound
            self.x_step = self.B[0][0]      # Step size for discretization in x axis
            self.y_step = self.B[1][1]      # Step size for discretization in y axis
            self.samples_x = math.ceil( (self.x_max - self.x_min) / (self.x_step) )                       # Number of samples (x)
            self.samples_y = math.ceil( (self.y_max - self.y_min) / (self.y_step) )                       # Number of samples (y)
            self.max_dist_x = math.ceil(self.B[0][0] / self.x_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_y = math.ceil(self.B[1][1] / self.y_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_diag = math.ceil( math.sqrt(self.max_dist_x**2 + self.max_dist_y**2) )    # Maximum distance it can travel in one step (diagonal)
            self.p3_ = np.array([ [0.3,  -0.1],[0.7,  -0.1] ])                                        # Parking spot 2 vertices

            # Create a list of all (x, y) points between the two points in p1_, p2_, p3_, p4_
            self.initial_points = []
            for i in range(int(abs(self.p3_[1][0] - self.p3_[0][0])/self.x_step) + 2):    # These are the horizontal lines
                self.initial_points.append([self.p3_[0][0] + i*self.x_step, self.p3_[0][1]])

            self.initial_points = np.array(self.initial_points)   
            self.already_contained_points = self.initial_points 

            # Discretize the x-y state space
            self.x_space = np.arange(self.x_min, self.x_max, self.x_step)
            self.y_space = np.arange(self.y_min, self.y_max, self.y_step)      

            # Associated flag for already contained points
            self.is_already_contained = np.zeros((self.samples_y, self.samples_x))

            # Update the flag for already contained points
            for p in self.already_contained_points:
                x_idx = np.argmin(np.abs(self.x_space - p[0]))
                y_idx = np.argmin(np.abs(self.y_space - p[1]))
                self.is_already_contained[y_idx, x_idx] = 1    

        if options == 'outer':
            # self.x_min = -2.1; self.x_max = 1.5; self.y_min = -1.0; self.y_max = 1.0                # Bounds
            self.x_min = -2.45; self.x_max = 1.85; self.y_min = -1.35; self.y_max = 1.45                # Bounds
            self.x_step = self.B[0][0]; self.y_step = self.B[1][1]                                  # Step size
            self.samples_x = math.ceil( (self.x_max - self.x_min) / (self.x_step) )                       # Number of samples (x)
            self.samples_y = math.ceil( (self.y_max - self.y_min) / (self.y_step) )                       # Number of samples (y)
            self.max_dist_x = math.ceil(self.B[0][0] / self.x_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_y = math.ceil(self.B[1][1] / self.y_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_diag = math.ceil( math.sqrt(self.max_dist_x**2 + self.max_dist_y**2) )    # Maximum distance it can travel in one step (diagonal)
            self.p1_ = np.array([ [1.9,  0.2],[1.9,  -0.2] ])                                        # Parking spot 2 vertices

            # Create a list of all (x, y) points between the two points in p1_, p2_,
            self.initial_points = []
            for i in range(int(abs(self.p1_[1][1] - self.p1_[0][1])/self.y_step) + 1):    # These are the vertical lines
                self.initial_points.append([self.p1_[0][0], self.p1_[0][1] - i*self.y_step])

            self.initial_points = np.array(self.initial_points)  
            self.already_contained_points = self.initial_points

            # # Discretize the x-y state space
            # self.x_space = np.linspace(x_min, x_max, self.samples_x)
            # self.y_space = np.linspace(y_min, y_max, self.samples_y)
            self.x_space = np.arange(self.x_min, self.x_max, self.x_step)
            self.y_space = np.arange(self.y_min, self.y_max, self.y_step)      

            # Associated flag for already contained points
            self.is_already_contained = np.zeros((self.samples_y, self.samples_x))

            # Update the flag for already contained points
            for p in self.already_contained_points:
                x_idx = np.argmin(np.abs(self.x_space - p[0]))
                y_idx = np.argmin(np.abs(self.y_space - p[1]))
                self.is_already_contained[y_idx, x_idx] = 1                         

        if options == 'full':
            self.x_min = -2.45; self.x_max = 1.85; self.y_min = -1.35; self.y_max = 1.45                # Bounds
            self.x_step = self.B[0][0]; self.y_step = self.B[1][1]                                  # Step size
            self.samples_x = math.ceil( (self.x_max - self.x_min) / (self.x_step) )                       # Number of samples (x)
            self.samples_y = math.ceil( (self.y_max - self.y_min) / (self.y_step) )                       # Number of samples (y)
            self.max_dist_x = math.ceil(self.B[0][0] / self.x_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_y = math.ceil(self.B[1][1] / self.y_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_diag = math.ceil( math.sqrt(self.max_dist_x**2 + self.max_dist_y**2) )    # Maximum distance it can travel in one step (diagonal)
            self.p1_ = np.array([ [1.9,  0.2],[1.9,  -0.2] ])                                        # Parking spot 2 vertices
            self.p3_ = np.array([ [0.3,  -0.1],[0.7,  -0.1] ])                                        # Parking spot 2 vertices

            # Create a list of all (x, y) points between the two points in p1_, p2_,
            self.initial_points = []
            for i in range(int(abs(self.p1_[1][1] - self.p1_[0][1])/self.y_step) + 1):    # These are the vertical lines
                self.initial_points.append([self.p1_[0][0], self.p1_[0][1] - i*self.y_step])
            for i in range(int(abs(self.p3_[1][0] - self.p3_[0][0])/self.x_step) + 2):    # These are the horizontal lines
                self.initial_points.append([self.p3_[0][0] + i*self.x_step, self.p3_[0][1]])

            self.initial_points = np.array(self.initial_points)
            self.already_contained_points = self.initial_points 

            # # Discretize the x-y state space
            # self.x_space = np.linspace(x_min, x_max, self.samples_x)
            # self.y_space = np.linspace(y_min, y_max, self.samples_y)
            self.x_space = np.arange(self.x_min, self.x_max, self.x_step)
            self.y_space = np.arange(self.y_min, self.y_max, self.y_step)      

            # Associated flag for already contained points
            self.is_already_contained = np.zeros((self.samples_y, self.samples_x))

            # Update the flag for already contained points
            for p in self.already_contained_points:
                x_idx = np.argmin(np.abs(self.x_space - p[0]))
                y_idx = np.argmin(np.abs(self.y_space - p[1]))
                self.is_already_contained[y_idx, x_idx] = 1   


class StaticEnv2:
    def __init__(self, zono_op, dynamics, visualizer, options = 'outer') -> None:
        self.zono_op = zono_op      # Class for Zonotope operations
        self.vis = visualizer       # Class for visualizing the results
        
        self.dynamics = dynamics    # Class for dynamics of the system
        self.A = dynamics.A         # System dynamics
        self.B = dynamics.B         # System dynamics
        
        self.state_space = StateSpaceSafe().get_space(options)
        self.input_space = InputSpace().get_space(max_input=dynamics.v_max, min_input=dynamics.v_min)
        self.target_space = TargetSpace().get_space(options)

        self.brs_grid() 

    def brs_grid(self):
        '''
        This method is used to created a discrete grid that can be used to speed up the plotting process of the 
        reachable and invariant sets.
        '''
        self.x_min = -2.45; self.x_max = 1.85; self.y_min = -1.35; self.y_max = 1.45            # Bounds
        self.x_step = self.B[0][0]; self.y_step = self.B[1][1]                                  # Step size
        self.samples_x = math.ceil( (self.x_max - self.x_min) / (self.x_step) )                 # Number of samples (x)
        self.samples_y = math.ceil( (self.y_max - self.y_min) / (self.y_step) )                 # Number of samples (y)
        self.max_dist_x = math.ceil(self.B[0][0] / self.x_step)                                 # Maximum distance it can travel in one step (x)
        self.max_dist_y = math.ceil(self.B[1][1] / self.y_step)                                 # Maximum distance it can travel in one step (x)
        self.max_dist_diag = math.ceil( math.sqrt(self.max_dist_x**2 + self.max_dist_y**2) )    # Maximum distance it can travel in one step (diagonal)
        self.p1 = np.array([ [1.9,  0.2],[1.9,  -0.2] ])                                        # Vertices of the outer parking spot
        self.p2 = np.array([ [0.3,  -0.1],[0.7,  -0.1] ])                                       # Vertices of the inner parking spot


        self.step_size = np.array([self.x_step, self.y_step])                                    # Step size for discretization


        # Create a list of all (x, y) points between the two points in p1, p2
        self.initial_points = []
        for i in range(int(abs(self.p1[1][1] - self.p1[0][1])/self.y_step) + 1):    # These are the vertical lines
            self.initial_points.append([self.p1[0][0], self.p1[0][1] - i*self.y_step])

        for i in range(int(abs(self.p2[1][0] - self.p2[0][0])/self.x_step) + 2):    # These are the horizontal lines
            self.initial_points.append([self.p2[0][0] + i*self.x_step, self.p2[0][1]])

        self.initial_points = np.array(self.initial_points)  

        # Discretize the x-y state space
        self.x_space = np.arange(self.x_min, self.x_max, self.x_step)
        self.y_space = np.arange(self.y_min, self.y_max, self.y_step)      

        # Associated flag for already contained points
        self.grid = np.zeros((self.samples_y, self.samples_x))

        # Update the flag for already contained points
        for p in self.initial_points:
            x_idx = np.argmin(np.abs(self.x_space - p[0]))
            y_idx = np.argmin(np.abs(self.y_space - p[1]))
            self.grid[y_idx, x_idx] = 1 
        


    def vis_background(self):
        # Visualize background
        img = mpimg.imread('./images/park_env.png')
        self.vis.ax.imshow(img, extent=[-2.5, 2.5, -1.4, 1.4], zorder = 1)




class StateSpaceSafe:
    '''
    This class contains the safe state space of the system

    The state space consists of all the places in the environment that are considered to be immediately safe.
    By immediately safe we mean that by being at that specific place, the system is safe from collision.
    However, that does not mean that the system is safe from collision in the future.

    This version does not take into account limitation on changing lines or traffic direction of the lanes.
    '''
    
    def __init__(self):
        self.zono_op = ZonoOperations()

    def get_space(self, options):
        if options == 'outer':
            return self.road_outer
        elif options == 'inner':
            road_inner = self.zono_op.union_hz_hz(self.road_inner, self.road_park_1)
            road_inner = self.zono_op.union_hz_hz(road_inner, self.road_park_2)
            return road_inner
        elif options == 'full':
            return self.road_full
        else:
            raise ValueError('Invalid option for state space, choose from "outer", "inner", "full"')

    @property
    def road_outer(self):
        nx = 2          # Number of state variables
        lw = 0.38        # Width of one road lane [m]
        ll_h = 4.40    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03    # Length of road segments 3 and 4 [m]   (Vertical roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 2; nc = 0; nb = 1
        c_road = np.array([ [-0.3],     # Center of road in x-direction
                            [ 0.0]      # Center of road in y-direction
                        ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_h = np.array([
            [ll_h/2,  0.0],
            [  0.0  , lw/2]
        ])
        Gb_road_h = np.array([
            [0.0],
            [1.21]
        ])
        Gc_road_v = np.array([
            [lw/2,    0.0],
            [ 0.0, ll_v/2]
        ])
        Gb_road_v = np.array([ [2.01],
                                [ 0.0]
                            ])
        
        road_h = HybridZonotope(Gc_road_h, Gb_road_h, c_road, Ac_road, Ab_road, b_road)
        road_v = HybridZonotope(Gc_road_v, Gb_road_v, c_road, Ac_road, Ab_road, b_road)

        road = self.zono_op.union_hz_hz(road_h, road_v)

        return road
    
    @property
    def road_inner(self):
        nx = 2          # Number of state variables
        lw = 0.38        # Width of one road lane [m]
        ll_h = 3.56    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 1.21    # Length of road segments 3 and 4 [m]   (Vertical roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 2; nc = 0; nb = 1
        c_road = np.array([ [-0.3],     # Center of road in x-direction
                            [ 0.0]      # Center of road in y-direction
                        ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_h = np.array([
            [ll_h/2 , 0.0 ],
            [  0.0  , lw/2]
        ])
        Gb_road_h = np.array([
            [  0.0],
            [0.795]
        ])
        Gc_road_v = np.array([
            [lw/2,    0.0],
            [ 0.0, ll_v/2]
        ])
        Gb_road_v = np.array([  [1.59],
                                [0.0]
        ])
        road_h = HybridZonotope(Gc_road_h, Gb_road_h, c_road, Ac_road, Ab_road, b_road)
        road_v = HybridZonotope(Gc_road_v, Gb_road_v, c_road, Ac_road, Ab_road, b_road)

        road = self.zono_op.union_hz_hz(road_h, road_v)

        return road

    @property
    def road_park_1(self):
        nx = 2          # Number of state variables
        w = 0.394        # Width of one road lane [m]
        l = 2.6    # Length of road segments 1 and 2 [m] (Horizontal roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 2; nc = 0; nb = 0
        c = np.array([ [-0.3], [-0.3]])
        Ac = np.zeros((nc, ng))
        b = np.zeros((nc, 1))
        Ab = np.zeros((nc, nb))
        Gc = np.array([
            [  l/2, 0.0],
            [  0.0, w/2]
        ])
        Gb = np.zeros((ng, nb))

        road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)
        
        return road

    @property
    def road_park_2(self):
        nx = 2          # Number of state variables
        w = 0.1        # Width of one road lane [m]
        l = 0.4    # Length of road segments 1 and 2 [m] (Horizontal roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 2; nc = 0; nb = 1
        c = np.array([ [-0.3], [-0.55] ])
        Ac = np.zeros((nc, ng))
        b = np.zeros((nc, 1))
        Ab = np.zeros((nc, nb))
        Gc = np.array([
            [  l/2, 0.0],
            [  0.0, w/2]
        ])
        Gb = np.array([
            [0.8],
            [0.0]
        ])

        road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return road

    @property
    def road_full(self):
        road_full = self.zono_op.union_hz_hz(self.road_outer, self.road_inner)
        road_full = self.zono_op.union_hz_hz(road_full, self.road_park_1)
        road_full = self.zono_op.union_hz_hz(road_full, self.road_park_2)
        
        return road_full

class InputSpace:
    def __init__(self):
        self.zono_op = ZonoOperations()

    def get_space(self, max_input = None, min_input = None):
        assert max_input is not None, 'Maximum input must be specified'
        assert min_input is not None, 'Minimum input must be specified'

        ni = 2          # Number of input variables
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 2; nc = 0; nb = 0

        mean_x = (abs(max_input[0]) + abs(min_input[0]))[0]/2
        mean_y = (abs(max_input[1]) + abs(min_input[1]))[0]/2

        Gc = np.array([
            [mean_x, 0.0],
            [0.0, mean_y]
        ])

        Gb = np.zeros((ng, nb))

        c = np.array([
            [0.0],
            [0.0]
        ])

        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)

class TargetSpace:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()

    def get_space(self, options):
        if options == 'outer':
            return self.park_outer
        elif options == 'inner':
            return self.park_inner
        elif options == 'full':
            return self.park_full
        else:
            raise ValueError('Invalid option for target space. Options are "outer", "inner" and "full"')

    @property
    def park_outer(self):
        pw = 0.4        # Width of parking slot [m]
        pl = 0.6        # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 0; nc = 0
        Gc_park = np.array([
            [pl/2, 0.0],
            [0.0, pw/2]
        ])

        Gb_park = np.zeros((ng, nb))
        c_park = np.array([ [2.2], [0.0] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)

        return parking
    
    @property
    def park_inner(self):
        pw = 0.4        # Width of parking slot [m]
        pl = 0.6        # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 0; nc = 0
        Gc_park = np.array([
            [pw/2, 0.0],
            [0.0, pl/2]
        ])
        Gb_park = np.zeros((ng, nb))
        c_park = np.array([ [0.5], [0.2] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)

        return parking

    @property
    def park_full(self):
        return self.zono_op.union_hz_hz(self.park_outer, self.park_inner)
    


