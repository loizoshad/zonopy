import numpy as np
import matplotlib.image as mpimg
import math

from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations

'''
This environment includes disturbances

This environment is gonna be used for the final testing
'''


class ParamBRS:
    def __init__(self, dynamics):

        self.min = np.array([
            -2.45,              # Minimum value for x-direction
            -1.35               # Minimum value for y-direction
        ])
        self.max = np.array([
            1.85,               # Maximum value for x-direction
            1.45                # Maximum value for y-direction
        ])
        self.step = np.array([
            0.1,
            0.1
        ])   

        # Vertices of outer parking spot
        self.p1 = np.array([[1.9,   0.2],     # x, y
                            [1.9,  -0.2] ])   # x, y
        # Vertices of inner parking spot
        self.p2 = np.array([[0.3,  -0.1],     # x, y
                            [0.7,  -0.1] ])   # x, y

        self.samples_x = math.ceil( (self.max[1] - self.min[1]) / (self.step[1]) )              # Number of samples (x)
        self.samples_y = math.ceil( (self.max[0] - self.min[0]) / (self.step[0]) )              # Number of samples (y)
        self.max_dist_x = 4
        self.max_dist_y = 4
        self.max_dist_diag = 4

        # Create a list of all (x, y) points between the two points in p1, p2
        self.initial_points = []
        p1_points = int(abs(self.p1[1][1] - self.p1[0][1])/self.step[0]) + 1
        p2_points = int(abs(self.p2[1][0] - self.p2[0][0])/self.step[1]) + 2
        for i in range(p1_points):    # These are the vertical lines (y-direction)
            self.initial_points.append([self.p1[0][0],                      # x
                                        self.p1[0][1] - i*self.step[0]      # y
                                        ])
        for i in range(p2_points):    # These are the horizontal lines (x-direction)
            self.initial_points.append([self.p2[0][0] + i*self.step[1],     # x
                                        self.p2[0][1]                       # y
                                        ])


        axis_ranges = [(min, max, step) for min, max, step in zip(self.min, self.max, self.step)]
        meshgrid = np.meshgrid(*[np.arange(start, stop, step) for start, stop, step in axis_ranges])
        self.space = np.stack(meshgrid, axis=-1)    # Stack the grids into a single n-dimensional space


        # JUST FOR THE X, Y DIMENSIONS
        self.x_space = np.arange(self.min[0], self.max[0], self.step[0])
        self.y_space = np.arange(self.min[1], self.max[1], self.step[1])


        # Associated flag for already contained points
        self.is_already_contained = np.zeros(self.space.shape)

        # JUST FOR THE X, Y DIMENSIONS
        self.is_already_contained_xy = np.zeros((self.y_space.shape[0], self.x_space.shape[0]))


        # Update the flag for already contained points
        for p in self.initial_points:
            x_idx = math.floor( (p[0] - self.min[0]) / self.step[0] )
            y_idx = math.floor( (p[1] - self.min[1]) / self.step[1] )
            self.is_already_contained[y_idx, x_idx, :, :] = 1   

            # JUST FOR THE X, Y DIMENSIONS
            self.is_already_contained_xy[y_idx, x_idx] = 1

class ParamFRS:
    def __init__(self, dynamics):

        self.min = np.array([
            -2.45,              # Minimum value for x-direction
            -1.35              # Minimum value for y-direction
        ])
        self.max = np.array([
            1.85,               # Maximum value for x-direction
            1.45               # Maximum value for y-direction
        ])
        self.step = np.array([
            0.1,
            0.1
        ])

        self.samples_x = math.ceil( (self.max[1] - self.min[1]) / (self.step[1]) )              # Number of samples (x)
        self.samples_y = math.ceil( (self.max[0] - self.min[0]) / (self.step[0]) )              # Number of samples (y)                                    # Parking spot 2 vertices

        axis_ranges = [(min, max, step) for min, max, step in zip(self.min, self.max, self.step)]
        meshgrid = np.meshgrid(*[np.arange(start, stop, step) for start, stop, step in axis_ranges])
        self.space = np.stack(meshgrid, axis=-1)    # Stack the grids into a single n-dimensional space


        # JUST FOR THE X, Y DIMENSIONS
        self.x_space = np.arange(self.min[0], self.max[0], self.step[0])
        self.y_space = np.arange(self.min[1], self.max[1], self.step[1])


        # Associated flag for already contained points
        self.is_already_contained = np.zeros(self.space.shape)

        # JUST FOR THE X, Y DIMENSIONS
        self.is_already_contained_xy = np.zeros((self.y_space.shape[0], self.x_space.shape[0]))

class DynamicEnv:
    def __init__(self, zono_op, dynamics, visualizer, options) -> None:

        assert options in ['outer', 'inner', 'full'], 'Invalid option for state space, choose from "outer", "inner", "full"'

        self.zono_op = zono_op      # Class for Zonotope operations
        self.vis = visualizer       # Class for visualizing the results        
        self.dynamics = dynamics    # Class for dynamics of the system
        
        # self.brs_settings = ParamBRS(dynamics)
        self.brs_settings = ParamFRS(dynamics)  # TODO: Automate this choice


        self.A = dynamics.A         # System dynamics
        self.B = dynamics.B         # System dynamics
        self.W = dynamics.W         # System dynamics
        

        self.state_space = StateSpaceSafe().get_space(options)
        self.input_space = InputSpace().get_space(max_input=dynamics.v_max, min_input=dynamics.v_min)
        self.initial_space = DynamicObstacleSpace().get_space()

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
        self.vx_max = 1.0    # TODO: Define based on dynamics model
        self.vy_max = 1.0    # TODO: Define based on dynamics model


    def get_space(self, options):
        return self.road_outer
        

    @property
    def road_outer(self):
        #####
        road_d = self.road_down
        road_u = self.road_up
        road_l = self.road_left
        road_r = self.road_right

        road = self.zono_op.union_hz_hz_v2(road_d, road_r)

        return road

    @property
    def road_down(self):
        nx = 2          # Number of state variables
        lw = 0.38       # Width of one road lane [m]
        ll_h = 4.40     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03     # Length of road segments 3 and 4 [m] (Vertical roads)
        ng = 2; nc = 0; nb = 0

        # c_d = np.array([    [-0.3],     # Center of road in x-direction (Position)
        #                     [-1.21]     # Center of road in y-direction (Position)
        #                 ])
        Ac_d = np.zeros((nc, ng))
        b_d = np.zeros((nc, 1))
        Ab_d = np.zeros((nc, nb))
        # Gc_d = np.array([
        #     [ll_h/2, 0.0 ],
        #     [0.0   , lw/2]
        # ])
        eps = 0.1
        c_d = np.array([    [ 0.00],     # Center of road in x-direction (Position)
                            [-0.25 - eps]     # Center of road in y-direction (Position)
                        ])        
        Gc_d = np.array([
            [1.0, 0.0 ],
            [0.0, 0.25]
        ])
        Gb_d = np.zeros((ng, nb))

        return HybridZonotope(Gc_d, Gb_d, c_d, Ac_d, Ab_d, b_d)

    @property
    def road_up(self):
        nx = 2          # Number of state variables
        lw = 0.38       # Width of one road lane [m]
        ll_h = 4.40     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03     # Length of road segments 3 and 4 [m] (Vertical roads)

        ng = 2; nc = 0; nb = 0
        c_u = np.array([    [-0.3],     # Center of road in x-direction (Position)
                            [1.21],     # Center of road in y-direction (Position)
                        ])
        Ac_u = np.zeros((nc, ng))
        b_u = np.zeros((nc, 1))
        Ab_u = np.zeros((nc, nb))

        Gc_u = np.array([
            [ll_h/2, 0.0 ],
            [0.0   , lw/2]
        ])
        Gb_u = np.zeros((ng, nb))

        return HybridZonotope(Gc_u, Gb_u, c_u, Ac_u, Ab_u, b_u)

    @property
    def road_right(self):
        nx = 2          # Number of state variables
        lw = 0.38       # Width of one road lane [m]
        ll_h = 4.40     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03     # Length of road segments 3 and 4 [m] (Vertical roads)

        ng = 2; nc = 0; nb = 0
        # c_r = np.array([ [-0.3 + 2.01],     # Center of road in x-direction (Position)
        #                     [ 0.0],     # Center of road in y-direction (Position)
        #                 ])
        Ac_r = np.zeros((nc, ng))
        b_r = np.zeros((nc, 1))
        Ab_r = np.zeros((nc, nb))
        # Gc_r = np.array([
        #     [lw/2, 0.0   ],
        #     [0.0 , ll_v/2]
        # ])
        eps = 0.1
        c_r = np.array([[0.00],     # Center of road in x-direction (Position)
                        [0.25 + eps],     # Center of road in y-direction (Position)
        ])
        Gc_r = np.array([
            [1.0, 0.00],
            [0.0, 0.25]
        ])

        Gb_r = np.zeros((nx, 0))
        
        return HybridZonotope(Gc_r, Gb_r, c_r, Ac_r, Ab_r, b_r)

    @property
    def road_left(self):
        nx = 2          # Number of state variables
        lw = 0.38       # Width of one road lane [m]
        ll_h = 4.40     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03     # Length of road segments 3 and 4 [m] (Vertical roads)

        ng = 2; nc = 0; nb = 0
        c_r = np.array([ [-0.3 - 2.01],     # Center of road in x-direction (Position)
                            [ 0.0]     # Center of road in y-direction (Position)
                        ])
        Ac_r = np.zeros((nc, ng))
        b_r = np.zeros((nc, 1))
        Ab_r = np.zeros((nc, nb))

        Gc_r = np.array([
            [lw/2, 0.0   ],
            [0.0 , ll_v/2]
        ])        
        Gb_r = np.zeros((nx, 0))
        
        return HybridZonotope(Gc_r, Gb_r, c_r, Ac_r, Ab_r, b_r)











class InputSpace:
    def __init__(self):
        self.zono_op = ZonoOperations()
        
        # Maximum rate of change in velocity (acceleration)
        self.ax_max = 1.0       # TODO: Define based on dynamics model
        self.ay_max = 1.0       # TODO: Define based on dynamics model

    def get_space(self, max_input = None, min_input = None):
        assert max_input is not None, 'Maximum input must be specified'
        assert min_input is not None, 'Minimum input must be specified'

        ni = 2          # Number of input variables
        ng = 2; nc = 0; nb = 0

        Gc = np.array([
            [self.ax_max/2, 0.0],
            [0.0, self.ay_max/2]
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




    
class DynamicObstacleSpace:
    '''
    This class contains all the possible initial states for the non-ego vehicles
    '''

    def get_space(self):
        car_d = self.car_outer_d
        car_u = self.car_outer_u
        car_r = self.car_outer_r
        car_l = self.car_outer_l

        car = car_d

        return car

    @property
    def car_outer_d(self):
        # w = 0.38        # Width of vehicle [m]
        w = 0.20        # Width of vehicle [m]
        l = 0.60        # Length of vehicle [m]
        nx = 2          # Number of state variables
        ng = 2; nb = 0; nc = 0

        # Car starts at the bottom of the road with a slightly positive velocity in the x-direction
        Gc = np.array([
            [l/2, 0.0],
            [0.0, w/2],
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [-0.3],
                        [-1.2]
                        ])
        
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))        

        outer_road_car = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return outer_road_car
    
    @property
    def car_outer_u(self):

        # w = 0.38        # Width of vehicle [m]
        w = 0.20        # Width of vehicle [m]
        l = 0.60        # Length of vehicle [m]
        nx = 2          # Number of state variables
        ng = 2; nb = 0; nc = 0

        # Car starts at the bottom of the road with a slightly positive velocity in the x-direction
        Gc = np.array([
            [l/2, 0.0],
            [0.0, w/2]
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [-0.3],
                        [1.2]
                        ])
        
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))        

        outer_road_car = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return outer_road_car
    
    @property
    def car_outer_r(self):
        # w = 0.38        # Width of vehicle [m]
        w = 0.20        # Width of vehicle [m]
        l = 0.60        # Length of vehicle [m]
        nx = 2          # Number of state variables
        ng = 2; nb = 0; nc = 0

        # Car starts at the bottom of the road with a slightly positive velocity in the x-direction
        eps = 1e-2
        Gc = np.array([
            [w/2, 0.0],
            [0.0, l/2]
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [-0.3 + 2.01],
                        [0.0]
                        ])
        
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))        

        outer_road_car = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return outer_road_car
    
    @property
    def car_outer_l(self):
        # w = 0.38        # Width of vehicle [m]
        w = 0.20        # Width of vehicle [m]
        l = 0.60        # Length of vehicle [m]
        nx = 2         # Number of state variables
        ng = 2; nb = 0; nc = 0

        # Car starts at the bottom of the road with a slightly positive velocity in the x-direction
        Gc = np.array([
            [w/2, 0.0],
            [0.0, l/2]
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [-0.3 - 2.01],
                        [0.0]
                        ])
        
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))        

        outer_road_car = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return outer_road_car