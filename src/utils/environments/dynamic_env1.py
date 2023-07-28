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
    def __init__(self, dynamics):

        self.min = np.array([
            -2.45,              # Minimum value for x-direction
            -1.35,              # Minimum value for y-direction
            dynamics.v_min[0],  # Minimum value for velocity in x-direction
            dynamics.v_min[1]   # Minimum value for velocity in y-direction
        ])
        self.max = np.array([
            1.85,               # Maximum value for x-direction
            1.45,               # Maximum value for y-direction
            dynamics.v_max[0],  # Maximum value for velocity in x-direction
            dynamics.v_max[1]   # Maximum value for velocity in y-direction
        ])
        # self.step = np.array([
        #     dynamics.v_max[0]*dynamics.dt,   # Step size for position in x
        #     dynamics.v_max[1]*dynamics.dt,   # Step size for position in y
        #     dynamics.v_max[0]*dynamics.dt,   # Step size for velocity in x
        #     dynamics.v_max[1]*dynamics.dt    # Step size for velocity in y          
        # ])
        self.step = np.array([
            0.1,
            0.1,
            0.1,
            0.1
        ])   

        # Vertices of outer parking spot
        self.p1 = np.array([[1.9,   0.2, 0.0, 0.0],     # x, y, vx, vy
                            [1.9,  -0.2, 0.0, 0.0] ])   # x, y, vx, vy
        # Vertices of inner parking spot
        self.p2 = np.array([[0.3,  -0.1, 0.0, 0.0],     # x, y, vx, vy
                            [0.7,  -0.1, 0.0, 0.0] ])   # x, y, vx, vy

        self.samples_x = math.ceil( (self.max[1] - self.min[1]) / (self.step[1]) )              # Number of samples (x)
        self.samples_y = math.ceil( (self.max[0] - self.min[0]) / (self.step[0]) )              # Number of samples (y)
        # self.max_dist_x = math.ceil(dynamics.B[1][1] / self.step[1])                            # Maximum distance it can travel in one step (x)
        # self.max_dist_y = math.ceil(dynamics.B[0][0] / self.step[0])                            # Maximum distance it can travel in one step (y)
        # self.max_dist_diag = math.ceil( math.sqrt(self.max_dist_x**2 + self.max_dist_y**2) )    # Maximum distance it can travel in one step (diagonal)                                     # Parking spot 2 vertices
        self.max_dist_x = 4
        self.max_dist_y = 4
        self.max_dist_diag = 4

        # Create a list of all (x, y) points between the two points in p1, p2
        self.initial_points = []
        p1_points = int(abs(self.p1[1][1] - self.p1[0][1])/self.step[0]) + 1
        p2_points = int(abs(self.p2[1][0] - self.p2[0][0])/self.step[1]) + 2
        for i in range(p1_points):    # These are the vertical lines (y-direction)
            self.initial_points.append([self.p1[0][0],                      # x
                                        self.p1[0][1] - i*self.step[0],     # y
                                        0.0,                                # vx
                                        0.0])                               # vy
        for i in range(p2_points):    # These are the horizontal lines (x-direction)
            self.initial_points.append([self.p2[0][0] + i*self.step[1],     # x
                                        self.p2[0][1],                      # y
                                        0.0,                                # vx
                                        0.0])                               # vy


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



class DynamicEnv1:
    def __init__(self, zono_op, dynamics, visualizer, options) -> None:

        assert options in ['outer', 'inner', 'full'], 'Invalid option for state space, choose from "outer", "inner", "full"'

        self.zono_op = zono_op      # Class for Zonotope operations
        self.vis = visualizer       # Class for visualizing the results        
        self.dynamics = dynamics    # Class for dynamics of the system
        self.brs_settings = ParamBRS(dynamics)

        self.A = dynamics.A         # System dynamics
        self.B = dynamics.B         # System dynamics
        self.W = dynamics.W         # System dynamics
        

        self.state_space = StateSpaceSafe().get_space(options)
        self.input_space = InputSpace().get_space(max_input=dynamics.v_max, min_input=dynamics.v_min)
        self.target_space = TargetSpace().get_space(options)
        self.initial_space = InitialSpace().get_space()

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
        if options == 'outer':
            road_outer = self.zono_op.redundant_c_gc_hz_v2(self.road_outer)
            return road_outer
            # return self.road_outer
        elif options == 'inner':
            road_inner_a = self.zono_op.union_hz_hz_v2(self.road_park_1, self.road_park_2)
            road_inner_a = self.zono_op.redundant_c_gc_hz_v2(road_inner_a)
            road_inner_b = self.zono_op.redundant_c_gc_hz_v2(self.road_inner)
            road_inner = self.zono_op.union_hz_hz_v2(road_inner_a, road_inner_b)
            return road_inner
        elif options == 'full':
            return self.road_full
        else:
            raise ValueError('Invalid option for state space, choose from "outer", "inner", "full"')

    @property
    def road_outer(self):
        nx = 4          # Number of state variables
        lw = 0.38        # Width of one road lane [m]
        ll_h = 4.40    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03    # Length of road segments 3 and 4 [m]   (Vertical roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [-0.3],     # Center of road in x-direction
                            [ 0.0],     # Center of road in y-direction
                            [ 0.0],     # Velocity in x
                            [ 0.0]      # Velocity in y
                        ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_h = np.array([
            [ll_h/2,  0.0, 0.0, 0.0],
            [0.0   , lw/2, 0.0, 0.0],
            [0.0   , 0.0 , self.vx_max/2, 0.0],
            [0.0   , 0.0 , 0.0, self.vy_max/2]
        ])
        Gb_road_h = np.array([
            [0.0],
            [1.21],
            [-0.45],    # TODO: How did you determine this value?
            [0.0]
        ])
        Gc_road_v = np.array([
            [lw/2, 0.0   , 0.0, 0.0],
            [0.0 , ll_v/2, 0.0, 0.0],
            [0.0 , 0.0   , self.vx_max/2, 0.0],
            [0.0 , 0.0   , 0.0, self.vy_max/2]
        ])        
        Gb_road_v = np.array([  [2.01],
                                [ 0.0],
                                [ 0.0],
                                [ 0.5]  # TODO: How did you determine this value
                            ])
        
        road_h = HybridZonotope(Gc_road_h, Gb_road_h, c_road, Ac_road, Ab_road, b_road)
        road_v = HybridZonotope(Gc_road_v, Gb_road_v, c_road, Ac_road, Ab_road, b_road)

        road = self.zono_op.union_hz_hz_v2(road_h, road_v)

        return road
    
    @property
    def road_inner(self):
        nx = 4          # Number of state variables
        lw = 0.38        # Width of one road lane [m]
        ll_h = 3.56    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 1.21    # Length of road segments 3 and 4 [m]   (Vertical roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [-0.3],     # Center of road in x-direction
                            [ 0.0],     # Center of road in y-direction
                            [ 0.0],     # Velocity in x
                            [ 0.0]      # Velocity in y
                        ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_h = np.array([
            [ll_h/2, 0.0 , 0.0, 0.0],
            [0.0   , lw/2, 0.0, 0.0],
            [0.0   , 0.0 , self.vx_max/2, 0.0],
            [0.0   , 0.0 , 0.0, self.vy_max/2]
        ])
        Gb_road_h = np.array([
            [  0.0],
            [0.795],
            [ 0.45],    # TODO: How did you determine this value?
            [  0.0]
        ])
        Gc_road_v = np.array([
            [lw/2, 0.0   , 0.0, 0.0],
            [0.0 , ll_v/2, 0.0, 0.0],
            [0.0 , 0.0   , self.vx_max/2, 0.0],
            [0.0 , 0.0   , 0.0, self.vy_max/2]
        ])
        Gb_road_v = np.array([  [1.59],
                                [ 0.0],
                                [ 0.0],
                                [-0.5]      # TODO: How did you determine this value?
        ])
        road_h = HybridZonotope(Gc_road_h, Gb_road_h, c_road, Ac_road, Ab_road, b_road)
        road_v = HybridZonotope(Gc_road_v, Gb_road_v, c_road, Ac_road, Ab_road, b_road)

        # road = self.zono_op.union_hz_hz(road_h, road_v)
        road = self.zono_op.union_hz_hz_v2(road_h, road_v)

        return road

    @property
    def road_park_1(self):
        nx = 4          # Number of state variables
        w = 0.394        # Width of one road lane [m]
        l = 2.6    # Length of road segments 1 and 2 [m] (Horizontal roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 4; nc = 0; nb = 0
        c = np.array([ [-0.3], [-0.3], [0.0], [0.25]])
        Ac = np.zeros((nc, ng))
        b = np.zeros((nc, 1))
        Ab = np.zeros((nc, nb))
        Gc = np.array([
            [l/2, 0.0, 0.0, 0.0],
            [0.0, w/2, 0.0, 0.0],
            [0.0, 0.0, 0.5, 0.0],
            [0.0, 0.0, 0.0, 0.25]
        ])
        Gb = np.zeros((ng, nb))

        road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)
        
        return road

    @property
    def road_park_2(self):
        nx = 4          # Number of state variables
        w = 0.1        # Width of one road lane [m]
        l = 0.4    # Length of road segments 1 and 2 [m] (Horizontal roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 4; nc = 0; nb = 1
        c = np.array([ [-0.3], [-0.55], [0.0], [0.25] ])
        Ac = np.zeros((nc, ng))
        b = np.zeros((nc, 1))
        Ab = np.zeros((nc, nb))
        Gc = np.array([
            [l/2, 0.0, 0.0, 0.0 ],
            [0.0, w/2, 0.0, 0.0 ],
            [0.0, 0.0, 0.5, 0.0 ],
            [0.0, 0.0, 0.0, 0.25]
        ])
        Gb = np.array([
            [0.8],
            [0.0],
            [0.0],
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
        
        # Maximum rate of change in velocity (acceleration)
        self.ax_max = 5.5    # TODO: Define based on dynamics model
        self.ay_max = 5.5    # TODO: Define based on dynamics model


    def get_space(self, max_input = None, min_input = None):
        assert max_input is not None, 'Maximum input must be specified'
        assert min_input is not None, 'Minimum input must be specified'

        ni = 2          # Number of input variables
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

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
        nx = 4          # Number of state variables
        ng = 2; nb = 0; nc = 0
        Gc_park = np.array([
            [pl/2, 0.0],
            [0.0, pw/2],
            [0.0, 0.0],
            [0.0, 0.0]
        ])

        Gb_park = np.zeros((nx, nb))
        c_park = np.array([ [2.2], [0.0], [0.0], [0.0] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)

        return parking
    
    @property
    def park_inner(self):
        pw = 0.4        # Width of parking slot [m]
        pl = 0.6        # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        nx = 4          # Number of state variables
        ng = 2; nb = 0; nc = 0
        Gc_park = np.array([
            [pw/2, 0.0],
            [0.0, pl/2],
            [0.0, 0.0],
            [0.0, 0.0]
        ])
        Gb_park = np.zeros((nx, nb))
        c_park = np.array([ [0.5], [0.2], [0.0], [0.0] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)

        return parking

    @property
    def park_full(self):
        return self.zono_op.union_hz_hz(self.park_outer, self.park_inner)
    
class InitialSpace:
    '''
    This class contains all the possible initial states for the non-ego vehicles
    '''

    def get_space(self):
        return self.initial_space_1
        # return self.initial_space_2

    @property
    def initial_space_1(self):
        w = 0.25        # Width of vehicle [m]
        l = 0.45        # Length of vehicle [m]
        nx = 4         # Number of state variables
        ng = 2; nb = 0; nc = 0
        Gc = np.array([
            [w/2, 0.0],
            [0.0, l/2],
            [0.0, 0.0],
            [0.0, 0.0]
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [1.29], 
                        [0.375], 
                        [0.0],
                        [0.0] ])
        
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))        

        # initial_space = HybridZonotope(Gc[:2,:], Gb[:2,:], c[:2,:], Ac, Ab, b)
        initial_space = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return initial_space


    @property
    def initial_space_2(self):
        w = 0.25        # Width of vehicle [m]
        l = 0.45        # Length of vehicle [m]
        nx = 4         # Number of state variables
        ng = 2; nb = 0; nc = 0
        Gc = np.array([
            [w/2, 0.0],
            [0.0, l/2],
            [0.0, 0.0],
            [0.0, 0.0]
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [1.7], 
                        [-1.0], 
                        [0.0],
                        [0.0] ])
        
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))        

        initial_space = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return initial_space