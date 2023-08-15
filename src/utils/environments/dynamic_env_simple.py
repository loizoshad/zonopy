import numpy as np
import matplotlib.image as mpimg
import math

from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations

from utils.ego_model_4d import DynamicsModel            # Dynamics model for vehicles


'''
This environment makes use of the 'park_env.png'.

In this environment there are non-ego vehicles acting as obstacles

Specifically, there will be one non-ego vehicle moving in the outer lane,
while the ego-vehicle starts in the inner lane and tries to park on the
outer parking spot.


TODO: 
    - ParamBRS
    - ParamFRS

'''


class ParamBRS:
    def __init__(self, dynamics):

        self.min = np.array([
            -0.475,              # Minimum value for x-direction
            -1.375               # Minimum value for y-direction
        ])
        self.max = np.array([
            1.95,               # Maximum value for x-direction
            0.975                # Maximum value for y-direction
        ])
        self.step = np.array([
            0.05,
            0.05
        ])   

        # Vertices of outer parking spot
        self.p1 = np.array([[1.925,  0.175],     # x, y
                            [1.925, -0.175] ])   # x, y
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

        # JUST FOR THE X, Y DIMENSIONS
        self.is_already_contained_xy = np.zeros((self.y_space.shape[0], self.x_space.shape[0]))

        # Update the flag for already contained points
        for p in self.initial_points:
            x_idx = math.floor( (p[0] - self.min[0]) / self.step[0] )
            y_idx = math.floor( (p[1] - self.min[1]) / self.step[1] )
            # JUST FOR THE X, Y DIMENSIONS
            self.is_already_contained_xy[y_idx, x_idx] = 1


class ParamBRSV2:
    def __init__(self):

        step_size = 0.05

        # space_grid 
        self.space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': []         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
        }

        # Step 1: Fill the 'points' and 'flags' lists for the different parts of the road


        # Vertical Road points (outer)
        for x in np.arange(1.55 + step_size/2, 1.9 - step_size/2, step_size):
            for y in np.arange(-1.05 + step_size/2, 0.85 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)
        
        # Vertical Road points (inner)
        for x in np.arange(1.10 + step_size/2, 1.45 - step_size/2, step_size):
            for y in np.arange(-0.95 + step_size/2, 0.95 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)

        # Passage
        for x in np.arange(1.45 + step_size/2, 1.55 - step_size/2, step_size):
            for y in np.arange(-0.20 + step_size/2, 0.20 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)

        # Horizontal Road points (outer)
        for x in np.arange(-1.20 + step_size/2, 1.9 - step_size/2, step_size):
            for y in np.arange(-1.40 + step_size/2, -1.05 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)
        
        # Horizontal Road points (inner)
        for x in np.arange(-2.0 + step_size/2, 1.10 - step_size/2, step_size):
            for y in np.arange(0.60 + step_size/2, 0.95 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)

        # Vertices of outer parking spot
        self.p1 = np.array([[1.925,  0.175],     # x, y
                            [1.925, -0.175] ])   # x, y
        # Vertices of inner parking spot
        self.p2 = np.array([[0.3,  -0.1],     # x, y
                            [0.7,  -0.1] ])   # x, y

        # Find all the points between the two coordinates in p1 that map to a point in space['points'] and and set their space['flags'] to 1
        for i in range(len(self.space['points'])):
            if self.space['points'][i][0] == self.p1[0][0] and self.space['points'][i][1] <= self.p1[0][1] and self.space['points'][i][1] >= self.p1[1][1]:
                self.space['flags'][i] = 1

        # Find all the points between the two coordinates in p2 that map to a point in space['points'] and and set their space['flags'] to 1
        for i in range(len(self.space['points'])):
            if self.space['points'][i][1] == self.p2[0][1] and self.space['points'][i][0] >= self.p2[0][0] and self.space['points'][i][0] <= self.p2[1][0]:
                self.space['flags'][i] = 1

                







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

class Ego:
    def __init__(self, zono_op, visualizer) -> None:

        self.zono_op = zono_op      # Class for Zonotope operations
        self.vis = visualizer       # Class for visualizing the results        
        self.dynamics = DynamicsModel()    # Class for dynamics of the system
        
        # self.brs_settings = ParamBRS(self.dynamics)
        # self.brs_settings = ParamFRS(self.dynamics)  # TODO: Automate this choice
        self.brs_settings = ParamBRSV2()

        self.A = self.dynamics.A         # System dynamics
        self.B = self.dynamics.B         # System dynamics
        self.W = self.dynamics.W         # System dynamics        

        self.step_size = 0.05
        self.vx_max = 1.0
        self.vy_max = 1.0


    def vis_background(self):
        # Visualize background
        img = mpimg.imread('./images/park_env.png')
        self.vis.ax.imshow(img, extent=[-2.5, 2.5, -1.4, 1.4], zorder = 1)

    
    def state_space(self, options = 'outer'):
        if options == 'full':
            return self.road_full
        elif options == 'inner':
            return self.road_inner
        elif options == 'outer':
            return self.road_outer
        elif options == 'brs_outer':
            return self.road_brs_outer

    def parking(self, options = 'outer'):
        if options == 'full':
            return self.parking_full
        elif options == 'inner':
            return self.parking_inner
        elif options == 'outer':
            return self.parking_outer

    def input(self):
        # Maximum rate of change in velocity (acceleration)
        self.ax_max = 5.5    # TODO: Define based on dynamics model
        self.ay_max = 5.5    # TODO: Define based on dynamics model
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


    @property
    def road_outer(self):
        '''
        States:
        1 : Position in x-direction
        2 : Position in y-direction
        3 : Velocity in x-direction
        4 : Velocity in y-direction
        '''

        nx = 4          # Number of state variables
        lw = 0.38       # Width of one road lane [m]
        ll_h = 4.40     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03     # Length of road segments 3 and 4 [m] (Vertical roads)

        self.vx_max = self.dynamics.self.vx_max
        self.vy_max = self.dynamics.self.vy_max

        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [-0.3], [ 0.0], [ 0.0], [ 0.0] ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))

        # Horizontal Road Section
        Gc_road_h = np.diag(np.array([ ll_h/2, lw/2  , self.vx_max/2, self.vy_max/2 ]))
        Gb_road_h = np.array([ [0.00], [1.21], [-0.5], [0.0] ])
        road_h = HybridZonotope(Gc_road_h, Gb_road_h, c_road, Ac_road, Ab_road, b_road)

        # Vertical Road Section
        Gc_road_v = np.diag(np.array([ lw/2  , ll_v/2, self.vx_max/2, self.vy_max/2 ]))
        Gb_road_v = np.array([ [2.01], [0.00], [ 0.0], [0.5] ])
        road_v = HybridZonotope(Gc_road_v, Gb_road_v, c_road, Ac_road, Ab_road, b_road)
        
        return self.zono_op.union_hz_hz_v2(road_h, road_v)

    @property
    def road_inner(self):
        '''
        States:
        1 : Position in x-direction
        2 : Position in y-direction
        3 : Velocity in x-direction
        4 : Velocity in y-direction
        '''
        self.vx_max = self.dynamics.self.vx_max
        self.vy_max = self.dynamics.self.vy_max        


        ## Road sections outside the parking space
        lw = 0.38       # Width of one road lane [m]
        ll_h = 3.56     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 1.21     # Length of road segments 3 and 4 [m] (Vertical roads)
        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [-0.3], [ 0.0], [ 0.0], [ 0.0] ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))

        # Horizontal Road Section Outside The Parking Space
        Gc_road_h = np.diag(np.array([ ll_h/2, lw/2  , self.vx_max/2, self.vy_max/2 ]))
        Gb_road_h = np.array([ [0.0], [0.795], [0.5], [0.0] ])
        road_h = HybridZonotope(Gc_road_h, Gb_road_h, c_road, Ac_road, Ab_road, b_road)

        # Horizontal Road Section Outside The Parking Space
        Gc_road_v = np.diag(np.array([ lw/2  , ll_v/2, self.vx_max/2, self.vy_max/2 ]))
        Gb_road_v = np.array([ [1.59], [0.0], [ 0.0], [-0.5] ])
        road_v = HybridZonotope(Gc_road_v, Gb_road_v, c_road, Ac_road, Ab_road, b_road)
        
        # Reduce Redundancy
        road_1 = self.zono_op.union_hz_hz_v2(road_h, road_v)        
        road_1 = self.zono_op.redundant_c_gc_hz_v2(road_1)
        road_1 = self.zono_op.redundant_c_gc_hz_v1(road_1, options = 'slow')


        ## Road sections inside the parking space
        w = 0.1     # Width of one road lane [m]
        l = 0.4     # Length of road segments 1 and 2 [m] (Horizontal roads)

        ng = 4; nc = 0; nb = 1
        c = np.array([ [-0.3], [-0.55], [0.0], [0.25] ])
        Ac = np.zeros((nc, ng))
        b = np.zeros((nc, 1))
        Ab = np.zeros((nc, nb))

        Gc = np.diag(np.array([ l/2, w/2  , 0.5, 0.25 ]))
        Gb = np.array([ [0.8], [0.0], [0.0], [0.0] ])
        road_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)        


        #
        w = 0.394  # Width of one road lane [m]
        l = 2.6    # Length of road segments 1 and 2 [m] (Horizontal roads)

        ng = 4; nc = 0; nb = 0
        c = np.array([ [-0.3], [-0.3], [0.0], [0.25]])
        Ac = np.zeros((nc, ng))
        b = np.zeros((nc, 1))
        Ab = np.zeros((nc, nb))
        Gc = np.diag(np.array([ l/2, w/2  , 0.5, 0.25 ]))
        Gb = np.zeros((ng, nb))
        road_3 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road = self.zono_op.union_hz_hz_v2(road_2, road_3)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road, options = 'slow')

        road = self.zono_op.union_hz_hz_v2(road, road_1)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road, options = 'slow')

        return road

    @property
    def road_full(self):
        road = self.zono_op.union_hz_hz_v2(self.road_outer, self.road_inner)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road, options = 'slow')

        return road

    @property
    def road_brs_outer(self):
        '''
        This space contains:
            - Right vertical road of outer lane
            - Down horizontal road of outer lane
            
            - Right vertical road of inner lane
            - Up horizontal road of inner lane

            - Obstacle between the two Right vertical roads to prevent the ego-vehicle from changing lane, except from one small section
            
        This space can be used instead of the full road space when computing the backward reachable set from the outer parking spot,
        and the ego-vehicle is on the up horizontal road of inner lane. This is done to reduce the complexity of the state space, since
        the rest of the roads are redundant in this test case.
        '''

        # Horizontal and vertical road sections
        road = self.zono_op.union_hz_hz_v2(self.road_h, self.road_v)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road, options = 'slow')

        # Passage
        road = self.zono_op.union_hz_hz_v2(road, self.passage)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road, options = 'slow')
        
        print(f'ng = {road.ng}, nc = {road.nc}, nb = {road.nb}')

        return road


    @property
    def road_h(self):
        lw = 7 * self.step_size
        ll_h = 62 * self.step_size     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ng = 4; nc = 0

        # Horizontal Road Section
        c_road_h = np.array([ [-0.3 + 5*self.step_size], [-0.225], [ self.vx_max/2], [ 0.0] ])
        Gc_road_h = np.diag(np.array([ ll_h/2  , lw/2, self.vx_max/2, self.vy_max ]))
        Gb_road_h = np.array([ [8 * self.step_size], [-1.0], [0.0], [0.0] ])
        Ac_road = np.zeros((nc, ng))
        Ab_road_h = np.zeros((nc, 1))
        b_road = np.zeros((nc, 1))
        road_h = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)

        return road_h        


    @property
    def road_v(self):
        lw = 7 * self.step_size
        ll_v = 40 * self.step_size
        ng = 4; nc = 0

        # Vertical Road Section
        c_road_v = np.array([ [1.5], [-self.step_size], [ 0.0], [ 0.0] ])
        Gc_road_v = np.diag(np.array([ lw/2  , (ll_v - 2*self.step_size)/2, self.vx_max, self.vy_max/2 ]))
        Gb_road_v = np.array([ [0.225], [-self.step_size], [0.0], [self.vy_max/2] ])
        Ac_road = np.zeros((nc, ng))
        Ab_road_v = np.zeros((nc, 1))
        b_road = np.zeros((nc, 1))
        road_v = HybridZonotope(Gc_road_v, Gb_road_v, c_road_v, Ac_road, Ab_road_v, b_road)

        return road_v

    @property
    def passage(self):
        nx = 4; ng = 4; nc = 0; nb = 0
        # Passage
        c_pass  = np.array([ [1.5], [0.0], [0.0], [self.vy_max/2]])
        Gc_pass = np.diag(np.array([ 2*self.step_size /2  , (8*self.step_size)/2, self.vx_max/2, self.vy_max/2 ]))
        Gb_pass = np.zeros((nx, nb))
        Ac_pass = np.zeros((nc, ng))
        Ab_pass = np.zeros((nc, nb))
        b_pass = np.zeros((nc, 1))
        passage = HybridZonotope(Gc_pass, Gb_pass, c_pass, Ac_pass, Ab_pass, b_pass)

        return passage













    @property
    def parking_outer(self):
        pw = 0.4        # Width of parking slot [m]
        pl = 0.6        # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 0; nc = 0
        Gc_park = np.array([
            [pl/2, 0.0],
            [0.0, pw/2],
        ])

        Gb_park = np.zeros((ng, nb))
        c_park = np.array([ [2.2], [0.0] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)

        return parking






















class StateSpaceEgo:
    '''
    This class contains the safe state space of the system

    The state space consists of all the places in the environment that are considered to be immediately safe.
    By immediately safe we mean that by being at that specific place, the system is safe from collision.
    However, that does not mean that the system is safe from collision in the future.

    This version does not take into account limitation on changing lines or traffic direction of the lanes.
    '''
    
    def __init__(self):
        self.zono_op = ZonoOperations()
        self.self.vx_max = 1.0    # TODO: Define based on dynamics model
        self.self.vy_max = 1.0    # TODO: Define based on dynamics model


    def get_space(self):
            road = self.zono_op.union_hz_hz_v2(self.road_v, self.road_h)
            return road
        









    @property
    def road_down(self):
        '''
        Velocity limits:
            - x-direction:  0.4 -> 1.0
            - y-direction: -0.1 -> 0.1
        '''
        nx = 4          # Number of state variables
        lw = 0.38 + 0.02       # Width of one road lane [m]
        ll_h = 4.40     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03     # Length of road segments 3 and 4 [m] (Vertical roads)
        ng = 4; nc = 0; nb = 0
        # c_d = np.array([    [-0.3],     # Center of road in x-direction (Position)
        #                     [-1.21],     # Center of road in y-direction (Position)
        #                     [ 0.5],     # Velocity in x-direction
        #                     [ 0.5]      # Velocity in y-direction
        #                 ])
        Ac_d = np.zeros((nc, ng))
        b_d = np.zeros((nc, 1))
        Ab_d = np.zeros((nc, nb))

        # Gc_d = np.array([
        #     [ll_h/2, 0.0 , 0.0, 0.0],
        #     [0.0   , lw/2, 0.0, 0.0],
        #     [0.0   , 0.0 , self.self.vx_max/2, 0.0],
        #     [0.0   , 0.0 , 0.0, self.self.vy_max/2]
        # ])
        eps = 0.2
        Gc_d = np.array([
            [ll_h/2, 0.0 , 0.00, 0.00],
            [0.0   , lw/2, 0.00, 0.00],
            [0.0   , 0.0 , 0.025, 0.00],
            [0.0   , 0.0 , 0.00, 0.20]
        ])
        c_d = np.array([    [-0.3],     # Center of road in x-direction (Position)
                            [-1.21 + 0.01],     # Center of road in y-direction (Position)
                            [ 0.475],     # Velocity in x-direction
                            [ 0.2]      # Velocity in y-direction
                        ])
        Gb_d = np.zeros((ng, nb))

        print(f'Road down:')
        print(f'x :\t[{(c_d[0, 0] - Gc_d[0, 0]):+0,.2f} -> {(c_d[0, 0] + Gc_d[0, 0]):+0,.2f}]')
        print(f'y :\t[{(c_d[1, 0] - Gc_d[1, 1]):+0,.2f} -> {(c_d[1, 0] + Gc_d[1, 1]):+0,.2f}]')
        print(f'vx:\t[{(c_d[2, 0] - Gc_d[2, 2]):+0,.2f} -> {(c_d[2, 0] + Gc_d[2, 2]):+0,.2f}]')
        print(f'vy:\t[{(c_d[3, 0] - Gc_d[3, 3]):+0,.2f} -> {(c_d[3, 0] + Gc_d[3, 3]):+0,.2f}]')

        return HybridZonotope(Gc_d, Gb_d, c_d, Ac_d, Ab_d, b_d)

    @property
    def road_up(self):
        nx = 4          # Number of state variables
        lw = 0.38       # Width of one road lane [m]
        ll_h = 4.40     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03     # Length of road segments 3 and 4 [m] (Vertical roads)

        ng = 4; nc = 0; nb = 0
        c_u = np.array([[-0.3],     # Center of road in x-direction (Position)
                              [1.21],     # Center of road in y-direction (Position)
                              [-0.5],     # Velocity in x-direction
                              [-0.5]      # Velocity in y-direction
                        ])
        Ac_u = np.zeros((nc, ng))
        b_u = np.zeros((nc, 1))
        Ab_u = np.zeros((nc, nb))

        Gc_u = np.array([
            [ll_h/2, 0.0 , 0.0, 0.0],
            [0.0   , lw/2, 0.0, 0.0],
            [0.0   , 0.0 , self.self.vx_max/2, 0.0],
            [0.0   , 0.0 , 0.0, self.self.vy_max/2]
        ])
        Gb_u = np.zeros((ng, nb))

        return HybridZonotope(Gc_u, Gb_u, c_u, Ac_u, Ab_u, b_u)

    @property
    def road_right(self):
        '''
        Velocity limits:
            - x-direction: -0.1 -> 0.1
            - y-direction:  0.4 -> 1.0
        '''
        nx = 4          # Number of state variables
        lw = 0.38       # Width of one road lane [m]
        ll_v = 2.03 - 0.02     # Length of road segments 3 and 4 [m] (Vertical roads)

        ng = 4; nc = 0; nb = 0
        Ac_r = np.zeros((nc, ng))
        b_r = np.zeros((nc, 1))
        Ab_r = np.zeros((nc, nb))

        Gc_r = np.array([
            [lw/2, 0.0   , 0.00, 0.00],
            [0.0 , ll_v/2, 0.00, 0.00],
            [0.0 , 0.0   , 0.20, 0.00],
            [0.0 , 0.0   , 0.00, 0.025]
        ])
        c_r = np.array([ [-0.3 + 2.01],     # Center of road in x-direction (Position)
                            [ 0.0],     # Center of road in y-direction (Position)
                            [ 0.2],     # Velocity in x-direction
                            [ 0.475]      # Velocity in y-direction
                        ])
        Gb_r = np.zeros((nx, 0))

        print(f'Road right:')
        print(f'x :\t[{(c_r[0, 0] - Gc_r[0, 0]):+0,.2f} -> {(c_r[0, 0] + Gc_r[0, 0]):+0,.2f}]')
        print(f'y :\t[{(c_r[1, 0] - Gc_r[1, 1]):+0,.2f} -> {(c_r[1, 0] + Gc_r[1, 1]):+0,.2f}]')
        print(f'vx:\t[{(c_r[2, 0] - Gc_r[2, 2]):+0,.2f} -> {(c_r[2, 0] + Gc_r[2, 2]):+0,.2f}]')
        print(f'vy:\t[{(c_r[3, 0] - Gc_r[3, 3]):+0,.2f} -> {(c_r[3, 0] + Gc_r[3, 3]):+0,.2f}]')

        return HybridZonotope(Gc_r, Gb_r, c_r, Ac_r, Ab_r, b_r)

    @property
    def road_left(self):
        nx = 4          # Number of state variables
        lw = 0.38       # Width of one road lane [m]
        ll_h = 4.40     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_v = 2.03     # Length of road segments 3 and 4 [m] (Vertical roads)

        ng = 4; nc = 0; nb = 0
        c_r = np.array([ [-0.3 - 2.01],     # Center of road in x-direction (Position)
                            [ 0.0],     # Center of road in y-direction (Position)
                            [ 0.0],     # Velocity in x-direction
                            [-0.5]      # Velocity in y-direction
                        ])
        Ac_r = np.zeros((nc, ng))
        b_r = np.zeros((nc, 1))
        Ab_r = np.zeros((nc, nb))

        Gc_r = np.array([
            [lw/2, 0.0   , 0.0, 0.0],
            [0.0 , ll_v/2, 0.0, 0.0],
            [0.0 , 0.0   , self.self.vx_max/2, 0.0],
            [0.0 , 0.0   , 0.0, self.self.vy_max/2]
        ])        
        Gb_r = np.zeros((nx, 0))
        
        return HybridZonotope(Gc_r, Gb_r, c_r, Ac_r, Ab_r, b_r)

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
            [ll_h/2, 0.0 ],
            [0.0   , lw/2]
        ])
        Gb_road_h = np.array([
            [  0.0],
            [0.795]
        ])
        Gc_road_v = np.array([
            [lw/2, 0.0   ],
            [0.0 , ll_v/2]
        ])
        Gb_road_v = np.array([  [1.59],
                                [ 0.0]
        ])
        road_h = HybridZonotope(Gc_road_h, Gb_road_h, c_road, Ac_road, Ab_road, b_road)
        road_v = HybridZonotope(Gc_road_v, Gb_road_v, c_road, Ac_road, Ab_road, b_road)

        # road = self.zono_op.union_hz_hz(road_h, road_v)
        road = self.zono_op.union_hz_hz_v2(road_h, road_v)

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
            [l/2, 0.0],
            [0.0, w/2]
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
            [l/2, 0.0],
            [0.0, w/2]
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
        # w = 0.20        # Width of vehicle [m]
        w = 0.40        # Width of vehicle [m]
        l = 0.60        # Length of vehicle [m]
        nx = 4          # Number of state variables
        ng = 4; nb = 0; nc = 0


        eps = 1e-4
        Gc = np.array([
            [l/2, 0.0, 0.0, 0.0],
            [0.0, w/2, 0.0, 0.0],
            [0.0, 0.0, eps, 0.0],
            [0.0, 0.0, 0.0, eps]
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [ 1.3],
                        [-1.2],
                        [ 0.5],
                        [ 0.0]
                        ])
        
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))        

        outer_road_car = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        print(f'Car outer down:')
        print(f'x :\t[{(c[0, 0] - Gc[0, 0]):+0,.2f} -> {(c[0, 0] + Gc[0, 0]):+0,.2f}]')
        print(f'y :\t[{(c[1, 0] - Gc[1, 1]):+0,.2f} -> {(c[1, 0] + Gc[1, 1]):+0,.2f}]')
        print(f'vx:\t[{(c[2, 0] - Gc[2, 2]):+0,.2f} -> {(c[2, 0] + Gc[2, 2]):+0,.2f}]')
        print(f'vy:\t[{(c[3, 0] - Gc[3, 3]):+0,.2f} -> {(c[3, 0] + Gc[3, 3]):+0,.2f}]')

        return outer_road_car
    
    @property
    def car_outer_u(self):

        # w = 0.38        # Width of vehicle [m]
        w = 0.20        # Width of vehicle [m]
        l = 0.60        # Length of vehicle [m]
        nx = 4          # Number of state variables
        ng = 4; nb = 0; nc = 0

        # Car starts at the bottom of the road with a slightly positive velocity in the x-direction
        eps = 1e-2
        Gc = np.array([
            [l/2, 0.0, 0.0, 0.0],
            [0.0, w/2, 0.0, 0.0],
            [0.0, 0.0, eps, 0.0],
            [0.0, 0.0, 0.0, eps]
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [-0.3],
                        [1.2],
                        [ -eps],
                        [ 0.0]
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
        nx = 4          # Number of state variables
        ng = 4; nb = 0; nc = 0

        # Car starts at the bottom of the road with a slightly positive velocity in the x-direction
        # eps = 1e-2
        # Gc = np.array([
        #     [w/2, 0.0, 0.0, 0.0],
        #     [0.0, l/2, 0.0, 0.0],
        #     [0.0, 0.0, eps, 0.0],
        #     [0.0, 0.0, 0.0, eps]
        # ])
        # Gb = np.zeros((nx, nb))
        # c = np.array([  [-0.3 + 2.01],
        #                 [0.0],
        #                 [ 0.0],
        #                 [ eps]
        #                 ])
        eps = 1e-2
        Gc = np.array([
            [w/2, 0.0, 0.0, 0.0],
            [0.0, l/2, 0.0, 0.0],
            [0.0, 0.0, eps, 0.0],
            [0.0, 0.0, 0.0, eps]
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [-0.3 + 2.01],
                        [0.0],
                        [ 0.0],
                        [ 0.5]
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
        nx = 4          # Number of state variables
        ng = 4; nb = 0; nc = 0

        # Car starts at the bottom of the road with a slightly positive velocity in the x-direction
        eps = 1e-2
        Gc = np.array([
            [w/2, 0.0, 0.0, 0.0],
            [0.0, l/2, 0.0, 0.0],
            [0.0, 0.0, eps, 0.0],
            [0.0, 0.0, 0.0, eps]
        ])
        Gb = np.zeros((nx, nb))
        c = np.array([  [-0.3 - 2.01],
                        [0.0],
                        [ 0.0],
                        [-eps]
                        ])
        
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))        

        outer_road_car = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return outer_road_car