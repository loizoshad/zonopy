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

'''


class ParamBRSV2:
    def __init__(self):

        step_size = 0.05

        # space_grid 
        self.space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': []         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
        }

        # Vertical Road points (outer)
        for x in np.arange(1.55 + step_size/2, 1.9 - step_size/2, step_size):
            for y in np.arange(-1.05 + step_size/2, 0.20 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)
        
        # Vertical Road points (inner)
        for x in np.arange(1.10 + step_size/2, 1.45 - step_size/2, step_size):
            for y in np.arange(-0.175 + step_size/2, 0.95 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)

        # Passage
        for x in np.arange(1.45 + step_size/2, 1.55 - step_size/2, step_size):
            for y in np.arange(-0.20 + step_size/2, 0.20 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)

        # Horizontal Road points (outer)
        for x in np.arange(0.3 + step_size/2, 1.9 - step_size/2, step_size):
            for y in np.arange(-1.40 + step_size/2, -1.05 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)
        
        # Horizontal Road points (inner)
        for x in np.arange(-0.5 + step_size/2, 1.10 - step_size/2, step_size):
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


class Ego:
    def __init__(self, zono_op, visualizer) -> None:

        self.zono_op = zono_op      # Class for Zonotope operations
        self.vis = visualizer       # Class for visualizing the results        
        self.dynamics = DynamicsModel()    # Class for dynamics of the system
        
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
        return self.road

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
    def road(self):
        lw = 8 * self.step_size
        ll_h = 50 * self.step_size     # Length of road segments 1 and 2 [m] (Horizontal roads)
        nx = 4; ng = 4; nc = 0; nb = 0

        # Horizontal Road Section
        c_road_h = np.array([ [0.0], [0.0], [ self.vx_max/2], [ 0.0] ])
        Gc_road_h = np.diag(np.array([ ll_h/2  , lw/2, self.vx_max/2, self.vy_max ]))
        Gb_road_h = np.zeros((nx, nb))
        Ac_road = np.zeros((nc, ng))
        Ab_road_h = np.zeros((nc, nb))
        b_road = np.zeros((nc, 1))
        road = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)

        return road       

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






class NonEgoParamBRSV2:
    def __init__(self):

        step_size = 0.05

        # space_grid 
        self.space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': []         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
        }

        # Vertical Road points (outer)
        for x in np.arange(1.55 + step_size/2, 1.9 - step_size/2, step_size):
            for y in np.arange(-1.05 + step_size/2, 0.20 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)

        # Horizontal Road points (outer)
        for x in np.arange(0.3 + step_size/2, 1.90 - step_size/2, step_size):
            for y in np.arange(-1.40 + step_size/2, -1.05 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)

class NonEgo:
    '''
    This class contains the safe state space of the system

    The state space consists of all the places in the environment that are considered to be immediately safe.
    By immediately safe we mean that by being at that specific place, the system is safe from collision.
    However, that does not mean that the system is safe from collision in the future.

    This version does not take into account limitation on changing lines or traffic direction of the lanes.
    '''
    
    def __init__(self, visualizer):
        self.step_size = 0.05
        self.zono_op = ZonoOperations()
        self.vx_max = 1.0    # TODO: Define based on dynamics model
        self.vy_max = 1.0    # TODO: Define based on dynamics model
        self.vis = visualizer       # Class for visualizing the results        
        self.brs_settings = NonEgoParamBRSV2()

        self.dynamics = DynamicsModel()    # Class for dynamics of the system
        self.A = self.dynamics.A         # System dynamics
        self.B = self.dynamics.B         # System dynamics
        self.W = self.dynamics.W         # System dynamics        


    def vis_background(self):
        # Visualize background
        img = mpimg.imread('./images/park_env.png')
        self.vis.ax.imshow(img, extent=[-2.5, 2.5, -1.4, 1.4], zorder = 1)        

    def state_space(self, options = 1):
        return self.non_ego_1_space

    def car(self, options = 1):
        return self.car_1


    @property
    def non_ego_1_space(self):
        lw = 8 * self.step_size
        ll_v = 40 * self.step_size
        nx = 4; ng = 4; nc = 0; nb = 0

        # Vertical Road Section
        c_road_v = np.array([ [10*self.step_size], [0.0*self.step_size], [ 0.0], [self.vy_max/2] ])
        Gc_road_v = np.diag(np.array([ lw/2 , (ll_v - 2*self.step_size)/2, self.vx_max, self.vy_max/2 ]))
        Gb_road_v = np.zeros((nx, nb))
        Ac_road = np.zeros((nc, ng))
        Ab_road_v = np.zeros((nc, nb))
        b_road = np.zeros((nc, 1))
        non_ego_1_space = HybridZonotope(Gc_road_v, Gb_road_v, c_road_v, Ac_road, Ab_road_v, b_road)

        return non_ego_1_space

    @property
    def car_1(self):
        step_size = 0.05
        w = 8*step_size # Width of vehicle [m]
        l = 0.60        # Length of vehicle [m]
        nx = 4          # Number of state variables
        ng = 4; nb = 0; nc = 0


        eps = 1e-4
        Gc = np.diag(np.array([ w/2, l/2, eps, eps ]))
        Gb = np.zeros((nx, nb))
        c = np.array([  [10*step_size], [-13*self.step_size], [ 0.0], [ 0.6] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))        

        car_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # print(f'Car outer down:')
        # print(f'x :\t[{(c[0, 0] - Gc[0, 0]):+0,.2f} -> {(c[0, 0] + Gc[0, 0]):+0,.2f}]')
        # print(f'y :\t[{(c[1, 0] - Gc[1, 1]):+0,.2f} -> {(c[1, 0] + Gc[1, 1]):+0,.2f}]')
        # print(f'vx:\t[{(c[2, 0] - Gc[2, 2]):+0,.2f} -> {(c[2, 0] + Gc[2, 2]):+0,.2f}]')
        # print(f'vy:\t[{(c[3, 0] - Gc[3, 3]):+0,.2f} -> {(c[3, 0] + Gc[3, 3]):+0,.2f}]')

        return car_1
