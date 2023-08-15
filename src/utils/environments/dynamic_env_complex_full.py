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

        for x in np.arange(-1.45 + step_size/2, 1.45 - step_size/2, step_size):
            for y in np.arange(-0.25 + step_size/2, 0.25 - step_size/2, step_size):
                self.space['points'].append([x, y])
                self.space['flags'].append(0)



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


    def get_obstacle(self, X, U, I, A, B, W, bounds):
        obs = self.zono_op.one_step_frs_hz(X = X, U = U, I = I, A = A, B = B, W = W)
        obs  = self.zono_op.cz_to_hz( self.zono_op.oa_cz_to_hypercube_tight_4d( self.zono_op.oa_hz_to_cz(obs), bounds = bounds) )
        return obs
    
    def get_safe_space(self, obs, safe_space, A, B, i):
        obs_pos = HybridZonotope(obs.Gc[0:2, :], obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
        obs_pos = self.zono_op.reduce_gc_hz(obs_pos)
        print(f'obs_pos : ng = {obs_pos.ng}\t nc = {obs_pos.nc}\t nb = {obs_pos.nb}')
        brs_obs_compl = self.zono_op.complement_cz_to_hz(self.zono_op.oa_hz_to_cz(obs_pos))
        brs_obs_compl = self.zono_op.redundant_c_gc_hz_v1(brs_obs_compl)
        print(f'brs_obs_compl : ng = {brs_obs_compl.ng}\t nc = {brs_obs_compl.nc}\t nb = {brs_obs_compl.nb}')
        # brs_obs_compl = zono_op.intersection_hz_hz(brs, brs_obs_compl)         # Compute the intersection of the obstacle complement with the full BRS for the parking -> ego-vehicle

        # Compute the 'i'-step BRS from the obstacle complement
        for j in range(i):
            brs_obs_compl = self.zono_op.one_step_brs_hz(X = self.full_space, T = brs_obs_compl, D = np.block([A, B]))
            brs_obs_compl = self.zono_op.redundant_c_gc_hz_v1(brs_obs_compl)

        safe_space = self.zono_op.intersection_hz_hz(safe_space, brs_obs_compl)
        
        return safe_space


    ## V2
    # In this version we are computing the BRS of the obstacle itself to find the non safe space, then over-approximate it as a hypercube and compute its complement.
    # Then interset its complement with the full BRS space to get the safe space.
    # This will result in an under-apporximation of the safe space, but it will significantly increase the computation speed.

    def get_safe_space_v2(self, obs, safe_space, A, B, i, bounds):
        # Step 1: Remove redundancy from the 2D obstacle
        obs_pos = HybridZonotope(obs.Gc[0:2, :], obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
        brs_obs = self.zono_op.reduce_gc_hz(obs_pos)

        # Step 2: Compute the i-step BRS of the obstacle
        for j in range(i):
            brs_obs = self.zono_op.one_step_brs_hz(X = self.full_space, T = brs_obs, D = np.block([A, B]))

        # Step 3: Over-approximate as a constrained zonotope hypercube
        brs_obs  = self.zono_op.oa_cz_to_hypercube_tight_2d( self.zono_op.oa_hz_to_cz(brs_obs), bounds = bounds)
        print(f'brs_obs : ng = {brs_obs.ng}\t nc = {brs_obs.nc}')

        # Step 4: Compute the complement of the obstacle BRS
        brs_obs_compl = self.zono_op.complement_cz_to_hz(brs_obs)
        print(f'brs_obs_compl : ng = {brs_obs_compl.ng}\t nc = {brs_obs_compl.nc}\t nb = {brs_obs_compl.nb}')

        # Step 5: Compute the intersection of the obstacle complement with the full BRS from the parking
        safe_space = self.zono_op.intersection_hz_hz(safe_space, brs_obs_compl)
        
        return safe_space







    def vis_background(self):
        # Visualize background
        img = mpimg.imread('./images/park_env.png')
        self.vis.ax.imshow(img, extent=[-2.5, 2.5, -1.4, 1.4], zorder = 1)

    def vis_safe_space(self, hz):
        '''
        Visualizes a backward reachable set repsented by a hybrid zonotope

        This version is particularly useful when you already know which points can be contained in the BRS
        '''            
        marker_size = 0.25 * 39.36           # 0.1m in inches
        marker_size = marker_size**2        # area of the marker

        space = self.brs_settings.space['points']
        flags = self.brs_settings.space['flags']

        # Loop through all points in the space
        for i, point in enumerate(space):
            p = np.array([ [point[0]], [point[1]] ])
            if self.zono_op.is_inside_hz(hz, p):
                flags[i] = 1
                self.vis.ax.scatter(p[0], p[1], marker = 's', s = marker_size, color = '#6C8EBF', alpha = 1.0, zorder = 11, edgecolors = 'face' )

        # Update flags
        self.brs_settings.space['flags'] = flags

    @property
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
    def state_space(self):
        lw = 8 * self.step_size
        # ll_h = 50 * self.step_size     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_h = 2.8 # [m]
        nx = 4; ng = 4; nc = 0; nb = 0

        # Horizontal Road Section
        c_road_h = np.array([ [0.0], [0.0], [ self.vx_max/2 + 0.5], [ 0.0] ])
        Gc_road_h = np.diag(np.array([ ll_h/2  , lw/2, self.vx_max/2, self.vy_max ]))
        Gb_road_h = np.zeros((nx, nb))
        Ac_road = np.zeros((nc, ng))
        Ab_road_h = np.zeros((nc, nb))
        b_road = np.zeros((nc, 1))
        road = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)

        return road       


    @property
    def full_space(self):
        lw = 20.0
        # ll_h = 50 * self.step_size     # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_h = 20.0 # [m]
        nx = 4; ng = 4; nc = 0; nb = 0

        # Horizontal Road Section
        c_road_h = np.array([ [0.0], [0.0], [ 0.5 + 0.5 + 0.25], [ 0.0] ])
        # Gc_road_h = np.diag(np.array([ ll_h/2  , lw/2, self.vx_max/2, 0.0 ]))
        Gc_road_h = np.diag(np.array([ ll_h/2  , lw/2, 0.25, 0.0 ]))
        Gb_road_h = np.zeros((nx, nb))
        Ac_road = np.zeros((nc, ng))
        Ab_road_h = np.zeros((nc, nb))
        b_road = np.zeros((nc, 1))
        road = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)

        return road    


    @property
    def extra_space(self):
        lw = 8 * self.step_size
        ll_h = 2.8 # [m]
        nx = 2; ng = 2; nc = 0; nb = 0

        Gb_road_h = np.zeros((nx, nb))
        Ac_road = np.zeros((nc, ng))
        Ab_road_h = np.zeros((nc, nb))
        b_road = np.zeros((nc, 1))

        # Road 1
        c_road_h = np.array([ [-1.2], [0.0] ])
        Gc_road_h = np.diag(np.array([ lw/2  , ll_h/2 ]))
        road_1 = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)

        # Road 2
        c_road_h = np.array([ [-0.4], [0.0] ])
        Gc_road_h = np.diag(np.array([ lw/2  , ll_h/2 ]))
        road_2 = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)

        # Road 3
        c_road_h = np.array([ [0.4], [0.0] ])
        Gc_road_h = np.diag(np.array([ lw/2  , ll_h/2 ]))
        road_3 = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)

        # Road 4
        c_road_h = np.array([ [1.2], [0.0] ])
        Gc_road_h = np.diag(np.array([ lw/2  , ll_h/2 ]))
        road_4 = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)

        # Road 5 (Horizontal Top)
        c_road_h = np.array([ [0.0], [1.2] ])
        Gc_road_h = np.diag(np.array([ ll_h/2  , lw/2 ]))
        road_5 = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)

        # Road 6 (Horizontal Bottom)
        c_road_h = np.array([ [0.0], [-1.2] ])
        Gc_road_h = np.diag(np.array([ ll_h/2  , lw/2 ]))
        road_6 = HybridZonotope(Gc_road_h, Gb_road_h, c_road_h, Ac_road, Ab_road_h, b_road)


        return [road_1, road_2, road_3, road_4, road_5, road_6]



    @property
    def parking(self):
        pw = 0.4        # Width of parking slot [m]
        pl = 0.6        # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 0; nc = 0
        Gc_park = np.array([
            [pl/2, 0.0],
            [0.0, pw/2],
        ])

        Gb_park = np.zeros((ng, nb))
        c_park = np.array([ [34*self.step_size], [0.0] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)

        return parking






class ParamBRSNonEgo1:
    def __init__(self, x_min = -2.0, x_max = 2.0, y_min = -2.0, y_max = 2.0):

        step_size = 0.05

        self.x_min = x_min; self.y_min = y_min
        self.x_max = x_max; self.y_max = y_max

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
    
    def __init__(self, visualizer, car = 1):
        self.step_size = 0.05        
        self.lw = 8 * self.step_size
        self.ll_v = 2.8
        self.vx_max = 1.0    # TODO: Define based on dynamics model
        self.vy_max = 1.0    # TODO: Define based on dynamics model
        
        self.zono_op = ZonoOperations()
        self.vis = visualizer       # Class for visualizing the results        
        self.brs_settings = ParamBRSNonEgo1()

        self.dynamics = DynamicsModel()    # Class for dynamics of the system
        self.A = self.dynamics.A         # System dynamics
        self.B = self.dynamics.B         # System dynamics
        self.W = self.dynamics.W         # System dynamics      


        if car == 1:
            self.cx =  -1.2
            self.cy = -22*self.step_size
        elif car == 2:
            self.cx =  -0.4
            self.cy = -22*self.step_size
        elif car == 3:
            self.cx =  0.4
            self.cy = -22*self.step_size
        elif car == 4:
            self.cx =  1.2
            self.cy = -22*self.step_size

        # self.bounds = np.array([
        #     [self.cx - self.lw/2 - self.step_size, self.cx + self.lw/2 + self.step_size],
        #     [self.cy - self.ll_v/2 - self.step_size, self.cy + self.ll_v/2 + self.step_size],
        #     [-self.vx_max - self.step_size, self.vx_max + self.step_size],
        #     [0.7025 - self.step_size, 0.9425 + self.step_size]
        # ])
        self.bounds = np.array([
            [self.cx - 1.0 - self.step_size, self.cx + 1.0 + self.step_size],
            [- self.ll_v/2 - self.step_size, self.ll_v/2 + self.step_size],
            [-self.vx_max - self.step_size, self.vx_max + self.step_size],
            [0.7025 - self.step_size, 0.9425 + self.step_size]
        ])

    @property
    def state_space(self):
        nx = 4; ng = 4; nc = 0; nb = 0

        # Vertical Road Section
        c_road_v = np.array([ [self.cx], [0.0*self.step_size], [ 0.0], [0.8225] ])
        Gc_road_v = np.diag(np.array([ self.lw/2 , self.ll_v/2, self.vx_max, 0.12 ]))
        Gb_road_v = np.zeros((nx, nb))
        Ac_road = np.zeros((nc, ng))
        Ab_road_v = np.zeros((nc, nb))
        b_road = np.zeros((nc, 1))
        non_ego_1_space = HybridZonotope(Gc_road_v, Gb_road_v, c_road_v, Ac_road, Ab_road_v, b_road)

        return non_ego_1_space

    @property
    def car(self):
        step_size = 0.05
        w = 8*step_size # Width of vehicle [m]
        l = 0.60        # Length of vehicle [m]
        nx = 4          # Number of state variables
        ng = 4; nb = 0; nc = 0


        eps = 1e-4
        Gc = np.diag(np.array([ w/2, l/2, eps, eps ]))
        Gb = np.zeros((nx, nb))
        c = np.array([  [self.cx], [self.cy], [ 0.0], [ 0.65] ])
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