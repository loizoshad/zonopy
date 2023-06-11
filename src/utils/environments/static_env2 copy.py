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
    def __init__(self, dynamics, space = None):
        # Dynamics Model
        self.A = dynamics.A
        self.B = dynamics.B
                
        if space == 'inner':
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

        if space == 'outer':
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

        if space == 'full':
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
    def __init__(self, zono_op, dynamics, visualizer) -> None:
        self.zono_op = zono_op      # Class for Zonotope operations
        self.vis = visualizer       # Class for visualizing the results
        self.dynamics = dynamics    # Class for dynamics of the system
        self.A = dynamics.A         # System dynamics
        self.B = dynamics.B         # System dynamics

    def get_state_space_outer(self):
        '''
        This function returns the state space X for the outer section
        '''

        # Extract only the dimensions that are relevant to the state space
        # Only the first 'ns' dimensions are relevant to the state space

        ns = 2  # Number of states
        ni = 2  # Number of control inputs

        space_outer, _, _ = self.road_outer

        ng = space_outer.Gc.shape[1]
        nc = space_outer.Ac.shape[0]
        nb = space_outer.Ab.shape[1]

        Gc = space_outer.Gc[:ns, :]
        Gb = space_outer.Gb[:ns, :]
        c = space_outer.C[:ns]
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))




        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)


    def get_target_space_outer(self):
        '''
        This function returns the target space T for the outer section.
        The outer target space T contains the free parking spots of the outer section
        '''

        target_outer, _, _ = self.park_1

        return target_outer


    def get_input_space_outer(self):
        '''
        This function returns the input space U for the outer section.
        '''

        space_outer, _, _ = self.road_outer

        # Extract only the dimensions that are relevant to the input space
        # Only the last 'ni' dimensions are relevant to the input space
        
        ns = 2  # Number of states
        ni = 2  # Number of control inputs

        ng = space_outer.Gc.shape[1]
        nc = space_outer.Ac.shape[0]
        nb = space_outer.Ab.shape[1]

        Gc = space_outer.Gc[ns:4, ns:4]
        Gb = space_outer.Gb[ns:4, :]
        c = space_outer.C[2:4]
        Ac = np.zeros((nc, ni))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)



    def get_sets(self):
        '''
        This class returns a list of hybrid zonotopes each representing a set
        in the environment.
        '''
        env = {
            'sets': [],
            'vis': [],
            'colors': []
        }

        # Road
        road_outer = self.road_outer
        road_inner = self.road_inner
        road_park_1 = self.road_park_1
        road_park_2 = self.road_park_2
        env['sets'].append(road_outer[0]); env['vis'].append(road_outer[1]); env['colors'].append(road_outer[2])
        env['sets'].append(road_inner[0]); env['vis'].append(road_inner[1]); env['colors'].append(road_inner[2])
        env['sets'].append(road_park_1[0]); env['vis'].append(road_park_1[1]); env['colors'].append(road_park_1[2])
        env['sets'].append(road_park_2[0]); env['vis'].append(road_park_2[1]); env['colors'].append(road_park_2[2])
        
        # Parking
        park_1 = self.park_1
        park_2 = self.park_2
        env['sets'].append(park_1[0]); env['vis'].append(park_1[1]); env['colors'].append(park_1[2])
        env['sets'].append(park_2[0]); env['vis'].append(park_2[1]); env['colors'].append(park_2[2])

        # Obstacles
        obs_1 = self.obstacle_1
        obs_2 = self.obstacle_2
        obs_3 = self.obstacle_3
        obs_4 = self.obstacle_4
        env['sets'].append(obs_1[0]); env['vis'].append(obs_1[1]); env['colors'].append(obs_1[2])
        env['sets'].append(obs_2[0]); env['vis'].append(obs_2[1]); env['colors'].append(obs_2[2])
        env['sets'].append(obs_3[0]); env['vis'].append(obs_3[1]); env['colors'].append(obs_3[2])
        env['sets'].append(obs_4[0]); env['vis'].append(obs_4[1]); env['colors'].append(obs_4[2])

        # Occupied Parking Spots
        occ_1 = self.occupied_1
        occ_2 = self.occupied_2
        occ_3 = self.occupied_3
        env['sets'].append(occ_1[0]); env['vis'].append(occ_1[1]); env['colors'].append(occ_1[2])
        env['sets'].append(occ_2[0]); env['vis'].append(occ_2[1]); env['colors'].append(occ_2[2])
        env['sets'].append(occ_3[0]); env['vis'].append(occ_3[1]); env['colors'].append(occ_3[2])

        return env['sets'], env['vis'], env['colors']


    def get_inner_env(self):
        '''
        Returns both the obstacles and the safe roads of the inner environment.
        '''
        env = {
            'sets': [],
            'vis': [],
            'colors': []
        }

        # Road
        road_inner = self.road_inner
        road_park_1 = self.road_park_1
        road_park_2 = self.road_park_2
        env['sets'].append(road_inner[0]); env['vis'].append(road_inner[1]); env['colors'].append(road_inner[2])
        env['sets'].append(road_park_1[0]); env['vis'].append(road_park_1[1]); env['colors'].append(road_park_1[2])
        env['sets'].append(road_park_2[0]); env['vis'].append(road_park_2[1]); env['colors'].append(road_park_2[2])        

        # Obstacles
        obs_1 = self.obstacle_1
        obs_2 = self.obstacle_2
        obs_3 = self.obstacle_3
        obs_4 = self.obstacle_4
        env['sets'].append(obs_1[0]); env['vis'].append(obs_1[1]); env['colors'].append(obs_1[2])
        env['sets'].append(obs_2[0]); env['vis'].append(obs_2[1]); env['colors'].append(obs_2[2])
        env['sets'].append(obs_3[0]); env['vis'].append(obs_3[1]); env['colors'].append(obs_3[2])
        env['sets'].append(obs_4[0]); env['vis'].append(obs_4[1]); env['colors'].append(obs_4[2])

        # Occupied Parking Spots
        occ_1 = self.occupied_1
        occ_2 = self.occupied_2
        env['sets'].append(occ_1[0]); env['vis'].append(occ_1[1]); env['colors'].append(occ_1[2])
        env['sets'].append(occ_2[0]); env['vis'].append(occ_2[1]); env['colors'].append(occ_2[2])

        return env['sets'], env['vis'], env['colors']

    @property
    def road_outer(self):
        nx = 2          # Number of state variables
        ni = 2          # Number of input variables
        lw = 0.38        # Width of one road lane [m]
        ll_12 = 4.40    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_34 = 2.03    # Length of road segments 3 and 4 [m]   (Vertical roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [-0.3], [0.0], [0.0], [0.0] ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_12 = np.array([
            [ll_12/2, 0.0, 0.0, 0.0],
            [  0.0  , lw/2, 0.0, 0.0],
            [  0.0  , 0.0, 0.5, 0.0],
            [  0.0  , 0.0, 0.0, 0.5]
        ])
        Gb_road_12 = np.array([
            [0.0],
            [1.21],
            [-0.45],
            [0.0]
        ])
        Gc_road_34 = np.array([
            [  lw/2 , 0.0, 0.0, 0.0],
            [  0.0, ll_34/2 , 0.0, 0.0],
            [  0.0, 0.0, 0.5, 0.0],
            [  0.0, 0.0, 0.0, 0.5]
        ])
        Gb_road_34 = np.array([
            [2.01],
            [0.0],
            [0.0],
            [0.5]
        ])
        road_12 = HybridZonotope(Gc_road_12, Gb_road_12, c_road, Ac_road, Ab_road, b_road)
        road_34 = HybridZonotope(Gc_road_34, Gb_road_34, c_road, Ac_road, Ab_road, b_road)

        road = self.zono_op.union_hz_hz(road_12, road_34)

        road_vis = HybridZonotope(road.Gc[0:2, :], road.Gb[0:2, :], road.C[0:2, :], road.Ac, road.Ab, road.b)

        return [road, road_vis, color]

    @property
    def road_inner(self):
        nx = 2          # Number of state variables
        ni = 2          # Number of input variables
        lw = 0.38        # Width of one road lane [m]
        ll_12 = 3.56    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_34 = 1.21    # Length of road segments 3 and 4 [m]   (Vertical roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [-0.3], [0.0], [0.0], [0.0] ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_12 = np.array([
            [ll_12/2, 0.0, 0.0, 0.0],
            [  0.0  , lw/2, 0.0, 0.0],
            [  0.0  , 0.0, 0.5, 0.0],
            [  0.0  , 0.0, 0.0, 0.5]
        ])
        Gb_road_12 = np.array([
            [0.0],
            [0.795],
            [0.45],
            [0.0]
        ])
        Gc_road_34 = np.array([
            [  lw/2 , 0.0, 0.0, 0.0],
            [  0.0, ll_34/2 , 0.0, 0.0],
            [  0.0, 0.0, 0.5, 0.0],
            [  0.0, 0.0, 0.0, 0.5]
        ])
        Gb_road_34 = np.array([
            [1.59],
            [0.0],
            [0.0],
            [-0.5]
        ])
        road_12 = HybridZonotope(Gc_road_12, Gb_road_12, c_road, Ac_road, Ab_road, b_road)
        road_34 = HybridZonotope(Gc_road_34, Gb_road_34, c_road, Ac_road, Ab_road, b_road)

        road = self.zono_op.union_hz_hz(road_12, road_34)

        road_vis = HybridZonotope(road.Gc[0:2, :], road.Gb[0:2, :], road.C[0:2, :], road.Ac, road.Ab, road.b)

        return [road, road_vis, color]

    @property
    def road_park_1(self):
        nx = 2          # Number of state variables
        ni = 2          # Number of input variables
        w = 0.394        # Width of one road lane [m]
        l = 2.6    # Length of road segments 1 and 2 [m] (Horizontal roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 4; nc = 0; nb = 0
        c = np.array([ [-0.3], [-0.3], [0.0], [0.25] ])
        Ac = np.zeros((nc, ng))
        b = np.zeros((nc, 1))
        Ab = np.zeros((nc, nb))
        Gc = np.array([
            [  l/2, 0.0, 0.0, 0.0],
            [  0.0, w/2, 0.0, 0.0],
            [  0.0, 0.0, 0.5, 0.0],
            [  0.0, 0.0, 0.0, 0.25]
        ])
        Gb = np.zeros((ng, nb))

        road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_vis = HybridZonotope(road.Gc[0:2, :], road.Gb[0:2, :], road.C[0:2, :], road.Ac, road.Ab, road.b)

        return [road, road_vis, color]

    @property
    def road_park_2(self):
        nx = 2          # Number of state variables
        ni = 2          # Number of input variables
        w = 0.1        # Width of one road lane [m]
        l = 0.4    # Length of road segments 1 and 2 [m] (Horizontal roads)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        ng = 4; nc = 0; nb = 1
        c = np.array([ [-0.3], [-0.55], [0.0], [0.25] ])
        Ac = np.zeros((nc, ng))
        b = np.zeros((nc, 1))
        Ab = np.zeros((nc, nb))
        Gc = np.array([
            [  l/2, 0.0, 0.0, 0.0],
            [  0.0, w/2, 0.0, 0.0],
            [  0.0, 0.0, 0.5, 0.0],
            [  0.0, 0.0, 0.0, 0.25]
        ])
        Gb = np.array([
            [0.8],
            [0.0],
            [0.0],
            [0.0]
        ])


        road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_vis = HybridZonotope(road.Gc[0:2, :], road.Gb[0:2, :], road.C[0:2, :], road.Ac, road.Ab, road.b)

        return [road, road_vis, color]

    @property
    def park_1(self):
        pw = 0.4    # Width of parking slot [m]
        pl = 0.6      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
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
        # color = (0.4,  0.4,  0.4, 1.0)
        color = (0.000001,  0.000001,  0.000001, 0.6)  # Gray

        return [parking, parking, color]
    
    @property
    def park_2(self):
        pw = 0.4    # Width of parking slot [m]
        pl = 0.6      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
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
        # color = (0.4,  0.4,  0.4, 1.0)
        color = (0.000001,  0.000001,  0.000001, 0.6)  

        return [parking, parking, color]

    @property
    def obstacle_1(self):
        '''
        Internal obstacle
        '''
        nx = 2  # Number of state variables
        ni = 2  # Number of input variables
        ng = 4; nc = 0; nb = 0
        # Obstacle 1
        l = 2.805
        w = 0.1
        Gc_obs = np.array([
            [l/2, 0.0, 0.0, 0.0],
            [0.0, w/2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        Gb_obs = np.zeros((nx + ni, nb))
        c_obs = np.array([ [-0.3], [0.55], [0.0], [0.0] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_vis = HybridZonotope(obs.Gc[0:2, :], obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
        # color = (0.949, 0.262, 0.227, 1.0)  # Red
        color = (0.949, 0.262, 0.227, 0.6)  # Red

        return [obs, obs_vis, color]

    @property
    def obstacle_2(self):
        '''
        Internal obstacle
        '''
        nx = 2  # Number of state variables
        ni = 2  # Number of input variables
        ng = 4; nc = 0; nb = 1
        # Obstacle 1
        l = 1.1
        w = 0.1
        Gc_obs = np.array([
            [w/2, 0.0, 0.0, 0.0],
            [0.0, l/2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        Gb_obs = np.array([
            [1.35],
            [0.0],
            [0.0],
            [0.0]
        ])
        c_obs = np.array([ [-0.3], [-0.05], [0.0], [0.0] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_vis = HybridZonotope(obs.Gc[0:2, :], obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
       # color = (0.949, 0.262, 0.227, 1.0)  # Red
        color = (0.949, 0.262, 0.227, 0.6)  # Red

        return [obs, obs_vis, color]

    @property
    def obstacle_3(self):
        '''
        Internal obstacle
        '''
        nx = 2  # Number of state variables
        ni = 2  # Number of input variables
        ng = 4; nc = 0; nb = 1
        # Obstacle 1
        l = 0.3
        w = 0.1
        Gc_obs = np.array([
            [l/2, 0.0, 0.0, 0.0],
            [0.0, w/2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        Gb_obs = np.array([
            [1.15],
            [0.0],
            [0.0],
            [0.0]
        ])
        c_obs = np.array([ [-0.3], [-0.55], [0.0], [0.0] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_vis = HybridZonotope(obs.Gc[0:2, :], obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
        # color = (0.949, 0.262, 0.227, 1.0)  # Red
        color = (0.949, 0.262, 0.227, 0.6)  # Red
    
        return [obs, obs_vis, color]
    
    @property
    def obstacle_4(self):
        '''
        Internal obstacle
        '''
        nx = 2  # Number of state variables
        ni = 2  # Number of input variables
        ng = 4; nc = 0; nb = 0
        # Obstacle 1
        l = 1.2
        w = 0.1
        Gc_obs = np.array([
            [l/2, 0.0, 0.0, 0.0],
            [0.0, w/2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        Gb_obs = np.zeros((ng, nb))
        c_obs = np.array([ [-0.3], [-0.55], [0.0], [0.0] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_vis = HybridZonotope(obs.Gc[0:2, :], obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
        # color = (0.949, 0.262, 0.227, 1.0)  # Red
        color = (0.949, 0.262, 0.227, 0.6)  # Red
    
        return [obs, obs_vis, color]

    @property
    def occupied_1(self):
        '''
        Occupied Parking Spot
        '''
        nx = 2  # Number of state variables
        ni = 2  # Number of input variables
        ng = 4; nc = 0; nb = 0
        # Obstacle 1
        l = 0.6
        w = 0.29
        Gc = np.array([
            [w/2, 0.0, 0.0, 0.0],
            [0.0, l/2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        Gb = np.zeros((ng, nb))
        c = np.array([ [0.85], [0.2], [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        occ = HybridZonotope(Gc, Gb, c, Ac, Ab, b)
        occ_vis = HybridZonotope(occ.Gc[0:2, :], occ.Gb[0:2, :], occ.C[0:2, :], occ.Ac, occ.Ab, occ.b)
        # color = (0.949, 0.262, 0.227, 1.0)  # Red
        color = (0.949, 0.262, 0.227, 0.6)  # Red
    
        return [occ, occ_vis, color]

    @property
    def occupied_2(self):
        '''
        Occupied Parking Spot
        '''
        nx = 2  # Number of state variables
        ni = 2  # Number of input variables
        ng = 4; nc = 0; nb = 0
        # Obstacle 1
        l = 0.6
        w = 0.4*4.7
        Gc = np.array([
            [w/2, 0.0, 0.0, 0.0],
            [0.0, l/2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        Gb = np.zeros((ng, nb))
        c = np.array([ [-0.65], [0.2], [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        occ = HybridZonotope(Gc, Gb, c, Ac, Ab, b)
        occ_vis = HybridZonotope(occ.Gc[0:2, :], occ.Gb[0:2, :], occ.C[0:2, :], occ.Ac, occ.Ab, occ.b)
        # color = (0.949, 0.262, 0.227, 1.0)  # Red
        color = (0.949, 0.262, 0.227, 0.6)  # Red
    
        return [occ, occ_vis, color]

    @property
    def occupied_3(self):
        '''
        Occupied Parking Spot
        '''
        nx = 2  # Number of state variables
        ni = 2  # Number of input variables
        ng = 4; nc = 0; nb = 1
        # Obstacle 1
        l = 0.6
        w = 0.4
        Gc = np.array([
            [l/2, 0.0, 0.0, 0.0],
            [0.0, w/2, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]
        ])
        Gb = np.array([
            [0.0],
            [0.4],
            [0.0],
            [0.0]
        ])
        c = np.array([ [2.2], [0.0], [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        occ = HybridZonotope(Gc, Gb, c, Ac, Ab, b)
        occ_vis = HybridZonotope(occ.Gc[0:2, :], occ.Gb[0:2, :], occ.C[0:2, :], occ.Ac, occ.Ab, occ.b)
        # color = (0.949, 0.262, 0.227, 1.0)  # Red
        color = (0.949, 0.262, 0.227, 0.6)  # Red
    
        return [occ, occ_vis, color]

    @property
    def parking_lot(self):
        '''
        Get a rectangle representing the entire parking lot
        '''
        nx = 2  # Number of state variables
        ni = 2  # Number of input variables
        ng = 4; nc = 0; nb = 0
        # Obstacle 1
        l = 2.805
        w = 1.2
        Gc = np.array([
            [l/2, 0.0, 0.0, 0.0],
            [0.0, w/2, 0.0, 0.0],
            [0.0, 0.0, 0.5, 0.0],
            [0.0, 0.0, 0.0, 0.5]
        ])
        Gb = np.zeros((nx + ni, nb))
        c = np.array([ [-0.3], [0.0], [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        park_lot = HybridZonotope(Gc, Gb, c, Ac, Ab, b)
        park_lot_vis = HybridZonotope(park_lot.Gc[0:2, :], park_lot.Gb[0:2, :], park_lot.C[0:2, :], park_lot.Ac, park_lot.Ab, park_lot.b)
        color = (0.949, 0.262, 0.227, 0.6)  # Red

        return [park_lot, park_lot_vis, color]        








    def vis_env(self):
        # Visualize environment
        self.vis_background()
        self.vis_sets()

        plt.show()

    def vis_background(self):
        # Visualize background
        img = mpimg.imread('./images/park_env.png')
        self.vis.ax.imshow(img, extent=[-2.5, 2.5, -1.4, 1.4], zorder = 1)

    def vis_sets(self):
        # Visualize hybrid zonotopes
        _, hz_vis, colors = self.get_sets()
        self.vis.vis_hz(
            hzonotopes = hz_vis,
            colors = colors,
            zorder = 2)
