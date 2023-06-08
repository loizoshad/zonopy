'''
This file contains a collection of example sets represented as Hybrid Zonotopes
'''

import numpy as np
import math

from utils.sets.zonotopes import Zonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations

from matplotlib.patches import Polygon


class ParkEnv1:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()

        # Define the dynamics model in here (Create a separate class for this which this env can take)

        self.x_min = -2.5; self.x_max = 2.5; self.y_min = -1.4; self.y_max = 1.4
        self.samples_x = int(40*self.x_max); self.samples_y = int(40*self.y_max); self.max_dist = 0.3
        self.x_step = (self.x_max - self.x_min) / (self.samples_x)
        self.y_step = (self.y_max - self.y_min) / (self.samples_y)
        self.max_dist_x = math.ceil(self.max_dist / self.x_step) # Maximum number of steps in the x direction
        self.max_dist_y = math.ceil(self.max_dist / self.y_step) # Maximum number of steps in the y direction
        self.max_dist_diag = math.ceil( 1.1 * self.max_dist_x)        
        self.p1_ = np.array([ [ 1.9,  1.4],[ 1.9,  1.0] ])
        self.p2_ = np.array([ [ 1.9,  0.2],[ 1.9, -0.2] ])
        self.p3_ = np.array([ [-0.9,  0.6],[-0.5,  0.6] ])
        self.p4_ = np.array([ [-0.1,  0.6],[ 0.3,  0.6] ])

        # Create a list of all (x, y) points between the two points in p1_, p2_, p3_, p4_
        self.initial_points = []
        for i in range(int(abs(self.p3_[1][0] - self.p3_[0][0])/self.x_step) + 1):    # These are the horizontal lines
            self.initial_points.append([self.p3_[0][0] + i*self.x_step, self.p3_[0][1]])
            self.initial_points.append([self.p4_[0][0] + i*self.x_step, self.p4_[0][1]])
        for i in range(int(abs(self.p1_[1][1] - self.p1_[0][1])/self.y_step) + 2):    # These are the vertical lines
            self.initial_points.append([self.p1_[0][0], self.p1_[0][1] - i*self.y_step])
            self.initial_points.append([self.p2_[0][0], self.p2_[0][1] - i*self.y_step])            

        self.initial_points = np.array(self.initial_points)


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
        road = self.road
        env['sets'].append(road[0]); env['vis'].append(road[1]); env['colors'].append(road[2])
        # Parking
        park = self.park_1
        env['sets'].append(park[0]); env['vis'].append(park[1]); env['colors'].append(park[2])
        park = self.park_2
        env['sets'].append(park[0]); env['vis'].append(park[1]); env['colors'].append(park[2])        
        # Obstacles
        obs = self.obstacle_1
        env['sets'].append(obs[0])#; env['vis'].append(obs[1]); env['colors'].append(obs[2])
        obs = self.obstacle_2
        env['sets'].append(obs[0])#; env['vis'].append(obs[1]); env['colors'].append(obs[2])
        obs = self.obstacle_3
        env['sets'].append(obs[0])#; env['vis'].append(obs[1]); env['colors'].append(obs[2])

        return env['sets'], env['vis'], env['colors']

    def get_extra_visuals(self):
        '''
        This class returns all the extra visuals that are not part of the sets
        # TODO: Add the white polygon lines in here instead of defining them as hybrid zonotopes
        '''
        pass

    @property
    def road(self):
        nx = 2          # Number of state variables
        ni = 2          # Number of input variables
        lw = 0.4        # Width of one road lane [m]
        ll_12 = 4.40    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_34 = 1.20    # Length of road segments 3 and 4 [m]   (Vertical roads)
        color = (0.09804,  0.09804,  0.09804, 0.5)  # Gray

        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [-0.3], [0.0], [0.0], [0.0] ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_12 = np.array([
            [ll_12/2, 0.0, 0.0, 0.0],
            [  0.0  , lw, 0.0, 0.0],
            [  0.0  , 0.0, 1.0, 0.0],
            [  0.0  , 0.0, 0.0, 1.0]
        ])
        Gb_road_12 = np.array([
            [0.0],
            [1.0],
            [0.0],
            [0.0]
        ])
        Gc_road_34 = np.array([
            [  lw , 0.0, 0.0, 0.0],
            [  0.0, ll_34/2 , 0.0, 0.0],
            [  0.0, 0.0, 1.0, 0.0],
            [  0.0, 0.0, 0.0, 1.0]
        ])
        Gb_road_34 = np.array([
            [1.8],
            [0.0],
            [0.0],
            [0.0]
        ])
        road_12 = HybridZonotope(Gc_road_12, Gb_road_12, c_road, Ac_road, Ab_road, b_road)
        road_34 = HybridZonotope(Gc_road_34, Gb_road_34, c_road, Ac_road, Ab_road, b_road)

        road = self.zono_op.union_hz_hz(road_12, road_34)

        road_vis = HybridZonotope(road.Gc[0:2, :], road.Gb[0:2, :], road.C[0:2, :], road.Ac, road.Ab, road.b)

        return [road, road_vis, color]

    @property
    def obstacle_1(self):
        '''
        Internal obstacle
        '''
        nx = 2  # Number of state variables
        ng = 2; nc = 0; nb = 0
        # Obstacle 1
        Gc_obs = np.array([
            [0.6, 0.0],
            [0.0, 0.3]
        ])
        Gb_obs = np.zeros((nx, nb))
        c_obs = np.array([ [-0.3], [-0.3] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_color = (0.949, 0.262, 0.227, 1.0)  # Red
        
        return [obs, obs, obs_color]

    @property
    def obstacle_2(self):
        '''
        Internal obstacle
        '''
        ng = 2; nc = 0; nb = 1
        # Obstacle 2
        Gc_obs = np.array([
            [0.4, 0.0],
            [0.0, 0.6]
        ])
        Gb_obs = np.array([
            [1.0],
            [0.0]
        ])
        c_obs = np.array([ [-0.3], [0.0] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_color = (0.949, 0.262, 0.227, 1.0)  # Red
        
        return [obs, obs, obs_color]

    @property
    def obstacle_3(self):
        '''
        Internal obstacle
        '''
        nx = 2; ng = 2; nc = 0; nb = 0
        # Obstacle 2
        Gc_obs = np.array([
            [0.2, 0.0],
            [0.0, 0.3]
        ])
        Gb_obs = np.zeros((nx, nb))
        c_obs = np.array([ [-0.3], [0.3] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_color = (0.949, 0.262, 0.227, 1.0)  # Red
        
        return [obs, obs, obs_color]

    @property
    def park_1(self):
        pw = 0.4    # Width of parking slot [m]
        pl = 0.6      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 1; nc = 0
        Gc_park = np.array([
            [pl/2, 0.0],
            [0.0, pw/2]
        ])
        Gb_park = np.array([
            [0.0],
            [0.6]
        ])
        c_park = np.array([ [2.2], [0.6] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)
        parking_color = (0.09804,  0.09804,  0.09804, 0.5)

        return parking, parking, parking_color
    
    @property
    def park_2(self):
        pw = 0.4    # Width of parking slot [m]
        pl = 0.6      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 1; nc = 0
        Gc_park = np.array([
            [pw/2, 0.0],
            [0.0, pl/2]
        ])
        Gb_park = np.array([
            [0.4],
            [0.0]
        ])
        c_park = np.array([ [-0.3], [0.3] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)
        parking_color = (0.09804,  0.09804,  0.09804, 0.5)

        return parking, parking, parking_color


class ParkEnv2:
    def __init__(self, road = 'full') -> None:
        self.zono_op = ZonoOperations()
        self.road_type = road

        # Dynamic Model
        self.A = np.array([
            [1.0, 0.0],
            [0.0, 1.0]
        ])

        self.B = np.array([
            [0.1, 0.0],
            [0.0, 0.1]
        ])        

        # Define the dynamics model in here (Create a separate class for this which this env can take)
        if road == 'full':
            self.x_min = -2.5; self.x_max = 2.5; self.y_min = -1.4; self.y_max = 1.4
            self.samples_x = int(50*self.x_max); self.samples_y = int(50*self.y_max); self.max_dist = 0.3
            self.x_step = (self.x_max - self.x_min) / (self.samples_x)
            self.y_step = (self.y_max - self.y_min) / (self.samples_y)
            self.max_dist_x = math.ceil(self.max_dist / self.x_step) # Maximum number of steps in the x direction
            self.max_dist_y = math.ceil(self.max_dist / self.y_step) # Maximum number of steps in the y direction
            self.max_dist_diag = math.ceil( 1.1 * self.max_dist_x)        
            self.p1_ = np.array([ [ 1.90,  0.6],[ 1.90,  0.6] ])
            self.p2_ = np.array([ [ 1.90, -0.2],[ 1.90, -0.6] ])
            self.p3_ = np.array([ [-0.90,  0.6],[-0.50,  0.6] ])
            self.p4_ = np.array([ [-0.10,  0.6],[ 0.30,  0.6] ])

            # Create a list of all (x, y) points between the two points in p1_, p2_, p3_, p4_
            self.initial_points = []
            for i in range(int(abs(self.p3_[1][0] - self.p3_[0][0])/self.x_step) + 1):    # These are the horizontal lines
                self.initial_points.append([self.p3_[0][0] + i*self.x_step, self.p3_[0][1]])
                self.initial_points.append([self.p4_[0][0] + i*self.x_step, self.p4_[0][1]])
            for i in range(int(abs(self.p1_[1][1] - self.p1_[0][1])/self.y_step) + 2):    # These are the vertical lines
                self.initial_points.append([self.p1_[0][0], self.p1_[0][1] - i*self.y_step])
                self.initial_points.append([self.p2_[0][0], self.p2_[0][1] - i*self.y_step])            

            self.initial_points = np.array(self.initial_points)
        elif road == 'outer':
            self.x_min = -2.5; self.x_max = 2.5; self.y_min = -1.4; self.y_max = 1.4                # Bounds
            self.x_step = self.B[0][0]; self.y_step = self.B[1][1]                                  # Step size
            self.samples_x = int( (self.x_max - self.x_min) / (self.x_step) )                       # Number of samples (x)
            self.samples_y = int( (self.y_max - self.y_min) / (self.y_step) )                       # Number of samples (y)
            self.max_dist_x = math.ceil(self.B[0][0] / self.x_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_y = math.ceil(self.B[1][1] / self.y_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_diag = math.ceil( math.sqrt(self.max_dist_x**2 + self.max_dist_y**2) )    # Maximum distance it can travel in one step (diagonal)
            self.p1_ = np.array([ [ 1.90,  0.2],[ 1.90,  0.6] ])                                    # Parking spot 1 vertices
            self.p2_ = np.array([ [ 1.90, -0.6],[ 1.90, -0.2] ])                                    # Parking spot 2 vertices

            # Create a list of all (x, y) points between the two points in p1_, p2_,
            self.initial_points = []
            for i in range(int(abs(self.p1_[1][1] - self.p1_[0][1])/self.y_step) + 2):    # These are the vertical lines
                self.initial_points.append([self.p1_[0][0], self.p1_[0][1] - i*self.y_step])
                self.initial_points.append([self.p2_[0][0], self.p2_[0][1] - i*self.y_step])            

            self.initial_points = np.array(self.initial_points)
        elif road == 'inner':
            self.x_min = -2.1; self.x_max = 1.5; self.y_min = -1.0; self.y_max = 1.0                # Bounds
            self.x_step = self.B[0][0]; self.y_step = self.B[1][1]                                  # Step size
            self.samples_x = int( (self.x_max - self.x_min) / (self.x_step) )                       # Number of samples (x)
            self.samples_y = int( (self.y_max - self.y_min) / (self.y_step) )                       # Number of samples (y)
            self.max_dist_x = math.ceil(self.B[0][0] / self.x_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_y = math.ceil(self.B[1][1] / self.y_step)                                 # Maximum distance it can travel in one step (x)
            self.max_dist_diag = math.ceil( math.sqrt(self.max_dist_x**2 + self.max_dist_y**2) )    # Maximum distance it can travel in one step (diagonal)
            self.p3_ = np.array([ [-0.9,  0.6],[-0.5,  0.6] ])                                      # Parking spot 3 vertices
            self.p4_ = np.array([ [-0.1,  0.6],[ 0.3,  0.6] ])                                      # Parking spot 4 vertices

            # Create a list of all (x, y) points between the two points in p1_, p2_, p3_, p4_
            self.initial_points = []
            for i in range(int(abs(self.p3_[1][0] - self.p3_[0][0])/self.x_step) + 1):    # These are the horizontal lines
                self.initial_points.append([self.p3_[0][0] + i*self.x_step, self.p3_[0][1]])
                self.initial_points.append([self.p4_[0][0] + i*self.x_step, self.p4_[0][1]])        

            self.initial_points = np.array(self.initial_points)            
        else:
            raise ValueError('Invalid road type. Please choose from "full", "outer", "inner"')          



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
        if self.road_type == 'full':    
            road = self.road_outer
            env['sets'].append(road[0]); env['vis'].append(road[1]); env['colors'].append(road[2])
            road = self.road_inner
            env['sets'].append(road[0]); env['vis'].append(road[1]); env['colors'].append(road[2])
            # Parking
            park = self.park_1
            env['sets'].append(park[0]); env['vis'].append(park[1]); env['colors'].append(park[2])
            park = self.park_2
            env['sets'].append(park[0]); env['vis'].append(park[1]); env['colors'].append(park[2])  
        elif self.road_type == 'outer':
            road = self.road_outer
            env['sets'].append(road[0]); env['vis'].append(road[1]); env['colors'].append(road[2])
            # Parking
            park = self.park_1
            env['sets'].append(park[0]); env['vis'].append(park[1]); env['colors'].append(park[2])
        elif self.road_type == 'inner':
            road = self.road_inner
            env['sets'].append(road[0]); env['vis'].append(road[1]); env['colors'].append(road[2])
            # Parking
            park = self.park_2
            env['sets'].append(park[0]); env['vis'].append(park[1]); env['colors'].append(park[2])              
      
        # Obstacles
        obs = self.obstacle_1
        env['sets'].append(obs[0])#; env['vis'].append(obs[1]); env['colors'].append(obs[2])
        obs = self.obstacle_2
        env['sets'].append(obs[0])#; env['vis'].append(obs[1]); env['colors'].append(obs[2])
        obs = self.obstacle_3
        env['sets'].append(obs[0])#; env['vis'].append(obs[1]); env['colors'].append(obs[2])

        return env['sets'], env['vis'], env['colors']

    def get_extra_visuals(self):
        '''
        This class returns all the extra visuals that are not part of the sets
        # TODO: Add the white polygon lines in here instead of defining them as hybrid zonotopes
        '''
        pass

    @property
    def road_outer(self):
        nx = 2          # Number of state variables
        ni = 2          # Number of input variables
        lw = 0.4        # Width of one road lane [m]
        ll_12 = 4.40    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_34 = 2.00    # Length of road segments 3 and 4 [m]   (Vertical roads)
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
            [1.2],
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
            [2.0],
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
        lw = 0.4        # Width of one road lane [m]
        ll_12 = 3.60    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_34 = 1.20    # Length of road segments 3 and 4 [m]   (Vertical roads)
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
            [0.8],
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
            [1.6],
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
    def park_1(self):
        pw = 0.4    # Width of parking slot [m]
        pl = 0.6      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 1; nc = 0
        Gc_park = np.array([
            [pl/2, 0.0],
            [0.0, pw/2]
        ])
        Gb_park = np.array([
            [0.0],
            [0.4]
        ])
        c_park = np.array([ [2.2], [0.0] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)
        parking_color = (0.4,  0.4,  0.4, 1.0)  

        return parking, parking, parking_color
    
    @property
    def park_2(self):
        pw = 0.4    # Width of parking slot [m]
        pl = 0.6      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 1; nc = 0
        Gc_park = np.array([
            [pw/2, 0.0],
            [0.0, pl/2]
        ])
        Gb_park = np.array([
            [0.4],
            [0.0]
        ])
        c_park = np.array([ [-0.3], [0.3] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)
        parking_color = (0.000001,  0.000001,  0.000001, 0.6)  

        return parking, parking, parking_color

    @property
    def obstacle_1(self):
        '''
        Internal obstacle
        '''
        nx = 2  # Number of state variables
        ng = 2; nc = 0; nb = 0
        # Obstacle 1
        Gc_obs = np.array([
            [0.6, 0.0],
            [0.0, 0.3]
        ])
        Gb_obs = np.zeros((nx, nb))
        c_obs = np.array([ [-0.3], [-0.3] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_color = (0.949, 0.262, 0.227, 1.0)  # Red
        
        return [obs, obs, obs_color]

    @property
    def obstacle_2(self):
        '''
        Internal obstacle
        '''
        ng = 2; nc = 0; nb = 1
        # Obstacle 2
        Gc_obs = np.array([
            [0.4, 0.0],
            [0.0, 0.6]
        ])
        Gb_obs = np.array([
            [1.0],
            [0.0]
        ])
        c_obs = np.array([ [-0.3], [0.0] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_color = (0.949, 0.262, 0.227, 1.0)  # Red
        
        return [obs, obs, obs_color]

    @property
    def obstacle_3(self):
        '''
        Internal obstacle
        '''
        nx = 2; ng = 2; nc = 0; nb = 0
        # Obstacle 2
        Gc_obs = np.array([
            [0.2, 0.0],
            [0.0, 0.3]
        ])
        Gb_obs = np.zeros((nx, nb))
        c_obs = np.array([ [-0.3], [0.3] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_color = (0.949, 0.262, 0.227, 1.0)  # Red
        
        return [obs, obs, obs_color]


class SamplesVis:
    '''
    This class contains some shapes that are used in this project
    just for visualization purposes
    '''
    @property
    def road_line(self):
        # Define a polygon
        vertices = np.array([
            [-2.1,  1.0],
            [ 1.5,  1.0],
            [ 1.5, -1.0],
            [-2.1, -1.0]
        ])

        # Create a line between the first two vertices
        line = Polygon(vertices, closed = True, fill = False, color = 'white', linestyle = '--', linewidth = 4)
        
        return line


class SamplesZ:
    def __init__(self) -> None:
        pass

    @property
    def set_1(self):
        c = np.array([  [0],
                        [0]
                    ])
        G = np.array([  [1, 0],
                        [0, 1]
                    ]) 

        return Zonotope(c, G)

    @property
    def set_2(self):
        c = np.array([  [0],
                        [0]
                    ])
        G = np.array([  [1, 0],
                        [1, 1]
                    ])

        return Zonotope(c, G)      


class SamplesCZ:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()

    @property
    def set_learning(self):
        G = np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        C = np.array([
            [0.0],
            [0.0]
        ])
        A = np.array([
            [1.0, 1.0, 1.0]
        ])
        b = np.array([
            [-1.0]
        ])        

        return ConstrainedZonotope(G, C, A, b)

    @property
    def set_1(self):
        G = np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        C = np.array([
            [0.0],
            [0.0]
        ])
        A = np.array([
            [1.0, 1.0, 1.0]
        ])
        b = np.array([
            [-1.0]
        ])        

        return ConstrainedZonotope(G, C, A, b)
    
    @property
    def set_2(self):
        G = np.array([
            [1.0, 0.0, 0.0, 0.1],
            [0.0, 1.0, 0.0, 0.8]
        ])        
        C = np.array([
            [0.0],
            [0.0]
        ])
        A = np.array([
            [-1.0, 1.0, 0.3, 1.0]
        ])
        b = np.array([
            [1.0]
        ])

        return ConstrainedZonotope(G, C, A, b)        


class SamplesHZ:
    def __init__(self) -> None:

        self.zono_op = ZonoOperations()

    @property
    def road_line(self):
        nx = 2      # Number of state variables
        ni = 2      # Number of input variables
        lw = 0.05    # Width of one road line [m]
        ll_12 = 11.12  # Length of road segments 1 and 2 [m]
        ll_34 = 11.88   # Length of road segments 3 and 4 [m]
        color = (1.0, 1.0, 1.0 , 1.0)   # White

        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [0.0], [0.0], [0.0], [0.0] ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_12 = np.array([
            [ll_12/2, 0.0, 0.0, 0.0],
            [  0.0  , lw, 0.0, 0.0],
            [  0.0  , 0.0, 1.0, 0.0],
            [  0.0  , 0.0, 0.0, 1.0]
        ])
        Gb_road_12 = np.array([
            [0.0],
            [6.0],
            [0.0],
            [0.0]
        ])
        Gc_road_34 = np.array([
            [  lw ,   0.0  , 0.0, 0.0],
            [  0.0, ll_34/2, 0.0, 0.0],
            [  0.0,   0.0  , 1.0, 0.0],
            [  0.0,   0.0  , 0.0, 1.0]
        ])
        Gb_road_34 = np.array([
            [5.5],
            [0.0],
            [0.0],
            [0.0]
        ])
        road_12 = HybridZonotope(Gc_road_12, Gb_road_12, c_road, Ac_road, Ab_road, b_road)
        road_34 = HybridZonotope(Gc_road_34, Gb_road_34, c_road, Ac_road, Ab_road, b_road)

        road = self.zono_op.union_hz_hz(road_12, road_34)

        road_vis = HybridZonotope(road.Gc[0:2, :], road.Gb[0:2, :], road.C[0:2, :], road.Ac, road.Ab, road.b)

        return road, road_vis, color


    @property
    def road(self):
        nx = 2          # Number of state variables
        ni = 2          # Number of input variables
        lw = 0.4        # Width of one road lane [m]
        ll_12 = 4.40    # Length of road segments 1 and 2 [m] (Horizontal roads)
        ll_34 = 1.20    # Length of road segments 3 and 4 [m]   (Vertical roads)
        color = (0.09804,  0.09804,  0.09804, 0.5)  # Gray

        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [-0.3], [0.0], [0.0], [0.0] ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_12 = np.array([
            [ll_12/2, 0.0, 0.0, 0.0],
            [  0.0  , lw, 0.0, 0.0],
            [  0.0  , 0.0, 1.0, 0.0],
            [  0.0  , 0.0, 0.0, 1.0]
        ])
        Gb_road_12 = np.array([
            [0.0],
            [1.0],
            [0.0],
            [0.0]
        ])
        Gc_road_34 = np.array([
            [  lw , 0.0, 0.0, 0.0],
            [  0.0, ll_34/2 , 0.0, 0.0],
            [  0.0, 0.0, 1.0, 0.0],
            [  0.0, 0.0, 0.0, 1.0]
        ])
        Gb_road_34 = np.array([
            [1.8],
            [0.0],
            [0.0],
            [0.0]
        ])
        road_12 = HybridZonotope(Gc_road_12, Gb_road_12, c_road, Ac_road, Ab_road, b_road)
        road_34 = HybridZonotope(Gc_road_34, Gb_road_34, c_road, Ac_road, Ab_road, b_road)

        road = self.zono_op.union_hz_hz(road_12, road_34)

        road_vis = HybridZonotope(road.Gc[0:2, :], road.Gb[0:2, :], road.C[0:2, :], road.Ac, road.Ab, road.b)

        return road, road_vis, color

    @property
    def obstacle_1(self):
        '''
        Internal obstacle
        '''
        nx = 2  # Number of state variables
        ng = 2; nc = 0; nb = 0
        # Obstacle 1
        Gc_obs = np.array([
            [0.6, 0.0],
            [0.0, 0.3]
        ])
        Gb_obs = np.zeros((nx, nb))
        c_obs = np.array([ [-0.3], [-0.3] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_color = (0.949, 0.262, 0.227, 1.0)  # Red
        
        return obs, obs_color

    @property
    def obstacle_2(self):
        '''
        Internal obstacle
        '''
        ng = 2; nc = 0; nb = 1
        # Obstacle 2
        Gc_obs = np.array([
            [0.4, 0.0],
            [0.0, 0.6]
        ])
        Gb_obs = np.array([
            [1.0],
            [0.0]
        ])
        c_obs = np.array([ [-0.3], [0.0] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_color = (0.949, 0.262, 0.227, 1.0)  # Red
        
        return obs, obs_color    

    @property
    def obstacle_3(self):
        '''
        Internal obstacle
        '''
        nx = 2; ng = 2; nc = 0; nb = 0
        # Obstacle 2
        Gc_obs = np.array([
            [0.2, 0.0],
            [0.0, 0.3]
        ])
        Gb_obs = np.zeros((nx, nb))
        c_obs = np.array([ [-0.3], [0.3] ])
        Ac_obs = np.zeros((nc, ng))
        Ab_obs = np.zeros((nc, nb))
        b_obs = np.zeros((nc, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)
        obs_color = (0.949, 0.262, 0.227, 1.0)  # Red
        
        return obs, obs_color

    @property
    def obstacles(self):
        '''
        Returns a list of all obstacles
        '''
        obs_1, obs_color_1 = self.obstacle_1
        obs_2, obs_color_2 = self.obstacle_2
        obs_3, obs_color_3 = self.obstacle_3
        obs_list = [obs_1, obs_2, obs_3]
        obs_color_list = [obs_color_1, obs_color_2, obs_color_3]
        
        return obs_list, obs_color_list

    @property
    def park_1(self):
        pw = 0.4    # Width of parking slot [m]
        pl = 0.6      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 1; nc = 0
        Gc_park = np.array([
            [pl/2, 0.0],
            [0.0, pw/2]
        ])
        Gb_park = np.array([
            [0.0],
            [0.6]
        ])
        c_park = np.array([ [2.2], [0.6] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)
        parking_color = (0.09804,  0.09804,  0.09804, 0.5)

        return parking, parking_color
    
    @property
    def park_2(self):
        pw = 0.4    # Width of parking slot [m]
        pl = 0.6      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 1; nc = 0
        Gc_park = np.array([
            [pw/2, 0.0],
            [0.0, pl/2]
        ])
        Gb_park = np.array([
            [0.4],
            [0.0]
        ])
        c_park = np.array([ [-0.3], [0.3] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)
        parking_color = (0.423, 0.556, 0.749, 1.0)

        return parking, parking_color

    @property
    def parkings(self):
        '''
        Returns a list of all parking spots
        '''
        park_1, park_color_1 = self.park_1
        park_2, park_color_2 = self.park_2
        park_list = [park_1, park_2]
        park_color_list = [park_color_1, park_color_2]

        return park_list, park_color_list



    @property
    def set_a(self):
        n = 2; ng = 3; nc = 0; nb = 0
        Gc = 0.1*np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)
    
    @property
    def set_b(self):
        n = 2; ng = 3; nc = 1; nb = 0
        Gc = 0.07*np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = np.zeros((n, nb))
        C = 0.15*np.array([ [-1.2], [0.2] ])

        Ac = np.array([
            [1.0, 1.0, 1.0],
        ])

        Ab = np.zeros((nc, nb))

        b = np.array([ 
            [1.0] 
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)    


    @property
    def set_c(self):
        n = 2; ng = 4; nc = 3; nb = 0
        Gc = np.array([
            [1.5, -1.5, 0.5, 0.9],
            [1.0, 0.5, -1.0, 1.0]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])

        Ac = np.array([
            [0.8, 0.8, 0.8, 0.8],
            [5.0, 5.0, 5.0, 5.0],
            [5.0, 5.0, 5.0, 5.0]
        ])

        Ab = np.zeros((nc, nb))

        b = np.array([ 
            [0.8],
            [5.0],
            [5.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)  


    @property
    def set_d(self):
        n = 2; ng = 4; nc = 0; nb = 0
        Gc = np.array([
            [1.0, 0.0, 1.0, 0.5],
            [0.0, 1.0, 0.0, 0.5]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))


        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)       


    @property
    def set_e(self):
        n = 2; ng = 4; nc = 1; nb = 0
        Gc = np.array([
            [1.0, 0.5, 1.0, 1.6],
            [0.0, 1.0, 2.0, 2.1]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            # [1.0, 0.5, 0.1, 0.2]
            [1.0, 1.0, 1.0, 1.0]
        ])
        Ab = np.zeros((nc, nb))
        b = np.array([
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)
    
    @property
    def set_f(self):
        n = 2; ng = 7; nc = 1; nb = 0
        Gc = np.array([
            [1.0, 0.5, 1.0, 1.6, 0.4, 0.0, 0.5 ],
            [0.0, 1.0, 2.0, 2.1, 0.3, 0.1, 0.3 ]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
        ])
        Ab = np.zeros((nc, nb))
        b = np.array([
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)
    
    @property
    def set_g(self):
        n = 2; ng = 4; nc = 4; nb = 0
        Gc = 0.1*np.array([
            [1.5, 1.0, 1.6, 1.6],
            [0.0, 0.8, 1.2, 1.2]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            [1.2, 1.0, 0.3, 0.1],
            [2.4, 2.0, 0.6, 0.2],
            [1.2, 1.0, 0.1, 0.1],
            [2.4, 2.0, 0.4, 0.2]
        ])
        Ab = np.zeros((nc, nb))
        b = np.array([
            [0.5],
            [1.0],
            [0.5],
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)    
    
    @property
    def set_1(self):
        ng = 3; nc = 0; nb = 3
        Gc = np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = 2*Gc
        C = np.array([ [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    @property
    def set_2(self):
        ng = 3; nc = 1; nb = 3
        Gc = np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = 2*Gc
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            [1.0, 1.0, 1.0]
        ])
        Ab = np.zeros((nc, nb))
        b = np.array([
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)


    @property
    def set_learning(self):
        n = 6; ng = 4; nc = 4; nb = 0
        Gc = np.array([
            [2.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 2.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 2.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 2.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 2.0]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([  [0.0], 
                        [0.0],
                        [0.0],
                        [0.0],
                        [0.0],
                        [0.0] ])
        # Ac = np.zeros((nc, ng))
        Ac = np.array([
            [ 1.0,  0.0,  1.0,  0.0, 0.0,  0.0],
            [-1.0,  0.0,  0.0,  1.0, 0.0,  0.0],
            [ 0.0,  1.0,  0.0,  0.0, 1.0,  0.0],
            [ 0.0, -1.0,  0.0,  0.0, 0.0,  1.0]
        ])
        Ab = np.zeros((nc, nb))  
        # b = np.zeros((nc, 1))
        b = np.array([
            [ 0.1],
            [ 0.7],
            [ 0.7],
            [ 0.1]
        ])

        hz = HybridZonotope(Gc, Gb, C, Ac, Ab, b)

        # Return only first two dimensions
        return HybridZonotope(hz.Gc[0:2, :], hz.Gb[0:2, :], hz.C[0:2, :], hz.Ac, hz.Ab, hz.b)

    @property
    def set_3(self):
        ng = 3; nc = 1; nb = 3
        Gc = np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = 2*Gc
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            [1.0, 1.0, 1.0]
        ])
        Ab = Ac
        b = np.array([
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)
   
    @property
    def set_4(self):
        ng = 3; nc = 1; nb = 3
        # Gc = np.array([
        #     [1.5, -1.5, 0.5],
        #     [1.0, 0.5, -1.0]
        # ])
        Gc = np.array([
            [2.0, 0.0, 0.5],
            [0.0, 4.0, -1.0]
        ])        
        Gb = np.array([
            [3.0, -3.0,  1.0],
            [2.0,  1.0, -2.0]
        ])
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            [1.0, 1.0, 1.0]
        ])
        Ab = np.array([
            [0.0, 0.0, 1.0]
        ])
        b = np.array([
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)



    @property
    def set_5(self):
        n = 2; ng = 3; nc = 0; nb = 0
        Gc = 1.4*np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    @property
    def set_6(self):
        n = 2; ng = 3; nc = 1; nb = 0
        Gc = 1.4*np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            [1.0, 2.0, 2.0]
        ])
        Ab = np.zeros((nc, nb))
        b = np.array([
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    @property
    def set_7(self):
        n = 2; ng = 3; nc = 1; nb = 2
        Gc = 1.4*np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = np.array([
            [3.5, -3.5],
            [2.5, 2.5]
        ])
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            [1.0, 2.0, 2.0]
        ])
        Ab = np.zeros((nc, nb))
        b = np.array([
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)



    @property
    def space_old(self):
        '''
        Returns a Hybrid zonotope that represents the position dimensions of entire state space
        '''
        nx = 2
        ni = 0
        nc = 0
        nb = 0
        x_dim = 16
        y_dim = 18
        
        Gc = np.array([
            [x_dim/2, 0.0],
            [0.0, y_dim/2]
        ])
        Gb = np.zeros((nx, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.zeros((nc, nx))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))
        
        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    @property
    def roads_old(self):
        nx = 2      # Number of state variables
        ni = 2      # Number of input variables
        lw = 3.0    # Width of one road lane [m]
        ll_12 = 16  # Length of road segments 1 and 2 [m]
        ll_34 = 5   # Length of road segments 3 and 4 [m]

        ng = 4; nc = 0; nb = 1
        c_road = np.array([ [0.0], [0.0], [0.0], [0.0] ])
        Ac_road = np.zeros((nc, ng))
        b_road = np.zeros((nc, 1))
        Ab_road = np.zeros((nc, nb))
        Gc_road_12 = np.array([
            [ll_12/2, 0.0, 0.0, 0.0],
            [  0.0  , lw, 0.0, 0.0],
            [  0.0  , 0.0, 1.0, 0.0],
            [  0.0  , 0.0, 0.0, 1.0]
        ])
        Gb_road_12 = np.array([
            [0.0],
            [6.0],
            [0.0],
            [0.0]
        ])
        Gc_road_34 = np.array([
            [ll_34/2, 0.0, 0.0, 0.0],
            [  0.0  , lw , 0.0, 0.0],
            [  0.0  , 0.0, 1.0, 0.0],
            [  0.0  , 0.0, 0.0, 1.0]
        ])
        Gb_road_34 = np.array([
            [5.5],
            [0.0],
            [0.0],
            [0.0]
        ])
        road_12 = HybridZonotope(Gc_road_12, Gb_road_12, c_road, Ac_road, Ab_road, b_road)
        road_34 = HybridZonotope(Gc_road_34, Gb_road_34, c_road, Ac_road, Ab_road, b_road)

        road = self.zono_op.union_hz_hz(road_12, road_34)

        road_vis = HybridZonotope(road.Gc[0:2, :], road.Gb[0:2, :], road.C[0:2, :], road.Ac, road.Ab, road.b)

        return road, road_vis

    @property
    def obstacles_old(self):
        nx = 2  # Number of state variables
        
        # Obstacle
        Gc_obs = np.array([
            [3.0, 0.0],
            [0.0, 3.0]
        ])
        Gb_obs = np.zeros((nx, 0))
        c_obs = np.array([ [0.0], [0.0] ])
        Ac_obs = np.zeros((0, nx))
        Ab_obs = np.zeros((0, 0))
        b_obs = np.zeros((0, 1))

        obs = HybridZonotope(Gc_obs, Gb_obs, c_obs, Ac_obs, Ab_obs, b_obs)

        return obs
    
    @property
    def park_old_1(self):
        pw = 2.5    # Width of parking slot [m]
        pl = 3      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 1; nc = 0
        Gc_park = np.array([
            [pl/2, 0.0],
            [0.0, pw/2]
        ])
        Gb_park = np.array([
            [0.0],
            [7.7]
        ])
        c_park = np.array([ [9.5], [0.0] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)

        return parking
    
    @property
    def park_old_2(self):
        pw = 2.5    # Width of parking slot [m]
        pl = 3      # Length of parking slot [m] (This is not a realistic length, but it is used to make the plot look nicer)
        ng = 2; nb = 1; nc = 0
        Gc_park = np.array([
            [pl/2, 0.0],
            [0.0, pw/2]
        ])
        Gb_park = np.array([
            [0.0],
            [-7.7]
        ])
        c_park = np.array([ [-9.5], [0.0] ])
        Ac_park = np.zeros((nc, ng))
        Ab_park = np.zeros((nc, nb))
        b_park = np.zeros((nc, 1))        

        parking = HybridZonotope(Gc_park, Gb_park, c_park, Ac_park, Ab_park, b_park)

        return parking    
    
