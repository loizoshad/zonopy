'''
This file contains a collection of example sets represented as Hybrid Zonotopes
'''

import numpy as np

from utils.sets.zonotopes import Zonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations

from matplotlib.patches import Polygon


class SamplesCZ:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()

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



class SamplesVis:
    '''
    This class contains some shapes that are used in this project
    just for visualization purposes
    '''
    @property
    def road_line(self):
        # Define a polygon
        vertices = np.array([
            [-5.52, 6.0],
            [5.5, 6.0],
            [5.5, -6.0],
            [-5.52, -6.0]
        ])

        # Create a line between the first two vertices
        line = Polygon(vertices, closed = True, fill = False, color = 'white', linestyle = '--', linewidth = 4)
        
        return line

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
        nx = 2      # Number of state variables
        ni = 2      # Number of input variables
        lw = 3.0    # Width of one road lane [m]
        ll_12 = 16  # Length of road segments 1 and 2 [m]
        ll_34 = 5   # Length of road segments 3 and 4 [m]
        color = (0.09804,  0.09804,  0.09804, 0.5)  # Gray

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

        return road, road_vis, color

    @property
    def obstacles(self):
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
    def park_1(self):
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
    def park_2(self):
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
    
