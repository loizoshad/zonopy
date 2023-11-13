import numpy as np
import matplotlib.image as mpimg
import matplotlib.patches as patches
import math
import time

from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations


import matplotlib.pyplot as plt

# step_size = 0.1
# step_size = 0.05
step_size = 0.025
# step_size = 0.02

class Environment:
    def __init__(self, vis, N) -> None:
        # Step 0: Initialize parameters and objects
        self.vis = vis
        self.zono_op = ZonoOperations()
        self.step_size = step_size
        self.init_vis_space()    # Initialize visualization space

        # self.dynamics = DynamicsModel2d() 
        # self.state_space = self.state_space_2d()
        # self.target = self.parking_slots_2d()

        self.dynamics = DynamicsModel4d() 
        self.state_space = self.state_space_4d()
        self.target = self.parking_slots_4d()


    def parking_slots_2d(self):
        # # Parking 1 (Taxi)
        # Gb = np.zeros((2, 0))
        # Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # Gc = np.diag(np.array([ 0.1, 0.05]))
        # c = np.array([ [-0.70], [0.60]])
        # park_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)    

        # # Parking 2
        # Gc = np.diag(np.array([ 0.1, 0.05]))
        # c = np.array([ [0.70], [0.60]])
        # Gb = np.zeros((2, 0))
        # Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # park_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Parking 3
        # Gc = np.diag(np.array([ 0.1, 0.05]))
        # c = np.array([ [1.1], [0.60]])
        # Gb = np.zeros((2, 0))
        # Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # park_3 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Parking 4
        # Gc = np.diag(np.array([ 0.1, 0.05]))
        # c = np.array([ [-0.2], [-0.3]])
        # Gb = np.zeros((2, 0))
        # Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # park_4 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Parking 5
        # Gc = np.diag(np.array([ 0.1, 0.05]))
        # c = np.array([ [0.2], [-0.6]])
        # Gb = np.zeros((2, 0))
        # Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # park_5 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)        
        
        # return park_1

        # Parking 1, 3
        Gc = np.diag(np.array([ 0.1, 0.05]))
        Gb = np.array([ [-0.90], [0.0]])
        c = np.array([ [0.2], [0.60]])
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        park_13 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Parking 4, 5
        Gc = np.diag(np.array([ 0.1, 0.05]))
        Gb = np.array([ [-0.2], [0.15]])
        c = np.array([ [0.0], [-0.45]])
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        park_45 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        parking = self.zono_op.union_hz_hz_v2(park_13, park_45)
        parking = self.zono_op.redundant_c_gc_hz_v2(parking)
        parking = self.zono_op.redundant_c_gc_hz_v1(parking)

        print(f'target: ng = {parking.ng}, nc = {parking.nc}, nb = {parking.nb}')

        return parking

    def state_space_2d(self):
        lw = 0.1
        lh_v = 1.9         # Lane height of vertical lanes [m]
        lh_h = 2.8         # Lane height of horizontal lanes [m]

         # Vertical Road Sections (Left)
        Gc = np.diag(np.array([ lw/2  , lh_v/2, 0.0, 0.5 ]))
        Gb = np.array([ [0.45], [0.0], [0.0], [-0.5] ])
        c = np.array([ [-0.9], [0.0], [ 0.0], [ 0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_v_left = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections (Right)
        Gc = np.diag(np.array([ lw/2  , lh_v/2, 0.0, 0.5 ]))
        Gb = np.array([ [0.45], [0.0], [0.0], [-0.5] ])
        c = np.array([ [0.9], [0.0], [ 0.0], [ 0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_v_right = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Horizontal Road Sections (Exterior)
        Gc = np.diag(np.array([ lh_h/2  , lw/2, 0.5, 0.0 ]))
        Gb = np.array([ [0.0], [0.9], [0.5], [0.0] ])
        c = np.array([ [0.0], [0.0], [ 0.0], [ 0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_h_ext = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Horizontal Road Sections (Middle)
        Gc = np.diag(np.array([ lh_h/2 + 0.2  , lw/2, 0.5, 0.0 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.2], [0.0], [ 0.5], [ 0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_h_mid = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_v = self.zono_op.union_hz_hz_v2(road_v_left, road_v_right)
        road_v = self.zono_op.redundant_c_gc_hz_v2(road_v)
        road_v = self.zono_op.redundant_c_gc_hz_v1(road_v)

        road_h = self.zono_op.union_hz_hz_v2(road_h_ext, road_h_mid)
        road_h = self.zono_op.redundant_c_gc_hz_v2(road_h)
        road_h = self.zono_op.redundant_c_gc_hz_v1(road_h)

        state_space = self.zono_op.union_hz_hz_v2(road_v, road_h)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        # # Target 1 (Taxi)
        # Gc = np.diag(np.array([ 0.20, 0.05, 1.0, 1.0]))
        # c = np.array([ [-0.65], [0.60], [0.0], [0.0]])
        # Gb = np.zeros((4, 0))
        # Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # parking_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # state_space = self.zono_op.union_hz_hz_v2(state_space, parking_1)
        # state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        # state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        print(f'state space: ng = {state_space.ng}, nc = {state_space.nc}, nb = {state_space.nb}')

        return state_space
    


    def parking_slots_4d(self):
        # # Parking 1 (Taxi)
        # Gb = np.zeros((4, 0))
        # Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # Gc = np.diag(np.array([ 0.3, 0.05, 0.0, 0.0]))
        # c = np.array([ [-0.70], [0.60], [0.0], [0.0]])
        # park_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)    

        # # Parking 2
        # Gc = np.diag(np.array([ 0.1, 0.05]))
        # c = np.array([ [0.70], [0.60]])
        # Gb = np.zeros((2, 0))
        # Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # park_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Parking 3
        # Gc = np.diag(np.array([ 0.1, 0.05]))
        # c = np.array([ [1.1], [0.60]])
        # Gb = np.zeros((2, 0))
        # Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # park_3 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Parking 4
        # Gc = np.diag(np.array([ 0.1, 0.05]))
        # c = np.array([ [-0.2], [-0.3]])
        # Gb = np.zeros((2, 0))
        # Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # park_4 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Parking 5
        # Gc = np.diag(np.array([ 0.1, 0.05]))
        # c = np.array([ [0.2], [-0.6]])
        # Gb = np.zeros((2, 0))
        # Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        # park_5 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)        
        
        # return park_1

        # Parking 1, 3
        Gc = np.diag(np.array([ 0.3, 0.05, 0.0, 0.0]))
        Gb = np.array([ [-0.90], [0.0], [0.0], [0.0]])
        c = np.array([ [0.2], [0.60], [0.0], [0.0]])
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        park_13 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Parking 4, 5
        Gc = np.diag(np.array([ 0.3, 0.05, 0.0, 0.0]))
        Gb = np.array([ [-0.2], [0.15], [0.0], [0.0]])
        c = np.array([ [0.0], [-0.45], [0.0], [0.0]])
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        park_45 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        parking = self.zono_op.union_hz_hz_v2(park_13, park_45)
        parking = self.zono_op.redundant_c_gc_hz_v2(parking)
        parking = self.zono_op.redundant_c_gc_hz_v1(parking)

        print(f'target: ng = {parking.ng}, nc = {parking.nc}, nb = {parking.nb}')

        return parking

    def state_space_4d(self):
        lw = 0.1
        lh_v = 1.9         # Lane height of vertical lanes [m]
        lh_h = 2.8         # Lane height of horizontal lanes [m]

         # Vertical Road Sections (Left)
        Gc = np.diag(np.array([ lw/2  , lh_v/2, 0.0, 0.5, 1.0, 1.0 ]))
        Gb = np.array([ [0.45], [0.0], [0.0], [-0.5], [0.0], [0.0] ])
        c = np.array([ [-0.9], [0.0], [ 0.0], [ 0.0], [0.0], [0.0] ])
        Ac = np.zeros((0, 6))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_v_left = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections (Right)
        Gc = np.diag(np.array([ lw/2  , lh_v/2, 0.0, 0.5, 1.0, 1.0 ]))
        Gb = np.array([ [0.45], [0.0], [0.0], [-0.5], [0.0], [0.0] ])
        c = np.array([ [0.9], [0.0], [ 0.0], [ 0.0], [0.0], [0.0] ])
        Ac = np.zeros((0, 6))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_v_right = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Horizontal Road Sections (Exterior)
        Gc = np.diag(np.array([ lh_h/2  , lw/2, 0.5, 0.0, 1.0, 1.0 ]))
        Gb = np.array([ [0.0], [0.9], [0.5], [0.0], [0.0], [0.0] ])
        c = np.array([ [0.0], [0.0], [ 0.0], [ 0.0], [0.0], [0.0] ])
        Ac = np.zeros((0, 6))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_h_ext = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Horizontal Road Sections (Middle)
        Gc = np.diag(np.array([ lh_h/2 + 0.2  , lw/2, 0.5, 0.0, 1.0, 1.0 ]))
        Gb = np.zeros((6, 0))
        c = np.array([ [0.2], [0.0], [ 0.5], [ 0.0], [0.0], [0.0] ])
        Ac = np.zeros((0, 6))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_h_mid = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_v = self.zono_op.union_hz_hz_v2(road_v_left, road_v_right)
        road_v = self.zono_op.redundant_c_gc_hz_v2(road_v)
        road_v = self.zono_op.redundant_c_gc_hz_v1(road_v)

        road_h = self.zono_op.union_hz_hz_v2(road_h_ext, road_h_mid)
        road_h = self.zono_op.redundant_c_gc_hz_v2(road_h)
        road_h = self.zono_op.redundant_c_gc_hz_v1(road_h)

        state_space = self.zono_op.union_hz_hz_v2(road_v, road_h)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        print(f'state space: ng = {state_space.ng}, nc = {state_space.nc}, nb = {state_space.nb}')

        return state_space





    def compute_brs(self, N = 50):
        '''
        Computes the BRS from each target area starting from the state space
        '''
        D = np.block([self.dynamics.A, self.dynamics.B])
        brs = self.target
        X = self.state_space

        print(f'starting time')
        start_time = time.perf_counter()
        for n in range(1, N + 1):
            brs = self.zono_op.one_step_brs_hz(X = X, T = brs, D = D)
        
        end_time = time.perf_counter()
        print(f'n = {n}, brs: ng = {brs.ng}, nc = {brs.nc}, nb = {brs.nb}')
        print(f'total time = {end_time - start_time}')

        # # Visualization
        # brs_vis = HybridZonotope(brs.Gc[0:2, :], brs.Gb[0:2, :], brs.C[0:2, :], brs.Ac, brs.Ab, brs.b)
        # self.vis_brs(brs_vis)
        # plt.show()
        # name = f'brs_{n}'
        # self.vis.fig.savefig(f'./results/static_final_paper/{name}.png', dpi=300)


        return brs

   

    def init_vis_space(self):
        '''
        Initializes the visualization space
        '''
        # step_size = 0.2
        # space_grid 
        self.vis_space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': [],         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
            'direction': []
        }

        # Horizontal top lane
        y = 0.9
        for x in np.arange(-1.4 + step_size/2, 1.5 - step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('horizontal')
        # Horizontal middle lane
        y = 0.0
        for x in np.arange(-1.4 + step_size/2, 1.4 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('horizontal')
        # Horizontal bottom lane left
        y = -0.9
        for x in np.arange(-1.4 + step_size/2, 1.4 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('horizontal')

        # Vertical far left lane
        x = -1.35
        for y in np.arange(0.05 + step_size/2, 0.85 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        for y in np.arange(-0.85 + step_size/2, -0.05 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        # Vertical left lane
        x = -0.45
        for y in np.arange(0.05 + step_size/2, 0.85 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        for y in np.arange(-0.85 + step_size/2, -0.05 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        # Vertical right lane
        x = 0.45
        for y in np.arange(0.05 + step_size/2, 0.85 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        for y in np.arange(-0.85 + step_size/2, -0.05 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        # Vertical far right lane
        x = 1.35
        for y in np.arange(0.05 + step_size/2, 0.85 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        for y in np.arange(-0.85 + step_size/2, -0.05 + step_size/2, step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')    

        # print(f'Number of points in vis_space = {len(self.vis_space["points"])}')

    def vis_env(self):
        '''
        Visualizes the plain environment
        '''
        # Visualize background
        img = mpimg.imread('./images/park_env_static.png')
        # self.vis.ax.imshow(img, extent=[-1.05, 1.05, -0.6, 0.6], zorder = 1)
        self.vis.ax.imshow(img, extent=[-1.5, 1.8, -1.05, 1.05], zorder = 1)

    def vis_hz(self, hz):
        '''
        This version of hybrid zonotope visualization is particularly useful when you already have information about the geometry of the hz
        '''            
        space = self.vis_space['points']
        flags = self.vis_space['flags']
        dir = self.vis_space['direction']

        length = len(self.vis_space["points"])
        start_time_total = time.perf_counter()
        # Loop through all points in the space
        for i, point in enumerate(space):
            start_time = time.perf_counter()
            p = np.array([ [point[0]], [point[1]] ])
            if self.zono_op.is_inside_hz(hz, p):
                flags[i] = 1

                if dir[i] == 'horizontal':
                    w = step_size
                    h = 0.1
                    rect = patches.Rectangle((p[0] - w/2, p[1] - h/2), w, h, linewidth = 0.1, edgecolor = '#6C8EBF', facecolor = '#6C8EBF', alpha = 0.4, zorder = 11)
                elif dir[i] == 'vertical':
                    w = 0.1
                    h = step_size
                    rect = patches.Rectangle((p[0] - w/2, p[1] - h/2), w, h, linewidth = 0.1, edgecolor = '#6C8EBF', facecolor = '#6C8EBF', alpha = 0.4, zorder = 11)

                self.vis.ax.add_patch(rect)
            end_time = time.perf_counter()
            print(f'Point {i} / {length}\t Vis time = {end_time - start_time}')

        end_time_total = time.perf_counter()
        print(f'Total visualization time = {end_time_total - start_time_total}')
        # Update flags
        self.vis_space['flags'] = flags

    def vis_brs(self, brs):
        '''
        Visualizes both the environment and the safe space given as an input
        '''
        self.vis_env()
        self.vis_hz(brs)


class DynamicsModel2d:
    '''
    This class defines a discrete-time linear dynamics model without disturbances.

    x_{k+1} = Ax_k + Bu_k
    
    These dynamics are used for the propagation of the backward reachable set

    '''
    def __init__(self) -> None:
        self.vx_max = 1.0   # m/s Maximum permissible velocity in x-axis
        self.vy_max = 1.0   # m/s Maximum permissible velocity in y-axis

        # self.dt = 0.1       # [s] Time step (10 Hz)
        self.dt = step_size       # [s] Time step (20 Hz)
        
        self.A = np.array([
            [1.0, 0.0],     # x - position
            [0.0, 1.0],     # y - position
        ])
        self.B = np.array([
            [self.vx_max*self.dt,          0.0       ],     # x - velocity
            [       0.0         , self.vy_max*self.dt],     # y - velocity
        ])


class DynamicsModel4d:
    '''
    This class defines a discrete-time linear dynamics model without disturbances.

    x_{k+1} = Ax_k + Bu_k
    
    These dynamics are used for the propagation of the backward reachable set

    States: position, velocity
    Inputs: acceleration
    
    '''
    def __init__(self) -> None:
        self.a_x_max = 1.0  # m/s^2 Maximum permissible acceleration in x-axis
        self.a_y_max = 1.0  # m/s^2 Maximum permissible acceleration in y-axis

        self.dt = step_size       # [s] Time step (20 Hz)
        
        self.A = np.array([
            [1.0, 0.0, self.dt,    0.0 ],     # x - position
            [0.0, 1.0,    0.0 , self.dt],     # y - position
            [0.0, 0.0, 1.0, 0.0],     # x - velocity
            [0.0, 0.0, 0.0, 1.0],     # y - velocity
        ])
        # self.B = np.array([
        #     [0.5*self.a_x_max*self.dt**2,          0.0       ],     # x - acceleration
        #     [       0.0         , 0.5*self.a_y_max*self.dt**2],     # y - acceleration
        #     [self.a_x_max*self.dt,          0.0       ],     # x - velocity
        #     [       0.0         , self.a_y_max*self.dt],     # y - velocity
        # ])
        self.B = np.array([
            [       0.0,                    0.0         ],     # x - acceleration
            [       0.0         ,           0.0         ],     # y - acceleration
            [self.a_x_max*self.dt,          0.0         ],     # x - velocity
            [       0.0         , self.a_y_max*self.dt  ],     # y - velocity
        ])        

