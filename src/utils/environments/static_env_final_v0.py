import numpy as np
import matplotlib.image as mpimg
import matplotlib.patches as patches
import math
import time

from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations

# step_size = 0.1
# step_size = 0.05
step_size = 0.025
# step_size = 0.02

class Environment:
    def __init__(self, vis) -> None:
        # Step 0: Initialize parameters and objects
        self.vis = vis
        self.zono_op = ZonoOperations()
        self.dynamics = DynamicsModel() 
        self.step_size = step_size

        self.targets = [Target1(), Target2(), Target3(), Target4(), Target5()]
        # self.targets = [Target5()]


    @property
    def state_space(self):
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

        # Target 1 (Taxi)
        Gc = np.diag(np.array([ 0.20, 0.05, 1.0, 1.0]))
        c = np.array([ [-0.65], [0.60], [0.0], [0.0]])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        parking_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        state_space = self.zono_op.union_hz_hz_v2(state_space, parking_1)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)


        return state_space
    

    def compute_brs(self, N = 50):
        '''
        Computes the BRS from each target area starting from the state space
        '''
        D = np.block([self.dynamics.A, self.dynamics.B])
        brs = [target.hz for target in self.targets]

        for n in range(N):
            print(f'Time step: {n + 1}')
            for t_i, target in enumerate(self.targets):
                X = target.state_space
                brs[t_i] = self.zono_op.one_step_brs_hz(X = X, T = brs[t_i], D = D)
                target.brs = brs[t_i]
                print(f'Target {t_i}: ng = {brs[t_i].ng}, nc = {brs[t_i].nc}, nb = {brs[t_i].nb}')

            # Visualize current time step
            self.vis_targets(self.targets)

            # plt.show()
            name = f'brs_{n}'
            # self.vis.fig.savefig(f'./results/static_final/target1/png/{name}.png', dpi=300)
            # self.vis.fig.savefig(f'./results/static_final/target5/png_step_025/{name}.png', dpi=300)

            # self.vis.fig.savefig(f'./results/static_final/png_step_1/{name}.png', dpi=300)
            # self.vis.fig.savefig(f'./results/static_final/png_step_05/{name}.png', dpi=300)
            # self.vis.fig.savefig(f'./results/static_final/png_step_02/{name}.png', dpi=300)
            self.vis.fig.savefig(f'./results/static_final/png_step_025/{name}.png', dpi=300)



        # # Compute the union between all brs[t_i]
        # self.brs = brs[0]
        # self.brs = self.zono_op.union_hz_hz_v2(brs[0], brs[1])
        # for t_i in range(2, len(brs)):
        #     self.brs = self.zono_op.union_hz_hz_v2(brs, brs[t_i])

        return self.brs

   

    def init_vis_space(self):
        '''
        Initializes the visualization space
        '''
        # space_grid 
        self.vis_space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': [],         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
            'direction': []
        }

        # Horizontal top lane
        y = 0.9
        for x in np.arange(-1.4 + self.step_size/2, 1.5 - self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('horizontal')
        # Horizontal middle lane
        y = 0.0
        for x in np.arange(-1.4 + self.step_size/2, 1.4 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('horizontal')
        # Horizontal bottom lane left
        y = -0.9
        for x in np.arange(-1.4 + self.step_size/2, 1.4 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('horizontal')

        # Vertical far left lane
        x = -1.35
        for y in np.arange(0.05 + self.step_size/2, 0.85 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        for y in np.arange(-0.85 + self.step_size/2, -0.05 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        # Vertical left lane
        x = -0.45
        for y in np.arange(0.05 + self.step_size/2, 0.85 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        for y in np.arange(-0.85 + self.step_size/2, -0.05 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        # Vertical right lane
        x = 0.45
        for y in np.arange(0.05 + self.step_size/2, 0.85 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        for y in np.arange(-0.85 + self.step_size/2, -0.05 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        # Vertical far right lane
        x = 1.35
        for y in np.arange(0.05 + self.step_size/2, 0.85 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')
        for y in np.arange(-0.85 + self.step_size/2, -0.05 + self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)
            self.vis_space['direction'].append('vertical')    

        # print(f'Number of points in vis_space = {len(self.vis_space["points"])}')

    def vis_safe_space(self, safe_space):
        '''
        Visualizes both the environment and the safe space given as an input
        '''
        self.vis_env()
        self.vis_hz(safe_space)

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

        # Loop through all points in the space
        for i, point in enumerate(space):
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

        # Update flags
        self.vis_space['flags'] = flags

    def vis_brs(self):
        '''
        Visualizes both the environment and the safe space given as an input
        '''
        self.vis_env()
        self.vis_brs_()

    def vis_brs_(self):
        '''
        This version of hybrid zonotope visualization is particularly useful when you already have information about the geometry of the hz
        '''
        
        for t_i, target in enumerate(self.targets):
            space = target.vis_space['points']
            flags = target.vis_space['flags']
            dir = target.vis_space['direction']

            # Loop through all points in the space
            for i, point in enumerate(space):
                p = np.array([ [point[0]], [point[1]] ])
                # if flags[i] == 1:
                #     continue
                # if self.zono_op.is_inside_hz(target.hz, p):
                if self.zono_op.is_inside_hz(self.brs, p):                
                    flags[i] = 1

                    if dir[i] == 'horizontal':
                        w = step_size
                        # h = 0.1
                        h = step_size
                        rect = patches.Rectangle((p[0] - w/2, p[1] - h/2), w, h, linewidth = 0.1, edgecolor = '#6C8EBF', facecolor = '#6C8EBF', alpha = 0.4, zorder = 11)
                    elif dir[i] == 'vertical':
                        # w = 0.1
                        w = step_size
                        h = step_size
                        rect = patches.Rectangle((p[0] - w/2, p[1] - h/2), w, h, linewidth = 0.1, edgecolor = '#6C8EBF', facecolor = '#6C8EBF', alpha = 0.4, zorder = 11)

                    self.vis.ax.add_patch(rect)

            # Update flags
            target.vis_space['flags'] = flags

    def vis_targets(self, targets):
        '''
        Visualizes both the environment and the safe space given as an input
        '''
        self.vis_env()
        self.vis_target_hz(targets)

    def vis_target_hz(self, targets):
        '''
        This version of hybrid zonotope visualization is particularly useful when you already have information about the geometry of the hz
        '''
        
        for t_i, target in enumerate(targets):
            space = target.vis_space['points']
            flags = target.vis_space['flags']
            dir = target.vis_space['direction']

            # Loop through all points in the space
            for i, point in enumerate(space):
                p = np.array([ [point[0]], [point[1]] ])
                if flags[i] == 1:
                    continue
                if self.zono_op.is_inside_hz(target.brs, p):
                    flags[i] = 1

                    w = step_size
                    h = step_size
                    rect = patches.Rectangle((p[0] - w/2, p[1] - h/2), w, h, linewidth = 0.1, edgecolor = '#6C8EBF', facecolor = '#6C8EBF', alpha = 0.4, zorder = 11)                    

                    self.vis.ax.add_patch(rect)

            # Update flags
            target.vis_space['flags'] = flags



class Target1:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()
        self.step_size = step_size
        self.state_space = self.set_state_space()
        self.hz = self.set_hz()
        self.set_vis_space()
        self.brs = self.hz

    def set_hz(self):
        # Target 1 (Taxi)
        Gc = np.diag(np.array([ 0.1, 0.05]))
        # Gc = np.diag(np.array([ 10.0, 10.0]))
        c = np.array([ [-0.70], [0.60]])

        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        target = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return target        

    def set_state_space(self):
        # Vertical Road Sections
        Gc = np.diag(np.array([ 0.05, 0.4, 0.5, 0.5 ]))
        c = np.array([ [-0.9], [0.45], [ 0.0], [ 0.0] ])
        Gb = np.array([ [0.45], [0.00], [0.00], [-0.5] ])
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Horizontal Top Lane Section
        Gc = np.diag(np.array([ 0.5, 0.05, 0.5, 0.5 ]))
        c = np.array([ [-0.9], [0.9], [ 0.5], [ 0.0] ])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)        

        state_space = self.zono_op.union_hz_hz_v2(road_v, road_h)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        # Target Space
        Gc = np.diag(np.array([ 0.20, 0.05, 1.0, 1.0]))
        c = np.array([ [-0.65], [0.60], [0.0], [0.0]])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        parking = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        state_space = self.zono_op.union_hz_hz_v2(state_space, parking)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        return state_space


    def set_vis_space(self):
        self.vis_space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': [],         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
            'direction': []
        }

        # Parking Place
        x = -1.35
        for y in np.arange(0.55 + self.step_size/2, 0.65 + self.step_size/2, self.step_size):
            for x in np.arange(-0.8 + self.step_size/2, -0.5, self.step_size):
            # for x in np.arange(-0.8 + self.step_size/2, -0.5 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')
        # Vertical right lane
        for y in np.arange(0.55 + self.step_size/2, 0.85 + self.step_size/2, self.step_size):
            for x in np.arange(-0.5 + self.step_size/2, -0.4 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('vertical')
        # Horizontal top lane
        for y in np.arange(0.85 + self.step_size/2, 0.95 + self.step_size/2, self.step_size):
            for x in np.arange(-1.4 + self.step_size/2, -0.4 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')
        # Vertical left lane
        for y in np.arange(0.05 + self.step_size/2, 0.85 + self.step_size/2, self.step_size):
            for x in np.arange(-1.4 + self.step_size/2, -1.3 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('vertical')

        print(f'Target 1 - Vis Space: {len(self.vis_space["points"])}')

class Target2:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()
        self.step_size = step_size
        self.state_space = self.set_state_space()
        self.hz = self.set_hz()
        self.set_vis_space()
        self.brs = self.hz

    def set_hz(self):
        Gc = np.diag(np.array([ 0.1, 0.05]))
        c = np.array([ [0.70], [0.60]])

        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        target = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return target        

    def set_state_space(self):
        # Vertical Road Section
        Gc = np.diag(np.array([ 0.05, 0.70, 0.5, 0.5 ]))
        c  = np.array([ [0.45], [0.0], [0.0], [ 0.5] ])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Horizontal Middle Lane Section
        Gc = np.diag(np.array([ 0.4, 0.05, 0.5, 0.5 ]))
        c = np.array([ [0.0], [0.0], [ 0.5], [ 0.0] ])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)        

        state_space = self.zono_op.union_hz_hz_v2(road_v, road_h)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        # Target Space
        Gc = np.diag(np.array([ 0.15, 0.05, 1.0, 1.0]))
        c = np.array([ [0.65], [0.60], [0.0], [0.0]])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        parking = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        state_space = self.zono_op.union_hz_hz_v2(state_space, parking)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        return state_space


    def set_vis_space(self):
        self.vis_space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': [],         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
            'direction': []
        }

        # Parking Place
        for y in np.arange(0.55 + self.step_size/2, 0.65 + self.step_size/2, self.step_size):
            for x in np.arange(0.4 + self.step_size/2, 0.8 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')
        # Vertical lane
        for y in np.arange(-0.55 + self.step_size/2, 0.55 + self.step_size/2, self.step_size):
            for x in np.arange(0.4 + self.step_size/2, 0.5 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('vertical')
        # Horizontal middle lane
        for y in np.arange(-0.05 + self.step_size/2, 0.05 + self.step_size/2, self.step_size):
            for x in np.arange(-0.4 + self.step_size/2, 0.4 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')

        print(f'Target 2 - Vis Space: {len(self.vis_space["points"])}')
        
class Target3:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()
        self.step_size = step_size
        self.state_space = self.set_state_space()
        self.hz = self.set_hz()
        self.set_vis_space()
        self.brs = self.hz

    def set_hz(self):
        Gc = np.diag(np.array([ 0.1, 0.05]))
        c = np.array([ [1.1], [0.60]])

        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        target = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return target        

    def set_state_space(self):
        # Vertical Road Sections
        Gc = np.diag(np.array([ 0.05, 0.15, 0.5, 0.5 ]))
        c  = np.array([ [0.9], [0.70], [0.0], [ 0.0] ])
        Gb = np.array([ [0.45], [0.0], [0.0], [-0.5]])
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Horizontal Top Lane Section
        Gc = np.diag(np.array([ 1.0, 0.05, 0.5, 0.5 ]))
        c = np.array([ [0.45], [0.90], [ 0.5], [ 0.0] ])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)        

        state_space = self.zono_op.union_hz_hz_v2(road_v, road_h)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        # Target Space
        Gc = np.diag(np.array([ 0.15, 0.05, 1.0, 1.0]))
        c = np.array([ [1.15], [0.60], [0.0], [0.0]])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        parking = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        state_space = self.zono_op.union_hz_hz_v2(state_space, parking)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        return state_space


    def set_vis_space(self):
        self.vis_space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': [],         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
            'direction': []
        }

        # Parking Place
        for y in np.arange(0.55 + self.step_size/2, 0.65 + self.step_size/2, self.step_size):
            for x in np.arange(1.0 + self.step_size/2, 1.4, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')
        # Vertical lane left
        for y in np.arange(0.65 + self.step_size/2, 0.85 + self.step_size/2, self.step_size):
            for x in np.arange(0.4 + self.step_size/2, 0.5 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('vertical')
        # Vertical lane right
        for y in np.arange(0.65 + self.step_size/2, 0.85 + self.step_size/2, self.step_size):
            for x in np.arange(1.3 + self.step_size/2, 1.4 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('vertical')                
        # Horizontal Top lane
        for y in np.arange(0.85 + self.step_size/2, 0.95 + self.step_size/2, self.step_size):
            for x in np.arange(-0.4 + self.step_size/2, 1.4 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')

        print(f'Target 3 - Vis Space: {len(self.vis_space["points"])}')




class Target4:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()
        self.step_size = step_size
        self.state_space = self.set_state_space()
        self.hz = self.set_hz()
        self.set_vis_space()
        self.brs = self.hz

    def set_hz(self):
        Gc = np.diag(np.array([ 0.1, 0.05]))
        c = np.array([ [-0.2], [-0.3]])

        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        target = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return target        

    def set_state_space(self):
        # Vertical Road Sections
        Gc = np.diag(np.array([ 0.05, 0.85, 0.5, 0.5 ]))
        c  = np.array([ [-0.9], [0.0], [0.0], [ 0.0] ])
        Gb = np.array([ [0.45], [0.0], [0.0], [-0.5]])
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Horizontal Lane Sections
        Gc = np.diag(np.array([ 0.95, 0.05, 0.5, 0.5 ]))
        c = np.array([  [-0.45], [-0.45], [0.0], [0.0] ])
        Gb = np.array([ [ 0.0], [0.45], [0.5], [0.0]])
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)        

        state_space = self.zono_op.union_hz_hz_v2(road_v, road_h)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        # Target Space
        Gc = np.diag(np.array([ 0.15, 0.05, 1.0, 1.0]))
        c = np.array([ [-0.25], [-0.3], [0.0], [0.0]])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        parking = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        state_space = self.zono_op.union_hz_hz_v2(state_space, parking)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        return state_space

    def set_vis_space(self):
        self.vis_space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': [],         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
            'direction': []
        }

        # Parking Place
        for y in np.arange(-0.35 + self.step_size/2, -0.25 + self.step_size/2, self.step_size):
            for x in np.arange(-0.4 + self.step_size/2, -0.1 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')
        # Vertical lane left
        for y in np.arange(-0.85 + self.step_size/2, -0.05 + self.step_size/2, self.step_size):
            for x in np.arange(-1.4 + self.step_size/2, -1.3 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('vertical')
        # Vertical lane right
        for y in np.arange(-0.85 + self.step_size/2, 0.55 + self.step_size/2, self.step_size):
            for x in np.arange(-0.5 + self.step_size/2, -0.4 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('vertical')                
        # Horizontal Middle lane
        for y in np.arange(-0.05 + self.step_size/2, 0.05 + self.step_size/2, self.step_size):
            for x in np.arange(-1.4 + self.step_size/2, -0.5 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')
        # Horizontal Bottom lane
        for y in np.arange(-0.95 + self.step_size/2, -0.85 + self.step_size/2, self.step_size):
            for x in np.arange(-1.4 + self.step_size/2, 0.4 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')

        print(f'Target 4 - Vis Space: {len(self.vis_space["points"])}')

class Target5:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()
        self.step_size = step_size
        self.state_space = self.set_state_space()
        self.hz = self.set_hz()
        self.set_vis_space()
        self.brs = self.hz

    def set_hz(self):
        Gc = np.diag(np.array([ 0.1, 0.05]))
        c = np.array([ [0.2], [-0.6]])

        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        target = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        return target        

    def set_state_space(self):
        # Vertical Road Sections
        Gc = np.diag(np.array([ 0.05, 0.85, 0.5, 0.5 ]))
        c  = np.array([ [0.9], [0.0], [0.0], [ 0.0] ])
        Gb = np.array([ [0.45], [0.0], [0.0], [-0.5]])
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Horizontal Lane Sections
        # Gc = np.diag(np.array([ 0.45, 0.05, 0.5, 0.5 ]))
        # c = np.array([  [0.85], [-0.45], [0.0], [0.0] ])
        # Gb = np.array([ [-0.05], [0.45], [0.5], [0.0]])
        # Ac = np.zeros((0, 4)); Ab = np.zeros((0, 1)); b = np.zeros((0, 1))
        # road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)  

        # Horizontal Middle Lane Section
        Gc = np.diag(np.array([ 0.40, 0.05, 0.5, 0.5 ]))
        c = np.array([  [0.90], [0.0], [0.5], [0.0] ])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        road_h_mid = HybridZonotope(Gc, Gb, c, Ac, Ab, b)  

        # Horizontal Bottom Lane Section
        Gc = np.diag(np.array([ 0.90, 0.05, 0.5, 0.5 ]))
        c = np.array([  [0.90], [-0.90], [-0.5], [0.0] ])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        road_h_bot = HybridZonotope(Gc, Gb, c, Ac, Ab, b)  

        road_h = self.zono_op.union_hz_hz_v2(road_h_mid, road_h_bot)
        road_h = self.zono_op.redundant_c_gc_hz_v2(road_h)
        road_h = self.zono_op.redundant_c_gc_hz_v1(road_h)

        state_space = self.zono_op.union_hz_hz_v2(road_v, road_h)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        # Target Space
        Gc = np.diag(np.array([ 0.15, 0.05, 1.0, 1.0]))
        c = np.array([ [0.25], [-0.6], [0.0], [0.0]])
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        parking = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        state_space = self.zono_op.union_hz_hz_v2(state_space, parking)
        state_space = self.zono_op.redundant_c_gc_hz_v2(state_space)
        state_space = self.zono_op.redundant_c_gc_hz_v1(state_space)

        return state_space

    def set_vis_space(self):
        self.vis_space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': [],         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
            'direction': []
        }

        # Parking Place
        for y in np.arange(-0.65 + self.step_size/2, -0.55 + self.step_size/2, self.step_size):
            for x in np.arange(0.1 + self.step_size/2, 0.5 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')
        # Vertical lane left
        for y in np.arange(-0.95 + self.step_size/2, -0.65 + self.step_size/2, self.step_size):
            for x in np.arange(0.4 + self.step_size/2, 0.5 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('vertical')
        # Vertical lane right
        for y in np.arange(-0.85 + self.step_size/2, 0.55 + self.step_size/2, self.step_size):
            for x in np.arange(1.3 + self.step_size/2, 1.4 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('vertical')
        # Horizontal Middle lane
        for y in np.arange(-0.05 + self.step_size/2, 0.05 + self.step_size/2, self.step_size):
            for x in np.arange(0.5 + self.step_size/2, 1.3 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')        
        # Horizontal Bottom lane
        for y in np.arange(-0.95 + self.step_size/2, -0.85 + self.step_size/2, self.step_size):
            # for x in np.arange(0.5 + self.step_size/2, 0.7 + self.step_size/2, self.step_size):
            for x in np.arange(0.5 + self.step_size/2, 1.8 + self.step_size/2, self.step_size):
                self.vis_space['points'].append([x, y])
                self.vis_space['flags'].append(0)
                self.vis_space['direction'].append('horizontal')


        print(f'Target 5 - Vis Space: {len(self.vis_space["points"])}')






class DynamicsModel:
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
