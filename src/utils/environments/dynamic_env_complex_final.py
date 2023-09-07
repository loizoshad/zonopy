import numpy as np
import matplotlib.image as mpimg
import matplotlib.patches as patches
import math
import time

from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations

from utils.cars import Car1, Car2, Car3, Car4, Car5
from utils.cars import DynamicsModel, DynamicsModel4D
from utils.targets import Targets

step_size = 0.05
# step_size = 0.1

class Environment:
    def __init__(self, vis, t) -> None:
        # Step 0: Initialize parameters and objects
        car1 = Car1(t); car2 = Car2(t); car3 = Car3(t); car4 = Car4(t); car5 = Car5(t)
        self.vis = vis
        self.zono_op = ZonoOperations()
        self.dynamics = DynamicsModel()     # 2D dynamics model # TODO: Create a new file to add the definition of the dynamics model
        self.dynamics4D = DynamicsModel4D()
        self.step_size = step_size    # Step size for discretization of the space [m]

        # Step 1: Initialize state space
        self.state_space = car1.state_space4D
        self.static_brs = car1.state_space2D

        # Step 2: Initialize targets/goals
        self.target = Targets().exitparking     # Hybrid Zonotope representing the target area to be reached
        self.brs = self.target                  # Initialize brs as the target

        # Step 3: Initialize cars (All cars have the same dynamics)
        self.cars = [car1, car2, car3, car4, car5]
        # self.cars = [car1, car2]

        # Step 4: Initialize visualization
        self.init_vis_space()


    def compute_brs(self):
        '''
        Computes the full BRS of the environment's state space from the specified target
        '''
        N = 56  # Empirically it needs 55 steps for the BRS to converge
        for i in range(N):
            self.brs = self.zono_op.one_step_brs_hz(X = self.state_space, T = self.brs, D = np.block([self.dynamics.A, self.dynamics.B]))

        return self.brs

    def compute_full_safe_space(self, N):
        '''
        Computes the safe after taking into account dynamic obstacles
        
        # Step 1: Compute 'obs'

        obs is a list containing the non-safe space introduced due to each vehicle.
        
        # TODO: Replace N with a check if the safe space has converged

        '''
        
        # Init 2D list of obstacles, where each sublist contains all the obstacles introduced by each car
        obs = [[] for i in range(len(self.cars))]
        safe_space = [self.static_brs for i in range(len(self.cars))]

        for i, car in enumerate(self.cars):
            if car.state_spaceFRS is None:
                continue
            print(f'******************************************************************')
            print(f'Car {i + 1} :')
            print(f'******************************************************************')
            obs = car.initial_space4D

            # print(f'safe_space: ng = {safe_space[i].ng}, nc = {safe_space[i].nc}, nb = {safe_space[i].nb}')
            is_obsolete = False
            for n in range(N):
                if n > 15 and i != 0:
                    print(f'EXCEEDED n = {n} for car {i + 1}')
                    break
                obs = self.zono_op.one_step_frs_hz_v3(X = car.state_spaceFRS, U = car.input_space, I = obs, A = car.dynamics.A, B = car.dynamics.B)
                obs_pos = HybridZonotope(obs.Gc[0:2, :],  obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
                obs_pos_h = obs_pos

                obstacles = self.conflict_zone_intersections(car, obs_pos)

                print(f'TIME STEP: {n + 1}')
                for obs_i, obstacle in enumerate(obstacles):
                    if obstacle is not None:
                        print(f'\tconflict_zone[{obs_i}] is NOT empty for time step {n + 1}')
                        obs_pos = self.zono_op.oa_hz_to_cz(obstacle)
                        bounds = self.compute_conflict_zone_bounds(car, obs_i)
                        obs_pos = self.zono_op.oa_cz_to_hypercube_tight_2d(obs_pos, bounds = bounds)
                        # print(f'Before: obs_pos: G = \n{obs_pos.G}')
                        obs_pos = self.check_zero_hypercube(obs_pos)            # Check 'zero' hypercube dimensions
                        # print(f'After: obs_pos: G = \n{obs_pos.G}')
                        obs_pos = self.zono_op.redundant_g_cz(obs_pos)
                        obs_compl = self.zono_op.complement_cz_to_hz(obs_pos)
                                            
                        for n_i in range(n):
                            obs_compl = self.zono_op.one_step_brs_hz(X = self.state_space, T = obs_compl, D = np.block([self.dynamics.A, self.dynamics.B]))


                        safe_space[i] = self.zono_op.intersection_hz_hz(safe_space[i], obs_compl)
                        # print(f'safe_space: ng = {safe_space[i].ng}, nc = {safe_space[i].nc}, nb = {safe_space[i].nb}')

                        # Check if the all k steps such that k > n are obsolete 
                        if self.zono_op.is_empty_hz(self.zono_op.intersection_hz_hz(obs_compl, car.related_conflict_area)):
                            print(f'OBSOLETE FOR ALL k > {n + 1}')
                            is_obsolete = True
                            break
                    else:
                        # pass
                        print(f'\tconflict_zone[{obs_i}] is empty for time step {n + 1}')

                if is_obsolete:
                    break


            safe_space[i] = self.zono_op.union_hz_hz_v2(safe_space[i], car.current_road)


        full_safe_space = safe_space[0]
        for c_i in range(1, len(self.cars)):
            full_safe_space = self.zono_op.intersection_hz_hz(full_safe_space, safe_space[c_i])

        
        return full_safe_space




    def conflict_zone_intersections(self, car, obs):
        obstacles = []
        for cz in car.conflict_zone:
            inters = self.zono_op.intersection_hz_hz(obs, cz)
            if self.zono_op.is_empty_hz(inters):
                obstacles.append(None)
            else:
                obstacles.append(inters)

        return obstacles

    def compute_conflict_zone_bounds(self, car, i):
        cx = car.conflict_zone[i].C[0, 0]; gx = car.conflict_zone[i].Gc[0, 0]
        cy = car.conflict_zone[i].C[1, 0]; gy = car.conflict_zone[i].Gc[1, 1]

        min_x = cx - abs(gx) - 2*self.step_size
        max_x = cx + abs(gx) + 2*self.step_size
        min_y = cy - abs(gy) - 2*self.step_size
        max_y = cy + abs(gy) + 2*self.step_size

        return np.array([ [min_x, max_x], [min_y, max_y] ])

    def set_bounds(self, car, obs):
        min_val = np.zeros((obs.dim, 1))
        max_val = np.zeros((obs.dim, 1))

        for n in range(obs.dim):
            min_val[n, 0] = obs.C[n, 0] - abs(obs.G[n, n]) - 2*self.step_size
            max_val[n, 0] = obs.C[n, 0] + abs(obs.G[n, n]) + 2*self.step_size

        car.bounds = np.block([ [min_val, max_val] ])

    def check_zero_hypercube(self, obs_pos):
        '''
        This method checks if any of the hypercube dimensions are zero. If so, it replaces the dimension with a very small value

        Sometimes, the dynamics of the car can lead to a very small jump not detected by the hypercube over-approximation.
        This method is a workaround to fix this issue by replacing the zero dimension with a very small value and still provide an over-approximation.        
        '''
        eps = 1e-4
        for i in range(obs_pos.ng):
            if abs(obs_pos.G[i, i]) <= eps:
                obs_pos.G[i, i] = self.step_size / 4
                # obs_pos.G[i, i] = self.step_size / 2

        return obs_pos


    




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

    def vis_exit_space(self):
        w = 0.4; h = 0.1


        # Exit 1
        p = np.array([ [1.6], [0.9] ])
        rect = patches.Rectangle((p[0] - w/2, p[1] - h/2), w, h, linewidth = 0.1, edgecolor = '#6C8EBF', facecolor = '#6C8EBF', alpha = 0.4, zorder = 11)            
        self.vis.ax.add_patch(rect)
    
        # Exit 2
        p = np.array([ [1.6], [0.0] ])
        rect = patches.Rectangle((p[0] - w/2, p[1] - h/2), w, h, linewidth = 0.1, edgecolor = '#6C8EBF', facecolor = '#6C8EBF', alpha = 0.4, zorder = 11)            
        self.vis.ax.add_patch(rect)

        # Entrance
        p = np.array([ [1.6], [-0.9] ])
        rect = patches.Rectangle((p[0] - w/2, p[1] - h/2), w, h, linewidth = 0.1, edgecolor = '#6C8EBF', facecolor = '#6C8EBF', alpha = 0.4, zorder = 11)            
        self.vis.ax.add_patch(rect)

    def vis_env(self):
        '''
        Visualizes the plain environment
        '''
        # Visualize background
        img = mpimg.imread('./images/park_env_dynamic.png')
        self.vis.ax.imshow(img, extent=[-1.5, 1.8, -1.05, 1.05], zorder = 1)

    def vis_hz(self, hz):
        '''
        This version of hybrid zonotope visualization is particularly useful when you already have information about the geometry of the hz
        '''            
        # TODO: Set the marker size of the scatter method to be a square with sides equal to self.step_size
        # # marker_size = 0.645 * 39.36   # Step size 0.1
        # marker_size = 0.64024 * 39.36 # Step size 0.1
        # marker_size = marker_size**2

        d_points = (step_size * 2.51) * self.vis.fig.dpi
        d_points = (0.01) * self.vis.fig.dpi
        marker_size = d_points**2

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













    def compute_full_safe_space_v1(self, N):
        '''
        LATEST WORKING METHOD. After that I tried to optimize it
        '''
        
        # Init 2D list of obstacles, where each sublist contains all the obstacles introduced by each car
        obs = [[] for i in range(len(self.cars))]
        safe_space = [self.static_brs for i in range(len(self.cars))]

        for i, car in enumerate(self.cars):
            if car.state_spaceFRS is None:
                continue
            # print(f'******************************************************************')
            # print(f'Car {i + 1} :')
            # print(f'******************************************************************')
            obs = car.initial_space4D

            # print(f'safe_space: ng = {safe_space[i].ng}, nc = {safe_space[i].nc}, nb = {safe_space[i].nb}')
            for n in range(N):
                obs = self.zono_op.one_step_frs_hz_v3(X = car.state_spaceFRS, U = car.input_space, I = obs, A = car.dynamics.A, B = car.dynamics.B)
                obs_pos = HybridZonotope(obs.Gc[0:2, :],  obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
                obs_pos_h = obs_pos

                obstacles = self.conflict_zone_intersections(car, obs_pos)

                for obs_i, obstacle in enumerate(obstacles):
                    if obstacle is not None:
                        # print(f'conflict_zone[{obs_i}] is NOT empty for time step {n + 1}')
                        obs_pos = self.zono_op.oa_hz_to_cz(obstacle)
                        bounds = self.compute_conflict_zone_bounds(car, obs_i)
                        obs_pos = self.zono_op.oa_cz_to_hypercube_tight_2d(obs_pos, bounds = bounds)
                        # print(f'Before: obs_pos: G = \n{obs_pos.G}')
                        obs_pos = self.check_zero_hypercube(obs_pos)            # Check 'zero' hypercube dimensions
                        # print(f'After: obs_pos: G = \n{obs_pos.G}')
                        obs_pos = self.zono_op.redundant_g_cz(obs_pos)
                        obs_compl = self.zono_op.complement_cz_to_hz(obs_pos)
                                            
                        for n_i in range(n):
                            obs_compl = self.zono_op.one_step_brs_hz(X = self.state_space, T = obs_compl, D = np.block([self.dynamics.A, self.dynamics.B]))

                        safe_space[i] = self.zono_op.intersection_hz_hz(safe_space[i], obs_compl)
                        # print(f'safe_space: ng = {safe_space[i].ng}, nc = {safe_space[i].nc}, nb = {safe_space[i].nb}')
                    # else:
                    #     # pass
                    #     print(f'conflict_zone[{obs_i}] is empty for time step {n + 1}')

            safe_space[i] = self.zono_op.union_hz_hz_v2(safe_space[i], car.current_road)


        full_safe_space = safe_space[0]
        for c_i in range(1, len(self.cars)):
            full_safe_space = self.zono_op.intersection_hz_hz(full_safe_space, safe_space[c_i])

        
        # full_safe_space = self.remove_static_obstacles(full_safe_space)

        return full_safe_space


