import numpy as np
import matplotlib.image as mpimg
import math
import time

from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations

from utils.cars import Car1, Car2, Car3, Car4, Car5
from utils.cars import DynamicsModel, DynamicsModel4D
from utils.targets import Targets



class Environment:
    def __init__(self, vis) -> None:
        # Step 0: Initialize parameters and objects
        car1 = Car1()
        car2 = Car2()
        car3 = Car3()
        car4 = Car4()
        car5 = Car5()
        self.vis = vis
        self.zono_op = ZonoOperations()
        self.dynamics = DynamicsModel()     # 2D dynamics model # TODO: Create a new file to add the definition of the dynamics model
        self.dynamics4D = car1.dynamics
        self.step_size = 0.1    # Step size for discretization of the space [m]

        # Step 1: Initialize state space
        self.state_space = car1.state_space4D
        self.static_brs = car1.state_space2D

        # Step 2: # Initialize Static Obstacles

        # Step 3: Initialize targets/goals
        self.target = Targets().exitparking     # Hybrid Zonotope representing the target area to be reached
        self.brs = self.target                  # Initialize brs as the target

        # Step 4: Initialize cars (All cars have the same dynamics)
        self.cars = [car1, car2, car3, car4, car5]
        
        # Step 5: Initialize visualization
        self.init_vis_space()


    def compute_brs(self):
        '''
        Computes the full BRS of the environment's state space from the specified target
        '''
        N = 56  # Empirically it needs 55 steps for the BRS to converge
        for i in range(N):
            self.brs = self.zono_op.one_step_brs_hz(X = self.state_space, T = self.brs, D = np.block([self.dynamics.A, self.dynamics.B]))

        return self.brs

    def compute_full_safe_space_v0(self, N):
        '''
        Computes the safe after taking into account dynamic obstacles
        
        # Step 1: Compute 'obs'

        obs is a list containing the non-safe space introduced due to each vehicle.
        
        # TODO: Replace N with a check if the safe space has converged

        '''
        
        # Init 2D list of obstacles, where each sublist contains all the obstacles introduced by each car
        obs = [[] for i in range(len(self.cars))]
        safe_space = [self.static_brs for i in range(len(self.cars))]

        start_time = time.perf_counter()
        for i, car in enumerate(self.cars):
            print(f'******************************************************************')
            print(f'Car {i + 1} :')
            print(f'******************************************************************')
            obs[i] = self.compute_obs_v0(car, N)
        end_time = time.perf_counter()
        print(f'time taken to compute obs = {end_time - start_time}')
        
        return self.zono_op.cz_to_hz(obs[0][N])
        # return obs[0][N]

        for c_i, car in enumerate(self.cars):   # range(len(self.cars))
            # Loop through the obs list for each car
            for i in range(len(obs[c_i])):
                
                # Check if the obstacle can be ignored
                if obs[c_i][i] is not None:
                    obs_compl = self.zono_op.complement_cz_to_hz(obs[c_i][i])
                    # Compute the 'i'-step BRS from the obstacle complement
                    for j in range(i):
                        obs_compl = self.zono_op.one_step_brs_hz(X = self.state_space, T = obs_compl, D = np.block([self.dynamics.A, self.dynamics.B]))

                    safe_space[c_i] = self.zono_op.intersection_hz_hz(safe_space[c_i], obs_compl)
                
            safe_space[c_i] = self.zono_op.union_hz_hz_v2(safe_space[c_i], car.current_road)
            # print(f'safe_space car {c_i + 1}: ng = {safe_space.ng}, nc = {safe_space.nc}, nb = {safe_space.nb}')            

        # Compute the intersection of safe space for all cars
        full_safe_space = safe_space[0]
        for c_i in range(1, len(self.cars)):
            full_safe_space = self.zono_op.intersection_hz_hz(full_safe_space, safe_space[c_i])


        return full_safe_space

    def compute_obs_v0(self, car, N):
        '''
        This method computes the non-safe space introduced due to the input moving obstacles 'cars'

        TODO: Add the funcionality of checking if the non-safe space has converged. This should be used as a terminating criterion for the loop
        '''

        obs = car.initial_space4D
        obs_pos = HybridZonotope(obs.Gc[0:2, :],  obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
        
        # Check if the obstacle is inside the conflict zone of its state space of interest
        inters = self.zono_op.intersection_hz_hz(obs_pos, car.conflict_zone)
        if self.zono_op.is_empty_hz(inters):
            full_obs = [None]
        else:
            obs_pos = self.zono_op.oa_hz_to_cz(obs_pos)
            obs_pos = self.zono_op.oa_cz_to_hypercube_tight_2d(obs_pos, bounds = car.bounds)
            obs_pos = self.zono_op.redundant_g_cz(obs_pos)
            full_obs = [obs_pos]

        for i in range(N):
            obs = self.zono_op.one_step_frs_hz_v3(X = car.state_spaceFRS, U = car.input_space, I = obs, A = car.dynamics.A, B = car.dynamics.B)
            obs_pos = HybridZonotope(obs.Gc[0:2, :],  obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)

            inters = self.zono_op.intersection_hz_hz(obs_pos, car.conflict_zone)
            if self.zono_op.is_empty_hz(inters):
                print(f'intersection is empty for time step {i}')
                full_obs.append(None)
            else:
                print(f'intersection is not empty for time step {i}')
                obs_pos = self.zono_op.oa_hz_to_cz(obs_pos)
                obs_pos = self.zono_op.oa_cz_to_hypercube_tight_2d(obs_pos, bounds = car.bounds)
                obs_pos = self.zono_op.redundant_g_cz(obs_pos)
                full_obs.append(obs_pos)
    
        return full_obs    
    
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
            print(f'******************************************************************')
            print(f'Car {i + 1} :')
            print(f'******************************************************************')
            obs = car.initial_space4D

            # # TESTING
            # obs_pos = HybridZonotope(obs.Gc[0:2, :],  obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
            # return obs_pos

            for n in range(N):
                print(f'Time step: {n + 1}')
                obs = self.zono_op.one_step_frs_hz_v3(X = car.state_spaceFRS, U = car.input_space, I = obs, A = car.dynamics.A, B = car.dynamics.B)
                obs_pos = HybridZonotope(obs.Gc[0:2, :],  obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
                obs_pos_h = obs_pos

                obstacles = self.conflict_zone_intersections(car, obs_pos)

                for obs_i, obstacle in enumerate(obstacles):
                    if obstacle is not None:
                        print(f'conflict_zone[{obs_i}] is NOT empty for time step {n + 1}')
                        obs_pos = self.zono_op.oa_hz_to_cz(obstacle)
                        bounds = self.compute_conflict_zone_bounds(car, obs_i)
                        obs_pos = self.zono_op.oa_cz_to_hypercube_tight_2d(obs_pos, bounds = bounds)
                        obs_pos = self.zono_op.redundant_g_cz(obs_pos)
                    
                        # TODO: Implement: safe space update
                        obs_compl = self.zono_op.complement_cz_to_hz(obs_pos)
                        for n_i in range(n):
                            obs_compl = self.zono_op.one_step_brs_hz(X = self.state_space, T = obs_compl, D = np.block([self.dynamics.A, self.dynamics.B]))

                        safe_space[i] = self.zono_op.intersection_hz_hz(safe_space[i], obs_compl)
                    else:
                        print(f'conflict_zone[{obs_i}] is empty for time step {n + 1}')
  
            safe_space[i] = self.zono_op.union_hz_hz_v2(safe_space[i], car.current_road)

        # # TESTING
        # return obs_pos_h

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



    def init_vis_space(self):
        '''
        Initializes the visualization space
        '''
        # space_grid 
        self.vis_space = {
            'points': [],       # A list of all [x, y] points in the space
            'flags': []         # A list of flags for each point in the space (0 if it does not exist yet, 1 if it does)
        }

        # Horizontal top lane
        y = 0.9
        for x in np.arange(-1.4 + self.step_size/2, 1.5 - self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)

        # Horizontal middle lane
        y = 0.0
        for x in np.arange(-1.4 + self.step_size/2, 1.9 - self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)

        # Horizontal bottom lane
        y = -0.9
        for x in np.arange(-1.4 + self.step_size/2, 1.4 - self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)

        # Vertical far left lane
        x = -1.35
        for y in np.arange(-0.95 + self.step_size/2, 0.95 - self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)

        # Vertical left lane
        x = -0.45
        for y in np.arange(-0.95 + self.step_size/2, 0.95 - self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)

        # Vertical right lane
        x = 0.45
        for y in np.arange(-0.95 + self.step_size/2, 0.95 - self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)

        # Vertical far right lane
        x = 1.35
        for y in np.arange(-0.95 + self.step_size/2, 0.95 - self.step_size/2, self.step_size):
            self.vis_space['points'].append([x, y])
            self.vis_space['flags'].append(0)

        # print(f'Number of points in the space: {len(self.vis_space["points"])}')

        # # TODO: TEMP FOR CAR1
        # # Horizontal middle lane : # TODO: TEMP
        # for x in np.arange(-1.7 + self.step_size/2, 0.3 - self.step_size/2, self.step_size):
        #     for y in np.arange(-0.35 + self.step_size/2, 0.35 - self.step_size/2, self.step_size):            
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)
        # # Vertical right lane : # TODO: TEMP
        # for x in np.arange(-1.7 + self.step_size/2, -1.0 - self.step_size/2, self.step_size):
        #     for y in np.arange(-0.55 + self.step_size/2, 1.05 - self.step_size/2, self.step_size):
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)

        # # TODO: TEMP FOR CAR2
        # # Horizontal middle lane : # TODO: TEMP
        # for x in np.arange(-1.4 + self.step_size/2, 1.9 - self.step_size/2, self.step_size):
        #     for y in np.arange(-0.15 + self.step_size/2, 0.25 - self.step_size/2, self.step_size):            
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)
        # # Vertical right lane : # TODO: TEMP
        # for x in np.arange(-0.6 + self.step_size/2, -0.2 - self.step_size/2, self.step_size):
        #     for y in np.arange(-0.95 + self.step_size/2, 0.95 - self.step_size/2, self.step_size):
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)

        # # TODO: TEMP FOR CAR3
        # # Horizontal middle lane : # TODO: TEMP
        # for x in np.arange(-0.9 + self.step_size/2, 0.4 - self.step_size/2, self.step_size):
        #     for y in np.arange(-0.15 + self.step_size/2, 0.25 - self.step_size/2, self.step_size):            
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)
        # # Vertical right lane : # TODO: TEMP
        # for x in np.arange(-0.6 + self.step_size/2, 0.2 - self.step_size/2, self.step_size):
        #     for y in np.arange(-0.95 + self.step_size/2, 1.05 - self.step_size/2, self.step_size):
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)        

        # # TODO: TEMP FOR CAR4
        # # Horizontal middle lane : # TODO: TEMP
        # for x in np.arange(0.1 + self.step_size/2, 1.7 - self.step_size/2, self.step_size):
        #     for y in np.arange(-0.35 + self.step_size/2, 0.35 - self.step_size/2, self.step_size):            
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)
        # # Vertical right lane : # TODO: TEMP
        # for x in np.arange(0.1 + self.step_size/2, 0.8 - self.step_size/2, self.step_size):
        #     for y in np.arange(-0.55 + self.step_size/2, 1.05 - self.step_size/2, self.step_size):
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)  
        
        # # TODO: TEMP FOR CAR5
        # # Horizontal middle lane : # TODO: TEMP
        # for x in np.arange(0.6 + self.step_size/2, 1.4 - self.step_size/2, self.step_size):
        #     for y in np.arange(0.55 + self.step_size/2, 1.25 - self.step_size/2, self.step_size):            
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)
        # # Vertical right lane : # TODO: TEMP
        # for x in np.arange(1.0 + self.step_size/2, 2.0 - self.step_size/2, self.step_size):
        #     for y in np.arange(-0.75 + self.step_size/2, 1.25 - self.step_size/2, self.step_size):
        #         self.vis_space['points'].append([x, y])
        #         self.vis_space['flags'].append(0)          


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
        img = mpimg.imread('./images/park_env_dynamic.png')
        self.vis.ax.imshow(img, extent=[-1.5, 1.8, -1.05, 1.05], zorder = 1)

    def vis_hz(self, hz):
        '''
        This version of hybrid zonotope visualization is particularly useful when you already have information about the geometry of the hz
        '''            
        # TODO: Set the marker size of the scatter method to be a square with sides equal to self.step_size
        marker_size = 0.645 * 39.36
        marker_size = marker_size**2

        space = self.vis_space['points']
        flags = self.vis_space['flags']

        # Loop through all points in the space
        for i, point in enumerate(space):
            p = np.array([ [point[0]], [point[1]] ])
            if self.zono_op.is_inside_hz(hz, p):
                flags[i] = 1
                self.vis.ax.scatter(p[0], p[1], marker = 's', s = marker_size, color = '#6C8EBF', alpha = 0.5, zorder = 11, edgecolors = 'face' )

        # Update flags
        self.vis_space['flags'] = flags















