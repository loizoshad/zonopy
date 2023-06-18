ALWAYS_SYMBOL = '\u25A1'
EVENTUALLY_SYMBOL = '\u25CA'
OR_SYMBOL = '\u2228'
AND_SYMBOL = '\u2227'
UNION_SYMBOL = '\u222A'
INTERSECTION_SYMBOL = '\u2229'
NEXT_SYMBOL = '\u25EF'
UNTIL_SYMBOL = '\u0055'
WUNTIL_SYMBOL = '\u0057'
REACH_SYMBOL = '\u211B'

import casadi as ca
import numpy as np
import math
from scipy.optimize import linprog
from copy import deepcopy
import itertools
import time

# For mixed-integer linear programming
import gurobipy as gp
# from gurobipy import GRB
from gurobipy import *

import scipy.sparse as sp

from utils.sets.zonotopes import Zonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.tlt.nodes import OR, AND, UNTIL, set_node
from utils.tlt.tree import Tree

'''

References:

[1] J. Scott, et al. 'Constrained zonotopes: A new tool for set-based estimation and fault detection'
[2] L. Yang, et al.  'Efficient Backward Reachability Using the Minkowski Difference of Constrained Zonotopes'
[3] T. Bird, et al.  'Unions and Complements of Hybrid Zonotopes'
'''

class ZonoOperations:
    def __init__(self, visualizer = None):

        self.visualizer = visualizer

    def ms_z_z(self, z1: Zonotope, z2: Zonotope) -> Zonotope:
        '''
        Computes Minkowski sum of two zonotopes.

        - z1: first zonotope
        - z2: second zonotope
        '''
        assert z1.dim == z2.dim, f'Both zonotopes must have the same dimensionality, whereas z1.dimension = {z1.dimension} and z2.dimension = {z2.dimension}.'

        C = z1.C + z2.C
        G = np.hstack((z1.G, z2.G))

        return Zonotope(C, G)
    
    def lt_z(self, M: np.ndarray, z: Zonotope) -> Zonotope:
        '''
        Computes the linear transformation of a zonotope.
        A zonotope resulting from a linear transformation of a zonotope z = (C, < g1, g2, ..., gn >)
        
        M @ z = (M @ C, < M @ g1, M @ g2, ..., M @ gn >)
        '''

        assert M.shape[0] == M.shape[1] == z.dim,   f'Linear transformation matrix must be square \
                                                    whereas A.shape = [{M.shape[0]}, {M.shape[1]}] and z.dim = {z.dim}.'

        return Zonotope(M @ z.C , M @ z.G)

    def lt_cz(self, M: np.ndarray, cz: ConstrainedZonotope) -> ConstrainedZonotope:
        '''
        Computes the linear transformation of a constrained zonotope.
        A constrained zonotope resulting from a linear transformation of a constrained zonotope cz = (C, G, A, b)
        
        M @ cz = (M @ C, M g, Aa, Ab)

        See the work in [1] for details
        '''

        assert M.shape[0] == M.shape[1] == cz.dim,   f'Linear transformation matrix must be square \
                                                    whereas M.shape = [{M.shape[0]}, {M.shape[1]}] and cz.dim = {cz.dim}.'

        return ConstrainedZonotope(M @ cz.G, M @ cz.C , cz.A, cz.b)
    
    def ms_cz_cz(self, cz1: ConstrainedZonotope, cz2: ConstrainedZonotope) -> ConstrainedZonotope:
        '''
        Computes Minkowski sum of two constrained zonotopes.

        - cz1: first constrained zonotope
        - cz2: second constrained zonotope

        See the work in [1] for details
        '''
        assert cz1.dim == cz2.dim, f'Both constrained zonotopes must have the same dimensionality, \
                                    whereas cz1.dimension = {cz1.dimension} and cz2.dimension = {cz2.dimension}.'

        C = cz1.C + cz2.C
        G = np.hstack((cz1.G, cz2.G))

        A = np.block([
            [cz1.A, np.zeros((cz1.A.shape[0], cz2.A.shape[1]))],
            [np.zeros((cz2.A.shape[0], cz1.A.shape[1])), cz2.A]
        ])

        b = np.vstack((cz1.b, cz2.b))

        return ConstrainedZonotope(G, C, A, b)
    
    def intersection_cz_cz(self, cz1: ConstrainedZonotope, cz2: ConstrainedZonotope) -> ConstrainedZonotope:
        '''
        Computes the intersection of two constrained zonotopes

        See the work in [1] for details
        '''
        assert cz1.dim == cz2.dim, f'Both constrained zonotopes must have the same dimensionality, \
                                    whereas cz1.dimension = {cz1.dimension} and cz2.dimension = {cz2.dimension}.'
        
        C = cz1.C
        G = np.block([cz1.G, np.zeros((cz1.G.shape[0], cz2.G.shape[1]))]) # TODO

        A = np.block([
            [cz1.A, np.zeros((cz1.A.shape[0], cz2.A.shape[1]))],
            [np.zeros((cz2.A.shape[0], cz1.A.shape[1])), cz2.A],
            [cz1.G, -cz2.G]
        ])

        b = np.block([
            [cz1.b],
            [cz2.b],
            [cz2.C - cz1.C]
        ])
        
        return ConstrainedZonotope(G, C, A, b)
    
    def is_inside_cz(self, cz: ConstrainedZonotope, point: np.ndarray) -> bool:
        # Step 1: Objective function
        c = np.zeros((cz.ng, 1)).reshape(-1)

        # Step 2: Equality constraints
        #  A*x = b and G*x + C = point
        A_eq = np.concatenate((cz.A, cz.G), axis=0)
        b_eq = np.concatenate((cz.b, point - cz.C), axis=0)

        # Step 3: Inequality constraints
        # -1 <= x <= 1
        bounds = (-1, 1)

        options = {
            'maxiter': 5,
            'disp': False
        }

        method = 'highs-ds'   # This is the fastest by testing

        # Step 4: Solve the LP
        res = linprog(c, A_eq=A_eq, b_eq=b_eq, method = method, options = options, bounds = bounds)

        return res.success


    def lt_hz(self, M: np.ndarray, hz: HybridZonotope) -> HybridZonotope:
        '''
        Computes the linear transformation of a hybrid zonotope
        A hybrid zonotope resultg from the linear transformation of a hybrid zonotope hz = (C, Gc, Gb, Ac, Ab, b)

        M @ hz = (M @ Gc, M @ Gb, M @ C, Ac, Ab, b)
        '''
        C = M @ hz.C
        Gc = M @ hz.Gc
        Gb = M @ hz.Gb
        Ac = hz.Ac
        Ab = hz.Ab
        b = hz.b

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)
    
    def intersection_hz_hz(self, hz1: HybridZonotope, hz2: HybridZonotope) -> HybridZonotope:
        '''
        Computes the intersection of two hybrid zonotopes
        
        hz1 inters hz2 = (Gc, Gb, C, Ac, Ab, b), where

        C = hz1.C
        Gc = [hz1.Gc, 0]
        Gb = [hz1.Gb, 0]
        Ac = [
            [hz1.Ac,       0],
            [     0,  hz2.Ac],
            [hz1.Gc, -hz2.Gc]
            ]
        Ab = [
            [hz1.Ab,      0],
            [     0,  hz2.Ab],
            [hz1.Gb, -hz2.Gb]
            ]
        b = [
            hz1.b,
            hz2.b,
            hz2.C - hz1.C
            ]
        '''

        C = hz1.C
        Gc = np.hstack( (hz1.Gc, np.zeros((hz1.dim, hz2.ng))) )
        Gb = np.hstack( (hz1.Gb, np.zeros((hz1.dim, hz2.nb))) )

        Ac = np.vstack((np.hstack( (hz1.Ac, np.zeros((hz1.nc, hz2.ng))) ),
                        np.hstack( (np.zeros((hz2.nc, hz1.ng)), hz2.Ac) ),
                        np.hstack( (hz1.Gc, -hz2.Gc))
                        ))
        Ab = np.vstack((np.hstack( (hz1.Ab, np.zeros((hz1.nc, hz2.nb))) ),
                        np.hstack( (np.zeros((hz2.nc, hz1.nb)), hz2.Ab) ),
                        np.hstack( (hz1.Gb, -hz2.Gb))
                        ))
        b = np.vstack((hz1.b, hz2.b, hz2.C - hz1.C))

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    def intersection_hz_R_hz(self, hz1: HybridZonotope, hz2: HybridZonotope, R: np.ndarray) -> HybridZonotope:
        '''
        Computes the generalized intersection of two hybrid zonotopes
        
        hz1 inters_R hz2 = (Gc, Gb, C, Ac, Ab, b), where

        C = hz1.C
        Gc = [hz1.Gc, 0]
        Gb = [hz1.Gb, 0]
        Ac = [
            [hz1.Ac,       0],
            [     0,  hz2.Ac],
            [R*hz1.Gc, -hz2.Gc]
            ]
        Ab = [
            [hz1.Ab,      0],
            [     0,  hz2.Ab],
            [R*hz1.Gb, -hz2.Gb]
            ]
        b = [
            hz1.b,
            hz2.b,
            hz2.C - R*hz1.C
            ]
        '''

        C = hz1.C
        Gc = np.hstack( (hz1.Gc, np.zeros((hz1.dim, hz2.ng))) )
        Gb = np.hstack( (hz1.Gb, np.zeros((hz1.dim, hz2.nb))) )

        Ac = np.vstack((np.hstack( (hz1.Ac, np.zeros((hz1.nc, hz2.ng))) ),
                        np.hstack( (np.zeros((hz2.nc, hz1.ng)), hz2.Ac) ),
                        np.hstack( (R @ hz1.Gc, -hz2.Gc))
                        ))
        Ab = np.vstack((np.hstack( (hz1.Ab, np.zeros((hz1.nc, hz2.nb))) ),
                        np.hstack( (np.zeros((hz2.nc, hz1.nb)), hz2.Ab) ),
                        np.hstack( (R @ hz1.Gb, -hz2.Gb))
                        ))
        b = np.vstack((hz1.b, hz2.b, hz2.C - R @ hz1.C))

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    def union_hz_hz(self, hz1: HybridZonotope, hz2: HybridZonotope) -> HybridZonotope:
        '''
        Computes the union of two hybrid zonotopes. This function supports disjoint sets as well
        
        For more info check the work in [3]

        TODO: Optimize the computation of the inverse matrices        
        '''

        # Step 1: Solve the set of linear equations
        I = np.linalg.inv(np.block([
            [ np.eye(hz1.dim), np.eye(hz1.dim)],
            [-np.eye(hz1.dim), np.eye(hz1.dim)]
        ]))
        X1 = I @ np.block([
                        [ hz2.Gb @ np.ones((hz2.nb, 1)) + hz1.C ],
                        [ hz1.Gb @ np.ones((hz1.nb, 1)) + hz2.C ]
                    ])
        Gb_hat = X1[0:hz1.dim, :]
        C_hat = X1[hz1.dim:, :]

        I = np.linalg.inv(np.block([
            [-np.eye(hz1.nc), np.eye(hz1.nc)],
            [ np.eye(hz1.nc), np.eye(hz1.nc)]
        ]))
        X2 = I @ np.block([
                        [ hz1.b ],
                        [ -hz1.Ab @ np.ones((hz1.nb, 1)) ]
                    ])
        Ab1_hat = X2[0:hz1.nc, :]
        b1_hat = X2[hz1.nc:, :]

        I = np.linalg.inv(np.block([
            [-np.eye(hz2.nc), np.eye(hz2.nc)],
            [ np.eye(hz2.nc), np.eye(hz2.nc)]
        ]))
        X3 = I @ np.block([
                        [ -hz2.Ab @ np.ones((hz2.nb, 1)) ],
                        [ hz2.b ]
                    ])
        Ab2_hat = X3[0:hz2.nc, :]
        b2_hat = X3[hz2.nc:, :]

        # Auxiliary variables
        n1 = 2*(hz1.ng + hz2.ng + hz1.nb + hz2.nb)


        # Step 2: Construct auxiliary matrices, Ac3 is of dimension ( nc_u - hz1.nc - hz2.nc, hz1.ng + hz2.ng )
        Ac3 = np.block([
            [     np.eye(hz1.ng, hz1.ng),   np.zeros((hz1.ng, hz2.ng))],
            [    -np.eye(hz1.ng, hz1.ng),   np.zeros((hz1.ng, hz2.ng))],
            [ np.zeros((hz2.ng, hz1.ng)),       np.eye(hz2.ng, hz2.ng)],
            [ np.zeros((hz2.ng, hz1.ng)),      -np.eye(hz2.ng, hz2.ng)],
            [ np.zeros((hz1.nb, hz1.ng)),   np.zeros((hz1.nb, hz2.ng))],
            [ np.zeros((hz1.nb, hz1.ng)),   np.zeros((hz1.nb, hz2.ng))],
            [ np.zeros((hz2.nb, hz1.ng)),   np.zeros((hz2.nb, hz2.ng))],
            [ np.zeros((hz2.nb, hz1.ng)),   np.zeros((hz2.nb, hz2.ng))]

        ])
        Ab3 = np.block([
            [ np.zeros((hz1.ng, hz1.nb)),    np.zeros((hz1.ng, hz2.nb)),     0.5*np.ones((hz1.ng, 1))],
            [ np.zeros((hz1.ng, hz1.nb)),    np.zeros((hz1.ng, hz2.nb)),     0.5*np.ones((hz1.ng, 1))],
            [ np.zeros((hz2.ng, hz1.nb)),    np.zeros((hz2.ng, hz2.nb)),    -0.5*np.ones((hz2.ng, 1))],
            [ np.zeros((hz2.ng, hz1.nb)),    np.zeros((hz2.ng, hz2.nb)),    -0.5*np.ones((hz2.ng, 1))],
            [ 0.5*np.eye(hz1.nb, hz1.nb),    np.zeros((hz1.nb, hz2.nb)),     0.5*np.ones((hz1.nb, 1))],
            [-0.5*np.eye(hz1.nb, hz1.nb),    np.zeros((hz1.nb, hz2.nb)),     0.5*np.ones((hz1.nb, 1))],
            [ np.zeros((hz2.nb, hz1.nb)),    0.5*np.eye(hz2.nb, hz2.nb),    -0.5*np.ones((hz2.nb, 1))],
            [ np.zeros((hz2.nb, hz1.nb)),   -0.5*np.eye(hz2.nb, hz2.nb),    -0.5*np.ones((hz2.nb, 1))]
        ])
        b3 = np.block([
            [0.5*np.ones((hz1.ng, 1))],
            [0.5*np.ones((hz1.ng, 1))],
            [0.5*np.ones((hz2.ng, 1))],
            [0.5*np.ones((hz2.ng, 1))],
            [    np.zeros((hz1.nb, 1))],
            [     np.ones((hz1.nb, 1))],
            [    np.zeros((hz2.nb, 1))],
            [     np.ones((hz2.nb, 1))],
        ])

        # Step 3: Construst the union of the hybrid zonotopes
        C = C_hat

        Gc = np.block([
            [hz1.Gc, hz2.Gc, np.zeros((hz1.dim, n1))],
        ])
        Gb = np.block([
            [hz1.Gb, hz2.Gb, Gb_hat]
        ])

        Ac = np.block([
            [         hz1.Ac           ,    np.zeros((hz1.nc, hz2.ng)),    np.zeros((hz1.nc, n1))],
            [np.zeros((hz2.nc, hz1.ng)),             hz2.Ac           ,    np.zeros((hz2.nc, n1))],
            [                                 Ac3                     ,         np.eye(n1, n1)   ]
        ])

        Ab = np.block([
            [          hz1.Ab          ,    np.zeros((hz1.nc, hz2.nb)),    Ab1_hat],
            [np.zeros((hz2.nc, hz1.nb)),             hz2.Ab           ,    Ab2_hat],
            [                                 Ab3                                 ]
        ])

        b = np.block([
            [b1_hat],
            [b2_hat],
            [b3]
        ])       

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    def one_step_brs_hz(self, X: HybridZonotope, T: HybridZonotope, D: np.ndarray) -> HybridZonotope:
        '''
        Computes a one-step backward reachable set using the hybrid zonotope representation

        # TODO: Is the HZ 'u' just for the inputs? If not, how can we include the admissible set of states?
        # If not, perhaps we can just compute the intersection of the BRS with the admissible set of states
        # at each iteration.
        
        X: Admissible (state X control) set
        t: Target set
        D: Dynamics matrix
            D = [A, B]

        '''

        Gc = np.block([
            [X.Gc[:T.dim, :], np.zeros((T.dim, T.ng))]
        ])

        Gb = np.block([
            [X.Gb[:T.dim, :], np.zeros((T.dim, T.nb))]
        ])

        C = X.C[:T.dim, :]

        Ac = np.block([
            [         X.Ac         ,    np.zeros((X.nc, T.ng))],
            [np.zeros((T.nc, X.ng)),            T.Ac          ],
            [      D @ X.Gc        ,           -T.Gc          ]
        ])

        Ab = np.block([
            [         X.Ab         ,    np.zeros((X.nc, T.nb))],
            [np.zeros((T.nc, X.nb)),            T.Ab          ],
            [      D @ X.Gb        ,           -T.Gb          ]
        ])

        b = np.block([
            [X.b],
            [T.b],
            [T.C - D @ X.C]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    def brs_hz(self, X: HybridZonotope, T: HybridZonotope, D: np.ndarray, N: int, visualize = False, env = None) -> HybridZonotope:
        '''
        Computes the N-step backward reachable set using the hybrid zonotope representation
        '''
        if self.visualizer is not None and visualize and env is not None:
            self.visualizer.brs_plot_settings(env)

        for i in range(N):
            T = self.one_step_brs_hz(X, T, D)
            if self.visualizer is not None and visualize:
                self.visualizer.vis_hz_brs(T)
        return T

    def decompose_hz(self, hz: HybridZonotope):
        '''
        Decomposes the input hybrid zonotope into a list of constrained zonotopes
        '''
        start_time = time.perf_counter()

        cz = [] # List that will hold all the constrained zonotopes

        # Step 1: Enumerate all possible binary combinations for the binary generators hz.nb
        b_combs = np.array(list(itertools.product([-1, 1], repeat = hz.nb)))
        
        # Iterate over all the binary combinations
        for b in b_combs:
            b = b.reshape(-1, 1)
            # Step 2.1: Create a constrained zonotope object out of the binary combination
            cz.append(ConstrainedZonotope(hz.Gc, hz.C + hz.Gb @ b, hz.Ac, hz.b - hz.Ab @ b))

        end_time = time.perf_counter()
        print(f'Decompose time  = {end_time - start_time}')

        return cz

    def is_inside_hz(self, hz: HybridZonotope, z: np.ndarray):
        '''
        Checks if a point z is inside the hybrid zonotope hz by solving the following mixed integer linear program: 
        
        min ||x_c||_inf s.t.

        np.block([
            [hz.Gc, hz.Gb],
            [hz.Ac, hz.Ab]
        ])
        @
        np.block([
            [x_c],
            [x_b]
        ])
        ==
        np.block([
            [z - hz.C],
            [hz.b]
        ])
        
        where:
            ||x_c||_inf <= 1
        and
            x_b \in {-1, 1}^hz.nb
            (that is x_b is a binary vector)
        '''
        
        ## Step 1: Create a model
        model = gp.Model('is_inside_hz')
        model.Params.OutputFlag = 0         # Disable verbose output
        model.Params.MIPFocus = 1           # Set MIPFocus = 1 (Focus more on finding feasible solutions)
        model.Params.ImproveStartTime = 0   # Set ImproveStartTime = 0 (To start focusing on finding feasible solutions immediately) (seconds)
        model.Params.SolutionLimit = 1      # Set the SolutionLimit parameter to 1 (to find only one feasible solution)

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (hz.ng, ), lb = np.array([-1] * hz.ng), ub = np.array([ 1] * hz.ng), vtype = np.array([gp.GRB.CONTINUOUS] * hz.ng), name = 'x_c')
        x_b = model.addMVar(shape = (hz.nb, ), lb = np.array([-1] * hz.nb), ub = np.array([ 1] * hz.nb), vtype = np.array([gp.GRB.INTEGER] * hz.nb), name = 'x_b')

        # Enforce that x_b only takes values in {-1, 1}^hz.nb
        for i in range(hz.nb):
            model.addConstr(x_b[i] * x_b[i] == 1 )

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        ## Step 4: Add constraints  # TODO: Check what can be done about making it into a sparse matrix
        rhs = z - hz.C                      # Right hand side of equality constraint equation
        lhs = hz.Gc @ x_c + hz.Gb @ x_b     # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)

        rhs = hz.b                          # Right hand side of equality constraint equation
        lhs = hz.Ac @ x_c + hz.Ab @ x_b     # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)
        
        model.addConstr(norm_inf == gp.norm(x_c, gp.GRB.INFINITY))  # Use the 'norm' General constraint helper function from the gurobi API

        ## Step 3: Set the objective function
        model.setObjective(norm_inf, gp.GRB.MINIMIZE)  


        ## Step 4: Solve the model
        model.optimize()

        ## Step 5: Check if the solution is feasible
        if model.status == gp.GRB.OPTIMAL or model.status == gp.GRB.SUBOPTIMAL:
            return True
        else:
            return False

    def is_inside_hz_space(self, hz: HybridZonotope, brs_settings) -> np.ndarray:
        new_points = []

        # Index 10 is the zero value for each velocity dimension
        i = 0


        for y_i in range(brs_settings.space.shape[0]):
            for x_i in range(brs_settings.space.shape[1]):
            

                # Check if the point 'p' is contained in any of the constrained zonotopes
                if brs_settings.is_already_contained_xy[y_i, x_i] == 1:
                    continue

                # close_enough = False
                # if 1 in brs_settings.is_already_contained_xy[y_i, max(0, x_i - brs_settings.max_dist_y):x_i ]:
                #     close_enough = True
                # elif 1 in brs_settings.is_already_contained_xy[y_i, x_i + 1:min(brs_settings.y_space.shape[0] - 1, x_i + brs_settings.max_dist_y + 1)]:
                #     close_enough = True
                # elif 1 in brs_settings.is_already_contained_xy[y_i + 1:min(brs_settings.x_space.shape[0] - 1, y_i + brs_settings.max_dist_x + 1), x_i]:
                #     close_enough = True
                # elif 1 in brs_settings.is_already_contained_xy[max(0, y_i - brs_settings.max_dist_x):y_i, x_i]:
                #     close_enough = True
                # else:
                #     for q in range(brs_settings.max_dist_diag):                    
                #         if brs_settings.is_already_contained_xy[ min(brs_settings.y_space.shape[0] - 1, y_i + q),   max(0, x_i - q)]:
                #             close_enough = True
                #             break 
                #         # Move top and to the right (diagonally) of the point 'p'
                #         if brs_settings.is_already_contained_xy[ min(brs_settings.y_space.shape[0] - 1, y_i + q),   min(brs_settings.x_space.shape[0] - 1, x_i + q)]:
                #             close_enough = True
                #             break
                #         # Move bottom and to the left (diagonally) of the point 'p'
                #         if brs_settings.is_already_contained_xy[ max(0, y_i - q),   max(0, x_i - q)]:
                #             close_enough = True
                #             break
                #         # Move bottom and to the right (diagonally) of the point 'p'
                #         if brs_settings.is_already_contained_xy[ max(0, y_i - q),   min(brs_settings.x_space.shape[0] - 1, x_i + q)]:
                #             close_enough = True
                #             break


                p = brs_settings.space[y_i, x_i, 0, 0]
                p = np.array([ [p[0]], [p[1]], [0.0], [0.0] ])
                # print(f'(i = {i})  p = {p.T}')
                # i += 1

                close_enough = True
                if close_enough:
                    if self.is_inside_hz(hz, p):
                        brs_settings.is_already_contained[y_i, x_i, 10, 10] = 1

                        brs_settings.is_already_contained_xy[y_i, x_i] = 1

                        # Add the point p in the list of new points
                        new_points.append(p)

        return np.array(new_points)     








    def ms_hz_hz(self, hz1: HybridZonotope, hz2: HybridZonotope) -> HybridZonotope:
        '''
        Computes the minkowski sum of two hybrid zonotopes.
        '''
        
        c = hz1.C + hz2.C

        Gc = np.block([
            hz1.Gc, hz2.Gc
        ])

        Gb = np.block([
            hz1.Gb, hz2.Gb
        ])

        Ac = np.block([
            [hz1.Ac, np.zeros((hz1.nc, hz2.ng))],
            [np.zeros((hz2.nc, hz1.ng)), hz2.Ac]
        ])

        Ab = np.block([
            [hz1.Ab, np.zeros((hz1.nc, hz2.nb))],
            [np.zeros((hz2.nc, hz1.nb)), hz2.Ab]
        ])

        b = np.block([
            [hz1.b], 
            [hz2.b]
        ])

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)

    def md_hz_z(self, hz: HybridZonotope, z: Zonotope) -> HybridZonotope:
        '''
        Computes the minkowski difference of a hybrid zonotope minuend and a zonotope subtrahend.
        '''

        hz_i = HybridZonotope(hz.Gc, hz.Gb, hz.C - z.C, hz.Ac, hz.Ab, hz.b) # hz_0
        
        for i, g in enumerate(z.G):
            hz_i = self.intersection_hz_hz(
                hz1 = HybridZonotope(hz_i.Gc, hz_i.Gb, hz_i.C + g, hz_i.Ac, hz_i.Ab, hz_i.b),
                hz2 = HybridZonotope(hz_i.Gc, hz_i.Gb, hz_i.C - g, hz_i.Ac, hz_i.Ab, hz_i.b)
            )

        return hz_i
    
    def one_step_brs_hz_v2(self, X: HybridZonotope, U: HybridZonotope, T: HybridZonotope, A: np.ndarray, B: np.ndarray) -> HybridZonotope:
        BU = self.lt_hz(-B, U)
        T_plus_BU = self.ms_hz_hz(hz1 = T, hz2 = BU)
        A_inv = np.linalg.inv(A)
        A_inv_T_W_plus_BU = self.lt_hz(A_inv, T_plus_BU)

        # Compute intersection with safe space X
        X_intersection_A_inv_T_W_plus_BU = self.intersection_hz_hz(hz1 = X, hz2 = A_inv_T_W_plus_BU)


        return X_intersection_A_inv_T_W_plus_BU

    def one_step_frs_hz(self, X: HybridZonotope, U: HybridZonotope, I: HybridZonotope, A: np.ndarray, B: np.ndarray, W: HybridZonotope) -> HybridZonotope:
        '''
        X: State space (Obstacles are not included)
        U: Input space
        I: Starting set (Initial set)
        A: System matrix
        B: Input matrix
        W: Disturbance set

        FRS = X \cap (A @ I + B @ U + W )

        '''

        A_I = self.lt_hz(A, I)                                          # A @ I
        # print(f'A_I: ng = {A_I.ng},\t nc = {A_I.nc},\t nb = {A_I.nb}')
        B_U = self.lt_hz(B, U)                                          # B @ U
        # print(f'B_I: ng = {B_U.ng},\t nc = {B_U.nc},\t nb = {B_U.nb}')
        A_I_plus_B_U = self.ms_hz_hz(hz1 = A_I, hz2 = B_U)              # A @ I + B @ U
        # print(f'A_I_plus_B_U: ng = {A_I_plus_B_U.ng},\t nc = {A_I_plus_B_U.nc},\t nb = {A_I_plus_B_U.nb}')
        A_I_plus_B_U_plus_W = self.ms_hz_hz(hz1 = A_I_plus_B_U, hz2 = W)    # A @ I + B @ U + W
        # print(f'A_I_plus_B_U_plus_W: ng = {A_I_plus_B_U_plus_W.ng},\t nc = {A_I_plus_B_U_plus_W.nc},\t nb = {A_I_plus_B_U_plus_W.nb}')


        return A_I_plus_B_U_plus_W


        # X_intersection_A_I_plus_B_U_plus_W = self.intersection_hz_hz(hz1 = X, hz2 = A_I_plus_B_U_plus_W)
        

        # return X_intersection_A_I_plus_B_U_plus_W


    def reduce_hz(self, hz: HybridZonotope) -> HybridZonotope:
        '''
        Reduces the number of continuous generators and the number
        of constraints of a Hybrid Zonotope.
        '''
        # hz = self.reduce_c_hz(hz)
        print(f'Before: size of Gc {hz.Gc.shape}\t size of Ac {hz.Ac.shape}')
        hz = self.reduce_gc_hz(hz)
        print(f'After: size of Gc {hz.Gc.shape}\t size of Ac {hz.Ac.shape}')

        return hz
    
    def reduce_c_hz(self, hz: HybridZonotope) -> HybridZonotope:
        '''
        Reduces the number of constraints of a Hybrid Zonotope.

        In this version we are removing the following constraints:
            - Any constraint whose constraint matrix component (the particular row in [Ac Ab]) is all zeros
            - Any constraint that there is another constraint that is equivalent to it
                e.g., x + y = 1 and 2x + 2y = 2, 5x + 5x = 5 only one out of these three constraints will be kept
        '''

        threshold = 1e-7
        # max_angle = 5 * np.pi / 180
        # threshold = np.cos(max_angle)

        A = np.block([hz.Ac, hz.Ab, hz.b])

        nc = A.shape[0]

        # Loop through all the columns of Gc
        i = 0; j = 0; k = 0

        while i < nc - k:
            c1 = A[i, :].T
            c1 = c1.reshape((c1.shape[0], 1))
            c1_mag = np.linalg.norm(c1)    # Magnitude of c1
            c1_unit = c1 / c1_mag           # Unit vector of c1

            # if np.abs(c1_mag) <= threshold:
            if np.abs(c1_mag) <= 0.001:
                A = np.delete(A, i, axis=0)
                k += 1
                continue

            j = 0
            while j < nc - k:
                if i == j:
                    j += 1
                    continue

                c2 = A[j, :].T
                c2 = c2.reshape((c2.shape[0], 1))
                c2_mag = np.linalg.norm(c2)     # Magnitude of c2
                c2_unit = c2 / c2_mag            # Unit vector of c2

                # dot_product = np.dot(c1.T, c2)  # Dot product between c1 and c2
                # print(f'dot_product 1: {dot_product}')
                dot_product = np.dot(c1_unit.T, c2_unit) # Dot product between c1 and c2 unit vectors
                # print(f'dot_product 2: {dot_product}')

                # if np.abs(np.abs(dot_product) - c1_mag*c2_mag) <= threshold or (c2_mag <= threshold):
                if np.abs(np.abs(dot_product) - c1_mag*c2_mag) <= threshold or (c2_mag <= 0.001):
                    A = np.delete(A, j, axis=0)   # Remove the second constraint
                    k += 1

                j += 1

            i +=1

        Ac = A[:, :hz.Ac.shape[1]]
        Ab = A[:, hz.Ac.shape[1]:-1]
        b = A[:, -1].reshape((A.shape[0], 1))

        return HybridZonotope(hz.Gc, hz.Gb, hz.C, Ac, Ab, b)

    def reduce_gc_hz(self, hz: HybridZonotope) -> HybridZonotope:
        '''
        Reduces the number of continuous generators of a Hybrid Zonotope.
        '''

        # threshold = 1e-7
        max_angle = 10 * math.pi / 180
        threshold = math.cos(max_angle)

        # Step 1: Stack Gc and Ac
        G = np.block([
            [hz.Gc],
            [hz.Ac]
        ])


        n_row = G.shape[0]; ng = G.shape[1]

        # Loop through all the rows of Gc
        i = 0; j = 0; k = 0

        while i < ng - k:
            g1 = G[:, i]
            g1_mag = np.linalg.norm(g1)    # Magnitude of g1
            g1_unit = g1 / g1_mag           # Unit vector of g1

            if np.abs(g1_mag) <= 0.001:
                G = np.delete(G, i, axis=1)
                k += 1
                continue

            j = 0
            while j < ng - k:
                if i == j:
                    j += 1
                    continue

                g2 = G[:, j]
                g2_mag = np.linalg.norm(g2)     # Magnitude of g2
                g2_unit = g2 / g2_mag            # Unit vector of g2

                dot_product = np.dot(g1_unit.T, g2_unit) # Dot product between g1 and g2 unit vectors

                if np.abs(np.abs(dot_product) - g1_mag*g2_mag) <= threshold or (g2_mag <= 0.001):                
                    G = np.delete(G, j, axis=1)   # Remove the second generator
                    k += 1

                j += 1

            i +=1

        Gc = G[:hz.dim, :]
        Ac = G[hz.dim:, :]

        return HybridZonotope(Gc, hz.Gb, hz.C, Ac, hz.Ab, hz.b)






class TreeOperations:

    op = ZonoOperations()

    def __init__(self):
        pass

    def attach_trees(self, connection, tree1, tree2, N = 1):
        '''
        Given multiple trees, attach them
        '''

        # TODO: Think of how to handle the connections between trees that do not have a new common root (e.g., Until operation)
        
        if connection == 'OR':
            return self.OR(tree1, tree2)
        elif connection == 'AND':
            return self.AND(tree1, tree2)
        elif connection == 'UNTIL':
            return self.UNTIL(tree1, tree2, N)

    def OR(self, tree1, tree2):        
        # Step 1: Find root node of each tree
        root1 = tree1.root; root2 = tree2.root

        # Step 2: Compute the union of the hybrid zonotopes of the root nodes
        union = self.op.union_hz_hz(root1.set, root2.set)

        # Step 3: Construct the new root node
        root = set_node(union, f'{root1} {UNION_SYMBOL} {root2}')

        # Step 4: Set the root1 and root2 flags to False
        root1.is_root = False; root2.is_root = False

        # Step 5: Create an OR node
        operation = OR(root, root1, root2)

        # Step 6: Construct the new tree
        nodes = [root, operation] + list(tree1.nodes) + list(tree2.nodes)
        return Tree(*nodes)

    def AND(self, tree1, tree2):
        # Step 1: Find root node of each tree
        root1 = tree1.root; root2 = tree2.root

        # Step 2: Compute the union of the hybrid zonotopes of the root nodes
        union = self.op.intersection_hz_hz(root1.set, root2.set)

        # Step 3: Construct the new root node
        root = set_node(union, f'{root1} {INTERSECTION_SYMBOL} {root2}')

        # Step 4: Set the root1 and root2 flags to False
        root1.is_root = False; root2.is_root = False

        # Step 5: Create an OR node
        operation = AND(root, root1, root2)

        # Step 6: Construct the new tree
        nodes = [root, operation] + list(tree1.nodes) + list(tree2.nodes)
        return Tree(*nodes)

    def UNTIL(self, tree1, tree2, N = 1):

        # TODO: Step 0: Temporarily hardcode the model
        # Dynamic Model
        A = np.array([
            [1.0, 0.0],
            [0.0, 1.0]
        ])

        B = np.array([
            [1.0, 1.0],
            [0.0, 1.0]
        ])

        D = np.block([A, B])

        # Step 1: Find root node of each tree
        root1 = tree1.root; root2 = tree2.root

        # Step 2: Find the leaves of the first tree
        leaves1 = tree1.leaves

        # Step 3: For each leaf, compute the BRS of the hybrid zonotopes of the leaf and the root of the second tree
        # Then replace the leaf node with the new node containing the BRS.
        # Then attach to the new node, the entire tree2
        for leaf in leaves1:
            # Compute the BRS
            brs = self.op.brs_hz(X = leaf.set, D = D, T = root2.set, N = N)

            # Construct the new node
            new_node = set_node(brs, f'{REACH_SYMBOL}({leaf}, {root2})')

            # Replace the leaf with the new node
            tree1.replace_node(leaf, new_node)


            # Create a copy of tree2
            tree2_copy = deepcopy(tree2)

            # Replace the label of each node of tree2 by just adding _tree2_{leaf_name} to the end of it
            for node in tree2_copy.nodes:
                node.extra_name = f'_tree_{leaf.name}'

            # Attach to the new node, a node with the UNTIL operation
            operation = UNTIL(new_node, tree2_copy.root)

            # Step 4: Construct the new tree
            nodes = list(tree1.nodes) + list(tree2_copy.nodes) + [operation]

            # Step 5: Create the new tree
            new_tree1 = Tree(*nodes)


        # Step 6: Pass the new_tree1 and tree2 to the OR operation
        return self.OR(new_tree1, tree2)
    








