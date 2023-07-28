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

import numpy as np
import sympy as sympy
import math
from scipy.optimize import linprog
from copy import deepcopy
import itertools
import time

# For mixed-integer linear programming
import gurobipy as gp
from gurobipy import *

from utils.sets.zonotopes import Zonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.tlt.nodes import OR, AND, UNTIL, set_node
from utils.tlt.tree import Tree


'''

References:

[1] J. Scott, et al.        'Constrained zonotopes: A new tool for set-based estimation and fault detection'
[2] L. Yang, et al.         'Efficient Backward Reachability Using the Minkowski Difference of Constrained Zonotopes'
[3] T. Bird, et al.         'Unions and Complements of Hybrid Zonotopes'
[4] L. Yang, et al.         'Scalable Zonotopic Under-approximation of Backward Reachable Sets for Uncertain Lienar Systems'
[5] Y. Zhang, et al.        'Reachability Analysis and Safety Verification of Neural Feedback Systems via Hybrid Zonotopes'
[6] V. Raghuraman, et al.   'Set Operations and Order reductions for constrained zonotopes'
[7] T. Bird                 'PhD Dissertation: Hybrid Zonotopes: A Mixed-Integer Set Representation For The Analysis Of Hybrid Zonotopes
'''

class ZonoOperations:
    def __init__(self, visualizer = None):

        self.visualizer = visualizer

    ###############################################################################################################
    # Zonotope Methods

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

    def redundant_g_z(self, z: Zonotope) -> Zonotope:
        '''
        Removes redundant generators from a Zonotope.

        This method scans all generators and whenver it finds a pair of generators that are
        parallel to each other, it adds one to the other and removes the other one.

        Example: If we have two generators g1 and g2, and g1 = 2*g2,
        then we update g1 as g1 = g1 + g2 = 3*g2 and remove g2. 
        '''

        max_angle = 0.05 * math.pi / 180
        threshold = 1 - math.sin(max_angle)

        # Compute norm of all columns
        norms = np.linalg.norm(z.G, axis = 0)
        # Find all indices of 'norms' that are zero
        zeros = np.where(norms == 0)[0]
        # Remove all 'zero' norm columngs from G
        G = np.delete(arr = z.G, obj = zeros, axis = 1)

        ng = G.shape[1]

        # Loop through all the rows of G
        i = 0; j = 0; k = 0

        while i < ng - k:
            g1_unit = G[:, i] / np.linalg.norm(G[:, i]) # Unit vector of g1

            j = i + 1
            while j < ng - k:

                g2 = G[:, j]
                g2_unit = g2 / np.linalg.norm(g2)       # Unit vector of g2

                if np.abs(np.dot(g1_unit.T, g2_unit)) >= threshold:
                    G[:, i] = G[:, i] + g2
                    G = np.delete(G, j, axis=1)   # Remove the second generator
                    k += 1
                else:
                    j += 1

            i +=1

        return Zonotope(z.C, G)

    def z_to_hz(self, z: Zonotope) -> HybridZonotope:
        '''
        This method takes in a zonotope and returns an equivalent representation as a Hybrid Zonotope
        '''   
        Gc = z.G
        Gb = np.zeros((z.dim, 0))
        C = z.C
        Ac = np.zeros((0, z.ng))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    def z_to_cz(self, z: Zonotope) -> ConstrainedZonotope:
        '''
        This method takes in a zonotope and returns an equivalent representation as a Constrained Zonotope
        '''   
        G = z.G
        C = z.C
        A = np.zeros((0, z.ng))
        b = np.zeros((0, 1))

        return ConstrainedZonotope(G, C, A, b)

    def oa_z_to_hypercube(self, z: Zonotope) -> Zonotope:
        '''
        This method over-approximates an n-dimensional zonotope as an n-dimensional hypercube.
        '''
        row_sums = np.sum(np.abs(z.G), axis=1)
        G = np.diag(row_sums)

        return Zonotope(z.C, G)
        
    def is_inside_z(self, z: Zonotope, p: np.ndarray):
       
        ## Step 1: Create a model
        model = gp.Model('is_inside_z')
        model.Params.OutputFlag = 0         # Disable verbose output
        model.Params.MIPFocus = 1           # Set MIPFocus = 1 (Focus more on finding feasible solutions)
        model.Params.ImproveStartTime = 0   # Set ImproveStartTime = 0 (To start focusing on finding feasible solutions immediately) (seconds)
        model.Params.SolutionLimit = 1      # Set the SolutionLimit parameter to 1 (to find only one feasible solution)

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (z.ng, ), lb = np.array([-1] * z.ng), ub = np.array([ 1] * z.ng), vtype = np.array([gp.GRB.CONTINUOUS] * z.ng), name = 'x_c')

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        ## Step 4: Add constraints  # TODO: Check what can be done about making it into a sparse matrix
        rhs = p - z.C                      # Right hand side of equality constraint equation
        lhs = z.G @ x_c     # Left hand side of equality constraint equation
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

    def is_inside_z_v2(self, z, p):

        G = z.G
        C = z.C
        
        c = np.zeros(G.shape[1])  # Objective function coefficients
        bounds = [(-1, 1)] * G.shape[1]  # Variable bounds

        # Define the equality constraints
        A_eq = G
        b_eq = p - C

        # Solve the linear program
        result = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds)

        if result.success:
            return True
        else:
            return False     

    def is_empty_z(self, z: Zonotope) -> bool:
        ## Step 1: Create a model
        model = gp.Model('is_empty_z')
        model.Params.OutputFlag = 0         # Disable verbose output
        model.Params.MIPFocus = 1           # Set MIPFocus = 1 (Focus more on finding feasible solutions)
        model.Params.ImproveStartTime = 0   # Set ImproveStartTime = 0 (To start focusing on finding feasible solutions immediately) (seconds)
        model.Params.SolutionLimit = 1      # Set the SolutionLimit parameter to 1 (to find only one feasible solution)

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (z.ng, ), lb = np.array([-1] * z.ng), ub = np.array([ 1] * z.ng), vtype = np.array([gp.GRB.CONTINUOUS] * z.ng), name = 'x_c')

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        ## Step 3: Set the objective function
        model.setObjective(norm_inf, gp.GRB.MINIMIZE)  

        ## Step 4: Solve the model
        model.optimize()

        ## Step 5: Check if the solution is feasible
        if model.status == gp.GRB.OPTIMAL or model.status == gp.GRB.SUBOPTIMAL:
            return False
        else:
            return True          
    

    ###############################################################################################################
    # Constrained Zonotope Methods

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
                                    whereas cz1.dimension = {cz1.dim} and cz2.dimension = {cz2.dim}.'
        
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

    def is_inside_cz_v2(self, cz: ConstrainedZonotope, z: np.ndarray):
        '''
        Checks if a point z is inside the constrained zonotope cz by solving the following linear program: 

        See work in [1] for details
        '''

        ## Step 1: Create a model
        model = gp.Model('is_inside_cz')
        model.Params.OutputFlag = 0         # Disable verbose output
        model.Params.MIPFocus = 1           # Set MIPFocus = 1 (Focus more on finding feasible solutions)
        model.Params.ImproveStartTime = 0   # Set ImproveStartTime = 0 (To start focusing on finding feasible solutions immediately) (seconds)
        model.Params.SolutionLimit = 1      # Set the SolutionLimit parameter to 1 (to find only one feasible solution)

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (cz.ng, ), lb = np.array([-1] * cz.ng), ub = np.array([1] * cz.ng), vtype = np.array([gp.GRB.CONTINUOUS] * cz.ng), name = 'x_c')

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        ## Step 4: Add constraints  # TODO: Check what can be done about making it into a sparse matrix
        rhs = z - cz.C                      # Right hand side of equality constraint equation
        lhs = cz.G @ x_c                    # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)

        rhs = cz.b                          # Right hand side of equality constraint equation
        lhs = cz.A @ x_c                    # Left hand side of equality constraint equation
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
    
    def is_empty_cz(self, cz: ConstrainedZonotope):
        '''
        Checks if the constrained zonotope cz is empty

        See work in [1] for details
        '''
        
        ## Step 1: Create a model
        model = gp.Model('is_empty_cz')
        model.Params.OutputFlag = 0         # Disable verbose output

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (cz.ng, ), lb = np.array([-1] * cz.ng), ub = np.array([1] * cz.ng), vtype = np.array([gp.GRB.CONTINUOUS] * cz.ng), name = 'x_c')

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        rhs = cz.b                          # Right hand side of equality constraint equation
        lhs = cz.A @ x_c                    # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)
        
        model.addConstr(norm_inf == gp.norm(x_c, gp.GRB.INFINITY))  # Use the 'norm' General constraint helper function from the gurobi API

        ## Step 3: Set the objective function
        model.setObjective(norm_inf, gp.GRB.MINIMIZE)

        ## Step 4: Solve the model
        model.optimize()

        ## Step 5: Check if the solution is feasible
        if model.status == gp.GRB.OPTIMAL:
            return False
        else:
            return True    
    
    def complement_cz_to_hz(self, cz: ConstrainedZonotope) -> HybridZonotope:
        '''
        This method computes the complement of a Constrained Zonotope cz inside space X represented as a Constrained Zonotope.

        See work in [6] for details

        The complement is represented as a Hybrid Zonotope
        
        Gf1     :   (2*n_{g,z}, 2*n_{g,z})

        Cf1     :   (2*n_{g,z}, 1)

        Gf2     :   (4*n_{g,z}, 4*n_{g,z})

        Cf2     :   (4*n_{g,z}, 1)

        AcPF    :   [ (n_{g,z}, n_{g,z}), (n_{g,z}, 1), (n_{g,z}, n_{z} + n_{c,z} + 2*n_{g,z}) ]
                    [ (n_{g,z}, n_{g,z}), (n_{g,z}, 1), (n_{g,z}, n_{z} + n_{c,z} + 2*n_{g,z}) ]

        AcDF    :   [ (n_{g,z}, n_{g,z} + 1), (n_{g,z}, n_{z} + n_{c,z}), (n_{g,z}, n_{g,z}), (n_{g,z}, n_{g,z}) ]
                    [ (1, n_{g,z} + 1)      , (1, n_{z} + n_{c,z})      , (1, n_{g,z})      , (1, n_{g,z})       ]

        bDF     :   [ (n_{g,z}, 1) ]
                    [ (1, 1)       ]

        AcCS    :   [ (n_{g,z}, n_{g,z}), (n_{g,z}, 1), (n_{g,z}, n_{z} + n_{c,z}), (n_{g,z}, n_{g,z}), (n_{g,z}, n_{g,z}) ]
                    [ (n_{g,z}, n_{g,z}), (n_{g,z}, 1), (n_{g,z}, n_{z} + n_{c,z}), (n_{g,z}, n_{g,z}), (n_{g,z}, n_{g,z}) ]
                    [ (n_{g,z}, n_{g,z}), (n_{g,z}, 1), (n_{g,z}, n_{z} + n_{c,z}), (n_{g,z}, n_{g,z}), (n_{g,z}, n_{g,z}) ]
                    [ (n_{g,z}, n_{g,z}), (n_{g,z}, 1), (n_{g,z}, n_{z} + n_{c,z}), (n_{g,z}, n_{g,z}), (n_{g,z}, n_{g,z}) ]

        AbCS    :   [ (n_{g,z}, n_{g,z}), (n_{g,z}, n_{g,z}) ]
                    [ (n_{g,z}, n_{g,z}), (n_{g,z}, n_{g,z}) ]
                    [ (n_{g,z}, n_{g,z}), (n_{g,z}, n_{g,z}) ]
                    [ (n_{g,z}, n_{g,z}), (n_{g,z}, n_{g,z}) ]

        '''
        G = cz.G; C = cz.C; A = cz.A; b = cz.b
        ng = cz.ng; nc = cz.nc; n = cz.dim
        ng_ng_zero = np.zeros((ng, ng))
        ng_1_zero  = np.zeros((ng, 1))
        ng_ng_eye  = np.eye(ng)
        ng_1_ones  = np.ones((ng, 1))
        ng_nnc_zero = np.zeros((ng, n + nc))
        ng_nncngng_zero = np.zeros((ng, n + nc + 2*ng))


        dm = 50 # TODO
        lm = 50 # TODO
        m = dm + 1


        AcPF = np.block([
            [ m*ng_ng_eye, -(dm/2)*ng_1_ones, ng_nncngng_zero],
            [-m*ng_ng_eye, -(dm/2)*ng_1_ones, ng_nncngng_zero]])

        AcDF = np.block([
            [       ng_ng_zero, ng_1_zero, lm*np.block([G.T, A.T]),        0.5*ng_ng_eye,       -0.5*ng_ng_eye],
            [np.zeros((1, ng)),         0,   np.zeros((1, n + nc)), 0.5*np.ones((1, ng)), 0.5*np.ones((1, ng))]])
        
        bDF = np.block([
            [ng_1_zero],
            [1 - ng]])

        AcCS = np.block([
            [-m*ng_ng_eye, (dm/2)*ng_1_ones, ng_nnc_zero, ng_ng_zero, ng_ng_zero],
            [ m*ng_ng_eye, (dm/2)*ng_1_ones, ng_nnc_zero, ng_ng_zero, ng_ng_zero],
            [  ng_ng_zero,        ng_1_zero, ng_nnc_zero,  ng_ng_eye, ng_ng_zero],
            [  ng_ng_zero,        ng_1_zero, ng_nnc_zero, ng_ng_zero,  ng_ng_eye]])

        AbCS = np.block([
            [m*ng_ng_eye,  ng_ng_zero],
            [ ng_ng_zero, m*ng_ng_eye],
            [ -ng_ng_eye,  ng_ng_zero],
            [ ng_ng_zero,  -ng_ng_eye]])

        cf1 = np.zeros((2*ng, 1))
        Gf1 = (m + dm/2) * np.eye(2*ng)

        cf2 = np.block([
            [-(dm + 1)*np.ones((2*ng, 1))],
            [-np.ones((2*ng, 1))]
        ])
        Gf2 = np.block([
            [((3*dm/2) + 1)*np.eye(2*ng), np.zeros((2*ng, 2*ng))],
            [np.zeros((2*ng, 2*ng))     , np.eye(2*ng)          ]
        ])


        Gc = np.block([m*G, np.zeros((n, 1+n+nc+ng+ng+2*ng+4*ng))])
        Gb = np.zeros((n, ng+ng))
        
        Ac = np.block([
            [m*A, np.zeros((nc, 1+n+nc+ng+ng+2*ng+4*ng))],
            [AcPF, Gf1, np.zeros((2*ng, 4*ng))],
            [AcDF, np.zeros((ng+1, 2*ng)), np.zeros((ng+1, 4*ng))],
            [AcCS, np.zeros((4*ng, 2*ng)), Gf2]
        ])

        Ab = np.block([
            [np.zeros((nc+2*ng + ng + 1, 2*ng))],
            [AbCS]
        ])

        b = np.block([
            [b],
            [cf1],
            [bDF],
            [cf2]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)
    
    def oa_cz_to_z(self, cz: ConstrainedZonotope) -> Zonotope:
        '''
        This method takes in a constrained zonotope and returns a zonotope
        over-approximating the constrained zonotope.
        '''   
        G = cz.G
        C = cz.C

        return Zonotope(C, G)

    def oa_lifted_cz_to_hypercube(self, cz: ConstrainedZonotope) -> ConstrainedZonotope:
        '''
        This method provides an over-approximation of a constrained zonotope as a hypercube.
        However, this over-approximation is not the tightest hypercube over-approximation,
        since it neglects constraints of the constrained zonotope.
        '''
        dim = cz.dim
        z = self.lifted_cz(cz)          # Compute  lifted_cz
        z = self.oa_z_to_hypercube(z)   # Compute the hypercube over-approximation of the lifted cz
        cz = self.z_to_cz(z)            # Express the hypercube as a constrained zonotope

        G = cz.G[:dim, :]
        C = cz.C[:dim, :]

        z = Zonotope(C, G)
        z = self.redundant_g_z(z)
        cz = self.z_to_cz(z)

        return cz

    def cz_to_hz(self, cz: ConstrainedZonotope) -> HybridZonotope:
        '''
        This method takes in a constrained zonotope and returns an equivalent representation as a Hybrid Zonotope
        '''   
        Gc = cz.G
        Gb = np.zeros((cz.dim, 0))
        C = cz.C
        Ac = cz.A
        Ab = np.zeros((cz.nc, 0))
        b = cz.b

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    def lifted_cz(self, cz: ConstrainedZonotope) -> Zonotope:
        '''
        This method returns the lifted constrained zonotope
        '''
        G = np.block([[cz.G], [cz.A]])
        C = np.block([[cz.C], [-cz.b]])

        return Zonotope(C, G)

    def redundant_g_cz(self, cz: ConstrainedZonotope) -> ConstrainedZonotope:
        '''
        Removes redundant generators from a Constrained Zonotope.

        This method first forms the lifted constrained zonotope. Then it
        scans all generators and whenver it finds a pair of generators that are
        parallel to each other, it adds one to the other and removes the other one.

        Example: If we have two generators g1 and g2, and g1 = 2*g2,
        then we update g1 as g1 = g1 + g2 = 3*g2 and remove g2. 
        '''

        max_angle = 0.05 * math.pi / 180
        threshold = 1 - math.sin(max_angle)

        # Step 1: Stack Gc and Ac
        G = np.block([
            [cz.G],
            [cz.A]
        ])

        norms = np.linalg.norm(G, axis = 0)             # Compute norm of all columns
        zeros = np.where(norms == 0)[0]                 # Find all indices of 'norms' that are zero
        G = np.delete(arr = G, obj = zeros, axis = 1)   # Remove all 'zero' norm columngs from G
        ng = G.shape[1]

        i = 0; j = 0; k = 0
        while i < ng - k:
            g1_unit = G[:, i] / np.linalg.norm(G[:, i]) # Unit vector of g1

            j = i + 1
            while j < ng - k:

                g2 = G[:, j]
                g2_unit = g2 / np.linalg.norm(g2)       # Unit vector of g2

                if np.abs(np.dot(g1_unit.T, g2_unit)) >= threshold:
                    G[:, i] = G[:, i] + g2
                    G = np.delete(G, j, axis=1)         # Remove the second generator
                    k += 1
                else:
                    j += 1

            i +=1

        return ConstrainedZonotope(G[:cz.dim, :], cz.C, G[cz.dim:, :], cz.b)

    def redundant_c_cz(self, cz: ConstrainedZonotope) -> ConstrainedZonotope:
        '''
        Reduces the number of constraints of a Constrained Zonotope.

        In this method we are removing the following constraints:
            - Any constraint whose constraint matrix component (the particular row in Ac) is all zeros
            - Any constraint that there is another constraint that is equivalent to it
                e.g., x + y = 1 and 2x + 2y = 2, 5x + 5x = 5 only one out of these three constraints will be kept
        '''
        max_angle = 0.05 * math.pi / 180
        threshold = 1 - math.sin(max_angle)

        A = np.block([cz.A, cz.b])

        norms = np.linalg.norm(A, axis = 1)             # Compute norm of all rows
        zeros = np.where(norms == 0)[0]                 # Find all indices of 'norms' that are zero
        A = np.delete(arr = A, obj = zeros, axis = 0)   # Remove all 'zero' norm columngs from G
        nc = A.shape[0]

        i = 0; j = 0; k = 0
        while i < nc - k:
            c1_unit = A[i, :] / np.linalg.norm(A[i, :]) # Unit vector of c1

            j = i + 1
            while j < nc - k:
                c2 = A[j, :]
                c2_unit = c2 / np.linalg.norm(c2)       # Unit vector of c2

                if np.abs(np.dot(c1_unit.T, c2_unit)) >= threshold:
                    A = np.delete(A, j, axis=0)         # Remove the second constraint
                    k += 1
                else:
                    j += 1

            i +=1

        return ConstrainedZonotope(cz.G, cz.C, A[:, :cz.A.shape[1]], A[:, -1].reshape((A.shape[0], 1)))

    def redundant_c_g_cz(self, cz: ConstrainedZonotope, options = 'slow') -> ConstrainedZonotope:
        '''
        This method removes redundant constraints and generators from a Constrained Zonotope using the method described in [6]
        '''
        epsilon = 1e-3
        redundant = True

        cz = self.redundant_c_cz(cz)   # Remove trivially redundant constraints
        cz = self.redundant_g_cz(cz)   # Remove trivially redundant generators

        while redundant:
            redundant = False
            cz = self.rref_cz(cz)
            if options == 'fast':
                E = self.find_E_cz_fast(cz)
            else:
                E = self.find_E_cz_slow(cz)
                if E is None: # No solution found
                    continue

            for c in range (cz.ng):
                for r in range(cz.nc):
                    if np.abs(cz.A[r, c]) >= epsilon and self.is_inside_interval(E[c], np.array([-1, 1])):
                        a_rc_inv = (1/cz.A[r, c])
                        R_rc = np.array([a_rc_inv*cz.b[r,0], a_rc_inv*cz.b[r,0]])
                        temp = np.array([0.0, 0.0])
                        for k in range(cz.ng):
                            if k != c:
                                temp = self.interval_add(temp, self.interval_scalar_mul(cz.A[r, k], E[k,:]))
                        temp = self.interval_scalar_mul(a_rc_inv, temp)
                        R_rc = self.interval_sub(R_rc, temp)
                        
                        if self.is_inside_interval(R_rc, np.array([-1, 1])):                            
                            cz = self.remove_c_g_cz(cz = cz, c = c, r = r)
                            redundant = True
                            break
                if redundant:
                    break

        return cz

    def find_E_cz_slow(self, cz):
        '''
        This method finds the E interval by solving 2*ng LPs as described in [1]
        This method provides the exact E bounds but it is generally more computationally expensive
        '''
        
        ## Step 1: Create a model
        model = gp.Model('intervals_cz')
        model.Params.OutputFlag = 0         # Disable verbose output

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (cz.ng, ), lb = np.array([-1] * cz.ng), ub = np.array([1] * cz.ng), vtype = np.array([gp.GRB.CONTINUOUS] * cz.ng), name = 'x_c')

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        ## Step 3: Add constraints
        rhs = cz.b                          # Right hand side of equality constraint equation
        lhs = cz.A @ x_c                    # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)
        
        model.addConstr(norm_inf == gp.norm(x_c, gp.GRB.INFINITY))  # Use the 'norm' General constraint helper function from the gurobi API

        x_L = []
        for g in range(cz.ng):
            model.setObjective(x_c[g], gp.GRB.MINIMIZE)
            model.optimize()
            if model.status == gp.GRB.OPTIMAL:
                x_L.append(x_c[g].X)
            else:
                return None

        x_U = []
        for g in range(cz.ng):
            model.setObjective(x_c[g], gp.GRB.MAXIMIZE)
            model.optimize()
            if model.status == gp.GRB.OPTIMAL:
                x_U.append(x_c[g].X)
            else:
                return None
              
        x_U = np.array(x_U); x_L = np.array(x_L)

        E = np.block([x_L.reshape(-1, 1), x_U.reshape(-1, 1)])

        return E

    def find_E_cz_slow_v2(self, cz):
        '''
        This method finds the E interval by solving 2*ng LPs as described in [1]
        This method provides the exact E bounds but it is generally more computationally expensive
        '''
        
        ## Step 1: Create a model
        model = gp.Model('intervals_cz')
        model.Params.OutputFlag = 0         # Disable verbose output

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (cz.ng, ), lb = np.array([-1] * cz.ng), ub = np.array([1] * cz.ng), vtype = np.array([gp.GRB.CONTINUOUS] * cz.ng), name = 'x_c')

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        ## Step 3: Add constraints
        rhs = cz.b                          # Right hand side of equality constraint equation
        lhs = cz.A @ x_c                    # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)
        
        model.addConstr(norm_inf == gp.norm(x_c, gp.GRB.INFINITY))  # Use the 'norm' General constraint helper function from the gurobi API

        x_L = []
        for g in range(cz.ng):
            model.setObjective(x_c[g], gp.GRB.MINIMIZE)
            model.optimize()
            x_L.append(x_c[g].X)

        x_U = []
        for g in range(cz.ng):
            model.setObjective(x_c[g], gp.GRB.MAXIMIZE)
            model.optimize()
            x_U.append(x_c[g].X)

        x_U = np.array(x_U); x_L = np.array(x_L)

        E = np.block([x_L.reshape(-1, 1), x_U.reshape(-1, 1)])

        return E

    def find_E_cz_fast(self, cz):
        '''
        This method computes the Intervals of the input constrained zonotope according to Algorithm 1 in [6]
        This method does not provide the exact E bounds but it is generally more computationally efficient
        '''

        # Step 1: Initialize intervals Ej and Rj as Ej <- [-1, 1], Rj <- [-inf, inf], i,j <- 1
        E = np.array([ [-1, 1] for g in range(cz.ng) ])
        R = np.array([ [-np.inf, np.inf] for g in range(cz.ng) ])

        A = cz.A
        epsilon = 1e-5
        iterations = 50  # Maximum number of iterations

        for iter in range(iterations):
            for i in range(cz.nc):
                for j in range(cz.ng):
                    if abs(A[i, j]) >= epsilon:
                        ## V2
                        temp = np.array([0.0, 0.0])
                        R_rc = np.array([cz.b[i,0] / A[i, j], cz.b[i,0] / A[i, j]])
                        for k in range(cz.ng):
                            if k != j:
                                aij_ak = cz.A[i, k] / A[i, j]
                                temp = self.interval_add(temp, self.interval_scalar_mul(aij_ak, E[k,:]))
                        R_rc = self.interval_sub(R_rc, temp)
                        R[j] = self.intesection_intervals(R[j], R_rc)
                        E[j] = self.intesection_intervals(E[j], R[j])
        return E

    def rref_cz(self, cz):
        '''
        This method computes the reduced row echelon form of matrix [A | b] for a linear system of equations (Ax = b)
        using Gauss-Jordan Elimination with full pivoting.

        - FUTURE IMPROVEMENTS: Experiment with other preconditioning strategies
        '''
        A = np.block([cz.A, cz.b])

        rows = A.shape[0]

        pivots = []
        for r in range(rows):
            # Find the pivot row and column
            pivot, pivots = self.find_pivot(A[r, :-1], pivots)

            # Check if there is a new pivot
            if len(pivots) < r + 1:
                continue
            
            # Normalize the pivot row to turn the pivot element into 1
            A[r, :] = A[r, :] / A[r, pivot]

            # Turn the elements of all other rows in the pivot column to zero
            for r2 in range(rows):
                if r2 != r:
                    A[r2, :] = A[r2, :] - A[r2, pivot] * A[r, :]

        b = A[:, -1].reshape((A.shape[0], 1))
        A = A[:, :-1]

        return ConstrainedZonotope(cz.G, cz.C, A, b)

    def find_pivot(self, row, pivots):
        ''' 
        This method finds the pivot row and column for the Gauss-Jordan Elimination
        '''

        abs_row = np.abs(row)
        
        pivot = 0   # Init with dummy value
        
        for c in range(abs_row.shape[0]):
            if abs_row[c] != 0 and c not in pivots:
                pivot = c
                pivots.append(c)
                break

        return pivot, pivots

    def remove_c_g_cz(self, cz, c, r):
        '''
        This method removes the 'c' constraint and 'r' generator from a constrained zonotope

        c: constraint index
        r: generator index
        '''
        Ecr = np.zeros((cz.ng, cz.nc))
        Ecr[c, r] = 1

        Lg = cz.G @ Ecr * (1/cz.A[r, c])
        La = cz.A @ Ecr * (1/cz.A[r, c])

        G = cz.G - Lg @ cz.A
        C = cz.C + Lg @ cz.b
        A = cz.A - La @ cz.A
        b = cz.b - La @ cz.b

        G = np.delete(G, c, 1)
        A = np.delete(A, c, 1)
        A = np.delete(A, r, 0)
        b = np.delete(b, r, 0)

        return ConstrainedZonotope(G, C, A, b)   

    def oa_cz_vertex_enumeration(self, cz: ConstrainedZonotope) -> ConstrainedZonotope:
        '''
        This method over-approximates a constrained zonotope.

        To do that it first performs vertex enumeration of the constrained zonotope.
        Then, it defines a generator for each edge of the convex hull of the vertices.
        Finally, it scales down the generators until at least one of the vertices is not inside the new constrained zonotope.

        This method has the potential to perform a very tight over-approximation of the constrained zonotope, however,
        to do is it would require a lot of computation time, both because of the vertex enumaration, but also because it would
        require iterating between scaling and checking if the vertices are inside the new constrained zonotope.
        The latter is computationally expensive because it requires solving an LP for each vertex.
        '''
        v, _, failed = cz.g2v()
        if failed:
            return cz

        # Step 2: Compute the vertex centroid of the convex hull of V
        # c = np.mean(v, axis = 0).reshape(-1, 1)
        cx = (np.max(v[:, 0]) + np.min(v[:, 0])) / 2; cy = (np.max(v[:, 1]) + np.min(v[:, 1])) / 2
        c = np.array([cx, cy]).reshape(-1, 1)
        
        # Step 3: Compute the midpoint between all consecutive vertices
        midpoints = []
        for i in range(v.shape[0]):
            midpoints.append( (v[i, :] + v[(i+1)%v.shape[0], :]) / 2 )
        midpoints = np.array(midpoints)

        # Step 4: Define the new generators where a generator is the vector starting from 'c' and ends at a midpoint
        G = np.zeros((c.shape[0], midpoints.shape[0]))
        for i in range(midpoints.shape[0]):
            is_inside = True
            G[:, i] = midpoints[i, :] - c.reshape(-1)

            z = Zonotope(c, G[:, :i])

            if i == 0:
                continue

            for vertex in v:
                p = np.array([ [vertex[0]], [vertex[1]] ])
                if not self.is_inside_z_v2(z, p):
                    is_inside = False
                    break
                
            if is_inside:
                break
        # Step 5: Keep on scaling down the generators until at least one of the vertices is not inside the new constrained zonotope
        # step = 0.1
        step = 0.01
        n = 0
        while True:
            scale = 1 - n*step
            z = Zonotope(c, scale * G)
            for vertex in v:
                p = np.array([ [vertex[0]], [vertex[1]] ])
                if not self.is_inside_z_v2(z, p):
                    return ConstrainedZonotope((1 - (n-1)*step) * G, c, np.zeros((0, G.shape[1])), np.zeros((0, 1)))
            n += 1

    def oa_cz_to_hypercube_tight(self, cz: ConstrainedZonotope) -> ConstrainedZonotope:
        '''
        This method is specifically designed for constrained zonotopes that represent very simple sets, however with a lot of redundancy 

        To set the theme, this is gonna be used for the FRS of moving obstacles in the environment.
        More precisely, given that the obstacle is a moving car and we initially represent it as a simple rectangle, we then wanna compute
        its forward reachable set and the RCI for each FRS. Then we wanna compute its complement to see which set of states are actually safe
        for the ego vehicle. Since we only have at our disposal a formula for the computation of the complement of a constrained zonotope
        but not for a hybrid zonotope, we wanna convert that hybrid zonotope into a constrained zonotope.

        Generally, this is gonna result in an over-approximation. However, even that over-approximation contains a lot of redundant information
        because all we do (at the moment) to generate that constrained zonotope is to simply use the method 'oa_hz_to_cz'.

        Therefore, we need to take extra steps to reduce the complexity of that CZ. This method does exactly that, that is it takes care of that
        second step of over-approximation to significantly reduce the complexity of the CZ. The goal will be to generate a hypercube over-approximation        
        
        In this version we:

        Assume that the dimensionality of the space is 'n'
        Define a hypercube of dimensionality 'n' as a zonotope where each side of the hypercube is equal to the max and min of the
        constrained zonotope when all of its constraints are ignored. This applies for all dimensions apart from the one that will be examined at each time.
        For the dimension that is examined just define its size to be the same with the step size (This is gonna be clearer later on)
        
        Then for each dimension we compute the intersection of the original cosntrained zonotope with the hypercube with its center shifted by a single step.

        Then save the value of the center of the hypercube in that dimension

        These values are the min and max values in each dimensions of the constrained zonotope.
        
        We can use these min/max values to define a tight hypercube over-approximation of the constrained zonotope
        '''

        bounds = np.array([
            [-2.5, 2.0],        # Bounds in x-direction
            [-1.45, 1.45]         # Bounds in y-direction
        ])

        # Define Parameters
        step_size = 0.05
        min_val = np.zeros((cz.dim, 1))         # To store the minimum value of each dimension
        max_val = np.zeros((cz.dim, 1))         # To store the maximum value of each dimension
        c_new   = np.zeros((cz.dim, 1))         # To store the new center
        G_new   = np.zeros((cz.dim, cz.dim))    # To store the new generators
        
        G_hyper = np.array([
            [2.5, 0.0],
            [0.0, 2.5]
        ])
        C_hyper = np.array([
            [0.0],
            [0.0]
        ])

        hypercube = ConstrainedZonotope(G_hyper, C_hyper, np.zeros((0, 2)), np.zeros((0, 1)))

        for d in range(cz.dim):
            # Find maximum vertical value of dimension d
            ctr = 0
            original_G = hypercube.G[d, d]

            hypercube.G[d, d] = step_size

            # n_steps = int((bounds[d, 1] - bounds[d, 0]))
            # Loop through the positive axis 
            val = bounds[d, 0]
            # while True and val < n_steps/2:
            while True and val < bounds[d, 1]:
                val += step_size
                hypercube.C[d, 0] = val
                inters = self.intersection_cz_cz(cz, hypercube)
                if ctr == 0:
                    if not self.is_empty_cz(inters):
                        # min_val[d, 0] = val# + step_size
                        min_val[d, 0] = val + step_size
                        ctr += 1
                elif ctr == 1:
                    if self.is_empty_cz(inters):
                        # max_val[d, 0] = val - 2*step_size
                        max_val[d, 0] = val - 2*step_size
                        break

            # Reset G, C
            hypercube.G[d, d] = original_G
            hypercube.C[d, 0] = 0

            # Compute the new center as the middle point of each dimension
            c_new[d, 0] = (max_val[d, 0] + min_val[d, 0]) / 2
            G_new[d, d] = (max_val[d, 0] - min_val[d, 0]) / 2

        print(f'max_val = {max_val.T}')
        print(f'min_val = {min_val.T}')

        return ConstrainedZonotope(G_new, c_new, np.zeros((0, G_new.shape[1])), np.zeros((0, 1)))

    def oa_cz_to_hypercube_tight_4d(self, cz: ConstrainedZonotope) -> ConstrainedZonotope:
        '''
        This method is specifically designed for constrained zonotopes that represent very simple sets, however with a lot of redundancy 

        To set the theme, this is gonna be used for the FRS of moving obstacles in the environment.
        More precisely, given that the obstacle is a moving car and we initially represent it as a simple rectangle, we then wanna compute
        its forward reachable set and the RCI for each FRS. Then we wanna compute its complement to see which set of states are actually safe
        for the ego vehicle. Since we only have at our disposal a formula for the computation of the complement of a constrained zonotope
        but not for a hybrid zonotope, we wanna convert that hybrid zonotope into a constrained zonotope.

        Generally, this is gonna result in an over-approximation. However, even that over-approximation contains a lot of redundant information
        because all we do (at the moment) to generate that constrained zonotope is to simply use the method 'oa_hz_to_cz'.

        Therefore, we need to take extra steps to reduce the complexity of that CZ. This method does exactly that, that is it takes care of that
        second step of over-approximation to significantly reduce the complexity of the CZ. The goal will be to generate a hypercube over-approximation        
        
        In this version we:

        Assume that the dimensionality of the space is 'n'
        Define a hypercube of dimensionality 'n' as a zonotope where each side of the hypercube is equal to the max and min of the
        constrained zonotope when all of its constraints are ignored. This applies for all dimensions apart from the one that will be examined at each time.
        For the dimension that is examined just define its size to be the same with the step size (This is gonna be clearer later on)
        
        Then for each dimension we compute the intersection of the original cosntrained zonotope with the hypercube with its center shifted by a single step.

        Then save the value of the center of the hypercube in that dimension

        These values are the min and max values in each dimensions of the constrained zonotope.
        
        We can use these min/max values to define a tight hypercube over-approximation of the constrained zonotope
        '''

        bounds = np.array([
            [-2.5, 2.1],        # Bounds on position in x-direction
            [-1.5, 1.5],      # Bounds on position in y-direction
            [-1.1, 1.1],        # Bounds on velocity in x-direction
            [-1.1, 1.1]         # Bounds on velocity in y-direction 
        ])

        # Define Parameters
        step_size = 0.01
        min_val = np.zeros((cz.dim, 1))         # To store the minimum value of each dimension
        max_val = np.zeros((cz.dim, 1))         # To store the maximum value of each dimension
        c_new   = np.zeros((cz.dim, 1))         # To store the new center
        G_new   = np.zeros((cz.dim, cz.dim))    # To store the new generators
        
        G_hyper = np.array([
            [3.0, 0.0, 0.0, 0.0],
            [0.0, 3.0, 0.0, 0.0],
            [0.0, 0.0, 2.0, 0.0],
            [0.0, 0.0, 0.0, 2.0]
        ])
        C_hyper = np.array([
            [0.0],
            [0.0],
            [0.0],
            [0.0]
        ])

        hypercube = ConstrainedZonotope(G_hyper, C_hyper, np.zeros((0, 4)), np.zeros((0, 1)))

        for d in range(cz.dim):
            # Find maximum vertical value of dimension d
            ctr = 0
            original_G = hypercube.G[d, d]

            hypercube.G[d, d] = step_size

            # n_steps = int((bounds[d, 1] - bounds[d, 0]))
            # Loop through the positive axis 
            val = bounds[d, 0]
            # while True and val < n_steps/2:
            while True and val < bounds[d, 1]:
                val += step_size
                hypercube.C[d, 0] = val
                inters = self.intersection_cz_cz(cz, hypercube)
                if ctr == 0:
                    if not self.is_empty_cz(inters):
                        # min_val[d, 0] = val# + step_size
                        min_val[d, 0] = val + step_size
                        ctr += 1
                elif ctr == 1:
                    if self.is_empty_cz(inters):
                        # max_val[d, 0] = val - 2*step_size
                        max_val[d, 0] = val - 2*step_size
                        break

            # Reset G, C
            hypercube.G[d, d] = original_G
            hypercube.C[d, 0] = 0

            # Compute the new center as the middle point of each dimension
            c_new[d, 0] = (max_val[d, 0] + min_val[d, 0]) / 2
            G_new[d, d] = (max_val[d, 0] - min_val[d, 0]) / 2

        print(f'x :\t[{min_val[0, 0]:+0,.2f} -> {max_val[0, 0]:+0,.2f}]')
        print(f'y :\t[{min_val[1, 0]:+0,.2f} -> {max_val[1, 0]:+0,.2f}]')
        print(f'vx:\t[{min_val[2, 0]:+0,.2f} -> {max_val[2, 0]:+0,.2f}]')
        print(f'vy:\t[{min_val[3, 0]:+0,.2f} -> {max_val[3, 0]:+0,.2f}]')

        # print(f'max_val = {max_val.T}')
        # print(f'min_val = {min_val.T}')

        return ConstrainedZonotope(G_new, c_new, np.zeros((0, G_new.shape[1])), np.zeros((0, 1)))

    ############################################################################################################
    # Hybrid Zonotope methods

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
        
    def union_hz_hz_v2(self, hz1: HybridZonotope, hz2: HybridZonotope) -> HybridZonotope:
        '''
        Computes the union of two hybrid zonotopes. This function supports disjoint sets as well

        For more info check the work in [3]

        TODO: Optimize the computation of the inverse matrices
        '''
        # Step 1: Solve the set of linear equations
        ones_1 = np.ones((hz1.nb, 1)); ones_2 = np.ones((hz2.nb, 1))
        Ab_1_ones = hz1.Ab @ ones_1; Ab_2_ones = hz2.Ab @ ones_2
        Gb_1_ones = hz1.Gb @ ones_1; Gb_2_ones = hz2.Gb @ ones_2

        Gb_hat = 0.5 * ( ( Gb_2_ones + hz1.C) - (Gb_1_ones + hz2.C) )
        Ab1_hat = 0.5 * ( -Ab_1_ones - hz1.b )
        Ab2_hat = 0.5 * ( Ab_2_ones + hz2.b )
        b1_hat = 0.5 * ( -Ab_1_ones + hz1.b )
        b2_hat = 0.5 * ( -Ab_2_ones + hz2.b )
        C_hat = 0.5 * ( ( Gb_2_ones + hz1.C) + (Gb_1_ones + hz2.C) )

        # Find the index of all non-zero columns of Gc
        nonzero_gc_1 = np.nonzero(hz1.Gc.any(axis=0))[0]
        nonzero_gb_1 = np.nonzero(hz1.Gb.any(axis=0))[0]
        nonzero_gb_1 = nonzero_gb_1# + hz1.ng

        # Find the index of all non-zero columns of Gc
        nonzero_gc_2 = np.nonzero(hz2.Gc.any(axis=0))[0]
        nonzero_gb_2 = np.nonzero(hz2.Gb.any(axis=0))[0]
        nonzero_gb_2 = nonzero_gb_2# + hz2.ng

        staircase_ac3_left  = np.zeros((nonzero_gc_1.shape[0], hz1.ng))
        staircase_ac3_right = np.zeros((nonzero_gc_2.shape[0], hz2.ng))

        for r, c in enumerate(nonzero_gc_1):
            staircase_ac3_left[r, c] = 1

        for r, c in enumerate(nonzero_gc_2):
            staircase_ac3_right[r, c] = 1


        staircase_ab3_left  = np.zeros((nonzero_gb_1.shape[0], hz1.nb))
        staircase_ab3_right = np.zeros((nonzero_gb_2.shape[0], hz2.nb))

        for r, c in enumerate(nonzero_gb_1):
            staircase_ab3_left[r, c] = 1
        for r, c in enumerate(nonzero_gb_2):
            staircase_ab3_right[r, c] = 1

        # Auxiliary variables
        n1 = 2*(staircase_ac3_left.shape[0] + staircase_ac3_right.shape[0] + staircase_ab3_left.shape[0] + staircase_ab3_right.shape[0])

        # Step 3: Construst the union of the hybrid zonotopes
        C = C_hat

        Gc = np.block([
            [hz1.Gc, hz2.Gc, np.zeros((hz1.dim, n1))],
        ])
        Gb = np.block([
            [hz1.Gb, hz2.Gb, Gb_hat]
        ])


        Ac3 = np.block([
            [     staircase_ac3_left,   np.zeros((staircase_ac3_left.shape[0], hz2.ng))],
            [    -staircase_ac3_left,   np.zeros((staircase_ac3_left.shape[0], hz2.ng))],
            [ np.zeros((staircase_ac3_right.shape[0], hz1.ng)),       staircase_ac3_right],
            [ np.zeros((staircase_ac3_right.shape[0], hz1.ng)),      -staircase_ac3_right],
            [ np.zeros((staircase_ab3_left.shape[0], hz1.ng)),   np.zeros((staircase_ab3_left.shape[0], hz2.ng))],
            [ np.zeros((staircase_ab3_left.shape[0], hz1.ng)),   np.zeros((staircase_ab3_left.shape[0], hz2.ng))],
            [ np.zeros((staircase_ab3_right.shape[0], hz1.ng)),   np.zeros((staircase_ab3_right.shape[0], hz2.ng))],
            [ np.zeros((staircase_ab3_right.shape[0], hz1.ng)),   np.zeros((staircase_ab3_right.shape[0], hz2.ng))]
        ])

        # print(f'Ac3 = \n{Ac3}')


        Ab3 = np.block([
            [ np.zeros((staircase_ac3_left.shape[0], hz1.nb)),  np.zeros((staircase_ac3_left.shape[0], hz2.nb)), 0.5*np.ones((staircase_ac3_left.shape[0], 1))],
            [ np.zeros((staircase_ac3_left.shape[0], hz1.nb)),  np.zeros((staircase_ac3_left.shape[0], hz2.nb)), 0.5*np.ones((staircase_ac3_left.shape[0], 1))],
            [ np.zeros((staircase_ac3_right.shape[0], hz1.nb)), np.zeros((staircase_ac3_right.shape[0], hz2.nb)), -0.5*np.ones((staircase_ac3_right.shape[0], 1))],
            [ np.zeros((staircase_ac3_right.shape[0], hz1.nb)), np.zeros((staircase_ac3_right.shape[0], hz2.nb)), -0.5*np.ones((staircase_ac3_right.shape[0], 1))],
            [ 0.5*staircase_ab3_left,    np.zeros((staircase_ab3_left.shape[0], hz2.nb)),     0.5*np.ones((staircase_ab3_left.shape[0], 1))],
            [-0.5*staircase_ab3_left,    np.zeros((staircase_ab3_left.shape[0], hz2.nb)),     0.5*np.ones((staircase_ab3_left.shape[0], 1))],
            [ np.zeros((staircase_ab3_right.shape[0], hz1.nb)),    0.5*staircase_ab3_right,    -0.5*np.ones((staircase_ab3_right.shape[0], 1))],
            [ np.zeros((staircase_ab3_right.shape[0], hz1.nb)),   -0.5*staircase_ab3_right,    -0.5*np.ones((staircase_ab3_right.shape[0], 1))]
        ])
        b3 = np.block([
            [0.5*np.ones((staircase_ac3_left.shape[0], 1))],
            [0.5*np.ones((staircase_ac3_left.shape[0], 1))],
            [0.5*np.ones((staircase_ac3_right.shape[0], 1))],
            [0.5*np.ones((staircase_ac3_right.shape[0], 1))],
            [    np.zeros((staircase_ab3_left.shape[0], 1))],
            [     np.ones((staircase_ab3_left.shape[0], 1))],
            [    np.zeros((staircase_ab3_right.shape[0], 1))],
            [     np.ones((staircase_ab3_right.shape[0], 1))],
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

    def oa_hz_to_cz(self, hz: HybridZonotope) -> ConstrainedZonotope:
        '''
        This method takes in a hybrid zonotope and returns a constrained zonotope
        over-approximating the hybrid zonotope.
        '''   
        G = np.block([hz.Gc, hz.Gb])
        C = hz.C
        A = np.block([hz.Ac, hz.Ab])
        b = hz.b

        return ConstrainedZonotope(G, C, A, b)

    def oa_hz_to_z(self, hz: HybridZonotope) -> Zonotope:
        '''
        This method takes in a hybrid zonotope and returns a zonotope
        over-approximating the hybrid zonotope.
        '''   
        G = np.block([hz.Gc, hz.Gb])
        C = hz.C
 
        return Zonotope(C, G)

    ##### Redundancy removal
    def redundant_gc_hz(self, hz: HybridZonotope) -> HybridZonotope:
        '''
        Removes redundant generators from a Hybrid Zonotope.

        This method first partially forms the lifted hybrid zonotope. Then it
        scans all generators and whenver it finds a pair of generators that are
        parallel to each other, it adds one to the other and removes the other one.

        Example: If we have two generators g1 and g2, and g1 = 2*g2,
        then we update g1 as g1 = g1 + g2 = 3*g2 and remove g2. 
        '''

        max_angle = 0.05 * math.pi / 180
        threshold = 1 - math.sin(max_angle)

        # Step 1: Stack Gc and Ac
        G = np.block([
            [hz.Gc],
            [hz.Ac]
        ])

        norms = np.linalg.norm(G, axis = 0)             # Compute norm of all columns
        zeros = np.where(norms == 0)[0]                 # Find all indices of 'norms' that are zero
        G = np.delete(arr = G, obj = zeros, axis = 1)   # Remove all 'zero' norm columngs from G
        ng = G.shape[1]

        i = 0; j = 0; k = 0
        while i < ng - k:
            g1_unit = G[:, i] / np.linalg.norm(G[:, i]) # Unit vector of g1

            j = i + 1
            while j < ng - k:

                g2 = G[:, j]
                g2_unit = g2 / np.linalg.norm(g2)       # Unit vector of g2

                if np.abs(np.dot(g1_unit.T, g2_unit)) >= threshold:
                    G[:, i] = G[:, i] + g2
                    G = np.delete(G, j, axis=1)         # Remove the second generator
                    k += 1
                else:
                    j += 1

            i +=1

        return HybridZonotope(G[:hz.dim, :], hz.Gb, hz.C, G[hz.dim:, :], hz.Ab, hz.b)    

    def redundant_c_hz(self, hz: HybridZonotope) -> HybridZonotope:
        '''
        Reduces the number of constraints of a hybrid Zonotope.

        In this method we are removing the following constraints:
            - Any constraint whose constraint matrix component (the particular row in Ac) is all zeros
            - Any constraint that there is another constraint that is equivalent to it
                e.g., x + y = 1 and 2x + 2y = 2, 5x + 5x = 5 only one out of these three constraints will be kept
        '''
        max_angle = 0.05 * math.pi / 180
        threshold = 1 - math.sin(max_angle)

        A = np.block([hz.Ac, hz.Ab, hz.b])

        norms = np.linalg.norm(A, axis = 1)             # Compute norm of all rows
        zeros = np.where(norms == 0)[0]                 # Find all indices of 'norms' that are zero
        A = np.delete(arr = A, obj = zeros, axis = 0)   # Remove all 'zero' norm columngs from G
        nc = A.shape[0]

        i = 0; j = 0; k = 0
        while i < nc - k:
            c1_unit = A[i, :] / np.linalg.norm(A[i, :]) # Unit vector of c1

            j = i + 1
            while j < nc - k:
                c2 = A[j, :]
                c2_unit = c2 / np.linalg.norm(c2)       # Unit vector of c2

                if np.abs(np.dot(c1_unit.T, c2_unit)) >= threshold:
                    A = np.delete(A, j, axis=0)         # Remove the second constraint
                    k += 1
                else:
                    j += 1

            i +=1


        return HybridZonotope(hz.Gc, hz.Gb, hz.C, A[:, :hz.Ac.shape[1]], A[:, hz.Ac.shape[1]:-1], A[:, -1].reshape((A.shape[0], 1)))    

    def redundant_c_gc_hz_v1(self, hz: HybridZonotope, options = 'slow') -> HybridZonotope:
        '''
        This method performs redundancy removal for constraints and continuous generators in a hybrid zonotope.

        # This version always works
        '''
        epsilon = 1e-3
        redundant = True

        Eb = np.array([ [-1, 1] for b in range(hz.nb) ])

        hz = self.redundant_c_hz(hz)
        hz = self.redundant_gc_hz(hz)

        while redundant:
            redundant = False
            hz = self.rref_hz(hz)
            if options == 'fast':
                E = self.find_E_hz_fast(hz)
            else:
                E = self.find_E_hz_slow(hz)

            for c in range (hz.ng):
                for r in range(hz.nc):
                    if np.abs(hz.Ac[r, c]) >= epsilon:
                        a_rc_inv = (1/hz.Ac[r, c])
                        R_rc = np.array([hz.b[r,0], hz.b[r,0]])

                        tempc = np.array([0.0, 0.0])
                        tempb = np.array([0.0, 0.0])
                        for k in range(hz.ng):
                            if k != c:
                                tempc = self.interval_add(tempc, self.interval_scalar_mul(hz.Ac[r, k], E[k, :]))

                        for b in range(hz.nb):
                            tempb = self.interval_add(tempb, self.interval_scalar_mul(hz.Ab[r, b], Eb[b, :]))

                        R_rc = self.interval_sub(R_rc, tempc)
                        R_rc = self.interval_sub(R_rc, tempb)

                        R_rc = self.interval_scalar_mul(a_rc_inv, R_rc)

                        if self.is_inside_interval(R_rc, np.array([-1, 1])):
                            hz = self.remove_c_g_hz(hz = hz, c = c, r = r)
                            redundant = True

                            break
                if redundant:
                    break

        return hz

    def find_E_hz_slow(self, hz):
        '''
        This method finds the E interval by solving 2*ng MILPs in equations (6.2a, 6.2b) from Section 6.1.2 in [7]
        This method provides the exact E bounds but it is generally more computationally expensive
        '''
        
        ## Step 1: Create a model
        model = gp.Model('intervals_cz')
        model.Params.OutputFlag = 0         # Disable verbose output

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (hz.ng, ), lb = np.array([-1] * hz.ng), ub = np.array([1] * hz.ng), vtype = np.array([gp.GRB.CONTINUOUS] * hz.ng), name = 'x_c')
        x_b = model.addMVar(shape = (hz.nb, ), lb = np.array([-1] * hz.nb), ub = np.array([1] * hz.nb), vtype = np.array([gp.GRB.INTEGER] * hz.nb), name = 'x_b')

        # Enforce that x_b only takes values in {-1, 1}^hz.nb
        for i in range(hz.nb):
            model.addConstr(x_b[i] * x_b[i] == 1 )

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        ## Step 3: Add constraints
        rhs = hz.b                          # Right hand side of equality constraint equation
        lhs = hz.Ac @ x_c + hz.Ab @ x_b     # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)
        
        model.addConstr(norm_inf == gp.norm(x_c, gp.GRB.INFINITY))  # Use the 'norm' General constraint helper function from the gurobi API

        x_L = []
        for g in range(hz.ng):
            model.setObjective(x_c[g], gp.GRB.MINIMIZE)
            model.optimize()
            x_L.append(x_c[g].X)

        x_U = []
        for g in range(hz.ng):
            model.setObjective(x_c[g], gp.GRB.MAXIMIZE)
            model.optimize()
            x_U.append(x_c[g].X)

        x_U = np.array(x_U); x_L = np.array(x_L)

        E = np.block([x_L.reshape(-1, 1), x_U.reshape(-1, 1)])

        return E

    def find_E_hz_fast(self, hz):
        '''
        This method computes the Intervals of the input hybrid zonotope according to Algorithm 1 in [6]
        which is then adapted to work for Hybrid zonotopes.
        This method does not provide the exact E bounds but it is generally more computationally efficient
        '''

        # Step 1: Initialize intervals Ej and Rj as Ej <- [-1, 1], Rj <- [-inf, inf], i,j <- 1
        E = np.array([ [-1, 1] for g in range(hz.ng) ])
        Eb = np.array([ [-1, 1] for b in range(hz.nb) ])
        R = np.array([ [-np.inf, np.inf] for g in range(hz.ng) ])

        A = hz.Ac
        epsilon = 1e-5
        iterations = 50  # Maximum number of iterations

        for iter in range(iterations):
            for i in range(hz.nc):
                for j in range(hz.ng):
                    if abs(A[i, j]) >= epsilon:
                        a_rc_inv = (1/hz.Ac[i, j])
                        R_rc = np.array([hz.b[i,0], hz.b[i,0]])

                        tempc = np.array([0.0, 0.0])
                        tempb = np.array([0.0, 0.0])
                        for k in range(hz.ng):
                            if k != j:
                                tempc = self.interval_add(tempc, self.interval_scalar_mul(hz.Ac[i, k], E[k, :]))
                        for b in range(hz.nb):
                            tempb = self.interval_add(tempb, self.interval_scalar_mul(hz.Ab[i, b], Eb[b, :]))
                        R_rc = self.interval_sub(R_rc, tempc)
                        R_rc = self.interval_sub(R_rc, tempb)
                        R_rc = self.interval_scalar_mul(a_rc_inv, R_rc)


                        R[j] = self.intesection_intervals(R[j], R_rc)
                        E[j] = self.intesection_intervals(E[j], R[j])
        return E

    def rref_hz(self, hz):
        '''
        This method computes the reduced row echelon form of matrix [A | b] for a linear system of equations (Ax = b)
        using Gauss-Jordan Elimination with full pivoting.

        - FUTURE IMPROVEMENTS: Experiment with other preconditioning strategies
        '''
        A = np.block([hz.Ac, hz.Ab, hz.b])

        rows = A.shape[0]

        pivots = []
        for r in range(rows):
            # Find the pivot row and column
            pivot, pivots = self.find_pivot(A[r, :-1], pivots)

            # Check if there is a new pivot
            if len(pivots) < r + 1:
                continue
            
            # Normalize the pivot row to turn the pivot element into 1
            A[r, :] = A[r, :] / A[r, pivot]

            # Turn the elements of all other rows in the pivot column to zero
            for r2 in range(rows):
                if r2 != r:
                    A[r2, :] = A[r2, :] - A[r2, pivot] * A[r, :]

        Ac = A[:, :hz.Ac.shape[1]]
        Ab = A[:, hz.Ac.shape[1]:-1]
        b = A[:, -1].reshape((A.shape[0], 1))

        return HybridZonotope(hz.Gc, hz.Gb, hz.C, Ac, Ab, b)

    def remove_c_g_hz(self, hz, c, r):
        '''
        c: constraint index
        r: generator index
        '''
        Ecr = np.zeros((hz.ng, hz.nc))
        Ecr[c, r] = 1

        Lg = hz.Gc @ Ecr * (1/hz.Ac[r, c])
        La = hz.Ac @ Ecr * (1/hz.Ac[r, c])

        Gc = hz.Gc - Lg @ hz.Ac
        Gb = hz.Gb - Lg @ hz.Ab
        C  = hz.C  + Lg @ hz.b
        Ac = hz.Ac - La @ hz.Ac
        Ab = hz.Ab - La @ hz.Ab
        b  = hz.b  - La @ hz.b

        Gc = np.delete(Gc, c, axis=1)
        Ac = np.delete(Ac, c, axis=1)
        Ac = np.delete(Ac, r, axis=0)
        Ab = np.delete(Ab, r, axis=0)
        b  = np.delete(b, r, axis=0)

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)

    def redundant_c_gc_hz_v2(self, hz: HybridZonotope) -> HybridZonotope:
        '''
        # This version does not always work. 
        Nevertheless, I still use it for the initial space as it works in these cases and reduces the redundancy better than redundant_c_gc_hz_v1
        '''
        E, R = self.intervals_hz(hz)
        A = hz.Ac

        epsilon = 1e-3

        already_removed_c = []; already_removed_g = []
        for c in range (hz.ng):
            for r in range(hz.nc):
                if np.abs(A[r, c]) >= epsilon:
                    a_rc_inv = (1/A[r, c])
                    sum = 0
                    for k in range(hz.ng):
                        if k != c:
                            sum = sum + A[r, k] * E[k]
                    R_rc = a_rc_inv * hz.b[r,0] - a_rc_inv * sum

                    if self.is_inside_interval(R_rc, np.array([-1, 1])) and (r not in already_removed_c) and (c not in already_removed_g):
                        already_removed_c.append(r); already_removed_g.append(c)
                        Ecr = np.zeros((hz.ng, hz.nc))
                        Ecr[c, r] = 1

                        Lg = hz.Gc @ Ecr * (1/A[r, c])
                        La = hz.Ac @ Ecr * (1/A[r, c])

                        # Check if Lg has only zero zero values
                        full_zero = True
                        for x in range(Lg.shape[1]):
                            for y in range(Lg.shape[0]):
                                if Lg[y, x] != 0:
                                    full_zero = False

                        if not (full_zero):
                            Gc = hz.Gc - Lg @ hz.Ac
                            Gb = hz.Gb - Lg @ hz.Ab
                            C  = hz.C  + Lg @ hz.b
                            Ac = hz.Ac - La @ hz.Ac
                            Ab = hz.Ab - La @ hz.Ab
                            b  = hz.b  - La @ hz.b

                            hz = HybridZonotope(Gc, Gb, C, Ac, Ab, b)



        hz = self.reduce_c_hz(hz)   # Remove the redundant or zero constraints
        hz = self.reduce_gc_hz(hz)


        return hz

    def intervals_hz(self, hz):
        '''
        This method computes the Intervals of the input hybrid zonotope
        '''

        # Step 1: Initialize intervals Ej and Rj as Ej <- [-1, 1], Rj <- [-inf, inf], i,j <- 1
        E = np.array([ [-1, 1] for g in range(hz.ng + hz.nb) ])
        R = np.array([ [-np.inf, np.inf] for g in range(hz.ng + hz.nb) ])
        i = 0; j = 0

        A = np.block([hz.Ac, hz.Ab])

        while i < (hz.nc):
            while j < (hz.ng + hz.nb):
                if A[i, j] != 0:
                    a_ij_inv = (1/A[i, j])
                    sum = 0
                    for k in range(hz.ng + hz.nb):
                        if k != j:
                            sum += A[i, k] * E[k]
                    gen_val = a_ij_inv * hz.b[i,0] - a_ij_inv * sum
                    R[j] = self.intesection_intervals(R[j], gen_val)
                    E[j] = self.intesection_intervals(E[j], R[j])

                j += 1
            i += 1
            j = 0

        return E, R
    
    def reduce_c_hz(self, hz: HybridZonotope) -> HybridZonotope:
        '''
        Reduces the number of constraints of a Hybrid Zonotope.

        In this version we are removing the following constraints:
            - Any constraint whose constraint matrix component (the particular row in [Ac Ab]) is all zeros
            - Any constraint that there is another constraint that is equivalent to it
                e.g., x + y = 1 and 2x + 2y = 2, 5x + 5x = 5 only one out of these three constraints will be kept

        TODO: NOW THAT YOU HAVE IMPLEMENTED THE NEW METHOD CHECK IF THIS IS REDUNDANT
        '''
        max_angle = 0.05 * math.pi / 180
        threshold = 1 - math.sin(max_angle)

        A = np.block([hz.Ac, hz.Ab, hz.b])

        nc = A.shape[0]

        # Loop through all the columns of Gc
        i = 0; j = 0; k = 0

        while i < nc - k:
            c1 = A[i, :].T
            c1_mag = np.linalg.norm(c1)    # Magnitude of c1

            if np.abs(c1_mag) <= 0.001:
                A = np.delete(A, i, axis=0)
                k += 1
                continue

            c1_unit = c1 / c1_mag           # Unit vector of c1


            j = 0
            while j < nc - k:
                if i == j:
                    j += 1
                    continue

                c2 = A[j, :].T
                c2_mag = np.linalg.norm(c2)     # Magnitude of c2

                if (c2_mag <= 0.001) or np.abs(np.dot(c1_unit.T, c2 / c2_mag)) >= threshold:
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
        Removes redundant continuous generators from a Hybrid Zonotope.

        This method first forms the lifted hybrid zonotope. Then it
        scans all generators and whenver it finds a pair of generators that are
        parallel to each other, it adds one to the other and removes the other one.

        Example: If we have two generators g1 and g2, and g1 = 2*g2,
        then we update g1 as g1 = g1 + g2 = 3*g2 and remove g2. 

        TODO: NOW THAT YOU HAVE IMPLEMENTED THE NEW METHOD CHECK IF THIS IS REDUNDANT

        '''

        # threshold = 1e-7
        max_angle = 0.05 * math.pi / 180
        # max_angle = 0.5 * math.pi / 180
        threshold = 1 - math.sin(max_angle)

        # Step 1: Stack Gc and Ac
        G = np.block([
            [hz.Gc],
            [hz.Ac]
        ])

        ng = G.shape[1]

        # Loop through all the rows of Gc
        i = 0; j = 0; k = 0

        while i < ng - k:
            g1 = G[:, i]
            g1_mag = np.linalg.norm(g1)    # Magnitude of g1

            if np.abs(g1_mag) <= 0.001:
                G = np.delete(G, i, axis=1)
                k += 1
                continue

            g1_unit = g1 / g1_mag           # Unit vector of g1


            j = 0
            while j < ng - k:
                if i == j:
                    j += 1
                    continue

                g2 = G[:, j]
                g2_mag = np.linalg.norm(g2)     # Magnitude of g2


                if (g2_mag <= 0.001):
                    G = np.delete(G, j, axis=1)   # Remove the second generator
                    k += 1                    
                # elif np.abs(np.dot(g1_unit.T, g2 / g2_mag)) >= threshold:
                #     G[:, i - k] = g1 + g2
                #     G = np.delete(G, j, axis=1)   # Remove the second generator
                #     k += 1

                j += 1

            i +=1

        Gc = G[:hz.dim, :]
        Ac = G[hz.dim:, :]


        return HybridZonotope(Gc, hz.Gb, hz.C, Ac, hz.Ab, hz.b)

    def ua_gc_hz(self, hz: HybridZonotope, N: int) -> HybridZonotope:
        '''
        TODO: This method needs to be checked again if it indeed works.

        Reduces the number of continuous generators of a Hybrid Zonotope.

        This method first removes all parallel generators of the lifted hybrid zonotope.

        Then it makes use of the work in [4] to further reduce the number of continuous generators
        by under-approximation of maximum N generators
        '''
        # Continuous Generators of Lifted Hybrid Zonotope
        G = np.block([
            [hz.Gc],
            [hz.Ac]
        ])

        ng = G.shape[1]

        # Loop through all the rows of Gc
        i = 0; j = 0

        objective = []

        while i < ng:
            g1 = G[:, i]
            g1_mag = np.linalg.norm(g1)     # Magnitude of g1
            g1_unit = g1 / g1_mag           # Unit vector of g1

            j = 0
            while j < ng:
                if i == j:
                    j += 1
                    continue

                g2 = G[:, j]

                objective.append( ( g1_mag * np.linalg.norm(g2 - np.dot(g1_unit, np.dot(g2.T, g1_unit)) ),
                                    i,
                                    j))

                j += 1
            i +=1

        # # Sort the objective list from min to max
        # objective = sorted(objective, key=lambda x: x[0])   # This has a cost of O(n log n)
        objective.sort()


        remaining_indices = set(range(G.shape[1]))  # Set to keep track of the remaining generator indices
        updated_generators = []                     # List to store the updated generators

        # Perform generator elimination until 'N' generators are left
        while len(remaining_indices) > N:
            # Find the pair with the smallest objective value
            _, i, j = objective.pop(0)

            # Check if generators still exist
            if i in remaining_indices and j in remaining_indices:
                # Replace g1 by g1 + g2
                G[:, i] += G[:, j]
                remaining_indices.remove(j)

        # Create the updated matrix with the remaining vectors
        for i in range(G.shape[1]):
            if i in remaining_indices:
                updated_generators.append(G[:, i])

        G = np.column_stack(updated_generators)


        Gc = G[:hz.dim, :]
        Ac = G[hz.dim:, :]

        return HybridZonotope(Gc, hz.Gb, hz.C, Ac, hz.Ab, hz.b)
    
    ##### Reachable Sets

    def one_step_brs_hz(self, X: HybridZonotope, T: HybridZonotope, D: np.ndarray) -> HybridZonotope:
        '''
        Computes a one-step backward reachable set using the hybrid zonotope representation
        
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

    def one_step_brs_hz_v2(self, X: HybridZonotope, U: HybridZonotope, T: HybridZonotope, A: np.ndarray, B: np.ndarray) -> HybridZonotope:
        BU = self.lt_hz(-B, U)
        T_plus_BU = self.ms_hz_hz(hz1 = T, hz2 = BU)
        A_inv = np.linalg.inv(A)
        A_inv_T_W_plus_BU = self.lt_hz(A_inv, T_plus_BU)

        # Compute intersection with safe space X
        X_intersection_A_inv_T_W_plus_BU = self.intersection_hz_hz(hz1 = X, hz2 = A_inv_T_W_plus_BU)


        return X_intersection_A_inv_T_W_plus_BU

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


                # 4D
                # p = brs_settings.space[y_i, x_i, 0, 0]
                # p = np.array([ [p[0]], [p[1]], [0.0], [0.0] ])
                
                # 2D
                p = brs_settings.space[y_i, x_i]
                p = np.array([ [p[0]], [p[1]] ])

                # print(f'(i = {i})  p = {p.T}')
                # i += 1

                close_enough = True
                if close_enough:
                    if self.is_inside_hz(hz, p):
                        
                        # Only for 4D
                        # brs_settings.is_already_contained[y_i, x_i, 10, 10] = 1

                        brs_settings.is_already_contained_xy[y_i, x_i] = 1

                        # Add the point p in the list of new points
                        new_points.append(p)

        return np.array(new_points)     

    def is_empty_hz(self, hz: HybridZonotope):
        '''
        Checks if the hybrid zonotope hz is empty
        '''
        
        ## Step 1: Create a model
        model = gp.Model('is_empty_hz')
        model.Params.OutputFlag = 0         # Disable verbose output
        # model.Params.MIPFocus = 1           # Set MIPFocus = 1 (Focus more on finding feasible solutions)
        # model.Params.ImproveStartTime = 0   # Set ImproveStartTime = 0 (To start focusing on finding feasible solutions immediately) (seconds)
        # model.Params.SolutionLimit = 1      # Set the SolutionLimit parameter to 1 (to find only one feasible solution)

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (hz.ng, ), lb = np.array([-1] * hz.ng), ub = np.array([ 1] * hz.ng), vtype = np.array([gp.GRB.CONTINUOUS] * hz.ng), name = 'x_c')
        x_b = model.addMVar(shape = (hz.nb, ), lb = np.array([-1] * hz.nb), ub = np.array([ 1] * hz.nb), vtype = np.array([gp.GRB.INTEGER] * hz.nb), name = 'x_b')

        # Enforce that x_b only takes values in {-1, 1}^hz.nb
        for i in range(hz.nb):
            model.addConstr(x_b[i] * x_b[i] == 1 )

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        rhs = hz.b                          # Right hand side of equality constraint equation
        lhs = hz.Ac @ x_c + hz.Ab @ x_b     # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)
        
        model.addConstr(norm_inf == gp.norm(x_c, gp.GRB.INFINITY))  # Use the 'norm' General constraint helper function from the gurobi API

        ## Step 3: Set the objective function
        model.setObjective(norm_inf, gp.GRB.MINIMIZE)

        ## Step 4: Solve the model
        model.optimize()

        # ## Step 5: Check if the solution is feasible
        # if model.status == gp.GRB.OPTIMAL or model.status == gp.GRB.SUBOPTIMAL:
        #     return False
        # else:
        #     return True
        ## Step 5: Check if the solution is feasible
        if model.status == gp.GRB.OPTIMAL:
            for i in range(hz.ng):      # Check if each value of the solution are less than '1'
                if abs(x_c[i].X) > 1:
                    return True
            return False
        else:
            return True

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

        A_I = self.lt_hz(A, I)                                              # A @ I
        B_U = self.lt_hz(B, U)                                              # B @ U
        A_I_plus_B_U = self.ms_hz_hz(hz1 = A_I, hz2 = B_U)                  # A @ I + B @ U
        A_I_plus_B_U_plus_W = self.ms_hz_hz(hz1 = A_I_plus_B_U, hz2 = W)    # A @ I + B @ U + W

        X_intersection_A_I_plus_B_U_plus_W = self.intersection_hz_hz(hz1 = X, hz2 = A_I_plus_B_U_plus_W)
        
        return X_intersection_A_I_plus_B_U_plus_W

    ##### TODO:

    def rci_hz(self, X: HybridZonotope, T: HybridZonotope, D: np.ndarray, N: int) -> HybridZonotope:
        '''
        
        '''
        for i in range(N):
            pass

    ############################################################################################################
    # Interval set methods

    def interval_add(self, x, y):
        lb = min(x[0], x[1]); ub = max(x[0], x[1]); x[0] = lb; x[1] = ub
        lb = min(y[0], y[1]); ub = max(y[0], y[1]); y[0] = lb; y[1] = ub

        lb = x[0] + y[0]
        ub = x[1] + y[1]
        
        return np.array([lb, ub])

    def interval_sub(self, x, y):
        lb = min(x[0], x[1]); ub = max(x[0], x[1]); x[0] = lb; x[1] = ub
        lb = min(y[0], y[1]); ub = max(y[0], y[1]); y[0] = lb; y[1] = ub

        lb = x[0] - y[1]
        ub = x[1] - y[0]
        
        return np.array([lb, ub])

    def interval_mul(self, x, y):
        '''
        This method implements the multiplication between two interval sets
        '''
        lb = min(x[0], x[1]); ub = max(x[0], x[1]); x[0] = lb; x[1] = ub
        lb = min(y[0], y[1]); ub = max(y[0], y[1]); y[0] = lb; y[1] = ub

        xl = min(x[0], x[1]); xu = max(x[0], x[1])
        yl = min(y[0], y[1]); yu = max(y[0], y[1])

        if yl >= 0 and xl >= 0:
            lb = xl*yl
            ub = xu*yu
        elif yl >= 0 and (xl <= 0 and xu >= 0):
            lb = xl*yu
            ub = xu*yu
        elif yl >= 0 and xu <= 0:
            lb = xl*yu
            ub = xu*yl
        elif (yl <= 0 and yu >= 0) and xl >= 0:
            lb = xu*yl
            ub = xu*yu
        elif (yl <= 0 and yu >= 0) and (xl <= 0 and xu >= 0):
            lb = min(xl*yu, xu*yl)
            ub = max(xl*yl, xu*yu)
        elif (yl <= 0 and yu >= 0) and xu <= 0:
            lb = xl*yu
            ub = xl*yl
        elif yu <= 0 and xl >= 0:
            lb = xu*yl
            ub = xl*yu
        elif yu <= 0 and (xl <= 0 and xu >= 0):
            lb = xu*yl
            ub = xl*yl
        elif yu <= 0 and xu <= 0:
            lb = xu*yu
            ub = xl*yl


        return np.array([lb, ub])
            
    def interval_scalar_mul(self, k, x):
        '''
        This method implements the multiplication between a scalar and an interval set
        '''
        lb = min(x[0], x[1]); ub = max(x[0], x[1]); x[0] = lb; x[1] = ub

        if k >= 0:
            lb = k*x[0]
            ub = k*x[1]
        else:
            lb = k*x[1]
            ub = k*x[0]

        return np.array([lb, ub])        

    def infinum(self, S):
        '''
        This method computes all the infinum elements of an n-dimensional interval set
        '''
        inf = np.zeros((S.shape[0], 1))
        for j, e in enumerate(S):
            inf[j, 0] = min(S[j, 0], S[j, 1])

        return inf

    def supremum(self, S):
        '''
        This method computes all the supremum elements of an n-dimensional interval set
        '''
        sup = np.zeros((S.shape[0], 1))
        for j, e in enumerate(S):
            sup[j, 0] = max(S[j, 0], S[j, 1])

        return sup     

    def is_inside_interval(self, interval_1, interval_2):
        '''
        Check if interval_1 is a subset of interval_2
        '''

        # sort the intervals
        l = min(interval_1[0], interval_1[1])
        r = max(interval_1[0], interval_1[1])

        intersection = self.intesection_intervals(interval_1, interval_2)

        if intersection[0] == l and intersection[1] == r:
            return True
        else:
            return False

    def is_inside_1(self, interval):
        '''
        Check if interval_1 is a subset of interval_2
        '''
        epsilon = 1e-3
        # sort the intervals
        l = min(interval[0], interval[1])
        r = max(interval[0], interval[1])

        # if l >= -1 + epsilon and r <= 1 - epsilon:
        if l >= -1 and r <= 1:
            return True
        else:
            return False

    def intesection_intervals(self, interval_1, interval_2):
        l1 = min(interval_1[0], interval_1[1])
        r1 = max(interval_1[0], interval_1[1])        

        l2 = min(interval_2[0], interval_2[1])
        r2 = max(interval_2[0], interval_2[1])        

        if (l2 > r1 or r2 < l1):
            pass

        # Else update the intersection
        else:
            l1 = max(l1, l2)
            r1 = min(r1, r2)


        return np.array([l1, r1])


    ############################################################################################################
    # Not used methods
    
    def rescale_hz(self, hz):
        '''
        Extends the notion of rescaling a constrained zonotope of work [1] for hybrid zonotopes
        '''
        
        ## Step 1: Create a model
        model = gp.Model('rescale_hz')
        model.Params.OutputFlag = 0         # Disable verbose output

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (hz.ng, ), lb = np.array([-1] * hz.ng), ub = np.array([ 1] * hz.ng), vtype = np.array([gp.GRB.CONTINUOUS] * hz.ng), name = 'x_c')
        x_b = model.addMVar(shape = (hz.nb, ), lb = np.array([-1] * hz.nb), ub = np.array([ 1] * hz.nb), vtype = np.array([gp.GRB.INTEGER] * hz.nb), name = 'x_b')

        # Enforce that x_b only takes values in {-1, 1}^hz.nb
        for i in range(hz.nb):
            model.addConstr(x_b[i] * x_b[i] == 1 )

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        ## Step 3: Add constraints
        rhs = hz.b                          # Right hand side of equality constraint equation
        lhs = hz.Ac @ x_c + hz.Ab @ x_b     # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)
        
        model.addConstr(norm_inf == gp.norm(x_c, gp.GRB.INFINITY))  # Use the 'norm' General constraint helper function from the gurobi API


        x_L = []
        for g in range(hz.ng):
            model.setObjective(x_c[g], gp.GRB.MINIMIZE)
            model.optimize()
            x_L.append(x_c[g].X)

        x_U = []
        for g in range(hz.ng):
            model.setObjective(x_c[g], gp.GRB.MAXIMIZE)
            model.optimize()
            x_U.append(x_c[g].X)

        x_U = np.array(x_U); x_L = np.array(x_L)
        x_m = (x_U + x_L)/2
        x_r = (x_U - x_L)/2

        x_m = x_m.reshape((hz.ng, 1))
        x_r = x_r.reshape((hz.ng, 1))
        diag = np.diag(x_r.flatten())

        Gc = hz.Gc @ diag
        Gb = hz.Gb
        c = hz.C + hz.Gc @ x_m
        Ac = hz.Ac @ diag
        Ab = hz.Ab
        b = hz.b - hz.Ac @ x_m

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)

    def rescale_cz(self, cz):
        '''
        This method rescales a constrained zonotope of work according to work [1]
        '''
        
        ## Step 1: Create a model
        model = gp.Model('rescale_cz')
        model.Params.OutputFlag = 0         # Disable verbose output

        ## Step 2: Create the variables
        x_c = model.addMVar(shape = (cz.ng, ), lb = np.array([-1] * cz.ng), ub = np.array([1] * cz.ng), vtype = np.array([gp.GRB.CONTINUOUS] * cz.ng), name = 'x_c')

        # Compute the infinity norm of x_c
        norm_inf = model.addMVar(shape = 1, lb = 0, vtype = gp.GRB.CONTINUOUS, name = 'norm_inf')

        ## Step 3: Add constraints
        rhs = cz.b                          # Right hand side of equality constraint equation
        lhs = cz.A @ x_c                    # Left hand side of equality constraint equation
        for left, right in zip(lhs, rhs):
            model.addConstr(left == right)
        
        model.addConstr(norm_inf == gp.norm(x_c, gp.GRB.INFINITY))  # Use the 'norm' General constraint helper function from the gurobi API

        x_L = []
        for g in range(cz.ng):
            model.setObjective(x_c[g], gp.GRB.MINIMIZE)
            model.optimize()
            x_L.append(x_c[g].X)

        x_U = []
        for g in range(cz.ng):
            model.setObjective(x_c[g], gp.GRB.MAXIMIZE)
            model.optimize()
            x_U.append(x_c[g].X)

        x_U = np.array(x_U); x_L = np.array(x_L)
        x_m = (x_U + x_L)/2
        x_r = (x_U - x_L)/2

        x_m = x_m.reshape((cz.ng, 1))
        x_r = x_r.reshape((cz.ng, 1))
        diag = np.diag(x_r.flatten())

        G = cz.G @ diag
        C = cz.C + cz.G @ x_m
        A = cz.A @ diag
        b = cz.b - cz.A @ x_m

        return ConstrainedZonotope(G, C, A, b)





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
    




