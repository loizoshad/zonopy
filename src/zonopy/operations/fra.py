import math
from copy import deepcopy

import numpy as np
import scipy.linalg as la

from zonopy.operations.operations import ZonoOperations
from zonopy.sets.zonotopes import Zonotope
from zonopy.visualization import ZonoVisualizer


class FRA:
    """
    This class implements all the functionality for Forward Reachability Analysis
    """

    def __init__(self) -> None:
        self.zono_op = ZonoOperations()
        self.visualizer = ZonoVisualizer()
        self.solver_options()  # Initialize solver options

    def solver_options(
        self,
        mu=0.5,  # Maximum uncertainty
        dt=0.02,  # Time step
        max_ord=10,  # Maximum order of the zonotope (20 generators)
        T=2,  # Time horizon
        solver="ag",
    ):
        assert mu >= 0, "Maximum uncertainty (mu) must be a non-negative number"
        assert dt > 0, "Time step (dt) must be a positive number"
        assert max_ord > 0, "Maximum order of the zonotope (max_ord) must be a positive number"
        assert T > 0, "Time horizon (T) must be a positive number"
        assert solver in ["ag"], "Invalid forward reachability analysis method"

        self.mu = mu
        self.dt = dt
        self.max_ord = max_ord
        self.T = T
        self.solver = solver

    def solve(self, A: np.array, z0=None):
        """
        A: Dynamics matrix
        z0: Initial set
        """
        if self.solver == "ag":
            R, R_approx = self.solve_ag(A, z0)

        return R, R_approx

    def solve_ag(self, A, z0):
        """
        This function impements the forward reachability analysis algorithm proposed by Antoine Girard
        in his paper 'Reachability of Uncertain Linear Systems Using Zonotopes' in 2005
        """
        # Dynamics model: x_dot = Ax + u, inf_norm(u) <= mu

        ## Initialization

        # Initialize parameters
        r = self.dt
        T = self.T
        mu = self.mu
        N = int(T / r)  # Number of time steps
        A_inf = np.linalg.norm(A, np.inf)  # Infinity norm of the dynamics matrix

        # sup_x is the Supremum of the infinity norm of the state vector x in the initial set defined by the zonotope z0
        sup_x = np.linalg.norm(z0.C + np.max(np.abs(z0.G), axis=1).reshape(-1, 1), np.inf)  # TODO: Is this correct?
        a_r = (math.e ** (r * A_inf) - 1 - r * A_inf) * sup_x
        b_r = (math.e ** (r * A_inf) - 1) * mu / A_inf

        # Initialize the zonotope P0
        L = (1 / 2) * (np.eye(A.shape[0]) + la.expm(r * A))
        M = (1 / 2) * (np.eye(A.shape[0]) - la.expm(r * A))

        c0 = L @ z0.C
        g0 = M @ z0.C

        for i in range(z0.ng):
            g0 = np.append(g0, L @ z0.G[:, i].reshape(-1, 1), axis=1)
            g0 = np.append(g0, M @ z0.G[:, i].reshape(-1, 1), axis=1)

        P = [Zonotope(c0, g0)]
        Q = [self.zono_op.ms_z_z(P[0], self.hypercube(a_r + b_r))]
        R = deepcopy(Q)  # Initialize the reachable set
        R_approx = deepcopy(R)

        ## Loop over time steps i = 1, 2, ..., N - 1

        for i in range(1, N):
            P.append(self.zono_op.lt_z(la.expm(r * A), R[i - 1]))
            Q.append(self.zono_op.ms_z_z(P[i], self.hypercube(b_r)))

            # Approximate the zonotope Q[i] with a zonotope of order 2n/d
            if self.max_ord < math.ceil(Q[i].order):
                R_approx.append(self.overapproximate_ag(Q[i]))
            else:
                R_approx.append(Q[i])

            R.append(R_approx[i])

        return R, R_approx

    def overapproximate_ag(self, z: Zonotope):
        """
        This function overapproximates the input zonotope of order m + 1 with a zonotope of order m
        """

        ## Step 1: Sort the generators and choose 2n generators from the input zonotope to be replaced by n generators of the output zonotope

        # Step 1.1: Sort the generators of the zonotope z such that:
        # np.linalg.norm(z.G[:,0], 1) - np.linalg.norm(z.G[:,0], inf) <= np.linalg.norm(z.G[:,1], 1) - np.linalg.norm(z.G[:,1], inf) <= ...
        norms = np.linalg.norm(z.G, ord=1, axis=0) - np.linalg.norm(z.G, ord=np.inf, axis=0)
        sorted_norms = np.argsort(norms)
        gen_sorted = z.G[:, sorted_norms]

        # Step 1.2: Choose 2*n generators, where n = math.floor(z.order)
        n = z.dim
        gen_sorted[:, 0 : 2 * n]
        h_keep = gen_sorted[:, 2 * n :]

        # TODO: Fix step 2, it is not working properly
        # # Step 2: Compute the n generators of the output zonotope
        # g = np.zeros((n, n)); sum = 0

        # # Compute all the sums: sum_{i=1}^{2n} |h_{i}^{j}|
        # # h_{i} is the i-th generator of the zonotope z
        # # h_{i}^{j} is the j-th component in the vector of the i-th generator of the zonotope z, j = 1, 2, ..., n
        # for j in range(0, n):
        #     for i in range(0, 2*n):
        #         sum += abs(h[j, i])

        #     for k in range(0, n):
        #         g[k, j] = sum

        # # Add the generators g to the generators h_keep
        # for i in range(0, n):
        #     h_keep = np.append(h_keep, g[:,i].reshape(-1,1), axis = 1)

        return Zonotope(z.C, h_keep)

    def hypercube(self, b_r):
        """
        This function returns a zonotope that represents the hypercube of side length 2*b_r
        """
        c = np.zeros((2, 1))
        g = np.array([[b_r, 0.0], [0.0, b_r]])

        return Zonotope(c, g)
