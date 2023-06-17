'''
The properties of the constrained zonotope class follow the Scott's, et al. work
'Constrained zonotopes: A new tool for set-based estimation and fault detection'

References:

[1] Liren Yang, et al. 'Efficient Backward Reachability Using the Minkowski Difference of Constrained Zonotopes'
# [?] Scott, et al. 'Constrained zonotopes: A new tool for set-based estimation and fault detection'
'''

import numpy as np
import cdd as pcdd # Enumerate vertices of a polytope
from scipy.spatial import ConvexHull
from scipy.spatial import HalfspaceIntersection
from scipy.optimize import linprog

class ConstrainedZonotope:
    def __init__(self, G: np.ndarray, C: np.ndarray, A: np.ndarray, b: np.ndarray) -> None:
        '''
        Constrained Zonotope CZ = (G, C, A, b)

        - G: generators of the constrained zonotope
            - [g_1, g_2, ..., g_ng] where g_1, g_2, ..., g_ng are column vectors of dimension n
        - C: center of the constrained zonotope
            - [c1, c2, ..., cn]^T where c1, c2, ..., cn are scalars
        - A: matrix of linear constraints of dimensions (nc, ng), nc is the number of constraints
        - b: vector of linear constraints of dimensions (nc, 1), nc is the number of constraints
        '''
        self.C = C.reshape(-1, 1)   # Reshape center
        self.G = G
        self.A = A
        self.b = b

    @property
    def dim(self) -> int:
        return self.C.shape[0]

    @property
    def ng(self) -> int:
        '''
        Number of generators
        '''
        return self.G.shape[1]

    @property
    def nc(self) -> int:
        '''
        Number of constraints
        '''
        return self.A.shape[0]

    @property
    def order(self) -> int:
        return (self.ng - self.nc)/self.dim

    def g2v(self):
        '''
        See work in [1] for details
        '''
        Ap = np.concatenate([
            self.A,
            -self.A,
            np.eye(self.ng),
            -np.eye(self.ng)
        ])
        bp = np.concatenate([
            self.b,
            -self.b,
            np.ones((2*self.ng, 1))
        ])

        vertices, rays, empty_flag = self.h2v(Ap, bp)

        if empty_flag == True:
            return np.zeros((0, self.dim)), np.zeros((0, self.dim)), empty_flag

        # Apply the affine transformation
        vertices = self.C + self.G @ vertices.T
        vertices = vertices.T


        if rays.size == 0:
            rays = self.G @ rays.T
            rays = rays.T

        # Remove duplicate vertices
        vertices = np.unique(vertices, axis=0)

        if vertices.shape[0] < 3:
            return np.zeros((0, self.dim)), np.zeros((0, self.dim)), True

        # Remove vertices not on the edge of the convex hull
        # This is done for visualization purposes
        hull = ConvexHull(vertices)
        vertices = vertices[hull.vertices]

        return vertices, rays, False

    def h2v(self, A, b):
        '''
        Given a Polytope defined by Ax <= b
        Return the vertices and rays of the Polytope
        '''
        M = np.hstack( (b, -A) )

        # Convert to cdd format
        P = pcdd.Matrix(M)
        P.rep_type = pcdd.RepType.INEQUALITY
        P = pcdd.Polyhedron(P)
        Q = P.get_generators()
        # Convert to numpy array
        Q = np.array(Q)

        # Check if Q is empty
        if Q.size == 0:
            vertices = None
            rays = None
            empty_flag = True
        else:
            # Extract vertices and rays
            vertices = Q[Q[:, 0] == 1, 1:]
            rays = Q[Q[:, 0] == 0, 1:]
            empty_flag = False

        return vertices, rays, empty_flag

