import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull


class Zonotope:
    def __init__(self, C: np.ndarray, G: np.ndarray) -> None:
        """
        Zonotope: Z = (C, G)

        - C: center of the zonotope
            - [c1, c2, ..., cn]^T where c1, c2, ..., cn are scalars
        - G: generators of the zonotope
            - [g_1, g_2, ..., g_ng] where g_1, g_2, ..., g_ng are column vectors of dimension n
        """
        assert (
            C.shape[0] == G.shape[0]
        ), f"Center and generators must have the same dimensionality, whereas \
                                            C.shape[0] = {C.shape[0]} and G.shape[0] = {G.shape[0]}."

        self.C = C.reshape(-1, 1)  # Reshape center
        self.G = G

    @property
    def dim(self) -> int:
        return self.C.shape[0]

    @property
    def order(self) -> int:
        return self.G.shape[1] / self.G.shape[0]

    @property
    def ng(self) -> int:
        return self.G.shape[1]

    def g2v(self):
        """
        - Computes the vertices of the zonotope as [v1, v2, ..., vn] where v1, v2, ..., vn are column vectors

        - Compute all 2^self.ng possible combinations of the weights (-1, 1)
        - Example: 2 generators: 2^n = 4
            Decimal -> Binary -> Weights
            0 -> 00 -> [-1, -1]
            1 -> 01 -> [ 1,  1]
            2 -> 10 -> [ 1, -1]
            3 -> 11 -> [-1,  1]
        """

        p = self.ng  # Number of generators
        num_vertices = 2**p  # Number of vertices
        # Init Vertex matrix
        vertices = np.zeros((self.dim, num_vertices))  # Matrix of vertices

        for i in range(num_vertices):
            b = np.binary_repr(i, width=p)  # Binary value of i
            # b = bin(i)[2:].zfill(p)                           # Binary value of i
            w = np.array([1 if c == "1" else -1 for c in b])  # Compute weights
            vertices[:, i] = self.C.reshape(-1) + (self.G @ w)  # Compute vertex

        # Compute the convex hull of the vertices
        vertices = np.concatenate((vertices, self.C), axis=1)  # Add the center to the vertices
        hull = ConvexHull(vertices.T)

        return vertices[:, hull.vertices]  # Return the vertices of the convex hull
