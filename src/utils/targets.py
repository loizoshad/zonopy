import numpy as np
from utils.sets.hybrid_zonotopes import HybridZonotope



class Targets:

    @property
    def exitparking(self):
        # Vertical Road Sections (Left)
        Gc = np.diag(np.array([ 0.15  , 0.1]))
        Gb = np.zeros((2, 0))
        c = np.array([ [1.65], [0.0]])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)


