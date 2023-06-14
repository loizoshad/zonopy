import numpy as np

from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations

zono_op = ZonoOperations()


n = 4
ng = 4; nb = 0; nc = 0
c = np.array([0.0, 
              0.0, 
              0.0, 
              0.0
            ])
Ac = np.zeros((nc, ng))
Ab = np.zeros((nc, nb))
b = np.zeros((nc, 1))
Gc = np.array([
    [5.0, 0.0, 0.0, 0.0],
    [0.0, 5.0, 0.0, 0.0],
    [0.0, 0.0, 5.0, 0.0],
    [0.0, 0.0, 0.0, 5.0]
])
Gb = np.zeros((n, nb))

my_hz = HybridZonotope(Gc, Gb, c, Ac, Ab, b)


p = np.array([
    [1.75],
    [0.0],
    [0.0],
    [0.5]
])

print(zono_op.is_inside_hz(my_hz, p))