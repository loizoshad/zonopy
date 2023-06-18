'''
This file contains a collection of example sets represented as zonotopes, constrained zonotopes and hybrid zonotopes.
'''

import numpy as np

from utils.sets.zonotopes import Zonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations



class SamplesZ:

    @property
    def set_1(self):
        c = np.array([  [0],
                        [0]
                    ])
        G = np.array([  [1, 0],
                        [0, 1]
                    ]) 

        return Zonotope(c, G)

    @property
    def set_2(self):
        c = np.array([  [0],
                        [0]
                    ])
        G = np.array([  [1, 0],
                        [1, 1]
                    ])

        return Zonotope(c, G)      


class SamplesCZ:
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()

    @property
    def set_1(self):
        G = np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        C = np.array([
            [0.0],
            [0.0]
        ])
        A = np.array([
            [1.0, 1.0, 1.0]
        ])
        b = np.array([
            [-1.0]
        ])        

        return ConstrainedZonotope(G, C, A, b)
    
    @property
    def set_2(self):
        G = np.array([
            [1.0, 0.0, 0.0, 0.1],
            [0.0, 1.0, 0.0, 0.8]
        ])        
        C = np.array([
            [0.0],
            [0.0]
        ])
        A = np.array([
            [-1.0, 1.0, 0.3, 1.0]
        ])
        b = np.array([
            [1.0]
        ])

        return ConstrainedZonotope(G, C, A, b)        


class SamplesHZ:

    @property
    def set_a(self):
        n = 2; ng = 3; nc = 0; nb = 0
        Gc = 0.1*np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)
    
    @property
    def set_b(self):
        n = 2; ng = 3; nc = 1; nb = 0
        Gc = 0.07*np.array([
            [1.5, -1.5, 0.5],
            [1.0, 0.5, -1.0]
        ])
        Gb = np.zeros((n, nb))
        C = 0.15*np.array([ [-1.2], [0.2] ])

        Ac = np.array([
            [1.0, 1.0, 1.0],
        ])

        Ab = np.zeros((nc, nb))

        b = np.array([ 
            [1.0] 
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)    

    @property
    def set_c(self):
        n = 2; ng = 4; nc = 3; nb = 0
        Gc = np.array([
            [1.5, -1.5, 0.5, 0.9],
            [1.0, 0.5, -1.0, 1.0]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])

        Ac = np.array([
            [0.8, 0.8, 0.8, 0.8],
            [5.0, 5.0, 5.0, 5.0],
            [5.0, 5.0, 5.0, 5.0]
        ])

        Ab = np.zeros((nc, nb))

        b = np.array([ 
            [0.8],
            [5.0],
            [5.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)  

    @property
    def set_d(self):
        n = 2; ng = 4; nc = 0; nb = 0
        Gc = np.array([
            [1.0, 0.0, 1.0, 0.5],
            [0.0, 1.0, 0.0, 0.5]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))


        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)       

    @property
    def set_e(self):
        n = 2; ng = 4; nc = 1; nb = 0
        Gc = np.array([
            [1.0, 0.5, 1.0, 1.6],
            [0.0, 1.0, 2.0, 2.1]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            # [1.0, 0.5, 0.1, 0.2]
            [1.0, 1.0, 1.0, 1.0]
        ])
        Ab = np.zeros((nc, nb))
        b = np.array([
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)
    
    @property
    def set_f(self):
        n = 2; ng = 7; nc = 1; nb = 0
        Gc = np.array([
            [1.0, 0.5, 1.0, 1.6, 0.4, 0.0, 0.5 ],
            [0.0, 1.0, 2.0, 2.1, 0.3, 0.1, 0.3 ]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            [1.0, 1.0, 2.0, 1.0, 1.0, 1.0, 1.0]
        ])
        Ab = np.zeros((nc, nb))
        b = np.array([
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)
    
    @property
    def set_g(self):
        n = 2; ng = 4; nc = 4; nb = 0
        Gc = 0.1*np.array([
            [1.5, 1.0, 1.6, 1.6],
            [0.0, 0.8, 1.2, 1.2]
        ])
        Gb = np.zeros((n, nb))
        C = np.array([ [0.0], [0.0] ])
        Ac = np.array([
            [1.2, 1.0, 0.3, 0.1],
            [2.4, 2.0, 0.6, 0.2],
            [1.2, 1.0, 0.1, 0.1],
            [2.4, 2.0, 0.4, 0.2]
        ])
        Ab = np.zeros((nc, nb))
        b = np.array([
            [0.5],
            [1.0],
            [0.5],
            [1.0]
        ])

        return HybridZonotope(Gc, Gb, C, Ac, Ab, b)    


