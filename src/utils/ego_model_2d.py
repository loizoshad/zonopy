import numpy as np
from utils.sets.hybrid_zonotopes import HybridZonotope

class DynamicsModel:
    '''
    This class defines a discrete-time linear dynamics model without disturbances.

    x_{k+1} = Ax_k + Bu_k

    In addition this class contains information about constraints on the control inputs
    
    '''
    def __init__(self) -> None:
        self.vx_max = 1.0   # m/s Maximum permissible velocity in x-axis
        self.vy_max = 1.0   # m/s Maximum permissible velocity in y-axis
        self.v_max = np.array([
            [self.vx_max],
            [self.vy_max]
        ])
        self.v_min = -self.v_max
        # self.dt = 0.1       # [s] Time step
        self.dt = 0.05          # [s] Time step (This time step size, follows the step size of the environment)
        self.W = self.get_disturbance()
        
        self.A = np.array([
            [1.0, 0.0],     # x - position
            [0.0, 1.0],     # y - position
        ])
        self.B = np.array([
            [self.vx_max*self.dt,          0.0       ],     # x - velocity
            [       0.0         , self.vy_max*self.dt],     # y - velocity
        ])

        self.max_w = np.array([
            [0.0],
            [0.0]
        ])

    def get_disturbance(self):
        '''
        This function returns the disturbance w_k
        '''
        w = 0.0 * self.dt

        n = 2
        ng = 2; nc = 0; nb = 0
        
        Gc = np.array([
            [  w, 0.0],
            [0.0,   w]
        ])
        Gb = np.zeros((n, nb))
        c = np.array([
            [0.0],
            [0.0]
        ])

        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)    