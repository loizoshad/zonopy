import numpy as np

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
            self.vx_max,
            self.vy_max
        ])
        self.v_min = -self.v_max

        a_max = 5.0 # v1
        a_max = 1.0 # v2

        self.a_max = np.array([
            a_max,    # x - acceleration
            a_max     # y - acceleration
        ])  # m/s^2 Maximum permissible acceleration


        self.dt = 0.2       # [s] Time step

        self.A = np.array([
            [1.0, 0.0, self.dt, 0.0    ],     # x - position
            [0.0, 1.0, 0.0    , self.dt],     # y - position
            [0.0, 0.0, 1.0    , 0.0    ],     # x - velocity
            [0.0, 0.0, 0.0    , 1.0    ],     # y - velocity
        ])
        self.B = np.array([
            [       0.0         ,             0.0      ],     # x - position
            [       0.0         ,             0.0      ],     # y - position
            [self.a_max[0]*self.dt,           0.0      ],     # x - velocity
            [       0.0         , self.a_max[1]*self.dt],     # y - velocity
        ])



    