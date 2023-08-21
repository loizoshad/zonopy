import numpy as np
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations



class Car1:
    def __init__(self) -> None:
        self.intersections = Intersections().intersections
        self.zono_op = ZonoOperations()
        self.dynamics = DynamicsModel4D()
        self.step_size = 0.1
        self.lw = self.step_size
        self.lh_v = 2.1 - 0.2         # Lane height of vertical lanes [m]
        self.lh_h = 2.8         # Lane height of horizontal lanes [m]
        self.region = 12

    @property
    def bounds(self):
        bounds = np.array([
            [-1.0, 0.1],
            [-0.65, 0.15]
        ])

        return bounds

    @property
    def state_space2D(self):
        # Vertical Road Sections (Left)
        Gc = np.diag(np.array([ self.lw/2  , self.lh_v/2]))
        Gb = np.array([ 
            [0.9, 0.45], 
            [0.0 , 0.0]
            ])
        c = np.array([ [0.0], [0.0]])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 2))
        b = np.zeros((0, 1))

        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Horizontal Road Sections (Exterior)
        Gc = np.diag(np.array([ self.lh_h/2  , self.lw/2]))
        Gb = np.array([ [0.0], [0.9]])
        c = np.array([ [0.0], [0.0]])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_h_ext = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Horizontal Road Sections (Middle)
        Gc = np.diag(np.array([ self.lh_h/2 + 0.2  , self.lw/2]))
        Gb = np.zeros((2, 0))
        c = np.array([ [0.2], [0.0]])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_h_mid = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_h = self.zono_op.union_hz_hz_v2(road_h_ext, road_h_mid)
        road_h = self.zono_op.redundant_c_gc_hz_v2(road_h)
        road_h = self.zono_op.redundant_c_gc_hz_v1(road_h)

        road = self.zono_op.union_hz_hz_v2(road_v, road_h)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road)

        return road

    @property
    def state_space4D(self):
        # Vertical Road Sections (Left)
        Gc = np.diag(np.array([ self.lw/2  , self.lh_v/2, 0.0, 1e-4 ]))
        # Gc = np.diag(np.array([ self.lw/2  , self.lh_v/2, 1.0, 1e-4 ]))
        Gb = np.array([ [0.45], [0.0], [0.0], [-1.0] ])
        c = np.array([ [-0.9], [0.0], [ 0.0], [ 0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_v_left = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections (Right)
        Gc = np.diag(np.array([ self.lw/2  , self.lh_v/2, 0.0, 1e-4 ]))
        # Gc = np.diag(np.array([ self.lw/2  , self.lh_v/2, 1.0, 1e-4 ]))
        Gb = np.array([ [0.45], [0.0], [0.0], [-1.0] ])
        c = np.array([ [0.9], [0.0], [ 0.0], [ 0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_v_right = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Horizontal Road Sections (Exterior)
        Gc = np.diag(np.array([ self.lh_h/2  , self.lw/2, 1e-4, 0.0 ]))
        # Gc = np.diag(np.array([ self.lh_h/2  , self.lw/2, 1e-4, 1.0 ]))
        Gb = np.array([ [0.0], [0.9], [1.0], [0.0] ])
        c = np.array([ [0.0], [0.0], [ 0.0], [ 0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_h_ext = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # # Horizontal Road Sections (Middle)
        Gc = np.diag(np.array([ self.lh_h/2 + 0.2  , self.lw/2, 1e-4, 0.0 ]))
        # Gc = np.diag(np.array([ self.lh_h/2 + 0.2  , self.lw/2, 1e-4, 1.0 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.2], [0.0], [ 1.0], [ 0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_h_mid = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_v = self.zono_op.union_hz_hz_v2(road_v_left, road_v_right)
        road_v = self.zono_op.redundant_c_gc_hz_v2(road_v)
        road_v = self.zono_op.redundant_c_gc_hz_v1(road_v)

        road_h = self.zono_op.union_hz_hz_v2(road_h_ext, road_h_mid)
        road_h = self.zono_op.redundant_c_gc_hz_v2(road_h)
        road_h = self.zono_op.redundant_c_gc_hz_v1(road_h)

        road = self.zono_op.union_hz_hz_v2(road_v, road_h)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road)

        return road
    
    @property
    def state_spaceFRS(self):
        # Find which i_xy intersection of the class Intersection has as a 'coming' road section the region 'self.region'
        for intersection in self.intersections:
            if self.region in intersection['coming']:
                return intersection['ss']

    @property
    def state_spaceFRS_OLD(self):
        '''
        4D space for FRS computation

        This version contains the fully detailed state space where intersections are distinguished from the lanes
        '''
        # Vertical Road Sections 1
        Gc = np.diag(np.array([ self.lw/2  , 0.4, 0.0, 1e-4 ]))
        Gb = np.array([ 
            [0.9, 0.0 ], 
            [0.0, 0.45], 
            [0.0, 0.0 ], 
            [0.0, 0.0 ] 
            ])
        c = np.array([ [-0.45], [0.0], [ 0.0], [1.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 2))
        b = np.zeros((0, 1))

        road_v_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections 2
        Gc = np.diag(np.array([ self.lw/2  , 0.4, 0.0, 1e-4 ]))
        Gb = np.array([ 
            [0.9, 0.0 ], 
            [0.0, 0.45], 
            [0.0, 0.0 ], 
            [0.0, 0.0 ] 
            ])
        c = np.array([ [0.45], [0.0], [ 0.0], [-1.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 2))
        b = np.zeros((0, 1))

        road_v_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_v = self.zono_op.union_hz_hz_v2(road_v_1, road_v_2)
        road_v = self.zono_op.redundant_c_gc_hz_v2(road_v)
        road_v = self.zono_op.redundant_c_gc_hz_v1(road_v)

        # Horizontal Road Sections 1
        Gc = np.diag(np.array([ 0.4, self.lw/2, 1e-4, 0.0 ]))
        Gb = np.array([ 
            [0.45, 0.45, 0.0 ], 
            [0.0 , 0.0 , 0.45], 
            [0.0 , 0.0 , 0.0 ], 
            [0.0 , 0.0 , 0.0 ]
            ])
        c = np.array([ [0.0], [0.45], [1.0], [0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 3))
        b = np.zeros((0, 1))

        road_h_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Horizontal Road Sections 2
        Gc = np.diag(np.array([ 0.4, self.lw/2, 1e-4, 0.0 ]))
        Gb = np.array([ 
            [0.45, 0.45], 
            [0.0 , 0.0 ], 
            [0.0 , 0.0 ], 
            [0.0 , 0.0 ]
            ])
        c = np.array([ [0.0], [-0.9], [-1.0], [0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 2))
        b = np.zeros((0, 1))

        road_h_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_h = self.zono_op.union_hz_hz_v2(road_h_1, road_h_2)
        road_h = self.zono_op.redundant_c_gc_hz_v2(road_h)
        road_h = self.zono_op.redundant_c_gc_hz_v1(road_h)

        road = self.zono_op.union_hz_hz_v2(road_v, road_h)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road)



        # Intersection Road Sections 1
        Gc = np.diag(np.array([ 0.05, 0.05, 1e-4, 1e-4 ]))
        Gb = np.array([ 
            [0.9, 0.0 ], 
            [0.0, 0.45], 
            [0.0, 0.0 ], 
            [0.0, 0.0 ]
            ])
        c = np.array([ [-0.45], [0.45], [1.0], [1.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 2))
        b = np.zeros((0, 1))

        road_i_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Intersection Road Sections 2
        Gc = np.diag(np.array([ 0.05, 0.05, 1e-4, 1e-4 ]))
        Gb = np.array([ 
            [0.9, 0.0 ], 
            [0.0, 0.45], 
            [0.0, 0.0 ], 
            [0.0, 0.0 ]
            ])
        c = np.array([ [0.45], [0.45], [1.0], [-1.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 2))
        b = np.zeros((0, 1))

        road_i_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_i_12 = self.zono_op.union_hz_hz_v2(road_i_1, road_i_2)
        road_i_12 = self.zono_op.redundant_c_gc_hz_v2(road_i_12)
        road_i_12 = self.zono_op.redundant_c_gc_hz_v1(road_i_12)

        # Intersection Road Sections 3
        Gc = np.diag(np.array([ 0.05, 0.05, 1e-4, 1e-4 ]))
        Gb = np.array([ 
            [0.9], 
            [0.0],
            [0.0], 
            [0.0]
            ])
        c = np.array([ [-0.45], [-0.9], [-1.0], [1.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_i_3 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Intersection Road Sections 4
        Gc = np.diag(np.array([ 0.05, 0.05, 1e-4, 1e-4 ]))
        Gb = np.array([ 
            [0.9], 
            [0.0],
            [0.0], 
            [0.0]
            ])
        c = np.array([ [0.45], [-0.9], [-1.0], [0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_i_4 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road_i_34 = self.zono_op.union_hz_hz_v2(road_i_3, road_i_4)
        road_i_34 = self.zono_op.redundant_c_gc_hz_v2(road_i_34)
        road_i_34 = self.zono_op.redundant_c_gc_hz_v1(road_i_34)

        road_i = self.zono_op.union_hz_hz_v2(road_i_12, road_i_34)
        road_i = self.zono_op.redundant_c_gc_hz_v2(road_i)
        road_i = self.zono_op.redundant_c_gc_hz_v1(road_i)

        road = self.zono_op.union_hz_hz_v2(road, road_i)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road)

        return road
        
    @property
    def state_spaceFRS_2_OLD(self):
        '''
        4D space for FRS computation
        '''

        # Horizontal Road Sections
        Gc = np.diag(np.array([ 0.85, self.lw/2, 0.15, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [0.0], [0.85], [-0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections 2
        Gc = np.diag(np.array([ self.lw/2  , 0.85, 0.5, 0.15 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [0.0], [ 0.5], [-0.85] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        road = self.zono_op.union_hz_hz_v2(road_v, road_h)
        road = self.zono_op.redundant_c_gc_hz_v2(road)
        road = self.zono_op.redundant_c_gc_hz_v1(road)

        return road


    @property
    def initial_space4D(self):
        # Gc = np.diag(np.array([ 3*self.step_size/2, self.lw/2, 1.0, 1.0 ]))
        # Gb = np.zeros((4, 0))
        # c = np.array([ [-0.75], [0.0], [ 0.0], [0.0] ])
        # Ac = np.zeros((0, 4))
        # Ab = np.zeros((0, 0))
        # b = np.zeros((0, 1))

        Gc = np.diag(np.array([ 3*self.step_size/2, self.lw/2, 0.5, 0.0 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.75], [0.0], [ 0.5], [0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))


        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)  

    @property
    def initial_space2D(self):
        Gc = np.diag(np.array([ 3*self.step_size/2, self.lw/2]))
        Gb = np.zeros((2, 0))
        c = np.array([ [-0.75], [0.0]])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)        

    @property
    def input_space(self):
        # Maximum rate of change in velocity (acceleration)
        ng = 2; nc = 0; nb = 0

        Gc = np.array([
            [1.0, 0.0],
            [0.0, 1.0]
        ])

        c = np.array([ [0.0], [0.0] ])

        Gb = np.zeros((ng, nb))
        Ac = np.zeros((nc, ng))
        Ab = np.zeros((nc, nb))
        b = np.zeros((nc, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b) 



    def get_state2D(self):
        pass

    def get_state4D(self):
        pass

    def set_state2D(self):
        pass

    def set_state4D(self):
        pass

    def take_action(self):
        '''
        Move the car one time step forward
        '''
        pass



class Intersections:
    '''
    coming: Integer number representing the roads that can immediately reach the intersection point
    The numbering system can be found in [...]
    ss: State space relevant to the intersection
    '''
    
    def __init__(self) -> None:
        self.zono_op = ZonoOperations()
        self.step_size = 0.1

        # Define list containing all i_xy intersections. This list should adapt to the intersections defined in this class automatically
        # self.intersections = [self.i_00, self.i_01, self.i_11]
        self.intersections = []

        for x in range(0, 2):
            for y in range(0, 2):
                self.intersections.append(getattr(self, f'i_{x}{y}') )


    @property
    def i_00(self):
        # Horizontal Road Sections
        Gc = np.diag(np.array([ 0.95, self.step_size/2, 0.15, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [0.9], [0.85], [-0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections
        Gc = np.diag(np.array([ self.step_size/2  , 0.5, 0.5, 0.15 ]))
        Gb = np.array([ [0.45], [0.0], [0.0], [-0.85] ])
        c = np.array([ [-0.9], [0.45], [ 0.5], [0.0] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 1))
        b = np.zeros((0, 1))

        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ss = self.zono_op.union_hz_hz_v2(road_v, road_h)
        ss = self.zono_op.redundant_c_gc_hz_v2(ss)
        ss = self.zono_op.redundant_c_gc_hz_v1(ss)        

        i_00 = {
            'coming': [1],
            'ss': ss
        }

        return i_00
    
    @property
    def i_01(self):
        i_01 = {
            'coming': [9],
            'ss': None
        }

        return i_01
    
    @property
    def i_10(self):
        i_10 = {
            'coming': [2],
            'ss': None
        }

        return i_10
    

    @property
    def i_11(self):

        # Horizontal Road Sections
        Gc = np.diag(np.array([ 0.85, self.step_size/2, 0.15, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [0.0], [0.85], [-0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections
        Gc = np.diag(np.array([ self.step_size/2  , 0.85, 0.5, 0.15 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [0.0], [ 0.5], [-0.85] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ss = self.zono_op.union_hz_hz_v2(road_v, road_h)
        ss = self.zono_op.redundant_c_gc_hz_v2(ss)
        ss = self.zono_op.redundant_c_gc_hz_v1(ss)        
        
        i_11 = {
            'coming': [3, 12],
            'ss': ss
        }

        return i_11








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
        self.dt = 0.1       # [s] Time step (10 Hz)
        
        self.A = np.array([
            [1.0, 0.0],     # x - position
            [0.0, 1.0],     # y - position
        ])
        self.B = np.array([
            [self.vx_max*self.dt,          0.0       ],     # x - velocity
            [       0.0         , self.vy_max*self.dt],     # y - velocity
        ])

class DynamicsModel4D:
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

        a_max = 4.0 # [m/s^2]
        
        self.dt = 0.1       # [s] Time step

        self.A = np.array([
            [1.0, 0.0, self.dt, 0.0    ],     # x - position
            [0.0, 1.0, 0.0    , self.dt],     # y - position
            [0.0, 0.0, 1.0    , 0.0    ],     # x - velocity
            [0.0, 0.0, 0.0    , 1.0    ],     # y - velocity
        ])
        self.B = np.array([
            [       0.0         ,     0.0      ],     # x - position
            [       0.0         ,     0.0      ],     # y - position
            [   a_max*self.dt   ,     0.0      ],     # x - velocity
            [       0.0         , a_max*self.dt],     # y - velocity
        ])