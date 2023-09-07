import numpy as np
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.operations.operations import ZonoOperations


def cars_input_space():
    # Maximum rate of change in velocity (acceleration)
    ng = 2; nc = 0; nb = 0

    Gc = np.array([
        [1.0, 0.0],
        [0.0, 1.0]
    ])

    c = np.array([ [0.0], [0.0] ]); Gb = np.zeros((ng, nb))
    Ac = np.zeros((nc, ng)); Ab = np.zeros((nc, nb)); b = np.zeros((nc, 1))

    return HybridZonotope(Gc, Gb, c, Ac, Ab, b) 

step_size = 0.05

# t = 1
car_1_c = np.array([ [-1.35], [0.5], [0.5], [0.95] ])
car_2_c = np.array([ [-0.55], [0.0], [ 0.95], [0.0] ])
car_3_c = np.array([ [-0.45], [0.65], [ 0.0], [-0.95] ])
car_4_c = np.array([ [0.45], [-0.2], [ 0.0], [0.95] ])
car_5_c = np.array([ [1.05], [0.9], [ 0.95], [0.0] ])
#
car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.25, 0.05 ]))
car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))

# # t = 2
# car_1_c = np.array([ [-1.35], [0.55], [0.5], [0.95] ])
# car_2_c = np.array([ [-0.50], [0.0], [ 0.95], [0.0] ])
# car_3_c = np.array([ [-0.45], [0.60], [ 0.0], [-0.95] ])
# car_4_c = np.array([ [0.45], [-0.15], [ 0.0], [0.95] ])
# car_5_c = np.array([ [1.10], [0.9], [ 0.95], [0.0] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.25, 0.05 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))

# # t = 3
# car_1_c = np.array([ [-1.35], [0.60], [0.5], [0.95] ])
# car_2_c = np.array([ [-0.45], [0.0], [ 0.95], [0.0] ])
# car_3_c = np.array([ [-0.45], [0.55], [ 0.0], [-0.95] ])
# car_4_c = np.array([ [0.45], [-0.10], [ 0.0], [0.95] ])
# car_5_c = np.array([ [1.15], [0.9], [ 0.95], [0.0] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.25, 0.05 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))

# # t = 4
# car_1_c = np.array([ [-1.35], [0.65], [0.00], [0.95] ])
# car_2_c = np.array([ [-0.40], [0.00], [0.95], [0.00] ])
# car_3_c = np.array([ [-0.45], [0.50], [ 0.0], [-0.95]])
# car_4_c = np.array([ [0.45], [-0.05], [ 0.0], [0.95] ])
# car_5_c = np.array([ [1.35], [0.85], [ 0.0], [-0.95] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.25, 0.05 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))

# # t = 5
# car_1_c = np.array([ [-1.35], [0.70], [0.00], [ 0.95] ])
# # car_2_c = np.array([ [-0.35], [0.00], [0.95], [ 0.00] ])
# car_2_c = np.array([ [-0.375], [0.00], [0.95], [ 0.00] ])
# car_3_c = np.array([ [-0.45], [0.45], [0.00], [-0.95] ])
# car_4_c = np.array([ [ 0.45], [0.00], [0.00], [ 0.95] ])
# car_5_c = np.array([ [ 1.35], [0.80], [0.00], [-0.95] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.25, 0.05 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# # car_5_gc = np.array([ [ 0.10607, 0.10607, 0.000000, 0.000000], [ 0.10607, 0.10607, 0.000000, 0.000000], [ 0.00000, 0.00000, 0.035360, 0.000000],[ 0.00000, 0.00000, 0.000000, -0.03536] ])

# # t = 6
# car_1_c = np.array([ [-1.30], [0.90], [0.95], [ 0.00] ])
# car_2_c = np.array([ [-0.30], [0.00], [0.95], [ 0.00] ])
# car_3_c = np.array([ [-0.45], [0.40], [0.00], [-0.95] ])
# car_4_c = np.array([ [ 0.45], [0.05], [0.00], [ 0.95] ])
# car_5_c = np.array([ [ 1.35], [0.75], [0.00], [-0.95] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))

# # t = 7
# car_1_c = np.array([ [-1.25], [0.90], [0.00], [ 0.95] ])
# car_2_c = np.array([ [-0.25], [0.00], [0.95], [ 0.00] ])
# car_3_c = np.array([ [-0.45], [0.35], [0.00], [-0.95] ])
# car_4_c = np.array([ [ 0.45], [0.10], [0.00], [ 0.95] ])
# car_5_c = np.array([ [ 1.35], [0.70], [0.00], [-0.95] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.25, 0.05 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))

# # t = 8
# car_1_c = np.array([ [-1.20], [0.90], [0.95], [ 0.00] ])
# car_2_c = np.array([ [-0.20], [0.00], [0.95], [ 0.00] ])
# car_3_c = np.array([ [-0.45], [0.30], [0.00], [-0.95] ])
# car_4_c = np.array([ [ 0.45], [0.15], [0.00], [ 0.95] ])
# car_5_c = np.array([ [ 1.35], [0.65], [0.00], [-0.95] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))

# # t = 9
# car_1_c = np.array([ [-1.15], [0.90], [0.95], [ 0.00] ])
# car_2_c = np.array([ [-0.15], [0.00], [0.95], [ 0.00] ])
# car_3_c = np.array([ [-0.45], [0.25], [0.00], [-0.95] ])
# car_4_c = np.array([ [ 0.45], [0.20], [0.00], [ 0.95] ])
# car_5_c = np.array([ [ 1.35], [0.60], [0.00], [-0.95] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))

# # t = 10
# car_1_c = np.array([ [-1.10], [0.90], [0.95], [ 0.00] ])
# car_2_c = np.array([ [-0.10], [0.00], [0.95], [ 0.00] ])
# car_3_c = np.array([ [-0.45], [0.20], [0.00], [-0.95] ])
# car_4_c = np.array([ [ 0.45], [0.25], [0.00], [ 0.95] ])
# car_5_c = np.array([ [ 1.35], [0.55], [0.00], [-0.95] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))

# # t = 11
# car_1_c = np.array([ [-1.05], [0.90], [0.95], [ 0.00] ])
# car_2_c = np.array([ [-0.05], [0.00], [0.95], [ 0.00] ])
# car_3_c = np.array([ [-0.45], [0.15], [0.00], [-0.95] ])
# car_4_c = np.array([ [ 0.45], [0.30], [0.00], [ 0.95] ])
# car_5_c = np.array([ [ 1.35], [0.50], [0.00], [-0.95] ])
# #
# car_1_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_2_gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.00 ]))
# car_3_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_4_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))
# car_5_gc = np.diag(np.array([ 0.15, 0.15, 0.00, 0.05 ]))


class Car1:
    def __init__(self, t) -> None:
        self.intersections = Intersections(t).intersections
        self.zono_op = ZonoOperations()
        self.dynamics = DynamicsModel4D()
        self.lw = 0.1
        self.lh_v = 1.9         # Lane height of vertical lanes [m]
        self.lh_h = 2.8         # Lane height of horizontal lanes [m]

        self.set_bounds(np.array([ [-1.55, 0.05], [-0.60, 1.1] ]))

        self.initial_state = self.initial_space4D
        self.init_space()

        self.input_space = cars_input_space()
    
    def set_bounds(self, bounds):
        self.bounds = bounds

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

    def init_space(self):
        for intersection in self.intersections:
            p = np.array([ [self.initial_state.C[0, 0] ], [self.initial_state.C[1, 0]] ])
            if self.zono_op.is_inside_hz(intersection['current_road'], p):
                self.state_spaceFRS = intersection['ss']
                self.conflict_zone = intersection['conflict_zone']
                self.current_road = intersection['current_road']
                break
            else:
                self.state_spaceFRS = None
                self.conflict_zone = None
                self.current_road = None

    @property
    def initial_space4D(self):
        # Gc = np.diag(np.array([0.15, 0.15, 0.25, 0.05 ]))
        # c = np.array([ [-1.35], [-0.1], [0.5], [0.95] ])
        
        c = car_1_c
        Gc = car_2_gc
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)     

class Car2:
    def __init__(self, t) -> None:
        self.intersections = Intersections(t).intersections
        self.zono_op = ZonoOperations()
        self.dynamics = DynamicsModel4D()

        self.initial_state = self.initial_space4D
        self.input_space = cars_input_space()
        self.init_space()

        self.set_bounds(np.array([ [-1.05, 0.35], [-0.90, 0.3] ]))

    def set_bounds(self, bounds):
        self.bounds = bounds
    
    def init_space(self):
        for intersection in self.intersections:
            p = np.array([ [self.initial_state.C[0, 0] ], [self.initial_state.C[1, 0]] ])
            if self.zono_op.is_inside_hz(intersection['current_road'], p):
                self.state_spaceFRS = intersection['ss']
                self.conflict_zone = intersection['conflict_zone']
                self.current_road = intersection['current_road']
                break
            else:
                self.state_spaceFRS = None
                self.conflict_zone = None
                self.current_road = None

    @property
    def initial_space4D(self):
        # Gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.0 ]))
        # c = np.array([ [-0.65], [0.0], [ 0.95], [0.0] ])
        
        c = car_2_c
        Gc = car_2_gc
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)    

class Car3:
    def __init__(self, t) -> None:
        self.intersections = Intersections(t).intersections
        self.zono_op = ZonoOperations()
        self.dynamics = DynamicsModel4D()

        self.initial_state = self.initial_space4D
        self.init_space()
        self.input_space = cars_input_space()

        self.set_bounds(np.array([ [-0.75, 0.45], [-1.1, 1.1] ]))

    def set_bounds(self, bounds):
        self.bounds = bounds

    def init_space(self):
        for intersection in self.intersections:
            p = np.array([ [self.initial_state.C[0, 0] ], [self.initial_state.C[1, 0]] ])
            if self.zono_op.is_inside_hz(intersection['current_road'], p):
                self.state_spaceFRS = intersection['ss']
                self.conflict_zone = intersection['conflict_zone']
                self.current_road = intersection['current_road']
                break
            else:
                self.state_spaceFRS = None
                self.conflict_zone = None
                self.current_road = None

    @property
    def initial_space4D(self):
        # Gc = np.diag(np.array([ 0.15, 0.15, 0.0, 0.05 ]))
        # c = np.array([ [-0.45], [0.8], [ 0.0], [-0.95] ])
        
        c = car_3_c
        Gc = car_3_gc
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)    

class Car4:
    def __init__(self, t) -> None:
        self.intersections = Intersections(t).intersections
        self.zono_op = ZonoOperations()
        self.dynamics = DynamicsModel4D()

        self.initial_state = self.initial_space4D
        self.init_space()
        self.input_space = cars_input_space()

        self.set_bounds(np.array([ [-0.15, 1.95], [-1.1, 1.3] ]))

    def set_bounds(self, bounds):
        self.bounds = bounds

    def init_space(self):
        for intersection in self.intersections:
            p = np.array([ [self.initial_state.C[0, 0] ], [self.initial_state.C[1, 0]] ])
            if self.zono_op.is_inside_hz(intersection['current_road'], p):
                self.state_spaceFRS = intersection['ss']
                self.conflict_zone = intersection['conflict_zone']
                self.current_road = intersection['current_road']
                self.related_conflict_area = intersection['related_conflict_area']
                break
            else:
                self.state_spaceFRS = None
                self.conflict_zone = None
                self.current_road = None

    @property
    def initial_space4D(self):
        # Gc = np.diag(np.array([ 0.15, 0.15, 0.0, 0.05 ]))
        # c = np.array([ [0.45], [-0.2], [ 0.0], [0.95] ])

        c = car_4_c
        Gc = car_4_gc
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)    

class Car5:
    def __init__(self, t) -> None:
        self.intersections = Intersections(t).intersections
        self.zono_op = ZonoOperations()
        self.dynamics = DynamicsModel4D()

        self.initial_state = self.initial_space4D
        self.input_space = cars_input_space()
        self.init_space()

        self.set_bounds(np.array([ [0.75, 1.95], [-0.6, 1.1] ]))

    def set_bounds(self, bounds):
        self.bounds = bounds

    def init_space(self):
        for intersection in self.intersections:
            p = np.array([ [self.initial_state.C[0, 0] ], [self.initial_state.C[1, 0]] ])
            if self.zono_op.is_inside_hz(intersection['current_road'], p):
                self.state_spaceFRS = intersection['ss']
                self.conflict_zone = intersection['conflict_zone']
                self.current_road = intersection['current_road']
                break
            else:
                self.state_spaceFRS = None
                self.conflict_zone = None
                self.current_road = None

    @property
    def initial_space4D(self):
        # Gc = np.diag(np.array([ 0.15, 0.15, 0.05, 0.0 ]))
        # c = np.array([ [1.05], [0.9], [ 0.95], [0.0] ])

        c = car_5_c
        Gc = car_5_gc
        Gb = np.zeros((4, 0))
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        return HybridZonotope(Gc, Gb, c, Ac, Ab, b)    



class Intersections:
    '''
    coming: Integer number representing the roads that can immediately reach the intersection point
    The numbering system can be found in [...]
    ss: State space relevant to the intersection
    '''
    
    def __init__(self, t) -> None:
        self.t = t
        self.zono_op = ZonoOperations()

        # self.intersections = [self.i_02_l, self.i_02_d, self.i_11_u, self.i_11_l_1, self.i_11_l_2, self.i_12_l, self.i_12_d, self.i_13_u, self.i_13_l]
        self.intersections = [self.i_02_l, self.i_02_d, self.i_11_u, self.i_12_l, self.i_11_l, self.i_12_d, self.i_13_u, self.i_13_l]

    @property
    def i_02_l(self):
        '''
        i: Intersection - First row from top, third column from left
        l: Coming direction - Left
        '''
        ####################################################
        # State Space
        ####################################################        
        # Horizontal Road Sections
        Gc = np.diag(np.array([ 1.5, 0.15, 0.025, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.0], [0.9], [1.05], [-0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        ss = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Conflict Zone
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.15])); Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        
        c = np.array([ [0.45], [0.9] ])
        conflict_zone = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Current road
        ####################################################
        Gc = np.diag(np.array([ 0.45, 0.10]))
        Gb = np.zeros((2, 0))
        c = np.array([ [-0.15], [0.95] ])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        current_road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        i_02_l = {
            'ss': ss,
            'conflict_zone': [conflict_zone],
            'current_road': current_road
        }

        return i_02_l

    @property
    def i_02_d(self):
        '''
        i: Intersection - First row from top, third column from left
        d: Coming direction - Down
        '''
        ####################################################
        # State Space
        ####################################################        
        # Vertical Road Sections
        Gc = np.diag(np.array([0.15, 1.05, 0.5, 0.025 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.45], [0.6], [0.5], [1.05] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        ss = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Conflict Zone
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.15]))
        Gb = np.zeros((2, 0))
        c = np.array([ [0.45], [0.9] ])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        conflict_zone = HybridZonotope(Gc, Gb, c, Ac, Ab, b) 

        ####################################################
        # Current road
        ####################################################
        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        Gc = np.diag(np.array([ 0.15, 0.45]))
        c = np.array([ [0.45], [0.6] ])

        if self.t == 8 or self.t == 9 or self.t == 10 or self.t == 11 or self.t == 12:
            Gc = np.diag(np.array([ 0.15, 0.6]))
            c = np.array([ [0.45], [0.45] ])            
        current_road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        i_02_d = {
            'ss': ss,
            'conflict_zone': [conflict_zone],
            'current_road': current_road
        }

        return i_02_d

    @property
    def i_11_u(self):
        '''
        i: Intersection - Second row from top, second column from left
        u: Coming direction - up
        '''
        ####################################################
        # State Space
        ####################################################        
        # Horizontal Road Sections
        Gc = np.diag(np.array([ 1.05, 0.15, 0.025, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [0.0], [1.05], [-0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections
        Gc = np.diag(np.array([0.15, 1.05, 0.5, 0.025 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [0.0], [0.5], [-1.05] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ss = self.zono_op.union_hz_hz_v2(road_v, road_h)
        ss = self.zono_op.redundant_c_gc_hz_v2(ss)
        ss = self.zono_op.redundant_c_gc_hz_v1(ss)

        ####################################################
        # Conflict Zone
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.15])); Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        
        c = np.array([ [-0.45], [0.0] ])
        conflict_zone_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # c = np.array([ [0.45], [0.0] ])
        # conflict_zone_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        c = np.array([ [-0.45], [-0.9] ])
        conflict_zone_3 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        conflict_zones = [conflict_zone_1]

        if self.t >= 10 and self.t <= 25:
            conflict_zones = [conflict_zone_1, conflict_zone_3] 

        ####################################################
        # Current road
        ####################################################
        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        Gc = np.diag(np.array([ 0.15, 0.35]))
        c = np.array([ [-0.45], [0.5] ])

        if self.t >= 8 and self.t <= 21:
            Gc = np.diag(np.array([ 0.15, 0.6]))
            c = np.array([ [-0.45], [0.25] ])
        current_road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        i_11_u = {
            'ss': ss,
            # 'conflict_zone': [conflict_zone_1, conflict_zone_2, conflict_zone_3],
            'conflict_zone': conflict_zones,
            'current_road': current_road
        }

        return i_11_u

    @property
    def i_11_l(self):
        '''
        i: Intersection - Second row from top, second column from left
        l: Coming direction - left
        '''
        ####################################################
        # State Space
        ####################################################        
        # Horizontal Road Sections
        Gc = np.diag(np.array([ 1.05, 0.15, 0.025, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [0.0], [1.05], [-0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections
        Gc = np.diag(np.array([0.15, 1.05, 0.5, 0.025 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [0.0], [0.5], [-1.05] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ss = self.zono_op.union_hz_hz_v2(road_v, road_h)
        ss = self.zono_op.redundant_c_gc_hz_v2(ss)
        ss = self.zono_op.redundant_c_gc_hz_v1(ss)

        ####################################################
        # Conflict Zone
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.15])); Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        
        c = np.array([ [-0.45], [0.0] ])
        conflict_zone_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        c = np.array([ [0.45], [0.0] ])
        conflict_zone_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Current road
        ####################################################
        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        Gc = np.diag(np.array([ 0.45, 0.15]))
        c = np.array([ [-1.05], [0.0] ])
        current_road_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)
        
        Gc = np.diag(np.array([ 0.45, 0.1]))
        c = np.array([ [-0.15], [-0.05] ])
        current_road_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        current_road = self.zono_op.union_hz_hz_v2(current_road_1, current_road_2)
        current_road = self.zono_op.redundant_c_gc_hz_v2(current_road)
        current_road = self.zono_op.redundant_c_gc_hz_v1(current_road)

        conflict_zones = [conflict_zone_1]

        if self.t == 4 or self.t == 5:
            conflict_zones = [conflict_zone_1, conflict_zone_2]

        if self.t == 6 or self.t == 7 or self.t == 8 or self.t == 9 or self.t == 10:
            conflict_zones = [conflict_zone_2]


        i_11_l = {
            'ss': ss,
            'conflict_zone': conflict_zones,
            'current_road': current_road
        }

        return i_11_l


    @property
    def i_12_l(self):
        '''
        i: Intersection - Second row from top, third column from left
        l: Coming direction - Left
        '''
        ####################################################
        # State Space
        ####################################################        
        # Horizontal Road Sections
        Gc = np.diag(np.array([ 0.9, 0.15, 0.05, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.6], [0.0], [1.15], [0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        ss = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Conflict Zone
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.15])); Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        
        c = np.array([ [0.45], [0.0] ])
        conflict_zone_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        c = np.array([ [1.35], [0.0] ])
        conflict_zone_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # c = np.array([ [0.45], [0.9] ])
        # conflict_zone_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Current road
        ####################################################
        Gc = np.diag(np.array([ 0.3, 0.15]))
        Gb = np.zeros((2, 0))
        c = np.array([ [0.0], [0.0] ])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        current_road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        i_12_l = {
            'ss': ss,
            # 'conflict_zone': conflict_zone,
            'conflict_zone': [conflict_zone_1, conflict_zone_2],
            'current_road': current_road
        }

        return i_12_l  

    @property
    def i_12_d(self):
        '''
        i: Intersection - Second row from top, third column from left
        d: Coming direction - Down
        '''
        ####################################################
        # State Space
        ####################################################        
        # Horizontal Road Sections
        Gc = np.diag(np.array([ 1.05, 0.15, 0.025, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.45], [0.0], [1.05], [0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections
        Gc = np.diag(np.array([0.15, 1.05, 0.5, 0.025 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.45], [0.3], [0.5], [1.05] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ss = self.zono_op.union_hz_hz_v2(road_v, road_h)
        ss = self.zono_op.redundant_c_gc_hz_v2(ss)
        ss = self.zono_op.redundant_c_gc_hz_v1(ss)

        ####################################################
        # Conflict Zone
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.15])); Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        
        c = np.array([ [0.45], [0.0] ])
        conflict_zone_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        c = np.array([ [1.35], [0.0] ])
        conflict_zone_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        c = np.array([ [0.45], [0.9] ])
        conflict_zone_3 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Current road
        ####################################################
        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        Gc = np.diag(np.array([ 0.15, 0.35]))
        c = np.array([ [0.45], [-0.5] ])
        current_road_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        Gc = np.diag(np.array([ 0.1, 0.45]))
        c = np.array([ [0.5], [0.3] ])
        current_road_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        current_road = self.zono_op.union_hz_hz_v2(current_road_1, current_road_2)
        current_road = self.zono_op.redundant_c_gc_hz_v2(current_road)
        current_road = self.zono_op.redundant_c_gc_hz_v1(current_road)

        conflict_zones = [conflict_zone_1, conflict_zone_2]

        ####################################################
        # Current road
        ####################################################
        Gb = np.zeros((2, 0)); Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))

        Gc = np.diag(np.array([ 0.15, 0.35]))
        c = np.array([ [0.45], [-0.5] ])
        related_conflict_area_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        related_conflict_area = related_conflict_area_1
        ####################################################
        # Cases
        ####################################################
        if self.t == 4 or self.t == 5:
            conflict_zones = [conflict_zone_1]

        if self.t == 6 or self.t == 7:
            conflict_zones = [conflict_zone_1, conflict_zone_3]

        if self.t == 8 or self.t == 9 or self.t == 10 or self.t == 11 or self.t == 12:
            conflict_zones = [conflict_zone_3]


        i_12_d = {
            'ss': ss,
            'conflict_zone': conflict_zones,
            'related_conflict_area': related_conflict_area,
            'current_road': current_road
        }


        return i_12_d


    @property
    def i_13_u(self):
        '''
        i: Intersection - Second row from top, fourth column from left
        u: Coming direction - Up
        '''
        ####################################################
        # State Space
        ####################################################        
        # Horizontal Road Sections
        Gc = np.diag(np.array([ 0.6, 0.15, 0.05, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.9], [0.9], [1.15], [-0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections
        Gc = np.diag(np.array([ 0.15, 0.6, 0.5, 0.05 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [1.35], [0.45], [0.5], [-1.15] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ss = self.zono_op.union_hz_hz_v2(road_v, road_h)
        ss = self.zono_op.redundant_c_gc_hz_v2(ss)
        ss = self.zono_op.redundant_c_gc_hz_v1(ss)    

        ####################################################
        # Conflict Zone
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.15]))
        Gb = np.zeros((2, 0))
        c = np.array([ [1.35], [0.0] ])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        conflict_zone = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Current road
        ####################################################
        Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))

        # Gc = np.diag(np.array([ 0.15, 0.45]))
        # c = np.array([ [1.35], [0.6] ])
        Gc = np.diag(np.array([ 0.15, 0.6]))
        c = np.array([ [1.35], [0.45] ])
        current_road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        i_13_u = {
            'ss': ss,
            'conflict_zone': [conflict_zone],
            'current_road': current_road
        }

        return i_13_u        

    @property
    def i_13_l(self):
        '''
        i: Intersection - Second row from top, fourth column from left
        l: Coming direction - Left
        '''
        ####################################################
        # State Space
        ####################################################        
        # Horizontal Road Sections
        Gc = np.diag(np.array([ 0.45, 0.15, 0.05, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [1.05], [0.0], [1.15], [-0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        ss = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Conflict Zone
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.15]))
        Gb = np.zeros((2, 0))
        c = np.array([ [1.35], [0.0] ])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        conflict_zone = HybridZonotope(Gc, Gb, c, Ac, Ab, b) 

        ####################################################
        # Current road
        ####################################################
        Gc = np.diag(np.array([ 0.45, 0.15]))
        Gb = np.zeros((2, 0))
        c = np.array([ [1.05], [0.0] ])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        current_road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        i_13_l = {
            'ss': ss,
            'conflict_zone': [conflict_zone],
            'current_road': current_road
        }

        return i_13_l  

    
    


    @property
    def i_21_u(self):
        '''
        i: Intersection - Third row from top, second column from left
        u: Coming direction - up
        '''
        ####################################################
        # State Space
        ####################################################
        # Vertical Road Sections
        Gc = np.diag(np.array([0.15, 1.05, 0.5, 0.05 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [-0.45], [-0.6], [-0.5], [-1.15] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        ss = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ####################################################
        # Conflict Zone
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.15]))
        Gb = np.zeros((2, 0))
        c = np.array([ [-0.45], [-0.9] ])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        conflict_zone = HybridZonotope(Gc, Gb, c, Ac, Ab, b) 

        ####################################################
        # Current road
        ####################################################
        Gc = np.diag(np.array([ 0.15, 0.45]))
        Gb = np.zeros((2, 0))
        c = np.array([ [-0.45], [-0.6] ])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        current_road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        i_21_u = {
            'ss': ss,
            'conflict_zone': [conflict_zone],
            'current_road': current_road
        }

        return i_21_u

    @property
    def i_21_r(self):
        '''
        i: Intersection - Third row from top, second column from left
        r: Coming direction - right
        '''
        ####################################################
        # State Space
        ####################################################        
        # Horizontal Road Sections
        Gc = np.diag(np.array([ 0.9, 0.15, 0.05, 0.5 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.3], [-0.9], [-1.15], [0.5] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        road_h = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        # Vertical Road Sections
        Gc = np.diag(np.array([0.15, 1.05, 0.5, 0.05 ]))
        Gb = np.zeros((4, 0))
        c = np.array([ [0.45], [0.0], [0.5], [1.15] ])
        Ac = np.zeros((0, 4))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        road_v = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        ss = self.zono_op.union_hz_hz_v2(road_v, road_h)
        ss = self.zono_op.redundant_c_gc_hz_v2(ss)
        ss = self.zono_op.redundant_c_gc_hz_v1(ss)

        ####################################################
        # Conflict Zone
        ####################################################
        # Gc = np.diag(np.array([ 0.15, 0.15]))
        # Gb = np.zeros((2, 0))
        # c = np.array([ [-0.45], [-0.9] ])
        # Ac = np.zeros((0, 2))
        # Ab = np.zeros((0, 0))
        # b = np.zeros((0, 1))
        # conflict_zone = HybridZonotope(Gc, Gb, c, Ac, Ab, b) 

        Gc = np.diag(np.array([ 0.15, 0.15])); Gb = np.zeros((2, 0))
        Ac = np.zeros((0, 2)); Ab = np.zeros((0, 0)); b = np.zeros((0, 1))
        
        c = np.array([ [-0.45], [-0.9] ])
        conflict_zone_1 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        c = np.array([ [0.45], [0.0] ])
        conflict_zone_2 = HybridZonotope(Gc, Gb, c, Ac, Ab, b)        

        ####################################################
        # Current road
        ####################################################
        Gc = np.diag(np.array([ 0.75, 0.15]))
        Gb = np.zeros((2, 0))
        c = np.array([ [0.45], [-0.9] ])
        Ac = np.zeros((0, 2))
        Ab = np.zeros((0, 0))
        b = np.zeros((0, 1))
        current_road = HybridZonotope(Gc, Gb, c, Ac, Ab, b)

        i_21_r = {
            'ss': ss,
            # 'conflict_zone': conflict_zone,
            'conflict_zone': [conflict_zone_1, conflict_zone_2],
            'current_road': current_road
        }

        return i_21_r











class DynamicsModel:
    '''
    This class defines a discrete-time linear dynamics model without disturbances.

    x_{k+1} = Ax_k + Bu_k
    
    These dynamics are used for the propagation of the backward reachable set

    '''
    def __init__(self) -> None:
        self.vx_max = 1.0   # m/s Maximum permissible velocity in x-axis
        self.vy_max = 1.0   # m/s Maximum permissible velocity in y-axis

        # self.dt = 0.1       # [s] Time step (10 Hz)
        self.dt = step_size       # [s] Time step (20 Hz)
        
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

    These dynamics are used for the propagation of the forward reachable set
    
    '''
    def __init__(self) -> None:
        a_max = 4.5 # [m/s^2]
        
        # self.dt = 0.1       # [s] Time step (10 Hz)
        self.dt = step_size       # [s] Time step (20 Hz)

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














