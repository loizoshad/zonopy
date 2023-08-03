import matplotlib.pyplot as plt
import numpy as np

from utils.environments.dynamic_env1 import DynamicEnv, StateSpaceSafe, DynamicObstacleSpace
from utils.sets.hybrid_zonotopes import HybridZonotope
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations
from utils.ego_model_4d import DynamicsModel

np.set_printoptions(edgeitems=10, linewidth=10000)
colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]
obs_color = (0.949, 0.262, 0.227, 0.6)

marker_size = 0.5 * 39.36           # 0.2m in inches
marker_size = marker_size**2        # area of the marker
first_time = True


def get_obstacle(road_down, road_right, input, car_down, car_right, A, B, W):
    car_down_1 = zono_op.one_step_frs_hz(X = road_down, U = input, I = car_down, A = A, B = B, W = W)     # Car down  # V3
    car_down = car_down_1
    print(f'- car_down:')
    car_down  = zono_op.cz_to_hz( zono_op.oa_cz_to_hypercube_tight_4d( zono_op.oa_hz_to_cz(car_down)))      # Simplify car down

    car_right_1 = zono_op.one_step_frs_hz(X = road_right, U = input, I = car_right, A = A, B = B, W = W)  # Car right # V3
    car_right_2 = zono_op.one_step_frs_hz(X = road_right, U = input, I = car_down, A = A, B = B, W = W)  # Car right # V3
    print(f'- car_right_1:')
    car_right_1 = zono_op.cz_to_hz( zono_op.oa_cz_to_hypercube_tight_4d( zono_op.oa_hz_to_cz(car_right_1)))     # Simplify car right_1
    print(f'- car_right_2:')
    car_right_2 = zono_op.cz_to_hz( zono_op.oa_cz_to_hypercube_tight_4d( zono_op.oa_hz_to_cz(car_right_2)))     # Simplify car right_2
    car_right = zono_op.union_hz_hz_v2(car_right_1, car_right_2)

    return car_down, car_right




def get_safe_space_down(car_down, space) -> HybridZonotope:
    G_d = np.zeros((2, 2))
    G_d[0, 0] = car_down.Gc[0, 0]; G_d[1, 1] = car_down.Gc[1, 1]
    C_d = np.zeros((2, 1))
    C_d[0, 0] = car_down.C[0, 0]; C_d[1, 0] = car_down.C[1, 0]
    cz_down = ConstrainedZonotope(G_d, C_d, np.zeros((0, G_d.shape[1])), np.zeros((0, 1)))
    safe_down = zono_op.complement_cz_to_hz(cz_down)

    # Add the velocity states
    Gc_d = np.block([
        [safe_down.Gc, np.zeros((safe_down.Gc.shape[0], 2))],
        [np.zeros((2, safe_down.Gc.shape[1])), np.eye(2)]
    ])
    #
    Gb_d = np.block([
        [safe_down.Gb],
        [np.zeros((2, safe_down.Gb.shape[1]))]
    ])
    #
    C_d = np.zeros((4, 1))
    C_d[0, 0] = safe_down.C[0, 0]; C_d[1, 0] = safe_down.C[1, 0]
    C_d[2, 0] = 0.0; C_d[3, 0] = 0.0  # Velocity states
    #
    Ac_d = np.block([
        [safe_down.Ac, np.zeros((safe_down.Ac.shape[0], 2))],
    ])
    Ab_d = safe_down.Ab
    b_d = safe_down.b

    safe_down = HybridZonotope(Gc_d, Gb_d, C_d, Ac_d, Ab_d, b_d)

    safe_space = safe_down
    safe_space = zono_op.intersection_hz_hz(safe_space, space)

    return safe_space

def get_safe_space_right(car_right, space):
    G_r = np.zeros((2, 2))
    G_r[0, 0] = car_right.Gc[0, 0]; G_r[1, 1] = car_right.Gc[1, 1]
    C_r = np.zeros((2, 1))
    C_r[0, 0] = car_right.C[0, 0]; C_r[1, 0] = car_right.C[1, 0]
    cz_right = ConstrainedZonotope(G_r, C_r, np.zeros((0, G_r.shape[1])), np.zeros((0, 1)))
    safe_right = zono_op.complement_cz_to_hz(cz_right)

    # Add the velocity states
    Gc_r = np.block([
        [safe_right.Gc, np.zeros((safe_right.Gc.shape[0], 2))],
        [np.zeros((2, safe_right.Gc.shape[1])), np.eye(2)]
    ])
    #
    Gb_r = np.block([
        [safe_right.Gb],
        [np.zeros((2, safe_right.Gb.shape[1]))]
    ])
    #
    C_r = np.zeros((4, 1))
    C_r[0, 0] = safe_right.C[0, 0]; C_r[1, 0] = safe_right.C[1, 0]
    C_r[2, 0] = 0.0; C_r[3, 0] = 0.0  # Velocity states
    #
    Ac_r = np.block([
        [safe_right.Ac, np.zeros((safe_right.Ac.shape[0], 2))],
    ])
    Ab_r = safe_right.Ab
    b_r = safe_right.b

    safe_right = HybridZonotope(Gc_r, Gb_r, C_r, Ac_r, Ab_r, b_r)

    safe_space = safe_right
    safe_space = zono_op.intersection_hz_hz(safe_space, space)

    return safe_space

def get_safe_space(car_right, car_down, space):
    if zono_op.is_empty_cz(zono_op.oa_hz_to_cz(car_right)):
        print(f'CAR RIGHT IS EMPTY')
        safe_space = get_safe_space_down(car_down, space)
    elif zono_op.is_empty_cz(zono_op.oa_hz_to_cz(car_down)):
        print(f'ROAD DOWN IS EMPTY')
        safe_space = get_safe_space_down(car_right, space)
    else:
        print(f'NEITHER CAR IS EMPTY')
        safe_space_down = get_safe_space_down(car_down, space)
        safe_space_right = get_safe_space_right(car_right, space)
        safe_space = zono_op.intersection_hz_hz(safe_space_down, safe_space_right)




options = 'outer'
reduction_options = 'reduced'

road = 'down_right_v4_b'
print(f'****************************************************************************************************')
print(f'                                          Settings                                                  ')
print(f'****************************************************************************************************')
print(f'Roads included : {road}')

# Initialize objects
dynamics = DynamicsModel()
zono_op = ZonoOperations()
vis = ZonoVisualizer(zono_op = zono_op)
env = DynamicEnv(zono_op = zono_op, dynamics = dynamics, visualizer = vis, options = options)

# Create the space
# space = env.state_space             # Safe state space
target = env.target_space           # Free parking spots
input = env.input_space
obs = env.initial_space   # Initial space of non-ego vehicles

road_down = StateSpaceSafe().road_down
road_right = StateSpaceSafe().road_right
space = zono_op.union_hz_hz_v2(road_down, road_right)

print(f'****************************************************************************************************')
print(f'                                          FRS SETS                                                  ')
print(f'****************************************************************************************************')

# Initialize obstacle based on region it is in
car_down = zono_op.intersection_hz_hz(obs, road_down)
car_right = zono_op.intersection_hz_hz(obs, road_right)


N = 101
for i in range(N):
    print(f'*********************************************************')
    print(f'Iteration {i}')

    #### OBSTACLES

    car_down, car_right = get_obstacle(road_down, road_right, input, car_down, car_right, dynamics.A, dynamics.B, dynamics.W)

    # print(f'car right:')
    # print(f'Gc = \n{car_right.Gc}')
    # print(f'Gb = \n{car_right.Gb}')
    # print(f'C = {car_right.C.T}')
    # print(f'Ac = \n{car_right.Ac}')
    # print(f'Ab = \n{car_right.Ab}')
    # print(f'b = {car_right.b.T}')

    # Plot obstacles
    obs = zono_op.union_hz_hz_v2(car_down, car_right)
    print(f'Obs. : ng = {obs.ng}\t nc = {obs.nc}\t nb = {obs.nb}')
    env.vis_background()
    static_obs_vis = HybridZonotope(obs.Gc[0:2, :], obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
    vis.vis_hz([static_obs_vis], colors = obs_color, show_edges=True, zorder=100)

    # static_obs_vis = HybridZonotope(car_down.Gc[0:2, :], car_down.Gb[0:2, :], car_down.C[0:2, :], car_down.Ac, car_down.Ab, car_down.b)
    # vis.vis_hz([static_obs_vis], colors = obs_color, show_edges=True, zorder=100)
    # static_obs_vis = HybridZonotope(car_right.Gc[0:2, :], car_right.Gb[0:2, :], car_right.C[0:2, :], car_right.Ac, car_right.Ab, car_right.b)
    # vis.vis_hz([static_obs_vis], colors = obs_color, show_edges=True, zorder=100)    




    # env.grid = env.vis.vis_hz_brs(hz = static_obs_vis, brs_settings=env.brs_settings)
    plt.show()
    env.vis.ax.set_title(f'road = {road}, iter = {i}, states: position, ng = {obs.ng}, nc = {obs.nc}, nb = {obs.nb}', fontsize=16)
    name = f'frs_N_{i}'
    # env.vis.fig.savefig(f'./results/dynamic/{road}/{name}_obs.pdf', dpi=300)
    plt.close(env.vis.fig)
    env.vis.new_fig()


    # ##### SAFE SPACE

    # safe_space = get_safe_space(car_right, car_down, space)

    # # Plot safe space
    # print(f'safe_space. : ng = {safe_space.ng}\t nc = {safe_space.nc}\t nb = {safe_space.nb}')
    # env.vis_background()
    # safe_space_vis = HybridZonotope(safe_space.Gc[0:2, :], safe_space.Gb[0:2, :], safe_space.C[0:2, :], safe_space.Ac, safe_space.Ab, safe_space.b)
    # # vis.vis_hz([safe_space_vis], colors = obs_color, show_edges=True, zorder=100)
    # env.grid = env.vis.vis_hz_brs(hz = safe_space_vis, brs_settings=env.brs_settings)
    # plt.show()
    # env.vis.ax.set_title(f'road = {road}, iter = {i}, states: position, ng = {obs.ng}, nc = {obs.nc}, nb = {obs.nb}', fontsize=16)
    # name = f'frs_N_{i}'
    # env.vis.fig.savefig(f'./results/dynamic/{road}/{name}_safe.pdf', dpi=300)
    # plt.close(env.vis.fig)
    # env.vis.new_fig()






