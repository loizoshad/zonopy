import matplotlib.pyplot as plt
import numpy as np

from utils.environments.dynamic_env import DynamicEnv, StateSpaceSafe, DynamicObstacleSpace
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


def get_obstacle(obs: HybridZonotope):
    '''
    This method takes in the forward reachable set of an obstacle and it then computes its over-approximation as a hypercube.
    In addition this over-approximation is used to make sure that the ego vehicle does not collide with the obstacle,
    cause it assumes that at that time step the object is static at that position.
    '''
    # Extract position states
    obs_pos = HybridZonotope(obs.Gc[0:2, :], obs.Gb[0:2, :], obs.C[0:2, :], obs.Ac, obs.Ab, obs.b)
    cz = zono_op.oa_hz_to_cz(obs)
    # cz = zono_op.redundant_c_g_cz(cz)
    cz = zono_op.oa_cz_to_hypercube_tight_4d(cz)


    # Set the velocity space to the full velocity space
    G = np.array([
        [cz.G[0, 0],    0.0    , 0.0, 0.0],
        [   0.0    , cz.G[1, 1], 0.0, 0.0],
        [   0.0    ,    0.0    , 1.0, 0.0],
        [   0.0    ,    0.0    , 0.0, 1.0]
    ])
    C = np.array([
        [cz.C[0, 0]],
        [cz.C[1, 0]],
        [0.0],
        [0.0]
    ])
    hz = HybridZonotope(G, np.zeros((cz.dim, 0)), C, np.zeros((0, cz.dim)), np.zeros((0, 0)), np.zeros((0, 1)) )
    hz = zono_op.intersection_hz_hz(hz, space)

    return hz





options = 'outer'
reduction_options = 'reduced'

road = 'down_right_v4'
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


    car_down_1 = zono_op.one_step_frs_hz(X = road_down, U = input, I = car_down, A = env.A, B = env.B, W = env.W)     # Car down  # V3
    #
    car_right_1 = zono_op.one_step_frs_hz(X = road_right, U = input, I = car_right, A = env.A, B = env.B, W = env.W)  # Car right # V3
    car_right_2 = zono_op.one_step_frs_hz(X = road_right, U = input, I = car_down, A = env.A, B = env.B, W = env.W)  # Car right # V3


    car_down = car_down_1
    car_right = zono_op.union_hz_hz_v2(car_right_1, car_right_2)


    car_down  = zono_op.cz_to_hz( zono_op.oa_cz_to_hypercube_tight_4d( zono_op.oa_hz_to_cz(car_down)))      # Simplify car down
    car_right = zono_op.cz_to_hz( zono_op.oa_cz_to_hypercube_tight_4d( zono_op.oa_hz_to_cz(car_right)))     # Simplify car right


    obs = zono_op.union_hz_hz_v2(car_down, car_right)
    car_vis = obs





    print(f'Obs. : ng = {obs.ng}\t nc = {obs.nc}\t nb = {obs.nb}')
    env.vis_background()
    static_obs_vis = HybridZonotope(car_vis.Gc[0:2, :], car_vis.Gb[0:2, :], car_vis.C[0:2, :], car_vis.Ac, car_vis.Ab, car_vis.b)
    vis.vis_hz([static_obs_vis], colors = obs_color, show_edges=True, zorder=100)
    # env.grid = env.vis.vis_hz_brs(hz = static_obs_vis, brs_settings=env.brs_settings)
    plt.show()
    env.vis.ax.set_title(f'road = {road}, iter = {i}, states: position, ng = {obs.ng}, nc = {obs.nc}, nb = {obs.nb}', fontsize=16)
    name = f'frs_N_{i}'
    env.vis.fig.savefig(f'./results/dynamic/{road}/{name}_pos.pdf', dpi=300)
    plt.close(env.vis.fig)
    env.vis.new_fig()
    #
    env.vis_background()
    static_obs_vis = HybridZonotope(car_vis.Gc[2:, :], car_vis.Gb[2:, :], car_vis.C[2:, :], car_vis.Ac, car_vis.Ab, car_vis.b)
    vis.vis_hz([static_obs_vis], colors = obs_color, show_edges=True, zorder=100)
    # env.grid = env.vis.vis_hz_brs(hz = static_obs_vis, brs_settings=env.brs_settings)
    plt.show()
    env.vis.ax.set_title(f'road = {road}, iter = {i}, states: velocity, ng = {obs.ng}, nc = {obs.nc}, nb = {obs.nb}', fontsize=16)
    name = f'frs_N_{i}'
    env.vis.fig.savefig(f'./results/dynamic/{road}/{name}_vel.pdf', dpi=300)    
    plt.close(env.vis.fig)
    env.vis.new_fig()










