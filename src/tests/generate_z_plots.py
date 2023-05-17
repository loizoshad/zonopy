import matplotlib.pyplot as plt

from utils.environments.environments import SamplesHZ
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations

'''
Run this script to test the union and intersection operations on Hybrid zonotopes.
In addition, this script provides information on how the order of the zonotope evolves.
'''

colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.423, 0.556, 0.749, 0.9)      # BRS (Blue)
]

# ##############################################################################
# #                              Zonotope                                 #
# ##############################################################################
z = SamplesHZ().set_5
vis = ZonoVisualizer()
vis.vis_hz([z], title = '', colors = colors, legend_labels=['$HZ_{15}$'], add_legend=False)
plt.show()



##############################################################################
#                           Constrained Zonotope                             #
##############################################################################
cz = SamplesHZ().set_6
vis = ZonoVisualizer()
vis.vis_hz([z, cz], title = '', colors = colors, legend_labels=['$HZ_{15}$'], add_legend=False)
plt.show()

##############################################################################
#                              Hybrid Zonotope                                 #
##############################################################################

colors = [
    (0.423, 0.556, 0.749, 1.0)      # BRS (Blue)
]
hz = SamplesHZ().set_7
vis = ZonoVisualizer()
vis.vis_hz([hz], title = '', colors = colors, legend_labels=['$HZ_{15}$'], add_legend=False)
plt.show()









# ##############################################################################
# #                                  Union                                     #
# ##############################################################################
# print(f'--------------------------------------------------')
# print(f'Dimensions of Unions')
# hz_15_U_16 = ZonoOperations().union_hz_hz(hz_15, hz_16)
# vis = ZonoVisualizer()
# print(f'HZ_15_U_16: ng = {hz_15_U_16.ng}, nc = {hz_15_U_16.nc}, nb = {hz_15_U_16.nb}')
# vis.vis_hz([hz_15_U_16], title = 'Hybrid Zonotope', legend_labels=['$HZ_{15} \cup HZ_{16}$'], add_legend=True)
# plt.show()
# hz_15_U_17 = ZonoOperations().union_hz_hz(hz_15, hz_17)
# vis = ZonoVisualizer()
# print(f'HZ_15_U_17: ng = {hz_15_U_17.ng}, nc = {hz_15_U_17.nc}, nb = {hz_15_U_17.nb}')
# vis.vis_hz([hz_15_U_17], title = 'Hybrid Zonotope', legend_labels=['$HZ_{15} \cup HZ_{17}$'], add_legend=True)
# plt.show()
# hz_16_U_17 = ZonoOperations().union_hz_hz(hz_16, hz_17)
# vis = ZonoVisualizer()
# print(f'HZ_16_U_17: ng = {hz_16_U_17.ng}, nc = {hz_16_U_17.nc}, nb = {hz_16_U_17.nb}')
# vis.vis_hz([hz_16_U_17], title = 'Hybrid Zonotope', legend_labels=['$HZ_{16} \cup HZ_{17}$'], add_legend=True)
# plt.show()


















