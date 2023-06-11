import matplotlib.pyplot as plt

from utils.environments.samples import SamplesHZ
from utils.visualization import ZonoVisualizer
from utils.operations.operations import ZonoOperations

'''
Run this script to test the union and intersection operations on Hybrid zonotopes.
In addition, this script provides information on how the order of the zonotope evolves.
'''

colors = [
    (0.949, 0.262, 0.227, 0.6),     # Obstacle (Red)
    (0.717, 0.694, 0.682, 0.5),     # Road (Gray)
    (0.231, 0.780, 0.160, 1.0),     # Parking spot (Green)
    (0.423, 0.556, 0.749, 0.5)      # BRS (Blue)
]

##############################################################################
#                              Original Sets                                 #
##############################################################################
vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

hz_15 = SamplesHZ().set_1
hz_16 = SamplesHZ().set_2
hz_17 = SamplesHZ().set_3
vis.vis_hz([hz_15])
plt.show()
print(f'*******************************************************************************')
print(f'Dimensions of original Hybrid Zonotopes:')
print(f'HZ_15: ng = {hz_15.ng}, nc = {hz_15.nc}, nb = {hz_15.nb}')
print(f'HZ_16: ng = {hz_16.ng}, nc = {hz_16.nc}, nb = {hz_16.nb}')
print(f'HZ_17: ng = {hz_17.ng}, nc = {hz_17.nc}, nb = {hz_17.nb}')



# ##############################################################################
# #                              Intersection                                  #
# ##############################################################################
print(f'--------------------------------------------------')
print(f'Dimensions of intersections')


vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

hz_15_IS_16 = ZonoOperations().intersection_hz_hz(hz_15, hz_16)
print(f'HZ_15_IS_16: ng = {hz_15_IS_16.ng}, nc = {hz_15_IS_16.nc}, nb = {hz_15_IS_16.nb}')
vis.vis_hz([hz_15_IS_16])
plt.show()


vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

hz_15_IS_17 = ZonoOperations().intersection_hz_hz(hz_15, hz_17)
print(f'HZ_15_IS_17: ng = {hz_15_IS_17.ng}, nc = {hz_15_IS_17.nc}, nb = {hz_15_IS_17.nb}')
vis.vis_hz([hz_15_IS_17])
plt.show()


vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

hz_16_IS_17 = ZonoOperations().intersection_hz_hz(hz_16, hz_17)
print(f'HZ_16_IS_17: ng = {hz_16_IS_17.ng}, nc = {hz_16_IS_17.nc}, nb = {hz_16_IS_17.nb}')
vis.vis_hz([hz_16_IS_17])
plt.show()


vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

hz_15_IS_16_IS_17 = ZonoOperations().intersection_hz_hz(hz_15_IS_16, hz_17)
print(f'HZ_15_IS_16_IS_17: ng = {hz_15_IS_16_IS_17.ng}, nc = {hz_15_IS_16_IS_17.nc}, nb = {hz_15_IS_16_IS_17.nb}')
vis.vis_hz([hz_15_IS_16_IS_17])
plt.show()

##############################################################################
#                                  Union                                     #
##############################################################################
print(f'--------------------------------------------------')
print(f'Dimensions of Unions')


vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

hz_15_U_16 = ZonoOperations().union_hz_hz(hz_15, hz_16)
print(f'HZ_15_U_16: ng = {hz_15_U_16.ng}, nc = {hz_15_U_16.nc}, nb = {hz_15_U_16.nb}')
vis.vis_hz([hz_15_U_16])
plt.show()


vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

hz_15_U_17 = ZonoOperations().union_hz_hz(hz_15, hz_17)
print(f'HZ_15_U_17: ng = {hz_15_U_17.ng}, nc = {hz_15_U_17.nc}, nb = {hz_15_U_17.nb}')
vis.vis_hz([hz_15_U_17])
plt.show()


vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

hz_16_U_17 = ZonoOperations().union_hz_hz(hz_16, hz_17)
print(f'HZ_16_U_17: ng = {hz_16_U_17.ng}, nc = {hz_16_U_17.nc}, nb = {hz_16_U_17.nb}')
vis.vis_hz([hz_16_U_17])
plt.show()


vis = ZonoVisualizer()
vis.ax.set_xlim(-10, 10); vis.ax.set_ylim(-10, 10)
vis.ax.spines['right'].set_visible(True); vis.ax.spines['left'].set_visible(True)
vis.ax.spines['top'].set_visible(True); vis.ax.spines['bottom'].set_visible(True)
vis.ax.get_xaxis().set_visible(True); vis.ax.get_yaxis().set_visible(True)
vis.ax.grid(True)

hz_15_U_16_U_17 = ZonoOperations().union_hz_hz(hz_15_U_16, hz_17)
print(f'HZ_15_U_16_U_17: ng = {hz_15_U_16_U_17.ng}, nc = {hz_15_U_16_U_17.nc}, nb = {hz_15_U_16_U_17.nb}')
vis.vis_hz([hz_15_U_16_U_17])
plt.show()


















