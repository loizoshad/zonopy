import matplotlib.pyplot as plt
import numpy as np
from utils.environments.environments import SamplesCZ
from utils.operations.operations import ZonoOperations
from utils.sets.constrained_zonotopes import ConstrainedZonotope
from utils.sets.zonotopes import Zonotope
from utils.visualization import ZonoVisualizer

"""
Run this script to test all the supported Constrained Zonotope operations.

TODO: Add a test for set membership
"""


##############################################################################
#                              Original Sets                                 #
##############################################################################
cz1 = SamplesCZ().set_1
cz2 = SamplesCZ().set_2
vis = ZonoVisualizer()
vis.vis_cz([cz1, cz2], title="Original Constrained Zonotopes", legend_labels=["$CZ_{1}$", "$CZ_{2}$"], add_legend=True)
plt.show()

##############################################################################
#                              Minkowski Sum                                 #
##############################################################################
cz3 = ZonoOperations().ms_cz_cz(cz1, cz2)
vis = ZonoVisualizer()
vis.vis_cz(
    [cz1, cz2, cz3],
    title="Minkowski Sum",
    legend_labels=["$CZ_{1}$", "$CZ_{2}$", "$CZ_{1} \\oplus CZ_{2}$"],
    add_legend=True,
)
plt.show()

##############################################################################
#                            Linear Transformation                           #
##############################################################################
M = np.array([[1.0, 1.0], [0.0, 1.0]])
cz4 = ZonoOperations().lt_cz(M, cz1)
vis = ZonoVisualizer()
vis.vis_cz([cz1, cz4], title="Linear Transformation", legend_labels=["$CZ_{1}$", "$M CZ_{1}$"], add_legend=True)
plt.show()

##############################################################################
#                              Intersection                                  #
##############################################################################
cz5 = ZonoOperations().intersection_cz_cz(cz1, cz2)
vis = ZonoVisualizer()
vis.vis_cz(
    [cz1, cz2, cz5],
    title="Intersection",
    legend_labels=["$CZ_{1}$", "$CZ_{2}$", "$CZ_{1} \\cap CZ_{2}$"],
    add_legend=True,
)
plt.show()
