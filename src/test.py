import numpy as np
import math
np.set_printoptions(precision=2)

x_start = 280
x_end = 300

y_start = 0
y_end = 20

phi_start = 90
phi_end = 180

# r = math.sqrt((x_end - x_start)**2 + (y_end - y_start)**2)
# steps = math.ceil(r/2.5)
steps = 12

x = np.linspace(x_start, x_end, steps)
y = np.linspace(y_start, y_end, steps)
phi = np.linspace(phi_start, phi_end, steps)

print(f'x = {x.T}')
print(f'y = {y.T}')
print(f'phi = {phi.T}')