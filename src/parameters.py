import numpy as np

# Defines Paramaters for the stellite

# Constants
Mc = 2         # kg
Mc += 0.2 * Mc * (2*np.random.rand() - 1)  # 0.2 uncertainty

Jc = 0.009     # kg m^2
Jc += 0.2 * Jc * (2*np.random.rand() - 1)
Mr = Ml = 0.3  # kg

d = 0.28       # m
d += 0.2 * d * (2*np.random.rand() - 1)

mu = 0.21      # kg s^-1
mu += 0.2 * mu * (2*np.random.rand() - 1)

g = 9.81       # m s^-2

# For drawing
rad_rot = d / 3
side_bod = d / 2
side_tar = d / 2

# For Controller
kd_h = 0.5 * (Mc + Ml + Mr)
kp_h = 0.06 * (Mc + Mr + Mr)
kd_th = 3.8885*Jc
kp_th = 7.5625*Jc
kd_z = (1/g)*(mu/(Mc + Ml + Mr) - 0.38885)
kp_z = -0.075625 / g
