import numpy as np
import control as cnt

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

# For PID Controller
kd_h = 0.38885 * (Mc + Ml + Mr)
kp_h = 0.075625 * (Mc + Mr + Mr)
kd_th = 3.8885*Jc
kp_th = 7.5625*Jc
kd_z = (1/g)*(mu/(Mc + Ml + Mr) - 0.38885)
kp_z = -0.075625 / g
ki_z = 0.000007


## FEEDBACK CONTROLLER GAINS

# tuning parameters
wn_h = 0.275
wn_z = 0.275
wn_tht = 2.75
zeta_h = zeta_z = zeta_tht = 0.707

# state space matrices
A_lat = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, -g, -mu / (Mc + Ml + Mr) , 0],
    [0, 0, 0, 0]
])
B_lat = np.array([
    [0],
    [0],
    [0],
    [1 / Jc]
])
C_lat = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]
])
D_lat = np.array([
    [0]
])

A_lon = np.array([
    [0, 1],
    [0, 0]
])
B_lon = np.array([
    [0],
    [1 / (Mc + Ml + Mr)]
])
C_lon = np.array([
    [1, 0]
])
D_lon = np.array([
    [0]
])

# gain calculations for longitudinal dynamics
desired_char_poly_lon = [1, 2*zeta_h*wn_h, wn_h**2]
desired_poles_lon = np.roots(desired_char_poly_lon)

if np.linalg.matrix_rank(cnt.ctrb(A_lon, B_lon)) != 2:
    print("Longitudinal dynamics not controllable!")
else:
    K_lon = cnt.acker(A_lon, B_lon, desired_poles_lon)
    kr_lon = -1.0 / (C_lon @ np.linalg.inv(A_lon - B_lon @ K_lon) @ B_lon)
    print('K_lon: ', K_lon)
    print('kr_lon: ', kr_lon)

# gain calculations for lateral dynamics
desired_char_poly_lat = np.convolve([1, 2*zeta_z*wn_z, wn_z**2], [1, 2*zeta_tht*wn_tht, wn_tht**2])
desired_poles_lat = np.roots(desired_char_poly_lat)

if np.linalg.matrix_rank(cnt.ctrb(A_lat, B_lat)) != 4:
    print("Lateral dynamics not controllable!")
else:
    K_lat = cnt.acker(A_lat, B_lat, desired_poles_lat)
    Cr_lat = np.array([1, 0, 0, 0])
    kr_lat = -1.0 / (Cr_lat @ np.linalg.inv(A_lat - B_lat @ K_lat) @ B_lat)
    print('K_lat: ', K_lat)
    print('kr_lat: ', kr_lat)
