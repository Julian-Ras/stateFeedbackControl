"""
This code shows a trajectory tracking of a chai  
of integrators system 

Note : To run the script type in terminal:
        pip install numpy
        pip install scipy
"""


import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# --- System matrices ---
A = np.array([[0, 1, 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0]])

B = np.array([[0],
              [0],
              [0],
              [1]])

B1 = np.array([0, 0, 0, 9.8])

# --- Desired poles ---
p = np.array([-1, -2, -3, -4])

# --- Pole placement (equivalent to MATLAB place) ---
K = signal.place_poles(A, B, p).gain_matrix  # shape (1,4)

# --- Simulation parameters ---
dt = 0.001
t_final = 50
s = np.arange(0, t_final + dt, dt)

# --- State variables ---
x = np.zeros((4, len(s)))
x[:, 0] = np.array([0, 0, 0, 0])

e = np.zeros((4, len(s)))
r = np.zeros((4, len(s)))
u = np.zeros(len(s))

# --- Simulation loop ---
for i in range(len(s) - 1):
    t = s[i]

    # Reference
    r[:, i] = np.array([
        4 + np.sin(0.5 * t),
        0.5 * np.cos(0.5 * t),
        -0.25 * np.sin(0.5 * t),
        -0.125 * np.cos(0.5 * t)
    ])

    # 4th derivative reference
    xpppp = 0.0625 * np.sin(0.5 * t)

    # Error
    e[:, i] = x[:, i] - r[:, i]

    # Control input (state feedback)
    u[i] = xpppp + (-K @ e[:, i]).item() / 9.8
    u[i] = u[i].item()  # convert from array to scalar

    # System dynamics (Euler integration)
    x_dot = A @ x[:, i] + B1 * u[i] - np.array([0, 0, 0, xpppp])
    x[:, i + 1] = x[:, i] + dt * x_dot

# --- Plot 1: Position ---
plt.figure()
plt.plot(s, x[0, :], label='x1')
plt.plot(s, r[0, :], label='x1_ref')
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend()
plt.grid()

# --- Plot 2: Errors ---
plt.figure()
for i in range(4):
    plt.plot(s, e[i, :], label=f'e{i+1}')
plt.xlabel("Time")
plt.ylabel("Errors")
plt.legend()
plt.grid()

# --- Plot 3: Control input ---
plt.figure()
plt.plot(s, u, label='u')
plt.xlabel("Time")
plt.ylabel("Control input")
plt.grid()

plt.show()