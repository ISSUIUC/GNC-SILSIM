import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd



# https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
def quat_to_rotmat(q):
    w, x, y, z = q
    return np.array([
        [1-2*(y**2+z**2), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x**2+z**2), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x**2+y**2)]
    ])



df = pd.read_csv('GNC-SILSIM\plotter\mqekf_quaternion_output.csv')  

# Convert to NumPy array
quats = df[['w', 'x', 'y', 'z']].to_numpy()  # pull quaternion rows


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# Body axes
lines = [
    ax.plot([], [], [], 'r', lw=3)[0],
    ax.plot([], [], [], 'g', lw=3)[0],
    ax.plot([], [], [], 'b', lw=3)[0],
]


def update(i):
    q = quats[i]
    q = q / np.linalg.norm(q)

    R = quat_to_rotmat(q)

    axes = R @ (np.eye(3))

    for line, vec in zip(lines, axes.T):
        line.set_data([0, vec[0]], [0, vec[1]])
        line.set_3d_properties([0, vec[2]])

    return lines

ani = FuncAnimation(fig, update, frames=len(quats), interval=20)
plt.show()
