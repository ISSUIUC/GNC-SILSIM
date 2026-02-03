import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.spatial.transform import Rotation as R
from matplotlib.animation import FuncAnimation

# Load quaternion data
df = pd.read_csv("./gnc/mqekf_quaternion_output.csv")

# Extract quaternions (x, y, z, w)
quaternions = np.array([
    df["quaternion_x"].values,
    df["quaternion_y"].values,
    df["quaternion_z"].values,
    df["quaternion_w"].values
]).T

# Normalize quaternions
quaternions /= np.linalg.norm(quaternions, axis=1)[:, None]

# Create rotation objects
rotations = R.from_quat(quaternions).inv()  # invert if needed to match body/world frame

# Time vector
time = np.arange(len(quaternions))

# Convert to Euler angles for plotting
euler = rotations.as_euler("xyz", degrees=True)

# Plot Euler angles
plt.figure()
plt.plot(time, euler[:, 0], label="Roll (X)")
plt.plot(time, euler[:, 1], label="Pitch (Y)")
plt.plot(time, euler[:, 2], label="Yaw (Z)")
plt.xlabel("Time step")
plt.ylabel("Degrees")
plt.legend()
plt.title("Euler Angles from Quaternions")
plt.show()


# 3D Rocket orientation plot
fig = plt.figure()
ax = fig.add_subplot(projection="3d")

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

ax.set_xlabel("Y (Right)")
ax.set_ylabel("Z (Forward)")
ax.set_zlabel("X (Up)")
ax.set_title("Rocket Orientation (X is Up)")

# World axes (dashed)
ax.quiver(0, 0, 0, 0, 0, 1, linestyle="dashed", alpha=0.4)  # X up
ax.quiver(0, 0, 0, 1, 0, 0, linestyle="dashed", alpha=0.4)  # Y right
ax.quiver(0, 0, 0, 0, 1, 0, linestyle="dashed", alpha=0.4)  # Z forward

# Initialize rocket body axes
rocket_x = ax.quiver(0, 0, 0, 0, 0, 1, color="r", linewidth=3)
rocket_y = ax.quiver(0, 0, 0, 1, 0, 0, color="g", linewidth=2)
rocket_z = ax.quiver(0, 0, 0, 0, 1, 0, color="b", linewidth=2)


def update(frame):
    global rocket_x, rocket_y, rocket_z

    rocket_x.remove()
    rocket_y.remove()
    rocket_z.remove()

    Rm = rotations[frame].as_matrix()

    bx = Rm[:, 0]  # Rocket X (UP)
    by = Rm[:, 1]  # Rocket Y
    bz = Rm[:, 2]  # Rocket Z

    # Map into matplotlib axes: (Y, Z, X) -> (x, y, z)
    rocket_x = ax.quiver(0, 0, 0, bx[1], bx[2], bx[0], color="r", linewidth=3)
    rocket_y = ax.quiver(0, 0, 0, by[1], by[2], by[0], color="g", linewidth=2)
    rocket_z = ax.quiver(0, 0, 0, bz[1], bz[2], bz[0], color="b", linewidth=2)

    return rocket_x, rocket_y, rocket_z


ani = FuncAnimation(fig, update, frames=len(rotations), interval=50, blit=False)

plt.show()
