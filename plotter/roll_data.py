import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

# ---------- LOAD CSV ----------
csv_path = "./data/TeleMega.csv"
df = pd.read_csv(csv_path)

time = df["time"].to_numpy()
gyro_roll = df["gyro_roll"].to_numpy()
gyro_pitch = df["gyro_pitch"].to_numpy()
gyro_yaw = df["gyro_yaw"].to_numpy()

# ---------- INTEGRATE GYRO (deg/s → deg) ----------
dt = np.diff(time, prepend=time[0])

roll = np.cumsum(gyro_roll * dt)
pitch = np.cumsum(gyro_pitch * dt)
yaw = np.cumsum(gyro_yaw * dt)

# ---------- PLOT SETUP ----------
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.set_xlim(-1.2, 1.2)
ax.set_ylim(-1.2, 1.2)
ax.set_zlim(-1.2, 1.2)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

ax.set_box_aspect([1, 1, 1])

# ---------- ROCKET BODY AXES ----------
L = 1.0
body_axes = np.array([
    [0, 0, L],  # X (red)   -> UP / nose
    [L, 0, 0],  # Y (green) -> right
    [0, L, 0],  # Z (blue)  -> left/down
])
# ---------- INITIAL QUIVER ----------
quiver = ax.quiver(
    [0, 0, 0], [0, 0, 0], [0, 0, 0],
    body_axes[:,0], body_axes[:,1], body_axes[:,2],
    color=["r", "g", "b"],
    linewidth=3
)

# ---------- UPDATE FUNCTION ----------
def update(i):
    r = R.from_euler("xyz", [roll[i], pitch[i], yaw[i]], degrees=True)
    rotated = r.apply(body_axes)

    quiver.set_segments([
        [[0, 0, 0], rotated[0]],
        [[0, 0, 0], rotated[1]],
        [[0, 0, 0], rotated[2]],
    ])

    ax.set_title(
        f"t = {time[i]:.2f}s\n"
        f"Roll={roll[i]:.1f}°, Pitch={pitch[i]:.1f}°, Yaw={yaw[i]:.1f}°"
    )

# ---------- ANIMATION ----------
ani = FuncAnimation(
    fig,
    update,
    frames=len(time),
    interval=20,
    blit=False
)

plt.show()
