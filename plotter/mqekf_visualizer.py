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

import os

file1 = r'../gnc/mqekf_quaternion_output.csv'
file2 = r'../gnc/TeleMega_quaternion_append.csv'

print(f"File 1 exists: {os.path.exists(file1)}")
print(f"File 2 exists: {os.path.exists(file2)}")

if not os.path.exists(file1):
    print(f"Looking for: {os.path.abspath(file1)}")

df = pd.read_csv('../gnc/mqekf_quaternion_output.csv')  
df_non_filter =  pd.read_csv('../gnc/TeleMega_quaternion_append.csv')  
# Convert to NumPy array
quats = df[['quaternion_w', 'quaternion_x', 'quaternion_y', 'quaternion_z']].to_numpy()  # pull quaternion rows
quats_unfilter = df_non_filter[['quaternion_w', 'quaternion_x', 'quaternion_y', 'quaternion_z']].to_numpy()  # pull quaternion rows


fig = plt.figure(figsize=(14, 6))
ax1 = fig.add_subplot(121, projection="3d")

ax1.set_xlim([-1, 1])
ax1.set_ylim([-1, 1])
ax1.set_zlim([-1, 1])
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_zlabel("Z")
ax1.set_title("Filtered (MQEKF)")


ax2 = fig.add_subplot(122, projection="3d")
ax2.set_xlim([-1, 1])
ax2.set_ylim([-1, 1])
ax2.set_zlim([-1, 1])
ax2.set_xlabel("X")
ax2.set_ylabel("Y")
ax2.set_zlabel("Z")
ax2.set_title("Unfiltered (TeleMega)")

# Body axes for filtered
lines_filtered = [
    ax1.plot([], [], [], 'r', lw=3, label='X')[0],
    ax1.plot([], [], [], 'g', lw=3, label='Y')[0],
    ax1.plot([], [], [], 'b', lw=3, label='Z')[0],
]

# Body axes for unfiltered
lines_unfiltered = [
    ax2.plot([], [], [], 'r', lw=3, label='X')[0],
    ax2.plot([], [], [], 'g', lw=3, label='Y')[0],
    ax2.plot([], [], [], 'b', lw=3, label='Z')[0],
]

ax1.legend()
ax2.legend()

time_array = df_non_filter['time'].to_numpy()

def update(i):
    
    q_filt = quats[i]
    q_filt = q_filt / np.linalg.norm(q_filt)
    R_filt = quat_to_rotmat(q_filt)
    axes_filt = R_filt @ np.eye(3)
    #ax2.set_title(f"Unfiltered (TeleMega) - Time: {time_array[i]:.2f}")
    #ax1.set_title(f"Filtered (MQEKF) - Time: {time_array[i]:.2f}")
    print(f"Time: {time_array[i]:.2f} s")
    for line, vec in zip(lines_filtered, axes_filt.T):
        line.set_data([0, vec[0]], [0, vec[1]])
        line.set_3d_properties([0, vec[2]])
    
    q_unfilt = quats_unfilter[i]
    q_unfilt = q_unfilt / np.linalg.norm(q_unfilt)
    R_unfilt = quat_to_rotmat(q_unfilt)
    axes_unfilt = R_unfilt @ np.eye(3)
    
    for line, vec in zip(lines_unfiltered, axes_unfilt.T):
        line.set_data([0, vec[0]], [0, vec[1]])
        line.set_3d_properties([0, vec[2]])
    
    return lines_filtered + lines_unfiltered

ani = FuncAnimation(fig, update, frames=min(len(quats), len(quats_unfilter)), interval = 0.0001, blit=True)
plt.tight_layout()
plt.show()
