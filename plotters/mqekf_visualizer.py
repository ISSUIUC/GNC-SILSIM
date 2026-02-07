import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# Quaternion to rotation matrix
def quat_to_rotmat(q):
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
        ]
    )


# Load data
file1 = r"../output/mqekf_quaternion_output.csv"
df = pd.read_csv(file1)
quats = df[["quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z"]].to_numpy()
time_array = df["timestamp"].to_numpy()

# Set up figure
fig = plt.figure(figsize=(10, 8))
ax1 = fig.add_subplot(111, projection="3d")
ax1.set_xlim([-1, 1])
ax1.set_ylim([-1, 1])
ax1.set_zlim([-1, 1])
ax1.set_xlabel("X")
ax1.set_ylabel("Y")
ax1.set_zlabel("Z")
ax1.set_title("Rocket Orientation (MQEKF)")

# Body axes
lines_filtered = [
    ax1.plot([], [], [], "r", lw=3, label="X")[0],
    ax1.plot([], [], [], "g", lw=3, label="Y")[0],
    ax1.plot([], [], [], "b", lw=3, label="Z")[0],
]
ax1.legend(loc='upper right')


# Define a realistic rocket
def create_rocket():
    nose_length = 0.3
    body_length = 1.0
    radius = 0.05
    fin_height = 0.1
    n_theta = 20  # smoother cylinder/nose

    # Circle for cross-section
    theta = np.linspace(0, 2 * np.pi, n_theta)

    # Nose cone (tip at x=0)
    nose_tip = np.array([0, 0, 0])
    nose_base = np.array(
        [[nose_length, radius * np.cos(t), radius * np.sin(t)] for t in theta]
    )
    nose_faces = [
        [nose_tip, nose_base[i], nose_base[(i + 1) % n_theta]] for i in range(n_theta)
    ]

    # Cylinder body
    x_vals = [nose_length, nose_length + body_length]
    body_vertices = np.array(
        [[x, radius * np.cos(t), radius * np.sin(t)] for x in x_vals for t in theta]
    )
    body_faces = []
    for i in range(n_theta):
        j = (i + 1) % n_theta
        body_faces.append(
            [
                body_vertices[i],
                body_vertices[j],
                body_vertices[j + n_theta],
                body_vertices[i + n_theta],
            ]
        )

    # Fins (triangular, near base)
    fins = []
    fin_base_x = nose_length + body_length * 0.9
    for sign_y in [-1, 1]:
        for sign_z in [-1, 1]:
            fin = np.array(
                [
                    [fin_base_x, 0, 0],
                    [fin_base_x, sign_y * fin_height, 0],
                    [fin_base_x, 0, sign_z * fin_height],
                ]
            )
            fins.append(fin)

    return nose_faces, body_faces, fins


# Helper: compute rocket COM
def rocket_com(nose_faces, body_faces):
    verts = []
    for face in nose_faces + body_faces:
        verts.extend(face)
    verts = np.array(verts)
    return np.mean(verts, axis=0)


# Helper: rotate faces or vertices around COM
def rotate_around_com(vertices_list, com, R):
    rotated_list = []
    for verts in vertices_list:
        verts = np.array(verts)
        rotated = (R @ (verts - com).T).T + com
        rotated_list.append(rotated)
    return rotated_list


# Create rocket
nose_faces, body_faces, fins_vertices = create_rocket()

# Flip rocket 180° around COM (Z-axis)
com = rocket_com(nose_faces, body_faces)
R_flip = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
nose_faces = rotate_around_com(nose_faces, com, R_flip)
body_faces = rotate_around_com(body_faces, com, R_flip)
fins_vertices = [(R_flip @ (fin - com).T).T + com for fin in fins_vertices]

# Poly3DCollections
nose_polys = [Poly3DCollection([face], color="orange") for face in nose_faces]
body_polys = [Poly3DCollection([face], color="gray") for face in body_faces]
fins_polys = [Poly3DCollection([fin], color="red") for fin in fins_vertices]

for poly in nose_polys + body_polys + fins_polys:
    ax1.add_collection3d(poly)


# Animation update
def update(i):
    q_filt = quats[i] / np.linalg.norm(quats[i])
    R = quat_to_rotmat(q_filt)

    # Update body axes (X-axis along nose)
    axes = R @ np.eye(3)
    for line, vec in zip(lines_filtered, axes.T):
        line.set_data([0, vec[0]], [0, vec[1]])
        line.set_3d_properties([0, vec[2]])

    # Rotate nose faces
    for poly, face in zip(nose_polys, nose_faces):
        rotated = [(R @ vert.T).T for vert in face]
        poly.set_verts(rotated)

    # Rotate body faces
    for poly, face in zip(body_polys, body_faces):
        rotated = [(R @ vert.T).T for vert in face]
        poly.set_verts(rotated)

    # Rotate fins
    for poly, fin in zip(fins_polys, fins_vertices):
        rotated = (R @ fin.T).T
        poly.set_verts([rotated])

    ax1.set_title(f"Time: {time_array[i]:.2f} s")
    return lines_filtered + nose_polys + body_polys + fins_polys


# Animate
ani = FuncAnimation(fig, update, frames=len(quats), interval=50, blit=False)
ani.save('rocket_animation.mp4', writer='ffmpeg', fps=20, dpi=200)

plt.tight_layout()
# plt.show()
