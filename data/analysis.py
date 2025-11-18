import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)
from matplotlib.animation import FuncAnimation

# Load CSV (adjust filename if needed)
data = pd.read_csv("MIDAS_booster.csv")

# Filter for rows where sensor == "orientation"
# data = data[data["sensor"] == "orientation"]

# Extract quaternion columns
cols = [
    "sensor",
    "timestamp",
    "orientation.orientation_quaternion.w",
    "orientation.orientation_quaternion.x",
    "orientation.orientation_quaternion.y",
    "orientation.orientation_quaternion.z",
    "orientation.has_data",
    "orientation.reading_type",
]

for c in cols:
    if c not in data.columns:
        raise RuntimeError(f"Expected column '{c}' not found in CSV")

qdf = data[cols].copy()
qdf.to_csv("orientation_quaternions.csv", index=False)
print(f"Filtered to {len(qdf)} rows with sensor='orientation'")

# print(qdf["orientation.has_data"].head(30))
# print(len(data[data["orientation.has_data"] == 0]))

# Convert to numpy arrays
ts = qdf["timestamp"].to_numpy(dtype=float)
qw = qdf["orientation.orientation_quaternion.w"].to_numpy(dtype=float)
qx = qdf["orientation.orientation_quaternion.x"].to_numpy(dtype=float)
qy = qdf["orientation.orientation_quaternion.y"].to_numpy(dtype=float)
qz = qdf["orientation.orientation_quaternion.z"].to_numpy(dtype=float)


delta_t_values = qdf["timestamp"].diff().dropna()
# print(delta_t_values)

average_delta_t = delta_t_values.mean()
# print(f"Average delta t: {average_delta_t}")

average_delta_t_seconds = average_delta_t / 1000
print(f"Average delta t in seconds: {average_delta_t_seconds}")

sampling_frequency_hz = 1 / average_delta_t_seconds
print(f"Sampling frequency: {sampling_frequency_hz} Hz")

# Compute quaternion norm and basic diagnostics
qnorm = np.sqrt(
    np.nan_to_num(qw) ** 2
    + np.nan_to_num(qx) ** 2
    + np.nan_to_num(qy) ** 2
    + np.nan_to_num(qz) ** 2
)
finite_mask = (
    np.isfinite(qnorm)
    & np.isfinite(qw)
    & np.isfinite(qx)
    & np.isfinite(qy)
    & np.isfinite(qz)
)
bad_mask = (~finite_mask) | (qnorm < 1e-6) | (qnorm < 0.9) | (qnorm > 1.1)

print(f"Quaternion samples: {len(qnorm)}")
print(f"Non-finite quaternion components: {np.count_nonzero(~finite_mask)}")
print(
    f"Out-of-range norms (<0.9 or >1.1) : {np.count_nonzero((qnorm < 0.9) | (qnorm > 1.1))}"
)

# Plot components and norm (commented out to allow animation to run)
# Uncomment below to see diagnostics instead of animation
# plt.figure(figsize=(10, 8))
#
# ax1 = plt.subplot(2, 1, 1)
# ax1.plot(ts, qw, label="w")
# ax1.plot(ts, qx, label="x")
# ax1.plot(ts, qy, label="y")
# ax1.plot(ts, qz, label="z")
# ax1.scatter(ts[bad_mask], qw[bad_mask], color="red", s=10, label="bad samples")
# ax1.set_ylabel("Quaternion components")
# ax1.legend(loc="upper right")
# ax1.grid(True)
#
# ax2 = plt.subplot(2, 1, 2, sharex=ax1)
# ax2.plot(ts, qnorm, label="norm", color="black")
# ax2.axhline(1.0, color="green", linestyle="--", label="ideal=1.0")
# ax2.axhline(0.9, color="orange", linestyle=":", label="tolerance 0.9-1.1")
# ax2.axhline(1.1, color="orange", linestyle=":")
# ax2.scatter(ts[bad_mask], qnorm[bad_mask], color="red", s=12)
# ax2.set_ylabel("Quaternion norm")
# ax2.set_xlabel("timestamp")
# ax2.legend()
# ax2.grid(True)
#
# plt.suptitle("Quaternion components and norm over time")
# plt.tight_layout(rect=[0, 0, 1, 0.96])
# plt.show()

# ------------------------- 3D Orientation Visualizer -------------------------


# Build rotation matrix from quaternion (w, x, y, z)
def quat_to_rot_matrix(w, x, y, z):
    # normalize and guard
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if not np.isfinite(n) or n < 1e-8:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    # Rotation matrix (Hamilton convention)
    R = np.zeros((3, 3))
    R[0, 0] = 1 - 2 * (y * y + z * z)
    R[0, 1] = 2 * (x * y - z * w)
    R[0, 2] = 2 * (x * z + y * w)
    R[1, 0] = 2 * (x * y + z * w)
    R[1, 1] = 1 - 2 * (x * x + z * z)
    R[1, 2] = 2 * (y * z - x * w)
    R[2, 0] = 2 * (x * z - y * w)
    R[2, 1] = 2 * (y * z + x * w)
    R[2, 2] = 1 - 2 * (x * x + y * y)
    return R


# Define a simple cube centered at origin
cube_size = 0.5
half = cube_size / 2.0
cube_vertices = np.array(
    [
        [-half, -half, -half],
        [half, -half, -half],
        [half, half, -half],
        [-half, half, -half],
        [-half, -half, half],
        [half, -half, half],
        [half, half, half],
        [-half, half, half],
    ]
)

# Cube edges (pairs of vertex indices)
edges = [
    (0, 1),
    (1, 2),
    (2, 3),
    (3, 0),  # bottom
    (4, 5),
    (5, 6),
    (6, 7),
    (7, 4),  # top
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7),  # verticals
]

# Axis unit vectors for drawing body axes (X=red, Y=green, Z=blue)
axes = np.eye(3) * (cube_size * 1.2)


fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection="3d")
ax.set_box_aspect((1, 1, 1))

# Set axis limits
lim = cube_size * 2.0
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_zlim(-lim, lim)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("Orientation Visualizer (Quaternion -> 3D)")

# Lines for cube edges and axis arrows
edge_lines = [ax.plot([], [], [], color="black", linewidth=2)[0] for _ in edges]
axis_lines = [ax.plot([], [], [], color=c, linewidth=3)[0] for c in ("r", "g", "b")]
time_text = ax.text2D(0.02, 0.95, "", transform=ax.transAxes)


def init_anim():
    for ln in edge_lines + axis_lines:
        ln.set_data([], [])
        ln.set_3d_properties([])
    time_text.set_text("")
    return edge_lines + axis_lines + [time_text]


def update_anim(frame):
    # frame is index into quaternion arrays
    i = frame
    w, x, y, z = qw[i], qx[i], qy[i], qz[i]
    R = quat_to_rot_matrix(w, x, y, z)

    # rotate vertices
    verts_rot = (R @ cube_vertices.T).T

    # update edges
    for e_idx, (a, b) in enumerate(edges):
        p0 = verts_rot[a]
        p1 = verts_rot[b]
        xs = [p0[0], p1[0]]
        ys = [p0[1], p1[1]]
        zs = [p0[2], p1[2]]
        edge_lines[e_idx].set_data(xs, ys)
        edge_lines[e_idx].set_3d_properties(zs)

    # update axes
    origin = np.array([0.0, 0.0, 0.0])
    axes_rot = (R @ axes.T).T
    for k in range(3):
        p = axes_rot[k]
        axis_lines[k].set_data([0, p[0]], [0, p[1]])
        axis_lines[k].set_3d_properties([0, p[2]])

    # timestamp display (convert if timestamps are large)
    try:
        ts_val = ts[i]
        # If timestamp looks like microseconds or large ints, normalize to seconds
        if ts_val > 1e12:
            ts_disp = ts_val / 1e6
        elif ts_val > 1e9:
            ts_disp = ts_val / 1e3
        else:
            ts_disp = ts_val
    except Exception:
        ts_disp = i
    time_text.set_text(
        f"index={i}  timestamp={ts_disp:.3f}  q=({w:.3f},{x:.3f},{y:.3f},{z:.3f})"
    )

    if i % 50 == 0:
        print(f"  Frame {i}: q=({w:.3f},{x:.3f},{y:.3f},{z:.3f})")

    return edge_lines + axis_lines + [time_text]


# Choose frame indices: use all samples or subsample if too many
max_frames = 800
N = len(ts)
if N == 0:
    raise RuntimeError("No quaternion samples found")
step = max(1, N // max_frames)
frame_indices = list(range(0, N, step))

interval_ms = 1
print(f"\nStarting 3D orientation animation...")
print(f"Total frames: {len(frame_indices)}, interval: {interval_ms}ms")
print(f"Subsampling: every {step}th sample")
print(f"Close the animation window to complete.\n")


ani = FuncAnimation(
    fig,
    update_anim,
    frames=frame_indices,
    init_func=init_anim,
    interval=interval_ms,
    blit=False,
    repeat=True,
)

print(f"Animation ready. Displaying...")
plt.show()
print(f"Animation complete.")
