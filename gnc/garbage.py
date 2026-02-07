import numpy as np
import pandas as pd

# Load data from MIDAS sustainer acceleration
df = pd.read_csv("./data/MIDAS_Sustainer.csv")

# Extract acceleration and integrate to get velocity and position
accel_cols = ["highg.ax", "highg.ay", "highg.az"]
time = df["timestamp"].values
dt = np.diff(time)

# Initialize arrays
velocity = np.zeros((len(time), 3))
position = np.zeros((len(time), 3))

for i, col in enumerate(accel_cols):
    accel = df[col].values
    # Integrate acceleration to get velocity
    velocity[1:, i] = np.cumsum(accel[:-1] * dt)
    # Integrate velocity to get position
    position[1:, i] = np.cumsum(velocity[1:, i] * dt)

# Plot position and velocity
import matplotlib.pyplot as plt

fig, (ax1) = plt.subplots(1, 1, figsize=(10, 8))

ax1.plot(time, position[:, 0], label="Pos X")
ax1.plot(time, position[:, 1], label="Pos Y")
ax1.plot(time, position[:, 2], label="Pos Z")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Position (m)")
ax1.ticklabel_format(style="plain", axis="y", useOffset=False)
ax1.set_title("Why GNC engineers have trust issues (Time vs Position for October 2025)")
ax1.legend()
ax1.grid()

plt.tight_layout()
plt.show()
