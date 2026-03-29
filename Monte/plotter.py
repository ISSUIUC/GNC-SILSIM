import pandas as pd
import matplotlib.pyplot as plt

# === LOAD DATA ===
file_path = r"C:\Users\ahmed\Silsim\GNC-SILSIM\Monte\rocket_sim_cpp\6DOF_RK4_SIMULATED.csv"  # <-- change to your CSV filename
df = pd.read_csv(file_path)

# === CLEAN COLUMN NAMES (optional but helpful) ===
df.columns = df.columns.str.strip()

# === TIME AXIS ===
# Try to use timestamp if available, otherwise fallback to index
if "timestamp" in df.columns:
    x = df["timestamp"]
    plt.xlabel("Timestamp")
else:
    x = df.index
    plt.xlabel("Sample Index")

# === CREATE FIGURE ===
plt.figure(figsize=(12, 8))

# === EXAMPLE PLOTS ===
# You can comment/uncomment depending on what you want

# Altitude (barometer + kalman)
if "barometer.altitude" in df.columns:
    plt.plot(x, df["barometer.altitude"], label="Barometer Altitude")

if "kalman.altitude" in df.columns:
    plt.plot(x, df["kalman.altitude"], label="Kalman Altitude")

# # Acceleration (low-g)
# if "lowg.ax" in df.columns:
#     plt.plot(x, df["lowg.ax"], label="LowG Ax", alpha=0.7)
# if "lowg.ay" in df.columns:
#     plt.plot(x, df["lowg.ay"], label="LowG Ay", alpha=0.7)
# if "lowg.az" in df.columns:
#     plt.plot(x, df["lowg.az"], label="LowG Az", alpha=0.7)

# # High-G acceleration
# if "highg.az" in df.columns:
#     plt.plot(x, df["highg.az"], label="HighG Az", linestyle="--")

# # Voltage
# if "voltage.voltage" in df.columns:
#     plt.plot(x, df["voltage.voltage"], label="Voltage")

# # GPS Altitude
# if "gps.altitude" in df.columns:
#     plt.plot(x, df["gps.altitude"], label="GPS Altitude")

# # Orientation (pitch/roll)
# if "orientation.pitch" in df.columns:
#     plt.plot(x, df["orientation.pitch"], label="Pitch")
# if "orientation.roll" in df.columns:
#     plt.plot(x, df["orientation.roll"], label="Roll")

# === FINAL TOUCHES ===
plt.title("Telemetry Data Plot")
plt.legend()
plt.grid()

plt.tight_layout()
plt.show()