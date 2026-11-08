import pandas as pd
from pathlib import Path
import numpy as np 

files = [
r'C:\Users\ahmed\Silsim\GNC-SILSIM\Barometer\data\midas_booster_flight.csv',
r'C:\Users\ahmed\Silsim\GNC-SILSIM\Barometer\data\midas_sustainer_flight.csv'
]

# output column -> source column in the new format
mapping = {
    "timestamp_ms":       "timestamp_ms",
    "acceleration_g":     "imu.lowg_acceleration.ax",
    "altitude_m":        "barometer.altitude",
    "vertical_speed_mps": "kalman.velocity.vx",
     #"vx_mps":             "kalman.velocity.vx",
    "tilt_deg":           "angular_kalman.mq_tilt",
}

out_dir = Path(r"C:\Users\ahmed\Silsim\GNC-SILSIM\Barometer\data")
out_dir.mkdir(parents=True, exist_ok=True)

for f in files:
    df = pd.read_csv(f, skipinitialspace=True)
    df.columns = df.columns.str.strip()

    out = df[list(mapping.values())].copy()
    out.columns = list(mapping.keys())
    out['timestamp_ms'] = out['timestamp_ms'] - out['timestamp_ms'][0]
    out_path = out_dir / (Path(f).stem + "_filtered.csv")
    out.to_csv(out_path, index=False)
    print(f"Saved {out_path} ({len(out)} rows)")

    print(f"Pyro:1 _ {f} = {out['timestamp_ms'][np.argmax(out['pyro.channel_firing[0]'])]}")
    print(f"Pyro:2  _ {f}= {out['timestamp_ms'][np.argmax(out['pyro.channel_firing[1]'])]}")
    print(f"Pyro:3 _ {f}= {out['timestamp_ms'][np.argmax(out['pyro.channel_firing[2]'])]}")