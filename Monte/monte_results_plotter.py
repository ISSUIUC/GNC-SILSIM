from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


run_folder = Path(r"C:\Users\ahmed\Silsim\GNC-SILSIM\Monte\Monte_Output")

sim_dir = run_folder / "SimData"
fig_dir = run_folder / "figures"
fig_dir.mkdir(parents=True, exist_ok=True)

nominal = np.load(sim_dir / "nominal.npy")
sample_paths = sorted(sim_dir.glob("sim_data_*.npy"))

plt.figure(figsize=(11, 6))

for path in sample_paths:
    data = np.load(path)

    time = data[:, 18]
    #print(time)
    true_pos_x = data[:, 0]
    ekf_pos_x = data[:, 34]

    plt.plot(
        time,
        true_pos_x,
        #color="tab:blue",
        alpha=0.25,
        linewidth=1,
    )

    plt.plot(
        time,
        ekf_pos_x,
        #color="tab:red",
        alpha=0.18,
        linewidth=1,
        linestyle="--",
    )
    print(f"Plotted: {path.name}")

plt.plot(
    nominal[:, 18],
    nominal[:, 0],
    color="black",
    linewidth=2.5,
    label="Nominal true pos_x",
)

plt.plot(
    nominal[:, 18],
    nominal[:, 34],
    color="darkred",
    linewidth=2,
    linestyle="--",
    label="Nominal EKF pos_x",
)

plt.title("Monte Carlo pos_x vs Time")
plt.xlabel("Time (s)")
plt.ylabel("pos_x / Altitude (m)")
plt.grid(True, alpha=0.3)
plt.legend()
plt.tight_layout()

out_path = fig_dir / "pos_x_vs_time_all_runs_with_ekf.png"
plt.savefig(out_path, dpi=200)
plt.show()

print(f"Saved: {out_path}")
