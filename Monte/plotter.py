from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

SCRIPT_DIR = Path(__file__).parent 
DEFAULT_CSV = SCRIPT_DIR / "rocket_sim_cpp" / "6DOF_RK4_SIMULATED.csv"

def load_data(file_path):
    df = pd.read_csv(file_path)
    df.columns = df.columns.str.strip()
    return df


def get_time_axis(df):
    if "timestamp" in df.columns:
        return df["timestamp"], "Timestamp"
    return df.index, "Sample Index"


def existing_columns(df, columns):
    return [column for column in columns if column in df.columns]


def add_series(ax, df, x, columns, ylabel: str, title: str):
    present = existing_columns(df, columns)
    if not present:
        ax.text(0.5, 0.5, "No matching columns found", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        return

    for column in present:
        ax.plot(x, df[column], label=column, linewidth=1.6)

    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")


def make_sensor_window(df, x, xlabel: str, figure_title: str, panels):
    fig, axes = plt.subplots(len(panels), 1, figsize=(12, 3.6 * len(panels)), sharex=True)
    fig.canvas.manager.set_window_title(figure_title)

    if len(panels) == 1:
        axes = [axes]

    for ax, panel in zip(axes, panels):
        add_series(
            ax,
            df,
            x,
            panel["columns"],
            panel["ylabel"],
            panel["title"],
        )

    axes[-1].set_xlabel(xlabel)
    fig.suptitle(figure_title, fontsize=14)
    fig.tight_layout()
    return fig


def main():
    df = load_data(DEFAULT_CSV)
    x, xlabel = get_time_axis(df)

    figures = [
        (
            "Barometer",
            [
                {
                    "title": "Barometer Altitude",
                    "columns": ["barometer.altitude", "kalman.altitude", "gps.altitude"],
                    "ylabel": "Altitude",
                },
                {
                    "title": "Barometer Pressure / Temperature",
                    "columns": ["barometer.pressure", "barometer.temperature"],
                    "ylabel": "Sensor Value",
                },
            ],
        ),
        (
            "Magnetometer",
            [
                {
                    "title": "Magnetometer",
                    "columns": ["magnetometer.mx", "magnetometer.my", "magnetometer.mz"],
                    "ylabel": "nT",
                },
                {
                    "title": "Orientation Magnetometer",
                    "columns": [
                        "orientation.magnetometer.mx",
                        "orientation.magnetometer.my",
                        "orientation.magnetometer.mz",
                    ],
                    "ylabel": "nT",
                },
            ],
        ),
        (
            "Acceleration",
            [
                {
                    "title": "High-G Accelerometer",
                    "columns": ["highg.ax", "highg.ay", "highg.az"],
                    "ylabel": "Acceleration",
                },
                {
                    "title": "Low-G Accelerometer",
                    "columns": ["lowg.ax", "lowg.ay", "lowg.az"],
                    "ylabel": "Acceleration",
                },
                {
                    "title": "Linear Acceleration",
                    "columns": [
                        "orientation.linear_acceleration.ax",
                        "orientation.linear_acceleration.ay",
                        "orientation.linear_acceleration.az",
                    ],
                    "ylabel": "Acceleration",
                },
            ],
        ),
        (
            "Velocity",
            [
                {
                    "title": "GPS Speed",
                    "columns": ["gps.speed"],
                    "ylabel": "Speed",
                },
                {
                    "title": "Orientation Velocity",
                    "columns": [
                        "orientation.orientation_velocity.vx",
                        "orientation.orientation_velocity.vy",
                        "orientation.orientation_velocity.vz",
                    ],
                    "ylabel": "Velocity",
                },
                {
                    "title": "Angular Velocity (orientation.*)",
                    "columns": [
                        "orientation.angular_velocity.vx",
                        "orientation.angular_velocity.vy",
                        "orientation.angular_velocity.vz",
                    ],
                    "ylabel": "Angular Velocity",
                },
            ],
        ),
        (
            "Gyro",
            [
                {
                    "title": "Orientation Gyro",
                    "columns": ["orientation.gx", "orientation.gy", "orientation.gz"],
                    "ylabel": "Angular Rate",
                },
                {
                    "title": "Low-G LSM Gyro",
                    "columns": ["lowglsm.gx", "lowglsm.gy", "lowglsm.gz"],
                    "ylabel": "Angular Rate",
                },
            ],
        ),
    ]

    for figure_title, panels in figures:
        make_sensor_window(df, x, xlabel, figure_title, panels)

    plt.show()


if __name__ == "__main__":
    main()
