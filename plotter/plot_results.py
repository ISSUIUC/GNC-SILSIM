

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def benchmark_results(csv_file1 ="output/TeleMega (AL3, EEPROM).csv", csv_file2 = "output/results.csv"):
    
    def exist(file):
        if not os.path.exists(file):        
            print(f"Error: File {csv_file1} not found")
            print("Please run the C++ simulation first to generate results")
        return
    exist(csv_file1)
    exist(csv_file2)

    
    # Get the directory of the CSV file to save plots in the same location
    csv_dir1 = os.path.dirname(os.path.abspath(csv_file1))
    csv_dir2 = os.path.dirname(os.path.abspath(csv_file2))
    if not csv_dir1:
        csv_dir1 = "."
    if not csv_dir2:
        csv_dir2 = "."
    
    try:
        df1 = pd.read_csv(csv_file1)
        df2 = pd.read_csv(csv_file2)
        print(f"Loaded {len(df1)} data points from {csv_file1}")
        print(f"Loaded {len(df2)} data points from {csv_file2}")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    

    time1 = df1['time'] # Telemega Time
    time2 = df2['timestamp'] # ISS EKF Time


    # df2['pos_x'] += 100
    # df1['altitude'] -= 100

    # Add 3.5 seconds to time2 in EKF results to align with Telemega data
    # time2 -= 3.5

    def align_position_data(time1, data1, time2, data2):
        # Always starts at zero
        offset1 = data1[0]
        offset2 = data2[0]

        data1_zero_aligned = data1 - offset1
        data2_zero_aligned = data2 - offset2

        # find index of max along the columns of data
        max1_index = data1_zero_aligned.idxmax()
        max2_index = data2_zero_aligned.idxmax()

        time_diff = time1[max1_index] - time2[max2_index]

        time2_aligned = time2 + time_diff

        return time2_aligned, data1_zero_aligned, data2_zero_aligned
    
    
    time2_aligned, altitude1_aligned, altitude2_aligned = align_position_data(time1, df1['altitude'], time2, df2['pos_x'])


    fig1, ax = plt.subplots(3, 2, figsize=(15,12))
    ax[0,0].plot(time1, altitude1_aligned, label='Telemega')
    ax[0,0].set_title('Position X')
    ax[0,0].set_ylabel('Position (m)')

    ax[0,0].plot(time2_aligned, altitude2_aligned, label = 'ISS EKF')
    ax[0,0].legend()

    ax[0,1].plot(time1, df1['accel_x'], label='Telemega')
    ax[0,1].set_title('Acceleration X')
    ax[0,1].set_ylabel("Acceleration (m/s^2)")

    ax[0,1].plot(time2_aligned, df2['acc_x'], label = 'ISS EKF')
    ax[0,1].legend()

    ax[1,1].plot(time1, df1['accel_y'], label = 'Telemega')
    ax[1,1].set_title('Acceleration Y')
    ax[1,1].set_ylabel("Acceleration (m/s^2)")

    ax[1,1].plot(time2_aligned, df2['acc_y'], label = 'ISS EKF')
    ax[1,1].legend()

    
    ax[2,1].plot(time1, df1['accel_z'], label = 'Telemega')
    ax[2,1].set_title('Acceleration Z')
    ax[2,1].set_ylabel("Acceleration (m/s^2)")

    ax[2,1].plot(time2_aligned, df2['acc_z'], label = 'ISS EKF')
    ax[2,1].legend()
    
    plt.show()




def plot_ekf_results(csv_file="output/results.csv"):
    if not os.path.exists(csv_file):
        print(f"Error: File {csv_file} not found")
        print("Please run the C++ simulation first to generate results")
        return
    
    # Get the directory of the CSV file to save plots in the same location
    csv_dir = os.path.dirname(os.path.abspath(csv_file))
    if not csv_dir:
        csv_dir = "."
    
    try:
        df = pd.read_csv(csv_file)
        print(f"Loaded {len(df)} data points from {csv_file}")
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    fig1, axes1 = plt.subplots(3, 3, figsize=(15, 12))
    fig1.suptitle('EKF Simulation Results - Standalone Plots', fontsize=16)
    
    fig2, axes2 = plt.subplots(2, 2, figsize=(12, 10))
    fig2.suptitle('EKF Simulation Results - Comparison Plots', fontsize=16)
    
    time = df['timestamp']
    
    fsm_changes = []
    used_states_order = []
    if 'fsm' in df.columns:
        prev = None
        for i, (t, s) in enumerate(zip(df['timestamp'], df['fsm'])):
            if i == 0:
                prev = s
                fsm_changes.append((t, s))
                if s not in used_states_order:
                    used_states_order.append(s)
            else:
                if s != prev:
                    fsm_changes.append((t, s))
                    if s not in used_states_order:
                        used_states_order.append(s)
                    prev = s
    
    axes1[0, 0].plot(time, df['pos_x'], 'b-', linewidth=1)
    axes1[0, 0].set_title('Position X (Altitude)')
    axes1[0, 0].set_ylabel('Position (m)')
    axes1[0, 0].grid(True)
    
    axes1[0, 1].plot(time, df['pos_y'], 'g-', linewidth=1)
    axes1[0, 1].set_title('Position Y')
    axes1[0, 1].set_ylabel('Position (m)')
    axes1[0, 1].grid(True)
    
    axes1[0, 2].plot(time, df['pos_z'], 'r-', linewidth=1)
    axes1[0, 2].set_title('Position Z')
    axes1[0, 2].set_ylabel('Position (m)')
    axes1[0, 2].grid(True)
    
    axes1[1, 0].plot(time, df['vel_x'], 'b-', linewidth=1)
    axes1[1, 0].set_title('Velocity X')
    axes1[1, 0].set_ylabel('Velocity (m/s)')
    axes1[1, 0].grid(True)
    
    axes1[1, 1].plot(time, df['vel_y'], 'g-', linewidth=1)
    axes1[1, 1].set_title('Velocity Y')
    axes1[1, 1].set_ylabel('Velocity (m/s)')
    axes1[1, 1].grid(True)
    
    axes1[1, 2].plot(time, df['vel_z'], 'r-', linewidth=1)
    axes1[1, 2].set_title('Velocity Z')
    axes1[1, 2].set_ylabel('Velocity (m/s)')
    axes1[1, 2].grid(True)
    
    axes1[2, 0].plot(time, df['acc_x'], 'b-', linewidth=1)
    axes1[2, 0].set_title('Acceleration X')
    axes1[2, 0].set_ylabel('Acceleration (m/s²)')
    axes1[2, 0].set_xlabel('Time (s)')
    axes1[2, 0].grid(True)
    
    axes1[2, 1].plot(time, df['acc_y'], 'g-', linewidth=1)
    axes1[2, 1].set_title('Acceleration Y')
    axes1[2, 1].set_ylabel('Acceleration (m/s²)')
    axes1[2, 1].set_xlabel('Time (s)')
    axes1[2, 1].grid(True)
    
    axes1[2, 2].plot(time, df['acc_z'], 'r-', linewidth=1)
    axes1[2, 2].set_title('Acceleration Z')
    axes1[2, 2].set_ylabel('Acceleration (m/s²)')
    axes1[2, 2].set_xlabel('Time (s)')
    axes1[2, 2].grid(True)
    
    if 'raw_highg_ax' in df.columns and 'raw_baro_alt' in df.columns:
        raw_acc_x = df['raw_highg_ax'] * 9.81
        raw_acc_y = df['raw_highg_ay'] * 9.81  
        raw_acc_z = df['raw_highg_az'] * 9.81
        
        axes2[0, 0].plot(time, df['pos_x'], 'b-', linewidth=2, label='KF Z Position')
        axes2[0, 0].plot(time, df['raw_baro_alt'], 'r--', linewidth=1, alpha=0.7, label='Raw Barometer Altitude')
        axes2[0, 0].set_title('KF Position Z vs Barometric Altitude')
        axes2[0, 0].set_ylabel('Altitude (m)')
        axes2[0, 0].set_xlabel('Time (s)')
        axes2[0, 0].legend()
        axes2[0, 0].grid(True)
        
        axes2[0, 1].plot(time, df['acc_x'], 'b-', linewidth=2, label='KF Z Acceleration')
        axes2[0, 1].plot(time, raw_acc_x, 'r--', linewidth=1, alpha=0.7, label='Raw HighG X')
        axes2[0, 1].set_title('KF Acceleration X vs Raw HighG X')
        axes2[0, 1].set_ylabel('Acceleration (m/s²)')
        axes2[0, 1].set_xlabel('Time (s)')
        axes2[0, 1].legend()
        axes2[0, 1].grid(True)
        
        axes2[1, 0].plot(time, df['acc_y'], 'g-', linewidth=2, label='KF Y Acceleration')
        axes2[1, 0].plot(time, raw_acc_y, 'r--', linewidth=1, alpha=0.7, label='Raw HighG Y')
        axes2[1, 0].set_title('KF Acceleration Y vs Raw HighG Y')
        axes2[1, 0].set_ylabel('Acceleration (m/s²)')
        axes2[1, 0].set_xlabel('Time (s)')
        axes2[1, 0].legend()
        axes2[1, 0].grid(True)
        
        axes2[1, 1].plot(time, df['acc_z'], 'b-', linewidth=2, label='KF X Acceleration')
        axes2[1, 1].plot(time, raw_acc_z, 'r--', linewidth=1, alpha=0.7, label='Raw HighG Z')
        axes2[1, 1].set_title('KF Acceleration Z vs Raw HighG Z')
        axes2[1, 1].set_ylabel('Acceleration (m/s²)')
        axes2[1, 1].set_xlabel('Time (s)')
        axes2[1, 1].legend()
        axes2[1, 1].grid(True)
    else:
        axes2[0, 0].text(0.5, 0.5, 'Raw sensor data not available', ha='center', va='center', transform=axes2[0, 0].transAxes)
        axes2[0, 0].set_title('KF Position Z vs Barometric Altitude')
        axes2[0, 1].text(0.5, 0.5, 'Raw sensor data not available', ha='center', va='center', transform=axes2[0, 1].transAxes)
        axes2[0, 1].set_title('KF Acceleration Z vs Raw HighG X')
        axes2[1, 0].text(0.5, 0.5, 'Raw sensor data not available', ha='center', va='center', transform=axes2[1, 0].transAxes)
        axes2[1, 0].set_title('KF Acceleration Y vs Raw HighG Y')
        axes2[1, 1].text(0.5, 0.5, 'Raw sensor data not available', ha='center', va='center', transform=axes2[1, 1].transAxes)
        axes2[1, 1].set_title('KF Acceleration X vs Raw HighG Z')
    

    if fsm_changes:
        for t, label in fsm_changes:
            for ax in axes1.flat:
                ax.axvline(x=t, color='k', linestyle='--', alpha=0.25)
            for ax in axes2.flat:
                ax.axvline(x=t, color='k', linestyle='--', alpha=0.25)

        annotated = set()
        ylim_top1 = axes1[0, 0].get_ylim()[1]
        ylim_top2 = axes2[0, 0].get_ylim()[1]
        for t, label in fsm_changes:
            if label not in annotated:
                axes1[0, 0].text(t, ylim_top1, label, rotation=90,
                                verticalalignment='bottom', fontsize=8, color='k')
                axes2[0, 0].text(t, ylim_top2, label, rotation=90,
                                verticalalignment='bottom', fontsize=8, color='k')
                annotated.add(label)

        if used_states_order:
            legend_text = 'FSM states used: ' + ', '.join(used_states_order)
            fig1.text(0.5, 0.98, legend_text, ha='center', va='top', fontsize=9)
            fig2.text(0.5, 0.98, legend_text, ha='center', va='top', fontsize=9)
    
    fig1.tight_layout()
    fig1.show()
    # PNG saving disabled during run_sim
    
    fig2.tight_layout()
    fig2.show()
    # PNG saving disabled during run_sim
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.plot(df['pos_z'], df['pos_y'], df['pos_x'], 'b-', linewidth=2, label='Rocket Trajectory')
    ax.scatter(df['pos_z'].iloc[0], df['pos_y'].iloc[0], df['pos_x'].iloc[0], 
               color='green', s=100, label='Start')
    ax.scatter(df['pos_z'].iloc[-1], df['pos_y'].iloc[-1], df['pos_x'].iloc[-1], 
               color='red', s=100, label='End')
    
    ax.set_xlabel('Position Z (m)')
    ax.set_ylabel('Position Y (m)')
    ax.set_zlabel('Position X (m)')
    ax.set_title('3D Rocket Trajectory')
    ax.legend()
    
    plt.show()
    # PNG saving disabled during run_sim
    
    # Wait for user input before closing
    print("\n" + "="*50)
    print("Plots are now displayed. Press 'w' and Enter to close and continue...")
    print("="*50)
    try:
        user_input = input("Press 'w' + Enter to close plots: ").strip().lower()
        while user_input != 'w':
            print("Please press 'w' and Enter to close the plots.")
            user_input = input("Press 'w' + Enter to close plots: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\nPlots closed.")
    
    plt.close('all')
    
    print("\n=== EKF Simulation Summary ===")
    print(f"Total flight time: {time.iloc[-1]:.2f} seconds")
    print(f"Maximum altitude: {df['pos_z'].max():.2f} m")
    print(f"Maximum velocity magnitude: {np.sqrt(df['vel_x']**2 + df['vel_y']**2 + df['vel_z']**2).max():.2f} m/s")
    print(f"Maximum acceleration magnitude: {np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2).max():.2f} m/s²")
    
    print(f"\nFinal position: ({df['pos_x'].iloc[-1]:.2f}, {df['pos_y'].iloc[-1]:.2f}, {df['pos_z'].iloc[-1]:.2f}) m")
    print(f"Final velocity: ({df['vel_x'].iloc[-1]:.2f}, {df['vel_y'].iloc[-1]:.2f}, {df['vel_z'].iloc[-1]:.2f}) m/s")

def main():
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file2 = "output/results.csv"
        csv_file1 = "output/TeleMega (AL3, EEPROM).csv"
    
    # plot_ekf_results(csv_file)
    benchmark_results(csv_file1, csv_file2)

if __name__ == "__main__":
    main()
