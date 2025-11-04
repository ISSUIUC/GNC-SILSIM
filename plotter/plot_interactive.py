#!/usr/bin/env python3

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons, RadioButtons
import sys
import os

def plot_interactive_ekf_results(csv_file):
    """Create an interactive EKF results plot with zoom/pan and plot selection"""
    
    # Get the directory of the CSV file to save plots in the same location
    csv_dir = os.path.dirname(os.path.abspath(csv_file))
    if not csv_dir:
        csv_dir = "."
    
    print(f"Loading data from {csv_file}")
    df = pd.read_csv(csv_file)
    print(f"Loaded {len(df)} data points from {csv_file}")
    
    time = df['timestamp']
    
    fig, ax = plt.subplots(figsize=(14, 10))
    plt.subplots_adjust(left=0.1, bottom=0.3, right=0.7, top=0.95)
    
    plot_options = {
        'KF Position X': (df['pos_x'], 'Position (m)', 'b-'),
        'KF Position Y': (df['pos_y'], 'Position (m)', 'g-'),
        'KF Position Z': (df['pos_z'], 'Position (m)', 'r-'),
        'KF Velocity X': (df['vel_x'], 'Velocity (m/s)', 'b--'),
        'KF Velocity Y': (df['vel_y'], 'Velocity (m/s)', 'g--'),
        'KF Velocity Z': (df['vel_z'], 'Velocity (m/s)', 'r--'),
        'KF Acceleration X': (df['acc_x'], 'Acceleration (m/s²)', 'b:'),
        'KF Acceleration Y': (df['acc_y'], 'Acceleration (m/s²)', 'g:'),
        'KF Acceleration Z': (df['acc_z'], 'Acceleration (m/s²)', 'r:'),
        'KF Altitude': (df['altitude'], 'Altitude (m)', 'purple'),
    }
    
    if 'raw_baro_alt' in df.columns:
        plot_options['Raw Baro Altitude'] = (df['raw_baro_alt'], 'Altitude (m)', 'orange')
        print("Found raw barometer data")
    if 'raw_highg_ax' in df.columns:
        plot_options['Raw HighG X (m/s²)'] = (df['raw_highg_ax'] * 9.81, 'Acceleration (m/s²)', 'orange')
        print("Found raw HighG X data")
    if 'raw_highg_ay' in df.columns:
        plot_options['Raw HighG Y (m/s²)'] = (df['raw_highg_ay'] * 9.81, 'Acceleration (m/s²)', 'orange')
        print("Found raw HighG Y data")
    if 'raw_highg_az' in df.columns:
        plot_options['Raw HighG Z (m/s²)'] = (df['raw_highg_az'] * 9.81, 'Acceleration (m/s²)', 'orange')
        print("Found raw HighG Z data")
    
    fsm_changes = []
    if 'fsm' in df.columns:
        current_fsm = None
        for i, fsm in enumerate(df['fsm']):
            if fsm != current_fsm:
                fsm_changes.append((time.iloc[i], fsm))
                current_fsm = fsm
        print(f"Found {len(fsm_changes)} FSM state changes")
    
    lines = {}
    for name, (data, ylabel, style) in plot_options.items():
        if 'Raw' not in name:
            line, = ax.plot(time, data, style, linewidth=1.5, label=name)
            lines[name] = line
        else:
            line, = ax.plot(time, data, style, linewidth=1.5, label=name, visible=False)
            lines[name] = line
    
    fsm_lines = []
    for t, label in fsm_changes:
        line = ax.axvline(x=t, color='k', linestyle='--', alpha=0.3, linewidth=1)
        fsm_lines.append((line, t, label))
        ax.text(t, ax.get_ylim()[1], label, rotation=90, 
                verticalalignment='bottom', fontsize=8, color='k', alpha=0.7)
    
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Value')
    ax.set_title('Interactive EKF Results - Select plots to display')
    ax.grid(True, alpha=0.3)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    
    ax_check = plt.axes([0.02, 0.02, 0.15, 0.25])
    checkbox_labels = list(plot_options.keys())
    initial_states = [name not in plot_options or 'Raw' not in name for name in checkbox_labels]
    check = CheckButtons(ax_check, checkbox_labels, initial_states)
    
    ax_radio = plt.axes([0.02, 0.3, 0.15, 0.2])
    radio = RadioButtons(ax_radio, ['All KF', 'Position Only', 'Velocity Only', 'Acceleration Only', 'Raw Sensors Only', 'Custom'])
    
    def update_plots(label):
        """Update which plots are visible based on checkboxes"""
        checkbox_states = check.get_status()
        
        for i, (name, line) in enumerate(lines.items()):
            if i < len(checkbox_states):
                line.set_visible(checkbox_states[i])
        
        visible_lines = [line for line in lines.values() if line.get_visible()]
        if visible_lines:
            ax.legend(visible_lines, [line.get_label() for line in visible_lines], 
                     bbox_to_anchor=(1.05, 1), loc='upper left')
        else:
            ax.legend().remove()
        
        fig.canvas.draw()
    
    def update_plot_type(label):
        """Update plot selection based on radio button"""
        if label == 'All KF':
            for i, name in enumerate(checkbox_labels):
                check.set_active(i, 'Raw' not in name)
        elif label == 'Position Only':
            for i, name in enumerate(checkbox_labels):
                check.set_active(i, 'Position' in name and 'Raw' not in name)
        elif label == 'Velocity Only':
            for i, name in enumerate(checkbox_labels):
                check.set_active(i, 'Velocity' in name and 'Raw' not in name)
        elif label == 'Acceleration Only':
            for i, name in enumerate(checkbox_labels):
                check.set_active(i, 'Acceleration' in name and 'Raw' not in name)
        elif label == 'Raw Sensors Only':
            for i, name in enumerate(checkbox_labels):
                check.set_active(i, 'Raw' in name)
        elif label == 'Custom':
            pass
        
        update_plots('radio_update')
    
    check.on_clicked(update_plots)
    radio.on_clicked(update_plot_type)
    
    fig.text(0.02, 0.9, 'Controls:\n• Mouse wheel: Zoom\n• Right-click drag: Pan\n• Checkboxes: Select plots\n• Radio buttons: Quick selections', 
             fontsize=9, verticalalignment='top')
    
    ax.set_xlim(time.min(), time.max())
    
    print("\n=== EKF Simulation Summary ===")
    print(f"Total flight time: {time.iloc[-1]:.2f} seconds")
    print(f"Maximum altitude: {df['pos_z'].max():.2f} m")
    print(f"Maximum velocity magnitude: {np.sqrt(df['vel_x']**2 + df['vel_y']**2 + df['vel_z']**2).max():.2f} m/s")
    print(f"Maximum acceleration magnitude: {np.sqrt(df['acc_x']**2 + df['acc_y']**2 + df['acc_z']**2).max():.2f} m/s²")
    
    print(f"\nFinal position: ({df['pos_x'].iloc[-1]:.2f}, {df['pos_y'].iloc[-1]:.2f}, {df['pos_z'].iloc[-1]:.2f}) m")
    print(f"Final velocity: ({df['vel_x'].iloc[-1]:.2f}, {df['vel_y'].iloc[-1]:.2f}, {df['vel_z'].iloc[-1]:.2f}) m/s")
    
    if fsm_changes:
        unique_states = list(set([label for _, label in fsm_changes]))
        print(f"\nFSM States found: {', '.join(unique_states)}")
    
    # PNG saving disabled during run_sim
    
    plt.ion()
    
    from matplotlib.backends.backend_tkagg import NavigationToolbar2Tk
    plt.show(block=False)
    
    print("\n=== Interactive Plot Controls ===")
    print("• Mouse wheel: Zoom in/out")
    print("• Right-click + drag: Pan around")
    print("• Left-click + drag: Select region to zoom")
    print("• Checkboxes: Toggle individual plots")
    print("• Radio buttons: Quick plot selections")
    print("• Close window to exit")
    
    print("\n" + "="*50)
    print("Interactive plot is now displayed. Press 'w' and Enter to close and continue...")
    print("="*50)
    try:
        user_input = input("Press 'w' + Enter to close interactive plot: ").strip().lower()
        while user_input != 'w':
            print("Please press 'w' and Enter to close the interactive plot.")
            user_input = input("Press 'w' + Enter to close interactive plot: ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\nInteractive plot closed.")
    
    plt.close('all')

def main():
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        csv_file = "output/results.csv"
    
    plot_interactive_ekf_results(csv_file)

if __name__ == "__main__":
    main()