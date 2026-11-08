import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
#data_frame = pd.read_csv('../data/Ashley_Data_MIDAS_MINI.csv') # nominal data
plot_data_raw = True 
data_frame = pd.read_csv(r'C:\Users\ahmed\Silsim\GNC-SILSIM\Barometer\data\midas_booster_flight.csv')
# t_ms = data_frame['timestamp_ms'] - data_frame['timestamp_ms'][0]
# for ch in range(3):
#     col = data_frame[f'pyro.channel_firing[{ch}]'].to_numpy()
#     max_val = col.max()
#     if max_val == 0:
#         print(f"Pyro {ch+1}: never fired")
#         continue

#     idxs = np.flatnonzero(col == max_val)  # every row at the max
#     first, last = idxs[0], idxs[-1]
#     print(f"Pyro {ch+1}: max={max_val}, "
#           f"first row={first} (t={t_ms[first]} ms), "
#           f"last row={last} (t={t_ms[last]} ms), "
#           f"rows at max={len(idxs)}")

if plot_data_raw:
    # plt.figure(figsize=(10, 6))
    # plt.plot(data_frame['timestamp_ms'], data_frame['kalman.position.px'], label='Altitude', color='blue')
    # plt.title('KF Altitude vs Time')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Altitude (m)')
    # plt.grid()


    plt.figure(figsize=(10, 6))
    plt.plot(data_frame['timestamp_ms'], data_frame['barometer.temperature'], label='Altitude', color='blue')
    plt.title('Temperature vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Temperature')
    plt.grid()

    plt.figure(figsize=(10, 6))
    plt.plot(data_frame['timestamp_ms'], data_frame['barometer.pressure'], label='Altitude', color='blue')
    plt.title('Pressure vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Pressure')
    plt.grid()

    plt.figure(figsize=(10, 6))
    plt.plot(data_frame['timestamp_ms'], data_frame['barometer.altitude'], label='Altitude', color='blue')
    plt.title('Barometer Altitude vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.grid()



    plt.figure(figsize=(10, 6))
    plt.plot(data_frame['timestamp_ms'], data_frame['imu.highg_acceleration.ax'], label='accel', color='blue')
    plt.title('acccel vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.grid()


    plt.figure(figsize=(10, 6))
    plt.plot(data_frame['timestamp_ms'], data_frame['fsm.state'], label='states', color='blue')
    plt.title('fsm vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('fsm (m)')
    plt.grid()

    plt.show()


# Now lets modify the data to prevent this risk 

# lets start with an alpha filter...

# og_y = deepcopy(data_frame['barometer.altitude'].to_numpy())
# y = deepcopy(data_frame['barometer.altitude'].to_numpy() )
# t = data_frame['timestamp_ms'].to_numpy()

# position_data = data_frame['kalman.position.px'].to_numpy()
# velocity_data = data_frame['kalman.velocity.vx'].to_numpy()
# acceleration_data = data_frame['kalman.acceleration.ax'].to_numpy()



# Apply alpha filter

# for i in range(1, len(y)):
#     alpha = 0.9
#     y[i] = alpha * y[i-1] + (1 - alpha) * y[i] #+ 500
#     print(f"Original: {og_y[i]}, Filtered: {y[i]}") 


# okay this wont work because the data is too large of a spike this works more so for single valued spikes, but not for large spikes  across large data sets.


# what about a moving average? like 10 point 

# for i in range(100, len(y)-100):
#     y[i] = np.mean(og_y[i-100:i]) #+ 500
#     print(f"Original: {og_y[i]}, Filtered: {y[i]}")


# yea this does not work either, because the data is too large of a spike this works more so for single valued spikes, but not for large spikes  across large data sets.


# maybe lets look at this from a statistical perspective....MLE? 

# def compute_mle(i, time_series, data_series, window_size = 100, threshold = 0.1):

#     # the last one is assumed to be the new data point 
#     t_mean = np.mean(time_series[i-window_size:i+1])
#     d_mean = np.mean(data_series[i-window_size:i+1])#[~np.isnan(data_series[i-window_size:i+1])]
#     print(t_mean, d_mean)

#     slope = np.sum((time_series[i-window_size:i+1] - t_mean) * (data_series[i-window_size:i+1] - d_mean)) / np.sum((time_series[i-window_size:i+1] - t_mean)**2)

#     intercept = d_mean - slope * t_mean

#     h_expected = slope * time_series[i] + intercept
#     residual = data_series[i] - h_expected

#     if abs(residual) > abs(d_mean) * threshold:
#         print("Spike detected, replacing with expected value")
#         return h_expected   
#     else:
#         print("No spike detected, keeping original value")
#         return data_series[i]

# what if we did a coarse short term propogation of the data based on acceleration and velocity, and then compared the expected value to the actual value, if the difference is too large, we can replace it with the expected value.

# for i in range(0, len(y)): 
#     acceleration = acceleration_data[i]
#     velocity = velocity_data[i]
#     if i == 0:
#         velocity = 0
#         expected_altitude = y[i]
#     else:
#         dt = (t[i] - t[i-1]) / 1000.0  # Convert milliseconds to seconds
#         velocity += acceleration * dt
#         expected_altitude = y[i-1] + velocity * dt
#     y[i] = expected_altitude




# for i in range(100, len(y)-100):
#     y[i] = compute_mle(i, t, y, window_size=10, threshold=0.1)
#     print(f"Original: {og_y[i]}, Filtered: {y[i]}")





# plt.figure(figsize=(10, 6))
# plt.plot(t, og_y, label='Original Altitude', color='blue')
# plt.plot(t, y, label='Filtered Altitude', color='orange')
# plt.legend()
# plt.show() 