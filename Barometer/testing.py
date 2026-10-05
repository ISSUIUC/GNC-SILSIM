import pandas as pd 
import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
#data_frame = pd.read_csv('../data/Ashley_Data_MIDAS_MINI.csv') # nominal data
data_frame = pd.read_csv('./data/midas_booster_flight.csv')


plot_data_raw = False 

if plot_data_raw:
    plt.figure(figsize=(10, 6))
    plt.plot(data_frame['timestamp_ms'], data_frame['kalman.position.px'], label='Altitude', color='blue')
    plt.title('KF Altitude vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.grid()


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


    plt.show()


# Now lets modify the data to prevent this risk 

# lets start with an alpha filter...

og_y = deepcopy(data_frame['barometer.altitude'].to_numpy())
y = deepcopy(data_frame['barometer.altitude'].to_numpy() )
t = data_frame['timestamp_ms'].to_numpy()

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

def compute_mle(i, time_series, data_series, window_size = 100, threshold = 0.1):

    # the last one is assumed to be the new data point 
    t_mean = np.mean(time_series[i-window_size:i+1])
    d_mean = np.mean(data_series[i-window_size:i+1])#[~np.isnan(data_series[i-window_size:i+1])]
    print(t_mean, d_mean)

    slope = np.sum((time_series[i-window_size:i+1] - t_mean) * (data_series[i-window_size:i+1] - d_mean)) / np.sum((time_series[i-window_size:i+1] - t_mean)**2)

    intercept = d_mean - slope * t_mean

    h_expected = slope * time_series[i] + intercept
    residual = data_series[i] - h_expected

    if abs(residual) > abs(d_mean) * threshold:
        print("Spike detected, replacing with expected value")
        return h_expected   
    else:
        print("No spike detected, keeping original value")
        return data_series[i]


for i in range(100, len(y)-100):
    y[i] = compute_mle(i, t, y, window_size=10, threshold=0.1)
    print(f"Original: {og_y[i]}, Filtered: {y[i]}")





plt.figure(figsize=(10, 6))
plt.plot(t, og_y, label='Original Altitude', color='blue')
plt.plot(t, y, label='Filtered Altitude', color='orange')
plt.legend()
plt.show() 