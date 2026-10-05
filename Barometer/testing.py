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

for i in range(1, len(y)):
    alpha = 0.9
    y[i] = alpha * y[i-1] + (1 - alpha) * y[i] #+ 500
    print(f"Original: {og_y[i]}, Filtered: {y[i]}") 

# okay this wont work because the data is too large of a spike this works more so for single valued spikes, but not for large spikes  across large data sets.


plt.figure(figsize=(10, 6))
plt.plot(t, og_y, label='Original Altitude', color='blue')
plt.plot(t, y, label='Filtered Altitude', color='orange')
plt.show() 