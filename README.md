# Overview
The program is the Illinois Space Society's Guidance, Navigation, and Contol Software-in-the-Loop simulation. Through this program, the GNC team is able to test its in-flight implementation of the Extended Kalman Filter. The EKF is stored in the GNC folder and is implemented in C++. 

## Getting Started
To start, make sure you have a C++ compiler installed with a minimum version of 17. Otherwise, clamp will not work. Make sure you have all the requirements installed as well. Create a data folder in root and save the data files there, Ex: "./data/MIDAS_Booster.csv". Then, create an output folder in root.

## Commands (in order)
From the root of the folder, to compile the C++ files, run
`cd gnc`
`g++ -I. -o main ./test_ekf.cpp ./ekf.cpp`

If there's a clamp error, run

`g++ -std=c++17 -I. -o main ./test_ekf.cpp ./ekf.cpp`

Then, to run the EKF file on a particular data file, run 

`cd ..`

`./gnc/main "./data/MIDAS_booster.csv" "./output/results.csv"`

The data file can be changed to any data file.

Finally, to create the graphs, run 

`python3 ./plotter/plot_results.py`
