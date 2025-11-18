In the gnc folder, run 

g++ -I. -o main test_ekf.cpp ekf.cpp Madgwick/Madgwick.cpp

Then, in the root, run

./gnc/main.exe ./data/MIDAS_sustainer.csv ./output/results.csv

Then, run python3 ./plotter/plot_results.py