COMMANDS:

g++ -I. -o main test_ekf.cpp ekf.cpp

./main "./data/MIDAS_booster.cs" "./output/results.csv"

python3 ./plotter/plot_results.py