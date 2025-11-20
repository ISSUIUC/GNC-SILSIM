cd gnc
g++ -I. -o main test_ekf.cpp ekf.cpp Madgwick/Madgwick.cpp
cd ..
./gnc/main "./data/MIDAS Trimmed (AL0, CSV).csv" "./output/results.csv"
