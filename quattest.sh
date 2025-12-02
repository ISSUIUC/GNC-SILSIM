cd gnc
# windows -> msys mingw64
g++ -I. -o main test_ekf.cpp ekf.cpp Madgwick/Madgwick.cpp

#mac
g++ -std=c++17 -I. -o main test_ekf.cpp ekf.cpp Madgwick/Madgwick.cpp
cd ..
./gnc/main "./data/MIDAS Trimmed (AL0, CSV).csv" "./output/results.csv"
