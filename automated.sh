#!/bin/bash

# ============================================================
#  EKF Automation Script for Mac/Linux Machines
#  Compiles, runs, and plots EKF results automatically
#  Required to run from the root of the GNC-SILSIM
# ============================================================

set -e  # Exit on error

echo "------------------------------------------------------------"
echo "Step 1: Building code..."
echo "------------------------------------------------------------"

# Check for Eigen3
if ! pkg-config --exists eigen3; then
    echo "[ERROR] Eigen3 not found. Please install Eigen3:"
    echo "  macOS: brew install eigen"
    echo "  Ubuntu: sudo apt-get install libeigen3-dev"
    echo "  Or download from: https://eigen.tuxfamily.org/"
    exit 1
fi

EIGEN_INCLUDE=$(pkg-config --cflags eigen3 | sed 's/-I//')

echo "Compiling with Eigen at: $EIGEN_INCLUDE"

# Create simulation directory if it doesn't exist
mkdir -p simulation

g++ -std=c++17 -O2 -Wall -Wextra \
    -I"$EIGEN_INCLUDE" \
    -I. \
    -Ignc \
    gnc/test_ekf.cpp \
    gnc/ekf.cpp \
    -o simulation/test_ekf

if [ $? -ne 0 ]; then
    echo "[ERROR] Build failed. Exiting."
    exit 1
fi

echo "Build successful!"

echo ""
echo "------------------------------------------------------------"
echo "Step 2: Run EKF on selected data file"
echo "------------------------------------------------------------"

DATA_FILE="./data/MIDAS Trimmed (AL2, CSV).csv"
OUTPUT_FILE="./output/results.csv"

# Create output directory if it doesn't exist
mkdir -p output

echo "Using data file: $DATA_FILE"
echo "Output will be saved to: $OUTPUT_FILE"

if [ ! -f "$DATA_FILE" ]; then
    echo "[ERROR] Data file '$DATA_FILE' not found!"
    exit 1
fi

if [ -f "./simulation/test_ekf" ]; then
    ./simulation/test_ekf "$DATA_FILE" "$OUTPUT_FILE"
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to run EKF executable."
        exit 1
    fi
else
    echo "[ERROR] simulation/test_ekf executable not found!"
    exit 1
fi

echo ""
echo "------------------------------------------------------------"
echo "Step 3: Generate plots"
echo "------------------------------------------------------------"

if [ -f "./plotter/plot_results.py" ]; then
    python3 ./plotter/plot_results.py "$OUTPUT_FILE"
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to generate plots. Check Python installation or script."
        exit 1
    fi
else
    echo "[ERROR] plotter/plot_results.py not found!"
    exit 1
fi

echo ""
echo "------------------------------------------------------------"
echo "All steps completed successfully."
echo "------------------------------------------------------------"

