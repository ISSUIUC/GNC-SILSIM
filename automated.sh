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

# Check if build.sh exists and use it, otherwise compile directly
if [ -f "mac_files/build.sh" ]; then
    ./mac_files/build.sh
    if [ $? -ne 0 ]; then
        echo "[ERROR] Build failed. Exiting."
        exit 1
    fi
else
    echo "[ERROR] mac_files/build.sh not found. Exiting."
    exit 1
fi

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

if [ -f "./gnc/test_ekf" ]; then
    ./simulation/test_ekf "$DATA_FILE" "$OUTPUT_FILE"
    if [ $? -ne 0 ]; then
        echo "[ERROR] Failed to run EKF executable."
        exit 1
    fi
else
    echo "[ERROR] gnc/test_ekf executable not found!"
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

