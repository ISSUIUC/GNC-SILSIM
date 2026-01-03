@echo off
REM ============================================================
REM  EKF Automation Script for Windows Machines
REM  Compiles, runs, and plots EKF results automatically
REM  Required to run from the root of the GNC-SILSIM
REM ============================================================

echo ------------------------------------------------------------
echo Step 1: Navigate to the GNC folder
echo ------------------------------------------------------------
cd gnc
if errorlevel 1 (
    echo [ERROR] Could not change to 'gnc' directory.
    pause
    exit /b
)

echo ------------------------------------------------------------
echo Step 2: Compile C++ files
echo ------------------------------------------------------------
g++ -I. -o main ./test_ekf.cpp ./ekf.cpp
if errorlevel 1 (
    echo [WARNING] Compilation failed. Trying with C++17 flag...
    g++ -std=c++17 -I. -o main ./test_ekf.cpp ./ekf.cpp
    if errorlevel 1 (
        echo [ERROR] Compilation failed again. Exiting.
        pause
        exit /b
    )
)

echo ------------------------------------------------------------
echo Step 3: Run EKF on selected data file
echo ------------------------------------------------------------
cd ..
set "DATA_FILE=./data/Aether_MIDAS_booster.csv"
set "OUTPUT_FILE=./output/results.csv"

echo Using data file: %DATA_FILE%
echo Output will be saved to: %OUTPUT_FILE%

gnc\main "%DATA_FILE%" "%OUTPUT_FILE%"
if errorlevel 1 (
    echo [ERROR] Failed to run EKF executable.
    pause
    exit /b
)

echo ------------------------------------------------------------
echo Step 4: Generate plots
echo ------------------------------------------------------------
python ./plotter/plot_results.py
if errorlevel 1 (
    echo [ERROR] Failed to generate plots. Check Python installation or script.
    pause
    exit /b
)

echo ------------------------------------------------------------
echo All steps completed successfully.
echo ------------------------------------------------------------
pause
