#!/bin/bash

echo "Building EKF Simulator..."

if ! pkg-config --exists eigen3; then
    echo "Eigen3 not found. Please install Eigen3:"
    echo "  macOS: brew install eigen"
    echo "  Ubuntu: sudo apt-get install libeigen3-dev"
    echo "  Or download from: https://eigen.tuxfamily.org/"
    exit 1
fi

EIGEN_INCLUDE=$(pkg-config --cflags eigen3 | sed 's/-I//')

echo "Compiling with Eigen at: $EIGEN_INCLUDE"

g++ -std=c++17 -O2 -Wall -Wextra \
    -I"$EIGEN_INCLUDE" \
    -I. \
    -Ignc \
    gnc/test_ekf.cpp \
    gnc/ekf.cpp \
    -o gnc/test_ekf

if [ $? -eq 0 ]; then
    echo "Build successful! Run with: ./gnc/test_ekf"
else
    echo "Build failed!"
    exit 1
fi
