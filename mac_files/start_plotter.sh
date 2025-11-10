#!/bin/bash

echo "🚀 Starting EKF Interactive Plotter..."
echo ""

# Check if results.csv exists
if [ ! -f "output/results.csv" ]; then
    echo "❌ Error: output/results.csv not found!"
    echo "Please run the EKF simulation first:"
    echo "  ./run_simulation.sh"
    echo ""
    exit 1
fi

echo "✅ Found results.csv with $(wc -l < output/results.csv) lines"
echo ""

# Start the interactive plotter
cd interactive_plotter
python3 server.py
