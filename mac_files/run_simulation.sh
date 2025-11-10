#!/bin/bash

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

INPUT_FILE="data/MIDAS Trimmed (AL2, CSV).csv"
OUTPUT_FILE="output/results.csv"
PLOT_RESULTS=true
STOP_STATE="STATE_LANDED"
INTERACTIVE_PLOT=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -i|--input)
            INPUT_FILE="$2"
            shift 2
            ;;
        -o|--output)
            OUTPUT_FILE="$2"
            shift 2
            ;;
        --no-plot)
            PLOT_RESULTS=false
            shift
            ;;
        -s|--stop-state)
            STOP_STATE="$2"
            shift 2
            ;;
        --interactive)
            INTERACTIVE_PLOT=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -i, --input FILE     Input CSV file (default: MIDAS Sustainer (Trimmed CSV).csv)"
            echo "  -o, --output FILE    Output CSV file (default: results.csv)"
            echo "  -s, --stop-state     FSM state to stop simulation at (default: STATE_LANDED)"
            echo "  --interactive        Use interactive plotting (zoom/pan/select)"
            echo "  --no-plot           Skip plotting results"
            echo "  -h, --help          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Use defaults"
            echo "  $0 -i data/sample_1k.csv -o output/results.csv   # Specify files"
            echo "  $0 -s STATE_COAST                    # Stop at coast phase"
            echo "  $0 --interactive                      # Interactive zoom/pan plots"
            echo "  $0 --no-plot                         # Skip plotting"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    EKF Flight Data Simulator Runner    ${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Create data and output directories if they don't exist
mkdir -p data output

if [ ! -f "$INPUT_FILE" ]; then
    echo -e "${RED}Error: Input file '$INPUT_FILE' not found!${NC}"
    echo "Available CSV files in data/ directory:"
    ls -la data/*.csv 2>/dev/null || echo "No CSV files found in data/ directory"
    echo ""
    echo "Please copy your CSV files to the data/ directory:"
    echo "  mkdir -p data output"
    echo "  cp your_flight_data.csv data/"
    exit 1
fi

echo -e "${YELLOW}Configuration:${NC}"
echo "  Input file:  $INPUT_FILE"
echo "  Output file: $OUTPUT_FILE"
echo "  Stop state:  $STOP_STATE"
echo "  Plot results: $PLOT_RESULTS"
echo "  Interactive: $INTERACTIVE_PLOT"
echo ""

echo -e "${YELLOW}Step 1: Building code...${NC}"
if [ -f "build.sh" ]; then
    ./build.sh
    if [ $? -ne 0 ]; then
        echo -e "${RED}Build failed!${NC}"
        exit 1
    fi
    echo -e "${GREEN}Build completed successfully!${NC}"
else
    echo -e "${RED}Error: build.sh not found!${NC}"
    exit 1
fi
echo ""

echo -e "${YELLOW}Running KF simulation...${NC}"
if [ -f "./simulation/test_ekf" ]; then
    ./simulation/test_ekf "$INPUT_FILE" "$OUTPUT_FILE" "$STOP_STATE"
    if [ $? -ne 0 ]; then
        echo -e "${RED}Simulation failed!${NC}"
        exit 1
    fi
    echo -e "${GREEN}Simulation completed successfully!${NC}"
    echo "Results saved to: $OUTPUT_FILE"
else
    echo -e "${RED}Error: simulation/test_ekf executable not found!${NC}"
    echo "Make sure the build completed successfully."
    exit 1
fi
echo ""

if [ "$PLOT_RESULTS" = true ]; then
    echo -e "${YELLOW}Plotting results...${NC}"
    if [ "$INTERACTIVE_PLOT" = true ]; then
        if [ -f "plotter/plot_interactive.py" ]; then
            echo -e "${BLUE}Launching interactive plot...${NC}"
            python3 plotter/plot_interactive.py "$OUTPUT_FILE"
            if [ $? -ne 0 ]; then
                echo -e "${RED}Interactive plotting failed!${NC}"
                echo "You can still view the results in: $OUTPUT_FILE"
            else
                echo -e "${GREEN}Interactive plot launched successfully!${NC}"
            fi
        else
            echo -e "${RED}Error: plotter/plot_interactive.py not found!${NC}"
            echo "Falling back to standard plots..."
            python3 plotter/plot_results.py "$OUTPUT_FILE"
        fi
    else
        if [ -f "plotter/plot_results.py" ]; then
            python3 plotter/plot_results.py "$OUTPUT_FILE"
            if [ $? -ne 0 ]; then
                echo -e "${RED}Plotting failed!${NC}"
                echo "You can still view the results in: $OUTPUT_FILE"
            else
                echo -e "${GREEN}Plots generated successfully!${NC}"
            fi
        else
            echo -e "${RED}Error: plotter/plot_results.py not found!${NC}"
            echo "Results are available in: $OUTPUT_FILE"
        fi
    fi
else
    echo -e "${YELLOW}Step 3: Skipping plots (--no-plot specified)${NC}"
fi
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}           SIMULATION COMPLETE           ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${BLUE}Summary:${NC}"
echo "  ✓ Project built successfully"
echo "  ✓ Simulation completed"
echo "  ✓ Results saved to: $OUTPUT_FILE"

if [ "$PLOT_RESULTS" = true ]; then
    echo "  ✓ Plots generated"
fi

echo ""
echo -e "${BLUE}Next steps:${NC}"
echo "  • View results: cat $OUTPUT_FILE"
echo "  • Plot results: python3 plot_results.py $OUTPUT_FILE"
echo "  • Run with different input: $0 -i your_file.csv -o your_results.csv"
echo ""
echo -e "${GREEN}Done! 🚀${NC}"
