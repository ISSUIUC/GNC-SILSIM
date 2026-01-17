set -e  # Exit on error

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

INPUT_FILE="data/simulated_6dof.csv"
OUTPUT_FILE="output/results.csv"
PLOT_RESULTS=true
STOP_STATE="STATE_LANDED"

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
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -i, --input FILE     Input CSV file (default: data/MIDAS Trimmed (AL2, CSV).csv)"
            echo "  -o, --output FILE    Output CSV file (default: output/results.csv)"
            echo "  -s, --stop-state     FSM state to stop simulation at (default: STATE_LANDED)"
            echo "  --no-plot           Skip plotting results"
            echo "  -h, --help          Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Use defaults"
            echo "  $0 -i data/sample_1k.csv -o output/results.csv   # Specify files"
            echo "  $0 -s STATE_COAST                    # Stop at coast phase"
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
echo -e "${BLUE}    EKF Automation Script               ${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Create data and output directories if they don't exist
mkdir -p data output

if [ ! -f "$INPUT_FILE" ]; then
    echo -e "${RED}Error: Input file '$INPUT_FILE' not found!${NC}"
    echo "Available CSV files in data/ directory:"
    ls -la data/*.csv 2>/dev/null || echo "No CSV files found in data/ directory"
    exit 1
fi

echo -e "${YELLOW}Configuration:${NC}"
echo "  Input file:  $INPUT_FILE"
echo "  Output file: $OUTPUT_FILE"
echo "  Stop state:  $STOP_STATE"
echo "  Plot results: $PLOT_RESULTS"
echo ""

echo -e "${YELLOW}Step 1: Building code...${NC}"

# Check for Eigen3
if ! pkg-config --exists eigen3; then
    echo -e "${RED}[ERROR] Eigen3 not found. Please install Eigen3:${NC}"
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

if [ $? -ne 0 ]; then
    echo -e "${RED}[ERROR] Build failed. Exiting.${NC}"
    exit 1
fi

echo -e "${GREEN}Build successful!${NC}"

echo ""
echo -e "${YELLOW}Step 2: Running KF simulation...${NC}"

if [ -f "./gnc/test_ekf" ]; then
    ./gnc/test_ekf "$INPUT_FILE" "$OUTPUT_FILE" "$STOP_STATE"
    if [ $? -ne 0 ]; then
        echo -e "${RED}[ERROR] Simulation failed!${NC}"
        exit 1
    fi
    echo -e "${GREEN}Simulation completed successfully!${NC}"
    echo "Results saved to: $OUTPUT_FILE"
else
    echo -e "${RED}[ERROR] gnc/test_ekf executable not found!${NC}"
    echo "Make sure the build completed successfully."
    exit 1
fi

echo ""

if [ "$PLOT_RESULTS" = true ]; then
    echo -e "${YELLOW}Step 3: Plotting results...${NC}"
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
echo -e "${GREEN}Done! 🚀${NC}"

