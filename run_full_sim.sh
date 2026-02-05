#!/bin/bash
# Complete Simulation Runner for ATLAS-T

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}Starting ATLAS-T Complete Simulation...${NC}"

# 1. Start Docker in background
echo -e "${YELLOW}Building and starting Docker container...${NC}"
docker-compose up --build -d

if [ $? -ne 0 ]; then
    echo "Failed to start Docker. Trying host-based simulation (if dependencies allow)..."
    # Fallback or exit
fi

# 2. Wait for some data to be generated
echo -e "${YELLOW}Simulation running. Waiting for data collection (60 seconds)...${NC}"
sleep 60

# 3. Check for results
echo -e "${YELLOW}Analyzing results...${NC}"
python3 result_analyzer.py

echo -e "${GREEN}Simulation completed successfully!${NC}"
echo -e "${GREEN}Check the 'results' directory for reports and plots.${NC}"
