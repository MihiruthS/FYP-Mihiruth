#!/bin/bash
#
# Start Robot Voice Pipeline with ROS2 Integration
#
# This script:
# 1. Sources the Python virtual environment
# 2. Sources the ROS2 workspace for custom_interfaces
# 3. Starts the robot voice pipeline with camera integration
#

set -e  # Exit on error

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}🤖 Starting Robot Voice Pipeline with ROS2 Integration${NC}\n"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 1. Source Python virtual environment
echo -e "${GREEN}📦 Activating Python virtual environment...${NC}"
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
    echo -e "${GREEN}✅ Virtual environment activated${NC}\n"
else
    echo -e "${RED}❌ Virtual environment not found at venv/bin/activate${NC}"
    echo -e "${YELLOW}Please create it with: python -m venv venv${NC}"
    exit 1
fi

# 2. Source ROS2 workspace
ROS2_WORKSPACE="/home/quanta/Desktop/Head-Code-Old/receptionist-3.0-head/install/setup.bash"
echo -e "${GREEN}📦 Sourcing ROS2 workspace for custom_interfaces...${NC}"
if [ -f "$ROS2_WORKSPACE" ]; then
    source "$ROS2_WORKSPACE"
    echo -e "${GREEN}✅ ROS2 workspace sourced${NC}\n"
else
    echo -e "${YELLOW}⚠️  ROS2 workspace not found at $ROS2_WORKSPACE${NC}"
    echo -e "${YELLOW}⚠️  Camera user detection may not work without custom_interfaces${NC}\n"
fi

# 3. Check if camera node is running
echo -e "${GREEN}📹 Checking for camera feed...${NC}"
if ros2 topic list 2>/dev/null | grep -q "/active_users"; then
    echo -e "${GREEN}✅ Camera feed detected on /active_users topic${NC}\n"
else
    echo -e "${YELLOW}⚠️  Camera feed not detected on /active_users topic${NC}"
    echo -e "${YELLOW}⚠️  Make sure camera node is running for user detection${NC}\n"
fi

# 4. Start the robot system
echo -e "${GREEN}🚀 Starting robot voice pipeline...${NC}\n"
echo "======================================================================="
python src/robot_pipeline/ros2_main.py
