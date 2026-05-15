#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== VisionBot Container Entrypoint ===${NC}"

# Source ROS 2 Jazzy FIRST
echo -e "${YELLOW}Sourcing ROS 2 Jazzy...${NC}"
source /opt/ros/jazzy/setup.bash

# Navigate to workspace
cd ~/ros2_ws

# Build the workspace
echo -e "${YELLOW}Building VisionBot project...${NC}"
colcon build --symlink-install

# Source the workspace
if [ -f "install/setup.bash" ]; then
    echo -e "${YELLOW}Sourcing workspace...${NC}"
    source install/setup.bash
    echo -e "${GREEN}Workspace sourced!${NC}"
fi

# Setup .bashrc for ros2 development
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi
if ! grep -q "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi" >> ~/.bashrc
fi


# Set up environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/vision_bot/description

echo -e "${GREEN}=== Environment Info ===${NC}"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "Workspace: $(pwd)"
echo ""

# Helpful aliases and functions
echo -e "${GREEN}=== Useful Commands ===${NC}"
echo "• colcon build --symlink-install"
echo "• ros2 launch vision_bot sim.launch.py"
echo "• ros2 topic list"
echo "• ros2 node list"
echo "• ros2 control list_controllers"
echo ""

# Execute the command passed to docker run, or start bash
if [ $# -eq 0 ]; then
    echo -e "${GREEN}Starting interactive shell...${NC}"
    exec /bin/bash
else
    echo -e "${GREEN}Executing: $@${NC}"
    exec "$@"
fi
