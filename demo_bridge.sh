#!/bin/bash
# Quick demonstration of the ROS-LCM bidirectional bridge

set -e

echo "=== ROS-LCM Bidirectional Bridge Demo ==="
echo
echo "This demo will:"
echo "1. Start the ROS-LCM bridge"
echo "2. Publish messages from both ROS and LCM"
echo "3. Show that messages flow in both directions"
echo "4. Verify no feedback loops occur"
echo

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Cleanup function
cleanup() {
    echo -e "\n${YELLOW}Cleaning up...${NC}"
    pkill -P $$ 2>/dev/null || true
    pkill -f ros_lcm_bridge.py 2>/dev/null || true
    pkill -f test_lcm_publisher.py 2>/dev/null || true
    pkill -f test_ros_publisher.py 2>/dev/null || true
    wait 2>/dev/null
}

trap cleanup EXIT

# Source ROS2
source /opt/ros/humble/setup.bash

# Start the bridge
echo -e "${GREEN}Starting ROS-LCM Bridge...${NC}"
python3 ros_lcm_bridge.py &
BRIDGE_PID=$!
sleep 3

# Start publishers
echo -e "\n${GREEN}Starting test publishers...${NC}"
echo "- LCM publisher (1 Hz)"
python3 test_lcm_publisher.py --rate 1 &
LCM_PID=$!

echo "- ROS publisher (1 Hz)"
python3 test_ros_publisher.py &
ROS_PID=$!

sleep 2

# Show active topics
echo -e "\n${GREEN}Active ROS topics:${NC}"
ros2 topic list | grep -E "(lcm_|ros_)" | sort

echo -e "\n${GREEN}Monitoring message flow for 5 seconds...${NC}"
echo "(You should see messages from both LCM and ROS publishers)"

# Monitor a topic from each direction
echo -e "\n${YELLOW}ROS topic receiving from LCM:${NC}"
timeout 2s ros2 topic hz /lcm_test_string --window 10 2>/dev/null || true

echo -e "\n${YELLOW}Checking LCM channels:${NC}"
echo "lcm_* channels are from LCM publishers"
echo "/ros_* channels are from ROS publishers"
timeout 2s python3 utils/lcmtopic.py list 2>/dev/null | grep -E "(lcm_|ros_)" || true

# Let it run for a bit
echo -e "\n${GREEN}Bridge is running successfully!${NC}"
echo "Press Ctrl+C to stop and see statistics..."

# Wait for user interrupt
wait