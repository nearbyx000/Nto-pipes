#!/bin/bash

# Exit on error
set -e

# Load ROS Environment
source /opt/ros/melodic/setup.bash
source /home/user/catkin_ws/devel/setup.bash

echo ">>> STARTING PIPELINE INSPECTION SYSTEM <<<"

# 1. Start core ROS services (Simulated Drone)
# Note: 'launch_sim_inside.sh' handles Gazebo + Drone spawn.
# We assume the simulator is ALREADY RUNNING via that script.
# If not, this script will fail to find ROS master.

# 2. Generate World (Pipeline + Taps)
echo ">>> Generating World and Setting Params..."
python ./world-gen.py

# 3. Start Mission Control Node
echo ">>> Starting Mission Control..."
python ./mission.py &
MISSION_PID=$!

# 4. Start Web Interface
echo ">>> Starting Web Interface..."
python ./server.py &
SERVER_PID=$!

echo ">>> SYSTEM READY. Access UI at http://localhost:5001 <<<"

# Cleanup
trap "kill $MISSION_PID $SERVER_PID; exit" SIGINT SIGTERM
wait