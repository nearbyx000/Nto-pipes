#!/bin/bash

# Source ROS setup
source /opt/ros/melodic/setup.bash
source /home/user/catkin_ws/devel/setup.bash

# Set Gazebo Paths
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/user/Nto-pipes/clever-show-ds/gazebo/plugins
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/user/Nto-pipes/clever-show-ds/gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/user/Nto-pipes/clever-show-ds/gazebo/plugins
export PX4_HOME_LAT=55.7031751
export PX4_HOME_LON=37.7248118

echo ">>> LAUNCHING GAZEBO SIMULATION <<<"

# 1. Launch Gazebo (headless or gui handled by Xvfb/VNC)
roslaunch /home/user/Nto-pipes/clever-show-ds/gazebo/gazebo.launch &
GAZEBO_PID=$!
sleep 15 # Give Gazebo time to initialize

# 2. Spawn Drone Model
echo ">>> Spawning Drone..."
roslaunch /home/user/Nto-pipes/clever-show-ds/gazebo/single_vehicle_spawn.launch ID:=1 port:=14560 x:=0 y:=0 &
SPAWN_PID=$!
sleep 10

# 3. Start Application Logic (World Gen, Server, Mission)
echo ">>> Starting Application Stack..."
./start.sh

# Cleanup trap
trap "kill $GAZEBO_PID $SPAWN_PID; exit" SIGINT SIGTERM
wait