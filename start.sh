
source /opt/ros/melodic/setup.bash
source /home/user/catkin_ws/devel/setup.bash

chmod +x world-gen.py mission.py server.py

roslaunch clover main_camera.launch &
P1=$!
sleep 15

roslaunch clover aruco.launch &
P2=$!
sleep 5

./world-gen.py
./server.py &
P3=$!
./mission.py &
P4=$!

trap "kill $P1 $P2 $P3 $P4; exit" SIGINT
wait