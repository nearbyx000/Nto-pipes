roslaunch clover_simulation main_camera.launch &
PID_SIM=$!
sleep 15

roslaunch aruco_pose aruco.launch &
PID_ARUCO=$!
sleep 10

./world-gen.py


./server.py &
PID_WEB=$!

./mission.py &
PID_MISSION=$!

trap "kill $PID_SIM $PID_ARUCO $PID_WEB $PID_MISSION; exit" SIGINT
wait
