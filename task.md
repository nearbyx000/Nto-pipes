# NTO 2025-2026: Pipeline Inspection Task

## Overview
The goal of this task is to program an autonomous drone to inspect a pipeline system in a simulated environment. The pipeline consists of yellow tubes connected on the ground. Some sections of the pipeline have "taps" (red cylinders) that need to be identified and reported.

## Mission Objectives
1.  **Takeoff**: The drone must take off safely to an altitude of 2 meters.
2.  **Scan**: The drone must fly over the inspection area (covering coordinates from X: 0-12m, Y: 0-12m) to locate the pipeline elements.
3.  **Detect**: The drone must detect the location of red "taps" on the pipeline.
    *   *Note*: For this simulation stage, detection is simulated based on the drone's proximity to the ground truth locations of the taps. A tap is considered "detected" if the drone flies within **1.0 meter** of its location (XY plane).
4.  **Report**: The detected tap locations must be reported in real-time to the Mission Control Center (Web Interface) via the ROS topic `/tubes`.
5.  **Return**: After scanning the area, the drone must return to the home position (0, 0) and land.

## System Architecture
*   **Simulator**: Gazebo with PX4/Clover (Dockerized).
*   **Mission Control**: A Python script (`mission.py`) using `rospy` and `clover` services to control the drone.
*   **Web Interface**: A Flask server (`server.py`) that displays the drone's position and detected taps.
*   **World Generation**: A script (`world-gen.py`) that generates a random pipeline configuration for each run.

## Interfaces
*   **Command Topic**: `/mission_cmd` (String) - Receives 'start', 'stop', 'kill' commands from the Web UI.
*   **Data Topic**: `/tubes` (String/JSON) - Publishes the drone's current position and list of found taps.
    *   Format: `{"drone": [x, y], "taps": [[tx1, ty1], [tx2, ty2], ...]}`

## Execution
To run the solution:
1.  Start the Docker simulation environment (see `README.md`).
2.  Run the initialization script: `./start.sh`
3.  Open the Web UI at `http://localhost:5001`.
4.  Click "START" to begin the autonomous mission.
