# NTO 2025-2026: Pipeline Inspection Project

## Overview
This project implements an autonomous drone mission for the NTO 2025-2026 Flying Robotics task. The drone operates in a simulated Gazebo environment to inspect a randomly generated pipeline system and identify specific "taps" on the pipeline. A web-based user interface allows for mission control and visualization of the drone's progress and findings.

## Features
*   **Dynamic World Generation**: Generates a unique pipeline layout with taps in Gazebo for each simulation run.
*   **Autonomous Mission Control**: Python-based logic to control the drone's takeoff, flight path (zig-zag scan), tap detection, and landing.
*   **Simulated Detection**: Detects taps based on the drone's proximity to ground truth locations, mimicking a visual inspection.
*   **Web-based User Interface**: Provides a live view of the drone's position and detected taps, with controls to start, stop, or kill the mission.
*   **ROS Integration**: Utilizes ROS (Robot Operating System) for inter-process communication between mission control, simulation, and UI.

## Getting Started

### Prerequisites
Before you begin, ensure you have the following installed:
*   **Docker**: For running the simulation environment.
*   **Python 3**: For running helper scripts and the main project logic.
*   **git**: For cloning the repository.

### Installation

1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/your-repo/Nto-pipes.git # Replace with actual repo URL
    cd Nto-pipes
    ```

2.  **Build the Docker Image**:
    Navigate to the `clever-show-ds` directory and build the custom Docker image. This image contains the necessary ROS and Clover setup.
    ```bash
    docker build -t clever-show-ds:custom clever-show-ds
    ```
    *(Note: If the base image `goldarte/clover-ds` is not found, Docker will automatically pull it.)*

## Running the Simulation

1.  **Start the Docker Environment and Gazebo**:
    This script launches the Gazebo simulation and the necessary Docker containers for the drone.
    Run this command on your **HOST machine**:
    ```bash
    python clever-show-ds/simulate.py
    ```
    This will start Gazebo and one or more drone simulation instances. Wait for Gazebo to load completely.

2.  **Execute the Mission Logic**:
    The mission control, world generation, and web server scripts are intended to run *inside* one of the Docker containers spawned by `simulate.py`.

    First, attach to the running container. By default, the container is named `sim-1` (if you are running only one drone).
    ```bash
    docker exec -it sim-1 bash
    ```
    Once inside the container, navigate to the project directory and run the `start.sh` script:
    ```bash
    cd /home/user/Nto-pipes
    ./start.sh
    ```
    This script will:
    *   Source ROS environment variables.
    *   Install Python dependencies from `requirements.txt`.
    *   Run `world-gen.py` to spawn the pipeline in Gazebo.
    *   Start `server.py` (the web interface).
    *   Start `mission.py` (the drone control logic).

3.  **Access the Web Interface**:
    Once the `server.py` is running inside the container, you can access the mission control web interface from your **HOST machine**'s web browser:
    ```
    http://localhost:5001
    ```
    (Note: The port `5001` is for `sim-1`. If you are running multiple drones or a different ID, adjust the port accordingly as per `clever-show-ds/run` script mapping: `5000 + ID`).

    Click the "START" button in the web interface to initiate the drone's mission.

## Project Structure
*   `access_instructions.md`: Instructions for accessing the web interface (now mostly covered in README).
*   `mission.py`: Contains the drone's autonomous mission logic, including takeoff, scanning, and tap detection.
*   `requirements.txt`: Python dependencies for the project.
*   `server.py`: The Flask web server for mission control and visualization.
*   `start.sh`: Shell script to initialize the environment and run project components inside the Docker container.
*   `task.md`: Detailed description of the mission objectives and technical specifications.
*   `world-gen.py`: Generates the Gazebo world (pipeline and taps) and publishes tap ground truth to ROS parameters.
*   `clever-show-ds/`: Contains Dockerfile and scripts for the simulation environment setup.

For a detailed description of the task, please refer to `task.md`.