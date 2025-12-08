# Instructions to Access Web Interface

To access the Flask web interface running inside your Docker container, you need to ensure the port is correctly mapped from the container to your host machine. The `clever-show-ds/run` script has been updated to handle this mapping.

### Required Actions:

1.  **Stop the current container (if running):**
    *   Run this command on your **HOST machine** (not inside the Docker container):
        ```bash
        docker kill sim-1 # Replace 'sim-1' with your container name if it's different
        ```

2.  **Start the simulation again, using the updated `run` script:**
    *   Run this command on your **HOST machine**:
        ```bash
        python clever-show-ds/simulate.py # Or whatever command you use to start your simulation
        ```
    *   This will use the `clever-show-ds/run` script, which now includes the port mapping.

3.  **Access the Web Interface:**
    *   Once the new container is running, open your web browser and navigate to: **[http://localhost:5001](http://localhost:5001)**
    *   *(Note: This URL assumes the default container ID (sim-1) is used. The port is mapped as `5000 + ID`. If your container has a different ID, adjust the host port accordingly.)*

4.  **Inside the Container (if needed):**
    *   If you need to interact with the container's shell, attach to it:
        ```bash
        docker exec -it sim-1 bash # Replace 'sim-1' with your container name if it's different
        ```
    *   Navigate to your project directory and run your `start.sh` script:
        ```bash
        cd /home/user/Nto-pipes # Adjust path if necessary
        ./start.sh
        ```
