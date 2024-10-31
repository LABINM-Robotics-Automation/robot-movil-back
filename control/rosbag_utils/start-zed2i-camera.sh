#!/bin/bash
# Run the ROS launch command
roslaunch blueberry-detection-ros start-zed2i-camera.launch

# Check if roslaunch command was successful
if [ $? -eq 0 ]; then
    echo "Zed2i camera started successfully"
    exit 0  # Success exit code
else
    echo "Failed to start Zed2i camera"
    exit 1  # Failure exit code
fi

