#!/bin/bash

# Set your project directory
PROJECT_DIR="/home/labinm-jetson/xavier_ssd/robot-movil-back"

# Set the name for the tmux session
SESSION_NAME="robot_movil_backend_server"

# Change to the project directory
cd "$PROJECT_DIR" || exit

# Activate the virtual environment
source /home/labinm-jetson/xavier_ssd/robot-movil-back/robot_movil_back/bin/activate 

# Start or attach to the tmux session and run the Django server
tmux new-session -d -s "$SESSION_NAME" "python manage.py runserver 0.0.0.0:8000"

echo "Django server started in tmux session: $SESSION_NAME"
