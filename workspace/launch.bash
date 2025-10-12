#!/bin/bash
set -e

# Activate environment
cd ..
source /opt/ros/humble/setup.bash
source venv/bin/activate

#sudo chmod 666 /dev/ttyUSB0

cd workspace
source install/setup.bash

# Create new tmux session
session_name="rescue_aesir_$(date +%s)"
tmux new-session -d -s "$session_name"

# Split the window vertically into top (pane 0) and bottom (pane 1)
tmux select-pane -t 0
tmux split-window -v -p 50

# Top row: split pane 0 horizontally into two (pane 0 and pane 2)
tmux select-pane -t 0
tmux split-window -h -p 50

# Top row: split the right top pane (pane 2) horizontally into two (pane 2 and pane 3)
tmux select-pane -t 2
tmux split-window -h -p 50

# Pane 0 (top): run hardware node
tmux select-pane -t 0
tmux send-keys "ros2 run teleoperation command_server.py" Enter
sleep 2

# Pane 1 (bottom-left): run teleoperation command_server.py
tmux select-pane -t 1
tmux send-keys "ros2 run teleoperation server.py" Enter
sleep 2

# Pane 2 (bottom-right): run teleoperation server.py
tmux select-pane -t 2
tmux send-keys "ros2 run hardware dc_motors" Enter
sleep 2

# Pane 2 (bottom-right): run teleoperation server.py
tmux select-pane -t 3
tmux send-keys "ros2 run vision video_stream_publisher" Enter
sleep 2

# Optional key bindings to switch panes
tmux bind-key -n C-a select-pane -t :.+
tmux bind-key -n C-s select-pane -t :.-

# Attach to session
tmux -2 attach-session -t "$session_name"
