#!/bin/bash
set -e

# Activate environment
cd ..
source /opt/ros/humble/setup.bash
source venv/bin/activate

cd workspace
source install/setup.bash

# Create new tmux session
session_name="rescue_aesir_$(date +%s)"
tmux new-session -d -s "$session_name"

# Split window vertically (top/bottom)
tmux split-window -v -p 50

# Pane 0 (top): run hardware node
tmux select-pane -t 0
tmux send-keys "ros2 run hardware dc_motors" Enter
sleep 5

# Pane 1 (bottom): launch teleoperation
tmux select-pane -t 1
tmux send-keys "ros2 launch teleoperation teleoperation.launch.py" Enter
sleep 5

# Optional key bindings to switch panes
tmux bind-key -n C-a select-pane -t :.+
tmux bind-key -n C-s select-pane -t :.-

# Attach to session
tmux -2 attach-session -t "$session_name"
