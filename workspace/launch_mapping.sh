#!/bin/bash

SOURCE_LOCAL_DIR="install/setup.bash"
SOURCE_VENV_DIR="../.venv/bin/activate"
# Source global ROS version helper (workspace/launch.bash lives in workspace/)
# Compute script dir robustly (works when run with bash or sh)
if [ -n "${BASH_SOURCE[0]:-}" ]; then
	SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
	SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

echo "launch.bash: ROS_UBUNTU_VERSION='${ROS_UBUNTU_VERSION:-}'"

if [ -n "${ROS_UBUNTU_VERSION:-}" ]; then
	if [ -f "/opt/ros/${ROS_UBUNTU_VERSION}/setup.bash" ]; then
		SOURCE_ROS_DIR="/opt/ros/${ROS_UBUNTU_VERSION}/setup.bash"
	else
		echo "Warning: /opt/ros/${ROS_UBUNTU_VERSION}/setup.bash not found."
		SOURCE_ROS_DIR=""
	fi
elif [ -f "/opt/ros/humble/setup.bash" ]; then
	SOURCE_ROS_DIR="/opt/ros/humble/setup.bash"
else
	SOURCE_ROS_DIR="/opt/ros/jazzy/setup.bash"
fi

#sudo chmod 666 /dev/ttyUSB0

echo "launch.bash: using ROS setup: $SOURCE_ROS_DIR"
echo "launch.bash: using venv:      $SOURCE_VENV_DIR"
echo "launch.bash: using overlay:   $SOURCE_LOCAL_DIR"

# Verify key files exist
[[ -f "$SOURCE_ROS_DIR" ]] || { echo "Missing: $SOURCE_ROS_DIR" >&2; exit 1; }
[[ -f "$SOURCE_LOCAL_DIR" ]] || { echo "Missing: $SOURCE_LOCAL_DIR (did you colcon build?)" >&2; exit 1; }
[[ -f "$SOURCE_VENV_DIR" ]] || { echo "Missing: $SOURCE_VENV_DIR (did you create the venv?)" >&2; exit 1; }

ENV_CMD="bash -lc ' \
  source \"$SOURCE_VENV_DIR\" && \
  source \"$SOURCE_ROS_DIR\" && \
  source \"$SOURCE_LOCAL_DIR\" && \
  exec bash'"

session_name="rescue_aesir_$(date +%s)"

# Crear sesión con default-command
tmux new-session -d -s "$session_name" -n main "$ENV_CMD"
tmux set-option -t "$session_name" default-command "$ENV_CMD"

# Splits
# Top row: split pane 0 horizontally into two (pane 0 and pane 1)
tmux select-pane -t 0
tmux split-window -h -p 50

# Top row: split the right top pane (pane 1) horizontally into two (pane 1 and pane 2)
tmux select-pane -t 1
tmux split-window -h -p 50

# Top row: split the right top pane (pane 2) horizontally into two (pane 2 and pane 3)
tmux select-pane -t 2
tmux split-window -h -p 50

# Split the window vertically into top (pane 0) and bottom (pane 4)
tmux select-pane -t 0
tmux split-window -v -p 50

# Split the window vertically into top (pane 1) and bottom (pane 5)
tmux select-pane -t 1
tmux split-window -v -p 50

# Split the window vertically into top (pane 2) and bottom (pane 6)
tmux select-pane -t 2
tmux split-window -v -p 50

# Split the window vertically into top (pane 3) and bottom (pane 7)
tmux select-pane -t 3
tmux split-window -v -p 50

# Split the window vertically into top (pane 8) and bottom (pane 8)
tmux select-pane -t 7
tmux split-window -v -p 50

tmux select-layout tiled

# Comandos
tmux send-keys -t 0 "python3 src/teleoperation/scripts/server_rtc.py --cert-file /home/aesir/AESIR/develop/cert.pem --key-file /home/aesir/AESIR/develop/key.pem --host 0.0.0.0 --port 8081" Enter
tmux send-keys -t 1 "ros2 run rosbridge_server rosbridge_websocket --ros-args --param ssl:=false --param port:=9090 --param address:=\"0.0.0.0\"" Enter
sleep 1
tmux send-keys -t 2 "ros2 launch hardware hardware.launch.py" Enter
sleep 1
tmux send-keys -t 3 "ros2 launch vision vision.launch.py" Enter
sleep 1

# Temporal
tmux send-keys -t 4 "cd ../.." Enter
tmux send-keys -t 4 "cd arm/workspace" Enter
tmux send-keys -t 4 "source install/setup.bash" Enter
tmux send-keys -t 4 "ros2 launch robot_moveit_config bringup.launch.py" Enter
sleep 1
# Temporal
tmux send-keys -t 6 "cd ../.." Enter
tmux send-keys -t 6 "cd mapping/workspace" Enter
tmux send-keys -t 6 "source install/setup.bash" Enter
tmux send-keys -t 6 "ros2 launch mapping test_sergio_no_lo_borres.py" Enter
sleep 1
tmux send-keys -t 7 "cd ../.." Enter
tmux send-keys -t 7 "cd mapping/workspace" Enter
tmux send-keys -t 7 "source install/setup.bash" Enter
tmux send-keys -t 7 "ros2 launch navigation navigation.launch.py" Enter
sleep 1
tmux send-keys -t 5 "htop" Enter
tmux send-keys -t 8 "jtop" Enter

tmux send-keys -t 9 "" Enter

# Optional key bindings to switch panes
tmux bind-key -n C-a select-pane -t :.+
tmux bind-key -n C-s select-pane -t :.-

# Attach to session
tmux -2 attach-session -t "$session_name"
