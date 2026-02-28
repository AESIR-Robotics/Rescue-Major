#!/bin/bash
set -e

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

if [ -f "/dev/ttyUSB0" ]; then
	sudo chmod 666 /dev/ttyUSB0
fi

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

session_name="test_move_$(date +%s)"

# Crear sesión con default-command
tmux new-session -d -s "$session_name" -n main "$ENV_CMD"
tmux set-option -t "$session_name" default-command "$ENV_CMD"

# Splits: 3 panes
# Layout:
# ┌──────────┬──────────┐
# │    0     │    1     │
# ├──────────┴──────────┤
# │    2     │    3     │
# └──────────┴──────────┘
tmux split-window -h
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v

# Comandos
# Pane 0: Joint teleop (top left)
tmux send-keys -t 0 "ros2 run hardware joint_mux.py" Enter

# Pane 1: Differential robot teleop (top right)
# Using teleop_twist_keyboard package with custom topic
tmux send-keys -t 1 "ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=hardware_node/cmd_vel" Enter

# Pane 2: Monitor joint states (bottom left)
tmux send-keys -t 2 "ros2 topic echo /hardware_node/joint_states" Enter

# Pane 3: Monitor velocity states (bottom right)
tmux send-keys -t 3 "ros2 topic echo /hardware_node/state_vel" Enter

sleep 3

# Optional key bindings to switch panes
tmux bind-key -n C-a select-pane -t :.+
tmux bind-key -n C-s select-pane -t :.-

# Attach to session
tmux -2 attach-session -t "$session_name"