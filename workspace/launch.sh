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

# Crear layout de 4 panes (2x2)
# Pane 0: RTC Server (top-left)
# Pane 1: Hardware (top-right)
# Pane 2: Rosbridge (bottom-left)
# Pane 3: Video (bottom-right)
tmux split-window -h -t "$session_name"    # Split horizontal: pane 0 | pane 1
tmux split-window -v -t "$session_name:0.0" # Split pane 0 vertical: pane 0 arriba, pane 2 abajo
tmux split-window -v -t "$session_name:0.1" # Split pane 1 vertical: pane 1 arriba, pane 3 abajo

# Esperar a que los panes carguen el ambiente
sleep 2

# Pane 0 (top-left): RTC Server
tmux send-keys -t "$session_name:0.0" "python3 src/teleoperation/scripts/server_rtc.py --cert-file ~/aesir/cert.pem --key-file ~/aesir/key.pem --host 0.0.0.0 --port 8081" Enter

# Pane 1 (top-right): Hardware
tmux send-keys -t "$session_name:0.1" "ros2 run hardware serial_sender.py" Enter

# Pane 2 (bottom-left): Rosbridge Server
tmux send-keys -t "$session_name:0.2" "ros2 run rosbridge_server rosbridge_websocket --ros-args --param ssl:=true --param certfile:=\"$HOME/aesir/cert.pem\" --param keyfile:=\"$HOME/aesir/key.pem\" --param port:=9090 --param address:=\"0.0.0.0\"" Enter

# Pane 3 (bottom-right): Video/Vision
tmux send-keys -t "$session_name:0.3" "ros2 launch vision vision.launch.py" Enter

sleep 1

# Optional key bindings to switch panes
tmux bind-key -n C-a select-pane -t :.+
tmux bind-key -n C-s select-pane -t :.-

# Attach to session
tmux -2 attach-session -t "$session_name"
