#!/bin/bash

cd ../workspace

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

echo "launch.bash: using ROS setup: $SOURCE_ROS_DIR"
echo "launch.bash: using venv:      $SOURCE_VENV_DIR"
echo "launch.bash: using overlay:   $SOURCE_LOCAL_DIR"

# Verify key files exist
[[ -f "$SOURCE_ROS_DIR" ]] || { echo "Missing: $SOURCE_ROS_DIR" >&2; exit 1; }
[[ -f "$SOURCE_LOCAL_DIR" ]] || { echo "Missing: $SOURCE_LOCAL_DIR (did you colcon build?)" >&2; exit 1; }
[[ -f "$SOURCE_VENV_DIR" ]] || { echo "Missing: $SOURCE_VENV_DIR (did you create the venv?)" >&2; exit 1; }

source "$SOURCE_VENV_DIR" 
source "$SOURCE_ROS_DIR" 
source "$SOURCE_LOCAL_DIR"  

ros2 run rosbridge_server rosbridge_websocket --ros-args --param ssl:=false --param port:=9090 --param address:=\"0.0.0.0\"