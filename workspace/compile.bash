#!/bin/bash
set -e

# Locate script directory and source global ROS_UBUNTU_VERSION helper
if [ -n "${BASH_SOURCE[0]:-}" ]; then
	SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
	SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

if [ -f "$SCRIPT_DIR/../setup/ros_ubuntu_version.sh" ]; then
	# shellcheck source=/dev/null
	source "$SCRIPT_DIR/../setup/ros_ubuntu_version.sh"
elif [ -f "./setup/ros_ubuntu_version.sh" ]; then
	# executed from repo root
	# shellcheck source=/dev/null
	source "./setup/ros_ubuntu_version.sh"
fi

# Debug output
echo "compile.bash: ROS_UBUNTU_VERSION='${ROS_UBUNTU_VERSION:-}'"

if [ -n "${ROS_UBUNTU_VERSION:-}" ] && [ -f "/opt/ros/${ROS_UBUNTU_VERSION}/setup.bash" ]; then
	source "/opt/ros/${ROS_UBUNTU_VERSION}/setup.bash"
else
	echo "ROS_UBUNTU_VERSION not set or /opt/ros/<distro>/setup.bash not found - attempting jazzy"
	source /opt/ros/jazzy/setup.bash
fi

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON


