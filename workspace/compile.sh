#!/bin/bash
set -e

# Locate script directory and source global ROS_UBUNTU_VERSION helper
if [ -n "${BASH_SOURCE[0]:-}" ]; then
	SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
	SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

# Debug output
echo "compile.sh: ROS_UBUNTU_VERSION='${ROS_UBUNTU_VERSION:-}'"

try_source_ros() {
  local distro="$1"
  local setup="/opt/ros/${distro}/setup.bash"
  if [ -f "$setup" ]; then
    echo "Sourcing ROS: $distro ($setup)"
    # shellcheck source=/dev/null
    source "$setup"
    return 0
  fi
  return 1
}

if [ -n "${ROS_UBUNTU_VERSION:-}" ] && try_source_ros "${ROS_UBUNTU_VERSION}"; then
  :
else
  # Otherwise prefer Humble, then Jazzy
  try_source_ros "humble" || try_source_ros "jazzy" || {
    # Otherwise pick *any* installed ROS under /opt/ros
    if [ -d /opt/ros ]; then
      echo "Humble/Jazzy not found. Installed ROS distros under /opt/ros:"
      ls -1 /opt/ros || true

      for d in /opt/ros/*; do
        d="$(basename "$d")"
        if try_source_ros "$d"; then
          break
        fi
      done
    fi
  }
fi

colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON


