#!/usr/bin/env bash
# Export a single variable that indicates the ROS Ubuntu/ROS distro to use.
# This file is intended to be sourced by other scripts so they can use
# $ROS_UBUNTU_VERSION (values like "humble" or "jazzy").

# Detect whether this file is being sourced (so we can 'return') or executed
if (return 0 2>/dev/null); then
    _ROS_SOURCED=1
else
    _ROS_SOURCED=0
fi

# If the user has already set ROS_UBUNTU_VERSION in the environment, keep it.
if [ -n "${ROS_UBUNTU_VERSION:-}" ]; then
    export ROS_UBUNTU_VERSION
    echo "ROS_UBUNTU_VERSION already set: $ROS_UBUNTU_VERSION"
    if [ "$_ROS_SOURCED" -eq 1 ]; then
        return 0
    else
        exit 0
    fi
fi

# If ROS_DISTRO is set (common in some environments), prefer it.
if [ -n "${ROS_DISTRO:-}" ]; then
    ROS_UBUNTU_VERSION="$ROS_DISTRO"
    export ROS_UBUNTU_VERSION
    echo "Detected ROS_DISTRO=$ROS_DISTRO, setting ROS_UBUNTU_VERSION=$ROS_UBUNTU_VERSION"
    if [ "$_ROS_SOURCED" -eq 1 ]; then
        return 0
    else
        exit 0
    fi
fi

# Detect installed ROS under /opt/ros
if [ -d "/opt/ros/humble" ]; then
    echo "Detected ROS installation: humble"
    ROS_UBUNTU_VERSION=humble
elif [ -d "/opt/ros/jazzy" ]; then
    echo "Detected ROS installation: jazzy"
    ROS_UBUNTU_VERSION=jazzy
else
    # Default fallback (safe choice); user may override by exporting ROS_UBUNTU_VERSION
    ROS_UBUNTU_VERSION=humble
    echo "Warning: could not detect /opt/ros/<distro>. Defaulting ROS_UBUNTU_VERSION=$ROS_UBUNTU_VERSION"
fi

# For some reason the export in this script does not work, you need to do it manually
echo "Final ROS_UBUNTU_VERSION=$ROS_UBUNTU_VERSION"
export ROS_UBUNTU_VERSION
