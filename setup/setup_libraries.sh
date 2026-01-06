#!/usr/bin/env bash
set -e

# Source global ROS_UBUNTU_VERSION if available
# Try to find the ros version helper relative to this script
if [ -n "${BASH_SOURCE[0]:-}" ]; then
	SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
else
	SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
fi

if [ -f "$SCRIPT_DIR/ros_ubuntu_version.sh" ]; then
	# when run from setup/ this path works
	# shellcheck source=/dev/null
	source "$SCRIPT_DIR/ros_ubuntu_version.sh"
elif [ -f "$(pwd)/setup/ros_ubuntu_version.sh" ]; then
	# fallback to repo-relative path (if script run from other cwd)
	# shellcheck source=/dev/null
	source "$(pwd)/setup/ros_ubuntu_version.sh"
fi

echo "setup_libraries.sh: ROS_UBUNTU_VERSION='${ROS_UBUNTU_VERSION:-}'"

# Helpers
command_exists() { command -v "$1" >/dev/null 2>&1; }

apt_install_if_missing() {
	local pkgs=("$@")
	local missing=()
	for p in "${pkgs[@]}"; do
		if ! dpkg -s "$p" >/dev/null 2>&1; then
			missing+=("$p")
		fi
	done
	if [ ${#missing[@]} -gt 0 ]; then
		echo "Instalando paquetes apt faltantes: ${missing[*]}"
		sudo apt update
		sudo apt install -y "${missing[@]}"
	else
		echo "Paquetes apt ya instalados: ${pkgs[*]}"
	fi
}

pip_install_if_missing() {
	local pkgs=("$@")
	local missing=()
	for p in "${pkgs[@]}"; do
		if ! python3 -m pip show "$p" >/dev/null 2>&1; then
			missing+=("$p")
		fi
	done
	if [ ${#missing[@]} -gt 0 ]; then
		echo "Instalando paquetes pip faltantes: ${missing[*]}"
		python3 -m pip install "${missing[@]}"
	else
		echo "Paquetes pip ya instalados: ${pkgs[*]}"
	fi
}

cd ..
# Crear y activar venv si no existe
if [ ! -d venv ]; then
	python3 -m venv venv
fi
source venv/bin/activate

# Pip packages básicos
pip_install_if_missing em lark

# Vision/python libs
pip_install_if_missing numpy aiohttp aiortc opencv-python av2 catkin-pkg

# vision: zbar y pyzbar
apt_install_if_missing libzbar0
pip_install_if_missing pyzbar ultralytics DateTime websockets

# Dependencias del sistema
apt_install_if_missing libwebsocketpp-dev libboost-all-dev libssl-dev tmux ros-${ROS_UBUNTU_VERSION}-cv-bridge libopencv-dev tmux

#Websockets